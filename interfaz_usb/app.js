"use strict";

/*
 * INTERFAZ LOCAL DE DEMOSTRACION (SIN CONTROL FISICO)
 * -------------------------------------------------
 * Se usan las ecuaciones de CD y CI de la V6 para previsualizar valores.
 * Esta pagina no abre USB/UART ni envia comandos al ESP32.
 * Las comprobaciones de seguridad reales corresponden al firmware.
 */
const MODEL = Object.freeze({
  L1:217, L2:105, L3:122, D2:33, D3:23,
  baseMin:-140.025, baseMax:105,
  shoulderMin:-89.25, shoulderMax:131.025,
  elbowMin:0, elbowMax:180,
  baseStepsPerDeg:4800/360,
  shoulderStepsPerDeg:4800/360,
  elbowPwmMin:500, elbowPwmMax:2500
});
const RAD=Math.PI/180;
const DEG=180/Math.PI;
const $=id=>document.getElementById(id);
const state={
  pose:{b:0,h:0,c:90},gripper:"Cerrada",homed:false,ready:false,stopped:false,moving:false,
  step:1,buzzer:true,pending:null,ciSolutions:[],selectedCiIndex:-1,timer:null
};
const fmt=(value,digits=2)=>Number(value).toFixed(digits);
const bound=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
const normRad=r=>Math.atan2(Math.sin(r),Math.cos(r));

/** Conversiones matematicas EXACTAS usadas por CD de la V6:
 * q1=-Base, q2=Hombro, q3=90-Codo, d=-(delta2+delta3).
 */
function direct(pose){
  const q1=-pose.b*RAD,q2=pose.h*RAD,q3=(90-pose.c)*RAD;
  const r=MODEL.L2*Math.sin(q2)+MODEL.L3*Math.sin(q2+q3);
  const d=-(MODEL.D2+MODEL.D3);
  return {
    x:Math.cos(q1)*r-Math.sin(q1)*d,
    y:Math.sin(q1)*r+Math.cos(q1)*d,
    z:MODEL.L1+MODEL.L2*Math.cos(q2)+MODEL.L3*Math.cos(q2+q3)
  };
}
function generalLimits(p){
  return Number.isFinite(p.b)&&Number.isFinite(p.h)&&Number.isFinite(p.c) &&
    p.b>=MODEL.baseMin-1e-6&&p.b<=MODEL.baseMax+1e-6 &&
    p.h>=MODEL.shoulderMin-1e-6&&p.h<=MODEL.shoulderMax+1e-6 &&
    p.c>=MODEL.elbowMin-1e-6&&p.c<=MODEL.elbowMax+1e-6;
}
function quantizePose(p){
  const baseSteps=bound(Math.round(p.b*MODEL.baseStepsPerDeg),-1867,1400);
  const shoulderSteps=bound(1320+Math.round(p.h*MODEL.shoulderStepsPerDeg),130,3067);
  const elbowUs=bound(Math.round(500+p.c*(2000/180)),MODEL.elbowPwmMin,MODEL.elbowPwmMax);
  return {
    b:baseSteps/MODEL.baseStepsPerDeg,
    h:(shoulderSteps-1320)/MODEL.shoulderStepsPerDeg,
    c:(elbowUs-500)*180/2000
  };
}
function inv(x,y,z){
  const d=-(MODEL.D2+MODEL.D3);
  const radio=Math.hypot(x,y), h=z-MODEL.L1;
  if(![x,y,z].every(Number.isFinite)||radio<Math.abs(d)-0.0015)return [];
  const rAbs=Math.sqrt(Math.max(0,radio*radio-d*d));
  const cosq3=(rAbs*rAbs+h*h-MODEL.L2*MODEL.L2-MODEL.L3*MODEL.L3)/(2*MODEL.L2*MODEL.L3);
  const length=Math.hypot(rAbs,h);
  if(!Number.isFinite(cosq3)||length>MODEL.L2+MODEL.L3+0.0015||length<Math.abs(MODEL.L2-MODEL.L3)-0.0015)return [];
  const q3Abs=Math.acos(bound(cosq3,-1,1));
  const baseDir=Math.atan2(y,x),solutions=[];
  for(let ir=0;ir<2;ir++){
    const r=ir===0?rAbs:-rAbs;
    const q1=normRad(baseDir-Math.atan2(d,r));
    for(let ie=0;ie<2;ie++){
      const q3=ie===0?q3Abs:-q3Abs;
      const q2=normRad(Math.atan2(r,h)-Math.atan2(MODEL.L3*Math.sin(q3),MODEL.L2+MODEL.L3*Math.cos(q3)));
      const pose={b:-q1*DEG,h:q2*DEG,c:90-q3*DEG};
      if(solutions.some(s=>Math.abs(s.pose.b-pose.b)<1e-4&&Math.abs(s.pose.h-pose.h)<1e-4&&Math.abs(s.pose.c-pose.c)<1e-4))continue;
      const p=direct(pose),err=Math.hypot(p.x-x,p.y-y,p.z-z);
      if(!Number.isFinite(err)||err>0.0015+1e-8)continue;
      const admissible=generalLimits(pose);
      const cost=Math.abs(pose.b-state.pose.b)+Math.abs(pose.h-state.pose.h)+Math.abs(pose.c-state.pose.c);
      solutions.push({pose,admissible,cost,err,index:solutions.length+1});
    }
  }
  return solutions;
}
function readNumber(id){
  const input=$(id);const raw=input.value.trim().replace(",",".");
  if(raw==="")return null;
  const val=Number(raw);
  return Number.isFinite(val)?val:null;
}
function printPose(p){return `B ${fmt(p.b,3)}° · H ${fmt(p.h,3)}° · C ${fmt(p.c,3)}°`;}
function log(message,type="info"){
  const list=$("log-list");
  const line=document.createElement("div");line.className=`log-entry ${type}`;
  const time=document.createElement("time");time.textContent=new Date().toLocaleTimeString("es-MX",{hour12:false});
  const dot=document.createElement("i");dot.setAttribute("aria-hidden","true");
  const label=document.createElement("span");label.className="message";label.textContent=message;
  line.append(time,dot,label);list.append(line);
  if(list.children.length>170)list.firstElementChild.remove();
  list.scrollTop=list.scrollHeight;
}
function update(){
  const p=direct(state.pose);
  $("tele-x").textContent=fmt(p.x,2);
  $("tele-y").textContent=fmt(p.y,2);
  $("tele-z").textContent=fmt(p.z,2);
  $("tele-b").textContent=fmt(state.pose.b,3);
  $("tele-h").textContent=fmt(state.pose.h,3);
  $("tele-c").textContent=fmt(state.pose.c,3);
  $("tele-grip").textContent=state.gripper;
  $("status-home").textContent=state.homed?"Completado":"Pendiente";
  $("status-ready").textContent=state.ready?"Sí":"No";
  $("status-motion").textContent=state.stopped?"PARO":state.moving?"Simulando":"Inactivo";
  $("status-buzzer").textContent=state.buzzer?"Activado":"Desactivado";
  $("status-motion").style.color=state.stopped?"#ff8a87":state.moving?"#ffc471":"#8de7bc";
  $("status-home").style.color=state.homed?"#8de7bc":"#ffbf80";
  $("status-ready").style.color=state.ready?"#8de7bc":"#ffbf80";
}
function modal(title,message,options){
  return new Promise(resolve=>{
    const dialog=$("dialog"),btns=$("modal-buttons");
    $("modal-title").textContent=title;
    $("modal-message").textContent=message;
    btns.replaceChildren();
    for(const opt of options){
      const button=document.createElement("button");button.textContent=opt.label;
      if(opt.style)button.className=opt.style;
      button.addEventListener("click",()=>{dialog.close();resolve(opt.value);},{once:true});
      btns.append(button);
    }
    const esc=()=>resolve(null);
    dialog.addEventListener("cancel",esc,{once:true});
    dialog.showModal();
  });
}
const notify=(title,message)=>modal(title,message,[{label:"Entendido",value:true,style:"primary"}]);
function stopAnimation(){if(state.timer){clearInterval(state.timer);state.timer=null;}state.moving=false;}
function canDemoMove(){
  if(window.robotUSB?.connected){notify("Consola USB conectada","Los botones de movimiento simulados están desactivados mientras se recibe información real. Para la V6 actual puedes responder mediante la consola serial inferior.");return false;}
  if(state.stopped){notify("PARO activo","En esta demostración debes realizar HOME nuevamente antes de mover.");log("Movimiento rechazado: PARO activo.","warning");return false;}
  if(!state.homed||!state.ready){notify("Primero HOME y READY","Para probar movimientos, completa la secuencia HOME y después READY.");log("Movimiento rechazado: faltan HOME/READY.","warning");return false;}
  if(state.moving){notify("Movimiento en curso","Espera a que termine la demostración.");return false;}
  return true;
}
function animatePose(target,description){
  stopAnimation();
  state.moving=true;update();
  const start={...state.pose},beg=performance.now(),duration=850;
  log(`${description}: iniciando simulación numérica (${printPose(target)}).`);
  state.timer=setInterval(()=>{
    const t=bound((performance.now()-beg)/duration,0,1);
    const smooth=t*t*(3-2*t);
    for(const k of ["b","h","c"])state.pose[k]=start[k]+(target[k]-start[k])*smooth;
    update();
    if(t>=1){
      stopAnimation();state.pose={...target};update();
      log(`Destino simulado alcanzado: ${printPose(state.pose)}.`,"good");
      log("Sin movimiento físico; todavía no hay comunicación con ESP32.","warning");
    }
  },30);
}
async function requestMove(pose,from){
  if(!canDemoMove())return;
  if(!generalLimits(pose)){
    log("Objetivo fuera de los límites articulares generales de la maqueta.","error");
    notify("Fuera de límites","La solución supera el rango general configurado. MOV2–MOV4 y el despeje físico deberán verificarse en el firmware.");
    return;
  }
  const p=direct(pose);
  const ok=await modal("¿Estás seguro?",`Origen: ${from}\nÁngulos deseados: ${printPose(pose)}\nXYZ calculado: (${fmt(p.x,3)}, ${fmt(p.y,3)}, ${fmt(p.z,3)}) mm\n\nSOLO DEMOSTRACIÓN: no se moverá el robot. No se verifican colisiones físicas ni restricciones MOV2–MOV4.`,[
    {label:"Cancelar",value:false},{label:"Simular movimiento",value:true,style:"primary"}
  ]);
  if(!ok){log("Movimiento cancelado.","warning");return;}
  if(state.stopped)return;
  const quantized=quantizePose(pose);
  animatePose(quantized,from);
}
async function demoHome(){
  if(window.robotUSB?.connected){await window.robotUSB.sendCommand("@HOME");return;}
  if(state.moving){notify("Movimiento activo","Termina o detén la simulación antes de HOME.");return;}
  log("HOME solicitado. ¿Hacia dónde se realiza la primera búsqueda: derecha o izquierda?");
  const direction=await modal("Dirección inicial de HOME","Selecciona hacia dónde se realizaría la primera búsqueda del HOME de la base.",[
    {label:"Cancelar",value:null},
    {label:"Izquierda",value:"izquierda",style:"primary"},
    {label:"Derecha",value:"derecha",style:"primary"}
  ]);
  if(!direction){log("HOME cancelado.","warning");return;}
  log(`Dirección de búsqueda elegida: ${direction}. ¿Estás seguro?`);
  const yes=await modal("¿Estás seguro?",`Primera búsqueda HOME de BASE hacia la ${direction}.\n\nEsta es únicamente una demostración visual; no habrá pulsos ni motores.`,[
    {label:"No, cancelar",value:false},{label:"Sí, continuar",value:true,style:"primary"}
  ]);
  if(!yes){log("HOME cancelado por el operador.","warning");return;}
  stopAnimation();state.homed=true;state.ready=false;state.stopped=false;
  state.pose={b:0,h:-99,c:0};state.pending=null;
  log("HOME de BASE completado (DEMO).","good");
  log("HOME de HOMBRO completado (DEMO).","good");
  update();
}
async function demoReady(){
  if(window.robotUSB?.connected){await window.robotUSB.requestReady();return;}
  if(!state.homed||state.stopped){notify("HOME requerido","Completa HOME antes de pasar a READY.");log("READY rechazado: falta HOME.","warning");return;}
  if(state.moving)return;
  const yes=await modal("Pasar a READY","El brazo se llevaría a su referencia operativa READY.\n¿Deseas simular esta transición?",[
    {label:"Cancelar",value:false},{label:"Sí, READY",value:true,style:"primary"}
  ]);
  if(!yes){log("READY cancelado.","warning");return;}
  // Solo representación: el HOME mecánico de hombro equivale a H=-99°;
  // READY equivale a H=0° y codo nominal=90°.
  state.pose={b:0,h:0,c:90};state.ready=true;state.pending=null;
  log("READY alcanzado (DEMO).","good");update();
}
function demoStop(){
  if(window.robotUSB?.connected){void window.robotUSB.stop();return;}
  stopAnimation();state.stopped=true;state.ready=false;state.homed=false;state.pending=null;
  update();log("PARO activado (DEMO). HOME obligatorio antes de continuar.","error");
}
function calculateDirect(){
  const b=readNumber("cd-b"),h=readNumber("cd-h"),c=readNumber("cd-c");
  if([b,h,c].some(v=>v===null)){
    notify("Dato incorrecto","Escribe un número válido para cada articulación.");return;
  }
  const pose={b,h,c};
  if(!generalLimits(pose)){
    $("cd-result").innerHTML="<span>Resultado:</span><strong>Ángulos fuera de los límites generales.</strong>";
    log(`CD no habilitada para movimiento: ${printPose(pose)} fuera de topes.`,"warning");
    state.pending=null;return;
  }
  const p=direct(pose);
  $("cd-result").replaceChildren();
  const sub=document.createElement("span");sub.textContent="XYZ calculado (mm):";
  const text=document.createElement("strong");text.textContent=`X ${fmt(p.x,3)}   ·   Y ${fmt(p.y,3)}   ·   Z ${fmt(p.z,3)}`;
  $("cd-result").append(sub,text);
  state.pending={type:"cd",pose};
  log(`CD calculada: ${printPose(pose)} → XYZ (${fmt(p.x,3)}, ${fmt(p.y,3)}, ${fmt(p.z,3)}) mm.`,"good");
  log("Si lo deseas, pulsa MOVER en CD para simular exactamente esos ángulos.");
}
function chooseSolution(index){
  const sol=state.ciSolutions[index];
  if(!sol||!sol.admissible||(window.robotUSB?.connected && sol.routeApproved!==true))return;
  state.selectedCiIndex=index;state.pending={type:"ci",pose:sol.pose};
  renderSolutions();log(`CI: seleccionada solución ${sol.index} (${printPose(sol.pose)}).`);
}
function renderSolutions(){
  const list=$("solutions");list.replaceChildren();
  if(!state.ciSolutions.length){
    const p=document.createElement("p");p.className="empty-state";p.textContent="No se encontraron soluciones matemáticas.";list.append(p);return;
  }
  const admissible=state.ciSolutions.filter(s=>s.admissible && (!window.robotUSB?.connected || s.routeApproved===true));
  const best=admissible.length?admissible.reduce((a,b)=>a.cost<b.cost?a:b):null;
  state.ciSolutions.forEach((s,i)=>{
    const el=document.createElement("label");
    const habilitada=s.admissible && (!window.robotUSB?.connected || s.routeApproved===true);
    el.className=`solution-item ${habilitada?"valid":"invalid"} ${state.selectedCiIndex===i?"selected":""}`;
    const input=document.createElement("input");input.type="radio";input.name="ci-solution";input.value=String(i);input.checked=state.selectedCiIndex===i;input.disabled=!habilitada;
    input.addEventListener("change",()=>chooseSolution(i));
    const text=document.createElement("div"),top=document.createElement("div"),strong=document.createElement("strong"),badge=document.createElement("em"),desc=document.createElement("div");
    top.className="sol-top";strong.textContent=`Solución ${s.index}`;
    badge.textContent=!s.admissible?"FUERA DE TOPES":
      window.robotUSB?.connected && s.routeApproved===false?"RUTA RECHAZADA":
      window.robotUSB?.connected && s.routeApproved!==true?"REVISANDO RUTA":
      s===best?"RECOMENDADA":"ADMISIBLE";
    if(!s.admissible)badge.className="bad";
    top.append(strong,badge);desc.className="sol-values";desc.textContent=printPose(s.pose)+
      (window.robotUSB?.connected && s.routeReason?` | ${s.routeReason}`:"");
    text.append(top,desc);el.append(input,text);list.append(el);
  });
}
function calculateInverse(){
  const x=readNumber("ci-x"),y=readNumber("ci-y"),z=readNumber("ci-z");
  if([x,y,z].some(v=>v===null)){
    notify("Dato incorrecto","Escribe valores numéricos válidos para X, Y y Z.");return;
  }
  state.ciSolutions=inv(x,y,z);
  state.ciSolutions.forEach(s=>{s.routeApproved=null;s.routeReason="";});
  const live=Boolean(window.robotUSB?.connected);
  const acceptable=state.ciSolutions.map((s,i)=>({...s,i})).filter(s=>s.admissible);
  state.selectedCiIndex=!live && acceptable.length?acceptable.reduce((a,b)=>a.cost<b.cost?a:b).i:-1;
  $("solutions-count").textContent=`${acceptable.length} admisibles / ${state.ciSolutions.length} calculadas`;
  state.pending=state.selectedCiIndex>=0?{type:"ci",pose:state.ciSolutions[state.selectedCiIndex].pose}:null;
  renderSolutions();
  if(live && window.robotUSB?.protocolReady) void window.robotUSB.sendCommand(`@PLAN CI ${x} ${y} ${z}`);
  log(`CI (${fmt(x,3)}, ${fmt(y,3)}, ${fmt(z,3)}) mm: ${state.ciSolutions.length} soluciones matemáticas, ${acceptable.length} dentro de topes generales.`,acceptable.length?"good":"warning");
  if(!acceptable.length) log("No hay solución dentro de topes articulares.","warning");
  else if(!live) log(`CI recomendada por cercanía: solución ${state.ciSolutions[state.selectedCiIndex].index}.`);
  else log("Comprobando las rutas de CI en el ESP32 (MOV2–MOV4)…");
}
async function cdMove(){
  if(!state.pending || state.pending.type!=="cd"){notify("No hay destino de CD","Primero utiliza CALCULAR XYZ para generar el destino angular de cinemática directa.");return;}
  if(window.robotUSB?.connected){
    const p=state.pending.pose;
    await window.robotUSB.sendCommand(`@CD ${p.b} ${p.h} ${p.c}`);
    return;
  }
  await requestMove(state.pending.pose,"CD por ángulos");
}
async function ciMove(){
  if(state.selectedCiIndex<0){notify("Selecciona una solución","Calcula primero la CI y selecciona una solución admisible.");return;}
  if(window.robotUSB?.connected){
    const x=readNumber("ci-x"),y=readNumber("ci-y"),z=readNumber("ci-z");
    const index=state.ciSolutions[state.selectedCiIndex].index;
    await window.robotUSB.sendCommand(`@CI ${x} ${y} ${z} ${index}`);
    return;
  }
  await requestMove(state.ciSolutions[state.selectedCiIndex].pose,"CI por coordenadas");
}
function wheelClick(button){
  if(window.robotUSB?.connected){
    if(window.robotUSB?.recoveryMode){
      if(button.dataset.joint==="b")
        void window.robotUSB.sendRecoveryJog(Number(button.dataset.direction));
      return;
    }
    if(window.robotUSB?.adjustmentJoint){
      if(button.dataset.joint===window.robotUSB.adjustmentJoint)
        void window.robotUSB.sendAdjustmentJog(button.dataset.joint,Number(button.dataset.direction));
      return;
    }
    if(button.dataset.grip) void window.robotUSB.sendCommand(
      button.dataset.grip==="Abierta"?"@GRIP OPEN":"@GRIP CLOSE");
    else {
      const joint={b:"B",h:"H",c:"C"}[button.dataset.joint];
      const degrees=Number(button.dataset.direction)*state.step;
      void window.robotUSB.sendCommand(`@JOG ${joint} ${degrees}`);
    }
    return;
  }
  if(!canDemoMove())return;
  const grip=button.dataset.grip;
  if(grip){
    state.gripper=grip;
    update();log(`Pinza: consigna de ${grip.toLowerCase()} (solo demostración).`,"good");return;
  }
  const joint=button.dataset.joint,direction=Number(button.dataset.direction);
  const next={...state.pose};
  next[joint]=Math.round((next[joint]+direction*state.step)*1000000)/1000000;
  if(!generalLimits(next)){
    log(`Límite general: no se permite ${joint.toUpperCase()} = ${fmt(next[joint],3)}°.`,"warning");
    notify("Límite general","El incremento solicitado supera el rango configurado para esta articulación. La ESP32 deberá verificar también todas las restricciones restantes.");return;
  }
  state.pose=quantizePose(next);state.pending=null;update();
  const labels={b:"BASE",h:"HOMBRO",c:"CODO"};
  log(`Movimiento manual ${labels[joint]} ${direction>0?"+":"−"}${state.step}° (simulado). Objetivo estimado: ${fmt(state.pose[joint],3)}°.`,"good");
}
function init(){
  $("btn-home").addEventListener("click",demoHome);
  $("btn-ready").addEventListener("click",demoReady);
  $("btn-stop").addEventListener("click",demoStop);
  $("btn-cd").addEventListener("click",calculateDirect);
  $("btn-cd-move").addEventListener("click",cdMove);
  $("btn-ci").addEventListener("click",calculateInverse);
  $("btn-ci-move").addEventListener("click",ciMove);
  $("btn-clear-log").addEventListener("click",()=>{$("log-list").replaceChildren();log("Registro de demostración limpiado.");});
  document.querySelectorAll(".wheel-sector").forEach(b=>b.addEventListener("click",()=>wheelClick(b)));
  document.querySelectorAll(".step-btn").forEach(b=>b.addEventListener("click",()=>{
    state.step=Number(b.dataset.step);
    document.querySelectorAll(".step-btn").forEach(other=>{other.classList.toggle("active",other===b);other.setAttribute("aria-pressed",String(other===b));});
    log(`Incremento manual seleccionado: ${state.step}°.`);
  }));

  ["cd-b","cd-h","cd-c"].forEach(id=>$(id).addEventListener("input",()=>{
    if(state.pending?.type==="cd")state.pending=null;
    $("cd-result").innerHTML="<span>Resultado:</span><strong>Recalcula para actualizar XYZ</strong>";
  }));
  ["ci-x","ci-y","ci-z"].forEach(id=>$(id).addEventListener("input",()=>{
    if(state.pending?.type==="ci")state.pending=null;
    state.selectedCiIndex=-1;state.ciSolutions=[];$("solutions-count").textContent="Sin calcular";renderSolutions();
  }));
  update();
  log("Modelo CD/CI verificado: desplazamiento lateral −56 mm · versión XY-NEG56-V2.","good");
  log("Interfaz preparada. Conecta el ESP32 por USB para consultar su estado.");
}
document.addEventListener("DOMContentLoaded",init);

// Interfaz interna para el módulo Web Serial, sin compartir referencias de motores.
window.robotDemo = {
  mathRevision:"XY-NEG56-V2",
  setRouteResult(index,approved,reason){
    const s=state.ciSolutions[index-1];
    if(!s)return;
    s.routeApproved=approved;
    s.routeReason=reason||"";
    renderSolutions();
  },
  routePlanFinished(){
    const valid=state.ciSolutions.map((s,i)=>({...s,i})).filter(s=>s.admissible&&s.routeApproved===true);
    state.selectedCiIndex=valid.length?valid.reduce((a,b)=>a.cost<b.cost?a:b).i:-1;
    state.pending=state.selectedCiIndex>=0?{type:"ci",pose:state.ciSolutions[state.selectedCiIndex].pose}:null;
    $("solutions-count").textContent=`${valid.length} rutas válidas / ${state.ciSolutions.length} soluciones`;
    renderSolutions();
  },
  setLiveMode(live,protocolReady=false,canMove=false,canRecoveryJog=false,adjustmentJoint=null){
    if(live)stopAnimation();
    // HOME/READY se permiten al reconocer el protocolo; para mover se exige
    // HOME valido + READY, ademas de que el firmware revalida todo.
    ["btn-home","btn-ready"].forEach(id=>$(id).disabled=live && !protocolReady);
    ["btn-cd-move","btn-ci-move"].forEach(id=>$(id).disabled=live && (!protocolReady || !canMove));
    $("btn-stop").disabled=false;
    document.querySelectorAll(".wheel-sector").forEach(el=>{
      const baseRecovery=canRecoveryJog && el.dataset.joint==="b";
      const serviceRecovery=Boolean(adjustmentJoint && el.dataset.joint===adjustmentJoint);
      el.disabled=live && (!protocolReady || (!canMove && !baseRecovery && !serviceRecovery));
    });
    const reason = !protocolReady ? "Esperando protocolo USB V2" :
      canRecoveryJog ? "Búsqueda manual HOME BASE: máximo 30° por pulsación" :
      adjustmentJoint ? `Ajuste manual de ${adjustmentJoint==="h"?"HOMBRO":"CODO"}: 10° por pulsación` :
      !canMove ? "Completa HOME y READY antes de mover" : "Controles físicos habilitados";
    ["btn-cd-move","btn-ci-move",...Array.from(document.querySelectorAll(".wheel-sector"))].forEach(b=>{
      const el=typeof b==="string"?$(b):b;
      if(el)el.title=live?reason:"Controles de demostración";
    });
    if(!live)update();
  }
};
