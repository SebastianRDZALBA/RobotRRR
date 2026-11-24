"use strict";

/*
 * CONEXIÓN USB/UART CON LA V6 EXISTENTE — ETAPA 1
 * - LEE datos reales por USB y muestra la consola original del ESP32.
 * - PERMITE enviar únicamente lo que el operador escriba, sin macros.
 * - NO activa HOME, motores, CD/CI, ni movimientos automáticamente.
 * - La consola de la V6 original sigue siendo la autoridad de confirmaciones.
 * - La telemetría es estimada por pasos/PWM, no medida con encoders.
 * - Al usar USB, los controles SIMULADOS se bloquean intencionalmente.
 */
(() => {
  const $ = id => document.getElementById(id);
  const BAUD = 115200;
  const encoder = new TextEncoder();
  const decoder = new TextDecoder("utf-8");
  let port = null;
  let reader = null;
  let readTask = null;
  let connected = false;
  let disconnecting = false;
  let sending = false;
  let protocolReady = false;
  let recoveryMode = false;
  let recoveryCanCommand = false;
  // Sólo se habilita la articulación indicada expresamente por el firmware.
  let adjustmentJoint = null; // null, "h" (hombro) o "c" (codo)
  let adjustmentCanCommand = false;
  let elbowAwaitingDirection = 0;
  let readyPrompt = false;
  let pendingQuestion = false;
  let lastPromptKey = "";
  let safeExitLogged=false;
  let heartbeatTimer = null;
  let heartbeatActive = false;
  let writeTail = Promise.resolve();
  let buffer = "";
  let flushTimer = null;
  const data = {
    baseHome: null,
    shoulderHome: null,
    prepared: null,
    shoulderReady: null,
    moving: false,
  };

  const status = (id, text, kind) => {
    const element = $(id);
    if(!element) return;
    element.textContent = text;
    element.style.color = kind === "good" ? "#8de7bc" : kind === "bad" ? "#ffbf80" : "#a7c2d5";
  };
  const value = (id, amount, decimals) => {
    const element = $(id);
    if(element && Number.isFinite(amount)) element.textContent = amount.toFixed(decimals);
  };
  const serialLog = (message, type="info") => {
    if(typeof log === "function") log("ESP32  " + message, type);
  };

  // Refleja por qué un control todavía no puede mover el hardware.
  function refreshControls(){
    if(!connected){window.robotDemo?.setLiveMode(false,false,false);return;}
    const canMove=protocolReady && data.baseHome===true &&
      data.shoulderHome===true && data.prepared===true && !data.moving &&
      !recoveryMode && !adjustmentJoint;
    window.robotDemo?.setLiveMode(true,protocolReady,canMove,
                                  recoveryMode && recoveryCanCommand,
                                  adjustmentCanCommand ? adjustmentJoint : null);
    const message=recoveryMode
      ? (recoveryCanCommand
         ? "HOME manual BASE: activa Base + / Base −. Tramos máximos de 30°."
         : "HOME manual BASE: esperando la siguiente orden del ESP32.")
      : adjustmentJoint
      ? (adjustmentCanCommand
         ? `HOME manual ${adjustmentJoint==="h"?"HOMBRO":"CODO"}: sectores + / − activos (10° por pulsación).`
         : `HOME manual ${adjustmentJoint==="h"?"HOMBRO":"CODO"}: esperando al ESP32.`)
      : !protocolReady
      ? "Esperando reconocimiento del ESP32. Si no aparece, consulta p cuando el robot esté en el menú."
      : !data.baseHome || !data.shoulderHome || !data.prepared
      ? "USB reconocido. Completa HOME y READY para habilitar los movimientos."
      : data.moving ? "Movimiento en curso; disponible PARO." : "Controles físicos habilitados";
    $("usb-note").textContent=message;
  }
  function renderMode(isLive) {
    $("usb-connect").disabled = isLive;
    $("usb-disconnect").disabled = !isLive;
    $("serial-terminal").hidden = !isLive;
    $("usb-note").textContent = isLive
      ? (protocolReady ? "Controles USB habilitados" :
         "Conexión real. Esperando protocolo USB del ESP32.")
      : "Sin conexión: controles en modo demostración.";
    refreshControls();
    if(isLive) {
      status("status-usb", "Conectada", "good");
      status("status-home", "Sin consultar");
      status("status-ready", "Sin consultar");
      status("status-motion", "Sin consultar");
      status("status-buzzer", "Sin consultar");
      status("status-actuators", "Sin consultar");
      status("status-pwm", "Sin consultar");
      ["tele-x", "tele-y", "tele-z", "tele-b", "tele-h", "tele-c"].forEach(id => $(id).textContent="—");
      $("tele-grip").textContent="Desconocida";
    } else {
      status("status-usb", "No conectada", "bad");
    }
  }

  async function writeRaw(message) {
    if(!connected || !port?.writable) return false;
    // Un solo escritor a la vez. El heartbeat nunca se mezcla con comandos.
    const run = async () => {
      const writer=port.writable.getWriter();
      try {await writer.write(encoder.encode(message));return true;}
      finally {writer.releaseLock();}
    };
    const result=writeTail.then(run,run);
    writeTail=result.catch(()=>{});
    try {return await result;}
    catch(e) {serialLog("Fallo de escritura USB: "+e.message,"warning");return false;}
  }
  function stopHeartbeat(){
    heartbeatActive=false;
    if(heartbeatTimer){clearInterval(heartbeatTimer);heartbeatTimer=null;}
  }
  function startHeartbeat(){
    stopHeartbeat();
    heartbeatActive=true;
    void writeRaw("~");
    heartbeatTimer=setInterval(()=>{
      if(!connected){stopHeartbeat();return;}
      void writeRaw("~");
    },180);
  }
  // Conservar TODAS las preguntas que requieren respuesta; solo filtrar
  // el detalle técnico repetido. Nunca fabricar respuestas del operador.
  function showQuestion(key, message, type="info") {
    pendingQuestion=true;
    if(lastPromptKey!==key) serialLog(message,type);
    lastPromptKey=key;
    return true;
  }
  function recoverySet(active,canCommand=false){
    const changed = recoveryMode!==active;
    recoveryMode=active;
    recoveryCanCommand=active && canCommand;
    refreshControls();
    if(changed && active) serialLog("Búsqueda manual HOME: usa Base + / Base −. Cada pulsación busca hasta 30° y termina al detectar el interruptor.","warning");
    if(changed && !active) serialLog("Búsqueda manual HOME finalizada.");
  }
  function setAdjustment(joint,ready=false){
    const changed=adjustmentJoint!==joint;
    adjustmentJoint=joint;
    adjustmentCanCommand=Boolean(joint && ready);
    elbowAwaitingDirection=0;
    refreshControls();
    if(changed && joint){
      serialLog(joint==="h"
        ? "Ajuste manual de HOMBRO: usa Hombro + / − (10°). Al quedar seguro, escribe s."
        : "Ajuste manual de CODO: usa Codo + / − (10°). Al terminar, escribe s para salir del ajuste; después confirma el despeje cuando el ESP32 lo pida.","warning");
    }
    if(changed && !joint)serialLog("Ajuste manual finalizado. Continúa las preguntas de HOME.");
  }
  async function sendAdjustmentJog(joint,direction){
    if(!connected || !protocolReady || !adjustmentCanCommand ||
       joint!==adjustmentJoint || heartbeatActive || ![1,-1].includes(direction))return false;
    adjustmentCanCommand=false;
    refreshControls();
    if(joint==="h"){
      const ok=await writeRaw(direction>0?"r":"l");
      if(ok)serialLog(`Ajuste HOMBRO solicitado: ${direction>0?"+":"−"}10° (referencia relativa, sin HOME todavía).`);
      else {adjustmentCanCommand=true;refreshControls();}
      return ok;
    }
    // Codo PRE-HOME: el menú original exige seleccionar 4 (=10°)
    // y después confirmar r/l. Esperamos un marcador del firmware antes
    // de enviar la dirección para no adelantarnos ni perder la secuencia.
    elbowAwaitingDirection=direction;
    const ok=await writeRaw("4");
    if(!ok){elbowAwaitingDirection=0;adjustmentCanCommand=true;refreshControls();}
    else serialLog(`Ajuste CODO solicitado: ${direction>0?"+":"−"}10° (pendiente de respuesta del ESP32).`);
    return ok;
  }
  function checkPrompt(line){
    const l=line.toLowerCase();
    if(/primer home.*paso 1:.*hombro.*segura/.test(l))
      return showQuestion("safe-shoulder","HOME: ¿Hombro seguro para su movimiento y el del codo? s=Sí · n=Ajustar · q=Cancelar","warning");
    if(/primer home.*paso 2:.*codo.*home/.test(l))
      return showQuestion("safe-elbow","HOME: ¿El codo puede ir a su referencia HOME? s=Sí · n=Ajuste manual · q=Cancelar","warning");
    if(/tras home base:.*codo.*bajar hombro/.test(l))
      return showQuestion("safe-lower-shoulder","HOME HOMBRO: ¿El codo permite bajar el hombro al microswitch? s=Sí · n=Ajustar","warning");
    if(/iniciar (?:retorno del hombro a home mecanico|home del hombro ahora)\?/.test(l))
      return showQuestion("confirm-shoulder-home","¿Iniciar búsqueda HOME del hombro? Verifica que el recorrido esté despejado. s=Sí · n=No","warning");
    // La pregunta r=DERECHA/l=IZQUIERDA también existe en el ajuste
    // manual del CODO: no confundirla con el HOME de la BASE.
    if(/el[ií]ge direccion para la busqueda inicial|home base:.*direccion inicial/.test(l))
      return showQuestion("direction","HOME BASE: elige dirección inicial. l=Izquierda · r=Derecha · Esc=Paro controlado");
    if(/buscar home hacia (derecha|izquierda).*\?/.test(l))
      return showQuestion("confirm-base-direction",`¿Confirmar búsqueda HOME de BASE hacia ${l.includes('derecha')?'derecha':'izquierda'}? s=Sí · n=No`,"warning");
    if(/hombro y codo: recorrido.*despejado\?/.test(l))
      return showQuestion("confirm-return-home","¿Recorrido de hombro y codo despejado para volver a HOME? s=Sí · n=No","warning");
    if(/retorno a home: codo permite/.test(l))
      return showQuestion("safe-return-home","¿El codo permite regresar el hombro a HOME? s=Sí · n=Ajustar","warning");
    if(/salir: hombro realmente sobre tope/.test(l))
      return showQuestion("confirm-exit","¿Confirmas SALIR y deshabilitar motores/PWM? Asegura el brazo antes de cortar torque. s=Sí · n=No","warning");
    if(/la pinza puede cerrarse/.test(l))
      return showQuestion("confirm-gripper","¿La pinza puede cerrarse antes del apagado? s=Sí · n=No","warning");
    if(/s=ir a preparado/i.test(l)) {
      readyPrompt=true;
      return showQuestion("after-home","HOME completado. ¿Qué deseas hacer? s=READY · a=SALIR (NEMA y PWM OFF)");
    }
    if(/no se encontro el switch/.test(l)) {
      serialLog("HOME BASE: no se encontró el interruptor; se requiere recuperación.","warning");
      return true;
    }
    if(/o: buscar en sentido contrario|j: busqueda manual en tramos/i.test(l))
      return showQuestion("home-recovery","Recuperación HOME: o=Dirección contraria · j=Búsqueda manual · q=Cancelar","warning");
    if(/estas seguro\?|s=si\s*\|\s*n=no|responde s, n, (?:!|esc) o x/i.test(l)){
      // Evitar duplicar una pregunta específica seguida del texto genérico.
      if(lastPromptKey.startsWith("confirm-") || lastPromptKey.startsWith("safe-")) return true;
      return showQuestion("confirm-generic","¿Confirmas la operación? s=Sí · n=No · Esc=Paro controlado","warning");
    }
    // Se mantiene la autorización, pero se evita imprimir la descripción larga.
    if(/autorizar trayectoria usb supervisada/i.test(l)) return true;
    // Preguntas de seguridad breves que no son pasos de motor.
    if(/^(?:regresar a preparado|base con codo fuera de ready|codo listo y despejado|intentar primer home supervisado)/i.test(l) && l.includes('?'))
      return showQuestion("confirm-other",line.replace(/=+/g,"").trim()+" (s=Sí · n=No)","warning");
    return false;
  }
  async function sendCommand(command){
    if(!connected || !protocolReady){
      serialLog("No se permite control físico: falta detectar UI:PROTOCOLO_V2 con la nueva V6.","warning");
      return false;
    }
    if(heartbeatActive){serialLog("Movimiento en curso. Solo PARO está disponible.","warning");return false;}
    if(pendingQuestion){
      serialLog("Responde primero a la pregunta pendiente del ESP32.","warning");
      return false;
    }
    if(!/^@[A-Z][A-Z0-9 .+\-]*$/.test(command)){
      serialLog("Orden UI no válida.","warning");return false;
    }
    const ok=await writeRaw(command+"\n");
    if(ok) {
      const map={"@HOME":"HOME solicitado. Sigue las preguntas de seguridad.","@READY":"READY solicitado."};
      serialLog(map[command] ?? (command.startsWith("@JOG") ? "Movimiento manual solicitado: "+command.slice(5) : "Solicitud enviada: "+command.slice(1)));
    }
    return ok;
  }
  async function sendKey(key){
    if(!connected || !/^[a-z0-9!]$/.test(key) || heartbeatActive)return false;
    pendingQuestion=false;
    readyPrompt=false;
    lastPromptKey="";
    return writeRaw(key);
  }
  // Solo la rueda BASE está habilitada durante recuperación HOME. Una
  // orden r/l por pantalla, y no se libera otra hasta que vuelva el prompt.
  async function sendRecoveryJog(direction){
    if(!connected || !protocolReady || !recoveryMode || !recoveryCanCommand ||
       heartbeatActive || (direction!==1 && direction!==-1))return false;
    recoveryCanCommand=false;
    refreshControls();
    const ok=await writeRaw(direction>0 ? "r" : "l");
    if(ok) serialLog(`HOME manual: buscando hacia ${direction>0?"derecha":"izquierda"} (máx. 30° por tramo).`);
    else {recoveryCanCommand=true;refreshControls();}
    return ok;
  }
  async function requestReady(){
    if(readyPrompt) return sendKey("s");
    return sendCommand("@READY");
  }
  async function stop(){
    if(!connected)return false;
    pendingQuestion=false;
    if(adjustmentJoint)setAdjustment(null);
    if(recoveryMode)recoverySet(false);
    // 0x03 siempre significa PARO CONTROLADO, incluso dentro de preguntas.
    const sent=await writeRaw("\x03");
    if(sent)serialLog("PARO CONTROLADO enviado (tecla Esc o botón PARO).", "warning");
    return sent;
  }
  function updateHome() {
    if(data.baseHome === null || data.shoulderHome === null) return;
    status("status-home", data.baseHome && data.shoulderHome ? "Completado" : "Pendiente",
      data.baseHome && data.shoulderHome ? "good" : "bad");
  }

  function parseLine(line) {
    let m;
    m = line.match(/^BASE:\s*HOME=(SI|NO)\b.*?PREPARADO BASE=(SI|NO)/i);
    if(m) {
      data.baseHome = m[1].toUpperCase()==="SI";
      data.prepared = m[2].toUpperCase()==="SI";
      updateHome();
      if(data.prepared) status("status-ready","Sí","good");
      else status("status-ready","No","bad");
    }
    m = line.match(/^HOMBRO:\s*HOME=(SI|NO)\s*\|\s*EN READY=(SI|NO)/i);
    if(m) {
      data.shoulderHome = m[1].toUpperCase()==="SI";
      data.shoulderReady = m[2].toUpperCase()==="SI";
      updateHome();
    }
    m = line.match(/^BASE:\s*([+-]?\d+(?:\.\d+)?)\s+grados OBJETIVO NOMINAL;/i);
    if(m) value("tele-b",Number(m[1]),3);
    m = line.match(/^HOMBRO:\s*([+-]?\d+(?:\.\d+)?)\s+grados OPERATIVOS/i);
    if(m) value("tele-h",Number(m[1]),3);
    m = line.match(/^CODO:\s*([+-]?\d+(?:\.\d+)?)\s+grados NOMINALES/i);
    if(m) value("tele-c",Number(m[1]),3);
    if(/^CODO:\s*PWM OFF/i.test(line)) $("tele-c").textContent = "—";
    if(/^BASE: SIN HOME/i.test(line)) $("tele-b").textContent = "—";
    if(/^HOMBRO: SIN HOME/i.test(line)) $("tele-h").textContent = "—";
    m = line.match(/P_x=\s*([+-]?\d+(?:\.\d+)?)\s*mm\s*\|\s*P_y=\s*([+-]?\d+(?:\.\d+)?)\s*mm\s*\|\s*P_z=\s*([+-]?\d+(?:\.\d+)?)\s*mm/i);
    if(m) {
      value("tele-x", Number(m[1]),2);
      value("tele-y", Number(m[2]),2);
      value("tele-z", Number(m[3]),2);
    }
    if(/P_x, P_y, P_z:\s*NO CALCULABLES/i.test(line))
      ["tele-x","tele-y","tele-z"].forEach(id => $(id).textContent="—");
    m = line.match(/^PINZA:\s*([+-]?\d+(?:\.\d+)?)\s+grados NOMINALES de apertura/i);
    if(m) $("tele-grip").textContent = Number(m[1]) >= 40 ? "Abierta" : "Cerrada";
    if(/^PINZA:\s*PWM OFF/i.test(line)) $("tele-grip").textContent="Desconocida";
    if(/MODO PREPARADO ACTIVADO|READY alcanzado/i.test(line)) {
      data.prepared = true;
      status("status-ready","Sí","good");
    }
    if(/HOME TOTAL COMPLETADO|HOME TOTAL TERMINADO|HOME DE RECUPERACION COMPLETADO/i.test(line)) {
      data.baseHome=true;data.shoulderHome=true;data.prepared=false;
      updateHome();status("status-ready","No","bad");
    }
    if(/\[PARO (?:[!sx]|CONTROLADO)\]|PARO ENCLAVADO|UI:SALIDA_SEGURA|SALIDA SEGURA LOGICA/i.test(line)) {
      data.baseHome=false;data.shoulderHome=false;data.prepared=false;
      updateHome();status("status-ready","No","bad");
      status("status-motion","PARO","bad");
      ["tele-x","tele-y","tele-z","tele-b","tele-h","tele-c"].forEach(id=>$(id).textContent="—");
      $("tele-grip").textContent="Desconocida";
    }
    if(/UI:SALIDA_SEGURA|SALIDA SEGURA LOGICA|\[PARO x\]/i.test(line)) {
      if(/UI:SALIDA_SEGURA|SALIDA SEGURA LOGICA/i.test(line))
        status("status-motion","Cerrado","bad");
      status("status-actuators","NEMA OFF","bad");
      status("status-pwm","PWM OFF","bad");
    } else if(/\[PARO (?:!|CONTROLADO)\]/i.test(line)) {
      status("status-actuators","Estado conservado");
      status("status-pwm","Estado conservado");
    }
    if(/EJECUTANDO\s+\d+\s+ticks/i.test(line)) status("status-motion","Ejecutando","bad");
    if(/DESTINO alcanzado SEGUN PASOS\/PWM|TRAYECTORIA CANCELADA|CICLO PINZA FINALIZADO/i.test(line))
      status("status-motion","En espera","good");
    if(/^UI:ARM/.test(line))data.moving=true;
    if(/^UI:IDLE|^UI:MOVIMIENTO_INTERRUMPIDO|^UI:SALIDA_SEGURA/.test(line))data.moving=false;
    refreshControls();
  }

  function receiveLine(line) {
    // Todo el texto original sigue procesandose para telemetria.
    line = line.replace(/\x1b\[[0-9;]*[A-Za-z]/g,"").replace(/[\x00-\x08\x0b-\x1f\x7f]/g,"").trim();
    if(!line)return;
    if(/UI:PROTOCOLO_V1/.test(line)) {
      serialLog("Firmware anterior (V1). Actualiza a protocolo V2 para habilitar los controles.","warning");
      return;
    }
    if(/UI:PROTOCOLO_V2/.test(line)) {
      if(!protocolReady)serialLog("Firmware USB reconocido.","good");
      protocolReady=true;
      refreshControls();
    }
    if(/UI:ARM/.test(line))startHeartbeat();
    if(/UI:IDLE|\[PARO (?:[!sx]|CONTROLADO)\]|UI:MOVIMIENTO_INTERRUMPIDO|UI:SALIDA_SEGURA/.test(line))stopHeartbeat();
    parseLine(line);
    if(/^UI:HOME_AJUSTE_HOMBRO_INICIO$/.test(line)) {setAdjustment("h",false);return;}
    if(/^UI:HOME_AJUSTE_HOMBRO_LISTO$/.test(line)) {
      if(adjustmentJoint==="h") {adjustmentCanCommand=true;refreshControls();}
      return;
    }
    if(/^UI:HOME_AJUSTE_HOMBRO_FIN$/.test(line)) {setAdjustment(null);return;}
    if(/^UI:HOME_AJUSTE_CODO_INICIO$/.test(line)) {setAdjustment("c",false);return;}
    if(/^UI:HOME_AJUSTE_CODO_LISTO$/.test(line)) {
      if(adjustmentJoint==="c") {
        if(elbowAwaitingDirection) {
          // No llego a aceptarse el incremento: no enviar la dirección a otro menú.
          serialLog("El codo volvió al menú sin pedir dirección. Comprueba PWM y límites.","warning");
          elbowAwaitingDirection=0;
        }
        adjustmentCanCommand=true;refreshControls();
      }
      return;
    }
    if(/^UI:HOME_AJUSTE_CODO_DIRECCION$/.test(line)) {
      if(adjustmentJoint==="c" && elbowAwaitingDirection){
        const direction=elbowAwaitingDirection;
        elbowAwaitingDirection=0;
        void writeRaw(direction>0?"r":"l");
      }
      return;
    }
    if(/^UI:HOME_AJUSTE_CODO_TERMINADO_POR_OPERADOR$/.test(line)) {
      serialLog("Ajuste del CODO terminado. Espera la pregunta del ESP32 y confirma la postura segura con s/n.","good");
      return;
    }
    if(/^UI:HOME_AJUSTE_CODO_FIN$/.test(line)) {setAdjustment(null);return;}
    if(/^UI:HOME_RECUP_MANUAL_INICIO/.test(line)) {
      recoverySet(true,false); return;
    }
    if(/^UI:HOME_RECUP_MANUAL_FIN/.test(line)) {
      recoverySet(false); return;
    }
    if(recoveryMode && /Comando manual \(r\/l\/p\/q\/x\):/i.test(line)) {
      recoveryCanCommand=true;refreshControls();return;
    }
    if(checkPrompt(line))return;

    if(/UI:SALIDA_SEGURA|SALIDA SEGURA LOGICA/i.test(line)) {
      if(adjustmentJoint)setAdjustment(null);
      pendingQuestion=false;readyPrompt=false;lastPromptKey="";
      protocolReady=false;
      refreshControls();
      $("usb-note").textContent="Salida completada. Reinicia el ESP32 para volver a operar.";
      if(!safeExitLogged) serialLog("SALIDA completada: drivers NEMA OFF y PWM de servos OFF. Referencias invalidadas; reinicia ESP32 para operar.","good");
      safeExitLogged=true;
      return;
    }
    if(/\[PARO x\]/i.test(line)) {
      if(adjustmentJoint)setAdjustment(null);
      if(recoveryMode)recoverySet(false);
      serialLog("PARO OFF: drivers NEMA y PWM deshabilitados; HOME obligatorio.","warning");return;
    }
    if(/\[PARO (?:!|CONTROLADO)\]/i.test(line)) {
      if(adjustmentJoint)setAdjustment(null);
      if(recoveryMode)recoverySet(false);
      const cause=line.replace(/^\[PARO (?:!|CONTROLADO)\]\s*/i,"").trim();
      serialLog("PARO CONTROLADO: "+(cause ? cause+". " : "")+"Pulsos detenidos; NEMA/PWM conservados. HOME obligatorio.","warning");return;
    }
    if(/HOME BASE COMPLETADO/i.test(line)) {if(recoveryMode)recoverySet(false);serialLog("HOME de BASE completado.","good");return;}
    if(/HOME HOMBRO COMPLETADO/i.test(line)) {serialLog("HOME de HOMBRO completado.","good");return;}
    if(/HOME TOTAL COMPLETADO/i.test(line)) {serialLog("HOME total completado.","good");return;}
    if(/HOME BASE.*INCOMPLETO|HOME HOMBRO.*INCOMPLETO/i.test(line)) {
      serialLog("HOME incompleto: "+line,"warning");return;
    }
    let m;
    m=line.match(/^UI:SOLUCION\s+(\d+)\s+(SI|NO|SIN_REFERENCIA)\s*(.*)$/i);
    if(m){
      const index=Number(m[1]),allowed=m[2].toUpperCase()==="SI",reason=m[3].trim();
      window.robotDemo?.setRouteResult(index,allowed,reason);
      if(!allowed) serialLog(`Solución ${index} rechazada: ${reason.replace(/_/g," ")}`,"warning");
      return;
    }
    if(/^UI:PLAN_FIN/.test(line)){
      window.robotDemo?.routePlanFinished();
      serialLog("Validación de las soluciones de CI terminada.","good");
      return;
    }
    if(/^UI:PLAN_INICIO/.test(line))return;
    if(/^UI:RECIBIDO|^UI:PROTOCOLO|^UI:ARM|^UI:IDLE/.test(line))return;
    if(/^UI:JOG_OK/.test(line)){
      serialLog("Movimiento manual completado (estimado).","good");return;
    }
    if(/^UI:GRIP_OK/.test(line)){
      serialLog("Pinza: "+line.replace(/^UI:GRIP_OK\s*/,""),"good");return;
    }
    if(/^UI:SOLUCION_ELEGIDA/.test(line)){
      serialLog("CI: "+line.replace(/^UI:SOLUCION_ELEGIDA\s*/,"solución ")+" seleccionada.","good");return;
    }
    if(/^UI:CONFIRMADO/.test(line)){
      serialLog("Confirmación aceptada: "+line.replace(/^UI:CONFIRMADO\s*/,""),"good");return;
    }
    m=line.match(/^UI:POS_ANTES\s*(.*)$/);
    if(m){serialLog("Posición anterior: "+m[1]);return;}
    m=line.match(/^UI:POS_NUEVA\s*(.*)$/);
    if(m){serialLog("Nueva posición: "+m[1]);return;}
    m=line.match(/^UI:RUTA\s*(.*)$/);
    if(m){serialLog("Trayectoria propuesta: "+m[1],"good");return;}
    if(/^UI:PROPUESTA/.test(line)){
      serialLog(line.replace(/^UI:PROPUESTA\s*/,"Movimiento solicitado: "));
      return;
    }
    if(/^UI:DESTINO XYZ/.test(line)) return;
    if(/^UI:DESTINO_ALCANZADO/.test(line)){
      serialLog("Posición objetivo alcanzada (estimada por pasos/PWM).","good");return;
    }
    if(/^UI:MOVIMIENTO_COMPLETADO/.test(line)){
      serialLog("Movimiento finalizado correctamente.","good");return;
    }
    if(/^UI:MOVIMIENTO_INTERRUMPIDO/.test(line)){
      serialLog("Movimiento interrumpido.","warning");return;
    }
    if(/^UI:CANCELADO/.test(line)){
      serialLog("Operación cancelada: "+line.replace(/^UI:CANCELADO\s*/,""),"warning");return;
    }
    if(/^UI:BLOQUEO/.test(line)){
      serialLog("Límite aplicado: "+line.slice(11),"warning");return;
    }
    if(/MOV1 inactivo/i.test(line)) return;
    if(/^AVISO:.*pinza.*activar/i.test(line)){
      serialLog("Aviso: la pinza puede moverse al activar su PWM.","warning");return;
    }
    if(/PUEDE HABER CAIDA POR GRAVEDAD/i.test(line)){
      serialLog("Advertencia: el brazo podría moverse por gravedad al deshabilitar actuadores.","warning");return;
    }
    if(/^UI:RECHAZADO|^UI:ERROR|^UI:PLAN_ERROR/.test(line)){
      serialLog(line.replace(/^UI:/,""),"warning");return;
    }
    if(/^(?:\[PARO (?:[!sx]|CONTROLADO)\]|PARO ENCLAVADO|HOME INCOMPLETO|TRANSICION.*INCOMPLETA|ERROR:|ADVERTENCIA:)/i.test(line)){
      serialLog(line.replace(/=+/g,""),"warning");return;
    }
    if(/HOME TOTAL (COMPLETADO|TERMINADO)|HOME DE RECUPERACION COMPLETADO/i.test(line)){
      pendingQuestion=false;
      serialLog("HOME completado.","good");return;
    }
    if(/MODO PREPARADO ACTIVADO|READY alcanzado/i.test(line)){
      pendingQuestion=false;
      serialLog("READY alcanzado.","good");return;
    }
    if(/CICLO PINZA FINALIZADO/i.test(line)){
      serialLog("Pinza: apertura y cierre completados según PWM.","good");return;
    }
    // La consola presenta solo eventos esenciales; mensajes técnicos de
    // pasos, PWM intermedios, menús, separadores y ecuaciones se omiten.
  }

  function receive(bytes) {
    buffer += decoder.decode(bytes,{stream:true}).replace(/\r/g,"\n");
    if(flushTimer) {clearTimeout(flushTimer);flushTimer=null;}
    const parts=buffer.split("\n");
    buffer=parts.pop()??"";
    for(const part of parts) receiveLine(part);
    // Algunos prompts de la V6 no finalizan con un salto de línea.
    if(buffer.length>6000){receiveLine(buffer);buffer="";}
    if(buffer.trim()) flushTimer=setTimeout(()=>{
      if(buffer.trim()) receiveLine(buffer);
      buffer="";flushTimer=null;
    },350);
  }

  async function readLoop(activePort) {
    try {
      reader = activePort.readable.getReader();
      while(connected && !disconnecting) {
        const {value:chunk,done} = await reader.read();
        if(done) break;
        if(chunk) receive(chunk);
      }
    } catch(err) {
      if(!disconnecting) serialLog("Lectura USB interrumpida: " + err.message, "warning");
    } finally {
      try {reader?.releaseLock();} catch(_) {}
      reader=null;
      if(connected && !disconnecting) void disconnect("El puerto dejó de enviar datos.");
    }
  }

  async function connect() {
    if(connected)return;
    if(!window.isSecureContext || !navigator.serial) {
      serialLog("Web Serial no disponible. Abre desde http://localhost:5500 con Chrome o Edge.","warning");
      return;
    }
    try {
      // requestPort debe ejecutarse directamente desde el clic del usuario.
      const chosen=await navigator.serial.requestPort();
      await chosen.open({baudRate:BAUD, dataBits:8, stopBits:1, parity:"none", flowControl:"none"});
      port=chosen;
      connected=true;
      disconnecting=false;
      buffer="";
      data.baseHome=null;data.shoulderHome=null;data.prepared=null;data.shoulderReady=null;
      protocolReady=false;recoveryMode=false;recoveryCanCommand=false;adjustmentJoint=null;adjustmentCanCommand=false;elbowAwaitingDirection=0;readyPrompt=false;pendingQuestion=false;lastPromptKey="";safeExitLogged=false;lastPromptKey="";
      renderMode(true);
      serialLog("USB conectado a 115200 baudios.","good");
      serialLog("HOME obligatorio antes de mover: escribe h o pulsa HOME y completa las confirmaciones; después, READY.","warning");
      serialLog("Comandos: h=HOME · p=Estado · s/n=Sí/No · l/r=Dirección · Esc=Paro controlado · x=NEMA/PWM OFF.");
      readTask=readLoop(chosen);
      // Identificacion inofensiva del firmware, incluso si HOME está preguntando.
      // El nuevo firmware responde sin interpretar HELLO como respuesta.
      void writeRaw("@HELLO\n");
      setTimeout(()=>{if(connected && !protocolReady)void writeRaw("@HELLO\n");},1400);
    } catch(err) {
      if(err.name!=="NotFoundError") serialLog("No se pudo conectar: "+err.message,"warning");
      try {if(port && !connected)await port.close();} catch(_) {}
      port=null;
    }
  }

  async function disconnect(message="USB desconectado.") {
    if(!connected || disconnecting)return;
    disconnecting=true;
    stopHeartbeat();
    protocolReady=false;recoveryMode=false;recoveryCanCommand=false;adjustmentJoint=null;adjustmentCanCommand=false;elbowAwaitingDirection=0;readyPrompt=false;pendingQuestion=false;lastPromptKey="";safeExitLogged=false;
    if(flushTimer){clearTimeout(flushTimer);flushTimer=null;}
    if(buffer.trim())receiveLine(buffer);
    buffer="";
    try {await reader?.cancel();} catch(_) {}
    try {await readTask;} catch(_) {}
    try {await port?.close();} catch(err) {serialLog("No fue posible cerrar el puerto: "+err.message,"warning");}
    port=null;readTask=null;connected=false;disconnecting=false;
    renderMode(false);
    serialLog(message,"warning");
  }

  async function send(raw, kind) {
    if(!connected || !port?.writable || sending) return;
    const entry=$("serial-entry");
    const input=entry.value.trim();
    if(!input)return;
    // Atajo h de la INTERFAZ para HOME, en cualquier menú apto.
    // No enviar h cruda: en el menú PREPARADO del firmware significa HOMBRO.
    if(input.toLowerCase()==="h") {
      if(pendingQuestion) {
        serialLog("Responde primero a la pregunta de seguridad pendiente.","warning");
        return;
      }
      const ok=await sendCommand("@HOME");
      if(ok){entry.value="";entry.focus();}
      return;
    }
    if(kind==="key" && (input.length!==1 || !/^[\x21-\x7e]$/.test(input))) {
      serialLog("Tecla: introduce exactamente un carácter ASCII de menú (por ejemplo p, s, n o q).","warning");
      return;
    }
    if(kind==="line" && !/^[\x20-\x7e]{1,64}$/.test(input)) {
      serialLog("Línea: utiliza texto ASCII breve, por ejemplo -86.267.","warning");
      return;
    }
    // Durante el movimiento PWM, comprobar_x() solo procesa PARO y puede
    // descartar s/q. Evitar aceptarlos hasta que el firmware publique LISTO.
    if(adjustmentJoint && !adjustmentCanCommand && /^[sq]$/i.test(input)) {
      serialLog("Espera a que finalice el ajuste manual antes de confirmar o salir.","warning");
      return;
    }
    sending=true;
    try {
      const actualKind=kind==="auto" ? (/^[a-z!]$/i.test(input)?"key":"line") : kind;
      const message=actualKind==="key"?input:input+"\n";
      if(recoveryMode && actualKind==="key" && /^[rl]$/i.test(input)) {
        if(!recoveryCanCommand){serialLog("Espera el próximo comando manual de HOME antes de continuar.","warning");return;}
        recoveryCanCommand=false;refreshControls();
      }
      if(await writeRaw(message)) {
        serialLog("Operador: "+input);
        if(actualKind==="key") {pendingQuestion=false;readyPrompt=false;lastPromptKey="";}
        entry.value="";
        entry.focus();
      }
    } finally {sending=false;}
  }

  window.robotUSB = {
    get connected(){return connected;},
    get protocolReady(){return protocolReady;},
    get recoveryMode(){return recoveryMode;},
    get adjustmentJoint(){return adjustmentJoint;},
    sendCommand,sendKey,sendRecoveryJog,sendAdjustmentJog,requestReady,stop
  };
  document.addEventListener("DOMContentLoaded",()=>{
    $("usb-connect").addEventListener("click", connect);
    $("usb-disconnect").addEventListener("click",()=>void disconnect());
    $("usb-send-enter").addEventListener("click",()=>void send(null,"auto"));
    $("serial-entry").addEventListener("keydown",e=>{
      if(e.key==="Enter") {e.preventDefault();void send(null,"auto");}
    });
    // Una sola tecla: Escape solicita paro controlado (mismo byte 0x03).
    // Se atiende aun si el foco esta en la caja de entrada de la consola.
    window.addEventListener("keydown",e=>{
      if(e.key!=="Escape" || e.repeat || !connected)return;
      e.preventDefault();
      e.stopImmediatePropagation();
      void stop();
    },true);
    if(navigator.serial) {
      navigator.serial.addEventListener("disconnect",()=>{
        if(connected && !disconnecting)void disconnect("El dispositivo USB se desconectó del equipo.");
      });
    }
  });
})();
