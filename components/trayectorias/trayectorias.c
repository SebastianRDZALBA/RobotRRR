/*
 * Componente trayectorias: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "trayectorias.h"

double ci_b_grados_estado(ci_estado_t e)
{
    return (double)e.b_pulsos*360.0/PULSOS_VUELTA_BASE;
}

double ci_h_grados_estado(ci_estado_t e)
{
    return (double)(e.h_pulsos-HOMBRO_READY_PULSOS)*360.0/PULSOS_VUELTA_HOMBRO;
}

double ci_c_grados_estado(ci_estado_t e)
{
    return codo_grados_de_pwm(e.c_us);
}

ci_estado_t ci_cuantizar_estado(double b,double h,double c)
{
    return (ci_estado_t){
        .b_pulsos=(int32_t)lround(b*PULSOS_VUELTA_BASE/360.0),
        .h_pulsos=HOMBRO_READY_PULSOS+
                   (int32_t)lround(h*PULSOS_VUELTA_HOMBRO/360.0),
        .c_us=CODO_HOME_US+
                (int)lround(c*(CODO_PWM_CAL_MAX_US-CODO_HOME_US)/180.0)
    };
}

bool ci_igual(ci_estado_t a,ci_estado_t b)
{
    return a.b_pulsos==b.b_pulsos && a.h_pulsos==b.h_pulsos &&
           a.c_us==b.c_us;
}

bool ci_topes_cuantizados(ci_estado_t e)
{
    return e.b_pulsos>=BASE_LIMITE_IZQUIERDA_PULSOS &&
           e.b_pulsos<=BASE_LIMITE_DERECHA_PULSOS &&
           e.h_pulsos>=HOMBRO_LIMITE_MIN_PULSOS &&
           e.h_pulsos<=HOMBRO_LIMITE_MAX_PULSOS &&
           e.c_us>=CODO_PWM_CAL_MIN_US && e.c_us<=CODO_PWM_CAL_MAX_US;
}

double ci_margen_topes_grados(ci_estado_t e)
{
    const double b=ci_b_grados_estado(e), h=ci_h_grados_estado(e),
                 c=ci_c_grados_estado(e);
    const double h_min=(HOMBRO_LIMITE_MIN_PULSOS-HOMBRO_READY_PULSOS)*
                       360.0/PULSOS_VUELTA_HOMBRO;
    const double h_max=(HOMBRO_LIMITE_MAX_PULSOS-HOMBRO_READY_PULSOS)*
                       360.0/PULSOS_VUELTA_HOMBRO;
    double m=fmin(b-BASE_LIMITE_IZQUIERDA_ESTIMADO,
                   BASE_LIMITE_DERECHA_ESTIMADO-b);
    m=fmin(m,fmin(h-h_min,h_max-h));
    return fmin(m,fmin(c,180.0-c));
}

bool ci_cerca_referencia(ci_estado_t e)
{
    const double h=ci_h_grados_estado(e), c=ci_c_grados_estado(e);
    return fabs(h+89.25)<CI_ALERTA_REFERENCIA_GRADOS ||
           fabs(h-80.10)<CI_ALERTA_REFERENCIA_GRADOS ||
           (h>=125.5 && h<=131.1 && c>=88.5 && c<=109.1);
}

void ci_muestra(ci_ruta_t *r,ci_estado_t punto)
{
    r->muestras++;
    const double m=ci_margen_topes_grados(punto);
    if (m<r->margen_min_topes_deg) r->margen_min_topes_deg=m;
    if (ci_cerca_referencia(punto)) r->cerca_referencia_modelada=true;
}

bool ci_fallar(ci_ruta_t *r,int eje,ci_estado_t punto,
                      const char *motivo)
{
    r->compatible=false;
    r->tiene_fallo=true;
    r->eje_fallo=eje;
    r->fallo=punto;
    r->motivo=motivo;
    return false;
}

bool ci_evaluar_eje(ci_estado_t antes,ci_estado_t despues,
                           int eje,ci_ruta_t *r)
{
    const int32_t desde=(eje==0 ? antes.b_pulsos :
                          eje==1 ? antes.h_pulsos : antes.c_us);
    const int32_t hasta=(eje==0 ? despues.b_pulsos :
                          eje==1 ? despues.h_pulsos : despues.c_us);
    const int sentido=hasta>=desde?1:-1;
    const double b0=ci_b_grados_estado(antes),c0=ci_c_grados_estado(antes);
    // Se recorren muestras de todo el trayecto, incluyendo nodos.
    for (int32_t p=desde;;p+=sentido) {
        ci_estado_t punto=antes;
        if (eje==0) punto.b_pulsos=p;
        if (eje==1) punto.h_pulsos=p;
        if (eje==2) punto.c_us=(int)p;
        ci_muestra(r,punto);
        if (!ci_topes_cuantizados(punto))
            return ci_fallar(r,eje,punto,"Tope general despues de cuantizar");
        const char *razon=NULL;
        if (eje==0) {
            const double b=ci_b_grados_estado(punto);
            // Se replica la comprobacion completa desde el inicio hacia
            // cada posicion intermedia, no solo el destino final.
            if (MOV1_ACTIVO && p!=desde &&
                !ci_base_mov1(antes.h_pulsos,c0,b0,b))
                return ci_fallar(r,eje,punto,
                           "MOV1: base excede -51/+2.5 en postura critica");
            if (antes.h_pulsos==PERFIL_HOMBRO_CRITICO_PULSOS && p!=desde) {
                if (c0>PERFIL_CODO_MAX_EXTENDIDO+PERFIL_TOLERANCIA_ANGULO)
                    return ci_fallar(r,eje,punto,
                             "MOV4: codo demasiado extendido para giro base");
                const double lo=fmin(b0,b),hi=fmax(b0,b);
                const bool cruza_centro=
                    hi>PERFIL_BASE_CAMBIO_CENTRAL+PERFIL_TOLERANCIA_ANGULO &&
                    lo<PERFIL_BASE_CAMBIO_POSITIVO-PERFIL_TOLERANCIA_ANGULO;
                const bool cruza_neg=
                    lo<PERFIL_BASE_CAMBIO_NEGATIVO-PERFIL_TOLERANCIA_ANGULO;
                if (c0>PERFIL_CODO_MAX_DESPEJADO+PERFIL_TOLERANCIA_ANGULO &&
                    (cruza_centro || cruza_neg))
                    return ci_fallar(r,eje,punto,
                        "MOV4: base cruza sector con codo demasiado extendido");
            }
        } else if (eje==1) {
            if (MOV1_ACTIVO && p!=desde && sentido>0 &&
                !m1_hombro_derecha_valido(p,b0,c0))
                return ci_fallar(r,eje,punto,
                    "MOV1: avance positivo de hombro fuera de perfil V3");
            if (p!=desde && sentido<0 &&
                antes.h_pulsos>=PERFIL_HOMBRO_M2_PULSOS &&
                despues.h_pulsos<PERFIL_HOMBRO_M2_PULSOS &&
                despues.h_pulsos>=HOMBRO_READY_PULSOS &&
                c0<=PERFIL_M2_CODO_MIN+PERFIL_TOLERANCIA_ANGULO &&
                p<PERFIL_HOMBRO_M2_PULSOS)
                return ci_fallar(r,eje,punto,
                                 "MOV2: descenso de hombro bloqueado");
        }
        if (!ci_postura_m2_m4(punto,&razon))
            return ci_fallar(r,eje,punto,razon);
        if (p==hasta) break;
    }
    return true;
}

ci_ruta_t ci_probar_orden(ci_estado_t inicio,ci_estado_t destino,
                                int numero_orden)
{
    ci_ruta_t r={0};
    r.compatible=true;
    r.motivo="Sin incumplimientos detectados en el modelo";
    r.eje_fallo=-1;
    r.margen_min_topes_deg=INFINITY;
    ci_estado_t actual=inicio;
    for (int j=0;j<3;j++) {
        const int eje=ci_ordenes[numero_orden][j];
        ci_estado_t siguiente=actual;
        if (eje==0) siguiente.b_pulsos=destino.b_pulsos;
        else if (eje==1) siguiente.h_pulsos=destino.h_pulsos;
        else siguiente.c_us=destino.c_us;
        if (!ci_evaluar_eje(actual,siguiente,eje,&r)) return r;
        actual=siguiente;
    }
    return r;
}

double ci_costo_estados(ci_estado_t a,ci_estado_t b)
{
    return fabs(ci_b_grados_estado(b)-ci_b_grados_estado(a))+
           fabs(ci_h_grados_estado(b)-ci_h_grados_estado(a))+
           fabs(ci_c_grados_estado(b)-ci_c_grados_estado(a));
}

ci_analisis_t ci_analizar_solucion(const ci_solucion_t *s,
                                         ci_estado_t inicio,
                                         double px,double py,double pz)
{
    ci_analisis_t a={0};
    a.orden=-1;
    a.orden_segundo=-1;
    a.motivo="No hay ruta compatible con el modelo";
    a.margen_min_topes_deg=INFINITY;
    const ci_estado_t fin=ci_cuantizar_estado(s->base_oper_deg,
                                             s->hombro_oper_deg,
                                             s->codo_nominal_deg);
    a.costo_grados=ci_costo_estados(inicio,fin);
    double x,y,z;
    if (cd_calcular_mm(ci_b_grados_estado(fin),ci_h_grados_estado(fin),
                       ci_c_grados_estado(fin),&x,&y,&z))
        a.error_cuantizado_mm=hypot(hypot(px-x,py-y),pz-z);
    else a.error_cuantizado_mm=INFINITY;
    if (!ci_topes_cuantizados(fin)) {
        a.motivo="Topes generales despues de cuantizar";
        return a;
    }
    // Los seis resultados directos quedan en el informe con el primer
    // bloqueo y su punto concreto. No movemos ninguno de los ejes.
    for (int k=0;k<6;k++) {
        a.directas[k]=ci_probar_orden(inicio,fin,k);
        a.muestras_totales+=a.directas[k].muestras;
        if (a.directas[k].compatible) {
            a.rutas_directas_ok++;
            if (a.orden<0) {
                a.orden=k;
                a.modelo_permite_recorrido=true;
                a.muestras_elegida=a.directas[k].muestras;
                a.margen_min_topes_deg=a.directas[k].margen_min_topes_deg;
                a.cerca_referencia=a.directas[k].cerca_referencia_modelada;
                a.motivo="Ruta directa compatible con el modelo (NO fisica)";
            }
        }
    }
    if (a.modelo_permite_recorrido) return a;
    a.motivo=a.directas[0].motivo;

    const ci_estado_t listo=ci_cuantizar_estado(0.0,0.0,90.0);
    if (ci_igual(inicio,listo) || ci_igual(fin,listo)) return a;
    // Reutilizar solo seis primeros tramos, luego 6 por cada uno.
    // El costo adicional y el numero de muestras quedan explicitados.
    const double costo_ready=ci_costo_estados(inicio,listo)+
                             ci_costo_estados(listo,fin);
    for (int k=0;k<6;k++) {
        ci_ruta_t primero=ci_probar_orden(inicio,listo,k);
        a.muestras_totales+=primero.muestras;
        if (!primero.compatible) continue;
        for (int m=0;m<6;m++) {
            ci_ruta_t segundo=ci_probar_orden(listo,fin,m);
            a.muestras_totales+=segundo.muestras;
            if (!segundo.compatible) continue;
            a.modelo_permite_recorrido=true;
            a.via_ready=true;
            a.orden=k;
            a.orden_segundo=m;
            a.costo_grados=costo_ready;
            a.muestras_elegida=primero.muestras+segundo.muestras;
            a.margen_min_topes_deg=fmin(primero.margen_min_topes_deg,
                                       segundo.margen_min_topes_deg);
            a.cerca_referencia=primero.cerca_referencia_modelada ||
                              segundo.cerca_referencia_modelada;
            a.motivo="Alternativa VIA PREPARADO compatible SOLO con modelo";
            return a;
        }
    }
    return a;
}

ci_estado_t ci521_interpolar(ci_estado_t a,ci_estado_t b,
                                     unsigned k,unsigned n)
{
    const double u=n>0?(double)k/n:1.0;
    return (ci_estado_t){
        a.b_pulsos+(int32_t)lround((b.b_pulsos-a.b_pulsos)*u),
        a.h_pulsos+(int32_t)lround((b.h_pulsos-a.h_pulsos)*u),
        a.c_us+(int)lround((b.c_us-a.c_us)*u)
    };
}

unsigned ci521_muestras_intervalo(ci_estado_t a,ci_estado_t b)
{
    unsigned nb=(unsigned)labs((long)b.b_pulsos-a.b_pulsos);
    unsigned nh=(unsigned)labs((long)b.h_pulsos-a.h_pulsos);
    unsigned nc=(unsigned)abs(b.c_us-a.c_us);
    unsigned max=nb>nh?nb:nh;
    return max>nc?max:nc;
}

ci_ruta_t ci521_probar_sincronizada(ci_estado_t inicio,
                                           ci_estado_t fin,int orden)
{
    ci_ruta_t r={0};
    r.compatible=true;
    r.eje_fallo=-1;
    r.motivo="Sin bloqueos conocidos en el modelo parcial";
    r.margen_min_topes_deg=INFINITY;
    ci_estado_t actual=inicio;
    unsigned n=ci521_muestras_intervalo(inicio,fin);
    ci_muestra(&r,actual);
    if (!ci_topes_cuantizados(actual)) {
        (void)ci_fallar(&r,0,actual,"Inicio fuera de topes cuantizados");
        return r;
    }
    const char *motivo=NULL;
    if (!ci_postura_m2_m4(actual,&motivo)) {
        (void)ci_fallar(&r,0,actual,motivo);
        return r;
    }
    for (unsigned k=1;k<=n;k++) {
        ci_estado_t proximo=ci521_interpolar(inicio,fin,k,n);
        for (int j=0;j<3;j++) {
            const int eje=ci_ordenes[orden][j];
            ci_estado_t paso=actual;
            if (eje==0) paso.b_pulsos=proximo.b_pulsos;
            else if(eje==1) paso.h_pulsos=proximo.h_pulsos;
            else paso.c_us=proximo.c_us;
            if (!ci_evaluar_eje(actual,paso,eje,&r)) return r;
            actual=paso;
        }
        // El avance simultaneo puede atravesar configuraciones que los
        // ejes individuales evitan, por lo que se comprueba en cada tick.
        if (!ci_topes_cuantizados(actual)) {
            (void)ci_fallar(&r,0,actual,"Topes en tick sincronizado");
            return r;
        }
        motivo=NULL;
        if (!ci_postura_m2_m4(actual,&motivo)) {
            (void)ci_fallar(&r,2,actual,motivo);
            return r;
        }
    }
    return r;
}

ci521_sincronizado_t ci521_analizar_sincronizada(
                                        ci_estado_t inicio,ci_estado_t fin)
{
    ci521_sincronizado_t a={0};
    a.orden=-1;
    a.costo=ci_costo_estados(inicio,fin);
    if (!ci_topes_cuantizados(inicio) || !ci_topes_cuantizados(fin))
        return a;
    for(int i=0;i<6;i++) {
        a.rutas[i]=ci521_probar_sincronizada(inicio,fin,i);
        if(a.rutas[i].compatible && a.orden<0) {
            a.orden=i;
            a.compatible=true;
            a.muestras=a.rutas[i].muestras;
        }
    }
    return a;
}

void ci_imprimir_fallo(const ci_ruta_t *r)
{
    if (!r->tiene_fallo) return;
    const double b=ci_b_grados_estado(r->fallo),
                 h=ci_h_grados_estado(r->fallo),
                 c=ci_c_grados_estado(r->fallo);
    double x=0,y=0,z=0;
    const bool xyz=cd_calcular_mm(b,h,c,&x,&y,&z);
    printf("     Primer bloqueo en %s: B=%+.3f H=%+.3f C=%.3f deg\n",
           ci_nombres_eje[r->eje_fallo],b,h,c);
    if (xyz)
        printf("     Punto teorico: Px=%+.3f Py=%+.3f Pz=%+.3f mm\n",x,y,z);
    printf("     Motivo: %s | muestras hasta fallo: %u\n",
           r->motivo,r->muestras);
}

bool ci_postura_actual(ci_estado_t *e)
{
    if (!e || paro || sistema_cerrado || !base_home || !hombro_home ||
        !servo_codo.pwm_activo) return false;
    const double b=(double)base_pulsos*360.0/PULSOS_VUELTA_BASE;
    const double h=(double)(hombro_rel_pulsos-HOMBRO_READY_PULSOS)*
                   360.0/PULSOS_VUELTA_HOMBRO;
    const double c=codo_grados_de_pwm(servo_codo.us_actual);
    if (!ci_limites_generales(b,h,c)) return false;
    *e=(ci_estado_t){base_pulsos,hombro_rel_pulsos,servo_codo.us_actual};
    return true;
}

void menu_cinematica_inversa(void)
{
    if (paro || sistema_cerrado) return;
    if (paro) return;
    printf("\n===== CINEMATICA INVERSA Y RUTAS V6 | SOLO CALCULO =====\n");
    printf("NO mueve motores, NO envia PWM ni altera HOME o limites.\n");
    printf("Modelo: L1=%.1f L2=%.1f L3=%.1f D2=%.1f D3=%.1f mm.\n",
           CD_L1_MM,CD_L2_MM,CD_L3_MM,CD_DELTA2_MM,CD_DELTA3_MM);
    printf("Convencion validada: q1=-BASE, q2=HOMBRO, q3=90-CODO.\n");
    printf("Tolerancia cartesiana %.4f mm, solo para numeros redondeados.\n",
           CI_TOL_CARTESIANA_MM);
    printf("Ingrese coordenadas XYZ de la PINZA en milimetros.\n");
    double px,py,pz;
    if (!ci_leer_coordenada_mm("P_x",&px) ||
        !ci_leer_coordenada_mm("P_y",&py) ||
        !ci_leer_coordenada_mm("P_z",&pz)) return;
    ci_solucion_t soluciones[CI_MAX_SOLUCIONES];
    const int total=ci_calcular_mm(px,py,pz,soluciones);
    printf("\n--- RESULTADOS CI: P=(%+.6f,%+.6f,%+.6f) mm ---\n",px,py,pz);
    if (total==0) {
        printf("SIN SOLUCION REAL dentro de tolerancia geometrica de %.4f mm.\n",
               CI_TOL_CARTESIANA_MM);
        printf("No se calcula ni ejecuta ningun movimiento.\n");
        return;
    }
    ci_estado_t actual={0};
    const bool tiene_referencia=ci_postura_actual(&actual);
    ci_analisis_t mejor_analisis={0};
    int mejor=-1,validas=0;
    double costo_mejor=INFINITY;
    bool preferencia_directa=false;
    if (tiene_referencia) {
        printf("\nPOSTURA ACTUAL ESTIMADA: BASE=%+.3f HOMBRO=%+.3f CODO=%.3f deg\n",
               ci_b_grados_estado(actual),ci_h_grados_estado(actual),
               ci_c_grados_estado(actual));
        printf("Se comparan 6 ordenes por ejes; si fallan, alternativa VIA PREPARADO.\n");
        printf("Un 'punto' es 1 micropaso NEMA o 1 us de PWM pedido, NO\n");
        printf("un sensor de posicion ni un detector geometrico de choques.\n");
    } else {
        printf("\nSIN PRESELECCION: faltan HOME y/o PWM o estado fuera de topes.\n");
        printf("Se muestran unicamente soluciones matematicas.\n");
    }
    for (int i=0;i<total;i++) {
        const ci_solucion_t *sol=&soluciones[i];
        printf("\nSOLUCION %d de %d | %s\n",i+1,total,
               sol->limites_articulares_generales?
                 "DENTRO DE TOPES TEORICOS (con tolerancia)":
                 "FUERA DE TOPES ARTICULARES GENERALES");
        printf("  Matematica: q1=%+.3f q2=%+.3f q3=%+.3f grados\n",
               sol->q1_deg,sol->q2_deg,sol->q3_deg);
        printf("  ESP32: BASE=%+.3f | HOMBRO=%+.3f | CODO=%.3f nominal\n",
               sol->base_oper_deg,sol->hombro_oper_deg,
               sol->codo_nominal_deg);
        printf("  Error reconstruccion CD: %.6f mm\n",
               sol->error_reconstruccion_mm);
        if (sol->usa_tolerancia_geometrica)
            printf("  AVISO: punto proyectado por redondeo; error no nulo.\n");
        if (sol->usa_tolerancia_articular)
            printf("  AVISO: limite teorico aceptado por redondeo;\n"
                   "         se verifican topes estrictos tras cuantizar.\n");
        if (sol->limites_articulares_generales) validas++;
        if (!sol->limites_articulares_generales || !tiene_referencia) continue;
        ci_analisis_t resultado=ci_analizar_solucion(sol,actual,px,py,pz);
        const ci_analisis_t *a=&resultado;
        printf("  Rutas DIRECTAS compatibles con modelo: %d/6\n",
               a->rutas_directas_ok);
        printf("  Muestras revisadas entre rutas probadas: %u\n",
               a->muestras_totales);
        printf("  Error destino cuantizado: %.3f mm (sin feedback)\n",
               a->error_cuantizado_mm);
        if (a->modelo_permite_recorrido) {
            printf("  Ruta candidata: %s\n",a->via_ready?
                   "VIA PREPARADO (2 tramos)":"DIRECTA (3 ejes)");
            printf("  Tramo 1: %s\n",ci_nombres_orden[a->orden]);
            printf("  Muestras de ruta candidata: %u\n",a->muestras_elegida);
            if (a->via_ready)
                printf("  Tramo 2 despues de PREPARADO: %s\n",
                       ci_nombres_orden[a->orden_segundo]);
            printf("  Cambio angular total: %.3f grados\n",a->costo_grados);
            printf("  Menor margen a TOPES GENERALES: %.3f grados\n",
                   a->margen_min_topes_deg);
            if (a->cerca_referencia || a->margen_min_topes_deg<0.5)
                printf("  ALERTA: cerca de tope o referencia del modelo;\n"
                       "         NECESITA VERIFICACION FISICA.\n");
            const bool nueva_directa=!a->via_ready;
            // Entre candidatas modeladas preferimos ruta directa a via READY;
            // si comparten tipo, se compara suma total de cambios angulares.
            if (mejor<0 || (nueva_directa && !preferencia_directa) ||
                (nueva_directa==preferencia_directa &&
                 a->costo_grados<costo_mejor)) {
                mejor=i;
                mejor_analisis=resultado;
                preferencia_directa=nueva_directa;
                costo_mejor=a->costo_grados;
            }
        } else {
            printf("  SIN RUTA COMPATIBLE con restricciones modeladas.\n");
            for (int k=0;k<6;k++) {
                printf("   Orden %d (%s): %s\n",k+1,ci_nombres_orden[k],
                       a->directas[k].motivo);
                ci_imprimir_fallo(&a->directas[k]);
            }
            printf("  Alternativa VIA PREPARADO: tampoco compatible segun modelo\n"
                   "  (o no aplica a esta postura).\n");
        }
    }
    printf("\n%d soluciones reales; %d dentro de topes TEORICOS.\n",
           total,validas);
    if (mejor>=0) {
        const ci_analisis_t *a=&mejor_analisis;
        const ci_solucion_t *sol=&soluciones[mejor];
        printf("\nPRESELECCION SOLO TEORICA: SOLUCION %d\n",mejor+1);
        printf("  B=%+.3f H=%+.3f C=%.3f nominal\n",sol->base_oper_deg,
               sol->hombro_oper_deg,sol->codo_nominal_deg);
        printf("  Tipo: %s | cambios: %.3f grados\n",
               a->via_ready?"VIA PREPARADO":"DIRECTA",a->costo_grados);
        printf("  Tramo 1: %s\n",ci_nombres_orden[a->orden]);
        if (a->via_ready)
            printf("  Tramo 2: %s\n",ci_nombres_orden[a->orden_segundo]);
        printf("  Ruta NO AUTORIZADA para ejecucion automatica.\n");
    } else {
        printf("\nSIN PRESELECCION: falta referencia o todas las rutas\n"
               "son incompatibles con el MODELO DISPONIBLE.\n");
    }
    printf("\nLIMITACION: MOV1 INACTIVO; MOV2/MOV4 se basan en datos\n"
           "parciales, NO en geometria 3D. PWM no confirma el angulo real.\n"
           "Muestrear micropasos NO demuestra que no haya choque real.\n"
           "NO se envia ninguna orden de motor ni PWM.\n");
}

void menu_sincronizado_521(void)
{
    if (paro || sistema_cerrado) {
        printf("HOME obligatorio tras paro; planificacion bloqueada.\n");
        return;
    }
    if (paro) return;
    printf("\n===== V6: PLAN SINCRONIZADO (SIN MOVIMIENTO) =====\n");
    printf("Motor BASE y HOMBRO en micropasos; CODO en consignas PWM.\n");
    printf("El programa NO conoce la velocidad ni la posicion real MG996R.\n");
    printf("Solo acepta soluciones con referencia HOME y PWM activos.\n");
    double px,py,pz;
    if (!ci_leer_coordenada_mm("P_x",&px) ||
        !ci_leer_coordenada_mm("P_y",&py) ||
        !ci_leer_coordenada_mm("P_z",&pz)) return;
    ci_solucion_t soluciones[CI_MAX_SOLUCIONES];
    int total=ci_calcular_mm(px,py,pz,soluciones);
    ci_estado_t inicio={0};
    bool referenciada=ci_postura_actual(&inicio);
    printf("Destino P=(%+.3f,%+.3f,%+.3f) mm\n",px,py,pz);
    if(!total) {
        printf("RECHAZADA: no existe solucion geometrica real. Introduce otro destino.\n");
        return;
    }
    if(!referenciada) {
        printf("RECHAZADA: faltan referencias HOME/PWM. No planificar recorrido.\n");
        return;
    }
    printf("Inicio estimado: B=%+.3f H=%+.3f C=%.3f nominal\n",
           ci_b_grados_estado(inicio),ci_h_grados_estado(inicio),
           ci_c_grados_estado(inicio));
    int mejor=-1; double menor=INFINITY;
    ci521_sincronizado_t elegido={0};
    for(int i=0;i<total;i++) {
        ci_solucion_t *sol=&soluciones[i];
        printf("\nSOLUCION %d: B=%+.3f H=%+.3f C=%.3f\n",i+1,
               sol->base_oper_deg,sol->hombro_oper_deg,sol->codo_nominal_deg);
        if(!sol->limites_articulares_generales) {
            printf("  RECHAZADA: excede topes teoricos articulares.\n");
            continue;
        }
        ci_estado_t fin=ci_cuantizar_estado(sol->base_oper_deg,
                                           sol->hombro_oper_deg,
                                           sol->codo_nominal_deg);
        if(!ci_topes_cuantizados(fin)) {
            printf("  RECHAZADA: fuera de topes al cuantizar NEMA/PWM.\n");
            continue;
        }
        // Mantener, ademas, el diagnostico por ejes de V5.19/V5.20.
        ci_analisis_t seq=ci_analizar_solucion(sol,inicio,px,py,pz);
        printf("  V5.20 por ejes: %d/6 directas compatibles; VIA PREPARADO=%s\n",
               seq.rutas_directas_ok,
               seq.via_ready?"SI":"NO/NO NECESARIA");
        ci521_sincronizado_t a=ci521_analizar_sincronizada(inicio,fin);
        if(a.compatible) {
            printf("  MODELO: CANDIDATA sincronizada | microorden %s\n",
                   ci_nombres_orden[a.orden]);
            printf("  Cambio total estimado %.3f deg | %u muestras\n",
                   a.costo,a.muestras);
            printf("  AVISO: no es prueba de colision fisica.\n");
            if(mejor<0 || a.costo<menor) {
                mejor=i; menor=a.costo; elegido=a;
            }
        } else {
            printf("  RECHAZADA: 6 microordenes fallan restricciones modeladas.\n");
            for(int k=0;k<6;k++) {
                printf("   %s: %s\n",ci_nombres_orden[k],a.rutas[k].motivo?
                       a.rutas[k].motivo:"fuera de topes");
                if(a.rutas[k].tiene_fallo) ci_imprimir_fallo(&a.rutas[k]);
            }
        }
    }
    if(mejor>=0) {
        printf("\nPRESELECCION SINCRONIZADA: SOLUCION %d | %s\n",
               mejor+1,ci_nombres_orden[elegido.orden]);
        printf("Suma de cambios: %.3f deg | ruta solo modelada.\n",menor);
    } else {
        printf("\nSIN RUTA compatible con el modelo: solicita otras XYZ.\n");
    }
    printf("Comando w: SOLO SIMULACION. Para ejecucion supervisada usa r.\n");
    printf("No se ha movido ningun motor ni cambiado PWM.\n");
}

void ci521_prueba_paro_virtual(void)
{
    if(paro || sistema_cerrado) return;
    printf("\n===== TEST DE PARO SIN MOTORES (12 segundos) =====\n");
    printf("Teclea s para mantener torque, x para deshabilitar todo.\n");
    printf("Ambos paros invalidan HOME. No se generan pulsos ni PWM nuevos.\n");
    // FreeRTOS ya esta incluido: no se requiere el componente esp_timer.
    const TickType_t inicio = xTaskGetTickCount();
    const TickType_t duracion = pdMS_TO_TICKS(CI521_TEST_VIRTUAL_MS);
    while (!paro &&
           (TickType_t)(xTaskGetTickCount() - inicio) < duracion) {
        if (comprobar_x()) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!paro) printf("TEST terminado sin paro. El sistema conserva HOME.\n");
    else printf("TEST detenido de inmediato; h=HOME recuperacion (se confirma mas tarde).\n");
}

bool ci522_esperar_interrumpible_ms(unsigned ms)
{
    TickType_t inicio=xTaskGetTickCount();
    TickType_t ticks=pdMS_TO_TICKS(ms);
    if (ticks==0) ticks=1;
    while ((TickType_t)(xTaskGetTickCount()-inicio)<ticks) {
        if (comprobar_x()) return false;
        vTaskDelay(1);
    }
    return !comprobar_x();
}

bool ci522_esperar_interrumpible_us(uint32_t microsegundos)
{
    int64_t fin=esp_timer_get_time() + (int64_t)microsegundos;
    while(esp_timer_get_time()<fin) {
        if(comprobar_x()) return false;
        int64_t restante=fin-esp_timer_get_time();
        if(restante>2500) vTaskDelay(1);
        else if(restante>0) {
            uint32_t tramo=(uint32_t)(restante>500 ? 500 : restante);
            esp_rom_delay_us(tramo);
        }
    }
    return !comprobar_x();
}

bool ci523_ciclo_pinza_destino(void)
{
    if (comprobar_x()) return false;
    if (!servo_pinza.pwm_activo) {
        printf("PINZA NO PREPARADA: se cancela el ciclo automatico.\n");
        paro_controlado("PWM pinza no activo tras llegar a XYZ");
        return false;
    }
    printf("\n===== V5.23: CICLO AUTOMATICO PINZA =====\n");
    printf("PINZA: ABRIR a %d us, mantener 1 segundo, CERRAR a %d us.\n",
           PINZA_ABIERTA_US,PINZA_CERRADA_US);
    printf("!=PARO CON TORQUE | x=PARO PWM/NEMA OFF, sin confirmaciones.\n");
    if (comprobar_x()) return false;
    if (!servo_escribir_pwm(&servo_pinza,PINZA_ABIERTA_US)) {
        if(!paro) paro_controlado("Fallo al ordenar apertura de pinza");
        return false;
    }
    printf("PINZA: apertura solicitada (%d us); posicion fisica sin sensor.\n",
           PINZA_ABIERTA_US);
    if (!ci522_esperar_interrumpible_ms(CI523_PINZA_ABIERTA_TIEMPO_MS)) {
        printf("CICLO PINZA INTERRUMPIDO antes del cierre. HOME obligatorio.\n");
        return false;
    }
    if (comprobar_x()) return false;
    if (!servo_escribir_pwm(&servo_pinza,PINZA_CERRADA_US)) {
        if(!paro) paro_controlado("Fallo al ordenar cierre de pinza");
        return false;
    }
    printf("PINZA: cierre solicitado (%d us); sin confirmacion fisica.\n",
           PINZA_CERRADA_US);
    if(!ci522_esperar_interrumpible_ms(CI523_PINZA_CIERRE_ASENTAMIENTO_MS)) {
        printf("CICLO PINZA INTERRUMPIDO durante cierre. HOME obligatorio.\n");
        return false;
    }
    printf("CICLO PINZA FINALIZADO SEGUN CONSIGNAS PWM.\n");
    return true;
}

void ci522_error_en_movimiento(const char *razon)
{
    if (!paro) paro_controlado(razon ? razon :
                              "Error durante la ejecucion sincronizada");
}

bool ci522_comprobar_estado_cuantizado(ci_estado_t esperado)
{
    return base_pulsos==esperado.b_pulsos &&
           hombro_rel_pulsos==esperado.h_pulsos &&
           servo_codo.pwm_activo && servo_codo.us_actual==esperado.c_us;
}

bool ci522_ejecutar(ci_estado_t inicio,ci_estado_t destino,int orden)
{
    if (paro || sistema_cerrado || orden<0 || orden>=6) return false;
    ci_estado_t lectura={0};
    if (!ci_postura_actual(&lectura) || !ci_igual(lectura,inicio)) {
        printf("EJECUCION RECHAZADA: referencias/posicion estimada cambiaron.\n");
        return false;
    }
    // Se vuelve a validar JUSTO antes de habilitar los drivers.
    ci_ruta_t validacion=ci521_probar_sincronizada(inicio,destino,orden);
    if (!validacion.compatible || !ci_topes_cuantizados(destino)) {
        printf("EJECUCION RECHAZADA por topes/MOV2-4: %s\n",
               validacion.motivo ? validacion.motivo:"trayectoria incompatible");
        if (validacion.tiene_fallo) ci_imprimir_fallo(&validacion);
        return false;
    }
    if (comprobar_x()) return false;

    const unsigned n=ci521_muestras_intervalo(inicio,destino);
    if (n==0) {
        printf("YA EN DESTINO: no hay movimiento de articulaciones.\n");
        return true;
    }

    // Energizar sin la espera bloqueante de habilitar(): aqui el tiempo
    // de asentamiento tambien consulta !/x y no se pierde el paro.
    gpio_set_level(base.en,0);
    base_driver_habilitado=true;
    gpio_set_level(hombro.en,0);
    if (!ci522_esperar_interrumpible_ms(20)) return false;
    if (comprobar_x()) return false;
    printf("EJECUTANDO %u ticks; microorden %s. !=PARO TORQUE, x=PARO OFF.\n",
           n,ci_nombres_orden[orden]);
    ci_estado_t actual=inicio;
    for(unsigned k=1;k<=n;k++) {
        if (comprobar_x()) return false;
        ci_estado_t proximo=ci521_interpolar(inicio,destino,k,n);
        for(int j=0;j<3;j++) {
            const int eje=ci_ordenes[orden][j];
            ci_estado_t siguiente=actual;
            if(eje==0) siguiente.b_pulsos=proximo.b_pulsos;
            else if(eje==1) siguiente.h_pulsos=proximo.h_pulsos;
            else siguiente.c_us=proximo.c_us;
            if(ci_igual(actual,siguiente)) continue;

            // Guardar la misma regla y sentido que verifico el planificador.
            ci_ruta_t guard={0};
            guard.compatible=true;
            guard.eje_fallo=-1;
            guard.margen_min_topes_deg=INFINITY;
            if(!ci_evaluar_eje(actual,siguiente,eje,&guard)) {
                printf("FALLO de la verificacion MOV2-4 EN EJECUCION:\n");
                if(guard.tiene_fallo) ci_imprimir_fallo(&guard);
                ci522_error_en_movimiento("Restriccion detectada durante la ruta");
                return false;
            }
            if(comprobar_x()) return false;
            if(eje==0) {
                // Un unico micropaso: ci521_interpolar() limita el salto.
                int dir=(siguiente.b_pulsos>actual.b_pulsos) ?
                          BASE_DERECHA : BASE_IZQUIERDA;
                if (!paso(&base,dir,CI522_PASO_NEMA_US)) return false;
                base_objetivo_grados=ci_b_grados_estado(siguiente);
            } else if(eje==1) {
                const int dir=(siguiente.h_pulsos>actual.h_pulsos) ?
                         HOMBRO_DIR_ALEJAR : HOMBRO_DIR_HOME;
                // Mantener el chequeo del sensor del movimiento manual.
                if(switch_estable(hombro.sw,1)) {
                    printf("ERROR: switch hombro activo durante desplazamiento operativo.\n");
                    ci522_error_en_movimiento("HOME hombro no confiable por switch");
                    return false;
                }
                if (!paso(&hombro,dir,CI522_PASO_NEMA_US)) return false;
                if(switch_estable(hombro.sw,1)) {
                    printf("ERROR: switch hombro activado durante operacion normal.\n");
                    ci522_error_en_movimiento("HOME hombro no confiable por switch");
                    return false;
                }
                hombro_preparado=(hombro_rel_pulsos==HOMBRO_READY_PULSOS);
            } else {
                if (!servo_escribir_pwm(&servo_codo,siguiente.c_us)) {
                    ci522_error_en_movimiento("Error PWM o restriccion MOV2/4");
                    return false;
                }
                codo_angulo_objetivo=ci_c_grados_estado(siguiente);
            }
            actual=siguiente;
            if (!ci522_comprobar_estado_cuantizado(actual)) {
                ci522_error_en_movimiento("Contador/PWM distinto a la ruta esperada");
                return false;
            }
        }
        // Cada tick contiene como mucho 1 micropaso por NEMA y 1 us
        // de actualizacion PWM; la demora adicional limita la rapidez.
        if(!ci522_esperar_interrumpible_us(CI522_PAUSA_TICK_US)) return false;
        if(k%100==0 || k==n) {
            printf("AVANCE MODELO: %u/%u | B%+.3f H%+.3f C%.3f grados\n",
                   k,n,ci_b_grados_estado(actual),
                   ci_h_grados_estado(actual),ci_c_grados_estado(actual));
        }
    }
    if(comprobar_x()) return false;
    if (!ci522_comprobar_estado_cuantizado(destino)) {
        ci522_error_en_movimiento("Destino no coincide con pasos/PWM");
        return false;
    }
    ++num_movimientos;
    printf("\nDESTINO alcanzado SEGUN PASOS/PWM, SIN sensores absolutos.\n");
    printf("B%+.3f H%+.3f C%.3f | HOME sigue valido; no se midio error fisico.\n",
           ci_b_grados_estado(destino),ci_h_grados_estado(destino),
           ci_c_grados_estado(destino));
    return true;
}

void ci522_menu_ejecutar_xyz(void)
{
    if(paro || sistema_cerrado) return;
    if (paro) return;
    printf("\n===== V6: CINEMATICA INVERSA CON MOTORES =====\n");
    printf("SIMULACION + COMPROBACION COMPLETA ANTES DE ACTIVAR MOTORES.\n");
    printf("SIN limites extra: topes vigentes y MOV2-4; MOV1 inactivo.\n");
    printf("!=paro controlado con torque; x=paro PWM/NEMA OFF.\n");
    ci_estado_t inicio={0};
    if (!ci_postura_actual(&inicio)) {
        printf("RECHAZADA: HOME base/hombro y PWM codo obligatorios.\n");
        return;
    }
    double px,py,pz;
    if (!ci_leer_coordenada_mm("P_x",&px) ||
        !ci_leer_coordenada_mm("P_y",&py) ||
        !ci_leer_coordenada_mm("P_z",&pz)) return;
    if (comprobar_x()) return;
    // Si se interrumpio o cambio la postura, el preanalisis no se conserva.
    ci_estado_t actual={0};
    if (!ci_postura_actual(&actual) || !ci_igual(actual,inicio)) {
        printf("RECHAZADA: postura cambiada antes del calculo.\n");
        return;
    }
    ci_solucion_t soluciones[CI_MAX_SOLUCIONES];
    int total=ci_calcular_mm(px,py,pz,soluciones);
    if(total<=0) {
        printf("RECHAZADA: XYZ sin solucion geometrica. Introduce otra posicion.\n");
        return;
    }
    int mejor=-1;
    double menor=INFINITY;
    bool candidatas[CI_MAX_SOLUCIONES]={false};
    ci_estado_t destinos[CI_MAX_SOLUCIONES]={0};
    int ordenes[CI_MAX_SOLUCIONES]={0};
    double costos[CI_MAX_SOLUCIONES]={0};
    printf("Inicio: B%+.3f H%+.3f C%.3f | Objetivo P=(%.3f,%.3f,%.3f) mm\n",
           ci_b_grados_estado(inicio),ci_h_grados_estado(inicio),
           ci_c_grados_estado(inicio),px,py,pz);
    for(int i=0;i<total;i++) {
        const ci_solucion_t *sol=&soluciones[i];
        if(!sol->limites_articulares_generales) {
            printf("SOLUCION %d DESCARTADA: topes articulares generales.\n",i+1);
            continue;
        }
        ci_estado_t fin=ci_cuantizar_estado(sol->base_oper_deg,
                                           sol->hombro_oper_deg,
                                           sol->codo_nominal_deg);
        if(!ci_topes_cuantizados(fin)) {
            printf("SOLUCION %d DESCARTADA: tope mecanico tras cuantizar.\n",i+1);
            continue;
        }
        // Recuperar analisis secuencial V5.19/20 COMO DIAGNOSTICO.
        // Una ruta sincronizada no necesita que haya una secuencial valida,
        // siempre que respete POR SI MISMA todas las restricciones.
        ci_analisis_t seq=ci_analizar_solucion(sol,inicio,px,py,pz);
        ci521_sincronizado_t sync=ci521_analizar_sincronizada(inicio,fin);
        if(!sync.compatible) {
            printf("SOLUCION %d DESCARTADA: trayectorias sincronizadas fallan.\n",i+1);
            for(int j=0;j<6;j++) if(sync.rutas[j].tiene_fallo) {
                printf("  Orden %s: %s\n",ci_nombres_orden[j],sync.rutas[j].motivo);
                ci_imprimir_fallo(&sync.rutas[j]);
                break;
            }
            continue;
        }
        printf("SOLUCION %d ADMISIBLE SEGUN MODELO: B%+.3f H%+.3f C%.3f\n",
               i+1,ci_b_grados_estado(fin),ci_h_grados_estado(fin),
               ci_c_grados_estado(fin));
        printf("  V5.19/20: %d/6 rutas secuenciales directas; sincronizada %s.\n",
               seq.rutas_directas_ok,ci_nombres_orden[sync.orden]);
        printf("  Cambio angular %.3f grados; calculo NO garantiza despeje 3D.\n",
               sync.costo);
        candidatas[i]=true;
        destinos[i]=fin;
        ordenes[i]=sync.orden;
        costos[i]=sync.costo;
        if (sync.costo<menor) {
            menor=sync.costo;
            mejor=i;
        }
    }
    if(mejor<0) {
        printf("SIN RUTA: topes/MOV2-4 no permiten el destino en el modelo.\n");
        printf("No se envio ningun pulso ni PWM. Introduce otras XYZ.\n");
        return;
    }
    printf("\n===== V5.23: ELEGIR SOLUCION DE CI =====\n");
    printf("RECOMENDADA POR CERCANIA: SOLUCION %d (%.3f grados).\n",
           mejor+1,menor);
    for(int i=0;i<total;i++) {
        if (!candidatas[i]) continue;
        printf("  %d) B%+.3f H%+.3f C%.3f | cambio %.3f | orden %s\n",
               i+1,ci_b_grados_estado(destinos[i]),
               ci_h_grados_estado(destinos[i]),ci_c_grados_estado(destinos[i]),
               costos[i],ci_nombres_orden[ordenes[i]]);
    }
    printf("ELIGE: a=RECOMENDADA | 1..4=SOLUCION MANUAL ADMISIBLE | n=CANCELAR\n");
    printf("       !=PARO CON TORQUE | x=PARO OFF\n");
    int elegida=mejor;
    for(;;) {
        char c=leer_char(); // leer_char() ya ejecuta paro_total() al recibir x.
        if(c=='x' || paro) return;
        if(c=='n' || c=='q') {
            printf("CANCELADO: sin ejecutar movimiento.\n");
            return;
        }
        if(c=='a') break;
        if(c>='1' && c<='4') {
            int idx=c-'1';
            if(idx<total && candidatas[idx]) {
                elegida=idx;
                break;
            }
            printf("SOLUCION %d NO AUTORIZABLE: topes o MOV2-4; elige otra.\n",
                   idx+1);
            continue;
        }
        printf("Responde a, numero de solucion valida, n, ! o x: ");
    }
    const ci_estado_t objetivo=destinos[elegida];
    const int orden=ordenes[elegida];
    const double costo=costos[elegida];
    // Revalidar antes de permitir autorizacion de actuadores.
    ci_estado_t nuevo_inicio={0};
    if (!ci_postura_actual(&nuevo_inicio) || !ci_igual(nuevo_inicio,inicio)) {
        printf("CANCELADO: las referencias o postura cambiaron antes de autorizar.\n");
        return;
    }
    ci_ruta_t ruta=ci521_probar_sincronizada(inicio,objetivo,orden);
    if(!ruta.compatible) {
        printf("SOLUCION SELECCIONADA RECHAZADA: %s\n",ruta.motivo);
        if(ruta.tiene_fallo) ci_imprimir_fallo(&ruta);
        return;
    }
    printf("\nPROPUESTA CONFIRMADA: SOLUCION %d | %s\n",elegida+1,
           ci_nombres_orden[orden]);
    printf("  B%+.3f H%+.3f C%.3f grados; cambio total %.3f.\n",
           ci_b_grados_estado(objetivo),ci_h_grados_estado(objetivo),
           ci_c_grados_estado(objetivo),costo);
    printf("  !/x se revisan DURANTE movimientos, espera y cierre de pinza.\n");
    printf("  AVISO: MOV1 inactivo, MOV2-4 parciales; sin deteccion 3D ni feedback PWM.\n");
    if (!servo_pinza.pwm_activo) {
        printf("  PINZA PWM OFF: al autorizar se ACTIVARA primero en CERRADA "
               "(%dus).\n",PINZA_CERRADA_US);
        printf("  IMPORTANTE: su posicion real es DESCONOCIDA; la primera "
               "activacion puede moverla.\n");
    } else {
        printf("  PINZA PWM ON: consigna actual %d us.\n",
               servo_pinza.us_actual);
    }
    printf("  DESPUES de llegar: ABRIR (%d us), mantener 1 s, CERRAR (%d us).\n",
           PINZA_ABIERTA_US,PINZA_CERRADA_US);
    printf("  Comprueba despeje del brazo y pinza ANTES de autorizar.\n");
    printf("AUTORIZAR MOVIMIENTO + CICLO PINZA: a=SI | n=NO | "
           "!=PARO TORQUE | x=PARO OFF\n");
    for(;;) {
        char c=leer_char(); // x ejecuta el paro inmediatamente.
        if(c=='x' || paro) return;
        if(c=='n' || c=='q') {
            printf("CANCELADO. Ningun movimiento XYZ autorizado.\n");
            return;
        }
        if(c=='a') break;
        printf("Responde a, n, ! o x: ");
    }
    if(comprobar_x()) return;
    if(!servo_pinza.pwm_activo) {
        // Activacion explicita autorizada: salida a 1500us (CERRADA).
        // No conocemos la posicion REAL anterior del MG90S.
        if(!servo_escribir_pwm(&servo_pinza,PINZA_CERRADA_US)) {
            printf("FALLO al activar pinza; se cancela movimiento XYZ.\n");
            if(!paro) paro_controlado("No se pudo preparar PWM de pinza");
            return;
        }
        printf("PINZA ARMADA EN CERRADA (%dus solicitados).\n",
               PINZA_CERRADA_US);
        if(!ci522_esperar_interrumpible_ms(300)) return;
    }
    // Exactamente misma ruta discreta que valida el planificador.
    if(!ci522_ejecutar(inicio,objetivo,orden)) {
        if(paro) printf("TRAYECTORIA CANCELADA. Solo HOME de recuperacion.\n");
        else printf("SIN MOVIMIENTO o ruta rechazada por modelo.\n");
        return; // No ejecutar la pinza si el desplazamiento fallo.
    }
    if(comprobar_x()) return;
    (void)ci523_ciclo_pinza_destino();

}

