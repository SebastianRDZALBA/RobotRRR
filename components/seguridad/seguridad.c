/*
 * Componente seguridad: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "seguridad.h"

double m1_interpolar(double x, double x0, double y0,
                             double x1, double y1)
{
    return y0 + (x-x0)*(y1-y0)/(x1-x0);
}

bool m1_base_config_critica(int32_t h, double c)
{
    return h >= M1_BASE_H_MIN && h <= M1_BASE_H_MAX &&
           fabs(c-M1_BASE_C_CENTRO) <= M1_BASE_C_TOL;
}

bool m1_recorrido_base_admisible(int32_t h, double c,
                                        double desde, double hasta)
{
    if (!m1_base_config_critica(h,c)) return true;
    const double e=PERFIL_TOLERANCIA_ANGULO;
    if (desde > M1_P1_BASE_MAX+e) {
        if (hasta < desde-e && hasta >= M1_P1_BASE_MIN-e) return true;
        printf("RESTRICCION MOV 1 BASE: fuera por derecha; solo regresar hacia [-51,+2.5].\n");
        return false;
    }
    if (desde < M1_P1_BASE_MIN-e) {
        if (hasta > desde+e && hasta <= M1_P1_BASE_MAX+e) return true;
        printf("RESTRICCION MOV 1 BASE: fuera por izquierda; solo regresar hacia [-51,+2.5].\n");
        return false;
    }
    if (hasta < M1_P1_BASE_MIN-e || hasta > M1_P1_BASE_MAX+e) {
        printf("RESTRICCION MOV 1 BASE: H/C criticos. BASE permitido [-51,+2.5]; pedido %+.3f.\n",hasta);
        return false;
    }
    return true;
}

double m1_max_h_por_c(double c)
{
    if (c < M1_P1_C-M1_TOL_REFERENCIA_C)
        return M1_P6_P7_H; // CODO mas levantado: no inventar limite de H
    if (c <= M1_P2_C)
        return m1_interpolar(c,M1_P1_C,M1_P1_H,M1_P2_C,M1_P2_H);
    if (c < M1_P3_P4_C-M1_TOL_REFERENCIA_C)
        return m1_interpolar(c,M1_P2_C,M1_P2_H,M1_P3_P4_C,M1_P3_H);
    if (c < M1_P7_C)
        return M1_P4_P5_H;  // P3->P4, despues P4->P5
    if (c <= M1_P5_C+PERFIL_TOLERANCIA_ANGULO)
        return m1_interpolar(c,M1_P7_C,M1_P6_P7_H,
                               M1_P5_C,M1_P4_P5_H);
    // Fuera del extremo medido de codo: no ampliar la frontera.
    return (double)M1_P3_H-1.0;
}

double m1_max_c_por_h(int32_t h)
{
    if (h < M1_P3_H) return 180.0;
    if (h < M1_P4_P5_H) return M1_P3_P4_C; // P3->P4
    if (h == M1_P4_P5_H) return M1_P5_C;  // P4->P5
    return m1_interpolar((double)h, (double)M1_P4_P5_H,M1_P5_C,
                          (double)M1_P6_P7_H,M1_P7_C);
}

bool m1_hombro_derecha_valido(int32_t h, double b, double c)
{
    if (m1_base_config_critica(h,c) &&
        (b < M1_P1_BASE_MIN-PERFIL_TOLERANCIA_ANGULO ||
         b > M1_P1_BASE_MAX+PERFIL_TOLERANCIA_ANGULO))
        return false;
    if (h < M1_P3_H) return true;
    if (h == M1_P6_P7_H && c <= M1_P7_C+PERFIL_TOLERANCIA_ANGULO)
        return true;
    if ((double)h > m1_max_h_por_c(c)+0.00001) return false;
    if (c > m1_max_c_por_h(h)+PERFIL_TOLERANCIA_ANGULO) return false;
    return true;
}

bool m1_comprobar_hombro_derecha(int32_t h, double b, double c)
{
    if (m1_hombro_derecha_valido(h,b,c)) return true;
    printf("RESTRICCION MOV 1 HOMBRO DERECHA: H=%+.3f C=%.3f B=%+.3f.\n",
           (double)(h-HOMBRO_READY_PULSOS)*360.0/PULSOS_VUELTA_HOMBRO,
           c,b);
    if (m1_base_config_critica(h,c))
        printf("  Configuracion critica de BASE: -51.000 a +2.500 grados.\n");
    else
        printf("  Limite HOMBRO/CODO al avanzar DERECHA.\n");
    printf("  Puede regresar el HOMBRO a IZQUIERDA para recuperar.\n");
    return false;
}

bool perfil_v2_activo(void)
{
    return hombro_home && hombro_rel_pulsos == PERFIL_HOMBRO_CRITICO_PULSOS;
}

double perfil_max_codo_en_base(double b)
{
    if ((b > PERFIL_BASE_CAMBIO_CENTRAL + PERFIL_TOLERANCIA_ANGULO &&
         b < PERFIL_BASE_CAMBIO_POSITIVO - PERFIL_TOLERANCIA_ANGULO) ||
        b < PERFIL_BASE_CAMBIO_NEGATIVO - PERFIL_TOLERANCIA_ANGULO)
        return PERFIL_CODO_MAX_DESPEJADO;
    return PERFIL_CODO_MAX_EXTENDIDO;
}

bool restricciones_postura_evaluar(int32_t h_pulsos,
                                          double b_grados,
                                          double c_grados,
                                          bool evaluar_m1)
{
    if (!hombro_home) return true;
    if (MOV1_ACTIVO && evaluar_m1 &&
        !m1_comprobar_hombro_derecha(h_pulsos,b_grados,c_grados))
        return false;
    if (h_pulsos == PERFIL_HOMBRO_M2_PULSOS &&
        c_grados < PERFIL_M2_CODO_MIN - PERFIL_TOLERANCIA_ANGULO) {
        printf("RESTRICCION MOV 2: HOMBRO +80.100; CODO %.3f no puede bajar de su limite 1.980.\n",
               c_grados);
        return false;
    }
    if (h_pulsos == PERFIL_HOMBRO_CRITICO_PULSOS) {
        const double maximo=perfil_max_codo_en_base(b_grados);
        if (c_grados > maximo + PERFIL_TOLERANCIA_ANGULO) {
            printf("RESTRICCION MOV 4: BASE %+.3f HOMBRO -89.250; CODO %.3f excede maximo %.3f.\n",
                   b_grados,c_grados,maximo);
            return false;
        }
    }
    return true;
}

bool restricciones_postura_admisible(int32_t h_pulsos,
                                          double b_grados,
                                          double c_grados)
{
    return restricciones_postura_evaluar(h_pulsos,b_grados,c_grados,false);
}

bool restricciones_recorrido_hombro_admisible(int32_t desde,
                                                    int32_t hasta,
                                                    double b_grados,
                                                    double c_grados)
{
    if (!hombro_home) return true;
    // MOV1 inactivo: la comprobacion de su perfil se omite.
    // MOV2/MOV4 siguen activos en ambos sentidos.
    const bool entrar_derecha=(hasta > desde);
    if (!restricciones_postura_evaluar(hasta,b_grados,c_grados,
                                       entrar_derecha)) return false;
    if (MOV1_ACTIVO && entrar_derecha) {
        for (int32_t p=desde+1; p<=hasta; ++p)
            if (!m1_comprobar_hombro_derecha(p,b_grados,c_grados)) return false;
    }
    // MOV 2: conservar sin cambios la proteccion V5.13.
    if (desde >= PERFIL_HOMBRO_M2_PULSOS &&
        hasta < PERFIL_HOMBRO_M2_PULSOS &&
        hasta >= HOMBRO_READY_PULSOS &&
        c_grados <= PERFIL_M2_CODO_MIN + PERFIL_TOLERANCIA_ANGULO) {
        printf("RESTRICCION MOV 2: HOMBRO no puede bajar de +80.100 ");
        printf("dentro de la zona DERECHA con CODO %.3f.\n",c_grados);
        printf("Primero eleva el codo. La region IZQUIERDA usa MOV 3/4.\n");
        return false;
    }
    return true;
}

bool restricciones_recorrido_codo_admisible(int desde_us, int hasta_us,
                                                 double b_grados)
{
    (void)desde_us;
    const double c_final = (double)(hasta_us-CODO_HOME_US)*180.0/
           (double)(CODO_PWM_CAL_MAX_US-CODO_HOME_US);
    return restricciones_postura_evaluar(hombro_rel_pulsos,b_grados,
                                        c_final,false);
}

bool perfil_codo_admisible(double b, double c)
{
    return restricciones_postura_admisible(hombro_rel_pulsos,b,c);
}

bool perfil_recorrido_base_admisible(double desde, double hasta)
{
    if (!perfil_v2_activo()) return true;
    if (!servo_codo.pwm_activo) {
        printf("RESTRICCION MOV 4: CODO sin PWM, posicion desconocida. No girar BASE.\n");
        return false;
    }
    const double codo = (double)(servo_codo.us_actual-CODO_HOME_US)*180.0 /
          (double)(CODO_PWM_CAL_MAX_US-CODO_HOME_US);
    double minimo=fmin(desde,hasta), maximo=fmax(desde,hasta);
    if (codo > PERFIL_CODO_MAX_EXTENDIDO + PERFIL_TOLERANCIA_ANGULO) {
        printf("RESTRICCION MOV 4: CODO %.3f supera maximo extendido %.3f.\n",
               codo,PERFIL_CODO_MAX_EXTENDIDO);
        return false;
    }
    bool cruza_centro=maximo > PERFIL_BASE_CAMBIO_CENTRAL + PERFIL_TOLERANCIA_ANGULO &&
                       minimo < PERFIL_BASE_CAMBIO_POSITIVO - PERFIL_TOLERANCIA_ANGULO;
    bool cruza_extremo_negativo=minimo < PERFIL_BASE_CAMBIO_NEGATIVO - PERFIL_TOLERANCIA_ANGULO;
    if (codo > PERFIL_CODO_MAX_DESPEJADO + PERFIL_TOLERANCIA_ANGULO &&
        (cruza_centro || cruza_extremo_negativo)) {
        printf("RESTRICCION MOV 4: BASE %+.3f -> %+.3f con CODO %.3f NO permitida.\n",
               desde,hasta,codo);
        printf("El recorrido atraviesa una zona que requiere CODO <=153.000.\n");
        return false;
    }
    return perfil_codo_admisible(hasta,codo);
}

void mostrar_referencias_mecanicas_v2(void)
{
    printf("\n===== LIMITES: MOV1 INACTIVO; MOV2,3,4 VIGENTES =====\n");
    printf("MOV 1: DATOS HISTORICOS V3; NO se aplican como restricciones.\n");
    printf("  P1 H+127.500 C90.000; BASE -51..+2.5 con H+126..+127.5 C90+/-1.\n");
    printf("  P2 H+127.275 C93.960; P3 H+126.000 C99.000.\n");
    printf("  P4 H+129.150 C99.000; P5 H+129.150 C108.540.\n");
    printf("  P6 H+131.025: SOLO tope H; C108.540 accidental/no es limite.\n");
    printf("  P7 H+131.025 C101.520: referencia del limite para mover HOMBRO.\n");
    printf("  +HOMBRO: limitar avance derecha segun CODO; -HOMBRO: recuperar.\n");
    printf("  MOV1 DESACTIVADO para BASE, HOMBRO y validacion de rutas.\n");
    printf("  Al mover CODO siguen activos MOV2, MOV4 y topes de PWM.\n");
    printf("  OJO: interpolacion por tramos, SIN garantia de anti-colision.\n");
    printf("MOV 2: HOMBRO +80.100 (2388 pasos): CODO >=1.980 nominal.\n");
    printf("       Descenso con CODO <=1.980 bloqueado solo en zona DERECHA.\n");
    printf("MOV 3: HOMBRO -89.250, CODO 0: DESPEJE confirmado.\n");
    printf("MOV 4: HOMBRO -89.250 (reglas por sectores SIN CAMBIOS):\n");
    printf("  BASE [+84,+105] y [-95,-27.5]: CODO <=176.490.\n");
    printf("  BASE (-27.5,+84) y [-140,-95): CODO <=153.000.\n");
    printf("  En +84,-27.5,-95 se permite cambiar postura del codo.\n");
    printf("  Se comprueba el recorrido completo al girar la base.\n");
    printf("BASE general: -140 nominal (-140.025 por pulsos) a +105 grados. HOME=0.\n");
}

bool ci_postura_m2_m4(ci_estado_t e,const char **motivo)
{
    const double b=ci_b_grados_estado(e),c=ci_c_grados_estado(e);
    if (e.h_pulsos==PERFIL_HOMBRO_M2_PULSOS &&
        c<PERFIL_M2_CODO_MIN-PERFIL_TOLERANCIA_ANGULO) {
        *motivo="MOV2: relacion hombro/codo";
        return false;
    }
    if (e.h_pulsos==PERFIL_HOMBRO_CRITICO_PULSOS &&
        c>perfil_max_codo_en_base(b)+PERFIL_TOLERANCIA_ANGULO) {
        *motivo="MOV4: codo excedido para sector de base";
        return false;
    }
    return true;
}

bool ci_base_mov1(int32_t h,double c,double desde,double hasta)
{
    if (!m1_base_config_critica(h,c)) return true;
    const double t=PERFIL_TOLERANCIA_ANGULO;
    if (desde>M1_P1_BASE_MAX+t)
        return hasta<desde-t && hasta>=M1_P1_BASE_MIN-t;
    if (desde<M1_P1_BASE_MIN-t)
        return hasta>desde+t && hasta<=M1_P1_BASE_MAX+t;
    return hasta>=M1_P1_BASE_MIN-t && hasta<=M1_P1_BASE_MAX+t;
}

void invalidar_referencias_521(void)
{
    base_home=false;
    hombro_home=false;
    hombro_preparado=false;
    preparado_base=false;
    frente_provisional=false;
    // Contadores no se consideran posicion valida despues del paro.
    base_objetivo_grados=0.0;
    hombro_provisional=false;
}

void paro_total(void)
{
    paro=true;
    ui_watchdog_activo=false;
    gpio_set_level(BUZZER_GPIO, 0);
    ci521_paro_suave_ocurrio=false;
    gpio_set_level(base.step,0);
    gpio_set_level(hombro.step,0);
    gpio_set_level(base.en,1);
    gpio_set_level(hombro.en,1);
    servos_detener_senal();
    base_driver_habilitado=false;
    invalidar_referencias_521();
    base_pulsos=0;
    hombro_rel_pulsos=0;
    printf("\n[PARO x] NEMA OFF y PWM OFF. TODAS LAS TRAYECTORIAS CANCELADAS.\n");
    printf("PUEDE HABER CAIDA POR GRAVEDAD. HOME supervisado obligatorio.\n");
}

void paro_controlado(const char *motivo)
{
    if(paro) return;
    paro=true;
    ui_watchdog_activo=false;
    gpio_set_level(BUZZER_GPIO, 0);
    ci521_paro_suave_ocurrio=true;
    gpio_set_level(base.step,0);
    gpio_set_level(hombro.step,0);
    // NO cambiar niveles EN: conservan el torque solo si ya estaban ON.
    // NO desactivar LEDC: mantiene ultimo PWM pedido.
    invalidar_referencias_521();
    printf("\n[PARO CONTROLADO] %s\n",motivo?motivo:"Paro controlado solicitado");
    printf("Pulsos detenidos; drivers y PWM conservan ultimo estado electrico.\n");
    printf("Posiciones reales no confirmadas: HOME supervisado obligatorio.\n");
}

bool comprobar_x(void)
{
    if(paro) return true;
    uint8_t c;
    for(int i=0;i<32;i++) {
        if(uart_read_bytes(UART_NUM_0,&c,1,0)<=0) break;
        if(c=='~') {
            ui_ping_recibido=true;
            ui_ultimo_ping=xTaskGetTickCount();
            continue;
        }
        if(c==0x03) {
            paro_controlado("PARO controlado solicitado desde interfaz USB");
            return true;
        }
        if(c=='x' || c=='X') {
            paro_total();
            return true;
        } else if(c=='!') {
            paro_controlado("Tecla ! recibida durante movimiento");
            return true;
        }
        // Otros caracteres se descartan MIENTRAS hay movimiento.
    }
    if(ui_watchdog_activo && !paro &&
       (TickType_t)(xTaskGetTickCount()-ui_ultimo_ping) > pdMS_TO_TICKS(UI_WATCHDOG_MS)) {
        paro_controlado("Sin heartbeat USB durante movimiento remoto");
        return true;
    }
    return paro;
}

