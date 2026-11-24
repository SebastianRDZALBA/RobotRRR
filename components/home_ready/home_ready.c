/*
 * Componente home_ready: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "home_ready.h"

bool codo_postura_segura_prehome(void)
{
    // Esta pregunta es posterior a confirmar que el HOMBRO permite
    // TODO el desplazamiento del codo hasta HOME=500us (opcion B).
    for (;;) {
        printf("\nPRIMER HOME - PASO 2: el CODO puede dirigirse hacia HOME=500us?\n");
        printf("s=SI, activar 500us | n=activar 500us y abrir ajuste +/-10 grados\n");
        printf("q=cancelar | x=PARO\n");
        char c=leer_char();
        if (c=='q' || c=='x') return false;
        if (c!='s' && c!='n') continue;
        if (!servo_codo.pwm_activo) {
            printf("PRIMER PWM CODO: 500us. Movimiento inicial puede ser brusco.\n");
            if (!servo_escribir_pwm(&servo_codo,CODO_HOME_US)) return false;
            codo_angulo_objetivo=0.0;
            registrar_codo("ACTIVACION A HOME PREVIA A HOMING",-1,0.0);
        } else if (servo_codo.us_actual != CODO_HOME_US) {
            int anterior=servo_codo.us_actual;
            if (!codo_mover_home_ready_gradual(CODO_HOME_US)) return false;
            codo_angulo_objetivo=0.0;
            registrar_codo("REUBICACION HOME PREVIA A HOMING",anterior,
                           codo_grados_de_pwm(CODO_HOME_US)-codo_grados_de_pwm(anterior));
        }
        if (c=='n') {
            printf("CODO ya recibe 500us. Modo de acomodo manual +/-10 grados.\n");
            servo_menu_manual(&servo_codo,true);
            if (paro) return false;
            if (!confirmar("CODO listo y despejado para continuar al HOME BASE?"))
                continue;
        }
        return true;
    }
}

bool codo_postura_segura_para_home_hombro(void)
{
    for (;;) {
        printf("\nTras HOME BASE: el CODO permite bajar HOMBRO a microswitch?\n");
        printf("s=SI | n=ajustar CODO manualmente | q=salir | x=PARO\n");
        char c=leer_char();
        if (c=='s') {printf("UI:CONFIRMADO Codo despejado para HOME hombro.\n");return true;}
        if (c=='q' || c=='x') return false;
        if (c=='n') servo_menu_manual(&servo_codo,true);
    }
}

bool codo_postura_segura_para_retorno(void)
{
    for (;;) {
        printf("\nRETORNO A HOME: CODO permite llevar HOMBRO a READY y CODO a 1500us?\n");
        printf("s=SI | n=ajustar codo manual +/-10 grados | q=cancelar | x=PARO\n");
        char c=leer_char();
        if (c=='s') return true;
        if (c=='q' || c=='x') return false;
        if (c=='n') servo_menu_manual(&servo_codo,true);
    }
}

bool despeje_hombro_5_si_switch(void)
{
    bool venia_en_switch = switch_estable(hombro.sw,1);
    printf("\nPASO 3: elevacion adicional HOMBRO +5.025 grados (67 micropasos), UNA VEZ.\n");
    printf("Anteriormente SW HOMBRO=%d. x=PARO.\n", venia_en_switch?1:0);
    habilitar(&hombro);
    for (int i=0;i<67;++i) {
        if (!paso(&hombro,HOMBRO_DIR_ALEJAR,PASO_HOMBRO_AUTO_US)) return false;
        if ((i%32)==31) vTaskDelay(1);
    }
    if (venia_en_switch && !switch_estable(hombro.sw,0)) {
        printf("ERROR: no libero SW HOMBRO al levantar 5 grados. No girar BASE.\n");
        return false;
    }
    printf("Despeje inicial +5 grados completado. No se repetira durante este HOME.\n");
    return true;
}

bool postura_segura_prehome(void)
{
    printf("\nPRIMER HOME - PASO 1: HOMBRO en posicion segura PARA EL MISMO Y PARA EL CODO?\n");
    printf("s=SI | n=NO, despejar hombro | q=salir | x=PARO\n");
    char c;
    for (;;) {
        c = leer_char();
        if (c == 'q' || c == 'x') return false;
        if (c == 's') {printf("UI:CONFIRMADO Hombro seguro segun operador.\n");return true;}
        if (c == 'n') break;
        printf("Usa s, n, q o x.\n");
    }

    int32_t origen = hombro_rel_pulsos;
    printf("UI:HOME_AJUSTE_HOMBRO_INICIO\n");
    printf("\n===== AJUSTE DE POSICION SEGURA =====\n");
    printf("r/l: hombro +/-10 grados repetibles, sin confirmacion.\n");
    printf("s: ya esta seguro, continuar con HOME BASE | n: seguir ajustando\n");
    printf("p: estado | q: salir | x: PARO\n");
    for (;;) {
        printf("UI:HOME_AJUSTE_HOMBRO_LISTO\n");
        printf("HOMBRO relativo=%+.3f deg | SW=%d | comando: ",
               grados_hombro(hombro_rel_pulsos - origen), gpio_get_level(hombro.sw));
        c = leer_char();
        if (c == 'q' || c == 'x') {
            printf("UI:HOME_AJUSTE_HOMBRO_FIN\n");
            return false;
        }
        if (c == 'p' || c == 'n') continue;
        if (c == 's') {
            printf("UI:HOME_AJUSTE_HOMBRO_FIN\n");
            printf("Usuario indico postura segura. Continuar con HOME BASE.\n");
            return true;
        }
        if (c != 'r' && c != 'l') {
            printf("Usa r, l, p, s, n, q o x.\n");
            continue;
        }
        if (!mover_hombro_despeje(c == 'r' ? HOMBRO_DIR_ALEJAR : HOMBRO_DIR_HOME,
                                  origen)) {
            printf("UI:HOME_AJUSTE_HOMBRO_FIN\n");
            return false;
        }
    }
}

bool pasar_a_preparado(void)
{
    if (!base_home || !hombro_home || base_pulsos != 0 || paro || sistema_cerrado) {
        printf("PREPARADO exige HOME BASE y HOMBRO validos, BASE a cero.\n");
        return false;
    }
    if (!servo_codo.pwm_activo) {
        printf("CODO no activo. Primero PRIMER HOME para activar HOME=500us.\n");
        return false;
    }
    if (hombro_rel_pulsos != 0 || !switch_estable(hombro.sw,1)) {
        printf("HOME -> PREPARADO solo desde HOMBRO HOME (0 pulsos, SW=1).\n");
        return false;
    }
    if (servo_codo.us_actual != CODO_HOME_US) {
        printf("HOME -> PREPARADO exige CODO plegado en 500us.\n");
        return false;
    }
    printf("\n===== HOME -> PREPARADO AUTOMATICO =====\n");
    printf("1) HOMBRO HOME -> +1320 pulsos.\n");
    printf("2) CODO HOME 500us -> READY %dus.\n", servo_codo.ready_us);
    printf("3) BASE: referencia operativa a 0, SIN girar fisicamente.\n");
    printf("Usuario verifico espacio libre para esta transicion. x=PARO.\n");
    bool ok=false;
    transicion_coordinada=true;
    if (!hombro_mover_a(HOMBRO_READY_PULSOS,true)) goto fin;
    if (!switch_estable(hombro.sw,0)) {
        printf("ERROR: SW hombro activo en READY. No extender codo.\n");
        hombro_home=false; hombro_preparado=false; preparado_base=false;
        goto fin;
    }
    {
        int anterior=servo_codo.us_actual;
        if (!codo_mover_home_ready_gradual(servo_codo.ready_us)) goto fin;
        codo_angulo_objetivo=codo_grados_de_pwm(servo_codo.ready_us);
        registrar_codo("AUTOMATICO HOME A READY",anterior,
                       codo_grados_de_pwm(servo_codo.ready_us)-codo_grados_de_pwm(anterior));
    }
    preparado_base=true;
    ok=true;
    printf("\n*** MODO PREPARADO ACTIVADO ***\n");
    buzzer_sonar(BUZZER_READY);
    printf("HOMBRO: 1320 pulsos (0 operativo); CODO: %dus; BASE: 0 operativo.\n",
           servo_codo.us_actual);
fin:
    transicion_coordinada=false;
    if (!ok) printf("TRANSICION HOME->PREPARADO INCOMPLETA. Revisar postura.\n");
    return ok;
}

bool robot_a_preparado(void)
{
    if (paro || sistema_cerrado || !base_home || !hombro_home ||
        !servo_codo.pwm_activo) {
        printf("PREPARADO requiere referencia valida y PWM activo del codo.\n");
        return false;
    }
    if (hombro_rel_pulsos==0 && base_pulsos==0 &&
        servo_codo.us_actual==CODO_HOME_US)
        return pasar_a_preparado();
    if (!confirmar("REGRESAR A PREPARADO: recorrido HOMBRO->READY, CODO->READY, BASE->0 despejado?"))
        return false;
    printf("\n===== RETORNO A PREPARADO DESDE POSICION REFERENCIADA =====\n");
    bool ok=false;
    transicion_coordinada=true;
    if (!hombro_mover_a(HOMBRO_READY_PULSOS,true)) goto fin;
    {
        int anterior=servo_codo.us_actual;
        if (!codo_mover_home_ready_gradual(servo_codo.ready_us)) goto fin;
        codo_angulo_objetivo=codo_grados_de_pwm(servo_codo.ready_us);
        registrar_codo("REGRESO A PREPARADO",anterior,
            codo_grados_de_pwm(servo_codo.ready_us)-codo_grados_de_pwm(anterior));
    }
    if (base_pulsos!=0 && !mover_base_a(0.0)) goto fin;
    preparado_base=true;
    printf("PREPARADO: HOMBRO 1320, CODO %d us, BASE 0.\n",servo_codo.us_actual);
    ok=true;
    buzzer_sonar(BUZZER_READY);
fin:
    transicion_coordinada=false;
    if (!ok) printf("RETORNO A PREPARADO INTERRUMPIDO.\n");
    return ok;
}

bool robot_a_home(void)
{
    if (paro || sistema_cerrado || !base_home || !hombro_home ||
        !servo_codo.pwm_activo) {
        printf("RETORNO HOME requiere HOME BASE/HOMBRO previos y codo con PWM activo.\n");
        printf("Si hubo perdida de referencias, realiza PRIMER HOME.\n");
        return false;
    }
    printf("\n===== DESDE POSICION REFERENCIADA A HOME =====\n");
    if (!codo_postura_segura_para_retorno()) return false;
    if (!confirmar("HOMBRO y CODO: recorrido hacia READY y despues HOME despejado?"))
        return false;
    bool ok=false;
    transicion_coordinada=true;
    printf("PASO II: HOMBRO a arranque READY (+1320 pasos).\n");
    if (!hombro_mover_a(HOMBRO_READY_PULSOS,true)) goto fin;
    printf("PASO III: CODO a arranque READY (1500us).\n");
    {
        int anterior=servo_codo.us_actual;
        if (!codo_mover_home_ready_gradual(servo_codo.ready_us)) goto fin;
        codo_angulo_objetivo=codo_grados_de_pwm(servo_codo.ready_us);
        registrar_codo("ANTES HOME BASE: CODO READY",anterior,
                       codo_grados_de_pwm(servo_codo.ready_us)-codo_grados_de_pwm(anterior));
    }
    printf("PASO IV: BASE a 0 y repetir HOME BASE calibrado.\n");
    if (base_pulsos!=0 && !mover_base_a(0.0)) goto fin;
    if (!home_base()) goto fin;
    printf("PASO V: CODO hacia HOME 500us plegado.\n");
    {
        int anterior=servo_codo.us_actual;
        if (!codo_mover_home_ready_gradual(CODO_HOME_US)) goto fin;
        codo_angulo_objetivo=0;
        registrar_codo("RETORNO CODO A HOME",anterior,
                       codo_grados_de_pwm(CODO_HOME_US)-codo_grados_de_pwm(anterior));
    }
    printf("PASO VI: HOMBRO hacia HOME mecanico.\n");
    if(!confirmar("Iniciar retorno del HOMBRO a HOME mecanico? Recorrido despejado?"))
        goto fin;
    if (!regresar_hombro_a_home()) goto fin;
    ok=true;
    printf("HOME TOTAL TERMINADO: BASE=0 HOMBRO=SW1 CODO=500us.\n");
fin:
    transicion_coordinada=false;
    if (!ok) {
        printf("HOME INCOMPLETO: no apagar actuadores por secuencia automatica.\n");
        return false;
    }
    preguntar_preparado_despues_home();
    return true;
}

void preguntar_preparado_despues_home(void)
{
    if (paro || sistema_cerrado) return;
    bool fisicamente_home = base_home && hombro_home && base_pulsos==0 &&
                            hombro_rel_pulsos==0 && switch_estable(hombro.sw,1) &&
                            servo_codo.pwm_activo &&
                            servo_codo.us_actual==CODO_HOME_US;
    if (!fisicamente_home) {
        printf("ERROR: no se confirma HOME de todas las articulaciones.\n");
        return;
    }
    printf("\n====== HOME TOTAL COMPLETADO ======\n");
    buzzer_sonar(BUZZER_HOME);
    printf("BASE=0 | HOMBRO HOME SW=1 | CODO plegado=500us.\n");
    printf("s=IR A PREPARADO | a=SALIR (DESHABILITAR MOTORES) | x=PARO\n");
    for (;;) {
        char c=leer_char();
        if (c=='x' || paro) return;
        if (c=='s') {
            if (pasar_a_preparado()) menu_preparado();
            return;
        }
        if (c=='a') {
            if (!confirmar("SALIR: hombro realmente sobre tope, codo plegado y brazo sostenido?"))
                continue;
            // Si la pinza estaba activa, dejarla cerrada antes de cortar PWM.
            if (servo_pinza.pwm_activo &&
                servo_pinza.us_actual!=PINZA_CERRADA_US) {
                if (!confirmar("La PINZA puede CERRARSE hacia 1500us antes del apagado?"))
                    continue;
                if (!servo_mover_pwm_directo(&servo_pinza,PINZA_CERRADA_US))
                    return;
            }
            gpio_set_level(base.en,1);
            gpio_set_level(hombro.en,1);
            gpio_set_level(base.step,0);
            gpio_set_level(hombro.step,0);
            servos_detener_senal();
            base_driver_habilitado=false;
            base_home=false;
            hombro_home=false;
            hombro_preparado=false;
            preparado_base=false;
            sistema_cerrado=true;
            gpio_set_level(BUZZER_GPIO,0);
            printf("\nSALIDA SEGURA LOGICA: drivers NEMA OFF y PWM servos OFF.\n");
            printf("UI:SALIDA_SEGURA NEMA_OFF PWM_OFF REFERENCIAS_INVALIDAS\n");
            printf("NO CORTA FISICAMENTE 5V/12V. Desconectar fuentes manualmente.\n");
            printf("Referencias por pasos invalidadas; reiniciar ESP32 para operar.\n");
            return;
        }
        printf("Usa s, a o x.\n");
    }
}

bool primer_home(void)
{
    if (paro || sistema_cerrado) return false;
    printf("\n=========== PRIMER HOME V5.6 ===========\n");
    printf("1) Comprobar HOMBRO seguro tanto para HOMBRO como CODO.\n");
    if (!postura_segura_prehome()) return false;
    printf("2) Comprobar CODO y ACTIVAR DIRECTAMENTE A HOME=500us.\n");
    if (!codo_postura_segura_prehome()) return false;
    printf("3) Elevar HOMBRO +5grados exactamente una vez.\n");
    if (!despeje_hombro_5_si_switch()) return false;
    // La busqueda mecanica no debe heredar HOME de hombro anterior.
    hombro_home=false;
    hombro_preparado=false;
    preparado_base=false;
    printf("4) HOME BASE con procedimiento anterior y confirmacion de direccion.\n");
    if (!home_base()) {
        printf("ERROR: HOME BASE incompleto; no iniciar HOME HOMBRO.\n");
        return false;
    }
    printf("5) Confirmar codo seguro para bajar el hombro.\n");
    if (!codo_postura_segura_para_home_hombro()) return false;
    printf("6) HOME automatico HOMBRO, rutina original.\n");
    if (!confirmar("Iniciar HOME del HOMBRO ahora? Confirma que el recorrido esta despejado."))
        return false;
    if (!home_automatico_hombro()) {
        printf("ERROR: HOME HOMBRO incompleto.\n");
        return false;
    }
    printf("7) HOME CODO a 500us con HOMBRO ya sobre su tope.\n");
    if (!servo_codo.pwm_activo) {
        printf("ERROR: CODO sin PWM; HOME total no valido.\n");
        return false;
    }
    {
        int anterior=servo_codo.us_actual;
        if (!codo_mover_home_ready_gradual(CODO_HOME_US)) return false;
        codo_angulo_objetivo=0.0;
        registrar_codo("FIN PRIMER HOME: CODO PLEGADO",anterior,
            codo_grados_de_pwm(CODO_HOME_US)-codo_grados_de_pwm(anterior));
    }
    preguntar_preparado_despues_home();
    return true;
}

