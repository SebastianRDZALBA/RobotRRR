/*
 * Componente servos: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "servos.h"

double codo_grados_de_pwm(int us)
{
    return (double)(us - CODO_HOME_US) * 180.0 /
           (double)(CODO_PWM_CAL_MAX_US - CODO_HOME_US);
}

int codo_pwm_de_grados(double grados)
{
    return CODO_HOME_US + (int)lround(grados *
           (double)(CODO_PWM_CAL_MAX_US-CODO_HOME_US)/180.0);
}

void registrar_codo(const char *motivo, int anterior, double grados_solicitados)
{
    ++num_movimientos;
    if (anterior < 0)
        printf("\n[REGISTRO #%05lu] %s | CODO | inicio sin PWM previo\n",
               num_movimientos, motivo);
    else
        printf("\n[REGISTRO #%05lu] %s | CODO | %+0.3f grados nominales solicitados\n",
               num_movimientos, motivo, grados_solicitados);
    if (anterior < 0)
        printf("  PWM: OFF -> %d us (posicion fisica inicial DESCONOCIDA)\n",
               servo_codo.us_actual);
    else
        printf("  PWM %d -> %d us | posicion nominal objetivo %.3f grados\n",
               anterior, servo_codo.us_actual, codo_angulo_objetivo);
    printf("  HOMBRO: %+.3f grados operativos / %ld pulsos | BASE: %+.3f grados\n",
           (double)(hombro_rel_pulsos-HOMBRO_READY_PULSOS)*360.0/
              PULSOS_VUELTA_HOMBRO, (long)hombro_rel_pulsos,
           (double)base_pulsos*360.0/PULSOS_VUELTA_BASE);
    printf("  Resultado: consigna PWM enviada, SIN sensor de posicion de codo.\n");
}

int servo_min_us(const servo_t *s)
{
    return (s == &servo_codo) ? CODO_PWM_CAL_MIN_US : PINZA_PWM_CAL_MIN_US;
}

int servo_max_us(const servo_t *s)
{
    return (s == &servo_codo) ? CODO_PWM_CAL_MAX_US : PINZA_PWM_CAL_MAX_US;
}

bool servo_us_admisible(const servo_t *s, int us)
{
    return us >= servo_min_us(s) && us <= servo_max_us(s);
}

uint32_t servo_duty_de_us(int us)
{
    return (uint32_t)(((uint64_t)us * (1ULL << 16) * SERVO_PWM_HZ + 500000ULL) / 1000000ULL);
}

void servos_iniciar(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = SERVO_PWM_MODE,
        .duty_resolution = SERVO_PWM_RES,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t canales[2] = {
        {.gpio_num = CODO_PWM_GPIO, .speed_mode = SERVO_PWM_MODE,
         .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = SERVO_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = PINZA_PWM_GPIO, .speed_mode = SERVO_PWM_MODE,
         .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = SERVO_TIMER, .duty = 0, .hpoint = 0}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&canales[0]));
    ESP_ERROR_CHECK(ledc_channel_config(&canales[1]));
    // duty=0 significa salida baja, sin pulsos al encender.
    // No llamar ledc_stop aqui: despues queremos activar con set_duty.
    servos_pwm_inicializado = true;
}

void servos_detener_senal(void)
{
    if (servos_pwm_inicializado) {
        (void)ledc_stop(SERVO_PWM_MODE, servo_codo.canal, 0);
        (void)ledc_stop(SERVO_PWM_MODE, servo_pinza.canal, 0);
    }
    servo_codo.pwm_activo = false;
    servo_pinza.pwm_activo = false;
    modo_preparado_codo_calibrado = false;
}

bool servo_escribir_pwm(servo_t *s, int microsegundos)
{
    if (paro || !servo_us_admisible(s, microsegundos))
        return false;
    // CODO: MOV1 inactivo; conservar comprobaciones MOV2 y MOV4.
    // Solo MOV2/MOV4 y topes propios PWM, sin tocar las reglas de +H.
    // Al activar desde PWM OFF se comprueba SOLO el destino (no se conoce
    // el angulo fisico actual del servo); el primer HOME es explicito.
    if (s == &servo_codo && base_home && hombro_home) {
        if (s->pwm_activo) {
            if (!restricciones_recorrido_codo_admisible(s->us_actual,
                    microsegundos,base_objetivo_grados)) return false;
        } else {
            const double c=codo_grados_de_pwm(microsegundos);
            if (!restricciones_postura_admisible(hombro_rel_pulsos,
                                                  base_objetivo_grados,c)) return false;
        }
    }
    esp_err_t e = ledc_set_duty(SERVO_PWM_MODE, s->canal,
                                 servo_duty_de_us(microsegundos));
    if (e == ESP_OK) e = ledc_update_duty(SERVO_PWM_MODE, s->canal);
    if (e != ESP_OK) {
        printf("ERROR PWM de %s: %s\n", s->nombre, esp_err_to_name(e));
        return false;
    }
    s->us_actual = microsegundos;
    s->pwm_activo = true;
    if (s == &servo_codo)
        modo_preparado_codo_calibrado =
            s->ready_calibrado && microsegundos == s->ready_us;
    return true;
}

bool servo_activar_manual(servo_t *s)
{
    if (s->pwm_activo) {
        printf("%s: PWM ya activo; ultimo solicitado=%d us.\n",
               s->nombre, s->us_actual);
        return true;
    }
    int objetivo = (s == &servo_pinza) ? PINZA_CERRADA_US : CODO_HOME_US;
    printf("PRIMER PWM %s: %d us. LA POSICION FISICA ES DESCONOCIDA.\n",
           s->nombre, objetivo);
    printf("Puede haber un movimiento brusco hasta el objetivo.\n");
    if (s == &servo_codo && !confirmar(
        "CODO: hombro apoyado y trayectoria HACIA 500us (HOME plegado) despejada?"))
        return false;
    if (s == &servo_pinza && !confirmar(
        "PINZA: mecanismo despejado para el primer cierre a 1500 us?"))
        return false;
    bool ok=servo_escribir_pwm(s, objetivo);
    if (ok && s == &servo_codo) {
        codo_angulo_objetivo=0.0;
        registrar_codo("ACTIVACION MANUAL A HOME",-1,0.0);
    }
    return ok;
}

void servo_diagnostico(servo_t *s)
{
    printf("\n===== DIAGNOSTICO PWM %s =====\n", s->nombre);
    printf("PWM activo: %s\n", s->pwm_activo ? "SI" : "NO");
    printf("PWM solicitado actual: %d us\n", s->us_actual);
    printf("Banda de ensayo del programa: %d a %d us\n",
           servo_min_us(s), servo_max_us(s));
    printf("Margen hacia MENOR PWM: %d us\n",
           s->us_actual - servo_min_us(s));
    printf("Margen hacia MAYOR PWM: %d us\n",
           servo_max_us(s) - s->us_actual);
    if (s->us_actual <= servo_min_us(s))
        printf("SOFTWARE: tope de PWM MINIMO alcanzado.\n");
    if (s->us_actual >= servo_max_us(s))
        printf("SOFTWARE: tope de PWM MAXIMO alcanzado.\n");
    printf("IMPORTANTE: si el PWM cambia pero el eje no gira, no concluyas ");
    printf("que sea un limite de software. Puede ser atasco, alimentacion, ");
    printf("sobrecarga o version de giro continuo.\n");
    printf("Si hay crujido o movimiento inesperado, detente y comprueba ");
    printf("el mecanismo sin carga.\n");
    if (s == &servo_codo) {
        printf("CODO: centro nominal 1500 us; READY exacto registrado=%s (%d us).\n",
               s->ready_calibrado ? "SI" : "NO, PENDIENTE",
               s->ready_us);
    }
}

bool servo_mover_pwm_directo(servo_t *s, int destino)
{
    if (!s->pwm_activo || paro) {
        printf("%s: primero activar PWM con i.\n", s->nombre);
        return false;
    }
    if (!servo_us_admisible(s, destino)) {
        printf("%s: destino %d us fuera de referencias permitidas.\n",
               s->nombre, destino);
        return false;
    }
    if (comprobar_x()) return false;
    if (s == &servo_codo && base_home && hombro_home &&
        !restricciones_recorrido_codo_admisible(s->us_actual,destino,
                                                 base_objetivo_grados))
        return false;
    printf("%s: consigna DIRECTA %d -> %d us. Sin rampa programada.\n",
           s->nombre, s->us_actual, destino);
    if (!servo_escribir_pwm(s, destino)) return false;
    printf("PWM enviado; posicion fisica NO medida.\n");
    return true;
}

bool codo_mover_home_ready_gradual(int destino)
{
    if (!servo_codo.pwm_activo || paro || !servo_us_admisible(&servo_codo,destino))
        return false;
    int origen=servo_codo.us_actual;
    if(origen==destino) return !comprobar_x();
    if(base_home && hombro_home &&
       !restricciones_recorrido_codo_admisible(origen,destino,
                                               base_objetivo_grados)) return false;
    const int signo=destino>origen ? 1 : -1;
    while(servo_codo.us_actual != destino) {
        if(comprobar_x()) return false;
        int restante=abs(destino-servo_codo.us_actual);
        int salto=restante < CODO_HOME_READY_PASO_PWM_US ? restante :
                   CODO_HOME_READY_PASO_PWM_US;
        if(!servo_escribir_pwm(&servo_codo,servo_codo.us_actual + signo*salto))
            return false;
        // Comprobacion del paro mientras se asienta la nueva consigna.
        TickType_t total=pdMS_TO_TICKS(CODO_HOME_READY_INTERVALO_MS);
        if(total<1) total=1;
        for(TickType_t t=0;t<total;t++) {
            if(comprobar_x()) return false;
            vTaskDelay(1);
        }
    }
    return !comprobar_x();
}

void servos_leer_calibracion_nvs(void)
{
    // La V5.4 tenia referencias antiguas que pueden diferir de las medidas
    // de este proyecto. Se usa namespace NUEVO para no importar accidentalmente
    // una calibracion antigua de READY. HOME codo y pinza son constantes.
    nvs_handle_t h;
    if (nvs_open("brazo_v55", NVS_READONLY, &h) != ESP_OK) return;
    int32_t valor = 0;
    if (nvs_get_i32(h, "c_ready", &valor) == ESP_OK &&
        servo_us_admisible(&servo_codo, (int)valor) && valor == 1500) {
        servo_codo.ready_us = (int)valor;
        servo_codo.ready_calibrado = true;
    }
    nvs_close(h);
}

void reportar_servos(void)
{
    servo_t *servos[2] = {&servo_codo, &servo_pinza};
    for (int i=0; i<2; ++i) {
        servo_t *s=servos[i];
        printf("%s: PWM=%s | ultimo solicitado=%d us | refs=%s/%s\n",
               s->nombre, s->pwm_activo?"ACTIVO":"INACTIVO",
               s->us_actual, s->home_calibrado?"SI":"NO",
               s->ready_calibrado?"SI":"NO");
        if (s->home_calibrado)
            printf("  %s=%d us\n", i==0 ? "HOME plegado" : "ABIERTA", s->home_us);
        if (s->ready_calibrado)
            printf("  %s=%d us\n", i==0 ? "READY vertical" : "CERRADA", s->ready_us);
    }
    printf("PINZA: refs FIJAS por prueba: ABIERTA=611us (80 grados nominales), ");
    printf("CERRADA=1500us (0 grados de apertura). Modo: RAPIDO.\n");
    printf("CODO: HOME 500us, READY 1500us ya confirmados; nominal 0..180 grados.\n");

    // Resumen conjunto disponible con 'p' tanto en menu principal como en
    // menus de servos. Mostrar estados desconocidos sin inventar angulos.
    printf("\n===== RESUMEN ANGULAR DEL BRAZO =====\n");
    if (base_home) {
        printf("BASE: %+.3f grados OBJETIVO NOMINAL; %+.3f grados estimados por pasos (HOME=0); contador=%ld pasos.\n",
               base_objetivo_grados,
               (double)base_pulsos*360.0/PULSOS_VUELTA_BASE,
               (long)base_pulsos);
    } else if (frente_provisional) {
        printf("BASE: %+.3f grados PROVISIONALES (sin HOME validado).\n",
               (double)base_pulsos*360.0/PULSOS_VUELTA_BASE);
    } else {
        printf("BASE: SIN HOME; angulo absoluto DESCONOCIDO.\n");
    }
    if (hombro_home) {
        printf("HOMBRO: %+.3f grados OPERATIVOS (0=READY a 1320 pasos); ",
               (double)(hombro_rel_pulsos-HOMBRO_READY_PULSOS)*360.0/
               PULSOS_VUELTA_HOMBRO);
        printf("%+.3f grados desde HOME mecanico; contador=%ld pasos.\n",
               (double)hombro_rel_pulsos*360.0/PULSOS_VUELTA_HOMBRO,
               (long)hombro_rel_pulsos);
    } else {
        printf("HOMBRO: SIN HOME; angulo operativo DESCONOCIDO ");
        printf("(contador relativo=%ld pasos).\n", (long)hombro_rel_pulsos);
    }
    if (servo_codo.pwm_activo) {
        printf("CODO: %.3f grados NOMINALES desde HOME (500us=0, 1500us=90); ",
               codo_grados_de_pwm(servo_codo.us_actual));
        printf("PWM solicitado=%d us.\n",servo_codo.us_actual);
    } else {
        printf("CODO: PWM OFF; angulo fisico DESCONOCIDO. ");
        printf("Ultima consigna=%d us.\n",servo_codo.us_actual);
    }
    if (servo_pinza.pwm_activo) {
        printf("PINZA: %.2f grados NOMINALES de apertura ",
               (PINZA_CERRADA_US-servo_pinza.us_actual)*80.0/
               (PINZA_CERRADA_US-PINZA_ABIERTA_US));
        printf("(0=cerrada, 80=abierta); PWM solicitado=%d us.\n",
               servo_pinza.us_actual);
    } else {
        printf("PINZA: PWM OFF; angulo fisico DESCONOCIDO. ");
        printf("Ultima consigna=%d us.\n",servo_pinza.us_actual);
    }
    printf("NOTA: NEMA estimados por pasos; servos por consigna PWM, ");
    printf("no por sensores de posicion.\n");
    mostrar_cinematica_directa();
}

void servo_menu_manual_impl(servo_t *s, bool recuperacion)
{
    if (s == &servo_pinza) { menu_pinza_operacion(); return; }
    if (!recuperacion && (!base_home || !hombro_home || !preparado_base)) {
        printf("CODO: primero HOME de BASE/HOMBRO y entrar a PREPARADO.\n");
        return;
    }
    if (paro) return;
    for (;;) {
        if (paro || sistema_cerrado) return;
        printf("\n======= CODO MG996R | %s =======\n",
               recuperacion ? "ACOMODO PRE-HOME +/-10 GRADOS" :
                              "PRUEBAS DE LIMITES HOMBRO-CODO");
        printf("0 nominal=500us IZQUIERDA | 90 nominal=1500us READY | 180 nominal=2500us DERECHA.\n");
        printf("HOME=500us | READY=%dus (registrado) | PWM actual=%dus [%s].\n",
               s->ready_us, s->us_actual, s->pwm_activo ? "ON" : "OFF");
        printf("i: habilitar PWM HACIA HOME 500us (si ya ON no cambia consigna)\n");
        if (recuperacion) {
            printf("4: incremento +/-10 grados nominales | r derecha, l izquierda\n");
        } else {
            printf("1: 0.5 grados | 2: 1 grado | 3: 5 grados | 4: 10 grados\n");
            printf("5: 20 grados  | 6: 30 grados | 7: 45 grados | 8: 90 grados\n");
            printf("Despues: r DERECHA / l IZQUIERDA.\n");
        }
        printf("c: CODO a READY=1500us | o: CODO HOME=500us\n");
        if (recuperacion)
            printf("s: terminar AJUSTE del CODO y pasar a la confirmacion de despeje.\n");
        printf("v: diagnostico | p: estado | q: atras | x: PARO.\n");
        printf("Grados NOMINALES; PWM indica consigna y NO mide el angulo fisico.\n");
        printf("MOV1 INACTIVO; MOV2/4 y topes PWM activos. Tecla n.\n");
        if (recuperacion) printf("UI:HOME_AJUSTE_CODO_LISTO\n");
        printf("Comando: ");
        char c = leer_char();
        if (c == 'q' || c == 'x') return;
        // En el ajuste PRE-HOME, s cierra el menu (NO mueve al codo ni
        // confirma por si sola el despeje): el llamador pregunta de nuevo.
        if (recuperacion && c == 's') {
            printf("UI:HOME_AJUSTE_CODO_TERMINADO_POR_OPERADOR\n");
            return;
        }
        if (c == 'i') {(void)servo_activar_manual(s); continue;}
        if (c == 'p') {reportar_servos(); continue;}
        if (c == 'n') {mostrar_referencias_mecanicas_v2(); continue;}
        if (c == 'v') {servo_diagnostico(s); continue;}
        if (c == 'c' || c == 'o') {
            if (!s->pwm_activo) {printf("Primero activa PWM con i.\n");continue;}
            int destino = (c == 'c') ? s->ready_us : CODO_HOME_US;
            int antes = s->us_actual;
            // En el codo, incluso los controles manuales HOME/READY
            // utilizan la rampa lenta; la pinza conserva control directo.
            if (codo_mover_home_ready_gradual(destino)) {
                codo_angulo_objetivo = codo_grados_de_pwm(destino);
                registrar_codo(c=='c' ? "CODO A READY" : "CODO A HOME",
                               antes, codo_grados_de_pwm(destino)-codo_grados_de_pwm(antes));
            }
            continue;
        }
        double incremento=0.0;
        switch(c) {
            case '1': if (!recuperacion) incremento=0.5; break;
            case '2': if (!recuperacion) incremento=1.0; break;
            case '3': if (!recuperacion) incremento=5.0; break;
            case '4': incremento=10.0; break;
            case '5': if (!recuperacion) incremento=20.0; break;
            case '6': if (!recuperacion) incremento=30.0; break;
            case '7': if (!recuperacion) incremento=45.0; break;
            case '8': if (!recuperacion) incremento=90.0; break;
            default: break;
        }
        if (incremento <= 0.0) {printf("Selecciona una opcion valida.\n");continue;}
        if (!s->pwm_activo) {printf("Primero activa PWM hacia HOME con i.\n");continue;}
        printf("Incremento NOMINAL %.1f grados; r=DERECHA (+), l=IZQUIERDA (-), q=cancelar.\n",
               incremento);
        if (recuperacion) printf("UI:HOME_AJUSTE_CODO_DIRECCION\n");
        char dir = leer_char();
        if (dir == 'x') return;
        if (dir == 'q') continue;
        if (dir != 'r' && dir != 'l') {printf("Direccion invalida.\n");continue;}
        double delta = (dir=='r') ? incremento : -incremento;
        // Se acumulan los grados solicitados en coma flotante y SOLO AL FINAL
        // se redondea a us. Asi 0.5+0.5 representa 1.0 grado nominal.
        double nuevo = codo_angulo_objetivo+delta;
        if (nuevo < -0.000001 || nuevo > 180.000001) {
            printf("TOPE NOMINAL CODO: %.3f grados solicitados fuera de 0..180.\n",nuevo);
            continue;
        }
        if (nuevo < 0.0) nuevo=0.0;
        if (nuevo > 180.0) nuevo=180.0;
        int objetivo = codo_pwm_de_grados(nuevo);
        int antes=s->us_actual;
        printf("CODO: %+0.1f grados nominales -> objetivo %.3f grados nominales.\n",
               delta,nuevo);
        // CODO manual: misma rampa de consignas que en HOME/READY,
        // 8 us cada 20 ms (20% menor que 10 us/20 ms de referencia).
        if (codo_mover_home_ready_gradual(objetivo)) {
            codo_angulo_objetivo=nuevo;
            registrar_codo(dir=='r'?"MANUAL DERECHA":"MANUAL IZQUIERDA",
                           antes,delta);
        }
    }
}

void servo_menu_manual(servo_t *s, bool recuperacion)
{
    if (recuperacion && s == &servo_codo)
        printf("UI:HOME_AJUSTE_CODO_INICIO\n");
    servo_menu_manual_impl(s,recuperacion);
    if (recuperacion && s == &servo_codo)
        printf("UI:HOME_AJUSTE_CODO_FIN\n");
}

void menu_pinza_operacion(void)
{
    servo_t *s = &servo_pinza;
    if (paro) return;
    for (;;) {
        if (paro) return;
        printf("\n======= PINZA MG90S | OPERACION =======\n");
        printf("Referencias: ABIERTA=611us izq (80 grados nominales), CERRADA=1500us.\n");
        printf("i: activar PWM a 1500us (puede moverse)\n");
        printf("a: ABRIR 80 grados (611us) | c: CERRAR (1500us)\n");
        printf("Movimiento SOLO RAPIDO: consigna directa, sin rampa.\n");
        printf("p: RESUMEN con angulos BASE/HOMBRO/CODO/PINZA | q: salir | x: PARO\n");
        printf("PWM=%s/%dus | comando: ",
               s->pwm_activo ? "ON" : "OFF", s->us_actual);
        char c = leer_char();
        if (c == 'q' || c == 'x') return;
        if (c == 'i') {(void)servo_activar_manual(s); continue;}
        if (c == 'p') {reportar_servos(); continue;}
        if (c == 'a' || c == 'c') {
            if (!s->pwm_activo) {printf("Primero i, con mecanismo despejado.\n");continue;}
            int objetivo = c == 'a' ? PINZA_ABIERTA_US : PINZA_CERRADA_US;
            if (servo_mover_pwm_directo(s, objetivo)) {
                printf("PINZA %s: %.0f grados nominales de apertura; ",
                       c == 'a' ? "ABIERTA" : "CERRADA",
                       c == 'a' ? 80.0 : 0.0);
                printf("PWM=%d us. Movimiento directo enviado.\n", objetivo);
            }
            continue;
        }
        printf("Comando invalido.\n");
    }
}

