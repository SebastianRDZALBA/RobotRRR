/*
 * Componente hombro: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "hombro.h"

bus_resultado_t buscar_estado_hombro(int dir, int nivel,
                                           int max_pulsos, int pausa_us)
{
    for (int i = 0; i < max_pulsos; ++i) {
        if (comprobar_x()) return BUS_ERR;
        if (switch_estable(hombro.sw, nivel)) return BUS_SI;
        if (!paso(&hombro, dir, pausa_us)) return BUS_ERR;
        if ((i % 32) == 31) vTaskDelay(1);
    }
    if (comprobar_x()) return BUS_ERR;
    return switch_estable(hombro.sw, nivel) ? BUS_SI : BUS_NO;
}

bool hombro_mover_a(int32_t destino, bool movimiento_auto_lento)
{
    if (!base_home || !hombro_home || paro) {
        printf("ERROR: HOME BASE y HOMBRO obligatorios. Usa h.\n");
        return false;
    }

    // Movimiento manual supervisado, con las restricciones PDF V3/MOV2-4 activas.
    if (!transicion_coordinada && servo_codo.pwm_activo &&
        servo_codo.us_actual != servo_codo.ready_us) {
        printf("AVISO: HOMBRO con CODO fuera de READY: supervise el recorrido.\n");
    }
    const int32_t solicitado = destino;
    // Cuando estamos fisicamente en HOME (0), nunca convertir una orden
    // hacia IZQUIERDA en un movimiento hacia DERECHA por el recorte.
    if (hombro_rel_pulsos < HOMBRO_LIMITE_MIN_PULSOS &&
        solicitado < HOMBRO_LIMITE_MIN_PULSOS) {
        printf("HOMBRO en zona especial de HOME: movimiento normal bloqueado. "
               "Usa y/s para salir hacia READY.\n");
        return false;
    }
    if (destino < HOMBRO_LIMITE_MIN_PULSOS) destino = HOMBRO_LIMITE_MIN_PULSOS;
    if (destino > HOMBRO_LIMITE_MAX_PULSOS) destino = HOMBRO_LIMITE_MAX_PULSOS;
    if (destino != solicitado) {
        printf("LIMITE MECANICO HOMBRO: pedido %+.3f deg desde HOME; "
               "destino permitido %+.3f deg.\n",
               grados_hombro(solicitado), grados_hombro(destino));
        printf("Solo avanzara lo disponible: %+ld pulsos (%.3f deg de movimiento).\n",
               (long)(destino-hombro_rel_pulsos),
               grados_hombro(destino-hombro_rel_pulsos));
    }

    // MOV1 inactivo: no bloquear avance de H por PWM ausente debido a MOV1.
    // MOV2/MOV4 siguen validados con su logica original V5.13.
    if (MOV1_ACTIVO && hombro_home && !servo_codo.pwm_activo &&
        destino > hombro_rel_pulsos && destino >= M1_P3_H) {
        printf("RESTRICCION MOV 1: CODO sin PWM, no se conoce postura.\n");
        return false;
    }
    if (base_home && servo_codo.pwm_activo &&
        !restricciones_recorrido_hombro_admisible(hombro_rel_pulsos, destino,
                 base_objetivo_grados,
                 codo_grados_de_pwm(servo_codo.us_actual)))
        return false;
    int32_t delta = destino - hombro_rel_pulsos;
    if (delta == 0) {
        hombro_preparado = (hombro_rel_pulsos == HOMBRO_READY_PULSOS);
        printf("HOMBRO: ya en %+.3f deg desde HOME / %+.3f deg operativos.\n",
               grados_hombro(hombro_rel_pulsos),
               grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS));
        return true;
    }

    int dir = (delta > 0) ? HOMBRO_DIR_ALEJAR : HOMBRO_DIR_HOME;
    int n = (int)(delta > 0 ? delta : -delta);

    // Un switch activado lejos de HOME significa error de sensor o referencia.
    if (hombro_rel_pulsos > HOMBRO_SW_LIBERAR_READY_MAX &&
        switch_estable(hombro.sw, 1)) {
        printf("ERROR: HOMBRO SW=1 lejos de HOME. Referencia invalidada.\n");
        hombro_home = false;
        hombro_preparado = false;
        preparado_base = false;
        return false;
    }

    habilitar(&hombro);
    hombro_preparado = false;
    printf("HOMBRO: %+ld -> %+ld pulsos (HOME %+.3f -> %+.3f grados; "
           "operacion %+.3f -> %+.3f grados).\n",
           (long)hombro_rel_pulsos, (long)destino,
           grados_hombro(hombro_rel_pulsos), grados_hombro(destino),
           grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS),
           grados_hombro(destino-HOMBRO_READY_PULSOS));

    for (int i = 0; i < n; ++i) {
        if (comprobar_x()) return false;
        if (dir == HOMBRO_DIR_HOME && switch_estable(hombro.sw, 1)) {
            printf("ERROR: switch activado antes del limite normal. HOME invalido.\n");
            hombro_home = false;
            preparado_base = false;
            return false;
        }
        int t = i;
        int faltan = n-i-1;
        if (faltan < t) t = faltan;
        if (t > RAMPA_PULSOS) t = RAMPA_PULSOS;
        int us = movimiento_auto_lento ? PASO_HOMBRO_AUTO_US :
            PASO_INICIAL_US -
            (PASO_INICIAL_US-PASO_MOVIMIENTO_US)*t/RAMPA_PULSOS;
        if (!paso(&hombro, dir, us)) return false;
        if ((i % 32) == 31) vTaskDelay(1);

        if (dir == HOMBRO_DIR_HOME && switch_estable(hombro.sw, 1)) {
            printf("ERROR: switch activado durante operacion normal. HOME invalido.\n");
            hombro_home = false;
            preparado_base = false;
            return false;
        }
        if (dir == HOMBRO_DIR_ALEJAR &&
            hombro_rel_pulsos > HOMBRO_SW_LIBERAR_READY_MAX &&
            switch_estable(hombro.sw, 1)) {
            printf("ERROR: microswitch no libero al salir de HOME.\n");
            hombro_home = false;
            preparado_base = false;
            return false;
        }
    }
    hombro_preparado = (hombro_rel_pulsos == HOMBRO_READY_PULSOS);
    printf("HOMBRO: HOME=%+.3f deg | OPERACION=%+.3f deg | %ld pulsos.\n",
           grados_hombro(hombro_rel_pulsos),
           grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS),
           (long)hombro_rel_pulsos);
    ++num_movimientos;
    printf("[REGISTRO #%05lu] HOMBRO: %d pulsos ejecutados, destino=%ld; ",
           num_movimientos, n, (long)hombro_rel_pulsos);
    printf("angulo operativo=%+.3f grados (estimado por pasos).\n",
           grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS));
    return true;
}

bool regresar_hombro_a_home(void)
{
    if (paro || !base_home || !hombro_home || base_pulsos != 0) {
        printf("Para HOME del HOMBRO: ambos HOME validos y BASE en 0.\n");
        return false;
    }
    if (hombro_rel_pulsos == 0 && switch_estable(hombro.sw, 1)) {
        printf("HOMBRO ya en HOME mecanico, SW=1.\n");
        hombro_preparado = false;
        preparado_base = false;
        return true;
    }
    if (hombro_rel_pulsos == 0 && !switch_estable(hombro.sw, 1)) {
        printf("ERROR: contador en HOME pero SW=0. Referencia invalida.\n");
        hombro_home = false;
        preparado_base = false;
        return false;
    }
    if (hombro_rel_pulsos < HOMBRO_LIMITE_MIN_PULSOS) {
        printf("ERROR: hombro entre HOME y limite operativo sin referencia de retorno.\n");
        hombro_home = false;
        preparado_base = false;
        return false;
    }
    printf("\nRETORNO HOMBRO A HOME: usando rutina especial bajo 9.750 deg.\n");
    if (!hombro_mover_a(HOMBRO_LIMITE_MIN_PULSOS, true)) return false;
    if (switch_estable(hombro.sw, 1)) {
        printf("ERROR: SW activo a 130 pulsos, antes de HOME. Referencia invalida.\n");
        hombro_home = false;
        preparado_base = false;
        return false;
    }
    printf("Aproximacion lenta IZQUIERDA hasta SW=1; %d pulsos nominales. x=PARO.\n",
           HOMBRO_RETORNO_HOME_MAX);
    bus_resultado_t r = buscar_estado_hombro(HOMBRO_DIR_HOME, 1,
                                             HOMBRO_RETORNO_HOME_MAX,
                                             PASO_HOMBRO_APROX_US);
    if (r == BUS_NO) {
        // El contacto puede activarse justo al detenerse el ultimo paso.
        // Dar tiempo a que el brazo se asiente antes de ordenar mas movimiento.
        printf("HOMBRO: SW aun libre tras %d pasos; esperar %d ms para estabilizar.\n",
               HOMBRO_RETORNO_HOME_MAX, HOMBRO_RETORNO_ESPERA_MS);
        vTaskDelay(pdMS_TO_TICKS(HOMBRO_RETORNO_ESPERA_MS));
        if (comprobar_x()) return false;
        if (switch_estable(hombro.sw, 1)) {
            r = BUS_SI;
            printf("HOMBRO: SW detectado tras asentamiento, sin pasos extra.\n");
        } else {
            // Margen maximo de solo 0.450 grados del hombro; buscar_estado_hombro
            // comprueba el SW antes de cada paso y se detiene al activarse.
            printf("HOMBRO: ajuste fino adicional hacia IZQUIERDA, ");
            printf("max %d micropasos (%.3f grados). x=PARO.\n",
                   HOMBRO_RETORNO_MARGEN_PULSOS,
                   grados_hombro(HOMBRO_RETORNO_MARGEN_PULSOS));
            r = buscar_estado_hombro(HOMBRO_DIR_HOME, 1,
                                     HOMBRO_RETORNO_MARGEN_PULSOS,
                                     PASO_HOMBRO_APROX_US);
            if (r == BUS_NO) {
                // Ultima comprobacion por posible rebote/retardo del switch.
                vTaskDelay(pdMS_TO_TICKS(HOMBRO_RETORNO_ESPERA_MS));
                if (comprobar_x()) return false;
                if (switch_estable(hombro.sw, 1)) {
                    r = BUS_SI;
                    printf("HOMBRO: SW detectado al asentarse tras ajuste fino.\n");
                }
            }
        }
    }
    if (r != BUS_SI) {
        if (!paro) {
            printf("ERROR: no se detecto SW al retornar a HOME ni con margen ");
            printf("de %d micropasos. No forzar el tope.\n",
                   HOMBRO_RETORNO_MARGEN_PULSOS);
            hombro_home = false;
            preparado_base = false;
        }
        return false;
    }
    hombro_rel_pulsos = 0;
    hombro_preparado = false;
    preparado_base = false;
    printf("HOME HOMBRO RECONFIRMADO: 0 pulsos | SW=1.\n");
    return true;
}

bool home_automatico_hombro(void)
{
    if (paro || !base_home || base_pulsos != 0) {
        printf("HOMBRO: primero HOME BASE valido a 0 grados.\n");
        return false;
    }
    hombro_home = false;
    hombro_preparado = false;
    preparado_base = false;
    printf("\n===== HOME AUTOMATICO HOMBRO (SOLO REFERENCIA) =====\n");
    printf("Leyendo SW antes de mover. Si SW=1, NO se buscara a izquierda inicialmente.\n");
    printf("Busqueda IZQUIERDA solo si SW=0, hasta %d pulsos. x=PARO\n",
           HOMBRO_BUSQUEDA_MAX);
    printf("Velocidad: mitad que HOME de BASE (11000 us entre pulsos).\n");
    habilitar(&hombro);
    bus_resultado_t r;

    if (!switch_estable(hombro.sw, 1)) {
        // Primera aproximacion: debe activarse antes del tope rigido.
        r = buscar_estado_hombro(HOMBRO_DIR_HOME, 1,
                                 HOMBRO_BUSQUEDA_MAX, PASO_HOMBRO_AUTO_US);
        if (r != BUS_SI) {
            if (!paro) printf("ERROR: HOME HOMBRO no detectado en recorrido permitido.\n");
            return false;
        }
    } else {
        printf("HOMBRO SW=1 al inicio: NO mover IZQUIERDA. Solo liberar a DERECHA y reaportar lento.\n");
    }
    if (paro) return false;
    printf("HOMBRO: microswitch ACTIVADO (SW=1).\n");

    // Liberar en sentido opuesto SIN seguir empujando contra el tope.
    printf("HOMBRO: elevando hacia DERECHA hasta que el switch quede LIBRE (SW=0)...\n");
    r = buscar_estado_hombro(HOMBRO_DIR_ALEJAR, 0,
                             HOMBRO_LIBERAR_MAX, PASO_HOMBRO_AUTO_US);
    if (r != BUS_SI) {
        if (!paro) printf("ERROR: HOMBRO SW no libero. Revisar switch NC y mecanica.\n");
        return false;
    }
    if (!switch_estable(hombro.sw, 0)) {
        printf("ERROR: estado del microswitch no estable al liberar.\n");
        return false;
    }
    printf("HOMBRO: microswitch LIBRE (SW=0).\n");

    // No se agrega margen: exactamente libera y vuelve lentamente al contacto.
    printf("HOMBRO: segunda aproximacion LENTA hacia IZQUIERDA, hasta SW=1...\n");
    r = buscar_estado_hombro(HOMBRO_DIR_HOME, 1,
                             HOMBRO_APROX_MAX, PASO_HOMBRO_APROX_US);
    if (r != BUS_SI) {
        if (!paro) printf("ERROR: no se detecto segunda activacion HOME HOMBRO.\n");
        return false;
    }
    if (!switch_estable(hombro.sw, 1)) {
        printf("ERROR: no se confirma microswitch activado al finalizar.\n");
        return false;
    }
    hombro_rel_pulsos = 0;
    hombro_home = true;
    hombro_preparado = false;
    printf("\nHOME HOMBRO COMPLETADO: 0 pulsos, SW=%d.\n", gpio_get_level(hombro.sw));
    printf("Posicion erguida calibrada: +%d pulsos (%.3f grados).\n",
           HOMBRO_READY_PULSOS, grados_hombro(HOMBRO_READY_PULSOS));
    printf("El hombro PERMANECE en HOME hasta que elijas PREPARADO.\n");
    return true;
}

bool mover_hombro_despeje(int dir, int32_t origen)
{
    if (paro) return false;
    int32_t siguiente = hombro_rel_pulsos +
        (dir == HOMBRO_DIR_ALEJAR ? HOMBRO_PASO_10G : -HOMBRO_PASO_10G);
    if (siguiente - origen > HOMBRO_DESPEJE_MAX ||
        siguiente - origen < -HOMBRO_DESPEJE_MAX) {
        printf("DESPEJE: recorrido relativo maximo de +/-120 grados.\n");
        return true;
    }
    if (dir == HOMBRO_DIR_HOME && switch_estable(hombro.sw, 1)) {
        printf("DESPEJE: switch HOMBRO activo. No continuar hacia IZQUIERDA.\n");
        return true;
    }
    habilitar(&hombro);
    for (int i = 0; i < HOMBRO_PASO_10G; ++i) {
        if (comprobar_x()) return false;
        if (dir == HOMBRO_DIR_HOME && switch_estable(hombro.sw, 1)) {
            printf("DESPEJE: microswitch HOMBRO alcanzado.\n");
            break;
        }
        if (!paso(&hombro, dir, PASO_HOME_US)) return false;
        if ((i % 32) == 31) vTaskDelay(1);
        if (dir == HOMBRO_DIR_HOME && switch_estable(hombro.sw, 1)) {
            printf("DESPEJE: microswitch HOMBRO alcanzado.\n");
            break;
        }
    }
    printf("HOMBRO: desplazamiento desde inicio de DESPEJE %+.3f grados.\n",
           grados_hombro(hombro_rel_pulsos - origen));
    return true;
}

void reportar_hombro(void)
{
    printf("HOMBRO: HOME=%s | EN READY=%s | SW=%d | contador=%+ld pulsos\n",
           hombro_home ? "SI" : "NO", hombro_preparado ? "SI" : "NO",
           gpio_get_level(hombro.sw), (long)hombro_rel_pulsos);
    if (hombro_home) {
        printf("Posicion desde HOME mecanico: %+.3f grados.\n",
               grados_hombro(hombro_rel_pulsos));
        printf("Posicion OPERATIVA: %+.3f grados (0 = READY, 1320 pulsos).\n",
               grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS));
        printf("Limites de operacion: -89.250 a +131.025 grados "
               "(130..3067 pulsos desde HOME).\n");
    } else {
        printf("Posicion absoluta de HOMBRO desconocida; solo conteo relativo.\n");
    }
}

void modo_fino_hombro(void)
{
    if (!base_home || base_pulsos != 0 || !hombro_home) {
        printf("Primero completa HOME BASE y HOMBRO (h).\n");
        return;
    }
    printf("\n===== AJUSTE FINO HOMBRO =====\n");
    printf("r/l: +/-10 deg | a/d: +/-1 deg | w/e: +/-0.5 deg | +/-: 1 micropaso\n");
    printf("y: volver a READY (cero operativo) | o: volver a HOME mecanico\n");
    printf("p: estado | q: salir | x: PARO. Sin confirmaciones.\n");
    for (;;) {
        printf("HOMBRO operativo %+.3f deg (%ld pulsos desde HOME) | comando: ",
               grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS), (long)hombro_rel_pulsos);
        char c = leer_char();
        if (c == 'x' || c == 'q') return;
        if (c == 'p') {reportar_hombro(); continue;}
        if (c == 'y') {(void)hombro_mover_a(HOMBRO_READY_PULSOS, false); continue;}
        if (c == 'o') {if (regresar_hombro_a_home()) return; else continue;}
        int dir=0,n=0;
        switch (c) {
            case 'r': dir=1; n=HOMBRO_PASO_10G; break;
            case 'l': dir=0; n=HOMBRO_PASO_10G; break;
            case 'a': dir=1; n=HOMBRO_PASO_1G; break;
            case 'd': dir=0; n=HOMBRO_PASO_1G; break;
            case 'w': dir=1; n=HOMBRO_PASO_05G; break;
            case 'e': dir=0; n=HOMBRO_PASO_05G; break;
            case '+': dir=1; n=1; break;
            case '-': dir=0; n=1; break;
            default: printf("Comando no reconocido.\n"); continue;
        }
        int32_t objetivo = hombro_rel_pulsos + (dir == 1 ? n : -n);
        (void)hombro_mover_a(objetivo, false);
        if (!hombro_home || paro) return;
    }
}

void probar_hombro_10(void)
{
    if (!base_home || base_pulsos != 0 || !hombro_home) {
        printf("Primero HOME completo de BASE y HOMBRO. Usa h.\n");
        return;
    }
    printf("\n===== HOMBRO: TRAMOS REPETIDOS DE 10 GRADOS =====\n");
    printf("r=+10, l=-10 | y=READY (0 operativo) | p=estado | q=salir | x=PARO\n");
    for (;;) {
        printf("HOMBRO operativo %+.3f deg (%ld pasos desde HOME) | comando: ",
               grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS), (long)hombro_rel_pulsos);
        char c = leer_char();
        if (c == 'q' || c == 'x') return;
        if (c == 'p') {reportar_hombro(); continue;}
        if (c == 'y') {(void)hombro_mover_a(HOMBRO_READY_PULSOS, false); continue;}
        if (c != 'r' && c != 'l') {printf("Usa r/l/y/p/q/x.\n"); continue;}
        int32_t destino = hombro_rel_pulsos + (c=='r' ? HOMBRO_PASO_10G : -HOMBRO_PASO_10G);
        (void)hombro_mover_a(destino, false);
        if (!hombro_home || paro) return;
    }
}

