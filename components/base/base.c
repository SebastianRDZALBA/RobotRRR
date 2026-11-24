/*
 * Componente base: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "base.h"

bus_resultado_t buscar_estado_base(int dir, int nivel,
                                           int max_pulsos, int pausa_us,
                                           bool mostrar_avance, int32_t origen)
{
    for (int i = 0; i < max_pulsos; ++i) {
        if (comprobar_x()) return BUS_ERR;
        if (switch_estable(base.sw, nivel)) return BUS_SI;
        if (!paso(&base, dir, pausa_us)) return BUS_ERR;
        if ((i % 32) == 31) vTaskDelay(1);
        if (mostrar_avance && ((i + 1) % REPORTE_CADA_PULSOS == 0)) {
            printf("[MANUAL] desde inicio=%+.3f grados | ",
                   grados_base(base_pulsos - origen));
            printf("pasos=%+ld | switch=%d\n",
                   (long)(base_pulsos - origen), gpio_get_level(base.sw));
        }
    }
    if (comprobar_x()) return BUS_ERR;
    return switch_estable(base.sw, nivel) ? BUS_SI : BUS_NO;
}

bool finalizar_home_base(int direccion_busqueda)
{
    bus_resultado_t r;
    int32_t inicio;
    int liberar_inicial = !direccion_busqueda;

    printf("Microswitch activado. Liberando hacia %s...\n",
           liberar_inicial == BASE_DERECHA ? "DERECHA" : "IZQUIERDA");
    inicio = base_pulsos;
    r = buscar_estado_base(liberar_inicial, 0,
                           SALIDA_SWITCH_MAX, PASO_LENTO_US, false, 0);
    printf("Liberacion inicial: %ld pasos | switch=%d\n",
           (long)labs((long)(base_pulsos-inicio)), gpio_get_level(base.sw));
    if (r != BUS_SI) {
        printf("ERROR: No se libero el microswitch dentro del recorrido de seguridad.\n");
        return false;
    }

    // Si se detecto viniendo de DERECHA, al retroceder a IZQUIERDA
    // ya estamos del lado necesario para la ultima aproximacion.
    // Si se detecto viniendo de IZQUIERDA, el primer retroceso fue
    // hacia DERECHA. Es necesario cruzar con cuidado al lado IZQUIERDO
    // para recuperar la misma referencia que los 90 pulsos originales.
    if (direccion_busqueda == BASE_IZQUIERDA) {
        printf("Preparando referencia calibrada: cruzar el contacto hacia IZQUIERDA...\n");

        // Desde el lado DERECHO libre, entrar al switch hacia izquierda.
        r = buscar_estado_base(BASE_IZQUIERDA, 1,
                               SALIDA_SWITCH_MAX, PASO_LENTO_US, false, 0);
        if (r != BUS_SI) {
            printf("ERROR: No hubo nueva activacion al cruzar el switch.\n");
            return false;
        }
        // Continuar hacia izquierda hasta que el switch vuelva a quedar libre.
        inicio = base_pulsos;
        r = buscar_estado_base(BASE_IZQUIERDA, 0,
                               SALIDA_SWITCH_MAX, PASO_LENTO_US, false, 0);
        printf("Cruce del switch: %ld pasos hasta LIBRE (0).\n",
               (long)labs((long)(base_pulsos-inicio)));
        if (r != BUS_SI) {
            printf("ERROR: No se logro cruzar al lado izquierdo libre.\n");
            return false;
        }
    }

    // Estamos del lado izquierdo, con switch=0.
    if (!mover(&base, BASE_IZQUIERDA, SALIDA_MARGEN, false)) return false;
    if (!switch_estable(base.sw, 0)) {
        printf("ERROR: Switch no permanece libre tras el margen.\n");
        return false;
    }
    printf("Aproximacion lenta FINAL hacia DERECHA (igual que calibracion original)...\n");
    r = buscar_estado_base(BASE_DERECHA, 1,
                           APROX_FINAL_MAX, PASO_LENTO_US, false, 0);
    if (r != BUS_SI) {
        printf("ERROR: No se detecto la segunda aproximacion.\n");
        return false;
    }

    printf("Aplicando correccion frontal original: %ld pulsos DIR=1.\n",
           (long)correccion_base);
    if (!mover(&base, BASE_DERECHA, (int)correccion_base, false)) return false;

    base_pulsos = 0;
    base_objetivo_grados = 0.0;
    base_home = true;
    frente_provisional = false;
    preparado_base = false;
    num_homes++;
    printf("\nHOME BASE COMPLETADO | 0.000 grados | numero %u\n", num_homes);
    printf("HOME BASE validado. Siguiente: HOME del HOMBRO sin mover a READY.\n");
    return true;
}

void modo_manual_base_impl(void)
{
    // Referencia temporal cero al ENTRAR al modo manual, no al encender.
    int32_t origen = base_pulsos;
    printf("\n===== RECUPERACION MANUAL DE HOME =====\n");
    printf("Este punto se toma como 0.000 grados RELATIVOS.\n");
    printf("r: +30 grados | l: -30 grados (SIN confirmacion)\n");
    printf("p: ver desplazamiento relativo | q: salir | x: PARO\n");
    printf("No hay HOME hasta detectar una nueva transicion del switch.\n");
    printf("Los limites de base -140/+105 no aplican durante recuperacion HOME.\n");

    for (;;) {
        if (paro) return;
        printf("[MANUAL] relativo=%+.3f grados | pulsos=%+ld | switch=%d\n",
               grados_base(base_pulsos - origen),
               (long)(base_pulsos - origen), gpio_get_level(base.sw));
        printf("Comando manual (r/l/p/q/x): ");
        char c = leer_char();
        if (c == 'x') return;
        if (c == 'q') {
            printf("Salida de recuperacion; HOME sigue sin validar.\n");
            return;
        }
        if (c == 'p') continue;
        if (c != 'r' && c != 'l') {
            printf("Usa r, l, p, q o x.\n");
            continue;
        }
        int dir = (c == 'r') ? BASE_DERECHA : BASE_IZQUIERDA;
        habilitar(&base);
        printf("Buscando hasta 30 grados %s, x=PARO.\n",
               dir == BASE_DERECHA ? "DERECHA" : "IZQUIERDA");

        // Si ya esta presionado, el tramo se usa para LIBERAR (1 -> 0)
        // y no se confunde nivel HIGH con una deteccion nueva.
        bool activo = switch_estable(base.sw, 1);
        int estado_buscado = activo ? 0 : 1;
        bus_resultado_t r = buscar_estado_base(
            dir, estado_buscado, MANUAL_PULSOS, PASO_HOME_US, true, origen);
        if (r == BUS_ERR) return;
        printf("Tramo: relativo=%+.3f grados | pulsos=%+ld | switch=%d\n",
               grados_base(base_pulsos - origen),
               (long)(base_pulsos - origen), gpio_get_level(base.sw));

        if (r != BUS_SI) {
            printf("No hubo cambio al estado solicitado en este tramo.\n");
            continue;
        }
        if (activo) {
            printf("Switch LIBRE (0). Ya puedes buscar una nueva activacion.\n");
            continue;
        }

        printf("Nueva activacion del microswitch detectada.\n");
        if (finalizar_home_base(dir)) return;
        if (paro) return;
        printf("Afinado no completado; sigue en recuperacion manual.\n");
    }
}

void modo_manual_base(void)
{
    printf("UI:HOME_RECUP_MANUAL_INICIO\n");
    modo_manual_base_impl();
    printf("UI:HOME_RECUP_MANUAL_FIN\n");
}

bool home_base(void)
{
    // Se puede entrar sin f; f es opcional, nunca sustituye HOME.
    // Si hay HOME conocido lejos de cero, volver mediante b.
    if (base_home && base_pulsos != 0) {
        printf("Primero regresa la base a cero con b.\n");
        return false;
    }
    if (paro) {
        printf("PARO activo: reinicia ESP32 y comprueba el mecanismo.\n");
        return false;
    }

    printf("\n===== HOME BASE =====\n");
    printf("Elige direccion para la busqueda inicial:\n");
    printf("l=IZQUIERDA (DIR=0) | r=DERECHA (DIR=1) | x=PARO\n");
    int dir;
    for (;;) {
        char c = leer_char();
        if (c == 'x') return false;
        if (c == 'l') { dir = BASE_IZQUIERDA; break; }
        if (c == 'r') { dir = BASE_DERECHA; break; }
        printf("Usa l, r o x.\n");
    }

    if (!confirmar(dir == BASE_DERECHA
                  ? "Buscar HOME hacia DERECHA, max 120 grados de base?"
                  : "Buscar HOME hacia IZQUIERDA, max 120 grados de base?"))
        return false;

    base_home = false;
    preparado_base = false;
    habilitar(&base);
    bool segunda_usada = false;

    for (;;) {
        printf("BUSCANDO hacia %s: hasta %d pulsos del motor. x=PARO\n",
               dir == BASE_DERECHA ? "DERECHA" : "IZQUIERDA",
               BUSQUEDA_MAX_PULSOS);
        // No declarar HOME solo por leer HIGH: puede ser un cable NC abierto.
        // Si comienza activo, la rutina de afinado exigira primero LOW y
        // luego una nueva transicion LOW -> HIGH por la derecha.
        bus_resultado_t r = switch_estable(base.sw, 1)
            ? BUS_SI
            : buscar_estado_base(dir, 1, BUSQUEDA_MAX_PULSOS,
                                 PASO_HOME_US, false, 0);
        if (r == BUS_ERR) return false;

        if (r == BUS_SI) {
            if (finalizar_home_base(dir)) return true;
            if (paro) return false;
            printf("HOME sin finalizar. No se han aplicado 90 pulsos.\n");
            printf("j=recuperacion manual | q=salir | x=PARO\n");
            for (;;) {
                char c = leer_char();
                if (c == 'x') return false;
                if (c == 'j') { modo_manual_base(); return base_home; }
                if (c == 'q') return false;
                printf("Usa j, q o x.\n");
            }
        }

        printf("No se encontro el switch en esta busqueda.\n");
        if (frente_provisional)
            printf("Posicion provisional: %+.3f grados.\n",
                   grados_base(base_pulsos));
        else
            printf("Movimiento desde encendido: %+.3f grados (NO absoluto).\n",
                   grados_base(base_pulsos));
        printf("Punto actual conservado como inicio para recuperacion manual.\n");
        printf("Que deseas hacer?\n");
        if (!segunda_usada)
            printf("o: buscar en sentido contrario (con confirmacion)\n");
        printf("j: busqueda manual en tramos de 30 grados SIN confirmacion\n");
        printf("q: salir | x: PARO\n");

        bool siguiente_intento = false;
        for (;;) {
            char c = leer_char();
            if (c == 'x') return false;
            if (c == 'q') return false;
            if (c == 'j') { modo_manual_base(); return base_home; }
            if (c == 'o' && !segunda_usada) {
                int opuesto = !dir;
                if (!confirmar(opuesto == BASE_DERECHA
                              ? "Repetir busqueda hacia DERECHA?"
                              : "Repetir busqueda hacia IZQUIERDA?")) {
                    if (paro) return false;
                    printf("Busqueda contraria cancelada.\n");
                    continue;
                }
                dir = opuesto;
                segunda_usada = true;
                siguiente_intento = true;
                break;
            }
            printf("Selecciona una de las opciones mostradas.\n");
        }
        if (siguiente_intento) continue;
    }
}

bool mover_base_a(double deg)
{
    if (!base_home) {
        printf("ERROR: HOME BASE obligatorio. Usa h.\n");
        return false;
    }
    // No solicitar confirmacion especial cuando el codo esta fuera de READY.
    // Conservamos los limites y las validaciones automaticas de postura/ruta
    // que se ejecutan a continuacion, antes de generar pulsos de BASE.
    // Limites ABSOLUTOS por lado, configurables sin cambiar HOME.
    if (!isfinite(deg) || deg > BASE_LIMITE_DERECHA_GRADOS ||
        deg < BASE_LIMITE_IZQUIERDA_GRADOS) {
        printf("ERROR: destino BASE %+.3f fuera de [%.3f, %.3f] grados. Sin movimiento.\n",
               deg, BASE_LIMITE_IZQUIERDA_GRADOS, BASE_LIMITE_DERECHA_GRADOS);
        return false;
    }
    int32_t destino = pulsos_base(deg);
    if (destino > BASE_LIMITE_DERECHA_PULSOS ||
        destino < BASE_LIMITE_IZQUIERDA_PULSOS) {
        printf("ERROR: destino excede limites de pulsos BASE configurados.\n");
        return false;
    }
    // MOV1 inactivo: se omiten la barrera especial de BASE y su
    // bloqueo si CODO PWM esta apagado. MOV4 permanece vigente.
    if (MOV1_ACTIVO && hombro_home && !servo_codo.pwm_activo &&
        hombro_rel_pulsos >= M1_BASE_H_MIN &&
        hombro_rel_pulsos <= M1_BASE_H_MAX) {
        printf("RESTRICCION MOV 1 BASE: CODO sin PWM; posicion desconocida.\n");
        return false;
    }
    if (hombro_home && servo_codo.pwm_activo) {
        const double c=codo_grados_de_pwm(servo_codo.us_actual);
        if (!restricciones_postura_admisible(hombro_rel_pulsos, deg, c))
            return false; // MOV 2 y 4: sin cambios
        if (MOV1_ACTIVO &&
            !m1_recorrido_base_admisible(hombro_rel_pulsos,c,
                                         base_objetivo_grados,deg)) return false;
    }
    if (!perfil_recorrido_base_admisible(base_objetivo_grados,deg))
        return false;
    const double nominal_anterior = base_objetivo_grados;
    const int32_t pulsos_anteriores = base_pulsos;
    int32_t delta = destino - base_pulsos;
    if (!delta) {
        // Puede ser el mismo micropaso tras una nueva consigna nominal.
        base_objetivo_grados = deg;
        printf("BASE: sin pasos adicionales; consigna nominal=%+.3f grados; estimado=%+.3f grados.\n",
               base_objetivo_grados, grados_base(base_pulsos));
        return true;
    }
    int dir = (delta > 0) ? BASE_DERECHA : BASE_IZQUIERDA;
    int n = (delta > 0) ? (int)delta : (int)(-delta);
    habilitar(&base);
    printf("BASE: consigna %+.3f -> %+.3f grados (cambio %+0.3f).\n",
           nominal_anterior, deg, deg-nominal_anterior);
    printf("BASE: estimado por pasos %+.3f -> %+.3f grados | %d micropasos.\n",
           grados_base(base_pulsos), grados_base(destino), n);
    if (!mover(&base, dir, n, true)) {
        // Si hubo un paro o fallo a mitad, la posicion fisica puede ser
        // desconocida. No conservar una meta que NO fue completada.
        base_objetivo_grados = grados_base(base_pulsos);
        if (!paro) paro_total();
        return false;
    }
    base_objetivo_grados = deg;
    printf("BASE: final nominal %+.3f grados; estimada por pasos %+.3f grados.\n",
           base_objetivo_grados, grados_base(base_pulsos));
    ++num_movimientos;
    printf("[REGISTRO #%05lu] BASE %s | nominal %+.3f -> %+.3f grados (delta %+.3f); "
           "pulsos %ld -> %ld; estimado final %+.3f grados.\n",
           num_movimientos, dir==BASE_DERECHA ? "DERECHA" : "IZQUIERDA",
           nominal_anterior, base_objetivo_grados, base_objetivo_grados-nominal_anterior,
           (long)pulsos_anteriores, (long)base_pulsos, grados_base(base_pulsos));
    printf("  HOMBRO: %+.3f grados operativos; CODO: %s",
           grados_hombro(hombro_rel_pulsos-HOMBRO_READY_PULSOS),
           servo_codo.pwm_activo ? "PWM activo, " : "PWM desactivado, ");
    if (servo_codo.pwm_activo)
        printf("%.3f grados nominales.\n",codo_grados_de_pwm(servo_codo.us_actual));
    else
        printf("posicion real desconocida.\n");
    printf("  NOTA: base/hombro calculados por pasos; sin encoder de posicion real.\n");
    return true;
}

bool mover_base_incremental(double incremento)
{
    if (!base_home || !hombro_home || !preparado_base || paro) {
        printf("BASE INCREMENTAL: primero HOME BASE/HOMBRO y modo PREPARADO.\n");
        return false;
    }
    if (!isfinite(incremento)) return false;
    const double antes=base_objetivo_grados;
    const double despues=antes+incremento;
    printf("BASE INCREMENTAL: %+.3f grados %+.3f grados => %+.3f grados nominales.\n",
           antes, incremento, despues);
    if (despues < BASE_LIMITE_IZQUIERDA_GRADOS ||
        despues > BASE_LIMITE_DERECHA_GRADOS) {
        printf("LIMITE BASE [%.3f, %.3f]: orden rechazada, permanece en %+.3f grados nominales.\n",
               BASE_LIMITE_IZQUIERDA_GRADOS, BASE_LIMITE_DERECHA_GRADOS,
               base_objetivo_grados);
        return false;
    }
    return mover_base_a(despues);
}

void calibrar_base(void)
{
    if (!base_home || preparado_base || base_pulsos != 0) {
        printf("Calibracion disponible solo en HOME BASE a 0.\n");
        return;
    }
    int32_t antes = correccion_base;
    int32_t ajuste = 0;
    printf("\nCALIBRACION BASE | anterior=%ld pasos.\n", (long)antes);
    printf("+/-: 1 paso | a/d: 10 pasos | p: ver | s: guardar ajuste | x: PARO\n");
    for (;;) {
        char c = leer_char();
        if (c == 'x') return;
        if (c == 's') {
            correccion_base = antes + ajuste;
            base_pulsos = 0;
            base_objetivo_grados = 0.0;
            printf("Anterior=%ld | ajuste=%+ld | nuevo=%ld pasos.\n",
                   (long)antes, (long)ajuste, (long)correccion_base);
            printf("Cambio angular: %+.3f grados. Solo RAM.\n",
                   grados_base(ajuste));
            return;
        }
        int n = (c == '+' ? 1 : c == '-' ? -1 :
                 c == 'a' ? 10 : c == 'd' ? -10 : 0);
        if (c != 'p' && n == 0) continue;
        if (n != 0) {
            int32_t nuevo = ajuste + n;
            if (nuevo > 30 || nuevo < -30 || (antes + nuevo) <= 0) {
                printf("Limite de ajuste +/-30 pasos.\n");
                continue;
            }
            if (!mover(&base, n > 0 ? BASE_DERECHA : BASE_IZQUIERDA,
                       n > 0 ? n : -n, false)) return;
            ajuste = nuevo;
        }
        printf("Ajuste=%+ld pasos (%+.3f grados) | nuevo=%ld\n",
               (long)ajuste, grados_base(ajuste), (long)(antes + ajuste));
    }
}

void liberar_base_para_alinear(void)
{
    if (paro) {
        printf("PARO total activo. Reinicia para continuar.\n");
        return;
    }
    if (base_home) {
        printf("HOME valido: no liberar la base mediante u.\n");
        return;
    }
    gpio_set_level(base.step, 0);
    gpio_set_level(base.en, 1);
    base_driver_habilitado = false;
    frente_provisional = false;
    printf("BASE DESHABILITADA. Alinea fisicamente el frente y envia f.\n");
    printf("Comprueba que ningun cable se tense durante el ajuste.\n");
}

void fijar_frente_provisional(void)
{
    if (paro) {
        printf("PARO activo. Reinicia ESP32 para volver a fijar referencia.\n");
        return;
    }
    if (base_home) {
        printf("HOME ya es valido; f no debe sobrescribirlo.\n");
        return;
    }
    if (base_driver_habilitado) {
        printf("ERROR: Base energizada. No se permite redefinir f.\n");
        printf("No cortes torque sin asegurar antes el mecanismo.\n");
        return;
    }
    base_pulsos = 0;
    base_objetivo_grados = 0.0;
    frente_provisional = true;
    preparado_base = false;
    printf("REFERENCIA PROVISIONAL f: BASE=0; no equivale a HOME.\n");
    printf("Ahora h para buscar el microswitch; j para recuperacion.\n");
}

void base_angulo_con_direccion(double grados)
{
    if (!base_home || !preparado_base || !hombro_home) {
        printf("Primero completa ambos HOME y entra a PREPARADO con s.\n");
        return;
    }
    printf("BASE INCREMENTAL %.1f grados DESDE POSICION ACTUAL: ",grados);
    printf("r=SUMAR (DERECHA) | l=RESTAR (IZQUIERDA) | q=cancelar\n");
    printf("Actual nominal %+.3f grados, limites [%.3f, %.3f].\n",
           base_objetivo_grados, BASE_LIMITE_IZQUIERDA_GRADOS,
           BASE_LIMITE_DERECHA_GRADOS);
    for (;;) {
        char c=leer_char();
        if (c=='q' || c=='x') return;
        if (c=='r') { (void)mover_base_incremental(grados); return; }
        if (c=='l') { (void)mover_base_incremental(-grados); return; }
        printf("Selecciona r, l o q: ");
    }
}

