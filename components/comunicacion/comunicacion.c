/*
 * Componente comunicacion: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "comunicacion.h"

bool ci_leer_coordenada_mm(const char *nombre, double *valor)
{
    char linea[48];
    size_t n=0;
    printf("%s en mm (ej. -122.5, ENTER; q=cancelar, x=PARO): ",nombre);
    fflush(stdout);
    for (;;) {
        if (paro || sistema_cerrado) return false;
        uint8_t ch=0;
        if (uart_read_bytes(UART_NUM_0,&ch,1,pdMS_TO_TICKS(100))<=0) {
            continue;
        }
        if ((ch=='q' || ch=='Q') && n==0) {
            printf("\nCalculo cancelado.\n");
            return false;
        }
        if (ch==0x03) {
            paro_controlado("PARO USB al ingresar coordenadas");
            return false;
        }
        if (ch=='x' || ch=='X') {
            paro_total();
            return false;
        }
        if (ch=='!') {
            paro_controlado("Tecla ! durante entrada de coordenadas");
            return false;
        }
        if (ch=='\r' || ch=='\n') {
            if (n==0) continue; // ignorar LF residual de CRLF
            linea[n]='\0';
            char *fin=NULL;
            const double v=strtod(linea,&fin);
            if (fin==linea || *fin!='\0' || !isfinite(v) || fabs(v)>100000.0) {
                printf("\nValor no valido. Repite %s en mm: ",nombre);
                n=0;
                fflush(stdout);
                continue;
            }
            *valor=v;
            printf("%s = %+.6f mm\n",nombre,v);
            return true;
        }
        if (ch==8 || ch==127) {
            if (n>0) n--;
            continue;
        }
        if (ch==' ' || ch=='\t') continue;
        if (ch==',') ch='.'; // admite coma decimal
        if ((ch>='0' && ch<='9') || ch=='-' || ch=='+' || ch=='.') {
            if (n<sizeof(linea)-1) linea[n++]=(char)ch;
            else {
                printf("\nNumero demasiado largo. Repite %s: ",nombre);
                n=0;
                fflush(stdout);
            }
        } else {
            // Invalidar la linea completa, no interpretar parcialmente.
            printf("\nCaracter invalido. Escribe de nuevo %s: ",nombre);
            n=0;
            fflush(stdout);
        }
    }
}

void iniciar_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &cfg));
}

char leer_char(void)
{
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(100)) <= 0) {
            continue;
        }
        if (c=='~') {
            // Conservar el heartbeat durante preguntas y confirmaciones.
            // El watchdog puede estar activo mientras confirmar() usa leer_char().
            ui_ping_recibido=true;
            ui_ultimo_ping=xTaskGetTickCount();
            continue;
        }
        if (c==0x03) {
            paro_controlado("PARO controlado desde interfaz USB");
            return 'x';
        }
        if (c=='@') {
            // Consumir el comando COMPLETO para que sus letras nunca se
            // interpreten accidentalmente como pulsaciones de menu.
            size_t n=0;
            bool fin=false, excesivo=false;
            TickType_t comienzo=xTaskGetTickCount();
            while((TickType_t)(xTaskGetTickCount()-comienzo) < pdMS_TO_TICKS(2500)) {
                uint8_t ch;
                if(uart_read_bytes(UART_NUM_0,&ch,1,pdMS_TO_TICKS(30))<=0)
                    continue;
                if(ch=='\n' || ch=='\r') {fin=true;break;}
                if(ch==0x03 || ch=='!') {
                    paro_controlado("PARO controlado durante orden USB");
                    return 'x';
                }
                if(ch=='~') continue;
                if(ch<' ' || ch>126) {excesivo=true;continue;}
                if(n+1<sizeof(ui_linea)) ui_linea[n++]=(char)ch;
                else excesivo=true;
            }
            ui_linea[n]='\0';
            // Identificacion USB no altera la pregunta activa (HOME, READY, etc.).
            // Se procesa aqui para que @HELLO nunca se interprete como respuesta.
            if(fin && !excesivo && strcmp(ui_linea,"HELLO")==0) {
                printf("UI:PROTOCOLO_V2\n");
                fflush(stdout);
                continue;
            }
            if(!fin || excesivo) {
                printf("UI:ERROR Comando incompleto o demasiado largo.\n");
                return 0;
            }
            return '@';
        }
        if (isspace((unsigned char)c)) continue;
        if (!isprint((unsigned char)c)) continue;
        c = (uint8_t)tolower((unsigned char)c);
        if (c == '!') {
            paro_controlado("Tecla !: paro controlado");
            return 'x'; // centinela de salida; el paro real conserva torque/PWM
        }
        if (c == 'x') {
            paro_total();
            return 'x';
        }
        return (char)c;
    }
}

bool confirmar(const char *descripcion)
{
    printf("\n%s\nEstas seguro? s=SI | n=NO | Esc=PARO CONTROLADO (interfaz) | x=PARO OFF\n", descripcion);
    for (;;) {
        char c = leer_char();
        if (c == 's') {printf("UI:CONFIRMADO Respuesta SI del operador.\n");return true;}
        if (c == 'n') {printf("UI:CANCELADO Respuesta NO del operador.\n");return false;}
        if (c == 'q' || c == 'x') return false;
        printf("Responde s, n o x; Esc desde la interfaz para paro controlado: ");
    }
}

void mostrar_estado(void)
{
    printf("UI:PROTOCOLO_V2\n");
    printf("\n===== ESTADO V6 =====\n");
    printf("BASE: HOME=%s | PREPARADO BASE=%s | frente provisional=%s\n",
           base_home ? "SI" : "NO", preparado_base ? "SI" : "NO",
           frente_provisional ? "SI" : "NO");
    if (base_home)
        printf("BASE: consigna %+.3f grados; estimada %+.3f grados (%ld pulsos). Limites nominales [%.3f, %+.3f] grados.\n",
               base_objetivo_grados,grados_base(base_pulsos),(long)base_pulsos,
               BASE_LIMITE_IZQUIERDA_GRADOS,BASE_LIMITE_DERECHA_GRADOS);
    else
        printf("BASE: SIN HOME. Posicion fisica desconocida.\n");
    reportar_hombro();
    reportar_servos();
    printf("CODO READY: %s | consigna en READY guardado: %s.\n",
           servo_codo.ready_calibrado ? "GUARDADO" : "PENDIENTE",
           modo_preparado_codo_calibrado ? "SI" : "NO");
    printf("BASE HOME efectuados: %u | correccion: %ld pulsos.\n",
           num_homes, (long)correccion_base);
    printf("SW BASE=%d | SW HOMBRO=%d (1=contacto O cable NC abierto).\n",
           gpio_get_level(base.sw), gpio_get_level(hombro.sw));
}

void menu(void)
{
    printf("\n========== BRAZO V5.23 | BASE + HOMBRO + CODO + PINZA ==========\n");
    printf("h : PRIMER HOME (sin referencias) o RETORNO HOME (referenciado)\n");
    printf("s : HOME -> PREPARADO automatico (+1320 hombro, +1500 codo)\n");
    printf("z : REGRESAR A PREPARADO desde postura referenciada\n");
    printf("g/o : REGRESAR ROBOT A HOME -> SALIR o PREPARADO\n");
    printf("e : MOVIMIENTOS CODO 0.5/1/5/10/20/30/45/90 grados, R/L\n");
    printf("En PREPARADO, b: BASE INCREMENTAL 0.5/1/10/20/30; limites por lado.\n");
    printf("n : ver MOVIMIENTOS 1-4 registrados y perfil de despeje V2.\n");
    printf("i : PINZA MG90S ABRIR 80grados (611us) / CERRAR 1500us\n");
    printf("u : liberar SOLO BASE antes de HOME\n");
    printf("f : frente provisional BASE (NO HOME) | a : hombro relativo provisional\n");
    printf("j : recuperacion manual BASE +/-30grados | v : leer SW\n");
    printf("t : HOMBRO +/-10 grados | k : HOMBRO ajuste fino\n");
    printf("y : HOMBRO a READY 1320 | b : BASE a 0 | c: calibrar frente BASE\n");
    printf("p : estado y CD | d: CI secuencial | w: CI sincronizada SIMULADA | r: XYZ MOTOR + PINZA\n");
    printf("m: menu | x: PARO OFF | ! durante movimiento: PARO CON TORQUE\n");
    printf("Base STEP18 DIR19 EN32 SW26, Hombro STEP27 DIR14 EN13 SW25.\n");
    printf("CODO PWM16: HOME500 READY1500 (0..180 nominal) | PINZA PWM17.\n");
    printf("Los servos inician sin PWM; en PRIMER HOME activar codo a 500us.\n");
    printf("CODO manual disponible con limites V2 cuando HOMBRO esta en -89.250.\n");
    printf("==============================================================\n");
}

void menu_movimientos_base(void)
{
    if (paro) return;
    for (;;) {
        if (paro || !base_home || !preparado_base) return;
        printf("\n===== BASE INCREMENTAL V6 =====\n");
        printf("1: +/-0.5 grados | 2: +/-1 grado | 3: +/-10 grados\n");
        printf("4: +/-20 grados | 5: +/-30 grados (r derecha, l izquierda)\n");
        printf("Cada orden PARTE DE LA ULTIMA POSICION. Limite IZQ=%.3f DER=%.3f.\n",
               BASE_LIMITE_IZQUIERDA_GRADOS, BASE_LIMITE_DERECHA_GRADOS);
        printf("0: volver BASE a 0 ABSOLUTO | p: posicion | q: atras | x: PARO\n");
        printf("Consigna actual=%+.3f grados | estimada=%+.3f grados, %ld micropasos.\n",
               base_objetivo_grados,grados_base(base_pulsos),(long)base_pulsos);
        char c=leer_char();
        if (c=='q' || c=='x') return;
        if (c=='p') {
            printf("BASE: consigna nominal %+.3f grados; estimada por pasos %+.3f grados; %ld micropasos.\n",
                   base_objetivo_grados, grados_base(base_pulsos), (long)base_pulsos);
            mostrar_cinematica_directa();
            continue;
        }
        if (c=='0') { (void)mover_base_a(0.0); continue; }
        switch(c) {
            case '1': base_angulo_con_direccion(0.5); break;
            case '2': base_angulo_con_direccion(1.0); break;
            case '3': base_angulo_con_direccion(10.0); break;
            case '4': base_angulo_con_direccion(20.0); break;
            case '5': base_angulo_con_direccion(30.0); break;
            default: printf("Selecciona 0,1,2,3,4,5,p,q o x.\n"); break;
        }
    }
}

void menu_movimientos_hombro(void)
{
    if (paro) return;
    for (;;) {
        if (paro || !base_home || !hombro_home || !preparado_base) return;
        printf("\n===== MOVIMIENTOS DE HOMBRO (0 = READY) =====\n");
        printf("1: 1 micropaso (0.075 grados) | 2: 0.5 grados (7 pulsos)\n");
        printf("3: 1 grado (13 pulsos) | 4: 10 grados (133 pulsos)\n");
        printf("5: 20 grados (267 pulsos)\n");
        printf("0: volver a 0 OPERATIVO (READY=1320 pulsos)\n");
        printf("o: HOME total coordinado (requiere confirmar postura segura)\n");
        printf("p: posicion | q: atras | x: PARO\n");
        char c=leer_char();
        if(c=='q' || c=='x') return;
        if(c=='p') { reportar_hombro(); mostrar_cinematica_directa(); continue; }
        if(c=='0') { (void)hombro_mover_a(HOMBRO_READY_PULSOS, true); continue; }
        if(c=='o') { (void)robot_a_home(); return; }
        int n=0;
        switch(c) {
            case '1': n=1; break;
            case '2': n=HOMBRO_PASO_05G; break;
            case '3': n=HOMBRO_PASO_1G; break;
            case '4': n=HOMBRO_PASO_10G; break;
            case '5': n=HOMBRO_PASO_20G; break;
            default: printf("Selecciona 0,1,2,3,4,5,o,p,q o x.\n"); continue;
        }
        printf("HOMBRO: %d micropasos (%.3f grados nominales).\n",n,grados_hombro(n));
        printf("r=DERECHA (+) | l=IZQUIERDA (-) | q=cancelar | x=PARO\n");
        char d;
        do {
            d=leer_char();
            if(d=='x' || d=='q') break;
            if(d!='r' && d!='l') printf("Elige r, l, q o x.\n");
        } while(d!='r' && d!='l');
        if(d=='x' || d=='q') continue;
        int32_t signo=(d=='r') ? 1 : -1;
        int64_t candidato=(int64_t)hombro_rel_pulsos + (int64_t)signo*n;
        if(candidato < INT32_MIN || candidato > INT32_MAX) {
            printf("ERROR: contador fuera de rango.\n");
            continue;
        }
        // hombro_mover_a() recorta el movimiento al limite seguro.
        (void)hombro_mover_a((int32_t)candidato, false);
        if (!hombro_home) return;
    }
}

void menu_preparado(void)
{
    for (;;) {
        if (paro || !base_home || !hombro_home || !preparado_base) return;
        printf("\n========== MODO PREPARADO V6 ==========\n");
        printf("b: BASE INCREMENTAL (0.5,1,10,20,30 grados R/L; -140/+105 abs.)\n");
        printf("h: MOVIMIENTOS HOMBRO (1 micropaso,0.5,1,10,20 grados)\n");
        printf("e: CODO MG996R: 0.5,1,5,10,20,30,45,90 grados nominales\n");
        printf("n: referencias MOVIMIENTOS 1-4 y limites V2\n");
        printf("i: PINZA MG90S: ABRIR 80grados (611us) / CERRAR 1500us\n");
        printf("z: volver a PREPARADO (secuencia coordinada)\n");
        printf("g: volver a HOME TOTAL y elegir SALIR/PREPARADO\n");
        printf("p: estado/CD | d: CI secuencial | w: CI sincronizada (simulacion) | r: XYZ MOTOR + PINZA\n");
        printf("v: TEST PARO sin motor | !: PARO CON TORQUE | x: PARO OFF\n"
        "q: menu principal\n");
        char c=leer_char();
        if(c=='@') {ui_despachar(ui_linea,true);continue;}
        if(c=='q' || c=='x') return;
        switch(c) {
            case 'b': menu_movimientos_base(); break;
            case 'h': menu_movimientos_hombro(); break;
            case 'e': servo_menu_manual(&servo_codo,false); break;
            case 'n': mostrar_referencias_mecanicas_v2(); break;
            case 'i': servo_menu_manual(&servo_pinza,false); break;
            case 'z': (void)robot_a_preparado(); break;
            case 'g': (void)robot_a_home(); return;
            case 'p': mostrar_estado(); break;
            case 'd': menu_cinematica_inversa(); break;
            case 'w': menu_sincronizado_521(); break;
            case 'r': ci522_menu_ejecutar_xyz(); break;
            case 'v': ci521_prueba_paro_virtual(); break;
            // ! queda reservado como paro controlado GLOBAL en leer_char().
            // s nunca debe usarse como paro: confirma preguntas o entra en READY.

            default: printf("Usa b,h,e,i,z,g,p,d,w,r,v,s,q o x.\n"); break;
        }
    }
}

bool ui_esperar_heartbeat(void)
{
    ui_watchdog_activo=true;
    ui_ping_recibido=false;
    ui_ultimo_ping=xTaskGetTickCount();
    printf("UI:ARM\n");
    fflush(stdout);
    for(int t=0;t<100;t++) {
        if(comprobar_x()) {ui_watchdog_activo=false;return false;}
        if(ui_ping_recibido) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ui_watchdog_activo=false;
    printf("UI:ERROR Sin heartbeat: movimiento NO iniciado.\n");
    printf("UI:IDLE\n");
    return false;
}

void ui_terminar_movimiento(void)
{
    ui_watchdog_activo=false;
    printf("UI:IDLE\n");
    if(!paro && !sistema_cerrado) mostrar_estado();
}

void ui_ejecutar_destino(ci_estado_t fin, const char *origen,
                               bool ciclo_pinza)
{
    if(paro || sistema_cerrado || !preparado_base) {
        printf("UI:RECHAZADO HOME/READY obligatorios.\n");return;
    }
    ci_estado_t inicio={0};
    if(!ci_postura_actual(&inicio) || !ci_topes_cuantizados(fin)) {
        printf("UI:RECHAZADO Referencias o limites generales invalidos.\n");return;
    }
    ci521_sincronizado_t plan=ci521_analizar_sincronizada(inicio,fin);
    if(!plan.compatible) {
        printf("UI:RECHAZADO Ninguna de las 6 microtrayectorias supera las validaciones.\n");
        bool reportado_m2=false, reportado_m4=false, reportado_topes=false;
        for(int i=0;i<6;i++) {
            const ci_ruta_t *r=&plan.rutas[i];
            if(!r->tiene_fallo || !r->motivo) continue;
            const bool m2=strstr(r->motivo,"MOV2")!=NULL;
            const bool m4=strstr(r->motivo,"MOV4")!=NULL;
            const bool otros=!m2&&!m4;
            if((m2&&reportado_m2)||(m4&&reportado_m4)||(otros&&reportado_topes))continue;
            if(m2) reportado_m2=true;
            if(m4) reportado_m4=true;
            if(otros) reportado_topes=true;
            printf("UI:BLOQUEO %s | B=%+.3f H=%+.3f C=%.3f | %s\n",
                   m2?"MOV2":m4?"MOV4":"TOPES/OTRA REGLA",
                   ci_b_grados_estado(r->fallo),
                   ci_h_grados_estado(r->fallo),
                   ci_c_grados_estado(r->fallo),r->motivo);
        }
        return;
    }
    int rutas_ok=0;
    for(int i=0;i<6;i++) if(plan.rutas[i].compatible) rutas_ok++;
    printf("UI:RUTA %s | alternativas del modelo: %d/6\n",
           ci_nombres_orden[plan.orden],rutas_ok);
    double px,py,pz;
    if(!cd_calcular_mm(ci_b_grados_estado(fin),ci_h_grados_estado(fin),
                       ci_c_grados_estado(fin),&px,&py,&pz)) {
        printf("UI:RECHAZADO No se pudo calcular CD.\n");return;
    }
    printf("UI:POS_ANTES B=%+.3f H=%+.3f C=%.3f\n",
           ci_b_grados_estado(inicio),ci_h_grados_estado(inicio),
           ci_c_grados_estado(inicio));
    printf("UI:POS_NUEVA B=%+.3f H=%+.3f C=%.3f\n",
           ci_b_grados_estado(fin),ci_h_grados_estado(fin),
           ci_c_grados_estado(fin));
    printf("\nUI:PROPUESTA %s | B%+.3f H%+.3f C%.3f grados\n",
           origen,ci_b_grados_estado(fin),ci_h_grados_estado(fin),
           ci_c_grados_estado(fin));
    printf("UI:DESTINO XYZ (%+.3f,%+.3f,%+.3f) mm; ruta %s.\n",
           px,py,pz,ci_nombres_orden[plan.orden]);
    printf("AVISO: MOV1 inactivo. MOV2–MOV4 son un modelo parcial; NO hay deteccion de colision 3D.\n");
    if(ciclo_pinza && !servo_pinza.pwm_activo)
        printf("AVISO: la pinza se activara en CERRADA 1500us antes de iniciar.\n");
    if(!confirmar("Autorizar trayectoria USB supervisada y confirmar DESPEJE fisico?")) {
        printf("UI:CANCELADO por operador.\n");return;
    }
    printf("UI:CONFIRMADO Movimiento autorizado por el operador.\n");
    ci_estado_t ahora={0};
    if(!ci_postura_actual(&ahora) || !ci_igual(ahora,inicio) ||
       !preparado_base) {
        printf("UI:RECHAZADO Posicion alterada antes de confirmar.\n");return;
    }
    if(!ui_esperar_heartbeat()) return;
    // El planificador original revalida el recorrido antes de dar pulsos.
    bool ok=true;
    if(ciclo_pinza && !servo_pinza.pwm_activo) {
        ok=servo_escribir_pwm(&servo_pinza,PINZA_CERRADA_US);
        if(ok) ok=ci522_esperar_interrumpible_ms(300);
        if(!ok && !paro) paro_controlado("Fallo al preparar pinza en CI por USB");
    }
    if(ok && !paro) {
        ok=ci522_ejecutar(inicio,fin,plan.orden);
        if(ok && !paro) {
            printf("UI:DESTINO_ALCANZADO segun pasos/PWM, sin sensores absolutos.\n");
            buzzer_sonar(BUZZER_POSICION);
        }
    }
    if(ok && ciclo_pinza && !paro) ok=ci523_ciclo_pinza_destino();
    printf("UI:%s\n",ok && !paro?"MOVIMIENTO_COMPLETADO":"MOVIMIENTO_INTERRUMPIDO");
    ui_terminar_movimiento();
}

void ui_planificar_ci(double x, double y, double z)
{
    if(!isfinite(x) || !isfinite(y) || !isfinite(z)) {
        printf("UI:PLAN_ERROR Coordenadas invalidas.\n");return;
    }
    ci_solucion_t soluciones[CI_MAX_SOLUCIONES];
    int total=ci_calcular_mm(x,y,z,soluciones);
    ci_estado_t inicio={0};
    bool referencia=preparado_base && !paro && !sistema_cerrado &&
                    ci_postura_actual(&inicio);
    printf("UI:PLAN_INICIO %d\n",total);
    for(int i=0;i<total;i++) {
        const ci_solucion_t *sol=&soluciones[i];
        if(!sol->limites_articulares_generales) {
            printf("UI:SOLUCION %d NO TOPES_ARTICULARES\n",i+1);continue;
        }
        ci_estado_t fin=ci_cuantizar_estado(sol->base_oper_deg,
                                           sol->hombro_oper_deg,
                                           sol->codo_nominal_deg);
        if(!ci_topes_cuantizados(fin)) {
            printf("UI:SOLUCION %d NO TOPES_CUANTIZADOS\n",i+1);continue;
        }
        if(!referencia) {
            printf("UI:SOLUCION %d SIN_REFERENCIA HOME_READY_REQUERIDOS\n",i+1);
            continue;
        }
        ci521_sincronizado_t plan=ci521_analizar_sincronizada(inicio,fin);
        if(plan.compatible) {
            int alternativas=0;
            for(int j=0;j<6;j++) if(plan.rutas[j].compatible) alternativas++;
            printf("UI:SOLUCION %d SI ruta=%s alternativas=%d/6\n",
                   i+1,ci_nombres_orden[plan.orden],alternativas);
        } else {
            const char *razon="SIN_RUTA_ADMISIBLE";
            for(int j=0;j<6;j++) {
                if(plan.rutas[j].tiene_fallo && plan.rutas[j].motivo) {
                    razon=plan.rutas[j].motivo;break;
                }
            }
            printf("UI:SOLUCION %d NO %s\n",i+1,razon);
        }
    }
    printf("UI:PLAN_FIN\n");
}

void ui_despachar(const char *cmd, bool en_preparado)
{
    if(!cmd || !*cmd) return;
    if(strcmp(cmd,"HELLO")==0) {
        printf("UI:PROTOCOLO_V2\n");
        fflush(stdout);
        return;
    }
    printf("UI:RECIBIDO %s\n",cmd);
    if(strcmp(cmd,"STATUS")==0) {mostrar_estado();return;}
    double plan_x=0,plan_y=0,plan_z=0;
    char plan_extra=0;
    if(sscanf(cmd,"PLAN CI %lf %lf %lf %c",&plan_x,&plan_y,&plan_z,&plan_extra)==3) {
        ui_planificar_ci(plan_x,plan_y,plan_z);
        return;
    }
    if(strcmp(cmd,"HOME")==0) {
        if(paro || sistema_cerrado) {printf("UI:RECHAZADO PARO activo: recuperar con h desde consola.\n");return;}
        if(base_home && hombro_home) (void)robot_a_home();
        else (void)primer_home();
        return;
    }
    if(strcmp(cmd,"READY")==0) {
        if(!robot_a_preparado()) {
            printf("UI:RECHAZADO No se pudo alcanzar READY.\n");return;
        }
        if(!en_preparado) menu_preparado();
        return;
    }
    if(!preparado_base || !base_home || !hombro_home || paro || sistema_cerrado) {
        printf("UI:RECHAZADO HOME y READY validos obligatorios.\n");return;
    }
    char eje=0, sobra=0;
    double incremento=0.0;
    if(sscanf(cmd,"JOG %c %lf %c",&eje,&incremento,&sobra)==2) {
        if(!isfinite(incremento) || fabs(incremento)>10.00001 ||
           (fabs(incremento)!=0.5 && fabs(incremento)!=1.0 &&
            fabs(incremento)!=5.0 && fabs(incremento)!=10.0)) {
            printf("UI:RECHAZADO Incremento permitido: +/-0.5/1/5/10 grados.\n");return;
        }
        if(!ui_esperar_heartbeat()) return;
        ci_estado_t manual_antes={0};
        bool antes_valido=ci_postura_actual(&manual_antes);
        bool ok=false;
        if(eje=='B') ok=mover_base_incremental(incremento);
        if(eje=='H') {
            int32_t pasos=(int32_t)lround(incremento*PULSOS_VUELTA_HOMBRO/360.0);
            int32_t nuevo=hombro_rel_pulsos+pasos;
            if(nuevo>=HOMBRO_LIMITE_MIN_PULSOS && nuevo<=HOMBRO_LIMITE_MAX_PULSOS)
                ok=hombro_mover_a(nuevo,false);
        }
        if(eje=='C' && servo_codo.pwm_activo) {
            double nuevo=codo_angulo_objetivo+incremento;
            if(nuevo>=0.0 && nuevo<=180.0) {
                int pwm=codo_pwm_de_grados(nuevo);
                // La rueda de CODO en modo normal también usa rampa lenta.
                ok=codo_mover_home_ready_gradual(pwm);
                if(ok)codo_angulo_objetivo=nuevo;
            }
        }
        if(!ok) printf("UI:RECHAZADO JOG no permitido.\n");
        else {
            ci_estado_t manual_despues={0};
            if(antes_valido && ci_postura_actual(&manual_despues)) {
                printf("UI:POS_ANTES B=%+.3f H=%+.3f C=%.3f\n",
                       ci_b_grados_estado(manual_antes),
                       ci_h_grados_estado(manual_antes),
                       ci_c_grados_estado(manual_antes));
                printf("UI:POS_NUEVA B=%+.3f H=%+.3f C=%.3f\n",
                       ci_b_grados_estado(manual_despues),
                       ci_h_grados_estado(manual_despues),
                       ci_c_grados_estado(manual_despues));
            }
            printf("UI:JOG_OK\n");
        }
        ui_terminar_movimiento();
        return;
    }
    if(strcmp(cmd,"GRIP OPEN")==0 || strcmp(cmd,"GRIP CLOSE")==0) {
        if(!servo_pinza.pwm_activo) {
            printf("UI:AVISO Primera activacion de pinza requiere confirmacion.\n");
            if(!servo_activar_manual(&servo_pinza)) {printf("UI:RECHAZADO Pinza no activada.\n");return;}
        }
        const int objetivo=strcmp(cmd,"GRIP OPEN")==0 ?
                           PINZA_ABIERTA_US:PINZA_CERRADA_US;
        if(!servo_mover_pwm_directo(&servo_pinza,objetivo))
            printf("UI:RECHAZADO Orden de pinza.\n");
        else {printf("UI:GRIP_OK %s\n",
                     strcmp(cmd,"GRIP OPEN")==0?"Abierta":"Cerrada");
              mostrar_estado();}
        return;
    }
    double b=0,h=0,c=0,x=0,y=0,z=0;
    int indice=0;
    if(sscanf(cmd,"CD %lf %lf %lf %c",&b,&h,&c,&sobra)==3) {
        if(!ci_limites_generales(b,h,c)) {
            printf("UI:RECHAZADO CD fuera de topes generales.\n");return;
        }
        ci_estado_t fin=ci_cuantizar_estado(b,h,c);
        if(!ci_topes_cuantizados(fin)) {printf("UI:RECHAZADO CD cuantizada.\n");return;}
        ui_ejecutar_destino(fin,"CD ANGULOS DIRECTOS",true);
        return;
    }
    if(sscanf(cmd,"CI %lf %lf %lf %d %c",&x,&y,&z,&indice,&sobra)==4) {
        if(!isfinite(x)||!isfinite(y)||!isfinite(z) ||
           indice<1 || indice>CI_MAX_SOLUCIONES) {
            printf("UI:RECHAZADO Parametros CI invalidos.\n");return;
        }
        ci_solucion_t soluciones[CI_MAX_SOLUCIONES];
        int n=ci_calcular_mm(x,y,z,soluciones);
        if(indice>n || !soluciones[indice-1].limites_articulares_generales) {
            printf("UI:RECHAZADO Solucion CI no admisible.\n");return;
        }
        ci_solucion_t sol=soluciones[indice-1];
        ci_estado_t fin=ci_cuantizar_estado(sol.base_oper_deg,
                                           sol.hombro_oper_deg,
                                           sol.codo_nominal_deg);
        printf("UI:SOLUCION_ELEGIDA %d\n",indice);
        ui_ejecutar_destino(fin,"CI XYZ",true);
        return;
    }
    printf("UI:ERROR Comando no reconocido.\n");
}

