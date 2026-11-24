/*
// longitudes de los brazos
L1=217 mm, L2=105 mm, L3=122 mm.

CINEMATICA DIRECTA
q1 = -Base_operativa          
q2 = +Hombro_operativo        
q3 = 90 grados - Codo_nominal 
d  = -(Delta2 + Delta3) = -56 mm   
r  = L2*sin(q2) + L3*sin(q2+q3)     
Px = cos(q1)*r - sin(q1)*d
Py = sin(q1)*r + cos(q1)*d
Pz = L1 + L2*cos(q2) + L3*cos(q2+q3)  

CINEMATICA INVERSA
h = Pz - L1                           
d = -(Delta2 + Delta3) = -56 mm
R = sqrt(Px^2 + Py^2 - d^2)          
cos(q3) = (R^2 + h^2 - L2^2 - L3^2)/(2*L2*L3)  
q3 = +/-acos(cos(q3))                
q2 = atan2(R,h) - atan2(L3*sin(q3),L2+L3*cos(q3))
q1 = atan2(Py,Px) - atan2(d,R)
*/

// declaraciones de todos los modulos y librerias
#include "robot_interno.h"  

void app_main(void)
{
    iniciar_gpio();  // configura pines de motores, finales de carrera y buzzer
    iniciar_uart();  // abre el puerto serie USB (comunicacion con la interfaz)
    
    esp_err_t e_nvs=nvs_flash_init();  // inicia memoria NVS (guarda calibraciones)
    if (e_nvs != ESP_OK)               // si NVS falla...
    {
        printf("ADVERTENCIA: NVS no disponible (%s). No se borrara ninguna particion.\n",
               esp_err_to_name(e_nvs));  // ...avisa, sin borrar nada
    }

    servos_iniciar();  // configura PWM de codo y pinza (aun sin senal)
    
    if (e_nvs == ESP_OK) servos_leer_calibracion_nvs();  // carga calibracion guardada, si NVS sirve
    printf("\nESP32 | BRAZO V6 | BASE -140/+105 | BUZZER GPIO23 | MOV1 INACTIVO\n");  // version y limites de base
    printf("RETORNO HOMBRO: tras 130 pasos, espera 80ms y hasta 6 micropasos extra (0.450 grados).\n");  // regla de retorno a HOME del hombro
    printf("BASE: 1/8, 3:1, HOME frontal +90 pulsos, IZQ %.3f y DER %.3f grados.\n",BASE_LIMITE_IZQUIERDA_GRADOS, BASE_LIMITE_DERECHA_GRADOS);  // datos y topes de la base
    printf("BASE PREPARADO: incrementos +/-0.5, 1, 10, 20, 30 grados desde ultima consigna.\n");  // pasos permitidos en modo preparado
    printf("BASE: -140 nominal = -140.025 estimado (-1867 pasos); +105 = +1400 pasos.\n");  // equivalencia grados-pasos
    printf("BUZZER ACTIVO GPIO23: HOME, READY y destino CD/CI; manual y telemetria silenciosos.\n");  // cuando suena el buzzer
    printf("HOMBRO: 1/8, 3:1, HOME a izquierda, READY=1320, operacion -89.250..+131.025 deg.\n");  // datos y rango del hombro
    printf("VREF reportado: 1.315 V en ambos TMC2209 (informativo).\n");  // referencia de corriente de los drivers
    printf("CODO GPIO16 (500/1500/2500); PINZA GPIO17 (ABRE611/CIERRA1500).\n");  // pines y anchos de pulso de servos
    printf("MOV1 INACTIVO para ensayos; MOV2/MOV3/MOV4 conservados. Tecla n.\n");  // restricciones mecanicas activas
    printf("CD V6: p | CI secuencial: d | sinc. simulada: w | XYZ con motor: r.\n");  // teclas de cinematica
    printf("READY CODO confirmado por operador: %d us (90 grados nominales).\n",servo_codo.ready_us);  // PWM de READY del codo
    printf("SERVO 5V externo GND comun: sin PWM hasta activacion manual.\n");  // servos sin senal al arrancar
    printf("PWM: CODO %d..%d us; PINZA %d..%d us; pinza SOLO RAPIDA.\n",CODO_PWM_CAL_MIN_US, CODO_PWM_CAL_MAX_US,
           PINZA_PWM_CAL_MIN_US, PINZA_PWM_CAL_MAX_US);  // rangos PWM calibrados
    printf("AVISO: quitar PWM o energia puede permitir movimiento por gravedad.\n");  // advertencia de seguridad
    printf("x=PARO OFF; durante movimiento !=PARO controlado (conserva torque).\n");  // x apaga todo, ! frena conservando torque
    printf("Tras !/x, unico retorno: h=HOME supervisado.\n");  // tras un paro solo se sale con HOME
    printf("V6: w=PLAN SINCRONIZADO SOLO CALCULO; r=XYZ MOTOR + PINZA; v=TEST PARO.\n");  // resumen de comandos V6
    printf("UI USB: comandos @... con ENTER; heartbeat '~'; ! o Ctrl-C = PARO CONTROLADO.\n");  // protocolo de la interfaz
    printf("UI:PROTOCOLO_V2\n");  // identificador que lee la interfaz web
    menu();  // muestra el menu de comandos

    for (;;)
    {
        char c = leer_char();  // espera una tecla (o '@' si llega comando de la interfaz)
        if (c == 'x') continue;  // 'x' sin paro activo: no hace nada
        if (sistema_cerrado)     // tras SALIR, el sistema queda bloqueado
        {
            printf("SALIR completado: motores deshabilitados. Reinicia ESP32 para operar.\n");  // pide reiniciar
            continue;  // ignora la tecla
        }
        if (paro) {  // si hay un paro activo, solo se acepta recuperacion
            if (c=='h' || (c=='@' && strcmp(ui_linea,"HOME")==0)) {  // 'h' o comando HOME de la interfaz
                printf("\nRECUPERACION DESDE PARO %s.\n",ci521_paro_suave_ocurrio?"CONTROLADO !":"EMERGENCIA x");  // indica tipo de paro
                printf("Brazo con posicion real DESCONOCIDA. Comprueba apoyo y despeje.\n");  // advertencia: se perdio la referencia
                if (!confirmar("Intentar PRIMER HOME supervisado desde esta postura?"))  // pide confirmacion al operador
                    continue;  // si no confirma, vuelve a esperar
                // La unica salida del bloqueo es entrar en esta rutina
                // completa. Si falla, se vuelve a enclavar el paro.
                paro=false;  // libera el paro para poder mover
                bool home_ok=primer_home();  // ejecuta el HOME completo
                if (sistema_cerrado) 
                {
                    // a=SALIR es una salida deliberada, no un HOME fallido.
                    printf("UI:SALIDA_SEGURA NEMA_OFF PWM_OFF REFERENCIAS_INVALIDAS\n");  // avisa a la interfaz
                    continue;  // vuelve al bucle
                }
                if (!home_ok || !base_home || !hombro_home || paro)  // si HOME no quedo completo
                {
                    if(!paro) paro_controlado("HOME de recuperacion no completado");  // vuelve a enclavar el paro
                } 
                else 
                {
                    printf("HOME DE RECUPERACION COMPLETADO. Referencias restablecidas.\n");  // exito
                }
            } 
            else if(c=='x')  // 'x' durante un paro
            {
                printf("PARO x: actuadores ya deshabilitados.\n");  // ya estaba apagado
            } 
            else  // cualquier otra tecla
            {
                printf("PARO ENCLAVADO. Solo h=HOME supervisado o x=PARO OFF.\n");  // recuerda como salir
            }
            continue;  // en paro no se ejecuta nada mas
        }
        if(c=='@') {ui_despachar(ui_linea,false);continue;}  // comando de la interfaz: lo interpreta y ejecuta
        switch (c)  // teclado manual: cada letra es un comando
        {
            case 'u': liberar_base_para_alinear(); break;  // suelta la base para alinearla a mano
            case 'f': fijar_frente_provisional(); break;  // marca la posicion actual como frente provisional
            case 'a':  // cero relativo del hombro
                if (!hombro_home) {  // solo si aun no hay HOME valido
                    hombro_rel_pulsos=0;  // pone el contador del hombro en 0
                    hombro_provisional=true;  // lo marca como provisional
                    printf("HOMBRO: 0 relativo PROVISIONAL; no equivale a HOME.\n");  // aviso
                } else printf("HOMBRO ya tiene HOME valido.\n");  // ya tenia HOME
                break;
            case 'e': servo_menu_manual(&servo_codo,false); break;  // menu manual del codo
            case 'n': mostrar_referencias_mecanicas_v2(); break;  // muestra referencias y limites mecanicos
            case 'i': servo_menu_manual(&servo_pinza,false); break;  // menu manual de la pinza
            case 'v': verificar_interruptores(); break;  // prueba los finales de carrera
            case 'j':  // HOME manual de la base
                if (base_home) printf("HOME BASE ya valido.\n");  // ya hecho
                else modo_manual_base();  // modo manual para buscar HOME
                break;
            case 'h':  // HOME general
                if (base_home && hombro_home)  // si ya hay referencias validas
                    (void)robot_a_home();  // solo regresa a HOME
                else
                    (void)primer_home();  // si no, hace la busqueda completa
                break;
            case 'k': modo_fino_hombro(); break;  // ajuste fino del hombro
            case 't': probar_hombro_10(); break;  // prueba de movimiento corto del hombro
            case 'y':  // hombro a posicion READY
                (void)hombro_mover_a(HOMBRO_READY_PULSOS, true);  // mueve al valor READY (modo lento)
                break;
            case 'o':  // regresa a HOME
                (void)robot_a_home();
                break;
            case 'z':  // pasa a READY y abre su menu
                if (robot_a_preparado()) menu_preparado();  // solo si llego a READY
                break;
            case 'g':  // igual que 'o': regresa a HOME
                (void)robot_a_home();
                break;
            case 'b':  // base a cero (frente)
                if (base_home && mover_base_a(0.0)) {  // requiere HOME de base
                    preparado_base = false;  // la base deja de estar "preparada"
                    printf("BASE en cero. El HOMBRO no se movio.\n");  // confirma
                }
                break;
            case 's':  // menu preparado
                if (preparado_base && base_home && hombro_home) menu_preparado();  // ya esta preparado: abre menu
                else if (pasar_a_preparado()) menu_preparado();  // si no, se prepara primero
                break;
            case '1': case '3': case '4': case '9': case '5': case '0':  // teclas de movimiento de base
                printf("Movimientos de BASE: primero s, luego b en MENU PREPARADO.\n");  // solo valen dentro del menu preparado
                break;
            case 'c': calibrar_base(); break;  // calibracion de la base
            case 'p': mostrar_estado(); break;  // estado actual + cinematica directa (XYZ)
            case 'd': menu_cinematica_inversa(); break;  // cinematica inversa secuencial
            case 'w': menu_sincronizado_521(); break;  // plan sincronizado (solo calculo, sin mover)
            case 'r': ci522_menu_ejecutar_xyz(); break;  // ejecuta XYZ con motores + ciclo de pinza
            case 'm': menu(); break;  // vuelve a mostrar el menu
            default: printf("Comando no reconocido. m=menu.\n"); break;  // tecla invalida
        }
    }
}