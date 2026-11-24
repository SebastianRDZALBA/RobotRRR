#ifndef BRAZO_CONFIG_H
#define BRAZO_CONFIG_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"


/* ===== 01. GPIO - BASE ===== */
#define BASE_STEP GPIO_NUM_18
#define BASE_DIR  GPIO_NUM_19
#define BASE_EN   GPIO_NUM_32
#define BASE_SW   GPIO_NUM_26

/* ===== 02. GPIO - HOMBRO ===== */
#define HOMBRO_STEP GPIO_NUM_27
#define HOMBRO_DIR  GPIO_NUM_14
#define HOMBRO_EN   GPIO_NUM_13
#define HOMBRO_SW   GPIO_NUM_25

/* ===== 03. GPIO - SERVOS ===== */
#define CODO_PWM_GPIO GPIO_NUM_16
#define PINZA_PWM_GPIO GPIO_NUM_17

/* ===== 04. GPIO - BUZZER (23, sin cambios) ===== */
#define BUZZER_GPIO GPIO_NUM_23  // salida hacia resistencia/base de 2N2222 (opcion acordada)

/* ===== 05. NEMA17: motor y reduccion de base ===== */
#define PASOS_COMPLETOS 200
#define MICROSTEP 8
#define REDUCCION_BASE 3
#define PULSOS_VUELTA_MOTOR (PASOS_COMPLETOS * MICROSTEP)
#define PULSOS_VUELTA_BASE (PULSOS_VUELTA_MOTOR * REDUCCION_BASE)

/* ===== 06. Referencia HOME de BASE ===== */
#define CORRECCION_FRONTAL 90
#define BASE_DERECHA 1
#define BASE_IZQUIERDA 0
#define BASE_APROX_FINAL BASE_DERECHA

/* ===== 07. Topes finales de BASE ===== */
#define BASE_LIMITE_IZQUIERDA_GRADOS (-140.0)
#define BASE_LIMITE_DERECHA_GRADOS (+105.0)
#define BASE_LIMITE_IZQUIERDA_PULSOS (-1867)
#define BASE_LIMITE_DERECHA_PULSOS (+1400)
#define BASE_LIMITE_IZQUIERDA_ESTIMADO ((double)BASE_LIMITE_IZQUIERDA_PULSOS * 360.0 / PULSOS_VUELTA_BASE)
#define BASE_LIMITE_DERECHA_ESTIMADO ((double)BASE_LIMITE_DERECHA_PULSOS * 360.0 / PULSOS_VUELTA_BASE)

/* ===== 08. Limites de busqueda de HOME de BASE ===== */
#define BUSQUEDA_MAX_PULSOS PULSOS_VUELTA_MOTOR
#define SALIDA_SWITCH_MAX 800 // Maximo 60 grados de base al liberar; nunca indefinido
#define SALIDA_MARGEN 15
#define APROX_FINAL_MAX 140

/* ===== 09. Movimiento manual de BASE ===== */
#define MANUAL_PULSOS 400
#define REPORTE_CADA_PULSOS 67

/* ===== 10. HOMBRO: reduccion, HOME, READY, topes ===== */
#define REDUCCION_HOMBRO 3
#define PULSOS_VUELTA_HOMBRO (PULSOS_VUELTA_MOTOR * REDUCCION_HOMBRO)
#define HOMBRO_DIR_HOME 0             // IZQUIERDA hacia microswitch
#define HOMBRO_DIR_ALEJAR 1           // DERECHA hacia vertical
#define HOMBRO_PASO_10G 133           // 9.975 grados nominales
#define HOMBRO_PASO_1G 13             // 0.975 grados nominales
#define HOMBRO_PASO_05G 7             // 0.525 grados nominales
#define HOMBRO_READY_PULSOS 1320
#define HOMBRO_LIMITE_MIN_PULSOS 130    // +9.750 grados desde HOME
#define HOMBRO_LIMITE_MAX_PULSOS 3067   // +230.025 grados desde HOME
#define HOMBRO_RETORNO_HOME_MAX 130     // Recorrido nominal del limite 130 a HOME
#define HOMBRO_RETORNO_MARGEN_PULSOS 6  // Max 0.450 grados adicionales; parar en SW=1
#define HOMBRO_RETORNO_ESPERA_MS 80    // Dejar estabilizar el switch al acabar el tramo
#define HOMBRO_PASO_20G 267  // 20.025 grados nominales
#define HOMBRO_DESPEJE_MAX 1600
#define HOMBRO_BUSQUEDA_MAX 1600      // 120 grados max. desde punto desconocido
#define HOMBRO_LIBERAR_MAX 400        // 30 grados max. para liberar contacto
#define HOMBRO_APROX_MAX 400          // aprox. lenta al switch
#define HOMBRO_SW_LIBERAR_READY_MAX 200 // validar que SW quede libre al salir

/* ===== 11. Velocidad de motores NEMA y rampas ===== */
#define PASO_HOME_US 5500
#define PASO_LENTO_US 8000
#define PASO_HOMBRO_AUTO_US (2 * PASO_HOME_US)
#define PASO_HOMBRO_APROX_US (2 * PASO_LENTO_US)
#define PASO_MOVIMIENTO_US 4500
#define RAMPA_PULSOS 60
#define PASO_INICIAL_US 8000

/* ===== 12. PWM LEDC y referencias de servos ===== */
#define SERVO_PWM_HZ 50
#define SERVO_PWM_RES LEDC_TIMER_16_BIT
#define SERVO_PWM_MODE LEDC_LOW_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_0
#define CODO_PWM_CAL_MIN_US 500
#define CODO_PWM_CAL_MAX_US 2500
#define PINZA_PWM_CAL_MIN_US 611
#define PINZA_PWM_CAL_MAX_US 1500
#define SERVO_PWM_CENTRO 1500
#define CODO_HOME_US 500
#define CODO_READY_PROVISIONAL_US 1500
#define PINZA_ABIERTA_US 611
#define PINZA_CERRADA_US 1500

/* ===== 13. Trayectorias CD/CI: temporizacion ===== */
#define CI521_TEST_VIRTUAL_MS (12000)
#define CI522_PASO_NEMA_US (8939)
#define CI522_PAUSA_TICK_US (18696)

/* ===== 14. PWM del codo en HOME y READY ===== */
#define CODO_HOME_READY_PASO_PWM_US (8)
#define CODO_HOME_READY_INTERVALO_MS (20)

/* ===== 15. Tiempos de pinza automatica ===== */
#define CI523_PINZA_ABIERTA_TIEMPO_MS (1000)
#define CI523_PINZA_CIERRE_ASENTAMIENTO_MS (250)

/* ===== 16. Comunicacion y supervision USB ===== */
#define UI_WATCHDOG_MS (1500)

/* ===== 17. Dimensiones cinematicas ===== */
#define CD_L1_MM (217.0)
#define CD_L2_MM (105.0)
#define CD_L3_MM (122.0)
#define CD_DELTA2_MM (33.0)
#define CD_DELTA3_MM (23.0)
#define CD_CODO_READY_NOMINAL_GRADOS (90.0)

/* ===== 18. Conversion de angulos matematicos ===== */
#define CD_SIGNO_Q1 (-1.0)
#define CD_SIGNO_Q2 (+1.0)
#define CD_SIGNO_Q3 (-1.0)
#define CD_RAD_POR_GRADO (0.017453292519943295769)

/* ===== 19. Limites MOV2 a MOV4 ===== */
#define PERFIL_HOMBRO_CRITICO_PULSOS 130      // MOV 4: -89.250
#define PERFIL_HOMBRO_M2_PULSOS 2388          // MOV 2: +80.100
#define PERFIL_M2_CODO_MIN (1.980)
#define PERFIL_BASE_CAMBIO_POSITIVO (+84.0)
#define PERFIL_BASE_CAMBIO_CENTRAL (-27.5)
#define PERFIL_BASE_CAMBIO_NEGATIVO (-95.0)
#define PERFIL_CODO_MAX_EXTENDIDO (176.490)
#define PERFIL_CODO_MAX_DESPEJADO (153.000)
#define PERFIL_TOLERANCIA_ANGULO 0.0001

/* ===== 20. MOV1 (inactivo: mismo estado del original) ===== */
#define MOV1_ACTIVO false

/* ===== 21. Datos historicos MOV1 (inactivos) ===== */
#define M1_P1_H 3020        // +127.500 y C=90.000
#define M1_P2_H 3017        // +127.275 y C=93.960
#define M1_P3_H 3000        // +126.000 y C=99.000
#define M1_P4_P5_H 3042     // +129.150: C=99.000 ->108.540
#define M1_P6_P7_H 3067     // +131.025: limite SOLO del HOMBRO
#define M1_P1_C 90.000
#define M1_P2_C 93.960
#define M1_P3_P4_C 99.000
#define M1_P5_C 108.540
#define M1_P7_C 101.520    // REFERENCIA para la frontera de HOMBRO
#define M1_P1_BASE_MIN (-51.000)
#define M1_P1_BASE_MAX (+2.500)
#define M1_TOL_REFERENCIA_C (0.045)
#define M1_BASE_H_MIN M1_P3_H
#define M1_BASE_H_MAX M1_P1_H
#define M1_BASE_C_CENTRO M1_P1_C
#define M1_BASE_C_TOL (1.000)

/* ===== 22. Cinematica inversa: tolerancias y soluciones ===== */
#define CI_MAX_SOLUCIONES 4
#define CI_GRADOS_POR_RAD (1.0/CD_RAD_POR_GRADO)
#define CI_TOL_CARTESIANA_MM (0.0015)
#define CI_TOL_ANGULO_GRADOS (0.0050)   // solo clasificar soluciones teoricas
#define CI_TOL_SOLUCION_REPETIDA (0.0001)
#define CI_ALERTA_REFERENCIA_GRADOS (0.50)

#endif /* BRAZO_CONFIG_H */
