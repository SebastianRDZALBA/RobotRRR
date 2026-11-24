#ifndef BRAZO_ESTADO_ROBOT_H
#define BRAZO_ESTADO_ROBOT_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "config.h"

/* Tipos conservados literalmente de V6 original. */
typedef enum { BUZZER_HOME, BUZZER_READY, BUZZER_POSICION } buzzer_evento_t;

typedef struct {
    gpio_num_t step;
    gpio_num_t dir;
    gpio_num_t en;
    gpio_num_t sw;
} motor_t;

typedef struct {
    const char *nombre;
    ledc_channel_t canal;
    int gpio;
    int us_actual;              // ultimo PWM solicitado; NO lectura real
    bool pwm_activo;
    int home_us;                // CODO: plegado | PINZA: abierta
    int ready_us;               // CODO: vertical | PINZA: cerrada
    bool home_calibrado;
    bool ready_calibrado;
} servo_t;

typedef struct {
    double q1_deg, q2_deg, q3_deg;
    double base_oper_deg, hombro_oper_deg, codo_nominal_deg;
    double error_reconstruccion_mm;
    bool limites_articulares_generales;
    bool usa_tolerancia_geometrica;
    bool usa_tolerancia_articular;
} ci_solucion_t;

typedef struct {
    int32_t b_pulsos;
    int32_t h_pulsos;
    int c_us;
} ci_estado_t;

typedef struct {
    bool compatible;
    const char *motivo;
    bool tiene_fallo;
    ci_estado_t fallo;
    int eje_fallo;
    unsigned muestras;
    double margen_min_topes_deg;
    bool cerca_referencia_modelada;
} ci_ruta_t;

typedef struct {
    bool modelo_permite_recorrido;
    bool via_ready;
    double costo_grados;
    int orden;
    int orden_segundo;
    const char *motivo;
    double error_cuantizado_mm;
    int rutas_directas_ok;
    unsigned muestras_totales;
    unsigned muestras_elegida;
    double margen_min_topes_deg;
    bool cerca_referencia;
    ci_ruta_t directas[6];
} ci_analisis_t;

typedef struct {
    bool compatible;
    int orden;
    unsigned muestras;
    double costo;
    ci_ruta_t rutas[6];
} ci521_sincronizado_t;

typedef enum {BUS_ERR=-1, BUS_NO=0, BUS_SI=1} bus_resultado_t;

/* Estado compartido: DEFINIDO UNA SOLA VEZ en estado_robot.c. */
extern bool ci521_paro_suave_ocurrio;
extern char ui_linea[120];
extern bool ui_watchdog_activo;
extern bool ui_ping_recibido;
extern TickType_t ui_ultimo_ping;
extern const motor_t base;
extern const motor_t hombro;
extern bool base_home;
extern bool frente_provisional;
extern bool base_driver_habilitado;
extern bool preparado_base;
extern bool paro;
extern bool hombro_home;
extern bool hombro_preparado;
extern int32_t base_pulsos;
extern double base_objetivo_grados;
extern int32_t hombro_rel_pulsos;
extern int32_t correccion_base;
extern unsigned num_homes;
extern servo_t servo_codo;
extern servo_t servo_pinza;
extern bool hombro_provisional;
extern bool transicion_coordinada;
extern bool sistema_cerrado;
extern unsigned long num_movimientos;
extern double codo_angulo_objetivo;
extern const int ci_ordenes[6][3];
extern const char *const ci_nombres_orden[6];
extern const char *const ci_nombres_eje[3];
extern bool servos_pwm_inicializado;
extern bool modo_preparado_codo_calibrado;

#endif /* BRAZO_ESTADO_ROBOT_H */
