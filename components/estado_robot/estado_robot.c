/* Estado global original de V6, trasladado sin alterar inicializadores. */
#include "estado_robot.h"

bool ci521_paro_suave_ocurrio = false;

char ui_linea[120] = {0};

bool ui_watchdog_activo = false;

bool ui_ping_recibido = false;

TickType_t ui_ultimo_ping = 0;

const motor_t base = {BASE_STEP, BASE_DIR, BASE_EN, BASE_SW};

const motor_t hombro = {HOMBRO_STEP, HOMBRO_DIR, HOMBRO_EN, HOMBRO_SW};

bool base_home = false;

bool frente_provisional = false;

bool base_driver_habilitado = false;

bool preparado_base = false;

bool paro = false;

bool hombro_home = false;

bool hombro_preparado = false;

int32_t base_pulsos = 0;

double base_objetivo_grados = 0.0;

int32_t hombro_rel_pulsos = 0;

int32_t correccion_base = CORRECCION_FRONTAL;

unsigned num_homes = 0;

servo_t servo_codo = {"MG996R CODO", LEDC_CHANNEL_0, CODO_PWM_GPIO,
                            SERVO_PWM_CENTRO, false, CODO_HOME_US,
                            CODO_READY_PROVISIONAL_US, true, true};

servo_t servo_pinza = {"MG90S PINZA", LEDC_CHANNEL_1, PINZA_PWM_GPIO,
                             SERVO_PWM_CENTRO, false, PINZA_ABIERTA_US,
                             PINZA_CERRADA_US, true, true};

bool hombro_provisional = false;

bool transicion_coordinada = false;

bool sistema_cerrado = false;

unsigned long num_movimientos = 0;

double codo_angulo_objetivo = 90.0;

const int ci_ordenes[6][3]={
    {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
};

const char *const ci_nombres_orden[6]={
    "BASE-HOMBRO-CODO", "BASE-CODO-HOMBRO", "HOMBRO-BASE-CODO",
    "HOMBRO-CODO-BASE", "CODO-BASE-HOMBRO", "CODO-HOMBRO-BASE"
};

const char *const ci_nombres_eje[3]={"BASE","HOMBRO","CODO"};

bool servos_pwm_inicializado = false;

bool modo_preparado_codo_calibrado = false;
