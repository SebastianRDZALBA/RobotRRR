#ifndef BRAZO_MOD_SERVOS_H
#define BRAZO_MOD_SERVOS_H
#include "estado_robot.h"

double codo_grados_de_pwm(int us);

int codo_pwm_de_grados(double grados);

void registrar_codo(const char *motivo, int anterior, double grados_solicitados);

int servo_min_us(const servo_t *s);

int servo_max_us(const servo_t *s);

bool servo_us_admisible(const servo_t *s, int us);

uint32_t servo_duty_de_us(int us);

void servos_iniciar(void);

void servos_detener_senal(void);

bool servo_escribir_pwm(servo_t *s, int microsegundos);

bool servo_activar_manual(servo_t *s);

void servo_diagnostico(servo_t *s);

bool servo_mover_pwm_directo(servo_t *s, int destino);

bool codo_mover_home_ready_gradual(int destino);

void servos_leer_calibracion_nvs(void);

void reportar_servos(void);

void servo_menu_manual_impl(servo_t *s, bool recuperacion);

void servo_menu_manual(servo_t *s, bool recuperacion);

void menu_pinza_operacion(void);

#endif
