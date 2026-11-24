#ifndef BRAZO_MOD_DRIVER_MOTORES_H
#define BRAZO_MOD_DRIVER_MOTORES_H
#include "estado_robot.h"

void iniciar_gpio(void);

bool switch_estable(gpio_num_t pin, int estado);

double grados_base(int32_t p);

int32_t pulsos_base(double deg);

double grados_hombro(int32_t p);

void habilitar(const motor_t *m);

bool paso(const motor_t *m, int dir, int pausa_us);

bool mover(const motor_t *m, int dir, int n, bool rampa);

void verificar_interruptores(void);

#endif
