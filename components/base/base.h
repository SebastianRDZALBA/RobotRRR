#ifndef BRAZO_MOD_BASE_H
#define BRAZO_MOD_BASE_H
#include "estado_robot.h"

bus_resultado_t buscar_estado_base(int dir, int nivel,
                                           int max_pulsos, int pausa_us,
                                           bool mostrar_avance, int32_t origen);

bool finalizar_home_base(int direccion_busqueda);

void modo_manual_base_impl(void);

void modo_manual_base(void);

bool home_base(void);

bool mover_base_a(double deg);

bool mover_base_incremental(double incremento);

void calibrar_base(void);

void liberar_base_para_alinear(void);

void fijar_frente_provisional(void);

void base_angulo_con_direccion(double grados);

#endif
