#ifndef BRAZO_MOD_HOMBRO_H
#define BRAZO_MOD_HOMBRO_H
#include "estado_robot.h"

bus_resultado_t buscar_estado_hombro(int dir, int nivel,
                                           int max_pulsos, int pausa_us);

bool hombro_mover_a(int32_t destino, bool movimiento_auto_lento);

bool regresar_hombro_a_home(void);

bool home_automatico_hombro(void);

bool mover_hombro_despeje(int dir, int32_t origen);

void reportar_hombro(void);

void modo_fino_hombro(void);

void probar_hombro_10(void);

#endif
