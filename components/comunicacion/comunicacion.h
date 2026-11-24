#ifndef BRAZO_MOD_COMUNICACION_H
#define BRAZO_MOD_COMUNICACION_H
#include "estado_robot.h"

bool ci_leer_coordenada_mm(const char *nombre, double *valor);

void iniciar_uart(void);

char leer_char(void);

bool confirmar(const char *descripcion);

void mostrar_estado(void);

void menu(void);

void menu_movimientos_base(void);

void menu_movimientos_hombro(void);

void menu_preparado(void);

bool ui_esperar_heartbeat(void);

void ui_terminar_movimiento(void);

void ui_ejecutar_destino(ci_estado_t fin, const char *origen,
                               bool ciclo_pinza);

void ui_planificar_ci(double x, double y, double z);

void ui_despachar(const char *cmd, bool en_preparado);

#endif
