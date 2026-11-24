#ifndef BRAZO_MOD_CINEMATICA_H
#define BRAZO_MOD_CINEMATICA_H
#include "estado_robot.h"

bool cd_calcular_mm(double base_oper_deg, double hombro_oper_deg,
                           double codo_nom_deg,
                           double *px_mm, double *py_mm, double *pz_mm);

double ci_normalizar_rad(double angulo);

bool ci_limites_generales(double base_deg, double hombro_deg,
                                double codo_deg);

int ci_calcular_mm(double px, double py, double pz,
                          ci_solucion_t soluciones[CI_MAX_SOLUCIONES]);

void mostrar_cinematica_directa(void);

#endif
