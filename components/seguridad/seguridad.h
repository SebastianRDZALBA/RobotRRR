#ifndef BRAZO_MOD_SEGURIDAD_H
#define BRAZO_MOD_SEGURIDAD_H
#include "estado_robot.h"

double m1_interpolar(double x, double x0, double y0,
                             double x1, double y1);

bool m1_base_config_critica(int32_t h, double c);

bool m1_recorrido_base_admisible(int32_t h, double c,
                                        double desde, double hasta);

double m1_max_h_por_c(double c);

double m1_max_c_por_h(int32_t h);

bool m1_hombro_derecha_valido(int32_t h, double b, double c);

bool m1_comprobar_hombro_derecha(int32_t h, double b, double c);

bool perfil_v2_activo(void);

double perfil_max_codo_en_base(double b);

bool restricciones_postura_evaluar(int32_t h_pulsos,
                                          double b_grados,
                                          double c_grados,
                                          bool evaluar_m1);

bool restricciones_postura_admisible(int32_t h_pulsos,
                                          double b_grados,
                                          double c_grados);

bool restricciones_recorrido_hombro_admisible(int32_t desde,
                                                    int32_t hasta,
                                                    double b_grados,
                                                    double c_grados);

bool restricciones_recorrido_codo_admisible(int desde_us, int hasta_us,
                                                 double b_grados);

bool perfil_codo_admisible(double b, double c);

bool perfil_recorrido_base_admisible(double desde, double hasta);

void mostrar_referencias_mecanicas_v2(void);

bool ci_postura_m2_m4(ci_estado_t e,const char **motivo);

bool ci_base_mov1(int32_t h,double c,double desde,double hasta);

void invalidar_referencias_521(void);

void paro_total(void);

void paro_controlado(const char *motivo);

bool comprobar_x(void);

#endif
