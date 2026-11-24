#ifndef BRAZO_MOD_TRAYECTORIAS_H
#define BRAZO_MOD_TRAYECTORIAS_H
#include "estado_robot.h"

double ci_b_grados_estado(ci_estado_t e);

double ci_h_grados_estado(ci_estado_t e);

double ci_c_grados_estado(ci_estado_t e);

ci_estado_t ci_cuantizar_estado(double b,double h,double c);

bool ci_igual(ci_estado_t a,ci_estado_t b);

bool ci_topes_cuantizados(ci_estado_t e);

double ci_margen_topes_grados(ci_estado_t e);

bool ci_cerca_referencia(ci_estado_t e);

void ci_muestra(ci_ruta_t *r,ci_estado_t punto);

bool ci_fallar(ci_ruta_t *r,int eje,ci_estado_t punto,
                      const char *motivo);

bool ci_evaluar_eje(ci_estado_t antes,ci_estado_t despues,
                           int eje,ci_ruta_t *r);

ci_ruta_t ci_probar_orden(ci_estado_t inicio,ci_estado_t destino,
                                int numero_orden);

double ci_costo_estados(ci_estado_t a,ci_estado_t b);

ci_analisis_t ci_analizar_solucion(const ci_solucion_t *s,
                                         ci_estado_t inicio,
                                         double px,double py,double pz);

ci_estado_t ci521_interpolar(ci_estado_t a,ci_estado_t b,
                                     unsigned k,unsigned n);

unsigned ci521_muestras_intervalo(ci_estado_t a,ci_estado_t b);

ci_ruta_t ci521_probar_sincronizada(ci_estado_t inicio,
                                           ci_estado_t fin,int orden);

ci521_sincronizado_t ci521_analizar_sincronizada(
                                        ci_estado_t inicio,ci_estado_t fin);

void ci_imprimir_fallo(const ci_ruta_t *r);

bool ci_postura_actual(ci_estado_t *e);

void menu_cinematica_inversa(void);

void menu_sincronizado_521(void);

void ci521_prueba_paro_virtual(void);

bool ci522_esperar_interrumpible_ms(unsigned ms);

bool ci522_esperar_interrumpible_us(uint32_t microsegundos);

bool ci523_ciclo_pinza_destino(void);

void ci522_error_en_movimiento(const char *razon);

bool ci522_comprobar_estado_cuantizado(ci_estado_t esperado);

bool ci522_ejecutar(ci_estado_t inicio,ci_estado_t destino,int orden);

void ci522_menu_ejecutar_xyz(void);

#endif
