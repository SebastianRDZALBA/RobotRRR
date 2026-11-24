#ifndef BRAZO_ROBOT_INTERNO_H
#define BRAZO_ROBOT_INTERNO_H
#include "estado_robot.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Firmas de las funciones originales para llamadas entre componentes. */
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
double codo_grados_de_pwm(int us);
bool cd_calcular_mm(double base_oper_deg, double hombro_oper_deg,
                           double codo_nom_deg,
                           double *px_mm, double *py_mm, double *pz_mm);
double ci_normalizar_rad(double angulo);
bool ci_limites_generales(double base_deg, double hombro_deg,
                                double codo_deg);
int ci_calcular_mm(double px, double py, double pz,
                          ci_solucion_t soluciones[CI_MAX_SOLUCIONES]);
bool ci_leer_coordenada_mm(const char *nombre, double *valor);
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
bool ci_postura_m2_m4(ci_estado_t e,const char **motivo);
bool ci_base_mov1(int32_t h,double c,double desde,double hasta);
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
void mostrar_cinematica_directa(void);
int codo_pwm_de_grados(double grados);
void registrar_codo(const char *motivo, int anterior, double grados_solicitados);
int servo_min_us(const servo_t *s);
int servo_max_us(const servo_t *s);
bool servo_us_admisible(const servo_t *s, int us);
void iniciar_uart(void);
void invalidar_referencias_521(void);
void paro_total(void);
void paro_controlado(const char *motivo);
bool comprobar_x(void);
bool buzzer_esperar(int ms);
void buzzer_sonar(buzzer_evento_t evento);
char leer_char(void);
bool confirmar(const char *descripcion);
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
bool codo_postura_segura_prehome(void);
bool codo_postura_segura_para_home_hombro(void);
bool codo_postura_segura_para_retorno(void);
bool despeje_hombro_5_si_switch(void);
void iniciar_gpio(void);
bool switch_estable(gpio_num_t pin, int estado);
double grados_base(int32_t p);
int32_t pulsos_base(double deg);
double grados_hombro(int32_t p);
void habilitar(const motor_t *m);
bool paso(const motor_t *m, int dir, int pausa_us);
bool mover(const motor_t *m, int dir, int n, bool rampa);
bool ci522_esperar_interrumpible_ms(unsigned ms);
bool ci522_esperar_interrumpible_us(uint32_t microsegundos);
bool ci523_ciclo_pinza_destino(void);
void ci522_error_en_movimiento(const char *razon);
bool ci522_comprobar_estado_cuantizado(ci_estado_t esperado);
bool ci522_ejecutar(ci_estado_t inicio,ci_estado_t destino,int orden);
void ci522_menu_ejecutar_xyz(void);
bus_resultado_t buscar_estado_base(int dir, int nivel,
                                           int max_pulsos, int pausa_us,
                                           bool mostrar_avance, int32_t origen);
bool finalizar_home_base(int direccion_busqueda);
void modo_manual_base_impl(void);
void modo_manual_base(void);
bool home_base(void);
bool mover_base_a(double deg);
bool mover_base_incremental(double incremento);
bus_resultado_t buscar_estado_hombro(int dir, int nivel,
                                           int max_pulsos, int pausa_us);
bool hombro_mover_a(int32_t destino, bool movimiento_auto_lento);
bool regresar_hombro_a_home(void);
bool home_automatico_hombro(void);
bool mover_hombro_despeje(int dir, int32_t origen);
bool postura_segura_prehome(void);
void reportar_hombro(void);
void modo_fino_hombro(void);
void probar_hombro_10(void);
void mostrar_estado(void);
void calibrar_base(void);
void liberar_base_para_alinear(void);
void fijar_frente_provisional(void);
void verificar_interruptores(void);
void menu(void);
void base_angulo_con_direccion(double grados);
bool pasar_a_preparado(void);
void menu_movimientos_base(void);
void menu_movimientos_hombro(void);
bool robot_a_preparado(void);
bool robot_a_home(void);
void menu_preparado(void);
void preguntar_preparado_despues_home(void);
bool primer_home(void);
bool ui_esperar_heartbeat(void);
void ui_terminar_movimiento(void);
void ui_ejecutar_destino(ci_estado_t fin, const char *origen,
                               bool ciclo_pinza);
void ui_planificar_ci(double x, double y, double z);
void ui_despachar(const char *cmd, bool en_preparado);

#endif /* BRAZO_ROBOT_INTERNO_H */
