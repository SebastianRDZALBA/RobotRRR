/*
 * Componente buzzer: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "buzzer.h"

bool buzzer_esperar(int ms)
{
    for (int t=0; t<ms; t+=10) {
        if (paro || sistema_cerrado || comprobar_x()) {
            gpio_set_level(BUZZER_GPIO, 0);
            return false;
        }
        int restante=ms-t;
        vTaskDelay(pdMS_TO_TICKS(restante<10 ? restante : 10));
    }
    return true;
}

void buzzer_sonar(buzzer_evento_t evento)
{
    if (paro || sistema_cerrado) return;
    int n=0;
    int on[3]={0}, pausa[3]={0};
    switch (evento) {
        case BUZZER_HOME: n=2; on[0]=170; on[1]=280; pausa[0]=110; break;
        case BUZZER_READY: n=3; on[0]=90; on[1]=90; on[2]=160; pausa[0]=75; pausa[1]=75; break;
        case BUZZER_POSICION: n=1; on[0]=70; break;
        default: return;
    }
    for (int i=0; i<n; ++i) {
        if (paro || comprobar_x()) break;
        gpio_set_level(BUZZER_GPIO, 1);
        bool correcto=buzzer_esperar(on[i]);
        gpio_set_level(BUZZER_GPIO, 0);
        if (!correcto || paro) break;
        if (i+1<n && !buzzer_esperar(pausa[i])) break;
    }
    gpio_set_level(BUZZER_GPIO, 0);
}

