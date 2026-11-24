/*
 * Componente driver_motores: funciones V6 reorganizadas sin reescribir su logica.
 * Los cuerpos de todas las funciones coinciden con main.c original.
 */
#include "robot_interno.h"
#include "driver_motores.h"

void iniciar_gpio(void)
{
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << BASE_EN) | (1ULL << HOMBRO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&en));
    gpio_set_level(BASE_EN, 1);
    gpio_set_level(HOMBRO_EN, 1);

    gpio_config_t salidas = {
        .pin_bit_mask = (1ULL << BASE_STEP) | (1ULL << BASE_DIR) |
                        (1ULL << HOMBRO_STEP) | (1ULL << HOMBRO_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&salidas));
    gpio_set_level(BASE_STEP, 0);
    gpio_set_level(BASE_DIR, 0);
    gpio_set_level(HOMBRO_STEP, 0);
    gpio_set_level(HOMBRO_DIR, 0);

    gpio_config_t buzzer_salida = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&buzzer_salida));
    gpio_set_level(BUZZER_GPIO, 0);

    gpio_config_t entradas = {
        .pin_bit_mask = (1ULL << BASE_SW) | (1ULL << HOMBRO_SW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&entradas));
}

bool switch_estable(gpio_num_t pin, int estado)
{
    for (int i = 0; i < 3; ++i) {
        if (gpio_get_level(pin) != estado) return false;
        esp_rom_delay_us(1500);
    }
    return true;
}

double grados_base(int32_t p)
{
    return (double)p * 360.0 / PULSOS_VUELTA_BASE;
}

int32_t pulsos_base(double deg)
{
    double p = deg * PULSOS_VUELTA_BASE / 360.0;
    return (int32_t)(p >= 0 ? p + 0.5 : p - 0.5);
}

double grados_hombro(int32_t p)
{
    return (double)p * 360.0 / PULSOS_VUELTA_HOMBRO;
}

void habilitar(const motor_t *m)
{
    gpio_set_level(m->en, 0);
    if (m == &base) base_driver_habilitado = true;
    vTaskDelay(pdMS_TO_TICKS(100));
}

bool paso(const motor_t *m, int dir, int pausa_us)
{
    if (comprobar_x()) return false;
    gpio_set_level(m->dir, dir);
    esp_rom_delay_us(20);
    gpio_set_level(m->step, 1);
    esp_rom_delay_us(10);
    gpio_set_level(m->step, 0);
    esp_rom_delay_us(pausa_us);

    if (m == &base) {
        base_pulsos += (dir == BASE_DERECHA ? 1 : -1);
    } else {
        hombro_rel_pulsos += (dir == 1 ? 1 : -1);
    }
    return true;
}

bool mover(const motor_t *m, int dir, int n, bool rampa)
{
    for (int i = 0; i < n; ++i) {
        int us = PASO_LENTO_US;
        if (rampa) {
            int t = i;
            int restantes = n - i - 1;
            if (restantes < t) t = restantes;
            if (t > RAMPA_PULSOS) t = RAMPA_PULSOS;
            us = PASO_INICIAL_US -
                 (PASO_INICIAL_US - PASO_MOVIMIENTO_US) * t / RAMPA_PULSOS;
        }
        if (!paso(m, dir, us)) return false;
        if ((i % 32) == 31) vTaskDelay(1);
    }
    return true;
}

void verificar_interruptores(void)
{
    printf("SW BASE=%d | SW HOMBRO=%d  (1=activado O cable NC abierto)\n",
           gpio_get_level(base.sw), gpio_get_level(hombro.sw));
}

