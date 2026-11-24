idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES config estado_robot base hombro driver_motores servos home_ready seguridad cinematica trayectorias buzzer comunicacion
    PRIV_REQUIRES nvs_flash esp_timer esp_driver_gpio esp_driver_uart esp_driver_ledc
)
