idf_component_register(
    SRCS "comunicacion.c"
    INCLUDE_DIRS "."
    REQUIRES config estado_robot
    PRIV_REQUIRES esp_timer nvs_flash esp_driver_gpio esp_driver_uart esp_driver_ledc
)
