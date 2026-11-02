
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        freertos
        log
        esp_timer
        esp_adc
        esp_driver_gpio
        esp_driver_ledc
        esp_rom
)
