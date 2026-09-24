idf_component_register(SRCS "gpio_out.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_timer esp_driver_gpio cfg meas_core)
