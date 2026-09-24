idf_component_register(SRCS "tower_light.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES gpio_out esp_driver_gpio esp_timer
                                     cfg meas_core net_mgr uplink mb_rtu)
