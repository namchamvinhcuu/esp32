idf_component_register(SRCS "mb_rtu.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_uart cfg meas_core esp_timer)
