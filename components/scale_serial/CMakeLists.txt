idf_component_register(SRCS "scale_serial.c" "scale_parse.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_uart cfg meas_core esp_timer
                                     uplink)
