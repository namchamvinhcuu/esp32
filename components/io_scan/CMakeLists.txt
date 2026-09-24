idf_component_register(SRCS "io_scan.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio esp_timer cfg meas_core
                                     uplink)
