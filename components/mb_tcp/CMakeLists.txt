idf_component_register(SRCS "mb_tcp.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES lwip cfg meas_core esp_timer)
