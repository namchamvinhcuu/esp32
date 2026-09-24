idf_component_register(SRCS "spooler.c"
                       INCLUDE_DIRS "."
                       REQUIRES meas_core
                       PRIV_REQUIRES heap esp_common)
