idf_component_register(SRCS "mqtt_link.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES mqtt espressif__cjson cfg meas_core
                                     net_mgr esp_event esp_timer
                                     spooler gpio_out)
