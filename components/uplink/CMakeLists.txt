idf_component_register(SRCS "uplink.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_http_client espressif__cjson cfg meas_core
                                     spooler net_mgr esp_timer ble_print mbedtls
                                     gpio_out mqtt_link wdt_util)
