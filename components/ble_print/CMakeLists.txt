idf_component_register(SRCS "ble_print.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES bt nvs_flash esp_timer)
