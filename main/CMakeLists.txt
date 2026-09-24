idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_timer
                                     cfg meas_core net_mgr mb_tcp mb_rtu
                                     i2c_sens io_scan gpio_out scale_serial
                                     spooler uplink mqtt_link ble_uart ble_central
                                     ble_print prov_portal tower_light diag)
