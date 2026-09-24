idf_component_register(SRCS "prov_portal.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "portal.html"
                       PRIV_REQUIRES esp_http_server esp_wifi esp_netif
                                     esp_event esp_timer esp_driver_gpio
                                     lwip espressif__cjson cfg net_mgr
                                     meas_core spooler uplink tower_light)
