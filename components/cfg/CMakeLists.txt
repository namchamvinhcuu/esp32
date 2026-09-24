idf_component_register(SRCS "cfg.c"
                       INCLUDE_DIRS "."
                       # channel table baked into the image; edit channels.json
                       # then rebuild+flash (server config-pull comes later)
                       EMBED_TXTFILES "channels.json"
                       PRIV_REQUIRES nvs_flash espressif__cjson esp_hw_support)
