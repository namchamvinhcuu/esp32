idf_component_register(SRCS "i2c_sens.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c cfg meas_core esp_timer)
