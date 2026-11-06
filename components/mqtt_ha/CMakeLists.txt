idf_component_register(SRCS "mqtt_ha.c"
                       INCLUDE_DIRS "include"
                       REQUIRES config
                       PRIV_REQUIRES mqtt json motion net ui esp_timer esp_app_format mbedtls)
