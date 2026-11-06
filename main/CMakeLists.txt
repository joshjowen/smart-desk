idf_component_register(SRCS "main.c" "console.c"
                       INCLUDE_DIRS "."
                       REQUIRES board motion height vl53l0x mqtt_ha ui config net web console esp_system esp_app_format app_update esp_timer lwip)
