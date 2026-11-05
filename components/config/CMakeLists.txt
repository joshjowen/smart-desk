idf_component_register(SRCS "config.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash
                       PRIV_REQUIRES mbedtls esp_hw_support)
