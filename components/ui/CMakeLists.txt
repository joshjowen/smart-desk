idf_component_register(SRCS "ui.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_lcd motion board config net height vl53l0x
                       PRIV_REQUIRES esp_timer heap)
