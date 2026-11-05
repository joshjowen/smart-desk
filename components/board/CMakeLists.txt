idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "board.c"
                       INCLUDE_DIRS "include" "${project_dir}/main"
                       REQUIRES esp_lcd esp_driver_gpio esp_driver_i2c esp_driver_spi esp_driver_ledc jd9853 axs5106)
