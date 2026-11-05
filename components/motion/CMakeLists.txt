idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "motion.c" "duty.c" "opto.c"
                       INCLUDE_DIRS "include" "${project_dir}/main"
                       REQUIRES esp_timer esp_hw_support esp_driver_gpio esp_driver_spi esp_driver_i2c
                       PRIV_REQUIRES esp_system hal)
# Invariant 9: force the opto outputs off in the panic handler (see opto.c).
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler")
