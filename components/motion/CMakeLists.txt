idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "motion.c" "duty.c"
                       INCLUDE_DIRS "include" "${project_dir}/main"
                       REQUIRES esp_timer esp_hw_support
                       PRIV_REQUIRES esp_system)
