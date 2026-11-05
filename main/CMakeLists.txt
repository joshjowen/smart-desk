idf_component_register(SRCS "main.c" "console.c"
                       INCLUDE_DIRS "."
                       REQUIRES board motion ui console esp_system esp_app_format)
