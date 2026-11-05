idf_build_get_property(project_dir PROJECT_DIR)

idf_component_register(SRCS "web.c" "web_api.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "."
                       REQUIRES esp_http_server
                       PRIV_REQUIRES config motion net ui json app_update esp_app_format esp_timer esp_system bootloader_support)

# Gzip the single-page web app at build time and embed it.
set(WEB_SRC ${project_dir}/web/index.html)
set(WEB_GZ ${CMAKE_CURRENT_BINARY_DIR}/index.html.gz)
add_custom_command(OUTPUT ${WEB_GZ}
                   COMMAND gzip -9 -n -c ${WEB_SRC} > ${WEB_GZ}
                   DEPENDS ${WEB_SRC}
                   VERBATIM)
add_custom_target(web_gz DEPENDS ${WEB_GZ})
target_add_binary_data(${COMPONENT_TARGET} ${WEB_GZ} BINARY DEPENDS web_gz)
