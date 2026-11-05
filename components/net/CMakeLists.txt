idf_component_register(SRCS "net.c" "dns_captive.c"
                       INCLUDE_DIRS "include"
                       REQUIRES config esp_wifi esp_netif
                       PRIV_REQUIRES esp_event esp_timer lwip)
