idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs driver esp_netif esp_wifi)

if(CONFIG_ESP32_SERPROG_WIFI_ENABLED)
    target_sources(${COMPONENT_TARGET} PRIVATE "wifi.c")
endif()
