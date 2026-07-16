idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES vfs esp_netif esp_wifi esp_driver_spi esp_driver_usb_serial_jtag esp_driver_uart)

if(CONFIG_ESP32_SERPROG_WIFI_ENABLED)
    target_sources(${COMPONENT_TARGET} PRIVATE "wifi.c")
endif()
