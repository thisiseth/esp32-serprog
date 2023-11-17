#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

esp_err_t wifi_init(const char *ssid, const char *password, const char *hostname, uint16_t port);

int wifi_getchar(void);
int wifi_putchar(int character);
int wifi_read(void *buffer, size_t element_size, size_t count);
int wifi_write(void *buffer, size_t element_size, size_t count);
#define wifi_flush() (void)0