#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "serprog.h"

#ifdef CONFIG_IDF_TARGET_ESP32
    #define EEPROM_HOST  HSPI_HOST
    #define PIN_NUM_MISO 18
    #define PIN_NUM_MOSI 23
    #define PIN_NUM_CLK  19
    #define PIN_NUM_CS   13

#elif defined CONFIG_IDF_TARGET_ESP32S2
    #define EEPROM_HOST  SPI2_HOST
    #define PIN_NUM_MISO 37
    #define PIN_NUM_MOSI 35
    #define PIN_NUM_CLK  36
    #define PIN_NUM_CS   34

#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
    #define EEPROM_HOST  SPI2_HOST
    #define PIN_NUM_MISO 2
    #define PIN_NUM_MOSI 7
    #define PIN_NUM_CLK  6
    #define PIN_NUM_CS   10

#elif defined CONFIG_IDF_TARGET_ESP32S3
    #define EEPROM_HOST  SPI2_HOST
    #define PIN_NUM_MISO 5
    #define PIN_NUM_MOSI 16
    #define PIN_NUM_CLK  15
    #define PIN_NUM_CS   4

#endif

#ifdef CONFIG_ESP32_SERPROG_WIFI_ENABLED
    #include "wifi.h"

    #define USE_WIFI

    #define WIFI_SSID     CONFIG_ESP32_SERPROG_WIFI_SSID
    #define WIFI_PASSWORD CONFIG_ESP32_SERPROG_WIFI_PASSWORD
    #define WIFI_HOSTNAME CONFIG_ESP32_SERPROG_WIFI_HOSTNAME
    #define WIFI_TCP_PORT CONFIG_ESP32_SERPROG_WIFI_TCP_PORT

#elif defined CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    #include "driver/usb_serial_jtag.h"
    #include "esp_vfs_usb_serial_jtag.h"

    #define USE_SERIAL_JTAG

#elif defined CONFIG_ESP_CONSOLE_UART
    #include "driver/uart.h"

    #define USE_HW_UART

    #define HW_UART_NUM CONFIG_ESP_CONSOLE_UART_NUM

    //use already allocated (default) pins
    #define HW_UART_PIN_TX  UART_PIN_NO_CHANGE
    #define HW_UART_PIN_RX  UART_PIN_NO_CHANGE
    #define HW_UART_PIN_CTS UART_PIN_NO_CHANGE
    #define HW_UART_PIN_RTS UART_PIN_NO_CHANGE

    #define HW_UART_BAUD_RATE 4000000
    #define HW_UART_FLOW_CONTROL UART_HW_FLOWCTRL_DISABLE

    #define HW_UART_SERBUF_SIZE 4096

#elif defined CONFIG_ESP_CONSOLE_USB_CDC
    #define USE_USB_CDC

#else
    #error no serial option selected

#endif

static const char TAG[] = "serprog";

#define BUS_SPI         (1 << 3)
#define S_SUPPORTED_BUS   BUS_SPI
#define S_CMD_MAP ( \
  (1 << S_CMD_NOP)       | \
  (1 << S_CMD_Q_IFACE)   | \
  (1 << S_CMD_Q_CMDMAP)  | \
  (1 << S_CMD_Q_PGMNAME) | \
  (1 << S_CMD_Q_SERBUF)  | \
  (1 << S_CMD_Q_BUSTYPE) | \
  (1 << S_CMD_SYNCNOP)   | \
  (1 << S_CMD_O_SPIOP)   | \
  (1 << S_CMD_S_BUSTYPE) | \
  (1 << S_CMD_S_SPI_FREQ) | \
  (1 << S_CMD_Q_WRNMAXLEN) | \
  (1 << S_CMD_Q_RDNMAXLEN) \
)

#ifdef USE_WIFI
    #define serprog_getchar()                         wifi_getchar()
    #define serprog_putchar(_character)               wifi_putchar(_character)
    #define serprog_read(_dst, _elementSize, _count)  wifi_read(_dst, _elementSize, _count)
    #define serprog_write(_src, _elementSize, _count) wifi_write(_src, _elementSize, _count)
    #define serprog_flush()                           wifi_flush()
    
#else
    #define serprog_getchar()                         getchar()
    #define serprog_putchar(_character)               putchar(_character)
    #define serprog_read(_dst, _elementSize, _count)  fread(_dst, _elementSize, _count, stdin)
    #define serprog_write(_src, _elementSize, _count) fwrite(_src, _elementSize, _count, stdout)
    #define serprog_flush()                           fflush(stdout)

#endif

static inline uint32_t serprog_getu24(void) 
{
    uint32_t c1 = serprog_getchar();
    uint32_t c2 = serprog_getchar();
    uint32_t c3 = serprog_getchar();
    return c1 | (c2<<8) | (c3<<16);
}

static inline uint32_t serprog_getu32(void) 
{
    uint32_t c1 = serprog_getchar();
    uint32_t c2 = serprog_getchar();
    uint32_t c3 = serprog_getchar();
    uint32_t c4 = serprog_getchar();
    return c1 | (c2<<8) | (c3<<16) | (c4<<24);
}

static inline void serprog_putu32(uint32_t d) 
{
    char buf[4];
    memcpy(buf, &d, 4);
    serprog_putchar(buf[0]);
    serprog_putchar(buf[1]);
    serprog_putchar(buf[2]);
    serprog_putchar(buf[3]);
}

#define SPI_BUF_SIZE 4094

DMA_ATTR unsigned char write_buffer[SPI_BUF_SIZE];
DMA_ATTR unsigned char read_buffer[SPI_BUF_SIZE];

uint8_t syncnop[] = { S_NAK, S_ACK };

spi_device_handle_t spi_device;

static uint32_t serprog_spi_init(uint32_t freq) 
{
    esp_err_t ret;

    if (spi_device != NULL)
    {
        spi_bus_remove_device(spi_device);
    }
    spi_bus_free(EEPROM_HOST);

    //initialize SPI host
    spi_bus_config_t buscfg = 
    {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUF_SIZE,
    };
    
    ret = spi_bus_initialize(EEPROM_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }

    spi_device_interface_config_t devcfg = 
    {
        .clock_speed_hz = freq,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 2,
        .flags = 0,
        .input_delay_ns = 0
    };

    //Attach the EEPROM to the SPI bus
    ret = spi_bus_add_device(EEPROM_HOST, &devcfg, &spi_device);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }

    int freq_khz;

    ret = spi_device_get_actual_freq(spi_device, &freq_khz);
    if (ret != ESP_OK)
    {
        goto cleanup;
    }

    ESP_ERROR_CHECK(ret);

    return freq_khz * 1000;

    //release spi if error
cleanup:
    if (spi_device != NULL)
    {
        spi_bus_remove_device(spi_device);
        spi_device = NULL;
    }
    spi_bus_free(EEPROM_HOST);
    return 0;
}

static void process(int command) 
{
    switch(command) 
    {
        case S_CMD_NOP:
            serprog_putchar(S_ACK);
            break;
        case S_CMD_Q_IFACE:
            serprog_putchar(S_ACK);
            serprog_putchar(0x01);
            serprog_putchar(0x00);
            break;
        case S_CMD_Q_CMDMAP:
            serprog_putchar(S_ACK);
            serprog_putu32(S_CMD_MAP);

            for(int i = 0; i < 32 - sizeof(uint32_t); ++i) 
            {
                serprog_putchar(0);
            }
            break;
        case S_CMD_Q_PGMNAME:
            serprog_putchar(S_ACK);
            serprog_write("esp32-serprog\x0\x0\x0", 1, 16);
            break;
        case S_CMD_Q_SERBUF:
            serprog_putchar(S_ACK);

#if defined HW_UART_SERBUF_SIZE && (HW_UART_FLOW_CONTROL == UART_HW_FLOWCTRL_DISABLE)
            serprog_putchar(HW_UART_SERBUF_SIZE & 0xFF);
            serprog_putchar((HW_UART_SERBUF_SIZE >> 8) & 0xFF);
#else
            //serprog protocol: 
            // If the programmer has a guaranteed working flow control,
		    // it should return a big bogus value - eg 0xFFFF
            serprog_putchar(0xFF);
            serprog_putchar(0xFF);
#endif

            break;
        case S_CMD_Q_RDNMAXLEN:
        case S_CMD_Q_WRNMAXLEN:
            //because of missing halfduplex support single transaction RX+TX is limited to 4094 (SPI_BUF_SIZE)
            serprog_putchar(S_ACK);
            serprog_putchar((SPI_BUF_SIZE / 2) & 0xFF);
            serprog_putchar(((SPI_BUF_SIZE / 2) >> 8) & 0xFF);
            serprog_putchar(((SPI_BUF_SIZE / 2) >> 16) & 0xFF);
            break;
        case S_CMD_Q_BUSTYPE:
            serprog_putchar(S_ACK);
            serprog_putchar(S_SUPPORTED_BUS);
            break;
        case S_CMD_SYNCNOP:
            serprog_write(syncnop, 1, 2);
            break;
        case S_CMD_S_BUSTYPE:
            {
                int bustype = serprog_getchar();
                if((bustype | S_SUPPORTED_BUS) == S_SUPPORTED_BUS) 
                {
                    serprog_putchar(S_ACK);
                } else 
                {
                    serprog_putchar(S_NAK);
                }
            }
            break;
        case S_CMD_O_SPIOP:
            {
                uint32_t wlen = serprog_getu24();
                uint32_t rlen = serprog_getu24();

                //at least esp32s3's HW SPI does not support:
                // 1) halfduplex with both TX and RX phases present
                // 2) DMA buffer length of more than 4094 (?) bytes
                // => i had to use fullduplex & combined TX&RX transmission is limited to 4094 bytes

                if ((wlen + rlen) > SPI_BUF_SIZE)
                {
                    serprog_putchar(S_NAK);
                    break;
                }

                if (serprog_read(write_buffer, 1, wlen) < wlen)
                {
                    break;
                }
                
                memset(write_buffer + wlen, 0, SPI_BUF_SIZE - wlen);

                esp_err_t err;

                err = spi_device_acquire_bus(spi_device, portMAX_DELAY);
                if (err != ESP_OK) 
                {
                    serprog_putchar(S_NAK);
                    break;
                }

                spi_transaction_t trans = 
                {
                    .flags = 0,
                    .length = (wlen + rlen) * 8,
                    .tx_buffer = write_buffer,
                    .rxlength = (wlen + rlen) * 8,
                    .rx_buffer = read_buffer
                };

                err = spi_device_polling_transmit(spi_device, &trans);
                if (err != ESP_OK)
                {
                    spi_device_release_bus(spi_device);
                    serprog_putchar(S_NAK);
                    break;
                }

                spi_device_release_bus(spi_device);

                serprog_putchar(S_ACK);

                if (rlen > 0) 
                {
                    serprog_write(read_buffer + wlen, 1, rlen);
                }
            }
            break;
        case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = serprog_getu32();
                if (freq >= 1) 
                {
                    serprog_putchar(S_ACK);
                    serprog_putu32(serprog_spi_init(freq));
                } 
                else 
                {
                    serprog_putchar(S_NAK);
                }
            }
            break;
        default:
            serprog_putchar(S_NAK);
    }

    serprog_flush();
}

void app_main(void)
{
    esp_err_t ret;

#ifdef USE_WIFI
    ESP_LOGI(TAG, "initializing serprog-wifi");
    ret = wifi_init(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME, WIFI_TCP_PORT);
    ESP_ERROR_CHECK(ret);

#else
    ESP_LOGI(TAG, "disabling built-in logging for serial");
    esp_log_level_set("*", ESP_LOG_NONE);
    fflush(stdout);

    //configure UART for blocking mode

    //disable input buffering
    setvbuf(stdin, NULL, _IONBF, 0);

    #ifdef USE_SERIAL_JTAG
    //configure builtin serial/jtag
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = (usb_serial_jtag_driver_config_t){4096, 4096};

    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    ESP_ERROR_CHECK(ret);

    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    
    esp_vfs_usb_serial_jtag_use_driver();

    #elif defined USE_HW_UART
    //configure HW uart
    uart_config_t uart_config = 
    {
        .baud_rate = HW_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = HW_UART_FLOW_CONTROL
    };

    ret = uart_param_config(HW_UART_NUM, &uart_config);
    ESP_ERROR_CHECK(ret);

    ret = uart_set_pin(HW_UART_NUM, HW_UART_PIN_TX, HW_UART_PIN_RX, HW_UART_PIN_RTS, HW_UART_PIN_CTS);
    ESP_ERROR_CHECK(ret);

    ret = uart_driver_install(HW_UART_NUM, HW_UART_SERBUF_SIZE, 0, 0, NULL, ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(ret);

    esp_vfs_dev_uart_port_set_rx_line_endings(HW_UART_NUM, ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_uart_port_set_tx_line_endings(HW_UART_NUM, ESP_LINE_ENDINGS_LF);

    esp_vfs_dev_uart_use_driver(HW_UART_NUM);

    #elif USE_USB_CDC
        #error not implemented

    #endif
#endif

    serprog_spi_init(8*1000*1000); //use 8M spispeed as initial value

    for (;;) 
    {
        int command = serprog_getchar();

        process(command);

        //just to be safe from RTOS watchdog
        vTaskDelay(1);
    }
}
