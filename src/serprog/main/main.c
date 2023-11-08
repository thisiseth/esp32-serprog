#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "serprog.h"


#ifdef CONFIG_IDF_TARGET_ESP32

#  define EEPROM_HOST  HSPI_HOST
#  define PIN_NUM_MISO 18
#  define PIN_NUM_MOSI 23
#  define PIN_NUM_CLK  19
#  define PIN_NUM_CS   13

#elif defined CONFIG_IDF_TARGET_ESP32S2

#  define EEPROM_HOST  SPI2_HOST
#  define PIN_NUM_MISO 37
#  define PIN_NUM_MOSI 35
#  define PIN_NUM_CLK  36
#  define PIN_NUM_CS   34

#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2

#  define EEPROM_HOST  SPI2_HOST
#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10

#elif CONFIG_IDF_TARGET_ESP32S3

#  define EEPROM_HOST  SPI2_HOST
#  define PIN_NUM_MISO 5
#  define PIN_NUM_MOSI 16
#  define PIN_NUM_CLK  15
#  define PIN_NUM_CS   4

#  define USE_SERIAL_JTAG

#endif

#ifdef USE_SERIAL_JTAG

#  include "driver/usb_serial_jtag.h"
#  include "esp_vfs_usb_serial_jtag.h"

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

static uint32_t getu24() 
{
    uint32_t c1 = getchar();
    uint32_t c2 = getchar();
    uint32_t c3 = getchar();
    return c1 | (c2<<8) | (c3<<16);
}

static uint32_t getu32() 
{
    uint32_t c1 = getchar();
    uint32_t c2 = getchar();
    uint32_t c3 = getchar();
    uint32_t c4 = getchar();
    return c1 | (c2<<8) | (c3<<16) | (c4<<24);
}

static void putu32(uint32_t d) 
{
    char buf[4];
    memcpy(buf, &d, 4);
    putchar(buf[0]);
    putchar(buf[1]);
    putchar(buf[2]);
    putchar(buf[3]);
}

#define SPI_BUF_SIZE 4094

DMA_ATTR unsigned char write_buffer[SPI_BUF_SIZE];
DMA_ATTR unsigned char read_buffer[SPI_BUF_SIZE];

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
            putchar(S_ACK);
            break;
        case S_CMD_Q_IFACE:
            putchar(S_ACK);
            putchar(0x01);
            putchar(0x00);
            break;
        case S_CMD_Q_CMDMAP:
            putchar(S_ACK);
            putu32(S_CMD_MAP);

            for(int i = 0; i < 32 - sizeof(uint32_t); ++i) 
            {
                putchar(0);
            }
            break;
        case S_CMD_Q_PGMNAME:
            putchar(S_ACK);
            fwrite("esp32-serprog\x0\x0\x0", 1, 16, stdout);
            fflush(stdout);
            break;
        case S_CMD_Q_SERBUF:
            putchar(S_ACK);
            putchar(0xFF);
            putchar(0xFF);
            break;
        case S_CMD_Q_RDNMAXLEN:
        case S_CMD_Q_WRNMAXLEN:
            putchar(S_ACK);
            putchar((SPI_BUF_SIZE / 2) & 0xFF);
            putchar(((SPI_BUF_SIZE / 2) >> 8) & 0xFF);
            putchar(((SPI_BUF_SIZE / 2) >> 16) & 0xFF);
            break;
        case S_CMD_Q_BUSTYPE:
            putchar(S_ACK);
            putchar(S_SUPPORTED_BUS);
            break;
        case S_CMD_SYNCNOP:
            putchar(S_NAK);
            putchar(S_ACK);
            break;
        case S_CMD_S_BUSTYPE:
            {
                int bustype = getchar();
                if((bustype | S_SUPPORTED_BUS) == S_SUPPORTED_BUS) 
                {
                    putchar(S_ACK);
                } else 
                {
                    putchar(S_NAK);
                }
            }
            break;
        case S_CMD_O_SPIOP:
            {
                uint32_t wlen = getu24();
                uint32_t rlen = getu24();

                //at least esp32s3's HW SPI does not support:
                // 1) halfduplex with both TX and RX phases present
                // 2) DMA buffer length of more than 4094 (?) bytes
                // => i had to use fullduplex & combined TX&RX transmission is limited to 4094 bytes

                if ((wlen + rlen) > SPI_BUF_SIZE)
                {
                    putchar(S_NAK);
                    break;
                }

                fread(write_buffer, 1, wlen, stdin);
                memset(write_buffer + wlen, 0, SPI_BUF_SIZE - wlen);

                esp_err_t err;

                err = spi_device_acquire_bus(spi_device, portMAX_DELAY);
                if (err != ESP_OK) 
                {
                    putchar(S_NAK);
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
                    putchar(S_NAK);
                    break;
                }

                spi_device_release_bus(spi_device);

                putchar(S_ACK);

                if (rlen > 0) 
                {
                    fwrite(read_buffer + wlen, 1, rlen, stdout);
                }
            }
            break;
        case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = getu32();
                if (freq >= 1) 
                {
                    putchar(S_ACK);
                    putu32(serprog_spi_init(freq));
                } else 
                {
                    putchar(S_NAK);
                }
            }
            break;
        default:
            putchar(S_NAK);
    }

    fflush(stdout);
}

void app_main(void)
{
    ESP_LOGI(TAG, "disabling built-in logging for serial");
    esp_log_level_set("*", ESP_LOG_NONE);
    fflush(stdout);

    //configure UART for blocking mode
    esp_err_t ret;

    //disable input buffering
    setvbuf(stdin, NULL, _IONBF, 0);

#ifdef USE_SERIAL_JTAG
    //configure builtin serial/jtag
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = (usb_serial_jtag_driver_config_t){1024, 1024};

    ret = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    ESP_ERROR_CHECK(ret);

    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    
    esp_vfs_usb_serial_jtag_use_driver();
#else
    //i dont have other esp controllers around but the procedure is similar for your chosen UART if it is not serial/jtag
    // 1) initialize driver
    // 2) 'use driver' i.e. enable blocking stdin/stdout

    //todo: write other stuff
    esp_vfs_dev_uart_port_set_rx_line_endings(uart_num, ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_uart_port_set_tx_line_endings(uart_num, ESP_LINE_ENDINGS_LF);

    esp_vfs_dev_uart_use_driver(uart_num);
#endif

    for (;;) 
    {
        int command = getchar();

        process(command);
        fflush(stdout);
    }
}
