#include "wifi.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

static const char TAG[] = "serprog-wifi";

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

#define SOCKET_READY_BIT            BIT0

#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3
#define SERPROG_TCP_NODELAY         1

static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t s_socket_event_group;

static volatile int open_socket;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Event: station started");
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Event: station connected to AP");
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        ESP_LOGW(TAG, "Event: station disconnected");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Event: got IP: "IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void tcp_server_task(void* pv) 
{
    int listen_sock = *(int*)pv;
    char addr_str[128];
    struct sockaddr_storage source_addr; 
    socklen_t addr_len = sizeof(source_addr);

    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    int tcpNodelay = SERPROG_TCP_NODELAY;

    ESP_LOGI(TAG, "Tcp server task started");

    for (;;)
    {
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) 
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        if (open_socket >= 0)
        {
            shutdown(open_socket, 0);
            close(open_socket);
            open_socket = -1;
            
            ESP_LOGW(TAG, "Previously open socket closed");
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &tcpNodelay, sizeof(int));

        open_socket = sock;
        xEventGroupSetBits(s_socket_event_group, SOCKET_READY_BIT);

        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) 
        {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
        else if (source_addr.ss_family == PF_INET6) 
        {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }

        //signal socket is ready

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
    }

    vTaskDelete(NULL);
}

esp_err_t wifi_init(const char *ssid, const char *password, const char *hostname, uint16_t port) 
{
    ///////// INITIALIZE WI-FI AND TCP/IP STACK ////////
    ESP_LOGI(TAG, "netif_init");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_LOGI(TAG, "netif_create_sta");

    esp_netif_t *esp_netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(esp_netif_sta, hostname);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;

    ESP_LOGI(TAG, "wifi_init");

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_sta_config = 
    {
        .sta = 
        {
            .scan_method = WIFI_FAST_SCAN,
            .threshold.authmode = WIFI_AUTH_WEP,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH
        }
    };

    strncpy((char*)wifi_sta_config.sta.ssid, ssid, sizeof(wifi_sta_config.sta.ssid) - 1);
    strncpy((char*)wifi_sta_config.sta.password, password, sizeof(wifi_sta_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));

    ESP_LOGI(TAG, "wifi_start");

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGW(TAG, "Connected to ap SSID: %s", ssid);
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", ssid);
        return ESP_FAIL;
    }
    else 
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "netif_set_default_netif");

    esp_netif_set_default_netif(esp_netif_sta);

    esp_netif_ip_info_t ip_info;
    const char *actual_hostname;

    ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_sta, &ip_info));
    ESP_ERROR_CHECK(esp_netif_get_hostname(esp_netif_sta, &actual_hostname));

    ///////// INITIALIZE TCP LISTENER ////////

    struct sockaddr_storage dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(port);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) 
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    ESP_ERROR_CHECK(bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)));
    ESP_LOGI(TAG, "Socket bound, port: %d", port);

    ESP_ERROR_CHECK(listen(listen_sock, 1));
    ESP_LOGI(TAG, "Socket listening");

    open_socket = -1;
    s_socket_event_group = xEventGroupCreate();
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4096, &listen_sock, 5, NULL, 1);

    ESP_LOGW(TAG, "WiFi ready, IP: "IPSTR", hostname: %s, port: %d", IP2STR(&ip_info.ip), actual_hostname, port);

    return ESP_OK;
}

static int get_current_socket_or_wait(void) 
{
    int current_socket;

    for (;;) 
    {
        current_socket = open_socket;

        if (current_socket >= 0)
            return current_socket;

        ESP_LOGI(TAG, "get_current_socket: waiting for connection");

        xEventGroupWaitBits(s_socket_event_group,
                            SOCKET_READY_BIT,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);
    }
}

int wifi_getchar(void)
{
    uint8_t ret;

    for (;;)
    {
        if (wifi_read(&ret, 1, 1) > 0)
        {
            ESP_LOGD(TAG, "GETCHAR: %x", ret);
            return ret;
        }
    }
}

int wifi_putchar(int character)
{
    uint8_t buffer = (uint8_t)(character & 0xFF);

    ESP_LOGD(TAG, "PUTCHAR: %x", buffer);
    return wifi_write(&buffer, 1, 1) > 0 ? 0 : -1;
}

int wifi_read(void *buffer, size_t element_size, size_t count)
{
    int current_socket;
    int to_receive = element_size*count;

    for (;;) 
    {
        current_socket = get_current_socket_or_wait();

        int received_total = 0;

        while (to_receive > received_total) 
        {
            ESP_LOGD(TAG, "recv: to_receive: %d, received_total: %d", to_receive, received_total);
            int received = recv(current_socket, buffer + received_total, to_receive - received_total, 0);
            if (received <= 0)
            {
                open_socket = -1;
                close(current_socket);
                ESP_LOGW(TAG, "Error occurred during receiving: errno %d", errno);
                break;
            }
            received_total += received;
        }

        if (received_total > 0)
            return received_total;
    }
}

int wifi_write(void *buffer, size_t element_size, size_t count)
{
    int current_socket = open_socket;

    if (current_socket < 0)
        return 0;

    int to_write = element_size*count;
    int written_total = 0;

    while (to_write > written_total) 
    {
        ESP_LOGD(TAG, "send: to_write: %d, written_total: %d", to_write, written_total);
        int written = send(current_socket, buffer + written_total, to_write - written_total, 0);
        if (written <= 0)
        {
            open_socket = -1;
            close(current_socket);
            ESP_LOGW(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }
        written_total += written;
    }

    return written_total;
}