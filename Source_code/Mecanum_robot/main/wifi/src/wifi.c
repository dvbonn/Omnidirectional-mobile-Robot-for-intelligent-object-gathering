#include "wifi.h"

static void wifi_connected(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI("WIFI", "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    
    wifi_config_t wifi_config;
    esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);

    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "ssid", (char *)wifi_config.sta.ssid));
    ESP_ERROR_CHECK(nvs_set_str(my_handle, "password", (char *)wifi_config.sta.password));
    nvs_close(my_handle);   
}

void connect_wifi(char *SSID, char* PASSWORD)
{
    esp_wifi_disconnect();
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .failure_retry_cnt = 100
        },
    };

    strncpy((char *)wifi_config.sta.ssid, SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, PASSWORD, sizeof(wifi_config.sta.password));

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_connect();
}

esp_err_t start_wifi(void)
{
    char* ssid = NULL;
    char* password = NULL;
    nvs_handle_t my_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
    size_t required_size = 0;
    esp_err_t err = nvs_get_str(my_handle, "ssid", NULL, &required_size);
    if(err == ESP_OK)
    {
        ssid = malloc(required_size);
        nvs_get_str(my_handle, "ssid", ssid, &required_size);
    }
    required_size = 0;
    err = nvs_get_str(my_handle, "password", NULL, &required_size);
    if(err == ESP_OK)
    {
        password = malloc(required_size);
        nvs_get_str(my_handle, "password", password, &required_size);
    }

    nvs_close(my_handle);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_connected, NULL, NULL));
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_APSTA);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "Hello",
            .ssid_len = 5,
            .channel = 11,
            .password = "123456789",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK
        }
    };
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    esp_wifi_start();

    wifi_config = (wifi_config_t){
        .sta = {
            .ssid = "sus_thing",
            .password = "t1i8e8n2004",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .failure_retry_cnt =100
        }
    };

    if(ssid != NULL && password != NULL)
    {
        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
        free(ssid);
        free(password);
    }

    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_connect();
    return ESP_OK;
}