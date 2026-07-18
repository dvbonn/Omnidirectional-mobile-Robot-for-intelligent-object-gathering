#include "canbus.h"

static twai_handle_t canbus_handle = NULL;

esp_err_t canbus_send(const uint8_t *data, const uint8_t data_length)
{
    if (data_length > 8)
    {
        ESP_LOGE(TAG, "Data length exceeds 8 bytes, cannot send");
        return ESP_FAIL;
    }
    twai_message_t message = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = CANBUS_FRAME_ID,
        .data_length_code = data_length,
    };

    for (int i = 0; i < data_length; i++)
    {
        message.data[i] = data[i];
    }

    esp_err_t ret = twai_transmit_v2(canbus_handle, &message, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to transmit CAN message");
    }
    else
    {
        ESP_LOGI(TAG, "CAN message transmitted successfully");
    }
    return ret;
}

esp_err_t canbus_receive(uint8_t *data, uint8_t *data_length)
{
    twai_message_t message;
    esp_err_t ret = twai_receive_v2(canbus_handle, &message, pdMS_TO_TICKS(100));
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Received CAN message with ID: 0x%X", message.identifier);
        *data_length = message.data_length_code;
        memcpy(data, message.data, message.data_length_code);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to receive CAN message");
    }
    return ret;
}

static void dummy_send_task(void *arg)
{
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    twai_reconfigure_alerts_v2(canbus_handle, TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED, NULL);
    while (1)
    {
        if (canbus_send(data, sizeof(data)) != ESP_OK)
        {
            uint32_t current_alerts;
            twai_read_alerts_v2(canbus_handle, &current_alerts, pdMS_TO_TICKS(100));
            if (current_alerts & TWAI_ALERT_BUS_OFF)
            {
                ESP_LOGE(TAG, "CAN bus is off. Attempting to recover...");
                twai_initiate_recovery_v2(canbus_handle);
                do
                {
                    twai_read_alerts_v2(canbus_handle, &current_alerts, portMAX_DELAY);

                } while (!(current_alerts & TWAI_ALERT_BUS_RECOVERED));
                ESP_LOGI(TAG, "CAN bus recovered successfully.");
                twai_start_v2(canbus_handle);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void dummy_receive_task(void *arg)
{
    uint8_t buffer[8];
    uint8_t data_length = 0;
    while (1)
    {
        if (canbus_receive(buffer, &data_length) == ESP_OK)
        {
            ESP_LOGI(TAG, "Received data: ");
            for (int i = 0; i < data_length; i++)
            {
                ESP_LOGI(TAG, "0x%02X ", buffer[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t canbus_init(void)
{
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CANBUS_TX_GPIO, CANBUS_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    esp_err_t ret;
    ret = twai_driver_install_v2(&g_config, &t_config, &f_config, &canbus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_direction(CANBUS_TX_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CANBUS_TX_GPIO, 1);
    gpio_set_direction(CANBUS_RX_GPIO, GPIO_MODE_INPUT);

    ret = twai_start_v2(canbus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    xTaskCreate(dummy_send_task, "dummy_send_task", 4096, NULL, 5, NULL);
    xTaskCreate(dummy_receive_task, "dummy_receive_task", 4096, NULL, 5, NULL);

    return ESP_OK;
}