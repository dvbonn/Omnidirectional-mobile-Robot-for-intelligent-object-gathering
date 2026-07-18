#include "public.h"
#include "nvs_storage.h"
#include "spiffs_storage.h"
#include "wifi.h"
#include "mdns_service.h"
#include "socket_handler.h"
#include "motor_control.h"
#include "i2c_driver.h"
#include "pmw3901.h"
#include "canbus.h"

#define ALERT_INIT(func)\
{\
    ESP_LOGI(TAG, "Initializing " #func);\
    if(func() != ESP_OK)\
    ESP_LOGE(TAG, "Failed to initialize " #func);\
    else ESP_LOGI(TAG, "Successfully initialized " #func);\
}\

const char *TAG = "TAG";


void init_components()
{
    ALERT_INIT(nvs_storage_init);
    ALERT_INIT(spiffs_init);
    ALERT_INIT(start_wifi);
    // ALERT_INIT(start_mdns_service);
    ALERT_INIT(start_socket_handler);
    ALERT_INIT(motor_control_init);
    ALERT_INIT(i2c_init);
    ALERT_INIT(init_spi_driver);
    // ALERT_INIT(canbus_init);
    ALERT_INIT(establish_bno055_i2c_device);
    ALERT_INIT(init_pmw3901);
    ALERT_INIT(init_ekf);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");
    init_components();
}