#include "mdns_service.h"

esp_err_t start_mdns_service(void)
{
    esp_err_t err = mdns_init();
    if(err)
    {
        printf("MDNS Init failed: %d\n", err);
        return err;
    }

    mdns_txt_item_t test_broadcast[] = {
        {"message", "Hello World!"},
        {"from", "Mecanum Robot"}
    };

    mdns_hostname_set("mecanum_robot");
    mdns_instance_name_set("Mecanum Robot");
    mdns_service_add(NULL, "_robot", "_tcp", 2004, test_broadcast, 2);
    return ESP_OK;
}