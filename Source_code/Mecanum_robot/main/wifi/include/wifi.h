#ifndef WIFI_H_
#define WIFI_H_

#include "public.h"
#include "esp_wifi.h"

void connect_wifi(char *SSID, char* PASSWORD);
esp_err_t start_wifi(void);

#endif