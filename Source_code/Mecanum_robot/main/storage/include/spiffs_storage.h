#ifndef SPIFFS_STORAGE_H_
#define SPIFFS_STORAGE_H_

#include "public.h"
#include <fcntl.h>
#include "errno.h"
#include "esp_spiffs.h"

#define MOTOR_SPECS_FILE_PATH "/spiffs/motor_specs.cfg"

esp_err_t spiffs_init(void);
esp_err_t spiffs_write(const char* path, const char* data, size_t len);
esp_err_t spiffs_read(const char* path, char* data, size_t max_len);
esp_err_t spiffs_remove(const char* path);

#endif