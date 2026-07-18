#include "spiffs_storage.h"

esp_err_t spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    return ESP_OK;
}

esp_err_t spiffs_write(const char* path, const char* data, size_t len)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0777);    
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", strerror(errno));
        return ESP_FAIL;
    }
    
    ssize_t written = write(fd, data, len);
    if (written == -1)
    {
        ESP_LOGE(TAG, "Failed to write to file: %s", strerror(errno));
        close(fd);
        return ESP_FAIL;
    }
    close(fd);
    return ESP_OK;
}

esp_err_t spiffs_read(const char* path, char* data, size_t max_len)
{
    int fd = open(path, O_RDONLY);
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", strerror(errno));
        return ESP_FAIL;
    }
    
    ssize_t read_bytes = read(fd, data, max_len - 1);
    if (read_bytes == -1)
    {
        ESP_LOGE(TAG, "Failed to read from file: %s", strerror(errno));
        close(fd);
        return ESP_FAIL;
    }
    data[read_bytes] = '\0'; // Null-terminate the string
    close(fd);
    return ESP_OK;
}

esp_err_t spiffs_remove(const char* path)
{
    if (unlink(path) == -1)
    {
        ESP_LOGE(TAG, "Failed to remove file: %s", strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}