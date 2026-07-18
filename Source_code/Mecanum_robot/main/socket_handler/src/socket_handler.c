#include "socket_handler.h"

#define OTA_BUFFER_SIZE 64 * 1024

#define SEND_DATA(sock, frame)                 \
    {                                          \
        if (send_frame(sock, frame) != ESP_OK) \
            vTaskDelete(NULL);                 \
    }

#define DISPOSE_SOCKET(sock) \
    {                        \
        close(sock);         \
        vTaskDelete(NULL);   \
    }

static state_t robot_state = {0};
static bool is_ota = false;

static SemaphoreHandle_t sock_mutex = NULL;

static esp_err_t send_frame(int *sock, const frame_t *frame)
{
    if (frame->length + 1 > 512)
    {
        ESP_LOGE(TAG, "Frame too long to send");
        return ESP_ERR_NO_MEM;
    }

    int err = send(*sock, frame, frame->length + 1, 0);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        close(*sock);
        return ESP_FAIL;
    }
    // ESP_LOGI(TAG, "Sent %d bytes. Frame %u, length: %u", err, frame->payload, frame->length);
    return ESP_OK;
}

static bool ota_update(const size_t firmware_size, const char *expected_sha256, int *sock)
{
    frame_t frame = {
        .length = sizeof(ota_msg_t),
        .payload.cmd = OTA_UPDATE,
    };
    mbedtls_sha256_context sha256_ctx;
    mbedtls_sha256_init(&sha256_ctx);
    if (mbedtls_sha256_starts(&sha256_ctx, 0) < 0)
    {
        ESP_LOGE(TAG, "SHA256 start failed");
        mbedtls_sha256_free(&sha256_ctx);
        frame.payload.ota_msg.state = false;
        SEND_DATA(sock, &frame);
        fsync(*sock);
        return false;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota_handle;
    if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA begin failed");
        mbedtls_sha256_free(&sha256_ctx);
        frame.payload.ota_msg.state = false;
        SEND_DATA(sock, &frame);
        fsync(*sock);
        return false;
    }

    size_t received_bytes = 0;
    uint8_t *ota_buffer = malloc(OTA_BUFFER_SIZE);
    if (ota_buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate OTA buffer");
        esp_ota_abort(ota_handle);
        mbedtls_sha256_free(&sha256_ctx);
        frame.payload.ota_msg.state = false;
        SEND_DATA(sock, &frame);
        fsync(*sock);
        return false;
    }

    frame.payload.ota_msg.state = true;
    SEND_DATA(sock, &frame);
    fsync(*sock);

    struct timeval timeout = {
        .tv_sec = 10,
        .tv_usec = 0};

    setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    while (received_bytes != firmware_size)
    {
        int len = recv(*sock, ota_buffer, OTA_BUFFER_SIZE, 0);
        if (len <= 0)
        {
            ESP_LOGE(TAG, "Receive error");
            esp_ota_abort(ota_handle);
            free(ota_buffer);
            return false;
        }
        mbedtls_sha256_update(&sha256_ctx, ota_buffer, len);
        if (esp_ota_write(ota_handle, ota_buffer, len) != ESP_OK)
        {
            esp_ota_abort(ota_handle);
            free(ota_buffer);
            return false; // file corrupted
        }
        received_bytes += len;
    }
    uint8_t received_sha256[32];
    mbedtls_sha256_finish(&sha256_ctx, received_sha256);
    mbedtls_sha256_free(&sha256_ctx);

    char calculated_hex[65];
    for (uint8_t i = 0; i < 32; i++)
    {
        sprintf(calculated_hex + (i * 2), "%02x", received_sha256[i]);
    }
    calculated_hex[64] = '\0';

    if (strcmp(expected_sha256, calculated_hex) != 0)
    {
        ESP_LOGE(TAG, "Firmware corrupted");
        frame.payload.ota_msg.state = false;
        SEND_DATA(sock, &frame);
        fsync(*sock);
        esp_ota_abort(ota_handle);
        free(ota_buffer);
        return false;
    }

    esp_ota_end(ota_handle);
    esp_ota_set_boot_partition(update_partition);
    frame.payload.ota_msg.state = true;
    SEND_DATA(sock, &frame);
    fsync(*sock);

    shutdown(*sock, SHUT_RDWR);

    char buff[1];
    if (recv(*sock, buff, 1, 0) < 0)
    {
        ESP_LOGI(TAG, "Connection closed, restarting...");
    }

    close(*sock);
    free(ota_buffer);
    esp_restart();
}

static void receive_task(void *arg)
{
    int sock = (int)(intptr_t)arg;
    int len;
    uint8_t rx_buffer[512];
    while (1)
    {
        if (is_ota)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        switch (len)
        {
        case -1:
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            robot_brake();
            DISPOSE_SOCKET(sock);
            xSemaphoreGive(sock_mutex);
            break;
        case 0:
            ESP_LOGI(TAG, "Connection closed");
            DISPOSE_SOCKET(sock);
            xSemaphoreGive(sock_mutex);
            break;
        default:
            generic_msg_t *msg = (generic_msg_t *)rx_buffer;
            switch (msg->cmd)
            {
            case WIFI_SET:
                msg->wifi_set_msg.ssid[31] = '\0';
                msg->wifi_set_msg.password[63] = '\0';
                connect_wifi(msg->wifi_set_msg.ssid, msg->wifi_set_msg.password);
                break;
            case OTA_UPDATE:
                msg->ota_update_msg.sha256[64] = '\0';
                is_ota = true;
                ota_update(msg->ota_update_msg.firmware_size, msg->ota_update_msg.sha256, &sock);
                is_ota = false;
                break;
            case SET_MOTOR_SPEED:
                TaskHandle_t navigation_task = xTaskGetHandle("navigation_task");
                vTaskSuspend(navigation_task);
                motor_set_speeds(msg->motor_msg.motor_speeds);
                break;
            case SET_ROBOT_VELOCITY:
                TaskHandle_t navigation_task_2 = xTaskGetHandle("navigation_task");
                vTaskResume(navigation_task_2);
                motor_set_velocity(&msg->velocity_msg.velocity);
                break;
            case STOP_ROBOT:
                robot_brake();
                break;
            case AUTO_TUNE:
                motor_auto_tune();
                break;
            case MOTOR_SPECS:
                frame_t frame = {
                    .length = sizeof(motor_specs_msg_t),
                    .payload.cmd = MOTOR_SPECS,
                };
                motor_get_specs(frame.payload.motor_specs_msg.specs);
                SEND_DATA(&sock, &frame);
                break;
            case BNO055_RECALIBRATION:
                bno055_recalibration();
                break;
            default:
                ESP_LOGW(TAG, "Unknown command received: %u", rx_buffer[0]);
                break;
            }
            ESP_LOGI(TAG, "Received %d bytes: %u", len, rx_buffer);
            break;
        }
    }
}

static void send_task(void *arg)
{
    int sock = (int)(intptr_t)arg;
    frame_t frame;
    bno055_msg_t bno055_data = {0};
    sample_motor_speeds_t sample = {0};
    pmw3901_msg_t pmw3901_data = {0};
    while (1)
    {
        if (is_ota)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        get_robot_state(&robot_state);
        get_sample_motor_speeds(&sample);
        get_imu_data(&bno055_data);
        get_optical_flow_data(&pmw3901_data);

        memset(&frame, 0, sizeof(frame));
        frame.payload.cmd = MOTOR_SPEED;
        memcpy(frame.payload.motor_msg.motor_speeds, sample.motor_speeds, 4 * sizeof(float));
        frame.length = sizeof(motor_msg_t);
        SEND_DATA(&sock, &frame);

        memset(&frame, 0, sizeof(frame));
        frame.payload.cmd = BNO055_DATA;
        frame.payload.bno055_msg.heading = bno055_data.heading;
        frame.payload.bno055_msg.calibration_status = bno055_data.calibration_status;
        frame.length = sizeof(bno055_msg_t);
        SEND_DATA(&sock, &frame);

        memset(&frame, 0, sizeof(frame));
        frame.payload.cmd = ROBOT_STATE;
        frame.payload.robot_state_msg.robot_state = robot_state;
        frame.length = sizeof(robot_state_msg_t);
        SEND_DATA(&sock, &frame);

        memset(&frame, 0, sizeof(frame));
        frame.payload.cmd = PMW3901_DATA;
        frame.payload.pmw3901_msg.vx = pmw3901_data.vx;
        frame.payload.pmw3901_msg.vy = pmw3901_data.vy;
        frame.length = sizeof(pmw3901_msg_t);
        SEND_DATA(&sock, &frame);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// static void socket_handler(void *arg)
// {
//     int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // Create endpoint comunication
//     if(listen_sock < 0)
//     {
//             ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
//             vTaskDelete(NULL);
//     }
//     uint8_t opt = 1;
//     setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(uint8_t)); // Reused local address

//     //Bind (rang buoc) address to socket
//     struct sockaddr_storage dest_addr;
//     struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
//     dest_addr_ip4->sin_family = AF_INET;
//     dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
//     dest_addr_ip4->sin_port = htons(2004);

//     if(bind(listen_sock, (const struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
//     {
//         ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
//         DISPOSE_SOCKET(listen_sock);
//     }

//     //Set socket to listen mode with 5 stack connections
//     if(listen(listen_sock, 5) != 0)
//     {
//         ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
//         DISPOSE_SOCKET(listen_sock);
//     }

//     uint8_t keepAlive = 1;
//     uint8_t keepIdle = 29;
//     uint8_t keepInterval = 1;
//     uint8_t keepCount = 1;

//     ESP_LOGI(TAG, "Socket created, listening on port %d", 2004);

//     while (1)
//     {
//         if(is_ota)
//         {
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//             continue;
//         }
//         ESP_LOGI(TAG, "Waiting for connection...");

//         int sock = accept(listen_sock, NULL, NULL);
//         if (sock < 0)
//         {
//             ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
//             break;
//         }

//         ESP_LOGI(TAG, "New connection accepted");

//         // Set TCP options
//         setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(uint8_t));
//         setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(uint8_t));
//         setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(uint8_t));
//         setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(uint8_t));
//         setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(uint8_t));

//         xTaskCreate(receive_task, "receive_task", 7168, (void *)(intptr_t)sock, 5, NULL);
//         xTaskCreate(send_task, "send_task", 4096, (void *)(intptr_t)sock, 5, NULL);
//     }
// }

#define DEST_IP_ADDR "192.168.137.1"

static void socket_handler(void *arg)
{
    while (1)
    {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
        {
            ESP_LOGI(TAG, "Waiting for WiFi connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        xSemaphoreTake(sock_mutex, portMAX_DELAY);
        if (is_ota)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            xSemaphoreGive(sock_mutex);
            continue;
        }

        ESP_LOGI(TAG, "Creating socket...");
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            xSemaphoreGive(sock_mutex);
            continue;
        }

        struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(2004);
        dest_addr.sin_addr.s_addr = inet_addr(DEST_IP_ADDR);

        ESP_LOGI(TAG, "Connecting to %s:%d...", DEST_IP_ADDR, 2004);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            xSemaphoreGive(sock_mutex);
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected!");

        int opt = 1;
        int keepAlive = 1;
        int keepIdle = 29;
        int keepInterval = 1;
        int keepCount = 1;

        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(int));

        xTaskCreate(receive_task, "receive_task", 7168, (void *)(intptr_t)sock, 5, NULL);
        xTaskCreate(send_task, "send_task", 4096, (void *)(intptr_t)sock, 5, NULL);
    }
}

esp_err_t start_socket_handler(void)
{
    sock_mutex = xSemaphoreCreateMutex();
    xTaskCreate(socket_handler, "socket_handler", 3072, NULL, 5, NULL);
    return ESP_OK;
}