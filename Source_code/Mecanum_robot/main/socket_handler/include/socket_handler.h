#ifndef SOCKET_HANDLER_H
#define SOCKET_HANDLER_H

#include "public.h"
#include "fcntl.h"
#include "sys/socket.h"
#include "sys/stat.h"
#include "sys/uio.h"
#include "sys/un.h"
#include <string.h>
#include "wifi.h"
#include "esp_ota_ops.h"
#include "mbedtls/sha256.h"
#include "esp_timer.h"
#include "message_type.h"
#include "ekf.h"

esp_err_t start_socket_handler(void);

#endif