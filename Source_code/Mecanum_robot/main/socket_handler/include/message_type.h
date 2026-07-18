#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <stdint.h>
#include "motor_control.h"
#include "bno055.h"
#include "freertos/queue.h"

typedef enum 
{
    WIFI_SET,
    OTA_UPDATE,
    SET_MOTOR_SPEED,
    SET_ROBOT_VELOCITY,
    STOP_ROBOT,
    AUTO_TUNE,
    MOTOR_SPECS,
    MOTOR_SPEED,
    ROBOT_STATE,
    BNO055_DATA,
    BNO055_RECALIBRATION,
    PMW3901_DATA
} cmd_type_t;

#pragma pack(1)
typedef struct {
    uint8_t cmd;
    float motor_speeds[4];
} motor_msg_t;

typedef struct {
    uint8_t cmd;
    state_t robot_state;
} robot_state_msg_t;

typedef struct {
    uint8_t cmd;
    bool state; //true for ok, false for error
} ota_msg_t;

typedef struct {
    uint8_t cmd;
    uint8_t calibration_status;
    float heading;
} bno055_msg_t;

typedef struct {
    uint8_t cmd;
    motor_specs_t specs[4];
} motor_specs_msg_t;

typedef struct {
    uint8_t cmd;
    char ssid[32];
    char password[64];
} wifi_set_msg_t;

typedef struct {
    uint8_t cmd;
    velocity_t velocity;
} velocity_msg_t;

typedef struct {
    uint8_t cmd;
    size_t firmware_size;
    char sha256[65];
} ota_update_msg_t;

typedef struct {
    uint8_t cmd;
    float vx;
    float vy;
} pmw3901_msg_t;

typedef union {
    uint8_t cmd;
    motor_msg_t motor_msg;
    robot_state_msg_t robot_state_msg;
    ota_msg_t ota_msg;
    bno055_msg_t bno055_msg;
    motor_specs_msg_t motor_specs_msg;
    wifi_set_msg_t wifi_set_msg;
    velocity_msg_t velocity_msg;
    ota_update_msg_t ota_update_msg;
    pmw3901_msg_t pmw3901_msg;
} generic_msg_t;

typedef struct {
    uint8_t length;
    generic_msg_t payload;
} frame_t;

#pragma pack()

#endif