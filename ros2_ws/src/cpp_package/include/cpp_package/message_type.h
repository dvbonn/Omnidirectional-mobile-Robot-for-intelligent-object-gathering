#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

#include <stdint.h>

typedef enum
{
    SET_ROBOT_VELOCITY = 3,
    // STOP_ROBOT = 4,          // added for the watchdog (matches the firmware value)
    ROBOT_STATE = 8,
} cmd_type_t;

typedef struct {
    float v_x;
    float v_y;
    float v_theta;
} velocity_t;

typedef struct {
    float x;
    float y;
    float theta;
} position_t;

typedef struct {
    velocity_t velocity;
    position_t position;
} state_t;

#pragma pack(1)

typedef struct {
    uint8_t cmd;
    state_t robot_state;
} robot_state_msg_t;

typedef struct {
    uint8_t cmd;
    velocity_t velocity;
} velocity_msg_t;

typedef union {
    uint8_t cmd;
    robot_state_msg_t robot_state_msg;
    velocity_msg_t velocity_msg;
} generic_msg_t;

typedef struct {
    uint8_t length;
    generic_msg_t payload;
} frame_t;

#pragma pack()

#endif