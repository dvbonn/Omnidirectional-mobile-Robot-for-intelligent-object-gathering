#ifndef CANBUS_H_
#define CANBUS_H_
#include "public.h"
#include "driver/twai.h"
#include "freertos/queue.h"
#include "message_type.h"

#define CANBUS_TX_GPIO GPIO_NUM_1
#define CANBUS_RX_GPIO GPIO_NUM_2

#define CANBUS_FRAME_ID 0x100

// typedef enum 
// {   
//     OTA_UPDATE = 1,
//     SET_MOTOR_SPEED,
//     SET_ROBOT_VELOCITY,
//     STOP_ROBOT,
//     AUTO_TUNE,
//     MOTOR_SPECS,
//     MOTOR_SPEED,
//     ROBOT_STATE,
//     BNO055_DATA,
//     BNO055_RECALIBRATION,
//     PMW3901_DATA
// } canbus_data_id_t;

esp_err_t canbus_send(const uint8_t *data, const uint8_t data_length);
esp_err_t canbus_receive(uint8_t *data, uint8_t *data_length);
esp_err_t canbus_init(void);

#endif