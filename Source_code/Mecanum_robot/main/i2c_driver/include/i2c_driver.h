#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "public.h"
#include "bno055.h"
#include "driver/i2c_master.h"

typedef enum 
{
    BNO055_SDA_GPIO = GPIO_NUM_36,
    BNO055_SCL_GPIO = GPIO_NUM_35,
} bno055_i2c_pins;

esp_err_t i2c_init(void);

#endif