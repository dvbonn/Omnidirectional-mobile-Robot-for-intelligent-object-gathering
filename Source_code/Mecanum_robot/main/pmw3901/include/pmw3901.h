#ifndef PMW3901_H_
#define PMW3901_H_
#include "public.h"
#include "spi_driver.h"
#include "esp_timer.h"
#include <math.h>

#define PMW3901_CLOCK_SPEED 2000000
#define PMW3901_CS_PIN GPIO_NUM_45
#define PMW3901_RST_PIN GPIO_NUM_46

#define PMW3901_REG_PRODUCT_ID 0x00
#define PMW3901_REG_MOTION 0x02
#define PMW3901_REG_DELTA_X_L 0x03
#define PMW3901_REG_DELTA_X_H 0x04
#define PMW3901_REG_DELTA_Y_L 0x05
#define PMW3901_REG_DELTA_Y_H 0x06
#define PMW3901_REG_MOTION_BURST 0x16
#define PMW3901_REG_INVERSE_PRODUCT_ID 0x5F

#define PMW3901_ANGLE_OFFSET (-M_PI / 2)

#define HEIGHT 0.06f // meters, distance from sensor to ground
#define FOV (42 * M_PI / 180) //radians
#define PIXELS 35
#define K (2 * tanf(FOV / 2) / PIXELS)

esp_err_t init_pmw3901(void);
esp_err_t get_pmw3901_data(float *vx, float *vy, const float *heading);

#endif