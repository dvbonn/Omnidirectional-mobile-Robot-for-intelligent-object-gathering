#ifndef EFK_H_
#define EFK_H_
#include "public.h"
#include "message_type.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "math.h"
#include "pmw3901.h"

#define QA 0.0001f
#define QB 0.0001f
#define QC 0.0001f
#define QD 0.0001f
#define QE 0.0001f

#define RA 0.01f
#define RB 0.9f
#define RC 0.9f

#define PA 1.0f
#define PB 1.0f
#define PC 1.0f
#define PD 1.0f
#define PE 1.0f

#define USE_EKF 1

void get_robot_state(state_t *data);
void get_sample_motor_speeds(sample_motor_speeds_t *data);
void get_imu_data(bno055_msg_t *data);
void get_optical_flow_data(pmw3901_msg_t *data);
esp_err_t init_ekf(void);

#endif