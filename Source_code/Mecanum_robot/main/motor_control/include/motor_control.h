#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include "public.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/pulse_cnt.h"
#include "kinematic.h"
#include "spiffs_storage.h"

typedef struct {
    float J;
    float B;
    float K1;
    float K2;
} motor_specs_t;

typedef struct{
    mcpwm_cmpr_handle_t cmpr_l;
    mcpwm_cmpr_handle_t cmpr_r;
    mcpwm_gen_handle_t gen_l;
    mcpwm_gen_handle_t gen_r;
    pcnt_unit_handle_t pcnt_unit;
    float integral;
    motor_specs_t specs;
}motor_driver_manager_t;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
} PID_t;

typedef enum {
    TIMER_NUM_0,
    TIMER_NUM_1
} timer_num_t;

esp_err_t motor_control_init(void);
void motor_set_velocity(const velocity_t *ref_vel);
void motor_set_speeds(float *motor_speeds);
void motor_get_speeds(float *motor_speeds);
void motor_get_specs(motor_specs_t specs[]);
void robot_brake();
void motor_stop_emergency(motor_driver_manager_t *manager);
void motor_auto_tune();

#endif