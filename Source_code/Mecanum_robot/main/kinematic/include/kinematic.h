#ifndef KINEMATIC_H_
#define KINEMATIC_H_
#include "public.h"
#include "math.h"

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

typedef struct {
    float motor_speeds[4];
    float delta_time;
} sample_motor_speeds_t;

void inverse_kinematic(float *motor_speeds, const velocity_t *ref_vel, const position_t *position);
void forward_kinematic(state_t *current_state, const sample_motor_speeds_t *sample);

#endif