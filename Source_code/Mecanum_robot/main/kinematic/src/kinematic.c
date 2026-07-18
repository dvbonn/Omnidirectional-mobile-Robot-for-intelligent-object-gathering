#include "kinematic.h"
// Robot parameters
#define WHEEL_RADIUS 0.0485f // meters
#define WIDTH 0.0955f      // meters
#define LENGTH 0.13565f       // meters
#define ROBOT_RADIUS sqrt(pow(WIDTH, 2) + pow(LENGTH, 2)) // meters
#define WHEEL_POSITION_ANGLE atan(WIDTH / LENGTH) // radians

// Kinematic parameters
#define INV_WHEEL_RADIUS (1.0f / WHEEL_RADIUS)
#define WHEEL_RADIUS_SCALE (WHEEL_RADIUS / 4.0f)
#define YAW_COEFFICIENT (-sqrtf(2.0f) * ROBOT_RADIUS * sinf(M_PI / 4.0f + WHEEL_POSITION_ANGLE))
#define IK_YAW_FACTOR (INV_WHEEL_RADIUS * YAW_COEFFICIENT)
#define FK_YAW_FACTOR (1.0f / (IK_YAW_FACTOR * 4.0f))

void inverse_kinematic(float *motor_speeds, const velocity_t *ref_vel, const position_t *position)
{
    const float cos_yaw = cosf(position->theta);
    const float sin_yaw = sinf(position->theta);

    const float cos_plus_sin = cos_yaw + sin_yaw;
    const float cos_minus_sin = cos_yaw - sin_yaw;
    const float yaw_component = ref_vel->v_theta * IK_YAW_FACTOR;

    const float vel_w13_component = INV_WHEEL_RADIUS*(ref_vel->v_x * cos_plus_sin - ref_vel->v_y * cos_minus_sin);
    const float vel_w24_component = INV_WHEEL_RADIUS*(ref_vel->v_x * cos_minus_sin + ref_vel->v_y * cos_plus_sin);

    motor_speeds[0] = vel_w13_component + yaw_component;
    motor_speeds[1] = vel_w24_component + yaw_component;
    motor_speeds[2] = -vel_w13_component + yaw_component;
    motor_speeds[3] = -vel_w24_component + yaw_component;
}

void forward_kinematic(state_t *current_state, const sample_motor_speeds_t *sample)
{
    const float cos_yaw = cosf(current_state->position.theta);
    const float sin_yaw = sinf(current_state->position.theta);

    const float cos_plus_sin_component = WHEEL_RADIUS_SCALE*(cos_yaw + sin_yaw);
    const float cos_minus_sin_component = WHEEL_RADIUS_SCALE*(cos_yaw - sin_yaw);

    current_state->velocity.v_x = cos_plus_sin_component * (sample->motor_speeds[0] - sample->motor_speeds[2]) 
                                  + cos_minus_sin_component * (sample->motor_speeds[1] - sample->motor_speeds[3]);

    current_state->velocity.v_y = -cos_minus_sin_component * (sample->motor_speeds[0] - sample->motor_speeds[2]) 
                                  + cos_plus_sin_component * (sample->motor_speeds[1] - sample->motor_speeds[3]);

    current_state->velocity.v_theta = FK_YAW_FACTOR*(sample->motor_speeds[0] + sample->motor_speeds[1] 
                                      + sample->motor_speeds[2] + sample->motor_speeds[3]);

    current_state->position.x += current_state->velocity.v_x * sample->delta_time;
    current_state->position.y += current_state->velocity.v_y * sample->delta_time;
    current_state->position.theta += current_state->velocity.v_theta * sample->delta_time;
    current_state->position.theta = atan2f(sinf(current_state->position.theta), cosf(current_state->position.theta));
}