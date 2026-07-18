#include "motor_control.h"

#include "ekf.h" //for ignore nest include error of message_type.h

#define TIMER_RESOLUTION_HZ 10000000 // 10 MHz
#define TIMER_PERIOD_TICKS 400       // 40 ms period, 25 kHz frequency

#define PCNT_HIGH_LIMIT 20000 //16 bit counter ko cho
#define PCNT_LOW_LIMIT -20000

#define GEAR_RATIO 270
#define PULSES_PER_ROUND 16
#define ENCODER_PPR (GEAR_RATIO * PULSES_PER_ROUND * 4)
#define MAX_MOTOR_RPM 22
#define MAX_MOTOR_RadPS ((MAX_MOTOR_RPM * 2 * M_PI) / 60)

static motor_driver_manager_t motor_driver_managers[4];

static uint8_t l_motor_gpio[4] = {GPIO_NUM_15, GPIO_NUM_4, GPIO_NUM_12, GPIO_NUM_8};
static uint8_t r_motor_gpio[4] = {GPIO_NUM_16, GPIO_NUM_5, GPIO_NUM_13, GPIO_NUM_9};
static uint8_t enc_a_gpio[4] = {GPIO_NUM_17, GPIO_NUM_6, GPIO_NUM_14, GPIO_NUM_10};
static uint8_t enc_b_gpio[4] = {GPIO_NUM_18, GPIO_NUM_7, GPIO_NUM_21, GPIO_NUM_11};
static volatile float motor_current_speeds[4] = {0};

static velocity_t target_vel = {0.0f, 0.0f, 0.0f};
PID_t vx_pid = {.Kp = 1.0f, .Ki = 0.05f, .Kd = 0.05f, .integral = 0.0f, .prev_error = 0.0f};
PID_t vy_pid = {.Kp = 1.0f, .Ki = 0.05f, .Kd = 0.05f, .integral = 0.0f, .prev_error = 0.0f};
PID_t heading_pid = {.Kp = 1.0f, .Ki = 0.1f, .Kd = 0.1f, .integral = 0.0f, .prev_error = 0.0f};

static float motor_target_speeds[4] = {0};
static bool is_brake = true;
static bool is_tuning = false;

static mcpwm_timer_handle_t create_mcpwm_timer(bool timer_num)
{
    mcpwm_timer_handle_t temp_timer_handle;
    mcpwm_timer_config_t timer_config = {
        .group_id = timer_num,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMER_RESOLUTION_HZ,
        .period_ticks = TIMER_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP};

    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &temp_timer_handle));
    return temp_timer_handle;
}

static mcpwm_oper_handle_t create_mcpwm_oper(mcpwm_timer_handle_t timer_handle, bool timer_num)
{
    mcpwm_oper_handle_t temp_oper_handle;
    mcpwm_operator_config_t oper_config = {
        .group_id = timer_num};

    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &temp_oper_handle));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(temp_oper_handle, timer_handle));
    return temp_oper_handle;
}

static void create_mcpwm_cmpr(mcpwm_oper_handle_t oper_handle, uint8_t driver_num)
{
    mcpwm_comparator_config_t cmpr_config = {
        .flags.update_cmp_on_tez = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_handle, &cmpr_config, &motor_driver_managers[driver_num].cmpr_l));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_handle, &cmpr_config, &motor_driver_managers[driver_num].cmpr_r));
}

static void create_mcpwm_gen(mcpwm_oper_handle_t oper_handle, uint8_t driver_num)
{
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = l_motor_gpio[driver_num]};

    ESP_ERROR_CHECK(mcpwm_new_generator(oper_handle, &gen_config, &motor_driver_managers[driver_num].gen_l));
    gen_config.gen_gpio_num = r_motor_gpio[driver_num];
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_handle, &gen_config, &motor_driver_managers[driver_num].gen_r));

    mcpwm_comparator_set_compare_value(motor_driver_managers[driver_num].cmpr_l, 0);
    mcpwm_comparator_set_compare_value(motor_driver_managers[driver_num].cmpr_r, 0);

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor_driver_managers[driver_num].gen_l,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor_driver_managers[driver_num].gen_l,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_driver_managers[driver_num].cmpr_l, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor_driver_managers[driver_num].gen_r,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor_driver_managers[driver_num].gen_r,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor_driver_managers[driver_num].cmpr_r, MCPWM_GEN_ACTION_LOW)));
}

static void create_pcnt_channels(pcnt_unit_handle_t pcnt_unit, uint8_t a_gpio, uint8_t b_gpio)
{
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = a_gpio,
        .level_gpio_num = b_gpio,
    };
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = b_gpio,
        .level_gpio_num = a_gpio,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;

    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
}

static void create_driver(uint8_t driver_num)
{
    mcpwm_oper_handle_t oper_handle;
    mcpwm_timer_handle_t timer_handle;
    bool timer_num = driver_num > 2;
    timer_handle = create_mcpwm_timer(timer_num);
    oper_handle = create_mcpwm_oper(timer_handle, timer_num);
    create_mcpwm_cmpr(oper_handle, driver_num);
    create_mcpwm_gen(oper_handle, driver_num);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_handle));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_handle, MCPWM_TIMER_START_NO_STOP));

    // create_pcnt_unit(driver_num);
    pcnt_unit_config_t pcnt_config = {
        .intr_priority = 0,
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_config, &motor_driver_managers[driver_num].pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_driver_managers[driver_num].pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_driver_managers[driver_num].pcnt_unit, PCNT_LOW_LIMIT));

    pcnt_unit_clear_count(motor_driver_managers[driver_num].pcnt_unit);
    
    create_pcnt_channels(motor_driver_managers[driver_num].pcnt_unit, enc_a_gpio[driver_num], enc_b_gpio[driver_num]);

    ESP_ERROR_CHECK(pcnt_unit_enable(motor_driver_managers[driver_num].pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(motor_driver_managers[driver_num].pcnt_unit));
}

static void motor_forward(motor_driver_manager_t *manager)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_l, -1, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_r, 0, true));
}

static void motor_reverse(motor_driver_manager_t *manager)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_l, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_r, -1, true));
}

static void motor_brake(motor_driver_manager_t *manager)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_l, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_r, 0, true));
}

void robot_brake()
{
    is_brake = true;
    for(uint8_t i = 0; i < 4; i++)
    {
        motor_brake(&motor_driver_managers[i]);
    }
}


void motor_stop_emergency(motor_driver_manager_t *manager)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_l, 1, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(manager->gen_r, 1, true));
}

void motor_set_velocity(const velocity_t *ref_vel)
{
    target_vel = *ref_vel;
    for(uint8_t i = 0; i < 4; i++)
    {
        motor_driver_managers[i].integral = 0.0f;
    }
    is_brake = false;
}

static void cal_target_motor_speeds(const velocity_t *ref_vel, const position_t *position)
{
    inverse_kinematic(motor_target_speeds, ref_vel, position);
}


void motor_set_speeds(float *motor_speeds)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_target_speeds[i] = motor_speeds[i] / 60;
    }
    is_brake = false;
}

void motor_get_speeds(float *motor_speeds)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_speeds[i] = is_tuning ? 0 : motor_current_speeds[i];
    }
}

void motor_get_specs(motor_specs_t specs[])
{
    specs[0] = motor_driver_managers[0].specs;
    specs[1] = motor_driver_managers[1].specs;
    specs[2] = motor_driver_managers[2].specs;
    specs[3] = motor_driver_managers[3].specs;
}

static void read_motor_speed_task(void *arg)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t PreLastWakeTime = 0;
    while (1)
    {
        PreLastWakeTime = xLastWakeTime;
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        float delta_time = (float)(xLastWakeTime - PreLastWakeTime) / configTICK_RATE_HZ; // seconds
        for (uint8_t i = 0; i < 4; i++)
        {
            int count;
            ESP_ERROR_CHECK(pcnt_unit_get_count(motor_driver_managers[i].pcnt_unit, &count));
            motor_current_speeds[i] = ((float)count / (ENCODER_PPR * delta_time)) * 2 * M_PI;
            ESP_ERROR_CHECK(pcnt_unit_clear_count(motor_driver_managers[i].pcnt_unit));
        }
    }
}

static void motor_set_cmprs(int *cmpr_values)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        int cmpr_value = cmpr_values[i];
        if (cmpr_value < 0)
        {
            motor_reverse(&motor_driver_managers[i]);
            cmpr_value = -cmpr_value;
        }
        else
        {
            motor_forward(&motor_driver_managers[i]);
        }
        mcpwm_comparator_set_compare_value(motor_driver_managers[i].cmpr_l, (uint32_t)cmpr_value);
        mcpwm_comparator_set_compare_value(motor_driver_managers[i].cmpr_r, (uint32_t)cmpr_value);
    }
}

static void store_specs(void)
{
        char motor_specs[176];
        memset(motor_specs, 0, sizeof(motor_specs));
        for (uint8_t i = 0; i < 4; i++)
        {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%.5f\n", motor_driver_managers[i].specs.J);
            strcat(motor_specs, buffer);
            snprintf(buffer, sizeof(buffer), "%.5f\n", motor_driver_managers[i].specs.B);
            strcat(motor_specs, buffer);
            snprintf(buffer, sizeof(buffer), "%.5f\n", motor_driver_managers[i].specs.K1);
            strcat(motor_specs, buffer);
            snprintf(buffer, sizeof(buffer), "%.5f\n", motor_driver_managers[i].specs.K2);
            strcat(motor_specs, buffer);
        }
    esp_err_t ret = spiffs_write(MOTOR_SPECS_FILE_PATH, motor_specs, strlen(motor_specs));
    if (ret != ESP_OK)
    {
        ESP_LOGE("MOTOR_CONTROL", "Failed to store motor specs to SPIFFS");
    }
}

void motor_auto_tune()
{
    TaskHandle_t control_handle = xTaskGetHandle("control_task");
    vTaskSuspend(control_handle);
    is_tuning = true;
    int cmpr_values[4];
    float zeta = 1.0f;
    float wn = 18.0f;
    
    int test_pwm = 80 * TIMER_PERIOD_TICKS / 100;
    cmpr_values[0] = -test_pwm;   
    cmpr_values[1] = test_pwm;  
    cmpr_values[2] = test_pwm;   
    cmpr_values[3] = -test_pwm;  
    
    motor_set_cmprs(cmpr_values);
    vTaskDelay(pdMS_TO_TICKS(1500));

    for(uint8_t i = 0; i < 4; i++)
    {
        float current_v = fabsf(motor_current_speeds[i]);
        if (current_v > 0.1f)
        {
            motor_driver_managers[i].specs.B = (float)test_pwm / current_v;
        }
        else 
        {
            motor_driver_managers[i].specs.B = 0.1f;
        }
        cmpr_values[i] = -cmpr_values[i];
    }

    TickType_t t1 = xTaskGetTickCount();
    float vel_1[4] = {motor_current_speeds[0], motor_current_speeds[1], motor_current_speeds[2], motor_current_speeds[3]};
    motor_set_cmprs(cmpr_values);
    vTaskDelay(pdMS_TO_TICKS(60));
    TickType_t t2 = xTaskGetTickCount();
    float vel_2[4] = {motor_current_speeds[0], motor_current_speeds[1], motor_current_speeds[2], motor_current_speeds[3]};
    robot_brake();
    float dt = (float)(t2 - t1) / configTICK_RATE_HZ;

    for(uint8_t i = 0; i < 4; i++)
    {
        float dv = fabsf(vel_2[i] - vel_1[i]);
        if(dv > 0.001f)
        {
            motor_driver_managers[i].specs.J = fabsf(((float)cmpr_values[i] - motor_driver_managers[i].specs.B * vel_1[i]) * dt / dv);
        }
        else
        {
            motor_driver_managers[i].specs.J = 0.01f;
        }

        motor_driver_managers[i].specs.K1 = motor_driver_managers[i].specs.J * (wn * wn);
        motor_driver_managers[i].specs.K2 = 2 * motor_driver_managers[i].specs.J * zeta * wn - motor_driver_managers[i].specs.B;

        if(motor_driver_managers[i].specs.K2 < 0.0f)
        {
            motor_driver_managers[i].specs.K2 = 0.0f;
        }
    }
    store_specs();
    is_tuning = false;
    vTaskResume(control_handle);
}

static float cal_pid(PID_t *pid, const float *error, const float *dt)
{
    pid->integral += (*error) * (*dt);
    float derivative = ((*error) - pid->prev_error) / (*dt);
    pid->prev_error = *error;
    return pid->Kp * (*error) + pid->Ki * pid->integral + pid->Kd * derivative;
}

static void navigation_task(void *arg)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();
    float target_heading = 0.0f; // radians
    bool is_rotating = false;
    while(true)           
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if(is_brake)
        {
            continue;
        }
        state_t robot_state;
        get_robot_state(&robot_state);

#ifdef USE_NAV
        float delta_time = (esp_timer_get_time() - last_time) / 1000000.0f; // seconds
        last_time = esp_timer_get_time();
        float v_theta;
        if(target_vel.v_theta != 0.0f)
        {
            is_rotating = true;
            v_theta = target_vel.v_theta;
        }
        else
        {
            if(is_rotating)
            {
                target_heading = robot_state.position.theta;
                is_rotating = false;
            }

            float heading_error = target_heading - robot_state.position.theta;
            heading_error = atan2(sinf(heading_error), cosf(heading_error)); // Normalize to [-pi, pi]

            if (fabs(heading_error) < 0.008f) 
            {
                heading_error = 0.0f;
                heading_pid.integral = 0; 
            }

            v_theta = cal_pid(&heading_pid, &heading_error, &delta_time);
        }

        float vx_error = target_vel.v_x - robot_state.velocity.v_x;
        float vy_error = target_vel.v_y - robot_state.velocity.v_y;

        velocity_t ref_vel = {
            .v_x = cal_pid(&vx_pid, &vx_error, &delta_time),
            .v_y = cal_pid(&vy_pid, &vy_error, &delta_time),
            .v_theta = v_theta
        };
#else
        velocity_t ref_vel = {
            .v_x = target_vel.v_x,
            .v_y = target_vel.v_y,
            .v_theta = target_vel.v_theta
        };
#endif

        cal_target_motor_speeds(&ref_vel, &robot_state.position);
    }
}

static void control_task(void *arg)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (is_brake)
        {
            continue;
        }
        int cmpr_values[4];
        for (uint8_t i = 0; i < 4; i++)
        {
            float error = motor_target_speeds[i] - motor_current_speeds[i];
            motor_driver_managers[i].integral += error * 0.01f;
            float feedforward = motor_driver_managers[i].specs.B * motor_target_speeds[i];
            cmpr_values[i] = (int)(feedforward + motor_driver_managers[i].specs.K2 * error + motor_driver_managers[i].specs.K1 * motor_driver_managers[i].integral);
            if(cmpr_values[i] > TIMER_PERIOD_TICKS)
            {
                cmpr_values[i] = TIMER_PERIOD_TICKS;
            }
            else if(cmpr_values[i] < -TIMER_PERIOD_TICKS)
            {
                cmpr_values[i] = -TIMER_PERIOD_TICKS;
            }
        }
        motor_set_cmprs(cmpr_values);
    }
}

static void load_specs(void)
{
    char motor_specs[176];
    memset(motor_specs, 0, sizeof(motor_specs));
    esp_err_t ret = spiffs_read(MOTOR_SPECS_FILE_PATH, motor_specs, sizeof(motor_specs));
    if (ret == ESP_OK)
    {
        char *token = strtok(motor_specs, "\n");
        for (uint8_t i = 0; i < 4; i++)
        {
            motor_driver_managers[i].specs.J = atof(token);
            token = strtok(NULL, "\n");
            motor_driver_managers[i].specs.B = atof(token);
            token = strtok(NULL, "\n");
            motor_driver_managers[i].specs.K1 = atof(token);
            token = strtok(NULL, "\n");
            motor_driver_managers[i].specs.K2 = atof(token);
            token = strtok(NULL, "\n");
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            motor_driver_managers[i].specs.J = 0.01f;
            motor_driver_managers[i].specs.B = 0.1f;
            motor_driver_managers[i].specs.K1 = 1.0f;
            motor_driver_managers[i].specs.K2 = 0.1f;
        }
    }
}

esp_err_t motor_control_init(void)
{
    for (uint8_t driver_num = 0; driver_num < 4; driver_num++)
    {
        create_driver(driver_num);
        motor_brake(&motor_driver_managers[driver_num]);
        motor_driver_managers[driver_num].integral = 0.0f;
    }
    load_specs();

    xTaskCreate(read_motor_speed_task, "read_speed", 2048, NULL, 1, NULL);
    xTaskCreate(navigation_task, "navigation_task", 4096, NULL, 1, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 1, NULL);
    return ESP_OK;
}