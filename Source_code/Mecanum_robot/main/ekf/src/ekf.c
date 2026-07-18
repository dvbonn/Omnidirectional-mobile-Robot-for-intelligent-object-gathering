#include "ekf.h" 

static SemaphoreHandle_t ekf_mutex;
static portMUX_TYPE speed_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE state_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE imu_mutex = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE optical_flow_mutex = portMUX_INITIALIZER_UNLOCKED;

static state_t robot_state = {0};
static sample_motor_speeds_t sample = {0};
static bno055_msg_t bno055_data = {0};
static pmw3901_msg_t pmw3901_data = {0};

static float P[5][5] = {
    {PA,   0.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, PB,   0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, PC,   0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, PD,   0.0f},
    {0.0f, 0.0f, 0.0f, 0.0f, PE}
};

void get_robot_state(state_t *data)
{
    taskENTER_CRITICAL(&state_mutex);
    memcpy(data, &robot_state, sizeof(state_t));
    taskEXIT_CRITICAL(&state_mutex);
}

void get_sample_motor_speeds(sample_motor_speeds_t *data)
{
    taskENTER_CRITICAL(&speed_mutex);
    memcpy(data, &sample, sizeof(sample_motor_speeds_t));
    taskEXIT_CRITICAL(&speed_mutex);
}

void get_imu_data(bno055_msg_t *data)
{
    taskENTER_CRITICAL(&imu_mutex);
    memcpy(data, &bno055_data, sizeof(bno055_msg_t));
    taskEXIT_CRITICAL(&imu_mutex);
}

void get_optical_flow_data(pmw3901_msg_t *data)
{
    taskENTER_CRITICAL(&optical_flow_mutex);
    memcpy(data, &pmw3901_data, sizeof(pmw3901_msg_t));
    taskEXIT_CRITICAL(&optical_flow_mutex);
}

static void normalize_P(void)
{
    for(int i = 0; i < 5; i++)
    {
        for(int j = i + 1; j < 5; j++)
        {
            P[i][j] = P[j][i];
        }
    }
}

static void ekf_predict(void *arg)
{
    int64_t last_time = esp_timer_get_time(); // for delta_time calculation
    TickType_t xLastWakeTime = xTaskGetTickCount(); // for periodic execution
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    while(true)
    {
        xSemaphoreTake(ekf_mutex, portMAX_DELAY);
        
        taskENTER_CRITICAL(&speed_mutex);
        sample.delta_time = (esp_timer_get_time() - last_time) / 1000000.0f; // Convert to seconds
        motor_get_speeds(sample.motor_speeds);
        taskEXIT_CRITICAL(&speed_mutex);
        
        {
            taskENTER_CRITICAL(&state_mutex);
            state_t current_state = robot_state;
            taskEXIT_CRITICAL(&state_mutex);
            
            forward_kinematic(&current_state, &sample);

            taskENTER_CRITICAL(&state_mutex);
            robot_state = current_state;
            taskEXIT_CRITICAL(&state_mutex);
        }


        last_time = esp_timer_get_time();   
        
#if USE_EKF
        const float s = sinf(robot_state.position.theta);
        const float c = cosf(robot_state.position.theta); 


        float M = (-robot_state.velocity.v_x*s - robot_state.velocity.v_y*c) * sample.delta_time;
        float N = (robot_state.velocity.v_x*c - robot_state.velocity.v_y*s) * sample.delta_time;

        float F00 = P[0][0] + P[0][2]*M + P[0][3]*c*sample.delta_time - P[0][4]*s*sample.delta_time; 
        float F01 = P[0][1] + P[1][2]*M + P[1][3]*c*sample.delta_time - P[1][4]*s*sample.delta_time;
        float F02 = P[0][2] + P[2][2]*M + P[2][3]*c*sample.delta_time - P[2][4]*s*sample.delta_time;
        float F03 = P[0][3] + P[2][3]*M + P[3][3]*c*sample.delta_time - P[3][4]*s*sample.delta_time;
        float F04 = P[0][4] + P[2][4]*M + P[3][4]*c*sample.delta_time - P[4][4]*s*sample.delta_time;

        float F11 = P[1][1] + P[1][2]*N + P[1][3]*s*sample.delta_time + P[1][4]*c*sample.delta_time;
        float F12 = P[1][2] + P[2][2]*N + P[2][3]*s*sample.delta_time + P[2][4]*c*sample.delta_time;
        float F13 = P[1][3] + P[2][3]*N + P[3][3]*s*sample.delta_time + P[3][4]*c*sample.delta_time;
        float F14 = P[1][4] + P[2][4]*N + P[3][4]*s*sample.delta_time + P[4][4]*c*sample.delta_time;

        float M00 = F00 + F02*M + F03*c*sample.delta_time - F04*s*sample.delta_time;
        float M01 = F01 + F02*N + F03*s*sample.delta_time + F04*c*sample.delta_time;
        float M11 = F11 + F12*N + F13*s*sample.delta_time + F14*c*sample.delta_time;


        P[0][1] = M01;
        P[1][0] = M01;

        P[0][0] = M00 + QA;
        P[1][1] = M11 + QB;
        P[2][2] +=  QC;
        P[3][3] +=  QD;
        P[4][4] +=  QE;

        P[0][2] = F02;
        P[0][3] = F03;
        P[0][4] = F04;
        P[1][2] = F12;
        P[1][3] = F13;
        P[1][4] = F14;

        normalize_P();
#endif

        xSemaphoreGive(ekf_mutex);
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void ekf_imu_update(float *yaw)
{
    float Gk[5] = {
        P[0][2] / (P[2][2] + RA),
        P[1][2] / (P[2][2] + RA),
        P[2][2] / (P[2][2] + RA),
        P[2][3] / (P[2][2] + RA),
        P[2][4] / (P[2][2] + RA)
    };

    float delta_yaw = *yaw - robot_state.position.theta;
    delta_yaw = atan2f(sinf(delta_yaw), cosf(delta_yaw)); // Normalize to [-pi, pi]

    float l = delta_yaw / (P[2][2] + RA);

    {
        taskENTER_CRITICAL(&state_mutex);
        state_t current_state = robot_state;
        taskEXIT_CRITICAL(&state_mutex);

        current_state.position.x += P[0][2]*l;
        current_state.position.y += P[1][2]*l;
        current_state.position.theta += P[2][2]*l;
        current_state.velocity.v_x += P[2][3]*l;
        current_state.velocity.v_y += P[2][4]*l;

        taskENTER_CRITICAL(&state_mutex);
        robot_state = current_state;
        taskEXIT_CRITICAL(&state_mutex);
    }

    float P22;

    for(uint8_t i = 0; i < 5; i++)
    {
        if(i == 2)
        {
            P22 =  P[2][2] - P[i][2]*Gk[i];
        }
        else
        {
            P[i][i] -= P[i][2]*Gk[i]; 
        }
    }


    for(uint8_t i = 0; i < 5; i++)
    {
        for(uint8_t j = i + 1; j < 5; j++)
        {
            P[i][j] -= P[j][2]*Gk[i];
        }
    }

    P[2][2] = P22;

    normalize_P();
}

static void ekf_pmw3901_update(float *vx, float *vy)
{

    float g = 1.0f / ((P[3][3] + RB) * (P[4][4] + RC) - P[3][4] * P[3][4]);

    float Gk[5][5] = {0};

    for (uint8_t i = 0; i < 5; i++)
    {
        Gk[i][0] = g * (P[i][3] * (P[4][4] + RC) - P[i][4] * P[3][4]);
        Gk[i][1] = g * (P[i][4] * (P[3][3] + RB) - P[i][3] * P[3][4]);
    }

    float delta_vx = *vx - robot_state.velocity.v_x;
    float delta_vy = *vy - robot_state.velocity.v_y;
    
    {
        taskENTER_CRITICAL(&state_mutex);
        state_t current_state = robot_state;
        taskEXIT_CRITICAL(&state_mutex);

        current_state.position.x += delta_vx * Gk[0][0] + delta_vy * Gk[0][1];
        current_state.position.y += delta_vx * Gk[1][0] + delta_vy * Gk[1][1];
        current_state.position.theta += delta_vx * Gk[2][0] + delta_vy * Gk[2][1];
        current_state.velocity.v_x += delta_vx * Gk[3][0] + delta_vy * Gk[3][1];
        current_state.velocity.v_y += delta_vx * Gk[4][0] + delta_vy * Gk[4][1];
        
        taskENTER_CRITICAL(&state_mutex);
        robot_state = current_state;
        taskEXIT_CRITICAL(&state_mutex);
    }

    float P33, P44;

    for(uint8_t i = 0; i < 5; i++)
    {
        if(i == 3)
        {
            P33 = P[3][3] - P[i][3]*Gk[i][0] - P[i][4]*Gk[i][1];
        }
        else if(i == 4)
        {
            P44 = P[4][4] - P[i][3]*Gk[i][0] - P[i][4]*Gk[i][1];
        }
        else
        {
            P[i][i] -= P[i][3]*Gk[i][0] + P[i][4]*Gk[i][1];
        }
    }


    for(uint8_t i = 0; i < 5; i++)
    {
        for(uint8_t j = i + 1; j < 5; j++)
        {
            P[i][j] -= P[j][3]*Gk[i][0] + P[j][4]*Gk[i][1];
        }
    }

    P[3][3] = P33;
    P[4][4] = P44;

    normalize_P();
}

static void read_bno055(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount(); // for periodic execution
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    bno055_msg_t local_data;
    while(true)
    {
        get_bno055_data(&local_data.heading, 
            &local_data.calibration_status);

#if USE_EKF          
        xSemaphoreTake(ekf_mutex, portMAX_DELAY);
        ekf_imu_update(&local_data.heading);
        xSemaphoreGive(ekf_mutex);
#else
        taskENTER_CRITICAL(&state_mutex);
        robot_state.position.theta = local_data.heading;
        taskEXIT_CRITICAL(&state_mutex);
#endif
            
        taskENTER_CRITICAL(&imu_mutex);
        bno055_data = local_data;
        taskEXIT_CRITICAL(&imu_mutex);
            
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void read_pmw3901(void *arg)
{
    TickType_t xLastWakeTime = xTaskGetTickCount(); // for periodic execution
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    pmw3901_msg_t local_data;
    float heading;
    while(true)
    {
        taskENTER_CRITICAL(&imu_mutex);
        heading = bno055_data.heading;
        taskEXIT_CRITICAL(&imu_mutex);
        
        get_pmw3901_data(&local_data.vx, &local_data.vy, &heading);

#if USE_EKF
        xSemaphoreTake(ekf_mutex, portMAX_DELAY);
        if(local_data.vx != 0 || local_data.vy != 0) ekf_pmw3901_update(&local_data.vx, &local_data.vy);
        xSemaphoreGive(ekf_mutex);
#endif

        taskENTER_CRITICAL(&optical_flow_mutex);
        pmw3901_data = local_data;
        taskEXIT_CRITICAL(&optical_flow_mutex);
        
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

esp_err_t init_ekf(void) 
{
    ekf_mutex = xSemaphoreCreateMutex();
    xTaskCreate(ekf_predict, "ekf_predict", 4096, NULL, 1, NULL);
    xTaskCreate(read_bno055, "read_bno055", 4096, NULL, 2, NULL);
    xTaskCreate(read_pmw3901, "read_pmw3901", 4096, NULL, 2, NULL);
    return ESP_OK;
}