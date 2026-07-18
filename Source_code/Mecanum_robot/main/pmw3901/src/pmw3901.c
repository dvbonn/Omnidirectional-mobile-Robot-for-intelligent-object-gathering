#include "pmw3901.h"

static spi_device_handle_t pmw3901_spi_handle;
const uint8_t PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[][2] = {
    {0x7F, 0x00},
    {0x55, 0x01},
    {0x50, 0x07},
    {0x7F, 0x0E},
    {0x43, 0x10}, //write 0x10 and read 0x47
    {0x47, 0x08}, //if read not 0x08, try again until 3 times, if still fail, reset the sensor and start over
    {0x67, 0xFF}, //read 0x67 and write to 0x48 = Bit 7 ? 0x04 : 0x02
    {0x48, 0xFF},
    {0x7F, 0x00},
    {0x51, 0x7B},
    {0x50, 0x00},
    {0x55, 0x00},
    {0x7F, 0x0E},
    {0x73, 0x00}, //read and expect 0x00
    {0x70, 0xFF}, //read and store as C1
    {0x71, 0xFF}, //read and store as C2
    {0x7F, 0x00},
    {0x61, 0xAD},
    {0x51, 0x70},
    {0x7F, 0x0E},
    {0x70, 0xFF}, //write C1
    {0x71, 0xFF}, //write C2
    {0x7F, 0x00},
    {0x61, 0xAD},
    {0x7F, 0x03},
    {0x40, 0x00},
    {0x7F, 0x05},
    {0x41, 0xB3},
    {0x43, 0xF1},
    {0x45, 0x14},
    {0x5B, 0x32},
    {0x5F, 0x34},
    {0x7B, 0x08},
    {0x7F, 0x06},
    {0x44, 0x1B},
    {0x40, 0xBF},
    {0x4E, 0x3F},
    {0x7F, 0x08},
    {0x65, 0x20},
    {0x6A, 0x18},
    {0x7F, 0x09},
    {0x4F, 0xAF},
    {0x5F, 0x40},
    {0x48, 0x80},
    {0x49, 0x80},
    {0x57, 0x77},
    {0x60, 0x78},
    {0x61, 0x78},
    {0x62, 0x08},
    {0x63, 0x50},
    {0x7F, 0x0A},
    {0x45, 0x60},
    {0x7F, 0x00},
    {0x4D, 0x11},
    {0x55, 0x80},
    {0x74, 0x1F},
    {0x75, 0x1F},
    {0x4A, 0x78},
    {0x4B, 0x78},
    {0x44, 0x08},
    {0x45, 0x50},
    {0x64, 0xFF},
    {0x65, 0x1F},
    {0x7F, 0x14},
    {0x65, 0x67},
    {0x66, 0x08},
    {0x63, 0x70},
    {0x7F, 0x15},
    {0x48, 0x48},
    {0x7F, 0x07},
    {0x41, 0x0D},
    {0x43, 0x14},
    {0x4B, 0x0E},
    {0x45, 0x0F},
    {0x44, 0x42},
    {0x4C, 0x80},
    {0x7F, 0x10},
    {0x5B, 0x02},
    {0x7F, 0x07},
    {0x40, 0x41},
    {0x70, 0x00},
    //delay 10ms
    {0x32, 0x44},
    {0x7F, 0x07},
    {0x40, 0x40},
    {0x7F, 0x06},
    {0x62, 0xF0},
    {0x63, 0x00},
    {0x7F, 0x0D},
    {0x48, 0xC0},
    {0x6F, 0xD5},
    {0x7F, 0x00},
    {0x5B, 0xA0},
    {0x4E, 0xA8},
    {0x5A, 0x50},
    {0x40, 0x80}
};

static void pmw3901_write_reg(uint8_t reg, uint8_t data)
{
    spi_transaction_t frame = {
        .flags = SPI_TRANS_USE_TXDATA,
        .addr = reg | 0x80,
        .length = 8,
        .tx_data[0] = data
    };
    spi_device_polling_transmit(pmw3901_spi_handle, &frame);
    esp_rom_delay_us(45);
}

static void pmw3901_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    spi_transaction_t frame = {
        .addr = reg,
        .rxlength = 8*len,
        .rx_buffer = data
    };
    spi_device_polling_transmit(pmw3901_spi_handle, &frame);
    esp_rom_delay_us(20);
}

static void pmw3901_reset(void)
{
    gpio_set_level(PMW3901_CS_PIN, 1);
    esp_rom_delay_us(1000);
    gpio_set_level(PMW3901_CS_PIN, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(PMW3901_CS_PIN, 1);
    esp_rom_delay_us(1000);
    
    gpio_set_level(PMW3901_RST_PIN, 1);
    esp_rom_delay_us(1000);
    gpio_set_level(PMW3901_RST_PIN, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(PMW3901_RST_PIN, 1);
    esp_rom_delay_us(5000);
}

esp_err_t init_pmw3901(void)
{
    spi_device_interface_config_t spi_dev_cfg = {
        .clock_speed_hz = PMW3901_CLOCK_SPEED,
        .mode = 3,
        .spics_io_num = PMW3901_CS_PIN,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .address_bits = 8,
        .cs_ena_pretrans = 100,
        .cs_ena_posttrans = 100
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_dev_cfg, &pmw3901_spi_handle));
    
    vTaskDelay(pdMS_TO_TICKS(40));
    
    bool success = false;
    while(!success)
    {
        pmw3901_reset();

        uint8_t chipId;
        uint8_t invChipId;
        pmw3901_read_reg(0x00, &chipId, 1);
        pmw3901_read_reg(0x5F, &invChipId, 1);

        if(chipId != 0x49 || invChipId != 0xB6)
        {
            ESP_LOGE("PMW3901", "Device ID mismatch (0x%02X, 0x%02X)", chipId, invChipId);
            return ESP_FAIL;
        }
        ESP_LOGI("PMW3901", "Device ID verified (0x%02X, 0x%02X)", chipId, invChipId);


        uint8_t buffer;
        pmw3901_read_reg(PMW3901_REG_MOTION, &buffer, 1);
        pmw3901_read_reg(PMW3901_REG_DELTA_X_L, &buffer, 1);
        pmw3901_read_reg(PMW3901_REG_DELTA_X_H, &buffer, 1);
        pmw3901_read_reg(PMW3901_REG_DELTA_Y_L, &buffer, 1);
        pmw3901_read_reg(PMW3901_REG_DELTA_Y_H, &buffer, 1);
        esp_rom_delay_us(1000);

        uint8_t retry_count = 0;
    
        for(uint8_t i = 0; i < 4; i++)
        {
            pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][0]
                , PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][1]);
        }
    
        do
        {
            pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[4][0]
                , PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[4][1]);
            pmw3901_read_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[5][0], &buffer, 1);
            if (buffer == PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[5][1])
            {
                break;
            }    
        } while (++retry_count < 3);
    
        if (retry_count == 3)
        {
            ESP_LOGW(TAG, "0x47 is not as 0x08 (0x%02X), restart the initial sequence", buffer);
            continue;
        }
        else
        {
            pmw3901_read_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[6][0], &buffer, 1);
            pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[7][0]
                , (buffer & 0x80) ? 0x04 : 0x02);

            for(uint8_t i = 8; i < 13; i++)
            {
                pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][0]
                    , PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][1]);
            }

            pmw3901_read_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[13][0], &buffer, 1);
            if(buffer == PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[13][1])
            {
                uint8_t C1, C2;
                pmw3901_read_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[14][0], &C1, 1);
                if(C1 <= 28) C1 += 14; else C1 += 11; 
                if(C1 > 0x3F) C1 = 0x3F;
                pmw3901_read_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[15][0], &C2, 1);
                ESP_LOGI(TAG, "C1 and C2 here we go (0x%02X, 0x%02X)", C1, C2);
                C2 = (C2 * 45) / 100;

                for(uint8_t i = 16; i < 22; i++)
                {
                    pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][0]
                        , (i == 20) ? C1 : (i == 21) ? C2 : PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][1]);
                }
            }

            for(uint8_t i = 22; i < 81; i++)
            {
                pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][0]
                    , PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][1]);
            }

            esp_rom_delay_us(1000);
            for(uint8_t i = 81; i < 95; i++)
            {
                pmw3901_write_reg(PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][0]
                    , PMW3901_PERFORMANCE_OPTIMIZATION_TABLE[i][1]);
            }
            success = true;
        }
        
    }    

    return ESP_OK;
}

esp_err_t get_pmw3901_data(float *vx, float *vy, const float *heading)
{
    *vx = 0.0f;
    *vy = 0.0f;
    static int64_t last_time = -1;
    float delta_time = 0;
    if(last_time != -1)
    {
        delta_time = (esp_timer_get_time() - last_time) / 1000000.0f;
    }
    uint8_t buffer[12] = {0};
    //buffer[1] for observation feature, not actually what we need. So ignore
    pmw3901_read_reg(PMW3901_REG_MOTION_BURST, buffer, 12);

    //i don't know why this shit is here, checking later
    esp_err_t ret = ESP_OK;
    if(ret != ESP_OK) {
        return ret;
    }

    if ((buffer[0] & 0x80) && !((buffer[6] < 0x19) && (buffer[10] == 0x1F))) // check motion and it fake report condition
    {
        int16_t x = (int16_t)(buffer[3] << 8 | buffer[2]);
        int16_t y = (int16_t)(buffer[5] << 8 | buffer[4]);

        float scale_factor = K * HEIGHT / delta_time; // Convert to m/s

        float cos_heading = cosf(*heading + PMW3901_ANGLE_OFFSET);
        float sin_heading = sinf(*heading + PMW3901_ANGLE_OFFSET);
        float temp_vx = x * scale_factor;
        float temp_vy = y * scale_factor;

        *vx = temp_vx * cos_heading - temp_vy * sin_heading;
        *vy = temp_vx * sin_heading + temp_vy * cos_heading;
        if(fabsf(*vx) > 0.1f) *vx = *vx > 0 ? 0.1f : -0.1f; // Robot max speed 0.1m/s, so wtf
        if(fabsf(*vy) > 0.1f) *vy = *vy > 0 ? 0.1f : -0.1f;
    }

    last_time = esp_timer_get_time();
    // ESP_LOGI(TAG, "Condition data [%d 0x%02X 0x%02X]", buffer[0] >> 7, buffer[6], buffer[10]);
    // ESP_LOGI(TAG, "Velocity: (%.2f, %.2f) m/s", *vx, *vy);
    return ESP_OK;
}

