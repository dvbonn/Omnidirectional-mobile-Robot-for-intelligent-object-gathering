/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {  
  uint32_t I_scale_analog : 1;
  uint32_t internal_Rsense : 1;
  uint32_t en_SpreadCycle : 1;
  uint32_t shaft : 1;
  uint32_t index_otpw : 1;
  uint32_t index_step : 1;
  uint32_t pdn_disable : 1;
  uint32_t mstep_reg_select : 1;
  uint32_t multistep_filt : 1;
  uint32_t test_mode : 1;
  uint32_t reserved : 22;
} TMC2209_GCONF_t;

typedef struct {
  uint32_t IHOLD : 5;
  uint32_t IRUN : 5;
  uint32_t reserved0 : 3;
  uint32_t IHOLDDELAY : 4;
  uint32_t reserved1 : 15;
} TMC2209_IHOLD_IRUN_t;

typedef struct {
  uint32_t semin : 4;
  uint32_t reserved0 : 1;
  uint32_t seup : 2;
  uint32_t reserved1 : 1;
  uint32_t semax : 4;
  uint32_t reserved2 : 1;
  uint32_t sedn : 2;
  uint32_t seimin : 1;
  uint32_t reserved3 : 16;
} TMC2209_COOLCONF_t;

typedef struct {
  uint32_t toff : 4;
  uint32_t hstrt : 3;
  uint32_t hend : 4;
  uint32_t reserved0 : 4;
  uint32_t tbl : 2;
  uint32_t vsense : 1;
  uint32_t reserved1 : 6;
  uint32_t mres : 4;
  uint32_t intpol : 1;
  uint32_t dedge : 1;
  uint32_t diss2g : 1;
  uint32_t diss2vs : 1;
} TMC2209_CHOPCONF_t;

typedef struct 
{
  uint32_t PWM_OFS : 8;
  uint32_t PWM_GRAD : 8;
  uint32_t pwm_freg : 2;
  uint32_t pwm_autoscale : 1;
  uint32_t pwm_autograd : 1;
  uint32_t freewheel : 2;
  uint32_t reserved0 : 2;
  uint32_t PWM_REG : 4;
  uint32_t PWM_LIM : 4;
} TMC2209_PWMCONF_t;

typedef union {
  TMC2209_GCONF_t GCONF;
  TMC2209_IHOLD_IRUN_t IHOLD_IRUN;
  TMC2209_COOLCONF_t COOLCONF;
  TMC2209_CHOPCONF_t CHOPCONF;
  TMC2209_PWMCONF_t PWMCONF;
  uint32_t value;
} TMC2209_data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Global Configuration registers
#define GCONF_ADDR 0x00
#define GSTAT_ADDR 0x01
#define NODECONF_ADDR 0x03

//Velocity Dependent Control registers
#define IHOLD_IRUN_ADDR 0x10
#define TPOWERDOWN_ADDR 0x11
#define TSTEP_ADDR 0x12
#define TPWMTHRS_ADDR 0x13
#define VACTUAL_ADDR 0x22

//StallGuard Control registers
#define TCOOLTHRS_ADDR 0x14
#define SGTHRS_ADDR 0x40
#define SG_RESULT_ADDR 0x41
#define COOLCONF_ADDR 0x42

//Chopper Control registers
#define CHOPCONF_ADDR 0x6C
#define PWMCONF_ADDR 0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t CRC_calculate(const uint8_t *frame, const uint8_t length);
static void Write_Uart(UART_HandleTypeDef *huart, const uint8_t *payload);
static void Read_Request_Uart(UART_HandleTypeDef *huart, const uint8_t *payload);
static void Read_Reply_Uart(UART_HandleTypeDef *huart, uint8_t *payload);
static void TMC2209_get_data(UART_HandleTypeDef *huart, const uint8_t device_address, uint8_t register_address, TMC2209_data_t *data);
static void TMC2209_set_data(UART_HandleTypeDef *huart, const uint8_t device_address, uint8_t register_address, TMC2209_data_t data);

static void TMC2209_Init(UART_HandleTypeDef *huart, const uint8_t device_address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t CRC_calculate(const uint8_t *frame, const uint8_t length)
{
    uint8_t crc = 0;
    uint8_t i, j;
    for (i = 0; i < length; i++) {
        uint8_t currentByte = frame[i];
        for (j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    return crc;
}

static void Write_Uart(UART_HandleTypeDef *huart, const uint8_t *payload)
{
  uint8_t frame[8];
  frame[0] = 0x05; // Sync byte 
  frame[1] = payload[0]; // Device address
  frame[2] = payload[1] | 0x80; // Register address OR command with MSB set to 1 for write operation
  frame[3] = payload[2]; // Data byte 1
  frame[4] = payload[3]; // Data byte 2
  frame[5] = payload[4]; // Data byte 3
  frame[6] = payload[5]; // Data byte 4
  frame[7] = CRC_calculate(frame, 7); // Checksum
  
  HAL_UART_Transmit(huart, frame, 8, HAL_MAX_DELAY);
}

static void Read_Request_Uart(UART_HandleTypeDef *huart, const uint8_t *payload)
{
  uint8_t frame[4];
  frame[0] = 0x05; // Sync byte 
  frame[1] = payload[0]; // Device address
  frame[2] = payload[1];
  frame[3] = CRC_calculate(frame, 3 ); // Checksum
  
  HAL_UART_Transmit(huart, frame, 4, HAL_MAX_DELAY);
}

static void Read_Reply_Uart(UART_HandleTypeDef *huart, uint8_t *payload)
{
  uint8_t frame[8];
  do
  {
    HAL_UART_Receive(huart, frame, 1, HAL_MAX_DELAY);
  } while (frame[0] != 0x05); // Wait for sync byte
  
  HAL_UART_Receive(huart, frame, 8, HAL_MAX_DELAY);
  if(frame[7] == CRC_calculate(frame, 7)) {
    payload[0] = frame[3]; // Data byte 1
    payload[1] = frame[4]; // Data byte 2
    payload[2] = frame[5]; // Data byte 3
    payload[3] = frame[6]; // Data byte 4
  }
}

static void TMC2209_get_data(UART_HandleTypeDef *huart, const uint8_t device_address, uint8_t register_address, TMC2209_data_t *data)
{
  Read_Request_Uart(huart, (uint8_t[]){device_address, register_address});
  HAL_Delay(10);
  Read_Reply_Uart(huart, (uint8_t*)&data->value);
}

static void TMC2209_set_data(UART_HandleTypeDef *huart, const uint8_t device_address, uint8_t register_address, TMC2209_data_t data)
{
  uint8_t payload[6];
  payload[0] = device_address;
  payload[1] = register_address;
  memcpy(&payload[2], &data.value, sizeof(data.value));
  Write_Uart(huart, payload);
}

static void TMC2209_Init(UART_HandleTypeDef *huart, const uint8_t device_address)
{
  TMC2209_data_t data;
  TMC2209_get_data(huart, device_address, GCONF_ADDR, &data);

  data.GCONF.en_SpreadCycle = 1;
  data.GCONF.index_otpw = 1;
  data.GCONF.index_step = 0;
  data.GCONF.pdn_disable = 1;
  data.GCONF.mstep_reg_select = 1;
  data.GCONF.test_mode = 0;

  TMC2209_set_data(huart, device_address, GCONF_ADDR, data);
  TMC2209_set_data(huart, device_address, NODECONF_ADDR, (TMC2209_data_t){.value = (8 << 8)});

  data.value = 0;
  data.IHOLD_IRUN.IHOLD = 8;
  data.IHOLD_IRUN.IRUN = 20;
  data.IHOLD_IRUN.IHOLDDELAY = 8;
  TMC2209_set_data(huart, device_address, IHOLD_IRUN_ADDR, data);

  data.value = 0;
  data.COOLCONF.semin = 5;
  data.COOLCONF.seup = 0b00;
  data.COOLCONF.semax = 5;
  data.COOLCONF.sedn = 0b01;
  data.COOLCONF.seimin = 1;
  TMC2209_set_data(huart, device_address, COOLCONF_ADDR, data);

  data.value = 0;
  data.CHOPCONF.toff = 3;
  data.CHOPCONF.hstrt = 0b000;
  data.CHOPCONF.hend = 0b0000;
  data.CHOPCONF.tbl = 2;
  data.CHOPCONF.vsense = 1;
  data.CHOPCONF.mres = 0b0000;
  data.CHOPCONF.intpol = 1;
  data.CHOPCONF.dedge = 0;
  data.CHOPCONF.diss2g = 0;
  data.CHOPCONF.diss2vs = 0;  
  TMC2209_set_data(huart, device_address, CHOPCONF_ADDR, data);

  data.value = 0;
  data.PWMCONF.PWM_OFS = 36;
  data.PWMCONF.PWM_GRAD = 8;
  data.PWMCONF.pwm_freg = 0b10;
  data.PWMCONF.pwm_autoscale = 1;
  data.PWMCONF.pwm_autograd = 1;
  data.PWMCONF.freewheel = 0b00;
  data.PWMCONF.PWM_REG = 0b0010;
  data.PWMCONF.PWM_LIM = 12;
  TMC2209_set_data(huart, device_address, PWMCONF_ADDR, data);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
