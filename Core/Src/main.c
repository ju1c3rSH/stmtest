/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PI 3.14159265358979323846f
#include "oled.h"
#include <stdio.h>
#include <stdbool.h>
#include "mpu9250.h"
#include <math.h>
#include "delay.h"
#include "MahonyAHRS.h"
#include "subtask.h"
#include <stdarg.h >
#include <string.h>
// #include "atgm336h.h"
#include "semphr.h"
#include "oled.h"
#include "Car.h"
#include "uart_pid_parse.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// GPS_PropTypeDef g_gps_data = {0};
extern uint8_t s_pid_uart_rx_buf[PID_UART1_RX_BUF_SIZE];
// extern SemaphoreHandle_t xGnssMutex;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartRecvTask */
osThreadId_t uartRecvTaskHandle;
const osThreadAttr_t uartRecvTask_attributes = {
  .name = "uartRecvTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void UartRecvTask(void *argument);

/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *file)
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

MPU9250 mpu;
uint8_t mpu9250_WhoAmI = 0;
uint8_t ak8963_WhoAmI = 0;
Car_TypeDef *car_instance;
SemaphoreHandle_t xParsePIDMutex = NULL;
QueueHandle_t xUART1ReceiveQueue = NULL;
SemaphoreHandle_t xUART1ProcessingSemaphore = NULL;
static volatile uint32_t ulCurrentDMATransferSize = 0;
static volatile uint32_t ulLastDMATransferSize = 0;
/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == huart1.Instance)
  {
    ulCurrentDMATransferSize = Size;

    static UART1_DMA_Received_Data_t xReceivedData;

    if (ulCurrentDMATransferSize > 0)
    {
      memcpy(xReceivedData.buffer, s_pid_uart_rx_buf, ulCurrentDMATransferSize);
      xReceivedData.length = ulCurrentDMATransferSize;
    }
    if (xQueueSendFromISR(xUART1ReceiveQueue, &xReceivedData, &xHigherPriorityTaskWoken) != pdPASS)
    {
      // Process Queue Full Error>..
    }

    // Now Restart DMA Reception after clear the flag
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx));

    if (HAL_UART_Receive_DMA(&huart1, s_pid_uart_rx_buf, PID_UART1_RX_BUF_SIZE) != HAL_OK)
    {
      // Handle Error
    };
    // 發送信號量以通知數據已準備好
    xSemaphoreGiveFromISR(xUART1ProcessingSemaphore, &xHigherPriorityTaskWoken); // 釋放信號量
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

*/
void StartUART1DMAReceive(void)
{
  // memset(s_uart_rx_buf, 0, UART1_PID_BUFFER_SIZE);

  /*

  if (HAL_UART_Receive_DMA(&huart1, s_pid_uart_rx_buf, PID_UART1_RX_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  };
  只能启动一次 DMA
  */
  HAL_UART_DMAStop(&huart1);
  __HAL_UART_CLEAR_IDLEFLAG(&huart1);

  if (HAL_UART_Receive_DMA(&huart1, s_pid_uart_rx_buf, PID_UART1_RX_BUF_SIZE) != HAL_OK)
  {
    u1_printf("DMA Start Failed!\r\n");
    Error_Handler();
  };
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void Test_SPI_Communication(void)
{
  uint8_t test_data = 0x75; // WHO_AM_I register address
  uint8_t rx_data = 0;

  u1_printf("=== SPI Communication Test ===\r\n");

  // Test 1: Direct HAL_SPI_TransmitReceive test
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET); // CS LOW

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, &test_data, &rx_data, 1, 1000);

  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET); // CS HIGH

  u1_printf("SPI transmit status: %s\r\n", (status == HAL_OK) ? "SUCCESS" : "FAILED");
  u1_printf("Sent: 0x%02X, Received: 0x%02X\r\n", test_data, rx_data);

  // Test 2: Read WHO_AM_I register
  u1_printf("--- Reading WHO_AM_I Register ---\r\n");
  test_data = 0x75 | 0x80; // Read operation, set MSB

  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET); // CS LOW
  status = HAL_SPI_TransmitReceive(&hspi1, &test_data, &rx_data, 1, 1000);
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET); // CS HIGH

  u1_printf("WHO_AM_I read status: %s\r\n", (status == HAL_OK) ? "SUCCESS" : "FAILED");
  u1_printf("WHO_AM_I value: 0x%02X\r\n", rx_data);

  u1_printf("=== SPI Test Complete ===\r\n\r\n");
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  // 使能TB6612
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
  u1_printf("CS HIGH\r\n");

  // HAL_Delay(100);
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
  u1_printf("CS LOW\r\n");
  // HAL_Delay(100);
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
  u1_printf("CS TEST DONE\r\n");
  LoadPIDParamsFromFlash();
  Car_Init(&mpu);
  car_instance = Car_GetInstance();
  /*  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
    */
  // initPIDMutex();
  xParsePIDMutex = xSemaphoreCreateMutex();

  Set_PID(&g_stored_pid_params[PID_TYPE_BALANCE_PITCH], // pitch
          g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kp,
          g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Ki,
          g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kd);
  Set_PID(&g_stored_pid_params[PID_TYPE_BALANCE_YAW], // yaw
          g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kp,
          g_stored_pid_params[PID_TYPE_BALANCE_YAW].Ki,
          g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kd);
  Set_PID(&g_stored_pid_params[PID_TYPE_SPEED], // speed
          g_stored_pid_params[PID_TYPE_SPEED].Kp,
          g_stored_pid_params[PID_TYPE_SPEED].Ki,
          g_stored_pid_params[PID_TYPE_SPEED].Kd);

  u1_printf("{\"data\":{"
            "\"pitch_kp\":%.3f,\"pitch_ki\":%.3f,\"pitch_kd\":%.3f,"
            "\"yaw_kp\":%.3f,\"yaw_ki\":%.3f,\"yaw_kd\":%.3f,"
            "\"speed_kp\":%.3f,\"speed_ki\":%.3f,\"speed_kd\":%.3f"
            "}}\n",
            g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kp,
            g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Ki,
            g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kd,
            g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kp,
            g_stored_pid_params[PID_TYPE_BALANCE_YAW].Ki,
            g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kd,
            g_stored_pid_params[PID_TYPE_SPEED].Kp,
            g_stored_pid_params[PID_TYPE_SPEED].Ki,
            g_stored_pid_params[PID_TYPE_SPEED].Kd);
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // AIN1 = 1
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // AIN2 = 0
  // uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);   // 获取当前周期值 (ARR)
  // uint32_t pulse_value = (period + 1) / 2;              //(CCR = ARR/2)
  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_value);
  StartUART1DMAReceive();
  // DWT_Delay_us(20000);

  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

  if (MPU9250_Init(&mpu, &hspi1, MPU9250_CS_GPIO, MPU9250_CS_PIN))
  {
    u1_printf("MPU9250 init SUCCESSFUL\r\n");
  }
  else
  {
    u1_printf("MPU9250 init FAILED\r\n");
  }

  Mahony_Init(50.0f);
  // Test_SPI_Communication();

  // ATGM336H_Init(&g_gps_data);
  /*
     OLED_PropTypeDef oled_cfg = {
          .scl_gpio_port = OLED_SCL_GPIO_Port,
          .scl_gpio_pin  = OLED_SCL_Pin,
          .sda_gpio_port = OLED_SDA_GPIO_Port,
          .sda_gpio_pin  = OLED_SDA_Pin
      };

      // 初始化 OLED（内部会创建 mutex + 发送初始化命令）
      OLED_InitWithConfig(&oled_cfg);
      OLED_Init();        // 发送 SSD1306 初始化序列

      OLED_Clear();
      OLED_ShowString(0, 0, "OLED OK!", 8);
      OLED_Update();
    */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */

  /* add mutexes, ... */
	
	
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  xUART1ProcessingSemaphore = xSemaphoreCreateBinary();
  if (xUART1ProcessingSemaphore == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  xUART1ReceiveQueue = xQueueCreate(5, sizeof(UART1_DMA_Received_Data_t));
  if (xUART1ReceiveQueue == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uartRecvTask */
  uartRecvTaskHandle = osThreadNew(UartRecvTask, NULL, &uartRecvTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (defaultTaskHandle == NULL)
  {
    u1_printf("ERROR: StartDefaultTask create failed!\r\n");
  }
  if (uartRecvTaskHandle == NULL)
  {
    u1_printf("ERROR: UartRecvTask create failed!\r\n");
  }
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  // HAL_UART_Receive_DMA(&huart1, s_pid_uart_rx_buf, PID_UART1_RX_BUF_SIZE);
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  u1_printf("Queue is %s\n", (xUART1ReceiveQueue ? "OK" : "NULL"));
  // #define SAMPLE_RATE 0.05f
  static uint32_t last_time = 0;
  float yaw = 0.00f;

  for (;;)
  {
    //u1_printf("Default Task Running at time: %lu ms\r\n", HAL_GetTick());
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    //vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Get_Data_SubTask();
    Normal_Balance_SubTask(car_instance);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // IN1 = 1
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2 = 0

    // 确保 PWM 非零（如果使用 PWM 控制）
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
     //vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /*
  for (;;)
  {
    mpu9250_WhoAmI = mpu_r_WhoAmI(&mpu);
    MPU9250_ReadAccel(&mpu);
    MPU9250_ReadGyro(&mpu);

    float pitch = atan2(-mpu.mpu_data.Accel[0], sqrt(mpu.mpu_data.Accel[1] * mpu.mpu_data.Accel[1] + mpu.mpu_data.Accel[2] * mpu.mpu_data.Accel[2])) * 180 / PI;
    float roll = atan2(mpu.mpu_data.Accel[1], mpu.mpu_data.Accel[2]) * 180 / PI;

    float dt = SAMPLE_RATE;

    float gyro_yaw = mpu.mpu_data.Gyro[2] -
                     (mpu.mpu_data.Gyro[0] * sinf(roll * PI / 180) +
                      mpu.mpu_data.Gyro[1] * sinf(pitch * PI / 180) * cosf(roll * PI / 180) +
                      mpu.mpu_data.Gyro[2] * cosf(pitch * PI / 180) * cosf(roll * PI / 180)) *
                         tanf(pitch * PI / 180);

    yaw += gyro_yaw * dt;

    if (yaw >= 360.0f)
      yaw -= 360.0f;
    if (yaw < 0.0f)
      yaw += 360.0f;

    u1_printf("{\"sensor\":\"mpu9250\",\"timestamp\":%lu,\"data\":{",
              HAL_GetTick());
    u1_printf("\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},",
              mpu.mpu_data.Accel[0], mpu.mpu_data.Accel[1], mpu.mpu_data.Accel[2]);
    u1_printf("\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},",
              mpu.mpu_data.Gyro[0], mpu.mpu_data.Gyro[1], mpu.mpu_data.Gyro[2]);
    u1_printf("\"attitude\":{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f}",
              pitch, roll, yaw);
    u1_printf("}}\r\n");

 
    // osDelay(1);
    osDelay(10);
  }
    */
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UartRecvTask */
/**
 * @brief Function implementing the uartRecvTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UartRecvTask */
void UartRecvTask(void *argument)
{
  /* USER CODE BEGIN UartRecvTask */

  /*
  未处理粘包/多帧情况
  当前逻辑假设一帧 = 一个 JSON + 换行。
  如果一次 DMA 接收包含多个 JSON（如 "{"a":1}\n{"b":2}\n"），只会处理第一个.
   */
  /* Infinite loop */
  static UART1_DMA_Received_Data_t xReceivedData;
  static char local_buffer[PID_UART1_RX_BUF_SIZE];
  u1_printf("[UART TASK] Started!\r\n");

  for (;;)
  {
    if (xQueueReceive(xUART1ReceiveQueue, &xReceivedData, portMAX_DELAY) == pdPASS)
    {
      /*
      if (Parse_PID_Command(xReceivedData.buffer, xReceivedData.length, &new_kp, &new_ki, &new_kd))
      {
        g_kp = new_kp;
        g_ki = new_ki;
        g_kd = new_kd;
        u1_printf("[UartRecvTask]Updated PID params: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", g_kp, g_ki, g_kd);
      }
      else
      {
        u1_printf("[UartRecvTask]Invalid PID command received.\r\n");
      }
*/
      char *json_end = NULL;

      uint16_t copy_length = (xReceivedData.length < PID_UART1_RX_BUF_SIZE) ? xReceivedData.length : PID_UART1_RX_BUF_SIZE;
      memcpy(local_buffer, xReceivedData.buffer, copy_length);
      local_buffer[copy_length] = '\0'; // 确保字符串以 null 结尾
      for (int i = 0; i < copy_length; i++)
      {
        // u1_printf("%02X ", xReceivedData.buffer[i]);
        if (local_buffer[i] == '\n' || local_buffer[i] == '\r')
        {
          // xReceivedData.buffer[i] = '\0';
          // 不要直接修改DMA缓冲区内容，改为使用本地缓冲区

          json_end = &local_buffer[i];

          break;
        }
        // 注意128堆栈够不够
        // this代码模块未审计
      }
      if (json_end)
      {

        u1_printf("[UartRecvTask]Received JSON");
        printf("[UartRecvTask]Now Processing JSON: %s\r\n", local_buffer);
        PID_UART_PARSE_Params_t pid_params;
        if (UART1_ParsePIDData(local_buffer, copy_length, &pid_params))
        {
          // g_kp = pid_params.kp;
          // g_ki = pid_params.ki;
          //  = pid_params.kd;
          if (SavePIDParamsToFlash(pid_params.pid_type, pid_params.kp, pid_params.ki, pid_params.kd))
          {
            u1_printf("[UartRecvTask]Updated PID params from JSON and saved to Flash: Type=%d, Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                      pid_params.pid_type, pid_params.kp, pid_params.ki, pid_params.kd);

            // while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);
            // while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
            u1_printf("[UartRecvTask]Resetting system...\r\n");
            HAL_Delay(10);
            __disable_irq();

            vTaskDelay(pdMS_TO_TICKS(100));

            NVIC_SystemReset();
          }
          else
          {
            u1_printf("Failed to save PID params to Flash.\r\n");
          }
        }
        else
        {
          u1_printf("Invalid PID JSON received Or Not Updated.\r\n");
        }
      }
      else
      {
        u1_printf("Received JSON (no newline): %s\r\n", xReceivedData.buffer);
      }
    }
    // osDelay(1);
  }
  /* USER CODE END UartRecvTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
