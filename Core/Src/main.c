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
#include <stdarg.h >
#include <string.h>
#include "atgm336h.h"
#include "semphr.h"
#include "oled.h"
#include "uart_pid_parse.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
GPS_PropTypeDef g_gps_data = {0};
extern SemaphoreHandle_t xGnssMutex;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledInitTask */
osThreadId_t oledInitTaskHandle;
const osThreadAttr_t oledInitTask_attributes = {
  .name = "oledInitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartRecvTask */
osThreadId_t uartRecvTaskHandle;
const osThreadAttr_t uartRecvTask_attributes = {
  .name = "uartRecvTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void OledInitTask(void *argument);
void UartRecvTask(void *argument);

/* USER CODE BEGIN PFP */

int fputc(int ch, FILE *file)
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
  uint8_t buffer[UART1_PID_BUFFER_SIZE];
  uint16_t length;
} UART1_DMA_Received_Data_t;

MPU9250 mpu;
uint8_t mpu9250_WhoAmI = 0;
uint8_t ak8963_WhoAmI = 0;

static float g_kp = 1.0f;
static float g_ki = 0.1f;
static float g_kd = 0.01f;
QueueHandle_t xUART1ReceiveQueue = NULL;
SemaphoreHandle_t xUART1ProcessingSemaphore = NULL;
static volatile uint32_t ulCurrentDMATransferSize = 0;
static volatile uint32_t ulLastDMATransferSize = 0;

void HAL_UART1_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == huart1.Instance)
  {
    ulCurrentDMATransferSize = Size;

    static UART1_DMA_Received_Data_t xReceivedData;

    if (ulCurrentDMATransferSize > 0)
    {
      memcpy(xReceivedData.buffer, s_uart1_rx_buf, ulCurrentDMATransferSize);
      xReceivedData.length = ulCurrentDMATransferSize;
    }
    if (xQueueSendFromISR(xUART1ReceiveQueue, &xReceivedData, &xHigherPriorityTaskWoken) != pdPASS)
    {
      // Process Queue Full Error>..
    }

    // Now Restart DMA Reception after clear the flag
    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx));

    if (HAL_UART_Receive_DMA(&huart1, s_uart1_rx_buf, UART1_PID_BUFFER_SIZE) != HAL_OK)
    {
      // Handle Error
    };
    // 發送信號量以通知數據已準備好
    xSemaphoreGiveFromISR(xUART1ProcessingSemaphore, &xHigherPriorityTaskWoken); // 釋放信號量
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void StartUART1DMAReceive(void)
{
  // memset(s_uart_rx_buf, 0, UART1_PID_BUFFER_SIZE);

  if (HAL_UART_Receive_DMA(&huart1, s_uart1_rx_buf, UART1_PID_BUFFER_SIZE) != HAL_OK)
  {
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET); // CS高
  u1_printf("CS HIGH\r\n");
  // HAL_Delay(100);
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET); // CS低
  u1_printf("CS LOW\r\n");
  // HAL_Delay(100);
  HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET); // CS高
  u1_printf("CS TEST DONE\r\n");

  // 尝试初始化
  if (MPU9250_Init(&mpu, &hspi1, MPU9250_CS_GPIO, MPU9250_CS_PIN))
  {
    u1_printf("MPU9250 init SUCCESSFUL\r\n");
  }
  else
  {
    u1_printf("MPU9250 init FAILED\r\n");
  }
  Test_SPI_Communication();
  ATGM336H_Init(&g_gps_data);
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

  /* creation of oledInitTask */
  oledInitTaskHandle = osThreadNew(OledInitTask, NULL, &oledInitTask_attributes);

  /* creation of uartRecvTask */
  uartRecvTaskHandle = osThreadNew(UartRecvTask, NULL, &uartRecvTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_SCL_Pin|OLED_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
#define SAMPLE_RATE 0.05f
  static uint32_t last_time = 0;
  float yaw = 0.00f; // 需要磁力计或陀螺积分计算
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

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // osDelay(1);
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_OledInitTask */
/**
 * @brief Function implementing the oledInitTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_OledInitTask */
void OledInitTask(void *argument)
{
  /* USER CODE BEGIN OledInitTask */
  /* Infinite loop

    */

  static bool initialized = false;

  // osDelay(1);

  OLED_PropTypeDef oled_cfg = {
      .scl_gpio_port = OLED_SCL_GPIO_Port,
      .scl_gpio_pin = OLED_SCL_Pin,
      .sda_gpio_port = OLED_SDA_GPIO_Port,
      .sda_gpio_pin = OLED_SDA_Pin};

  if (!initialized)
  {
    OLED_InitWithConfig(&oled_cfg);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "System OK", OLED_6X8);
    OLED_Update();
    initialized = true;
  }

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(500)); // 每 500ms 刷新一次

    GPS_PropTypeDef local_copy;
    if (xSemaphoreTake(xGnssMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      local_copy = g_gps_data;
      xSemaphoreGive(xGnssMutex);
    }
    else
    {
      continue;
    }

    if (local_copy.is_available)
    {
      OLED_Clear();
      char buf[32];

      OLED_Printf(0, 0, OLED_6X8, "Lat: %.5f", local_copy.latitude);

      // 显示经度
      OLED_Printf(0, 9, OLED_6X8, "Lon: %.5f", local_copy.longitude);

      // 显示时间
      OLED_Printf(0, 19, OLED_6X8, "UTC: %s", local_copy.UTC_time);

      // 显示速度
      OLED_Printf(0, 29, OLED_6X8, "Spd: %.1f kt", local_copy.speed);

      // 显示航向
      OLED_Printf(0, 39, OLED_6X8, "Hdg: %.1f", local_copy.heading);

      // 显示卫星数
      OLED_Printf(0, 49, OLED_6X8, "Sat: %d", local_copy.satellites);
    }
    else
    {
      // OLED_SetCursor(0, 0);
      OLED_ShowString(0, 9, "NMEA Not available.", OLED_6X8);
    }

    OLED_Update(); // 刷新屏幕
  }
  // osThreadExit();

  /* USER CODE END OledInitTask */
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
  /* Infinite loop */
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
      for (int i = 0; i < xReceivedData.length; i++)
      {
        // u1_printf("%02X ", xReceivedData.buffer[i]);
        if (xReceivedData.buffer[i] == '\n' || xReceivedData.buffer[i] == '\r')
        {
          xReceivedData.buffer[i] = '\0';
          json_end = (char *)&xReceivedData.buffer[i];
          break;
        }
        // 注意128堆栈够不够
      }
      if (json_end)
      {

        u1_printf("[UartRecvTask]Received JSON");
        printf("Now Processing JSON: %s\r\n", xReceivedData.buffer);
        PID_Params_t pid_params;
        if (UART1_ParsePIDData((char *)xReceivedData.buffer, xReceivedData.length,&pid_params))
        {
          g_kp = pid_params.kp;
          g_ki = pid_params.ki;
          g_kd = pid_params.kd;
          u1_printf("Updated PID params: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", g_kp, g_ki, g_kd);
        }
        else
        {
          u1_printf("Invalid PID JSON received.\r\n");
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
