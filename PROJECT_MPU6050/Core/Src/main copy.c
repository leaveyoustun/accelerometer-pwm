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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMPI2C_HandleTypeDef hfmpi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorReadTask */
osThreadId_t SensorReadTaskHandle;
const osThreadAttr_t SensorReadTask_attributes = {
  .name = "SensorReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PWMControlTask */
osThreadId_t PWMControlTaskHandle;
const osThreadAttr_t PWMControlTask_attributes = {
  .name = "PWMControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LCDControlTask */
osThreadId_t LCDControlTaskHandle;
const osThreadAttr_t LCDControlTask_attributes = {
  .name = "LCDControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMPI2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void sensor_read_task(void *argument);
void pwm_control_task(void *argument);
void lcd_control_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t g_counter = 0;
volatile uint32_t g_channel_1_state = 16;
volatile uint32_t g_channel_2_state = 8;




const mpu6050_accel_data_t error_offset = {
  .x = 490,
  .y = -40,
  .z = 16788 - 16384
};

#define PWM_PULSE_MIN 0
#define PWM_PULSE_MAX 40

#define ANGLE_POS_MIN 0
#define ANGLE_POS_MAX 90

#define GRID_SIZE 10
char grid[GRID_SIZE][GRID_SIZE];
char uart_buffer[128];

// Normalize accelerometer value to grid index
int map_to_grid_index(int16_t accel, int16_t min_val, int16_t max_val) {
    if (accel < min_val) accel = min_val;
    if (accel > max_val) accel = max_val;
    return ((accel - min_val) * (GRID_SIZE - 1)) / (max_val - min_val);
}

void draw_grid(int16_t x, int16_t y) {
    // Clear screen and move cursor to home
    HAL_UART_Transmit(&huart2, (uint8_t*)"\033[2J\033[H", strlen("\033[2J\033[H"), 100);

    // Clear grid
    for (int i = 0; i < GRID_SIZE; i++)
        for (int j = 0; j < GRID_SIZE; j++)
            grid[i][j] = '.';

    // Map to grid coordinates
    int gx = map_to_grid_index(x, -16000, 16000);
    int gy = map_to_grid_index(y, -16000, 16000);

    grid[gy][gx] = 'X';  // Place marker

    // Print grid to UART
    for (int i = 0; i < GRID_SIZE; i++) {
        int idx = 0;
        for (int j = 0; j < GRID_SIZE; j++)
            uart_buffer[idx++] = grid[i][j];
        uart_buffer[idx++] = '\r';
        uart_buffer[idx++] = '\n';
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buffer, idx, 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100); // newline
}


//map function: re-maps a number from one range to another

long map(long x, long in_min, long in_max, long out_min, long out_max){
  // perform mapping

  long result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

  //clam the result to the output range

  if (result > out_max){
    result = out_max;
  }else if (result < out_min){
    result = out_min;
  }
  return result;
}

void change_pwm_duty_cycle(uint32_t pwm_pulse, uint8_t timer_channel){
  __HAL_TIM_SET_COMPARE(&htim2, timer_channel, pwm_pulse);
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
  MX_USART2_UART_Init();
  MX_FMPI2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  if (mpu6050_init(&hfmpi2c1, MPU6050_I2C_ADDR) != MPU6050_OK){
    Error_Handler();
  }

  mpu6050_disable_interrupt(&hfmpi2c1, ALL_INT);
  mpu6050_interrupt_config(&hfmpi2c1, INT_LEVEL_ACTIVE_HIGH);
  mpu6050_enable_interrupt(&hfmpi2c1, RAW_RDY_INT);

#if 1
  if (mpu6050_configure_low_pass_filter(&hfmpi2c1, DLPF_CFG_21HZ) != MPU6050_OK){
    Error_Handler();
  }
#endif

  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  if (HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK){
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK){
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK){
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK){
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorReadTask */
  SensorReadTaskHandle = osThreadNew(sensor_read_task, NULL, &SensorReadTask_attributes);

  /* creation of PWMControlTask */
  PWMControlTaskHandle = osThreadNew(pwm_control_task, NULL, &PWMControlTask_attributes);

  /* creation of LCDControlTask */
  LCDControlTaskHandle = osThreadNew(lcd_control_task, NULL, &LCDControlTask_attributes);

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
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    

#ifdef PWM_DEBUG
    g_counter = __HAL_TIM_GET_COUNTER(&htim2);
#endif


    
    //HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMPI2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMPI2C1_Init(void)
{

  /* USER CODE BEGIN FMPI2C1_Init 0 */

  /* USER CODE END FMPI2C1_Init 0 */

  /* USER CODE BEGIN FMPI2C1_Init 1 */

  /* USER CODE END FMPI2C1_Init 1 */
  hfmpi2c1.Instance = FMPI2C1;
  hfmpi2c1.Init.Timing = 0x0000020B;
  hfmpi2c1.Init.OwnAddress1 = 0;
  hfmpi2c1.Init.AddressingMode = FMPI2C_ADDRESSINGMODE_7BIT;
  hfmpi2c1.Init.DualAddressMode = FMPI2C_DUALADDRESS_DISABLE;
  hfmpi2c1.Init.OwnAddress2 = 0;
  hfmpi2c1.Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK;
  hfmpi2c1.Init.GeneralCallMode = FMPI2C_GENERALCALL_DISABLE;
  hfmpi2c1.Init.NoStretchMode = FMPI2C_NOSTRETCH_DISABLE;
  if (HAL_FMPI2C_Init(&hfmpi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_FMPI2CEx_ConfigAnalogFilter(&hfmpi2c1, FMPI2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMPI2C1_Init 2 */

  /* USER CODE END FMPI2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 8;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LCD_D7_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_E_Pin|LCD_D5_Pin|LCD_D4_Pin|LCD_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LCD_D7_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LCD_D7_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_D5_Pin LCD_D4_Pin LCD_D6_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_D5_Pin|LCD_D4_Pin|LCD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_sensor_read_task */
/**
* @brief Function implementing the SensorReadTask_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_read_task */
void sensor_read_task(void *argument)
{
  /* USER CODE BEGIN sensor_read_task */

  mpu6050_accel_data_t g_accel_data;

  uint32_t previous_tick = osKernelGetTickCount();
  int16_t roll_angle;
  int16_t pitch_angle;
  int16_t roll_angle_filtered;
  int16_t pitch_angle_filtered;
  uint32_t roll_pitch_angles;

  float dt = 0;

  KalmanFilter kf_roll;
  kalman_filter_init(&kf_roll);

  KalmanFilter kf_pitch;
  kalman_filter_init(&kf_pitch);

  /* Infinite loop */
  for(;;)
  {
    //sensor read
    if (mpu6050_read_accelerometer_data(&hfmpi2c1, MPU6050_I2C_ADDR, &g_accel_data) != MPU6050_OK){
      Error_Handler();
    }
    
    g_accel_data = mpu6050_accelerometer_callibration(&error_offset, &g_accel_data);

    uint32_t current_tick = osKernelGetTickCount();
    dt = (current_tick - previous_tick) / 1000.0f;
    previous_tick = current_tick;

    roll_angle = atan2(g_accel_data.y, g_accel_data.z) * (180 / M_PI); 
    pitch_angle = atan2((-g_accel_data.x), sqrt(g_accel_data.y * g_accel_data.y + g_accel_data.z *g_accel_data.z)) * (180 / M_PI);

    roll_angle_filtered = (int16_t)kalman_filter_get_angle(&kf_roll, roll_angle, dt);
    pitch_angle_filtered = (int16_t)kalman_filter_get_angle(&kf_pitch, pitch_angle, dt);

    roll_pitch_angles = roll_angle_filtered;
    roll_pitch_angles = (roll_angle_filtered << 16) | pitch_angle_filtered;
    //send notificiation to pwm control task
    xTaskNotify((TaskHandle_t)PWMControlTaskHandle, roll_pitch_angles, eSetValueWithOverwrite);
    xTaskNotify((TaskHandle_t)LCDControlTaskHandle, roll_pitch_angles, eSetValueWithOverwrite);
    //enter blocked state
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
  }
  /* USER CODE END sensor_read_task */
}

/* USER CODE BEGIN Header_pwm_control_task */
/**
* @brief Function implementing the PWMControlTask_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pwm_control_task */
void pwm_control_task(void *argument)
{
  /* USER CODE BEGIN pwm_control_task */

  
  int16_t roll_angle_filtered;
  int16_t pitch_angle_filtered;
  int16_t abs_roll_angle_filtered;
  int16_t abs_pitch_angle_filtered;

  uint32_t roll_pitch_angles;

  

  /* Infinite loop */
  for(;;)
  {
    // wakes up on notification 
    xTaskNotifyWait(0, ULONG_MAX, &roll_pitch_angles, portMAX_DELAY);

    pitch_angle_filtered = (int16_t) roll_pitch_angles & 0xFFFF;
    roll_angle_filtered = (int16_t) ((roll_pitch_angles >> 16)  & 0xFFFF);
    
    uint8_t raw_channel = (roll_angle_filtered < 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
    uint8_t pitch_channel = (pitch_angle_filtered < 0) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;

    abs_roll_angle_filtered = (roll_angle_filtered < 0 ) ? -roll_angle_filtered : roll_angle_filtered;
    abs_pitch_angle_filtered = (pitch_angle_filtered < 0 ) ? -pitch_angle_filtered : pitch_angle_filtered;

    uint32_t raw_pwm_pulse = map(abs_roll_angle_filtered, ANGLE_POS_MIN, ANGLE_POS_MAX, PWM_PULSE_MIN, PWM_PULSE_MAX);
    uint32_t pitch_pwm_pulse = map(abs_pitch_angle_filtered, ANGLE_POS_MIN, ANGLE_POS_MAX, PWM_PULSE_MIN, PWM_PULSE_MAX);

    change_pwm_duty_cycle(raw_pwm_pulse, raw_channel);
    change_pwm_duty_cycle(pitch_pwm_pulse, pitch_channel);
  }
  /* USER CODE END pwm_control_task */
}

/* USER CODE BEGIN Header_lcd_control_task */
/**
* @brief Function implementing the LCDControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lcd_control_task */
void lcd_control_task(void *argument)
{
  /* USER CODE BEGIN lcd_control_task */
  Lcd_PortType ports[] = { LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port };
  Lcd_PinType pins[] = {LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin};
  Lcd_HandleTypeDef lcd;
  lcd = Lcd_create(ports, pins, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_GPIO_Port, LCD_E_Pin, LCD_4_BIT_MODE);

  int16_t roll_angle_filtered;
  int16_t pitch_angle_filtered;

  uint32_t roll_pitch_angles;


  /* Infinite loop */
  for(;;)
  {
    // wakes up on notification 
    xTaskNotifyWait(0, ULONG_MAX, &roll_pitch_angles, portMAX_DELAY);

    pitch_angle_filtered = (int16_t) roll_pitch_angles & 0xFFFF;
    roll_angle_filtered = (int16_t) ((roll_pitch_angles >> 16)  & 0xFFFF);

    // Format buffer with roll and pitch
    char lcd_line1[17];
    char lcd_line2[17];

    snprintf(lcd_line1, sizeof(lcd_line1), "Roll: %+3d deg", roll_angle_filtered);
    snprintf(lcd_line2, sizeof(lcd_line2), "Pitch: %+3d deg", pitch_angle_filtered);

    // Clear screen and update LCD
    Lcd_clear(&lcd);
    Lcd_cursor(&lcd, 0, 0);
    Lcd_string(&lcd, lcd_line1);
    Lcd_cursor(&lcd, 1, 0);
    Lcd_string(&lcd, lcd_line2);
  }
  /* USER CODE END lcd_control_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  g_counter = 0;
  g_channel_1_state = 16;
  g_channel_2_state = 8;

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

#ifdef  USE_FULL_ASSERT
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
