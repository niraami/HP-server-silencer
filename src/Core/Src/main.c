/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "curves.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/**
 * Maximum number of messages that are pending from the TIM1 IRQ handler. This
 * buffer is used to handle bursts of messages from the IRQ - ex. when the PWM
 * signal is slowly changing.
 */
#define TIM1_IRQ_BUFFER_MAX_BACKLOG 4

/**
 * Maximum number of messages/events stored in the TIM1 IRQ buffer
 * This value directly affects how much smoothing is applied to the input signal
 * @note High values may lead to artifacts
 */
#define TIM1_IRQ_BUFFER_LENGTH 8

/**
 * Maximum lifespan of a buffer entry (in ms)
 * Specifies the maximum amount of time an entry is going to be counted into
 * the input average. After this time, the entry is guaranteed to be thrown
 * out. This might also happen much sooner if the interval at which the entries
 * are being added into the buffer is smaller than this value divided by the
 * buffer's length.
 * @note I recommend setting this value to be neatly divisible by the buffer's
 * length, otherwise rounding will take place anyways
 */
#define TIM1_IRQ_BUFFER_LIFESPAN 400

/**
 * This duty cycle is used on edge-cases where calculations fail or receive
 * invalid data
 * The default value is 100
 */
#define FAILSAFE_DUTY_CYCLE 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for inputFilter */
osThreadId_t inputFilterHandle;
const osThreadAttr_t inputFilter_attributes = {
  .name = "inputFilter",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/**
 * Message queue used to store reported CCR values from the TIM1_CC_IRQ handler
 */
MessageBufferHandle_t tim1_irq_backlog = NULL;
/**
 * A small buffer used to transfer the filtered signal from the input filter to
 * the main thread.
 */
MessageBufferHandle_t tim1_filtered_buffer = NULL;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void InputFilterTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Sets the PWM duty cycle for the desired TIMER channel
 * @param htim Timer handle
 * @param channel PWM generation channel
 * @param duty_cycle Desired duty cycle in percent. Resolution is calculated
 * based on the period configured of the timer instance.
 */
static void setDutyCycle(TIM_HandleTypeDef* const htim,
    uint32_t channel, float duty_cycle)
{
  // Constrain the provided duty cycle to avoid undefined behaviour
  if (duty_cycle > 100) duty_cycle = 100;
  if (duty_cycle < 0) duty_cycle = 0;

  // Calculate the period resolution (1% of the period value)
  float pw_resolution = ((float)htim->Init.Period + 1.0f) / 100.0f;
  // Calculate & set the actual PWM period
  uint16_t pw_desired = pw_resolution * duty_cycle;
  __HAL_TIM_SET_COMPARE(htim, channel, pw_desired);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  tim1_irq_backlog = xMessageBufferCreate(
    TIM1_IRQ_BUFFER_MAX_BACKLOG * (sizeof(CCRPair) + sizeof(size_t)) );
  tim1_filtered_buffer = xMessageBufferCreate(
    (sizeof(CCRPair) + sizeof(size_t)) );
  /**
   * Asserts will fail if there isn't sufficient FreeRTOS heap available for the
   * buffers to be created successfully.
   */
  assert_param(tim1_irq_backlog != NULL);
  assert_param(tim1_filtered_buffer != NULL);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Start interrupts on TIM1 CH1 & CH2
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

  // Start PWM generation on TIM2 CH1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /**
   * Clear the TIM1 IRQ message queue, just in case anything got added during
   * initialization
   * @note This is mostly useful while using GDB for debugging
   */
  xMessageBufferReset(tim1_irq_backlog);

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

  /* creation of inputFilter */
  inputFilterHandle = osThreadNew(InputFilterTask, NULL, &inputFilter_attributes);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 512 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 10 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 128;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 PA8
                           PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

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
  CCRPair evt = { 0, 0 };

  /* Infinite loop */
  for(;;)
  {
    /** If > 0, we've received a message and it was stored to `evt` */
    size_t recv = xMessageBufferReceive(tim1_filtered_buffer,
      &evt, sizeof(evt), portMAX_DELAY);

    if (recv == 0) {
      // Continue waiting if the specified timeout expired
      continue;
    }

    /** Calculated input PWM frequency */
    uint32_t pwm_freq = HAL_RCC_GetSysClockFreq() / (TIM1->PSC + 1) / evt.CCR2;
    /** Ignore bogus input frequencies (outside of the PWM spec) */
    if (pwm_freq > 28000 || pwm_freq < 21000) {
      continue;
    }

    /** Calculated input PWM duty cycle as percentage */
    float pwm_duty = (evt.CCR2 - evt.CCR1) / (evt.CCR2 / 100.0f);
    // Invert the duty cycle, as it's polarity is reversed (low instead of high)
    pwm_duty = 100.0f - pwm_duty;

    // Find the first curve range that matches our input
    uint8_t n_curves = sizeof(k_curve) / sizeof(CurvePoint);

    for (uint8_t i = 0; i < n_curves; i++) {
      CurvePoint point = k_curve[i];

      if (point.in_start < pwm_duty && point.in_end > pwm_duty) {
        /** Target PWM duty cycle calculated based on the point's parameters */
        float target = point.out_start + (
          (point.out_end - point.out_start) * (pwm_duty - point.in_start) /
            (point.in_end - point.in_start)
        );

        setDutyCycle(&htim2, TIM_CHANNEL_1, target);

        break;
      }

      if (i + 1 == n_curves) {
        // Set failsafe duty cycle if no curve was found
        setDutyCycle(&htim2, TIM_CHANNEL_1, FAILSAFE_DUTY_CYCLE);
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_InputFilterTask */
/**
* @brief Function implementing the inputFilter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_InputFilterTask */
void InputFilterTask(void *argument)
{
  /* USER CODE BEGIN InputFilterTask */
  CCRPair data[TIM1_IRQ_BUFFER_LENGTH] = {0};
  CCRPair buffer = {0, 0};
  CCRPair sum = {0, 0};

  CCRPair* head_ptr = data;

  size_t recv = 0;
  size_t n_entries = 0;

  /** Used to optimize CPU cycles when no action is required */
  uint32_t cycle_ms = portMAX_DELAY;

  /* Infinite loop */
  for(;;)
  {
    // If > 0, we've received a message and it was stored to the buffer
    recv = xMessageBufferReceive(tim1_irq_backlog, &buffer,
      sizeof(CCRPair), cycle_ms);

    if (!recv && n_entries <= 1) {
      // Wait for a message indefinitely
      cycle_ms = portMAX_DELAY;

      continue;
    } else {
      cycle_ms = pdMS_TO_TICKS(
        TIM1_IRQ_BUFFER_LIFESPAN / TIM1_IRQ_BUFFER_LENGTH );
    }

    // Remove the entry that is about to get overwritten from the sum
    sum = (CCRPair) {
      sum.CCR1 - head_ptr->CCR1,
      sum.CCR2 - head_ptr->CCR2
    };

    if (!recv) {
      // Clear the current entry if no message was received
      *head_ptr = (CCRPair) {0, 0};
      // Decrement the number of available entries
      n_entries--;
    } else {
      // Append the new entry to the buffer & the rolling sum
      *head_ptr = buffer;

      sum = (CCRPair) {
        sum.CCR1 + head_ptr->CCR1,
        sum.CCR2 + head_ptr->CCR2
      };

      // Only increment the number of entries if the buffer isn't already full
      if (n_entries < TIM1_IRQ_BUFFER_LENGTH) {
        n_entries++;
      }
    }

    // Push the head_ptr address forward & check for bounds
    if (++head_ptr == &data[TIM1_IRQ_BUFFER_LENGTH]) {
      head_ptr = data;
    }

    // Send new input value to the main task
    CCRPair avg = {
      sum.CCR1 / n_entries,
      sum.CCR2 / n_entries
    };
    xMessageBufferSend(tim1_filtered_buffer, &avg, sizeof(CCRPair), 0);
  }
  /* USER CODE END InputFilterTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
