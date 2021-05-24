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
#include <string.h>
#include <stdio.h>
#include "../Inc/Message.h"
#include "tm_stm32_mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_LEN 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_lpuart1_rx;

/* Definitions for TransmitTask_Se */
osThreadId_t TransmitTask_SeHandle;
const osThreadAttr_t TransmitTask_Se_attributes = {
  .name = "TransmitTask_Se",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TransmitTask_BT */
osThreadId_t TransmitTask_BTHandle;
const osThreadAttr_t TransmitTask_BT_attributes = {
  .name = "TransmitTask_BT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PingTask */
osThreadId_t PingTaskHandle;
const osThreadAttr_t PingTask_attributes = {
  .name = "PingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SR_Queue */
osMessageQueueId_t SR_QueueHandle;
const osMessageQueueAttr_t SR_Queue_attributes = {
  .name = "SR_Queue"
};
/* Definitions for BT_Queue */
osMessageQueueId_t BT_QueueHandle;
const osMessageQueueAttr_t BT_Queue_attributes = {
  .name = "BT_Queue"
};
/* USER CODE BEGIN PV */

uint8_t BT_BUFFER[BUFFER_LEN];
uint8_t SR_BUFFER[BUFFER_LEN] = "";

TM_MPU6050_t MPU6050;
TM_MPU6050_Interrupt_t MPU6050_Interrupts;

uint8_t read = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
void StartTransmitTask_Serial(void *argument);
void StartTransmitTask_BT(void *argument);
void StartPingTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA (&hlpuart1, SR_BUFFER, BUFFER_LEN);
  HAL_UART_Receive_IT(&huart4, BT_BUFFER, BUFFER_LEN);


  if (TM_MPU6050_Init(&MPU6050, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok) {
  		HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_SET);
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

  /* Create the queue(s) */
  /* creation of SR_Queue */
  SR_QueueHandle = osMessageQueueNew (64, 1, &SR_Queue_attributes);

  /* creation of BT_Queue */
  BT_QueueHandle = osMessageQueueNew (64, 1, &BT_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TransmitTask_Se */
  TransmitTask_SeHandle = osThreadNew(StartTransmitTask_Serial, NULL, &TransmitTask_Se_attributes);

  /* creation of TransmitTask_BT */
  TransmitTask_BTHandle = osThreadNew(StartTransmitTask_BT, NULL, &TransmitTask_BT_attributes);

  /* creation of PingTask */
  PingTaskHandle = osThreadNew(StartPingTask, NULL, &PingTask_attributes);

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

	  if(read == 1){
		  read = 0;
		  TM_MPU6050_ReadInterrupts(&MPU6050, &MPU6050_Interrupts);

		  if(MPU6050_Interrupts.F.MotionDetection == 1){
			  HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_SET);
		  }
		  if(MPU6050_Interrupts.F.DataReady == 1){
			  TM_MPU6050_ReadAll(&MPU6050);
			  char msg[500];
			  sprintf(msg,"\rAccX: %hu, AccY: %hu, AccZ: %hu\n GyrX: %hu, GyrY: %hu, GyrZ:%hu\n Temp: %f\n ",MPU6050.Accelerometer_X,MPU6050.Accelerometer_Y,MPU6050.Accelerometer_Z,MPU6050.Gyroscope_X,MPU6050.Gyroscope_Y,MPU6050.Gyroscope_Z, MPU6050.Temperature);
			  HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, sizeof(msg), 100);
			  /* Raw data are available for use when needed */
			  //MPU6050.Accelerometer_X;
			  //MPU6050.Accelerometer_Y;
			  //MPU6050.Accelerometer_Z;
			  //MPU6050.Gyroscope_X;
			  //MPU6050.Gyroscope_Y;
			  //MPU6050.Gyroscope_Z;
			  //MPU6050.Temperature;
		  }
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  huart4.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(redLED_GPIO_Port, redLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(greenLED_GPIO_Port, greenLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : redLED_Pin */
  GPIO_InitStruct.Pin = redLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(redLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : greenLED_Pin */
  GPIO_InitStruct.Pin = greenLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(greenLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &hlpuart1){
		osMessageQueuePut(SR_QueueHandle, SR_BUFFER, 0, 0);
		HAL_UART_Receive_DMA(&hlpuart1, SR_BUFFER, BUFFER_LEN);
	}
	if(huart->Instance == huart4.Instance){
		osMessageQueuePut(BT_QueueHandle, BT_BUFFER, 0, 0);
		HAL_UART_Receive_IT(&huart4, BT_BUFFER, BUFFER_LEN);
	}
}

void TM_EXTI_Handler(uint16_t GPIO_Pin) {
	/* Check for PIN */
	/* Read interrupts from MPU6050 */
	read = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTransmitTask_Serial */
/**
  * @brief  Function implementing the TransmitTask_Se thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTransmitTask_Serial */
void StartTransmitTask_Serial(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	char msg_sr[64];
	uint16_t count_sr = 0;
	char c[1];

  /* Infinite loop */
	for(;;)
	{
		memset(msg_sr,0,64);
		count_sr = osMessageQueueGetCount(SR_QueueHandle);
		if(count_sr > 0){
			for (int i = 0; i < count_sr; i++) {
				osMessageQueueGet(SR_QueueHandle, c, 0, 100);
				strncat(msg_sr, c, 1);
			}
			HAL_UART_Transmit(&huart4, (uint8_t *)msg_sr, sizeof(msg_sr), 100);
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg_sr, sizeof(msg_sr), 100);
		}
		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTransmitTask_BT */
/**
* @brief Function implementing the TransmitTask_BT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitTask_BT */
void StartTransmitTask_BT(void *argument)
{
  /* USER CODE BEGIN StartTransmitTask_BT */

	char msg_bt[64];
	uint16_t count_bt = 0;
	char b[1];
  /* Infinite loop */
	for(;;)
	{
		memset(msg_bt,0,64);
		count_bt = osMessageQueueGetCount(BT_QueueHandle);
		if(count_bt > 0){
			for (int j = 0; j < count_bt; j++) {
				osMessageQueueGet(BT_QueueHandle, b, 0, 100);
				strncat(msg_bt,b,1);
			}
			HAL_UART_Transmit(&huart4, (uint8_t *)msg_bt, sizeof(msg_bt), 100);
			HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg_bt, sizeof(msg_bt), 100);
		}
		osDelay(100);
	}
  /* USER CODE END StartTransmitTask_BT */
}

/* USER CODE BEGIN Header_StartPingTask */
/**
* @brief Function implementing the PingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPingTask */
void StartPingTask(void *argument)
{
  /* USER CODE BEGIN StartPingTask */
//	char msg[8];

//	uint8_t * pointer;
//	uint8_t msgStr[64];

//	uint8_t i=0;
//	reset( msgStr, pointer);
//	addStr(msg_,7, pointer);

  /* Infinite loop */
  for(;;)
  {
	  /*i++;

	  switch(i){
		  case 0:
			  strcpy(msg,"012345\0");
			  break;
		  case 1:
			  strcpy(msg,"123456\0");
			  break;
		  case 2:
			  strcpy(msg,"234567\0");
			  break;
		  case 3:
			  strcpy(msg,"345678\0");
			  break;
		  case 4:
			  strcpy(msg,"456789\0");
			  break;
		  default:
			  i = 0;
			  //j = 0;
	  }
	  HAL_UART_Transmit(&huart4, (uint8_t *)msg, 7, 100);*/
	  sendPose();
	  osDelay(150);
  }
  /* USER CODE END StartPingTask */
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
  if (htim->Instance == TIM6) {
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
