/* USER CODE BEGIN Header */
/*
 Autores: Lucyanno Frota (2016116214) e Milena Mori (2016193815)
 Turma: PL2
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifndef _STDIO_H_
#include <stdio.h>
#endif
#ifndef _STRING_H_
#include <string.h>
#endif
#ifndef _STDLIB_H_
#include <stdlib.h>
#endif
#ifndef INC_FUNCTIONS_C_
#include "functions.h"
#endif
#ifndef  _MATH_H_
#include <math.h>
#endif
#ifndef _LCD_H_
#include "lcd.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SerialInputDete */
osThreadId_t SerialInputDeteHandle;
const osThreadAttr_t SerialInputDete_attributes = {
  .name = "SerialInputDete",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialDisplay */
osThreadId_t SerialDisplayHandle;
const osThreadAttr_t SerialDisplay_attributes = {
  .name = "SerialDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for buttonDetection */
osThreadId_t buttonDetectionHandle;
const osThreadAttr_t buttonDetection_attributes = {
  .name = "buttonDetection",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LCDDisplay */
osThreadId_t LCDDisplayHandle;
const osThreadAttr_t LCDDisplay_attributes = {
  .name = "LCDDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SerialInput */
osMessageQueueId_t SerialInputHandle;
uint8_t SerialInputBuffer[ 128 * 1 ];
osStaticMessageQDef_t SerialInputControlBlock;
const osMessageQueueAttr_t SerialInput_attributes = {
  .name = "SerialInput",
  .cb_mem = &SerialInputControlBlock,
  .cb_size = sizeof(SerialInputControlBlock),
  .mq_mem = &SerialInputBuffer,
  .mq_size = sizeof(SerialInputBuffer)
};
/* Definitions for CommandQueu */
osMessageQueueId_t CommandQueuHandle;
uint8_t CommandQueuBuffer[ 16 * sizeof( struct Command ) ];
osStaticMessageQDef_t CommandQueuControlBlock;
const osMessageQueueAttr_t CommandQueu_attributes = {
  .name = "CommandQueu",
  .cb_mem = &CommandQueuControlBlock,
  .cb_size = sizeof(CommandQueuControlBlock),
  .mq_mem = &CommandQueuBuffer,
  .mq_size = sizeof(CommandQueuBuffer)
};
/* USER CODE BEGIN PV */
uint8_t rxBuffer[1] = "";
enum SerialDisp DispState = 0;
enum LCDDispState LCDState = LCDhome;
uint16_t tvalue = 0;
uint16_t prodV = 0;
uint16_t selectedProd;
uint16_t lastProd;
uint8_t vchanged;
uint8_t dip_LC = 0;

uint8_t disp;
uint8_t sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartDefaultTask(void *argument);
void SerialInput_Detect(void *argument);
void Serial_Display(void *argument);
void StartbuttonDetection(void *argument);
void LCD_Display(void *argument);

static void MX_NVIC_Init(void);
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

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&hlpuart1, rxBuffer, 1);
	LCD_Init();
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
  /* creation of SerialInput */
  SerialInputHandle = osMessageQueueNew (128, 1, &SerialInput_attributes);

  /* creation of CommandQueu */
  CommandQueuHandle = osMessageQueueNew (16, sizeof(struct Command), &CommandQueu_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SerialInputDete */
  SerialInputDeteHandle = osThreadNew(SerialInput_Detect, NULL, &SerialInputDete_attributes);

  /* creation of SerialDisplay */
  SerialDisplayHandle = osThreadNew(Serial_Display, NULL, &SerialDisplay_attributes);

  /* creation of buttonDetection */
  buttonDetectionHandle = osThreadNew(StartbuttonDetection, NULL, &buttonDetection_attributes);

  /* creation of LCDDisplay */
  LCDDisplayHandle = osThreadNew(LCD_Display, NULL, &LCDDisplay_attributes);

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
	while (1) {
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* LPUART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_DB5_Pin|LCD_DB4_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|red_Led_Pin|LCD_RW_Pin|blue_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(green_Led_GPIO_Port, green_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LCD_DB6_Pin|LCD_DB7_Pin|YellowLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : sensor_Pin */
  GPIO_InitStruct.Pin = sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB5_Pin LCD_DB4_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_DB5_Pin|LCD_DB4_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin red_Led_Pin LCD_RW_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|red_Led_Pin|LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : green_Led_Pin */
  GPIO_InitStruct.Pin = green_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(green_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DB6_Pin LCD_DB7_Pin YellowLed_Pin */
  GPIO_InitStruct.Pin = LCD_DB6_Pin|LCD_DB7_Pin|YellowLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : blue_Led_Pin */
  GPIO_InitStruct.Pin = blue_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(blue_Led_GPIO_Port, &GPIO_InitStruct);

  /**/
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_PB7);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	osMessageQueuePut(SerialInputHandle, rxBuffer, 0, 0);
	//	HAL_GPIO_TogglePin(green_Led_GPIO_Port, green_Led_Pin);
	HAL_UART_Receive_DMA(&hlpuart1, rxBuffer, 1);
}


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
	state_t state = INIT;
	uint16_t coinV;
	/* Infinite loop*/
	for(;;){
		switch (state){
		case INIT:{
			tvalue = 0;
			selectedProd = 0;
			lastProd = 0;
			prodV = 0;
			coinV = 0;
			disp = 0;
			state = WAIT;
			break;
		}
		case WAIT:{
			enum CType t;
			uint16_t v = readBuffer(&t,CommandQueuHandle);
			if(t != unknown){
				switch (t){
				case coin:
					coinV = v;
					break;
				case product:
					selectedProd = v;
					break;
				default:
					break;
				}
			}
			if (coinV != 0) state = ADD;
			else{
				if (selectedProd != 0 || lastProd != selectedProd){
					if (prodV == 0 || lastProd != selectedProd) state = SEL;
					else {
						if(prodV <= tvalue) state = DISP;
						else state = WAIT;
					}
				}
				else state = WAIT;
			}
			if(tvalue != 0 || selectedProd != 0) DispState = vending;
			else DispState = home;
			lastProd = selectedProd;
			break;
		}
		case ADD:{
			LCDState = LCDvending;
			tvalue += coinV;
			coinV = 0;
			vchanged = 1;
			state = WAIT;
			break;
		}
		case SEL: // product value depends on its column
		{
			LCDState = LCDvending;
			uint8_t col = selectedProd - 10*floor(selectedProd/10);
			switch (col){
			case 1:{
				prodV = 150;
				break;
			}
			case 2:{
				prodV = 250;
				break;
			}
			case 3:{
				prodV = 50;
				break;
			}
			case 4:{
				prodV = 400;
				break;
			}
			case 5:{
				prodV = 560;
				break;
			}
			default:
				break;
			}
			vchanged = 1;
			state = WAIT;
			break;
		}
		case DISP:{
			state = BLINK;
			disp = 1;
			DispState = dispensing;
			LCDState = LCDdispensing;
			break;
		}
		case BLINK:{
			if(sensor == 0){
				dispLEDs(200);
				//				state = BLINK;
				continue;
			}
			else{
				blinkAll(300);
				osDelay(200);
				blinkAll(300);
				sensor = 0;
				disp = 0;
				state = INIT;
			}
			//			while (sensor == 0) dispLEDs(200);
			break;
		}
		default:
			break;
		}
		osDelay(50);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SerialInput_Detect */
/**
 * @brief Function implementing the SerialInputDete thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SerialInput_Detect */
void SerialInput_Detect(void *argument)
{
  /* USER CODE BEGIN SerialInput_Detect */
	uint32_t count = 0;
	//	char outp[64 + 10 + 4];
	char msg[64];
	char c[1];
	//	char nl[] = "\n\r";
	/* Infinite loop */
	for (;;) {
		memset(msg, 0, 64);
		count = osMessageQueueGetCount(SerialInputHandle);
		if (count > 0) {
			blinkBlue(400);
			for (int i = 0; i < count; i++) {
				osMessageQueueGet(SerialInputHandle, c, 0, 100);
				strncat(msg, c, 1);
			}
			if (count > 0) {
				//				memset(outp, 0, 74);
				struct Command a = stringToCommand(msg);
				//				strcpy(outp, "\n\rType: ");
				//				char c[] = "Coin";
				//				char p[] = "Product";
				//				char v1[] = ", Value1: ";
				//				char v2[] = ", Value2: ";
				//				char vl1[5];
				//				char vl2[5];
				//				if(a.type == unknown) continue;
				//				if (a.type == coin) strncat((char*) outp, c, strlen(c));
				//				else strncat((char*) outp, p, strlen(p));
				//				strncat((char*) outp, v1, strlen(v1));
				//				itoa(a.value1, vl1, 10);
				//				strncat((char*) outp, vl1, strlen(vl1));
				//				strncat((char*) outp, v2, strlen(v2));
				//				itoa(a.value2, vl2, 10);
				//				strncat((char*) outp, vl2, strlen(vl2));
				//				strncat((char*) outp, nl, strlen(nl));
				osMessageQueuePut(CommandQueuHandle, &a, 0, 300);
				//HAL_UART_Transmit(&hlpuart1, (uint8_t*) outp, strlen(outp),300);
				//				memset(outp, 0, 74);
				//				sprintf(outp,"\r\nType: Coin, Value1: %i, Value2: %i\r\n",a.value1,a.value2);
				//HAL_UART_Transmit(&hlpuart1, (uint8_t*) outp, strlen(outp),300);
			}
		}
		osDelay(100);
	}
  /* USER CODE END SerialInput_Detect */
}

/* USER CODE BEGIN Header_Serial_Display */
/**
 * @brief Function implementing the SerialDisplay thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Serial_Display */
void Serial_Display(void *argument)
{
  /* USER CODE BEGIN Serial_Display */
	char header[3] = "\n\r";
	char mainScreen[40];
	char vendingScreen[40];
	char dispensingScreen[40];
	sprintf(mainScreen,"%sInsert a coin,or\n\rselect a product",header);
	enum SerialDisp lastDispState = 0, localState = 0;
	uint8_t counter = 0;


	/* Infinite loop */
	for(;;)
	{
		localState = DispState;
		if((localState != lastDispState) || vchanged || (counter >= 20)){
			switch(DispState){
			case home:
				dip_LC = 0;
				HAL_UART_Transmit(&hlpuart1, (uint8_t*) mainScreen, strlen(mainScreen),30);
				HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_RESET);
				if((localState != lastDispState)) LCDState = LCDhome;
				break;
			case vending:
				sprintf(vendingScreen,"%sPrice = %i\n\rTotal = %i",header,prodV,tvalue);
				HAL_UART_Transmit(&hlpuart1, (uint8_t*) vendingScreen, strlen(vendingScreen),30);
				if((localState != lastDispState) || vchanged){
					vchanged = 0;
					LCDState = LCDvending;
				}
				break;
			case dispensing:
				sprintf(dispensingScreen,"%sDispensing\n\rProduct: %i",header,selectedProd);
				HAL_UART_Transmit(&hlpuart1, (uint8_t*) dispensingScreen, strlen(dispensingScreen),30);
				HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_SET);
				if(dip_LC == 0) LCDState = LCDdispensing;
				break;
			}
			counter = 0;
		}
		lastDispState = localState;
		counter++;
		osDelay(50);
	}
  /* USER CODE END Serial_Display */
}

/* USER CODE BEGIN Header_StartbuttonDetection */
/**
 * @brief Function implementing the buttonDetection thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartbuttonDetection */
void StartbuttonDetection(void *argument)
{
  /* USER CODE BEGIN StartbuttonDetection */
	/* Infinite loop */
	for(;;)
	{
		if((HAL_GPIO_ReadPin(sensor_GPIO_Port, sensor_Pin) == GPIO_PIN_SET) && disp == 1) sensor = 1;
		osDelay(50);
	}
  /* USER CODE END StartbuttonDetection */
}

/* USER CODE BEGIN Header_LCD_Display */
/**
 * @brief Function implementing the LCDDisplay thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LCD_Display */
void LCD_Display(void *argument)
{
  /* USER CODE BEGIN LCD_Display */
	LCD_Init();
	char mainScreen1[] = "Insert a coin,or";
	char mainScreen2[] = "select a product";

	const char _vendingScreen1[] = "Price = ";
	const char _vendingScreen2[] = "Total = ";

	char vendingScreen1[16];
	char vendingScreen2[16];

	char dispensingScreen1[] = "Dispensing";
	const char _dispensingScreen2[] = "Product: ";

	char dispensingScreen2[16];

	/* Infinite loop */
	for(;;)
	{
		switch(LCDState){
			case LCDidle:
				break;
			case LCDhome:
				LCD_Clear();
				LCD_Write(mainScreen1,mainScreen2);
				LCDState = LCDidle;
				break;
			case LCDvending:
				LCD_Clear();
				memset(vendingScreen1,0,16);
				memset(vendingScreen2,0,16);
				sprintf(vendingScreen1,"%s%i",_vendingScreen1,prodV);
				sprintf(vendingScreen2,"%s%i",_vendingScreen2,tvalue);
				LCD_Write(vendingScreen1,vendingScreen2);
				LCDState = LCDidle;
				break;
			case LCDdispensing:
				LCD_Clear();
				memset(dispensingScreen2,0,16);
				sprintf(dispensingScreen2,"%s%i",_dispensingScreen2,selectedProd);
				LCD_Write(dispensingScreen1,dispensingScreen2);
				dip_LC = 1;
				LCDState = LCDidle;
				break;
		}
		osDelay(50);
	}
  /* USER CODE END LCD_Display */
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
	char t[] = "ErrorHandler\n";
	HAL_UART_Transmit(&hlpuart1,(uint8_t *)t, strlen(t), 300);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
