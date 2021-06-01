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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU6050
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_MEM_START ((void*)0x1800)

struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};
struct hal_s {
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned short report;
	unsigned short dmp_features;
	unsigned char motion_int_mode;
	struct rx_s rx;
};
struct hal_s hal = {0};


static signed char gyro_orientation[9] = {-1, 0, 0,
		0,-1, 0,
		0, 0, 1};
volatile unsigned char rx_new;

enum packet_type_e {
	PACKET_TYPE_ACCEL,
	PACKET_TYPE_GYRO,
	PACKET_TYPE_QUAT,
	PACKET_TYPE_TAP,
	PACKET_TYPE_ANDROID_ORIENT,
	PACKET_TYPE_PEDO,
	PACKET_TYPE_MISC
};

void send_packet(char packet_type, void *data)
{
	#define MAX_BUF_LENGTH  (18)
	char buf[MAX_BUF_LENGTH], length;

	memset(buf, 0, MAX_BUF_LENGTH);
	buf[0] = '$';
	buf[1] = packet_type;

	if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
		short *sdata = (short*)data;
		buf[2] = (char)(sdata[0] >> 8);
		buf[3] = (char)sdata[0];
		buf[4] = (char)(sdata[1] >> 8);
		buf[5] = (char)sdata[1];
		buf[6] = (char)(sdata[2] >> 8);
		buf[7] = (char)sdata[2];
		length = 8;
	} else if (packet_type == PACKET_TYPE_QUAT) {
		long *ldata = (long*)data;
		buf[2] = (char)(ldata[0] >> 24);
		buf[3] = (char)(ldata[0] >> 16);
		buf[4] = (char)(ldata[0] >> 8);
		buf[5] = (char)ldata[0];
		buf[6] = (char)(ldata[1] >> 24);
		buf[7] = (char)(ldata[1] >> 16);
		buf[8] = (char)(ldata[1] >> 8);
		buf[9] = (char)ldata[1];
		buf[10] = (char)(ldata[2] >> 24);
		buf[11] = (char)(ldata[2] >> 16);
		buf[12] = (char)(ldata[2] >> 8);
		buf[13] = (char)ldata[2];
		buf[14] = (char)(ldata[3] >> 24);
		buf[15] = (char)(ldata[3] >> 16);
		buf[16] = (char)(ldata[3] >> 8);
		buf[17] = (char)ldata[3];
		length = 18;
	} else if (packet_type == PACKET_TYPE_TAP) {
		buf[2] = ((char*)data)[0];
		buf[3] = ((char*)data)[1];
		length = 4;
	} else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
		buf[2] = ((char*)data)[0];
		length = 3;
	} else if (packet_type == PACKET_TYPE_PEDO) {
		long *ldata = (long*)data;
		buf[2] = (char)(ldata[0] >> 24);
		buf[3] = (char)(ldata[0] >> 16);
		buf[4] = (char)(ldata[0] >> 8);
		buf[5] = (char)ldata[0];
		buf[6] = (char)(ldata[1] >> 24);
		buf[7] = (char)(ldata[1] >> 16);
		buf[8] = (char)(ldata[1] >> 8);
		buf[9] = (char)ldata[1];
		length = 10;
	} else if (packet_type == PACKET_TYPE_MISC) {
		buf[2] = ((char*)data)[0];
		buf[3] = ((char*)data)[1];
		buf[4] = ((char*)data)[2];
		buf[5] = ((char*)data)[3];
		length = 6;
	}
	HAL_UART_Transmit(&hlpuart1, (uint8_t *) buf, length, 100);
}
static inline unsigned short inv_row_2_scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
		b = 0;
	else if (row[0] < 0)
		b = 4;
	else if (row[1] > 0)
		b = 1;
	else if (row[1] < 0)
		b = 5;
	else if (row[2] > 0)
		b = 2;
	else if (row[2] < 0)
		b = 6;
	else
		b = 7;      // error
	return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
		const signed char *mtx)
{
	unsigned short scalar;

	/*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
	 */

	scalar = inv_row_2_scale(mtx);
	scalar |= inv_row_2_scale(mtx + 3) << 3;
	scalar |= inv_row_2_scale(mtx + 6) << 6;


	return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
	unsigned char mask = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON)
		mask |= INV_XYZ_GYRO;
	/* If you need a power transition, this function should be called with a
	 * mask of the sensors still enabled. The driver turns off any sensors
	 * excluded from this mask.
	 */
	mpu_set_sensors(mask);
	if (!hal.dmp_on)
		mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count)
{
	char data[2];
	data[0] = (char)direction;
	data[1] = (char)count;
	send_packet(PACKET_TYPE_TAP, data);
}

static void android_orient_cb(unsigned char orientation)
{
	send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
}

static inline void run_self_test(void)
{
	int result;
	char test_packet[4] = {0};
	long gyro[3], accel[3];
	unsigned char i = 0;

#if defined (MPU6500) || defined (MPU9250)
	result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined MPU6050 || defined MPU9150
	result = mpu_run_self_test(gyro, accel);
#endif
	if (result == 0x7) {
		/* Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		for(i = 0; i<3; i++) {
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 2048.f; //convert to +-16G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel);
#endif

	}

	/* Report results. */
	test_packet[0] = 't';
	test_packet[1] = result;
	send_packet(PACKET_TYPE_MISC, test_packet);
}


/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	unsigned long timestamp;
	struct int_param_s int_param;
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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
	int_param.cb = gyro_data_ready_cb;
	mpu_init(&int_param);
	setup_gyro();
	mpu_set_bypass(1);
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	/* Push both gyro and accel data into the FIFO. */
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
	/* Initialize HAL state variables. */
	memset(&hal, 0, sizeof(hal));
	hal.sensors = ACCEL_ON | GYRO_ON;
	hal.report = PRINT_QUAT;

	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	dmp_register_tap_cb(tap_cb);
	dmp_register_android_orient_cb(android_orient_cb);

	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
			DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
			DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	mpu_set_dmp_state(1);
	hal.dmp_on = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		unsigned long sensor_timestamp;

		//handle_input();
		get_ms(&timestamp);

		if (hal.motion_int_mode) {
			/* Enable motion interrupt. */
			mpu_lp_motion_interrupt(500, 1, 5);
			hal.new_gyro = 0;
			/* Wait for the MPU interrupt. */
			if(!hal.new_gyro)
				continue;
			/* Restore the previous sensor configuration. */
			mpu_lp_motion_interrupt(0, 0, 0);
			hal.motion_int_mode = 0;
		}


		if (hal.new_gyro && hal.dmp_on) {
			short gyro[3], accel[3], sensors;
			unsigned char more;
			long quat[4];
			/* This function gets new data from the FIFO when the DMP is in
			 * use. The FIFO can contain any combination of gyro, accel,
			 * quaternion, and gesture data. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
			 * the FIFO isn't being filled with accel data.
			 * The driver parses the gesture data to determine if a gesture
			 * event has occurred; on an event, the application will be notified
			 * via a callback (assuming that a callback function was properly
			 * registered). The more parameter is non-zero if there are
			 * leftover packets in the FIFO.
			 */
			dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);

			if (!more) hal.new_gyro = 0;
			/* Gyro and accel data are written to the FIFO by the DMP in chip
			 * frame and hardware units. This behavior is convenient because it
			 * keeps the gyro and accel outputs of dmp_read_fifo and
			 * mpu_read_fifo consistent.
			 */
			if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO){
				send_packet(PACKET_TYPE_GYRO, gyro);
			}
			if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL) send_packet(PACKET_TYPE_ACCEL, accel);
			/* Unlike gyro and accel, quaternions are written to the FIFO in
			 * the body frame, q30. The orientation is set by the scalar passed
			 * to dmp_set_orientation during initialization.
			 */
			if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT) send_packet(PACKET_TYPE_QUAT, quat);
			//HAL_UART_Transmit(&hlpuart1, (uint8_t *) quat, sizeof(quat), 100);
			send_packet(PACKET_TYPE_ACCEL, accel);
			char *n = "\r\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) n, sizeof(n), 100);
			send_packet(PACKET_TYPE_GYRO, gyro);
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) n, sizeof(n), 100);
			 send_packet(PACKET_TYPE_QUAT, quat);


		} else if (hal.new_gyro) {
			short gyro[3], accel[3];
			unsigned char sensors, more;
			/* This function gets new data from the FIFO. The FIFO can contain
			 * gyro, accel, both, or neither. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
			 * being filled with accel data. The more parameter is non-zero if
			 * there are leftover packets in the FIFO.
			 */
			mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
			//HAL_UART_Transmit(&hlpuart1, (uint8_t *)accel, sizeof(accel), 100);
			if (!more)
				hal.new_gyro = 0;
			if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
				send_packet(PACKET_TYPE_GYRO, gyro);
			if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
				send_packet(PACKET_TYPE_ACCEL, accel);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
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
  hi2c1.Init.Timing = 0x00602173;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : int_Pin */
  GPIO_InitStruct.Pin = int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(int_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	hal.new_gyro = 1;
	char msg[20] = "\r\nBlabla";
	HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, 8, 100);
}

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
