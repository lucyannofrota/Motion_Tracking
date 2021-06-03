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
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "macros.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
#undef MPL_LOGI
#define MPL_LOGI(...) HAL_UART_Transmit(&hlpuart1,(uint8_t *)__VA_ARGS__,sizeof(__VA_ARGS__),100)
#undef MPL_LOGE
#define MPL_LOGE(...) HAL_UART_Transmit(&hlpuart1,(uint8_t *)__VA_ARGS__,sizeof(__VA_ARGS__),100)

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t *) ptr, len, 10);
	return len;
}
static struct hal_s hal = {0};



static inline void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];


	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7) {
		MPL_LOGI("Passed!\n");
		printf("accel: %7.4f %7.4f %7.4f\n",
				accel[0]/65536.f,
				accel[1]/65536.f,
				accel[2]/65536.f);
		printf("gyro: %7.4f %7.4f %7.4f\n",
				gyro[0]/65536.f,
				gyro[1]/65536.f,
				gyro[2]/65536.f);
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

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
#else
		/* Push the calibrated data to the MPL library.
		 *
		 * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		//inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		//inv_set_gyro_bias(gyro, 3);
#endif
	}
	else {
		if (!(result & 0x1)){
			char msg[20] = "Gyro failed.\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
		}
		if (!(result & 0x2)){
			char msg[20] = "Accel failed.\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
		}
		if (!(result & 0x4)){
			char msg[20] = "Compass failed.\n";
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
		}
	}

}
void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

static void read_from_mpl(void)
{
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;
	float float_data[3] = {0};

	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
		/* Sends a quaternion packet to the PC. Since this is used by the Python
		 * test app to visually represent a 3D quaternion, it's sent each time
		 * the MPL has new data.
		 */
		eMPL_send_quat(data);

		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}

	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
#ifdef COMPASS_ENABLED
	if (hal.report & PRINT_COMPASS) {
		if (inv_get_sensor_type_compass(data, &accuracy,
				(inv_time_t*)&timestamp))
			eMPL_send_data(PACKET_DATA_COMPASS, data);
	}
#endif
if (hal.report & PRINT_EULER) {
	if (inv_get_sensor_type_euler(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_EULER, data);
}
if (hal.report & PRINT_ROT_MAT) {
	if (inv_get_sensor_type_rot_mat(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_ROT, data);
}
if (hal.report & PRINT_HEADING) {
	if (inv_get_sensor_type_heading(data, &accuracy,
			(inv_time_t*)&timestamp))
		eMPL_send_data(PACKET_DATA_HEADING, data);
}
if (hal.report & PRINT_LINEAR_ACCEL) {
	if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
		printf("Linear Accel: %7.5f %7.5f %7.5f\r\n",
				float_data[0], float_data[1], float_data[2]);
	}
}
if (hal.report & PRINT_GRAVITY_VECTOR) {
	if (inv_get_sensor_type_gravity(float_data, &accuracy,
			(inv_time_t*)&timestamp))
		printf("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
				float_data[0], float_data[1], float_data[2]);
}
if (hal.report & PRINT_PEDO) {
	unsigned long timestamp;
	get_tick_count(&timestamp);
	if (timestamp > hal.next_pedo_ms) {
		hal.next_pedo_ms = timestamp + PEDO_READ_MS;
		unsigned long step_count, walk_time;
		dmp_get_pedometer_step_count(&step_count);
		dmp_get_pedometer_walk_time(&walk_time);

		printf("Walked %ld steps over %ld milliseconds..\n", step_count,
				walk_time);
	}
}

/* Whenever the MPL detects a change in motion state, the application can
 * be notified. For this example, we use an LED to represent the current
 * motion state.
 */
msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
		INV_MSG_NO_MOTION_EVENT);
if (msg) {
	if (msg & INV_MSG_MOTION_EVENT) {
		MPL_LOGI("Motion!\n");
	} else if (msg & INV_MSG_NO_MOTION_EVENT) {
		MPL_LOGI("No motion!\n");
	}
}
}

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	unsigned char accel_fsr,  new_temp = 0;
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
	int result = mpu_init(&int_param);
	if (result !=0) {
		MPL_LOGI("Could not start MPU!\n");
	}
	result = inv_init_mpl();
	if (result !=0) {
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Could not initialize MPL.\n", 26, 100);
	}
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();

	inv_enable_fast_nomot();
	inv_enable_gyro_tc();
#ifdef COMPASS_ENABLED
	/* Compass calibration algorithms. */
	inv_enable_vector_compass_cal();
	inv_enable_magnetic_disturbance();
#endif
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) {
		while (1) {
			MPL_LOGE("Not authorized.\n");
		}
	}
	if (result) {
		MPL_LOGE("Could not start the MPL.\n");
	}
#ifdef COMPASS_ENABLED
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
	inv_enable_eMPL_outputs();

	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);

#ifdef COMPASS_ENABLED
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
	inv_set_gyro_orientation_and_scale(
	            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
	            (long)gyro_fsr<<15);
	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;
    get_tick_count(&timestamp);

	if(dmp_load_motion_driver_firmware()){
		char *msg = "Could not start DMP!\r\n";
		HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, 24, 100);
	}
	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	/* Compass reads are handled by scheduler. */
	get_tick_count(&timestamp);
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);

	if(mpu_set_dmp_state(1)){
		char *msg = "Could not enable DMP!\r\n";
		HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, 24, 100);
	}
	hal.dmp_on = 1;

	run_self_test();
	//HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		continue;
		unsigned long sensor_timestamp;
		int new_data = 0;
		get_tick_count(&timestamp);
#ifdef COMPASS_ENABLED
		/* We're not using a data ready interrupt for the compass, so we'll
		 * make our compass reads timer-based instead.
		 */
		if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
				hal.new_gyro && (hal.sensors & COMPASS_ON)) {
			hal.next_compass_ms = timestamp + COMPASS_READ_MS;
			new_compass = 1;
		}
#endif

/* Temperature data doesn't need to be read with every gyro sample.
 * Let's make them timer-based like the compass reads.
 */
		if (timestamp > hal.next_temp_ms) {
			hal.next_temp_ms = timestamp + TEMP_READ_MS;
			new_temp = 1;
		}
		if (!hal.sensors || !hal.new_gyro) {
			continue;
		}
		if (hal.new_gyro && hal.dmp_on) {
			short gyro[3], accel_short[3], sensors;
			unsigned char more;
			long accel[3], quat[4], temperature;
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
			dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
			//char *msg = "\r\naaa";
			//HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
			if (!more)
				hal.new_gyro = 0;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					//char *msg = "\r\naaa";
					//HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
			if (sensors & INV_WXYZ_QUAT) {
				inv_build_quat(quat, 0, sensor_timestamp);
				new_data = 1;
			}
		} else if (hal.new_gyro) {
			short gyro[3], accel_short[3];
			unsigned char sensors, more;
			long accel[3], temperature;
			/* This function gets new data from the FIFO. The FIFO can contain
			 * gyro, accel, both, or neither. The sensors parameter tells the
			 * caller which data fields were actually populated with new data.
			 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
			 * being filled with accel data. The more parameter is non-zero if
			 * there are leftover packets in the FIFO. The HAL can use this
			 * information to increase the frequency at which this function is
			 * called.
			 */
			hal.new_gyro = 0;
			mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
					&sensors, &more);
			if (more)
				hal.new_gyro = 1;
			if (sensors & INV_XYZ_GYRO) {
				/* Push the new data to the MPL. */
				//				inv_build_gyro(gyro, sensor_timestamp);
				new_data = 1;
				if (new_temp) {
					new_temp = 0;
					/* Temperature only used for gyro temp comp. */
					mpu_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
			}
			if (sensors & INV_XYZ_ACCEL) {
				accel[0] = (long)accel_short[0];
				accel[1] = (long)accel_short[1];
				accel[2] = (long)accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
			}
#ifdef COMPASS_ENABLED
			if (new_compass) {
				short compass_short[3];
				long compass[3];
				new_compass = 0;
				/* For any MPU device with an AKM on the auxiliary I2C bus, the raw
				 * magnetometer registers are copied to special gyro registers.
				 */
				if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
					compass[0] = (long)compass_short[0];
					compass[1] = (long)compass_short[1];
					compass[2] = (long)compass_short[2];
					/* NOTE: If using a third-party compass calibration library,
					 * pass in the compass data in uT * 2^16 and set the second
					 * parameter to INV_CALIBRATED | acc, where acc is the
					 * accuracy from 0 to 3.
					 */
					inv_build_compass(compass, 0, sensor_timestamp);
				}
				new_data = 1;
			}
#endif
			if (new_data) {
				inv_execute_on_data();
				/* This function reads bias-compensated sensor data and sensor
				 * fusion outputs from the MPL. The outputs are formatted as seen
				 * in eMPL_outputs.c. This function only needs to be called at the
				 * rate requested by the host.
				 */
				read_from_mpl();
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : int_Pin */
  GPIO_InitStruct.Pin = int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	gyro_data_ready_cb();
	//char *msg = "\r\naaa";
	//HAL_UART_Transmit(&hlpuart1, (uint8_t *) msg, sizeof(msg), 100);
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
