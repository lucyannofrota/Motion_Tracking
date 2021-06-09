/*
 * macros.h
 *
 *  Created on: May 31, 2021
 *      Author: kmori
 */

#ifndef INC_MACROS_H_
#define INC_MACROS_H_
#include "stm32l4xx.h"
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;

#define BLUETOOTH 1

#if BLUETOOTH == 1
#define transmit(x) HAL_UART_Transmit(&huart4, (uint8_t *)x, sizeof(x), 10);
#define transmit_l(x,len) HAL_UART_Transmit(&huart4, (uint8_t *)x, len, 10);
#else
#define transmit(...) 	  printf(__VA_ARGS__);
#define transmit_l(x,len) HAL_UART_Transmit(&hlpuart1, (uint8_t *)x, len, 10);
#endif
struct platform_data_s {
    signed char orientation[9];
};


#define MPU6050 1
//#define MPU9150 1

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)


#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (20)
#undef  FLASH_SIZE
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};
struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	volatile unsigned char new_gyro;
	unsigned char motion_int_mode;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned long next_temp_ms;
	unsigned long next_compass_ms;
	unsigned int report;
	unsigned short dmp_features;
	struct rx_s rx;
};

#undef MPL_LOGI
#define MPL_LOGI(...) printf(__VA_ARGS__)
#undef MPL_LOGE
#define MPL_LOGE(...) printf(__VA_ARGS__)

#define USE_CAL_HW_REGISTERS 1


#define FILTER_MPU_DIVIDER 1 // >> 3 => /8
#define FILTER_MPU_NSAMPLES 2 // Cada amostra demora em meida 5ms. Logo, 5*8 = 40ms de taxa de amostragem


#define REMOVE_GRAVITY 1

#endif /* INC_MACROS_H_ */
