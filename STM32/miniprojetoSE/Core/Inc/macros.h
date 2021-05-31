/*
 * macros.h
 *
 *  Created on: May 31, 2021
 *      Author: kmori
 */

#ifndef INC_MACROS_H_
#define INC_MACROS_H_

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;

#define BLUETOOTH 1

#if BLUETOOTH == 1
#define transmit(x) HAL_UART_Transmit(&huart4, (uint8_t *)x, sizeof(x), 100);
#define transmit_l(x,len) HAL_UART_Transmit(&huart4, (uint8_t *)x, len, 100);
#else
#define transmit(x) HAL_UART_Transmit(&hlpuart1, (uint8_t *)x, sizeof(x), 100);
#define transmit_l(x,len) HAL_UART_Transmit(&hlpuart1, (uint8_t *)x, len, 100);
#endif
#endif /* INC_MACROS_H_ */
