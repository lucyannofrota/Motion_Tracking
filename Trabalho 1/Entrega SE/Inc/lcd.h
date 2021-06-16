/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 *      Adapted by: Milena Mori (2016193815) e Lucyanno Frota (2016116214)
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32l4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "main.h"

#ifndef _STDLIB_H_
#include <stdlib.h>
#endif

#ifndef CMSIS_OS_H_
#include "cmsis_os.h"
#endif

/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font
#define SETCGRAM_ADDR 0x040
#define SET_DDRAM_ADDR 0x80				// Set DDRAM address

#define _LCD_RS_PORT LCD_RS_GPIO_Port
#define _LCD_RS_PIN LCD_RS_Pin
#define _LCD_RW_PORT LCD_RW_GPIO_Port
#define _LCD_RW_PIN LCD_RW_Pin
#define _LCD_EN_PORT LCD_EN_GPIO_Port
#define _LCD_EN_PIN LCD_EN_Pin
#define _LCD_DB4_PORT LCD_DB4_GPIO_Port
#define _LCD_DB4_PIN LCD_DB4_Pin
#define _LCD_DB5_PORT LCD_DB5_GPIO_Port
#define _LCD_DB5_PIN LCD_DB5_Pin
#define _LCD_DB6_PORT LCD_DB6_GPIO_Port
#define _LCD_DB6_PIN LCD_DB6_Pin
#define _LCD_DB7_PORT LCD_DB7_GPIO_Port
#define _LCD_DB7_PIN LCD_DB7_Pin



#define LCD_EN_Delay 1

void LCD_DATA(unsigned char Data);
void LCD_CMD(unsigned char CMD);
void LCD_Init();
void LCD_Write_Char(char Data);
void LCD_Write_String(char *str);
void LCD_Clear();
void LCD_Set_Cursor(unsigned char r, unsigned char c);
void LCD_SR();
void LCD_SL();
void LCD_Write(char *l1, char *l2);
UART_HandleTypeDef hlpuart1;


enum LCDDispState{
	LCDidle = 0,
	LCDhome = 1,
	LCDvending = 2,
	LCDdispensing = 3,
};

#endif /* LCD_H_ */
