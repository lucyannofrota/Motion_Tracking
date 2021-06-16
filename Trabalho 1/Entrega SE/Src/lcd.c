/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 *      Adapted by: Milena Mori (2016193815) e Lucyanno Frota (2016116214)
 */

#include "lcd.h"

void LCD_DATA(unsigned char Data){
	  if(Data & 1)
		  HAL_GPIO_WritePin(_LCD_DB4_PORT, _LCD_DB4_PIN, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(_LCD_DB4_PORT, _LCD_DB4_PIN, GPIO_PIN_RESET);
	  if(Data & 2)
		  HAL_GPIO_WritePin(_LCD_DB5_PORT, _LCD_DB5_PIN, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(_LCD_DB5_PORT, _LCD_DB5_PIN, GPIO_PIN_RESET);
	  if(Data & 4)
		  HAL_GPIO_WritePin(_LCD_DB6_PORT, _LCD_DB6_PIN, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(_LCD_DB6_PORT, _LCD_DB6_PIN, GPIO_PIN_RESET);
	  if(Data & 8)
		  HAL_GPIO_WritePin(_LCD_DB7_PORT, _LCD_DB7_PIN, GPIO_PIN_SET);
	  else
		  HAL_GPIO_WritePin(_LCD_DB7_PORT, _LCD_DB7_PIN, GPIO_PIN_RESET);
}

void LCD_CMD(unsigned char CMD){
  // Select Command Register
	HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN, GPIO_PIN_RESET);
  // Move The Command Data To LCD
	LCD_DATA(CMD);
  // Send The EN Clock Signal
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_SET);
	HAL_Delay(LCD_EN_Delay);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_RESET);
	HAL_Delay(LCD_EN_Delay);
}

void LCD_Init()
{
	HAL_Delay(500);
  // IO Pin Configurations
	HAL_GPIO_WritePin(_LCD_DB4_PORT, _LCD_DB4_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_DB5_PORT, _LCD_DB5_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_DB6_PORT, _LCD_DB6_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_DB7_PORT, _LCD_DB7_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(_LCD_RW_PORT, _LCD_RW_PIN, GPIO_PIN_RESET);


  // The Init. Procedure
	LCD_DATA(0);
	HAL_Delay(LCD_EN_Delay);

	LCD_CMD(0x03);
	HAL_Delay(10); //37us
	LCD_CMD(0x02);
	LCD_CMD(0x08);
	HAL_Delay(10);//37us
	LCD_CMD(0x02);
	LCD_CMD(FUNCTION_SET | OPT_N);
	HAL_Delay(10);//37us
	LCD_CMD(0x00);
	LCD_CMD(DISPLAY_ON_OFF_CONTROL | OPT_D);
	HAL_Delay(10);//37us
	LCD_CMD(0x00);
	LCD_CMD(CLEAR_DISPLAY);
	HAL_Delay(20); //1.52ms
	LCD_CMD(0x00);
	LCD_CMD(ENTRY_MODE_SET | OPT_INC);
	HAL_Delay(500);

	LCD_Clear();

	LCD_Set_Cursor(1, 1);
	LCD_Write_String("Initializing");
	LCD_Set_Cursor(2, 1);
	LCD_Write_String("Initializing");
	int i = 0;
	for(i = 0; i<5;i++){
		LCD_SR();  HAL_Delay(100);
	}
	for(i = 0; i<5;i++){
		LCD_SL();  HAL_Delay(100);
	}
	LCD_Clear();
	HAL_Delay(500);
}

void LCD_Write_Char(char Data)
{
	char Low4,High4;
	Low4 = Data & 0x0F;
	High4 = Data & 0xF0;

	HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(_LCD_RW_PORT, _LCD_RW_PIN, GPIO_PIN_RESET);

	LCD_DATA(High4>>4);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_SET);
	HAL_Delay(LCD_EN_Delay);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_RESET);
	HAL_Delay(LCD_EN_Delay);

	LCD_DATA(Low4);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_SET);
	HAL_Delay(LCD_EN_Delay);
	HAL_GPIO_WritePin(_LCD_EN_PORT, _LCD_EN_PIN, GPIO_PIN_RESET);
	HAL_Delay(LCD_EN_Delay);
}
void LCD_Write_String(char *str)
{
  int i;
  for(i=0;str[i]!='\0';i++){
	LCD_Write_Char(str[i]);
  }
  HAL_Delay(1);
}
void LCD_Write(char *l1, char *l2){
//	HAL_UART_Transmit(&hlpuart1, l1, strlen(l1), 10);
//	HAL_UART_Transmit(&hlpuart1, l2, strlen(l2), 10);
//	LCD_CMD(0x00);
//	LCD_CMD(CLEAR_DISPLAY);
//	HAL_Delay(10); //1.52ms
//	LCD_CMD(0x00);
//	LCD_CMD(ENTRY_MODE_SET | OPT_INC);
	LCD_Set_Cursor(1, 1);
	LCD_Write_String(l1);
	LCD_Set_Cursor(2, 1);
	LCD_Write_String(l2);
}
void LCD_Clear()
{
  LCD_CMD(0);
//  HAL_Delay(20);
  LCD_CMD(1);
  HAL_Delay(20);
}
void LCD_Set_Cursor(unsigned char r, unsigned char c)
{
  unsigned char Temp,Low4,High4;
  if(r == 1)
  {
    Temp = 0x80 + c - 1; //0x80 is used to move the cursor
    High4 = Temp >> 4;
    Low4 = Temp & 0x0F;
    LCD_CMD(High4);
    LCD_CMD(Low4);
  }
  if(r == 2)
  {
    Temp = 0xC0 + c - 1;
    High4 = Temp >> 4;
    Low4 = Temp & 0x0F;
    LCD_CMD(High4);
    LCD_CMD(Low4);
  }
  HAL_Delay(1);
}
void LCD_SL()
{
    LCD_CMD(0x01);
    LCD_CMD(0x08);
}
void LCD_SR()
{
    LCD_CMD(0x01);
    LCD_CMD(0x0C);
}
