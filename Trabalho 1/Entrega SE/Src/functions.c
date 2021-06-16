/*
 * functions.c
 *
 *  Created on: Apr 19, 2021
 *      Author: Milena Mori (2016193815) e Lucyanno Frota (2016116214)
 */

#ifndef INC_FUNCTIONS_C_
#include "functions.h"
#endif
#ifndef _STRING_H_
#include <string.h>
#endif
#ifndef _STDLIB_H_
#include <stdlib.h>
#endif
#ifndef CMSIS_OS_H_
#include "cmsis_os.h"
#endif

void popString(char *str1, char *str2, uint8_t t){
	uint8_t t_ = 0;
	uint16_t len = strlen(str1);
    for(uint8_t i = 0; i<len; i++){
        if(t_<t){
            str2[t_] = str1[i];
            t_++;
        }
        str1[i] = str1[t+i];
    }
}

struct Command stringToCommand(char *str){
	struct Command com;

	uint8_t p = 0;
	char c[] = "";

	char paramBuff[5];
	uint8_t buffC = 0;

	int8_t type = -1;
	int16_t param1 = -1;
	int16_t param2 = -1;

	int len1 = strlen(str);

	for(int i = 0; (str[0]!='\0') && (i < len1); i++){
		popString(str,c,1);
		switch(c[0]){
			case '\0':
				com.type = unknown;
				com.value1 = -1;
				com.value2 = -1;
				return com;
				break;
			case '[':
				p = 1;
				memset(paramBuff, 0, 5);
				break;
			case ',':
				switch(p){
					case 1:
						buffC = 0;
						p++;
						break;
					case 2:
						param1 = atoi(paramBuff);
						memset(paramBuff, 0, 5);
						buffC = 0;
						p++;
						break;
					default:
						break;
				}
				break;
			case ']':
				if((type == -1) || (p < 2)){
					com.type = unknown;
					com.value1 = -1;
					com.value2 = -1;
					return com;
				}
				else{
					if(p == 2){
						param1 = atoi(paramBuff);
						memset(paramBuff, 0, 5);
						buffC = 0;
					}
					else{
						param2 = atoi(paramBuff);
						memset(paramBuff, 0, 5);
						buffC = 0;
					}
				}

				//command
				com.type = type;
				com.value1 = param1;
				com.value2 = param2;
				return com;
				break;
			default:
				switch(p){
					case 1:
						if(!(c[0] == 'P' || c[0] == 'p') && !(c[0] == 'C' || c[0] == 'c')){
							com.type = unknown;
							com.value1 = -1;
							com.value2 = -1;
							return com;
						}
						else{
							if(c[0] == 'P' || c[0] == 'p') type = 1;
							else type = 0;
						}
						break;
					case 2:
						if(c[0] >= '0' && c[0] <= '9'){
							paramBuff[buffC] = c[0];
							buffC++;
						}
						else{
							com.type = unknown;
							com.value1 = -1;
							com.value2 = -1;
							return com;
						}
						// param1 = atoi(paramBuff);
						break;
					case 3:
						if(c[0] >= '0' && c[0] <= '9'){
							paramBuff[buffC] = c[0];
							buffC++;
						}
						else{
							com.type = unknown;
							com.value1 = -1;
							com.value2 = -1;
							return com;
						}
						break;
					default:
						break;
				}
				break;
		}
	}



	com.type = unknown;
	com.value1 = -1;
	com.value2 = -1;
	return com;
}

uint16_t readBuffer(enum CType *t,osMessageQueueId_t *CommandQueuHandle){ // Convert to read q
	struct Command info;
	info.type = unknown;
	info.value1 = -1;
	info.value2 = -1;
	*t = info.type;
	if (osMessageQueueGet(CommandQueuHandle, &info, 0, 100) == osOK)
	{
		*t = info.type;
		switch (info.type)
		{
		case coin:
			return info.value1;
			break;
		case product:
			return info.value1*10 + info.value2;
			break;
		default:
			return 0;
			break;
		}
	}

	return 0;
}
//16x2
void blinkRed(uint16_t delay){
	HAL_GPIO_WritePin(red_Led_GPIO_Port, red_Led_Pin, GPIO_PIN_SET);
	osDelay(delay);
	HAL_GPIO_WritePin(red_Led_GPIO_Port, red_Led_Pin, GPIO_PIN_RESET);
}

void blinkGreen(uint16_t delay){
	HAL_GPIO_WritePin(green_Led_GPIO_Port, green_Led_Pin, GPIO_PIN_SET);
	osDelay(delay);
	HAL_GPIO_WritePin(green_Led_GPIO_Port, green_Led_Pin, GPIO_PIN_RESET);
}

void blinkBlue(uint16_t delay){
	HAL_GPIO_WritePin(blue_Led_GPIO_Port, blue_Led_Pin, GPIO_PIN_SET);
	osDelay(delay);
	HAL_GPIO_WritePin(blue_Led_GPIO_Port, blue_Led_Pin, GPIO_PIN_RESET);
}

void blinkAll(uint16_t delay){
	HAL_GPIO_WritePin(red_Led_GPIO_Port, red_Led_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(green_Led_GPIO_Port, green_Led_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(blue_Led_GPIO_Port, blue_Led_Pin, GPIO_PIN_SET);
	osDelay(delay);
	HAL_GPIO_WritePin(blue_Led_GPIO_Port, blue_Led_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(green_Led_GPIO_Port, green_Led_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(red_Led_GPIO_Port, red_Led_Pin, GPIO_PIN_RESET);
}

void dispLEDs(uint16_t delay){
	blinkRed(delay);
	blinkBlue(delay);
	blinkGreen(delay);
}
