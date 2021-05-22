/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"
#include "main.h"
uint8_t * pointer;
uint8_t   msgStr[64];

extern UART_HandleTypeDef hlpuart1;


void reset(){
	msgStr[0] = 0x3A;
	msgStr[1] = 0x29;
	pointer = &msgStr[2];
}

void addChar(char data){
	if(available()){
		memcpy(pointer, &data, sizeof(data));
		pointer += sizeof(data);
	}
}

void addInt(int16_t data){
	if(available()){
		memcpy(pointer, &data, sizeof(data));
		pointer += sizeof(data);
	}
}

void addStr(uint8_t *data,uint8_t len){
	if(available()){
		memcpy(pointer, data, len+1);
		pointer += len+1;
	}
}

uint8_t length(){
	return pointer - &msgStr[0];
}

void writeBytes(){
	HAL_UART_Transmit(&hlpuart1, msgStr, length(), 100);
}

uint8_t available(){
	return &msgStr[64 - 1] - pointer + 1;
}

void setTemp(int16_t num){
	reset();
	uint8_t b[] = "batata";
	addStr(b,6);
	addInt(num);
	addChar('a');
	writeBytes();
}
