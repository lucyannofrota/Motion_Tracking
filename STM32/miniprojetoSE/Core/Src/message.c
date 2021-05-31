/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"
#include "main.h"
#include "macros.h"
uint8_t * pointer;
uint8_t   msgStr[64];

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;

void reset(){
	//msgStr[0] = 0x06;
	msgStr[0] = 0x7B;
	pointer = &msgStr[1];
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
	transmit_l(msgStr, length());
}

uint8_t available(){
	return &msgStr[64 - 1] - pointer + 1;
}

void sendPose(){
	reset();
	addChar('P');
	/*addInt(0x4D);
	addInt(0x4E);
	addInt(0x4F);
	addInt(0x50);
	addInt(0x51);
	addInt(0x52);*/
	addInt(32767);
	addInt(126);
	addInt(0b011);
	addInt(0b011 << 8);
	addInt(253);
	addInt(-32768);
	addChar('}');
	addChar('\n');
	writeBytes();
}

void setTemp(){
	reset();
	//addInt(9);
	uint8_t b[] = "batata";
	addStr(b,6);
	addInt(0x21);
	addChar('}');
	writeBytes();
}
