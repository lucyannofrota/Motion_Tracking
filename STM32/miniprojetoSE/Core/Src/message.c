/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"
#include "main.h"
#include <math.h>
uint8_t * pointer;
uint8_t   msgStr[64];
extern uint16_t counter;

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

void addShort(int8_t data){
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
	HAL_UART_Transmit(&hlpuart1, msgStr, length(), 100);
}

uint8_t available(){
	return &msgStr[64 - 1] - pointer + 1;
}

void sendPose(struct angles_t angle, struct accel_t accel){
	reset();
	addChar('P');
	addInt(accel.accel_x);
	addInt(accel.accel_y);
	addInt(accel.accel_z);
	addInt(angle.roll);
	addInt(angle.pitch);
	addInt(angle.yaw);
	addChar('}');
	addChar('\n');
	writeBytes();
}

void sendSensor(struct angles_t angle, struct accel_t accel){
	reset();
	addChar('S');
	addShort(1);
	addInt(accel.accel_x);
	addInt(accel.accel_y);
	addInt(accel.accel_z);
	addInt(round(1.8*angle.roll/M_PI));
	addInt(round(1.8*angle.pitch/M_PI));
	addInt(round(1.8*angle.yaw/M_PI));
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
