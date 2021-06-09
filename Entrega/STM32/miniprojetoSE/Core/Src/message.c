/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"
#include "main.h"
#include <math.h>
#include <mySensor.h>
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
	HAL_UART_Transmit(&hlpuart1, msgStr, length(), 10);
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

void sendSensor(struct sensor_t *sens){
//	printf("Ping\n");
//	if(sens->count != 0) return;
//	printf("A(%i,%i,%i)\nG(%i,%i,%i)\n",1,2,3,4,5,6);
//	printf("A(%i,%i,%i)\nG(%i,%i,%i)\n",sens.acc.accel_x,sens.acc.accel_y,sens.acc.accel_z,sens.ang.roll*1000,sens.ang.pitch*1000,sens.ang.yaw*1000);
	reset();
	addChar('S');
	addShort(sens->idx);
	addInt(sens->filtered.accelerations.accel_x);
	addInt(sens->filtered.accelerations.accel_y);
	addInt(sens->filtered.accelerations.accel_z);
	addInt(sens->filtered.angles.roll);
	addInt(sens->filtered.angles.pitch);
	addInt(sens->filtered.angles.yaw);
	addChar('}');
	addChar('\n');
	writeBytes();
//	printf("PingEnd\n");
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
