/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"



void reset(){
	msgStr[0] = 0x06;
	msgStr[1] = 0x85;
	pointer = &msgStr[2];
}

void addChar(char data){
	memcpy(pointer, &data, sizeof(data));
	pointer += sizeof(data);
}

void addInt(uint16_t data){
	memcpy(pointer, &data, sizeof(data));
	pointer += sizeof(data);
}

void addStr(uint8_t *data,uint8_t len){
	memcpy(pointer, data, len);
	pointer += sizeof(data[len]);
}
