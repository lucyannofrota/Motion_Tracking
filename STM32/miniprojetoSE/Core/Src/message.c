/*
 * Message.cpp
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#include "Message.h"



void reset(uint8_t * msgStr, uint8_t* pointer){
	msgStr[0] = 0x06;
	msgStr[1] = 0x85;
	pointer = &msgStr[2];
}

void addChar(char data, uint8_t* pointer){
	memcpy(pointer, &data, sizeof(data));
	pointer += sizeof(data);
}

void addInt(uint16_t data, uint8_t*pointer){
	memcpy(pointer, &data, sizeof(data));
	pointer += sizeof(data);
}

void addStr(uint8_t *data, uint8_t len, uint8_t* pointer){
	pointer = data;
	pointer += sizeof(uint8_t)*len;
}
