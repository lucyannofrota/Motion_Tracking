/*
 * Message.h
 *
 *  Created on: May 21, 2021
 *      Author: kmori
 */

#ifndef SRC_MESSAGE_H_
#define SRC_MESSAGE_H_

#include <stdio.h>
#include <string.h>


void reset(uint8_t * msgStr, uint8_t* pointer);
void addChar(char data, uint8_t * pointer);
void addInt(uint16_t data, uint8_t *pointer);
void addStr(uint8_t *data,uint8_t len, uint8_t* pointer);



#endif /* SRC_MESSAGE_H_ */
