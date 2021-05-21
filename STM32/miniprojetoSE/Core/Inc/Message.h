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


uint8_t * pointer;
uint8_t msgStr[64];

void reset(void);
void addChar(char);
void addInt(uint16_t);
void addStr(uint8_t *,uint8_t);



#endif /* SRC_MESSAGE_H_ */
