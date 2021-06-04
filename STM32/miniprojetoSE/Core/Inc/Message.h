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
#include "macros.h"

void reset();
void addChar(char data);
void addInt(int16_t data);
void addStr(uint8_t *data,uint8_t len);
uint8_t available(void);

void writeBytes();
void setTemp();
void sendPose(struct angles_t angle, struct accel_t accel);
uint8_t length();
#endif /* SRC_MESSAGE_H_ */
