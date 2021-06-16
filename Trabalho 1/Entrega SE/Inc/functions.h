/*
 * functions.c
 *
 *  Created on: Apr 19, 2021
 *      Author: Milena Mori (2016193815) e Lucyanno Frota (2016116214)
 */

#ifndef INC_FUNCTIONS_C_
#define INC_FUNCTIONS_C_

#ifndef _STDIO_H_
#include <stdio.h>
#endif
#ifndef CMSIS_OS_H_
#include "cmsis_os.h"
#endif
#ifndef __MAIN_H
#include "main.h"
#endif

enum CType{
	coin = 0,
	product = 1,
	unknown = -1,
};

enum SerialDisp{
	home = 0,
	vending = 1,
	dispensing = 2,
};

typedef enum{
    INIT,
    WAIT,
    ADD,
    SEL,
    DISP,
	BLINK
} state_t;

struct Command {
	enum CType type;
	int16_t value1;
	int16_t value2;
};

void popString(char *, char *, uint8_t);

struct Command stringToCommand(char *);

uint16_t readBuffer(enum CType *t,osMessageQueueId_t *CommandQueuHandle);

void blinkRed(uint16_t);
void blinkGreen(uint16_t);
void blinkBlue(uint16_t);
void dispLEDs(uint16_t);
void blinkAll(uint16_t);
#endif /* INC_FUNCTIONS_C_ */
