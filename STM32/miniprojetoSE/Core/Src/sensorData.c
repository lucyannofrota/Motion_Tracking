/*
 * sensorData.c
 *
 *  Created on: Jun 5, 2021
 *      Author: lucya
 */

#include "sensorData.h"
#include "macros.h"

void AccelpAssign(struct accel_t *prim, struct accel_t *sec){
	prim->accel_x+=sec->accel_x;
	prim->accel_y+=sec->accel_y;
	prim->accel_z+=sec->accel_z;
}

void AccelsRight(struct accel_t *prim, int N){
	prim->accel_x>>=N;
	prim->accel_y>>=N;
	prim->accel_z>>=N;
}

void AngpAssign(struct angles_t *prim, struct angles_t *sec){
	prim->pitch+=sec->pitch;
	prim->roll+=sec->roll;
	prim->yaw+=sec->yaw;
}

void AngsRight(struct angles_t *prim, int N){
	prim->pitch>>=N;
	prim->roll>>=N;
	prim->yaw>>=N;
}

void filter_mpuReadings(struct sensor_t *sens,struct readings_t *readings ,struct readings_t *temp){
	AccelpAssign(&readings->acc,&temp->acc);
	AngpAssign(&readings->ang,&temp->ang);
	readings->count++;
	if(readings->count >= FILTER_MPU_NSAMPLES){
		AccelsRight(&readings->acc,FILTER_MPU_DIVIDER);
		AngsRight(&readings->ang,FILTER_MPU_DIVIDER);
		readings->count=0;


		sens->acc = readings->acc;
		sens->ang = readings->ang;

		readings->acc.accel_x = 0;
		readings->acc.accel_y = 0;
		readings->acc.accel_z = 0;
		readings->ang.pitch = 0;
		readings->ang.roll = 0;
		readings->ang.yaw = 0;
	}
//	readings->acc.accel_x = readings->acc.accel_x + temp->acc;
}


struct sensor_t MPU1 = {{0,0,0},{0,0,0}};
