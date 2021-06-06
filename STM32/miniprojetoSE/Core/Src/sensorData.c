/*
 * sensorData.c
 *
 *  Created on: Jun 5, 2021
 *      Author: lucya
 */

#include "sensorData.h"
#include "macros.h"
#include "math.h"
#include "stdio.h"
#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

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

void filter_mpuReadings(struct sensor_t *sens,struct sensor_t *readings ,struct sensor_t *temp){
	AccelpAssign(&readings->acc,&temp->acc);
	AngpAssign(&readings->ang,&temp->ang);
	readings->count++;
	if(readings->count >= FILTER_MPU_NSAMPLES){
		AccelsRight(&readings->acc,FILTER_MPU_DIVIDER);
		AngsRight(&readings->ang,FILTER_MPU_DIVIDER);
		readings->count=0;

		sens->count = 1;
		sens->acc = readings->acc;
		sens->ang = readings->ang;

		readings->acc.accel_x = 0;
		readings->acc.accel_y = 0;
		readings->acc.accel_z = 0;
		readings->ang.pitch = 0;
		readings->ang.roll = 0;
		readings->ang.yaw = 0;

		sens->count = 0;
	}
//	readings->acc.accel_x = readings->acc.accel_x + temp->acc;
}

/*
void removeGravity(struct sensor_t *sens){
	float tempX = sens->acc.accel_x/65536.f, tempY = sens->acc.accel_y/65536.f, tempZ = sens->acc.accel_z/65536.f;
	float x = tempX, y = tempY, z = 0;
	tempX -= x*cosd(sens->ang.yaw) - y*sind(sens->ang.yaw);
	tempY -= x*sind(sens->ang.yaw) - y*cosd(sens->ang.yaw);

	x = tempX; z = tempZ;
	tempX -= x*cosd(sens->ang.pitch) + z*sind(sens->ang.pitch);
	tempZ -= -x*sind(sens->ang.pitch) - z*cosd(sens->ang.pitch);

	y = tempY; z = tempZ;
	tempY -= y*cosd(sens->ang.roll) - z*sind(sens->ang.roll);
	tempZ -= y*sind(sens->ang.roll) + z*cosd(sens->ang.roll);

	sens->acc.accel_x = tempX*1000; sens->acc.accel_y = tempY*1000; sens->acc.accel_z = tempZ*1000;
}
*/

void removeGravity(struct accel_t *ac,short *accel,float *q){
	float g[4] = {0,0,0,0};

	g[0] = 2*(q[1]*q[3]-q[0]*q[2]);
	g[1] = 2*(q[0]*q[1]+q[2]*q[3]);
	g[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	float const cot = 4.37;
	float ax = (accel[0]/65536.f)*cot,ay = (accel[1]/65536.f)*cot,az = (accel[2]/65536.f)*cot;
//	printf("A (%f,%f,%f)\n",ax,ay,az);
//	printf("g (%f,%f,%f)\n",g[0],g[1],g[2]);
	ac->accel_x = (ax-g[0])*1000; ac->accel_y = (ay-g[1])*1000; ac->accel_z = (az-g[2])*1000;
//	printf("Axe (%i,%i,%i)\n",ac->accel_x,ac->accel_y,ac->accel_z);
}

struct sensor_t MPU1 = {{0,0,0},{0,0,0},0};
