/*
 * sensorData.h
 *
 *  Created on: Jun 5, 2021
 *      Author: lucya
 */

#ifndef INC_SENSORDATA_H_
#define INC_SENSORDATA_H_

//#include "stlib.h"

struct angles_t{
	int roll;
	int pitch;
	int yaw;
};

struct accel_t{
	int accel_x;
	int accel_y;
	int accel_z;
};

struct sensor_t{
	struct angles_t ang;
	struct accel_t acc;
	int count;
};

/*struct sensor_t{
	struct angles_t ang;
	struct accel_t acc;
};*/

//Delete
//struct angles_t angle = {0,0,0};
//struct accel_t accel = {0,0,0};
//Delete

void AccelpAssign(struct accel_t *prim, struct accel_t *sec);

void AccelsRight(struct accel_t *prim, int N);

void AngpAssign(struct angles_t *prim, struct angles_t *sec);

void AngsRight(struct angles_t *prim, int N);

void filter_mpuReadings(struct sensor_t *sens,struct sensor_t *readings ,struct sensor_t *temp);

void removeGravity(struct sensor_t *sens);


#endif /* INC_SENSORDATA_H_ */
