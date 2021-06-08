/*
 * sensorData.h
 *
 *  Created on: Jun 5, 2021
 *      Author: lucya
 */

#ifndef INC_MYSENSOR_H_
#define INC_MYSENSOR_H_

//#include "stlib.h"

#include "../../Drivers/Sensor/inv_mpu.h"

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

struct s_data{
	struct angles_t angles;
	struct accel_t accelerations;
};

struct sensor_t{
	int idx;
	struct gyro_state_s st;
	struct s_data raw;
	int count;
	struct s_data filtered;
//	struct angles_t ang;
//	struct accel_t acc;
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

void filter_mpuReadings(struct sensor_t *sens,struct accel_t *acc ,struct angles_t *ang);

//void removeGravity(struct sensor_t *sens);

void removeGravity(struct accel_t *ac,short *accel,float *q);





//mpu





//struct IMU{
//	int idx;
//	struct gyro_state_s st;
//};

int MPU_init(struct sensor_t *sens,int AD0);
struct angles_t toEuler(float qw,float qx, float qy, float qz);
void run_self_test(struct gyro_state_s *st);
void gyro_data_ready_cb(void);
void readSensorData(struct sensor_t *sens);

#endif /* INC_MYSENSOR_H_ */
