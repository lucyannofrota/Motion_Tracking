/*
 * mpu.h
 *
 *  Created on: May 22, 2021
 *      Author: kmori
 */

#ifndef INC_MPU_H_
#define INC_MPU_H_


struct magnetometer{
	float mx;
	float my;
	float mz;
};
struct accelerometer{
	float ax;
	float ay;
	float az;
};
struct gyroscope{
	float gx;
	float gy;
	float gz;
};

struct MPU{
	struct magnetometer mag;
	struct accelerometer acc;
	struct gyroscope gyro;
} mpu1;
#endif /* INC_MPU_H_ */
