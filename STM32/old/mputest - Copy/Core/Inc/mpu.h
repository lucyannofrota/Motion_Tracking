/*
 * mpu.h
 *
 *  Created on: Jun 3, 2021
 *      Author: kmori
 */

#ifndef INC_MPU_H_
#define INC_MPU_H_


int MPU_init(void);
struct angles_t toEuler(float qw,float qx, float qy, float qz);
void run_self_test(void);
void gyro_data_ready_cb(void);
void readSensorData(void);
#endif /* INC_MPU_H_ */
