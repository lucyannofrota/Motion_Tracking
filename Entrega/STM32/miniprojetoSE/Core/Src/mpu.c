///*
// * mpu.c
// *
// *  Created on: May 22, 2021
// *      Author: kmori
// */
//
//
//#include <mySensor.h>
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "math.h"
//#include "mpu.h"
//#include "Message.h"
//#include "macros.h"
//#include <stdio.h>
//
//extern struct hal_s hal;
//volatile uint32_t hal_timestamp = 0;
//unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";
////extern struct angles_t angle;
//
////struct sensor_t readingsMPU1 = {{0,0,0},{0,0,0},0};
////struct sensor_t tempReading = {{0,0,0},{0,0,0},0};
//
//
////extern struct sensor_t MPU1;
//
////extern struct readings_t readingsMPU1;
//
//int MPU_init(struct sensor_t *sens,int AD0){
//	unsigned char accel_fsr;
//	unsigned short gyro_rate, gyro_fsr;
//	struct int_param_s int_param;
//	int_param.cb = gyro_data_ready_cb;
//
//	if(AD0 == 0) sens->idx = 0;
//	else sens->idx = 1;
//
//	sens
//
//	sens->ang = {0,0,0};
//	sens->acc = {0,0,0};
//
////	struct gyro_state_s st;
//
//	if (mpu_init(&int_param,AD0,&(sens->st)) !=0){
//		MPL_LOGI("Could not start MPU!\n");
//		return -1;
//	}
//
//#ifdef COMPASS_ENABLED
//	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
//#else
//	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL,&(sens->st));
//#endif
//
//	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL,&(sens->st));
//	mpu_set_sample_rate(DEFAULT_MPU_HZ,&(sens->st));
//
//#ifdef COMPASS_ENABLED
//	/* The compass sampling rate can be less than the gyro/accel sampling rate.
//	 * Use this function for proper power management.
//	 */
//	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
//#endif
//	mpu_get_sample_rate(&gyro_rate,&(sens->st));
//	mpu_get_gyro_fsr(&gyro_fsr,&(sens->st));
//	mpu_get_accel_fsr(&accel_fsr,&(sens->st));
//#ifdef COMPASS_ENABLED
//    mpu_get_compass_fsr(&compass_fsr);
//#endif
//#ifdef COMPASS_ENABLED
//    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
//#else
//    hal.sensors = ACCEL_ON | GYRO_ON;
//#endif
//    hal.dmp_on = 0;
//    hal.report = 0;
//    hal.rx.cmd = 0;
//    hal.next_pedo_ms = 0;
//    hal.next_compass_ms = 0;
//    hal.next_temp_ms = 0;
//	if(dmp_load_motion_driver_firmware(&(sens->st))){
//		printf("Could not start DMP!\r\n");
//		return -1;
//	}
//
//	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS,&(sens->st));
//
//	hal.dmp_features = DMP_FEATURE_SEND_RAW_ACCEL |DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
//	dmp_enable_feature(hal.dmp_features,&(sens->st));
//	dmp_set_fifo_rate(DEFAULT_MPU_HZ,&(sens->st));
//
//	if(mpu_set_dmp_state(1,&(sens->st))){
//		printf("Could not enable DMP!\r\n");
//		return -1;
//	}
//	hal.dmp_on = 1;
//
//	run_self_test(&(sens->st));
//
//	return 0;
//}
//
//struct angles_t toEuler(float qw,float qx, float qy, float qz){
//	struct angles_t angle;
//	float epsilon = 0.00001;
//
//	//	roll
//	float r1 = 2 * (qw * qx + qy * qz);
//	float r2 = 1 - 2 * (qx * qx + qy * qy);
//	angle.roll = 180*atan2(r1,r2)/M_PI;
//
//	float p1 = 2 * (qw * qy - qz * qx);
//	if (fabs(p1) >= 1 - epsilon)
//		angle.pitch = 180*copysignf(M_PI / 2, p1); // use 90 degrees if out of range
//	else
//		angle.pitch = 180*asin(p1)/M_PI;
//
//	// yaw (z-axis rotation)
//	float siny_cosp = 2 * (qw * qz + qx * qy);
//	float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
//	angle.yaw = 180*atan2(siny_cosp, cosy_cosp)/M_PI;
//
//	return angle;
//
//}
//
//void run_self_test(struct gyro_state_s *st)
//{
//	int result;
//	long gyro[3], accel[3];
//
//
//	result = mpu_run_self_test(gyro, accel,st);
//	if (result == 0x7) {
//		MPL_LOGI("Passed!\n");
//		printf("accel: %7.4f %7.4f %7.4f\n",
//				accel[0]/65536.f,
//				accel[1]/65536.f,
//				accel[2]/65536.f);
//		printf("gyro: %7.4f %7.4f %7.4f\n",
//				gyro[0]/65536.f,
//				gyro[1]/65536.f,
//				gyro[2]/65536.f);
//		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/
//
//#ifdef USE_CAL_HW_REGISTERS
//		/*
//		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
//		 * instead of pushing the cal data to the MPL software library
//		 */
//		unsigned char i = 0;
//
//		for(i = 0; i<3; i++) {
//			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
//			accel[i] *= 2048.f; //convert to +-16G
//			accel[i] = accel[i] >> 16;
//			gyro[i] = (long)(gyro[i] >> 16);
//		}
//
//		mpu_set_gyro_bias_reg(gyro,st);
//
//#if defined (MPU6500) || defined (MPU9250)
//		mpu_set_accel_bias_6500_reg(accel);
//#elif defined (MPU6050) || defined (MPU9150)
//		mpu_set_accel_bias_6050_reg(accel,st);
//#endif
//#else
//		/* Push the calibrated data to the MPL library.
//		 *
//		 * MPL expects biases in hardware units << 16, but self test returns
//		 * biases in g's << 16.
//		 */
//		unsigned short accel_sens;
//		float gyro_sens;
//
//		mpu_get_accel_sens(&accel_sens);
//		accel[0] *= accel_sens;
//		accel[1] *= accel_sens;
//		accel[2] *= accel_sens;
//		//inv_set_accel_bias(accel, 3);
//		mpu_get_gyro_sens(&gyro_sens);
//		gyro[0] = (long) (gyro[0] * gyro_sens);
//		gyro[1] = (long) (gyro[1] * gyro_sens);
//		gyro[2] = (long) (gyro[2] * gyro_sens);
//		//inv_set_gyro_bias(gyro, 3);
//#endif
//	}
//	else {
//		if (!(result & 0x1)) printf("Gyro failed.\n");
//		if (!(result & 0x2)) printf("Accel failed.\n");
//		if (!(result & 0x4)) printf("Compass failed.\n");
//	}
//
//}
//
//void gyro_data_ready_cb(void)
//{
//	hal.new_gyro = 1;
//}
//
//void readSensorData(struct gyro_state_s *st){
//	unsigned long sensor_timestamp;
//
//	if (!hal.sensors || !hal.new_gyro) {
//		return;
//	}
//	if (hal.new_gyro && hal.dmp_on) {
//		short gyro[3], accel_short[3], sensors;
//		unsigned char more;
//		long quat[4];
//		float qw,qx,qy,qz;
//		dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more,st);
//
//		if (!more)
//			hal.new_gyro = 0;
//
//		if (sensors & INV_XYZ_GYRO) {
//			/*printf("gyro: %7.4f %7.4f %7.4f\n",
//					gyro[0]/65536.f,
//					gyro[1]/65536.f,
//					gyro[2]/65536.f);*/
//		}
//		if(sensors & INV_XYZ_ACCEL || sensors & INV_WXYZ_QUAT){
//			if(sensors & INV_WXYZ_QUAT)
//				{
//					qw = quat[0] / 65536.f;
//					qx = quat[1] / 65536.f;
//					qy = quat[2] / 65536.f;//(float) norm;
//					qz = quat[3] / 65536.f;//(float) norm;
//
//					float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
//
//					qw /= (float)norm;
//					qx /= (float)norm;
//					qy /= (float)norm;//(float) norm;
//					qz /= (float)norm;//(float) norm;
//					tempReading.ang = toEuler(qw, qx, qy, qz);
//				}
//			if (sensors & INV_XYZ_ACCEL) {
//#if REMOVE_GRAVITY == 0
//				tempReading.acc.accel_x = accel_short[0];//65536.f;
//				tempReading.acc.accel_y = accel_short[1];//65536.f;
//				tempReading.acc.accel_z = accel_short[2];//65536.f;
//
//
//#else
//				float qt[4] = {qw,qx,qy,qz};
//				removeGravity(&tempReading.acc,accel_short,qt);
////				tempReading.acc.accel_x
//
//#endif
//				/*
//				printf("accel: %7.4f %7.4f %7.4f\n",
//						1000*accel.accel_x/65536.f,
//						1000*accel.accel_y/65536.f,
//						1000*accel.accel_z/65536.f);*/
//
//			}
////			readingsMPU1.count++;
//			filter_mpuReadings(&MPU1,&readingsMPU1,&tempReading);
////			if(readingsMPU1.count >= FILTER_MPU_NSAMPLES) ;
//			//sendPose(angle,accel);
//		}
//	}
//}
//
