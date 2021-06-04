/*
 * mpu.c
 *
 *  Created on: May 22, 2021
 *      Author: kmori
 */


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "mpu.h"
#include "Message.h"
#include "macros.h"
#include <stdio.h>

extern struct hal_s hal;
volatile uint32_t hal_timestamp = 0;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";
extern struct angles_t angle;
extern struct accel_t accel;

int MPU_init(){
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	struct int_param_s int_param;
	int_param.cb = gyro_data_ready_cb;

	if (mpu_init(&int_param) !=0){
		MPL_LOGI("Could not start MPU!\n");
		return -1;
	}

#ifdef COMPASS_ENABLED
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif

	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);

#ifdef COMPASS_ENABLED
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
	mpu_get_sample_rate(&gyro_rate);
	mpu_get_gyro_fsr(&gyro_fsr);
	mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;
	if(dmp_load_motion_driver_firmware()){
		printf("Could not start DMP!\r\n");
		return -1;
	}

	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);

	hal.dmp_features = DMP_FEATURE_SEND_RAW_ACCEL |DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);

	if(mpu_set_dmp_state(1)){
		printf("Could not enable DMP!\r\n");
		return -1;
	}
	hal.dmp_on = 1;

	run_self_test();

	return 0;
}

struct angles_t toEuler(float qw,float qx, float qy, float qz){
	struct angles_t angle;
	//roll
	double r1 = 2 * (qw * qx + qy * qz);
	double r2 = 1 - 2 * (qx * qx + qy * qy);
	angle.roll = 100*atan2(r1,r2);

	double p1 = 2 * (qw * qy - qz * qx);
	if (fabs(p1) >= 1)
		angle.pitch = 100*copysign(M_PI / 2, p1); // use 90 degrees if out of range
	else
		angle.pitch = 100*asin(p1);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (qw * qz + qx * qy);
	double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
	angle.yaw = 100*atan2(siny_cosp, cosy_cosp);

	return angle;
}

void run_self_test(void)
{
	int result;
	long gyro[3], accel[3];


	result = mpu_run_self_test(gyro, accel);
	if (result == 0x7) {
		MPL_LOGI("Passed!\n");
		printf("accel: %7.4f %7.4f %7.4f\n",
				accel[0]/65536.f,
				accel[1]/65536.f,
				accel[2]/65536.f);
		printf("gyro: %7.4f %7.4f %7.4f\n",
				gyro[0]/65536.f,
				gyro[1]/65536.f,
				gyro[2]/65536.f);
		/* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
		/*
		 * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
		 * instead of pushing the cal data to the MPL software library
		 */
		unsigned char i = 0;

		for(i = 0; i<3; i++) {
			gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
			accel[i] *= 2048.f; //convert to +-16G
			accel[i] = accel[i] >> 16;
			gyro[i] = (long)(gyro[i] >> 16);
		}

		mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel);
#endif
#else
		/* Push the calibrated data to the MPL library.
		 *
		 * MPL expects biases in hardware units << 16, but self test returns
		 * biases in g's << 16.
		 */
		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		//inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long) (gyro[0] * gyro_sens);
		gyro[1] = (long) (gyro[1] * gyro_sens);
		gyro[2] = (long) (gyro[2] * gyro_sens);
		//inv_set_gyro_bias(gyro, 3);
#endif
	}
	else {
		if (!(result & 0x1)) printf("Gyro failed.\n");
		if (!(result & 0x2)) printf("Accel failed.\n");
		if (!(result & 0x4)) printf("Compass failed.\n");
	}

}

void gyro_data_ready_cb(void)
{
	hal.new_gyro = 1;
}

void readSensorData(void){
	unsigned long sensor_timestamp;

	if (!hal.sensors || !hal.new_gyro) {
		return;
	}
	if (hal.new_gyro && hal.dmp_on) {
		short gyro[3], accel_short[3], sensors;
		unsigned char more;
		long quat[4];
		dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

		if (!more)
			hal.new_gyro = 0;

		if (sensors & INV_XYZ_GYRO) {
			/*printf("gyro: %7.4f %7.4f %7.4f\n",
					gyro[0]/65536.f,
					gyro[1]/65536.f,
					gyro[2]/65536.f);*/
		}
		if(sensors & INV_XYZ_ACCEL || sensors & INV_WXYZ_QUAT){
			if (sensors & INV_XYZ_ACCEL) {
				accel.accel_x = accel_short[0];
				accel.accel_y = accel_short[1];
				accel.accel_z = accel_short[2];
			}
			if(sensors & INV_WXYZ_QUAT)
				{
					angle = toEuler(quat[0], quat[1], quat[2], quat[3]);
				}
			//sendPose(angle,accel);
		}
	}
}

