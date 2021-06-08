/*
 * sensorData.c
 *
 *  Created on: Jun 5, 2021
 *      Author: lucya
 */

#include <mySensor.h>
#include "macros.h"
#include "math.h"
#include "stdio.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Message.h"


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

void filter_mpuReadings(struct sensor_t *sens,struct accel_t *acc ,struct angles_t *ang){
//	sens->filtered
	AccelpAssign(&(sens->raw.accelerations),acc);
	AngpAssign(&(sens->raw.angles),ang);
	sens->count++;
	if(sens->count >= FILTER_MPU_NSAMPLES){
		AccelsRight(&(sens->raw.accelerations),FILTER_MPU_DIVIDER);
		AngsRight(&(sens->raw.angles),FILTER_MPU_DIVIDER);


		sens->filtered.accelerations = sens->raw.accelerations;
		sens->filtered.angles = sens->raw.angles;

		sens->raw.accelerations.accel_x = 0;
		sens->raw.accelerations.accel_y = 0;
		sens->raw.accelerations.accel_z = 0;
		sens->raw.angles.pitch = 0;
		sens->raw.angles.roll = 0;
		sens->raw.angles.yaw = 0;

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

//struct sensor_t MPU1 = {{0,0,0},{0,0,0},0};











//mpu






/*
 * mpu.c
 *
 *  Created on: May 22, 2021
 *      Author: kmori
 */
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "math.h"
//#include "mpu.h"
//#include "Message.h"
//#include "macros.h"
//#include <stdio.h>

extern struct hal_s hal;
volatile uint32_t hal_timestamp = 0;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";
//extern struct angles_t angle;

//struct sensor_t readingsMPU1 = {{0,0,0},{0,0,0},0};
//struct sensor_t tempReading = {{0,0,0},{0,0,0},0};


struct accel_t Acctemp = {0,0,0};
struct angles_t Angtemp = {0,0,0};

//extern struct sensor_t MPU1;

//extern struct readings_t readingsMPU1;

int MPU_init(struct sensor_t *sens,int AD0){
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	struct int_param_s int_param;
//	int_param.cb = gyro_data_ready_cb;

	// Structure initialization;
	if(AD0 == 0) sens->idx = 1;
	else sens->idx = 2;

	sens->count = 0;

	sens->filtered.accelerations.accel_x = 0;
	sens->filtered.accelerations.accel_y = 0;
	sens->filtered.accelerations.accel_x = 0;

	sens->raw.accelerations.accel_x = 0;
	sens->raw.accelerations.accel_y = 0;
	sens->raw.accelerations.accel_z = 0;

	sens->filtered.angles.pitch = 0;
	sens->filtered.angles.roll = 0;
	sens->filtered.angles.yaw = 0;

	sens->raw.angles.pitch = 0;
	sens->raw.angles.roll = 0;
	sens->raw.angles.yaw = 0;

	sens->interrupt_flag = 0;


	if (mpu_init(&int_param,AD0,&(sens->st)) !=0){
		MPL_LOGI("Could not start MPU!\n");
		return -1;
	}





	// Structure initialization;

#ifdef COMPASS_ENABLED
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL,&(sens->st));
#endif

	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL,&(sens->st));
	mpu_set_sample_rate(DEFAULT_MPU_HZ,&(sens->st));

#ifdef COMPASS_ENABLED
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
	mpu_get_sample_rate(&gyro_rate,&(sens->st));
	mpu_get_gyro_fsr(&gyro_fsr,&(sens->st));
	mpu_get_accel_fsr(&accel_fsr,&(sens->st));
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
	if(dmp_load_motion_driver_firmware(&(sens->st))){
		printf("Could not start DMP!\r\n");
		return -1;
	}

	dmp_set_interrupt_mode(DMP_INT_CONTINUOUS,&(sens->st));

	hal.dmp_features = DMP_FEATURE_SEND_RAW_ACCEL |DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
	dmp_enable_feature(hal.dmp_features,&(sens->st));
	dmp_set_fifo_rate(DEFAULT_MPU_HZ,&(sens->st));

	if(mpu_set_dmp_state(1,&(sens->st))){
		printf("Could not enable DMP!\r\n");
		return -1;
	}
	hal.dmp_on = 1;

	run_self_test(&(sens->st));

	return 0;
}

struct angles_t toEuler(float qw,float qx, float qy, float qz){
	struct angles_t angle;
	float epsilon = 0.00001;

	//	roll
	float r1 = 2 * (qw * qx + qy * qz);
	float r2 = 1 - 2 * (qx * qx + qy * qy);
	angle.roll = 180*atan2(r1,r2)/M_PI;

	float p1 = 2 * (qw * qy - qz * qx);
	if (fabs(p1) >= 1 - epsilon)
		angle.pitch = 180*copysignf(M_PI / 2, p1); // use 90 degrees if out of range
	else
		angle.pitch = 180*asin(p1)/M_PI;

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (qw * qz + qx * qy);
	float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
	angle.yaw = 180*atan2(siny_cosp, cosy_cosp)/M_PI;

	return angle;

}

void run_self_test(struct gyro_state_s *st)
{
	int result;
	long gyro[3], accel[3];


	result = mpu_run_self_test(gyro, accel,st);
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

		mpu_set_gyro_bias_reg(gyro,st);

#if defined (MPU6500) || defined (MPU9250)
		mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
		mpu_set_accel_bias_6050_reg(accel,st);
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


void readSensorData(struct sensor_t *sens){
	unsigned long sensor_timestamp;

//	if (!hal.sensors || !sens->interrupt_flag) {
//		return;
//	}
	if (sens->interrupt_flag && hal.dmp_on) {
		short gyro[3], accel_short[3], sensors;
		unsigned char more;
		long quat[4];
		float qw,qx,qy,qz;
		dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more,&(sens->st));

		if (!more)
			sens->interrupt_flag = 0;


		if(sensors & INV_XYZ_ACCEL || sensors & INV_WXYZ_QUAT){
			if(sensors & INV_WXYZ_QUAT)
				{
					qw = quat[0] / 65536.f;
					qx = quat[1] / 65536.f;
					qy = quat[2] / 65536.f;//(float) norm;
					qz = quat[3] / 65536.f;//(float) norm;

					float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

					qw /= (float)norm;
					qx /= (float)norm;
					qy /= (float)norm;//(float) norm;
					qz /= (float)norm;//(float) norm;
					Angtemp = toEuler(qw, qx, qy, qz);
				}
			if (sensors & INV_XYZ_ACCEL) {
#if REMOVE_GRAVITY == 0
				Acctemp.accel_x = accel_short[0];//65536.f;
				Acctemp.accel_y = accel_short[1];//65536.f;
				Acctemp.accel_z = accel_short[2];//65536.f;


#else
				float qt[4] = {qw,qx,qy,qz};
				removeGravity(&Acctemp,accel_short,qt);

#endif
//				struct accel_t Acctemp = {0,0,0};
//				struct angles_t Angtemp = {0,0,0};
			}
//			readingsMPU1.count++;
			filter_mpuReadings(sens,&Acctemp,&Angtemp);
//			if(readingsMPU1.count >= FILTER_MPU_NSAMPLES) ;
			//sendPose(angle,accel);
		}
	}
}


