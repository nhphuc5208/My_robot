#ifndef __MPU_H
#define __MPU_H
#include <stdio.h>
#include <stdint.h>
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#define MPU6050_ADDRESS  (0x68<<1)
#define HMC5883L_ADDRESS (0x1E<<1)
#define ACCEL_XOUT_H      0x3B
#define TEMP_OUT_H        0x41
#define GYRO_XOUT_H       0x43
#define EXT_SENS_DATA_00  0x49
#define M_PI              3.14159265358979323846
extern I2C_HandleTypeDef hi2c1;
typedef struct {
	float accel_x_raw, accel_y_raw, accel_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw, magn_x_raw, magn_y_raw, magn_z_raw;
	} data_mpu;
void mpu6050_config(void);
void hmc5883l_config(void);
void mpu_10dof_config(void);
data_mpu mpu6050_read_sensors(void);
float IMU_Kalman(float newrate, float newangle, float dt);
#endif