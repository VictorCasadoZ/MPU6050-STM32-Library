/*
 * MPU6050.h
 *
 *  Created on: 16 jul. 2020
 *      Author: pirul
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 			0x19
#define MPU6050_CONFIG			0x1A
#define GYRO_CONFIG_REG 		0x1B
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_H_REG 		0x3B
#define TEMP_OUT_H_REG 			0x41
#define GYRO_XOUT_H_REG 		0x43
#define PWR_MGMT_1_REG 			0x6B
#define WHO_AM_I_REG 			0x75

#define RAD_TO_DEG 				57.2957795130823
#define PI						3.14159265358979

typedef struct{			//Struct to store the data
	float Ax;
	float Ay;
	float Az;
	float Gx;
	float Gy;
	float Gz;
	float Temp;
	float Angle_X;
	float Angle_Y;
	float Angle_Z;
}MPU6050_Data_Struct;

uint8_t MPU6050_Init (I2C_HandleTypeDef *I2Cx);									//Initializes the MPU
void MPU6050_Read_Accel (I2C_HandleTypeDef *I2Cx, MPU6050_Data_Struct *data);	//Gets the Accelerometer data from the MPU
void MPU6050_Read_Gyro (I2C_HandleTypeDef *I2Cx, MPU6050_Data_Struct *data);	//Gets the Gyroscope data from the MPU
void MPU6050_Read_Temp (I2C_HandleTypeDef* I2Cx,MPU6050_Data_Struct *data);		//Gets the Temperature from the MPU
void MPU6050_Calculate_Angles (MPU6050_Data_Struct *data);						//Calculate the angle using Accelerometer data

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_H_ */
