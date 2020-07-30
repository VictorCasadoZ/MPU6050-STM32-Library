/*
 * MPU6050.c
 *
 *  Created on: 16 jul. 2020
 *      Author: pirul
 */

#include "MPU6050.h"

int minVal=265;
int maxVal=402;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t MPU6050_Init (I2C_HandleTypeDef *I2Cx)
{
	uint8_t check;
	uint8_t Data;

	I2C_HandleTypeDef* Handle = I2Cx;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (Handle, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(Handle, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(Handle, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(Handle, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		//Set the Low Pass Filter at 94 Hz
		Data = 0x02;
		HAL_I2C_Mem_Write(Handle, MPU6050_ADDR, MPU6050_CONFIG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(Handle, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
		return 1;
	}
	return -1;

}


void MPU6050_Read_Accel (I2C_HandleTypeDef *I2Cx, MPU6050_Data_Struct *data)
{
	uint8_t Rec_Data[6];
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;

	I2C_HandleTypeDef* Handle = I2Cx;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (Handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	data->Ax = Accel_X_RAW/16384.0;
	data->Ay = Accel_Y_RAW/16384.0;
	data->Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (I2C_HandleTypeDef *I2Cx, MPU6050_Data_Struct *data)
{
	uint8_t Rec_Data[6];
	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;

	I2C_HandleTypeDef* Handle = I2Cx;

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (Handle, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	data->Gx = Gyro_X_RAW/131.0;
	data->Gy = Gyro_Y_RAW/131.0;
	data->Gz = Gyro_Z_RAW/131.0;
}

void MPU6050_Read_Temp (I2C_HandleTypeDef* I2Cx, MPU6050_Data_Struct *data)
{
	uint8_t Rec_Data[2];
	int16_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;

	/* Read temperature */
	HAL_I2C_Mem_Read (Handle, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);

	/* Format temperature */
	temp = (Rec_Data[0] << 8 | Rec_Data[1]);
	data->Temp = (float)((int16_t)temp / (float)340.0 + (float)36.53);

}

void MPU6050_Calculate_Angles (MPU6050_Data_Struct *data){

	double x_Buff = data->Ax;
	double y_Buff = data->Ay;
	double z_Buff = data->Az;

	data->Angle_X = atan2(y_Buff , z_Buff) * 57.3;
	data->Angle_Y = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
}
