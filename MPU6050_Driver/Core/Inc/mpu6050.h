/*
 * mpu6050.h
 *
 *  Created on: Sep 18, 2025
 *      Author: samuel.meysembourg
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"

// A struct to hold the sensor data
typedef struct
{
	I2C_HandleTypeDef *hi2c;
	int accelx;
	int accely;
	int accelz;
	int gyrox;
	int gyroy;
	int gyroz;
} MPU6050_Data;

// Function prototypes for the MPU6050 library

#endif /* INC_MPU6050_H_ */
