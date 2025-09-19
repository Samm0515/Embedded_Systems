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
	UART_HandleTypeDef *huart;
	int16_t accelx;
	int16_t accely;
	int16_t accelz;
	int16_t gyrox;
	int16_t gyroy;
	int16_t gyroz;
} MPU6050_Data;

// Function prototypes for the MPU6050 library

/*
 * Function Prototype : 			_
 *
 * Description :					_
 *
 * Inputs :							_
 *
 * Outputs :						_
 *
 * Side Effects :					_
 *
 * Example Usage :					_
 */
MPU6050_Data* MPU6050_INIT(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

void MPU6050_ConfigureDevice (MPU6050_Data* MPU6050, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

void MPU6050_Update_Values(MPU6050_Data* MPU6050, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif /* INC_MPU6050_H_ */
