/*
 * mpu6050.c
 *
 *  Created on: Sep 18, 2025
 *      Author: samuel.meysembourg
 */
#include "mpu6050.h"

// Private Includes
#include <stdlib.h>
#include <string.h>

// MCU Dependant
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

// MPU6050 Register Addresses
#define MPU6050_ADDR 0b11010000

// Debug Variables
#define VERBOSITY 0
int ERROR_CODE = 0;
uint8_t DEBUG_BUFFER[100];
uint8_t ERROR_BUFFER[100];
HAL_StatusTypeDef ret;
/*
 * ERROR CODES :
 * [1] Memory Allocation Error
 * [2] Device Not Ready
 */



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
MPU6050_Data* MPU6050_INIT(I2C_HandleTypeDef *hi2c)
{
	// Create Data structure on the heap
	MPU6050_Data* MPU6050 = (MPU6050_Data*)malloc(sizeof(MPU6050_Data));

	// Check if memory was properly allocated
	if (MPU6050 == NULL)
	{
		// Memory Allocation Error
		if (VERBOSITY >= 1)
		{
			ERROR_CODE = 1;
			strcpy((char*)ERROR_BUFFER, "MPU6050 ERROR : [1] Memory Allocation Error__\n\r");
		}
		return NULL;
	}

	// INIT Members of structure
	MPU6050 -> accelx = 0;
	MPU6050 -> accely = 0;
	MPU6050 -> accelz = 0;
	MPU6050 -> gyrox = 0;
	MPU6050 -> gyroy = 0;
	MPU6050 -> gyroz = 0;
	MPU6050 -> hi2c = hi2c;

	// Check if device is ready
	ret = HAL_I2C_IsDeviceReady(MPU6050 -> hi2c, MPU6050_ADDR, 1, 100);
	if (ret == HAL_OK)
	{
		if (VERBOSITY >= 1)
		{
			strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Device Ready__\n\r");
		}
	}
	else
	{
		ERROR_CODE = 2;
		strcpy((char*)ERROR_BUFFER, "MPU6050 ERROR : [2] Device Not Ready__\n\r");
		free(MPU6050);
		return NULL;
	}

	// Return pointer to new structure
	return MPU6050;
}
