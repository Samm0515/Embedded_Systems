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
#include "main.h"

// MCU Dependant
extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart1;




// MPU6050 Register Addresses
#define MPU6050_ADDR 0b1101000

// Debug Variables
#define VERBOSITY 1
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
MPU6050_Data* MPU6050_INIT (I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
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
	MPU6050 -> huart = huart;

	// Check if device is ready
	ret = HAL_I2C_IsDeviceReady(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 0, 1, 100);
	if (ret == HAL_OK)
	{
		if (VERBOSITY >= 1)
		{
			strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Device Ready__\n\r");
			HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
		}
	}
	else
	{
		ERROR_CODE = 2;
		strcpy((char*)ERROR_BUFFER, "MPU6050 ERROR : [2] Device Not Ready__\n\r");
		HAL_UART_Transmit(MPU6050 -> huart, ERROR_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
		free(MPU6050);
		return NULL;
	}

	// Return pointer to new structure
	return MPU6050;
}


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
void MPU6050_ConfigureDevice (MPU6050_Data* MPU6050, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	// Configure register 27 GYRO
	uint8_t temp_data = 0b00001000;
	ret =  HAL_I2C_Mem_Write(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 0, 27, 1, &temp_data, 1, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		{
			if (VERBOSITY >= 1)
			{
				strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Configuring Gyroscope__\n\r");
				HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
			}
		}
		else
		{
			strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Failed to configure Gyroscope 27__\n\r");
			HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
		}

	// Configure register 28 ACCEL
	temp_data = 0b00001000;
	ret =  HAL_I2C_Mem_Write(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 0, 28, 1, &temp_data, 1, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		{
			if (VERBOSITY >= 1)
			{
				strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Configuring Accelerometer__\n\r");
				HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
			}
		}
		else
		{
			strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Failed to configure Accelerometer 28__\n\r");
			HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
		}

	// Configure Power mode and enable temperature sensor
	temp_data = 0;
		ret =  HAL_I2C_Mem_Write(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 0, 107, 1, &temp_data, 1, HAL_MAX_DELAY);
		if (ret == HAL_OK)
			{
				if (VERBOSITY >= 1)
				{
					strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Exiting Sleep Mode__\n\r");
					HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), HAL_MAX_DELAY);
				}
			}
			else
			{
				strcpy((char*)DEBUG_BUFFER, "MPU6050 DEBUG : Failed to exit sleep mode__\n\r");
				HAL_UART_Transmit(MPU6050 -> huart, DEBUG_BUFFER, sizeof(DEBUG_BUFFER), 100);
			}
}


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
void MPU6050_Update_Values(MPU6050_Data* MPU6050, I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
	uint8_t data[2];
	uint16_t x_acc;
	uint16_t y_acc;
	uint16_t z_acc;
	uint16_t temp;

	uint8_t buffer[100];

	ret = HAL_I2C_Mem_Read(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 1, 59, 1, data, 2, 100);
	x_acc = (int16_t)(data[0] << 8 | data[1]);

	ret = HAL_I2C_Mem_Read(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 1, 61, 1, data, 2, 100);
	y_acc = (int16_t)(data[0] << 8 | data[1]);

	ret = HAL_I2C_Mem_Read(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 1, 63, 1, data, 2, 100);
	z_acc = (int16_t)(data[0] << 8 | data[1]);

	ret = HAL_I2C_Mem_Read(MPU6050 -> hi2c, (MPU6050_ADDR <<1) + 1, 65, 1, data, 2, 100);
	temp = (int16_t)(data[0] << 8 | data[1]);
	temp = (temp / 340) + 36;

	MPU6050 -> accelx = x_acc;
	MPU6050 -> accely = y_acc;
	MPU6050 -> accelz = z_acc;

	if (VERBOSITY >= 1)
	{
		sprintf(buffer, "Accelerometer XYZ Data : (%d, %d, %d, %d)\n\r", x_acc, y_acc, z_acc, temp);
		HAL_UART_Transmit(MPU6050 -> huart, (uint8_t*)buffer, strlen(buffer), 100);
	}
}
