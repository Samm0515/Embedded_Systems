/*
 * 		EE340.c
 *
 *		LAB03_EE340
 *
 *  	Created on: Oct 6, 2025
 *
 *  	Author: Samuel Meysembourg
 *
 */
#include "EE340.h"
#include <string.h>
#include <stdlib.h>
/*
 * Function Prototype : 			void INIT_Buffer(buffer_t* buffer);
 *
 * Description :					This function initializes a new buffer. Pass in the new buffer data structure. Then
 * 										this function will fill the buffer and accumulate the running total.
 *
 * Inputs :							buffer_t* buffer : buffer data structure pointer as defined in header file for library.
 * 										Data structure to be created on stack or in dynamic memory.
 *
 * Outputs :						NONE
 *
 * Side Effects :					Buffer data structures values will be modified where it is through pointers.
 *
 * Example Usage :					buffer_t BUFFER;
 *	  	  	  	  	  	  	  	  	INIT_Buffer(&BUFFER);
 */
void INIT_Buffer(buffer_t* buffer)
{
	memset(buffer->buff, 0, sizeof(buffer->buff));
	buffer->index = 0;
	buffer->total = 0;

	// Fill Entire Buffer
	while (buffer->index < BUFFER_SIZE)
	{
		// Poll ADC for new Value
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1000);
		uint32_t value = HAL_ADC_GetValue(&hadc1);

		// Add new value into the buffer
		buffer->buff[buffer->index] = value;
		buffer->index = (buffer->index + 1);

		// Add to running total
		buffer->total += value;
	}

	// Reset to beginning
	buffer->index = 0;
}


/*
 * Function Prototype : 			uint32_t processADC(buffer_t* buffer);
 *
 * Description :					This function takes a new reading from the ADC and adds it to the buffer and recalculates
 * 										the new running average value for the ADC. It then returns this new running average value
 * 										for the ADC as a 32 bit unsigned integer.
 *
 * Inputs :							buffer_t* buffer : buffer structure to store the new values of the buffer and the running
 * 										average total for the ADC.
 *
 * Outputs :						uint32_t value representing the latest running average value for the ADC.
 *
 * Side Effects :					Buffer data structures values will be modified where it is through pointers.
 *
 * Example Usage :					uint32_t raw = processADC(&BUFFER);
 */
uint32_t processADC(buffer_t* buffer)
{
	// Poll ADC for new Value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint32_t value = HAL_ADC_GetValue(&hadc1);

	// Subtract oldest value from the buffer
	buffer->total -= buffer->buff[buffer->index];

	// Add new value into the buffer
	buffer->buff[buffer->index] = value;

	// Add new value to running sum
	buffer->total += value;

	// Update Index Value with the modulus ( when Index + 1 = 100 and buffer size = 100 output is 0)
	buffer->index = (buffer->index + 1) % BUFFER_SIZE;

	// Return the average of the buffer
	return (uint32_t)(buffer->total >> 4); // Shift over 4 bits effectively performing integer division of 16 (2^4) Every shift is another division by 2
}
