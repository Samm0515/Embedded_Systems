/*
 * 		EE340.h
 *
 *      Description :					This is a library to read the values for an ADC and process
 *      									them through a running average.
 *
 *      Author :						Samuel Meysembourg
 *
 *      Date Created :					01-13-2025
 */

#ifndef INC_EE340_H_
#define INC_EE340_H_

#include <stdint.h>
#include "stm32l5xx_hal.h"

// External Variables
extern ADC_HandleTypeDef hadc1;

// Private Macros
#define BUFFER_SIZE 16

// Data Structure for Buffer
typedef struct buffer
{
	// Buffer
	uint32_t buff[BUFFER_SIZE];
	// Buffer Index
	uint16_t index;
	// Running sum
	uint32_t total;

}buffer_t;


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
void INIT_Buffer(buffer_t* buffer);


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
uint32_t processADC(buffer_t* buffer);


#endif /* INC_EE340_H_ */
