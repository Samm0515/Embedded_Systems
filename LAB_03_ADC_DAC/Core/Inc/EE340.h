/*
 * EE340.h

 *      Description :					This is a library to read the values for an ADC
 *      Author :						Samuel Meysembourg
 *      Date Created :					01-13-2025
 */

#ifndef INC_EE340_H_
#define INC_EE340_H_

#include <stdint.h>
#include "stm32l5xx_hal.h"
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
void INIT_Buffer(buffer_t* buffer);

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
uint32_t processADC(buffer_t* buffer);





#endif /* INC_EE340_H_ */
