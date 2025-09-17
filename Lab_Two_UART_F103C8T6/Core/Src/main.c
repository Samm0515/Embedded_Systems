/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Pin Structure
typedef struct
{
	GPIO_TypeDef *port;
	uint16_t pin;
}DEFINE_GPIO;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DOT_LENGTH 200
#define DASH_LENGTH 1000
#define INTERMEDIATE_LENGTH 200

// F104C8T6
#define LED_ON GPIO_PIN_RESET		// Flip this to run on L55!!!!!!!!
#define LED_OFF GPIO_PIN_SET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//Test 01
uint8_t tx_buffer[27] = "Welcome to EE340\n\r";

// Test 02
uint8_t rx_index;
uint8_t rx_data[2];
uint8_t rx_buffer[100];
uint8_t transfer_cplt = 0;

bool running = false;
uint8_t display_buffer[100];
uint8_t display_index = 0;

// Pin Definitions
DEFINE_GPIO GREEN_LED = {GPIOC, GPIO_PIN_13};

// Morse Code Lookup Table
int lookup_table[26][5] = 	{
							{65,1,2,0,0},     // A: · –
							{66,2,1,1,1},     // B: – · · ·
							{67,2,1,2,1},     // C: – · – ·
							{68,2,1,1,0},     // D: – · ·
							{69,1,0,0,0},     // E: ·
							{70,1,1,2,1},     // F: · · – ·
							{71,2,2,1,0},     // G: – – ·
							{72,1,1,1,1},     // H: · · · ·
							{73,1,1,0,0},     // I: · ·
							{74,1,2,2,2},     // J: · – – –
							{75,2,1,2,0},     // K: – · –
							{76,1,2,1,1},     // L: · – · ·
							{77,2,2,0,0},     // M: – –
							{78,2,1,0,0},     // N: – ·
							{79,2,2,2,0},     // O: – – –
							{80,1,2,2,1},     // P: · – – ·
							{81,2,2,1,2},     // Q: – – · –
							{82,1,2,1,0},     // R: · – ·
							{83,1,1,1,0},     // S: · · ·
							{84,2,0,0,0},     // T: –
							{85,1,1,2,0},     // U: · · –
							{86,1,1,1,2},     // V: · · · –
							{87,1,2,2,0},     // W: · – –
							{88,2,1,1,2},     // X: – · · –
							{89,2,1,2,2},     // Y: – · – –
							{90,2,2,1,1}      // Z: – – · ·
							};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

// Function Prototypes
bool Display_Character(char input_char);
bool dot(GPIO_TypeDef *port, uint16_t pin);
bool dash(GPIO_TypeDef *port, uint16_t pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Functions

/*
 * Function Prototype 					int ConvertToMorseCode(char character);
 *
 * Description:							This function will parse out the input character into the corresponding morse code sequence.
 *
 * Inputs:								char character : character representing character typed by user. Domain A - Z (capitol).
 *
 * Outputs:								int : 1-26 representing the character that was the input. -1 if the function did not find a match.
 *
 * Side Effects:						Function holds processor with waiting timer until the function successfully displayed the
 * 											led sequence associated with the character input.
 *
 * Example Usage:						for (int i = 0; i < display_index; i++)				// Display all of the characters in the display buffer in their morse code conversions
 *										{
 *											ConvertToMorseCode((char)display_buffer[i]);	// Need to cast uint8_t to char
 *										}
*/
int ConvertToMorseCode(char character)
{
	// Test the input
	switch (character)
	{

	case 'A':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 1;

	case 'B':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 2;

	case 'C':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 3;

	case 'D':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 4;

	case 'E':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 5;

	case 'F':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 6;

	case 'G':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 7;

	case 'H':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 8;

	case 'I':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 9;

	case 'J':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 10;

	case 'K':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 11;

	case 'L':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 12;

	case 'M':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 13;

	case 'N':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 14;

	case 'O':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 15;

	case 'P':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 16;

	case 'Q':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 17;

	case 'R':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 18;

	case 'S':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 19;

	case 'T':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 20;

	case 'U':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 21;

	case 'V':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 22;

	case 'W':
		// Morse
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 23;

	case 'X':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 24;

	case 'Y':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		return 25;

	case 'Z':
		// Morse
		dash(GREEN_LED.port, GREEN_LED.pin);
		dash(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		dot(GREEN_LED.port, GREEN_LED.pin);
		return 26;

	default:
		// Report Error
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		//HAL_Delay(DASH_LENGTH);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		return -1;
	}
}

bool Display_Character(char input_char)
{
	// Display morse code
	    int row_match = 0;
	    bool match_found = false;

	    for (int i = 0; i < 26; i++)                    // For each row
	    {
	        for (int j = 0; j < 5; j++)                 // For each position in each row
	        {
	            // Look for match in the table
	            if (lookup_table[i][j] == input_char)
	            {
	                row_match = i;
	                match_found = true;
	                break;
	            }
	        }
	        if (match_found)
	        break;
	    }
	    if (match_found)
	    {
	        for (int i = 1; i < 5; i++)
	        {
	            int value = lookup_table[row_match][i];
	            if (value == 1)
	            {
	            	dot(GREEN_LED.port, GREEN_LED.pin);
	            }
	            if (value == 2)
	            {
	            	dash(GREEN_LED.port, GREEN_LED.pin);
	            }
	        }
	        // Character found and displayed
	        return true;
	    }
	    // No match found
	    else
	    {
	    	return false;
	    }
}


/*
 * Function Prototype 					bool dot(GPIO_TypeDef *PORT, uint16_t PIN);
 *
 * Description:							This function will display a "dot" in morse code through the LED which it takes the parameters
 * 											for as inputs.
 *
 * Inputs:								GPIO_TypeDef *PORT : GPIO port from the PinDefine structure.
 * 										uint16_t PIN : GPIO_PIN_X pin from the PinDefine structure.
 *
 * Outputs:								bool : true and false representing if the function has completed succesfully or unsucessfully respectivley.
 *
 * Side Effects:						Function holds processor with waiting timer until the morse code finishes.
 *
 * Example Usage:						dot(GREEN_LED.port, GREEN_LED.pin);
*/
bool dot(GPIO_TypeDef *PORT, uint16_t PIN)
{
	// Turn on LED
	HAL_GPIO_WritePin(PORT, PIN, LED_ON);
	// Delay for dot
	HAL_Delay(DOT_LENGTH);
	// Turn LED off
	HAL_GPIO_WritePin(PORT, PIN, LED_OFF);
	// Delay Normal Time period
	HAL_Delay(INTERMEDIATE_LENGTH);
	// Return True to indicate success
	return true;
}


/*
 * Function Prototype 					bool dash(GPIO_TypeDef *PORT, uint16_t PIN);
 *
 * Description:							This function will display a "dash" in morse code through the LED which it takes the parameters
 * 											for as inputs.
 *
 * Inputs:								GPIO_TypeDef *PORT : GPIO port from the PinDefine structure.
 * 										uint16_t PIN : GPIO_PIN_X pin from the PinDefine structure.
 *
 * Outputs:								bool : true and false representing if the function has completed succesfully or unsucessfully respectivley.
 *
 * Side Effects:						Function holds processor with waiting timer until the morse code finishes.
 *
 * Example Usage:						dot(GREEN_LED.port, GREEN_LED.pin);
*/
bool dash(GPIO_TypeDef *PORT, uint16_t PIN)
{
	// Turn on LED
	HAL_GPIO_WritePin(PORT, PIN, LED_ON);
	// Delay for dot
	HAL_Delay(DASH_LENGTH);
	// Turn LED off
	HAL_GPIO_WritePin(PORT, PIN, LED_OFF);
	// Delay Normal Time period
	HAL_Delay(INTERMEDIATE_LENGTH);
	// Return True to indicate success
	return true;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Test 02
  HAL_UART_Receive_IT(&huart1, rx_data, 1);		// Start initial uart rx interrupt
  HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_OFF);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  // Test 01
	  /*
	  HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 10);
	  HAL_Delay(1000);
	  */

	  // Test 02
	  if (running)		// If Display flag is set
	  {
		  HAL_UART_AbortReceive_IT(&huart1);					// Disable rx interrupt while displaying morse code conversions
		  for (int i = 0; i < display_index; i++)				// Display all of the characters in the display buffer in their morse code conversions
			{
			  if (!Display_Character((char)display_buffer[i]))
				{
					HAL_UART_Transmit(&huart1, "Character not found...\n\r", sizeof("Character not found...\n\r"), 10);
				}
			}


		  // Reset display buffers and index
		  for(int i = 0; i < 100; i++)
			{
			  display_buffer[i] = 0;
			}
		  display_index = 0;
		  running = false;
		  // Restart UART interrupt to start receiving data again
		  HAL_UART_Receive_IT(&huart1, rx_data, 1);
	  }
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


// Test 02
/*
 * Function Prototype 					void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
 *
 * Description:							This function is the non weak definition of the UART callback for receive. This will run when
 * 											the UART RX interrupt triggers it (when one byte of data is received. This function will
 * 											run if the running flag is false meaning the main while loop is not displaying the
 * 											display_buffer. When the user types the return character the function will transfer the
 * 											rx_buffer to the display_buffer then set the running flag to true to trigger the main while
 * 											loop to start displaying the characters in the buffer in their morse code conversions.
 *
 * Inputs:								UART_HandleTypeDef *huart : pointer to the UART structure to communicate with.
 *
 * Outputs:								void : NONE
 *
 * Side Effects:						Function changes the buffer and display variables / turns off LEDs
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	uint8_t i;
	if((huart->Instance == USART1) && !running)									// Check if interrupt came from USART 3 and not any of the other uart multiplexed with this port
	{
		if(rx_index == 0)														// If the position in the buffer is 0 then reset the whole buffer
		{
			for(i = 0; i < 100; i++)
			{
				rx_buffer[i] = 0;
			}

			HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_OFF);				// Turn off the LED to indicate the buffer has been cleared
	}

	if(rx_data[0] != 13)														// If incoming data is not the carriage return save data to buffer
	{
		rx_buffer[rx_index++] = rx_data[0];										// Add data to rx_buffer if not carriage return
	}

	else																		// If incoming data is the carriage return do this...
	{
		transfer_cplt = 1;														// Set transfer complete flag to high
		HAL_UART_Transmit(&huart1, "\n\r", 2, 100);								// Send new line and carriage return to serial monitor
		// Start Display
		running = true;
		// Copy buffer ind index
		display_index = rx_index;
		for(i = 0; i < 100; i++)												// Transfer the rx_buffer into a new display buffer
		{
			display_buffer[i] = rx_buffer[i];
		}

		rx_index = 0;															// Reset index to 0


		/*
		// Test 01
		if(!strcmp(rx_buffer, "LED ON"))										// If data in buffer is the string "LED ON" turn the led on
		{
			HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_ON);				// Turn on LED to indicate "LED ON" works
		}
		*/


	}
	HAL_UART_Receive_IT(&huart1, rx_data, 1);									// Initilize uart interrupt again
	HAL_UART_Transmit(&huart1, rx_data, 1, 100);								// Echo back usrt recived data
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
