/* USER CODE BEGIN Header */
// By Samuel Meysembourg
// EE340 Fall 2025
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

// Pin Definition Struct
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

#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

RTC_HandleTypeDef hrtc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

//Test 01
uint8_t tx_buffer[27] = "Welcome to EE340\n\r";

// Test 02
uint8_t rx_index;
uint8_t rx_data[2];
uint8_t rx_buffer[100];
uint8_t transfer_cplt;

bool running = false;
uint8_t display_buffer[100];
uint8_t display_index = 0;

// Test select
uint8_t test = 1;		// 0 = test 01, 1 = test 02

// Pin Definitions
DEFINE_GPIO GREEN_LED = {GPIOC, GPIO_PIN_7};
DEFINE_GPIO RED_LED = {GPIOA, GPIO_PIN_9};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// Function Prototypes
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
		HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, GPIO_PIN_SET);
		HAL_Delay(DASH_LENGTH);
		HAL_GPIO_WritePin(RED_LED.port, RED_LED.pin, GPIO_PIN_RESET);
		return -1;
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
 * Outputs:								bool : true and false representing if the function has completed successfully or unsuccessfully respectively.
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
 * Outputs:								bool : true and false representing if the function has completed successfully or unsuccessfully respectively.
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
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_UCPD1_Init();
  MX_USB_PCD_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Test 02
  HAL_UART_Receive_IT(&huart3, rx_data, 1);		// Start initial UART RX interrupt
  HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_OFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Test 00
	  //HAL_UART_Transmit(&huart3, tx_buffer, sizeof(tx_buffer), 10);
	  //HAL_Delay(1000);

	  // Test 01
	  // NONE

	  // Test 02
	  if (test && running)		// If Display flag is set
	  {
		  HAL_UART_AbortReceive_IT(&huart3);					// Disable RX interrupt while displaying morse code conversions
		  for (int i = 0; i < display_index; i++)				// Display all of the characters in the display buffer in their morse code conversions
			{
				ConvertToMorseCode((char)display_buffer[i]);	// Need to cast uint8_t to char
			}


		  // Reset display buffers and index
		  for(int i = 0; i < 100; i++)
			{
			  display_buffer[i] = 0;
			}
		  display_index = 0;
		  running = false;
		  // Restart UART interrupt to start receiving data again
		  HAL_UART_Receive_IT(&huart3, rx_data, 1);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**UCPD1 GPIO Configuration
  PB15   ------> UCPD1_CC2
  PA15 (JTDI)   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_DBN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = UCPD_DBN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	if((huart->Instance == USART3) && !running)									// Check if interrupt came from USART 3 and not any of the other UART multiplexed with this port
	{
		if(rx_index == 0)														// If the position in the buffer is 0 then reset the whole buffer
		{
			for(i = 0; i < 100; i++)
			{
				rx_buffer[i] = 0;
			}

			HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_OFF);			// Turn off the LED to indicate the buffer has been cleared
	}

	if(rx_data[0] != 13)														// If incoming data is not the carriage return save data to buffer
	{
		rx_buffer[rx_index++] = rx_data[0];										// Add data to rx_buffer if not carriage return
	}

	else																		// If incoming data is the carriage return do this...
	{
		transfer_cplt = 1;														// Set transfer complete flag to high
		HAL_UART_Transmit(&huart3, (uint8_t *)"\n\r", 2, 100);								// Send new line and carriage return to serial monitor
		// Start Display

		// Test 02
		if (test)
		{
			running = true;
			// Copy buffer display index
			display_index = rx_index;
			for(i = 0; i < 100; i++)												// Transfer the rx_buffer into a new display buffer
			{
				display_buffer[i] = rx_buffer[i];
			}
		}
		rx_index = 0;															// Reset index to 0


		if (test)
		{
			// Test 01
			if(!strcmp((char *)rx_buffer, "LED ON"))										// If data in buffer is the string "LED ON" turn the led on
			{
				HAL_GPIO_WritePin(GREEN_LED.port, GREEN_LED.pin, LED_ON);			// Turn on LED to indicate "LED ON" works
			}
		}




	}
	HAL_UART_Receive_IT(&huart3, rx_data, 1);									// Initialize UART interrupt again
	HAL_UART_Transmit(&huart3, rx_data, 1, 100);								// Echo back UART received data
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
