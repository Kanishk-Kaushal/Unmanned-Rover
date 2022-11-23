/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#define PI 3.14

//#include "lsm9ds1_read_data_polling.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint16_t samples[2] ={ 0, 0 };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan;


/* USER CODE BEGIN PV */

CAN_FilterTypeDef sFilterConfig, sFilterConfig1;	// struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;	 					// struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   					// struct for transmitted dataframe
uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };		// array for recieved data (8 bytes)
uint8_t txData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };		// array for transmitting data (8 bytes)
uint32_t usedmailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
void Ports_Clocks(void);
void Timers_Init(void);
void ADC_Init(void);
int map(float k, float l, float h, float L, float H);
int ADC_2ch_read(int k);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Ports_Clocks();
	Timers_Init();
	ADC_Init();

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  TxMessage.IDE = CAN_ID_STD;				// standard identifier format (11bit)
  TxMessage.StdId = 0x446;					// identifier value
  TxMessage.RTR = CAN_RTR_DATA;				// indicates frame mode (data frame or remote frame)
  TxMessage.DLC = 8;						// data length (8 bytes)
  TxMessage.TransmitGlobalTime = DISABLE;	// time of transmission is not transmitted along with the data

  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	// filter bank consists of 2 32bit values (mask and ID)
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// filter set to mask and ID mode
  sFilterConfig.FilterBank = 0;							// filter bank number 0 selected
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;	// assign filter bank to FIFO 0
  sFilterConfig.FilterIdHigh = 0x211 << 5;				// STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
  sFilterConfig.FilterIdLow = 0;						// LSB
  sFilterConfig.FilterMaskIdHigh = 0x211 << 5;			// 0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
  sFilterConfig.FilterMaskIdLow = 0;					// LSB
  sFilterConfig.FilterActivation = ENABLE;				// activate filter

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
  HAL_CAN_Start(&hcan);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(GPIOA -> IDR & 0x00000001)
	{
		txData[0] = 0;
		txData[1] = 0;
	}
	else
	{
		txData[0] = ADC_2ch_read(0); 											// Data to be Transmitted over CAN
		txData[1] = ADC_2ch_read(1);
		txData[2] = 1;
	}



	HAL_CAN_AddTxMessage(&hcan, &TxMessage, txData, &usedmailbox);  // Transmitting

	//HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  __HAL_RCC_CAN1_CLK_ENABLE();
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Ports_Clocks()
{

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   // ENABLE CLOCK | PORT A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // ENABLE CLOCK | PORT B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // ENABLE ALTERNATE FUNCTION

	GPIOA -> CRL &= 0xFFFFFFF0; // RESET PORT A PIN TO 0
	GPIOA -> CRL |= 0x8; // PIN A0 | INPUT | PUSH PULL

	// PORT A | PIN 1 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1);

	// PORT A | PIN 2 | INPUT MODE | ANALOG INPUT
	GPIOA->CRL &= ~(GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);

	// PORT A | PIN 4 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE4;
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);

	// PORT A | PIN 5 | OUTPUT MODE | MAX SPEED = 50MHz | PUSH-PULL
	GPIOA->CRL |= GPIO_CRL_MODE5;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);


	// PORT B | PIN 6 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE6;
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	// PORT B | PIN 7 | OUTPUT MODE | MAX SPEED = 50MHz | ALTERNATE FUNCTION
	GPIOB->CRL |= GPIO_CRL_MODE7;
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

}

void Timers_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;          // TIMER 4 ENABLE
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; // CHANNEL 1 & CHANNEL 2 -> OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;                   // ENABLE ARPE | ARPE -> AUTO PRE-LOAD ENABLE

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   // CHANNEL 1 PRELOAD ENABLE
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   // CHANNEL 2 PRELOAD ENABLE

	// OUPUT COMPARE 1 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);

	// OUPUT COMPARE 2 MODE SET AS PWM MODE 1
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;     // PRESCALAR
	TIM4->ARR = 4095;  // AUTO-RELOAD VALUE -> (2^16 - 1)
	TIM4->CCR1 = 0;    // CAPTURE/COMPARE REGISTERS
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   // BEFORE STARTING TIMER -> INITIALIZE ALL REGISTERS
	TIM4->CR1 |= TIM_CR1_CEN;  // COUNTER ENABLE
}

void ADC_Init()
{
	GPIOA->BSRR |= 1 << 4 | 1 << 5;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   // PRESCLAR -> 72MHz/6
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // ADC1 CLOCK ENABLE
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;    // DMA1 CLOCK ENABLE

	// SAMPLING RATES FOR CHANNEL 1 & CHANNEL 2
	ADC1->SMPR2 |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0;
	ADC1->SMPR2 |= ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0;

	// SET CHANNEL 1 & 2 | SET SEQUENCE
	ADC1->SQR1 |= 1 << 20;
	ADC1->SQR3 |= ADC_SQR3_SQ1_0;
	ADC1->SQR3 |= ADC_SQR3_SQ2_1;

	ADC1->CR1 |= ADC_CR1_SCAN;   // SCAN MODE ENABLE
	ADC1->CR2 |= ADC_CR2_DMA;    // DMA MODE ENABLE


	// DMA CONFIGURATIONS

	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR));
	DMA1_Channel1->CMAR = (uint32_t) samples;
	DMA1_Channel1->CNDTR = 2;

	DMA1_Channel1->CCR |= DMA_CCR_CIRC;
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;

	DMA1_Channel1->CCR |= DMA_CCR_EN;   // DMA1 ENABLE

	ADC1->CR2 |= ADC_CR2_ADON;   // TURN ADC ON
	ADC1->CR2 |= ADC_CR2_CONT;   // ADC CONTINUOS MODE

	ADC1->CR2 |= ADC_CR2_ADON; // TURN ADC ON AGAIN | GIVEN IN REF MANUAL
	ADC1->CR2 |= ADC_CR2_CAL;  // RUN CALIBRATION

}

// MAP VALUES
int map(float k, float l, float h, float L, float H)
{
	return ((k - l) / (h - l)) * (H - L) + L;
}

// ADC 2 CHANNEL READ
int ADC_2ch_read(int k)
{
	int Joyval = 0;

	Joyval = samples[k];

	if(k == 0)
	{

		Joyval = map(Joyval, 0, 4095, 0, 255);
	}


	else
	{

		Joyval = map(Joyval, 0, 4095, 0, 255);

	}


	return Joyval;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
