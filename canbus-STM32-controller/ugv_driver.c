/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int _write(int fd, char* ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

CAN_HandleTypeDef hcan;

CAN_FilterTypeDef sFilterConfig;				// Struct containing filter settings
CAN_RxHeaderTypeDef RxMessage;			    	// Struct for recieved data frame
CAN_TxHeaderTypeDef TxMessage;   				// Struct for transmitted dataframe
uint8_t rxData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };	// Array for recieved data (8 bytes)
uint8_t txData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t usedmailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Ports_Clocks(void);
void Timers_Init(void);
void UART_Initialize(void);
void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y);
void MotorCode(int x, int y);
int map(float k, float l, float h, float L, float H);
int adjust_val(int m);
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

	int mapped_x, mapped_y;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Ports_Clocks();
  Timers_Init();
  UART_Initialize();
  TxMessage.IDE = CAN_ID_STD;				// Standard identifier format (11bit)
  TxMessage.StdId = 0x211;					// Identifier value
  TxMessage.RTR = CAN_RTR_DATA;				// Indicates frame mode (data frame or remote frame)
  TxMessage.DLC = 8;						// Data length (8 bytes)
  TxMessage.TransmitGlobalTime = DISABLE;	// Time of transmission is not transmitted along with the data

  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;	// Filter bank consists of 2 32bit values (mask and ID)
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;		// Filter set to mask and ID mode
  sFilterConfig.FilterBank = 0;							// Filter bank number 0 selected
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;	// Assign filter bank to FIFO 0
  sFilterConfig.FilterIdHigh = 0x446 << 5;				// STD ID value is 7, here shifted by 5 because 11 bits starting from the left are for STD ID (FilterIdHigh is 16bit)
  sFilterConfig.FilterIdLow = 0;						// LSB
  sFilterConfig.FilterMaskIdHigh = 0x446 << 5;			// 0b111 shifted by 5 for the same reason, first 11 bits are for Identifier
  sFilterConfig.FilterMaskIdLow = 0;					// LSB
  sFilterConfig.FilterActivation = ENABLE;				// Activate filter

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);	// Commits filter settings
  HAL_CAN_Start(&hcan);							// Start the CAN peripherals


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) 					// Checks if the number of messages in FIFO 0 is non zero
	{
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxMessage, rxData);  // Stores the data frame in RxMessage struct, stores data in rsData array

		//TIM4->CCR1 = rxData[0];

		mapped_x = map(rxData[0], 0, 255, -4095, 4095);
		mapped_y = map(rxData[1], 0, 255, 4095, -4095);

		//mapped_x = adjust_val(mapped_x);
		//mapped_y = adjust_val(mapped_y);

		MotorCode(mapped_x, mapped_y);

		//TIM4->CCR1 = mapped_x;

		/*
		if(rxData[2] == 1)
		{
			for(int i = 0; i<4; i++)
			{
				GPIOC -> ODR |= 0x2000;
				HAL_Delay(500);
				GPIOC -> ODR &= ~0x2000;
				HAL_Delay(500);
				rxData[2] = 0;
			}
		}
		*/
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


void MotorCode(int x, int y)
{


	// STOP
	if ((abs(x) < 500 && abs(y) < 500))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = 0;   // LEFT SPEED
	  TIM4->CCR2 = 0;   // RIGHT SPEED
	}


	// FORWARD MAX
	else if(abs(x) < 10 && y > 0)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y)*0.5 ;  // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(y)*0.5;   // RIGHT SPEED

	}

	// BACKWARD MAX
	else if(abs(x) < 10 && y < 0)
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(y))*0.5;  // LEFT SPEED
	  TIM4->CCR2 = (uint32_t)  abs(y)*0.5;   // RIGHT SPEED
	}

	// SPOT LEFT
	else if (x < 0 && abs(y) <= 10)
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x)*0.5;   // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(x)*0.5;    // RIGHT SPEED
	}

	// SPOT RIGHT
	else if (x > 0 && abs(y) <= 10)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x)*0.5;		// LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(x)*0.5;   // RIGHT SPEED
	}

	// OCTET 1
	else if(x > 0 && y > 0 && x > y)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
	  TIM4->CCR1 = (uint32_t) abs(x)*0.5;                 // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // RIGHT SPEED
	}

	// OCTET 2
	else if(x > 0 && y > 0 && x < y)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y*0.5);                // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // RIGHT SPEED
	}

	// OCTET 3
	else if(x < 0 && y > 0 && abs(x) < y)
	{
	  GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(y)*0.5;                  // RIGHT SPEED
	}

  // OCTET 4
	else if(x < 0 && y > 0 && abs(x) >= y)
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(x)*0.5;                  // RIGHT SPEED
	}

  // OCTET 5
	else if(x < 0 && y < 0 && abs(x) > abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(x)*0.5;                // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // RIGHT SPEED
	}

	// OCTET 6
	else if(x < 0 && y < 0 && abs(x) < abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(y)*0.5;                // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // RIGHT SPEED
	}

	// OCTET 7
	else if(x > 0 && y < 0 && abs(x) < abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(y)*0.5;                  // RIGHT SPEED
	}

	// OCTET 8
	else if(x > 0 && y < 0 && abs(x) > abs(y))
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) abs(abs(x) - abs(y))*0.5;   // LEFT SPEED
	  TIM4->CCR2 = (uint32_t) abs(x)*0.5;                  // RIGHT SPEED
	}
}

// MAP VALUES
int map(float k, float l, float h, float L, float H)
{
	return ((k - l) / (h - l)) * (H - L) + L;
}

void UART_Initialize()
{
	/*
    	//PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 4<<4 | 5<<0;   //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      Tx Enable	  UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
	*/


	// CONFIGURE UART | TX -> A9 | RX -> A10
	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN;

	// RX - A10 | FLOATING INPUT
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA -> CRH |= GPIO_CRH_CNF10_0;

	// TX - A9 | ALTERNATE OUTPUT PUSH-PULL
	GPIOA -> CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA -> CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;

	// BAUD RATE
	USART1 -> BRR |= 0x271; // 115200

	// CONTROL REGISTER | USART ENABLE | TRANSMIT ENABLE | RECEIVE ENABLE
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

}

int adjust_val(int m)
{
	//  BUFFERS FOR JOYSTICK
	if (m < -3900)
	{
		m = -4095;
	}

	else if (m > 3900)
	{
		m = 4095;
	}

	else if (abs(m) < 1000)
	{
		m = 0;
	}

	return m;
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
