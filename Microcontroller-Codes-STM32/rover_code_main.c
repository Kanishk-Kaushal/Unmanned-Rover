#include "stm32f1xx.h"
#include "stdlib.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

uint16_t pwm = 5500;
//MACROS
#define stop_swivel TIM2->CCR1 = 0;
#define move_swivel TIM2->CCR1 = pwm;

#define move_link1(x) TIM2->CCR2 = (x);
#define move_link2(x) TIM2->CCR3 = (x);

#define stop_roll TIM2->CCR4 = 0;
#define move_roll TIM2->CCR4 = pwm;

#define stop_pitch TIM3->CCR2 = 0;
#define move_pitch TIM3->CCR2 = pwm;

#define stop_gripper TIM3->CCR3 = 0;
#define move_gripper TIM3->CCR3 = pwm;

void GPIO_Initialize() {
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //Enable Alternate Function

	//MOTOR
	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH |= GPIO_CRH_CNF10;   //Input with pull-up/pull-down (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);

	//PB4 Setup:
	//Disable SWD & JTAG:
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);
	GPIOB->CRL |= (GPIO_CRL_MODE4);   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR = 1 << (4);   //Mandatory turn off

	//1)
	//PB3 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE3;	//OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF3); //Output Push-Pull (00)
	GPIOB->BRR |= 1 << 3;		//Mandatory disable to turn it off initially.
	//Now PB4 can be used as a normal GPIO

	//PA0 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE0;  //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF0_1; //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0);

	//2)
	//PB4 Setup: (DIR)
	/*PB4 is by default initialized as a JTRST Pin and thus has to be disabled.
	 By default the pin is at a high state and doesnt function as a normal GPIO
	 Read AFIO debug configeration in the reference manual for more data on how to disable JTRST*/
	GPIOB->CRL |= GPIO_CRL_MODE4;	//OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4); //Output Push-Pull (00)
	GPIOB->BRR |= 1 << 4;		//Mandatory disable to turn it off initially.
	//Now PB4 can be used as a normal GPIO

	//PA1 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE1;  //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF1_1; //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0);

	//3)
	//PB5 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE5;	//OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF5); //Output Push-Pull (00)

	//PA2 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE2;  //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF2_1; //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0);

	//4)
	//PA15 Setup: (DIR)
	GPIOA->CRH |= GPIO_CRH_MODE15;	 //OUTPUT Mode (11)
	GPIOA->CRH &= ~(GPIO_CRH_CNF15); //Output Push-Pull (00)

	//PA3 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE3;  //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF3_1; //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF3_0);

	//5)
	//PB2 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE2;	//OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF2); //Output Push-Pull (00)

	//PB0 Setup: (PWM)
	GPIOB->CRL |= GPIO_CRL_MODE0;  //OUTPUT Mode (11)
	GPIOB->CRL |= GPIO_CRL_CNF0_1; //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF0_0);

	//6)
	//PB10 Setup: (DIR)
	GPIOB->CRH |= GPIO_CRH_MODE10;	 //OUTPUT Mode (11)
	GPIOB->CRH &= ~(GPIO_CRH_CNF10); //Output Push-Pull (00)

	//PB1 Setup: (PWM)
	GPIOB->CRL |= GPIO_CRL_MODE1;  //OUTPUT Mode (11)
	GPIOB->CRL |= GPIO_CRL_CNF1_1; //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF1_0);

	//7)
	//PB11 Setup: (DIR)
	GPIOB->CRH |= GPIO_CRH_MODE11;	 //OUTPUT Mode (11)
	GPIOB->CRH &= ~(GPIO_CRH_CNF11); //Output Push-Pull (00)

	//PA7 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE7;  //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF7_1; //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF7_0);
}

void Timer_Initialize() {
	//MOTOR
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;   //freq/1 = 8 Mhz
	TIM4->ARR = 8000;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting

	//Timer 2: PA0
	//TIMER 2 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Timer 2 Enable
	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; //Enable Channel 1,2,3,4 as OUTPUT
	TIM2->CR1 |= TIM_CR1_ARPE;			//Enable Auto Re-Load Preload (ARPE)

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //Enable Preload for Channel 1
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; //Enable Preload for Channel 2
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE; //Enable Preload for Channel 3
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE; //Enable Preload for Channel 4

	//PWM Mode 1 for Channel 1:
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_0);

	//PWM Mode 1 for Channel 2:
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC4M_0);

	TIM2->PSC = 1;	  //freq/1 = 72 Mhz
	TIM2->ARR = 8000; //16 Bit value

	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	TIM2->EGR |= TIM_EGR_UG;  //Update Registers
	TIM2->CR1 |= TIM_CR1_CEN; //Start Counting

	//TIMER 3 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   //Timer 3 Enable
	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; //Enable Channel 1,2,3,4 as OUTPUT

	TIM3->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable Preload for Channel 3
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;   //Enable Preload for Channel 4

	//PWM Mode 1 for Channel 1:
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM3->CCMR2 &= ~(TIM_CCMR2_OC4M_0);

	TIM3->PSC = 1;   //freq/1 = 72 Mhz
	TIM3->ARR = 8000;   //16 Bit value
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->EGR |= TIM_EGR_UG;   //Update Registers
	TIM3->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear) {
	if (DL == 1)
		GPIOA->BSRR |= 1 << 4;   //Turn on LEFT LED
	else
		GPIOA->BRR |= 1 << 4;   //Turn off LEFT LED

	if (DR == 1)
		GPIOA->BSRR |= 1 << 5;   //Turn on RIGHT LED
	else
		GPIOA->BRR |= 1 << 5;   //Turn off RIGHT LED

	TIM4->CCR1 = (uint32_t) abs(abs(a * X) - abs(b * Y)) * (gear * 0.1); //Left PWM
	TIM4->CCR2 = (uint32_t) abs(abs(p * X) - abs(q * Y)) * (gear * 0.1); //Right PWM
}

void MotorCode(int x, int y, float g) {
//void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, int g)

	if (abs(x) < 20 && abs(y) < 20)   //No Motion
		Drive(0, 0, 0, 0, 0, 0, 0, 0, 0);

	else if (abs(x) < 10 && y < 0)   //Full Backward
		Drive(0, 0, 0, 1, 0, 1, x, y, g);

	else if (abs(x) < 10 && y > 0)   //Full Forward
		Drive(1, 1, 0, 1, 0, 1, x, y, g);

	else if (x < 0 && abs(y) <= 10)   //Spot Turn Left
		Drive(0, 1, 1, 0, 1, 0, x, y, g);

	else if (x > 0 && abs(y) <= 10)   //Spot Turn Right
		Drive(1, 0, 1, 0, 1, 0, x, y, g);

	else if (x > 0 && y > 0 && x > y)   //Octet 1
		Drive(1, 0, 1, 0, 1, 1, x, y, g);

	else if (x > 0 && y > 0 && x < y)   //Octet 2
		Drive(1, 1, 0, 1, 1, 1, x, y, g);

	else if (x < 0 && y > 0 && abs(x) < y)   //Octet 3
		Drive(1, 1, 1, 1, 0, 1, x, y, g);

	else if (x < 0 && y > 0 && abs(x) >= y)   //Octet 4
		Drive(0, 1, 1, 1, 1, 0, x, y, g);

	else if (x < 0 && y < 0 && abs(x) > abs(y))   //Octet 5
		Drive(0, 1, 1, 0, 1, 1, x, y, g);

	else if (x < 0 && y < 0 && abs(x) < abs(y))   //Octet 6
		Drive(0, 0, 0, 1, 1, 1, x, y, g);

	else if (x > 0 && y < 0 && abs(x) < abs(y))   //Octet 7
		Drive(0, 0, 1, 1, 0, 1, x, y, g);

	else if (x > 0 && y < 0 && abs(x) > abs(y))   //Octet 8
		Drive(1, 0, 1, 1, 1, 0, x, y, g);

	//Test Drive:
	//Drive(1,1,1,0,0,1,x,y,g);
}
void UART_Initilaize() {
	//PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 0x45; //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      Tx Enable  UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
}

uint8_t getuval() //Reads UART Values
{
	uint8_t data;
	while (!(USART1->SR & USART_SR_RXNE))
		; //Check Status Register if all is Recieved
	data = USART1->DR;
	return data;
}
int Adjust(int k) {
	k = k - 8000;
	if (abs(k) < 500)
		k = 0;
	return k;
}
int main() {
	GPIO_Initialize();
	Timer_Initialize();
	UART_Initilaize();
	uint8_t mode = 0;
	int x = 0, y = 0;
	float gear = 1.0;
	int swivel = 0, link1 = 0;
	int link2 = 0, roll = 0;
	int gripper = 0, pitch = 0;
	int trash = 0;
	//float gear = 1.0;
	while (1) {
		//Read LAN2UART Values

		mode = getuval();
		if (mode == 'a') {
			if (getuval() == 's')		//SWIVEL
					{
				swivel = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //x1 value
				if (swivel == 0)
					stop_swivel

				else if (swivel == 16000) {
					GPIOB->BSRR = 1 << 5;
					move_swivel
				}

				else if (swivel == 16001) {
					GPIOB->BSRR = 1 << (5 + 16);
					move_swivel
				}
			}
			if (getuval() == 'o')		//LINK1
					{
				link1 = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //y1 value

				link1 = Adjust(link1);
				if (link1 < 0)
					GPIOB->BSRR = 1 << 4;
				else
					GPIOB->BSRR = 1 << (4 + 16);
				move_link1(abs(link1))
			}
			if (getuval() == 't')		//LINK2
					{
				link2 = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //x2 value
				link2 = Adjust(link2);
				if (link2 < 0)
					GPIOB->BRR |= 1 << (3);
				else
					GPIOB->ODR |= 1 << 3;

				move_link2(abs(link2))
			}
			if (getuval() == 'r')		//ROLL
					{
				roll = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //y2 value
				if (roll == 0)
					stop_roll

				else if (roll == 16000) {
					GPIOA->BSRR = 1 << (15 + 16);
					move_roll
				}

				else if (roll == 16001) {
					GPIOA->BSRR = 1 << 15;
					move_roll
				}
			}
			if (getuval() == 'p')		//PITCH
					{
				pitch = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //Left Trigger

				if (pitch == 0)
					stop_pitch

				else if (pitch == 16000) {
					GPIOB->BSRR = 1 << (10 + 16);
					move_pitch
				} else if (pitch == 16001) {
					GPIOB->BSRR = 1 << 10;
					move_pitch
				}
			}
			if (getuval() == 'g')		//GRIPPER
					{
				gripper = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0'); //Right Trigger
				if (gripper == 0)
					stop_gripper

				else if (gripper == 16000) {
					GPIOB->BSRR = 1 << (11 + 16);
					move_gripper
				} else if (gripper == 16001) {
					GPIOB->BSRR = 1 << 11;
					move_gripper
				}
			}

			//ALLEN HEAD TBD
		} else if (mode == 'm') {
			gear = (int) ((getuval() - '0') + 1);   //Get gear value
			if (getuval() == 's') {
				x = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0');   //x value
			}
			if (getuval() == 'f') {
				y = (getuval() - '0') * 10000 + (getuval() - '0') * 1000
						+ (getuval() - '0') * 100 + (getuval() - '0') * 10
						+ (getuval() - '0');   //y value
			}
			trash = getuval(); //This is actually Mast CAM values but we're ignoring it for now
			x = x - 8000;
			y = y - 8000;

			if (abs(x) < 500)
				x = 0;
			if (abs(y) < 500)
				y = 0;

			MotorCode(x, y, gear);   //Run MotorCode
		}

		else {
			trash = trash + 1 - 1;   //Bakchodi
		}

	}

} 							//LINK1   //LINK2
//Data Packets: a s <16001> o <16000> t <16000> r <16001> p <16001> g <16001>
