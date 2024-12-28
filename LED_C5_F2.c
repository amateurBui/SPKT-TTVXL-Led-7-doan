// LED C5 - OK
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Clock_Init(void);
void sendPulse(void);
void data_74_decode(unsigned char data);
void displayLED(uint32_t numbers);
void delay(uint16_t time);
void read_74HC165_Data(void);
void USART1_init(void);
void USART1_sendChar(char data);
void USART1_sendString(char *string);
void TIM2_Init(void);
void TIM2_IRQHandler(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
unsigned char position[5] = {0x08, 0x10, 0x20, 0x40, 0x80};  //Tuong ung voi led so 1, so 2, so 3, so 4, so 5 (trai sang phai)
unsigned char number[10] = {0xAF, 0xA0, 0x6E, 0xEA, 0xE1, 0xCB, 0xCF, 0xA2, 0xEF, 0xEB};  //0 1 2 3 4 5 6 7 8 9
unsigned char data165 = 0xFF; 
int count = 0;
int bt0_state = 0; //Button 0 state
int bt5_state = 0; //Button 5 state
int bt6_state = 0; //Button 6 state
int bt7_state = 0; //Button 7 state
char buffer[50];
char data_rx[2];
uint8_t count_rx = 0;
int value = 0;
int mili_sec = 0;


int value1 = 0;
int value2 = 0;
int value3 = 0;

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	Clock_Init();
  USART1_init();
	TIM2_Init();
  USART1_sendString("Program started!!!\n");
	while (1)
		{
			read_74HC165_Data();
			//e.g: 0b1001 1110 
			//Button 0
			if(((data165 & (1<<0))==0) && (bt0_state == 0))
			{
				value1 = value1 + 1;
				bt0_state = 1;
				if (value1 > 9){
					value1 = 0;
				}
			}
			else if (((data165 & (1<<0))!=0) && (bt0_state == 1))
			{
				bt0_state = 0;
			}
			
			//Button 5
			if(((data165 & (1<<5))==0) && (bt5_state == 0))
			{
				value2 = value2 +1;
				bt5_state = 1;
				if (value2 > 9){
					value2 = 0;
				}
			}
			else if (((data165 & (1<<5))!=0) && (bt5_state == 1))
			{
				bt5_state = 0;
			}
			//Button 6
			if(((data165 & (1<<6))==0) && (bt6_state == 0))
			{
				value3 = value3 +1;
				bt5_state = 1;
				if (value3 > 9){
					value3 = 0;
				}
			}
			else if (((data165 & (1<<6))!=0) && (bt6_state == 1))
			{
				bt6_state = 0;
			}
			count = value3*100+value2*10+value1;
			//Button 7 
			if(((data165 & (1<<7))==0) && (bt7_state == 0))
			{
				sprintf(buffer, "display: %d\n", count);
				USART1_sendString(buffer);
        count_rx = 0;
				bt6_state = 1;
				// Enable counter
				TIM2->CR1 |= (1<<0);
			}
			else if (((data165 & (1<<7))!=0) && (bt7_state == 1))
			{
				bt7_state = 0;
			}			
			displayLED(count);// displayLED(count) display 5
			/* USER CODE BEGIN 3 */
		}
  /* USER CODE END 3 */
}
void Clock_Init(void)
{
	RCC->AHB1ENR |= (1<<1) | (1<<4); //Enable GPIOB and GPIOE
	GPIOB->MODER &= ~(3<<6); 
//	GPIOB->MODER &= ~(3<<10);
//	GPIOB->MODER &= ~(3<<20);
	GPIOB->MODER |= (1<<20) | (1<<10) | (1<<6);  //Enable B3, B5 and B10 is output
	GPIOB->MODER &= ~(3<<8); //Enable B4 as input 
	//GPIOB->OTYPER = 0; //Push-pull mode 
	GPIOB->OSPEEDR = 0; //Low speed 
	//GPIOB->PUPDR = 0; //No pull up, pull down 
	GPIOB->ODR = 0xFFFF; //Reset all LEDs
	
//	GPIOE->MODER &= ~(3<<0);
//	GPIOE->MODER &= ~(3<<2);
	GPIOE->MODER |= (1<<4) | (1<<2) |(1<<0); //Enable E0 and E2
//	GPIOE->OTYPER = 0 ; //Push-pull mode 
	GPIOE->OSPEEDR = 0; //Low speed 
	//GPIOE->PUPDR = 0; //No pull up, pull down 	
	GPIOE->ODR &= ~(1<<2); 
}
//Function to make PB3 send a pulse
void PB3sendPulse(void)/////////////////////////////////////////////////////////////////////////////////////////////////////////////// giu nguyen
{
	GPIOB->ODR |= (1<<3);
	GPIOB->ODR &= ~(1<<3);
}
//Send data to 74HC594 decoder
void data_74_decode(unsigned char data)/////////////////////////////////////////////////////////////////////////////////////////////// giu nguyen
{
	//SET Data (DS) mode 
	for(int i = 0; i<8; i++) //From 0 to 7
	{
		if ((data & (1<<(7-i))) == 0) //Check from 7th to 0th. If it = 0 => DS(PB5) set to low
		{
			GPIOB->ODR &= ~(1<<5); //Low
		}
		else //Check from 7th to 0th. If it = 1 => DS(PB5) set to High
		{
			GPIOB->ODR |= (1<<5);//High
		}
		//Send a Pulse
		PB3sendPulse();
	}
}

void displayLED(uint32_t numbers)//////////////////////////////////////////////////////////////////////////////////////////////////////// giu nguyen
{
	uint8_t disp[5];
	disp[4] = numbers / 100;
	disp[3] = 0xff;
	disp[2] = numbers%100/10;
	disp[1] = 0xff;
	disp[0] = numbers%10;
	for(int i=0; i<5; i++)
	{
		data_74_decode(position[4-i]);// i sang led tu trai -> phai || 4-i sang led tu phai -> trai
		data_74_decode(number[disp[i]]);// i sang led tu hang don vi -> hang chuc ngan
		GPIOE->ODR |= (1<<0);// RCK On (PE0)
		GPIOE->ODR &= ~(1<<0);
	}
}

void USART1_init(void)
{
   RCC->APB2ENR |= (1<<4); //Enable UART1
   GPIOA->MODER |= (2<<18) | (2<<20); //Set PA9 and PA10 as AF mode
   USART1->BRR = (104<<4)|(3<<0); 
   //In baud rate table for 16 bit, value for 9.6k = 104.1875
   //=> 104<<4 and (0.1875*16)<<0
   GPIOA->AFR[1] |= (7<<4) | (7<<8); //Set PA9 and PA10 AF
   USART1->CR1 |= (1<<13) | (1<<5) | (1<<3) | (1<<2); //Enable USART, RX interrupt, enable TX and RX
   
   NVIC_EnableIRQ(USART1_IRQn); //Enable USART1 NVIC
}

void TIM2_Init (void)
{
	// Enable TIM2
	RCC->APB1ENR |= (1<<0);
	// Set prescaler
	TIM2->PSC = 160-1;
	// Set max value
	TIM2->ARR = 100-1;
	// Reset counter
	TIM2->CNT = 0;
	// Enable interupt
	TIM2->DIER |= (1<<0);
	// Enable globle interupt
	NVIC->ISER[0] |= (1<<28);

}

void TIM2_IRQHandler(void)
{
	// Clear interupt flag
	TIM2->SR &= ~(1<<0);
	// Do something
	mili_sec++;
	if(mili_sec >= count){
		mili_sec = 0;
		GPIOB->ODR ^= (3<<10);
	}
}

void USART1_IRQHandler(void)
{
   if(USART1->DR == ';')
   {
      count_rx=0;
      value = atoi(data_rx);
   }
   else
   {
      data_rx[count_rx] = USART1->DR;
      count_rx++;
   }
}

void USART1_sendChar(char data)
{
   while((USART1->SR & (1<<6))==0); //Wait until previous transmission is completed
   USART1->DR = data;
}
void USART1_sendString(char *string)
{
   while(*string)
   {
      USART1_sendChar(*string++);
   }
}

void delay(uint16_t time)
{
	while(time>0) time--;
}
void read_74HC165_Data(void)//////////////////////////////////////////////////////////////////////////////////////////////////////////////// giu nguyen
{
	//PE1 is High by default (set chan SH/LD len muc cao)
	GPIOE->ODR &= ~(1<<1);
	GPIOE->ODR |= (1<<1);
	for(int i = 0; i<8; i++)// doc 8 bit du lieu tu chan QH (PB4) de hien thi so
	{
		if ((GPIOB->IDR & (1<<4))==0) 
			data165 &= ~(1<<(7-i));// 74HC165 (PB4) doc 8 bit tu trong so cao -> trong so thap (tu bit 7 -> bit 0)
		else 
			data165 |= (1<<(7-i));
		PB3sendPulse();
	}
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
