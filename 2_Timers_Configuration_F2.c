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
#include "stdlib.h"
#include "stdio.h"
#include <stdint.h>
#include <string.h>
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


//them timer 3
volatile int timer3_counter = 0;
volatile uint8_t buzzer_state = 0;
volatile uint32_t display_value = 0;
static void MX_TIM3_Init(void);
void TIM3_IRQHandler(void);

/* USER CODE BEGIN PFP */
void send_char (char data){  
	while ((USART1->SR &(1<<6)) == 0); 
	USART1->DR = data; 
}
void send_string (char *str) 
{while (*str) send_char(*str++);} 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t led_pos[5]={0x08,0x10,0x20,0x40,0x80};
uint8_t led_num[11]={0xAF,0xA0,0x6E,0xEA,0xE1,0xCB,0xCF,0xA2,0xEF,0xEB,0x40};
volatile char received_chars[4]; // Array to store received characters (3 characters + null terminator)
volatile uint8_t char_index = 0; // Index to track position in received_chars array
uint8_t disp[5];
volatile int count = 0;
unsigned int x = 0;
float t=0;
int value = 0;
int sec = 0;
char buffer[50];
uint8_t count_rx=0;
uint8_t phut= 0;
uint8_t giay= 0;
int mode= 0;
unsigned char two_dot = 0x40;
int count_2 = 0;
int mili_sec = 0;
int bt0_state = 0; //Button 0 state
int bt5_state = 0; //Button 5 state
int bt6_state = 0; //Button 6 state
int bt7_state = 0; //Button 7 state

//int out;
unsigned char data165 = 0xFF;
char data_rx[3];

void delay_ms(float ms){
		volatile float count_1;
		for(uint32_t ms = 0; ms >0 ; ms ++) {
			for(count_1 = 0;count_1<1000;count_1++) {}}
}		

void clock(void) {

    GPIOB->BSRR |= (1 << 3);  // Set PB3
    GPIOB->BSRR |= (1 << (3+16)); // Reset PB3
	delay_ms(0.1);
}
	
void read_74HC165_Data(void)
{
	//PE1 is High by default
	GPIOE->ODR &= ~(1<<1);
	GPIOE->ODR |= (1<<1);
	for(int i = 0; i<8; i++)
	{
		if ((GPIOB->IDR & (1<<4))==0)
			data165 &= ~(1<<(7-i));
		else
			data165 |= (1<<(7-i));
		clock();
	}
}

void data_74_decode(unsigned char data)
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
		clock();
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
void displayLED(uint32_t numbers)
{
	disp[4] = numbers / 10000;
	disp[3] = numbers%10000/1000;
	disp[2] = numbers%1000/100;
	disp[1] = numbers%100/10;
	disp[0] = numbers%10;
	for(int i=0; i<5; i++)
	{
		data_74_decode(led_pos[4-i]);
		data_74_decode(led_num[disp[i]]);
		GPIOE->ODR |= (1<<0);
		GPIOE->ODR &= ~(1<<0);
	}
	
}
void displayLED1(uint32_t numbers)
{
	phut = numbers/60;
	giay = numbers%60;
	disp[4] = phut/10;
	disp[3] = phut%10;
	disp[2] = 10;
	disp[1] = giay/10;
	disp[0] = giay%10;
	for(int i=0; i<5; i++)
	{
		data_74_decode(led_pos[4-i]);
		data_74_decode(led_num[disp[i]]);
		GPIOE->ODR |= (1<<0);
		GPIOE->ODR &= ~(1<<0);
	}
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
  /* USER CODE BEGIN 2 */
	RCC->APB2ENR |= (1<<0)|(1<<3)|(1<<6); // clock for GPIOA
	RCC->APB2ENR |= (1<<14); // clock for USART 1
	GPIOA->CRH = 0;
	RCC->APB1ENR |= (1<<0)|(1<<1);
	GPIOB->CRL = 0x44242444; //Set PB3 and PB5 as output
	GPIOB->CRH = 0;
	GPIOB->CRH = (1<<8); //output mode for PB10
	GPIOB -> CRH |= (1<<12); // output PP for PB11
	GPIOB->ODR = 0xFFFF; //Reset all LEDs
	GPIOE->CRL = 0x44442222; //Enable E0->E3 as output
	GPIOE->ODR &= ~(1<<3);
	GPIOE->ODR &= ~(1<<2);
	TIM2-> PSC = 3199; //1s
	TIM2-> ARR = 2499;
	TIM2-> DIER |= (1<<0);
	NVIC->ISER[0] |= (1<<28);
	GPIOA -> CRH |= (9<<4); // PA9 TX, Output AF mode
	GPIOA-> CRH |= (8<<8); // PA10 RX, Input floating mode
	USART1->BRR = (52 << 4) | (1<<0); // baudrate 9600
	USART1->CR1 = (1<<2)|(1<<3)|(1<<13); // enable TX, RX, USART
	USART1-> CR1 |= (1<<5); //enable RX interrupt
	NVIC-> ISER[1] = (1<<5); //enable global interrupt USART1
	
	MX_TIM3_Init();
	//Start TIMER3
	TIM3 -> CR1 |= TIM_CR1_CEN;
  while (1)
  {
    /* USER CODE END WHILE */
		read_74HC165_Data();
         //e.g: 0b1001 1110 
         //Button 0
         if(((data165 & (1<<0))==0) && (bt0_state == 0))
         {
            value += 1; 
            sprintf(buffer,"Number: %d\n",value);
            USART1_sendString(buffer);
            bt0_state = 1;
         }
         else if (((data165 & (1<<0))!=0) && (bt0_state == 1))
         {

            bt0_state = 0;
         }
         //Button 5
         if(((data165 & (1<<5))==0) && (bt5_state == 0))
         {
            value -= 1;
            sprintf(buffer,"Number: %d\n",value);
            USART1_sendString(buffer);
            bt5_state = 1;
         }
         else if (((data165 & (1<<5))!=0) && (bt5_state == 1))
         {
            bt5_state = 0;
         }
         //Button 6
         if(((data165 & (1<<6))==0) && (bt6_state == 0))
         {
            bt6_state = 1;
					
         }
         else if (((data165 & (1<<6))!=0) && (bt6_state == 1))
         {
            bt6_state = 0;
         }
         //Button 7 
         if(((data165 & (1<<7))==0) && (bt7_state == 0))
         {
            bt7_state = 1;
         }
         else if (((data165 & (1<<7))!=0) && (bt7_state == 1))
         {
            bt7_state = 0;
         }
				 
				 if (mode == 1){
						displayLED1(value);
				 }
				 else if (mode == 0){
						displayLED(value);
						if (value > 9){
							GPIOE->ODR |= (1<<3); ////////////////////////////////////////////////PE3 or PE4
						}
						if (value > 99){
								value = 0;
						}
				 }
//		out = atoi(data_rx);
//		displayLED(out);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

static void MX_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Enable TIMER3 clock

    TIM3->PSC = 7999; // Prescaler to make 1ms tick (assuming 8MHz clock)
    TIM3->ARR = 999; // Auto-reload value for 1000ms period

    TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_EnableIRQ(TIM3_IRQn); // Enable TIMER3 interrupt in NVIC

    TIM3->CR1 |= TIM_CR1_CEN; // Enable TIMER3
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIOE->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // Clear mode and config
    GPIOE->CRL |= GPIO_CRL_MODE3_1; // Set PE3 as output, 2MHz
}

void TIM3_IRQHandler(void) {
    // Check if update interrupt flag is set
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Clear the flag

        if (mode == 0) { // Mode D (counting 0-99)
            display_value++;
            if (display_value > 99) {
                display_value = 0;
            }
            displayLED(display_value);

            timer3_counter++;
            if (timer3_counter >= 10) { // 10 seconds have passed
                timer3_counter = 0;
                buzzer_state = 1; // Activate buzzer
                GPIOE->ODR |= (1 << 3); // Turn on buzzer
            }
        }

        if (buzzer_state) {
            delay_ms(500); // Keep buzzer on for 500ms
            GPIOE->ODR &= ~(1 << 3); // Turn off buzzer
            buzzer_state = 0;
        }
    }
}



void TIM2_IRQHandler(void)
{
	// Clear interupt flag
	TIM2->SR &= ~(1<<0);
	// Do something
	if (mode == 1){
		sec++;
		if(sec >= 0){
			value++;
			sec = 0;
		}
	}
	else if (mode == 0){
		sec++;
		if(sec >= 0){
			value++;
			sec = 0;
		}
	}
}
int i=0;
void USART1_IRQHandler(void) {
    // Define the variable to hold the received character
//   data_rx[i]= USART1-> DR;
//	 i++;
//   if(i==3){i=0;}
	if(USART1->DR == ';')
	{
		if (strcmp(data_rx, "H") == 0)
		{
			mode = 1;
			TIM2->CR1 |= (1<<0);
		}
		else if (strcmp(data_rx, "D") == 0)
		{
			mode = 0;
			TIM2->CR1 |= (1<<0);
		}
        sprintf(buffer, "Received command: %s \n", data_rx);
        USART1_sendString(buffer);
        count_rx = 0;
	}
	else
	{
		data_rx[count_rx] = USART1->DR;
		count_rx++;

	}
    
}

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

  /* GPIO Ports Clock 
	Enable */
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
