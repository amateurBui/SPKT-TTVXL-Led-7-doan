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
void Clock_Init(void);
void Relay();
void delay(uint32_t dlyTicks);
uint8_t state = 0;
uint32_t count;
char data_rx;
uint8_t previous_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void send_char (char data){
	while ((USART1->SR &(1<<6)) == 0);
	USART1->DR = data;
}
void send_string (char *str)
{while (*str) send_char(*str++);}
int State =1;
int out;
int i=0;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
Clock_Init();
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
  while (1)
  {
    /* USER CODE END WHILE */
	  Relay();
	    if ((GPIOA->IDR & (1<<2)) == 0)
	    {

	  uint8_t current_state = (GPIOB->ODR & (1<<6)) != 0;
	  if (current_state && !previous_state){
	      count++;
	      char buffer[50];
	      sprintf(buffer, "Count: %lu\r\n", count);
	      send_string(buffer);
	  }
	  previous_state = current_state;
	    }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void Clock_Init(void){
	RCC->APB2ENR |=(1<<2)|(1<<3)|(1<<6)|(1<<5)|(1<<4);
	GPIOA->CRH |=(1<<0); //DEN1
	GPIOC->CRH |=(1<<4); //DEN2
	GPIOD->CRH |=(1<<12); //RELAY Y4 QUA TRAI
	GPIOC->CRL |=(1<<24); //RELAY Y3 QUA PHAI
	GPIOB->CRL |=(1<<28); //RELAY Y2 DI XUONG
	GPIOB->CRL |=(1<<24); //RELAY Y1 KEP
	GPIOE->CRH &=~(1<<20); //INPUT NUR NHAN E13
	GPIOE->CRH &=~(1<<16); //INPUT NUT NHAN E12
	GPIOE->CRH &=~(1<<8); //INPUT NUT NHAN E10
	GPIOE->CRH &=~(1<<4); //INPUT SWITCH 1
	GPIOE->CRH &=~(1<<0); //INPUT NUT NHAN E8
	GPIOB->CRL &=~(1<<8); //INPUT SWITCH 2
	GPIOC->CRL &=~(1<<16); //SENSOR S1
	GPIOA->CRL &=~(1<<28); //SENSOR S2
	GPIOA->CRL &=~(1<<24); //SENSOR S3
	GPIOA->CRL &=~(1<<8); //SENSOR S4
	GPIOA->CRL &=~(1<<0); //SENSOR S0
	GPIOA->ODR &=~(1<<8);
	GPIOC->ODR &=~(1<<9);
	//USART
	RCC->APB2ENR |= (1<<14); // clock for USART 1
	GPIOA -> CRH |= (9<<4); // PA9 TX, Output AF mode
	GPIOA-> CRH |= (8<<8); // PA10 RX, Input floating mode
	USART1->BRR = (52 << 4) | (1<<0); // baudrate 9600
	USART1->CR1 = (1<<2)|(1<<3)|(1<<13); // enable TX, RX, USART
	USART1-> CR1 |= (1<<5); //enable RX interrupt
	NVIC-> ISER[1] = (1<<5); //enable global interrupt USART1
}
void delay_ms(uint32_t ms)
{
    uint32_t count = 16000 * ms;
    while (count--) { __asm__("nop"); }
}
void Relay(){
	if ((GPIOE->IDR &(1<<9))==0){
		State=0;
	}
	if ((GPIOB->IDR &(1<<2))==0){
		State=1;
	}
	if (State == 0){	//Manual
			GPIOA->ODR &=~(1<<8); //TURN ON LED 1
			GPIOC->ODR |=(1<<9); //TURN OFF LED 2
			GPIOB->ODR |=(1<<6)|(1<<7);
			GPIOC->ODR |=(1<<6);
			GPIOD->ODR |=(1<<11);
			if((GPIOE->IDR & (1<<13))==0){
				GPIOC->ODR &=~(1<<6);
			}
			else if((GPIOE->IDR & (1<<12))==0){
				GPIOB->ODR &=~(1<<6);
			}
			else if((GPIOE->IDR & (1<<10))==0){
				GPIOB->ODR |= (1<<6);
			}
			else if ((GPIOE->IDR & (1<<8))==0){
				GPIOD->ODR &=~(1<<11);
			}
	}
	if (State ==1){	//AUTO
		GPIOA->ODR |=(1<<8); //TURN ON LED 1
		GPIOC->ODR &=~(1<<9); //TURN OFF LED 2
	    switch (state)
	    {
	        case 0:
	            GPIOD->ODR &= ~(1<<11);
//	            GPIOB->ODR |=(1<<6);
	            if ((GPIOE->IDR & (1<<13)) == 0)
	            {
	                GPIOD->ODR |= (1<<11);
	                state = 1;
	            }
	            break;

	        case 1:
	            GPIOC->ODR &= ~(1<<6);
	            if ((GPIOA->IDR & (1<<2)) == 0)
	            {
	                GPIOC->ODR |= (1<<6);
	                delay_ms(100);
	                state = 2;
	            }
	            break;

	        case 2:
	            GPIOB->ODR &= ~(1<<7);
	            delay_ms(100);
	            state = 3;
	            break;

	        case 3:
	            GPIOB->ODR &= ~(1<<6);
	            delay_ms(100);
	            state = 4;
	            break;

	        case 4:
	            GPIOB->ODR |= (1<<7);
	            delay_ms(100);
	            GPIOE->ODR %=~ (1<<7);
	            state = 0;
	            break;
	        default:
	            state = 0;
	            break;
	    }

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
