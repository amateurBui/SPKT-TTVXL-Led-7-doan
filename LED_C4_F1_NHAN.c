#include "main.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void GPIO_init (void);
void IC_clock (void);
void IC_latch (void);
void IC_send_data (uint8_t data);
void IC_show_LED (uint8_t led, uint8_t symbol);
void IC_display_INT (int n);
void IC_get_data (void);
void AFIO_init (void);
void TIM3_init (void);
void TIM2_init (void);
void UART1_init (void);
void UART1_send_string(char *str);

char Tx_Buffer[100];	// UART1
char Rx_Buffer[100];	// UART1
uint8_t Rx_count = 0;	// UART1
uint8_t input_data = 0xFF, bt0_state =	0, bt5_state = 0, bt6_state = 0, bt7_state = 0;	//Buttons
int num = 1;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
	GPIO_init();
	AFIO_init();
	TIM3_init();
	TIM2_init();
	UART1_init();
	
  while (1){		
		IC_get_data(); // 0xE1 --> 1110 0000
		
		if ((input_data & (1<<0)) == 0 && (bt0_state == 0)){ // button 0
			// Do somethings
			num++;
			sprintf(Tx_Buffer, "Value: %d\n", num);
			UART1_send_string(Tx_Buffer);
			bt0_state = 1;
		} else if (((input_data & (1<<0)) != 0) && (bt0_state == 1)){
			bt0_state = 0;
		}
			
		if ((input_data & (1<<5)) == 0 && (bt5_state == 0)){ // button 5
			// Do somethings
			num--;
			sprintf(Tx_Buffer, "Value: %d\n", num);
			UART1_send_string(Tx_Buffer);
			bt5_state = 1;
		} else if (((input_data & (1<<5)) != 0) && (bt5_state == 1)){
			bt5_state = 0;
		}

		if (( input_data & (1<<6)) == 0 && (bt6_state == 0)){ // button 6
			// Do somethings
			num++;
			sprintf(Tx_Buffer, "Value: %d\n", num);
			UART1_send_string(Tx_Buffer);
			bt6_state = 1;
		} else if (((input_data & (1<<6)) != 0) && (bt6_state == 1)){
			bt6_state = 0;
		}
		
		if (( input_data & (1<<7)) == 0 && (bt7_state == 0)){ // button 7
			// Do somethings
			num--;
			sprintf(Tx_Buffer, "Value: %d\n", num);
			UART1_send_string(Tx_Buffer);
			bt7_state = 1;
		} else if (((input_data & (1<<7)) != 0) && (bt7_state == 1)){
			bt7_state = 0;
		}
		
		TIM2->CCR3 = num;
		TIM2->CCR4 = num;
		
		IC_display_INT(num);
	}
}

void UART1_send_string(char *str)
{
	while (*str){
		while (!(USART1->SR & (1 << 7)));
		USART1->DR = *str++;
	}
}

void UART1_init (void)
{
	// Enable clock for USART1
	RCC->APB2ENR |= (1 << 14);
	// Set baud rate to 9600 (assuming APB2 clock is 8 MHz)
	USART1->BRR = (52 << 4) | (1<<0);
	// Enable USART1, Tx and Rx
	USART1->CR1 |= (1 << 3) | (1 << 2);
	// Enable Tx, Rx interrupt
	USART1->CR1 |= (1<<5); 
	NVIC_EnableIRQ(USART1_IRQn);	
	USART1->CR1 |= (1 << 13);
}

void USART1_IRQHandler (void)
{
	// Clear Rx interupt flag
	USART1->SR &= ~(1UL<<5);
	// Get a string of data
	if (USART1->DR != ';'){
		Rx_Buffer[Rx_count] = USART1->DR;
		Rx_count++;
	} else {
		Rx_Buffer[Rx_count] = '\0';
		Rx_count = 0;
		// Estimate Rx data
		sscanf(Rx_Buffer, "%d", &num);
	}
}

void TIM3_init (void) // Interupt every 1s
{
	// Enable TIM3
	RCC->APB1ENR |= (1<<1);
	// Set prescaler
	TIM3->PSC = 800-1;
	// Set max value
	TIM3->ARR = 10000-1;
	// Reset counter
	TIM3->CNT = 0;
	// Enable interupt
	TIM3->DIER |= (1<<0);
	// Enable globle interupt
	NVIC_EnableIRQ(TIM3_IRQn);
	// Enable counter
	TIM3->CR1 |= (1<<0);
}

void TIM3_IRQHandler (void)
{
	// Clear interupt flag
	TIM3->SR &= ~(1<<0);
	// Do Somethings
	num++;
}

void AFIO_init (void)
{
	AFIO->MAPR |= (3<<8);															// Full Remap timer 2 
	RCC->APB2ENR |= (1<<3) | (1<<0);											// CLock GPIOB, AFIO
	GPIOB->CRH &= ~(0xFF<<8) | (0xFF<<12); 								// Clear config on PB10, PB11  
	GPIOB->CRH |= (9<<8) | (9<<12); 						// output PB10, PB11 as AF
}

void TIM2_init (void) // PWM PB10_CH3 and PB11_CH4 100Hz
{
	// Enable clock tim2
	RCC->APB1ENR |= (1<<0);
	// Set prescaler
	TIM2->PSC = 800-1;
	// Set max counting value
	TIM2->ARR = 100-1;
	// Set intial suty cycle of CH3 CH4
	TIM2->CCR3 = 100;
	TIM2->CCR4 = 100;
	// Set CH3 and CH4 as PWM mode1 1
	TIM2->CCMR2 |= (7<<4);
	TIM2->CCMR2 |= (7<<12);
	// Enable CH3, CH4
	TIM2->CCER |= (1<<8) | (1<<12);
	// Enable counter Tim2
	TIM2->CR1 |= (1<<0);
}

void IC_get_data (void)
{
	// Toggle PE1
	GPIOE->ODR |= (1<<1);
	GPIOE->ODR &= ~(1<<1); 
	GPIOE->ODR |= (1<<1);
	// D7 first
	for(char i = 0 ; i < 8; i++){
		if ((GPIOB->IDR & (1UL<< 4)) == 0) // read PB4
			input_data &= ~(1UL<<(7-i));
		else 
			input_data |= (1<<(7-i));
		IC_clock();
	}
}

void IC_display_INT (int n)
{
	uint8_t temp, check = 0;
	uint8_t num[10] = {0xAF, 0xA0, 0x6E, 0xEA, 0xE1, 0xCB, 0xCF, 0xA2, 0xEF, 0xEB};
	for (int i=0; i<5; i++){
		n = n % (int)pow(10, 5-i);
		temp = n / (int)pow(10, 4-i);
		if(temp) check = 1;
		if(check)
			IC_show_LED(i, num[temp]);
		else
			IC_show_LED(i, 0x00);
		HAL_Delay(1);
	}
}

void IC_show_LED (uint8_t index, uint8_t symbol)
{
	// Choose LED to display
	uint8_t led[5] = {0x08, 0x10, 0x20, 0x40, 0x80};
	IC_send_data(led[index]);
	// Show symbol
	IC_send_data(symbol);
}

void IC_send_data (uint8_t data)
{
	for (int i=7; i>=0; i--){
		if((data>>i)&1)
			GPIOB->ODR |= (1UL<<5);
		else
			GPIOB->ODR &= ~(1UL<<5);
		IC_clock();
		IC_latch();
	}
}

void IC_clock (void) // Toggle PB3
{
	GPIOB->ODR |= (1UL<<3);
	GPIOB->ODR &= ~(1UL<<3);
}

void IC_latch (void) // Toggle PE0
{
	GPIOE->ODR |= (1UL<<0);
	GPIOE->ODR &= ~(1UL<<0);
}

void GPIO_init (void) // PB3 clock, PB5 data, PE0 latch, PE2 LOW to enables
{
	// Enable GPIOB
	RCC->APB2ENR |= (1<< 2) | (1<<3) | (1<<6);
	// Reset reg
	GPIOE->CRL = 0;
	GPIOB->CRL = 0;
	GPIOB->CRH = 0;
	// Configure PA9 (Tx) as Alternate Function Push-Pull
	GPIOA->CRH &= ~(0xF << 4);  																		// Clear configuration bits for PA9
	GPIOA->CRH |= (0xB << 4);   																		// Set PA9 as AF Push-Pull
	// Configure PA10 (Rx) as Input Floating
	GPIOA->CRH &= ~(0xF << 8);  																		// Clear configuration bits for PA10
	GPIOA->CRH |= (4<< 8);   																				// Set PA10 as Input Floating
	// Set PB3, PB5 as general purpose output
	GPIOB->CRL |= (1UL<<12) | (1UL<<20);
	// Set PB10 and PB11 as general purpose output
	GPIOB->CRH |= (9UL<<8) | (9UL<<12);
	// Set PB4 as general input
	GPIOB->CRL |= (8UL<<16);
	// Set PE0, PE1 and PE2 as general output
	GPIOE->CRL |= (1UL<<8) | (1UL<<4) | (1UL<<0);
	// Reset all pin to LOW
	GPIOB->ODR = 0;
//	GPIOB->ODR |= (1UL<<4); // PB4 pull-up
	GPIOE->ODR &= ~(1UL<<2);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


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
