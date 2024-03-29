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

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void TIM2_IRQHandler(void);

void init_leds(void);

void init_tim2(void);

void init_tim3(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	init_leds();
		
  /* Configure the system clock */
  SystemClock_Config();
	
	init_tim2();
	
	init_tim3();
	
  while (1)
  {

  }

}

void TIM2_IRQHandler(void) 
{
	// Toggle green and orange LEDs
	GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	// Clear pending flag 
	TIM2->SR = 0;
}

void init_leds(void)
{
	// Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set pins PC6 to general purpose 
  GPIOC->MODER |= (1<<12); // PC6
	GPIOC->MODER |= (1<<14); // PC7
  GPIOC->MODER |= (1<<16); // PC8
	GPIOC->MODER |= (1<<18); // PC9
	
	// Set pins to push-pull 
	GPIOC->OTYPER &= ~(1<<6); // PC6
	GPIOC->OTYPER &= ~(1<<7); // PC7
	GPIOC->OTYPER &= ~(1<<8); // PC8
	GPIOC->OTYPER &= ~(1<<9); // PC9
	
	// Set pins to low speed
	GPIOC->OSPEEDR &= ~(1<<12); // PC6
	GPIOC->OSPEEDR &= ~(1<<14); // PC7
	GPIOC->OSPEEDR &= ~(1<<16); // PC8
	GPIOC->OSPEEDR &= ~(1<<18); // PC9
	
	// Set to no pull-up/down resistors 
	GPIOC->PUPDR &= ~((1<<12) | (1<<13)); // PC6
	GPIOC->PUPDR &= ~((1<<14) | (1<<15)); // PC7
	GPIOC->PUPDR &= ~((1<<16) | (1<<17)); // PC8
	GPIOC->PUPDR &= ~((1<<18) | (1<<19)); // PC9
	
	// Set green PC9 high
	GPIOC->ODR |= GPIO_ODR_9; 
}

void init_tim2(void) 
{
	// Enable TIM2 in RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Trigger update event at 4Hz - PSC=7999 and ARR=250ms
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	// Enable update interrupt in the DIER register
	TIM2->DIER |= 1;
	
	// Start timer
	TIM2->CR1 = 1;
	
	// Enable timer interrupt handler via NVIC 
	NVIC_EnableIRQ(TIM2_IRQn);
}

void init_tim3(void)
{
	// Enable TIM3 in RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Setting PSC and ARR values
	TIM3->PSC = 1999;
	TIM3->ARR = 5;
	
	// Configuring output channels to PWM mode
	
	// CC1S[1:0] set to output
	TIM3->CCMR1 |= (0 << 1);
	// CC2S[1:0] set to output
	TIM3->CCMR1 |= (0 << 8);
	// For OC1M[2:0] set output channel 1 to PWM 2 
	TIM3->CCMR1 |= (7 << 4);
	// For OC2M[2:0] set output channel 2 to PWM 1 
	TIM3->CCMR1 |= (6 << 12);
	
	// Enable output compare preload for both channels 
	TIM3->CCMR1 |= (1 << 3);
	TIM3->CCMR1 |= (1 << 11);
	
	// Set output enable bits in CCER register for channels 1 and 2
	TIM3->CCER |= (1 << 0);
	TIM3->CCER |= (1 << 4);
	
	// Set capture/compare registers for both channels to 20% of ARR value
	TIM3->CCR1 = 1;
	TIM3->CCR2 = 1;
	
	// Enable TIM3 in RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Configure PC6 and PC7 LED pins to alternate function mode 
	GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) 
								| GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 
								
	// Select appropriate function number in alternate function registers 
	GPIOC->AFR[0] |= 0x0 << GPIO_AFRL_AFRL6_Pos;
	GPIOC->AFR[0] |= 0x0 << GPIO_AFRL_AFRL7_Pos;
	
	// Start timer
	TIM3->CR1 = 1;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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