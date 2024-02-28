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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	// Function to read a character from USART3
char USART3_ReadChar(void) {
    while (!(USART3->ISR & USART_ISR_RXNE));  // Wait for RXNE (Receive data register not empty)
    return USART3->RDR;  // Read the received data
}
void USART3_WriteChar(char ch) {
    while (!(USART3->ISR & USART_ISR_TXE));  // Wait for TXE (Transmit data register empty)
    USART3->TDR = ch;  // Transmit the character
}
// Function to send a string via USART3
void USART3_SendString(const char* str) {
    while (*str) {
        USART3_WriteChar(*str++);
    }
}

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// Enable GPIOC clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Enable system clock for USART3 in RCC peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	

    // Set PC4 and PC5 to Alternate Function mode
    GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOC->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);

    							
	// Select appropriate function number in alternate function registers 
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL4_Pos;
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL5_Pos;

    // Configure USART3
    uint32_t clk_freq = HAL_RCC_GetHCLKFreq();
		USART3->BRR = clk_freq / 115200;
	
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable TX and RX
    USART3->CR1 |= USART_CR1_UE; // Enable USART3
		   GPIO_InitTypeDef initc6789 = {GPIO_PIN_8 | GPIO_PIN_9|GPIO_PIN_6|GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	 HAL_GPIO_Init(GPIOC, &initc6789);
		
  while (1)
  { int c=0;
		USART3_SendString("\n Color Command?\n");
		char ch = USART3_ReadChar();  // Read a character from USART3
		USART3_SendString("Action?\n");
		char action = USART3_ReadChar();// Process the received character
        switch (ch) {
					  case 'R':
							switch (action){
								case '0':
									GPIOC->BSRR |= GPIO_PIN_6 << 16;break;
								case '1':
									GPIOC->BSRR |= GPIO_PIN_6;break;
								case '2':
                  while(c<10){GPIOC->ODR ^= GPIO_ODR_6;HAL_Delay(200);c++;}// Toggle RED LED
                break;
								default:
						   	USART3_SendString("Error: Unrecognized action\n");break;
							}break;
						case 'B':
               switch (action){
								case '0':
									GPIOC->BSRR |= GPIO_PIN_7 << 16;break;
								case '1':
									GPIOC->BSRR |= GPIO_PIN_7;break;
								case '2':
									
                 while(c<10){ GPIOC->ODR ^= GPIO_ODR_7; HAL_Delay(200);c++;}// Toggle BLUE LED
                break;
								default:
						   	USART3_SendString("Error: Unrecognized action\n");break;
							}break;
            case 'O':
               switch (action){
								case '0':
									GPIOC->BSRR |= GPIO_PIN_8 << 16;break;
								case '1':
									GPIOC->BSRR |= GPIO_PIN_8;break;
								case '2':
                  while(c<10){GPIOC->ODR ^= GPIO_ODR_8;HAL_Delay(200);c++;}// Toggle ORANGE LED
                break;
								default:
						   	USART3_SendString("Error: Unrecognized action\n");break;
							}break;
            case 'G':
               switch (action){
								case '0':
									GPIOC->BSRR |= GPIO_PIN_9 << 16;break;
								case '1':
									GPIOC->BSRR |= GPIO_PIN_9;break;
								case '2':
                 while(c<10){ GPIOC->ODR ^= GPIO_ODR_9;HAL_Delay(200);c++;}// Toggle GREEN LED
                break;
								default:
						   	USART3_SendString("Error: Unrecognized action\n");break;
							}break;
            // Add more cases for other characters
            default:
							USART3_SendString("Error: Unrecognized command\n"); // Send an error message if character doesn't match
                break;
        }
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