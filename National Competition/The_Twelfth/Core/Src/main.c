/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "lcd.h"
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
__IO uint32_t uwTick_USART_Set_Point;
__IO uint32_t uwTick_Lcd_Set_Point;
__IO uint32_t uwTick_ADC_Set_Point;
uint32_t Num;

uint8_t RX_BUF[200];
uint8_t Rx_Counter;
uint8_t rx_buffer;
uint8_t Rx_Flag;
uint8_t pucStr[21];
uint16_t a_buf[5];
uint16_t q_a_buf[5];
uint16_t temp;
uint8_t i;
char str[40];
double ADC_Value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USART_Proc(void);
void LCD_Proc(void);
void ADC_Proc(void);
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
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ADC_Proc();
		LCD_Proc();
		USART_Proc();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ADC_Proc(void)
{
	if(uwTick - uwTick_ADC_Set_Point <= 1000)
		return;
	uwTick_ADC_Set_Point = uwTick;
	
	//ADC_Value = Get_R37_Value() / 4095. * 3.3;
	Num++;
	a_buf[i++] = Num;
	if(i >= 5)
		i = 0;
	
	for(int j = 0; j < 5; j++)
	{
		q_a_buf[j] = a_buf[j];
	}
	for(int k = 0; k <= 2; k++)
	{
		for(int j = 0; k < 5 - j - 1; j++)
		{
			if(q_a_buf[j] > q_a_buf[j + 1])
			{
				temp = q_a_buf[j];
				q_a_buf[j] = q_a_buf[j + 1];
				q_a_buf[j + 1] = temp;
			}
		}
	}
}

void LCD_Proc(void)
{
	if(uwTick - uwTick_Lcd_Set_Point <= 100)
		return;
	uwTick_Lcd_Set_Point = uwTick;
	
	sprintf((char*)pucStr, "ADC:%3.2lf", ADC_Value);
	LCD_DisplayStringLine(Line1, (u8*)pucStr);
}

void USART_Proc(void)
{
	if(uwTick - uwTick_USART_Set_Point <= 100)
		return;
	uwTick_USART_Set_Point = uwTick;
	
	if(Rx_Flag)
	{
//		if(strcmp((const char*)RX_BUF, "a?") == 0)
//		{
//			sprintf(str, "a\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else if(strcmp((const char*)RX_BUF, "b?") == 0)
//		{
//			sprintf(str, "b\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else if(strcmp((const char*)RX_BUF, "aa?") == 0)
//		{
//			sprintf(str, "aa:%d-%d-%d-%d-%d\r\n", a_buf[0], a_buf[1], a_buf[2], a_buf[3], a_buf[4]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else if(strcmp((const char*)RX_BUF, "bb?") == 0)
//		{
//			sprintf(str, "bb\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else if(strcmp((const char*)RX_BUF, "qa?") == 0)
//		{
//			sprintf(str, "qa:%d-%d-%d-%d-%d\r\n", q_a_buf[0], q_a_buf[1], q_a_buf[2], q_a_buf[3], q_a_buf[4]);
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else if(strcmp((const char*)RX_BUF, "qb?") == 0)
//		{
//			sprintf(str, "qb\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
//		else
//		{
//			sprintf(str, "Error\r\n");
//			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
//		}
		
		if(strcmp((const char*)RX_BUF, "1") == 0)
		{
			sprintf(str, "1\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
		}
		else if(strcmp((const char*)RX_BUF, "2") == 0)
		{
			sprintf(str, "2\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
		}
		else
		{
			sprintf(str, "0\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 50);
		}
		
		Rx_Flag = 0;
		Rx_Counter = 0;
		memset(RX_BUF, 0, sizeof(RX_BUF));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Flag = 1;
	RX_BUF[Rx_Counter++] = rx_buffer;
	HAL_UART_Receive_IT(&huart1, &rx_buffer, 1);
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

