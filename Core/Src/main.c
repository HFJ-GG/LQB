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
#include "lcd.h"
#include "i2c - hal.h"
#include "stdio.h"
#include "string.h"
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
__IO uint32_t uwTick_Lcd_Set_Point = 0;
__IO uint32_t uwTick_ADC_Set_Point = 0;
__IO uint32_t uwTick_Key_Set_Point = 0;
__IO uint32_t uwTick_Led_Set_Point = 0;
__IO uint32_t uwTick_Led_Set_Point_1 = 0;
uint8_t ucKey_Val, ucKey_Down, ucKey_Up, ucKey_Old;
uint8_t Lcd_Disp_String[21];
uint8_t Height = 0;
uint8_t Level;
uint8_t Level_Old;
uint8_t ucState;
double SUM_AD_R37;
double AVE_AD_R37;
uint8_t pucTh[3];
unsigned char pucRcv[1];
char str[40];
unsigned char ucLed, ucLd2, ucLd3, ucNum_1 = 10, ucNum_2 = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void ADC_Proc(void);
void Key_Proc(void);
void Led_Proc(void);
void Led_Proc_1(void);
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
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(pucRcv), 1);
	I2CInit();
	I2C_24C02_Read(pucTh, 0, 3);
	HAL_Delay(5);
	if(pucTh[0] > 100)
		pucTh[0] = 10;
	if(pucTh[1] > 100)
		pucTh[1] = 20;
	if(pucTh[2] > 100)
		pucTh[2] = 30;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Led_Proc();
		Led_Proc_1();
		ADC_Proc();
		Key_Proc();
		Lcd_Proc();
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
void Lcd_Proc(void)
{
	if(((uwTick - uwTick_Lcd_Set_Point) < 100) && !ucState)
		return;
	
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!ucState)
	{
		LCD_DisplayStringLine(Line1, (u8*)"    Liquid Level    ");
	
		sprintf((char*)Lcd_Disp_String, "    Height:%3ucm     ", Height);
		LCD_DisplayStringLine(Line4, Lcd_Disp_String);	
	
		sprintf((char*)Lcd_Disp_String, "    VR37:%4.2lfV      ", AVE_AD_R37);
		LCD_DisplayStringLine(Line6, Lcd_Disp_String);
	
		sprintf((char *)Lcd_Disp_String,"    Level:%1u        ", Level);
		LCD_DisplayStringLine(Line8, Lcd_Disp_String);
	}
	else
	{
		LCD_DisplayStringLine(Line1, (u8*)"  Threshold Setup   ");
		
		sprintf((char*)Lcd_Disp_String, "  Threshold 1: %02ucm  ", pucTh[0]);
		if(ucState == 1) 
			LCD_SetBackColor(Red);
		LCD_DisplayStringLine(Line4, Lcd_Disp_String);
		LCD_SetBackColor(White);
		
		sprintf((char*)Lcd_Disp_String, "  Threshold 2: %02ucm  ", pucTh[1]);
		if(ucState == 2) 
			LCD_SetBackColor(Red);
		LCD_DisplayStringLine(Line6, Lcd_Disp_String);
		LCD_SetBackColor(White);
		
		sprintf((char*)Lcd_Disp_String, "  Threshold 3: %02ucm  ", pucTh[2]);
		if(ucState == 3) 
			LCD_SetBackColor(Yellow);
		LCD_DisplayStringLine(Line8, Lcd_Disp_String);
		LCD_SetBackColor(White);
	}
}

void ADC_Proc(void)
{
	if((uwTick - uwTick_ADC_Set_Point) < 100)
		return;
	
	uwTick_ADC_Set_Point = uwTick;
	
//	for(int i = 0; i < 10; i++)
//	{
//		SUM_AD_R37 += ((float)getADC2());
//	}
//	SUM_AD_R37 = SUM_AD_R37 /4095 *3.3;
//	AVE_AD_R37 = SUM_AD_R37 / 10;
//	SUM_AD_R37 = 0;
	
	AVE_AD_R37 = ((double)getADC2()) / 4095 * 3.3;
	Height = (uint8_t)(AVE_AD_R37 * 30.3);
	
	Level_Old = Level;
	
	if(Height < pucTh[0])
		Level = 0;
	else if(Height < pucTh[1])
		Level = 1;
	else if(Height < pucTh[2])
		Level = 2;
	else
		Level = 3;
	
//	if(Level > Level_Old)
//	{
//		sprintf((char*)str, "UP  Level:%1u  Level_Old:%1u\r\n", Level, Level_Old);
//		HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
//	}
//	else if(Level < Level_Old)
//	{
//		sprintf((char*)str, "DOWN  Level:%1u  Level_Old:%1u\r\n", Level, Level_Old);
//		HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
//	}
	if(Level != Level_Old)
	{
		if(Level > Level_Old)
		{
			sprintf((char*)str, "A:H%02u+L%1u+U\r\n", Height, Level);
			HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
//			sprintf((char*)str, "UP  Level:%1u  Level_Old:%1u\r\n", Level, Level_Old);
//			HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
		}
		else
		{
			sprintf((char*)str, "A:H%02u+L%1u+D\r\n", Height, Level);
			HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
//			sprintf((char*)str, "DOWN  Level:%1u  Level_Old:%1u\r\n", Level, Level_Old);
//			HAL_UART_Transmit(&huart1, (unsigned char*)str, strlen(str), 50);
		}
		
	ucLd2 = 1;
	}
}

void Key_Proc(void)
{
	if((uwTick -  uwTick_Key_Set_Point)<50)	
		return;//¼õËÙº¯Êý
	uwTick_Key_Set_Point = uwTick;
	
	ucKey_Val = Key_Scan();
	ucKey_Down = ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Up = ~ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Old = ucKey_Val;
	
	switch(ucKey_Down)
	{
		case 1:
			if(!ucState)
			{
				ucState = 1;
			}
			else
			{
				if((pucTh[0] < pucTh[1]) && (pucTh[1] < pucTh[2]))
				{
					ucState = 0;
					LCD_DisplayStringLine(Line3, (u8*)"                    ");
					I2C_24C02_Write(pucTh, 0, 3);
					HAL_Delay(5);
				}
				else
				{
					LCD_SetTextColor(Red);
					LCD_DisplayStringLine(Line3, (uint8_t*)"   Threshold Error   ");
					LCD_SetTextColor(Blue);
				}
			}	
		break;
			
		case 2:
			if(ucState)
				if(++ucState  == 4)
					ucState = 1;
		break;
				
		case 3:
			if(ucState)
				if(pucTh[ucState - 1] < 95)
					pucTh[ucState - 1] += 5;
		break;
		
		case 4:
			if(ucState)
				if(pucTh[ucState - 1] > 5)
					pucTh[ucState - 1] -= 5;
		break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(pucRcv[0] == 'C')
	{
		sprintf((char*)str,"C:H%02u+L%1u\r\n", Height, Level);
		HAL_UART_Transmit(&huart1,(unsigned char *)str, strlen(str), 50);
	}
	if(pucRcv[0] == 'S')
	{
		sprintf((char*)str,"S:TL%02u+TM%2u+TH%02u\r\n", pucTh[0], pucTh[1], pucTh[2]);
		HAL_UART_Transmit(&huart1,(unsigned char *)str, strlen(str), 50);
	}
	if(pucRcv[0] == 'C' || pucRcv[0] == 'S')
	{
		ucLd3 = 1;
	}
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(pucRcv), 1);
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) < 200)
		return;
	
	uwTick_Led_Set_Point = uwTick;
	
	if(ucLd2)
	{
		if(ucNum_1--)
			ucLed ^= 2;
		else
		{
			ucLd2 = 0;
			ucNum_1 = 10;
		}
	}
	
	if(ucLd3)
	{
		if(ucNum_2--)
			ucLed ^= 4;
		else
		{
			ucLd3 = 0;
			ucNum_2 = 10;
		}
	}
	
	Led_Disp(ucLed);
}

void Led_Proc_1(void)
{
	if((uwTick - uwTick_Led_Set_Point_1) < 1000)
		return;
	
	uwTick_Led_Set_Point_1 = uwTick;
	
	ucLed ^= 1;
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

