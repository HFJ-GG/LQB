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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
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
__IO uint32_t uwTick_Lcd_Set_Point;
__IO uint32_t uwTick_Key_Set_Point;
__IO uint32_t uwTick_ADC_Set_Point;
__IO uint32_t uwTick_Led_Set_Point;

unsigned char pucStr_Disp[21];
double ADC_Value_AVE, ADC_Value_SUM, ADC_Value_AVE_Old;
unsigned char ucTime;
unsigned char ucKey_Val, ucKey_Down, ucKey_Old;
unsigned char ucState, ucFlag = 0;
double V_max = 3, V_min = 1, V_max_Ctrl = 3, V_min_Ctrl = 1;
unsigned char ucNum_ADC;
unsigned long ulSec;
unsigned char ucLed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void Key_Proc(void);
void ADC_Proc(void);
void Led_Proc(void);
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	Led_Disp(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		LCD_Proc();
		Key_Proc();
		ADC_Proc();
		Led_Proc();
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
void LCD_Proc(void)
{
	if((uwTick - uwTick_Lcd_Set_Point) <= 500)
		return;
	
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!ucState)
	{
		LCD_DisplayStringLine(Line0, (u8*)"      Data          ");
		
		sprintf((char*)pucStr_Disp, " V:%3.2lfV            ", ADC_Value_AVE);
		LCD_DisplayStringLine(Line2, pucStr_Disp);
		
		sprintf((char*)pucStr_Disp, " T:%02lus              ", ulSec);
		LCD_DisplayStringLine(Line3, pucStr_Disp);
	}
	else
	{
		LCD_DisplayStringLine(Line0, (u8*)"      Para          ");
		
		sprintf((char*)pucStr_Disp, " Vmax:%2.1lfV        ", V_max);
		LCD_DisplayStringLine(Line2, pucStr_Disp);
		
		sprintf((char*)pucStr_Disp, " Vmin:%2.1lfV        ", V_min);
		LCD_DisplayStringLine(Line3, pucStr_Disp);		
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) <= 100)
		return;
	
	uwTick_Key_Set_Point = uwTick;
	
	ucKey_Val = Key_Scan();
	ucKey_Down = ucKey_Val & (ucKey_Old ^ ucKey_Val);
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
				ucState = 0;
				if(V_max >= (V_min + 1))
				{
					V_max_Ctrl = V_max;
					V_min_Ctrl = V_min;
					ucLed &= ~0x02;
				}
				else
				{
					V_max = V_max_Ctrl;
					V_min = V_min_Ctrl;
					ucLed |= 0x02;
				}
			}
		break;
			
		case 2:
			if(ucState)
			{
				V_max += 0.1;
				if(V_max >= 3.4)
					V_max = 0;
			}
		break;
			
		case 3:
			if(ucState)
			{
				V_min += 0.1;
				if(V_min >= 3.4)
					V_min = 0;
			}
		break;		
	}
}

void ADC_Proc(void)
{
	if((uwTick - uwTick_ADC_Set_Point) <= 50)
		return;
	
	uwTick_ADC_Set_Point = uwTick;

//该滤波法存在问题：当ADC_Value_AVE的初始值位于V_min和V_max之间时，ulSec会增加。（不清楚原因）
//	ucNum_ADC++;
//	ADC_Value_SUM += ADC2_Get() * 3.3 / 4095;
//	if(ucNum_ADC == 10)
//	{
//		ucNum_ADC = 0;
//		
//		ADC_Value_AVE = ADC_Value_SUM / 10;
//		
//		ADC_Value_SUM = 0;
//	}
	for(ucNum_ADC = 0; ucNum_ADC < 10; ucNum_ADC++)
	{
		ADC_Value_SUM += (double)ADC2_Get() * 3.3 / 4095;
		HAL_Delay(10);
	}
	ADC_Value_AVE = ADC_Value_SUM / 10;
	ADC_Value_SUM = 0;
	
	if(ADC_Value_AVE < V_min_Ctrl)
	{
		ucFlag = 1;
	}
	if((ADC_Value_AVE > V_min_Ctrl) && (ADC_Value_AVE < V_max_Ctrl))
	{
		if(ucFlag == 1)
		{
			ucFlag = 2;
			ulSec = 0;
			ucLed |= 0x01;
			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
	if(ADC_Value_AVE > V_max_Ctrl)
	{
		ucLed &= ~0x01;
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) <= 50)
		return;
	
	uwTick_Led_Set_Point = uwTick;
	
	Led_Disp(ucLed);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM6)
	{
			ulSec++;
	}
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

