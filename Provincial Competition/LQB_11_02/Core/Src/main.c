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
unsigned char ucKey_Val, ucKey_Down, ucKey_Old;
unsigned char ucState, ucState_1;
unsigned char ucModel, ucPWM_State;
double duty_1, duty_2;
uint32_t Compare_1, Compare_2;
double ADC_Value;
unsigned char ucLed = 3;
uint16_t PA6_Duty_Int = 10;
uint16_t PA7_Duty_Int = 10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Key_Proc(void);
void PWM_Proc(void);
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
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 50);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Lcd_Proc();
		Key_Proc();
		ADC_Proc();
		PWM_Proc();
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
void Lcd_Proc(void)
{
	if((uwTick - uwTick_Lcd_Set_Point) <= 100)
		return;
	
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!ucState)
	{
		LCD_DisplayStringLine(Line0, (u8*)"      Data          ");
		sprintf((char*)pucStr_Disp, "    V:%3.2lfV         ", ADC_Value);
		LCD_DisplayStringLine(Line2, (u8*)pucStr_Disp);
		if(ucPWM_State >= 1)
			LCD_DisplayStringLine(Line4, (u8*)"    Mode:AUTO       ");
		else
			LCD_DisplayStringLine(Line4, (u8*)"    Mode:MANU       ");
	}
	else
	{
		LCD_DisplayStringLine(Line0, (u8*)"      Para          ");
		sprintf((char*)pucStr_Disp, "    PA6:%2u%%         ", PA6_Duty_Int);
		LCD_DisplayStringLine(Line2, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "    PA7:%2u%%         ", PA7_Duty_Int);
		LCD_DisplayStringLine(Line4, (u8*)pucStr_Disp);
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) <= 50)
		return;
	
	uwTick_Key_Set_Point = uwTick;
	
	ucKey_Val = Key_Scan();
	ucKey_Down = ucKey_Val & (ucKey_Val ^ ucKey_Old);
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
			}
		break;
		
		case 2:
			if(ucState)
			{
				PA6_Duty_Int += 10;
				if(PA6_Duty_Int >= 110)
					PA6_Duty_Int = 10;
			}
		break;

		case 3:
			if(ucState)
			{
				PA7_Duty_Int += 10;
				if(PA7_Duty_Int >= 110)
					PA7_Duty_Int = 10;
			}
			else
			{
				if(ucPWM_State == 0)
					ucPWM_State = 1;
				else if(ucPWM_State == 1)
					ucPWM_State = 0;
				else if(ucPWM_State == 2)
					ucPWM_State = 3;
				else if(ucPWM_State == 3)
					ucPWM_State = 2;
			}
		break;
		
		case 4:
			if(ucPWM_State == 0)
				ucPWM_State = 2;
			else if(ucPWM_State == 2)
				ucPWM_State = 0;
			else if(ucPWM_State == 1)
				ucPWM_State = 3;
			else if(ucPWM_State == 3)
				ucPWM_State = 1;
		break;		
	}
}

void ADC_Proc(void)
{
	if((uwTick - uwTick_ADC_Set_Point) <= 50)
		return;
	
	uwTick_ADC_Set_Point = uwTick;
	
	ADC_Value = (double)ADC2_Get() * 3.3 / 4095;
}

void PWM_Proc(void)
{
	switch(ucPWM_State)
	{
		case 0:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000);	//PA6（绿色）的频率为100hz
			__HAL_TIM_SET_AUTORELOAD(&htim17, 500);//PA7（黄色）的频率为200hz
		
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PA6_Duty_Int * 10);//PA6（绿色）的占空比跟随设置
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7_Duty_Int * 5);//PA7（黄色）的占空比跟随设置	
		break;
		
		case 1:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 500);	//PA6（绿色）的频率为200hz
			__HAL_TIM_SET_AUTORELOAD(&htim17, 1000);//PA7（黄色）的频率为100hz
		
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PA6_Duty_Int * 5);//PA6（绿色）的占空比跟随设置
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, PA7_Duty_Int * 10);//PA7（黄色）的占空比跟随设置	
		break;
		
		case 2:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 1000);	//PA6（绿色）的频率为100hz
			__HAL_TIM_SET_AUTORELOAD(&htim17, 500);//PA7（黄色）的频率为200hz
		
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ADC_Value / 3.3 * 1000);//PA6（绿色）的占空比跟随设置
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, ADC_Value / 3.3 * 500);//PA7（黄色）的占空比跟随设置	
		break;
		
		case 3:
			__HAL_TIM_SET_AUTORELOAD(&htim3, 500);	//PA6（绿色）的频率为100hz
			__HAL_TIM_SET_AUTORELOAD(&htim17, 1000);//PA7（黄色）的频率为200hz
		
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ADC_Value / 3.3 * 500);//PA6（绿色）的占空比跟随设置
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, ADC_Value / 3.3 * 1000);//PA7（黄色）的占空比跟随设置	
		break;
	}
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) <= 100)
		return;
	
	uwTick_Led_Set_Point = uwTick;
	
	if(ucPWM_State <= 1)
		ucLed &= ~0x01;
	else
		ucLed |= 0x01;
	if(!ucState)
		ucLed |= 0x02;
	else
		ucLed &= ~0x02;
	
	Led_Disp(ucLed);
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

