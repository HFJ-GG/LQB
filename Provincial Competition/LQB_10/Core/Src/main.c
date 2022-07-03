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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "i2c - hal.h"
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
__IO uint32_t uwTick_Led_Set_Point;

double ADC_Value;
unsigned char pucStr_Disp[21], pucStr[4];
unsigned char ucKey_Val, ucKey_Down, ucKey_Old;
unsigned char ucState, ucNo;
unsigned char ucMax, ucMin, ucUpper, ucLower;
unsigned char ucLed = 1;
unsigned char ucStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void ADC_Proc(void);
void Key_Proc(void);
void B3_Proc(void);
void B4_Proc(void);
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
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
	I2CInit();
	I2C_24C02_Read(pucStr, 0, 4);
	ucMax = pucStr[0];
	ucMin = pucStr[1];
	ucUpper = pucStr[2];
	ucLower = pucStr[3];
	if(ucMax >= 33)
		ucMax = 24;
	if(ucMin >= 33)
		ucMin = 12;
	if(ucUpper >= 9)
		ucUpper = 1;
	if(ucLower >= 9)
		ucLower = 2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Lcd_Proc();
		ADC_Proc();
		Key_Proc();
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
	if((uwTick - uwTick_Lcd_Set_Point) < 100)
		return;
	
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!ucState)
	{	
		LCD_DisplayStringLine(Line2, (u8*)"        Main        ");
		
		sprintf((char*)pucStr_Disp, "     Volt: %3.2lfV     ", (double)ADC2_Get() / 4095 * 3.3);
		LCD_DisplayStringLine(Line4, (u8*)pucStr_Disp);
		
		switch(ucStatus)
		{
			case 1:
				LCD_DisplayStringLine(Line5, (u8*)"     Stat: Upper    ");
			break;
			case 2:
				LCD_DisplayStringLine(Line5, (u8*)"     Stat: Lower    ");
			break;
			case 3:
				LCD_DisplayStringLine(Line5, (u8*)"     Stat: Normal   ");
			break;
		}
		LCD_DisplayStringLine(Line6, (u8*)"                    ");
		LCD_DisplayStringLine(Line7, (u8*)"                    ");
	}
	else
	{
		LCD_DisplayStringLine(Line2, (u8*)"      Setting       ");
		
		if(ucState == 1)
			LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    Max Volt: %2.1lfV    ", (double)ucMax / 10);
		LCD_DisplayStringLine(Line4, (u8*)pucStr_Disp);
		LCD_SetBackColor(White);
		
		if(ucState == 2)
			LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    Max Volt: %2.1lfV    ", (double)ucMin / 10);
		LCD_DisplayStringLine(Line5, (u8*)pucStr_Disp);
		LCD_SetBackColor(White);
		
		if(ucState == 3)
			LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    Upper:    LD%1u    ", ucUpper);
		LCD_DisplayStringLine(Line6, (u8*)pucStr_Disp);
		LCD_SetBackColor(White);
		
		if(ucState == 4)
			LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    Upper:    LD%1u    ", ucLower);
		LCD_DisplayStringLine(Line7, (u8*)pucStr_Disp);
		LCD_SetBackColor(White);
	}
}

void ADC_Proc(void)
{
	ADC_Value = (double)ADC2_Get() / 4095 * 330;
	
	if(ADC_Value > (ucMax * 10))
	{
		ucLed ^= (1 << (ucUpper - 1));
		ucStatus = 1;
	}
	if(ADC_Value < (ucMin * 10))
	{
		ucLed ^= (1 << (ucLower - 1));
		ucStatus = 2;
	}
	if(ADC_Value <= (ucMax * 10) && ADC_Value >= (ucMin * 10))
	{
		ucLed = 0;
		ucStatus = 3;
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) < 50)
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
				
				pucStr[0] = ucMax;
				pucStr[1] = ucMin;
				pucStr[2] = ucUpper;
				pucStr[3] = ucLower;
				I2C_24C02_Write(pucStr, 0, 4);
			}
		break;
		
		case 2:
			if(ucState)
			{
				if(++ucState == 5)
					ucState = 1;
			}
		break;
			
		case 3:
				B3_Proc();
		break;
		
		case 4:
				B4_Proc();	
		break;
	}
}

void B3_Proc(void)
{
	switch(ucState)
	{
		case 1:
			if(ucMax < 33)
					ucMax += 3;
		break;
			
		case 2:
			if((ucMax - ucMin) > 3)
				ucMin += 3;
		break;
			
		case 3:
			if(++ucUpper > 8)
				ucUpper = 1;
			if(ucUpper == ucLower)
				if(++ucUpper > 8)
					ucUpper = 1;
		break;
		
		case 4:
			if(++ucLower > 8)
				ucLower = 1;
			if(ucUpper == ucLower)
				if(++ucLower > 8)
					ucLower = 1;
		break;
	}
}

void B4_Proc(void)
{
	switch(ucState)
	{
		case 1:
			if((ucMax - ucMin) > 3)
					ucMax -= 3;
		break;
			
		case 2:
			if(ucMin > 3)
				ucMin -= 3;
		break;
			
		case 3:
			if(--ucUpper == 0)
				ucUpper = 8;
			if(ucUpper == ucLower)
				if(--ucUpper == 0)
					ucUpper = 8;
		break;
		
		case 4:
			if(--ucLower == 0)
				ucLower = 8;
			if(ucUpper == ucLower)
				if(--ucLower == 0)
					ucLower = 8;
		break;
	}
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) < 200)
		return;
	
	uwTick_Led_Set_Point = uwTick;
	
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

