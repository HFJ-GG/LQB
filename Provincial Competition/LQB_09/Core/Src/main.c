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
#include "tim.h"
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
__IO uint32_t uwTick_Lcd_Set_point;
__IO uint32_t uwTick_Key_Set_point;
__IO uint32_t uwTick_Key_Long_Set_point;
__IO uint32_t uwTick_Sec_Set_point;
__IO uint32_t uwTick_Led_Set_point_1;
__IO uint32_t uwTick_Led_Set_point_2;

char pucStr_Disp[21];
unsigned long ulSec;
unsigned char pucStr[3];
unsigned char ucNum;
unsigned char ucHour, ucMinute, ucSecond;
unsigned char ucKey_Val, ucKey_Down, ucKey_Old, ucKey_Up;
unsigned char ucState, ucNo = 1;
unsigned char ucLed = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Key_Proc(void);
void B3_Proc(void);
void Count_Proc(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
	I2CInit();
	I2C_24C02_Read(pucStr, 0, 3);
	ucHour = pucStr[0];
	ucMinute = pucStr[1];
	ucSecond = pucStr[2];
	if(ucHour > 24)
		ucHour = 0;
	if(ucMinute > 60)
		ucMinute = 0;
	if(ucSecond > 60)
		ucSecond = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Lcd_Proc();
		Key_Proc();
		Count_Proc();
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
	if((uwTick - uwTick_Lcd_Set_point) < 100)
		return;
	
	uwTick_Lcd_Set_point = uwTick;
	
	sprintf((char*)pucStr_Disp, "        No.%1u      ", ucNo);
	LCD_DisplayStringLine(Line3, (u8*)pucStr_Disp);
	
	if(ucState == 1)
		LCD_SetBackColor(Red);
	LCD_DisplayChar(Line5, 224, ucHour / 10 + 0x30);
	LCD_DisplayChar(Line5, 208, ucHour % 10 + 0x30);
	LCD_SetBackColor(White);
	
	LCD_DisplayChar(Line5, 192, ':');
	
	if(ucState == 2)
		LCD_SetBackColor(Red);
	LCD_DisplayChar(Line5, 176, ucMinute / 10 + 0x30);
	LCD_DisplayChar(Line5, 160, ucMinute % 10 + 0x30);
	LCD_SetBackColor(White);
	
	LCD_DisplayChar(Line5, 144, ':');
	
	if(ucState == 3)
		LCD_SetBackColor(Red);
	LCD_DisplayChar(Line5, 128, ucSecond / 10 + 0x30);
	LCD_DisplayChar(Line5, 112, ucSecond % 10 + 0x30);
	LCD_SetBackColor(White);
	
	switch(ucState)
	{
		case 0:
			LCD_DisplayStringLine(Line7, (u8*)"      Standby.       ");
		break;
		
		case 1:
		case 2:
		case 3:
			LCD_DisplayStringLine(Line7, (u8*)"      Setting.       ");
		break;
		
		case 4:
			LCD_DisplayStringLine(Line7, (u8*)"      Running.      ");
		break;
		
		case 5:
			LCD_DisplayStringLine(Line7, (u8*)"       Pause.      ");
		break;
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_point) < 50)
		return;
	
	uwTick_Key_Set_point = uwTick;
	
	ucKey_Val = Key_Scan();
	ucKey_Down = ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Up = ~ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Old = ucKey_Val;
	
	if(ucKey_Down > 0)
		uwTick_Key_Long_Set_point = uwTick;
	
	if((uwTick - uwTick_Key_Long_Set_point) <= 800)
	{
		switch(ucKey_Up)
		{
			case 1:
				if(!ucState)
				{
					if(++ucNo == 6)
						ucNo = 1;
						
					I2C_24C02_Read(pucStr, (ucNo - 1) * 3, 3);
					ucHour = pucStr[0];
					ucMinute = pucStr[1];
					ucSecond = pucStr[2];
				}
			break;
			
			case 2:
				ucNum = 0;
				if(!ucState)
					ucState = 1;
				else
				{
					if(ucState < 4)
						if(++ucState == 4)
							ucState = 1;
				}
			break;
				
			case 3:
				B3_Proc();
			break;
			
			case 4:
				if(ucNum)
				{
					if(ucState < 4)
						ulSec = ucHour * 3600 + ucMinute * 60 + ucSecond;
					if(ucState != 4)
					{
						ucState = 4;
					}
					else
						ucState = 5;
				}
			break;
		}
	}
	else
	{
		switch(ucKey_Val)
		{
			case 2:
				if(ucState == 1 || ucState == 2 || ucState == 3)
				{
					pucStr[0] = ucHour;
					pucStr[1] = ucMinute;
					pucStr[2] = ucSecond;
					I2C_24C02_Write(pucStr, (ucNo - 1) * 3, 3);
					ucState = 0;
					ucNum = 1;
				}
			break;
				
			case 3:
				B3_Proc();
			break;
			
			case 4:
				if(ucState == 4 || ucState == 5)
					ucState = 0;
			break;
		}
	}
}

void B3_Proc(void)
{
	switch(ucState)
	{
		case 1:
			if(++ucHour > 23)
				ucHour = 0;
		break;
			
		case 2:
			if(++ucMinute > 59)
				ucMinute = 0;
		break;
			
		case 3:
			if(++ucSecond > 59)
				ucSecond = 0;
		break;
	}
}

void Count_Proc(void)
{
	if(ucState == 4)
	{
		if((uwTick - uwTick_Sec_Set_point) >= 1000)
		{
			uwTick_Sec_Set_point = uwTick;
			ulSec--;
			
			ucHour = ulSec / 3600;
			ucMinute = ulSec % 3600 / 60;
			ucSecond = ulSec % 3600 % 60;
		}
	}
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_point_1) < 1000)
		return;
	
	uwTick_Led_Set_point_1 = uwTick;
	
	if(ucState == 4)
	{
		ucLed ^= 1;
		Led_Disp(ucLed);
	}
	else
	{
		ucLed = 0;
		Led_Disp(ucLed);
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

