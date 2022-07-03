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
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
__IO uint32_t uwTick_Lcd_Set_Point;
__IO uint32_t uwTick_Key_Set_Point;
__IO uint32_t uwTick_Set_Point;

unsigned char Lcd_Disp_String[21];
uint8_t Platform = 1;
uint8_t Key_Val, Key_Down, Key_Old;
uint8_t Led = 1;
uint8_t Dir;
RTC_TimeTypeDef H_M_S_Time;
RTC_DateTypeDef Y_M_D_Date;
uint8_t State, Set;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Key_Proc(void);
void ELEV_Proc(void);
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
  MX_TIM17_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Black);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Key_Proc();
		Lcd_Proc();
		ELEV_Proc();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
	
	sprintf((char*)Lcd_Disp_String, "  Current Platform  ");
	LCD_DisplayStringLine(Line2, Lcd_Disp_String);
	
	sprintf((char*)Lcd_Disp_String, "          %1u           ", Platform);
	LCD_DisplayStringLine(Line4, Lcd_Disp_String);
	
	HAL_RTC_GetTime(&hrtc, &H_M_S_Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Y_M_D_Date, RTC_FORMAT_BIN);
	sprintf((char*)Lcd_Disp_String, "      %02u-%02u-%02u     ", H_M_S_Time.Hours, H_M_S_Time.Minutes, H_M_S_Time.Seconds);
	LCD_DisplayStringLine(Line6, Lcd_Disp_String);
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) <= 50)
		return;
	
	uwTick_Key_Set_Point = uwTick;
	
	Key_Val = Key_Scan();
	Key_Down = Key_Val & (Key_Val ^ Key_Old);
	Key_Old = Key_Val;
	
	if(!State)
	{
		switch(Key_Down)
		{
			case 1:
				if (Platform != 1)
					Set |= 1;
			break;
			
			case 2:
				if (Platform != 2)
					Set |= 2;
			break;
			
			case 3:
				if (Platform != 3)
					Set |= 4;
			break;
			
			case 4:
				if (Platform != 4)
					Set |= 8;
			break;
		}
	}
	
	if(Key_Val)
		uwTick_Set_Point = uwTick;
	
	Led_Disp(Set);
}

void ELEV_Proc(void)
{
	if(Set)
	{
		switch(State)
		{
			case 0:
				if((uwTick - uwTick_Set_Point) >= 1000)
				{
					State = 1;
				}
				else
					break;
				
			case 1:
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 250);
				HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
			
				LCD_DisplayStringLine(Line8, (u8*)"Door Closing        ");
				
				uwTick_Set_Point = uwTick;
				State = 2;
			
			case 2:
				if((uwTick - uwTick_Set_Point) >= 4000)
				{
					HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
					
					LCD_DisplayStringLine(Line8, (u8*)"Door Closed        ");
					
					State = 3;
				}
				else
					break;
			
			case 3:
				if(Set > (1 << (Platform - 1)))
				{
					Dir = 1;
					
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
					
					LCD_DisplayStringLine(Line8, (u8*)" ELevator Up...     ");
				}		
				else if(Set < (1 << (Platform - 1)))
				{
					Dir = 2;
					
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
					
					LCD_DisplayStringLine(Line8, (u8*)" ELevator Down...   ");
				}
				uwTick_Set_Point = uwTick;
				State = 4;
				
			case 4:
				if((uwTick - uwTick_Set_Point) >= 6000)
				{
					if(Dir == 1)
						Platform++;
					else if(Dir == 2)
						Platform--;
					
					LCD_DisplayChar(Line4, 160, Platform + 0x30);
					
					State = 5;
				}
				else
				{
					if(Dir == 1)
					{
						Led <<= 1;
						if(Led == 0x10)
							Led = 1;
					}
					if(Dir == 2)
					{
						Led >>= 1;
						if(Led == 0)
							Led = 8;
					}
					Led_Disp((Led << 4) + Set);
					HAL_Delay(250);
					break;
				}
			
			case 5:
				if(Set & (1 << (Platform - 1)))
				{
					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
					
					LCD_DisplayChar(Line4, 160, Platform + 0x30);
					HAL_Delay(250);
					LCD_DisplayChar(Line4, 160, ' ');
					HAL_Delay(250);
					LCD_DisplayChar(Line4, 160, Platform + 0x30);
					HAL_Delay(250);
					LCD_DisplayChar(Line4, 160, ' ');
					HAL_Delay(250);
					
					LCD_DisplayStringLine(Line8, (u8*)"    Door Open...    ");
					
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 300);
					HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
					
					uwTick_Set_Point = uwTick;
					State = 6;
				}
				else
				{
					uwTick_Set_Point = uwTick;
					State = 4;
					break;
				}
				
			case 6:
				if((uwTick - uwTick_Set_Point) >= 4000)
				{
					HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
					
					LCD_DisplayStringLine(Line8, (u8*)"  Door Opened       ");
					
					Set &= (~(1 << (Platform - 1)));
					Led_Disp(Set);
					
					State = 7;
				}
				else
					break;
			
			case 7:
				if(Set)
				{
					uwTick_Set_Point = uwTick;
					
					State = 8;
				}
				else
				{
					State = 0;
					
					LCD_DisplayStringLine(Line8, (u8*)"                    ");
					
					break;
				}
			
			case 8:
				if((uwTick - uwTick_Set_Point) >= 2000)
				{
					LCD_DisplayStringLine(Line8, (u8*)"                    ");
					
					State = 1;
				}
				else
					break;
		}
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

