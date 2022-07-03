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

unsigned char pucStr_Disp[21], pucStr[2];
unsigned char ucKey_Val, ucKey_Down, ucKey_Old;
unsigned char ucState;
uint16_t PULS1_T_Count, PULS2_T_Count;
uint16_t FRQ1, FRQ2;
float ADC_Value_AO1, ADC_Value_AO2;
uint8_t Dividing = 1, Doubling = 1;
unsigned char ucLed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Key_Proc(void);
void ADC_Proc(void);
void Led_Proc(void);
void Output_Ctrl(_Bool state);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	I2CInit();
	I2C_24CO2_Read(pucStr, 0, 2);
	Dividing = pucStr[0];
	Doubling = pucStr[1];
	if((Dividing < 1) || (Dividing > 4))
		Dividing = 1;
	if((Doubling < 1) || (Doubling > 4))
		Doubling = 1;
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
	
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Led_Proc();
		Lcd_Proc();
		Key_Proc();
		ADC_Proc();
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
		sprintf((char*)pucStr_Disp, "     Measuring      ");
		LCD_DisplayStringLine(Line0, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "     PULS1:%2ukHz    ", FRQ1);
		LCD_DisplayStringLine(Line2, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "     PULS2:%2ukHz    ", FRQ2);
		LCD_DisplayStringLine(Line4, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "       AO1:%3.2fV    ", ADC_Value_AO1);
		LCD_DisplayStringLine(Line6, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "       AO2:%3.2fV    ", ADC_Value_AO2);
		LCD_DisplayStringLine(Line8, (u8*)pucStr_Disp);
		sprintf((char*)pucStr_Disp, "                   1"); 
		LCD_DisplayStringLine(Line9, (u8*)pucStr_Disp);
	}
	else
	{
		sprintf((char*)pucStr_Disp, "   Configuration    ");
		LCD_DisplayStringLine(Line0, (u8*)pucStr_Disp);
		
		sprintf((char*)pucStr_Disp, "                    ");
		LCD_DisplayStringLine(Line2, (u8*)pucStr_Disp);
		
		if(ucState == 1) LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "     DIVIDE:%u       ", Dividing);
		LCD_DisplayStringLine(Line4, (u8*)pucStr_Disp);
		LCD_SetBackColor(Black);
		
		if(ucState == 2) LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "     DOUBLE:%u       ", Doubling);
		LCD_DisplayStringLine(Line6, (u8*)pucStr_Disp);
		LCD_SetBackColor(Black);
		
		sprintf((char*)pucStr_Disp, "                    ");
		LCD_DisplayStringLine(Line8, (u8*)pucStr_Disp);
		
		sprintf((char*)pucStr_Disp, "                   2"); 
		LCD_DisplayStringLine(Line9, (u8*)pucStr_Disp);
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) <= 50)
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
				
				Output_Ctrl(0);
//				HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
//				HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_2);	
			}
			else
			{
				ucState = 0;
				
				pucStr[0] = Dividing;
				pucStr[1] = Doubling;
				I2C_24C02_Write(pucStr, 0, 2);
				
				Output_Ctrl(1);
//				HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
//				HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_2);
			}
		break;
		
		case 2:
			if(ucState)
			{
				if(++ucState == 3)
					ucState = 1;
			}
		break;
			
		case 3:
			if(ucState == 1)
			{
				if(Dividing < 4)
					Dividing++;
			}
			if(ucState == 2)
			{
				if(Doubling < 4)
					Doubling++;
			}
		break;
		
		case 4:
			if(ucState == 1)
			{
				if(Dividing > 1)
					Dividing--;
			}
			if(ucState == 2)
			{
				if(Doubling > 1)
					Doubling--;
			}
		break;
	}
}

void ADC_Proc(void)
{
	uint16_t ADC_Buf[10];
	uint16_t temp;
	
	if((uwTick - uwTick_ADC_Set_Point) <= 20)
		return;
	uwTick_ADC_Set_Point = uwTick;
	
	for(int i = 0; i < 10; i++)
	{
		ADC_Buf[i] = Get_ADC2_AO1();
	}
	for(int i = 0; i <= 5; i++)
	{
		for(int j = 0; j < 9 - i - 1; j++)
		{
			if(ADC_Buf[j] > ADC_Buf[j + 1])
			{
				temp = ADC_Buf[j + 1];
				ADC_Buf[j + 1] = ADC_Buf[j];
				ADC_Buf[j] = temp;
			}
		}
	}
	ADC_Value_AO1 = (ADC_Buf[4] + ADC_Buf[5]) / 2 / 4095. * 3.3;
	
	for(int i = 0; i < 10; i++)
	{
		ADC_Buf[i] = Get_ADC2_AO2();
	}
	for(int i = 0; i <= 5; i++)
	{
		for(int j = 0; j < 9 - i - 1; j++)
		{
			if(ADC_Buf[j] > ADC_Buf[j + 1])
			{
				temp = ADC_Buf[j + 1];
				ADC_Buf[j + 1] = ADC_Buf[j];
				ADC_Buf[j] = temp;
			}
		}
	}
	ADC_Value_AO2 = (ADC_Buf[4] + ADC_Buf[5]) / 2 / 4095. * 3.3;
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) <= 100)
		return;
	uwTick_Led_Set_Point = uwTick;
	
	if(!ucState)
	{
		ucLed &= ~0x01;
	}
	else
	{
		ucLed |= 0x01;
	}
	
	if(ADC_Value_AO1 > ADC_Value_AO2)
	{
		ucLed |= 0x80;
	}
	else
	{
		ucLed &= ~0x80;
	}
	
	Led_Disp(ucLed);
}

void Output_Ctrl(_Bool State)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	if(State == 1)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	}
	else
	{
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	}
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			PULS1_T_Count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) + 1;
			FRQ1 = 1000 / PULS1_T_Count;
		}
	}
	else if(htim->Instance == TIM15)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			PULS2_T_Count = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1) + 1;
			FRQ2 = 1000 / PULS2_T_Count;
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (__HAL_TIM_GetCounter(htim) + HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) / 2 * Dividing));
		}
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (__HAL_TIM_GetCounter(htim) + HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1) / 2 / Doubling));
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

