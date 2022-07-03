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
#include "usart.h"
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
uint32_t uwTick_LCD_Set_Point;
uint32_t uwTick_ADC_Set_Point;
uint32_t uwTick_Key_Set_Point;
uint32_t uwTick_Key_Long_Set_Point;
uint32_t uwTick_USART_Set_Point;
uint32_t uwTick_USART_Set_Point_1;
uint32_t uwTick_USART_Count_Set_Point;
uint32_t uwTick_Led_Set_Point;

uint32_t ADC_Count = 20, Led_Count, temp;
unsigned char pucStr[21], EEPROM_Num[2];
double ADC_Value_R37, ADC_Value_R38;
unsigned char ucKey_Val, ucKey_Down, ucKey_Old, ucKey_Up;
uint8_t State;
uint16_t PWM1_T_Count, PWM1_H_Count, Num, Channel, Channel_Old;
double Duty;
float DS18B20_Value;
uint8_t DS18B20_Value_T = 30, DS18B20_Value_T_Old = 30;
unsigned char pucRcv[1], rx_buffer, Count = 0, Rx_Buf[200], Rx_Buf_1[200];
unsigned char ucLed, ucLed_Flag;
uint8_t Rx_Flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void ADC_Proc(void);
void Key_Proc(void);
void USART_Proc(void);
void USART_Porc_1(void);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	I2CInit();
	I2C_24C02_Read(EEPROM_Num, 0, 2);
	Num = EEPROM_Num[0] + (EEPROM_Num[1] << 8);
	
	ds18b20_init_x();
	
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*)(&rx_buffer), 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Key_Proc();
		LCD_Proc();
		ADC_Proc();
		USART_Proc();
		USART_Porc_1();
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
	if((uwTick - uwTick_LCD_Set_Point) < 300)
		return;
	uwTick_LCD_Set_Point = uwTick;
	
	if(!State)
	{
		sprintf((char*)pucStr, "        Main          ");
		LCD_DisplayStringLine(Line0, pucStr);
		sprintf((char*)pucStr, "                      ");
		LCD_DisplayStringLine(Line1, pucStr);
		sprintf((char*)pucStr, "      R37:%3.2lfV      ", ADC_Value_R37);
		LCD_DisplayStringLine(Line2, pucStr);
		sprintf((char*)pucStr, "      R38:%3.2lfV      ", ADC_Value_R38);
		LCD_DisplayStringLine(Line3, pucStr);
		sprintf((char*)pucStr, "      PWM2:%.0lf%%    ", Duty * 100);
		LCD_DisplayStringLine(Line4, pucStr);
		sprintf((char*)pucStr, "      Temp:%.2fC  ", DS18B20_Value);
		LCD_DisplayStringLine(Line5, pucStr);
		sprintf((char*)pucStr, "      N:%d        ", Num);
		LCD_DisplayStringLine(Line6, pucStr);
	}
	else
	{
		sprintf((char*)pucStr, "        Para         ");
		LCD_DisplayStringLine(Line0, pucStr);
		
		if(State == 1)
			LCD_SetBackColor(Red);
		sprintf((char*)pucStr, "       T:%d          ", DS18B20_Value_T);
		LCD_DisplayStringLine(Line1, pucStr);
		LCD_SetBackColor(Black);
		
		sprintf((char*)pucStr, "                      ");
		LCD_DisplayStringLine(Line2, pucStr);
		
		if(State == 2)
			LCD_SetBackColor(Red);
		if(!Channel)
			sprintf((char*)pucStr, "       X:AO1        ");
		else
			sprintf((char*)pucStr, "       X:AO2        ");
		LCD_DisplayStringLine(Line3, pucStr);
		LCD_SetBackColor(Black);
		
		sprintf((char*)pucStr, "                      ");
		LCD_DisplayStringLine(Line4, pucStr);
		sprintf((char*)pucStr, "                      ");
		LCD_DisplayStringLine(Line5, pucStr);
		sprintf((char*)pucStr, "                      ");
		LCD_DisplayStringLine(Line6, pucStr);
	}
}

void ADC_Proc(void)
{
	if((uwTick - uwTick_ADC_Set_Point) < 100)
		return;
	uwTick_ADC_Set_Point = uwTick;
	
	ADC_Value_R37 = Get_ADC_R37() / 4095. * 3.3;
	ADC_Value_R38 = Get_ADC_R38() / 4095. * 3.3;
	
	ADC_Count--;
	if(!ADC_Count)
	{
		ADC_Count = 20;
		DS18B20_Value = (DS18B20_Read() & 0x07FF) / 16.0;
	}
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) < 50)
		return;
	uwTick_Key_Set_Point = uwTick;
	
	ucKey_Val = Key_Scan();
	ucKey_Down = ucKey_Val & (ucKey_Val ^ ucKey_Old);
	ucKey_Up = ~ucKey_Val & (ucKey_Val ^ ucKey_Old);
	ucKey_Old = ucKey_Val;
	
	if(ucKey_Down)
		uwTick_Key_Long_Set_Point = uwTick;
	
	switch(ucKey_Down)
	{
		case 1:
			if(!State)
			{
				State = 1;
			}
			else
			{
				State = 0;
				
				if((Channel != Channel_Old) || (DS18B20_Value_T != DS18B20_Value_T_Old))
				{
					Num++;
					EEPROM_Num[0] = Num & 0x00ff;
					EEPROM_Num[1] = Num >> 8;
					I2C_24C02_Write(EEPROM_Num, 0, 2);
					
					DS18B20_Value_T_Old = DS18B20_Value_T;
					Channel_Old = Channel;
				}
			}
		break;
		
		case 2:
				if(State)
				{
					if(++State == 3)
						State = 1;
				}
		break;		
	}
	
	if((uwTick - uwTick_Key_Long_Set_Point) <= 800)
	{
		switch(ucKey_Up)
		{
			case 3:
				if(State == 1)
				{
					if(DS18B20_Value_T < 40)
						DS18B20_Value_T++;
				}
				else if(State == 2)
				{
					if(!Channel)
						Channel = 1;
					else
						Channel = 0;
				}
			break;
				
			case 4:
				if(State == 1)
				{				
					if(DS18B20_Value_T > 20)
						DS18B20_Value_T--;
				}
				else if(State == 2)
				{
					if(!Channel)
						Channel = 1;
					else
						Channel = 0;			
				}
			break;
		}
	}
	else
	{
		switch(ucKey_Val)
		{
			case 3:
				if(State == 1)
				{
					if(DS18B20_Value_T < 40)
						DS18B20_Value_T++;
				}
			break;
				
			case 4:
				if(State == 1)
				{
					if(DS18B20_Value_T > 20)
						DS18B20_Value_T--;
				}
			break;
		}
	}
}

void USART_Proc(void)
{
	if((uwTick - uwTick_USART_Set_Point) <= 100)
		return;
	uwTick_USART_Set_Point = uwTick;
	
	if(Rx_Flag)
	{
		if(strcmp((const char*)Rx_Buf, "ST\r\n") == 0)
		{
			sprintf((char*)pucStr, "$%5.2f\r\n", DS18B20_Value);
			HAL_UART_Transmit(&huart1, (u8*)pucStr, strlen((char*)pucStr), 50);
			
			ucLed |= 0x01;
		}
		else if(strcmp((const char*)Rx_Buf, "PARA\r\n") == 0)
		{
			if(!Channel)
				sprintf((char*)pucStr, "#%d,AO1\r\n", DS18B20_Value_T);
			else
				sprintf((char*)pucStr, "#%d,AO2\r\n", DS18B20_Value_T);
			HAL_UART_Transmit(&huart1, (u8*)pucStr, strlen((char*)pucStr), 50);
			
			ucLed &= ~0x01;
		}
		else
		{
			sprintf((char*)pucStr, "Error\r\n");
			HAL_UART_Transmit(&huart1, (u8*)pucStr, strlen((char*)pucStr), 50);
			
			ucLed &= ~0x01;
		}
		
		memset(Rx_Buf, 0, sizeof(Rx_Buf));
		Rx_Flag = 0;
		Count = 0;
	}
}

void USART_Porc_1(void)
{
	if((uwTick - uwTick_USART_Set_Point) < 1000)
		return;
	uwTick_USART_Set_Point = uwTick;
	
	if((ADC_Value_R37 > Duty * 3.3) || (ADC_Value_R38 > Duty * 3.3))
	{
		sprintf((char*)pucStr, "$%5.2f\r\n", DS18B20_Value);
		HAL_UART_Transmit(&huart1, (u8*)pucStr, strlen((char*)pucStr), 50);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		PWM1_T_Count = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1) + 1; 
		
		Duty = (float)PWM1_H_Count / PWM1_T_Count;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		PWM1_H_Count = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2) + 1; 
	}
}

void Led_Proc(void)
{
	if((uwTick - uwTick_Led_Set_Point) < 200)
		return;
	uwTick_Led_Set_Point = uwTick;
	
	if(DS18B20_Value > DS18B20_Value_T_Old)
	{
		ucLed ^= (1 << 7);
	}
	else
	{
		ucLed &= ~(1 << 7);
	}
	
	if((ADC_Value_R37 > Duty * 3.3) || (ADC_Value_R38 > Duty * 3.3))
	{
		ucLed |= 0x01;
	}
	else
	{
		ucLed &= ~0x01;
	}
	Led_Disp(ucLed);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Flag = 1;
	Rx_Buf[Count++] = rx_buffer;
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_buffer, 1);
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

