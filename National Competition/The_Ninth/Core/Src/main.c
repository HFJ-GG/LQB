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
__IO uint32_t uwTick_ADC_Set_Point;
__IO uint32_t uwTick_Key_Set_Point;
__IO uint32_t uwTick_Key_Long_Set_Point;
__IO uint32_t uwTick_Led1_Set_Point;
__IO uint32_t uwTick_Led2_Set_Point;

unsigned char pucStr_Disp[21],  pucStr[21];
float ADC_Value, Weight;
double Price_1, Price_2, Price_3, Total;
uint8_t Num = 1;
unsigned char ucKey_Down, ucKey_Val, ucKey_Old, ucKey_Up;
uint8_t State;
uint16_t Setting_Num;
uint16_t puiUw[3], puiUw_Old[3], ADC_RP5[11];
unsigned char str[40];
uint8_t Flag_Usart1, Flag_Led;
unsigned char ucLed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void ADC_Proc(void);
void Key_Proc(void);
void S1_Proc(void);
void S2_Proc(void);
void S3_Proc(void);
void Para_Read(void);
void USART_Proc(void);
void Led1_Proc(void);
void Led2_Proc(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	I2CInit();
	Para_Read();
	
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
		USART_Proc();
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
void Para_Read(void)
{
	I2C_24C02_Read(pucStr, 0, 8);
	if(pucStr[0] == 0xaa)
	{
		puiUw[0] = pucStr[2] * 100 + pucStr[1];
		puiUw[1] = pucStr[4] * 100 + pucStr[3];
		puiUw[2] = pucStr[6] * 100 + pucStr[5];
		Setting_Num = pucStr[7];
	}
	else
	{
		puiUw[0] = 100;
		puiUw[1] = 100;
		puiUw[2] = 100;
		Setting_Num = 0;
	}
}
	
void Lcd_Proc(void)
{
	if((uwTick - uwTick_Lcd_Set_Point) <= 100)
		return;
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!State)
	{
		sprintf((char*)pucStr_Disp, "      Parament      ");
		LCD_DisplayStringLine(Line0, pucStr_Disp);
		sprintf((char*)pucStr_Disp, "    Num:%1u           ", Num);
		LCD_DisplayStringLine(Line2, pucStr_Disp);
		sprintf((char*)pucStr_Disp, "    Price:%.2lfR/kg  ", puiUw[Num - 1] / 100.);
		LCD_DisplayStringLine(Line4, pucStr_Disp);
		sprintf((char*)pucStr_Disp, "    Weight:%.2fkg    ", Weight);
		LCD_DisplayStringLine(Line6, pucStr_Disp);
		sprintf((char*)pucStr_Disp, "    Total:%.2lfRMB   ", Total);
		LCD_DisplayStringLine(Line8, pucStr_Disp);
	}
	else
	{
		sprintf((char*)pucStr_Disp, "      Setting       ");
		LCD_DisplayStringLine(Line0, pucStr_Disp);
		
		if(State == 1) LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    1_Price:%.2lfR/kg ", puiUw[0] / 100.);
		LCD_DisplayStringLine(Line2, pucStr_Disp);
		LCD_SetBackColor(Black);
		
		if(State == 2) LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    2_Price:%.2lfR/kg ", puiUw[1] / 100.);
		LCD_DisplayStringLine(Line4, pucStr_Disp);
		LCD_SetBackColor(Black);
		
		if(State == 3) LCD_SetBackColor(Red);
		sprintf((char*)pucStr_Disp, "    3_Price:%.2lfR/kg ", puiUw[2] / 100.);
		LCD_DisplayStringLine(Line6, pucStr_Disp);
		LCD_SetBackColor(Black);
		
		sprintf((char*)pucStr_Disp, "    Set_Num:%u        ", Setting_Num);
		LCD_DisplayStringLine(Line8, pucStr_Disp);
	}
}

void ADC_Proc(void)
{
	int temp = 0;
	
	if((uwTick - uwTick_ADC_Set_Point) <= 10)
		return;
	uwTick_ADC_Set_Point = uwTick;
	
	for(int i = 0; i < 11; i++)
	{
		ADC_RP5[i] = Get_ADC2_RP5();
	}
	for(int i = 0; i < 5; i++)
	{
		for(int j = 0; j < 10 - i; j++)
		{
			if(ADC_RP5[j] > ADC_RP5[j + 1])
			{
				temp = ADC_RP5[j + 1];
				ADC_RP5[j + 1] = ADC_RP5[j];
				ADC_RP5[j] = temp;
			}
		}
	}
	Weight =  ADC_RP5[5] / 4095. * 3.3 * 3.03;
}

void USART_Proc(void)
{
	if(Flag_Usart1 == 1)
	{
		sprintf((char*)str, "U.W.1:%.2f\r\n", puiUw[0] / 100.);
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
		sprintf((char*)str, "U.W.2:%.2f\r\n", puiUw[1] / 100.);
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
		sprintf((char*)str, "U.W.3:%.2f\r\n", puiUw[2] / 100.);
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
	}
	else if(Flag_Usart1 == 2)
	{
		if(Num == 1)
		{
			sprintf((char*)str, "U.W.1:%.2f\r\n", puiUw[0] / 100.);
		}
		else if(Num == 2)
		{
			sprintf((char*)str, "U.W.2:%.2f\r\n", puiUw[1] / 100.);
		}
		else if(Num == 3)
		{
			sprintf((char*)str, "U.W.3:%.2f\r\n", puiUw[2] / 100.);
		}
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
		sprintf((char*)str, "G.W:%.2f\r\n", Weight);
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
		sprintf((char*)str, "Total:%.2f\r\n", Total);
		HAL_UART_Transmit(&huart1, (u8*)str, strlen((char*)str), 50);
	}
	
	Flag_Usart1 = 0;
}

void Key_Proc(void)
{
	if((uwTick - uwTick_Key_Set_Point) <= 50)
		return;
	uwTick_Key_Set_Point = uwTick;
	
	ucKey_Val = Scan_Btn();
	ucKey_Down = ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Up = ~ucKey_Val & (ucKey_Old ^ ucKey_Val);
	ucKey_Old = ucKey_Val;
	
	if(ucKey_Down > 0)
		uwTick_Key_Long_Set_Point = uwTick;
	
	if((uwTick - uwTick_Key_Long_Set_Point) <= 800)
	{
		switch(ucKey_Up)
		{
			case 1:
				S1_Proc();
			break;
			
			case 2:
				S2_Proc();
			break;
			
			case 3:
				S3_Proc();
			break;
			
			case 4:
				if(State)
				{
					if(++State == 4)
						State = 1;
				}
			break;
			
			case 5:
				if(!State) 
					Num = 1;
			break;
				
			case 6:
				if(!State) 
					Num = 2;
			break;	
				
			case 7:
				if(!State) 
					Num = 3;
			break;
				
			case 8:
				if(!State) 
				{
					Flag_Usart1 = 2;
			
					Total = puiUw[Num - 1] * Weight;
				}
			break;
		}
	}
	else
	{
		switch(ucKey_Val)
		{
			case 2:
				if(State)
				{
					S2_Proc();
				}
			break;
			
			case 3:
				if(State)
				{
					S3_Proc();
				}
			break;
		}
	}
}

void S1_Proc(void)
{
	if(!State)
	{
		State = 1;
		Flag_Led = 1;
		
		puiUw_Old[0] = puiUw[0];
		puiUw_Old[1] = puiUw[1];
		puiUw_Old[2] = puiUw[2];
	}
	else
	{
		State = 0;
		Flag_Led = 0;
		
		if((puiUw[0] != puiUw_Old[0]) || (puiUw[1] != puiUw_Old[1]) || (puiUw[2] != puiUw_Old[2]))
		{
			pucStr[0] = 0xaa;
			pucStr[1] = puiUw[0] % 100;
			pucStr[2] = puiUw[0] / 100;
			pucStr[3] = puiUw[1] % 100;
			pucStr[4] = puiUw[1] / 100;
			pucStr[5] = puiUw[2] % 100;
			pucStr[6] = puiUw[2] / 100;
			pucStr[7] = ++Setting_Num;
			
			I2C_24C02_Write(pucStr, 0, 8);
		}
		Flag_Usart1 = 1;
	}
}
void S2_Proc(void)
{
	if(State)
	{
		if(puiUw[State - 1] < 1000)
		{
			puiUw[State - 1]++;
		}
	}
}

void S3_Proc(void)
{
	if(State)
	{
		if(puiUw[State - 1] > 0)
		{
			puiUw[State - 1]--;
		}
	}
}

void Led1_Proc(void)
{
	if((uwTick - uwTick_Led1_Set_Point) <= 800)
		return;
	uwTick_Led1_Set_Point = uwTick;
	
	ucLed ^= 0x01;
}

void Led2_Proc(void)
{
	if((uwTick - uwTick_Led1_Set_Point) <= 400)
		return;
	uwTick_Led1_Set_Point = uwTick;
	
	ucLed ^= 0x01;
}

void Led_Proc(void)
{
	if(!Flag_Led)
		Led1_Proc();
	else
		Led2_Proc();
	
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

