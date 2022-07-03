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
#include "usart.h"
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
__IO uint32_t uwTick_Led_Set_Point;
__IO uint32_t uwTick_Usart_Set_Point;

unsigned char ucState, ucState_1;
unsigned char pucStr[21];
unsigned char ucKey_Val, ucKey_Down, ucKey_Old;
unsigned char ucNum_CNBR, ucNum_VNBR, ucNum_IDLE = 8;
double Ratio_CNBR = 3.50, Ratio_VNBR = 2.00;
unsigned char ucLed;
uint8_t RX_BUF[200];
uint8_t Rx_Counter;
char str_str[40];
uint8_t Rx_Counter;
uint8_t rx_buffer;
uint8_t year_temp, month_temp, day_temp, hour_temp, min_temp, sec_temp;
uint8_t car_id[5];
uint8_t car_type[5];
unsigned char i_1, i_2;

typedef struct
{
	uint8_t type[5];
	uint8_t id[5];
	uint8_t year_in;
	uint8_t month_in;
	uint8_t day_in;
	uint8_t hour_in;
	uint8_t min_in;
	uint8_t sec_in;
	_Bool notEmpty;
} Car_Data_Storage_Type;

Car_Data_Storage_Type Car_Data_Storage[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Key_Proc(void);
void Led_Proc(void);
_Bool CheckCmd(uint8_t *str);
void substr(uint8_t *d_str, uint8_t *s_str, uint8_t locate, uint8_t length);
uint8_t isExist(uint8_t *str);
uint8_t FindLocate(void);
void Usart_Proc(void);
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
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(&rx_buffer), 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Lcd_Proc();
		Key_Proc();
		Led_Proc();
		Usart_Proc();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Lcd_Proc(void)
{
	if(uwTick - uwTick_Lcd_Set_Point <= 100)
		return;
	
	uwTick_Lcd_Set_Point = uwTick;
	
	if(!ucState)
	{
		LCD_DisplayStringLine(Line1, (u8*)"       Data         ");
		sprintf((char*)pucStr, "   CNBR:%1u          ", ucNum_CNBR);
		LCD_DisplayStringLine(Line3, (u8*)pucStr);
		sprintf((char*)pucStr, "   VNBR:%1u          ", ucNum_VNBR);
		LCD_DisplayStringLine(Line5, (u8*)pucStr);
		sprintf((char*)pucStr, "   IDLE:%1u          ", ucNum_IDLE);
		LCD_DisplayStringLine(Line7, (u8*)pucStr);
	}
	else
	{
		LCD_DisplayStringLine(Line1, (u8*)"       Para         ");
		sprintf((char*)pucStr, "   CNBR:%3.2lf        ", Ratio_CNBR);
		LCD_DisplayStringLine(Line3, (u8*)pucStr);
		sprintf((char*)pucStr, "   VNBR:%3.2lf        ", Ratio_VNBR);
		LCD_DisplayStringLine(Line5, (u8*)pucStr);
		sprintf((char*)pucStr, "                    ");
		LCD_DisplayStringLine(Line7, (u8*)pucStr);
	}
}

void Key_Proc(void)
{
	if(uwTick - uwTick_Key_Set_Point <= 100)
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
				Ratio_CNBR += 0.5;
				Ratio_VNBR += 0.5;
			}
		break;
		
		case 3:
			if(ucState)
			{
				Ratio_CNBR -= 0.5;
				if(Ratio_CNBR < 0)
					Ratio_CNBR = 0;
				Ratio_VNBR -= 0.5;
				if(Ratio_VNBR < 0)
					Ratio_VNBR = 0;
			}
		break;
			
		case 4:
			if(!ucState_1)
			{
				ucState_1 = 1;
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
			}
			else
			{
				ucState_1 = 0;
				__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 100);
			}
		break;
	}
}

void Led_Proc(void)
{
	if(uwTick - uwTick_Led_Set_Point <= 100)
		return;
	
	uwTick_Led_Set_Point = uwTick;
	
	if(!ucState_1)
		ucLed &= ~0x02;
	else
		ucLed |= 0x02;
	
	Led_Disp(ucLed);
}

_Bool CheckCmd(uint8_t *str)
{
	if(Rx_Counter != 22)
		return 0;
	if((str[0] == 'C' || str[0] == 'V') && (str[1] == 'N') && (str[2] == 'B') && (str[3] == 'R') && (str[4] == ':') && (str[9] == ':'))
	{
		for(int i = 10; i < 22; i++)
		{
			if(str[i] > '9' || str[i] < '0')
				return 0;
		}
	}
	
	return 1;
}

void substr(uint8_t *d_str, uint8_t *s_str, uint8_t locate, uint8_t length)
{
	for(int i = 0; i < length; i++)
	{
		d_str[i] = s_str[locate + i];
	}
	d_str[length] = '\0';
}

uint8_t isExist(uint8_t *str)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		if((strcmp((const char*)str, (const char*)Car_Data_Storage[i].id)) == 0)
		{
			return i;
		}
	}
	return 0xFF;
}

uint8_t FindLocate(void)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		if(Car_Data_Storage[i].notEmpty == 0)
			return i;
	}
	return 0xFF;
}

void Usart_Proc(void)
{
	if(uwTick - uwTick_Usart_Set_Point <= 100)
		return;
	
	uwTick_Usart_Set_Point = uwTick;
	
	if(CheckCmd(RX_BUF))
	{
		year_temp = (RX_BUF[10] - '0') * 10 + (RX_BUF[11] - '0');
		month_temp = (RX_BUF[12] - '0') * 10 + (RX_BUF[13] - '0');
		day_temp = (RX_BUF[14] - '0') * 10 + (RX_BUF[15] - '0');
		hour_temp = (RX_BUF[16] - '0') * 10 + (RX_BUF[17] - '0');
		min_temp = (RX_BUF[18] - '0') * 10 + (RX_BUF[19] - '0');
		sec_temp = (RX_BUF[20] - '0') * 10 + (RX_BUF[21] - '0');
		
		if((month_temp > 12) || (day_temp> 31) || (hour_temp > 23) || (min_temp > 59) || (sec_temp > 59))
		{
			i_1 = 1;
			goto SEND_ERROR;
		}
		
		substr(car_id, RX_BUF, 5, 4);
		substr(car_type, RX_BUF, 0, 4);
		
		if(isExist(car_id) == 0xFF)
		{
			uint8_t locate = FindLocate();
			
			if(locate == 0xFF)
			{
				i_1 = 2;
				goto SEND_ERROR;
			}
			substr(Car_Data_Storage[locate].type, car_type, 0, 4);
			substr(Car_Data_Storage[locate].id, car_id, 0, 4);
			Car_Data_Storage[locate].year_in = year_temp;
			Car_Data_Storage[locate].month_in = month_temp;
			Car_Data_Storage[locate].day_in = day_temp;
			Car_Data_Storage[locate].hour_in = hour_temp;
			Car_Data_Storage[locate].min_in = min_temp;
			Car_Data_Storage[locate].sec_in = sec_temp;
			Car_Data_Storage[locate].notEmpty = 1;
			
			if(Car_Data_Storage[locate].type[0] == 'C')
				ucNum_CNBR++;
			else if(Car_Data_Storage[locate].type[0] == 'V')
				ucNum_VNBR++;
			
			ucNum_IDLE--;
		}
		else
		{
			int64_t Second_derta;
			uint8_t in_locate = isExist(car_id);
			
			if(strcmp((const char*)car_type, (const char*)Car_Data_Storage[in_locate].type) != 0)
			{
				i_1 = 3;
				goto SEND_ERROR;
			}
			Second_derta = (year_temp - Car_Data_Storage[in_locate].year_in) * 365 * 24 * 60 * 60 +\
										 (month_temp - Car_Data_Storage[in_locate].month_in) * 30 * 24 * 60 * 60 +\
										 (day_temp - Car_Data_Storage[in_locate].day_in) * 24 * 60 * 60 +\
										 (hour_temp - Car_Data_Storage[in_locate].hour_in) * 60 * 60 +\
										 (min_temp - Car_Data_Storage[in_locate].min_in) * 60 +\
										 (sec_temp - Car_Data_Storage[in_locate].sec_in);
			if(Second_derta < 0)
			{
				i_1 = 4;
				goto SEND_ERROR;
			}
			Second_derta = (Second_derta + 3599) / 3600;
			sprintf((char*)str_str, "%s:%s:%d:%.2lf\r\n", Car_Data_Storage[in_locate].type, Car_Data_Storage[in_locate].id, (unsigned int)Second_derta, ((double)Second_derta * (Car_Data_Storage[in_locate].type[0] == 'C' ? Ratio_CNBR : Ratio_VNBR)));
			HAL_UART_Transmit(&huart1,(unsigned char *)str_str, strlen((char*)str_str), 50);
			if(Car_Data_Storage[in_locate].type[0] == 'C')
				ucNum_CNBR--;
			else if(Car_Data_Storage[in_locate].type[0] == 'V')
				ucNum_VNBR--;
			
			ucNum_IDLE++;
			
			memset(&Car_Data_Storage[in_locate],0,sizeof(Car_Data_Storage[in_locate]));
		}
		goto CMD_YES;
		
		SEND_ERROR:
		sprintf(str_str, "Error:%1u\r\n", i_1);
			HAL_UART_Transmit(&huart1,(unsigned char *)str_str, strlen(str_str), 50);
		CMD_YES:
			memset(&RX_BUF[0], 0, sizeof(RX_BUF));
			Rx_Counter = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RX_BUF[Rx_Counter] = rx_buffer;
	Rx_Counter++;
	HAL_UART_Receive_IT(&huart1, (uint8_t *)(&rx_buffer), 1);
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

