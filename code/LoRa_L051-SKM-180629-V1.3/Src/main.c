/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "common.h"
//#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//LPTIM_HandleTypeDef hlptim1;

//RTC_HandleTypeDef hrtc;

//SPI_HandleTypeDef hspi1;

u32 RTC_Cnt = 0;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static 
//void MX_GPIO_Init(void);
//static 

//static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern u8 bMcuInit;
/* USER CODE END 0 */

//float a = 120.45f;  
//u8 * b = (u8*)&a;  

//for(u8 i = 0; i < 4; i++)
//{		
//	PRINTF("0x%02x ", b[i]);
//}

//u8 b[12] = "120.45";
//u8* c = "032125.000";
//float c;

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_TIM21_Init();
  BSP_LSI_Calibrator();
  RTC_Cali(BSP_Get_RTCFreq());

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//   MX_GPIO_Init();
	HW_RTC_Init();
    MX_ADC_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();      //BD/GPS uart
	MX_USART2_UART_Init();      //debug uart
//	MX_LPUART1_UART_Init();      ////LPUART for HB Uart
	Uart_Enable();

//	Uart2_Send("hello!\r\n", 9);
//	Uart_Send("hello!\r\n", 9);

	
//	LF_Init();
	GPS_GPIO_Init();

	PWM_GPIO_Init();
	LED_GPIO_Init();
	Key_GPIO_INT_Init();

//	SPK_GPIO_Init();	 
	Pwr_Init();
	Time_Init();

	OHR_init();
	LoRa_Init();

	LED_R_ON();	
  	HAL_Delay(200);    //for debug  	
	LED_R_OFF();	
	
  	LED_G_ON();	
  	HAL_Delay(200);    //for debug  	
	LED_G_OFF();	

	LED_B_ON();	
  	HAL_Delay(200);    //for debug  	
	LED_B_OFF();	

	SPK_Show(Flash, 1, SPK_100MS);
//	GPS_VCC_ON();

//	PRINTF("0x%x\r\n", GpstoHex(b));
//	PRINTF("0x%x\r\n", GpstoHex(c));

	
//	PRINTF(b);
//	c = atof(b);
//	PRINTF("%.3lf\r\n", c);

//  	HAL_Delay(5000);    //for debug
  /* USER CODE BEGIN 2 */
//	Hw_Init();

//  	LPMoveCount_Start();
//    LF_AS3932Init();	//LF enable and diable

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */	
	
	RTC_Cnt = HW_RTC_Tick2ms(HW_RTC_GetTimerValue());

	////// sys and debug //////
	Store_UART_Data(RTC_Cnt);
	Process_UART_Data(RTC_Cnt);//(RTC_Cnt);//(HAL_GetTick());

	#if 1
	AwarePro_Join(RTC_Cnt);//(HAL_GetTick());
	AwarePro_Run(RTC_Cnt);//(HAL_GetTick());
	Process_LoRa_Data(RTC_Cnt);//(HAL_GetTick());
	#endif

	Process_OHR(RTC_Cnt);
//	OHR_event_handler();

	/////  motion  //////////
	Process_Motion(RTC_Cnt);
	
	////////  LF  ////////
	Process_LF(RTC_Cnt);
	
	//// register /////
	Process_Reg(RTC_Cnt);

	///// heartbeat  ///////
	Process_HeartBeat(RTC_Cnt);
	
	//////// bind  //////////
	Process_Bind(RTC_Cnt);
	
	//// LED  /////////
	Process_LED_Event(RTC_Cnt);

	///////// Key  /////////
	Process_Key(RTC_Cnt);

	////  spk //////
	Process_PWM_SPK(RTC_Cnt);

	///pwr ////
	Process_Pwr(RTC_Cnt);     //babytag
//	Process_Bat(RTC_Cnt);

	/// work mode /////
	Process_WorkMode(RTC_Cnt);

	/////// GPS/DB ///
	Process_GPS(RTC_Cnt);

	/////// poll net and rejoin ///
	Process_Net_Status(RTC_Cnt);
	
	//////// Tag Drop Alarm ////
	Process_Tag_Alarm(RTC_Cnt);
	
	Systick_Event(RTC_Cnt);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;//RCC_HSI_ON;//RCC_HSI_DIV4;//;  //RCC_HSI_ON;//
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
//  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC
                                       |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2; 
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);     //  1000

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /// set wake up clock HSI  
  SET_BIT(RCC->CFGR, RCC_CFGR_STOPWUCK);  //add by dozen 20171220
  
}

/* LPTIM1 init function */
//static void MX_LPTIM1_Init(void)
//{

//  hlptim1.Instance = LPTIM1;
//  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
//  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
//  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
//  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
//  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
//  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
//  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//}

/* RTC init function */
//static void MX_RTC_Init(void)
//{

//  RTC_TimeTypeDef sTime;
//  RTC_DateTypeDef sDate;

//    /**Initialize RTC Only 
//    */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = 127;
//  hrtc.Init.SynchPrediv = 255;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Initialize RTC and set the Time and Date 
//    */
//  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 0x1;
//  sDate.Year = 0x0;

//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
//  }

//}

/* SPI1 init function */
//static void MX_SPI1_Init(void)
//{

//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 7;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
