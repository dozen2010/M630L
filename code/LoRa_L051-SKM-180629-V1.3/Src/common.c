/*

Description: app driver, app protocol

License: 

Maintainer: Hangzhou AwareTag tech
*/
 /******************************************************************************
  * @file    common.c
  * @author  dozen Yao
  * @version V0.0.1
  * @date    2017/09/21
  * @brief   App Protocol and API
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "common.h"


const char AtOpMode[]        = "AT#RFMODE";               //Read LoRa operate mode
const char AtSleep[]         = "AT#SLEEP";               //SET LoRa SLEEP mode
const char AtStandby[]       = "AT#STANDBY";               //SET STANDBY mode
const char AtRxTime[]        = "AT#RXTIME";               //SET RXTIME
const char AtRst[]           = "AT#RST";               //SET reset
const char AtVer[]           = "AT#VER";               //read version
const char AtBat[]           = "AT#BAT";               //read battery 0-254
const char AtUid[]           = "AT#UID";               //read unique id
const char AtCmac[]          = "AT#CUID";               //read coo MAC id
const char AtNodeTab[]       = "AT#NDTB";               //read coo MAC id
const char AtPollBuf[]       = "AT#PBUF";               //read coo MAC id
const char AtDefault[]       = "AT#DEF";               //read coo MAC id
const char AtRFRate[]        = "AT#RFRATE";               //read coo MAC id
const char AtRFCH[]          = "AT#RFCH";
const char AtPWR[]           = "AT#PWR";
const char AtPID[]           = "AT#PID";               //
const char AtJoin[]          = "AT#JN";               //
const char AtPollGap[]       = "AT#PGAP";               //  ZED Poll GAP
const char AtBeatGap[]       = "AT#HBGAP";               //  Router heartbeat GAP
const char AtLeave[]         = "AT#DEL";               //  Router heartbeat GAP

// for test
const char AtPoll[]          = "AT#POLL";               //
const char AtMRST[]          = "AT#MRST";               //
const char AtCALI[]          = "AT#CAL";               //
const char AtRFIni[]         = "AT#RFINI";          
const char AtIni[]		     = "AT#INI";		
const char AtTst[]		     = "AT#TST";		
const char Atbind[]		     = "AT#BIND";		
const char AtOHR[]		     = "AT#OHR";		
const char AtGPS[]		     = "AT#GPS";		

const FuncReg_t FuncReg = 
{
	AwarePro_NV_Init,
	Process_App_RF_Rec,
	Process_RF_Tx,	
	Process_Join_Status,
	Process_App_Ack,
	Process_Cmd_Status,
	#if defined(COO)
	Node_Change,
	Process_App_RF_RT_Rec,
	#endif	
	Process_Error,
};


//////////////  UART /////////////////////

GPS_BUFFER asGPS_Buffer[UART_BUFFER_SIZE] = {0};

UART_BUFFER	 asUART0_Buffer[UART_BUFFER_SIZE] = {0};
//UART_BUFFER	 asUART2_Buffer[UART_BUFFER_SIZE] = {0};
//UART_BUFFER	 asLPUART1_Buffer[UART_BUFFER_SIZE] = {0};

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef hlpuart1;

/////// DMA ////////////
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_lpuart1_rx;

//////iic  //////////
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

//////// Rtc /////////////
u16 RTC_Sec_Cnt = 1024;    //default RTC Value

/////////////  LoRa  //////////////////  

u8 Event_Flg = 0;

static RadioEvents_t RadioEvents;
static TimerEvent_t Period;

NV_Config_Typedef RFConfig;

RF_Rx_Typedef LoRaRxBuffer[RF_RX_BUF_NUM];
__align(4) u8 LoRaTxBuffer[LORA_MAX_LEN];     //flash unit 32bit


Location_Typedef Loc;

SysStatus_Typedef SysStatus;
Pwr_Typedef Pwr;

/////////// gps/bd //////////
GPS_Typedef Gps = {0};
GPStoSer_Typedef Gps_Rp = {0};
GPS_Status_Typedef GpsStatus = {0};

///////// heartrate ////////
HR_Typedef HB = {0};
vu8 ucHeart_Status;

///////// key ////////////
//u8 Key_Status = 0;
Key_typedef Func_Key;
Key_typedef Match_Key;

///////// led, speek //////
Opt_typedef RedLED;
Opt_typedef GreenLED;
Opt_typedef BlueLED;
Opt_typedef Speek;

///// SN  ////////////
u8 SN[4] __attribute__((at(TAG_SN_ID_ADDR)))=
{
	0x55,0x66,0x77,0x88
};

///// extern ///////////
extern TimerEvent_t timer_ohr;

////  for test //

//u8 TestFlg = 0;
//u16 Cnt     = 0;
//u16 fail    = 0;


static u8 Check_Sum(u8* Data, u8 Len);
static void OnPollData(void);


void Time_Init(void)
{
	
	TimerInit(&Period, OnPollData);   
}


static void OnPollData(void)
{
//	PRINTF("15s Gap WakeUp!\r\n");

//	WakeUpCnt++;

//	if(Sys_Status == MOVE_STATUS)
//	{
//		if(WakeUpCnt % 30 == 0)
//		{
//			if(AwarePro_NV_JoinFlg() == NET_FLG)
//			{
//				AwarePro_ZED_Poll_Once();
//				
//			}
//		}
//	}
		
}

void Get_TagSN(uint8_t* sn)
{

    sn[0] = (uint8_t)(TAG_SN_Type>>24);
    sn[1] = (uint8_t)(TAG_SN_Type>>16);
    sn[2] = (uint8_t)(TAG_SN_Type>>8);
    sn[3] = (uint8_t)(TAG_SN_Type);   
	sn[4] = SN[0];
    sn[5] = SN[1];
    sn[6] = SN[2];
    sn[7] = SN[3];

}

void Dev_Event(u32 ulSysTime)
{
	static u32 ulPretick = 0;

	if(Event_Flg == EVN_IDLE)
	{
		ulPretick = ulSysTime;
	}
	else if(Event_Flg == EVN_RST)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(3))
		{
			ulPretick = ulSysTime;
			
			Event_Flg = EVN_IDLE;
		
			NVIC_SystemReset();
		}
		

	}
	else if(Event_Flg == EVN_SET_CH)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(3))
		{
			ulPretick = ulSysTime;
			
			Event_Flg = EVN_IDLE;

			#if defined(DEBUG)
			Delete_NV();
			Write_App_NV();
			#endif
			
			RF_CH_Config(GetCH(), GetBW());   

			NVIC_SystemReset();
		
		}
		

	}
	else if(Event_Flg == EVN_SET_TXRATE)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(3))
		{
			ulPretick = ulSysTime;
			
			Event_Flg = EVN_IDLE;
			
			RF_DataRate_Config((RF_Rate_Enum)GetRFRate(), &RFConfig.ucBand_Width, GetRFPwr());
		
		}
		

	}
	else if(Event_Flg == EVN_SET_TXPWR)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(3))
		{
			ulPretick = ulSysTime;
			
			Event_Flg = EVN_IDLE;

			RF_DataRate_Config((RF_Rate_Enum)GetRFRate(), &RFConfig.ucBand_Width, GetRFPwr());
		
		}
		

	}
	else if(Event_Flg == EVN_EXIT_SEEK)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(60))   //60s
		{
			ulPretick = ulSysTime;
			
			Event_Flg = EVN_IDLE;

			SysStatus.ucWork_Status = WORK;
		}
		

	}
	else if(Event_Flg == EVN_TEST)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(3))
		{
			ulPretick = ulSysTime;
			SysStatus.ucWork_Status  = TEST;
			
			Event_Flg = EVN_IDLE;
		}
	}
	else if(Event_Flg == EVN_OHR)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(5))
		{
			ulPretick = ulSysTime;
			
			StartOHR(2, 10);
			Event_Flg = EVN_IDLE;
		}
	}
	
	else
	{
		Event_Flg = EVN_IDLE;
	}

}

u8 HeartRate_Idle(void)
{
	if(((OHR_status == OHR_STATUS_DISABLED)||(OHR_status == OHR_STATUS_ENABLED))
		&&(!ucHeart_Status))
	{

		return 1;
	}
	else
	{
//		PRINTF("OHR:%d,HB:%d.\r\n",OHR_status, ucHeart_Status);
		return 0;
	}
}

u8 GPS_Idle(void)
{

	if(GpsStatus.ucIdle == FALSE)
	{

		return 1;
	}
	else
	{
//		PRINTF("OHR:%d,HB:%d.\r\n",OHR_status, ucHeart_Status);
		return 0;
	}
}

void Systick_Event(u32 ulSysTime)
{
	
	Dev_Event(ulSysTime);
	
	#if defined(ZED)

	u32 WakeGap = 0;
//    DISABLE_IRQ( );
	 /* if an interupt has occured after __disable_irq, it is kept pending 
     * and cortex will not enter low power anyway  */
     
//	if(bMcuInit)
	{
		if((AwarePro_DevStatus())
			&&(KEY_Idle())
			&&(LED_Idle())
			&&(SPK_Idle())
			&&(Register_Done())
			&&(Bind_Done())
			&&(LF_Done())
			&&(HeartRate_Idle())
			&&(GPS_Idle())
			&&(RF_TX_RUNNING != Radio.GetStatus()))
		{
			
			Radio.Sleep();

			if((SysStatus.ucWork_Status == SLEEP)
				||(SysStatus.ucWork_Status == CHG))
			{
				WakeGap = (RTC_Sec_Cnt*2);//(RT0C_Sec_Cnt >> 1);  //RTC_Sec_Cnt*60;//
				TimerSetValue(&Period, WakeGap);		
				TimerStart(&Period );
			}

			
//			PRINTF("sleep,rtc:%d!\r\n", (u16)ulSysTime);
			LowPower_Handler(); 	//mcu stop mode			
//			PRINTF("wakeup,rtc:%d!\r\n", (u16)HW_RTC_Tick2ms(HW_RTC_GetTimerValue()));
		}
	}
//	else
//	{

//		if((LED_Idle())
//		&&(SPK_Idle())
//		&&((SysStatus.ucBand_Status & ADC_STATUS) == 0))
//		{
//					
//			WakeGap = RTC_Sec_Cnt*60;//(RTC_Sec_Cnt >> 1);
//			TimerSetValue(&Period, WakeGap);		
//			TimerStart(&Period );
//				
//			LowPower_Handler(); 	//mcu stop mode

//		}
//	}

		
//	ENABLE_IRQ( );

	#endif
}


void OnTxDone( void )
{
	//idle status --> Rx  clear fifo 

	Radio.Sleep();     //ZED can join Router Node.tx--rx change bug
	
	#if defined(ZED)

		
	Radio.Rx( RX_TIMEOUT_VALUE );


	#else
	
	Radio.Rx( RX_TIMEOUT_VALUE );

	#endif

	AwarePro_Tx_Done_Process();
	
	#if defined(DEBUG)

//	PRINTF("OnTxDone\n");

	#endif

}



void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{

	if(LoRaRxBuffer[0].ucLen == 0)
	{
		LoRaRxBuffer[0].ucLen = (u8)size;
		LoRaRxBuffer[0].scSnr = snr;
		LoRaRxBuffer[0].siRssi = rssi;
//		LoRaRxBuffer[0].pPayload = payload;
	
		memcpy(LoRaRxBuffer[0].aucPayload, payload, size);

	}
	else
	{
		
		PRINTF("buf0 full!\r\n");
	}

	////////// test//
//	LED_Show(Green_Led, Flash, 1, FLASH_50MS);
	
	AwarePro_Rx_Done_Process();
	
	#if defined(ZED)

//	if(Nv_Para.ucJoin_Flag != NET_FLG)
	{
		Radio.Sleep();
		Radio.Rx( RX_TIMEOUT_VALUE );
	}
//	else
//	{
//		
//		Radio.Sleep();
//	}
	#else
	
	Radio.Sleep();
	Radio.Rx( RX_TIMEOUT_VALUE );
	
	#endif

	
//	PRINTF("Rssi:%d, Snr:%d", rssi, snr);

}

void OnTxTimeout( void )
{

	#if defined(ZED)

	Radio.Sleep();	
	Radio.Rx( RX_TIMEOUT_VALUE );
	
	#else

	Radio.Sleep();
	Radio.Rx( RX_TIMEOUT_VALUE );
	
	#endif

	AwarePro_Tx_Timeout_Process();
 	
#if defined(DEBUG)
    PRINTF("OnTxTimeout\n");
#endif
}


void OnRxTimeout( void )
{
	AwarePro_Rx_Timeout_Process();

	#if defined(ZED)
		
	Radio.Sleep();	
//	Radio.Rx( RX_TIMEOUT_VALUE );

	#else
	
	Radio.Sleep();	
	Radio.Rx( RX_TIMEOUT_VALUE );
	
	#endif
}


void OnRxError( void )
{
	
#if defined(DEBUG)
    PRINTF("OnRxError\n"); 
#endif

	AwarePro_Rx_Error_Process();

	Radio.Sleep();                //fifo clear
	Radio.Rx( RX_TIMEOUT_VALUE );

}

/**
  * @brief This function return a random seed
  * @note based on the device unique ID
  * @param None
  * @retval see
  */
uint32_t HW_GetRandomSeed( void )
{
  return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

/**
  * @brief This function return a unique ID
  * @param unique ID
  * @retval none
  */
void HW_GetUniqueId( uint8_t *id )
{
//    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
//    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
//    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
//    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
//    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
//    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
//    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
//    id[0] = ( ( *( uint32_t* )ID2 ) );

		///stm32L0x
	id[7] = ( ( *( uint16_t* )ID1 )+ ( *( uint32_t* )ID4 ) ) >> 24;
    id[6] = ( ( *( uint16_t* )ID1 )+ ( *( uint32_t* )ID4 ) ) >> 16;
    id[5] = ( ( *( uint16_t* )ID2 )+ ( *( uint32_t* )ID4 ) ) >> 8;
    id[4] = ( ( *( uint16_t* )ID2 )+ ( *( uint32_t* )ID4 ) );
    id[3] = ( ( *( uint32_t* )ID3 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID3 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID3 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID3 ) );
	
}


u8 RF_CH_Config(u8 ch, u8 BW)
{

	u32 Freq = RF_FREQUENCY;  
	u32 OffSet_Freq = 0;

	if(BW != 0xFF)
	{
		if(0 == BW)
		{
			OffSet_Freq = (125000 << 3)*ch;   //125KHZ*2
		}
		else
		{
			OffSet_Freq = (250000 << 2)*ch;
		}
	}
	else
	{
		OffSet_Freq = (125000 << 3)*ch;   //125KHZ*2
	}

	Freq = RF_FREQUENCY + OffSet_Freq;

	Radio.SetChannel( Freq );

#if defined(DEBUG)

	PRINTF("Set Freq:%d\r\n",Freq);
#endif

	return 0;
}


u8 RF_DataRate_Config(RF_Rate_Enum ucRate, u8* BW, u8 RF_Pwr)
{
//	u8 BandWidth = 0;
	u8 SF = 0;
	
	if(RF_Rate_250 == ucRate)
	{
		*BW = 0;
		SF = 12;
	}

	else if(RF_Rate_440 == ucRate)
	{
		*BW = 0;
		SF = 11;
	}

	else if(RF_Rate_980 == ucRate)
	{
		*BW = 0;
		SF = 10;
	}

	else if(RF_Rate_1760 == ucRate)
	{
		*BW = 0;
		SF = 9;
	}

	else if(RF_Rate_3125 == ucRate)
	{
		*BW = 0;
		SF = 8;
	}

	else if(RF_Rate_5470 == ucRate)
	{
		*BW = 0;
		SF = 7;
	}

	else if(RF_Rate_11000 == ucRate)
	{
		*BW = 1;
		SF = 7;
	}
	else if(RF_Rate_20000 == ucRate)
	{
		*BW = 2;
		SF = 7;
	}

	else
	{
		*BW = 0;
		SF = 7;
	}


	Radio.SetTxConfig( MODEM_LORA, RF_Pwr, 0, *BW,
                       SF, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       true, 0, 0, LORA_IQ_INVERSION_ON, 5000 );         //(SX1276GetTimeOnAir(MODEM_LORA,255)+10) 

	Radio.SetRxConfig( MODEM_LORA, *BW, SF,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

//	Nv_Para.ucBand_Width = SET_MASK_BIT(Nv_Para.ucBand_Width);
//	Write_NV(Band_Width);
	PRINTF("Set BW:%d,SF:%d, RF_Pwr:%d\r\n", *BW, SF, RF_Pwr);

	return 0;

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */
  	__HAL_RCC_GPIOB_CLK_ENABLE();
  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;//GPIO_MODE_AF_OD;//GPIO_MODE_AF_PP;//GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;//GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  /* USER CODE END I2C1_MspInit 1 */
  }

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

    i2cHandle->State = HAL_I2C_STATE_READY;	  //  add by dozen  20180515
  	__HAL_RCC_GPIOB_CLK_ENABLE();
  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;//GPIO_MODE_AF_OD;//GPIO_MODE_AF_PP;//GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL;//GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C1 interrupt Init */
//    HAL_NVIC_SetPriority(I2C2_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  /* USER CODE END I2C1_MspInit 1 */
  }
}


void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */
  	__HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */
  
    i2cHandle->State = HAL_I2C_STATE_RESET;	  //	add by dozen  20180515
  	__HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* I2C1 interrupt Deinit */
//    HAL_NVIC_DisableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
} 



/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A; // //400k
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* I2C1 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0010061A; // //400k
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
 
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;//19200;//19200;//115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;//19200;//115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 19200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
//static 
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);        //uart1
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

    /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);     //uart2,lpuart1
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  
//  GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();


    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   //DIV1 -->DIV4
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
//  sConfig.Channel = ADC_CHANNEL_17;
//  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
  
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  	GPIO_InitStruct.Pin  = SW_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

//	HAL_GPIO_Init(SW_GPIO_PORT, &GPIO_InitStruct);	
//	
//  	GPIO_InitStruct.Pin  = ADC_EN_PIN;
//	HAL_GPIO_Init(ADC_EN_GPIO_PORT, &GPIO_InitStruct);	

//	SW_OFF();
//	ADC_EN_OFF();
//	
//	HAL_ADC_Start(&hadc);
}


/**
  * @brief This function De-initializes the ADC
  * @param Channel
  * @retval Value
  */
uint16_t HW_AdcReadChannel( uint32_t Channel )
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;
  
//  if( AdcInitialized == true )
  {
    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};
      
    
	__HAL_RCC_ADC1_CLK_ENABLE( );

    
    /*calibrate ADC if any calibraiton hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED );
    
    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE; 
    HAL_ADC_ConfigChannel( &hadc, &adcConf);
      
    /* configure adc channel */
    adcConf.Channel = Channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel( &hadc, &adcConf);

    /* Start the conversion process */
    HAL_ADC_Start( &hadc);
      
    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion( &hadc, HAL_MAX_DELAY );
      
    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue ( &hadc);

    __HAL_ADC_DISABLE( &hadc) ;

    __HAL_RCC_ADC1_CLK_DISABLE();
  }
  return adcData;
}

//#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))

u8 HW_GetBatteryLevel(void)
{
	u16 Vref_Adc = 0;
	u16 Vdd = 0;
	u8 Level = 0;
	
	
//	Vref_Adc = HW_AdcReadChannel(ADC_CHANNEL_17);//HAL_ADC_GetValue ( &hadc);//HW_AdcReadChannel(ADC_CHANNEL_17);//
//	Vdd = (( (uint32_t) VDDA_TEMP_CAL * (*VREFINT_CAL ) )/ Vref_Adc);

	ADC_EN_ON();

//	HAL_Delay(10);
     // PA0
	Vref_Adc = HW_AdcReadChannel(ADC_CHANNEL_0);//HAL_ADC_GetValue ( &hadc);//HW_AdcReadChannel(ADC_CHANNEL_17);//

	Vdd = ((2 * Vref_Adc * BAT_REF_VOLT) >> ADC_ACC) + CALI_VOLT;
	
	PRINTF("Bat vdd:%d\r\n", Vdd);

	if(Vdd > BAT_FULL_VOLT)
	{
		Level = 100;
	}
	else if(Vdd <= BAT_MIN_VOLT)
	{
		Level = 1;
	}
	else
	{
		Level = (Vdd - BAT_MIN_VOLT)/5;
	}

	ADC_EN_OFF();
	
	return (Level);
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0
    PA2     ------> ADC_IN2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */
    GPIO_InitStruct.Pin  = ADC_EN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(ADC_EN_GPIO_PORT, &GPIO_InitStruct);	

	ADC_EN_OFF();
	

  /* USER CODE END ADC1_MspInit 1 */
  }
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

//	GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0
    PA2     ------> ADC_IN2 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_0);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

//u8 buf[64] = {0};

void Uart_Enable(void)
{
	//enable DMA mode and Idle IT mode
	/// add by dozen 

	//// debug uart ///
	__HAL_UART_FLUSH_DRREGISTER(&huart2);  //clear data reg	
   HAL_UART_Receive_DMA(&huart2, asUART0_Buffer[0].aucData, UART_FRAME_SIZE+1);
  	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);	  //使能空闲中断
//	__HAL_UART_FLUSH_DRREGISTER(&huart1);  //clear data reg

//    HAL_UART_Receive_DMA(&huart1, asGPS_Buffer[0].aucData, GPS_FRAME_SIZE+1);
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);	  //使能空闲中断

	//////// bd/gps  uart  ////
	__HAL_UART_FLUSH_DRREGISTER(&huart1);  //clear data reg
   HAL_UART_Receive_DMA(&huart1, asGPS_Buffer[0].aucData, GPS_FRAME_SIZE+1);
  	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);	  //使能空闲中断

	///////// heartbeat uart ////
//	__HAL_UART_FLUSH_DRREGISTER(&hlpuart1);  //clear data reg
//   HAL_UART_Receive_DMA(&hlpuart1, asLPUART1_Buffer[0].aucData, UART_FRAME_SIZE+1);
//  	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);	  //使能空闲中断
}

void Pwr_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();

	 GPIO_InitStruct.Pin = CHG_PIN;
	 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(CHG_GPIO_PORT, &GPIO_InitStruct);

	 GPIO_InitStruct.Pin = CHG_CHK_PIN; 
	 HAL_GPIO_Init(CHG_CHK_PORT, &GPIO_InitStruct);
	
}

void Pwr_DeInit(void)
{

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();	
	__HAL_RCC_GPIOH_CLK_ENABLE();

	 HAL_GPIO_DeInit(CHG_GPIO_PORT, CHG_PIN);
	 HAL_GPIO_DeInit(CHG_CHK_PORT, CHG_CHK_PIN);
	
}

void Hw_Init(void)
{
	
//	HW_SPI_Init();
	LoRa_SPISetup();
	Radio.IoInit();
	Pwr_Init();	
 	HAL_ADC_Init(&hadc);
//	HAL_UART_Init(&huart1);
	
	HAL_UART_MspInit(&huart2);
//	HAL_UART_MspInit(&huart2);
//	HAL_UART_Init(&huart2);
//	SPK_GPIO_Init();
//	LF_SPI_GPIO_Init();

//	if(GpsStatus.Run_Status == TRUE)
	{
//		HAL_UART_Init(&huart2);
	}
	
//	if((SysStatus.ucActived == TRUE)
	if((SysStatus.ucWork_Status == WORK)
		&&(RFConfig.ucReg_Status == Reg_Done))
	{

		if((SysStatus.ucBand_Status & CONNECT_STATUS) == CONNECT_STATUS)
		{
//			Touch_GPIO_Init();
		}
	}
	

/*	
	if(bMcuInit == TRUE)      //actived
	{
		//	MX_ADC_Init();
	//	HW_RTC_Init();
	//	HW_SPI_Init();
	
//		LoRa_SPISetup();
//		Radio.IoInit();
		
//		MX_DMA_Init();
//		MX_USART1_UART_Init();
//		Uart_Enable();
//		LED_GPIO_Init();
//		SPK_GPIO_Init();	
	//	Touch_GPIO_Init();
		LPMoveCount_Start();	
		
//		LoRa_Init();
	
	}
*/
	

}

/**
  * @brief This function Deinitializes the hardware
  * @param None
  * @retval None
  */
void HW_DeInit( void )
{
	HW_SPI_DeInit( );
	Radio.IoDeInit( );
	Pwr_DeInit();
 	HAL_ADC_DeInit(&hadc);
//	HAL_UART_DeInit(&huart1);	
//	HAL_UART_DeInit(&huart2);
	HAL_I2C_MspDeInit(&hi2c1);

	HAL_UART_MspDeInit(&huart2);
	
	if(GpsStatus.Run_Status != TRUE)
	{
		
		HAL_UART_MspDeInit(&huart1);
	}
//	HAL_UART_DeInit(&huart2);	   // can not enable OHR mode ??
//	LF_AS3932DeInit();
//	LF_SPI_GPIO_DeInit();

	if((SysStatus.ucWork_Status == WORK)
		&&(RFConfig.ucReg_Status == Reg_Done))
	{

		if((SysStatus.ucBand_Status & CONNECT_STATUS) == CONNECT_STATUS)
		{
//			Touch_GPIO_DeInit();
		}
		
	}
	else
	{
//		HW_SPI_DeInit( );
//		LF_AS3932DeInit();
	}
	
	
//	Key_GPIO_INT_DeInit();

//	LED_GPIO_DeInit();
//	SPK_GPIO_DeInit();
//  vcom_DeInit( );
//  McuInitialized = false;
}

void GPIO_DeInit(void)
{
	GPIO_InitTypeDef initStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	initStruct.Mode =GPIO_MODE_OUTPUT_PP;
	initStruct.Pull =GPIO_NOPULL  ; 
	HW_GPIO_Init ( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct ); 
	HW_GPIO_Write( RADIO_MOSI_PORT, RADIO_MOSI_PIN, 0 );

	initStruct.Pull =GPIO_PULLDOWN; 
	HW_GPIO_Init ( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct ); 
	HW_GPIO_Write( RADIO_MISO_PORT, RADIO_MISO_PIN, 0 );

	initStruct.Pull =GPIO_NOPULL  ; 
	HW_GPIO_Init ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct ); 
	HW_GPIO_Write(  RADIO_SCLK_PORT, RADIO_SCLK_PIN, 0 );

	initStruct.Pull =GPIO_PULLUP  ; 
	HW_GPIO_Init ( RADIO_NSS_PORT, RADIO_NSS_PIN , &initStruct ); 
	HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN , 1 );


	initStruct.Mode = GPIO_MODE_IT_RISING ; //GPIO_MODE_ANALOG;
	initStruct.Pull = GPIO_PULLDOWN;
	HW_GPIO_Init( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, &initStruct );

	initStruct.Mode = GPIO_MODE_INPUT;
	initStruct.Pull = GPIO_NOPULL;
	HW_GPIO_Init( AS_CS_PORT, AS_CS_PIN, &initStruct); 
	
}

void Init_SysConfig(void)
{
	Read_App_NV();

	if(Check_APP_NV_Para())
	{
		Delete_App_NV();
		Delete_NV();
		
		RFConfig.ucRF_Channel     = 0;
		RFConfig.ucBand_Width     = BW_125K;
		RFConfig.ucRF_Rate        = RF_Rate_5470;
		RFConfig.ucRF_Pwr         = TX_OUTPUT_POWER;
		RFConfig.ucHB_Indicate_EN = ENABLE;
		RFConfig.ucSleep_EN       = ENABLE;		
		RFConfig.ucReg_Status     = Reg_None;
		RFConfig.ucWork_Status    = 0;
//		RFConfig.ucBind_Status    = 0;
		RFConfig.ucStatic_Time    = STATIC_TIME;//TIME_S(STATIC_TIME);
		RFConfig.uiHB_Gap         = HB_TIME;//TIME_S(HB_TIME);  //HB_TIME
		RFConfig.uiMove_Threshold = MOTION_THRES;

		RFConfig.ucPati_Area      = 0;
		RFConfig.ucPati_Bed       = 0;
//		RFConfig.ulMatch_ID1      = DEF_MATCH_TAG_ID;		
//		RFConfig.ulMatch_ID2      = DEF_MATCH_TAG_ID;
//		RFConfig.ulMatch_ID3	  = DEF_MATCH_TAG_ID;
//		RFConfig.ulTAG_ID         = DEF_TAG_ID;		   //read from flash
//		osal_revmemcpy((u8*)&RFConfig.ulTAG_ID, &SN, sizeof(RFConfig.ulTAG_ID));

		osal_memset((u8*)&RFConfig.aucMatch_SN1, 0, sizeof(Dev_Addr));
		osal_memset((u8*)&RFConfig.aucMatch_SN2, 0, sizeof(Dev_Addr));
		osal_memset((u8*)&RFConfig.aucMatch_SN3, 0, sizeof(Dev_Addr));

		Get_TagSN((u8*)&RFConfig.aucTag_SN);

		Write_App_NV();
		
	}
	else
	{
		
		Get_TagSN((u8*)&RFConfig.aucTag_SN);
	}

	SysStatus.ucWork_Status = RFConfig.ucWork_Status;
	
	LoRa_Config(&RFConfig);

	PRINTF("LoRa RF Config!\r\n");
}


void LoRa_Init(void)
{
	
	//gpio
	LoRa_SPISetup();
//	HW_SPI_Init();
	Radio.IoInit();
	
	 // Radio initialization
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init( &RadioEvents );

	Init_SysConfig();
	/////////  aware protocol  init//
//	HW_GetUniqueId((u8*)&DeviceAddr);
	
//	Get_TagSN((u8*)&DeviceAddr);
	
	Radio.SetMaxPayloadLength(MODEM_LORA, LORA_MAX_LEN);     //add by dozen  20180611
	
	FuncReg.Para_Init(POLL_GAP_S, HEARTBEAT_GAP_S, Radio.Random(), &RFConfig.aucTag_SN);

	AwarePro_Init(&FuncReg);

//	PRINTF("RX_TIMEOUT:%d!\r\n", RX_TIMEOUT_VALUE);

	PRINTF("UID:");
	print_8_02x((u8*)&RFConfig.aucTag_SN);	

	
	PRINTF("M_SN1:");
	print_8_02x((u8*)&RFConfig.aucMatch_SN1);
	PRINTF("M_SN1 Status:%d\r\n", RFConfig.ucMatch1_Status);
	
	PRINTF("M_SN2:");
	print_8_02x((u8*)&RFConfig.aucMatch_SN2);	
	PRINTF("M_SN2 Status:%d\r\n", RFConfig.ucMatch2_Status);
	
	PRINTF("M_SN3:");
	print_8_02x((u8*)&RFConfig.aucMatch_SN3);	
	PRINTF("M_SN3 Status:%d\r\n", RFConfig.ucMatch3_Status);
//	print_8_02x((u8*)FuncReg.DeviceAddr);	

//	PRINTF("Reset!\r\n");

	PRINTF("PID:0x%02x!\r\n", AwarePro_GetPID());	
	PRINTF("Join:0x%02x\r\n", AwarePro_NV_JoinFlg());	
	PRINTF("Register:0x%02x\r\n", RFConfig.ucReg_Status);
	PRINTF("HostAddr:");
	
	print_8_02x((u8*)AwarePro_GetHostExtAddr());
	
	PRINTF("NwkAddr:0x%04x\r\n", AwarePro_GetNwkAddr());
	PRINTF("RF_Channel:0x%02x\r\n", GetCH());
//	PRINTF("Band_Width:0x%02x\r\n", GetBW());	
//	PRINTF("RF_Rate:0x%02x\r\n", GetRFRate());
	PRINTF("Ver:%02x.\r\n", AwarePro_GetVerion());
	
	#if defined(COO)
		PRINTF("COO_TYPE\r\n");		
	
		Radio.Rx( RX_TIMEOUT_VALUE );	 //Rx continus
		Radio.Send(NULL, 0);
		
	#elif defined(Router)
		PRINTF("ROUTER_TYPE\r\n");
	
		Radio.Rx( RX_TIMEOUT_VALUE );	 //Rx continus
		Radio.Send(NULL, 0);
	#elif defined(ZED)
		PRINTF("ZED_TYPE\r\n");
		Radio.Rx( RX_TIMEOUT_VALUE );	 //Rx continus
		Radio.Send(NULL, 0);
	
	#endif
	
}

void LoRa_Config(NV_Config_Typedef* config)
{
	// user config RF para
	RF_DataRate_Config((RF_Rate_Enum)config->ucRF_Rate, &config->ucBand_Width, config->ucRF_Pwr);
	RF_CH_Config(config->ucRF_Channel, config->ucBand_Width);  //>720MHZ default 800MHZ
//	Write_App_NV();
}

//void LoRa_Config(CNF_Parameter* config)
//{
//	// user config RF para
//	RF_DataRate_Config((RF_Rate_Enum)config->RF_DataRate, &config->RF_Band_Width, config->RF_PA_Select);
//	RF_CH_Config(config->RF_FRQ_CHN, config->RF_Band_Width);  //>720MHZ default 800MHZ
//}


void Node_Change( Dev_Addr*ExtAddr, u16 uiAddr, u8 ucType, u8 ucOpt)
{
	#if defined(DEBUG)

	if(ucOpt)
	{
		PRINTF("Leave,MAC:");		
		print_8_02x((u8*)ExtAddr);
		PRINTF("\r\nAddr:0x%04x,Type:%d\r\n", uiAddr, ucType);	
	}
	else
	{
		PRINTF("Join,MAC:");		
		print_8_02x((u8*)ExtAddr);
		PRINTF("\r\nAddr:0x%04x,Type:%d\r\n", uiAddr, ucType);
	}
	
	
	#endif
}

u8 MINI_Pro_Resp(u8* pucData, u32 ulSysTime)
{
	Dev_Pro_Typedef* DevPro     = (Dev_Pro_Typedef*)pucData;
	MinPro_Typedef* DevFunc     = (MinPro_Typedef*)(DevPro->ucPayLoad);
	Dev_Pro_Typedef* RespPro    = (Dev_Pro_Typedef*)LoRaTxBuffer;
	MinPro_Typedef* Resp        = (MinPro_Typedef*)(RespPro->ucPayLoad);

//	u8 RespFlg  = 1;
	u8 NvFlg = 0;
	u8 ucLen = 0;

	if(DevFunc->ucIndex == BAT_R_INDEX)
	{
	
		Resp->ucIndex   = BAT_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = Pwr.ucBat_level;

		ucLen = 3;
		
	}
	else if(DevFunc->ucIndex == HB_GAP_W_INDEX)
	{
		RFConfig.uiHB_Gap = ((u16)DevFunc->ucData[0]<<8)+((u16)DevFunc->ucData[1]);

		if((RFConfig.uiHB_Gap < HB_MIN_TIME)&&(RFConfig.uiHB_Gap != 0))
		{

			RFConfig.uiHB_Gap = HB_MIN_TIME;
		}
		
		Resp->ucIndex   = HB_GAP_WS_INDEX;
		Resp->ucData[0] = 0;

		NvFlg = TRUE;
		ucLen = 2;
	}
	else if(DevFunc->ucIndex == HB_GAP_R_INDEX)
	{
		Resp->ucIndex   = HB_GAP_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.uiHB_Gap >> 8;	
		Resp->ucData[2] = RFConfig.uiHB_Gap;

		ucLen = 4;
	}
	else if(DevFunc->ucIndex == LC_ID_R_INDEX)
	{
		Resp->ucIndex   = LC_ID_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = Loc.uiID >> 8;	
		Resp->ucData[2] = Loc.uiID;

		ucLen = 4;
	}
	else if(DevFunc->ucIndex == LC_HIS_R_INDEX)
	{
		Resp->ucIndex   = LC_HIS_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = Loc.uiHis_ID >> 8;	
		Resp->ucData[2] = Loc.uiHis_ID;

		ucLen = 4;
	}
	else if(DevFunc->ucIndex == SN_W_INDEX)
	{
		//////// babySN: 80 01 01 37 11 22 33 44  ///

		osal_memcpy((u8*)&RFConfig.aucMatch_SN1, DevFunc->ucData, sizeof(RFConfig.aucMatch_SN1));
		
//		RFConfig.ulMatch_ID1 = ((u32)DevFunc->ucData[0] << 24)
//							    +((u32)DevFunc->ucData[1] << 16)
//							    +((u32)DevFunc->ucData[2] << 8)
//							    +((u32)DevFunc->ucData[3]);
		
		Resp->ucIndex   = SN_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;
		
		NvFlg = 1;
		
	}
	else if(DevFunc->ucIndex == SN2_W_INDEX)
	{
		
//		RFConfig.ulMatch_ID2 = ((u32)DevFunc->ucData[0] << 24)
//							    +((u32)DevFunc->ucData[1] << 16)
//							    +((u32)DevFunc->ucData[2] << 8)
//							    +((u32)DevFunc->ucData[3]);

		osal_memcpy((u8*)&RFConfig.aucMatch_SN2, DevFunc->ucData, sizeof(RFConfig.aucMatch_SN2));

		Resp->ucIndex   = SN2_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;
		
		NvFlg = 1;
		
	}
	else if(DevFunc->ucIndex == SN3_W_INDEX)
	{
//		RFConfig.ulMatch_ID3 = ((u32)DevFunc->ucData[0] << 24)
//							    +((u32)DevFunc->ucData[1] << 16)
//							    +((u32)DevFunc->ucData[2] << 8)
//							    +((u32)DevFunc->ucData[3]);
		
		osal_memcpy((u8*)&RFConfig.aucMatch_SN3, DevFunc->ucData, sizeof(RFConfig.aucMatch_SN3));
		Resp->ucIndex   = SN3_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;
		
		NvFlg = 1;
		
	}
	else if(DevFunc->ucIndex == SN_R_INDEX)
	{
		
		Resp->ucIndex   = SN_RS_INDEX;
		Resp->ucData[0] = 0;
		
		osal_memcpy((u8*)&Resp->ucData[1], (u8*)&RFConfig.aucMatch_SN1, sizeof(RFConfig.aucMatch_SN1));
//		Resp->ucData[1] = RFConfig.ulMatch_ID1 >> 24;		
//		Resp->ucData[2] = RFConfig.ulMatch_ID1 >> 16;
//		Resp->ucData[3] = RFConfig.ulMatch_ID1 >> 8;
//		Resp->ucData[4] = RFConfig.ulMatch_ID1;

		ucLen = 10;
		
	}
	else if(DevFunc->ucIndex == SN2_R_INDEX)
	{
		
		Resp->ucIndex   = SN2_RS_INDEX;
		Resp->ucData[0] = 0;
		
		osal_memcpy((u8*)&Resp->ucData[1], (u8*)&RFConfig.aucMatch_SN2, sizeof(RFConfig.aucMatch_SN2));

//		Resp->ucData[1] = RFConfig.ulMatch_ID2 >> 24;		
//		Resp->ucData[2] = RFConfig.ulMatch_ID2 >> 16;
//		Resp->ucData[3] = RFConfig.ulMatch_ID2 >> 8;
//		Resp->ucData[4] = RFConfig.ulMatch_ID2;

		ucLen = 10;
		
	}
	else if(DevFunc->ucIndex == SN3_R_INDEX)
	{
		
		Resp->ucIndex   = SN3_RS_INDEX;
		Resp->ucData[0] = 0;
	
		osal_memcpy((u8*)&Resp->ucData[1], (u8*)&RFConfig.aucMatch_SN3, sizeof(RFConfig.aucMatch_SN3));
//		Resp->ucData[1] = RFConfig.ulMatch_ID3 >> 24;		
//		Resp->ucData[2] = RFConfig.ulMatch_ID3 >> 16;
//		Resp->ucData[3] = RFConfig.ulMatch_ID3 >> 8;
//		Resp->ucData[4] = RFConfig.ulMatch_ID3;

		ucLen = 10;
		
	}
	else if(DevFunc->ucIndex == MATCH1_R_INDEX)
	{
		
		Resp->ucIndex   = MATCH1_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucMatch1_Status;		

		ucLen = 3;
	}
	else if(DevFunc->ucIndex == MATCH1_W_INDEX)  //add by dozen 20180207 unbinding
	{
		
		Resp->ucIndex   = MATCH1_WS_INDEX;
		Resp->ucData[0] = 0;
		
		RFConfig.ucMatch1_Status = 0;
		osal_memset((u8*)&RFConfig.aucMatch_SN1, 0, sizeof(RFConfig.aucMatch_SN1));

		#if defined(NV)
		Write_App_NV();
		#endif

		ucLen = 2;
	}
	else if(DevFunc->ucIndex == MATCH2_R_INDEX)
	{
		
		Resp->ucIndex   = MATCH2_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucMatch2_Status;		

		ucLen = 3;
	}
	else if(DevFunc->ucIndex == MATCH2_W_INDEX)  //add by dozen 20180207 unbinding
	{
		
		Resp->ucIndex   = MATCH2_WS_INDEX;
		Resp->ucData[0] = 0;
		
		RFConfig.ucMatch2_Status = 0;
		osal_memset((u8*)&RFConfig.aucMatch_SN2, 0, sizeof(RFConfig.aucMatch_SN2));

		#if defined(NV)
		Write_App_NV();
		#endif

		ucLen = 2;
	}
	else if(DevFunc->ucIndex == MATCH3_R_INDEX)
	{
		
		Resp->ucIndex   = MATCH3_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucMatch3_Status;		

		ucLen = 3;
	}
	else if(DevFunc->ucIndex == MATCH3_W_INDEX)  //add by dozen 20180207 unbinding
	{
		
		Resp->ucIndex   = MATCH3_WS_INDEX;
		Resp->ucData[0] = 0;
		
		RFConfig.ucMatch3_Status = 0;
		osal_memset((u8*)&RFConfig.aucMatch_SN3, 0, sizeof(RFConfig.aucMatch_SN3));

		#if defined(NV)
		Write_App_NV();
		#endif

		ucLen = 2;
	}
	else if(DevFunc->ucIndex == CUT_R_INDEX)
	{
		
		Resp->ucIndex   = CUT_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = SysStatus.ucBand_Status;	

		ucLen = 3;
		
	}
	else if(DevFunc->ucIndex == STATUS_W_INDEX)
	{
		
		Resp->ucIndex   = STATUS_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;

		if(DevFunc->ucData[0] == SLEEP)
		{
//			SysStatus.ucActived = FALSE;

//			LPMoveCount_Stop();          //move disable
//			Touch_GPIO_DeInit();            //touch disable
			ucLen = 0;


			// band not connect
			if((SysStatus.ucBand_Status & CONNECT_STATUS) != CONNECT_STATUS)
			{
				SysStatus.ucWork_Status = SLEEP;
			}
			
			
		}
		else if(DevFunc->ucData[0] == WORK)
		{
			ucLen = 0;
			SysStatus.ucWork_Status = WORK;
		}
		// seek sos ///
		else if(DevFunc->ucData[0] == SEEK)
		{
			ucLen = 0;
//			LED_Show(Blue_Led, Flash, 1, TIME_S(2));	
//			LED_Show(Green_Led, Flash, 1, TIME_S(2));
//			SPK_Show(Flash, 1, SPK_ON_3S);
			
			SysStatus.ucWork_Status = SEEK;
			Event_Flg = EVN_EXIT_SEEK;
		}
		else if(DevFunc->ucData[0] == TEST)
		{
			ucLen = 0;
			SysStatus.ucWork_Status = TEST;
		}		

//		if(RFConfig.ucReg_Status == Reg_Done)
//		{
//			if(SysStatus.ucPreStatus != SysStatus.ucWork_Status)
//			{
//				SysStatus.ucPreStatus = SysStatus.ucWork_Status;
//				WorkMode_Report(ulSysTime);
//			}
//			
//		}
//		else if(RFConfig.ucReg_Status == Reg_Failed)
//		{
//			RFConfig.ucReg_Status = Reg_None;
//		}
//		else
//			{
//			}
	}
	else if(DevFunc->ucIndex == PATI_AREA_R_INDEX)
	{
	
		Resp->ucIndex   = PATI_AREA_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucPati_Area;

		ucLen = 3;
	}
	else if(DevFunc->ucIndex == PATI_AREA_W_INDEX)
	{

		Resp->ucIndex   = PATI_AREA_WS_INDEX;
		Resp->ucData[0] = 0;

		RFConfig.ucPati_Area = DevFunc->ucData[0];

		ucLen = 2;
		NvFlg = 1;

	}
	else if(DevFunc->ucIndex == PATI_BED_R_INDEX)
	{
	
		Resp->ucIndex   = PATI_BED_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucPati_Bed;

		ucLen = 3;
	}
	else if(DevFunc->ucIndex == PATI_BED_W_INDEX)
	{

		Resp->ucIndex   = PATI_BED_WS_INDEX;
		Resp->ucData[0] = 0;

		RFConfig.ucPati_Bed = DevFunc->ucData[0];

		ucLen = 2;
		NvFlg = 1;

	}
	else if(DevFunc->ucIndex == RFCH_W_INDEX)
	{
		if(RFConfig.ucRF_Channel != DevFunc->ucData[0])
		{
			RFConfig.ucRF_Channel = DevFunc->ucData[0];
			Event_Flg = EVN_SET_CH;
		}
		
//		RF_CH_Config(RFConfig.ucRF_Channel, GetBW());   
		
		Resp->ucIndex   = RFCH_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;
		NvFlg = 1;
	}
	else if(DevFunc->ucIndex == RFCH_R_INDEX)
	{

		Resp->ucIndex   = RFCH_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucRF_Channel;

		ucLen = 3;

	}
	else if(DevFunc->ucIndex == RFRATE_W_INDEX)
	{
		if(RFConfig.ucRF_Rate != DevFunc->ucData[0])
		{
			RFConfig.ucRF_Rate = DevFunc->ucData[0];
			Event_Flg = EVN_SET_TXRATE;
		}
		
//		RF_DataRate_Config((RF_Rate_Enum)RFConfig.ucRF_Rate, &RFConfig.ucBand_Width, GetRFPwr());
		
		Resp->ucIndex   = RFRATE_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;

		NvFlg = 1;
	}
	else if(DevFunc->ucIndex == RFRATE_R_INDEX)
	{

		Resp->ucIndex   = RFRATE_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucRF_Rate;

		ucLen = 3;

	}
	else if(DevFunc->ucIndex == RFPWR_W_INDEX)
	{
		if(RFConfig.ucRF_Pwr != DevFunc->ucData[0])
		{
			RFConfig.ucRF_Pwr = DevFunc->ucData[0];	
			Event_Flg = EVN_SET_TXPWR;
		}
			
//		RF_DataRate_Config((RF_Rate_Enum)GetRFRate(), &RFConfig.ucBand_Width, RFConfig.ucRF_Pwr);
		
		Resp->ucIndex   = RFPWR_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;
		NvFlg = 1;
	}
	else if(DevFunc->ucIndex == RFPWR_R_INDEX)
	{

		Resp->ucIndex   = RFPWR_RS_INDEX;
		Resp->ucData[0] = 0;
		Resp->ucData[1] = RFConfig.ucRF_Pwr;

		ucLen = 3;

	}
	else if(DevFunc->ucIndex == RST_W_INDEX)
	{

		Resp->ucIndex   = RST_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;

		Event_Flg  = EVN_RST;
	}
	else if(DevFunc->ucIndex == DEF_W_INDEX)
	{

		Resp->ucIndex   = DEF_WS_INDEX;
		Resp->ucData[0] = 0;

		ucLen = 2;

		Delete_NV();
		Delete_App_NV();

		Event_Flg  = EVN_RST;
	}
	else if(DevFunc->ucIndex == GPS_ONOFF_R_INDEX)
	{

		Resp->ucIndex   = GPS_ONOFF_RS_INDEX;		
		Resp->ucData[0] = 0;
		Resp->ucData[1] = GpsStatus.Run_Status;

		ucLen = 3;

	}
	else if(DevFunc->ucIndex == GPS_ONOFF_W_INDEX)
	{

		Resp->ucIndex   = GPS_ONOFF_WS_INDEX;		
		Resp->ucData[0] = 0;

		ucLen = 2;

		if(SysStatus.ucWork_Status != WORK)
		{
			
			Resp->ucData[0] = 1;
		}
		else
		{

			if(DevFunc->ucData[0] != 0)
			{
				if(GpsStatus.Run_Status != TRUE)
				{
					GpsStatus.Run_Status = TRUE;
					GpsStatus.ucStart_Mode = GPS_COLD_RESTART;
					GpsStatus.ucIdle = TRUE;
					HAL_UART_MspInit(&huart2);					
					GPS_VCC_ON();
					PRINTF("GPS Cold on!\r\n");
				}
			}
			else
			{
				
				if(GpsStatus.Run_Status != FALSE)
				{
					GpsStatus.Run_Status = FALSE;				
					GpsStatus.ONOFF_Gap = 0;
					GpsStatus.ucIdle = FALSE;
					GPS_VCC_OFF();
					
					PRINTF("GPS off!\r\n");
				}
				
			}
		}
	

	}
	else if(DevFunc->ucIndex == GPS_GAP_R_INDEX)
	{
	
		Resp->ucIndex	= GPS_GAP_RS_INDEX;		
		Resp->ucData[0] = 0;

		if(GpsStatus.ONOFF_Gap == 0)
		{
			Resp->ucData[1] = GPS_ONOFF_GAP_S;
		}
		else
		{
			Resp->ucData[1] = GpsStatus.ONOFF_Gap;
		}
		
		ucLen = 3;


	}
	else if(DevFunc->ucIndex == GPS_GAP_W_INDEX)
	{
		Resp->ucIndex   = GPS_GAP_WS_INDEX;		
		Resp->ucData[0] = 0;

		ucLen = 2;

		if(SysStatus.ucWork_Status != WORK)
		{
			
			Resp->ucData[0] = 1;
		}
		else
		{

			if(DevFunc->ucData[0] != 0)
			{
				if(GpsStatus.Run_Status != TRUE)
				{
					GpsStatus.Run_Status = TRUE;
					GpsStatus.ONOFF_Gap = DevFunc->ucData[0];
					GpsStatus.ucStart_Mode = GPS_COLD_RESTART;
					GpsStatus.ucIdle = TRUE;
					HAL_UART_MspInit(&huart2);					
					GPS_VCC_ON();
					
					PRINTF("GPS Cold on, Gap:%d!\r\n", GpsStatus.ONOFF_Gap);
				}
			}
			else
			{
				
				if(GpsStatus.Run_Status != FALSE)
				{
					GpsStatus.Run_Status = FALSE;
					GpsStatus.ONOFF_Gap = 0;
					GpsStatus.ucIdle = FALSE;
					GPS_VCC_OFF();
					
					PRINTF("GPS off!\r\n");
				}
				
			}
		
		}

	}
	else
	{
		
	}

	if(ucLen > 0)
	{
		DevPro_Send((u8*)Resp, ucLen, DevPro->ucSer_Seq, 0, ulSysTime);
	}

	if(NvFlg)
	{
		NvFlg = 0;
		Write_App_NV();
	}
	
	return 0;
}

u8 Tag_Bind_Resp(u8 status, u32 ulSysTime)
{

	if(status)
	{
		LoRaTxBuffer[0] = TAG_Type;
		osal_revmemcpy(&LoRaTxBuffer[1], (const u8*)&SN, 4);
	}
	else
	{
		LoRaTxBuffer[0] = TAG_Type;
		LoRaTxBuffer[1] = 0;
		LoRaTxBuffer[2] = 0;
		LoRaTxBuffer[3] = 0;
		LoRaTxBuffer[4] = 0;
	}

	AwarePro_RF_Tx_Process(LoRaTxBuffer, 5, BroadAddr, LORA_ACK_RESP, ulSysTime);

	return 0;
}

u8 Match_Tag_ID(u32 MatchID)
{
	u8 Nv_flg = 0;
	u32 TempID1 = 0;	
	u32 TempID2 = 0;
	u32 TempID3 = 0;
		
	osal_revmemcpy((u8*)&TempID1, (u8*)&RFConfig.aucMatch_SN1.aucAddr[4], sizeof(TempID1));
	osal_revmemcpy((u8*)&TempID2, (u8*)&RFConfig.aucMatch_SN2.aucAddr[4], sizeof(TempID2));
	osal_revmemcpy((u8*)&TempID3, (u8*)&RFConfig.aucMatch_SN3.aucAddr[4], sizeof(TempID3));

	if(MatchID == TempID1)
	{
		if(RFConfig.ucMatch1_Status != TRUE)
		{
			RFConfig.ucMatch1_Status = TRUE;

			Nv_flg = TRUE;
		}
		else
		{
			return 1;
		}
		
	}
	else if(MatchID == TempID2)
	{
		if(RFConfig.ucMatch2_Status != TRUE)
		{
			RFConfig.ucMatch2_Status = TRUE;

			Nv_flg = TRUE;
		}
		else
		{
			return 1;
		}
	}
	else if(MatchID == TempID3)
	{
		if(RFConfig.ucMatch3_Status != TRUE)
		{
			RFConfig.ucMatch3_Status = TRUE;

			Nv_flg = TRUE;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 0;
	}
	
	if(Nv_flg)
	{
		#if defined(NV)
			Write_App_NV();
		#endif

		return 1;
	}

	return 0;

}

void Process_Error(u8 ucError)
{	
	if((ucError == Rx_error)
		||(ucError == Tx_error)
		||(ucError == NO_Rx))
	{
		
		Event_Flg  = EVN_RST;    //reset
	}

}

u8 Process_RF_Tx(u8* pucData, u8 ucLen)
{
	
	if(RF_TX_RUNNING != Radio.GetStatus())	  //Tx_Running
	{		
		Radio.Send(pucData, ucLen);	  
		
		return 0;
	}
	else
	{
		return 1;
	}

	
}

u8 Process_App_RF_RT_Rec(u8* pucData, u8 ucLen, Dev_Addr* pSrcAddr, u32 ulSysTime)
{
	#if defined(DEBUG)
	u8 i;

	print_8_02x((u8*)pSrcAddr);
	PRINTF("Data:");

	for(i = 0; i < ucLen; i++)
	{
		PRINTF("%02x ", pucData[i]);
	}
	PRINTF("\r\n");
	
	#endif

	return 0;
}


u8 Process_App_RF_Rec(u8* pucData, u8 ucLen, u16 uiSrcAddr, u32 ulSysTime)
{
	//user can rec app data and process.
//	Uart_Send(pucData, ucLen);
	
	#if defined(DEBUG)
	u8 i;
	
	PRINTF("App Rec len:%d, SrcAddr:0x%04x\r\n", ucLen, uiSrcAddr);
	PRINTF("Data:");

	for(i = 0; i < ucLen; i++)
	{
		PRINTF("%02x ", pucData[i]);
	}
	PRINTF("\r\n");
	
	#endif

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)pucData;

	
	if(DevPro->ucType == STA_PRO_TYPE)
	{		
		
		if(DevPro->ucLen != (ucLen - 1))
		{
			return 1;
		}

		StaPro_Typedef* DevFunc = (StaPro_Typedef*)(DevPro->ucPayLoad);

		if((DevFunc->uiCID == BIG_ENDIAN_2(CORE_CID))
			&&(DevFunc->ucCMD == REGIST_CMD)
			&&(DevFunc->ucID == S_RESP))
		{
			if(DevFunc->ucData[0] == 0x00)
			{
				if(RFConfig.ucReg_Status != Reg_Done)
				{
				
					RFConfig.ucReg_Status = Reg_Done;
					Write_App_NV();
				
//					LED_Show(Green_Led, Flash, 1, FLASH_3S);	//joining
				}
				

				PRINTF("Register Success!\r\n");
			}
			else
			{
				if(RFConfig.ucReg_Status != Reg_Failed)
				{
					
					RFConfig.ucReg_Status = Reg_Failed;
					
//					LED_Show(Red_Led, Flash, 1, FLASH_3S);
//					SPK_Show(Flash, 1,SPK_ON_2S);
//					Write_App_NV();
				}
								
				PRINTF("Register Failed!\r\n");
			}
				
		}
	}
	else if(DevPro->ucType == MIN_PRO_TYPE)
	{
		
		if(DevPro->ucLen != (ucLen - 1))
		{
			return 1;
		}
	
		MINI_Pro_Resp(pucData, ulSysTime);

	}
	else
	{
		
		if((SysStatus.ucBind_Status == SN_MATCHING))
		{
			SysStatus.ucBind_Status = SN_MATCH_END;
			
			if(pucData[0] == MATCH_TAG_Type)
			{
				u32 TempID = 0;

				osal_revmemcpy((u8*)&TempID, (u8*)&pucData[1], sizeof(TempID));

//				if(TempID == RFConfig.ulMatch_ID1)
				if(Match_Tag_ID(TempID))
				{
					PRINTF("Match Success!\r\n");

					LED_Show(Blue_Led, Flash, 2, FLASH_50MS);
					LED_Show(Green_Led, Flash, 2, FLASH_50MS);

				}
				else
				{
					
					PRINTF("Match ID diff!\r\n");
				}
			}
			else
			{
				PRINTF("Match Failed!\r\n");
				
			}
			
		}
		else
		{
			PRINTF("Bind Status:%d!\r\n", SysStatus.ucBind_Status);
		}
		
	}
	
	return 0;
}


u8 Process_App_Ack(u16 uiSrcAddr, u8 bStatus, u32 ulSysTime)
{

	PRINTF("Addr:0x%04x, Status:%d\t\n", uiSrcAddr, bStatus);
	
	if((AwarePro_NV_JoinFlg() == NET_FLG)
		&&(RFConfig.ucReg_Status == Reg_Done))   //join and register
	{	
		if(SysStatus.ucWork_Status != TEST)
		{
			AwarePro_ZED_Poll_Once();
		}
		
	}
	
	return 0;
}

u8 Process_Join_Status(Join_Status_Enum bStatus)
{

	SysStatus.ucJoin_Status = (u8)bStatus;
	PRINTF("Join Status:%d\t\n",bStatus);
	
	return 0;
}

u8 Process_Cmd_Status(u16 uiSrcAddr, CMD_Enum Cmd, u8* CmdValue, u8 ucLen, u8 ucOpt, u8 bStatus)
{
	///////// status 0:success,1:failed//// 
	
//	PRINTF("Addr:0x%04x, CMD:0x%02x, Opt:%d, Status:%d\r\n", uiSrcAddr, (u8)Cmd, ucOpt, bStatus);
	PRINTF("Addr:0x%04x, CMD:0x%02x, Opt:%d, CmdValue:%d, Status:%d\r\n", uiSrcAddr, (u8)Cmd, ucOpt, *CmdValue, bStatus);
	return 0;
}

/// return 
// 1: address me; 2:PID error;3:dup frame; 4:type error;5:addr error;6:status error

u8 Process_LoRa_Data(u32 ulSysTime)
{

	if(LoRaRxBuffer[0].ucLen != 0)
	{
		
		if(LoRaRxBuffer[1].ucLen != 0)
		{
			PRINTF("buf1 full\r\n");
		}
		else
		{	
			memcpy((u8*)&LoRaRxBuffer[1], (u8*)&LoRaRxBuffer[0], sizeof(RF_Rx_Typedef));
			LoRaRxBuffer[0].ucLen = 0;
		}
		
	}

	if(LoRaRxBuffer[1].ucLen == 0)
	{
		return 0;
	}

	AwarePro_RF_Rec_Process(LoRaRxBuffer[1].aucPayload, LoRaRxBuffer[1].ucLen, LoRaRxBuffer[1].siRssi, LoRaRxBuffer[1].scSnr, ulSysTime);

	LoRaRxBuffer[1].ucLen = 0;

	return 0;
}

void Process_OHR(u32 ulSysTime)
{	
/*	static u32 ulPretick = 0;

	if(OHR_status != OHR_STATUS_DISABLED)
	{
		
		if((u32)(ulSysTime - ulPretick) > TIME_S(1))  //
		{
			if(OHR_status == OHR_STATUS_INITIALIZING)
			{
				
				ulPretick = ulSysTime;
				OHR_event_handler();
			
			}
			else if(OHR_status == OHR_STATUS_WAITFORRESPONSE)
			{		
				PRINTF("OHR Resp TimeOut!\r\n");
				
				StopOHR();
				
//				HAL_UART_MspDeInit(&huart2); 

//				OHR_status = OHR_STATUS_INITIALIZING;
				ucHeart_Status = 0;
			}
			else if(OHR_status == OHR_EVENT_START_FAIL)
			{
	
				/// dvail port low level

				
			}
			else
				{
				}
		}
		
	}
	else
	{
		ulPretick = ulSysTime;
	}

*/

	///UART///
//	OHR_event_handler();

	//////////// iic //////////
//	static u8 ucPreHr = 0;

	OHR_event_handler();

//	if(OHRdata.hr != ucPreHr)
//	{
//		ucPreHr = OHRdata.hr; 
//		PRINTF("HR:%d, SNR:%d, On:%d, Current:%d\r\n", 
//			  OHRdata.hr, OHRdata.quality, OHRdata.onwrist, OHRdata.current);
//	}
	
}
//// binding matetag and babytag
void Process_Bind(u32 ulSysTime)
{
	static u32 ulPretick = 0;
	
	if(SysStatus.ucWork_Status != WORK)
	{
		SysStatus.ucBind_Status = SN_IDLE_STATUS;
		return;
	}

	if(RFConfig.ucReg_Status != Reg_Done)
	{
		SysStatus.ucBind_Status = SN_IDLE_STATUS;
		return;
	}

	if(SysStatus.ucBind_Status == SN_IDLE_STATUS)
	{
		return;
	}

	if(SysStatus.ucBind_Status == SN_TRIG_MATCH)
	{
		
		RF_Config_Typedef LF;

		LF.ucCmd   = MATCH_CMD;
		LF.ucTagID = MATETAG_ID;
		memcpy((u8*)&LF.ulTagSN, SN, sizeof(LF.ulTagSN));

		
		LF_SendConfig((u8*)&LF);

		SysStatus.ucBind_Status = SN_REC_MATCH_RSP;

		
	}	
	else if(SysStatus.ucBind_Status == SN_REC_MATCH_RSP)
	{
		if(Radio.GetStatus() == RF_IDLE)
		{
			RF_CH_Config(RF_BIND_CH, GetBW());         //470M
			Radio.Rx( RX_TIMEOUT_VALUE );
			
			SysStatus.ucBind_Status = SN_MATCHING;

			ulPretick = ulSysTime;
		}
		
	}
	else if(SysStatus.ucBind_Status == SN_MATCHING)
	{
		if((u32)(ulSysTime - ulPretick) > TIME_S(6))
		{
			SysStatus.ucBind_Status = SN_MATCH_END;
			PRINTF("Match End!\r\n");
		}
	}
	else if(SysStatus.ucBind_Status == SN_MATCH_END)
	{	
		
		AwarePro_Cancel_Poll();

		if(Radio.GetStatus() == RF_IDLE)
		{
//			SetCH(curCH);
			RF_CH_Config(GetCH(), GetBW());         //470M
			Radio.Rx( RX_TIMEOUT_VALUE );

			SysStatus.ucBind_Status = SN_IDLE_STATUS;

			PWM_GPIO_Init();
			
			LPMoveCount_Start();
		}

		
	}
	else
	{
		SysStatus.ucBind_Status = SN_IDLE_STATUS;
	}
	
}

void Process_LF_TX(u32 ulSysTime)
{
	u8 sendbuf[10] = {0};
	
	static u32 ulPertick = 0;
	
	if((u32)(ulSysTime - ulPertick) > TIME_S(1))  //HB_TIME
	{
		
		ulPertick = ulSysTime;

//		sendbuf[0] = 0x69;
		sendbuf[0] = 0x91;
		sendbuf[1] = 0x35;
		sendbuf[2] = 0x55;//low byte of ID;
		sendbuf[3] = 0x66;//low byte of ID;
		sendbuf[4] = 0x77;//low byte of ID;
		sendbuf[5] = 0x88;//low byte of ID; 

		LF_SendConfig(sendbuf);

		PRINTF("LF Send!\r\n");
//		SendLF(sendbuf);
	}
	
}

void Process_LF(u32 ulSysTime)
{
	u8 i;
//	u8 Staflg = 0;
//	u16 ID;
	static u32 ulPretick = 0;
//	static u8 ucStartLF  = 0;

//	if(SysStatus.ucActived == FALSE)
//	if(SysStatus.ucWork_Status != WORK)
//	{
//		LFReceived = 0;
//		return;
//	}

//	if(RFConfig.ucReg_Status != Reg_Done)        //LF config 
//	{
//		return;
//	}
	
	if(SysStatus.ucMove_Status == STATIC_STATUS)
	{
		LF.Protocal = 0;
		ulPretick = ulSysTime;

		return;
	}
	
	if((LF.Protocal == LF_SINGLE)||(LF.Protocal == LF_MULTIPLE))
	{

		if(LF.Protocal == LF_SINGLE)
		{
			
			PRINTF("Single ADDR:%04x\r\n", LF.Data.SingleAddr);
		}
		else
		{
			PRINTF("Multi ADDR:%04x\r\n", LF.Data.MultipleAddr.Addr);
		}

//		for(i = 0; i < 8; i++)
//		{
//			PRINTF("%02x ", LF_RecByte[i]);
//		}
//		PRINTF("\r\n");

		ulPretick = ulSysTime;

		if(LF.Protocal == LF_SINGLE)
		{

			if(LF.Data.SingleAddr != Loc.uiID)
			{				
				Loc.uiID = LF.Data.SingleAddr;
				Loc.uiHis_ID = LF.Data.SingleAddr;
				Loc.ucType   = 0;
				Loc.ucStatus = LOC_ENTER;
				Loc.ulPretick = ulSysTime;

	//			if(LF_RecByte[4] > LF_RecByte[5])
				if(LF.RSSIx > LF.RSSIy)
				{
					Loc.scRSSI = LF.RSSIx;
				}
				else
				{
					Loc.scRSSI = LF.RSSIy;
				}

				SysStatus.ucMove_Status = LOC_STATUS;

				if((AwarePro_NV_JoinFlg() == NET_FLG)
					&&(RFConfig.ucReg_Status == Reg_Done))
				{
					LF_Report(LOC_ENTER, ulSysTime);

					LED_Show(Green_Led, Flash, 2, FLASH_50MS);
				}
				else
				{
					AwarePro_ReJoin();
				}
			
///				PRINTF("11bit Location status\r\n");
				
			}
			else
			{			
				if((u32)(ulSysTime - Loc.ulPretick) > TIME_S(10))	
				{

					LF_AS3932DeInit();
					
					PRINTF("same ID rec > 10s\r\n");
					
					SysStatus.ucMove_Status = STATIC_STATUS;

				}
			}
		}
		else
		{

			if(LF.Data.MultipleAddr.Addr != Loc.uiID)
			{				
				Loc.uiID = LF.Data.MultipleAddr.Addr;
				Loc.uiHis_ID = LF.Data.MultipleAddr.Addr;
				Loc.ucStatus = LOC_ENTER;
				Loc.ulPretick = ulSysTime;

	//			if(LF_RecByte[4] > LF_RecByte[5])
				if(LF.RSSIx > LF.RSSIy)
				{
					Loc.scRSSI = LF.RSSIx;
				}
				else
				{
					Loc.scRSSI = LF.RSSIy;
				}

				if(LF.Data.MultipleAddr.Type == 0)   //normal
				{
					
					Loc.ucType	 = 0;
					LF_Report(LOC_ENTER, ulSysTime);
				}
				else                                 //alarm in_out
				{
					
					Loc.ucType	 = 1;
					LF_Alarm_Report(LOC_ENTER, ulSysTime);
				}
				
				SysStatus.ucMove_Status = LOC_STATUS;

				if((AwarePro_NV_JoinFlg() == NET_FLG)
					&&(RFConfig.ucReg_Status == Reg_Done))
				{
					
					LED_Show(Green_Led, Flash, 2, FLASH_50MS);
				}
				else
				{
					AwarePro_ReJoin();
				}
			
//				PRINTF("16bit Location status\r\n");
				
			}
			else
			{			
				if((u32)(ulSysTime - Loc.ulPretick) > TIME_S(10))	
				{

					LF_AS3932DeInit();
					
					PRINTF("same ID rec > 10s\r\n");
					
					SysStatus.ucMove_Status = STATIC_STATUS;

				}
			}
		}
	
		LF.Protocal = LF_IDLE;
		return;
	}
	else if(LF.Protocal == LF_CONFIG)            //lf config  protocol unknow
	{	
		PRINTF("Data:");
		for(i = 0; i < 6; i++)
		{
			PRINTF("%02x ", LF.Data.ConfigData[i]);
		}
		PRINTF("\r\n");

		PRINTF("error:%d\r\n", LF.DecodeErrCnt);
		
		LF_Config(ulSysTime);
		LF.Protocal = LF_IDLE;
		
	}
	else if(LF.Protocal == LF_ERROR )
	{
		LF.Protocal = LF_IDLE;

//		return;
	}
	else
	{
		LF.Protocal = LF_IDLE;
	}

	if((Loc.ucStatus == LOC_ENTER)&&((u32)(ulSysTime - ulPretick) > TIME_S(3)))
	{
		ulPretick = ulSysTime;	

		Loc.ucStatus = LOC_EXIT;

		if(Loc.ucType == 1)
		{
			
			LF_Alarm_Report(LOC_EXIT, ulSysTime);   //type 0 or 1 exit
		}
		else
		{
			
			LF_Report(LOC_EXIT, ulSysTime);   //type 0 or 1 exit
		}
		
		Loc.uiID = 0;

		LF_AS3932DeInit();
	
		PRINTF("no rec > 3s\r\n");
		
		SysStatus.ucMove_Status = STATIC_STATUS;

		
		if((AwarePro_NV_JoinFlg() == NET_FLG)
			&&(RFConfig.ucReg_Status == Reg_Done))
		{
			LED_Show(Green_Led, Flash, 3, FLASH_50MS);
		}
		
		
	
	}

}

void Process_Motion(u32 ulSysTime)
{

	static u8 Preflg = 0;     //mov status
	static u32 Pretick = 0;
	static u32 StartTick = 0;

//	if(SysStatus.ucActived == FALSE)
//	if(SysStatus.ucWork_Status != WORK)
//	{
//		return;
//	}

//	if(RFConfig.ucReg_Status != Reg_Done)         //LF config
//	{
//		return;
//	}
	
	if((u32)(ulSysTime - Pretick) > MOTION_SAMPLE_TIME)
	{
		Pretick = ulSysTime;
		
		if(GetLPMoveCount() > RFConfig.uiMove_Threshold)  //MOTION_THRES
		{			
			PRINTF("Move status\r\n");


			if(SysStatus.ucMove_Status != MOVE_STATUS)
			{
				SysStatus.ucMove_Status = MOVE_STATUS;
				
				LF_AS3932Init();
			}

//			Pretick = ulSysTime;
			Preflg = 0;
			
		}
		else
		{

			if(!Preflg)
			{
				Preflg = 1;
				StartTick = ulSysTime;
			}
			
		}
	}


	if(Preflg)
	{
		if((u32)(ulSysTime - StartTick) >= TIME_S(RFConfig.ucStatic_Time))  // = 30s TIME_S(STATIC_TIME)
		{
			StartTick = ulSysTime;
			
			if(SysStatus.ucMove_Status == MOVE_STATUS)
			{
				SysStatus.ucMove_Status = STATIC_STATUS;

				LF_AS3932DeInit();
				
				PRINTF("static status\r\n");
			
//				LED_Show(Blue_Led, Flash, 1, 10);
				
//				SPK_Show(Flash, 1, SPK_FAST);
			}
			
		}
	}
	
	
}

void Process_Bat(u32 ulSysTime)
{
	
	static u32 ulPertick = 0;
	static u8 ucPreStatus = 0;

	///// no charge   CHG high level
	///// charging    CHG low level
	///// charge full CHG high level and 4.15v

	if(CHG_Read() == 0)
	{
		if(Pwr.ucPwr_Type != ADAPTER_5V)
		{
			Pwr.ucPwr_Type = ADAPTER_5V;
//			Pwr.ucBat_Status = BAT_CHGING;
		}
		
	}
	else
	{
		if((Pwr.ucPwr_Type == ADAPTER_5V))
//			&&(Pwr.ucBat_Status == BAT_CHG_FULL))
		{
			Pwr.ucPwr_Type = BAT;
		}
	}
	
	if(Pwr.ucPwr_Type == BAT)
	{
		if(Pwr.ucPwr_Type != ucPreStatus)
		{
			ucPreStatus = Pwr.ucPwr_Type;
	
			PRINTF("adapter-->Bat.\r\n");
			Pwr.ucBat_level  = 0;       //clr bat level
			Pwr.ucBat_Status = 0;

			
			SysStatus.ucWork_Status = SLEEP;
			
//			if((AwarePro_NV_JoinFlg() == NET_FLG)
//				&&(RFConfig.ucReg_Status == Reg_Done))
//			{
//								
//				SysStatus.ucWork_Status = SLEEP;
//					
//			}
//			else if(AwarePro_NV_JoinFlg() != NET_FLG)
//			{
//				AwarePro_ReJoin();
//			}
//			else if(RFConfig.ucReg_Status != Reg_Done)
//			{
//				RFConfig.ucReg_Status = Reg_None;
//			}
//			else
//				{
//				}

				//close blue led 
			LED_Show(Blue_Led, OFF, 0, 0);

		}

		if((u32)(ulSysTime - ulPertick) > TIME_S(60))  //HB_TIME
		{
			u8 level = 0;
			
			ulPertick = ulSysTime;
			level = HW_GetBatteryLevel();

			if(Pwr.ucBat_level == 0)           //adapter -> bat
			{
				Pwr.ucBat_level = level;
			}
			else
			{
				if((level < Pwr.ucBat_level)
					&&(Pwr.ucBat_level - level) <= BAT_OFFSET)
				{
					 Pwr.ucBat_level = level;
				}
			}
			
			
//			PRINTF("BAT level:%d\r\n", Pwr.ucBat_level);
		}
	}
	
	else if(Pwr.ucPwr_Type == ADAPTER_5V)
	{
		if(Pwr.ucPwr_Type != ucPreStatus)
		{
			ucPreStatus = Pwr.ucPwr_Type;		
			ulPertick = ulSysTime;
			
			PRINTF("Bat-->Adapter.\r\n");

			///connect adapter and enter charging mode
			if((AwarePro_NV_JoinFlg() == NET_FLG)
				&&(RFConfig.ucReg_Status == Reg_Done))
			{
				
//				ucPreMode = SysStatus.ucWork_Status;
				SysStatus.ucWork_Status = CHG;
			
			}
			else
			{
				SysStatus.ucWork_Status = CHG;

				LPMoveCount_Stop(); 		   //move func disable
				StopOHR();                     //disable heartrate report
				ucHeart_Status = 0;
					
				if(SysStatus.ucMove_Status != STATIC_STATUS)
				{
					LF_AS3932DeInit();        //lf disable
				}
			}

			
		}

		
		if((u32)(ulSysTime - ulPertick) > TIME_S(1))
		{		
			static u8 cnt = 0;
			
			ulPertick = ulSysTime;

			if(CHG_Read())              //bat full
			{
				if(cnt > 5)
				{
					cnt = 0;
					/// bat full
					if(Pwr.ucBat_Status != BAT_CHG_FULL)
					{
						Pwr.ucBat_Status = BAT_CHG_FULL;
						
						PRINTF("BAT full!\r\n");

						SPK_Show(Flash, 1, SPK_100MS);   //SPK_ON_3S bat low
						LED_Show(Blue_Led, ON, 0, 0);
						/// report info lora

						CHG_Report(ulSysTime);
						
					}
					
				}
				else
				{
					cnt++;
				}
				
				
			}
			else                 //charging
			{
				
				if(Pwr.ucBat_Status != BAT_CHGING)    //bat->adapter
				{
					Pwr.ucBat_Status = BAT_CHGING;
					
					PRINTF("BAT charging!\r\n");
					
					/// report info lora
					CHG_Report(ulSysTime);
					
				}
				
			}


		}

		if(Pwr.ucBat_Status == BAT_CHGING)
		{
			
			if(LED_Idle())
			{
				LED_Show(Blue_Led, Flash, 255, FLASH_50MS);
			}
		
		}
		
	}
	else
		{
		}
}

/////////// babyTag pwr manage ////////////
void Process_Pwr(u32 ulSysTime)
{
	static u32 ulPertick = 0;
	static u8 ucPreStatus = 0;
//	static u8 ucPreMode  = 0;

	if(CHG_CHK_Read())      //adapt socket on
	{
		if(Pwr.ucPwr_Type != ADAPTER_5V)
		{
			Pwr.ucPwr_Type = ADAPTER_5V;
		}
		
	}
	else
	{
		if(Pwr.ucPwr_Type != BAT)
		{
			Pwr.ucPwr_Type = BAT;
		}
	}
	
	/// charge or vdd check
	if(Pwr.ucPwr_Type == BAT)
	{	
		if(Pwr.ucPwr_Type != ucPreStatus)
		{
			ucPreStatus = Pwr.ucPwr_Type;
	
			PRINTF("adapter-->Bat.\r\n");
			Pwr.ucBat_level  = 0;       //clr bat level
			Pwr.ucBat_Status = 0;

			if((AwarePro_NV_JoinFlg() == NET_FLG)
				&&(RFConfig.ucReg_Status == Reg_Done))
			{
				
				SysStatus.ucWork_Status = SLEEP;
//				SysStatus.ucWork_Status = ucPreMode;
					
			}
			else if(AwarePro_NV_JoinFlg() != NET_FLG)
			{
				AwarePro_ReJoin();
			}
			else if(RFConfig.ucReg_Status != Reg_Done)
			{
				RFConfig.ucReg_Status = Reg_None;
			}
			else
				{
				}

				//close blue led 
			LED_Show(Blue_Led, OFF, 0, 0);

		}

		if((u32)(ulSysTime - ulPertick) > TIME_S(60))  //HB_TIME
		{
			u8 level = 0;
			
			ulPertick = ulSysTime;
			level = HW_GetBatteryLevel();
//			PRINTF("BAT vdd:%d, Pwr_bat:%d\r\n", level, Pwr.ucBat_level);

			if(Pwr.ucBat_level == 0)           //adapter -> bat
			{
				Pwr.ucBat_level = level;
			}
			else
			{
				if((level < Pwr.ucBat_level)
					&&(Pwr.ucBat_level - level) <= BAT_OFFSET)
				{
					 Pwr.ucBat_level = level;
				}
			}
			
			
//			PRINTF("BAT level:%d\r\n", Pwr.ucBat_level);
		}
		
	}
	else if(Pwr.ucPwr_Type == ADAPTER_5V)
	{
//		static u8 ucPreBatStatus = 0;

		if(Pwr.ucPwr_Type != ucPreStatus)
		{
			ucPreStatus = Pwr.ucPwr_Type;		
			ulPertick = ulSysTime;
			
			PRINTF("Bat-->Adapter.\r\n");

			if((AwarePro_NV_JoinFlg() == NET_FLG)
				&&(RFConfig.ucReg_Status == Reg_Done))
			{
				
//				ucPreMode = SysStatus.ucWork_Status;
				SysStatus.ucWork_Status = CHG;
			
			}
//			else if(AwarePro_NV_JoinFlg() != NET_FLG)
//			{
//				AwarePro_ReJoin();
//			}
//			else if(RFConfig.ucReg_Status != Reg_Done)
//			{
//				RFConfig.ucReg_Status = Reg_None;
//			}
//			else
//				{
//				}

		}

		if((u32)(ulSysTime - ulPertick) > TIME_S(1))
		{		
			static u8 cnt = 0;
//			static u8 Time_s = 0;
			
			ulPertick = ulSysTime;

			if(CHG_Read())
			{
				if(cnt > 5)
				{
					cnt = 0;
					/// bat full
					if(Pwr.ucBat_Status != BAT_CHG_FULL)
					{
						Pwr.ucBat_Status = BAT_CHG_FULL;
						
						PRINTF("BAT full!\r\n");

						SPK_Show(Flash, 1, SPK_100MS);
						LED_Show(Blue_Led, ON, 0, 0);
						/// report info lora

						CHG_Report(ulSysTime);
						
					}
					
				}
				else
				{
					cnt++;
				}
				
				
			}
			else                 //charging
			{
				
				if(Pwr.ucBat_Status != BAT_CHGING)    //bat->adapter
				{
					Pwr.ucBat_Status = BAT_CHGING;
					
					PRINTF("BAT charging!\r\n");
					
					/// report info lora
					CHG_Report(ulSysTime);
					
				}
				
			}

//			if(Time_s > CHG_REPORT_GAP)
//			{
//				Time_s = 0;
//				Pwr.ucBat_level = HW_GetBatteryLevel();
//				PRINTF("BAT level:%d\r\n", Pwr.ucBat_level);

//				BAT_Level_Report(ulSysTime);
//			}
//			else
//			{
//				Time_s++;
//			}
			
		}

		if(Pwr.ucBat_Status == BAT_CHGING)
		{
			
			if(LED_Idle())
			{
				LED_Show(Blue_Led, Flash, 255, FLASH_50MS);
			}
		
		}
		

	}
	else
	{
		
	}
	
}

void Process_WorkMode(u32 ulSysTime)
{
/*
	if(AwarePro_NV_JoinFlg() != NET_FLG)
	{
		if(SysStatus.ucWork_Status != SysStatus.ucPreStatus)
		{
//			SysStatus.ucPreStatus = SysStatus.ucWork_Status;
			AwarePro_ReJoin();
		}

		return;
	}
	
	if(RFConfig.ucReg_Status != Reg_Done)
	{
		if(SysStatus.ucWork_Status != SysStatus.ucPreStatus)
		{
//			SysStatus.ucPreStatus = SysStatus.ucWork_Status;

			RFConfig.ucReg_Status = Reg_None;
		}
		
		return;
	}

*/
	if((AwarePro_NV_JoinFlg() != NET_FLG)
		||(RFConfig.ucReg_Status != Reg_Done))
	{

		return;
	
	}
	
	if(SysStatus.ucWork_Status == SLEEP)   //work to sleep  or charge to sleep
	{
		if(SysStatus.ucPreStatus != SLEEP)  //ADC enable
		{
			if(SysStatus.ucPreStatus == CHG)  //Charg to sleep
			{
			
			}
			else
			{
				LPMoveCount_Stop(); 		   //move func disable
				StopOHR();
				ucHeart_Status = 0;

				if(GpsStatus.Run_Status == TRUE)
				{
					GpsStatus.Run_Status = FALSE;
					GPS_VCC_OFF();					
					PRINTF("Gps Sleep off!\r\n");
				}
					
				if(SysStatus.ucMove_Status != STATIC_STATUS)
				{
					LF_AS3932DeInit();        //lf disable
				}
			}
			
			SysStatus.ucPreStatus = SLEEP;
			RFConfig.ucWork_Status = SysStatus.ucWork_Status;		
			Write_App_NV();

			WorkMode_Report(ulSysTime);
		}
		
	}
	else if(SysStatus.ucWork_Status == CHG)
	{
		
		if(SysStatus.ucPreStatus != CHG)  //ADC enable  work to charge
		{
			
			if(SysStatus.ucPreStatus == SLEEP)  //sleep to charge
			{
			
			}
			else
			{
				LPMoveCount_Stop(); 		   //move func disable
				StopOHR();
				ucHeart_Status = 0;
				
				if(GpsStatus.Run_Status == TRUE)
				{
					GpsStatus.Run_Status = FALSE;
					GPS_VCC_OFF();
					PRINTF("Gps chg off!\r\n");
				}
					
				if(SysStatus.ucMove_Status != STATIC_STATUS)
				{
					LF_AS3932DeInit();        //lf disable
				}
			}
			
			SysStatus.ucPreStatus = CHG;
			RFConfig.ucWork_Status = SysStatus.ucWork_Status;
			Write_App_NV();

			WorkMode_Report(ulSysTime);
		}
		
	}
	else if(SysStatus.ucWork_Status == WORK)
	{		
		if(SysStatus.ucPreStatus != WORK)
		{
			if((SysStatus.ucPreStatus == SLEEP)
			||(SysStatus.ucPreStatus == CHG))   //sleep or charge mode to work mode
			{
			
				LPMoveCount_Start();	
//				Event_Flg = EVN_OHR;
//				ucHeart_Status = 1;
				StartOHR(2, 10);                  //heartbeat start
				
//				StartOHR(0, 0);                  //heartbeat start
				PRINTF("Start OHR!\r\n");
//				Touch_GPIO_Init();
						
			}
			
			SysStatus.ucPreStatus = WORK;			
			RFConfig.ucWork_Status = SysStatus.ucWork_Status;
			Write_App_NV();
			
			WorkMode_Report(ulSysTime);
		}
		
	}
	else if(SysStatus.ucWork_Status == SEEK)
	{
		if(SysStatus.ucPreStatus != SEEK)
		{
			SysStatus.ucPreStatus  = SEEK;

			if(GpsStatus.Run_Status == TRUE)
			{
				GpsStatus.Run_Status = FALSE;
				GPS_VCC_OFF();
			}

			LED_Show(Blue_Led, Flash, 1, TIME_S(2));	
			LED_Show(Green_Led, Flash, 1, TIME_S(2));
			SPK_Show(Flash, 1, SPK_100MS);

			WorkMode_Report(ulSysTime);
		}
		
	}
	else if(SysStatus.ucWork_Status == TEST)
	{
		if(SysStatus.ucPreStatus != TEST)
		{
			SysStatus.ucPreStatus  = TEST;

			WorkMode_Report(ulSysTime);
		}
		
	}
	else
	{
		SysStatus.ucWork_Status = SLEEP;
	}
}


//////// Tag Drop Alarm ///////
void Process_Tag_Alarm(u32 ulSysTime)
{
	
	static u32 ulPreTick = 0;
		
	if((OHRdata.hr != 0))
	{

		if( OHRdata.quality > HB_VALID_SNR)     //SNR > 5
		{

			if(HB.ucSum_Cnt == HB_SUM_CNT) 
			{
				
				HB.ucSum_Cnt = 0;
				HB.ucAve_HR = (HB.uiSum_HR >> 5);    //32
				HB.uiSum_HR = 0;
				
				PRINTF("HB Aver:%d\r\n", HB.ucAve_HR);
			}
			else
			{

				HB.uiSum_HR += OHRdata.hr;
				HB.ucSum_Cnt++;
//				ucHB_Aver = (ucHB_Aver >> 1);
			}

			
			HB.ucUnvalid_Cnt = 0;
		}
		
		OHRdata.hr = 0;
		
	}	

	
	if((AwarePro_NV_JoinFlg() != NET_FLG)
		||(RFConfig.ucReg_Status != Reg_Done))
	{

		if((SysStatus.ucWork_Status == WORK)
			&&((u32)(ulSysTime - ulPreTick) > TIME_S(300)))    //5 min =60*5=300
		{

			u8 i = 0;
			
			ulPreTick = ulSysTime;

			if(HB.ucAve_HR == 0)
			{
				return;
			}

			i = Add_HB_Record(HB.ucAve_HR);
			
			if(i != 0xFF)
			{
				PRINTF("HB Record:%d,Num:%d\r\n", HB.ucAve_HR, i + 1);
			}
			else
			{
				PRINTF("HB Record Failed!\r\n");
			}
			
		}
		
		return;
	
	}

	if(HB.ucAlarm_HR == 0)

	{
		if((HB.ucAve_HR > HB_MAX)
		||((HB.ucAve_HR < HB_MIN)&&(HB.ucAve_HR != 0)))
		{
		/// report Tag Alarm 
			Tag_Alarm_Report(ulSysTime, 0x02);
			HB.ucAlarm_HR = 1;
//			HB.ucAve_HR = 0;

			PRINTF("HB Alarm!\r\n");
		}

		if(OHRdata.quality < HB_UNVALID_SNR)
		{
			if(HB.ucUnvalid_Cnt > HB_UNVALID_MAX)
			{
				HB.ucAlarm_HR = 1;
				HB.ucAve_HR = 0;
				HB.ucSum_Cnt = 0;
				HB.uiSum_HR = 0;
				HB.ucUnvalid_Cnt = 0;
				
				Tag_Alarm_Report(ulSysTime, 0x01);
				
				PRINTF("Tag Drop Alarm!\r\n");
			}
			else
			{
				HB.ucUnvalid_Cnt++;
			}
			
		}
	}
	else
	{
		if((HB.ucAve_HR <= HB_MAX)
		&&(HB.ucAve_HR >= HB_MIN))
		{

			if(OHRdata.quality < HB_VALID_SNR)
			{
				return;
			}
			
			Tag_Alarm_Report(ulSysTime, 0x00);			
			HB.ucAlarm_HR = 0;
			PRINTF("Tag Put On!\r\n");
		}
	}
	
	////////// poll HB record and report server ////
	if(Poll_Record_Num() > 0)
	{
		HB_Record_Report(ulSysTime);
		Erase_HB_Record();
		
		PRINTF("HB Record Report!\r\n");
	}
	
/*
	if((u32)(ulSysTime - ulPreTick) > TIME_S(1))
	{
		ulPreTick = ulSysTime;


				/////// tag drop////
		if((SysStatus.ucHeartRate == 0)
			&&(SysStatus.ucHeartRate != ucPreHeartRate))
		{
//			if(ucCnt > 3)
			{
//				ucCnt = 0;
				ucPreHeartRate = SysStatus.ucHeartRate;

				/// report Tag Alarm 
				Tag_Alarm_Report(ulSysTime, 0x01);

				PRINTF("Tag Drop Alarm!\r\n");
				
			}
//			else
			{
//				ucCnt++;
			}
			

			
		}

		///// tag put on//
		else if((SysStatus.ucHeartRate != 0)
				&&(ucPreHeartRate == 0))	
		{
			
			ucPreHeartRate = SysStatus.ucHeartRate;

			//////// reprot  //////////
			
			Tag_Alarm_Report(ulSysTime, 0x00);
			
			PRINTF("Tag Put On!\r\n");
		}
		else
		{
//			ucCnt = 0;
		}
	
	}

	*/

}


void Process_GPS(u32 ulSysTime)
{	
	static u32 ulPretick  = 0;
	u8 Gap = 0;

	if(GpsStatus.Run_Status == FALSE)
	{
		ulPretick = ulSysTime;
		return;
	}

	if(GpsStatus.ucIdle == FALSE)
	{
	
		Gap = (GpsStatus.ONOFF_Gap == 0 ? GPS_ONOFF_GAP_S: GpsStatus.ONOFF_Gap);
	//	Gap = (GpsStatus.ONOFF_Gap > GPS_ONOFF_GAP_S ? GpsStatus.ONOFF_Gap : GPS_ONOFF_GAP_S);
		
		if((u32)(ulSysTime - ulPretick) > TIME_S(Gap))
		{
			ulPretick = ulSysTime;
			HAL_UART_MspInit(&huart1);
	
			GPS_VCC_ON();
			GpsStatus.ucStart_Mode = GPS_HOT_RESTART;

			PRINTF("GPS HOT ON!\r\n");
			
			GpsStatus.ucIdle = TRUE;
		}
	}
	else                        //awakeup
	{
	
		if(GpsStatus.ucStart_Mode == GPS_COLD_RESTART)
		{
			if((u32)(ulSysTime - ulPretick) > TIME_S(GPS_COLD_WAIT_S))
			{
				ulPretick = ulSysTime;

				GpsStatus.ucIdle = FALSE;
				GPS_VCC_OFF();
				
				PRINTF("GPS Cold TimeOut!\r\n");
			}
		}
		else if(GpsStatus.ucStart_Mode == GPS_HOT_RESTART)
		{
		
			if((u32)(ulSysTime - ulPretick) > TIME_S(GPS_HOT_WAIT_S))
			{
				ulPretick = ulSysTime;

				GpsStatus.ucIdle = FALSE;
				GPS_VCC_OFF();
				
				PRINTF("GPS Hot TimeOut!\r\n");
			}
		}
		else
		{
			
		}
		
		
		if(Gps.isUsefull == TRUE)     //update gps date
		{
			Gps.isUsefull = FALSE;

			if(GpsStatus.ucStart_Mode == GPS_COLD_RESTART)
			{
				return;
			}
			
			GPS_VCC_OFF();
			
			GpsStatus.ucIdle = FALSE;
			
			ulPretick = ulSysTime;
			
			///LoRa report gps to server
			Gps_Rp.ulLati    = GpstoHex(Gps.latitude);			
			Gps_Rp.ulLong    = GpstoHex(Gps.longitude);
			Gps_Rp.ulUTC     = GpstoHex(Gps.UTCTime);
			Gps_Rp.ucLong_EW = (Gps.E_W[0] == 'W' ? 1 : 0);			
			Gps_Rp.ucLati_NS = (Gps.N_S[0] == 'N' ? 1 : 0);
			
			GPS_Info_Report(ulSysTime);
			
			PRINTF("GPS Off!\r\n");
			
		}
	}

	
}

//offline, then trigger rejoin or Period rejoin(10min)
void Process_Net_Status(u32 ulSysTime)
{
	static u32 ulPretick  = 0;

	if(SysStatus.ucJoin_Status != Join_TimeOut)
	{
		ulPretick = ulSysTime;
		return;
	}
	
	if((u32)(ulSysTime - ulPretick) > TIME_S(600))
	{
		ulPretick = ulSysTime;
		AwarePro_ReJoin();
		PRINTF("Period 10min Rejoin!\r\n");
	}
}

////////////////  uart ///////////////////
u8 IS_Set_ATCmd(u8 *buf)
{
	if( '=' == *buf )
	{
		return 1;
	}
	return 0;
	
}

u32 GpstoHex( u8* pData)
{
	float fData;
	
	fData = atof((const char*)pData);

	return (HL_ftoh(fData));
}

/************************************************************************/
/*	This function converts float data to 32bit Hex number					*/
/*	parameters:															*/
/*		pData		float data need to be converted							*/
/*	Returns:															*/
/*		u32		Convert result										*/
/************************************************************************/

u32 HL_ftoh(float fData)
{
	return (*(u32*)&fData);
}

/************************************************************************/
/*	This function converts 2-byte string to Hex number					*/
/*	parameters:															*/
/*		pData		string need to be converted							*/
/*	Returns:															*/
/*		INT8		Convert result										*/
/************************************************************************/
u8	HL_atoh( u8 * pData )
{
	u8	cValue = 0;

	if ( ( pData[0] >= '0' ) && ( pData[0] <= '9' ) )
		cValue = ( pData[0] - '0' ) << 4;
	else
		if ( ( pData[0] >= 'A' ) && ( pData[0] <= 'F' ) )
			cValue = ( pData[0] - 'A' + 10) << 4;

	if ( ( pData[1] >= '0' ) && ( pData[1] <= '9' ) )
		cValue |= ( pData[1] - '0' );
	else
		if ( ( pData[1] >= 'A' ) && ( pData[1] <= 'F' ) )
			cValue |= ( pData[1] - 'A' + 10);

	return	cValue;
}


void print_8_02x(uint8_t *pt)
{
  PRINTF("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
}

void LpUart1_Send(uint8_t* data, uint8_t len)
{	
	
//	HAL_UART_Transmit(&huart1, data, len, 1000);
	HAL_UART_Transmit_IT(&hlpuart1, data, len);
}

void Uart2_Send(uint8_t* data, uint8_t len)
{	
	
//	HAL_UART_Transmit(&huart1, data, len, 1000);
	HAL_UART_Transmit_IT(&huart2, data, len);
}


void Uart_Send(uint8_t* data, uint8_t len)
{	
	
//	HAL_UART_Transmit(&huart1, data, len, 1000);
	HAL_UART_Transmit_IT(&huart1, data, len);
}


void Store_UART_Data( u32 ulSysTime )
{
	/// uart2		

	if ( asUART0_Buffer[0].ucStatus & UART_FRAME_STATUS_READY ) 	// remain data need to be processed
	{
		if ( ( asUART0_Buffer[1].ucStatus & UART_FRAME_STATUS_MASK ) == 0 ) //no data sending
		{
			
				memcpy( (uint8_t*)&asUART0_Buffer[1], (uint8_t*)&asUART0_Buffer[0], sizeof(UART_BUFFER) );
				memset( (uint8_t*)&asUART0_Buffer[0], 0, sizeof(UART_BUFFER) );
			
				asUART0_Buffer[1].ucStatus = UART_FRAME_STATUS_READY;
		}
		else
		{
			return;
		}
	
	}

	//// uart1
	if ( asGPS_Buffer[0].ucStatus & UART_FRAME_STATUS_READY ) 	// remain data need to be processed
	{
		if ( ( asGPS_Buffer[1].ucStatus & UART_FRAME_STATUS_MASK ) == 0 ) //no data sending
		{
			
				memcpy( (uint8_t*)&asGPS_Buffer[1], (uint8_t*)&asGPS_Buffer[0], sizeof(GPS_BUFFER) );
				memset( (uint8_t*)&asGPS_Buffer[0], 0, sizeof(GPS_BUFFER) );
			
				asGPS_Buffer[1].ucStatus = UART_FRAME_STATUS_READY;
		}
		else
		{
			return;
		}
	
	}

	/// uart2
//	if ( asUART2_Buffer[0].ucStatus & UART_FRAME_STATUS_READY ) 	// remain data need to be processed
//	{
//		if ( ( asUART2_Buffer[1].ucStatus & UART_FRAME_STATUS_MASK ) == 0 ) //no data sending
//		{
//			
//				memcpy( (uint8_t*)&asUART2_Buffer[1], (uint8_t*)&asUART2_Buffer[0], sizeof(UART_BUFFER));
//				memset( (uint8_t*)&asUART2_Buffer[0], 0, sizeof(UART_BUFFER) );
//			
//				asUART2_Buffer[1].ucStatus = UART_FRAME_STATUS_READY;
//		}
//		else
//		{
//			return;
//		}
//	}

	/// lpuart1
	/*
	if ( asLPUART1_Buffer[0].ucStatus & UART_FRAME_STATUS_READY ) 	// remain data need to be processed
	{
		if ( ( asLPUART1_Buffer[1].ucStatus & UART_FRAME_STATUS_MASK ) == 0 ) //no data sending
		{
			
				memcpy( (uint8_t*)&asLPUART1_Buffer[1], (uint8_t*)&asLPUART1_Buffer[0], sizeof(UART_BUFFER) );
				memset( (uint8_t*)&asLPUART1_Buffer[0], 0, sizeof(UART_BUFFER) );
			
				asLPUART1_Buffer[1].ucStatus = UART_FRAME_STATUS_READY;
		}
		else
		{
			return;
		}
	}
	*/

}

u8	Process_UART_Data( u32 ulSysTime )
{

#if 1
	if( asUART0_Buffer[1].ucStatus & UART_FRAME_STATUS_READY )         // from uart 
	{			
		asUART0_Buffer[1].ucStatus = 0;
	
		if(UART_AT_CMD_Process(asUART0_Buffer[1].aucData, asUART0_Buffer[1].ucLength, ulSysTime))
		{
		
			PRINTF((const char*)asUART0_Buffer[1].aucData);

			//////////////  test ///////
			
			Parse_GpsBuffer((u8*)strstr((const char*)asUART0_Buffer[1].aucData, "RMC"), &Gps);

			if(Gps.isUsefull == TRUE)
			{
				Gps.isUsefull = FALSE;

				Gps_Rp.ulLati    = GpstoHex(Gps.latitude);			
				Gps_Rp.ulLong    = GpstoHex(Gps.longitude);
				Gps_Rp.ulUTC     = GpstoHex(Gps.UTCTime);
				Gps_Rp.ucLong_EW = (Gps.E_W[0] == 'W' ? 1 : 0);			
				Gps_Rp.ucLati_NS = (Gps.N_S[0] == 'N' ? 1 : 0);

				
				GPS_Info_Report(ulSysTime);

				PRINTF("\r\nUTC:");
				PRINTF((const char*)Gps.UTCTime);
				PRINTF("\r\n");

				PRINTF("Lat:");
				PRINTF((const char*)Gps.latitude);					
				PRINTF("\r\n");

				PRINTF("N_S:");
				PRINTF((const char*)Gps.N_S);					
				PRINTF("\r\n");

				PRINTF("Lon:");
				PRINTF((const char*)Gps.longitude);
				PRINTF("\r\n");

				PRINTF("E_W:");
				PRINTF((const char*)Gps.E_W);					
				PRINTF("\r\n");
				
				PRINTF("UTC:0x%x\r\n", Gps_Rp.ulUTC);

				PRINTF("ulLati:0x%x\r\n", Gps_Rp.ulLati);
				
				PRINTF("ulLong:0x%x\r\n", Gps_Rp.ulLong);
				
				PRINTF("N_S:0x%x\r\n", Gps_Rp.ucLati_NS);
				
				PRINTF("E_W:0x%x\r\n", Gps_Rp.ucLong_EW);
			}
			
//			if(asUART0_Buffer[1].ucLength == OHR_LEN)
//			{
//				OHR_Typedef* Ohr = (OHR_Typedef*)asUART0_Buffer[1].aucData;

//				if((Ohr->ucHead == HEAD_OHR2HOST)&&(Ohr->ucCmd == CMD_NO_RAW))
//				{
//					PRINTF("HeartRate:%d\r\n", Ohr->ucHeartRate);
//				}
//			}
			
//			AwarePro_UART_Rec_Process(asUART0_Buffer[1].aucData, asUART0_Buffer[1].ucLength, ulSysTime);

			/*
			///for test
			LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)(asUART0_Buffer[1].aucData);

			if(LoRa->ucLoRa_Cmd == APP_DATA_TRANS)
			{

				//////  App data trans
				AwarePro_RF_Tx_Process(LoRa->ucPayload, asUART0_Buffer[1].ucLength - 9, LoRa->uiDst_Addr, 0, ulSysTime);
			}
			else
			{
				u8 Opt = LoRa->ucLoRa_Cmd & OPT_MASK;

				if(Opt == READ_CMD)
				{
					Opt = OPT_READ;
				}
				else
				{
					Opt = OPT_WRITE;
				}

				//  Rf config
				AwarePro_RF_Config(LoRa->uiDst_Addr, (CMD_Enum)(LoRa->ucLoRa_Cmd & End_CMD), LoRa->ucPayload, 1, Opt, ulSysTime);
			}

			////////  for test

			*/

			
			
		}
//		memset( (void*)&asUART0_Buffer[1], 0, sizeof(UART_BUFFER) );		
		
	}
#endif

	if( asGPS_Buffer[1].ucStatus & UART_FRAME_STATUS_READY )		   // from uart 
	{			
		asGPS_Buffer[1].ucStatus = 0;
	
		
		char* SAddr = NULL;
		
//			PRINTF((const char*)asGPS_Buffer[1].aucData);

		if(strstr((const char*)asGPS_Buffer[1].aucData, "RMC") != NULL)   //poll $GNRMC/$GPRMC
		{
//			GPS_Typedef Gps = {0};
				
			SAddr = strstr((const char*)asGPS_Buffer[1].aucData, "RMC");
			
			PRINTF(SAddr);

			Parse_GpsBuffer((u8*)SAddr, &Gps);

			if(Gps.isUsefull == TRUE)
			{
				PRINTF("UTC:");
				PRINTF((const char*)Gps.UTCTime);
				PRINTF("\r\n");

				PRINTF("Lat:");
				PRINTF((const char*)Gps.latitude);					
				PRINTF("\r\n");

				PRINTF("N_S:");
				PRINTF((const char*)Gps.N_S);					
				PRINTF("\r\n");

				PRINTF("Lon:");
				PRINTF((const char*)Gps.longitude);
				PRINTF("\r\n");

				PRINTF("E_W:");
				PRINTF((const char*)Gps.E_W);					
				PRINTF("\r\n");
				
			}
			
		}
		else
		{
//			PRINTF((const char*)asGPS_Buffer[1].aucData);
		}

			
			
//			if(strstr((const char*)asGPS_Buffer[0].aucData, "\r\n") != NULL)
//			{

//				EAddr = strstr((const char*)asGPS_Buffer[0].aucData, "\r\n");
//			
//				do
//				{
//					EAddr++;
//				}
//				while((Addr = strstr((const char*)EAddr, "\r\n")) != NULL);
//			}
				
		
	}

	
//	if( asUART2_Buffer[1].ucStatus & UART_FRAME_STATUS_READY )         // from uart2 
//	{			
//		u8 i = 0;
//		
//		asUART2_Buffer[1].ucStatus = 0;

//		/// BD/GPS uart data  //
//		PRINTF("Data:");

//		for(i = 0; i < asUART2_Buffer[1].ucLength; i++)
//		{
//			PRINTF("%c",asUART2_Buffer[1].aucData[i]);
//		}
//		
//		PRINTF("\r\n");
//	}

	/*
	if( asLPUART1_Buffer[1].ucStatus & UART_FRAME_STATUS_READY )         // from uart2 
	{			
		asLPUART1_Buffer[1].ucStatus = 0;
	
		if(asLPUART1_Buffer[1].ucLength == OHR_LEN)
		{
			OHR_Typedef* Ohr = (OHR_Typedef*)asLPUART1_Buffer[1].aucData;

			if((Ohr->ucHead == HEAD_OHR2HOST)&&(Ohr->ucCmd == CMD_NO_RAW))
			{
//				SysStatus.ucHeartRate =  Ohr->ucHeartRate;
				PRINTF("HeartRate:%d\r\n", Ohr->ucHeartRate);

				ucHeart_Status = 0;
				
				OHR_status = OHR_STATUS_DISABLED;
			
				TimerStop(&timer_ohr);	
				
			}
//			else if((Ohr->ucHead == HEAD_OHR2HOST)&&(Ohr->ucCmd == START_RESP))
//			{

//				Uart2_Send(asUART2_Buffer[1].aucData, asUART2_Buffer[1].ucLength);

//				ucHeart_Status = 0;
//				
//				OHR_status = OHR_STATUS_DISABLED;
//			}
		}
		else
		{
			PRINTF((const char*)asLPUART1_Buffer[1].aucData);
		}

		
	}
	*/
	
	return 0;
}


void Parse_GpsBuffer(u8* Data, GPS_Typedef* Gps)
{
	char *subString;
	char *subStringNext;
	char i = 0;
//	GPS_Typedef* Gps = (GPS_Typedef*)Data;
	
	for (i = 0 ; i <= 6 ; i++)
	{
		if (i == 0)
		{
			if ((subString = strstr((const char*)Data, ",")) == NULL)
				return;	//解析错误
		}
		else
		{
			subString++;
			if ((subStringNext = strstr(subString, ",")) != NULL)
			{
				char usefullBuffer[2]; 
				switch(i)
				{
					case 1:memcpy(Gps->UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
					case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
					case 3:memcpy(Gps->latitude, subString, subStringNext - subString);break;	//获取纬度信息
					case 4:memcpy(Gps->N_S, subString, subStringNext - subString);break;	//获取N/S
					case 5:memcpy(Gps->longitude, subString, subStringNext - subString);break;	//获取经度信息
					case 6:memcpy(Gps->E_W, subString, subStringNext - subString);break;	//获取E/W

					default:break;
				}

				subString = subStringNext;
//				Gps.isParseData = true;
				if(usefullBuffer[0] == 'A')
					Gps->isUsefull = TRUE;
				else if(usefullBuffer[0] == 'V')
					Gps->isUsefull = FALSE;

			}
			else
			{
				return;	//解析错误
			}
		}

	}
}

u8 GetCH(void)
{
	return (RFConfig.ucRF_Channel);
//	return (Sys_ConfigOption.RF_FRQ_CHN);
}

u8 SetCH(u8 ucRF_CH)
{
	RFConfig.ucRF_Channel = ucRF_CH;

	#if defined(NV)

	Write_App_NV();
	
	#endif
	
	return 0;

//	Sys_ConfigOption.RF_FRQ_CHN = ucRF_CH;
}

u8 GetBW(void)
{	
	return (RFConfig.ucBand_Width);
}

u8 SetBW(u8 ucRF_BW)
{
	RFConfig.ucBand_Width = ucRF_BW;

	#if defined(NV)

	Write_App_NV();
	
	#endif
	
	return 0;
}


u8 GetRFRate(void)
{	
	return (RFConfig.ucRF_Rate);
}

u8 SetRFRate(u8 ucRF_Rate)
{
	RFConfig.ucRF_Rate = ucRF_Rate;

	#if defined(NV)

	Write_App_NV();
	
	#endif
	
	return 0;
}

u8 GetRFPwr(void)
{	
	return (RFConfig.ucRF_Pwr);
}

u8 SetRFPwr(u8 ucRF_Pwr)
{
	RFConfig.ucRF_Pwr = ucRF_Pwr;

	#if defined(NV)

	Write_App_NV();
	
	#endif
	
	return 0;
}

u8 UART_AT_CMD_Process(u8* data, u8 ucLen, u32 ulSysTime)
{
	if((data[0] == 'A')&&(data[1] == 'T')&&(data[2] == '#'))
	{
		
		if(strncmp(AtOpMode, (const char*)data, sizeof(AtOpMode)-1) == 0)  
		{
			u8 OpMode = 0;

			OpMode = SX1276Read( REG_OPMODE )& ~RF_OPMODE_MASK;
			PRINTF("OpMode:0x%02x\r\n",OpMode);
//			PRINTF("\t\n");
			PRINTF("LoRa state:%d\r\n",Radio.GetStatus());   //0:idel,1:Rx_Running,2:Tx_Running
//			PRINTF("\n");
			return 0;
		}
		else if(strncmp(AtSleep,(const char*)data, sizeof(AtSleep)-1) == 0)   
		{
			
			Radio.Sleep();
			PRINTF("Enter Sleep Mode!\n");
			
			return 0;
		}
		else if(strncmp(AtStandby, (const char*)data, sizeof(AtStandby)-1) == 0)   
		{
			
			Radio.Standby();
			
			PRINTF("Enter Standby Mode!\n");
			
			return 0;
		}
		else if(strncmp(AtPoll, (const char*)data, sizeof(AtPoll)-1) == 0)   
		{
			
			AwarePro_ZED_Poll_Once();
			
			PRINTF("ZED Poll Req!\n");
			
			return 0;
		}
		else if(strncmp(AtRst, (const char*)data, sizeof(AtRst)-1) == 0)   
		{
			Event_Flg  = EVN_RST;
			
//			NVIC_SystemReset();
			
			PRINTF("Enter Reset!\n");
			
			return 0;
		}
		else if(strncmp(AtLeave, (const char*)data, sizeof(AtLeave)-1) == 0)   
		{
			
			AwarePro_Leave(ulSysTime);
			
			PRINTF("Leave Req!\n");
			
			return 0;
		}
		else if(strncmp(AtJoin, (const char*)data, sizeof(AtJoin)-1) == 0)   
		{

			if(AwarePro_NV_JoinFlg() == NET_FLG)
			{
				
				PRINTF("Online!\r\n");
			}
			else
			{
				
				PRINTF("Offline!\r\n");
			}
			
			
			return 0;
		}
		else if(strncmp(AtVer, (const char*)data, sizeof(AtVer)-1) == 0)   
		{
			
			PRINTF("LoRa Version:0x%08x, APP Version:0x%08x\n",LORA_MAC_VERSION,APP_VERSION);
			
			return 0;
		}
//		else if(strncmp(AtBat, (const char*)data, sizeof(AtBat)-1) == 0)   
//		{
//			u8 Batlevel  = 0;
//			Batlevel = HW_GetBatteryLevel();
//			
//			PRINTF("BatLevel:%d\n",Batlevel);
//			
//			return 0;
//		}
		else if(strncmp(AtUid, (const char*)data, sizeof(AtUid)-1) == 0)   
		{
			u8 UID[8] = {0};
			u8 i;
			
			HW_GetUniqueId(UID);
			PRINTF("DevEui= %02X", UID[0]);
			for( i = 1; i < 8; i ++) 
			{
				PRINTF("-%02X", UID[i]); 
			} 
			PRINTF("\n\r");
			
			return 0;
		}
		else if(strncmp(AtRxTime, (const char*)data, sizeof(AtRxTime)-1) == 0)   //AT#BR=115200
		{
			
			u8 len = sizeof(AtRxTime)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Time; 						
				
				Time = HL_atoh(data + offset);	

				Radio.Rx(Time*1000);
				
				PRINTF("Set RxTimeOut:0x%02x Second!\n",Time);
			}
			
			
			return 0;
		}
		else if(strncmp(AtCmac, (const char*)data, sizeof(AtCmac)-1) == 0)   
		{
			#if !defined(COO)
			u8 i;
			PRINTF("COO DevEui= %02X", Nv_Para.sHost_Addr.aucAddr[0]);
			for(i = 1; i < 8; i ++) 
			{
				PRINTF("-%02X", Nv_Para.sHost_Addr.aucAddr[i]); 
			} 
			PRINTF("\n\r");
			#else
				PRINTF("Host of COO is itself!\n");
			#endif
			
			return 0;
		}
		else if(strncmp(AtNodeTab, (const char*)data, sizeof(AtNodeTab)-1) == 0)   
		{
			#if defined(COO) || defined(Router)
 
			u8 i;
			for(i = 0; i < MAX_ROUTABLE; i++)
			{
				if(memcmp(&RoutTable[i].sNode_Addr, &NullAddr, sizeof(Dev_Addr)) != 0)
				{

					PRINTF("Node Addr:");
					print_8_02x(RoutTable[i].sNode_Addr.aucAddr);
								
					PRINTF("Node NwkAddr:0x%04x\r\n", RoutTable[i].uiAddr);
					PRINTF("NodeType:0x%02x\r\n", RoutTable[i].ucNode_Type);		
					PRINTF("Node Age:0x%02x\r\n", RoutTable[i].ucAge);				
					PRINTF("Node Seq:0x%02x\r\n", RoutTable[i].ucSeq);
				
				}
			}
			
			#else
				PRINTF("Node is ZED!\n");
			#endif
			
			return 0;
		}
		else if(strncmp(AtPollBuf, (const char*)data, sizeof(AtPollBuf)-1) == 0)   
		{
			#if defined(COO)|| defined(Router)

			
			extern POLL_BUFFER_Typedef asPoll_Buffer[POLL_BUFFER_SIZE];
			u8 i;
			for(i = 0; i < POLL_BUFFER_SIZE; i++)
			{
				if(asPoll_Buffer[i].ucMessage_Length != 0)
				{
					PRINTF("Child Addr:");
					print_8_02x(asPoll_Buffer[i].sChild_Addr.aucAddr);
				
					PRINTF("Node Age:0x%02x\r\n", asPoll_Buffer[i].ucAge);
			
				}
			}
			
			#else
				PRINTF("Node is ZED!\n");
			#endif
			
			return 0;
		}
		else if(strncmp(AtDefault,(const char*)data, sizeof(AtDefault)-1) == 0)   
		{
			
			
//			#if defined(NV)
//			
//				Delete_NV();
//				
//				#if defined(COO)
//						
//					Erase_NodeTable();
//					
//					PRINTF("Node Restore Default!\r\n");
//					
//					#elif defined(Router)

//					Erase_NodeTable();
//					
//					PRINTF("Node Restore Default!\r\n");
//				
//				#else

//				#endif
//				
//			#endif

			AwarePro_Default();		
			Delete_App_NV();

			PRINTF("Node Restore Default!\r\n");
			Event_Flg  = EVN_RST;

//			NVIC_SystemReset();
			
			return 0;
		}
		else if(strncmp(AtRFRate, (const char*)data, sizeof(AtRFRate)-1) == 0)   
		{
		
			u8 len = sizeof(AtRFRate)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Rate; 						
				u8 Bw = GetBW();
				
				Rate = HL_atoh(data + offset);	

				if(Rate > RF_Rate_11000)
				{
					PRINTF("Error Parameter!\r\n");
					return 0;
				}
				
				RF_DataRate_Config((RF_Rate_Enum)Rate, &Bw, GetRFPwr());
//				Nv_Para.ucRF_Rate = Rate;
				SetRFRate(Rate);
				
				PRINTF("RF Rate:0x%02x!\r\n", Rate);
			}
			else
			{
				PRINTF("RF Rate:0x%02x!\r\n", GetRFRate());
			}
			
			return 0;
		}
		else if(strncmp(AtRFCH, (const char*)data, sizeof(AtRFCH)-1) == 0)   
		{
		
			u8 len = sizeof(AtRFCH)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 ch; 	
//				u8 OpMode;
				
				ch = HL_atoh(data + offset);	
				
				RF_CH_Config(ch, GetBW());
				
//					Nv_Para.ucRF_Channel = ch;
				SetCH(ch);
				
				PRINTF("RF CH:0x%02x!\r\n", ch);

				
			}
			else
			{
				PRINTF("RF Rate:0x%02x!\r\n", GetCH());
			}
			
			return 0;
		}
		else if(strncmp(AtPWR, (const char*)data, sizeof(AtPWR)-1) == 0)   
		{
		
			u8 len = sizeof(AtPWR)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 pwr; 						
				
				pwr = HL_atoh(data + offset);	

				if(pwr <= 20)
				{
					
					SX1276SetRfTxPower( pwr );
					
//					Nv_Para.ucRF_Pwr = pwr;
					SetRFPwr(pwr);		
					
					PRINTF("Pwr:0x%02x!\r\n", pwr);

				}
				else
				{
					
					
					PRINTF("Pwr Value Error!\r\n");

				}

				
			}
			
			else
			{
				PRINTF("Pwr:0x%02x!\r\n", GetRFPwr());
			}
			
			return 0;
		}
		else if(strncmp(AtPollGap, (const char*)data, sizeof(AtPollGap)-1) == 0)   
		{		
			
			u8 len = sizeof(AtPollGap) - 1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Gap; 						
				
				Gap = HL_atoh(data + offset);	

				if( Gap < POLL_GAP_MIN )
				{
					PRINTF("Error Parameter!\r\n");
					return 0;
				}
				
//				Nv_Para.ucPoll_TimeOut = Gap;
				AwarePro_SetPGap(Gap);

				#if defined(ZED)
				
				PRINTF("ZED Poll Gap:0x%02x!\r\n", Gap);
				
				#else
				
				PRINTF("PollBuf TimeOut:0x%02x!\r\n", Gap);

				#endif
			}
			else
			{
				
				#if defined(ZED)
				
				PRINTF("ZED Poll Gap:0x%02x!\r\n", AwarePro_GetPGap());

				#else
				
				PRINTF("PollBuf TimeOut:0x%02x!\r\n", AwarePro_GetPGap());

				#endif
			}
			
			return 0;
		}
		else if(strncmp(AtBeatGap, (const char*)data, sizeof(AtBeatGap)-1) == 0)   
		{
		
			#if defined(Router)
			
			u8 len = sizeof(AtBeatGap) - 1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Gap; 						
				
				Gap = HL_atoh(data + offset);	

				if(Gap == 0)
				{

					AwarePro_SetHBGap(Gap);
					
					PRINTF("HeartBeat Cancel!\r\n");
					return 0;
				}

				if( Gap < HEARTBEAT_GAP_MIN )
				{
					PRINTF("Error Parameter!\r\n");
					return 0;
				}
				
				AwarePro_SetHBGap(Gap);				
				
				PRINTF("Router HeatBeat Gap:0x%02x!\r\n", Gap);
			}
			else
			{
				
				PRINTF("Router HeatBeat Gap:0x%02x!\r\n", AwarePro_GetHBGap());
			}

			#else
				PRINTF("Not Router Node!\r\n");
			#endif
			
			return 0;
		}

		else if(strncmp(AtPID, (const char*)data, sizeof(AtPID)-1) == 0)   
		{
		
			u8 len = sizeof(AtPID)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 pid; 						
				
				pid = HL_atoh(data + offset);	

				if(pid != 0)
				{
											
					AwarePro_SetPID(pid);
					
					PRINTF("PANID:0x%02x!\r\n", pid);

				}
				else
				{
					
					
					PRINTF("PANID Value Error!\r\n");

				}

				
			}
			
			else
			{
				PRINTF("PANID:0x%02x!\r\n", AwarePro_GetPID());
			}
			
			return 0;
		}
		else if(strncmp(AtTst, (const char*)data, sizeof(AtTst)-1) == 0)   //AT#BR=115200
		{
			
			u8 len = sizeof(AtTst)-1;

			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 time; 
				u8 Period;
				
				time = HL_atoh(data + offset);	
				Period = HL_atoh(data + offset + 2);	

//				LED_Show(Red_Led, Flash, time, FLASH_2S);			
//				LED_Show(Blue_Led, Flash, time, FLASH_2S);
				SPK_Show(Flash, time, Period);
					
//				Tag_Bind_Resp(1, ulSysTime);
//				Process_LED_Event(ulSysTime);
				
//				PRINTF("Led Flash Time:%d!\n", time);
			}
			
			
			return 0;
		}
		else if(strncmp(Atbind, (const char*)data, sizeof(Atbind)-1) == 0)   //AT#BR=115200
		{
			
			u8 len = sizeof(Atbind)-1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
//				u32 id; 

//				osal_revmemcpy((u8*)&RFConfig.aucMatch_SN1,const void * src,u32 len)
				RFConfig.aucMatch_SN1.aucAddr[0] = 0x80;
				RFConfig.aucMatch_SN1.aucAddr[1] = 0x01;
				RFConfig.aucMatch_SN1.aucAddr[2] = 0x01;
				RFConfig.aucMatch_SN1.aucAddr[3] = 0x37;
				
				RFConfig.aucMatch_SN1.aucAddr[4] = (u32)HL_atoh(data + offset);
				
				RFConfig.aucMatch_SN1.aucAddr[5] = (u32)HL_atoh(data + offset + 2);

				RFConfig.aucMatch_SN1.aucAddr[6] = (u32)HL_atoh(data + offset + 4);

				RFConfig.aucMatch_SN1.aucAddr[7] = (u32)HL_atoh(data + offset + 6);

//				RFConfig.ulMatch_ID1 = (u32)HL_atoh(data + offset)<<24
//										|(u32)HL_atoh(data + offset + 2)<<16
//										|(u32)HL_atoh(data + offset + 4)<<8
//										|(u32)HL_atoh(data + offset + 6);	

//				RFConfig.ucBind_Status = SN_MATCH_STATUS;
				
//				PRINTF("Match id:0x%x!\n", RFConfig.ulMatch_ID1);
				print_8_02x((u8*)&RFConfig.aucMatch_SN1);

			}
			
			
			return 0;
		}
		else if(strncmp(AtOHR, (const char*)data, sizeof(AtOHR)-1) == 0)   
		{
		
			u8 len = sizeof(AtOHR) - 1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Gap; 						
				
				Gap = HL_atoh(data + offset);	

				if(Gap == 0)
				{
					StopOHR();
					ucHeart_Status = 0;
					PRINTF("Stop OHR!\r\n");
				}
				else if(Gap == 0x01)
				{
					StartOHR(2, 10);	
//					StartOHR(0, 0);				  //heartbeat start
					PRINTF("Start fix Cur OHR!\r\n");
				}	
				else
				{
					StartOHR(0, 0);	
//					StartOHR(0, 0); 			  //heartbeat start
					PRINTF("Start OHR!\r\n");
				}

				
			}
	
			return 0;
		}
		else if(strncmp(AtGPS, (const char*)data, sizeof(AtGPS)-1) == 0)   
		{
		
			u8 len = sizeof(AtGPS) - 1;
					
			if( IS_Set_ATCmd(data + len) )
			{
				u8 offset = 1 + len;
				u8 Gap; 						
				
				Gap = HL_atoh(data + offset);	

				if(Gap == 0)
				{
					GPS_VCC_OFF();
					PRINTF("GPS OFF!\r\n");
				}
				else if(Gap == 0x01)
				{
					GPS_VCC_ON();
					PRINTF("GPS ON!\r\n");
				}	
				else
				{
					
				}

				
			}
	
			return 0;
		}
		else
		{
			PRINTF("AT cmd unvalid!\n");
			return 0;
		}	
		
	}

	
	return 1;
}


/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */

	NVIC_SystemReset();
  
/*
  	if(huart == &huart1)
	{
		
		HAL_UART_DMAStop(&huart1);
		asUART0_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		asUART0_Buffer[0].ucLength = UART_FRAME_SIZE;
		HAL_UART_Receive_DMA(&huart1, asUART0_Buffer[0].aucData, UART_FRAME_SIZE+1);

	}
	else if(huart == &huart2)
	{
	
		HAL_UART_DMAStop(&huart2);
		asGPS_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		asGPS_Buffer[0].ucLength = GPS_FRAME_SIZE;
		HAL_UART_Receive_DMA(&huart2, asGPS_Buffer[0].aucData, GPS_FRAME_SIZE+1);
		
//		HAL_UART_DMAStop(&huart2);
//		asUART2_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
//		asUART2_Buffer[0].ucLength = UART_FRAME_SIZE;
//		HAL_UART_Receive_DMA(&huart2, asGPS_Buffer[0].aucData, UART_FRAME_SIZE+1);
	}
	else if(huart == &hlpuart1)
	{
		HAL_UART_DMAStop(&hlpuart1);
		asLPUART1_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		asLPUART1_Buffer[0].ucLength = UART_FRAME_SIZE;
		HAL_UART_Receive_DMA(&hlpuart1, asLPUART1_Buffer[0].aucData, UART_FRAME_SIZE+1);
	}
	else
		{
		}

		*/
	
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(husart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_RxHalfCpltCallback can be implemented in the user file
   */

}


void Uart_Rec_Idle(UART_HandleTypeDef *huart, DMA_HandleTypeDef *dma)
{
	
	u32 temp = 0;
    
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
	{ 
		  __HAL_UART_CLEAR_IDLEFLAG(huart);
		  __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
		//	  temp = huart1.Instance->SR;		   //clear flag
		//	  temp = huart1.Instance->DR;		   //clear flag
		  HAL_UART_DMAStop(huart);

		//	  temp	= hdma_usart1_rx.Instance->NDTR;  
		  temp	= __HAL_DMA_GET_COUNTER(dma);

		if(huart == &huart2)
		{
			
			if(asUART0_Buffer[0].ucStatus != UART_FRAME_STATUS_READY)
			{
				asUART0_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;
			
				if(asUART0_Buffer[0].ucLength > 0)
				{
				
					asUART0_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
				}
			} 
					
			HAL_UART_Receive_DMA(&huart2, asUART0_Buffer[0].aucData, UART_FRAME_SIZE+1);
		}
		else if(huart == &huart1)
		{

			if(asGPS_Buffer[0].ucStatus != UART_FRAME_STATUS_READY)
			{
				asGPS_Buffer[0].ucLength = GPS_FRAME_SIZE - temp + 1;
			
				if(asGPS_Buffer[0].ucLength > 0)
				{
						
					asGPS_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
				}
			}
				
			HAL_UART_Receive_DMA(&huart1, asGPS_Buffer[0].aucData, GPS_FRAME_SIZE+1);
			
//			asUART2_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;
//			
//			if(asUART2_Buffer[0].ucLength > 0)
//			{
//				
//				asUART2_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
//			}
//	
//			HAL_UART_Receive_DMA(&huart2, asUART2_Buffer[0].aucData, UART_FRAME_SIZE+1);
		}

		/*
		else if(huart == &hlpuart1)
		{
			
			asLPUART1_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;

			if(asLPUART1_Buffer[0].ucLength > 0)
			{

				asLPUART1_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
			}

			HAL_UART_Receive_DMA(&hlpuart1, asLPUART1_Buffer[0].aucData, UART_FRAME_SIZE+1);
  
		}
		*/
		
		else
			{
			}

		
	}
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  Uart_Rec_Idle(&huart1, &hdma_usart1_rx);

  /*
  uint32_t tmp_flag = 0;
  uint32_t temp;
  
  tmp_flag =  __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); 
  
  if((tmp_flag != RESET))
  { 
	  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
	  __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_IDLE);
//	  temp = huart1.Instance->SR;		   //clear flag
//	  temp = huart1.Instance->DR;		   //clear flag
	  HAL_UART_DMAStop(&huart1);

//	  temp	= hdma_usart1_rx.Instance->NDTR;  
	  temp	= __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);


		asGPS_Buffer[0].ucLength = GPS_FRAME_SIZE - temp + 1;

		if(asGPS_Buffer[0].ucLength > 0)
		{
				
			asGPS_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		}
	 
		
		HAL_UART_Receive_DMA(&huart1, asGPS_Buffer[0].aucData, GPS_FRAME_SIZE+1);

	
//		asUART0_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;

//		if(asUART0_Buffer[0].ucLength > 0)
//		{
//			
//			asUART0_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
//		}
//	 
//		
//		HAL_UART_Receive_DMA(&huart1, asUART0_Buffer[0].aucData, UART_FRAME_SIZE+1);

   }
   */

  /* USER CODE END USART1_IRQn 1 */
}


/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART1_IRQn 1 */

  Uart_Rec_Idle(&huart2, &hdma_usart2_rx);

  /*
  uint32_t tmp_flag = 0;
  uint32_t temp;
  
  tmp_flag =  __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE); 
  
  if((tmp_flag != RESET))
  { 
	  __HAL_UART_CLEAR_IDLEFLAG(&huart2);
	  __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_IDLE);
//	  temp = huart1.Instance->SR;		   //clear flag
//	  temp = huart1.Instance->DR;		   //clear flag
	  HAL_UART_DMAStop(&huart2);

//	  temp	= hdma_usart1_rx.Instance->NDTR;  
	  temp	= __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

	
		asUART2_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;

		if(asUART2_Buffer[0].ucLength > 0)
		{
			
			asUART2_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		}
	 
		
		HAL_UART_Receive_DMA(&huart2, asUART2_Buffer[0].aucData, UART_FRAME_SIZE+1);

   }
   */

  /* USER CODE END USART1_IRQn 1 */
}

void AES_RNG_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN AES_RNG_LPUART1_IRQn 0 */

  /* USER CODE END AES_RNG_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN AES_RNG_LPUART1_IRQn 1 */
 
  Uart_Rec_Idle(&hlpuart1, &hdma_lpuart1_rx);
  /*
  	uint32_t tmp_flag = 0;
	uint32_t temp;
	
	tmp_flag =	__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_IDLE); 
	
	if((tmp_flag != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&hlpuart1);
		__HAL_UART_CLEAR_FLAG(&hlpuart1, UART_FLAG_IDLE);
  //	temp = huart1.Instance->SR; 		 //clear flag
  //	temp = huart1.Instance->DR; 		 //clear flag
		HAL_UART_DMAStop(&hlpuart1);
  
  //	temp  = hdma_usart1_rx.Instance->NDTR;	
		temp  = __HAL_DMA_GET_COUNTER(&hdma_lpuart1_rx);
  
	  
		  asLPUART1_Buffer[0].ucLength = UART_FRAME_SIZE - temp + 1;
  
		  if(asLPUART1_Buffer[0].ucLength > 0)
		  {
			  
			  asLPUART1_Buffer[0].ucStatus = UART_FRAME_STATUS_READY;
		  }
	   
		  
		  HAL_UART_Receive_DMA(&hlpuart1, asLPUART1_Buffer[0].aucData, UART_FRAME_SIZE+1);
  
	 }
	 */

  /* USER CODE END AES_RNG_LPUART1_IRQn 1 */
}

void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel 4, channel 5, channel 6 and channel 7 interrupts.
*/
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  HAL_DMA_IRQHandler(&hdma_lpuart1_rx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}


/**
* @brief This function handles TIM21 global interrupt.
*/
//void TIM21_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM21_IRQn 0 */

//  /* USER CODE END TIM21_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim21);
//  /* USER CODE BEGIN TIM21_IRQn 1 */

//  /* USER CODE END TIM21_IRQn 1 */
//}


/**
* @brief This function handles I2C2 interrupt.
*/
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}


void RTC_IRQHandler( void )  //RTC_Alarm_IRQn
{
	HW_RTC_IrqHandler( );
}


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	
	TimerIrqHandler( );
}

/* USER CODE BEGIN 1 */
/**
* @brief This function handles EXTI line 0 and line 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
	
  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
* @brief This function handles EXTI line 2 and line 3 interrupts.
*/
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */
	
  /* USER CODE END EXTI2_3_IRQn 0 */ 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);     //PB2 AWAKE 
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);  
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}


/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);     
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);     //PA5  OHR DAVAIL INT
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);    
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);    
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);     
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);     
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);     //PB12 key2
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);     //pb13 key1
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);     
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);

  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}


/////////////////  APP NV  /////////////////////
u8 Check_APP_NV_Para(void)
{
	if((RFConfig.ucStx == NV_STX)
		&&(RFConfig.ucEnd == NV_END))
	{
		u8 Sum = 0;
		Sum = Check_Sum(&RFConfig.ucRF_Channel, sizeof(RFConfig)-3);

		if(Sum == RFConfig.ucCheck)
		{
			return 0;
		}
	}
	
	return 1;
}

u8 Write_App_NV(void)
{
//	RFConfig.ucNv_Flag = 1;
	RFConfig.ucStx = NV_STX;
	RFConfig.ucEnd = NV_END;
	RFConfig.ucCheck = Check_Sum(&RFConfig.ucRF_Channel, sizeof(RFConfig)-3);

	Erase_Block_Flash(APP_DATA_ADDR, APP_DATA_LEN);
	Write_Data_To_Flash(APP_DATA_ADDR, (u8*)&RFConfig, sizeof(RFConfig));
	
	return 0;
	
}

u8 Read_App_NV(void)
{
	
	Read_Data_From_Flash(APP_DATA_ADDR, (u8*)&RFConfig, sizeof(RFConfig));
	
	return 0;
	
}


u8 Delete_App_NV(void)
{	
	
	Erase_Block_Flash(APP_DATA_ADDR, APP_DATA_LEN);
	
	return 0;
}

///////////////  key  /////////////////////
u8 KEY_Idle(void)
{
	if((Func_Key.ucKey_INT == 0)
		&&(Match_Key.ucKey_INT == 0))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void Key_GPIO_INT_Init(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin  = KEY_PIN|M_KEY_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//	HAL_NVIC_SetPriority( EXTI4_15_IRQn , 1, 0);      //LoRa IRQ enable
//	    
//    HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );

	Func_Key.ucKey = Key_Func;
	Match_Key.ucKey = Key_Match;
}

void Key_GPIO_INT_DeInit(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin  = KEY_PIN|M_KEY_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_NVIC_DisableIRQ( EXTI4_15_IRQn );
	
}

u8 Key_Read(u8 ucKey)
{
	if(ucKey == Key_Func)
	{
		return(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN));
	}
	else if(ucKey == Key_Match)
	{
		return(HAL_GPIO_ReadPin(M_KEY_PORT, M_KEY_PIN));
	}
	else
		{
		}

	return 0;
}

void Key_Status_Poll(Key_typedef *pKey, u32 ulSysTime)
{
	
	static u32 systick = 0;

	if(pKey->ucKey_Status == Key_Idle)
	{		
//		if(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN))	 //press on high level
		if(Key_Read(pKey->ucKey))
		{
			pKey->ucKey_Status = Key_Poll;
			systick = ulSysTime;
		
		}
		else
		{
			pKey->ucKey_INT = 0;
		}
	}
	else if(pKey->ucKey_Status == Key_Poll)
	{
		if((u32)(ulSysTime - systick) > KEY_PRESS_TIME)
		{
			systick = ulSysTime;
//			if(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN))   //press on
			
			if(Key_Read(pKey->ucKey))
			{				
				pKey->ucKey_Status = Key_On;//key_Press;
			}
			else
			{
				pKey->ucKey_Status = Key_Idle;
				
				pKey->ucKey_INT = 0;
				
			}
		}

	}
	else if(pKey->ucKey_Status == Key_On)
	{
		
//		if(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == 0)	 //release key
		
		if(Key_Read(pKey->ucKey) == 0)
		{
			pKey->ucKey_Status = key_Press;
		}
		else
		{
			if((u32)(ulSysTime - systick) > KEY_LONG_PRESS_TIME)
			{
				systick = ulSysTime;
//				if(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN))   //press on
				{				
					pKey->ucKey_Status = Key_Long;//key_Press;
				}
//				else
//				{
//					Key_Status = Key_Idle;
//				}
			}

		}
	}	
	else if(pKey->ucKey_Status == Key_Long)
	{
//		if(HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == 0)	 //release key
		
		if(Key_Read(pKey->ucKey) == 0)
		{
			pKey->ucKey_Status = Key_Long_Press;
		}
	}
	else
		{
		}
}

#if defined(ZED)
void Key_Func_Handler(Key_typedef *pKey, u32 ulSysTime)
{

	if(pKey->ucKey_Status == key_Press)
	{
		pKey->ucKey_Status = Key_Idle;
		pKey->ucKey_INT = 0;

		if(AwarePro_NV_JoinFlg() != NET_FLG)
		{
			AwarePro_ReJoin();
			
			LPMoveCount_Start();

			PRINTF("Offline!\r\n");
		}
		else if(RFConfig.ucReg_Status != Reg_Done)
		{
			RFConfig.ucReg_Status = Reg_None;
			
			LPMoveCount_Start();

			PRINTF("Reg not Succcess!\r\n");
		}
		else
		{
			
			if(pKey->ucKey == Key_Func)
			{
				
				PRINTF("SOS!\r\n");
			
				//////////	SOS report	////////
			
				Band_SOS_Report( 1, ulSysTime);
				LED_Show(Blue_Led, Flash, 2, FLASH_50MS);

				//// for Test  ////
				SPK_Show(Flash, 2, SPK_100MS);

				
//				if(GpsStatus.Run_Status != TRUE)
//				{
//					GpsStatus.Run_Status = TRUE;
//					GpsStatus.ucStart_Mode = GPS_COLD_RESTART;
//					GpsStatus.ucIdle = TRUE;
//					HAL_UART_MspInit(&huart2);					
//					GPS_VCC_ON();
//					PRINTF("GPS Cold on!\r\n");
//				}
				
			}
			else if(pKey->ucKey == Key_Match)
			{				
				PRINTF("Match Key Press!\r\n");

				if(SysStatus.ucWork_Status == WORK)
				{
					//////////// 125K Match Tag//////////
					if(SysStatus.ucBind_Status == SN_IDLE_STATUS)
					{
						LPMoveCount_Stop();
						LF_AS3932DeInit();
						
						LF_Init();      //SPK Init
						SysStatus.ucBind_Status = SN_TRIG_MATCH;
					}	
				}			
				
			}
			else
				{
				}
		}

	}
	else if(pKey->ucKey_Status == Key_Long_Press)
	{

		pKey->ucKey_Status = Key_Idle;
		pKey->ucKey_INT = 0;

		if(pKey->ucKey == Key_Match)
		{
			
			PRINTF("Default!\r\n");

			Delete_App_NV();
			Delete_NV();

			Event_Flg = EVN_RST;
			
		}


		if(AwarePro_NV_JoinFlg() != NET_FLG)
		{
			AwarePro_ReJoin();
			
			LPMoveCount_Start();

			PRINTF("Offline!\r\n");
		}
		else if(RFConfig.ucReg_Status != Reg_Done)
		{
			RFConfig.ucReg_Status = Reg_None;
			
			LPMoveCount_Start();

			PRINTF("Reg not Succcess!\r\n");
		}
		else
		{
			if(pKey->ucKey == Key_Func)
			{
				
				PRINTF("Func Key Long Press!\r\n");

				///// sleep mode to work mode /////
				
				if(SysStatus.ucWork_Status == SLEEP)
				{
					SysStatus.ucWork_Status = WORK;
					PRINTF("Sleep to Work!\r\n");
				}
				
			}
//			else if(pKey->ucKey == Key_Match)
//			{
//				
//				PRINTF("Default!\r\n");

//				Delete_App_NV();
//				Delete_NV();
//				
//			}
			else
				{
				}
		}

	}
}

#elif defined(COO)

void Key_Func_Handler(Key_typedef *pKey, u32 ulSysTime)
{

	if(pKey->ucKey_Status == key_Press)
	{
		pKey->ucKey_Status = Key_Idle;
		pKey->ucKey_INT = 0;

		
		if(pKey->ucKey == Key_Func)
		{
			
			PRINTF("SOS!\r\n");
		
			//////////	SOS report	////////
			LED_Show(Blue_Led, Flash, 2, FLASH_50MS);

			//// for Test  ////
			SPK_Show(Flash, 2, SPK_100MS);
			
		}
		else if(pKey->ucKey == Key_Match)
		{				
			PRINTF("Match Key Press!\r\n");	

			LED_Show(Blue_Led, Flash, 3, FLASH_50MS);
			SPK_Show(Flash, 2, SPK_100MS);
			
		}
		else
			{
			}

	}
	else if(pKey->ucKey_Status == Key_Long_Press)
	{

		pKey->ucKey_Status = Key_Idle;
		pKey->ucKey_INT = 0;

		if(pKey->ucKey == Key_Func)
		{
			
			PRINTF("Func Key Long Press!\r\n");

			///// sleep mode to work mode /////
			
		}
		else if(pKey->ucKey == Key_Match)
		{
			
			PRINTF("Default!\r\n");

			Delete_App_NV();
			Delete_NV();
			
		}
		else
			{
			}

	}
}


#endif



void Process_Key(u32 ulSysTime)
{
	Key_Status_Poll(&Func_Key,ulSysTime);	
	Key_Status_Poll(&Match_Key,ulSysTime);
	
	Key_Func_Handler(&Func_Key,ulSysTime);	
	Key_Func_Handler(&Match_Key,ulSysTime);
}

void Touch_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();	
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin  = Q1_VDD_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(Q1_VDD_GPIO_PORT, &GPIO_InitStruct);		
	HAL_GPIO_WritePin(Q1_VDD_GPIO_PORT, Q1_VDD_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin  = Q2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	HAL_GPIO_Init(Q2_GPIO_PORT, &GPIO_InitStruct);	

//	PRINTF("Touch Init!\r\n");
	
}

void Touch_GPIO_DeInit(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();	
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_DeInit(Q1_VDD_GPIO_PORT, Q1_VDD_PIN);		
	HAL_GPIO_DeInit(Q2_GPIO_PORT, Q2_PIN);		

	
//	PRINTF("Touch DeInit!\r\n");
}

//void SPK_PWM_ON(void)
//{
//	HAL_TIM_PWM_Init(&htim2);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//}

//void SPK_PWM_OFF(void)
//{
//	
//	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_PWM_DeInit(&htim2);
//}


/* TIM2 init function */
void SPK_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;        // 1M
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;//200us;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100; // 50%   
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

void SPK_GPIO_Init(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;

//	__HAL_RCC_GPIOB_CLK_ENABLE();

//	GPIO_InitStruct.Pin  = SPK_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

//	HAL_GPIO_Init(SPK_GPIO_PORT, &GPIO_InitStruct);	

//	SPK_OFF();

	SPK_TIM2_Init();

	SPK_PWM_OFF();
//	SPK_OFF();

}

void SPK_GPIO_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin  = SPK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(SPK_GPIO_PORT, &GPIO_InitStruct);	

	SPK_OFF();
}

u8 SPK_PWM_Toggle(void)
{
//	if(HAL_TIM_STATE_READY == HAL_TIM_PWM_GetState(&htim2))   //pwm stop now
//	{
//		SPK_PWM_ON();
//	}
//	else if(HAL_TIM_STATE_BUSY == HAL_TIM_PWM_GetState(&htim2))
//	{
//		SPK_PWM_OFF();
//	}
//	else
//	{
//		SPK_PWM_OFF();
//	}

	static u8 Prestatus = 0;

	if(!Prestatus)
	{
		Prestatus = 1;
		SPK_PWM_ON();
	}
	else
	{
		Prestatus = 0;
		SPK_PWM_OFF();
//		SPK_OFF();     //add by dozen 
	}
	
	return 0;

}

u8 SPK_PWM_Ctrl(Opt_typedef* Speek, u32 ulSysTime)
{
	if(Speek->ucStatus == None)
	{
		return 1;
	}
	
	if(Speek->ucStatus == ON)
	{
		SPK_PWM_ON();
		Speek->ucStatus = None;
	}
	else if(Speek->ucStatus == OFF)
	{
		SPK_PWM_OFF();
		Speek->ucStatus = None;
	}	
	else if(Speek->ucStatus == Flash)
	{	

		if(Speek->ucTimes > 0)
		{
			if((u32)(ulSysTime - Speek->ulPertick) > Speek->uiPeriod)
			{
				Speek->ulPertick = ulSysTime;

				SPK_PWM_Toggle();

				Speek->ucTimes--;

				if(Speek->ucTimes == 0)
				{
					Speek->ucStatus = None; 	
					Speek->ulPertick = 0;

					HAL_TIM_PWM_DeInit(&htim2);
					PWM_GPIO_Init();			 //pwm gpio set high level, reduce I.
				}
			}
		}		
		
	}	
	
	return 0;
}

u8 Process_PWM_SPK( u32 ulSysTime )
{	
	SPK_PWM_Ctrl(&Speek, ulSysTime);

	return 0;
}


u8 SPK_Ctrl(Opt_typedef* Speek, u32 ulSysTime)
{
		
	if(Speek->ucStatus == None)
	{
		return 1;
	}

	if(Speek->ucStatus == ON)
	{
		SPK_ON();
		Speek->ucStatus = None;
	}
	else if(Speek->ucStatus == OFF)
	{
		SPK_OFF();
		Speek->ucStatus = None;
	}	
	else if(Speek->ucStatus == Flash)
	{	

		if(Speek->ulPertick == 0)
		{
			Speek->ulPertick = ulSysTime;
			Speek->ucTimes--;
			
			SPK_ON();
//			if(Led->uiPeriod >= FLASH_50MS)
//			{
//				
//				OFF_Period = Led->uiPeriod;
//				Led->uiPeriod = ON_Period;
//			}
		}

		if((u32)(ulSysTime - Speek->ulPertick) > Speek->uiPeriod)
		{
			Speek->ulPertick = ulSysTime;

			SPK_TOG();
//			if(Led->uiPeriod == ON_Period)
//			{				
//				Led->uiPeriod = OFF_Period;
//			}
//			else if(Led->uiPeriod == OFF_Period)
//			{				
//				Led->uiPeriod = ON_Period;
//			}
//			else
//				{
//				}
			
			Speek->ucTimes--;

			if(Speek->ucTimes == 0)
			{
				Speek->ucStatus = None;		
				Speek->ulPertick = 0;

			}
		}
		
	}	

	return 0;
}

u8 SPK_Idle(void)
{
	if(Speek.ucStatus == None)
	{
		return 1;
	}
	
	return 0;
}


void SPK_Show(Status_Enum SPK_Status, u8 ucTimes, u16 uiPeriod)
{
	
	Speek.ucStatus = SPK_Status;

	if(Speek.ucStatus == Flash)
	{
		Speek.ucTimes  = (ucTimes << 1);   //*2
		Speek.uiPeriod = uiPeriod;
	}

	Speek.ulPertick = 0;

	SPK_GPIO_Init();    //LF 125K Init

}

u8 Register_Done(void)
{
//	if(SysStatus.ucActived == FALSE)
	if(SysStatus.ucWork_Status == SLEEP)
	{
		return 1;
	}
	
	if(SysStatus.ucJoin_Status == Join_Success)
	{
		if(RFConfig.ucReg_Status != Reg_None)
		{
			return 1;
		}
		
	}
	else
	{
		return 1;
	}

	PRINTF("reg not OK!\r\n");
	
	return 0;
}

u8 Bind_Done(void)
{
//	if(SysStatus.ucActived == FALSE)
	if(SysStatus.ucWork_Status == SLEEP)
	{
		return 1;
	}

	if(SysStatus.ucBind_Status == SN_IDLE_STATUS)
	{
		return 1;
	}
	else
	{
//		PRINTF("Binding!\r\n");
		return 0;
	}
	
}

u8 LF_Done(void)
{
//	static u8 flg = 0;
	
	if(LF.Protocal == LF_IDLE)
	{
//		flg = 0;
		return 1;
	}
	else
	{
//		if(flg == 0)
		{
//			flg = 1;
//			PRINTF("LF　rec!\r\n");
		}
		
		return 0;
	
	}
}


////////// MCU SYSCLK > 16M  or PWM  ////////////
u8 Process_SPK_Event( u32 ulSysTime )
{	
//	SPK_Ctrl(&Speek, ulSysTime);
	static u32 Time = 0;
	static u16 Period = 0;
	static u32 ulPretick = 0;

	
	if(Speek.ucStatus == None)
	{
		return 1;
	}

	if(Time >= Speek.ucTimes)
	{
		Speek.ucStatus = None;
		Speek.ucTimes = 0;
		Speek.uiPeriod = 0;

		Period = 0;
		Time = 0;
			
	}
	else
	{
		u16 Off_Gap = 0;
		
		if(Speek.uiPeriod  == SPK_3S)
		{
			Off_Gap = TIME_S(3);
		}
		else
		{
			Off_Gap = 100 * Speek.uiPeriod;
		}
		
		if(Period > (Speek.uiPeriod*100))
		{
			
			SPK_OFF();

			if((u32)(ulSysTime - ulPretick) > Off_Gap)
			{
				ulPretick = ulSysTime;
				Time++;
				Period = 0;
			}
			
			
		}
		else
		{
			
			SPK_TOG();
			Period++;
			ulPretick = ulSysTime;
		}
		
	}


	return 0;
}


/**
  * @brief  SYSTICK callback.
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
	//SPK_TOG();
//	Process_SPK_Event(HAL_GetTick());
}

void GPS_GPIO_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin  = GPS_VCC_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPS_VCC_PORT, &GPIO_InitStruct); 

	GPS_VCC_OFF();
}

void LED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin  = LED_R_PIN|LED_G_PIN|LED_B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);	

	LED_R_OFF();
	LED_G_OFF();	
	LED_B_OFF();

}


void LED_GPIO_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin  = LED_R_PIN|LED_G_PIN|LED_B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
}

u8 LED_Ctrl(LED_Enum eLed, Opt_typedef* Led, u32 ulSysTime)
{
//	static u8 ON_Period = 1;
//	static u8 OFF_Period = 0;
		
	if(Led->ucStatus == None)
	{
		return 1;
	}

	if(Led->ucStatus == ON)
	{
		if(eLed == Red_Led)
		{
			LED_R_ON();
		}
		else if(eLed == Green_Led)
		{
			LED_G_ON();
		}
		else if(eLed == Blue_Led)
		{
			LED_B_ON();
		}
		else
		{
			
		}
		
		Led->ucStatus = None;
	}
	else if(Led->ucStatus == OFF)
	{
		if(eLed == Red_Led)
		{
			LED_R_OFF();
		}
		else if(eLed == Green_Led)
		{
			LED_G_OFF();
		}
		else if(eLed == Blue_Led)
		{
			LED_B_OFF();
		}
		else
		{
			
		}
		
		Led->ucStatus = None;
	}	
	else if(Led->ucStatus == Flash)
	{	

		if(Led->ulPertick == 0)
		{
			Led->ulPertick = ulSysTime;
			Led->ucTimes--;
			
			if(eLed == Red_Led)
			{
				LED_R_ON();
			}
			else if(eLed == Green_Led)
			{
				LED_G_ON();
			}
			else if(eLed == Blue_Led)
			{
				LED_B_ON();
			}
			else
			{
				
			}

//			if(Led->uiPeriod >= FLASH_50MS)
//			{
//				
//				OFF_Period = Led->uiPeriod;
//				Led->uiPeriod = ON_Period;
//			}
		}

		if((u32)(ulSysTime - Led->ulPertick) > Led->uiPeriod)
		{
			Led->ulPertick = ulSysTime;

			if(eLed == Red_Led)
			{
				LED_R_TOG();
			}
			else if(eLed == Green_Led)
			{
				LED_G_TOG();
			}
			else if(eLed == Blue_Led)
			{
				LED_B_TOG();
			}
			else
			{
				
			}

//			if(Led->uiPeriod == ON_Period)
//			{				
//				Led->uiPeriod = OFF_Period;
//			}
//			else if(Led->uiPeriod == OFF_Period)
//			{				
//				Led->uiPeriod = ON_Period;
//			}
//			else
//				{
//				}
			
			Led->ucTimes--;

			if(Led->ucTimes == 0)
			{
				Led->ucStatus = None;		
				Led->ulPertick = 0;
			}
		}
		
	}	

	return 0;
}

u8 LED_Idle(void)
{
	if((RedLED.ucStatus == None)&&(BlueLED.ucStatus == None)&&(GreenLED.ucStatus == None))
	{
		return 1;
	}
	
	return 0;
}

void LED_Show( LED_Enum Led, Status_Enum Led_Status, u8 ucTimes,  u16 uiPeriod)
{
	if(Led == Red_Led)
	{
		RedLED.ucStatus = Led_Status;

		if(RedLED.ucStatus == Flash)
		{
			RedLED.ucTimes  = (ucTimes << 1);   //*2
			RedLED.uiPeriod = uiPeriod;
		}

		RedLED.ulPertick = 0;
		
	}
	else if(Led == Green_Led)
	{
		GreenLED.ucStatus = Led_Status;

		if(GreenLED.ucStatus == Flash)
		{
			GreenLED.ucTimes  = (ucTimes << 1);   //*2
			GreenLED.uiPeriod = uiPeriod;
		}

		GreenLED.ulPertick = 0;
		
	}
	else if(Led == Blue_Led)
	{
		BlueLED.ucStatus = Led_Status;

		if(BlueLED.ucStatus == Flash)
		{
			BlueLED.ucTimes  = (ucTimes << 1);   //*2
			BlueLED.uiPeriod = uiPeriod;
		}

		BlueLED.ulPertick = 0;
		
	}
	else
		{
		}

	
}

u8 Process_LED_Event( u32 ulSysTime )
{	
	LED_Ctrl(Red_Led, &RedLED, ulSysTime);	
	LED_Ctrl(Green_Led, &GreenLED, ulSysTime);
	LED_Ctrl(Blue_Led, &BlueLED, ulSysTime);

	return 0;
}


u8 Process_HeartBeat( u32 ulSysTime )
{	
	static u32 ulPretick = 0;	

	if((SysStatus.ucWork_Status == SLEEP)||(SysStatus.ucWork_Status == CHG))
	{
		ulPretick = ulSysTime;
		return 1;
	}

	if(AwarePro_NV_JoinFlg() != NET_FLG)
	{
		ulPretick = ulSysTime;
		return 1;
	}

	if(RFConfig.ucReg_Status == Reg_Done)
	{
		if(SysStatus.ucWork_Status == TEST)
		{
			
			if((u32)(ulSysTime - ulPretick) > TIME_S(5))  //TIME_S(30)  TIME_S(RFConfig.uiHB_Gap)
			{
				ulPretick = ulSysTime;
				AwarePro_ZED_Poll_Once();
			}
		}
		else
		{
			if(GetCH() == RF_BIND_CH)
			{
				return 0;
			}

			if(RFConfig.uiHB_Gap == 0)
			{
				return 0;
			}

			if((u32)(ulSysTime - ulPretick) > TIME_S(RFConfig.uiHB_Gap))  //TIME_S(RFConfig.uiHB_Gap)
			{
//				if(Pwr.ucBat_level != 0)
				{
					ulPretick = ulSysTime;
					Dev_Period_Report(ulSysTime);

				}

			}
		}
		
	}
	
	return 0;
}

u8 Process_Reg( u32 ulSysTime )
{
	static u32 ulPretick = 0;	
	static u32 ulPretick2 = 0;
	static u8  flg = 0;
	static u8  Regflg = 0;
//	if(SysStatus.ucActived == FALSE)
//	if(SysStatus.ucWork_Status == SLEEP)
//	{
//		return 1;
//	}
	
	if(SysStatus.ucJoin_Status != Join_Success)
	{
		if((SysStatus.ucJoin_Status == Join_TimeOut)&&(!flg))        //join failing
		{
			flg = 1;
			
			LED_Show(Red_Led, Flash, 1, FLASH_3S);
//			SPK_Show(Flash, 1,SPK_ON_2S);
			
			PRINTF("Join Failed!\r\n");
		}
		else if(SysStatus.ucJoin_Status != Join_TimeOut)
		{
			flg  = 0;
		}
		
		return 0;
	}

	flg  = 0;
	
	if(RFConfig.ucReg_Status)	//Register finish or timeout,failed
	{
		if((RFConfig.ucReg_Status != Reg_Done)&&(!Regflg))
		{
			Regflg = 1;
			LED_Show(Red_Led, Flash, 1, FLASH_3S);
//			SPK_Show(Flash, 1,SPK_ON_2S);
			
			PRINTF("Reg Failed!\r\n");
		}
		else
			{
			}
		
		return 0;
	}

	Regflg = 0;
		
	if((u32)(ulSysTime - ulPretick) > TIME_S(5))
	{
		static u8 cnt = 0;

		ulPretick = ulSysTime;

		if(cnt > 2)
		{
			cnt = 0;
			RFConfig.ucReg_Status = Reg_TimeOut;

		}
		else
		{
			cnt++;
			
			Dev_Register(ulSysTime);
		}
		
	}
	
	if((u32)(ulSysTime - ulPretick2) > TIME_S(2))
	{
		ulPretick2 = ulSysTime;

		AwarePro_ZED_Poll_Once();
	}
	
	return 0;
}

u8 Dev_Register( u32 ulSysTime )
{
	
	static u8 ucSeq = 0;
	u16 CID = CORE_CID;
	u32 Data = 0x00000001;

	if(AwarePro_NV_JoinFlg() != NET_FLG)
	{
		return 1;
	}

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	StaPro_Typedef* DevFunc = (StaPro_Typedef*)(DevPro->ucPayLoad);

	DevPro->ucLen     = PRO_HEAD_LEN + STA_HEAD_LEN + sizeof(u32) + 1;  //add check
	DevPro->ucType    = STA_PRO_TYPE;
	DevPro->ucDev_Seq = ucSeq++;
	DevPro->ucSer_Seq = 0;

	DevFunc->ucEP     = 0;
	osal_revmemcpy(&DevFunc->uiCID, &CID, 2);
	DevFunc->ucCMD    = REGIST_CMD;
	DevFunc->ucID     = D_REQ;
	osal_revmemcpy(&DevFunc->ucData, &Data, sizeof(u32));
	DevFunc->ucData[sizeof(u32)] = Check_Sum(&DevPro->ucDev_Seq, DevPro->ucLen - 2);

	AwarePro_RF_Tx_Process(LoRaTxBuffer, DevPro->ucLen + 1, AwarePro_GetPareAddr(), LORA_ACK_RESP, ulSysTime);

	return 0;
}

u8 GPS_Info_Report(u32 ulSysTime)
{
	
	static u8 seq = 0;
	u8 len = 0;

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);
	GPStoSer_Typedef* GpsSer   = (GPStoSer_Typedef*)(DevFunc->ucData);

	if((AwarePro_NV_JoinFlg() != NET_FLG)
		||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}

	len = sizeof(Gps_Rp.ucLati_NS)
		+ sizeof(Gps_Rp.ucLong_EW)
		+ sizeof(Gps_Rp.ulLati)
		+ sizeof(Gps_Rp.ulLong)
		+ sizeof(Gps_Rp.ulUTC);
	
	DevFunc->ucIndex   = GPS_INFO_S_INDEX;
	GpsSer->ucLati_NS  =  Gps_Rp.ucLati_NS;
	GpsSer->ucLong_EW  =  Gps_Rp.ucLong_EW;	

	/////// big endian report  copy rev ////
	osal_revmemcpy(&GpsSer->ulLati, &Gps_Rp.ulLati, sizeof(Gps_Rp.ulLati));	
	osal_revmemcpy(&GpsSer->ulLong, &Gps_Rp.ulLong, sizeof(Gps_Rp.ulLong));	
	osal_revmemcpy(&GpsSer->ulUTC, &Gps_Rp.ulUTC, sizeof(Gps_Rp.ulUTC));	

//	len += UTCTime_Length;
//	osal_memcpy(DevFunc->ucData, Gps.UTCTime, len);	
//	osal_memcpy((u8*)&DevFunc->ucData[len], Gps.latitude, len + latitude_Length);
//	len += latitude_Length;
//	osal_memcpy((u8*)&DevFunc->ucData[len], Gps.N_S, len + N_S_Length);
//	len += N_S_Length;
//	osal_memcpy((u8*)&DevFunc->ucData[len], Gps.longitude, len + longitude_Length);
//	len += longitude_Length;
//	osal_memcpy((u8*)&DevFunc->ucData[len], Gps.E_W, len + E_W_Length);	
//	len += E_W_Length;

	DevPro_Send((u8*)DevFunc, len + 1, 0, seq++, ulSysTime);


	return 0;
}


u8 Band_SOS_Report( u8 Status, u32 ulSysTime)
{
	
	static u8 seq = 0;

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevFunc->ucIndex   = SOS_S_INDEX;
	DevFunc->ucData[0] = Status;   

//	if(Status == CUT_STATUS)
//	{
//		DevPro_RT_Send((u8*)DevFunc, 2, 0, seq++, ulSysTime);
//	}
//	else
	{	
		DevPro_Send((u8*)DevFunc, 2, 0, seq++, ulSysTime);
	}

	return 0;
}

//u8 Tag_Bind_Report( u8 Status, u32 ulSysTime)
//{
//	
//	static u8 seq = 0;

//	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
//	{
//		return 1;
//	}

//	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
//	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

//	DevFunc->ucIndex   = MATCH_S_INDEX;
//	DevFunc->ucData[0] = Status;   
//	DevPro_Send((u8*)DevFunc, 2, 0, seq++, ulSysTime);

//	return 0;
//}

u8 HB_Record_Report(u32 ulSystime)
{
	static u8 ucSeq = 0;
	u8 len = 0;
	
	Dev_Pro_Typedef* Dev = (Dev_Pro_Typedef*)LoRaTxBuffer;
	MinPro_Typedef* Min = (MinPro_Typedef*)(Dev->ucPayLoad);				

	
	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}

	len = Poll_Record_Num();
	
	Min->ucIndex   = HB_RECORD_INDEX;
	osal_memcpy(Min->ucData, HB.aucHB_Record, len);
			
	DevPro_Send((u8*)Min, len + 1, 0, ucSeq++, ulSystime);

	return 0;
	
}

u8 LF_Report(u8 Status, u32 ulSystime)
{
	static u8 ucSeq = 0;
	
	Dev_Pro_Typedef* Dev = (Dev_Pro_Typedef*)LoRaTxBuffer;
	MinPro_Typedef* Min = (MinPro_Typedef*)(Dev->ucPayLoad);				

	
	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}
	
	Min->ucIndex   = LC_S_INDEX;
	Min->ucData[0] = Status;
	Min->ucData[1] = Loc.scRSSI;
	Min->ucData[2] = Loc.uiID >> 8;			
	Min->ucData[3] = Loc.uiID;
			
	DevPro_Send((u8*)Min, 5, 0, ucSeq++, ulSystime);

	return 0;
	
}

u8 LF_Alarm_Report(u8 Status, u32 ulSystime)
{
	static u8 ucSeq = 0;
	
	Dev_Pro_Typedef* Dev = (Dev_Pro_Typedef*)LoRaTxBuffer;
	MinPro_Typedef* Min = (MinPro_Typedef*)(Dev->ucPayLoad);				

//	if(AwarePro_NV_JoinFlg() != NET_FLG)
//	{
//		return 1;
//	}
	
	Min->ucIndex   = LC_ALARM_S_INDEX;
	Min->ucData[0] = Status;
	Min->ucData[1] = Loc.scRSSI;
	Min->ucData[2] = Loc.uiID >> 8;			
	Min->ucData[3] = Loc.uiID;
			
	DevPro_RT_Send((u8*)Min, 5, 0, ucSeq++, ulSystime);

	return 0;
	
}

u8 Tag_Alarm_Report(u32 ulSystime, u8 Alarm_Type)
{
	static u8 Chg_seq = 0;

	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevFunc->ucIndex   = TAG_ALARM_S_INDEX;
	DevFunc->ucData[0] = Alarm_Type;   //charge/full
	DevPro_Send((u8*)DevFunc, 2, 0, Chg_seq++, ulSystime);

	return 0;
	
}

u8 CHG_Report(u32 ulSystime)
{
	static u8 Chg_seq = 0;

	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevFunc->ucIndex   = CHG_S_INDEX;
	DevFunc->ucData[0] = Pwr.ucBat_Status;   //charge/full
	DevPro_Send((u8*)DevFunc, 2, 0, Chg_seq++, ulSystime);

	return 0;
	
}

u8 BAT_Level_Report(u32 ulSystime)
{
	static u8 seq = 0;

	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevFunc->ucIndex   = BAT_S_INDEX;
	DevFunc->ucData[0] = Pwr.ucBat_level;   //charge/full
	DevPro_Send((u8*)DevFunc, 2, 0, seq++, ulSystime);

	return 0;
	
}


u8 WorkMode_Report(u32 ulSystime)
{
	static u8 seq = 0;

	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}
	
	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevFunc->ucIndex   = STATUS_S_INDEX;
	DevFunc->ucData[0] = SysStatus.ucWork_Status;   
	DevPro_Send((u8*)DevFunc, 2, 0, seq++, ulSystime);

	return 0;
}

u8 Dev_Period_Report( u32 ulSysTime )
{
	static u8 seq = 0;
	
	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}
	
	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;
	MinPro_Typedef* DevFunc = (MinPro_Typedef*)(DevPro->ucPayLoad);

	DevPro->ucLen     = PRO_HEAD_LEN + MIN_HEAD_LEN + 4 + 1;  //add check
	DevPro->ucType    = MIN_PRO_TYPE;
	DevPro->ucDev_Seq = seq++;
	DevPro->ucSer_Seq = 0;

	DevFunc->ucIndex  = HB_S_INDEX;     //change 	
	DevFunc->ucData[0] = SysStatus.ucWork_Status;	
	DevFunc->ucData[1] = SysStatus.ucMove_Status;	
	DevFunc->ucData[2] = HB.ucAve_HR;//SysStatus.ucHeartRate;	        //touch  --> heartRate
	DevFunc->ucData[3] = Pwr.ucBat_level;

//	DevFunc->ucData[3] = Loc.uiID >> 8;	
//	DevFunc->ucData[4] = Loc.uiID;

	
//	osal_revmemcpy(&DevFunc->ucData, &Data, sizeof(u32));
	DevFunc->ucData[4] = Check_Sum(&DevPro->ucDev_Seq, DevPro->ucLen - 2);

	if(AwarePro_NV_JoinFlg() != NET_FLG)
	{
		AwarePro_RF_Tx_RT_Process(LoRaTxBuffer, DevPro->ucLen + 1, ulSysTime);
	}
	else
	{
//		
		AwarePro_RF_Tx_Process(LoRaTxBuffer, DevPro->ucLen + 1, AwarePro_GetPareAddr(), LORA_ACK_RESP, ulSysTime);
	}

	return 0;
}


u8 DevPro_Send( u8* pData, u8 ucLen, u8 ucSerSeq, u8 ucDevSeq, u32 ulSysTime )
{

	if((AwarePro_NV_JoinFlg() != NET_FLG)||(RFConfig.ucReg_Status != Reg_Done))
	{
		return 1;
	}
	
	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;

	DevPro->ucLen     = PRO_HEAD_LEN + ucLen + 1;  //add check
	DevPro->ucType    = MIN_PRO_TYPE;
	DevPro->ucDev_Seq = ucDevSeq;
	DevPro->ucSer_Seq = ucSerSeq;

	osal_memcpy(&DevPro->ucPayLoad, pData, ucLen);
	
	DevPro->ucPayLoad[ucLen] = Check_Sum(&DevPro->ucDev_Seq, DevPro->ucLen - 2);

	AwarePro_RF_Tx_Process(LoRaTxBuffer, DevPro->ucLen + 1, AwarePro_GetPareAddr(), LORA_ACK_RESP, ulSysTime);

	return 0;
}

u8 DevPro_RT_Send( u8* pData, u8 ucLen, u8 ucSerSeq, u8 ucDevSeq, u32 ulSysTime )
{

	Dev_Pro_Typedef* DevPro  = (Dev_Pro_Typedef*)&LoRaTxBuffer;

	DevPro->ucLen     = PRO_HEAD_LEN + ucLen + 1;  //add check
	DevPro->ucType    = MIN_PRO_TYPE;
	DevPro->ucDev_Seq = ucDevSeq;
	DevPro->ucSer_Seq = ucSerSeq;

	osal_memcpy(&DevPro->ucPayLoad, pData, ucLen);
	
	DevPro->ucPayLoad[ucLen] = Check_Sum(&DevPro->ucDev_Seq, DevPro->ucLen - 2);

	AwarePro_RF_Tx_RT_Process(LoRaTxBuffer, DevPro->ucLen + 1, ulSysTime);
//	AwarePro_RF_Tx_Process(LoRaTxBuffer, DevPro->ucLen + 1, AwarePro_GetPareAddr(), 0, ulSysTime);

	return 0;
}


void LF_Config( u32 ulSysTime )
{
	static u8 ucSeq = 0;
	LFProtocolType parameter;
	u8 Temp[5];
	u8 send_flag = 1;
	
	if(LF_ProtocolAnalyzing(LF_RecByte, &parameter) == LF_PROTOCOL_OK)	//LF_RecByte
	{
		if(parameter.operation == OPE_SYS)
		{
			if(parameter.OpeType.ope_sys == SYS_VERSIONS)
			{
				Temp[0] = TAG_Type;
				Temp[1] = HARDVERSION;
				Temp[2] = FirmwareVERSION; 
				Temp[3] = Pwr.ucBat_level;
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_sys == SYS_IN_REST)      //reset
			{
				Event_Flg  = EVN_RST;
				Temp[0] = TAG_Type;
				Temp[1] = HARDVERSION;
				Temp[2] = FirmwareVERSION; 
				Temp[3] = Pwr.ucBat_level;
				Temp[4] = 0xCC; 

				PRINTF("LF RESET!\r\n");
			}		
			else if(parameter.OpeType.ope_sys == SYS_IN_SLEEP)      //sleep
			{
				
				Event_Flg = EVN_TEST;
			}
			else
			{
				send_flag = 0;
			}
		}
		else if(parameter.operation == OPE_SET)
		{
			
			osal_memcpy(Temp, &LF_RecByte[2], 5);
			if(parameter.OpeType.ope_set == SET_MATE_TAG_ID)
			{
//				RFConfig.ulMatch_ID1 = parameter.Config.MateTageID;
			}
			else if(parameter.OpeType.ope_set == SET_TAG_ID)
			{
//				RFConfig.ulTAG_ID = parameter.Config.TageID;
			}
			else if(parameter.OpeType.ope_set == SET_RF_CONFIG)
			{
				if(RFConfig.ucRF_Channel != parameter.Config.RFConfig.RF_FRQ_CHN)
				{
					RFConfig.ucRF_Channel = parameter.Config.RFConfig.RF_FRQ_CHN;

					Event_Flg = EVN_SET_CH;
				}

				if(RFConfig.ucRF_Rate != parameter.Config.RFConfig.RF_DataRate)
				{
					RFConfig.ucRF_Rate  = parameter.Config.RFConfig.RF_DataRate;

					Event_Flg = EVN_SET_TXRATE;
				}
				
				if(RFConfig.ucRF_Pwr != parameter.Config.RFConfig.RF_PA_Select)
				{
					RFConfig.ucRF_Pwr = parameter.Config.RFConfig.RF_PA_Select;

					Event_Flg = EVN_SET_TXPWR;
				}
		
			}
			else if(parameter.OpeType.ope_set == SET_HB_CYCLE)
			{
				RFConfig.uiHB_Gap  = parameter.Config.HB_CycleBasic;

				if((RFConfig.uiHB_Gap < HB_MIN_TIME)
					&&(RFConfig.uiHB_Gap != 0))
				{
					RFConfig.uiHB_Gap = HB_MIN_TIME;
				}
			}
			else if(parameter.OpeType.ope_set == SET_OTHER)
			{
				RFConfig.ucHB_Indicate_EN   = parameter.Config.OtherConfig.HB_Indicate_EN;
//				RFConfig.ucAlertingNumber = parameter.Config.OtherConfig.AlertingNumber;
				RFConfig.ucStatic_Time      = parameter.Config.OtherConfig.MotionlessTime;
				RFConfig.ucSleep_EN         = parameter.Config.OtherConfig.AUTO_Sleep_EN;
			}
			else if(parameter.OpeType.ope_set == SET_LONGPRESS)
			{
				RFConfig.uiMove_Threshold = parameter.Config.LongPressConfig.Move_Threshold;
			}
			else if(parameter.OpeType.ope_set == SET_SAVE)
			{
				if(Write_App_NV())    //ok
				{					
					send_flag = 0;
				}
			}
			else
			{
				send_flag = 0;
			}
		}
		else if(parameter.operation == OPE_GET)
		{
			if(parameter.OpeType.ope_get == GET_MATE_TAG_ID)
			{
//				Temp[0] = (uint8_t)(RFConfig.ulMatch_ID1>>24);
//				Temp[1] = (uint8_t)(RFConfig.ulMatch_ID1>>16);
//				Temp[2] = (uint8_t)(RFConfig.ulMatch_ID1>>8);
//				Temp[3] = (uint8_t)(RFConfig.ulMatch_ID1);
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_get == GET_TAG_ID)
			{
//				Temp[0] = (uint8_t)(RFConfig.ulTAG_ID>>24);
//				Temp[1] = (uint8_t)(RFConfig.ulTAG_ID>>16);
//				Temp[2] = (uint8_t)(RFConfig.ulTAG_ID>>8);
//				Temp[3] = (uint8_t)(RFConfig.ulTAG_ID);
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_get == GET_RF_CONFIG)
			{
				Temp[0] = RFConfig.ucRF_Channel;
				Temp[1] = RFConfig.ucRF_Rate;
				Temp[2] = RFConfig.ucRF_Pwr;
				Temp[3] = 0x55;
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_get == GET_HB_CYCLE)
			{
				Temp[0] = (uint8_t)(RFConfig.uiHB_Gap>> 8);
				Temp[1] = (uint8_t)(RFConfig.uiHB_Gap);
				Temp[2] = 0x55;
				Temp[3] = 0x55;
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_get == GET_OTHER)
			{
				Temp[0] = RFConfig.ucHB_Indicate_EN;
				Temp[1] = 0;//RFConfig.AlertingNumber;
				Temp[2] = RFConfig.ucStatic_Time;
				Temp[3] = RFConfig.ucSleep_EN;
				Temp[4] = 0x55;
			}
			else if(parameter.OpeType.ope_get == GET_LONGPRESS)
			{
				Temp[0] = (uint8_t)(RFConfig.uiMove_Threshold >> 8);
				Temp[1] = (uint8_t)(RFConfig.uiMove_Threshold);
				Temp[2] = 0x55;
				Temp[3] = 0x55;
				Temp[4] = 0x55;
			}
			else
			{
				send_flag = 0;
			}
		}
		else if(parameter.operation == OPE_TEST)
		{
			send_flag = 0;
			
			if(parameter.OpeType.ope_test == TEST_F)
			{
				PackFrame_Data7Byte(LoRaTxBuffer, 0x9f, ucSeq++, &LF_RecByte[2]);
				AwarePro_RF_Tx_Process((uint8_t *)LoRaTxBuffer, 17, AwarePro_GetPareAddr(), 0, ulSysTime);
			}
			
		}
		else
		{
			send_flag = 0;
		}

		if(send_flag)
		{
			if(AwarePro_NV_JoinFlg() != NET_FLG)
			{
				return;
			}
//			LED_Blue_ON();
			PackFrame_Config(LoRaTxBuffer, Temp);
//			test_printf((char *)FrameBuffer, 17);
			AwarePro_RF_Tx_Process((uint8_t *)LoRaTxBuffer, 17, AwarePro_GetPareAddr(), 0, ulSysTime);
//			LED_Blue_OFF();
		}  		
	}
	
}

///////////// record hearbear /////////////
// sucess  0-23
// failed  0xFF

u8 Add_HB_Record(u8 ucHB)
{
	u8 i;

	for(i = 0; i < 24; i++)
	{
		if(HB.aucHB_Record[i] == 0)
		{
			break;
		}
	}

	if(i < 24)
	{		
		HB.aucHB_Record[i] = ucHB;
		return i;
	}

	return 0xFF;
}


u8 Poll_Record_Num(void)
{
	u8 i;

	for(i = 0; i < 24; i++)
	{
		if(HB.aucHB_Record[i] == 0)
		{
			return i;
		}
	}

	return i;
}

u8 Erase_HB_Record(void)
{
	osal_memset(HB.aucHB_Record, 0, sizeof(HB.aucHB_Record));

	return 0;
}

static u8 Check_Sum(u8* Data, u8 Len)
{
	u8 i;
	u8 CheckSum = 0x33;   //add 0x33

	for( i = 0; i < Len; i++ )
	{
		CheckSum += Data[i];
	}
	
	return CheckSum;
}

void RTC_Cali(u32 Freq)
{
	RTC_Sec_Cnt = (u16)((u32)(RTC_Sec_Cnt * Freq) >> 15) + 1;  //32768

	HAL_TIM_IC_DeInit(&htim21);
}



/////////////  433 old protocol  ///////////
void PackFrame_Data7Byte(uint8_t *Frame, uint8_t type, uint8_t cnt, uint8_t *SS) 
{ 
//	uint32_t tag_id; 
	
	Frame[0] = 16;
     
	Frame[1] = 0x5A;
	Frame[2] = 0x80;
         
	Frame[3] = TAG_Type;
  
//	tag_id =  RFConfig.ulTAG_ID;
//	Frame[4] = (uint8_t)(tag_id>>24);//high byte of ID
//	Frame[5] = (uint8_t)(tag_id>>16);//low byte of ID
//	Frame[6] = (uint8_t)(tag_id>>8);//S3
//	Frame[7] = (uint8_t)(tag_id);//S2

	Frame[4] = SN[0];
	Frame[5] = SN[1];
	Frame[6] = SN[2];
	Frame[7] = SN[3];

	Frame[8] = 0x07;//
   
	Frame[9] = type;
   
	Frame[10] = cnt;
	Frame[11] = SS[0];
	Frame[12] = SS[1];
	Frame[13] = SS[2];
	Frame[14] = SS[3];
	Frame[15] = SS[4];
     
	Frame[16] = 0x33+Frame[2]+Frame[3]+Frame[4]+Frame[5] +Frame[6]+Frame[7]+Frame[8]+\
              Frame[9]+Frame[10]+Frame[11] +Frame[12]+Frame[13]+Frame[14]+Frame[15]; 
 
}

void PackFrame_Config(uint8_t *Frame, uint8_t *SS) 
{ 
	static uint8_t cnt = 0;  
//	uint32_t tag_id; 
	
	cnt++;
     
	Frame[0] = 16;
     
	Frame[1] = 0x5A;
     
	Frame[2] = 0x80;
	Frame[3] = TAG_Type;
     
//	tag_id = RFConfig.ulTAG_ID;;
//	Frame[4] = (uint8_t)(tag_id>>24);//high byte of ID
//	Frame[5] = (uint8_t)(tag_id>>16);//low byte of ID
//	Frame[6] = (uint8_t)(tag_id>>8);//S3
//	Frame[7] = (uint8_t)(tag_id);//S2
	Frame[4] = SN[0];
	Frame[5] = SN[1];
	Frame[6] = SN[2];
	Frame[7] = SN[3];

	Frame[8] = 0x07;//
   
	Frame[9] = 0xB9;
   
	Frame[10] = cnt;
	Frame[11] = SS[0];
	Frame[12] = SS[1];
	Frame[13] = SS[2];
	Frame[14] = SS[3];
	Frame[15] = SS[4];
     
	Frame[16] = 0x33+Frame[2]+Frame[3]+Frame[4]+Frame[5]+Frame[6]+Frame[7]+Frame[8]+\
              Frame[9]+Frame[10]+Frame[11]+Frame[12]+Frame[13]+Frame[14]+Frame[15];

}



void *osal_memcpy( void *dst, const void *src, u32 len )
{
	u8 *pDst;
	const u8 *pSrc;

	pSrc = src;
	pDst = dst;

	while ( len-- )
	*pDst++ = *pSrc++;

	return ( pDst );
}

void *osal_revmemcpy( void *dst, const void *src, u32 len )
{
  u8 *pDst;
  const u8 *pSrc;

  pSrc = src;
  pSrc += (len-1);
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc--;

  return ( pDst );
}

void *osal_memset( void *dst, u8 value, u32 len )
{
  u8 *pDst;

  pDst = dst;

  while ( len-- )
    *pDst++ = value;

  return ( pDst );
}



/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1,&ch, 1, 0xffff);
  return ch;
}

