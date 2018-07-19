//#version 00.05


#include "main.h"
#include "stm32l0xx_hal.h"
#include "common.h"
//#include "usart.h"
//#include "gpio.h"
#include "ohr_interface.h"
#include "timeServer.h"

#if (OHR_INTERFACE==OHR_UART)
uint8_t OHR_status = OHR_STATUS_DISABLED;
type_OHRL_output OHRdata;
uint8_t uarterror = 0;
#define BUFFER_LENGTH 48
uint8_t Rxbuf[BUFFER_LENGTH];
uint8_t Rxlen;
static uint8_t omode, ocurrent;

TimerEvent_t timer_ohr;

//app_timer_id_t		timer_ohr;
//static uint8_t missdata = 0;
static uint8_t ohr_event_internal;

uint8_t car[6];

#define OHR_EVENT_INTERNAL_RXREADY  1
#define OHR_EVENT_INTERNAL_TIMEROUT 2

static void ohrCmd_start(uint8_t mode,uint8_t current);
static OHR_EVENT_t Rx_handle(void);
static void timer_ohr_handler(void);//(void * p_context);
//static void uart_disable(uint8_t txd_pin_number, uint8_t rxd_pin_number);
//static void uart_enable(uint8_t txd_pin_number, uint8_t rxd_pin_number);
//static void uart_config(uint8_t txd_pin_number, uint8_t rxd_pin_number);
extern 	UART_HandleTypeDef hlpuart1;
//extern UART_HandleTypeDef huart2;
extern vu8 ucHeart_Status;


void nrf_gpio_cfg_output(uint32_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
  GPIO_InitStruct.Pin = pin;//GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void nrf_gpio_cfg_input(uint32_t pin)
{
		GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
	
  GPIO_InitStruct.Pin = pin;//GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
}

void OHR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = VCC_PIN | VBAT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;//GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	
//	GPIO_InitStruct.Pin = DAVAIL_PIN;	
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);

	VCC_OFF();
	VBAT_OFF();
}

void OHR_GPIO_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = UART2_RX_PIN| UART2_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = DAVAIL_PIN;	
	HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(UART2_RX_PORT, UART2_RX_PIN, GPIO_PIN_RESET); 	
	HAL_GPIO_WritePin(UART2_TX_PORT, UART2_TX_PIN, GPIO_PIN_RESET); 	
	HAL_GPIO_WritePin(DAVAIL_PORT, DAVAIL_PIN, GPIO_PIN_RESET); 

//	VCC_OFF();
//	VBAT_OFF();
}


void Davail_DeInit(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	HAL_GPIO_DeInit(DAVAIL_PORT, DAVAIL_PIN);

}

void Davail_Input(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = DAVAIL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);

}

void UART_RX_Input(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = UART2_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(UART2_RX_PORT, &GPIO_InitStruct);

}

void UART_TX_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = UART2_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(UART2_TX_PORT, &GPIO_InitStruct);
	
//	HAL_GPIO_WritePin(UART2_TX_PORT, UART2_TX_PIN, GPIO_PIN_RESET);	  //add by dozen
}

void UART_TX_Output_Low(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = UART2_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(UART2_TX_PORT, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(UART2_TX_PORT, UART2_TX_PIN, GPIO_PIN_RESET);	  //add by dozen
}

//call before main loop
void OHR_init(void)
{	

// 	uint32_t err_code = app_timer_create(&timer_ohr,APP_TIMER_MODE_SINGLE_SHOT,timer_ohr_handler); //for ohr uart receive timeout
//	APP_ERROR_CHECK(err_code);	

//	    NRF_GPIO->PIN_CNF[HW_OHR_EN_P] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//                                            | (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos)
//                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//	
//	uart_config_ohrm(OHRM_HW_UART_TX_P,OHRM_HW_UART_RX_P);  
//	HW_OHR_OFF();
//	StopOHR();
	
	TimerInit(&timer_ohr, timer_ohr_handler);     ////for ohr uart receive timeout
	OHR_GPIO_Init();
	StopOHR();       
}	


void StartOHR(uint8_t mode,uint8_t current)
{
/*
		nrf_gpio_cfg_input(HW_DAVAIL_P, NRF_GPIO_PIN_NOPULL);  
		nrf_gpio_cfg_input(OHRM_HW_UART_RX_P, NRF_GPIO_PIN_NOPULL);  
		nrf_gpio_cfg_output(OHRM_HW_UART_TX_P);  
		
		HW_OHR_ON();
		NRF_GPIO->OUTCLR = OHRM_HW_UART_TX_M;		//inform ohr module use uart interface
	
		OHR_status = OHR_STATUS_INITIALIZING;
		app_timer_start(timer_ohr,APP_TIMER_TICKS(500, 0),NULL);
		omode = mode;
		ocurrent = current;
		uarterror = 0;
		missdata = 0;
#ifdef OHR_RAWDATA_BLELOG
		memset(pbuf,0,sizeof(pbuf));
		pindex = 0;
#endif	

*/
	
//	OHR_init();
	Davail_Input();
	UART_RX_Input();
	UART_TX_Output();

	VCC_ON();
	VBAT_ON();	
	UART2_TX_LOW();

	OHR_status = OHR_STATUS_INITIALIZING;
	TimerSetValue(&timer_ohr, 500);		
	TimerStart(&timer_ohr );
	omode = mode;
	ocurrent = current;
	uarterror = 0;
	
}

void StopOHR(void)
{
/*
	HW_OHR_OFF();
	uart_disable(OHRM_HW_UART_TX_P,OHRM_HW_UART_RX_P); 
	nrf_gpio_cfg_output(HW_DAVAIL_P);  
	nrf_gpio_cfg_output(OHRM_HW_UART_TX_P);  
	nrf_gpio_cfg_output(OHRM_HW_UART_RX_P); 
	NRF_GPIO->OUTCLR = OHRM_HW_UART_TX_P;
	NRF_GPIO->OUTCLR = OHRM_HW_UART_RX_P;
	NRF_GPIO->OUTCLR = HW_DAVAIL_P;
	OHR_status = OHR_STATUS_DISABLED;
	ohr_mode = 0;
*/

	VCC_OFF();
	VBAT_OFF();
	Davail_DeInit();
	
//	HAL_UART_DeInit(&hlpuart1); 
	HAL_UART_MspDeInit(&hlpuart1); 
//	Davail_DeInit();

//	OHR_GPIO_DeInit();	
	OHR_status = OHR_STATUS_DISABLED;
}

void nrf_gpio_cfg_sense_input()
{		
	GPIO_InitTypeDef GPIO_InitStruct;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = DAVAIL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;//GPIO_PULLUP;
  HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	
//	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static uint8_t  OHR_startup(void)
{

/*	
	if((NRF_GPIO->IN & HW_DAVAIL_M)==0)
	{
		StopOHR();
		return(OHR_FAIL);
	}
	
	uart_enable(OHRM_HW_UART_TX_P,OHRM_HW_UART_RX_P); 
	OHR_status = OHR_STATUS_WAITFORRESPONSE;
	nrf_gpio_cfg_sense_input(HW_DAVAIL_P,NRF_GPIO_PIN_NOPULL,NRF_GPIO_PIN_SENSE_LOW);
	ohrCmd_start(omode, ocurrent);
	app_timer_start(timer_ohr,APP_TIMER_TICKS(100, 0),NULL);
	
	return(OHR_SUCC);


*/

	if(!READ_DAVAIL())
	{
		StopOHR();
		PRINTF("Low level!\r\n");
		return(OHR_FAIL);
	}
	
	HAL_UART_MspInit(&hlpuart1);
//	MX_LPUART1_UART_Init();

	OHR_status = OHR_STATUS_WAITFORRESPONSE;
	nrf_gpio_cfg_sense_input();
	ohrCmd_start(omode, ocurrent);
	TimerSetValue(&timer_ohr, 3000); 	
	TimerStart(&timer_ohr );
	PRINTF("OHR start up!\r\n");
	
	return(OHR_SUCC);
}

// handle of rx handshake
void OHR_davail_handler(void)
{
	MX_USART2_UART_Init();
	HAL_UART_MspInit(&hlpuart1);
//		uart_enable(HW_UART_TX_P,HW_UART_RX_P); 
}

//put it at main loop
OHR_EVENT_t OHR_event_handler(void)
{
	if((ohr_event_internal & OHR_EVENT_INTERNAL_TIMEROUT)==OHR_EVENT_INTERNAL_TIMEROUT) 
	{
		ohr_event_internal &= ~OHR_EVENT_INTERNAL_TIMEROUT;
			
		switch(OHR_status)
		{
			case OHR_STATUS_INITIALIZING:
			if(OHR_startup() == OHR_SUCC)
			{
				//ohr started success
				return OHR_EVENT_START_WAITFORRESPONSE;
			}
			else
			{
					//ohr started fail
				StopOHR();
				return OHR_EVENT_START_FAIL;
			}

			case OHR_STATUS_WAITFORRESPONSE:			
			case OHR_STATUS_ENABLED:
//				if(ohr_mode == 0)StopOHR();
				PRINTF("start resp timeout!\r\n");
//				OHR_startup();

				ucHeart_Status = 0;

//				StopOHR();
				
				return OHR_EVENT_START_FAIL;
			
			default:
				break;
		}


	}
//	else if((ohr_event_internal & OHR_EVENT_INTERNAL_RXREADY)==OHR_EVENT_INTERNAL_RXREADY) 
//	{
//		ohr_event_internal &= ~OHR_EVENT_INTERNAL_RXREADY;
//		
//		uint8_t resp = Rx_handle();
//		
//		if(resp == OHR_EVENT_START_FAIL)
//		{
//			StopOHR();
//		}
//		
//		return(resp);
//	}	
	
	return OHR_NO_EVENT;
}

/*
OHR_EVENT_t OHR_event_handler(void)
{		
		switch(OHR_status)
		{
			case OHR_STATUS_INITIALIZING:
				if(OHR_startup()==OHR_SUCC)
				{
					//ohr started success
					return OHR_EVENT_START_WAITFORRESPONSE;
				}
				else
				{
						//ohr started fail
					StopOHR();

					return OHR_EVENT_START_FAIL;
				}

			case OHR_STATUS_WAITFORRESPONSE:			
			case OHR_STATUS_ENABLED:
//				StopOHR();			
				return OHR_EVENT_START_FAIL;
			
			default:
				break;
		}
	return OHR_NO_EVENT;
}
*/

static void ohr_timerout_hander(void)
{
	if((ohr_event_internal & OHR_EVENT_INTERNAL_RXREADY) == OHR_EVENT_INTERNAL_RXREADY)	
	{		
		ohr_event_internal &= ~OHR_EVENT_INTERNAL_TIMEROUT;
	}
	else
	{
		ohr_event_internal |= OHR_EVENT_INTERNAL_TIMEROUT;
	}
}

static void timer_ohr_handler(void)//(void * p_context)//ohr uart time out
{
		ohr_timerout_hander();
}


void ohrCmd_start(uint8_t mode,uint8_t current)
{//0xFE,0x1,0x2, <PAR1>,<PAR2><CHKSUM>
	car[0]=0xFE;
	car[1]=0x01;
	car[2]=0x02;
	car[3]=mode;
	car[4]=current;
	car[5]=~(mode+current+3)+1;
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)car, 6);

	LpUart1_Send(car, 6);
}


static OHR_EVENT_t Rx_handle(void) 
{

	uint8_t i,data,chksum1;	
	uint32_t delaytime = 3000; // seconds timeout for normal mode

	TimerStop(&timer_ohr);	
		
	#if 1
	if(omode&1)
	{
		if(OHR_status == OHR_STATUS_ENABLED)
			delaytime = 200;  //200 ms timeout for next data at raw data mode
		else
			delaytime = 500;	//500 ms timeout at start up
	}
	#endif		
	
	TimerSetValue(&timer_ohr, delaytime);		
	TimerStart(&timer_ohr );		
						
	if(OHR_status != OHR_STATUS_ENABLED && OHR_status!=255)
	{
		if(Rxbuf[1]==0x80)
		{
			#ifdef OHR_RAWDATA_BLELOG
			OHRdata.sensortype = Rxbuf[17] - '0';	
			#endif
			
			uint8_t *adr1,*adr2;
			adr1 = (uint8_t *)OHRdata.HWVERSION;
			adr2 = &Rxbuf[4];
			for(i=0;i<3;i++) 
			{
				if(*adr2=='-')break;
				*adr1++= *adr2++;
			}				
			* adr1 = 0;
			adr2++;
			adr1 = (uint8_t *)OHRdata.FWVERSION;
			for(i=0;i<7;i++) 
			{
				if(*adr2=='-')break;
				*adr1++= *adr2++;
			}				
			* adr1 = 0;	
			adr2++;
			adr1 = (uint8_t *)OHRdata.OHRLIBVERSION;
			for(i=0;i<11;i++) 
			{
				if(*adr2=='-')break;
				*adr1++= *adr2++;
			}				
			* adr1 = 0;		
			adr2++;
			adr1 = (uint8_t *)OHRdata.OHRSTEPVERSION;
			for(i=0;i<11;i++) 
			{
				if(*adr2=='-')break;
				*adr1++= *adr2++;
			}				
			* adr1 = 0;			
			
			OHR_status = OHR_STATUS_ENABLED;
			return OHR_EVENT_START_SUCCESS;	
		}
		else 
			return OHR_EVENT_START_FAIL;
			
	}
	switch(Rxbuf[1])
	{
		case 0x50:
			OHRdata.hr = Rxbuf[3];
			OHRdata.quality = Rxbuf[4];
			OHRdata.onwrist = Rxbuf[5];	
			OHRdata.current = Rxbuf[6];
			break;			
		case 0x51:
			OHRdata.hr = Rxbuf[3];
			OHRdata.quality = Rxbuf[4];
			OHRdata.onwrist = Rxbuf[5];	
			OHRdata.current = Rxbuf[6];		
			
		  #ifdef OHR_RAWDATA_BLELOG
		    OHRdata.pulse = Rxbuf[9] + Rxbuf[10]*0x100 + Rxbuf[11]*0x10000;
			OHRdata.amp =   Rxbuf[12] + Rxbuf[13]*0x100+ Rxbuf[14]*0x10000;
			OHRdata.pulseamp = Rxbuf[26];
			OHRdata.PedoSleep_flag = Rxbuf[25];
			OHRdata.current = Rxbuf[23];
			OHRdata.x = (Rxbuf[15] + Rxbuf[16]*0x100)/32;
			OHRdata.y = (Rxbuf[17] + Rxbuf[18]*0x100)/32;
			OHRdata.z = (Rxbuf[18] + Rxbuf[20]*0x100)/32;
			if(Rxbuf[7] + Rxbuf[8]*0x100>10000)
			{
				__NOP();
			}
			ble_send_ohrrawdata(&Rxbuf[7],20);
		  	for(i=0;i<15;i++)pbuf[i]=pbuf[i+1];
		  	pbuf[15]=OHRdata.pulse;
			if(pindex<16)++pindex;

			#endif	
			
		default:
			break;
	}
	
	if(OHRdata.onwrist) OHRdata.hr = 0;
	
	Rxlen = 0;
	
	return OHR_EVENT_DATA_READY;
	
}





/** @brief Function for handling the UART Interrupt. 
 */
//void USART1_IRQHandler(void)
//{
//	static uint8_t chksum = 0;
//	if ( NRF_UART0->EVENTS_RXDRDY ) {
//		NRF_UART0->EVENTS_RXDRDY = 0;
//		uint8_t data  = NRF_UART0->RXD;
//	
//		Rxbuf[Rxlen] = data; //command	
//		if(data == 0xFD && Rxlen==0)
//		{
//			Rxlen = 1;
//			chksum = 0;
//		}
//		else
//		 if(Rxlen>0)
//		 {
//		 	chksum+=data;
//			if(++Rxlen>=BUFFER_LENGTH)
//			{
//				Rxlen = 0;
//				uarterror ++;
//			}
//			
//			if(Rxlen>3)
//			{
//				if(Rxlen==Rxbuf[2]+4)
//				{
//					if(chksum==0)
//					{
//						ohr_event_internal |= OHR_EVENT_INTERNAL_RXREADY;	
//						uart_disable(HW_UART_TX_P,HW_UART_RX_P); 
//						uint32_t delaytime = 3000;//3 seconds timeout for normal mode
//						if(omode&1)
//						{
//							if(OHR_status == OHR_STATUS_ENABLED)
//								delaytime = 200;  //200 ms timeout for next data at raw data mode
//							else
//								delaytime = 500;	//500 ms timeout at start up
//						}
//						app_timer_stop(timer_ohr);
//						app_timer_start(timer_ohr,APP_TIMER_TICKS(delaytime, 0),NULL);
//					}
//					else 
//					{
//						Rxlen = 0;
//						++uarterror;
//						uart_disable(HW_UART_TX_P,HW_UART_RX_P); 						
//					}
//				}
//			}
//		 }
//	 }
//	
//	 if ( NRF_UART0->EVENTS_ERROR ) 
//		{
//				uint32_t       error_source;
//				NRF_UART0->EVENTS_ERROR = 0;
//				error_source        = NRF_UART0->ERRORSRC;
//				NRF_UART0->ERRORSRC = error_source;			
//				Rxlen = 0;	
//				uart_disable(HW_UART_TX_P,HW_UART_RX_P); 
//		}
//	
//}
#endif
/** 
 * @}
 */
