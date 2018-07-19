//#version 00.05
#include "ohr_interface.h"
#if (OHR_INTERFACE==OHR_I2C)
//#include "nrf_delay.h"
//#include "nrf.h"
//#include "nrf_gpio.h"
//#include "app_timer.h"
//#include "app_error.h"
//#include "app_util_platform.h"

//#include "i2c_driver.h"
//#include <stdint.h>
#include "stm32l0xx.h"
#include "Aware_Config.h"

//#include "i2c.h"         //add by dozen
#include "timeServer.h"

#define OHR_I2C_SLAVE_ADDR  0x20//0x10

uint8_t OHR_status = OHR_STATUS_DISABLED;
type_OHRL_output OHRdata;
uint8_t uarterror = 0;
#define BUFFER_LENGTH 48
uint8_t Rxbuf[BUFFER_LENGTH];
uint8_t Rxlen;
static uint8_t omode, ocurrent;
//app_timer_id_t		timer_ohr;
TimerEvent_t timer_ohr;

extern I2C_HandleTypeDef hi2c1;
extern vu8 ucHeart_Status;

static uint8_t ohr_event_internal;
#define OHR_EVENT_INTERNAL_RXREADY  1//BIT_0
#define OHR_EVENT_INTERNAL_TIMEROUT 2//BIT_1

static void ohrCmd_start(uint8_t mode,uint8_t current);
static OHR_EVENT_t Rx_handle(void);
static void timer_ohr_handler(void);//(void * p_context);

void nrf_gpio_cfg_sense_input(void);

void OHR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = VCC_PIN | VBAT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  //GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	
//	GPIO_InitStruct.Pin = DAVAIL_PIN;	
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);

	VCC_OFF();
	VBAT_OFF();

//	VBAT_ON();
//	VCC_ON();
}


//call before main loop
void OHR_init(void)
{	
//		
//	uint32_t err_code = app_timer_create(&timer_ohr,APP_TIMER_MODE_SINGLE_SHOT,timer_ohr_handler); //for ohr uart receive timeout
//	APP_ERROR_CHECK(err_code);	

//	    NRF_GPIO->PIN_CNF[HW_OHR_EN_P] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
//                                            | (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos)
//                                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
//                                            | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//                                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

//	HW_OHR_OFF();
//  I2CDeInit();

	
	TimerInit(&timer_ohr, timer_ohr_handler);     ////for ohr uart receive timeout

	OHR_GPIO_Init();
	
//	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_MspDeInit(&hi2c1);
//	I2CDeInit();
//	StopOHR();

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

void Davail_DeInit(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	HAL_GPIO_DeInit(DAVAIL_PORT, DAVAIL_PIN);


/*	
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();

	
//	HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);     //RF DIO INT GPIO can not disable
	
	GPIO_InitStruct.Pin = DAVAIL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DAVAIL_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(DAVAIL_PORT, DAVAIL_PIN, GPIO_PIN_RESET);
*/	
	

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
	
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);         //RF DIO already on int
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}


//call where want ohr start
/***********************************************************
input of StartOHR
mode current
0x0		0									normal measurement without raw data output
0x1		0									normal measurement with raw data output.
0x2		0x0 to 0x3F				start measurement with fixed current without raw data output
0x3		0x0 to 0x3F				start measurement with fixed current with raw data output
*/   
void StartOHR(uint8_t mode,uint8_t current)
{
//  nrf_gpio_cfg_sense_input(HW_DAVAIL_P,NRF_GPIO_PIN_NOPULL,NRF_GPIO_PIN_SENSE_LOW);
//  I2CInit();		
//	HW_OHR_ON();	

//	OHR_status = OHR_STATUS_INITIALIZING;
//	app_timer_start(timer_ohr,APP_TIMER_TICKS(500, 0),NULL);//wait 500ms for startup
//	omode = mode;
//	ocurrent = current;
//	uarterror = 0;
			
//	Davail_Input();	
	nrf_gpio_cfg_sense_input();
	
//	HAL_I2C_Init(&hi2c2);
	
	HAL_I2C_MspInit(&hi2c1);
//	I2CInit();
	VBAT_ON();
	VCC_ON();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	OHR_status = OHR_STATUS_INITIALIZING;

	TimerSetValue(&timer_ohr, 500);		
	TimerStart(&timer_ohr );
	
	omode = mode;
	ocurrent = current;
	uarterror = 0;

}
//call where want ohr stop
void StopOHR(void)
{	
//	HW_OHR_OFF();
//	I2CDeInit();
//	nrf_gpio_cfg_output(HW_DAVAIL_P);  
//	NRF_GPIO->OUTCLR = HW_DAVAIL_P;
//	OHR_status = OHR_STATUS_DISABLED;

//	uint8_t ret = 0xff;

//	I2CDeInit();
//	__HAL_I2C_RESET_HANDLE_STATE(&hi2c2);
	HAL_I2C_MspDeInit(&hi2c1);
//	HAL_I2C_DeInit(&hi2c2);
	Davail_DeInit();
		
	VBAT_OFF();
	VCC_OFF();


	OHR_status = OHR_STATUS_DISABLED;
}


static uint8_t  OHR_startup(void)
{

//	if((NRF_GPIO->IN & HW_DAVAIL_M)==0)
//	{
//		return(OHR_FAIL);
//	}
//	
//	app_timer_start(timer_ohr,APP_TIMER_TICKS(20, 0),NULL);
//	
//	OHR_status = OHR_STATUS_WAITFORRESPONSE; 
//	ohrCmd_start(omode, ocurrent);		
//		
//	return(OHR_SUCC);

	
	if(!READ_DAVAIL())
	{
//		StopOHR();
		PRINTF("Low level!\r\n");
		return(OHR_FAIL);
	}
	
//	nrf_gpio_cfg_sense_input();
	
	TimerSetValue(&timer_ohr, 200);		
	TimerStart(&timer_ohr );
	
	OHR_status = OHR_STATUS_WAITFORRESPONSE; 
	ohrCmd_start(omode, ocurrent);		
		
	return(OHR_SUCC);

	

}

// handle of rx handshake
//DAVAIL interrupt handle, put this at DAVAIL io interrupt
void OHR_davail_handler(void)
{
  //read data out
	uint8_t data,chksum=0;	
  	uint8_t ret = 0xFF;

	//BUFFER_LENGTH
//	ret = I2CDevice_Read(OHR_I2C_SLAVE_ADDR, 0, Rxbuf, 8);//read data out	
//	ret = (uint8_t)HAL_I2C_Master_Receive(&hi2c1, OHR_I2C_SLAVE_ADDR, Rxbuf, BUFFER_LENGTH, 1000);
	
	ret = (uint8_t)HAL_I2C_Mem_Read(&hi2c1, OHR_I2C_SLAVE_ADDR, 0, I2C_MEMADD_SIZE_8BIT, Rxbuf, BUFFER_LENGTH, 50);

	if(ret)
	{
		PRINTF("R failed:%d!\r\n",ret);

//		ucHeart_Status = 0;
		
		return;
	}
	else
	{
		
//		PRINTF("R Success!\r\n");
	}
	
	Rxlen = 0;
	for(int i=0; i<BUFFER_LENGTH; i++)
	{
		data = Rxbuf[Rxlen]; //command	

		if(data == 0xFD && Rxlen==0)
		{
			Rxlen = 1;
			chksum = 0;
		}
		else
		{
			if(Rxlen > 0)
			{
				chksum+=data;

				if(++Rxlen>3)
				{
					if(Rxlen == Rxbuf[2]+4)
					{
						if(chksum==0)
						{							
//							uint32_t delaytime = 100; //seconds timeout for normal mode

							if(omode&1)
							{
//								if(OHR_status == OHR_STATUS_ENABLED)
//									delaytime = 200;  //200 ms timeout for next data at raw data mode
//								else
///									delaytime = 500;	//500 ms timeout at start up
							}
//							app_timer_stop(timer_ohr);
//							app_timer_start(timer_ohr,APP_TIMER_TICKS(delaytime, 0),NULL);
							
							TimerStop(&timer_ohr);
							ohr_event_internal |= OHR_EVENT_INTERNAL_RXREADY;
//							TimerSetValue(&timer_ohr, delaytime);		
//							TimerStart(&timer_ohr );

						}
						else 
						{
							Rxlen = 0;
						}
						
						break;
					}
				}
			}
		 	else
		 		{
		 		}
		}
			
	
	 }
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
				if(OHR_startup()==OHR_SUCC)
				{
				
//					PRINTF("startup Succecc!\r\n");
					//ohr started success
					return OHR_EVENT_START_WAITFORRESPONSE;
				}
				else
				{
						//ohr started fail
					PRINTF("startup failed!\r\n");
					StopOHR();
					return OHR_EVENT_START_FAIL;
				}

			case OHR_STATUS_WAITFORRESPONSE:			
//			case OHR_STATUS_ENABLED:

				PRINTF("start resp timeout!\r\n");
				
//				ohrCmd_start(omode, ocurrent);
				
//				OHR_status = OHR_STATUS_DISABLED;
				ucHeart_Status = 0;

				StopOHR();
				return OHR_EVENT_START_FAIL;
			
			default:
				break;
		}


	}
	else if((ohr_event_internal & OHR_EVENT_INTERNAL_RXREADY)==OHR_EVENT_INTERNAL_RXREADY) 
	{
		
		OHR_EVENT_t resp = Rx_handle();
		ohr_event_internal &= ~OHR_EVENT_INTERNAL_RXREADY;
		
		if(resp == OHR_EVENT_START_FAIL)
		{
			PRINTF("Rx error!\r\n");
						
			OHR_status = OHR_STATUS_ENABLED;
		//				StopOHR();
		}

		// for test///
		if(OHRdata.hr != 0)
		{
			PRINTF("HR:%d, SNR:%d, On:%d, Current:%d\r\n", 
			  		OHRdata.hr, OHRdata.quality, OHRdata.onwrist, OHRdata.current);
		}
		
//		ucHeart_Status = 0;
		
		return(resp);
	}	
	
	return OHR_NO_EVENT;
}

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

static void ohrCmd_start(uint8_t mode,uint8_t current)
{//0xFE,0x1,0x2, <PAR1>,<PAR2><CHKSUM>
	
  uint8_t buf[6];
  uint8_t ret = 0xff;
  
	buf[0]=0xFE;
	buf[1] =0x1;
	buf[2] = 2;
	buf[3] = mode;
	buf[4] = current;	
	buf[5] = ~(current+3+mode)+1;
//	ret = I2CDevice_Write(OHR_I2C_SLAVE_ADDR,0,buf,6);
//	ret = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, OHR_I2C_SLAVE_ADDR, buf, 6, 1000);
	ret = (uint8_t)HAL_I2C_Mem_Write(&hi2c1, OHR_I2C_SLAVE_ADDR, 0, I2C_MEMADD_SIZE_8BIT, buf, 6, 5000);
	if(ret)
	{
		PRINTF("IIC send failed:%d!\r\n", ret);
	}
	else
	{
		
		PRINTF("IIC send Success!\r\n");
	}
}


static OHR_EVENT_t Rx_handle(void) 
{
	uint8_t i;//data,chksum1;	

	if(OHR_status != OHR_STATUS_ENABLED)
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
		  OHRdata.pulse = Rxbuf[8] + Rxbuf[9]*0x100 + Rxbuf[10]*0x10000;
			OHRdata.amp =   Rxbuf[11] + Rxbuf[12]*0x100+ Rxbuf[13]*0x10000;
			OHRdata.pulseamp = Rxbuf[25];
			OHRdata.PedoSleep_flag = Rxbuf[24];
			OHRdata.current = Rxbuf[22];
			OHRdata.x = (Rxbuf[14] + Rxbuf[15]*0x100)/16;
			OHRdata.y = (Rxbuf[16] + Rxbuf[17]*0x100)/16;
			OHRdata.z = (Rxbuf[17] + Rxbuf[19]*0x100)/16;					
			ble_send_ohrrawdata(&Rxbuf[6],20);
			#endif	
		default:
			break;
	}

	ucHeart_Status = 0;
	
  	return OHR_EVENT_DATA_READY;
}

#endif
/** 
 * @}
 */
