//#version 00.05

#ifndef OHR_INTERFACE_H
#define OHR_INTERFACE_H

#include <stdint.h>
#define OHR_I2C  1
#define OHR_UART 0
#define OHR_INTERFACE OHR_I2C//OHR_UART

//#define OHR_RAWDATA_BLELOG  

#define OHR_SUCC   0
#define OHR_FAIL   0XFF

#define VCC_PORT                        GPIOB
#define VCC_PIN					     	GPIO_PIN_9
#define VCC_OFF()    HAL_GPIO_WritePin(VCC_PORT, VCC_PIN, GPIO_PIN_SET)
#define VCC_ON()     HAL_GPIO_WritePin(VCC_PORT, VCC_PIN, GPIO_PIN_RESET) 

#define VBAT_PORT                       GPIOB
#define VBAT_PIN					    GPIO_PIN_8
#define VBAT_OFF()    HAL_GPIO_WritePin(VBAT_PORT, VBAT_PIN, GPIO_PIN_SET)
#define VBAT_ON()     HAL_GPIO_WritePin(VBAT_PORT, VBAT_PIN, GPIO_PIN_RESET) 

#define DAVAIL_PORT                     GPIOB
#define DAVAIL_PIN					    GPIO_PIN_3
#define READ_DAVAIL()     HAL_GPIO_ReadPin(DAVAIL_PORT, DAVAIL_PIN)

#define UART2_RX_PORT                   GPIOB//GPIOA
#define UART2_RX_PIN					GPIO_PIN_11//GPIO_PIN_3

#define UART2_TX_PORT                   GPIOB//GPIOA
#define UART2_TX_PIN					GPIO_PIN_10//GPIO_PIN_2
#define UART2_TX_LOW()    HAL_GPIO_WritePin(UART2_TX_PORT, UART2_TX_PIN, GPIO_PIN_RESET) 


#define HW_DAVAIL_P						GPIO_PIN_5

#define HW_UART_RX_P						GPIO_PIN_10
#define HW_UART_TX_P						GPIO_PIN_9


#define HW_OHR_EN_P             GPIO_PIN_4                     

#define HW_OHR_OFF()    HAL_GPIO_WritePin(GPIOA, HW_OHR_EN_P, GPIO_PIN_RESET)
#define HW_OHR_ON()     HAL_GPIO_WritePin(GPIOA, HW_OHR_EN_P, GPIO_PIN_SET)       
//#define READ_DAVAIL()   HAL_GPIO_ReadPin(GPIOA, HW_DAVAIL_P)

typedef struct {
	uint8_t hr;
	int8_t quality;				// -30 to 30, the bigger the beter 
	uint8_t onwrist;      //0 - on wrist, >0  - off wrist
	uint8_t current;      //led current percentage 0 to 100%
	char HWVERSION[4];     //ohr module hardware version
	char FWVERSION[8];     //ohr module firmware version 
	char OHRLIBVERSION[12]; 
	char OHRSTEPVERSION[12]; 
#ifdef OHR_RAWDATA_BLELOG	
	uint8_t pulseamp;     //static pulse amplitude
	int32_t pulse;      //ppg average 
	int32_t amp;
	uint8_t PedoSleep_flag;	
	uint8_t sensortype;	
	int8_t x;
	int8_t y;
	int8_t z;		
#endif	
} type_OHRL_output;

extern type_OHRL_output OHRdata;  //data from OHR module


typedef enum
{
     OHR_STATUS_DISABLED = 0,    
		 OHR_STATUS_INITIALIZING,    
		 OHR_STATUS_WAITFORRESPONSE,	
		 OHR_STATUS_ENABLED,	
}OHR_STATUS_t;

extern uint8_t OHR_status; 

#ifdef OHR_RAWDATA_BLELOG
extern uint8_t uarterror;  //count for uart error
extern uint8_t misscounts;
#endif

void OHR_init(void);  //call before main loop
	
/***********************************************************
input of StartOHR
mode current
0x0		0									normal measurement without raw data output
0x1		0									normal measurement with raw data output.
0x2		0x0 to 0x3F				start measurement with fixed current without raw data output
0x3		0x0 to 0x3F				start measurement with fixed current with raw data output
*/

void OHR_GPIO_Init(void);

void Davail_DeInit(void);

void Davail_Input(void);

void UART_RX_Input(void);

void UART_TX_Output(void);



void StartOHR(uint8_t mode,uint8_t current);  //start ohr

void StopOHR(void);   //stop ohr 

/*    how to start/stop ohr module
			  switch(OHR_status)
				{					
					case OHR_STATUS_DISABLED:
						StartOHR(0,0);
						break;
					case OHR_STATUS_ENABLED:
						StopOHR();
						break;
					default:
						break;					
				}
*/

void OHR_davail_handler(void);  //DAVAIL interrupt handle, put this at DAVAIL io interrupt

typedef enum
{
     OHR_NO_EVENT = 0,    
		 OHR_EVENT_START_FAIL,    //ohr start fail
		 OHR_EVENT_START_WAITFORRESPONSE,//ohr start but wait for response
		 OHR_EVENT_START_SUCCESS,	// ohr start success 
     OHR_EVENT_FAIL_RX,       //rx timeout    
		 OHR_EVENT_DATA_READY,    //ohr data ready 
}OHR_EVENT_t;

OHR_EVENT_t OHR_event_handler(void);   //OHR event handle, put it at main loop 
#ifdef OHR_RAWDATA_BLELOG
extern  int32_t pbuf[16];
extern  uint8_t pindex;
#endif
#endif
