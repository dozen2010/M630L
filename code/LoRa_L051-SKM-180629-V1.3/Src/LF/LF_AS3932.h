#ifndef __LF_AS3932_H__
#define __LF_AS3932_H__

//#include "gpio.h"
#include "stm32l0xx.h"

#include "ccRFID_CRC8.h"

#define LF_IDLE        0x00
#define LF_START_REC   0x01
#define LF_REC         0x02
#define LF_CONF_STX    0x33
#define LF_ERROR       0x5A
//#define LF_LOCATION    0xAA
//#define LF_CONFIG      0xCC
#define LF_SINGLE 	    0xAA
#define LF_MULTIPLE     0xBB
#define LF_CONFIG      0xCC

//#define LF_Single 	    0xAA
//#define LF_Multiple     0xBB
//#define LF_Config    	0xCC

typedef struct
{
	uint8_t Protocal;			//LF协议类型:LF_Single,LF_Multiple,LF_Config
	uint8_t DecodeErrCnt;		//LF协议解析错误计数
	uint8_t RSSIx;				//X轴信号强度
	uint8_t RSSIy;				//Y轴信号强度
	union
	{
		uint16_t SingleAddr;	//11位定位地址
		struct
		{
			uint8_t Type;		//16位定位地址类型
			uint16_t Addr;		//16位定位地址
		}MultipleAddr;			
		uint8_t *ConfigData;	//LF配置数据,6字节
	}Data;
}LF_TypedDef;

//Mode Select
#define WriteMode 		    0X00  //0B00000000
#define ReadMode  		    0X40  //0B01000000
#define NotAllowdeMode  	0X80  //0B10000000
#define DirectCommandMode  	0XC0  //0B11000000
//Register Address
#define R0 			0X00  //0B00000000
#define R1 			0X01  //0B00000001
#define R2 			0X02  //0B00000010
#define R3 			0X03  //0B00000011
#define R4 			0X04  //0B00000100
#define R5 			0X05  //0B00000101
#define R6 			0X06  //0B00000110
#define R7 			0X07  //0B00000111
#define R8 			0X08  //0B00001000
#define R9 			0X09  //0B00001001
#define R10 		0X0A  //0B00001010
#define R11 		0X0B  //0B00001011
#define R12 		0X0C  //0B00001100
#define R13 		0X0D  //0B00001101
#define R14 		0X0e  
#define R15		     0X0f 
#define R16 		0X10 
#define R17 		0X11 
#define R18 		0X12 
#define R19 		0X13 
//Command
#define ClearWake 		0X00  //0B00000000
#define ResetAGC		0X01  //0B00000001
#define TrimOSC 		0X02  //0B00000010
#define ClearFalse 		0X03  //0B00000011
#define PresetDefault 	0X04  //0B00000100
#define CenterOSC 		0X05  //0B00000101



/***********************数据或函数声明**************************/
extern LF_TypedDef LF;

#define AS_CS_PORT       GPIOB
#define AS_CS_PIN        GPIO_PIN_13
#define AS_CS_INIT
#define AS_CS_1          HAL_GPIO_WritePin(AS_CS_PORT, AS_CS_PIN, GPIO_PIN_SET)
#define AS_CS_0          HAL_GPIO_WritePin(AS_CS_PORT, AS_CS_PIN, GPIO_PIN_RESET)

#define AS_SCK_PORT      GPIOA
#define AS_SCK_PIN       GPIO_PIN_5
#define AS_SCK_INIT      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)   //low level
#define AS_SCK_1         HAL_GPIO_WritePin(AS_SCK_PORT, AS_SCK_PIN, GPIO_PIN_SET)
#define AS_SCK_0         HAL_GPIO_WritePin(AS_SCK_PORT, AS_SCK_PIN, GPIO_PIN_RESET)

#define AS_SDI_PORT      GPIOA
#define AS_SDI_PIN       GPIO_PIN_7
#define AS_SDI_INIT      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)    //high level
#define AS_SDI_1         HAL_GPIO_WritePin(AS_SDI_PORT, AS_SDI_PIN, GPIO_PIN_SET)
#define AS_SDI_0         HAL_GPIO_WritePin(AS_SDI_PORT, AS_SDI_PIN, GPIO_PIN_RESET)

#define AS_SDO_PORT      GPIOA
#define AS_SDO_PIN       GPIO_PIN_6
#define AS_SDO_INIT
#define AS_SDO_IN        HAL_GPIO_ReadPin(AS_SDO_PORT, AS_SDO_PIN)
#define AS_SDO           1

#define AS_WAKE_PORT     GPIOB//GPIOA
#define AS_WAKE_PIN      GPIO_PIN_14//GPIO_PIN_3
#define AS_WAKE_INIT
#define AS_WAKE_IN       HAL_GPIO_ReadPin(AS_WAKE_PORT, AS_WAKE_PIN)
#define AS_WAKE          1

#define AS_CL_DAT_PORT   GPIOA
#define AS_CL_DAT_PIN    GPIO_PIN_8
#define AS_CL_DAT_INIT
#define AS_CL_DAT_IN     HAL_GPIO_ReadPin(AS_CL_DAT_PORT, AS_CL_DAT_PIN)
#define AS_CL_DAT        1

#define AS_DAT_PORT      GPIOB
#define AS_DAT_PIN       GPIO_PIN_15
#define AS_DAT_INIT
#define AS_DAT_IN        HAL_GPIO_ReadPin(AS_DAT_PORT, AS_DAT_PIN)
#define AS_DAT           1

#define LF_INTERRUPT_DIS  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn)      //EXTI2_3_IRQn
#define LF_INTERRUPT_EN   HAL_NVIC_EnableIRQ(EXTI4_15_IRQn)

extern void LF_SPI_Write(unsigned char data);
extern void LF_Write_Register(unsigned char RegAdd,unsigned char Data);
extern void LF_Write_Command(unsigned char Command);
extern unsigned char LF_Read_Register(unsigned char RegAdd);

extern void LF_SPI_GPIO_Init(void);

extern void LF_SPI_GPIO_DeInit(void);

extern void LF_AS3932Init(void);
extern void LF_AS3932DeInit(void);
extern void LF_AwakeUp(void);

extern unsigned char LF_Rec_Data( unsigned int ulSysTime );

extern void TimerHwInit(void);

extern void TimerIncrementTickCounter( void );

extern uint32_t TimerHwGetTimerValue( void );

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);



// LF通信得到的全局变量 
//#ifdef LF_LIB
//unsigned char LF_RecByte[8];
//unsigned char LFReceived;             //flag of receive
//unsigned char LF_DecodeErr_cnt;
//#else
extern unsigned char LF_RecByte[8];
//extern unsigned char LFReceived;             // LFReceived = 0xAA;接收到4个字节 LFReceived = 0xCC;接收到8个字节
//extern unsigned char LF_DecodeErr_cnt;
//#endif


#endif
