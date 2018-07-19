#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif


#include<string.h>
#include<stdio.h>
#include<stdlib.h>

#include "Aware_Protocol.h"
#include "Aware_Config.h"

#include "stm32l0xx_hal.h"


#include "hw.h"
//#include "cmsis_os.h"
#include "Radio.h"
#include "timeServer.h"

#include "Flash.h"
#include "sx1276.h"
#include "low_power.h"

#include "tim.h"

//#include "mlm32l07x01.h"

#include "lptim.h"
#include "LF_AS3932.h"
#include "LF_Protocol.h"

#include "ohr_interface.h"

//#include "wakeup.h"

#include "LFwakeup.h"
//////////  debug info ////////////
//#define DEBUG


////////////  verion  /////////////////////
#define TEST_VERSION (uint32_t) 0x00000000  /*2 lsbs is always 00 in releases   */
#define LORA_MAC_VERSION   (uint32_t) 0x44000000
#define APP_VERSION  (uint32_t) 0x00020000  /*3 hex in the middle i_cube release*/
#define VERSION   (uint32_t) (LORA_MAC_VERSION | APP_VERSION | TEST_VERSION)


#define TAG_SN_Type     0x80010138     // 8001 aware, 0136 babytag

#define TAG_SN_ID_ADDR  0x08000800  //0x08000800     //user sn store addr and bootload need attention

#define MATCH_TAG_Type        0x37   //标签类型   babytag  0x36
#define TAG_Type        0x38   //标签类型   babytag  0x36
#define FirmwareVERSION 0x10	 //固件版本
#define HARDVERSION     0x10   //硬件版本


#define LORA_MAX_LEN                                255

#define LORA_NO_ACK            0
#define LORA_ACK_RESP          1

/////  Return value  //////////////

//#define VALID_POLL                   (2)
//#define NO_VALID_POLL                (3)

//#define NET_FLG                     (0x01)
//#define NO_NET_FLG                  (0x00)
//#define NO_PARA                     (0xff)

////////////// Uart  ////////////
//#define PRINTF(...)            printf(__VA_ARGS__)

//#define NOVALID                         255
#define GPS_FRAME_SIZE					1000//126    for Super Frame
#define UART_FRAME_SIZE					128//255//126    for Super Frame
#define	UART_BUFFER_SIZE      				2              //add rx buff
//#define UART_RX_BUFFER                  2

#define UART_FRAME_EMPTY				0x00
#define UART_FRAME_STATUS_READY			0x01
#define UART_FRAME_STATUS_SENDING		0x02
//#define UART_FRAME_STATUS_CMD       	0x04
#define UART_FRAME_RECEIVING			0x04
#define UART_FRAME_STATUS_MASK          0x07           

//#define RECEIVE_BUF_LEN    255//64//32

#define UART_RX_GAP                     (1)

#pragma pack (1)

typedef struct
{
	u8			aucData[UART_FRAME_SIZE];
	u8			ucLength;					// ucLength=0 means empty
	u8			ucStatus;
	u32 		ulReceive_Time; 			// sys tick
}	UART_BUFFER;

typedef struct
{
	u8			aucData[GPS_FRAME_SIZE];
	u8			ucStatus;	
	u16			ucLength;					// ucLength=0 means empty
//	u32 		ulReceive_Time; 			// sys tick
}	GPS_BUFFER;


void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle);

void Uart_Enable(void);

void MX_I2C2_Init(void);

void MX_DMA_Init(void);

void MX_USART1_UART_Init(void);

void MX_USART2_UART_Init(void);

void MX_LPUART1_UART_Init(void);

void Uart_Send(uint8_t* data, uint8_t len);

void Uart2_Send(uint8_t* data, uint8_t len);

void LpUart1_Send(uint8_t* data, uint8_t len);


void print_8_02x(uint8_t *pt);

void Store_UART_Data( u32 ulSysTime );

u8	Process_UART_Data( u32 ulSysTime );

u8 UART_AT_CMD_Process(u8* data, u8 ucLen, u32 ulSysTime);


#define DEF_MATCH_TAG_ID    (0x000017DD)
//#define DEF_TAG_ID        (0x11223344)

#define EVN_IDLE       0
#define EVN_RST        1
#define EVN_SET_CH     2
#define EVN_SET_TXRATE 3
#define EVN_SET_TXPWR  4
#define EVN_EXIT_SEEK  5
#define EVN_TEST       6
#define EVN_OHR        7


typedef enum
{
	RF_Rate_250 = 0x00,       //SF12/125KHZ
	RF_Rate_440,              //SF11/125KHZ
	RF_Rate_980,	          //SF10/125KHZ
	RF_Rate_1760,             //SF9/125KHZ
	RF_Rate_3125,	          //SF8/125KHZ
	RF_Rate_5470,             //SF7/125KHZ
	RF_Rate_11000,             //SF7/250KHZ
	RF_Rate_20000              //SF7/500KHZ  OTA 
	
}RF_Rate_Enum;

typedef enum
{
	BW_125K = 0x00,      
	BW_250K,              
	BW_500K          
	
}RF_BW_Enum;

#define KEY_PRESS_TIME       50
#define KEY_LONG_PRESS_TIME  5000

#define KEY_PORT       GPIOA
#define KEY_PIN        GPIO_PIN_11

#define M_KEY_PORT       GPIOA
#define M_KEY_PIN        GPIO_PIN_12


typedef enum
{
	Key_Func = 0x00,       
	Key_Match,              
	
}Key_Enum;

typedef struct
{
	u8   ucKey;
	u8   ucKey_Status;
	vu8  ucKey_INT;
	
}Key_typedef;


typedef enum
{
	Key_Idle = 0x00,       
	Key_Int,              
	Key_Poll,
	Key_On,
	key_Press,	
	Key_Long,
	Key_Long_Press,
	
}Key_Status_Enum;


#define FLASH_50MS         50
#define FLASH_100MS        100
#define FLASH_2S           2000
#define FLASH_3S           3000

#define SPK_50MS    50//1600//   160
#define SPK_100MS    100//1600//   160
#define SPK_500MS    500//1600//   160
#define SPK_3S       3000//1600//   160



typedef enum
{
	None= 0x00,       	
	ON,    
	OFF,
	Flash,
	
}Status_Enum;

typedef struct
{
	u8  ucStatus;
	u8  ucTimes;
	u16 uiPeriod;
	u32 ulPertick;
	
}Opt_typedef;


typedef enum
{
	Led_None = 0x00,       
	Red_Led,              
	Green_Led,
	Blue_Led,
	
}LED_Enum;

#pragma pack ()

#define GPS_VCC_PORT      GPIOB
#define GPS_VCC_PIN       GPIO_PIN_11

#define GPS_VCC_ON()     HAL_GPIO_WritePin(GPS_VCC_PORT, GPS_VCC_PIN, GPIO_PIN_RESET)
#define GPS_VCC_OFF()    HAL_GPIO_WritePin(GPS_VCC_PORT, GPS_VCC_PIN, GPIO_PIN_SET)



///////////  adc en  ////

#define ADC_EN_PIN         GPIO_PIN_1
#define ADC_EN_GPIO_PORT   GPIOA

#define ADC_EN_ON()     HAL_GPIO_WritePin(ADC_EN_GPIO_PORT, ADC_EN_PIN, GPIO_PIN_RESET)
#define ADC_EN_OFF()    HAL_GPIO_WritePin(ADC_EN_GPIO_PORT, ADC_EN_PIN, GPIO_PIN_SET)


///////// touch ///////////
#define Q1_VDD_PIN            GPIO_PIN_10
#define Q1_VDD_GPIO_PORT      GPIOA
#define Q1_VDD_ON()     HAL_GPIO_WritePin(Q1_VDD_GPIO_PORT, Q1_VDD_PIN, GPIO_PIN_SET)
#define Q1_VDD_OFF()    HAL_GPIO_WritePin(Q1_VDD_GPIO_PORT, Q1_VDD_PIN, GPIO_PIN_RESET)
#define Q1_READ()       HAL_GPIO_ReadPin(Q1_VDD_GPIO_PORT, Q1_VDD_PIN)

#define Q2_PIN                GPIO_PIN_15
#define Q2_GPIO_PORT          GPIOC
#define Q2_READ()       HAL_GPIO_ReadPin(Q2_GPIO_PORT, Q2_PIN)



//////////// speek ///////////
#define  SPK_PWM_ON()         HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) //(TIM4->CCER &=~(0x0100)) // TIM4_CH3
#define  SPK_PWM_OFF()        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) //(TIM4->CCER |=0x0100)   	// TIM4_CH3

#define SPK_PIN         GPIO_PIN_15
#define SPK_GPIO_PORT   GPIOA

#define SPK_ON()   HAL_GPIO_WritePin(SPK_GPIO_PORT, SPK_PIN, GPIO_PIN_RESET)
#define SPK_OFF()  HAL_GPIO_WritePin(SPK_GPIO_PORT, SPK_PIN, GPIO_PIN_SET)
#define SPK_TOG()  HAL_GPIO_TogglePin(SPK_GPIO_PORT, SPK_PIN)

///////////////  led////////////////////
#define LED_R_PIN         GPIO_PIN_13
#define LED_R_GPIO_PORT   GPIOC

#define LED_R_ON()   HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_PIN, GPIO_PIN_RESET)
#define LED_R_OFF()  HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_PIN, GPIO_PIN_SET)
#define LED_R_TOG()  HAL_GPIO_TogglePin(LED_R_GPIO_PORT, LED_R_PIN)

#define LED_G_PIN         GPIO_PIN_14
#define LED_G_GPIO_PORT   GPIOC

#define LED_G_ON()   HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, GPIO_PIN_RESET)
#define LED_G_OFF()  HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, GPIO_PIN_SET)
#define LED_G_TOG()  HAL_GPIO_TogglePin(LED_G_GPIO_PORT, LED_G_PIN)

#define LED_B_PIN         GPIO_PIN_15
#define LED_B_GPIO_PORT   GPIOC

#define LED_B_ON()   HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, GPIO_PIN_RESET)
#define LED_B_OFF()  HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, GPIO_PIN_SET)
#define LED_B_TOG()  HAL_GPIO_TogglePin(LED_B_GPIO_PORT, LED_B_PIN)

///////////////// charg ///
#define CHG_CHK_PIN        GPIO_PIN_0
#define CHG_CHK_PORT       GPIOH
#define CHG_CHK_Read()     HAL_GPIO_ReadPin(CHG_CHK_PORT, CHG_CHK_PIN)

#define CHG_PIN         GPIO_PIN_1
#define CHG_GPIO_PORT   GPIOH
#define CHG_Read()     HAL_GPIO_ReadPin(CHG_GPIO_PORT, CHG_PIN)



#define RF_RX_BUF_NUM                               2     

//////////////////// LoRa  ///////////////////
#if defined(Router)|defined(COO)

#define RX_TIMEOUT_VALUE                            0

#else

#define RX_TIMEOUT_VALUE                            2000

#endif

#define RF_BIND_CH                                  37      //470m

#define RF_FREQUENCY                                433000000//433000000 // Hz   0 channel Freq

#if defined(PA)
#define TX_OUTPUT_POWER                             20        // dBm  PA 20dbm need PCB PA mode
#else
#define TX_OUTPUT_POWER                             5//14        // dBm  PA 20dbm need PCB PA mode

#endif


//#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
//#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        6//8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

//////motion //////////
//#define TIME_MS(msec)                                 ((float)RTC_Sec_Cnt/1000*msec)
#define TIME_S(sec)                                 (RTC_Sec_Cnt*sec)

#define HB_TIME                                     60
#define HB_MIN_TIME                                 15
#define STATIC_TIME                                 30//5MIN = 60000*5
#define MOTION_SAMPLE_TIME                          500//500//500  //MS
#define MOTION_THRES                                40

#define BAT_REF_VOLT           3140//4150    //4000mv
#define BAT_FULL_VOLT          4100    //3900mv
#define BAT_LOW_VOLT           3800    //3700mv
#define BAT_MIN_VOLT           3600    //3500mv

#define CALI_VOLT              5//55

#define ADC_ACC                12      //4096

#define TAG_CON_ADC_MIN        700    // > 2K band
#define TAG_CON_ADC            1200    // < 4K band
#define TAG_ERROR_ADC          1500
#define TAG_CHG_ADC            2500
#define ADC_THRED              100

///////////////  Dev protocol //////////// 
#define BIG_ENDIAN_2(x) (u16)(((x&0xFF00)>>8)|((x&0x00FF)<<8))
#define BIG_ENDIAN_4(x) (u32)(((x&0xFF000000)>>24)|((x&0x00FF0000)>>8)|((x&0x000000FF)<<24)|((x&0x0000FF00)<<8))

#define MIN_PRO_TYPE      0x01
#define STA_PRO_TYPE      0x02

#define CORE_CID         (0x2000)
#define BEACON_CMD        (0x01)
#define REGIST_CMD        (0x02)

///////////  minipro index  table////////////////
#define HB_S_INDEX          0   //report, work status+motion + heartRate + BAT
#define HB_GAP_W_INDEX      1       //write, report gap
#define HB_GAP_WS_INDEX     2       //write resp, report gap
#define HB_GAP_R_INDEX      3       //read, report gap
#define HB_GAP_RS_INDEX     4   //read resp, report gap

#define STATUS_S_INDEX      5   //report sleep,charge,work
#define STATUS_W_INDEX      6   //write seek mode, test mode
#define STATUS_WS_INDEX     7   //write resp seek mode, test mode

#define BAT_R_INDEX         8   //read, bat level
#define BAT_RS_INDEX        9   //read resp, bat level
#define BAT_S_INDEX         10   //read, bat level
#define CHG_S_INDEX         11   //report, charging/full

#define LC_ALARM_S_INDEX    14   //report, location status(u8) +RSSI + location id(u16)  DEV--SER
#define LC_S_INDEX          15   //report, location status(u8) +RSSI + location id(u16)  DEV--SER
#define LC_ID_R_INDEX       16   //read, location id(u16)
#define LC_ID_RS_INDEX      17  //read resp, location id(u16)
#define LC_HIS_S_INDEX      18   //report, location history id(u16)
#define LC_HIS_R_INDEX      19   //read, location history id(u16)
#define LC_HIS_RS_INDEX     20   //read resp, location history id(u16)

#define SN2_W_INDEX          22   //write, bindtagSN(4B)
#define SN2_WS_INDEX         23   //write resp, bindtagSN(4B)
#define SN2_R_INDEX          24   //read, bindtagSN(4B)
#define SN2_RS_INDEX         25   //read resp, bindtagSN(4B)

#define SN_W_INDEX          26   //write, bindtagSN(4B)
#define SN_WS_INDEX         27   //write resp, bindtagSN(4B)
#define SN_R_INDEX          28   //read, bindtagSN(4B)
#define SN_RS_INDEX         29   //read resp, bindtagSN(4B)

#define SN3_W_INDEX          30   //write, bindtagSN(4B)
#define SN3_WS_INDEX         31   //write resp, bindtagSN(4B)
#define SN3_R_INDEX          32   //read, bindtagSN(4B)
#define SN3_RS_INDEX         33   //read resp, bindtagSN(4B)



#define SOS_S_INDEX         35   //report sos
#define TAG_ALARM_S_INDEX     36   //
#define HB_RECORD_INDEX       37

#define CUT_S_INDEX         40//report band status 
#define CUT_R_INDEX         41//report band status 
#define CUT_RS_INDEX        42  //report band status 

#define MATCH1_R_INDEX       50  //read match status  1 match  
#define MATCH1_RS_INDEX      51   //read resp match status  1 match
#define MATCH1_W_INDEX       52  //write match status  1 match
#define MATCH1_WS_INDEX      53   //write resp match status  1 match

#define MATCH2_R_INDEX       54  //read match status  1 match  
#define MATCH2_RS_INDEX      55   //read resp match status  1 match
#define MATCH2_W_INDEX       56  //write match status  1 match
#define MATCH2_WS_INDEX      57   //write resp match status  1 match

#define MATCH3_R_INDEX       58  //read match status  1 match  
#define MATCH3_RS_INDEX      59   //read resp match status  1 match
#define MATCH3_W_INDEX       60  //write match status  1 match
#define MATCH3_WS_INDEX      61   //write resp match status  1 match

#define PATI_AREA_R_INDEX      65    //read patient area info
#define PATI_AREA_RS_INDEX     66    //read resp
#define PATI_AREA_W_INDEX      67    //write patient area info
#define PATI_AREA_WS_INDEX     68    //write resp
#define PATI_BED_R_INDEX       69    //read patient bed info
#define PATI_BED_RS_INDEX      70    //read resp 
#define PATI_BED_W_INDEX       71    //write patient bed info
#define PATI_BED_WS_INDEX      72    //write resp


#define RFCH_W_INDEX        80//write, LoRa channel
#define RFCH_WS_INDEX       81//write resp, LoRa channel
#define RFCH_R_INDEX        82//read, LoRa channel
#define RFCH_RS_INDEX       83//read resp, LoRa channel

#define RFRATE_W_INDEX      84//write, LoRa rate
#define RFRATE_WS_INDEX     85//write resp, LoRa rate
#define RFRATE_R_INDEX      86//read, LoRa rate
#define RFRATE_RS_INDEX     87//read resp, LoRa rate

#define RFPWR_W_INDEX       88//write, LoRa Tx pwr
#define RFPWR_WS_INDEX      89//write resp, LoRa Tx pwr
#define RFPWR_R_INDEX       90//read, LoRa Tx pwr
#define RFPWR_RS_INDEX      91//read resp, LoRa Tx pwr

#define RST_W_INDEX         95//Write, reset
#define RST_WS_INDEX        96//write resp, reset

#define DEF_W_INDEX         97//Write, default
#define DEF_WS_INDEX        98//write resp, default

#define GPS_ONOFF_R_INDEX    100  // read gps on/off, 1 on
#define GPS_ONOFF_RS_INDEX   101   //read resp
#define GPS_ONOFF_W_INDEX    102  //write gps on/off  1 on
#define GPS_ONOFF_WS_INDEX   103  //write resp 

#define GPS_GAP_R_INDEX      104  //read gps sleep gap, unit:sec
#define GPS_GAP_RS_INDEX     105   //read resp 
#define GPS_GAP_W_INDEX      106  //write gps sleep gap
#define GPS_GAP_WS_INDEX     107  //write resp 

////UTC(hhmmss.sss)+Lat(ddmm.mmmm)+uLat(N,S)+Lon(dddmm.mmm)+uLon(E,W)
#define GPS_INFO_S_INDEX      108  //Lati_NS(1B) + Long_EW(1B) + Lati(4B) + Long(4B) + Utc(4B)




/////////////////  cmd ACTION///////////
#define D_REPORT        0x00
#define S_WRITE         0x01
#define S_READ          0x02
#define D_REQ           0x03

#define D_WRITE_ACK     0x81
#define D_READ_ACK      0x82
#define S_RESP          0x83



#pragma pack (1)

#define GPS_ONOFF_GAP_S                             10                               
#define GPS_COLD_WAIT_S                             360  //6*60                               
#define GPS_HOT_WAIT_S                              30   //5                             

#define GPS_OFF                      0
#define GPS_COLD_RESTART             1
#define GPS_HOT_RESTART              2


typedef struct
{	
	u8 Run_Status;		       //gps enable/disable
	u8 ONOFF_Gap;		        //sec	
	u8 ucStart_Mode;                  // 1 waiting gps info
	u8 ucIdle;                  // 1 waiting gps info

}GPS_Status_Typedef;

typedef struct
{	
	u8 ucLati_NS;              // 1--N, 0--S	
	u8 ucLong_EW;              // 1--w, 0--E
	u32 ulLati;               
	u32 ulLong;
	u32 ulUTC;

}GPStoSer_Typedef;



#define UTCTime_Length 11
#define latitude_Length 10
#define N_S_Length 2
#define longitude_Length 11
#define E_W_Length 2 

typedef struct
{	
	u8 isGetData;		//是否获取到GPS数据
	u8 UTCTime[UTCTime_Length];		//UTC时间
	u8 latitude[latitude_Length];		//纬度
	u8 N_S[N_S_Length];		//N/S
	u8 longitude[longitude_Length];		//经度
	u8 E_W[E_W_Length];		//E/W
	u8 isUsefull;		//定位信息是否有效

}GPS_Typedef;



#define OHR_LEN         8
#define HEAD_OHR2HOST   0xFD
#define HEAD_HOST2OHR   0xFE

#define CMD_NO_RAW      0x50
#define START_RESP      0x80

typedef struct
{	
	u8 ucHead;       //5V or bat
	u8 ucCmd; 
	u8 ucLen;
	u8 ucHeartRate;
	u8 ucSnr;
	u8 ucStatus;
	u8 ucCur;
	u8 ucCRC;

}OHR_Typedef;


typedef enum
{
	Reg_None = 0x00,       
	Reg_Done, 
	Reg_Failed,
	Reg_TimeOut,
	
}Reg_Enum;


#define MATCH_CMD        (0x91)
#define MATETAG_ID       (0x38)

typedef struct
{	
	u8 ucCmd;
	u8 ucTagID;
	u32 ulTagSN;
	
}RF_Config_Typedef;

/////// pwr status  ////////
#define BAT                     0
#define ADAPTER_5V              1

#define BAT_CHGING              1
#define BAT_CHG_FULL            0

#define CHG_REPORT_GAP          20//120
#define BAT_OFFSET              3

typedef struct
{	
	u8 ucPwr_Type;       //5V or bat
	u8 ucBat_Status;     //charging or full ,5v
	u8 ucBat_level;      // 0-100% level, BAT
	
}Pwr_Typedef;

/// work status
#define STATIC_STATUS                               0
#define MOVE_STATUS                                 1
#define LOC_STATUS                                  2


////////// band status ////////
#define CUT_STATUS              0
#define CONNECT_STATUS          1
#define ABNORMAL_STATUS         2
#define ADC_STATUS              0x10


#define SLEEP                   0
#define CHG                     1
#define WORK                    2
#define SEEK                    3
#define TEST                    4

#define HB_MAX                  120
#define HB_MIN                  50
#define HB_OFFSET               5
#define HB_RECORD_MAX           24
#define HB_VALID_SNR            5
#define HB_UNVALID_SNR          -5
#define HB_RECORD_GAP           5
#define HB_UNVALID_MAX          3
#define HB_SUM_CNT              32

typedef struct
{
	
	u8  ucSum_Cnt;                    //16 or 32 == 16sec or 32sec
	u8  ucRecord_Gap;                 //unit min, > 5	
	u8  ucUnvalid_Cnt;       
	u8  ucAlarm_HR;                   // 1 drop alarm, 0 put on
	u8  ucAve_HR;                     //average heartRate
	u8  aucHB_Record[HB_RECORD_MAX];    
	u16 uiSum_HR;                     // 32 SUM HR
	
}HR_Typedef;


typedef struct
{
	u8 ucPreStatus;         //pre work status
	u8 ucWork_Status;      //sleep, charge, work, seek, test
	u8 ucMove_Status;      //move or static
//	u8 ucReg_Status;       //register or not
	u8 ucBind_Status;      // matetag bind
	u8 ucJoin_Status;      
	u8 ucBand_Status;      //band cut,connect,abnormal 
	u8 ucTouch;            //touch cnt	
//	u8 ucHeartRate;            //touch cnt
	
}SysStatus_Typedef;

#define SN_IDLE_STATUS               0
#define SN_MATCH_STATUS              1
#define SN_RSP_MATETAG               2
#define SN_RSP_SERVER                3
#define SN_TRIG_MATCH                4
#define SN_REC_MATCH_RSP             5    // matetag rec 470M lora resp
#define SN_MATCHING                  6
#define SN_MATCH_END                 7

typedef struct
{
	u8   ucStx;
	u8   ucRF_Channel;
	u8   ucRF_Rate;
	u8   ucBand_Width;
	u8   ucRF_Pwr;
	u8   ucHB_Indicate_EN;    //指示灯使能标志
	u8   ucSleep_EN;     //自动休眠使能标志	
	u8   ucStatic_Time;    //长静止等待时间(单位:分钟)
	u8   ucReg_Status;       //register or not
	u8   ucWork_Status;
	u8   ucPati_Area;        //病区信息
	u8   ucPati_Bed;         //病人信息
//	u8   ucBind_Status;      // matetag bind	
  	u8   ucMatch1_Status;	 //匹配状态值，1表示匹配ID1成功
  	u8   ucMatch2_Status;    //匹配状态值，1表示匹配ID2成功
  	u8   ucMatch3_Status;    //匹配状态值，1表示匹配ID3成功
	u16  uiHB_Gap;         //心跳帧周期(单位:500ms)
  	u16  uiMove_Threshold;    //震动阀值	
//	u32  ulMatch_ID1;       //匹配标签ID1
//	u32  ulMatch_ID2;       //匹配标签ID2 
//	u32  ulMatch_ID3;       //匹配标签ID3
	Dev_Addr aucMatch_SN1;       //匹配标签SN1
	Dev_Addr aucMatch_SN2;       //匹配标签SN2
	Dev_Addr aucMatch_SN3;       //匹配标签SN3
	Dev_Addr aucTag_SN;       //标签SN
//	u32  ulTAG_ID;            //标签ID	
	u8   ucCheck;
	u8   ucEnd;
	
}NV_Config_Typedef;

#define LOC_EXIT    0
#define LOC_ENTER   1
#define LOC_PASS    2


typedef struct
{
	u8    ucStatus;     //0 exit,1 enter ,2 pass
	s8    scRSSI;
	u8    ucType;       //0 normal, 1 alarm 
	u16   uiID;
	u16   uiHis_ID;
	u32   ulPretick;
	
}Location_Typedef;


#define MAX_PAYLOAD_LEN    49 //53 -4 = 49
#define MINPRO_DATA_LEN    48 //53 -4 -1 = 48
#define STAPRO_DATA_LEN    44 //53 -4 -5 = 44

#define PRO_HEAD_LEN       3
#define MIN_HEAD_LEN       1
#define STA_HEAD_LEN       5


typedef struct
{
	u8   ucLen;
	u8   ucType;
	u8   ucDev_Seq;
	u8   ucSer_Seq;
	u8   ucPayLoad[MAX_PAYLOAD_LEN];
	
}Dev_Pro_Typedef;

typedef struct
{
	u8   ucIndex;
	u8   ucData[MINPRO_DATA_LEN];
	
}MinPro_Typedef;

typedef struct
{
	u8    ucEP;
	u16   uiCID;
	u8    ucCMD;
	u8    ucID;
	u8    ucData[STAPRO_DATA_LEN];
	
}StaPro_Typedef;


#pragma pack ()



void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

void OnCadDone( bool channelActivityDetected );

void Time_Init(void);

void Get_TagSN(uint8_t* sn);

u8 Poll_RF_Buffer(void);

u8 Poll_UART_Buffer(void);

void Enter_Sleep(void);

u8 RF_DataRate_Config(RF_Rate_Enum ucRate, u8* BW, u8 RF_Pwr);

void Parse_GpsBuffer(u8* Data, GPS_Typedef* Gps);

u8 RF_CH_Config(u8 ch, u8 BW);

void Dev_Event(u32 ulSysTime);

u8 GPS_Idle(void);

u8 HeartRate_Idle(void);

void Systick_Event(u32 ulSysTime);

void MX_ADC_Init(void);

void Pwr_Init(void);

void Pwr_DeInit(void);

u16 HW_AdcReadChannel( uint32_t Channel );

u8 HW_GetBatteryLevel(void);


//void HW_AdcInit( void );

void GPIO_DeInit(void);

void Hw_Init(void);

void LoRa_Init(void);

void LoRa_Config(NV_Config_Typedef* config);
//void LoRa_Config(CNF_Parameter* config);

u8 Process_App_RF_RT_Rec(u8* pucData, u8 ucLen, Dev_Addr* pSrcAddr, u32 ulSysTime);

u8 Process_App_RF_Rec(u8* pucData, u8 ucLen, u16 uiSrcAddr, u32 ulSysTime);

u8 Process_Cmd_Status(u16 uiSrcAddr, CMD_Enum Cmd, u8* CmdValue, u8 ucLen, u8 ucOpt, u8 bStatus);

u8 Process_Join_Status(Join_Status_Enum bStatus);

u8 Process_App_Ack(u16 uiSrcAddr, u8 bStatus, u32 ulSysTime);

u8 Process_RF_Tx(u8* pucData, u8 ucLen);

/////// spk ////////
//void SPK_PWM_ON(void);

//void SPK_PWM_OFF(void);

void SPK_TIM2_Init(void);

void SPK_GPIO_Init(void);

void SPK_GPIO_DeInit(void);

u8 SPK_PWM_Toggle(void);

u8 SPK_PWM_Ctrl(Opt_typedef* Speek, u32 ulSysTime);

u8 SPK_Ctrl(Opt_typedef* Speek, u32 ulSysTime);

u8 Process_PWM_SPK( u32 ulSysTime );

u8 SPK_Idle(void);

u8 Register_Done(void);

void SPK_Show(Status_Enum SPK_Status, u8 ucTimes, u16 uiPeriod);

u8 Process_SPK_Event( u32 ulSysTime );

void GPS_GPIO_Init(void);

////////// led ///////////
void LED_GPIO_Init(void);

void LED_GPIO_DeInit(void);

u8 LED_Idle(void);

u8 LED_Ctrl(LED_Enum eLed, Opt_typedef* Led, u32 ulSysTime);

void LED_Show( LED_Enum Led, Status_Enum Led_Status, u8 ucTimes,  u16 uiPeriod);

u8 Process_LED_Event( u32 ulSysTime );
////////////////////

u8 Bind_Done(void);

u8 LF_Done(void);

void Process_OHR(u32 ulSysTime);

void Process_LF(u32 ulSysTime);

void Process_LF_TX(u32 ulSysTime);

void Process_Motion(u32 ulSysTime);

void Process_Pwr(u32 ulSysTime);

void Process_Bat(u32 ulSysTime);

void Process_Bind(u32 ulSysTime);

void Process_WorkMode(u32 ulSysTime);

void Process_Tag_Alarm(u32 ulSysTime);

void Process_GPS(u32 ulSysTime);

void Process_Net_Status(u32 ulSysTime);

void Node_Change( Dev_Addr*ExtAddr, u16 uiAddr, u8 ucType, u8 ucOpt);

u8 MINI_Pro_Resp(u8* pucData, u32 ulSysTime);

u8 Tag_Bind_Resp(u8 status, u32 ulSysTime);

void Process_Error(u8 ucError);

void Debug_Printf(int Data );

u8 Match_Tag_ID(u32 MatchID);

u8 GetCH(void);

u8 SetCH(u8 ucRF_CH);

u8 GetBW(void);

u8 SetBW(u8 ucRF_BW);

u8 GetRFRate(void);

u8 SetRFRate(u8 ucRF_Rate);

u8 GetRFPwr(void);

u8 SetRFPwr(u8 ucRF_Pwr);

u8 KEY_Idle(void);

void Key_GPIO_INT_Init(void);

void Key_GPIO_INT_DeInit(void);

void Key_Status_Poll(Key_typedef *pKey, u32 ulSysTime);

void Key_Func_Handler(Key_typedef *pKey, u32 ulSysTime);

u8 Key_Read(u8 ucKey);


void Process_Key(u32 ulSysTime);

void Touch_GPIO_Init(void);

void Touch_GPIO_DeInit(void);

u8 Process_Touch( u32 ulSysTime );



/*!
 *  \brief Unique Devices IDs register set ( STM32L05x )
 */
#define         ID1                                 ( 0x1FF80050 )   //2B
#define         ID2                                 ( 0x1FF80054 )   //2B
#define         ID3                                 ( 0x1FF80058 )   //4B
#define         ID4                                 ( 0x1FF8005C )   //4B

uint32_t HW_GetRandomSeed( void );

void HW_GetUniqueId( uint8_t *id );

u8 Process_LoRa_Data(u32 ulSysTime);

u8 Check_APP_NV_Para(void);

u8 Write_App_NV(void);

u8 Read_App_NV(void);

u8 Delete_App_NV(void);

void RTC_Cali(u32 Freq);

u8 Process_Reg( u32 ulSysTime );

u8 GPS_Info_Report(u32 ulSysTime);

u8 Process_HeartBeat( u32 ulSysTime );

u8 Dev_Period_Report( u32 ulSysTime );

u8 Tag_Alarm_Report(u32 ulSystime, u8 Alarm_Type);

u8 CHG_Report(u32 ulSystime);

u8 BAT_Level_Report(u32 ulSystime);

u8 WorkMode_Report(u32 ulSystime);

u8 Band_SOS_Report( u8 Status, u32 ulSysTime);

u8 LF_Report(u8 Status, u32 ulSystime);

u8 LF_Alarm_Report(u8 Status, u32 ulSystime);

u8 Tag_Bind_Report( u8 Status, u32 ulSysTime);

u8 HB_Record_Report(u32 ulSystime);

u8 Dev_Register( u32 ulSysTime );

u8 DevPro_Send( u8* pData, u8 ucLen, u8 ucSerSeq, u8 ucDevSeq, u32 ulSysTime );

u8 DevPro_RT_Send( u8* pData, u8 ucLen, u8 ucSerSeq, u8 ucDevSeq, u32 ulSysTime );

void *osal_memcpy( void *dst, const void *src, u32 len );

void *osal_revmemcpy( void *dst, const void *src, u32 len );

void *osal_memset( void *dst, u8 value, u32 len );

u8 Check_Sum(u8* Data, u8 Len);

void Init_SysConfig(void);

u8 Add_HB_Record(u8 ucHB);

u8 Poll_Record_Num(void);

u8 Erase_HB_Record(void);


//u8 Store_SysConfig(void);

//void Delete_SysConfig(void);


void PackFrame_Data7Byte(uint8_t *Frame, uint8_t type, uint8_t cnt, uint8_t *SS);

void PackFrame_Config(uint8_t *Frame, uint8_t *SS);

void LF_Config( u32 ulSysTime );

u32 HL_ftoh(float fData);

u32 GpstoHex( u8* pData);


extern u16 RTC_Sec_Cnt;    //default RTC Value
extern UART_BUFFER	 asUART0_Buffer[UART_BUFFER_SIZE];
extern NV_Config_Typedef RFConfig;


#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H__ */

