#ifndef __AWARE_CONFIG_H
#define __AWARE_CONFIG_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>
	
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */
	
	/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
	
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t	s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;	/*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;	/*!< Read Only */
typedef __I int16_t vsc16;	/*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;	 /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;	/*!< Read Only */
	

#define PRINTF(...)            printf(__VA_ARGS__)

//////////  debug info ////////////
#define DEBUG


////////////  usr config para ////////////
#define NV     //enable Nv store


//#define MAX_ERROR_NUM                               3      
#define RF_TX_QUE_SIZE					           5
#define	RF_TX_MAX_RETRY_TIMES			           3
#define MAX_PARE_TBL                               10          //parent table number   
#define	POLL_BUFFER_SIZE		                   10          //parent poll buffer number

#define RF_MIN_GAP_MS                              5//10          
#define JOIN_PERIOD_S                              5000//2000
#define JOIN_DELAY_S                               2000
#define JOIN_TIMEOUT_S                             30000//(1000*30)
#define SUPER_FRAME_TIMEOUT                        5000//3000//(HW_RTC_ms2Tick(3000))//3000


#define POLL_GAP_MIN                               15//60 //      100 node 60s,  50 node 30s
#define POLL_GAP_S                                 15 //      100 node 60s,  50 node 30s

#define HEARTBEAT_GAP_MIN                          30        //100 node 120s, 50 node 60s
#define HEARTBEAT_GAP_S                            120//60    100 node 120s, 50 node 60s

///////////////////////// Node Table  //////////////////
#define COO_MAX_ROUTABLE                            100    //14*1000=14k 1024 1 page 
#define ROUTER_MAX_ROUTABLE                         100

#if defined(COO)

#define MAX_ROUTABLE   (COO_MAX_ROUTABLE)

#elif defined(Router)

#define MAX_ROUTABLE   (ROUTER_MAX_ROUTABLE)

#endif

/////////////////////////////////////////

#define NV_STX        0x5A
#define NV_END        0xA5


#pragma pack (1)

typedef struct 
{
	u8 aucAddr[8];
} Dev_Addr;


typedef struct
{
	u8   ucStx;           // 0x5A
//	u8   ucNv_Flag;      // 1: store valid para .
	u8   ucJoin_Flag;
	u8   ucPoll_TimeOut;
	u8   ucHeartBeat_Gap;
	u8   ucPID;	
	s8   scRSSI;           //parent rssi
	u16  uiNwkAddr;
	u16  uiPare_Addr;
	Dev_Addr sHost_Addr;
	u8   ucCheck;         //sum 
	u8   ucEnd;           //0xA5
	
}NV_Para_Typedef;


typedef struct
{
	Dev_Addr sNode_Addr;
	u16      uiAddr;
	s8       cRssi;
	u8       ucAge;
	u8       ucNode_Type;
	u8       ucSeq;
	
}Router_Table_Typedef;




#pragma pack ()


#endif

