/*

Description: AwareTag RF Protocol and API 

License: 

Maintainer: Hangzhou AwareTag tech
*/
 /******************************************************************************
  * @file    Aware_Protocol.h
  * @author  dozen Yao
  * @version V0.0.1
  * @date    2017/09/21
  * @brief   RF Protocol and API
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef ___AWARE_PROTOCOL_H
#define ___AWARE_PROTOCOL_H


#ifdef __cplusplus
 extern "C" {
#endif


#include "Aware_Config.h"

//////////  AwarePro verion  /////////
#define PRO_VER                  (0x04)

///////  Return value  //////////////
#define TRUE                          1
#define FALSE                         0

#define VALID_POLL                   (2)
#define NO_VALID_POLL                (3)

#define NET_FLG                     (0x01)
#define NO_NET_FLG                  (0x00)
#define NO_PARA                     (0xff)


//#define NOVALID                         255
#define JOIN_MIN_RSSI              -100//-90
////////////  NODE TYPE///////////////////////
#define COO_TYPE                   0x00
#define ROUTER_TYPE                0x01
#define ZED_TYPE	               0x02

////////////  join ///////////
//#define		JOIN_STATUS_INIT				0
//#define		JOIN_STATUS_SENT_BCON_REQ		1
//#define		JOIN_STATUS_SENT_JOIN_REQ		2
//#define		JOIN_STATUS_REJOIN_REQ   		3
//#define		JOIN_STATUS_RETRY_BECON_REQ		4
//#define     JOIN_STATUS_TIMEOUT             5
//#define     JOIN_STATUS_SUCCESS             6
//#define     LEAVE_STATUS                    7


//#define     DEV_STATUS_BUSY                 1
//#define     DEV_STATUS_IDEL                 0           //USR CAN ENTER STOP MODE


#define MAX_PACK_SIZE                               255//64//128

#define LORA_PACK_HEAD                              9//20
#define LORA_JOIN_LEN                               21
#define JOIN_RESP_LEN                               24
#define JOIN_RESP_ACK_LEN                           9

#define BEACON_RESP_LEN                             22


#define MAC_DATA_MAX_LEN                            100

//#define MAX_ERROR_NUM                               3
#define OPT_READ                                     (0x00)
#define OPT_WRITE                                    (0x01)

#define OPT_JOIN                                     (0x00)
#define OPT_DEL                                      (0x01)

#pragma pack (1)

typedef enum
{
	Join_Init = 0,
	Beacon_req,
	Join_req,
	Rejoin_req,
	Retry_Beacon_req,
	Join_TimeOut,
	Join_Success,
	Leave,
	
}Join_Status_Enum;


typedef enum
{
	Start_CMD      = 0x05,
	Del_CMD        = 0x05,
	Poll_Gap_CMD,
	HeartBeat_Gap_CMD,
	ReJoin_CMD,
	Reset_CMD,
	Default_CMD,

	End_CMD = 0x0F
	
}CMD_Enum;


typedef struct
{	
	void (*Para_Init)(u8 ucPollGap, u8 ucHBGap, u32 ulJoinDelay, Dev_Addr* pAddr);
		
	u8 (*RF_Rec)(u8* pucData, u8 ucLen, u16 uiSrcAddr, u32 ulSysTime);
	
	u8 (*RF_Tx)(u8* pucData, u8 ucLen);

	u8 (*Join_Status)(Join_Status_Enum bStatus);

	u8 (*App_Ack)(u16 uiSrcAddr, u8 bStatus, u32 ulSysTime);
	
	u8 (*Cmd)(u16 uiSrcAddr, CMD_Enum Cmd, u8* CmdValue, u8 ucLen, u8 ucOpt, u8 bStatus);    //status:0 success, 1 failed

	#if defined(COO)

	void (*Node)( Dev_Addr*ExtAddr, u16 uiAddr, u8 ucType, u8 ucOpt);
	
	u8 (*RF_RT_Rec)(u8* pucData, u8 ucLen, Dev_Addr*ExtAddr, u32 ulSysTime);
	
	#endif
	
	void (*Error)(u8 ucError);
	
	
}FuncReg_t;


typedef enum
{
	Normal = 0x00,      
	Rx_error,          //onRxError     
	Tx_error,	       //can not tx, OnTxtimerout
	Status_error,      //idle or standby instead of rx
	NO_Rx             //    tx ok, can not rx 
	
}LoRa_Status;

typedef	struct
{
	u8 ErrCnt;        //
	u8 TxCnt;        //rx error
	u8 TimeOutCnt;   //tx timeout error
	u8 Status;	
//	u16 ErrTime;     //idle
	
}Handle_Error_Typedef;

typedef	struct
{
	u8  ucJoinStatus;
	u8  ucRunStatus;  
	u8  ucDevType;
	u8  ucPacketSeq;
	u8	ucLastTXQuePos;	 //	= RF_TX_QUE_SIZE
	u8  ucPollReq;
	u8  ucRespFlag;      //cmd resp flag
	u32 ulJoinDelay;
	Dev_Addr* pLocalAddr;
	
}Sys_Para_Typedef;


#pragma pack ()

#define RELAY_TYPE_BIT             0x10     //no child node
#define OFFLINE_BIT                0x20

//// CMD///
#define JOIN_CMD                 0x01            //node join request
//#define DEL_CMD                  0x02            //COO delete node
#define POLL_CMD                 0x03            //ZED data request
#define HEARTBEAT_CMD            0x04

//#define CH_CMD                   0x05
//#define RF_RATE_CMD              0x06
//#define POLL_GAP_CMD             0x07
//#define HEARTBEAT_GAP_CMD        0x08
//#define RF_PWR_CMD               0x09

//#define RST_CMD                  0x0E
//#define DEF_CMD                  0x0F

#define APP_DATA_TRANS           0xF0

#define OPT_MASK                 0xF0
#define READ_CMD                 0x10
#define WRITE_CMD                0x20
#define REPORT_CMD               0x40
#define RESP_CMD                 0x80

#define JOIN_RESP_CMD            0x81
#define DEL_RESP_CMD             0x82
#define POLL_RESP_CMD            0x83

//#define LORA_PAYLOAD_MAX_LEN     (244)// 255-2-9    //(100 - 9)

//////  Frame Type  ///////////
#define DATA_NOACK_TYPE          0x01
#define DATA_ACK_TYPE            0x11

#define ACK_BIT                  0x10
#define ENCRY_BIT                0x20
#define SUPER_BIT                0x40


#define ACK_TYPE                 0x02
#define JOIN_TYPE                0x03
#define JOIN_ACK_TYPE            0x04
#define BEACON_TYPE              0x05
#define BEACON_RESP_TYPE         0x06
#define EXT_PID_TYPE             0x07

//#define RF_TX_QUE_SIZE					10

//#define	RF_TX_MAX_RETRY_TIMES			3

#define	RF_TX_QUE_OPT_DELAY				0x01
#define	RF_TX_QUE_OPT_RELAY				0x02
#define	RF_TX_QUE_OPT_BCAST				0x04
#define	RF_TX_QUE_OPT_POLL_MSG			0x10
#define	RF_TX_QUE_OPT_JOIN				0x20
#define	RF_TX_QUE_OPT_RT     			0x40
#define	RF_TX_QUE_OPT_SUPER  			0x80


#define	RF_TX_QUE_ACT_IN_PROCESS		0x01	// Local frame sending in progress. Will not switch to next local frame until former frame is sent out
#define	RF_TX_QUE_ACT_SENT_PNEXT		0x02


#pragma pack (1)

#define RF_MAX_LEN       255

typedef	struct
{
	u16		uiSrc_Addr;			// ulSrc_Addr+ucSeq identifies a unique frame
	u16		uiDst_Addr;			// for ACK handling
	u8		ucLength;			// ucLength = 0 means empty
	u8		ucSeq;				// equal to MAC_FRAME.ucSeq, for ACK processing purpose
	u8		ucTried_Times;
	u8		ucOption;
	u16		uiRetry_Gap;		// gap to next retry send
	u16		uiLast_Try_Time;	// if never tried to send before, i.e., ucTried_Times==0, this is the load time
	u8		ucActive;			// for local issued frame only. none zero means this frame is in sending progress
	u8		aucData[RF_MAX_LEN];
//	u8		ucCheck;			// check sum, to provend ram crush
}	RF_TX_QUE;


typedef	struct
{
	u8  ucLen;
	s8  scSnr;
	s16 siRssi;
//	u8* pPayload;
	u8  aucPayload[RF_MAX_LEN];
	
}RF_Rx_Typedef;

#define SUPER_DATA_MAX_LEN                          255
#define PHY_MAX_LEN                                 255
#define PHY_MAX_PAYLOAD_LEN                         53
#define SUPER_FRAME_HEAD_LEN                        6
#define SUPER_FRAME_DATA_LEN                        58

#define ACT_IN_IDLE                                 (0x00)
#define ACT_IN_PROCESS                              (0x01)
#define ACT_IN_RX_DONE                              (0x02)
#define ACT_IN_END                                  (0x03)
  

typedef	struct
{
	u8 ucLen;
	u8 ucStatus;
	u8 ucSeq;
	u16 uiDst_Addr;
	u16 uiSrc_Addr;
	u32 ulTime;
	u8 auSuperDataBuffer[SUPER_DATA_MAX_LEN];
	
}Super_Data_Typedef;

typedef	struct
{
	u8 ucFrameType;
	u8 ucTotalLen;
	u8 ucTotalNum;
	u8 ucLen;
	u8 ucNum;
	u8 auSuperData[SUPER_FRAME_DATA_LEN];
	u8 ucCRC;
	
}Super_Frame_Typedef;

//#define MAX_PARE_TBL    (10)

typedef struct 
{
	u8       ucTry_Join;
	u8       ucDev_Type;            //·ÀÖ¹ÎÞÏÞÕðµ´
	s16      siRssi;
	u16      uiPare_Addr;     // parenet nwkaddr
	Dev_Addr sPare_Addr;
	
} Pare_Table_Typedef;


typedef struct 
{
	Dev_Addr sDst_Addr;
	Dev_Addr sSrc_Addr;           //parent saddr
	u8       ucLoRa_Cmd;
	u8       ucDev_Type;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucPID;               // ucSer_Seq
	u16      uiPare_Addr;     // parenet nwkaddr
	
} Beacon_Resp_Typedef;

typedef struct
{
	Dev_Addr sDst_Addr;
	Dev_Addr sSrc_Addr;
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucPID;               // ucSer_Seq
	u8       ucDev_Type;     // req:dev_type  resp: nwkaddr
	
}LoRa_Join_Typedef;

typedef struct
{
	Dev_Addr sDst_Addr;
	Dev_Addr sSrc_Addr;
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucPID;               // ucSer_Seq
	u16      uiDev_Addr;     //dev nwkaddr	
	u16      uiPare_Addr;     // parenet nwkaddr
	
}Join_Resp_Typedef;

typedef struct
{
	
	u16      uiPare_Addr;
	u16      uiDev_Addr;
	u8       ucPID;
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucSer_Seq;

}Resp_Ack_Typedef;



#define RELAY_JOIN_LEN       (29)
#define RELAY_RESP_LEN       (30)


typedef struct
{
	Dev_Addr sDst_Addr;
	Dev_Addr sSrc_Addr;	
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	s8       cRssi;	
	Dev_Addr sPre_Addr;       //relay node addr	
	u8       ucDev_Type;
	
}Relay_Join_Typedef;


typedef struct
{
	Dev_Addr sDst_Addr;
	Dev_Addr sSrc_Addr;	
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucPID;	
	Dev_Addr sPre_Addr;       //relay node addr	
	u16      uiDst_Addr;
	
}Relay_Resp_Typedef;

#define LORA_PAYLOAD_MAX_LEN     (244)// 255-2-9    //(100 - 9)

typedef struct
{
//	u8       ucFrame_Ctrl;       //0x11,0x02 //bit0-bit1,1:data/2:ack,  bit7,1:ack req,0:no ack req
	u16      uiDst_Addr;
	u16      uiSrc_Addr;
	u8       ucPID;
	u8       ucLoRa_Cmd;
	u8       ucHop;            //·ÀÖ¹ÎÞÏÞÕðµ´
	u8       ucDev_Seq;
	u8       ucSer_Seq;
	u8       ucPayload[LORA_PAYLOAD_MAX_LEN];
//	u8       ucCheck;
	
}LoRa_Data_Typedef;


#define RT_PAYLOAD_MAX_LEN      52//64-2-10
#define RT_PAYLOAD_HEAD_LEN     10//64-2-10

typedef struct
{
	Dev_Addr sSrc_Addr;
	u8       ucLoRa_Cmd;
	u8       ucSeq;
	u8       ucPayload[RT_PAYLOAD_MAX_LEN];
	
}LoRa_RT_Data_Typedef;



#define ACK_LEN           (2)

typedef struct
{
	u8       ucFrame_Ctrl;       //0x11,0x02 //bit0-bit1,1:data/2:ack,  bit7,1:ack req,0:no ack req
//	u16      uiDst_Addr;
//	u16      uiSrc_Addr;
	u8       ucDev_Seq;
//	u8       ucCheck;
	u8       ucPend;
	u32      ultick;
	
}ACK_Typedef;


//#define		POLL_BUFFER_SIZE		10
#define		POLL_MESSAGE_MAX_SIZE	255//64

typedef struct
{
	Dev_Addr sChild_Addr;
	u8		ucAge;					// in seconds
	u8		ucMessage_Length;
	u8		aucMessage[POLL_MESSAGE_MAX_SIZE];
	
}	POLL_BUFFER_Typedef;

#define IDLE_STATUS          0x00
#define POLL_WAITING_ACK     0x01
#define POLL_RESET           0x02
#define POLL_RETRY           0x03
#define POLL_TIMEOUT         0x04


//#define MAX_FAIL_TIMES       0x03


typedef struct
{

	u8		ucSeq;					// in seconds
	u8		ucStatus;
	u8		ucFail_Time;
	
}	POLL_Resp_Ack_Typedef;

#pragma pack ()


/////////////////  timer delay ////////////////////
//#define LED_PERIOD_MS               500
//#define JOIN_START_DELAY            5000
//#define JOIN_PERIOD_S               5000//2000
//#define JOIN_DELAY_S                2000
//#define JOIN_TIMEOUT_S              30000//(1000*30)

#define PERIOD_S                    1000//(HW_RTC_ms2Tick(1000))//1000//(1000*60)
//#define SUPER_FRAME_TIMEOUT         5000//3000//(HW_RTC_ms2Tick(3000))//3000

//#define POLL_GAP_MIN                15//60 //      100 node 60s,  50 node 30s
//#define POLL_GAP_S                  15 //      100 node 60s,  50 node 30s

//#define HEARTBEAT_GAP_MIN           30        //100 node 120s, 50 node 60s
//#define HEARTBEAT_GAP_S             120//60    100 node 120s, 50 node 60s




//////  extern para  /////////////xt	
extern const u16 COO_NwkAddr;
extern const Dev_Addr BroadCastAddr;
extern const u16 BroadAddr;
extern const Dev_Addr NullAddr;

 
void AwarePro_Init(const FuncReg_t* Func);

void AwarePro_NV_Init(u8 Poll_Gap, u8 HB_Gap, u32 ulJoinDelay, Dev_Addr* pAddr);//(u8 Poll_Gap, u8 HB_Gap, u32 RandSeed)

void AwarePro_Run(u32 ulSysTime);

void AwarePro_Join(u32 ulSysTime);

void AwarePro_ReJoin(void);

void AwarePro_Leave(u32 ulSysTime);

void AwarePro_Cancel_Poll(void);

u8 AwarePro_DevStatus(void);

u8 AwarePro_JoinStatus(void);

u8 AwarePro_NV_JoinFlg(void);

u8 AwarePro_Default(void);

u8 AwarePro_GetNodeType(void);

u16 AwarePro_GetNwkAddr(void);

u16 AwarePro_GetPareAddr(void);

u16 AwarePro_GetRSSI(void);

u8 AwarePro_GetVerion(void);

u8 AwarePro_ExtAddrLookup(u16 uiNwkAddr, Dev_Addr* ExtAddr);

u8 AwarePro_NwkAddrLookup(Dev_Addr* ExtAddr, u16* uiNwkAddr);

Dev_Addr* AwarePro_GetHostExtAddr(void);

Dev_Addr* AwarePro_GetLocalExtAddr(void);

u8 AwarePro_GetPID(void);

u8 AwarePro_SetPID(u8 ucPID);

u8 AwarePro_GetPGap(void);

u8 AwarePro_SetPGap(u8 ucPoll_Gap);

u8 AwarePro_GetHBGap(void);

u8 AwarePro_SetHBGap(u8 ucHB_Gap);

u8 AwarePro_ZED_Poll_Once(void);

u8 AwarePro_Get_Poll_Status(void);

u8 AwarePro_UART_Rec_Process(u8* pucData, u8 ucLen, u32 ulSysTime);

u8 AwarePro_RF_Rec_Process(u8* pucData, u8 ucLen, s16 siRssi, s8 scSnr, u32 ulSysTime);

u8 AwarePro_RF_Tx_RT_Process(u8* pSendFrame, u8 ucLen, u32 ulSysTime);

u8 AwarePro_RF_Tx_Process(u8* pSendFrame, u8 ucLen, u16 uiDstAddr, u8 ucAck, u32 ulSysTime);

u8 AwarePro_RF_Config(u16 uiDstAddr, CMD_Enum Cmd, u8* CmdValue, u8 ucLen, u8 ucOpt, u32 ulSysTime);

u8 AwarePro_Period_Event(u32 ulTime);

u8 AwarePro_Create_PID(u32 RandSeed);


//////////// rx tx process  /////////////
void AwarePro_Tx_Done_Process(void);

void AwarePro_Tx_Timeout_Process(void);

void AwarePro_Rx_Done_Process(void);

void AwarePro_Rx_Timeout_Process(void);

void AwarePro_Rx_Error_Process(void);


////////// addr  //////////
u8	Is_Local_Addr( Dev_Addr* Addr );

u8	Is_Local_NwkAddr( u16 NwkAddr );

u8 Is_Dup_Frame(u8* ucFrame);



/// 0 Poll Req;
u8 Is_ZED_PollReq(void);

/// 0 Rf Tx pending
u8 Is_RF_Tx_Pending(void);

//0 joining
u8 Is_Joining(void);

/// 0 Ack pending
u8 Is_Ack_Pending(void);

u8 Is_Cmd_Resp_Pending(void);

u8 Update_Parent_RSSI(u16 uiSrcAddr, s16 siRssi);

void Handle_Error_Process(u32 ulSysTime);

u32 rand1( void );

void srand1( u32 seed );

u32 randr( u32 min, u32 max );

//void NV_Init(void);


//u8 Process_AT_CMD(u8* data, u8 ucLen);


u8 Poll_RF_Buffer(void);

//void Poll_RF_Status(void);

//////////////  superframe  ////////////
//void Store_SuperFrame(u8* Data, u8 Len);

//u8 Process_SuperFrame(u32 ulSysTime);



/////////join /////////////
u8 Node_Join_LoRa(u32 ulSysTime);

u8 Find_Best_Parent(void);

u8 Relay_Join_Resp(Relay_Join_Typedef* JoinData, u16 NwkAddr, u32 ulSysTime);

u8 Join_Resp(LoRa_Join_Typedef* JoinData, u16 NwkAddr, u32 ulSysTime);

u8 Relay_Join_Cmd(LoRa_Join_Typedef* JoinData, s8 JoinRssi, u32 ulSysTime);

u8 Process_Join_Req(Join_Resp_Typedef* LoRaData, s8 scRssi);

u8 Beacon_Req(u32 ulSysTime);

u8 Beacon_Resp(LoRa_Join_Typedef* Beacon, u32 ulSysTime);

void Join_Req( u32 ulSysTime );

u8 Period_Poll_Data(u32 ulSysTime);

u8 Accept_Join_LoRa(u8* RxData, s8 Rssi, u32 ulSysTime);

u8 Router_Join_Resp(Relay_Resp_Typedef* JoinData, u32 ulSysTime);

u8 Process_Join_Cmd(u8* JoinData, u8 len, s16 rssi, u32 ulSysTime);

u8 ZED_Data_Req(u8 ucOpt, u32 ulSysTime);

u8 Period_Poll_Data(u32 ulSysTime);

u8 ZED_Poll_Resp(LoRa_Data_Typedef* PollData, s8 cRssi, u32 ulSysTime);

u8 ZED_Poll_Report(LoRa_Data_Typedef* PollData, u32 ulSysTime);

u8 Router_Test_Report(u32 ulSysTime);

u8 Router_HeartBeat_Report(u32 ulSysTime);

u8 Dev_Leave_Req(u32 ulSysTime);

u8 ZED_DataReq_Event(u32 ulSysTime);

u8 Add_Poll_Msg(LoRa_Data_Typedef* LoRaData, u8 len);

void Period_Update_Event(u32 ulSysTime);

void Poll_Buffer_Age_Increase( void );

void RoutTab_Age_Increase( void );

u8 Process_Ack(ACK_Typedef* LoRaData, u32 ulSysTime);

u8 ACK_Send(u8 ucSeq, u32 ulSysTime);

u8 RF_Tx_Inc(void);

u8 Relay_RF_Data(u8* RF_Data);

u8	TX_RF_Data( u32 ulSysTime );

u8	Add_RF_Frame_To_TX_Que( u8 * pFrame, u8 ucLength, u8 ucOpt, u32 ulSysTime, u16 uiDst_Addr, u16 uiSrc_Addr, u8 ucSeq);

u8 Process_App_Cmd(LoRa_Data_Typedef* LoRaData, u16 size, s16 rssi, s8 snr, u32 ulSysTime);

u8 Process_Cmd(u8 ucCmd, u8* CmdValue, u8 ucLen,  u8 ucOpt );

u8 Get_Empty_RoutTable(Dev_Addr* Addr);

u8	Get_Node_Entry( Dev_Addr* Addr );

u8	Get_NwkAddr_Entry( u16 NwkAddr );

u8	Get_PareNode_Entry( Dev_Addr* Addr );

u8 Get_Empty_PareTable(Dev_Addr* Addr);

u8 Poll_Empty_RoutTbl(void);

u8 Add_Node_RouteTbl(Dev_Addr* sNodeAddr, u16 uiNodeAddr, s8 Rssi, u8 ucDev_Type);

u8 Update_NwkAddr_Table(u16 Addr, s16 rssi);

u8 Build_LoRa_App_Frame(u8* pRawData, u8 ucLen, u16 uiDstAddr, u8 ucAck, u8* pFramebuf);

u8 Build_LoRa_RT_Frame(u8* pRawData, u8 ucLen, u8* pFramebuf);

u8 Build_MAC_Frame(u8* Nwk_Frame, u8 len, u8 Frame_Type, u8* MAC_Frame);

u8 Build_Cmd_Resp(LoRa_Data_Typedef* CmdData, u8* pucAppData, u8 ucAppLen, u8* Respbuf);

u8 Build_Cmd_Req(u16 uiDstAddr, u8 ucCmd, u8* pucVal, u8 ucLen, u8* pFramebuf);

u8 Build_Zero_Poll_Resp(LoRa_Data_Typedef* PollData);

u8 Check_NV_Para(void);

u8 Check_Sum(u8* buf, u8 len);



/////////////  user add code for NV ////////////////////////////////
u8 Write_NV(void);

u8 Read_NV(void);

u8 Delete_NV(void);

u8 Add_Node_To_Flash(u8 Index);

u8 Read_One_Node_From_Flash(u8 Index);

u8 Del_Node_From_Flash(u8 Index);

u8 Read_Node_From_Flash(void);

u8 Erase_NodeTable(void);



#ifdef __cplusplus
}
#endif


#endif

