/*

Description: AwareTag RF Protocol and API 

License: 

Maintainer: Hangzhou AwareTag tech
*/
 /******************************************************************************
  * @file    Aware_Protocol.c
  * @author  dozen Yao
  * @version V0.0.1
  * @date    2017/09/21
  * @brief   RF Protocol and API
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "Aware_Protocol.h"

//#include "Flash.h"


//u8 Join_Status = 0;
//u32 JoinDelay = 0;

//u8 DevType = 0;

//u8 ucPacketSeq = 0;

//u8 ucLoRaStatus = 0;   

//u8	ucLast_TX_Que_Pos	= RF_TX_QUE_SIZE;	
//Dev_Addr* pLocalAddr = NULL;

Sys_Para_Typedef SysPara = 
{
	0,     //JoinStatus;
	0,     //RunStatus;
	0,     //DevType;
	0,     //Seq;
	RF_TX_QUE_SIZE,     // LastTxPos
	0,      //PollReq
	0,      //JoinDelay
	NULL,   //LocalAddr
	
};

u8 ucPreJoinStatus = 0;

u8 RadioSendBuffer[MAX_PACK_SIZE];

Super_Data_Typedef SuperData;

Super_Data_Typedef RxSuperData;

POLL_Resp_Ack_Typedef Poll_Ack;

Pare_Table_Typedef PareTbl[MAX_PARE_TBL];

RF_TX_QUE asRF_TX_Que[RF_TX_QUE_SIZE];

Handle_Error_Typedef HandleError = {0};

ACK_Typedef Ack;

FuncReg_t* Func_t = NULL;


NV_Para_Typedef Nv_Para = {0};

#if defined(COO)|| defined(Router)

Router_Table_Typedef RoutTable[MAX_ROUTABLE] = {0};
POLL_BUFFER_Typedef asPoll_Buffer[POLL_BUFFER_SIZE] = {0};


#endif


const Dev_Addr BroadCastAddr = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const Dev_Addr NullAddr = {0};
const u16 COO_NwkAddr = 0x0000;
const u16 BroadAddr = 0xFFFF;




void AwarePro_Init(const FuncReg_t* Func)
{

	if(Func == NULL)
	{
		#if defined(DEBUG)
		PRINTF("Func is NULL!\r\n");
		#endif

		while(1);
	}
	else
	{
		Func_t = (FuncReg_t*)Func;

/*
		if(Func_t->DeviceAddr != NULL )
		{

//			memcpy(&DeviceAddr, Func_t->DeviceAddr, sizeof(Dev_Addr));
			
		}
		else
		{
						
			#if defined(DEBUG)
						
			PRINTF("Device MAC Addr or SN is NULL!\r\n");
			
			#endif
			
			while(1);

		}
*/

		if((Func_t->RF_Rec == NULL)
			||(Func_t->RF_Tx == NULL)
			||(Func_t->Error == NULL)
			||(Func_t->Para_Init == NULL))
		{
			
			#if defined(DEBUG)
			PRINTF("Loss CallBack!\r\n");
			#endif
		
			while(1);
		}
	}
	
//	AwarePro_NV_Init(Func_t->ucPollGap, Func_t->ucHBGap, Func_t->JoinDelay);
	
//	if(Func_t->JoinDelay != 0)
//	{

//		Func_t->JoinDelay = Func_t->JoinDelay%5000 + 1;
//	}
//	else
//	{

//		
//		Func_t->JoinDelay  = JOIN_DELAY_S;
//	#if defined(DEBUG)
//		
//		PRINTF("JoinDelay is NULL!\r\n");
//	#endif
//	}
			
	#if defined(COO)
		SysPara.ucDevType = COO_TYPE;
		
	#elif defined(Router)
		SysPara.ucDevType = ROUTER_TYPE;

	#elif defined(ZED)
		SysPara.ucDevType = ZED_TYPE;
	
	#endif

}


void AwarePro_NV_Init(u8 Poll_Gap, u8 HB_Gap, u32 ulJoinDelay, Dev_Addr* pAddr)//(u8 Poll_Gap, u8 HB_Gap, u32 RandSeed)
{

	u8 Init_Flg = 0;


	#if defined(NV)
		
	Read_NV();   //from flash
	
//	if((0 == Nv_Para.ucNv_Flag )||(NO_PARA == Nv_Para.ucNv_Flag ))  //eeprom :0,flash : 0xff
	if(Check_NV_Para())
	{
		Delete_NV();
		
		if(Poll_Gap >= POLL_GAP_MIN)
		{
			
			Nv_Para.ucPoll_TimeOut = Poll_Gap;	
		}
		else
		{
			Nv_Para.ucPoll_TimeOut = POLL_GAP_MIN;
		}

		if((HB_Gap == 0)||(HB_Gap >= HEARTBEAT_GAP_MIN))
		{
			
			Nv_Para.ucHeartBeat_Gap = HB_Gap;
		}
		else
		{			
			Nv_Para.ucHeartBeat_Gap = HEARTBEAT_GAP_MIN;
		}

		Init_Flg = 1;
	}
	else
	{
		
		if(AwarePro_NV_JoinFlg() != NET_FLG)	 //creat network
		{			
			Init_Flg = 1;
		}
		else                                      //have join Network
		{
			
			#if defined(COO) || defined(Router)
						
			Read_Node_From_Flash();
		
			#endif

		}
		
	}
	
	#else

		if(Poll_Gap >= POLL_GAP_MIN)
		{
			
			Nv_Para.ucPoll_TimeOut = Poll_Gap;	
		}
		else
		{
			Nv_Para.ucPoll_TimeOut = POLL_GAP_MIN;
		}

		if((HB_Gap == 0)||(HB_Gap >= HEARTBEAT_GAP_MIN))
		{
			
			Nv_Para.ucHeartBeat_Gap = HB_Gap;
		}
		else
		{			
			Nv_Para.ucHeartBeat_Gap = HEARTBEAT_GAP_MIN;
		}
		
		Init_Flg = 1;

	#endif

	if(Init_Flg)
	{
		Init_Flg = 0;
		#if defined(COO)
	
			Nv_Para.ucJoin_Flag  = NET_FLG;
			Nv_Para.uiNwkAddr = COO_NwkAddr;
			Nv_Para.ucPID     = AwarePro_Create_PID(ulJoinDelay);
			Nv_Para.uiPare_Addr  = COO_NwkAddr;
			memcpy((u8*)&Nv_Para.sHost_Addr, (u8*)pAddr, sizeof(Dev_Addr));

			#if defined(NV)
			Write_NV();
			#endif
			
		#else

			SysPara.ucJoinStatus = Join_Init;
			Nv_Para.ucJoin_Flag  = NO_NET_FLG;
			Nv_Para.uiNwkAddr = 0;
			Nv_Para.ucPID	  = 0;
			Nv_Para.uiPare_Addr  = 0;
			memcpy((u8*)&Nv_Para.sHost_Addr, (u8*)&NullAddr, sizeof(Dev_Addr));

			
		#endif
	}

	if(ulJoinDelay != 0)
	{

		SysPara.ulJoinDelay = ulJoinDelay % 5000 + 1;  // 1-5000ms
	}
	else
	{	
		SysPara.ulJoinDelay  = JOIN_DELAY_S;
	#if defined(DEBUG)
		
		PRINTF("JoinDelay is NULL!\r\n");
	#endif
	}
	
	
	if(pAddr != NULL )
	{
		SysPara.pLocalAddr = pAddr;
	}
	else
	{
					
		#if defined(DEBUG)
					
		PRINTF("Device MAC Addr or SN is NULL!\r\n");
		
		#endif
		
		while(1);

	}

}


void AwarePro_Run(u32 ulSysTime)
{
	TX_RF_Data(ulSysTime);
	Process_SuperFrame(ulSysTime);
	AwarePro_Period_Event(ulSysTime);	

}

// ulTime  -- systime
// JoinDelayTime --- start beacon time    

void AwarePro_Join(u32 ulSysTime)
{
	#if defined(Router) || defined(ZED)
	
	static u32 ulPreTime = 0;
	static u32 ulStartTime = 0;
//	static u8 ucPreStatus = 0;

	
	if((Nv_Para.ucJoin_Flag == NET_FLG)&&(SysPara.ucJoinStatus != Leave))
	{
		if(SysPara.ucJoinStatus == 0)	
		{
			SysPara.ucJoinStatus = Join_Success;
			Func_t->Join_Status((Join_Status_Enum)SysPara.ucJoinStatus);
		}
				
		return;
	}

	if((ucPreJoinStatus == Leave)
		||(ucPreJoinStatus == Join_TimeOut))
	{
		return;
	}

	if(ucPreJoinStatus != SysPara.ucJoinStatus)
	{
		ucPreJoinStatus = SysPara.ucJoinStatus;
		Func_t->Join_Status((Join_Status_Enum)SysPara.ucJoinStatus);
	}
	
	if((SysPara.ucJoinStatus == Join_Init)
		||(SysPara.ucJoinStatus == Rejoin_req))
	{
		ulPreTime = ulSysTime;	
		memset(PareTbl, 0x00, MAX_PARE_TBL*sizeof(Pare_Table_Typedef));
		SysPara.ucJoinStatus = Beacon_req;
	}
	else if(SysPara.ucJoinStatus == Beacon_req)
	{
		if((u32)(ulSysTime - ulPreTime) > SysPara.ulJoinDelay)   //Func_t->JoinDelay
		{
			ulPreTime = ulSysTime;
			ulStartTime = ulSysTime;
			Beacon_Req(ulSysTime);
			SysPara.ucJoinStatus = Retry_Beacon_req;

			#if defined(DEBUG)
			PRINTF("Beacon Req!\r\n");
			#endif
		}
	}
	else if(SysPara.ucJoinStatus == Retry_Beacon_req)
	{
		if((u32)(ulSysTime - ulStartTime) > JOIN_TIMEOUT_S)
		{
			SysPara.ucJoinStatus = Join_TimeOut;
			
			#if defined(DEBUG)
			PRINTF("Join TimeOut!\r\n");
			#endif
			///Callback devstatus
		
			Func_t->Join_Status((Join_Status_Enum)SysPara.ucJoinStatus);

		}
		else if((u32)(ulSysTime - ulPreTime) > JOIN_PERIOD_S)
		{
			ulPreTime = ulSysTime;
			Beacon_Req(ulSysTime);
		
			#if defined(DEBUG)
			PRINTF("Beacon Req!\r\n");
			#endif
//			Join_Status = JOIN_STATUS_RETRY_BECON_REQ;
		}
	}
	else if(SysPara.ucJoinStatus == Join_req)  //rec beacon resp
	{
		if((u32)(ulSysTime - ulPreTime) > JOIN_DELAY_S)
		{
			ulPreTime = ulSysTime;
			Join_Req(ulSysTime );

			SysPara.ucJoinStatus = Retry_Beacon_req;
			
//			PRINTF("Join Req!\r\n");
		}
	}	
	else if(SysPara.ucJoinStatus == Join_Success)
	{
		Nv_Para.ucJoin_Flag = NET_FLG;

		#if defined(NV)

		Write_NV();
		
		#endif
//		Func_t->Join_Status(SysPara.ucJoinStatus);
	}
	else if(SysPara.ucJoinStatus == Leave)
	{
	
		Nv_Para.ucJoin_Flag = 0;
//		Nv_Para.uiNwkAddr   = 0x0000;
//		memset((u8*)&Nv_Para.sHost_Addr, 0x00, sizeof(Dev_Addr));	

//		memset((u8*)&Nv_Para, 0x00, sizeof(Nv_Para));		
		memset((u8*)&Poll_Ack, 0x00, sizeof(Poll_Ack));

		#if defined(NV)
//		Write_NV();
		Delete_NV();
		#endif

		SysPara.ucRespFlag = FALSE;
	
	}
	else if(SysPara.ucJoinStatus == Join_TimeOut)  //rec join resp
	{		
		
		
	
	}
	else
	{
	
	}

	#endif
	
}

void AwarePro_ReJoin(void)
{
	ucPreJoinStatus = Join_Init;
	SysPara.ucJoinStatus = Join_Init;
	Nv_Para.ucJoin_Flag = NO_NET_FLG;

	#if defined(DEBUG)
	Delete_NV();
	#endif
}


void AwarePro_Leave(u32 ulSysTime)
{
	Dev_Leave_Req(ulSysTime);
	SysPara.ucRespFlag = TRUE;
}

void AwarePro_Cancel_Poll(void)
{
	Poll_Ack.ucStatus = IDLE_STATUS;
}

// 1 mean can enter stop mode
u8 AwarePro_DevStatus(void)
{
//	if((RF_TX_QUE_SIZE == Poll_RF_Buffer())
//	&&( Poll_Ack.ucStatus == 0)
//	&&( SysPara.ucPollReq == 0)
//	&&(Ack.ucPend == FALSE)
//	&&(SysPara.ucJoinStatus != JOIN_STATUS_SENT_BCON_REQ)
//	&&(SysPara.ucJoinStatus != JOIN_STATUS_SENT_JOIN_REQ)
//	&&(SysPara.ucJoinStatus != JOIN_STATUS_RETRY_BECON_REQ))

	if((Is_RF_Tx_Pending())
		&&(Is_ZED_PollReq())
		&&(Is_Ack_Pending())
		&&(Is_Joining())
		&&(Is_Cmd_Resp_Pending()))
	{
	
		SysPara.ucRunStatus = 1;
	
	}
	else
	{
		
		SysPara.ucRunStatus = 0;
	}
	
	return SysPara.ucRunStatus;
}

//init mode, join success, timeout
u8 AwarePro_JoinStatus(void)
{	
	return SysPara.ucJoinStatus;
}

u8 AwarePro_NV_JoinFlg(void)
{	
	return Nv_Para.ucJoin_Flag;
}


u8 AwarePro_Default(void)
{
	SysPara.ucJoinStatus  = 0;
	SysPara.ucRunStatus = 0;
	SysPara.ucDevType      = 0;
//	JoinStartTime= 0;
	
	memset((u8*)&Nv_Para, 0x00, sizeof(Nv_Para));
	
	#if defined(NV)
			
		Delete_NV();
		
		#if defined(COO)
				
			Erase_NodeTable();
						
			#elif defined(Router)

			Erase_NodeTable();
					
		#else

		#endif
		
	#endif
	
	return 0;
}

u8 AwarePro_Create_PID(u32 RandSeed)
{
	u32 Pid = 0;

	srand1(RandSeed);
	//1-255
	Pid = randr(1, 255);
//	Pid = Radio.Random()%255 + 1;

	return ((u8)Pid);
}

u8 AwarePro_GetNodeType(void)
{
	return (SysPara.ucDevType);
}

u16 AwarePro_GetNwkAddr(void)
{
	return (Nv_Para.uiNwkAddr);
}

u16 AwarePro_GetRSSI(void)
{
	return (Nv_Para.scRSSI);
}

u16 AwarePro_GetPareAddr(void)
{
	return (Nv_Para.uiPare_Addr);
}

Dev_Addr* AwarePro_GetLocalExtAddr(void)
{
	return (SysPara.pLocalAddr);
}

Dev_Addr* AwarePro_GetHostExtAddr(void)
{
	return (&Nv_Para.sHost_Addr);
}

u8 AwarePro_ExtAddrLookup(u16 uiNwkAddr, Dev_Addr* ExtAddr)
{
	#if defined(COO) || defined(Router)
	
	u8 i;

	i = Get_NwkAddr_Entry(uiNwkAddr);

	if(MAX_ROUTABLE > i)
	{
		memcpy(ExtAddr, &RoutTable[i].sNode_Addr, sizeof(Dev_Addr));
	}
	else
	{
		#if defined(DEBUG)
			PRINTF("Not find Node!\r\n");
		#endif
		
		return 1;
	}

	#endif

	return 0;
	
}

u8 AwarePro_NwkAddrLookup(Dev_Addr* ExtAddr, u16* uiNwkAddr)
{
	#if defined(COO) || defined(Router)
	
	u8 i;

	i = Get_Node_Entry(ExtAddr);

	if(MAX_ROUTABLE > i)
	{
		memcpy(uiNwkAddr, &RoutTable[i].uiAddr, sizeof(u16));
	}
	else
	{
		#if defined(DEBUG)
			PRINTF("Not find Node!\r\n");
		#endif
		
		return 1;
	}

	#endif

	return 0;
	
}

u8 AwarePro_GetVerion(void)
{
	return (PRO_VER);
}

u8 AwarePro_GetPID(void)
{
	return (Nv_Para.ucPID);
}

u8 AwarePro_SetPID(u8 ucPID)
{

	if(ucPID != 0)
	{		
		Nv_Para.ucPID = ucPID;
		
		#if defined(NV)

		Write_NV();
		
		#endif
		
		return 0;
	}
	else
	{
		return 1;
	}

	
}

u8 AwarePro_GetPGap(void)
{	
	return (Nv_Para.ucPoll_TimeOut);
}

u8 AwarePro_SetPGap(u8 ucPoll_Gap)
{
	Nv_Para.ucPoll_TimeOut = ucPoll_Gap;

	#if defined(NV)

	Write_NV();
	
	#endif
	
	return 0;
}

u8 AwarePro_GetHBGap(void)
{	
	return (Nv_Para.ucHeartBeat_Gap);
}

u8 AwarePro_SetHBGap(u8 ucHB_Gap)
{
	Nv_Para.ucHeartBeat_Gap = ucHB_Gap;

	#if defined(NV)

	Write_NV();
	
	#endif
	
	return 0;
}

u8 AwarePro_ZED_Poll_Once(void)
{
	SysPara.ucPollReq  = TRUE;
	
	return 0;
}

u8 AwarePro_Get_Poll_Status(void)
{

	return SysPara.ucPollReq;
}


/*
u8 AwarePro_UART_Rec_Process(u8* pucData, u8 ucLen, u32 ulSysTime)
{
//	if(Process_AT_CMD(pucData, ucLen))
//	if(UART_AT_CMD_Process(pucData))
	{
			
//			PRINTF("len:%d\r\n",asUART0_Buffer[1].ucLength);
//			
//			for(i=0; i<asUART0_Buffer[1].ucLength; i++)
//		    {
//		        PRINTF("%02x ", asUART0_Buffer[1].aucData[i]);
//		    }
//			PRINTF("\r\n");

		u8 i = 0;			
		LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)(pucData);
		LoRa->ucDev_Seq = ucPacketSeq;

		#if defined(COO)
			
		if(Add_Poll_Msg(LoRa, ucLen))  //0 add success,poll msg
		{
			if(COO_MAX_ROUTABLE > Get_NwkAddr_Entry(LoRa->uiDst_Addr))
			{
				if(ucLen <= (PHY_MAX_LEN - 2))
				{
					
					i = Build_MAC_Frame(pucData, ucLen, DATA_ACK_TYPE, RadioSendBuffer);

					Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
	
//						PRINTF("Seq:%d,len:%d\r\n",ucPacketSeq, asUART0_Buffer[1].ucLength);
				}
				else
				{
					/// CPY TO superFramebuffer
					Store_SuperFrame(pucData, ucLen);
				}

			}
			
		}
			
		#else

		if((Nv_Para.uiPare_Addr == LoRa->uiDst_Addr)&&(Nv_Para.uiNwkAddr == LoRa->uiSrc_Addr))
		{
			
			if(ucLen <= (PHY_MAX_LEN - 2))
			{
				i = Build_MAC_Frame(pucData, ucLen, 
									DATA_ACK_TYPE, RadioSendBuffer);
		
				Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
			}
			else
			{
				#if defined(Router)	
				Store_SuperFrame(pucData, ucLen);

				#else
//				Uart_Send(pucData, ucLen);
				#endif
			}

			
		}
		else if(Nv_Para.uiNwkAddr == LoRa->uiDst_Addr)    //send self
		{
			Process_App_Cmd(LoRa, ucLen, 0, 0, ulSysTime);
		}
		else       //auto add protocol head
		{

			if(Nv_Para.ucJoin_Flag == NET_FLG)
			{
				
				AwarePro_RF_Tx_Process(pucData, ucLen, ulSysTime);
			}

		}
			

		#endif
			
				
	}

	return 0;
}
*/

u8 AwarePro_RF_Rec_Process(u8* pucData, u8 ucLen, s16 siRssi, s8 scSnr, u32 ulSysTime)
{

	if(ACK_TYPE == pucData[0])
	{
		ACK_Typedef* Ack = (ACK_Typedef*)pucData;

		Process_Ack(Ack, ulSysTime);

	}
	else if(EXT_PID_TYPE == (pucData[0]&EXT_PID_TYPE))
	{
		
		LoRa_RT_Data_Typedef* LoRaData = (LoRa_RT_Data_Typedef*)(&pucData[1]);
		
		
		if(Is_Local_Addr(&LoRaData->sSrc_Addr))
		{
			return 1;
		}

		#if defined(COO)

		static Dev_Addr sSrc_Addr = {0};
		static u8 Seq             = 0;
		
		if(ACK_BIT == (pucData[0]&ACK_BIT))
		{
			
			ACK_Send(LoRaData->ucSeq, ulSysTime);//(&pucData[1]);	
		}
		
		if(SysPara.ucDevType == COO_TYPE)
		{
			if((Seq != LoRaData->ucSeq)
				||(memcmp(&sSrc_Addr, &LoRaData->sSrc_Addr, sizeof(Dev_Addr)) != 0))      //dup frame not callback
			{
				memcpy(&sSrc_Addr, &LoRaData->sSrc_Addr, sizeof(Dev_Addr));
				Seq = LoRaData->ucSeq;
				
				Func_t->RF_RT_Rec(LoRaData->ucPayload, (ucLen - RT_PAYLOAD_HEAD_LEN -2), &LoRaData->sSrc_Addr, ulSysTime);
			}
		}
		
		#endif
		
	}
	else if((DATA_NOACK_TYPE == pucData[0])
		||(DATA_ACK_TYPE == pucData[0]))
	{

		LoRa_Data_Typedef* LoRaData = (LoRa_Data_Typedef*)(&pucData[1]);

		ucLen -= 2;
		if(Is_Local_NwkAddr(LoRaData->uiSrc_Addr))  //addr myself
		{	

			return 1;
		}

		if(LoRaData->ucPID != Nv_Para.ucPID)    //not my network
		{
			
			return 2;
		}
		
		Update_Parent_RSSI(LoRaData->uiSrc_Addr, siRssi);    //update parent rssi;
		
		#if defined(Router)
		if((Is_Local_NwkAddr(LoRaData->uiDst_Addr))
			||(MAX_ROUTABLE > Get_NwkAddr_Entry(LoRaData->uiDst_Addr)))

		#else
		
		if(Is_Local_NwkAddr(LoRaData->uiDst_Addr))
			
		#endif
		{
			if(DATA_ACK_TYPE == pucData[0])
			{
				ACK_Send(LoRaData->ucDev_Seq, ulSysTime);//(&pucData[1]);	
			}

			if(Is_Dup_Frame((u8*)LoRaData))       //add by dozen 20180126
			{
				return 3;
			}
			
			HandleError.TxCnt = 0;      //add by dozen  20170823  clr for my data
		}
		
		#if defined(COO)

		if(Is_Local_NwkAddr(LoRaData->uiDst_Addr))     //local Frame
		{
			
			Update_NwkAddr_Table(LoRaData->uiSrc_Addr, siRssi);
			Process_App_Cmd(LoRaData, ucLen, siRssi, scSnr, ulSysTime);
			
		}

		
		#elif defined(Router)

		//ZED->Router->COO
		if(Is_Local_NwkAddr(LoRaData->uiDst_Addr)
			&&(ROUTER_MAX_ROUTABLE > Get_NwkAddr_Entry(LoRaData->uiSrc_Addr)))   //R+Z+p--->C+Z+p
		{

			Update_NwkAddr_Table(LoRaData->uiSrc_Addr, siRssi);   //clr age
			
			if( POLL_CMD == LoRaData->ucLoRa_Cmd )
			{
				
		//		Uart_Send((u8*)LoRaData, size);
				if( NO_VALID_POLL == ZED_Poll_Resp(LoRaData, siRssi, ulSysTime))
				{
			
					ZED_Poll_Report(LoRaData, ulSysTime);
					
					#if defined(DEBUG)
					PRINTF("Poll Report!\r\n");
					#endif

				}
				
			}
			else if(DEL_RESP_CMD == LoRaData->ucLoRa_Cmd)
			{
				u8 i;

				i = Get_NwkAddr_Entry(LoRaData->uiSrc_Addr);

				memset((u8*)&RoutTable[i], 0x00, sizeof(Router_Table_Typedef));

				#if defined(NV)
//				Update_Node_to_Flash();
				Del_Node_From_Flash(i);
				
				#endif
				
				#if defined(DEBUG)
				PRINTF("Router Delete NodeTbl!\r\n");
				#endif
			}
			else
			{
					
				Relay_RF_Data(&pucData[1]);

				Add_RF_Frame_To_TX_Que(pucData, ucLen + 2, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);
				
				#if defined(DEBUG)
				PRINTF("Relay LoRa Data!\n");
				#endif
			}		
			
		}
		else if(Is_Local_NwkAddr(LoRaData->uiDst_Addr))   //COO TO me or join 
		{

			Process_App_Cmd(LoRaData, ucLen, siRssi, scSnr, ulSysTime);
						
		} 
		//to my child 
		else if(!Add_Poll_Msg(LoRaData, ucLen))	//coo -> Router -> ZED   
		{
			
		}
		else
		{
			
		}
		
		#else  //ZED

		if(Is_Local_NwkAddr(LoRaData->uiDst_Addr))
		{

			Process_App_Cmd(LoRaData, ucLen, siRssi, scSnr, ulSysTime);
			
		}

		#endif
	}
	else if(BEACON_TYPE == pucData[0])
	{

		#if defined(COO) || defined(Router)
		
		if(Nv_Para.ucJoin_Flag  == NET_FLG)
		{

			LoRa_Join_Typedef* Beacon = (LoRa_Join_Typedef*)&pucData[1];
			
			if((Beacon->ucDev_Type == ROUTER_TYPE)&&(SysPara.ucDevType == ROUTER_TYPE))
			{
				
//				LoRaRxBuffer[1].ucLen = 0;
				return 4;
			}
			
			if(Poll_Empty_RoutTbl() < MAX_ROUTABLE)    //accept join Node
			{
				
				Beacon_Resp(Beacon, ulSysTime);

				#if defined(DEBUG)

				PRINTF("Beacon Resp!\r\n");
				#endif
			}
		}

		#endif
	}
	
	else if(BEACON_RESP_TYPE == pucData[0])
	{
		#if defined(Router) || defined(ZED)
		
		Beacon_Resp_Typedef* BeaconResp = (Beacon_Resp_Typedef*)&pucData[1];
		u8 i;

		if(memcmp(&BeaconResp->sDst_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr)))  // !=
		{
			
//			LoRaRxBuffer[1].ucLen = 0;
			return 5;
		}
		
		i = Get_Empty_PareTable(&BeaconResp->sSrc_Addr);
		if(i < MAX_PARE_TBL)
		{
//			PareTbl[i].ucTry_Join = 0;
			PareTbl[i].ucDev_Type  = BeaconResp->ucDev_Type;
			PareTbl[i].siRssi      = siRssi;
			PareTbl[i].uiPare_Addr = BeaconResp->uiPare_Addr;
			memcpy((u8*)&PareTbl[i].sPare_Addr, (u8*)&BeaconResp->sSrc_Addr, sizeof(Dev_Addr));
			
		}

		SysPara.ucJoinStatus = Join_req;
		
		#endif
	}
	
	else if(JOIN_TYPE == pucData[0])
	{
		ucLen -=2;
		
		if((LORA_JOIN_LEN == ucLen)||(JOIN_RESP_LEN == ucLen))
		{
			
			LoRa_Join_Typedef* LoRaJoin   = (LoRa_Join_Typedef*)(&pucData[1]);

			if(Is_Local_Addr(&LoRaJoin->sDst_Addr))
			{
				
				Process_Join_Cmd(&pucData[1], ucLen, siRssi, ulSysTime);
			}
		}

		#if defined(COO) 
		
		else if(RELAY_JOIN_LEN == ucLen)
		{
			
			Relay_Join_Typedef* RelayJoin = (Relay_Join_Typedef*)(&pucData[1]);

			if(Is_Local_Addr(&RelayJoin->sDst_Addr))
			{
				
				Process_Join_Cmd(&pucData[1], ucLen-2, siRssi, ulSysTime);
			}
		}

		#elif defined(Router)
		
		else if(RELAY_RESP_LEN == ucLen)
		{

			Relay_Resp_Typedef* RelayJoin = (Relay_Resp_Typedef*)(&pucData[1]);

			if(Is_Local_Addr(&RelayJoin->sPre_Addr))
			{
				
				Process_Join_Cmd(&pucData[1], ucLen-2, siRssi, ulSysTime);
			}
		}

		#else

		#endif
		
		else
			{
			}

	}
	
	else if(SUPER_BIT == (pucData[0]&SUPER_BIT))     //super frame
	{

		Super_Frame_Typedef* SFrame = (Super_Frame_Typedef*)pucData;


		if(1 == SFrame->ucNum)    //first Frame
		{

			LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)(SFrame->auSuperData);

			if(LoRa->ucPID != Nv_Para.ucPID)
			{
				
				return 2;
			}
			
			if(RxSuperData.ucStatus != ACT_IN_IDLE)   //processing frame
			{
				return 6;
			}

			if(LoRa->uiDst_Addr == Nv_Para.uiNwkAddr)
			{
				RxSuperData.ucLen      = SFrame->ucTotalLen;
				RxSuperData.ucSeq      = LoRa->ucDev_Seq;
				RxSuperData.uiDst_Addr = LoRa->uiDst_Addr;
				RxSuperData.uiSrc_Addr = LoRa->uiSrc_Addr;
				RxSuperData.ucStatus   = ACT_IN_PROCESS;
				RxSuperData.ulTime     = ulSysTime;
//				RxSuperData.ulTime     =  HW_RTC_Tick2ms(HW_RTC_GetTimerValue());
			}
			
		}
		else
		{
			if(RxSuperData.ucStatus == ACT_IN_PROCESS)  //address me
			{
				RxSuperData.ulTime     = ulSysTime;
				
//				RxSuperData.ulTime	   =  HW_RTC_Tick2ms(HW_RTC_GetTimerValue());
			}
			else
			{
				
				return 6;
			}
		}

		if(RxSuperData.uiDst_Addr != Nv_Para.uiNwkAddr)    //not address me
		{
			
			return 5;
		}
		
		if(ACK_BIT == (pucData[0] & ACK_BIT))
		{
//			ACK_Send(&payload[1]);	
//			LoRa_Data_Typedef* LoRaAck = (LoRa_Data_Typedef*)RadioSendBuffer;

//			LoRaAck->uiDst_Addr = RxSuperData.uiDst_Addr;
//			LoRaAck->uiSrc_Addr = RxSuperData.uiSrc_Addr;
//			LoRaAck->ucDev_Seq  = RxSuperData.ucSeq;
			
			ACK_Send(RxSuperData.ucSeq, ulSysTime);//((u8*)LoRaAck);
		}

		// store Rx data to buffer
		memcpy(&RxSuperData.auSuperDataBuffer[(SFrame->ucNum - 1)*SUPER_FRAME_DATA_LEN], 
				SFrame->auSuperData, 
				SFrame->ucLen); 

		if(SFrame->ucTotalNum == SFrame->ucNum)
		{
//			PRINTF("len:%d\r\n", RxSuperData.ucLen);

//			for(i = 0; i < RxSuperData.ucLen; i++)
//			{
//				PRINTF("%02x ", RxSuperData.auSuperDataBuffer[i]);
//			}
//			PRINTF("\r\n");

			LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)RxSuperData.auSuperDataBuffer;

			Update_NwkAddr_Table(LoRa->uiSrc_Addr, siRssi);
			Process_App_Cmd(LoRa, RxSuperData.ucLen, siRssi, scSnr, ulSysTime);

			// SuperDatabuffer clear
			RxSuperData.ucStatus = ACT_IN_END;//add 20170802
		}
		
			
	}
	else
	{
	}
	
	return 0;
}

u8 AwarePro_RF_Tx_RT_Process(u8* pSendFrame, u8 ucLen, u32 ulSysTime)
{

	u8 i = 0;

    i =	Build_LoRa_RT_Frame(pSendFrame, ucLen, RadioSendBuffer);
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, RF_TX_QUE_OPT_RT, ulSysTime, BroadAddr, 0, SysPara.ucPacketSeq);
	
	return 0;
}

u8 AwarePro_RF_Tx_Process(u8* pSendFrame, u8 ucLen, u16 uiDstAddr, u8 ucSerSeq, u32 ulSysTime)
{	
	u8 i = 0;

	if(uiDstAddr == Nv_Para.uiNwkAddr)
	{
		#if defined(DEBUG)
			PRINTF("DstAddr is error!\r\n");
		#endif
		
		return 1;
	}
	
	i = Build_LoRa_App_Frame(pSendFrame, ucLen, uiDstAddr, ucSerSeq, RadioSendBuffer);

	#if defined(COO)
		
		LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)(&RadioSendBuffer[1]);
	
		if(Add_Poll_Msg(LoRa, i-2))  //0 add success,poll msg
		{
		
			if(COO_MAX_ROUTABLE > Get_NwkAddr_Entry(LoRa->uiDst_Addr))   //my child
			{
				if(i <= PHY_MAX_LEN)
				{
					
					Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
	
//						PRINTF("Seq:%d,len:%d\r\n",ucPacketSeq, asUART0_Buffer[1].ucLength);
				}
				else
				{
					/// CPY TO superFramebuffer
					Store_SuperFrame((u8*)LoRa, i-2);
				}

			}
		}

	#else

	if(Nv_Para.ucJoin_Flag != NET_FLG)
	{
		#if defined(DEBUG)
			PRINTF("Dev is offline!\r\n");
		#endif
		
		return 1;
	}

	if((uiDstAddr != Nv_Para.uiPare_Addr)&&(uiDstAddr != BroadAddr))
	{
		#if defined(DEBUG)
			PRINTF("DstAddr is error!\r\n");
		#endif
		
		return 1;
	}

	if(i <= PHY_MAX_LEN)
	{
	
		Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
	}
	else
	{
		#if defined(Router)
		
		Store_SuperFrame(RadioSendBuffer, i);

		#else

		#if defined(DEBUG)
			PRINTF("RF len error!\r\n");
		#endif
		
		return 1;

		#endif
	}

	#endif


	return 0;
}

u8 AwarePro_RF_Config(u16 uiDstAddr, CMD_Enum Cmd, u8* CmdValue, u8 ucLen, u8 ucOpt, u32 ulSysTime)
{
	#if defined(COO)
	
	u8 i = 0;	
	u8 ucZED_Flg = 0;

	if(uiDstAddr == Nv_Para.uiNwkAddr)   // myself
	{
		Process_Cmd(Cmd, CmdValue, ucLen, ucOpt);
	}
	else
	{
	
		i = Get_NwkAddr_Entry(uiDstAddr);		
		if(i >= MAX_ROUTABLE)
		{
			#if defined(DEBUG)
				PRINTF("No Child Node!\r\n");
			#endif
			return 1;
		}

		if(RoutTable[i].ucNode_Type == ZED_TYPE)
		{
			ucZED_Flg = TRUE;
		}

		if(ucOpt == OPT_READ)
		{
			Cmd |= READ_CMD;
			i = Build_Cmd_Req(uiDstAddr, (u8)Cmd, CmdValue, ucLen, RadioSendBuffer);		
		
		}
		else if(ucOpt == OPT_WRITE)
		{
			Cmd |= WRITE_CMD;
			i = Build_Cmd_Req(uiDstAddr, (u8)Cmd, CmdValue, ucLen, RadioSendBuffer);		
		}
		else
		{
		
		}

		if(ucZED_Flg)
		{
			Add_Poll_Msg((LoRa_Data_Typedef *)&RadioSendBuffer[1], i-2);
		}
		else
		{
			
			Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
		}
		
	}
	
	#endif
	
	return 0;
}

    
u8 AwarePro_Period_Event(u32 ulSysTime)
{
	Period_Poll_Data(ulSysTime);       //period wakeup mcu and poll data from parent node
	Period_Update_Event(ulSysTime);
	Handle_Error_Process(ulSysTime);
	
	return 0;
}

//////////////  rx tx process  ///////////////
void AwarePro_Tx_Done_Process(void)
{
	RF_Tx_Inc();

	HandleError.TimeOutCnt = 0;     //tx ok
	
}

void AwarePro_Tx_Timeout_Process(void)
{	
	HandleError.TimeOutCnt++;
}


void AwarePro_Rx_Done_Process(void)
{
	HandleError.ErrCnt = 0;
}

void AwarePro_Rx_Timeout_Process(void)
{

	#if defined(ZED)

//	if(POLL_WAITING_ACK == Poll_Ack.ucStatus)
//	{
//		if(Poll_Ack.ucFail_Time > RF_TX_MAX_RETRY_TIMES)
//		{
//			
//			Poll_Ack.ucStatus = POLL_RESET;

//		}
//		else
//		{
//			Poll_Ack.ucFail_Time++;
//			Poll_Ack.ucStatus = POLL_RETRY;

//		}
//		
//		
//	}
//	else
//	{	
//		
//	}
	
	#else      //RX all Time
	

	#endif
}

void AwarePro_Rx_Error_Process(void)
{
	HandleError.ErrCnt++;
}


//////////////  addr ////////////////

u8	Is_Local_Addr( Dev_Addr* Addr )
{
	
	if(memcmp(SysPara.pLocalAddr, Addr, sizeof(Dev_Addr))== 0)
	{
		return 1;
	}
	else if(memcmp(&BroadCastAddr, Addr, sizeof(Dev_Addr))== 0)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}


u8	Is_Local_NwkAddr( u16 NwkAddr )
{
	
	if(Nv_Para.uiNwkAddr == NwkAddr)
	{
		return 1;
	}
	else if(BroadAddr == NwkAddr)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}


//return 1 dup_frame
u8 Is_Dup_Frame(u8* ucFrame)
{
	
#if  defined(COO)|| defined(Router)
	u8 i = 0;
	LoRa_Data_Typedef * LoRa = (LoRa_Data_Typedef *)ucFrame;

	if((JOIN_CMD == LoRa->ucLoRa_Cmd)
		||(JOIN_RESP_CMD == LoRa->ucLoRa_Cmd))
	{
		return 0;
	}

	if(HEARTBEAT_CMD == LoRa->ucLoRa_Cmd)    //ZED heartbeat creat RouteTbl add 20170728
	{
		return 0;    
	}
	
	if( LoRa->ucDev_Seq == RoutTable[i].ucSeq)
	{
		return 1;
	}
	else
	{
	
		RoutTable[i].ucSeq = LoRa->ucDev_Seq;
		
		return 0;
	}
#else
	return 0;
#endif

	
}

u8 Update_Parent_RSSI(u16 uiSrcAddr, s16 siRssi)
{
	if(uiSrcAddr == Nv_Para.uiPare_Addr)
	{
		Nv_Para.scRSSI = (s8)siRssi;

		return 0;
	}

	return 1;
}

/// 0 Poll Req;
u8 Is_ZED_PollReq(void)
{
//	static u8 flg = 0;
	
	if(( Poll_Ack.ucStatus == 0)&&( SysPara.ucPollReq == 0))
	{	
//		flg = 0;
		return 1;
	}
	else
	{
		
		#if defined(DEBUG)
		
//			if(flg == 0)
			{
//				flg = 1;
				
//				PRINTF("Poll_Ack Pending,status:%d,Req:%d!\r\n", Poll_Ack.ucStatus, SysPara.ucPollReq);
				
			}
			
		#endif
		
		return 0;
	
}
}

/// 0 Rf Tx pending
u8 Is_RF_Tx_Pending(void)
{
//	static u8 flg = 0;
	
	if(RF_TX_QUE_SIZE == Poll_RF_Buffer())
	{
//		flg = 0;
		return 1;
	}
	else
	{
		#if defined(DEBUG)
		
		
//			if(flg == 0)
			{
//				flg = 1;
//				PRINTF("RF Pending!\r\n");
			}
			
		#endif
		
		return 0;
	}
	
}


//0 joining
u8 Is_Joining(void)
{
//	if((SysPara.ucJoinStatus != JOIN_STATUS_SENT_BCON_REQ)
//		&&(SysPara.ucJoinStatus != JOIN_STATUS_SENT_JOIN_REQ)
//		&&(SysPara.ucJoinStatus != JOIN_STATUS_RETRY_BECON_REQ)
//		&&(SysPara.ucJoinStatus != JOIN_STATUS_REJOIN_REQ))

	if((AwarePro_NV_JoinFlg() == NET_FLG)
		||((AwarePro_NV_JoinFlg() != NET_FLG)&&(SysPara.ucJoinStatus == Join_TimeOut))
		||((AwarePro_NV_JoinFlg() != NET_FLG)&&(SysPara.ucJoinStatus == Leave)))
	{
		return 1;
	}
	else
	{
		#if defined(DEBUG)
//		PRINTF("JoinStatus:%d!\r\n",SysPara.ucJoinStatus);
		#endif
		return 0;
	}
}

/// 0 Ack pending
u8 Is_Ack_Pending(void)
{
	if(Ack.ucPend == FALSE)
	{
		return 1;
	}
	else
	{
		#if defined(DEBUG)
//			PRINTF("ACK Pending!\r\n");
		#endif
		
		return 0;
	}
}

/// 0 Ack pending
u8 Is_Cmd_Resp_Pending(void)
{
	if(SysPara.ucRespFlag == FALSE)
	{
		return 1;
	}
	else
	{
		#if defined(DEBUG)
//			PRINTF("Cmd no Resp!\r\n");
		#endif
		return 0;
	}
}


// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static u32 next = 1;

u32 rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( u32 seed )
{
    next = seed;
}
// Standard random functions redefinition end

u32 randr( u32 min, u32 max )
{
    return ( u32 )rand1( ) % ( max - min + 1 ) + min;
}



// return 0xff no RF data
u8 Poll_RF_Buffer(void)
{
	u8 i = 0xFF;

	for(i = 0; i < RF_TX_QUE_SIZE; i++)
	{
		if(asRF_TX_Que[i].ucLength != 0)
		{
			return i;
		}
	}

	return i;
}

/*
void Poll_RF_Status(void)
{	
	if((RF_TX_QUE_SIZE == Poll_RF_Buffer())
//		&&(UART_BUFFER_SIZE == Poll_UART_Buffer())
		&&( Poll_Ack.ucStatus == 0)
		&&( SysPara.ucPollReq == 0)
		&&(Ack.ucPend == FALSE)
		&&(SysPara.ucJoinStatus != JOIN_STATUS_SENT_BCON_REQ)
		&&(SysPara.ucJoinStatus != JOIN_STATUS_SENT_JOIN_REQ)
		&&(SysPara.ucJoinStatus != JOIN_STATUS_RETRY_BECON_REQ))
	{
	
		SysPara.ucRunStatus = 0;
	
	}
	else
	{
		
		SysPara.ucRunStatus = 1;
	}

}
*/


void Handle_Error_Process(u32 ulSysTime)
{

	// tx cnt:tx fail add 1,rx data clear
	if(HandleError.TxCnt > RF_TX_MAX_RETRY_TIMES)   // can not rx rf data
	{
		HandleError.TxCnt = 0;
		HandleError.Status = NO_Rx;
	}
	
	if(HandleError.TimeOutCnt > RF_TX_MAX_RETRY_TIMES)  //tx timeout
	{
		HandleError.TimeOutCnt = 0;
		HandleError.Status = Tx_error;
	}
	
	if((NO_Rx == HandleError.Status)
		||(Tx_error == HandleError.Status))
	{
		Func_t->Error(HandleError.Status);

		HandleError.Status = Normal;
	}
	else
	{
		
	}
	
}


//////////////  superframe  //////////////
void Store_SuperFrame(u8* Data, u8 Len)
{
	
	LoRa_Data_Typedef* LoRaData = (LoRa_Data_Typedef*)Data;

	if(SuperData.ucStatus == ACT_IN_PROCESS)
	{
		#if defined(DEBUG)
		PRINTF("SuperData Processing!\r\n");
		#endif
		return;
	}
	
	
	SuperData.ucLen      = Len;
	SuperData.ucStatus   = ACT_IN_IDLE;
	SuperData.ucSeq      = LoRaData->ucDev_Seq;
	SuperData.uiDst_Addr = LoRaData->uiDst_Addr;
	SuperData.uiSrc_Addr = LoRaData->uiSrc_Addr;
	
	memcpy(SuperData.auSuperDataBuffer, Data, Len);
	
}

u8 Process_SuperFrame(u32 ulSysTime)
{
	// TX 
	
	static u8 CurNum = 0;

	if((SuperData.ucLen == 0)&&(RxSuperData.ucStatus == ACT_IN_IDLE))
	{
		return 0;
	}
	
	if((SuperData.ucLen > 0)&&(SuperData.ucStatus == ACT_IN_IDLE))
	{
	
		u8 Temp,Num;
		Super_Frame_Typedef* SuperFrame = (Super_Frame_Typedef*)RadioSendBuffer;

		Temp = SuperData.ucLen % SUPER_FRAME_DATA_LEN;
		Num  = SuperData.ucLen / SUPER_FRAME_DATA_LEN;

		SuperFrame->ucFrameType = SUPER_BIT | ACK_BIT;    
		SuperFrame->ucTotalLen  = SuperData.ucLen;

		if(Temp)
		{
			
			SuperFrame->ucTotalNum	= Num + 1;
		}
		else
		{
			
			SuperFrame->ucTotalNum	= Num;
		}
		
		CurNum++;

		if(CurNum == SuperFrame->ucTotalNum)
		{
			SuperFrame->ucLen = SuperData.ucLen - SUPER_FRAME_DATA_LEN*(CurNum - 1);
			SuperFrame->ucNum = CurNum;
			memcpy((u8*)&SuperFrame->auSuperData[0], (u8*)&SuperData.auSuperDataBuffer[(CurNum - 1)*SUPER_FRAME_DATA_LEN], SuperFrame->ucLen);
			SuperFrame->auSuperData[SuperFrame->ucLen] = 0xFF;   //crc
//			SuperFrame->ucCRC = 0xFF;	
			CurNum = 0;
			SuperData.ucLen = 0;
		}
		else
		{
	
			SuperFrame->ucLen = SUPER_FRAME_DATA_LEN;
			SuperFrame->ucNum = CurNum;			
			memcpy((u8*)&SuperFrame->auSuperData[0], (u8*)&SuperData.auSuperDataBuffer[(CurNum - 1)*SUPER_FRAME_DATA_LEN], SUPER_FRAME_DATA_LEN);
			SuperFrame->ucCRC = 0xFF;
		}
		
		Add_RF_Frame_To_TX_Que(RadioSendBuffer, 
								SuperFrame->ucLen + SUPER_FRAME_HEAD_LEN, 
								RF_TX_QUE_OPT_SUPER,
								ulSysTime,
								SuperData.uiDst_Addr,
								SuperData.uiSrc_Addr,
								SuperData.ucSeq);

		SuperData.ucStatus = ACT_IN_PROCESS;
		SuperData.ulTime   = ulSysTime;
//		SuperData.ulTime   =  HW_RTC_Tick2ms(HW_RTC_GetTimerValue());
	}
	else if(SuperData.ucStatus == ACT_IN_PROCESS)
	{
		if((u32)(ulSysTime - SuperData.ulTime) > SUPER_FRAME_TIMEOUT)
			
//		if((u32)( HW_RTC_Tick2ms(HW_RTC_GetTimerValue()) - SuperData.ulTime) > SUPER_FRAME_TIMEOUT)
		{
			SuperData.ucStatus = ACT_IN_IDLE;
			
			memset((u8*)&SuperData, 0x00, sizeof(SuperData));
			CurNum = 0;

			#if defined(DEBUG)
			PRINTF("Tx Super Data Timeout!\r\n");
			#endif
		}
	}
	else
		{
		}

	//RX
	if(RxSuperData.ucStatus == ACT_IN_END)
	{
		memset((u8*)&RxSuperData, 0x00, sizeof(RxSuperData));
	}
	else if((RxSuperData.ucStatus == ACT_IN_PROCESS)
			||(RxSuperData.ucStatus == ACT_IN_RX_DONE))    //rx first frame
	{
		if((u32)( ulSysTime - RxSuperData.ulTime) > SUPER_FRAME_TIMEOUT)
		{
			RxSuperData.ucStatus = ACT_IN_END;

			#if defined(DEBUG)
			PRINTF("Receive Super Data Timeout!\r\n");
			#endif
		}
	}
	else
	{
		RxSuperData.ucStatus = ACT_IN_IDLE;
	}

	return 0;
}

///////////////  join net  /////////////////

u8 Beacon_Req(u32 ulSysTime)
{
//	u8 i;
	
	LoRa_Join_Typedef* LoRaJoin = (LoRa_Join_Typedef*)&RadioSendBuffer[1];
	u8 len = LORA_JOIN_LEN;

	RadioSendBuffer[0]  = BEACON_TYPE;
	memcpy(&LoRaJoin->sDst_Addr, &BroadCastAddr, sizeof(Dev_Addr));
	memcpy(&LoRaJoin->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	LoRaJoin->ucLoRa_Cmd    = JOIN_CMD;
	LoRaJoin->ucDev_Seq     = SysPara.ucPacketSeq;
	LoRaJoin->ucPID         = 0;
	LoRaJoin->ucHop         = 0;
	LoRaJoin->ucDev_Type    =  SysPara.ucDevType;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_JOIN, ulSysTime, 0, 0, LoRaJoin->ucDev_Seq);

	//for debug
	
//	PRINTF("join cmd:\r\n");
//	for(i = 0; i < len; i++)
//	{
//		PRINTF("%02x ", RadioSendBuffer[i]);
//	}
//	
//	Uart_Send(RadioSendBuffer, len);
	return 0;
}

u8 Beacon_Resp(LoRa_Join_Typedef* Beacon, u32 ulSysTime)
{
//	u8 i;
	
	Beacon_Resp_Typedef* LoRaJoin = (Beacon_Resp_Typedef*)&RadioSendBuffer[1];
	u8 len = BEACON_RESP_LEN;

	RadioSendBuffer[0]  = BEACON_RESP_TYPE;
	memcpy(&LoRaJoin->sDst_Addr, &Beacon->sSrc_Addr, sizeof(Dev_Addr));
	memcpy(&LoRaJoin->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	LoRaJoin->ucLoRa_Cmd    = JOIN_CMD;	
	LoRaJoin->ucDev_Type    =  SysPara.ucDevType;
	LoRaJoin->ucDev_Seq     = SysPara.ucPacketSeq;
	LoRaJoin->ucPID         = Nv_Para.ucPID;
	LoRaJoin->uiPare_Addr   = Nv_Para.uiNwkAddr;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);

	//for debug
	
//	PRINTF("join cmd:\r\n");
//	for(i = 0; i < len; i++)
//	{
//		PRINTF("%02x ", RadioSendBuffer[i]);
//	}
//	
//	Uart_Send(RadioSendBuffer, len);
	return 0;
}


u8 Node_Join_LoRa(u32 ulSysTime)
{
//	u8 i;
	
	LoRa_Join_Typedef* LoRaJoin = (LoRa_Join_Typedef*)&RadioSendBuffer[1];
	u8 len = LORA_JOIN_LEN;

	RadioSendBuffer[0]  = JOIN_TYPE;
	memcpy(&LoRaJoin->sDst_Addr, &Nv_Para.sHost_Addr, sizeof(Dev_Addr));  //BroadCastAddr
	memcpy(&LoRaJoin->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	LoRaJoin->ucLoRa_Cmd    = JOIN_CMD;
	LoRaJoin->ucDev_Seq     = SysPara.ucPacketSeq;
	LoRaJoin->ucPID         = 0;
	LoRaJoin->ucHop         = 0;
	LoRaJoin->ucDev_Type    =  SysPara.ucDevType;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_JOIN, ulSysTime, 0, 0, LoRaJoin->ucDev_Seq);

	//for debug
	
//	PRINTF("join cmd:\r\n");
//	for(i = 0; i < len; i++)
//	{
//		PRINTF("%02x ", RadioSendBuffer[i]);
//	}
//	
//	Uart_Send(RadioSendBuffer, len);
	return 0;
}


u8 Find_Best_Parent(void)
{
	#if defined(Router) || defined(ZED)
	u8 i;
	u8 ucIndex = 0xff;
//	u8 ucJoinTimes = 0xff;
	s16 siRssi = JOIN_MIN_RSSI;
	
	for(i = 0; i < MAX_PARE_TBL; i++)
	{
		if(PareTbl[i].ucTry_Join < 3)
		{
			if((COO_NwkAddr ==PareTbl[i].uiPare_Addr)
			&&(memcmp(&PareTbl[i].sPare_Addr, &NullAddr, sizeof(Dev_Addr))))  //COO 
			{
				return i;
			}
			else
			{
				if((PareTbl[i].uiPare_Addr != 0)
					&&(PareTbl[i].ucDev_Type == ROUTER_TYPE))
				{
					if(PareTbl[i].siRssi > siRssi)  //max 
//					if(PareTbl[i].ucTry_Join < ucJoinTimes)
					{
						ucIndex = i;
						siRssi  = PareTbl[i].siRssi;
//						ucJoinTimes = PareTbl[i].ucTry_Join;
					}
				}
				
			}
		}
		else
		{
			#if defined(DEBUG)
				PRINTF("Pare_NwkAddr:%04x try join max\r\n",PareTbl[i].uiPare_Addr);
			#endif
		}
		
	}

	if(ucIndex < MAX_PARE_TBL)
	{
		return ucIndex;
	}

	return i;

	#else
	return 0;
	#endif
}

void Join_Req( u32 ulSysTime )
{
	
	#if !defined(COO)
	
	u8 i;
			
	i = Find_Best_Parent();
	if(i < MAX_PARE_TBL)
	{		
		memcpy(&Nv_Para.sHost_Addr, &PareTbl[i].sPare_Addr, sizeof(Dev_Addr));
	}
	else
	{
		#if defined(DEBUG)
		PRINTF("No Parent Node!\r\n");
		#endif
		
		SysPara.ucJoinStatus = Join_Init;
		
		return;
	}
	
	PareTbl[i].ucTry_Join++;
	//join Req
	Node_Join_LoRa(ulSysTime);

	#if defined(DEBUG)
	PRINTF("Join Request!\r\n");
	#endif

//	Join_Status = JOIN_STATUS_REC_JOIN_RESP;

	#endif
}



u8 Relay_Join_Resp(Relay_Join_Typedef* JoinData, u16 NwkAddr, u32 ulSysTime)
{
	
	u8 len = RELAY_RESP_LEN;
	
	Relay_Resp_Typedef* JoinResp = (Relay_Resp_Typedef*)&RadioSendBuffer[1];
	
	RadioSendBuffer[0] = JOIN_TYPE;
	
	memcpy(&JoinResp->sDst_Addr, &JoinData->sSrc_Addr, sizeof(Dev_Addr));
	memcpy(&JoinResp->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));
	memcpy(&JoinResp->sPre_Addr, &JoinData->sPre_Addr, sizeof(Dev_Addr));

	JoinResp->ucLoRa_Cmd = JOIN_CMD | RESP_CMD;
	JoinResp->ucHop      = 0;
	JoinResp->ucDev_Seq  = JoinData->ucDev_Seq;
	JoinResp->ucPID      = Nv_Para.ucPID;
	JoinResp->uiDst_Addr = NwkAddr;
	
//	memcpy(&JoinResp->ucPayload, (u8*)&NwkAddr,sizeof(u16));


	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

//	LoRa_Send(RadioSendBuffer, len + 2);

	len += 2;
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	//for debug //
//	Uart_Send(RadioSendBuffer, len);

	return 0;
}

u8 Join_Resp(LoRa_Join_Typedef* JoinData, u16 NwkAddr, u32 ulSysTime)
{
//	u8 i;
	
	u8 len = JOIN_RESP_LEN;
	
	Join_Resp_Typedef* JoinResp = (Join_Resp_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0] = JOIN_TYPE;
	
	memcpy(&JoinResp->sDst_Addr, &JoinData->sSrc_Addr, sizeof(Dev_Addr));
	memcpy(&JoinResp->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	JoinResp->ucLoRa_Cmd = JOIN_CMD | RESP_CMD;
	JoinResp->ucHop      = 0;
	JoinResp->ucDev_Seq  = JoinData->ucDev_Seq;
	JoinResp->ucPID      = Nv_Para.ucPID;
	JoinResp->uiDev_Addr = NwkAddr;
	JoinResp->uiPare_Addr = Nv_Para.uiNwkAddr;

//	memcpy(&JoinResp->ucPayload, (u8*)&NwkAddr,sizeof(u16));

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);
	//for debug
	
//	PRINTF("join Resp:\r\n");
//	for(i = 0; i < len; i++)
//	{
//		PRINTF("%02x ", RadioSendBuffer[i]);
//	}
	
	return 0;
}


u8 Relay_Join_Cmd(LoRa_Join_Typedef* JoinData, s8 JoinRssi, u32 ulSysTime)
{

	u8 len = RELAY_JOIN_LEN;
	
	Relay_Join_Typedef* JoinRelay = (Relay_Join_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0] = JOIN_TYPE;
	
	memcpy(&JoinRelay->sDst_Addr, &Nv_Para.sHost_Addr, sizeof(Dev_Addr));	
	memcpy(&JoinRelay->sSrc_Addr, &JoinData->sSrc_Addr, sizeof(Dev_Addr));
	memcpy(&JoinRelay->sPre_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	JoinRelay->ucLoRa_Cmd = JOIN_CMD;
	JoinRelay->ucHop      = JoinData->ucHop + 1;
	JoinRelay->ucDev_Seq  = JoinData->ucDev_Seq;
	JoinRelay->cRssi      = JoinRssi;
	JoinRelay->ucDev_Type = ZED_TYPE;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	return len;
}

u8 Accept_Join_LoRa(u8* RxData, s8 Rssi, u32 ulSysTime)
{

#if defined(COO)

	u8 i;
	
	LoRa_Join_Typedef* JoinData = (LoRa_Join_Typedef*)RxData;

	if(JoinData->ucHop > 0)  //relay frame
	{
		u8 i = 0xFF;
		
		Relay_Join_Typedef* RelayData = (Relay_Join_Typedef*)RxData;

		i = Get_Node_Entry(&RelayData->sPre_Addr);   //Router Node must my Child Node
		if(i < COO_MAX_ROUTABLE)
		{

			#if defined(DEBUG)
//			print_8_02x((u8*)&RelayData->sPre_Addr);
			#endif
			
			RoutTable[i].cRssi     = Rssi;     //relay node
			RoutTable[i].ucAge     = 0;        
			
			i = Get_Empty_RoutTable(&RelayData->sSrc_Addr);  //ZED node

//			if((memcmp(&RoutTable[i].sNode_Addr, &RelayData->sSrc_Addr, sizeof(Dev_Addr)) == 0)
//				&&(ZED_TYPE == RoutTable[i].ucNode_Type))
//			{

//				PRINTF("Have Joined!\r\n");
//				return 0xFF;
//			}
//			else
			{
				memcpy(&RoutTable[i].sNode_Addr, &RelayData->sSrc_Addr, sizeof(Dev_Addr));
				RoutTable[i].uiAddr    = (u16)(i + 1);
				RoutTable[i].cRssi     = RelayData->cRssi;
				RoutTable[i].ucNode_Type = RelayData->ucDev_Type | RELAY_TYPE_BIT;
				RoutTable[i].ucAge      = 0;
				RoutTable[i].ucSeq      = 0;
			}
			

			#if defined(NV)
//			Update_Node_to_Flash();			
			Add_Node_To_Flash(i);
			#endif
			
			Relay_Join_Resp(RelayData, RoutTable[i].uiAddr, ulSysTime);

			
			#if defined(DEBUG)
			PRINTF("Relay Resp!\r\n");
			#endif
			
		}
	}
	
	else
	{
//		LoRa_Join_Typedef* JoinData = (LoRa_Join_Typedef*)RxData;

//		if(ZED_TYPE == JoinData->ucDev_Type)     ///for test router join relay
//		{
//			return i;
//		}
		
		i = Get_Empty_RoutTable(&JoinData->sSrc_Addr);
	
		memcpy(&RoutTable[i].sNode_Addr, &JoinData->sSrc_Addr, sizeof(Dev_Addr));
		RoutTable[i].uiAddr    = (u16)(i + 1);
		RoutTable[i].cRssi     = Rssi;
//		RoutTable[i].ucNode_Type = JoinData->ucDev_Type | OFFLINE_BIT;   //add by dozen 20170724
		RoutTable[i].ucNode_Type = JoinData->ucDev_Type;   //add by dozen 20170803
		RoutTable[i].ucAge      = 0;
		RoutTable[i].ucSeq      = 0;

		#if defined(NV)
//		Update_Node_to_Flash();
		
		Add_Node_To_Flash(i);
		#endif

		Func_t->Node(&RoutTable[i].sNode_Addr, RoutTable[i].uiAddr, RoutTable[i].ucNode_Type, OPT_JOIN);
		//for debug
//		Uart_Send((u8*)JoinData, sizeof(LoRa_Join_Typedef));  //change to profile and report service
		Join_Resp(JoinData, RoutTable[i].uiAddr, ulSysTime);
	}
	

	return i;

#elif defined(Router)
	
	
	u8 i = 0xFF;
	
	LoRa_Join_Typedef* JoinData = (LoRa_Join_Typedef*)RxData;

	if(!Nv_Para.ucJoin_Flag)
	{
		
		#if defined(DEBUG)
		PRINTF("Router Node have not joined LoRa!\r\n");
		#endif
		
		return i;
	}
		
	if( ZED_TYPE == JoinData->ucDev_Type )
	{
		
		i = Get_Empty_RoutTable(&JoinData->sSrc_Addr);
			
		memcpy(&RoutTable[i].sNode_Addr, &JoinData->sSrc_Addr, sizeof(Dev_Addr));
		RoutTable[i].uiAddr    = 0;
		RoutTable[i].cRssi	   = Rssi;
//		RoutTable[i].ucNode_Type = ZED_TYPE | OFFLINE_BIT;	
		RoutTable[i].ucNode_Type = ZED_TYPE;
		RoutTable[i].ucAge		= 0;
		RoutTable[i].ucSeq		= 0;
	
//		Add_Node_To_Flash(i);
//		Uart_Send((u8*)JoinData, sizeof(LoRa_Join_Typedef));	//change to profile and report service
//		Join_Resp(JoinData, RoutTable[i].uiAddr);

		Relay_Join_Cmd(JoinData, Rssi, ulSysTime);

		#if defined(DEBUG)

		PRINTF("Relay join cmd!\r\n");

		#endif
	}
	
	
	return i;
	
#else

	return 0;

#endif
}


u8 Process_Join_Req(Join_Resp_Typedef* LoRaData, s8 scRssi)
{
	#if defined(ZED) || defined(Router)

	if(LoRaData->uiDev_Addr == 0x0000)
	{

		#if defined(DEBUG)
		PRINTF("Join LoRa Failed!\n");
		#endif
		return 1;
	}

	if(Nv_Para.ucJoin_Flag)
	{
		
		#if defined(DEBUG)
		PRINTF("Have Succeeded!\r\n");
		#endif
		
		return 2;
	}
	
	memcpy(&Nv_Para.sHost_Addr, &LoRaData->sSrc_Addr, sizeof(Dev_Addr));	
//	memcpy((u16*)&Nv_Para.uiNwkAddr, (u16*)&LoRaData->ucPayload, sizeof(u16));
	Nv_Para.uiNwkAddr   = LoRaData->uiDev_Addr;
	Nv_Para.ucPID       = LoRaData->ucPID;
	Nv_Para.uiPare_Addr = LoRaData->uiPare_Addr;
	Nv_Para.scRSSI      = scRssi;
//	Nv_Para.ucJoin_Flag = 1;


//	#if defined(NV)

//	Write_NV();
//	
//	#endif
	
	#if defined(DEBUG)
	PRINTF("Join LoRa Success!\n");
	#endif

	
	SysPara.ucJoinStatus = Join_Success;   //add by 20170901	
	memset(PareTbl, 0x00, MAX_PARE_TBL*sizeof(Pare_Table_Typedef));

	#endif
	
	return 0;
}


u8 Router_Join_Resp(Relay_Resp_Typedef* JoinData, u32 ulSysTime)
{
	
	u8 len = JOIN_RESP_LEN;
	
	Join_Resp_Typedef* JoinResp = (Join_Resp_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0] = JOIN_TYPE;
	
	memcpy(&JoinResp->sDst_Addr, &JoinData->sDst_Addr, sizeof(Dev_Addr));
	memcpy(&JoinResp->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));

	JoinResp->ucLoRa_Cmd = JoinData->ucLoRa_Cmd;
	JoinResp->ucHop      = JoinData->ucHop + 1;
	JoinResp->ucDev_Seq  = JoinData->ucDev_Seq;
	JoinResp->ucPID      = JoinData->ucPID;
	JoinResp->uiDev_Addr = JoinData->uiDst_Addr;
	JoinResp->uiPare_Addr= Nv_Para.uiNwkAddr;
	
//	pRadioSendBuffer->ucPayload[0]  =  JoinData->ucPayload[0];

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);


//	LoRa_Send(RadioSendBuffer, len + 2);
	len += 2;
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);

	return 0;
}


u8 Process_Join_Cmd(u8* JoinData, u8 len, s16 rssi, u32 ulSysTime)
{
	
	LoRa_Join_Typedef* Join = (LoRa_Join_Typedef*)JoinData;
	
	#if defined(COO)
	
		if( JOIN_CMD == Join->ucLoRa_Cmd )
		{
			Accept_Join_LoRa((u8*)Join, rssi, ulSysTime);
		}
		
	#elif defined(Router)
	
		if( JOIN_CMD == Join->ucLoRa_Cmd )
		{
			Accept_Join_LoRa((u8*)Join, rssi, ulSysTime);
		}
		else if( JOIN_RESP_CMD == Join->ucLoRa_Cmd )  //from COO
		{
			if(Is_Local_Addr(&Join->sDst_Addr))
			{
				
				Join_Resp_Typedef* JoinResp = (Join_Resp_Typedef*)JoinData;
				
				if(!Process_Join_Req(JoinResp, rssi))
				{	
					//join successful
					
//					Join_Resp_ACK(JoinResp);
				}
			}
			else   //relay frame
			{
				u8 i = 0xFF;
				
				i = Get_Node_Entry(&Join->sDst_Addr);
				if(i < ROUTER_MAX_ROUTABLE)
				{
					Relay_Resp_Typedef* RelayResp = (Relay_Resp_Typedef*)JoinData;
					
//					memcpy((u16*)&RoutTable[i].uiAddr, (u16*)&RelayJoin->ucPayload, sizeof(u16));

					RoutTable[i].uiAddr = RelayResp->uiDst_Addr;
					RoutTable[i].ucAge = 0;
					RoutTable[i].ucSeq = 0;

					#if defined(NV)
//					Update_Node_to_Flash();
					
					Add_Node_To_Flash(i);
					#endif
					
					Router_Join_Resp(RelayResp, ulSysTime);

					
					#if defined(DEBUG)
					PRINTF("Route join Resp!\r\n");
					#endif
				}
				
			}
			
		}

	#else

		if( JOIN_RESP_CMD == Join->ucLoRa_Cmd )
		{

			if(JOIN_RESP_LEN == len)
			{
				Join_Resp_Typedef* JoinResp = (Join_Resp_Typedef*)JoinData;
			
				if(!Process_Join_Req(JoinResp, rssi))
				{	
//					Join_Resp_ACK(JoinResp);
							
//					Period_Poll_Data();
					//callback poll data
				}
			}
			
		}
		
	#endif
		
		return 0;
}

u8 ZED_Data_Req(u8 ucOpt, u32 ulSysTime)
{
	
#if defined(ZED)

	u8 len = LORA_PACK_HEAD + 1;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];

	RadioSendBuffer[0]  = DATA_NOACK_TYPE;

	LoRa->uiDst_Addr = Nv_Para.uiPare_Addr;            //parent addr   ??
	
	LoRa->uiSrc_Addr    = Nv_Para.uiNwkAddr;

	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucHop      = 0;
	LoRa->ucLoRa_Cmd = POLL_CMD;
	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = 0;

	LoRa->ucPayload[0]  =  0;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, ucOpt, ulSysTime, 0, 0, 0);

#endif

	return 0;
	
}

u8 Period_Poll_Data(u32 ulSysTime)
{

	#if defined(ZED)
	
//	static u32 ulPreTime = 0;
//	u32 ulPeriod_Time = (u32)(Nv_Para.ucPoll_TimeOut*PERIOD_S);

	if(Nv_Para.ucJoin_Flag != NET_FLG)
	{
		 return 1;
	}
	
//	if(SysPara.ulJoinDelay > 0)        //send first poll
//	{
//		SysPara.ulJoinDelay = 0;
//		if((u32)(ulSysTime - ulPreTime) > SysPara.ulJoinDelay)
//		{
//			ulPreTime = ulSysTime;
//		
//			Poll_Ack.ucSeq       = SysPara.ucPacketSeq;
//	    	Poll_Ack.ucStatus    = POLL_WAITING_ACK;
//			Poll_Ack.ucFail_Time = 0;
//			
//			ZED_Data_Req(0, ulSysTime);

//			#if defined(DEBUG)
//			PRINTF("First Poll!\r\n");
//			#endif
//		}
//	}
	
//	if(((u32)(ulSysTime - ulPreTime) > ulPeriod_Time)
//		||(SysPara.ucPollReq))

	if(SysPara.ucPollReq)
	{
//		ulPreTime = ulSysTime;
		
		SysPara.ucPollReq = 0;
		
		Poll_Ack.ucSeq       = SysPara.ucPacketSeq;
    	Poll_Ack.ucStatus    = POLL_WAITING_ACK;
		Poll_Ack.ucFail_Time = 0;
		
		ZED_Data_Req(0, ulSysTime);
		
		#if defined(DEBUG)
//		PRINTF("Seq:%d,Staus:%d!\n",Poll_Ack.ucSeq,Poll_Ack.ucStatus);
		#endif

//		ucLoRaStatus = 1;
	}

	
	ZED_DataReq_Event(ulSysTime);

	#endif
	
	return 0;
}


u8 ZED_Poll_Resp(LoRa_Data_Typedef* PollData, s8 cRssi, u32 ulSysTime)
{
	u8 Ret = 0;

	#if defined(COO)
	
	u8 len = 0;
	u8 i,j;
	
	
	i = Get_NwkAddr_Entry(PollData->uiSrc_Addr);
	if(i >= COO_MAX_ROUTABLE)
	{

		return 1;
	}

	RoutTable[i].ucNode_Type = ZED_TYPE;
	RoutTable[i].cRssi       = cRssi;
	RoutTable[i].ucAge       = 0;
	RoutTable[i].ucSeq       = PollData->ucDev_Seq;

//	Uart_Send((u8*)JoinData, len);  //change to profile and report service
	for(j = 0; j < POLL_BUFFER_SIZE; j++)
	{
		if(memcmp(&asPoll_Buffer[j].sChild_Addr, &RoutTable[i].sNode_Addr, sizeof(Dev_Addr))== 0)
		{
			break;
		}
	}

	if(j < POLL_BUFFER_SIZE)      //add MAC frame
	{
		len = Build_MAC_Frame(asPoll_Buffer[j].aucMessage, asPoll_Buffer[j].ucMessage_Length,
						DATA_NOACK_TYPE, RadioSendBuffer);
		
//		len = asPoll_Buffer[j].ucMessage_Length;
//		memcpy((u8*)pRadioSendBuffer, asPoll_Buffer[j].aucMessage, len);
		memset((u8*)&asPoll_Buffer[j], 0x00, sizeof(POLL_BUFFER_Typedef));

		Ret = VALID_POLL;
	}
	else
	{
		len = Build_Zero_Poll_Resp(PollData);

		Ret = NO_VALID_POLL;

//		PRINTF("Zero Poll Resp!\r\n");
	}
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	#endif
	
	#if defined(Router)

	u8 len = 0;
	u8 i,j;
	
	i = Get_NwkAddr_Entry(PollData->uiSrc_Addr);
	if(i >= ROUTER_MAX_ROUTABLE)
	{

		return 1;
	}

	for(j = 0; j < POLL_BUFFER_SIZE; j++)
	{
		if(memcmp(&asPoll_Buffer[j].sChild_Addr, &RoutTable[i].sNode_Addr, sizeof(Dev_Addr))== 0)
		{
			break;
		}
	}

	if(j < POLL_BUFFER_SIZE)      //add MAC frame
	{
		len = Build_MAC_Frame(asPoll_Buffer[j].aucMessage, asPoll_Buffer[j].ucMessage_Length,
								DATA_NOACK_TYPE, RadioSendBuffer);

		memset((u8*)&asPoll_Buffer[j], 0x00, sizeof(POLL_BUFFER_Typedef));

		Ret = VALID_POLL;
	}
	else
	{
		len = Build_Zero_Poll_Resp(PollData);
		
		Ret = NO_VALID_POLL;
		
	}
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	#endif

	return Ret;
}

u8 Add_Poll_Msg(LoRa_Data_Typedef* LoRaData, u8 len)
{
	#if defined(COO)
	
	u8 i = 0xff;
	u8 j;

	if((len > (POLL_MESSAGE_MAX_SIZE - 2))&&(len < LORA_PACK_HEAD))  //max len 64-2
	{
		return 1;
	}
	
	i = Get_NwkAddr_Entry(LoRaData->uiDst_Addr);
	
	if(( i < COO_MAX_ROUTABLE)&&(ZED_TYPE == RoutTable[i].ucNode_Type))
	{
		for(j = 0; j < POLL_BUFFER_SIZE; j++)
		{
			if(asPoll_Buffer[j].ucMessage_Length == 0)
			{
				break;
			}
			else
			{
				if( asPoll_Buffer[j].ucAge > Nv_Para.ucPoll_TimeOut )
				{
					break;
				}
			}
		}

		if(j < POLL_BUFFER_SIZE)
		{
			asPoll_Buffer[j].sChild_Addr      = RoutTable[i].sNode_Addr;
			asPoll_Buffer[j].ucAge            = 0;
			asPoll_Buffer[j].ucMessage_Length = len;
			
			memcpy((u8*)asPoll_Buffer[j].aucMessage, (u8*)LoRaData, len);
			
		}
	}
	else
	{
		return 2;
	}

	#elif defined(Router)

	
	u8 i = 0xff;
	u8 j;

	if((len > POLL_MESSAGE_MAX_SIZE)&&(len < LORA_PACK_HEAD))
	{
		return 1;
	}
	
	i = Get_NwkAddr_Entry(LoRaData->uiDst_Addr);
	
	if(( i < ROUTER_MAX_ROUTABLE)&&(ZED_TYPE == RoutTable[i].ucNode_Type))
	{
		for(j = 0; j < POLL_BUFFER_SIZE; j++)
		{
			if(asPoll_Buffer[j].ucMessage_Length == 0)
			{
				break;
			}
			else
			{
				if( asPoll_Buffer[j].ucAge > Nv_Para.ucPoll_TimeOut )
				{
					break;
				}
			}
		}

		if(j < POLL_BUFFER_SIZE)
		{
			asPoll_Buffer[j].sChild_Addr      = RoutTable[i].sNode_Addr;
			asPoll_Buffer[j].ucAge            = 0;
			asPoll_Buffer[j].ucMessage_Length = len;
			
			memcpy((u8*)asPoll_Buffer[j].aucMessage, (u8*)LoRaData, len);
			
		}
	}
	else
	{
		return 2;
	}

	#else

	#endif
			
	return 0;
}

u8 ZED_DataReq_Event(u32 ulSysTime)
{
	#if defined(ZED)

	static u32 ulPreTime = 0;
	static u16 uiGap = 0;
	
	if(Poll_Ack.ucStatus == IDLE_STATUS)
	{
//		ucLoRaStatus = 0;
		return 1;
	}
	else if(Poll_Ack.ucStatus == POLL_WAITING_ACK)
	{
		ulPreTime = ulSysTime;
		Poll_Ack.ucStatus = POLL_TIMEOUT;

		srand1(ulSysTime);
		uiGap = randr(1000, 3000);               //1s -- 3s
	}
	else if(Poll_Ack.ucStatus == POLL_TIMEOUT)
	{

		if((u32)(ulSysTime - ulPreTime) > uiGap)
		{

			
			ulPreTime = ulSysTime;
			if(Poll_Ack.ucFail_Time > RF_TX_MAX_RETRY_TIMES)
			{
				
				Poll_Ack.ucStatus = POLL_RESET;

			}
			else
			{
				Poll_Ack.ucFail_Time++;
				Poll_Ack.ucStatus = POLL_RETRY;

			}
		}	
	
	}
	else if(Poll_Ack.ucStatus == POLL_RETRY)
	{
	
		Poll_Ack.ucSeq = SysPara.ucPacketSeq;
		Poll_Ack.ucStatus = POLL_WAITING_ACK;
		ZED_Data_Req(0, ulSysTime);

		PRINTF("Poll Retry:%d\r\n",Poll_Ack.ucFail_Time);
//		ucLoRaStatus = 1;
		
	}
	else if(Poll_Ack.ucStatus == POLL_RESET)
	{
					
		memset((u8*)&Poll_Ack, 0x00, sizeof(Poll_Ack));
		Nv_Para.ucJoin_Flag = 0;
		SysPara.ucJoinStatus = Rejoin_req;     //rejoin

		#if defined(NV)
//		Write_NV();
		Delete_NV();
		#endif

		PRINTF("Poll Rejoin!\r\n");
		
//		NVIC_SystemReset();
		
//			Join_Init();                     //rejoin Lora
	}
	else
	{
		
	}

//	ucLoRaStatus = 1;
	
	#endif

	return 0;
}


//Router send to COO
u8 ZED_Poll_Report(LoRa_Data_Typedef* PollData, u32 ulSysTime)
{
	#if defined(Router)
	u8 i;
	u8 len = LORA_PACK_HEAD + 1 + 8;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	
	RadioSendBuffer[0]           = DATA_NOACK_TYPE;
	
	LoRa->uiDst_Addr = COO_NwkAddr;
//	LaRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
	LoRa->uiSrc_Addr = PollData->uiSrc_Addr;    //ZED addr
	LoRa->ucPID      = Nv_Para.ucPID;

	LoRa->ucLoRa_Cmd = HEARTBEAT_CMD;
	LoRa->ucHop      = PollData->ucHop + 1;

	LoRa->ucDev_Seq  = PollData->ucDev_Seq;
	LoRa->ucSer_Seq  = 0;


	i = Get_NwkAddr_Entry(PollData->uiSrc_Addr);
	if(i < MAX_ROUTABLE)
	{
		
		LoRa->ucPayload[0]	=  RoutTable[i].ucNode_Type;
		memcpy(&LoRa->ucPayload[1], (u8*)&RoutTable[i].sNode_Addr, sizeof(Dev_Addr));
	}
	else
	{
		return 1;
	}

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);

	return (len);  //FrameType + check

	#else
	
	return 0;
	
	#endif
}

//Router send to COO
u8 Router_Test_Report(u32 ulSysTime)
{
	u8 i;
	u8 len = LORA_PACK_HEAD + 39;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0]           = DATA_ACK_TYPE;
	
	LoRa->uiDst_Addr = COO_NwkAddr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
//	LaRa->uiSrc_Addr = PollData->uiSrc_Addr;    //ZED addr
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = APP_DATA_TRANS;
	LoRa->ucHop      = 0;

	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = 0;

	for(i = 0; i < 39; i++)
	{
		LoRa->ucPayload[i] = i;
	}
//	LoRa->ucPayload[0]  =  DevType;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	return (len);  //FrameType + check
}


//Router send to COO
u8 Router_HeartBeat_Report(u32 ulSysTime)
{
	u8 len = LORA_PACK_HEAD + 1;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0]           = DATA_NOACK_TYPE;
	
	LoRa->uiDst_Addr = COO_NwkAddr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
//	LaRa->uiSrc_Addr = PollData->uiSrc_Addr;    //ZED addr
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = HEARTBEAT_CMD;
	LoRa->ucHop      = 0;

	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = 0;

	LoRa->ucPayload[0]  =  SysPara.ucDevType;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);

	return (len);  //FrameType + check
}

u8 Dev_Leave_Req(u32 ulSysTime)
{
	u8 len = LORA_PACK_HEAD + 1;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0]           = DATA_NOACK_TYPE;
	
	LoRa->uiDst_Addr = Nv_Para.uiPare_Addr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = Del_CMD|WRITE_CMD;//DEL_CMD;
	LoRa->ucHop      = 0;

	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = 0;

	LoRa->ucPayload[0]  =  SysPara.ucDevType;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	len += 2;
	
	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	return (len);  //FrameType + check
}


u8 Process_Ack(ACK_Typedef* LoRaData, u32 ulSysTime)
{
	u8 i = 0xFF;


	for(i = 0; i < RF_TX_QUE_SIZE; i++)
	{
		//R:C+Z+p, ACK:Z+C+seq
//		if((LoRaData->uiSrc_Addr == asRF_TX_Que[i].uiDst_Addr)
//			&&(LoRaData->uiDst_Addr == asRF_TX_Que[i].uiSrc_Addr))
		if(LoRaData->ucDev_Seq == asRF_TX_Que[i].ucSeq)
		{

			Func_t->App_Ack(asRF_TX_Que[i].uiDst_Addr, 0, ulSysTime);

			#if defined(DEBUG)
			PRINTF("Rx Ack,seq:%d!\r\n", asRF_TX_Que[i].ucSeq);
			#endif
			
			if(RF_TX_QUE_OPT_SUPER == asRF_TX_Que[i].ucOption)
			{
				SuperData.ucStatus = ACT_IN_IDLE;
			}
			
			memset((u8*)&asRF_TX_Que[i], 0x00, sizeof(RF_TX_QUE));

//			TestFlg = 0;  //for test
			
			
			
			return i;
		}

	}

	return i;
}

u8 ACK_Send(u8 ucSeq, u32 ulSysTime)//(u8* NwkData)
{
	Ack.ucFrame_Ctrl = ACK_TYPE;
	Ack.ucDev_Seq    = ucSeq;
	Ack.ultick       = ulSysTime;
	Ack.ucPend       = TRUE;
	
	return 0;
}

void Period_Update_Event(u32 ulSysTime)
{
	#if defined(COO) || defined(Router)
	
	static u32 ulPre = 0;

	if((u32)(ulSysTime - ulPre) > PERIOD_S)   //1s
	{
		ulPre = ulSysTime;

		Poll_Buffer_Age_Increase();
		RoutTab_Age_Increase();

		#if defined(Router)

		static u8 cnt = 0;

		if(Nv_Para.ucHeartBeat_Gap != 0)   //
		{

			if( cnt >= Nv_Para.ucHeartBeat_Gap )
			{
				cnt = 0;
				
				if(Nv_Para.ucJoin_Flag)
				{
					Router_HeartBeat_Report(ulSysTime);
					
					#if defined(DEBUG)
					PRINTF("HeartBeat Report!\r\n");
					#endif
				}
				
			}
			else
			{
				cnt++;
			}
			
		}
	
		#endif
	}

	#endif


}

void Poll_Buffer_Age_Increase( void )
{
	#if defined(COO)||defined(Router)
	
	u8	i;

	for ( i=0; i<POLL_BUFFER_SIZE; i++ )
	{
		if ( asPoll_Buffer[i].ucMessage_Length > 0 )
		{
			if ( ( asPoll_Buffer[i].ucAge >= Nv_Para.ucPoll_TimeOut) )
			{
				memset( (void*)&asPoll_Buffer[i], 0, sizeof( POLL_BUFFER_Typedef) );
			}
			else
			{			
				asPoll_Buffer[i].ucAge ++;
			}
		}
	}

	#endif
}

void RoutTab_Age_Increase( void )
{
	
	#if defined(COO)|defined(Router)

	u8	i;
	u8 MaxAge = 0;       //sec
	
	if( Nv_Para.ucPoll_TimeOut > 80 )
	{

		MaxAge = 0xFF;
	}
	else
	{
		MaxAge = 3*Nv_Para.ucPoll_TimeOut;
	}

	for ( i=0; i<MAX_ROUTABLE; i++ )
	{
		if(RoutTable[i].uiAddr != 0x00)	
		{
		
			if(OFFLINE_BIT == (RoutTable[i].ucNode_Type & OFFLINE_BIT))
			{
				if(RoutTable[i].ucAge > 5)  //5 sec
				{
					
					memset( (void*)&RoutTable[i], 0, sizeof( Router_Table_Typedef) );
				}
			}
			
			if((RoutTable[i].ucNode_Type & ZED_TYPE) == ZED_TYPE)
			{

				if(RoutTable[i].ucAge >= MaxAge)
				{
					RoutTable[i].ucAge = MaxAge;
//					memset( (void*)&RoutTable[i], 0, sizeof( Router_Table_Typedef) );
//					Update_Node_to_Flash();
//					Del_Node_From_Flash(i);
				}
				else
				{
					
					RoutTable[i].ucAge ++;
				}
			}
			else      //Router
			{
				
				if(RoutTable[i].ucAge >= 0xFF) //65536s = 18 h
				{
					RoutTable[i].ucAge = 0xFF;
//						Add_Node_To_Flash(i);       //
				}
				else
				{
					
					RoutTable[i].ucAge ++;
				}

			}
			
		}
	}

	#endif
}


u8 RF_Tx_Inc(void)
{
	if( SysPara.ucLastTXQuePos != RF_TX_QUE_SIZE )
	{
		if(asRF_TX_Que[SysPara.ucLastTXQuePos].ucLength != 0)
		{
			
			asRF_TX_Que[SysPara.ucLastTXQuePos].ucTried_Times ++;     
		}
	}

	return 0;
}

u8 Relay_RF_Data(u8* RF_Data)
{
	LoRa_Data_Typedef* RelayData = (LoRa_Data_Typedef*)RF_Data;

	RelayData->uiDst_Addr = COO_NwkAddr;
	RelayData->ucHop     += 1; 
	RelayData->ucDev_Seq  = SysPara.ucPacketSeq;

	return 0;
}


/************************************************************************************/
/*	Send out the data buffered in asRF_TX_Que[] by RF								*/
/*	Parameters																		*/
/*		None																		*/
/*	return																			*/
/*		process result																*/
/*	Note																			*/
/*		1.	For local issued frame, always send the earliest frame first			*/
/*		2.	For relaying frames, send the frame that reaches the retry gap. If		*/
/*			multiple frame meets the condition, send the one that tried less times	*/
/************************************************************************************/
u8	TX_RF_Data( u32 ulSysTime )
{
	u8			i; //p, ucSum		= 0;
	u8			j				= 0xFF;
	u16			uiTime			= ulSysTime;//HW_RTC_Tick2ms(HW_RTC_GetTimerValue()); //systick;
	u16			uiGap			= 0;

	////  ack send  add by dozen 20170815///

	if(Ack.ucPend)
	{		
//		Radio.Send((u8*)&Ack, sizeof(Ack));

		if((u32)(ulSysTime - Ack.ultick) > RF_MIN_GAP_MS)
		{
			if(Func_t->RF_Tx((u8*)&Ack, 2))    //send failed
			{
				return 0xFF;
			}
//			Func_t->RF_Tx((u8*)&Ack, 2);		
			Ack.ucPend = FALSE;

			#if defined(DEBUG)
			
			PRINTF("Seq:%d\r\n",Ack.ucDev_Seq);
			
			#endif
			
			memset((u8*)&Ack, 0x00, sizeof(Ack));

			return 2;
		}
	
	}
	
	for ( i=0; i<RF_TX_QUE_SIZE; i++ )
	{
		if ( ( asRF_TX_Que[i].ucLength >= LORA_PACK_HEAD ) &&
			 ( asRF_TX_Que[i].ucLength < 128 ) &&
			 ( ( asRF_TX_Que[i].ucOption & RF_TX_QUE_OPT_RELAY ) == 0 ) )	// local issued frame
		{
			if ( asRF_TX_Que[i].ucActive & RF_TX_QUE_ACT_IN_PROCESS )
			{
				if ( (u16)( uiTime - asRF_TX_Que[i].uiLast_Try_Time ) > asRF_TX_Que[i].uiRetry_Gap )
					j	= i;
				else
					j	= 0xFF;

				break;
			}

			if ( (u16)( uiTime - asRF_TX_Que[i].uiLast_Try_Time ) > asRF_TX_Que[i].uiRetry_Gap )
			{
				if ( (u16)( uiTime - asRF_TX_Que[i].uiLast_Try_Time ) > uiGap )
				{
					j		= i;
					uiGap	= (u16)( uiTime - asRF_TX_Que[i].uiLast_Try_Time );
				}
			}
		}
	}

	if ( j < RF_TX_QUE_SIZE )
	{

		asRF_TX_Que[j].ucActive	|= RF_TX_QUE_ACT_IN_PROCESS;

//		Radio.Send(asRF_TX_Que[j].aucData, asRF_TX_Que[j].ucLength);

		if(Func_t->RF_Tx(asRF_TX_Que[j].aucData, asRF_TX_Que[j].ucLength))
		{
			return 0xff;
		}
	
		if( ACK_BIT != (asRF_TX_Que[j].aucData[0] & ACK_BIT))
		{
			memset( (void*)&asRF_TX_Que[j], 0, sizeof(RF_TX_QUE) );
			return	j;
		}

		asRF_TX_Que[j].uiLast_Try_Time	=  ulSysTime;//HW_RTC_Tick2ms(HW_RTC_GetTimerValue());//systick;
		

		if ( asRF_TX_Que[j].ucTried_Times >= RF_TX_MAX_RETRY_TIMES )
		{
			SysPara.ucLastTXQuePos	= RF_TX_QUE_SIZE;
			
			//for test
//			fail++;
//			TestFlg = 0;
			
			Func_t->App_Ack(asRF_TX_Que[j].uiDst_Addr, 1, ulSysTime);

			HandleError.TxCnt++;    //add 20170821
			
			memset( (void*)&asRF_TX_Que[j], 0, sizeof(RF_TX_QUE) );
			return	j;
			
		}
		else
		{
			SysPara.ucLastTXQuePos			 = j;

			if(asRF_TX_Que[j].ucOption == RF_TX_QUE_OPT_RT)
			{
				uiGap = 400;
//				srand1(ulSysTime);
//				uiGap = randr(500, 2000);
			}
			else
			{				
				srand1(ulSysTime);
				uiGap = randr(500, 2000);
			}
			
			asRF_TX_Que[j].uiRetry_Gap += uiGap;

			// for debug
			#if defined(DEBUG)
			PRINTF("Gap:%d,try:%d\r\n",asRF_TX_Que[j].uiRetry_Gap, asRF_TX_Que[j].ucTried_Times);
			#endif
								
		}
	}

	return	i;
}

u8	Add_RF_Frame_To_TX_Que( u8 * pFrame, u8 ucLength, u8 ucOpt, u32 ulSysTime, u16 uiDst_Addr, u16 uiSrc_Addr, u8 ucSeq)
{
	u8				i; //j, ucSum;
	u16				uiGap	= RF_MIN_GAP_MS;	// 0; try to send 3ms later


	if ( ( ucLength < LORA_PACK_HEAD ) || ( ucLength > PHY_MAX_LEN) )	// wrong length
		return	0xFF;

	if ( ucOpt & RF_TX_QUE_OPT_RELAY )
	{
		if (ZED_TYPE == SysPara.ucDevType)
		{
			return	0xFF;
		}
	}

	if ( ucOpt & RF_TX_QUE_OPT_DELAY )
	{
		//50-999ms
//		uiGap = Radio.Random()%1000;		// ZED would call this function so no delay will be generated for ZED

//		if ( uiGap < 0x32 )
//		{
//			uiGap	= 0x32;
//		}
		srand1(ulSysTime);
		uiGap = randr(100, 800);

		#if defined(DEBUG)
//		PRINTF("Delay Time:%d\r\n", uiGap);
		#endif
			
	}

	for( i=0; i<RF_TX_QUE_SIZE; i++ )						// find a empty position for new frame
	{
		if ( ( asRF_TX_Que[i].ucLength < LORA_PACK_HEAD ) ||
			 ( asRF_TX_Que[i].ucLength > PHY_MAX_LEN ) )
			break;
	}

	if ( i < RF_TX_QUE_SIZE )
	{
		if ( ( ( ucOpt & RF_TX_QUE_OPT_RELAY ) == 0 ))		// not relay frame, increase ucPacket_Seq )			// not Neighbor Table Exchange Frame
			SysPara.ucPacketSeq ++;

		if((ucOpt == RF_TX_QUE_OPT_SUPER)
			||(ucOpt == RF_TX_QUE_OPT_JOIN)
			||(ucOpt == RF_TX_QUE_OPT_RT))
		{
			asRF_TX_Que[i].uiSrc_Addr		= uiSrc_Addr;
			asRF_TX_Que[i].uiDst_Addr       = uiDst_Addr;
			asRF_TX_Que[i].ucLength			= ucLength;
			asRF_TX_Que[i].ucSeq			= ucSeq;
			asRF_TX_Que[i].uiRetry_Gap		= uiGap;
			asRF_TX_Que[i].ucTried_Times	= 0;
			asRF_TX_Que[i].ucOption			= ucOpt;
			asRF_TX_Que[i].uiLast_Try_Time	= ulSysTime; //HW_RTC_Tick2ms(HW_RTC_GetTimerValue());//systick;
			asRF_TX_Que[i].ucActive			= 0;
			memcpy( asRF_TX_Que[i].aucData, (u8*)pFrame, ucLength );
		}
		else
		{
			LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&pFrame[1];
		
			asRF_TX_Que[i].uiSrc_Addr		= LoRa->uiSrc_Addr;
			asRF_TX_Que[i].uiDst_Addr       = LoRa->uiDst_Addr;
			asRF_TX_Que[i].ucLength			= ucLength;
			asRF_TX_Que[i].ucSeq			= LoRa->ucDev_Seq;
			asRF_TX_Que[i].uiRetry_Gap		= uiGap;
			asRF_TX_Que[i].ucTried_Times	= 0;
			asRF_TX_Que[i].ucOption			= ucOpt;
			asRF_TX_Que[i].uiLast_Try_Time	= ulSysTime;//HW_RTC_Tick2ms(HW_RTC_GetTimerValue());//systick;
			asRF_TX_Que[i].ucActive			= 0;
			memcpy( asRF_TX_Que[i].aucData, (u8*)pFrame, ucLength );
		}
		

//		PRINTF("Len:%d,Gap:%d,time:%d\r\n",ucLength,uiGap, asRF_TX_Que[i].uiLast_Try_Time);
		
		if ( ucOpt & RF_TX_QUE_OPT_POLL_MSG )
		{
		
			asRF_TX_Que[i].ucTried_Times	= RF_TX_MAX_RETRY_TIMES - 2;
		}
		else if ( ucOpt & RF_TX_QUE_OPT_RT )
		{
		
			asRF_TX_Que[i].ucTried_Times	= RF_TX_MAX_RETRY_TIMES - 1;
		}
		else
		{
			
		}

		return	i;
	}

	return 0xFF;
}


u8 Process_App_Cmd(LoRa_Data_Typedef* LoRaData, u16 size, s16 rssi, s8 snr, u32 ulSysTime)
{
	
#if defined(COO)
	
	if( APP_DATA_TRANS == LoRaData->ucLoRa_Cmd )
	{	
//		Uart_Send((u8*)LoRaData, size);
		Func_t->RF_Rec(LoRaData->ucPayload, (size - LORA_PACK_HEAD), LoRaData->uiSrc_Addr, ulSysTime);
			
//		Func_t->RF_Rec((u8*)LoRaData, size, LoRaData->uiSrc_Addr);
	}
	else if( POLL_CMD == LoRaData->ucLoRa_Cmd )
	{
		ZED_Poll_Resp(LoRaData, rssi, ulSysTime);
	}
	/*
	else if((DEL_CMD == LoRaData->ucLoRa_Cmd)||(DEL_RESP_CMD == LoRaData->ucLoRa_Cmd))          //node  req leave 
	{
		u8 i;

		if(DEL_CMD == LoRaData->ucLoRa_Cmd)
		{
			u8 RespData = 0;
			
			i = Build_Cmd_Resp(LoRaData, &RespData, 1, RadioSendBuffer);
			Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
		}
		
		i = Get_NwkAddr_Entry(LoRaData->uiSrc_Addr);

		if(i < MAX_ROUTABLE)
		{
			
			Func_t->Node(&RoutTable[i].sNode_Addr, RoutTable[i].uiAddr, RoutTable[i].ucNode_Type, OPT_DEL);			
			memset((u8*)&RoutTable[i], 0x00, sizeof(Router_Table_Typedef));

			#if defined(NV)
//			Update_Node_to_Flash();		
			Del_Node_From_Flash(i);
			#endif
			
			#if defined(DEBUG)
			PRINTF("Delete NodeTbl!\r\n");
			#endif
		}
		
	}
	*/
	else if(HEARTBEAT_CMD ==  LoRaData->ucLoRa_Cmd )
	{

		if(ZED_TYPE == (LoRaData->ucPayload[0]&ZED_TYPE))
		{
			if(0xFF == Get_NwkAddr_Entry(LoRaData->uiSrc_Addr))  //not find node
			{
				
				Add_Node_RouteTbl((Dev_Addr*)&LoRaData->ucPayload[1], LoRaData->uiSrc_Addr, 0, LoRaData->ucPayload[0]|RELAY_TYPE_BIT);
				
			}
		}
	}
	else if(((LoRaData->ucLoRa_Cmd & End_CMD) >= Start_CMD)
			&&((LoRaData->ucLoRa_Cmd & End_CMD) <= End_CMD))
	{
		if((LoRaData->ucLoRa_Cmd & READ_CMD) == READ_CMD)
		{
			
			Func_t->Cmd(LoRaData->uiSrc_Addr, (CMD_Enum)(LoRaData->ucLoRa_Cmd & End_CMD), LoRaData->ucPayload, 1, OPT_READ, 0);
		}
		else if((LoRaData->ucLoRa_Cmd & WRITE_CMD) == WRITE_CMD)
		{
			
			Func_t->Cmd(LoRaData->uiSrc_Addr, (CMD_Enum)(LoRaData->ucLoRa_Cmd & End_CMD), LoRaData->ucPayload, 1, OPT_WRITE, 0);
		}
		else
		{
		}

		if((LoRaData->ucLoRa_Cmd & End_CMD) == Del_CMD)
		{
			u8 i;
			u8 RespData = 0;
			
			if((LoRaData->ucLoRa_Cmd & RESP_CMD) != RESP_CMD)
			{
				i = Build_Cmd_Resp(LoRaData, &RespData, 1, RadioSendBuffer);
				Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);
			}
			
			i = Get_NwkAddr_Entry(LoRaData->uiSrc_Addr);

			if(i < MAX_ROUTABLE)
			{
				
				Func_t->Node(&RoutTable[i].sNode_Addr, RoutTable[i].uiAddr, RoutTable[i].ucNode_Type, OPT_DEL);			
				memset((u8*)&RoutTable[i], 0x00, sizeof(Router_Table_Typedef));

				#if defined(NV)
	//			Update_Node_to_Flash();		
//				Del_Node_From_Flash(i);
				Add_Node_To_Flash(i);
				#endif
				
				#if defined(DEBUG)
				PRINTF("Delete NodeTbl!\r\n");
				#endif
			}
		}
		
		
	}
	else
	{
	  	
		#if defined(DEBUG)
	    PRINTF("OnRxDone\n");
	    PRINTF("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
		#endif
	}

#else
	

	if(Nv_Para.ucJoin_Flag != NET_FLG)
	{
		return 1;
	}
	
	if( APP_DATA_TRANS == LoRaData->ucLoRa_Cmd )
	{	

//		Uart_Send((u8*)LoRaData, size);
		
//		Func_t->RF_Rec(LoRaData->ucPayload, (size - LORA_PACK_HEAD));
		
		Func_t->RF_Rec(LoRaData->ucPayload, (size - LORA_PACK_HEAD), LoRaData->uiSrc_Addr, ulSysTime);
		
	}
	/*
	else if((DEL_CMD == LoRaData->ucLoRa_Cmd)||(DEL_RESP_CMD == LoRaData->ucLoRa_Cmd))
	{
		
		u8 i;
		
		if(DEL_CMD == LoRaData->ucLoRa_Cmd)
		{
			u8 RespData = 0;
			
			i = Build_Cmd_Resp(LoRaData, &RespData, 1, RadioSendBuffer);
			Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, RF_TX_QUE_OPT_DELAY, ulSysTime, 0, 0, 0);
		}
	

		Nv_Para.ucJoin_Flag = 0x00;
		Nv_Para.uiNwkAddr   = 0x0000;
		memset((u8*)&Nv_Para.sHost_Addr, 0x00, sizeof(Dev_Addr));

		SysPara.ucJoinStatus = LEAVE_STATUS;

		#if defined(NV)
		Write_NV();
		#endif
		
		#if defined(DEBUG)
		PRINTF("Leave Net!\r\n");
		#endif
	}
	*/
	
	else if(((LoRaData->ucLoRa_Cmd & End_CMD) >= Start_CMD)
			&&((LoRaData->ucLoRa_Cmd & End_CMD) <= End_CMD))
	{
		u8 i;
		
		if( (LoRaData->ucLoRa_Cmd & READ_CMD) == READ_CMD )
		{		
			i = Process_Cmd((LoRaData->ucLoRa_Cmd & End_CMD), LoRaData->ucPayload, 1, OPT_READ);
		}
		else if( (LoRaData->ucLoRa_Cmd & WRITE_CMD) == WRITE_CMD )
		{
			if( (LoRaData->ucLoRa_Cmd & RESP_CMD) != RESP_CMD )
			{				
				i = Process_Cmd((LoRaData->ucLoRa_Cmd & End_CMD), LoRaData->ucPayload, 1, OPT_WRITE);
			}
			else
			{
				i = NO_PARA;

				if( (LoRaData->ucLoRa_Cmd & End_CMD) == Del_CMD )
				{
					SysPara.ucJoinStatus = Leave;
				}

			}
		}
		else
			{
			}
		

		if(i == 0)
		{
			i = Build_Cmd_Resp(LoRaData, LoRaData->ucPayload, 1, RadioSendBuffer);
			Add_RF_Frame_To_TX_Que(RadioSendBuffer, i, 0, ulSysTime, 0, 0, 0);  //RF_TX_QUE_OPT_DELAY

			#if defined(DEBUG)
				PRINTF("CMD Resp!\r\n");
			#endif
		}
	}
	
	#if defined(ZED)
	
	else if( POLL_RESP_CMD == LoRaData->ucLoRa_Cmd )
	{	
		if(Poll_Ack.ucSeq == LoRaData->ucDev_Seq)
		{
			Poll_Ack.ucSeq       = 0;
			Poll_Ack.ucFail_Time = 0;
			Poll_Ack.ucStatus    = IDLE_STATUS;

			
			#if defined(DEBUG)
			PRINTF("Poll Seq Correct!\n");
			#endif
		}
		else
		{
			
			#if defined(DEBUG)
			PRINTF("Resp Seq:%d!\n",LoRaData->ucDev_Seq );
			#endif
		}
	}
	
	#endif
		
	else
	{
	  
		#if defined(DEBUG)
		PRINTF("OnRxDone\n");
		PRINTF("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
		#endif
	}

		
	#if defined(ZED)
	
	if( POLL_RESP_CMD != LoRaData->ucLoRa_Cmd )
	{
		if((LoRaData->ucLoRa_Cmd & End_CMD) != Del_CMD )
		{
			if(Nv_Para.ucJoin_Flag)
			{
				Poll_Ack.ucSeq = SysPara.ucPacketSeq;
				Poll_Ack.ucStatus = POLL_WAITING_ACK;
				ZED_Data_Req(RF_TX_QUE_OPT_DELAY, ulSysTime);

				#if defined(DEBUG)
				PRINTF("Pro Seq:%d,Staus:%d!\n", Poll_Ack.ucSeq, Poll_Ack.ucStatus);
				#endif
			}
			else
			{
				memset((u8*)&Poll_Ack, 0x00, sizeof(Poll_Ack));
			}
		}		
		
	}	
	
	#endif
		
	
	
#endif

	return 0;
	
}

u8 Process_Cmd(u8 ucCmd, u8* CmdValue, u8 ucLen,  u8 ucOpt )
{
	if(ucCmd == HeartBeat_Gap_CMD)
	{
		if(ucOpt == OPT_READ)
		{
			*CmdValue = Nv_Para.ucHeartBeat_Gap;

			#if defined(DEBUG)
				PRINTF("Read HeartBeat Gap:%d!\r\n",Nv_Para.ucHeartBeat_Gap);
			#endif

			return 0;
		}
		else if(ucOpt == OPT_WRITE)
		{
			Nv_Para.ucHeartBeat_Gap = *CmdValue;

			#if defined(DEBUG)
				PRINTF("Write HeartBeat Gap:%d!\r\n",Nv_Para.ucHeartBeat_Gap);
			#endif

			#if defined(NV)

			Write_NV();
			
			#endif

			return 0;
		}
		else
		{
			return 1;
		}
	}
	else if(ucCmd == Poll_Gap_CMD)
	{
		if(ucOpt == OPT_READ)
		{
			*CmdValue = Nv_Para.ucPoll_TimeOut;

			#if defined(DEBUG)
				PRINTF("Read Poll Gap:%d!\r\n",Nv_Para.ucPoll_TimeOut);
			#endif

			return 0;
		}
		else if(ucOpt == OPT_WRITE)
		{
			Nv_Para.ucPoll_TimeOut = *CmdValue;
			
			#if defined(DEBUG)
				PRINTF("Write Poll Gap:%d!\r\n",Nv_Para.ucPoll_TimeOut);
			#endif

			#if defined(NV)

			Write_NV();
			
			#endif

			return 0;
		}
		else
		{
			return 1;
		}
	}
	else if(ucCmd == Del_CMD)
	{
		
		if(ucOpt == OPT_WRITE)
		{
			
			SysPara.ucJoinStatus = Leave;
			
			#if defined(DEBUG)
			PRINTF("Leave Net!\r\n");
			#endif
			
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else if(ucCmd == Reset_CMD)
	{
		
		if(ucOpt == OPT_WRITE)
		{
			
			#if defined(DEBUG)
				PRINTF("Reset!\r\n");
			#endif
			
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else if(ucCmd == Default_CMD)
	{
		
		if(ucOpt == OPT_WRITE)
		{
		
//			AwarePro_Default();
			
			#if defined(DEBUG)
				PRINTF("Default!\r\n");
			#endif
			
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}
}

/////////////////  Node Table  ////////////



u8 Get_Empty_RoutTable(Dev_Addr* Addr)
{

#if  defined(COO)|| defined(Router)
	
	u8 i;
	u8 Index = 0;
	u8 MaxAge = 0;

	i = Get_Node_Entry(Addr);
	
	if( i < MAX_ROUTABLE)      //find node in TBL
	{
		return i;
	}
	else
	{
		
		for(i = 0; i < MAX_ROUTABLE; i++)
		{
		
			if(memcmp(&RoutTable[i].sNode_Addr, &NullAddr, sizeof(Dev_Addr)) == 0)
			{
				return i;
			}
			else                           //find max age node
			{
			
				if( RoutTable[i].ucAge > MaxAge )
				{
					MaxAge = RoutTable[i].ucAge;
					Index = i;
				}
				
			}
			
		}
	}

	if(MaxAge == 0xFF)
	{		
		memset(&RoutTable[Index], 0, sizeof(Router_Table_Typedef));
	}
	
	return Index;

#else
	
	return 0;

#endif
	
}


u8	Get_Node_Entry( Dev_Addr* Addr )
{
	
#if  defined(COO) || defined(Router)
	u8	i;

//	if ( Addr->aucAddr[0] != 0L )
	if(memcmp((u8*)Addr, &NullAddr, sizeof(Dev_Addr)) != 0)
	{ 
		for ( i = 0; i<MAX_ROUTABLE; i++ )
		{
			if(memcmp(&RoutTable[i].sNode_Addr, Addr, sizeof(Dev_Addr)) == 0)
			{
				return	i;
			}
		}
	}

#endif

	return	0xFF;
}

u8	Get_NwkAddr_Entry( u16 NwkAddr )
{
	
#if  defined(COO)|| defined(Router)
	u8	i;

	if ( NwkAddr != 0L )
	{ 
		for ( i = 0; i< MAX_ROUTABLE; i++ )
		{
			if(NwkAddr == RoutTable[i].uiAddr)
			{
				return	i;
			}
		}
	}

#endif

	return	0xFF;
}

u8 Poll_Empty_RoutTbl(void)
{

	#if  defined(COO)|| defined(Router)
		
	u8 i;

	for(i = 0; i < MAX_ROUTABLE; i++)
	{
		if(memcmp(&RoutTable[i].sNode_Addr, &NullAddr, sizeof(Dev_Addr)) == 0)
		{
			return i;
		}
	}

	return i;

	#else

	return 0;

	#endif

}

u8	Get_PareNode_Entry( Dev_Addr* Addr )
{
	
#if  defined(ZED) || defined(Router)
	u8	i;

	if ( Addr->aucAddr[0] != 0L )
	{ 
		for ( i = 0; i<MAX_PARE_TBL; i++ )
		{
			if(memcmp(&PareTbl[i].sPare_Addr, Addr, sizeof(Dev_Addr)) == 0)
			{
				return	i;
			}
		}
	}

#endif

	return	0xFF;
}

u8 Get_Empty_PareTable(Dev_Addr* Addr)
{

#if  defined(ZED)|| defined(Router)
	
	u8 i;

	i = Get_PareNode_Entry(Addr);
	
	if( i < MAX_PARE_TBL)      //find node in TBL
	{
		return i;
	}
	else
	{
		
		for(i = 0; i < MAX_PARE_TBL; i++)
		{
		
			if(memcmp(&PareTbl[i].sPare_Addr, &NullAddr, sizeof(Dev_Addr)) == 0)
			{
				return i;
			}
			
		}
	}

	return i;

#else
	
	return 0;

#endif
	
}

u8 Add_Node_RouteTbl(Dev_Addr* sNodeAddr, u16 uiNodeAddr, s8 Rssi, u8 ucDev_Type)
{
	#if defined(COO) || defined(Router)
	u8 i;
				
	i = Get_Empty_RoutTable(sNodeAddr);  //ZED node

	memcpy(&RoutTable[i].sNode_Addr, sNodeAddr, sizeof(Dev_Addr));
	RoutTable[i].uiAddr    = uiNodeAddr;
	RoutTable[i].cRssi     = Rssi;
	RoutTable[i].ucNode_Type = ucDev_Type;
	RoutTable[i].ucAge      = 0;
	RoutTable[i].ucSeq      = 0;

	return i;

	#else

	return 0xFF;

	#endif
	
}

u8 Update_NwkAddr_Table(u16 Addr, s16 rssi)
{

#if  defined(COO)|| defined(Router)

	u8 i = 0xFF;

	i = Get_NwkAddr_Entry(Addr);
	if( i < MAX_ROUTABLE )
	{
		RoutTable[i].cRssi = rssi;
		RoutTable[i].ucAge = 0;
	}
	
	return 0;

#else

	return 0;

#endif

	
}

//////////////// frame  ////////////////

u8 Build_LoRa_App_Frame(u8* pRawData, u8 ucLen, u16 uiDstAddr, u8 ucSerSeq, u8* pFramebuf)
{

	u8 len = ucLen + LORA_PACK_HEAD;
	
	if(ucLen > LORA_PAYLOAD_MAX_LEN)
	{
		return 0;
	}

	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&pFramebuf[1];

	pFramebuf[0] = DATA_ACK_TYPE;

	LoRa->uiDst_Addr = uiDstAddr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = APP_DATA_TRANS;
	LoRa->ucHop      = 0;
	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = ucSerSeq;

	memcpy(LoRa->ucPayload, pRawData, ucLen);
	
	pFramebuf[len + 1] = Check_Sum(&pFramebuf[1], len);
	
	return (len + 2);
}

u8 Build_LoRa_RT_Frame(u8* pRawData, u8 ucLen, u8* pFramebuf)
{

	u8 len = ucLen + RT_PAYLOAD_HEAD_LEN;
	
	if(ucLen > RT_PAYLOAD_MAX_LEN)
	{
		return 0;
	}

	LoRa_RT_Data_Typedef* LoRa = (LoRa_RT_Data_Typedef*)&pFramebuf[1];

	pFramebuf[0] = EXT_PID_TYPE | ACK_BIT;

	memcpy(&LoRa->sSrc_Addr, SysPara.pLocalAddr, sizeof(Dev_Addr));
	LoRa->ucLoRa_Cmd = APP_DATA_TRANS;
	LoRa->ucSeq  = SysPara.ucPacketSeq;

	memcpy(LoRa->ucPayload, pRawData, ucLen);
	
	pFramebuf[len + 1] = Check_Sum(&pFramebuf[1], len);
	
	return (len + 2);
}


u8 Build_MAC_Frame(u8* Nwk_Frame, u8 len, u8 Frame_Type, u8* MAC_Frame)
{
	if(len > MAC_DATA_MAX_LEN)
	{
		return 0;
	}
	
	MAC_Frame[0] = Frame_Type;
	memcpy(&MAC_Frame[1], Nwk_Frame, len);
	MAC_Frame[len+1] = Check_Sum(&MAC_Frame[1], len);

	return (len + 2);
}

u8 Build_Cmd_Req(u16 uiDstAddr, u8 ucCmd, u8* pucVal, u8 ucLen, u8* pFramebuf)
{
	u8 len = LORA_PACK_HEAD + ucLen;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&pFramebuf[1];
//	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	pFramebuf[0]           = DATA_NOACK_TYPE;
	
	LoRa->uiDst_Addr = uiDstAddr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = ucCmd;
	LoRa->ucHop      = 0;

	LoRa->ucDev_Seq  = SysPara.ucPacketSeq;
	LoRa->ucSer_Seq  = 0;

	memcpy(LoRa->ucPayload, pucVal, ucLen);

	pFramebuf[len + 1] = Check_Sum(&pFramebuf[1], len);

	len += 2;
	
//	Add_RF_Frame_To_TX_Que(RadioSendBuffer, len, 0, ulSysTime, 0, 0, 0);

	return (len);  //FrameType + check
}


u8 Build_Cmd_Resp(LoRa_Data_Typedef* CmdData, u8* pucAppData, u8 ucAppLen, u8* Respbuf)
{
	//resp ZED no data
	u8 len = LORA_PACK_HEAD + ucAppLen;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&Respbuf[1];
	
	Respbuf[0]  = DATA_NOACK_TYPE;
	LoRa->uiDst_Addr = CmdData->uiSrc_Addr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = CmdData->ucLoRa_Cmd | RESP_CMD;
	LoRa->ucDev_Seq  = CmdData->ucDev_Seq;
	LoRa->ucSer_Seq  = 0;

	memcpy(LoRa->ucPayload, pucAppData, ucAppLen);
//	LoRa->ucPayload[0]  =  0;

	Respbuf[len + 1] = Check_Sum(&Respbuf[1], len);

	return (len + 2);
}



u8 Build_Zero_Poll_Resp(LoRa_Data_Typedef* PollData)
{
	//resp ZED no data
	u8 len = LORA_PACK_HEAD + 1;
	LoRa_Data_Typedef* LoRa = (LoRa_Data_Typedef*)&RadioSendBuffer[1];
	RadioSendBuffer[0]           = DATA_NOACK_TYPE;
	
	LoRa->uiDst_Addr = PollData->uiSrc_Addr;
	LoRa->uiSrc_Addr = Nv_Para.uiNwkAddr;
//	memcpy(&pRadioSendBuffer->sDst_Addr, &PollData->sSrc_Addr, sizeof(Dev_Addr));
//	memcpy(&pRadioSendBuffer->sSrc_Addr, &DeviceAddr, sizeof(Dev_Addr));

	LoRa->ucPID      = Nv_Para.ucPID;
	LoRa->ucLoRa_Cmd = POLL_CMD | RESP_CMD;
	LoRa->ucHop      = 0;

	LoRa->ucDev_Seq  = PollData->ucDev_Seq;
	LoRa->ucSer_Seq  = 0;

//	memcpy(&JoinData->Payload, pRadioSendBuffer->Payload, );
	LoRa->ucPayload[0]  =  0;

	RadioSendBuffer[len + 1] = Check_Sum(&RadioSendBuffer[1], len);

	return (len + 2);  //FrameType + check
}

////////check or CRC //////////////////
u8 Check_Sum(u8* buf, u8 len)
{
	u8 i;
	u8 Sum = 0;

	for(i = 0; i < len; i++)
	{
		Sum += buf[i];
	}

	return Sum;
}

u8 Check_NV_Para(void)
{
	if((Nv_Para.ucStx == NV_STX)
		&&(Nv_Para.ucEnd == NV_END))
	{
		u8 Sum = 0;
		Sum = Check_Sum(&Nv_Para.ucJoin_Flag, sizeof(Nv_Para)-3);

		if(Sum == Nv_Para.ucCheck)
		{
			return 0;
		}
	}
	
	return 1;
}


