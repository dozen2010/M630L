#include "LFwakeup.h"
#include "ccRFID_CRC8.h"
#include "Tim.h"

u8 LFByteP;
u8 LFBitP;
u8 LFByteLength;
u8 LFSendDat[32];


//********************************************************************
//函 数 名：void LF_Init(void)
//功    能：初始化LF所需定时器
//说    明：由用户自己编写
//输		入：无
//输		出：无
//********************************************************************
void LF_Init(void)
{
	
	MX_TIM22_Init();   //PWM 125K
	MX_TIM6_Init();   //timer 350uS sendbit
	
	LF_Reset();
	
}

void PWM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin  = LF_PWM_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(LF_PWM_PORT, &GPIO_InitStruct);	
	HAL_GPIO_WritePin(LF_PWM_PORT, LF_PWM_PIN, GPIO_PIN_SET);

	GPIO_InitStruct.Pin  = SPK_PWM_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(SPK_PWM_PORT, &GPIO_InitStruct);	

	HAL_GPIO_WritePin(SPK_PWM_PORT, SPK_PWM_PIN, GPIO_PIN_SET);
}




//********************************************************************
//函 数 名：void LF_Reset(void)
//功    能：
//说    明：
//输		入：无
//输		出：无
//********************************************************************
void LF_Reset(void)
{
    LFByteP = 0;
    LFBitP = 0;
    LFByteLength = 0;
    PWM_OFF;
    BITTIMESTOP;   
}

//********************************************************************
//函 数 名：void LF_SendBit(void)
//功    能：
//说    明：
//输		入：无
//输		出：无
//********************************************************************
void LF_SendBit(void)
{
	if(LFByteP==LFByteLength)
	{
		PWM_OFF;
		BITTIMESTOP;
	}
	else
	{
		if((LFSendDat[LFByteP]&0x80)==0x80)
		{
			PWM_ON;
		}
		else
		{
			PWM_OFF;
		}
		LFSendDat[LFByteP] <<= 1;
		LFBitP++;
		if(LFBitP==8)
		{
			LFBitP = 0; 
			LFByteP++;
		}
	}
}


//********************************************************************
//函 数 名：void LF_DataSend(u8 *Pdata,u8 length)
//功    能：
//说    明：
//输		入：u8 *Pdata,u8 length
//输		出：无
//********************************************************************
static void LF_DataSend(u8 *Pdata,u8 length)
{
 	u8 i,j,k,t;
	u8 SendBuffer[10];
	
	if(!(length == 4 || length == 5 || length == 8))
		return;
	
	BITTIMESTOP;
	switch(length)
	{
		case 4: //11位定位器地址

		case 5: //16位定位器地址
			for(i=0;i<(length-1);i++)
				SendBuffer[i] = Pdata[i];
			SendBuffer[length-1] = CRC8(Pdata,length-1);
			break;
			
		case 8:				//LF配置协议
			for(i=0;i<3;i++)
				SendBuffer[i] = Pdata[i];
		
			for(i=4;i<8;i++)
				SendBuffer[i] = Pdata[i-1];
		
			SendBuffer[3] = CRC8(Pdata,7);
			if(SendBuffer[3] ==CRC8( Pdata, 3))
				SendBuffer[3]++;
		
			break;
	}

	for(i=0;i<32;i++)
		LFSendDat[i] = 0;

	LFSendDat[0] = 0xFF;
	LFSendDat[1] = 0xFE;

	LFSendDat[2] = 0xAA;

	LFSendDat[3] = 0x96;
	LFSendDat[4] = 0x69;
//--------data-------------------
// Send 6 Bytes
// ------------------------------
// | 0X96 | DATA3 | DATA2 | DATA1 | DATA0 | CRC | 
// ------------------------------
	t = 5;
	k = 0;
	for(i=0;i<length;i++)
	{
		for(j=0;j<8;j++)
		{
			if ((SendBuffer[i] & 0X80)!=0) // Bit1
	 	 	{
				LFSendDat[t] += 0x02<<2*(3-k);
		 	}
			else //Bit0
		 	{
				LFSendDat[t] += 0x01<<2*(3-k);	 
		 	}
			SendBuffer[i]  <<= 1;	       
			k++;
			if(k==4)
			{
			  k = 0;
			  t++;
			}
		}
	}
	LFByteP = 0;
	LFBitP = 0;
	LFByteLength = t;
	BITTIMESTART;
}



//********************************************************************
//函 数 名：void LF_Awaken(u16 Address,LF_Typedef LF_Type)
//功    能：发送定位信号,当Type为LF_Normal时发送11位定位地址
//说    明：
//输		入：u16 Address,LF_Typedef LF_Type
//输		出：无
//********************************************************************
void LF_Awaken(u16 Address,LF_Typedef LF_Type)
{
	u8 CRC_DAT[4];
	u8 Addr_H,Addr_L;

	Addr_L = (u8)Address;
	Addr_H = (u8)(Address >> 8);	
	if(LF_Type == LF_Normal)
	{
		Addr_H = (Addr_H<<5)& 0xE0;
		Addr_H |= (Addr_L ^ 0XAA) & 0X1F;
		CRC_DAT[0]=0x96;
		CRC_DAT[1]=Addr_L;
		CRC_DAT[2]=Addr_H;	
		LF_DataSend(CRC_DAT,4);		
	}
	else
	{
		CRC_DAT[0] = 0x5A;
		CRC_DAT[1] = Addr_L;
		CRC_DAT[2] = Addr_H;
		CRC_DAT[3] = ((((Addr_L+Addr_H)^0xAA)&0x1F)<<3)|(LF_Type&0x07);
		LF_DataSend(CRC_DAT,5);  		
	}
}

//********************************************************************
//函 数 名：void LF_SendConfig(const u8 *data)
//功    能：发送6字节配置数据
//说    明：
//输		入：6字节配置数据
//输		出：无
//********************************************************************
void LF_SendConfig(const u8 *Data)
{
	u8 i,temp[7];
	temp[0] = 0x69;
    for(i=0;i<6;i++)
		temp[i+1] = Data[i];
	LF_DataSend(temp,8);
}


//////// TIM6 定时发送数据，每隔350us //////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance)
    {
		LF_SendBit();
    }
}

