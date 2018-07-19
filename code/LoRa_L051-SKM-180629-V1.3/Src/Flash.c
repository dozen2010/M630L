/*

Description: AwareTag Flash API 

License: 

Maintainer: Hangzhou AwareTag tech
*/
 /******************************************************************************
  * @file    Falsh.c
  * @author  dozen Yao
  * @version V0.0.1
  * @date    2017/09/21
  * @brief   AwareTag Flash API
  ******************************************************************************
  * @attention
  * User 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "Flash.h"


////////////////////  FLASH   //////////////////////

void Erase_Block_Flash(u32 Addr, u32 len)
{
	// usr code begin ///
	u32 i;
	//unlock
	HAL_FLASHEx_DATAEEPROM_Unlock();
	//erase
	
	for(i = 0; i < len; i += 4)
	{		
		HAL_FLASHEx_DATAEEPROM_Erase(Addr + i);
	}

	//Lock
	HAL_FLASHEx_DATAEEPROM_Lock();

	
	// usr code end ///
}



//u32 d = 0,d2 = 0;

void Write_Data_To_Flash(u32 Addr, u8* data, u32 len)
{
	
	// usr code begin ///
	u32 i = 0;
	
	//unlock
	HAL_FLASHEx_DATAEEPROM_Unlock();

	for(i = 0; i < len; i++)
	{	
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, (Addr + i), data[i]);
	}
	//Lock
	HAL_FLASHEx_DATAEEPROM_Lock();
		
	// usr code end ///

}

void Read_Data_From_Flash(u32 Addr, u8* data, u32 len)
{
	
	// usr code begin ///
	//unlock
	HAL_FLASHEx_DATAEEPROM_Unlock();

	//read
	memcpy((u32*)data, (const u32 *)Addr, len);

	//Lock
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	// usr code end ///

}



u8 Write_NV(void)
{
//	Nv_Para.ucNv_Flag = 1;
	Nv_Para.ucStx = NV_STX;
	Nv_Para.ucEnd = NV_END;
	Nv_Para.ucCheck = Check_Sum(&Nv_Para.ucJoin_Flag, sizeof(Nv_Para)-3);

	Erase_Block_Flash(DATA_EEPROM_BASE, NV_PARA_LEN);
	Write_Data_To_Flash(DATA_EEPROM_BASE, (u8*)&Nv_Para, sizeof(Nv_Para));
	
	return 0;
	
}

u8 Read_NV(void)
{
	
	Read_Data_From_Flash(DATA_EEPROM_BASE, (u8*)&Nv_Para, sizeof(Nv_Para));
	
	return 0;
	
}


u8 Delete_NV(void)
{	
	
	Erase_Block_Flash(DATA_EEPROM_BASE, NV_PARA_LEN);
	
	return 0;
}

#if defined(COO)||defined(Router)

u8 Add_Node_To_Flash(u8 Index)
{
	#if defined(COO) | defined(Router)
	
	u32 Addr;
	Addr = (NET_NODE_TABLE_ADDR + Index * sizeof(Router_Table_Typedef));
	
	Write_Data_To_Flash(Addr, (u8*)&RoutTable[Index], sizeof(Router_Table_Typedef));

	#endif
	
	return 0;
}

u8 Read_One_Node_From_Flash(u8 Index)
{
	u32 Addr;
	Addr = (NET_NODE_TABLE_ADDR + Index * sizeof(Router_Table_Typedef));
	
	Read_Data_From_Flash(Addr, (u8*)&RoutTable[Index], sizeof(Router_Table_Typedef));

	return 0;
}

u8 Del_Node_From_Flash(u8 Index)
{
	u32 Addr;
	Addr = (NET_NODE_TABLE_ADDR + Index * sizeof(Router_Table_Typedef));

	Erase_Block_Flash(Addr, sizeof(Router_Table_Typedef));

	return 0;
}


/// Init Para
u8 Read_Node_From_Flash(void)
{
	
	u32 Addr = NET_NODE_TABLE_ADDR;

	Read_Data_From_Flash(Addr, (u8*)RoutTable, MAX_ROUTABLE*sizeof(Router_Table_Typedef));

	return 0;
}

u8 Erase_NodeTable(void)
{
		
	Erase_Block_Flash(NET_NODE_TABLE_ADDR, MAX_ROUTABLE * sizeof(Router_Table_Typedef));	
	
	return 0;
}

#endif

/*
u8 Write_NV(NV_Type Type)
{
	// eeprom nv write  

	if(Join_Flg == Type)
	{	
		#if defined(Router)
		Write_Byte_To_Flash(NET_ONLINE_FLG_ADDR, Nv_Para.ucJoin_Flag);	
		#endif
	}
	else if(Host_Node_Addr == Type)
	{	
		#if defined(Router)
		Write_Data_To_Flash(NET_HOST_NODE_ADDR, (u8*)&Nv_Para.sHost_Addr, sizeof(Dev_Addr));
		#endif
	}
	else if(RF_CH == Type)
	{	
		Write_Byte_To_Flash(RF_CH_OR_FREQ_ADDR, Nv_Para.ucRF_Channel);
	}
	else if(RF_Rate == Type)
	{	
		Write_Byte_To_Flash(RF_DATA_RATE_ADDR, Nv_Para.ucRF_Rate);
	}
	else if(RF_Pwr == Type)
	{	
		Write_Byte_To_Flash(RF_PWR_ADDR, Nv_Para.ucRF_Pwr);
	}
	else if(Uart_BaudRate == Type)
	{	
		Write_Byte_To_Flash(UART_RATE_ADDR, Nv_Para.ucUart_Rate);
	}
	else if(Band_Width == Type)
	{	
		Write_Byte_To_Flash(BANDWIDTH_ADDR, Nv_Para.ucBand_Width);
	}
	else if(PID == Type)
	{	
		Write_Byte_To_Flash(NET_PID_ADDR, Nv_Para.ucPID);
	}
	else if(NwkAddr == Type)
	{	
		#if defined(Router)
		Write_Data_To_Flash(NWKADDR_ADDR, (u8*)&Nv_Para.uiNwkAddr, sizeof(u16));
		#endif
	}
	else if(Parent_NwkAddr == Type)
	{	
		#if defined(Router)
		Write_Data_To_Flash(PARENT_NWKADDR_ADDR, (u8*)&Nv_Para.uiPare_Addr, sizeof(u16));
		#endif
	}
	else if(All_Type == Type)
	{	
		#if defined(Router)
		Write_Byte_To_Flash(NET_ONLINE_FLG_ADDR, Nv_Para.ucJoin_Flag);		
		Write_Data_To_Flash(NET_HOST_NODE_ADDR, (u8*)&Nv_Para.sHost_Addr, sizeof(Dev_Addr));		
		Write_Data_To_Flash(NWKADDR_ADDR, (u8*)&Nv_Para.uiNwkAddr, sizeof(u16));
		Write_Data_To_Flash(PARENT_NWKADDR_ADDR, (u8*)&Nv_Para.uiPare_Addr, sizeof(u16));
		#endif
		Write_Byte_To_Flash(RF_CH_OR_FREQ_ADDR, Nv_Para.ucRF_Channel);
		Write_Byte_To_Flash(RF_DATA_RATE_ADDR, Nv_Para.ucRF_Rate);	
		Write_Byte_To_Flash(UART_RATE_ADDR, Nv_Para.ucUart_Rate);	
		Write_Byte_To_Flash(BANDWIDTH_ADDR, Nv_Para.ucBand_Width);	
		Write_Byte_To_Flash(RF_PWR_ADDR, Nv_Para.ucRF_Pwr);
		Write_Byte_To_Flash(NET_PID_ADDR, Nv_Para.ucPID);
	}
	else
	{
		
	}

	
	return 0;
}

u8 Read_NV(NV_Type Type)
{


	if(Join_Flg == Type)
	{
		#if defined(Router)
		Read_Data_From_Flash(NET_ONLINE_FLG_ADDR, &Nv_Para.ucJoin_Flag, 1);	
		#endif
	}
	else if(Host_Node_Addr == Type)
	{	
		#if defined(Router)
		Read_Data_From_Flash(NET_HOST_NODE_ADDR, (u8*)&Nv_Para.sHost_Addr, sizeof(Dev_Addr));
		#endif
	}
	else if(NwkAddr == Type)
	{	
		#if defined(Router)
		Read_Data_From_Flash(NWKADDR_ADDR, (u8*)&Nv_Para.uiNwkAddr, sizeof(u16));
		#endif
	}
	else if(Parent_NwkAddr == Type)
	{	
		#if defined(Router)
		Read_Data_From_Flash(PARENT_NWKADDR_ADDR, (u8*)&Nv_Para.uiPare_Addr, sizeof(u16));
		#endif
	}
	else if(RF_CH == Type)
	{	
		Read_Data_From_Flash(RF_CH_OR_FREQ_ADDR, &Nv_Para.ucRF_Channel, 1);
	}
	else if(RF_Rate == Type)
	{	
		Read_Data_From_Flash(RF_DATA_RATE_ADDR, &Nv_Para.ucRF_Rate, 1);
	}
	else if(Uart_BaudRate == Type)
	{	
		Read_Data_From_Flash(UART_RATE_ADDR, &Nv_Para.ucUart_Rate, 1);
	}
	else if(Band_Width == Type)
	{	
		Read_Data_From_Flash(BANDWIDTH_ADDR, &Nv_Para.ucBand_Width, 1);
	}
	else if(PID == Type)
	{	
		Read_Data_From_Flash(NET_PID_ADDR, &Nv_Para.ucPID, 1);
	}
	else if(All_Type == Type)
	{	
		#if defined(Router)
		Read_Data_From_Flash(NET_ONLINE_FLG_ADDR, &Nv_Para.ucJoin_Flag, 1);		
		Read_Data_From_Flash(NET_HOST_NODE_ADDR, (u8*)&Nv_Para.sHost_Addr, sizeof(Dev_Addr));		
		Read_Data_From_Flash(NWKADDR_ADDR, (u8*)&Nv_Para.uiNwkAddr, sizeof(u16));
		Read_Data_From_Flash(PARENT_NWKADDR_ADDR, (u8*)&Nv_Para.uiPare_Addr, sizeof(u16));
		#endif
		Read_Data_From_Flash(RF_CH_OR_FREQ_ADDR, &Nv_Para.ucRF_Channel, 1);
		Read_Data_From_Flash(RF_DATA_RATE_ADDR, &Nv_Para.ucRF_Rate, 1);	
		Read_Data_From_Flash(UART_RATE_ADDR, &Nv_Para.ucUart_Rate, 1);	
		Read_Data_From_Flash(BANDWIDTH_ADDR, &Nv_Para.ucBand_Width, 1);	
		Read_Data_From_Flash(NET_PID_ADDR, &Nv_Para.ucPID, 1);
		
	}
	else
		{
		}
	
	
}


u8 Delete_NV(void)
{

// eeprom


	if(Join_Flg == Type)
	{
		#if defined(Router)
		Erase_Word_Flash(NET_ONLINE_FLG_ADDR);	
		#endif
	}
	else if(Host_Node_Addr == Type)
	{
		
		#if defined(Router)
		Erase_Block_Flash(NET_HOST_NODE_ADDR, sizeof(Dev_Addr));
		#endif
	}
	else if(NwkAddr == Type)
	{
		
		#if defined(Router)
		Erase_Block_Flash(NWKADDR_ADDR, sizeof(u16));
		#endif
	}
	else if(RF_CH == Type)
	{	
		Erase_Word_Flash(RF_CH_OR_FREQ_ADDR);
	}
	else if(RF_Rate == Type)
	{	
		Erase_Word_Flash(RF_DATA_RATE_ADDR);
	}
	else if(Uart_BaudRate == Type)
	{	
		Erase_Word_Flash(UART_RATE_ADDR);
	}
	else if(Band_Width == Type)
	{	
		Erase_Word_Flash(BANDWIDTH_ADDR);
	}
	else if(All_Type == Type)
	{	
		
		#if defined(Router)
		Erase_Word_Flash(NET_ONLINE_FLG_ADDR);		
		Erase_Block_Flash(NET_HOST_NODE_ADDR, sizeof(Dev_Addr));
		#endif
		
		Erase_Word_Flash(RF_CH_OR_FREQ_ADDR);
		Erase_Word_Flash(RF_DATA_RATE_ADDR);	
		Erase_Word_Flash(UART_RATE_ADDR);	
		Erase_Word_Flash(BANDWIDTH_ADDR);
		
	}
	else
		{
		}



	return 0;
}

*/




