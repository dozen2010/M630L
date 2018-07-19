#include "LF_Protocol.h"
#include "ccRFID_CRC8.h"

#include "common.h"
//u32 MateTag_ID = 0;

u8 LF_4ByteCheck(u8 *data)
{
	u8 crc_data;

	crc_data=CRC8(data,3);
	
	if((data[0] == 0x96) && (data[3] == crc_data) && (((data[1] ^ 0XAA) & 0X1F) == (data[2] & 0X1F)))
		return CHECK_OK;

	return CHECK_ERR;
}

u8 LF_8ByteCheck(u8 *data)
{
	u8 crc_data;
	crc_data=CRC8(data,7);
    if(crc_data==CRC8(data,3))
    {
    	crc_data++;
    }

	if(data[7]==crc_data)
	{
		return CHECK_OK;
	}

	return CHECK_ERR;
}


u8 LF_ProtocolAnalyzing(u8 *in, LFProtocolType *parameter)
{
	u8 operation;
	u8 object;
	u32 cmd_key;

	if(in == 0 || parameter == 0)
		return LF_INVALID_ADDR;

	operation = (in[1] >> 4) & 0x0F;
	object = in[1] & 0x0F;

	parameter->operation = (OperationType)operation;
					
	switch(operation)
	{
		case OPE_SYS:
			parameter->OpeType.ope_sys = (OpeSysType)object;
			switch(object)
			{
				case SYS_VERSIONS:
					if((in[1] & in[2]) != 0)
						return LF_INVALID_CMD;
						
				case SYS_IN_SLEEP:
				case SYS_IN_REST:
					cmd_key = ((u32)in[3] << 24) | ((u32)in[4] << 16) | ((u32)in[5] << 8) | (u32)in[6];
					if(LF_CMD_KEY != cmd_key)
						return LF_ERR_CMD_KEY;				
					break;
                    
				default:
					return LF_INVALID_CMD;
					//break;
			}
			break;

		case OPE_SET:
			if(in[2] != LF_TYPEID)
				return LF_ERR_TYPEID;
            
			parameter->OpeType.ope_set = (OpeSetType)object;
			switch(object)
			{
				case SET_MATE_TAG_ID:
					parameter->Config.MateTageID = ((u32)in[3] << 24) | ((u32)in[4] << 16) | ((u32)in[5] << 8) | (u32)in[6];
					break;

				case SET_TAG_ID:
					parameter->Config.TageID = ((u32)in[3] << 24) | ((u32)in[4] << 16) | ((u32)in[5] << 8) | (u32)in[6];
					break;

				case SET_RF_CONFIG:
					parameter->Config.RFConfig.RF_FRQ_CHN = in[3];
					parameter->Config.RFConfig.RF_DataRate = in[4];
					parameter->Config.RFConfig.RF_PA_Select = in[5];
					break;

				case SET_HB_CYCLE:
					parameter->Config.HB_CycleBasic = ((u16)in[3] << 8) | (u16)in[4];
					break;

				case SET_OTHER:
					parameter->Config.OtherConfig.HB_Indicate_EN = in[3];
					parameter->Config.OtherConfig.AlertingNumber = in[4];
					parameter->Config.OtherConfig.MotionlessTime = in[5];
					parameter->Config.OtherConfig.AUTO_Sleep_EN = in[6];
					break;

				case SET_LONGPRESS:
					parameter->Config.LongPressConfig.LongPress_EN = in[3];
					parameter->Config.LongPressConfig.Move_Threshold = ((u16)in[4] << 8) | (u16)in[5];
					break;

				case SET_SAVE:

					break;
                    
				default:
					return LF_INVALID_CMD;
					//break;
			}
			break;

		case OPE_GET:
			if(in[2] != LF_TYPEID)
				return LF_ERR_TYPEID;
            
			parameter->OpeType.ope_get = (OpeGetType)object;
			switch(object)
			{
				case GET_MATE_TAG_ID:
				case GET_TAG_ID:
				case GET_RF_CONFIG:
				case GET_HB_CYCLE:
				case GET_OTHER:  
				case GET_LONGPRESS:
					break;
                    
				default:
					return LF_INVALID_CMD;
					//break;  
			}
			break;

		case OPE_TEST:
			if(in[2] != LF_TYPEID)
				return LF_ERR_TYPEID;
            
			parameter->OpeType.ope_test = (OpeTestType)object;
			switch(object)
			{
				case TEST_F:
					break;
                    
				default:
					return LF_INVALID_CMD;
					//break;  
			}
			break;

		case OPE_BIND:           //add by dozen 20180116
			{
//				if(in[2] != 0x37)     //matetag  typeid
//				{
//					return LF_ERR_TYPEID;
//				}

//				MateTag_ID = ((u32)in[3] << 24) 
//							| ((u32)in[4] << 16) 
//							| ((u32)in[5] << 8) 
//							| (u32)in[6];

//				SysStatus.ucBind_Status  = 1;

				break;
			}
            
		default:
			return LF_INVALID_CMD;
			//break;
	}
	
	return LF_PROTOCOL_OK;
}
