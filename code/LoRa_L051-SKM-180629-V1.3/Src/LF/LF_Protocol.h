#ifndef __LF_PROTOCOL_H
#define __LF_PROTOCOL_H

//#include "stdint.h"
#include "Aware_Config.h"

#define LF_CMD_KEY 0xAA559966
#define LF_TYPEID  0x16        //Ó¤¶ù±êÇ©0x36

#define CHECK_OK  1
#define CHECK_ERR 0

#define LF_PROTOCOL_OK  0
#define LF_INVALID_ADDR 1
#define LF_INVALID_CMD  2
#define LF_ERR_CMD_KEY  3
#define LF_ERR_TYPEID   4

typedef enum{
	OPE_SYS = 0x1,
	OPE_SET = 0x2,
	OPE_GET = 0x3,
	OPE_TEST = 0xA,
	OPE_BIND = 0x9,    //add by dozen 20180116
}OperationType;

typedef enum{
	SYS_VERSIONS = 0x0,
	SYS_IN_SLEEP = 0xE,
	SYS_IN_REST = 0xF,
}OpeSysType;

typedef enum{
	SET_MATE_TAG_ID = 0x0,
	SET_TAG_ID = 0x1,
	SET_RF_CONFIG = 0x2,
	SET_HB_CYCLE = 0x3,
	SET_OTHER = 0x4,
	SET_LONGPRESS = 0x5,
	SET_SAVE = 0xF,
}OpeSetType;

typedef enum{
	GET_MATE_TAG_ID = 0x0,
	GET_TAG_ID = 0x1,
	GET_RF_CONFIG = 0x2,
	GET_HB_CYCLE = 0x3,
	GET_OTHER = 0x4,
	GET_LONGPRESS = 0x5,
}OpeGetType;

typedef enum
{
	TEST_F = 0xF,
}OpeTestType;

typedef struct
{
	OperationType operation;
	union {
		OpeSysType ope_sys;
		OpeSetType ope_set;
		OpeGetType ope_get;
		OpeTestType ope_test;
	}OpeType;
	union {
		u32 MateTageID;
		u32 TageID;
		struct {
			u8 RF_FRQ_CHN;
			u8 RF_DataRate;
			u8 RF_PA_Select;
		}RFConfig;
		u16 HB_CycleBasic;
		struct {
			u8 HB_Indicate_EN;
			u8 AlertingNumber;
			u8 MotionlessTime;
			u8 AUTO_Sleep_EN;
		}OtherConfig;
		struct {
			u8 LongPress_EN;
			u16 Move_Threshold;			
		}LongPressConfig;
	}Config;
	
}LFProtocolType;

u8 LF_4ByteCheck(u8 *data);
u8 LF_8ByteCheck(u8 *data);
u8 LF_ProtocolAnalyzing(u8 *in, LFProtocolType *parameter);

//extern u32 MateTag_ID;

#endif /* __LF_PROTOCOL_H */

