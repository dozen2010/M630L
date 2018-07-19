#define LF_LIB

#include "LF_AS3932.h"
#include "LF_Protocol.h"
#include "hw_gpio.h"
#include "hw_spi.h"

#define _NOP __NOP

static unsigned char LF_CLK_Data; 
static unsigned char LF_Byte_cnt;             //Byte counter of 125k
static unsigned char LF_Bit_cnt;               //Bit counter of 125k
//static 
unsigned char LF_RecByte[8];
LF_TypedDef LF;

//TIM_HandleTypeDef htim2;

volatile uint32_t TimerTickCounter = 1;

//volatile u8 WakeFlg = 0;


void Cali_RC(void);

/*
static void delay(unsigned int i)
{
  while(i--);
}
*/
static void Delaymm(void);

static void Delaymm(void)
{
	u8 i;
	
	for(i = 0; i < 20; i++);
		
}

void LF_SPI_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HW_GPIO_Init( AS_CS_PORT, AS_CS_PIN, &GPIO_InitStruct); 
	HW_GPIO_Init( AS_SCK_PORT, AS_SCK_PIN, &GPIO_InitStruct); 
	HW_GPIO_Init( AS_SDI_PORT, AS_SDI_PIN, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HW_GPIO_Init( AS_SDO_PORT, AS_SDO_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_CL_DAT_PORT, AS_CL_DAT_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_DAT_PORT, AS_DAT_PIN, &GPIO_InitStruct );

	AS_CS_0;	
//	AS_SCK_0;
}

void LF_SPI_GPIO_DeInit(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;//GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HW_GPIO_Init( AS_CS_PORT, AS_CS_PIN, &GPIO_InitStruct); 
	HW_GPIO_Init( AS_SCK_PORT, AS_SCK_PIN, &GPIO_InitStruct); 
	HW_GPIO_Init( AS_SDI_PORT, AS_SDI_PIN, &GPIO_InitStruct); 

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HW_GPIO_Init( AS_SDO_PORT, AS_SDO_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_CL_DAT_PORT, AS_CL_DAT_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_DAT_PORT, AS_DAT_PIN, &GPIO_InitStruct );

}

static void LF_AS3932_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HW_GPIO_Init( AS_CS_PORT, AS_CS_PIN, &GPIO_InitStruct); 
//	HW_GPIO_Init( AS_SCK_PORT, AS_SCK_PIN, &GPIO_InitStruct); 
//	HW_GPIO_Init( AS_SDI_PORT, AS_SDI_PIN, &GPIO_InitStruct); 
	
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HW_GPIO_Init( AS_SDO_PORT, AS_SDO_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_CL_DAT_PORT, AS_CL_DAT_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_DAT_PORT, AS_DAT_PIN, &GPIO_InitStruct );

	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;	
	HW_GPIO_Init( AS_WAKE_PORT, AS_WAKE_PIN, &GPIO_InitStruct );

//	HAL_NVIC_SetPriority( EXTI2_3_IRQn , 2, 0);       //add by dozen 180521
	
	AS_CS_0;	
//	AS_SCK_0;
}

static void LF_AS3932_GPIO_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;//GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HW_GPIO_Init( AS_CS_PORT, AS_CS_PIN, &GPIO_InitStruct); 
//	HW_GPIO_Init( AS_SCK_PORT, AS_SCK_PIN, &GPIO_InitStruct); 
//	HW_GPIO_Init( AS_SDI_PORT, AS_SDI_PIN, &GPIO_InitStruct); 

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HW_GPIO_Init( AS_SDO_PORT, AS_SDO_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_CL_DAT_PORT, AS_CL_DAT_PIN, &GPIO_InitStruct );
	HW_GPIO_Init( AS_DAT_PORT, AS_DAT_PIN, &GPIO_InitStruct );

	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;	
	HW_GPIO_Init( AS_WAKE_PORT, AS_WAKE_PIN, &GPIO_InitStruct );
}

/*

static void SPIWriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{	
    uint8_t i;

  	AS_CS_1;
	
    HW_SPI_InOut( addr );

    for( i = 0; i < size; i++ )
    {
        HW_SPI_InOut( buffer[i] );

    }

    AS_CS_0;
}

static void SPIReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
	uint8_t i;

	
  	AS_CS_1;	

	HW_SPI_InOut( addr );

	for( i = 0; i < size; i++ )
	{
		buffer[i] = HW_SPI_InOut( 0 );	  

	}

	AS_CS_0;

}

static void SPI_Write( uint8_t addr, uint8_t data )
{
    SPIWriteBuffer( addr, &data, 1 );
}

static uint8_t SPI_Read( uint8_t addr )
{
    uint8_t data;
    SPIReadBuffer( addr, &data, 1 );
    return data;
}
*/
//****************************************
//函数功能：SPI写
//input:  data(SPI的设置字节)
//output:
//****************************************

static void LF_SPI_Write(unsigned char data)
{
	unsigned char i;	
	
	for(i=8;i>0;i--)
	{
		
    	if(data&0X80)
    	{
    		AS_SDI_1;
    	}
    	else
    	{
    		AS_SDI_0;
    	}

//		Delaymm();
    	AS_SCK_1;	
		Delaymm();
//            _NOP();
//            _NOP();
//            _NOP();
//            _NOP();
//              
		
		AS_SCK_0;
		data <<= 1;	
   
	}
//	
//	AS_SCK_0;
//	AS_SDI_1;
}


//****************************************
//函数功能：SPI读
//input:  data(SPI的设置字节)
//output:
//****************************************

static unsigned char LF_SPI_Read(void)
{
	unsigned char i,data;
	
	for(i=8;i>0;i--)
	{
		data <<= 1;	
		AS_SCK_1;		
//		_NOP();
//		_NOP();
//		_NOP();
//		_NOP();
//     
		Delaymm();
		AS_SCK_0;
		
        if(AS_SDO_IN & AS_SDO)
        {
            data += 1;
        }

	}

	AS_SCK_0;
	
	return data;
}

//****************************************
//函数功能：写寄存器
//input:  RegAdd，data(SPI的设置字节)
//output:
//****************************************
void LF_Write_Register(unsigned char RegAdd,unsigned char Data)
{

    AS_CS_1;
	Delaymm();
    LF_SPI_Write(WriteMode | RegAdd);
    LF_SPI_Write(Data);
    AS_SDI_0;	
	Delaymm();
    AS_CS_0;
	
//	SPI_Write( WriteMode | RegAdd, Data );

}
//****************************************
//函数功能：写命令
//input:  Command(命令设置字节)
//output:
//****************************************
void LF_Write_Command(unsigned char Command)
{
    AS_CS_1;
	Delaymm();
    LF_SPI_Write(DirectCommandMode|Command);
    AS_SDI_1;
	Delaymm();
    AS_CS_0;

//	SPI_Write( DirectCommandMode|Command, 0 );

}
//****************************************
//函数功能：读寄存器
//input:  RegAdd(寄存器地址)
//output: Val(寄存器中的值)
//****************************************
unsigned char LF_Read_Register(unsigned char RegAdd)
{
	unsigned char Val;
	
    AS_CS_1;
	
	Delaymm();
    LF_SPI_Write(ReadMode|RegAdd);
    Val = LF_SPI_Read();
  
  	Delaymm();
    AS_CS_0;
////    _NOP();
  
	return Val;
//	return(SPI_Read( ReadMode|RegAdd ));

}

//****************************************
//函数功能：读寄存器
//input:  RegAdd(寄存器地址)
//output: Val(寄存器中的值)
//****************************************
// You can check the RCO frequency before and after calibration on the WAKE pin.

// If you write in R8=0x04 and in R9=0x14 the RCO frequency (divided by 2) will be displayed on the WAKE pin. 

// This is an internal test mode that is not documented in the datasheet. Let’s use this one for debugging.

void LF_AS3932Init(void)
{
//    unsigned char i;
	
	LF_AS3932_GPIO_Init();
   LF_INTERRUPT_DIS; 
//-------端口初始化------------------------  
    AS_CS_INIT;
    AS_SDO_INIT;
    AS_SDI_INIT;
    AS_SCK_INIT;

    AS_CL_DAT_INIT;
    AS_DAT_INIT;
    AS_WAKE_INIT;
    
//-------AS3932 初始化------------------------ 
    LF_Write_Command(PresetDefault);
//	as3933_write_dcommand(PresetDefault);
    
	/*//Register  AS3932
    LF_Write_Register(R0,0X2A);//0B0010 1010	//2A
    LF_Write_Register(R1,0X2A);//0B0010 1010   //2A
    LF_Write_Register(R2,0);//0B00000000 bit5~~~bit6 
    LF_Write_Register(R3,0X90);//0B1001 0000
    LF_Write_Register(R4,0xF2);//0B0000 0010 on/off mode
    LF_Write_Register(R7,0X0B);//0B0000 1011 360us/bit
    LF_Write_Register(R8,0X00);//0B00000000*/
	
    //Register  AS3933
    LF_Write_Register(R0,0X2A);//0B0010 1010	//EA
    LF_Write_Register(R1,0X2A);//0B0010 1010   //2A
    LF_Write_Register(R2,0);//0B00000000 bit5~~~bit6 
    LF_Write_Register(R3,0X90);//0B1001 0000
    LF_Write_Register(R4,0x02);//0B0000 0010 on/off mode
    LF_Write_Register(R7,0X4B);//0B0000 1011 360us/bit

//	as3933_write_byte(R0, 0X2A);

//	as3933_write_byte(R1,0X2A);

//	as3933_write_byte(R2,0);

//	as3933_write_byte(R3,0X90);

//	as3933_write_byte(R4,0x02);

//	as3933_write_byte(R7,0X4B);
    
  // LF_Write_Register(R8,0X04);//0B00000000
 //  LF_Write_Register(R9,0x14);//0B00000000
 
    //LF_Write_Register(R16,0XC1);//0B10000000
    //LF_Write_Register(R17,0X03);
    LF_CLK_Data = LF_Read_Register(R0);
    //LFReceived = LF_Read_Register(R14);
    //LFReceived = LF_Read_Register(R15);
    LF_Write_Command(ClearFalse);
    LF_Write_Command(ResetAGC);
    LF_Write_Command(ClearWake);

//	Cali_RC();
//	as3933_write_dcommand(ClearFalse);

//	as3933_write_dcommand(ResetAGC);

//	as3933_write_dcommand(ClearWake);
	
  // LF_Write_Command(CenterOSC);

/*  
    AS_CS_1;
    LF_SPI_Write(DirectCommandMode|TrimOSC);
    AS_SDI_0;  
    _NOP();_NOP();_NOP();_NOP();_NOP(); 
    _NOP();_NOP();_NOP();_NOP();_NOP(); 
    
	for(i=0;i<65;i++)//29us ==>28.8uS   // dig350uS/12 =29.12
	{
	     AS_SCK_1;
	    _NOP();_NOP();_NOP();_NOP();
	    _NOP();_NOP();_NOP();_NOP();
	    AS_SCK_0;
	    _NOP();_NOP();_NOP();_NOP();
	    _NOP();_NOP();     
	  
	}  

	*/
  
  /*  for(i=0;i<65;i++)//29us ==>28.8uS   // dig350uS/12 =29.12
    {
         AS_SCK_1;
       delay(15);
      _NOP();_NOP();_NOP();_NOP();
        _NOP();_NOP();_NOP();_NOP();
          _NOP();_NOP();_NOP();_NOP();
          _NOP(); _NOP();_NOP(); _NOP();_NOP();
          AS_SCK_0;
        _NOP();_NOP();_NOP();_NOP();
          _NOP();_NOP();_NOP();_NOP();
            _NOP();_NOP();_NOP();_NOP();
            _NOP();_NOP();_NOP();_NOP(); _NOP();_NOP();
        delay(12);
      
    } */
    AS_CS_0;
     
    LF_CLK_Data = 0; 

    LF_Byte_cnt = 0;             //Byte counter of 125k
    LF_Bit_cnt = 0;               //Bit counter of 125k
    LF.Protocal = 0;     //flag of receive
    LF.DecodeErrCnt = 0;

//	LF_INTERRUPT_EN;  
}

void LF_AS3932DeInit(void)
{
//	LF_INTERRUPT_DIS; 
//-------端口初始化------------------------  
    AS_CS_INIT;
    AS_SDO_INIT;
    AS_SDI_INIT;
    AS_SCK_INIT;

    AS_CL_DAT_INIT;
    AS_DAT_INIT;
    AS_WAKE_INIT;
	
	LF_Write_Register(R0,0X01);
	
	LF_AS3932_GPIO_DeInit();
}

/*
void Cali_RC(void)
{
	volatile u8 Temp = 0;

	///SPI 31.25KHZ
	AS_CS_1;
	Delaymm();
//	Temp = LF_Read_Register(R14);

    LF_Write_Command(DirectCommandMode|TrimOSC);
    AS_SDI_0;  
	AS_SCK_0;
//	Delaymm();
//	Temp = LF_Read_Register(R14);

	TimerHwInit();
	
	while((LF_Read_Register(R14)&0xC0) != 0x80)//(Temp != 0x80)//((LF_Read_Register(R14)&0xB0) == 0x80);
	{	
//		
//		Delaymm();		
//		Delaymm();
//		
//		Temp = LF_Read_Register(R14);		
//		Temp &= 0xC0;

		if((LF_Read_Register(R14)&0xC0) == 0x40)
		{			
			HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);
			
			TimerHwInit();
		}
	}
	
 	AS_CS_0;
	
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_DeInit(&htim2);
	
	LF_AS3932_GPIO_Init();
	
}

*/

void  LF_AwakeUp(void)
{
    unsigned char CRC_DAT;
    LF_INTERRUPT_DIS;
	
    for(CRC_DAT=0;CRC_DAT<8;CRC_DAT++)
      LF_RecByte[CRC_DAT]=0x00;
              
    LF_Byte_cnt = 0;
    LF_Bit_cnt = 0;
    LF_CLK_Data = 0;
	LF.Protocal = 0;
    //====================================
    while(1)
    {  
        if((AS_WAKE_IN & AS_WAKE)==0) //End of receive date
        {
            LF.Protocal = 0x50;
            LF.DecodeErrCnt++;
            break;

        }  
        if(AS_CL_DAT_IN & AS_CL_DAT)  //CL_DAT  High level,receive date
        {	 
            if(LF_CLK_Data)
            {
                LF_CLK_Data = 0;
                
                LF_RecByte[LF_Byte_cnt] <<= 1;  
                if(AS_DAT_IN & AS_DAT)//Dat High level
                {
                    LF_RecByte[LF_Byte_cnt]++;
                }
                
                LF_Bit_cnt ++;
                if(LF_Bit_cnt > 7)	//One 16bit received	
                {
                    LF_Bit_cnt = 0;
                    LF_Byte_cnt++;
                    // 4 字节处理
                    if((LF.Protocal == 0) && (LF_Byte_cnt > 3))//Data received
                    {
                        if((LF_RecByte[0]==0X96)&&(LF_RecByte[3]==CRC8(LF_RecByte,3))&&\
        			    (((LF_RecByte[1] ^ 0XAA)&0X1F)==(LF_RecByte[2]&0X1F)))
                        {	
							LF.RSSIx = LF_Read_Register(R10);
							LF.RSSIy = LF_Read_Register(R11);
							LF.Protocal = LF_SINGLE;
							LF.Data.SingleAddr = (((uint16_t)(LF_RecByte[2]&0XE0))<<3) + LF_RecByte[1];
                            break;
                        }
						else if (LF_RecByte[0] == 0x5A)		//带选项的定位协议
						{
							LF.Protocal = 0x44;
						}
                        else if(LF_RecByte[0] == 0x69)		//LF配置
                        {
                            LF.Protocal = 0x33;
                        }
                        else 			//Data error
                        {
							LF.DecodeErrCnt++;
                            break;
                        }
                    }
					if(LF_Byte_cnt > 4 && LF.Protocal == 0x44)
					{
						if((LF_RecByte[3] & 0xF8) == ((((LF_RecByte[1]+LF_RecByte[2])^0xAA)&0x1F)<<3) && \
							LF_RecByte[4] == CRC8(LF_RecByte,4))
						{
							LF.RSSIx = LF_Read_Register(R10);
							LF.RSSIy = LF_Read_Register(R11);
							LF.Protocal = LF_MULTIPLE;
							LF.Data.MultipleAddr.Type = LF_RecByte[3] & 0x07;
							LF.Data.MultipleAddr.Addr = LF_RecByte[1]+(((uint16_t)LF_RecByte[2])<<8);
							break;
						}
					}
                    else if((LF_Byte_cnt > 7)&&(LF.Protocal ==0x33))//Data received
                    {
						LF.Protocal = LF_RecByte[3];
                        LF_RecByte[3] = LF_RecByte[4]; 
                        LF_RecByte[4] = LF_RecByte[5];
                        LF_RecByte[5] = LF_RecByte[6]; 
                        LF_RecByte[6] = LF_RecByte[7];
                        LF_RecByte[7] = LF.Protocal;
						CRC_DAT = CRC8(LF_RecByte,7);
						if(CRC_DAT==CRC8(LF_RecByte,3))
        				{
        				    CRC_DAT++;
        				}
			            if(LF_RecByte[7]==CRC_DAT)
			            {
							LF.Protocal = LF_CONFIG;
							LF.Data.ConfigData = &LF_RecByte[1];
                        }
                        else
                        {
                            LF.DecodeErrCnt++;
                            LF.Protocal = 0x5A;
                        }
                        break;
                    }
                }
            }		
        }
        else
        {
            LF_CLK_Data = 1;
        }	        
    }// end of while
//================================================
    LF_Write_Command(ClearWake);
    LF_Write_Command(ClearFalse);
    LF_Write_Command(ResetAGC);

    LF_INTERRUPT_EN;
 
}

/*
////////  sleep rec error ///
unsigned char LF_Rec_Data( unsigned int ulSysTime )
{
	unsigned char CRC_DAT;
	static unsigned char Endflg = 0;
	
	if(LFReceived == LF_IDLE)
	{		
		return 1;
	} 
	else if(LFReceived == LF_START_REC)      //inti awakeup set
	{

		for(CRC_DAT = 0; CRC_DAT < 8; CRC_DAT++)
		{		
			LF_RecByte[CRC_DAT]=0x00;
		}
		              
	    LF_Byte_cnt = 0;
	    LF_Bit_cnt = 0;

	    LF_CLK_Data = 0;//1;

		LFReceived = LF_REC;

		PRINTF("LF rec:%d\r\n", LFReceived);
	
	}

	if(LFReceived == LF_REC)
	{
	
		if(AS_CL_DAT_IN & AS_CL_DAT)  //CL_DAT  High level,receive date
    	{
    		
			if(LF_CLK_Data)
			{
				LF_CLK_Data = 0;
				
	    		LF_RecByte[LF_Byte_cnt] <<= 1;  
				
	            if(AS_DAT_IN & AS_DAT)//Dat High level
	            {
	                LF_RecByte[LF_Byte_cnt]++;
	            }
	            
	            LF_Bit_cnt ++;
			
           		if(LF_Bit_cnt > 7)	//One 16bit received	
				{
                    LF_Bit_cnt = 0;
                    LF_Byte_cnt++;
                    // 4 字节处理
                    if(LF_Byte_cnt > 3)//Data received
	                {       
                        if((LF_RecByte[0] == 0x96) && (LF_4ByteCheck(LF_RecByte) == CHECK_OK))
        			    {	
                            LF_RecByte[4] = LF_Read_Register(R10);   //ch1
                            LF_RecByte[5] = LF_Read_Register(R11);   //ch2
                            LFReceived = LF_SINGLE;
                            Endflg = 1;
                        }
                        else if(LF_RecByte[0] == 0x69)
                        {
                            LFReceived = LF_CONF_STX;
                        }
                        else 			//Data error
                        {
                            LF_DecodeErr_cnt++;  
                            LFReceived = LF_ERROR;
                            LF_Byte_cnt = 0;
							Endflg = 1;

							for(CRC_DAT = 0; CRC_DAT < 8; CRC_DAT++)
							{
								PRINTF("%02x ", LF_RecByte[CRC_DAT]);
							}
							PRINTF("\r\n");
                        }
                    }
                    if((LF_Byte_cnt > 7)&&(LFReceived == LF_CONF_STX))//Data received
                    {  
                       
                        LF_Byte_cnt = 0;
                        LFReceived = LF_RecByte[3];
                        LF_RecByte[3] = LF_RecByte[4]; 
                        LF_RecByte[4] = LF_RecByte[5];
                        LF_RecByte[5] = LF_RecByte[6]; 
                        LF_RecByte[6] = LF_RecByte[7];
                        LF_RecByte[7] = LFReceived;

			            if(LF_8ByteCheck(LF_RecByte) == CHECK_OK)
			            {
			                 LFReceived = LF_CONFIG;                                                
                        }
                        else
                        {
                            LF_DecodeErr_cnt++;  
                            LFReceived = LF_ERROR;
                            LF_Byte_cnt = 0; 
                        }   
						
						Endflg = 1;
                    }
                }
			}
    	}
		else
        {
            LF_CLK_Data = 1;
        }	    
        
	}	

	if(Endflg)
	{
		
		PRINTF("LF rec:%d\r\n", LFReceived);

		LF_Write_Command(ClearWake);
		LF_Write_Command(ClearFalse);
		LF_Write_Command(ResetAGC);
		Endflg = 0;

		LF_INTERRUPT_EN;
	}
	
	return 0;
}

*/

/* TIM2 init function */

/*
void TimerHwInit(void)
{
//    TimerDelayCounter = 0;
//    TimeoutCntValue = 0;

//    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
	
	TIM_OC_InitTypeDef sConfigOC;

  // 4M / 128 = 31.25kHZ  16M /512  = 31.25KHZ
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 511;//511;//63;//127;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

//    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 255;//255;   //64/128 = 50% pwm
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

  	HAL_TIM_MspPostInit(&htim2);

	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
    //Start Hw Timer2 Now
//    HAL_TIM_Base_Start_IT(&htim2);
}

*/

//void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* tim_ocHandle)
//{

//  if(tim_ocHandle->Instance==TIM2)
//  {
//  /* USER CODE BEGIN TIM2_MspInit 0 */

//  /* USER CODE END TIM2_MspInit 0 */
//    /* TIM2 clock enable */
//    __HAL_RCC_TIM2_CLK_ENABLE();
//  /* USER CODE BEGIN TIM2_MspInit 1 */

//  /* USER CODE END TIM2_MspInit 1 */
//  }
//}

//void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* tim_ocHandle)
//{

//  if(tim_ocHandle->Instance==TIM2)
//  {
//  /* USER CODE BEGIN TIM2_MspDeInit 0 */

//  /* USER CODE END TIM2_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM2_CLK_DISABLE();
//  /* USER CODE BEGIN TIM2_MspDeInit 1 */

//  /* USER CODE END TIM2_MspDeInit 1 */
//  }
//} 


//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{

//  GPIO_InitTypeDef GPIO_InitStruct;
//  if(timHandle->Instance==TIM2)
//  {
//  /* USER CODE BEGIN TIM2_MspPostInit 0 */

//  /* USER CODE END TIM2_MspPostInit 0 */
//  
//    /**TIM2 GPIO Configuration    
//    PA5     ------> TIM2_CH1 
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  /* USER CODE BEGIN TIM2_MspPostInit 1 */

//  /* USER CODE END TIM2_MspPostInit 1 */
//  }

//}

//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
//{
//    if(htim_base->Instance==TIM2)
//    {
//        /* Peripheral clock enable */
//        __HAL_RCC_TIM2_CLK_ENABLE();
//        
//        /* Peripheral interrupt init */
//        HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
//        HAL_NVIC_EnableIRQ(TIM2_IRQn);
//    }
//}

//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
//{
//    if(htim_base->Instance==TIM2)
//    {
//        /* Peripheral clock disable */
//        __HAL_RCC_TIM2_CLK_DISABLE();
//        
//        /* Peripheral interrupt Deinit*/
//        HAL_NVIC_DisableIRQ(TIM2_IRQn);
//    }
//} 

