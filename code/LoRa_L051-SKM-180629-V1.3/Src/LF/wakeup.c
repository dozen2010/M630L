#include "stm32l0xx_hal.h"
#include "wakeup.h"
#include "ccRFID_CRC8.h"


//const unsigned char CRC8Table[] =   //CRCÂë±í¸ß×Ö½Ú
//{
//  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
//  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
//  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
//  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
//  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
//  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
//  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
//  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
//  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
//  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
//  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
//  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
//  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
//  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
//  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
//  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
//};

//unsigned char CRC8(unsigned char const * pData, unsigned char len)
//{
//    unsigned char CRC_BUF = 0;
//    for( ; len > 0; len--)
//    {
//        CRC_BUF = CRC8Table[CRC_BUF ^ *pData];
//        pData++;
//    }
//    return(CRC_BUF);
//}

void Time_1Bit(void)
{
  uint16_t  i,j;
	for(j=3;j>0;j--)
   { 
		 for(i=90;i>0;i--)
    {
    ;
    }
	}   
}

//void delay_ms1(int32_t ms)  
//{  
//		Delay_us1(1000*ms);
//}  
//void Delay_us1(uint32_t us)
//	{
//    uint16_t counter=us&0xffff;

//    HAL_TIM_Base_Start(&htim6);
//    __HAL_TIM_SetCounter(&htim6,counter);

//    while(counter>1)
//    {
//        counter=__HAL_TIM_GetCounter(&htim6);
//    }

//    HAL_TIM_Base_Stop(&htim6);
//}

void Time_1BitA(void)
{
    uint16_t  i,j;
    //for(i=79;i>0;i--)
		for(j=3;j>0;j--)
   { 
    for(i=87;i>0;i--)
    {
    ;
    }
	} 
}
void SendLF(uint8_t  *Pdata)
{
 	uint16_t  i,j;
  uint8_t  SendBuffer[10];
    
    
	SendBuffer[0] = Pdata[0];
	SendBuffer[1] = Pdata[1];
	SendBuffer[2] = Pdata[2];	
	SendBuffer[4] = Pdata[3];
	SendBuffer[5] = Pdata[4];
  SendBuffer[6] = Pdata[5];
	SendBuffer[7] = Pdata[6];
	
	SendBuffer[3] = CRC8( Pdata, 7);
	if(SendBuffer[3] ==CRC8( Pdata, 3))
	{
		SendBuffer[3]++;
	}
 	
//	NVIC_SETFAULTMASK(); //????? 
  PWM_INIT;
  PWM_ON; //Carrier Burst
	for(i=0;i<15;i++)//15
		Time_1Bit();
//-----sepeation bit------//
	PWM_OFF;
	Time_1Bit();
//-----Preamble-----------//10101010
	PWM_ON;
	Time_1Bit();

	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
//--------R6--------------//10010110
//------------------------1
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
//------------------------0
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
//------------------------0
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
//------------------------1
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
//------------------------0
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();
//------------------------1
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
//------------------------1
	PWM_ON;
	Time_1Bit();
	PWM_OFF;
	Time_1Bit();
//------------------------0
	PWM_OFF;
	Time_1Bit();
	PWM_ON;
	Time_1Bit();

//--------data-------------
// Send 4 Bytes
// ------------------------------
// | 0X69 | DATA1 | DATA0 | CRC | 
// ------------------------------
// CRC = DATA1 XOR DATA0

 for(i=0;i<8;i++)
 {
    for(j=8;j>0;j--)
    {
        if ((SendBuffer[i] & 0X80)!=0) // Bit1
     	 {
    		PWM_ON;		
    		Time_1Bit();
    		PWM_OFF;
    		Time_1BitA();
    	 }
    	 else //Bit0
    	 {
    		PWM_OFF;
    		Time_1Bit();
    		PWM_ON;
    		Time_1BitA();	 
    	 }
    	 SendBuffer[i] <<= 1;
    }
 }

PWM_OFF;
}



