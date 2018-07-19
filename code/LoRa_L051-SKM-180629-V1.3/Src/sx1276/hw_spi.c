/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board SPI driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /*******************************************************************************
  * @file    hw_spi.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    27-February-2017
  * @brief   manages the SPI interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
//#include "utilities.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef hspi;
/* Private function prototypes -----------------------------------------------*/

/*!
 * @brief Calculates Spi Divisor based on Spi Frequency and Mcu Frequency
 *
 * @param [IN] Spi Frequency
 * @retval Spi divisor
 */
static uint32_t SpiFrequency( uint32_t hz );
static void Delaymm(void);

/* Exported functions ---------------------------------------------------------*/

/*!
 * @brief Initializes the SPI object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI_Init( void )
{

  GPIO_InitTypeDef initStruct={0};
  /*##-1- Configure the SPI peripheral */
  /* Set the SPI parameters */

  hspi.Instance = SPI2;

  hspi.Init.BaudRatePrescaler = SpiFrequency( 1000000 );//SPI_BAUDRATEPRESCALER_4;//SpiFrequency( 10000000 );
  hspi.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi.Init.Mode           = SPI_MODE_MASTER;
  hspi.Init.CLKPolarity    = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;  
  hspi.Init.FirstBit       = SPI_FIRSTBIT_MSB;
  hspi.Init.NSS            = SPI_NSS_SOFT;
  hspi.Init.TIMode         = SPI_TIMODE_DISABLE;


  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  if(HAL_SPI_Init( &hspi) != HAL_OK)
  {
    /* Initialization Error */
     Error_Handler();
  }

//  /*##-2- Configure the SPI GPIOs */
//  initStruct.Mode =GPIO_MODE_AF_PP;
//  initStruct.Pull = GPIO_PULLDOWN;
//  initStruct.Speed = GPIO_SPEED_HIGH;
//  initStruct.Alternate= GPIO_AF0_SPI1;//SPI1_AF ;

//  HW_GPIO_Init( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct); 
//  HW_GPIO_Init( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct); 
//  HW_GPIO_Init( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct); 


//	
  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLUP;

  HW_GPIO_Init(  RADIO_NSS_PORT, RADIO_NSS_PIN, &initStruct );

  HW_GPIO_Write ( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );

  HW_GPIO_Init(  RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN, &initStruct );   //RF vdd

  HW_GPIO_Write ( RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN, 1 );

}

/*!
 * @brief De-initializes the SPI object and MCU peripheral
 *
 * @param [IN] none
 */
void HW_SPI_DeInit( void )
{
  GPIO_InitTypeDef initStruct={0};

//  HAL_SPI_DeInit( &hspi);
  
  __HAL_RCC_GPIOB_CLK_ENABLE();

    /*##-1- Reset peripherals ####*/
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();
    
  initStruct.Mode =GPIO_MODE_OUTPUT_PP;

  initStruct.Pull =GPIO_NOPULL  ; 
  HW_GPIO_Init ( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct ); 
  HW_GPIO_Write( RADIO_MOSI_PORT, RADIO_MOSI_PIN, 0 );
  
  initStruct.Pull =GPIO_PULLDOWN; 
  HW_GPIO_Init ( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct ); 
  HW_GPIO_Write( RADIO_MISO_PORT, RADIO_MISO_PIN, 0 );
  
  initStruct.Pull =GPIO_NOPULL  ; 
  HW_GPIO_Init ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct ); 
  HW_GPIO_Write(  RADIO_SCLK_PORT, RADIO_SCLK_PIN, 0 );

  initStruct.Pull = GPIO_PULLUP; 
  HW_GPIO_Init ( RADIO_NSS_PORT, RADIO_NSS_PIN , &initStruct ); 
  HW_GPIO_Write( RADIO_NSS_PORT, RADIO_NSS_PIN , 1 );

	
  initStruct.Pull =GPIO_NOPULL; 
  HW_GPIO_Init ( RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN , &initStruct ); 
  HW_GPIO_Write ( RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN, 0 );   //add 20180112  
//  HW_GPIO_Write ( RADIO_ANT_SWITCH_PORT_RX, RADIO_ANT_SWITCH_PIN_RX, 0 );   //add 20180112

}

/*!
 * @brief Sends outData and receives inData
 *
 * @param [IN] outData Byte to be sent
 * @retval inData      Received byte.
 */
uint16_t HW_SPI_InOut( uint16_t txData )
{
  uint16_t rxData ;

  HAL_SPI_TransmitReceive( &hspi, ( uint8_t * ) &txData, ( uint8_t* ) &rxData, 1, HAL_MAX_DELAY);	

  return rxData;
}

/* Private functions ---------------------------------------------------------*/

static uint32_t SpiFrequency( uint32_t hz )
{
  uint32_t divisor = 0;
  uint32_t SysClkTmp = SystemCoreClock;
  uint32_t baudRate;
  
  while( SysClkTmp > hz)
  {
    divisor++;
    SysClkTmp= ( SysClkTmp >> 1);
    
    if (divisor >= 7)
      break;
  }
  
  baudRate =((( divisor & 0x4 ) == 0 )? 0x0 : SPI_CR1_BR_2  )| 
            ((( divisor & 0x2 ) == 0 )? 0x0 : SPI_CR1_BR_1  )| 
            ((( divisor & 0x1 ) == 0 )? 0x0 : SPI_CR1_BR_0  );
  
  return baudRate;
}


/////////////////////////  GPIO  TO SPI  DRIVER //////////////////////

void LoRa_SPISetup(void)
{
	GPIO_InitTypeDef initStruct={0};

	__HAL_RCC_GPIOA_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*##-2- Configure the SPI GPIOs */
//	initStruct.Pin  = RADIO_VDD_PIN | RADIO_MOSI_PIN | RADIO_MISO_PIN
	initStruct.Mode = GPIO_MODE_OUTPUT_PP;
	initStruct.Pull = GPIO_PULLUP;
	initStruct.Speed = GPIO_SPEED_HIGH;

	HW_GPIO_Init( RADIO_MOSI_PORT, RADIO_MOSI_PIN, &initStruct); 	
	HW_GPIO_Init( RADIO_SCLK_PORT, RADIO_SCLK_PIN, &initStruct); 	
	HW_GPIO_Init( RADIO_NSS_PORT, RADIO_NSS_PIN, &initStruct); 	

	
	initStruct.Pull = GPIO_NOPULL;
	HW_GPIO_Init(  RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN, &initStruct );   //RF vdd

	initStruct.Mode = GPIO_MODE_INPUT;
	initStruct.Pull = GPIO_PULLUP;
	initStruct.Speed = GPIO_SPEED_HIGH;
	
	HW_GPIO_Init( RADIO_MISO_PORT, RADIO_MISO_PIN, &initStruct); 

	HW_GPIO_Write ( RADIO_NSS_PORT, RADIO_NSS_PIN, 1 );	
	HW_GPIO_Write ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, 0 );	
	HW_GPIO_Write ( RADIO_RF_VDD_PORT, RADIO_RF_VDD_PIN, 1 );
//	HW_GPIO_Write ( RADIO_SCLK_PORT, RADIO_SCLK_PIN, 0 );
//	HW_GPIO_Write(SPI1_SCLK_GPIO, SPI1_SCLK_Pin, 1);
//	HW_GPIO_Write(SPI1_SCLK_GPIO, SPI1_SCLK_Pin, 0);
//	SPI1_SCLK_1;
//	SPI1_SCLK_0;
//	SPI1_MOSI_1;
//	SPI1_MOSI_0;
	
}


/*******************************************************************
** SpiInOut : Sends and receives a byte from the SPI bus          **
********************************************************************
** In  : outputByte                                               **
** Out : inputByte                                                **
*******************************************************************/
uint8_t SpiInOut (uint8_t outputByte)
{
    uint8_t bitCounter;
    uint8_t inputByte = 0;

    SPI1_SCLK_0;
    for(bitCounter = 0x80; bitCounter != 0x00; bitCounter >>= 1)   //msb
	{
        if (outputByte & bitCounter)
		{
            SPI1_MOSI_1;
        }
        else
		{
            SPI1_MOSI_0;
        }
		
//		Delaymm();
		
        SPI1_SCLK_1;
//		
//		Delaymm();        //add by dozen 
//		SPI1_SCLK_0;     //add by dozen 
		
//		Delaymm();        //add by dozen 
		
        if (SPI1_MISO_IN!=0x00)
		{
            inputByte |= bitCounter;
        }
				
//		Delaymm();
        SPI1_SCLK_0;
    }  // for(BitCounter = 0x80; BitCounter != 0x00; BitCounter >>= 1)
    SPI1_MOSI_0;

    return inputByte;
} // INT8U SpiInOut (INT8U outputByte)

static void Delaymm(void)
{
	u8 i;
	
	for(i = 0; i < 200; i++);
		
}

void SPICmd8bit(unsigned char outtt)
{
	unsigned char i;
	for (i=0;i<8;i++)
	{   
		SPI1_SCLK_0;	
		if (outtt & 0x80)			/* check if MSB is high */
			SPI1_MOSI_1;
		else 
			SPI1_MOSI_0;						/* if not, set to low */

		Delaymm();
		SPI1_SCLK_1;						  /* toggle clock high */
		Delaymm();
		outtt = (outtt << 1);	/* shift 1 place for next bit */

	}
	SPI1_SCLK_0;
	SPI1_MOSI_1;
}

unsigned char SPIRead8bit(void)
{	 
	unsigned char j;
	unsigned char i;
	j=0;

	SPI1_MOSI_1;
	for (i = 0; i < 8; i++)
	{	 
		SPI1_SCLK_0; 
		Delaymm();
		j = (j << 1);			// shift 1 place to the left or shift in 0 //
		SPI1_SCLK_1;
		
		if(SPI1_MISO_IN!=0x00)	// check to see if bit is high //
			j = j | 0x01; 	// if high, make bit high //
										
		Delaymm();
	}
	
	SPI1_SCLK_0;			  // toggle clock low //  
	
	return j;						// toggle clock low //
}

void RF_SPI_MasterIO(unsigned char outtt)
{
	unsigned char i;
	for (i=0;i<8;i++)
	{   
		if (outtt & 0x80)			/* check if MSB is high */
			SPI1_MOSI_1;
		else 
			SPI1_MOSI_0;						/* if not, set to low */

		SPI1_SCLK_1;						  /* toggle clock high */
		Delaymm();
		outtt = (outtt << 1);	/* shift 1 place for next bit */
		SPI1_SCLK_0;							/* toggle clock low */
		Delaymm();
	}
}


unsigned char RF_SPI_READ_BYTE()
{	 
	unsigned char j;
	unsigned char i;
	j=0;
	for (i = 0; i < 8; i++)
	{	 
		SPI1_SCLK_1; 
		Delaymm();
		j = (j << 1);			// shift 1 place to the left or shift in 0 //
		if(SPI1_MISO_IN!=0x00)	// check to see if bit is high //
			j = j | 0x01; 	// if high, make bit high //
										  // toggle clock high // 
		SPI1_SCLK_0; 			  // toggle clock low //  
		Delaymm();
	}
	return j;						// toggle clock low //
}

void SPI_write(unsigned char dat)
{
    unsigned char temp;
	
     for(temp=0x80;temp!=0;temp>>=1)
     {
         SPI1_SCLK_0;
		 
         if((temp&dat)==0)
         {
            SPI1_MOSI_0;
         }
         else
         {
			SPI1_MOSI_1;
         }
         Delaymm(); //»√MOSIŒ»∂®
         SPI1_SCLK_1;
     }
}

unsigned char SPI_read(void)
{
	unsigned char temp;
	unsigned char dat;
	
	for(temp=0x80;temp!=0;temp>>=1)
	{
	     SPI1_SCLK_1;
	     Delaymm();  //»√SCKŒ»∂®
	     SPI1_SCLK_0;
		 
	     if(SPI1_MISO_IN ==1)
	     {
	          dat|=temp;
	     }
	     else
	     {
	          dat&=~temp;
	     }
		 
	}
	
	return dat; 
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

