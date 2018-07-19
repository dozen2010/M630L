 /*******************************************************************************
  * @file    low_power.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    27-February-2017
  * @brief   driver for low power
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
#include "Low_power.h"
#include "timeServer.h"
#include "common.h"
//#include "hw_msp.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * \brief Flag to indicate if MCU can go to low power mode
 *        When 0, MCU is authorized to go in low power mode
 */
static uint32_t LowPower_State = 0;

/* Private function prototypes -----------------------------------------------*/
extern ADC_HandleTypeDef hadc;
extern UART_HandleTypeDef huart1;
extern void MX_USART1_UART_Init(void);
//extern void MX_DMA_Init(void);
//extern void MX_GPIO_Init(void);

/* Exported functions ---------------------------------------------------------*/


/**
  * @brief Enters Low Power Stop Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void HW_EnterStopMode( void)
{
  BACKUP_PRIMASK();

  DISABLE_IRQ( );

  HW_DeInit( );
  
  /*clear wake up flag*/
  SET_BIT(PWR->CR, PWR_CR_CWUF);

  RESTORE_PRIMASK( );

  /* Enter Stop Mode */
  HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/**
  * @brief Exists Low Power Stop Mode
  * @note Enable the pll at 32MHz
  * @param none
  * @retval none
  */
void HW_ExitStopMode( void)
{
  /* Disable IRQ while the MCU is not running on HSI */

  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );


  Hw_Init();


  RESTORE_PRIMASK( );
}

/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exits the function when waking up
  * @param none
  * @retval none
  */
void HW_EnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/**
 * \brief API to set flag allowing power mode
 *
 * \param [IN] enum e_LOW_POWER_State_Id_t  
 */
void LowPower_Disable( e_LOW_POWER_State_Id_t state )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  LowPower_State |= state;

  RESTORE_PRIMASK( );
}

/**
 * \brief API to reset flag allowing power mode
 *
 * \param [IN] enum e_LOW_POWER_State_Id_t 
 */
void LowPower_Enable( e_LOW_POWER_State_Id_t state )
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  LowPower_State &= ~state;
  
  RESTORE_PRIMASK( );
}

/**
 * \brief API to get flag allowing power mode
 * \note When flag is 0, low power mode is allowed
 * \param [IN] state
 * \retval flag state 
 */
uint32_t LowPower_GetState( void )
{
  return LowPower_State;
}

/**
 * @brief  Handle Low Power
 * @param  None
 * @retval None
 */

void LowPower_Handler( void )
{
//  DBG_GPIO_RST(GPIOB, GPIO_PIN_15);
  
//  DBG_GPIO_RST(GPIOB, GPIO_PIN_14);
	
#if defined(ZED)
  
  if ( LowPower_State == 0 )
  {    
    
//    DBG_PRINTF_CRITICAL("dz\n\r");
    
//	PRINTF("enter sleep,rtc:%d!\r\n", RTC_Cnt);

    HW_EnterStopMode( );
    
    /* mcu dependent. to be implemented by user*/
    HW_ExitStopMode();
//    DBG_GPIO_SET(GPIOB, GPIO_PIN_15);
    
//	SystemClock_Config();	//add by dozen 
    HW_RTC_setMcuWakeUpTime( );

//	PRINTF("Wakeup,rtc:%d!\r\n", RTC_Cnt);

//  	PRINTF("wake up!\r\n");

  }
  else
  {
  	
     PRINTF("Not Enter Stop Mode!\r\n");
 //   DBG_PRINTF_CRITICAL("z\n\r");
    
//    HW_EnterSleepMode( );

//    DBG_GPIO_SET(GPIOB, GPIO_PIN_14);

//	//test for power
//	HW_EnterStopMode( );
//    
//    /* mcu dependent. to be implemented by user*/
//    HW_ExitStopMode();
//    
//    
//    HW_RTC_setMcuWakeUpTime( );
  }

  #endif
  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


