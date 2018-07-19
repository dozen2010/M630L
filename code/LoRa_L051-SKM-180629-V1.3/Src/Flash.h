/*

Description: AwareTag Falsh API 

License: 

Maintainer: Hangzhou AwareTag tech
*/
 /******************************************************************************
  * @file    Falsh.h
  * @author  dozen Yao
  * @version V0.0.1
  * @date    2017/09/21
  * @brief   Falsh API
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef __FLASH_H__
#define __FLASH_H__


#ifdef __cplusplus
 extern "C" {
#endif

// usr code begin ///
#include "stm32l0xx.h"
#include "stm32l0xx_hal_flash.h"
// usr code end ///


#include <string.h>
#include "Aware_Config.h"
#include "Aware_Protocol.h"

//#include "common.h"


//#if defined(STM32F10X_HD)

//#define NET_NODE_TABLE_ADDR        (0x0803F000)     //126 page 2K
//#define DATA_EEPROM_BASE           (0x0803F800)     //127 page 2K

//#elif defined(STM32F10X_MD)

//#define NET_NODE_TABLE_ADDR        (0x0800F800)     //62 page 1K
//#define DATA_EEPROM_BASE           (0x0800FC00)     //63 page 1K

//#else

//#endif

// usr code begin ///


#define NV_PARA_LEN                                 32//0x40
#define APP_DATA_LEN                                64//0x24
//#define CON_DATA_LEN                                32//0x24


#define NET_NODE_TABLE_ADDR        (DATA_EEPROM_BASE + 100)     //2K eeprom

#define APP_DATA_ADDR              (DATA_EEPROM_BASE + 32)

//#define CONFIG_DATA_ADD            (DATA_EEPROM_BASE + 64)

//#define DATA_EEPROM_BASE           (0x080C0000)     //sector 10 128K

// usr code end ///



extern NV_Para_Typedef Nv_Para;

#if defined(COO)|| defined(Router)

extern Router_Table_Typedef RoutTable[MAX_ROUTABLE];

#endif
////////////////////  FLASH   //////////////////////
void Erase_Block_Flash(u32 Addr, u32 len);

void Write_Data_To_Flash(u32 Addr, u8* data, u32 len);

void Read_Data_From_Flash(u32 Addr, u8* data, u32 len);



#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H__ */


