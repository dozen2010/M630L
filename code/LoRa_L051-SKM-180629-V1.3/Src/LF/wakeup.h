#ifndef __AwakeUp_H 
#define __AwakeUp_H

#include "stm32l0xx_hal.h"
#include "tim.h"
//#define     PWM_ON		
//#define     PWM_OFF	
#define PWM_INIT   MX_TIM2_Init();
#define PWM_ON     HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);            //?????ACLK,???A?????//??P1.2,P1.3 TA1/2?????
#define PWM_OFF    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);//{P2SEL &= ~PWMIO;}
//extern void delay_ms1(int32_t ms) ; 
//extern void Delay_us1(uint32_t us);
extern void Time_1Bit(void);
extern void SendLF(uint8_t *Pdata);

#endif

