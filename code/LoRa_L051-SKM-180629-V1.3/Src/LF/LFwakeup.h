#ifndef __PWM_H
#define __PWM_H

#include "stm32L0xx.h"

//���û��ṩ

#define  LF_PWM_PORT    GPIOB
#define  LF_PWM_PIN     GPIO_PIN_4       //PWM TIM22_CH1
#define  SPK_PWM_PORT   GPIOA
#define  SPK_PWM_PIN    GPIO_PIN_15      //TIM2_CH1

#define  PWM_ON         HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1)  //(TIM4->CCER &=~(0x0100)) // TIM4_CH3
#define  PWM_OFF        HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_1)  //(TIM4->CCER |=0x0100)   	// TIM4_CH3
#define  BITTIMESTART   HAL_TIM_Base_Start_IT(&htim6)
#define  BITTIMESTOP    HAL_TIM_Base_Stop_IT(&htim6)



/************��λ������************/
typedef enum
{
	LF_Exit = 0,		//���ڼ���
	LF_Normal = 0xFF,	//��ͨ11λ��λ
	//�Ժ����������Type����
}LF_Typedef;


//���ṩ
void LF_Reset(void);
void LF_Init(void);

void PWM_GPIO_Init(void);

void LF_SendBit(void);
void LF_Awaken(u16 Address,LF_Typedef LF_Type);
void LF_SendConfig(const u8 *Data);

#endif
