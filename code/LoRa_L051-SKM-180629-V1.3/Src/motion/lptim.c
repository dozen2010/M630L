/**
  ******************************************************************************
  * File Name          : LPTIM.c
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lptim.h"

//#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

LPTIM_HandleTypeDef hlptim1;

/* LPTIM1 init function */
void MX_LPTIM1_Init(void)
{

  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_ULPTIM;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* lptimHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(lptimHandle->Instance==LPTIM1)
  {
  /* USER CODE BEGIN LPTIM1_MspInit 0 */

  /* USER CODE END LPTIM1_MspInit 0 */
    /* LPTIM1 clock enable */
    __HAL_RCC_LPTIM1_CLK_ENABLE();
  
    /**LPTIM1 GPIO Configuration    
    PB5     ------> LPTIM1_IN1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_LPTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN LPTIM1_MspInit 1 */

  /* USER CODE END LPTIM1_MspInit 1 */
  }
}

void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* lptimHandle)
{

  if(lptimHandle->Instance==LPTIM1)
  {
  /* USER CODE BEGIN LPTIM1_MspDeInit 0 */

  /* USER CODE END LPTIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM1_CLK_DISABLE();
  
    /**LPTIM1 GPIO Configuration    
    PB5     ------> LPTIM1_IN1 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  /* USER CODE BEGIN LPTIM1_MspDeInit 1 */

  /* USER CODE END LPTIM1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void Motion_GPIO_Init(void)
{	
	
	GPIO_InitTypeDef GPIO_InitStruct;     //sense_vdd PA11

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin  = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void LPMoveCount_Start(void)
{
	MX_LPTIM1_Init();
//	Motion_GPIO_Init();
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_LPTIM_Counter_Start(&hlptim1, 65535);
}

void LPMoveCount_Stop(void)
{
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_LPTIM_Counter_Stop(&hlptim1);
	HAL_LPTIM_DeInit(&hlptim1);
}

uint32_t GetLPMoveCount(void)
{
	static uint32_t pre_cnt;
	uint32_t cnt, retdata;
	
	cnt = HAL_LPTIM_ReadCounter(&hlptim1);
	if(cnt >= pre_cnt)
	{
		retdata = cnt - pre_cnt;
	}
	else
	{
		retdata = 65535 - pre_cnt + cnt;
	}		
	pre_cnt = cnt;

	return retdata;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
