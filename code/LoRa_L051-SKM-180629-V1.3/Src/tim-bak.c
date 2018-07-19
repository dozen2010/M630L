/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
static uint8_t get_flag = 0;
static uint32_t freq_input;
/* USER CODE END 0 */

TIM_HandleTypeDef htim21;

/* TIM21 init function */
void MX_TIM21_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;//8-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 0xFFFF;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim21, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIMEx_RemapConfig(&htim21, TIM21_TI1_LSI) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM21)
  {
  /* USER CODE BEGIN TIM21_MspInit 0 */

  /* USER CODE END TIM21_MspInit 0 */
    /* TIM21 clock enable */
    __HAL_RCC_TIM21_CLK_ENABLE();

    /* TIM21 interrupt Init */
    HAL_NVIC_SetPriority(TIM21_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM21_IRQn);
  /* USER CODE BEGIN TIM21_MspInit 1 */

  /* USER CODE END TIM21_MspInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM21)
  {
  /* USER CODE BEGIN TIM21_MspDeInit 0 */

  /* USER CODE END TIM21_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM21_CLK_DISABLE();

    /* TIM21 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM21_IRQn);
  /* USER CODE BEGIN TIM21_MspDeInit 1 */

  /* USER CODE END TIM21_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
uint32_t Get_RTC_frequency(void)
{
	HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1);	
//	HAL_NVIC_DisableIRQ(RTC_IRQn);
	while(!get_flag);	
//	HAL_NVIC_EnableIRQ(RTC_IRQn);
	HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);
	
	return freq_input;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t start_value = 0, end_value = 0;
	static uint8_t flag_capture = 0;
	uint32_t time_value;
	if (htim->Instance == TIM21)
	{
		if(flag_capture == 0)
		{
			flag_capture = 1;
			start_value = htim21.Instance->CCR1;
		}
		else if(flag_capture == 1)
		{
			flag_capture = 0;
			end_value = htim21.Instance->CCR1;
			if(end_value > start_value)
			{
				time_value = end_value - start_value;
			}
			else if(end_value < start_value)
			{
				time_value = 0xFFFF - start_value + end_value;
			}
			else
			{
				time_value = 0;
			}
			
			freq_input = HAL_RCC_GetHCLKFreq()*8 / time_value;
			get_flag = 1;
		}
	}
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
