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
#include "common.h"

/* USER CODE BEGIN 0 */
//#define FREQ_CNT 10
//static uint8_t flag_capture = 0;
static volatile uint8_t get_flag = 0;
//static volatile uint8_t freq_index = 0;
//static volatile uint32_t freq_input[FREQ_CNT];
static uint32_t LSI_Freq;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;
TIM_HandleTypeDef htim6;


/* TIM6 init function */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;            //16M
  htim6.Init.Prescaler = 15;   //1M 
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 350;   // 350us
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
} 


/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

	////// 16M / 128  = 125K or 4M /32 = 125K
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 127;//33-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 64; //16M 50%, //16; 4M 50%    
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}


/* TIM2 init function */
void MX_TIM22_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

	////// 16M / 128  = 125K or 4M /32 = 125K    //for LF  125K 
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 127;//33-1;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 64; //16M 50%, //16; 4M 50%    
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim22);

}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else   if(tim_pwmHandle->Instance==TIM22)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM22_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM22_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM22_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM22)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM22_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM22_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  
  if(timHandle->Instance == TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  	__HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE END TIM2_MspPostInit 0 */
  
       /**TIM2 GPIO Configuration    
    PA15     ------> TIM2_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /* USER CODE BEGIN TIM2_MspPostInit 1 */
  
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  
  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(timHandle->Instance == TIM22)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

//  	__HAL_RCC_GPIOA_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE END TIM2_MspPostInit 0 */
  
       /**TIM2 GPIO Configuration    
//    PA15     ------> TIM2_CH1
    PB4     ------> TIM22_CH1 
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF5_TIM2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM22;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	

  /* USER CODE BEGIN TIM2_MspPostInit 1 */
  
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  /* USER CODE END TIM2_MspPostInit 1 */
  }

}


/* TIM21 init function */
void MX_TIM21_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
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
uint32_t BSP_Get_RTCFreq(void)
{
	return LSI_Freq;
}

void BSP_LSI_Calibrator(void)
{
	HAL_Delay(100);
	HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1);
	while(!get_flag);	
	HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);
}

//void BSP_LSI_Calibrator(void)
//{
//	HAL_SuspendTick();
//	while(freq_index < FREQ_CNT)
//	{
//		flag_capture = 0;
//		get_flag = 0;
//		HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1);
//		while(!get_flag);	
//		HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);

//		freq_index++;		
//	}
//	for(uint8_t i = 0; i < FREQ_CNT; i++)
//		LSI_Freq += freq_input[i];
//	LSI_Freq /= FREQ_CNT;
//	HAL_ResumeTick();
//}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t flg = 0;
	static uint32_t start_value = 0;
	
	if (htim->Instance == TIM21)
	{
		
		if(flg == 0)
		{
			flg = 1;
			start_value = htim21.Instance->CCR1;
		}
		else if(flg == 1)
		{
			flg = 0;
					
			LSI_Freq = HAL_RCC_GetHCLKFreq() * 8 / ((uint32_t)(htim21.Instance->CCR1 - start_value));
			get_flag = 1;

		}
	}
}

/**
* @brief This function handles TIM21 global interrupt.
*/
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */

  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/**
* @brief This function handles TIM21 global interrupt.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */

  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}


//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	static uint16_t start_value = 0, end_value = 0;
//	uint16_t time_cnt;
//	uint32_t time_value;
//	
//	if (htim->Instance == TIM21)
//	{
//		time_cnt = htim21.Instance->CCR1;
//		if(flag_capture == 0)
//		{
//			flag_capture = 1;
//			start_value = time_cnt;
//		}
//		else if(flag_capture == 1)
//		{
//			flag_capture = 0;
//			end_value = time_cnt;
//			if(end_value > start_value)
//			{
//				time_value = end_value - start_value;
//			}
//			else if(end_value < start_value)
//			{
//				time_value = 0xFFFF - start_value + end_value;
//			}
//			else
//			{
//				time_value = 0;
//			}
//			
//			freq_input[freq_index] = HAL_RCC_GetHCLKFreq() * 8 / time_value;
//			get_flag = 1;

//		}
//	}
//}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
