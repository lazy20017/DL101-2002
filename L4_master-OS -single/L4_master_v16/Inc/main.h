/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define AD_Efilde_1M_Pin GPIO_PIN_1
#define AD_Efilde_1M_GPIO_Port GPIOC
#define AD_CURRENT0_half_Pin GPIO_PIN_2
#define AD_CURRENT0_half_GPIO_Port GPIOC
#define AD_CURRENT_ALL_Pin GPIO_PIN_3
#define AD_CURRENT_ALL_GPIO_Port GPIOC
#define EXIT_dianliu_Pin GPIO_PIN_0
#define EXIT_dianliu_GPIO_Port GPIOA
#define EXIT_dianliu_EXTI_IRQn EXTI0_IRQn
#define EXIT_jiedi_Pin GPIO_PIN_1
#define EXIT_jiedi_GPIO_Port GPIOA
#define EXIT_jiedi_EXTI_IRQn EXTI1_IRQn
#define TX2_DEBUG_Pin GPIO_PIN_2
#define TX2_DEBUG_GPIO_Port GPIOA
#define RX2_DEBUG_Pin GPIO_PIN_3
#define RX2_DEBUG_GPIO_Port GPIOA
#define DA_VREF_duanliu_Pin GPIO_PIN_4
#define DA_VREF_duanliu_GPIO_Port GPIOA
#define DA_VREF_jiedi_Pin GPIO_PIN_5
#define DA_VREF_jiedi_GPIO_Port GPIOA
#define AD_Zaixian_Pin GPIO_PIN_6
#define AD_Zaixian_GPIO_Port GPIOA
#define AD_V1V2_Pin GPIO_PIN_7
#define AD_V1V2_GPIO_Port GPIOA
#define AD_Sunpwr_Pin GPIO_PIN_4
#define AD_Sunpwr_GPIO_Port GPIOC
#define AD_Libat_Pin GPIO_PIN_5
#define AD_Libat_GPIO_Port GPIOC
#define AD_Gan_Bat_Pin GPIO_PIN_0
#define AD_Gan_Bat_GPIO_Port GPIOB
#define ZAIXIN_IN2505_Ctrl_Pin GPIO_PIN_10
#define ZAIXIN_IN2505_Ctrl_GPIO_Port GPIOB
#define Gandianchi_V3V3_Ctrl_Pin GPIO_PIN_11
#define Gandianchi_V3V3_Ctrl_GPIO_Port GPIOB
#define EN_2505_Ctrl_Pin GPIO_PIN_13
#define EN_2505_Ctrl_GPIO_Port GPIOB
#define C433_CTRL_Pin GPIO_PIN_15
#define C433_CTRL_GPIO_Port GPIOB
#define GDO0_IRQ_Pin GPIO_PIN_6
#define GDO0_IRQ_GPIO_Port GPIOC
#define GDO0_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define GPIO2_INPUT_Pin GPIO_PIN_7
#define GPIO2_INPUT_GPIO_Port GPIOC
#define SPI3_NSSI_CSN_Pin GPIO_PIN_9
#define SPI3_NSSI_CSN_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOB
#define WDI1_Pin GPIO_PIN_7
#define WDI1_GPIO_Port GPIOB
#define Source_Ctrl_Pin GPIO_PIN_9
#define Source_Ctrl_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
