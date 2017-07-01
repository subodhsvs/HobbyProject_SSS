/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "cube_hal.h"
#ifdef USE_IKS01A2
#include "x_nucleo_iks01a2.h"
#include "x_nucleo_iks01a2_accelero.h"
#include "x_nucleo_iks01a2_gyro.h"
#include "x_nucleo_iks01a2_magneto.h"
#include "x_nucleo_iks01a2_pressure.h"
#include "x_nucleo_iks01a2_temperature.h"
#include "x_nucleo_iks01a2_humidity.h"

#elif USE_IKS01A1
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_humidity.h"
#endif
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RC_CH5_Pin GPIO_PIN_1
#define RC_CH5_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MOTOR1_Pin GPIO_PIN_6
#define MOTOR1_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_7
#define MOTOR2_GPIO_Port GPIOA
#define MOTOR3_Pin GPIO_PIN_0
#define MOTOR3_GPIO_Port GPIOB
#define MOTOR4_Pin GPIO_PIN_1
#define MOTOR4_GPIO_Port GPIOB
#define RC_CH6_Pin GPIO_PIN_10
#define RC_CH6_GPIO_Port GPIOB
#define RC_CH1_Pin GPIO_PIN_8
#define RC_CH1_GPIO_Port GPIOA
#define RC_CH2_Pin GPIO_PIN_9
#define RC_CH2_GPIO_Port GPIOA
#define RC_CH3_Pin GPIO_PIN_10
#define RC_CH3_GPIO_Port GPIOA
#define RC_CH4_Pin GPIO_PIN_11
#define RC_CH4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Definition for TIMx clock resources : Timer used for PE algorithm */
#define TIM_FX                          TIM3
#define TIM_FX_CLK_ENABLE               __TIM3_CLK_ENABLE
#define TIM_FX_CLK_DISABLE              __TIM3_CLK_DISABLE

/* Definition for TIMx's NVIC */
#define TIM_FX_IRQn                     TIM3_IRQn
#define TIM_FX_IRQHandler               TIM3_IRQHandler

/* Enable sensor masks */
#define PRESSURE_SENSOR                 ((uint32_t)0x00000001)
#define TEMPERATURE_SENSOR              ((uint32_t)0x00000002)
#define HUMIDITY_SENSOR                 ((uint32_t)0x00000004)
#define ACCELEROMETER_SENSOR            ((uint32_t)0x00000010)
#define GYROSCOPE_SENSOR                ((uint32_t)0x00000020)
#define MAGNETIC_SENSOR                 ((uint32_t)0x00000040)

/* Exported functions ------------------------------------------------------- */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);
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
