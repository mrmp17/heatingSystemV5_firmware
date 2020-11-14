/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CH_ILIM_CTRL_Pin GPIO_PIN_14
#define CH_ILIM_CTRL_GPIO_Port GPIOC
#define CH_CE_Pin GPIO_PIN_15
#define CH_CE_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_1
#define BUTTON_GPIO_Port GPIOA
#define BUTTON_EXTI_IRQn EXTI0_1_IRQn
#define FORCE_SNS_PEN_Pin GPIO_PIN_2
#define FORCE_SNS_PEN_GPIO_Port GPIOA
#define USB_CC1_ANALOG_Pin GPIO_PIN_3
#define USB_CC1_ANALOG_GPIO_Port GPIOA
#define USB_CC2_ANALOG_Pin GPIO_PIN_4
#define USB_CC2_ANALOG_GPIO_Port GPIOA
#define VBAT_SNS_Pin GPIO_PIN_5
#define VBAT_SNS_GPIO_Port GPIOA
#define VBAT_SNS_CTRL_Pin GPIO_PIN_6
#define VBAT_SNS_CTRL_GPIO_Port GPIOA
#define HTR_DET_ADC_Pin GPIO_PIN_7
#define HTR_DET_ADC_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_15
#define LD3_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_3
#define LD2_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_4
#define LD1_GPIO_Port GPIOB
#define CH_STAT_Pin GPIO_PIN_5
#define CH_STAT_GPIO_Port GPIOB
#define CH_PG_Pin GPIO_PIN_6
#define CH_PG_GPIO_Port GPIOB
#define HTR_DET_Pin GPIO_PIN_7
#define HTR_DET_GPIO_Port GPIOB
#define HTR_SW_CTRL_Pin GPIO_PIN_8
#define HTR_SW_CTRL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
