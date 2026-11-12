/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#define DRV_ENABLE_Pin GPIO_PIN_3
#define DRV_ENABLE_GPIO_Port GPIOA
#define DRV_CAL_Pin GPIO_PIN_4
#define DRV_CAL_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define DRV_FAULT_Pin GPIO_PIN_1
#define DRV_FAULT_GPIO_Port GPIOB
#define ADC_IN15_Pin GPIO_PIN_15
#define ADC_IN15_GPIO_Port GPIOB
#define GPIO_Output_Pin GPIO_PIN_15
#define GPIO_Output_GPIO_Port GPIOA
#define ENC_CS_Pin GPIO_PIN_6
#define ENC_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
