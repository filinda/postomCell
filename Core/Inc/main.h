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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mosfet1_Pin GPIO_PIN_2
#define mosfet1_GPIO_Port GPIOC
#define mosfet2_Pin GPIO_PIN_3
#define mosfet2_GPIO_Port GPIOC
#define mosfet4_Pin GPIO_PIN_4
#define mosfet4_GPIO_Port GPIOC
#define mosfet3_Pin GPIO_PIN_5
#define mosfet3_GPIO_Port GPIOC
#define side8_Pin GPIO_PIN_2
#define side8_GPIO_Port GPIOB
#define side7_Pin GPIO_PIN_12
#define side7_GPIO_Port GPIOB
#define side6_Pin GPIO_PIN_6
#define side6_GPIO_Port GPIOC
#define side5_Pin GPIO_PIN_7
#define side5_GPIO_Port GPIOC
#define side4_Pin GPIO_PIN_8
#define side4_GPIO_Port GPIOC
#define side3_Pin GPIO_PIN_9
#define side3_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_8
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_9
#define LED_GREEN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_10
#define LED_BLUE_GPIO_Port GPIOA
#define side2_Pin GPIO_PIN_12
#define side2_GPIO_Port GPIOA
#define side1_Pin GPIO_PIN_2
#define side1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
