/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define HC_SR04p_Trig_Pin GPIO_PIN_0
#define HC_SR04p_Trig_GPIO_Port GPIOA
#define HC_SR04p_Echo_Pin GPIO_PIN_1
#define HC_SR04p_Echo_GPIO_Port GPIOA
#define LINE_TRACK_LEFT_Pin GPIO_PIN_2
#define LINE_TRACK_LEFT_GPIO_Port GPIOA
#define LINE_TRACK_RIGHT_Pin GPIO_PIN_3
#define LINE_TRACK_RIGHT_GPIO_Port GPIOA
#define WS2812_Pin GPIO_PIN_7
#define WS2812_GPIO_Port GPIOA
#define MOTOR1_A_Pin GPIO_PIN_0
#define MOTOR1_A_GPIO_Port GPIOB
#define MOTOR1_B_Pin GPIO_PIN_1
#define MOTOR1_B_GPIO_Port GPIOB
#define MOTOR1_PWM_Pin GPIO_PIN_8
#define MOTOR1_PWM_GPIO_Port GPIOA
#define MOTOR2_PWM_Pin GPIO_PIN_9
#define MOTOR2_PWM_GPIO_Port GPIOA
#define MOTOR2_A_Pin GPIO_PIN_10
#define MOTOR2_A_GPIO_Port GPIOA
#define MOTOR2_B_Pin GPIO_PIN_11
#define MOTOR2_B_GPIO_Port GPIOA
#define IR_Pin GPIO_PIN_15
#define IR_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
