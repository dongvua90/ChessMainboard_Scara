/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN1_Pin GPIO_PIN_13
#define BTN1_GPIO_Port GPIOC
#define HAL_SENSOR_B_Pin GPIO_PIN_14
#define HAL_SENSOR_B_GPIO_Port GPIOC
#define HAL_SENSOR_A_Pin GPIO_PIN_15
#define HAL_SENSOR_A_GPIO_Port GPIOC
#define J3_STEP_TIM5_CH1_Pin GPIO_PIN_0
#define J3_STEP_TIM5_CH1_GPIO_Port GPIOA
#define J3_DIR_Pin GPIO_PIN_1
#define J3_DIR_GPIO_Port GPIOA
#define ADC_VOLT_SENSOR_Pin GPIO_PIN_4
#define ADC_VOLT_SENSOR_GPIO_Port GPIOA
#define J2_STEP_TIM2_CH1_Pin GPIO_PIN_5
#define J2_STEP_TIM2_CH1_GPIO_Port GPIOA
#define J2_DIR_Pin GPIO_PIN_6
#define J2_DIR_GPIO_Port GPIOA
#define J2_EN_Pin GPIO_PIN_7
#define J2_EN_GPIO_Port GPIOA
#define J1_STEP_TIM3_CH3_Pin GPIO_PIN_0
#define J1_STEP_TIM3_CH3_GPIO_Port GPIOB
#define J1_DIR_Pin GPIO_PIN_1
#define J1_DIR_GPIO_Port GPIOB
#define J1_EN_Pin GPIO_PIN_2
#define J1_EN_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_12
#define BTN2_GPIO_Port GPIOB
#define BATTERY_CHANGER_Pin GPIO_PIN_14
#define BATTERY_CHANGER_GPIO_Port GPIOB
#define OUT_GND_Pin GPIO_PIN_15
#define OUT_GND_GPIO_Port GPIOB
#define AS5600_2_SCL_Pin GPIO_PIN_8
#define AS5600_2_SCL_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define AS5600_2_SDA_Pin GPIO_PIN_4
#define AS5600_2_SDA_GPIO_Port GPIOB
#define J3_EN_Pin GPIO_PIN_5
#define J3_EN_GPIO_Port GPIOB
#define AS5600_1_SCL_Pin GPIO_PIN_6
#define AS5600_1_SCL_GPIO_Port GPIOB
#define AS5600_1_SDA_Pin GPIO_PIN_7
#define AS5600_1_SDA_GPIO_Port GPIOB
#define SERVO1_TIM4_CH3_Pin GPIO_PIN_8
#define SERVO1_TIM4_CH3_GPIO_Port GPIOB
#define SERVO_J3_TIM4_CH4_Pin GPIO_PIN_9
#define SERVO_J3_TIM4_CH4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
