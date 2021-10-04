/*
 * pickAndDrop.h
 *
 *  Created on: Apr 10, 2021
 *      Author: pika
 */

#ifndef INC_PICKANDDROP_H_
#define INC_PICKANDDROP_H_

#include "main.h"
#include "cmsis_os.h"

#define HAL_SENSOR_UP_GET HAL_GPIO_ReadPin(HAL_SENSOR_A_GPIO_Port, HAL_SENSOR_A_Pin)
#define HAL_SENSOR_DOWN_GET  HAL_GPIO_ReadPin(HAL_SENSOR_B_GPIO_Port, HAL_SENSOR_B_Pin)
#define SERVO_PICKUP  	TIM4->CCR3=1650
#define SERVO_DROP		TIM4->CCR3=1200
#define SERVO_EXTERN	TIM4->CCR3=1000
#define SERVO_DISABLE	TIM4->CCR3=0;

#endif /* INC_PICKANDDROP_H_ */
