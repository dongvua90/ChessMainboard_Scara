/* Cac chuc nang lien quan den pickup(gap) va drop(tha) piece(quan co)
 * Control:
 * 			-Servo:		using TIM14 PWM
 * 			-J3Motor: 	using TIM3 PWM Chanel1 & Chanel2
 * 			-Hal_Sensor: GPIO Input */
#include "pickAndDrop.h"


extern TIM_HandleTypeDef htim4;	// For Servo

//
//void pickAndDropInit()
//{
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Start PWM for Servo
//	SERVO_DROP;
//}

