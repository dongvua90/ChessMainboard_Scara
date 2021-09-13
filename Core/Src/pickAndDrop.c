/* Cac chuc nang lien quan den pickup(gap) va drop(tha) piece(quan co)
 * Control:
 * 			-Servo:		using TIM14 PWM
 * 			-J3Motor: 	using TIM3 PWM Chanel1 & Chanel2
 * 			-Hal_Sensor: GPIO Input */
#include "pickAndDrop.h"


extern TIM_HandleTypeDef htim4;	// For Servo
extern TIM_HandleTypeDef htim5;	// For Motor_J3


void pickAndDropInit()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Start PWM for Servo
	osDelay(100);
	SERVO_DROP;
	j3Stop();
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	// Start PWM for Servo_J3
}
/* Control Motor J3 (0 < speed <= 1000) */
void j3Up()
{
//	HAL_GPIO_WritePin(J3_A_GPIO_Port, J3_A_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(J3_B_GPIO_Port, J3_B_Pin, GPIO_PIN_RESET);
	TIM4->CCR4=2000;
}
void j3Down(){
//	HAL_GPIO_WritePin(J3_A_GPIO_Port, J3_A_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(J3_B_GPIO_Port, J3_B_Pin, GPIO_PIN_SET);
	TIM4->CCR4=1580;
}
void j3Stop()
{
//	HAL_GPIO_WritePin(J3_A_GPIO_Port, J3_A_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(J3_B_GPIO_Port, J3_B_Pin, GPIO_PIN_RESET);
	TIM4->CCR4=0;
}
void j3MoveUp()
{
	uint16_t left = 1000; 	// thoi gian dichuyen laf 1000ms
	j3Up();							// Move Up
	while(HAL_SENSOR_UP_GET==1 && left>0){
		osDelay(1);		// Wait for finish
		left--;
	}
	j3Stop();									// Stop
	osDelay(300);
}
void j3MoveDown()
{
	uint16_t lefttim = 1000; 	// thoi gian dichuyen laf 1000ms
	j3Down();							// Move Down
	while(HAL_SENSOR_DOWN_GET==1 && lefttim>0){
		osDelay(1);		// Wait for finish
		lefttim--;
	}
	j3Stop();									// Stop
	osDelay(300);
}
void pickupPiece()
{
	SERVO_DROP;
	osDelay(200);
	j3MoveDown();
	SERVO_PICKUP;
	osDelay(200);
	j3MoveUp();
}
void dropPiece()
{
	j3MoveDown();
	SERVO_DROP;
	osDelay(200);
	j3MoveUp();
}
