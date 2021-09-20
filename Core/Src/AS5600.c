#include "AS5600.h"
#include "stdbool.h"

//#define USER_DEBUG
#define TIM_AS5600 			TIM10
#define TIM_AS5600_UPDATE	htim10
#define AS5600_1_I2C		I2C1
#define AS5600_2_I2C 		I2C3
extern I2C_HandleTypeDef 	hi2c1;
extern I2C_HandleTypeDef	hi2c3;


uint16_t data_AS5600_M1,data_AS5600_M2;

bool FLAG_AS5600_M1 = HAL_OK, FLAG_AS5600_M2 = HAL_OK;



void AS5600_M1_getPOS(){
	HAL_I2C_Mem_Read_DMA(&hi2c1,0x36<<1,_RAWANGLEAddressLSB,1,(uint8_t*)&data_AS5600_M1,2);
}
void AS5600_M2_getPOS(){
	HAL_I2C_Mem_Read_DMA(&hi2c3,0x36<<1,_RAWANGLEAddressLSB,1,(uint8_t *)&data_AS5600_M2,2);
}
char AS5600_M1_status(){
	uint8_t data_status[1];
  HAL_I2C_Mem_Read(&hi2c1,0x36<<1,_STATUSAddress,1,data_status,1,10);

	switch(data_status[0]){
		case 39: return 1; // phat hien nam cham qua manh
		case 55: return 2;  //phat hien nam cham binh thuong
		case 23: return 3; //phat hien nam cham yeu
	}
	return data_status[0];
}
char AS5600_M2_status(){
	uint8_t data_status[1];
  HAL_I2C_Mem_Read(&hi2c3,0x36<<1,_STATUSAddress,1,data_status,1,10);

	switch(data_status[0]){
		case 39: return 1; // phat hien nam cham qua manh
		case 55: return 2;  //phat hien nam cham binh thuong
		case 23: return 3; //phat hien nam cham yeu
	}
	return data_status[0];
}
void AS5600_Start_Update(){
	TIM_AS5600->ARR = 1000;         //for frequency = 100hz
	HAL_TIM_Base_Start_IT(&TIM_AS5600_UPDATE);
}
void AS5600_Start_Update_Low(){
	TIM_AS5600->ARR = 1000;         //for frequency = 100hz
}
void AS5600_Start_Update_High(){
	TIM_AS5600->ARR = 800;          //for frequency = 8khz
}

