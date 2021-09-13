/*
 * batteryVolt.c
 *
 *  Created on: Apr 10, 2021
 *      Author: pika
 */
#include "batteryVolt.h"

extern ADC_HandleTypeDef hadc1;

uint32_t battery_sum;								// Variable for Calculator Battery
uint16_t battery,battery_tik;						// Variable Battery: ex. battery=1623 => 16.23v

/* Ham Callback khi chuyen doi ADC hoan tat */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	battery_sum += ADC1->DR;						// tinh gia tri ADC trung binh trong 65000 lan lay mau , de dat duoc ket qua chinh xac
	battery_tik ++;
	if(battery_tik==65000){
		battery = ((battery_sum/battery_tik)*510)/100;  	//  Ratio ADC = 16,3/1,04 => Vbattery = Val_ADC*(3,3/1024)*(16,3/1,04) with ADC is 10bit
		battery_sum =0;
		battery_tik=0;
	}
}

void batteryVoltInit()
{
	HAL_ADC_Start_IT(&hadc1);						// Start ADC to get Battery Votl
}
uint16_t batteryGet()
{
	return battery;
}
