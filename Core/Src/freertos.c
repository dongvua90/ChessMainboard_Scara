/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "usart.h"
#include "AS5600.h"
#include "i2c.h"
#include "pickAndDrop.h"
#include "batteryVolt.h"
#include "AccelStepper.h"
#include <string.h>
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BTN1_PRESS HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)==GPIO_PIN_RESET
#define BTN2_PRESS HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin)==GPIO_PIN_RESET
#define BTN3_PRESS HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin)==GPIO_PIN_RESET
#define J1_SPEED		10000
#define J1_ACCEL_MAX	500000//300000
#define j1_ACCEL_MIN	100000
#define J2_SPEED		10000
#define J2_ACCEL_MAX	400000//200000
#define J2_ACCEL_MIN	100000

#define CMD_I2C_MOVE_PIECE				1
#define CMD_I2C_MOVE_HOME				2
#define CMD_I2C_MOVE_KILL				2
#define CMD_I2C_MOVE_CASLLINGG_QUEEN  	3
#define CMD_I2C_MOVE_CASLLING_KING    	4
#define CMD_I2C_MOVE_PASSANT			5
#define CMD_I2C_SETPOS_SQUARE			6


// User Debug
#define MDEBUG
//#define USERGETPOS // need disable TaskMotor


#define CMD_INFO		'I'
#define CMD_GRIPPER 	'G'
#define CMD_PICK		'p'
#define CMD_PICKPIECE	'P'
#define CMD_MOVE		'm'
#define CMD_MOVEPIECE	'M'
#define CMD_MOVEHOME  	'H'

#define CMD_GETPOS 		'S'
#define CMD_GETPOS_BACK 'B'
#define CMD_GETPOS_NEXT 'N'

#define ADDR_EE_POSJ1 0x5555
#define ADDR_EE_POSJ2 0x7777
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t uart2_main_buf[];
extern bool uart2_onData;
extern uint8_t uart2_data_length;
struct Point{
	uint16_t j1,j2;
};

struct Point square1,square2;
//const struct Point square[80]={
//		{2113,2830},{1792,2896},{1645,2887},{1609,2818},{1627,2726},{1667,2626},{1712,2518},{1769,2402},
//		{2349,2660},{2163,2690},{2002,2689},{1919,2650},{1879,2592},{1877,2511},{1896,2419},{1937,2312},
//		{2492,2514},{2360,2539},{2246,2537},{2154,2514},{2102,2462},{2087,2390},{2087,2305},{2115,2197},
//		{2590,2380},{2493,2398},{2407,2396},{2335,2375},{2292,2329},{2261,2267},{2260,2181},{2277,2073},
//		{2696,2226},{2603,2250},{2530,2247},{2474,2222},{2434,2173},{2408,2115},{2398,2038},{2414,1937},
//		{2805,2062},{2717,2082},{2648,2081},{2596,2059},{2560,2019},{2538,1961},{2534,1881},{2548,1776},
//		{2925,1884},{2837,1901},{2770,1902},{2724,1882},{2684,1851},{2673,1789},{2671,1716},{2703,1597},
//		{3068,1690},{2986,1712},{2915,1714},{2863,1701},{2837,1652},{2827,1591},{2841,1498},{2920,1325},
//	//		a			b			c			d			e			f			g			h
//		{1885,2187},{2021,2109},{2191,1988},{2330,1885},{2467,1744},{1984,2029},{2111,1975},{2229,1913},
//		{2365,1797},{2509,1650},{2064,1919},{2181,1856},{2301,1797},{2416,1696},{2536,1578},{2076,3040}};
//																	         // Home ------------^

struct Point square[80];

uint8_t square_kill_number=0;		// Variable save Square kiled
uint16_t num_squa=0;

void moveToSquare(uint8_t point,bool continues);		// Point: square in board , continues: tiep tuc di chuyen ngay sau do hay khong?
void movePiece(uint8_t qFrom,uint8_t qTo,uint8_t option);	// qFrom: square from is move , qTo: square to is move , option:MOVE_PIECE,MOVE_KILL,MOVE_CALLING
void moveToHome();										// move to home and off motor
void moveToKill();
void updateInfo();
void updateSquarePosition(uint16_t squa);
void readSquarePosition();
//test
void testDebug();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;						// AS5600_J1
extern I2C_HandleTypeDef hi2c3;						// AS5600_J2
extern TIM_HandleTypeDef htim3; 					// STEPPER_J1
extern TIM_HandleTypeDef htim2;						// STEPPER_J2
extern uint16_t data_AS5600_M1,data_AS5600_M2;		// Data of AS5600.c
struct AccelStepperData motor_j1_data,motor_j2_data;// Variable of Stepper
uint16_t accel_j1_tik=0,accel_j2_tik=0;				// Variable of Change Accel
bool new_master_cmd = false;						// su ly sau khi nhan duoc data tu master
uint8_t square_getpos=0;	// for debug getpos
/* Protocol i2c-interface Write: 4 Byte
* [CMD]--[DATA0]-[DATA1]-[DATA2]
* CMD: 1 => Move Piece , qFrom = DATA0 , sTo = DATA1 , option = DATA2.
* CMD: 2 => Move Home
* Protocol i2c-interface Read:	10 Byte
* [AS5600_J1]-[AS5600-J2]-[BATTERY]-[STATUS]
* 	2-Byte		2-Byte		2-Byte	  1-Byte
* 	STATUS: 0b76543210 => 0:Hal_sensor_up 1:Hal_sensor_down 2:Bat_Changer 3:isFinish
*/
uint8_t data_trans_master[7];
uint8_t data_rev_master[4];
bool moveIsFinish = false;



/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId motorJ1TaskHandle;
osThreadId motorJ2TaskHandle;
osThreadId moveTaskHandle;
osSemaphoreId binarySem_motorJ1Handle;
osSemaphoreId binarySem_motorJ2Handle;
osSemaphoreId binarySem_masterCmdHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// for Motor J1 J2
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)								// Stepper J1
	{
		motor_j1_data._currentPos = data_AS5600_M1;			// Set Current Position
		osSemaphoreRelease(binarySem_motorJ1Handle);		// Release Semaphore for Calculator Stepper (run)
		accel_j1_tik++;
		if(accel_j1_tik==1000)								// Changer Accel
		{
			accel_j1_tik=0;
			long distance = distanceToGo(&motor_j1_data);
			if(labs(distance) < 100){
				setAcceleration(&motor_j1_data, j1_ACCEL_MIN);
			}else{
				setAcceleration(&motor_j1_data, J1_ACCEL_MAX);
			}
		}
	}
	if(htim->Instance==TIM2)								// Stepper J2
		{
			motor_j2_data._currentPos = data_AS5600_M2;		// Set Current Position
			osSemaphoreRelease(binarySem_motorJ2Handle);	// Release Semaphore for Calculator Stepper (run)
			accel_j2_tik++;
			if(accel_j2_tik==500)
			{
				accel_j2_tik=0;
				long distance = distanceToGo(&motor_j2_data);
				if(labs(distance) < 100){
					setAcceleration(&motor_j2_data, J2_ACCEL_MIN);
				}else{
					setAcceleration(&motor_j2_data, J2_ACCEL_MAX);
				}
			}
		}
}

// I2C-Interface
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(hi2c->Instance==I2C2)
	{
		if(TransferDirection == I2C_DIRECTION_TRANSMIT)
		{
			HAL_I2C_Slave_Seq_Receive_DMA(&hi2c2,data_rev_master,4,I2C_FIRST_AND_LAST_FRAME);
		}else if(TransferDirection == I2C_DIRECTION_RECEIVE)
		{
			HAL_I2C_Slave_Seq_Transmit_DMA(&hi2c2, data_trans_master, 7, I2C_LAST_FRAME);
		}
	}
}
// 	I2C2 for interface
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(&hi2c2);
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance==I2C2)
	{
		osSemaphoreRelease(binarySem_masterCmdHandle);// sau khi nhan duoc lenh tu master
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
}

// Handler I2C Error
extern bool FLAG_AS5600_M1, FLAG_AS5600_M2;
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance==I2C1){
       FLAG_AS5600_M1 = HAL_ERROR;
	}
	if(hi2c->Instance==I2C3){
		FLAG_AS5600_M2 = HAL_ERROR;
	}
	 if(hi2c->Instance==I2C2){
		 HAL_I2C_DeInit(&hi2c2);
		 MX_I2C2_Init();
		 HAL_I2C_EnableListen_IT(&hi2c2);	// I2C2 for interface
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	 }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskMotorJ1(void const * argument);
void StartTaskMotorJ2(void const * argument);
void StartTaskMove(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of binarySem_motorJ1 */
  osSemaphoreDef(binarySem_motorJ1);
  binarySem_motorJ1Handle = osSemaphoreCreate(osSemaphore(binarySem_motorJ1), 1);

  /* definition and creation of binarySem_motorJ2 */
  osSemaphoreDef(binarySem_motorJ2);
  binarySem_motorJ2Handle = osSemaphoreCreate(osSemaphore(binarySem_motorJ2), 1);

  /* definition and creation of binarySem_masterCmd */
  osSemaphoreDef(binarySem_masterCmd);
  binarySem_masterCmdHandle = osSemaphoreCreate(osSemaphore(binarySem_masterCmd), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorJ1Task */
  osThreadDef(motorJ1Task, StartTaskMotorJ1, osPriorityHigh, 0, 512);
  motorJ1TaskHandle = osThreadCreate(osThread(motorJ1Task), NULL);

  /* definition and creation of motorJ2Task */
  osThreadDef(motorJ2Task, StartTaskMotorJ2, osPriorityHigh, 0, 512);
  motorJ2TaskHandle = osThreadCreate(osThread(motorJ2Task), NULL);

  /* definition and creation of moveTask */
  osThreadDef(moveTask, StartTaskMove, osPriorityRealtime, 0, 4096);
  moveTaskHandle = osThreadCreate(osThread(moveTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
#ifdef MDEBUG
	printf("Robochess 2021\r\n");
#endif
	readSquarePosition();
	HAL_I2C_EnableListen_IT(&hi2c2);	// I2C2 for interface
	AS5600_Start_Update();				// Start Tim10 & get data of AS5600
	batteryVoltInit();
	pickAndDropInit();
  /* Infinite loop */
  for(;;)
  {
	  updateInfo();

#ifdef MDEBUG
	  if(uart2_onData){
		  uart2_onData=false; //realease
		  printf("Rev %dBYTE: %s\r\n",uart2_data_length,uart2_main_buf);
		  // Gripper
		  if(uart2_main_buf[0]==CMD_GRIPPER){
			  char sval[1];
			  sval[0]=uart2_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  SERVO_DROP;
				  printf("Gripper open\r\n");
			  }else if(val==1){
				  SERVO_PICKUP;
				  printf("Gripper close\r\n");
			  }
		// Info
		  }else if(uart2_main_buf[0]==CMD_INFO){
		 	  printf("AS5600_J1:%d_J2:%d Hal_Up:%d_Down:%d Bat:%2d Status:%d\r\n",
		 			  data_AS5600_M1,data_AS5600_M2,HAL_SENSOR_UP_GET,HAL_SENSOR_DOWN_GET,
					  batteryGet(),data_trans_master[6]);
		// PICK
		  }else if(uart2_main_buf[0]==CMD_PICK){
			  char sval[1];
			  sval[0]=uart2_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  printf("J3 MOVE UP\r\n");
				  j3MoveUp();
			  }else if(val==1){
				  printf("J3 MOVE DOWN\r\n");
				  j3MoveDown();
			  }
		// PICK PIECE
		  }else if(uart2_main_buf[0]==CMD_PICKPIECE){
			  char sval[1];
			  sval[0]=uart2_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  printf("DROP PIECE\r\n");
				  dropPiece();
			  }else if(val==1){
				  printf("PICKUP PIECE\r\n");
				  pickupPiece();
			  }
		// MOVE
		  }else if(uart2_main_buf[0]==CMD_MOVE){
			  char sval[2];
			  if(uart2_data_length>2){
				  sval[0]=uart2_main_buf[1];
				  sval[1]=uart2_main_buf[2];
			  }else{
				  sval[0]='0';
				  sval[1]=uart2_main_buf[1];
			  }
			  int val=atoi(sval);
			  if(val>=0 && val<80){
				  printf("MOVE TO SQUARE:%d\r\n",val);
				  moveToSquare(val, false);
			  }else{
				  printf("ERROR! Cannot move to square %d\r\n",val);
			  }
		// MOVE PIECE
		  }else if(uart2_main_buf[0]==CMD_MOVEPIECE){
			  char sFrom[2],sTo[2],sOption[1];
			  sFrom[0]=uart2_main_buf[1];
			  sFrom[1]=uart2_main_buf[2];
			  sTo[0]=uart2_main_buf[3];
			  sTo[1]=uart2_main_buf[4];
			  sOption[0]=uart2_main_buf[5];
			  int _from=atoi(sFrom);
			  int _to =atoi(sTo);
			  int _option =atoi(sOption);
			  if((_from>=0 && _from<80) && (_to>=0 && _to<80)){
				  printf("MOVEPIECE %d->%d Option:%d\r\n",_from,_to,_option);
				  movePiece(_from, _to, _option);
			  }else{
				  printf("ERROR! Out Range of square\r\n");
			  }
		  }else if(uart2_main_buf[0]==CMD_MOVEHOME){
			  printf("MOVE HOME\r\n");
			  moveToHome();
		  }else if(uart2_main_buf[0]=='w'){
			  char sdat[3];
			  sdat[0]=uart2_main_buf[1];
			  sdat[1]=uart2_main_buf[2];
			  uint16_t squa = atoi(sdat);
			  num_squa = squa;
			  printf("set square to %d\r\n",num_squa);
		  }else if(uart2_main_buf[0]=='r'){
			  readSquarePosition();
		  }
		  else if(uart2_main_buf[0]=='W'){
			  if(num_squa>=80) return;
		  	updateSquarePosition(num_squa);
		  	num_squa++;
		  	printf("next square:%d\r\n",num_squa);
		  }
#ifdef USERGETPOS
			else if (uart2_main_buf[0] == CMD_GETPOS) {
				printf("%d.{%d,%d}\r\n",square_getpos,data_AS5600_M1,data_AS5600_M2);
				square_getpos++;
			}else if(uart2_main_buf[0] == CMD_GETPOS_BACK)
			{
				square_getpos--;
				printf("Square:%d\r\n",square_getpos);
			}else if(uart2_main_buf[0] == CMD_GETPOS_NEXT)
			{
				square_getpos++;
				printf("Square:%d\r\n",square_getpos);
			}
#endif
//		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
#endif
	 	  osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMotorJ1 */
/**
* @brief Function implementing the motorJ1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotorJ1 */
__weak void StartTaskMotorJ1(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotorJ1 */
		osDelay(2000);						// Wait for finish Init
		motor_j1_data.GPIO_PIN_Dir		= J1_DIR_Pin;
		motor_j1_data.GPIO_PORT_Dir		= J1_DIR_GPIO_Port;
		motor_j1_data.GPIO_PORT_Enable	= J1_EN_GPIO_Port;
		motor_j1_data.GPIO_PIN_Enable	= J1_EN_Pin;
		motor_j1_data.USER_TIMER		= TIM3;
		motor_j1_data.TIM_CHANEL		= TIM_CHANNEL_3;
		motor_j1_data.isStop			= false;
		AccelStepper_init(&motor_j1_data, htim3, data_AS5600_M1, J1_SPEED, J1_ACCEL_MAX);
#ifdef USERGETPOS
	enableStepper(&motor_j1_data, OFF);
	while(1){			// disable this task
		osDelay(1000);
	}
#endif
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(binarySem_motorJ1Handle, osWaitForever);
	  	  run(&motor_j1_data);
  }
  /* USER CODE END StartTaskMotorJ1 */
}

/* USER CODE BEGIN Header_StartTaskMotorJ2 */
/**
* @brief Function implementing the motorJ2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotorJ2 */
__weak void StartTaskMotorJ2(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotorJ2 */

		osDelay(2000);						// Wait for finish Init
		motor_j2_data.GPIO_PIN_Dir		= J2_DIR_Pin;
		motor_j2_data.GPIO_PORT_Dir		= J2_DIR_GPIO_Port;
		motor_j2_data.GPIO_PORT_Enable	= J2_EN_GPIO_Port;
		motor_j2_data.GPIO_PIN_Enable	= J2_EN_Pin;
		motor_j2_data.USER_TIMER		= TIM2;
		motor_j2_data.TIM_CHANEL		= TIM_CHANNEL_1;
		motor_j2_data.isStop			= false;
		AccelStepper_init(&motor_j2_data, htim2, data_AS5600_M2, J2_SPEED, J2_ACCEL_MAX);
#ifdef USERGETPOS
	enableStepper(&motor_j2_data, OFF);
	while(1){			// disable this task
		osDelay(1000);
	}
#endif
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(binarySem_motorJ2Handle, osWaitForever);
	  run(&motor_j2_data);
  }
  /* USER CODE END StartTaskMotorJ2 */
}

/* USER CODE BEGIN Header_StartTaskMove */
/**
* @brief Function implementing the moveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMove */
__weak void StartTaskMove(void const * argument)
{
  /* USER CODE BEGIN StartTaskMove */
	osDelay(2500);
  /* Infinite loop */
	for (;;) {
		osSemaphoreWait(binarySem_masterCmdHandle, osWaitForever);
		moveIsFinish = false;
		if (data_rev_master[0] == CMD_I2C_MOVE_PIECE) {
#ifdef MDEBUG
			printf("I2C-MOVE-%d->%d OP:%d\r\n",data_rev_master[1],data_rev_master[2],data_rev_master[3]);
#endif
			movePiece(data_rev_master[1], data_rev_master[2],data_rev_master[3]);
			moveIsFinish = true;
		} else if (data_rev_master[0] == CMD_I2C_MOVE_HOME) {
#ifdef MDEBUG
			printf("I2C-MOVEHOME\r\n");
#endif
			moveToHome();
			moveIsFinish = true;
		}else if(data_rev_master[0] == CMD_I2C_SETPOS_SQUARE){
			updateSquarePosition(data_rev_master[1]);
		}
		osDelay(10);
	}
  /* USER CODE END StartTaskMove */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void moveToSquare(uint8_t point,bool continues)
{
	AS5600_Start_Update_High();
	enableStepper(&motor_j1_data, ON);
	enableStepper(&motor_j2_data, ON);
	moveTo(&motor_j1_data,square[point].j1);
	run(&motor_j1_data);
	moveTo(&motor_j2_data,square[point].j2);
	run(&motor_j2_data);
	uint16_t check_time_out=0;
	while( isRunning(&motor_j1_data) || isRunning(&motor_j2_data) ){	// Waiting for move finish
		if( labs(distanceToGo(&motor_j1_data)) < 2  &&  labs(distanceToGo(&motor_j2_data)) < 2 ){		// neu dung sai la nho thi thoat trong khoang 500ms
			check_time_out++;
			if(check_time_out > 100) break;
		}
		osDelay(10);
	}
	motor_j1_data.isStop = true;
	motor_j2_data.isStop = true;

	if(continues==false)
	{
		enableStepper(&motor_j1_data, OFF);
		enableStepper(&motor_j2_data, OFF);
		AS5600_Start_Update_Low();
	}
}

void moveToHome()
{
	moveToSquare(79, false);
}
void moveToKill()
{
	moveToSquare(square_kill_number + 64, true);
	square_kill_number++;
}
void movePiece(uint8_t qFrom,uint8_t qTo,uint8_t option)
{
	if(option == CMD_I2C_MOVE_KILL)										// Neu la nuoc di an quan thi gap piece ra khoi ban co
	{
		moveToSquare(qTo, true);		// move to piece kill
		pickupPiece();					// pickup
		moveToKill();		// move to square die
		dropPiece();					// drop
	}else if(option == CMD_I2C_MOVE_CASLLING_KING)
	{
		moveToSquare(4, true);
		pickupPiece();
		moveToSquare(6, true);
		dropPiece();
		moveToSquare(7, true);
		pickupPiece();
		moveToSquare(5, true);
		dropPiece();
		moveToHome();
		return;
	}else if(option == CMD_I2C_MOVE_CASLLINGG_QUEEN)
	{
		moveToSquare(4, true);
		pickupPiece();
		moveToSquare(2, true);
		dropPiece();
		moveToSquare(0, true);
		pickupPiece();
		moveToSquare(3, true);
		dropPiece();
		moveToHome();
		return;
	}else if(option == CMD_I2C_MOVE_PASSANT){
		moveToSquare(qTo-8, true);
		pickupPiece();
		moveToKill();
		dropPiece();
		moveToSquare(qFrom, true);
		pickupPiece();
		moveToSquare(qTo, true);
		dropPiece();
		moveToHome();
		return;
	}
	moveToSquare(qFrom, true);
	pickupPiece();
	moveToSquare(qTo, true);
	dropPiece();
	moveToHome();
}

void updateInfo(){
	  int bat_volt	= batteryGet();
	  uint8_t status=0;
	  data_trans_master[0] = (uint8_t)(data_AS5600_M1 & 0xFF);
	  data_trans_master[1] = (uint8_t)((data_AS5600_M1>>8) & 0xFF);
	  data_trans_master[2] = (uint8_t)(data_AS5600_M2 & 0xFF);
	  data_trans_master[3] = (uint8_t)((data_AS5600_M2>>8) & 0xFF);
	  data_trans_master[4] = (uint8_t)(bat_volt & 0xFF);
	  data_trans_master[5] = (uint8_t)((bat_volt>>8) & 0xFF);
	  if(HAL_SENSOR_UP_GET) status|=(1<<0); else status &=~(1<<0);																// Hal_sensor_up
	  if(HAL_SENSOR_DOWN_GET) status |= (1<<1); else status &=~(1<<1);															// Hal_sensor_down
	  if(HAL_GPIO_ReadPin(BATTERY_CHANGER_GPIO_Port, BATTERY_CHANGER_Pin)==GPIO_PIN_RESET) status |=(1<<2); else status &=~(1<<2);  	// Battery Chnager
	  if(moveIsFinish == true) status |=(1<<3); else status &=~(1<<3);
	  data_trans_master[6] = status;
}

void updateSquarePosition(uint16_t squa){
	EE_WriteVariable(ADDR_EE_POSJ1 + squa, data_AS5600_M1);
	EE_WriteVariable(ADDR_EE_POSJ2 + squa, data_AS5600_M2);
	printf("Saved square %d to EEPROM\r\n",squa);
}
void readSquarePosition(){
#ifdef MDEBUG
	printf("reading...\r\n");
#endif
	uint16_t posj1,posj2;
	for(int i=0;i<80;i++){
		EE_ReadVariable(ADDR_EE_POSJ1 + i, &posj1);
		EE_ReadVariable(ADDR_EE_POSJ2 + i, &posj2);
		square[i].j1 = posj1;
		square[i].j2 = posj2;
	}
#ifdef MDEBUG
	for(int x=0;x<10;x++){
		for(int y=0;y<8;y++){
			printf("{%d,%d},",square[x*8 +y].j1,square[x*8 +y].j2);
		}
		printf("\r\n");
	}
	printf("read finish\r\n");
#endif
}
void testDebug(){
	uint8_t _qfrom,_qto;
	_qfrom = 63;
	_qto = 0;
	for(int i=0;i<32;i++){
		movePiece(_qfrom,_qto,0);
		osDelay(500);
		_qfrom--;
		movePiece(_qto, _qfrom, 0);
		_qto++;
		printf("\aMoveCount:%d\r\n",i);
		osDelay(1000);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
