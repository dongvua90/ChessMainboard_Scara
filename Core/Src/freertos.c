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
#include <math.h>
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
#define J3_SPEED		13000
#define J3_ACCEL_MAX	100000//200000
#define J3_ACCEL_MIN	50000
#define J3_LIMIT_DOWN		0
#define J3_LIMIT_UP	 	-3750 ///-3350

#define CMD_I2C_MOVE_PIECE				1
#define CMD_I2C_MOVE_HOME				2
#define CMD_I2C_MOVE_KILL				2
#define CMD_I2C_MOVE_CASLLINGG_QUEEN  	3
#define CMD_I2C_MOVE_CASLLING_KING    	4
#define CMD_I2C_MOVE_PASSANT			5


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
#define CMD_MOVETEST	'A'
#define CMD_MOVETESTJ2	'B'
#define CMD_KINETIC		'K'

#define CMD_GETPOS 		'S'
#define CMD_GETPOS_BACK 'b'
#define CMD_GETPOS_NEXT 'N'

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
const struct Point square[80]={
		{2096,2841},{1764,2915},{1609,2906},{1577,2837},{1598,2742},{1637,2640},{1680,2531},{1739,2415},	//1
		{2320,2674},{2121,2708},{1976,2704},{1884,2668},{1844,2606},{1842,2526},{1862,2432},{1897,2324},	//2
		{2453,2532},{2324,2557},{2206,2554},{2117,2527},{2068,2477},{2050,2408},{2050,2322},{2070,2218},	//3
		{2563,2388},{2462,2410},{2372,2410},{2300,2389},{2252,2343},{2223,2278},{2219,2195},{2233,2093},	//4
		{2663,2241},{2571,2259},{2497,2261},{2438,2240},{2396,2197},{2374,2133},{2368,2053},{2378,1950},	//5
		{2768,2077},{2679,2098},{2612,2099},{2562,2077},{2526,2033},{2507,1971},{2499,1894},{2510,1802},	//6
		{2883,1900},{2800,1916},{2735,1916},{2684,1905},{2656,1856},{2636,1804},{2637,1724},{2658,1625},	//7
		{3015,1715},{2941,1730},{2877,1728},{2825,1712},{2801,1666},{2787,1612},{2808,1516},{2870,1367},	//8
	//		a			b			c			d			e			f			g			h
		{1854,2178},{2004,2089},{2156,1992},{2302,1872},{2434,1740},{1946,2028},{2089,1937},{2234,1833}, // Square Kill-1
		{2360,1731},{2508,1578},{2062,1852},{2176,1791},{2302,1698},{2439,1577},{2598,1391},{2389,3053}  // Square Kill-2
																	         // Home ------------^
};

uint8_t square_kill_number=0;		// Variable save Square kiled

void moveToSquare(uint8_t point,bool continues);		// Point: square in board , continues: tiep tuc di chuyen ngay sau do hay khong?
void movePiece(uint8_t qFrom,uint8_t qTo,uint8_t option);	// qFrom: square from is move , qTo: square to is move , option:MOVE_PIECE,MOVE_KILL,MOVE_CALLING
void moveToHome();										// move to home and off motor
void moveToKill();
void updateInfo();

void pickAndDropInit();
void pickupPiece();
void dropPiece();
int j3MoveUp();
int j3MoveDown();

#define PI 3.14159f;
#define sq(x) ((x)*(x))
uint16_t J1ORIGIN = 2389;
uint16_t J2ORIGIN =	3053;
float theta1,theta2;
const float L1=180.0f;
const float L2=314.0f;
bool user_calibase =false;
bool runfirts = false;
void inverseKinematics(float x, float y);
void calibase();
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
struct AccelStepperData motor_j1_data,motor_j2_data,motor_j3_data;// Variable of Stepper
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
osThreadId motorJ3TaskHandle;
osSemaphoreId binarySem_motorJ1Handle;
osSemaphoreId binarySem_motorJ2Handle;
osSemaphoreId binarySem_masterCmdHandle;
osSemaphoreId binarySem_motorJ3Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// for Motor J1 J2
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)								// Stepper J1
	{
		if(user_calibase==false) motor_j1_data._currentPos = data_AS5600_M1;			// Set Current Position
		osSemaphoreRelease(binarySem_motorJ1Handle);		// Release Semaphore for Calculator Stepper (run)
		if(user_calibase) return;
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
		if(user_calibase == false) motor_j2_data._currentPos = data_AS5600_M2;		// Set Current Position
		osSemaphoreRelease(binarySem_motorJ2Handle);	// Release Semaphore for Calculator Stepper (run)
		if(user_calibase) return;
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
	if(htim->Instance==TIM5)								// Stepper J2
	{
		osSemaphoreRelease(binarySem_motorJ3Handle);	// Release Semaphore for Calculator Stepper (run)
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
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskMotorJ1(void const * argument);
void StartTaskMotorJ2(void const * argument);
void StartTaskMove(void const * argument);
void StartTaskMotorJ3(void const * argument);

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

  /* definition and creation of binarySem_motorJ3 */
  osSemaphoreDef(binarySem_motorJ3);
  binarySem_motorJ3Handle = osSemaphoreCreate(osSemaphore(binarySem_motorJ3), 1);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorJ1Task */
  osThreadDef(motorJ1Task, StartTaskMotorJ1, osPriorityHigh, 0, 2048);
  motorJ1TaskHandle = osThreadCreate(osThread(motorJ1Task), NULL);

  /* definition and creation of motorJ2Task */
  osThreadDef(motorJ2Task, StartTaskMotorJ2, osPriorityHigh, 0, 2048);
  motorJ2TaskHandle = osThreadCreate(osThread(motorJ2Task), NULL);

  /* definition and creation of moveTask */
  osThreadDef(moveTask, StartTaskMove, osPriorityRealtime, 0, 1024);
  moveTaskHandle = osThreadCreate(osThread(moveTask), NULL);

  /* definition and creation of motorJ3Task */
  osThreadDef(motorJ3Task, StartTaskMotorJ3, osPriorityHigh, 0, 1024);
  motorJ3TaskHandle = osThreadCreate(osThread(motorJ3Task), NULL);

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
	HAL_I2C_EnableListen_IT(&hi2c2);	// I2C2 for interface
	AS5600_Start_Update();				// Start Tim10 & get data of AS5600
	batteryVoltInit();
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Start PWM for Servo
//	SERVO_DROP;
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
		 	 	enableStepper(&motor_j1_data, OFF);
		 	 	enableStepper(&motor_j2_data, OFF);
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
		  }else if(uart2_main_buf[0]==CMD_MOVETEST){
			  user_calibase = true;
			  if(runfirts==false){
				  runfirts = true;
				  motor_j1_data._currentPos =0;
				  motor_j1_data._targetPos =0;
			  }

			  printf("MOVE:AS5600:%d\r\n",data_AS5600_M1);
			  char sposJ1[4];
			  for(int i=0;i<4;i++){
				  sposJ1[i] = uart2_main_buf[i+1];
			  }
			  int posJ1 = atoi(sposJ1);
			  enableStepper(&motor_j1_data, ON);
			  moveTo(&motor_j1_data,posJ1);
			  run(&motor_j1_data);
				while( isRunning(&motor_j1_data)){	// Waiting for move finish
					osDelay(10);
				}
				osDelay(300);
				printf("Finish:AS5600:%d\r\n",data_AS5600_M1);
				motor_j1_data.isStop = true;
//				enableStepper(&motor_j1_data, OFF);
		  }else if(uart2_main_buf[0]==CMD_MOVETESTJ2){
			  enableStepper(&motor_j1_data, ON);
			  printf("MOVE:AS56002:%d\r\n",data_AS5600_M2);
			  char sposJ2[4];
			  for(int i=0;i<4;i++){
				  sposJ2[i] = uart2_main_buf[i+1];
			  }
			  int posJ2 = atoi(sposJ2);
			  enableStepper(&motor_j2_data, ON);
			  moveTo(&motor_j2_data,posJ2);
			  run(&motor_j2_data);
				while( isRunning(&motor_j2_data)){	// Waiting for move finish
					osDelay(10);
				}
				osDelay(300);
				printf("Finish:AS56002:%d\r\n",data_AS5600_M2);
				motor_j2_data.isStop = true;
//				enableStepper(&motor_j2_data, OFF);
		  }else if(uart2_main_buf[0]==CMD_KINETIC){
			  int dx,dy;
			  char sdx[5],sdy[5];
			  for(int i=0;i<4;i++){
				  sdx[i]=uart2_main_buf[i+1];
				  sdy[i] = uart2_main_buf[i+6];
			  }
			  dx = atoi(sdx);
			  dy = atoi(sdy);
			  inverseKinematics(dx, dy);
		  }else if(uart2_main_buf[0]=='f'){
//			  calibase();
			  char dat[5];
			  for(int i=0;i<4;i++){
				  dat[i] = uart2_main_buf[i+1];
			  }
			  int mdat = atoi(dat);
			  EE_WriteVariable(0x1111, mdat);
			  printf("EE write:%d\r\n",mdat);
		  }else if(uart2_main_buf[0]=='F'){
			  uint16_t dat=0;
			  EE_ReadVariable(0x1111, &dat);
			  printf("EE read:%d\r\n",dat);
		  }else if(uart2_main_buf[0]=='J'){
			  printf("Pickup\r\n");
			  pickupPiece();
//			  enableStepper(&motor_j3_data, ON);
//			  moveTo(&motor_j3_data, mdat);
//			  run(&motor_j3_data);
//			  while( isRunning(&motor_j3_data) ){	// Waiting for move finish
//					osDelay(10);
//			 }
//			 motor_j3_data.isStop = true;
//			 enableStepper(&motor_j3_data, OFF);
		  }else if(uart2_main_buf[0]=='j'){
			  printf("pickDown\r\n");
			  dropPiece();
		  }else if(uart2_main_buf[0]=='y'){
			  printf("Test do ben:\r\n");
			  for(int i=0;i<1000;i++){
				  pickupPiece();
				  osDelay(500);
				  dropPiece();
				  osDelay(500);
				  printf("picked: %d\r\n",i);
			  }
			  printf("finish ok\r\n");
//			  char dat[5];
//			 for(int i=0;i<4;i++){
//			 	dat[i] = uart2_main_buf[i+1];
//			 }
//			 int mdat = atoi(dat);
//			 TIM4->CCR3=mdat;
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
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
	printf("Robochess 2021 -taskJ1\r\n");
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
		} else if (data_rev_master[0] == CMD_I2C_MOVE_HOME) {
#ifdef MDEBUG
			printf("I2C-MOVEHOME\r\n");
#endif
			moveToHome();
			moveIsFinish = true;
		}
		osDelay(10);
	}
  /* USER CODE END StartTaskMove */
}

/* USER CODE BEGIN Header_StartTaskMotorJ3 */
/**
* @brief Function implementing the motorJ3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotorJ3 */
__weak void StartTaskMotorJ3(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotorJ3 */
	osDelay(2000);						// Wait for finish Init
	motor_j3_data.GPIO_PIN_Dir		= J3_DIR_Pin;
	motor_j3_data.GPIO_PORT_Dir		= J3_DIR_GPIO_Port;
	motor_j3_data.GPIO_PORT_Enable	= J3_EN_GPIO_Port;
	motor_j3_data.GPIO_PIN_Enable	= J3_EN_Pin;
	motor_j3_data.USER_TIMER		= TIM5;
	motor_j3_data.TIM_CHANEL		= TIM_CHANNEL_1;
	motor_j3_data.isStop			= false;
	AccelStepper_init(&motor_j3_data, htim5, 0, J3_SPEED, J3_ACCEL_MAX);
	#ifdef USERGETPOS
		enableStepper(&motor_j3_data, OFF);
		while(1){			// disable this task
			osDelay(1000);
		}
	#endif
		pickAndDropInit();
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreWait(binarySem_motorJ3Handle, osWaitForever);
	  run(&motor_j3_data);
  }
  /* USER CODE END StartTaskMotorJ3 */
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
// INVERSE KINEMATICS
void inverseKinematics(float x, float y) {
	  printf("x:%f  ",x);
	  printf("y:%f\r\n",y);
  theta2 = acos((sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
 if (x < 0 && y < 0) {
    theta2 = (-1) * theta2;
 }

  theta1 = atan(y/x) + atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));

  theta2 = theta2 * 180 / PI;
  theta1 = theta1 * 180 / PI;
  if(theta1<0) theta1 =180 - (-1)*theta1;

//  printf("theta1:%f\r\n",theta1);
//  printf("theta2:%f\r\n",theta2);

  theta1 = 90 - theta1;
  theta2 = 180 - theta2;
//  printf("Q1:%f\r\n",theta1);
//  printf("Q2:%f\r\n",theta2);
  long int pul1,pul2;
  pul1 = J1ORIGIN + theta1*11.377;
  pul2 = J2ORIGIN	-theta2*11.377;
//  pul1 = round(pul1);
//  pul2 = round(pul2);
  printf("P1:%ld\r\n",pul1);
  printf("P2:%ld\r\n",pul2);

	AS5600_Start_Update_High();
	enableStepper(&motor_j1_data, ON);
	enableStepper(&motor_j2_data, ON);
	moveTo(&motor_j1_data,pul1);
	run(&motor_j1_data);
	moveTo(&motor_j2_data,pul2);
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
	enableStepper(&motor_j1_data, OFF);
	enableStepper(&motor_j2_data, OFF);
	AS5600_Start_Update_Low();
}

void calibase(){
	user_calibase =true;
	int pos = 0;
	float fpos = 0;

	int change=0;
	AccelStepper_init(&motor_j1_data, htim3, 0, J1_SPEED, 10000);
	enableStepper(&motor_j1_data, ON);
	run(&motor_j1_data);
	osDelay(3000);
	float fAS5600 = data_AS5600_M1;
	for(int i=0;i<1024;i++){
//		fpos += 174;
//		fAS5600 += 11.378f;
		fpos += 16;
		fAS5600 +=1;
		pos =fpos;
		moveTo(&motor_j1_data, pos);
		  run(&motor_j1_data);
			while( isRunning(&motor_j1_data)){	// Waiting for move finish
				osDelay(10);
			}
			osDelay(500);
		change = data_AS5600_M1 - fAS5600;
		if(i<3) fAS5600 = data_AS5600_M1;
//		printf("%d - %d  %f  %d\r\n",i,data_AS5600_M1,fAS5600, change);
		printf("%d\r\n",change);
	}

}
void pickAndDropInit(){
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Start PWM for Servo
	SERVO_DROP;
	if(HAL_SENSOR_UP_GET==GPIO_PIN_SET){		// neu ko J3 ko phai o vi tri up
		enableStepper(&motor_j3_data, ON);
		moveTo(&motor_j3_data, J3_LIMIT_UP);
		run(&motor_j3_data);
		while( isRunning(&motor_j3_data) ){	// Waiting for move finish
			if(HAL_SENSOR_UP_GET==GPIO_PIN_RESET)
				enableStepper(&motor_j3_data, OFF);
			osDelay(10);
		}
		enableStepper(&motor_j3_data, OFF);
	}else{
		motor_j3_data._currentPos = J3_LIMIT_UP;
		motor_j3_data._targetPos = J3_LIMIT_UP;
	}
}
void pickupPiece(){
	SERVO_DROP;
	if(j3MoveDown()==-1){
		j3MoveUp();
		SERVO_EXTERN;
		osDelay(200);
		j3MoveDown();
	}
	SERVO_PICKUP;
	osDelay(300);
	j3MoveUp();
}
void dropPiece(){
	j3MoveDown();
	SERVO_DROP;
	osDelay(300);
	j3MoveUp();
}
int j3MoveUp(){
	enableStepper(&motor_j3_data, ON);
	moveTo(&motor_j3_data, J3_LIMIT_UP);
	run(&motor_j3_data);
	osDelay(300);
	while( isRunning(&motor_j3_data) && HAL_SENSOR_UP_GET == GPIO_PIN_SET ){	// Waiting for move finish
		osDelay(10);
	}
	enableStepper(&motor_j3_data, OFF);
	if(HAL_SENSOR_UP_GET == GPIO_PIN_RESET)
		return 0;
	else
		return -1;
}
int j3MoveDown(){
	enableStepper(&motor_j3_data, ON);
	moveTo(&motor_j3_data, J3_LIMIT_DOWN);
	run(&motor_j3_data);
	osDelay(300);
	while( isRunning(&motor_j3_data) && HAL_SENSOR_DOWN_GET == GPIO_PIN_SET){	// Waiting for move finish
		osDelay(10);
	}
	enableStepper(&motor_j3_data, OFF);
	if(HAL_SENSOR_DOWN_GET == GPIO_PIN_RESET)
		return 0;
	else
		return -1;
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
