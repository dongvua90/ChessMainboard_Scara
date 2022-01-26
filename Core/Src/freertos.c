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
#define J1_SPEED		12000//10000
#define J1_ACCEL_MAX	500000
#define j1_ACCEL_MIN	100000
#define J2_SPEED		40000//20000
#define J2_ACCEL_MAX	120000//120000
#define J2_ACCEL_MIN	90000
#define J3_SPEED		13000
#define J3_ACCEL_MAX	200000//70000
#define J3_ACCEL_MIN	5000
#define J3_LIMIT_DOWN		0
#define J3_LIMIT_UP	 	-2400 ///-3350

#define MOVE_PIECE				1
#define MOVE_HOME				2
#define MOVE_KILL				3
#define MOVE_BCASLLINGG_QUEEN  	4
#define MOVE_BCASLLING_KING    	5
#define MOVE_PASSANT			6
#define MOVE_WCASLLINGG_QUEEN  	7
#define MOVE_WCASLLING_KING    	8



// User Debug
#define MDEBUG
//#define USERGETPOS // need disable TaskMotor


#define CMD_INFO		'I'
#define CMD_GRIPPER 	'G'
#define CMD_PICK		'p'
#define CMD_PICKPIECE	'P'
#define CMD_MOVE		'm'
#define CMD_MOVEPIECE	'M'
#define CMD_WRITEPOS	'W'
#define CMD_READPOS		'R'
#define CMD_INFOBAT		'i'
#define CMD_MOVETEST	'T'
#define CMD_RESET_SQUARE_KILL 'X'

#define CMD_GETPOS 		'S'
#define CMD_GETPOS_BACK 'b'
#define CMD_GETPOS_NEXT 'N'

#define ADDR_EE_POSJ1 0x5555
#define ADDR_EE_POSJ2 0x7777

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t uart2_main_buf[];
extern bool uart2_onData;
extern uint8_t uart2_data_length;
extern uint8_t uart1_main_buf[];
extern bool uart1_onData;
extern uint8_t uart1_data_length;
struct Point{
	uint16_t j1,j2;
};

struct Point square1,square2;
struct Point square[80];
uint8_t square_kill_number=0;		// Variable save Square kiled
uint16_t num_squa=0;

//test
uint32_t i2c_code_error;
uint8_t kaka=0;

void moveToSquare(uint8_t point,bool continues);		// Point: square in board , continues: tiep tuc di chuyen ngay sau do hay khong?
void movePiece(uint8_t qFrom,uint8_t qTo,uint8_t option);	// qFrom: square from is move , qTo: square to is move , option:MOVE_PIECE,MOVE_KILL,MOVE_CALLING
void moveToHome();										// move to home and off motor
void moveToKill();
void updateInfo();
void updateSquarePosition(uint16_t squa);
void readSquarePosition();

void pickAndDropInit();
void pickupPiece();
void dropPiece();
int j3MoveUp(bool keep_enable);
int j3MoveDown();

bool user_calibase =false;
int battery_save=0;
char sbattery[9];
char scharging[7];
char sinfo[15]; // "-I1234 4321 0 1" - J1+J2+Hallup+halldown
char sinfobat[10];  // "-i1234 1"  - bat + charger
bool charging = false;
//enum {J3_UP,J3_DOWN,J3_NONE}J3_state;

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
uint16_t accel_j1_tik=0,accel_j2_tik=0;			// Variable of Change Accel
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
		if(accel_j1_tik==500)								// Changer Accel
		{
			accel_j1_tik=0;
			long distance = distanceToGo(&motor_j1_data);
			if(labs(distance) < 150){
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
			if(labs(distance) < 150){
				setAcceleration(&motor_j2_data, J2_ACCEL_MIN);
			}else{
				setAcceleration(&motor_j2_data, J2_ACCEL_MAX);
			}
		}
	}
	if(htim->Instance==TIM5)								// Stepper J3
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
		 i2c_code_error = hi2c->ErrorCode;
		 printf("i2c-2 error:%d\r\n",(int)i2c_code_error);
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
	for(int k=0;k<6;k++){				// nháy LED 3 lần khi khởi động
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(250);
	}
#ifdef MDEBUG
	printf("Robochess 2021\r\n");		// in thông báo khởi tạo qua UART2
#endif
	readSquarePosition();				// đọc vị trí Squares từ EEPROM-SIMULATOR
	AS5600_Start_Update();				// Start Tim10 & get data of AS5600
	batteryVoltInit();
	pickAndDropInit();
  /* Infinite loop */
  for(;;)
  {
	  updateInfo(); // edit -> tách biệt các nhiệm vụ, chỉ cần đọc các thông tin khi có yêu cầu

#ifdef MDEBUGk
	  if(uart1_onData){
		  uart1_onData=false; //realease
		  printf("Rev %dBYTE: %s\r\n",uart1_data_length,uart1_main_buf);
		  // Gripper
		  if(uart1_main_buf[0]==CMD_GRIPPER){
			  char sval[1];
			  sval[0]=uart1_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  SERVO_DROP;
				  printf("Gripper open\r\n");
			  }else if(val==1){
				  SERVO_PICKUP;
				  printf("Gripper close\r\n");
			  }
		// Info
		  }else if(uart1_main_buf[0]==CMD_INFO){
			  snprintf(sinfo,sizeof(sinfo),"-I%04d%04d%d%d\r\n",data_AS5600_M1,data_AS5600_M2,HAL_SENSOR_UP_GET,HAL_SENSOR_DOWN_GET);
			  HAL_UART_Transmit(&huart1,(uint8_t*)sinfo,14,1000);
		 	  printf("AS5600_J1:%d_J2:%d Hal_Up:%d_Down:%d Bat:%2d Status:%d\r\n",
		 			  data_AS5600_M1,data_AS5600_M2,HAL_SENSOR_UP_GET,HAL_SENSOR_DOWN_GET,
					  batteryGet(),data_trans_master[6]);
		 	  printf("-B%04d\r\n",batteryGet());
		 	  printf("-b%d\r\n",HAL_GPIO_ReadPin(BATTERY_CHANGER_GPIO_Port, BATTERY_CHANGER_Pin));

		// info battery
		  }else if(uart1_main_buf[0]==CMD_INFOBAT){
			  snprintf(sinfobat,sizeof(sinfobat),"-i%04d%d\r\n",batteryGet(),HAL_GPIO_ReadPin(BATTERY_CHANGER_GPIO_Port, BATTERY_CHANGER_Pin));
			  HAL_UART_Transmit(&huart1,(uint8_t*)sinfobat,9,1000);
		// PICK
		  }else if(uart1_main_buf[0]==CMD_PICK){
			  char sval[1];
			  sval[0]=uart1_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  printf("J3 MOVE UP\r\n");
				  j3MoveUp(false);
			  }else if(val==1){
				  printf("J3 MOVE DOWN\r\n");
				  j3MoveDown();
			  }
		// PICK PIECE
		  }else if(uart1_main_buf[0]==CMD_PICKPIECE){
			  char sval[1];
			  sval[0]=uart1_main_buf[1];
			  int val=atoi(sval);
			  if(val==0){
				  printf("DROP PIECE\r\n");
				  dropPiece();
			  }else if(val==1){
				  printf("PICKUP PIECE\r\n");
				  pickupPiece();
			  }
		// MOVE
		  }else if(uart1_main_buf[0]==CMD_MOVE){
			  char sval[2];
			  if(uart1_data_length>2){
				  sval[0]=uart1_main_buf[1];
				  sval[1]=uart1_main_buf[2];
			  }else{
				  sval[0]='0';
				  sval[1]=uart1_main_buf[1];
			  }
			  int val=atoi(sval);
			  if(val>=0 && val<80){
				  printf("MOVE TO SQUARE:%d\r\n",val);
				  moveToSquare(val, false);
			  }else{
				  printf("ERROR! Cannot move to square %d\r\n",val);
			  }
		// MOVE PIECE
		  }else if(uart1_main_buf[0]==CMD_MOVEPIECE){
			  HAL_UART_Transmit(&huart1,(uint8_t*)"-Y\r\n",4,1000); // reply da nhan duoc lenh ok
			  char sFrom[2],sTo[2],sOption[1];
			  sFrom[0]=uart1_main_buf[1];
			  sFrom[1]=uart1_main_buf[2];
			  sTo[0]=uart1_main_buf[3];
			  sTo[1]=uart1_main_buf[4];
			  sOption[0]=uart1_main_buf[5];
			  int _from=atoi(sFrom);
			  int _to =atoi(sTo);
			  int _option =atoi(sOption);
			  if((_from>=0 && _from<80) && (_to>=0 && _to<80)){
				  printf("MOVEPIECE %d->%d Option:%d\r\n",_from,_to,_option);
				  movePiece(_from, _to, _option);
			  }else{
				  printf("ERROR! Out Range of square\r\n");
			  }
		  }else if(uart1_main_buf[0]=='w'){
			  char sdat[3];
			  sdat[0]=uart1_main_buf[1];
			  sdat[1]=uart1_main_buf[2];
			  uint16_t squa = atoi(sdat);
			  num_squa = squa;
			  if(num_squa>=80) return;
			  updateSquarePosition(num_squa);
			  printf("set square to %d\r\n",num_squa);
		  }else if(uart1_main_buf[0]=='r'){
			  readSquarePosition();
		  }
		  else if(uart1_main_buf[0]=='W'){
			  if(num_squa>=80) return;
		  	updateSquarePosition(num_squa);
		  	num_squa++;
		  	printf("next square:%d\r\n",num_squa);
		  }
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  }
#endif
	  osDelay(50);
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
/*	SoC --i2c -->MAINBOARD (write)
 *	[CMD]							[DATA0]					[DATA1]			[DATA2]
 *	1-GRIPPER					(O=open / 1=close)			(none)			(none)
 *	2-J3_UP_DOWN				(0=down / 1=up)				(none)			(none)
 *	3-PICK_&_DROP PIECE			(0=drop / 1=pick)			(none)			(none)
 * 	4-MOVE_TO_SQUARE			(square:0-79 (79=home))		(none)			(none)
 * 	5-MOVE_PIECE				(square from)				(square to)		(option)
 * 	6-WRITE_POS_TO_EEPROM		(square)					(none)			(none)
 * 	*/
/*	SoC <--i2c --MAINBOARD (read)
 * 	[DATA0]		[DATA1]		[DATA2]		[DATA3]		[DATA4]		[DATA5]		[DATA6]
 * 	[	 AS5600_J1	  ]		[	AS5600_J2	  ]		[	  BATTERY	  ]		[STATUS]
 * 	[STATUS] = [HAL_UP] [HAL_DOWN] [CHARGING] [MOVE_FINISH]
 * */
#define TEST_DISABLE_STEPPER
/* USER CODE END Header_StartTaskMove */
__weak void StartTaskMove(void const * argument)
{
  /* USER CODE BEGIN StartTaskMove */
	HAL_I2C_EnableListen_IT(&hi2c2);	// khởi tạo giao tiếp I2C với SoC (Address = 0x23)
  /* Infinite loop */
	for (;;) {
		osSemaphoreWait(binarySem_masterCmdHandle, osWaitForever);

		if (data_rev_master[0] == CMD_GRIPPER){
			  if(data_rev_master[1]==0){
				  SERVO_DROP;
				  printf("Gripper open\r\n");
			  }else if(data_rev_master[1]==1){
				  SERVO_PICKUP;
				  printf("Gripper close\r\n");
			  }
		}else if (data_rev_master[0] == CMD_PICK){
			if(data_rev_master[1]==0){
				printf("J3 MOVE DOWN\r\n");
//#ifndef TEST_DISABLE_STEPPER
				j3MoveDown();
//#endif
			}else if(data_rev_master[1]==1){
				printf("J3 MOVE UP\r\n");
//#ifndef TEST_DISABLE_STEPPER
				j3MoveUp(false);
//#endif
			}
		}else if (data_rev_master[0] == CMD_PICKPIECE){
			if(data_rev_master[1]==0){
				printf("DROP PIECE\r\n");
//#ifndef TEST_DISABLE_STEPPER
				dropPiece();
//#endif
			}else if(data_rev_master[1]==1){
				printf("PICKUP PIECE\r\n");
//#ifndef TEST_DISABLE_STEPPER
				pickupPiece();
//#endif
			}
		}else if (data_rev_master[0] == CMD_MOVE){
			if(data_rev_master[1]>=0 && data_rev_master[1]<80){
				printf("MOVE TO SQUARE:%d\r\n",data_rev_master[1]);
//#ifndef TEST_DISABLE_STEPPER
				moveToSquare(data_rev_master[1], false);
//#endif
			}else{
				printf("ERROR! Cannot move to square %d\r\n",data_rev_master[1]);
			}
		}else if (data_rev_master[0] == CMD_MOVEPIECE){
			moveIsFinish = false;
			if((data_rev_master[1]>=0 && data_rev_master[1]<80) && (data_rev_master[2]>=0 && data_rev_master[2]<80)){
				printf("MOVEPIECE %d->%d Option:%d\r\n",data_rev_master[1],data_rev_master[2],data_rev_master[3]);
//#ifndef TEST_DISABLE_STEPPER
				movePiece(data_rev_master[1], data_rev_master[2], data_rev_master[3]);
//#endif
				moveIsFinish = true;
			}else{
				printf("ERROR! Out Range of square\r\n");
			}
		}else if (data_rev_master[0] == CMD_WRITEPOS){
			printf("Write Pos %d To EEPROM\r\n",data_rev_master[1]);
			updateSquarePosition(data_rev_master[1]);
		}else if (data_rev_master[0] == CMD_READPOS){
			printf("Read Pos from EEPROM\r\n");
			readSquarePosition();
		}else if(data_rev_master[0] == CMD_RESET_SQUARE_KILL){
			square_kill_number = 0;
			printf("Reset Square Kill\r\n");
		}
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
//	osDelay(2000);						// Wait for finish Init
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
//		pickAndDropInit();
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
	if(option == MOVE_PIECE){
		moveToSquare(qFrom, true);
		pickupPiece();
		moveToSquare(qTo, true);
		dropPiece();
		moveToHome();
	}else if(option == MOVE_HOME){
		moveToHome();
	}else if(option == MOVE_KILL)										// Neu la nuoc di an quan thi gap piece ra khoi ban co
	{
		moveToSquare(qTo, true);		// move to piece kill
		pickupPiece();					// pickup
		moveToKill();		// move to square die
		dropPiece();					// drop
		moveToSquare(qFrom, true);
		pickupPiece();
		moveToSquare(qTo, true);
		dropPiece();
		moveToHome();
	}else if(option == MOVE_BCASLLING_KING)
	{
		moveToSquare(60, true);
		pickupPiece();
		moveToSquare(62, true);
		dropPiece();
		moveToSquare(63, true);
		pickupPiece();
		moveToSquare(61, true);
		dropPiece();
		moveToHome();
	}else if(option == MOVE_BCASLLINGG_QUEEN)
	{
		moveToSquare(60, true);
		pickupPiece();
		moveToSquare(58, true);
		dropPiece();
		moveToSquare(56, true);
		pickupPiece();
		moveToSquare(59, true);
		dropPiece();
		moveToHome();
	}
	else if(option == MOVE_WCASLLING_KING)
		{
			moveToSquare(59, true);
			pickupPiece();
			moveToSquare(57, true);
			dropPiece();
			moveToSquare(56, true);
			pickupPiece();
			moveToSquare(58, true);
			dropPiece();
			moveToHome();
		}else if(option == MOVE_WCASLLINGG_QUEEN)
		{
			moveToSquare(59, true);
			pickupPiece();
			moveToSquare(61, true);
			dropPiece();
			moveToSquare(63, true);
			pickupPiece();
			moveToSquare(60, true);
			dropPiece();
			moveToHome();
		}else if(option == MOVE_PASSANT){
		moveToSquare(qTo+8, true);
		pickupPiece();
		moveToKill();
		dropPiece();
		moveToSquare(qFrom, true);
		pickupPiece();
		moveToSquare(qTo, true);
		dropPiece();
		moveToHome();
	}
	enableStepper(&motor_j3_data, OFF);
//	printf("-f\r\n"); //send to Soc ,moved finish
	HAL_UART_Transmit(&huart1,(uint8_t *)"-f\r\n",4,1000);
}

void updateInfo(){
	  int bat_volt	= batteryGet();
	  if(battery_save != bat_volt){
		  battery_save = bat_volt;
		  printf("-B%04d\r\n",battery_save);
		  snprintf(sbattery,sizeof(sbattery),"-B%04d\r\n",battery_save);
		  HAL_UART_Transmit(&huart1,(uint8_t*)sbattery,8,1000);
	  }
	  char getcharging = HAL_GPIO_ReadPin(BATTERY_CHANGER_GPIO_Port, BATTERY_CHANGER_Pin);
	  if(getcharging != charging){
		  charging = getcharging;
		  printf("-b%d\r\n",charging);
		  snprintf(scharging,sizeof(scharging),"-b%d\r\n",charging);
		  HAL_UART_Transmit(&huart1,(uint8_t*)scharging,5,1000);
	  }
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
	if(EE_WriteVariable(ADDR_EE_POSJ1 + squa, data_AS5600_M1) != EE_OK){
		printf("Error to Save %d to EEPROM\r\n",squa);
		return;
	}
	if(EE_WriteVariable(ADDR_EE_POSJ2 + squa, data_AS5600_M2) != EE_OK){
		printf("Error to Save %d to EEPROM\r\n",squa);
		return;
	}
	printf("Saved  square  %d to EEPROM\r\n",squa);
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
#ifdef MDEBUG2
for(int x=0;x<10;x++){
	for(int y=0;y<8;y++){
		printf("{%d,%d},",square[x*8 +y].j1,square[x*8 +y].j2);
	}
	printf("\r\n");
}
#endif
#ifdef MDEBUG
	printf("read finish\r\n");
#endif

}
void pickAndDropInit(){
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Start PWM for Servo
	SERVO_DROP;
	if(HAL_SENSOR_UP_GET==GPIO_PIN_SET){		// neu ko J3 ko phai o vi tri up
		enableStepper(&motor_j3_data, ON);
		moveTo(&motor_j3_data, J3_LIMIT_UP);
		run(&motor_j3_data);
		osDelay(30);
		while( isRunning(&motor_j3_data) && HAL_SENSOR_UP_GET == GPIO_PIN_SET ){	// Waiting for move finish
			osDelay(10);
		}
		enableStepper(&motor_j3_data, OFF);
	}
	motor_j3_data._currentPos = J3_LIMIT_UP;
	motor_j3_data._targetPos = J3_LIMIT_UP;

}
void pickupPiece(){
	SERVO_DROP;
	if(j3MoveDown()==-1){
		j3MoveUp(true);
		SERVO_EXTERN;
		osDelay(200);
		j3MoveDown();
	}
	SERVO_PICKUP;
	osDelay(300);
	j3MoveUp(true);
}
void dropPiece(){
	j3MoveDown();
	SERVO_DROP;
	osDelay(300);
	j3MoveUp(true);
}
int j3MoveUp(bool keep_enable){
	setSpeed(&motor_j3_data, J3_SPEED);
	enableStepper(&motor_j3_data, ON);
	setAcceleration(&motor_j3_data, J3_ACCEL_MAX);
	moveTo(&motor_j3_data, J3_LIMIT_UP);
	run(&motor_j3_data);
	osDelay(300);
	while( isRunning(&motor_j3_data)>0){	// Waiting for move finish
		osDelay(10);
	}
	enableStepper(&motor_j3_data, keep_enable);
	if(HAL_SENSOR_UP_GET == GPIO_PIN_RESET)
		return 0;
	else
		return -1;
}
int j3MoveDown(){
	setSpeed(&motor_j3_data, J3_SPEED);
	enableStepper(&motor_j3_data, ON);
	setAcceleration(&motor_j3_data, J3_ACCEL_MAX);
	moveTo(&motor_j3_data, J3_LIMIT_DOWN);
	motor_j3_data.isComplete=false;
	run(&motor_j3_data);
	osDelay(200);
	while( isRunning(&motor_j3_data)>0){	// Waiting for move finish
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
