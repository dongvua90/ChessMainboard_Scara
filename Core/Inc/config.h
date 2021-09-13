

 //SERVO: 1000->2000
#define SERVO_PICK TIM3->CCR4=1600
#define SERVO_DROP TIM3->CCR4=2000

#define J1_POS_MAX 2750
#define J1_POS_MIN 800
#define J2_POS_MAX 2400
#define J2_POS_MIN 500
#define J3_POS_MAX 2000
#define J3_POS_MIN 0

#define AS5600_READ_FAST TIM7->ARR =200; TIM7->CNT=0;  // Frequency: 5Khz
#define AS5600_READ_SLOW //TIM7->ARR =2000; TIM7->CNT=0; // Frequency: 500Hz
