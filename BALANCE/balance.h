#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		4     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小


#if USE_US_Avoid
extern u8 Open_US_avoid;
extern float start_avoid;

#endif

void Balance_task(void *pvParameters);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx, float vz, float step, float step_Vz);
void PS2_control(void);
float float_abs(float insert);
u8 Remote_Choose_mode(void);

extern u8 allow_Recharge_time;
extern u8 allow_recharge_time_on;
extern u8 motor_clear_error;
extern u8 rm_stop_scan;
extern u8 SecurityPLY;
void robot_check(void);
void auto_pwm_clear(void);


#endif  

