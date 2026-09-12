#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"

#define MotorState_TASK_PRIO		4     //Task priority //任务优先级
#define MotorState_STK_SIZE 		128   //Task stack size //任务堆栈大小

#define EN     PDin(0)  

//割草电机引脚
#define mow_dir   PCout(5)
#define mow_motor TIM14 -> CCR1

void Enable_Pin(void);
void Get_MotorState_task(void *pvParameters);
void Mow_motor_Init(void);

extern u8 send_error_app;
extern u8 get_error_flag;
#endif
