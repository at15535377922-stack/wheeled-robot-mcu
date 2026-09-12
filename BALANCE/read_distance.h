#ifndef __Read_Distance_H
#define __Read_Distance_H

void ReadUS_task(void *pvParameters);
#include "system.h"

#define US_TASK_PRIO		4     //Task priority //任务优先级
#define US_STK_SIZE 		512   //Task stack size //任务堆栈大小

extern volatile u8 timeout_A,timeout_B,timeout_C,timeout_D,timeout_E,timeout_F;
void Rainbow_RGB(void);
void Monochrome_RGB(void);

extern u8 rgb_lights_showmode;


#endif
