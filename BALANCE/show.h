#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "oled.h"
#include "system.h"
#define SHOW_TASK_PRIO		3
#define SHOW_STK_SIZE 		2048  

extern u8 Low_PowerMode;
extern u8 show_motor_state; //电机状态

//开启DeBug模式定义的变量
#if OLED_DEBUG_MODE
extern u8 checkauto_data;
extern u8 oled_show_mode;
extern u8 oled_reflash_flag;
extern float pos_x,pos_y,pos_z;

#define OLED_MAX_PAGE 5
#endif


void beep_beep(u8 flag);
extern u8 avoid_flag_beep;
extern u8 beep_flag; 
extern u8 motor_beep;
void show_task(void *pvParameters);
void oled_show(void);
void APP_Show(void);
extern u8 recharge_flag_beep;//自动回充蜂鸣提示
float VolMean_Filter(float data);
extern float base_vol;
void APP_Debug_Show(void);
extern u8 APP_Debug;
#endif
