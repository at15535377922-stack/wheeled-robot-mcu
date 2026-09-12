#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"

#define LED_TASK_PRIO		3     //Task priority //任务优先级
#define LED_STK_SIZE 		128   //Task stack size //任务堆栈大小


#define LED PDout(5)

/*--------Buzzer control pin--------*/
#define Buzzer_PORT GPIOD
#define Buzzer_PIN GPIO_Pin_4
#define beep PDout(4)
/*----------------------------------*/

/*--------RGB control pin--------*/
#define LED_R TIM9  -> CCR2
#define LED_G TIM11 -> CCR1
#define LED_B TIM9  -> CCR1
/*----------------------------------*/


#if USE_RGB_lights
//RGB灯带模式
enum{
	User_defined,//用户自定义灯光
	Auto_recharge,//自动回充模式灯光
	Natural_mode, //自然状态灯光
	LowPower_mode //低电量模式
};
#endif
 

void LED_Init(void);  
void Buzzer_Init(void); 
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
void RGB_lights_init(void);
void RGB_Set(u8 r_value,u8 g_value,u8 b_value);

#endif
