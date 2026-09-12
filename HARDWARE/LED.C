#include "led.h"

int Led_Count=500; //LED flicker time control //LED闪烁时间控制

/**************************************************************************
Function: LED interface initialization
Input   : none
Output  : none
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;//LED对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOD,GPIO_Pin_5);
}
/**************************************************************************
Function: Buzzer interface initialized
Input   : none
Output  : none
函数功能：蜂鸣器接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void Buzzer_Init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin =  Buzzer_PIN;//LED对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIO
	beep=0;
}
/**************************************************************************
Function: LED light flashing task
Input   : none
Output  : none
函数功能：LED灯闪烁任务
入口参数：无 
返回  值：无
**************************************************************************/
void led_task(void *pvParameters)
{
    while(1)
    { 
		LED = !LED;   
		
		//The LED flicker task is very simple, requires low frequency accuracy, and uses the relative delay function	
		//LED闪烁任务非常简单，对频率精度要求低，使用相对延时函数			
		vTaskDelay(Led_Count); 
    }
}  

/**************************************************************************
Function: The LED flashing
Input   : none
Output  : blink time
函数功能：LED闪烁
入口参数：闪烁时间
返 回 值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if(0==time) LED=0;
	  else		if(++temp==time)	LED = !LED,temp=0;
}

//RGB灯带初始化
void RGB_lights_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//使能对应定时器、GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);  
	
	//GPIO初始化 -> PE6 ==> LED_R
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//PB9 ==> LED_G
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//PE5 ==> LED_B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
	//TIM11基础配置
	TIM_TimeBaseStructure.TIM_Period = 254; //驱动灯带使用500Hz PWM信号，避免高频导致芯片发热
	TIM_TimeBaseStructure.TIM_Prescaler =6588; //PSC分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //时钟分频系数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数模式
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure); //写入配置
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //使用PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //禁用互补输出
	TIM_OCInitStructure.TIM_Pulse = 0; //不分频
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //输出极性为高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;//输出极性为高
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);//写入上述配置到TIM13_CH1
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //使用PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //禁用互补输出
	TIM_OCInitStructure.TIM_Pulse = 0; //不分频
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //输出极性为高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;//输出极性为高
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);//写入上述配置到TIM13_CH1
	
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);//使能重装载
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);//使能重装载
	
	TIM_ARRPreloadConfig(TIM9, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM9, ENABLE);  

	
	//TIM14基础配置
	TIM_TimeBaseStructure.TIM_Period = 254; //驱动灯带使用500Hz PWM信号，避免高频导致芯片发热
	TIM_TimeBaseStructure.TIM_Prescaler =6588; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure); 
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM11, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM11, ENABLE); //使能TIMx在ARR上的预装载寄存器
	TIM_Cmd(TIM11, ENABLE);  
	
	LED_R = 0;
	LED_G = 0;
	LED_B = 0;
}

//灯带颜色设置函数
void RGB_Set(u8 r_value,u8 g_value,u8 b_value)
{
	if( r_value>255 ) r_value=255;
	if( r_value<1 )   r_value=0;
	if( g_value>255 ) g_value=255;
	if( g_value<1 )   g_value=0;
	if( b_value>255 ) b_value=255;
	if( b_value<1 )   b_value=0;
	LED_R = r_value;
	LED_G = g_value;
	LED_B = b_value;
}

