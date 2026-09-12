#include "motor.h"

u8 send_error_app=0;
u8 get_error_flag=0;

void Get_MotorState_task(void *pvParameters)
{	
	static u8 countimes;
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
		//The task is run at 10hz
		//此任务以10Hz的频率运行 	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1_HZ));
		
		if(APP_Debug==1)
		{
			APP_Debug_Show();
		}
		
		if(get_error_flag==1)
		{
			countimes++;
			if(countimes==3) //错峰打印，防止与其他数据抢占printf函数
			{
				if(Self_CheckingFlag!=0)
				{
					u8 tip=1;
					send_error_app=1;
					printf("{#当前机器人底盘状态：");
					
					if(Car_Mode==S200)
					{
						if(Get_Checking_FLAG(Drive1_ERROR)==0)//如果驱动器1无报错,则剩下的报错是驱动器2的
						{
							if(Get_Checking_FLAG(Drvie_overVOL)) printf("\r\n%d",tip++),printf(".前驱电机驱动过压");
								
							if(Get_Checking_FLAG(Drvie_underVOL)) printf("\r\n%d",tip++),printf(".前驱电机驱动欠压");
				
							if(Get_Checking_FLAG(Drvie_EEPROM_ERROR)) printf("\r\n%d",tip++),printf(".前驱电机驱动器内部EEPROM读写错误");
								
							if(Get_Checking_FLAG(L_Motor_overCUR)) printf("\r\n%d",tip++),printf(".前驱左电机过流");
								
							if(Get_Checking_FLAG(L_Motor_overLoad)) printf("\r\n%d",tip++),printf(".前驱左电机过载");
								
							if(Get_Checking_FLAG(L_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".前驱左电机电流异常");
								
							if(Get_Checking_FLAG(L_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".前驱左电机编码器数据异常");
								
							if(Get_Checking_FLAG(L_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".前驱左电机速度异常");
								
							if(Get_Checking_FLAG(L_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".前驱左电机内部参考电压出错");
								
							if(Get_Checking_FLAG(L_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".前驱左电机霍尔线未插"); 

							if(Get_Checking_FLAG(R_Motor_overCUR)) printf("\r\n%d",tip++),printf(".前驱右电机过流");
								
							if(Get_Checking_FLAG(R_Motor_overLoad)) printf("\r\n%d",tip++),printf(".前驱右电机过载");
								
							if(Get_Checking_FLAG(R_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".前驱右电机电流异常");
								
							if(Get_Checking_FLAG(R_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".前驱右电机编码器数据异常");
								
							if(Get_Checking_FLAG(R_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".前驱右电机速度异常");
								
							if(Get_Checking_FLAG(R_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".前驱右电机内部参考电压出错");
								
							if(Get_Checking_FLAG(R_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".前驱右电机霍尔线未插"); 

						}
						else //驱动器1有报错
						{
							if(Get_Checking_FLAG(Drvie_overVOL)) printf("\r\n%d",tip++),printf(".后驱电机驱动过压");
								
							if(Get_Checking_FLAG(Drvie_underVOL)) printf("\r\n%d",tip++),printf(".后驱电机驱动欠压");
				
							if(Get_Checking_FLAG(Drvie_EEPROM_ERROR)) printf("\r\n%d",tip++),printf(".后驱电机驱动器内部EEPROM读写错误");
								
							if(Get_Checking_FLAG(L_Motor_overCUR)) printf("\r\n%d",tip++),printf(".后驱左电机过流");
								
							if(Get_Checking_FLAG(L_Motor_overLoad)) printf("\r\n%d",tip++),printf(".后驱左电机过载");
								
							if(Get_Checking_FLAG(L_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".后驱左电机电流异常");
								
							if(Get_Checking_FLAG(L_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".后驱左电机编码器数据异常");
								
							if(Get_Checking_FLAG(L_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".后驱左电机速度异常");
								
							if(Get_Checking_FLAG(L_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".后驱左电机内部参考电压出错");
								
							if(Get_Checking_FLAG(L_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".后驱左电机霍尔线未插"); 

							if(Get_Checking_FLAG(R_Motor_overCUR)) printf("\r\n%d",tip++),printf(".后驱右电机过流");
								
							if(Get_Checking_FLAG(R_Motor_overLoad)) printf("\r\n%d",tip++),printf(".后驱右电机过载");
								
							if(Get_Checking_FLAG(R_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".后驱右电机电流异常");
								
							if(Get_Checking_FLAG(R_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".后驱右电机编码器数据异常");
								
							if(Get_Checking_FLAG(R_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".后驱右电机速度异常");
								
							if(Get_Checking_FLAG(R_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".后驱右电机内部参考电压出错");
								
							if(Get_Checking_FLAG(R_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".后驱右电机霍尔线未插"); 
							
							if(Get_Checking_FLAG(Drive2_ERROR)) printf("\r\n%d",tip++),printf(".前驱电机有错误,请先解决后驱报错再次查看获取具体错误"); 
						}
						
						if(Get_Checking_FLAG(Drvie_Timeout)) printf("\r\n%d",tip++),printf(".前驱电机驱动器离线"); 
						if(Get_Checking_FLAG(Drvie2_Timeout)) printf("\r\n%d",tip++),printf(".后驱电机驱动器离线"); 
					}
					else
					{
						if(Get_Checking_FLAG(Drvie_overVOL)) printf("\r\n%d",tip++),printf(".电机驱动过压");
							
						if(Get_Checking_FLAG(Drvie_underVOL)) printf("\r\n%d",tip++),printf(".电机驱动欠压");
			
						if(Get_Checking_FLAG(Drvie_EEPROM_ERROR)) printf("\r\n%d",tip++),printf(".电机驱动器内部EEPROM读写错误");
							
						if(Get_Checking_FLAG(L_Motor_overCUR)) printf("\r\n%d",tip++),printf(".左电机过流");
							
						if(Get_Checking_FLAG(L_Motor_overLoad)) printf("\r\n%d",tip++),printf(".左电机过载");
							
						if(Get_Checking_FLAG(L_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".左电机电流异常");
							
						if(Get_Checking_FLAG(L_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".左电机编码器数据异常");
							
						if(Get_Checking_FLAG(L_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".左电机速度异常");
							
						if(Get_Checking_FLAG(L_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".左电机内部参考电压出错");
							
						if(Get_Checking_FLAG(L_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".左电机霍尔线未插"); 

						if(Get_Checking_FLAG(R_Motor_overCUR)) printf("\r\n%d",tip++),printf(".右电机过流");
							
						if(Get_Checking_FLAG(R_Motor_overLoad)) printf("\r\n%d",tip++),printf(".右电机过载");
							
						if(Get_Checking_FLAG(R_Motor_CUR_ERROR)) printf("\r\n%d",tip++),printf(".右电机电流异常");
							
						if(Get_Checking_FLAG(R_Motor_Encoder_ERROR)) printf("\r\n%d",tip++),printf(".右电机编码器数据异常");
							
						if(Get_Checking_FLAG(R_Motor_SPEED_ERROR)) printf("\r\n%d",tip++),printf(".右电机速度异常");
							
						if(Get_Checking_FLAG(R_Motor_VOL_ERROR)) printf("\r\n%d",tip++),printf(".右电机内部参考电压出错");
							
						if(Get_Checking_FLAG(R_Motor_HAL_ERROR)) printf("\r\n%d",tip++),printf(".右电机霍尔线未插"); 

						if(Get_Checking_FLAG(Drvie_Timeout)) printf("\r\n%d",tip++),printf(".电机驱动器离线"); 
					}
				
					if(Get_Checking_FLAG(AutoRecharge_Timeout)) printf("\r\n%d",tip++),printf(".自动回充装备离线");
						
					if(Get_Checking_FLAG(Lower_Power)) printf("\r\n%d",tip++),printf(".电池电压不足");
					
					if(Get_Checking_FLAG(Stop_Switch_DOWN)) printf("\r\n%d",tip++),printf(".急停开关被按下");
					
					if(Get_Checking_FLAG(lost_left_redsignal)) printf("\r\n%d",tip++),printf(".未检测到充电桩左边的红外信号");

					if(Get_Checking_FLAG(lost_right_redsignal)) printf("\r\n%d",tip++),printf(".未检测到充电桩右边的红外信号");
					
					printf("}$");				
					send_error_app=0;
				}
				else
				{
					send_error_app=1;
					printf("{#底盘当前无报错/警告}$");
					send_error_app=0;								
				}
				
				countimes=0,get_error_flag=0;
			}	
		}
		else
			__nop();

    }
} 


/**************************************************************************
Function: Enable switch pin initialization
Input   : none
Output  : none
函数功能：使能开关引脚初始化
入口参数：无
返回  值：无 
**************************************************************************/
void Enable_Pin(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //KEY对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化GPIOB14
} 



void Mow_motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	// APB1 Timer clock 84M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM14); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
 
	TIM_TimeBaseStructure.TIM_Period = 8399; 
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure); 

 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     
	TIM_OC1Init(TIM14, &TIM_OCInitStructure); 
	
	//TIM_CtrlPWMOutputs(TIM14,ENABLE);

	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  
	TIM_ARRPreloadConfig(TIM14, ENABLE); 
	
	mow_motor = 8400;//满占空比停转电机	
	TIM_Cmd(TIM14, ENABLE);  
	
	//普通IO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	mow_dir = 0; //方向设置

}
