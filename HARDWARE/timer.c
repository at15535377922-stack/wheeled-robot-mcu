#include "timer.h"

//Input the capture flag for channel 1, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u8 TIM4CH1_CAPTURE_STA = 0;	
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

//Input the capture flag for channel 2, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM4CH2_CAPTURE_STA = 0;		
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

//Input the capture flag for channel 3, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志	
u8 TIM4CH3_CAPTURE_STA = 0;		
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

//Input the capture flag for channel 4, 
//the capture flag for the higher bits, and the overflow flag for the lower 6 bits
//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志
u8 TIM4CH4_CAPTURE_STA = 0;			
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

u32 TIM4_T1;
u32 TIM4_T2;
u32 TIM4_T3;
u32 TIM4_T4;

//Variables related to remote control acquisition of model aircraft
//航模遥控采集相关变量
int Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;
//Model aircraft remote control receiver variable
//航模遥控接收变量
int L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  

//航模误判检测变量
u8 error_judge = 0;

/**************************************************************************
Function: Model aircraft remote control initialization function, timer 1 input capture initialization
Input   : arr: Automatic reload value, psc: clock preset frequency
Output  : none
函数功能：航模遥控初始化函数，定时器1输入捕获初始化
入口参数：arr：自动重装值，psc：时钟预分频数 
返 回 值：无
**************************************************************************/ 
void TIM4_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM4时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTE时钟	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);

	/*** Initialize timer 1 || 初始化定时器1 ***/
	//Set the counter to automatically reload //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Period = arr; 
	//Pre-divider //预分频器 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	
	//Set the clock split: TDTS = Tck_tim //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	//TIM up count mode //TIM向上计数模式	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	//Initializes the timebase unit for TIMX based on the parameter specified in TIM_TimeBaseInitStruct
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	/*** 初始化TIM4输入捕获参数，通道1 || Initialize TIM4 for the capture parameter, channel 1 ***/
	//Select input //选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
  //Rising edge capture //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
  //IC1F=0000 Configure input filter //配置输入滤波器
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	/*** 初始化TIM4输入捕获参数，通道2 || Initialize TIM4 for the capture parameter, channel 2 ***/
	//CC1S=01 Select input //选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	//Rising edge capture //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	/*** 初始化TIM4输入捕获参数，通道3 || Initialize TIM4 for the capture parameter, channel 3 ***/
	//Select input //选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;   
	//Rising edge capture //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  
	//IC1F=0000 Configure input filter //配置输入滤波器，不滤波  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	/*** 初始化TIM4输入捕获参数，通道4 || Initialize TIM4 for the capture parameter, channel 4 ***/
	//Select input //选择输入端 
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	//Rising edge capture //上升沿捕获
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	//Configure input frequency division, regardless of frequency //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	   
	//IC1F=0000 Configure input filter //配置输入滤波器，不滤波  
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

  /*** interrupt packet initialization || 中断分组初始化 ***/
  //TIM4 interrupts //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
  //Preempt priority 0 //先占优先级0级	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	//Level 0 from priority //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	//IRQ channels are enabled //IRQ通道被使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	//Initializes the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	NVIC_Init(&NVIC_InitStructure);   
	
  //Allow CC1IE,CC2IE,CC3IE,CC4IE to catch interrupts, not allowed update_interrupts
  //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   

	//Enable timer //使能定时器
	TIM_Cmd(TIM4, ENABLE); 		
}
/**************************************************************************
Function: Model aircraft remote control receiving interrupt, namely timer 1 input capture interrupt
Input   : none
Output  : none
函数功能：航模遥控接收中断，即定时器1输入捕获中断
入口参数：无
返 回 值：无
**************************************************************************/ 
void TIM4_IRQHandler(void)
{
	static u8 ch1_filter_times=0,ch2_filter_times=0,ch3_filter_times=0,ch4_filter_times=0;
	
	error_judge = 0; //进入中断，清空错误累计变量
	
	//连接航模遥遥控器后，需要推下前进杆，才可以正式航模控制小车
	//After connecting the remote controller of the model aircraft, 
	//you need to push down the forward lever to officially control the car of the model aircraft
  if(Remoter_Ch2>1600&&Get_Control_Mode(_RC_Control)==0&&Deviation_Count>=CONTROL_DELAY&&Allow_Recharge==0) //自动回充模式下禁用航模使能标志位，但是仍接收航模摇杆的控制量
  {
		//Model aircraft remote control mark position 1, other marks position 0
		//航模遥控标志位置1，其它标志位置0
		Set_Control_Mode(_RC_Control);
	}
	 
	//进入更新中断时，直接清除中断标志位，不作其他操作
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	
	//Channel 1 //通道一
	if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) 			
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) //A capture event occurred on channel 1 //通道1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); //Clear the interrupt flag bit //清除中断标志位
			if (TIM4CH1_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
				{
					TIM4_T1 = 9999;
				}
				else
					TIM4_T1 = 0;
				Remoter_Ch1 = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL + TIM4_T1;	//Time to get the total high level //得到总的高电平的时间
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500)
				{
					ch1_filter_times++;
					if( ch1_filter_times<=5 ) Remoter_Ch1=L_Remoter_Ch1; //Filter //滤波	
					else ch1_filter_times=0;
				}
				else
				{
					ch1_filter_times=0;
				}
				L_Remoter_Ch1=Remoter_Ch1;
				
				TIM4CH1_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4); //Obtain rising edge data //获取上升沿数据
				TIM4CH1_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}
  //Channel 2 //通道二
	if ((TIM4CH2_CAPTURE_STA & 0X80) == 0)		
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)	//A capture event occurred on channel 2 //通道2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2); //Clear the interrupt flag bit //清除中断标志位
			if (TIM4CH2_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
				{
					TIM4_T2 = 9999;
				}
				else
					TIM4_T2 = 0;
				Remoter_Ch2 = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL + TIM4_T2; //Time to get the total high level //得到总的高电平的时间
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)
				{
					ch2_filter_times++;
					if( ch2_filter_times<=5 ) Remoter_Ch2=L_Remoter_Ch2; //Filter //滤波	
					else ch2_filter_times=0;
				}
				else
				{
					ch2_filter_times=0;
				}
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM4CH2_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4); //Obtain rising edge data //获取上升沿数据
				TIM4CH2_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}
  //Channel 3 //通道三
	if ((TIM4CH3_CAPTURE_STA & 0X80) == 0)			
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)	//A capture event occurred on channel 3 //通道3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); //Clear the interrupt flag bit //清除中断标志位
			if (TIM4CH3_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
			{
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4); //Record the timer value at this point //记录下此时的定时器计数值
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
				{
					TIM4_T3 = 9999;
				}
				else
					TIM4_T3 = 0;
				Remoter_Ch3 = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL + TIM4_T3; //Time to get the total high level //得到总的高电平的时间
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)
				{
					ch3_filter_times++;
					if( ch3_filter_times<=5 ) Remoter_Ch3=L_Remoter_Ch3; //Filter //滤波	
					else ch3_filter_times=0;
				}
				else
				{
					ch3_filter_times=0;
				}
				L_Remoter_Ch3=Remoter_Ch3;
				TIM4CH3_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
			}
			else 
			{
				//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				//发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4); //Obtain rising edge data //获取上升沿数据
				TIM4CH3_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
			}
		}
	}
		//Channel 4 //通道四
		if ((TIM4CH4_CAPTURE_STA & 0X80) == 0)		
		{
			if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)	//A capture event occurred on channel 4 //通道4发生捕获事件
			{
				TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); //Clear the interrupt flag bit //清除中断标志位
				if (TIM4CH4_CAPTURE_STA & 0X40)	//A falling edge is caught //捕获到一个下降沿
				{
					TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4); //Record the timer value at this point //记录下此时的定时器计数值
					if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
					{
						TIM4_T4 = 9999;
					}
					else
						TIM4_T4 = 0;
					Remoter_Ch4 = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL + TIM4_T4; //Time to get the total high level //得到总的高电平的时间
					if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)
					{
						ch4_filter_times++;
						if( ch4_filter_times<=5 ) Remoter_Ch4=L_Remoter_Ch4; //Filter //滤波	
						else ch4_filter_times=0;
					}
					else
					{
						ch4_filter_times=0;
					}
					L_Remoter_Ch4=Remoter_Ch4;				
					TIM4CH4_CAPTURE_STA = 0; //Capture flag bit to zero	//捕获标志位清零
					TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //Set to rising edge capture //设置为上升沿捕获		  
				}
				else 
				{
					//When the capture time occurs but not the falling edge, the first time the rising edge is captured, record the timer value at this time
				  //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
					TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4); //Obtain rising edge data //获取上升沿数据
					TIM4CH4_CAPTURE_STA |= 0X40; //The flag has been caught on the rising edge //标记已捕获到上升沿
					TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling); //Set to Falling Edge Capture //设置为下降沿捕获
				}
			}
		}
		
	//如果开启了自动回充模式，需要检测航模摇杆是否有取消自动回充模式的操作
	if(Allow_Recharge)
	{
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);
		//摇杆外八,左摇杆往左下角，右摇杆往右下角，航模开启/关闭自动回充模式
		if((Remoter_Ch1>1950&&Remoter_Ch1<2005)&&(Remoter_Ch2>1000&&Remoter_Ch2<1050)&&(Remoter_Ch3>1000&&Remoter_Ch3<1050)&&(Remoter_Ch4>1000&&Remoter_Ch4<1050))
		{
			allow_recharge_time_on=1;
			if(allow_Recharge_time>=100) //allow_Recharge_time在100Hz任务里走时，100代表走时1s；实际上航模识别不是完全稳定的，可能用时不止1s才触发
			{
				allow_Recharge_time=0;
				allow_recharge_time_on=0;
				Allow_Recharge=0;
				recharge_flag_beep=1;
				rm_stop_scan=1;
			}
		}
		else
			allow_recharge_time_on=0;
	}
		
}

