#include "Ultrasonic.h"

#define USE_KALMAN_FILTER 0   //是否使用卡尔曼滤波器
#define USE_SLIDIING_FILTER 1 //是否使用滑动滤波器

//注：US代号，含义UltraSound
//超声波输入捕获状态位
//状态规定：0初始化状态；2第二次进入捕获
//注：在中断与任务之间共享，必须加volatile，否则编译器优化后可能把变量缓存进寄存器
volatile u8 US_A_FLag = 0;
volatile u8 US_B_FLag = 0;
volatile u8 US_C_FLag = 0;
volatile u8 US_D_FLag = 0;
volatile u8 US_E_FLag = 0;
volatile u8 US_F_FLag = 0;

//用于保存第一次捕获时的数据
u16 Last_US_A_Data;
u16 Last_US_B_Data;
u16 Last_US_C_Data;
u16 Last_US_D_Data;
u16 Last_US_E_Data;
u16 Last_US_F_Data;

//用于保存第二次捕获时的数据
u16 US_A_Data;
u16 US_B_Data;
u16 US_C_Data;
u16 US_D_Data;
u16 US_E_Data;
u16 US_F_Data;

//用于表示当前通道已经捕获结束
//注：ReadUS_task在while(1)里轮询这些标志，中断里清零，必须加volatile
volatile u8 END_A=0;
volatile u8 END_B=0;
volatile u8 END_C=0;
volatile u8 END_D=0;
volatile u8 END_E=0;
volatile u8 END_F=0;

//用于保存计算得到的距离
//float ultrasonic.A=5.0f;
//float ultrasonic.B=5.0f;
//float ultrasonic.C=5.0f;
//float ultrasonic.D=5.0f;
//float ultrasonic.E=5.0f;
//float ultrasonic.F=5.0f;

//超声波触发IO初始化
void Trigger_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE, ENABLE);//Enable port clock //使能端口时钟
	
	//PE8 ->A
	//PE7 ->B
	//PC4 ->C
	//PA4 ->D
	//PA5 ->E
	//PC5 ->F
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //Port configuration //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);	//Initialize GPIO with the specified parameters //根据设定参数初始化GPIO
	//Trig为高电平有效脉冲，空闲态必须为低，否则第1次触发发不出上升沿
	GPIO_ResetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5; //Port configuration //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);	//Initialize GPIO with the specified parameters //根据设定参数初始化GPIO
	GPIO_ResetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_5);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8; //Port configuration //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);	//Initialize GPIO with the specified parameters //根据设定参数初始化GPIO
	GPIO_ResetBits(GPIOE,GPIO_Pin_7|GPIO_Pin_8);
}


//超声波捕获定时器引脚初始化
//超声波C -> TIM3_CH1 -> PA6
//超声波F -> TIM3_CH2 -> PA7
//超声波A -> TIM3_CH4 -> PB1
//超声波B -> TIM3_CH3 -> PB0

//超声波D -> TIM2_CH1 -> PA0
//超声波E -> TIM2_CH2 -> PA1
void Ultrasonic_Init(void)
{
	Trigger_IO_Init(); //初始化普通IO部分
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//IO时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//开启对应时钟，配置对应GPIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0|GPIO_Pin_1); //设置成低电平
	
	//定时器基础属性初始化
	//定时器基础属性初始化 注：TIM2的计数器CNT是32位的，不止可以设置65535，最高2^32-1=4,294,967,295
	TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = 83; //预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	
	//定时器输入捕获通道初始化
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CH1通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CH1通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//强占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);//设置优先级分组
	TIM_ITConfig(TIM2, TIM_IT_CC1|TIM_IT_CC2,ENABLE);//允许TIM1_CH1捕获中断
	TIM_Cmd(TIM2, ENABLE);//使能定时器1
	
	//开启对应时钟，配置对应GPIO
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7); //设置成低电平
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0|GPIO_Pin_1); //设置成低电平
	
	TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = 83; //预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
	
	//定时器输入捕获通道初始化
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CH2通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CH2通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //CH3通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //CH4通道
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连模式
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x00;//滤波器设置，暂时不配置
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//强占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);//设置优先级分组
	TIM_ITConfig(TIM3, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);//允许TIM1_CH1捕获中断
	TIM_Cmd(TIM3, ENABLE);//使能定时器1

}


//超声波A -> TIM3_CH4 -> PB1
//超声波B -> TIM3_CH3 -> PB0
//超声波C -> TIM3_CH1 -> PA6
//超声波D -> TIM2_CH1 -> PA0
//超声波E -> TIM2_CH2 -> PA1
//超声波F -> TIM3_CH2 -> PA7
//宏定义接口
/*
1.设置捕获极性 Set_US_A_Rising、Set_US_A_Falling
2.获取计数值   Get_US_A_CNT
3.获取中断状态 Get_US_A_State
4.清除中断状态 Clear_US_A_State
*/
	
float rangeA_KalmanFilter(float distance);
float rangeB_KalmanFilter(float distance);
float rangeC_KalmanFilter(float distance);
float rangeD_KalmanFilter(float distance);
float rangeE_KalmanFilter(float distance);
float rangeF_KalmanFilter(float distance);
	
void US_ABCF_Read(void)
{
	////////////////  超声波A ////////////////
	if(Get_US_A_State != RESET)//TIM5CH3发生输入捕获事件
	{
		Clear_US_A_State;//清除中断标志位

		if(US_A_FLag==2)//第2次进入中断
		{
			US_A_Data = Get_US_A_CNT;//记录第二次捕获到的值
			if(US_A_Data>Last_US_A_Data) //计数器没有溢出的情况
			{
				ultrasonic.A = (US_A_Data - Last_US_A_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.A = (US_A_Data + 65536 - Last_US_A_Data) / 5750.0f;
			}
			
			if(ultrasonic.A>3.0f) ultrasonic.A = 3.0f;		
			
			#if USE_KALMAN_FILTER
			ultrasonic.A = rangeA_KalmanFilter(ultrasonic.A);
			#endif
			
			#if USE_SLIDIING_FILTER
			ultrasonic.A = Mean_Filter_A(ultrasonic.A);
			#endif
			
			US_A_FLag=0; //状态位复原
			timeout_A=0; //超时时间复位
			Set_US_A_Rising;  //设置TIM5CH3为上升沿捕获		  
			END_A = 0;
		}
		else if(US_A_FLag==0)
		{
			Last_US_A_Data = Get_US_A_CNT; //记录第1次捕获到的值
			US_A_FLag = 2; //标记已经经过了第1次捕获
			Set_US_A_Falling; //设置TIM5CH3为下降沿捕获
		}

	}
	////////////////  超声波A ////////////////
	
	////////////////  超声波B ////////////////
	if(Get_US_B_State != RESET)//TIM2CH4发生输入捕获事件
	{
		Clear_US_B_State;//清除中断标志位

		if(US_B_FLag==2)//第2次进入中断
		{
			US_B_Data = Get_US_B_CNT;//记录第二次捕获到的值
			if(US_B_Data>Last_US_B_Data) //计数器没有溢出的情况
			{
				ultrasonic.B = (US_B_Data - Last_US_B_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.B = (US_B_Data + 65536 - Last_US_B_Data) / 5750.0f;
			}
			
			if(ultrasonic.B>3.0f) ultrasonic.B = 3.0f;
			#if USE_KALMAN_FILTER
			ultrasonic.B = rangeB_KalmanFilter(ultrasonic.B);
			#endif	
			
			#if USE_SLIDIING_FILTER
			ultrasonic.B = Mean_Filter_B(ultrasonic.B);
			#endif
			US_B_FLag=0; //状态位复原
			timeout_B=0; //超时时间复位
			Set_US_B_Rising;  //设置TIM2CH4为上升沿捕获		  
			END_B = 0;
		}
		else if(US_B_FLag==0)
		{
			Last_US_B_Data = Get_US_B_CNT; //记录第1次捕获到的值
			US_B_FLag = 2; //标记已经经过了第1次捕获
			Set_US_B_Falling; //设置TIM2CH4为下降沿捕获
		}

	}
	////////////////  超声波B ////////////////

	////////////////  超声波C ////////////////
	if(Get_US_C_State != RESET)//TIM2CH3发生输入捕获事件
	{
		Clear_US_C_State;//清除中断标志位

		if(US_C_FLag==2)//第2次进入中断
		{
			US_C_Data = Get_US_C_CNT;//记录第二次捕获到的值
			if(US_C_Data>Last_US_C_Data) //计数器没有溢出的情况
			{
				ultrasonic.C = (US_C_Data - Last_US_C_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.C = (US_C_Data + 65536 - Last_US_C_Data) / 5750.0f;
			}
			
			if(ultrasonic.C>3.0f) ultrasonic.C = 3.0f;
			#if USE_KALMAN_FILTER
			ultrasonic.C = rangeC_KalmanFilter(ultrasonic.C);
			#endif	
			
			#if USE_SLIDIING_FILTER
			ultrasonic.C = Mean_Filter_C(ultrasonic.C);
			#endif
			US_C_FLag=0; //状态位复原
			timeout_C=0; //超时时间复位
			Set_US_C_Rising;  //设置TIM2CH3为上升沿捕获		  
			END_C = 0;
		}
		else if(US_C_FLag==0)
		{
			Last_US_C_Data = Get_US_C_CNT; //记录第1次捕获到的值
			US_C_FLag = 2; //标记已经经过了第1次捕获
			Set_US_C_Falling; //设置TIM2CH3为下降沿捕获
		}

	}
	////////////////  超声波C ////////////////
	
	////////////////  超声波F ////////////////
	if(Get_US_F_State != RESET)//TIM5CH4发生输入捕获事件
	{
		Clear_US_F_State;//清除中断标志位

		if(US_F_FLag==2)//第2次进入中断
		{
			US_F_Data = Get_US_F_CNT;//记录第二次捕获到的值
			if(US_F_Data>Last_US_F_Data) //计数器没有溢出的情况
			{
				ultrasonic.F = (US_F_Data - Last_US_F_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.F = (US_F_Data + 65536 - Last_US_F_Data) / 5750.0f;
			}
			
			if(ultrasonic.F>3.0f) ultrasonic.F = 3.0f;
			#if USE_KALMAN_FILTER
			ultrasonic.F = rangeF_KalmanFilter(ultrasonic.F);
			#endif	
			
			#if USE_SLIDIING_FILTER
			ultrasonic.F = Mean_Filter_F(ultrasonic.F);
			#endif
			US_F_FLag=0; //状态位复原
			timeout_F=0; //超时时间复位
			Set_US_F_Rising;  //设置TIM5CH4为上升沿捕获		  
			END_F = 0;
		}
		else if(US_F_FLag==0)
		{
			Last_US_F_Data = Get_US_F_CNT; //记录第1次捕获到的值
			US_F_FLag = 2; //标记已经经过了第1次捕获
			Set_US_F_Falling; //设置TIM5CH4为下降沿捕获
		}

	}
	////////////////  超声波F ////////////////
}

void US_DE_Read(void)
{
	////////////////  超声波D ////////////////
	if(Get_US_D_State != RESET)//TIM2CH2发生输入捕获事件
	{
		Clear_US_D_State;//清除中断标志位

		if(US_D_FLag==2)//第2次进入中断
		{
			US_D_Data = Get_US_D_CNT;//记录第二次捕获到的值
			if(US_D_Data>Last_US_D_Data) //计数器没有溢出的情况
			{
				ultrasonic.D = (US_D_Data - Last_US_D_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.D = (US_D_Data + 65536 - Last_US_D_Data) / 5750.0f;
			}
			
			if(ultrasonic.D>3.0f) ultrasonic.D = 3.0f;
			#if USE_KALMAN_FILTER
			ultrasonic.D = rangeD_KalmanFilter(ultrasonic.D);
			#endif	
			
			#if USE_SLIDIING_FILTER
			ultrasonic.D = Mean_Filter_D(ultrasonic.D);
			#endif			
			US_D_FLag=0; //状态位复原
			timeout_D=0; //超时时间复位
			Set_US_D_Rising;  //设置TIM2CH2为上升沿捕获		  
			END_D = 0;
		}
		else if(US_D_FLag==0)
		{
			Last_US_D_Data = Get_US_D_CNT; //记录第1次捕获到的值
			US_D_FLag = 2; //标记已经经过了第1次捕获
			Set_US_D_Falling; //设置TIM2CH2为下降沿捕获
		}

	}
	////////////////  超声波D ////////////////

	////////////////  超声波E ////////////////
	if(Get_US_E_State != RESET)//TIM1CH1发生输入捕获事件
	{
		Clear_US_E_State;//清除中断标志位

		if(US_E_FLag==2)//第2次进入中断
		{
			US_E_Data = Get_US_E_CNT;//记录第二次捕获到的值
			if(US_E_Data>Last_US_E_Data) //计数器没有溢出的情况
			{
				ultrasonic.E = (US_E_Data - Last_US_E_Data) / 5750.f;
			}
			else //计数器已经溢出，需要加回溢出值
			{
				ultrasonic.E = (US_E_Data + 65536 - Last_US_E_Data) / 5750.0f;
			}
			
			if(ultrasonic.E>3.0f) ultrasonic.E = 3.0f;
			
			#if USE_KALMAN_FILTER
			ultrasonic.E = rangeE_KalmanFilter(ultrasonic.E);
			#endif	
			
			#if USE_SLIDIING_FILTER
			ultrasonic.E = Mean_Filter_E(ultrasonic.E);
			#endif
					
			US_E_FLag=0; //状态位复原
			timeout_E=0; //超时时间复位
			Set_US_E_Rising;//设置TIM1CH1上升沿捕获
			END_E = 0;

		}
		else if(US_E_FLag==0)
		{
			Last_US_E_Data = Get_US_E_CNT; //记录第1次捕获到的值
			US_E_FLag = 2; //标记已经经过了第1次捕获
			Set_US_E_Falling;//设置TIM1CH1下降沿捕获
		}

	}
	////////////////  超声波E ////////////////
}

//////////////  滑动均值滤波  ////////////////
#define FILTERING_TIMES 4

//滤波缓冲区复位请求标志
//置1的时机：(1)上电首次采样，避免缓冲区初值0导致输出从0往上爬的开机毛刺
//           (2)该路超时后恢复，避免缓冲区里残留超时前的旧数据
//由ReadUS_task(任务上下文)置1，由滤波函数(中断上下文)读取并清零；
//缓冲区本身始终只被中断碰，所以不存在任务与中断争用缓冲区的问题
volatile u8 us_filter_reset_A = 1;
volatile u8 us_filter_reset_B = 1;
volatile u8 us_filter_reset_C = 1;
volatile u8 us_filter_reset_D = 1;
volatile u8 us_filter_reset_E = 1;
volatile u8 us_filter_reset_F = 1;

float Mean_Filter_A(float dis_A)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_A)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_A;
    us_filter_reset_A = 0;
    return dis_A;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_A;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}

float Mean_Filter_B(float dis_B)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_B)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_B;
    us_filter_reset_B = 0;
    return dis_B;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_B;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}

float Mean_Filter_C(float dis_C)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_C)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_C;
    us_filter_reset_C = 0;
    return dis_C;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_C;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}

float Mean_Filter_D(float dis_D)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_D)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_D;
    us_filter_reset_D = 0;
    return dis_D;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_D;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}

float Mean_Filter_E(float dis_E)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_E)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_E;
    us_filter_reset_E = 0;
    return dis_E;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_E;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}

float Mean_Filter_F(float dis_F)
{
  u8 i;
  float Sum_Count = 0;
  float Filter_ref;
  static float Filter_Buf[FILTERING_TIMES]={0};//滤波缓冲区

  //收到复位请求：用当前值填满缓冲区并直接返回，不让旧数据参与平均
  if(us_filter_reset_F)
  {
    for(i = 0 ; i < FILTERING_TIMES; i++) Filter_Buf[i] = dis_F;
    us_filter_reset_F = 0;
    return dis_F;
  }

  //所有数据左移1格，剔除最开始的数据
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Filter_Buf[i - 1] = Filter_Buf[i];
  }
  
  //放入新数据
  Filter_Buf[FILTERING_TIMES - 1] = dis_F;

  //新数据取平均值
  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Count += Filter_Buf[i];
  }
  Filter_ref = (Sum_Count / FILTERING_TIMES);
  
  //返回滤波结果
  return Filter_ref;
}


//权重：R越大，更相信预测值；R越小，更相信测量值
#define Kalman_R 0.2f 

float rangeA_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	
	static float Q=0.01f;//噪声，测量值与实际误差，超声波测量值比较准确，可设置为0

	X_dot=X;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	
	return X;
}

float rangeB_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	static float Q=0.01f;//噪声

	X_dot=X+0;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	return X;
}

float rangeC_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	static float Q=0.01f;//噪声

	X_dot=X+0;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	return X;
}

float rangeD_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	static float Q=0.01f;//噪声

	X_dot=X+0;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	return X;
}

float rangeE_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	static float Q=0.01f;//噪声

	X_dot=X+0;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	return X;
}

float rangeF_KalmanFilter(float distance)
{
	//一维卡尔曼滤波
	static float P=1;
	static float P_dot;
	static float X=0;
	static float X_dot;
	static float K=0;
	static float Q=0.01f;//噪声

	X_dot=X+0;
	P_dot=P+Q;
	K=P_dot/(P_dot+Kalman_R);
	X=X_dot+K*(distance-X_dot);
	P=P_dot-K*P_dot;
	return X;
}

