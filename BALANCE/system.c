/***********************************************





版本：V3.5
修改时间：2021-01-29






Version: V3.5
Update：2021-01-29


***********************************************/

#include "system.h"

//Robot software fails to flag bits
//机器人软件失能标志位
u8 Flag_Stop=0;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADC值分段变量，取决于小车型号数量，目前有6种小车型号
int Divisor_Mode;

// Robot type variable
//机器人型号变量
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//舵机控制PWM值，阿克曼小车专用
int Servo;  

//Default speed of remote control car, unit: mm/s
//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500; 

//Vehicle three-axis target moving speed, unit: m/s
//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//速度控制PID参数
float Velocity_KP=700,Velocity_KI=700; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//The parameter structure of the motor
//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

//串口数据检测变量,当串口无数据时，停止小车运动
u8 disable_robot_count;

/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//编码器精度
float Encoder_precision; 
//Wheel circumference, unit: m
//轮子周长，单位：m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//主动轮轮距，单位：m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//小车前后轴的轴距，单位：m
float Wheel_axlespacing; 
/************ 小车型号相关变量 **************************/
/************ Variables related to car model ************/
//robot control mode
//机器人控制模式，默认为ROS控制
u8 Control_Mode=0x01;

//Bluetooth remote control associated flag bits
//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//向蓝牙APP发送参数的标志位
u8 PID_Send; 

//The PS2 gamepad controls related variables
//PS2手柄控制相关变量
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

//Self-check the relevant flag variables
//自检相关标志变量
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; 

//Check the result code
//自检结果代码
long int ErrorCode=0; 

//实例化超声波测距结构体
Ultrasonic ultrasonic;

//自动回充板子检测参数，用于检查是否掉线
u8 CheckAutoRc=0,Last_CheckAutoRc=0,AutoRcCount =0;

//驱动报错检查标志位
u8 motor_checkerror_flag=0;
u8 motor2_checkerror_flag=0;

//RTOS频率监测变量
float fre_show;

u8 HUB1_EnableState=0;
u8 HUB2_EnableState=0;

u16 HUB1_TEST_L = 0;
u16 HUB1_TEST_R = 0;
u16 HUB2_TEST_L = 0;
u16 HUB2_TEST_R = 0;

u32 can1_send_error=0;
u32 can2_send_error=0;

void systemInit(void)
{       
	//Interrupt priority group setting
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//Delay function initialization
	//延时函数初始化
	delay_init(168);	
	
	//Cortex-M内核DWT计时器初始化
	#if USE_DWT_CORE
	DWT_Init();
	#endif

	//ADC pin initialization, used to read the battery voltage and potentiometer gear, 
	//potentiometer gear determines the car after the boot of the car model
	//ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
 	Adc_Init();  
	//According to the tap position of the potentiometer, determine which type of car needs to be matched, 
	//and then initialize the corresponding parameters	
	//根据电位器的档位判断需要适配的是哪一种型号的小车，然后进行对应的参数初始化	
	Robot_Select();       
	
	//Initialize the hardware interface connected to the LED lamp
	//初始化与LED灯连接的硬件接口
	LED_Init();                     
	    
	//Initialize the hardware interface connected to the buzzer	
	//初始化与蜂鸣器连接的硬件接口
	Buzzer_Init();  
	
	//Initialize the hardware interface connected to the enable switch
	//初始化与使能开关连接的硬件接口
	Enable_Pin();

	//Initialize the hardware interface connected to the OLED display
	//初始化与OLED显示屏连接的硬件接口	
	OLED_Init();     
	
	//Initialize the hardware interface connected to the user's key
	//初始化与用户按键连接的硬件接口
	KEY_Init();	
	
	//Serial port 1 initialization, communication baud rate 115200, 
	//can be used to communicate with ROS terminal
	//串口1初始化，通信波特率115200，可用于与ROS端通信
	uart1_init(115200);	  
	
	//wifi模块
//	uart6_init(460800);
	
	//Serial port 4 initialization, communication baud rate 9600, 
	//used to communicate with Bluetooth APP terminal
	//串口2初始化，通信波特率9600，用于与蓝牙APP端通信
//	uart2_init(230400);  
	uart2_init(9600); 
		
	//Serial port 4 is initialized and the baud rate is 115200. 
	//Serial port 4 is the default port used to communicate with ROS terminal
	//串口4初始化，通信波特率115200，串口4为默认用于与ROS端通信的串口
	uart4_init(115200);
	
	//串口转RS485初始化
	uart3_init(115200);

	//Initialize the CAN communication interface
	//CAN通信接口初始化
	//(1+7+6)*6 = 84M  波特率 = 42M/84M = 500k
	CAN_1_2_Init(1,6,7,6,0);

	//航模引脚初始化
	//定时器TIM4的频率为84M
	TIM4_Cap_Init(9999,84-1);   

	//超声波硬件初始化
	Ultrasonic_Init();
	
	//RGB灯带初始化
	RGB_lights_init();
	
	//IIC initialization for MPU9250
	//IIC初始化，用于MPU6050
	I2C_GPIOInit();

	//ICM20948 is initialized to read the vehicle's three-axis attitude, 
	//three-axis angular velocity and three-axis acceleration information
	//ICM20948初始化，用于读取小车三轴姿态、三轴角速度、三轴加速度信息
	invMSInit();       	 
	
	//给超声波测距变量赋初始值
	ultrasonic.A = ultrasonic.B = ultrasonic.C = ultrasonic.D = 
	ultrasonic.E = ultrasonic.F = ultrasonic.G = ultrasonic.H = 3;
	
	/* 无刷驱动器初始化前状态 */
	LED = 0;
	rgb_set[1] = 255 , rgb_set[2] = 0 , rgb_set[3] = 255;
	RGB_Set(rgb_set[1],rgb_set[2],rgb_set[3]);
	delay_ms(1000);
	delay_ms(500);
	HUB_Init(COBID_Send_1);//无刷电机初始化
	if(Car_Mode==S200) HUB_Init(COBID_Send_2);
	
	#if USE_IWDG //是否使用看门狗
	//看门狗初始化
	//32KHz总频率，设置32分频，1ms计1。
	//200ms必须喂狗1次
	IWDG_Init(IWDG_Prescaler_32,200);
	#endif
	
	//电池参考电压
	base_vol = Get_battery_volt();
}


//RTOS频率检查函数
float Check_Fre(void)
{
	static u32 temp_check_begin,temp_check_end;
	static int64_t temp_check_time;
	static u8 time_count_flag=0;
	float use_time;//记录用时，单位ms

	time_count_flag = !time_count_flag; //监测频率时，用于记录两次翻转的时间间隔
	if(time_count_flag==1) temp_check_begin = DWT_GetTick_us();//第1次翻转 || 记录中断最开始的时间
	else temp_check_end = DWT_GetTick_us();//第二次翻转

	if(time_count_flag==0)//第二次翻转后即可计算出频率
	{
		temp_check_time = temp_check_end - temp_check_begin;//翻转总时间
	if(temp_check_time<0) 
	{
		temp_check_time = temp_check_begin + 4294967295u - temp_check_end;//DWT为32位寄存器，如果小于0则加上32位寄存器最大值得出正确时间
	}
		use_time = (float)(temp_check_time/168.0f)/1000.0f;// DWT计时器频率为168M，÷168表示用了多少 us, ÷1000把单位转换为ms
	}

	return use_time;
}

