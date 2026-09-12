#include "balance.h"

//Whether the robot model is incorrectly marked
//机器人型号是否错误标志位
u32 Self_CheckingFlag=0;
int Time_count=0; //Time variable //计时变量

//Forward and backward velocity acceleration
//前进后退速度加速度
float Velocity_Smoother_Rate=0.02;
float Velocity_Smoother_Rate_Z=0.02;

u8 DisableMotor; //电机禁用计时
u8 motor_state=0;  //电机状态识别，用于判断是否需要发送失能或使能电机命令
u8 motor_clear_error=0; //无刷电机报错消除判断位

u8 allow_Recharge_time=0;//进入自动回充模式的计时内核
u8 allow_recharge_time_on=0;//开启自动回充模式内核走时的标志位
u8 rm_stop_scan=0; //停止航模扫描标志位

u8 SecurityPLY = 0;//安全策略标志位

#if USE_US_Avoid
u8 Open_US_avoid=0;//底盘避障状态标志位
float start_avoid = 0.5f; //0.5m开始避障
void Ultrasonic_avoid_S150(float* vx,float* vz);
void Ultrasonic_avoid_S300(float* vx,float* vz);
void Ultrasonic_avoid_S100(float* vx,float* vz);
#endif

void Ultrasonic_safeguard(float* vx,float* vz); //底线防撞处理逻辑

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vz)
{
	//Wheel target speed limit 
	//车轮目标速度限幅 单位m/s
	float amplitude=3.5;
	
	//如果启用了底盘避障
	#if USE_US_Avoid
	//在 ROS\CAN\串口 控制下取消底盘自带的避障功能 
	if(Get_Control_Mode(_ROS_Control)||Get_Control_Mode(_USART_Control)||Get_Control_Mode(_CAN_Control))
	{
		Open_US_avoid=0;//ROS控制下关闭底盘避障功能
	}
	else //非ROS控制且用户启用了底盘避障，开启超声波避障
	{
		if(Open_US_avoid)
		{
			if(Car_Mode==S300)
			{
				Ultrasonic_avoid_S300(&Vx,&Vz);//S300避障方法
			}
			else if (Car_Mode==S150)
			{
				Ultrasonic_avoid_S150(&Vx,&Vz);//S150避障方法
			}
			else if (Car_Mode==S100)
			{
				Ultrasonic_avoid_S100(&Vx,&Vz);//S100避障方法(超声波为用户加装)
			}

		}
	}
	#endif

	//底线防撞处理，无论上层如何控制，超声波传感器检测低于阈值后自动避障
	Ultrasonic_safeguard(&Vx,&Vz);

	Smooth_control(Vx,Vz, Velocity_Smoother_Rate, Velocity_Smoother_Rate_Z); //对输入速度进行平滑处理	    
	Vx=smooth_control.VX;   //获取平滑处理后的数据
	Vz=smooth_control.VZ;   //获取平滑处理后的数据
	
	if(Car_Mode==S300 || Car_Mode==S150||Car_Mode==S100)
	{
		//Inverse kinematics //运动学逆解
		MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; 
		MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; 

		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude);
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude);
	}
	else if(Car_Mode==S200)
	{
		MOTOR_A.Target = +Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
		MOTOR_B.Target = +Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
		MOTOR_C.Target = +Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
		MOTOR_D.Target = +Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
		
		//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);
	}

}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
//临时调试使用
extern short showArpm,showBrpm,showCrpm,showDrpm;
float simple_pid(double now,double target)
{
	static float Last_bais;
	float bais;
	float speed;
	
	bais = now - target;
	speed = -0.2f*bais - 0.2f*Last_bais;
	Last_bais = bais;
	
	return speed;
}
u8 lineflag=0,turnflag=0;
//临时调试使用

void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
		// This task runs at a frequency of 100Hz (10ms control once)
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
		
		#if USE_IWDG
		IWDG_ReloadCounter();//200ms内必须喂狗1次，否则将导致系统复位重启
		#endif
		
		//当 CAN/串口 控制时，如果出现接口内无数据超出1秒，则停止机器人运动
		if( SecurityPLY==0 ) //安全策略，该位 置1时 解除安全措施
		{
			if(Get_Control_Mode(_USART_Control)||Get_Control_Mode(_CAN_Control)||Get_Control_Mode(_ROS_Control))
			{
				disable_robot_count++;
				if(disable_robot_count>100) disable_robot_count=0,Move_X=0,Move_Y=0,Move_Z=0;	
			}
		}
		else
		{
			LED = 0;//解除了安全措施，LED常亮
		}

		//机器人溜车/驱动异常行为制止
//		robot_check();
			
		//Time count is no longer needed after 30 seconds
		//时间计数，30秒后不再需要
		if(Time_count<3000)
		{
			Time_count++;
			if(Time_count==500) get_error_flag=1;//在开机第5秒时主动返回1次自检数据到app
		}
		
		/* 编码器读取以及计算速度在CAN1中断 */
		
		/* 接收控制命令与运动学解析部分 */
		//自动回充模式，由充电装备控制机器人运动
		if(Allow_Recharge==1)
		{
			//如果开启了导航回充，同时没有接收到红外信号，接收来自上位机的的回充控制命令
			if      (nav_walk==1 && RED_STATE==0) Drive_Motor(Recharge_UP_Move_X,Recharge_UP_Move_Z); 
			
			//接收到了红外信号，接收来自回充装备的回充控制命令
			else if (RED_STATE!=0) nav_walk = 0,Drive_Motor(Recharge_Red_Move_X,Recharge_Red_Move_Z); 
			
			//防止没有红外信号时小车运动
			if (nav_walk==0&&RED_STATE==0) Drive_Motor(0,0); 
			
			if(allow_recharge_time_on) allow_Recharge_time++; //自动回充模式走时
			else allow_Recharge_time=0;
			
		}
		else//非自动回充模式，由蓝牙\航模\ROS控制
		{		
			if      (Get_Control_Mode(_APP_Control))     Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			else if (Get_Control_Mode( _RC_Control))     Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
			else if (Get_Control_Mode(_PS2_Control))     PS2_control();    //PS2手柄控制命令
			
			//CAN, Usart 1, Usart 3 control can directly get the three axis target speed, 
			//without additional processing
			//CAN、串口1、串口3(ROS)控制直接得到三轴目标速度，无须额外处理
			else                      Drive_Motor(Move_X,Move_Z);
		}
		
		/* 电机逻辑控制部分 */
		//电机允许被控制的情况:
		//1、电压足够 voltage>20
		//2、软件使能位未作用   Flag_Stop=0
		//3、硬件使能开关未作用 EN=1
		//4、电压不足但是处于自动回充模式
		if(Turn_Off(Voltage)==0)
		{
			short Arpm=0,Brpm=0,Crpm=0,Drpm=0;
			DisableMotor=0;
			
			//Target speed is converted from mm/s to RPM (cycles per minute)
			//目标速度单位mm/s转换为rpm(圈/分钟)
			Arpm =  round(MOTOR_A.Target*60/Wheel_perimeter); 
			Brpm =  round(MOTOR_B.Target*60/Wheel_perimeter);	
			Crpm =  round(MOTOR_C.Target*60/Wheel_perimeter);	
			Drpm =  round(MOTOR_D.Target*60/Wheel_perimeter);
			
			if(Car_Mode==S300)
			{
				set_MotorRPM(COBID_Send_1,Arpm, -Brpm);
			}
			else if(Car_Mode==S150||Car_Mode==S100)
			{
				set_MotorRPM(COBID_Send_1,-Arpm, Brpm);
			}
			else if(Car_Mode==S200)
			{
				//临时调试使用
				showArpm = Arpm,showBrpm=Brpm;
				showCrpm = -Crpm,showDrpm=-Drpm;
				set_MotorRPM(COBID_Send_1, Arpm, -Drpm);
				set_MotorRPM(COBID_Send_2, Brpm, -Crpm);
			}
			
			//检测是否需要清除PWM并自动执行清理
			auto_pwm_clear();
			
		}
		
		else //电机不允许控制
		{
			if(motor_state==0)
			{
				DisableMotor++; //滤波次数：5次共50ms
				if(DisableMotor>5)
				{
					vTaskDelay(1);
					set_MotorRPM(COBID_Send_1,0,0);//电机速度置0
					set_MotorRPM(COBID_Send_2,0,0);//电机速度置0
					vTaskDelay(1);
					CAN_Stop(COBID_Send_1);
					vTaskDelay(1);
					CAN_Stop(COBID_Send_2);
					vTaskDelay(1);
					
					//收到两个驱动的失能反馈,才结束can数据的发送
					if( HUB1_EnableState==MOTOR_DISABLE && HUB2_EnableState==MOTOR_DISABLE )
						DisableMotor=0, //滤波器复位
						motor_state=1;//标记电机当前的状态，已发送失能命令
					
				}
			}
		}
		
		//无刷电机报错消除
		//如出现电机驱动报错，可通过蓝牙\航模消除报错
		if(motor_clear_error)
		{
			Allow_Recharge=0; //关闭自动回充模式
			CAN_ClearError(COBID_Send_1);//清除报错
			CAN_Enable(COBID_Send_1);//重新使能电机
			CAN_ClearError(COBID_Send_2);//清除报错
			CAN_Enable(COBID_Send_2);//重新使能电机
			set_MotorRPM(COBID_Send_1,0,0);//设置电机速度为0防止机器人运动
			set_MotorRPM(COBID_Send_2,0,0);//设置电机速度为0防止机器人运动
			if( HUB1_EnableState==MOTOR_ENABLE && HUB2_EnableState==MOTOR_ENABLE )
				motor_clear_error=0,
				beep_flag=1,//蜂鸣器提示进入消除报错模式
				motor_beep=1;
		}
		
		//Click double the user button to update the gyroscope zero
		//双击用户按键更新陀螺仪零点,单击用户按键切换显示屏菜单
		Key();
    }	
}

    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off(int voltage)
{
	//禁用电机情况：
	//1、电压低于20V时，没有开启自动回充
	//2、软件使能开关作用
	//3、硬件使能开关作用
	u8 temp;
	static u8 lock_motor=0;
	static u16 lowpower_filter=0;
	
	//电压过低，需要做滤波处理，滤波结果还是低电压则失能电机
	if(voltage<MIN_VOL&&Allow_Recharge==0)
	{
		lowpower_filter++;
		if(lowpower_filter>200) //检测到出现2秒的低电量
		{
			lowpower_filter=0;
			lock_motor=1;//电量低，锁住电机不允许控制
			if(Low_PowerMode==0) beep_flag=1,Low_PowerMode=1;//进入低电量模式，蜂鸣器提醒1次
			Set_SystemError_FLAG(Lower_Power);//低压自检警报
		}
	}
	else
		lowpower_filter=0,lock_motor=0;
	
	//使能开关被按下、或软件失能
	if(EN==0||Flag_Stop==1)
	{
		if(EN==0) Set_SystemError_FLAG(Stop_Switch_DOWN);//急停开关被按下
		temp=1;//电机失能
	}
	else
		temp=0;//电机使能
	
	if( Car_Mode==S200 )//四驱车
	{
		//任意1个驱动掉线、任意1个驱动报错都需要把整车电机失能，不允许控制
		if( Get_Checking_FLAG(Drive1_ERROR) || Get_Checking_FLAG(Drive2_ERROR) )
			temp=1;
		if( Get_Checking_FLAG(Drvie_Timeout) || Get_Checking_FLAG(Drvie2_Timeout) )
			temp=1;
	}

	
	//急停开关弹起
	if(Get_Checking_FLAG(Stop_Switch_DOWN)!=0&&EN==1) Clear_Error_Flag(Stop_Switch_DOWN);
	
	show_motor_state = temp|lock_motor; //获取该变量的值，用于OLED显示
	
	//电机被标记过发送失能函数，此时temp=0恢复电机状态
	if(temp==0&&motor_state)
	{
		CAN_Enable(COBID_Send_1);
		vTaskDelay(1);
		CAN_Enable(COBID_Send_2);
		vTaskDelay(1);
		set_MotorRPM(COBID_Send_1,0,0);//电机速度置0
		vTaskDelay(1);
		set_MotorRPM(COBID_Send_2,0,0);//电机速度置0
		vTaskDelay(1);
		
//		motor_state=0; //调试
		
		//收到两个电机的使能反馈,才结束使能语句的发送
		if( HUB1_EnableState==MOTOR_ENABLE && HUB2_EnableState==MOTOR_ENABLE )
			motor_state=0; //复位状态位
		
	}
	
	//电池电量低不允许控制机器人，同时锁住电机，不失能电机，防止用户使用过程有意外。
	if(lock_motor==1)
	{
		LED=0;//常亮LED
		MOTOR_A.Target=0;
		MOTOR_B.Target=0;
		MOTOR_C.Target=0;
		MOTOR_D.Target=0;
		if(Car_Mode==S200) beep_flag=1;//四驱车无灯带，不间断使用蜂鸣提示
	}
	
	return temp;
						
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_C (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_D (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	{ 
		case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;        break;
		case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/4;    break;
		case 3:      Move_X=0;      		 Move_Z=-PI/4;    break;
		case 4:      Move_X=-RC_Velocity;  	 Move_Z=+PI/4;    break;//差速车Z代表顺(<0)逆(>0)时针旋转
		case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;        break;
		case 6:      Move_X=-RC_Velocity;  	 Move_Z=-PI/4;    break;
		case 7:      Move_X=0;     	 		 Move_Z=+PI/4;    break;
		case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/4;    break; 
		default:     Move_X=0;               Move_Z=0;        break;
	}

	//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	//差速车Z代表顺(<0)逆(>0)时针旋转
	//The greater the forward speed, the greater the rotation speed
	//前进速度越大旋转速度越大
	Move_Z=Move_Z*RC_Velocity/500.0f; 

	//Unit conversion, mm/s -> m/s
	//单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;

	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Z);
}


/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
			
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);
		LX=-(PS2_LY-128);
		RY=-(PS2_RX-128);
	
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0;
		if(LY>-Threshold&&LY<Threshold)LY=0;
		if(RY>-Threshold&&RY<Threshold)RY=0;
		if(LX==0) Move_X=Move_X/1.2f;
		if(RY==0) Move_Z=Move_Z/1.2f;
	
		if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
		else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
		//Handle PS2 controller control commands
		//对PS2手柄控制命令进行处理
		Move_X=LX;
		Move_Y=LY;
		Move_Z=RY;
		Move_X=Move_X*RC_Velocity/128;
		Move_Y=Move_Y*RC_Velocity/128;
		Move_Z=Move_Z*(PI/4)*(RC_Velocity/500)/128;
		
		//Unit conversion, mm/s -> m/s
		//单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;
		Move_Y=Move_Y/1000;
		 
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
	//Data within 1 second after entering the model control mode will not be processed
	//对进入航模控制模式后1秒内的数据不处理
	static u8 thrice=100;
	int Yuzhi=100; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
	u8 mode_flag;//航模模式检测标志位
	//limiter //限幅
	int LX,LY,RY,RX,Remote_RCvelocity; 
	
	//航模遥控控制器检测,在没有进入航模的定时器中断后，清空机器人控制量
	//防止静电产生误识别导致机器人不受控制
	error_judge++;
	if(error_judge>50) error_judge = 0,Remoter_Ch1 = Remoter_Ch2 =\
                 		Remoter_Ch3 = Remoter_Ch4 = 1500;
	
	Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
	Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
	Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
	Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);
	
	//检测用户是否有 进入自动回充模式/避障模式/电机报错清除 请求
	mode_flag = Remote_Choose_mode();
	if(mode_flag) return; //如果进入了航模切换模式的状态，直接返回，不要控制机器人运动
	
	//Front and back direction of left rocker. Control forward and backward.
	//左摇杆前后方向。控制前进后退。
	LX=Remoter_Ch2-1500;
	//The channel is not currently in use
	//该通道暂时没有使用到
	LY=Remoter_Ch4-1500;
	//Right stick left and right. To control the rotation. 
	//右摇杆左右方向。控制自转。
	RY=-(Remoter_Ch1-1500);//自转
	//右摇杆油门上下方向
	RX = Remoter_Ch3-1500;

	if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
	if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
	if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
	if(RX>-Yuzhi&&RX<Yuzhi)RX=0;

	//Throttle related //油门相关
	Remote_RCvelocity=RC_Velocity+RX; // 默认V是500,RX取值-500~500
	if(Remote_RCvelocity<0)Remote_RCvelocity=0;

	//The remote control command of model aircraft is processed
	//对航模遥控控制命令进行处理
	Move_X=LX;
	Move_Z=RY;
	Move_X=Move_X*(float)Remote_RCvelocity/500.0f; //油门÷500代表倍数，油门取值在0~1000 
	Move_Z=(Move_Z/500)*((float)Remote_RCvelocity/500.0f)*(PI/4);

	//Differential car Z stands for clockwise(<0) and counterclockwise(>0) rotation 
	//差速车Z代表顺(<0)逆(>0)时针旋转
	if(Move_X<0)Move_Z=-Move_Z;

	//Unit conversion, mm/s -> m/s
	//单位转换，mm/s -> m/s
	Move_X=Move_X/1000;

	//Data within 1 second after entering the model control mode will not be processed
	//对进入航模控制模式后1秒内的数据不处理
	if(thrice>0) Move_X=0,Move_Z=0,thrice--;

	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=KEY_Scan(100,0);
	if(tmp==single_click)
	{
		#if OLED_DEBUG_MODE
		oled_show_mode++;
		oled_reflash_flag=1;
		if(oled_show_mode==OLED_MAX_PAGE) oled_show_mode=0;
		#endif
	}
	else if(tmp==double_click||BT_Key==1) memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel)),BT_Key=0;
	
	else if(tmp==long_click)
	{
		rgb_lights_showmode = !rgb_lights_showmode;
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	//The encoder converts the raw data to wheel speed in m/s
	//编码器原始数据转换为车轮速度，单位m/s
	if(Car_Mode==S300)
	{
		MOTOR_A.Encoder = MOTOR_A.Encoder_Rpm/60*Wheel_perimeter/10; //除以10是因为读取到的转速单位为0.1rpm
		MOTOR_B.Encoder = MOTOR_B.Encoder_Rpm/60*Wheel_perimeter/10;
	}
	else if(Car_Mode==S150||Car_Mode==S100)
	{
		MOTOR_A.Encoder = -MOTOR_A.Encoder_Rpm/60*Wheel_perimeter/10; //除以10是因为读取到的转速单位为0.1rpm
		MOTOR_B.Encoder = -MOTOR_B.Encoder_Rpm/60*Wheel_perimeter/10;
	}
	else if(Car_Mode==S200)
	{
		MOTOR_A.Encoder =  MOTOR_A.Encoder_Rpm/60*Wheel_perimeter/10; //除以10是因为读取到的转速单位为0.1rpm
		MOTOR_B.Encoder =  MOTOR_B.Encoder_Rpm/60*Wheel_perimeter/10;
		MOTOR_C.Encoder =  MOTOR_C.Encoder_Rpm/60*Wheel_perimeter/10; //除以10是因为读取到的转速单位为0.1rpm
		MOTOR_D.Encoder =  MOTOR_D.Encoder_Rpm/60*Wheel_perimeter/10;
	}
	
	//自动回充装备获取小车控制情况
	u8 tmp[8];
	if(MOTOR_A.Encoder>0) tmp[0] = 1;
	else if(MOTOR_A.Encoder<0) tmp[0] = 2;
	else tmp[0] = 0;
	
	if(MOTOR_B.Encoder>0) tmp[1] = 1;
	else if(MOTOR_B.Encoder<0) tmp[1] = 2;
	else tmp[1] = 0;
	
	//数据在CAN1转发，自动回充装备捕获
	CAN1_Send_Num(0x185,tmp);

}

/**************************************************************************
Function: Smoothing the target velocity
Input   : Target velocity
Output  : none
函数功能：对目标速度做平滑处理
入口参数：目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx, float vz, float step, float step_Vz)
{
	//X轴速度平滑
	if(vx>smooth_control.VX)
	{
		smooth_control.VX+=step;
		if(smooth_control.VX>vx) smooth_control.VX=vx;
	}
	else if (vx<smooth_control.VX)
	{
		smooth_control.VX-=step;
		if(smooth_control.VX<vx) smooth_control.VX=vx;
	}
	else
		 smooth_control.VX =vx;

	//Z轴速度平滑
	if(vz>smooth_control.VZ)
	{
		smooth_control.VZ+=step;
		if(smooth_control.VZ>vz) smooth_control.VZ=vz;
	}
	else if (vz<smooth_control.VZ)
	{
		smooth_control.VZ-=step;
		if(smooth_control.VZ<vz) smooth_control.VZ=vz;
	}
	else
		 smooth_control.VZ =vz;
	
	//0速时保证静止稳定
	if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
	if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
}


#if USE_US_Avoid
void Ultrasonic_avoid_S300(float* vx,float* vz)
{
	static u8 us_filter_count;
	static u8 turn_left=0,turn_right=0;
	
	//检测到障碍物
	if(ultrasonic.A<start_avoid||ultrasonic.B<start_avoid||ultrasonic.C<start_avoid||ultrasonic.D<start_avoid||ultrasonic.E<start_avoid||ultrasonic.F<start_avoid)
	{
		us_filter_count++;
	}
	else
		us_filter_count=0,turn_left=0,turn_right=0;
	
	
	if(us_filter_count>=5)//连续50ms检测到障碍物(10ms控制1次)
	{
		us_filter_count=6;//当有障碍物时锁住变量防止循环
		if(ultrasonic.A+ultrasonic.B+ultrasonic.C<ultrasonic.D+ultrasonic.E+ultrasonic.F)
		{
			turn_right=1;
			turn_left=0;
		}
		else
		{
			turn_right=0,
			turn_left=1;
		}	
	}
	
	//避障
	if(*vx>0)
	{
		if(turn_right)
		{
			*vz = -(*vx/0.5f)*PI/4;
			*vx = 0;
		}
		else if(turn_left)
		{
			*vz = (*vx/0.5f)*PI/4;
			*vx = 0;
		}
	}
	
}

void Ultrasonic_avoid_S150(float* vx,float* vz)
{
	static u8 us_filter_count;
	static u8 turn_left=0,turn_right=0;
	
	//检测到障碍物
	if(ultrasonic.A<start_avoid||ultrasonic.B<start_avoid||ultrasonic.C<start_avoid||ultrasonic.D<start_avoid||ultrasonic.E<start_avoid)
	{
		us_filter_count++;
	}
	else
		us_filter_count=0,turn_left=0,turn_right=0;
	
	
	if(us_filter_count>=5)//连续50ms检测到障碍物(10ms控制1次)
	{
		us_filter_count=6;//当有障碍物时锁住变量防止循环
		if(ultrasonic.A+ultrasonic.B<ultrasonic.D+ultrasonic.E)
		{
			turn_right=1;
			turn_left=0;
		}
		else
		{
			turn_right=0,
			turn_left=1;
		}	
	}
	
	//避障
	if(*vx>0)
	{
		if(turn_right)
		{
			*vz = -(*vx/0.5f)*PI/4;
			*vx = 0;
		}
		else if(turn_left)
		{
			*vz = (*vx/0.5f)*PI/4;
			*vx = 0;
		}
	}

}

//S100避障：超声波为用户加装，安装位置 D前左 E前右 C左侧 F右侧 B后左 A后右
//与S300/S150不同，这里只用前向和侧向探头决策，后方A/B不参与
//(避障只在前进时介入，后退的防护由Ultrasonic_safeguard负责)
void Ultrasonic_avoid_S100(float* vx,float* vz)
{
	static u8 us_filter_count;
	static u8 turn_left=0,turn_right=0;
	//侧向阈值单独设置：侧面探头贴着墙走是常态，用start_avoid会导致走廊里反复原地打转
	const float side_avoid = 0.20f;

	//检测到障碍物
	if(ultrasonic.D<start_avoid || ultrasonic.E<start_avoid ||
	   ultrasonic.C<side_avoid  || ultrasonic.F<side_avoid)
	{
		us_filter_count++;
	}
	else
		us_filter_count=0,turn_left=0,turn_right=0;

	if(us_filter_count>=5)//连续50ms检测到障碍物(10ms控制1次)
	{
		us_filter_count=6;//当有障碍物时锁住变量防止循环

		//先看侧向：某一侧贴太近就往另一侧让，避免转向时把车尾/车身刮上去
		if(ultrasonic.C<side_avoid && ultrasonic.F>=side_avoid)
			turn_right=1,turn_left=0;   //左侧贴墙，右转让开
		else if(ultrasonic.F<side_avoid && ultrasonic.C>=side_avoid)
			turn_right=0,turn_left=1;   //右侧贴墙，左转让开
		//两侧都近(走廊)或两侧都不近，则按前方左右哪边更近来决定
		else if(ultrasonic.D<ultrasonic.E)
			turn_right=1,turn_left=0;   //前左更近，右转
		else
			turn_right=0,turn_left=1;   //前右更近(或相等)，左转
	}

	//避障：Vz为正表示左转(见Drive_Motor中的运动学逆解)
	//注意先用*vx算出*vz再把*vx清零，顺序不能反
	if(*vx>0)
	{
		if(turn_right)
		{
			*vz = -(*vx/0.5f)*PI/4;
			*vx = 0;
		}
		else if(turn_left)
		{
			*vz = (*vx/0.5f)*PI/4;
			*vx = 0;
		}
	}
}
#endif

// 底线防撞保护和避障功能
void Ultrasonic_safeguard(float* vx,float* vz)
{
	static u8 us_forward_count;
	static u8 us_backword_count;
	static u8 turn_left=0,turn_right=0;
	static float safe_distance = 0.3f;  //前后距离少于30cm时触发保护
	static float safe_side_distance = 0.1f;  //左右距离小于10cm时触发保护

	//超声波通道与安装位置对应：D前左 E前右 C左侧 F右侧 B后左 A后右
	//检测前方到障碍物
	if(ultrasonic.C < safe_side_distance || ultrasonic.D < safe_distance||
		ultrasonic.E < safe_distance || ultrasonic.F < safe_side_distance)
	{
		us_forward_count++;
	}
	else
		us_forward_count=0;

	if(ultrasonic.A < safe_distance || ultrasonic.B < safe_distance)
	{
		us_backword_count++;
	}
	else
		us_backword_count=0;

	
	if(us_forward_count>=5)//连续50ms检测到障碍物(10ms控制1次)
	{
		us_forward_count=6;//当有障碍物时锁住变量防止循环
		if ( *vx > 0 )
		{   
			*vx = 0;  // 不允许前进
		}
	}

	if(us_backword_count>=5)//连续50ms检测到障碍物(10ms控制1次)
	{
		us_backword_count=6;//当有障碍物时锁住变量防止循环
		if ( *vx < 0 )
		{   
			*vx = 0;  // 不允许后退
		}
	}

}


//通过航模来切换控制状态
u8 Remote_Choose_mode(void)
{
	/* 
	          航模摇杆与通道的对应关系以及取值范围
	           CH2 2000           CH3 2000
	              |                  |
	              |                  |
	CH4 1000 ————————————       ————————————— CH1 2000
	              |                  |
	              |                  |
	            1000               1000
	*/
	
	#if USE_US_Avoid
	//是否进入底盘避障模式变量检测
	static u8 open_us_avoid_filter=0;//计时变量
	static u8 avoid_once=1;//在识别稳定的前提下保证1次操作只触发1次
	const u8 open_us_avoid_time=80; //80ms实际情况航模识别不稳定，不止需要80ms才能触发
	#endif
	//是否开启自动回充变量检测
	static u8 open_auto_rc_filter=0;//计时变量
	const u8 open_auto_rc_time=80; //80ms 实际情况航模识别不稳定，不止需要80ms才能触发
	static u8 auto_rc_once=1;//在识别稳定的前提下保证1次操作只触发1次
	//是否进入电机报错清除模式
	static u8 clear_error_filter=0;//计时变量
	const u8 clear_error_time=80;//80ms 实际情况航模识别不稳定，不止需要80ms才能触发
	static u8 clear_once=1;//在识别稳定的前提下保证1次操作只触发1次
	
	if(rm_stop_scan) auto_rc_once=0;//自动回充切换回非自动回充后，防止用户一直外八摇杆导致重新进入自动回充模式
	
	//摇杆外八,左摇杆往左下角，右摇杆往右下角，航模开启自动回充模式
	if((Remoter_Ch1>1950&&Remoter_Ch1<2005)&&(Remoter_Ch2>1000&&Remoter_Ch2<1050)&&(Remoter_Ch3>1000&&Remoter_Ch3<1050)&&(Remoter_Ch4>1000&&Remoter_Ch4<1050))
	{
		if(auto_rc_once==1)
		{
			open_auto_rc_filter++;
			if(open_auto_rc_filter>=open_auto_rc_time)
			{
				open_auto_rc_filter=0;
				Allow_Recharge=1;
				recharge_flag_beep=1;
				auto_rc_once=0;
			}
		}
		Drive_Motor(0,0);
		return 1;//返回防止机器人被控制
	}
	else
		open_auto_rc_filter=0,auto_rc_once=1,rm_stop_scan=0;
	
	#if USE_US_Avoid
	if(Car_Mode==S300||Car_Mode==S150||Car_Mode==S100) //S150/S300/S100才可以开启底盘避障功能
	{
		//摇杆内八，左摇杆往右下角，右摇杆往左下角，航模开启底盘避障功能
		if((Remoter_Ch1>1000&&Remoter_Ch1<1050)&&(Remoter_Ch2>1000&&Remoter_Ch2<1050)&&(Remoter_Ch3>1000&&Remoter_Ch3<1050)&&(Remoter_Ch4>1950&&Remoter_Ch4<2005)) 
		{
			if(avoid_once==1)
			{
				if(Get_Control_Mode(_ROS_Control)==0&&Get_Control_Mode(_USART_Control)==0&&Get_Control_Mode(_CAN_Control)==0)
				{
					open_us_avoid_filter++;//非 ros/串口/CAN 控制允许启动底盘避障
				}
				
				if(open_us_avoid_filter>=open_us_avoid_time)
				{
					avoid_flag_beep=1;//蜂鸣器提示进入底盘避障模式
					open_us_avoid_filter=0;
					Open_US_avoid = !Open_US_avoid;
					avoid_once=0;
				}
			}
			Drive_Motor(0,0);
			return 1;//返回防止机器人被控制
		}
		else
			open_us_avoid_filter=0,avoid_once=1;
	}

	#endif
	
	//摇杆往两边掰，左摇杆往左，右摇杆往右，清除电机报错
	if((Remoter_Ch1>1995&&Remoter_Ch1<2005)&&(Remoter_Ch2>1495&&Remoter_Ch2<1505)&&(Remoter_Ch3>1300&&Remoter_Ch3<1700)&&(Remoter_Ch4>995&&Remoter_Ch4<1005))
	{
		if(clear_once==1)
		{
			clear_error_filter++;
			if(clear_error_filter>=clear_error_time)
			{
				clear_error_filter=0;
				motor_clear_error = 1;
				clear_once=0;
			}
		}
		Drive_Motor(0,0);
		return 1;//返回防止机器人被控制
	}
	else
		clear_error_filter=0,clear_once=1;
	
	return 0;
}

//机器人溜车/驱动器异常检查
void robot_check(void)
{
	static float left_error,right_error;
	static u8 stop_once=1;
	
	if(Car_Mode==S300||Car_Mode==S100||Car_Mode==S150)
	{
		if(MOTOR_A.Target==0&&MOTOR_B.Target==0&&show_motor_state==0)
		{
			left_error += MOTOR_A.Encoder*0.01f;
			right_error+= MOTOR_B.Encoder*0.01f;
			if(stop_once&&(left_error>0.02f||left_error<-0.02f||right_error>0.02f||right_error<-0.02f))
			{
				stop_once=0;
				CAN_Enable(COBID_Send_1);
			}
			
		}
		else
		{
			left_error=0,right_error=0,stop_once=1;
		}
	}

}


//PWM消除函数
void auto_pwm_clear(void)
{
	//========== PWM清除使用变量 ==========//
	static u8 start_check_flag = 0;//标记是否需要清空PWM
	static u16 wait_clear_times = 0;
	static volatile u8 start_clear = 0;     //标记开始清除PWM
	static u8 clear_done_once = 0; //清除完成标志位
	static u16 clear_again_times = 0;
	static volatile u8 clear_state = 0x00;
	/*------------------------------------*/
	
	//小车姿态简易判断
	float y_accle = (float)(accel[1]/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(accel[2]/1671.84f);//Z轴加速度实际值
	float diff;
	
	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	static u8 error_lock=0;
	if( diff > 8.8f )
	{
		if( error_lock==0 )
		{
			error_lock=1;
			vTaskDelay(1);
			set_error_state(COBID_Send_1,0);
			if(Car_Mode==S200)
			{
				vTaskDelay(1);
				set_error_state(COBID_Send_2,0);
			}
		}
	}
	else
	{
		if( error_lock==1 ) 
		{
			error_lock=0;
			//机器人在斜坡上,将驱动报警状态设置为锁轴
			vTaskDelay(1);
			set_error_state(COBID_Send_1,1);
			if(Car_Mode==S200)
			{
				vTaskDelay(1);
				set_error_state(COBID_Send_2,1);
			}
		}
	}
	
	//PWM消除检测
	if( MOTOR_A.Target !=0.0f || MOTOR_B.Target != 0.0f || MOTOR_C.Target != 0.0f ||MOTOR_D.Target != 0.0f)
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		
		if(clear_done_once==1)
		{
			vTaskDelay(1);
			robot_ParkMode(COBID_Send_1,0);//退出驻车模式
			if(Car_Mode==S200) vTaskDelay(1),robot_ParkMode(COBID_Send_2,0);//退出驻车模式
		}
		
		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 10 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 1000 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			float cur_warning = 0.5f;
			//完成清除后pwm再次积累，重新清除
			if( float_abs(Back_Drive.L_motorCurrent)>cur_warning || float_abs(Back_Drive.R_motorCurrent)>cur_warning || float_abs(Front_Drive.L_motorCurrent)>cur_warning || float_abs(Front_Drive.R_motorCurrent)>cur_warning )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
	
	//执行1次清除
	if( start_clear )
	{
		robot_ParkMode(COBID_Send_1,1);//启用驻车模式
		if(Car_Mode==S200) robot_ParkMode(COBID_Send_2,1);//启用驻车模式
		start_clear=0,
		clear_done_once=1;
	}		
}


