#include "show.h"

int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;

u8 Low_PowerMode=0;//低电量模式
u8 LowPower_FilterCount;//低电量滤波

u8 beep_flag=1; 
u8 beep_time;//蜂鸣器蜂鸣时间
u8 show_motor_state; //电机状态
u8 motor_beep=0;

//开启DeBug模式定义的变量
#if OLED_DEBUG_MODE
u8 checkauto_data;
u8 oled_show_mode=4;//default:4
u8 oled_reflash_flag=0;
float pos_x,pos_y,pos_z;
void imu_debug_show(void);
void US_Mr_Auto_DebugShow(void);
void car_v_p_show(void);
void hub_state_show(void);
void mow_motor_show(void);
#endif

u8 recharge_flag_beep=0;//自动回充蜂鸣提示

u8 bb_flag=0;

#if USE_US_Avoid
u8 avoid_flag_beep=0;

#endif
u8 turn_off_beep;//关闭蜂鸣器

u8 APP_Debug=0;//APP聊天界面输出调试信息
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void show_task(void *pvParameters)
{
//	const u8 used_time=20; //用于本任务使用到的计时滤波，如果修改本任务频率，该数值需要对应修改
						   //10Hz任务->100ms运行1次； 变量20 -> 2s
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{	
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //此任务以10Hz的频率运行
		
		///////////////////////// 蜂鸣器show ///////////////////////
		//开机或低电量时,蜂鸣器响1秒
		u8 wait_time=10;//蜂鸣默认值1000ms
		if(beep_flag)
		{
			beep_time++;//蜂鸣器走时
			beep=1;
			if(motor_beep==1) wait_time=3;//清除电机报错，蜂鸣300ms
			if(beep_time>=wait_time)//到达规定的时间，停止发声
			{
				beep_time=0;
				beep=0;
				beep_flag=0;
				motor_beep=0;
			}
		}
		
		#if USE_US_Avoid //S300、S150机器人才有避障模式
		if(avoid_flag_beep==1)//避障模式响3声
		{
			bb_flag=1;
			turn_off_beep++;
			if(turn_off_beep>=11) turn_off_beep=0,avoid_flag_beep=0,bb_flag=0;
		}
		#endif
		
		if(recharge_flag_beep==1)//自动回充蜂鸣2声
		{
			bb_flag=1;
			turn_off_beep++;
			if(turn_off_beep>=9) turn_off_beep=0,recharge_flag_beep=0,bb_flag=0;
		}
		beep_beep(bb_flag);//bb_flag=1的时间越长，蜂鸣器提示的次数越长
		///////////////////////// 蜂鸣器show ///////////////////////
		
		
		/////////////////////// 电压情况show ///////////////////////
		//Read the battery voltage //读取电池电压
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage = VolMean_Filter(Voltage);
		Voltage_All=0;
		
		if(Low_PowerMode&&Voltage>21.5f) Low_PowerMode = 0,Clear_Error_Flag(Lower_Power); //进入低电量模式后，检测到电压回升则退出低电量模式
		/////////////////////// 电压情况show ///////////////////////
		
		/////////////////////// APP show ///////////////////////
		if(APP_Debug==1)
		{
			APP_Debug_Show();
		}
		else
		{
			//Send data to the APP //向APP发送数据
			APP_Show();	 
		}

		/////////////////////// APP show ///////////////////////
		
		/////////////////////// OLED show ///////////////////////
		//Tasks are displayed on the screen //显示屏显示任务
		#if OLED_DEBUG_MODE
		if(oled_reflash_flag==1) OLED_Clear(),oled_reflash_flag=0;//切换菜单
		else
		#endif
		oled_show(); 
		/////////////////////// OLED show ///////////////////////
	}
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	#if OLED_DEBUG_MODE  //DEBUG模式显示的内容
	if(oled_show_mode==0)  //imu debug
	{
		imu_debug_show();
	}
	else if(oled_show_mode==1)// 超声波 、自动回充通信debug
	{
		US_Mr_Auto_DebugShow();
	}
	else if(oled_show_mode==2) //机器人速度、里程计情况debug
	{
		car_v_p_show();
	}
	else if (oled_show_mode==3)
	{
		hub_state_show();
	}
	
	else if (oled_show_mode==4)
	{
	#endif
		int Car_Mode_Show;

		//Collect the tap information of the potentiometer, 
		//and display the car model to be fitted when the car starts up in real time
		//采集电位器档位信息，实时显示小车开机时要适配的小车型号
		Car_Mode_Show=(int) (Get_Adc(CAR_MODE_ADC)/Divisor_Mode);

		Voltage_Show=Voltage*100;

		//The first line of the display displays the content//
		//显示屏第1行显示内容//	
			 if(Car_Mode_Show==S300) OLED_ShowString(0,0,"S300");
		else if(Car_Mode_Show==S200) OLED_ShowString(0,0,"S200");
		else if(Car_Mode_Show==S150) OLED_ShowString(0,0,"S150");
		else if(Car_Mode_Show==S100) OLED_ShowString(0,0,"S100");
		else if(Car_Mode_Show==SX03) OLED_ShowString(0,0,"SX03");
		else if(Car_Mode_Show==SX04) OLED_ShowString(0,0,"SX04");
		else OLED_ShowString(0,0,"XXXX");
		
		//充电电流显示
		OLED_ShowString(55,0,"Cur:");
		oled_showfloat(Charging_Current/1000.0f,87,0,1,2);
		OLED_ShowString(120,0,"A");
		
		if(Car_Mode==S300||Car_Mode==S150||Car_Mode==S100)
		{
			//The second line of the display displays the content//
			//显示屏第2行显示内容//	
			//Display Z-axis angular velocity //显示Z轴角速度
			OLED_ShowString(00,10,"GYRO_Z"); 
			if( gyro[2]<0)   OLED_ShowString(65,10,"-"),
			OLED_ShowNumber(75,10,-gyro[2],5,12);
			else             OLED_ShowString(65,10,"+"),
			OLED_ShowNumber(75,10, gyro[2],5,12);		
			
			//The third line of the display displays the content//
			//显示屏第3行显示内容//
			//Display the target speed and current speed of motor A
			//显示电机A的目标速度和当前速度	 
			OLED_ShowString(0,20,"L:");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
			OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
			OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12); 

			if( MOTOR_A.Encoder<0)OLED_ShowString(60,20,"-"),
			OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
			OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);

			//The fourth line of the display displays the content//
			//显示屏第4行显示内容//	
			//Display the target speed and current speed of motor B
			//显示电机B的目标速度和当前速度
			OLED_ShowString(0,30,"R:");
			if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
			OLED_ShowNumber(20,30,- MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
			OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12); 

			if( MOTOR_B.Encoder<0)OLED_ShowString(60,30,"-"),
			OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
			OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

			//Line 5 of the display displays the content//
			//显示屏第5行显示内容//
			//Display the current PWM value of A and B motors
			//显示A、B电机当前的PWM值
			OLED_ShowString(00,40,"RPM");
			if( MOTOR_A.Encoder_Rpm<0)OLED_ShowString(30,40,"-"),
			OLED_ShowNumber(50,40,-MOTOR_A.Encoder_Rpm,4,12);
			else                 	    OLED_ShowString(30,40,"+"),
			OLED_ShowNumber(50,40, MOTOR_A.Encoder_Rpm,4,12); 

			if(MOTOR_B.Encoder_Rpm<0) OLED_ShowString(80,40,"-"),
			OLED_ShowNumber(100,40,-MOTOR_B.Encoder_Rpm,4,12);
			else                 	    OLED_ShowString(80,40,"+"),
			OLED_ShowNumber(100,40, MOTOR_B.Encoder_Rpm,4,12);
		}
		else if(Car_Mode==S200)
		{
			OLED_ShowString(0,10,"A:");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
			OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
			OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 

			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
			OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
			OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);

			OLED_ShowString(0,20,"B:");
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
			OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
			OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 

			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
			OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
			OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);	
			
			OLED_ShowString(0,30,"C:");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
			OLED_ShowNumber(20,30,-MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
			OLED_ShowNumber(20,30, MOTOR_C.Target*1000,5,12); 

			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
			OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
			OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);	
			
			OLED_ShowString(0,40,"D:");
			if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
			OLED_ShowNumber(20,40,-MOTOR_D.Target*1000,5,12);
			else                 	OLED_ShowString(15,40,"+"),
			OLED_ShowNumber(20,40, MOTOR_D.Target*1000,5,12); 

			if( MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
			OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,40,"+"),
			OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);	
			
		}

		
		//显示屏第6行显示内容
		//Line 6 of the display displays the contents
		//Displays the current control mode //显示当前控制模式
		if(Allow_Recharge)  OLED_ShowString(0,50,"RCM  ");
		else
		{
			if      (Get_Control_Mode( _PS2_Control )) OLED_ShowString(0,50,"PS2  ");
			else if (Get_Control_Mode( _APP_Control )) OLED_ShowString(0,50,"APP  ");
			else if (Get_Control_Mode(  _RC_Control )) OLED_ShowString(0,50,"R-C  ");
			else if (Get_Control_Mode( _CAN_Control )) OLED_ShowString(0,50,"CAN  ");
			else if (Get_Control_Mode(_USART_Control)) OLED_ShowString(0,50,"USART");
			else if (Get_Control_Mode( _ROS_Control )) OLED_ShowString(0,50,"ROS ");
		}
		//Displays whether controls are allowed in the current car
		//显示当前小车是否允许控制	
		if(show_motor_state)   OLED_ShowString(45,50,"OFF"); 
		else                      OLED_ShowString(45,50," ON");

		//Displays the current battery voltage
		//显示当前电池电压	
		OLED_ShowString(88,50,".");
		OLED_ShowString(110,50,"V");
		OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
		OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
		if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);

		//Refresh the screen //刷新屏幕
		OLED_Refresh_Gram();	
	
	#if OLED_DEBUG_MODE
	}
	#endif
}

/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
#include "stdarg.h"
uint8_t APPChat_Buff[256];     //串口x发送缓冲区
void APP_Chat(char *format,...)
{	
	uint8_t i;                                           //用于for循环
	
	va_list listdata;                                     //建立一个va_list变量listdata
	va_start(listdata,format);                            //向listdata加载...代表的不定长的参数
	vsprintf((char *)APPChat_Buff,format,listdata);          //格式化输出到缓冲区U0_TxBuff
	va_end(listdata);                                     //释放listdata
	
	while((USART2->SR&0x40)==0);
	USART2->DR = '{';
	while((USART2->SR&0x40)==0);
	USART2->DR = '#';
	
	for(i=0;i<strlen((const char*)APPChat_Buff);i++){        //根据U0_TxBuff缓冲区数据量，一个字节一个字节的循环发送
		while((USART2->SR&0x40)==0);
		USART2->DR = APPChat_Buff[i];
			
	}
	
	while((USART2->SR&0x40)==0);
	USART2->DR = '}';
	while((USART2->SR&0x40)==0);
	USART2->DR = '$';
	while((USART2->SR&0x40)==0);	     //等到最后一个字节数据发送完毕，再退出函数
}

void APP_Debug_Show(void)
{
	static u8 showcount=0;
	char showcar[10];
	char showMode[5];
	char showstate[3];
	char hubstate[3];
	
	//车型赋值
	memset(showcar,'\0',sizeof(showcar));
		 if(Car_Mode==S300) strcpy(showcar,"S300");
	else if(Car_Mode==S200) strcpy(showcar,"S200");
	else if(Car_Mode==S150) strcpy(showcar,"S300M");
	else if(Car_Mode==S100) strcpy(showcar,"S100");
	else if(Car_Mode==SX03) strcpy(showcar,"SX03");
	else if(Car_Mode==SX04) strcpy(showcar,"SX04");
	else strcpy(showcar,"unkown");
	
	//使能赋值
	memset(showstate,'\0',sizeof(showstate));
	if(show_motor_state) strcpy(showstate,"OFF");
	else strcpy(showstate,"ON");
	
	//驱动器是否报错
	memset(hubstate,'\0',sizeof(hubstate));
	if( Get_Checking_FLAG(Drive1_ERROR) || Get_Checking_FLAG(Drive2_ERROR) ) strcpy(hubstate,"YES");
	else strcpy(hubstate,"NO");
	
	//控制模式赋值
	memset(showMode,'\0',sizeof(showMode));
	if(Allow_Recharge) strcpy(showMode,"RCM");
	else
	{
		if      (Get_Control_Mode( _PS2_Control )) strcpy(showMode,"PS2");
		else if (Get_Control_Mode( _APP_Control )) strcpy(showMode,"APP");
		else if (Get_Control_Mode(  _RC_Control )) strcpy(showMode,"R-C");
		else if (Get_Control_Mode( _CAN_Control )) strcpy(showMode,"CAN");
		else if (Get_Control_Mode(_USART_Control)) strcpy(showMode,"USART");
		else if (Get_Control_Mode( _ROS_Control )) strcpy(showMode,"ROS");
	}
	
	vTaskDelay(1000);
	showcount++;
	if( showcount==1 )
		APP_Chat("Type:%s",showcar);//打印车型
	else if( showcount==2 )
		APP_Chat("L:%.2f",MOTOR_A.Encoder);//打印左轮信息
	else if( showcount==3 )
		APP_Chat("R:%.2f",MOTOR_B.Encoder);//打印右轮信息
	else if( showcount==4 )
		APP_Chat("Mode:%s",showMode);//打印控制模式
	else if( showcount==5 )
		APP_Chat("Vol:%.2fV",Voltage);//打印电池电压
	else if( showcount==6 )
		APP_Chat("EN:%s",showstate);//打印小车是否被使能
	else if( showcount==7 )
		APP_Chat("H ERR:%s",hubstate);//打印小车是否被使能
	else if( showcount==8 )
		APP_Chat("########");
	else if( showcount==12 )
		showcount=0;
}


void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*100-2000)*5/26;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	 
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
	 if(send_error_app==0)
	 {
		 if(PID_Send==1)
		 {
			 //Send parameters to the APP, the APP is displayed in the debug screen
			 //发送参数到APP，APP在调试界面显示
			 printf("{C%d:%d:%d:%d:%d:%d}$",
			 (int)RC_Velocity,
				 
			 #if USE_US_Avoid
			 (int)(start_avoid*100),
			 #else
			 0,
			 #endif
			 
			 #if USE_RGB_lights
			 rgb_r,
			 rgb_b,
			 rgb_b,
			 #else
			  0,
			 0,
			 0,
			 #endif
			 mow_motor

			);
			 
			 PID_Send=0;
			 get_error_flag=1;			 
		 }	
		 else	if(flag_show==0)
		 {
			 //Send parameters to the APP and the APP will be displayed on the front page
			 //发送参数到APP，APP在首页显示
		   printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]); 
		 }
		 else
		 {
			 //Send parameters to the APP, the APP is displayed in the waveform interface
			 //发送参数到APP，APP在波形界面显示
		   printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);
		 }
	 }

}

//蜂鸣器bb响
void beep_beep(u8 flag)
{
	static u8 times_core;
	static u8 last_flag;
	
	if(last_flag==1&&flag==0)
	{
		beep=0;
	}
	
	if(flag) times_core++;//放置于100ms任务中
	else times_core=0;
	
	last_flag = flag;
	
	if(flag)
	{
		if(times_core<2) beep=1;
		if(times_core>2&&times_core<4) beep=0;
		if(times_core>4&&times_core<6) beep=1;
		if(times_core>6&&times_core<8) beep=0;
		if(times_core==8) times_core=0;
	}
}

extern u16 Drive_Version;
extern u16 io_mode;

#if OLED_DEBUG_MODE
void imu_debug_show(void)
{
	#if 1
	OLED_ShowString(00,00,"X:"); 
	if( accel[0]<0) OLED_ShowString(15,00,"-"),OLED_ShowNumber(30,00, -((int)accel[0]),5,12);  
	else           OLED_ShowString(15,00,"+"),OLED_ShowNumber(30,00, ((int)accel[0]),5,12);

	if( gyro[0]<0) OLED_ShowString(75,00,"-"),OLED_ShowNumber(85,00, -((int)gyro[0]),5,12);  
	else           OLED_ShowString(75,00,"+"),OLED_ShowNumber(85,00, ((int)gyro[0]),5,12);

	OLED_ShowString(00,10,"Y:"); 
	if( accel[1]<0) OLED_ShowString(15,10,"-"),OLED_ShowNumber(30,10,-((int)accel[1]),5,12);  
	else           OLED_ShowString(15,10,"+"),OLED_ShowNumber(30,10,((int)accel[1]),5,12);

	if( gyro[1]<0) OLED_ShowString(75,10,"-"),OLED_ShowNumber(85,10, -((int)gyro[1]),5,12);  
	else           OLED_ShowString(75,10,"+"),OLED_ShowNumber(85,10, ((int)gyro[1]),5,12);

	OLED_ShowString(00,20,"Z:"); 
	if( accel[2]<0) OLED_ShowString(15,20,"-"),OLED_ShowNumber(30,20, -((int)accel[2]),5,12);  
	else           OLED_ShowString(15,20,"+"),OLED_ShowNumber(30,20, ((int)accel[2]),5,12);

	if( gyro[2]<0) OLED_ShowString(75,20,"-"),OLED_ShowNumber(85,20, -((int)gyro[2]),5,12);  
	else           OLED_ShowString(75,20,"+"),OLED_ShowNumber(85,20, ((int)gyro[2]),5,12);
	
	//显示驱动版本号
	OLED_ShowString(0,40,"Ver:");
	OLED_ShowNumber(55,40,Back_Drive.SoftwareVersion,5,12);

//	OLED_ShowString(0,30,"X");
//	OLED_ShowString(0,40,"Y");
//	OLED_ShowString(0,50,"Z");
//	oled_showfloat((float)accel[0]/1671.84f,20,30,2,2);
//	oled_showfloat((float)accel[1]/1671.84f,20,40,2,2);
//	oled_showfloat((float)accel[2]/1671.84f,20,50,2,2);

//	oled_showfloat((float)gyro[0]/16.384f,70,30,2,2);
//	oled_showfloat((float)gyro[1]/16.384f,70,40,2,2);
//	oled_showfloat((float)gyro[2]/16.384f,70,50,2,2);

	OLED_Refresh_Gram();
	#else
	OLED_ShowNumber(0,0,Drive_Version,10,12);
	OLED_ShowNumber(0,20,io_mode,10,12);
//	OLED_ShowString(0,0,"pwm");
//	OLED_ShowNumber(60,0,mow_motor,4,12);
//	
//	OLED_ShowString(0,20,"dir");
//	OLED_ShowNumber(60,20,mow_dir,1,12);
	
	OLED_Refresh_Gram();
	#endif
}

void US_Mr_Auto_DebugShow(void)
{
	//超声波数据检查
	oled_showfloat(ultrasonic.A,10,0,1,3);
	oled_showfloat(ultrasonic.B,10,10,1,3);
	oled_showfloat(ultrasonic.C,10,20,1,3);
	oled_showfloat(ultrasonic.D,10,30,1,3);
	oled_showfloat(ultrasonic.E,10,40,1,3);
	oled_showfloat(ultrasonic.F,10,50,1,3);
	OLED_ShowString(0,0,"A:");
	OLED_ShowString(0,10,"B:");
	OLED_ShowString(0,20,"C:");
	OLED_ShowString(0,30,"D:");
	OLED_ShowString(0,40,"E:");
	OLED_ShowString(0,50,"F:");
	
	//自动回充电流检查
	oled_showfloat(Charging_Current/1000.0f,60,0,1,2);
	
	//自动回充电压检查
	oled_showfloat(Recharge_VOL,60,10,2,2);
	
	//航模遥控数据检查
	OLED_ShowNumber(112,0,error_judge,2,12);
	OLED_ShowNumber(60,20,Remoter_Ch1,4,12);
	OLED_ShowNumber(60,30,Remoter_Ch2,4,12);
	OLED_ShowNumber(60,40,Remoter_Ch3,4,12);
	OLED_ShowNumber(60,50,Remoter_Ch4,4,12);
	
	//RGB灯带数值
//	OLED_ShowNumber(88,20,rgb_set[1],3,12);
//	OLED_ShowNumber(88,30,rgb_set[2],3,12);
//	OLED_ShowNumber(88,40,rgb_set[3],3,12);
	
	//自动回充目标速度
	OLED_ShowNumber(112,20,red_ignore,1,12);
	oled_showfloat(Recharge_Red_Move_X,96,30,1,2);
	oled_showfloat(Recharge_Red_Move_Z,96,40,1,2);

	OLED_ShowNumber(96,50,L_A,1,12);
	OLED_ShowNumber(104,50,L_B,1,12);
	OLED_ShowNumber(112,50,R_B,1,12);
	OLED_ShowNumber(120,50,R_A,1,12);
	
	
	OLED_Refresh_Gram();
}

void car_v_p_show(void)
{
	//左轮、右轮目标速度
	OLED_ShowString(0,00,"L:");
	OLED_ShowString(0,10,"R:");
	oled_showfloat(MOTOR_A.Target,20,0,2,2);
	oled_showfloat(MOTOR_A.Encoder,80,0,2,2);
	
	oled_showfloat(MOTOR_B.Target,20,10,2,2);
	oled_showfloat(MOTOR_B.Encoder,80,10,2,2);
	
	//X轴速度、位置
	OLED_ShowString(0,20,"X:");
	oled_showfloat((MOTOR_A.Encoder+MOTOR_B.Encoder)/2,20,20,1,2);
	oled_showfloat(pos_x,80,20,2,2);

	//Y轴速度、位置
	OLED_ShowString(0,30,"Y:");
	oled_showfloat(0,20,30,1,2);
	oled_showfloat(pos_y,80,30,2,2);
	
	//Z轴速度、位置
	OLED_ShowString(0,40,"Z:");
	oled_showfloat((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing,20,40,1,2);
	oled_showfloat(pos_z,80,40,2,2);	
	
	#if USE_US_Avoid
	//是否开启了底层超声波避障
	OLED_ShowString(0,50,"Avoid:");
	if(Open_US_avoid) OLED_ShowString(50,50," ON");
	else OLED_ShowString(50,50,"OFF");
	oled_showfloat(start_avoid,80,50,1,1);
	#endif
	
	OLED_Refresh_Gram();
}

void hub_state_show(void)
{
	OLED_ShowString(0,0,"A:");
	oled_showfloat(Back_Drive.L_motorCurrent,12,0,2,1);
	oled_showfloat(Back_Drive.L_motorTemperature,70,0,2,2);
	
	OLED_ShowString(0,10,"B:");
	oled_showfloat(Front_Drive.L_motorCurrent,12,10,2,1);
	oled_showfloat(Front_Drive.L_motorTemperature,70,10,2,2);
	
	OLED_ShowString(0,20,"C:");
	oled_showfloat(Front_Drive.R_motorCurrent,12,20,2,1);
	oled_showfloat(Front_Drive.R_motorTemperature,70,20,2,2);
	
	OLED_ShowString(0,30,"D:");
	oled_showfloat(Back_Drive.R_motorCurrent,12,30,2,1);
	oled_showfloat(Back_Drive.R_motorTemperature,70,30,2,2);
	
	OLED_ShowNumber(0,40,HUB1_EnableState,1,12);
	OLED_ShowNumber(0,50,HUB2_EnableState,1,12);
//	
//	OLED_ShowNumber(15,40,HUB1_TEST_L,5,12);
//	OLED_ShowNumber(70,40,HUB1_TEST_R,5,12);
//	OLED_ShowNumber(15,50,HUB2_TEST_L,5,12);
//	OLED_ShowNumber(70,50,HUB2_TEST_R,5,12);
	
	OLED_ShowNumber(70,40,can1_send_error,3,12);
	OLED_ShowNumber(70,50,can2_send_error,3,12);
	
//	OLED_ShowString(0,0,"LCur:");
//	oled_showfloat(HUB_Drive.L_motorCurrent,60,0,2,1);
//	
//	OLED_ShowString(00,10,"LTem:");
//	oled_showfloat(HUB_Drive.L_motorTemperature,60,10,3,2);
//	
//	OLED_ShowString(0,20,"RCur:");
//	oled_showfloat(HUB_Drive.R_motorCurrent,60,20,2,1);
//	
//	OLED_ShowString(0,30,"RTem:");
//	oled_showfloat(HUB_Drive.R_motorTemperature,60,30,3,2);
//	
//	OLED_ShowString(0,40,"HUBT:");
//	oled_showfloat(HUB_Drive.R_motorTemperature,60,40,3,2);
//	
//	OLED_ShowString(0,50,"VOL:");
//	oled_showfloat(HUB_Drive.Voltage,60,50,3,2);
//	
	OLED_Refresh_Gram();
}

void mow_motor_show(void)
{

	OLED_ShowString(0,0,"pwm");
	OLED_ShowNumber(60,0,mow_motor,4,12);
	
	OLED_ShowString(0,20,"dir");
	OLED_ShowNumber(60,20,mow_dir,1,12);
	
	OLED_Refresh_Gram();

}

#endif

#define VOL_COUNT 100
float base_vol = 22.5f;
float VolMean_Filter(float data)
{
    u8 i;
    double Sum_Speed = 0;
    float Filter_Speed;
    static  float Speed_Buf[VOL_COUNT]= {0};
	
	/*----------- 数组初始化 -----------*/
	static u8 once=1;
	if(once)
	{
		once=0;
		for(i=0;i<VOL_COUNT;i++)
			Speed_Buf[i]=base_vol;
	}
	/*-------------------------------*/
	
    for(i = 1 ; i<VOL_COUNT; i++)
    {
        Speed_Buf[i - 1] = Speed_Buf[i];
    }
    Speed_Buf[VOL_COUNT - 1] =data;

    for(i = 0 ; i < VOL_COUNT; i++)
    {
        Sum_Speed += Speed_Buf[i];
    }
    Filter_Speed = (float)(Sum_Speed / VOL_COUNT);
    return Filter_Speed;
}

