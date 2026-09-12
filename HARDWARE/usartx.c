#include "usartx.h"
#include "modbus.h"

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;

//超声波数据发送结构体
SEND_DISTANCE_DATA Distance_Data;

//自动回充数据发送结构体
SEND_AutoCharge_DATA Send_AutoCharge_Data;

#if USE_RGB_lights
//灯带数据
u8 rgb_set[8] = {Natural_mode,0,0,0,0,0,0,0};
u8 rgb_r=127,rgb_g=127,rgb_b=127;
#endif

u8 BT_Key=0; //蓝牙按键，用于更新Z轴加速度，实现实体按键一样的效果

//电机自检频率控制值
u8 check_motor_hz=0;

/**************************************************************************
Function: Usartx3, Usartx1 and CAN send data task
Input   : none
Output  : none
函数功能：串口3、串口1、CAN发送数据任务
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();

    while(1)
    {
        //The task is run at 20hz
        //此任务以20Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));

        //Assign the data to be sent
        //对要进行发送的数据进行赋值
        data_transition();

        UART4_SEND();     //Serial port 4 (ROS) sends data  //串口4(ROS)发送数据
		
		//机器人基础24字节发送
		CAN_SEND_24BasicData();
        //////////////////自动回充相关数据发送//////////////////
        CAN_Send_AutoRecharge();

        //////////////////电机驱动自检//////////////////
        if(++check_motor_hz>40)
        {
            check_motor_hz=0;		
			motor_checkerror_flag++;//驱动1离线检查
			if( motor_checkerror_flag>1 )
			{
				motor_checkerror_flag=2;
				Clear_Error_Flag(Drive1_ERROR);
				Set_SystemError_FLAG(Drvie_Timeout);
				if(Car_Mode!=S200) Self_CheckingFlag &= 0xFFFE0000;//非四驱车,驱动掉线时直接清空驱动报错,不需要确认2号驱动的情况
			}
			else
			{
				Clear_Error_Flag(Drvie_Timeout);
			}
				
			if(Car_Mode==S200)
			{	//四驱车型,驱动2离线检查
				motor2_checkerror_flag++;
				if( motor2_checkerror_flag>1 )
				{
					motor2_checkerror_flag=2;
					Clear_Error_Flag(Drive2_ERROR);
					Set_SystemError_FLAG(Drvie2_Timeout);
				}
				else
				{
					Clear_Error_Flag(Drvie2_Timeout);
				}			
				//两个驱动同时掉线,清空报错
				if( Get_Checking_FLAG(Drvie2_Timeout)&&Get_Checking_FLAG(Drvie_Timeout) )
				{
					Self_CheckingFlag &= 0xFFFE0000;
				}
			}		
        }
		//////////////////电机驱动自检//////////////////
    }
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
    static u8 autorc_Timeout = 0; //用于检测CAN中断里是否还存在自动回充的数据
//    static u8 RED;
    Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
    Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾

    //Forward kinematics solution, from the current speed of each wheel to calculate the current speed of the three axis
    //运动学正解，从各车轮当前速度求出三轴当前速度
	if(Car_Mode==S300||Car_Mode==S150||Car_Mode==S100)
	{
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //小车x轴速度
		Send_Data.Sensor_Str.Y_speed = 0;
		Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;//小车z轴速度
	}
	else if(Car_Mode==S200)
	{
		Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; 
		Send_Data.Sensor_Str.Y_speed = 0; 
		Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Wheel_axlespacing+Wheel_spacing))*1000;
	}


#if OLED_DEBUG_MODE
    pos_x += ((Send_Data.Sensor_Str.X_speed/1000.0f)*cos(pos_z) - (Send_Data.Sensor_Str.Y_speed/1000.0f)*sin(pos_z))*0.05f;
    pos_y += ((Send_Data.Sensor_Str.X_speed/1000.0f)*sin(pos_z) + (Send_Data.Sensor_Str.Y_speed/1000.0f)*cos(pos_z))*0.05f;
    pos_z += (Send_Data.Sensor_Str.Z_speed/1000.0f)*0.05f;
    if(pos_x>100 || pos_x<-100) pos_x=0;
    if(pos_y>100 || pos_y<-100) pos_y=0;
    if(pos_z>100 || pos_z<-100) pos_z=0;//防止变量溢出，一般的debug也不需要用到这么大的数值
#endif

    //The acceleration of the triaxial acceleration //加速度计三轴加速度
    Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
    Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
    Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2]; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴

    //The Angle velocity of the triaxial velocity //角速度计三轴角速度
    Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
    Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
    if(show_motor_state==0)
        //If the motor control bit makes energy state, the z-axis velocity is sent normall
        //如果电机控制位使能状态，那么正常发送Z轴角速度
        Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];
    else
        //If the robot is static (motor control dislocation), the z-axis is 0
        //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0
        Send_Data.Sensor_Str.Gyroscope.Z_data=0;

    //Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
    //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)
    Send_Data.Sensor_Str.Power_Voltage = Voltage*1000;

    Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
    Send_Data.buffer[1]=show_motor_state; //Car software loss marker //小车软件失能标志位

    //The three-axis speed of / / car is split into two eight digit Numbers
    //小车三轴速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8;
    Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;
    Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;
    Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;
    Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8;
    Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;

    //The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
    //IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8;
    Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;
    Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
    Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
    Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
    Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;

    //The axis of the triaxial velocity of the / /imu is divided into two eight digits
    //IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
    Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
    Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
    Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
    Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
    Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
    Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;

    //Battery voltage, split into two 8 digit Numbers
    //电池电压,拆分为两个8位数据发送
    Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8;
    Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage;

    //Data check digit calculation, Pattern 1 is a data check
    //数据校验位计算，模式1是发送数据校验
    Send_Data.buffer[22]=Check_Sum(22,1);
    Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾

	//====================== 以下是超声波赋值数据 =====================//
	Distance_Data.Distance_Str.Frame_Header=Distance_HEADER;
	Distance_Data.Distance_Str.Frame_Tail=Distance_TAIL;
	Distance_Data.Distance_Str.distanceA =ultrasonic.A*1000;
	Distance_Data.Distance_Str.distanceB =ultrasonic.B*1000;
	Distance_Data.Distance_Str.distanceC =ultrasonic.C*1000;
	Distance_Data.Distance_Str.distanceD =ultrasonic.D*1000;
	Distance_Data.Distance_Str.distanceE =ultrasonic.E*1000;
	Distance_Data.Distance_Str.distanceF =ultrasonic.F*1000;
	Distance_Data.Distance_Str.distanceG =0;
	Distance_Data.Distance_Str.distanceH =0;
	Distance_Data.buffer[0]=Distance_Data.Distance_Str.Frame_Header;
	Distance_Data.buffer[1]=Distance_Data.Distance_Str.distanceA>>8;
	Distance_Data.buffer[2]=Distance_Data.Distance_Str.distanceA;
	Distance_Data.buffer[3]=Distance_Data.Distance_Str.distanceB>>8;
	Distance_Data.buffer[4]=Distance_Data.Distance_Str.distanceB;
	Distance_Data.buffer[5]=Distance_Data.Distance_Str.distanceC>>8;
	Distance_Data.buffer[6]=Distance_Data.Distance_Str.distanceC;
	Distance_Data.buffer[7]=Distance_Data.Distance_Str.distanceD>>8;
	Distance_Data.buffer[8]=Distance_Data.Distance_Str.distanceD;
	Distance_Data.buffer[9]=Distance_Data.Distance_Str.distanceE>>8;
	Distance_Data.buffer[10]=Distance_Data.Distance_Str.distanceE;
	Distance_Data.buffer[11]=Distance_Data.Distance_Str.distanceF>>8;
	Distance_Data.buffer[12]=Distance_Data.Distance_Str.distanceF;

	Distance_Data.buffer[13]=Self_CheckingFlag>>24; //机器人自检参数
	Distance_Data.buffer[14]=Self_CheckingFlag>>16;
	Distance_Data.buffer[15]=Self_CheckingFlag>>8;
	Distance_Data.buffer[16]=Self_CheckingFlag;

	Distance_Data.buffer[17]=Check_Sum(17,3);
	Distance_Data.buffer[18]=Distance_Data.Distance_Str.Frame_Tail;
	//====================== 以上是超声波赋值数据 =====================//


    ///////////////////////自动回充相关变量赋值/////////////////////
    AutoRcCount = CheckAutoRc;
    if(AutoRcCount==Last_CheckAutoRc) autorc_Timeout++;
    else autorc_Timeout=0;

    if(autorc_Timeout>20) //自动回充数据超时
    {
        autorc_Timeout=21; //锁住变量
        Set_SystemError_FLAG(AutoRecharge_Timeout);//自动回充掉线标志位
        Clear_Error_Flag(lost_left_redsignal);
        Clear_Error_Flag(lost_right_redsignal);
    }
    else
        Clear_Error_Flag(AutoRecharge_Timeout);//自动回充上线

    Send_AutoCharge_Data.AutoCharge_Str.Frame_Header = AutoCharge_HEADER;   //帧头赋值0x7C
    Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail = AutoCharge_TAIL;		//帧尾赋值0x7F
    Send_AutoCharge_Data.AutoCharge_Str.Charging_Current = (short)Charging_Current;//充电电流赋值

//    if(RED_STATE>0)RED=1;
//    else           RED=0;
    Send_AutoCharge_Data.AutoCharge_Str.RED = RED_STATE; //红外标志位赋值
    Send_AutoCharge_Data.AutoCharge_Str.Charging = Charging; //是否在充电标志位赋值

    Send_AutoCharge_Data.buffer[0] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Header;		//帧头0x7C
    Send_AutoCharge_Data.buffer[1] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current>>8;//充电电流高8位
    Send_AutoCharge_Data.buffer[2] = Send_AutoCharge_Data.AutoCharge_Str.Charging_Current;	//充电电流低8位
    Send_AutoCharge_Data.buffer[3] = Send_AutoCharge_Data.AutoCharge_Str.RED;				//是否接收到红外标志位
    Send_AutoCharge_Data.buffer[4] = Send_AutoCharge_Data.AutoCharge_Str.Charging;			//是否在充电标志位
    Send_AutoCharge_Data.buffer[5] = Allow_Recharge;														//预留位
    Send_AutoCharge_Data.buffer[6] = Check_Sum_AutoCharge(6,1);								//校验位
    Send_AutoCharge_Data.buffer[7] = Send_AutoCharge_Data.AutoCharge_Str.Frame_Tail;		//帧尾0x7F
    Last_CheckAutoRc = CheckAutoRc;
    ///////////////////////自动回充相关变量赋值/////////////////////
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
    unsigned char i = 0;

    //基础24字节发送
    for(i=0; i<24; i++)
    {
        usart1_send(Send_Data.buffer[i]);
    }
    //超声波数据发送
    for(i=0; i<19; i++)
    {
        usart1_send(Distance_Data.buffer[i]);
    }
    //自动回充相关变量发送
    for(i=0; i<8; i++)
    {
        usart1_send(Send_AutoCharge_Data.buffer[i]);
    }
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
函数功能：串口3发送数据
入口参数：无
返回  值：无
**************************************************************************/

//临时调试使用
struct Robotmsg robotmsg;//机器人需要上发的数据组
u8 robotmsg_len = sizeof(robotmsg);
short showArpm=0,showBrpm=0,showCrpm=0,showDrpm=0;
uint8_t RobotMsg_BCC(const struct Robotmsg* msg) 
{
    uint8_t bcc = 0;
    const uint8_t* data = (const uint8_t*)msg;
    
    // 计算BCC，从第一个字节开始，一直到BCC_Check字段之前的最后一个字节
    for (size_t i = 0; i < sizeof(struct Robotmsg) - 2; i++) {
        bcc ^= data[i];
    }
    
    return bcc;
}
//临时调试使用

void UART4_SEND(void)
{
    unsigned char i = 0;
    //基础24字节数据
    for(i=0; i<24; i++)
    {
        uart4_send(Send_Data.buffer[i]);
    }
	
//	if( Car_Mode!=S200 ) //调试临时使用
//	{
		//S100不发送超声波数据(s100 ros上层未有对应处理逻辑)
		if( Car_Mode!=S100 || Car_Mode!=S200 )
		{
			//超声波数据+自检数据
			for(i=0; i<19; i++)
			{
				uart4_send(Distance_Data.buffer[i]);
			}
		}

		//自动回充相关变量发送
		for(i=0; i<8; i++)
		{
			uart4_send(Send_AutoCharge_Data.buffer[i]);
		}
//	}
//	else //四驱车调试数据 调试临时使用
//	{
//		//数据赋值
//		robotmsg.Head = 0xCC;
//		robotmsg.vol = Voltage;
//		robotmsg.Tvx = Move_X;
//		robotmsg.Vx = (MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4.0f; 
//		robotmsg.Tvz = Move_Z;
//		robotmsg.Vz = (-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4.0f/(Wheel_axlespacing+Wheel_spacing);
//		robotmsg.MotoA_TRPM = showArpm;
//		robotmsg.MotoA_RPM =  MOTOR_A.Encoder_Rpm/10.0f;
//		robotmsg.MotoB_TRPM = showBrpm;
//		robotmsg.MotoB_RPM =  MOTOR_B.Encoder_Rpm/10.0f;
//		robotmsg.MotoC_TRPM = showCrpm;
//		robotmsg.MotoC_RPM =  MOTOR_C.Encoder_Rpm/10.0f;
//		robotmsg.MotoD_TRPM = showDrpm;
//		robotmsg.MotoD_RPM =  MOTOR_D.Encoder_Rpm/10.0f;
//		robotmsg.A_Temp = Back_Drive.L_motorTemperature;
//		robotmsg.B_Temp = Front_Drive.L_motorTemperature;
//		robotmsg.C_Temp = Front_Drive.R_motorTemperature;
//		robotmsg.D_Temp = Back_Drive.R_motorTemperature;
//		robotmsg.A_cur = Back_Drive.L_motorCurrent;
//		robotmsg.B_cur = Front_Drive.L_motorCurrent;
//		robotmsg.C_cur = Front_Drive.R_motorCurrent;
//		robotmsg.D_cur = Back_Drive.R_motorCurrent;
//		robotmsg.BCCcheck  = RobotMsg_BCC(&robotmsg);
//		robotmsg.End = 0xDD;
//		
//		//数据发送
//		u8* sendptr = (u8*)&robotmsg;
//		for(u8 i=0;i<robotmsg_len;i++)
//		{
//			uart4_send(*sendptr);
//			sendptr++;
//		}
//	}

}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
函数功能：CAN发送数据
入口参数：无
返 回 值：无
**************************************************************************/
void CAN_SEND_24BasicData(void)
{
    u8 CAN_SENT[8],i;

    for(i=0; i<8; i++)
    {
        CAN_SENT[i]=Send_Data.buffer[i];
    }
    CAN1_Send_Num(0x101,CAN_SENT);

	
    for(i=0; i<8; i++)
    {
        CAN_SENT[i]=Send_Data.buffer[i+8];
    }
    CAN1_Send_Num(0x102,CAN_SENT);

	
    for(i=0; i<8; i++)
    {
        CAN_SENT[i]=Send_Data.buffer[i+16];
    }
    CAN1_Send_Num(0x103,CAN_SENT);
	
	//CAN压力测试,每秒2000帧可使用
//    for(i=0; i<8; i++)
//    {
//        CAN_SENT[i]=Send_Data.buffer[i];
//    }
//	for(i=0;i<100;i++)
//	{
//		CAN1_Send_Num((0x100+i),CAN_SENT);
//	}
//	
	
	
}

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
    //Subpriority //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
    USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}
/**************************************************************************
Function: Serial port 4 initialization
Input   : none
Output  : none
函数功能：串口4初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
    //Subpriority //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
    USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
    USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //初始化串口3

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //使能串口3
}

/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
函数功能：串口4初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
    USART_Init(UART4, &USART_InitStructure);      //Initialize serial port 3 //初始化串口3

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(UART4, ENABLE);                     //Enable serial port 3 //使能串口3
}

void uart6_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //Enable the Usart clock //使能USART时钟

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

    //UsartNVIC configuration //UsartNVIC配置
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    //Preempt priority //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=4 ;
    //Subpriority //子优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //Enable the IRQ channel //IRQ通道使能
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //Initialize the VIC register with the specified parameters
    //根据指定的参数初始化VIC寄存器
    NVIC_Init(&NVIC_InitStructure);

    //USART Initialization Settings 初始化设置
    USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
    USART_Init(USART6, &USART_InitStructure); //Initialize serial port 1 //初始化串口1

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
    USART_Cmd(USART6, ENABLE);                     //Enable serial port 1 //使能串口1
}

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{
    static u8 Count=0;
	
    u8 Usart_Receive;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        Usart_Receive = USART_ReceiveData(USART1);//Read the data //读取数据

        //Fill the array with serial data
        //串口数据填入数组
        Receive_Data.buffer[Count]=Usart_Receive;
		
        //Ensure that the first data in the array is FRAME_HEADER
        //确保数组第一个数据为FRAME_HEADER
        if(Usart_Receive == FRAME_HEADER||Count>0)
            Count++;
        else
            Count=0;
		
        if (Count == 11) //Verify the length of the packet //验证数据包的长度
        {
            Count=0; //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
            if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
            {
                //Data exclusionary or bit check calculation, mode 0 is sent data check
                //数据异或位校验计算，模式0是发送数据校验
                if(Receive_Data.buffer[9] ==Check_Sum(9,0))
                {
                    disable_robot_count=0;//计时变量清零

                    //Serial port 1 controls flag position 1, other flag position 0
                    //串口1控制标志位置1，其它标志位置0
                    Set_Control_Mode(_USART_Control);

                    //Calculate the target speed of three axis from serial data, unit m/s
                    //从串口数据求三轴目标速度， 单位m/s
                    Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
                    Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
                    Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
                }
            }
        }
    }
    return 0;
}

int USART6_IRQHandler(void)
{
	static u8 Res_count = 0;
	static u8 Res_buf[5];
    u8 Usart_Receive;

    if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        Usart_Receive = USART_ReceiveData(USART6);//Read the data //读取数据
				
		// boot检查
		Res_buf[Res_count] = Usart_Receive;
        if(Usart_Receive == 'r'||Res_count>0)
            Res_count++;
        else
            Res_count=0;
		
		if (Res_count==5)
		{
			Res_count = 0;
			
			if(Res_buf[0]=='r'&&Res_buf[1]=='e'&&Res_buf[2]=='s'&&Res_buf[3]=='e'&&Res_buf[4]=='t')
			{
				while(1) 
				{
					set_MotorRPM(COBID_Send_1,0,0);//电机速度置0
					set_MotorRPM(COBID_Send_2,0,0);//电机速度置0
					NVIC_SystemReset(); //进入BootLoader
				}
			}
		}	
    }
    return 0;
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口4接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{
    int Usart_Receive;
#if USE_US_Avoid
    static u8 open_us_avoid_filter=0;
#endif
    static u8 clear_error_count=0;
    static u8 debug_for_autorc=0;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
        static float Data;

        Usart_Receive=USART2->DR; //Read the data //读取数据
		
		//蓝牙连接与断开字符过滤
		if(AT_Command_Capture(Usart_Receive)) return 1;
		
		//复位指令
		_System_Reset_(Usart_Receive);
		
        if(Deviation_Count<CONTROL_DELAY)
            // Data is not processed until 10 seconds after startup
            //开机10秒前不处理数据
            return 0;
		
		//app进入debug模式
		APP_DebugMode(Usart_Receive);
		
        if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&Get_Control_Mode(_APP_Control)==0)
            //10 seconds after startup, press the forward button of APP to enter APP control mode
            //The APP controls the flag position 1 and the other flag position 0
            //开机10秒之后，按下APP的前进键进入APP控制模式
            //APP控制标志位置1，其它标志位置0
            Set_Control_Mode(_APP_Control);
        Last_Usart_Receive=Usart_Receive;

        if(Usart_Receive==0x4B)
            //Enter the APP steering control interface
            //进入APP转向控制界面
            Turn_Flag=1;
        else	if(Usart_Receive==0x49||Usart_Receive==0x4A)
            // Enter the APP direction control interface
            //进入APP方向控制界面
            Turn_Flag=0;

        if(Turn_Flag==0)
        {
            //App rocker control interface command
            //APP摇杆控制界面命令
            if(Usart_Receive>=0x41&&Usart_Receive<=0x48)
            {
                Flag_Direction=Usart_Receive-0x40;
            }
            else	if(Usart_Receive<=8)
            {
                Flag_Direction=Usart_Receive;
            }
            else  Flag_Direction=0;
        }
        else if(Turn_Flag==1)
        {
            //与摇杆界面相同
            if(Usart_Receive>=0x41&&Usart_Receive<=0x48)
                Flag_Direction=Usart_Receive-0x40;
            else
                Flag_Direction=0;
            //麦轮式控制
//			//APP steering control interface command
//			//APP转向控制界面命令
//			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
//			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
//			else                         Flag_Left=0,Flag_Right=0;
//			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
//			else  Flag_Direction=0;
        }

        if(Usart_Receive==0x58&&debug_for_autorc==0)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
        if(Usart_Receive==0x58&&debug_for_autorc==1)  Allow_Recharge = !Allow_Recharge,recharge_flag_beep=1;

        if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s

        ///////////// 以下为APP调试界面按键功能 /////////////
        if(Usart_Receive=='a') BT_Key=1;//陀螺仪数据更新

        //按键开启自动回充功能
        else if(Usart_Receive=='h') Allow_Recharge = !Allow_Recharge,recharge_flag_beep=1;

		#if USE_RGB_lights
        else if(Usart_Receive==0x62)
        {
            if(rgb_set[0]==User_defined)//已经是用户自定义模式
            {
                rgb_set[0]=Natural_mode;//恢复自然模式
            }
            else//处于其他模式，切换成用户自定义模式
                rgb_set[0]=User_defined; //用户自定义
        }
		#endif

		#if OLED_DEBUG_MODE
        else if(Usart_Receive=='m')//菜单切换
        {
            oled_show_mode++;
            oled_reflash_flag=1;
            if(oled_show_mode==OLED_MAX_PAGE) oled_show_mode=0;
        }
        //里程计清0
        else if(Usart_Receive=='c') pos_x=0,pos_y=0,pos_z=0;
		#endif

        //底盘避障功能开启/关闭
		#if USE_US_Avoid
        else if(Usart_Receive=='n')
        {
            if(Get_Control_Mode(_APP_Control)||Get_Control_Mode(_RC_Control)||Get_Control_Mode(_PS2_Control))//APP、航模、PS2控制时，允许开启底层避障
            {
                if(Car_Mode==S300||Car_Mode==S150||Car_Mode==S100)
                    open_us_avoid_filter++;//S300/S150/S100在非ros控制下允许启动底盘避障
            }

            if(open_us_avoid_filter>=3)
            {
                avoid_flag_beep=1;//蜂鸣器提示进入底盘避障模式
                open_us_avoid_filter=0;
                Open_US_avoid = !Open_US_avoid;
            }
        }
        else if(Usart_Receive=='l') start_avoid+=0.1f;
        else if(Usart_Receive=='o') start_avoid-=0.1f;
		#endif

        //电机故障消除
        else if(Usart_Receive=='i')
        {
            clear_error_count++;
            if(clear_error_count==3)
            {
                clear_error_count=0;
                motor_clear_error=1;
            }
        }

        //自动回充debug模式，用加速按键开启自动回充
        else if (Usart_Receive=='k')
        {
            debug_for_autorc=!debug_for_autorc;
        }
		
		else if(Usart_Receive=='d')
		{
			mow_dir = !mow_dir;
		}
		
		else if(Usart_Receive=='e')
		{
			static u16 last_motorpwm = 0;
			if(mow_motor!=8400) last_motorpwm = mow_motor , mow_motor = 8400;
			else mow_motor = last_motorpwm;
		}
		
        /*          APP调试页按键布局及功能

        陀螺仪更新	          灯带自定义			里程计清0

        	割草电机方向切换	割草电机开启/关闭		 f

        	g		   开启自动回充功能		电机故障消除(3)

        	j		   切换加速按键功能		避障距离+0.1

        切换菜单		开启底盘避障(3)		避障距离-0.1
        */
        ///////////// 以上为APP调试界面按键功能 /////////////
		
        // The following is the communication with the APP debugging interface
        //以下是与APP调试界面通讯
        if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP参数指令起始位
        if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位

        if(Flag_PID==1) //Collect data //采集数据
        {
            Receive[i]=Usart_Receive;
            i++;
        }
        if(Flag_PID==2) //Analyze the data //分析数据
        {
            if(Receive[3]==0x50) 	 PID_Send=1;
            else  if(Receive[1]!=0x23)
            {
                for(j=i; j>=4; j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
                case 0x30:
                    RC_Velocity=Data;
                    break;

				#if USE_US_Avoid
                case 0x31:
                    start_avoid = (float)Data/100.0f;
                    break;
				#else
                case 0x31:
                    break;
				#endif

				#if USE_RGB_lights
                case 0x32:
                    rgb_r = Data;
                    break;
                case 0x33:
                    rgb_g = Data;
                    break;
                case 0x34:
                    rgb_b = Data;
                    break;
				#else
                case 0x32:
                    break;
                case 0x33:
                    break;
                case 0x34:
                    break;
				#endif
                case 0x35:
                   mow_motor = Data;
				   break;
                case 0x36:
                    break;
                case 0x37:
                    break;
                case 0x38:
                    break;
                }
            }
            //Relevant flag position is cleared
            //相关标志位清零
            Flag_PID=0;
            i=0;
            j=0;
            Data=0;
            memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//数组清零
        }
        if(RC_Velocity<0)   RC_Velocity=0;
    }
    return 0;
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口4接收中断
入口参数：无
返回  值：无
**************************************************************************/
int UART4_IRQHandler(void)
{
    static u8 Count=0;
    u8 Usart_Receive;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        Usart_Receive = USART_ReceiveData(UART4);//Read the data //读取数据
        if(Time_count<CONTROL_DELAY)
            // Data is not processed until 10 seconds after startup
            //开机10秒前不处理数据
            return 0;

        //Fill the array with serial data
        //串口数据填入数组
        Receive_Data.buffer[Count]=Usart_Receive;

        // Ensure that the first data in the array is FRAME_HEADER
        //确保数组第一个数据为FRAME_HEADER
        if(Usart_Receive == FRAME_HEADER||Count>0)
            Count++;
        else
            Count=0;

        if (Count == 11) //Verify the length of the packet //验证数据包的长度
        {
            Count=0; //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
            if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
            {
                //Data exclusionary or bit check calculation, mode 0 is sent data check
                //数据异或位校验计算，模式0是发送数据校验
                if(Receive_Data.buffer[9] ==Check_Sum(9,0))
                {
                    disable_robot_count=0;//计时变量清零
					
                    if(Receive_Data.buffer[1]==0)
                    {
                        Allow_Recharge=0; //关闭自动回充
                        //All modes flag position 0, USART3 control mode
                        //所有模式标志位置0，为Usart3控制模式
                        Set_Control_Mode(_ROS_Control);
						
						SecurityPLY = Receive_Data.buffer[2];
						
                        //Calculate the target speed of three axis from serial data, unit m/s
                        //从串口数据求三轴目标速度， 单位m/s
                        Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
                        Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
                        Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
                    }
                    //Add AutoRecharger relevant, 2022.01.18
                    //添加自动回充相关，2022.01.18
                    else if( Receive_Data.buffer[1]==1 || Receive_Data.buffer[1]==2 )
                    {
                        Allow_Recharge=1; //开启自动回充
                        if(Receive_Data.buffer[1]==1 && RED_STATE==0) nav_walk=1; //开启自动回充，上位机控制机器人

                        //Calculate the target speed of three axis from serial data, unit m/s
                        //从串口数据求三轴目标速度，单位m/s
                        Recharge_UP_Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
                        Recharge_UP_Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
                        Recharge_UP_Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
                    }
                    else if( Receive_Data.buffer[1]==3 )
                    {
                        //Set the speed of the infrared interconnection, unit m/s
                        //设置红外对接的速度大小，单位m/s
                        Red_Docker_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
                        Red_Docker_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
                        Red_Docker_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
                    }
					
					//灯带设置
					else if( Receive_Data.buffer[1]==4 )
					{
						if( Receive_Data.buffer[2]==1 )
						{
							rgb_set[0] = User_defined;
							rgb_r = Receive_Data.buffer[3];
							rgb_g = Receive_Data.buffer[4];
							rgb_b = Receive_Data.buffer[5];
						}
						else if( Receive_Data.buffer[2]==0 ) rgb_set[0] = Natural_mode;
					}
					
					//临时调试使用
					else if( Receive_Data.buffer[1]==0x0F )
					{
						if( Receive_Data.buffer[2]==0xF0 ) beep=!beep;
					}
					//临时调试使用
					
                }
            }
        }
    }
    return 0;
}

/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
函数功能：串口3接收中断(串口转RS485)
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{
    u8 Usart_Receive;

    static u8 _Test_RS485_Data[8];
    static u8 Count = 0;

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
    {
        Usart_Receive = USART_ReceiveData(USART3);//Read the data //读取数据

        _Test_RS485_Data[Count] = Usart_Receive;//接收数据

        //RS485测试demo:接收到帧头0xAA ，帧尾0xFF的8字节数据包时，则返回该数据包内容
        if(Usart_Receive == 0xAA || Count>0)
            Count++;

        if(Count==8)
        {
            Count=0;
            if(_Test_RS485_Data[7]==0xFF)
            {
                RS485_Send(_Test_RS485_Data,sizeof(_Test_RS485_Data));
            }
        }

    }
    return 0;
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
    //Data conversion intermediate variable
    //数据转换的中间变量
    short transition;

    //将高8位和低8位整合成一个16位的short型数据
    //The high 8 and low 8 bits are integrated into a 16-bit short data
    transition=((High<<8)+Low);
    return
        transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
    USART1->DR = data;
    while((USART1->SR&0x40)==0);
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
    USART2->DR = data;
    while((USART2->SR&0x40)==0);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
    USART3->DR = data;
    while((USART3->SR&0x40)==0);
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void uart4_send(u8 data)
{
    UART4->DR = data;
    while((UART4->SR&0x40)==0);
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
    unsigned char check_sum=0,k;

    //Validate the data to be sent
    //对要发送的数据进行校验
    if(Mode==1)
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Send_Data.buffer[k];
        }

    //Verify the data received
    //对接收到的数据进行校验
    if(Mode==0)
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Receive_Data.buffer[k];
        }

    //对要发送的超声波数据进行校验
    if(Mode==3)
    {
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Distance_Data.buffer[k];
        }
    }
    return check_sum;
}

//自动回充发送字节专用校验函数
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode)
{
    unsigned char check_sum=0,k;

    //Validate the data to be sent
    //对要发送的数据进行校验
    if(Mode==1)
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Send_AutoCharge_Data.buffer[k];
        }

    return check_sum;
}


//RS485常用函数
//Modbus的crc校验，查表法
u16 Modbus_crc(u8* arry,u8 size)
{
    u16 crc = 0xFFFF;
    u16 ref;

    while(size--)
    {
        crc = (crc >> 8) ^ crc_table[(crc ^ *arry++) & 0xff];
    }

    ref = crc>>8;
    ref = ref | (crc<<8);

    return ref;
}

//Modbus协议读取数据
void Modbus_ReadData(u8 addr,u8 Funcode,u16 beginaddr,u16 len)
{
    u8 data[6];
    u8 crc_data[2];
    u16 crc_ref;

    data[0] = addr; //设备地址
    data[1] = Funcode; //功能码

    data[2] = beginaddr>>8; //寄存器地址
    data[3] = beginaddr;

    data[4] = len>>8; //数据长度/读取的寄存器个数
    data[5] = len;

    crc_ref = Modbus_crc(data,sizeof(data));

    crc_data[0] = crc_ref>>8;
    crc_data[1] = crc_ref;

    RS485_Send(data,sizeof(data));
    RS485_Send(crc_data,sizeof(crc_data));
}

void RS485_Send(u8* data,u8 len)
{
    u8 i=0;
    for(i=0; i<len; i++)
    {
        usart3_send(data[i]);
    }
}

//蓝牙AT指令抓包，防止指令干扰到机器人正常的蓝牙通信
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	蓝牙链接时发送的字符，00:11:22:33:44:55为蓝牙的MAC地址
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	共44个字符
	
	蓝牙断开时发送的字符
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	共34个字符
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //蓝牙接受时指针记录器
	static u8 bt_line = 0; //表示现在在第几行
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//断开连接
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//开始连接
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//特殊标识符，开始警惕(使用时要-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //收到‘+’，表示切换了行数	
		disconnect++,connect++;
		return 1;//抓包，禁止控制
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//开始追踪数据是否符合断开的特征，符合时全部屏蔽，不符合时取消屏蔽
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//抓包，禁止控制
		}			

		//追踪连接特征 (bt_line==1&&connect>=13)区段是蓝牙MAC地址，每一个蓝牙MAC地址都不相同，所以直接屏蔽过去
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//抓包，禁止控制
		}	

		//在抓包期间收到其他命令，停止抓包
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//非禁止数据，可以控制
		}			
	}
	
	return 0;//非禁止数据，可以控制
}


//软复位进BootLoader区域
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//进行软件复位，复位后执行 BootLoader 程序
		}
	}
}

void APP_DebugMode(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='d'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;

		if( res_buf[0]=='d'&&res_buf[1]=='e'&&res_buf[2]=='b'&&res_buf[3]=='u'&&res_buf[4]=='g' )
		{
			APP_Debug = !APP_Debug;
		}
	}
}

