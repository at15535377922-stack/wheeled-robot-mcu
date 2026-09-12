#include "bsp_can.h"

#define CAN1_RX0_INT_ENABLE 1
#define CAN2_RX1_INT_ENABLE 1

// CAN1 CAN2 初始化
// 波特率计算公式  CAN总线频率 / [（ tsjw + tbs2 + tbs1 ） * brp ]
// 建议配置 tbs2 < tbs1
void CAN_1_2_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	//-1是因为这些配置实际数值从0开始
	//brp不需要-1，因为其配置数值是从1开始
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0) return ;
	tsjw-=1; //Subtract 1 before setting //先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	
	#if CAN1_RX0_INT_ENABLE 
	NVIC_InitTypeDef  NVIC_InitStructure;
	#endif
	
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	   

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	

	//初始化GPIO
	//CAN1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);           

	//CAN2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
	
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	//模式设置 
	
	//波特率相关配置
	CAN_InitStructure.CAN_SJW=tsjw;	    //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1;     //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;     //Tbs2范围CAN_BS2_1tq ~CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;//分频系数(Fdiv)为brp+1	
	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 	
	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2
	
	//配置过滤器
	//CAN1
	CAN_FilterInitStructure.CAN_FilterNumber=0;	                   //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;               //32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 	

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);            //滤波器初始化
	
	//CAN2
	//CAN2必须从14号滤波器起，否则无法进入中断
	CAN_FilterInitStructure.CAN_FilterNumber=14;                  //过滤器14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000; 

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;//过滤器0关联到FIFO1
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器1
	CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2
	
	//CAN1 FIFO0 中断使能
	#if CAN1_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;// 主优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // 次优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
	
	//CAN2 FIFO1 中断使能
	#if CAN2_RX1_INT_ENABLE
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//FIFO1消息挂号中断允许.		    
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif
}

  
// CAN1发送函数(不等待本次数据，检查上一次数据是否完成)
u8 CAN1_Send_Num(u32 id,u8 *data)
{
	//CAN发送数据结构体
	CanTxMsg msg;
	
	volatile u16 i=0;   //超时时间
	u8 mbox; 
	
	#if 1
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//发送的报文属于数据帧
	//数据帧 CAN_RTR_DATA
	//遥控帧 CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//发送的数据长度
	// 1~8
	msg.DLC = 8;
	
	//将要发送的数据复制入 msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN 发送
	mbox = CAN_Transmit(CAN1,&msg);
	
	//等待发送完成
	while( CAN_TransmitStatus(CAN1,mbox)== CAN_TxStatus_Pending && i<0xffff ) i++;
	
	//发送未完成，本次无法发送。记录错误。
	if(i>=0xffff) 
	{
		return 1;
	}
	return 0;
}


// CAN2发送函数
u8 CAN2_Send_Num(u32 id,u8 *data)
{
	//CAN发送数据结构体
	CanTxMsg msg;
	
	volatile u16 i=0;   //超时时间
	u8 mbox; 
	
	#if 1
	msg.StdId = id;
	msg.IDE = CAN_Id_Standard;
	#else
	msg.ExtId = id;
	msg.IDE = CAN_Id_Extended;
	#endif
	
	//发送的报文属于数据帧
	//数据帧 CAN_RTR_DATA
	//遥控帧 CAN_RTR_REMOTE
	msg.RTR = CAN_RTR_DATA;
	
	//发送的数据长度
	// 1~8
	msg.DLC = 8;
	
	//将要发送的数据复制入 msg.Data
	memcpy(msg.Data,data,8);
	
	//CAN 发送
	mbox = CAN_Transmit(CAN2,&msg);
	
	//等待发送完成
	while( CAN_TransmitStatus(CAN2,mbox)== CAN_TxStatus_Pending && i<0xffff ) i++;
	
	//发送未完成，本次无法发送。记录错误。
	if(i>=0xffff) 
	{
		return 1;
	}
	return 0;
}


#if CAN1_RX0_INT_ENABLE

u8 ChargDelay = 0;

// CAN1 FIFO0 接收中断
void CAN1_RX0_IRQHandler(void)
{
	//CAN接收数据结构体
	CanRxMsg RxMessage;
	
	//////////////// 用户数据定义区 ////////////////
	u8 temp_rxbuf[8];     //接收缓冲区
	
	//////////////// 用户数据定义区 ////////////////
	
	//读取CAN1 FIFO0邮箱的数据
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	//把数据读出到缓冲区
	memcpy(temp_rxbuf,RxMessage.Data,8);

	static u8 rgb_free=0;//灯带延迟反应的操作
	
	//////////////// CAN控制小车数据解析 //////////////
	if(RxMessage.StdId==0x181)
	{
		Set_Control_Mode(_CAN_Control); //标记为CAN控制
		
		disable_robot_count = 0;
		
		//CAN数据解析
		Move_X =  ((float)((short)((temp_rxbuf[0]<<8)|(temp_rxbuf[1]))))/1000;
		Move_Y =  ((float)((short)((temp_rxbuf[2]<<8)|(temp_rxbuf[3]))))/1000;
		Move_Z =  ((float)((short)((temp_rxbuf[4]<<8)|(temp_rxbuf[5]))))/1000;
		
	}
	
	//////////////// 自动回充数据 ////////////////
	if(RxMessage.StdId==0x182)
	{
		CheckAutoRc++;
		
		//Calculate the three-axis target velocity, unit: mm/s
		//计算三轴目标速度，单位：mm/s
		Recharge_Red_Move_X=((float)((short)((temp_rxbuf[0]<<8)+(temp_rxbuf[1]))))/1000;
		
		which_mode = temp_rxbuf[2];
		
		//Y轴不使用，S系列改成上传充电装备测量到的电压，方便debug
		Recharge_VOL= (temp_rxbuf[3]*30)/100.0f;
		
		Recharge_Red_Move_Z=((float)((short)((temp_rxbuf[4]<<8)+(temp_rxbuf[5]))))/1000;
		
		
		Charging=temp_rxbuf[6]&1;		//充电状态标志位
//		RED_STATE=(temp_rxbuf[6]>>1)&1; //红外信号接收情况
		
		//A:39ms红外情况（正面面向充电桩左边的红外）
		//B:52ms红外情况（正面面向充电桩右边的红外）
		L_A = (temp_rxbuf[6]>>5)&0x01;
		L_B = (temp_rxbuf[6]>>4)&0x01;
		R_B = (temp_rxbuf[6]>>3)&0x01;
		R_A = (temp_rxbuf[6]>>2)&0x01;
		
		RED_STATE = L_A+L_B+R_B+R_A;
		
		//自动回充装备对红外的屏蔽情况 0:未屏蔽   \
								    1:设置了手动屏蔽
		red_ignore = (temp_rxbuf[6]>>6)&0x01;
		
		//当开启自动回充模式后，增加对充电桩两路红外的自检
		if(Allow_Recharge)
		{
			if(L_A||R_A) Clear_Error_Flag(lost_left_redsignal);
			else Set_SystemError_FLAG(lost_left_redsignal);
			
			if(L_B||R_B) Clear_Error_Flag(lost_right_redsignal);
			else Set_SystemError_FLAG(lost_right_redsignal);
		}
		else
			Clear_Error_Flag(lost_left_redsignal),Clear_Error_Flag(lost_right_redsignal);
			
		//未识别到红外信号、已经在充电 -> 速度置0
		if(RED_STATE==0) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0;
		if( Charging==1) Recharge_Red_Move_X = Recharge_Red_Move_Y = Recharge_Red_Move_Z = 0,ChargDelay=1;//标记已经处于过充电状态
		
		//接近满电状态，清空速度防止出现充满后小车运动
		if( which_mode==0xAB && Voltage>25.0f ) Recharge_Red_Move_X = 0 , Recharge_Red_Move_Z = 0;
		
		//充电电流换算
		if(temp_rxbuf[7]>128)Charging_Current=-(256-temp_rxbuf[7])*30;
		else Charging_Current=(temp_rxbuf[7]*30);
		
		//充电状态切换到未充电状态过度
		if(ChargDelay&&!Charging)
		{
			rgb_free++;
			if(rgb_free>=30) //充电装备20ms发送1次数据 30*20ms=0.6s
			{
				rgb_free=0;
				ChargDelay = 0;
			}
		}
	}
	//////////////// 自动回充数据 ////////////////
	
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);  /* 清除挂起中断 */
}
#endif


#if CAN2_RX1_INT_ENABLE
// CAN2 FIFO1 接收中断
void CAN2_RX1_IRQHandler(void)
{
	//CAN接收数据结构体
	CanRxMsg RxMessage;
	
	//////////////// 用户数据定义区 ////////////////
	u8 temp_rxbuf[8];     //接收缓冲区
	
	//////////////// 用户数据定义区 ////////////////
	
	//读取 CAN2 FIFO1 的内容
	CAN_Receive(CAN2, CAN_FIFO1, &RxMessage);
	
	//把数据转移到缓冲区
	memcpy(temp_rxbuf,RxMessage.Data,8);
		
	//////////////// 编码器数据 ////////////////
	if(RxMessage.StdId==0x185)
	{
		if(Car_Mode==S300||Car_Mode==S150||Car_Mode==S100)
		{
			MOTOR_A.Encoder_Rpm  = (short)((temp_rxbuf[1]<<8)+(temp_rxbuf[0]));
			MOTOR_B.Encoder_Rpm  =-(short)((temp_rxbuf[5]<<8)+(temp_rxbuf[4]));
		}
		else if(Car_Mode==S200)
		{
			MOTOR_A.Encoder_Rpm  = (short)((temp_rxbuf[1]<<8)+(temp_rxbuf[0]));
			MOTOR_D.Encoder_Rpm  =-(short)((temp_rxbuf[5]<<8)+(temp_rxbuf[4]));		
		}
		Get_Velocity_Form_Encoder();
	}
	else if( RxMessage.StdId==0x285 )
	{
		MOTOR_B.Encoder_Rpm  = (short)((temp_rxbuf[1]<<8)+(temp_rxbuf[0]));
		MOTOR_C.Encoder_Rpm  =-(short)((temp_rxbuf[5]<<8)+(temp_rxbuf[4]));
		Get_Velocity_Form_Encoder();
	}
	//////////////// 编码器数据 ////////////////
	
	//////////////// 驱动状态数据 ////////////////
	else if(RxMessage.StdId==0x186) //存放电机电流、母线电压
	{
		Back_Drive.L_motorCurrent = -((short)(temp_rxbuf[1]<<8|temp_rxbuf[0]))*0.1f;
		Back_Drive.L_motorTemperature = ((short)(temp_rxbuf[3]<<8|temp_rxbuf[2]))*0.1f;
		Back_Drive.R_motorCurrent = ((short)(temp_rxbuf[5]<<8|temp_rxbuf[4]))*0.1f;
		Back_Drive.R_motorTemperature = ((short)(temp_rxbuf[7]<<8|temp_rxbuf[6]))*0.1f;
	}
	else if(RxMessage.StdId==0x286)
	{
		Front_Drive.L_motorCurrent = -((short)(temp_rxbuf[1]<<8|temp_rxbuf[0]))*0.1f;
		Front_Drive.L_motorTemperature = ((short)(temp_rxbuf[3]<<8|temp_rxbuf[2]))*0.1f;
		Front_Drive.R_motorCurrent = ((short)(temp_rxbuf[5]<<8|temp_rxbuf[4]))*0.1f;
		Front_Drive.R_motorTemperature = ((short)(temp_rxbuf[7]<<8|temp_rxbuf[6]))*0.1f;
	}
	//////////////// 驱动状态数据 ////////////////
	
	else if( RxMessage.StdId==0x187 )
	{
		motor_checkerror_flag=0;//驱动离线检测
		u16 Lstate = temp_rxbuf[1]<<8 | temp_rxbuf[0];//左电机状态
		u16 Rstate = temp_rxbuf[3]<<8 | temp_rxbuf[2];//右电机状态
		if(Lstate==0&&Rstate==0)
		{
			Clear_Error_Flag(Drive1_ERROR);
		}
		else
		{
			Set_SystemError_FLAG(Drive1_ERROR);
			set_hub_errorstate(Lstate,Rstate);//设置报错
		}
		
		//状态字
		Lstate = temp_rxbuf[5]<<8 | temp_rxbuf[4];//左电机
		Rstate = temp_rxbuf[7]<<8 | temp_rxbuf[6];//右电机
		HUB1_TEST_L = Lstate;
		HUB1_TEST_R = Rstate;
		if( Get_Checking_FLAG(Drive1_ERROR)==0 ) //驱动器不报错的情况
		{
			#if 0
			//         :  bit_6  bit_5  bit_3  bit_2  bit_1  bit_0
			//急停状态字：   1      x      0      0      0      0    
			//使能状态字：   0      1      0      1      1      1
			//与运算，未知的bit就拿0与,结果一定是0,已知的位就拿1与,得到已知的相同结果
			
			//急停状态字与结果： X1XX 0000 & 0100 1111 = state&0x4F == 0100 0000(0x40) 被失能
			//使能状态字与结果： X01X 0111 & 0110 1111 = state&0x6F == 0010 0111(0x27) 被使能
			if( (Lstate&0x006F)==0x27 && (Rstate&0x006F)==0x27 ) HUB1_EnableState=MOTOR_ENABLE; //两个电机都被使能才算完成使能
			
			//注意：这里有3个判断失能的条件是因为，使能电机时需要执行3条指令,若任意一条指令丢失,都无法成功使能,所以需要形成闭环判断。
			if( (Lstate&0x006F)==0x21 && (Rstate&0x006F)==0x21 ) HUB1_EnableState=MOTOR_DISABLE;//初始化未完成,等于失能
				
			if( (Lstate&0x006F)==0x23 && (Rstate&0x006F)==0x23 ) HUB1_EnableState=MOTOR_DISABLE;//初始化未完成,等于失能
				
			if( (Lstate&0x004F)==0x40 && (Rstate&0x004F)==0x40 ) HUB1_EnableState=MOTOR_DISABLE;//两个电机都被失能才算完成失能	
			if(Car_Mode!=S200) HUB2_EnableState = HUB1_EnableState;//非4驱车,复制驱动器1的状态给不存在的驱动器2
			//关于状态字的bit
			//         :  bit_6  bit_5  bit_3  bit_2  bit_1  bit_0
			//急停状态字：   1      x      0      0      0      0   完全失能 
			//readytoon：   0      1      0      0      0      1   进行使能步骤第1步 --> 此时算失能  // X01X0001 & 0110 1111(0x6F) == 0010 0001(0x21)
			//switch on:    0      1      0      0      1      1   进行使能步骤第2步 --> 此时算失能  // X01X0011 & 0110 1111(0x6F) == 0010 0011(0x23)
			//使能状态字：   0      1      0      1      1      1   使能成功,此时才可以正常控制
			#else
			
			if( (  (Lstate)&0x0027  )==0x27 && (  (Rstate)&0x0027  )==0x27 )
			{
				HUB1_EnableState=MOTOR_ENABLE;
			}
			else
				HUB1_EnableState=MOTOR_DISABLE;
			
			#endif
			
		}
		else//驱动1有报错,默认标记失能
		{
			HUB1_EnableState=MOTOR_DISABLE;
		}
		
		if(Car_Mode!=S200) HUB2_EnableState = HUB1_EnableState;//非4驱车,复制驱动器1的状态给不存在的驱动器2
		
	}
	
	else if( RxMessage.StdId==0x287 )
	{
		motor2_checkerror_flag=0;//驱动离线检测
		u16 Lstate = temp_rxbuf[1]<<8 | temp_rxbuf[0];//左电机状态
		u16 Rstate = temp_rxbuf[3]<<8 | temp_rxbuf[2];//右电机状态
		
		if(Lstate==0&&Rstate==0)
		{
			Clear_Error_Flag(Drive2_ERROR);
			if( Get_Checking_FLAG(Drive1_ERROR)==0 ) Self_CheckingFlag &= 0xFFFE0000;//驱动器1、2均无报错,清除与驱动器、电机相关的所有报错.
		}
		else
		{
			Set_SystemError_FLAG(Drive2_ERROR);
			
			if(Get_Checking_FLAG(Drive1_ERROR)==0)//驱动器1没有报错的前提下,设置驱动2的报错
			{
				set_hub_errorstate(Lstate,Rstate);//设置报错
			}
			
		}
		//状态字
		Lstate = temp_rxbuf[5]<<8 | temp_rxbuf[4];//左电机
		Rstate = temp_rxbuf[7]<<8 | temp_rxbuf[6];//右电机
		HUB2_TEST_L = Lstate;
		HUB2_TEST_R = Rstate;
		if( Get_Checking_FLAG(Drive2_ERROR)==0 ) //驱动器不报错的情况
		{
			#if 0
			//         :  bit_6  bit_5  bit_3  bit_2  bit_1  bit_0
			//急停状态字：   1      x      0      0      0      0    
			//使能状态字：   0      1      0      1      1      1
			//与运算，未知的bit就拿0与,结果一定是0,已知的位就拿1与,得到已知的相同结果
			
			//急停状态字与结果： X1XX 0000 & 0100 1111 = state&0x4F == 0100 0000(0x40) 被失能
			//使能状态字与结果： X01X 0111 & 0110 1111 = state&0x6F == 0010 0111(0x27) 被使能
			if( (Lstate&0x006F)==0x27 && (Rstate&0x006F)==0x27 ) HUB2_EnableState=MOTOR_ENABLE; //两个电机都被使能才算完成使能
			
			if( (Lstate&0x006F)==0x21 && (Rstate&0x006F)==0x21 ) HUB2_EnableState=MOTOR_DISABLE;//初始化未完成,等于失能
				
			if( (Lstate&0x006F)==0x23 && (Rstate&0x006F)==0x23 ) HUB2_EnableState=MOTOR_DISABLE;//初始化未完成,等于失能
				
			if( (Lstate&0x004F)==0x40 && (Rstate&0x004F)==0x40 ) HUB2_EnableState=MOTOR_DISABLE;//两个电机都被失能才算完成失能	
			#else
			
			if( (  (Lstate)&0x0027  )==0x27 && (  (Rstate)&0x0027  )==0x27 )
			{
				HUB2_EnableState=MOTOR_ENABLE;
			}
			else
				HUB2_EnableState=MOTOR_DISABLE;
			
			#endif
		}
		else//驱动1有报错,默认标记失能
		{
			HUB2_EnableState=MOTOR_DISABLE;
		}
	}
	else if( RxMessage.StdId==0x581 )
	{
		u16 addr=0;
		if( temp_rxbuf[0]==0x4B ) //2字节读反馈
		{
			addr = temp_rxbuf[2]<<8 | temp_rxbuf[1];//地址
			if( addr==0x2031 )
			{
				Back_Drive.SoftwareVersion = temp_rxbuf[5]<<8 | temp_rxbuf[4];
			}
		}
		
	}

	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);  /* 清除挂起中断 */
}


//设置驱动的错误状态
void set_hub_errorstate(u16 left_motor_state,u16 right_motor_state)
{
	if(Get_DriveError_FLAG(right_motor_state,Drvie_EEPROM_ERROR)!=0)     Set_SystemError_FLAG(Drvie_EEPROM_ERROR);//EEPROM读写错误
	else Clear_Error_Flag(Drvie_EEPROM_ERROR);
	
	if(Get_DriveError_FLAG(right_motor_state,Drvie_overVOL)!=0)          Set_SystemError_FLAG(Drvie_overVOL);//过压
	else Clear_Error_Flag(Drvie_overVOL);
	
	if(Get_DriveError_FLAG(right_motor_state,Drvie_underVOL)!=0)         Set_SystemError_FLAG(Drvie_underVOL);//欠压
	else Clear_Error_Flag(Drvie_underVOL);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_overCUR)!=0)        Set_SystemError_FLAG(R_Motor_overCUR);//过流
	else Clear_Error_Flag(R_Motor_overCUR);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_overLoad)!=0)       Set_SystemError_FLAG(R_Motor_overLoad);//过载
	else Clear_Error_Flag(R_Motor_overLoad);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_CUR_ERROR)!=0)      Set_SystemError_FLAG(R_Motor_CUR_ERROR);//电流超差
	else Clear_Error_Flag(R_Motor_CUR_ERROR);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_Encoder_ERROR)!=0)  Set_SystemError_FLAG(R_Motor_Encoder_ERROR);//编码器超差
	else Clear_Error_Flag(R_Motor_Encoder_ERROR);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_SPEED_ERROR)!=0)    Set_SystemError_FLAG(R_Motor_SPEED_ERROR);//速度超差
	else Clear_Error_Flag(R_Motor_SPEED_ERROR);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_VOL_ERROR)!=0)      Set_SystemError_FLAG(R_Motor_VOL_ERROR);//电机参考电压出错
	else Clear_Error_Flag(R_Motor_VOL_ERROR);
	
	if(Get_DriveError_FLAG(right_motor_state,L_Motor_HAL_ERROR)!=0)      Set_SystemError_FLAG(R_Motor_HAL_ERROR);//霍尔出错
	else Clear_Error_Flag(R_Motor_HAL_ERROR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_overCUR)!=0)        Set_SystemError_FLAG(L_Motor_overCUR);//过流
	else Clear_Error_Flag(L_Motor_overCUR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_overLoad)!=0)       Set_SystemError_FLAG(L_Motor_overLoad);//过载
	else Clear_Error_Flag(L_Motor_overLoad);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_CUR_ERROR)!=0)      Set_SystemError_FLAG(L_Motor_CUR_ERROR);//电流超差
	else Clear_Error_Flag(L_Motor_CUR_ERROR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_Encoder_ERROR)!=0)  Set_SystemError_FLAG(L_Motor_Encoder_ERROR);//编码器超差
	else Clear_Error_Flag(L_Motor_Encoder_ERROR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_SPEED_ERROR)!=0)    Set_SystemError_FLAG(L_Motor_SPEED_ERROR);//速度超差
	else Clear_Error_Flag(L_Motor_SPEED_ERROR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_VOL_ERROR)!=0)      Set_SystemError_FLAG(L_Motor_VOL_ERROR);//电机参考电压出错
	else Clear_Error_Flag(L_Motor_VOL_ERROR);
	
	if(Get_DriveError_FLAG(left_motor_state,L_Motor_HAL_ERROR)!=0)      Set_SystemError_FLAG(L_Motor_HAL_ERROR);//霍尔出错
	else Clear_Error_Flag(L_Motor_HAL_ERROR);
}

#endif

