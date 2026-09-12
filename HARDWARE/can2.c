#include "can2.h"
#include "system.h"

/**************************************************************************
Function: CAN2 initialization
Input   : tsjw：Resynchronize the jump time unit, Scope: 1 ~ 3;
 			    tbs2：Time unit of time period 2, range :1~8;
 			    tbs1：Time unit of time period 1, range :1~16;
 			    brp ：Baud rate divider, range :1 to 1024;(We're actually going to add 1, which is 1 to 1024) tq=(brp)*tpclk1
 			    mode：0, normal mode;1. Loop mode;
Output  : 0- Initialization successful;Other - initialization failed
Note: none of the entry parameters (except mode) can be 0
函数功能：CAN2初始化
入口参数：tsjw：重新同步跳跃时间单元，范围:1~3;
 			    tbs2：时间段2的时间单元，范围:1~8;
 			    tbs1：时间段1的时间单元，范围:1~16;
 			    brp ：波特率分频器，范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
 			    mode：0,普通模式;1,回环模式;
返回  值：0-初始化成功; 其他-初始化失败
注意：入口参数(除了mode)均不能为0
波特率/Baud rate=Fpclk1/((tbs1+tbs2+1)*brp)，Fpclk1为36M
                =36M/((3+2+1)*6)
						    =1M
**************************************************************************/

u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	u16 i=0;
 	if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
	tsjw-=1; //Subtract 1 before setting //先减去1.再用于设置
	tbs2-=1;
	tbs1-=1;
	brp-=1;

	//Enable GPIO port clock //使能GPIO端口时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//Enable CAN2 clock //使能CAN2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//Reuse push-pull output //复用推挽输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
 	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOB8复用为CAN2
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOB9复用为CAN2
	
	CAN2->MCR=0x0000;	//Exit sleep mode (setting all bits to 0 at the same time) //退出睡眠模式(同时设置所有位为0)
	CAN2->MCR|=1<<0;	//Request CAN2 to enter initialization mode //请求CAN2进入初始化模式
	while((CAN2->MSR&1<<0)==0)
	{
		i++;
		if(i>100)return 2; //Failed to enter initialization mode //进入初始化模式失败
	}
	
	//Non-time triggered communication mode
	//非时间触发通信模式
	CAN2->MCR|=0<<7;
  //Software automatic offline management	
	//软件自动离线管理
	CAN2->MCR|=0<<6;	
	//Sleep mode is awakened by software (clear CAN2- >;
  //睡眠模式通过软件唤醒(清除CAN2->MCR的SLEEP位)	
	CAN2->MCR|=0<<5;
	//Disallow automatic message transmission
  //禁止报文自动传送	
	CAN2->MCR|=1<<4;
	//Messages are not locked, the new overwrites the old
  //报文不锁定,新的覆盖旧的	
	CAN2->MCR|=0<<3;
	//The priority is determined by the message identifier	
  //优先级由报文标识符决定  
	CAN2->MCR|=0<<2;
  //Clear the original Settings	
  //清除原来的设置	
	CAN2->BTR=0x00000000; 
	//Mode set to 0, normal mode;1. Loop mode;
	//模式设置 0,普通模式;1,回环模式;
	CAN2->BTR|=mode<<30;
  //Resynchronization jump width (TSJW) is TSJW +1 time unit	
	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
	CAN2->BTR|=tsjw<<24; 
  //Tbs2= Tbs2 +1 time unit	
	//Tbs2=tbs2+1个时间单位
	CAN2->BTR|=tbs2<<20; 
	//Tbs1= Tbs1 +1 time unit	
  //Tbs1=tbs1+1个时间单位	
	CAN2->BTR|=tbs1<<16;
	//Frequency division coefficient (Fdiv) is brp +1, boulder rate: Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
  //分频系数(Fdiv)为brp+1，波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
	CAN2->BTR|=brp<<0;  
  //Request CAN2 to exit initialization mode	
  //请求CAN2退出初始化模式			
	CAN2->MCR&=~(1<<0);	  
	while((CAN2->MSR&1<<0)==1)
	{
		i++;
		if(i>0XFFF0)return 3; //Failed to exit initialization mode //退出初始化模式失败
	}
	
	//CAN filter init 2021.07.19
	CAN_SlaveStartBank(14);
  CAN_FilterInitStructure.CAN_FilterNumber=14;//指定过滤器为1
  CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//指定过滤器为标识符屏蔽位模式
  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//过滤器位宽为32位
  CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;// 过滤器标识符的高16位值
  CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//	 过滤器标识符的低16位值
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//过滤器屏蔽标识符的高16位值
  CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;//	过滤器屏蔽标识符的低16位值
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;// 设定了指向过滤器的FIFO为0
  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;// 使能过滤器
  CAN_FilterInit(&CAN_FilterInitStructure);//	按上面的参数初始化过滤器
  //CAN filter init 2021.07.19

#if CAN2_RX0_INT_ENABLE
  //Enable to interrupt reception //使能中断接收
   CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);			
	
	//Configure CAN to receive interrupts
	//配置CAN接收中断
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  //Preemption priority	
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Son priority
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //IRQ channel enablement
	//IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initializes the VIC register according to the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	


	
#endif
	return 0; 
}
/**************************************************************************
Function: CAN sends data
Input   : id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames
			    rtr:0, data frame;1, remote frame
			    len:Length of data to be sent (fixed at 8 bytes, valid data is 6 bytes in time-triggered mode) 
			    *dat:Pointer to the data
Output  : 0~3, mailbox number. 0xFF, no valid mailbox
函数功能：CAN发送数据
入口参数：id:标准ID(11位)/扩展ID(11位+18位)	    
			    ide:0,标准帧;1,扩展帧
			    rtr:0,数据帧;1,远程帧
			    len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
			    *dat:数据指针.
返回  值：0~3,邮箱编号.0XFF,无有效邮箱
**************************************************************************/
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN2->TSR&(1<<26))mbox=0;		  //Mailbox 0 is empty //邮箱0为空
	else if(CAN2->TSR&(1<<27))mbox=1;	//Mailbox 1 is empty //邮箱1为空
	else if(CAN2->TSR&(1<<28))mbox=2;	//Mailbox 2 is empty //邮箱2为空
	else return 0XFF;					        //No empty mailbox, cannot send //无空邮箱,无法发送 
	
	CAN2->sTxMailBox[mbox].TIR=0; //Clear the previous Settings //清除之前的设置		
	if(ide==0) //The standard frame //标准帧
	{
		id&=0x7ff; //Take the low 11 bit STDID //取低11位stdid
		id<<=21;		  
	}else	//Extend the frame //扩展帧
	{
		id&=0X1FFFFFFF; //Take a low 32-bit extid //取低32位extid
		id<<=3;									   
	}
	CAN2->sTxMailBox[mbox].TIR|=id;		 
	CAN2->sTxMailBox[mbox].TIR|=ide<<2;	  
	CAN2->sTxMailBox[mbox].TIR|=rtr<<1;
	len&=0X0F; //Get lower 4 bits //得到低四位
	CAN2->sTxMailBox[mbox].TDTR&=~(0X0000000F);
	CAN2->sTxMailBox[mbox].TDTR|=len;	//Set the DLC	//设置DLC
	//The data to be sent is stored in the mailbox
	//待发送数据存入邮箱
	CAN2->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
								((u32)dat[6]<<16)|
 								((u32)dat[5]<<8)|
								((u32)dat[4]));
	CAN2->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
								((u32)dat[2]<<16)|
 								((u32)dat[1]<<8)|
								((u32)dat[0]));
	CAN2->sTxMailBox[mbox].TIR|=1<<0; //Request to send mailbox data//请求发送邮箱数据
	return mbox;
}
/**************************************************************************
Function: Get the send status
Input   : Mbox: mailbox number
Output  : 0, hang;0X05, send failed;0X07, successful transmission
函数功能：获得发送状态
入口参数：mbox：邮箱编号
返回  值：0,挂起;0X05,发送失败;0X07,发送成功
**************************************************************************/
u8 CAN2_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN2->TSR&(1<<0);			   //RQCP0
			sta |= CAN2->TSR&(1<<1);			   //TXOK0
			sta |=((CAN2->TSR&(1<<26))>>24); //TME0
			break;
		case 1: 
			sta |= CAN2->TSR&(1<<8)>>8;		   //RQCP1
			sta |= CAN2->TSR&(1<<9)>>8;		   //TXOK1
			sta |=((CAN2->TSR&(1<<27))>>25); //TME1	   
			break;
		case 2: 
			sta |= CAN2->TSR&(1<<16)>>16;	   //RQCP2
			sta |= CAN2->TSR&(1<<17)>>16;	   //TXOK2
			sta |=((CAN2->TSR&(1<<28))>>26); //TME2
			break;
		default:
			sta=0X05; //Wrong email number, failed //邮箱号不对,失败
		break;
	}
	return sta;
} 
/**************************************************************************
Function: Returns the number of packets received in FIFO0/FIFO1
Input   : Fifox: FIFO number (0, 1)
Output  : Number of packets in FIFO0/FIFO1
函数功能：得到在FIFO0/FIFO1中接收到的报文个数
入口参数：fifox：FIFO编号（0、1）
返回  值：FIFO0/FIFO1中的报文个数
**************************************************************************/
u8 CAN2_Msg_Pend(u8 fifox)
{
	if(fifox==0)return CAN2->RF0R&0x03; 
	else if(fifox==1)return CAN2->RF1R&0x03; 
	else return 0;
}
/**************************************************************************
Function: Receive data
Input   : fifox：Email
		    	id:Standard ID(11 bits)/ Extended ID(11 bits +18 bits)
			    ide:0, standard frame;1, extension frames 
			    rtr:0, data frame;1, remote frame
			    len:Length of data received (fixed at 8 bytes, valid at 6 bytes in time-triggered mode)
			    dat:Data cache
Output  : none
函数功能：接收数据
入口参数：fifox：邮箱号
		    	id:标准ID(11位)/扩展ID(11位+18位)
			    ide:0,标准帧;1,扩展帧
			    rtr:0,数据帧;1,远程帧
			    len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
			    dat:数据缓存区
返回  值：无 
**************************************************************************/
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide=CAN2->sFIFOMailBox[fifox].RIR&0x04; //Gets the value of the identifier selection bit //得到标识符选择位的值  
 	if(*ide==0) //Standard identifier //标准标识符
	{
		*id=CAN2->sFIFOMailBox[fifox].RIR>>21;
	}else	     //Extended identifier //扩展标识符
	{
		*id=CAN2->sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr=CAN2->sFIFOMailBox[fifox].RIR&0x02;	//Gets the remote send request value //得到远程发送请求值
	*len=CAN2->sFIFOMailBox[fifox].RDTR&0x0F; //Get the DLC //得到DLC
 	//*fmi=(CAN2->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF; //Get the FMI //得到FMI
	//Receive data //接收数据
	dat[0]=CAN2->sFIFOMailBox[fifox].RDLR&0XFF;
	dat[1]=(CAN2->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
	dat[2]=(CAN2->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
	dat[3]=(CAN2->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
	dat[4]=CAN2->sFIFOMailBox[fifox].RDHR&0XFF;
	dat[5]=(CAN2->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
	dat[6]=(CAN2->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
	dat[7]=(CAN2->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
  if(fifox==0)CAN2->RF0R|=0X20;      //Free the FIFO0 mailbox //释放FIFO0邮箱
	else if(fifox==1)CAN2->RF1R|=0X20; //Free the FIFO1 mailbox //释放FIFO1邮箱	 
}
/**************************************************************************
Function: CAN receives interrupt service function, conditional compilation
Input   : none
Output  : none
函数功能：CAN接收中断服务函数，条件编译
入口参数：无
返回  值：无 
更新    ：取消使能步骤，改为接收数据直接开始控制 2021.07.16
**************************************************************************/
#if CAN2_RX0_INT_ENABLE	//Enable RX0 interrupt //使能RX0中断	    
void CAN2_RX0_IRQHandler(void)
{
	u32 id;
	u8 ide,rtr,len;     

	u8 temp_rxbuf[8];

 	CAN2_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
	
	#if USE_CAN2
	//////////////// 编码器数据 ////////////////
	if(id==0x181)
	{
		MOTOR_A.Encoder_Rpm  = (short)((temp_rxbuf[1]<<8)+(temp_rxbuf[0]));
		MOTOR_B.Encoder_Rpm  =-(short)((temp_rxbuf[5]<<8)+(temp_rxbuf[4]));
		Get_Velocity_Form_Encoder();
	}
	//////////////// 编码器数据 ////////////////
	
	//////////////// 驱动状态数据 ////////////////
	else if(id==0x151) //存放电机电流、母线电压
	{
		HUB_Drive.L_motorCurrent = ((short)(temp_rxbuf[1]<<8|temp_rxbuf[0]))*0.1f;
		HUB_Drive.R_motorCurrent = ((short)(temp_rxbuf[3]<<8|temp_rxbuf[2]))*0.1f;
		HUB_Drive.Voltage = ((short)(temp_rxbuf[5]<<8|temp_rxbuf[4]))*0.01f;
	}
	else if(id==0x150) //存放电机、驱动温度
	{
		HUB_Drive.L_motorTemperature = ((short)(temp_rxbuf[1]<<8|temp_rxbuf[0]))*0.1f;
		HUB_Drive.R_motorTemperature = ((short)(temp_rxbuf[3]<<8|temp_rxbuf[2]))*0.1f;
		HUB_Drive.Drive_Temperature  = ((short)(temp_rxbuf[5]<<8|temp_rxbuf[4]))*0.1f;
	}
	//////////////// 驱动状态数据 ////////////////
	
	//电机报错检查，如开启了实时监测，则2秒检查1次
	//无论是否开启实时监测，开机初始化后都会对驱动器检查1次
	//问询方式id = 0x581 / 映射方式id = 0x191
	else if(motor_checkerror_flag&&id==0x581)
	{
		motor_checkerror_flag=0;//检查标志位复位
		
		u16 left_motor_state;
		u16 right_motor_state;
		static u8 need_clear=0;
		
		left_motor_state = temp_rxbuf[5]<<8 | temp_rxbuf[4];
		right_motor_state  = temp_rxbuf[7]<<8 | temp_rxbuf[6];
		
		if(left_motor_state==0&&right_motor_state==0)
		{
			if(need_clear) //标记过错误则清除
			{
				need_clear=0;			
				Self_CheckingFlag &= 0xFFFE0000;//清除与驱动器、电机相关的所有报错
			}
			
		}
		else
		{
			need_clear=1;//存在错误则标记	
			
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
			
	}
	#endif

}
#endif
/**************************************************************************
Function: CAN2 sends a set of data (fixed format :ID 0X601, standard frame, data frame)
Input   : msg:Pointer to the data
    			len:Data length (up to 8)
Output  : 0, success, others, failure;
函数功能：CAN2发送一组数据(固定格式:ID为0X601,标准帧,数据帧)
入口参数：msg:数据指针
    			len:数据长度(最大为8)
返回  值：0,成功，其他,失败;
**************************************************************************/
u8 CAN2_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN2_Tx_Msg(0X601,0,0,len,msg);   
	while((CAN2_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1; //Send failure //发送失败
	return 0;	//Send a success //发送成功									
}
/**************************************************************************
Function: The CAN2 port receives data queries
Input   : Buf: The data cache
Output  : 0, number of data received, other, length of data received
函数功能：CAN2口接收数据查询
入口参数：buf:数据缓存区
返回  值：0,无数据被收到，其他,接收的数据长度
**************************************************************************/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN2_Msg_Pend(0)==0)return 0;			   //No data received, exit directly //没有接收到数据,直接退出 	 
  	CAN2_Rx_Msg(0,&id,&ide,&rtr,&len,buf); //Read the data //读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		 //Receive error //接收错误	   
	return len;	
}
/**************************************************************************
Function: CAN2 sends a set of data tests
Input   : msg:Pointer to the data
			    len:Data length (up to 8)
Output  : 0, success, 1, failure
函数功能：CAN2发送一组数据测试
入口参数：msg:数据指针
			    len:数据长度(最大为8)
返回  值：0,成功，1,失败
**************************************************************************/
u8 CAN2_Send_MsgTEST(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN2_Tx_Msg(0X701,0,0,len,msg);   
	while((CAN2_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1;	//Send failure //发送失败
	return 0;	//Send a success //发送成功
}
/**************************************************************************
Function: Sends an array to the given ID
Input   : id：ID no.
			    msg：The transmitted data pointer
Output  : 0, success, 1, failure
函数功能：给给定的id发送一个数组的命令
入口参数：id：ID号
			    msg：被输送数据指针
返回  值：0,成功，1,失败
**************************************************************************/
u8 CAN2_Send_Num(u32 id,u8* msg)
{
	u8 mbox;
	u16 i=0;	  	 						       
  mbox=CAN2_Tx_Msg(id,0,0,8,msg);   
	while((CAN2_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++; //Waiting for the end of sending //等待发送结束
	if(i>=0XFFF)return 1;	//Send failure //发送失败
	return 0;
}
