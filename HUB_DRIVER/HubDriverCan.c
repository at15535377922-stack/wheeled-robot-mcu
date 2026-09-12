#include "HubDriverCan.h"

HubDrive_parameter HUB_Drive;

//获取设置参数函数
uint8_t* get_setparam(uint8_t rw, uint8_t bytes,uint32_t addr,uint32_t writedata)
{
	static uint8_t user_needdata[8];
	uint8_t askmsg;//访问命令字，根据读写的字节个数不一样改变
	if(rw==0x01) // r=0,w=1,写数据
	{
			 if (bytes==1) askmsg=0x2f;
		else if (bytes==2) askmsg=0x2B;
		else if (bytes==3) askmsg=0x27;
		else if (bytes==4) askmsg=0x23;
        //要写入的数据从低位向高位排序取出
        user_needdata[4] = writedata&0xff;
        user_needdata[5] = (writedata>>8)&0xff;
        user_needdata[6] = (writedata>>16)&0xff;
        user_needdata[7] = (writedata>>24)&0xff;
	}
	else if(rw == 0x10) // r=1,w=0,读数据
	{
			 if (bytes==1) askmsg=0x4f;
		else if (bytes==2) askmsg=0x4B;
		else if (bytes==3) askmsg=0x47;
		else if (bytes==4) askmsg=0x43;
        //读数据时写数据内容为0
        user_needdata[4] = 0;
        user_needdata[5] = 0;
        user_needdata[6] = 0;
        user_needdata[7] = 0;
	}

	user_needdata[0] = askmsg;//命令字
	user_needdata[1] = (addr>>8)&0xff;  //取出地址低8位
	user_needdata[2] = (addr>>16)&0xff; //取出地址高8位
	user_needdata[3] = addr&0xff;       //取出索引值
	return user_needdata;
}

void cui_hubinit(void)
{
	//1.设置全部基础参数
	
}

//2.设置映射值
//3.设置电机控制方式


/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：轮毂速度模式初始化：30ms
入口参数：无
返回  值：无
**************************************************************************/
int acc_time=5; //经测试，加减速时间最低3ms，再低电机会停止响应
void hub_CAN_velocity_init(void)
{
	//CAN_setBaudrate(1000);
	CAN_Asyn_ctrl_set(0);
	delay_xms(5);
	CAN_V_mode_set();
	delay_xms(5);
	CAN_acc_time(acc_time);
	delay_xms(5);
//	CAN_acc_time_left(0);
//	CAN_dec_time_left(0);
//	CAN_acc_time_right(0);
//	CAN_dec_time_right(0);
	
	CAN_Enable();
	delay_xms(5);
}

void hub_CAN_position_init(void)
{
	CAN_P_mode_set();
	CAN_acc_time(acc_time);
	CAN_Enable();
	CAN_SetMaxRpm_left_Right(60, 60);
}

void hub_CAN_torque_init(void)
{
	CAN_Asyn_ctrl_set(1);
	CAN_T_mode_set();
	CAN_Left_Torque_mAs(100);
	CAN_Right_Torque_mAs(100);
	CAN_Enable();
}

void CAN_setBaudrate(int Baudrate)
{
	u8 baudrate[8]= {0x2f, 0x0b, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	switch(Baudrate)
  {
	 case 1000: baudrate[4]=0; break;
	 case 500:  baudrate[4]=1; break;
	 case 250:  baudrate[4]=2; break;
	 case 125:  baudrate[4]=3; break;
	 case 100:  baudrate[4]=4; break;
	 case 50:   baudrate[4]=5; break;
	 case 25:   baudrate[4]=6; break;
  }
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,baudrate);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,baudrate);
	#endif
	delay_xms(100);
	CAN_Write_EEPROM();
	delay_xms(100);
}

void CAN_Write_EEPROM(void)
{
	u8 write_EEPROM[8]= {0x2f, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_EEPROM);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_EEPROM);
	#endif
}

void CAN_Stop(void)
{
	u8 Stop[8]= {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	#if USE_CAN1
	CAN1_Send_Num(0x601,Stop);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,Stop);
	#endif
}

void CAN_ClearError(void)
{
	u8 Clear_Error[8]= {0x2b, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
	#if USE_CAN1
	CAN1_Send_Num(0x601,Clear_Error);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,Clear_Error);
	#endif
}

void CAN_Enable(void)
{
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
	u8 enable3[8]= {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
	
	#if USE_CAN1
	delay_xms(20);
	CAN1_Send_Num(0x601,enable1);
	delay_xms(20);
	CAN1_Send_Num(0x601,enable2);
	delay_xms(20);
	CAN1_Send_Num(0x601,enable3);
	#endif
	
	#if USE_CAN2
	delay_xms(20);
	CAN2_Send_Num(0x601,enable1);
	delay_xms(20);
	CAN2_Send_Num(0x601,enable2);
	delay_xms(20);
	CAN2_Send_Num(0x601,enable3);
	#endif
}

void CAN_Asyn_ctrl_set(int asyn)
{
	u8 asyn_ctrl_set[8]= {0x2b, 0x0f, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	if(asyn==0)asyn_ctrl_set[4]=0x01;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,asyn_ctrl_set);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,asyn_ctrl_set);
	#endif
}

void CAN_V_mode_set(void)
{
	u8 v_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,v_mode_set);
	#endif
	
	#if USE_CAN2
	delay_xms(20);
	CAN2_Send_Num(0x601,v_mode_set);
	#endif
}

void CAN_P_mode_set(void)
{
	u8 p_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
	delay_ms(5);
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,p_mode_set);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,p_mode_set);
	#endif
}

void CAN_T_mode_set(void)
{
	u8 t_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
	
	delay_ms(5);
	#if USE_CAN1
	CAN1_Send_Num(0x601,t_mode_set);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,t_mode_set);
	#endif	
	
}

void CAN_acc_time(int time)
{
	u8 left_acc_time [8]= {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	u8 right_acc_time[8]= {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	u8 left_dec_time [8]= {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	u8 right_dec_time[8]= {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time>0)
		left_acc_time [5]=time>>8,
	  left_acc_time [4]=time;
	  right_acc_time[5]=time>>8,
	  right_acc_time[4]=time;
		left_dec_time [5]=time>>8,
	  left_dec_time [4]=time;
		right_dec_time[5]=time>>8,
	  right_dec_time[4]=time;
	
	#if USE_CAN1
	delay_xms(20);
	CAN1_Send_Num(0x601,left_acc_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,right_acc_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,left_dec_time);
	delay_xms(20);
	CAN1_Send_Num(0x601,right_dec_time);
	#endif
	
	#if USE_CAN2
	delay_xms(20);
	CAN2_Send_Num(0x601,left_acc_time);
	delay_xms(20);
	CAN2_Send_Num(0x601,right_acc_time);
	delay_xms(20);
	CAN2_Send_Num(0x601,left_dec_time);
	delay_xms(20);
	CAN2_Send_Num(0x601,right_dec_time);
	#endif

}

void CAN_acc_time_left(int time)
{
	u8 left_acc_time [8]= {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		left_acc_time [5]=time>>8,
	  left_acc_time [4]=time;

	delay_xms(1);
	#if USE_CAN1
	CAN1_Send_Num(0x601,left_acc_time);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,left_acc_time);
	#endif
}

void CAN_dec_time_left(int time)
{
	u8 left_dec_time [8]= {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		left_dec_time [5]=time>>8,
		left_dec_time [4]=time;

	delay_xms(1);
	#if USE_CAN1
	CAN1_Send_Num(0x601,left_dec_time);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,left_dec_time);
	#endif	
}

void CAN_acc_time_right(int time)
{
	u8 right_acc_time[8]= {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
	  right_acc_time[5]=time>>8,
	  right_acc_time[4]=time;
	
	delay_xms(1);
	#if USE_CAN1
	CAN1_Send_Num(0x601,right_acc_time);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,right_acc_time);
	#endif
	
}

void CAN_dec_time_right(int time)
{
	u8 right_dec_time[8]= {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	if(time<0) time=0;
	if(time>0)
		right_dec_time[5]=time>>8,
	  right_dec_time[4]=time;

	delay_xms(1);
	#if USE_CAN1
	CAN1_Send_Num(0x601,right_dec_time);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,right_dec_time);
	#endif
}

void CAN_SetMaxRpm_left_Right(int Left, int Right)
{
  u8 set_left_max[8]=  {0x23, 0x81, 0x60, 0x01, 0x3C, 0x00, 0x00, 0x00};
  u8 set_right_max[8]= {0x23, 0x81, 0x60, 0x02, 0x3C, 0x00, 0x00, 0x00};
	
	delay_ms(5);
		if(Left>0)
	  set_left_max [5]=Left>>8,
	  set_left_max [4]=Left;
	if(Right>0)
	  set_right_max[5]=Right>>8,
	  set_right_max[4]=Right;

	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_max);
	delay_ms(5);
	CAN1_Send_Num(0x601,set_right_max);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_max);
	delay_ms(5);
	CAN2_Send_Num(0x601,set_right_max);
	#endif
}

/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置左电机转矩斜率,默认500mA/s
入口参数：mAs：转矩斜率，单位mA/s
返回  值：无
**************************************************************************/
void CAN_Left_Torque_mAs(int mAs)
{
  u8 left_torque[8]= {0x23, 0x87, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};

	if(mAs>0)
		left_torque[5]=mAs>>8,
	  left_torque[4]=mAs;
	
	delay_ms(5);	
	#if USE_CAN1
	CAN1_Send_Num(0x601,left_torque);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,left_torque);
	#endif
}
/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置右电机转矩斜率,默认500mA/s
入口参数：mAs：转矩斜率，单位mA/s
返回  值：无
**************************************************************************/
void CAN_Right_Torque_mAs(int mAs)
{
  u8 right_torque[8]= {0x23, 0x87, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00};

	if(mAs>0)
		right_torque[5]=mAs>>8,
	  right_torque[4]=mAs;
	
	delay_ms(5);	
	#if USE_CAN1
	CAN1_Send_Num(0x601,right_torque);	
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,right_torque);
	#endif
	
}

/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置轮毂速度
入口参数：±20480
返回  值：无
**************************************************************************/
void hub_CAN_Rpm(int R_L, int rpm)
{
	u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00};
	
	//设置左电机还是右电机
	if     (R_L==0) set_rpm[3]=0x01; //左电机
	else if(R_L==1) set_rpm[3]=0x02; //右电机
	
	//设置转速与方向
	if(rpm>=0)
	{
		set_rpm[4]=rpm;
		set_rpm[5]=rpm>>8;
	}
	else
	{
		set_rpm[4]=(0xffff+rpm+1);
		set_rpm[5]=(0xffff+rpm+1)>>8;
	}
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_rpm);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_rpm);
	#endif
	
}

void hub_CAN_Syn_Rpm(int left_rpm, int right_rpm)
{
  u8 set_rpm[8]= {0x23, 0xFF, 0x60, 0x03, 0x64, 0x00, 0x64, 0x00};
	
	//设置转速与方向
	if(left_rpm>=0)
	{
		set_rpm[4]=left_rpm;
		set_rpm[5]=left_rpm>>8;
	}
	else
	{
		set_rpm[4]=(0xffff+left_rpm+1);
		set_rpm[5]=(0xffff+left_rpm+1)>>8;
	}
	if(right_rpm>=0)
	{
		set_rpm[6]=right_rpm;
		set_rpm[7]=right_rpm>>8;
	}
	else
	{
		set_rpm[6]=(0xffff+right_rpm+1);
		set_rpm[7]=(0xffff+right_rpm+1)>>8;
	}	
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_rpm);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_rpm);
	#endif
	
}


/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置轮毂位置
入口参数：无
返回  值：无
**************************************************************************/
void hub_CAN_Rel_Position(int R_L, long int position)
{
	u8 set_position[8]= {0x23, 0x7a, 0x60, 0x01, 0x00, 0x7d, 0x00, 0x00};
  u8 set_start1[8]=   {0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00};
  u8 set_start2[8]=   {0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00};
	
	//设置左电机还是右电机
	if     (R_L==0) set_position[3]=0x01; //左电机
	else if(R_L==1) set_position[3]=0x02; //右电机
	
	if(position>=0)
	{
		set_position[7] = position>>24;
		set_position[6] = position>>16;
		set_position[5] = position>>8;
		set_position[4] = position;
	}
	else
	{
		set_position[7] =(0xffffffff+position+1)>>24;
		set_position[6] =(0xffffffff+position+1)>>16;
		set_position[5] =(0xffffffff+position+1)>>8;
		set_position[4] =(0xffffffff+position+1);
	}
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_position);
	delay_xms(1);
	CAN1_Send_Num(0x601,set_start1);
	delay_xms(1);
	CAN1_Send_Num(0x601,set_start2);
	delay_xms(1);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_position);
	delay_xms(1);
	CAN2_Send_Num(0x601,set_start1);
	delay_xms(1);
	CAN2_Send_Num(0x601,set_start2);
	delay_xms(1);
	#endif
}

void hub_CAN_Abs_Position(int R_L, long int position)
{
	u8 set_position[8]= {0x23, 0x7a, 0x60, 0x01, 0x00, 0x7d, 0x00, 0x00};
	u8 set_start1[8]=   {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
	u8 set_start2[8]=   {0x2b, 0x40, 0x60, 0x00, 0x1f, 0x00, 0x00, 0x00};
	
	//设置左电机还是右电机
	if     (R_L==0) set_position[3]=0x01; //左电机
	else if(R_L==1) set_position[3]=0x02; //右电机
	
	if(position>=0)
	{
		set_position[7] = position>>24;
		set_position[6] = position>>16;
		set_position[5] = position>>8;
		set_position[4] = position;
	}
	else
	{
		set_position[7] =(0xffffffff+position+1)>>24;
		set_position[6] =(0xffffffff+position+1)>>16;
		set_position[5] =(0xffffffff+position+1)>>8;
		set_position[4] =(0xffffffff+position+1);
	}
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_position);
	delay_xms(1);
	CAN1_Send_Num(0x601,set_start1);
	delay_xms(1);
	CAN1_Send_Num(0x601,set_start2);
	delay_xms(1);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_position);
	delay_xms(1);
	CAN2_Send_Num(0x601,set_start1);
	delay_xms(1);
	CAN2_Send_Num(0x601,set_start2);
	delay_xms(1);
	#endif
}

//驱动器报错查询
void CAN_CheckError(void)
{
	u8 check_error[8] = {0x43,0x3f,0x60,0x00,0x00,0x00,0x00,0x00};
	#if USE_CAN1
	CAN1_Send_Num(0x601,check_error);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,check_error);
	#endif
	//指令发送后，驱动会返回帧ID为0x581的数据，其中有效数据为低4字节
	//示例 【帧ID 0x581 数据 0x43,0x3f,0x60,0x00,0x?,0x?,0x?,0x】
	//当后面4字节的0x?全部为0x00时，表示驱动无异常
	
	#if 0
	//读取驱动版本号
	delay_xms(1);
	u8 check_Version[8]={ 0x43,0x31,0x20,0,0,0,0,0 };
	CAN2_Send_Num(0x601,check_Version);
	
	//读取IO口急停模式
	delay_xms(1);
	u8 see_io_mode[8]={ 0x43,0x26,0x20,0x03,0,0,0,0 };
	CAN2_Send_Num(0x601,see_io_mode);
	#endif
}


/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置轮毂转矩
入口参数：±2000ma
返回  值：无
**************************************************************************/
void hub_CAN_Torque(int R_L, int torque)
{
  u8 set_torque[8]= {0x2b, 0x71, 0x60, 0x01, 0xe8, 0x03, 0x00, 0x00};
	
	//设置左电机还是右电机
	if     (R_L==0) set_torque[3]=0x01; //左电机
	else if(R_L==1) set_torque[3]=0x02; //右电机
	
	//设置转速与方向
	if(torque>=0)
	{
		set_torque[7] = torque>>24;
		set_torque[6] = torque>>16;
		set_torque[5] = torque>>8;
		set_torque[4] = torque;
	}
	else
	{
		set_torque[7] =(0xffffffff+torque+1)>>24;
		set_torque[6] =(0xffffffff+torque+1)>>16;
		set_torque[5] =(0xffffffff+torque+1)>>8;
		set_torque[4]= (0xffffffff+torque+1);
	}
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_torque);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_torque);
	#endif
	
}
/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：设置轮毂同步转矩
入口参数：±2000ma
返回  值：无
**************************************************************************/
void hub_CAN_Syn_Torque(int torque)
{
	u8 set_torque[8]= {0x23, 0x71, 0x60, 0x03, 0xe8, 0x03, 0x00, 0x00};
	
	if(torque>=0)
	{
		set_torque[7] = torque>>8;
		set_torque[6] = torque;
		set_torque[5] = torque>>8;
		set_torque[4] = torque;
	}
	else
	{
		set_torque[7] =(0xffff+torque+1)>>8;
		set_torque[6] =(0xffff+torque+1);
		set_torque[5] =(0xffff+torque+1)>>8;
		set_torque[4]= (0xffff+torque+1);
	}
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_torque);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_torque);
	#endif
}

/**************************************************************************
Function: 
Input   : none
Output  : none
函数功能：开启读取实时编码器
入口参数：±2000ma
返回  值：无
**************************************************************************/
void hub_CAN_Encoder_init(void)
{
	u8 Clean_TPD00 [8]= {0x2f, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00}; //清空TPDO0上的所有映射
	
	u8 Mapping_01_V  [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x6C, 0x60};//映射字典0x606C 01 （左电机转速）到 0x1A01 01 中，数据长度为32字节
	u8 Mapping_02_V  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x6C, 0x60};//映射字典0x606C 02 （右电机转速）到 0x1A01 02 中，数据长度为32字节
	
//	u8 Mapping_01_P  [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x64, 0x60}; //位置：64 60, 转矩：71 60, 温度：32 20
//	u8 Mapping_02_P  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x64, 0x60};
//	
//	u8 Mapping_01_T  [8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x01, 0x71, 0x60}; //位置：64 60, 转矩：71 60, 温度：32 20
//	u8 Mapping_02_T  [8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x02, 0x71, 0x60};
	
	u8 Set_ID_181  [8]= {0x23, 0x00, 0x18, 0x01, 0x85, 0x01, 0x00, 0x00}; //设置COB-ID（CAN ID）为0x150 
	u8 Set_event   [8]= {0x2f, 0x00, 0x18, 0x02, 0xfe, 0x00, 0x00, 0x00}; //设置254事件（当被映射的数据有变化或计时器已经到达，则发送CAN数据）
	u8 Set_hibTime [8]= {0x2b, 0x00, 0x18, 0x03, 0xc8, 0x00, 0x00, 0x00}; //间隔时间：20ms=(0x01*256+0xc8)/10
	u8 StartMapping[8]= {0x2f, 0x00, 0x1a, 0x00, 0x02, 0x00, 0x00, 0x00}; //开启2个TPDO0映射
	u8 SaveEEPROM  [8]= {0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00}; //保存参数到EEPROM
	u8 Enter       [8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //启动映射工作，数据开始输出
	
	//映射后数据存放格式
	//     ID           0         1           2          3         4            5       6       7
	//CAN-ID:0x181  左电机低8位 左电机高8位     0           0    右电机低8位   左电机高8位   0       0
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,Clean_TPD00);
	delay_xms(15);
	CAN1_Send_Num(0x601,Mapping_01_V);
	delay_xms(15);
	CAN1_Send_Num(0x601,Mapping_02_V);
	delay_xms(15);
	CAN1_Send_Num(0x601,Set_ID_181);
	delay_xms(15);
	CAN1_Send_Num(0x601,Set_event);
	delay_xms(15);
	CAN1_Send_Num(0x601,Set_hibTime);
	delay_xms(15);
	CAN1_Send_Num(0x601,StartMapping);
	delay_xms(15);
	CAN1_Send_Num(0x601,SaveEEPROM);
	delay_xms(15);
	CAN1_Send_Num(0x00,Enter);
	delay_xms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,Clean_TPD00);
	delay_xms(15);
	CAN2_Send_Num(0x601,Mapping_01_V);
	delay_xms(15);
	CAN2_Send_Num(0x601,Mapping_02_V);
	delay_xms(15);
	CAN2_Send_Num(0x601,Set_ID_181);
	delay_xms(15);
	CAN2_Send_Num(0x601,Set_event);
	delay_xms(15);
	CAN2_Send_Num(0x601,Set_hibTime);
	delay_xms(15);
	CAN2_Send_Num(0x601,StartMapping);
	delay_xms(15);
	CAN2_Send_Num(0x601,SaveEEPROM);
	delay_xms(15);
	CAN2_Send_Num(0x00,Enter);
	delay_xms(15);
	#endif
}

//映射电机温度、驱动温度到TPDO1
void hub_MotorTemperature_Init(void)
{
	u8 Clear_TPDO1[8] = {0x2f,0x01,0x1a,0x00,0,0,0,0}; //清除TPDO1上的所有映射
	u8 Mapping_LeftMotor[8] =  {0x23,0x01,0x1a,0x01,0x10,0x01,0x32,0x20}; //映射字典0x2032 01 （左电机温度）到 0x1A01 01 中，数据长度为16字节
	u8 Mapping_RightMotor[8] = {0x23,0x01,0x1a,0x02,0x10,0x02,0x32,0x20}; //映射字典0x2032 02（右电机温度） 到 0x1A01 02 中，数据长度为16字节
	u8 Mapping_Drive[8] =      {0x23,0x01,0x1a,0x03,0x10,0x03,0x32,0x20}; //映射字典0x2032 03  （驱动温度） 到 0x1A01 03 中，数据长度为16字节
	u8 Set_COBID[8] = {0x23,0x01,0x18,0x01,0x50,0x01,0x00,0x00}; //设置COB-ID（CAN ID）为0x150 
	u8 Set_Event[8] = {0x2f,0x01,0x18,0x02,0xfe,0x00,0x00,0x00}; //设置254事件（当被映射的数据有变化或计时器已经到达，则发送CAN数据）
	u8 Set_Data_Send_MinTime[8] = {0x2b,0x01,0x18,0x03,0x88,0x13,0x00,0x00}; //设置最小定时时间 0x1388->5000us->500ms
	u8 Start_Mapping[8] = {0x2f,0x01,0x1a,0x00,0x03,0x00,0x00,0x00}; //开启3个TPDO1映射
	u8 SaveEEPROM[8] = {0x2b,0x10,0x20,0x00,0x01,0x00,0x00,0x00}; //保存参数到EEPROM
	u8 Enter     [8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//启动映射工作
	
	//映射后数据存放格式
	//     ID           0         1           2          3         4        5       6       7
	//CAN-ID:0x150  左电机低8位 左电机高8位 右电机低8位 右电机高8位 驱动低8位 驱动高8位   0       0
	#if USE_CAN1
	CAN1_Send_Num(0x601,Clear_TPDO1);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_LeftMotor);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_RightMotor);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_Drive);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);	
	CAN1_Send_Num(0x601,Start_Mapping);
	delay_ms(15);
	CAN1_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN1_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,Clear_TPDO1);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_LeftMotor);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_RightMotor);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_Drive);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);	
	CAN2_Send_Num(0x601,Start_Mapping);
	delay_ms(15);
	CAN2_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN2_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
}

//映射电机电流、驱动母线电压到TPDO2
void hub_MotorCurrent_Init(void)
{
	u8 Clear_TPDO2[8] = {0x2f,0x02,0x1a,0x00,0,0,0,0}; //清除TPDO2上的所有映射
	u8 Mapping_MotorCurrent[8] =  {0x23,0x02,0x1a,0x01,0x20,0x03,0x77,0x60}; //映射字典0x6077 03 （左右电机电流）到 0x1A02 01 中，数据长度为32字节
	u8 Mapping_DriveVoltage[8] =  {0x23,0x02,0x1a,0x02,0x10,0x00,0x35,0x20}; //映射字典0x2035 00  （母线电压） 到 0x1A01 02 中，数据长度为16字节
	u8 Set_COBID[8] = {0x23,0x02,0x18,0x01,0x51,0x01,0x00,0x00}; //设置COB-ID（CAN ID）为0x151 
	u8 Set_Event[8] = {0x2f,0x02,0x18,0x02,0xfe,0x00,0x00,0x00}; //设置254事件（当被映射的数据有变化或计时器已经到达，则发送CAN数据）
	u8 Set_Data_Send_MinTime[8] = {0x2b,0x02,0x18,0x03,0x88,0x13,0x00,0x00}; //设置最小定时时间 0x1388->5000us->500ms
	u8 Start_Mapping[8] = {0x2f,0x02,0x1a,0x00,0x02,0x00,0x00,0x00}; //开启2个TPDO1映射
	u8 SaveEEPROM[8] = {0x2b,0x10,0x20,0x00,0x01,0x00,0x00,0x00}; //保存参数到EEPROM
	u8 Enter     [8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//启动映射工作
	
	//映射后数据存放格式
	//     ID           0         1           2          3         4        5       6       7
	//CAN-ID:0x151  左电机低8位 左电机高8位 右电机低8位 右电机高8位 电压低8位 电压高8位   0       0
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,Clear_TPDO2);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_MotorCurrent);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_DriveVoltage);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);
	CAN1_Send_Num(0x601,Start_Mapping);
	delay_ms(15);	
	CAN1_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN1_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
	
	#if USE_CAN2

	CAN2_Send_Num(0x601,Clear_TPDO2);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_MotorCurrent);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_DriveVoltage);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);
	CAN2_Send_Num(0x601,Start_Mapping);
	delay_ms(15);	
	CAN2_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN2_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
}

//映射驱动报错参数
void hub_MotorERROR_Init(void)
{
	u8 Clear_TPDO3[8] = {0x2f,0x03,0x1a,0x00,0,0,0,0}; //清除TPDO3上的所有映射
	u8 Mapping_MotorState[8] =  {0x23,0x03,0x1a,0x01,0x20,0x00,0x3f,0x60}; //映射字典0x603f 00 （左右电机电流）到 0x1A03 01 中，数据长度为32字节

	u8 Set_COBID[8] = {0x23,0x03,0x18,0x01,0x91,0x01,0x00,0x00}; //设置COB-ID（CAN ID）为0x191 
	u8 Set_Event[8] = {0x2f,0x03,0x18,0x02,0xff,0x00,0x00,0x00}; //设置255事件（当计数到设定时间时，则发送CAN数据）
	u8 Set_Data_Send_MinTime[8] = {0x2b,0x03,0x18,0x03,0x10,0x27,0x00,0x00}; //设置最小定时时间 0x2710->10000->1000ms
	u8 Start_Mapping[8] = {0x2f,0x03,0x1a,0x00,0x01,0x00,0x00,0x00}; //开启1个TPDO1映射
	u8 SaveEEPROM[8] = {0x2b,0x10,0x20,0x00,0x01,0x00,0x00,0x00}; //保存参数到EEPROM
	u8 Enter     [8]= {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};//启动映射工作
	
	//映射后数据存放格式
	//     ID           0         1           2          3         4        5       6       7
	//CAN-ID:0x191  左电机低8位 左电机高8位 右电机低8位 右电机高8位     0       0   0       0
	#if USE_CAN1
	CAN1_Send_Num(0x601,Clear_TPDO3);
	delay_ms(15);
	CAN1_Send_Num(0x601,Mapping_MotorState);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN1_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);
	CAN1_Send_Num(0x601,Start_Mapping);
	delay_ms(15);	
	CAN1_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN1_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,Clear_TPDO3);
	delay_ms(15);
	CAN2_Send_Num(0x601,Mapping_MotorState);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_COBID);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Event);
	delay_ms(15);
	CAN2_Send_Num(0x601,Set_Data_Send_MinTime);
	delay_ms(15);
	CAN2_Send_Num(0x601,Start_Mapping);
	delay_ms(15);	
	CAN2_Send_Num(0x601,SaveEEPROM);
	delay_ms(15);
	CAN2_Send_Num(0x00,Enter);
	delay_ms(15);
	#endif
}

//设置急停口的模式
void set_IO_mode(u8 mode)
{
	u8  set_io_mode[8] =  {0x2b,0x26,0x20,0x03,0,0,0,0};
	
	set_io_mode[4] = mode;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_io_mode);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_io_mode);
	delay_ms(15);
	#endif
}


//设置左右轮速度环KP参数
//注：PID参数不需要负数,设置范围0~30000
void hub_speedloop_kp(u16 L_kp,u16 R_kp)
{
	u8  set_left_kp[8] = {0x2b,0x1d,0x20,0x01,0,0,0,0};
	u8 set_right_kp[8] = {0x2b,0x1d,0x20,0x02,0,0,0,0};
	
	if(L_kp>30000) L_kp=30000;
	if(R_kp>30000) R_kp=30000;
	
	 set_left_kp[4] = L_kp;
	 set_left_kp[5] = L_kp>>8;
	set_right_kp[4] = R_kp;
	set_right_kp[5] = R_kp>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_kp);
	delay_ms(30);
	CAN1_Send_Num(0x601,set_right_kp);
	delay_ms(30);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_kp);
	delay_ms(30);
	CAN2_Send_Num(0x601,set_right_kp);
	delay_ms(30);
	#endif
}

//设置左右轮速度环Ki参数
//注：PID参数不需要负数,设置范围0~30000
void hub_speedloop_ki(u16 L_ki,u16 R_ki)
{
	u8  set_left_ki[8] = {0x2b,0x1e,0x20,0x01,0,0,0,0};
	u8 set_right_ki[8] = {0x2b,0x1e,0x20,0x02,0,0,0,0};
	
	if(L_ki>30000) L_ki=30000;
	if(R_ki>30000) R_ki=30000;
	
	 set_left_ki[4] = L_ki;
	 set_left_ki[5] = L_ki>>8;
	set_right_ki[4] = R_ki;
	set_right_ki[5] = R_ki>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_ki);
	delay_ms(30);
	CAN1_Send_Num(0x601,set_right_ki);
	delay_ms(30);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_ki);
	delay_ms(30);
	CAN2_Send_Num(0x601,set_right_ki);
	delay_ms(30);
	#endif
}

//设置电机过载保护时间,单位*10ms
//范围0~6553
void hub_set_overload_time(u16 L_time,u16 R_time)
{
	u8  set_left_time[8] = {0x2b,0x16,0x20,0x01,0,0,0,0};
	u8 set_right_time[8] = {0x2b,0x16,0x20,0x02,0,0,0,0};
	
	if(L_time>6553) L_time=6553;
	if(R_time>6553) R_time=6553;
	
	 set_left_time[4] = L_time;
	 set_left_time[5] = L_time>>8;
	set_right_time[4] = R_time;
	set_right_time[5] = R_time>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_time);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_time);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_time);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_time);
	delay_ms(15);
	#endif
	
}

//设置电机过载保护系数,单位*1%
//设置范围0~300
void hub_set_overload_factor(u16 L_factor,u16 R_factor)
{
	u8  set_left_factor[8] = {0x2b,0x12,0x20,0x01,0,0,0,0};
	u8 set_right_factor[8] = {0x2b,0x12,0x20,0x02,0,0,0,0};
	
	if(L_factor>300) L_factor=300;
	if(R_factor>300) R_factor=300;
	
	 set_left_factor[4] = L_factor;
	 set_left_factor[5] = L_factor>>8;
	set_right_factor[4] = R_factor;
	set_right_factor[5] = R_factor>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_factor);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_factor);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_factor);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_factor);
	delay_ms(15);
	#endif
}

//设置同步控制
void hub_set_SynchronousControl_Enable(void)
{
	u8 data[8]={0x2b,0x0f,0x20,0x00,0x01,0x00,0x00,0x00};//0x200F 00 
	#if USE_CAN1
	CAN1_Send_Num(0x601,data);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,data);
	delay_ms(15);
	#endif
}

//设置异步控制
void hub_set_SynchronousControl_Disable(void)
{
	u8 data[8]={0x2b,0x0f,0x20,0x00,0x00,0x00,0x00,0x00};//0x200F 00 
	#if USE_CAN1
	CAN1_Send_Num(0x601,data);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,data);
	delay_ms(15);
	#endif
}

//设置编码器线数
//范围0~4096
void hub_set_encoder_accuracy(u16 L_encoder,u16 R_encoder)
{
	u8  set_left_encoder[8] = {0x2b,0x0e,0x20,0x01,0,0,0,0};
	u8 set_right_encoder[8] = {0x2b,0x0e,0x20,0x02,0,0,0,0};
	
	if(L_encoder>4096) L_encoder=4096;
	if(R_encoder>4096) R_encoder=4096;
	
	 set_left_encoder[4] = L_encoder;
	 set_left_encoder[5] = L_encoder>>8;
	set_right_encoder[4] = R_encoder;
	set_right_encoder[5] = R_encoder>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_encoder);
	delay_ms(30);
	CAN1_Send_Num(0x601,set_right_encoder);
	delay_ms(30);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_encoder);
	delay_ms(30);
	CAN2_Send_Num(0x601,set_right_encoder);
	delay_ms(30);
	#endif
}

//设置电机极对数
//范围4~64
void hub_set_motor_poles(u16 L_pole,u16 R_pole)
{
	u8  set_left_pole[8] = {0x2b,0x0c,0x20,0x01,0,0,0,0};
	u8 set_right_pole[8] = {0x2b,0x0c,0x20,0x02,0,0,0,0};
	
	 if(L_pole<4) L_pole=4;
	 if(R_pole<4) R_pole=4;
	if(L_pole>64) L_pole=64;
	if(R_pole>64) R_pole=64;
	
	 set_left_pole[4] = L_pole;
	 set_left_pole[5] = L_pole>>8;
	set_right_pole[4] = R_pole;
	set_right_pole[5] = R_pole>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_pole);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_pole);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_pole);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_pole);
	delay_ms(15);
	#endif
}

//设置电机与Hall偏移角度
//范围-360~+360
void hub_set_motor_OffsetAngle(int L_angle,int R_angle)
{
	u8  set_left_angle[8] = {0x2b,0x11,0x20,0x01,0,0,0,0};
	u8 set_right_angle[8] = {0x2b,0x11,0x20,0x02,0,0,0,0};
	
	 if(L_angle<-360) L_angle=-360;
	 if(R_angle<-360) R_angle=-360;
	if(L_angle>360) R_angle=360;
	if(R_angle>360) R_angle=360;
	
	if(L_angle<0)
	{
		set_left_angle[4] =0xffff+L_angle+1;
		set_left_angle[5] =(0xffff+L_angle+1)>>8;	
	}
	else
	{
		 set_left_angle[4] = L_angle;
		 set_left_angle[5] = L_angle>>8;
	}
	
	if(R_angle<0)
	{
		set_right_angle[4] =0xffff+R_angle+1;
		set_right_angle[5] =(0xffff+R_angle+1)>>8;
	}
	else
	{
		set_right_angle[4] = R_angle;
		set_right_angle[5] = R_angle>>8;
	}

	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_angle);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_angle);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_angle);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_angle);
	delay_ms(15);
	#endif
}

//设置电机最大转速
//范围1~1000
void set_max_motor_speed(u16 speed)
{
	u8  set_max_speed[8] = {0x2b,0x08,0x20,0x00,0,0,0,0};
	
	 if(speed<1) speed=1;
	 if(speed>1000) speed=1000;
	
	 set_max_speed[4] = speed;
	 set_max_speed[5] = speed>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_max_speed);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_max_speed);
	delay_ms(15);
	#endif
}

//设置电机额定电流
//范围0~150，单位*0.1A
void set_motor_rated_current(u16 L_cur,u16 R_cur)
{
	u8  set_left_cur[8] = {0x2b,0x14,0x20,0x01,0,0,0,0};
	u8 set_right_cur[8] = {0x2b,0x14,0x20,0x02,0,0,0,0};
	
	if(L_cur>150) L_cur=150;
	if(R_cur>150) R_cur=150;
	
	 set_left_cur[4] = L_cur;
	 set_left_cur[5] = L_cur>>8;
	set_right_cur[4] = R_cur;
	set_right_cur[5] = R_cur>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_cur);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_cur);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_cur);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_cur);
	delay_ms(15);
	#endif
}

//设置电机额定电流
//范围0~300，单位*0.1A
void set_motor_max_current(u16 L_cur,u16 R_cur)
{
	u8  set_left_cur[8] = {0x2b,0x15,0x20,0x01,0,0,0,0};
	u8 set_right_cur[8] = {0x2b,0x15,0x20,0x02,0,0,0,0};
	
	if(L_cur>300) L_cur=300;
	if(R_cur>300) R_cur=300;
	
	 set_left_cur[4] = L_cur;
	 set_left_cur[5] = L_cur>>8;
	set_right_cur[4] = R_cur;
	set_right_cur[5] = R_cur>>8;
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,set_left_cur);
	delay_ms(15);
	CAN1_Send_Num(0x601,set_right_cur);
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,set_left_cur);
	delay_ms(15);
	CAN2_Send_Num(0x601,set_right_cur);
	delay_ms(15);
	#endif
}

//设置驱动超差报警阈值Out-of-tolerance alarm threshold 1~6553  0x2017 01 02 U16
void set_tolerance_alarm_threshold(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x17,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x17,0x20,0x02,0,0,0,0};
	
	if(L_Data>6553) L_Data=6553;
	if(R_Data>6553) R_Data=6553;
	
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置温度保护阈值Temperature protection threshold 0~1200 0x2013 01 02 U16
void set_Temp_PH(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x13,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x13,0x20,0x02,0,0,0,0};
	if(L_Data>1200) L_Data=1200;
	if(R_Data>1200) R_Data=1200;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置位置环Kp 0~30000 0x2020 01 02 U16
void set_hub_posloop_kp(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x20,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x20,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置位置前馈Kf 0~30000 0x2021 01 02 U16
void set_hub_posloop_kf(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x21,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x21,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置速度前馈kf 0~30000 0x201F 01 02 U16
void set_hub_speedloop_kf(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x1f,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x1f,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置速度平滑系数Speed smoothing coefficient 0~30000 0x2018 01 02 U16
void set_speed_smooth(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x18,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x18,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置前馈平滑系数0~30000 0x201B 01 02 U16
void set_smooth_kf(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x1b,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x1b,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置转矩平滑系数0~30000 0x201C 01 02 U16
void set_torque_smooth(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x1c,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x1c,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置电流环kp 0~30000 0x2019 01 02 U16
void set_hub_curloop_kp(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x19,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x19,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置电流环Ki 0~30000 0x201A 01 02 U16
void set_hub_curloop_ki(u16 L_Data,u16 R_Data)
{
	u8 write_data_L[8] = {0x2b,0x1a,0x20,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x2b,0x1a,0x20,0x02,0,0,0,0};
	if(L_Data>30000) L_Data=30000;
	if(R_Data>30000) R_Data=30000;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置电机加速时间0~32767 0x6083 01 02 U32
void set_hub_Acctime(u32 L_Data,u32 R_Data)
{
	u8 write_data_L[8] = {0x23,0x83,0x60,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x23,0x83,0x60,0x02,0,0,0,0};
	if(L_Data>32767) L_Data=32767;
	if(R_Data>32767) R_Data=32767;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_L[6] = L_Data>>16;
	write_data_L[7] = L_Data>>24;
	
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	write_data_R[6] = R_Data>>16;
	write_data_R[7] = R_Data>>24;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置电机减速时间0~32767 0x6084 01 02 U32
void set_hub_Dectime(u32 L_Data,u32 R_Data)
{
	u8 write_data_L[8] = {0x23,0x84,0x60,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x23,0x84,0x60,0x02,0,0,0,0};
	if(L_Data>32767) L_Data=32767;
	if(R_Data>32767) R_Data=32767;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_L[6] = L_Data>>16;
	write_data_L[7] = L_Data>>24;
	
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	write_data_R[6] = R_Data>>16;
	write_data_R[7] = R_Data>>24;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}
//设置急停减速时间0~32767 0x6085 01 02 U32
void set_hub_StopDectime(u32 L_Data,u32 R_Data)
{
	u8 write_data_L[8] = {0x23,0x85,0x60,0x01,0,0,0,0};
	u8 write_data_R[8] = {0x23,0x85,0x60,0x02,0,0,0,0};
	if(L_Data>32767) L_Data=32767;
	if(R_Data>32767) R_Data=32767;
	write_data_L[4] = L_Data;
	write_data_L[5] = L_Data>>8;
	write_data_L[6] = L_Data>>16;
	write_data_L[7] = L_Data>>24;
	
	write_data_R[4] = R_Data;
	write_data_R[5] = R_Data>>8;
	write_data_R[6] = R_Data>>16;
	write_data_R[7] = R_Data>>24;
	#if USE_CAN1
	CAN1_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN1_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
	#if USE_CAN2
	CAN2_Send_Num(0x601,write_data_L);
	delay_ms(15);
	CAN2_Send_Num(0x601,write_data_R);
	delay_ms(15);
	#endif
}

//驱动器重要参数设置（pid大小、编码器线数、过载系数和时间）
void hub_mainParam_set(void)
{
	u8 SaveEEPROM[8]={0x2B,0x10,0x20,0x00,0x01,0x00,0x00,0x00};
	
	//以下配置顺序按照上位机参数顺序配置 
	if(Car_Mode==S300 || Car_Mode == S130 ) //8寸轮毂电机
	{
		hub_set_SynchronousControl_Enable();              //设置电机同步控制
		hub_set_encoder_accuracy(1024,1024);delay_ms(50); //设置编码器线数为1024	
		hub_set_motor_OffsetAngle(0,0);delay_ms(50);      //设置电机与Hall偏移角度	
		hub_set_motor_poles(15,15);delay_ms(15);          //设置电机极对数	
		set_motor_rated_current(150,150);delay_ms(5);     //设置额定电流	
		set_motor_max_current(300,300);delay_ms(5);       //设置最大电流	
		set_max_motor_speed(1000);delay_ms(5);            //设置额定转速	
		hub_set_overload_factor(200,200);delay_ms(5);     //过载保护系数为200%	
		hub_set_overload_time(800,800);delay_ms(5);       //过载保护时间为800*10ms=8s	(S系列机器人拥有自动回充功能，需要把过载保护时间加长)
		set_tolerance_alarm_threshold(409,409);delay_ms(5);//设置超差报警阈值
		set_Temp_PH(800,800);delay_ms(5);                  //设置温度保护阈值80度
		hub_speedloop_kp(500,500);delay_ms(5); //速度环kp	
		hub_speedloop_ki(333,333);delay_ms(5); //速度环ki
		set_hub_speedloop_kf(1000,1000);delay_ms(5);//速度前馈kf
		set_speed_smooth(50,50);delay_ms(5);//设置速度平滑系数
		set_smooth_kf(100,100);delay_ms(5);//设置前馈平滑系数
	}
	else //5寸轮毂电机
	{
		hub_set_SynchronousControl_Enable();              //设置电机同步控制
		
		#if 1 //V2.0版本轮毂电机
		hub_set_encoder_accuracy(4096,4096);delay_ms(50); //设置编码器线数为4096
		#else //V1.0版本轮毂电机
		hub_set_encoder_accuracy(1024,1024);delay_ms(50); //设置编码器线数为1024
		#endif
		
		hub_set_motor_OffsetAngle(240,240);delay_ms(50);      //设置电机与Hall偏移角度	
		hub_set_motor_poles(10,10);delay_ms(15);          //设置电机极对数	
		set_motor_rated_current(150,150);delay_ms(5);     //设置额定电流	
		set_motor_max_current(300,300);delay_ms(5);       //设置最大电流	
		set_max_motor_speed(1000);delay_ms(5);            //设置额定转速	
		hub_set_overload_factor(200,200);delay_ms(5);     //过载保护系数为200%	
		hub_set_overload_time(800,800);delay_ms(5);       //过载保护时间为800*10ms=8s	(S系列机器人拥有自动回充功能，需要把过载保护时间加长)
		set_tolerance_alarm_threshold(1638,1638);delay_ms(5);//设置超差报警阈值
		set_Temp_PH(800,800);delay_ms(5);                  //设置温度保护阈值80度
		hub_speedloop_kp(300,300);delay_ms(5); //速度环kp	
		hub_speedloop_ki(200,200);delay_ms(5); //速度环ki
		set_hub_speedloop_kf(1000,1000);delay_ms(5);//速度前馈kf
		set_speed_smooth(50,50);delay_ms(5);//设置速度平滑系数
		set_smooth_kf(100,100);delay_ms(5);//设置前馈平滑系数
	}
	
	/* 注：只用到速度控制，不可对另外两个环的PID进行设置，设置会使电机发生抖动 */
	//set_hub_posloop_kp(50,50);delay_ms(5);//位置环kp
	//set_hub_posloop_kf(200,200);delay_ms(5);//位置环kf
	//set_torque_smooth(100,100);delay_ms(5);//设置转矩平滑系数
	//set_hub_curloop_kp(1200,1200);delay_ms(5);//电流环kp
	//set_hub_curloop_ki(300,300);delay_ms(5);//电流环ki
	
	set_hub_Acctime(30,30);delay_ms(5);       //加速时间
	set_hub_Dectime(30,30);delay_ms(5);       //减速时间
	set_hub_StopDectime(100,100);delay_ms(5); //急停时间
	
	#if USE_CAN1
	CAN1_Send_Num(0x601,SaveEEPROM);//将上述设置的参数保存至EEPROM
	delay_ms(15);
	#endif
	
	#if USE_CAN2
	CAN2_Send_Num(0x601,SaveEEPROM);//将上述设置的参数保存至EEPROM
	delay_ms(15);
	#endif
	
	//解轴
	set_IO_mode(1);
}
