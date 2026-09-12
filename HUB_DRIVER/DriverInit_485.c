#include "DriverInit_485.h"

/*
文件说明：
本文件提供485接口控制轮毂电机，但实际未使用。
如有使用RS485接口控制轮毂电机的需求可参考此文件或调用此文件的API
*/


//rs485_modbus设置参数（仅可写单个寄存器以及读取多个寄存器）
// 驱动器地址  读写  字节数   寄存器起始地址   数据1  
uint8_t* Modbus_get_setparam (uint8_t hub_addr,uint8_t rw, uint8_t bytes,uint16_t addr,uint32_t writedata)
{
	static uint8_t user_needdata[8];
	u16 crc_ref;
	user_needdata[0]=hub_addr;
	user_needdata[2]=(addr>>8)&0xff;//取出寄存器起始地址高8位;
	user_needdata[3]=addr&0xff;     //取出寄存器起始地址低8位;
	
	if(rw==0x01)// r=1,w=0,写数据
	{
		if (bytes<=2){
			user_needdata[1]=0x06;//写单个寄存器
			user_needdata[4]=(writedata>>8)&0xff;//取出数据高8位;
			user_needdata[5]=writedata&0xff;     //取出数据低8位;
		} 
		else if (bytes>2)
		{ 
			user_needdata[2]=0x10;
			return user_needdata;//写多个寄存器(数据帧不定长不好整体定义，未实现)
		}
	}
	else if(rw==0x10)// r=1,w=0,读数据
	{
		user_needdata[1]=0x03;//读多个寄存器
		if (bytes<=2){
			user_needdata[4]=0x00;//寄存器个数高8位;
			user_needdata[5]=0x01;//寄存器个数低8位;
		}else if (bytes>2&&bytes<=4){ 
			user_needdata[4]=0x00;//寄存器个数高8位;
			user_needdata[5]=0x02;//寄存器个数低8位;
		}else if (bytes>4&&bytes<=6){
			user_needdata[4]=0x00;//寄存器个数高8位;
			user_needdata[5]=0x03;//寄存器个数低8位;
		}else if (bytes>6&&bytes<=8){
			user_needdata[4]=0x00;//寄存器个数高8位;
			user_needdata[5]=0x04;//寄存器个数低8位;
		}
	}
	crc_ref = Modbus_crc(user_needdata,6); //crc差错校验
	user_needdata[6]=crc_ref>>8;	//crc校验数据高8位
	user_needdata[7]=crc_ref;		//crc校验数据低8位
	return user_needdata;
}

//设置编码器线数
void RS485_set_encoder_accuracy(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_ENCODER_ACCURACY_ADDR,L_data);//左电机编码器线数
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_ENCODER_ACCURACY_ADDR,R_data);//右电机编码器线数
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机HALL偏移角度
void RS485_set_HallOffset(short L_data,short R_data){
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_HALL_OFFSET_ADDR,L_data);//左电机HALL偏移角度
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_HALL_OFFSET_ADDR,R_data);//右电机HALL偏移角度
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机极对数
void RS485_set_poles(u16 L_data,u16 R_data){
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_POLES_ADDR,L_data);//左电机极对数
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_POLES_ADDR,R_data);//右电机极对数
	RS485_Send(p,8);
	delay_ms(5);

}

//设置电机额定电流
void RS485_set_rated_current(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_RATEDCUR_ADDR,L_data);//左电机额定电流
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_RATEDCUR_ADDR,R_data);//右电机额定电流
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机最大电流
void RS485_set_max_current(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_MAXCUR_ADDR,L_data);//左电机最大电流
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_MAXCUR_ADDR,R_data);//右电机最大电流
	RS485_Send(p,8);
	delay_ms(5);
}
//设置电机最大转速
void RS485_set_max_rpm(u16 data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_MOTOR_MAX_RPM,data);//电机最大转速
	RS485_Send(p,8);
	delay_ms(5);
}
//设置过载保护系数
void RS485_set_overload_factor(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_OVERLOAD_FACTOR_ADDR,L_data);//左电机过载保护时间
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_OVERLOAD_FACTOR_ADDR,R_data);//右电机过载保护时间
	RS485_Send(p,8);
	delay_ms(5);
}

//设置过载保护时间
void RS485_set_overload_time(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_OVERLOAD_TIME_ADDR,L_data);//左电机过载保护时间
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_OVERLOAD_TIME_ADDR,R_data);//右电机过载保护时间
	RS485_Send(p,8);
	delay_ms(5);
}
//设置编码器超差报警阈值
void RS485_set_encoder_ErrorAlarm(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_ENCODER_ERRORALARM_ADDR,L_data);//左电机超差报警阈值
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_ENCODER_ERRORALARM_ADDR,R_data);//右电机超差报警阈值
	RS485_Send(p,8);
	delay_ms(5);
}

//设置温度保护阈值
void RS485_set_maxTemp(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MAX_TEMPERATURE_ADDR,L_data);//左电机温度保护阈值
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MAX_TEMPERATURE_ADDR,R_data);//右电机温度保护阈值
	RS485_Send(p,8);
	delay_ms(5);
}

//速度环kp
void RS485_set_speed_kp(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_SPEED_KP_ADDR,L_data);//左电机速度环kp
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_SPEED_KP_ADDR,R_data);//右电机速度环kp
	RS485_Send(p,8);
	delay_ms(5);
}

//速度环ki
void RS485_set_speed_ki(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_SPEED_KI_ADDR,L_data);//左电机速度环ki
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_SPEED_KI_ADDR,R_data);//右电机速度环ki
	RS485_Send(p,8);
	delay_ms(5);
}

//速度前馈kf
void RS485_set_speed_kf(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_SPEED_KF_ADDR,L_data);//左电机速度前馈kf
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_SPEED_KF_ADDR,R_data);//右电机速度前馈kf
	RS485_Send(p,8);
	delay_ms(5);
}
//速度平滑系数
void RS485_set_speed_smooth(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_SPEED_SMOOTH_ADDR,L_data);//左电机速度平滑系数
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_SPEED_SMOOTH_ADDR,R_data);//右电机速度平滑系数
	RS485_Send(p,8);
	delay_ms(5);
}

//前馈平滑系数
void RS485_set_smooth_kf(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_SMOOTH_KF_ADDR,L_data);//左电机速度前馈平滑系数
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_SMOOTH_KF_ADDR,R_data);//右电机速度前馈平滑系数
	RS485_Send(p,8);
	delay_ms(5);
}
//设置电机加速时间
void RS485_set_acctime(u16 L_data,u16 R_data)
{
	uint8_t *p;
	if(L_data>32767) L_data=32767;
	if(R_data>32767) R_data=32767;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_ACCTIME_ADDR,L_data);//左电机加速时间
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_ACCTIME_ADDR,R_data);//右电机加速时间
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机减速时间
void RS485_set_dectime(u16 L_data,u16 R_data)
{
	uint8_t *p;
	if(L_data>32767) L_data=32767;
	if(R_data>32767) R_data=32767;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_DECTIME_ADDR,L_data);//左电机减速时间
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_DECTIME_ADDR,R_data);//右电机减速时间
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机急停时间
void RS485_set_stoptime(u16 L_data,u16 R_data)
{
	uint8_t *p;
	if(L_data>32767) L_data=32767;
	if(R_data>32767) R_data=32767;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_MOTOR_STOPTIME_ADDR,L_data);//左电机急停时间
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_MOTOR_STOPTIME_ADDR,R_data);//右电机急停时间
	RS485_Send(p,8);
	delay_ms(5);
}

//位置控制模式下最大速度设置
void RS485_set_PosControl_MaxSpeed(u16 L_data,u16 R_data)
{
	uint8_t *p;
	if(L_data>1000) L_data=1000;
	if(R_data>1000) R_data=1000;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_POSCon_MAX_SPEED_ADDR,L_data);//左电机位置控制模式下最大速度
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_POSCon_MAX_SPEED_ADDR,R_data);//右电机位置控制模式下最大速度
	RS485_Send(p,8);
	delay_ms(5);
}

//转矩斜率设置
void RS485_set_Tor_Slope(u16 L_data,u16 R_data)
{
	uint8_t *p;
	if(L_data>32767) L_data=32767;
	if(R_data>32767) R_data=32767;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_TOR_SLOPE_ADDR,L_data);//左电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_TOR_SLOPE_ADDR,R_data);//右电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
}

//设置电机外部急停的方式(0:急停后锁住轮子 1:急停后释放轮子)
void RS485_set_IO_StopMode(u16 data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_MOTOR_IO_STOP_ADDR,data);//电机外部急停的方式
	RS485_Send(p,8);
	delay_ms(5);
}

//将参数保存到EEPROM
void RS485_EEPROM_Save(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_EEPROM_SAVE_ADDR,RS485_EEPROM_SAVE);//将参数保存到EEPROM
	RS485_Send(p,8);
	delay_ms(5);
}


//设置控制模式
//RS485_POS_REL_CONTROL 相对位置控制
//RS485_POS_ABS_CONTROL 绝对位置控制
//RS485_VEL_CONTROL     速度控制
//RS485_TOR_CONTROL     力矩控制
void RS485_set_ControlMode(u8 data)
{
	uint8_t *p;
	if(data>4) data=0;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_OPERATION_MODE,data);
	RS485_Send(p,8);
	delay_ms(5);
}

//电机失能
void RS485_Stop(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Stop);//写控制寄存器，失能电机
	RS485_Send(p,8);
	delay_ms(5);
}

//电机使能
void RS485_Enable(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Enable);//写控制寄存器，使能电机
	RS485_Send(p,8);
	delay_ms(5);
}
//位置模式同步控制下启动电机
void POS_Syn_START_MOTOR(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Start);//位置模式同步控制下，使能电机
	RS485_Send(p,8);
	delay_ms(5);
}
//位置模式异步控制下启动左电机
void POS_Asy_START_MOTOR_L(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Start_L);//位置模式下，使能左电机
	RS485_Send(p,8);
	delay_ms(5);
}
//位置模式异步控制下启动右电机
void POS_Asy_START_MOTOR_R(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Start_R);//位置模式下，使能右电机
	RS485_Send(p,8);
	delay_ms(5);
}
//电机清除报错
void RS485_ClearError(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_WORD,HUB_Alarm_Clean);//写控制寄存器，清除报错
	RS485_Send(p,8);
	delay_ms(5);
}
//设置电机转速(同时写多个寄存器)（2个）
void RS485_set_MotorRPM(short L_data,short R_data)
{
	u16 crc_ref;
	//转速限幅
	if(L_data>3000) L_data=3000;if(L_data<-3000) L_data=-3000;
	if(R_data>3000) R_data=3000;if(R_data<-3000) R_data=-3000;
	//数据帧赋值
	uint8_t p[13]={0x01,0x10,0x20,0x88,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
	p[7]=(L_data>>8)&0xff;//取出数据高8位;
	p[8]=L_data&0xff;     //取出数据低8位;
	p[9]=(R_data>>8)&0xff;//取出数据高8位;
	p[10]=R_data&0xff;     //取出数据低8位;
	crc_ref = Modbus_crc(p,11); //crc差错校验
	p[11]=crc_ref>>8;	//crc校验数据高8位
	p[12]=crc_ref;		//crc校验数据低8位
	RS485_Send(p,13);
	delay_ms(5);
}
//设置电机目标位置(同时写多个寄存器)（4个）
void RS485_set_MotorPOS(int32_t  L_data,int32_t  R_data)
{
	u16 crc_ref;
	//数据帧赋值
	uint8_t p[17]={0x01,0x10,0x20,0x8A,0x00,0x04,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	//32位数据1
	p[7]=(L_data>>24)&0xff;
	p[8]=(L_data>>16)&0xff;     
	p[9]=(L_data>>8)&0xff;
	p[10]=L_data&0xff;  
	//32位数据2
	p[11]=(R_data>>24)&0xff;
	p[12]=(R_data>>16)&0xff;
	p[13]=(R_data>>8)&0xff;
	p[14]=R_data&0xff;  
	crc_ref = Modbus_crc(p,15); //crc差错校验
	p[15]=crc_ref>>8;	//crc校验数据高8位
	p[16]=crc_ref;		//crc校验数据低8位
	RS485_Send(p,17);
	delay_ms(5);
}

//设置电机目标转矩(同时写多个寄存器)（2个）
void RS485_set_MotorTOR(short L_data,short R_data)
{
	u16 crc_ref;
	//转速限幅
	if(L_data>30000) L_data=30000;if(L_data<-30000) L_data=-30000;
	if(R_data>30000) R_data=30000;if(R_data<-30000) R_data=-30000;
	//数据帧赋值
	uint8_t p[13]={0x01,0x10,0x20,0x90,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
	p[7]=(L_data>>8)&0xff;//取出数据高8位;
	p[8]=L_data&0xff;     //取出数据低8位;
	p[9]=(R_data>>8)&0xff;//取出数据高8位;
	p[10]=R_data&0xff;     //取出数据低8位;
	crc_ref = Modbus_crc(p,11); //crc差错校验
	p[11]=crc_ref>>8;	//crc校验数据高8位
	p[12]=crc_ref;		//crc校验数据低8位
	RS485_Send(p,13);
	delay_ms(5);
}

//设置电机控制同步/异步控制(0:异步控制 1:同步控制)
void RS485_set_SynCon_state(u8 data)
{
	uint8_t *p;
	if(data!=0) data=1;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_CONTROL_MODE,data);
	RS485_Send(p,8);
	delay_ms(5);
}
//访问状态字
void RS485_Read_StatusWord(void){
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,2,RS485_STATUS_WORD,0);
	RS485_Send(p,8);
	delay_ms(5);
}
//检查电机报错情况
void RS485_Check_MotorError(void){
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,2,RS485_L_DRIVE_STATUS,0);//左电机故障查询
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,2,RS485_R_DRIVE_STATUS,0);//右电机故障查询
	RS485_Send(p,8);
	delay_ms(5);
}

//查询驱动器温度
void RS485_Read_Hub_Temperature(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,2,RS485_HUB_TEMPERATURE,0);
	RS485_Send(p,8);
	delay_ms(5);
}
//查询电机温度
void RS485_Read_Motor_Temperature(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,2,RS485_MOTOR_TEMPERATURE,0);
	RS485_Send(p,8);//返回的数据，高八位为左电机，低八位为右电机
	delay_ms(5);
}
//查询电机实际转速
void RS485_Read_Motor_RPM(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,4,RS485_L_ACTUAL_RPM_FEEDBACK,0);
	RS485_Send(p,8);
	delay_ms(5);
}

//查询电机实际转矩(电流大小)
void RS485_Read_Motor_TOR(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,4,RS485_L_ACTUAL_TOR_FEEDBACK,0);
	RS485_Send(p,8);
	delay_ms(5);
}

//查询电机实际位置
void RS485_Read_Motor_POS(void)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_read_hub,8,RS485_L_ACTUAL_POS_FEEDBACK_H16,0);//返回两个32位的电机位置数据
	RS485_Send(p,8);
	delay_ms(5);
}

//位置模式同步控制Demo
void RS485_PosSynControl_Demo(void)
{
	//初始化:
	RS485_set_SynCon_state(1);//设置电机控制方式为同步控制
	RS485_set_ControlMode(RS485_POS_REL_CONTROL);//设置电机工作在相对位置模式
	RS485_set_acctime(100,100);//设置电机S型加速时间为100ms
	RS485_set_dectime(100,100);//设置电机S型减速时间为100ms
	RS485_set_PosControl_MaxSpeed(60,60);//设置电机最大速度为60rpm
	RS485_Enable(); //使能电机运动
	//相对位置同步控制demo
	RS485_set_MotorPOS(32000,-32000);
	POS_Syn_START_MOTOR();
	RS485_set_MotorPOS(-32000,32000);
	POS_Syn_START_MOTOR();
	RS485_Stop();
	//绝对位置同步控制demo：将上方demo中工作模式设置为RS485_POS_ABS_CONTROL即可
}
//位置模式异步控制Demo
void RS485_PosAsyControl_Demo(void)
{
	//初始化:
	RS485_set_SynCon_state(0);//设置电机控制方式为异步控制
	RS485_set_ControlMode(RS485_POS_REL_CONTROL);//设置电机工作在相对位置模式
	RS485_set_acctime(100,100);//设置电机S型加速时间为100ms
	RS485_set_dectime(100,100);//设置电机S型减速时间为100ms
	RS485_set_PosControl_MaxSpeed(60,60);//设置电机最大速度为60rpm
	RS485_Enable(); //使能电机运动
	//相对位置异步控制demo
	RS485_set_MotorPOS(32000,-32000);
	POS_Asy_START_MOTOR_L();//左电机启动
	POS_Asy_START_MOTOR_R();//右电机启动
	RS485_set_MotorPOS(-32000,32000);
	POS_Asy_START_MOTOR_L();//左电机启动
	POS_Asy_START_MOTOR_R();//右电机启动
	RS485_Stop();
	//绝对位置同步控制demo：将上方demo中工作模式设置为RS485_POS_ABS_CONTROL即可
}

//力矩控制Demo
void RS485_TorControl_Demo(void)
{
	//初始化:
	RS485_set_SynCon_state(1);//设置电机控制方式为同步控制
	RS485_set_ControlMode(RS485_TOR_CONTROL);//设置电机工作在转矩模式
	RS485_set_Tor_Slope(500,500);//设置电机转矩斜率为500mA/s
	RS485_Enable(); //使能电机运动
	//力矩控制demo:
	RS485_set_MotorTOR(2000,2000);
	RS485_set_MotorTOR(-2000,-2000);
	RS485_Stop();
}
//转矩平滑系数设置
void RS485_set_tor_smooth(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_TOR_SMOOTH_ADDR,L_data);//左电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_TOR_SMOOTH_ADDR,R_data);//右电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
}
//电流环Kp设置
void RS485_set_tor_kp(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_TOR_KP_ADDR,L_data);//左电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_TOR_KP_ADDR,R_data);//右电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
}
//电流环ki设置
void RS485_set_tor_ki(u16 L_data,u16 R_data)
{
	uint8_t *p;
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_L_TOR_KI_ADDR,L_data);//左电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
	p=Modbus_get_setparam(RS485_addr,RS485_write_hub,2,RS485_R_TOR_KI_ADDR,R_data);//右电机转矩斜率
	RS485_Send(p,8);
	delay_ms(5);
}
void RS485_Hub_Init(void){
	/* 驱动器基本参数配置 */
	if(Car_Mode==S300) //S300 8寸轮毂电机
	{
		RS485_set_encoder_accuracy(1024,1024);		//编码器线数
		RS485_set_HallOffset(0,0);					//HALL偏移角度
		RS485_set_poles(15,15);						//电机极对数
		RS485_set_rated_current(150,150);			//额定电流
		RS485_set_max_current(300,300); 			//最大电流
		RS485_set_max_rpm(1000); 					//最大转速
		RS485_set_overload_factor(200,200);			//过载系数
		RS485_set_overload_time(800,800);			//过载时间
		RS485_set_encoder_ErrorAlarm(409,409); 		//编码器超差阈值
		RS485_set_maxTemp(800,800);					//温度保护阈值
		RS485_set_speed_kp(500,500);				//速度环kp
		RS485_set_speed_ki(333,333);				//速度环ki
		RS485_set_speed_kf(1000,1000);				//速度环kf
		RS485_set_speed_smooth(50,50);				//速度平滑系数
		RS485_set_smooth_kf(100,100);				//前馈平滑系数
		RS485_set_acctime(50,50);					//加速时间
		RS485_set_dectime(50,50);					//减速时间
		RS485_set_stoptime(100,100);				//急停时间
		RS485_set_IO_StopMode(RS485_IO_UNLOCK);		//外部IO急停时为解轴模式
		RS485_EEPROM_Save();						//保存参数到EEPROM
	}
	else if(Car_Mode==SX03) //无刷四驱车
	{
		RS485_set_encoder_accuracy(4096,4096);  //编码器线数
		RS485_set_HallOffset(0,0);           	//HALL偏移角度
		RS485_set_poles(15,15);               //电机极对数
		RS485_set_rated_current(150,150);      //额定电流
		RS485_set_max_current(300,300);        //最大电流
		RS485_set_max_rpm(1000);           		//最大转速
		RS485_set_overload_factor(200,200);    //过载系数
		RS485_set_overload_time(800,800);      //过载时间
		RS485_set_encoder_ErrorAlarm(409,409); //编码器超差阈值
		RS485_set_maxTemp(800,800);            //温度保护阈值
		RS485_set_speed_kp(450,450);           //速度环kp
		RS485_set_speed_ki(250,250);           //速度环ki
		RS485_set_speed_kf(1000,1000);          //速度环kf
		RS485_set_speed_smooth(50,50);        //速度平滑系数
		RS485_set_smooth_kf(100,100);          //前馈平滑系数
		RS485_set_tor_smooth(100,100);         //转矩平滑系数
		RS485_set_tor_kp(700,700);             //电流环Kp
		RS485_set_tor_ki(250,250);              //电流环ki
		RS485_set_acctime(30,30);             //加速时间
		RS485_set_dectime(30,30);             //减速时间
		RS485_set_stoptime(100,100);           //急停时间
		RS485_set_IO_StopMode(RS485_IO_UNLOCK);  //外部IO急停时为解轴模式
		RS485_EEPROM_Save();                //保存参数到EEPROM
	}
	/* 电机控制模式配置 */
	RS485_set_SynCon_state(1);      //设置电机控制方式为同步控制
	RS485_set_ControlMode(RS485_VEL_CONTROL);//设置电机在速度模式下运动
	RS485_EEPROM_Save();            //保存参数到EEPROM
	/* 电机使能控制 */
	RS485_set_MotorRPM(0,0);        //设置电机转速为0
	RS485_Enable();                 //使能电机
}




































