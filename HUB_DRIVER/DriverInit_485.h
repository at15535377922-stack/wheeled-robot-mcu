#ifndef __DRIVERINIT_485_H
#define __DRIVERINIT_485_H
#include "system.h"

//驱动器地址
#define RS485_addr 0x01

//驱动器的读写属性
#define RS485_write_hub 0x01 //写数据 rw=0x01
#define RS485_read_hub  0x10 //读数据 rw=0x10
//外部急停解轴、锁轴设置
#define RS485_IO_LOCK   0 //锁轴
#define RS485_IO_UNLOCK 1 //解轴
#define RS485_EEPROM_SAVE 1	
//控制模式
#define RS485_POS_REL_CONTROL 1 //相对位置控制
#define RS485_POS_ABS_CONTROL 2 //绝对位置控制
#define RS485_VEL_CONTROL 3 //速度控制
#define RS485_TOR_CONTROL 4 //转矩控制
//控制字
#define HUB_Emergency_Stop  0x05	//急停
#define HUB_Alarm_Clean		0x06    //报警清除
#define HUB_Stop			0x07    //停机
#define HUB_Enable          0x08    //使能
#define HUB_Start           0x10    //启动(同步)(位置模式下需要)
#define HUB_Start_L			0x11	//左电机启动(异步)(位置模式下需要)
#define HUB_Start_R			0x12	//右电机启动(异步)(位置模式下需要)

//左右电机公用参数
#define RS485_FEEDBACK_POS_CLEAN_ADDR			0x2005  //清除反馈位置
#define RS485_CURRENT_POS_CLEAN_ADDR			0x2006	//绝对位置模式时用于清除当前位置
#define RS485_LOCK_METHOD_ADRR					0x2007	//上电锁轴方式
#define RS485_MOTOR_MAX_RPM						0x2008	//电机最大运行速度
#define RS485_OPERATION_MODE					0x200D	//运行模式
#define RS485_CONTROL_WORD						0x200E	//控制寄存器--启停/使能/停机等
#define RS485_CONTROL_MODE						0x200F	//同步/异步控制模式
#define RS485_EEPROM_SAVE_ADDR					0x2010	//是否保存参数于EEPROM
#define RS485_HUB_MAX_TEMPERATURE_ADDR			0x201E	//驱动器温度保护阈值
#define RS485_MOTOR_IO_STOP_ADDR         		0x2021 //外部IO急停模式设置

//左电机独立参数
#define RS485_L_ENCODER_ACCURACY_ADDR      		0X2030 	//编码器线数设置
#define RS485_L_HALL_OFFSET_ADDR           		0x2031 	//电机与HALL偏移角度
#define RS485_L_MOTOR_OVERLOAD_FACTOR_ADDR 		0X2032 	//过载系数设置
#define RS485_L_MOTOR_RATEDCUR_ADDR		   		0X2033 	//额定电流设置
#define RS485_L_MOTOR_MAXCUR_ADDR				0x2034	//最大电流设置
#define RS485_L_MOTOR_OVERLOAD_TIME_ADDR		0x2035	//过载保护时间设置
#define RS485_L_ENCODER_ERRORALARM_ADDR			0x2036	//编码器超差阈值设置
#define RS485_L_SPEED_SMOOTH_ADDR				0x2037	//速度平滑系数设置
#define RS485_L_TOR_KP_ADDR						0x2038	//电流环比例系数设置
#define RS485_L_TOR_KI_ADDR						0x2039  //电流环积分系数设置
#define RS485_L_SMOOTH_KF_ADDR					0x203A  //前馈输出平滑系数设置
#define RS485_L_TOR_SMOOTH_ADDR					0x203B  //转矩输出平滑系数设置
#define RS485_L_SPEED_KP_ADDR					0x203C  //速度环KP设置
#define RS485_L_SPEED_KI_ADDR					0x203D  //速度环Ki设置
#define RS485_L_SPEED_KF_ADDR					0x203E  //速度前馈kf设置
#define RS485_L_POS_KP_ADDR						0x203F  //位置比例增益kp设置
#define RS485_L_POS_KF_ADDR                     0x2040	//位置前馈增益kf设置
#define RS485_L_START_SPEED						0x2043  //起始速度设置
#define RS485_L_POSCon_Start_stop_SPEED_ADDR	0x2044  //位置模式启停速度设置
#define RS485_L_MOTOR_POLES_ADDR				0x2045  //电机极对数
#define RS485_L_MAX_TEMPERATURE_ADDR			0x2046  //电机温度保护阈值
#define RS485_L_VELOCITY_OBSERVER1				0x2047  //速度观测器系数1
#define RS485_L_VELOCITY_OBSERVER2				0x2048  //速度观测器系数2
#define RS485_L_VELOCITY_OBSERVER3				0x2049  //速度观测器系数3
#define RS485_L_VELOCITY_OBSERVER4				0x204A	//速度观测器系数4
#define RS485_L_MOTOR_ACCTIME_ADDR				0x2080	//电机加速时间设置
#define RS485_L_MOTOR_DECTIME_ADDR				0x2082  //电机减速时间设置
#define RS485_L_MOTOR_STOPTIME_ADDR				0x2084  //电机急停时间设置
#define RS485_L_TOR_SLOPE_ADDR					0x2086  //转矩斜率设置
#define RS485_L_TARGET_RPM_ADDR					0x2088  //电机目标转速设置地址
#define RS485_L_TARGET_POS_H16_ADDR				0X208A  //电机目标位置高16位设置地址
#define RS485_L_TARGET_POS_L16_ADDR				0X208B  //电机目标位置低16位设置地址
#define RS485_L_POSCon_MAX_SPEED_ADDR			0X208E  //位置模式下最大速度设置
#define RS485_L_TARGET_TOR_ADDR					0X2090  //电机目标转矩设置地址

//右电机独立参数                                         
#define RS485_R_ENCODER_ACCURACY_ADDR      		0X2060 	//编码器线数设置
#define RS485_R_HALL_OFFSET_ADDR           		0x2061 	//电机与HALL偏移角度
#define RS485_R_MOTOR_OVERLOAD_FACTOR_ADDR 		0X2062 	//过载系数设置
#define RS485_R_MOTOR_RATEDCUR_ADDR		   		0X2063 	//额定电流设置
#define RS485_R_MOTOR_MAXCUR_ADDR				0x2064	//最大电流设置
#define RS485_R_MOTOR_OVERLOAD_TIME_ADDR		0x2065	//过载保护时间设置
#define RS485_R_ENCODER_ERRORALARM_ADDR			0x2066	//编码器超差阈值设置
#define RS485_R_SPEED_SMOOTH_ADDR				0x2067	//速度平滑系数设置
#define RS485_R_TOR_KP_ADDR						0x2068	//电流环比例系数设置
#define RS485_R_TOR_KI_ADDR						0x2069  //电流环积分系数设置
#define RS485_R_SMOOTH_KF_ADDR					0x206A  //前馈输出平滑系数设置
#define RS485_R_TOR_SMOOTH_ADDR					0x206B  //转矩输出平滑系数设置
#define RS485_R_SPEED_KP_ADDR					0x206C  //速度环KP设置
#define RS485_R_SPEED_KI_ADDR					0x206D  //速度环Ki设置
#define RS485_R_SPEED_KF_ADDR					0x206E  //速度前馈kf设置
#define RS485_R_POS_KP_ADDR						0x206F  //位置比例增益kp设置
#define RS485_R_POS_KF_ADDR                     0x2070	//位置前馈增益kf设置
#define RS485_R_START_SPEED						0x2073  //起始速度设置
#define RS485_R_POSCon_Start_stop_SPEED_ADDR	0x2074  //位置模式启停速度设置
#define RS485_R_MOTOR_POLES_ADDR				0x2075  //电机极对数
#define RS485_R_MAX_TEMPERATURE_ADDR			0x2076  //电机温度保护阈值
#define RS485_R_VELOCITY_OBSERVER1				0x2077  //速度观测器系数1
#define RS485_R_VELOCITY_OBSERVER2				0x2078  //速度观测器系数2
#define RS485_R_VELOCITY_OBSERVER3				0x2079  //速度观测器系数3
#define RS485_R_VELOCITY_OBSERVER4				0x207A	//速度观测器系数4                                                        
#define RS485_R_MOTOR_ACCTIME_ADDR				0x2081	//电机加速时间设置
#define RS485_R_MOTOR_DECTIME_ADDR				0x2083  //电机减速时间设置
#define RS485_R_MOTOR_STOPTIME_ADDR				0x2084  //电机急停时间设置
#define RS485_R_TOR_SLOPE_ADDR					0x2087  //转矩斜率设置
#define RS485_R_TARGET_RPM_ADDR					0x2089  //电机目标转速设置地址
#define RS485_R_TARGET_POS_H16_ADDR				0X208C  //电机目标位置高16位设置地址
#define RS485_R_TARGET_POS_L16_ADDR				0X208D  //电机目标位置低16位设置地址
#define RS485_R_POSCon_MAX_SPEED_ADDR			0X208F  //位置模式下最大速度设置
#define RS485_R_TARGET_TOR_ADDR					0X2091  //电机目标转矩设置地址

//只读参数
#define RS485_SOFTWARE_VERSION					0x20A0	//软件版本
#define RS485_BUS_VOLTAGE                       0x20A1  //母线电压
#define RS485_STATUS_WORD                       0x20A2  //电机状态字
#define RS485_HALL_INPUT_STATUS                 0x20A3  //霍尔输入状态
#define RS485_MOTOR_TEMPERATURE                 0x20A4  //电机温度
#define RS485_L_DRIVE_STATUS                    0x20A5  //驱动器最近一次故障码（左）
#define RS485_R_DRIVE_STATUS                    0x20A6  //驱动器最近一次故障码（右）
#define RS485_L_ACTUAL_POS_FEEDBACK_H16         0x20A7  //实际位置反馈位置高 16 位(左)
#define RS485_L_ACTUAL_POS_FEEDBACK_L16         0x20A8  //实际位置反馈位置低 16 位(左)
#define RS485_R_ACTUAL_POS_FEEDBACK_H16         0x20A9  //实际位置反馈位置高 16 位(右)
#define RS485_R_ACTUAL_POS_FEEDBACK_L16         0x20AA  //实际位置反馈位置低 16 位(右)
#define RS485_L_ACTUAL_RPM_FEEDBACK             0x20AB  //实际速度反馈(左)
#define RS485_R_ACTUAL_RPM_FEEDBACK             0x20AC  //实际速度反馈(右)
#define RS485_L_ACTUAL_TOR_FEEDBACK             0x20AD  //实际转矩反馈(左)
#define RS485_R_ACTUAL_TOR_FEEDBACK             0x20AE  //实际转矩反馈(右)
#define RS485_HUB_TEMPERATURE                   0x20B0  //驱动器温度

//RS485相关功能函数 
u16 Modbus_crc(u8* arry,u8 size);
void RS485_Send(u8* data,u8 len);
uint8_t* Modbus_get_setparam (uint8_t hub_addr,uint8_t rw, uint8_t bytes,uint16_t addr,uint32_t writedata);
void RS485_Hub_Init(void);
void RS485_set_encoder_accuracy(u16 L_data,u16 R_data);
void RS485_set_HallOffset(short L_data,short R_data);
void RS485_Stop(void);
//电机使能
void RS485_Enable(void);
//位置模式同步控制下启动电机
void POS_Syn_START_MOTOR(void);
//位置模式异步控制下启动左电机
void POS_Asy_START_MOTOR_L(void);
//位置模式异步控制下启动右电机
void POS_Asy_START_MOTOR_R(void);
//设置电机转速(同时写多个寄存器)（2个）
void RS485_set_MotorRPM(short L_data,short R_data);
//设置电机目标位置(同时写多个寄存器)（4个）
void RS485_set_MotorPOS(int32_t  L_data,int32_t  R_data);
//设置电机目标转矩(同时写多个寄存器)（2个）
void RS485_set_MotorTOR(short L_data,short R_data);
//访问状态字
void RS485_Read_StatusWord(void);
//检查电机报错情况
void RS485_Check_MotorError(void);
//查询驱动器温度
void RS485_Read_Hub_Temperature(void);
//查询电机温度
void RS485_Read_Motor_Temperature(void);
//查询电机实际转速
void RS485_Read_Motor_RPM(void);
//查询电机实际转矩(电流大小)
void RS485_Read_Motor_TOR(void);
//查询电机实际位置
void RS485_Read_Motor_POS(void);

#endif
























