#ifndef __HUBDRIVERCAN_H
#define __HUBDRIVERCAN_H	 

#include "sys.h"
#include "system.h"

//轮毂电机驱动器结构体
typedef struct  
{
	float L_motorTemperature; //原始数据：单位0.1度，有符号16位
	float R_motorTemperature;
	float Drive_Temperature;
	
	float L_motorCurrent; //原始数据：单位0.1A，有符号16位
	float R_motorCurrent;
	
	float Voltage; //原始数据：单位0.01V，无符号16位

}HubDrive_parameter;

extern HubDrive_parameter HUB_Drive;

uint8_t* get_setparam(uint8_t rw, uint8_t bytes,uint32_t addr,uint32_t writedata);

extern int acc_time;

void hub_CAN_velocity_init(void);
void hub_CAN_position_init(void);
void hub_CAN_torque_init(void);

void CAN_setBaudrate(int Baudrate);
void CAN_Write_EEPROM(void);

void CAN_Stop(void);
void CAN_Enable(void);
void CAN_ClearError(void);

void CAN_Asyn_ctrl_set(int asyn);

void CAN_V_mode_set(void);
void CAN_P_mode_set(void);
void CAN_T_mode_set(void);

void CAN_acc_time(int time);
void CAN_acc_time_left(int time);
void CAN_dec_time_left(int time);
void CAN_acc_time_right(int time);
void CAN_dec_time_right(int time);
	
void CAN_SetMaxRpm_left_Right(int Left, int Right);

void CAN_Left_Torque_mAs(int mAs);
void CAN_Right_Torque_mAs(int mAs);

void hub_CAN_Rpm(int R_L, int rpm);
void hub_CAN_Syn_Rpm(int left_rpm, int right_rpm);
void hub_CAN_Rel_Position(int R_L, long int position);
void hub_CAN_Abs_Position(int R_L, long int position);
void hub_CAN_Torque(int R_L, int torque);
void hub_CAN_Syn_Torque(int torque);

void hub_CAN_Encoder_init(void);
void CAN_CheckError(void);

void hub_speedloop_kp(u16 L_kp,u16 R_kp);
void hub_speedloop_ki(u16 L_ki,u16 R_ki);
void hub_set_overload_time(u16 L_time,u16 R_time);
void hub_set_overload_factor(u16 L_factor,u16 R_factor);
void hub_set_encoder_accuracy(u16 L_encoder,u16 R_encoder);
void hub_set_motor_poles(u16 L_pole,u16 R_pole);
void hub_set_motor_OffsetAngle(int L_angle,int R_angle);
void set_max_motor_speed(u16 speed);
void set_motor_rated_current(u16 L_cur,u16 R_cur);
void set_motor_max_current(u16 L_cur,u16 R_cur);

void hub_MotorERROR_Init(void);


void set_tolerance_alarm_threshold(u16 L_Data,u16 R_Data);
void set_Temp_PH(u16 L_Data,u16 R_Data);
void set_hub_posloop_kp(u16 L_Data,u16 R_Data);
void set_hub_posloop_kf(u16 L_Data,u16 R_Data);
void set_hub_speedloop_kf(u16 L_Data,u16 R_Data);
void set_speed_smooth(u16 L_Data,u16 R_Data);
void set_smooth_kf(u16 L_Data,u16 R_Data);
void set_torque_smooth(u16 L_Data,u16 R_Data);
void set_hub_curloop_kp(u16 L_Data,u16 R_Data);
void set_hub_curloop_ki(u16 L_Data,u16 R_Data);
void set_hub_Acctime(u32 L_Data,u32 R_Data);
void set_hub_Dectime(u32 L_Data,u32 R_Data);
void set_hub_StopDectime(u32 L_Data,u32 R_Data);
void hub_set_SynchronousControl_Enable(void);
void hub_set_SynchronousControl_Disable(void);
void hub_MotorTemperature_Init(void);
void hub_MotorCurrent_Init(void);
void hub_mainParam_set(void);
void set_IO_mode(u8 mode);

#endif

