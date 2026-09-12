#ifndef __SYSTEM_H
#define __SYSTEM_H



//系统功能选配
#define USE_DWT_CORE 1   //是否使用Cortex-M内核计时器，经实验，如果板子使用GD32则不可以开启，会出现不稳定
#define USE_CAN2 1        //是否使用CAN1与驱动器通信
#define OLED_DEBUG_MODE 1 //是否开启DEBUG调试模式，开启后可便捷查看机器人各个传感器数据
#define USE_US_Avoid 1    //是否启用底盘避障功能，开启后，非 ros/串口/CAN 控制下可开启/关闭底盘避障
#define USE_IWDG 0        //是否使用看门狗对程序进行监视
#define USE_RGB_lights 1  //是否使用RGB灯带

// Refer to all header files you need
//引用所有需要用到的头文件
#include "FreeRTOSConfig.h"
//FreeRTOS相关头文件 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//外设的相关头文件
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "can2.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"
#include "I2C.h"
#include "ICM20948.h"
#include "Ultrasonic.h"
#include "read_distance.h"
#include "Public_StdTypes.h"
#include "Public_StdMacros.h"
#include "DriverInit.h"
#include "DriverInit_485.h"
#include "MPU6050.h"
#include "AutoRecharge.h"
#include "dwt.h"
#include "iwdg.h"
#include "bsp_can.h"

//机器人允许的最低电压
#define MIN_VOL 20.0f

#define MOTOR_ENABLE  1
#define MOTOR_DISABLE 0

//机器人自检相关参数与自检操作
extern u32 Self_CheckingFlag;
enum 
{
	Drvie_overVOL				= (1 << 0),  //电机驱动过压
    Drvie_underVOL              = (1 << 1),  //电机驱动欠压
    L_Motor_overCUR             = (1 << 2),  //左电机过流
    L_Motor_overLoad            = (1 << 3),  //左电机过载
    L_Motor_CUR_ERROR           = (1 << 4),  //左电机电流异常
    L_Motor_Encoder_ERROR       = (1 << 5),  //左电机编码器数据异常
    L_Motor_SPEED_ERROR         = (1 << 6),  //左电机速度异常
    L_Motor_VOL_ERROR           = (1 << 7),  //左电机内部参考电压出错
	Drvie_EEPROM_ERROR          = (1 << 8),	 //驱动器内部EEPROM读写错误
    L_Motor_HAL_ERROR           = (1 << 9),  //左电机霍尔线未插

	R_Motor_overCUR             = (1 << 10),  //右电机过流
    R_Motor_overLoad            = (1 << 11),  //右电机过载
    R_Motor_CUR_ERROR           = (1 << 12),  //右电机电流异常
    R_Motor_Encoder_ERROR       = (1 << 13),  //右电机编码器数据异常
    R_Motor_SPEED_ERROR         = (1 << 14),  //右电机速度异常
    R_Motor_VOL_ERROR           = (1 << 15),  //右电机内部参考电压出错
    R_Motor_HAL_ERROR           = (1 << 16),  //右电机霍尔线未插
	
    Drvie_Timeout               = (1 << 17), //电机驱动器离线
    AutoRecharge_Timeout        = (1 << 18), //自动回充装备离线
    Lower_Power                 = (1 << 20), //电池电压不足
	Stop_Switch_DOWN            = (1 << 21), //急停开关被按下
	lost_left_redsignal         = (1 << 22), //未接收到充电桩左边的红外(39ms红外)
	lost_right_redsignal        = (1 << 23), //未接收到充电桩右边的红外(52ms红外)
	Drive2_ERROR                = (1 << 19), //驱动器2报错
	Drive1_ERROR                = (1 << 24), //驱动器1报错
	Drvie2_Timeout              = (1 << 25), //电机驱动器2离线
	
};
#define Clear_Error_Flag(mask)   (Self_CheckingFlag &= ~(mask))            //清除报错标志位
#define Set_SystemError_FLAG(mask)    (Self_CheckingFlag |= (mask))        //设置报错标志位
#define Get_Checking_FLAG(mask)       (Self_CheckingFlag & (mask))         //读取报错标志位
#define Get_DriveError_FLAG(checkdata,countdata)  (checkdata&(countdata))  //根据电机的输入读取对应的报错标志位


//机器人控制方式设置与读取
extern u8 Control_Mode;
enum
{
	_ROS_Control   =  (1<<0), //ROS控制
	_PS2_Control   =  (1<<1), //PS2控制(S系列小车不配备该功能，也不配备该硬件)
	_APP_Control   =  (1<<2), //APP控制
	_RC_Control    =  (1<<3), //航模遥控控制
	_CAN_Control   =  (1<<4), //CAN通信控制
	_USART_Control =  (1<<5), //串口控制
};
//设置机器人控制方式
#define Set_Control_Mode(mask)  (Control_Mode |= (mask), Control_Mode &= (mask)) 
//读取机器人控制方式
#define Get_Control_Mode(mask)  (Control_Mode & (mask))

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //编码器数值，读取电机实时速度
	float Encoder_Rpm;   //Encoder value, motor real-time speed, unit: rpm //编码器数值，电机实时速度，单位：rpm
	float Control_Rpm;   //Control the target speed of the brushless motor //控制无刷电机的目标转速
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //电机PWM数值，控制电机实时速度
	float Target;      //Control the target speed of the motor //电机目标速度值，控制电机目标速度
	float Velocity_KP; //Speed control PID parameters //速度控制PID参数
	float	Velocity_KI; //Speed control PID parameters //速度控制PID参数
}Motor_parameter;

//Smoothed the speed of the three axes
//平滑处理后的三轴速度
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;


//超声波距离
typedef struct  
{
	float A;
	float B;
	float C;
	float D;
	float E;
	float F;
	float G;
	float H;
}Ultrasonic;


/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** 外部变量定义，当其它c文件引用system.h时，也可以使用system.c定义的变量 ******/
extern u8 Flag_Stop, Last_Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Wheel_axlespacing; 
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;                                            										                 
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern int robot_mode_check_flag;

extern u8 CheckAutoRc,Last_CheckAutoRc,AutoRcCount;

extern Ultrasonic ultrasonic;
extern u8 LED_COLOR,Last_LED_COLOR,Color_state;
extern u8 motor_checkerror_flag,motor2_checkerror_flag;

extern u8 disable_robot_count;
extern float fre_show;

extern u8 HUB1_EnableState;
extern u8 HUB2_EnableState;

extern u16 HUB1_TEST_L ;
extern u16 HUB1_TEST_R ;
extern u16 HUB2_TEST_L ;
extern u16 HUB2_TEST_R ;
extern u32 can1_send_error;
extern u32 can2_send_error;

void systemInit(void);
float Check_Fre(void);

/***Macros define***/ /***宏定义***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//开机(1000/100hz=10)秒后才允许控制小车进行运动
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//机器人型号数量，决定Divisor_Mode的值，目前有6种小车类型
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***宏定义***/

//C library function related header file
//C库函数的相关头文件
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
