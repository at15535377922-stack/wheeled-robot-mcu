#include "robot_select_init.h"

u8 data_2_ulr[8];

//Initialize the robot parameter structure
//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC值分段变量，取决于小车型号数量
	Divisor_Mode=4096/(Number_of_CAR-1);
	
	//S系列暂定0~5 六种车型
	Car_Mode=(int) (Get_adc_Average(CAR_MODE_ADC,10)/Divisor_Mode); //Collect the pin information of potentiometer //采集电位器引脚信息		
	
	//将车型通过CAN总线发布
	#if USE_RGB_lights
	rgb_set[7] = Car_Mode;
	#else
	data_2_ulr[7] = Car_Mode;
	#endif
	
	if (Car_Mode==S300)  Robot_Init(S300_Wheelspacing,0,0,0,S300_Diameter);  //S300
	if (Car_Mode==S150)  Robot_Init(S150_Wheelspacing,0,0,0,S150_Diameter);  //S150
	if (Car_Mode==S100)  Robot_Init(S100_Wheelspacing,0,0,0,S100_Diameter);  //S100
	
	if (Car_Mode==S200)  Robot_Init(S200_Wheelspacing,S200_axlespacing,0,0,S200_Diameter); //无刷四驱
	
	if (Car_Mode==SX04)  Robot_Init(S300_Wheelspacing,0,0,0,S300_Diameter);  //暂定
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
	Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing 轮距  
	Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing 轴距
	Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio //电机减速比
	Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines //编码器精度(编码器线数)
	Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel //主动轮轮径

	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing 轮距 
	Wheel_spacing=Robot_Parament.WheelSpacing;  
	//Wheel_axlespacing (Wheel_axlespacing is not required for motion analysis of differential trolleys) //轴距(差速小车的运动分析不需要轴距)	
	Wheel_axlespacing=Robot_Parament.AxleSpacing;    
}


