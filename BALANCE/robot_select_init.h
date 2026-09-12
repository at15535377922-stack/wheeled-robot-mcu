#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //轮距 麦轮车为半轮距
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //轴距 麦轮车为半轴距	
  int   GearRatio;         //Motor_gear_ratio //电机减速比
  int   EncoderAccuracy;   //Number_of_encoder_lines //编码器精度(编码器线数)
  float WheelDiameter;     //Diameter of driving wheel //主动轮直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向轮小车旋转半径
}Robot_Parament_InitTypeDef;

//S300轮距、轮径
#define S300_Wheelspacing 0.398
#define S300_Diameter 0.20

//S150轮距、轮径
#define S150_Wheelspacing 0.26
#define S150_Diameter 0.13

//S100轮距、轮距
#define S100_Wheelspacing 0.2272f
#define S100_Diameter 0.13

//S200轮距、轮径
//#define S200_Wheelspacing 0.29 //半轮距
//#define S200_axlespacing  0.175//半轴距
#define S200_Wheelspacing 0.23 //半轮距
#define S200_axlespacing  0.15//半轴距
#define S200_Diameter 0.205

//The encoder octave depends on the encoder initialization Settings
//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples 4
//Encoder data reading frequency
//编码器数据读取频率
#define CONTROL_FREQUENCY 100

//#define PI 3.1415f  //PI //圆周率

//车型枚举
enum CAR_MODE{
	S300,
	S200,
	S150,
	S100,
	SX03,
	SX04,
	Number_of_CAR
};

void Robot_Select(void);
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) ;
extern u8 data_2_ulr[8];
#endif
