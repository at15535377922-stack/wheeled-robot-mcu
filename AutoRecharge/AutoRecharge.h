#ifndef __AUTORECHARGE_H
#define __AUTORECHARGE_H
#include "sys.h"
#include "system.h"

extern u8 nav_walk, Allow_Recharge, Charging, RED_STATE;
extern float Recharge_VOL;
extern float Charging_Current;
extern float Recharge_Red_Move_X, Recharge_Red_Move_Y, Recharge_Red_Move_Z; 
extern float Recharge_UP_Move_X, Recharge_UP_Move_Y, Recharge_UP_Move_Z; 
extern float Red_Docker_X, Red_Docker_Y, Red_Docker_Z; 
extern u8 L_A,L_B,R_B,R_A;

void CAN_Send_AutoRecharge(void);

extern u8 red_ignore;

extern u8 which_mode;
#endif
