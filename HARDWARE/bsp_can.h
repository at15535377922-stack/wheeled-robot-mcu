#ifndef __BSP_CAN_H
#define __BSP_CAN_H
#include "system.h"

void CAN_1_2_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN1_Send_Num(u32 id,u8 *data);
u8 CAN2_Send_Num(u32 id,u8 *data);

void set_hub_errorstate(u16 left_motor_state,u16 right_motor_state);

#endif
