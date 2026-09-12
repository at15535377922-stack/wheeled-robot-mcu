#ifndef __CAN2_H
#define __CAN2_H	 
#include "sys.h"	    
#include "system.h"
 
//CAN2 receives RX0 interrupt enablement
//CAN2接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1	//0, not enabling;1, can make //0,不使能; 1,使能										    		

u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 CAN2_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);	
u8 CAN2_Msg_Pend(u8 fifox);								
void CAN2_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);
u8 CAN2_Tx_Staus(u8 mbox);  							
u8 CAN2_Send_Msg(u8* msg,u8 len);				
u8 CAN2_Receive_Msg(u8 *buf);			
void CAN2_RX0_IRQHandler(void);
u8 CAN2_Send_MsgTEST(u8* msg,u8 len);
u8 CAN2_Send_Num(u32 id,u8* msg);



#endif

















