#ifndef __ultrasonic_H
#define __ultrasonic_H
#include "sys.h"
#include "system.h"


//超声波A -> TIM3_CH4 -> PB1
//超声波B -> TIM3_CH3 -> PB0
//超声波C -> TIM3_CH1 -> PA6
//超声波D -> TIM2_CH1 -> PA0
//超声波E -> TIM2_CH2 -> PA1
//超声波F -> TIM3_CH2 -> PA7
#define Set_US_A_Rising  TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising)  //上升沿配置
#define Set_US_B_Rising  TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising)
#define Set_US_C_Rising  TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising)
#define Set_US_D_Rising  TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising)
#define Set_US_E_Rising  TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising)
#define Set_US_F_Rising  TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising)

#define Set_US_A_Falling TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling)  //下降沿配置
#define Set_US_B_Falling TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling)
#define Set_US_C_Falling TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling)
#define Set_US_D_Falling TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling)
#define Set_US_E_Falling TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling)
#define Set_US_F_Falling TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling)

#define Get_US_A_CNT TIM_GetCapture4(TIM3) //获取计数器的值
#define Get_US_B_CNT TIM_GetCapture3(TIM3)
#define Get_US_C_CNT TIM_GetCapture1(TIM3)
#define Get_US_D_CNT TIM_GetCapture1(TIM2)
#define Get_US_E_CNT TIM_GetCapture2(TIM2)
#define Get_US_F_CNT TIM_GetCapture2(TIM3)

#define Get_US_A_State TIM_GetITStatus(TIM3, TIM_IT_CC4) //获取中断的状态
#define Get_US_B_State TIM_GetITStatus(TIM3, TIM_IT_CC3)
#define Get_US_C_State TIM_GetITStatus(TIM3, TIM_IT_CC1)
#define Get_US_D_State TIM_GetITStatus(TIM2, TIM_IT_CC1)
#define Get_US_E_State TIM_GetITStatus(TIM2, TIM_IT_CC2)
#define Get_US_F_State TIM_GetITStatus(TIM3, TIM_IT_CC2)

#define Clear_US_A_State TIM_ClearITPendingBit(TIM3, TIM_IT_CC4) //清除中断标志位
#define Clear_US_B_State TIM_ClearITPendingBit(TIM3, TIM_IT_CC3)
#define Clear_US_C_State TIM_ClearITPendingBit(TIM3, TIM_IT_CC1)
#define Clear_US_D_State TIM_ClearITPendingBit(TIM2, TIM_IT_CC1)
#define Clear_US_E_State TIM_ClearITPendingBit(TIM2, TIM_IT_CC2)
#define Clear_US_F_State TIM_ClearITPendingBit(TIM3, TIM_IT_CC2)


#define US_ABCF_Read   TIM3_IRQHandler
#define US_DE_Read    TIM2_IRQHandler

//PE8 ->A
//PE7 ->B
//PC4 ->C
//PA4 ->D
//PA5 ->E
//PC5 ->F
#define TriggerA PEout(8)
#define TriggerB PEout(7)
#define TriggerC PCout(4)
#define TriggerD PAout(4)
#define TriggerE PAout(5)
#define TriggerF PCout(5)

extern volatile u8 US_A_FLag,US_B_FLag,US_C_FLag,US_D_FLag,US_E_FLag,US_F_FLag;
//extern float distance_A, distance_B, distance_C, distance_D, distance_E, distance_F;
extern volatile u8 END_A,END_B,END_C,END_D,END_E,END_F;
void Ultrasonic_Init(void);
void Trigger_IO_Init(void);

extern volatile u8 us_filter_reset_A,us_filter_reset_B,us_filter_reset_C;
extern volatile u8 us_filter_reset_D,us_filter_reset_E,us_filter_reset_F;

float Mean_Filter_A(float dis_A);
float Mean_Filter_B(float dis_B);
float Mean_Filter_C(float dis_C);
float Mean_Filter_D(float dis_D);
float Mean_Filter_E(float dis_E);
float Mean_Filter_F(float dis_F);

#endif
