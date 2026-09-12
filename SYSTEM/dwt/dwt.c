#include "dwt.h"

/*
DWT使用步骤：
a.先使能DWT外设，由内核调试寄存器DEM_CR的位24控制，写1使能。   DEM_CR地址0xE000EDFC
b.使能CYCCNT寄存器之前，先清0。                              CYCCNT地址0xE001004
c.使能CYCCNT寄存器，由DWT_CTRL的位0控制，写1使能。            DWT_CTRL地址0xE0001000

Cortex-M3、4都存在DWT计数器
使用DWT计数器的意义：
1.可节省定时器完成精准延迟
2.无需使用到SysTick，兼容实时操作系统
*/

//DWT延迟因子
uint16_t dwt_us;
uint32_t dwt_ms;

void DWT_Init(void)
{
	//1.使用DWT前必须使能DGB的系统跟踪（《Cortex-M3权威指南》）
	DEM_CR |= 1<<24;

	//2.计数器清0
	DWT_CYCCNT = (uint32_t)0u;

	//3.开启计时
	DWT_CTRL |= 1<<0;

	//4.计算DWT微秒延迟函数的延迟因子
	dwt_us = SystemCoreClock/1000000;

	//5.毫秒延迟因子
	dwt_ms = dwt_us*1000;
}

//返回32位计数器CYCCNT的值，用于毫秒与微秒延迟
uint32_t DWT_GetTick_us(void)/* 自定义函数 */
{
    return (uint32_t)DWT_CYCCNT;
}


uint32_t DWT_GetTick_ms(void)/* 自定义函数 */
{
    //把计数值除以内核频率的1000分之一倍，实现1毫秒返回1
    //以C8T6为例：因为计数到72是1微秒（1÷72MHz）
    //能返回的最大的毫秒值是 2^32 - 1 / 72000 = 59652  (取整数)
    return ((uint32_t)DWT_CYCCNT/(SystemCoreClock/1000));
}


//DWT毫秒级延迟（重写HAL_Delay实现）
//Delay 范围： 0 ~ ( 2^32-1 / （系统时钟÷1000））
//72MHz时钟： 0~59652
//84MHz时钟： 0~51130
//180MHz时钟：0~23860
//400Mhz时钟：0~10737
void DWT_delayms(uint32_t delay)//使用DWT完成毫秒级延迟函数
{
	uint32_t tick = DWT_GetTick_ms();     //获取当前的数值
	uint64_t wait = delay*dwt_ms+tick;  //计算需要延迟的毫秒对应的微秒数
	uint32_t overflow_count = 0;
	uint32_t now_tick;

    if(wait>DWT_CNT_MAX) //如果有溢出
    {
        overflow_count = wait - DWT_CNT_MAX;//计算溢出的数值
        do{
            now_tick = DWT_GetTick_ms();
        }while(now_tick<overflow_count||now_tick>overflow_count);         
    }
    else//没有溢出，等待到对应的毫秒即可
    {
        do{
            now_tick = DWT_GetTick_ms();
        }while(now_tick<wait);
    }
}

//DWT微秒级延迟
//Delay 范围：是毫秒级延迟的1000倍
//72MHz时钟： 0~59652000
//84MHz时钟： 0~51130000
//180MHz时钟：0~23860000
//400Mhz时钟：0~10737000
void DWT_delayus(uint32_t Delay)//使用DWT完成微秒级延迟函数
{
    uint32_t tick = DWT_GetTick_us();   //获取当前的数值
    uint64_t wait = Delay*dwt_us+tick;  //计算需要延迟的毫秒对应的微秒数
    uint32_t overflow_count = 0;
    uint32_t now_tick;

    if(wait>DWT_CNT_MAX) //如果有溢出
    {
        overflow_count = wait - DWT_CNT_MAX;//计算溢出的数值
        do{
            now_tick = DWT_GetTick_us();
        }while(now_tick<overflow_count||now_tick>overflow_count);         
    }
    else//没有溢出，等待到对应的毫秒即可
    {
        do{
            now_tick = DWT_GetTick_us();
        }while(now_tick<wait);
    }
}


