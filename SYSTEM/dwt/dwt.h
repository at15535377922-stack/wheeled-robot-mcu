#ifndef __DWT_H
#define __DWT_H
#include "system.h"

//使用到的寄存器地址
#define DWT_CTRL        *(uint32_t*)0xE0001000
#define DWT_CYCCNT      *(uint32_t*)0xE0001004
#define DEM_CR          *(uint32_t*)0xE000EDFC

//DWT的32位计数器最大值
#define DWT_CNT_MAX  4294967295u

extern uint16_t dwt_us;
extern uint32_t dwt_ms;

void DWT_Init(void);

uint32_t DWT_GetTick_us(void);
uint32_t DWT_GetTick_ms(void);

void DWT_delayms(uint32_t delay);
void DWT_delayus(uint32_t delay);

#endif
