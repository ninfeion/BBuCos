#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32f10x.h"

void cycleCounterInit(void);
uint32_t currentTime(void);
void delay_current_us(uint32_t nus);
void delay_current_ms(uint32_t nms);

#endif