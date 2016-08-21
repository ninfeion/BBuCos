#include "delay.h"


static volatile uint32_t usTicks = 0;
// cycles per microsecond
volatile uint32_t sysTickUptime = 0;
// unit is us,so it will rollover after 71mins. but maybe we won't care.
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}


void SysTick_Handler(void)
{
	sysTickUptime ++;
}


uint32_t currentTime(void)
{
    register uint32_t us, cycle_cnt;
    do 
    {
        us = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (us != sysTickUptime);
    return (us) + (usTicks - cycle_cnt) / usTicks;
}


void delay_current_us(uint32_t nus)
{
	uint32_t t0=currentTime();
	while(currentTime() - t0 < nus);
}


void delay_current_ms(uint32_t nms)
{
	uint32_t t0=currentTime();
	while(currentTime() - t0 < nms * 1000);	
}
