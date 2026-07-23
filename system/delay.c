#include "delay.h"

void delay_ms(uint32_t ms)
{
   uint32_t cycle = (CPUCLK_FREQ / 1000) * ms;
   delay_cycles(cycle);
}

void delay_us(uint32_t us)
{
   uint32_t cycle = (CPUCLK_FREQ / 1000000) * us;
   delay_cycles(cycle);
}