#include "delay.h"
#include <stdint.h>

#define CYCLES_PER_US   (CPUCLK_FREQ / 1000000U)
#define CYCLES_PER_MS   (CPUCLK_FREQ / 1000U)

void delay_us(uint32_t us)
{
    // 处理 us=0
    if (us == 0) return;

    // 计算总 cycles，使用 64 位防溢出
    uint64_t cycles = (uint64_t)us * (uint64_t)CYCLES_PER_US;
    delay_cycles((uint32_t)cycles);   // 如果 delay_cycles 只支持 32 位，这里 us 不要太大
}

void delay_ms(uint32_t ms)
{
    if (ms == 0) return;

    // ms 可能很大，建议分段，避免超过 delay_cycles 的上限
    while (ms--)
    {
        delay_cycles(CYCLES_PER_MS);
    }
}