#include "delay_lib.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

// Initialize Timer for delay functions
void delay_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // Enable TIM2 clock (APB1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // Configure TIM2 for 1MHz (1us resolution)
    // With 100MHz system clock, APB1 = 50MHz, Timer clock = 100MHz (due to APB1 prescaler ≠ 1)
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; // 32-bit max
    TIM_TimeBaseStructure.TIM_Prescaler = 99; // 100MHz / (99+1) = 1MHz = 1us per tick
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // Enable TIM2
    TIM_Cmd(TIM2, ENABLE);
}

void delay_us(unsigned int us)
{
    uint32_t start = TIM_GetCounter(TIM2);
    while ((TIM_GetCounter(TIM2) - start) < us);
}

void delay_ms(unsigned int ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

void delay_s(unsigned int s)
{
    while (s--) {
        delay_ms(1000);
    }
}
