#include "delay_lib.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

// Initialize Timer for delay functions
void delay_init(void)
{
    TIM_TimeBaseInitTypeDef timer_config; // Timer configuration structure

    // Enable TIM2 clock (APB1 peripheral)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Configure TIM2 for 1MHz operation (1μs resolution)
    // System clock: 100MHz, APB1: 50MHz, Timer clock: 100MHz (due to APB1 prescaler ≠ 1)
    timer_config.TIM_Period = 0xFFFFFFFF;              // 32-bit maximum count for long delays
    timer_config.TIM_Prescaler = 99;                   // 100MHz / (99+1) = 1MHz = 1μs per tick
    timer_config.TIM_ClockDivision = TIM_CKD_DIV1;     // No additional clock division
    timer_config.TIM_CounterMode = TIM_CounterMode_Up; // Count upward
    TIM_TimeBaseInit(TIM2, &timer_config);

    // Enable TIM2 to start counting
    TIM_Cmd(TIM2, ENABLE);
}

void delay_us(unsigned int microseconds)
{
    uint32_t start_time = TIM_GetCounter(TIM2); // Get current timer count
    while ((TIM_GetCounter(TIM2) - start_time) < microseconds)
        ; // Wait for specified microseconds
}

void delay_ms(unsigned int milliseconds)
{
    while (milliseconds--)
    {                   // Loop for each millisecond
        delay_us(1000); // 1000 microseconds = 1 millisecond
    }
}

void delay_s(unsigned int seconds)
{
    while (seconds--)
    {
        delay_ms(1000);
    }
}
