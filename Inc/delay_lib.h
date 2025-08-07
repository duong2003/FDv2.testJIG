#ifndef __DELAY_LIB_H
#define __DELAY_LIB_H

#include "stm32f4xx.h"

// Initialize timer-based delay system
void delay_init(void);

// Delay functions
void delay_us(unsigned int us);   // Microsecond delay
void delay_ms(unsigned int ms);   // Millisecond delay  
void delay_s(unsigned int s);     // Second delay

#endif // __DELAY_LIB_H
