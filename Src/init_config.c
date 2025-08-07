#include "stm32f4xx.h"
#include "delay_lib.h"
#include "init_config.h"
#include "uart_config.h"

void init_config(void)
{
    GPIO_InitTypeDef gpioInit;
    USART_InitTypeDef usartInit;

    // Initialize timer-based delay system
    delay_init();

    // Enable clock for GPIOA and USART2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // PA2 (TX) as alternate function
    gpioInit.GPIO_Pin = GPIO_Pin_2;
    gpioInit.GPIO_Mode = GPIO_Mode_AF;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpioInit);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    // PA3 (RX) as alternate function (optional)
    gpioInit.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &gpioInit);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    // USART2 config: 115200 8N1
    usartInit.USART_BaudRate = 115200;
    usartInit.USART_WordLength = USART_WordLength_8b;
    usartInit.USART_StopBits = USART_StopBits_1;
    usartInit.USART_Parity = USART_Parity_No;
    usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usartInit);
    USART_Cmd(USART2, ENABLE);

    // Enable clock for GPIOC (PC13) and GPIOA (PA10)
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // Configure GPIO PC13 as output
    gpioInit.GPIO_Mode = GPIO_Mode_OUT;
    gpioInit.GPIO_OType = GPIO_OType_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_13;
    gpioInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpioInit.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOC, &gpioInit);

    // Configure GPIO PA10 as output
    gpioInit.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpioInit);
}
