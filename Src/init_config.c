#include "stm32f4xx.h"
#include "delay_lib.h"
#include "init_config.h"
#include "uart_config.h"

void init_config(void)
{
    GPIO_InitTypeDef gpio_config;     // GPIO configuration structure
    USART_InitTypeDef uart_config;    // UART configuration structure

    // Initialize timer-based delay system first (required for other functions)
    delay_init();

    // Enable peripheral clocks for GPIO and UART
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // Enable GPIOA clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Enable USART2 clock

    // Configure PA2 (USART2_TX) as alternate function
    gpio_config.GPIO_Pin = GPIO_Pin_2;           // Pin 2 for TX
    gpio_config.GPIO_Mode = GPIO_Mode_AF;        // Alternate function mode
    gpio_config.GPIO_Speed = GPIO_High_Speed;    // High speed for reliable communication
    gpio_config.GPIO_OType = GPIO_OType_PP;      // Push-pull output
    gpio_config.GPIO_PuPd = GPIO_PuPd_UP;        // Pull-up resistor (idle high)
    GPIO_Init(GPIOA, &gpio_config);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // Set alternate function

    // Configure PA3 (USART2_RX) as alternate function
    gpio_config.GPIO_Pin = GPIO_Pin_3;           // Pin 3 for RX
    GPIO_Init(GPIOA, &gpio_config);              // Same config as TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // Set alternate function

    // Configure USART2 parameters: 115200 baud, 8 data bits, no parity, 1 stop bit
    uart_config.USART_BaudRate = 115200;
    uart_config.USART_WordLength = USART_WordLength_8b;
    uart_config.USART_StopBits = USART_StopBits_1;
    uart_config.USART_Parity = USART_Parity_No;
    uart_config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_config.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &uart_config);
    USART_Cmd(USART2, ENABLE);

    // Enable clock for GPIO ports used for LEDs
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // Enable GPIOC for PC13
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // Enable GPIOA for PA10

    // Configure PC13 as output pin for LED control
    gpio_config.GPIO_Mode = GPIO_Mode_OUT;
    gpio_config.GPIO_OType = GPIO_OType_PP;
    gpio_config.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_config.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(GPIOC, &gpio_config);

    // Configure PA10 as output pin for LED control
    gpio_config.GPIO_Mode = GPIO_Mode_OUT;
    gpio_config.GPIO_OType = GPIO_OType_OD;
    gpio_config.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_config.GPIO_Speed = GPIO_High_Speed;
    gpio_config.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio_config);
}
