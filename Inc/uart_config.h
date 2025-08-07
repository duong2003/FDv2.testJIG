#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#include <stdint.h>

void usart2_send_string(const char *str);
void usart2_echo_char(char c);
void usart2_flush(void);
int usart2_read_line(char *buf, int max_len, uint32_t timeout_ms);

#endif // UART_CONFIG_H
