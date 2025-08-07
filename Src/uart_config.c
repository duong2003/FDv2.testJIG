#include "stm32f4xx.h"
#include <string.h>
#include "uart_config.h"

void usart2_send_string(const char *str)
{
    while (*str) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);
    }
}

void usart2_echo_char(char c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

void usart2_flush(void)
{
    // Wait for all data to be transmitted
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    // Clear any pending data in RX buffer
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
        USART_ReceiveData(USART2);
    }
}

int usart2_read_line(char *buf, int max_len, uint32_t timeout_ms)
{
    int idx = 0;
    uint32_t timeout_count = 0;
    uint32_t timeout_limit = timeout_ms * 100; // Adjusted for delay loop
    // Clear buffer first
    memset(buf, 0, max_len);
    while (idx < max_len - 1) {
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
            char c = USART_ReceiveData(USART2) & 0xFF;
            usart2_echo_char(c); // echo received character
            if (c == '\r' || c == '\n') {
                usart2_send_string("\r\n"); // Send newline for better formatting
                break;
            }
            buf[idx++] = c; // Keep original case
            timeout_count = 0; // Reset timeout when receiving data
        } else {
            // Small delay (~10us per loop)
            for(volatile int i = 0; i < 100; i++);
            timeout_count++;
            if (timeout_count > timeout_limit) {
                break; // Timeout reached
            }
        }
    }
    buf[idx] = '\0';
    return idx;
}
