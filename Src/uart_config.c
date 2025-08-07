#include "stm32f4xx.h"
#include <string.h>
#include "uart_config.h"

void usart2_send_string(const char *str)
{
    while (*str) {  // Loop through each character in string
        // Wait until transmit data register is empty
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, *str++);  // Send character and move to next
    }
}

/*
void usart2_echo_char(char c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}
*/

void usart2_flush(void)
{
    // Wait for all data to be transmitted completely
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    
    // Clear any pending data in RX buffer by reading all available data
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
        USART_ReceiveData(USART2);  // Read and discard pending data
    }
}

int usart2_read_line(char *buf, int max_len, uint32_t timeout_ms)
{
    int char_index = 0;                         // Index for character position in buffer
    uint32_t timeout_counter = 0;               // Counter for timeout tracking
    uint32_t timeout_max_count = timeout_ms * 100; // Maximum timeout count (timeout_ms * 100 loops)
    
    // Clear buffer first
    memset(buf, 0, max_len);
    
    while (char_index < max_len - 1) {          // Loop until buffer is almost full
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
            char received_char = USART_ReceiveData(USART2) & 0xFF;  // Read received character
            // usart2_echo_char(received_char); // echo disabled
            
            if (received_char == '\r' || received_char == '\n') {  // Check for Enter key
                usart2_send_string("\r\n");     // Send newline for better formatting
                buf[char_index] = '\0';          // Null terminate string
                return char_index;               // Return number of characters received
            }
            
            buf[char_index++] = received_char;   // Store character and increment index
            timeout_counter = 0;                 // Reset timeout when receiving data
        } else {
            // Small delay (~10us per loop) to prevent CPU spinning
            for(volatile int delay_loop = 0; delay_loop < 100; delay_loop++);
            timeout_counter++;                   // Increment timeout counter
            if (timeout_counter > timeout_max_count) {
                break;                           // Timeout reached, exit loop
            }
        }
    }
    buf[char_index] = '\0';                     // Ensure string is null terminated
    return char_index;                          // Return number of characters received
}
