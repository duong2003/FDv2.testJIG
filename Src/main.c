#include "AT_command.h"
#include "stm32f4xx.h"
#include "delay_lib.h"
#include "init_config.h"
#include "uart_config.h"
#include <string.h>

#define AT_BUFFER_SIZE 64
#define AT_READ_TIMEOUT_MS 1000  // Reduced timeout to 1 second

int main(void)
{
    // Initialize system configuration first
    init_config();
    
    // Flush UART completely
    usart2_flush();

    // Boot message - simple and clean
    usart2_send_string("\r\nBoot Status: OK\r\n");
    
    // Then initialize AT command system
    AT_init(); // Re-enable AT command system
    
    char at_buf[AT_BUFFER_SIZE];
    char at_resp[AT_BUFFER_SIZE];
    
    // Initialize response buffer
    memset(at_resp, 0, AT_BUFFER_SIZE);
    
    // Test UART: send log if in mode 1
    uint8_t current_mode = AT_get_mode();
    if (current_mode == 1) {
        usart2_send_string("Mode: Boot Mode\r\n");
        usart2_send_string("Type AT command first\r\n");
    } else {
        usart2_send_string("Mode: Run Mode\r\n");
    }

    // Main loop
    while (1) {
        current_mode = AT_get_mode(); // Re-enable AT_get_mode
        
        if (current_mode == 1) {
            
            // Clear buffers
            memset(at_buf, 0, AT_BUFFER_SIZE);
            memset(at_resp, 0, AT_BUFFER_SIZE);
            
            int len = usart2_read_line(at_buf, AT_BUFFER_SIZE, AT_READ_TIMEOUT_MS);
            
            if (len > 0 && at_buf[0] != '\0') {
                // Process AT command
                AT_handle_command(at_buf, at_resp);
                
                // Send response
                if (at_resp[0] != '\0') {
                    usart2_send_string(at_resp);
                    
                    // Wait for UART transmission to complete
                    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
                    
                    // Check if we need to reset after sending response
                    if (AT_should_reset()) {
                        delay_ms(100); // Small delay to ensure response is fully sent
                        NVIC_SystemReset();
                    }
                } else {
                    usart2_send_string("ERROR: No response\r\n");
                }
            }
            // Remove timeout messages - just continue silently
            
        } else if (current_mode == 2) {
            // Auto LED Mode
            GPIO_SetBits(GPIOC, GPIO_Pin_13);    // Turn LED ON (PC13)
            GPIO_SetBits(GPIOA, GPIO_Pin_10);    // Turn LED ON (PA10)
            delay_s(1);  // 1 second delay
            
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // Turn LED OFF (PC13)
            GPIO_ResetBits(GPIOA, GPIO_Pin_10);  // Turn LED OFF (PA10)
            delay_s(1);  // 1 second delay
            
            usart2_send_string("hello_world\r\n");
        } else {
            // Unknown mode - reset to mode 1
            usart2_send_string("Unknown mode, resetting to mode 1\r\n");
            AT_reset_password();
            delay_s(1);
        }
    }

    // This line should never be reached, but added to avoid warning
    return 0;
}
