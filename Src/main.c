#include "AT_command.h"
#include "stm32f4xx.h"
#include "delay_lib.h"
#include "init_config.h"
#include "uart_config.h"
#include <string.h>

#define AT_BUFFER_SIZE 64
#define AT_READ_TIMEOUT_MS 1000  // UART read timeout in milliseconds

int main(void)
{
    // Initialize system configuration first (GPIO, UART, Timer)
    init_config();
    
    // Flush UART completely to clear any pending data
    usart2_flush();

    // Boot message - simple and clean
    usart2_send_string("\nBoot Status: OK\r\n");
    
    // Initialize AT command system and read mode from flash
    AT_init();
    
    char command_buffer[AT_BUFFER_SIZE];     // Buffer for incoming AT commands
    char response_buffer[AT_BUFFER_SIZE];    // Buffer for AT command responses
    
    // Initialize response buffer to ensure clean start
    memset(response_buffer, 0, AT_BUFFER_SIZE);
    
    // Check current mode and send appropriate startup message
    uint8_t operating_mode = AT_get_mode();
    if (operating_mode == 1) {
        usart2_send_string("Mode: Boot Mode\r\n");
        usart2_send_string("Type AT command first\r\n");
    } else {
        usart2_send_string("Mode: Run Mode\r\n");
    }

    // Main application loop
    while (1) {
        operating_mode = AT_get_mode(); // Check current operating mode
        
        if (operating_mode == 1) {      // AT Command Mode
            
            // Clear buffers before processing new command
            memset(command_buffer, 0, AT_BUFFER_SIZE);
            memset(response_buffer, 0, AT_BUFFER_SIZE);
            
            // Read command line from UART with timeout
            int command_length = usart2_read_line(command_buffer, AT_BUFFER_SIZE, AT_READ_TIMEOUT_MS);
            
            if (command_length > 0 && command_buffer[0] != '\0') {
                // Process AT command and generate response
                AT_handle_command(command_buffer, response_buffer);
                
                // Send response back to user
                if (response_buffer[0] != '\0') {
                    usart2_send_string(response_buffer);
                    
                    // Wait for UART transmission to complete before reset
                    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
                    
                    // Check if system reset is required (after AT+END command)
                    if (AT_should_reset()) {
                        delay_ms(100); // Small delay to ensure response is fully sent
                        NVIC_SystemReset();
                    }
                } else {
                    usart2_send_string("ERROR: No response\r\n");
                }
            }
            // Continue silently if timeout or no input
            
        } else if (operating_mode == 2) {  // LED Blink Mode
            // Automatic LED blinking with periodic UART message
            GPIO_SetBits(GPIOC, GPIO_Pin_13);    // Turn LED ON (PC13)
            GPIO_SetBits(GPIOA, GPIO_Pin_10);    // Turn LED ON (PA10)
            delay_s(1);  // Wait 1 second with LEDs on
            
            GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // Turn LED OFF (PC13)
            GPIO_ResetBits(GPIOA, GPIO_Pin_10);  // Turn LED OFF (PA10)
            delay_s(1);  // Wait 1 second with LEDs off
            
            usart2_send_string("hello_world\r\n");  // Send periodic message
        } else {
            // Unknown mode - reset to safe state (mode 1)
            usart2_send_string("Unknown mode, resetting to mode 1\r\n");
            AT_reset_password();   // Reset password and mode state
            delay_s(1);            // Wait before retry
        }
    }

    // This line should never be reached, but added to avoid warning
    return 0;
}
