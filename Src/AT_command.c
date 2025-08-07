#include <stdio.h>
#include "AT_command.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdbool.h>
#include "delay_lib.h"

#define AT_PASSWORD "1234"
// Use flash sector for mode storage - 1 bit: 1=AT mode, 0=run mode
#define MODE_FLASH_ADDR ((uint32_t)0x08004000) // Sector 1 address

static uint8_t current_mode = 1;
static bool password_ok = false;

void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_sector(uint32_t addr) {
    // Wait for any pending operations
    while (FLASH->SR & FLASH_SR_BSY);
    
    // Clear error flags
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
    
    // Configure sector erase
    FLASH->CR &= ~FLASH_CR_SNB; // Clear sector number bits
    FLASH->CR |= FLASH_CR_SER | (1 << 3); // Sector 1 erase
    
    // Start erase operation
    FLASH->CR |= FLASH_CR_STRT;
    
    // Wait for operation to complete
    while (FLASH->SR & FLASH_SR_BSY);
    
    // Check for errors
    if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)) {
        // Clear error flags
        FLASH->SR = FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
    }
    
    // Clear sector erase bit
    FLASH->CR &= ~FLASH_CR_SER;
}

void flash_write_mode(uint8_t mode) {
    // Disable interrupts during flash operation
    __disable_irq();
    
    flash_unlock();
    
    // Wait for any pending operations
    while (FLASH->SR & FLASH_SR_BSY);
    
    // Clear error flags
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
    
    // Always erase sector first
    flash_erase_sector(MODE_FLASH_ADDR);
    
    // If we want run mode (mode 2), program 0x00000000
    // If we want AT mode (mode 1), leave it as 0xFFFFFFFF (already erased)
    if (mode == 2) {
        // Wait for erase to complete
        while (FLASH->SR & FLASH_SR_BSY);
        
        // Clear programming size bits and set to 32-bit
        FLASH->CR &= ~FLASH_CR_PSIZE;
        FLASH->CR |= FLASH_CR_PSIZE_1; // 32-bit programming
        
        // Enable programming
        FLASH->CR |= FLASH_CR_PG;
        
        // Write the data
        *(volatile uint32_t*)MODE_FLASH_ADDR = 0x00000000;
        
        // Wait for programming to complete
        while (FLASH->SR & FLASH_SR_BSY);
        
        // Check for errors
        if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)) {
            // Clear error flags
            FLASH->SR = FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
        }
        
        // Clear programming bit
        FLASH->CR &= ~FLASH_CR_PG;
        
        // Verification with timeout
        volatile int timeout = 10000;
        while (*(volatile uint32_t*)MODE_FLASH_ADDR != 0x00000000 && timeout > 0) {
            timeout--;
        }
    }
    
    flash_lock();
    
    // Re-enable interrupts
    __enable_irq();
}

uint8_t flash_read_mode(void) {
    uint32_t val = *(volatile uint32_t *)MODE_FLASH_ADDR;
    
    // If flash is erased (0xFFFFFFFF), it means AT mode (mode 1)
    // If flash is programmed (0x00000000), it means run mode (mode 2)
    if (val == 0xFFFFFFFF) {
        return 1; // AT mode
    } else {
        return 2; // Run mode
    }
}

void AT_init(void)
{
    current_mode = flash_read_mode(); // Read mode from flash storage
    password_ok = false;
}

void AT_handle_command(const char *cmd, char *response) {
    if (strcmp(cmd, "AT") == 0) {
        strcpy(response, "\r\nOK");
    } else if (strcmp(cmd, "ATC") == 0) {
        strcpy(response, "\r\nSTM32F411CEU6 AT COMMAND READY");
    } else if (strcmp(cmd, "AT+MODE?") == 0) {
        // Debug command to check current mode
        char mode_str[50];
        sprintf(mode_str, "\r\nCURRENT MODE: %d", current_mode);
        strcpy(response, mode_str);
    } else if (strcmp(cmd, "AT+FLASH?") == 0) {
        // Debug command to check flash value
        uint32_t flash_val = *(volatile uint32_t *)MODE_FLASH_ADDR;
        char flash_str[60];
        sprintf(flash_str, "\r\nFLASH: 0x%08X", (unsigned int)flash_val);
        strcpy(response, flash_str);
    } else if (strcmp(cmd, "AT+TEST") == 0) {
        // Test flash write
        flash_write_mode(2); // Try to write run mode
        uint32_t flash_val = *(volatile uint32_t *)MODE_FLASH_ADDR;
        char test_str[80];
        sprintf(test_str, "\r\nTEST WRITE DONE, FLASH: 0x%08X", (unsigned int)flash_val);
        strcpy(response, test_str);
    } else if (strncmp(cmd, "AT+PW=", 6) == 0) {  // Fixed: should be 6, not 7
        const char *password = cmd + 6;  // Fixed: should be 6, not 7
        if (strcmp(password, AT_PASSWORD) == 0) {
            password_ok = true;
            strcpy(response, "\r\nPASSWORD OK");
        } else {
            password_ok = false;
            strcpy(response, "\r\nPASSWORD ERROR. TRY AGAIN.");
        }
    } else if (strcmp(cmd, "AT+END") == 0) {
        if (!password_ok) {
            strcpy(response, "\r\nENTER PASSWORD FIRST");
        } else {
            // Send response first
            strcpy(response, "\r\nEND. RESTARTING IN MODE 2...");
            // Note: Reset will be handled by main.c after sending response
            // Write run mode to flash first
            flash_write_mode(2); // Run mode (mode 2)
            // Set flag for main.c to handle reset after response is sent
            current_mode = 2;
        }
    } else {
        // Check for invalid lowercase letters in command
        bool has_lowercase = false;
        for (int i = 0; cmd[i] != '\0'; i++) {
            if (cmd[i] >= 'a' && cmd[i] <= 'z') {
                has_lowercase = true;
                break;
            }
        }
        
        if (has_lowercase) {
            strcpy(response, "\r\nINVALID COMMAND");
        } else {
            strcpy(response, "\r\nINVALID COMMAND");
        }
    }
}

uint8_t AT_get_mode(void) {
    return current_mode;
}

bool AT_should_reset(void) {
    // Check if we need to reset (after AT+END command)
    return (current_mode == 2 && password_ok);
}

void AT_reset_password(void) {
    password_ok = false;
    current_mode = 1;
    flash_write_mode(current_mode); // Reset to mode 1 in flash storage
}
