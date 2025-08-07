# STM32F411CEU6 AT Command System - Technical Report

## 📋 Project Overview

**Project Name**: STM32F411CEU6 AT Command System  
**MCU**: STM32F411CEU6  
**Communication**: UART2 (115200 baud, 8N1)  
**Storage**: Flash Sector 1 (0x08004000)  
**Development Date**: August 2025  

## 🎯 System Purpose

Dual-mode STM32 system supporting:
- **AT Mode (Mode 1)**: Interactive AT command interface via UART
- **Run Mode (Mode 2)**: Autonomous LED blinking with periodic UART output

Mode selection persists through power cycles using Flash memory storage.

## 🏗️ Architecture Overview

### File Structure
```
STM32F411CEU6-Template/
├── Src/
│   ├── main.c           # Main application loop
│   ├── AT_command.c     # AT command processing & flash storage
│   ├── init_config.c    # Hardware initialization
│   ├── uart_config.c    # UART communication functions
│   └── delay_lib.c      # Timer-based delays
├── Inc/
│   ├── AT_command.h     # AT command interface
│   ├── init_config.h    # Hardware configuration
│   └── uart_config.h    # UART interface
└── REPORT.md           # This documentation
```

### System Components
1. **Main Controller** (`main.c`) - Core application logic
2. **AT Command Processor** (`AT_command.c`) - Command parsing & execution
3. **Hardware Abstraction** (`init_config.c`) - GPIO, USART, Timer setup
4. **Communication Layer** (`uart_config.c`) - UART functions
5. **Flash Storage Manager** - Non-volatile mode persistence

## 🔧 Hardware Configuration

### GPIO Mapping
| Pin | Function | Description |
|-----|----------|-------------|
| PA2 | USART2_TX | UART transmission |
| PA3 | USART2_RX | UART reception |
| PC13 | GPIO_OUT | LED control (active high) |
| PA10 | GPIO_OUT | LED control (active high) |

### USART2 Configuration
- **Baudrate**: 115200 bps
- **Data**: 8 bits
- **Parity**: None
- **Stop bits**: 1
- **Flow control**: None
- **Mode**: TX + RX

### Flash Memory Layout
- **Address**: 0x08004000 (Sector 1)
- **Size**: 4 bytes (1 word)
- **Logic**: 
  - `0xFFFFFFFF` → AT Mode (erased state)
  - `0x00000000` → Run Mode (programmed state)

## 📊 System Flow Diagrams

### Boot Sequence
```
System Reset
     ↓
Hardware Init (init_config)
     ↓
UART Flush & Boot Message
     ↓
AT System Init (flash_read_mode)
     ↓
Mode Detection
     ↓
┌─────────────┬─────────────┐
│  Mode 1     │   Mode 2    │
│ (AT Mode)   │ (Run Mode)  │
└─────────────┴─────────────┘
```

### AT Command Processing
```
UART Input → Echo Characters → Line Complete?
                                     ↓
                              AT_handle_command()
                                     ↓
                              Command Validation
                                     ↓
                              Response Generation
                                     ↓
                              UART Output
                                     ↓
                              Reset Check (AT+END)
```

### Mode Switching Flow
```
AT+PW=1234 → Password OK → AT+END → Flash Write → Response → Reset → Run Mode
```

## 🎮 AT Command Reference

### Basic Commands
| Command | Description | Response | Requirements |
|---------|-------------|----------|--------------|
| `AT` | Test command | `OK` | None |
| `ATC` | System info | `STM32F411CEU6 AT COMMAND READY` | None |

### System Commands
| Command | Description | Response | Requirements |
|---------|-------------|----------|--------------|
| `AT+MODE?` | Check current mode | `CURRENT MODE: X` | None |
| `AT+FLASH?` | Check flash value | `FLASH: 0xXXXXXXXX` | None |
| `AT+TEST` | Test flash write | `TEST WRITE DONE, FLASH: 0xXXXXXXXX` | None |

### Security Commands
| Command | Description | Response | Requirements |
|---------|-------------|----------|--------------|
| `AT+PW=1234` | Enter password | `PASSWORD OK` / `PASSWORD ERROR` | Correct password |
| `AT+END` | Switch to Run Mode | `END. RESTARTING IN MODE 2...` | Password required |

### Command Rules
- **Case Sensitive**: Only uppercase commands accepted
- **Password Protection**: AT+END requires prior authentication
- **Error Handling**: Invalid commands return `INVALID COMMAND`
- **Echo**: All input characters are echoed back immediately

## 💾 Flash Storage System

### Flash Operations
```c
// Unlock sequence
FLASH->KEYR = 0x45670123;
FLASH->KEYR = 0xCDEF89AB;

// Erase sector 1
FLASH->CR |= FLASH_CR_SER | (1 << 3);
FLASH->CR |= FLASH_CR_STRT;

// Program word (Run Mode only)
*(volatile uint32_t*)0x08004000 = 0x00000000;

// Lock flash
FLASH->CR |= FLASH_CR_LOCK;
```

### Storage Logic
- **AT Mode**: Flash remains erased (0xFFFFFFFF)
- **Run Mode**: Flash programmed to 0x00000000
- **Error Handling**: Clear status flags, verify writes
- **Atomic Operations**: Interrupts disabled during flash access

## 🔄 Operating Modes

### Mode 1: AT Command Mode
**Characteristics:**
- Interactive UART command interface
- Real-time command processing
- Echo input characters
- 1-second timeout for commands
- Password-protected mode switching

**Behavior:**
```
Boot Status: OK
Mode: Boot Mode
Type AT command first
[Wait for user input]
```

### Mode 2: Run Mode (LED Blinking)
**Characteristics:**
- Autonomous operation
- LED blinking pattern: 1s ON, 1s OFF
- Periodic UART output
- No user interaction required

**Behavior:**
```
Boot Status: OK
Mode: Run Mode
[LED PC13, PA10 ON] - 1 second
[LED PC13, PA10 OFF] - 1 second
hello_world
[Repeat cycle]
```

## 🔐 Security Features

### Password System
- **Password**: "1234" (hardcoded)
- **Session-based**: Valid until system reset
- **Required for**: Mode switching (AT+END)
- **Validation**: Exact string match

### Access Control
```c
if (!password_ok) {
    strcpy(response, "\r\nENTER PASSWORD FIRST");
} else {
    // Allow mode switching
}
```

## 🧪 Testing & Debug

### Manual Testing Commands
```bash
# Build project
make

# Flash firmware
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program build/template.hex verify reset exit"

# Manual mode control
# Set Run Mode
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "init" -c "halt" -c "flash erase_sector 0 1 1" -c "flash fillw 0x08004000 0x00000000 1" -c "reset" -c "exit"

# Set AT Mode
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "init" -c "halt" -c "flash erase_sector 0 1 1" -c "reset" -c "exit"
```

### Debug Commands
- `AT+MODE?` - Check current operational mode
- `AT+FLASH?` - Verify flash memory content
- `AT+TEST` - Test flash write functionality

### Test Scenarios
1. **Basic Commands**: AT, ATC functionality
2. **Password Flow**: AT+PW=1234 → AT+END
3. **Mode Persistence**: Power cycle, verify mode retention
4. **Error Cases**: Wrong password, invalid commands
5. **Flash Operations**: Manual flash manipulation

## 📈 Performance Metrics

### Memory Usage
- **Text Section**: ~8544 bytes
- **Data Section**: 84 bytes
- **BSS Section**: 1900 bytes
- **Total Flash**: ~8.6KB
- **Total RAM**: ~2KB

### Timing Characteristics
- **UART Timeout**: 1000ms
- **Flash Write**: <100ms
- **LED Cycle**: 2000ms (1s ON + 1s OFF)
- **Reset Delay**: 100ms post-response
- **Command Response**: <1ms

## 🚨 Error Handling

### Flash Error Recovery
- Clear error flags before operations
- Verify write completion with timeout
- Interrupt management during critical sections

### UART Error Management
- Timeout-based input handling
- Buffer overflow prevention
- Transmission completion verification

### System Robustness
- Watchdog-style timeout mechanisms
- Graceful mode switching
- Power-cycle persistence

## 🔧 Development Notes

### Code Quality
- **Modularity**: Clear separation of concerns
- **Documentation**: Inline comments and function headers
- **Error Handling**: Comprehensive error checking
- **Memory Safety**: Buffer bounds checking

### Known Limitations
1. **Single Flash Sector**: Limited to one mode storage location
2. **Hardcoded Password**: Password embedded in firmware
3. **No Encryption**: Plain text flash storage
4. **Limited Commands**: Fixed command set

### Future Enhancements
1. **Dynamic Password**: EEPROM-stored password
2. **Multiple Modes**: Expandable mode system
3. **Encrypted Storage**: Secure flash storage
4. **Remote Update**: OTA firmware capability
5. **Command Expansion**: User-definable commands

## 📋 Compliance & Standards

### Development Environment
- **Toolchain**: ARM GCC 14.3 rel1
- **Debugger**: OpenOCD + ST-Link V2
- **Build System**: Makefile
- **Version Control**: Git

### Code Standards
- **Style**: Consistent indentation and naming
- **Comments**: Function and complex logic documentation
- **Error Codes**: Systematic error handling
- **Testing**: Manual validation procedures

---

**Report Generated**: August 7, 2025  
**Version**: 1.0  
**Author**: Development Team  
**Status**: Production Ready
