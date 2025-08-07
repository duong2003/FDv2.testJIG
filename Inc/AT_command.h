#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#include <stdint.h>
#include <stdbool.h>

void AT_handle_command(const char *cmd, char *response);
uint8_t AT_get_mode(void);
void AT_reset_password(void);
void AT_init(void);
bool AT_should_reset(void);
bool AT_process_command_mode(char *command_buffer, char *response_buffer, int buffer_size, int timeout_ms);

#endif // AT_COMMAND_H
