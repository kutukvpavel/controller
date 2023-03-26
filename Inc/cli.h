#pragma once

#include "user.h"

#define CLI_NAME stm32f4
#include "../cli/inc/sys_command_line.h"

void my_cli_init(UART_HandleTypeDef* cli_uart);