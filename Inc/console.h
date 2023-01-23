#pragma once

#include <stdio.h>
#include <sys/stat.h>

_BEGIN_STD_C

void console_retarget_init(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

_END_STD_C