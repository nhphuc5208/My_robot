#ifndef __UART_H
#define __UART_H
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#define buffer 256
extern UART_HandleTypeDef huart2;
void clear_buffer(char *buff);
void send_uart(char *string);
char *angletostring(double roll, double pitch, double yaw);
#endif
