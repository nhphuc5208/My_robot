#ifndef __FLASH_H
#define __FLASH_H
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
void clear_flash(void);
void write_flash(char *Data, uint16_t len);
void read_flash(char *Data, uint16_t len);
#endif
