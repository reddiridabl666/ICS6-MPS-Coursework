#pragma once

#include <stdint.h>

void display_writeCommand(uint8_t cmd);

void display_writeData(uint8_t byte);

void display_writeString(uint8_t* str);

void display_writeHalfByte(uint8_t byte);
