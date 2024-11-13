#pragma once

#include <stdint.h>

uint8_t writeSerial(char* str);
uint8_t writeSerialByte(char c);

uint8_t uartReady();

uint16_t readSerialUntil(char* buf, char delim, uint16_t size);
uint8_t readByteSerial();

#ifdef DEBUG
char debug_buf[64]; 
#include <stdio.h>

#define debug_log(...) \
    sprintf(debug_buf, __VA_ARGS__); \
    writeSerial(debug_buf);
#else
    #define debug_log(...)
#endif
