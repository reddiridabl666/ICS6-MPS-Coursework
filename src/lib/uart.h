#pragma once

#include <stdint.h>

void serial_set_port(int port);

uint8_t writeSerial(char* str);
uint8_t writeSerialByte(char c);

uint8_t uartReady();

uint8_t readByteSerial();

#ifdef DEBUG
char debug_buf[64]; 
#include <stdio.h>

#define debug_log(...) \
    sprintf(debug_buf, __VA_ARGS__); \
    serial_set_port(0); \
    writeSerial(debug_buf); \
    serial_set_port(1); \
    writeSerial(debug_buf);
#else
    #define debug_log(...)
#endif
