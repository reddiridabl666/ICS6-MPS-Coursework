#include "display.h"
#include <avr/io.h>
#include <util/delay.h>

#define STROBE_START PORTA |= 0b00000010
#define STROBE_END   PORTA &= 0b11111101

#define SEND_DATA PORTA |= 0b00000001
#define SEND_CMD  PORTA &= 0b11111110

void display_writeHalfByte(uint8_t byte) {
    byte <<= 4;
    STROBE_START;
    
    _delay_us(50);
    PORTA &= 0b00001111;
    PORTA |= byte;
    
    STROBE_END;
    _delay_us(50);
}

static void writeByte(uint8_t byte) {
    uint8_t high = byte >> 4;
    display_writeHalfByte(high);
    display_writeHalfByte(byte);
}

void display_writeCommand(uint8_t cmd) {
    SEND_CMD;
    writeByte(cmd);
}

void display_writeData(uint8_t cmd) {
    SEND_DATA;
    writeByte(cmd);
}

void display_writeString(uint8_t* str) {
    SEND_DATA;
    for (; str++; *str) {
        writeByte(*str);
    }
}
