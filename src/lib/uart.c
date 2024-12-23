#include <avr/io.h>
#include "uart.h"

uint8_t _port = 0;

void serial_set_port(int port) {
    _port = port;
}

uint8_t writeSerial(char* str)
{
	for(; *str; str++)
	{
        writeSerialByte(*str);
	}
	return 0;
}

uint8_t writeSerialByte(char c)
{
    #ifdef UCSRA
        while(!(UCSRA&(1<<UDRE))){};
        UDR = c;
    #else
    if (_port == 0) {
        while(!(UCSR0A&(1<<UDRE0))){};
        UDR0 = c;
    } else {
        while(!(UCSR1A&(1<<UDRE1))){};
        UDR1 = c;
    }

    #endif
}

uint8_t readByteSerial() {
    if (uartReady()) {
        #ifdef UCSRA
            return UDR;
        #else
            if (_port == 0) {
                return UDR0;
            } else {
                return UDR1;
            }
        #endif
    }
    return 0;
}

inline uint8_t uartReady() {
    #ifdef UCSRA
        return UCSRA & (1<<RXC);
    #else
        if (_port == 0) {
            return UCSR0A & (1<<RXC0);
        } else {
            return UCSR1A & (1<<RXC1);
        }
    #endif
}
