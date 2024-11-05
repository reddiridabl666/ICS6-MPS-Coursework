#include <avr/io.h>
#include "uart.h"

uint8_t writeSerial(char* str)
{
	for(; *str; str++)
	{
        #ifdef UCSRA
            while(!(UCSRA&(1<<UDRE))){};
            UDR = *str;
        #else
            while(!(UCSR0A&(1<<UDRE0))){};
            UDR0 = *str;
        #endif
	}
	return 0;
}

uint16_t readSerial(char *buf, uint16_t size) {
    {
        uint8_t i = 0;
        do {
            while(!uartReady()) {}

            #ifdef UCSRA
                buf[i] = UDR;
            #else
                buf[i] = UDR0;
            #endif

            ++i;
        } while (buf[i - 1] != '\n' && i < size);

        return i;
    }
}

uint8_t readByteSerial() {
    if (uartReady()) {
        #ifdef UCSRA
            return UDR;
        #else
            return UDR0;
        #endif
    }
    return 0;
}

inline uint8_t uartReady() {
    #ifdef UCSRA
        return UCSRA & (1<<RXC);
    #else
        return UCSR0A & (1<<RXC0);
    #endif
}
