#define F_CPU 8000000

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include "enc28j60.h"
// #include <avr/iom32.h>

#define BUF_SIZE 500
#define PORTS_NUM 2

uint8_t buf[BUF_SIZE];
uint8_t ports[PORTS_NUM] = {PB1, PB2};

uint8_t i = 0;

// void receive_and_send_to_others(uint8_t port_num);

void receive_and_send_to_others_simple(uint8_t port_num);

int writeSerial(char* str)
{
	for(; *str; str++)
	{
		while(!(UCSRA&(1<<UDRE))){};
		UDR = *str;
	}
	return 0;
}

// #define DEBUG

#ifdef DEBUG
char debug_buf[100]; 
#endif

#ifdef DEBUG
#define debug_log(...) \
    sprintf(debug_buf, __VA_ARGS__); \
    writeSerial(debug_buf);
#else
    #define debug_log(...)
#endif

int main() {
    ENC28J60_SPI_DDR = (1<<PB1)|(1<<PB2)|ENC28J60_SPI_MOSI|ENC28J60_SPI_SCK;

    UBRRL=F_CPU / (16 * 38400) - 1;
	UCSRB=(1<<TXEN)|(1<<RXEN);
	UCSRC=(1<<URSEL)|(3<<UCSZ0);

    for (i = 0; i < PORTS_NUM; ++i) {
        enc28j60_init(ports[i]);
    }

    debug_log("enc28j60 initialized\r\n");

    while (1) {
        for (i = 0; i < PORTS_NUM; ++i) {
            debug_log("reading from PB%d\r\n", i+1);
            // receive_and_send_to_others(i);
            receive_and_send_to_others_simple(i);
            _delay_ms(100);
        }
    }
}

void receive_and_send_to_others_simple(uint8_t port_num) {
    uint16_t read = enc28j60_recv_packet(ports[port_num], buf, BUF_SIZE);
    if (read == 0) {
        return;
    }
    debug_log("Read %u bytes from PB%u\r\n", read, port_num+1);

    for (i = 0; i < PORTS_NUM; ++i) {
        if (i != port_num) {
            enc28j60_send_packet(ports[i], buf, read);
            debug_log("Send %u bytes from PB%u to PB%u\r\n", read, port_num+1, i+1);
        }
    }
}

/* uint16_t total_read = 0;

void receive_and_send_to_others(uint8_t port_num) {
    uint8_t port = ports[port_num];

    uint8_t status = enc28j60_recv_packet_start(port);
    if (status == ENC28J60_NO_DATA) {
        debug_log("PB%d NO DATA\r\n", port_num+1);
        return;
    } else if (status == ENC28J60_BAD) {
        debug_log("PB%d BAD STATUS\r\n", port_num+1);
        enc28j60_recv_packet_end(port);
        return;
    }
    
    for (i = 0; i < PORTS_NUM; ++i) {
        if (i != port_num) {
            enc28j60_send_packet_start(ports[i]);
        }
    }

    debug_log("Ports send started\r\n", port_num+1);

    ReadStatus read_status;
    do {
        read_status = enc28j60_recv_packet_part(port, buf, BUF_SIZE);
        debug_log("Received %d bytes from PB%d\r\n", read_status.read, port_num+1);

        if (read_status.read > 0) {
            total_read += read_status.read;

            for (i = 0; i < PORTS_NUM; ++i) {
                if (i != port_num) {
                    enc28j60_send_packet_part(ports[i], buf, read_status.read);
                    debug_log("Send %d bytes from PB%d to PB%d\r\n", read_status.read, port_num+1, i+1);
                }
            }
        }
    }
    while (!read_status.done && read_status.read > 0);

    enc28j60_recv_packet_end(port);
    debug_log("Receive end for PB%d\r\n", port_num+1);


    for (i = 0; i < PORTS_NUM; ++i) {
        if (i != port_num) {
            enc28j60_send_packet_end(ports[i], total_read);
        }
    }
    debug_log("Ports send end\r\n");
}
 */
