#ifndef F_CPU
    #define F_CPU 8000000
#endif

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>

#include "enc28j60.h"
#include "uart.h"
#include "display.h"
#include "LCD.h"

#define BUF_SIZE ENC28J60_MAXFRAME
#define PORTS_NUM 2
#define CMD_BUF_SIZE 32

uint8_t buf[BUF_SIZE];
uint8_t ports[PORTS_NUM] = {PB1, PB2};

uint16_t transmitted_data[PORTS_NUM] = {0, 0};

uint8_t cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_ptr = 0;

#ifdef DEBUG
uint16_t read_packets[PORTS_NUM] = {0, 0};
#endif

void receive_and_send_to_others(uint8_t port_num);
void process_uart_input();
void display_init();

void init_spi() {
    ENC28J60_SPI_DDR = (1<<PB1)|(1<<PB2)|ENC28J60_SPI_MOSI|ENC28J60_SPI_SCK;
    SPCR = (1<<SPE)|(1<<MSTR);
	SPSR |= (1<<SPI2X); // Maximum speed
}

void init_uart() {
#ifdef UCSRA
    UBRRL=F_CPU / (16 * 38400) - 1;
	UCSRB=(1<<TXEN)|(1<<RXEN);
	UCSRC=(1<<URSEL)|(3<<UCSZ0);
#else
    UBRR0L=F_CPU / (16 * 57600) - 1;
	UCSR0B=(1<<TXEN0)|(1<<RXEN0);
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);
#endif
}

int main() {
    memset(cmd_buf, 0, CMD_BUF_SIZE);
    memset(buf, 0, BUF_SIZE);

    init_spi();
    init_uart();

    display_init();

    for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        enc28j60_init(ports[i]);
    }

    debug_log("enc28j60 initialized\r\n");

    // display_writeString("Hello, world!");

    while (1) {
        LCDstring("Hello, world!", 0, 0);
        _delay_ms(100);
    }
    // while (1) {
    //     for (uint8_t i = 0; i < PORTS_NUM; ++i) {
    //         // debug_log("\r\nreading from PB%d\r\n", i+1);
    //         receive_and_send_to_others(i);
    //     }

    //     process_uart_input();
    // }
}

void process_uart_input() {
    uint8_t byte = readByteSerial();
    
    if (byte) {
        if (byte == '\r' || cmd_ptr >= BUF_SIZE) {
            writeSerial(cmd_buf);
            writeSerial("\r\n");

            cmd_ptr = 0;
            memset(cmd_buf, 0, BUF_SIZE);
        
            return;
        }
        cmd_buf[cmd_ptr++] = byte;
    }
}

void display_init() {
    // PORTA = 0x00;
    // DDRA = 0xFF;

    // _delay_ms(15);
    // display_writeHalfByte(0x30);
    // _delay_ms(5);
    // display_writeHalfByte(0x30);
    // _delay_ms(5);
    // display_writeHalfByte(0x30);

    // display_writeCommand(0x22);
    // display_writeCommand(0x0C);
    // display_writeCommand(0x01);
    _delay_ms(100);
    LCDinit();
    LCDclear();
}

void receive_and_send_to_others(uint8_t port_num) {
    uint16_t read = enc28j60_recv_packet(ports[port_num], buf, BUF_SIZE);
    if (read == 0) {
        return;
    }
    debug_log("Read %u bytes from PB%u\r\n", read, port_num+1);
    transmitted_data[port_num] += read;

#ifdef DEBUG
    read_packets[port_num] += 1;
    debug_log("Total read packets from PB%u: %u\r\n", port_num+1, read_packets[port_num])
#endif

    for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        if (i != port_num) {
            enc28j60_send_packet(ports[i], buf, read);
            debug_log("Send %u bytes from PB%u to PB%u\r\n", read, port_num+1, i+1);
            transmitted_data[i] += read;
        }
    }
}
