#ifndef F_CPU
    #define F_CPU 8000000
#endif

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "enc28j60.h"
#include "uart.h"
#include "display.h"
#include "LCD.h"

#define BUF_SIZE ENC28J60_MAXFRAME
#define PORTS_NUM 2
#define CMD_BUF_SIZE 8
#define CMD_LEN 4

#define XON  0x11
#define XOFF 0x13

inline void enable_rs() {
    PORTD &= 0xBF; // RTS BT disable
    PORTD |= 0x80; // analogue switch
    PORTD |= 0x20; // RTS RS-232 enable
}

inline void enable_bt() {
    // PORTD &= 0xDF; // RTS RS-232 disable 
    PORTD &= 0x7F; // analogue switch
    PORTD |= 0x40; // RTS BT enable
}

uint8_t buf[BUF_SIZE];
uint8_t ports[PORTS_NUM] = {PB1, PB2};

uint16_t transmitted_data[4] = {0, 0, 0, 0};

uint8_t cmd_buf_rs[CMD_BUF_SIZE];
uint8_t cmd_ptr_rs = 0;

uint8_t cmd_buf_bt[CMD_BUF_SIZE];
uint8_t cmd_ptr_bt = 0;

// #ifdef DEBUG
uint16_t read_packets[4] = {0, 0, 0, 0};
// #endif

void receive_and_send_to_others(uint8_t port_num);
void process_uart_input();
void display_init();

void init_spi();
void init_uart();
void init_timer();

int main() {
    memset(cmd_buf_rs, 0, CMD_BUF_SIZE);
    memset(cmd_buf_bt, 0, CMD_BUF_SIZE);
    memset(buf, 0, BUF_SIZE);

    DDRD = 0xFE;

    init_spi();
    init_uart();

    enable_rs();

    display_init();

    init_timer();

    for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        enc28j60_init(ports[i]);
    }

    debug_log("enc28j60 initialized\r\n");

    LCDstring("Hello, world!", 0, 0);

    while (1) {
        // for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        //     // debug_log("\r\nreading from PB%d\r\n", i+1);
        //     receive_and_send_to_others(i);
        // }

        process_uart_input();
    }
}

char* invalid_cmd = "Invalid command\r\n";
char* invalid_arg = "Invalid argument\r\n";

inline void process_command(char *c) {
    // writeSerial("Before all\r\n");
    // writeSerial(c);
    // writeSerial("\r\n");

    while (isspace(*c)) {
        ++c;
    }

    // writeSerial("1 check\r\n");
    // writeSerial(c);
    // writeSerial("\r\n");
    if (strlen(c) < CMD_LEN + 2 || c[CMD_LEN] != ' ') {
        writeSerial(invalid_cmd);
        return;
    }

    c[CMD_LEN] = '\0';
    // writeSerial("2 check\r\n");
    // writeSerial(c);
    // writeSerial("\r\n");
    if (strcmp(c, "ping") != 0) {
        writeSerial(invalid_cmd);
        return;
    }

    c += CMD_LEN + 1;
    while (isspace(*c)) {
        ++c;
    }

    uint8_t port = *c - '0';
    if (port < 1 || port > 4) {
        writeSerial(invalid_arg);
        return;
    }

    char last = *(c + 1);
    if (last != '\0') {
        writeSerial(invalid_arg);
        return;
    }

    writeSerial("ping command parsed, port is: ");
    writeSerial(c);
    writeSerial("\r\n");
}

inline void process_uart(char* buf, uint8_t *ptr) {
    uint8_t byte = readByteSerial();
    PORTD &= 0x9F; // RTS disable

    if (byte) {
        if (byte == '\r' || *ptr >= CMD_BUF_SIZE) {
            process_command(buf);

            *ptr = 0;
            memset(buf, 0, CMD_BUF_SIZE);
        
            return;
        }
        buf[(*ptr)++] = byte;
    }

    // if (!uartReady()) {
    //     return;
    // }
    // readSerialUntil(buf, '\r', CMD_BUF_SIZE);
    // process_command(buf);

    // memset(buf, 0, CMD_BUF_SIZE);
}

inline void process_uart_input() {
    process_uart(cmd_buf_rs, &cmd_ptr_rs);
    enable_bt();
    process_uart(cmd_buf_bt, &cmd_ptr_bt);
    enable_rs();
}

void display_init() {
    LCDinit();
    LCDclear();
}

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

void init_timer() {
    sei();
	TIMSK = (1<<TOIE1);

	TCNT1 = 0x85EE;

	TCCR1B = (1<<CS12);
}

char* lcd_msg_pattern = "P%u %u b/s %u pkt";
char lcd_msg[20];

#define apply_lcd_msg(port) \
    memset(lcd_msg, 0, sizeof(lcd_msg)); \
    sprintf(lcd_msg, lcd_msg_pattern, port+1, transmitted_data[port], read_packets[port])

ISR (TIMER1_OVF_vect) {
    TCNT1H = 0x85;
	TCNT1L = 0xEE;

    apply_lcd_msg(0);
    LCDstring(lcd_msg, 0, 0);

    apply_lcd_msg(1);
    LCDstring(lcd_msg, 0, 1);

    apply_lcd_msg(2);
    LCDstring(lcd_msg, 20, 0);

    apply_lcd_msg(3);
    LCDstring(lcd_msg, 20, 1);

    for (uint8_t i = 0; i < 4; ++i) {
        transmitted_data[i] = 0;
    }
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
