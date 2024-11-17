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

#define true 1
#define false 0

#define BUF_SIZE ENC28J60_MAXFRAME
#define PORTS_NUM 2
#define CMD_BUF_SIZE 8
#define CMD_LEN 4
#define MAC_ADDR_LEN 6

#define REGULAR_PING_INTERVAL 5

#define UART_NUM 2
#define RS232 0
#define BT 1
#define PING_DATA_SIZE 2

uint8_t buf[BUF_SIZE];
uint8_t ports[PORTS_NUM] = {PB1, PB2};

uint8_t macaddrs[PORTS_NUM][MAC_ADDR_LEN] = {
    {0x7a, 0xe8, 0x14, 0xa2, 0x6d, 0x86},
    {0x7a, 0xe8, 0x14, 0xa2, 0x6d, 0x97},
};

uint16_t transmitted_data[4] = {0, 0, 0, 0};
uint16_t delays_tmp[4] = {0, 0, 0, 0};
uint16_t delays[4] = {0, 0, 0, 0};

uint8_t ping_port[UART_NUM] = {-1, -1};
uint16_t ping_delay[UART_NUM] = {0, 0};

const char* ping_msg[UART_NUM] = {
    "RS", "BT"
};

uint8_t cmd_buf_rs[CMD_BUF_SIZE];
uint8_t cmd_ptr_rs = 0;

uint8_t cmd_buf_bt[CMD_BUF_SIZE];
uint8_t cmd_ptr_bt = 0;

uint8_t ping_timer = 0;

void receive_and_send_to_others(uint8_t port_num);
void ping(uint8_t port_num, uint8_t sender);
void ping_internal(uint8_t port_num);
void process_ping_result(uint8_t port_num);
void process_uart_input();
void display_init();

void init_spi();
void init_uart();
void init_timers();

int main() {
    memset(cmd_buf_rs, 0, CMD_BUF_SIZE);
    memset(cmd_buf_bt, 0, CMD_BUF_SIZE);
    memset(buf, 0, BUF_SIZE);

    init_spi();
    init_uart();

    display_init();

    init_timers();

    for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        enc28j60_init(ports[i], macaddrs[i]);
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

inline int8_t process_command(char *c) {
    // writeSerial("Got: ");
    // writeSerial(c);
    // writeSerial("\r\n");

    while (isspace(*c)) {
        ++c;
    }

    if (strlen(c) < CMD_LEN + 2 || c[CMD_LEN] != ' ') {
        writeSerial(invalid_cmd);
        return -1;
    }

    c[CMD_LEN] = '\0';
    if (strcmp(c, "ping") != 0) {
        writeSerial(invalid_cmd);
        return -1;
    }

    c += CMD_LEN + 1;
    while (isspace(*c)) {
        ++c;
    }

    uint8_t port = *c - '0';
    if (port < 1 || port > 4) {
        writeSerial(invalid_arg);
        return -1;
    }

    char last = *(c + 1);
    if (last != '\0') {
        writeSerial(invalid_arg);
        return -1;
    }

    writeSerial("ping command parsed, port is: ");
    writeSerial(c);
    writeSerial("\r\n");

    return port;
}

inline void process_uart(uint8_t sender, char* buf, uint8_t *ptr) {
    serial_set_port(sender);

    uint8_t byte = readByteSerial();

    if (byte) {
        if (byte == '\r' || *ptr >= CMD_BUF_SIZE - 1) {
            int8_t port = process_command(buf);
              
            *ptr = 0;
            memset(buf, 0, CMD_BUF_SIZE);

            if (port != -1) {
                ping(port, sender);
            }
            return;
        }
        buf[(*ptr)++] = byte;
    }
}

inline void process_uart_input() {
    process_uart(RS232, cmd_buf_rs, &cmd_ptr_rs);
    process_uart(BT, cmd_buf_bt, &cmd_ptr_bt);
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
    DDRD = 0xFA;

#ifdef UCSRA
    UBRRL=F_CPU / (16 * 38400) - 1;
	UCSRB=(1<<TXEN)|(1<<RXEN);
	UCSRC=(1<<URSEL)|(3<<UCSZ0);
#else
    UBRR0H = 0;
    UBRR0L=F_CPU / (16 * 38400) - 1;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);
	UCSR0C = (3 << UCSZ00);

    UBRR1H = 0;
    UBRR1L=F_CPU / (16 * 38400) - 1;
	UCSR1B = (1<<TXEN1)|(1<<RXEN1);
	UCSR1C = (3 << UCSZ10);
#endif
}

void init_timers() {
    sei();

    #ifdef TIMSK
	    TIMSK = (1<<TOIE1) | (1<<TOIE0);
    #else
        TIMSK1 = (1<<TOIE1) | (1<<TOIE0);
    #endif

	TCNT1 = 0x85EE; // 1 second
	TCNT0 = 131; // 1 ms

    #ifdef TCCR0B
	    TCCR0B = (1<<CS01) | (1<<CS00); // div 64
    #else
        TCCR0 = (1<<CS01) | (1<<CS00); // div 64
    #endif
	
	TCCR1B = (1<<CS12);             // div 256
}

char* lcd_msg_pattern = "P%u %u b/s %u ms";
char lcd_msg[20];

#define apply_lcd_msg(port) \
    memset(lcd_msg, 0, sizeof(lcd_msg)); \
    sprintf(lcd_msg, lcd_msg_pattern, port+1, transmitted_data[port], delays[port])

ISR (TIMER1_OVF_vect) {
    TCNT1 = 0x85EE; // 1 second

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

    ping_timer += 1;
    if (ping_timer >= REGULAR_PING_INTERVAL) {
        for (uint8_t i = 0; i < PORTS_NUM; ++i) {
            ping_internal(i);
        }
        ping_timer = 0;
    }
}

ISR (TIMER0_OVF_vect) {
    TCNT0 = 131; // 1 millisecond

    for (uint8_t i = 0; i < 4; ++i) {
        delays_tmp[i] += 1;
    }

    ping_delay[RS232] += 1;
    ping_delay[BT] += 1;
}

void receive_and_send_to_others(uint8_t port_num) {
    uint16_t read = enc28j60_recv_packet(ports[port_num], buf, BUF_SIZE);
    if (read == 0) {
        return;
    }
    debug_log("Read %u bytes from PB%u\r\n", read, port_num+1);
    transmitted_data[port_num] += read;

    if (strncmp(buf, macaddrs[port_num], MAC_ADDR_LEN) == 0) {
        delays[port_num] = delays_tmp[port_num];
        delays_tmp[port_num] = 0;
    
        for (uint8_t i = 0; i < 2; ++i) {
            if (ping_port[i] == port_num) {
                process_ping_result(i);
            }
        }
    }

    for (uint8_t i = 0; i < PORTS_NUM; ++i) {
        if (i != port_num) {
            enc28j60_send_packet(ports[i], buf, read);
            debug_log("Send %u bytes from PB%u to PB%u\r\n", read, port_num+1, i+1);
            transmitted_data[i] += read;
        }
    }
}

typedef struct eth_frame {
    uint8_t to_addr[6];
    uint8_t from_addr[6];
    uint16_t len;
    uint8_t data[];
} eth_frame_t;

uint8_t broadcast_mac[MAC_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void ping(uint8_t port_num, uint8_t sender) {
    eth_frame_t frame;

    memcpy(frame.from_addr, macaddrs[port_num], MAC_ADDR_LEN);
    memcpy(frame.to_addr, broadcast_mac, MAC_ADDR_LEN);
    frame.len = sizeof(frame) + PING_DATA_SIZE;
    memcpy(frame.data, ping_msg[sender], PING_DATA_SIZE);

    ping_port[sender] = port_num;
    ping_delay[sender] = 0;

    enc28j60_send_packet(ports[port_num], (void*)&frame, frame.len);
}

void ping_internal(uint8_t port_num) {
    eth_frame_t frame;

    memcpy(frame.from_addr, macaddrs[port_num], MAC_ADDR_LEN);
    memcpy(frame.to_addr, broadcast_mac, MAC_ADDR_LEN);
    frame.len = sizeof(frame) + 4;

    delays_tmp[port_num] = 0;
    delays[port_num] = 0;

    memcpy(frame.data, "TEST", 4);

    enc28j60_send_packet(ports[port_num], (void*)&frame, frame.len);
}

char* ping_resp_fmt = "ping ok: %u ms\r\n";
char ping_resp[32];

void process_ping_result(uint8_t sender) {
    eth_frame_t* frame = (eth_frame_t*)buf;
    if (strncmp(frame->data, ping_msg[sender], PING_DATA_SIZE) != 0) {
        return;
    }

    memset(ping_resp, 0, 32);
    sprintf(ping_resp, ping_resp_fmt, ping_delay[sender]);

    serial_set_port(sender);
    writeSerial(ping_resp);
}
