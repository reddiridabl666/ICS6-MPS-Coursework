F_CPU = 8000000
MCU = atmega324p
OUT = build/kursach
SRC = lib/enc28j60.c lib/uart.c lib/display.c lib/LCD.c main.c
INCLUDE = -I lib
DEBUG ?= false

.PHONY: build_slave build_slave hex_324 build_324

all: hex_slave hex_324

hex_slave: build_slave
	avr-objcopy -j .text -j .data -O ihex ${OUT}_324_slave.elf ${OUT}_324_slave.hex

hex_324: build_324
	avr-objcopy -j .text -j .data -O ihex ${OUT}_324.elf ${OUT}_324.hex

build_slave:
	avr-gcc -std=gnu99 -Os ${INCLUDE} -D DEBUG -D SLAVE -D F_CPU=${F_CPU} -g -mmcu=${MCU} ${SRC} -o ${OUT}_324_slave.elf

build_324:
	avr-gcc -std=gnu99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=${F_CPU} -g -mmcu=${MCU} ${SRC} -o ${OUT}_324.elf

clear:
	rm build/*
