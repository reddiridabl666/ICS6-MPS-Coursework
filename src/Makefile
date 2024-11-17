F_CPU = 8000000
# MCU = atmega8515
MCU = atmega32
OUT = build/kursach
SRC = lib/enc28j60.c lib/uart.c lib/display.c lib/LCD.c main.c
INCLUDE = -I lib
DEBUG ?= false

.PHONY: build hex hex_324 build_324

all: hex hex_324

hex: build
	avr-objcopy -j .text -j .data -O ihex ${OUT}.elf ${OUT}.hex

hex_324: build_324
	avr-objcopy -j .text -j .data -O ihex ${OUT}_324.elf ${OUT}_324.hex

build:
	avr-gcc -std=gnu99 -Os ${INCLUDE} -D DEBUG -D F_CPU=${F_CPU} -g -mmcu=${MCU}    ${SRC} -o ${OUT}.elf

build_324:
	avr-gcc -std=gnu99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=8000000 -g -mmcu=atmega324p ${SRC} -o ${OUT}_324.elf

clear:
	rm build/*
