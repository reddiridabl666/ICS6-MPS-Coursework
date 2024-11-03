F_CPU = 8000000
# MCU = atmega8515
MCU = atmega32
OUT = build/kursach
SRC = lib/enc28j60.c main.c
INCLUDE = -I lib
DEBUG ?= true

.PHONY: build hex hex_64 build_64

all: hex hex_64

hex: build
	avr-objcopy -j .text -j .data -O ihex ${OUT}.elf ${OUT}.hex

hex_64: build_64
	avr-objcopy -j .text -j .data -O ihex ${OUT}_64.elf ${OUT}_64.hex

build:
	avr-gcc -std=c99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=${F_CPU} -g -mmcu=${MCU}    ${SRC} -o ${OUT}.elf

build_64:
	avr-gcc -std=c99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=12000000 -g -mmcu=atmega644 ${SRC} -o ${OUT}_64.elf

clear:
	rm build/*
