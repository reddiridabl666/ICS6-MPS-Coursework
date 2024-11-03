F_CPU = 8000000
# MCU = atmega8515
MCU = atmega32
OUT = build/kursach
SRC = lib/enc28j60.c main.c
INCLUDE = -I lib
DEBUG ?= true

.PHONY: build hex build_8515

hex: build build_8515
	avr-objcopy -j .text -j .data -O ihex ${OUT}.elf ${OUT}.hex
	avr-objcopy -j .text -j .data -O ihex ${OUT}_8515.elf ${OUT}_8515.hex

build:
	avr-gcc -std=c99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=${F_CPU} -g -mmcu=${MCU} ${SRC} -o ${OUT}.elf

build_8515:
	avr-gcc -std=c99 -Os ${INCLUDE} -D DEBUG=${DEBUG} -D F_CPU=${F_CPU} -g -mmcu=atmega8515 ${SRC} -o ${OUT}_8515.elf
