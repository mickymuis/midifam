CC = avr-gcc
CFLAGS = -mmcu=attiny2313 -Wall -gdwarf-2 -Os -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
AVRDUDE = avrdude
PLATFORM = t2313
PROGRAMMER = dragon_isp

all: firmware.hex

firmware.hex: main.c
	${CC} ${CFLAGS} main.c -o firmware.hex

flash: firmware.hex
	${AVRDUDE} -p ${PLATFORM} -c ${PROGRAMMER} -U flash:w:firmware.hex
