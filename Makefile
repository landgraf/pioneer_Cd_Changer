
all: build flash


build:
	avr-gcc -O2 -std=c99 -mmcu=atmega16 -o emul.o emul.c
	avr-objcopy -O ihex emul.o emul.hex

flash: build
	sudo avrdude -p m16 -c avr109 -P /dev/ttyUSB0 -U flash:w:emul.hex




