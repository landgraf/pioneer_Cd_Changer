OPTFLAGS ?= -std=c99 -W -Wall -pedantic -Wstrict-prototypes -Wundef \
-funsigned-char -funsigned-bitfields -ffunction-sections -fpack-struct -fshort-enums \
-ffreestanding -Os -g -gdwarf-2 -flto \
-fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop \
-Wl,--relax,--gc-sections

MCU = atmega16
SOURCES = main.c mbus.c changer.c usart.c

all: build flash

build:
	avr-gcc ${OPTFLAGS} -mmcu=${MCU} -o emul.o ${SOURCES}
	avr-objcopy -O ihex emul.o emul.hex

flash: build
	sudo avrdude -p m16 -c avr109 -P /dev/ttyUSB0 -U flash:w:emul.hex

clean:
	rm -f emul.o emul.hex
