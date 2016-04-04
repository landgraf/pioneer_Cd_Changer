OPTFLAGS ?= -std=gnu99 -W -Wall -pedantic -Wstrict-prototypes -Wundef \
-funsigned-char -funsigned-bitfields -ffunction-sections -fpack-struct -fshort-enums \
-ffreestanding -Os -g -gdwarf-2 \
-fwhole-program \
-fno-inline-small-functions -fno-split-wide-types -fno-tree-scev-cprop \
-Wl,--relax,--gc-sections
all: build flash


build:
	avr-gcc ${OPTFLAGS}  -mmcu=atmega16  -o emul.o emul.c
	avr-objcopy -O ihex emul.o emul.hex

flash: build
	sudo avrdude -p m16 -c avr109 -P /dev/ttyUSB0 -U flash:w:emul.hex




