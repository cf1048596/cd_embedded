# Variables
MCU = atmega328p
F_CPU = 16000000UL
SRC = interrupt.c twimaster.c
OBJ = interrupt.o twimaster.o
BIN = interrupt.bin
HEX = interrupt.hex
INCLUDE = -I.

# Compiler and flags
CC = avr-gcc
CFLAGS = -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU) $(INCLUDE)
LDFLAGS = -mmcu=$(MCU)
OBJCOPY = avr-objcopy

# Build target: compile and link
build:
	$(CC) $(CFLAGS) -c -o interrupt.o interrupt.c
	$(CC) $(CFLAGS) -c -o twimaster.o twimaster.c
	$(CC) -o $(BIN) $(OBJ) $(LDFLAGS)
	$(OBJCOPY) -O ihex -R .eeprom $(BIN) $(HEX)

# Flash target: upload the compiled hex file to the microcontroller
flash:
	sudo avrdude -F -c arduino -P /dev/ttyACM0 -p m328p -b 115200 -U flash:w:$(HEX)

# Show target: generate assembly and disassembly
show:
	$(CC) $(CFLAGS) -S -o interrupt.s interrupt.c
	avr-objdump -d interrupt.o > interrupt.asm

# Clean target: remove generated files
clean:
	rm -f $(OBJ) $(BIN) $(HEX) interrupt.s interrupt.asm

.PHONY: build flash show clean
