build:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -I./pfleury -c -o interrupt.o interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -I./pfleury -c -o pfleury/twimaster.o pfleury/twimaster.c
	avr-gcc -o interrupt.bin interrupt.o pfleury/twimaster.o -mmcu=atmega328p
	avr-objcopy -O ihex -R .eeprom interrupt.bin interrupt.hex

flash:
	sudo avrdude -F -c arduino -p m328p -P /dev/ttyACM0 -b 115200 -U flash:w:interrupt.hex

show:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -I./pfleury -S -o interrupt.s interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -I./pfleury -c -o interrupt.o interrupt.c
	avr-objdump -d interrupt.o > interrupt.asm
