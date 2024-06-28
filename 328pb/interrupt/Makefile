build:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I. -c -o interrupt.o interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I. -c -o twimaster.o twimaster.c
	avr-gcc -o interrupt.bin interrupt.o twimaster.o -mmcu=atmega328pb
	avr-objcopy -O ihex -R .eeprom interrupt.bin interrupt.hex

flash:
	sudo avrdude -F -c xplainedmini -p m328pb -b 115200 -U flash:w:interrupt.hex

show:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I. -S -o interrupt.s interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I. -c -o interrupt.o interrupt.c
	avr-objdump -d interrupt.o > interrupt.asm
