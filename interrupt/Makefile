
build:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o interrupt.o interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o pfleury/twimaster.o pfleury/twimaster.c
	avr-gcc -o interrupt.elf interrupt.o pfleury/twimaster.o -lgcc -lm -lc
	avr-objcopy -O ihex -R .eeprom interrupt.elf interrupt.hex

flash:
	sudo avrdude -F -c xplainedmini -p m328pb -b 115200 -U flash:w:interrupt.hex

show:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -S -o interrupt.s interrupt.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o interrupt.o interrupt.c
	avr-objdump -d interrupt.o > interrupt.asm
