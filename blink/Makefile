build:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -c -o blink.o blink.c
	avr-gcc -o blink.bin blink.o
	avr-objcopy -O ihex -R .eeprom blink.bin blink.hex
	sudo avrdude -F -V -c xplainedmini -p m328pb -b 115200 -U flash:w:blink.hex

show:  
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -S -o blink.s blink.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -c -o blink.o blink.c
	avr-objdump -d blink.o > blink.asm
