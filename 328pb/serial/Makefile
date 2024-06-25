build:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -c -o serial.o serial.c
	avr-gcc -o serial.bin serial.o -mmcu=atmega328pb
	avr-objcopy -O ihex -R .eeprom serial.bin serial.hex

build2:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o serial.o serial.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o pfleury/twimaster.o pfleury/twimaster.c 
	avr-gcc -o serial.elf serial.o pfleury/twimaster.o -L/usr/lib/gcc/avr/5.4.0/avr5 -lgcc -mmcu=atmega328pb

flash:
	sudo avrdude -F -c xplainedmini -p m328pb -b 115200 -U flash:w:serial.hex

show:
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -S -o serial.s serial.c
	avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328pb -I./pfleury -c -o serial.o serial.c
	avr-objdump -d serial.o > serial.asm
