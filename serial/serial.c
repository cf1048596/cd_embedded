#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/twi.h>

volatile uint8_t prev_pin_state;
volatile uint8_t usart_flag;

#define FCPU 16000000UL
#define BAUD 9600
#define MYUBRR (FCPU/16/BAUD-1)

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

int uart_putchar(char c, FILE *stream) {
	if (c == '\n') uart_putchar('\r', stream);
  while ( !( UCSR0A & (1<<UDRE0)) )
	UDR0 = c; _delay_us(100);
	return 0;
}

void uart_init() {
  //set baud rate
  UBRR0H = (unsigned char)(MYUBRR>>8);
  UBRR0L = (unsigned char)(MYUBRR);
  //enable receive and transmit of the uart
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  //8data, 2stop bit
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
  //change where stdout goes to our one
	//stdout = &mystdout;
}

int main() {
  uart_init();
  while (1) {
    UDR0 = 'a';
    _delay_ms(250);
  }
}
