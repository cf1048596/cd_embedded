#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>


volatile uint8_t prev_pin_state;
volatile uint8_t usart_flag;

#define FOSC 1843200
#define BAUD 9600
#define MYUBBR FOSC/16/BAUD-1

ISR(PCINT0_vect) {
    uint8_t current_pin_state = PINB; // Read the current state of PORTB

    // Check if PCINT7 (PB7) triggered the interrupt
    if ((current_pin_state & 128) != (prev_pin_state & 128)) {
        PORTB ^= 32;
        usart_flag ^= 1;
    }

    // Update the previous pin state
    prev_pin_state = current_pin_state;
}

void setup() {
    // Set PB5 as output
    DDRB |= 128;

    // Set PB7 as input with internal pull-up resistor
    PORTB |= 128;

    // Save the initial state of PORTB
    prev_pin_state = PINB;

    // Enable pin change interrupt on PCINT7 (PB7)
    PCMSK0 |= 128;

    // Enable pin change interrupts for PCINT0 vector
    PCICR |= 1;


}

void usart_init(uint32_t ubrr) {
  //set baud rate
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  //enable receiver & transmitter
  UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
  //set frame format: 8 bit, 2 stop bits
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
  return;
}



void usart_transmit(unsigned char data) {
  while ( !( UCSR0A & 32));
  UDR0 = data;
}

int main() {
    setup();
    usart_init(MYUBBR);
    sei();

    while (1) {
      if (usart_flag == 1) {
        usart_transmit();

      }
    }
}
