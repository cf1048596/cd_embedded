#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>


volatile uint8_t prev_pin_state;

ISR(PCINT0_vect) {
    uint8_t current_pin_state = PINB; // Read the current state of PORTB

    // Check if PCINT7 (PB7) triggered the interrupt
    if ((current_pin_state & (1 << PINB7)) != (prev_pin_state & (1 << PINB7))) {
        // PCINT7 triggered the interrupt, handle it here
        // For example, toggle an LED on PB5
        PORTB ^= (1 << PORTB5);
    }

    // Update the previous pin state
    prev_pin_state = current_pin_state;
}

void setup() {
    // Set PB5 as output
    DDRB |= (1 << DDB5);

    // Set PB7 as input with internal pull-up resistor
    DDRB &= ~(1 << DDB7);
    PORTB |= (1 << PORTB7);

    // Save the initial state of PORTB
    prev_pin_state = PINB;

    // Enable pin change interrupt on PCINT7 (PB7)
    PCMSK0 |= (1 << PCINT7);

    // Enable pin change interrupts for PCINT0 vector
    PCICR |= (1 << PCIE0);

    // Enable global interrupts
    sei();
}

int main() {
    setup();

    while (1) {
        // Main loop does nothing, all work is done in ISR
    }
}
