#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main() {
  DDRB |=32;
  while(1){
    //turn pin ON
    PORTB |=32;
  }
}
