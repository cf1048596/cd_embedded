#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main() {
  DDRB = DDRB | (1 << DDB5);
  while(1){
    //turn pin ON
    PORTB = PORTB | (1 << PORTB5);
  }
}
