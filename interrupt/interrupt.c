#include <stdint.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <util/twi.h>
#include "pfleury/i2cmaster.h"
#include <avr/interrupt.h>

volatile uint8_t prev_pin_state;
volatile uint8_t usart_flag;

#define FCPU 160000000UL
#define BAUD 9600
#define MYUBRR (FCPU/16/BAUD-1)

//BMP180 Defines and Variables
#define BMP180_ADDR 0b11101110
#define BMP180_WRITE (BMP180_ADDR|TW_WRITE)
#define BMP180_READ (BMP180_ADDR|TW_READ)
#define BMP180_AC1_MSB 0xAA
#define BMP180_AC1_LSB 0xAB
#define BMP180_AC2_MSB 0xAC
#define BMP180_AC2_LSB 0xAD
#define BMP180_AC3_MSB 0xAE
#define BMP180_AC3_LSB 0xAF
#define BMP180_AC4_MSB 0xB0
#define BMP180_AC4_LSB 0xB1
#define BMP180_AC5_MSB 0xB2
#define BMP180_AC5_LSB 0xB3
#define BMP180_AC6_MSB 0xB4
#define BMP180_AC6_LSB 0xB5
#define BMP180_B1_MSB 0xB6
#define BMP180_B1_LSB 0xB7
#define BMP180_B2_MSB 0xB8
#define BMP180_B2_LSB 0xB9
#define BMP180_MB_MSB 0xBA
#define BMP180_MB_LSB 0xBB
#define BMP180_MC_MSB 0xBC
#define BMP180_MC_LSB 0xBD
#define BMP180_MD_MSB 0xBE
#define BMP180_MD_LSB 0xBF
const unsigned char OSS = 0b11;
short AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned short AC4, AC5, AC6;
long B5;


// Status LED Defines
#define STATUS_LED_PORT PORTC
#define STATUS_LED_PIN PORTC3

// setup printf to write to the serial port via usart/uart
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void i2c_error() {
	printf( "Unspecified I2C Error. Going into while(1) loop. ");
	while(1);
}

int uart_putchar(char c, FILE *stream) {
	if (c == '\n') uart_putchar('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;

}

//interrupt handler routine
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



uint8_t getDataFromBMP180Register(uint8_t loc)
{
	uint8_t data;
 
	i2c_start_wait(BMP180_WRITE);
	i2c_write(loc);
	i2c_rep_start(BMP180_READ);
	data = i2c_readNak();
	i2c_stop();
 
	return data;
}
 
void calibrate_bmp180(void)
{
	AC1 = (getDataFromBMP180Register(BMP180_AC1_MSB)<<8) + getDataFromBMP180Register(BMP180_AC1_LSB);
	AC2 = (getDataFromBMP180Register(BMP180_AC2_MSB)<<8) + getDataFromBMP180Register(BMP180_AC2_LSB);
	AC3 = (getDataFromBMP180Register(BMP180_AC3_MSB)<<8) + getDataFromBMP180Register(BMP180_AC3_LSB);
	AC4 = (getDataFromBMP180Register(BMP180_AC4_MSB)<<8) + getDataFromBMP180Register(BMP180_AC4_LSB);
	AC5 = (getDataFromBMP180Register(BMP180_AC5_MSB)<<8) + getDataFromBMP180Register(BMP180_AC5_LSB);
	AC6 = (getDataFromBMP180Register(BMP180_AC6_MSB)<<8) + getDataFromBMP180Register(BMP180_AC6_LSB);
	B1 = (getDataFromBMP180Register(BMP180_B1_MSB)<<8) + getDataFromBMP180Register(BMP180_B1_LSB);
	B2 = (getDataFromBMP180Register(BMP180_B2_MSB)<<8) + getDataFromBMP180Register(BMP180_B2_LSB);
	MB = (getDataFromBMP180Register(BMP180_MB_MSB)<<8) + getDataFromBMP180Register(BMP180_MB_LSB);
	MC = (getDataFromBMP180Register(BMP180_MC_MSB)<<8) + getDataFromBMP180Register(BMP180_MC_LSB);
	MD = (getDataFromBMP180Register(BMP180_MD_MSB)<<8) + getDataFromBMP180Register(BMP180_MD_LSB);
}
 
long getUcTemp(void)
{
	long UT;
 
	i2c_start_wait(BMP180_WRITE);
	if (i2c_write(0xF4)) i2c_error();
	if (i2c_write(0x2E)) i2c_error();
	i2c_stop();
	_delay_ms(5);
	UT = ((getDataFromBMP180Register(0xF6))<<8) + (getDataFromBMP180Register(0xF7));
 
	return UT;
}
 
short getTrueTemp(long UT)
{
	long X1, X2, T; // B5 declared globally
 
	//X1 = (((long)UT-(long)AC6)*(long)AC5)>>15;
	X2 = ((long)MC << 11)/(X1 + MD);
	B5 = X1 + X2;
	T = (B5+8)>>4;
	return T;
}
 
unsigned long getUcPressure(void)
{
	unsigned long UP;
	unsigned char MSB, LSB, XLSB;
 
	i2c_start_wait(BMP180_WRITE);
	if (i2c_write(0xF4)) i2c_error();
	if (i2c_write(0x34+(OSS<<6))) i2c_error();
	i2c_stop();
	_delay_ms(28);
	MSB = getDataFromBMP180Register(0xF6);
	LSB = getDataFromBMP180Register(0xF7);
	XLSB = getDataFromBMP180Register(0xF8);
	UP = (((unsigned long)MSB<<16) | ((unsigned long)LSB<<8) | ((unsigned long)XLSB)) >> (8-OSS);
 
	return UP;
}
 
long getTruePressure(unsigned long UP)
{
	long X1, X2, X3, B3, B6, p; // B5 declared globally.   Must run getTemp() just before getPressure().
	unsigned long B4, B7;
 
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6)>>12)>>11;
	X2 = (AC2 * B6)>>11;
	X3 = X1 + X2;
	B3 = (((((long)AC1)*4 + X3)<<OSS)+2)>>2;
	X1 = (AC3 * B6)>>13;
	X2 = (B1 * ((B6*B6)>>12))>>16;
	X3 = ((X1+X2)+2)>>2;
	//B4 = (AC4 * (unsigned long)(X3 + 32768))>>15;
	//B7 = ((unsigned long)UP-B3)*(50000>>OSS);
	if (B7<0x80000000UL) p = (B7<<1)/B4;
	else p = (B7/B4)<<1;
	X1 = (p>>8) * (p>>8);
	X1 = (X1*3038)>>16;
	X2 = (-7357*p)>>16;
	p += (X1 + X2 + 3791)>>4;
	return p;
}
 
long getPressure(void) {
	return getTruePressure(getUcPressure());
}
 
short getTemp(void) {
	return getTrueTemp(getUcTemp());
}
 
long getAltitude(void) {
	static long SLP = 101020; // Pa (get from local airport altimeter setting)
	long altitude;
	altitude = (float)44330 * (1-pow((float)getPressure()/SLP, (1/5.255)));
	return (long)altitude;
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

void uart_init() {
  //set baud rate
  UBRR0 = MYUBRR;
  //enable receive and transmit of the uart
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  //change where stdout goes to our one
	stdout = &mystdout;
}

void uart_trasmit(unsigned char data) {
  while ( !( UCSR0A & 32));
  UDR0 = data;
}

int main() {
  setup();
  uart_init();
  sei();

  while (1) {
  cli();
    printf("fuck you");
  }
}
