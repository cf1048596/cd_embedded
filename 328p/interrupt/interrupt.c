#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <math.h>
#include <stdio.h>
#include "i2cmaster.h"

volatile uint8_t prev_pin_state;
volatile uint8_t usart_flag = 0;
volatile uint32_t press_count = 0;

#define FCPU 16000000UL
#define BAUD 9600
#define MYUBRR (FCPU/16/BAUD-1)

//BMP180 Defines and Variables
#define BMP180_ADDR 0xEE
#define BMP180_WRITE (BMP180_ADDR|0)
#define BMP180_READ (BMP180_ADDR|1)
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


// setup printf to write to the serial port via usart/uart
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void i2c_error() {
  printf( "Unspecified I2C Error. Going into while(1) loop.");
	while(1);
}

int uart_putchar(char c, FILE *stream) {
	if (c == '\n') uart_putchar('\r', stream);
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

uint8_t getDataFromBMP180Register(uint8_t dev_register) {
	uint8_t data;
	i2c_start_wait(0xEE);
	i2c_write(dev_register);
	i2c_rep_start(0xEF);
	data = i2c_readNak();
	i2c_stop();
	return data;
}
 
void calibrate_bmp180() {
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
 
long getUcTemp(void) {
	long UT;
	i2c_start_wait(BMP180_WRITE);
	if (i2c_write(0xF4)) i2c_error();
	if (i2c_write(0x2E)) i2c_error();
	i2c_stop();
	_delay_ms(5);
	UT = ((getDataFromBMP180Register(0xF6))<<8) + (getDataFromBMP180Register(0xF7));
	return UT;
}
 

short getTrueTemp(long UT) {
  long long X1, X2, T; // B5 declared globally
  // Compute X1
  X1 = ((long long)(UT - AC6) * (long long)AC5) >> 15;
  // Compute X2
  B5 = (short)(X1 + X2);
  T = (B5 + 8) >> 4;
  return (short)T;
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
 
long getTruePressure(int32_t UP) {
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
	B4 = (AC4 * (unsigned long)(X3 + 32768))>>15;
	B7 = ((unsigned long)UP-B3)*(50000>>OSS);
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
	static long SLP = 101020; // Pa (get from dev_registeral airport altimeter setting)
	long altitude;
	altitude = (float)44330 * (1-pow((float)getPressure()/SLP, (1/5.255)));
	return (long)altitude;
}

/*
void setup() {
  // Set PB5 as output
  DDRB |= 128;
  DDRC |= 16;

  // Set PB7 as input with internal pull-up resistor
  PORTB |= 128;

  // Save the initial state of PORTB
  prev_pin_state = PINB;

  // Enable pin change interrupt on PCINT7 (PB7)
  PCMSK0 |= 128;
  // Enable pin change interrupts for PCINT0 vector
  PCICR |= 1;
  PORTC |= 16;
}
*/

void uart_init() {
  //set baud rate to 9600
  UBRR0H = (unsigned char)(MYUBRR>>8);
  UBRR0L = (unsigned char)(MYUBRR);
  //enable receive and transmit of the uart
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  //8data, 2stop bit
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
  //change where stdout goes to our/init printf
	stdout = &mystdout;
}

int main() {
  uart_init();
  printf("please work\n");
  int num = getDataFromBMP180Register(0xD0);
  printf("chip id: %d\n", num);
  while (1) {
  }
}
