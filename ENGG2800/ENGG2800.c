
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define F_CPU 16000000
#define USART_BAUDRATE 19200
#define UBRR_VALUE ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)

#define GSCLK_DDR DDRB
#define GSCLK_PORT PORTB
#define GSCLK_PIN PB0

#define SIN_DDR DDRB
#define SIN_PORT PORTB
#define SIN_PIN PB3

#define SCLK_DDR DDRB
#define SCLK_PORT PORTB
#define SCLK_PIN PB5

#define BLANK_DDR DDRB
#define BLANK_PORT PORTB
#define BLANK_PIN PB2

#define DCPRG_DDR DDRD
#define DCPRG_PORT PORTD
#define DCPRG_PIN PD4

#define VPRG_DDR DDRD
#define VPRG_PORT PORTD
#define VPRG_PIN PD7

#define XLAT_DDR DDRB
#define XLAT_PORT PORTB
#define XLAT_PIN PB1

#define TLC5940_N 2

#define dcData_t uint8_t
#define gsData_t uint8_t

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)16 * TLC5940_N)

volatile unsigned int ledRef = 0;

uint8_t dcData[12 * TLC5940_N] = {
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
};

uint8_t gsData1[16 * TLC5940_N] = {
// MSB		LSB
	0b11111111,		// Channel 17
	0b10000000,		// Channel 18
	0b01000000,		// Channel 19
	0b00100000,		// Channel 20
	0b00010000,		// Channel 21
	0b00001000,		// Channel 22
	0b00000100,		// Channel 23
	0b00000010,		// Channel 24
	0b00000001,		// Channel 25
	0b00000000,		// Channel 26
	0b00000000,		// Channel 27
	0b00000000,		// Channel 28
	0b00000000,		// Channel 29
	0b00000000,		// Channel 30
	0b00000000,		// Channel 31
	0b00000000,		// Channel 32
// MSB		LSB	
	0b00000000,		// Channel 1
	0b00000000,		// Channel 2
	0b00000000,		// Channel 3
	0b00000000,		// Channel 4
	0b00000000,		// Channel 5
	0b00000000,		// Channel 6
	0b00000000,		// Channel 7
	0b00000001,		// Channel 8
	0b00000010,		// Channel 9
	0b00000100,		// Channel 10
	0b00001000,		// Channel 11
	0b00010000,		// Channel 12
	0b00100000,		// Channel 13
	0b01000000,		// Channel 14
	0b10000000,		// Channel 15
	0b11111111,		// Channel 16
};

volatile uint8_t gsData2[16 * TLC5940_N] = {
// MSB		LSB	
	0b00000000,		
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,

	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
};

void setup(void) {
	/* Declare pins as outputs */
	GSCLK_DDR |= (1 << GSCLK_PIN);		// Grayscale PWM reference clock
	SCLK_DDR |= (1 << SCLK_PIN);		// Serial data shift clock
	DCPRG_DDR |= (1 << DCPRG_PIN);		// Dot correction programming
	VPRG_DDR |= (1 << VPRG_PIN);		// Mode selection
	XLAT_DDR |= (1 << XLAT_PIN);		// Latch signal
	BLANK_DDR |= (1 << BLANK_PIN);		// Blank all outputs 
	SIN_DDR |= (1 << SIN_PIN);			// Serial data output > input
	
	/* TLC5940 initialization routine */
	GSCLK_PORT &= ~(1 << GSCLK_PIN);	// Set grayscale clock pin low
	SCLK_PORT &= ~(1 << SCLK_PIN);		// Set serial data shift clock pin low
	DCPRG_PORT &= ~(1 << DCPRG_PIN);	// Set dot correction programming pin low
	VPRG_PORT |= (1 << VPRG_PIN);		// Set mode selection pin high
	XLAT_PORT &= ~(1 << XLAT_PIN);		// Set latch signal pin low
	BLANK_PORT |= (1 << BLANK_PIN);		// Set blank output pin high

	/* Initialize timer */
	SPCR = (1 << SPE)|(1 << MSTR);		// Enable SPI and Master
	SPSR = (1 << SPI2X);				// Set clock rate fck/2
	TCCR0A = (1 << WGM01);				// Set timer mode to CTC
	TCCR0B = (1 << CS02)|(1 << CS00);	// Set prescaler to 1024 and start timer

	// OCRn = [ (Clock speed / Prescale value) * (Desired time in seconds) ] - 1
	OCR0A = 3;							// Interrupt every 4096 clock cycles
	TIMSK0 |= (1 << OCIE0A);			// Enable Timer0 Match A interrupt
	
	DDRC |= (1 << PINC4);				// Declare test LED as output
	
	UBRR0 = 51;							// Set baudrate to 19200
	
	/* Enable UART transmission and reception */
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);
	UCSR0A = (0 << U2X0);				// Enable Receive Complete Interrupt
}

/* Get dot correction values upon initialization */
void getDC(void) {
	DCPRG_PORT |= (1 << DCPRG_PIN);		// Set dot correction pin high
	VPRG_PORT |= (1 << VPRG_PIN);		// Set mode selection pin high
	
	for (dcData_t i = 0; i < dcDataSize; i++) {
		SPDR = dcData[i];				// Start transmission
		while (!(SPSR & (1 << SPIF)));	// Wait for transmission complete
	}
	
	XLAT_PORT |= (1 << XLAT_PIN);
	XLAT_PORT &= ~(1 << XLAT_PIN);
}

ISR(TIMER0_COMPA_vect) {
	static uint8_t latchNeedsPulse = 0;
	
	BLANK_PORT |= (1 << BLANK_PIN);			// Set blank output pin high
	
	if (VPRG_PORT & (1 << VPRG_PIN)) {
		VPRG_PORT &= ~(1 << VPRG_PIN);		// Set mode selection pin low
		
		if (latchNeedsPulse) {
			XLAT_PORT |= (1 << XLAT_PIN);	// Pulse latch signal pin
			XLAT_PORT &= ~(1 << XLAT_PIN);
			latchNeedsPulse = 0;
		}
		
		SCLK_PORT |= (1 << SCLK_PIN);		// Pulse serial data clock pin
		SCLK_PORT &= ~(1 << SCLK_PIN);
		
	} else if (latchNeedsPulse) {
		XLAT_PORT |= (1 << XLAT_PIN);		// Pulse latch signal pin
		XLAT_PORT &= ~(1 << XLAT_PIN);
		latchNeedsPulse = 0;
	}
	
	BLANK_PORT &= ~(1 << BLANK_PIN);		// Set blank output pin low
	
	// Below this we have 4096 cycles to shift in the data for the next cycle
	
	/*
	for (gsData_t i = 0; i < gsDataSize; i++) {
		SPDR = gsData[i];
		while (!(SPSR & (1 << SPIF)));
	} */
    
    for (gsData_t i = 0; i < gsDataSize; i++) {
	    if (i%2 == 0) {
			SPDR = gsData2[i];				// Start transmission
			while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
		
		} else {
			int8_t buffer;
		    
			buffer = (gsData2[i] >> 4);		
			SPDR = buffer;					// Start transmission
			while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
		    
			buffer = (gsData2[i] << 4);		
		    SPDR = buffer;					// Start transmission
		    while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
	    }
    } 
	
	latchNeedsPulse = 1;
}

/* Interrupt handler for UART Receive Complete - i.e. a new byte has arrived in
 * the UART Data Register (UDR).
 */
ISR(USART_RX_vect) {
	int8_t input;
	input = UDR0;	// Extract character from UART Data Register
	
	PORTC ^= (1 << PORTC4);		// Toggle the test LED
	
	if (ledRef < 32) {
		gsData2[ledRef] = input;
		ledRef++;
	}
	
	
	UDR0 = input;	// Send character back over serial communication
}

int main(void) {
	setup();	// Initialize hardware
	getDC();	// Clock in dot correction data
	
	sei();		// Enable interrupts

	while(1);	// Infinite loop, waiting for interrupts
	
	return 0;
}