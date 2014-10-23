
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

#define F_CPU 16000000          // Clock, oscillator frequency
#define I2C_BAUDRATE 400000     // Max. 400kHz for 2.5V < Vcc < 5.5V

#define BDIV (F_CPU / I2C_BAUDRATE - 16) / 2 + 1




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

uint8_t gsData[16 * TLC5940_N] = {
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

/*
uint8_t gsData2[16 * TLC5940_N] = {
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
}; */

/*
 * Function: setup
 * -------------------
 * Setup the hardware. Declare required pins as inputs/outputs, initialize 
 * TLC5940 LED drivers, initialise timers and enable UART transmission/reception.
 *
 */
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
	
	DDRC |= (1 << PINC3);				// Declare test LED as output
	
	UBRR0 = 51;							// Set baudrate to 19200
	
	/* Enable UART transmission and reception */
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);
	UCSR0A = (0 << U2X0);				// Enable Receive Complete Interrupt
	
	//int8_t eepromAddress = 0xA0;     // I2C device address for 24LC256

	//TWSR = 0;       // Set prescale for 1
	//TWBR = BDIV;    // Set bit rate register
	
	PORTC |= (1 << PORTC3);
}


/*
 * Function: get_DC
 * -------------------
 * Get dot correction values upon initialisation and store in array. 
 * 
 */
void get_DC(void) {
	DCPRG_PORT |= (1 << DCPRG_PIN);		// Set dot correction pin high
	VPRG_PORT |= (1 << VPRG_PIN);		// Set mode selection pin high
	
	for (dcData_t i = 0; i < dcDataSize; i++) {
		SPDR = dcData[i];				// Start transmission
		while (!(SPSR & (1 << SPIF)));	// Wait for transmission complete
	}
	
	XLAT_PORT |= (1 << XLAT_PIN);
	XLAT_PORT &= ~(1 << XLAT_PIN);
}


/*
 * Function: init_Mode2
 * -------------------
 * Following Test Mode 1, initialises Test Mode 2. Outermost LED grayscale 
 * value is set to maximum brightness and all other LEDs are turned off.
 *
 */
int init_Mode2(void) {
    for (gsData_t i = 0; i < gsDataSize; i++) {
        gsData[i] = 0b11111111;		// Set all grayscale values to max. brightness
    }
    
    // Turn off rotation sensing
    // Set display speed threshold to 0
    
    return 0;
}

/*
 * Function: initMode3
 * -------------------
 * Following Test Mode 2, initialise Test Mode 3 by setting outermost LED grayscale 
 * value to maximum brightness and all other LEDs off.
 *
 */
int init_Mode3(void) {
    for (gsData_t i = 0; i < gsDataSize; i++) {
        gsData[i] = 0b00000000;		// Set all grayscale values to 0
    }
    gsData[15] = 0b11111111;        // Set outermost LED to max. brightness
    
    return 0;
}


// global variable to count the number of overflows
volatile uint8_t tot_overflow;

// initialize timer, interrupt and variable
void init_Timer2() {
	TCCR2A |= (1 << CS22)|(1 << CS21);	// Set prescaler to 256
	TCNT2 = 0;							// Initialize counter
	TIMSK2 |= (1 << TOIE2);				// Enable overflow interrupt
	
	overflow2 = 0;						// Set overflow counter
}

// TIMER0 overflow interrupt service routine
// called whenever TCNT0 overflows
ISR(TIMER2_OVF_vect) {
	overflow2++;		// Increment overflow count
}


int main(void) {
	setup();	// Initialize hardware
	get_DC();	// Clock in dot correction data
	sei();		// Enable interrupts

	while(1);	// Infinite loop, waiting for interrupts
	
	return 0;
} 

/*
int main(void) {
    int modeNumber = 3;     // Test Mode: 1-3 for each respective mode, 0 is normal operation
    int modeSwitch = 2;     // Boolean switch state: 1 is on, 0 is off
	
	setup();			// Initialize hardware
	get_DC();			// Clock in dot correction data	
	sei();				// Enable interrupts
    
    // Setup
    if (modeSwitch == 0) {      // If mode switch off, normal mode
        modeNumber = 0;
    } else if (modeSwitch == 1) {
        //initMode1();
        modeNumber = 1;         // If mode switch on, enter test mode 1
    }
    
    // Normal Mode
    while (modeNumber == 0) {
        printf("Normal Mode");
    }
    
    // Test Mode 1: Normal operation at fixed rotation speed of 200rpm.
    // Rotation sensing ignored and no standby mode.
    while (modeNumber == 1) {
        printf("Test Mode 1");
        if (modeSwitch == 0) {      // If mode switch off, enter test mode 2
			init_Mode2();
            modeNumber = 2;
        }
    }

    // Test Mode 2: All LEDs at maximum brightness, rotation sensing ignored
    // and no standby mode. 
    while (modeNumber == 2) {
        printf("Test Mode 2");
        if (modeSwitch == 1) {
            init_Mode3();
            modeNumber = 3;
        }
    }
    
    // Test Mode 3: Outermost LED at maximum brightness, all other LEDs off.
    // Rotation sensing ignored and no standby mode.
    while (modeNumber == 3) {
        printf("Test Mode 3");
    }
    
    return 0;
} */


/*
 * Interrupt handler: TIMER0_COMPA_vect
 * ------------------------------------
 * Shifts new grayscale values through LED array for display.
 * 
 */
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
    
    for (gsData_t i = 0; i < gsDataSize; i++) {
	    if (i%2 == 0) {
			SPDR = gsData[i];				// Start transmission
			while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
		
		} else {
			int8_t buffer;
		    
			buffer = (gsData[i] >> 4);		
			SPDR = buffer;					// Start transmission
			while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
		    
			buffer = (gsData[i] << 4);		
		    SPDR = buffer;					// Start transmission
		    while (!(SPSR & (1 << SPIF)));	// Wait for transmission to complete
	    }
    } 
	
	latchNeedsPulse = 1;
}


/* 
 * Interrupt handler: USART_RX_vect
 * --------------------------------
 * Interrupt handler for UART Receive Complete - i.e. a new byte has arrived in
 * the UART Data Register (UDR).
 *
 */
ISR(USART_RX_vect) {
	int8_t input;
	input = UDR0;	// Extract character from UART Data Register
	
	PORTC ^= (1 << PORTC3);		// Toggle the test LED
	
	if (ledRef < 32) {
		gsData[ledRef] = input;
		ledRef++;
	}
	
	UDR0 = input;	// Send character back over serial communication
}
