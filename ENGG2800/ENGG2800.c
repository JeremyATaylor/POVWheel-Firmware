
/*
 * NOTE: When programming the device, ensure fuses are set as follows:
 * -----
 * Extended Register: 0xFF
 * High Register: 0xDE
 * Low Register: 0xBF
 *
 */

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define F_CPU 16000000
#define USART_BAUDRATE 19200
#define UBRR_VALUE ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)

#define I2C_BAUDRATE 100000     // Max. 400kHz for 2.5V < Vcc < 5.5V
#define BDIV (F_CPU / I2C_BAUDRATE - 16) / 2 + 1

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

volatile uint8_t ledRef;
volatile uint8_t dataIndex;
int8_t deviceAddress = 0xA0;    // I2C device address for 24LC256

int oddCycle = 1;

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
	0b00000000,		// Channel 17
	0b00000000,		// Channel 18
	0b00000000,		// Channel 19
	0b00000000,		// Channel 20
	0b00000000,		// Channel 21
	0b00000000,		// Channel 22
	0b00000000,		// Channel 23
	0b00000001,		// Channel 24
	0b00000010,		// Channel 25
	0b00000100,		// Channel 26
	0b00001000,		// Channel 27
	0b00010000,		// Channel 28
	0b00100000,		// Channel 29
	0b01000000,		// Channel 30
	0b10000000,		// Channel 31
	0b11111111,		// Channel 32
// MSB		LSB
	0b11111111,		// Channel 1 
	0b10000000,		// Channel 2
	0b01000000,		// Channel 3
	0b00100000,		// Channel 4
	0b00010000,		// Channel 5
	0b00001000,		// Channel 6
	0b00000100,		// Channel 7
	0b00000010,		// Channel 8
	0b00000001,		// Channel 9
	0b00000000,		// Channel 10
	0b00000000,		// Channel 11
	0b00000000,		// Channel 12
	0b00000000,		// Channel 13
	0b00000000,		// Channel 14
	0b00000000,		// Channel 15
	0b00000000,		// Channel 16
};

uint8_t gsData1[16 * TLC5940_N] = {
	// MSB		LSB
	0b00000000,		// Channel 17
	0b00000000,		// Channel 18
	0b00000000,		// Channel 19
	0b00000000,		// Channel 20
	0b00000000,		// Channel 21
	0b00000000,		// Channel 22
	0b00000000,		// Channel 23
	0b00000001,		// Channel 24
	0b00000010,		// Channel 25
	0b00000100,		// Channel 26
	0b00001000,		// Channel 27
	0b00010000,		// Channel 28
	0b00100000,		// Channel 29
	0b01000000,		// Channel 30
	0b10000000,		// Channel 31
	0b11111111,		// Channel 32
	// MSB		LSB
	0b11111111,		// Channel 1
	0b10000000,		// Channel 2
	0b01000000,		// Channel 3
	0b00100000,		// Channel 4
	0b00010000,		// Channel 5
	0b00001000,		// Channel 6
	0b00000100,		// Channel 7
	0b00000010,		// Channel 8
	0b00000001,		// Channel 9
	0b00000000,		// Channel 10
	0b00000000,		// Channel 11
	0b00000000,		// Channel 12
	0b00000000,		// Channel 13
	0b00000000,		// Channel 14
	0b00000000,		// Channel 15
	0b00000000,		// Channel 16
};

uint8_t gsData2[16 * TLC5940_N] = {
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

void setup(void) {
	/* Declare pins as outputs */
	GSCLK_DDR |= (1 << GSCLK_PIN);		// Grayscale PWM reference clock
	SCLK_DDR |= (1 << SCLK_PIN);		// Serial data shift clock
	DCPRG_DDR |= (1 << DCPRG_PIN);		// Dot correction programming
	VPRG_DDR |= (1 << VPRG_PIN);		// Mode selection
	XLAT_DDR |= (1 << XLAT_PIN);		// Latch signal
	BLANK_DDR |= (1 << BLANK_PIN);		// Blank all outputs 
	SIN_DDR |= (1 << SIN_PIN);			// Serial data output > input
	DDRC |= (1 << PINC3);				// Declare test LED as output
	
	/* TLC5940 initialization routine */
	GSCLK_PORT &= ~(1 << GSCLK_PIN);	// Set grayscale clock pin low
	SCLK_PORT &= ~(1 << SCLK_PIN);		// Set serial data shift clock pin low
	DCPRG_PORT &= ~(1 << DCPRG_PIN);	// Set dot correction programming pin low
	VPRG_PORT |= (1 << VPRG_PIN);		// Set mode selection pin high
	XLAT_PORT &= ~(1 << XLAT_PIN);		// Set latch signal pin low
	BLANK_PORT |= (1 << BLANK_PIN);		// Set blank output pin high

	/* Initialize Timer0 */
	SPCR = (1 << SPE)|(1 << MSTR);		// Enable SPI and Master
	SPSR = (1 << SPI2X);				// Set clock rate fck/2
	TCCR0A = (1 << WGM01);				// Set timer mode to CTC
	TCCR0B = (1 << CS02)|(1 << CS00);	// Set prescaler to 1024 and start timer
	// OCRn = [ (Clock speed / Prescale value) * (Desired time in seconds) ] - 1
	OCR0A = 3;							// Interrupt every 4096 clock cycles
	TIMSK0 |= (1 << OCIE0A);			// Enable Timer0 Match A interrupt
	
	/* Enable UART transmission and reception */
	UBRR0 = 51;							// Set baudrate to 19200
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);
	UCSR0A = (0 << U2X0);				// Enable Receive Complete Interrupt
	
	/* Setup I2C, external EEPROM */
	
	TWSR = 0;                       // Set prescale for 1
	TWBR = BDIV;					// Set bit rate register
	dataIndex = 0;
}

/*
 * Function: getDC
 * ---------------
 * Calculate and store dot correction values in the cdData array upon
 * initialization.
 *
 */
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
	
	// Note: Below this we have 4096 cycles to shift in the data for the next cycle
	int8_t buffer;
	for (gsData_t i = 0; i < gsDataSize; i++) {
		if (i%2 == 0) {
			SPDR = gsData[i];
			while (!(SPSR & (1 << SPIF)));	
		} else {
			buffer = gsData[i] >> 4;
			SPDR = buffer;
			while (!(SPSR & (1 << SPIF)));
			
			buffer = gsData[i] << 4;
			SPDR = (buffer << 4);
			while (!(SPSR & (1 << SPIF)));
		}
	}
	
	latchNeedsPulse = 1;
}

/* 
 * Interrupt handler: UART Receive Complete
 * ----------------------------------------
 * i.e. a new byte has arrived in the UART Data Register (UDR).
 *
 */
ISR(USART_RX_vect) {
	char input;
	input = UDR0;	// Extract character from UART Data Register
    
    /* Setup device for recieving data */
	if (dataIndex == 0) {
        TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTA);   // Send start condition
        while (!(TWCR & (1 << TWINT)));                 // Wait for TWINT to be set

        TWDR = deviceAddress & 0xFE;            // Load device address and R/W = 0
        TWCR = (1 << TWINT) | (1 << TWEN);      // Start transmission
        while (!(TWCR & (1 << TWINT)));         // Wait for TWINT to be set
        
        // Delay 5ms
	}
    
    /* Write 32 data bytes to the slave device */
    TWDR = input;                           // Put next data byte in TWDR
    TWCR = (1 << TWINT) | (1 << TWEN);      // Start transmission
    while (!(TWCR & (1 << TWINT)));         // Wait for TWINT to be set
    dataIndex++;
    
    /* After last byte has been sent... */
	if (dataIndex == 32) {
	         TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWSTO);   // Send stop condition
			 dataIndex = 0;
	}
	PORTC ^= (1<<PORTC3);
}

/*
 * Function: initMode1
 * -------------------
 * Fix rotation speed to 200rpm, disable rotation sensing and standby modes.
 *
 */
int initMode1(void) {

	OCR1A = 15624*0.3;		// Set output compare register	
	// Turn off rotation sensing
	// Set display speed threshold to 0
	return 0;
}

/*
 * Function: initMode2
 * -------------------
 * Following Test Mode 1, initialise Test Mode 2 by setting all LED grayscale
 * values to maximum brightness.
 *
 */
int initMode2(void) {
	for (gsData_t i = 0; i < gsDataSize; i++) {		// Set all grayscale values to max.
		gsData[i] = 0b11111111;
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
int initMode3(void) {
	for (gsData_t i = 0; i < gsDataSize; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b00000000;
	} 
	gsData[31] = 0b11111111;	// Set outermost LED to max. brightness
	
	// Turn off rotation sensing
	// Set display speed threshold to 0
	return 0;
}
/*
int main(void) {
	int modeNumber;		// Test Modes 1-3 for each respective mode, 0 is normal operation
	int modeSwitch;		// Placeholder
	
	// Setup
	if (modeSwitch == 0) {			// If mode switch off, enter Normal Mode
		modeNumber = 0;
	} else if (modeSwitch == 1) {	// If mode switch on, enter Test Mode 1
		initMode1();
		modeNumber = 1;
	}
	
	// Normal Mode
	while (modeNumber == 0) {
		// Normal operation goes here	
	}
	
	// Test Mode 1: Normal operation at fixed rotation speed of 200rpm.
	// Rotation sensing is ignored and no standby mode. 
	while (modeNumber == 1) {
		// Test Mode 1 goes here
		if (modeSwitch == 0) {		// If mode switch off, enter Test Mode 2
			initMode2();
			modeNumber = 2;
		}
	}
	
	// Test Mode 2: All LEDs at maximum brightness, rotation sensing ignored
	// and no standby mode. 
	while (modeNumber == 2) {
		// Test Mode 2 goes here
		if (modeSwitch == 1) {
			initMode3();
			modeNumber = 3;
		}
	}
	
	// Test Mode 3: Outermost LED at maximum brightness, all other LEDs off.
	// Rotation sensing ignored and no standby mode. 
	while (modeNumber == 3) {
		// Test Mode 3 goes here
	}
	 
	return 0; 
}
*/

/*
 * Function: initTimer1
 * --------------------
 * Initialises Timer1, interrupt and global counter variable.
 * Note: For 60Hz, OCR1A = 15624.
 */
void initTimer1(void) {
    OCR1A = 15624;

    TCCR1B |= (1 << WGM12);		// Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A);	// Set interrupt on compare match
    
	// Set prescaler to 1024 and start the timer
    TCCR1B |= (1 << CS12) | (1 << CS10);
}

/*
 * Interrupt handler: TIMER1_COMPA_vect
 * ----------------------------------
 * Timer1 output compare interrupt, set to occur every 1 second.
 *
 */
ISR (TIMER1_COMPA_vect) {
	/*
	PORTC ^= (1 << PORTC3);		// Toggle the test LED
	if (oddCycle) {
		for (gsData_t i = 0; i < gsDataSize; i++) {		// Set all grayscale values to 0
			gsData[i] = gsData1[i];
		}
		oddCycle = !oddCycle;
	} else {
		for (gsData_t i = 0; i < gsDataSize; i++) {		// Set all grayscale values to 0
			gsData[i] = gsData2[i];
		}
		oddCycle = !oddCycle;		
	} 
	
*/
}

int main(void) {
	setup();		// Initialize hardware
	getDC();		// Clock in dot correction data
	initTimer1();	// Initialise Timer1
	initMode2();
	DDRC |= (1<<PORTC3);
	PORTC |= (0<<PORTC3);
	
	sei();			// Enable interrupts
	
	while(1);		// Infinite loop, waiting for interrupts
}