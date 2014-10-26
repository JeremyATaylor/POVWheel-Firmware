/*
 * NOTE: When programming the device, ensure fuses are set as follows:
 * -----
 * Extended Register: 0xFF
 * High Register: 0xDE
 * Low Register: 0xBF
 *
 */

#define F_CPU 16000000
#include <avr/boot.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>

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

#define TLC5940_N 2		// Number of TLC5940 LED drivers

#define CIRCUMFERENCE 1.76

#define dcData_t uint8_t
#define gsData_t uint8_t

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)16 * TLC5940_N)

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
};

volatile uint32_t thisPeriod;
volatile uint32_t period;
volatile uint8_t speed;
volatile uint32_t rotationCount;
volatile uint32_t distance;
volatile uint8_t displayFlag;

void displayZero(int T, int offset) {
	for (int i = offset; i < offset+6; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
	
	gsData[offset] = 0b11111111;
	for (int i = offset+1; i < offset+5; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b00000000;
	}
	gsData[offset+6] = 0b11111111;
	_delay_ms(T/180);
	
	for (int i = offset; i < offset+6; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayOne(int T, int offset) {
	for (int i = offset; i < offset+6; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b00000000;
	}
	_delay_ms(T/120);
	
	for (int i = offset; i < offset+7; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayTwo(int T, int offset) {
	gsData[offset] = 0b11111111;
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	for (int i = offset+3; i < offset+6; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
	
	gsData[offset+4] = 0b00000000;
	gsData[offset+5] = 0b00000000;
	_delay_ms(T/180);
	
	gsData[offset+1] = 0b11111111;
	gsData[offset+2] = 0b11111111;
	_delay_ms(T/360);
}
void displayThree(int T, int offset) {
	gsData[offset] = 0b1111111;
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	gsData[offset+3] = 0b1111111;
	gsData[offset+4] = 0b0000000;
	gsData[offset+5] = 0b0000000;
	gsData[offset+6] = 0b1111111;
	_delay_ms(T/120);
	
	for (int i = offset+1; i < offset+5; i++) {		// Set all grayscale values to 0
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayFour(int T, int offset) {
	for (int i = offset; i < offset+3; i++) {
		gsData[i] = 0b11111111;
	}
	for (int i = offset+4; i < offset+6; i++) {
		gsData[i] = 0b00000000;
	}
	_delay_ms(T/360);
	
	for (int i = offset; i < offset+2; i++) {
		gsData[i] = 0b00000000;
	}
	_delay_ms(T/180);
	
	for (int i = offset; i < offset+7; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayFive(int T, int offset) {
	for (int i = offset; i < offset+3; i++) {
		gsData[i] = 0b11111111;
	}
	gsData[offset+4] = 0b00000000;
	gsData[offset+5] = 0b00000000;
	gsData[offset+7] = 0b11111111;
	_delay_ms(T/360);
	
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	_delay_ms(T/180);
	
	gsData[offset+4] = 0b11111111;
	gsData[offset+5] = 0b11111111;
	_delay_ms(T/360);
}
void displaySix(int T, int offset) {
	for (int i = offset; i < offset+6; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
	
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	gsData[offset+4] = 0b00000000;
	gsData[offset+5] = 0b00000000;
	_delay_ms(T/180);
	
	gsData[offset+4] = 0b11111111;
	gsData[offset+5] = 0b11111111;
	_delay_ms(T/360);
}
void displaySeven(int T, int offset) {
	gsData[offset] = 0b11111111;
	for (int i = offset+1; i < offset+6; i++) {
		gsData[i] = 0b00000000;
	}
	_delay_ms(T/120);
	
	for (int i = offset; i < offset+6; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayEight(int T, int offset) {
	for (int i = offset; i < offset+6; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
	
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	gsData[offset+4] = 0b00000000;
	gsData[offset+5] = 0b00000000;
	_delay_ms(T/180);
	
	for (int i = offset; i < offset+6; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}
void displayNine(int T, int offset) {
	for (int i = offset; i < offset+3; i++) {
		gsData[i] = 0b11111111;
	}
	gsData[offset+4] = 0b00000000;
	gsData[offset+5] = 0b00000000;
	gsData[offset+6] = 0b11111111;
	_delay_ms(T/360);
	
	gsData[offset+1] = 0b00000000;
	gsData[offset+2] = 0b00000000;
	_delay_ms(T/180);
	
	for (int i = offset; i < offset+6; i++) {
		gsData[i] = 0b11111111;
	}
	_delay_ms(T/360);
}

void displayDigit(int digit, int T, int offset) {
	if (digit == 0) {
		displayZero(T, offset);
	} else if (digit == 1) {
		displayOne(T, offset);
	} else if (digit == 2) {
		displayTwo(T, offset);
	} else if (digit == 3) {
		displayThree(T, offset);
	} else if (digit == 4) {
		displayFour(T, offset);
	} else if (digit == 5) {
		displayFive(T, offset);
	} else if (digit == 6) {
		displaySix(T, offset);
	} else if (digit == 7) {
		displaySeven(T, offset);
	} else if (digit == 8) {
		displayEight(T, offset);
	} else if (diigt == 9) {
		displayNine(T, offset);
	}
}

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

	/* Initialize Timer2 */
	SPCR = (1 << SPE)|(1 << MSTR);		// Enable SPI and Master
	SPSR = (1 << SPI2X);				// Set clock rate fck/2
	TCCR2A = (1 << WGM21);				// Set timer 2 mode to CTC
	TCCR2B = (1 << CS22)|(1 << CS20);	// Set prescaler to 1024 and start timer 2
	OCR2A = 100;						// Interrupt every 13ms
	TIMSK2 |= (1 << OCIE2A);			// Enable Timer0 Match A interrupt
	
	/* Enable UART transmission and reception */
	UBRR0 = 51;							// Set baudrate to 19200
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);
	UCSR0A = (0 << U2X0);				// Enable Receive Complete Interrupt
	
	/* Setup Hall effect interrupt*/
	DDRD &= ~(1 << DDD3);
	PORTD |= (0 << PORTD3);
	EICRA |= (1 << ISC00);    // Set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT1);     // Enable INT1
 	
	/* Setup power LED */
	DDRC |= (1 << PORTC3);
	PORTC |= (0 << PORTC3);
 
	 /* Setup Mode switch */
	 DDRD &= ~(1 << PORTD6) ;
	 DDRD |= (1<< PORTD5);
	 PORTD |= (1<< PORTD5);
	 
 	/* Setup Timer 1 for normal operation */
//  TCCR1A |= (0 << WGM10) | (0 << WGM11) | (1 << COM1A1);	// Set normal clock operation
//  TCCR1B |= (1 << CS10) | (1 << CS12) | (0 << CS11);		// Set 1024 prescalar
// 	TIMSK1 |= (1 << OCIE1A);

 	/* Setup Timer 0 on CTC for interval compare */
	TCCR0A = (1 << WGM01);					// Set CTC clock operation
 	TCCR0B = (1 << CS00) | (1 << CS01);		// Set 64 prescalar
	OCR0A = 249;							
	TIMSK0 |= (1 << OCIE0A);				// Enable interrupt handler
	  	
	/* Set global variables */
    speed = 0;
    buffer = 0;
    rotationCount = 0;
    distance = 0;
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

/*
 * Function: initMode1
 * -------------------
 * Fix rotation speed to 200rpm, disable rotation sensing and standby modes.
 *
 */
int initMode1(void) {

	OCR2A = 15624*0.3/360;		// Set output compare register
	
	// Turn off rotation sensing
	// Set display speed threshold to 0
	// Make display at 200 rpm
	
	while(1) {
		if (PIND & (1<<PORTD6)) {
			
		} else {
			MODENUM = 2;
			initMode2();
		}
	}
	
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
	
	while(1) {
		if (PIND & (1<<PIND6)) {
			MODENUM = 3;
			initMode3();
		}
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
	
	gsData[0] = 0b11111111;		// Set outermost LED to max. brightness
	while(1);
	// Turn off rotation sensing
	// Set display speed threshold to 0
	return 0;
}

void USART0Init(void) {
	UBRR0 = 51;							// Set baud rate 19200
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);	// Enable transmission and reception
}


int main(void) {
	setup();				// Initialize hardware
	getDC();				// Clock in dot correction data
	USART0Init();			// Initialize USART0
	sei();					// Enable interrupts
	PORTC |= (1<<PORTC3);
	
	while (1) {
		if (displayFlag == 1) {
			_delay_ms(period/2)
			
			displayDigit(((speed/1000)%10), period, 4);
			displayDigit(((distance/1000)%10), period, 20);
			_delay_ms(period/360);
			
			displayDigit(((speed%1000)/100), period, 4);
			displayDigit((distance%1000)/100), period, 20);
			_delay_ms(period/360);
			
			displayDigit((speed%100)/10), period, 4);
			displayDigit((distance%100)/10), period, 20);
			_delay_ms(period/360);
			
			displayDigit((speed%10), period, 4);
			displayDigit((distance%10), period, 20);
			displayFlag = 0;
		}
	}
	return 0; 
}

/*
 * Interrupt handler: TIMER2_COMPA_vect
 * ----------------------------------
 * Timer2 output compare interrupt, set to occur every 13ms and updates display.
 *
 */
ISR (TIMER2_COMPA_vect) {
	TIMER++;
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
			XLAT_PORT |= (1 << XLAT_PIN);	// Pulse latch signal pin
			XLAT_PORT &= ~(1 << XLAT_PIN);
			latchNeedsPulse = 0;
		}
		
		BLANK_PORT &= ~(1 << BLANK_PIN);	// Set blank output pin low
		
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
 * Function: initTimer1
 * --------------------
 * Initialises Timer1, interrupt and global counter variable.
 * Note: For 60Hz, OCR1A = 15624.
 */
void initTimer1(void) {
    OCR1A = 15624;

    TCCR1B |= (1 << WGM12);		// Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A);    // Set interrupt on compare match
    
	// Set prescaler to 1024 and start the timer
    TCCR1B |= (1 << CS12) | (1 << CS10);
}

/*
 * Interrupt handler: TIMER0_COMPA_vect
 * ----------------------------------
 * Timer1 output compare interrupt, set to occur every 1ms.
 */
ISR (TIMER0_COMPA_vect) {
	thisPeriod++;
}

/*
 * Interrupt handler: TIMER1_COMPA_vect
 * ----------------------------------
 * Timer1 output compare interrupt, set to occur every 1 second.
 *
 */
ISR (TIMER1_COMPA_vect) {

}

/* Eternal Interrupt: INT1_vect
 * ----------------------------
 * Sets up external interrupt for hall effect sensor.
 *
 */
ISR (INT1_vect) {
	period = thisPeriod;
	thisPeriod = 0;
	
	rotationCount++;
	distance = rotationCount*CIRCUMFERENCE;
	
	displayFlag = 1;
}

