/*

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>

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

#define TLC5940_N 1

#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))
#define pulse(port, pin) do { \
	setHigh((port), (pin)); \
	setLow((port), (pin)); \
} while (0)
#define outputState(port, pin) ((port) & (1 << (pin)))

volatile int which_array = 0;

uint8_t dcData[96 * TLC5940_N] = {
// MSB            LSB
	1, 1, 1, 1, 1, 1,			// Channel 15
	1, 1, 1, 1, 1, 1,			// Channel 14
	1, 1, 1, 1, 1, 1,			// Channel 13
	1, 1, 1, 1, 1, 1, 			// Channel 12
	1, 1, 1, 1, 1, 1,			// Channel 11
	1, 1, 1, 1, 1, 1,			// Channel 10
	1, 1, 1, 1, 1, 1,			// Channel 9
	1, 1, 1, 1, 1, 1, 			// Channel 8
	1, 1, 1, 1, 1, 1, 			// Channel 7
	1, 1, 1, 1, 1, 1,			// Channel 6
	1, 1, 1, 1, 1, 1,			// Channel 5
	1, 1, 1, 1, 1, 1, 			// Channel 4
	1, 1, 1, 1, 1, 1, 			// Channel 3
	1, 1, 1, 1, 1, 1, 			// Channel 2
	1, 1, 1, 1, 1, 1, 			// Channel 1
	1, 1, 1, 1, 1, 1, 			// Channel 0
};

uint8_t gsData1[192 * TLC5940_N] = {
	// MSB                          LSB
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 		// Channel 15
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 14
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 13
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,			// Channel 12
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,			// Channel 11
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,			// Channel 10
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,			// Channel 9
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,			// Channel 8
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,			// Channel 7
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,			// Channel 6
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,			// Channel 5
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 4
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 3
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 2
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 1
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,			// Channel 0
};

uint8_t gsData2[192 * TLC5940_N] = {
	// MSB                          LSB
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 		// Channel 15
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,			// Channel 14
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,			// Channel 13
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,			// Channel 12
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,			// Channel 11
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 10
	0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0,			// Channel 9
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 8
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,			// Channel 7
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 6
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 5
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 4
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,			// Channel 3
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,			// Channel 2
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,			// Channel 1
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,			// Channel 0
};

uint8_t gsData3[192 * TLC5940_N] = {
// MSB                              LSB
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 		// Channel 15
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 14
	0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 13
	0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 12
	0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 11
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,			// Channel 10
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,			// Channel 9
	0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,			// Channel 8
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,			// Channel 7
	0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,			// Channel 6
	0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,			// Channel 5
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,			// Channel 4
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,			// Channel 3
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 2
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 1
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,			// Channel 0
};

uint8_t gsData[192 * TLC5940_N] = {};

void TLC5940_Init(void) {
	setOutput(GSCLK_DDR, GSCLK_PIN);
	setOutput(SCLK_DDR, SCLK_PIN);
	setOutput(DCPRG_DDR, DCPRG_PIN);
	setOutput(VPRG_DDR, VPRG_PIN);
	setOutput(XLAT_DDR, XLAT_PIN);
	setOutput(BLANK_DDR, BLANK_PIN);
	setOutput(SIN_DDR, SIN_PIN);
	
	setLow(GSCLK_PORT, GSCLK_PIN);
	setLow(SCLK_PORT, SCLK_PIN);
	setLow(DCPRG_PORT, DCPRG_PIN);
	setHigh(VPRG_PORT, VPRG_PIN);
	setLow(XLAT_PORT, XLAT_PIN);
	setHigh(BLANK_PORT, BLANK_PIN);
}

void TLC5940_ClockInDC(void) {
	setHigh(DCPRG_PORT, DCPRG_PIN);
	setHigh(VPRG_PORT, VPRG_PIN);

	uint8_t Counter = 0;
	
	while (1) {
		if (Counter > TLC5940_N * 96 - 1) {
			pulse(XLAT_PORT, XLAT_PIN);
			break;
			} else {
			if (dcData[Counter])
			setHigh(SIN_PORT, SIN_PIN);
			else
			setLow(SIN_PORT, SIN_PIN);
			pulse(SCLK_PORT, SCLK_PIN);
			Counter++;
		}
	}
}

void TLC5940_SetGS_And_GS_PWM(void) {
	uint8_t firstCycleFlag = 0;
	
	if (outputState(VPRG_PORT, VPRG_PIN)) {
		setLow(VPRG_PORT, VPRG_PIN);
		firstCycleFlag = 1;
	}
	
	uint16_t GSCLK_Counter = 0;
	uint8_t Data_Counter = 0;
	
	setLow(BLANK_PORT, BLANK_PIN);
	for (;;) {
		if (GSCLK_Counter > 4095) {
			setHigh(BLANK_PORT, BLANK_PIN);
			pulse(XLAT_PORT, XLAT_PIN);
			if (firstCycleFlag) {
				pulse(SCLK_PORT, SCLK_PIN);
				firstCycleFlag = 0;
			}
			break;
			} else {
			if (!(Data_Counter > TLC5940_N * 192 - 1)) {
				if (gsData[Data_Counter])
				setHigh(SIN_PORT, SIN_PIN);
				else
				setLow(SIN_PORT, SIN_PIN);
				pulse(SCLK_PORT, SCLK_PIN);
				Data_Counter++;
			}
		}
		pulse(GSCLK_PORT, GSCLK_PIN);
		GSCLK_Counter++;
	}
}
// Comment
// this code sets up a timer0 for 4ms @ 16Mhz clock cycle
// an interrupt is triggered each time the interval occurs.

#include <avr/io.h>
#include <avr/interrupt.h>

void timer_Init(void) {
	// Set the Timer Mode to CTC
	TCCR0A |= (1 << WGM01);

	// Set the value that you want to count to
	// OCRn =  [ (clock_speed / Prescaler_value) * Desired_time_in_Seconds ] - 1
	OCR0A = 0xFF;

	TIMSK0 |= (1 << OCIE0A);    // Set the ISR COMPA_vect

	sei();	// Enable interrupts

	// Set prescaler to 256 and start the timer
	TCCR0B |= (1 << CS00);
	TCCR0B |= (1 << CS02);
}

// timer0 overflow interrupt
ISR (TIMER0_COMPA_vect) {
	
	PORTC ^= (1<<PORTC4);
	
	
	if (which_array == 0) {
		for (int i = 0; i < (192 * TLC5940_N); i++) {
			gsData[i] = gsData1[i];
		}
		which_array++;
	} else if (which_array == 1) {
		for (int i = 0; i < (192 * TLC5940_N); i++) {
			gsData[i] = gsData2[i];
		}
		which_array++;
	} else  {
		for (int i = 0; i < (192 * TLC5940_N); i++) {
			gsData[i] = gsData3[i];
		}
		which_array = 0;
	}
}

void init_USART(void) {
	DDRC = (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3) | (1<<PORTC4) | (1<<PORTC5);
	// Set the baud rate to 19200 
	UBRR0 = 51;
	
	
	// Enable transmission and receiving via UART and also 
	// enable the Receive Complete Interrupt.
	// (See page 190 of the datasheet)
	
	UCSR0B = (1<< RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0A = (0<<U2X0);

	// NOTE: No need to set UCSR0C - we justs want the default value

	// Enable interrupts
	sei();
	
}

ISR(USART_RX_vect) {
	char input;

	// Extract character from UART Data register and place in input variable
	
	input = UDR0;
	
	// test whether it is between 1-5 and set the corresponding port
	if(input == 48){
		PORTC ^= (1<<PORTC0);
	}
	if(input == 49){
		PORTC ^= (1<<PORTC1);
	}
	if(input == 50){
		PORTC ^= (1<<PORTC2);
	}
	if(input == 51){
		PORTC ^= (1<<PORTC3);
	}
	if(input == 52){
		PORTC ^= (1<<PORTC4);
	}
	if(input == 53){
		PORTC ^= (1<<PORTC5);
	}

	// Send the character back over the serial connection
	UDR0 = input;
}

int main(void) {
	init_USART();
	timer_Init();
	TLC5940_Init();
	TLC5940_ClockInDC();	// try it both with and without this line

	while (1) {
		TLC5940_SetGS_And_GS_PWM();
	}
		
	return 0;
}
*/

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

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

/*#define setOutput(ddr, pin) ((ddr) |= (1 << (pin)))
#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))*/


#if (12 * TLC5940_N > 255)
#define dcData_t uint16_t
#else
#define dcData_t uint8_t
#endif

#if (24 * TLC5940_N > 255)
#define gsData_t uint16_t
#else
#define gsData_t uint8_t
#endif

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)24 * TLC5940_N)

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

uint8_t gsData[24 * TLC5940_N] = {
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000001,
	0b00000000,
	0b00100000,
	0b00000100,
	0b00000000,
	0b10000000,
	0b00010000,
	0b00000010,
	0b00000000,
	0b01000000,
	0b00001000,
	0b00000001,
	0b00000000,
	0b00100000,
	0b00000100,
	0b00000000,
	0b10000000,
	0b00001111,
	0b11111111,
	
	0b11111111, 
	0b11111000,
	0b00000000,
	0b01000000,
	0b00000010,
	0b00000000,
	0b00010000,
	0b00000000,
	0b10000000,
	0b00000100,
	0b00000000,
	0b00100000,
	0b00000001,
	0b00000000,
	0b00001000,
	0b00000000,
	0b01000000,
	0b00000010,
	0b00000000,
	0b00010000,
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
}

/* Get dot correction values */
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
	for (gsData_t i = 0; i < gsDataSize; i++) {
		SPDR = gsData[i];
		while (!(SPSR & (1 << SPIF)));
	}
	latchNeedsPulse = 1;
}

int main(void) {
	setup();	// Initialize hardware
	getDC();	// Clock in dot correction data
	
	sei();		// Enable interrupts

	while(1);	// Infinite loop, waiting for interrupts
	
	return 0;
}
