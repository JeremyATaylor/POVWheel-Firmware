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
#include <avr/pgmspace.h>
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

#define TLC5940_N 2

#define dcData_t uint8_t
#define gsData_t uint8_t

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)16 * TLC5940_N)

#define SAVE_ADDRESS 0x4000

#define CIRCUMFERENCE 1.76

volatile uint8_t ledRef;
volatile uint8_t dataIndex;
volatile long DEGINTERVAL;		// Interval for each degree
volatile long DISTANCE;			// Distance traveled since turned on
volatile long DEG;				// Where the display is positioned
volatile int MODENUM;			// Mode number
volatile int TOOSLOW;
volatile int SPEEDNUMS[3];
volatile int DISTANCENUMS[3];
volatile uint16_t TIMER;
static uint8_t imageData[128];	//Image data recieved for 4 columns
static uint8_t usbData[128];	//Immage data to be save 
volatile uint8_t usbEnable;
volatile uint8_t slowCounter;
volatile uint8_t standbyEnable;

volatile uint32_t thisPeriod;
volatile int period;
volatile uint8_t speed;
volatile uint32_t rotationCount;
volatile uint32_t distance;
volatile uint8_t displayFlag;

int initMode3(void);
uint8_t USART0_Receive(void);
void USART0_Transmit(uint8_t data);
void boot_program_page (uint32_t page, uint8_t *buf);
void getDataFromFlash(uint16_t pageNum);
void readUsb();

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

	/* Initialize Timer0 */
	SPCR = (1 << SPE)|(1 << MSTR);		// Enable SPI and Master
	SPSR = (1 << SPI2X);				// Set clock rate fck/2
	TCCR0A = (1 << WGM01);				// Set timer mode to CTC
	TCCR0B = (1 << CS02)|(1 << CS00);	// Set prescaler to 1024 and start timer
	// OCRn = [ (Clock speed / Prescale value) * (Desired time in seconds) ] - 1
	OCR0A = 3;							// Interrupt every 4096 clock cycles
	TIMSK0 |= (1 << OCIE0A);			// Enable Timer0 Match A interrupt
	
	DDRC |= (1 << PINC4);				// Declare test LED as output
	UBRR0 = 51;							// Set baudrate to 19200

    /* Setup Timer1 for interval compare */
    TCCR1B |= (1 << WGM12);					// Set CTC clock operation
    TCCR1B |= (1 << CS11)|(1 << CS10);	// Set prescaler to 64
    OCR1A = 249;
    TIMSK1 |= (1 << OCIE1A);				// Enable interrupt handler	
	
	/* Enable UART transmission and reception */
	UBRR0 = 51;						// Set baudrate to 19200
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);
	UCSR0C = (1 << USBS0)|(1 << UCSZ01)|(1 << UCSZ00); // Set two stop bits
	UCSR0A = (0 << U2X0);			// Enable Receive Complete Interrupt
		
	/* Setup Hall effect interrupt*/
	DDRD &= ~(1 << DDD3);
	DDRD &= ~(1 << DDD2);
	PORTD |= (0 << PORTD3) | (0<<PORTD2);  // Set INT0 to trigger on ANY logic change
	EICRA |= (1 << ISC00);
	EIMSK |= (1 << INT1) | (1<<INT0);     // Enable INT1
 	
	/* Setup power LED */
	DDRC |= (1 << PORTC3);
	PORTC |= (0 << PORTC3);
 
	/* Setup Mode switch */
	DDRD &= ~(1 << PORTD6) ;
	DDRD |= (1<< PORTD5);
	PORTD |= (1<< PORTD5);
	  	
	/* Set global variables */
	DEG = 0;
	DISTANCE = 0;
	dataIndex = 0;
	usbEnable = 0;
	slowCounter = 0;
	period = 0;
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
	// Turn off rotation sensing
	while(1);
	
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
	return 0;
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


void USART0Init(void) {
	UBRR0 = 51;							// Set baud rate 19200
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);	// Enable transmission and reception
}

int USART0SendByte(char u8Data, FILE *stream) {
	if (u8Data == '\n') {
		USART0SendByte('\r', stream);
	}
	
	while(!(UCSR0A & (1 << UDRE0)));	// Wait while previous byte is completed
	UDR0 = u8Data;						// Transmit data
	
	return 0;
}

// Set stream pointer
FILE usart0_str = FDEV_SETUP_STREAM(USART0SendByte, NULL, _FDEV_SETUP_WRITE);

void display_picture(uint8_t columnNum) {
	uint8_t num;
	num = columnNum*32;
	for (gsData_t i = 0; i < gsDataSize; i++) {		// Set all grayscale values to max brightness
		gsData[i] = imageData[num];
		num++;
	}
}
	


//void display(void) {
	//uint8_t pageCounter = 0;
	//int vertical;
	//
	//DEG = TIMER / DEGINTERVAL;
	//while(DEG > 360){
		//DEG = DEG -360;
		//}		// Calculate what pixel the display is up to
	//
	//if( MODENUM == 0 ){											
		//while(DEG > 4){
			//DEG = DEG - 4;
			//pageCounter++;
		//}
		//printf("%ld, %d", DEG, pageCounter);
		//if(DEG == 1){
			//getDataFromFlash(pageCounter);	
		//}
		//display_picture(DEG);
	//}
	//
//}

//int initMode4(void) {
	//display();	
	//return 0;
//}



void normalMode(void){
	while(1) {
		if (displayFlag == 1) {
			displayFlag = 0;
			for (uint8_t i = 0; i < 360; i++) {
				display_picture(i);
				_delay_ms(50);
			}
		}
	}
}



int main(void) {
	setup();				// Initialize hardware
	getDC();				// Clock in dot correction data

	sei();	
	PORTC |= (1<<PORTC3);
	
	 if(PIND & (1 << PORTD6)){
 		MODENUM = 1;
		initMode1();
 	} else {
 		MODENUM = 0;
 		normalMode();
 	}

	return 0; 
}


/*
 * Interrupt Handler: TIMER0_COMPA_vect
 * ------------------------------------
 * Displays the image.
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
 * Interrupt handler: TIMER0_COMPA_vect
 * ------------------------------------
 * Timer1 output compare interrupt, set to occur every 1ms.
 *
 */
ISR (TIMER1_COMPA_vect) {
    thisPeriod++;
}

/* 
 * External Interrupt: INT1_vect
 * -----------------------------
 * Sets up external interrupt for hall effect sensor, signifies one 
 * complete wheel rotation.
 *
 */
ISR (INT1_vect) {
    period = thisPeriod;
    thisPeriod = 0;
    
    rotationCount++;
    distance = rotationCount*CIRCUMFERENCE;
    
    displayFlag = 1;
}


/* 
 * Interrupt handler: UART Receive Complete
 * ----------------------------------------
 * i.e. a new byte has arrived in the UART Data Register (UDR).
 *
 */
ISR(USART_RX_vect) {
	cli();
	readUsb();
	usbEnable = 1;
}

uint8_t USART0_Receive(void){
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void USART0_Transmit(uint8_t data){
	while(!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

BOOTLOADER_SECTION void boot_program_page (uint32_t page, uint8_t *buf) {
	uint16_t i;
	uint8_t sreg;

	// Disable interrupts.

	sreg = SREG;
	cli();
	
	eeprom_busy_wait ();

	boot_page_erase (page);
	boot_spm_busy_wait ();      // Wait until the memory is erased.

	for (i=0; i<SPM_PAGESIZE; i+=2) {
		// Set up little-endian word.

		uint16_t w = *buf++;
		w += (*buf++) << 8;
		
		boot_page_fill (page + i, w);
	}

	boot_page_write (page);     // Store buffer in flash page.
	boot_spm_busy_wait();       // Wait until the memory is written.

	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.

	boot_rww_enable ();

	// Re-enable interrupts (if they were ever enabled).

	SREG = sreg;
}

void readUsb() {
	uint16_t pageNum;
	uint8_t byteNum;
	USART0_Transmit('1');
	for(pageNum = 0; pageNum < 90; pageNum++) {
		for(byteNum = 0; byteNum < 128; byteNum++) {
			usbData[byteNum] = USART0_Receive();
		}
	boot_program_page(SAVE_ADDRESS+(pageNum*128), usbData);
	PORTC ^= (1<<PORTC3);								// Blink LED when interacting with PC
	USART0_Transmit('1');								// Send Confirmation of save
	}
	usbEnable = 0;
}

void getDataFromFlash(uint16_t pageNum) {
	for (uint16_t i = 0; i < 128; i++) {
		imageData[i] = pgm_read_byte(SAVE_ADDRESS + i + pageNum*128);
		USART0_Transmit(imageData[i]);
	}
}