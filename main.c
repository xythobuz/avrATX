/* ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <taucher.bodensee@gmail.com> wrote this file. As long as you retain this
 * notice you can do whatever you want with this stuff. If we meet some day,
 * and you think this stuff is worth it, you can buy me a beer
 * in return! 												Thomas Buck
 * ----------------------------------------------------------------------------
 * Title:		Jugendtreff FiBa Roller Steuerung
 * Author:		Thomas Buck (www.xythobuz.org)
 * Software:	AVR-GCC 4.3.3 tested
 * Hardware:	AtTiny2313, 16MHz
 * Compilation: Standard win-avr makefile with correct settings.
 * Description:
 *				This program controls a ATX-Power Supply and some relays.
 *				It's designed as a simple State Machine.
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define R1 PD4
#define R2 PD5
#define R3 PB4
#define F PD6
#define GREEN PB1
#define RED PB2
#define OFF 0
#define ON 1

#ifndef F_CPU
#define F_CPU 16000000
#endif

volatile uint8_t mode = OFF;
volatile uint8_t blinkcount = 0;
uint8_t powersup = OFF;

void powerOn(void);
void powerOff(void);
void entprellung(volatile uint8_t*, uint8_t);
void setup(void);
int main(void);

void powerOn(void) {
	// Switch the Power Supply on. This takes a max. t of 500ms, so we delay.
	powersup = ON;
	PORTD &= ~(1 << F); // Power on
	_delay_ms(500);
	if ( !(PINB & (1 << PINB0)) ) { // If the power supply has an error...
		powersup = OFF;
		PORTD |= (1 << F); // ...turn it off again.
	}
}

void powerOff(void) {
	// Turn the power supply off.
	powersup = OFF;
	PORTD |= (1 << F);
}

void entprellung(volatile uint8_t *port, uint8_t maske) {
	uint8_t port_puffer;
	uint8_t entprellungs_puffer;
	
	for( entprellungs_puffer=0 ; entprellungs_puffer!=0xff ; ) {
		entprellungs_puffer<<=1;
		port_puffer = *port;
		_delay_us(150);
		if( (*port & maske) == (port_puffer & maske) )
			entprellungs_puffer |= 0x01;
	}
}

void setup(void) {
	// Set up the port directions, start values, configure Timer...
	PORTD |= (1 << F); // F Needs to be high first!
	DDRD |= (1 << F) | (1 << R1) | (1 << R2);
	DDRB |= (1 << GREEN) | (1 << RED) | (1 << R3);
	TCCR0B |= (1 << CS02) | (1 << CS00); // Timer0 prescaler 1024
	TIMSK |= (1 << TOIE0); // Enable Timer0 overflow interrupt
	sei(); // Enable Interrupts
}

ISR (TIMER0_OVF_vect) { // Timer 0 Overflow Routine
	/* 
	 * We want it to execute twice a second, but it occurs
	 * about 61 times a second! So we count a variable to 30!
	 */
	if (blinkcount >= 30) {
		blinkcount = 0; // Reset Counter
		if (mode == ON) {
			PORTB ^= (1 << R3); // Toggle R3
		}
	} else {
		blinkcount++; // We are too fast...
		return;
	}
}

int main(void) {
	/*
	 * Program execution starts here!
	 */
	uint8_t temp = 0;
	setup();
	_delay_ms(2000); // Small delay for Power supply to get ready.
	while (1) {
		entprellung(&PIND, (1 << PD3));
		temp = PIND;
		if ( !(temp & (1 << PD3)) ) {
			mode = OFF; // When master button is off, we are in mode off!
		}
		if ( temp & (1 << PD3) ) {
			mode = ON;
		}
		
		if (mode == OFF) {
			PORTB |= (1 << GREEN) | (1 << RED); // Turn LEDs on
			PORTD &= ~((1 << R1) | (1 << R2)); // R1, R2 off
			if (powersup == ON)
				powerOff();
		}
		if (mode == ON) {
			if (powersup == OFF)
				powerOn();
			
			if (powersup == OFF) {
				PORTB |= (1 << RED); // Red LED on
			} else {
				PORTB &= ~(1 << RED); // Red LED off
			}
			PORTB |= (1 << GREEN); // Green LED on
			PORTD |= (1 << R1) | (1 << R2); // R1, R2 on
		}
	}
	return 0;
}