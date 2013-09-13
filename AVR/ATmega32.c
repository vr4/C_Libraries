/*
	Author: Eric Conner
	Project: ATmega32
	Date: 05-31-2013
	File: ATmega32.c
	Change Log:
		v1.1 - 07-09-2013 - Fixed Pin Numbers
		v1.2 - 08-29-2013 - Added shiftIn() and shiftOut()
		v1.3 - 09-05-2013 - Added Delay
*/

#include "ATmega32.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define digitalPinToPort(P)		(pgm_read_byte(digital_pin_to_port_PGM + (P)))
#define digitalPinToBitMask(P)	(pgm_read_byte(digital_pin_to_bit_mask_PGM + (P)))
#define portOutputRegister(P)	((volatile int *)(pgm_read_word(port_to_output_PGM + (P))))
#define portInputRegister(P)	((volatile int *)(pgm_read_word(port_to_input_PGM + (P))))
#define portModeRegister(P)		((volatile int *)(pgm_read_word(port_to_mode_PGM + (P))))

const uint16_t PROGMEM port_to_mode_PGM[5] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[5] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[5] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const int PROGMEM digital_pin_to_port_PGM[32] = {
	PB,	// PB0 ** 0 ** D0
	PB,	// PB1 ** 1 ** D1
	PB,	// PB2 ** 2 ** D2
	PB,	// PB3 ** 3 ** D3
	PB,	// PB4 ** 4 ** D4
	PB,	// PB5 ** 5 ** D5
	PB,	// PB6 ** 6 ** D6
	PB,	// PB7 ** 7 ** D7
	PD,	// PD0 ** 8 ** D8
	PD,	// PD1 ** 9 ** D9
	PD,	// PD2 ** 10 ** D10
	PD,	// PD3 ** 11 ** D11
	PD,	// PD4 ** 12 ** D12
	PD,	// PD5 ** 13 ** D13
	PD,	// PD6 ** 14 ** D14
	PD,	// PD7 ** 15 ** D15
	PC,	// PC0 ** 16 ** D16
	PC,	// PC1 ** 17 ** D17
	PC,	// PC2 ** 18 ** D18
	PC,	// PC3 ** 19 ** D19
	PC,	// PC4 ** 20 ** D20
	PC,	// PC5 ** 21 ** D21
	PC,	// PC6 ** 22 ** D22
	PC,	// PC7 ** 23 ** D23
	PA,	// PA7 ** 24 ** A0 D24
	PA,	// PA6 ** 25 ** A1 D25
	PA,	// PA5 ** 26 ** A2 D26
	PA,	// PA4 ** 27 ** A3 D27
	PA,	// PA3 ** 28 ** A4 D28
	PA,	// PA2 ** 29 ** A5 D29
	PA,	// PA1 ** 30 ** A6 D30
	PA,	// PA0 ** 31 ** A7 D31
};

const int PROGMEM digital_pin_to_bit_mask_PGM[32] = {
	_BV(0),	// PB0 ** 0 ** D0
	_BV(1),	// PB1 ** 1 ** D1
	_BV(2),	// PB2 ** 2 ** D2
	_BV(3),	// PB3 ** 3 ** D3
	_BV(4),	// PB4 ** 4 ** D4
	_BV(5),	// PB5 ** 5 ** D5
	_BV(6),	// PB6 ** 6 ** D6
	_BV(7),	// PB7 ** 7 ** D7
	_BV(0),	// PD0 ** 8 ** D8
	_BV(1),	// PD1 ** 9 ** D9
	_BV(2),	// PD2 ** 10 ** D10
	_BV(3),	// PD3 ** 11 ** D11
	_BV(4),	// PD4 ** 12 ** D12
	_BV(5),	// PD5 ** 13 ** D13
	_BV(6),	// PD6 ** 14 ** D14
	_BV(7),	// PD7 ** 15 ** D15
	_BV(0),	// PC0 ** 16 ** D16
	_BV(1),	// PC1 ** 17 ** D17
	_BV(2),	// PC2 ** 18 ** D18
	_BV(3),	// PC3 ** 19 ** D19
	_BV(4),	// PC4 ** 20 ** D20
	_BV(5),	// PC5 ** 21 ** D21
	_BV(6),	// PC6 ** 22 ** D22
	_BV(7),	// PC7 ** 23 ** D23
	_BV(7),	// PA7 ** 24 ** A0 D24
	_BV(6),	// PA6 ** 25 ** A1 D25
	_BV(5),	// PA5 ** 26 ** A2 D26
	_BV(4),	// PA4 ** 27 ** A3 D27
	_BV(3),	// PA3 ** 28 ** A4 D28
	_BV(2),	// PA2 ** 29 ** A5 D29
	_BV(1),	// PA1 ** 30 ** A6 D30
	_BV(0),	// PA0 ** 31 ** A7 D31
};

void pinMode(int pin, int mode) {
	int bit = digitalPinToBitMask(pin);
	int port = digitalPinToPort(pin);
	volatile int *reg, *out;
	reg = portModeRegister(port);
	out = portOutputRegister(port);
	if (mode == INPUT) {
		*reg &= ~bit;
		*out &= ~bit;
	} else if (mode == INPUT_PULLUP) {
		*reg &= ~bit;
		*out |= bit;
	} else {
		*reg |= bit;
	}
}

int digitalRead(int pin) {
	int bit = digitalPinToBitMask(pin);
	int port = digitalPinToPort(pin);
	if (*portInputRegister(port) & bit) {
		return HIGH;
	} else {
		return LOW;
	}
}

void digitalWrite(int pin, int val) {
	int bit = digitalPinToBitMask(pin);
	int port = digitalPinToPort(pin);
	volatile int *out;
	out = portOutputRegister(port);
	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}
}

int shiftIn(int dataPin, int latchPin, int clockPin) {
	int value = 0;
	int i;
	digitalWrite(latchPin, LOW);
	digitalWrite(latchPin, HIGH);
	for (i = 0; i < 8; ++i) {
		digitalWrite(clockPin, HIGH);
		value |= digitalRead(dataPin) << (7 - i);
		digitalWrite(clockPin, LOW);
	}
	return value;
}

void shiftOut(int dataPin, int latchPin, int clockPin, int val) {
	int i;
	digitalWrite(latchPin, LOW);
	for (i = 0; i < 8; i++) {
		digitalWrite(dataPin, !!(val & (1 << (7 - i))));
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);
	}
	digitalWrite(latchPin, HIGH);
}

void delay(unsigned long ms) {
	uint16_t i;
    for (i = 0; i < ms; i++) {
		_delay_ms(1);
	}
}
