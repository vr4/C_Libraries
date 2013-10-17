/*
	Author: Eric Conner
	Project: ATtiny45
	Date: 05-31-2013
	File: ATtiny45.c
	Change Log:
		v1.1 - 07-09-2013 - Fixed Pin Numbers
		v1.2 - 08-29-2013 - Added Shift-In and Shift-Out
		v1.3 - 09-05-2013 - Added Delay
*/

#include "ATtiny45.h"
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
	NOT_A_PORT,
	(uint16_t) &DDRB,
	NOT_A_PORT,
	NOT_A_PORT,
};

const uint16_t PROGMEM port_to_output_PGM[5] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	NOT_A_PORT,
	NOT_A_PORT,
};

const uint16_t PROGMEM port_to_input_PGM[5] = {
	NOT_A_PIN,
	NOT_A_PIN,
	(uint16_t) &PINB,
	NOT_A_PIN,
	NOT_A_PIN,
};

const int PROGMEM digital_pin_to_port_PGM[6] = {
	PB,	// PB0 ** 0 ** D0
	PB,	// PB1 ** 1 ** D1
	PB,	// PB2 ** 2 ** D2
	PB,	// PB3 ** 3 ** D3
	PB,	// PB4 ** 4 ** D4
	PB,	// PB5 ** 5 ** D5
};

const int PROGMEM digital_pin_to_bit_mask_PGM[6] = {
	_BV(0),	// PB0 ** 0 ** D0
	_BV(1),	// PB1 ** 1 ** D1
	_BV(2),	// PB2 ** 2 ** D2
	_BV(3),	// PB3 ** 3 ** D3
	_BV(4),	// PB4 ** 4 ** D4
	_BV(5),	// PB5 ** 5 ** D5
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
