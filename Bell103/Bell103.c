/*
 * Bell103.c
 *
 *    Copyright 2014 Karl Koscher
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * From Wikipedia (http://en.wikipedia.org/wiki/Bell_103_modem):
 *  The originating station used a mark tone of 1,270 Hz and a space tone of 1,070 Hz.
 *  The answering station used a mark tone of 2,225 Hz and a space tone of 2,025 Hz.
 *
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <math.h>
#include <stdint.h>

const uint8_t sinTable[] PROGMEM = {
	0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 0x98, 0x9b, 0x9e, 0xa1, 0xa4, 0xa7, 0xaa, 0xad,
	0xb0, 0xb3, 0xb6, 0xb9, 0xbb, 0xbe, 0xc1, 0xc3, 0xc6, 0xc9, 0xcb, 0xce, 0xd0, 0xd2, 0xd5, 0xd7,
	0xd9, 0xdb, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe7, 0xe9, 0xeb, 0xec, 0xee, 0xf0, 0xf1, 0xf2, 0xf4,
	0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfb, 0xfc, 0xfd, 0xfd, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe,
	0xff, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfd, 0xfd, 0xfc, 0xfb, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 0xf6,
	0xf5, 0xf4, 0xf2, 0xf1, 0xf0, 0xee, 0xec, 0xeb, 0xe9, 0xe7, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdb,
	0xd9, 0xd7, 0xd5, 0xd2, 0xd0, 0xce, 0xcb, 0xc9, 0xc6, 0xc3, 0xc1, 0xbe, 0xbb, 0xb9, 0xb6, 0xb3,
	0xb0, 0xad, 0xaa, 0xa7, 0xa4, 0xa1, 0x9e, 0x9b, 0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83,
	0x80, 0x7c, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a, 0x67, 0x64, 0x61, 0x5e, 0x5b, 0x58, 0x55, 0x52,
	0x4f, 0x4c, 0x49, 0x46, 0x44, 0x41, 0x3e, 0x3c, 0x39, 0x36, 0x34, 0x31, 0x2f, 0x2d, 0x2a, 0x28,
	0x26, 0x24, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x18, 0x16, 0x14, 0x13, 0x11, 0x0f, 0x0e, 0x0d, 0x0b,
	0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x04, 0x03, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0a, 0x0b, 0x0d, 0x0e, 0x0f, 0x11, 0x13, 0x14, 0x16, 0x18, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x24,
	0x26, 0x28, 0x2a, 0x2d, 0x2f, 0x31, 0x34, 0x36, 0x39, 0x3c, 0x3e, 0x41, 0x44, 0x46, 0x49, 0x4c,
	0x4f, 0x52, 0x55, 0x58, 0x5b, 0x5e, 0x61, 0x64, 0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7c,
};

typedef struct {
	int16_t lastSample;
	int16_t lpfInBuf[3];
	enum {
		SYNC_STATE_IDLE,
		SYNC_STATE_MARK,
		SYNC_STATE_SPACE,
	} syncState;
	int8_t sampsSinceLastStateChange;

	// Offset into the outgoing sine wave
	uint16_t phaseOut;
} line_t;

line_t line;
	
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
	uint8_t i;
	int16_t sample;
	int16_t quadSample;
	
	PORTD |= _BV(PORTD2);
	
	sample = (int16_t)(ADC - 0x200);
	quadSample = sample * line.lastSample; 
	line.lastSample = sample;
	
	// Low-pass filter the mixed quadSample
	int16_t t = line.lpfInBuf[0];
	int16_t p = quadSample - t;
	line.lpfInBuf[0] = (p >> 2) + (p >> 4) + (p >> 6) + t;
	int16_t lpfSample = line.lpfInBuf[0] - p;
	
	t = line.lpfInBuf[2];
	p = line.lpfInBuf[1] - t;
	line.lpfInBuf[2] = (p >> 4) + (p >> 6) - (p >> 8);
	
	t = line.lpfInBuf[2] - p;
	p = quadSample - t;
	line.lpfInBuf[1] = (p >> 2) + (p >> 4) + (p >> 7) + t;
	lpfSample += line.lpfInBuf[1] - p;
	
	if (line.syncState == SYNC_STATE_IDLE) {
		if (lpfSample < 0) {
			line.syncState = SYNC_STATE_SPACE;
		}
	} else if (++line.sampsSinceLastStateChange == 14) {
		// Sample bit
		PORTD = (PORTD & ~_BV(PORTD1)) | (lpfSample > 0 ? _BV(PORTD1) : 0);
	} else if (line.sampsSinceLastStateChange > 32) {
		line.sampsSinceLastStateChange -= 28;
	} else if (line.sampsSinceLastStateChange > 24) {
		if (line.syncState == SYNC_STATE_SPACE && lpfSample > 0) {
			line.syncState = SYNC_STATE_MARK;
			line.sampsSinceLastStateChange = 0;
		} else if (line.syncState == SYNC_STATE_MARK && lpfSample < 0) {
			line.syncState = SYNC_STATE_SPACE;
			line.sampsSinceLastStateChange = 0;
		}
	}
	
	// Dumb output
	//PORTD = (PORTD & ~(_BV(PORTD2) | _BV(PORTD1))) |
	//	(lpfSample > 0 ? _BV(PORTD1) : 0);
	
	// Kick off the next ADC conversion
	ADCSRA |= _BV(ADSC);
}

ISR(TIMER0_OVF_vect) {
	// Read the RXD pin
	
	if (PIND & 1) {
		line.phaseOut += 4666; // ~2225 Hz
	} else {
		line.phaseOut += 4247; // ~2025 Hz
	}
	
	OCR0A = (pgm_read_byte(&sinTable[line.phaseOut >> 8]) >> 2);
}	


int main(void)
{	
	PORTB = 0;
	DDRB = 63; // 6 bit DAC output
	PORTC = 0;
	DDRC = 0;
	DDRD = _BV(PORTD1) | _BV(PORTD2) | _BV(PORTD6);
	
	// Initialize sample clock (~8500 Hz), assuming 16 MHz clock
	OCR2A = 235;
	TIMSK2 = _BV(OCIE2A);
    TCCR2A = _BV(WGM21); 
	TCCR2B = _BV(CS21) | _BV(CS20);
	
	// Initialize PWM
	OCR0A = 0x80;
	TCCR0A = _BV(WGM00) | _BV(COM0A1);
	TCCR0B = _BV(CS00);
	TIMSK0 = _BV(TOIE0);

	// Initialize the ADC
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	ADMUX = _BV(REFS0);
	ADCSRB = 0;
	ADCSRA |= _BV(ADSC);

	// Kill the USART
	UCSR0B = 0;
		
	sei();
	
	for (;;) {
		sleep_mode();
	}
}
