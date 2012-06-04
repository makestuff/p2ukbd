/* 
 * Copyright (C) 2011 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <makestuff.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "fifo.h"

static volatile uint8 buffer[128];
static volatile uint8 putPtr;
static volatile uint8 getPtr;
static volatile bool isFull;
static volatile bool isEmpty;

void fifoInit(void) {
	putPtr = 0;
	getPtr = 0;
	isFull = false;
	isEmpty = true;
}

void put(uint8 byte) {
	// Called from ISR, so no need to clear interrupts
	if ( isFull ) {
		// This should never happen, but if it does, make it obvious!
		cli();
		DDRD = 0x40;
		for ( ; ; ) {
			PORTD = 0x40;
			_delay_ms(25);
			PORTD = 0x00;
			_delay_ms(25);
		}
	}
	buffer[putPtr++] = byte;
	putPtr &= 0x7f;
	isEmpty = false;
	if ( putPtr == getPtr ) {
		isFull = true;
	}
}

bool get(uint8 *byte) {
	// Disable interrupts to stop clashing access to data
	cli();
	if ( isEmpty ) {
		sei();
		return false;
	}
	*byte = buffer[getPtr++];
	getPtr &= 0x7f;
	isFull = false;
	if ( putPtr == getPtr ) {
		isEmpty = true;
	}
	sei();
	return true;
}
