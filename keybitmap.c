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
#include "keybitmap.h"

static uint8_t pressed[64];

void bmInit(void) {
	uint8_t i;
	for ( i = 0; i < 64; i++ ) {
		pressed[i] = 0x00;
	}
}

void bmSetPressed(uint8_t key, bool extended) {
	if ( extended ) {
		pressed[(key>>3) + 32] |= (1<<(key&7));
	} else {
		pressed[key>>3] |= (1<<(key&7));
	}
}

void bmSetReleased(uint8_t key, bool extended) {
	if ( extended ) {
		pressed[(key>>3) + 32] &= ~(1<<(key&7));
	} else {
		pressed[key>>3] &= ~(1<<(key&7));
	}
}

bool bmIsPressed(uint16_t key) {
	return pressed[key>>3] & (1<<(key&7));
}
