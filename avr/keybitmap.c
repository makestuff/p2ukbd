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
