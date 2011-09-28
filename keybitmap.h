#ifndef KEYBITMAP_H
#define KEYBITMAP_H

#include <stdint.h>
#include <stdbool.h>

void bmInit(void);
void bmSetPressed(uint8_t key, bool extended);
void bmSetReleased(uint8_t key, bool extended);
bool bmIsPressed(uint16_t key);

#endif
