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
#ifndef KEYBITMAP_H
#define KEYBITMAP_H

#include <stdint.h>
#include <stdbool.h>

void bmInit(void);
void bmSetPressed(uint8_t key, bool extended);
void bmSetReleased(uint8_t key, bool extended);
bool bmIsPressed(uint16_t key);

#endif
