/*
 jihex.h parses simplified Intel hex files as used for programming many
 microcontrollers

 Copyright (C) 2015  Julian Ingram

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JIHEX_H
#define	JIHEX_H

#include <stdint.h>

int jihex_handle(const char* filename, void(*write)(void*, uint32_t, uint8_t),
                 void* arg);

#endif
