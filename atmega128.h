/*
 atmega128.h a wrapper for avr.c simulating an ATmega128
 * 
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

#ifndef ATMEGA128_H
#define	ATMEGA128_H

#include "avr.h"

#define ATMEGA128_PMEM_SIZE (0x10000) // in uint16_ts
#define ATMEGA128_DMEM_SIZE (0x1100) // in uint8_ts

#define ATMEGA128_IO_REGISTERS_START (0x20)
#define ATMEGA128_IO_REGISTERS_END (0xFF)
#define ATMEGA128_IO_REGISTERS_LENGTH (ATMEGA128_IO_REGISTERS_END \
- ATMEGA128_IO_REGISTERS_START)

#define ATMEGA128_SREG_LOC (0x5F)

#define ATMEGA128_STACK_POINTER_LOC (0x5D)
#define ATMEGA128_STACK_POINTER_SIZE (0x02)

#define ATMEGA128_PC_SIZE (0x02)

#define ATMEGA128_RAMPD_LOC (ATMEGA128_DMEM_SIZE)

#define ATMEGA128_X_LOC (0x1A)
// if these do not exist in the AVR implementation, define them as
// (ATMEGA128_DMEM_SIZE)
#define ATMEGA128_RAMPX_LOC (ATMEGA128_DMEM_SIZE)
#define ATMEGA128_Y_LOC (0x1C)
#define ATMEGA128_RAMPY_LOC (ATMEGA128_DMEM_SIZE)
#define ATMEGA128_Z_LOC (0x1E)
#define ATMEGA128_RAMPZ_LOC (0x5B)
#define ATMEGA128_EIND_LOC (ATMEGA128_DMEM_SIZE)

#define ATMEGA128_UDR0_LOC (0x2C)
#define ATMEGA128_UCSR0A_LOC (0x2B)
#define ATMEGA128_UCSR0B_LOC (0x2A)
#define ATMEGA128_UCSR0C_LOC (0x95)

struct atmega128
{
    uint8_t dmem[ATMEGA128_DMEM_SIZE];
    struct avr_dmem_cb callbacks[ATMEGA128_IO_REGISTERS_LENGTH];
    struct avr_pmem_decoded decoded[ATMEGA128_PMEM_SIZE];
    uint16_t pmem[ATMEGA128_PMEM_SIZE];
    struct avr avr;
    void(*uart0_write_cb)(void*, uint8_t);
    void* uart0_write_cb_arg;
    uint8_t uart0_rx_fifo;
    unsigned char uart0_rx_fifo_state;
    uint8_t uart1_rx_fifo;
    unsigned char uart1_rx_fifo_state;
};

void atmega128_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data);
void atmega128_tick(struct atmega128 * const mega);
void atmega128_uart0_write(struct atmega128 * const mega,
                            const uint8_t value);
void atmega128_reinit(struct atmega128 * const mega);
void atmega128_init(struct atmega128 * const mega,
                     void(* const uart0_write_cb) (void*, uint8_t),
                     void* const uart0_write_cb_arg);
int atmega128_load_hex(struct atmega128 * const mega,
                       const char* const filename);

#endif

