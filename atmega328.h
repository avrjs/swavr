/*
 atmega328.h a wrapper for avr.c simulating an ATmega328

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

#ifndef ATMEGA328_H
#define	ATMEGA328_H

#include "avr.h"

#define ATMEGA328_PMEM_SIZE (0x4000) // in uint16_ts
#define ATMEGA328_DMEM_SIZE (0x800) // in uint8_ts

#define ATMEGA328_IO_REGISTERS_START (0x20)
#define ATMEGA328_IO_REGISTERS_END (0xFF)
#define ATMEGA328_IO_REGISTERS_LENGTH (ATMEGA328_IO_REGISTERS_END \
- ATMEGA328_IO_REGISTERS_START)

#define ATMEGA328_SREG_LOC (0x5F)

#define ATMEGA328_STACK_POINTER_LOC (0x5D)
#define ATMEGA328_STACK_POINTER_SIZE (0x02)

#define ATMEGA328_PC_SIZE (0x02)

#define ATMEGA328_RAMPD_LOC (ATMEGA328_DMEM_SIZE)

#define ATMEGA328_X_LOC (0x1A)
// if these do not exist in the AVR implementation, define them as
// (ATMEGA328_DMEM_SIZE)
#define ATMEGA328_RAMPX_LOC (ATMEGA328_DMEM_SIZE)
#define ATMEGA328_Y_LOC (0x1C)
#define ATMEGA328_RAMPY_LOC (ATMEGA328_DMEM_SIZE)
#define ATMEGA328_Z_LOC (0x1E)
#define ATMEGA328_RAMPZ_LOC (ATMEGA328_DMEM_SIZE)
#define ATMEGA328_EIND_LOC (ATMEGA328_DMEM_SIZE)

#define ATMEGA328_MCUCR_LOC (0x55)

#define ATMEGA328_UDR0_LOC (0xC6)
#define ATMEGA328_UCSR0A_LOC (0xC0)
#define ATMEGA328_UCSR0B_LOC (0xC1)
#define ATMEGA328_UCSR0C_LOC (0xC2)

#define ATMEGA328_INT_VECT_USART0_RXC (0x24)
#define ATMEGA328_INT_VECT_USART0_UDRE (0x26)
#define ATMEGA328_INT_VECT_USART0_TXC (0x28)

struct atmega328_callbacks
{
    void (*sleep)(void*, uint8_t);
    void* sleep_arg;
    void(*uart0)(void*, uint8_t);
    void* uart0_arg;
};

struct atmega328_config
{
    unsigned char bootsz;
    unsigned char bootrst;
};

struct atmega328
{
    struct atmega328_callbacks callbacks;
    struct atmega328_config config;
    uint8_t dmem[ATMEGA328_DMEM_SIZE];
    struct avr_dmem_cb dmem_callbacks[ATMEGA328_IO_REGISTERS_LENGTH];
    struct avr_pmem_decoded decoded[ATMEGA328_PMEM_SIZE];
    uint16_t pmem[ATMEGA328_PMEM_SIZE];
    struct avr avr;
    uint8_t uart0_rx_fifo;
    uint8_t uart0_rx_errs; // these are the errors that get shifted with the
    // data directly above
    unsigned char uart0_rx_fifo_state;
    uint8_t uart1_rx_fifo;
    uint8_t uart1_rx_errs;
    unsigned char uart1_rx_fifo_state;

};

void atmega328_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data);
void atmega328_tick(struct atmega328 * const mega);
void atmega328_uart0_write(struct atmega328 * const mega,
                            const uint8_t value);
void atmega328_reinit(struct atmega328 * const mega);
void atmega328_init(struct atmega328 * const mega,
    const struct atmega328_callbacks callbacks,
    const struct atmega328_config config);
int atmega328_load_hex(struct atmega328 * const mega,
                       const char* const filename);

#endif
