/*
 attiny1634.h a wrapper for avr.c simulating an ATmega128

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

#ifndef ATTINY1634_H
#define	ATTINY1634_H

#include "avr.h"

#define ATTINY1634_PMEM_SIZE (0x2000) // in uint16_ts
#define ATTINY1634_DMEM_SIZE (0x0500) // in uint8_ts

#define ATTINY1634_IO_REGISTERS_START (0x20)
#define ATTINY1634_IO_REGISTERS_END (0xFF)
#define ATTINY1634_IO_REGISTERS_LENGTH (ATTINY1634_IO_REGISTERS_END \
- ATTINY1634_IO_REGISTERS_START)

#define ATTINY1634_SREG_LOC (0x5F)

#define ATTINY1634_STACK_POINTER_LOC (0x5D)
#define ATTINY1634_STACK_POINTER_SIZE (0x02)

#define ATTINY1634_PC_SIZE (0x02)

#define ATTINY1634_RAMPD_LOC (ATTINY1634_DMEM_SIZE)

#define ATTINY1634_X_LOC (0x1A)
// if these do not exist in the AVR implementation, define them as
// (ATTINY1634_DMEM_SIZE)
#define ATTINY1634_RAMPX_LOC (ATTINY1634_DMEM_SIZE)
#define ATTINY1634_Y_LOC (0x1C)
#define ATTINY1634_RAMPY_LOC (ATTINY1634_DMEM_SIZE)
#define ATTINY1634_Z_LOC (0x1E)
#define ATTINY1634_RAMPZ_LOC (ATTINY1634_DMEM_SIZE)
#define ATTINY1634_EIND_LOC (ATTINY1634_DMEM_SIZE)

#define ATTINY1634_MCUCR_LOC (0x56)

#define ATTINY1634_UDR0_LOC (0x40)
#define ATTINY1634_UCSR0A_LOC (0x46)
#define ATTINY1634_UCSR0B_LOC (0x45)
#define ATTINY1634_UCSR0C_LOC (0x44)
#define ATTINY1634_UCSR0D_LOC (0x43)

#define ATTINY1634_INT_VECT_USART0_RXC (0x20)
#define ATTINY1634_INT_VECT_USART0_DRE (0x22)
#define ATTINY1634_INT_VECT_USART0_TXC (0x24)


struct attiny1634
{
    void (*sleep_cb)(void*, uint8_t);
    void* sleep_cb_arg;
    uint8_t dmem[ATTINY1634_DMEM_SIZE];
    struct avr_dmem_cb callbacks[ATTINY1634_IO_REGISTERS_LENGTH];
    struct avr_pmem_decoded decoded[ATTINY1634_PMEM_SIZE];
    uint16_t pmem[ATTINY1634_PMEM_SIZE];
    struct avr avr;
    void(*uart0_cb)(void*, uint8_t);
    void* uart0_cb_arg;
    uint8_t uart0_rx_fifo;
    uint8_t uart0_rx_errs; // these are the errors that get shifted with the
    // data directly above
    unsigned char uart0_rx_fifo_state;
    uint8_t uart1_rx_fifo;
    uint8_t uart1_rx_errs;
    unsigned char uart1_rx_fifo_state;
};

void attiny1634_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data);
void attiny1634_tick(struct attiny1634 * const tiny);
void attiny1634_uart0_write(struct attiny1634 * const tiny,
                            const uint8_t value);
void attiny1634_reinit(struct attiny1634 * const tiny);
void attiny1634_init(struct attiny1634 * const tiny);
int attiny1634_load_hex(struct attiny1634 * const core,
                        const char* const filename);

#endif
