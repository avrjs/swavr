/*
 atmega128.c a wrapper for avr.c simulating an ATmega128
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

#include "atmega128.h"

#include "jihex.h"

#include <string.h>

void atmega128_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data)
{
    avr_pmem_write_byte((struct avr_pmem*) pmem, address, data);
}

void atmega128_tick(struct atmega128 * const mega)
{
    avr_tick(&mega->avr);
}

// this is called externally to write to the uart

void atmega128_uart0_write(struct atmega128 * const mega, const uint8_t value)
{
    switch (mega->uart0_rx_fifo_state)
    {
    case 0: // value into udr0
        mega->avr.dmem.mem[ATMEGA128_UDR0_LOC] = value;
        ++mega->uart0_rx_fifo_state;
        // set RXC0
        avr_dmem_write(&mega->avr.dmem, ATMEGA128_UCSR0A_LOC,
                       avr_dmem_read(&mega->avr.dmem,
                                     ATMEGA128_UCSR0A_LOC) | (1 << 7));
        break;
    case 1: // value goes into buffer
        mega->uart0_rx_fifo = value;
        ++mega->uart0_rx_fifo_state;
        break;
    }
}

void atmega128_ucsr0a_write_cb(void* arg, uint8_t value)
{ // this needs to check to see if there is data in the udr rx buffer and set
    // RXC if true
    struct atmega128* mega = (struct atmega128*) arg;
    if (mega->uart0_rx_fifo_state != 0)
    {
        value |= 1 << 7;
    }
    mega->avr.dmem.mem[ATMEGA128_UCSR0A_LOC] = value;
}

uint8_t atmega128_udr0_read_cb(void* arg, uint8_t value)
{
    struct atmega128* mega = (struct atmega128*) arg;
    if (mega->uart0_rx_fifo_state > 0)
    { // data is present
        if (mega->uart0_rx_fifo_state == 2)
        { // move fifo byte to udr0
            mega->avr.dmem.mem[ATMEGA128_UDR0_LOC] = mega->uart0_rx_fifo;
        }
        else
        {
            mega->avr.dmem.mem[ATMEGA128_UCSR0A_LOC] &= ~(1 << 7);
        }
        --mega->uart0_rx_fifo_state;
    }
    return value;
}

void atmega128_udr0_write_cb(void* arg, uint8_t value)
{
    struct atmega128* mega = (struct atmega128*) arg;
    mega->uart0_write_cb(mega->uart0_write_cb_arg, value);
    // set UDRE0 and TXC0
    mega->avr.dmem.mem[ATMEGA128_UCSR0A_LOC] |= 3 << 5;
}

void atmega128_init_regs(struct avr_dmem * const dmem)
{
    dmem->mem[ATMEGA128_UCSR0A_LOC] = 0x20;
    dmem->mem[ATMEGA128_UCSR0B_LOC] = 0x00;
    dmem->mem[ATMEGA128_UCSR0C_LOC] = 0x06;
}

void atmega128_reinit(struct atmega128 * const mega)
{
    memset(mega->avr.dmem.mem, 0, mega->avr.dmem.size *
           sizeof (*mega->avr.dmem.mem));
    memset(mega->avr.pmem.decoded, 0, mega->avr.pmem.size *
           sizeof (*mega->avr.pmem.decoded));
    memset(mega->avr.pmem.mem, 0, mega->avr.pmem.size *
           sizeof (*mega->avr.pmem.mem));

    atmega128_init_regs(&mega->avr.dmem);
    
    mega->avr.pc = 0;
}

void atmega128_init(struct atmega128 * const mega,
                     void(* const uart0_write_cb) (void*, uint8_t),
                     void* const uart0_write_cb_arg)
{
    mega->uart0_write_cb = uart0_write_cb;
    mega->uart0_write_cb_arg = uart0_write_cb_arg;
    mega->uart0_rx_fifo_state = 0;
    mega->uart1_rx_fifo_state = 0;

    struct avr * const avr = &(mega->avr);
    struct avr_dmem * const dmem = &(avr->dmem);
    struct avr_pmem * const pmem = &(avr->pmem);

    // give access to arrays
    dmem->mem = mega->dmem;
    dmem->callbacks = mega->callbacks;
    dmem->size = ATMEGA128_DMEM_SIZE;

    pmem->mem = mega->pmem;
    pmem->decoded = mega->decoded;
    pmem->size = ATMEGA128_PMEM_SIZE;

    // initialise the config variables
    avr->pc_size = ATMEGA128_PC_SIZE;
    
    dmem->io_resisters_start = ATMEGA128_IO_REGISTERS_START;
    dmem->io_resisters_end = ATMEGA128_IO_REGISTERS_END;
    dmem->io_resisters_length = ATMEGA128_IO_REGISTERS_LENGTH;
    dmem->sreg_loc = ATMEGA128_SREG_LOC;
    dmem->sp_loc = ATMEGA128_STACK_POINTER_LOC;
    dmem->sp_size = ATMEGA128_STACK_POINTER_SIZE;
    dmem->x_loc = ATMEGA128_X_LOC;
    dmem->y_loc = ATMEGA128_Y_LOC;
    dmem->z_loc = ATMEGA128_Z_LOC;
    dmem->rampx_loc = ATMEGA128_RAMPX_LOC;
    dmem->rampy_loc = ATMEGA128_RAMPY_LOC;
    dmem->rampz_loc = ATMEGA128_RAMPZ_LOC;
    dmem->rampd_loc = ATMEGA128_RAMPD_LOC;
    dmem->eind_loc = ATMEGA128_EIND_LOC;

    avr_init(avr);

    // initialise version specific callbacks
    dmem->callbacks[ATMEGA128_UDR0_LOC -
            ATMEGA128_IO_REGISTERS_START].write_callback =
            &atmega128_udr0_write_cb;
    dmem->callbacks[ATMEGA128_UDR0_LOC
            - ATMEGA128_IO_REGISTERS_START].write_callback_arg = mega;

    dmem->callbacks[ATMEGA128_UDR0_LOC -
            ATMEGA128_IO_REGISTERS_START].read_callback
            = &atmega128_udr0_read_cb;
    dmem->callbacks[ATMEGA128_UDR0_LOC -
            ATMEGA128_IO_REGISTERS_START].read_callback_arg = mega;

    dmem->callbacks[ATMEGA128_UCSR0A_LOC -
            ATMEGA128_IO_REGISTERS_START].write_callback
            = &atmega128_ucsr0a_write_cb;
    dmem->callbacks[ATMEGA128_UCSR0A_LOC -
            ATMEGA128_IO_REGISTERS_START].write_callback_arg = mega;

    // initialise version specific registers
    atmega128_init_regs(dmem);
}

int atmega128_load_hex(struct atmega128 * const mega,
                        const char* const filename)
{
    return jihex_handle(filename, &atmega128_pmem_write_byte, &mega->avr.pmem);
}
