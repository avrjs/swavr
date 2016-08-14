/*
 attiny1634.c a wrapper for avr.c simulating an ATTINY1634

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

#include "attiny1634.h"
#include "avr_sreg.h"

#include "jihex.h"

#include <string.h>

void attiny1634_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data)
{
    avr_pmem_write_byte((struct avr_pmem*) pmem, address, data);
}

void attiny1634_tick(struct attiny1634 * const tiny)
{
    avr_tick(&tiny->avr);
}

void attiny1634_set_rxc0(struct attiny1634* const tiny)
{
    tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] |= (1 << 7);
    if((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 7)) != 0)
    { // interrupt is enabled
        avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_RXC);
    }
}

void attiny1634_set_txc0(struct attiny1634* const tiny)
{
    if((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 6)) != 0)
    { // interrupt is enabled
        if (avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_TXC) == 0)
        { // if interupts are disabled globally
            tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] |= (1 << 6);
        }
    }
    else
    {
        tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] |= (1 << 6);
    }
}

void attiny1634_set_udre0(struct attiny1634* const tiny)
{
    if((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 5)) != 0)
    { // interrupt is enabled
        avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_DRE);
    }
}

// this is called externally to write to the uart

void attiny1634_uart0_write(struct attiny1634 * const tiny, const uint8_t value)
{
    if ((avr_dmem_read(&tiny->avr.dmem, ATTINY1634_UCSR0B_LOC) & (1 << 4)) != 0)
    { // receive enabled
        switch (tiny->uart0_rx_fifo_state)
        {
        case 0: // value into udr0
            tiny->avr.dmem.mem[ATTINY1634_UDR0_LOC] = value;
            ++tiny->uart0_rx_fifo_state;
            attiny1634_set_rxc0(tiny);
            break;
        case 1: // value goes into buffer
            tiny->uart0_rx_fifo = value;
            tiny->uart0_rx_errs = 0;
            ++tiny->uart0_rx_fifo_state;
            break;
        default:
            // TODO: set DOR flag
            break;
        }
    }
}

void attiny1634_ucsr0a_write_cb(void* arg, uint8_t value)
{
    struct attiny1634* tiny = (struct attiny1634*) arg;
    // TXC0 is cleared by writing a 1, all other bits in this reg are automatic
    if ((value & (1 << 6)) != 0)
    {
       tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] &= ~(1 << 6);
    }
}

void attiny1634_ucsr0b_write_cb(void* arg, uint8_t value)
{
    struct attiny1634* const tiny = (struct attiny1634*) arg;

    if ((value & (1 << 4)) == 0)
    { // RXEN cleared, flush rx fifo
        tiny->uart0_rx_fifo_state = 0;
        // clear RXC
        tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] &= ~(1 << 7);
    }
    // check if interrupt enables are being turned on, if they are then trigger
    // interrupts if flags set
    if(((value & (1 << 7)) != 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 7)) == 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 7)) != 0))
    { // interrupt is enabled and flag is set
        avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_RXC);
    }
    else if(((value & (1 << 6)) != 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 6)) == 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 6)) != 0))
    { // interrupt is enabled and flag is set
        if (avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_TXC) != 0)
        {
            tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] &= ~(1 << 6);
        }
    }
    else if(((value & (1 << 5)) != 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 5)) == 0) &&
        ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 5)) != 0))
    { // interrupt is enabled and flag is set
        avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_DRE);
    }

    tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] = value;
}


uint8_t attiny1634_udr0_read_cb(void* arg, uint8_t value)
{
    struct attiny1634* tiny = (struct attiny1634*) arg;
    if (tiny->uart0_rx_fifo_state > 0)
    { // data is present
        if (tiny->uart0_rx_fifo_state == 2)
        { // move fifo byte to udr0
            tiny->avr.dmem.mem[ATTINY1634_UDR0_LOC] = tiny->uart0_rx_fifo;
            // move the errors into ucsr0a
            tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] =
              (tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & 0xE3) |
              (tiny->uart0_rx_errs & 0x1C);
            // trigger RXC interrupt
            avr_interrupt(&tiny->avr, ATTINY1634_INT_VECT_USART0_RXC);
        }
        else
        {
            // clear RXC
            tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] &= ~(1 << 7);
        }
        --tiny->uart0_rx_fifo_state;
    }
    return value;
}

void attiny1634_udr0_write_cb(void* arg, uint8_t value)
{
    struct attiny1634* tiny = (struct attiny1634*) arg;
    tiny->uart0_cb(tiny->uart0_cb_arg, value);
    // trigger interrupts
    attiny1634_set_txc0(tiny);
    attiny1634_set_udre0(tiny);
}

void attiny1634_sreg_write_cb(void* arg, uint8_t value)
{
    struct attiny1634* const tiny = (struct attiny1634*) arg;
    // check if enabling interrupts
    if (((value & AVR_SREG_INTERRUPT_MASK) != 0) &&
       ((tiny->avr.dmem.mem[ATTINY1634_SREG_LOC] & AVR_SREG_INTERRUPT_MASK) ==
       0))
    { // check all interrupts to see if flags set
        // interupts need to be delayed 1 instuction after this bit is set
        tiny->avr.interrupt_delay = 1;
        if(((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 7)) != 0) &&
            ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 7)) != 0))
        { // interrupt is enabled and flag is set
            avr_interrupt_nocheck(&tiny->avr, ATTINY1634_INT_VECT_USART0_RXC);
        }
        else if(((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 6)) != 0) &&
            ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 6)) != 0))
        { // interrupt is enabled and flag is set
            if (avr_interrupt_nocheck(&tiny->avr,
                ATTINY1634_INT_VECT_USART0_TXC) != 0)
            {
                tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] &= ~(1 << 6);
            }
        }
        else if(((tiny->avr.dmem.mem[ATTINY1634_UCSR0B_LOC] & (1 << 5)) != 0) &&
            ((tiny->avr.dmem.mem[ATTINY1634_UCSR0A_LOC] & (1 << 5)) != 0))
        { // interrupt is enabled and flag is set
            avr_interrupt_nocheck(&tiny->avr, ATTINY1634_INT_VECT_USART0_DRE);
        }
    }
    tiny->avr.dmem.mem[ATTINY1634_SREG_LOC] = value;
}

void attiny1634_init_regs(struct avr_dmem * const dmem)
{
    dmem->mem[ATTINY1634_UCSR0A_LOC] = 0x20;
    dmem->mem[ATTINY1634_UCSR0B_LOC] = 0x00;
    dmem->mem[ATTINY1634_UCSR0C_LOC] = 0x06;
    dmem->mem[ATTINY1634_UCSR0D_LOC] = 0x20;
}

void attiny1634_reinit(struct attiny1634 * const tiny)
{
    memset(tiny->avr.dmem.mem, 0, tiny->avr.dmem.size *
           sizeof (*tiny->avr.dmem.mem));
    memset(tiny->avr.pmem.decoded, 0, tiny->avr.pmem.size *
           sizeof (*tiny->avr.pmem.decoded));
    memset(tiny->avr.pmem.mem, 0, tiny->avr.pmem.size *
           sizeof (*tiny->avr.pmem.mem));

    attiny1634_init_regs(&tiny->avr.dmem);

    tiny->avr.pc = 0;
}

void attiny1634_sleep_cb(void* arg, uint8_t sleep)
{
    struct attiny1634* tiny = (struct attiny1634*) arg;
    if ((tiny->avr.dmem.mem[ATTINY1634_MCUCR_LOC] & (1 << 4)) != 0)
    {
        tiny->sleep_cb(tiny->sleep_cb_arg, sleep);
    }
}

void attiny1634_init(struct attiny1634 * const tiny)
{
    tiny->uart0_rx_fifo_state = 0;
    tiny->uart1_rx_fifo_state = 0;

    struct avr * const avr = &(tiny->avr);
    struct avr_dmem * const dmem = &(avr->dmem);
    struct avr_pmem * const pmem = &(avr->pmem);

    // give access to arrays
    dmem->mem = tiny->dmem;
    dmem->callbacks = tiny->callbacks;
    dmem->size = ATTINY1634_DMEM_SIZE;

    pmem->mem = tiny->pmem;
    pmem->decoded = tiny->decoded;
    pmem->size = ATTINY1634_PMEM_SIZE;

    // initialise the config variables
    avr->pc_size = ATTINY1634_PC_SIZE;

    dmem->io_resisters_start = ATTINY1634_IO_REGISTERS_START;
    dmem->io_resisters_end = ATTINY1634_IO_REGISTERS_END;
    dmem->io_resisters_length = ATTINY1634_IO_REGISTERS_LENGTH;
    dmem->sreg_loc = ATTINY1634_SREG_LOC;
    dmem->sp_loc = ATTINY1634_STACK_POINTER_LOC;
    dmem->sp_size = ATTINY1634_STACK_POINTER_SIZE;
    dmem->x_loc = ATTINY1634_X_LOC;
    dmem->y_loc = ATTINY1634_Y_LOC;
    dmem->z_loc = ATTINY1634_Z_LOC;
    dmem->rampx_loc = ATTINY1634_RAMPX_LOC;
    dmem->rampy_loc = ATTINY1634_RAMPY_LOC;
    dmem->rampz_loc = ATTINY1634_RAMPZ_LOC;
    dmem->rampd_loc = ATTINY1634_RAMPD_LOC;
    dmem->eind_loc = ATTINY1634_EIND_LOC;

    avr_init(avr);

    avr->sleep_cb = &attiny1634_sleep_cb;
    avr->sleep_cb_arg = tiny;

    // initialise version specific callbacks
    dmem->callbacks[ATTINY1634_UDR0_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback =
            &attiny1634_udr0_write_cb;
    dmem->callbacks[ATTINY1634_UDR0_LOC
            - ATTINY1634_IO_REGISTERS_START].write_callback_arg = tiny;

    dmem->callbacks[ATTINY1634_UDR0_LOC -
            ATTINY1634_IO_REGISTERS_START].read_callback
            = &attiny1634_udr0_read_cb;
    dmem->callbacks[ATTINY1634_UDR0_LOC -
            ATTINY1634_IO_REGISTERS_START].read_callback_arg = tiny;

    dmem->callbacks[ATTINY1634_UCSR0A_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback
            = &attiny1634_ucsr0a_write_cb;
    dmem->callbacks[ATTINY1634_UCSR0A_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback_arg = tiny;

    dmem->callbacks[ATTINY1634_UCSR0B_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback
            = &attiny1634_ucsr0b_write_cb;
    dmem->callbacks[ATTINY1634_UCSR0B_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback_arg = tiny;

    dmem->callbacks[ATTINY1634_SREG_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback
            = &attiny1634_sreg_write_cb;
    dmem->callbacks[ATTINY1634_SREG_LOC -
            ATTINY1634_IO_REGISTERS_START].write_callback_arg = tiny;

    // initialise version specific registers
    attiny1634_init_regs(dmem);
}

int attiny1634_load_hex(struct attiny1634 * const tiny,
                        const char* const filename)
{
    return jihex_handle(filename, &attiny1634_pmem_write_byte, &tiny->avr.pmem);
}
