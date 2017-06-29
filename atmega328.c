/*
 atmega28.c a wrapper for avr.c simulating an ATmega328

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

#include "atmega328.h"
#include "avr_sreg.h"

#include "jihex.h"

#include <string.h>

void atmega328_pmem_write_byte(void* const pmem, const uint32_t address,
                                const uint8_t data)
{
    avr_pmem_write_byte((struct avr_pmem*) pmem, address, data);
}

void atmega328_tick(struct atmega328* const mega)
{
    avr_tick(&mega->avr);
}

static void atmega328_set_rxc0(struct atmega328* const mega)
{
    mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] |= (1 << 7);
    if((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 7)) != 0)
    { // interrupt is enabled
        avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_RXC);
    }
}

static void atmega328_set_txc0(struct atmega328* const mega)
{
    if((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 6)) != 0)
    { // interrupt is enabled
        if (avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_TXC) == 0)
        { // if interupts are disabled globally
            mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] |= (1 << 6);
        }
    }
    else
    {
        mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] |= (1 << 6);
    }
}

static void atmega328_set_udre0(struct atmega328* const mega)
{
    if((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 5)) != 0)
    { // interrupt is enabled
        avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_UDRE);
    }
}

// this is called externally to write to the uart

void atmega328_uart0_write(struct atmega328 * const mega, const uint8_t value)
{
    if ((avr_dmem_read(&mega->avr.dmem, ATMEGA328_UCSR0B_LOC) & (1 << 4)) != 0)
    { // receive enabled
        switch (mega->uart0_rx_fifo_state)
        {
        case 0: // value into udr0
            mega->avr.dmem.mem[ATMEGA328_UDR0_LOC] = value;
            ++mega->uart0_rx_fifo_state;
            atmega328_set_rxc0(mega);
            break;
        case 1: // value goes into buffer
            mega->uart0_rx_fifo = value;
            mega->uart0_rx_errs = 0;
            ++mega->uart0_rx_fifo_state;
            break;
        default:
            // TODO: set DOR flag
            break;
        }
    }
}

static void atmega328_ucsr0a_write_cb(void* arg, uint8_t value)
{
    struct atmega328* const mega = (struct atmega328*) arg;
    // TXC0 is cleared by writing a 1, all other bits in this reg are automatic
    if ((value & (1 << 6)) != 0)
    {
       mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] &= ~(1 << 6);
    }
}

static void atmega328_ucsr0b_write_cb(void* arg, uint8_t value)
{
    struct atmega328* const mega = (struct atmega328*) arg;

    if ((value & (1 << 4)) == 0)
    { // RXEN cleared, flush rx fifo
        mega->uart0_rx_fifo_state = 0;
        // clear RXC
        mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] &= ~(1 << 7);
    }
    // check if interrupt enables are being turned on, if they are then trigger
    // interrupts if flags set
    if(((value & (1 << 7)) != 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 7)) == 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 7)) != 0))
    { // interrupt is enabled and flag is set
        avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_RXC);
    }
    else if(((value & (1 << 6)) != 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 6)) == 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 6)) != 0))
    { // interrupt is enabled and flag is set
        if (avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_TXC) != 0)
        {
            mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] &= ~(1 << 6);
        }
    }
    else if(((value & (1 << 5)) != 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 5)) == 0) &&
        ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 5)) != 0))
    { // interrupt is enabled and flag is set
        avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_UDRE);
    }

    mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] = value;
}

static uint8_t atmega328_udr0_read_cb(void* arg, uint8_t value)
{
    struct atmega328* const mega = (struct atmega328*) arg;
    if (mega->uart0_rx_fifo_state > 0)
    { // data is present
        if (mega->uart0_rx_fifo_state == 2)
        { // move fifo byte to udr0
            mega->avr.dmem.mem[ATMEGA328_UDR0_LOC] = mega->uart0_rx_fifo;
            // move the errors into ucsr0a
            mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] =
              (mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & 0xE3) |
              (mega->uart0_rx_errs & 0x1C);
            // trigger RXC interrupt
            avr_interrupt(&mega->avr, ATMEGA328_INT_VECT_USART0_RXC);
        }
        else
        {
            // clear RXC
            mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] &= ~(1 << 7);
        }
        --mega->uart0_rx_fifo_state;
    }
    return value;
}

static void atmega328_udr0_write_cb(void* arg, uint8_t value)
{
    struct atmega328* const mega = (struct atmega328*) arg;
    mega->callbacks.uart0(mega->callbacks.uart0_arg, value);
    // trigger interrupts
    atmega328_set_txc0(mega);
    atmega328_set_udre0(mega);
}

static void atmega328_sreg_write_cb(void* arg, uint8_t value)
{
    struct atmega328* const mega = (struct atmega328*) arg;
    // check if enabling interrupts
    if (((value & AVR_SREG_INTERRUPT_MASK) != 0) &&
       ((mega->avr.dmem.mem[ATMEGA328_SREG_LOC] & AVR_SREG_INTERRUPT_MASK) ==
       0))
    { // check all interrupts to see if flags set
        // interupts need to be delayed 1 instuction after this bit is set
        mega->avr.interrupt_delay = 1;
        if(((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 7)) != 0) &&
            ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 7)) != 0))
        { // interrupt is enabled and flag is set
            avr_interrupt_nocheck(&mega->avr, ATMEGA328_INT_VECT_USART0_RXC);
        }
        else if(((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 6)) != 0) &&
            ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 6)) != 0))
        { // interrupt is enabled and flag is set
            if (avr_interrupt_nocheck(&mega->avr, ATMEGA328_INT_VECT_USART0_TXC)
              != 0)
            {
                mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] &= ~(1 << 6);
            }
        }
        else if(((mega->avr.dmem.mem[ATMEGA328_UCSR0B_LOC] & (1 << 5)) != 0) &&
            ((mega->avr.dmem.mem[ATMEGA328_UCSR0A_LOC] & (1 << 5)) != 0))
        { // interrupt is enabled and flag is set
            avr_interrupt_nocheck(&mega->avr, ATMEGA328_INT_VECT_USART0_UDRE);
        }
    }
    mega->avr.dmem.mem[ATMEGA328_SREG_LOC] = value;
}

static void atmega328_init_regs(struct avr_dmem * const dmem)
{
    dmem->mem[ATMEGA328_UCSR0A_LOC] = 0x20;
    dmem->mem[ATMEGA328_UCSR0B_LOC] = 0x00;
    dmem->mem[ATMEGA328_UCSR0C_LOC] = 0x06;
}

static uint32_t atmega328_boot_reset_addr(struct atmega328 * const mega) {
    switch (mega->config.bootsz)
    {
    case 0:
        return 0x3800;
    case 1:
        return 0x3c00;
    case 2:
        return 0x3e00;
    case 3:
        return 0x3f00;
    }
    return 0;
}

static void atmega328_init_pc(struct atmega328 * const mega) {
    mega->avr.pc = (mega->config.bootrst != 0) ? 0 : atmega328_boot_reset_addr(mega);
}

void atmega328_reinit(struct atmega328 * const mega)
{
    struct avr* const avr = &(mega->avr);
    memset(avr->dmem.mem, 0, avr->dmem.size * sizeof (*avr->dmem.mem));
    memset(avr->pmem.decoded, 0, avr->pmem.size * sizeof (*avr->pmem.decoded));
    memset(avr->pmem.mem, 0, avr->pmem.size * sizeof (*avr->pmem.mem));

    atmega328_init_regs(&avr->dmem);
    atmega328_init_pc(mega);
}

static void atmega328_sleep_cb(void* arg, uint8_t sleep)
{
    struct atmega328* mega = (struct atmega328*) arg;
    if (((mega->avr.dmem.mem[ATMEGA328_SMCR_LOC] & (1 << 0)) != 0) &&
        (mega->callbacks.sleep != 0))
    {
        mega->callbacks.sleep(mega->callbacks.sleep_arg, sleep);
    }
}

static uint32_t atmega328_iv_cb(void* arg, uint32_t iv)
{
    struct atmega328* mega = (struct atmega328*) arg;
    if ((mega->avr.dmem.mem[ATMEGA328_MCUCR_LOC] & (1 << 1)) != 0)
    {
        iv += atmega328_boot_reset_addr(mega);
    }
    return iv;
}

void atmega328_init(struct atmega328 * const mega,
    const struct atmega328_callbacks callbacks,
    const struct atmega328_config config)
{
    mega->uart0_rx_fifo_state = 0;
    mega->uart1_rx_fifo_state = 0;

    mega->config = config;
    mega->callbacks = callbacks;

    struct avr* const avr = &(mega->avr);
    struct avr_dmem* const dmem = &(avr->dmem);
    struct avr_pmem* const pmem = &(avr->pmem);

    // give access to arrays
    dmem->mem = mega->dmem;
    dmem->callbacks = mega->dmem_callbacks;
    dmem->size = ATMEGA328_DMEM_SIZE;

    pmem->mem = mega->pmem;
    pmem->decoded = mega->decoded;
    pmem->size = ATMEGA328_PMEM_SIZE;

    // initialise the config variables
    avr->pc_size = ATMEGA328_PC_SIZE;

    dmem->io_resisters_start = ATMEGA328_IO_REGISTERS_START;
    dmem->io_resisters_end = ATMEGA328_IO_REGISTERS_END;
    dmem->io_resisters_length = ATMEGA328_IO_REGISTERS_LENGTH;
    dmem->sreg_loc = ATMEGA328_SREG_LOC;
    dmem->sp_loc = ATMEGA328_STACK_POINTER_LOC;
    dmem->sp_size = ATMEGA328_STACK_POINTER_SIZE;
    dmem->x_loc = ATMEGA328_X_LOC;
    dmem->y_loc = ATMEGA328_Y_LOC;
    dmem->z_loc = ATMEGA328_Z_LOC;
    dmem->rampx_loc = ATMEGA328_RAMPX_LOC;
    dmem->rampy_loc = ATMEGA328_RAMPY_LOC;
    dmem->rampz_loc = ATMEGA328_RAMPZ_LOC;
    dmem->rampd_loc = ATMEGA328_RAMPD_LOC;
    dmem->eind_loc = ATMEGA328_EIND_LOC;

    struct avr_callbacks avr_callbacks = {
        .sleep = &atmega328_sleep_cb,
        .sleep_arg = mega,
        .iv = &atmega328_iv_cb,
        .iv_arg = mega
    };

    avr_init(avr, avr_callbacks);
    atmega328_init_pc(mega);

    // initialise version specific callbacks
    dmem->callbacks[ATMEGA328_UDR0_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback =
            &atmega328_udr0_write_cb;
    dmem->callbacks[ATMEGA328_UDR0_LOC
            - ATMEGA328_IO_REGISTERS_START].write_callback_arg = mega;

    dmem->callbacks[ATMEGA328_UDR0_LOC -
            ATMEGA328_IO_REGISTERS_START].read_callback
            = &atmega328_udr0_read_cb;
    dmem->callbacks[ATMEGA328_UDR0_LOC -
            ATMEGA328_IO_REGISTERS_START].read_callback_arg = mega;

    dmem->callbacks[ATMEGA328_UCSR0A_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback
            = &atmega328_ucsr0a_write_cb;
    dmem->callbacks[ATMEGA328_UCSR0A_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback_arg = mega;

    dmem->callbacks[ATMEGA328_UCSR0B_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback
            = &atmega328_ucsr0b_write_cb;
    dmem->callbacks[ATMEGA328_UCSR0B_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback_arg = mega;

    dmem->callbacks[ATMEGA328_SREG_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback
            = &atmega328_sreg_write_cb;
    dmem->callbacks[ATMEGA328_SREG_LOC -
            ATMEGA328_IO_REGISTERS_START].write_callback_arg = mega;

    // initialise version specific registers
    atmega328_init_regs(dmem);
}

int atmega328_load_hex(struct atmega328 * const mega,
                        const char* const filename)
{
    return jihex_handle(filename, &atmega328_pmem_write_byte, &mega->avr.pmem);
}
