/*
 avr.c A software implementation of the generic bits of an AVR microcontroller

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

#ifndef AVR_H
#define	AVR_H

#include <stdint.h>
#include <stdlib.h>

struct avr;

struct avr_instruction
{
    void(*function)(struct avr * const, uint16_t, uint16_t);
    uint16_t pattern;
    uint16_t mask;
    uint16_t(*get_arg0)(const uint16_t);
    uint16_t(*get_arg1)(const uint16_t);
    uint8_t length;
};

struct avr_pmem_decoded
{
    void(*function)(struct avr * const, uint16_t, uint16_t);
    uint16_t arg0;
    uint16_t arg1;
    uint8_t length;
};

struct avr_dmem_cb
{
    void(*write_callback)(void*, uint8_t);
    void* write_callback_arg;
    uint8_t(*read_callback)(void*, uint8_t);
    void* read_callback_arg;
};


struct avr_dmem
{
    uint8_t* mem; //[AVR_DMEM_SIZE];
    struct avr_dmem_cb* callbacks; //[AVR_IO_REGISTERS_LENGTH];
    size_t size; // in uint8_ts
    size_t io_resisters_start;
    size_t io_resisters_end;
    size_t io_resisters_length;
    size_t sreg_loc;
    size_t sp_loc;
    size_t x_loc;
    size_t y_loc;
    size_t z_loc;
    // if these do not exist in the AVR implementation, they should be set to
    // dmem_size
    size_t rampd_loc;
    size_t rampx_loc;
    size_t rampy_loc;
    size_t rampz_loc;
    size_t eind_loc;
    uint8_t sp_size;
};

struct avr_pmem
{
    struct avr_pmem_decoded* decoded; //[AVR_PMEM_SIZE];
    uint16_t* mem; //[AVR_PMEM_SIZE];
    size_t size; // in uint16_ts
    const struct avr_instruction * instructions;
    const struct avr_instruction * instructions_limit;
};

struct avr_callbacks
{
    void (*sleep)(void*, uint8_t);
    void* sleep_arg;
    uint32_t (*iv)(void*, uint32_t);
    void* iv_arg;
};

struct avr
{
    struct avr_callbacks callbacks;
    struct avr_dmem dmem;
    struct avr_pmem pmem;
    uint32_t pc;
    uint32_t interrupt_vector;
    uint8_t pc_size;
    uint8_t asleep;
    uint8_t interrupt_delay;
};

void avr_tick(struct avr* const avr);
void avr_init(struct avr* const avr, const struct avr_callbacks callbacks);
unsigned char avr_interrupt(struct avr* const avr, const uint32_t vector);
unsigned char avr_interrupt_nocheck(struct avr* const avr, const uint32_t vector);
void avr_dmem_write(struct avr_dmem * const dmem, const uint32_t address,
                    const uint8_t value);
uint8_t avr_dmem_read(const struct avr_dmem * const dmem, const uint32_t address);
void avr_pmem_write(struct avr_pmem * const pmem, const uint32_t address, const uint16_t value);
uint8_t avr_pmem_read_byte(const struct avr_pmem * const pmem, const uint32_t address);
void avr_pmem_write_byte(struct avr_pmem * const pmem, const uint32_t address,
                         const uint8_t value);

#endif
