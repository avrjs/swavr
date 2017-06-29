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

#include "avr.h"
#include "avr_sreg.h"

#include "attiny1634.h"

#include <string.h>

#include <stdarg.h>
#include <stdio.h>

#ifndef DEBUG
#define DEBUG (0)
#endif

int avrjs_debug_printf(const char *const fmt, ...) {
  int res = 0;
  if (DEBUG != 0) {
    va_list args;
    va_start(args, fmt);
    res = vfprintf(stdout, fmt, args);
    va_end(args);
  }
  return res;
}

void avr_dmem_write(struct avr_dmem * const dmem, const uint32_t address,
                    uint8_t value)
{
    const uint16_t io_address = address - dmem->io_resisters_start;
    if ((address >= dmem->io_resisters_start)
            && (address < dmem->io_resisters_end)
            && (dmem->callbacks[io_address].write_callback != 0))
    { // callback, note: the callbacks do not change the value by default
        dmem->callbacks[io_address].write_callback(dmem->callbacks[io_address]
                                                   .write_callback_arg, value);
    }
    else
    {
        dmem->mem[address] = value;
    }
}

uint8_t avr_dmem_read(const struct avr_dmem * const dmem,
    const uint32_t address)
{
    uint8_t value = dmem->mem[address];
    const uint16_t io_address = address - dmem->io_resisters_start;
    if ((address >= dmem->io_resisters_start)
            && (address < dmem->io_resisters_end)
            && (dmem->callbacks[io_address].read_callback != 0))
    {
        value = dmem->callbacks[io_address]
                .read_callback(dmem->callbacks[io_address]
                               .read_callback_arg, value);
    }
    return value;
}

struct avr_pmem_decoded avr_pmem_decode(const struct
                                        avr_instruction *
                                        instruction,
                                        const struct
                                        avr_instruction * const
                                        instructions_limit,
                                        const uint16_t value)
{
    while ((instruction != instructions_limit) && ((value & instruction->mask)
            != instruction->pattern))
    {
        ++instruction;
    }

    struct avr_pmem_decoded decoded;
    if (instruction == instructions_limit)
    { // not found
        decoded.function = 0;
        decoded.arg0 = 0;
        decoded.arg1 = 0;
        decoded.length = 1;
    }
    else
    {
        decoded.function = instruction->function;
        decoded.arg0 = instruction->get_arg0(value);
        decoded.arg1 = instruction->get_arg1(value);
        decoded.length = instruction->length;
    }
    return decoded;
}

void avr_pmem_write(struct avr_pmem * const pmem, const uint32_t address,
                    const uint16_t value)
{
    pmem->mem[address] = value;
    pmem->decoded[address]
            = avr_pmem_decode(pmem->instructions, pmem->instructions_limit,
                              value);
}

void avr_pmem_write_byte(struct avr_pmem * const pmem, const uint32_t address,
        const uint8_t value)
{
    const uint32_t w_address = address >> 1;
    uint16_t w = pmem->mem[w_address];

    if ((address % 2) != 0)
    {
        w &= 0x00FF;
        w |= value << 8;
    }
    else
    {
        w &= 0xFF00;
        w |= value;
    }

    avr_pmem_write(pmem, w_address, w);
}

uint8_t avr_pmem_read_byte(const struct avr_pmem * const pmem,
                           const uint32_t address)
{
    return ((address % 2) != 0) ? (uint8_t) (pmem->mem[address / 2] >> 8) & 0xFF
            : (uint8_t) pmem->mem[address / 2] & 0xFF;
}

uint16_t avr_x_read(const struct avr_dmem * const dmem)
{
    uint16_t X = avr_dmem_read(dmem, dmem->x_loc);
    if (dmem->size > 0x100)
    {
        X |= (((uint16_t) avr_dmem_read(dmem, dmem->x_loc + 1)) << 8);
    }
    return X;
}

uint32_t avr_rampx_x_read(const struct avr_dmem * const dmem)
{
    uint32_t X = avr_x_read(dmem);
    if (dmem->rampx_loc < dmem->size)
    {
        X |= ((uint32_t) avr_dmem_read(dmem, dmem->rampx_loc)) << 16;
    }
    return X;
}

void avr_x_write(struct avr_dmem * const dmem, const uint16_t value)
{
    avr_dmem_write(dmem, dmem->x_loc, value & 0xFF);
    if (dmem->size > 0x100)
    {
        avr_dmem_write(dmem, dmem->x_loc + 1, (value >> 8) & 0xFF);
    }
}

void avr_rampx_x_write(struct avr_dmem * const dmem, const uint32_t value)
{
    avr_x_write(dmem, (uint16_t) (value & 0xFFFF));
    if (dmem->rampx_loc < dmem->size)
    {
        avr_dmem_write(dmem, dmem->rampx_loc, (value >> 16) & 0xFF);
    }
}

uint16_t avr_y_read(const struct avr_dmem * const dmem)
{
    uint16_t Y = ((uint16_t) avr_dmem_read(dmem, dmem->y_loc));
    if (dmem->size > 0x100)
    {
        Y |= (((uint16_t) avr_dmem_read(dmem, dmem->y_loc + 1)) << 8);
    }
    return Y;
}

uint32_t avr_rampy_y_read(const struct avr_dmem * const dmem)
{
    uint32_t Y = avr_y_read(dmem);
    if (dmem->rampy_loc < dmem->size)
    {
        Y |= ((uint32_t) avr_dmem_read(dmem, dmem->rampy_loc)) << 16;
    }
    return Y;
}

void avr_y_write(struct avr_dmem * const dmem, const uint16_t value)
{
    avr_dmem_write(dmem, dmem->y_loc, value & 0xFF);
    if (dmem->size > 0x100)
    {
        avr_dmem_write(dmem, dmem->y_loc + 1, (value >> 8) & 0xFF);
    }
}

void avr_rampy_y_write(struct avr_dmem * const dmem, const uint32_t value)
{
    avr_y_write(dmem, (uint16_t) (value & 0xFFFF));
    if (dmem->rampy_loc < dmem->size)
    {
        avr_dmem_write(dmem, dmem->rampy_loc, (value >> 16) & 0xFF);
    }
}

uint16_t avr_z_read(const struct avr_dmem * const dmem)
{
    uint16_t Z = avr_dmem_read(dmem, dmem->z_loc);
    if (dmem->size > 0x100)
    {
        Z |= (((uint16_t) avr_dmem_read(dmem, dmem->z_loc + 1)) << 8);
    }
    return Z;
}

uint32_t avr_rampz_z_read(const struct avr_dmem * const dmem)
{
    uint32_t Z = avr_z_read(dmem);
    if (dmem->rampz_loc < dmem->size)
    {
        Z |= ((uint32_t) avr_dmem_read(dmem, dmem->rampz_loc)) << 16;
    }
    return Z;
}

void avr_z_write(struct avr_dmem * const dmem, const uint16_t value)
{
    avr_dmem_write(dmem, dmem->z_loc, value & 0xFF);
    if (dmem->size > 0x100)
    {
        avr_dmem_write(dmem, dmem->z_loc + 1, (value >> 8) & 0xFF);
    }
}

void avr_rampz_z_write(struct avr_dmem * const dmem, const uint32_t value)
{
    avr_z_write(dmem, (uint16_t) (value & 0xFFFF));
    if (dmem->rampz_loc < dmem->size)
    {
        avr_dmem_write(dmem, dmem->rampz_loc, (value >> 16) & 0xFF);
    }
}

uint32_t avr_sp_read(const struct avr_dmem * const dmem)
{
    uint32_t sp = avr_dmem_read(dmem, dmem->sp_loc);
    uint8_t i = 1;
    while (i < dmem->sp_size)
    {
        sp |= ((uint32_t) avr_dmem_read(dmem, dmem->sp_loc + i)) << (i << 3);
        ++i;
    }
    return sp;
}

void avr_sp_write(struct avr_dmem * const dmem, const uint32_t value)
{
    uint8_t i = 0;
    while (i < dmem->sp_size)
    {
        avr_dmem_write(dmem, dmem->sp_loc + i, (uint8_t) ((value >> (i << 3)) &
                       0xFF));
        ++i;
    }
}

uint8_t avr_pop(struct avr_dmem * const dmem)
{
    const uint32_t sp = avr_sp_read(dmem) + 1;
    avr_sp_write(dmem, sp);

    return avr_dmem_read(dmem, sp);
}

void avr_push(struct avr_dmem * const dmem, const uint8_t value)
{
    const uint32_t sp = avr_sp_read(dmem);
    avr_dmem_write(dmem, sp, value);
    avr_sp_write(dmem, sp - 1);
}

void avr_skip_next_instruction(struct avr * const avr)
{
    avr->pc += 1 + avr->pmem.decoded[avr->pc + 1].length;
}

unsigned char avr_interrupt_nocheck(struct avr* const avr,
    const uint32_t vector)
{
    if ((avr->asleep != 0) && (avr->callbacks.sleep != 0))
    {
        avr->callbacks.sleep(avr->callbacks.sleep_arg, 0);
        avr->asleep = 0;
    }
    if (avr->interrupt_vector == 0)
    { // no interrupt currently pending
        avr->interrupt_vector = vector;
        return 1;
    }
    return 0;
}

unsigned char avr_interrupt(struct avr* const avr, const uint32_t vector)
{
    avrjs_debug_printf("* Interrupt *");
    if (avr_sreg_read_bit(&avr->dmem, AVR_SREG_INTERRUPT_BIT) != 0)
    { // global interrupt enabled
        return avr_interrupt_nocheck(avr, vector);
    }
    return 0;
}

// push PC to stack and clear I flag in sreg
void avr_interrupt_call(struct avr* const avr)
{
    uint32_t pc_ret = avr->pc;
    uint8_t i;
    for (i = avr->pc_size - 1; i < avr->pc_size; --i)
    {
        avr_push(&(avr->dmem), pc_ret >> (i << 3));
    }
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_INTERRUPT_BIT);

    avrjs_debug_printf("* Interrupt_CB *");
    avr->pc = avr->callbacks.iv(avr->callbacks.iv_arg, avr->interrupt_vector);
    avr->interrupt_vector = 0;
}

uint16_t x0000(const uint16_t instruction)
{
    (void) instruction;
    return 0;
}

uint16_t x01F0(const uint16_t instruction)
{
    return (instruction & 0x01F0) >> 4;
}

uint16_t x020F(const uint16_t instruction)
{
    return (instruction & 0x000F) | (instruction & 0x0200) >> 5;
}

uint16_t x0030(const uint16_t instruction)
{
    return (instruction & 0x0030) >> 4;
}

uint16_t x00CF(const uint16_t instruction)
{
    return (instruction & 0x000F) | ((instruction & 0x00C0) >> 2);
}

uint16_t x0F0F(const uint16_t instruction)
{
    return (instruction & 0x000F) | ((instruction & 0x0F00) >> 4);
}

uint16_t x00F0(const uint16_t instruction)
{
    return (instruction & 0x00F0) >> 4;
}

uint16_t x0070(const uint16_t instruction)
{
    return (instruction & 0x0070) >> 4;
}

uint16_t x0007(const uint16_t instruction)
{
    return instruction & 0x0007;
}

uint16_t x03F8(const uint16_t instruction)
{
    return (instruction & 0x03F8) >> 3;
}

uint16_t x01F1(const uint16_t instruction)
{
    return ((instruction & 0x01F0) >> 3) | (instruction & 0x0001);
}

uint16_t x00F8(const uint16_t instruction)
{
    return (instruction & 0x00F8) >> 3;
}

uint16_t x03FF(const uint16_t instruction)
{
    return instruction & 0x03FF;
}

uint16_t x060F(const uint16_t instruction)
{
    return (instruction & 0x000F) | ((instruction & 0x0600) >> 5);
}

uint16_t x2C07(const uint16_t instruction)
{
    return (instruction & 0x0007) | ((instruction & 0x0C00) >> 7)
            | ((instruction & 0x2000) >> 8);
}

uint16_t x070F(const uint16_t instruction)
{
    return (instruction & 0x000F) | ((instruction & 0x0700) >> 4);
}

uint16_t x000F(const uint16_t instruction)
{
    return instruction & 0x000F;
}

uint16_t x0FFF(const uint16_t instruction)
{
    return instruction & 0x0FFF;
}

void avr_ins_nop(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) avr;
    (void) arg0;
    (void) arg1;
    avrjs_debug_printf("nop ");

    ++avr->pc;
}

void avr_ins_movw(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{

    arg0 <<= 1;
    arg1 <<= 1;
    uint8_t vall = avr_dmem_read(&(avr->dmem), (uint32_t) arg1);
    uint8_t valh = avr_dmem_read(&(avr->dmem), (uint32_t) arg1 + 1);

    uint16_t val = vall | (((uint16_t)valh) << 8);
    avrjs_debug_printf("movw\t%02x,\t%02x:\t%u ", arg0, arg1,
        val);

    avr_dmem_write(&(avr->dmem), (uint32_t) arg0, vall);
    avr_dmem_write(&(avr->dmem), (uint32_t) arg0 + 1, valh);

    ++avr->pc;
}

void avr_ins_muls(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const int8_t Rds = (Rd & 0x7F) - (Rd & 0x80);
    const int8_t Rrs = (Rr & 0x7F) - (Rr & 0x80);

    const int16_t R = ((int16_t) Rds) * ((int16_t) Rrs);

    avrjs_debug_printf("muls\t%02x,\t%02x:\t%d * %d = %d ", arg0,
        arg1, Rds, Rrs, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

void avr_ins_mulsu(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const int8_t Rds = (Rd & 0x7F) - (Rd & 0x80);

    int16_t R = ((int16_t) Rds) * ((uint16_t) Rr);

    avrjs_debug_printf("mulsu\t%02x,\t%02x:\t%d * %u = %d ", arg0,
        arg1, Rds, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

void avr_ins_fmul(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint16_t R = (((uint16_t) Rd) * ((uint16_t) Rr)) << 1;

    avrjs_debug_printf("fmul\t%02x,\t%02x:\t(%u * %u) << 1 = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

void avr_ins_fmuls(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const int8_t Rds = (Rd & 0x7F) - (Rd & 0x80);
    const int8_t Rrs = (Rr & 0x7F) - (Rr & 0x80);

    const int16_t R = (((int16_t) Rds) * ((int16_t) Rrs)) << 1;

    avrjs_debug_printf("fmuls\t%02x,\t%02x:\t(%d * %d) << 1 = %d ",
        arg0, arg1, Rds, Rrs, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

void avr_ins_fmulsu(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const int8_t Rds = (Rd & 0x7F) - (Rd & 0x80);

    int16_t R = (((int16_t) Rds) * ((uint16_t) Rr)) << 1;

    avrjs_debug_printf("fmulsu\t%02x,\t%02x:\t(%d * %u) << 1 = %d ",
        avr->pc, arg0, arg1, Rds, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

void avr_ins_cpc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const uint8_t C = avr_sreg_read_bit(&(avr->dmem), AVR_SREG_CARRY_BIT);
    const uint8_t R = (Rd - Rr) - C;

    avrjs_debug_printf("cpc\t%02x,\t%02x:\t(%u - %u) - %u = %u ",
        arg0, arg1, Rd, Rr, C, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, Rr, R);
    avr_sreg_zero3(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, Rr, R);

    ++avr->pc;
}

void avr_ins_sbc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const uint8_t C = avr_sreg_read_bit(&(avr->dmem), AVR_SREG_CARRY_BIT);
    const uint8_t R = (Rd - Rr) - C;

    avrjs_debug_printf("sbc\t%02x,\t%02x:\t(%u - %u) - %u = %u ",
        arg0, arg1, Rd, Rr, C, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, Rr, R);
    avr_sreg_zero3(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, Rr, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_add(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = ((Rd + Rr) & 0xFF);

    avrjs_debug_printf("add\t%02x,\t%02x:\t%u + %u = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry1(sreg, Rd, Rr, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow1(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry1(sreg, Rd, Rr, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_cpse(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    avrjs_debug_printf("cpse\t%02x,\t%02x:\t%u == %u ",
        arg0, arg1, Rd, Rr);

    if (Rd == Rr)
    {
        avr_skip_next_instruction(avr);
    }
    else
    {
        ++avr->pc;
    }
}

void avr_ins_cp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = Rd - Rr;

    avrjs_debug_printf("cp\t%02x,\t%02x:\t%u - %u = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, Rr, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, Rr, R);

    ++avr->pc;
}

void avr_ins_sub(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = Rd - Rr;

    avrjs_debug_printf("sub\t%02x,\t%02x:\t%u - %u = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, Rr, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, Rr, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_adc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);
    const uint8_t C = avr_sreg_read_bit(&(avr->dmem), AVR_SREG_CARRY_BIT);
    const uint8_t R = Rd + Rr + C;

    avrjs_debug_printf("adc\t%02x,\t%02x:\t%u + %u + %u = %u ",
        arg0, arg1, Rd, Rr, C, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry1(sreg, Rd, Rr, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow1(sreg, Rd, Rr, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry1(sreg, Rd, Rr, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_and(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = Rd & Rr;

    avrjs_debug_printf("and\t%02x,\t%02x:\t%u & %u = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_eor(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = Rd ^ Rr;

    avrjs_debug_printf("eor\t%02x,\t%02x:\t%u ^ %u = %u ",
        arg0, arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_or(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint8_t R = Rd | Rr;

    avrjs_debug_printf("or\t%02x,\t%02x:\t%u | %u = %u ", arg0,
        arg1, Rd, Rr, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_mov(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1);

    avrjs_debug_printf("mov\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_cpi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 += 16;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd - arg1;

    avrjs_debug_printf("cpi\t%02x,\t%02x:\t%u - %u = %u ",
        arg0 - 16, arg1, Rd, arg1, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, arg1, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, arg1, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, arg1, R);

    ++avr->pc;
}

void avr_ins_sbci(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 += 16;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t C = avr_sreg_read_bit(&(avr->dmem), AVR_SREG_CARRY_BIT);
    const uint8_t R = (Rd - arg1) - C;

    avrjs_debug_printf("sbci\t%02x,\t%02x:\t(%u - %u) - %u = %u ",
        arg0 - 16, arg1, Rd, arg1, C, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, arg1, R);
    avr_sreg_zero3(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, arg1, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, arg1, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_subi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 += 16;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd - arg1;

    avrjs_debug_printf("subi\t%02x,\t%02x:\t%u - %u = %u ",
        arg0 - 16, arg1, Rd, arg1,  R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry4(sreg, Rd, arg1, R);
    avr_sreg_zero3(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow4(sreg, Rd, arg1, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry2(sreg, Rd, arg1, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_ori(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 += 16;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd | arg1;

    avrjs_debug_printf("ori\t%02x,\t%02x:\t%u | %u = %u ",
        arg0 - 16, arg1, Rd, arg1,  R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_andi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 += 16;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd & arg1;

    avrjs_debug_printf("andi\t%02x,\t%02x:\t%u & %u = %u ",
        arg0 - 16, arg1, Rd, arg1,  R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_ld_zpq(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), Z + arg1);

    avrjs_debug_printf("ld\t%02x,\t(Z + %02x):\t*%04x -> %u ",
        avr->pc, arg0, arg1, Z + arg1, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_ld_ypq(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), Y + arg1);

    avrjs_debug_printf("ld\t%02x,\t(Y + %02x):\t*%04x -> %u ",
        avr->pc, arg0, arg1, Y + arg1, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_st_zpq(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\t(Z + %02x):\t%u -> *%04x ",
        arg0, arg1, val, Z + arg1);

    avr_dmem_write(&(avr->dmem),  Z + arg1, val);

    ++avr->pc;
}

void avr_ins_st_ypq(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\t(Y + %02x):\t%u -> *%04x ",
        arg0, arg1, val, Y + arg1);

    avr_dmem_write(&(avr->dmem), Y + arg1, val);

    ++avr->pc;
}

void avr_ins_lds(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;

    uint32_t addr = avr->pmem.mem[avr->pc + 1];
    if (avr->dmem.rampd_loc < avr->dmem.size)
    {
        addr |= (((uint32_t) avr_dmem_read(&(avr->dmem), avr->dmem.rampd_loc))
                << 16);
    }
    const uint8_t val = avr_dmem_read(&(avr->dmem), addr);

    avrjs_debug_printf("lds\t%02x,\t%04x:\t%u -> *%04x ",
        arg0, addr, val, addr);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr->pc += 2;
}

void avr_ins_ld_zp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), Z);

    avrjs_debug_printf("ld\t%02x,\tZ+:\t*%04x -> %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampz_z_write(&(avr->dmem), Z + 1);

    ++avr->pc;
}

void avr_ins_ld_mz(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), Z);

    avrjs_debug_printf("ld\t%02x,\t-Z:\t*%04x -> %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampz_z_write(&(avr->dmem), Z);

    ++avr->pc;
}

void avr_ins_lpm_z(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("lpm\t%02x,\tZ:\t*%04x = %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_lpm_zp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint16_t Z = avr_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("lpm\t%02x,\tZ+:\t*%04x = %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_z_write(&(avr->dmem), Z + 1);

    ++avr->pc;
}

void avr_ins_elpm_z(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("elpm\t%02x,\tZ:\t*%04x = %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_elpm_zp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("elpm\t%02x,\tZ+:\t*%04x = %u ", arg0, Z,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampz_z_write(&(avr->dmem), Z + 1);

    ++avr->pc;
}

void avr_ins_ld_yp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), Y);

    avrjs_debug_printf("ld\t%02x,\tY+:\t*%04x -> %u ", arg0, Y,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampy_y_write(&(avr->dmem), Y + 1);

    ++avr->pc;
}

void avr_ins_ld_my(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), Y);

    avrjs_debug_printf("ld\t%02x,\t-Y:\t*%04x -> %u ", arg0, Y,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampy_y_write(&(avr->dmem), Y);

    ++avr->pc;
}

void avr_ins_ld_x(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), X);

    avrjs_debug_printf("ld\t%02x,\tX:\t*%04x -> %u ", arg0, X,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_ld_xp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), X);

    avrjs_debug_printf("ld\t%02x,\tX+:\t*%04x -> %u ", arg0, X,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampx_x_write(&(avr->dmem), X + 1);

    ++avr->pc;
}

void avr_ins_ld_mx(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), X);

    avrjs_debug_printf("ld\t%02x,\t-X:\t*%04x -> %u ", arg0, X,
        val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    avr_rampx_x_write(&(avr->dmem), X);

    ++avr->pc;
}

void avr_ins_pop(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t val = avr_pop(&(avr->dmem));

    avrjs_debug_printf("pop\t%02x:\t%u ", arg0, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_sts(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);
    uint32_t addr = avr->pmem.mem[avr->pc + 1];
    if (avr->dmem.rampd_loc < avr->dmem.size)
    {
        addr |= (((uint32_t) avr_dmem_read(&(avr->dmem), avr->dmem.rampd_loc))
                << 16);
    }

    avrjs_debug_printf("sts\t%02x:\t%u -> *%04x ", arg0, val, addr);

    avr_dmem_write(&(avr->dmem), addr, val);

    avr->pc += 2;
}

void avr_ins_st_zp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\tZ+:\t%u -> *%04x ", arg0, val, Z);

    avr_dmem_write(&(avr->dmem), Z, val);

    avr_rampz_z_write(&(avr->dmem), Z + 1);

    ++avr->pc;
}

void avr_ins_st_mz(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\t-Z:\t%u -> *%04x ", arg0, val, Z);

    avr_dmem_write(&(avr->dmem), Z, val);

    avr_rampz_z_write(&(avr->dmem), Z);

    ++avr->pc;
}

void avr_ins_xch(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rm = avr_dmem_read(&(avr->dmem), Z);

    avrjs_debug_printf("xch\t%02x:\t%u -> *%04x, %u ",
		       arg0, Rr, Z, Rm);

    avr_dmem_write(&(avr->dmem), arg0, Rm);
    avr_dmem_write(&(avr->dmem), Z, Rr);

    ++avr->pc;
}

void avr_ins_las(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t Zv = avr_dmem_read(&(avr->dmem), Z);
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0) | Zv;

    avrjs_debug_printf("las\t%02x:\t%u -> *%04x, %u ",
		       arg0, val, Z, Zv);

    avr_dmem_write(&(avr->dmem), Z, val);
    avr_dmem_write(&(avr->dmem), arg0, Zv);

    ++avr->pc;
}

void avr_ins_lac(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t Zv = avr_dmem_read(&(avr->dmem), Z);
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0) & ~Zv;

    avrjs_debug_printf("lac\t%02x:\t%u -> *%04x, %u ",
		       arg0, val, Z, Zv);

    avr_dmem_write(&(avr->dmem), Z, val);
    avr_dmem_write(&(avr->dmem), arg0, Zv);

    ++avr->pc;
}

void avr_ins_lat(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t Zv = avr_dmem_read(&(avr->dmem), Z);
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0) ^ Zv;

    avrjs_debug_printf("lat\t%02x:\t%u -> *%04x, %u ",
		       arg0, val, Z, Zv);

    avr_dmem_write(&(avr->dmem), Z, val);
    avr_dmem_write(&(avr->dmem), arg0, Zv);

    ++avr->pc;
}

void avr_ins_st_yp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\tY+:\t%u -> *%04x ", arg0, val, Y);

    avr_dmem_write(&(avr->dmem), Y, val);

    avr_rampy_y_write(&(avr->dmem), Y + 1);

    ++avr->pc;
}

void avr_ins_st_my(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t Y = avr_rampy_y_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\t-Y:\t%u -> *%04x ", arg0, val, Y);

    avr_dmem_write(&(avr->dmem), Y, val);

    avr_rampy_y_write(&(avr->dmem), Y);

    ++avr->pc;
}

void avr_ins_st_x(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\tX:\t%u -> *%04x ", arg0, val, X);

    avr_dmem_write(&(avr->dmem), X, val);

    ++avr->pc;
}

void avr_ins_st_xp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem));
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\tX+:\t%u -> *%04x ", arg0, val, X);

    avr_dmem_write(&(avr->dmem), X, val);

    avr_rampx_x_write(&(avr->dmem), X + 1);

    ++avr->pc;
}

void avr_ins_st_mx(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint32_t X = avr_rampx_x_read(&(avr->dmem)) - 1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("st\t%02x,\t-X:\t%u -> *%04x ", arg0, val, X);

    avr_dmem_write(&(avr->dmem), X, val);

    avr_rampx_x_write(&(avr->dmem), X);

    ++avr->pc;
}

void avr_ins_push(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("push\t%02x:\t%u ", arg0, val);

    avr_push(&(avr->dmem), val);

    ++avr->pc;
}

void avr_ins_com(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = ~Rd;

    avrjs_debug_printf("com\t%02x:\t~%u = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_set_bit(&(avr->dmem), AVR_SREG_CARRY_BIT);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_OVERFLOW_BIT);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_neg(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = (~Rd) + 1;

    avrjs_debug_printf("neg\t%02x:\t(~%u + 1) = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry7(sreg, R);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow6(sreg, R);
    avr_sreg_sign(sreg);
    avr_sreg_half_carry4(sreg, Rd, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_swap(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t R = ((Rd & 0x0F) << 4) | ((Rd & 0xF0) >> 4);

    avrjs_debug_printf("swap\t%02x:\tswap(%u) = %u ", arg0, Rd, R);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_inc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd + 1;

    avrjs_debug_printf("inc\t%02x:\t%u + 1 = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow6(sreg, R);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_asr(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = (Rd >> 1) | (Rd & (1 << 7));

    avrjs_debug_printf("asr\t%02x:\tasr(%u) = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry3(sreg, Rd);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow3(sreg);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_lsr(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd >> 1;

    avrjs_debug_printf("lsr\t%02x:\t%u >> 1 = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry3(sreg, Rd);
    avr_sreg_zero1(sreg, R);
    avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_NEGATIVE_BIT);
    avr_sreg_overflow3(sreg);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_ror(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = (Rd >> 1) |
            (avr_sreg_read_bit(&(avr->dmem), AVR_SREG_CARRY_BIT) << 7);

    avrjs_debug_printf("ror\t%02x:\tror(%u) = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry3(sreg, Rd);
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow3(sreg);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_bset(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;

    avrjs_debug_printf("bset\t%02x ", arg0);

    avr_sreg_set_bit(&(avr->dmem), arg0);

    ++avr->pc;
}

void avr_ins_ijmp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;

    const uint16_t Z =  avr_z_read(&(avr->dmem));

    avrjs_debug_printf("ijmp:\t\t\t%04x", Z);

    avr->pc = Z;
}

void avr_ins_dec(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);

    const uint8_t R = Rd - 1;

    avrjs_debug_printf("dec\t%02x:\t\t%u - 1 = %u ", arg0, Rd, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_zero1(sreg, R);
    avr_sreg_negative1(sreg, R);
    avr_sreg_overflow5(sreg, R);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);

    ++avr->pc;
}

void avr_ins_jmp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;

    const uint32_t addr = (((uint32_t) arg0) << 16) |
        avr->pmem.mem[avr->pc + 1];

    avrjs_debug_printf("jmp\t%04x ", addr);

    avr->pc = addr;
}

void avr_ins_call(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    // push pc + 2 onto the stack
    const uint32_t pc_ret = avr->pc + 2;
    uint8_t i;
    for (i = avr->pc_size - 1; i < avr->pc_size; --i)
    {
        avr_push(&(avr->dmem), pc_ret >> (i << 3));
    }
    const uint32_t addr = (((uint32_t) arg0) << 16) |
        avr->pmem.mem[avr->pc + 1];

    avrjs_debug_printf("call\t%04x ", addr);

    avr->pc = addr;
}

void avr_ins_eijmp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;

    uint32_t addr = avr_z_read(&(avr->dmem));
    if (avr->dmem.eind_loc < avr->dmem.size)
    {
        addr |= ((uint32_t) avr_dmem_read(&(avr->dmem), avr->dmem.eind_loc))
                << 16;
    }

    avrjs_debug_printf("eijmp:\t\t\t%04x ", addr);

    avr->pc = addr;
}

void avr_ins_bclr(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;

    avrjs_debug_printf("bclr\t%02x ", arg0);

    avr_sreg_clear_bit(&(avr->dmem), arg0);

    ++avr->pc;
}

void avr_ins_ret(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    uint32_t addr = avr_pop(&(avr->dmem));
    uint8_t i;
    for (i = 1; i < avr->pc_size; ++i)
    {
        addr |= ((uint32_t) avr_pop(&(avr->dmem))) << (i << 3);
    }

    avrjs_debug_printf("ret:\t\t\t%04x ", addr);

    avr->pc = addr;
}

void avr_ins_icall(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    uint32_t pc_ret = avr->pc + 1;
    uint8_t i;
    for (i = avr->pc_size - 1; i < avr->pc_size; --i)
    {
        avr_push(&(avr->dmem), pc_ret >> (i << 3));
    }

    const uint16_t Z = avr_z_read(&(avr->dmem));

    avrjs_debug_printf("icall:\t\t\t%04x ", Z);

    avr->pc = Z;
}

void avr_ins_reti(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    uint32_t addr = avr_pop(&(avr->dmem));
    uint8_t i;
    for (i = 1; i < avr->pc_size; ++i)
    {
        addr |= ((uint32_t) avr_pop(&(avr->dmem))) << (i << 3);
    }

    avrjs_debug_printf("reti:\t\t\t%04x ", addr);

    avr->pc = addr;

    avr_sreg_set_bit(&(avr->dmem), AVR_SREG_INTERRUPT_BIT);
}

void avr_ins_eicall(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    const uint32_t pc_ret = avr->pc + 2;
    uint8_t i;
    for (i = avr->pc_size - 1; i < avr->pc_size; --i)
    {
        avr_push(&(avr->dmem), pc_ret >> (i << 3));
    }

    uint32_t addr = avr_z_read(&(avr->dmem));
    if (avr->dmem.eind_loc < avr->dmem.size)
    {
        addr |= ((uint32_t) avr_dmem_read(&(avr->dmem), avr->dmem.eind_loc))
                << 16;
    }

    avrjs_debug_printf("eicall:\t\t\t%04x ", addr);

    avr->pc = addr;
}

void avr_ins_sleep(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    if (avr->callbacks.sleep != 0)
    {
        avr->callbacks.sleep(avr->callbacks.sleep_arg, 1);
        avr->asleep = 1;
    }

    avrjs_debug_printf("sleep ");

    ++avr->pc;
}

void avr_ins_break(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;

    avrjs_debug_printf("break ");

    ++avr->pc;
}

void avr_ins_wdr(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;

    avrjs_debug_printf("wdr ");

    ++avr->pc;
}

void avr_ins_lpm(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    const uint16_t Z = avr_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("lpm:\t\t\t*%04x = %u ", Z, val);

    avr_dmem_write(&(avr->dmem), 0, val);

    ++avr->pc;
}

void avr_ins_elpm_r0(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg0;
    (void) arg1;
    const uint32_t Z = avr_rampz_z_read(&(avr->dmem));
    const uint8_t val = avr_pmem_read_byte(&(avr->pmem), Z);

    avrjs_debug_printf("elpm:\t\t\t*%04x = %u ", Z, val);

    avr_dmem_write(&(avr->dmem), 0, val);

    ++avr->pc;
}

void avr_ins_spm(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    avrjs_debug_printf("spm not implemented ");
    (void) arg0;
    (void) arg1;
    // TODO: SPM
    ++avr->pc;
}

void avr_ins_spm_zp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    avrjs_debug_printf("spm Zp not implemented ");
    (void) arg0;
    (void) arg1;
    // TODO: SPM
    ++avr->pc;
}

void avr_ins_adiw(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 = 24 + (arg0 << 1);
    const uint8_t Rdh = avr_dmem_read(&(avr->dmem), arg0 + 1);
    const uint16_t Rd = avr_dmem_read(&(avr->dmem), arg0) | (Rdh << 8);

    const uint16_t R = ((Rd + arg1) & 0xFFFF);

    avrjs_debug_printf("adiw\t%02x,\t%02x:\t%u + %u = %u ",
        arg0, arg1, Rd, arg1, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry2(sreg, Rdh, R);
    avr_sreg_zero2(sreg, R);
    avr_sreg_negative2(sreg, R);
    avr_sreg_overflow2(sreg, Rdh, R);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R & 0xFF);
    avr_dmem_write(&(avr->dmem), arg0 + 1, (R >> 8) & 0xFF);

    ++avr->pc;
}

void avr_ins_sbiw(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    arg0 = 24 + (arg0 << 1);
    const uint8_t Rdh = avr_dmem_read(&(avr->dmem), arg0 + 1);
    const uint16_t Rd = avr_dmem_read(&(avr->dmem), arg0) | (Rdh << 8);

    const uint16_t R = Rd - arg1;

    avrjs_debug_printf("sbiw\t%02x,\t%02x:\t%u - %u = %u ",
        arg0, arg1, Rd, arg1, R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry8(sreg, Rdh, R);
    avr_sreg_zero2(sreg, R);
    avr_sreg_negative2(sreg, R);
    avr_sreg_overflow7(sreg, Rdh, R);
    avr_sreg_sign(sreg);

    avr_dmem_write(&(avr->dmem), arg0, R);
    avr_dmem_write(&(avr->dmem), arg0 + 1, R >> 8);

    ++avr->pc;
}

void avr_ins_cbi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1 +
				      avr->dmem.io_resisters_start) & ~(1 << arg0);

    avrjs_debug_printf("cbi\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg1 + avr->dmem.io_resisters_start,
                   val);

    ++avr->pc;
}

void avr_ins_sbic(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1 +
        avr->dmem.io_resisters_start);

    avrjs_debug_printf("sbic\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    if ((val & (1 << arg0)) == 0)
    {
        avr_skip_next_instruction(avr);
    }
    else
    {
        ++avr->pc;
    }
}

void avr_ins_sbi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1 +
				      avr->dmem.io_resisters_start) | (1 << arg0);

    avrjs_debug_printf("sbi\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg1 + avr->dmem.io_resisters_start,
		   val);

    ++avr->pc;
}

void avr_ins_sbis(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1 +
        avr->dmem.io_resisters_start);

    avrjs_debug_printf("sbis\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    if ((val & (1 << arg0)) != 0)
    {
        avr_skip_next_instruction(avr);
    }
    else
    {
        ++avr->pc;
    }
}

void avr_ins_mul(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t Rd = avr_dmem_read(&(avr->dmem), arg0);
    const uint8_t Rr = avr_dmem_read(&(avr->dmem), arg1);

    const uint16_t R = ((uint16_t) Rd) * ((uint16_t) Rr);

    avrjs_debug_printf("mul\t%02x,\t%02x:\t%u * %u = %u ", arg0, arg1, Rd, Rr,
        R);

    uint8_t* const sreg = &avr->dmem.mem[avr->dmem.sreg_loc];
    avr_sreg_carry6(sreg, R);
    avr_sreg_zero2(sreg, R);

    avr_dmem_write(&(avr->dmem), 0, R);
    avr_dmem_write(&(avr->dmem), 1, R >> 8);

    ++avr->pc;
}

/*void avr_ins_lds16(struct avr* const avr, uint16_t arg0, uint16_t arg1)
{

}*/

/*void avr_ins_sts16(struct avr* const avr, uint16_t arg0, uint16_t arg1)
{

}*/

void avr_ins_in(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg1 +
				      avr->dmem.io_resisters_start);

    avrjs_debug_printf("in\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_out(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t val = avr_dmem_read(&(avr->dmem), arg0);

    avrjs_debug_printf("in\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg1 + avr->dmem.io_resisters_start,
                   val);

    ++avr->pc;
}

void avr_ins_rjmp(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const int16_t offs = (arg0 & 0x07FF) - (arg0 & 0x0800);

    avrjs_debug_printf("rjmp\t%03x:\t\t%d ", arg0, offs);

    avr->pc = (avr->pc + offs) + 1;
}

void avr_ins_rcall(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    (void) arg1;
    const int16_t offs = (arg0 & 0x07FF) - (arg0 & 0x0800);

    avrjs_debug_printf("rcall\t%03x:\t\t%d ", arg0, offs);

    avr->pc += 1;
    uint8_t i;
    for (i = avr->pc_size - 1; i < avr->pc_size; --i)
    {
        avr_push(&(avr->dmem), avr->pc >> (i << 3));
    }

    avr->pc += offs;
}

void avr_ins_ldi(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    avrjs_debug_printf("ldi\t%02x,\t%02x:\t%u ", arg0, arg1, arg1);

    arg0 += 16;
    avr_dmem_write(&(avr->dmem), arg0, arg1);

    ++avr->pc;
}

void avr_ins_brbs(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_sreg_read_bit(&(avr->dmem), arg0);
    const int8_t offs = (arg1 & 0x003F) - (arg1 & 0x0040);

    avrjs_debug_printf("brbs\t%02x,\t%02x:\t%u, %d ", arg0, arg1, bit, offs);

    if (bit != 0)
    {
        avr->pc += offs;
    }

    ++avr->pc;
}

void avr_ins_brbc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_sreg_read_bit(&(avr->dmem), arg0);
    const int8_t offs = (arg1 & 0x003F) - (arg1 & 0x0040);

    avrjs_debug_printf("brbc\t%02x,\t%02x:\t%u, %d ", arg0, arg1, bit, offs);

    if (bit == 0)
    {
        avr->pc += offs;
    }

    ++avr->pc;
}

void avr_ins_bld(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_sreg_read_bit(&(avr->dmem), AVR_SREG_TRANSFER_BIT);
    uint8_t val;

    if (bit != 0)
    {
        val = avr_dmem_read(&(avr->dmem), arg0) | (1 << arg1);
    }
    else
    {
        val = avr_dmem_read(&(avr->dmem), arg0) & ~(1 << arg1);
    }

    avrjs_debug_printf("bld\t%02x,\t%02x:\t%u ", arg0, arg1, val);

    avr_dmem_write(&(avr->dmem), arg0, val);

    ++avr->pc;
}

void avr_ins_bst(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_dmem_read(&(avr->dmem), arg0) & (1 << arg1);

    avrjs_debug_printf("bst\t%02x,\t%02x:\t%u ", arg0, arg1, bit);

    if (bit != 0)
    {
        avr_sreg_set_bit(&(avr->dmem), AVR_SREG_TRANSFER_BIT);
    }
    else
    {
        avr_sreg_clear_bit(&(avr->dmem), AVR_SREG_TRANSFER_BIT);
    }

    ++avr->pc;
}

void avr_ins_sbrc(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_dmem_read(&(avr->dmem), arg0) & (1 << arg1);

    avrjs_debug_printf("sbrc\t%02x,\t%02x:\t%u ", arg0, arg1, bit);

    if (bit == 0)
    {
        avr_skip_next_instruction(avr);
    }
    else
    {
        ++avr->pc;
    }
}

void avr_ins_sbrs(struct avr * const avr, uint16_t arg0, uint16_t arg1)
{
    const uint8_t bit = avr_dmem_read(&(avr->dmem), arg0) & (1 << arg1);

    avrjs_debug_printf("sbrs\t%02x,\t%02x:\t%u ", arg0, arg1, bit);

    if (bit != 0)
    {
        avr_skip_next_instruction(avr);
    }
    else
    {
        ++avr->pc;
    }
}

void avr_tick(struct avr * const avr)
{
    struct avr_pmem_decoded* dec = &avr->pmem.decoded[avr->pc];
    avrjs_debug_printf("%04x\t", avr->pc * 2);
    if (dec->function != 0)
    {
        dec->function(avr, dec->arg0, dec->arg1);
    }
    avrjs_debug_printf("\tsreg: %02x\n", avr->dmem.mem[avr->dmem.sreg_loc]);
    if (avr->interrupt_delay != 0)
    {
        --avr->interrupt_delay;
    }
    else
    {
        if (avr->interrupt_vector != 0)
        {
            avr_interrupt_call(avr);
        }
    }
}

void avr_init(struct avr* const avr, const struct avr_callbacks callbacks)
{
    static const struct avr_instruction instructions[] = {
        {.pattern = 0x0000, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_nop, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // NOP
        {.pattern = 0x0100, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_movw, .get_arg0 = &x00F0,
            .get_arg1 = &x000F}, // MOVW Rd,Rr
        {.pattern = 0x0200, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_muls, .get_arg0 = &x00F0,
            .get_arg1 = &x000F}, // MULS Rd,Rr
        {.pattern = 0x0300, .mask = 0xFF88, .length = 1,
            .function = &avr_ins_mulsu, .get_arg0 = &x0070,
            .get_arg1 = &x0007}, // MULSU Rd,Rr
        {.pattern = 0x0308, .mask = 0xFF88, .length = 1,
            .function = &avr_ins_fmul, .get_arg0 = &x0070,
            .get_arg1 = &x0007}, // FMUL Rd,Rr
        {.pattern = 0x0380, .mask = 0xFF88, .length = 1,
            .function = &avr_ins_fmuls, .get_arg0 = &x0070,
            .get_arg1 = &x0007}, // FMULS Rd,Rr
        {.pattern = 0x0388, .mask = 0xFF88, .length = 1,
            .function = &avr_ins_fmulsu, .get_arg0 = &x0070,
            .get_arg1 = &x0007}, // FMULSU Rd,Rr
        {.pattern = 0x0400, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_cpc, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // CPC Rd,Rr
        {.pattern = 0x0800, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_sbc, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // SBC Rd,Rr
        {.pattern = 0x0C00, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_add, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // ADD Rd,Rr
        {.pattern = 0x1000, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_cpse, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // CPSE Rd,Rr
        {.pattern = 0x1400, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_cp, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // CP Rd,Rr
        {.pattern = 0x1800, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_sub, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // SUB Rd,Rr
        {.pattern = 0x1C00, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_adc, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // ADC Rd,Rr
        {.pattern = 0x2000, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_and, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // AND Rd,Rr
        {.pattern = 0x2400, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_eor, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // EOR Rd,Rr
        {.pattern = 0x2800, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_or, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // OR Rd,Rr
        {.pattern = 0x2C00, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_mov, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // MOV Rd,Rr
        {.pattern = 0x3000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_cpi, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // CPI Rd,K
        {.pattern = 0x4000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_sbci, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // SBCI Rd,k
        {.pattern = 0x5000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_subi, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // SUBI Rd,K
        {.pattern = 0x6000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_ori, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // ORI Rd,K
        {.pattern = 0x7000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_andi, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // ANDI Rd,K
        {.pattern = 0x8000, .mask = 0xD208, .length = 1,
            .function = &avr_ins_ld_zpq, .get_arg0 = &x01F0,
            .get_arg1 = &x2C07}, // LD Rd, Z+q
        {.pattern = 0x8008, .mask = 0xD208, .length = 1,
            .function = &avr_ins_ld_ypq, .get_arg0 = &x01F0,
            .get_arg1 = &x2C07}, // LD Rd, Y+q
        {.pattern = 0x8200, .mask = 0xD208, .length = 1,
            .function = &avr_ins_st_zpq, .get_arg0 = &x01F0,
            .get_arg1 = &x2C07}, // ST Z+q, Rr
        {.pattern = 0x8208, .mask = 0xD208, .length = 1,
            .function = &avr_ins_st_ypq, .get_arg0 = &x01F0,
            .get_arg1 = &x2C07}, // ST Y+q, Rr
        {.pattern = 0x9000, .mask = 0xFE0F, .length = 2,
            .function = &avr_ins_lds, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LDS Rd,k
        {.pattern = 0x9001, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_zp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, Z+
        {.pattern = 0x9002, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_mz, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, -Z
        {.pattern = 0x9004, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_lpm_z, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LPM Rd,Z
        {.pattern = 0x9005, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_lpm_zp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LPM Rd Z+
        {.pattern = 0x9006, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_elpm_z, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ELPM, Z
        {.pattern = 0x9007, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_elpm_zp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ELPM, Z+
        {.pattern = 0x9009, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_yp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, Y+
        {.pattern = 0x900A, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_my, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, -Y
        {.pattern = 0x900C, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_x, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, X
        {.pattern = 0x900D, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_xp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, X+
        {.pattern = 0x900E, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ld_mx, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LD Rd, -X
        {.pattern = 0x900F, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_pop, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // POP Rd
        {.pattern = 0x9200, .mask = 0xFE0F, .length = 2,
            .function = &avr_ins_sts, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // STS k,Rd
        {.pattern = 0x9201, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_zp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST Z+,Rr
        {.pattern = 0x9202, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_mz, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST -Z, Rr
        {.pattern = 0x9204, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_xch, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // XCH Z,Rd
        {.pattern = 0x9205, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_las, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LAS Z,Rd
        {.pattern = 0x9206, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_lac, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LAC Z,Rd
        {.pattern = 0x9207, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_lat, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LAT Z,Rd
        {.pattern = 0x9209, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_yp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST Y+,Rr
        {.pattern = 0x920A, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_my, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST -Y, Rr
        {.pattern = 0x920C, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_x, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST X,Rr
        {.pattern = 0x920D, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_xp, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST X+,Rr
        {.pattern = 0x920E, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_st_mx, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ST -X, Rr
        {.pattern = 0x920F, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_push, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // PUSH Rr
        {.pattern = 0x9400, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_com, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // COM Rd
        {.pattern = 0x9401, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_neg, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // NEG Rd
        {.pattern = 0x9402, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_swap, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // SWAP Rd
        {.pattern = 0x9403, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_inc, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // INC Rd
        {.pattern = 0x9405, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_asr, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ASR Rd
        {.pattern = 0x9406, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_lsr, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // LSR Rd
        {.pattern = 0x9407, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_ror, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // ROR Rd
        {.pattern = 0x9408, .mask = 0xFF8F, .length = 1,
            .function = &avr_ins_bset, .get_arg0 = &x0070,
            .get_arg1 = &x0000}, // BSET
        {.pattern = 0x9409, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_ijmp, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // IJMP
        {.pattern = 0x940A, .mask = 0xFE0F, .length = 1,
            .function = &avr_ins_dec, .get_arg0 = &x01F0,
            .get_arg1 = &x0000}, // DEC Rd
        {.pattern = 0x940C, .mask = 0xFE0E, .length = 2,
            .function = &avr_ins_jmp, .get_arg0 = &x01F1,
            .get_arg1 = &x0000}, // JMP k
        {.pattern = 0x940E, .mask = 0xFE0E, .length = 2,
            .function = &avr_ins_call, .get_arg0 = &x01F1,
            .get_arg1 = &x0000}, // CALL
        {.pattern = 0x9419, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_eijmp, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // EIJMP
        {.pattern = 0x9488, .mask = 0xFF8F, .length = 1,
            .function = &avr_ins_bclr, .get_arg0 = &x0070,
            .get_arg1 = &x0000}, // BCLR s
        {.pattern = 0x9508, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_ret, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // RET
        {.pattern = 0x9509, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_icall, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // ICALL
        {.pattern = 0x9518, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_reti, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // RETI
        {.pattern = 0x9519, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_eicall, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // EICALL
        {.pattern = 0x9588, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_sleep, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // SLEEP
        {.pattern = 0x9598, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_break, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // BREAK
        {.pattern = 0x95A8, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_wdr, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // WDR
        {.pattern = 0x95C8, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_lpm, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // LPM
        {.pattern = 0x95D8, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_elpm_r0, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // ELPM
        {.pattern = 0x95E8, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_spm, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // SPM
        {.pattern = 0x95F8, .mask = 0xFFFF, .length = 1,
            .function = &avr_ins_spm_zp, .get_arg0 = &x0000,
            .get_arg1 = &x0000}, // SPM Z+
        {.pattern = 0x9600, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_adiw, .get_arg0 = &x0030,
            .get_arg1 = &x00CF}, // ADIW Rp,uimm6
        {.pattern = 0x9700, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_sbiw, .get_arg0 = &x0030,
            .get_arg1 = &x00CF}, // SBIW Rd+1:Rd,K
        {.pattern = 0x9800, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_cbi, .get_arg0 = &x0007,
            .get_arg1 = &x00F8}, // CBI A,b
        {.pattern = 0x9900, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_sbic, .get_arg0 = &x0007,
            .get_arg1 = &x00F8}, // SBIC A,b
        {.pattern = 0x9A00, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_sbi, .get_arg0 = &x0007,
            .get_arg1 = &x00F8}, // SBI A,b
        {.pattern = 0x9B00, .mask = 0xFF00, .length = 1,
            .function = &avr_ins_sbis, .get_arg0 = &x0007,
            .get_arg1 = &x00F8}, // SBIS A,b
        {.pattern = 0x9C00, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_mul, .get_arg0 = &x01F0,
            .get_arg1 = &x020F}, // MUL Rd,Rr
        //{.pattern = 0xA000, .mask = 0xF800, .length = 1,
        //.function = &avr_ins_lds16, .get_arg0 = &x00F0,
        //    .get_arg1 = &x0000},   // LDS (16 bit)
        //{.pattern = 0xA800, .mask = 0xF800, .length = 1,
        //.function = &avr_ins_sts16, .get_arg0 = &x00F0,
        //    .get_arg1 = &x0000},   // STS k,Rd
        {.pattern = 0xB000, .mask = 0xF800, .length = 1,
            .function = &avr_ins_in, .get_arg0 = &x01F0,
            .get_arg1 = &x060F}, // IN
        {.pattern = 0xB800, .mask = 0xF800, .length = 1,
            .function = &avr_ins_out, .get_arg0 = &x01F0,
            .get_arg1 = &x060F}, // OUT A,Rr
        {.pattern = 0xC000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_rjmp, .get_arg0 = &x0FFF,
            .get_arg1 = &x0000}, // RJMP k
        {.pattern = 0xD000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_rcall, .get_arg0 = &x0FFF,
            .get_arg1 = &x0000}, // RCALL k
        {.pattern = 0xE000, .mask = 0xF000, .length = 1,
            .function = &avr_ins_ldi, .get_arg0 = &x00F0,
            .get_arg1 = &x0F0F}, // LDI Rd,K
        {.pattern = 0xF000, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_brbs, .get_arg0 = &x0007,
            .get_arg1 = &x03F8}, // BRBS s,k
        {.pattern = 0xF400, .mask = 0xFC00, .length = 1,
            .function = &avr_ins_brbc, .get_arg0 = &x0007,
            .get_arg1 = &x03F8}, // BRBC s,k
        {.pattern = 0xF800, .mask = 0xFE08, .length = 1,
            .function = &avr_ins_bld, .get_arg0 = &x01F0,
            .get_arg1 = &x0007}, // BLD Rd,b
        {.pattern = 0xFA00, .mask = 0xFE08, .length = 1,
            .function = &avr_ins_bst, .get_arg0 = &x01F0,
            .get_arg1 = &x0007}, // BST Rd,b
        {.pattern = 0xFC00, .mask = 0xFE08, .length = 1,
            .function = &avr_ins_sbrc, .get_arg0 = &x01F0,
            .get_arg1 = &x0007}, // SBRC Rr,b
        {.pattern = 0xFE00, .mask = 0xFE08, .length = 1,
            .function = &avr_ins_sbrs, .get_arg0 = &x01F0,
            .get_arg1 = &x0007}, // SBRS Rr,b
    };

    memset(avr->dmem.callbacks, 0, avr->dmem.io_resisters_length *
           sizeof (*avr->dmem.callbacks));
    memset(avr->dmem.mem, 0, avr->dmem.size * sizeof (*avr->dmem.mem));
    memset(avr->pmem.decoded, 0, avr->pmem.size * sizeof (*avr->pmem.decoded));
    memset(avr->pmem.mem, 0, avr->pmem.size * sizeof (*avr->pmem.mem));

    avr->pmem.instructions = instructions;
    avr->pmem.instructions_limit = instructions + (sizeof (instructions) /
            sizeof (instructions[0]));

    avr->pc = 0;
    avr->asleep = 0;
    avr->interrupt_delay = 0;
    avr->interrupt_vector = 0;
    avr->callbacks = callbacks;
}
