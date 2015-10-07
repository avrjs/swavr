/*
 avr_sreg.h sreg related routines

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

#ifndef AVR_SREG_H
#define	AVR_SREG_H

#include "avr.h"

#define AVR_SREG_CARRY_BIT (0x00)
#define AVR_SREG_ZERO_BIT (0x01)
#define AVR_SREG_NEGATIVE_BIT (0x02)
#define AVR_SREG_OVERFLOW_BIT (0x03)
#define AVR_SREG_SIGN_BIT (0x04)
#define AVR_SREG_HALF_CARRY_BIT (0x05)
#define AVR_SREG_TRANSFER_BIT (0x06)
#define AVR_SREG_INTERRUPT_BIT (0x07)

#define AVR_SREG_CARRY_MASK (1 << AVR_SREG_CARRY_BIT)
#define AVR_SREG_ZERO_MASK (1 << AVR_SREG_ZERO_BIT)
#define AVR_SREG_NEGATIVE_MASK (1 << AVR_SREG_NEGATIVE_BIT)
#define AVR_SREG_OVERFLOW_MASK (1 << AVR_SREG_OVERFLOW_BIT)
#define AVR_SREG_SIGN_MASK (1 << AVR_SREG_SIGN_BIT)
#define AVR_SREG_HALF_CARRY_MASK (1 << AVR_SREG_HALF_CARRY_BIT)
#define AVR_SREG_TRANSFER_MASK (1 << AVR_SREG_TRANSFER_BIT)
#define AVR_SREG_INTERRUPT_MASK (1 << AVR_SREG_INTERRUPT_BIT)

static inline uint8_t avr_sreg_ebj(const uint8_t number, const uint8_t bit)
{
    return (number & (1 << bit)) >> bit;
}

static inline uint8_t avr_sreg_ebjn(const uint8_t number, const uint8_t bit)
{
    return avr_sreg_ebj(number, bit) ^ 1;
}

static inline uint8_t avr_sreg_ebj16(const uint16_t number, const uint8_t bit)
{
    return (number & (1 << bit)) >> bit;
}

static inline uint8_t avr_sreg_ebjn16(const uint16_t number, const uint8_t bit)
{
    return avr_sreg_ebj16(number, bit) ^ 1;
}

static inline uint8_t avr_sreg_read_bit(const struct avr_dmem * const dmem,
                                        const uint8_t index)
{
    return avr_sreg_ebj(dmem->mem[dmem->sreg_loc], index);
}

static inline void avr_sreg_set_bit(struct avr_dmem * const dmem,
                                    const uint8_t index)
{
    avr_dmem_write(dmem, dmem->sreg_loc, dmem->mem[dmem->sreg_loc] |
                   (1 << index));
}

static inline void avr_sreg_clear_bit(struct avr_dmem * const dmem,
                                      const uint8_t index)
{
    avr_dmem_write(dmem, dmem->sreg_loc, dmem->mem[dmem->sreg_loc] &
                   ~(1 << index));
}

// carry1: Rd7 & Rr7 | Rr7 & !R7 | !R7 & Rd7

static inline void avr_sreg_carry1(uint8_t * const sreg, const uint8_t Rd,
                                   const uint8_t Rr, const uint8_t R)
{
    if(((avr_sreg_ebj(Rd, 7) & avr_sreg_ebj(Rr, 7)) | (avr_sreg_ebj(Rr, 7)
            & avr_sreg_ebjn(R, 7)) | (avr_sreg_ebjn(R, 7)
            & avr_sreg_ebj(Rd, 7))) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry2: !R15 & Rdh7

static inline void avr_sreg_carry2(uint8_t * const sreg, const uint8_t Rdh,
                                   const uint16_t R)
{
    if ((avr_sreg_ebjn16(R, 15) & avr_sreg_ebj(Rdh, 7)) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry3: Rd0

static inline void avr_sreg_carry3(uint8_t * const sreg, const uint8_t Rd)
{
    if ((Rd & 0x01) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry4: !Rd7 & Rr7 | Rr7 & R7 | R7 & !Rd7

static inline void avr_sreg_carry4(uint8_t * const sreg, const uint8_t Rd,
                                   const uint8_t Rr, const uint8_t R)
{
    if (((avr_sreg_ebjn(Rd, 7) & avr_sreg_ebj(Rr, 7)) |
            (avr_sreg_ebj(Rr, 7) & avr_sreg_ebj(R, 7)) | (avr_sreg_ebj(R, 7) &
            avr_sreg_ebjn(Rd, 7))) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry5: Rd7

static inline void avr_sreg_carry5(uint8_t * const sreg, const uint8_t Rd)
{
    if ((Rd & 0x80) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry6: R15

static inline void avr_sreg_carry6(uint8_t * const sreg, const uint16_t R)
{
    if ((R & 0x8000) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry7: R7 | R6 | R5 | R4 | R3 | R2 | R1 | R0

static inline void avr_sreg_carry7(uint8_t * const sreg, const uint8_t R)
{
    if (R != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// carry8: R15 & !Rdh7

static inline void avr_sreg_carry8(uint8_t * const sreg, const uint8_t Rdh,
                                   const uint16_t R)
{
    if ((avr_sreg_ebj16(R, 15) & avr_sreg_ebjn(Rdh, 7)) != 0)
    {
        *sreg |= AVR_SREG_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_CARRY_MASK;
    }
}

// zero1: !R7 & !R6 & !R5 & !R4 & !R3 & !R2 & !R1 & !R0

static inline void avr_sreg_zero1(uint8_t * const sreg, const uint8_t R)
{
    if (R == 0)
    {
        *sreg |= AVR_SREG_ZERO_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_ZERO_MASK;
    }
}

// zero2: !R15 & !R14 & !R13 & !R12 & !R11 & !R10 & !R9 & !R8 & !R7 & !R6
// & !R5 & !R4 & !R3 & !R2 & !R1 & !R0

static inline void avr_sreg_zero2(uint8_t * const sreg, const uint16_t R)
{
    if (R == 0)
    {
        *sreg |= AVR_SREG_ZERO_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_ZERO_MASK;
    }
}

// zero3: !R7 & !R6 & !R5 & !R4 & !R3 & !R2 & !R1 & !R0 & Z

static inline void avr_sreg_zero3(uint8_t * const sreg, const uint8_t R)
{
    if (R != 0)
    {
        *sreg &= ~AVR_SREG_ZERO_MASK;
    }
}

// negative1: R7

static inline void avr_sreg_negative1(uint8_t * const sreg, const uint8_t R)
{
    if ((R & 0x80) != 0)
    {
        *sreg |= AVR_SREG_NEGATIVE_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_NEGATIVE_MASK;
    }
}

// negative2: R15

static inline void avr_sreg_negative2(uint8_t * const sreg, const uint16_t R)
{
    if ((R & 0x8000) != 0)
    {
        *sreg |= AVR_SREG_NEGATIVE_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_NEGATIVE_MASK;
    }
}

// twos complement overflow1: Rd7 & Rr7 & !R7 | !Rd7 & !Rr7 & R7

static inline void avr_sreg_overflow1(uint8_t * const sreg, const uint8_t Rd,
                                      const uint8_t Rr, const uint8_t R)
{
    if (((avr_sreg_ebj(Rd, 7) & avr_sreg_ebj(Rr, 7) & avr_sreg_ebjn(R, 7))
            | (avr_sreg_ebjn(Rd, 7) & avr_sreg_ebjn(Rr, 7)
            & avr_sreg_ebj(R, 7))) != 0)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow2: !Rdh7 & R15

static inline void avr_sreg_overflow2(uint8_t * const sreg, const uint8_t Rdh,
                                      const uint16_t R)
{
    if ((avr_sreg_ebjn(Rdh, 7) & avr_sreg_ebj(R, 15)) != 0)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow3: N ^ C

static inline void avr_sreg_overflow3(uint8_t * const sreg)
{
    if ((avr_sreg_ebj(*sreg, AVR_SREG_NEGATIVE_BIT)
            ^ avr_sreg_ebj(*sreg, AVR_SREG_CARRY_BIT)) != 0)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow4: Rd7 & !Rr7 & !R7 | !Rd7 & Rr7 & R7

static inline void avr_sreg_overflow4(uint8_t * const sreg, const uint8_t Rd,
                                      const uint8_t Rr, const uint8_t R)
{
    if (((avr_sreg_ebj(Rd, 7) & avr_sreg_ebjn(Rr, 7) & avr_sreg_ebjn(R, 7))
            | (avr_sreg_ebjn(Rd, 7) & avr_sreg_ebj(Rr, 7) & avr_sreg_ebj(R, 7)))
            != 0)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow5: !R7 & R6 & R5 & R4 & R3 & R2 & R1 & R0

static inline void avr_sreg_overflow5(uint8_t * const sreg, const uint8_t R)
{
    if (R == 0x7F)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow6: R7 & !R6 & !R5 & !R4 & !R3 & !R2 & !R1 & !R0

static inline void avr_sreg_overflow6(uint8_t * const sreg, const uint8_t R)
{
    if (R == 0x80)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// twos complement overflow7: Rdh7 & !R15

static inline void avr_sreg_overflow7(uint8_t * const sreg,
                                      const uint8_t Rdh, const uint16_t R)
{
    if ((avr_sreg_ebj(Rdh, 7) & avr_sreg_ebjn(R, 15)) != 0)
    {
        *sreg |= AVR_SREG_OVERFLOW_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_OVERFLOW_MASK;
    }
}

// sign: N ^ V

static inline void avr_sreg_sign(uint8_t * const sreg)
{
    if ((avr_sreg_ebj(*sreg, AVR_SREG_NEGATIVE_BIT)
            ^ avr_sreg_ebj(*sreg, AVR_SREG_OVERFLOW_BIT)) != 0)
    {
        *sreg |= AVR_SREG_SIGN_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_SIGN_MASK;
    }
}

// half carry1: Rd3 & Rr3 | Rr3 & !R3 | !R3 & Rd3

static inline void avr_sreg_half_carry1(uint8_t * const sreg,
                                        const uint8_t Rd, const uint8_t Rr,
                                        const uint8_t R)
{
    if (((avr_sreg_ebj(Rd, 3) & avr_sreg_ebj(Rr, 3)) | (avr_sreg_ebj(Rr, 3)
            & avr_sreg_ebjn(R, 3)) | (avr_sreg_ebjn(R, 3)
            & avr_sreg_ebj(Rd, 3))) != 0)
    {
        *sreg |= AVR_SREG_HALF_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_HALF_CARRY_MASK;
    }
}

// half carry2: !Rd3 & Rr3 | Rr3 & R3 | R3 & !Rd3

static inline void avr_sreg_half_carry2(uint8_t * const sreg,
                                        const uint8_t Rd, const uint8_t Rr,
                                        const uint8_t R)
{
    if (((avr_sreg_ebjn(Rd, 3) & avr_sreg_ebj(Rr, 3)) | (avr_sreg_ebj(Rr, 3)
            & avr_sreg_ebj(R, 3)) | (avr_sreg_ebj(R, 3) & avr_sreg_ebjn(Rd, 3)))
            != 0)
    {
        *sreg |= AVR_SREG_HALF_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_HALF_CARRY_MASK;
    }
}

// half carry3: Rd3

static inline void avr_sreg_half_carry3(uint8_t * const sreg,
                                        const uint8_t Rd)
{
    if ((Rd & 0x08) != 0)
    {
        *sreg |= AVR_SREG_HALF_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_HALF_CARRY_MASK;
    }
}

// half carry4: R3 | Rd3

static inline void avr_sreg_half_carry4(uint8_t * const sreg,
                                        const uint8_t Rd,
                                        const uint8_t R)
{
    if ((avr_sreg_ebj(R, 3) | avr_sreg_ebjn(Rd, 3)) != 0)
    {
        *sreg |= AVR_SREG_HALF_CARRY_MASK;
    }
    else
    {
        *sreg &= ~AVR_SREG_HALF_CARRY_MASK;
    }
}

#endif
