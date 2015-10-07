/*
 jihex.c parses simplified Intel hex files as used for programming many
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

#include "jihex.h"

#include <stdio.h>

struct jihex
{
    uint16_t h_index;
    uint32_t v_index;
    uint8_t byte_count;
    uint8_t data_index;
    uint16_t address;
    uint16_t base_address;
    uint8_t record_type;
    uint8_t hex_buffer; // this will hold the data from 2 hex ascii chars
};

uint8_t jihex_hex2nib(uint8_t hex)
{
    return (hex <= '9') ? hex - '0' : ((hex <= 'F') ? (hex - 'A') + 10 :
            (hex - 'a') + 10);
}

// the write argument must be a function with arguments (address, data)

uint8_t jihex_parse_char(struct jihex* j, char c,
                         void(*write)(void*, uint32_t, uint8_t), void* arg)
{
    if (c == ':') // start code
    {
        j->h_index = 1;
        ++j->v_index;
    }
    else if (j->h_index > 0)
    {
        switch (j->h_index)
        {
        case 1: // byte_count 1
            j->byte_count = jihex_hex2nib(c) << 4;
            break;
        case 2: // byte_count 2
            j->byte_count |= jihex_hex2nib(c);
        case 3: // address 0
            j->address = jihex_hex2nib(c) << 12;
            break;
        case 4: // address 1
            j->address |= jihex_hex2nib(c) << 8;
            break;
        case 5: // address 2
            j->address |= jihex_hex2nib(c) << 4;
            break;
        case 6: // address 3
            j->address |= jihex_hex2nib(c);
            break;
        case 7: // record_type 0
            j->record_type = jihex_hex2nib(c) << 4;
            break;
        case 8: // record_type 1
            j->record_type |= jihex_hex2nib(c);
            break;
        default:
            switch (j->record_type)
            {
            case 0: // data
                if (j->data_index < (j->byte_count << 1))
                {
                    if ((j->data_index & 1) == 1)
                    {
                        j->hex_buffer |= jihex_hex2nib(c);
                        write(arg, (((uint32_t) j->base_address) << 16)
                              | j->address, j->hex_buffer);
                        ++j->address;
                    }
                    else
                    {
                        j->hex_buffer = jihex_hex2nib(c) << 4;
                    }
                    ++j->data_index;
                }
                else // checksum
                {
                    j->record_type = 6;
                    j->data_index = 0;
                }
                break;
            case 1: // EOF
                return 1;
                break;
            case 2: // extended segment address
                // probably not used
                break;
            case 3: // start segment address
                // probably not used
                break;
            case 4: // extended linear address
                if (j->data_index == 0)
                {
                    j->base_address = jihex_hex2nib(c);
                    ++j->data_index;
                }
                else if (j->data_index < (j->byte_count << 1))
                {
                    j->base_address |= jihex_hex2nib(c) << (j->data_index << 2);
                    ++j->data_index;
                }
                else // checksum
                {
                    j->record_type = 6;
                    j->data_index = 0;
                }
                break;
            case 5: // start linear address
                // probably not used
                break;
            case 6: // checksum

                break;
            }
            break;
        }
        ++j->h_index;
    }
    return 0;
}

int jihex_handle(const char* filename, void(*write)(void*, uint32_t, uint8_t),
                 void* arg)
{
    FILE* fp;
    fp = fopen(filename, "r");
    if (fp == 0)
    {
        return -1;
    }
    struct jihex j = {0};
    int i;
    while (((i = fgetc(fp)) != EOF)
            && (jihex_parse_char(&j, (char) i, write, arg) == 0))
    {
    }
    return fclose(fp);
}
