/*
 * Copyright (C) 2025-2026 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <position_sense/endat3/include/endat3_drv.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MANCHESTER_DECODE_LUT_SIZE   256

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*  8x oversampling lookup table for manchester data (2bits of manchester data in 8 bit oversample data)*/
/*  Shared LUT with all 3 channels per PRU slice in load share and non-load share mode*/
static const uint8_t endat3_manchester_decode_lut_data[MANCHESTER_DECODE_LUT_SIZE] = {
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x40,
    0x00, 0xFF, 0xFF, 0x40, 0xFF, 0x40, 0x40, 0x40,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x40,
    0x00, 0xFF, 0xFF, 0x40, 0xFF, 0x40, 0x40, 0x40,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x40,
    0x00, 0xFF, 0xFF, 0x40, 0xFF, 0x40, 0x40, 0x40,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x40,
    0x00, 0xFF, 0xFF, 0x40, 0xFF, 0x40, 0x40, 0x40,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF, 0xFF, 0xC0,
    0x80, 0xFF, 0xFF, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0,
    0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0xFF, 0x40,
    0x00, 0xFF, 0xFF, 0x40, 0xFF, 0x40, 0x40, 0x40,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF, 0xFF, 0xC0,
    0x80, 0xFF, 0xFF, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF, 0xFF, 0xC0,
    0x80, 0xFF, 0xFF, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0,
    0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF, 0xFF, 0xC0,
    0x80, 0xFF, 0xFF, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0,
    0x80, 0x80, 0x80, 0xFF, 0x80, 0xFF, 0xFF, 0xC0,
    0x80, 0xFF, 0xFF, 0xC0, 0xFF, 0xC0, 0xC0, 0xC0
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
static void endat3_manchester_decode_lut(endat3_handle handle);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void endat3_manchester_decode_lut(endat3_handle handle)
{
    endat3_priv *priv;
    volatile uint8_t *lut_dest;
    uint32_t i;

    priv = handle->priv;

    /* Copy the Manchester decode lookup table to the lut member of endat3_interface structure in PRU DMEM */
    /* Use byte-by-byte copy to preserve volatile semantics for PRU-shared memory */
    lut_dest = (volatile uint8_t *)priv->endat3_interface->lut;

    for(i = 0; i < MANCHESTER_DECODE_LUT_SIZE; i++)
    {
        lut_dest[i] = endat3_manchester_decode_lut_data[i];
    }
}

void endat3_generate_memory_image(endat3_handle handle)
{
    endat3_manchester_decode_lut(handle);
}
