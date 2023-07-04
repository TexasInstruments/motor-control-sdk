/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <drivers/pinmux.h>
#include "cfg_pad.h"

static Pinmux_PerCfg_t gPinMuxMainDomainCfgSddf[] = {

    /* PRG0_ECAP0_IN_APWM_OUT,
       PRG0_PRU1_GPO15, PRG0_ECAP0_IN_APWM_OUT, U5, J2.C11  */
    {
        PIN_PRG0_PRU1_GPO15,
        ( PIN_MODE(10) | PIN_PULL_DISABLE )
    },
    /* SD8_CLK,
       PRG0_PRU0_GPI16, SD8_CLK,    U4, J2E:P9 */
    {
        PIN_PRG0_PRU0_GPO16,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD0_D,
      PRG0_PRU0_GPI1,   SD0_D,      R4, J2E:P8 */
    {
        PIN_PRG0_PRU0_GPO1,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD1_D,
       PRG0_PRU0_GPI3, SD1_D,      V2, J2A:P9  */
    {
        PIN_PRG0_PRU0_GPO3,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD2_D,
       PRG0_PRU0_GPI5,  SD2_D,      R3, J2C:P6  */
    {
        PIN_PRG0_PRU0_GPO5,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD3_D,
       PRG0_PRU0_GPI7,  SD3_D,      T1, J2B:P7  */
    {
        PIN_PRG0_PRU0_GPO7,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD4_D,
       PRG0_PRU0_GPI18,  SD4_D,     V1, J2B:P9  */
    {
        PIN_PRG0_PRU0_GPO18,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD5_D,
       PRG0_PRU0_GPI11, SD5_D,      Y3, J2B:P14 */
    {
        PIN_PRG0_PRU0_GPO11,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD6_D,
       PRG0_PRU0_GPI13, SD6_D,      R6, J2C:P5  */
    {
        PIN_PRG0_PRU0_GPO13,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD7_D,
       PRG0_PRU0_GPI15, SD7_D,      T5, J2D:P12 */
    {
        PIN_PRG0_PRU0_GPO15,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* SD8_D,
       PRG0_PRU0_GPI17, SD8_D,      U1, J2B:P8  */
    {
        PIN_PRG0_PRU0_GPO17,
        ( PIN_MODE(1) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },

    {PINMUX_END, PINMUX_END}
};

/* Configure SOC pads */
void cfgPad(void)
{
    Pinmux_config(gPinMuxMainDomainCfgSddf, PINMUX_DOMAIN_ID_MAIN);
}
