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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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


#include "stdint.h"
#include "math.h"
#include "stdio.h"

#include <drivers/pruicss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>

void init_IEP0_SYNC(void)
{

#if 0 // FL: move to R5
// ************************************* padconfig *************************************
// unlock PADMMR config register

// partition 0
    HW_WR_REG32(0x000f1008, 0x68EF3490);  // Kick 0
    HW_WR_REG32(0x000f100C, 0xD172BC5A);  // Kick 1

// partition 1
    HW_WR_REG32(0x000f5008, 0x68EF3490);  // Kick 0
    HW_WR_REG32(0x000f500C, 0xD172BC5A);  // Kick 1

// ICSS_G0
// IEP SYNC0 -> BP.45
    HW_WR_REG32(0x000F41AC, 0x00010000 + 2);  // mode 2 = SYNC0

// IEP SYNC1 -> BP.18
    HW_WR_REG32(0x000F41A4, 0x00010000 + 2);  // mode 2 = SYNC1
#endif
#if 0 // FL: move to R5
// ************************************* clock config **********************************
// core clock options:
// ICSSG0_CLKMUX_SEL: bit 0 = 1 PLL0, bit 0 = 0 PLL2
// MAIN_PLL0_HSDIV9: 2 (333 MHz), 3 (250 MHz)
// MAIN_PLL2_HSDIV2: 5 (300 MHz), 7 (225 MHz)
// ICSSG_CFG: iep_clk to follow core clock
//
// iep clock options
// ICSSG0_CLKMUX_SEL: bit 16-17:
// 0h - MAIN_PLL2_HSDIV5_CLKOUT
// 1h - MAIN_PLL0_HSDIV6_CLKOUT
// 7h - SYSCLK0 (500 MHz) - not recommended!!!
// recommend to run both clocks from same PLL0
// e.g. PLL0 iep_clk = 500 MHz, core_clock = 333 MHz

// configure ICSS clocks
// unlock CTRLMMR config register
// partition 2
    HW_WR_REG32(0x43009008, 0x68EF3490 );  // Kick 0
    HW_WR_REG32(0x4300900c, 0xD172BC5A );  // Kick 1

// ICSS_G1_CLKMUX_SEL:
// 1h - MAIN_PLL0_HSDIV9_CLKOUT,
    HW_WR_REG32(0x43008044, 0x00000001 );  // Kick 1

/*     .if(0)
    ; run iep_clk with core clock
        ldi32    r2, 0x00026030
        ldi32    r3, 0x00000001
        sbbo     &r3, r2, 0, 4
     .endif
*/

// unlock PLL0 ctrl register
    HW_WR_REG32(0x00680010, 0x68EF3490 );  // Kick 0
    HW_WR_REG32(0x00680014, 0xD172BC5A );  // Kick 1

// set MAIN_PLL0_HSDIV9 to 2 for core_clock =333MHz (3 for 250 MHz)
    HW_WR_REG8(0x006800a4, 2);

// set MAIN_PLL0_HSDIV6_CLKOUT iep_clock from PLL0 (1 = 500 MHz, 2 = 333 MHz, 3 = 250 MHz)
    HW_WR_REG8(0x00680098, 2);

// unlock PLL2 ctrl register
    HW_WR_REG32(0x00682010, 0x68EF3490 );  // Kick 0
    HW_WR_REG32(0x00682014, 0xD172BC5A );  // Kick 1

// set core clock MAIN_PLL2_HSDIV0 to 5 for 300 MHz (7 for 225 MHz, 5 for 300 MHz)
    HW_WR_REG8(0x00682080, 5);

// set MAIN_PLL2_HSDIV5_CLKOUT iep_clock from PLL2 (5= 300 MHz)
    HW_WR_REG8(0x00682094, 5);
#endif
// ************************************* iep config **********************************
#if 0 // FL: PRU code
// reset iep0 timer
    HW_WR_REG8(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x20);
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, 0xffffffff);
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, 0xffffffff);

// set CMP0 period - 64 kHZ 15625 ns, - cycle as we start from 0
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, 15625-3);
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, 15625-3);

//  set CMP1 period - SYNC01 trigger
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, 99);

//  set CMP2 period - SYNC1 trigger - with 6 ns delay - only used in independent mode
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, 99);

//  Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, 0x00A7);

//  Set SYNC0/1 high pulse time in iep clok cycles  ( 7 clocks for 20 MHz at 300 MHz iep clk)
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, 0x0006);

//  Set SYNC0/1 period  ( 15 clocks for 20 MHz at 300 MHz iep clk)
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, 14);

//  Set delay between SYNC0 and SYNC1 in clock cycles 2 = 3 clock cycles
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, 2);

//  Set offset from cpm1 hit
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

// set enable cmp1 and cmp2 for sync start trigger generation
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, 0x000000c);

// set enable cmp1 and cmp2 for sync start trigger generation
    HW_WR_REG32(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, 0x000000c);

// start iep0_timer - increment by 1 , start
    HW_WR_REG8(CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x0111);
#endif
    // reset iep0 timer
        HW_WR_REG8(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x20);
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, 0xffffffff);
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, 0xffffffff);

    // set CMP0 period - 64 kHZ 15625 ns, - cycle as we start from 0
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, 15625-3);
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, 15625-3);

    //  set CMP1 period - SYNC01 trigger
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, 99);

    //  set CMP2 period - SYNC1 trigger - with 6 ns delay - only used in independent mode
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, 99);

    //  Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, 0x00A7);

    //  Set SYNC0/1 high pulse time in iep clok cycles  ( 7 clocks for 20 MHz at 300 MHz iep clk)
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, 0x0006);

    //  Set SYNC0/1 period  ( 15 clocks for 20 MHz at 300 MHz iep clk)
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, 14);

    //  Set delay between SYNC0 and SYNC1 in clock cycles 2 = 3 clock cycles
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, 2);

    //  Set offset from cpm1 hit
        HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

    // set enable cmp1 and cmp2 for sync start trigger generation
       ///HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, 0x000000c);

    // set enable cmp1 and cmp2 for sync start trigger generation
       HW_WR_REG32(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, 0x00000Dc);

    // set default and compensation increment to 1
        HW_WR_REG8(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x0110);

// ************************************* task manager *************************************
//


    return;
}

void start_IEP0(void)
{
    uint8_t regVal;

    // start iep0_timer
    regVal = HW_RD_REG8(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    regVal += 0x1;
    HW_WR_REG8(CSL_PRU_ICSSG0_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);
}

#if 0 // FL: PRU code
/**
 * main.c
 */
int main(void)
{

    __asm("   tsen 0");
    init_IEP0_SYNC();

    while (1){};

    return 0;
}
#endif
