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

void init_IEP0_SYNC()
{
/* ************************************* iep config ********************************** */

    /*reset iep0 timer*/
    HW_WR_REG8(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x20);
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, 0xffffffff);
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, 0xffffffff);

    /* set CMP1 period - SYNC01 trigger */
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, 99);

    /* set CMP2 period - SYNC1 trigger - with 6 ns delay - only used in independent mode*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, 99);

    /*Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, 0x00A7);

    /*Set SYNC0/1 high pulse time in iep clok cycles  ( 7 clocks for 20 MHz at 300 MHz iep clk) */
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, 0x0006);

    /*Set SYNC0/1 period  ( 15 clocks for 20 MHz at 300 MHz iep clk)*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, 14);

    /*Set delay between SYNC0 and SYNC1 in clock cycles 2 = 3 clock cycles*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, 0);

    /*Set offset from cpm1 hit*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

    /*set enable cmp1 and cmp2 for sync start trigger generation*/
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, 0x000000c);
       
    /*set default and compensation increment to 1*/
    HW_WR_REG8(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, 0x0110);

    return;
}

void start_IEP0(void)
{
    uint8_t regVal;

    /*start iep0_timer*/
    regVal = HW_RD_REG8(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    regVal |= 0x1;
    HW_WR_REG8(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);
}

void config_SYNC_DELAY(uint32_t delay)
{
   /* Set delay between SYNC0 and SYNC1 in clock cycles 2 = 3 clock cycles */
    HW_WR_REG32(CSL_PRU_ICSSG1_IEP0_BASE+CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, delay);
}
