/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include "sfra_examples_hal.h"
#include "drivers/gpio/v0/gpio.h"

//
// Global variables used that are applicable to this board
//

//
//  This routine sets up the basic device configuration such as initializing PLL
//  copying code from FLASH to RAM
//

//
// setupADC()
//
void setupADC(void)
{

//    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
//    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
//    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    ADC_setPrescaler(ADCA_BASE_ADDR, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE_ADDR, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCC_BASE_ADDR, ADC_CLK_DIV_4_0);

    ADC_enableConverter(ADCA_BASE_ADDR);
    ADC_enableConverter(ADCB_BASE_ADDR);
    ADC_enableConverter(ADCC_BASE_ADDR);

//    DEVICE_DELAY_US(1000);

    //
    // setup ADC conversions for current and voltage signals
    //

}

//
// configurePWM1chUpCnt()
//
void configurePWM1chUpCnt(uint32_t base1, uint16_t period)
{

    //
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(base1, period - 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // configure PWM 1 as master and Phase 2 as slaves and
    // let it pass the sync in pulse from PWM1
    //
    EPWM_disablePhaseShiftLoad(base1);
    EPWM_enableSyncOutPulseSource(base1, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    //
    // Counter Compare Submodule Registers
    // set duty 0% initially
    //
    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Action Qualifier SubModule Registers
    //
    HW_WR_REG16((base1 + CSL_EPWM_AQCTLA), 0) ;
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

}

//
// disablePWMCLKCounting
//
void disablePWMCLKCounting(void)
{
  //  SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* Time base clock enable register belongs to partition 1 of the CTRL MMR */
    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);
    /* Disable Time base clock in CTRL MMR */
    CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
        ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
          CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) & ~(0xFFFFFFFF)));
}

//
// enablePWMCLKCounting
//
void enablePWMCLKCounting(void)
{
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* Time base clock enable register belongs to partition 1 of the CTRL MMR */

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, CONTROLSS_CTRL_PARTITION0);

    CSL_REG32_WR(CSL_CONTROLSS_CTRL_U_BASE + CSL_CONTROLSS_CTRL_EPWM_CLKSYNC,
        ((CSL_REG32_RD(CSL_CONTROLSS_CTRL_U_BASE +
          CSL_CONTROLSS_CTRL_EPWM_CLKSYNC) & CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MASK) | (0xFFFFFFFF)));
}

//
// setupProfilingGPIO()
//
void setupProfilingGPIO(void)
{
    GPIO_setDirMode(CSL_GPIO0_U_BASE, GPIO_PROFILING1, GPIO_DIRECTION_OUTPUT);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, GPIO_PROFILING2, GPIO_DIRECTION_OUTPUT);

    //IO MUX Unlocking
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + 0x1008 , 0x01234567); //LOCK KICK0
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + 0x100C , 0x0FEDCBA8); //LOCK KICK1
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0, 0x83E70B13) ; //Lock kick 0
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1, 0x95A4F1E0) ; //Lock kick 1

    HW_WR_REG32(CSL_IOMUX_U_BASE + GPIO_PROFILING1_PIN_CONFIG, 0x7) ; //Qual Sync, Pin config as GPIO
    HW_WR_REG32(CSL_IOMUX_U_BASE + GPIO_PROFILING2_PIN_CONFIG, 0x7) ; //Qual Sync, Pin config as GPIO

}

void setupUpDwnCountPWM(uint32_t base1, uint16_t pwm_period_ticks)
{

    //
    // PWM clock on F2837x is divided by 2
    // ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1
    // Deadband needs to be 0.5us => 10ns*50=500ns
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBasePeriod(base1, pwm_period_ticks >> 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

}

void setPinsAsPWM()
{

    GPIO_pinWriteLow(CSL_GPIO0_U_BASE, PWM_H_GPIO);
    GPIO_pinWriteLow(CSL_GPIO0_U_BASE, PWM_L_GPIO);

    GPIO_setDirMode(CSL_GPIO0_U_BASE, PWM_H_GPIO, GPIO_DIRECTION_OUTPUT);
    GPIO_setDirMode(CSL_GPIO0_U_BASE, PWM_L_GPIO, GPIO_DIRECTION_OUTPUT);

    //IO MUX Unlocking
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + 0x1008 , 0x01234567); //LOCK KICK0
    HW_WR_REG32(CSL_CONTROLSS_CTRL_U_BASE + 0x100C , 0x0FEDCBA8); //LOCK KICK1
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK0, 0x83E70B13) ; //Lock kick 0
    HW_WR_REG32(CSL_IOMUX_U_BASE + CSL_IOMUX_IO_CFG_KICK1, 0x95A4F1E0) ; //Lock kick 1

    HW_WR_REG32(CSL_IOMUX_U_BASE + PWM_H_GPIO_PIN_CONFIG, 0x0) ; // Pin config as EPWM0_A
    HW_WR_REG32(CSL_IOMUX_U_BASE + PWM_L_GPIO_PIN_CONFIG, 0x0) ; // Pin config as EPWM0_B


}

