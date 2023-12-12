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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <pruicss_pwm.h>
#include <drivers/pruicss.h>
#include <drivers/pinmux.h>

/** \brief Global Structure pointer holding PRUICSSG1 memory Map. */

PRUICSS_Handle gPruIcssHandle;

/*
 * This example uses the PRUICSS PWM module to generate a signal
 * with a specified duty cycle.
 *
 * The default parameters are : Frequency : 1kHz, Duty cycle : 25%,
 * All these parameters are configurable.
 *
 * In this example PWM0_2_NEG(alias signal B2) is used to generate the signal, the user can also
 * select a different one.
 *  
 * PWM0_2_NEG(alias signal B2) uses IEP0 CMP6 EVENT to control Duty cycle
 * & IEP0 CMP0 to control output Frequency
 * This example also showcases how to configure and use the PRUICSS PWM module.
 */

/*FIXME: IEP0_CLK_FREQ macro to be included in driver_config.h sysconfig generated file*/
#define PRUICSS_IEP0_CLK_FREQ                  (200000000U)
/* Modify this to change the IEP counter increment value*/
#define PRUICSS_IEP_COUNT_INCREMENT_VALUE      (1U)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define APP_PRUICSS_PWM_DUTY_CYCLE             (25U)
/* Frequency  of PWM output signal in Hz - 1 KHz is selected */
#define APP_PRUICSS_PWM_OUTPUT_FREQ            (1U * 1000U)
/* PRD value - this determines the period */
#define APP_PRUICSS_PWM_PRD_VAL                (((PRUICSS_IEP0_CLK_FREQ / APP_PRUICSS_PWM_OUTPUT_FREQ))*(PRUICSS_IEP_COUNT_INCREMENT_VALUE))
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define APP_PRUICSS_IEP0_COMP6_VAL              (APP_PRUICSS_PWM_PRD_VAL-((APP_PRUICSS_PWM_DUTY_CYCLE*APP_PRUICSS_PWM_PRD_VAL)/100))

Pinmux_PerCfg_t gPinMuxMainDomainCfg1[] = {

    /* PRU_ICSSG1_PWM0 pin config */
    /* PRG1_PWM0_B2 -> PRG1_PRU0_GPO17 (U7) */
    {
       PIN_PRG1_PRU0_GPO17,
       ( PIN_MODE(3) | PIN_PULL_DISABLE )
    },

    {PINMUX_END, PINMUX_END}
};
void pruicss_iep_init(void *args)
{
    int32_t status;
    /*Disable IEP0 counter*/
    status = PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    Pinmux_config(gPinMuxMainDomainCfg1, PINMUX_DOMAIN_ID_MAIN);

    /*Intialize IEP0 count value*/
    PRUICSS_PWM_setIepCounterLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
    PRUICSS_PWM_setIepCounterUpper_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);

    /*configure cmp 0 value with APP_PRUICSS_PWM_PRD_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, CMP_EVENT0, (APP_PRUICSS_PWM_PRD_VAL & 0xFFFFFFFF));
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure cmp 6 value with APP_PRUICSS_IEP0_COMP6_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0,CMP_EVENT6, (APP_PRUICSS_IEP0_COMP6_VAL & 0xFFFFFFFF));
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable cmp 0 and cmp 6*/
    status = PRUICSS_PWM_configureIepCompareEnable(gPruIcssHandle, PRUICSS_IEP_INST0, 0x41);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Set IEP0 counter Increment value*/
    status = PRUICSS_setIepCounterIncrementValue(gPruIcssHandle, PRUICSS_IEP_INST0, PRUICSS_IEP_COUNT_INCREMENT_VALUE);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable cmp 0 reset of counter*/
    PRUICSS_PWM_configureIepCmp0ResetEnable(gPruIcssHandle, PRUICSS_IEP_INST0, 0x1);

    /*Enable IEP0 counter*/
    status = PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 1);
    DebugP_assert(SystemP_SUCCESS == status);
    
}

void pruicss_pwm_init(void *args)
{
    int32_t status;
    /*Enable IEP CMP flags to auto clear after state transition*/
    status = PRUICSS_PWM_configurePwmEfficiencyModeEnable(gPruIcssHandle, 1);
    DebugP_assert(SystemP_SUCCESS == status);
    /*Enable compare0 trip reset */
    PRUICSS_PWM_configurePwmCmp0TripResetEnable(gPruIcssHandle, PRUICSS_PWM_SET0, 1);
    /*configure PWM B2 signal of set 0, intial state to low*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(gPruIcssHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_INTIAL_STATE, PRUICSS_PWM_OUTPUT_LOW);
    DebugP_assert(SystemP_SUCCESS == status);
    /*configure PWM B2 signal of set 0, active state to high*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(gPruIcssHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_ACTIVE_STATE, PRUICSS_PWM_OUTPUT_HIGH);
    DebugP_assert(SystemP_SUCCESS == status);
}

void pruicss_pwm_duty_cycle_main(void *args)
{

     int32_t status;

     Drivers_open(); // check return status

     status = Board_driversOpen();
     DebugP_assert(SystemP_SUCCESS == status);

     gPruIcssHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
     DebugP_assert(gPruIcssHandle != NULL);

     pruicss_pwm_init(NULL);

     pruicss_iep_init(NULL);
    
     while (1)
     {
        ClockP_usleep(1);
     }

     Board_driversClose();
     Drivers_close();
}
