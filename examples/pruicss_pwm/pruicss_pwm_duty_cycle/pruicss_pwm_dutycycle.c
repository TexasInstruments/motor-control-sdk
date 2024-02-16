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

/*
 * This example uses the PRUICSS PWM module to generate a signal
 * with a specified duty cycle.
 *
 * The default parameters are : Frequency : 1kHz, Duty cycle : 25%,
 * All these parameters are configurable.
 *
 * In this example PWM0_0_POS(alias signal PWM0_A0),PWM3_2_NEG(alias signal PWM3_B2) is used to generate the signal, the user can also
 * select a different one.
 *  
 * PWM0_0_POS(alias signal PWM0_A0) uses IEP0 CMP1 EVENT to control Duty cycle
 * & IEP0 CMP0 to control output Frequency
 *
 * PWM3_2_NEG(alias signal PWM3_B2) uses IEP1 CMP12 EVENT to control Duty cycle
 * & IEP0 CMP0 to control output Frequency
 *
 * This example also showcases how to configure and use the PRUICSS PWM module.
 */
#if defined(am243x_evm) || defined(am64x_evm)
#include <board/ioexp/ioexp_tca6424.h>

static TCA6424_Config  gTCA6424_Config;

static void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;
    TCA6424_Params_init(&tca6424Params);
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
    uint32_t            ioIndex;

    if(status == SystemP_SUCCESS)
    {
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
    }
    TCA6424_close(&gTCA6424_Config);
}
#endif
/** \brief Global Structure pointer holding PRUICSSG0 memory Map. */
PRUICSS_Handle gPruIcssHandle;

/*FIXME: IEP0_CLK_FREQ macro to be included in driver_config.h sysconfig generated file*/
#define PRUICSS_IEP0_CLK_FREQ                      (200000000U)
/* Modify this to change the IEP counter increment value*/
#define PRUICSS_IEP_COUNT_INCREMENT_VALUE          (1U)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define APP_PRUICSS_PWM3_B2_DUTY_CYCLE             (25U)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define APP_PRUICSS_PWM0_A0_DUTY_CYCLE             (25U)
/* Frequency  of PWM output signal in Hz - 1 KHz is selected */
#define APP_PRUICSS_PWM_OUTPUT_FREQ                (1U * 1000U)
/* PRD value - this determines the period */
#define APP_PRUICSS_PWM_PRD_VAL                    (((PRUICSS_IEP0_CLK_FREQ / APP_PRUICSS_PWM_OUTPUT_FREQ))*(PRUICSS_IEP_COUNT_INCREMENT_VALUE))
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define APP_PRUICSS_IEP0_COMP1_VAL                 (APP_PRUICSS_PWM_PRD_VAL-((APP_PRUICSS_PWM0_A0_DUTY_CYCLE*APP_PRUICSS_PWM_PRD_VAL)/100))
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define APP_PRUICSS_IEP1_COMP12_VAL                (APP_PRUICSS_PWM_PRD_VAL-((APP_PRUICSS_PWM3_B2_DUTY_CYCLE*APP_PRUICSS_PWM_PRD_VAL)/100))

void pruicss_iep_init(void *args)
{

    int status;
    /*Disable IEP0 counter*/
    status= PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable  IEP1 slave mode*/
    status = PRUICSS_PWM_enableIEP1Slave(gPruIcssHandle, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Intialize IEP0 count value*/
    PRUICSS_PWM_setIepCounterLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
    PRUICSS_PWM_setIepCounterUpper_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);

    /*configure cmp 0 value of IEP0 with APP_PRUICSS_PWM_PRD_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, CMP_EVENT0, (APP_PRUICSS_PWM_PRD_VAL & 0xFFFFFFFF));
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure cmp 1 value with APP_PRUICSS_IEP0_COMP1_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, CMP_EVENT1, (APP_PRUICSS_IEP0_COMP1_VAL & 0xFFFFFFFF));
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure cmp 12 value with APP_PRUICSS_IEP1_COMP12_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST1, CMP_EVENT12, (APP_PRUICSS_IEP1_COMP12_VAL & 0xFFFFFFFF));
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable cmp 0 and cmp 1 of IEP0*/
    status = PRUICSS_PWM_configureIepCompareEnable(gPruIcssHandle, PRUICSS_IEP_INST0, 0x3);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable cmp12 of IEP1*/
    status = PRUICSS_PWM_configureIepCompareEnable(gPruIcssHandle, PRUICSS_IEP_INST1, 0x1000);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Set IEP0 counter Increment value*/
    status = PRUICSS_setIepCounterIncrementValue(gPruIcssHandle, PRUICSS_IEP_INST0, PRUICSS_IEP_COUNT_INCREMENT_VALUE);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable cmp 0 reset of IEP0 counter*/
    status = PRUICSS_PWM_configureIepCmp0ResetEnable(gPruIcssHandle, PRUICSS_IEP_INST0, 0x1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable IEP0 counter*/
    status = PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 1);
    DebugP_assert(SystemP_SUCCESS == status);
    
}

void pruicss_pwm_init(void *args){

    int status;
    /*Enable IEP CMP flags to auto clear after state transition*/
    status = PRUICSS_PWM_configurePwmEfficiencyModeEnable(gPruIcssHandle, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable compare0 trip reset of set 0*/
    status = PRUICSS_PWM_configurePwmCmp0TripResetEnable(gPruIcssHandle, PRUICSS_PWM_SET0, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable compare0 trip reset of set 3 */
    status = PRUICSS_PWM_configurePwmCmp0TripResetEnable(gPruIcssHandle, PRUICSS_PWM_SET3, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure PWM B2 signal of set 0, intial state to low*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(gPruIcssHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_INTIAL_STATE,1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure PWM B2 signal of set 0, active state to high*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(gPruIcssHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_ACTIVE_STATE,2);
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure PWM B2 signal of set 0, intial state to low*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(gPruIcssHandle, PRUICSS_PWM_SET3, PRUICSS_PWM_INTIAL_STATE,1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*configure PWM B2 signal of set 0, active state to high*/
    status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(gPruIcssHandle, PRUICSS_PWM_SET3, PRUICSS_PWM_ACTIVE_STATE,2);
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

     #if defined(am243x_evm) || defined(am64x_evm)
     /* Configure the IO Expander to connect the PRU IOs to HSE */
     i2c_io_expander(NULL);
     #endif

     pruicss_pwm_init(NULL);

     pruicss_iep_init(NULL);
    
     while (1)
     {
        ClockP_usleep(1);
     }

     Board_driversClose();
     Drivers_close();
}
