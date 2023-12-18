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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <pruicss_pwm.h>
#include <drivers/pruicss.h>
#include <drivers/pinmux.h>

/* Frequency of PWM output signal in Hz - 1 KHz is selected */
#define SOC_EPWM_OUTPUT_FREQ                (1U * 1000U)
/* TB frequency in Hz - so that /4 divider is used */
#define SOC_EPWM_TB_FREQ                    (CONFIG_EPWM0_FCLK / 4U)
/* PRD value - this determines the period */
#define SOC_EPWM_PRD_VAL                    (SOC_EPWM_TB_FREQ / SOC_EPWM_OUTPUT_FREQ)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define SOC_EPWM_DUTY_CYCLE                 (25U)
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define SOC_EPWM_COMPA_VAL                  (SOC_EPWM_PRD_VAL - ((SOC_EPWM_DUTY_CYCLE *SOC_EPWM_PRD_VAL) / 100U))

/*FIXME: IEP0_CLK_FREQ macro to be included in driver_config.h sysconfig generated file*/
#define PRUICSS_IEP0_CLK_FREQ                      (200000000U)
/* Modify this to change the IEP counter increment value*/
#define PRUICSS_IEP_COUNT_INCREMENT_VALUE          (5U)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define APP_PRUICSS_PWM0_A0_DUTY_CYCLE             (50U)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define APP_PRUICSS_PWM3_B2_DUTY_CYCLE             (75U)
/* Frequency  of PWM output signal in Hz - 1 KHz is selected */
#define APP_PRUICSS_PWM_OUTPUT_FREQ                (SOC_EPWM_OUTPUT_FREQ)
/* PRD value - this determines the period */
#define APP_PRUICSS_PWM_PRD_VAL                    (((PRUICSS_IEP0_CLK_FREQ / APP_PRUICSS_PWM_OUTPUT_FREQ))*(PRUICSS_IEP_COUNT_INCREMENT_VALUE))
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define APP_PRUICSS_IEP0_COMP1_VAL                 (APP_PRUICSS_PWM_PRD_VAL-((APP_PRUICSS_PWM0_A0_DUTY_CYCLE*APP_PRUICSS_PWM_PRD_VAL)/100))
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define APP_PRUICSS_IEP1_COMP12_VAL                (APP_PRUICSS_PWM_PRD_VAL-((APP_PRUICSS_PWM3_B2_DUTY_CYCLE*APP_PRUICSS_PWM_PRD_VAL)/100))

/*FIXME: Add pinmux in  sysconfig generated file*/
Pinmux_PerCfg_t gPinMuxMainDomainCfg1[] = {

    /* PRU_ICSSG0_PWM0 pin config */
    /* PRG0_PWM0_A0 -> PRG0_PRU0_GPO12 (K1) */
    {
        PIN_PRG0_PRU0_GPO12,
        ( PIN_MODE(3) | PIN_PULL_DISABLE )
    },
    /* PRU_ICSSG0_PWM3 pin config */
    /* PRG0_PWM3_B2 -> PRG0_PRU0_GPO5 (F2) */
    {
        PIN_PRG0_PRU0_GPO5,
        ( PIN_MODE(3) | PIN_PULL_DISABLE )
    },
    {PINMUX_END, PINMUX_END}

};

/* Function Prototypes */
static void App_epwmConfig(uint32_t epwmBaseAddr, uint32_t epwmCh, uint32_t epwmFuncClk);
static void pruicss_iep_init(void *args);
static void pruicss_pwm_init(void *args);

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr;

/* Global Structure pointer holding PRUICSSG0 memory Map. */
PRUICSS_Handle gPruIcssHandle;

void pruicss_pwm_epwm_sync_main(void *args)
{

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    gPruIcssHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    DebugP_assert(gPruIcssHandle != NULL);

    Pinmux_config(gPinMuxMainDomainCfg1, PINMUX_DOMAIN_ID_MAIN);

    pruicss_pwm_init(NULL);

    pruicss_iep_init(NULL);

    /* Address translate */
    gEpwmBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    /* Configure SOC EPWM */
    App_epwmConfig(gEpwmBaseAddr, EPWM_OUTPUT_CH_A, CONFIG_EPWM0_FCLK);

    while (1)
    {
       ClockP_usleep(1);
    }

    Board_driversClose();
    Drivers_close();
}

static void App_epwmConfig(uint32_t epwmBaseAddr, uint32_t epwmCh, uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, SOC_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, SOC_EPWM_TB_FREQ, SOC_EPWM_OUTPUT_FREQ, EPWM_TB_COUNTER_DIR_UP, EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A, SOC_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE, EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    aqConfig.zeroAction = EPWM_AQ_ACTION_LOW;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &aqConfig);

    /* Configure Dead Band Submodule */
    EPWM_deadbandBypass(epwmBaseAddr);

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    EPWM_etIntrCfg(epwmBaseAddr, EPWM_ET_INTR_EVT_CNT_EQ_ZRO, EPWM_ET_INTR_PERIOD_FIRST_EVT);
    EPWM_etIntrEnable(epwmBaseAddr);
}

static void pruicss_iep_init(void *args)
{

    int status;
    /*Disable IEP0 counter*/
    status= PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_enableIEPResetOnEPWM0SyncOut(gPruIcssHandle, PRUICSS_IEP_INST0, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable  IEP1 slave mode*/
    status = PRUICSS_PWM_enableIEP1Slave(gPruIcssHandle, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Intialize IEP0 count value*/
    PRUICSS_PWM_setIepCounterLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
    PRUICSS_PWM_setIepCounterUpper_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, 0xFFFFFFFF);

    /*FIXME: Either compare event is not hit or no state transition when compare 0 configured with 0x00000000*/
    /*configure cmp 0 value of IEP0 with APP_PRUICSS_PWM_PRD_VAL*/
    status = PRUICSS_PWM_setIepCompareEventLower_32bitValue(gPruIcssHandle, PRUICSS_IEP_INST0, CMP_EVENT0, ((0x00000001) & 0xFFFFFFFF));
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

    /*Disable cmp 0 reset of IEP0 counter*/
    status = PRUICSS_PWM_configureIepCmp0ResetEnable(gPruIcssHandle, PRUICSS_IEP_INST0, 0x0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable IEP0 counter*/
    status = PRUICSS_controlIepCounter(gPruIcssHandle, PRUICSS_IEP_INST0, 1);
    DebugP_assert(SystemP_SUCCESS == status);

}

static void pruicss_pwm_init(void *args){

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

