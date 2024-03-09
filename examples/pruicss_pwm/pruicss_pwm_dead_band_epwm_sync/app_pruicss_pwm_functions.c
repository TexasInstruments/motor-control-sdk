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
#include "stdlib.h"
#include <drivers/pinmux.h>
#include "app_pruicss_pwm.h"
#include <board/ioexp/ioexp_tca6424.h>

/* Source Index of Icssg0 iep1 compare 0*/
#define TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX                (28U)
/* Source Index of Icssg1 iep1 compare 0*/
#define TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX                (28U)

/*FIXME: IEP0_CLK_FREQ macro to be included in driver_config.h sysconfig generated file*/
#define PRUICSS_IEP0_CLK_FREQ                               (200000000U)
/*IEP CLK period in nano seconds*/
#define PRUICSS_IEP0_CLK_PERIOD_IN_NANOSECONDS              (5)
/* Modify this to change the IEP counter increment value*/
#define PRUICSS_IEP_COUNT_INCREMENT_VALUE                   (1U)

extern uint8_t gUpdateNextRisingEdgeCmpValue;

#if defined(am243x_evm) || defined(am64x_evm)

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

void App_pruIcssPwmIepInit(AppPruIcssPwmIepCfg_t *pruIcssPwmIepCfg, uint32_t PruIcssPwmFrequency)
{
    pruIcssPwmIepCfg->pruIcssPwmFrequency = PruIcssPwmFrequency;
    pruIcssPwmIepCfg->enableIep0 = 1U;
    pruIcssPwmIepCfg->enableIEP0ShadowMode = 1U;
    pruIcssPwmIepCfg->enableIep0ResetOnEpwm0_Sync = 1U;
    pruIcssPwmIepCfg->enableIep0ResetOnEpwm3_Sync = 0U;
    pruIcssPwmIepCfg->enableIep0ResetOnCompare0 = 1U;
    pruIcssPwmIepCfg->enableIep1SlaveMode = 1U;
    pruIcssPwmIepCfg->enableAutoClearCompareStatus = 1U;
    pruIcssPwmIepCfg->iep0IncrementValue = PRUICSS_IEP_COUNT_INCREMENT_VALUE;
}

void App_pruIcssPwmIepConfig(PRUICSS_Handle handle, AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg)
{

    int status;

    /* compare0_val is calculated based on pwm period */
    uint32_t compare0_val = ((PRUICSS_IEP0_CLK_FREQ/(pruIcssPwmIepCfg.pruIcssPwmFrequency))*PRUICSS_IEP_COUNT_INCREMENT_VALUE);

    /*Disable IEP0 counter*/
    status= PRUICSS_controlIepCounter(handle, PRUICSS_IEP_INST0, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Intialize IEP0 count value*/
    status= PRUICSS_PWM_setIepCounterLower_32bitValue(handle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
    DebugP_assert(SystemP_SUCCESS == status);
    status= PRUICSS_PWM_setIepCounterUpper_32bitValue(handle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
    DebugP_assert(SystemP_SUCCESS == status);


    if(pruIcssPwmIepCfg.enableIep0 == 1U)
    {
        if(pruIcssPwmIepCfg.enableIEP0ShadowMode == 1U)
        {
            /*Enable or disable shadow mode of IEP*/
            status=PRUICSS_PWM_configureIepShadowModeEnable(handle, PRUICSS_IEP_INST0, 1);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmIepCfg.enableIep0ResetOnEpwm0_Sync == 1U)
        {
            /*Enable or disable EPWM0 sync out to reset IEP*/
            status = PRUICSS_PWM_enableIEPResetOnEPWM0SyncOut(handle, PRUICSS_IEP_INST0, 1);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmIepCfg.enableIep0ResetOnEpwm3_Sync == 1U)
        {
            /*Enable or disable EPWM3 sync out to reset IEP*/
            status = PRUICSS_PWM_enableIEPResetOnEPWM3SyncOut(handle, PRUICSS_IEP_INST0, 1);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmIepCfg.enableIep0ResetOnCompare0 == 1U)
        {
            /*Enable cmp 0 reset of IEP0 counter*/
            status = PRUICSS_PWM_configureIepCmp0ResetEnable(handle, PRUICSS_IEP_INST0, 0x1);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        /*Set IEP0 counter Increment value*/
        status = PRUICSS_setIepCounterIncrementValue(handle, PRUICSS_IEP_INST0, pruIcssPwmIepCfg.iep0IncrementValue);
        DebugP_assert(SystemP_SUCCESS == status);

         /*Enable IEP0 compare events*/
        status = PRUICSS_PWM_configureIepCompareEnable(handle, PRUICSS_IEP_INST0, 0xFFFF);
        DebugP_assert(SystemP_SUCCESS == status);

        /*Configure IEP0 compare 0 event to reset on (pruicss pwm period/2) with one clock cycle delay*/
        status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST0, CMP_EVENT0, ((compare0_val/2) +1));
        DebugP_assert(SystemP_SUCCESS == status);

         /*Configure IEP0 compare 0 event to reset on (pruicss pwm period/2) with one clock cycle delay*/
        status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST1, CMP_EVENT0, ((compare0_val/2) +1));
        DebugP_assert(SystemP_SUCCESS == status);

        if(pruIcssPwmIepCfg.enableIep1SlaveMode == 1U)
        {
            /*Enable  IEP1 slave mode*/
            status = PRUICSS_PWM_enableIEP1Slave(handle, 1);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable shadow mode of IEP1*/
            status=PRUICSS_PWM_configureIepShadowModeEnable(handle, PRUICSS_IEP_INST1, 1);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable cmp 0 reset of IEP1 counter*/
            status = PRUICSS_PWM_configureIepCmp0ResetEnable(handle, PRUICSS_IEP_INST1, 0x1);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable IEP1 compare events*/
            status = PRUICSS_PWM_configureIepCompareEnable(handle, PRUICSS_IEP_INST1, 0xFFFF);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmIepCfg.enableAutoClearCompareStatus == 1U)
        {
            /*Enable IEP CMP flags to auto clear after state transition*/
            status = PRUICSS_PWM_configurePwmEfficiencyModeEnable(handle, 1);
            DebugP_assert(SystemP_SUCCESS == status);
        }
    }

}

void App_pruIcssPwmInit(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET])
{
    /* Configure the IO Expander to connect the PRU IOs to HSE */
    i2c_io_expander(NULL);

    uint8_t compareEvent = CMP_EVENT1, iepInstance = PRUICSS_IEP_INST0, currentPwmInstance, currentPwmSet;

    /*Intializes the pwm parameters with default values & maps compare events, output in intial, active, trip states and disables all pwm signals*/
    for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet<=PRUICSS_PWM_SET1; currentPwmSet++)
    {
        for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
        {
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].iepInstance =  iepInstance;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].compareEvent = compareEvent;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgTripState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgActiveState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgInitialState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].fallEdgeDelay = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].riseEdgeDelay = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].dutyCycle = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].enable = 0;
            compareEvent++;
        }
    }

    compareEvent = CMP_EVENT1;
    iepInstance = PRUICSS_IEP_INST1;

    for(currentPwmSet=PRUICSS_PWM_SET2; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
    {
        for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
        {
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].iepInstance =  iepInstance;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].compareEvent = compareEvent;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgTripState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgActiveState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].outputCfgInitialState = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].fallEdgeDelay = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].riseEdgeDelay = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].dutyCycle = 0;
            pruIcssPwmCfg[currentPwmSet][currentPwmInstance].enable = 0;
            compareEvent++;
        }
    }

}

void App_pruIcssPwmCfg(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], uint8_t pwmSet, uint8_t instance,  uint32_t dutyCycle, uint32_t riseEdgeDelay, uint32_t fallEdgeDelay)
{
    /*Updates parameters of pwm signal with specified duty cycle, fall edge & rise edge delay*/
    if((pwmSet < PRUICSS_NUM_PWM_SETS) && (instance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET))
    {
        pruIcssPwmCfg[pwmSet][instance].dutyCycle = dutyCycle;
        pruIcssPwmCfg[pwmSet][instance].riseEdgeDelay = riseEdgeDelay;
        pruIcssPwmCfg[pwmSet][instance].fallEdgeDelay = fallEdgeDelay;
        pruIcssPwmCfg[pwmSet][instance].enable = 1;
    }
}

void App_pruIcssPwmStateInit(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], uint8_t pwmSet, uint8_t instance, uint8_t outputCfgInitialState, uint8_t outputCfgActiveState, uint8_t outputCfgTripState)
{
    if((pwmSet < PRUICSS_NUM_PWM_SETS) && (instance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET))
    {
        pruIcssPwmCfg[pwmSet][instance].outputCfgTripState = outputCfgTripState;
        pruIcssPwmCfg[pwmSet][instance].outputCfgActiveState = outputCfgActiveState;
        pruIcssPwmCfg[pwmSet][instance].outputCfgInitialState = outputCfgInitialState;
    }
}

void App_pruIcssPwmStateConfig(PRUICSS_Handle handle, AppPruIcssPwmCfg_t pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg)
{
    int status;
    for(uint8_t currentPwmSet=0; currentPwmSet< PRUICSS_NUM_PWM_SETS; currentPwmSet++)
    {
        if(pruIcssPwmCfg[currentPwmSet][0].enable == 1)
        {

            /*configure PWM A0 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][0].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][0].outputCfgActiveState );
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][0].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmCfg[currentPwmSet][1].enable == 1)
        {
            /*configure PWM B0 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][1].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][1].outputCfgActiveState );
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][1].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmCfg[currentPwmSet][2].enable == 1)
        {
            /*configure PWM A1 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][2].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][2].outputCfgActiveState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][2].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmCfg[currentPwmSet][3].enable == 1)
        {
            /*configure PWM B1 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][3].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][3].outputCfgActiveState );
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][3].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmCfg[currentPwmSet][4].enable == 1)
        {
            /*configure PWM A2 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][4].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][4].outputCfgActiveState );
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][4].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }

        if(pruIcssPwmCfg[currentPwmSet][5].enable == 1)
        {
            /*configure PWM B2 signal of intial, active, trip states*/
            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, pruIcssPwmCfg[currentPwmSet][5].outputCfgInitialState);
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, pruIcssPwmCfg[currentPwmSet][5].outputCfgActiveState );
            DebugP_assert(SystemP_SUCCESS == status);

            status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, pruIcssPwmCfg[currentPwmSet][5].outputCfgTripState);
            DebugP_assert(SystemP_SUCCESS == status);
        }
    }

}

void App_updateNextRisingEdgeCmpValue(PRUICSS_Handle handle, AppPruIcssPwmCfg_t pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg)
{

    int status;

    /* compare0_val is calculated based on pwm period */
    uint32_t compare0_val = ((PRUICSS_IEP0_CLK_FREQ/(pruIcssPwmIepCfg.pruIcssPwmFrequency))*PRUICSS_IEP_COUNT_INCREMENT_VALUE);

    /*div by 2 is done to update compare values half of the pwm period, as IEP cannot be configured in up-down mode*/
    compare0_val = (compare0_val)/2;

    uint32_t compare_val;
    uint8_t  currentPwmInstance, currentPwmSet;

    /*Next compare value which decides rising edge of PWM signal are computed & shadow compare register field is updated*/
    for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
    {
        for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
        {
           if(pruIcssPwmCfg[currentPwmSet][currentPwmInstance].enable == 1 )
           {

                compare_val = compare0_val - (((pruIcssPwmCfg[currentPwmSet][currentPwmInstance].dutyCycle)*(compare0_val))/100);

                compare_val = compare_val + (((pruIcssPwmCfg[currentPwmSet][currentPwmInstance].riseEdgeDelay)*(PRUICSS_IEP_COUNT_INCREMENT_VALUE))/PRUICSS_IEP0_CLK_PERIOD_IN_NANOSECONDS);

                /*configure cmp  value of current pwm*/
                status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, pruIcssPwmCfg[currentPwmSet][currentPwmInstance].iepInstance, pruIcssPwmCfg[currentPwmSet][currentPwmInstance].compareEvent, compare_val);
                DebugP_assert(SystemP_SUCCESS == status);
           }

        }
    }

    /*configure cmp  value of current pwm*/
    status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST0, CMP_EVENT0, compare0_val + 1);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST1, CMP_EVENT0, compare0_val + 1);
    DebugP_assert(SystemP_SUCCESS == status);

}

void App_sciclientCmpEventRouterIrqset(PRUICSS_Handle handle)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

    int32_t                             retVal;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;
    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;

    if(hwAttrs->instance == 0)
    {
        rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG0;
        rmIrqReq.src_index              = TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX;
    }
    else if(hwAttrs->instance == 1)
    {
        rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG1;
        rmIrqReq.src_index              = TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX;
    }

    rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
    rmIrqReq.dst_host_irq           = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
    if(0 != retVal)
    {
        DebugP_log("[Error] Sciclient event config failed!!!\r\n");
        DebugP_assert(FALSE);
    }
    return;
}

void App_sciclientCmpEventRouterIrqRelease(PRUICSS_Handle handle)
{
    int32_t                                 retVal;
    struct tisci_msg_rm_irq_release_req     rmIrqReq;

    PRUICSS_HwAttrs const   *hwAttrs;
    hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
    rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.global_event           = 0U;
    if(hwAttrs->instance == 0)
    {
        rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG0;
        rmIrqReq.src_index              = TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX;
    }
    else if(hwAttrs->instance == 1)
    {
        rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG1;
        rmIrqReq.src_index              = TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX;
    }
    rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
    rmIrqReq.dst_host_irq           = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    retVal = Sciclient_rmIrqRelease(&rmIrqReq, SystemP_WAIT_FOREVER);
    if(0 != retVal)
    {
        DebugP_log("[Error] Sciclient event reset failed!!!\r\n");
        DebugP_assert(FALSE);
    }
    return;
}

void App_pruicssIep1Compare0IrqSet(PRUICSS_Handle handle)
{
    int32_t         retVal;
    HwiP_Params     hwiPrms;
    HwiP_Object     compHwiObject;

    App_sciclientCmpEventRouterIrqset(handle);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
    hwiPrms.callback = &App_pruIcssPwmHalfDoneIrq;
    retVal = HwiP_construct(&compHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );
}

void App_epwm0Sync0IrqSet(PRUICSS_Handle handle, AppPruIcssPwmCfg_t pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg, uint32_t epwmBaseAddr)
{
    int32_t         retVal;

    HwiP_Params     hwiPrms;
    HwiP_Object     compHwiObject;

    AppEpwmSync0IrqArgs_t *AppEpwmSync0IrqArgs;

    AppEpwmSync0IrqArgs = (AppEpwmSync0IrqArgs_t*)malloc(sizeof(AppEpwmSync0IrqArgs_t));

    AppEpwmSync0IrqArgs->EpwmBaseAddr = epwmBaseAddr;
    AppEpwmSync0IrqArgs->pruIcssPwmCfg = pruIcssPwmCfg;
    AppEpwmSync0IrqArgs->pruIcssPwmIepCfg = pruIcssPwmIepCfg;
    AppEpwmSync0IrqArgs->pruIcssHandle = handle;

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = CONFIG_EPWM0_INTR;
    hwiPrms.callback = &App_epwmSync0Irq;
    hwiPrms.args     = (void *)(AppEpwmSync0IrqArgs);
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    retVal = HwiP_construct(&compHwiObject, &hwiPrms);
    DebugP_assert(retVal == SystemP_SUCCESS );

}

void App_pruIcssPwmHalfDoneIrq()
{
    gUpdateNextRisingEdgeCmpValue  = 1;
}

void App_changePruIcssPwmToIntialState(PRUICSS_Handle handle, uint8_t pwmSetMask)
{
    if(pwmSetMask & 0x1)
    {
        /*
        * generate trip reset status of pwm set 0
        */
        PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET0);
        /*
        * clear trip reset status of pwm set 0
        */
       PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET0);
    }    

    if(pwmSetMask & 0x2)
    {
        /*
        * generate trip reset status of pwm set 1
        */
        PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET1);
        /*
        * clear trip reset status of pwm set 1
        */
       PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET1);
    }  

    if(pwmSetMask & 0x4)
    {
        /*
        * generate trip reset status of pwm set 2
        */
        PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET2);
        /*
        * clear trip reset status of pwm set 2
        */
       PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET2);
    } 

    if(pwmSetMask & 0x8)
    {
        /*
        * generate trip reset status of pwm set 3
        */
        PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET3);
        /*
        * clear trip reset status of pwm set 3
        */
       PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET3);
    }   
}

void App_epwmSync0Irq(void *args)
{
    volatile uint16_t status;

    AppEpwmSync0IrqArgs_t *AppEpwmSync0IrqArgs = (AppEpwmSync0IrqArgs_t *)args;

    status = EPWM_etIntrStatus(AppEpwmSync0IrqArgs->EpwmBaseAddr);
    if(status & EPWM_ETFLG_INT_MASK)
    {
        EPWM_etIntrClear(AppEpwmSync0IrqArgs->EpwmBaseAddr);
    }

    /*Change state of all pwm signals to intial*/
    App_changePruIcssPwmToIntialState(AppEpwmSync0IrqArgs->pruIcssHandle, 0xF);

    /* compare0_val is calculated based on pwm period */
    uint32_t compare0_val = ((PRUICSS_IEP0_CLK_FREQ/((AppEpwmSync0IrqArgs->pruIcssPwmIepCfg).pruIcssPwmFrequency))*PRUICSS_IEP_COUNT_INCREMENT_VALUE);

    /*div by 2 is done to update compare values half of the pwm period, as IEP cannot be configured in up-down mode*/
    compare0_val = (compare0_val)/2;

    uint32_t compare_val;
    uint8_t  currentPwmInstance, currentPwmSet;

    /*Next compare value which decides fall edge of PWM signal are computed & shadow compare register field is updated*/
    for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
    {
        for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
        {
           if((AppEpwmSync0IrqArgs->pruIcssPwmCfg[currentPwmSet][currentPwmInstance]).enable == 1)
           {

                compare_val = ((((AppEpwmSync0IrqArgs->pruIcssPwmCfg[currentPwmSet][currentPwmInstance]).dutyCycle)*(compare0_val))/100);

                compare_val = compare_val + (((AppEpwmSync0IrqArgs->pruIcssPwmCfg[currentPwmSet][currentPwmInstance].fallEdgeDelay)*(PRUICSS_IEP_COUNT_INCREMENT_VALUE))/PRUICSS_IEP0_CLK_PERIOD_IN_NANOSECONDS);

                /*configure cmp  value of current pwm*/
                status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(AppEpwmSync0IrqArgs->pruIcssHandle, (AppEpwmSync0IrqArgs->pruIcssPwmCfg[currentPwmSet][currentPwmInstance]).iepInstance, (AppEpwmSync0IrqArgs->pruIcssPwmCfg[currentPwmSet][currentPwmInstance]).compareEvent, compare_val);
           }

        }
    }
}
