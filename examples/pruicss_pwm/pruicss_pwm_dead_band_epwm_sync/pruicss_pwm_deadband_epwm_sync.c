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
#include <drivers/pinmux.h>
#include <stdlib.h>
#include <board/ioexp/ioexp_tca6424.h>
#include "app_pruicss_pwm.h"

/* Frequency of PWM output signal in Hz - 16 KHz is selected */
#define SOC_EPWM_OUTPUT_FREQ                                (CONFIG_PRUICSS_PWM_INSTANCE0_FREQ_HZ)
/* TB frequency in Hz - so that /4 divider is used */
#define SOC_EPWM_TB_FREQ                                    (CONFIG_EPWM0_FCLK / 4U)
/* PRD value - this determines the period */
#define SOC_EPWM_PRD_VAL                                    ((SOC_EPWM_TB_FREQ / SOC_EPWM_OUTPUT_FREQ)/2)
/* Duty Cycle of PWM output signal in % - give value from 1 to 99 */
#define SOC_EPWM_DUTY_CYCLE                                 (25U)
/* DUTY CYCLE width - this determines width of PWM output signal duty cycle*/
#define SOC_EPWM_COMPA_VAL                                  (SOC_EPWM_PRD_VAL - ((SOC_EPWM_DUTY_CYCLE *SOC_EPWM_PRD_VAL) / 100U))

/*
 * This example uses the PRUICSS PWM module to generate a signal
 * with a specified duty cycle and deadband at rise edge and fall edge of pwm
 *
 * The default parameters are in the example are: 
 * 
 * Frequency : 16kHz  
 * PWM0_0_POS(alias signal PWM0_A0) is configured with duty cycle of 25% , rise edge delay as 0ns and fall edge delay as 0ns 
 * PWM0_0_NEG(alias signal PWM0_B0) is configured with duty cycle of 25% , rise edge delay as 200ns and fall edge delay as 400ns 
 * PWM2_0_POS(alias signal PWM2_A0) is configured with duty cycle of 75% , rise edge delay as 0ns and fall edge delay as 0ns 
 * PWM2_0_NEG(alias signal PWM2_B0) is configured with duty cycle of 75% , rise edge delay as 600ns and fall edge delay as 800ns 
 * All these parameters are configurable.
 * 
 * PWM0_0_POS, PWM0_0_NEG, PWM2_0_POS, PWM2_0_NEG uses IEP0 CMP0 & EPWM0 sync out signal to achieve pwm synchronization 
 * 
 * Individual compare event mapped to PWM_x signals controls duty cycle
 * 
 * Code Flow :-  
 * 
 * PRUICSS IEP configuration:
 * 
 * IEP shadow mode and slave mode are enabled (Refer section 6.4.13 of Technical Reference Manual) & IEP is configured to reset twice on every pwm period as mentioned below
 * 
 * EPWM0 sync out is configured to generate every PRUICSS PWM period, PRUICSS IEP COMPARE 0 is configured with one IEP cycle delay ((PWM_PERIOD/2)+1). 
 * By configuring PRUICSS IEP COMPARE 0 with one IEP cycle delay, IEP COMPARE 0 event is missed at the end of PRUICSS PWM period.
 * PRUICSS IEP CMP0 resets IEP couter in middle of PRUICSS PWM period.
 * EPWM0 sync out resets IEP counter at end of PRUICSS PWM period.   
 * 
 * PRUICSS PWM configuration:
 * 
 * PWM signal is configured to low in Intial state, Toggle in Active state, Change of state from Active to Intial is disabled on IEP0 CMP0 event
 * 
 * PWM Duty cycle, rise edge delay, fall edge delay can configured or updated using PRUICSS_PWM_config api call.
 * 
 * PWM Period can be Configured or updated using PRUICSS_PWM_pruIcssPwmFrequencyInit api call. 
 * 
 * Below two steps are executed once in pwm period: 
 * 
 * Step1 : When EPWM0 sync out resets IEP counter, PWM signals are moved to intial state on software reset
 * shadow compare register values which decides rise edge are moved to active register values, 
 * next compare value which decides fall edge of PWM signal are computed & shadow compare register field is updated
 * 
 * Step2 : When PRUICSS IEP CMP0 resets IEP counter, 
 * shadow compare register values which decides fall edge are moved to active register values,
 * new compare values which decides rise edge of PWM signal are computed from updated duty cycle 
 * & shadow compare register field is updated
 * 
 * The pruicss pwm signal generated is similar to epwm signal when epwm counter is configured in up-down mode.
 * 
 * This example showcases deadband feature of pruicss pwm, syncing pruicss pwm with epwm sync cout
 * 
 * Note: This example uses EPWM0 sync out to reset IEP at the pwm period, PRUICSS IEP CMP0 can also be used to do this when EPWM sync out signal is disabled.
 */

/* variable to hold base address of EPWM that is used */
uint32_t gEpwmBaseAddr;

/* Global Structure pointer holding PRUICSSG0 memory Map. */
PRUICSS_PWM_Handle gPruIcssPwmHandle;
PRUICSS_Handle     gpruIcssHandle;

uint8_t gUpdateNextRisingEdgeCmpValue  = 0;

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

void App_epwmConfig(uint32_t epwmBaseAddr, uint32_t epwmCh, uint32_t epwmFuncClk)
{
    EPWM_AqActionCfg  aqConfig;

    /* Configure Time base submodule */
    EPWM_tbTimebaseClkCfg(epwmBaseAddr, SOC_EPWM_TB_FREQ, epwmFuncClk);
    EPWM_tbPwmFreqCfg(epwmBaseAddr, SOC_EPWM_TB_FREQ, SOC_EPWM_OUTPUT_FREQ, EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
    EPWM_tbSyncDisable(epwmBaseAddr);
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO);
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A, SOC_EPWM_COMPA_VAL, EPWM_SHADOW_REG_CTRL_ENABLE, EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    aqConfig.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
    aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
    aqConfig.cmpADownAction = EPWM_AQ_ACTION_LOW;
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

void App_pruIcssPwmDeadbandMain(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    int status;

    gpruIcssHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    DebugP_assert(gpruIcssHandle != NULL);

    gPruIcssPwmHandle = PRUICSS_PWM_open(CONFIG_PRUICSS_PWM0, gpruIcssHandle);
    DebugP_assert(gPruIcssPwmHandle != NULL);
    
    #if defined(am243x_evm) || defined(am64x_evm)
    /* Configure the IO Expander to connect the PRU IOs to HSE */
    i2c_io_expander(NULL);
    #endif

    /*Intializes the pwm parameters with default values, output in intial, active, trip states and disables all pwm signals*/
    status = PRUICSS_PWM_attrsInit(gPruIcssPwmHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Intializes the output state of pwm signals in intial, active, trip states from the parameters specified*/
    status = PRUICSS_PWM_stateInit(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_A0, PRUICSS_PWM_OUTPUT_LOW, PRUICSS_PWM_OUTPUT_TOGGLE, PRUICSS_PWM_OUTPUT_LOW);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_stateInit(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_B0, PRUICSS_PWM_OUTPUT_LOW, PRUICSS_PWM_OUTPUT_TOGGLE, PRUICSS_PWM_OUTPUT_LOW);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_stateInit(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_A0, PRUICSS_PWM_OUTPUT_LOW, PRUICSS_PWM_OUTPUT_TOGGLE, PRUICSS_PWM_OUTPUT_LOW);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_stateInit(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_B0, PRUICSS_PWM_OUTPUT_LOW, PRUICSS_PWM_OUTPUT_TOGGLE, PRUICSS_PWM_OUTPUT_LOW);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enabling below pwm signals*/
    status = PRUICSS_PWM_signalEnable(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_A0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_signalEnable(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_B0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_signalEnable(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_A0);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_signalEnable(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_B0);
    DebugP_assert(SystemP_SUCCESS == status);

    /*If pwm signal is enabled ,configures output state defined in gPruIcssPwmCfg*/
    status = PRUICSS_PWM_stateConfig(gPruIcssPwmHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /*If pwm signal is enabled,configures pwm period & duty cycle*/
    status = PRUICSS_PWM_config(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_A0, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_A0_DUTY_CYCLE, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_A0_RISE_EDGE_DELAY_IN_NSEC, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_A0_FALL_EDGE_DELAY_IN_NSEC);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_config(gPruIcssPwmHandle, PRUICSS_PWM_SET0, PRUICSS_PWM_SET_INSTANCE_B0, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_B0_DUTY_CYCLE, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_B0_RISE_EDGE_DELAY_IN_NSEC, CONFIG_PRUICSS_PWM0_PWM_SET0_INSTANCE_B0_FALL_EDGE_DELAY_IN_NSEC);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_config(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_A0, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_A0_DUTY_CYCLE, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_A0_RISE_EDGE_DELAY_IN_NSEC, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_A0_FALL_EDGE_DELAY_IN_NSEC);
    DebugP_assert(SystemP_SUCCESS == status);

    status = PRUICSS_PWM_config(gPruIcssPwmHandle, PRUICSS_PWM_SET2, PRUICSS_PWM_SET_INSTANCE_B0, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_B0_DUTY_CYCLE, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_B0_RISE_EDGE_DELAY_IN_NSEC, CONFIG_PRUICSS_PWM0_PWM_SET2_INSTANCE_B0_FALL_EDGE_DELAY_IN_NSEC);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Intilizes or Updates pruicss pwm frequency to specified value*/
    status = PRUICSS_PWM_pruIcssPwmFrequencyInit(gPruIcssPwmHandle, CONFIG_PRUICSS_PWM_INSTANCE0_FREQ_HZ);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Configures the Iep parameters defined in gPruIcssPwmIepCfg*/
    status = PRUICSS_PWM_iepConfig(gPruIcssPwmHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Enable IEP CMP0/CMP1 compare events*/
    if((gPruIcssPwmHandle != NULL) && (gPruIcssPwmHandle->iepAttrs != NULL) && ((gPruIcssPwmHandle->iepAttrs)->enableIep0) == 1U)
    {
        /*Enable IEP0 CMP0/CMP1 compare events*/
        status = PRUICSS_PWM_configureIepCompareEnable(gPruIcssPwmHandle, PRUICSS_IEP_INST0,  0xFFFF);
        DebugP_assert(SystemP_SUCCESS == status);

        if((gPruIcssPwmHandle->iepAttrs)->enableIep1SlaveMode == 1U)
        {
            /*Enable IEP1 CMP0/CMP1 compare events*/
            status = PRUICSS_PWM_configureIepCompareEnable(gPruIcssPwmHandle, PRUICSS_IEP_INST1,  0xFFFF);
            DebugP_assert(SystemP_SUCCESS == status);
        }

    }
    

    /* This IEP1 CMP0 ISR controls the duty cycle and dead band at fall edge of PWM signals*/
    status = App_pruicssIep1Compare0IrqSet(gPruIcssPwmHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Address translate */
    gEpwmBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);

    /* This EPWM SYNC ISR controls the period of PRUICSS PWM signals, duty cycle and dead band at rise edge of PWM signals */
    status = App_epwm0Sync0IrqSet(gPruIcssPwmHandle, gEpwmBaseAddr);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Configure SOC EPWM */
    App_epwmConfig(gEpwmBaseAddr, EPWM_OUTPUT_CH_A, CONFIG_EPWM0_FCLK);

    /*Enable IEP counter*/
    status = PRUICSS_controlIepCounter(gPruIcssPwmHandle->pruIcssHandle, PRUICSS_IEP_INST0, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    while(1)
    {
        /*wait until IEP reset on compare 0 event, fall edge compare values are loaded from shadow to active compare registers*/
        while (gUpdateNextRisingEdgeCmpValue  == 0)
        {
           ClockP_usleep(1);
        }
        /*update next rising edge compare values*/
        status = App_updateNextRisingEdgeCmpValue(gPruIcssPwmHandle);
        DebugP_assert(SystemP_SUCCESS == status);
        gUpdateNextRisingEdgeCmpValue  = 0;
    }

    status = App_sciclientCmpEventRouterIrqRelease(gPruIcssPwmHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    Board_driversClose();
    Drivers_close();
}

