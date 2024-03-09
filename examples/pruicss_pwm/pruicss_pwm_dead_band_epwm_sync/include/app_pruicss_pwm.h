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

#include <pruicss_pwm.h>

typedef struct
{
    /*pwm frequency*/
    uint32_t pruIcssPwmFrequency;
    /*enable or disable IEP0*/
    uint8_t  enableIep0;
    /*enable or disable IEP0 reset on EPWM0_sync event*/
    uint8_t  enableIep0ResetOnEpwm0_Sync;
    /*enable or disable IEP0 reset on EPWM3_sync event*/
    uint8_t  enableIep0ResetOnEpwm3_Sync;
    /*enable or disable IEP0 reset on compare 0 event*/
    uint8_t  enableIep0ResetOnCompare0;
    /*enable or disable shadow mode of IEP*/
    uint8_t  enableIEP0ShadowMode;
    /*enable or disable slave mode, after enabling IEP1 counter follows IEP0*/
    uint8_t  enableIep1SlaveMode;
    /*enable or disable auto clear compare status of IEP0 and IEP1*/
    uint8_t  enableAutoClearCompareStatus;
    /*iep0 increment value*/
    uint8_t  iep0IncrementValue;

    /*
     * Note : It is recommended to enable shadow mode , reset on compare0, iep1 slave mode, enable auto clear compare status
     */

}AppPruIcssPwmIepCfg_t;

typedef struct
{

    /*
     *
     * Note : This structure is placed as instance in multidimensional array of (4 X 6)
     *
     * Each entry in (PRUICSS_NUM_PWM_SETS X PRUICSS_NUM_OF_PWMINSTANCES_PER_SET) array maps to config parameters of one PWM signal.
     *
     */

    /*duty cycle of current pwm signal*/
    uint32_t dutyCycle;
    /*fall edge of current pwm signal*/
    uint32_t fallEdgeDelay;
    /*rise edge of current pwm signal*/
    uint32_t riseEdgeDelay;
    /*current pwm signal output in intial state*/
    uint8_t  outputCfgInitialState;
    /*current pwm signal output in active state*/
    uint8_t  outputCfgActiveState;
    /*current pwm signal output in trip state*/
    uint8_t  outputCfgTripState;
    /*enable or disable current pwm signal*/
    uint8_t  enable;

    /*
     * Below parameters(iepInstance, compareEvent) are configured on pwm init api call. 
     */

    /*iep instance mapped to current pwm signal*/
    uint8_t  iepInstance;
    /*compare event mapped to current pwm signal*/
    uint8_t  compareEvent;

}AppPruIcssPwmCfg_t;

typedef struct{
    PRUICSS_Handle pruIcssHandle;
    AppPruIcssPwmCfg_t  (*pruIcssPwmCfg)[PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET];
    AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg;
    uint32_t EpwmBaseAddr;
}AppEpwmSync0IrqArgs_t;

/* Configures SOC EPWM */
void App_epwmConfig(uint32_t epwmBaseAddr, uint32_t epwmCh, uint32_t epwmFuncClk);
/* Intializes the Iep parameters with default values*/
void App_pruIcssPwmIepInit(AppPruIcssPwmIepCfg_t *pruIcssPwmIepCfg, uint32_t pruIcssPwmFrequency);
/* Configures the Iep parameters defined in pruIcssPwmIepCfg*/
void App_pruIcssPwmIepConfig(PRUICSS_Handle handle, AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg);
/* Intializes the pwm parameters output in intial, active, trip states with default values & maps compare events and disables all pwm signals*/
void App_pruIcssPwmInit(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET]);
/* Enables & configures pwm signal states & intializes pwm period & duty cycle,PruIcssPwmCfg function call can be used in motor control foc loop to update duty cycle of pwm*/
void App_pruIcssPwmCfg(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], uint8_t pwmSet, uint8_t instance,  uint32_t dutyCycle, uint32_t riseEdgeDelay, uint32_t fallEdgeDelay);
/*If pwm signal is enabled ,Intializes the pwm output configuration of pwm states from the parameters specified */
void App_pruIcssPwmStateInit(AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], uint8_t pwmSet, uint8_t instance, uint8_t outputCfgInitialState, uint8_t outputCfgActiveState, uint8_t outputCfgTripState);
/* If pwm signal is enabled ,Configures output state defined in pruIcssPwmCfg*/
void App_pruIcssPwmStateConfig(PRUICSS_Handle handle, AppPruIcssPwmCfg_t  pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg);
/* This funtions configures App_pruIcssPwmHalfDoneIrq ISR which controls the duty cycle and dead band at fall edge of PWM signals*/
void App_pruicssIep1Compare0IrqSet(PRUICSS_Handle handle);
void App_pruIcssPwmHalfDoneIrq();
/* This funtions configures App_epwmSync0Irq ISR which controls the period of PRUICSS PWM signals, duty cycle and dead band at rise edge of PWM signals */
void App_epwm0Sync0IrqSet(PRUICSS_Handle handle, AppPruIcssPwmCfg_t pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg, uint32_t epwmBaseAddr);
void App_epwmSync0Irq(void *args);
/* This function writes compare values which controls next rising edge of pwm signals from duty cycle & period configured*/
void App_updateNextRisingEdgeCmpValue(PRUICSS_Handle handle, AppPruIcssPwmCfg_t pruIcssPwmCfg[PRUICSS_NUM_PWM_SETS][PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET], AppPruIcssPwmIepCfg_t pruIcssPwmIepCfg);
/* Sciclient API calls are used in mapping interrupt request to destination id & index from source id & index specified
 * Refer : https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#cmp-event-introuter0-interrupt-router-input-sources
 *      : https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#cmp-event-introuter0-interrupt-router-output-destinations
 */
void App_sciclientCmpEventRouterIrqset(PRUICSS_Handle handle);
void App_sciclientCmpEventRouterIrqRelease(PRUICSS_Handle handle);
