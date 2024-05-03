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

#ifndef SFRA_EXAMPLES_HAL_H
#define SFRA_EXAMPLES_HAL_H

#ifdef __cplusplus

extern "C" {
#endif


//
// the includes
//

#include <stdint.h>
#include "ti_drivers_config.h"
#include "drivers/hw_include/hw_types.h"
#include "sfra_examples_settings.h"

//
//defines
//




#define PWM_TRIP_STATUS EPWM_getTripZoneFlagStatus

#define ADC_PU_SCALE_FACTOR  (float)(0.000244140625)
#define ADC_PU_PPB_SCALE_FACTOR 0.000488281250 //1/2^11

#define GET_TASK_A_TIMER_OVERFLOW_STATUS TimerP_isOverflowed(CPUTIMER0_BASE_ADDR)
#define CLEAR_TASK_A_TIMER_OVERFLOW_FLAG TimerP_clearOverflowInt(CPUTIMER0_BASE_ADDR)

#define GET_TASK_B_TIMER_OVERFLOW_STATUS TimerP_isOverflowed(CPUTIMER1_BASE_ADDR)
#define CLEAR_TASK_B_TIMER_OVERFLOW_FLAG TimerP_clearOverflowInt(CPUTIMER1_BASE_ADDR)

//
// globals
//

//
// the function prototypes
//


void setupADC(void);

void setupUpDwnCountPWM(uint32_t base1, uint16_t pwm_period_ticks);

void disablePWMCLKCounting(void);
void enablePWMCLKCounting(void);
void setPinsAsPWM();

void setupProfilingGPIO();

//
// ISR related
//

 void controlISR(void *args);



//
// Inline functions
//

//
// setProfilingGPIO
//
static inline void setProfilingGPIO(void)
{

    HW_WR_REG32((CSL_GPIO0_U_BASE + CSL_GPIO_SET_DATA(0)), 0x2000); // GPIO used by default is 13

}

//
// resetProfilingGPIO
//
static inline void resetProfilingGPIO(void)
{

    HW_WR_REG32((CSL_GPIO0_U_BASE + CSL_GPIO_CLR_DATA(0)), 0x2000); // GPIO used by default is 13

}

//
// setProfilingGPIO
//
static inline void setProfilingGPIO2(void)
{

    HW_WR_REG32((CSL_GPIO0_U_BASE + CSL_GPIO_SET_DATA(0)), 0x4000); // GPIO used by default is 14

}

//
// resetProfilingGPIO
//
static inline void resetProfilingGPIO2(void)
{

    HW_WR_REG32((CSL_GPIO0_U_BASE + CSL_GPIO_CLR_DATA(0)), 0x4000); // GPIO used by default is 14

}

//
// clearPWM Interrupt Flag
//
static inline void clearPWMInterruptFlag(uint32_t base)
{
    EPWM_clearEventTriggerInterruptFlag(base);
}

//
//enable PWM Interrupt generation
//
static inline void enablePWMInterruptGeneration(uint32_t base)
{
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_ZERO, 0);
    EPWM_setInterruptEventCount(base, CNTRL_ISR_FREQ_RATIO);
    EPWM_enableInterrupt(base);
    EPWM_clearEventTriggerInterruptFlag(base);
}



//
// clearInterrupt
//
int32_t  status;
HwiP_Params  hwiPrms;
HwiP_Object hwiObj;

static inline void setupInterrupt(void)
{
    enablePWMInterruptGeneration(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);

    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, ( C28x_CONTROLISR_INTERRUPT ), 0, 0, 0, 0, 0, 0);

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback    = &controlISR;
    hwiPrms.priority    = 3;
    status              = HwiP_construct(&hwiObj, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);
    EPWM_clearEventTriggerInterruptFlag(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);

}



#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
