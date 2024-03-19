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

/*---------------------------------------------------------------------------------------*/
/*TODO:                    sysconfig generation code starts here                         */
/*---------------------------------------------------------------------------------------*/

/* IEP0_CLK_FREQ in hertz*/
#define PRUICSS_IEP0_CLK_FREQ                               (200000000U)
/* IEP CLK period in nano seconds*/
#define PRUICSS_IEP0_CLK_PERIOD_IN_NANOSECONDS              (5U)
/* Modify this to change the IEP counter increment value*/
#define PRUICSS_IEP_COUNT_INCREMENT_VALUE                   (1U)
/* PRUICSS PWM frequency in Hz*/
#define PRUICSS_PWM_FREQUENCY                               (16000U)
/* Macro's to configure duty cycle, rise edge & fall edge deadband of pwm signals*/
#define PRUICSS_PWM_SET0_INSTANCE_A0_DUTY_CYCLE                   (25U)
#define PRUICSS_PWM_SET0_INSTANCE_A0_RISE_EDGE_DELAY_IN_ns        (0U)
#define PRUICSS_PWM_SET0_INSTANCE_A0_FALL_EDGE_DELAY_IN_ns        (0U)

#define PRUICSS_PWM_SET0_INSTANCE_B0_DUTY_CYCLE                   (25U)
#define PRUICSS_PWM_SET0_INSTANCE_B0_RISE_EDGE_DELAY_IN_ns        (200U)
#define PRUICSS_PWM_SET0_INSTANCE_B0_FALL_EDGE_DELAY_IN_ns        (400U)

#define PRUICSS_PWM_SET2_INSTANCE_A0_DUTY_CYCLE                   (75U)
#define PRUICSS_PWM_SET2_INSTANCE_A0_RISE_EDGE_DELAY_IN_ns        (0U)
#define PRUICSS_PWM_SET2_INSTANCE_A0_FALL_EDGE_DELAY_IN_ns        (0U)

#define PRUICSS_PWM_SET2_INSTANCE_B0_DUTY_CYCLE                   (75U)
#define PRUICSS_PWM_SET2_INSTANCE_B0_RISE_EDGE_DELAY_IN_ns        (600U)
#define PRUICSS_PWM_SET2_INSTANCE_B0_FALL_EDGE_DELAY_IN_ns        (800U)

#define CONFIG_PRUICSS_PWM_IEP_INSTANCE     0

/*-----------------------------------------------------------------------------------------*/
/*TODO:                    sysconfig generation code ends here                             */
/*-----------------------------------------------------------------------------------------*/

typedef struct{
    PRUICSS_PWM_Handle handle; 
    uint32_t EpwmBaseAddr;
}AppEpwmSync0IrqArgs_t;

/* Configures SOC EPWM */
void App_epwmConfig(uint32_t epwmBaseAddr, uint32_t epwmCh, uint32_t epwmFuncClk);
/* This funtions configures App_pruIcssPwmHalfDoneIrq ISR which controls the duty cycle and dead band at fall edge of PWM signals*/
int32_t App_pruicssIep1Compare0IrqSet(PRUICSS_PWM_Handle handle);
void App_pruIcssPwmHalfDoneIrq(void *args);
/* This funtions configures App_epwmSync0Irq ISR which controls the period of PRUICSS PWM signals, duty cycle and dead band at rise edge of PWM signals */
int32_t App_epwm0Sync0IrqSet(PRUICSS_PWM_Handle handle, uint32_t epwmBaseAddr);
void App_epwmSync0Irq(void *args);
/* This function writes compare values which controls next rising edge of pwm signals from duty cycle & period configured*/
int32_t App_updateNextRisingEdgeCmpValue(PRUICSS_PWM_Handle handle);
/* Sciclient API calls are used in mapping interrupt request to destination id & index from source id & index specified
 * Refer : https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#cmp-event-introuter0-interrupt-router-input-sources
 *      : https://software-dl.ti.com/tisci/esd/latest/5_soc_doc/am64x/interrupt_cfg.html#cmp-event-introuter0-interrupt-router-output-destinations
 */
int32_t App_sciclientCmpEventRouterIrqset(PRUICSS_PWM_Handle handle);
int32_t App_sciclientCmpEventRouterIrqRelease(PRUICSS_PWM_Handle handle);
