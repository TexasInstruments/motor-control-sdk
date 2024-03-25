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
