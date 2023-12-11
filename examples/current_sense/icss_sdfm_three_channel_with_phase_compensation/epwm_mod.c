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

#include "epwm_mod.h"

#include <stdint.h>
#include <math.h>


/* Min / max output amplitude.
   Waveform amplitude values beyond these thresholds are saturated. */
#define VREF_MAX                        (  1.0f )
#define VREF_MIN                        ( -1.0f )

/* Compute Duty Cycle & CMPx given Vref & EPWM period */
void computeCmpx(
    float       Vref,
    uint32_t    epwmPrdVal,
    float       *pEpwmDutyCycle,
    uint16_t    *pEpwmCmpVal
)
{
    float dc_f;
    float cmp_f;
    uint16_t cmp;

    if (Vref >= VREF_MAX) {
        /* 100% duty cycle */
        dc_f = 1.0;
    }
    else if (Vref <= VREF_MIN) {
        /* 0% duty cycle */
        dc_f = 0.0;
    }
    else {
        /* compute Duty Cycle */
        dc_f = 0.5*(Vref + 1.0);
    }

    /* compute CMPx */
    cmp_f = (1.0 - dc_f)*epwmPrdVal; /* up-down count */
    cmp = (uint16_t)roundf(cmp_f);

    *pEpwmDutyCycle = dc_f;
    *pEpwmCmpVal = cmp;
}
