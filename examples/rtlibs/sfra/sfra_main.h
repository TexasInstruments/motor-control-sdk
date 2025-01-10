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

#ifndef EXAMPLE1_STB_H
#define EXAMPLE1_STB_H

#ifdef __cplusplus

extern "C" {
#endif

//
// the includes
//
#include <stdint.h>
#include <math.h>


#include "sfra_examples_settings.h"
#include "sfra_examples_hal.h"

#include "dcl.h"

#include "sfra_f32.h"


//
// defines
//


//
// System Settings
//

//
// typedefs
//


//
// globals
//

extern DCL_PI gi;
extern volatile float32_t gi_out;
extern volatile float32_t gi_out_prev;
extern int32_t injcnt;
extern volatile int32_t var1;
extern float32_t injectarray[SFRA_FREQ_LENGTH];
//
// Reference variables
// current set point
//
extern volatile float32_t ac_cur_ref;
//
//flag to close the loop
//
extern volatile int32_t closeGiLoop;
int32_t temp=0;

//
// the function prototypes
//


void setupSFRA();
void globalVariablesInit();


//#pragma FUNC_ALWAYS_INLINE(controlCode)
//
//control Code
//
static inline void controlCode(void)
{
    setProfilingGPIO();

    if(closeGiLoop == 1)
    {
        injcnt++;
        gi_out = DCL_runPISeries(&gi, SFRA_F32_inject(ac_cur_ref),
                            gi_out_prev );

    }

    SFRA_F32_collect((float *)&gi_out, (float *)&gi_out_prev);
    gi_out_prev = gi_out;

    clearPWMInterruptFlag(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);
    resetProfilingGPIO();

}


#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
