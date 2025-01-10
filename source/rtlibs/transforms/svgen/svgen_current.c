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

//! \brief  Contains the various functions related to the 
//!         svgen current object
//!

// **************************************************************************
// the includes

#include "svgen_current.h"

// **************************************************************************
// the functions

SVGENCURRENT_Handle SVGENCURRENT_init(void *pMemory,const size_t numBytes)
{
  SVGENCURRENT_Handle svgencurrentHandle;

  if(numBytes < sizeof(SVGENCURRENT_Obj))
  {
    return((SVGENCURRENT_Handle)0x0);
  }

  //
  // Assign the handle
  //
  svgencurrentHandle = (SVGENCURRENT_Handle)pMemory;

  return(svgencurrentHandle);
} // end of SVGENCURRENT_init() function


void SVGENCURRENT_setup(SVGENCURRENT_Handle handle, const float32_t minWidth_usec,
                        const float32_t pwmFreq_kHz, const float32_t systemFreq_MHz)
{
    uint16_t minWidth_counts = (uint16_t)(minWidth_usec * systemFreq_MHz);

    //
    // maximum output voltage duty: 0.5
    // convert the minimum pwm width to duty by divide Tpwm
    // Tpwm = 1/(USER_PWM_FREQ_kHz * 1000) = * (USER_PWM_FREQ_kHz * 0.001)
    // The minimum pwm width is the same in two zero vectors for svpwm

    float32_t dutyLimit = 0.5f - (minWidth_usec * pwmFreq_kHz * 0.001f);

    SVGENCURRENT_setMinWidth(handle, minWidth_counts);
    SVGENCURRENT_setIgnoreShunt(handle, SVGENCURRENT_USE_ALL);
    SVGENCURRENT_setMode(handle, SVGENCURRENT_ALL_PHASE_MEASURABLE);
    SVGENCURRENT_setVlimit(handle, dutyLimit);

    return;
} // end of SVGENCURRENT_setup() function

// end of file
