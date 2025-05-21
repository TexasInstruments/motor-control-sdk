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

//! \file   libraries/control/vs_freq/source/vs_freq.c
//! \brief  Contains the public interface to the stator voltage frequency
//! \brief  generator (VS_FREQ)
//!         module routines
//!

// **************************************************************************
// the includes
#include "vs_freq.h"

//*****************************************************************************
//
// VS_FREQ_init
//
//*****************************************************************************
VS_FREQ_Handle VS_FREQ_init(void *pMemory,const size_t numBytes)
{
  VS_FREQ_Handle handle;

  if(numBytes < sizeof(VS_FREQ_Obj))
    return((VS_FREQ_Handle)0x0);

  // assign the handle
  handle = (VS_FREQ_Handle)pMemory;

  return(handle);
} // end of VS_FREQ_init() function


//! \brief     Sets the parameters
//! \param[in] handle               The volts/hertz profile (VS_FREQ) handle
//! \param[in] LowFreq              The low frequency for volts/hertz profile, Hz
//! \param[in] HighFreq             The high frequency for volts/hertz profile, Hz
//! \param[in] VoltMin              The minimum frequency for volts/hertz profile, Volts
//! \param[in] VoltMax              The maximum frequency for volts/hertz profile, Volts
void VS_FREQ_setProfile(VS_FREQ_Handle handle,
                        float32_t LowFreq, float32_t HighFreq,
                        float32_t VoltMin, float32_t VoltMax)

{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    obj->LowFreq = LowFreq;
    obj->HighFreq = HighFreq;

    obj->VoltMin = VoltMin;
    obj->VoltMax = VoltMax;

    obj->VfSlope = (obj->VoltMax - obj->VoltMin)/(obj->HighFreq - obj->LowFreq);

    obj->Vdq_gain.value[0] = 0.3f;

    obj->Vdq_gain.value[1] = sqrtf(obj->maxVsMag_pu * obj->maxVsMag_pu -
                               obj->Vdq_gain.value[0] * obj->Vdq_gain.value[0]);


  return;
} // end of VS_FREQ_setProfile() function

// end of the file
