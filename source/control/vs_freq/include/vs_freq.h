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

#ifndef VS_FREQ_H
#define VS_FREQ_H

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the includes

#include <stdlib.h>
#include <math.h>

#include "math_types.h"

/**
 *  \defgroup VSFREQ_API_MODULE APIs for Voltage Frequency Generator (VS_FREQ)
 *  \ingroup  CONTROLS_MODULE
 *
 *  Here is the list of VS_FREQ function APIs
 *  @{
 *
 *  \file           vs_freq.h
 *  \brief          Contains the public interface to the stator voltage frequency generator (VS_FREQ)
 */

//! \brief Defines the angle generator (ANGLE_COMP) object
//!
typedef struct _VS_FREQ_Obj_
{
    float32_t   maxVsMag_pu;
    float32_t   Freq;          //!< Input: Input Frequency (pu)
    float32_t   LowFreq;       //!< Parameter: Low Frequency (pu)
    float32_t   HighFreq;       //!< Parameter: High Frequency at rated voltage (pu)
    float32_t   MaxFreq;        //!< Parameter: Maximum Frequency (pu)
    float32_t   VoltMin;        //!< Parameter: Voltage at low Frequency range (pu)
    float32_t   VoltMax;        //!< Parameter: Rated voltage (pu)
    float32_t   VfSlope;        //!< Variable
    float32_t   Vs_out;         //!< Output: Output voltage (pu)
    MATH_Vec2   Vdq_gain;       //!< Variable
    MATH_Vec2   Vdq_out;        //!< Output: Output voltage (pu)
} VS_FREQ_Obj, *VS_FREQ_Handle;

// **************************************************************************
// the function prototypes

//*****************************************************************************
//!
//! \brief     Gets the Vd output value in VS_FREQ
//! \param[in] handle  The volts/hertz (VS_FREQ) handle
//! \return    The Vd output value in VS_FREQ, pu
//!
//*****************************************************************************
static inline float32_t VS_FREQ_getVd_out(VS_FREQ_Handle handle)
{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    return(obj->Vdq_out.value[0]);
} // end of VS_FREQ_getVd_pu() function

//*****************************************************************************
//!
//! \param[in] handle  The volts/hertz profile (VS_FREQ) handle
//! \return    The Vq output value in VS_FREQ, pu
//!
//*****************************************************************************
static inline float32_t VS_FREQ_getVq_out(VS_FREQ_Handle handle)
{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    return(obj->Vdq_out.value[1]);
} // end of VS_FREQ_getVq_pu() function

//*****************************************************************************
//!
//! \brief     Sets the parameters maximum frequency
//! \param[in] handle               The volts/hertz profile (VS_FREQ) handle
//! \param[in] maxFreq              The maixmum frequency for volts/hertz profile, Hz
//!
//*****************************************************************************
static inline void VS_FREQ_setMaxFreq(VS_FREQ_Handle handle,
                            float32_t maxFreq)
{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    obj->MaxFreq = maxFreq;

    return;
} // end of ANGLE_COMP_setParams() function

//*****************************************************************************
//!
//! \brief     Sets the parameters VsMag_pu
//! \param[in] handle               The volts/hertz profile (VS_FREQ) handle
//! \param[in] maxVsMag_pu          The maixmum magintude for volts/hertz profile, pu
//!
//*****************************************************************************
static inline void VS_FREQ_setVsMagPu(VS_FREQ_Handle handle,
                            float32_t maxVsMag_pu)
{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    obj->maxVsMag_pu = maxVsMag_pu;

    return;
} // end of ANGLE_COMP_setParams() function

//*****************************************************************************
//!
//! \brief     Sets the parameters
//! \param[in] handle               The volts/hertz profile (VS_FREQ) handle
//! \param[in] LowFreq              The low frequency for volts/hertz profile, Hz
//! \param[in] HighFreq             The high frequency for volts/hertz profile, Hz
//! \param[in] VoltMin              The minimum frequency for volts/hertz profile, Volts
//! \param[in] VoltMax              The maximum frequency for volts/hertz profile, Volts
//!
//*****************************************************************************
extern void VS_FREQ_setProfile(VS_FREQ_Handle handle,
                               float32_t LowFreq, float32_t HighFreq,
                               float32_t VoltMin, float32_t VoltMax);

//*****************************************************************************
//!
//! \brief     Initializes the angle generator (VS_FREQ) module
//! \param[in] pMemory   A pointer to the memory for the object
//!
//! \param[in] numBytes  The number of bytes allocated for the object, bytes\
//!
//! \return    The volts/hertz profile (VS_FREQ) object handle
//!
//*****************************************************************************
extern VS_FREQ_Handle VS_FREQ_init(void *pMemory,const size_t numBytes);

//*****************************************************************************
//!
//! \brief     Generates an output command voltage for a specific
//!            input command frequency according to the specified
//!            volts/hertz profile.
//! \param[in] handle     The volts/hertz profile (VS_FREQ) handle
//!
//! \param[in] Freq_pu    The input frequency in pu
//!
//*****************************************************************************
static __attribute__((always_inline))
void VS_FREQ_run(VS_FREQ_Handle handle,const float32_t Freq_pu)
{
    VS_FREQ_Obj *obj = (VS_FREQ_Obj *)handle;

    obj->Freq = fabsf(Freq_pu);

    if(obj->Freq <= obj->LowFreq)
    {
        obj->Vs_out = obj->VoltMin;
    }
    else if(obj->Freq >= obj->HighFreq)
    {
        obj->Vs_out = obj->VoltMax;
    }
    else
    {
        obj->Vs_out = obj->VoltMin + obj->VfSlope * (obj->Freq - obj->LowFreq);
    }

    obj->Vdq_out.value[0] = obj->Vs_out * obj->Vdq_gain.value[0];

    if(obj->Freq > 0.0f)
    {
        obj->Vdq_out.value[1] = obj->Vs_out * obj->Vdq_gain.value[1];
    }
    else
    {
        obj->Vdq_out.value[1] = -obj->Vs_out * obj->Vdq_gain.value[1];
    }

    return;
} // end of VS_FREQ_run()

#ifdef __cplusplus
}
#endif // extern "C"

//! @}  // ingroup

#endif // end of VS_FREQ_H definition

