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


//! \brief  These functions define the calculate the phase voltage
//!

#include "volt_recons.h"

//! \brief Defines the constant used to set parameters
//!
#define MOTOR_MIN_ELEC_FREQ_Hz          5.0f
#define MOTOR_MAX_ELEC_FREQ_Hz          800.0f
#define MOTOR_THRESHOLD_VOLTAGE_V       0.20f

//*****************************************************************************
//
// VOLREC_init
//
//*****************************************************************************
VOLREC_Handle VOLREC_init(void *pMemory, const size_t numBytes)
{
    VOLREC_Handle handle;

    if(numBytes < sizeof(VOLREC_Obj))
    {
        return((VOLREC_Handle)0x0);
    }

    // Assign the handle
    handle = (VOLREC_Handle)pMemory;

    return(handle);
} // end of VOLREC_init() function


//*****************************************************************************
//
// VOLREC_reset
//
//*****************************************************************************
void VOLREC_reset(VOLREC_Handle handle)
{
    VOLREC_Obj *obj = (VOLREC_Obj *)handle;

    obj->sf = 1.0f;
    obj->sfCalc = 1.0f;

    obj->VaSenSum = 0.0f;
    obj->VaSum = 0.0f;

    obj->VaSenRms = 0.0f;
    obj->VaRms = 0.0f;

    obj->numSamples = 0;

    obj->signPrev = 0;
    obj->signCurr = 0;
}

//*****************************************************************************
//
// VOLREC_setCoeffs
//
//*****************************************************************************
void VOLREC_setParams(VOLREC_Handle handle,
                const float32_t filterPole_rps, const float32_t ctrlFreq_Hz)
{
    VOLREC_Obj *obj = (VOLREC_Obj *)handle;
    float32_t beta_lp_rad;
    uint32_t cn;

    beta_lp_rad = filterPole_rps / ctrlFreq_Hz;

    obj->a1 = (beta_lp_rad - (float32_t)2.0f) / (beta_lp_rad + (float32_t)2.0f);
    obj->b0 = beta_lp_rad / (beta_lp_rad + (float32_t)2.0f);
    obj->b1 = obj->b0;

    for(cn = 0; cn < 3; cn++)
    {
        obj->x1.value[cn] = 0.0f;
        obj->y1.value[cn] = 0.0f;

        obj->Vin_V.value[cn] = 0.0f;
        obj->Vs_V.value[cn] = 0.0f;
    }

    obj->sf = 0.925f;
    obj->sfCalc = 0.925f;
    obj->threshold = MOTOR_THRESHOLD_VOLTAGE_V;      // 1.0V

    obj->VaSenSum = 0.0f;
    obj->VaSum = 0.0f;

    obj->numSamples = 0;
    obj->minSamples = (int32_t)(ctrlFreq_Hz / MOTOR_MAX_ELEC_FREQ_Hz);
    obj->maxSamples = (int32_t)(ctrlFreq_Hz / MOTOR_MIN_ELEC_FREQ_Hz);

    obj->signPrev = 0;
    obj->signCurr = 0;

    return;
}

// end of file
