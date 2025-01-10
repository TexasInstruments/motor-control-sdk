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

#ifndef SPEED_FR_H
#define SPEED_FR_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup SPEEDFR_API_MODULE APIs for speed frequency function
 *  \ingroup  OBSERVERS_MODULE
 *
 *  Here is the list of speed frequency function APIs sensored speed calculation
 *  @{
 *
 *  \file           speedfr.h
 *  \brief          Contains speed frequency functions implementation
 */

#include <stdlib.h>

#include "math_types.h"
#include "userParams.h"

typedef struct _SPDFR_obj_
{
    float32_t  theta;           // Input: reference set-point
    float32_t  thetaPrev;       // Input: feedback
    float32_t  K2;              // Parameter: proportional loop gain
    float32_t  K3;              // Parameter: integral gain
    float32_t  scaleFreq;       // Parameter:
    float32_t  speed_pu;        // Output: frequency
    float32_t  speed_Hz;        // Output: frequency
} SPDFR_Obj;

//! \brief Defines the SPDFR handle
//!
typedef struct _SPDFR_obj_ *SPDFR_Handle;

// ***************************************
// extern functions
// ***************************************

//*****************************************************************************
//
//! \brief     Set the SPDFR controller
//! \param[in] pMemory   A pointer to the memory for the SPDFR object
//! \param[in] numBytes  The number of bytes allocated for the SPDFR object, bytes
//! \return    The handle to SPDFR object
//
//*****************************************************************************
SPDFR_Handle SPDFR_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Set the SPDFR controller
//! \param[in] handle   The ESMO controller handle
//
//*****************************************************************************
extern void SPDFR_reset(SPDFR_Handle handle);

//*****************************************************************************
//
//! \brief     Set the SPDFR controller
//! \param[in] handle      The SPDFR controller handle
//! \param[in] pUserParams The pointer to User_Params object
//
//*****************************************************************************
extern void SPDFR_setParams(SPDFR_Handle handle, const USER_Params *pUserParams);

//*****************************************************************************
//
//! \brief     Set the SPDFR controller
//! \param[in] handle      The SPDFR controller handle
//
//*****************************************************************************
static inline float32_t SPDFR_getSpeedHz(SPDFR_Handle handle)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;

    return(obj->speed_Hz);
}

//*****************************************************************************
//
//! \brief     Set the SPDFR controller
//! \param[in] handle   The SPDFR controller handle
//! \param[in] theta    The rotor angle in radians
//
//*****************************************************************************
static __attribute__((always_inline))
void SPDFR_run(SPDFR_Handle handle, float32_t theta)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;
    float32_t error;

    obj->theta  = theta * MATH_ONE_OVER_TWO_PI;

    // error cal
    error = obj->theta - obj->thetaPrev;

    if(error < -0.5f)
    {
        error += 1.0f;
    }
    else if(error > 0.5f)
    {
        error -= 1.0f;
    }

    // Update the electrical angle
    obj->thetaPrev = obj->theta;

    // Low-pass filter
    obj->speed_pu = (obj->K2 * obj->speed_pu) + (obj->K3 * error);

    // Saturate the output
    obj->speed_pu = MATH_sat(obj->speed_pu, 1.0f, -1.0f);

    // Change motor speed for pu to rpm value
    obj->speed_Hz = obj->scaleFreq * obj->speed_pu;

    return;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //end of SPEED_FR_H definition
