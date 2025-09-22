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

#ifndef SPEED_CALC_H
#define SPEED_CALC_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup SPEEDCALC_API_MODULE APIs for sensorless speed calculation
 *  \ingroup  OBSERVERS_MODULE
 *
 *  Here is the list of function APIs
 *  @{
 *
 *  \file           speedcalc.h
 *  \brief          Contains speed calculation functions
 */

#include <stdlib.h>

#include "math_types.h"
#include "userParams.h"

typedef struct _SPDCALC_obj_
{
    float32_t  ref;             // Input: reference set-point
    float32_t  fbk;             // Input: feedback
    float32_t  err;             // Error
    float32_t  out;             // Output: controller output
    float32_t  Kp;              // Parameter: proportional loop gain
    float32_t  Ki;              // Parameter: integral gain
    float32_t  Umax;            // Parameter: upper saturation limit
    float32_t  Umin;            // Parameter: lower saturation limit
    float32_t  Up;              // Data: proportional term
    float32_t  Ui;              // Data: integral term
    float32_t  speed_Hz;        // Output freq
    float32_t  thetaDelta;      // Parameter: theta maximum
} SPDCALC_Obj;

//! \brief Defines the ESMO handle
//!
typedef struct _SPDCALC_obj_ *SPDCALC_Handle;

// ***************************************
// extern functions
// ***************************************

//*****************************************************************************
//
//! \brief     Set the SPDCALC controller
//! \param[in] pMemory   A pointer to the memory for the SPDCALC object
//! \param[in] numBytes  The number of bytes allocated for the SPDCALC object, bytes
//! \return    The handle to SPDCALC object

//*****************************************************************************
SPDCALC_Handle SPDCALC_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Set the SPDCALC controller
//! \param[in] handle   The ESMO controller handle
//!
//*****************************************************************************
extern void SPDCALC_reset(SPDCALC_Handle handle);

//*****************************************************************************
//
//! \brief     Set the SPDCALC controller
//! \param[in] handle      The SPDCALC controller handle
//! \param[in] pUserParams The pointer to User_Params object
//!
//*****************************************************************************
extern void SPDCALC_setParams(SPDCALC_Handle handle, const USER_Params *pUserParams);

//*****************************************************************************
//
//! \brief     Set the SPDCALC controller
//! \param[in] handle      The SPDCALC controller handle
//!
//*****************************************************************************
static inline float32_t SPDCALC_getSpeedHz(SPDCALC_Handle handle)
{
    SPDCALC_Obj *obj = (SPDCALC_Obj *)handle;

    return(obj->speed_Hz);
}

//*****************************************************************************
//
//! \brief     Set the SPDCALC controller
//! \param[in] handle   The SPDCALC controller handle
//! \param[in] theta    The rotor angle in radians
//!
//*****************************************************************************
static __attribute__((always_inline))
void SPDCALC_run(SPDCALC_Handle handle, float32_t theta)
{
    SPDCALC_Obj *obj = (SPDCALC_Obj *)handle;

    obj->ref  = theta;

    // error cal
    obj->err = obj->ref - obj->fbk;

    // roll in the error
    if(obj->err >= MATH_PI)
    {
        obj->err = obj->err - MATH_TWO_PI;
    }
    else if(obj->err <= -MATH_PI)
    {
        obj->err = obj->err + MATH_TWO_PI;
    }

    // P and I control
    obj->Up  = obj->Kp * obj->err;              // P control
    obj->Ui += obj->Ki * obj->err;              // I control
    obj->Ui  = MATH_sat(obj->Ui, obj->Umax, obj->Umin);

    // control output
    obj->out = obj->Up + obj->Ui;
    obj->out = MATH_sat(obj->out, obj->Umax, obj->Umin);      // rad/s

    obj->speed_Hz = obj->out * MATH_ONE_OVER_TWO_PI;

    // Latest angle feedback estimation --> ( Fbk = integral of speed )
    obj->fbk = obj->fbk + obj->out * obj->thetaDelta;

    // roll "Fbk" within -pi to pi
    if(obj->fbk >= MATH_PI)
    {
        obj->fbk = obj->fbk - MATH_TWO_PI;
    }
    else if(obj->fbk <= -MATH_PI)
    {
        obj->fbk = obj->fbk + MATH_TWO_PI;
    }

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

#endif //end of SPEED_CALC_H definition
