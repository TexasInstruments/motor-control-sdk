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

#ifndef PI_H
#define PI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "dcl.h"
#include <stdlib.h>

/**
 *  \defgroup PI_API_MODULE APIs for PI controller
 *  \ingroup  CONTROLS_MODULE
 *
 *  Here is the list of PI controller APIs
 *  @{
 *
 *  \file           pi.h
 *  \brief          PI (Proportional and Integral) controller library
 *
 *  \note   This library is meant to be API backwards-compatible with existing software.
 *  It uses DCL (digital control library)'s PI controller
 *  located in SDK's source/dcl folder.
 */

#define _PI_Obj_ dcl_pi;
typedef DCL_PI PI_Obj;

//*****************************************************************************
//
//! \brief      Gets the gains in the PI controller
//!
//! \param[in]  handle  The PI controller handle
//!
//! \param[in] pKp      The pointer to store the return proportional gain value
//!
//! \param[in] pKi      The pointer to store the return integrator gain value
//
//*****************************************************************************
static inline void
PI_getGains(PI_Handle handle, float32_t *pKp, float32_t *pKi)
{
    *pKp = handle->Kp;
    *pKi = handle->Ki;
} // end of PI_getGains() function

//*****************************************************************************
//
//! \brief     Gets the integral gain in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \return    The integral gain in the PI controller
//
//*****************************************************************************
static inline float32_t
PI_getKi(PI_Handle handle)
{
    return(handle->Ki);
} // end of PI_getKi() function

//*****************************************************************************
//
//! \brief     Gets the proportional gain in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \return    The proportional gain in the PI controller
//
//*****************************************************************************
static inline float32_t
PI_getKp(PI_Handle handle)
{
    return(handle->Kp);
} // end of PI_getKp() function

//*****************************************************************************
//
//! \brief      Gets the minimum and maximum output value allowed in the PI
//!             controller
//!
//! \param[in]  handle   The PI controller handle
//!
//! \param[in] pOutMin  The pointer to store the return minimum output value allowed
//!
//! \param[in] pOutMax  The pointer to store the return maximum output value allowed
//
//*****************************************************************************
static inline void
PI_getMinMax(PI_Handle handle, float32_t *pOutMin, float32_t *pOutMax)
{
    *pOutMin = handle->Umin;
    *pOutMax = handle->Umax;
} // end of PI_getMinMax() function

//*****************************************************************************
//
//! \brief      Gets the maximum output value allowed in the PI controller
//!
//! \param[in]  handle  The PI controller handle
//!
//! \return     The maximum output value allowed
//
//*****************************************************************************
static inline float32_t
PI_getOutMax(PI_Handle handle)
{
    return(handle->Umax);
} // end of PI_getOutMax() function

//*****************************************************************************
//
//! \brief      Gets the minimum output value allowed in the PI controller
//!
//! \param[in]  handle  The PI controller handle
//!
//! \return     The minimum output value allowed
//
//*****************************************************************************
static inline float32_t
PI_getOutMin(PI_Handle handle)
{
    return(handle->Umin);
} // end of PI_getOutMin() function

//*****************************************************************************
//
//! \brief     Gets the integrator start value in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \return    The integrator start value for the PI controller
//
//*****************************************************************************
static inline float32_t
PI_getUi(PI_Handle handle)
{
    return(handle->i10);
} // end of PI_getUi() function

//*****************************************************************************
//
//! \brief     Initializes the PI controller
//!
//! \param[in] pMemory   A pointer to the memory for the PI controller object
//!
//! \param[in] numBytes  The number of bytes allocated for the PI controller
//!                      object, bytes
//!
//! \return    The PI controller (PI) object handle
//
//*****************************************************************************
extern PI_Handle
PI_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Sets the gains in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] Kp      The proportional gain for the PI controller
//!
//! \param[in] Ki      The integrator gain for the PI controller
//
//*****************************************************************************
static inline void
PI_setGains(PI_Handle handle, const float32_t Kp, const float32_t Ki)
{
    handle->Kp = Kp;
    handle->Ki = Ki;
} // end of PI_setGains() function

//*****************************************************************************
//
//! \brief     Sets the integral gain in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] Ki         The integral gain for the PI controller
//
//*****************************************************************************
static inline void
PI_setKi(PI_Handle handle, const float32_t Ki)
{
    handle->Ki = Ki;
} // end of PI_setKi() function

//*****************************************************************************
//
//! \brief     Sets the proportional gain in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] Kp         The proportional gain for the PI controller
//
//*****************************************************************************
static inline void
PI_setKp(PI_Handle handle, const float32_t Kp)
{
    handle->Kp = Kp;
} // end of PI_setKp() function

//*****************************************************************************
//
//! \brief     Sets the minimum and maximum output value allowed in the PI
//!            controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] outMin  The minimum output value allowed
//!
//! \param[in] outMax  The maximum output value allowed
//
//*****************************************************************************
static inline void
PI_setMinMax(PI_Handle handle, const float32_t outMin, const float32_t outMax)
{
    handle->Umin = outMin;
    handle->Umax = outMax;
} // end of PI_setMinMax() function

//*****************************************************************************
//
//! \brief     Sets the maximum output value allowed in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] outMax  The maximum output value allowed
//
//*****************************************************************************
static inline void
PI_setOutMax(PI_Handle handle, const float32_t outMax)
{
    handle->Umax = outMax;
} // end of PI_setOutMax() function

//*****************************************************************************
//
//! \brief     Sets the minimum output value allowed in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] outMin  The minimum output value allowed
//
//*****************************************************************************
static inline void
PI_setOutMin(PI_Handle handle, const float32_t outMin)
{
    handle->Umin = outMin;
} // end of PI_setOutMin() function

//*****************************************************************************
//
//! \brief     Sets the integrator start value in the PI controller
//!
//! \param[in] handle  The PI controller handle
//!
//! \param[in] Ui      The integral start value for the PI controller
//
//*****************************************************************************
static inline void
PI_setUi(PI_Handle handle, const float32_t Ui)
{
    handle->i10 = Ui;
} // end of PI_setUi() function

//*****************************************************************************
//
//! \brief     Runs the parallel form of the PI controller
//!
//! \param[in] handle      The PI controller handle
//!
//! \param[in] refValue    The reference value to the controller
//!
//! \param[in] fbackValue  The feedback value to the controller
//!
//! \param[in] ffwdValue   The feedforward value to the controller
//!
//! \param[in] pOutValue   The pointer to the controller output value
//
//*****************************************************************************
static inline void
PI_run_parallel(PI_Handle handle, const float32_t refValue,
                const float32_t fbackValue, const float32_t ffwdValue,
                float32_t *pOutValue)
{
    handle->i10 += ffwdValue;
    *pOutValue = DCL_runPIParallel(handle, refValue, fbackValue);
    
} // end of PI_run_parallel() function

//*****************************************************************************
//
//! \brief     Runs the series form of the PI controller
//!
//! \param[in] handle      The PI controller handle
//!
//! \param[in] refValue    The reference value to the controller
//!
//! \param[in] fbackValue  The feedback value to the controller
//!
//! \param[in] ffwdValue   The feedforward value to the controller
//!
//! \param[in] pOutValue   The pointer to the controller output value
//
//*****************************************************************************
static inline void
PI_run_series(PI_Handle handle, const float32_t refValue,
              const float32_t fbackValue, const float32_t ffwdValue,
              float32_t *pOutValue)
{
    handle->i10 += ffwdValue;
    *pOutValue = DCL_runPISeries(handle, refValue, fbackValue);

} // end of PI_run_series() function


//*****************************************************************************
//
//! \brief     Runs the series form of the PI controller
//!
//! \param[in] handle      The PI controller handle
//!
//! \param[in] refValue    The reference value to the controller
//!
//! \param[in] fbackValue  The feedback value to the controller
//!
//! \param[in] pOutValue   The pointer to the controller output value
//
//*****************************************************************************
static inline void
PI_run(PI_Handle handle, const float32_t refValue,
       const float32_t fbackValue, float32_t *pOutValue)
{
    *pOutValue = DCL_runPISeries(handle, refValue, fbackValue);
} // end of PI_run_series() function

/** @} */

#ifdef __cplusplus
}
#endif

#endif // end of PI_H defines
