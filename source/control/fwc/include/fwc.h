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

#ifndef FWC_H
#define FWC_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup CONTROLS_MODULE Modules for control algorithms
 *  \defgroup FWC_API_MODULE APIs for Field Weakening Control (FWC)
 *  \ingroup  CONTROLS_MODULE
 *
 *  Here is the list of FWC function APIs
 *  @{
 *
 *  \file           fwc.h
 *  \brief          Contains Field Weakening Control (FWC) functions implementation
 */

#include <stdlib.h>
#include <stdbool.h>

#include "dcl.h"
#include "math_types.h"

//*****************************************************************************
//
//! \brief Defines the Field Weakening Control (FWC) object
//
//*****************************************************************************
typedef struct _FWC_Obj_
{
    DCL_PI    pi;                //!< the fwc angle PI controller object
    float32_t angleCurrent_rad;  //!< the stator current phase angle
    bool      flagEnable;        //!< a flag to enable the controller
} FWC_Obj, *FWC_Handle;

//*****************************************************************************
// the function prototypes

//*****************************************************************************
//!
//! \brief     Initializes the Field Weakening Control (FWC) module
//! \note      
//!
//! \param[in] pMemory   A pointer to the memory for the FWC object
//!
//! \param[in] numBytes  The number of bytes allocated for the FWC object
//!
//! \return The Field Weakening Control (FWC) object handle
//!
//*****************************************************************************
extern FWC_Handle FWC_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Sets the Field Weakening Control (FWC) module parmaeters
//!
//! \param[in] handle        The Field Weakening Control (FWC) handle
//!
//! \param[in] Kp            The Kp gain value
//!
//! \param[in] Ki            The Ki gain value
//!
//! \param[in] angleMin_rad  The angle out minimum value
//!
//! \param[in] angleMax_rad  The angle out maximum value
//!
//*****************************************************************************
extern void FWC_setParams(FWC_Handle handle,
                          const float32_t Kp, const float32_t Ki,
                          const float32_t angleMin_rad,
                          const float32_t angleMax_rad);

//*****************************************************************************
//!
//! \brief     Disables the FWC function
//!
//! \param[in] handle  The Field Weakening Control (FWC) handle
//!
//*****************************************************************************
static inline void FWC_disable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = false;
}

//*****************************************************************************
//!
//! \brief     Enables the FWC function
//!
//! \param[in] handle  The Field Weakening Control (FWC) handle
//!
//*****************************************************************************
static inline void FWC_enable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = true;
}

//*****************************************************************************
//!
//! \brief     Gets the stator current phase angle memory address
//!
//! \param[in] handle  The Field Weakening Control (FWC) handle
//!
//! \return    The stator current phase angle memory address
//!
//*****************************************************************************
static inline float32_t* FWC_getCurrentAngle_rad_addr(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(&(obj->angleCurrent_rad));
}

//*****************************************************************************
//!
//! \brief     Gets the stator current phase angle value (angleCurrent_rad)
//!
//! \param[in] handle  The Field Weakening Control (FWC) handle
//!
//! \return    The stator current phase angle value, rad
//!
//*****************************************************************************
static inline float32_t FWC_getCurrentAngle_rad(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->angleCurrent_rad);
}

//*****************************************************************************
//!
//! \brief     Gets the enable controller flag value from the FWC
//!
//! \param[in] handle  The Field Weakening Control (FWC) handle
//!
//! \return    The enable FWC flag value
//!
//*****************************************************************************
static inline bool FWC_getFlagEnable(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    return(obj->flagEnable);
}

//*****************************************************************************
//!
//! \brief     Sets the stator current phase angle value
//!
//! \param[in] handle      The Field Weakening Control (FWC) handle
//!
//! \param[in] angleCurrent_rad  The stator current phase angle value, rad
//!
//*****************************************************************************
static inline void FWC_setCurrentAngle_rad(FWC_Handle handle,
                                         const float32_t angleCurrent_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->angleCurrent_rad = angleCurrent_rad;
}

//*****************************************************************************
//!
//! \brief     Sets the enable flag
//!
//! \param[in] handle      The Field Weakening Control (FWC) handle
//!
//! \param[in] flagEnable  The enable flag
//!
//*****************************************************************************
static inline void FWC_setFlagEnable(FWC_Handle handle, const bool flagEnable)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->flagEnable = flagEnable;
}

//*****************************************************************************
//!
//! \brief     Sets the integral gain (Ki) value
//!
//! \param[in] handle    The Field Weakening Control (FWC) handle
//!
//! \param[in] Ki        The Ki value
//*****************************************************************************
static inline void FWC_setKi(FWC_Handle handle, const float32_t Ki)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->pi.Ki = Ki;
}

//*****************************************************************************
//!
//!
//! \brief     Sets the proportional gain (Kp) value
//!
//! \param[in] handle    The Field Weakening Control (FWC) handle
//!
//! \param[in] Kp        The Kp value
//!
//*****************************************************************************
static inline void FWC_setKp(FWC_Handle handle, const float32_t Kp)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->pi.Kp = Kp;
}

//*****************************************************************************
//! \brief     Sets the gain values
//!
//! \param[in] handle    The Field Weakening Control (FWC) handle
//!
//! \param[in] Kp        The Kp gain value
//!
//! \param[in] Ki        The Ki gain value
//!
//*****************************************************************************
static inline void FWC_setGains(FWC_Handle handle,
                                const float32_t Kp, const float32_t Ki)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    obj->pi.Kp = Kp;
    obj->pi.Ki = Ki;
}

//*****************************************************************************
//!
//! \brief     Sets the minimum and maximum output values of FWC
//!
//! \param[in] handle        The Field Weakening Control (FWC) handle
//!
//! \param[in] angleMin_rad  The angle out minimum value
//!
//! \param[in] angleMax_rad  The angle out maximum value
//!
//*****************************************************************************
static inline void FWC_setAngleMinMax(FWC_Handle handle,
                                      const float32_t angleMin_rad,
                                      const float32_t angleMax_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the minimum and maximum values of PI controller
    obj->pi.Umax = angleMax_rad;
    obj->pi.Umin = angleMin_rad;
}

//*****************************************************************************
//!
//! \brief     Sets the maximum output values of FWC
//!
//! \param[in] handle        The Field Weakening Control (FWC) handle
//!
//! \param[in] angleMax_rad  The angle out maximum value
//!
//*****************************************************************************
static inline void FWC_setAngleMax(FWC_Handle handle,
                                   const float32_t angleMax_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the minimum values of PI controller
    // convert the maximum angle to negative value in PI controller
    // the maximum angle = MATH_PI_OVER_TWO - minimum value of PI output
    obj->pi.Umax = angleMax_rad;
}

//*****************************************************************************
//!
//! \brief     Sets the minimum output values of FWC
//!
//! \param[in] handle        The Field Weakening Control (FWC) handle
//!
//! \param[in] angleMin_rad  The angle out minimum value
//!
//*****************************************************************************
static inline void FWC_setAngleMin(FWC_Handle handle,
                                   const float32_t angleMin_rad)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the minimum values of PI controller
    // convert the maximum angle to negative value in PI controller
    // the maximum angle = MATH_PI_OVER_TWO - minimum value of PI output
    obj->pi.Umin = angleMin_rad;
}

//*****************************************************************************
//!
//! \brief     Resets the integrator start value (Ui)
//!
//! \param[in] handle    The Field Weakening Control (FWC) handle
//!
//*****************************************************************************
#define FWC_resetUi FWC_resetController //Backwards compatible
static inline void FWC_resetController(FWC_Handle handle)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // Sets the integrator start value in the PI controller
    // reset the new integrator value to Zero
    obj->pi.i10 = 0.0f;
    // reset saturation to unclamped
    obj->pi.i6 = 1.0f;

}

//*****************************************************************************
//!
//! \brief     Sets the integrator start value (Ui)
//!
//! \param[in] handle    The Field Weakening Control (FWC) handle
//!
//! \param[in] Ui        The integrator start value
//!
//*****************************************************************************
static inline void FWC_setUi(FWC_Handle handle, const float32_t Ui)
{
    FWC_Obj *obj = (FWC_Obj *)handle;

    // set the new integrator value
    obj->pi.i10 = Ui;
}

//*****************************************************************************
//
//! \brief     Runs the Field Weakening Control (FWC)
//!
//! \param[in] handle   The Field Weakening Control (FWC) handle
//!
//! \param[in] Vs_V   The output voltage vector, V
//!
//! \param[in] VsRef_V  The reference voltage vector, V
//!
//*****************************************************************************
static __attribute__((always_inline))
void FWC_computeCurrentAngle(FWC_Handle handle,
                            const float32_t Vs_V, const float32_t VsRef_V)
{
    FWC_Obj *obj = (FWC_Obj *)handle;
    float32_t angle_rad;

    float32_t Vs_in_V = Vs_V;
    float32_t VsRef_in_V = VsRef_V;

    if(FWC_getFlagEnable(handle) == true)
    {
        // Perform the Field Weakening Control (FWC)
        angle_rad = DCL_runPISeries(&(obj->pi),
                                    VsRef_in_V,
                                    Vs_in_V);

        obj->angleCurrent_rad = MATH_PI_OVER_TWO - angle_rad;
    }
    else
    {
        obj->angleCurrent_rad = MATH_PI_OVER_TWO;
    }

    return;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // FWC_H
