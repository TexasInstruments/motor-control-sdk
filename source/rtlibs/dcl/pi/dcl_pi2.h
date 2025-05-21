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
 
#ifndef _DCL_PI2_H_
#define _DCL_PI2_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  \file       dcl_pi2.h
 *  \brief      Contains PI2 (double integrator) controller 
 *              with its related structures and functions
 */             

#include "../dcl_common.h"

//--- Linear PI2 controller --------------------------------------------------

//! \brief          Defines DCL_PI2 shadow parameter set
//!                 used for updating controller parameter
//!
typedef struct dcl_pi2_sps
{
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
} DCL_PI2_SPS;

//! \brief          Defines default values to initialize DCL_PI2_SPS
//!
#define PI2_SPS_DEFAULTS { 1.0f, 0.0f, 1.0f, -1.0f }

//! \brief          DCL_PI2 object for storing PI2 specific parameters
//!                 PI2 - PI controller with double integrators
//!
typedef _DCL_VOLATILE struct dcl_pi2
{
    /* controller parameter */
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit 

    /* internal storage */    
    float32_t i6;       //!< Integrator 1 feedback storage
    float32_t i9;       //!< Integrator 2 feedback storage
    float32_t i12;      //!< Saturation 1 multiplier, 1 means no saturation and 0 means fully saturated
    float32_t i13;      //!< Saturation 2 multiplier, 1 means no saturation and 0 means fully saturated

    /* miscellaneous */
    DCL_PI2_SPS *sps;   //!< updates controller parameter
    DCL_CSS *css;       //!< configuration & debugging
} DCL_PI2;

//! \brief  Defines default values to initialize DCL_PI2
//!
#define PI2_DEFAULTS { 1.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f, \
                &(DCL_PI2_SPS)PI2_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_PI2
//!                 Example: DCL_PI2 pi2_ctrl = { 
//!                                             .Kp = 1.0f,
//!                                             .Ki = 0.0f,
//!                                             .Umax = 1.0f,
//!                                             .Umin = -1.0f,
//!                                             PI2_INT_DEFAULTS
//!                                             };
#define PI2_INT_DEFAULTS .i6=0.0f, .i9=0.0f, .i12=1.0f, .i13=1.0f, \
.sps=&(DCL_PI2_SPS)PI2_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS           

//! \brief          Initialize DCL_PI2 struct with default parameters
//!                 Example: DCL_PI2 pi2_ctrl = DCL_initPI2();
//!
//! \return         A DCL_PI2* pointer
//!
#define DCL_initPI2() &(DCL_PI2)PI2_DEFAULTS

//! \brief          Initialize DCL_PI2 struct with input controller parameters
//!                 Example: DCL_PI2* pi2_ctrl = DCL_initPI2asParam(1.0f,0.0f,1.0f,-1.0f);
//! \note           Note: input parameter needs to be in the same order as listed in PI2_SPS struct
//!
//! \return         A DCL_PI2* pointer
//!
#define DCL_initPI2asParam(kp,ki,umax,umin) &(DCL_PI2){ .Kp=kp, .Ki=ki, \
                                .Umax=umax, .Umin=umin, PI2_INT_DEFAULTS }

//! \brief              Initialize DCL_PI2 struct with sps parameters
//!                     Example: DCL_PI2_SPS pi2_sps = { .Kp = , .Ki = , ...};
//!                              DCL_PI2 pi2_ctrl;
//!                              DCL_initPI2asSPS(&pi2_ctrl,&pi2_sps);
//!
//! \param[in] pi2_ptr  DCL_PI2* pointer that needs to be initialized
//! \param[in] sps_ptr  DCL_PI2_SPS* pointer with assigned parameters
//! \return             Returns DCL_PI2* with set sps parameters, default parameter will be used
//!                     if sps_ptr is not specified   
//!
#define DCL_initPI2asSPS(pi2_ptr,sps_ptr)                                                  \
({                                                                                         \
    DCL_PI2* new_pi = (pi2_ptr) ? pi2_ptr : DCL_initPI2();                                 \
    DCL_PI2_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_PI2_SPS)PI2_SPS_DEFAULTS;           \
    if(sps_ptr)                                                                            \
    {                                                                                      \
        *new_pi = (DCL_PI){ (new_sps)->Kp, (new_sps)->Ki, (new_sps)->Umax, (new_sps)->Umin,\
        0.0f, 0.0f, 1.0f, 1.0f, (DCL_PI_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };        \
    }                                                                                      \
    new_pi;                                                                                \
})

//! \brief            Resets PI2 internal storage data with interrupt protection
//!
//! \param[in] pi2    Pointer to the DCL_PI2 structure
//!
_DCL_CODE_ACCESS
void DCL_resetPI2(DCL_PI2 *pi2)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pi2->i6 = pi2->i9 = 0.0f;
    pi2->i12 = pi2->i13 = 1.0f;
    DCL_restoreInts(ints);
}

//! \brief            Loads PI2 tuning parameter from its SPS parameter without interrupt protection
//!
//! \param[in] pi2    Pointer to the  DCL_PI2 controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdatePI2(DCL_PI2 *pi2)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (pi2->sps->Umax <= pi2->sps->Umin) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pi2->css->T <= 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi2->sps->Kp < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi2->sps->Ki < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi2,err_code);
        DCL_getErrorInfo(pi2);
        DCL_runErrorHandler(pi2);
    }
#endif

    pi2->Ki = pi2->sps->Ki;
    pi2->Kp = pi2->sps->Kp;
    pi2->Umax = pi2->sps->Umax;
    pi2->Umin = pi2->sps->Umin;
}

//! \brief          Loads PI2 tuning parameter from its SPS parameter with interrupt protection
//!
//! \param[in] pi2  Pointer to the DCL_PI2 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updatePI2NoCheck(DCL_PI2 *pi2)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (pi2->sps->Umax <= pi2->sps->Umin) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pi2->css->T <= 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi2->sps->Kp < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi2->sps->Ki < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi2,err_code);
        DCL_getErrorInfo(pi2);
        DCL_runErrorHandler(pi2);
    }
#endif

    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pi2->Ki = pi2->sps->Ki;
    pi2->Kp = pi2->sps->Kp;
    pi2->Umax = pi2->sps->Umax;
    pi2->Umin = pi2->sps->Umin;
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update PI2
//!                  parameter from its SPS parameter and clear the status flag on completion.
//! \note            Note: Use DCL_setUpdateStatus(pi2) to set the update status.
//!     
//! \param[in] pi2    Pointer to the DCL_PI2 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updatePI2(DCL_PI2 *pi2)
{
    if (DCL_getUpdateStatus(pi2))
    {
        DCL_updatePI2NoCheck(pi2);
        DCL_clearUpdateStatus(pi2);
        return true;
    }
    return false;
}

//! \brief          Executes an inline series form PI2 controller on the FPU32
//!
//! \param[in] pi2  Pointer to the DCL_PI2 structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback
//! \return         The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runPI2Series(DCL_PI2 *pi2, float32_t rk, float32_t yk)
{
    float32_t v1, v2, v5, v8, v10, v11, v14;
    bool l1, l2, l3, l4, l5, l6;

    v1 = rk - yk;
    v2 = pi2->Kp * v1;
    v5 = (v1 * pi2->Ki * pi2->i12) + pi2->i6;
    pi2->i6 = v5;
    v8 = (v5 * pi2->i13) + pi2->i9;
    pi2->i9 = v8;
    v10 = v2 + v8;
    v11 = DCL_runSat(v10,pi2->Umax,pi2->Umin); 
    v14 = v10 - v11;
    l1 = (v1 > 0.0f) ? true : false;
    l2 = (v14 > 0.0f) ? true : false;
    l3 = (v14 == 0.0f) ? true : false;
    l4 = (v5 > 0.0f) ? true : false;
    l5 = l3 || (l1 ^ l2);
    l6 = l3 || (l4 ^ l2);
    pi2->i12 = (l5) ? 1.0f : 0.0f;
    pi2->i13 = (l6) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    pi2->css->tpt = v8;
#endif

    return(v11);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_PI2_H_
