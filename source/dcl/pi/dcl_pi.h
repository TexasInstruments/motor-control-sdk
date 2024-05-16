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
 
#ifndef _DCL_PI_H_
#define _DCL_PI_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_pi.h
 *  \brief      Contains PI controller with its related structures and functions 
 */              

#include "../dcl_common.h"

//--- Linear PI controller ---------------------------------------------------

//! \brief          Defines DCL_PI shadow parameter set
//!                 used for updating controller parameter
//!
typedef struct dcl_pi_sps
{
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
    float32_t Imax;     //!< Upper integrator saturation limit, only used in DCL_runPIParallelEnhanced()
    float32_t Imin;     //!< Lower integrator saturation limit, only used in DCL_runPIParallelEnhanced()
} DCL_PI_SPS;

//! \brief          Defines default values to initialize DCL_PI_SPS
//!
#define PI_SPS_DEFAULTS { 1.0f, 0.0f, 1.0f, -1.0f, 1.0f, -1.0f }

//! \brief          DCL_PI object for storing PI specific parameters
//!
typedef _DCL_VOLATILE struct dcl_pi
{
    /* controller parameter */
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Umax;     //!< Upper control saturation limit
    float32_t Umin;     //!< Lower control saturation limit 
    float32_t Imax;     //!< Upper integrator saturation limit, only used in DCL_runPIParallelEnhanced()
    float32_t Imin;     //!< Lower integrator saturation limit, only used in DCL_runPIParallelEnhanced()

    /* internal storage */ 
    float32_t i6;       //!< Saturation multiplier, 1 means no saturation and 0 means fully saturated
    float32_t i10;      //!< Integrator feedback storage
    float32_t i11;      //!< Tustin integrator storage, only used in DCL_runPISeriesTustin()
    
    /* miscellaneous */
    DCL_PI_SPS *sps;    //!< updates controller parameter
    DCL_CSS *css;       //!< configuration & debugging
} DCL_PI;

//! \brief          Defines default values to initialize DCL_PI
//!
#define PI_DEFAULTS { 1.0f, 0.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f, \
                    &(DCL_PI_SPS)PI_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_PI
//!                 Example: DCL_PI pi_ctrl = { 
//!                                             .Kp = 1.0f,
//!                                             .Ki = 0.0f,
//!                                             .Umax = 1.0f,
//!                                             .Umin = -1.0f,
//!                                             .Imax = 0.0f,
//!                                             .Imin = 0.0f,
//!                                             PI_INT_DEFAULTS
//!                                           };
#define PI_INT_DEFAULTS .i6=1.0f, .i10=0.0f, .i11=0.0f, \
.sps=&(DCL_PI_SPS)PI_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS           

//! \brief          Initialize DCL_PI struct with default parameters
//!                 Example: DCL_PI* pi_ctrl = DCL_initPI();
//!
//! \return         A DCL_PI* pointer
//!
#define DCL_initPI() &(DCL_PI)PI_DEFAULTS

//! \brief          Initialize DCL_PI struct with input controller parameters
//!                 Example: DCL_PI* pi_ctrl = DCL_initPIasParam(1.0f,0.0f,1.0f,-1.0f);
//! \note           Note: input parameter needs to be in the same order as listed in PI_SPS struct
//!
//! \return         A DCL_PI* pointer
//!
#define DCL_initPIasParam(kp,ki,umax,umin) &(DCL_PI){ .Kp=kp, .Ki=ki, \
                                .Umax=umax, .Umin=umin, PI_INT_DEFAULTS }

//! \brief              Initialize DCL_PI struct with sps parameters
//!                     Example: DCL_PI_SPS pi_sps = { .Kp = , .Ki = , ...};
//!                              DCL_PI pi_ctrl;
//!                              DCL_initPIasSPS(&pi_ctrl,&pi_sps);
//!
//! \param[in] pi_ptr   DCL_PI* pointer that needs to be initialized
//! \param[in] sps_ptr  DCL_PI_SPS* pointer with assigned parameters
//! \return             Returns DCL_PI* with set sps parameters, default parameter will be used
//!                     if sps_ptr is not specified   
//!
#define DCL_initPIasSPS(pi_ptr,sps_ptr)                                                    \
({                                                                                         \
    DCL_PI* new_pi = (pi_ptr) ? pi_ptr : DCL_initPI();                                     \
    DCL_PI_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_PI_SPS)PI_SPS_DEFAULTS;              \
    if(sps_ptr)                                                                            \
    {                                                                                      \
        *new_pi = (DCL_PI){ (new_sps)->Kp, (new_sps)->Ki, (new_sps)->Umax, (new_sps)->Umin,\
        (new_sps)->Imax, (new_sps)->Imin, 1.0f, 0.0f, 0.0f,                                \
        (DCL_PI_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };                                \
    }                                                                                      \
    new_pi;                                                                                \
})

//! \brief          Resets PI internal storage data with interrupt protection
//!
//! \param[in] pi   Pointer to the DCL_PI structure
//!
_DCL_CODE_ACCESS
void DCL_resetPI(DCL_PI *pi)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pi->i6 = 1.0f;
    pi->i10 = pi->i11 = 0.0f;
    DCL_restoreInts(ints);
}

//! \brief          Loads PI tuning parameter from its SPS parameter without interrupt protection   
//!        
//! \param[in] pi   Pointer to the DCL_PI controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdatePI(DCL_PI *pi)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (pi->sps->Umax <= pi->sps->Umin) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pi->css->T <= 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi->sps->Kp < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi->sps->Ki < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi,err_code);
        DCL_getErrorInfo(pi);
        DCL_runErrorHandler(pi);
    }
#endif

    pi->Kp = pi->sps->Kp;
    pi->Ki = pi->sps->Ki;
    pi->Umax = pi->sps->Umax;
    pi->Umin = pi->sps->Umin;
    pi->Imax = pi->sps->Imax;
    pi->Imin = pi->sps->Imin;
}

//! \brief          Loads PI tuning parameter from its SPS parameter with interrupt protection
//!
//! \param[in] pi   Pointer to the DCL_PI controller structure
//!
_DCL_CODE_ACCESS
void DCL_updatePINoCheck(DCL_PI *pi)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (pi->sps->Umax <= pi->sps->Umin) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pi->css->T <= 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi->sps->Kp < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (pi->sps->Ki < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi,err_code);
        DCL_getErrorInfo(pi);
        DCL_runErrorHandler(pi);
    }
#endif

    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pi->Kp = pi->sps->Kp;
    pi->Ki = pi->sps->Ki;
    pi->Umax = pi->sps->Umax;
    pi->Umin = pi->sps->Umin;
    pi->Imax = pi->sps->Imax;
    pi->Imin = pi->sps->Imin;
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update PI
//!                  parameter from its SPS parameter and clear the status flag on completion.
//! \note            Note: Use DCL_setUpdateStatus(pi) to set the update status.
//!     
//! \param[in] pi    Pointer to the DCL_PI controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updatePI(DCL_PI *pi)
{
    if (DCL_getUpdateStatus(pi))
    {
        DCL_updatePINoCheck(pi);
        DCL_clearUpdateStatus(pi);
        return true;
    }
    return false;
}

//! \brief          Configures a series PI controller in "zero-pole-gain" form
//! \note           Note: Sampling period pi->css->T are used in the calculation.
//!                 New settings take effect after DCL_updatePI().
//!                 Only z1 considered in DCL_ZPK3, other poles & zeros ignored.
//!                 Zero frequency assumed to be in radians/sec.
//!
//! \param[in] pi   Pointer to the DCL_PI controller structure
//! \param[in] zpk  Pointer to the ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadSeriesPIasZPK(DCL_PI *pi, DCL_ZPK3 *zpk) 
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (zpk->K < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (crealf(zpk->z1) > (1.0f / (2.0f * pi->css->T))) ? dcl_param_warn_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi,err_code);
        DCL_getErrorInfo(pi);
        DCL_runErrorHandler(pi);
    }
#endif

    float32_t T = pi->css->T;
    float32_t z1 = (float32_t) crealf(zpk->z1);
    pi->sps->Kp = zpk->K * (1.0f + (T * z1) / 2.0f);
    pi->sps->Ki = (-2.0f * T * z1) / (2.0f + (T * z1));

}

//! \brief          Configures a parallel PI controller in "zero-pole-gain" form
//! \note           Note: Sampling period pi->css->T are used in the calculation.
//!                 New settings take effect after DCL_updatePI().
//!                 Zero frequency assumed to be in radians/sec.
//!
//! \param[in] pi   Pointer to the active DCL_PI controller structure
//! \param[in] zpk  Pointer to the ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadParallelPIasZPK(DCL_PI *pi, DCL_ZPK3 *zpk)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (zpk->K < 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= (crealf(zpk->z1) > (1.0f / (2.0f * pi->css->T))) ? dcl_param_warn_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pi,err_code);
        DCL_getErrorInfo(pi);
        DCL_runErrorHandler(pi);
    }
#endif
    
    float32_t T = pi->css->T;
    float32_t z1 = (float32_t) crealf(zpk->z1);
    pi->sps->Kp = zpk->K * (1.0f + (T * z1) / 2.0f);
    pi->sps->Ki = -zpk->K * T * z1;
}

//! \brief          Executes an inline series form PI controller
//!
//! \param[in] pi   Pointer to the DCL_PI structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \return         The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runPISeries(DCL_PI *pi, float32_t rk, float32_t yk)
{
    float32_t v2, v4, v5, v9;

    v2 = pi->Kp * (rk - yk);
    v4 = pi->i10 + (pi->Ki * pi->i6 * v2);
    v5 = v2 + v4;
    v9 = DCL_runSat(v5, pi->Umax, pi->Umin);
    pi->i6 = (v5 == v9) ? 1.0f : 0.0f;
    pi->i10 = v4;  

#ifdef DCL_TESTPOINTS_ENABLED
    pi->css->tpt = v5;
#endif

    return(v9);
}

//! \brief           Executes a parallel form PI controller
//!                  Implemented as inline C function
//!
//! \param[in] pi    Pointer to the DCL_PI structure
//! \param[in] rk    The controller set-point reference
//! \param[in] yk    The measured feedback value
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runPIParallel(DCL_PI *pi, float32_t rk, float32_t yk)
{
    float32_t v1, v2, v4, v5, v9;

    v1 = rk - yk;
    v2 = pi->Kp * v1;
    v4 = (v1 * pi->Ki * pi->i6) + pi->i10;
    pi->i10 = v4;
    v5 = v2 + v4;
    v9 = DCL_runSat(v5, pi->Umax, pi->Umin);
    pi->i6 = (v5 == v9) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    pi->css->tpt = v5;
#endif

    return(v9);
}

//! \brief           Executes a parallel form PI controller with
//!                  enhanced anti-windup logic incorporating an
//!                  addintional integrator saturation clamp
//!
//! \param[in] pi    Pointer to the DCL_PI structure
//! \param[in] rk    The controller set-point reference
//! \param[in] yk    The measured feedback value
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runPIParallelEnhanced(DCL_PI *pi, float32_t rk, float32_t yk)
{
    float32_t v1, v5, v7, v8;
    bool l11, l14, l17, l18, l19;

    v1 = rk - yk;
    v5 = (v1 * pi->Ki * pi->i6) + pi->i10;
    pi->i10 = v5;
    v7 = (v1 * pi->Kp) + v5;
    v8 = DCL_runSat(v7, pi->Umax, pi->Umin);
    l17 = (v7 == v8) ? true : false;
    l11 = (DCL_runSat(v5, pi->Imax, pi->Imin) == v5) ? true : false;
    l19 = (v5 > 0) ? true : false;
    l14 = (v1 > 0) ? true : false;
    l18 = l17 && (!l11 || (l19 ^ l14));
    pi->i6 = (l18) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    pi->css->tpt = v7;
#endif

    return(v8);
}

//! \brief           Executes a series form PI controller with Tustin integrator
//!
//! \param[in] pi    Pointer to the DCL_PI structure
//! \param[in] rk    The controller set-point reference
//! \param[in] yk    The measured feedback value
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runPISeriesTustin(DCL_PI *pi, float32_t rk, float32_t yk)
{
    float32_t v2, v4, v5, v8, v9;

    v2 = (rk - yk) * pi->Kp;
    v8 = v2 * pi->Ki * pi->i6;
    v4 = v8 + pi->i11 + pi->i10;
    v5 = v2 + v4;
    pi->i10 = v4;
    pi->i11 = v8;
    v9 = DCL_runSat(v5, pi->Umax, pi->Umin);
    pi->i6 = (v5 == v9) ? 1.0f : 0.0f;

#ifdef DCL_TESTPOINTS_ENABLED
    pi->css->tpt = v4;
#endif

    return(v9);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_PI_H_
