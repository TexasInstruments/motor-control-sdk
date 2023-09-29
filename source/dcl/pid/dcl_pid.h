/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 
#ifndef _DCL_PID_H_
#define _DCL_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_pid.h
 *  \brief      Contains 32-bit PID controller with its related structures and functions
 */

#include "../dcl_common.h"

//--- Linear PID controller --------------------------------------------------

//! \brief          Defines DCL_PID shadow parameter set
//!                 used for updating controller parameter
//!
typedef struct dcl_pid_sps
{
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Kd;       //!< Derivative gain
    float32_t Kr;       //!< Set point weight, default is 1
    float32_t c1;       //!< D path filter coefficient 1, default is 1
    float32_t c2;       //!< D path filter coefficient 2, default is 0
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
} DCL_PID_SPS;

//! \brief          Defines default values to initialize DCL_PID_SPS
//!
#define PID_SPS_DEFAULTS { 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, -1.0f }

//! \brief          DCL_PID object for storing PID specific parameters
//!
typedef _DCL_VOLATILE struct dcl_pid
{
    /* controller parameter */
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t Kd;       //!< Derivative gain
    float32_t Kr;       //!< Set point weight, default is 1
    float32_t c1;       //!< D path filter coefficient 1, default is 1
    float32_t c2;       //!< D path filter coefficient 2, default is 0
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit  

    /* internal storage */ 
    float32_t d2;       //!< D path feedback value (Kd * c1)
    float32_t d3;       //!< D path feedback value (c2)
    float32_t i10;      //!< I path feedback value
    float32_t i14;      //!< I path saturation storage 
    
    /* miscellaneous */
    DCL_PID_SPS *sps;   //!< updates controller parameter
    DCL_CSS *css;       //!< configuration & debugging
} DCL_PID, *PID_Handle;

//! \brief          Defines default values to initialize the DCL_PID structure
//!
#define PID_DEFAULTS {  1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, \
                        1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, \
        &(DCL_PID_SPS)PID_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_PID
//!                 Example: DCL_PID pid_ctrl = { 
//!                                             .Kp = 1.0f,
//!                                             .Ki = 0.0f,
//!                                             ...
//!                                             .Umin = -1.0f,
//!                                             PID_INT_DEFAULTS
//!                                           };
#define PID_INT_DEFAULTS .d2=0.0f, .d3=0.0f, .i10=0.0f, .i14=1.0f, \
    .sps=&(DCL_PID_SPS)PID_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS 
                        
//! \brief          Initialize DCL_PID struct with default parameters
//!                 Example: DCL_PID* pid_ctrl = DCL_initPID();
//!
//! \return         A DCL_PID* pointer
//!
#define DCL_initPID() &(DCL_PID)PID_DEFAULTS

//! \brief          Initialize DCL_PID struct with input controller parameters
//!                 Example: DCL_PID* pid_ctrl = DCL_initPIDasParam(1.0f,0.0f,0.0f,1.0f,1.0f,0.0f,1.0f,-1.0f);
//!                 Note: input parameter needs to be in the same order as listed in PID_SPS struct
//!
//! \return         A DCL_PID* pointer
//!
#define DCL_initPIDasParam(kp,ki,kd,kr,_c1,_c2,umax,umin) &(DCL_PID){ .Kp=kp, .Ki=ki, .Kd=kd, .Kr=kr, \
                                        .c1=_c1, .c2=_c2, .Umax=umax, .Umin=umin, PID_INT_DEFAULTS }

//! \brief              Initialize DCL_PID struct with SPS parameters
//!                     Example: DCL_PID_SPS pid_sps = { .Kp = , .Ki = , ...}; //initial parameter
//!                              DCL_PID pid_ctrl;
//!                              DCL_initPIasSPS(&pid_ctrl,&pid_sps);
//!                     Note: The newly declared DCL_PID structure will use the SPS input parameter as its attribute for sps.
//!
//! \param[in] pid_ptr  DCL_PID* pointer that needs to be initialized
//! \param[in] sps_ptr  DCL_PID_SPS* pointer with assigned parameters
//! \return             Returns DCL_PID* with set sps parameters, default parameter will be used
//!                     if sps_ptr is not specified   
//!
#define DCL_initPIDasSPS(pid_ptr,sps_ptr)                                              \
({                                                                                     \
    DCL_PID* new_pid = (pid_ptr) ? pid_ptr : DCL_initPID();                            \
    DCL_PID_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_PID_SPS)PID_SPS_DEFAULTS;       \
    if(sps_ptr)                                                                        \
    {                                                                                  \
        *new_pid = (DCL_PID){ (new_sps)->Kp, (new_sps)->Ki, (new_sps)->Kd,(new_sps)->Kr,\
        (new_sps)->c1, (new_sps)->c2, (new_sps)->Umax, (new_sps)->Umin, 0.0f, 0.0f,    \
        0.0f, 1.0f, (DCL_PID_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };               \
    }                                                                                  \
    new_pid;                                                                           \
})

//! \brief            Resets PID internal storage data with interrupt protection
//!
//! \param[in] pid    Pointer to the DCL_PID structure
//!
_DCL_CODE_ACCESS
void DCL_resetPID(DCL_PID *pid)
{
    dcl_interrupt_t ints = DCL_disableInts();
    pid->d2 = pid->d3 = pid->i10 = 0.0f;
    pid->i14 = 1.0f;
    DCL_restoreInts(ints);
}

//! \brief            Loads PID tuning parameter from its SPS parameter
//!
//! \param[in] pid    Pointer to the active DCL_PID controller structure
//!
_DCL_CODE_ACCESS
void DCL_fupdatePID(DCL_PID *pid)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    float32_t tau = (2.0f - pid->sps->c1 * pid->css->t_sec) / (2.0f * pid->sps->c1);
    float32_t ec2 = pid->sps->c1 * (pid->css->t_sec - 2.0f * tau) / 2.0f;
    uint32_t err_code = dcl_none;
    err_code |= DCL_isValue(pid->sps->c2, ec2) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->sps->Umax > pid->sps->Umin) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->css->t_sec > 0.0f) ? dcl_none : dcl_param_range_err;
    err_code |= ((pid->sps->Kp > 0.0f) && (pid->sps->Ki > 0.0f) && (pid->sps->Kd > 0.0f) && (pid->sps->Kr > 0.0f)) ? dcl_none : dcl_param_range_err ;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    pid->Kp = pid->sps->Kp;
    pid->Ki = pid->sps->Ki;
    pid->Kd = pid->sps->Kd;
    pid->Kr = pid->sps->Kr;
    pid->c1 = pid->sps->c1;
    pid->c2 = pid->sps->c2;
    pid->Umax = pid->sps->Umax;
    pid->Umin = pid->sps->Umin;
}

//! \brief            Updates PID parameter from its SPS parameter with interrupt protection
//!
//! \param[in] pid    Pointer to the active DCL_PID controller structure
//! \return           'true' if update is successful, otherwise 'false'
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
bool DCL_updatePID(DCL_PID *pid)
{
 
#ifdef DCL_ERROR_HANDLING_ENABLED
    float32_t tau = (2.0f - pid->sps->c1 * pid->css->t_sec) / (2.0f * pid->sps->c1);
    float32_t ec2 = pid->sps->c1 * (pid->css->t_sec - 2.0f * tau) / 2.0f;
    uint32_t err_code = dcl_none;
    err_code |= DCL_isValue(pid->sps->c2, ec2) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->sps->Umax > pid->sps->Umin) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->css->t_sec > 0.0f) ? dcl_none : dcl_param_range_err;
    err_code |= ((pid->sps->Kp > 0.0f) && (pid->sps->Ki > 0.0f) && (pid->sps->Kd > 0.0f) && (pid->sps->Kr > 0.0f)) ? dcl_none : dcl_param_range_err ;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    if (!DCL_getUpdateStatus(pid))
    {
        dcl_interrupt_t ints = DCL_disableInts();
        DCL_setUpdateStatus(pid);
        pid->Kp = pid->sps->Kp;
        pid->Ki = pid->sps->Ki;
        pid->Kd = pid->sps->Kd;
        pid->Kr = pid->sps->Kr;
        pid->c1 = pid->sps->c1;
        pid->c2 = pid->sps->c2;
        pid->Umax = pid->sps->Umax;
        pid->Umin = pid->sps->Umin;
        DCL_clearUpdateStatus(pid);
        DCL_restoreInts(ints);
        return true;
    }
    return false;
}

//! \brief           A conditional update based on the pending-for-update flag.
//!                  If the pending status is set, the function will update PID
//!                  parameter from its SPS parameter and clear the status flag on completion.
//!                  Note: Use DCL_setPendingStatus(pid) to set the pending status.
//!     
//! \param[in] pid   Pointer to the DCL_PID controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
bool DCL_pendingUpdatePID(DCL_PID *pid)
{
    if (DCL_getPendingStatus(pid) && DCL_updatePID(pid))
    {
        DCL_clearPendingStatus(pid);
        return true;
    }
    return false;
}

//! \brief           Update SPS parameter with active param, userful when needing
//!                  to update only few active param from SPS and keep rest the same
//!   
//! \param[in] pid   Pointer to the active DCL_PID controller structure
//!
_DCL_CODE_ACCESS
void DCL_updatePIDSPS(DCL_PID *pid)
{
    pid->sps->Kp = pid->Kp;
    pid->sps->Ki = pid->Ki;
    pid->sps->Kd = pid->Kd;
    pid->sps->Kr = pid->Kr;
    pid->sps->c1 = pid->c1;
    pid->sps->c2 = pid->c2;
    pid->sps->Umax = pid->Umax;
    pid->sps->Umin = pid->Umin;
}

//! \brief            Loads the derivative path filter shadow coefficients.
//!                   Note: Sampling period pid->css->t_sec are used in the calculation.
//!                   New coefficients take effect when DCL_updatePID() is called.
//!
//! \param[in] pid    Pointer to the DCL_PID structure
//! \param[in] fc     The desired filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
void DCL_setPIDfilterBW(DCL_PID *pid, float32_t fc)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((fc >= 1.0f / (2.0f * pid->css->t_sec)) || (fc <= 0.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float32_t t_sec = pid->css->t_sec;
    float32_t tau = 1.0f / (2.0f * CONST_PI * fc);
    pid->sps->c1 = 2.0f / (t_sec + (2.0f * tau));
    pid->sps->c2 = (t_sec - (2.0f * tau)) / (t_sec + (2.0f * tau));
}

//! \brief           Loads the PID derivative path filter active coefficients
//!                  Note: Sampling period pid->css->t_sec are used in the calculation.
//!                  New coefficients take effect immediately. SPS & CSS contents are unaffected.            
//!
//! \param[in] pid   Pointer to the DCL_PID structure
//! \param[in] fc    The desired filter bandwidth in Hz
//! \param[in] t_sec The controller update rate in seconds
//!
_DCL_CODE_ACCESS
void DCL_setActivePIDfilterBW(DCL_PID *pid, float32_t fc, float32_t t_sec)
{
    
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((fc >= 1.0f / (2.0f * t_sec)) || (fc <= 0.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float32_t tau = 1.0f / (2.0f * CONST_PI * fc);
    pid->c1 = 2.0f / (t_sec + (2.0f * tau));
    pid->c2 = (t_sec - (2.0f * tau)) / (t_sec + (2.0f * tau));
}

//! \brief          Calculates the active derivative path filter bandwidth in Hz.
//!                 Note: Sampling period pid->css->t_sec are used in the calculation.
//! \param[in] pid  Pointer to the DCL_PID structure
//! \return         The filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
float32_t DCL_getPIDfilterBW(DCL_PID *pid)
{
    float32_t tau = ((2.0f - pid->c1 * pid->css->t_sec) / (2.0f * pid->c1));
    return(1.0f / (2.0f * CONST_PI * tau));
}

//! \brief          Configures a series PID controller parameter in ZPK form.
//!                 Note: Sampling period pid->css->t_sec are used in the calculation.
//!                 Parameters take effect after call to DCL_updatePID().
//!                 Only z1, z2 & p2 considered, p1 = 0 assumed.
//!
//! \param[in] pid  Pointer to the active DCL_PID controller structure
//! \param[in] zpk  Pointer to the DCL_ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadSeriesPIDasZPK(DCL_PID *pid, DCL_ZPK3 *zpk)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= DCL_isZero(cimagf(zpk->z1) + cimagf(zpk->z2)) ? dcl_none : dcl_param_invalid_err;
    err_code |= DCL_isZero(cimagf(zpk->p2)) ? dcl_none : dcl_param_invalid_err;
    err_code |= (crealf(zpk->p2) <= DCL_c2Limit) ? dcl_none :dcl_param_invalid_err;
    err_code |= (zpk->K >= 0.0f) ? dcl_none : dcl_param_range_err;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float32_t beta1 = -(float32_t) crealf(zpk->z1 + zpk->z2);
    float32_t beta0 = (float32_t) crealf(zpk->z1 * zpk->z2);
    float32_t alpha1 = -(float32_t) crealf(zpk->p1 + zpk->p2);
    float32_t alpha0 = (float32_t) crealf(zpk->p1 * zpk->p2);
    float32_t t_sec = pid->css->t_sec;
    float32_t a0p = 4.0f + (alpha1 * 2.0f * t_sec) + (alpha0 * t_sec * t_sec);
    float32_t b0 = zpk->K * (4.0f + (beta1 * 2.0f * t_sec) + (beta0 * t_sec *t_sec)) / a0p;
    float32_t b1 = zpk->K * (-8.0f + (2.0f * beta0 * t_sec * t_sec)) / a0p;
    float32_t b2 = zpk->K * (4.0f - (beta1 * 2.0f * t_sec) + (beta0 * t_sec * t_sec)) / a0p;
    float32_t a2 = (4.0f - (alpha1 * 2.0f * t_sec) + (alpha0 * t_sec * t_sec)) / a0p;
    float32_t c2 = -a2;
    float32_t tau = (t_sec / 2.0f) * (1.0f - c2) / (1.0f + c2);
    pid->sps->c1 = 2.0f / (t_sec + 2.0f * tau);
    pid->sps->c2 = c2;
    float32_t det = (c2 + 1.0f);
    det *= det;

#ifdef DCL_ERROR_HANDLING_ENABLED
    err_code = dcl_none;
    err_code |= (DCL_isZero(det)) ? dcl_param_invalid_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float32_t k1 = ((c2 * b0) - b1 - ((2.0f + c2) * b2)) / det;
    float32_t k2 = (c2 + 1.0f) * (b0 + b1 + b2) / det;
    float32_t k3 = ((c2 * c2 * b0) - (c2 * b1) + b2) / det;
    pid->sps->Kp = k1;
    pid->sps->Ki = k2 / k1;
    pid->sps->Kd = k3 / (k1 * pid->sps->c1);

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = det;
#endif

}

//! \brief            Configures a parallel PID controller in ZPK form.
//!                   Note: Sampling period pid->css->t_sec are used in the calculation.
//!                   Parameters take effect after call to DCL_updatePID().
//!                   Only z1, z2 & p2 considered, p1 = 0 assumed.
//!
//! \param[in] pid    Pointer to the active DCL_PID controller structure
//! \param[in] zpk    Pointer to the DCL_ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadParallelPIDasZPK(DCL_PID *pid, DCL_ZPK3 *zpk)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= DCL_isZero(cimagf(zpk->z1) + cimagf(zpk->z2)) ? dcl_none : dcl_param_invalid_err;
    err_code |= DCL_isZero(cimagf(zpk->p2)) ? dcl_none : dcl_param_invalid_err;
    err_code |= (crealf(zpk->p2) <= DCL_c2Limit) ? dcl_none : dcl_param_invalid_err;
    err_code |= (zpk->K >= 0.0f) ? dcl_none : dcl_param_range_err;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float32_t beta1 = -(float32_t) crealf(zpk->z1 + zpk->z2);
    float32_t beta0 = (float32_t) crealf(zpk->z1 * zpk->z2);
    float32_t alpha1 = -(float32_t) crealf(zpk->p1 + zpk->p2);
    float32_t alpha0 = (float32_t) crealf(zpk->p1 * zpk->p2);
    float32_t t_sec = pid->css->t_sec;
    float32_t a0p = 4.0f + (alpha1 * 2.0f * t_sec) + (alpha0 * t_sec * t_sec);
    float32_t b0 = zpk->K * (4.0f + (beta1 * 2.0f * t_sec) + (beta0 * t_sec * t_sec)) / a0p;
    float32_t b1 = zpk->K * (-8.0f + (2.0f * beta0 * t_sec * t_sec)) / a0p;
    float32_t b2 = zpk->K * (4.0f - (beta1 * 2.0f * t_sec) + (beta0 * t_sec * t_sec)) / a0p;
    float32_t a2 = (4.0f - (alpha1 * 2.0f * t_sec) + (alpha0 * t_sec * t_sec)) / a0p;
    float32_t c2 = -a2;
    float32_t tau = (t_sec / 2.0f) * (1.0f - c2) / (1.0f + c2);
    pid->sps->c1 = 2.0f / (t_sec + 2.0f * tau);
    pid->sps->c2 = c2;
    float32_t det = (c2 + 1.0f);
    det *= det;

#ifdef DCL_ERROR_HANDLING_ENABLED
    err_code = (DCL_isZero(det)) ? dcl_param_invalid_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    pid->sps->Kp = ((c2 * b0) - b1 - ((2.0f + c2) * b2)) / det;
    pid->sps->Ki = (c2 + 1.0f) * (b0 + b1 + b2) / det;
    pid->sps->Kd = ((c2 * c2 * b0) - (c2 * b1) + b2) / (det * pid->sps->c1);

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = det;
#endif

}

//! \brief            Executes an ideal form PID controller
//!
//! \param[in] pid    Pointer to the DCL_PID structure
//! \param[in] rk     The controller set-point reference
//! \param[in] yk     The measured feedback value
//! \param[in] lk     External output clamp flag
//! \return           The control effort
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
float32_t DCL_runPIDSeries(DCL_PID *pid, float32_t rk, float32_t yk, float32_t lk)
{
    float32_t v1, v4, v5, v8, v9, v10, v12;

    v5 = (pid->Kr * rk) - yk;
    v8 = ((rk - yk) * pid->Ki * pid->Kp * pid->i14) + pid->i10;
    pid->i10 = v8;
    v1 = yk * pid->Kd * pid->c1;
    v4 = v1 - pid->d2 - pid->d3;
    pid->d2 = v1;
    pid->d3 = v4 * pid->c2;
    v9 = ((v5 - v4) * pid->Kp) + v8;
    v10 = DCL_runSat(v9, pid->Umax, pid->Umin);
    v12 = (v10 == v9) ? 1.0f : 0.0f;
    pid->i14 = v12 * lk;

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = v4;
#endif

    return(v10);
}

//! \brief            Executes a parallel form PID controller
//!
//! \param[in] pid    Pointer to the DCL_PID structure
//! \param[in] rk     The controller set-point reference
//! \param[in] yk     The measured feedback value
//! \param[in] lk     External output clamp flag
//! \return           The control effort
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
float32_t DCL_runPIDParallel(DCL_PID *pid, float32_t rk, float32_t yk, float32_t lk)
{
    float32_t v1, v4, v5, v6, v8, v9, v10, v12;

    v5 = rk - yk;
    v6 = v5 * pid->Kp;
    v8 = v5 * pid->Ki * pid->i14 + pid->i10;
    pid->i10 = v8;
    v1 = v5 * pid->Kd * pid->c1;
    v4 = v1 - pid->d2 - pid->d3;
    pid->d2 = v1;
    pid->d3 = v4 * pid->c2;
    v9 = v6 + v8 + v4;
    v10 = DCL_runSat(v9, pid->Umax, pid->Umin);
    v12 = (v10 == v9) ? 1.0f : 0.0f;
    pid->i14 = v12 * lk;

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = v8;
#endif

    return(v10);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_PID_H_
