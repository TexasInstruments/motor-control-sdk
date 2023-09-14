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
 
#ifndef _DCL_PID64_H_
#define _DCL_PID64_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_pidf64.h
 *  \brief      Contains 64-bit PID controller with its related structures and functions
 */  

#include "../dcl_common.h"

//--- Linear PID 64bit controller --------------------------------------------------

//! \brief          Defines the shadow PID64 controller structure
//!                 used for updating controller parameter
//!
typedef struct dcl_pid64_sps {
    float64_t Kp;       //!< Proportional gain
    float64_t Ki;       //!< Integral gain
    float64_t Kd;       //!< Derivative gain
    float64_t Kr;       //!< Set point weight, default is 1
    float64_t c1;       //!< D-term filter coefficient 1, default is 1
    float64_t c2;       //!< D-term filter coefficient 2, default is 0
    float64_t Umax;     //!< Upper saturation limit
    float64_t Umin;     //!< Lower saturation limit
} DCL_PIDF64_SPS;

//! \brief          Defines default values to initialize the DCL_PIDF64 shadow structure
//!
#define PIDF64_SPS_DEFAULTS { 1.0L, 0.0L, 0.0L, 1.0L, 1.0L, 0.0L, 1.0L, -1.0L }

//! \brief          PID 64bit object for storing PID specific 64bit parameters
//!
typedef _DCL_VOLATILE struct dcl_pidf64 {
    /* controller parameter */
    float64_t Kp;       //!< Proportional gain
    float64_t Ki;       //!< Integral gain
    float64_t Kd;       //!< Derivative gain
    float64_t Kr;       //!< Set point weight, default is 1
    float64_t c1;       //!< D-term filter coefficient 1, default is 1
    float64_t c2;       //!< D-term filter coefficient 2, default is 0
    float64_t Umax;     //!< Upper saturation limit
    float64_t Umin;     //!< Lower saturation limit

    /* internal storage */ 
    float64_t d2;       //!< D path feedback value (Kd * c1)
    float64_t d3;       //!< D path feedback value (c2)
    float64_t i10;      //!< I path feedback value
    float64_t i14;      //!< I path saturation storage 
    
    /* miscellaneous */
    DCL_PIDF64_SPS *sps;  //!< updates controller parameter
    DCL_CSSF64 *css;      //!< configuration & debugging
} DCL_PIDF64, *PIDF64_Handle;

//! \brief          Defines default values to initialize the DCL_PID64 active structure
//!
#define PIDF64_DEFAULTS {  1.0L, 0.0L, 0.0L, 1.0L, 1.0L, 0.0L, \
                           1.0L, -1.0L, 0.0L, 0.0L, 0.0L, 1.0L, \
&(DCL_PIDF64_SPS)PIDF64_SPS_DEFAULTS, &(DCL_CSSF64)DCL_CSSF64_DEFAULTS }
                        
//! \brief          Macro for internal default values to initialize DCL_PIDF64
//!                 Example: DCL_PIDF64 pid_ctrl = { 
//!                                             .Kp = 1.0L,
//!                                             .Ki = 0.0L,
//!                                             ...
//!                                             .Umin = -1.0L,
//!                                             PIDF64_INT_DEFAULTS
//!                                           };
#define PIDF64_INT_DEFAULTS .d2=0.0L, .d3=0.0L, .i10=0.0L, .i14=1.0L, \
    .sps=&(DCL_PIDF64_SPS)PI_SPS_DEFAULTS, .css=&(DCL_CSSF64)DCL_CSS_DEFAULTS 

//! \brief          Initialize DCL_PIDF64 struct with default parameters
//!                 Example: DCL_PIDF64* pid_ctrl = DCL_initPID();
//!
//! \return         A DCL_PIDF64* pointer
//!
#define DCL_initPIDF64() &(DCL_PIDF64)PIDF64_DEFAULTS

//! \brief          Initialize DCL_PIDF64 struct with input controller parameters
//!                 Example: DCL_PIDF64* pid_ctrl = DCL_initPIDF64asParam(1.0L,0.0L,0.0L,1.0L,1.0L,0.0L,1.0L,-1.0L);
//!                 Note: input parameter needs to be in the same order as listed in PIDF64_SPS struct
//!
//! \return         A DCL_PID* pointer
//!
#define DCL_initPIDF64asParam(kp,ki,kd,kr,_c1,_c2,umax,umin) &(DCL_PIDF64){ .Kp=kp, .Ki=ki, .Kd=kd, .Kr=kr, \
                                            .c1=_c1, .c2=_c2, .Umax=umax, .Umin=umin, PIDF64_INT_DEFAULTS }

//! \brief              Initialize DCL_PIDF64 struct with sps parameters
//!                     Example: DCL_PIDF64_SPS pid_sps = { .Kp = , .Ki = , ...}; //initial parameter
//!                              DCL_PIDF64 pid_ctrl;
//!                              DCL_initPIasSPS(&pid_ctrl,&pid_sps);
//!
//! \param[in] pid_ptr  DCL_PIDF64* pointer that needs to be initialized
//! \param[in] sps_ptr  DCL_PIDF64_SPS* pointer with assigned parameters
//! \return             Returns DCL_PID* with set sps parameters, default parameter will be used
//!                     if sps_ptr is not specified   
//!
#define DCL_initPIDF64asSPS(pid_ptr,sps_ptr)                                           \
({                                                                                     \
    DCL_PIDF64* new_pid = (pid_ptr) ? pid_ptr : DCL_initPID();                         \
    DCL_PIDF64_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_PIDF64_SPS)PID_SPS_DEFAULTS; \
    if(sps_ptr)                                                                        \
    {                                                                                  \
        *new_pi = (DCL_PID){ (new_sps)->Kp, (new_sps)->Ki, (new_sps)->Kd,(new_sps)->Kr,\
        (new_sps)->c1, (new_sps)->c2, (new_sps)->Umax, (new_sps)->Umin, 0.0L, 0.0L,    \
        0.0L, 1.0L,(DCL_PIDF64_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };             \
    }                                                                                  \
    new_pid;                                                                           \
})

//! \brief          Resets PID64 internal storage data with interrupt protection
//!
//! \param[in] pid  Pointer to the DCL_PID64 structure
//!
_DCL_CODE_ACCESS
void DCL_resetPIDF64(DCL_PIDF64 *pid)
{
    dcl_interrupt_t ints = DCL_disableInts();
    pid->d2 = pid->d3 = pid->i10 = 0.0L;
    pid->i14 = 1.0L;
    DCL_restoreInts(ints);
}

//! \brief          Loads PIDF64 tuning parameter from its SPS parameter
//!
//! \param[in] pid  Pointer to the active DCL_PID64 controller structure
//!
_DCL_CODE_ACCESS
void DCL_fupdatePIDF64(DCL_PIDF64 *pid)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    float64_t tau = (2.0L - pid->sps->c1 * p->css->t_sec) / (2.0L * pid->sps->c1);
    float64_t ec2 = pid->sps->c1 * (pid->css->t_sec - 2.0L * tau) / 2.0L;
    uint32_t err_code = dcl_none;
    err_code |= DCL_isValue(pid->sps->c2, ec2) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->sps->Umax > pid->sps->Umin) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->css->t_sec > 0.0L) ? dcl_none : dcl_param_range_err;
    err_code |= ((pid->sps->Kp > 0.0L) && (pid->sps->Ki > 0.0L) && (pid->sps->Kd > 0.0L) && (pid->sps->Kr > 0.0L)) ? dcl_none : dcl_param_range_err ;
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


//! \brief          Updates PID parameter from its SPS parameter with interrupt protection
//!
//! \param[in] pid  Pointer to the DCL_PID64 controller structure
//! \return         'true' if update is successful, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updatePIDF64(DCL_PIDF64 *pid)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    float64_t tau = (2.0L - pid->sps->c1 * pid->css->t_sec) / (2.0L * pid->sps->c1);
    float64_t ec2 = pid->sps->c1 * (pid->css->t_sec - 2.0L * tau) / 2.0L;
    uint32_t err_code = dcl_none;
    err_code |= DCL_isValue(pid->sps->c2, ec2) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->sps->Umax > pid->sps->Umin) ? dcl_none : dcl_param_invalid_err;
    err_code |= (pid->css->t_sec > 0.0L) ? dcl_none : dcl_param_range_err;
    err_code |= ((pid->sps->Kp > 0.0L) && (pid->sps->Ki > 0.0L) && (pid->sps->Kd > 0.0L) && (pid->sps->Kr > 0.0L)) ? dcl_none : dcl_param_range_err ;
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
//!                  If the pending status is set, the function will update PIDF64
//!                  parameter from its SPS parameter and clear the status flag on completion.
//!                  Note: Use DCL_setPendingStatus(pid) to set the pending status.
//!     
//! \param[in] pid   Pointer to the DCL_PIDF64 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_pendingUpdatePIDF64(DCL_PIDF64 *pid)
{
    if (DCL_getPendingStatus(pid) && DCL_updatePIDF64(pid))
    {
        DCL_clearPendingStatus(pid);
        return true;
    }
    return false;
}

//! \brief           Update SPS parameter with active param, userful when needing
//!                  to update only few active param from SPS and keep rest the same   
//!
//! \param[in] pid   Pointer to the active DCL_PIDF64 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updatePIDF64SPS(DCL_PIDF64 *pid)
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

//! \brief          Loads the derivative path filter shadow coefficients
//!                 Note: Sampling period pid->css->t_sec are used in the calculation
//!                 Note: new coefficients take effect when DCL_updatePID64() is called
//!
//! \param[in] pid  Pointer to the DCL_PID64 structure
//! \param[in] fc   The desired filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
void DCL_setPIDF64filterBW(DCL_PIDF64 *pid, float64_t fc)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code;
    err_code = ((fc >= 1.0L / (2.0L * pid->css->t_sec)) || (fc <= 0.0L)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float64_t t_sec = pid->css->t_sec;
    float64_t tau = 1.0L / (2.0L * CONST_PI_F64 * fc);
    pid->sps->c1 = 2.0L / (t_sec + (2.0L * tau));
    pid->sps->c2 = (t_sec - (2.0L * tau)) / (t_sec + (2.0L * tau));
}

//! \brief           Loads the PID64 derivative path filter active coefficients
//!                  Note: Sampling period pid->css->t_sec are used in the calculation
//!                  Note: new coefficients take effect immediately.  SPS &
//!                  CSS contents are unaffected.
//!
//! \param[in] pid   Pointer to the DCL_PID64 structure
//! \param[in] fc    The desired filter bandwidth in Hz
//! \param[in] t_sec The controller update rate in seconds
//!
_DCL_CODE_ACCESS
void DCL_setActivePIDF64filterBW(DCL_PIDF64 *pid, float64_t fc, float64_t t_sec)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code;
    err_code = ((fc >= 1.0L / (2.0L * t_sec)) || (fc <= 0.0L)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    float64_t tau = 1.0L / (2.0L * CONST_PI_F64 * fc);
    pid->c1 = 2.0L / (t_sec + (2.0L * tau));
    pid->c2 = (t_sec - (2.0L * tau)) / (t_sec + (2.0L * tau));
}

//! \brief          Returns the active derivative path filter bandwidth in Hz
//!                 Note: Sampling period pid->css->t_sec are used in the calculation
//!
//! \param[in] pid  Pointer to the DCL_PID64 structure
//! \return         The filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
float64_t DCL_getPIDF64filterBW(DCL_PIDF64 *pid)
{
    float64_t tau = ((2.0L - pid->c1 * pid->css->t_sec) / (2.0L * pid->c1));
    return(1.0L / (2.0L * CONST_PI_F64 * tau));
}

//! \brief          Executes an ideal form PID64 controller
//!
//! \param[in] pid  Pointer to the DCL_PID64 structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \param[in] lk   External output clamp flag
//! \return         The control effort
//!
_DCL_CODE_ACCESS
float64_t DCL_runPIDF64Series(DCL_PIDF64 *pid, float64_t rk, float64_t yk, float64_t lk)
{
    float64_t v1, v4, v5, v8, v9, v10, v12;

    v5 = (pid->Kr * rk) - yk;
    v8 = ((rk - yk) * pid->Ki * pid->Kp * pid->i14) + pid->i10;
    pid->i10 = v8;
    v1 = yk * pid->Kd * pid->c1;
    v4 = v1 - pid->d2 - pid->d3;
    pid->d2 = v1;
    pid->d3 = v4 * pid->c2;
    v9 = ((v5 - v4) * pid->Kp) + v8;
    v10 = DCL_runSat(v9, pid->Umax, pid->Umin);
    v12 = (v10 == v9) ? 1.0 : 0.0;
    pid->i14 = v12 * lk;

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = v5;
#endif

    return(v10);
}

//! \brief          Executes an parallel form PID64 controller
//!
//! \param[in] pid  Pointer to the DCL_PID64 structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \param[in] lk   External output clamp flag
//! \return         The control effort
//!
_DCL_CODE_ACCESS
float64_t DCL_runPIDF64Parallel(DCL_PIDF64 *pid, float64_t rk, float64_t yk, float64_t lk)
{
    float64_t v1, v4, v5, v6, v8, v9, v10, v12;

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

#endif // _DCL_PIDF64_H_
