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

#ifndef _DCL_NLPID_H_
#define _DCL_NLPID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_nlpid.h
 *  \brief      Contains 32-bit non-linear PID controller with its related structures and functions
 */

#include <math.h>
#include "../dcl_common.h"

//! \brief          Sets the lower bound on the linear region semi-width
//!
#define DCL_DELTA_MIN   1.0e-04f

//! \brief          Sets an upper bound on allowable controller gain, and
//!                 therefore fixes the minimum linear region semi-width
//!
#define DCL_GAMMA_MAX   100

//! \brief          Defines the shadow DCL_NLPID controller structure
//!
typedef struct dcl_nlpid_sps 
{
    float32_t Kp;       //!< Linear proportional gain
    float32_t Ki;       //!< Linear integral gain
    float32_t Kd;       //!< Linear derivative gain
    float32_t alpha_p;  //!< P path non-linear exponent, default is 1
    float32_t alpha_i;  //!< I path non-linear exponent, default is 1
    float32_t alpha_d;  //!< D path non-linear exponent, default is 1
    float32_t delta_p;  //!< P path linearized range, default is 0.1
    float32_t delta_i;  //!< I path linearized range, default is 0.1
    float32_t delta_d;  //!< D path linearized range, default is 0.1
    float32_t gamma_p;  //!< P path gain limit, default is 1
    float32_t gamma_i;  //!< I path gain limit, default is 1
    float32_t gamma_d;  //!< D path gain limit, default is 1
    float32_t c1;       //!< D path low-pass filter coefficient 1, default is 1
    float32_t c2;       //!< D path low-pass filter coefficient 2, default is 0
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
} DCL_NLPID_SPS;

//! \brief          Defines default values to initialize the DCL_NLPID structure
//!
#define NLPID_SPS_DEFAULTS { 1.0f, 0.0f, 0.0f, \
                             1.0f, 1.0f, 1.0f, \
                             0.1f, 0.1f, 0.1f, \
                             1.0f, 1.0f, 1.0f, \
                             1.0f, 0.0f, \
                             1.0f, -1.0f }

//! \brief          Defines the active DCL_NLPID controller structure
//!
typedef _DCL_VOLATILE struct dcl_nlpid 
{
    /* controller parameter */
    float32_t Kp;       //!< Linear proportional gain
    float32_t Ki;       //!< Linear integral gain
    float32_t Kd;       //!< Linear derivative gain
    float32_t alpha_p;  //!< P path non-linear exponent, default is 1
    float32_t alpha_i;  //!< I path non-linear exponent, default is 1
    float32_t alpha_d;  //!< D path non-linear exponent, default is 1
    float32_t delta_p;  //!< P path linearized range, default is 0.1
    float32_t delta_i;  //!< I path linearized range, default is 0.1
    float32_t delta_d;  //!< D path linearized range, default is 0.1
    float32_t gamma_p;  //!< P path gain limit, default is 1
    float32_t gamma_i;  //!< I path gain limit, default is 1
    float32_t gamma_d;  //!< D path gain limit, default is 1
    float32_t c1;       //!< D path low-pass filter coefficient 1, default is 1
    float32_t c2;       //!< D path low-pass filter coefficient 2, default is 0
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
    
    /* internal storage */
    float32_t d2;       //!< D path low-pass filter storage (Kd * c1)
    float32_t d3;       //!< D path low-pass filter storage (c2)
    float32_t i7;       //!< I path feedback storage
    float32_t i16;      //!< Saturation multiplier, ranges between 1*lk ~ 0, where 0 means fully saturated

    float32_t i18;      //!< No longer needed

    /* miscellaneous */
    DCL_NLPID_SPS *sps; //!< Pointer to shadow parameter structure
    DCL_CSS *css;       //!< Pointer to controller support structure
} DCL_NLPID, *NLPID_Handle;

//! \brief          Defines default values to initialize the DCL_NLPID structure
//!
#define NLPID_DEFAULTS { 1.0f, 0.0f, 0.0f, \
                        1.0f, 1.0f, 1.0f, \
                        0.1f, 0.1f, 0.1f, \
                        1.0f, 1.0f, 1.0f, \
                        1.0f, 0.0f, \
                        0.0f, 0.0f, \
    0.0f, 1.0f, 0.0f, 1.0f, 0.0f, \
    &(DCL_NLPID_SPS)NLPID_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Resets NLPID internal storage data with interrupt protection
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//!
_DCL_CODE_ACCESS
void DCL_resetNLPID(DCL_NLPID *pid)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pid->d2 = pid->d3 = pid->i7 = 0.0f;
    pid->i16 = 1.0f;
    DCL_restoreInts(ints);
}

//! \brief            Loads NLPID tuning parameter from its SPS parameter without interrupt protection
//!
//! \param[in] pid    Pointer to the active DCL_NLPID controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdateNLPID(DCL_NLPID *pid)
{
    pid->Kp = pid->sps->Kp;
    pid->Ki = pid->sps->Ki;
    pid->Kd = pid->sps->Kd;
    pid->alpha_p = pid->sps->alpha_p;
    pid->alpha_i = pid->sps->alpha_i;
    pid->alpha_d = pid->sps->alpha_d;
    pid->delta_p = pid->sps->delta_p;
    pid->delta_i = pid->sps->delta_i;
    pid->delta_d = pid->sps->delta_d;
    pid->gamma_p = pid->sps->gamma_p;
    pid->gamma_i = pid->sps->gamma_i;
    pid->gamma_d = pid->sps->gamma_d;
    pid->c1 = pid->sps->c1;
    pid->c2 = pid->sps->c2;
    pid->Umax = pid->sps->Umax;
    pid->Umin = pid->sps->Umin;    
}

//! \brief           Loads PID tuning parameter from its SPS parameter with interrupt protection
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//!
_DCL_CODE_ACCESS
void DCL_updateNLPIDNoCheck(DCL_NLPID *pid)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    float32_t tau = (2.0f - pid->sps->c1 * pid->css->T) / (2.0f * pid->sps->c1);
    float32_t ec2 = pid->sps->c1 * (pid->css->T - 2.0f * tau) / 2.0f;
    uint32_t err_code = dcl_none;
    err_code |= ((pid->sps->c2 < (ec2 - DCL_FPU32_TOL)) || (pid->sps->c2 > (ec2 + DCL_FPU32_TOL))) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pid->sps->delta_p < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->delta_i < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->delta_d < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->Umax <= pid->sps->Umin) ? dcl_param_invalid_err : dcl_none;
    err_code |= (pid->css->T <= 0.0f) ? dcl_param_range_err : dcl_none;
    err_code |= ((pid->sps->Kp < 0.0f) || (pid->sps->Ki < 0.0f) || (pid->sps->Kd < 0.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    pid->Kp = pid->sps->Kp;
    pid->Ki = pid->sps->Ki;
    pid->Kd = pid->sps->Kd;
    pid->alpha_p = pid->sps->alpha_p;
    pid->alpha_i = pid->sps->alpha_i;
    pid->alpha_d = pid->sps->alpha_d;
    pid->delta_p = pid->sps->delta_p;
    pid->delta_i = pid->sps->delta_i;
    pid->delta_d = pid->sps->delta_d;
    pid->gamma_p = pid->sps->gamma_p;
    pid->gamma_i = pid->sps->gamma_i;
    pid->gamma_d = pid->sps->gamma_d;
    pid->c1 = pid->sps->c1;
    pid->c2 = pid->sps->c2;
    pid->Umax = pid->sps->Umax;
    pid->Umin = pid->sps->Umin;  
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update NLPID
//!                  parameter from its SPS parameter and clear the status flag on completion.
//! \note            Note: Use DCL_getUpdateStatus(pid) to set the update status.
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//! \return         'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updateNLPID(DCL_NLPID *pid)
{
    if (DCL_getUpdateStatus(pid))
    {
        DCL_updateNLPIDNoCheck(pid);
        DCL_clearUpdateStatus(pid);
        return true;
    }
    return false;
}

//! \brief           Loads the shadow derivative LP filter coefficients
//! \note            Note: active coefficients unchanged DCL_updateNLPID() called
//!
//! \param[in] pid   Pointer to the DCL_NLPID structure
//! \param[in] fc    The desired filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
void DCL_setNLPIDfilterBW(DCL_NLPID *pid, float32_t fc)
{
    float32_t tau = 1.0f / (2.0f * CONST_PI * fc);

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((fc >= 1.0f / (2.0f * pid->css->T)) || (fc <= 0.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    pid->sps->c1 = 2.0f / (pid->css->T + 2.0f * tau);
    pid->sps->c2 = (pid->css->T - 2.0f * tau) / (pid->css->T + 2.0f * tau);
}

//! \brief          Loads the NLPID derivative path filter active coefficients
//! \note           Note: new coefficients take effect immediately.  SPS &
//!                 CSS contents are unaffected.
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//! \param[in] fc   The desired filter bandwidth in Hz
//! \param[in] T    The controller update rate in seconds
//!
_DCL_CODE_ACCESS
void DCL_setActiveNLPIDfilterBW(DCL_NLPID *pid, float32_t fc, float32_t T)
{
    float32_t tau;

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((fc >= 1.0f / (2.0f * T)) || (fc <= 0.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    tau = 1.0f / (2.0f * CONST_PI * fc);
    pid->c1 = 2.0f / (T + 2.0f * tau);
    pid->c2 = (T - 2.0f * tau) / (T + 2.0f * tau);
}

//! \brief          Returns the derivative LP filter bandwidth in Hz
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//! \return         The filter bandwidth in Hz
//!
_DCL_CODE_ACCESS
float32_t DCL_getNLPIDfilterBW(DCL_NLPID *pid)
{
    float32_t tau = ((2.0f - pid->c1 * pid->css->T) / (2.0f * pid->c1));
    return(1.0f / (2.0f * CONST_PI * tau));
}

//! \brief              Returns the linearized region gain for specified (alpha,delta)
//!
//! \param[in] alpha    The non-linear gain
//! \param[in] delta    The linear region semi-width
//! \return             The linear region gain (gamma)
//!
_DCL_CODE_ACCESS
float32_t DCL_getNLPIDgamma(float32_t alpha, float32_t delta)
{
    return((float32_t) powf(delta, (alpha - 1.0f)));
}

//! \brief              Returns the semi-width of the linear gain region for specified (alpha,gamma)
//!
//! \param[in] alpha    The non-linear gain
//! \param[in] gamma    The linear region gain
//! \return             The linear region semi-width
//!
_DCL_CODE_ACCESS
float32_t DCL_getNLPIDdelta(float32_t alpha, float32_t gamma)
{
    return((float32_t) powf(gamma, (1.0f / (alpha - 1.0f))));
}

//! \brief          Computes the linearized gains for each path
//!                 Note: active coefficients not update DCL_updateNLPID() called
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//!
_DCL_CODE_ACCESS
void DCL_setNLPIDgamma(DCL_NLPID *pid)
{
    float32_t xP = (float32_t) powf(pid->sps->delta_p, (pid->sps->alpha_p - 1.0f));
    float32_t xI = (float32_t) powf(pid->sps->delta_i, (pid->sps->alpha_i - 1.0f));
    float32_t xD = (float32_t) powf(pid->sps->delta_d, (pid->sps->alpha_d - 1.0f));


#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((pid->sps->delta_p > 1.0f) || (pid->sps->delta_p > 1.0f) || (pid->sps->delta_p > 1.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->delta_p < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->delta_i < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->sps->delta_d < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= ((xP > DCL_GAMMA_MAX) || (xI > DCL_GAMMA_MAX) || (xD > DCL_GAMMA_MAX)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    pid->sps->gamma_p = xP;
    pid->sps->gamma_i = xI;
    pid->sps->gamma_d = xD;
}

//! \brief          Computes the linearized gains for each path and loads the
//!                 parameters in the active NLPID structure
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//!
_DCL_CODE_ACCESS
void DCL_setActiveNLPIDgamma(DCL_NLPID *pid)
{
    float32_t xP = (float32_t) powf(pid->delta_p, (pid->alpha_p - 1.0f));
    float32_t xI = (float32_t) powf(pid->delta_i, (pid->alpha_i - 1.0f));
    float32_t xD = (float32_t) powf(pid->delta_d, (pid->alpha_d - 1.0f));

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((pid->delta_p > 1.0f) || (pid->delta_p > 1.0f) || (pid->delta_p > 1.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->delta_p < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->delta_i < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= (pid->delta_d < DCL_DELTA_MIN) ? dcl_param_range_err : dcl_none;
    err_code |= ((xP > DCL_GAMMA_MAX) || (xI > DCL_GAMMA_MAX) || (xD > DCL_GAMMA_MAX)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    pid->gamma_p = xP;
    pid->gamma_i = xI;
    pid->gamma_d = xD;
}

//! \brief          Executes a parallel form non-linear PID controller
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \param[in] lk   External output clamp flag
//! \return         The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runNLPIDParallel(DCL_NLPID *pid, float32_t rk, float32_t yk, float32_t lk)
{
    float32_t v1, v2, v3, v4, v5, v8, v9, v10, v12, v13, v14, v15;

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (DCL_getControllerStatus(pid)) ? dcl_controller_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
    DCL_setControllerStatus(pid);
#endif

    // pre-conditioning block
    v1 = (rk - yk) * 0.5f;
    v2 = (v1 < 0.0f) ? -1.0f : 1.0f;
    v3 = fabsf(v1);

#ifdef DCL_ERROR_HANDLING_ENABLED
    err_code |= ((pid->alpha_p >= 2.0f) || (pid->alpha_p <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= ((pid->alpha_i >= 2.0f) || (pid->alpha_i <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= ((pid->alpha_d >= 2.0f) || (pid->alpha_d <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= (v3 > 1.0f) ? dcl_input_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    // non-linear modules
    v4 = ((v3 > pid->delta_p) ? (v2 * (float32_t) powf(v3, pid->alpha_p)) : (v1 * pid->gamma_p));
    v5 = ((v3 > pid->delta_i) ? (v2 * (float32_t) powf(v3, pid->alpha_i)) : (v1 * pid->gamma_i));
    v9 = ((v3 > pid->delta_d) ? (v2 * (float32_t) powf(v3, pid->alpha_d)) : (v1 * pid->gamma_d));

    // integral path
    v8 = (v5 * pid->Kp * pid->Ki * pid->i16) + pid->i7;
    pid->i7 = v8;

    // derivative path
    v10 = v9 * pid->Kd * pid->c1;
    v12 = v10 - pid->d2 - pid->d3;
    pid->d2 = v10;
    pid->d3 = v12 * pid->c2;

    // output sum & clamp
    v13 = (pid->Kp * (v4 + v12)) + v8;
    v14 = DCL_runSat(v13, pid->Umax, pid->Umin);
    v15 = (v14 == v13) ? 1.0f : 0.0f;
    pid->i16 = v15 * lk;

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = v14;
#endif

#ifdef DCL_ERROR_HANDLING_ENABLED
    DCL_clearControllerStatus(pid);
#endif

    return(v14);
}

//! \brief          Executes a series form non-linear PID controller
//!
//! \param[in] pid  Pointer to the DCL_NLPID structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \param[in] lk   External output clamp flag
//! \return         The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runNLPIDSeries(DCL_NLPID *pid, float32_t rk, float32_t yk, float32_t lk)
{
    float32_t v1, v2, vd2, v3, vd3, v4, v5, v6, v8, v9, v12, v15, v16, v17;

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (DCL_getControllerStatus(pid)) ? dcl_controller_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
    DCL_setControllerStatus(pid);
#endif

    // pre-conditioning block for P & I
    v1 = (rk - yk) * 0.5f;
    v2 = (v1 < 0.0f) ? -1.0f : 1.0f;
    v3 = fabsf(v1);

#ifdef DCL_ERROR_HANDLING_ENABLED
    err_code |= ((pid->alpha_p >= 2.0f) || (pid->alpha_p <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= ((pid->alpha_i >= 2.0f) || (pid->alpha_i <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= (v3 > 1.0f) ? dcl_input_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    // P & I non-linear modules
    v4 = ((v3 > pid->delta_p) ? (v2 * (float32_t) powf(v3, pid->alpha_p)) : (v1 * pid->gamma_p));
    v5 = ((v3 > pid->delta_i) ? (v2 * (float32_t) powf(v3, pid->alpha_i)) : (v1 * pid->gamma_i));

    // D path non-linear block
    vd2 = (yk < 0.0f) ? -1.0f : 1.0f;
    vd3 = fabsf(yk);

#ifdef DCL_ERROR_HANDLING_ENABLED
    err_code |= ((pid->alpha_d >= 2.0f) || (pid->alpha_d <= 0.0f)) ? dcl_param_range_err : dcl_none;
    err_code |= (vd3 > 1.0f) ? dcl_input_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(pid,err_code);
        DCL_getErrorInfo(pid);
        DCL_runErrorHandler(pid);
    }
#endif

    v6 = ((vd3 > pid->delta_d) ? (vd2 * (float32_t) powf(vd3, pid->alpha_d)) : (yk * pid->gamma_d));

    // integral path
    v8 = (v5 * pid->Kp * pid->Ki * pid->i16) + pid->i7;
    pid->i7 = v8;

    // derivative path
    v15 = v6 * pid->Kd * pid->c1;
    v16 = v15 - pid->d2 - pid->d3;
    pid->d2 = v15;
    pid->d3 = v16 * pid->c2;

    // output sum & clamp
    v9 = (pid->Kp * (v4 - v16)) + v8;
    v17 = DCL_runSat(v9, pid->Umax, pid->Umin);
    v12 = (v17 == v9) ? 1.0f : 0.0f;
    pid->i16 = v12 * lk;

#ifdef DCL_TESTPOINTS_ENABLED
    pid->css->tpt = v17;
#endif

#ifdef DCL_ERROR_HANDLING_ENABLED
    DCL_clearControllerStatus(pid);
#endif

    return(v17);
}

//--- Basic non-linear function ----------------------------------------------

//! \brief              Executes a basic non-linear control function
//!
//! \param[in] x        The input variable
//! \param[in] alpha    The non-linear exponent
//! \param[in] delta    The linear region semi-width
//! \return             The non-linear output
//!
_DCL_CRIT_ACCESS
float32_t DCL_runNLF(float32_t x, float32_t alpha, float32_t delta)
{
    float32_t v2 = (x < 0.0f) ? -1.0f : 1.0f;
    float32_t v3 = fabsf(x);

    return((v3 > delta) ? (v2 * (float32_t) powf(v3, alpha)) : (x * powf(delta, (alpha - 1.0f))));
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_NLPID_H_
