/*
 * Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef _DCL_DF23_H_
#define _DCL_DF23_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *
 *  \file       dcl_df23.h
 *  \brief      Contains direct form 2 3rd order DF23 compensator
 *              with its related structures and functions
 */               

#include "../dcl_common.h"

//--- Direct Form 2 - 3rd order ----------------------------------------------

//! \brief          Defines DCL_DF23 shadow parameter set
//!                 used for updating compensator parameter
//!
typedef struct dcl_df23_sps
{
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t b2;   //!< pos. coefficient to e(k-2)
    float32_t b3;   //!< pos. coefficient to e(k-3)
    float32_t a1;   //!< neg. coefficient to u(k-1)
    float32_t a2;   //!< neg. coefficient to u(k-2)
    float32_t a3;   //!< neg. coefficient to u(k-3)
} DCL_DF23_SPS;

#define DF23_SPS_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

//! \brief          DCL_DF23 object for storing df23 specific parameters
//!
typedef _DCL_VOLATILE struct dcl_df23
{
    /* compensator parameter */
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t b2;   //!< pos. coefficient to e(k-2)
    float32_t b3;   //!< pos. coefficient to e(k-3)
    float32_t a1;   //!< neg. coefficient to u(k-1)
    float32_t a2;   //!< neg. coefficient to u(k-2)
    float32_t a3;   //!< neg. coefficient to u(k-3)

    /* internal storage */
    float32_t x1;   //!< x1 = b1*e(k-1) - a1*u(k-1) + x2
    float32_t x2;   //!< x2 = b2*e(k-2) - a2*u(k-2) + x3
    float32_t x3;   //!< x3 = b3*e(k-3) - a3*u(k-3)

    /* miscellaneous */
    DCL_DF23_SPS *sps; //!< updates compensator parameter
    DCL_CSS *css;      //!< configuration & debugging
} DCL_DF23;

//! \brief          Defines default values to initialize DCL_DF23
//!
#define DF23_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
        &(DCL_DF23_SPS)DF23_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_DF23
//!                 Example: DCL_DF23 df23_ctrl = { 
//!                                                 .b0 = 1.0f,
//!                                                 .b1 = 0.0f,
//!                                                 ...
//!                                                 .a3 = 0.0f,
//!                                                 DF23_INT_DEFAULTS
//!                                               };
#define DF23_INT_DEFAULTS .x1=0.0f, .x2=0.0f, .x3=0.0f, \
.sps=&(DCL_DF23_SPS)DF23_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS 

//! \brief          Initialize DCL_DF23 struct with default parameters
//!                 Example: DCL_DF23* df23_ctrl = DCL_initDF23();
//!
//! \return         A DCL_DF23* pointer
//!
#define DCL_initDF23() &(DCL_DF23)DF23_DEFAULTS

//! \brief          Initialize DCL_DF23 struct with input compensator parameters
//!                 Example: DCL_DF23* DF23_ctrl = DCL_initDF23asParam(1.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
//! \note           Note: input parameter needs to be in the same order as listed in DF23_SPS struct.
//!
//! \return         A DCL_DF23* pointer
//!
#define DCL_initDF23asParam(_b0,_b1,_b2,_b3,_a1,_a2,_a3) &(DCL_DF23){ .b0=_b0, .b1=_b1, \
                        .b2=_b2, .b3=_b3, .a1=_a1, .a2=_a2, .a3=_a3, DF23_INT_DEFAULTS }

//! \brief                  Initialize DCL_DF23 struct with sps parameters
//!                         Example: DCL_DF23_SPS df_sps = { .b0 = , .b1 = , ...}; //initial parameter
//!                                  DCL_DF23 df_ctrl;
//!                                  DCL_initDF23asSPS(&df_ctrl,&df_sps);
//!
//! \param[in] df_ptr       DCL_DF23* pointer that needs to be initialized
//! \param[in] sps_ptr      DCL_DF23_SPS* pointer with assigned parameters
//! \return                 Returns DCL_DF23* with set sps parameters, default parameter will be used
//!                         if sps_ptr is not specified   
//!
#define DCL_initDF23asSPS(df_ptr,sps_ptr)                                          \
({                                                                                 \
    DCL_DF23* new_df = (df_ptr) ? df_ptr : DCL_initDF23();                         \
    DCL_DF23_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_DF23_SPS)DF23_SPS_DEFAULTS;\
    if(sps_ptr)                                                                    \
    {                                                                              \
        *new_df = (DCL_DF23){ (new_sps)->b0, (new_sps)->b1, (new_sps)->b2,         \
        (new_sps)->b3,(new_sps)->a1, (new_sps)->a2, (new_sps)->a3, 0.0f,           \
        0.0f, 0.0f, (DCL_DF23_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };          \
    }                                                                              \
    new_df;                                                                        \
})

//! \brief           Resets DF23 internal storage data with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//!
_DCL_CODE_ACCESS
void DCL_resetDF23(DCL_DF23 *df)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    df->x1 = df->x2 = df->x3 = 0.0f;
    DCL_restoreInts(ints);
}

//! \brief           Loads DF23 tuning parameter from its SPS parameter without interrupt protection
//!
//! \param[in] df    Pointer to the active DCL_DF23 controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdateDF23(DCL_DF23 *df)
{
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->b2 = df->sps->b2;
    df->b3 = df->sps->b3;
    df->a1 = df->sps->a1;
    df->a2 = df->sps->a2;
    df->a3 = df->sps->a3;
}

//! \brief           Updates DF23 parameter from its SPS parameter with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updateDF23NoCheck(DCL_DF23 *df)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->b2 = df->sps->b2;
    df->b3 = df->sps->b3;
    df->a1 = df->sps->a1;
    df->a2 = df->sps->a2;
    df->a3 = df->sps->a3;
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update DF23 
//!                  parameter from its SPS parameter and clear the status flag on completion.
//! \note            Note: Use DCL_setUpdateStatus(df) to set the update status.
//!     
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updateDF23(DCL_DF23 *df)
{
    if (DCL_setUpdateStatus(df))
    {
        DCL_updateDF23NoCheck(df);
        DCL_clearUpdateStatus(df);
        return true;
    }
    return false;
}

//! \brief           Determines stability of the shadow compensator
//!
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//! \return          'true' if all poles have magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStableDF23(DCL_DF23 *df)
{
    return(DCL_isStablePn3(1.0f, df->sps->a1, df->sps->a2, df->sps->a3));
}

//! \brief            Loads the DF23 shadow coefficients from a ZPK3 description
//! \note             Note: Sampling period df->css->T are used in the calculation.
//!                   New settings take effect after DCL_updateDF23().
//!
//! \param[in] df     Pointer to the DCL_DF23 controller structure
//! \param[in] zpk    Pointer to the DCL_ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadDF23asZPK(DCL_DF23 *df, DCL_ZPK3 *zpk)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= DCL_isZero(cimagf(zpk->z1) + cimagf(zpk->z2) + cimagf(zpk->z3)) ? dcl_none : dcl_param_invalid_err;
    err_code |= DCL_isZero(cimagf(zpk->p1) + cimagf(zpk->p2) + cimagf(zpk->p3)) ? dcl_none : dcl_param_invalid_err;
    if (err_code)
    {
        DCL_setError(df,err_code);
        DCL_getErrorInfo(df);
        DCL_runErrorHandler(df);
    }
#endif

    float32_t beta2 = -(float32_t) crealf(zpk->z1 + zpk->z2 + zpk->z3);
    float32_t beta1 = (float32_t) crealf((zpk->z1 * zpk->z2) + (zpk->z2 * zpk->z3) + (zpk->z1 * zpk->z3));
    float32_t beta0 = -(float32_t) crealf(zpk->z1 * zpk->z2 * zpk->z3);

    float32_t alpha2 = -(float32_t) crealf(zpk->p1 + zpk->p2 + zpk->p3);
    float32_t alpha1 = (float32_t) crealf((zpk->p1 * zpk->p2) + (zpk->p2 * zpk->p3) + (zpk->p1 * zpk->p3));
    float32_t alpha0 = -(float32_t) crealf(zpk->p1 * zpk->p2 * zpk->p3);

    float32_t T = df->css->T;
    float32_t a0p = 8.0f + (alpha2 * 4.0f * T) + (alpha1 * 2.0f * T * T) + (alpha0 * T * T * T);

    df->sps->b0 = zpk->K * (8.0f + (beta2 * 4.0f * T) + (beta1 * 2.0f * T * T) + (beta0 * T * T * T)) / a0p;
    df->sps->b1 = zpk->K * (-24.0f - (beta2 * 4.0f * T) + (beta1 * 2.0f * T * T) + (3.0f * beta0 * T * T * T)) / a0p;
    df->sps->b2 = zpk->K * (24.0f - (beta2 * 4.0f * T) - (beta1 * 2.0f * T * T) + (3.0f * beta0 * T * T * T)) / a0p;
    df->sps->b3 = zpk->K * (-8.0f + (beta2 * 4.0f * T) - (beta1 * 2.0f * T * T) + (beta0 * T * T * T)) / a0p;

    df->sps->a1 = (-24.0f - (alpha2 * 4.0f * T) + (alpha1 * 2.0f * T * T) + (3.0f * alpha0 * T * T * T)) / a0p;
    df->sps->a2 = (24.0f - (alpha2 * 4.0f * T) - (alpha1 * 2.0f * T * T) + (3.0f * alpha0 * T * T * T)) / a0p;
    df->sps->a3 = (-8.0f + (alpha2 * 4.0f * T) - (alpha1 * 2.0f * T * T) + (alpha0 * T * T * T)) / a0p;
}

//! \brief           Executes a 3rd order Direct Form 2 controller
//!                  
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF23(DCL_DF23 *df, float32_t ek)
{
    float32_t v7 = (ek * df->b0) + df->x1;
    df->x1 = (ek * df->b1) + df->x2 - (v7 * df->a1);
    df->x2 = (ek * df->b2) + df->x3 - (v7 * df->a2);
    df->x3 = (ek * df->b3) - (v7 * df->a3);
    return(v7);
}

//! \brief           Immediate computation to obtain DF23 servo error 
//!                  without updating the controller
//!
//! \param[in] df    Pointer to the DCL_DF23 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF23PartialCompute(DCL_DF23 *df, float32_t ek)
{
    return((ek * df->b0) + df->x1);
}

//! \brief          Update DF22 controller based on pre-computed control effort
//!                 
//! \param[in] df   Pointer to the DCL_DF23 controller structure
//! \param[in] ek   The servo error
//! \param[in] uk   The controller output in the previous sample interval
//!
_DCL_CRIT_ACCESS
void DCL_runDF23PartialUpdate(DCL_DF23 *df, float32_t ek, float32_t uk)
{
    df->x1 = (ek * df->b1) + df->x2 - (uk * df->a1);
    df->x2 = (ek * df->b2) + df->x3 - (uk * df->a2);
    df->x3 = (ek * df->b3) - (uk * df->a3);
}

//! \brief            Executes a 3rd order Direct Form 2 controller with clamp
//!                   
//! \param[in] df     Pointer to the DCL_DF23 controller structure
//! \param[in] ek     The servo error
//! \param[in] Umax   Upper saturation limit
//! \param[in] Umin   Lower saturation limit  
//! \return    uk     The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF23Clamp(DCL_DF23 *df, float32_t ek, float32_t Umax, float32_t Umin)
{
    float32_t uk = DCL_runDF23PartialCompute(df, ek);
    bool is_clamped = DCL_runClamp(&uk, Umax, Umin);
    if(!is_clamped) DCL_runDF23PartialUpdate(df, ek, uk);
    return(uk);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_DF23_H_
