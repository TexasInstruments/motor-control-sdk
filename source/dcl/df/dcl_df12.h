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
 
#ifndef _DCL_DF12_H_
#define _DCL_DF12_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *
 *  \file       dcl_df12.h
 *  \brief      Contains direct form 1 2nd order DF12 compensator
 *              with its related structures and functions 
 *
 */       

#include "../dcl_common.h"

//--- Direct Form 1 - 2nd order ----------------------------------------------

//! \brief          Defines the DCL_DF12 shadow parameter set
//!                 used for updating compensator parameter
//!
typedef struct dcl_df12_sps {
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t b2;   //!< pos. coefficient to e(k-2)
    float32_t a1;   //!< neg. coefficient to u(k-1)
    float32_t a2;   //!< neg. coefficient to u(k-2)
} DCL_DF12_SPS;

#define DF12_SPS_DEFAULTS { 0.25f, 0.25f, 0.25f, 0.0f, 0.0f }

//! \brief          DCL_DF12 object for storing df12 specific parameters
//!
typedef _DCL_VOLATILE struct dcl_df12 
{
    /* compensator parameter */
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t b2;   //!< pos. coefficient to e(k-2)
    float32_t a1;   //!< neg. coefficient to u(k-1)
    float32_t a2;   //!< neg. coefficient to u(k-2)

    /* internal storage */
    float32_t d1;   //!< e(k-1)
    float32_t d2;   //!< e(k-2)
    float32_t d3;   //!< u(k-1)
    float32_t d4;   //!< u(k-2)

    /* miscellaneous */
    DCL_DF12_SPS *sps; //!< updates compensator parameter
    DCL_CSS *css;      //!< configuration & debugging
} DCL_DF12, *DF12_Handle;

//! \brief          Defines default values to initialize DCL_DF12
//!
#define DF12_DEFAULTS { 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,\
            &(DCL_DF12_SPS)DF12_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_DF12
//!                 Example: DCL_DF12 df12_ctrl = { 
//!                                                 .b0 = 0.25f,
//!                                                 .b1 = 0.25f,
//!                                                 ...
//!                                                 .a2 = 0.0f,
//!                                                 DF12_INT_DEFAULTS
//!                                               };
#define DF12_INT_DEFAULTS .d1=0.0f, .d2=0.0f, .d3=0.0f, .d4=0.0f, \
                          .sps=&(DCL_DF12_SPS)DF12_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS 

//! \brief          Initialize DCL_DF12 struct with default parameters
//!                 Example: DCL_DF12* df12_ctrl = DCL_initDF12();
//!
//! \return         A DCL_DF12* pointer
//!
#define DCL_initDF12() &(DCL_DF12)DF12_DEFAULTS

//! \brief          Initialize DCL_DF12 struct with input compensator parameters
//!                 Example: DCL_DF12* DF12_ctrl = DCL_initDF12asParam(0.25f,0.25f,0.25f,0.0f,0.0f);
//! \note           Note: input parameter needs to be in the same order as listed in DF12_SPS struct
//!
//! \return         A DCL_DF12* pointer
//!
#define DCL_initDF12asParam(_b0,_b1,_b2,_a1,_a2) &(DCL_DF12){ .b0=_b0, .b1=_b1, \
                        .b2=_b2, .a1=_a1, .a2=_a2, DF12_INT_DEFAULTS }

//! \brief                  Initialize DCL_DF12 struct with sps parameters
//!                         Example: DCL_DF12_SPS df_sps = { .b0 = , .b1 = , ...}; //initial parameter
//!                                  DCL_DF12 df_ctrl;
//!                                  DCL_initDF12asSPS(&df_ctrl,&df_sps);
//!
//! \param[in] df_ptr       DCL_DF12* pointer that needs to be initialized
//! \param[in] sps_ptr      DCL_DF12_SPS* pointer with assigned parameters
//! \return    DCL_DF12*    Returns DCL_DF12* with set sps parameters, default parameter will be used
//!                         if sps_ptr is not specified   
//!
#define DCL_initDF12asSPS(df_ptr,sps_ptr)                                                  \
({                                                                                         \
    DCL_DF12* new_df = (df_ptr) ? df_ptr : DCL_initDF12();                                 \
    DCL_DF12_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_DF12_SPS)DF12_SPS_DEFAULTS;        \
    if(sps_ptr)                                                                            \
    {                                                                                      \
        *new_df =(DCL_DF12){ (new_sps)->b0, (new_sps)->b1, (new_sps)->b2,                  \
        (new_sps)->a1, (new_sps)->a2, 0.0f, 0.0f, 0.0f, 0.0f,                              \
        (DCL_DF12_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };                              \
    }                                                                                      \
    new_df;                                                                                \
})

//! \brief           Resets DF12 internal storage data with interrupt protection
//!                  Implemented as inline C function
//!
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//!
_DCL_CODE_ACCESS
void DCL_resetDF12(DCL_DF12 *df)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    df->d1 = df->d2 = df->d3 = df->d4 = 0.0f;
    DCL_restoreInts(ints);
}

//! \brief           Loads DF12 tuning parameter from its SPS parameter without interrupt protection
//!
//! \param[in] df    Pointer to the active DCL_DF12 controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdateDF12(DCL_DF12 *df)
{
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->b2 = df->sps->b2;
    df->a1 = df->sps->a1;
    df->a2 = df->sps->a2;   
}

//! \brief           Loads DF12 tuning parameter from its SPS parameter with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updateDF12NoCheck(DCL_DF12 *df)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->b2 = df->sps->b2;
    df->a1 = df->sps->a1;
    df->a2 = df->sps->a2;
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update DF12
//!                  parameter from its SPS parameter and clear the status flag on completion..
//! \note            Note: Use DCL_setUpdateStatus(df) to set the update status.
//!     
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updateDF12(DCL_DF12 *df)
{
    if (DCL_getUpdateStatus(df))
    {
        DCL_updateDF12NoCheck(df);
        DCL_clearUpdateStatus(df);
        return true;
    }
    return false;
}

//! \brief           Determines stability of the shadow compensator
//!
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//! \return          'true' if all poles have magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStableDF12(DCL_DF12 *df)
{
    return(DCL_isStablePn2(1.0f, df->sps->a1, df->sps->a2));
}

//! \brief            Loads the DF12 shadow coefficients from a ZPK3 description
//! \note             Note: Sampling period df->css->T are used in the calculation.
//!                   New settings take effect after DCL_updateDF12().
//!                   Only z1, z2, p1 & p2 are considered, z3 & p3 are ignored.
//!
//! \param[in] df     Pointer to the DCL_DF12 controller structure
//! \param[in] zpk    Pointer to the DCL_ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadDF12asZPK(DCL_DF12 *df, DCL_ZPK3 *zpk)
{

#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= DCL_isZero(cimagf(zpk->z1) + cimagf(zpk->z2)) ? dcl_none : dcl_param_invalid_err;
    err_code |= DCL_isZero(cimagf(zpk->p1) + cimagf(zpk->p2)) ? dcl_none : dcl_param_invalid_err;
    if (err_code)
    {
        DCL_setError(df,err_code);
        DCL_getErrorInfo(df);
        DCL_runErrorHandler(df);
    }
#endif

    float32_t beta1 = -(float32_t) crealf(zpk->z1 + zpk->z2);
    float32_t beta0 = (float32_t) crealf(zpk->z1 * zpk->z2);
    float32_t alpha1 = -(float32_t) crealf(zpk->p1 + zpk->p2);
    float32_t alpha0 = (float32_t) crealf(zpk->p1 * zpk->p2);

    float32_t T = df->css->T;
    float32_t a0p = 4.0f + (alpha1 * 2.0f * T) + (alpha0 * T * T);

    df->sps->b0 = zpk->K * (4.0f + (beta1 * 2.0f * T) + (beta0 * T * T)) / a0p;
    df->sps->b1 = zpk->K * (-8.0f + (2.0f * beta0 * T * T)) / a0p;
    df->sps->b2 = zpk->K * (4.0f - (beta1 * 2.0f * T) + (beta0 * T * T)) / a0p;
    df->sps->a1 = (-8.0f + (2.0f * alpha0 * T * T)) / a0p;
    df->sps->a2 = (4.0f - (alpha1 * 2.0f * T) + (alpha0 * T * T)) / a0p;
}

//! \brief           Executes a 3rd order Direct Form 1 controller
//!                  
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF12(DCL_DF12 *df, float32_t ek)
{
    float32_t v4 = (ek * df->b0) + (df->d1 * df->b1) + (df->d2 * df->b2) - (df->d3 * df->a1) - (df->d4 * df->a2);
    df->d2 = df->d1;
    df->d1 = ek;
    df->d4 = df->d3;
    df->d3 = v4;

    return(v4);
}

//! \brief           Immediate computation to obtain DF12 servo error 
//!                  without updating the controller
//!
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF12PartialCompute(DCL_DF12 *df, float32_t ek)
{
    float32_t v4 = (ek * df->b0) + (df->d1 * df->b1) + (df->d2 * df->b2) - (df->d3 * df->a1) - (df->d4 * df->a2);
    return(v4);
}

//! \brief           Update DF12 controller based on pre-computed control effort
//!                  
//! \param[in] df    Pointer to the DCL_DF12 controller structure
//! \param[in] ek    The servo error
//! \param[in] uk    The controller output in the previous sample interval
//!
_DCL_CRIT_ACCESS
void DCL_runDF12PartialUpdate(DCL_DF12 *df, float32_t ek, float32_t uk)
{
    df->d2 = df->d1;
    df->d1 = ek;
    df->d4 = df->d3;
    df->d3 = uk;
}

//! \brief            Executes a 3rd order Direct Form 1 controller with clamp
//!                 
//! \param[in] df     Pointer to the DCL_DF12 controller structure
//! \param[in] ek     The servo error
//! \param[in] Umax   Upper saturation limit
//! \param[in] Umin   Lower saturation limit
//! \return    uk     The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF12Clamp(DCL_DF12 *df, float32_t ek, float32_t Umax, float32_t Umin)
{
    float32_t uk = DCL_runDF12PartialCompute(df, ek);
    bool is_clamped = DCL_runClamp(&uk, Umax, Umin);
    if(!is_clamped) DCL_runDF12PartialUpdate(df, ek, uk);
    return(uk);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_DF12_H_
