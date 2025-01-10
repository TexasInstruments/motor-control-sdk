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
 
#ifndef _DCL_DF13_H_
#define _DCL_DF13_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *
 *  \file       dcl_df13.h
 *  \brief      Contains direct form 1 3rd order DF13 compensator
 *              with its related structures and functions 
 *
 *  \note       Due to compatibility reason, this file maintains a copy
 *              of legacy function DCL_runDF13_C5(), DCL_runDF13_C6(). 
 *              It is advised to not use these legacy functions for new development.
 */       

#include "../dcl_common.h"

//--- Direct Form 1 - 3rd order ----------------------------------------------

//! \brief          Defines the DCL_DF13 shadow parameter set
//!                 used for updating compensator parameter
//!
typedef struct dcl_df13_sps {
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t b2;   //!< pos. coefficient to e(k-2)
    float32_t b3;   //!< pos. coefficient to e(k-3)
    float32_t a1;   //!< neg. coefficient to u(k-1)
    float32_t a2;   //!< neg. coefficient to u(k-2)
    float32_t a3;   //!< neg. coefficient to u(k-3)
    float32_t a0;   //!< No Longer Needed
} DCL_DF13_SPS;

#define DF13_SPS_DEFAULTS { 0.25f, 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f, 0.0f }

//! \brief          DCL_DF13 object for storing df13 specific parameters
//!
typedef _DCL_VOLATILE struct dcl_df13 
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
    float32_t d1;   //!< e(k-1)
    float32_t d2;   //!< e(k-2)
    float32_t d3;   //!< e(k-3)
    float32_t d4;   //!< u(k-1)
    float32_t d5;   //!< u(k-2)
    float32_t d6;   //!< u(k-3)

    float32_t a0;   //!< No longer needed
    float32_t d0;   //!< No longer needed
    float32_t d7;   //!< No longer needed

    /* miscellaneous */
    DCL_DF13_SPS *sps; //!< updates compensator parameter
    DCL_CSS *css;      //!< configuration & debugging
} DCL_DF13;

//! \brief          Defines default values to initialize DCL_DF13
//!
#define DF13_DEFAULTS { 0.25f, 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f, \
                        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
            &(DCL_DF13_SPS)DF13_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_DF13
//!                 Example: DCL_DF13 df13_ctrl = { 
//!                                                 .b0 = 0.25f,
//!                                                 .b1 = 0.25f,
//!                                                 ...
//!                                                 .a3 = 0.0f,
//!                                                 DF13_INT_DEFAULTS
//!                                               };
#define DF13_INT_DEFAULTS .d1=0.0f, .d2=0.0f, .d3=0.0f, .d4=0.0f, .d5=0.0f, \
                          .d6=0.0f, .a0=0.0f, .d0=0.0f, .d7=0.0f, \
                          .sps=&(DCL_DF13_SPS)DF13_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS 

//! \brief          Initialize DCL_DF13 struct with default parameters
//!                 Example: DCL_DF13* df13_ctrl = DCL_initDF13();
//!
//! \return         A DCL_DF13* pointer
//!
#define DCL_initDF13() &(DCL_DF13)DF13_DEFAULTS

//! \brief          Initialize DCL_DF13 struct with input compensator parameters
//!                 Example: DCL_DF13* DF13_ctrl = DCL_initDF13asParam(0.25f,0.25f,0.25f,0.25f,0.0f,0.0f,0.0f);
//! \note           Note: input parameter needs to be in the same order as listed in DF13_SPS struct
//!
//! \return         A DCL_DF13* pointer
//!
#define DCL_initDF13asParam(_b0,_b1,_b2,_b3,_a1,_a2,_a3) &(DCL_DF13){ .b0=_b0, .b1=_b1, \
                        .b2=_b2, .b3=_b3, .a1=_a1, .a2=_a2, .a3=_a3, DF13_INT_DEFAULTS }

//! \brief                  Initialize DCL_DF13 struct with sps parameters
//!                         Example: DCL_DF13_SPS df_sps = { .b0 = , .b1 = , ...}; //initial parameter
//!                                  DCL_DF13 df_ctrl;
//!                                  DCL_initDF13asSPS(&df_ctrl,&df_sps);
//!
//! \param[in] df_ptr       DCL_DF13* pointer that needs to be initialized
//! \param[in] sps_ptr      DCL_DF13_SPS* pointer with assigned parameters
//! \return    DCL_DF13*    Returns DCL_DF13* with set sps parameters, default parameter will be used
//!                         if sps_ptr is not specified   
//!
#define DCL_initDF13asSPS(df_ptr,sps_ptr)                                                  \
({                                                                                         \
    DCL_DF13* new_df = (df_ptr) ? df_ptr : DCL_initDF13();                                 \
    DCL_DF13_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_DF13_SPS)DF13_SPS_DEFAULTS;        \
    if(sps_ptr)                                                                            \
    {                                                                                      \
        *new_df =(DCL_DF13){ (new_sps)->b0, (new_sps)->b1, (new_sps)->b2, (new_sps)->b3,   \
        (new_sps)->a1, (new_sps)->a2, (new_sps)->a3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,   \
        0.0f, 0.0f, 0.0f, (DCL_DF13_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };            \
    }                                                                                      \
    new_df;                                                                                \
})

//! \brief           Resets DF13 internal storage data with interrupt protection
//!                  Implemented as inline C function
//!
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//!
_DCL_CODE_ACCESS
void DCL_resetDF13(DCL_DF13 *df)
{
    dcl_interrupt_t ints;
    ints = DCL_disableInts();
    df->d1 = df->d2 = df->d3 = df->d4 = df->d5 = df->d6 = 0.0f;
    DCL_restoreInts(ints);
}

//! \brief           Loads DF13 tuning parameter from its SPS parameter without interrupt protection
//!
//! \param[in] df    Pointer to the active DCL_DF13 controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdateDF13(DCL_DF13 *df)
{
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->b2 = df->sps->b2;
    df->b3 = df->sps->b3;
    df->a1 = df->sps->a1;
    df->a2 = df->sps->a2;
    df->a3 = df->sps->a3;    
}

//! \brief           Loads DF13 tuning parameter from its SPS parameter with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updateDF13NoCheck(DCL_DF13 *df)
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
//!                  If the update status is set, the function will update DF13
//!                  parameter from its SPS parameter and clear the status flag on completion..
//! \note            Note: Use DCL_setUpdateStatus(df) to set the update status.
//!     
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updateDF13(DCL_DF13 *df)
{
    if (DCL_getUpdateStatus(df))
    {
        DCL_updateDF13NoCheck(df);
        DCL_clearUpdateStatus(df);
        return true;
    }
    return false;
}

//! \brief           Determines stability of the shadow compensator
//!
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//! \return          'true' if all poles have magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStableDF13(DCL_DF13 *df)
{
    return(DCL_isStablePn3(1.0f, df->sps->a1, df->sps->a2, df->sps->a3));
}

//! \brief            Loads the DF13 shadow coefficients from a ZPK3 description.
//! \note             Note: Sampling period df->css->T are used in the calculation.
//!                   New settings take effect after DCL_updateDF13().
//!
//! \param[in] df     Pointer to the DCL_DF13 controller structure
//! \param[in] zpk    Pointer to the DCL_ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadDF13asZPK(DCL_DF13 *df, DCL_ZPK3 *zpk)
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

//! \brief           Executes a 3rd order Direct Form 1 controller
//!                  
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF13(DCL_DF13 *df, float32_t ek)
{
    float32_t v4 = (ek * df->b0) + (df->d1 * df->b1) + (df->d2 * df->b2) + (df->d3 * df->b3) - (df->d4 * df->a1) - (df->d5 * df->a2) - (df->d6 * df->a3);
    df->d3 = df->d2;
    df->d2 = df->d1;
    df->d1 = ek;
    df->d6 = df->d5;
    df->d5 = df->d4;
    df->d4 = v4;

    return(v4);
}

//! \brief           Immediate computation to obtain DF13 servo error 
//!                  without updating the controller
//!
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//! \param[in] ek    The servo error
//! \return          The control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runDF13PartialCompute(DCL_DF13 *df, float32_t ek)
{
    float32_t v4 = (ek * df->b0) + (df->d1 * df->b1) + (df->d2 * df->b2) + (df->d3 * df->b3) - (df->d4 * df->a1) - (df->d5 * df->a2) - (df->d6 * df->a3);
    return(v4);
}

//! \brief           Update DF13 controller based on pre-computed control effort
//!                  
//! \param[in] df    Pointer to the DCL_DF13 controller structure
//! \param[in] ek    The servo error
//! \param[in] uk    The controller output in the previous sample interval
//!
_DCL_CRIT_ACCESS
void DCL_runDF13PartialUpdate(DCL_DF13 *df, float32_t ek, float32_t uk)
{
    df->d3 = df->d2;
    df->d2 = df->d1;
    df->d1 = ek;
    df->d6 = df->d5;
    df->d5 = df->d4;
    df->d4 = uk;
}

//! \brief            Executes a 3rd order Direct Form 1 controller with clamp
//!                 
//! \param[in] df     Pointer to the DCL_DF13 controller structure
//! \param[in] ek     The servo error
//! \param[in] Umax   Upper saturation limit
//! \param[in] Umin   Lower saturation limit
//! \return    uk     The control effort
//!
/*  
 *  \details    Previously, C28 version of DCL did not implement a function
 *              to run DF13 with clamp. Instead, user had to implement a routine 
 *              similar to the following: 
 *
 *  \code{.c}   extern DCL_DF13 df;
 *              extern float32_t ek, uk;
 *              float32_t vk = 0;
 *
 *              float32_t uk = DCL_runDF13_C2/C5(df, ek, vk);
 *              bool is_clamped = DCL_runClamp_C1/C2(&uk, Umax, Umin)
 *              if (!is_clamped)
 *              {
 *                  vk = DCL_runDF13_C3/C6(df, ek, uk);
 *              }     
 *  
 *              // A new function DCL_runDF13Clamp() was implemented to replace it,
 *              // therefore, user may change the aformentioned routine to just:
 *              uk = DCL_runDF13Clamp(df, ek, Umax, Umin);
 *  \endcode
 */
_DCL_CRIT_ACCESS
float32_t DCL_runDF13Clamp(DCL_DF13 *df, float32_t ek, float32_t Umax, float32_t Umin)
{
    float32_t uk = DCL_runDF13PartialCompute(df, ek);
    bool is_clamped = DCL_runClamp(&uk, Umax, Umin);
    if(!is_clamped) DCL_runDF13PartialUpdate(df, ek, uk);
    return(uk);
}

//! \brief          Legacy C28 Function for maintaining backwards compatibility.
//!                 It Executes an immediate 3rd order Direct Form 1 controller
//!
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] vk   The partial pre-computed control effort
//! \return    uk   The control effort
//!
_DCL_CRIT_ACCESS 
float32_t DCL_runDF13_C5(DCL_DF13 *p, float32_t ek, float32_t vk)
{
    p->d4 = (ek * p->b0) + vk;

    return(p->d4);
}

//! \brief          Legacy C28 Function for maintaining backwards compatibility.
//!                 It executes a partial pre-computed 3rd order Direct Form 1 controller
//!                 
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] uk   The controller output in the previous sample interval
//! \return    vk   The control effort
//!
_DCL_CRIT_ACCESS 
float32_t DCL_runDF13_C6(DCL_DF13 *p, float32_t ek, float32_t uk)
{
    float32_t v9;

    v9 = (ek * p->b1) + (p->d1 * p->b2) + (p->d2 * p->b3) - (uk * p->a1) - (p->d5 * p->a2) - (p->d6 * p->a3);
    p->d2 = p->d1;
    p->d1 = ek;
    p->d6 = p->d5;
    p->d5 = uk;

    return(v9);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_DF13_H_
