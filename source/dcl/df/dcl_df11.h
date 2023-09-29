/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _DCL_DF11_H_
#define _DCL_DF11_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *
 *  \file       dcl_df11.h
 *  \brief      Contains direct form 1 1st order DF11 compensator
 *              with its related structures and functions
 */                 

#include "../dcl_common.h"

//--- Direct Form 1 - 1st order ----------------------------------------------

//! \brief          Defines DCL_DF11 shadow parameter set
//!                 used for updating compensator parameter
//!
typedef struct dcl_df11_sps
{
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t a1;   //!< neg. coefficient to u(k-1)
} DCL_DF11_SPS;

#define DF11_SPS_DEFAULTS { 0.5f, 0.5f, 1.0f }

//! \brief          DCL_DF11 object for storing df11 specific parameters
//!
typedef _DCL_VOLATILE struct dcl_df11
{
    /* compensator parameter */
    float32_t b0;   //!< pos. coefficient to e(k)
    float32_t b1;   //!< pos. coefficient to e(k-1)
    float32_t a1;   //!< neg. coefficient to u(k-1)

    /* internal storage */
    float32_t d1;   //!< e(k-1)
    float32_t d2;   //!< u(k-1)

    /* miscellaneous */
    DCL_DF11_SPS *sps; //!< Pointer to the shadow parameter set
    DCL_CSS *css;      //!< Pointer to the common support structure
} DCL_DF11, *DF11_Handle;

//! \brief          Defines default values to initialize DCL_DF11
//!
#define DF11_DEFAULTS { 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, \
        &(DCL_DF11_SPS)DF11_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Macro for internal default values to initialize DCL_DF11
//!                 Example: DCL_DF11 df11_ctrl = { 
//!                                             .b0 = 0.5f,
//!                                             .b1 = 0.5f,
//!                                             .a1 = 1.0f,
//!                                             DF11_INT_DEFAULTS
//!                                           };
#define DF11_INT_DEFAULTS .d1=0.0f, .d2=0.0f, .sps=&(DCL_DF11_SPS)DF11_SPS_DEFAULTS, .css=&(DCL_CSS)DCL_CSS_DEFAULTS 

//! \brief          Initialize DCL_DF11 struct with default parameters
//!                 Example: DCL_DF11* df11_ctrl = DCL_initDF11();
//!
//! \return         A DCL_DF11* pointer
//!
#define DCL_initDF11() &(DCL_DF11)DF11_DEFAULTS

//! \brief          Initialize DCL_DF11 struct with input compensator parameters
//!                 Example: DCL_DF11* DF11_ctrl = DCL_initDF11asParam(0.5f,0.5f,1.0f);
//!                 Note: input parameter needs to be in the same order as listed in DF11_SPS struct
//!
//! \return         A DCL_DF11* pointer
//!
#define DCL_initDF11asParam(_b0,_b1,_a1) &(DCL_DF11){ .b0=_b0, .b1=_b1, .a1=_a1, \
                                DF11_INT_DEFAULTS }

//! \brief                  Initialize DCL_DF11 struct with sps parameters
//!                         Example: DCL_DF11_SPS df_sps = { .b0 = , .b1 = , ...}; //initial parameter
//!                                  DCL_DF11 df_ctrl;
//!                                  DCL_initDF11asSPS(&df_ctrl,&df_sps);
//!
//! \param[in] df_ptr       DCL_DF11* pointer that needs to be initialized
//! \param[in] sps_ptr      DCL_DF11_SPS* pointer with assigned parameters
//! \return    DCL_DF11*    Returns DCL_DF11* with set sps parameters, default parameter will be used
//!                         if sps_ptr is not specified   
//!
#define DCL_initDF11asSPS(df_ptr,sps_ptr)                                          \
({                                                                                 \
    DCL_DF11* new_df = (df_ptr) ? df_ptr : DCL_initDF11();                         \
    DCL_DF11_SPS* new_sps = (sps_ptr) ? sps_ptr : &(DCL_DF11_SPS)DF11_SPS_DEFAULTS;\
    if(sps_ptr)                                                                    \
    {                                                                              \
        *new_df = (DCL_DF11){ (new_sps)->b0, (new_sps)->b1, (new_sps)->a1,         \
        0.0f, 0.0f, (DCL_DF11_SPS*)new_sps, &(DCL_CSS)DCL_CSS_DEFAULTS };          \
    }                                                                              \
    new_df;                                                                        \
})

//! \brief           Resets DF11 internal storage data with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF11 structure
//!
_DCL_CODE_ACCESS
void DCL_resetDF11(DCL_DF11 *df)
{
    dcl_interrupt_t ints = DCL_disableInts();
    df->d1 = df->d2 = 0.0f;
    DCL_restoreInts(ints);
}

//! \brief           Loads DF11 tuning parameter from its SPS parameter
//!
//! \param[in] df    Pointer to the active DCL_DF11 controller structure
//!
_DCL_CODE_ACCESS
void DCL_fupdateDF11(DCL_DF11 *df)
{
    df->b0 = df->sps->b0;
    df->b1 = df->sps->b1;
    df->a1 = df->sps->a1;
}

//! \brief           Updates DF11 parameter from its SPS parameter with interrupt protection
//!
//! \param[in] df    Pointer to the DCL_DF11 controller structure
//! \return          'true' if update is successful, otherwise 'false'
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
bool DCL_updateDF11(DCL_DF11 *df)
{
    if (!DCL_getUpdateStatus(df))
    {
        dcl_interrupt_t ints = DCL_disableInts();
        DCL_setUpdateStatus(df);
        df->b0 = df->sps->b0;
        df->b1 = df->sps->b1;
        df->a1 = df->sps->a1;
        DCL_clearUpdateStatus(df);
        DCL_restoreInts(ints);
        return true;
    }
    return false;
}

//! \brief           A conditional update based on the pending-for-update flag.
//!                  If the pending status is set, the function will update DF11
//!                  parameter from its SPS parameter and clear the status flag on completion.
//!                  Note: Use DCL_setPendingStatus(df) to set the pending status.
//!     
//! \param[in] df    Pointer to the DCL_DF11 controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
bool DCL_pendingUpdateDF11(DCL_DF11 *df)
{
    if (DCL_getPendingStatus(df) && DCL_updateDF11(df))
    {
        DCL_clearPendingStatus(df);
        return true;
    }
    return false;
}

//! \brief           Update SPS parameter with active param, userful when needing
//!                  to update only few active param from SPS and keep rest the same  
//! 
//! \param[in] df    Pointer to the active DCL_DF11 controller structure
//!
_DCL_CODE_ACCESS
void DCL_updateDF11SPS(DCL_DF11 *df)
{
    df->sps->b0 = df->b0;
    df->sps->b1 = df->b1;
    df->sps->a1 = df->a1;
}

//! \brief           Determines stability of the shadow DF11 compensator
//!
//! \param[in] df    Pointer to the DCL_DF11 controller structure
//! \return         'true' if the pole has magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStableDF11(DCL_DF11 *df)
{
    return(DCL_isStablePn1(df->sps->a1));
}

//! \brief            Loads the DF11 shadow coefficients from a ZPK3 description
//!                   Note: Sampling period df->css->t_sec are used in the calculation.
//!                   New settings take effect after DCL_updateDF11().
//!                   Only real z1 & p1 considered, all other roots ignored.
//!
//! \param[in] df     Pointer to the DCL_DF11 controller structure
//! \param[in] zpk    Pointer to the ZPK3 structure
//!
_DCL_CODE_ACCESS
void DCL_loadDF11asZPK(DCL_DF11 *df, DCL_ZPK3 *zpk)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= DCL_isZero(cimagf(zpk->z1)) ? dcl_none : dcl_param_warn_err;
    err_code |= DCL_isZero(cimagf(zpk->p1)) ? dcl_none : dcl_param_warn_err;
    if (err_code)
    {
        DCL_setError(df,err_code);
        DCL_getErrorInfo(df);
        DCL_runErrorHandler(df);
    }
#endif

    float32_t t_sec = df->css->t_sec;
    float32_t a0p = 2.0f - (float32_t) crealf(zpk->p1) * t_sec;
    df->sps->b0 = zpk->K * (2.0f - (float32_t) crealf(zpk->z1) * t_sec) / a0p;
    df->sps->b1 = zpk->K * (-2.0f - (float32_t) crealf(zpk->z1) * t_sec) / a0p;
    df->sps->a1 = (-2.0f - (float32_t) crealf(zpk->p1) * t_sec) / a0p;
}

//! \brief           Loads compensator coefficients to emulate series form PI
//!                  Note: Sampling period df->css->t_sec are used in the calculation.
//!                  New settings take effect after DCL_updateDF11().
//!
//! \param[in] df    Pointer to the DCL_DF11 controller structure
//! \param[in] Kp    Proportional gain
//! \param[in] Ki    Integral gain
//!
_DCL_CODE_ACCESS
void DCL_loadDF11asPI(DCL_DF11 *df, float32_t Kp, float32_t Ki)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (Kp < 0.0f) ? dcl_param_invalid_err : dcl_none;
    err_code |= (Ki < 0.0f) ? dcl_param_invalid_err : dcl_none;
    if (err_code)
    {
        DCL_setError(df,err_code);
        DCL_getErrorInfo(df);
        DCL_runErrorHandler(df);
    }
#endif

    float32_t t_sec = df->css->t_sec;
    df->sps->b0 = Kp * ((Ki * t_sec) + 2.0f) / 2.0f;
    df->sps->b1 = Kp * ((Ki * t_sec) - 2.0f) / 2.0f;
    df->sps->a1 = -1.0f;
}

//! \brief           Executes a 1st order Direct Form 1 controller
//!                  
//! \param[in] df    Pointer to the DCL_DF11 controller structure
//! \param[in] ek    The servo error
//! \return    uk    The control effort
//!
_DCL_CODE_ACCESS _DCL_CODE_SECTION
float32_t DCL_runDF11(DCL_DF11 *df, float32_t ek)
{
    df->d2 = (ek * df->b0) + (df->d1 * df->b1) - (df->d2 * df->a1);
    df->d1 = ek;

    return(df->d2);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_DF11_H_
