/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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

#ifndef _DCL_GSM_H_
#define _DCL_GSM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_gsm.h
 *  \brief      Defines the interface to the Gain Scheduler Module (GSM)
 */

#include "../dcl_common.h"

//! \brief          Number of piecewise linear sections
//!
#ifndef GSM_N
#define GSM_N   8
#endif

//! \brief          Defines DCL_GSM shadow parameter set
//!                 used for updating parameters
//!
typedef struct dcl_gsm_sps
{
    float32_t m[GSM_N];     //!< sector gain array
    float32_t c[GSM_N+1];   //!< sector offset array
} DCL_GSM_SPS;

//! \brief          Defines default values to initialize the DCL_GSM shadow structure (GSM_N=8)
//!
#define GSM_SPS_DEFAULTS { { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }, \
        { 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f } }

//! \brief          Defines the TCM structure
//!
typedef _DCL_VOLATILE struct dcl_gsm {
    float32_t   m[GSM_N];       //!< sector gain array
    float32_t   c[GSM_N+1];     //!< sector offset array
    float32_t   h;              //!< interval size
    DCL_GSM_SPS *sps;           //!< Pointer to the shadow parameter set
    DCL_CSS *css;               //!< Pointer to the common support structure
} DCL_GSM;

//! \brief          Defines default values for the GSM structure (GSM_N=8)
//!
#define GSM_DEFAULTS { { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f }, \
                       { 0.0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1.0f }, \
                         1.0f/GSM_N, &(DCL_GSM_SPS)GSM_SPS_DEFAULTS, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Resets the active GSM parameters to unity gain
//! 
//! \param[in] gsm  Pointer to the active DCL_GSM controller structure
//!
_DCL_CODE_ACCESS
void DCL_resetGSM(DCL_GSM *gsm)
{
    int i;
    dcl_interrupt_t ints;

    ints = DCL_disableInts();
    for (i = 0; i < GSM_N; i++)
    {
        gsm->m[i] = 1.0f;
        gsm->c[i] = (float32_t) i * gsm->h;
    }
    gsm->c[GSM_N] = 1.0f;
    DCL_restoreInts(ints);
}

//! \brief          Loads the active GSM parameters from its SPS parameter without interrupt protection
//!
//! \param[in] gsm  Pointer to the active DCL_GSM controller structure
//!
_DCL_CODE_ACCESS
void DCL_forceUpdateGSM(DCL_GSM *gsm)
{
    int i;

    for (i = 0; i < GSM_N; i++)
    {
        gsm->m[i] = gsm->sps->m[i];
        gsm->c[i] = gsm->sps->c[i];
    }
    gsm->c[GSM_N] = gsm->sps->c[GSM_N];
}

//! \brief          Loads the active GSM parameters from its SPS parameter with interrupt protection
//!
//! \param[in] gsm  Pointer to the active DCL_GSM controller structure
//!
_DCL_CODE_ACCESS
void DCL_updateGSMNoCheck(DCL_GSM *gsm)
{;
    dcl_interrupt_t ints;

    ints = DCL_disableInts();
    DCL_forceUpdateGSM(gsm);
    DCL_restoreInts(ints);
}

//! \brief           A conditional update based on the update flag.
//!                  If the update status is set, the function will update PID
//!                  parameter from its SPS parameter and clear the status flag on completion.
//! \note            Note: Use DCL_setUpdateStatus(gsm) to set the update status.
//!
//! \param[in] gsm   Pointer to the active DCL_GSM controller structure
//! \return          'true' if an update is applied, otherwise 'false'
//!
_DCL_CODE_ACCESS
bool DCL_updateGSM(DCL_GSM *gsm)
{
    if (DCL_getUpdateStatus(gsm))
    {
        DCL_updateGSMNoCheck(gsm);
        DCL_clearUpdateStatus(gsm);
        return true;
    }
    return false;
}

//! \brief          Computes the shadow scheduler gains from the offset selection
//!                 Active gains are unaffected until DCL_updateGSM() is called
//! \param[in] gsm  Pointer to DCL_GSM structure
//!
_DCL_CODE_ACCESS
void DCL_loadGSMgains(DCL_GSM *gsm)
{
    int i;

    for (i = 0; i < GSM_N; i++)
    {
        gsm->sps->m[i] = (gsm->sps->c[i+1] - gsm->sps->c[i]) * (float32_t)GSM_N;
    }
}

//! \brief          Computes the shadow scheduler offsets from the gain selection
//!                 Active offsets are unaffected until DCL_updateGSM() is called
//! \param[in] gsm  Pointer to DCL_GSM structure
//!
_DCL_CODE_ACCESS
void DCL_loadGSMoffsets(DCL_GSM *gsm)
{
    int i;

    for (i = 0; i <= GSM_N; i++)
    {
        gsm->sps->c[i] = gsm->sps->c[i-1] + (gsm->sps->m[i-1] * gsm->h);
    }
    gsm->sps->c[0] = 0.0f;
}

//! \brief          Runs the Gain Scheduler Module
//! \param[in] gsm  Pointer to DCL_GSM structure
//! \param[in] x    Input
//! \return         Control effort
//!
_DCL_CRIT_ACCESS
float32_t DCL_runGSM(DCL_GSM *gsm, float32_t x)
{
    int sector;
    float32_t sgnx, modx, rval;

    sgnx = (x >= 0.0f) ? 1.0f : -1.0f;
    modx = fabsf(x);

    if (modx >= 1.0f)
    {
        rval = gsm->c[GSM_N] * sgnx;
    }
    else
    {
        sector = modx * (float32_t) GSM_N;
        rval = sgnx * (gsm->m[sector] * (modx - (sector * gsm->h)) + gsm->c[sector]);
    }

    return(rval);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_GSM_H_
