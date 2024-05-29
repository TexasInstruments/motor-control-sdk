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

#ifndef _DCL_REFGEN_H_
#define _DCL_REFGEN_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_refgen.h
 *  \brief      Defines the interface to reference generator module (REFGEN)
 */

#include "dcl_fdlog.h"
#include "../dcl_common.h"

//! \brief          Defines the minimum normalized increment
//!
#define DCL_REFGEN_MIN_INC      1.0e-06f

//! \brief          Defines the REFGEN operating modes
//!
typedef enum dcl_rgen_modes 
{
    REFGEN_OFF    =   0,
    REFGEN_STATIC =   1,
    REFGEN_SINE   =   2,
    REFGEN_SQUARE =   3,
    REFGEN_SAW    =   4,
    REFGEN_TRIANGLE = 5,
    REFGEN_PULSE  =   6,
    REFGEN_SINE2  =   7,
    REFGEN_SINE3  =   8
} DCL_refgen_modes;

//! \brief          Defines the DCL_REFGEN structure
//!
typedef _DCL_VOLATILE struct dcl_refgen 
{
    float32_t   rtgt;       //!< Target ramp value
    float32_t   rinc;       //!< Ramp increment
    float32_t   amtgt;      //!< Target amplitude value
    float32_t   aminc;      //!< Amplitude increment
    float32_t   fmtgt;      //!< Frequency ramp value
    float32_t   fminc;      //!< Frequency increment
    float32_t   thinc;      //!< Angular increment
    float32_t   duty;       //!< Pulse duty cycle
    float32_t   umax;       //!< Maximum allowable output
    float32_t   umin;       //!< Minimum allowable output
    float32_t   yr;         //!< Ramp generator output
    float32_t   ampl;       //!< Dynamic amplitude
    float32_t   freq;       //!< Dynamic frequency
    float32_t   theta;      //!< Normalized angle - phase A
    float32_t   thetb;      //!< Normalized angle - phase B
    float32_t   thetc;      //!< Normalized angle - phase C
    float32_t   ya;         //!< Phase A output
    float32_t   yb;         //!< Phase B output
    float32_t   yc;         //!< Phase C output
    uint32_t    mode;       //!< Operating mode
    DCL_CSS     *css;       //!< Pointer to the common support structure
} DCL_REFGEN;

//! \brief          Defines default values to initialize the DCL_REFGEN structure
//!
#define DCL_REFGEN_DEFAULTS { 0.0f,  0.0f, \
                              0.0f,  0.0f, \
                              0.0f,  0.0f,  0.0f, \
                              0.0f,  1.0f,  1.0f,  1.0f, -1.0f, \
                              0.0f,  0.0f,  0.0f,  0.0f, \
                              0.0f,  0.0f,  0.0f, \
                              REFGEN_OFF, &(DCL_CSS)DCL_CSS_DEFAULTS }

//! \brief          Resets DCL_REFGEN dynamic data
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//!
_DCL_CODE_ACCESS
void DCL_resetRefgen(DCL_REFGEN *p)
{
    p->ampl = 0.0f;
    p->freq = 0.0f;
    p->rinc = 0.0f;
    p->theta = 0.0f;
    p->thetb = 0.0f;
    p->thetc = 0.0f;
    p->thinc = 0.0f;
    p->yr = 0.0f;
    p->ya = 0.0f;
    p->yb = 0.0f;
    p->yc = 0.0f;
}

//! \brief          Loads the REFGEN ramp parameters
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] tgt  The new static reference target
//! \param[in] tr   The time interval to reach the new target
//!
_DCL_CODE_ACCESS
void DCL_setRefgenRamp(DCL_REFGEN *p, float32_t tgt, float32_t tr)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (tr < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(p,err_code);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
    }
#endif

    // compute ramp increment
    tr = (tr < p->css->T) ? p->css->T : tr;
    p->rtgt = tgt;
    p->rinc = p->css->T * (tgt - p->yr) / tr;

    // clamp minimum ramp increment
    if (!(0.0f == p->rinc) && (fabsf(p->rinc) < DCL_REFGEN_MIN_INC))
    {
#ifdef DCL_ERROR_HANDLING_ENABLED
        DCL_setError(p, dcl_comp_err);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
#endif
        if (signbit(p->rinc))
        {
            p->rinc = -DCL_REFGEN_MIN_INC;
        }
        else
        {
            p->rinc = DCL_REFGEN_MIN_INC;
        }
    }
}

//! \brief          Loads the REFGEN dynamic amplitude
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] ampl The new target amplitude
//! \param[in] tr   The time interval to reach the new target
//!
_DCL_CODE_ACCESS
void DCL_setRefgenAmpl(DCL_REFGEN *p, float32_t ampl, float32_t tr)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (tr < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(p,err_code);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
    }
#endif

    // compute amplitude increment
    tr = (tr < p->css->T) ? p->css->T : tr;
    p->amtgt = ampl;
    p->aminc = p->css->T * (ampl - p->ampl) / tr;

    // clamp minimum amplitude increment
    if ((p->aminc > 0.0f) && (fabsf(p->aminc) < DCL_REFGEN_MIN_INC))
    {
#ifdef DCL_ERROR_HANDLING_ENABLED
        DCL_setError(p, dcl_comp_err);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
#endif
        if (signbit(p->aminc))
        {
            p->aminc = -DCL_REFGEN_MIN_INC;
        }
        else
        {
            p->aminc = DCL_REFGEN_MIN_INC;
        }
    }
}

//! \brief          Loads the REFGEN frequency
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] freq The new frequency in Hz
//! \param[in] tr   The time interval to reach the new target
//!
_DCL_CODE_ACCESS
void DCL_setRefgenFreq(DCL_REFGEN *p, float32_t freq, float32_t tr)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (tr < 0.0f) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(p,err_code);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
    }
#endif

    // compute frequency increment
    tr = (tr < p->css->T) ? p->css->T : tr;
    p->fmtgt = freq;
    p->fminc = p->css->T * (freq - p->freq) / tr;

    // clamp minimum frequency increment
    if ((p->fminc > 0.0f) && (fabsf(p->fminc) < DCL_REFGEN_MIN_INC))
    {
#ifdef DCL_ERROR_HANDLING_ENABLED
        DCL_setError(p, dcl_comp_err);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
#endif
        if (signbit(p->fminc))
        {
            p->fminc = -DCL_REFGEN_MIN_INC;
        }
        else
        {
            p->fminc = DCL_REFGEN_MIN_INC;
        }
    }
}

//! \brief          Loads the REFGEN pulse duty cycle
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] duty The new per-unit duty cycle
//!
_DCL_CODE_ACCESS
void DCL_setRefgenDuty(DCL_REFGEN *p, float32_t duty)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= ((duty < 0.0f) || (duty > 1.0f)) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(p,err_code);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
    }
#endif

    p->duty = duty;
}

//! \brief          Loads the REFGEN output clam limits
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] max  The upper limit
//! \param[in] min  The lower limit
//!
_DCL_CODE_ACCESS
void DCL_setRefgenClamp(DCL_REFGEN *p, float32_t max, float32_t min)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    uint32_t err_code = dcl_none;
    err_code |= (max < min) ? dcl_param_range_err : dcl_none;
    if (err_code)
    {
        DCL_setError(p,err_code);
        DCL_getErrorInfo(p);
        DCL_runErrorHandler(p);
    }
#endif

    p->umax = max;
    p->umin = min;
}


//! \brief          Sets the REFGEN operating mode
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//! \param[in] mode The new operating mode
//!
_DCL_CODE_ACCESS
void DCL_setRefgenMode(DCL_REFGEN *p, int16_t mode)
{
    p->mode = mode;
}

//! \brief          Runs the REFGEN module
//!
//! \param[in] p    Pointer to the DCL_REFGEN structure
//!
_DCL_CODE_ACCESS
void DCL_runRefgen(DCL_REFGEN *p)
{
#ifdef DCL_ERROR_HANDLING_ENABLED
    float32_t v;
#endif

    // static offset generator
    if (!DCL_isZero(p->rinc))
    {
        if (fabsf(p->rinc) > fabsf(p->rtgt - p->yr))
        {
            p->yr = p->rtgt;
            p->rinc = 0.0f;
            // Adjustment complete
            DCL_clearUpdateStatus(p); 
        }
        else
        {
#ifdef DCL_ERROR_HANDLING_ENABLED
            v = p->yr;
#endif
            p->yr += p->rinc;
#ifdef DCL_ERROR_HANDLING_ENABLED
            if (DCL_isZero(v - p->yr))
            {
                p->css->err = dcl_comp_err;
                DCL_getErrorInfo(p);
                DCL_runErrorHandler(p);
            }
#endif
            // Adjustment running
            DCL_setUpdateStatus(p);
        }
    }

    // amplitude modulator
    if (!DCL_isZero(p->aminc))
    {
        if (fabsf(p->aminc) > fabsf(p->amtgt - p->ampl))
        {
            p->ampl = p->amtgt;
            p->aminc = 0.0f;
        }
        else
        {
#ifdef DCL_ERROR_HANDLING_ENABLED
            v = p->ampl;
#endif
            p->ampl += p->aminc;
#ifdef DCL_ERROR_HANDLING_ENABLED
            if (DCL_isZero(v - p->ampl))
            {
                p->css->err = dcl_comp_err;
                DCL_getErrorInfo(p);
                DCL_runErrorHandler(p);
            }
#endif
        }
    }

    // frequency modulator
    if (!DCL_isZero(p->fminc))
    {
        if (fabsf(p->fminc) > fabsf(p->fmtgt - p->freq))
        {
            p->freq = p->fmtgt;
            p->fminc = 0.0f;
        }
        else
        {
#ifdef DCL_ERROR_HANDLING_ENABLED
            v = p->freq;
#endif
            p->freq += p->fminc;
#ifdef DCL_ERROR_HANDLING_ENABLED
            if (DCL_isZero(v - p->freq))
            {
                p->css->err = dcl_comp_err;
                DCL_getErrorInfo(p);
                DCL_runErrorHandler(p);
            }
#endif
        }
    }

    // angle increment
    if (p->freq > 0.0f)
    {
        p->thinc = p->css->T * p->freq;
        if (p->thinc < DCL_REFGEN_MIN_INC)
        {
#ifdef DCL_ERROR_HANDLING_ENABLED
            p->css->err = dcl_comp_err;
            DCL_getErrorInfo(p);
            DCL_runErrorHandler(p);
#endif
            p->thinc = DCL_REFGEN_MIN_INC;
        }
    }
    else
    {
        p->thinc = 0.0f;
    }

    p->theta += p->thinc;
    p->theta -= (p->theta >= 1.0f) ? 1.0f : 0.0f;

    // dynamic signal generator
    switch (p->mode) 
    {
        case REFGEN_STATIC:
            p->ya = 0.0f;
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_SINE:
            p->ya = sinf(CONST_PI * p->theta);
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_SQUARE:
            p->ya = (p->theta > 0.5f) ? 1.0f : 0.0f;
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_SAW:
            p->ya = p->theta;
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_TRIANGLE:
            if (p->theta < 0.5f)
            {
                p->ya = 2.0f * p->theta;
            }
            else
            {
                p->ya = 1.0f - 2.0f * (p->theta - 0.5f);
            }
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_PULSE:
            p->ya = (p->theta > p->duty) ? 0.0f : 1.0f;
            p->yb = 0.0f;
            p->yc = 0.0f;
            break;

        case REFGEN_SINE2:
            p->ya = sinf(CONST_2PI * p->theta);
            p->yb = cosf(CONST_2PI * p->theta);
            p->yc = 0.0f;
            break;

        case REFGEN_SINE3:
            p->thetb = p->theta + 0.3333333333333f;
            p->thetc = p->theta + 0.6666666666667f;
            p->thetb -= (p->thetb >= 1.0f) ? 1.0f : 0.0f;
            p->thetc -= (p->thetc >= 1.0f) ? 1.0f : 0.0f;
            p->ya = sinf(CONST_2PI * p->theta);
            p->yb = sinf(CONST_2PI * p->thetb);
            p->yc = sinf(CONST_2PI * p->thetc);
            break;

        case REFGEN_OFF:
        default:
            p->yr = 0.0f;
            p->ya = 0.0f;
            p->yb = 0.0f;
            p->yc = 0.0f;
    }

    // output sum & saturation
    p->ya = DCL_runSat(p->ampl * p->ya + p->yr, p->umax, p->umin);
    p->yb = DCL_runSat(p->ampl * p->yb + p->yr, p->umax, p->umin);
    p->yc = DCL_runSat(p->ampl * p->yc + p->yr, p->umax, p->umin);
}

//! \brief          Returns the phase reference output
//!
//! \param[in] p    Pointer to the active DCL_REFGEN structure
//! \return         The phase A reference output
//!
_DCL_CODE_ACCESS
float32_t DCL_getRefgenPhaseA(DCL_REFGEN *p)
{
    return(p->ya);
}

//! \brief          Returns the phase reference output
//!
//! \param[in] p    Pointer to the active DCL_REFGEN structure
//! \return         The phase B reference output
//!
_DCL_CODE_ACCESS
float32_t DCL_getRefgenPhaseB(DCL_REFGEN *p)
{
    return(p->yb);
}

//! \brief          Returns the phase reference output
//!
//! \param[in] p    Pointer to the active DCL_REFGEN structure
//! \return         The phase C reference output
//!
_DCL_CODE_ACCESS
float32_t DCL_getRefgenPhaseC(DCL_REFGEN *p)
{
    return(p->yc);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_REFGEN_H_
