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

#ifndef _DCL_TCM_H_
#define _DCL_TCM_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_tcm.h
 *  \brief      Defines the interface to the Transient Capture Module (TCM)
 */

#include <math.h>
#include "dcl_fdlog.h"
#include "../dcl_common.h"

//! \brief          Defines the TCM structure
//!
typedef _DCL_VOLATILE struct dcl_tcm {
    DCL_FDLOG moniFrame;    //!< Monitor data frame
    DCL_FDLOG leadFrame;    //!< Lead data frame
    DCL_FDLOG captFrame;    //!< Capture data frame
    float32_t trigMax;      //!< Upper trigger threshold
    float32_t trigMin;      //!< Lower trigger threshold
    uint32_t mode;          //!< Operating mode
    uint32_t lead;          //!< Lead frame size in 32-bit words = trigger crossing index
} DCL_TCM, TCM;

//! \brief          Defines default values to initialize the TCM structure
//!
#define TCM_DEFAULTS { FDLOG_DEFAULTS, FDLOG_DEFAULTS, FDLOG_DEFAULTS, 0.1, -0.1, 0, 10 }

//! \brief          Enumerated TCM operating modes
//!
typedef enum dcl_tcm_states 
{
    TCM_INVALID  =  0,    //!< Buffer pointers not initialized
    TCM_IDLE     =  1,    //!< Memory initialized but module not armed
    TCM_ARMED    =  2,    //!< Armed: capturing monitor frame data and waiting for trigger
    TCM_CAPTURE  =  3,    //!< Triggered: logging data into capture frame and un-winding monitor frame
    TCM_COMPLETE =  4,    //!< Full data frame captured and available for read-out
} DCL_tcm_states;

//! \brief          Resets the TCM module: clears buffers and sets idle mode
//!
//! \param[in] q    The TCM structure
//! \param[in] addr The start address of the memory block
//! \param[in] size The size of the memory block in 32-bit words
//! \param[in] lead The length of the lead frame in samples
//! \param[in] tmin The upper trigger threshold
//! \param[in] tmax The lower trigger threshold
//!
_DCL_CODE_ACCESS
void DCL_initTCM(DCL_TCM *q, float32_t *addr, uint32_t size, uint32_t lead, float32_t tmin, float32_t tmax)
{
    // assign and clear capture frame, and initialize index to end of lead frame
    DCL_initLog(&(q->captFrame), addr, size);
    DCL_clearLog(&(q->captFrame));
    q->captFrame.dptr = q->captFrame.fptr + lead;

    // assign lead & monitor frame pointers
    DCL_initLog(&(q->leadFrame), addr, lead);
    DCL_initLog(&(q->moniFrame), q->captFrame.lptr - lead + 1, lead);

    // load remaining TCM vars
    q->lead = lead;
    q->trigMax = tmax;
    q->trigMin = tmin;
    q->mode = TCM_IDLE;
}

//! \brief          Resets the TCM module: clears all frame buffers and sets idle mode
//!
//! \param[in] q    The TCM structure
//!
_DCL_CODE_ACCESS
void DCL_resetTCM(DCL_TCM *q)
{
    DCL_clearLog(&(q->captFrame));
    q->captFrame.dptr = q->captFrame.fptr + q->lead - 1;
    DCL_resetLog(&(q->leadFrame));
    DCL_resetLog(&(q->moniFrame));
    q->mode = TCM_IDLE;
}

//! \brief          Changes the TCM mode to "TCM_ARMED".  
//! \note           Only valid if current operating mode is "TCM_IDLE"
//!
//! \param[in] q    The TCM structure
//! \return         The operating mode
//!
_DCL_CODE_ACCESS
uint16_t DCL_armTCM(DCL_TCM *q)
{
    q->mode = (q->mode == TCM_IDLE) ? TCM_ARMED : TCM_IDLE;
    return(q->mode);
}

//! \brief          Runs the TCM module
//!
//! \param[in] q    The TCM structure
//! \param[in] data The input data
//! \return         The operating mode
//!
_DCL_CODE_ACCESS
uint16_t DCL_runTCM(DCL_TCM *q, float32_t data)
{
    switch(q->mode)
    {
    // TCM not initialized
    case TCM_INVALID:
    // idle: do nothing
    case TCM_IDLE:
    // complete: do nothing - results available in capture buffer
    case TCM_COMPLETE:
    default:
        break;

    // armed: ready to begin capturing when either trigger threshold is crossed
    case TCM_ARMED:
        // check for trigger condition
        if ((data > q->trigMax) || (data < q->trigMin))
        {
            // capture first data point & switch to capture mode
            DCL_writeLog(&(q->captFrame), data);
            DCL_writeLog(&(q->leadFrame), DCL_readLog(&(q->moniFrame)));
            q->mode = TCM_CAPTURE;
        }
        else
        {
            // log data into monitor frame
            DCL_writeLog(&(q->moniFrame), data);
        }
        break;

    // capture mode: acquiring data
    case TCM_CAPTURE:
        // check for full capture frame
        if (q->captFrame.dptr > q->captFrame.fptr)
        {
            // write data into main frame
            DCL_writeLog(&(q->captFrame), data);

            // check for full lead frame
            if (q->leadFrame.dptr > q->leadFrame.fptr)
            {
                // un-wind monitor data into lead frame
                DCL_writeLog(&(q->leadFrame), DCL_readLog(&(q->moniFrame)));
            }
        }
        else
        {
            // TCM complete, capture frame spans full buffer
            q->captFrame.fptr = q->leadFrame.fptr;
            q->captFrame.dptr = q->captFrame.fptr;
            q->mode = TCM_COMPLETE;
        }
        break;
    }

    return(q->mode);
}

//! \brief          Computes ITAE performance index from a log of servo error.
//!
//! \param[in] elog The servo error log
//! \param[in] prd  The sampling period in seconds
//! \return         The ITAE index
//!
_DCL_CODE_ACCESS
float32_t DCL_runITAE(DCL_FDLOG *elog, float32_t prd)
{
    float32_t tim = 0.0f;
    float32_t rlt = 0.0f;
    uint32_t size = DCL_getLogSize(elog);

    // set index pointer to buffer start
    DCL_resetLog(elog);

    // accumulate ITAE data
    while (size--)
    {
        rlt += fabs(*(elog->dptr++)) * tim;
        tim += prd;
    }
    DCL_resetLog(elog);

    return(rlt);
}

//! \brief          Computes IAE performance index from a log of servo error.
//!
//! \param[in] elog The servo error log
//! \return         The IAE index
//!
_DCL_CODE_ACCESS
float32_t DCL_runIAE(DCL_FDLOG *elog)
{
    float32_t rlt = 0.0f;
    uint32_t size = DCL_getLogSize(elog);

    // set index pointer to buffer start
    DCL_resetLog(elog);

    // accumulate IAE data
    while (size--)
    {
        rlt += fabs(*(elog->dptr++));
    }
    DCL_resetLog(elog);

    return(rlt);
}

//! \brief          Computes IES performance index from a log of servo error.
//!
//! \param[in] elog The servo error log
//! \return         The IES index
//!
_DCL_CODE_ACCESS
float32_t DCL_runIES(DCL_FDLOG *elog)
{
    float32_t err;
    float32_t rlt = 0.0f;
    uint32_t size = DCL_getLogSize(elog);

    // set index pointers to buffer start
    DCL_resetLog(elog);

    // accumulate IES data
    while (size--)
    {
        err = *(elog->dptr++);
        rlt += (err * err);
    }

    // restore index pointer
    DCL_resetLog(elog);

    return(rlt);
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_TCM_H_
