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
 
#ifndef _DCL_ERROR_H_
#define _DCL_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_error.h
 *  \brief      Defines all error handling strctures and macro
 */

#include "../dcl_common.h"

//--- Error handling ---------------------------------------------------------

//! \brief          Defines the library enumerated error codes.
//!                 These will be applied as bit masks in the error handler
//!
typedef enum  
{
    dcl_none = 0U,                         //!< No error
    dcl_param_range_err =   (1U << 0),     //!< Parameter range exceeded
    dcl_param_invalid_err = (1U << 1),     //!< Parameter not valid
    dcl_param_warn_err =    (1U << 2),     //!< Parameter warning
    dcl_input_range_err =   (1U << 3),     //!< Input range exceeded
    dcl_overflow_err =      (1U << 4),     //!< Numerical overflow
    dcl_underflow_err =     (1U << 5),     //!< Numerical underflow  
    dcl_controller_err =    (1U << 6),     //!< Controller operation not completed
    dcl_timing_err =        (1U << 7)      //!< Timing error
} dcl_error_codes;

//! \brief            Macro to clear stored error code in CSS
//!
//! \param[in] ptr    DCL controller object that contains css
//!
#define DCL_clearError(ptr)        ((ptr)->css->err = dcl_none)

//! \brief            Macro to set error code in CSS
//!
//! \param[in] ptr    DCL controller object that contains css
//! \param[in] code   enum dcl_error_codes
//!
#define DCL_setError(ptr,code)        ((ptr)->css->err |= code)

//! \brief            Macro to store line location of error in CSS
//!
//! \param[in] ptr    DCL controller object that contains css
//!
#define DCL_getErrorLine(ptr)        ((ptr)->css->err_line = ((ptr)->css->err) ? __LINE__ : 0)

//! \brief            Macro to store function location of error in CSS
//!
//! \param[in] ptr    DCL controller object that contains css
//!
#define DCL_getErrorFunc(ptr)        ((ptr)->css->err_func = ((ptr)->css->err) ? __FUNCTION__ : NULL)

//! \brief            Macro to store error info in CSS
//!
//! \param[in] ptr    DCL controller object that contains css
//!
#define DCL_getErrorInfo(ptr) \
{                             \
    DCL_getErrorLine(ptr);    \
    DCL_getErrorFunc(ptr);    \
}

//! \brief            Prototype for basic error handler
//!
//! \param[in] ptr    DCL controller object that contains css
//!
#define DCL_runErrorHandler(ptr) \
{                                \
    if(dcl_none != ptr->css->err)\
    {                            \
        DCL_setBreakPoint();     \
        DCL_clearError(ptr);     \
    }                            \
}

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_ERROR_H_
