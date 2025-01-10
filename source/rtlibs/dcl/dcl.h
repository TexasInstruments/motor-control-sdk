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

#ifndef _DCL_H_
#define _DCL_H_

#ifdef __cplusplus
extern "C" {
#endif

 /**
 * \defgroup RTLIBS_API APIs for Real Time Libraries
 *
 * This module contains APIs for real time libraries in this SDK.
 */

/**
 *  \defgroup DCL_API_MODULE APIs for Digital Control Library
 *  \ingroup  RTLIBS_API
 *  
 *  Here is the list of APIs used for Digital Control Library
 *  @{
 *
 *  \file     dcl.h
 *  \brief    Top level header that contains all collections of Digital Controller Library functions
 *  
 *  \details  To use this library, simply use
 *  \code     #include "dcl.h" \endcode
 *            while making sure the path to this file is added as a compiler include path
 */

//! \brief          Library version number formatted for numerical comparison
//!                 v4.02.00.00
#define DCL_VERSION 4020000

//! \brief          Enable voltaile flag for dcl strcutures
//!                 Disabled by default
//#define DCL_VOLATILE_ENABLED

//! \brief          Build the library with error handling enabled
//!                 Performs error checks on various update and
//!                 stability functions
//!                 Disabled by default
//#define DCL_ERROR_HANDLING_ENABLED

//! \brief          Build the library with test points enabled
//!                 during arthmetic operations
//!                 Disabled by default
//#define DCL_TESTPOINTS_ENABLED

/* utilities */
#include "misc/dcl_error.h"
#include "misc/dcl_fdlog.h"
#include "misc/dcl_mlog.h"
#include "misc/dcl_refgen.h"
#include "misc/dcl_tcm.h"

/* 32bit arithmetic */
#include "pi/dcl_pi.h"
#include "pi/dcl_pi2.h"
#include "pid/dcl_pid.h"
#include "pid/dcl_nlpid.h"

#include "df/dcl_df11.h"
#include "df/dcl_df12.h"
#include "df/dcl_df13.h"
#include "df/dcl_df22.h"
#include "df/dcl_df23.h"

/* 64bit arithmetic */
#include "pid/dcl_pidf64.h"

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_H_
