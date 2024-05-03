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

#ifndef _DCL_AUX_H_
#define _DCL_AUX_H_

#ifdef __cplusplus
extern "C" {
#endif     

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_aux.h
 *  \brief      Defines all the auxiliary macros for DCL
 */

#ifndef NULL
#define NULL    0
#endif

//! \brief          Local definitions of the mathematical constant pi
//!
#define CONST_PI        3.14159265358979323846f
#define CONST_2PI       2.0f * CONST_PI
#define CONST_PI_F64    3.1415926535897932384626433832795028841971693993751058209749445923078164062L
#define CONST_2PI_F64   2.0L * CONST_PI_F64

//! \brief          Define the acceptable FPU numerical tolerances
//!
#define DCL_FPU32_TOL       1.0e-06f
#define DCL_FPU64_TOL       1.0e-10L

//! \brief          Define the default control period in seconds
//!
#define DCL_DEFAULT_PERIOD_S    100.0e-06f
#define DCL_DEFAULT_PERIODF64_S 100.0e-06L

//! \brief          Determines numerical proximity to specified value
//!
#define DCL_isValue(x,y)        (((x < (y + DCL_FPU32_TOL)) && (x > (y - DCL_FPU32_TOL))) ? 1U : 0U)
#define DCL_isValueF64(x,y)     (((x < (y + DCL_FPU64_TOL)) && (x > (y - DCL_FPU64_TOL))) ? 1U : 0U)

//! \brief          Determines floating point numerical proximity to zero
//!
#define DCL_isZero(x)           DCL_isValue(x,0.0f)
#define DCL_isZeroF64(x)        DCL_isValueF64(x,0.0L)

//! \brief          Returns a random floating point result between -1.0 and +1.0
//!                 where 'a' is the multiplier in single or double precision. 
//!                 Useful for initialization of arrays and matrices during test.
//!
//! \code
//!                 float32_t s = DCL_rand(1.0f);
//! \endcode
#define DCL_rand(a)         (a * ((float32_t) rand() / (float32_t) (RAND_MAX >> 1) - 1.0f))
#define DCL_randF64(a)      (a * ((float64_t) rand() / (float64_t) (RAND_MAX >> 1) - 1.0L))

//! \brief          Defines the lower limit on derivative filter coefficient c2
//!                 in order for fc to lie below the Nyquist frequency
//!
#define DCL_c2Limit         ((2.0f - CONST_PI) / (2.0f + CONST_PI))
#define DCL_c2LimitF64      ((2.0L - CONST_PI_F64) / (2.0L + CONST_PI_F64))

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_AUX_H_
