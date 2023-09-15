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
 
#ifndef _DCL_STABILITY_H_
#define _DCL_STABILITY_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_stability.h
 *  \brief      Defines polynomial stability functions
 */

//--- Polynomial stability functions -----------------------------------------

//! \brief          Determines stability of a first order real polynomial
//!                 P(z) = z + a1
//!
//! \param[in] a1   Coefficient a1
//! \return         'true' if the root has magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStablePn1(float32_t a1)
{
    return ((a1 * a1) < 1.0f);
}

//! \brief          Determines stability of a second order polynomial with real coefficients
//!                 P(z) = a0 z^2 + a1 z + a2
//!
//! \param[in] a0   Second order coefficient a1
//! \param[in] a1   First order coefficient a1
//! \param[in] a2   Coefficient a2
//! \return         'true' if both roots have magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStablePn2(float32_t a0, float32_t a1, float32_t a2)
{
    float32_t b0, b1, c0;

    b0 = a0 - a2 * a2 / a0;
    b1 = a1 - a1 * a2 / a0;
    c0 = b0 - b1 * b1 / b0;

    return ((a0 > 0.0f) && (b0 > 0.0f) && (c0 > 0.0f));
}

//! \brief          Determines stability of a third order polynomial with real coefficients
//!                 P(z) = a0 z^3 + a1 z^2 + a2 z + a3
//!
//! \param[in] a0   Third order coefficient a1
//! \param[in] a1   Second order coefficient a1
//! \param[in] a2   First order coefficient a2
//! \param[in] a3   Coefficient a3
//! \return         'true' if all roots have magnitude less than 1, 'false' otherwise
//!
_DCL_CODE_ACCESS
bool DCL_isStablePn3(float32_t a0, float32_t a1, float32_t a2, float32_t a3)
{
    float32_t b0, b1, b2, c0, c1, d0;

    b0 = a0 - a3 * a3 / a0;
    b1 = a1 - a2 * a3 / a0;
    b2 = a2 - a1 * a3 / a0;
    c0 = b0 - b2 * b2 / b0;
    c1 = b1 - b1 * b2 / b0;
    d0 = c0 - c1 * c1 / c0;

    return ((a0 > 0.0f) && (b0 > 0.0f) && (c0 > 0.0f) && (d0 > 0.0f));
}

/*
//! \brief         Determines stability of a ZPK3 representation by checking pole magnitude
//!                Located in common/dcl_zpk3.h
extern bool DCL_isStableZpk3(DCL_ZPK3 *q);
*/

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_STABILITY_H_
