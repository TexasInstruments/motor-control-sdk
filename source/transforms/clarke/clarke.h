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

#ifndef _CLARKE_H_
#define _CLARKE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup TRANSFORMS_API_MODULE APIs for Transformation Algorithms
 *  \ingroup  MOTOR_CONTROL_API
 *
 *  Here is the list of motor transformation function APIs
 *  @{
 *
 *  \file           clarke.h
 *  \brief          Contains abc to alpha/beta/0 transform implementation
 */

#include <stdint.h>

#ifndef IEEE754_TYPES
#define IEEE754_TYPES
typedef float   float32_t;
typedef double  float64_t;
#endif // IEEE754_TYPES

#define ONE_OVER_THREE      0.33333333333333f
#define ONE_OVER_SQRT_THREE 0.57735026918963f

//! \brief     Runs the Clarke transform module for three inputs
//!            
//! \param[in] inIa     Input current value in a-axis
//! \param[in] inIb     Input current value in b-axis
//! \param[in] inIc     Input current value in c-axis
//! \param[in] pIalpha  Output pointer to current value in alpha-axis
//! \param[in] pIbeta   Output pointer to current value in beta-axis
//!
static __attribute__((always_inline)) 
void CLARKE_run_threeInput(const float32_t inIa, const float32_t inIb, const float32_t inIc, \
                        float32_t* pIalpha, float32_t* pIbeta)
{
    *pIalpha = ((2.0f * inIa) - (inIb + inIc)) * ONE_OVER_THREE;
    *pIbeta = (inIb - inIc) * ONE_OVER_SQRT_THREE;
}

//! \brief     Runs the Clarke transform module for two inputs
//!            
//! \param[in] inIa     Input current value in a-axis
//! \param[in] inIb     Input current value in b-axis
//! \param[in] pIalpha  Output pointer to current value in alpha-axis
//! \param[in] pIbeta   Output pointer to current value in beta-axis
//!
static __attribute__((always_inline)) 
void CLARKE_run_twoInput(const float32_t inIa, const float32_t inIb, \
                        float32_t* pIalpha, float32_t* pIbeta)
{
    *pIalpha = inIa;
    *pIbeta = (inIa + (2.0f * inIb)) * ONE_OVER_SQRT_THREE;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _CLARKE_H_

