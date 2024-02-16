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
 
#ifndef _IPARK_H_
#define _IPARK_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \addtogroup TRANSFORMS_API_MODULE
 *  @{
 *
 *  \file   ipark.h
 *  \brief  Contains d/q/0 to alpha/beta/0 transform implementation
 */   

#include <stdint.h>

typedef float float32_t;

//! \brief     Runs the inverse Park transform module
//! \param[in] sinTh    Sine value in radian
//! \param[in] cosTh    Cosine value in radian
//! \param[in] inVd     Input voltage value in d-axis
//! \param[in] inVq     Input voltage value in q-axis
//! \param[in] pValpha  Output pointer to voltage value in alpha-axis
//! \param[in] pVbeta   Output pointer to voltage value in beta-axis
//!
static __attribute__((always_inline)) 
void IPARK_run(const float32_t sinTh, const float32_t cosTh, const float32_t inVd, const float32_t inVq, \
                float32_t* pValpha, float32_t* pVbeta)
{
    *pValpha = (inVd * cosTh) - (inVq * sinTh);
    *pVbeta = (inVq * cosTh) + (inVd * sinTh);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _IPARK_H_
