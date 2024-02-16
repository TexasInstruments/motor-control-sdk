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

#ifndef _PARK_H_
#define _PARK_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \addtogroup TRANSFORMS_API_MODULE
 *  @{
 *
 *  \file           park.h
 *  \brief          Contains alpha/beta/0 to d/q/0 transform implementation
 */

#include <stdint.h>

typedef float float32_t;

//! \brief     Runs the Park transform module
//! \param[in] sinTh    Sine value in radians
//! \param[in] cosTh    Cosine value in radians
//! \param[in] inIalpha Input current value in alpha-axis
//! \param[in] inIbeta  Input current value in beta-axis
//! \param[in] pId      Output pointer to voltage value in d-axis
//! \param[in] pIq      Output pointer to voltage value in q-axis
//!
static __attribute__((always_inline)) 
void PARK_run(const float32_t sinTh, const float32_t cosTh, const float32_t inIalpha, const float32_t inIbeta, \
            float32_t* pId, float32_t* pIq)
{
    *pId = (inIalpha * cosTh) + (inIbeta * sinTh);
    *pIq = (inIbeta * cosTh) - (inIalpha * sinTh);
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _PARK_H_
