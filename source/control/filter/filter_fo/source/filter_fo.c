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

#include "filter_fo.h"

//*****************************************************************************
//
// FILTER_FO_getDenCoeffs
//
//*****************************************************************************
void
FILTER_FO_getDenCoeffs(FILTER_FO_Handle handle, float32_t *pa1)
{
    *pa1 = handle->a1;

    return;
} // end of FILTER_FO_getDenCoeffs() function

//*****************************************************************************
//
// FILTER_FO_getInitialConditions
//
//*****************************************************************************
void
FILTER_FO_getInitialConditions(FILTER_FO_Handle handle, float32_t *px1,
                               float32_t *py1)
{
    FILTER_FO_Obj *obj = (FILTER_FO_Obj *)handle;

    *px1 = obj->d1;

    *py1 = obj->d2;

    return;
} // end of FILTER_FO_getInitialConditions() function

//*****************************************************************************
//
// FILTER_FO_getNumCoeffs
//
//*****************************************************************************
void
FILTER_FO_getNumCoeffs(FILTER_FO_Handle handle, float32_t *pb0, float32_t *pb1)
{
    *pb0 = handle->b0;
    *pb1 = handle->b1;

    return;
} // end of FILTER_FO_getNumCoeffs() function

//*****************************************************************************
//
// FILTER_FO_init
//
//*****************************************************************************
FILTER_FO_Handle FILTER_FO_init(void *pMemory,
                                const size_t numBytes)
{
    FILTER_FO_Handle handle;

    if((int32_t)numBytes < (int32_t)sizeof(FILTER_FO_Obj))
    {
        return((FILTER_FO_Handle)0x0);
    }

    //
    // Assign the handle
    //
    handle = (FILTER_FO_Handle)pMemory;

    *handle = (DCL_DF11) DF11_DEFAULTS;

    return(handle);
} // end of FILTER_FO_init() function

//*****************************************************************************
//
// FILTER_FO_setDenCoeffs
//
//*****************************************************************************
void
FILTER_FO_setDenCoeffs(FILTER_FO_Handle handle, const float32_t a1)
{
    handle->a1 = a1;

    return;
} // end of FILTER_FO_setDenCoeffs() function

//*****************************************************************************
//
// FILTER_FO_setInitialConditions
//
//*****************************************************************************
void
FILTER_FO_setInitialConditions(FILTER_FO_Handle handle, const float32_t x1,
                               const float32_t y1)
{
    handle->d1 = x1;

    handle->d2 = y1;

    return;
} // end of FILTER_FO_setInitialConditions() function

//*****************************************************************************
//
// FILTER_FO_setNumCoeffs
//
//*****************************************************************************
void
FILTER_FO_setNumCoeffs(FILTER_FO_Handle handle, const float32_t b0,
                       const float32_t b1)
{
    handle->b0 = b0;
    handle->b1 = b1;

    return;
} // end of FILTER_FO_setNumCoeffs() function

// end of file
