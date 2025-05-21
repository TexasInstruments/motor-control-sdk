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
 
// **************************************************************************
//
// the includes
//
#include "angle_gen.h"



//*****************************************************************************
//
// ANGLE_GEN_init
//
//*****************************************************************************
ANGLE_GEN_Handle ANGLE_GEN_init(void *pMemory, const size_t numBytes)
{
    ANGLE_GEN_Handle handle;

    if(numBytes < sizeof(ANGLE_GEN_Obj))
      return((ANGLE_GEN_Handle)0x0);

    //
    // assign the handle
    //
    handle = (ANGLE_GEN_Handle)pMemory;

    return(handle);
} // end of ANGLE_GEN_init() function

// Sets up parameters for angle generation
void ANGLE_GEN_setParams(ANGLE_GEN_Handle handle, float32_t ctrlPeriod_sec)
{
    ANGLE_GEN_Obj *obj = (ANGLE_GEN_Obj *)handle;

    obj->angleDeltaFactor = ctrlPeriod_sec * MATH_TWO_PI;
    obj->angleDelta_rad = 0.0;
    obj->angle_rad = 0.0;

    return;
} // end of ANGLE_COMP_setParams() function

// end of the file

