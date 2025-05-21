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
 
// **************************************************************************
// the includes
#include "encoder.h"


// **************************************************************************
// the functions

ENC_Handle ENC_init(void *pMemory, const size_t numBytes)
{
	ENC_Handle handle;

	if(numBytes < sizeof(ENC_Obj))
	{
		return((ENC_Handle)0x0);
	}

	// assign the handle
	handle = (ENC_Handle)pMemory;

	return(handle);
}

//------------------------------------------------------------------------------
void ENC_setParams(ENC_Handle handle, const USER_Params *pUserParams)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->Ts_sec = pUserParams->ctrlPeriod_sec;

    obj->polePairs = pUserParams->motor_numPolePairs;
    obj->encLines = pUserParams->motor_numEncSlots;

    obj->mechanicalScaler = 0.25f / obj->encLines;

    obj->encState = ENC_IDLE;

    return;
}

//----------------------------------------------------------------
//
// end of file
