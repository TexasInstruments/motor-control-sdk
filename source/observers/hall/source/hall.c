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
// the includes
#include "hall.h"

// **************************************************************************
// the defines

// **************************************************************************
// the globals


// **************************************************************************
// the functions

//------------------------------------------------------------------------------
HALL_Handle HALL_init(void *pMemory, const size_t numBytes)
{
	HALL_Handle handle;

	if(numBytes < sizeof(HALL_Obj))
	{
		return((HALL_Handle)0x0);
	}

	// assign the handle
	handle = (HALL_Handle)pMemory;

	return(handle);
} // end of HALL_init() function

//------------------------------------------------------------------------------
void HALL_setParams(HALL_Handle handle, const USER_Params *pUserParams)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    obj->capScaler = 3.0f * 1000000.0f * pUserParams->systemFreq_MHz;
    obj->pwmScaler = pUserParams->ctrlFreq_Hz / 2.0f;
    obj->thetaDelta_rad = MATH_TWO_PI / 36.0f;
    obj->timeCountMax = pUserParams->ctrlFreq_Hz / 1.5f;       // 0.25Hz
    obj->speedSwitch_Hz = 50.0f;

    obj->hallPrev[0] = 4;
    obj->hallPrev[1] = 3;
    obj->hallPrev[2] = 6;
    obj->hallPrev[3] = 2;
    obj->hallPrev[4] = 5;
    obj->hallPrev[5] = 1;
    obj->hallPrev[6] = 4;

#ifdef HALL_CAL
    obj->hallIndexFlag = 0;       //
#endif  //HALL_CAL

    return;
}

//------------------------------------------------------------------------------
void HALL_setAngleBuf(HALL_Handle handle, const float32_t *ptrAngleBuf)
{
    HALL_Obj *obj = (HALL_Obj *)handle;

    uint16_t  cnt;

    for(cnt = 0; cnt < 7; cnt++)
    {
        obj->thetaBuff[cnt]  = *(ptrAngleBuf + cnt);
    }

    return;
}

//----------------------------------------------------------------

// end of file
