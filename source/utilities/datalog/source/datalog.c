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
//#############################################################################

//! \file   datalog.c
//! \brief  These functions define the data logging (DATALOG) module routines
//!

// **************************************************************************
// the includes
#include <datalog.h>
#include <stdlib.h>

// **************************************************************************
// the defines

// **************************************************************************
// the globals

__attribute__((__section__(".datalog_data"))) DATALOG_Obj datalog;
DATALOG_Handle datalogHandle;       //!< the handle for the Datalog object


// **************************************************************************
// the functions

DATALOG_Handle DATALOG_init(void *pMemory, const size_t numBytes, Trigger_type trigger, float32_t trig_value, int32_t scale)
{
    DATALOG_Handle handle;

    if(numBytes < sizeof(DATALOG_Obj))
    {
        return((DATALOG_Handle)NULL);
    }

    // assign the handle
    handle = (DATALOG_Handle)pMemory;

    DATALOG_Obj *obj = (DATALOG_Obj *)handle;
    obj->size = DATA_LOG_BUFF_SIZE;
    obj->cntr = 0;
    obj->skipCount = 0;
    obj->trigger = trigger;
    obj->preScalar = scale;
    obj->status = 1;
    obj->trigValue = trig_value;

    return(handle);
} // end of DATALOG_init() function

// end of file
