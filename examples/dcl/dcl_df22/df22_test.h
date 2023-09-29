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

#ifndef DF22_TEST_H
#define DF22_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <dcl.h> 

//
//  User configurable values
//
#define DF22_TESTCASE   1     //!< Defines testcase, tests 1-3 is provided   
#define DATA_LENGTH     1601  //!< Size of total data inputs
#define NUM_ELEMENTS    400   //!< The number of samples ran, cannot exceed DATA_LENGTH

void dcl_df22_main(void *args);
int DF22_runTest(DCL_DF22 *ctrl_handle);

//
//  Controller initialization
// 
//! Method 1: 
//! Direct assignments
//!
DCL_DF22 *df22_controller = &(DCL_DF22)
{
    .b0 = 9.8549f,
    .b1 = -19.4165f,
    .b2 = 9.5636f,
    .a1 = -1.9417f,
    .a2 = 0.9417f,
    DF22_INT_DEFAULTS  // macro for default values of internal attributes
};

//! Method 2:
//! Initalizing with input controller parameters
//!
/*
float32_t b0 = 9.8549f, b1 = -19.4165f, b2 = 9.5636f, a1 = -1.9417f, a2 = 0.9417f;
DCL_DF22 *df22_controller = DCL_initDF22asParam(9.8549f,-19.4165f,9.5636f,-1.9417f,0.9417f); // (b0,b1,b2,a1,a2)
*/


//
//  Macros for selecting testvector based on DF22_TESTCASE 
//
#define _LSTR(x) #x
#define __LSTR(x) _LSTR(x)
#define DF22_IN_DATA(x)  __LSTR(data/DF22_in-x.dat) 
#define DF22_CTL_DATA(x) __LSTR(data/DF22_ctl-x.dat) 

//
// Allocate data sections and initialize test vector
//
const float32_t __attribute__((weak)) in_buffer[DATA_LENGTH] =
{
#include DF22_IN_DATA(DF22_TESTCASE)
};

const float32_t __attribute__((weak)) ctl_buffer[DATA_LENGTH] =
{
#include DF22_CTL_DATA(DF22_TESTCASE)
};

static float32_t out_buffer[DATA_LENGTH];

#ifdef __cplusplus
}
#endif // extern "C"

#endif
