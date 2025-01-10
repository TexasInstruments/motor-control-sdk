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
 
#ifndef PI_TEST_H
#define PI_TEST_H

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
#define PI_TESTCASE     1     //!< Defines testcase, tests 1-3 is provided            
#define DATA_LENGTH     1601  //!< Size of total data inputs
#define NUM_ELEMENTS    400   //!< The number of samples ran, cannot exceed DATA_LENGTH

void dcl_pi_main(void *args);
typedef float32_t (*DCL_PI_FUNC)(DCL_PI *p, float32_t rk, float32_t yk);
int PI_runTest(DCL_PI *ctrl_handle, DCL_PI_FUNC dcl_pi_func);


//
//  Controller initialization
// 
//! Method 1: 
//! Direct assignments
//!
DCL_PI *pi_controller = &(DCL_PI)
{
    .Kp  = 2.5f,
    .Ki  = 0.01f,
    .Umax = 2.0f,
    .Umin = -2.0f,
    .Imax = 1.0f,
    .Imin = -1.0f,
    PI_INT_DEFAULTS // macro for default values of internal attributes
};

//! Method 2:
//! Initalizing with input controller parameters
//!
/*
float32_t kp = 2.5f, ki = 0.01f, umax = 1.0f, umin = -1.0f;
DCL_PI *pi_controller = DCL_initPIasParam(2.5f,0.01f,1.0f,-1.0f); // (kp,ki,umax,umin)
*/

//
//  Macros for selecting testvector based on PI_TESTCASE 
//
#define _LSTR(x) #x
#define __LSTR(x) _LSTR(x)
#define PI_RK_DATA(x)  __LSTR(data/PI_rk-x.dat) 
#define PI_YK_DATA(x)  __LSTR(data/PI_yk-x.dat) 
#define PI_CTL_DATA(x) __LSTR(data/PI_ctl-x.dat) 

//
// Allocate data sections and initialize data
//
const float32_t __attribute__((weak)) rk_buffer[DATA_LENGTH] =
{
#include PI_RK_DATA(PI_TESTCASE)
};

const float32_t __attribute__((weak)) yk_buffer[DATA_LENGTH] =
{
#include PI_YK_DATA(PI_TESTCASE)
};

const float32_t __attribute__((weak)) ctl_buffer[DATA_LENGTH] =
{
#include PI_CTL_DATA(PI_TESTCASE)
};

static float32_t out_buffer[DATA_LENGTH];

#ifdef __cplusplus
}
#endif // extern "C"

#endif
