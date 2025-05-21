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
 
#ifndef TRANSFORMS_TEST
#define TRANSFORMS_TEST

#ifdef __cplusplus
extern "C" {
#endif

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "ti_arm_trig.h"
#include "clarke.h"
#include "park.h"
#include "ipark.h"
#include "svgen.h"

#include <stdbool.h>
#include <math.h>


typedef struct {
    /* Provided Input */
    float32_t  inA;
    float32_t  inB;
    float32_t  inC;
    /* Reference Output */
    float32_t  refAlpha; // expected value for PARK output
    float32_t  refBeta;  // expected value for PARK output
    float32_t  refA;     // expected value for SVGEN output
    float32_t  refB;     // expected value for SVGEN output
    float32_t  refC;     // expected value for SVGEN output
} transform_test_t;

#define ERROR_TOL 1e-05f
#define exceeds_tolerance(x) x > ERROR_TOL
#define is_different(x,y) exceeds_tolerance(fabsf(x - y))

static float32_t phasorOffset = M_PI_2;

static transform_test_t testParams[] =
{
    {
        1.0f, -0.5f,-0.5f,
        -0.0f, -1.0f,
        0.75f, -0.75f, -0.75f
    },
    {
        0.75f, -0.158494f, -0.591506f,
        0.250000f, -0.750000f,
        0.670753f, -0.237741f, -0.670753f
    },
    {
        0.5f, 0.183013,-0.683013,
        0.5f, -0.5f,
        0.591506f, 0.274520f, -0.591506f
    },
    {
        0.25f,0.524519f,-0.774519f,
        0.75f, -0.25f,
        0.375f, 0.649519f, -0.649519f
    },
    {
        0.0f,0.866025f,-0.866025f,
        1.0f, 0.0f,
        0.0f, 0.866025f, -0.866025f
    },
    {
        -0.25f,0.774519f,-0.524519f,
        0.75f, 0.25f,
        -0.375000f, 0.649519f, -0.649519f
    },
    {
        -0.5f,0.683013f,-0.183013f,
        0.5f, 0.5f,
        -0.591506f, 0.591506f, -0.274519f
    },
    {
        -0.75f,0.591506f,0.158494f,
        0.25f, 0.75f,
        -0.670753f, 0.670753f, 0.237741f
    },
    {
        -1.0f,0.5f,0.5f,
        0.0f, 1.0f,
        -0.75f, 0.75f, 0.75f
    },
    {
        -0.75f,0.591506f,0.158494f,
        0.25f,0.75f,
        -0.670753f, 0.670753f, 0.237741f
    },
    {
        -0.500000f, 0.683013f, -0.183013f,
        0.5f,0.5f,
        -0.591506f, 0.591506f, -0.274519f
    },
    {
        -0.250000f, -0.524519f, 0.774519f,
        -0.75f,0.25f,
        -0.375000f, -0.649519f, 0.649519f
    },
    {
        0.000000f, -0.866025f, 0.866025f,
        -1.0f,0.0f,
        0.0f, -0.866025f, 0.866025f
    },
    {
        0.250000f, -0.774519f, 0.524519f,
        -0.75f,-0.25f,
        0.375f, -0.649519, 0.649519f
    },
    {
        0.500000f, -0.683013f, 0.183013f,
        -0.5f,-0.5f,
        0.591506f, -0.591506f, 0.274519f
    },
    {
        0.750000f, -0.591506f, -0.158494f,
        -0.25f,-0.75f,
        0.670753f, -0.670753f, -0.237741f
    },
};


#ifdef __cplusplus
}
#endif // extern "C"

#endif
