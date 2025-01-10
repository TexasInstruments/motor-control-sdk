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

#include "mtpa.h"

// Comment this line if user defined "user.h" file isn't available to overwrite the 
// default values of Ls_d_h and Ls_q_H
#include "user.h"

#ifndef USER_MOTOR_Ls_d_H 
// the stator inductance value of the motor in the direct direction, in H
#define USER_MOTOR_Ls_d_H (0.000190442806f)
#endif

#ifndef USER_MOTOR_Ls_q_H
// the stator inductance value of the motor in the quadrature direction, in H
#define USER_MOTOR_Ls_q_H (0.000190442806f)
#endif  

#define MOTOR_Ls_d_H    USER_MOTOR_Ls_d_H
#define MOTOR_Ls_q_H    USER_MOTOR_Ls_q_H

//! \Defines the Ld array according to the specification of the motor,
//!          the user has to ask the motor manufacture to get the data
const float32_t MTPA_Ld_tableData_H[MTPA_LUT_INDEX_LD_MAX + 1] =               \
{                                                                              \
    MOTOR_Ls_d_H * 1.025f,       /* index =  0, Is = 0.00(A) */                \
    MOTOR_Ls_d_H * 1.020f,       /* index =  1, Is = 0.50(A) */                \
    MOTOR_Ls_d_H * 1.015f,       /* index =  2, Is = 1.00(A) */                \
    MOTOR_Ls_d_H * 1.010f,       /* index =  3, Is = 1.50(A) */                \
    MOTOR_Ls_d_H * 1.005f,       /* index =  4, Is = 2.00(A) */                \
    MOTOR_Ls_d_H * 1.000f,       /* index =  5, Is = 2.50(A) */                \
    MOTOR_Ls_d_H * 0.995f,       /* index =  6, Is = 3.00(A) */                \
    MOTOR_Ls_d_H * 0.990f,       /* index =  7, Is = 3.50(A) */                \
    MOTOR_Ls_d_H * 0.985f,       /* index =  8, Is = 4.00(A) */                \
    MOTOR_Ls_d_H * 0.980f,       /* index =  9, Is = 4.50(A) */                \
    MOTOR_Ls_d_H * 0.975f,       /* index = 10, Is = 5.00(A) */                \
    MOTOR_Ls_d_H * 0.970f,       /* index = 11, Is = 5.50(A) */                \
    MOTOR_Ls_d_H * 0.965f,       /* index = 12, Is = 6.00(A) */                \
    MOTOR_Ls_d_H * 0.960f,       /* index = 13, Is = 6.50(A) */                \
    MOTOR_Ls_d_H * 0.955f,       /* index = 14, Is = 7.00(A) */                \
    MOTOR_Ls_d_H * 0.950f,       /* index = 15, Is = 7.50(A) */                \
    MOTOR_Ls_d_H * 0.945f,       /* index = 16, Is = 8.00(A) */                \
    MOTOR_Ls_d_H * 0.940f,       /* index = 17, Is = 8.50(A) */                \
    MOTOR_Ls_d_H * 0.935f,       /* index = 18, Is = 9.00(A) */                \
    MOTOR_Ls_d_H * 0.930f,       /* index = 19, Is = 9.50(A) */                \
    MOTOR_Ls_d_H * 0.925f,       /* index = 20, Is = 10.0(A) */                \
};

//! \Defines the Lq array according to the specification of the motor,
//!          the user has to ask the motor manufacture to get the data
const float32_t MTPA_Lq_tableData_H[MTPA_LUT_INDEX_LQ_MAX + 1] =               \
{                                                                              \
    MOTOR_Ls_q_H * 1.050f,        /* index =  0, Is = 0.00(A) */               \
    MOTOR_Ls_q_H * 1.040f,        /* index =  1, Is = 0.50(A) */               \
    MOTOR_Ls_q_H * 1.030f,        /* index =  2, Is = 1.00(A) */               \
    MOTOR_Ls_q_H * 1.020f,        /* index =  3, Is = 1.50(A) */               \
    MOTOR_Ls_q_H * 1.010f,        /* index =  4, Is = 2.00(A) */               \
    MOTOR_Ls_q_H * 1.000f,        /* index =  5, Is = 2.50(A) */               \
    MOTOR_Ls_q_H * 0.990f,        /* index =  6, Is = 3.00(A) */               \
    MOTOR_Ls_q_H * 0.980f,        /* index =  7, Is = 3.50(A) */               \
    MOTOR_Ls_q_H * 0.970f,        /* index =  8, Is = 4.00(A) */               \
    MOTOR_Ls_q_H * 0.960f,        /* index =  9, Is = 4.50(A) */               \
    MOTOR_Ls_q_H * 0.950f,        /* index = 10, Is = 5.00(A) */               \
    MOTOR_Ls_q_H * 0.940f,        /* index = 11, Is = 5.50(A) */               \
    MOTOR_Ls_q_H * 0.930f,        /* index = 12, Is = 6.00(A) */               \
    MOTOR_Ls_q_H * 0.920f,        /* index = 13, Is = 6.50(A) */               \
    MOTOR_Ls_q_H * 0.910f,        /* index = 14, Is = 7.00(A) */               \
    MOTOR_Ls_q_H * 0.900f,        /* index = 15, Is = 7.50(A) */               \
    MOTOR_Ls_q_H * 0.890f,        /* index = 16, Is = 8.00(A) */               \
    MOTOR_Ls_q_H * 0.880f,        /* index = 17, Is = 8.50(A) */               \
    MOTOR_Ls_q_H * 0.870f,        /* index = 18, Is = 9.00(A) */               \
    MOTOR_Ls_q_H * 0.860f,        /* index = 19, Is = 9.50(A) */               \
    MOTOR_Ls_q_H * 0.850f,        /* index = 20, Is = 10.0(A) */               \
};

// ****************************************************************************
//
// MTPA_init
//
// ****************************************************************************
MTPA_Handle MTPA_init(void *pMemory, const size_t numBytes)
{
    MTPA_Handle handle;

    if((int32_t)numBytes < (int32_t)sizeof(MTPA_Obj))
    {
        return((MTPA_Handle)0x0);
    }

    //
    // assign the handle
    //
    handle = (MTPA_Handle)pMemory;

    return(handle);
} // end of MTPA_init() function


// ****************************************************************************
//
// MTPA_computeParameters
//
// ****************************************************************************
void MTPA_computeParameters(MTPA_Handle handle,
                               const float32_t Ls_d_H,
                               const float32_t Ls_q_H,
                               const float32_t flux_Wb)
{
    MTPA_Obj *obj = (MTPA_Obj *)handle;

    // Calculates the motor constant for mtpa
    if(Ls_q_H != Ls_d_H)
    {
        obj->kconst = 0.25 * (flux_Wb /
                (Ls_q_H - Ls_d_H));
    }
    else
    {
        obj->kconst = 0.0;
    }

    return;
} // end of MTPA_computeParameters() function

//
// end of file
//
