/*
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#ifndef TI_R5FMATH_TRIG_H_
#define TI_R5FMATH_TRIG_H_

#define TI_R5FMATH_PI               3.141592653589793238f
#define TI_R5FMATH_2PI              6.28318530717959f
#define TI_R5FMATH_PIOVER2          1.5707963267948966f
#define TI_R5FMATH_3PIOVER2         4.71238898f
#define TI_R5FMATH_PIOVER4          0.78539816f
#define TI_R5FMATH_ONEOVERPIOVER4   1.273239545f
#define TI_R5FMATH_PIOVER6          0.5235987755f
#define TI_R5FMATH_TANPIOVER6       0.577350269f
#define TI_R5FMATH_TANPIOVER12      0.26794919f
#define TI_R5FMATH_PIOVER180        0.017453292519943295769f

void ti_r5fmath_sincos(float inputRadians, float *PIconstants, float *sincosCoefficients, float *returnValues);

// globals
extern float ti_r5fmath_sincosCoef[8];
extern float ti_r5fmath_sincosPIconst[2];

#endif /* TI_R5FMATH_TRIG.H */
