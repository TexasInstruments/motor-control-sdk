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

#ifndef MATH_TYPES_H
#define MATH_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup MATH_API_MODULE APIs
 *  \ingroup  MOTOR_CONTROL_API
 *
 *  Here is the list of function APIs
 *  @{
 *
 *  \file           math_types.h
 *  \brief          Contains some common arthmetic operations and math constants
 */

//#include_next <math.h>
#include <stdint.h>

#ifndef IEEE754_TYPES
#define IEEE754_TYPES
typedef float         float32_t;
typedef double        float64_t;
#endif // IEEE754_TYPES

//*****************************************************************************
//
//! \brief Defines conversion scale factor from N*m to lb*in
//
//*****************************************************************************
#define MATH_Nm_TO_lbin_SF        ((float32_t)(8.8507457913f))

//*****************************************************************************
//
//! \brief Defines 2/3
//
//*****************************************************************************
#define MATH_TWO_OVER_THREE       ((float32_t)(0.6666666666666666666666666667f))

//*****************************************************************************
//
//! \brief Defines 1/3
//
//*****************************************************************************
#define MATH_ONE_OVER_THREE       ((float32_t)(0.3333333333333333333333333333f))

//*****************************************************************************
//
//! \brief Defines 1/(pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_PI          ((float32_t)(0.318309886183791f))


//*****************************************************************************
//
//! \brief Defines sqrt(2)
//
//*****************************************************************************
#define MATH_SQRT_TWO             ((float32_t)(1.414213562373095f))


//*****************************************************************************
//
//! \brief Defines sqrt(3)
//
//*****************************************************************************
#define MATH_SQRT_THREE           ((float32_t)(1.73205080756887f))

//*****************************************************************************
//
//! \brief Defines 1/sqrt(3)
//
//*****************************************************************************
#define MATH_ONE_OVER_SQRT_THREE  ((float32_t)(0.5773502691896257645091487805f))

//*****************************************************************************
//
//! \brief Defines 1/(4*pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_FOUR_PI     ((float32_t)(0.07957747154594767f))

//*****************************************************************************
//
//! \brief Defines 1/(2*pi)
//
//*****************************************************************************
#define MATH_ONE_OVER_TWO_PI      ((float32_t) (0.1591549430918954f))

//*****************************************************************************
//
//! \brief Defines pi
//
//*****************************************************************************
#define MATH_PI                   ((float32_t)(3.1415926535897932384626433832f))

//*****************************************************************************
//
//! \brief Defines pi per unit
//
//*****************************************************************************
#define MATH_PI_PU                ((float32_t)(0.5f))

//*****************************************************************************
//
//! \brief Defines 2*pi
//
//*****************************************************************************
#define MATH_TWO_PI               ((float32_t)(6.283185307179586f))

//*****************************************************************************
//
//! \brief Defines 2*pi per unit
//
//*****************************************************************************
#define MATH_TWO_PI_PU            ((float32_t)(1.0f))

//*****************************************************************************
//
//! \brief Defines 4*pi
//
//*****************************************************************************
#define MATH_FOUR_PI               ((float32_t)(12.56637061435917f))

//*****************************************************************************
//
//! \brief Defines 4*pi per unit
//
//*****************************************************************************
#define MATH_FOUR_PI_PU            ((float32_t)(2.0f))

//*****************************************************************************
//
//! \brief Defines pi/2
//
//*****************************************************************************
#define MATH_PI_OVER_TWO           ((float32_t)(1.570796326794897f))

//*****************************************************************************
//
//! \brief Defines pi/2 per unit
//
//*****************************************************************************
#define MATH_PI_OVER_TWO_PU        ((float32_t)(0.25f))


//*****************************************************************************
//
//! \brief Defines pi/3
//
//*****************************************************************************
#define MATH_PI_OVER_THREE         ((float32_t)(1.047197551196598f))

//*****************************************************************************
//
//! \brief Defines pi/4
//
//*****************************************************************************
#define MATH_PI_OVER_FOUR          ((float32_t)(0.785398163397448f))

//*****************************************************************************
//
//! \brief Defines pi/4 per unit
//
//*****************************************************************************
#define MATH_PI_OVER_FOUR_PU        ((float32_t)(0.125f))

//*****************************************************************************
//
//! \brief Defines 6*pi
//
//*****************************************************************************
#define MATH_SIX_PI               ((float32_t)(18.8495559215f))

//*****************************************************************************
//
//! \brief Defines sqrt(3)/2
//
//*****************************************************************************
#define MATH_SQRTTHREE_OVER_TWO       ((float32_t)(0.8660254038f))

//*****************************************************************************
//
//! \brief Defines a two element vector
//
//*****************************************************************************
typedef struct _MATH_Vec2_
{
    float32_t value[2];
} MATH_Vec2;

typedef MATH_Vec2 MATH_vec2;

//*****************************************************************************
//
//! \brief Defines a three element vector
//
//*****************************************************************************
typedef struct _MATH_Vec3_
{
    float32_t value[3];
} MATH_Vec3;

typedef MATH_Vec3 MATH_vec3;

//*****************************************************************************
//
//! \brief Defines unsigned integer two element vector
//
//*****************************************************************************
typedef struct _MATH_ui_Vec2_
{
    uint32_t value[2];
} MATH_ui_Vec2, MATH_ui_vec2;

//*****************************************************************************
//
//! \brief Defines unsigned integer three element vector
//
//*****************************************************************************
typedef struct _MATH_ui_Vec3_
{
    uint32_t value[3];
} MATH_ui_Vec3, MATH_ui_vec3;

//*****************************************************************************
//
//! \brief     Finds the absolute value
//!
//! \param[in] in   The input value
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_abs(const float32_t in)
{
    float32_t out = in;

    if(in < (float32_t)0.0f)
    {
        out = -in;
    }

    return(out);
} // end of MATH_abs() function


//*****************************************************************************
//
//! \brief     Finds the maximum value between the twp input values
//!
//! \param[in] in   The input values
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_max(const float32_t in1, const float32_t in2)
{
    float32_t out = in1;


    if(in1 < in2)
    {
        out = in2;
    }

    return(out);
} // end of MATH_max() function


//*****************************************************************************
//
//! \brief     Finds the minimum value between the twp input values
//!
//! \param[in] in   The input values
//!
//! \return    The absolute value
//
//*****************************************************************************
static inline
float32_t MATH_min(const float32_t in1, const float32_t in2)
{
    float32_t out = in1;


    if(in1 > in2)
    {
      out = in2;
    }

    return(out);
} // end of MATH_min() function

//*****************************************************************************
//
//! \brief     Increments an angle value and handles wrap-around
//!
//! \param[in] angle_rad       The angle value, rad
//!
//! \param[in] angleDelta_rad  The angle increment value, rad
//!
//! \return    The incremented angle value, rad
//
//*****************************************************************************


static inline float32_t
MATH_incrAngle(const float32_t angle_rad, const float32_t angleDelta_rad)
{
    float32_t angleNew_rad;

    //
    // Increment the angle
    //
    angleNew_rad = angle_rad + angleDelta_rad;

    angleNew_rad = (angleNew_rad > MATH_PI)  ? angleNew_rad - MATH_TWO_PI : angleNew_rad;
    angleNew_rad = (angleNew_rad < -MATH_PI) ? angleNew_rad + MATH_TWO_PI : angleNew_rad;

    return(angleNew_rad);
} // end of MATH_incrAngle() function

//*****************************************************************************
//
//! \brief     Saturates the input value between the minimum and maximum values
//!
//! \param[in] in   The input value
//!
//! \param[in] max  The maximum value allowed
//!
//! \param[in] min  The minimum value allowed
//!
//! \return    The saturated value
//
//*****************************************************************************
static inline float32_t
MATH_sat(const float32_t in, const float32_t max, const float32_t min)
{
    float32_t out = in;

    out = (out > max) ? max : out;
    out = (out < min) ? min : out;

    return(out);
} // end of MATH_sat() function

static inline
float32_t MATH_sign(const float32_t in)
{
    float32_t out = 1.0f;


    if(in < 0.0f)
    {
      out = -1.0f;
    }

    return(out);
} // end of MATH_sign() function

/** @} */

#ifdef __cplusplus
}
#endif

#endif // MATH_TYPES_H
