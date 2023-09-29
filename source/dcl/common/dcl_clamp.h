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

#ifndef _DCL_CLAMP_H_
#define _DCL_CLAMP_H_     

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *
 *  \file       dcl_clamp.h
 *  \brief      Defines both single, double precision clamp function for saturation
 */
        
//--- Controller clamping functions ------------------------------------

//! \brief          Saturates a control variable and returns true if either limit is exceeded
//!
//! \param[in] data The address of the data variable
//! \param[in] Umax The upper limit
//! \param[in] Umin The lower limit
//! \return         Returns 'false' if (Umin < data < Umax), else 'true'
//!
_DCL_CODE_ACCESS
bool DCL_runClamp(float32_t *data, float32_t Umax, float32_t Umin)
{   
    float32_t iv = *(data);
    *(data) = (*(data) > Umax) ? Umax : *(data);
    *(data) = (*(data) < Umin) ? Umin : *(data);
    return(((iv < Umax) && (iv > Umin)) ? false : true);
}

//! \brief          Saturates a control variable and returns true if either limit is exceeded 
//!
//! \param[in] data The address of the data variable
//! \param[in] Umax The upper limit
//! \param[in] Umin The lower limit
//! \return         Returns 'false' if (Umin < data < Umax), else 'true'
//!
_DCL_CODE_ACCESS
bool DCL_runClampF64(float64_t *data, float64_t Umax, float64_t Umin)
{
    float64_t iv = *(data);
    *(data) = (*(data) > Umax) ? Umax : *(data);
    *(data) = (*(data) < Umin) ? Umin : *(data);
    return(((iv < Umax) && (iv > Umin)) ? false : true);
}

//! \brief          Macro to saturate a control variable but does not change the data itself unlike runClamp() 
//!
//! \param[in] data The data value
//! \param[in] Umax The upper limit
//! \param[in] Umin The lower limit
//! \return         Returns unchanged data value is not saturated, else either Umax or Umin
#define DCL_runSat(data,Umax,Umin) (data > Umax) ? Umax : (data < Umin) ? Umin : data

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_CLAMP_H_
