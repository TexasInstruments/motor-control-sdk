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


//! \file   solutions/universal_motorcontrol_lab/common/include/hal_data.h
//! \brief  hardware abstraction layer (HAL) library
//!
//!


#ifndef HAL_DATA_H
#define HAL_DATA_H


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup HAL HAL_DATA
//! @{
//
//*****************************************************************************

#include "math_types.h"

//*****************************************************************************
//
//! \brief Defines the ADC data
//
//*****************************************************************************
typedef struct _HAL_ADCData_t_
{
    float32_t VdcBus_V;         //!< the dc bus voltage values

    MATH_Vec3 I_A;              //!< the current values
    MATH_Vec3 V_V;              //!< the voltage values

    MATH_Vec3 offset_I_ad;      //!< the motor current offset value
    MATH_Vec3 offset_V_sf;      //!< the motor voltage offset factor

    float32_t current_sf;       //!< the current scale factor, amps/cnt
    float32_t voltage_sf;       //!< the phase voltage scale factor, volts/cnt
    float32_t dcBusvoltage_sf;  //!< the phase voltage scale factor, volts/cnt

} HAL_ADCData_t;

//*****************************************************************************
//
//! \brief Defines the PWM data
//
//*****************************************************************************
typedef struct _HAL_PWMData_t_
{
    MATH_Vec3 Vabc_pu;     //!< the PWM time-durations for each motor phase

    uint16_t  cmpValue[3];
    uint16_t  deadband;
    uint16_t  noiseWindow;
    uint16_t  adcSample;
    uint16_t  minCMPValue;
    uint16_t  period;
    uint16_t  socCMP;
    Bool      flagEnablePwm;
} HAL_PWMData_t;



//*****************************************************************************
//
//! \brief Defines the DAC data
//
//*****************************************************************************
typedef struct _HAL_BuffDACData_t_
{
    volatile float32_t *ptrData[2]; //!< Input: First input pointer

    float32_t offset[2];          //!< the DAC data
    float32_t gain[2];            //!< the DAC data
} HAL_BuffDACData_t;


typedef struct _HAL_PwmDacData_t_
{
    volatile float32_t *ptrData[4];     //!< Input: First input pointer

    float32_t offset[4];                //!< the DAC data
    float32_t gain[4];                  //!< the DAC data

    uint16_t periodMax;
} HAL_PWMDACData_t;

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of HAL_DATA_H definition
