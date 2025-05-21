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

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \ingroup  OBSERVERS_MODULE
 *
 *  List of Motor structs and enums
 *  @{
 *
 *  \file           motor.h
 *  \brief          Contains motor-specific enum and structs
 */

// the includes
#include "math_types.h"


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//
//! \brief Enumeration for the motor types
//!
typedef enum
{
  MOTOR_TYPE_INDUCTION = 0,   //!< induction
  MOTOR_TYPE_PM        = 1    //!< permanent magnet
} MOTOR_Type_e;

//
//! \brief Enumeration for the Flying Start Mode
//
typedef enum
{
    FLYINGSTART_MODE_HALT    = 0,       //!< Halt Mode
    FLYINGSTART_MODE_STANDBY = 1        //!< Standby Mode
} FlyingStart_Mode_e;


//
//! \brief Enumeration for the braking Mode
//
typedef enum
{
    FREE_STOP_MODE           = 0,       //!< Free stop mode without braking
    HARDSWITCH_BRAKE_MODE    = 1,       //!< Hard switch braking mode
    FORCESTOP_BRAKE_MODE     = 2,       //!< Force alignment braking mode
    DYNAMIC_BRAKE_MODE       = 3,       //!< N/A, Dynamic braking mode
    REGENERATION_BRAKE_MODE  = 4        //!< N/A, Regeneration braking mode
} BRAKE_Mode_e;

//
//! \brief Enumeration for the control mode
//
typedef enum
{
    OPERATE_MODE_SCALAR = 0,        //!< V/f scalar control
    OPERATE_MODE_VQCL   = 1,        //!< Vq direction control
    OPERATE_MODE_TORQUE = 2,        //!< Torque control running mode
    OPERATE_MODE_SPEED  = 3,        //!< Speed closed-loop running mode
    OPERATE_MODE_POWER  = 4         //!< Constant power running mode
} OPERATE_Mode_e;

//! \brief Enumeration for the estimator mode
//
typedef enum
{
    SAMPLE_MODE_DCSS2  = 0,         //!< dclink_ss2
    SAMPLE_MODE_DCSS4  = 1,         //!< dclink_ss4
    SAMPLE_MODE_DCLINK = 2,         //!< dclink_ss
    SAMPLE_MODE_3LSR   = 3,         //!< three_shunt
    SAMPLE_MODE_3INL   = 4,         //!< inline
    SAMPLE_MODE_SDFM   = 5          //!< sdfm
} SAMPLE_Mode_e;

// State machine typedef for motor running status
typedef enum
{
    MOTOR_STOP_IDLE      = 0,
    MOTOR_FAULT_STOP     = 1,
    MOTOR_BRAKE_STOP     = 2,
    MOTOR_NORM_STOP      = 3,
    MOTOR_CHARGE         = 4,
    MOTOR_SEEK_POS       = 5,
    MOTOR_ALIGNMENT      = 6,
    MOTOR_IPD_HFI        = 7,
    MOTOR_OL_START       = 8,
    MOTOR_CL_RUNNING     = 9,
    MOTOR_CTRL_RUN       = 10,
    MOTOR_RUN_FWC        = 11,
    MOTOR_IDENTIFICATION = 12,
    MOTOR_RS_RECALC      = 13
} MOTOR_Status_e;

//! \brief Enumeration for the motor drive control state
//
typedef enum
{
    MOTOR_CTRL_FREE_STOP    = 0,    //!< (0000), Stop the motor with free mode
    MOTOR_CTRL_BRAKE_STOP   = 1,    //!< (0001), Stop the motor with braking mode
    MOTOR_CTRL_FREQ_CW      = 2,    //!< (0010), CW spin with speed closed-loop (Hz)
    MOTOR_CTRL_FREQ_CCW     = 3,    //!< (0011), CCW spin with speed closed-loop (Hz)
    MOTOR_CTRL_TORQUE_CW    = 4,    //!< (0100), CW spin with torque control
    MOTOR_CTRL_TORQUE_CCW   = 5,    //!< (0101), CCW spin with torque control
    MOTOR_CTRL_SPEED_CW     = 6,    //!< (0110), CW spin with speed closed-loop (rpm)
    MOTOR_CTRL_SPEED_CCW    = 7,    //!< (0111), CCW spin with speed closed-loop (rpm)
    MOTOR_CTRL_DEBUG_CHECK  = 8,    //!< (1000), debug monitor
    MOTOR_CTRL_URGENT_STOP  = 9,    //!< (1001), urgent stop
    MOTOR_CTRL_IDENTIFy     = 10,   //!< (1010), Motor parameters identification
    MOTOR_CTRL_RSCALC       = 11    //!< (1011), Rs recalculation
} MOTOR_CtrlMode_e;

//! \brief Defines the motor parameters
//!
typedef struct _MOTOR_Params_
{
  MOTOR_Type_e    type;               //!< Defines the motor type

  uint_least16_t  numPolePairs;       //!< Defines the number of pole pairs

  float32_t         Lmag_H;             //!< Defines the magnetizing inductance, H
  float32_t         Ls_d_H;             //!< Defines the direct stator inductance, H
  float32_t         Ls_q_H;             //!< Defines the quadrature stator inductance, H

  float32_t         Rr_d_Ohm;           //!< Defines the direct rotor resistance, Ohm
  float32_t         Rr_q_Ohm;           //!< Defines the quadrature rotor resistance, Ohm

  float32_t         Rs_d_Ohm;           //!< Defines the direct stator resistance, Ohm
  float32_t         Rs_q_Ohm;           //!< Defines the quadrature stator resistance, Ohm

  float32_t         ratedFlux_Wb;       //!< Defines the rated flux, Wb
} MOTOR_Params;

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

#endif // end of MOTOR_LIB_H definition





