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

#ifndef USERPARAMS_H
#define USERPARAMS_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \ingroup  OBSERVERS_MODULE
 *
 *  
 *  @{
 *
 *  \file           userParams.h
 *  \brief          Contains the public interface for the HAL and EST modules
 */

#include <stdbool.h>

#include "math_types.h"
#include "motor.h"

// **************************************************************************
// the defines


//! \brief Enumeration for the Motor numbers
//!
typedef enum
{
  MTR_1 = 0,
  MTR_2 = 1,
  MTR_3 = 2,
} MotorNum_e;

//*****************************************************************************
//
//! \brief Defines a structure for the user parameters
//
//*****************************************************************************
typedef struct _USER_Params_
{
    float32_t dcBus_nominal_V;            //!< Defines the nominal DC bus
                                        //!< voltage, V
    int_least16_t numIsrTicksPerCtrlTick;
                                        //!< Defines the number of Interrupt
                                        //!< Service Routine (ISR) clock ticks
                                        //!< per controller clock tick
    int_least16_t numIsrTicksPerEstTick;
                                        //!< Defines the number of Interrupt
                                        //!< Service Routine (ISR) clock ticks
                                        //!< per controller clock tick
    int_least16_t numIsrTicksPerTrajTick;
                                        //!< Defines the number of Interrupt
                                        //!< Service Routine (ISR) clock ticks
                                        //!< per controller clock tick
    int_least16_t numCtrlTicksPerCurrentTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per current controller
                                        //!< clock tick
    int_least16_t numCtrlTicksPerSpeedTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per speed controller
                                        //!< clock tick
    uint_least8_t numCurrentSensors;    //!< Defines the number of current
                                        //!< sensors
    uint_least8_t numVoltageSensors;    //!< Defines the number of voltage
                                        //!< sensors
    float32_t systemFreq_MHz;             //!< Defines the system clock
                                        //!< frequency, MHz
    float32_t pwmPeriod_usec;             //!< Defines the Pulse Width Modulation
                                        //!< (PWM) period, usec
    float32_t voltage_sf;                 //!< Defines the voltage scale factor
                                        //!< for the system
    float32_t current_sf;                 //!< Defines the current scale factor
                                        //!< for the system
    float32_t dcBusPole_rps;              //!< Defines the pole location for the
                                        //!< DC bus filter, rad/sec
    float32_t speedPole_rps;              //!< Defines the pole location for the
                                        //!< speed control filter, rad/sec
    float32_t voltageFilterPole_rps;      //!< Defines the analog voltage filter
                                        //!< pole location, rad/sec
    float32_t maxVsMag_pu;                //!< Defines the maximum Vs magnitude
                                        //!< in per units
    MOTOR_Type_e motor_type;              //!< Defines the motor type
    uint16_t motor_numPolePairs;          //!< Defines the number of pole pairs
                                        //!< for the motor
    uint16_t motor_numEncSlots;         //!< Defines the number of encoder
                                        //!< slots if quadrature encoder is
                                        //!< connected
    float32_t motor_ratedFlux_Wb;         //!< Defines the rated flux of the
                                        //!< motor, Wb
    float32_t motor_Rr_Ohm;               //!< Defines the direct rotor
                                        //!< resistance, Ohm
    float32_t motor_Rs_Ohm;               //!< Defines the alpha stator
                                        //!< resistance, Ohm
    float32_t motor_Ls_d_H;               //!< Defines the direct stator
                                        //!< inductance, H
    float32_t motor_Ls_q_H;               //!< Defines the quadrature stator
                                        //!< inductance, H
    float32_t maxCurrent_A;               //!< Defines the maximum current value,
                                        //!< A
    float32_t IdRated_A;                  //!< Defines the Id rated current
                                        //!< value, A
    float32_t Vd_sf;                      //!< Defines the Vd scale factor to
                                        //!< prevent a Vd only component for
                                        //!< the Vdq vector
    float32_t maxVsMag_V;                 //!< Defines the maximum stator voltage
                                        //!< magnitude, V
    float32_t BWc_rps;                    //!< Defines the bandwidth of the
                                        //!< current controllers, rad/sec
    float32_t BWdelta;                    //!< Defines the bandwidth scaling to
                                        //!< maximize phase margin
    float32_t Kctrl_Wb_p_kgm2;            //!< Defines the speed controller
                                        //!< constant, Wb/(kg*m^2)
    float32_t angleDelayed_sf_sec;        //!< Defines the scale factor for
                                        //!< computing the angle considering
                                        //!< system delay, sec
    float32_t fluxExcFreq_Hz;             //!< Defines the flux excitation
                                        //!< frequency, Hz
    float32_t ctrlFreq_Hz;                //!< Defines the controller frequency,
                                        //!< Hz
    float32_t estFreq_Hz;                 //!< Defines the estimator frequency,
                                        //!< Hz
    float32_t RoverL_excFreq_Hz;          //!< Defines the R/L excitation
                                        //!< frequency, Hz
    float32_t trajFreq_Hz;                //!< Defines the trajectory frequency,
                                        //!< Hz
    float32_t ctrlPeriod_sec;             //!< Defines the controller execution
                                        //!< period, sec
    float32_t maxAccel_Hzps;              //!< Defines the maximum acceleration
                                        //!< for the speed profiles, Hz/sec
    float32_t maxCurrent_resEst_A;        //!< Defines the maximum current value 
                                        //!< for resistance estimation, A
    float32_t maxCurrent_indEst_A;        //!< Defines the maximum current value
                                        //!< for inductance estimation, A
    float32_t maxCurrentDelta_A;          //!< Defines the maximum current delta
                                        //!< for Id current trajectory
    float32_t maxCurrentDelta_pw_A;       //!< Defines the maximum current delta
                                        //!< for Id current trajectory during
                                        //!< power warp mode
    float32_t IdRated_delta_A;            //!< Defines the Id rated delta current
                                        //!< value, A
    float32_t forceAngleFreq_Hz;          //!< Defines the forced angle
                                        //!< frequency, Hz
    float32_t forceAngleAccel_Hzps;       //!< Defines the forced angle
                                        //<! acceleration. Hzps
    float32_t indEst_speedMaxFraction;    //!< Defines the fraction of SpeedMax
                                        //!< to use during inductance
                                        //!< estimation
    float32_t IdRatedFraction_indEst;     //!< Defines the fraction of Id rated
                                        //!< current to use during inductance
                                        //!< estimation
    float32_t pwGain;                     //!< Defines the power warp gain for
                                        //!< computing Id reference
    float32_t Kp_min_VpA;                 //!< Defines the minimum Kp value for
                                        //!< the current controller, V/A
    float32_t Kp_max_VpA;                 //!< Defines the maximum Kp value for
                                        //!< the current controller, V/A
    float32_t RoverL_Kp_sf;               //!< Defines the Kp scale factor used
                                        //!< during R/L, pu
    float32_t RoverL_min_rps;             //!< Defines the minimum estimated R/L
                                        //!< value allowed, rad/sec
    float32_t RoverL_max_rps;             //!< Defines the maximum estimated R/L
                                        //!< value allowed, rad/sec
    float32_t oneOverDcBus_min_invV;      //!< Defines the minimum estimated
                                        //!< 1/dcBus value allowed, 1/V
    float32_t oneOverDcBus_max_invV;      //!< Defines the maximum estimated
                                        //!< 1/dcBus value allowed, 1/V
    float32_t Ls_d_H;                     //!< Defines the default stator
                                        //!< inductance in the direct
                                        //!< direction, H
    float32_t Ls_q_H;                     //!< Defines the default stator
                                        //!< inductance in the quadrature
                                        //!< direction, H
    float32_t Ls_coarseDelta_H;           //!< Defines the delta inductance value
                                        //!< during the coarse stator
                                        //!< inductance estimation, H
    float32_t Ls_fineDelta_H;             //!< Defines the delta inductance value
                                        //!< during the fine stator inductance
                                        //!< estimation, H
    float32_t Ls_min_H;                   //!< Defines the minimum stator
                                        //!< inductance value allowed, H
    float32_t Ls_max_H;                   //!< Defines the maximum stator
                                        //!< inductance value allowed, H
    float32_t Rr_Ohm;                     //!< Defines the default rotor
                                        //!< resistance value, Ohm
    float32_t Rr_coarseDelta_Ohm;         //!< Defines the delta resistance value
                                        //!< during coarse rotor resistance
                                        //!< estimation, Ohm
    float32_t Rr_fineDelta_Ohm;           //!< Defines the delta resistance value
                                        //!< during fine rotor resistance
                                        //!< estimation, Ohm
    float32_t Rr_min_Ohm;                 //!< Defines the minimum rotor
                                        //!< resistance value allowed, Ohm
    float32_t Rr_max_Ohm;                 //!< Defines the maximum rotor
                                        //!< resistance value allowed, Ohm
    float32_t Rs_Ohm;                     //!< Defines the default stator
                                        //!< resistance value, Ohm
    float32_t Rs_coarseDelta_Ohm;         //!< Defines the delta resistance value
                                        //!< during coarse stator resistance
                                        //!< estimation, Ohm
    float32_t Rs_fineDelta_Ohm;           //!< Defines the delta resistance value
                                        //!< during fine stator resistance
                                        //!< estimation, Ohm
    float32_t Rs_min_Ohm;                 //!< Defines the minimum stator
                                        //!< resistance value allowed, Ohm
    float32_t Rs_max_Ohm;                 //!< Defines the maximum stator
                                        //!< resistance value allowed, Ohm
    float32_t RsOnLine_DeltaInc_Ohm;      //!< Defines the delta resistance
                                        //!< increment value during online
                                        //!< stator resistance estimation, Ohm
    float32_t RsOnLine_DeltaDec_Ohm;      //!< Defines the delta resistance
                                        //!< decrement value during online
                                        //!< stator resistance estimation, Ohm
    float32_t RsOnLine_min_Ohm;           //!< Defines the minimum online stator
                                        //!< resistance value allowed, Ohm
    float32_t RsOnLine_max_Ohm;           //!< Defines the maximum online stator
                                        //!< resistance value allowed, Ohm
    float32_t RsOnLine_angleDelta_rad;    //!< Defines the delta angle value
                                        //!< during online stator resistance
                                        //!< estimation, rad
    float32_t RsOnLine_pole_rps;          //!< Defines the filter pole for online
                                        //!< stator resistance estimation,
                                        //!< rad/sec
    bool flag_bypassMotorId;            //!< A flag to bypass motor
                                        //!< identification and use the motor
                                        //!< parameters
    // *******************************************************************************************************
    // BELOW IS RESERVED
    float32_t freqNearZeroSpeedLimit_Hz;  //!< 1. Defines the low speed limit for frequency estimation, Hz
    float32_t directionPole_rps;          //!< 2. Defines the pole location for the direction filter, rad/sec
    float32_t directionPole_2_rps;        //!< 3. Defines the second pole for the direction filter, rad/sec
    float32_t fluxPole_rps;               //!< 4. Defines the pole location for the flux estimation, rad/sec
    float32_t RoverLPole_rps;             //!< 5. Defines the pole location for the R/L estimation, rad/sec
    float32_t estKappa;                   //!< 6. Defines the convergence factor for the estimator

    float32_t flux_Wb;                    //!< 7. Defines the default flux value, Wb
    float32_t flux_min_Wb;                //!< 8. Defines the minimum flux value allowed, Wb
    float32_t flux_max_Wb;                //!< 9. Defines the maximum flux value allowed, Wb

    float32_t oneOverFlux_min_sf;         //!< 10. Defines the scale factor for the minimum 1/flux value allowed
    float32_t oneOverFlux_max_sf;         //!< 11. Defines the scale factor for the maximum 1/flux value allowed
    float32_t Dir_fe_min_Hz;              //!< 12. Defines the minimum electrical frequency to still use direction estimation, V/Hz
    float32_t Dir_fe_max_Hz;              //!< 13. Defines the maximum electrical frequency to still use direction estimation, V/Hz

    // BELOW can be added with user's own varaibles
    float32_t maxFrequency_Hz;          //!< Defines the maximum frequency value,
                                        //!< Hz
} USER_Params, *userParams_Handle;

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

#endif // end of USERPARAMS_H definition
