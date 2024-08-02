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

#ifndef USER_COMMON_H
#define USER_COMMON_H

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
//! \defgroup USER COMMON
//! @{
//
//*****************************************************************************

// the includes
#include "userParams.h"

// the defines

//! \brief Defines the system clock frequency, Hz
//! if the CPU clock is not equal to DEVICE_SYSCLK_FREQ due to the settings in
//! SysCtl_setClock(), need to change this definition accordingly

//! \brief Define the system frequency (Hz)
#define DEVICE_SYSCLK_FREQ     200000000.0f

#define USER_SYSTEM_FREQ_Hz             (float32_t)(DEVICE_SYSCLK_FREQ)

//! \brief Defines the system clock frequency, MHz
#define USER_SYSTEM_FREQ_MHz            (float32_t)(USER_SYSTEM_FREQ_Hz / 1000000.0f)

//! \brief Defines the system time base period, ms
//!
#define USER_TIME_BASE_PERIOD_ms        (1.0f)

//! \brief Defines the system time base frequency, Hz
//!
#define USER_TIME_BASE_FREQ_Hz          (1000.0f / USER_TIME_BASE_PERIOD_ms)

//! \brief A flag to bypass motor identification (1/0 : TRUE/FALSE)
//!
#define USER_BYPASS_MOTOR_ID       (1)          // No motor parameters identification
//#define USER_BYPASS_MOTOR_ID        (0)           // Do motor parameters identification

#define USER_ENABLE_MOTOR_ID        0
#define USER_DISABLE_MOTOR_ID       1

//============================================================================================
// Motor defines

//************** Motor Parameters **************

// PMSM motors
#define Estun_EMJ_04APB22           101
#define CHMotor_WM_Test             102

#define ziehlab_Fan160hv            112
#define embpast_Fan160hv            113
#define QXA_A091ZE190A              114
#define GMCC_KSK89D53U              115
#define Baldor_BSM90N175            116
#define Marathon_N56PNRA10102       117

#define Teknic_M2310PLN04K          122
#define Anaheim_BLY172S_24V         124
#define Anaheim_BLY341S_48V         125
#define Anaheim_BLY341S_Y24V        126
#define Anaheim_BLY341S_D24V        127
#define Nedic_EPSMS037_D12V         128

#define Anaheim_BLZ362S             141
#define Anaheim_BLWS235D            142
#define Drone_BLK2BLADE             143
#define Drone_DJI920KV              144
#define Drone_SF_Black              145
#define Drone9616_110KV_48V         146

#define AirFan_MFA0500_24V          150
#define Tool_Makita_GFD01           151

#define Tamagawa_TS4606N8302        160     // dc24v
#define AKM21G_CK9NGE00             161     // dc75v

// ACIM motors
#define Marathon_5K33GN2A           200
#define Marathon_56H17T2011A        201
#define Dayton_3N352C               202
#define EMSYNERGY_LVACI             203

// User motors
#define my_pm_motor_1               301
#define my_aci_motor_2              302
#define RL_simulation               305

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


//// **************************************************************************
//// the functions
////! \brief      Sets the private user parameter values
////! \param[in]  pUserParams  The pointer to the user param structure
//extern void cla_USER_setParams_priv(USER_Params *pUserParams);

//! \brief      Sets the private user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure


extern void USER_setParams_priv(USER_Params *pUserParams);


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

#endif // end of USER_COMMON_H definition

