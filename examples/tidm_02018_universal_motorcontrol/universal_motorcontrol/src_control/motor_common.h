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


#ifndef _MOTOR_COMMON_H_
#define _MOTOR_COMMON_H_


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
//! \defgroup MOTOR COMMON
//! @{
//
//*****************************************************************************

// includes
#include "math_types.h"
#include <math.h>


#include "userParams.h"

#include "clarke.h"
#include "filter_fo.h"
#include "ipark.h"
#include "park.h"
#include "pi.h"
#include "svgen.h"
#include "svgen_current.h"
#include "traj.h"
#include "fwc.h"
#include "mtpa.h"

#include "vs_freq.h"
#include "angle_gen.h"
#include "volt_recons.h"


#if defined(MOTOR1_ENC)
#include "encoder.h"
#include "speedcalc.h"
#include "hall.h"
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
#include "hall.h"
#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
#include "esmo.h"
#include "speedfr.h"
#endif  // MOTOR1_ESMO


// solutions

#include "user.h"
#include "hal.h"

#if defined(SFRA_ENABLE)
#include "sfra_settings.h"
#include "sfra_f32.h"
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
#include "step_response.h"
#endif  // STEP_RP_EN

#if defined(CMD_CAN)
#include "can_msg.h"
#endif // CMD_CAN

//*****************************************************************************
#define M_OVER_VOLTAGE_BIT          0x0001    // DC Bus Over Voltage Fault
#define M_UNDER_VOLTAGE_BIT         0x0002    // DC Bus Under Voltage Fault
#define M_MOTOR_OVER_TEMPER_BIT     0x0004    // Motor over temperature Fault
#define M_MODULE_OVER_TEMPER_BIT    0x0008    // Module over temperature Fault

#define M_MODULE_OVER_CURRENT_BIT   0x0010    // Hardware Over Current Fault
#define M_OVER_PEAK_CURRENT_BIT     0x0020    // internal CMPSS Over Current Fault
#define M_MOTOR_OVER_LOAD_BIT       0x0040    // Over Load Error
#define M_MOTOR_LOST_PHASE_BIT      0x0080    // Motor Lost Phase

#define M_CURRENT_UNBALANCE_BIT     0x0100    // Motor Phase Current Unbalance
#define M_MOTOR_STALL_BIT           0x0200    // Motor Stall
#define M_STARTUP_FAILE_BIT         0x0400    // Startup failed
#define M_MOTOR_OVER_SPEED_BIT      0x0800    // Motor Over Speed

#define M_RESERVE_12_BIT            0x1000    // Reserved for further use
#define M_RESERVE_13_BIT            0x2000    // Reserved for further use
#define M_CURRENT_OFFSET_BIT        0x4000    // Current offsets
#define M_VOLTAGE_OFFSET_BIT        0x8000    // voltage offsets

#define M_MASK_ALL_FAULT_BITS       0x0000
#define M_ENABLE_ALL_FAULT_BITS     0xFFFF

//
// Block all fault protection except current, voltage and temperature faults
//
#define MTR_FAULT_OV_BRAKE             M_OVER_VOLTAGE_BIT

#define MTR_FAULT_ENABLE_OC             M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT

#define MTR_FAULT_ENABLE_OC_OUV         M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT
//
// Enable all fault protection
//
#define MTR_FAULT_ENABLE_ALL            M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT               \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_MOTOR_OVER_LOAD_BIT                  \
                                      + M_MOTOR_LOST_PHASE_BIT                 \
                                      + M_CURRENT_UNBALANCE_BIT                \
                                      + M_MOTOR_STALL_BIT                      \
                                      + M_STARTUP_FAILE_BIT                    \
                                      + M_MOTOR_OVER_SPEED_BIT                 \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT

// Clear all fault protection except over/under voltage and offset error
//
#define MTR_FAULT_CLEAR                 M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT

#define MTR_FAULT_DISABLE_ALL           0x0000

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC
#else
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC_OUV
#endif

//------------------------------------------------------------------------------
//
//! \brief Enumeration for the kit boards
//
typedef enum
{
    PRJ_NON_SYSCONFIG  = 0,         //!< Without using Sysconfig
    PRJ_DEV_SYSCONFIG  = 1,         //!< Multiple Sysconfig files only for device configuration
    PRJ_ALL_SYSCONFIG  = 2          //!< A single Sysconfig file for device and project configuration
} Project_Config_e;

//
//! \brief Enumeration for the kit boards
//
typedef enum
{
    BOARD_BSXL8323RS_REVA  = 0,         //!< the board is BOOSTXL_8323RS,  OK, in MCSDK
    BOARD_BSXL8323RH_REVB  = 1,         //!< the board is BOOSTXL_8323RH,  OK, in MCSDK
    BOARD_BSXL8353RS_REVA  = 2,         //!< the board is BOOSTXL_8353RS,  OK, in MCSDK
    BOARD_BSXL3PHGAN_REVA  = 3,         //!< the board is BOOSTXL_3PHGAN,  OK, in MCSDK
    BOARD_HVMTRPFC_REV1P1  = 4,         //!< the board is HVMTRPFC_REV1P1, OK, in MCSDK
    BOARD_BSXL8316RT_REVA  = 5,         //!< the board is BOOSTXL_8316RT,  OK, in MCSDK
    BOARD_DRV8329AEVM_REVA = 6,         //!< the board is DRV8329AEVM_REVA,OK, in MCSDK
    BOARD_BP_AM2BLDCSERVO  = 7          //!< the board is BP-AM2BLDCSERVO, OK, in MCSDK
} Board_Kit_e;

//
//! \brief Enumeration for the estimator mode
//  0 -ESTIMATOR_MODE_ABS
//  1 -ESTIMATOR_MODE_ESMO
//  2 -ESTIMATOR_MODE_ENC
//  3 -ESTIMATOR_MODE_RESL
//  4 -ESTIMATOR_MODE_SCOS
//  5 -ESTIMATOR_MODE_HALL
#if defined(MOTOR1_ENC) && defined(MOTOR1_HALL)
#error Can't support ENC and HALL simultaneously
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_HALL)
#error Can't support ESMO and HALL simultaneously
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
typedef enum
{
    ESTIMATOR_MODE_ESMO  = 1,             //!< ESMO estimator
    ESTIMATOR_MODE_ENC   = 2              //!< Encoder
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ABS_ENC)
typedef enum
{
    ESTIMATOR_MODE_ABS  = 0              //!< Absolute Encoder estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ESMO)
typedef enum
{
    ESTIMATOR_MODE_ESMO  = 1              //!< ESMO estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ENC)
typedef enum
{
    ESTIMATOR_MODE_ENC   = 2              //!< Encoder
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_HALL)
typedef enum
{
    ESTIMATOR_MODE_HALL  = 5              //!< Hall sensor
} ESTIMATOR_Mode_e;
#else
#error Not select a right estimator for this project
#endif

//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    EST_TYPE_ESMO          = 0,     //!< the estimator is only ESMO
    EST_TYPE_ENC           = 1,     //!< the estimator is only ENC
    EST_TYPE_ABS_ENC       = 2,     //!< the estimator is only Absolute ENC
    EST_TYPE_HALL          = 3,     //!< the estimator is only HALL
    EST_TYPE_RESL          = 4,     //!< the estimator is only RESOLVER
    EST_TYPE_PSCOS         = 5,     //!< the estimator is only SIN/COS
    EST_TYPE_ESMO_ENC      = 6      //!< the estimator is ESMO and ENC
} EST_Type_e;


typedef enum
{
    CURSEN_TYPE_SINGLE_SHUNT  = 0,   //!< the single shunt
    CURSEN_TYPE_THREE_SHUNT   = 1,   //!< the three shunt
    CURSEN_TYPE_INLINE_SHUNT  = 2,   //!< the inline shunt with ISO AMP
    CURSEN_TYPE_INLINE_HALL   = 3,   //!< the hall effect sensor
    CURSEN_TYPE_INLINE_SDFM   = 4    //!< the inline shunt with SDFM
} CURRENTSEN_Type_e;


#if defined(CMD_CAN)
typedef struct
{
    float32_t speedSet_Hz;           //[0:31]
    Bool flagCmdRun:1;               //[32]
    Bool flagEnableCmd:1;            //[33]
    Bool flagEnableSyncLead:1;       //[34]
    unsigned int Reserved_0:13;      //[35:47]
    unsigned int Reserved_1:16;      //[48:63]
}RX_1_Type;

typedef struct
{
    float32_t    speedSet_Hz;        //[0:31]
    unsigned int MotorFault:1;       //[32]
    unsigned int Reserved_0:15;      //[33:47]
    unsigned int Reserved_1:16;      //[48:63]
}TX_1_Type;
#endif // CMD_CAN

//
//! \brief typedefs for the fault
//
typedef struct _FAULT_MTR_BITS_
{             // bits  description
    uint16_t overVoltage:1;         // 0  DC Bus Over Voltage Fault
    uint16_t underVoltage:1;        // 1  DC Bus Under Voltage Fault
    uint16_t motorOverTemp:1;       // 2  Motor over temperature Fault
    uint16_t moduleOverTemp:1;      // 3  Power module over temperature Fault

    uint16_t moduleOverCurrent:1;   // 4  Hardware Over Current Fault Flag
    uint16_t overPeakCurrent:1;     // 5  internal CMPSS Over Current Fault Flag
    uint16_t overLoad:1;            // 6  Over Load Error
    uint16_t motorLostPhase:1;      // 7  Motor Lost Phase

    uint16_t currentUnbalance:1;    // 8  Motor Phase Current imbalance
    uint16_t motorStall:1;          // 9  Motor Stall
    uint16_t startupFailed:1;       // 10 Startup failed
    uint16_t overSpeed:1;           // 11 Motor Over Speed

    uint16_t reserve12:1;           // 12 Reserved
    uint16_t gateDriver:1;          // 13 Gate driver
    uint16_t currentOffset:1;       // 14 Current offset check
    uint16_t voltageOffset:1;       // 15 voltage offset check
} FAULT_MTR_BITS;

typedef union _FAULT_MTR_REG_t
{
    uint16_t        all;
    FAULT_MTR_BITS  bit;
}FAULT_MTR_REG_t;

typedef struct _MOTOR_SetVars_t_
{
    uint16_t overCurrentTimesSet;
    uint16_t voltageFaultTimeSet;
    uint16_t motorStallTimeSet;
    uint16_t startupFailTimeSet;

    uint16_t overSpeedTimeSet;
    uint16_t overLoadTimeSet;
    uint16_t unbalanceTimeSet;
    uint16_t lostPhaseTimeSet;

    uint16_t stopWaitTimeSet;
    uint16_t restartWaitTimeSet;
    uint16_t restartTimesSet;
    uint16_t bootChargeTimeSet;

    uint16_t  dacCMPValH;
    uint16_t  dacCMPValL;

    float32_t Rr_Ohm;
    float32_t Rs_Ohm;
    float32_t Ls_d_H;
    float32_t Ls_q_H;
    float32_t flux_VpHz;
    float32_t flux_Wb;
    float32_t RoverL_rps;
    float32_t RsOnLine_Ohm;

#if defined(MOTOR1_PI_TUNE)
    float32_t Gain_speed_high_Hz;
    float32_t Gain_speed_low_Hz;

    float32_t Kp_spd_slope_sf;
    float32_t Ki_spd_slope_sf;

    float32_t Kp_spd_start_sf;
    float32_t Ki_spd_start_sf;

    float32_t Kp_spd_high_sf;
    float32_t Ki_spd_high_sf;

    float32_t Kp_spd_low_sf;
    float32_t Ki_spd_low_sf;

    float32_t Kp_spd_mid_sf;
    float32_t Ki_spd_mid_sf;

    float32_t Gain_Iq_high_A;
    float32_t Gain_Iq_low_A;

    float32_t Kp_Iq_slope_sf;
    float32_t Ki_Iq_slope_sf;

    float32_t Kp_Iq_start_sf;
    float32_t Ki_Iq_start_sf;

    float32_t Kp_Iq_high_sf;
    float32_t Ki_Iq_high_sf;

    float32_t Kp_Iq_low_sf;
    float32_t Ki_Iq_low_sf;

    float32_t Kp_Iq_mid_sf;
    float32_t Ki_Iq_mid_sf;

    float32_t Kp_Id_sf;
    float32_t Ki_Id_sf;

    float32_t Kp_spd_set;
    float32_t Ki_spd_set;

    float32_t Kp_Id_set;
    float32_t Ki_Id_set;

    float32_t Kp_Iq_set;
    float32_t Ki_Iq_set;
#endif      // MOTOR1_PI_TUNE

    float32_t Kp_spd;
    float32_t Ki_spd;

    float32_t Kp_Id;
    float32_t Ki_Id;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Kp_fwc;
    float32_t Ki_fwc;
    float32_t angleFWCMax_rad;

    float32_t overModulation;

    float32_t RsOnLineCurrent_A;
    float32_t magneticCurrent_A;

    float32_t lostPhaseSet_A;
    float32_t unbalanceRatioSet;
    float32_t overLoadSet_W;
    float32_t toqueFailMinSet_Nm;
    float32_t speedFailMaxSet_Hz;

    float32_t speedFailMinSet_Hz;
    float32_t stallCurrentSet_A;
    float32_t IsFailedChekSet_A;

    float32_t maxPeakCurrent_A;
    float32_t overCurrent_A;
    float32_t currentInv_sf;

    float32_t overVoltageFault_V;
    float32_t overVoltageNorm_V;
    float32_t underVoltageFault_V;
    float32_t underVoltageNorm_V;
} MOTOR_SetVars_t;

//! \brief Defines the MOTOR_SetVars_t handle
//!
typedef struct _MOTOR_SetVars_t_ *MOTORSETS_Handle;

//******************************************************************************
// typedefs
typedef struct _MOTOR_Vars_t_
{
    Bool flagEnableRunAndIdentify;
    Bool flagRunIdentAndOnLine;
    Bool flagEnableRestart;
    Bool flagMotorIdentified;
    Bool flagEnableForceAngle;
    Bool flagEnableAlignment;
    Bool flagEnableOffsetCalc;

    Bool flagSetupController;
    Bool flagEnableSpeedCtrl;
    Bool flagEnableCurrentCtrl;

    Bool flagEnableFlyingStart;
    Bool flagStateFlyingStart;
    Bool flagEnableBraking;
    Bool flagBrakeDone;

    Bool flagEnableSSIPD;
    Bool flagEnableFWC;
    Bool flagEnableMTPA;
    Bool flagUpdateMTPAParams;
    Bool flagPhaseAdjustEnable;

    Bool flagClearFaults;
    Bool flagVIrmsCal;

    Bool enableSpeedCtrl;
    Bool enableCurrentCtrl;

    Bool flagInitializeDone;

    Bool flagCmdRpmOrHz;                // TRUE-rpm, FALSE-Hz
    Bool flagEnableTuneController;      // TRUE-enable, FALSE-disable

#if defined(MOTOR1_FILTERIS)
    bool flagEnableFilterIs;
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    bool flagEnableFilterVs;
#endif  // MOTOR1_FILTERVS

    FAULT_MTR_REG_t faultMtrNow;
    FAULT_MTR_REG_t faultMtrUse;
    FAULT_MTR_REG_t faultMtrMask;
    FAULT_MTR_REG_t faultMtrPrev;

    ESTIMATOR_Mode_e estimatorMode;
    SAMPLE_Mode_e sampleMode;
    MOTOR_Status_e motorState;

    FlyingStart_Mode_e flyingStartMode;
    BRAKE_Mode_e brakingMode;
    OPERATE_Mode_e operateMode;

    MotorNum_e motorNum;

    uint16_t overCurrentTimesCnt;
    uint16_t overVoltageTimeCnt;
    uint16_t underVoltageTimeCnt;

    uint16_t motorStallTimeCnt;
    uint16_t startupFailTimeCnt;

    uint16_t bootChargeTimeCnt;
    uint16_t stopWaitTimeCnt;
    uint16_t restartTimesCnt;
    uint16_t startSumTimesCnt;

    uint16_t overSpeedTimeCnt;
    uint16_t overLoadTimeCnt;
    uint16_t unbalanceTimeCnt;
    uint16_t lostPhaseTimeCnt;

    uint16_t VIrmsIsrSet;
    uint16_t VIrmsIsrCnt;

#ifdef BRAKE_ENABLE
    uint16_t brakingTimeDelay;
    uint16_t brakingTimeCnt;
#endif  // BRAKE_ENABLE

    uint16_t stateRunTimeCnt;

    uint16_t alignTimeDelay;
    uint16_t forceRunTimeDelay;
    uint16_t startupTimeDelay;
    uint16_t flyingStartTimeDelay;
    uint16_t fwcTimeDelay;

    uint16_t counterSpeed;
    uint16_t counterTrajSpeed;

    uint32_t ISRCount;

    HAL_ADCData_t adcData;
    HAL_PWMData_t pwmData;

    float32_t debugData;

    float32_t direction;                    // 1.0f->forward, -1.0f->reserve

    float32_t speedRef_rpm;                 // Speed target value, rpm
    float32_t speed_rpm;                    // Speed feedback value, rpm
    float32_t rpm2Hz_sf;                    // Convert rpm to Hz
    float32_t hz2Rpm_sf;                    // convert Hz to rpm

    float32_t speedRef_Hz;                  // Speed target value, Hz
    float32_t speed_int_Hz;                 // Speed reference value, Hz
    float32_t speed_Hz;                     // Speed feedback value, Hz
    float32_t speed_reg_Hz;                 // Speed feedback value, Hz
    float32_t speed_rps;

    float32_t speedStart_Hz;
    float32_t speedForce_Hz;
    float32_t speedAbs_Hz;
    float32_t speedFilter_Hz;
    float32_t speedFlyingStart_Hz;

    float32_t accelerationMax_Hzps;
    float32_t accelerationStart_Hzps;

    float32_t angleFWC_rad;
    float32_t angleCurrent_rad;


    float32_t Is_A;
    float32_t Vs_V;
    float32_t VsRef_pu;
    float32_t VsRef_V;
    float32_t VsMax_V;              // The maximum stator voltage magnitude, V
    float32_t oneOverDcBus_invV;    // The DC Bus inverse, 1/V

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    //!< the reference current on d&q rotation axis
    MATH_Vec2 Idq_set_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    float32_t IdRated_A;
    float32_t IsRef_A;
    float32_t IsSet_A;
    float32_t Is_ffwd_A;

    float32_t fluxCurrent_A;
    float32_t alignCurrent_A;
    float32_t startCurrent_A;
    float32_t maxCurrent_A;
    float32_t brakingCurrent_A;

    float32_t torque_Nm;

    MATH_Vec2 Vab_out_V;             // the output control voltage on alpha&beta axis

    MATH_Vec2 Vdq_out_V;            // the output control voltage on d&q axis

    MATH_Vec2 Vdq_ffwd_V;           // the output offset voltage on d&q axis

    MATH_Vec2 Iab_A;                // the alpha/beta current values, A

    MATH_Vec2 Vab_V;                // the alpha/beta current values, V

    MATH_Vec2 Eab_V;                // the alpha/beta back EMF voltage values, V

    // the d&q axis current are converter from 3-phase sampling input current of motor
    MATH_Vec2 Idq_in_A;

    // the reference current on d&q rotation axis
    MATH_Vec2 IdqRef_A;

    // the reference output current on d&q rotation axis
    MATH_Vec2 Idq_out_A;

    float32_t frswPos_sf;

    float32_t angleDelayed_sf;

    // the rotor angle from Generator modules
    float32_t angleGen_rad;

    // the rotor angle from FOC modules
    float32_t angleFOC_rad;

    // the rotor angle from FOC modules
    float32_t angleFOCAdj_rad;

    // the rotor angle from FOC modules
    float32_t anglePhaseAdj_rad;

    // the rotor angle for braking
    float32_t angleBrake_rad;

    float32_t VIrmsIsrScale;
    float32_t IrmsCalSF;

    float32_t IrmsCalSum[3];
    float32_t IrmsPrdSum[3];
    float32_t Irms_A[3];

    float32_t unbalanceRatio;

    float32_t power_sf;
    float32_t powerReal_W;
    float32_t powerActive_W;

    float32_t powerInvertOut_W;
    float32_t powerMotorOut_W;

    // the handle for the hardware abstraction layer to motor control
    HAL_MTR_Handle halMtrHandle;

    MOTORSETS_Handle motorSetsHandle;

    userParams_Handle   userParamsHandle;

    // the handle for the speed PI controller
    PI_Handle     piHandle_spd;

    // the handle for the Id PI controller
    PI_Handle     piHandle_Id;

    // the handle for the Iq PI controller
    PI_Handle     piHandle_Iq;

    // the handle for the speed reference trajectory
    TRAJ_Handle  trajHandle_spd;

#if defined(MOTOR1_FILTERIS)
    // the current filter pole location, rad/sec
    float32_t filterIsPole_rps;

    // first order current filter handle
    FILTER_FO_Handle filterHandle_Is[3];

    //!< the current values
    MATH_Vec3 adcIs_A;
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    // the voltage filter pole location, rad/sec
    float32_t filterVsPole_rps;

    // first order voltage filter handle
    FILTER_FO_Handle filterHandle_Vs[3];

    //!< the voltage values
    MATH_Vec3 adcVs_V;
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // the space vector generator current object
    SVGENCURRENT_Handle svgencurrentHandle;
    MATH_Vec3 adcDataPrev;
    MATH_Vec3 pwmDataPrev;
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle;
    SVGENCURRENT_VmidShunt_e midVolShunt;
#endif  // MOTOR1_OVM

#if defined(MOTOR1_FWC)
    // the handle for the fwc PI controller
    PI_Handle    piHandle_fwc;
#endif  // MOTOR1_FWC || MOTOR2_FWC

#if defined(MOTOR1_ENC)
    // the rotor angle from Encoder modules
    float32_t angleENC_rad;

    // the speed from Encoder module
    float32_t speedENC_Hz;

    //!< the handle for the enc object
    ENC_Handle encHandle;

    //!< the handle for the speedcalc object
    SPDCALC_Handle spdcalcHandle;
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
    // the rotor angle from Hall Sensor
    float32_t angleHall_rad;

    // the speed from Hall Sensor
    float32_t speedHall_Hz;

    //!< the handle for the Hall object
    HALL_Handle hallHandle;
#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
    // the rotor angle from SMO modules
#if defined(ESMO_DEBUG)
    float32_t angleSMO_rad;
#endif  //ESMO_DEBUG

    // the rotor angle delay compensation value
    float32_t anglePLLComp_rad;

    // the rotor angle from PLL modules
    float32_t anglePLL_rad;

    // the speed from PLL module
    float32_t speedPLL_Hz;

    //!< the handle for the speedfr object
    SPDFR_Handle spdfrHandle;

    //!< the handle for the esmo object
    ESMO_Handle esmoHandle;
#endif  // MOTOR1_ESMO

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
    //!< the handles for Angle Generate for open loop control
    ANGLE_GEN_Handle angleGenHandle;
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT

#if defined(MOTOR1_MTPA)
    //!< the handle for the Maximum torque per ampere (MTPA)
    MTPA_Handle  mtpaHandle;

    float32_t angleMTPA_rad;
    float32_t mtpaKconst;
    float32_t LsOnline_d_H;
    float32_t LsOnline_q_H;
    float32_t fluxOnline_Wb;
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_DCLINKSS)
    //!< the handle for single-shunt current reconstruction
    DCLINK_SS_Handle dclinkHandle;
    float32_t sector;
#endif  // MOTOR1_DCLINKSS

#if defined(MOTOR1_VOLRECT)
    //!< the handle for the voltage reconstruct
    VOLREC_Handle volrecHandle;
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    //!< the handles for Vs per Freq for open loop control
    VS_FREQ_Handle VsFreqHandle;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(CMD_CAN)
    RX_1_Type cmdCANRX;
    TX_1_Type cmdCANTX;
#endif // CMD_CAN
}MOTOR_Vars_t;

//! \brief Defines the MOTOR_Vars_t handle
//!
typedef volatile struct _MOTOR_Vars_t_ *MOTOR_Handle;

#if defined(SFRA_ENABLE)
extern float32_t   sfraNoiseId;
extern float32_t   sfraNoiseIq;
extern float32_t   sfraNoiseSpd;
extern float32_t   sfraNoiseOut;
extern float32_t   sfraNoiseFdb;
extern SFRA_TEST_e sfraTestLoop;
extern Bool        sfraCollectStart;
#endif  // SFRA_ENABLE

#if defined(CPUTIME_ENABLE)
extern volatile float32_t cpuCyclesAv;
extern volatile uint32_t cpuCycles;
extern volatile uint32_t cycleCountBefore;
extern volatile uint32_t cycleCountAfter;
extern volatile uint32_t cycleCountAfter2;
extern volatile float32_t ISRcount;
#endif  // CPUTIME_ENABLE

//*****************************************************************************
// the function prototypes

//! \brief calculate motor over current threshold
//! \param[in]  handle   The motor control handle
extern void calcMotorOverCurrentThreshold(MOTOR_Handle handle);

//! \brief checks motor faults
//! \param[in]  handle   The motor control handle
extern void checkMotorFaults(MOTOR_Handle handle);

//! \brief  Update the controllers
static inline void updateControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    if(obj->motorState == MOTOR_CTRL_RUN)
    {
        // update the Id controller
        PI_setGains(obj->piHandle_Id, objSets->Kp_Id, objSets->Ki_Id);

        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);
    }
}

//! \brief  Get the controllers Parameters
static inline void getControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    // Get the Id controller parameters
    objSets->Kp_Id = PI_getKp(obj->piHandle_Id);
    objSets->Ki_Id = PI_getKi(obj->piHandle_Id);

    // Get the Iq controller parameters
    objSets->Kp_Iq = PI_getKp(obj->piHandle_Iq);
    objSets->Ki_Iq = PI_getKi(obj->piHandle_Iq);

    // Get the speed controller parameters
    objSets->Kp_spd = PI_getKp(obj->piHandle_spd);
    objSets->Ki_spd = PI_getKi(obj->piHandle_spd);

#if defined(MOTOR1_PI_TUNE)
    objSets->Kp_Id_set = objSets->Kp_Id;
    objSets->Ki_Id_set = objSets->Ki_Id;

    objSets->Kp_Iq_set = objSets->Kp_Iq;
    objSets->Ki_Iq_set = objSets->Ki_Iq;

    objSets->Kp_spd_set = objSets->Kp_spd;
    objSets->Ki_spd_set = objSets->Ki_spd;
#endif      // MOTOR1_PI_TUNE
}

//! \brief  Sets up control parameters for stopping motor
extern void stopMotorControl(MOTOR_Handle handle);

//! \brief  Sets up control parameters for restarting motor
extern void restartMotorControl(MOTOR_Handle handle);

//! \brief  Resets motor control parameters for restarting motor
extern void resetMotorControl(MOTOR_Handle handle);

#if defined(MOTOR1_PI_TUNE)
//! \brief  Tune the gains of the controllers according to the speed or load
extern void tuneControllerGains(MOTOR_Handle handle);

//! \brief  set the coefficient of the controllers gains
extern void setupControllerSF(MOTOR_Handle handle);
#endif      // MOTOR1_PI_TUNE

//! \brief  Sets up the current controllers
extern void setupCurrentControllers(MOTOR_Handle handle);

//! \brief  Sets up the controllers
extern void setupControllers(MOTOR_Handle handle);

//! \brief  Collect the current and voltage data to calculate the RMS
extern void collectRMSData(MOTOR_Handle handle);

//! \brief  Calculate the RMS data
extern void calculateRMSData(MOTOR_Handle handle);

//! \brief run motor monitor in main loop timer
extern void runMotorMonitor(MOTOR_Handle handle);

//! \brief      Updates the global motor variables
//! \param[in]  estHandle   The estimator (EST) handle
extern void updateGlobalVariables(MOTOR_Handle handle);

//! \brief      Updates the FWC parameters
extern void updateFWCParams(MOTOR_Handle handle);

//! \brief      Updates the MTPA parameters
extern void updateMTPAParams(MOTOR_Handle handle);

#if defined(SFRA_ENABLE)
//------------------------------------------------------------------------------
// Using SFRA tool :
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
inline void injectSFRA(void)
{
    float32_t sfraNoiseInj_pu = 0.0f;

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;

    sfraNoiseInj_pu = SFRA_F32_inject(0.0f);

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseId = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseIq = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseSpd = sfraNoiseInj_pu * USER_MOTOR1_FREQ_MAX_Hz;
    }

    return;
}

//------------------------------------------------------------------------------
inline void collectSFRA(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseOut = objMtr->IsRef_A * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
        sfraNoiseFdb = objMtr->speed_Hz * (1.0f / USER_MOTOR1_FREQ_MAX_Hz);
    }

    SFRA_F32_collect(&sfraNoiseOut, &sfraNoiseFdb);

    return;
}
//------------------------------------------------------------------------------

#endif  // SFRA_ENABLE


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

#endif // end of _MOTOR_COMMON_H_ definition
