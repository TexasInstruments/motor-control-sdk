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
//!
//! \brief  This project is used to implement motor control with eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM boards
//!

//
// include the related header files
//
#include "sys_settings.h"
#include "sys_main.h"
#include "motor1_drive.h"

// the globals

//!< the hardware abstraction layer object to motor control
__attribute__ ((section("foc_data"))) volatile MOTOR_Handle motorHandle_M1;

__attribute__ ((section("foc_data"))) volatile MOTOR_Vars_t motorVars_M1;

__attribute__ ((section("foc_data"))) MOTOR_SetVars_t motorSetVars_M1;

__attribute__ ((section("foc_data"))) HAL_MTR_Obj    halMtr_M1;

//!< the Id PI controller object
__attribute__ ((section("foc_data"))) PI_Obj        pi_Id_M1;

//!< the Iq PI controller object
__attribute__ ((section("foc_data"))) PI_Obj        pi_Iq_M1;

//!< the speed PI controller object
__attribute__ ((section("foc_data"))) PI_Obj        pi_spd_M1;

#if defined(MOTOR1_OVM)
//!< the handle for the space vector generator current
__attribute__ ((section("foc_data"))) SVGENCURRENT_Obj svgencurrent_M1;
#endif  // MOTOR1_OVM

//!< the speed reference trajectory object
__attribute__ ((section("foc_data"))) TRAJ_Obj     traj_spd_M1;

#if defined(MOTOR1_FWC)
//!< the fwc PI controller object
__attribute__ ((section("foc_data"))) PI_Obj       pi_fwc_M1;
#endif  // MOTOR1_FWC

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
//!< the Angle Generate onject for open loop control
__attribute__ ((section("foc_data"))) ANGLE_GEN_Obj    angleGen_M1;
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
//!< the Vs per Freq object for open loop control
__attribute__ ((section("foc_data"))) VS_FREQ_Obj    VsFreq_M1;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)


#if defined(MOTOR1_ENC)
//!< the handle for the enc object
__attribute__ ((section("foc_data"))) ENC_Obj enc_M1;

//!< the handle for the speedcalc object
__attribute__ ((section("foc_data"))) SPDCALC_Obj speedcalc_M1;
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
//!< the handle for the enc object
__attribute__ ((section("foc_data"))) HALL_Obj hall_M1;


#if (USER_MOTOR1 == Teknic_M2310PLN04K)
const float32_t hallAngleBuf[7] = { 2.60931063f,  -0.45987016f, -2.57219672f, \
                                    -1.50797582f, 1.59862518f , 0.580176473f,
                                    2.60931063f };

#elif (USER_MOTOR1 == Anaheim_BLY172S_24V)
const float32_t hallAngleBuf[7] = { -1.41421735f,  1.75656128f, -2.48391223f, \
                                    2.76515913f,  -0.460148782f, 0.606459916f,
                                    -1.41421735f };
#elif (USER_MOTOR1 == Anaheim_BLWS235D)
const float32_t hallAngleBuf[7] = { 1.64448488f,  -1.54361129f,  0.548367858f, \
                                   -0.390248626f,  2.67842388f, -2.52673817f,
                                    1.64448488f };
#elif (USER_MOTOR1 == Tool_Makita_GFD01)
const float32_t hallAngleBuf[7] = { -2.71645141f,  0.399317622f, 2.47442961f,  \
                                    1.47019732f, -1.67825437f, -0.643157125f, \
                                    -2.71645141f };
#else   // !Teknic_M2310PLN04K | !Anaheim_BLY172S_24V | !Anaheim_BLWS235D | !Tool_Makita_GFD01
#error Not a right hall angle buffer for this project, need to do hall calibration
#endif  //

#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
//!< the speedfr object
__attribute__ ((section("foc_data"))) SPDFR_Obj spdfr_M1;

//!< the esmo object
__attribute__ ((section("foc_data"))) ESMO_Obj   esmo_M1;
#endif  // MOTOR1_ESMO
#if defined(MOTOR1_MTPA)
//!< the Maximum torque per ampere (MTPA) object
__attribute__ ((section("foc_data"))) MTPA_Obj     mtpa_M1;
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)
//!< the single-shunt current reconstruction object
__attribute__ ((section("foc_data"))) DCLINK_SS_Obj    dclink_M1;
#endif // MOTOR1_DCLINKSS

#if defined(MOTOR1_VOLRECT)
//!< the voltage reconstruct object
__attribute__ ((section("foc_data"))) VOLREC_Obj volrec_M1;
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_FILTERIS)
//!< first order current filter object
__attribute__ ((section("foc_data"))) FILTER_FO_Obj    filterIs_M1[3];
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
//!< first order voltage filter object
__attribute__ ((section("foc_data"))) FILTER_FO_Obj    filterVs_M1[3];
#endif  // MOTOR1_FILTERVS

#if (DMC_BUILDLEVEL == DMC_LEVEL_2)
__attribute__ ((section("foc_data"))) volatile float32_t Vdref_level_2 = 0.0f;
__attribute__ ((section("foc_data"))) volatile float32_t Vqref_level_2 = 4.0f;
#endif

#if defined(MOTOR1_SSIPD)
__attribute__ ((section("foc_data"))) SSIPD_Handle    ssipdHandle;
__attribute__ ((section("foc_data"))) SSIPD_Obj       ssipd;
__attribute__ ((section("foc_data"))) volatile Bool flagEnableIPD;
__attribute__ ((section("foc_data"))) volatile float32_t angleOffsetIPD_rad;
__attribute__ ((section("foc_data"))) volatile float32_t angleDetectIPD_rad;
#endif  // MOTOR1_SSIPD

// the control handles for motor 1
void initMotor1Handles(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    obj->motorNum = MTR_1;

    // initialize the driver
    obj->halMtrHandle = HAL_MTR1_init(&halMtr_M1, sizeof(halMtr_M1));

    obj->motorSetsHandle = &motorSetVars_M1;
    obj->userParamsHandle = &userParams_M1;

    return;
}

// initialize control parameters for motor 1
void initMotor1CtrlParameters(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(handle->userParamsHandle);

    // initialize the user parameters
    USER_setMotor1Params(obj->userParamsHandle);

    // set the driver parameters
    HAL_MTR_setParams(obj->halMtrHandle, obj->userParamsHandle);

    objSets->Kp_spd = 0.05f;
    objSets->Ki_spd = 0.005f;

    objSets->Kp_fwc = USER_M1_FWC_KP;
    objSets->Ki_fwc = USER_M1_FWC_KI;

    objSets->angleFWCMax_rad = USER_M1_FWC_MAX_ANGLE_RAD;
    objSets->overModulation = USER_M1_MAX_VS_MAG_PU;

    objSets->RsOnLineCurrent_A = 0.1f * USER_MOTOR1_MAX_CURRENT_A;

    objSets->lostPhaseSet_A = USER_M1_LOST_PHASE_CURRENT_A;
    objSets->unbalanceRatioSet = USER_M1_UNBALANCE_RATIO;
    objSets->overLoadSet_W = USER_M1_OVER_LOAD_POWER_W;

    objSets->toqueFailMinSet_Nm = USER_M1_TORQUE_FAILED_SET;
    objSets->speedFailMaxSet_Hz = USER_M1_FAIL_SPEED_MAX_HZ;
    objSets->speedFailMinSet_Hz = USER_M1_FAIL_SPEED_MIN_HZ;

    objSets->stallCurrentSet_A = USER_M1_STALL_CURRENT_A;
    objSets->IsFailedChekSet_A = USER_M1_FAULT_CHECK_CURRENT_A;

    objSets->maxPeakCurrent_A = USER_M1_ADC_FULL_SCALE_CURRENT_A * 0.475f;
    objSets->overCurrent_A = USER_MOTOR1_OVER_CURRENT_A;
    objSets->currentInv_sf = USER_M1_CURRENT_INV_SF;

    objSets->overVoltageFault_V = USER_M1_OVER_VOLTAGE_FAULT_V;
    objSets->overVoltageNorm_V = USER_M1_OVER_VOLTAGE_NORM_V;
    objSets->underVoltageFault_V = USER_M1_UNDER_VOLTAGE_FAULT_V;
    objSets->underVoltageNorm_V = USER_M1_UNDER_VOLTAGE_NORM_V;

    objSets->overCurrentTimesSet = USER_M1_OVER_CURRENT_TIMES_SET;
    objSets->voltageFaultTimeSet = USER_M1_VOLTAGE_FAULT_TIME_SET;
    objSets->motorStallTimeSet = USER_M1_STALL_TIME_SET;
    objSets->startupFailTimeSet = USER_M1_STARTUP_FAIL_TIME_SET;

    objSets->overSpeedTimeSet = USER_M1_OVER_SPEED_TIME_SET;
    objSets->overLoadTimeSet = USER_M1_OVER_LOAD_TIME_SET;
    objSets->unbalanceTimeSet = USER_M1_UNBALANCE_TIME_SET;
    objSets->lostPhaseTimeSet = USER_M1_LOST_PHASE_TIME_SET;

    objSets->stopWaitTimeSet = USER_M1_STOP_WAIT_TIME_SET;
    objSets->restartWaitTimeSet = USER_M1_RESTART_WAIT_TIME_SET;
    objSets->restartTimesSet = USER_M1_START_TIMES_SET;


    objSets->dacCMPValH = 2048U + 1024U;    // set default positive peak value
    objSets->dacCMPValL = 2048U - 1024U;    // set default negative peak value

    obj->adcData.current_sf = objUser->current_sf * USER_M1_SIGN_CURRENT_SF;

    obj->adcData.voltage_sf = objUser->voltage_sf;
    obj->adcData.dcBusvoltage_sf = objUser->voltage_sf;

    obj->speedStart_Hz = USER_MOTOR1_SPEED_START_Hz;
    obj->speedForce_Hz = USER_MOTOR1_SPEED_FORCE_Hz;
    obj->speedFlyingStart_Hz = USER_MOTOR1_SPEED_FS_Hz;

    obj->accelerationMax_Hzps = USER_MOTOR1_ACCEL_MAX_Hzps;
    obj->accelerationStart_Hzps = USER_MOTOR1_ACCEL_START_Hzps;

    obj->VsRef_pu = 0.98f * USER_M1_MAX_VS_MAG_PU;
    obj->VsRef_V =
            0.98f * USER_M1_MAX_VS_MAG_PU * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V;

    obj->IsSet_A = USER_MOTOR1_TORQUE_CURRENT_A;

    obj->fluxCurrent_A = USER_MOTOR1_FLUX_CURRENT_A;
    obj->alignCurrent_A = USER_MOTOR1_ALIGN_CURRENT_A;
    obj->startCurrent_A = USER_MOTOR1_STARTUP_CURRENT_A;
    obj->maxCurrent_A = USER_MOTOR1_MAX_CURRENT_A;

    obj->angleDelayed_sf = 0.5f * MATH_TWO_PI * USER_M1_CTRL_PERIOD_sec;

    obj->anglePhaseAdj_rad = MATH_PI * 0.001f;

    obj->power_sf = MATH_TWO_PI / USER_MOTOR1_NUM_POLE_PAIRS;
    obj->VIrmsIsrScale = objUser->ctrlFreq_Hz;

    obj->stopWaitTimeCnt = 0;
    obj->flagEnableRestart = FALSE;

    obj->faultMtrMask.all = MTR1_FAULT_MASK_SET;
    obj->operateMode = OPERATE_MODE_SPEED;

    obj->flyingStartTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.5f); // 0.5s
    obj->flyingStartMode = FLYINGSTART_MODE_HALT;

    if(objUser->flag_bypassMotorId == TRUE)
    {
        obj->flagEnableFWC = TRUE;
    }
    else
    {
        obj->flagEnableFWC = FALSE;
    }

    obj->flagEnableForceAngle = TRUE;

    // TRUE - enables flying start, FALSE - disables flying start
    obj->flagEnableFlyingStart = TRUE;

    // TRUE - enables SSIPD start, FALSE - disables SSIPD
    obj->flagEnableSSIPD = FALSE;

    obj->flagEnableSpeedCtrl = TRUE;
    obj->flagEnableCurrentCtrl = TRUE;

    obj->IsSet_A = 0.0f;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);      // 2.0s
    obj->forceRunTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 1.0f);   // 1.0s
    obj->startupTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 2.0f);    // 2.0s

#if defined(MOTOR1_ESMO)
    obj->estimatorMode = ESTIMATOR_MODE_ESMO;

    obj->flagEnableAlignment = TRUE;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
#elif defined(MOTOR1_ENC)
    obj->estimatorMode = ESTIMATOR_MODE_ENC;

    obj->flagEnableAlignment = TRUE;

    obj->alignTimeDelay = (uint16_t)(objUser->ctrlFreq_Hz * 0.1f);      // 0.1s
#elif defined(MOTOR1_HALL)
    obj->estimatorMode = ESTIMATOR_MODE_HALL;

    obj->flagEnableAlignment = TRUE;
#else   // Not select algorithm
#error Not select a right estimator for this project
#endif  // !MOTOR1_ESMO

    obj->speed_int_Hz = 0.0f;
    obj->speed_Hz = 0.0f;

    obj->speedAbs_Hz = 0.0f;
    obj->speedFilter_Hz = 0.0f;

#if defined(MOTOR1_FWC)
    obj->piHandle_fwc = PI_init(&pi_fwc_M1, sizeof(pi_fwc_M1));

    // set the FWC controller
    PI_setGains(obj->piHandle_fwc, USER_M1_FWC_KP, USER_M1_FWC_KI);
    PI_setUi(obj->piHandle_fwc, 0.0);
    PI_setMinMax(obj->piHandle_fwc, USER_M1_FWC_MAX_ANGLE_RAD,
                 USER_M1_FWC_MIN_ANGLE_RAD);
#endif  // MOTOR1_FWC

#ifdef MOTOR1_MTPA
    // initialize the Maximum torque per ampere (MTPA)
    obj->mtpaHandle = MTPA_init(&mtpa_M1, sizeof(mtpa_M1));

    // compute the motor constant for MTPA
    MTPA_computeParameters(obj->mtpaHandle,
                           objUser->motor_Ls_d_H,
                           objUser->motor_Ls_q_H,
                           objUser->motor_ratedFlux_Wb);
#endif  // MOTOR1_MTPA


#if defined(MOTOR1_DCLINKSS)
    obj->dclinkHandle = DCLINK_SS_init(&dclink_M1, sizeof(dclink_M1));

    DCLINK_SS_setInitialConditions(obj->dclinkHandle,
                                   HAL_getTimeBasePeriod(obj->halMtrHandle), 0.5f);

    //disable full sampling
//    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, FALSE);     // default
    DCLINK_SS_setFlag_enableFullSampling(obj->dclinkHandle, TRUE);    // test, not recommend in most cases

    //enable sequence control
//    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, FALSE);  // default
    DCLINK_SS_setFlag_enableSequenceControl(obj->dclinkHandle, TRUE); // test, not recommend in most cases

    // Tdt  =  55 ns (Dead-time between top and bottom switch)
    // Tpd  = 140 ns (Gate driver propagation delay)
    // Tr   = 136 ns (Rise time of amplifier including power switches turn on time)
    // Ts   = 800 ns (Settling time of amplifier)
    // Ts&h = 100 ns (ADC sample&holder = 1+(9)+2 = 12 SYSCLK)
    // T_MinAVDuration = Tdt+Tr+Tpd+Ts+Ts&h
    //                 = 55+140+136+800+100 = 1231(ns) => 148 SYSCLK cycles
    // T_SampleDelay   = Tdt+Tpd+Tr+Ts
    //                 = 55+140+136+800     = 1131(ns) => 136 SYSCLK cycles
    DCLINK_SS_setMinAVDuration(obj->dclinkHandle, USER_M1_DCLINKSS_MIN_DURATION);
    DCLINK_SS_setSampleDelay(obj->dclinkHandle, USER_M1_DCLINKSS_SAMPLE_DELAY);
#endif   // MOTOR1_DCLINKSS

#ifdef MOTOR1_VOLRECT
    // initialize the Voltage reconstruction
    obj->volrecHandle = VOLREC_init(&volrec_M1, sizeof(volrec_M1));

    // configure the Voltage reconstruction
    VOLREC_setParams(obj->volrecHandle,
                     objUser->voltageFilterPole_rps,
                     objUser->ctrlFreq_Hz);

    VOLREC_disableFlagEnableSf(obj->volrecHandle);
#endif  // MOTOR1_VOLRECT

#if defined(MOTOR1_ESMO)
    // initialize the esmo
    obj->esmoHandle = ESMO_init(&esmo_M1, sizeof(esmo_M1));

    // set parameters for ESMO controller
    ESMO_setKslideParams(obj->esmoHandle,
                         USER_MOTOR1_KSLIDE_MAX, USER_MOTOR1_KSLIDE_MIN);

    ESMO_setPLLParams(obj->esmoHandle, USER_MOTOR1_PLL_KP_MAX,
                      USER_MOTOR1_PLL_KP_MIN, USER_MOTOR1_PLL_KP_SF);

    ESMO_setPLLKi(obj->esmoHandle, USER_MOTOR1_PLL_KI);   // Optional

    ESMO_setBEMFThreshold(obj->esmoHandle, USER_MOTOR1_BEMF_THRESHOLD);
    ESMO_setOffsetCoef(obj->esmoHandle, USER_MOTOR1_THETA_OFFSET_SF);
    ESMO_setBEMFKslfFreq(obj->esmoHandle, USER_MOTOR1_BEMF_KSLF_FC_SF);
    ESMO_setSpeedFilterFreq(obj->esmoHandle, USER_MOTOR1_SPEED_LPF_FC_Hz);

    // set the ESMO controller parameters
    ESMO_setParams(obj->esmoHandle, obj->userParamsHandle);

    // initialize the spdfr
    obj->spdfrHandle = SPDFR_init(&spdfr_M1, sizeof(spdfr_M1));

    // set the spdfr parameters
    SPDFR_setParams(obj->spdfrHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;
#endif  //MOTOR1_ESMO


#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
    // initialize the angle generate module
    obj->angleGenHandle = ANGLE_GEN_init(&angleGen_M1, sizeof(angleGen_M1));

    ANGLE_GEN_setParams(obj->angleGenHandle, objUser->ctrlPeriod_sec);
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT || MOTOR1_ENC

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->Idq_set_A.value[0] = 0.0f;
    obj->Idq_set_A.value[1] = obj->startCurrent_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    // initialize the Vs per Freq module
    obj->VsFreqHandle = VS_FREQ_init(&VsFreq_M1, sizeof(VsFreq_M1));

    VS_FREQ_setVsMagPu(obj->VsFreqHandle, objUser->maxVsMag_pu);

    VS_FREQ_setMaxFreq(obj->VsFreqHandle, USER_MOTOR1_FREQ_MAX_Hz);

    VS_FREQ_setProfile(obj->VsFreqHandle,
                       USER_MOTOR1_FREQ_LOW_Hz, USER_MOTOR1_FREQ_HIGH_Hz,
                       USER_MOTOR1_VOLT_MIN_V, USER_MOTOR1_VOLT_MAX_V);
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)


#if defined(MOTOR1_ENC)
    // initialize the enc handle
    obj->encHandle = ENC_init(&enc_M1, sizeof(enc_M1));

    // set the ENC controller parameters
    ENC_setQEPHandle(obj->encHandle, MTR1_QEP_BASE);
    ENC_setParams(obj->encHandle, obj->userParamsHandle);

    // initialize the apll handle
    obj->spdcalcHandle = SPDCALC_init(&speedcalc_M1, sizeof(speedcalc_M1));

    // set the SPEEDCALC controller parameters
    SPDCALC_setParams(obj->spdcalcHandle, obj->userParamsHandle);

    obj->frswPos_sf = 0.6f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_ENC



#if defined(MOTOR1_HALL)
    // initialize the hall handle
    obj->hallHandle = HALL_init(&hall_M1, sizeof(hall_M1));

    // set the HALL controller parameters
    HALL_setParams(obj->hallHandle, obj->userParamsHandle);
    HALL_setAngleBuf(obj->hallHandle, &hallAngleBuf[0]);
    HALL_setAngleDelta_rad(obj->hallHandle, USER_MOTOR1_HALL_DELTA_rad);
    HALL_setGPIOs(obj->hallHandle,
                  MTR1_HALL_U_GPIO, MTR1_HALL_V_GPIO, MTR1_HALL_W_GPIO,
                  MTR1_HALL_U_GPIO_BASE_ADDR, MTR1_HALL_V_GPIO_BASE_ADDR, MTR1_HALL_W_GPIO_BASE_ADDR);

    obj->frswPos_sf = 1.0f;     // Tune this coefficient per the motor/system
#endif  // MOTOR1_HALL

    // initialize the PI controllers
    obj->piHandle_Id  = PI_init(&pi_Id_M1, sizeof(pi_Id_M1));
    obj->piHandle_Iq  = PI_init(&pi_Iq_M1, sizeof(pi_Iq_M1));
    obj->piHandle_spd = PI_init(&pi_spd_M1, sizeof(pi_spd_M1));

    // initialize the speed reference trajectory
    obj->trajHandle_spd = TRAJ_init(&traj_spd_M1, sizeof(traj_spd_M1));

    // configure the speed reference trajectory (Hz)
    TRAJ_setTargetValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    TRAJ_setMinValue(obj->trajHandle_spd, -objUser->maxFrequency_Hz);
    TRAJ_setMaxValue(obj->trajHandle_spd, objUser->maxFrequency_Hz);
    TRAJ_setMaxDelta(obj->trajHandle_spd, (objUser->maxAccel_Hzps * objUser->ctrlPeriod_sec));

#if !defined(MOTOR1_DCLINKSS)   // 2/3-shunt
    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.01f, 0.01f, 0.14f);
#else   // MOTOR1_DCLINKSS
    HAL_setTriggerPrams(&obj->pwmData, USER_SYSTEM_FREQ_MHz,
                        0.0f, 0.0f, 0.0f);
#endif  // MOTOR1_DCLINKSS

#if defined(MOTOR1_SSIPD)
    ssipdHandle = SSIPD_init(&ssipd, sizeof(ssipd));
    SSIPD_setParams(ssipdHandle, 0.75f, (MATH_TWO_PI / SSIPD_DETECT_NUM), 6);

    angleOffsetIPD_rad = MATH_PI / SSIPD_DETECT_NUM;

    flagEnableIPD = TRUE; //FALSE;
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_FILTERIS)
    obj->flagEnableFilterIs = TRUE;

    // assign the current filter handle (low pass filter)
    obj->filterHandle_Is[0] = FILTER_FO_init((void *)(&filterIs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[1] = FILTER_FO_init((void *)(&filterIs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Is[2] = FILTER_FO_init((void *)(&filterIs_M1[2]), sizeof(FILTER_FO_Obj));

    obj->filterIsPole_rps = USER_M1_IS_FILTER_POLE_rps;     //

    float32_t beta_lp_Is = obj->filterIsPole_rps * objUser->ctrlPeriod_sec;

    float32_t a1_Is = (beta_lp_Is - (float32_t)2.0f) / (beta_lp_Is + (float32_t)2.0f);
    float32_t b0_Is = beta_lp_Is / (beta_lp_Is + (float32_t)2.0f);
    float32_t b1_Is = b0_Is;

    // set filter coefficients for current filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[0], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[0], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[1], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[1], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Is[2], b0_Is, b1_Is);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Is[2], a1_Is);
    FILTER_FO_setInitialConditions(obj->filterHandle_Is[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_FILTERVS)
    obj->flagEnableFilterVs = TRUE;

    // assign the voltage filter handle (low pass filter)
    obj->filterHandle_Vs[0] = FILTER_FO_init((void *)(&filterVs_M1[0]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[1] = FILTER_FO_init((void *)(&filterVs_M1[1]), sizeof(FILTER_FO_Obj));
    obj->filterHandle_Vs[2] = FILTER_FO_init((void *)(&filterVs_M1[2]), sizeof(FILTER_FO_Obj));

    obj->filterVsPole_rps = USER_M1_VS_FILTER_POLE_rps;

    float32_t beta_lp_Vs = obj->filterVsPole_rps * objUser->ctrlPeriod_sec;

    float32_t a1_Vs = (beta_lp_Vs - (float32_t)2.0f) / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b0_Vs = beta_lp_Vs / (beta_lp_Vs + (float32_t)2.0f);
    float32_t b1_Vs = b0_Vs;

    // set filter coefficients for voltage filters (low pass filter)
    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[0], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[0], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[0], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[1], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[1], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[1], 0.0f, 0.0f);

    FILTER_FO_setNumCoeffs(obj->filterHandle_Vs[2], b0_Vs, b1_Vs);
    FILTER_FO_setDenCoeffs(obj->filterHandle_Vs[2], a1_Vs);
    FILTER_FO_setInitialConditions(obj->filterHandle_Vs[2], 0.0f, 0.0f);
#endif  // MOTOR1_FILTERVS

#if defined(MOTOR1_OVM)
    // Initialize and setup the 100% SVM generator
    obj->svgencurrentHandle =
            SVGENCURRENT_init(&svgencurrent_M1, sizeof(svgencurrent_M1));

    SVGENCURRENT_setup(obj->svgencurrentHandle, 1.0f,
                       USER_M1_PWM_FREQ_kHz, USER_SYSTEM_FREQ_MHz);
#endif  // MOTOR1_OVM

#ifdef BRAKE_ENABLE

    obj->brakingCurrent_A = USER_MOTOR1_BRAKE_CURRENT_A;

    obj->brakingTimeDelay = USER_MOTOR1_BRAKE_TIME_DELAY;

    obj->flagEnableBraking = FALSE;
    obj->brakingMode = HARDSWITCH_BRAKE_MODE;
#endif  // BRAKE_ENABLE

#if defined(MOTOR1_RPM_CMD)
    obj->flagCmdRpmOrHz = FALSE;     // the speed command is rpm
    obj->rpm2Hz_sf = objUser->motor_numPolePairs / 60.0f;
    obj->hz2Rpm_sf = 60.0f / objUser->motor_numPolePairs;
#endif  // MOTOR1_RPM_CMD


    // setup the controllers, speed, d/q-axis current pid regulator
    setupControllers(handle);

#if defined(MOTOR1_PI_TUNE)
    // set the coefficient of the controllers gains
    setupControllerSF(handle);
#endif      // MOTOR1_PI_TUNE

    // disable the PWM
    HAL_disablePWM(obj->halMtrHandle);

    return;
}   // end of initMotor1CtrlParameters() function

void runMotor1OffsetsCalculation(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;

    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);


    //Calculate motor protection value
    calcMotorOverCurrentThreshold(handle);

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

#if defined(MOTOR1_DCLINKSS)
    HAL_MTR_Obj *objHal = (HAL_MTR_Obj *)(obj->halMtrHandle);

    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, 5);

    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, 5);
    EPWM_setCounterCompareValue(objHal->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, 5);
#endif  // MOTOR1_DCLINKSS

        // Offsets in phase current sensing
#if defined(MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                              MTR1_IDC1_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                              MTR1_IDC2_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                              MTR1_IDC3_ADC_PPB_NUM);

    ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                              MTR1_IDC4_ADC_PPB_NUM);

    obj->adcData.offset_Idc_ad = USER_M1_IDC_OFFSET_AD;
#else // !(MOTOR1_DCLINKSS)
    ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                              (uint16_t)USER_M1_IA_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                              (uint16_t)USER_M1_IB_OFFSET_AD);

    ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                              (uint16_t)USER_M1_IC_OFFSET_AD);

    obj->adcData.offset_I_ad.value[0]  = USER_M1_IA_OFFSET_AD;
    obj->adcData.offset_I_ad.value[1]  = USER_M1_IB_OFFSET_AD;
    obj->adcData.offset_I_ad.value[2]  = USER_M1_IC_OFFSET_AD;
#endif // !(MOTOR1_DCLINKSS)

    if(obj->flagEnableOffsetCalc == TRUE)
    {
        float32_t offsetK1 = 0.998001f;  // Offset filter coefficient K1: 0.05/(T+0.05);
        float32_t offsetK2 = 0.001999f;  // Offset filter coefficient K2: T/(T+0.05);
        float32_t invCurrentSf = 1.0f / obj->adcData.current_sf;

        uint16_t offsetCnt;

        ClockP_usleep(2L);      // delay 2us

#if defined(MOTOR1_DCLINKSS)
        HAL_setOffsetTrigger(obj->halMtrHandle);

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);

        obj->adcData.offset_Idc_ad  = USER_M1_IDC_OFFSET_AD * USER_M1_CURRENT_SF;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

#else  // !(MOTOR1_DCLINKSS)
        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);
        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);

        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * obj->adcData.current_sf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * obj->adcData.current_sf;

        // Set the 3-phase output PWMs to 50% duty cycle
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;

        // write the PWM compare values
        HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif // !(MOTOR1_DCLINKSS)

        //Enable the PWM
        HAL_enablePWM(obj->halMtrHandle);

        for(offsetCnt = 0; offsetCnt < 32000; offsetCnt++)
        {
            // clear the ADC interrupt flag
            ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

            while(ADC_getInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM) == FALSE);

            HAL_readMtr1ADCData(&obj->adcData);

            if(offsetCnt >= 2000)       // Ignore the first 2000 times
            {
                // Offsets in phase current sensing
#if defined(MOTOR1_DCLINKSS)
                obj->adcData.offset_Idc_ad = offsetK1 * obj->adcData.offset_Idc_ad +
                               0.25f * offsetK2 *(obj->adcData.Idc1_A.value[0] +
                                                  obj->adcData.Idc1_A.value[1] +
                                                  obj->adcData.Idc2_A.value[0] +
                                                  obj->adcData.Idc2_A.value[1]);
#else // (MOTOR1_DCLINKSS)
                obj->adcData.offset_I_ad.value[0] =
                        offsetK1 * obj->adcData.offset_I_ad.value[0] +
                        obj->adcData.I_A.value[0] * offsetK2;

                obj->adcData.offset_I_ad.value[1] =
                        offsetK1 * obj->adcData.offset_I_ad.value[1] +
                        obj->adcData.I_A.value[1] * offsetK2;

                obj->adcData.offset_I_ad.value[2] =
                        offsetK1 * obj->adcData.offset_I_ad.value[2] +
                        obj->adcData.I_A.value[2] * offsetK2;
#endif // !(MOTOR1_DCLINKSS)

            }
            else if(offsetCnt <= 1000)
            {
                // enable the PWM
                HAL_enablePWM(obj->halMtrHandle);
            }
        } // for()

        // disable the PWM
        HAL_disablePWM(obj->halMtrHandle);

#if defined(MOTOR1_DCLINKSS)
        obj->adcData.offset_Idc_ad = obj->adcData.offset_Idc_ad * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);

        ADC_setPPBReferenceOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_Idc_ad);
#else // !(MOTOR1_DCLINKSS)
        obj->adcData.offset_I_ad.value[0] =
                 obj->adcData.offset_I_ad.value[0] * invCurrentSf;
        obj->adcData.offset_I_ad.value[1] =
                 obj->adcData.offset_I_ad.value[1] * invCurrentSf;
        obj->adcData.offset_I_ad.value[2] =
                 obj->adcData.offset_I_ad.value[2] * invCurrentSf;

        ADC_setPPBReferenceOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[0]);

        ADC_setPPBReferenceOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[1]);

        ADC_setPPBReferenceOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM,
                                  (uint16_t)obj->adcData.offset_I_ad.value[2]);
#endif // (MOTOR1_DCLINKSS)
    }   // flagEnableOffsetCalc = TRUE

#if defined(MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_Idc_ad > USER_M1_IDC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_Idc_ad < USER_M1_IDC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#else // !(MOTOR1_DCLINKSS)
    // Check current and voltage offset
    if( (obj->adcData.offset_I_ad.value[0] > USER_M1_IA_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[0] < USER_M1_IA_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[1] > USER_M1_IB_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[1] < USER_M1_IB_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }

    if( (obj->adcData.offset_I_ad.value[2] > USER_M1_IC_OFFSET_AD_MAX) ||
        (obj->adcData.offset_I_ad.value[2] < USER_M1_IC_OFFSET_AD_MIN) )
    {
        obj->faultMtrNow.bit.currentOffset = 1;
    }
#endif // (MOTOR1_DCLINKSS)

    if((obj->faultMtrNow.bit.voltageOffset == 0) &&
            (obj->faultMtrNow.bit.currentOffset == 0))
    {
        obj->flagEnableOffsetCalc = FALSE;
    }

    return;
} // end of runMotor1OffsetsCalculation() function



void runMotor1Control(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(obj->motorSetsHandle);
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == TRUE)
    {
        if(HAL_getMtrTripFaults(obj->halMtrHandle) != 0)
        {
            obj->faultMtrNow.bit.moduleOverCurrent = 1;
        }
    }

    obj->faultMtrPrev.all |= obj->faultMtrNow.all;
    obj->faultMtrUse.all = obj->faultMtrNow.all & obj->faultMtrMask.all;

    HAL_setMtrCMPSSDACValue(obj->halMtrHandle,
                            objSets->dacCMPValH, objSets->dacCMPValL);

    if(obj->flagClearFaults == TRUE)
    {
        HAL_clearMtrFaultStatus(obj->halMtrHandle);

        obj->faultMtrNow.all &= MTR_FAULT_CLEAR;
        obj->flagClearFaults = FALSE;
    }

    if(obj->flagEnableRunAndIdentify == TRUE)
    {
        // Had some faults to stop the motor
        if(obj->faultMtrUse.all != 0)
        {
            if(obj->flagRunIdentAndOnLine == TRUE)
            {
                obj->flagRunIdentAndOnLine = FALSE;
                obj->motorState = MOTOR_FAULT_STOP;

                obj->stopWaitTimeCnt = objSets->restartWaitTimeSet;
                obj->restartTimesCnt++;

                if(obj->flagEnableRestart == FALSE)
                {
                    obj->flagEnableRunAndIdentify = FALSE;
                    obj->stopWaitTimeCnt = 0;
                }
            }
            else if(obj->stopWaitTimeCnt == 0)
            {
                if(obj->restartTimesCnt < objSets->restartTimesSet)
                {
                    obj->flagClearFaults = 1;
                }
                else
                {
                    obj->flagEnableRunAndIdentify = FALSE;
                }
            }
        }
        // Restart
        else if((obj->flagRunIdentAndOnLine == FALSE) &&
                (obj->stopWaitTimeCnt == 0))
        {
        #if defined(MOTOR1_SSIPD)
            if(flagEnableIPD == TRUE)
            {
                if(SSIPD_getDoneStatus(ssipdHandle) == TRUE)
                {
                    if(obj->speedRef_Hz > 0.0f)
                    {
                        angleDetectIPD_rad = SSIPD_getAngleOut_rad(ssipdHandle) -
                                               angleOffsetIPD_rad;

                    }
                    else
                    {
                        angleDetectIPD_rad = SSIPD_getAngleOut_rad(ssipdHandle) +
                                               angleOffsetIPD_rad;
                    }

                    if(angleDetectIPD_rad < 0.0f)
                    {
                        angleDetectIPD_rad += MATH_TWO_PI;
                    }
                    else if(angleDetectIPD_rad > MATH_TWO_PI)
                    {
                        angleDetectIPD_rad -= MATH_TWO_PI;
                    }

                    restartMotorControl(handle);
                    obj->angleCurrent_rad = MATH_PI_OVER_TWO;

                    #if defined(MOTOR1_ESMO)
                    ESMO_resetParams(obj->esmoHandle);
                    #endif  //MOTOR1_ESMO
                }
                else if(SSIPD_getRunState(ssipdHandle) == TRUE)
                {
                    if(SSIPD_getFlagEnablePWM(ssipdHandle) == TRUE)
                    {
                        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == FALSE)
                        {
                            // enable the PWM for motor_1
                            HAL_enablePWM(obj->halMtrHandle);
                        }
                    }
                    else
                    {
                        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == TRUE)
                        {
                            // disable the PWM
                            HAL_disablePWM(obj->halMtrHandle);
                        }
                    }
                }
                else
                {
                    SSIPD_start(ssipdHandle);
                }
            }
            else
            {
                restartMotorControl(handle);
                obj->angleCurrent_rad = MATH_PI_OVER_TWO;

                #if defined(MOTOR1_ESMO)
                ESMO_resetParams(obj->esmoHandle);
                #endif  //MOTOR1_ESMO
            }
        #else  // !MOTOR1_SSIPD
            restartMotorControl(handle);
        #endif  // !MOTOR1_SSIPD
        }
    }
    else if(obj->flagRunIdentAndOnLine == TRUE)
    {
        stopMotorControl(handle);

        if(obj->flagEnableFlyingStart == FALSE)
        {
            obj->stopWaitTimeCnt = objSets->stopWaitTimeSet;
        }
        else
        {
            obj->stopWaitTimeCnt = 0;
        }
    }
    else
    {
        #if defined(MOTOR1_SSIPD)
        // Reset
        if(SSIPD_getDoneStatus(ssipdHandle) == TRUE)
        {
            SSIPD_reset(ssipdHandle);
        }
        #endif // MOTOR1_SSIPD
    }

    if(obj->flagRunIdentAndOnLine == TRUE)
    {
//        HAL_enablePWM(obj->halMtrHandle);
        if(HAL_getPwmEnableStatus(obj->halMtrHandle) == FALSE)
        {
            // enable the PWM
            HAL_enablePWM(obj->halMtrHandle);
        }


//        {


            if(obj->speedRef_Hz > 0.0f)
            {
                obj->direction = 1.0f;
            }
            else
            {
                obj->direction = -1.0f;
            }

            // Sets the target speed for the speed trajectory
        #if defined(MOTOR1_ESMO)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_ENC)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #elif defined(MOTOR1_HALL)
            if(obj->motorState >= MOTOR_CL_RUNNING)
            {
                TRAJ_setTargetValue(obj->trajHandle_spd, obj->speedRef_Hz);
            }
            else
            {
                TRAJ_setTargetValue(obj->trajHandle_spd,
                                    (obj->speedForce_Hz * obj->direction));
            }
        #else   // !MOTOR1_ESMO
        #error No select a right estimator for motor_1 control
        #endif  // MOTOR1_ESMO

            if((fabs(obj->speed_Hz) > obj->speedStart_Hz) ||
                    (obj->motorState == MOTOR_CTRL_RUN))
            {
                //  Sets the acceleration / deceleration for the speed trajectory
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationMax_Hzps * objUser->ctrlPeriod_sec));

                PI_setMinMax(obj->piHandle_spd, -obj->maxCurrent_A, obj->maxCurrent_A);

                if(obj->motorState == MOTOR_CL_RUNNING)
                {
                    obj->stateRunTimeCnt++;

                    if(obj->stateRunTimeCnt == obj->startupTimeDelay)
                    {
                        obj->Idq_out_A.value[0] = 0.0f;
                        obj->motorState = MOTOR_CTRL_RUN;
                    }
                }
            }
            else
            {
                TRAJ_setMaxDelta(obj->trajHandle_spd,
                  (obj->accelerationStart_Hzps * objUser->ctrlPeriod_sec));

                if(obj->speed_int_Hz >= 0.0f)
                {
                    PI_setMinMax(obj->piHandle_spd, 0.0f, obj->startCurrent_A);
                }
                else
                {
                    PI_setMinMax(obj->piHandle_spd, -obj->startCurrent_A, 0.0f);
                }
            }
//        }

        // Identification
#if(DMC_BUILDLEVEL == DMC_LEVEL_3)
        obj->Idq_out_A.value[0] = obj->Idq_set_A.value[0];
        obj->Idq_out_A.value[1] = obj->Idq_set_A.value[1] * obj->direction;

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)
    }
#if defined(MOTOR1_SSIPD)
    else if(SSIPD_getRunState(ssipdHandle) == FALSE)
#else
    else
#endif  // MOTOR1_SSIPD
    {
        // reset motor control parameters
        resetMotorControl(handle);
    }

    obj->flagMotorIdentified = TRUE;


    if(obj->flagMotorIdentified == TRUE)
    {
        if(obj->flagSetupController == TRUE)
        {
            // update the controller
            updateControllers(handle);
        }
        else
        {
            obj->flagSetupController = TRUE;

            setupControllers(handle);
        }
    }


    // update the global variables
    updateGlobalVariables(handle);

#if defined(MOTOR1_ESMO)
    if(obj->motorState >= MOTOR_CTRL_RUN)
    {
        ESMO_updateFilterParams(obj->esmoHandle);
        ESMO_updatePLLParams(obj->esmoHandle);
    }
#endif  // MOTOR1_ESMO

    return;
}   // end of the runMotor1Control() function

__attribute__ ((section(".tcm_code"))) void motor1CtrlISR(void *handle)
{

#if defined(CPUTIME_ENABLE)
    CycleCounterP_reset();
    cycleCountBefore = CycleCounterP_getCount32();
#endif  // CPUTIME_ENABLE

    motorVars_M1.ISRCount++;

    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)motorHandle_M1;
    USER_Params *objUser = (USER_Params *)(obj->userParamsHandle);

    // acknowledge the ADC interrupt
    HAL_ackMtr1ADCInt();

    // read the ADC data with offsets
    HAL_readMtr1ADCData(&obj->adcData);

#if defined(BSXL8316RT_REVA) && defined(OFFSET_CORRECTION)
    // CSA Offset correction
    I_correct_A.value[0] = 0.995832f * obj->adcData.I_A.value[0] -
                           0.028199f * obj->adcData.I_A.value[1] -
                           0.014988f * obj->adcData.I_A.value[2];

    I_correct_A.value[1] = 0.037737f * obj->adcData.I_A.value[0] +
                           1.007723f * obj->adcData.I_A.value[1] -
                           0.033757f * obj->adcData.I_A.value[2];

    I_correct_A.value[2] = 0.009226f * obj->adcData.I_A.value[0] +
                           0.029805f * obj->adcData.I_A.value[1] +
                           1.003268f * obj->adcData.I_A.value[2];

    obj->adcData.I_A.value[0] = I_correct_A.value[0];
    obj->adcData.I_A.value[1] = I_correct_A.value[1];
    obj->adcData.I_A.value[2] = I_correct_A.value[2];

#endif  // BSXL8316RT_REVA & OFFSET_CORRECTION
//------------------------------------------------------------------------------

#if defined(MOTOR1_DCLINKSS)
    // run single-shunt current reconstruction
    DCLINK_SS_runCurrentReconstruction(obj->dclinkHandle,
                                     &obj->adcData.Idc1_A, &obj->adcData.Idc2_A);

    obj->sector = DCLINK_SS_getSector1(obj->dclinkHandle);

    obj->adcData.I_A.value[0] = DCLINK_SS_getIa(obj->dclinkHandle);
    obj->adcData.I_A.value[1] = DCLINK_SS_getIb(obj->dclinkHandle);
    obj->adcData.I_A.value[2] = DCLINK_SS_getIc(obj->dclinkHandle);

#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == TRUE)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS
#else // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_FILTERIS)
    // run first order filters for current sensing
    obj->adcIs_A.value[0] = FILTER_FO_run(obj->filterHandle_Is[0], obj->adcData.I_A.value[0]);
    obj->adcIs_A.value[1] = FILTER_FO_run(obj->filterHandle_Is[1], obj->adcData.I_A.value[1]);
    obj->adcIs_A.value[2] = FILTER_FO_run(obj->filterHandle_Is[2], obj->adcData.I_A.value[2]);

    if(obj->flagEnableFilterIs == TRUE)
    {
        obj->adcData.I_A.value[0] = obj->adcIs_A.value[0];
        obj->adcData.I_A.value[1] = obj->adcIs_A.value[1];
        obj->adcData.I_A.value[2] = obj->adcIs_A.value[2];
    }
#endif  // MOTOR1_FILTERIS

#if defined(MOTOR1_OVM)
    // Over Modulation Supporting, run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(obj->svgencurrentHandle,
                                 &obj->adcData.I_A, &obj->adcDataPrev);
#endif  // MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

//------------------------------------------------------------------------------
#if defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run_threeInput(obj->adcData.I_A.value[0], obj->adcData.I_A.value[1], obj->adcData.I_A.value[2], &obj->Iab_A.value[0], &obj->Iab_A.value[1]);

    if(obj->flagRunIdentAndOnLine == TRUE)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = FALSE;
        obj->enableCurrentCtrl = FALSE;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;

    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG


    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);

    // run the encoder
    ENC_inline_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabs(obj->speedFilter_Hz);

    // Running state
    obj->stateRunTimeCnt++;

    if(obj->estimatorMode == ESTIMATOR_MODE_ESMO)
    {
        obj->speed_Hz = obj->speedPLL_Hz;

        if(obj->motorState >= MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->anglePLL_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = FALSE;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(fabs(obj->speed_int_Hz) >= obj->speedForce_Hz)
            {
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

                if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
                {
                    obj->motorState = MOTOR_CL_RUNNING;
                    obj->stateRunTimeCnt = 0;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                    PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
                }
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = FALSE;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ESMO_setAnglePu(obj->esmoHandle, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == FALSE))
            {
                obj->motorState = MOTOR_OL_START;
                obj->stateRunTimeCnt = 0;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);

                PI_setUi(obj->piHandle_spd, 0.0f);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = FALSE;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }
    else    // if(obj->estimatorMode == ESTIMATOR_MODE_ENC)
    {
        obj->speed_Hz = obj->speedENC_Hz;

        if(obj->motorState >= MOTOR_CTRL_RUN)
        {
            obj->angleFOC_rad = obj->angleENC_rad;

            ESMO_updateKslide(obj->esmoHandle);
        }
        else if(obj->motorState == MOTOR_CL_RUNNING)
        {
            obj->angleFOC_rad = obj->angleENC_rad;
            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        }
        else if(obj->motorState == MOTOR_OL_START)
        {
            obj->angleFOC_rad = obj->angleGen_rad;
            obj->enableSpeedCtrl = FALSE;

            obj->Idq_out_A.value[0] = 0.0f;

            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->startCurrent_A;
                obj->Idq_out_A.value[1] = obj->startCurrent_A;
            }
            else
            {
                obj->IsRef_A = -obj->startCurrent_A;
                obj->Idq_out_A.value[1] = -obj->startCurrent_A;
            }

            if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
            {
                obj->motorState = MOTOR_CL_RUNNING;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
            }
        }
        else if(obj->motorState == MOTOR_ALIGNMENT)
        {
            obj->angleFOC_rad = 0.0f;
            obj->enableSpeedCtrl = FALSE;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = obj->alignCurrent_A;
            obj->Idq_out_A.value[1] = 0.0f;

            TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
            ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

            ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

            if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                     (obj->flagEnableAlignment == FALSE))
            {
                obj->stateRunTimeCnt = 0;
                obj->motorState = MOTOR_OL_START;

                obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

                ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
                PI_setUi(obj->piHandle_spd, 0.0);
            }
        }
        else if(obj->motorState == MOTOR_SEEK_POS)
        {
            obj->enableSpeedCtrl = FALSE;

            obj->IsRef_A = 0.0f;
            obj->Idq_out_A.value[0] = 0.0f;
            obj->Idq_out_A.value[1] = 0.0f;

            if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
            {
                obj->stateRunTimeCnt = 0;

                if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
                {
                    obj->speed_int_Hz = obj->speedFilter_Hz;
                    TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                    PI_setUi(obj->piHandle_spd, 0.0f);

                    obj->motorState = MOTOR_CL_RUNNING;

                    ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
                }
                else
                {
                    obj->angleFOC_rad = 0.0f;
                    obj->motorState = MOTOR_ALIGNMENT;
                }
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = cosf(obj->angleFOC_rad);
    phasor.value[1] = sinf(obj->angleFOC_rad);

    PARK_run(phasor.value[1], phasor.value[0], obj->Iab_A.value[0], obj->Iab_A.value[1], &obj->Idq_in_A.value[0], &obj->Idq_in_A.value[1]);

// End of MOTOR1_ESMO && MOTOR1_ENC

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ESMO)
    // sensorless-FOC
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run_threeInput(obj->adcData.I_A.value[0], obj->adcData.I_A.value[1], obj->adcData.I_A.value[2], &obj->Iab_A.value[0], &obj->Iab_A.value[1]);

    if(obj->flagRunIdentAndOnLine == TRUE)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = FALSE;
        obj->enableCurrentCtrl = FALSE;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    // run the eSMO
    ESMO_setSpeedRef(obj->esmoHandle, obj->speed_int_Hz);
    ESMO_run(obj->esmoHandle, obj->adcData.VdcBus_V,
                    &(obj->pwmData.Vabc_pu), &(obj->Iab_A));

    obj->anglePLLComp_rad = obj->speedPLL_Hz * obj->angleDelayed_sf;
    obj->anglePLL_rad = MATH_incrAngle(ESMO_getAnglePLL(obj->esmoHandle), obj->anglePLLComp_rad);
#if defined(ESMO_DEBUG)
    obj->angleSMO_rad = ESMO_getAngleElec(obj->esmoHandle);
#endif  //ESMO_DEBUG

    SPDFR_run(obj->spdfrHandle, obj->anglePLL_rad);
    obj->speedPLL_Hz = SPDFR_getSpeedHz(obj->spdfrHandle);
    obj->speed_Hz = obj->speedPLL_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabs(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->anglePLL_rad;

        ESMO_updateKslide(obj->esmoHandle);
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = FALSE;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(fabs(obj->speed_int_Hz) >= obj->speedForce_Hz)
        {
            TRAJ_setIntValue(obj->trajHandle_spd, obj->speed_int_Hz);

            if(obj->stateRunTimeCnt > obj->forceRunTimeDelay)
            {
                obj->motorState = MOTOR_CL_RUNNING;
                obj->stateRunTimeCnt = 0;

                ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);

                PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
            }
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = FALSE;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ESMO_setAnglePu(obj->esmoHandle, obj->angleFOC_rad);
        ANGLE_GEN_setAngle(obj->angleGenHandle, obj->angleFOC_rad);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == FALSE))
        {
            obj->motorState = MOTOR_OL_START;
            obj->stateRunTimeCnt = 0;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = FALSE;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = cosf(obj->angleFOC_rad);
    phasor.value[1] = sinf(obj->angleFOC_rad);

    PARK_run(phasor.value[1], phasor.value[0], obj->Iab_A.value[0], obj->Iab_A.value[1], &obj->Idq_in_A.value[0], &obj->Idq_in_A.value[1]);

// End of MOTOR1_ESMO

//------------------------------------------------------------------------------
#elif defined(MOTOR1_ENC)
    MATH_Vec2 phasor;

    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);

    // run Clarke transform on current
    CLARKE_run_threeInput(obj->adcData.I_A.value[0], obj->adcData.I_A.value[1], obj->adcData.I_A.value[2], &obj->Iab_A.value[0], &obj->Iab_A.value[1]);

    if(obj->flagRunIdentAndOnLine == TRUE)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = FALSE;
        obj->enableCurrentCtrl = FALSE;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    ENC_run(obj->encHandle);
    obj->angleENC_rad = ENC_getElecAngle(obj->encHandle);

    SPDCALC_run(obj->spdcalcHandle, obj->angleENC_rad);
    obj->speedENC_Hz = SPDCALC_getSpeedHz(obj->spdcalcHandle);

    obj->speed_Hz = obj->speedENC_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabs(obj->speedFilter_Hz);

    obj->stateRunTimeCnt++;

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleENC_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleGen_rad;
        obj->enableSpeedCtrl = FALSE;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        if(ENC_getState(obj->encHandle) == ENC_CALIBRATION_DONE)
        {
            obj->motorState = MOTOR_CL_RUNNING;
        }
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = FALSE;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        ANGLE_GEN_setAngle(obj->angleGenHandle, 0.0f);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == FALSE))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            ENC_setState(obj->encHandle, ENC_WAIT_FOR_INDEX);
            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = FALSE;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = cosf(obj->angleFOC_rad);
    phasor.value[1] = sinf(obj->angleFOC_rad);

    // run the Park transform
    PARK_run(phasor.value[1], phasor.value[0], obj->Iab_A.value[0], obj->Iab_A.value[1], &obj->Idq_in_A.value[0], &obj->Idq_in_A.value[1]);

// End of MOTOR1_ENC
//------------------------------------------------------------------------------

#elif defined(MOTOR1_HALL)
    MATH_Vec2 phasor;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3)
    ANGLE_GEN_run(obj->angleGenHandle, obj->speed_int_Hz);
    obj->angleGen_rad = ANGLE_GEN_getAngle(obj->angleGenHandle);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    // run Clarke transform on current
    CLARKE_run_threeInput(obj->adcData.I_A.value[0], obj->adcData.I_A.value[1], obj->adcData.I_A.value[2], &obj->Iab_A.value[0], &obj->Iab_A.value[1]);

    if(obj->flagRunIdentAndOnLine == TRUE)
    {
        obj->counterTrajSpeed++;

        if(obj->counterTrajSpeed >= objUser->numIsrTicksPerTrajTick)
        {
            // clear counter
            obj->counterTrajSpeed = 0;

            // run a trajectory for speed reference,
            // so the reference changes with a ramp instead of a step
            TRAJ_run(obj->trajHandle_spd);
        }

        obj->enableCurrentCtrl = obj->flagEnableCurrentCtrl;
        obj->enableSpeedCtrl = obj->flagEnableSpeedCtrl;
    }
    else
    {
        obj->enableSpeedCtrl = FALSE;
        obj->enableCurrentCtrl = FALSE;
    }

    obj->speed_int_Hz = TRAJ_getIntValue(obj->trajHandle_spd);
    obj->oneOverDcBus_invV = 1.0f / obj->adcData.VdcBus_V;


    HALL_setTimeStamp(obj->hallHandle, HAL_calcCAPCount(obj->halMtrHandle));
    HALL_run(obj->hallHandle, obj->speed_int_Hz);
    obj->angleHall_rad = HALL_getAngle_rad(obj->hallHandle);
    obj->speedHall_Hz = HALL_getSpeed_Hz(obj->hallHandle);

    obj->speed_Hz = obj->speedHall_Hz;


    obj->speedFilter_Hz = obj->speedFilter_Hz *0.875f + obj->speed_Hz * 0.125f;
    obj->speedAbs_Hz = fabs(obj->speedFilter_Hz);

    if(obj->motorState >= MOTOR_CL_RUNNING)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
    }
    else if(obj->motorState == MOTOR_OL_START)
    {
        obj->angleFOC_rad = obj->angleHall_rad;
        obj->enableSpeedCtrl = FALSE;

        obj->Idq_out_A.value[0] = 0.0f;

        if(obj->speed_int_Hz > 0.0f)
        {
            obj->IsRef_A = obj->startCurrent_A;
            obj->Idq_out_A.value[1] = obj->startCurrent_A;
        }
        else
        {
            obj->IsRef_A = -obj->startCurrent_A;
            obj->Idq_out_A.value[1] = -obj->startCurrent_A;
        }

        obj->motorState = MOTOR_CL_RUNNING;
        PI_setUi(obj->piHandle_spd, (obj->frswPos_sf * obj->Idq_out_A.value[1]));
    }
    else if(obj->motorState == MOTOR_ALIGNMENT)
    {
        obj->angleFOC_rad = 0.0f;
        obj->enableSpeedCtrl = FALSE;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = obj->alignCurrent_A;
        obj->Idq_out_A.value[1] = 0.0f;

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
        HALL_setForceAngleAndIndex(obj->hallHandle, obj->speedRef_Hz);

        if((obj->stateRunTimeCnt > obj->alignTimeDelay) ||
                 (obj->flagEnableAlignment == FALSE))
        {
            obj->stateRunTimeCnt = 0;
            obj->motorState = MOTOR_OL_START;

            obj->Idq_out_A.value[0] = obj->fluxCurrent_A;

            PI_setUi(obj->piHandle_spd, 0.0);
        }
    }
    else if(obj->motorState == MOTOR_SEEK_POS)
    {
        obj->enableSpeedCtrl = FALSE;

        obj->stateRunTimeCnt++;

        obj->IsRef_A = 0.0f;
        obj->Idq_out_A.value[0] = 0.0f;
        obj->Idq_out_A.value[1] = 0.0f;

        if(obj->stateRunTimeCnt > obj->flyingStartTimeDelay)
        {
            obj->stateRunTimeCnt = 0;

            if(obj->speedAbs_Hz > obj->speedFlyingStart_Hz)
            {
                obj->speed_int_Hz = obj->speedFilter_Hz;
                TRAJ_setIntValue(obj->trajHandle_spd, obj->speedFilter_Hz);
                PI_setUi(obj->piHandle_spd, 0.0f);

                obj->motorState = MOTOR_CL_RUNNING;

                obj->IsRef_A = 0.0f;
                obj->Idq_out_A.value[0] = 0.0f;
            }
            else
            {
                obj->angleFOC_rad = 0.0f;
                obj->motorState = MOTOR_ALIGNMENT;
            }
        }
    }

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    obj->angleFOC_rad = obj->angleGen_rad;
#endif

    // compute the sin/cos phasor
    phasor.value[0] = cosf(obj->angleFOC_rad);
    phasor.value[1] = sinf(obj->angleFOC_rad);

    // run the Park transform
    PARK_run(phasor.value[1], phasor.value[0], obj->Iab_A.value[0], obj->Iab_A.value[1], &obj->Idq_in_A.value[0], &obj->Idq_in_A.value[1]);

// End of MOTOR1_HALL

//------------------------------------------------------------------------------
#else   // No Any Estimator
#error Not select a right estimator for this project
#endif  // (ESTIMATOR)

#if defined(MOTOR1_RPM_CMD)
    // convert the feedback speed to rpm
    obj->speed_rpm = obj->speed_Hz * obj->hz2Rpm_sf;

    if(obj->flagCmdRpmOrHz == FALSE)
    {
        obj->speedRef_rpm = obj->speedRef_Hz * obj->hz2Rpm_sf;
    }
    else
    {
        obj->speedRef_Hz = obj->speedRef_rpm * obj->rpm2Hz_sf;
    }
#endif  // MOTOR1_RPM_CMD

//---------- Common Speed and Current Loop for all observers -------------------
#if(DMC_BUILDLEVEL >= DMC_LEVEL_4)

#if defined(SFRA_ENABLE)

    if(sfraCollectStart == TRUE)
    {
        collectSFRA(motorHandle_M1);    // Collect noise feedback from loop
    }

    //  SFRA injection
    injectSFRA();                   // create SFRA Noise per 'sfraTestLoop'

    sfraCollectStart = TRUE;       // enable SFRA data collection
#endif  // SFRA_ENABLE

    // run the speed controller
    obj->counterSpeed++;

    if(obj->counterSpeed >= objUser->numCtrlTicksPerSpeedTick)
    {
        obj->counterSpeed = 0;

        obj->speed_reg_Hz = obj->speed_Hz;

        if(obj->enableSpeedCtrl == TRUE)
        {
            obj->Is_ffwd_A = 0.0f;


#if defined(SFRA_ENABLE)
            PI_run_series(obj->piHandle_spd,
                   (obj->speed_int_Hz + sfraNoiseSpd), obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#else     // !SFRA_ENABLE
            PI_run_series(obj->piHandle_spd,
                   obj->speed_int_Hz, obj->speed_reg_Hz,
                   obj->Is_ffwd_A, (float32_t *)&obj->IsRef_A);
#endif  // !SFRA_ENABLE
        }
        else if((obj->motorState >= MOTOR_CL_RUNNING) &&
                (obj->flagMotorIdentified == TRUE))
        {
            if(obj->speed_int_Hz > 0.0f)
            {
                obj->IsRef_A = obj->IsSet_A;
            }
            else
            {
                obj->IsRef_A = -obj->IsSet_A;
            }

            // for switching back speed closed-loop control
            PI_setUi(obj->piHandle_spd, obj->IsRef_A);
        }
    }
#if defined(MOTOR1_FWC) && defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad =
                (obj->angleFWC_rad > obj->angleMTPA_rad) ?
                        obj->angleFWC_rad : obj->angleMTPA_rad;

        fwcPhasor.value[0] = cosf(obj->angleCurrent_rad);
        fwcPhasor.value[1] = sinf(obj->angleCurrent_rad);

        if((obj->flagEnableFWC == TRUE) || (obj->flagEnableMTPA == TRUE))
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                sqrtf((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == TRUE)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == TRUE)
        {
            obj->angleMTPA_rad =
                    MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_FWC)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleFWC_rad;

        fwcPhasor.value[0] = cosf(obj->angleCurrent_rad);
        fwcPhasor.value[1] = sinf(obj->angleCurrent_rad);

        if(obj->flagEnableFWC == TRUE)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 2)
    {
        //
        // Compute the output and reference vector voltage
        obj->Vs_V =
                sqrtf((obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]) +
                       (obj->Vdq_out_V.value[1] * obj->Vdq_out_V.value[1]));

        obj->VsRef_V = obj->VsRef_pu * obj->adcData.VdcBus_V;

    }
    else if(obj->counterSpeed == 3)   // FWC
    {
        if(obj->flagEnableFWC == TRUE)
        {
            float32_t angleFWC;

            PI_run(obj->piHandle_fwc,
                   obj->VsRef_V, obj->Vs_V, (float32_t*)&angleFWC);
            obj->angleFWC_rad = MATH_PI_OVER_TWO - angleFWC;
        }
        else
        {
            PI_setUi(obj->piHandle_fwc, 0.0f);
            obj->angleFWC_rad = MATH_PI_OVER_TWO;
        }
    }
#elif defined(MOTOR1_MTPA)
    else if(obj->counterSpeed == 1)
    {
        MATH_Vec2 fwcPhasor;

        // get the current angle
        obj->angleCurrent_rad = obj->angleMTPA_rad;

        fwcPhasor.value[0] = cosf(obj->angleCurrent_rad);
        fwcPhasor.value[1] = sinf(obj->angleCurrent_rad);

        if(obj->flagEnableMTPA == TRUE)
        {
            obj->Idq_out_A.value[0] = obj->IsRef_A * fwcPhasor.value[0];
        }

        obj->Idq_out_A.value[1] = obj->IsRef_A * fwcPhasor.value[1];
    }
    else if(obj->counterSpeed == 4)   // MTPA
    {
        if(obj->flagEnableMTPA == TRUE)
        {
            obj->angleMTPA_rad = MTPA_computeCurrentAngle(obj->mtpaHandle, obj->IsRef_A);
        }
        else
        {
            obj->angleMTPA_rad = MATH_PI_OVER_TWO;
        }
    }
#else   // !MOTOR1_MTPA && !MOTOR1_FWC
    obj->Idq_out_A.value[1] = obj->IsRef_A;
#endif  // !MOTOR1_MTPA && !MOTOR1_FWC/

#if !defined(STEP_RP_EN)
    obj->IdqRef_A.value[0] = obj->Idq_out_A.value[0] + obj->IdRated_A;
#endif  // STEP_RP_EN

#if !defined(STEP_RP_EN)
#if defined(MOTOR1_VIBCOMP)
    // get the Iq reference value plus vibration compensation
    obj->IdqRef_A.value[1] = Idq_out_A.value[1] +
            VIB_COMP_inline_run(vibCompHandle, angleFOCM1_rad, Idq_in_A.value[1]);
#else
    obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
#endif  // MOTOR1_VIBCOMP
#else   // STEP_RP_EN
    if(GRAPH_getBufferMode(&stepRPVars) != GRAPH_STEP_RP_TORQUE)
    {
        obj->IdqRef_A.value[1] = obj->Idq_out_A.value[1];
    }
    else
    {
        PI_setUi(obj->piHandle_spd, obj->IdqRef_A.value[1]);
    }
#endif  // STEP_RP_EN


#elif(DMC_BUILDLEVEL == DMC_LEVEL_3)
    obj->IdqRef_A.value[0] = obj->Idq_set_A.value[0];
    obj->IdqRef_A.value[1] = obj->Idq_set_A.value[1];
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    if(obj->enableCurrentCtrl == TRUE)
    {
        obj->Vdq_ffwd_V.value[0] = 0.0f;
        obj->Vdq_ffwd_V.value[1] = 0.0f;


        // Maximum voltage output
        obj->VsMax_V = objUser->maxVsMag_pu * obj->adcData.VdcBus_V;
        PI_setMinMax(obj->piHandle_Id, -obj->VsMax_V, obj->VsMax_V);

#if defined(SFRA_ENABLE)
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      (obj->IdqRef_A.value[0] + sfraNoiseId), obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = sqrtf((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, (obj->IdqRef_A.value[1] + sfraNoiseIq),
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);

#else     // !SFRA_ENABLE
        // run the Id controller
        PI_run_series(obj->piHandle_Id,
                      obj->IdqRef_A.value[0], obj->Idq_in_A.value[0],
                      obj->Vdq_ffwd_V.value[0], (float32_t*)&obj->Vdq_out_V.value[0]);

        // calculate Iq controller limits
        float32_t outMax_V = sqrtf((obj->VsMax_V * obj->VsMax_V) -
                          (obj->Vdq_out_V.value[0] * obj->Vdq_out_V.value[0]));

        PI_setMinMax(obj->piHandle_Iq, -outMax_V, outMax_V);

        // run the Iq controller
        PI_run(obj->piHandle_Iq, obj->IdqRef_A.value[1],
               obj->Idq_in_A.value[1], (float32_t*)&obj->Vdq_out_V.value[1]);
#endif  // !SFRA_ENABLE


    }

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)

#define Manual_Vdq            // Set Vd and Vq manually for level 2

#ifndef Manual_Vdq
    VS_FREQ_run(obj->VsFreqHandle, obj->speed_int_Hz);
    obj->Vdq_out_V.value[0] = VS_FREQ_getVd_out(obj->VsFreqHandle);
    obj->Vdq_out_V.value[1] = VS_FREQ_getVq_out(obj->VsFreqHandle);
#else
    obj->Vdq_out_V.value[0] = Vdref_level_2;
    obj->Vdq_out_V.value[1] = Vqref_level_2;
#endif

#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(PHASE_ADJ_EN)
    if(obj->flagPhaseAdjustEnable == TRUE)
    {
        obj->angleFOCAdj_rad =
                MATH_incrAngle(obj->angleFOC_rad, obj->anglePhaseAdj_rad);

        // compute the sin/cos phasor
        phasor.value[0] = cosf(obj->angleFOCAdj_rad);
        phasor.value[1] = sinf(obj->angleFOCAdj_rad);
    }
    else
    {
        obj->angleFOCAdj_rad = obj->angleFOC_rad;
    }

#endif  // PHASE_ADJ_EN

#ifdef MOTOR1_SSIPD
    if(SSIPD_getRunState(ssipdHandle) == TRUE)
    {
        SSIPD_inine_run(ssipdHandle, &obj->Iab_A);

        obj->Vdq_out_V.value[0] = SSIPD_getVolInject_V(ssipdHandle);
        obj->angleFOC_rad = SSIPD_getAngleCmd_rad(ssipdHandle);

        TRAJ_setIntValue(obj->trajHandle_spd, 0.0f);
    }
#endif  // MOTOR1_SSIPD

    // run the inverse Park module
    IPARK_run(phasor.value[1], phasor.value[0], obj->Vdq_out_V.value[0], obj->Vdq_out_V.value[1], &obj->Vab_out_V.value[0], &obj->Vab_out_V.value[1]);

    // run the space vector generator (SVGEN) module
#if defined(MOTOR1_DCLINKSS)
    SVGEN_runCom(obj->oneOverDcBus_invV, obj->Vab_out_V.value[0], obj->Vab_out_V.value[1], &obj->pwmData.Vabc_pu.value[0], &obj->pwmData.Vabc_pu.value[1], &obj->pwmData.Vabc_pu.value[2]);
#else  // !(MOTOR1_DCLINKSS)
    SVGEN_runMin(obj->oneOverDcBus_invV, obj->Vab_out_V.value[0], obj->Vab_out_V.value[1], &obj->pwmData.Vabc_pu.value[0], &obj->pwmData.Vabc_pu.value[1], &obj->pwmData.Vabc_pu.value[2]);
#endif  // !(MOTOR1_DCLINKSS)

#if(DMC_BUILDLEVEL == DMC_LEVEL_1)
    // output 50%
    obj->pwmData.Vabc_pu.value[0] = 0.0f;
    obj->pwmData.Vabc_pu.value[1] = 0.0f;
    obj->pwmData.Vabc_pu.value[2] = 0.0f;
#endif

    if(HAL_getPwmEnableStatus(obj->halMtrHandle) == FALSE)
    {
        // clear PWM data
        obj->pwmData.Vabc_pu.value[0] = 0.0f;
        obj->pwmData.Vabc_pu.value[1] = 0.0f;
        obj->pwmData.Vabc_pu.value[2] = 0.0f;
    }

#if defined(MOTOR1_DCLINKSS)
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    // revise PWM compare(CMPA/B) values for shifting switching pattern
    // and, update SOC trigger point
    HAL_runSingleShuntCompensation(obj->halMtrHandle, obj->dclinkHandle,
                         &obj->Vab_out_V, &obj->pwmData, obj->adcData.VdcBus_V);
#else   // !(MOTOR1_DCLINKSS)
#if defined(MOTOR1_OVM)
    else
    {
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPWMData(obj->svgencurrentHandle,
                                 &obj->pwmData.Vabc_pu, &obj->pwmDataPrev);
    }

    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);

    obj->ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(obj->svgencurrentHandle);
    obj->midVolShunt = SVGENCURRENT_getVmid(obj->svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(obj->halMtrHandle,
                   &obj->pwmData, obj->ignoreShuntNextCycle, obj->midVolShunt);
#else   // !MOTOR1_OVM
    // write the PWM compare values
    HAL_writePWMData(obj->halMtrHandle, &obj->pwmData);
#endif  // !MOTOR1_OVM
#endif // !(MOTOR1_DCLINKSS)

    // Collect current and voltage data to calculate the RMS value
    collectRMSData(motorHandle_M1);

//------------------------------------------------------------------------------
#if defined(CPUTIME_ENABLE)
    cycleCountAfter = CycleCounterP_getCount32();
    cycleCountAfter2 = CycleCounterP_getCount32();
    cpuCycles += 2 * cycleCountAfter - cycleCountAfter2 - cycleCountBefore;
    ISRcount++;
    cpuCyclesAv = cpuCycles/ISRcount;
#endif  // CPUTIME_ENABLE

#if defined(STEP_RP_EN)
    // Collect predefined data into arrays
    GRAPH_updateBuffer(&stepRPVars);
#endif  // STEP_RP_EN

#if defined(EPWMDAC_MODE)
    // connect inputs of the PWMDAC module.
    HAL_writePWMDACData(halHandle, &pwmDACData);
#endif  // EPWMDAC_MODE

#if defined(DATALOG_EN)
        DATALOG_update(datalogHandle);
#endif  // DATALOG_EN

#if defined(DAC128S_ENABLE)
    // Write the variables data value to DAC128S085
    DAC128S_writeData(dac128sHandle);
#endif  // DAC128S_ENABLE



    return;
} // end of motor1CtrlISR() function

//
//-- end of this file ----------------------------------------------------------
//
