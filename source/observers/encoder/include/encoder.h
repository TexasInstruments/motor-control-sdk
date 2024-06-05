/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup OBSERVERS_MODULE APIs for Observers and Encoders
 *  \ingroup  RTLIBS_API
 *  \defgroup ENCODER_API_MODULE APIs for encoder object
 *  \ingroup  OBSERVERS_MODULE
 *
 *  Here is the list of encoder function APIs
 *  @{
 *
 *  \file           encoder.h
 *  \brief          Contains interface to encoder object
 */

// the includes
#include <stdint.h>
#include <stdlib.h>

#include <drivers/eqep.h>
#include <drivers/gpio.h>

#include "math_types.h"
#include "userParams.h"

// **************************************************************************
// the defines
//#define ENC_CALIB       1

// State machine typedef for ENC status
typedef enum
{
    ENC_IDLE             = 0,
    ENC_ALIGNMENT        = 1,
    ENC_WAIT_FOR_INDEX   = 2,
    ENC_CALIBRATION_DONE = 3
} ENC_Status_e;


//! \brief Defines the ENC controller object
//!
typedef struct _ENC_Obj_
{
    float32_t Ts_sec;               // sampling period (sec)
	float32_t polePairs;            // pole pairs of the motor
	float32_t encLines;             // lines of the encoder

    float32_t mechanicalScaler;

    float32_t thetaMech_pu;         // Mechanical angle
    float32_t thetaMech_rad;        // Mechanical angle
    float32_t thetaElec_rad;        // Electrical angle
    float32_t thetaHall_rad;        // Initial angle of the hall sensor
    float32_t *ptrHalltheta;
    uint32_t  *ptrHallPos;

    float32_t speedElec_Hz;         // target speed
    float32_t speedMech_Hz;         // estimated rotor speed

    uint32_t  qepHandle;            // the QEP handle
    uint32_t  indexOffset;          // the offset of index

#if defined(ENC_CALIB)
    float32_t hallThetaData[12];
    uint32_t  hallPosData[12];
    uint16_t  hallStateIndex[12];
    uint16_t  hallIndex;
    uint16_t  hallStatePrev;
#endif  // ENC_CALIB

    uint32_t  gpioHallU;            // the GPIO for Hall U
    uint32_t  gpioHallV;            // the GPIO for Hall V
    uint32_t  gpioHallW;            // the GPIO for Hall W
    uint32_t  gpioHallUBase;        // the GPIO base address for Hall U
    uint32_t  gpioHallVBase;        // the GPIO base address for Hall V
    uint32_t  gpioHallWBase;        // the GPIO base address for Hall W
    uint16_t  hallState;
    uint16_t  hallStateZero;

    ENC_Status_e encState;
} ENC_Obj, *ENC_Handle;

// **************************************************************************
// the function prototypes
//! \brief     Initializes the ENC controller
//! \param[in] pMemory   A pointer to the memory for the ENC controller object
//!
//! \param[in] numBytes  The number of bytes allocated for the ENC controller object, bytes
//!
//! \return The ENC controller (ENC) object handle
//
//*****************************************************************************
extern ENC_Handle ENC_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Set the controller parameters
//!
//! \param[in] handle      The ENC controller handle
//! \param[in] pUserParams The pointer to User_Params object
//
//*****************************************************************************
extern void ENC_setParams(ENC_Handle handle, const USER_Params *pUserParams);

//*****************************************************************************
//
//! \brief    Set the hall pins and address (only needed for calibration mode)
//!
//! \param[in] handle  the ENC Handle
//! \param[in] gpioHallU        GPIO pin for Hall U
//! \param[in] gpioHallV        GPIO pin for Hall V
//! \param[in] gpioHallW        GPIO pin for Hall W
//! \param[in] gpioHallUBase    the GPIO base address for Hall U
//! \param[in] gpioHallVBase    the GPIO base address for Hall V
//! \param[in] gpioHallWBase    the GPIO base address for Hall W
//
//*****************************************************************************
static inline void ENC_setHallGPIO(ENC_Handle handle, const uint32_t gpioHallU,
                     const uint32_t gpioHallV, const uint32_t gpioHallW,
                     const uint32_t gpioHallUBase, const uint32_t gpioHallVBase,
                     const uint32_t gpioHallWBase)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->gpioHallU = gpioHallU;
    obj->gpioHallV = gpioHallV;
    obj->gpioHallW = gpioHallW;
    obj->gpioHallUBase = gpioHallUBase;
    obj->gpioHallVBase = gpioHallVBase;
    obj->gpioHallWBase = gpioHallWBase;

    return;
}

//*****************************************************************************
//
//! \brief     Runs the ENC controller
//!
//! \param[in] handle      The ENC controller handle
//!
//! \return    speed from encoder
//
//*****************************************************************************
static inline float32_t ENC_getSpeedElec_Hz(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    return(obj->speedElec_Hz);
}

//*****************************************************************************
//
//! \brief     Gets the angle from encoder
//!
//! \param[in] handle      The ENC controller handle
//!
//! \return    angle from encoder
//
//*****************************************************************************
static inline float32_t ENC_getElecAngle(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    return(obj->thetaElec_rad);
}

//*****************************************************************************
//
//! \brief     Gets the state of the ENC controller
//!
//! \param[in] handle   the ENC Handle
//!
//! \return    state from encode
//
//*****************************************************************************
static inline float32_t ENC_getState(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    return(obj->encState);
}

//*****************************************************************************
//
//! \brief     Reset the ENC controller state
//!
//! \param[in] handle   the ENC Handle
//
//*****************************************************************************
static inline void ENC_resetState(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->encState = ENC_IDLE;

    return;
}

//*****************************************************************************
//
//! \brief     Sets up the ENC controller state
//!
//! \param[in] handle   the ENC Handle
//! \param[in] encState the enum state of ENC controller
//!
//
//*****************************************************************************
static inline void ENC_setState(ENC_Handle handle, const ENC_Status_e encState)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->encState = encState;

    return;
}

//*****************************************************************************
//
//! \brief     Set the ENC controller parameter Hall U (only needed for calibration mode)
//!
//! \param[in] handle           the ENC Handle
//! \param[in] gpioHallU        GPIO pin for Hall U
//! \param[in] gpioHallUBase    the GPIO base address for Hall U
//
//*****************************************************************************
static inline void ENC_setGPIOHallU(ENC_Handle handle, const uint32_t gpioHallU, const uint32_t gpioHallUBase)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->gpioHallU = gpioHallU;
    obj->gpioHallUBase = gpioHallUBase;

    return;
}

//*****************************************************************************
//
//! \brief     Set the ENC controller parameter Hall V (only needed for calibration mode)
//!
//! \param[in] handle           the ENC Handle
//! \param[in] gpioHallV        GPIO pin for Hall V
//! \param[in] gpioHallVBase    the GPIO base address for Hall V
//
//*****************************************************************************
static inline void ENC_setGPIOHallV(ENC_Handle handle, const uint32_t gpioHallV, const uint32_t gpioHallVBase)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->gpioHallV = gpioHallV;
    obj->gpioHallVBase = gpioHallVBase;

    return;
}

//*****************************************************************************
//
//! \brief     Set the ENC controller parameter Hall W (only needed for calibration mode)
//!
//! \param[in] handle   the ENC Handle
//! \param[in] gpioHallW        GPIO pin for Hall W
//! \param[in] gpioHallWBase    the GPIO base address for Hall W
//
//*****************************************************************************
static inline void ENC_setGPIOHallW(ENC_Handle handle, const uint32_t gpioHallW, const uint32_t gpioHallWBase)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->gpioHallW = gpioHallW;
    obj->gpioHallWBase = gpioHallWBase;

    return;
}

//*****************************************************************************
//
//! \brief     Set the ENC controller eqep handle
//!
//! \param[in] handle   the ENC Handle
//! \param[in] qepBase  the QEP handle
//
//*****************************************************************************
static inline void ENC_setQEPHandle(ENC_Handle handle, const uint32_t qepBase)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    obj->qepHandle = qepBase;

    return;
}

//*****************************************************************************
//
//! \brief     Get the hall state
//!
//! \param[in] handle  the ENC Handle
//
//*****************************************************************************
static inline uint16_t ENC_getHallState(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    uint32_t hallState = 0;

    hallState = GPIO_pinRead(obj->gpioHallUBase, obj->gpioHallU);
    hallState += GPIO_pinRead(obj->gpioHallVBase, obj->gpioHallV)<<1;
    hallState += GPIO_pinRead(obj->gpioHallWBase, obj->gpioHallW)<<2;

    return(hallState & 0x00000007);
}

#define ENC_inline_run ENC_run // Backwards compatible

//*****************************************************************************
//
//! \brief     Runs the ENC controller
//!
//! \param[in] handle  the ENC Handle
//
//*****************************************************************************
static __attribute__((always_inline)) 
void ENC_run(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    if(obj->encState == ENC_CALIBRATION_DONE)
    {
        obj->thetaMech_pu = obj->mechanicalScaler *
                (float32_t)EQEP_getPositionLatch(obj->qepHandle);

        obj->thetaMech_rad = obj->thetaMech_pu * MATH_TWO_PI;

        float32_t thetaElec_pu = obj->thetaMech_pu * obj->polePairs;

        obj->thetaElec_rad = (thetaElec_pu - ((int32_t)thetaElec_pu)) * MATH_TWO_PI;

        if(obj->thetaElec_rad >= MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad - MATH_TWO_PI;
        }
        else if(obj->thetaElec_rad <= -MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad + MATH_TWO_PI;
        }
    }
    else if(obj->encState == ENC_WAIT_FOR_INDEX)
    {
        if(EQEP_getInterruptStatus(obj->qepHandle) & EQEP_INT_INDEX_EVNT_LATCH)
        {
            EQEP_setInitialPosition(obj->qepHandle, EQEP_getPositionLatch(obj->qepHandle));

            EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);

            obj->indexOffset = EQEP_getPositionLatch(obj->qepHandle);
            obj->encState = ENC_CALIBRATION_DONE;
        }
    }
    else    // obj->encState == ENC_ALIGNMENT
    {

#if defined(ENC_CALIB)
        obj->hallStateZero = ENC_getHallState(handle);
        obj->hallIndex = 0;
#endif  // ENC_CALIB

        // during alignment, reset the current shaft position to zero
        EQEP_setPosition(obj->qepHandle, 0);

        // Reset pos cnt for QEP
        EQEP_clearInterruptStatus(obj->qepHandle, EQEP_INT_INDEX_EVNT_LATCH);

        // reset poscnt init on index
        EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_DO_NOTHING);
    }

#if defined(ENC_CALIB)
    obj->hallState = ENC_getHallState(handle);

    if(obj->hallState != obj->hallStatePrev)
    {
        obj->hallThetaData[obj->hallIndex] = obj->thetaElec_rad;
        obj->hallPosData[obj->hallIndex] = EQEP_getPositionLatch(obj->qepHandle);
        obj->hallStateIndex[obj->hallIndex] = obj->hallState;

        obj->hallIndex++;

        if(obj->hallIndex >= 12)
        {
            obj->hallIndex = 0;
        }
    }

    obj->hallStatePrev = obj->hallState;
#endif  // ENC_CALIB

    return;
}

#ifndef ENC_full_run
#define ENC_full_run ENC_runHall
#endif

//*****************************************************************************
//
//! \brief     Runs the hall of ENC controller
//!
//! \param[in] handle  The ENC controller handle
//
//*****************************************************************************
static __attribute__((always_inline)) 
void ENC_runHall(ENC_Handle handle)
{
    ENC_Obj *obj = (ENC_Obj *)handle;

    if(obj->encState == ENC_CALIBRATION_DONE)
    {
        obj->thetaMech_pu = obj->mechanicalScaler *
                (float32_t)EQEP_getPositionLatch(obj->qepHandle);

        obj->thetaMech_rad = obj->thetaMech_pu * MATH_TWO_PI;

        float32_t thetaElec_pu = obj->thetaMech_pu * obj->polePairs;

        obj->thetaElec_rad = (thetaElec_pu - ((int32_t)thetaElec_pu)) * MATH_TWO_PI;

        if(obj->thetaElec_rad >= MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad - MATH_TWO_PI;
        }
        else if(obj->thetaElec_rad <= -MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad + MATH_TWO_PI;
        }
    }
    else if(obj->encState == ENC_WAIT_FOR_INDEX)
    {
        if(EQEP_getInterruptStatus(obj->qepHandle) & EQEP_INT_INDEX_EVNT_LATCH)
        {
            EQEP_setInitialPosition(obj->qepHandle, EQEP_getPositionLatch(obj->qepHandle));

            EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);

            obj->encState = ENC_CALIBRATION_DONE;
        }
    }
    else    // obj->encState == ENC_ALIGNMENT
    {
        // during alignment, reset the current shaft position to zero
        EQEP_setPosition(obj->qepHandle, 0);

        // Reset pos cnt for QEP
        EQEP_clearInterruptStatus(obj->qepHandle, EQEP_INT_INDEX_EVNT_LATCH);

        // reset poscnt init on index
        EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_DO_NOTHING);
    }

    return;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H 

