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

#ifndef STEP_RP_H
#define STEP_RP_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup STEPRESPONSE_API_MODULE APIs
 *  \ingroup  MOTOR_CONTROL_API
 *
 *  Here is the list of GRAPHG function APIs
 *  @{
 *
 *  \file           step_response.h
 *  \brief          Contains graphing (GRAPH) functions implementation
 */

// the includes
#include <stdint.h>
#include <stdbool.h>

#include "math_types.h"

// modules

// drivers

// platforms

// **************************************************************************
// the defines
// MST: Changed to 4 buffer arrays
#define GRAPH_BUFFER_NR             2           // Number of data arrays

#define GRAPH_BUFFER_SIZE           256         // Size of data arrays
#define GRAPH_BUFFER_MASK           (256-1)

#define GRAPH_BUFFER_SPEED_TICK     100U
#define GRAPH_BUFFER_CURRENT_TICK   10U

#define GRAPH_SPEEDREF_DEFAULT      40.0f
#define GRAPH_SPEEDREF_STEP         60.0f

#define GRAPH_CURRENTREF_DEFAULT    0.0f
#define GRAPH_CURRENTREF_STEP       -0.5f

#define GRAPH_TORQUE_DEFAULT        1.0f
#define GRAPH_TORQUE_STEP           4.0f

//! \brief Initialization values of global variables
//!


// **************************************************************************
// the typedefs

//! \brief Enumeration for the number of buffers
//!
typedef enum
{
    GRAPH_BUFFER_NR0 = 0,               //!< Buffer define 0
    GRAPH_BUFFER_NR1 = 1                //!< Buffer define 1
} GRAPH_BufferNR_e;

typedef enum
{
    GRAPH_STEP_RP_IDLE       = 0,      //!< current step response
    GRAPH_STEP_RP_CURRENT    = 1,      //!< d-axis current step response
    GRAPH_STEP_RP_SPEED      = 2,      //!< speed step response
    GRAPH_STEP_RP_TORQUE     = 3       //!< q-axis current step response
} GRAPH_STEP_RPM_Mode_e;

typedef struct _GRAPH_Buffer_t_
{
    float32_t   data[GRAPH_BUFFER_SIZE];
    uint16_t    write;          // points to empty field
}GRAPH_Buffer_t;

typedef struct _GRAPH_StepVars_t_
{
    GRAPH_STEP_RPM_Mode_e bufferMode;   // used to define different values to
                                        // record for each mode during run time

    uint16_t stepResponse;              // value to start the step response generation
    uint16_t bufferCounter;             // used as an index into the buffer
    uint16_t bufferTickCounter;         // used to count the interrupts
    uint16_t bufferTick;                // defines how many interrupts per graph write

    volatile float32_t *pSpeed_ref;     // Pointer to Setting value of spdRef
    volatile float32_t *pSpeed_in;      // Pointer to Real value of Speed

    volatile float32_t *pId_ref;        // Pointer to Setting value of IdRef
    volatile float32_t *pId_in;         // Pointer to Real value of Id

    volatile float32_t *pIq_ref;        // Pointer to Setting value of IqRef
    volatile float32_t *pIq_in;         // Pointer to Real value of Iq

    float32_t spdRef_Default;           // Default starting value of spdRef
    float32_t spdRef_StepSize;          // Step size of spdRef

    float32_t IdRef_Default;            // Default starting value of IdRef
    float32_t IdRef_StepSize;           // Step size of IdRef

    float32_t IqRef_Default;            // Default starting value of IqRef
    float32_t IqRef_StepSize;           // Step size of IqRef

    GRAPH_Buffer_t bufferData[GRAPH_BUFFER_NR];
}GRAPH_StepVars_t;


#if defined(STEP_RP_EN)
// global variables
extern GRAPH_StepVars_t stepRPVars;
#endif  // STEP_RP_EN

// **************************************************************************
// the function prototypes

//*****************************************************************************
//
//! \brief      Data gathering function for any iq value
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pSpeed_in  The pointer to the speed feedback data
//! \param[in]  pId_in     The pointer to the Id feedback data
//! \param[in]  pIq_in     The pointer to the Iq feedback data
//! \param[in]  pSpeed_ref The pointer to the speed reference data
//! \param[in]  pId_ref    The pointer to the Id reference data
//
//*****************************************************************************
extern void GRAPH_init(GRAPH_StepVars_t *pStepVars,
                volatile float32_t *pSpeed_ref, volatile float32_t *pSpeed_in,
                volatile float32_t *pId_ref, volatile float32_t *pId_in,
                volatile float32_t *pIq_ref, volatile float32_t *pIq_in);

//*****************************************************************************
//
//! \brief      Sets the values to collect in a data array
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pStepVars   Pointer to the step variables
//
//*****************************************************************************
extern void GRAPH_generateStepResponse(GRAPH_StepVars_t *pStepVars);

//*****************************************************************************
//
//! \brief      Write into the buffer
//! \param[in]  GRAPH_Buffer_t          The pointer to the buffer data
//! \param[in]    word            Write data into the buffer
//
//*****************************************************************************
static inline 
bool GRAPH_writeDataBuffer(GRAPH_Buffer_t *pBuffer, float32_t data)
{
    uint16_t next = ((pBuffer->write + 1) & GRAPH_BUFFER_MASK);

    pBuffer->data[pBuffer->write & GRAPH_BUFFER_MASK] = data;
    pBuffer->write = next;

    return(true);
}

//*****************************************************************************
//
//! \brief      Data gathering function for any iq value
//! \param[in]  GRAPH_Vars_t          The pointer to the gGraphVars data
//! \param[in]  GRAPH_BufferNR_e      Definition of buffer number
//! \param[in]  gData                 Recorded iq data
//! \param[in]  TriggerValue          Trigger value to start the data recording
//
//*****************************************************************************
static inline void GRAPH_collectData (GRAPH_StepVars_t *pStepVars,
                             GRAPH_BufferNR_e bufferNum, float32_t gData)
{
    if( pStepVars->bufferCounter < GRAPH_BUFFER_SIZE)
    {
       if(pStepVars->bufferTickCounter >= pStepVars->bufferTick)
       {
           GRAPH_writeDataBuffer(&pStepVars->bufferData[bufferNum], gData);

            if (bufferNum <=  GRAPH_BUFFER_NR0)
            {
                pStepVars->bufferCounter++;
                pStepVars->bufferTickCounter = 0;
            }

       } // End (gGraphVars->Buffer_delay_counter <= gGraphVars->Buffer_delay)

       if (bufferNum <=  GRAPH_BUFFER_NR0)
       {
           pStepVars->bufferTickCounter++;
       }
    } // End if(gGraphVars->bufferCounter < GRAPH_BUFFER_SIZE)

    return;
} // End of Graph_Current_step()

static inline 
GRAPH_STEP_RPM_Mode_e GRAPH_getBufferMode(GRAPH_StepVars_t *pStepVars)
{
    return(pStepVars->bufferMode);
}

//*****************************************************************************
//
//! \brief      Sets the values to collect in a data array
//! \param[in]  handle      The controller (CTRL) handle
//! \param[in]  pGraphVars  Pointer to the graph variables
//! \param[in]  pStepVars   Pointer to the step variables
//
//*****************************************************************************
static inline void GRAPH_updateBuffer(GRAPH_StepVars_t *pStepVars)
{
    switch(pStepVars->bufferMode)
    {
        case GRAPH_STEP_RP_CURRENT:
            // Id current
            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR0,
                              (*pStepVars->pId_in));

            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR1,
                              (*pStepVars->pSpeed_in));
        break;

        case GRAPH_STEP_RP_SPEED:
            // Iq current and speed
            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR1,
                              (*pStepVars->pIq_in));

            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR0,
                              (*pStepVars->pSpeed_in));
        break;

        case GRAPH_STEP_RP_TORQUE:
            // Iq current & speed
            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR0,
                              (*pStepVars->pIq_in));

            GRAPH_collectData(pStepVars, GRAPH_BUFFER_NR1,
                              (*pStepVars->pSpeed_in));
        break;

        default:
            break;
    }

    return;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // end of STEP_RP_H definition

