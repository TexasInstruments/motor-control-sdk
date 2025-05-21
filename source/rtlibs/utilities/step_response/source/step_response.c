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


// the includes
#include "step_response.h"

#if defined(STEP_RP_EN)
// global variables
__attribute__ ((section(".datalog_data"))) GRAPH_StepVars_t stepRPVars;
#endif  // STEP_RP_EN

// **************************************************************************
// the function prototypes

//-----------------------------------------------------------------
void GRAPH_init(GRAPH_StepVars_t *pStepVars,
                volatile float32_t *pSpeed_ref, volatile float32_t *pSpeed_in,
                volatile float32_t *pId_ref, volatile float32_t *pId_in,
                volatile float32_t *pIq_ref, volatile float32_t *pIq_in)
{
    uint_least16_t buffer_array = 0;

    for(buffer_array = 0; buffer_array < GRAPH_BUFFER_NR; buffer_array++)
    {
        pStepVars->bufferData[buffer_array].write = 0;
    }

    pStepVars->bufferMode = GRAPH_STEP_RP_IDLE;

    pStepVars->bufferCounter = 0;
    pStepVars->bufferTickCounter =  0;
    pStepVars->bufferTick =  GRAPH_BUFFER_SPEED_TICK;

    pStepVars->spdRef_Default = GRAPH_SPEEDREF_DEFAULT;
    pStepVars->spdRef_StepSize = GRAPH_SPEEDREF_STEP;

    pStepVars->IdRef_Default = GRAPH_CURRENTREF_DEFAULT;
    pStepVars->IdRef_StepSize = GRAPH_CURRENTREF_STEP;

    pStepVars->IqRef_Default = GRAPH_TORQUE_DEFAULT;
    pStepVars->IqRef_StepSize = GRAPH_TORQUE_STEP;

    pStepVars->pSpeed_ref = pSpeed_ref;
    pStepVars->pSpeed_in = pSpeed_in;

    pStepVars->pId_ref = pId_ref;
    pStepVars->pId_in = pId_in;

    pStepVars->pIq_ref = pIq_ref;
    pStepVars->pIq_in = pIq_in;

    return;
}



//-----------------------------------------------------------------
void GRAPH_generateStepResponse(GRAPH_StepVars_t *pStepVars)
{
    if(pStepVars->stepResponse)
    {
        switch(pStepVars->bufferMode)
        {
            case GRAPH_STEP_RP_CURRENT:
                *pStepVars->pId_ref = pStepVars->IdRef_Default +
                                      pStepVars->IdRef_StepSize;
                pStepVars->bufferCounter = 0;
                pStepVars->bufferData[0].write = 0;
                pStepVars->bufferData[1].write = 0;
                break;

            case GRAPH_STEP_RP_SPEED:
                *pStepVars->pSpeed_ref = pStepVars->spdRef_Default +
                                         pStepVars->spdRef_StepSize;
                pStepVars->bufferCounter = 0;
                pStepVars->bufferData[0].write = 0;
                pStepVars->bufferData[1].write = 0;
                break;

            case GRAPH_STEP_RP_TORQUE:
                *pStepVars->pIq_ref = pStepVars->IqRef_Default +
                                      pStepVars->IqRef_StepSize;
                pStepVars->bufferCounter = 0;
                pStepVars->bufferData[0].write = 0;
                pStepVars->bufferData[1].write = 0;
                break;

            default:
                break;
        } // End switch
    } // End if

    if((!(pStepVars->stepResponse)) && (pStepVars->bufferCounter == GRAPH_BUFFER_SIZE))
    {
        switch(pStepVars->bufferMode)
        {
            case GRAPH_STEP_RP_CURRENT:
                *pStepVars->pId_ref = pStepVars->IdRef_Default;
                break;

            case GRAPH_STEP_RP_SPEED:
                *pStepVars->pSpeed_ref = pStepVars->spdRef_Default;
                break;

            case GRAPH_STEP_RP_TORQUE:
                *pStepVars->pIq_ref = pStepVars->IqRef_Default;
                break;

            default:
                break;
        } // End switch
    } // End if

    pStepVars->stepResponse = 0;

    return;
} // End of GRAPH_generateStepResponse()

// end of file

