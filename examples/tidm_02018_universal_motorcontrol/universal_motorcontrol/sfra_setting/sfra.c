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


#include "sfra_settings.h"

//
// SFRA Related Variables
//
__attribute__ ((section(".sfradata"))) SFRA_F32 sfra1;

__attribute__ ((section(".sfradata"))) float32_t plantMagVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t plantPhaseVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t olMagVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t olPhaseVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t clMagVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t clPhaseVect[SFRA_FREQ_LENGTH];
__attribute__ ((section(".sfradata"))) float32_t freqVect[SFRA_FREQ_LENGTH];


//
// configureSFRA
//
void configureSFRA(uint16_t plotOption, float32_t sfraISRFreq)
{
    //
    //Resets the internal data of sfra module to zero
    //
    SFRA_F32_reset(&sfra1);

    //
    //Configures the SFRA module
    //
    SFRA_F32_config(&sfra1,
                    sfraISRFreq,
                    SFRA_AMPLITUDE,
                    SFRA_FREQ_LENGTH,
                    SFRA_FREQ_START,
                    SFRA_FREQ_STEP_MULTIPLY,
                    plantMagVect,
                    plantPhaseVect,
                    olMagVect,
                    olPhaseVect,
                    clMagVect,
                    clPhaseVect,
                    freqVect,
                    SFRA_SWEEP_SPEED);

    //
    //Resets the response arrays to all zeroes
    //
    SFRA_F32_resetFreqRespArray(&sfra1);

    //
    //Initializes the frequency response array ,
    //The first element is SFRA_FREQ_START
    //The subsequent elements are freqVect[n-1]*SFRA_FREQ_STEP_MULTIPLY
    //This enables placing a fixed number of frequency points
    //between a decade of frequency.
    // The below routine can be substituted by a routine that sets
    // the frequency points arbitrarily as needed.
    //
    SFRA_F32_initFreqArrayWithLogSteps(&sfra1,
                                       SFRA_FREQ_START,
                                       SFRA_FREQ_STEP_MULTIPLY);

}
