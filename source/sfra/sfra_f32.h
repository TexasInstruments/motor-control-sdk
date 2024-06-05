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
//###########################################################################
//
// FILE:   sfra_f32.h
//
//#############################################################################


#ifndef SFRA_F32_H
#define SFRA_F32_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \defgroup SFRA_API_MODULE APIs for Software Frequency Response Analyzer Library
 *  \ingroup  RTLIBS_API
 *  
 *  Here is the list of APIs used for Digital Control Library
 *  @{
 */
//
// the includes & defines
//

#ifndef IEEE754_TYPES
#define IEEE754_TYPES

typedef float float32_t;
typedef double float64_t;

#endif //IEEE754_TYPES

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>



//! \brief          Defines the SFRA_F32 structure
//!
//! \details        The SFRA_F32 can be used to run a software based
//!                 frequency response analysis on power converters
//!
typedef struct{
    float32_t *h_magVect;     //!< Plant Mag SFRA Vector
    float32_t *h_phaseVect;   //!< Plant Phase SFRA Vector
    float32_t *gh_magVect;    //!< Open Loop Mag SFRA Vector
    float32_t *gh_phaseVect;  //!< Open Loop Phase SFRA Vector
    float32_t *cl_magVect;    //!< Closed Loop Mag SFRA Vector
    float32_t *cl_phaseVect;  //!< Closed Loop Phase SFRA Vector
    float32_t *freqVect;      //!< Frequency Vector
    float32_t amplitude;      //!< Injection Amplitude
    float32_t isrFreq;        //!< SFRA ISR frequency
    float32_t freqStart;      //!< Start frequency of SFRA sweep
    float32_t freqStep;       //!< Log space between frequency points (optional)
    int16_t start;            //!< Command to start SFRA
    int16_t state;            //!< State of SFRA
    int16_t status;           //!< Status of SFRA
    int16_t vecLength;        //!< No. of Points in the SFRA
    int16_t freqIndex;        //!< Index of the frequency vector
    int16_t storeH;           //!< Flag to indicate if H vector is stored
    int16_t storeGH;          //!< Flag to indicate if GH vector is stored
    int16_t storeCL;          //!< Flag to indicate if CL vector is stored
    int16_t speed;            //!< variable to change the speed of the sweep
}SFRA_F32;

//! \brief               Resets internal data of SFRA_F32 module
//! \param SFRA_F_obj    Pointer to the SFRA_F32 structure
//!
extern void SFRA_F32_reset(SFRA_F32 *SFRA_F_obj);

//! \brief Configures the SFRA_F32 module
//! \param *SFRA_F_obj  Pointer to the SFRA_F32 structure
//! \param isrFrequency  Frequency at which SFRA routine in called,
//!                      typically the control ISR rate
//! \param injectionAmplitude  Per Unit (PU) injection amplitude
//! \param noFreqPoints  Number of frequency points sweeped
//! \param fraSweepStartFreq  Start frequency of SFRA sweep
//! \param freqStep  Multiplier used to keep frequency points log step apart
//! \param *h_magVect  Pointer to array that stores plant FRA magnitude data
//! \param *h_phaseVect  Pointer to array that stores plant FRA phase data
//! \param *gh_magVect  Pointer to array that stores OL FRA magnitude data
//! \param *gh_phaseVect  Pointer to array that stores OL FRA phase data
//! \param *cl_magVect  Pointer to array that stores OL FRA magnitude data
//! \param *cl_phaseVect  Pointer to array that stores OL FRA phase data
//! \param *freqVect Pointer to array that stores the freq points for the sweep
//! \param speed indiactes the speed of the sweep
//!
extern void SFRA_F32_config(SFRA_F32 *SFRA_F_obj,
                     float32_t isrFrequency,
                     float32_t injectionAmplitude,
                     int16_t noFreqPoints,
                     float32_t fraSweepStartFreq,
                     float32_t freqStep,
                     float32_t *h_magVect,
                     float32_t *h_phaseVect,
                     float32_t *gh_magVect,
                     float32_t *gh_phaseVect,
                     float32_t *cl_magVect,
                     float32_t *cl_phaseVect,
                     float32_t *freqVect,
                     int16_t speed);

//! \brief Initailizes the freq vectors with points that are log step apart
//! \param *SFRA_F_obj  Pointer to the SFRA_F32 structure
//! \param fra_sweep_start_freq  Start frequency of SFRA sweep
//! \param freqStep  Multiplier used to keep frequency points log step apart
//!
extern void SFRA_F32_initFreqArrayWithLogSteps(SFRA_F32 *SFRA_F_obj,
                                        float32_t fra_sweep_start_freq,
                                        float32_t freqStep);

//! \brief Resets the response data stored in the ol and plant
//!        phase and mag vector
//! \param *SFRA_F_obj  Pointer to the SFRA_F32 structure pointer
//!
extern void SFRA_F32_resetFreqRespArray(SFRA_F32 *SFRA_F_obj);

//! \brief Updates injection amplitude
//! \param *SFRA_F_obj  Pointer to the SFRA_F32 structure
//! \param new_injection_amplitude Injection amplitude
//!
extern void SFRA_F32_updateInjectionAmplitude(SFRA_F32 *SFRA_F_obj,
                                       float32_t new_injection_amplitude);

//! \brief Injects small signal disturbance into the control loop
//! \param ref refernce value on which the injection is added
//! \return Routine returns the reference plus the injection when SFRA sweep
//!         is active, when SFRA sweep is not active that is if SFRA state is 0
//!         it returns the the refernce without any change
//!
extern float SFRA_F32_inject(float ref);

//! \brief Collects the response of the loop because of small signal disturbance
//!        injected
//! \param *control_output pointer to the variable where control output is saved
//!                        note though the parameter is passed by reference
//!                        it is unchanged by the module
//! \param *feedback pointer to the variable where control output is saved
//!                        note though the parameter is passed by reference
//!                        it is unchanged by the module
//!
extern void SFRA_F32_collect(float *control_output, float *feedback);

//! \brief Runs the background task, this routine executes the state machine
//!        when a frequency sweep is started and is responsible for changing
//!        the frequency points and saving the measured results in an array
//! \param *SFRA_F_obj  Pointer to the SFRA_F32 structure pointer
//!
extern void SFRA_F32_runBackgroundTask(SFRA_F32 *SFRA_F_obj);

// Close the Doxygen group.
/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // end of SFRA_F32_H definition
