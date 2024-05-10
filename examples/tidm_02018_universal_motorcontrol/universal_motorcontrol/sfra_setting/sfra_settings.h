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

//
// FILE:    sfra_settings.h
//
// TITLE:   SFRA settings definition
//

#ifndef SFRA_SETTINGS_H
#define SFRA_SETTINGS_H

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
//! \addtogroup SFRA SETTINGS
//! @{
//
//*****************************************************************************

//
// includes
//
#include "sfra_f32.h"
//#include "sfra_gui_scicomms_driverlib.h"

// platforms
#include "hal.h"


#define SFRA_GUI_PLOT_GH_H  1
#define SFRA_GUI_PLOT_GH_CL 2


//
// API prototypes
//
extern void configureSFRA(uint16_t plotOption, float32_t sfraISRFreq);

//
//defines
//
extern SFRA_F32 sfra1;


//
//! \brief Enumeration for SFRA test axis
//
typedef enum
{
    SFRA_TEST_D_AXIS    = 0,
    SFRA_TEST_Q_AXIS    = 1,
    SFRA_TEST_SPEEDLOOP = 2
} SFRA_TEST_e;

//
// Project Options
//

//
// 1 means control runs on C28x, otherwise it will run on CLA
//
#define C28x_CORE 1
#define CLA_CORE  2
#define CONTROL_RUNNING_ON 1

//
//SFRA Options
//
#define SFRA_FREQ_START     20

#define SFRA_SWEEP_SPEED    1           // the speed of the sweep

//
// SFRA step Multiply = 10^(1/No of steps per decade(40))
//
#define SFRA_FREQ_STEP_MULTIPLY         ((float32_t)(1.263355))
#define SFRA_AMPLITUDE                  ((float32_t)(0.005))
#define SFRA_FREQ_LENGTH                22

#define SFRA_GUI_VBUS_CLK               DEVICE_LSPCLK_FREQ      // 30MHz
#define SFRA_GUI_SCI_BAUDRATE           57600                   // 5.76kHz

#define SFRA_GUI_SCIRX_GPIO_PIN_CONFIG  GUI_SCI_SCIRX_PIN_CONFIG
#define SFRA_GUI_SCITX_GPIO_PIN_CONFIG  GUI_SCI_SCITX_PIN_CONFIG

#define SFRA_GUI_LED_GPIO               GUI_LED_GPIO
#define SFRA_GUI_LED_GPIO_PIN_CONFIG    GUI_LED_GPIO_GPIO_PIN_CONFIG

//
// if the following #define is set to 1 SFRA GUI indicates status on an LED
// otherwise LED code is ignored
//
#define SFRA_GUI_LED_INDICATOR          1

/* USER CODE END (section: User_Section) */

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

#endif // end of SFRA_SETTINGS_H definition
