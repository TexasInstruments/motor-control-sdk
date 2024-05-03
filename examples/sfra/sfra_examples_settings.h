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

#ifndef _SFRA_EXAMPLES_SETTINGS_H
#define _SFRA_EXAMPLES_SETTINGS_H

#ifdef __cplusplus

extern "C" {
#endif

//
//defines
//

#define CPU_SYS_CLOCK      (200*1000000)
#define PWMSYSCLOCK_FREQ   (200*1000000)
#define ECAPSYSCLOCK_FREQ   (200*1000000)

//
// Project Options
//

//
// 1 means control runs on C28x , otherwise it will run on CLA
//
#define C28x_CORE 1
#define CLA_CORE 2
#define CONTROL_RUNNING_ON 1

//
// Power Stage Related Values
//
#define PFC_PWM_SWITCHING_FREQUENCY ((float)100*1000)
#define PFC_PWM_PERIOD ((PWMSYSCLOCK_FREQ) / (PFC_PWM_SWITCHING_FREQUENCY))

//
// Control Loop Design
//
#define CONTROL_ISR_FREQUENCY ((PFC_PWM_SWITCHING_FREQUENCY) / (CNTRL_ISR_FREQ_RATIO))
#define CNTRL_ISR_FREQ_RATIO    1
#define VOLTAGE_LOOP_RUN_RATIO  1

//
//SFRA Options
//
#define SFRA_ISR_FREQ       CONTROL_ISR_FREQUENCY
#define SFRA_FREQ_START 2
//
//SFRA step Multiply = 10^(1/No of steps per decade(40))
//
#define SFRA_FREQ_STEP_MULTIPLY (float)1.105
#define SFRA_AMPLITUDE (float)0.005
#define SFRA_FREQ_LENGTH 100
#define SCI_VBUS_CLK 50000000
#define SFRA_GUI_SCI_BAUDRATE 57600


#define PI_VALUE 3.141592653589

#define GI_PI_KP    (float) 0.3496503407
#define GI_PI_KI    (float) 0.0020000510

//#define C28x_CONTROLISR_INTERRUPT_PIE_GROUP_NO INTERRUPT_ACK_GROUP3

#define C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE CSL_CONTROLSS_G0_EPWM0_U_BASE
#define C28x_CONTROLISR_INTERRUPT INT_XBAR_EPWM0_INT

#define GI_PI_MAX   1.0
#define GI_PI_MIN   -1.0

//
// PWM pin, ADC, SDFM, Relay Selection related variables
//

#define PWM_BASE                   CSL_CONTROLSS_G0_EPWM0_U_BASE

#define PWM_H_GPIO                 43
#define PWM_H_GPIO_PIN_CONFIG      CSL_IOMUX_EPWM0_A_CFG_REG

#define PWM_L_GPIO                 44
#define PWM_L_GPIO_PIN_CONFIG      CSL_IOMUX_EPWM0_B_CFG_REG

#define GPIO_PROFILING1 13
//#define GPIO_PROFILING1_SET GPIO_GPASET_GPIO16
//#define GPIO_PROFILING1_CLEAR GPIO_GPACLEAR_GPIO16
#define GPIO_PROFILING1_PIN_CONFIG CSL_IOMUX_SPI0_D0_CFG_REG

#define GPIO_PROFILING2 14
//#define GPIO_PROFILING2_SET GPIO_GPASET_GPIO17
//#define GPIO_PROFILING2_CLEAR GPIO_GPACLEAR_GPIO17
#define GPIO_PROFILING2_PIN_CONFIG CSL_IOMUX_SPI0_D1_CFG_REG


/* USER CODE END (section: User_Section) */


#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif
