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


//! \file
//! \brief  Contains public interface to various functions related
//!         to the HAL object
//!


#ifndef HAL_H
#define HAL_H


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
//! \defgroup HAL HAL
//! @{
//
//*****************************************************************************

// the includes
#include "userParams.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

// platforms
#include "hal_obj.h"
#include "pwmdac.h"
#include "svgen_current.h"

#if defined(DATALOG_EN)
#include "datalog.h"
#include "datalog_input.h"
#endif  //DATALOG_EN


// the globals
extern HAL_Handle    halHandle;
extern HAL_Obj       hal;

extern volatile uint16_t mtrPIEIER;
extern volatile uint16_t mtrIER;

extern uint32_t loadStart_hal_data;
extern uint32_t loadEnd_hal_data;
extern uint32_t loadSize_hal_data;

extern uint32_t loadStart_user_data;
extern uint32_t loadEnd_user_data;
extern uint32_t loadSize_user_data;

extern uint32_t loadStart_foc_data;
extern uint32_t loadEnd_foc_data;
extern uint32_t loadSize_foc_data;

extern uint32_t loadStart_sys_data;
extern uint32_t loadEnd_sys_data;
extern uint32_t loadSize_sys_data;

extern uint32_t loadStart_datalog_data;
extern uint32_t loadEnd_datalog_data;
extern uint32_t loadSize_datalog_data;

extern uint32_t loadStart_sfradata;
extern uint32_t loadEnd_sfradata;
extern uint32_t loadSize_sfradata;


// **************************************************************************
// the defines
//

// DAC SPI Baud Rate
#define DACS_SPI_BITRATE            mcspi1.mcspiChannel[0].bitRate //(2000000L)         // 2MHz

//! Trip Zones all interrupt
//!
#define HAL_TZFLAG_INTERRUPT_ALL    EPWM_TZ_INTERRUPT_DCBEVT2 |                \
                                    EPWM_TZ_INTERRUPT_DCBEVT1 |                \
                                    EPWM_TZ_INTERRUPT_DCAEVT2 |                \
                                    EPWM_TZ_INTERRUPT_DCAEVT1 |                \
                                    EPWM_TZ_INTERRUPT_OST |                    \
                                    EPWM_TZ_INTERRUPT_CBC

#define HAL_TZSEL_SIGNALS_ALL       EPWM_TZ_SIGNAL_CBC1 |                      \
                                    EPWM_TZ_SIGNAL_CBC2 |                      \
                                    EPWM_TZ_SIGNAL_CBC3 |                      \
                                    EPWM_TZ_SIGNAL_CBC4 |                      \
                                    EPWM_TZ_SIGNAL_CBC5 |                      \
                                    EPWM_TZ_SIGNAL_CBC6 |                      \
                                    EPWM_TZ_SIGNAL_DCAEVT2 |                   \
                                    EPWM_TZ_SIGNAL_DCBEVT2 |                   \
                                    EPWM_TZ_SIGNAL_OSHT1 |                     \
                                    EPWM_TZ_SIGNAL_OSHT2 |                     \
                                    EPWM_TZ_SIGNAL_OSHT3 |                     \
                                    EPWM_TZ_SIGNAL_OSHT4 |                     \
                                    EPWM_TZ_SIGNAL_OSHT5 |                     \
                                    EPWM_TZ_SIGNAL_OSHT6 |                     \
                                    EPWM_TZ_SIGNAL_DCAEVT1 |                   \
                                    EPWM_TZ_SIGNAL_DCBEVT1

//! \brief Defines the PWM frequency for PWMDAC
//!
#define HA_PWMDAC_FREQ_KHZ         100.0f

//! \brief Defines the comparator number for current protection
//!
#define HAL_NUM_CMPSS_CURRENT       3

//------------------------------------------------------------------------------
#if defined(BSXL3PHGAN_REVA)
//------------------------------------------------------------------------------
#define COM_CAN_BASE                    CONFIG_MCAN0_BASE_ADDR

//for SFRA
#define GUI_LED_GPIO                    CONFIG_GUI_LED_GPIO_PIN //25
#define GUI_LED_GPIO_BASE_ADDR          CONFIG_GUI_LED_GPIO_BASE_ADDR

// Install the boostxlPak or EVM on site 2 (near CPSW) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         CONFIG_EPWM13_BASE_ADDR
#define MTR1_PWM_V_BASE         CONFIG_EPWM3_BASE_ADDR
#define MTR1_PWM_W_BASE         CONFIG_EPWM9_BASE_ADDR


#define MTR1_CMPSS_U_BASE       CONFIG_CMPSS_IU_BASE_ADDR
#define MTR1_CMPSS_V_BASE       CONFIG_CMPSS_IV_BASE_ADDR
#define MTR1_CMPSS_W_BASE       CONFIG_CMPSS_IW_BASE_ADDR

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO                CONFIG_GATE_EN_GPIO_PIN  //67
#define MTR1_GATE_EN_GPIO_BASE_ADD       CONFIG_GATE_EN_GPIO_BASE_ADDR

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     CONFIG_M1_OT_PIN  //33
#define MTR1_PM_nFAULT_GPIO_BASE_ADD     CONFIG_M1_OT_BASE_ADDR


#define MTR1_QEP_BASE           CONFIG_EQEP_BASE_ADDR

#define MTR1_HALL_U_GPIO                CONFIG_HALL_U_GPIO_PIN //137 - eQEP2 pins on LP
#define MTR1_HALL_U_GPIO_BASE_ADDR      CONFIG_HALL_U_GPIO_BASE_ADDR
#define MTR1_HALL_V_GPIO                CONFIG_HALL_V_GPIO_PIN //135
#define MTR1_HALL_V_GPIO_BASE_ADDR      CONFIG_HALL_V_GPIO_BASE_ADDR
#define MTR1_HALL_W_GPIO                CONFIG_HALL_W_GPIO_PIN //134
#define MTR1_HALL_W_GPIO_BASE_ADDR      CONFIG_HALL_W_GPIO_BASE_ADDR


#define MTR1_CAP_U_BASE         CONFIG_ECAP0_BASE_ADDR  //ECAP1_BASE
#define MTR1_CAP_V_BASE         CONFIG_ECAP1_BASE_ADDR  //ECAP2_BASE
#define MTR1_CAP_W_BASE         CONFIG_ECAP2_BASE_ADDR  //ECAP3_BASE


//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM3_SOCA  // EPWM3_SOCA

#define MTR1_ADC_I_SAMPLEWINDOW     16
#define MTR1_ADC_V_SAMPLEWINDOW     16

#define MTR1_IU_ADC_BASE        CONFIG_ADC1_BASE_ADDR //J7.67 ADC1_AIN2
#define MTR1_IV_ADC_BASE        CONFIG_ADC2_BASE_ADDR //J7.68 ADC2_AIN2
#define MTR1_IW_ADC_BASE        CONFIG_ADC3_BASE_ADDR //J7.69 ADC3_AIN2
#define MTR1_VU_ADC_BASE        CONFIG_ADC3_BASE_ADDR //J7.64 ADC3_AIN1
#define MTR1_VV_ADC_BASE        CONFIG_ADC4_BASE_ADDR //J7.65 ADC4_AIN1
#define MTR1_VW_ADC_BASE        CONFIG_ADC0_BASE_ADDR //J7.66 ADC0_AIN2
#define MTR1_VDC_ADC_BASE       CONFIG_ADC2_BASE_ADDR //J7.63 ADC2_AIN1

#define MTR1_IU_ADCRES_BASE     CONFIG_ADC1_RESULT_BASE_ADDR
#define MTR1_IV_ADCRES_BASE     CONFIG_ADC2_RESULT_BASE_ADDR
#define MTR1_IW_ADCRES_BASE     CONFIG_ADC3_RESULT_BASE_ADDR
#define MTR1_VU_ADCRES_BASE     CONFIG_ADC3_RESULT_BASE_ADDR
#define MTR1_VV_ADCRES_BASE     CONFIG_ADC4_RESULT_BASE_ADDR
#define MTR1_VW_ADCRES_BASE     CONFIG_ADC0_RESULT_BASE_ADDR
#define MTR1_VDC_ADCRES_BASE    CONFIG_ADC2_RESULT_BASE_ADDR

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN2
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN2
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN2
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN1
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN1
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN2
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER0         // SOC0-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER0         // SOC0-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER0         // SOC0-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // SOC1
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // SOC1
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER1         // SOC1
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER1         // SOC1

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // SOC0-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // SOC0-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER1         // SOC0-PPB2

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       CONFIG_EPWM3_BASE_ADDR          // EPWM3

#define MTR1_ADC_INT_BASE       CONFIG_ADC0_BASE_ADDR           // ADC0_BASE
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1                 // ADC0_INT1-SOC1
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER1                 // ADC0_INT1-SOC1


#define MTR1_CMPSS_DACH_VALUE   (2048 + 1024 + 512)          //3584
#define MTR1_CMPSS_DACL_VALUE   (2048 - 1024 - 512)          //512
// end of BSXL3PHGAN_REVA

//------------------------------------------------------------------------------
#elif defined(HVMTRPFC_REV1P1)
//------------------------------------------------------------------------------
#define COM_CAN_BASE            CONFIG_MCAN0_BASE_ADDR

//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           CONFIG_EPWM5_DAC_BASE_ADDR
#define EPWMDAC2_BASE           CONFIG_EPWM5_DAC_BASE_ADDR
#define EPWMDAC3_BASE           CONFIG_EPWM6_DAC_BASE_ADDR
#define EPWMDAC4_BASE           CONFIG_EPWM6_DAC_BASE_ADDR

//------------------------------------------------------------------------------
//! \ Motor 1
#define MTR1_PWM_U_BASE         CONFIG_EPWM0_BASE_ADDR
#define MTR1_PWM_V_BASE         CONFIG_EPWM1_BASE_ADDR
#define MTR1_PWM_W_BASE         CONFIG_EPWM2_BASE_ADDR


// Three shunts
#define MTR1_CMPSS_U_BASE       CONFIG_CMPSS_IU_BASE_ADDR
#define MTR1_CMPSS_V_BASE       CONFIG_CMPSS_IV_BASE_ADDR
#define MTR1_CMPSS_W_BASE       CONFIG_CMPSS_IW_BASE_ADDR


//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO                CONFIG_GATE_EN_GPIO_PIN  // N/A
#define MTR1_GATE_EN_GPIO_BASE_ADD       CONFIG_GATE_EN_GPIO_BASE_ADDR

////! \brief Defines the gpio for the nFAULT of Power Module
//#define MTR1_PM_nFAULT_GPIO              CONFIG_PM_NFAULT_GPIO_PIN  //40
//#define MTR1_PM_nFAULT_GPIO_BASE_ADD     CONFIG_PM_NFAULT_GPIO_BASE_ADDR

//! \brief Defines the gpio for the OCP of Power Module
// not used
//#define MTR1_PM_nOCP_GPIO       59

#define MTR1_QEP_BASE           CONFIG_EQEP_BASE_ADDR//EQEP1_BASE

//! \brief Defines the CAP for hall sensor
#if defined(MOTOR1_HALL)

#define MTR1_HALL_U_GPIO                CONFIG_HALL_U_GPIO_PIN //17
#define MTR1_HALL_U_GPIO_BASE_ADDR      CONFIG_HALL_U_GPIO_BASE_ADDR
#define MTR1_HALL_V_GPIO                CONFIG_HALL_V_GPIO_PIN //18
#define MTR1_HALL_V_GPIO_BASE_ADDR      CONFIG_HALL_V_GPIO_BASE_ADDR
#define MTR1_HALL_W_GPIO                CONFIG_HALL_W_GPIO_PIN //16
#define MTR1_HALL_W_GPIO_BASE_ADDR      CONFIG_HALL_W_GPIO_BASE_ADDR

#define MTR1_CAP_U_BASE         CONFIG_ECAP0_BASE_ADDR  //ECAP1_BASE
#define MTR1_CAP_V_BASE         CONFIG_ECAP1_BASE_ADDR  //ECAP2_BASE
#define MTR1_CAP_W_BASE         CONFIG_ECAP2_BASE_ADDR  //ECAP3_BASE

#endif  // MOTOR1_HALL


//------------------------------------------------------------------------------
// ADC
// Three shunts
#define MTR1_ADC_TRIGGER_SOC    ADC_TRIGGER_EPWM0_SOCA  // EPWM0_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     28
#define MTR1_ADC_V_SAMPLEWINDOW     40

#define MTR1_IU_ADC_BASE        CONFIG_ADC1_BASE_ADDR // ADC1_AIN3 ---> CMPSSA3: inL (-IN): This CMPSS is not suitable for window comparison
#define MTR1_IV_ADC_BASE        CONFIG_ADC1_BASE_ADDR // ADC1_AIN5 ---> CMPSSB3: inH/inL (+IN/-IN)
#define MTR1_IW_ADC_BASE        CONFIG_ADC0_BASE_ADDR // ADC0_AIN5 ---> CMPSSB1: inH/inL (+IN/-IN)
#define MTR1_VU_ADC_BASE        CONFIG_ADC3_BASE_ADDR // ADC3_AIN1
#define MTR1_VV_ADC_BASE        CONFIG_ADC3_BASE_ADDR // ADC3_AIN0
#define MTR1_VW_ADC_BASE        CONFIG_ADC1_BASE_ADDR // ADC1_AIN4
#define MTR1_VDC_ADC_BASE       CONFIG_ADC4_BASE_ADDR // ADC4_AIN1

#define MTR1_IU_ADCRES_BASE     CONFIG_ADC1_RESULT_BASE_ADDR // ADC1_AIN3
#define MTR1_IV_ADCRES_BASE     CONFIG_ADC1_RESULT_BASE_ADDR // ADC1_AIN5
#define MTR1_IW_ADCRES_BASE     CONFIG_ADC0_RESULT_BASE_ADDR // ADC0_AIN5
#define MTR1_VU_ADCRES_BASE     CONFIG_ADC3_RESULT_BASE_ADDR // ADC3_AIN1
#define MTR1_VV_ADCRES_BASE     CONFIG_ADC3_RESULT_BASE_ADDR // ADC3_AIN0
#define MTR1_VW_ADCRES_BASE     CONFIG_ADC1_RESULT_BASE_ADDR // ADC1_AIN4
#define MTR1_VDC_ADCRES_BASE    CONFIG_ADC4_RESULT_BASE_ADDR // ADC4_AIN1

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN3           // ADC1_AIN3
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN5           // ADC1_AIN5
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN5           // ADC0_AIN5
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN1           // ADC3_AIN1
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN0           // ADC3_AIN0
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN4           // ADC1_AIN4
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1           // ADC4_AIN1

// Based on pin assignment in this inverter, same SOC is not used for all three-phase
// current and voltage measurements
#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER0         // ADC1_AIN3-SOC0-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADC1_AIN5-SOC1-PPB2
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER0         // ADC0_AIN5-SOC0-PPB1
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADC3_AIN1-SOC1
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADC3_AIN0-SOC2
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADC1_AIN4-SOC2
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER2         // ADC4_AIN1-SOC2

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADC1_AIN3-SOC0-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADC1_AIN5-SOC1-PPB2
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADC0_AIN5-SOC0-PPB1

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE

#define MTR1_ADC_INT_BASE       CONFIG_ADC0_BASE_ADDR   // ADC0_BASE
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADC0_INT1-SOC2
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER2         // ADC0_INT1-SOC2

//------------------------------------------------------------------------------
// CMPSS

#define MTR1_CMPSS_DACH_VALUE   (2048 + 1024 + 512)
#define MTR1_CMPSS_DACL_VALUE   (2048 - 1024 - 512)


// end of HVMTRPFC_REV1P1

//=============================================================================
//------------------------------------------------------------------------------
#else   // Not select a kit
#error Not select a kit and define the symbols in hal.h
#endif   // Not select a kit

// **************************************************************************
// the typedefs
//------------------------------------------------------------------------------
//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLEDOff              GPIO_pinWriteHigh

//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLEDOn               GPIO_pinWriteLow

//! \brief Defines the function to toggle LEDs
//!
#define HAL_toggleLED               HAL_toggleGPIO

//! \brief Enumeration for the LED numbers
//!
#if defined(BSXL3PHGAN_REVA)
#define HAL_GPIO_LED1C               CONFIG_LED1C_PIN  //GPIO26   //!< GPIO pin number for LaunchPad LED 1
#define HAL_GPIO_LED1C_BASE_ADD      CONFIG_LED1C_BASE_ADDR
#define HAL_GPIO_LED1B               CONFIG_LED1B_PIN  //GPIO 93   //!< GPIO pin number for BoostxlPak LED 1
#define HAL_GPIO_LED1B_BASE_ADD      CONFIG_LED1B_BASE_ADDR
#define HAL_GPIO_LED2B               CONFIG_LED2B_PIN  //GPIO 125   //!< GPIO pin number for BoostxlPak LED 2
#define HAL_GPIO_LED2B_BASE_ADD      CONFIG_LED2B_BASE_ADDR

// BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
#define HAL_GPIO_LED1C               CONFIG_LED1C_PIN   //!< GPIO pin number for controlCard LED 1
#define HAL_GPIO_LED1C_BASE_ADD      CONFIG_LED1C_BASE_ADDR
#define HAL_GPIO_LED1B               CONFIG_LED1B_PIN   //!< GPIO pin number for InverterBoard LED 1
#define HAL_GPIO_LED1B_BASE_ADD      CONFIG_LED1B_BASE_ADDR
#define HAL_GPIO_LED2B               CONFIG_LED2B_PIN   //!< GPIO pin number for InverterBoard LED 2
#define HAL_GPIO_LED2B_BASE_ADD      CONFIG_LED2B_BASE_ADDR
// HVMTRPFC_REV1P1
#else
#error Not defined GPIOs for LED & Debug in hal.h
#endif  //

//! \brief Enumeration for the sensor types
//!
typedef enum
{
    HAL_SENSORTYPE_CURRENT = 0,  //!< Enumeration for current sensor
    HAL_SENSORTYPE_VOLTAGE = 1   //!< Enumeration for voltage sensor
} HAL_SensorType_e;

//! \brief Enumeration for the QEP setup
//!
typedef enum
{
    HAL_QEP_QEP1=0,  //!< Select QEP1
    HAL_QEP_QEP2=1   //!< Select QEP2
} HAL_QEPSelect_e;

//! \brief Enumeration for the CPU Timer
//!
typedef enum
{
    HAL_CPU_TIMER0 = 0,  //!< Select CPU Timer0
    HAL_CPU_TIMER1 = 1,  //!< Select CPU Timer1
    HAL_CPU_TIMER2 = 2   //!< Select CPU Timer2
} HAL_CPUTimerNum_e;

// **************************************************************************
// the function prototypes

// the interrupt ISR for motor control
__attribute__ ((section(".tcm_code"))) extern void motor1CtrlISR(void  *handle);


//! \brief     Acknowledges an interrupt from the ADC so that another ADC
//!            interrupt can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_ackMtr1ADCInt(void)
{
    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);
    return;
} // end of HAL_ackADCInt() function


//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the 
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableCtrlInts(HAL_Handle handle);


//! \brief      Enables the ADC interrupts without CPU interrupts
//! \details    Enables the ADC interrupts to only trigger CLA, and without
//!             interrupting the CPU
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableADCIntsToTriggerCLA(HAL_Handle handle);


//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_MTR_Handle handle,uint16_t *pDutyCycles)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                               EPWM_COUNTER_COMPARE_A);
  return;
} // end of HAL_getDutyCycles() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint16_t HAL_getNumCurrentSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
  
  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint16_t HAL_getNumVoltageSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function

//! \brief     Gets the pwm enable status
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return    The pwm enable
static inline Bool HAL_getPwmEnableStatus(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->flagEnablePWM);
} // end of HAL_getPwmStatus() function


//! \brief     Get the period of EPWM time-base module
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The periode of EPWM time-base module
static inline uint16_t
HAL_getTimeBasePeriod(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));
} // end of HAL_getTimeBasePeriod() function


//! \brief      Clear assigned memory
//! \param[in]  The memory start address
//! \param[in]  The memory size
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);


//! \brief      Configures the fault protection logic
//! \details    Sets up the trip zone inputs so that when a comparator
//!             signal from outside the micro-controller trips a fault,
//!             the EPWM peripheral blocks will force the
//!             power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupMtrFaults(HAL_MTR_Handle handle);

//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL_MTR object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL_MTR) object handle
extern HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes);


//! \brief      Reads the ADC data with offset
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void
HAL_readMtr1ADCData(HAL_ADCData_t *pADCData)
{
    float32_t value;

    // convert phase A current
    value = (float32_t)ADC_readPPBResult(MTR1_IU_ADCRES_BASE, MTR1_IU_ADC_PPB_NUM);
    pADCData->I_A.value[0] = value * pADCData->current_sf;

    // convert phase B current
    value = (float32_t)ADC_readPPBResult(MTR1_IV_ADCRES_BASE, MTR1_IV_ADC_PPB_NUM);
    pADCData->I_A.value[1] = value * pADCData->current_sf;

    // convert phase C current
    value = (float32_t)ADC_readPPBResult(MTR1_IW_ADCRES_BASE, MTR1_IW_ADC_PPB_NUM);
    pADCData->I_A.value[2] = value * pADCData->current_sf;




    // convert phase A voltage
    value = (float32_t)ADC_readResult(MTR1_VU_ADCRES_BASE, MTR1_VU_ADC_SOC_NUM);
    pADCData->V_V.value[0] = value * pADCData->voltage_sf;

    // convert phase B voltage
    value = (float32_t)ADC_readResult(MTR1_VV_ADCRES_BASE, MTR1_VV_ADC_SOC_NUM);
    pADCData->V_V.value[1] = value * pADCData->voltage_sf;

    // convert phase C voltage
    value = (float32_t)ADC_readResult(MTR1_VW_ADCRES_BASE, MTR1_VW_ADC_SOC_NUM);
    pADCData->V_V.value[2] = value * pADCData->voltage_sf;

    // convert dc bus voltage
    value = (float32_t)ADC_readResult(MTR1_VDC_ADCRES_BASE, MTR1_VDC_ADC_SOC_NUM);
    pADCData->VdcBus_V = value * pADCData->dcBusvoltage_sf;

    return;
} // end of HAL_readMtr1ADCData() functions


//! \brief     Sets the value of the internal DAC of the high comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the high comparator
static inline void
HAL_setCMPSSDACValueHigh(HAL_MTR_Handle handle,
                         const uint16_t cmpssNumber, uint16_t dacValue)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // set DAC value of the high comparator
    CMPSS_setDACValueHigh(obj->cmpssHandle[cmpssNumber], dacValue);

    return;
} // end of HAL_setCMPSSDACValueHigh() function


//! \brief     Sets the value of the internal DAC of the low comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the low comparator
static inline void
HAL_setCMPSSDACValueLow(HAL_MTR_Handle handle,
                        const uint16_t cmpssNumber, uint16_t dacValue)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // set GPIO low
  CMPSS_setDACValueLow(obj->cmpssHandle[cmpssNumber], dacValue);

  return;
} // end of HAL_setCMPSSDACValueLow() function


//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
HAL_setNumVoltageSensors(HAL_MTR_Handle handle,const uint16_t numVoltageSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function

//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
HAL_setNumCurrentSensors(HAL_MTR_Handle handle,const uint16_t numCurrentSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams);

//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupADCs(HAL_Handle handle);

#if defined(MOTOR1_HALL)
//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCAPs(HAL_MTR_Handle handle);

//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle);

//! \brief     Read the CAP counters
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \return    The CAP counters
static inline uint32_t HAL_calcCAPCount(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;
    uint32_t capSumCount = 0;

    for(cnt = 0; cnt < 3; cnt++)
    {
        capSumCount += ECAP_getEventTimeStamp(obj->capHandle[cnt], ECAP_EVENT_2);
        capSumCount += ECAP_getEventTimeStamp(obj->capHandle[cnt], ECAP_EVENT_3);
    }

    return(capSumCount);
}

#endif  // MOTOR1_HALL

//! \brief      Sets up the CMPSSs (Comparator Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCMPSSs(HAL_MTR_Handle handle);

//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);

//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGPIOs(HAL_Handle handle);

#if defined(EPWMDAC_MODE)
//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz);
#endif  // EPWMDAC_MODE

#if defined(MOTOR1_ENC)
//! \brief     Sets up the QEP peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupQEP(HAL_MTR_Handle handle);
#endif  // MOTOR1_ENC

//! \brief     Sets up the I2CA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupI2CA(HAL_Handle halHandle);

// Declare HAL_setupGate and HAL_enableDRV
#if defined(BSXL3PHGAN_REVA)
//! \brief      Enables the gate driver
//! \details    Provides the correct timing to enable the gate driver
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);
// BSXL8323RH_REVB | BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
//! \brief      Enables the gate driver
//! \details    Provides the correct timing to enable the gate driver
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);
// HVMTRPFC_REV1P1
#endif  // Declare HAL_setupGate and HAL_enableDRV

//! \brief     Sets up the CPU timer for time base
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimeBaseTimer(HAL_Handle handle,
                                   const float32_t timeBaseFreq_Hz);

//! \brief     Sets up the CPU timer for ADC trigger
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupADCTriggerTimer(HAL_Handle handle,
                                     const float32_t adcTriggerFreq_Hz);

////! \brief     Sets up the timers
////! \param[in] handle          The hardware abstraction layer (HAL) handle
////! \param[in] cpuTimerNumber  The CPU timer number
static inline void
HAL_clearCPUTimerFlag(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    TimerP_clearOverflowInt(obj->timerHandle[cpuTimerNumber]);

    return;
}   // end of HAL_clearTimerFlag() function


//! \brief     Gets CPU Timer status
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
static inline Bool
HAL_getCPUTimerStatus(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    return (TimerP_isOverflowed(obj->timerHandle[cpuTimerNumber]));
}

//! \brief     Sets up the DMA
//! \param[in] N/A
extern void HAL_setupDMA(void);


////! \brief     Toggles the GPIO pin
////! \param[in] handle      The hardware abstraction layer (HAL) handle
////! \param[in] baseAddr    The memory address of the GPIO instance being used
////! \param[in] pinNum      The GPIO number
static inline void HAL_toggleGPIO(HAL_Handle handle, uint32_t baseAddr, uint32_t pinNum)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    if (obj->toggleGPIO[0] == 0)
        {
        GPIO_pinWriteHigh(baseAddr, pinNum);
        obj->toggleGPIO[0] = 1;
        }
    else
    {
        GPIO_pinWriteLow(baseAddr, pinNum);
        obj->toggleGPIO[0] = 0;
    }
    return;
}// end of HAL_toggleGPIO() function




#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
static inline void
HAL_writePWMDACData(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // convert values from float to unit16
    uint16_t cnt;
    float32_t period;
    float32_t dacData_sat_dc;
    float32_t dacData;
    float32_t value;

    int16_t cmpValue[4];

    period = (float32_t)pPWMDACData->periodMax;

    for(cnt = 0; cnt < 4; cnt++)
    {
        dacData = (*pPWMDACData->ptrData[cnt]);
        dacData_sat_dc = dacData * pPWMDACData->gain[cnt] +
                pPWMDACData->offset[cnt];

        value = dacData_sat_dc * period;

        cmpValue[cnt] = (int16_t)MATH_sat(value, period, 0);
    }

    // write the DAC data
    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_1],
                                EPWM_COUNTER_COMPARE_A, cmpValue[0]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_2],
                                EPWM_COUNTER_COMPARE_B, cmpValue[1]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_3],
                                EPWM_COUNTER_COMPARE_A, cmpValue[2]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_4],
                                EPWM_COUNTER_COMPARE_B, cmpValue[3]);

    return;
} // end of HAL_writePWMDACData() function
  // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData);


//! \brief      Clear assigned memory
//! \param[in]  The memory start address
//! \param[in]  The memory size
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);

//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t
HAL_readPWMPeriod(HAL_MTR_Handle handle,const uint16_t pwmNumber)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // the period value to be returned
  uint16_t pwmPeriodValue;

  pwmPeriodValue = EPWM_getTimeBasePeriod(obj->pwmHandle[pwmNumber]);

  return(pwmPeriodValue);
} // end of HAL_readPWMPeriod() function

//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMData(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    float32_t period = (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));

    uint16_t pwmCnt;

    for(pwmCnt=0; pwmCnt<3; pwmCnt++)
    {
      // compute the value
        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];      // Negative
        float32_t V_sat_pu = MATH_sat(V_pu, 0.5, -0.5);         // -0.5~0.5
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;                 // 0~1.0
        pPWMData->cmpValue[pwmCnt]  = (int16_t)(V_sat_dc_pu * period);  //

        if(pPWMData->cmpValue[pwmCnt] < pPWMData->minCMPValue)
        {
            pPWMData->cmpValue[pwmCnt] = pPWMData->minCMPValue;
        }

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMData->cmpValue[pwmCnt]);

        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_B,
                                    pPWMData->cmpValue[pwmCnt]);
    }

    return;
} // end of HAL_writePWMData() function


//! \brief      Enables the PWM devices for motor control
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(HVMTRPFC_REV1P1)
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    // HVMTRPFC_REV1P1
#elif defined(BSXL3PHGAN_REVA)

    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);


    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    // BSXL3PHGAN_REVA
#else   //!HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA


    obj->flagEnablePWM = TRUE;

    return;
} // end of HAL_enablePWM() function


//! \brief      Enables the PWM for braking
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enableBrakePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

#if defined(HVMTRPFC_REV1P1)
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, FALSE);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, FALSE);
    }

    GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    // HVMTRPFC_REV1P1
#elif defined(BSXL3PHGAN_REVA)
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, FALSE);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, FALSE);
    }

    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

#else   //!(HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA)
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, FALSE);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, FALSE);
    }
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA

    obj->flagEnablePWM = FALSE;

    return;
} // end of HAL_enableBrakePWM() function


//! \brief      Enables the PWM for braking
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_exitBrakeResetPWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

#if defined(HVMTRPFC_REV1P1)


    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);


    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, TRUE);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, TRUE);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }

    GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);
    //HVMTRPFC_REV1P1
#elif defined(BSXL3PHGAN_REVA)
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, TRUE);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, TRUE);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }

    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    // BSXL3PHGAN_REVA
#else   //!HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, TRUE);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, TRUE);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA

    obj->flagEnablePWM = FALSE;

    return;
} // end of HAL_exitBrakeResetPWM() function

//! \brief      clear fault status of motor control
//! \details
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_clearMtrFaultStatus(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);


    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_clearMtrFaultStatus() function

//! \brief      Disables the PWM device for motor control
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePWM(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(BSXL3PHGAN_REVA)

  GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

#endif  // BSXL3PHGAN_REVA

  obj->flagEnablePWM = FALSE;

  return;
} // end of HAL_disablePWM() function

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setupPWMs(HAL_MTR_Handle handle);

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
static inline uint16_t HAL_getMtrTripFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t tripFault = 0;

    tripFault = (EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1));


    return(tripFault);
}

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                               const uint16_t dacValH, const uint16_t dacValL);

#if defined(MOTOR1_OVM)
//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] ignoreShunt  The low side shunt that should be ignored
//! \param[in] midVolShunt  The middle length of output voltage
static inline void HAL_setTrigger(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData,
                                  const SVGENCURRENT_IgnoreShunt_e ignoreShunt,
                                  const SVGENCURRENT_VmidShunt_e midVolShunt)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    int16_t pwmNum = midVolShunt;
    int16_t pwmCMPA = EPWM_getCounterCompareValue(obj->pwmHandle[pwmNum],
                                                   EPWM_COUNTER_COMPARE_A);

    int16_t pwmSOCCMP = 5;

    if(ignoreShunt == SVGENCURRENT_USE_ALL)
    {
        // Set up event source for ADC trigger
        EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC,
                                 3);
    }
    else
    {
        pwmSOCCMP = pwmCMPA - pPWMData->deadband - pPWMData->noiseWindow;

        if(pwmSOCCMP <= 0)
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_U_CMPC,
                                     3);
        }
        else
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_D_CMPC,
                                     3);
        }

    }

    //
    pPWMData->socCMP = pwmSOCCMP;

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                pwmSOCCMP);
    return;
} // end of HAL_setTrigger() function
#endif  // MOTOR1_OVM

//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] deadband     The setting deadband for mosfet gate driver
//! \param[in] noisewindow  The noise window
//! \param[in] adcSample_us The adc sample time
extern void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData,
                                const float32_t systemFreq_MHz, const float32_t deadband_us,
                                const float32_t noiseWindow_us, const float32_t adcSample_us);





//! \brief     Sets up the gate driver for inverter board
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern Bool HAL_MTR_setGateDriver(HAL_MTR_Handle handle);

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

#endif // end of HAL_H definition

