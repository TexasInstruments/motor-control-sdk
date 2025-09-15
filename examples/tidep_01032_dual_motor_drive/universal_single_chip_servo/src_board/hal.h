/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#include "svgen_current.h"

#if defined(MOTOR1_ABS_ENC) || defined(MOTOR2_ABS_ENC)
#include "encoder.h"
#endif
#if defined(DATALOG_EN)
#include "datalog.h"
#include "datalog_input.h"
#endif  //DATALOG_EN

#if defined (SOC_AM243X)
#include "app_epwm.h"
#endif  

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


//------------------------------------------------------------------------------
#if defined(BP_AM2BLDCSERVO)

// EPWM defines 
#define MTR1_PWM_U_BASE         EPWM0_AXIS1_BASE_ADDR
#define MTR1_PWM_V_BASE         EPWM1_AXIS1_BASE_ADDR
#define MTR1_PWM_W_BASE         EPWM2_AXIS1_BASE_ADDR
#define MTR2_PWM_U_BASE         EPWM0_AXIS2_BASE_ADDR
#define MTR2_PWM_V_BASE         EPWM1_AXIS2_BASE_ADDR
#define MTR2_PWM_W_BASE         EPWM2_AXIS2_BASE_ADDR
#define MTR2_PWM_WB_BASE        EPWM2_B_AXIS2_BASE_ADDR

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO                CONFIG_AXIS1_GATE_EN_GPIO_PIN  //67
#define MTR1_GATE_EN_GPIO_BASE_ADD       CONFIG_AXIS1_GATE_EN_GPIO_BASE_ADDR

#define MTR2_GATE_EN_GPIO                CONFIG_AXIS2_GATE_EN_GPIO_PIN  //67
#define MTR2_GATE_EN_GPIO_BASE_ADD       CONFIG_AXIS2_GATE_EN_GPIO_BASE_ADDR

/*PRU defines */ 
/*PRUICSS*/
PRUICSS_Handle gPruIcssXHandle;
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define PRUICSS_INSTANCE  CONFIG_PRU_ICSS0
#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define ICSS_PRU_IEP_CLOCK CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ //CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ, add from sysconfig
#define ENDAT_INPUT_CLOCK_UART_FREQUENCY CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ
#define PRUICSS_ENABLE_SA_MUX_MODE 1

/*EnDat Encoder defines*/
#define  ENDAT_PRUICSSx      CONFIG_ENDAT0_PRUICSSx
#define  ENDAT_PRUICSS_SLICEx        CONFIG_ENDAT0_PRUICSS_PRUx

#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
#define  MOTOR1_ENDAT_PRUICSS_CORE          PRUICSS_RTU_PRU1 
#else
#define  MOTOR1_ENDAT_PRUICSS_CORE          PRUICSS_RTU_PRU0
#endif

#define  MOTOR1_ENDAT_ENABLE_CHANNEL        0
#define  MOTOR1_PRU_TRIGGER_HOST_ENDAT_EVT_NUMBER            ( 18 )
#define  MOTOR1_ICSSG_PRU_ENDAT_INT_NUM    CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0

#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
#define  MOTOR2_ENDAT_PRUICSS_CORE          PRUICSS_TX_PRU1
#else
#define  MOTOR2_ENDAT_PRUICSS_CORE          PRUICSS_TX_PRU0
#endif

#define  MOTOR2_ENDAT_ENABLE_CHANNEL        2
#define  MOTOR2_PRU_TRIGGER_HOST_ENDAT_EVT_NUMBER            ( 20 )
#define  MOTOR2_ICSSG_PRU_ENDAT_INT_NUM    CSLR_R5FSS0_CORE1_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_1

#define  ENDAT_RX_FIFO_CLOCK_SOURCE  CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE
#define  ENDAT_TX_FIFO_CLOCK_SOURCE  CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE

#define  ENDAT_ENABLE_CHANNEL_MASK  (CONFIG_ENDAT0_CHANNEL0<<0|CONFIG_ENDAT0_CHANNEL1<<1|CONFIG_ENDAT0_CHANNEL2<<2) 
#define  ENDAT_WAIT_5_SECOND        5000

#if CONFIG_ENDAT0_TX_RX_FIFO_CLOCK_SOURCE == 1
#define ENDAT_RX_INPUT_CLOCK_FREQUENCY ICSS_PRU_CORE_CLOCK
#define ENDAT_TX_INPUT_CLOCK_FREQUENCY ICSS_PRU_CORE_CLOCK
#else
#define ENDAT_RX_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY
#define ENDAT_TX_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY
#endif

#define ENDAT_RX_SAMPLE_SIZE    7

#define CLOCK_UPDATE                       100
#define CONFIG_TST_DELAY                   103
#define ENDAT_FREQUENCY                    8000000
/* Position feedback trigger point is set to 25 microseconds.
 * This is calculated as (25 * 300000000) / 10000000,
 * where the IEP clock is 300MHz.
 */
#define ENDAT_TRIGGER_POINT                7500


/*SDFM defines*/
#define SDFM_PRUICSSx    CONFIG_SDFM0_ICSSGx
#define  SDFM_PRUICSS_SLICEx        CONFIG_SDFM0_SLICE 

#if (SDFM_PRUICSS_SLICEx == PRUICSS_PRU1)
#define  MOTOR1_SDFM_PRUICSS_CORE          PRUICSS_RTU_PRU1
#else
#define  MOTOR1_SDFM_PRUICSS_CORE          PRUICSS_RTU_PRU0
#endif

#define  SDFM_MCLK_VALUE           CONFIG_SDFM0_CHANNEL0_MCLK /*Common clock is used for all channel */
#define  SDFM_NC_OSR_VALUE         CONFIG_SDFM0_CHANNEL0_NC_OSR /*Common NC OSR is used for all channel */
#define  SDFM_NORMAL_CURRENT_TRIGGER_POINT  CONFIG_SDFM0_CHANNEL0_FIRST_TRIGGER_POINT
#define  SDFM_EPWM_SYNC_SOURCE     CONFIG_SDFM0_CHANNEL0_EPWM_SOURCE

/* R5F interrupt settings for ICSSG */
#define  MOTOR1_ICSSG_PRU_SDFM_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3 )  /* VIM interrupt number */
#define  MOTOR1_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER   ( 6 + 18 ) /* PRU event number for SDFM interrupt */

#if (SDFM_PRUICSS_SLICEx == PRUICSS_PRU1)
#define  MOTOR2_SDFM_PRUICSS_CORE          PRUICSS_PRU1
#else
#define  MOTOR2_SDFM_PRUICSS_CORE          PRUICSS_PRU0
#endif

/* R5F interrupt settings for ICSSG */
#define  MOTOR2_ICSSG_PRU_SDFM_INT_NUM          ( CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_4 )  /* VIM interrupt number */
#define  MOTOR2_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER   ( 3+18 ) /* PRU event number for SDFM interrupt */

#define BP_AM2BLDCSERVO_VDC_BUS_VOLTAGE   24.0f
/* Sigma delta filter output range for SINC3 OSR64: 64*64*64 */
#define SDFM_FULL_SCALE         262144.0f
#define SDFM_HALF_SCALE         131072.0f

#else   // Not select a kit
#error Board configuration not specified. Please define a valid board for this project.
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
#if defined(BP_AM2BLDCSERVO) /* Configure */
#define HAL_GPIO_LED1C               CONFIG_LED1C_PIN  //GPIOo_27   //!< GPIO pin number for LaunchPad LED 1
#define HAL_GPIO_LED1C_BASE_ADD      CONFIG_LED1C_BASE_ADDR
#else
#error Board configuration not specified. Please define a valid board for this project.
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

//! \brief The main interrupt service (ISR) routine
__attribute__ ((section(".tcm_code"))) extern void motor2CtrlISR(void *handle);

#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
//! \brief     Acknowledges an interrupt from the SDFM so that another SDFM
//!            interrupt can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
/*FIXME*/
static inline void HAL_ackMtrSdfmInt(uint8_t motorNum)
{
    // check the motor number
    if(motorNum == 0)
    {
        // clear the SDFM interrupt flag for motor 1
        PRUICSS_clearEvent(gPruIcssXHandle, MOTOR1_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER);
    }
    else if(motorNum == 1)
    {
        // clear the SDFM interrupt flag for motor 2
        PRUICSS_clearEvent(gPruIcssXHandle, MOTOR2_PRU_TRIGGER_HOST_SDFM_EVT_NUMBER);
    }
    else
    {
        // invalid motor number
        return;
    }   
   
    return;
} // end of HAL_ackMtrSdfmInt() function
#endif

//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_MTR_Handle handle,uint16_t *pDutyCycles)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
  /*FIXME*/
#if defined(SOC_AM243X)
  pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                               0);
  pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                                 0);
  pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                                 0);
#else
  pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                               EPWM_COUNTER_COMPARE_A);
#endif
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
#if defined(BP_AM2BLDCSERVO)
#if defined(SOC_AM243X)
//! \brief      Initializes the PRU-ICSS
void HAL_pruIcssX_init();
#endif
#endif // BP_AM2BLDCSERVO
#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
//! \brief      Initializes the SDFM
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
void HAL_setupSDFM(HAL_Handle handle);
#endif

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
extern HAL_MTR_Handle HAL_MTR_init(void *pMemory, const size_t numBytes);

#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
//! \brief      Reads the SDFM data with offset
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pSdfmData  A pointer to the SDFM data buffer
void HAL_readMtrSdfmData(HAL_sdfmData_t *pSdfmData, uint32_t motorNum);
#endif

//! \brief     Sets the value of the internal DAC of the high comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the high comparator
static inline void
HAL_setCMPSSDACValueHigh(HAL_MTR_Handle handle,
                         const uint16_t cmpssNumber, uint16_t dacValue)
{
#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
 //support is not added
#endif

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
#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
  //support is not added
#endif
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

//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGPIOs(HAL_Handle handle);

#if defined(MOTOR1_ENC) || defined(MOTOR2_ENC)
#if defined(MOTOR1_ABS_ENC) || defined(MOTOR2_ABS_ENC)
//! \brief     Sets up the Encoder peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupEncoder(HAL_Handle handle);
//! \brief   read the ansolution of the encoder position  
//! \param[in] handle  the ENC Handle
extern void HAL_getMtrEncoderPosition(ENC_Handle handle, uint32_t motorNum);
#endif // MOTOR1_ABS_ENC || MOTOR2_ABS_ENC
#endif // MOTOR1_ENC || MOTOR2_ENC

// Declare HAL_setupGate and HAL_enableDRV
#if defined(BP_AM2BLDCSERVO)
//! \brief      Enables the gate driver
//! \details    Provides the correct timing to enable the gate driver
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);
#endif  // Declare HAL_setupGate and HAL_enableDRV

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
#if defined(SOC_AM243X)
       if(pwmCnt == 2 && obj->motorNum == MTR_2)
       {
            // write the PWM data value
            EPWM_setCounterCompareValue(MTR2_PWM_WB_BASE,
                EPWM_COUNTER_COMPARE_A,
                pPWMData->cmpValue[pwmCnt]);

            EPWM_setCounterCompareValue(MTR2_PWM_WB_BASE,
                                    EPWM_COUNTER_COMPARE_B,
                                    pPWMData->cmpValue[pwmCnt]);
       }
#endif
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

#if defined(BP_AM2BLDCSERVO)
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);
#else   
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
#endif  
    obj->flagEnablePWM = TRUE;

    return;
} // end of HAL_enablePWM() function


#ifdef BRAKE_ENABLE
//! \brief      Enables the PWM for braking
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enableBrakePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

#if defined(BP_AM2BLDCSERVO) || defined(SOC_AM243X)
    /*Not supported with AM243X and BP-AM2BLDCSERVO based example*/
#else  
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
#endif 

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

#if defined(BP_AM2BLDCSERVO) || defined(SOC_AM243X)
    /*Not supported with AM243X and BP-AM2BLDCSERVO based example*/
#else   
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
#endif  
    obj->flagEnablePWM = FALSE;
    return;
} // end of HAL_exitBrakeResetPWM() function
#endif // BRAKE_ENABLE
//! \brief      clear fault status of motor control
//! \details
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_clearMtrFaultStatus(HAL_MTR_Handle handle)
{
   
    // Clear any comparator digital filter output latch
#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
    // support is not added
#else

    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
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
#endif // defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)

    return;
} // end of HAL_clearMtrFaultStatus() function

//! \brief      Disables the PWM device for motor control
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePWM(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
#if defined(BP_AM2BLDCSERVO)
   GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);
#else
  EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(BSXL3PHGAN_REVA)

  GPIO_pinWriteHigh(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

#endif  // BSXL3PHGAN_REVA
#endif
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
    uint16_t tripFault = 0;

#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
    // support is not added
#else
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
#endif

    return(tripFault);
}

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                               const uint16_t dacValH, const uint16_t dacValL);

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

