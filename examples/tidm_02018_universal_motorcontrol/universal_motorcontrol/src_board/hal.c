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
//! \brief  Contains the various functions related to the HAL object

// the includes
#include "user.h"

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalog.h"

// **************************************************************************
// the globals
__attribute__ ((section("hal_data"))) HAL_Handle    halHandle;      //!< the handle for the hardware abstraction layer
__attribute__ ((section("hal_data"))) HAL_Obj       hal;            //!< the hardware abstraction layer object

// **************************************************************************
// the functions

HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    //initialize the ADC handles
    obj->adcHandle[0] = CONFIG_ADC0_BASE_ADDR;
    obj->adcHandle[1] = CONFIG_ADC1_BASE_ADDR;
    obj->adcHandle[2] = CONFIG_ADC2_BASE_ADDR;
    obj->adcHandle[3] = CONFIG_ADC3_BASE_ADDR;
    obj->adcHandle[4] = CONFIG_ADC4_BASE_ADDR;

    // initialize the ADC results
    obj->adcResult[0] = CONFIG_ADC0_RESULT_BASE_ADDR;
    obj->adcResult[1] = CONFIG_ADC1_RESULT_BASE_ADDR;
    obj->adcResult[2] = CONFIG_ADC2_RESULT_BASE_ADDR;
    obj->adcResult[3] = CONFIG_ADC3_RESULT_BASE_ADDR;
    obj->adcResult[4] = CONFIG_ADC4_RESULT_BASE_ADDR;

    // initialize CAN handle
    obj->canHandle = CONFIG_MCAN0_BASE_ADDR;             //!< the CAN0 handle

    // initialize the GPIO toggle
    obj->toggleGPIO[0] = 0;

    // initialize timer handles
    obj->timerHandle[0] = CPU_DIAGNOSTICS_TIMER0_BASE_ADDR;

#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWMDAC1_BASE;
    obj->pwmDACHandle[1] = EPWMDAC2_BASE;
    obj->pwmDACHandle[2] = EPWMDAC3_BASE;
    obj->pwmDACHandle[3] = EPWMDAC4_BASE;
    // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

    return(handle);
} // end of HAL_init() function


HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

    // initialize CMPSS handle
#if defined(HVMTRPFC_REV1P1)
    obj->cmpssHandle[1] = CONFIG_CMPSS_IV_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CONFIG_CMPSS_IW_BASE_ADDR;    //!< the CMPSS handle
#else   // !(HVMTRPFC_REV1P1)
    obj->cmpssHandle[0] = CONFIG_CMPSS_IU_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[1] = CONFIG_CMPSS_IV_BASE_ADDR;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CONFIG_CMPSS_IW_BASE_ADDR;    //!< the CMPSS handle
#endif  // !(HVMTRPFC_REV1P1)

#if defined(MOTOR1_HALL)
    // initialize CAP handles for Motor 1
    obj->capHandle[0] = MTR1_CAP_U_BASE;        //!< the CAP handle
    obj->capHandle[1] = MTR1_CAP_V_BASE;        //!< the CAP handle
    obj->capHandle[2] = MTR1_CAP_W_BASE;        //!< the CAP handle
#endif // MOTOR1_HALL

    // Assign gateEnableGPIO
#if defined(BSXL3PHGAN_REVA)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;

    // HVMTRPFC_REV1P1
#elif defined(HVMTRPFC_REV1P1)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
#endif  // Assign gateEnableGPIO

    // initialize QEP driver
    obj->qepHandle = MTR1_QEP_BASE;             // the QEP handle

    // initialize motor number
    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function


void HAL_setParams(HAL_Handle handle)
{

#if defined(EPWMDAC_MODE)
    // setup the PWM DACs
    HAL_setupPWMDACs(handle, USER_SYSTEM_FREQ_MHz);
#endif  //EPWMDAC_MODE

    return;
} // end of HAL_setParams() function


void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    // setup the PWMs
    HAL_setupPWMs(handle);

#if defined(MOTOR1_ENC)
    // setup the EQEP
    HAL_setupQEP(handle);
#endif  // MOTOR1_ENC

    // setup faults
    HAL_setupMtrFaults(handle);

#if defined(MOTOR1_HALL)
    // setup the CAPs
    HAL_setupCAPs(handle);
#endif  // MOTOR1_HALL

    // disable the PWM
    HAL_disablePWM(handle);

    return;
} // end of HAL_MTR_setParams() function

#if defined(MOTOR1_HALL)
void HAL_setupCAPs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Disable ,clear all capture flags and interrupts
        ECAP_disableInterrupt(obj->capHandle[cnt],
                              (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                               ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                               ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                               ECAP_ISR_SOURCE_COUNTER_COMPARE));

        ECAP_clearInterrupt(obj->capHandle[cnt],
                            (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                             ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                             ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                             ECAP_ISR_SOURCE_COUNTER_COMPARE));

        // Disable CAP1-CAP4 register loads
        ECAP_disableTimeStampCapture(obj->capHandle[cnt]);

        // Configure eCAP
        //    Enable capture mode.
        //    One shot mode, stop capture at event 3.
        //    Set polarity of the events to rising, falling, rising edge.
        //    Set capture in time difference mode.
        //    Select input from XBAR4/5/6.
        //    Enable eCAP module.
        //    Enable interrupt.
        ECAP_stopCounter(obj->capHandle[cnt]);
        ECAP_enableCaptureMode(obj->capHandle[cnt]);

        // Sets eCAP in Capture mode
        ECAP_setCaptureMode(obj->capHandle[cnt], ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_3);

        // Sets the Capture event prescaler.
        ECAP_setEventPrescaler(obj->capHandle[cnt], 0U);

        // Sets a phase shift value count.
        ECAP_setPhaseShiftCount(obj->capHandle[cnt], 0U);

        // Sets the Capture event polarity
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

        // Configure counter reset on events
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_1);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_2);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_3);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_4);

        // Configures emulation mode.
        ECAP_setEmulationMode(obj->capHandle[cnt], ECAP_EMULATION_FREE_RUN);

        // Enable counter loading with phase shift value.
        ECAP_enableLoadCounter(obj->capHandle[cnt]);

        // Configures Sync out signal mode.
        ECAP_setSyncOutMode(obj->capHandle[cnt], ECAP_SYNC_OUT_SYNCI);

        // Set up the source for sync-in pulse
        ECAP_setSyncInPulseSource(obj->capHandle[cnt], ECAP_SYNC_IN_PULSE_SRC_DISABLE);

        // Starts Time stamp counter
        ECAP_startCounter(obj->capHandle[cnt]);

        // Enables time stamp capture
        ECAP_enableTimeStampCapture(obj->capHandle[cnt]);

        // Re-arms the eCAP module
        ECAP_reArm(obj->capHandle[cnt]);
    }

    return;
}   // HAL_setupCAPs()



void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        ECAP_setAPWMPeriod(obj->capHandle[cnt], 0);
        ECAP_setAPWMCompare(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowPeriod(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowCompare(obj->capHandle[cnt], 0x01FFFFFF);
    }

    return;
}   // HAL_resetCAPTimeStamp()

#endif  // MOTOR1_HALL


// HAL_setupGate & HAL_enableDRV
#if defined(BSXL3PHGAN_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE (nEN_uC) to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_setupGate() function

// HVMTRPFC_REV1P1
#elif defined(HVMTRPFC_REV1P1)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_setupGate() function
// HVMTRPFC_REV1P1
#else
#error No HAL_setupGate or HAL_enableDRV
#endif  // HAL_setupGate & HAL_enableDRV


void HAL_setupMtrFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t cnt;

    for(cnt=0; cnt<3; cnt++)
    {
        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZFLAG_INTERRUPT_ALL);
    }


    // Clear any spurious fault
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_setupMtrFaults() function




void HAL_setupGPIOs(HAL_Handle handle)
{

#if defined(BSXL3PHGAN_REVA)
//    GPIO_pinWriteLow(CONFIG_DRV_CAL_BASE_ADDR, CONFIG_DRV_CAL_PIN);
    GPIO_pinWriteLow(CONFIG_LED1C_BASE_ADDR, CONFIG_LED1C_PIN);
    GPIO_pinWriteLow(CONFIG_LED1B_BASE_ADDR, CONFIG_LED1B_PIN);
    GPIO_pinWriteLow(CONFIG_LED2B_BASE_ADDR, CONFIG_LED2B_PIN);
#endif //defined(BSXL3PHGAN_REVA)

    return;
}  // end of HAL_setupGPIOs() function


void HAL_setupPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    uint16_t pwmPeriodCycles = (uint16_t)(USER_M1_PWM_TBPRD_NUM);
    uint16_t numPWMTicksPerISRTick = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        numPWMTicksPerISRTick = 15;
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        numPWMTicksPerISRTick = 1;
    }

    EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);

    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                    numPWMTicksPerISRTick);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                    numPWMTicksPerISRTick);

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], pwmPeriodCycles);

    // write the PWM data value  for ADC trigger
    // EPWM1 is pinouted for phase B
    EPWM_setCounterCompareValue(obj->pwmHandle[1], EPWM_COUNTER_COMPARE_C, 10);

    return;
}  // end of HAL_setupPWMs() function


#if defined(EPWMDAC_MODE)
void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;

    // PWMDAC frequency = 100kHz, calculate the period for pwm:2000
    uint16_t  period_cycles = (uint16_t)(systemFreq_MHz *
                                  (float32_t)(1000.0f / HA_PWMDAC_FREQ_KHZ));
    uint16_t  cnt;

    for(cnt = 0; cnt < 4; cnt++)
    {


        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], period_cycles);

    }

    return;
}  // end of HAL_setupPWMDACs() function
#endif  // EPWMDAC_MODE


#if defined(MOTOR1_ENC)
void HAL_setupQEP(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj   *obj = (HAL_MTR_Obj *)handle;

    //update the maximum posistion for reset based on user motor value
    EQEP_setPositionCounterConfig(obj->qepHandle, EQEP_POSITION_RESET_MAX_POS,
                                  (uint32_t)(USER_MOTOR1_ENC_POS_MAX));

#if defined(ENC_UVW)
    EQEP_setInitialPosition(obj->qepHandle, USER_MOTOR1_ENC_POS_OFFSET);

    EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);
#endif

    // Enable the unit timer, setting the frequency to 10KHz
    // QUPRD, QEPCTL
    EQEP_enableUnitTimer(obj->qepHandle, (USER_M1_QEP_UNIT_TIMER_TICKS - 1));

    // Enable the eQEP module
    EQEP_enableModule(obj->qepHandle);

    return;
}
#endif  // MOTOR1_ENC

void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);

    return;
}   // end of HAL_setMtrCMPSSDACValue() function


void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us,
                   const float32_t adcSample_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);
    uint16_t adcSample =  (uint16_t)(adcSample_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;
    pPWMData->adcSample = adcSample;

    pPWMData->minCMPValue = deadband + noiseWindow + adcSample;

    return;
}   // end of HAL_setTriggerPrams() function


Bool HAL_MTR_setGateDriver(HAL_MTR_Handle handle)
{
    Bool driverStatus = FALSE;

    ClockP_usleep(5000L);

    // Setup Gate Enable
#if defined(BSXL3PHGAN_REVA)
    // turn on the 3PhGaN if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    // BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
    // turn on the HvKit if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    //HVMTRPFC_REV1P1
#else
#error Not select a right supporting kit!
#endif  // Setup Gate Enable

    return(driverStatus);
}

// end of file
