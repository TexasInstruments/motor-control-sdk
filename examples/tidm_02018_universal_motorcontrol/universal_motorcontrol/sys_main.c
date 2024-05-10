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
//! 
//!
//! \brief  This project is used to implement motor control with sensored FOC with
//!         different position (Incremental encoder, Hall sensor) and sensorless
//!         FOC (eSMO).
//!         Supports multiple TI EVM boards.
//!


//#############################################################################
//! Make sure to include the right .syscfg file in the project based on the selected hardware kit.


//
// include the related header files
//
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/TimerP.h>

#include "user.h"
#include "sys_settings.h"
#include "sys_main.h"

__attribute__ ((section("sys_data"))) volatile SYSTEM_Vars_t systemVars;

#if defined(EPWMDAC_MODE)
#if defined(HVMTRPFC_REV1P1)
__attribute__ ((section("sys_data"))) HAL_PWMDACData_t pwmDACData;
  // HVMTRPFC_REV1P1
#else
#error EPWMDAC is not supported on this kit!
#endif  // !HVMTRPFC_REV1P1
#endif  // EPWMDAC_MODE

#if defined(SFRA_ENABLE)
__attribute__ ((section(".sfradata"))) float32_t   sfraNoiseId;
__attribute__ ((section(".sfradata"))) float32_t   sfraNoiseIq;
__attribute__ ((section(".sfradata"))) float32_t   sfraNoiseSpd;
__attribute__ ((section(".sfradata"))) float32_t   sfraNoiseOut;
__attribute__ ((section(".sfradata"))) float32_t   sfraNoiseFdb;
__attribute__ ((section(".sfradata"))) SFRA_TEST_e sfraTestLoop;
__attribute__ ((section(".sfradata"))) Bool        sfraCollectStart;
#endif  // SFRA_ENABLE

#if defined(CPUTIME_ENABLE)
__attribute__ ((section("sys_data"))) volatile float32_t cpuCyclesAv;
__attribute__ ((section("sys_data"))) volatile uint32_t cpuCycles;
__attribute__ ((section("sys_data"))) volatile uint32_t cycleCountBefore;
__attribute__ ((section("sys_data"))) volatile uint32_t cycleCountAfter;
__attribute__ ((section("sys_data"))) volatile uint32_t cycleCountAfter2;
__attribute__ ((section("sys_data"))) volatile float32_t ISRcount;
#endif  // CPUTIME_ENABLE

#if defined(CMD_CAN)
void mcanEnableTransceiver(void);
#endif // CMD_CAN

#if defined(AM263_CC)
void tca6416ConfigOutput(uint16_t port, uint16_t pin, uint16_t level);
#endif // AM263_CC


void universal_motorcontrol_main(void *args)
{
#if defined(SYSCONFIG_EN)
    systemVars.projectConfig = PRJ_DEV_SYSCONFIG;
#else
    systemVars.projectConfig = PRJ_NON_SYSCONFIG;
#endif  // SYSCONFIG_EN

#if defined(BSXL3PHGAN_REVA)
    systemVars.boardKit = BOARD_BSXL3PHGAN_REVA;    // BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
    systemVars.boardKit = BOARD_HVMTRPFC_REV1P1;    // HVMTRPFC_REV1P1
#else
#error Not select a right board for this project
#endif

#if defined(MOTOR1_ESMO)
    systemVars.estType = EST_TYPE_ESMO;         // the estimator is ESMO
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ESMO_ENC;     // the estimator is ESMO and ENC
#elif defined(MOTOR1_ABS_ENC)
    systemVars.estType = EST_TYPE_ABS_ENC;      // the sensor is ABS-ENC
#elif defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ENC;          // the sensor is ENC
#elif defined(MOTOR1_HALL)
    systemVars.estType = EST_TYPE_HALL;         // the sensor is HALL
#else
#error Not select a right estimator/sensor for this project
#endif

#if defined(MOTOR1_DCLINKSS)
    systemVars.currentSenseType = CURSEN_TYPE_SINGLE_SHUNT;
#elif defined(BSXL3PHGAN_REVA)
    systemVars.currentSenseType = CURSEN_TYPE_INLINE_SHUNT;
#else
    systemVars.currentSenseType = CURSEN_TYPE_THREE_SHUNT;
#endif  // Current Sense Type

#if defined(DATALOG_EN) && defined(STEP_RP_EN)
#error DATALOG and GRAPH_STEP_RESPONSE can't be used simultaneously on this device
#endif  // DATALOG_EN && STEP_RP_EN

#if defined(DAC128S_ENABLE) && defined(BSXL3PHGAN_REVA)
#error Don't enable DAC128S if using BSXL3PHGAN_REVA.
// BSXL3PHGAN_REVA supports CMPSS based window protection and installed on site #2 in the BoosterPack for this reason.
// SPI enable pin in site #2 is used for other purposes.
#endif  // DAC128S_ENABLE & BSXL3PHGAN_REVA

#if defined(CMD_CAN)
    MCAN_RxNewDataStatus    newDataStatus;
#endif // CMD_CAN

    // ** above codes are only for checking the settings, not occupy the memory

    //  Open drivers and boards
    Drivers_open();
    Board_driversOpen();

#if defined(CPUTIME_ENABLE)
    /* initialize PMU Cycle Counter*/
    CycleCounterP_init(SOC_getSelfCpuClk());
#endif  // CPUTIME_ENABLE

#if defined(AM263_CC)
    // Set P1.7 to high, ADC2_MUX_SEL, Select ADC4_IN0/1 for HSEC 25/27
    tca6416ConfigOutput(1, 7, 1);
#endif // AM263_CC

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    //set control parameters for motor 1
    motorHandle_M1 = (MOTOR_Handle)(&motorVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.flagEnableRunAndIdentify = FALSE;

    motorVars_M1.speedRef_Hz = 60.0f;       // Hz
    motorVars_M1.speedRef_rpm = 600.0f;     // rpm
    userParams_M1.flag_bypassMotorId = TRUE;

    initMotor1Handles(motorHandle_M1);
    initMotor1CtrlParameters(motorHandle_M1);

    // set up gate driver after completed GPIO configuration
    motorVars_M1.faultMtrNow.bit.gateDriver =
    HAL_MTR_setGateDriver(motorHandle_M1->halMtrHandle);

#if defined(EPWMDAC_MODE)
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &motorVars_M1.angleFOC_rad;               // PWMDAC1
//    pwmDACData.ptrData[0] = &motorVars_M1.anglePLL_rad;             // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleENC_rad;             // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleHall_rad;            // PWMDAC1
//    pwmDACData.ptrData[1] = &motorVars_M1.angleGen_rad;             // PWMDAC2
//    pwmDACData.ptrData[1] = &motorVars_M1.adcData.I_A.value[0];     // PWMDAC2
    pwmDACData.ptrData[1] = &motorVars_M1.speedAbs_Hz;                // PWMDAC2
    pwmDACData.ptrData[2] = &motorVars_M1.speedAbs_Hz;                // PWMDAC3
//    pwmDACData.ptrData[2] = &motorVars_M1.adcData.I_A.value[1];     // PWMDAC3
    pwmDACData.ptrData[3] = &motorVars_M1.adcData.I_A.value[1];       // PWMDAC4

    pwmDACData.offset[0] = 0.5f;    // PWMDAC1
//    pwmDACData.offset[1] = 0.5f;    // PWMDAC2
    pwmDACData.offset[1] = 0.0f;    // PWMDAC2
    pwmDACData.offset[1] = 0.0f;    // PWMDAC3
//    pwmDACData.offset[2] = 0.5f;    // PWMDAC3
    pwmDACData.offset[3] = 0.5f;    // PWMDAC4

    pwmDACData.gain[0] = 1.0f / MATH_TWO_PI;                          // PWMDAC1
//    pwmDACData.gain[1] = 1.0f / MATH_TWO_PI;                        // PWMDAC2
//    pwmDACData.gain[1] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;   // PWMDAC2
    pwmDACData.gain[1] = 1.0f / USER_MOTOR1_FREQ_MAX_Hz;              // PWMDAC2
    pwmDACData.gain[2] = 1.0f / USER_MOTOR1_FREQ_MAX_Hz;              // PWMDAC3
//    pwmDACData.gain[2] = 1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;   // PWMDAC3
    pwmDACData.gain[3] = 2.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;     // PWMDAC4
#endif  // EPWMDAC_MODE

#if defined(DATALOG_EN)
    // Initialize Datalog
    datalogHandle = DATALOG_init(&datalog, sizeof(datalog), manual, 0, 1);
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;
    datalogObj->flag_enableLogData = 1;

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = (float32_t*) &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = (float32_t*) &motorVars_M1.adcData.V_V.value[1];
    datalogObj->iptr[2] = (float32_t*) &motorVars_M1.adcData.V_V.value[2];
    datalogObj->iptr[3] = (float32_t*) &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = (float32_t*) &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = (float32_t*) &motorVars_M1.adcData.I_A.value[1];
    datalogObj->iptr[2] = (float32_t*) &motorVars_M1.adcData.I_A.value[2];
    datalogObj->iptr[3] = (float32_t*) &motorVars_M1.angleFOC_rad;
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
    datalogObj->iptr[0] = (float32_t*) &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = (float32_t*) &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[2] = (float32_t*) &motorVars_M1.speed_Hz;
    datalogObj->iptr[3] = (float32_t*) &motorVars_M1.angleFOC_rad;
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#endif  //DATALOG_EN

#if defined(SFRA_ENABLE)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, USER_M1_ISR_FREQ_Hz);

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;
    sfraNoiseOut = 0.0f;
    sfraNoiseFdb = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = FALSE;
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
    GRAPH_init(&stepRPVars,
               &motorVars_M1.speedRef_Hz, &motorVars_M1.speed_Hz,
               &motorVars_M1.IdqRef_A.value[0], &motorVars_M1.Idq_in_A.value[0],
               &motorVars_M1.IdqRef_A.value[1], &motorVars_M1.Idq_in_A.value[1]);
#endif  // STEP_RP_EN

#if defined(CMD_CAN)

    mcanEnableTransceiver();

    // Init MCAN communication
    HAL_CanMsgInit();
    HAL_initMcan();

#endif // CMD_CAN

    systemVars.flagEnableSystem = FALSE;
    motorVars_M1.flagEnableOffsetCalc = TRUE;

    //Run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

    // Hardware interrupt instance
    HwiP_Params hwiPrms;
    int32_t status;
    static HwiP_Object gAdcHwiObject;

    // Register & enable interrupt
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback = &motor1CtrlISR;
    hwiPrms.priority = 1U;
    hwiPrms.isPulse = 1U;
    status = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    // Clears ADC interrupt sources
    ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

    // Waiting one second for enable system flag to be set
    while(systemVars.flagEnableSystem == FALSE)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = TRUE;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    motorVars_M1.flagInitializeDone = TRUE;

    while(systemVars.flagEnableSystem == TRUE)
    {

        // loop while the enable system flag is TRUE
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            // toggle status LED on controller board
            systemVars.counterLEDC++;

            if(systemVars.counterLEDC > (uint16_t)(1000.0f/LED_BLINK_FREQ_Hz))
            {
                HAL_toggleGPIO(halHandle, HAL_GPIO_LED1C_BASE_ADD, HAL_GPIO_LED1C);     // Toggle on the LED

                systemVars.counterLEDC = 0;
            }

            if(motorVars_M1.motorState >= MOTOR_CL_RUNNING)
            {
                systemVars.timeWaitLEDB =
                        (uint16_t)(40000.0f / (fabs(motorVars_M1.speed_Hz) + 20.0f));

                // toggle status LED on inverter board if have
                systemVars.counterLEDB++;

                if(systemVars.counterLEDB > systemVars.timeWaitLEDB)
                {
                    // toggle status LED on BoosterPack if have

                    systemVars.counterLEDB = 0;
                }
            }
            else
            {
                GPIO_pinWriteHigh(HAL_GPIO_LED1B_BASE_ADD, HAL_GPIO_LED1B);     // Turn on the LED
            }

            systemVars.timerBase_1ms++;

            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
                    break;
                case 2:
                    calculateRMSData(motorHandle_M1);
                    break;
                case 3:
#if defined(MOTOR1_PI_TUNE)
                    // Tune the gains of the controllers
                    tuneControllerGains(motorHandle_M1);
#endif      // MOTOR1_PI_TUNE
                    break;
                case 4:     // calculate motor protection value
                    calcMotorOverCurrentThreshold(motorHandle_M1);
                    break;
                case 5:     // system control
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;
                    break;
            }

#if defined(CMD_CAN)
            if(McanMsgRdy)
            {
                McanMsgRdy = 0;

                /* Get the new data staus, indicates buffer num which received message */
                MCAN_getNewDataStatus(gMcanBaseAddr, &newDataStatus);
                MCAN_clearNewDataStatus(gMcanBaseAddr, &newDataStatus);

                if (newDataStatus.statusLow & 0x1)
                {
                    MCAN_readMsgRam(gMcanBaseAddr, MCAN_MEM_TYPE_BUF, 0, MCAN_RX_FIFO_NUM_0, &rxMsg);

                    HAL_CanRxMsgMapping_0xC0(&motorVars_M1);
                }

                // Send out CAN message
                HAL_CanTxMsgPacking(&motorVars_M1);
                HAL_SendCanMsg(0);

            }
#endif // CMD_CAN

#if defined(SFRA_ENABLE)
            SFRA_F32_runBackgroundTask(&sfra1);
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
            // Generate Step response
            GRAPH_generateStepResponse(&stepRPVars);
#endif  // STEP_RP_EN
        }       // 1ms Timer

        runMotor1Control(motorHandle_M1);

} // end of while() loop

// disable the PWM
HAL_disablePWM(motorHandle_M1->halMtrHandle);

DebugP_log("end of main!!\r\n");

Board_driversClose();
Drivers_close();
}
