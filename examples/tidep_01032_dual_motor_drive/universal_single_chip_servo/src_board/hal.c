/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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

#if defined(SOC_AM243X)
/*Includes source files for SDFM & ENDAT*/

extern PRUICSS_IntcInitData icss0_intc_initdata;

/*SDFM*/
#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)

#include <current_sense/sdfm/include/sdfm_api.h>
#if (SDFM_PRUICSS_SLICE == PRUICSS_PRU1)
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu1_bin.h>
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru1_bin.h>
#else
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu0_bin.h>
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru0_bin.h>
#endif // SDFM_PRUICSS_SLICE

/*SDFM handle */
SDFM_Handle gMotorSdfm = NULL;
int32_t gSdfmInitStatus = 0;

/* Sdfm output samples, written by PRU cores */
__attribute__((section(".gSddfChSampsRaw"))) uint32_t gSdfmSampleOutput[6] = { 0 };

#endif // defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)

/*ENDAT*/
#if defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)

#include <position_sense/endat/include/endat_drv.h>
#if (ENDAT_PRUICSS_SLICE == PRUICSS_PRU1)
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_rtu_pru1_bin.h>
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_tx_pru1_bin.h>
#else
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_rtu_pru0_bin.h>
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_tx_pru0_bin.h>
#endif // ENDAT_PRUICSS_SLICE

/* EnDat channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) endat_ch_rx_info_array gEndatChInfo;

/*EnDat handle*/
endat_handle gMotorEncoderHandle = NULL;

/*ENDAT Initialization Status*/
static int32_t gEndatInitStatus = 0;
/*ENDAT Position read failure counter*/
static uint32_t gEndatPosReadFailCountM1 = 0;
static uint32_t gEndatPosReadFailCountM2 = 0;

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)

#endif // defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)

/*EPWM*/

/* variables to hold EPWM base addresses */
uint32_t gEpwm0BaseAddr;
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;
uint32_t gEpwm0BaseAddrB;

#endif // defined(SOC_AM243X)
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

    /*initialize the ICSS PRU*/
#if defined (SOC_AM243X)
    HAL_pruIcssX_init();
#endif

#if defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)
    obj->encoderHandle = &gMotorEncoderHandle;
#endif

#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)
    obj->sdfmHandle = &gMotorSdfm;
#endif

    // initialize the GPIO toggle
    obj->toggleGPIO[0] = 0;

    // initialize timer handles
    obj->timerHandle[0] = CPU_DIAGNOSTICS_TIMER0_BASE_ADDR;

    return(handle);
} // end of HAL_init() function


HAL_MTR_Handle HAL_MTR_init(void *pMemory, const size_t numBytes)
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

    if(obj->motorNum == MTR_1)
    {
        // initialize PWM handles for Motor 1
        obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
        obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
        obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle
    }
    else if(obj->motorNum == MTR_2)
    {
        // initialize PWM handles for Motor 2
        obj->pwmHandle[0] = MTR2_PWM_U_BASE;        //!< the PWM handle
        obj->pwmHandle[1] = MTR2_PWM_V_BASE;        //!< the PWM handle
        obj->pwmHandle[2] = MTR2_PWM_W_BASE;        //!< the PWM handle
    }
    else
    {
        // Invalid motor number
        return((HAL_MTR_Handle)NULL);
    }

    if(obj->motorNum == MTR_1)
    {
        // Assign gateEnableGPIO
        obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
        obj->gateEnableGPIOBaseAdd = MTR1_GATE_EN_GPIO_BASE_ADD;
    }
    else if(obj->motorNum == MTR_2)
    {
        // Assign gateEnableGPIO
        obj->gateEnableGPIO = MTR2_GATE_EN_GPIO;
        obj->gateEnableGPIOBaseAdd = MTR2_GATE_EN_GPIO_BASE_ADD;
    }
    else
    {
        // Invalid motor number
        return((HAL_MTR_Handle)NULL);
    }

    return(handle);
} // end of HAL_MTR1_init() function


void HAL_setParams(HAL_Handle handle)
{

#if defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)
    /* Initialization of Encoders*/
    HAL_setupEncoder(handle);
#endif

#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)
    /* Initialization of SDFM */
    HAL_setupSDFM(handle);
#endif
    return;
} // end of HAL_setParams() function


void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    // setup the PWMs
    HAL_setupPWMs(handle);

    // setup faults
    HAL_setupMtrFaults(handle);

    // disable the PWM
    HAL_disablePWM(handle);

    return;
} // end of HAL_MTR_setParams() function

// HAL_setupGate & HAL_enableDRV
#if defined(BP_AM2BLDCSERVO)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_pinWriteLow(obj->gateEnableGPIOBaseAdd, obj->gateEnableGPIO);

    return;
} // HAL_enableDRV() function
#else
#error Board configuration not specified. Please define a valid board for this project.
#endif  // HAL_setupGate & HAL_enableDRV

void HAL_setupMtrFaults(HAL_MTR_Handle handle)
{
    //support is not added
    return;
} // end of HAL_setupMtrFaults() function

void HAL_setupGPIOs(HAL_Handle handle)
{
#if defined(BP_AM2BLDCSERVO)
    GPIO_pinWriteLow(CONFIG_LED1C_BASE_ADDR, CONFIG_LED1C_PIN);
#endif //defined(BP_AM2BLDCSERVO)
    return;
}  // end of HAL_setupGPIOs() function


void HAL_setupPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    AppEPwmCfg_t    appEpwmCfg;

    /* Address translate */
    if(obj->motorNum == MTR_1)
    {
        gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM0_AXIS1_BASE_ADDR);
        gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM1_AXIS1_BASE_ADDR);
        gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_AXIS1_BASE_ADDR);
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);
        /* Configure the SYNCI/SYNCO mapping to tie the three PWM groups together and have PWM0 SYNC from Time Sync Router 38 */
        CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM0_CTRL_SYNCIN_SEL_SHIFT));
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 1);
        /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 38 (0x26 + 4 = 0x2A + Time Sync Router Base */
        CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((38 * 4) + 4), (0x10000 | 29));
    }
    if(obj->motorNum == MTR_2)
    {
        gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM0_AXIS2_BASE_ADDR);
        gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM1_AXIS2_BASE_ADDR);
        gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_AXIS2_BASE_ADDR);
        gEpwm0BaseAddrB = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_B_AXIS2_BASE_ADDR);
        /*SYNC for all EPWM*/
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);
        /* Configure the SYNCI/SYNCO mapping to tie the three PWM groups together and have PWM3 SYNC from Time Sync Router 39 */
        CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_SHIFT));
        /* Configure the SYNCI/SYNCO mapping to tie the three PWM groups together and have PWM6 SYNC from Time Sync Router 40 */
        CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_SYNCIN_SEL_SHIFT));
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 1);
        /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 39 (0x26 + 4 = 0x2A + Time Sync Router Base */
        CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((39 * 4) + 4), (0x10000 | 29));
        /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 40 (0x26 + 4 = 0x2A + Time Sync Router Base */
        CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((40 * 4) + 4), (0x10000 | 29));
    }

    /* Configure PWMs */
    appEpwmCfg.epwmBaseAddr = gEpwm0BaseAddr;
    appEpwmCfg.epwmCh = EPWM_OUTPUT_CH_A;
    if(obj->motorNum == MTR_1)
    {
        appEpwmCfg.epwmFuncClk = EPWM0_AXIS1_FCLK;
        appEpwmCfg.epwmTbFreq = EPWM0_AXIS1_FCLK;
    }
    else
    {
        appEpwmCfg.epwmFuncClk = EPWM0_AXIS2_FCLK;
        appEpwmCfg.epwmTbFreq = EPWM0_AXIS2_FCLK;
    }

    appEpwmCfg.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
    appEpwmCfg.epwmDutyCycle = 50;
    appEpwmCfg.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
    appEpwmCfg.cfgTbSyncIn = TRUE;
    appEpwmCfg.tbPhsValue = 0;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    appEpwmCfg.cfgTbSyncOut = TRUE;
    appEpwmCfg.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
    appEpwmCfg.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_LOW;
    appEpwmCfg.aqCfg.cmpADownAction = EPWM_AQ_ACTION_HIGH;
    appEpwmCfg.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
    appEpwmCfg.cfgDb = TRUE;
    appEpwmCfg.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
    appEpwmCfg.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
    appEpwmCfg.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
    appEpwmCfg.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
    appEpwmCfg.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
    appEpwmCfg.cfgEt = TRUE;
    appEpwmCfg.intSel = EPWM_ET_INTR_EVT_CNT_EQ_PRD;
    appEpwmCfg.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
    App_epwmConfig(&appEpwmCfg);

    if(obj->motorNum == MTR_2)
    {
        appEpwmCfg.epwmBaseAddr = gEpwm0BaseAddrB;
        App_epwmConfig(&appEpwmCfg);
    }

    /* Configure EPWM1 (Phase B), EPWM2 (Phase A) */
    appEpwmCfg.tbPhsValue = 0;
    appEpwmCfg.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP;
    appEpwmCfg.cfgEt = FALSE;
    appEpwmCfg.epwmBaseAddr = gEpwm1BaseAddr;
    App_epwmConfig(&appEpwmCfg);
    appEpwmCfg.epwmBaseAddr = gEpwm2BaseAddr;
    App_epwmConfig(&appEpwmCfg);

    if(obj->motorNum == MTR_2)
    {
        /*FIXME Debug code, doing syn of EPWM3 and EPWM6*/
        uint16_t count;
        count = EPWM_tbReadTbCount(gEpwm0BaseAddr);
        EPWM_tbWriteTbCount(gEpwm0BaseAddrB, count);
        EPWM_tbWriteTbCount(gEpwm1BaseAddr, count);
        EPWM_tbWriteTbCount(gEpwm2BaseAddr, count);

    }

    /* Force SW sync for EPWM0. Other PWMs will be sync'd through HW sync daisy-chain. */
    EPWM_tbTriggerSwSync(gEpwm0BaseAddr);

    return;
}  // end of HAL_setupPWMs() function

#if defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)
/**
 * \brief Process host commands for EnDat encoder configuration
 *
 * \details This function processes two types of host commands:
 *          - CLOCK_UPDATE (100): Configure EnDat communication clock frequency
 *          - CONFIG_TST_DELAY (103): Configure encoder response delay (tST)
 *
 * \par CLOCK_UPDATE Command:
 *      Configures the EnDat clock frequency and adjusts RX timing based on
 *      cable propagation delay:
 *      - Calculates RX/TX clock dividers
 *      - Configures RX enable counter for sampling
 *      - Adjusts RX timing to compensate for propagation delay
 *      - Configures wire delay to balance multi-channel timing
 *      - Automatically calls CONFIG_TST_DELAY to set encoder response delay
 *
 * \par CONFIG_TST_DELAY Command:
 *      Configures the encoder response delay (tST) parameter:
 *      - For frequencies >= 1MHz: Sets tST to 2us (2000 ns)
 *      - For frequencies < 1MHz: Disables tST (0 ns)
 *      - Converts delay from nanoseconds to counter increments
 *
 * \param[in]  handle          EnDat driver handle
 * \param[in]  cmd             Command type (CLOCK_UPDATE or CONFIG_TST_DELAY)
 * \param[in]  cmd_supplement  Command parameters (frequency for CLOCK_UPDATE, delay for CONFIG_TST_DELAY)
 *
 * \return None
 *
 * \note This function is used instead of endat_config_clock() API to maintain
 *       compatibility with existing application code and provide fine-grained
 *       control over clock and delay parameters.
 */
static void endat_process_host_command(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    const endat_attrs *attrs = endat_get_attrs(handle);
    int32_t status;
    int32_t i;
    uint32_t val;

    if((handle == NULL) || (attrs == NULL) || (cmd_supplement == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/attrs/cmd_supplement\n");
        return;
    }

    /* clock configuration */
    if(cmd == CLOCK_UPDATE)
    {
        if(endat_config_clock(handle, cmd_supplement->frequency) != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: clock configuration failed\n|\n|\n");
            return;
        }
        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->delay = 2000;
        }
        else
        {
            cmd_supplement->delay = 0;
        }
        /* control loop */
        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1 << i))
                {
                    cmd_supplement->selected_channel = i;
                    endat_process_host_command(handle, 103, cmd_supplement);
                }
           }
        }
        else
        {
            endat_process_host_command(handle, 103, cmd_supplement);
        }
    }
    else if(cmd == CONFIG_TST_DELAY)
    {
        /* convert tst delay from ns to tst counts*/
        val = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->delay * attrs->core_clk_freq)/1000000000));

        if(val % 5)
        {
            val += 5, val /= 5, val *= 5;
            DebugP_log("\r| WARNING: delay not multiple of 5ns, rounding to %uns\n|\n|\n",
                val);
        }

        if(val <= 0xFFFFU)
        {
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                    return;
                }
            }
            status = endat_config_tst_delay(handle, (uint16_t) val);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_config_tst_delay failed with status %d\n", status);
            }
        }
        else
        {
            DebugP_log("\r| ERROR: delay greater than %uns, enter lesser value\n|\n|\n",
                0xFFFFU);
        }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
}

/**
 * \brief Initialize and configure EnDat encoder interface for dual motor position sensing
 *
 * \details This function initializes the EnDat 2.2 driver, configures
 *          PRU firmware, and sets up position encoder communication for two motors in load share mode.
 *
 * \par Configuration Overview:
 *      - **Load Share Mode**: Channel 0 for Motor 1, Channel 2 for Motor 2
 *      - **Protocol**: EnDat 2.2
 *      - **Trigger Mode**: Periodic trigger using IEP compare events
 *      - **Operating Frequency**: 8 MHz (default for EnDat 2.2)
 *      - **Position Command**: Command 8 (encoder send position values)
 *
 * \par Memory Configuration:
 *      Position data is stored in R5F TCM (Tightly Coupled Memory):
 *      - **Default**: CPU0_BTCM_SOCVIEW for R5FSS0_CORE0 (r5fss0-0_freertos)
 *      - **Note**: If using different R5F core or memory region (e.g., R5FSS1_CORE0,
 *        ATCM), update the address translation macro in params initialization:
 *        - For R5FSS1 BTCM: Use CPU1_BTCM_SOCVIEW
 *        - For ATCM: Use CPU0_ATCM_SOCVIEW or CPU1_ATCM_SOCVIEW
 *
 * \par Channel Mapping (Load Share Mode):
 *      - **Motor 1**: Channel 0 on RTU PRU core
 *      - **Motor 2**: Channel 2 on TX PRU core
 *      - Each channel provides absolute position (single-turn + multi-turn)
 *
 * \par Initialization Sequence:
 *      1. **PRU Configuration**: Set constant tables, clear memory
 *      2. **Driver Init**: Initialize with params (pruicss_handle, channel_rx_info)
 *      3. **Firmware Load**: Load EnDat firmware to RTU PRU and TX PRU cores
 *      4. **Low Speed Init**: Configure 200 KHz for encoder info retrieval
 *      5. **Operating Speed**: Switch to 8 MHz (EnDat 2.2) or 1 MHz (EnDat 2.1)
 *      6. **IEP Periodic Mode**: Configure IEP compare events for position sampling
 *      7. **Enable Periodic Trigger**: Start continuous position updates (Command 8)
 *
 * \par Clock Configuration:
 *      - **Initialization**: 200 KHz for safe encoder info reading
 *      - **Operating Frequency**:
 *        - EnDat 2.2 encoders: 8 MHz (configured via ENDAT_FREQUENCY)
 *        - EnDat 2.1 encoders: 1 MHz (auto-detected and configured)
 *
 * \par IEP Periodic Trigger:
 *      IEP counter is enabled in HAL_setupSDFM() function. This function only configures
 *      IEP compare events for periodic position sampling:
 *      - **Trigger Point**: ENDAT_TRIGGER_POINT defines when position is sampled
 *      - **Command 8**: "Encoder send position values" - provides position
 *
 * \par Feature Limitations:
 *      This implementation is configured for specific dual motor use case. For additional
 *      features, refer to the EnDat example application (examples/position_sense/endat_diagnostic/)
 *      which includes complete implementations of:
 *      - EnDat 2.2 encoder
 *
 * \par Prerequisites:
 *      - SysConfig must define CONFIG_ENDAT0 instance with proper channel configuration
 *      - This function should be called before HAL_setupSDFM (IEP shared resource)
 *
 * \param[in]  handle  HAL handle containing hardware peripheral handles
 *
 * \return None
 *
 * \note All configuration must be completed before enabling periodic trigger. .
 */
void HAL_setupEncoder(HAL_Handle handle)
{
    /* Local variable declarations */
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_params endatParams;
    endat_cmd_supplement cmd_supplement;
    int32_t status = SystemP_FAILURE;
    void *pruicss_iep;
    uint32_t j;
    uint64_t ch0_cmp;
    uint64_t ch2_cmp;
    uint32_t cmp_reg0, cmp_reg1;
    uint16_t event = 0, event_clear = 0;

    /* PRU ICSS configuration */
    /*Set in constant table C29 for  tx pru*/
#if defined(MOTOR1_ABS_ENC)
#if ENDAT_PRUICSS_INSTANCE == 1
#if (ENDAT_PRUICSS_SLICE == PRUICSS_PRU1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C29, 0xA58);

#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C29, 0xA50);
#endif
#else
#if (ENDAT_PRUICSS_SLICE == PRUICSS_PRU1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif
#endif
#endif /*MOTOR1_ABS_ENC*/

#if defined(MOTOR1_ABS_ENC)
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(MOTOR2_ABS_ENC)
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* clear ICSS PRU data RAM and IRAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(ENDAT_PRUICSS_SLICE));
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT_PRUICSS_SLICE));

    /*Initialize EnDat parameters structure */
    endat_params_init(&endatParams);
    endatParams.pruicss_handle = gPruIcssXHandle;
    endatParams.channel_rx_info = &gEndatChInfo;
    endatParams.ch_info_global_addr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndatChInfo);

    /*Initialize EnDat driver with SysConfig index */
    gMotorEncoderHandle = endat_init(CONFIG_ENDAT0, &endatParams);
    if(gMotorEncoderHandle == NULL)
    {
        DebugP_log("\r\nERROR: EnDat initialization failed\n");
        DebugP_log("\rexit %s due to failed initialization\n", __func__);
        goto deinit;
    }
    priv = endat_get_priv(gMotorEncoderHandle);
    attrs = endat_get_attrs(gMotorEncoderHandle);

    if(priv == NULL || attrs == NULL)
    {
        DebugP_log("\r\nERROR: EnDat get priv/attrs failed\n");
        DebugP_log("\rexit %s due to failed initialization\n", __func__);
        goto deinit;
    }

    /*Load and run firmware*/
#if defined(MOTOR1_ABS_ENC)
#if ENDAT_PRUICSS_SLICE == 1
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(ENDAT_PRUICSS_SLICE), 0, (uint32_t *) EnDatFirmwareMultiMakeRtuPru1_0, sizeof(EnDatFirmwareMultiMakeRtuPru1_0));
#else
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(ENDAT_PRUICSS_SLICE), 0, (uint32_t *) EnDatFirmwareMultiMakeRtuPru0_0, sizeof(EnDatFirmwareMultiMakeRtuPru0_0));
#endif
    if(status == 0)
    {
        DebugP_log("\r\nERROR: PRUICSS_writeMemory failed for RTU PRU\n");
        goto deinit;
    }
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: PRUICSS_enableCore failed for MOTOR1\n");
        goto deinit;
    }
#endif

#if defined(MOTOR2_ABS_ENC)
#if ENDAT_PRUICSS_SLICE == 1
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(ENDAT_PRUICSS_SLICE), 0, (uint32_t *) EnDatFirmwareMultiMakeTxPru1_0, sizeof(EnDatFirmwareMultiMakeTxPru1_0));
#else
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(ENDAT_PRUICSS_SLICE), 0, (uint32_t *) EnDatFirmwareMultiMakeTxPru0_0, sizeof(EnDatFirmwareMultiMakeTxPru0_0));
#endif
    if(status == 0)
    {
        DebugP_log("\r\nERROR: PRUICSS_writeMemory failed for TX PRU\n");
        goto deinit;
    }
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: PRUICSS_enableCore failed for MOTOR2\n");
        goto deinit;
    }
#endif

    /* Check initialization acknowledgment from firmware with 5 second timeout */
    status = endat_wait_initialization(gMotorEncoderHandle, ENDAT_WAIT_5_SECOND, attrs->channel_mask);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\t Check whether encoder is connected properly\n");
        goto deinit;
    }
    /* Read encoder info at low frequency (200KHz) to avoid cable length issues */
    status = endat_config_clock(gMotorEncoderHandle, 200 * 1000);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: endat_config_clock failed\n");
        goto deinit;
    }


    /* Initialize RT measurement and get encoder info for all channels */
    for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
    {
        if(attrs->channel_mask & 1 << j)
        {
            status = endat_multi_channel_set_cur(gMotorEncoderHandle, j);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat_multi_channel_set_cur failed for channel %d\n", j);
                goto deinit;
            }
            /* Get encoder information */
            if(endat_get_encoder_info(gMotorEncoderHandle) != SystemP_SUCCESS)
            {
                DebugP_log("\rEnDat initialization channel %d failed\n", j);
                DebugP_log("\rexit %s due to failed initialization\n", __func__);
                goto deinit;
            }
        }
    }

    /* Configure operating clock frequency based on encoder type */
    if(priv->cmd_set_2_2)
    {
        /* EnDat 2.2 encoders support 8 MHz */
        cmd_supplement.frequency = ENDAT_FREQUENCY;
    }
    else
    {
        /* EnDat 2.1 encoders limited to 1 MHz */
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }

    /* Apply clock configuration with propagation delay compensation */
    endat_process_host_command(gMotorEncoderHandle, CLOCK_UPDATE, &cmd_supplement);

    pruicss_iep = attrs->iep_base_addr;

    ch0_cmp = ENDAT_TRIGGER_POINT;
    ch2_cmp = ENDAT_TRIGGER_POINT;

    /* Configure IEP for periodic mode using attrs */
    event = HW_RD_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);

    /* Configure compare events for enabled channels using attrs */
    for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
    {
        if(attrs->channel_mask & (1 << j))
        {
            /* Enable compare event from attrs */
            event |= (0x1 << (attrs->iep_cmp_event[j] + 1));
            /* Enable capture event from attrs */
            event_clear |= (0x1 << attrs->iep_cap_event[j]);
        }
    }

    /* Configure CMP event for first channel (CH0) */
    cmp_reg0 = (ch0_cmp & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = ((ch0_cmp >> 32) & 0xffffffff);
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(attrs->iep_cmp_event[0] > 7)
    {
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[0],  cmp_reg0);
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[0],  cmp_reg1);
    }
    else
    {
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[0],  cmp_reg0);
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[0],  cmp_reg1);
    }

    /* Configure CMP event for second channel (CH2) */
    cmp_reg0 = (ch2_cmp & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = ((ch2_cmp >> 32) & 0xffffffff);
    /* IEP CMP registers 8-15 have a gap in memory layout and require an additional 8-byte offset */
    if(attrs->iep_cmp_event[2] > 7)
    {
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[2],  cmp_reg0);
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + ENDAT_8_BYTE_REG_OFFSET + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[2],  cmp_reg1);
    }
    else
    {
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[2],  cmp_reg0);
        HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + ENDAT_8_BYTE_REG_OFFSET*attrs->iep_cmp_event[2],  cmp_reg1);
    }

    /* Clear and enable events */
    HW_WR_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);
    HW_WR_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, event);

    status = endat_command_process(gMotorEncoderHandle, 8, NULL);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: endat_command_process failed\n");
        goto deinit;
    }

    endat_config_periodic_trigger_cmp_mode(gMotorEncoderHandle);

    handle->encoderHandle = gMotorEncoderHandle;
    DebugP_log("\rEnDat initialization completed!!!\r\n");
    gEndatInitStatus = SystemP_SUCCESS;
    return;
deinit:
    gEndatInitStatus = SystemP_FAILURE;
    if(gMotorEncoderHandle != NULL)
    {
        endat_deinit(gMotorEncoderHandle);
    }
    DebugP_log("\rEnDat initialization failed!!!\r\n");
    return;
}

void HAL_getMtrEncoderPosition(ENC_Handle handle, uint32_t motorNum)
{
    uint32_t pos, rev;
    ENC_Obj *obj = (ENC_Obj *)handle;

    /* Read the position data from memory */
    if(motorNum == MTR_1)
    {
        if(!(gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].crc_status & ENDAT_CRC_DATA))
        {
            gEndatPosReadFailCountM1++;
            return;
        }
        else
        {
            /*Clear the CRC status*/
            gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].crc_status = 0;
        }
        pos = gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].pos_word0;
        rev = gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].pos_word1;
    }
    else if(motorNum == MTR_2)
    {
        if(!(gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].crc_status & ENDAT_CRC_DATA))
        {
            gEndatPosReadFailCountM2++;
            return;
        }
        else
        {
            /*Clear the CRC status*/
            gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].crc_status = 0;
        }
        pos = gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].pos_word0;
        rev = gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].pos_word1;
    }
    else
    {
        DebugP_log("\rError: Invalid motor number %d\n", motorNum);
        return;
    }
    /* Reverse the bits since they arrive at the PRU in reverse order */
    asm("rbit %0,%1" : "=r"(pos) : "r"(pos));
    asm("rbit %0,%1" : "=r"(rev) : "r"(rev));
    /* Cobble the multiturn data together from pos0 and pos1 and create singleturn by shifting out F1/F2 and masking the multiturn bits */

    rev = ((rev & 0x07F00000) >> 15) | ((pos & 0xF8000000) >> 27);
    pos = (pos >> 2) & 0x1FFFFFF;

    obj->thetaMech_pu = obj->mechanicalScaler * pos;

    obj->thetaMech_rad = obj->thetaMech_pu * MATH_TWO_PI;

    if(obj->encState == ENC_CALIBRATION_DONE)
    {
        obj->thetaElec_rad = obj->thetaMech_rad * obj->polePairs;

        while(obj->thetaElec_rad > MATH_TWO_PI)
           obj->thetaElec_rad -= MATH_TWO_PI;

        if(motorNum == MTR_1)
        {
            obj->thetaElec_rad =  obj->thetaElec_rad + 0.5*MATH_PI;
        }
        else
        {
            obj->thetaElec_rad =  obj->thetaElec_rad + 0.5*MATH_PI;
        }

        if(obj->thetaElec_rad >= MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad - MATH_TWO_PI;
        }
        else if(obj->thetaElec_rad <= -MATH_PI)
        {
            obj->thetaElec_rad = obj->thetaElec_rad + MATH_TWO_PI;
        }
    }
    else if(obj->encState == ENC_WAIT_FOR_INDEX)
    {
           obj->encState = ENC_CALIBRATION_DONE;
    }

    return;
}
#endif

void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
#if defined(MOTOR1_INLINE_SDFM) || defined(MOTOR2_INLINE_SDFM)
   /*Over current support has not added yet*/
#endif
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

#if defined(BP_AM2BLDCSERVO)
    // turn on the BP servo if present
    HAL_enableDRV(handle);
    ClockP_usleep(1000U);

    //BP_AM2BLDCSERVO
#else
#error Board configuration not specified. Please define a valid board for this project.
#endif  // Setup Gate Enable

    return(driverStatus);
}
#if defined(SOC_AM243X)
void HAL_pruIcssX_init()
{
    /*initialize the ICSS PRU*/
    gPruIcssXHandle = PRUICSS_open(PRUICSS_INSTANCE);

    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
#if defined(CONFIG_ENDAT0_G_MUX_EN) || defined(CONFIG_SDFM0_G_MUX_EN)
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif

    /*Enable PRU Interrupt Controller*/
    uint32_t status;
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
    DebugP_assert(SystemP_SUCCESS == status);

}
#endif

#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)
/**
 * \brief Initialize and configure SDFM (Sigma-Delta Filter Module) for dual motor current sensing
 *
 * \details This function initializes the SDFM driver with params-based API, configures
 *          PRU firmware, and sets up current sensing channels for two motors in load share mode.
 *
 * \par Configuration Overview:
 *      - **Load Share Mode**: Channels 0-2 for Motor 1, Channels 3-5 for Motor 2
 *      - **Sampling Mode**: Trigger mode using snoop-based sampling
 *      - **Sampling Rate**: Single sample per EPWM cycle
 *      - **Synchronization**: Synchronized with EPWM
 *      - **Clock Source**: IEP sync out configured for 20 MHz (default)
 *
 * \par Memory Configuration:
 *      Sample data is stored in R5F TCM (Tightly Coupled Memory):
 *      - **Default**: CPU0_BTCM_SOCVIEW for R5FSS0_CORE0 (r5fss0-0_freertos)
 *      - **Note**: If using different R5F core or memory region (e.g., R5FSS1_CORE0,
 *        ATCM), update the address translation macro in params initialization:
 *        - For R5FSS1 BTCM: Use CPU1_BTCM_SOCVIEW
 *        - For ATCM: Use CPU0_ATCM_SOCVIEW or CPU1_ATCM_SOCVIEW
 *
 * \par Channel Mapping (Load Share Mode):
 *      - **Motor 1**: Channels 0-2 (Phase U, V, W) on RTU PRU core
 *      - **Motor 2**: Channels 3-5 (Phase U, V, W) on PRU core
 *      - Each motor uses 3 channels for three-phase current sensing
 *
 * \par Clock Configuration:
 *      IEP sync out is used for SDFM sigma-delta modulator clock:
 *      - **Default Clock**: 20 MHz (IEP clock 300 MHz / 15)
 *      - **Period**: 15 IEP cycles (14 in register, 0-indexed)
 *      - **High Pulse Width**: 7 IEP cycles (6 in register, 0-indexed)
 *      - **Note**: To use different clock frequency, update highPulseWidth and
 *        periodTime calculations based on desired divider ratio
 *
 * \par Feature Limitations:
 *      This implementation is configured for specific dual motor use case. For additional
 *      features, refer to the SDFM example application (examples/current_sense/sdfm_example.c)
 *      which includes complete implementations of:
 *      - Double sampling (two samples per EPWM cycle)
 *      - Overcurrent comparator with thresholds
 *      - Fast detect for quick overcurrent detection
 *      - Zero-cross detection
 *      - Phase delay measurement
 *      - Different clock sources (IEP, SD CLK pins)
 *
 * \par Prerequisites:
 *      - EnDat encoder initialization should be done first (shares IEP resource)
 *      - SysConfig must define CONFIG_SDFM0 instance with proper channel configuration
 *
 * \param[in]  handle  HAL handle containing hardware peripheral handles
 *
 * \return None
 *
 * \note IEP counter is enabled only once. Assumption: EnDat configuration is done
 *       first, then SDFM configuration. Both drivers share the same IEP resource.
 *
 * \note All configuration must be completed before calling SDFM_enable(). Once
 *       SDFM_enable() is executed, PRU firmware starts sampling immediately.
 */
void HAL_setupSDFM(HAL_Handle handle)
{
    /* Local variable declarations */
    int32_t status = SystemP_FAILURE;
    SDFM_Params sdfmParams;
    const SDFM_Attrs *attrs;
    uint32_t ch, i, local_addr, global_addr;
    uint32_t highPulseWidth;
    uint32_t periodTime;
    uint32_t syncStartTime;

    /*PRU initialization*/
    /*Clear ICSS PRU data RAM and IRAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(SDFM_PRUICSS_SLICE));
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(SDFM_PRUICSS_SLICE));

    /*Reset the PRU cores */
#if defined (MOTOR1_INLINE_SDFM)
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if defined (MOTOR2_INLINE_SDFM)
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined (MOTOR1_INLINE_SDFM)
    /*Load SDFM firmware */
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(SDFM_PRUICSS_SLICE), 0, (uint32_t *) pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0));
    if(status == 0)
    {
        DebugP_log("\r\nERROR: PRUICSS_writeMemory failed for RTU PRU SDFM\n");
        goto deinit;
    }
    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: PRUICSS_enableCore failed for MOTOR1 SDFM\n");
        goto deinit;
    }
#endif

#if defined (MOTOR2_INLINE_SDFM)
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(SDFM_PRUICSS_SLICE), 0, (uint32_t *) pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0));
    if(status == 0)
    {
        DebugP_log("\r\nERROR: PRUICSS_writeMemory failed for PRU SDFM\n");
        goto deinit;
    }
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: PRUICSS_enableCore failed for MOTOR2 SDFM\n");
        goto deinit;
    }
#endif

    /*Initialize SDFM parameters structure */
    SDFM_paramsInit(&sdfmParams);
    sdfmParams.pruicss_handle = gPruIcssXHandle;
    sdfmParams.pwm_handle = NULL;
    local_addr = (uint32_t)&gSdfmSampleOutput;
    /*
     * Configure sample output buffer address translation (TCM local to SoC global view)
     *
     * The sample output buffer is allocated in R5F TCM (Tightly Coupled Memory):
     * - R5F uses core-local view address to access the buffer directly
     * - PRU firmware uses SoC global view address to write samples via ICSSG memory interface
     *
     * Address Translation Requirements:
     * - CPU0_BTCM_SOCVIEW: Used for R5FSS0_CORE0 (default for r5fss0-0_freertos)
     * - CPU1_BTCM_SOCVIEW: Required if running on R5FSS1_CORE0 (r5fss1-0_freertos)
     * - CPU0_ATCM_SOCVIEW: Required if buffer allocated in ATCM instead of BTCM
     *
     * The macro translates local TCM address (0x00000000-0x0007FFFF) to SoC view:
     * - R5FSS0 BTCM: 0x70000000-0x7007FFFF
     * - R5FSS1 BTCM: 0x70100000-0x7017FFFF
     *
     * \note Update the address translation macro if using different R5F core or memory region
     */
    global_addr = CPU0_BTCM_SOCVIEW(local_addr);
    sdfmParams.sample_base_addr = local_addr;

    /*Initialize SDFM driver with SysConfig index */
    gMotorSdfm = SDFM_init(CONFIG_SDFM0, &sdfmParams);
    if (gMotorSdfm == NULL)
    {
        DebugP_log("\rSDFM initialization failed\n");
        DebugP_log("\rexit %s due to failed initialization\n", __func__);
        goto deinit;
    }

    /* Get attrs */
    attrs = SDFM_getAttrs(gMotorSdfm);
    if (attrs == NULL)
    {
        DebugP_log("\rSDFM get attrs failed\n");
        DebugP_log("\rexit %s due to failed get attrs\n", __func__);
        goto deinit;
    }

    /* Enable all configured SDFM channels using attrs channel_mask */
    for(i = 0; i < SDFM_NUM_OF_CH_PER_PRU_SLICE; i++)
    {
        if(attrs->channel_mask & (1 << i))
        {
            status = SDFM_setEnableChannel(gMotorSdfm, i);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_setEnableChannel failed for channel %d\n", i);
                goto deinit;
            }
        }
    }

    /* Configure sample output buffer address */
    status = SDFM_setSampleOutputInterfaceGlobalAddr(gMotorSdfm, global_addr);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: SDFM_setSampleOutputInterfaceGlobalAddr failed\n");
        goto deinit;
    }

    /* Configure IEP count for one EPWM period */
    status = SDFM_configIepCount(gMotorSdfm, APP_EPWM_OUTPUT_FREQ);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: SDFM_configIepCount failed\n");
        goto deinit;
    }

    /* Configure operation mode (snoop/trigger) for each enabled PRU core using attrs */
    for(i = 0; i < NUM_OF_PRU_CORE_PER_PRU_SLICE; i++)
    {
        if(attrs->pru_core_mask & (1 << i))
        {
            if(attrs->pru_core_config[i].enable_snoop_mode)
            {
                status = SDFM_enableSnoopBasedNC(gMotorSdfm, i);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r\nERROR: SDFM_enableSnoopBasedNC failed for core %d\n", i);
                    goto deinit;
                }
            }

            if(attrs->pru_core_config[i].enable_trigger_mode == 1)
            {
                status = SDFM_setSampleTriggerTime(gMotorSdfm, attrs->pru_core_config[i].first_samp_trig_time, i);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r\nERROR: SDFM_setSampleTriggerTime failed for core %d\n", i);
                    goto deinit;
                }
            }
        }
    }

    /* Configure filter parameters for enabled channels using attrs */
    for(ch = 0; ch < NUM_OF_PRU_CORE_PER_PRU_SLICE; ch++)
    {
        if(attrs->channel_mask & (1 << ch))
        {
            /* Configure overcurrent comparator filter OSR from attrs */
            /* Configure overcurrent comparator filter OSR from attrs.
               Note: Overcurrent comparator filter is not used in this example,
               but hardware register need to be configured for snoop mode as SDFM_setFilterOverSamplingRatio does not configure regsiter for OSR
               because sampling is based on IEP compare timing.
               So, we need to configure here to start sampling for sdfm hardware accumulator. */
            status = SDFM_setCompFilterOverSamplingRatio(gMotorSdfm, ch, attrs->channels[ch].over_current_osr);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_setCompFilterOverSamplingRatio failed for channel %d\n", ch);
                goto deinit;
            }

            /* Configure normal current filter OSR */
            status = SDFM_setFilterOverSamplingRatio(gMotorSdfm, ch, attrs->channels[ch].normal_current_osr);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_setFilterOverSamplingRatio failed for channel %d\n", ch);
                goto deinit;
            }

            /* Configure data filter type */
            status = SDFM_configDataFilter(gMotorSdfm, ch, attrs->channels[ch].filter_type);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_configDataFilter failed for channel %d\n", ch);
                goto deinit;
            }

            /* Configure clock source */
            status = SDFM_selectClockSource(gMotorSdfm, ch, attrs->channels[ch].clk_source);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_selectClockSource failed for channel %d\n", ch);
                goto deinit;
            }

            /* Configure clock inversion */
            status = SDFM_setClockInversion(gMotorSdfm, ch, attrs->channels[ch].clk_inv);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_setClockInversion failed for channel %d\n", ch);
                goto deinit;
            }
        }
    }

    /* Enable EPWM sync */
    status = SDFM_enableEpwmSync(gMotorSdfm, attrs->epwm_sync_source);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: SDFM_enableEpwmSync failed\n");
        goto deinit;
    }

    /* Configure IEP sync mode for SDFM clock generation */
    /* IEP clock 300MHz, SD clk = 20MHz
       Div = 300/20 = 15, one period time = 15 IEP cycles, high pulse time = 7 IEP cycles  */
    highPulseWidth = 6; /*7 - 1*/
    periodTime = 14;  /* 15 - 1*/
    syncStartTime = 0; /*clock generation start time*/
    status = SDFM_configIepSyncMode(gMotorSdfm, highPulseWidth, periodTime, syncStartTime);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: SDFM_configIepSyncMode failed\n");
        goto deinit;
    }

    /* Enable IEP counter */
    /* \note For this example, make sure it is enabled by one time. */
    /* ASSUMPTION: Endat configuration is done first then SDFM configuration */
    status = SDFM_enableIep(gMotorSdfm);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nERROR: SDFM_enableIep failed\n");
        goto deinit;
    }

    /* Enable SDFM firmware on all enabled PRU cores to start sampling */
    for(i = 0; i < NUM_OF_PRU_CORE_PER_PRU_SLICE; i++)
    {
        if(attrs->pru_core_mask & (1 << i))
        {
            status = SDFM_enable(gMotorSdfm, i);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: SDFM_enable failed for core %d\n", i);
                goto deinit;
            }
        }
    }

    /*Store handle in HAL object */
    handle->sdfmHandle = gMotorSdfm;

    DebugP_log("SDFM initialization done!\r\n");
    gSdfmInitStatus = SystemP_SUCCESS;
    return;
deinit:
    DebugP_log("\rSDFM initialization failed!!!\r\n");
    gSdfmInitStatus = SystemP_FAILURE;
    if(gMotorSdfm != NULL)
    {
        SDFM_deinit(gMotorSdfm);
    }
    return;
}

 void HAL_readMtrSdfmData(HAL_sdfmData_t *pSdfmData, uint32_t motorNum)
{
    float32_t value;

    /* Read channel 0 or 3 based on motor number (Motor 1: Ch0-2, Motor 2: Ch3-5) */
    value = SDFM_getFilterData(gMotorSdfm, (motorNum == MTR_1) ? 0 : 3);
    pSdfmData->I_A.value[0] = value * pSdfmData->current_sf;

    /* Read channel 1 or 4 based on motor number */
    value = SDFM_getFilterData(gMotorSdfm, (motorNum == MTR_1) ? 1 : 4);
    pSdfmData->I_A.value[1] = value * pSdfmData->current_sf;

    /* Read channel 2 or 5 based on motor number */
    value = SDFM_getFilterData(gMotorSdfm, (motorNum == MTR_1) ? 2 : 5);
    pSdfmData->I_A.value[2] = value * pSdfmData->current_sf;

    pSdfmData->VdcBus_V = BP_AM2BLDCSERVO_VDC_BUS_VOLTAGE;

    return;
}  // end of HAL_readMtrSdfmData() function
#endif //MOTOR1_INLINE_SDFM || MOTOR2_INLINE_SDFM

// end of file
