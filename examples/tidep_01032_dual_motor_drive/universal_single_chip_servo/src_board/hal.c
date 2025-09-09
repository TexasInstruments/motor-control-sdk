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

/*SDFM*/
#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)
#include <current_sense/sdfm/include/sdfm_api.h>
#if (SDFM_PRUICSS_SLICEx == PRUICSS_PRU1)
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu1_bin.h>
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru1_bin.h>
#else
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu0_bin.h>
#include <current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru0_bin.h>
#endif // SDFM_PRUICSS_SLICEx



/*SDFM handle */
sdfm_handle gMotorSdfm;


/* Sdfm output samples, written by PRU cores */
__attribute__((section(".gSddfChSampsRaw"))) uint32_t gSdfm_sampleOutput[6] = { 0 };

#endif

/*ENDAT*/
#if defined (MOTOR1_ABS_ENC) || defined (MOTOR2_ABS_ENC)
#include <position_sense/endat/include/endat_drv.h>
#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_rtu_pru1_bin.h>
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_tx_pru1_bin.h>
#else
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_rtu_pru0_bin.h>
#include <position_sense/endat/firmware/multi_channel_load_share/endat_receiver_multi_tx_pru0_bin.h>
#endif // ENDAT_PRUICSS_SLICEx


extern PRUICSS_IntcInitData icss0_intc_initdata;

/* EnDat channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) struct endatChRxInfo gEndatChInfo;

/*EnDat handle*/
struct endat_priv *priv;

/*ENDAT PRU IRQ counter*/
uint32_t gEndatPruIrqCount = 0;
/*ENDAT Initialization Status*/
uint32_t gEndatInitStatus = 0;
/*ENDAT Position read failure counter*/
uint32_t gEndatPosReadFailCount = 0;

static uint8_t gEndat_is_multi_ch;
static uint8_t gEndat_multi_ch_mask;
static uint8_t  gEndat_is_load_share_mode;
static uint32_t gEndat_prop_delay[3] = {0};
static uint32_t gEndat_prop_delay_max = 0;

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)
#endif

/*EPWM*/

/* variables to hold EPWM base addresses */
uint32_t gEpwm0BaseAddr;
uint32_t gEpwm1BaseAddr;
uint32_t gEpwm2BaseAddr;
uint32_t gEpwm0BaseAddrB;

#endif
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

#if defined (BP_AM2BLDCSERVO)   
    /*initialize the ICSS PRU*/
#if defined (SOC_AM243X)
    HAL_pruIcssX_init();
#endif
    obj->encoderHandle = &priv;             // EnDat handle
    obj->sdfmHandle = &gMotorSdfm;
#endif // BP_AM2BLDCSERVO
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

#if defined(BP_AM2BLDCSERVO)
    /* Initialization of SDFM */
    HAL_setupSDFM(handle);
   /* Initialization of Encoders*/
    HAL_setupEncoder(handle);

#endif //BP_AM2BLDCSERVO
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
    }
    if(obj->motorNum == MTR_2)
    {
        gEpwm0BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM0_AXIS2_BASE_ADDR);
        gEpwm1BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM1_AXIS2_BASE_ADDR);
        gEpwm2BaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_AXIS2_BASE_ADDR);
        gEpwm0BaseAddrB = (uint32_t)AddrTranslateP_getLocalAddr(EPWM2_B_AXIS2_BASE_ADDR);
        /*FIXME, SYNC for all EPWM*/
//      SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);
//      /*EPWM3 and 6 sync in sel*/
//      CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM3_CTRL_SYNCIN_SEL_SHIFT));
//      CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL, (2 << CSL_MAIN_CTRL_MMR_CFG0_EPWM6_CTRL_SYNCIN_SEL_SHIFT));
//      SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, 1);
//      /* Time Sync Router input 29 (ICSSG1 IEP0 SYNC0) -> Time Sync Router output 39 (0x26 + 4 = 0x2A + Time Sync Router Base */
//      CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((39 * 4) + 4), (0x10000 | 39));
//      CSL_REG32_WR(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + ((40 * 4) + 4), (0x10000 | 39));
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
static void endat_process_host_command(int32_t cmd,
    struct cmd_supplement *cmd_supplement, struct endat_priv *priv)
{
    struct endat_clk_cfg clk_cfg;
    /* clock configuration */
    if(cmd == CLOCK_UPDATE)
    {
        clk_cfg.rx_div = ENDAT_RX_INPUT_CLOCK_FREQUENCY/(cmd_supplement->frequency * 8) - 1;
        clk_cfg.tx_div = ENDAT_TX_INPUT_CLOCK_FREQUENCY/(cmd_supplement->frequency) - 1;
        uint32_t rx_cnt;
        rx_cnt = ENDAT_DELAY_COUNTER_INCREMENT*(2*ICSS_PRU_CORE_CLOCK/cmd_supplement->frequency);
        if(rx_cnt % 5)
        {
            rx_cnt /= 5, rx_cnt += 1,  rx_cnt *= 5;
        }
        clk_cfg.rx_en_cnt = rx_cnt; /* rx arm >= 2 clock */
        clk_cfg.rx_div_attr = ENDAT_RX_SAMPLE_SIZE;

        endat_config_clock(priv, &clk_cfg);

        priv->rx_en_cnt = clk_cfg.rx_en_cnt;

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            uint16_t d;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    /*convert rx_en_cnt into ns */
                    float ct = ((priv->rx_en_cnt/ENDAT_DELAY_COUNTER_INCREMENT)*((float)1000000000/priv->pru_clock))/2; /*one endat clock cycle time = 1/endat frequency = 2*rx_en_cnt*/
                    /* if propagation delay is more than half clock cycle time (2/endat frequency) then we have to reduce clock cycles for rx*/
                    if(gEndat_prop_delay[priv->current_channel] > (ct/2))
                    {
                        uint16_t dis = floor(gEndat_prop_delay[priv->current_channel]/ct);
                        /* convert propagation delay into rx arm counts */
                        uint16_t temp = ((uint16_t)(((float)gEndat_prop_delay[priv->current_channel] * priv->pru_clock )/1000000000)) * ENDAT_DELAY_COUNTER_INCREMENT;
                        endat_config_rx_arm_cnt(priv, temp);
                        /* propagation delay/cycle_time */
                        endat_config_rx_clock_disable(priv, dis);
                    }
                    else
                    {
                        endat_config_rx_arm_cnt(priv, priv->rx_en_cnt);
                        endat_config_rx_clock_disable(priv, 0);
                    }
                
                    d = gEndat_prop_delay_max - gEndat_prop_delay[j];
                    endat_config_wire_delay(priv, d);
                }
            }
        }
        else
        {
            /*convert rx_en_cnt into ns */
            float ct = ((priv->rx_en_cnt/ENDAT_DELAY_COUNTER_INCREMENT)*((float)1000000000/priv->pru_clock))/2; /*one endat clock cycle time = 1/endat frequency = 2*rx_en_cnt*/
            /* if propagation delay is more than half clock cycle time (2/endat frequency) then we have to reduce clock cycles for rx*/
            if(gEndat_prop_delay[priv->current_channel] > (ct/2))
            {
                uint16_t dis = floor(gEndat_prop_delay[priv->current_channel]/ct);
                /* convert propagation delay into rx arm counts */
                uint16_t temp = ((uint16_t)(((float)gEndat_prop_delay[priv->current_channel] * priv->pru_clock )/1000000000)) * ENDAT_DELAY_COUNTER_INCREMENT;
                endat_config_rx_arm_cnt(priv, temp);
                /* propagation delay/cycle_time */
                endat_config_rx_clock_disable(priv, dis);
            }
            else
            {
                endat_config_rx_arm_cnt(priv, priv->rx_en_cnt);
                endat_config_rx_clock_disable(priv, 0);
            }
        }

        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->frequency = 2000; 
        }
        else
        {
            cmd_supplement->frequency = 0;
        }

        /* control loop */
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_process_host_command(CONFIG_TST_DELAY, cmd_supplement, priv);
                }
           }
        }
        else
        {
            endat_process_host_command(CONFIG_TST_DELAY, cmd_supplement, priv);
        }

    }
    else if(cmd == CONFIG_TST_DELAY)
    {
        uint32_t delay;
 
        /* convert tst delay from ns to tst counts*/
        delay = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->frequency * priv->pru_clock)/1000000000));
        if(delay % 5)
        {
            delay += 5, delay /= 5, delay *= 5;
        }

        if(delay <= (uint16_t)~0)
        {
            endat_config_tst_delay(priv, (uint16_t) delay);
        }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
   
}
void HAL_setupEncoder(HAL_Handle handle)
{
    /*EnDat Intruppt code need to be add */
    struct cmd_supplement cmd_supplement;

    uint64_t icssClk;
    uint32_t status = SystemP_FAILURE;

    void *pruicss_cfg;
    void *pruicss_iep;

    endat_clock_config endat_clk_config;

    gEndat_is_multi_ch = CONFIG_ENDAT0_MODE & 1;
    gEndat_is_load_share_mode = CONFIG_ENDAT0_MODE & 2;

    /* PRU ICSS configuration */
    /*Set in constant table C29 for  tx pru*/
#if ENDAT_PRUICSSx == 1
#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C29, 0xA58);    
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C29, 0xA50);
#endif
#else
#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif
#endif
    /* clear ICSS PRU data RAM and IRAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(ENDAT_PRUICSS_SLICEx));
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(ENDAT_PRUICSS_SLICEx));

/*C16 pin High for Enabling ch0 in booster pack */
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL0)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif


    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        gEndat_multi_ch_mask=(CONFIG_ENDAT0_CHANNEL0<<0|CONFIG_ENDAT0_CHANNEL1<<1|CONFIG_ENDAT0_CHANNEL2<<2);
        if(!gEndat_multi_ch_mask)
        {
            DebugP_log("\r\nERROR: Please select multi-channel configuration -\n\n");
            DebugP_log("\rexit %s\n",
                          __func__);
            return;
        }
    }
    else
    {
        int i;
        i = CONFIG_ENDAT0_CHANNEL0 & 0;
        i += CONFIG_ENDAT0_CHANNEL1;
        i += CONFIG_ENDAT0_CHANNEL2<<1;
        if(i < 0 || i > 2)
        {
           DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
           i = 0;
        }
    }

    /*Translate the TCM local view addr to globel view addr */
    uint64_t gEndatChInfoGlobalAddr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndatChInfo);


    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    pruicss_iep  = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);

    icssClk = ICSS_PRU_CORE_CLOCK;

    /*3 channel pheripheral clock configuration*/
    endat_clk_config.pru_clock = icssClk;
    endat_clk_config.pru_uart_clock = ENDAT_INPUT_CLOCK_UART_FREQUENCY;
    endat_clk_config.rx_clock_source = ENDAT_RX_FIFO_CLOCK_SOURCE;
    endat_clk_config. tx_clock_source = ENDAT_TX_FIFO_CLOCK_SOURCE;


#if (ENDAT_PRUICSS_SLICEx == PRUICSS_PRU1)
    priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru1DramBase, &gEndatChInfo, gEndatChInfoGlobalAddr, pruicss_cfg, pruicss_iep, ENDAT_PRUICSS_SLICEx, &endat_clk_config);

#else
    priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru0DramBase, &gEndatChInfo, gEndatChInfoGlobalAddr,  pruicss_cfg, pruicss_iep, ENDAT_PRUICSS_SLICEx, &endat_clk_config);
#endif

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        endat_config_multi_channel_mask(priv, gEndat_multi_ch_mask, gEndat_is_load_share_mode);
    }
    else
    {
    endat_config_channel(priv, MOTOR1_ENDAT_ENABLE_CHANNEL);
    }

    endat_config_host_trigger(priv);

    /* Configure Delays based on the ICSSG frequency*/
    /* Count = ((required delay * icssClk)/1000) */
    priv->pruss_xchg->endat_delay_125ns = ((icssClk*125)/1000000000);
    priv->pruss_xchg->endat_delay_51us = ((icssClk*51)/1000000 );
    priv->pruss_xchg->endat_delay_5us = ((icssClk*5)/1000000);
    priv->pruss_xchg->endat_delay_1ms = ((icssClk/1000) * 1);
    priv->pruss_xchg->endat_delay_2ms = ((icssClk/1000) * 2);
    priv->pruss_xchg->endat_delay_12ms = ((icssClk/1000) * 12);
    priv->pruss_xchg->endat_delay_50ms = ((icssClk/1000) * 50);
    priv->pruss_xchg->endat_delay_380ms = ((icssClk/1000) * 380);
    priv->pruss_xchg->endat_delay_900ms = ((icssClk/1000) * 900);
    priv->pruss_xchg->icssg_clk = icssClk;

    /*Load the EnDat firmware*/
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);


    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(ENDAT_PRUICSS_SLICEx), 0, (uint32_t *) EnDatFirmwareMultiMakeRTU_0, sizeof(EnDatFirmwareMultiMakeRTU_0));
    DebugP_assert(0 != status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(ENDAT_PRUICSS_SLICEx), 0, (uint32_t *) EnDatFirmwareMultiMakeTXPRU_0, sizeof(EnDatFirmwareMultiMakeTXPRU_0));
    DebugP_assert(0 != status);

    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR1_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR2_ENDAT_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);

    /* check initialization ack from firmware, with a timeout of 5 second */
    status = endat_wait_initialization(priv, ENDAT_WAIT_5_SECOND, gEndat_multi_ch_mask);

    if(status < 0)
    {
        DebugP_log("\r\t Check whether encoder is connected properly \n");
        
        gEndatInitStatus = SystemP_FAILURE;
        return;
    }
    else
    {
        gEndatInitStatus = 1;
    }

    /* read encoder info at low frequency (200KHz) so that cable length won't affect */
    cmd_supplement.frequency = 200 * 1000;
    endat_process_host_command(CLOCK_UPDATE, &cmd_supplement, priv);
    

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        int32_t j;

        for(j = 0; j < 3; j++)
        {
            if(gEndat_multi_ch_mask & 1 << j)
            {
                endat_multi_channel_set_cur(priv, j);
                /*Initialization of RT parameters*/
                endat_init_rt_measurement(priv);
                if(endat_get_encoder_info(priv) < 0)
                {
                    DebugP_log("\rEnDat initialization channel %d failed\n", j);
                    DebugP_log("\rexit %s due to failed initialization\n", __func__);
                    return;
                }
                /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
                gEndat_prop_delay[priv->current_channel] = endat_get_prop_delay(priv)*((float)(1000000000)/icssClk);
                DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
            }
        }

        gEndat_prop_delay_max = gEndat_prop_delay[0] > gEndat_prop_delay[1] ?
                               gEndat_prop_delay[0] : gEndat_prop_delay[1];
        gEndat_prop_delay_max = gEndat_prop_delay_max > gEndat_prop_delay[2] ?
                               gEndat_prop_delay_max : gEndat_prop_delay[2];
    }
    else
    {
        /*Initialization of RT parameters*/
        endat_init_rt_measurement(priv);
        if(endat_get_encoder_info(priv) < 0)
        {
            DebugP_log("\rEnDat initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }
        /*convert cnt to time in ns ((cnt*1000000000)/icssClk) before use*/
        gEndat_prop_delay[priv->current_channel] = endat_get_prop_delay(priv)*((float)(1000000000)/icssClk);

    }

    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders*/
    if(priv->cmd_set_2_2)
    {
        cmd_supplement.frequency = 8 * 1000 * 1000;
    }
    else
    {
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }

    endat_process_host_command(CLOCK_UPDATE, &cmd_supplement, priv);

    uint64_t cmp3 = 800;
    uint64_t cmp4 = 800;
    uint32_t cmp_reg0, cmp_reg1;
    uint16_t event, event_clear;


    /* Configure IEP for peridoc mode  */
    event = HW_RD_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG);

    event |= (0x1 << 4 );
    event_clear |= (0x1 << 3);

    /*CH2*/
    event |= (0x1 << 7 );
    event_clear |= (0x1 << 6);

    cmp_reg0 = (cmp3 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (cmp3>>32 & 0xffffffff);
    HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0,  cmp_reg0);
    HW_WR_REG32(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1,  cmp_reg1);


    cmp_reg0 = (cmp4 & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (cmp4>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG0,  cmp_reg0);
    HW_WR_REG32((uint8_t*)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP6_REG1,  cmp_reg1);

    /*clear event*/
    HW_WR_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_STATUS_REG, event_clear);
    /*enable  event*/
    HW_WR_REG8(pruicss_iep + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG, event);
   
    for(int i=0; i<3;i++)
    {
        if(gEndat_multi_ch_mask & 1 << i)
        {
            endat_multi_channel_set_cur(priv, i);
        
            DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", i);
            DebugP_log("Encoder Channel Init Completed!!!\n");
        }
    }

    endat_config_periodic_trigger(priv);
    DebugP_assert(endat_command_process(priv, 8, NULL) >= 0);

    handle->encoderHandle = &priv;
    return;

}

void HAL_getMtrEncoderPosition(ENC_Handle handle, uint32_t motorNum)
{
    uint32_t pos, rev;
    ENC_Obj *obj = (ENC_Obj *)handle;

    /* Check CRC from the EnDat PRU */
    if(motorNum == MTR_1)
    {

        uint32_t status = gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].crcStatus;
        status = status & ENDAT_CRC_DATA;
        if(!(gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].crcStatus & ENDAT_CRC_DATA))
        {
            gEndatPosReadFailCount++;  
            return;
        }
        else 
        {
            /*FIXME add code to clear CRC status here*/
        }
        pos = gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].posWord0;
        rev = gEndatChInfo.ch[MOTOR1_ENDAT_ENABLE_CHANNEL].posWord1;
    }
    else if(motorNum == MTR_2)
    {
        uint32_t status = gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].crcStatus;
        status = status & ENDAT_CRC_DATA;
        if(!(gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].crcStatus & ENDAT_CRC_DATA))
        {
            gEndatPosReadFailCount++;
            return;
        }
        else
        {
            /*FIXME add code to clear CRC status here*/
        }
        pos = gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].posWord0;
        rev = gEndatChInfo.ch[MOTOR2_ENDAT_ENABLE_CHANNEL].posWord1;
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
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_ENABLE_SA_MUX_MODE);
#endif

    /*Enable PRU Interrupt Controller*/
    uint32_t status;
    status = PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
    DebugP_assert(SystemP_SUCCESS == status);

}
#endif

#if defined (MOTOR1_INLINE_SDFM) || defined (MOTOR2_INLINE_SDFM)
void HAL_setupSDFM(HAL_Handle handle)
{
    /*SDFM intruppt code need to add*/
    uint32_t status = SystemP_FAILURE;

    /*PRU intialization*/
    /*Clear ICSS PRU data RAM and IRAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(SDFM_PRUICSS_SLICEx));
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(SDFM_PRUICSS_SLICEx));

    /*Reset the PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_disableCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_resetCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);

    /*Load SDFM firmware */
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(SDFM_PRUICSS_SLICEx), 0, (uint32_t *) pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0));
    DebugP_assert(0 != status);
    status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(SDFM_PRUICSS_SLICEx), 0, (uint32_t *) pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0));
    DebugP_assert(0 != status);
    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR1_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, MOTOR2_SDFM_PRUICSS_CORE);
    DebugP_assert(SystemP_SUCCESS == status);

    /*SDFM Parameters configuration */
    for (int i = 1; i >= 0; i--)
    {

        if(i == 1)
        {
            /*SDFM instance for motor2*/
            gMotorSdfm = SDFM_init(gPruIcssXHandle, SDFM_PRUICSS_SLICEx, MOTOR2_SDFM_PRUICSS_CORE);
        }
        else
        {
            /*SDFM instance for motor1*/
            gMotorSdfm = SDFM_init(gPruIcssXHandle, SDFM_PRUICSS_SLICEx, MOTOR1_SDFM_PRUICSS_CORE);
        }
            /*SDFM instance for motor1*/
        if (gMotorSdfm == NULL)
        {
            DebugP_log("\rSDFM initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }

        gMotorSdfm->pruicssCfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
        /*IEP base address*/
        gMotorSdfm->pruicssIep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);

        /*enable load share mode*/
        if(i == 1)
        {
            SDFM_enableLoadShareMode(gMotorSdfm, SDFM_PRUICSS_SLICEx);
        }
    
        for(int j = 0; j<NUM_CH_SUPPORTED_PER_AXIS; j++)
        {
            if(i == 0)
            {
                SDFM_setEnableChannel(gMotorSdfm, j);
            }
            else
            {
                SDFM_setEnableChannel(gMotorSdfm, j + 3);
            }
        }

        gMotorSdfm->pruCoreClk = ICSS_PRU_CORE_CLOCK;
        gMotorSdfm->iepClock = ICSS_PRU_IEP_CLOCK;
        gMotorSdfm->sdfmClock = SDFM_MCLK_VALUE;
        uint32_t sampleOutputInterfaceGlobalAddr;
        if(i == 0)
        {
            gMotorSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput);
            sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW((uint32_t)&gSdfm_sampleOutput);

        }
        else
        {
            gMotorSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)((uint32_t)(&gSdfm_sampleOutput) + 12);
            sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW((uint32_t)(&gSdfm_sampleOutput) + 12);

        }

        gMotorSdfm->pSdfmInterface->sampleBufferBaseAdd = sampleOutputInterfaceGlobalAddr;
        if(i == 1)
        {
            gMotorSdfm->iepInc = 1; /* Default IEP increment 1 */
            /* Configure IEP sync mode for MCLK*/
            /* IEP clock 300MHz, SD clk = 20Mhz
            Div = 300/20 = 15, one period time = 15 IEP cycles, high plus time = 7 IEP cycles  */
            uint32_t highPulseWidth = 6; /*7 - 1*/
            uint32_t periodTime = 14;  /* 15 - 1*/
            uint32_t syncStartTime = 0; /*clock generation start time.*/
            SDFM_configIepSyncMode(gMotorSdfm, highPulseWidth, periodTime, syncStartTime);
            SDFM_enableIep(gMotorSdfm);
        }
        /*configure IEP count for one epwm period*/
        SDFM_configIepCount(gMotorSdfm, APP_EPWM_OUTPUT_FREQ); //3000

        SDFM_enableSnoopBasedNC(gMotorSdfm);
        /*configuration of sdfm parameters which are supported per axis, not for invidual channels.
        Channel0 perametrs value are used for all 3 channels of axis*/
        /*set Noraml current OSR */
        SDFM_setFilterOverSamplingRatio(gMotorSdfm, 0, SDFM_NC_OSR_VALUE);
        SDFM_setFilterOverSamplingRatio(gMotorSdfm, 1, SDFM_NC_OSR_VALUE);
        SDFM_setFilterOverSamplingRatio(gMotorSdfm, 2, SDFM_NC_OSR_VALUE);
        /*set first sample trigger time*/
        SDFM_setSampleTriggerTime(gMotorSdfm, SDFM_NORMAL_CURRENT_TRIGGER_POINT);
        /*enable epwm sync*/
        if(i == 1)
        {
            SDFM_enableEpwmSync(gMotorSdfm, SDFM_EPWM_SYNC_SOURCE);
        }
        
        /*below configuration for all three channel Ch0, Ch1, Ch2*/
        /*set comparator osr or Over current osr*/
        SDFM_setCompFilterOverSamplingRatio(gMotorSdfm, 0, SDFM_NC_OSR_VALUE);
        SDFM_setCompFilterOverSamplingRatio(gMotorSdfm, 1, SDFM_NC_OSR_VALUE);
        SDFM_setCompFilterOverSamplingRatio(gMotorSdfm, 2, SDFM_NC_OSR_VALUE);
       /*set ACC source or filter type*/
        SDFM_configDataFilter(gMotorSdfm, 0, CONFIG_SDFM0_CHANNEL0_ACC_SOURCE);
        SDFM_configDataFilter(gMotorSdfm, 1, CONFIG_SDFM0_CHANNEL1_ACC_SOURCE);
        SDFM_configDataFilter(gMotorSdfm, 2, CONFIG_SDFM0_CHANNEL2_ACC_SOURCE);
        /*set clock source for all three channel*/
        SDFM_selectClockSource(gMotorSdfm, 0, CONFIG_SDFM0_CHANNEL0_CLK_SOURCE);
        SDFM_selectClockSource(gMotorSdfm, 1, CONFIG_SDFM0_CHANNEL1_CLK_SOURCE);
        SDFM_selectClockSource(gMotorSdfm, 2, CONFIG_SDFM0_CHANNEL2_CLK_SOURCE);

        SDFM_configDataFilter(gMotorSdfm, 0, CONFIG_SDFM0_CHANNEL3_ACC_SOURCE);
        SDFM_configDataFilter(gMotorSdfm, 1, CONFIG_SDFM0_CHANNEL4_ACC_SOURCE);
        SDFM_configDataFilter(gMotorSdfm, 2, CONFIG_SDFM0_CHANNEL5_ACC_SOURCE);
        /*set clock source for all three channel*/
        SDFM_selectClockSource(gMotorSdfm, 0, CONFIG_SDFM0_CHANNEL3_CLK_SOURCE);
        SDFM_selectClockSource(gMotorSdfm, 1, CONFIG_SDFM0_CHANNEL4_CLK_SOURCE);
        SDFM_selectClockSource(gMotorSdfm, 2, CONFIG_SDFM0_CHANNEL5_CLK_SOURCE);

        /* Enable (global) SDFM */
        SDFM_enable(gMotorSdfm);

    }
    handle->sdfmHandle = &gMotorSdfm;
    return;
}

 void HAL_readMtrSdfmData(HAL_sdfmData_t *pSdfmData, uint32_t motorNum)
{
    float32_t value;
    if(motorNum == MTR_1)
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput);
        value =  SDFM_getFilterData(gMotorSdfm, 0);
    }
    else
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput + 12);
        value =  SDFM_getFilterData(gMotorSdfm, 0);
    }
    pSdfmData->I_A.value[0] = (value ) * pSdfmData->current_sf;
    
    if (motorNum == MTR_1)
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput);
        value =  SDFM_getFilterData(gMotorSdfm, 1);
    }
    else
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput + 12);
        value =  SDFM_getFilterData(gMotorSdfm, 1);
    }
    pSdfmData->I_A.value[1] = (value) * pSdfmData->current_sf;

    if(motorNum == MTR_1)
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput);
        value =  SDFM_getFilterData(gMotorSdfm, 2);
    }
    else
    {
        gMotorSdfm->sampleOutputInterface =  (SDFM_SampleOutInterface *)((uint32_t)&gSdfm_sampleOutput + 12);
        value =  SDFM_getFilterData(gMotorSdfm, 2);

    }
    pSdfmData->I_A.value[2] = (value) * pSdfmData->current_sf;

    pSdfmData->VdcBus_V = BP_AM2BLDCSERVO_VDC_BUS_VOLTAGE;
    return;
}  // end of HAL_readMtr1SdfmData() functions
#endif //MOTOR1_INLINE_SDFM || MOTOR2_INLINE_SDFM

// end of file
