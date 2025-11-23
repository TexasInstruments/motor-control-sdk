/*
 *  Copyright (C) 2023-25 Texas Instruments Incorporated
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

/**
 *  \file   sdfm_example.c
 *
 *  \brief  SDFM example initialization and configuration functions.
 *
 *  \details
 *  This file provides helper functions for SDFM (Sigma-Delta Filter Module)
 *  initialization on ICSSG PRU cores for current sensing applications.
 *
 *  Architecture:
 *  - PRU firmware runs real-time SDFM filtering on PRU-ICSS cores
 *  - ARM R5F manages initialization, configuration, and data processing
 *  - Supports single PRU mode (9 channels on one core) or load-share mode
 *    (3 channels each on RTU_PRU, PRU, and TX_PRU cores)
 *
 *  Key Functions:
 *  - SDFM_pruIcssInit()       : Initialize ICSSG subsystem 
 *  - initPruSdfm()            : Load firmware and configure SDFM 
 *  - initSdfmFw()             : Internal firmware configuration 
 *  - SDFM_configGpioPins()    : Internal GPIO setup for zero-cross 

 *  Firmware Loading:
 *  - Single PRU mode: Loads SDFM_PRU0/1_image_0 to PRU core
 *  - Load-share mode: Loads separate firmware to RTU_PRU, PRU, and TX_PRU
 *  - Firmware binaries are statically linked from firmware/ directory
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <stdint.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/pruicss.h>
#include <drivers/sciclient.h>

#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_txpru1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru1_bin.h"            /* SDFM image data */
#else
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_txpru0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru0_bin.h"            /* SDFM image data */
#endif

#include "sdfm_example.h"
#include "current_sense/sdfm/include/sdfm_api.h"

#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
#define SDFM_PRU_CORE         PRUICSS_PRU1
#define SDFM_RTU_CORE         PRUICSS_RTU_PRU1
#define SDFM_TXPRU_CORE       PRUICSS_TX_PRU1
#else
#define SDFM_PRU_CORE         PRUICSS_PRU0
#define SDFM_RTU_CORE         PRUICSS_RTU_PRU0
#define SDFM_TXPRU_CORE       PRUICSS_TX_PRU0
#endif

/* Number of PRU images */
#define PRU_SDFM_NUM_PRU_IMAGE  ( 4 )

/* ICSS INTC configuration */
#if CONFIG_SDFM0_ICSSGx == 1
/* These variables are defined in the generated SysCfg code */
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* PRU SDFM FW image info */
typedef struct PRUSDFM_PruFwImageInfo_s 
{
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDFM_PruFwImageInfo;

/* PRU SDFM image info */
static PRUSDFM_PruFwImageInfo gPruFwImageInfo[PRU_SDFM_NUM_PRU_IMAGE] =
{
#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
    {SDFM_PRU1_image_0, sizeof(SDFM_PRU1_image_0)}, /* single PRU FW binary */
    {pru_SDFM_PRU1_image_0, sizeof(pru_SDFM_PRU1_image_0)}, /* load share PRU FW binary */
    {pru_SDFM_RTU1_image_0, sizeof(pru_SDFM_RTU1_image_0)}, /*load share RTU FW binary */
    {pru_SDFM_TXPRU1_image_0, sizeof(pru_SDFM_TXPRU1_image_0)} /*load share TXPRU binary*/ 
#else
    {SDFM_PRU0_image_0, sizeof(SDFM_PRU0_image_0)}, /* single PRU FW binary */
    {pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0)}, /* load share PRU FW binary */
    {pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0)}, /*load share RTU FW binary */
    {pru_SDFM_TXPRU0_image_0, sizeof(pru_SDFM_TXPRU0_image_0)} /*load share TXPRU binary*/ 
#endif
};


/*
 *  ======== initIcss ========
 */
/* Initialize ICSSG */
int32_t SDFM_pruIcssInit(
    uint8_t icssInstId,
    uint8_t sliceId,
    uint8_t saMuxMode,
    uint8_t loadShareMode,
    PRUICSS_Handle *pPruIcssHandle
)
{
    PRUICSS_Handle pruIcssHandle;
    int32_t size;
    int32_t status;

    /* Open ICSS PRU instance */
    pruIcssHandle = PRUICSS_open(icssInstId);
    if (pruIcssHandle == NULL) {
        return SDFM_ERR_INIT_ICSSG;
    }

    /* Disable slice PRU cores */
    if (sliceId == PRUICSS_PRU0)
    {
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU0);
        if (status != SystemP_SUCCESS)
        {
            return SDFM_ERR_INIT_ICSSG;
        }

        if(loadShareMode)
        {
            status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_RTU_PRU0);
            if (status != SystemP_SUCCESS) 
            {
                return SDFM_ERR_INIT_ICSSG;
            }

            status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_TX_PRU0);
            if (status != SystemP_SUCCESS) 
            {
                return SDFM_ERR_INIT_ICSSG;
            }

        }
    }
    else if (sliceId == PRUICSS_PRU1)
    {
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU1);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_ICSSG;
        }

        if(loadShareMode)
        {
            status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_RTU_PRU1);
            if (status != SystemP_SUCCESS) 
            {
                return SDFM_ERR_INIT_ICSSG;
            }

            status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_TX_PRU1);
            if (status != SystemP_SUCCESS) 
            {
                return SDFM_ERR_INIT_ICSSG;
            }

        }
    }
    else
    {
        return SDFM_ERR_INIT_ICSSG;
    }

    /* Reset slice memories */
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_PRU(sliceId));
    if (size == 0)
    {
        return SDFM_ERR_INIT_ICSSG;
    }
    if(loadShareMode)
    {
        size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_RTU_PRU(sliceId));
        if (size == 0)
        {
            return SDFM_ERR_INIT_ICSSG;
        }
        size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_TX_PRU(sliceId));
        if (size == 0)
        {
            return SDFM_ERR_INIT_ICSSG;
        }
    }
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_DATARAM(sliceId));
    if (size == 0)
    {
        return SDFM_ERR_INIT_ICSSG;
    }

    /* Set ICSS pin mux */
#ifdef CONFIG_SDFM0_G_MUX_EN
    PRUICSS_setSaMuxMode(pruIcssHandle, saMuxMode);
#endif
    /* Initialize ICSS INTC */
#if CONFIG_SDFM0_ICSSGx == 1
    status = PRUICSS_intcInit(pruIcssHandle, &icss1_intc_initdata);
#else
    status = PRUICSS_intcInit(pruIcssHandle, &icss0_intc_initdata);
#endif
    if (status != SystemP_SUCCESS) {
        return SDFM_ERR_INIT_ICSSG;
    }

    *pPruIcssHandle = pruIcssHandle;

    return SDFM_ERR_NERR;
}
/*
 *  ======== SDFM_configGpioPins ========
 *  Internal helper function to configure GPIO pins for zero-cross detection.
 *  Called from initSdfmFw() for each enabled channel with zero-cross enabled.
 */
static void SDFM_configGpioPins(SDFM_Handle h_sdfm, uint8_t channel )
{
    switch (channel )
    {
        case 0:
#if (CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH0_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH0_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH0_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 1:
#if (CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH1_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH1_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH1_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;      
        case 2:
#if (CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH2_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH2_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH2_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 3:
#if (CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH3_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH3_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH3_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 4:
#if (CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH4_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH4_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH4_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 5:
#if (CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH5_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH5_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH5_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;  
        case 6:
#if (CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH6_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH6_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH6_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 7:
#if (CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH7_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH7_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH7_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 8:
#if (CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH8_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH8_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH8_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, channel, gpioBaseAddr, pinNum);
            }
#endif
            break;
        default:
            break;
    }
}

/*
 *  ======== initSdfmFw ========
 *  Internal function to initialize SDFM firmware and configure channels.
 *  Called from initPruSdfm() after PRU cores are loaded and running.
 *
 *  Configures:
 *  - Channel enable/disable
 *  - Filter parameters (OSR, filter type)
 *  - Clock source and inversion
 *  - Comparator thresholds (over-current, zero-cross)
 *  - Fast detect parameters
 *  - IEP configuration for trigger/snoop modes
 *  - EPWM synchronization
 *  - Phase delay compensation
 */
static int32_t initSdfmFw(SDFM_Params sdfm_params, SDFM_Handle *pHSdfm)
{
    SDFM_Handle hSdfm;
    uint8_t channel ;    

    /* Initialize SDFM instance */
    hSdfm = SDFM_init(CONFIG_SDFM0, sdfm_params);
    
    if (hSdfm == NULL)
    {
        return SDFM_ERR_INIT_SDFM;
    }

    for(int8_t i = 0; i< 9; i++)
    {
        if(sdfm_params.enable_channel_mask & (1 << i))
        {
            SDFM_setEnableChannel(hSdfm, i);
        }
    }
    uint32_t i;
    i = SDFM_getFirmwareVersion(hSdfm);
    DebugP_log("\n\n\n");
    DebugP_log("SDFM firmware version \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    /* Configure SDFM sample output interface */
    hSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)(sdfm_params.sample_base_addr);
    uint32_t sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW(sdfm_params.sample_base_addr);
    SDFM_setSampleOutputInterfaceGlobalAddr(hSdfm, sampleOutputInterfaceGlobalAddr);
    
#if (CONFIG_SDFM0_CLK_FROM_IEP != 0)
    /* IEP clock 300MHz, SD clk = 20Mhz
      Div = 300/20 = 15, one period time = 15 IEP cycles, high plus time = 7 IEP cycles  */
    uint32_t highPulseWidth = 6; /*7 - 1*/
    uint32_t periodTime = 14;  /* 15 - 1*/
    uint32_t syncStartTime = 0; /*clock generation start time.*/
    SDFM_configIepSyncMode(hSdfm, highPulseWidth, periodTime, syncStartTime);
    SDFM_enableIep(hSdfm);   
    hSdfm->clk_config.clock_source = SDFM_CLOCK_SOURCE_IEP;
    hSdfm->clk_config.sdfm_clock_value = CONFIG_SDFM0_CLOCK_VALUE;
#endif
     
    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
#if (CONFIG_SDFM0_CLK_FROM_ECAP != 0)
    uint8_t ecap_divider = 0x0F; /*PRU clock at 300MHz: SD clock = 300/15=20Mhz*/
    SDFM_configEcap(hSdfm, ecap_divider);
    hSdfm->clk_config.clock_source = SDFM_CLOCK_SOURCE_ECAP;
    hSdfm->clk_config.sdfm_clock_value = CONFIG_SDFM0_CLOCK_VALUE;
#endif
    
    /*SD clk configuration from GPO1 */
#if (CONFIG_SDFM0_CLK_FROM_GPIO1 != 0)
   /*Setting divisor values for 20MHz, @300Mhz PRU core. two divisors 15 and 1.
    15*1 = 300/20
    PRU0_GPO_DIV0 = 1Ch when divisor value 15
    PRU0_GPO_DIV1 = 0h when divisor value 1
    */
   uint8_t div0 = 0x1C;
   uint8_t div1 = 0x0;

   SDFM_configClockFromGPO1(hSdfm, div0, div1);
   hSdfm->clk_config.clock_source = SDFM_CLOCK_SOURCE_PRUGPIO1;
   hSdfm->clk_config.sdfm_clock_value = CONFIG_SDFM0_CLOCK_VALUE;
#endif

    
           
   /*Add code to confgure common configuration for all channels*/
   if(sdfm_params.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode || sdfm_params.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_snoop_mode || sdfm_params.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_snoop_mode)
   {
        /*configure IEP count for one epwm period*/
        SDFM_configIepCount(hSdfm, sdfm_params.iep_reset_freq);
   }

   for(int8_t i = 0; i< 3; i++)
   {
        if(sdfm_params.enable_pru_core_mask & (1 << i))
        {

            if(sdfm_params.pru_core_config[i].enable_snoop_mode)
            {
                SDFM_enableSnoopBasedNC(hSdfm, i);
            }
            if(sdfm_params.pru_core_config[i].enable_trigger_mode == 1)
            {
                SDFM_enableTriggerModeForNormalCurrent(hSdfm, i);
                SDFM_setSampleTriggerTime(hSdfm, sdfm_params.pru_core_config[i].first_samp_trig_time, i);
                if(sdfm_params.pru_core_config[i].en_double_nc_sampling)
                {
                    SDFM_enableDoubleSampling(hSdfm, sdfm_params.pru_core_config[i].second_samp_trig_time, i);
                }
                else
                {
                    SDFM_disableDoubleSampling(hSdfm, i);
                }

                SDFM_selectIepCmpEvent(hSdfm, sdfm_params.pru_core_config[i].iep_cmp_event, i);
            }
        }
   }
   
    /*enable epwm sync*/
    if(sdfm_params.enable_epwm_sync)
    {
        SDFM_enableEpwmSync(hSdfm, sdfm_params.epwm_sync_source);
        SDFM_enableIep(hSdfm);
    }
    else
    {
        if(sdfm_params.pru_core_config[0].enable_trigger_mode == 1|| sdfm_params.pru_core_config[1].enable_trigger_mode == 1|| sdfm_params.pru_core_config[2].enable_trigger_mode == 1)
        {
            SDFM_configIepCmp0ToResetIep(hSdfm, sdfm_params.iep_reset_freq);
            SDFM_enableIep(hSdfm);
        }
    }

    /*Phase delay calculation for ch0. With single PRU and no load share*/
    if(sdfm_params.enable_phase_delay)
    {
        if(sdfm_params.enable_channel_mask & (1 << 0))
        {
            SDFM_measureClockPhaseDelay(hSdfm, sdfm_params.channels[0].clk_inv, 0);
        }
    }

    /*below configuration for all three channel*/
    for(channel  = 0; channel  < 9; channel ++)
    {
        if(sdfm_params.enable_channel_mask & (1 << channel))
        {
            SDFM_setCompFilterOverSamplingRatio(hSdfm, channel , sdfm_params.channels[channel].over_current_osr);

            SDFM_setFilterOverSamplingRatio(hSdfm, channel , sdfm_params.channels[channel].normal_current_osr);

            /*set ACC source or filter type*/
            SDFM_configDataFilter(hSdfm, channel, sdfm_params.channels[channel].filter_type);

            /*set clock inversion & clock source for all three channel*/
            SDFM_selectClockSource(hSdfm, channel, sdfm_params.channels[channel].clk_source);

            /*set clock inversion*/
            SDFM_setClockInversion(hSdfm, channel, sdfm_params.channels[channel].clk_inv);

            if(sdfm_params.channels[channel].enable_comparator == 1)
            {
                SDFM_enableComparator(hSdfm, channel);
                /*set high and low thresholds value */
                uint32_t comThresholds[2];
                comThresholds[0] = sdfm_params.channels[channel].high_threshold;
                comThresholds[1] = sdfm_params.channels[channel].low_threshold;   
                SDFM_setCompFilterThresholds(hSdfm, channel, comThresholds);
            }

            if(sdfm_params.channels[channel].fd_enable == 1)
            {
                /*Fast detect configuration */
                uint8_t fdFields[NUM_FD_FIELDS];
                fdFields[0] = sdfm_params.channels[channel].fd_enable;
                fdFields[1] = sdfm_params.channels[channel].fd_window;
                fdFields[2] = sdfm_params.channels[channel].fd_zero_max;
                fdFields[3] = sdfm_params.channels[channel].fd_zero_min;
                SDFM_configFastDetect(hSdfm, channel, fdFields);
            }

            if(sdfm_params.channels[channel].en_zero_cross == 1)
            {
                /*zero cross configuration*/
                SDFM_enableZeroCrossDetection(hSdfm, channel, sdfm_params.channels[channel].zero_cross_threshold);
                /*GPIO pin configuration for zero cross*/
                SDFM_configGpioPins(hSdfm, channel);
            }
        }

    }
    
    /* Enable (global) SDFM */
    for(int8_t i = 0; i< 3; i++)
    {
        if(sdfm_params.enable_pru_core_mask & (1 << i))
        {
            SDFM_enable(hSdfm, i);
        }
    }
    *pHSdfm = hSdfm;

 return SDFM_ERR_NERR;
}
/*
 *  ======== initPruSdfm ========
 */
/* Initialize PRU core for SDFM */
int32_t initPruSdfm(
    PRUICSS_Handle pruIcssHandle,
    SDFM_Params pSdfmPrms,
    SDFM_Handle *pHSdfm
)
{
    uint32_t pruIMem;
    PRUSDFM_PruFwImageInfo *pPruFwImageInfo;
    int32_t size;
    const uint32_t *sourceMem;          /* Source memory[ Array of uint32_t ] */
    uint32_t imemOffset;    /* Offset at which write will happen */
    uint32_t byteLen;                   /* Total number of bytes to be written */
    int32_t status;
      
    /*Load the firmware image*/
    imemOffset = 0;
#if(CONFIG_SDFM0_LOAD_SHARE == 1)
    {
#if CONFIG_SDFM0_CHANNEL0 || CONFIG_SDFM0_CHANNEL1 || CONFIG_SDFM0_CHANNEL2
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_RTU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
        pPruFwImageInfo = &gPruFwImageInfo[2];
        pruIMem = PRUICSS_IRAM_RTU_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_RTU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
#endif
#if CONFIG_SDFM0_CHANNEL3 || CONFIG_SDFM0_CHANNEL4 || CONFIG_SDFM0_CHANNEL5
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
        pPruFwImageInfo = &gPruFwImageInfo[1];
        pruIMem = PRUICSS_IRAM_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }   

        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
#endif
#if CONFIG_SDFM0_CHANNEL6 || CONFIG_SDFM0_CHANNEL7 || CONFIG_SDFM0_CHANNEL8
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_TXPRU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
        pPruFwImageInfo = &gPruFwImageInfo[3];
        pruIMem = PRUICSS_IRAM_TX_PRU(CONFIG_SDFM0_SLICE);
        if(SDFM_TXPRU_CORE == PRUICSS_TX_PRU0)
        {
            PRUICSS_setConstantTblEntry(pruIcssHandle, SDFM_TXPRU_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A4);
        }
        else
        {
            PRUICSS_setConstantTblEntry(pruIcssHandle, SDFM_TXPRU_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A5);
        }
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }   

        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_TXPRU_CORE);
        if (status != SystemP_SUCCESS) 
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
#endif
    }
#else
    {
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
        pPruFwImageInfo = &gPruFwImageInfo[0];
        pruIMem = PRUICSS_IRAM_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SDFM_ERR_INIT_PRU_SDFM;
        }

        /* Enable PRU */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS) {
            return SDFM_ERR_INIT_PRU_SDFM;
        }
    }
#endif

    status = initSdfmFw(pSdfmPrms, pHSdfm);
    if (status != SDFM_ERR_NERR) 
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }
    return SDFM_ERR_NERR;

}
