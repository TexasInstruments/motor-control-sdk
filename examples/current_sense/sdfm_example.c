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
int32_t initIcss(
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
    if (sliceId == ICSSG_SLICE_ID_0)
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
    else if (sliceId == ICSSG_SLICE_ID_1)
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
void SDFM_configGpioPins(SDFM_Handle h_sdfm, uint8_t SD_CH)
{
    switch (SD_CH)
    {
        case 0:
#if (CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH0_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH0_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH0_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 1:
#if (CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH1_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH1_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH1_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;      
        case 2:
#if (CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH2_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH2_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH2_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 3:
#if (CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH3_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH3_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH3_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 4:
#if (CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH4_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH4_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH4_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 5:
#if (CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH5_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH5_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH5_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;  
        case 6:
#if (CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH6_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH6_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH6_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 7:
#if (CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH7_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH7_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH7_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        case 8:
#if (CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH8_BASE_ADDR);
                uint32_t pinNum = GPIO_ZC_TH_CH8_PIN;
                GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_ZC_TH_CH8_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, SD_CH, gpioBaseAddr, pinNum);
            }
#endif
            break;
        default:
            break;
    }
}

/* Initialize SDFM PRU FW */
int32_t initSdfmFw(SDFM_Params sdfm_params, SDFM_Handle *pHSdfm)
{
    SDFM_Handle hSdfm;
    uint8_t SDFM_CH;    

    /* Initialize SDFM instance */
    hSdfm = SDFM_init(sdfm_params, CONFIG_SDFM0);
    
    if (hSdfm == NULL)
    {
        return SDFM_ERR_INIT_SDFM;
    }

    if(sdfm_params.load_share_enable)
    {
        SDFM_enableLoadShareMode(hSdfm, sdfm_params.pru_slice_value);
    }

    for(int i = 0; i< 9; i++)
    {
        if(sdfm_params.sdfm_channel_mask & (1 << i))
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
    hSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)(sdfm_params.samplesBaseAddress);
    uint32_t sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW(sdfm_params.samplesBaseAddress);
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
#endif
     
    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
#if (CONFIG_SDFM0_CLK_FROM_ECAP != 0)
    uint8_t ecap_divider = 0x0F; /*PRU clock at 300MHz: SD clock = 300/15=20Mhz*/
    SDFM_configEcap(hSdfm, ecap_divider);
    hSdfm->clk_config.clock_source = SDFM_CLOCK_SOURCE_ECAP;
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
#endif
   for(i=0; i<SDFM_NUM_OF_CH_PER_PRU_SLICE; i++)
   {
        if(sdfm_params.sdfm_channel_mask & (1 << i))
        {
            hSdfm->clk_config.sdfm_clock_value = sdfm_params.channels[i].sdfmClock;
            break;
        }
   }
   /*Add code to confgure common configuration for all channels*/
   if(sdfm_params.enable_snoop_mode[0]|| sdfm_params.enable_snoop_mode[1]|| sdfm_params.enable_snoop_mode[2])
   {
        /*configure IEP count for one epwm period*/
        SDFM_configIepCount(hSdfm, sdfm_params.iep_reset_freq);
   }
   
   for(int i = 0; i< 3; i++)
   {
        if(sdfm_params.sdfm_enable_pru_core_mask & (1 << i))
        {
           
            if(sdfm_params.enable_snoop_mode[i])
            {
                SDFM_enableSnoopBasedNC(hSdfm, i);
            }
            if(sdfm_params.trigger_config[i].enable_trigger_mode == 1)
            {
                SDFM_enableTriggerModeForNormalCurrent(hSdfm, i);
                SDFM_setSampleTriggerTime(hSdfm, sdfm_params.trigger_config[i].first_samp_trig_time, i);
                if(sdfm_params.trigger_config[i].en_double_nc_sampling)
                {
                    SDFM_enableDoubleSampling(hSdfm, sdfm_params.trigger_config[i].second_samp_trig_time, i);
                }
                else
                {
                    SDFM_disableDoubleSampling(hSdfm, i);
                }
                
                SDFM_selectIepCmpEvent(hSdfm, sdfm_params.trigger_config[i].iep_cmp_event, i);  
            }
        }
   }
   
    /*enable epwm sync*/
    if(sdfm_params.sdfm_enable_epwm_sync)
    {
        SDFM_enableEpwmSync(hSdfm, sdfm_params.sdfm_epwm_sync_source);
        SDFM_enableIep(hSdfm);
    }
    else
    {
        if(sdfm_params.trigger_config[0].enable_trigger_mode == 1|| sdfm_params.trigger_config[1].enable_trigger_mode == 1|| sdfm_params.trigger_config[2].enable_trigger_mode == 1)
        {
            SDFM_configIepCmp0ToResetIep(hSdfm, sdfm_params.iep_reset_freq);
            SDFM_enableIep(hSdfm);
        }
    }
     
    /*Phase delay calculation for ch0. With single PRU and no load share*/
    if(sdfm_params.sdfm_enable_phase_delay)
    {
#if (CONFIG_SDFM0_CHANNEL0 == 1 && (CONFIG_SDFM0_LOAD_SHARE == 0))
        SDFM_measureClockPhaseDelay(hSdfm, sdfm_params.channels[0].clk_inv, 0);
#endif
    }

    /*below configuration for all three channel*/
    for(SDFM_CH = 0; SDFM_CH < 9; SDFM_CH++)
    {
        if(sdfm_params.sdfm_channel_mask & (1 << SDFM_CH))
        {
            SDFM_setCompFilterOverSamplingRatio(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].over_current_osr);

            SDFM_setFilterOverSamplingRatio(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].normal_current_osr);

            /*set ACC source or filter type*/
            SDFM_configDataFilter(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].filter_type);

            /*set clock inversion & clock source for all three channel*/
            SDFM_selectClockSource(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].clk_source);

            /*set clock inversion*/
            SDFM_setClockInversion(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].clk_inv);

            if(sdfm_params.channels[SDFM_CH].enable_comparator == 1)
            {
                SDFM_enableComparator(hSdfm, SDFM_CH);
                /*set high and low thresholds value */
                uint32_t comThresholds[2];
                comThresholds[0] = sdfm_params.channels[SDFM_CH].threshold_config.high_threshold;
                comThresholds[1] = sdfm_params.channels[SDFM_CH].threshold_config.low_threshold;   
                SDFM_setCompFilterThresholds(hSdfm, SDFM_CH, comThresholds);
            }

            if(sdfm_params.channels[SDFM_CH].enFastDetect == 1)
            {
                /*Fast detect configuration */
                uint8_t channels[NUM_FD_FIELD];
                channels[1] = sdfm_params.channels[SDFM_CH].fd_window;
                channels[2] = sdfm_params.channels[SDFM_CH].fd_zero_max;
                channels[3] = sdfm_params.channels[SDFM_CH].fd_zero_min;
                SDFM_configFastDetect(hSdfm, SDFM_CH, channels);
            }

            if(sdfm_params.channels[SDFM_CH].threshold_config.zeroCrossEn == 1)
            {
                /*zero cross configuration*/
                SDFM_enableZeroCrossDetection(hSdfm, SDFM_CH, sdfm_params.channels[SDFM_CH].threshold_config.zeroCrossTh);
                /*GPIO pin configuration for zero cross*/
                SDFM_configGpioPins(hSdfm, SDFM_CH);
            }
        }

    }
    
    /* Enable (global) SDFM */
    for(int i = 0; i< 3; i++)
    {
        if(sdfm_params.sdfm_enable_pru_core_mask & (1 << i))
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

/*
 *  ======== Initialize SDFM parameters ========
 */
 
 void sdfmParamsConfig(uint8_t channel, SDFM_Params *gSdfmPrms)
{
    SDFM_Params gTestSdfmPrms = *gSdfmPrms;
    switch(channel)
    {
        case 0:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL0;
#if (CONFIG_SDFM0_CHANNEL0 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL0_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL0_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL0_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL0_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL0_NC_OSR;
            
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL0_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL0_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL0_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL0_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL0_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL0_OC_ZC_TH;
#endif     
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL0_EN_FD;
#if(CONFIG_SDFM0_CHANNEL0_EN_FD != 0 )       
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL0_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL0_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL0_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 1:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL1;
#if (CONFIG_SDFM0_CHANNEL1 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL1_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL1_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL1_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL1_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL1_NC_OSR;
            
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL1_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL1_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL1_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL1_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL1_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL1_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL1_EN_FD;
#if(CONFIG_SDFM0_CHANNEL1_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL1_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL1_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL1_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 2:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL2;
#if (CONFIG_SDFM0_CHANNEL2 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL2_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL2_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL2_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL2_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL2_NC_OSR;
            
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL2_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL2_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL2_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL2_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL2_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL2_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL2_EN_FD;
#if(CONFIG_SDFM0_CHANNEL2_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL2_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL2_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL2_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 3:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL3;
#if (CONFIG_SDFM0_CHANNEL3 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL3_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL3_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL3_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL3_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL3_NC_OSR;
            
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL3_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL3_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL3_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL3_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL3_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL3_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL3_EN_FD;
#if(CONFIG_SDFM0_CHANNEL3_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL3_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL3_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL3_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 4:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL4;
#if (CONFIG_SDFM0_CHANNEL4 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL4_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL4_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL4_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL4_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL4_NC_OSR;
            
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL4_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL4_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL4_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL4_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL4_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL4_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL4_EN_FD;
#if(CONFIG_SDFM0_CHANNEL4_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL4_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL4_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL4_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 5:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL5;
#if (CONFIG_SDFM0_CHANNEL5 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL5_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL5_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL5_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL5_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL5_NC_OSR;
           
            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL5_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL5_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL5_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL5_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL5_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL5_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL5_EN_FD;
#if(CONFIG_SDFM0_CHANNEL5_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL5_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL5_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL5_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 6:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL6;
#if (CONFIG_SDFM0_CHANNEL6 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL6_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL6_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL6_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL6_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL6_NC_OSR;

            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL6_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL6_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL6_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL6_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL6_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL6_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL6_EN_FD;
#if(CONFIG_SDFM0_CHANNEL6_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL6_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_zero_max = CONFIG_SDFM0_CHANNEL6_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_zero_min = CONFIG_SDFM0_CHANNEL6_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 7:
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL7;
#if (CONFIG_SDFM0_CHANNEL7 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL7_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL7_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL7_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL7_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL7_NC_OSR;

            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL7_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL7_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL7_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL7_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL7_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossEn = CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL7_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL7_EN_FD;
#if(CONFIG_SDFM0_CHANNEL7_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL7_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_max_zero = CONFIG_SDFM0_CHANNEL7_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_min_zero = CONFIG_SDFM0_CHANNEL7_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 8: 
            gTestSdfmPrms.channels[channel].enabled  =  CONFIG_SDFM0_CHANNEL8;
#if (CONFIG_SDFM0_CHANNEL8 !=0 )
            /*Clock parameters*/
            gTestSdfmPrms.channels[channel].sdfmClock = CONFIG_SDFM0_CHANNEL8_MCLK;
            gTestSdfmPrms.channels[channel].clk_source = CONFIG_SDFM0_CHANNEL8_CLK_SOURCE;
            gTestSdfmPrms.channels[channel].clk_inv = CONFIG_SDFM0_CHANNEL8_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channels[channel].filter_type = CONFIG_SDFM0_CHANNEL8_ACC_SOURCE;
            gTestSdfmPrms.channels[channel].normal_current_osr = CONFIG_SDFM0_CHANNEL8_NC_OSR;
           

            /*Over current parameters*/
            gTestSdfmPrms.channels[channel].enable_comparator = CONFIG_SDFM0_CHANNEL8_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL8_EN_COMP  != 0)
            gTestSdfmPrms.channels[channel].over_current_osr = CONFIG_SDFM0_CHANNEL8_OC_OSR;
            gTestSdfmPrms.channels[channel].threshold_config.high_threshold = CONFIG_SDFM0_CHANNEL8_OC_HIGH_TH;
            gTestSdfmPrms.channels[channel].threshold_config.low_threshold = CONFIG_SDFM0_CHANNEL8_OC_LOW_TH;
            gTestSdfmPrms.channels[channel].zeroCrossEn = CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.channels[channel].threshold_config.zeroCrossTh = CONFIG_SDFM0_CHANNEL8_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.channels[channel].enFastDetect = CONFIG_SDFM0_CHANNEL8_EN_FD;
#if(CONFIG_SDFM0_CHANNEL8_EN_FD != 0 )
            gTestSdfmPrms.channels[channel].fd_window = CONFIG_SDFM0_CHANNEL8_FD_WINDOW_SIZE;
            gTestSdfmPrms.channels[channel].fd_max_zero = CONFIG_SDFM0_CHANNEL8_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.channels[channel].fd_min_zero = CONFIG_SDFM0_CHANNEL8_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        default:
            break;
    }

    *gSdfmPrms = gTestSdfmPrms;
}

void sdfmAxisParamsConfig(SDFM_Params *gSdfmPrms, uint8_t pru_core)
{
    SDFM_Params sdfmParams  = *gSdfmPrms;
    /* Configure Sdfm Axis level parameters */
    switch(pru_core)
    {

        case 0:
#if (CONFIG_SDFM0_PRU_CORE_ENABLE != 0 )
            sdfmParams.enable_snoop_mode[pru_core] = CONFIG_SDFM0_PRU_NC_SNOOP_MODE;
#if (CONFIG_SDFM0_PRU_EN_TRIGGER_MODE != 0)
            sdfmParams.trigger_config[pru_core].first_samp_trig_time = CONFIG_SDFM0_PRU_FIRST_TRIGGER_POINT;
            sdfmParams.trigger_config[pru_core].enable_trigger_mode = CONFIG_SDFM0_PRU_EN_TRIGGER_MODE;
            sdfmParams.trigger_config[pru_core].iep_cmp_event = CONFIG_SDFM0_PRU_IEP_CMP_EVENT;

#if (CONFIG_SDFM0_PRU_EN_DOUBLE_UPDATE != 0 )
            sdfmParams.trigger_config[pru_core].en_double_nc_sampling = CONFIG_SDFM0_PRU_EN_DOUBLE_UPDATE;
            sdfmParams.trigger_config[pru_core].second_samp_trig_time = CONFIG_SDFM0_PRU_SECOND_TRIGGER_POINT;
#endif
#endif
#endif
            break;
        case 1:
#if (CONFIG_SDFM0_RTU_CORE_ENABLE != 0 )
            sdfmParams.enable_snoop_mode[pru_core] = CONFIG_SDFM0_RTU_NC_SNOOP_MODE;    
#if (CONFIG_SDFM0_RTU_EN_TRIGGER_MODE != 0)
            sdfmParams.trigger_config[pru_core].first_samp_trig_time = CONFIG_SDFM0_RTU_FIRST_TRIGGER_POINT;
            sdfmParams.trigger_config[pru_core].enable_trigger_mode = CONFIG_SDFM0_RTU_EN_TRIGGER_MODE;
            sdfmParams.trigger_config[pru_core].iep_cmp_event = CONFIG_SDFM0_RTU_IEP_CMP_EVENT;
#if (CONFIG_SDFM0_RTU_EN_DOUBLE_UPDATE != 0 )
            sdfmParams.trigger_config[pru_core].en_double_nc_sampling = CONFIG_SDFM0_RTU_EN_DOUBLE_UPDATE;
            sdfmParams.trigger_config[pru_core].second_samp_trig_time = CONFIG_SDFM0_RTU_SECOND_TRIGGER_POINT;
#endif
#endif
#endif

            break;
        case 2:
#if (CONFIG_SDFM0_TXPRU_CORE_ENABLE != 0 )
            sdfmParams.enable_snoop_mode[pru_core] = CONFIG_SDFM0_TXPRU_NC_SNOOP_MODE;
#if (CONFIG_SDFM0_TXPRU_EN_TRIGGER_MODE != 0)
            sdfmParams.trigger_config[pru_core].first_samp_trig_time = CONFIG_SDFM0_TXPRU_FIRST_TRIGGER_POINT;
            sdfmParams.trigger_config[pru_core].enable_trigger_mode = CONFIG_SDFM0_TXPRU_EN_TRIGGER_MODE;
            sdfmParams.trigger_config[pru_core].iep_cmp_event = CONFIG_SDFM0_TXPRU_IEP_CMP_EVENT;   
#if (CONFIG_SDFM0_TXPRU_EN_DOUBLE_UPDATE != 0 )
            sdfmParams.trigger_config[pru_core].en_double_nc_sampling = CONFIG_SDFM0_TXPRU_EN_DOUBLE_UPDATE;
            sdfmParams.trigger_config[pru_core].second_samp_trig_time = CONFIG_SDFM0_TXPRU_SECOND_TRIGGER_POINT;
#endif
#endif
#endif
            break;
        default:
            break;
    }
    
    *gSdfmPrms = sdfmParams;
}
void sdfmGlobalParamsConfig(SDFM_Params *gSdfmPrms)
{
    SDFM_Params sdfmParams  = *gSdfmPrms;
    int32_t mask;
    /* Configure Sdfm parameters */
    sdfmParams.load_share_enable = CONFIG_SDFM0_LOAD_SHARE;
    sdfmParams.iep_clock = CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ;
    sdfmParams.pru_clock = CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ;
#if (CONFIG_SDFM0_TXPRU_EN_TRIGGER_MODE != 0 || CONFIG_SDFM0_RTU_EN_TRIGGER_MODE != 0 || CONFIG_SDFM0_PRU_EN_TRIGGER_MODE != 0)
    sdfmParams.iep_instance = CONFIG_SDFM0_IEP_INSTANCE;
    sdfmParams.iep_inc_value = 1;
    sdfmParams.iep_reset_freq = CONFIG_SDFM0_IEP_RESET_FREQ;
#endif
    sdfmParams.pru_slice_value = CONFIG_SDFM0_SLICE;
    mask = (CONFIG_SDFM0_CHANNEL0 ? 1<< 0 : 0) | (CONFIG_SDFM0_CHANNEL1 ? 1<< 1 : 0) | (CONFIG_SDFM0_CHANNEL2 ? 1<< 2 : 0) | \
           (CONFIG_SDFM0_CHANNEL3 ? 1<< 3 : 0) | (CONFIG_SDFM0_CHANNEL4 ? 1<< 4 : 0) | (CONFIG_SDFM0_CHANNEL5 ? 1<< 5 : 0) | \
           (CONFIG_SDFM0_CHANNEL6 ? 1<< 6 : 0) | (CONFIG_SDFM0_CHANNEL7 ? 1<< 7 : 0) | (CONFIG_SDFM0_CHANNEL8 ? 1<< 8 : 0);
    sdfmParams.sdfm_channel_mask = mask;
#if(CONFIG_SDFM0_LOAD_SHARE == 0)
    sdfmParams.sdfm_enable_pru_core_mask = 1;
#else
    sdfmParams.sdfm_enable_pru_core_mask |= (CONFIG_SDFM0_CHANNEL0 || CONFIG_SDFM0_CHANNEL1 || CONFIG_SDFM0_CHANNEL2) ? 1 >> 1: 0;
    sdfmParams.sdfm_enable_pru_core_mask |= (CONFIG_SDFM0_CHANNEL3 || CONFIG_SDFM0_CHANNEL4 || CONFIG_SDFM0_CHANNEL5) ? 1 >> 0: 0;
    sdfmParams.sdfm_enable_pru_core_mask |= (CONFIG_SDFM0_CHANNEL6 || CONFIG_SDFM0_CHANNEL7 || CONFIG_SDFM0_CHANNEL8) ? 1 >> 2: 0;  
#endif

#if (CONFIG_SDFM0_EPWM_SYNC_EN == 1)
    sdfmParams.sdfm_enable_epwm_sync = 1;
    sdfmParams.sdfm_epwm_sync_source = CONFIG_SDFM0_EPWM_SYNC_SOURCE;
#else
    sdfmParams.sdfm_enable_epwm_sync = 0;
#endif
    
#if CONFIG_SDFM0_PHASE_DELAY != 0
    sdfmParams.sdfm_phase_delay = 1;
#else
    sdfmParams.sdfm_phase_delay = 0;
#endif  
    *gSdfmPrms = sdfmParams;
}

