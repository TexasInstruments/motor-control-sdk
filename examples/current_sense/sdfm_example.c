/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "tisdfm_pruss_intc_mapping.h"  /* INTC configuration */
#include "current_sense/sdfm/firmware/sdfm_pru_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/sdfm_rtu_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/sdfm_txpru_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/sdfm_bin.h"            /* SDFM image data */

#include "sdfm_example.h"
#include "current_sense/sdfm/include/sdfm_api.h"

/* PRU SDFM FW image info */
typedef struct PRUSDFM_PruFwImageInfo_s 
{
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDFM_PruFwImageInfo;

/* Number of PRU images */
#define PRU_SDFM_NUM_PRU_IMAGE  ( 4 )

/* PRU SDFM image info */
static PRUSDFM_PruFwImageInfo gPruFwImageInfo[PRU_SDFM_NUM_PRU_IMAGE] =
{
    {SDFM_PRU0_image_0, sizeof(SDFM_PRU0_image_0)}, /* single PRU FW binary */
    {pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0)}, /* load share PRU FW binary */
    {pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0)}, /*load share RTU FW binary */
    {pru_SDFM_TXPRU0_image_0, sizeof(pru_SDFM_TXPRU0_image_0)} /*load share TXPRU binary*/ 
};

/* ICSS INTC configuration */
static const PRUICSS_IntcInitData gPruicssIntcInitdata = PRUICSS_INTC_INITDATA;

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
    PRUICSS_setSaMuxMode(pruIcssHandle, saMuxMode);

    /* Initialize ICSS INTC */
    status = PRUICSS_intcInit(pruIcssHandle, &gPruicssIntcInitdata);
    if (status != SystemP_SUCCESS) {
        return SDFM_ERR_INIT_ICSSG;
    }

    *pPruIcssHandle = pruIcssHandle;

    return SDFM_ERR_NERR;
}
void SDFM_configGpioPins(sdfm_handle h_sdfm, uint8_t loadShare, uint8_t pruInsId)
{
    if(loadShare)
    {
        uint32_t gpioBaseAddrCh;
        uint32_t pinNumCh;
        switch (pruInsId)
        {
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                /*ch5 GPIO configuration*/
                gpioBaseAddrCh = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH1_BASE_ADDR);
                pinNumCh       = GPIO_ZC_TH_CH1_PIN;
                GPIO_setDirMode(gpioBaseAddrCh, pinNumCh, GPIO_ZC_TH_CH1_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh, pinNumCh);
                break;
            case PRUICSS_RTU_PRU0:
            case PRUICSS_RTU_PRU1:
                /*ch2 GPIO configuration*/
                gpioBaseAddrCh = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH0_BASE_ADDR);
                pinNumCh       = GPIO_ZC_TH_CH0_PIN;
                GPIO_setDirMode(gpioBaseAddrCh, pinNumCh, GPIO_ZC_TH_CH0_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh, pinNumCh);
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                /*ch8 GPIO configuration*/
                gpioBaseAddrCh = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH2_BASE_ADDR);
                pinNumCh       = GPIO_ZC_TH_CH2_PIN;
                GPIO_setDirMode(gpioBaseAddrCh, pinNumCh, GPIO_ZC_TH_CH2_DIR);
                SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh, pinNumCh);
                break;
            default:
                break;
        }
    }
    else
    {
        /*ch0 GPIO configuration*/
        uint32_t gpioBaseAddrCh0 = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH0_BASE_ADDR);
        uint32_t pinNumCh0       = GPIO_ZC_TH_CH0_PIN;
        GPIO_setDirMode(gpioBaseAddrCh0, pinNumCh0, GPIO_ZC_TH_CH0_DIR);
        SDFM_configComparatorGpioPins(h_sdfm, 0, gpioBaseAddrCh0, pinNumCh0);
    
        /*ch1 GPIO configuration*/
        uint32_t gpioBaseAddrCh1 = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH1_BASE_ADDR);
        uint32_t pinNumCh1       = GPIO_ZC_TH_CH1_PIN;
        GPIO_setDirMode(gpioBaseAddrCh1, pinNumCh1, GPIO_ZC_TH_CH1_DIR);
        SDFM_configComparatorGpioPins(h_sdfm, 1, gpioBaseAddrCh1, pinNumCh1);
    
        /*ch2 GPIO configuration*/
        uint32_t gpioBaseAddrCh2 = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH2_BASE_ADDR);
        uint32_t pinNumCh2       = GPIO_ZC_TH_CH2_PIN;
        GPIO_setDirMode(gpioBaseAddrCh2, pinNumCh2, GPIO_ZC_TH_CH2_DIR);
        SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh2, pinNumCh2);
    }
}   

/* Initialize SDFM PRU FW */
int32_t initSdfmFw(uint8_t pruId, SdfmPrms *pSdfmPrms, sdfm_handle *pHSdfm,  PRUICSS_Handle pruIcssHandle)
{
    sdfm_handle hSdfm;    
    uint8_t SDFM_CH = 0;

    /* Initialize SDFM instance */
    hSdfm = SDFM_init(pruIcssHandle, pruId, pSdfmPrms->pruInsId);
    
    hSdfm->pruicssCfg = (void *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->cfgRegBase);


    /*IEP base address*/
    hSdfm->pruicssIep = (void *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->iep0RegBase);

    /*eCap base address*/
    hSdfm->pruicssEcap = (void *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->ecapRegBase);

    if( pSdfmPrms->loadShare )
    {
        if((pSdfmPrms->pruInsId == PRUICSS_PRU0)||(pSdfmPrms->pruInsId == PRUICSS_PRU1))
        {
            SDFM_enableLoadShareMode(hSdfm, pSdfmPrms->pruSliceId);
        }

        switch (pSdfmPrms->pruInsId)
        {
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                SDFM_CH = 3;
                break;
            case PRUICSS_RTU_PRU0:
            case PRUICSS_RTU_PRU1:
                SDFM_CH = 0;
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                SDFM_CH = 6;
                break;
            default:
                SDFM_CH = 0;
                break;
        }
    }
    for(int i = SDFM_CH; i<SDFM_CH + NUM_CH_SUPPORTED_PER_AXIS; i++)
    {
        SDFM_setEnableChannel(hSdfm, i);
    }
    uint32_t i;
    i = SDFM_getFirmwareVersion(hSdfm);
    DebugP_log("\n\n\n");
    DebugP_log("SDFM firmware version \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");
    if (hSdfm == NULL)
    {
        return SDFM_ERR_INIT_SDFM;
    }

    hSdfm->pruCoreClk = pSdfmPrms->pruClock;
    hSdfm->iepClock = pSdfmPrms->iepClock[0]; 
    hSdfm->sdfmClock = pSdfmPrms->clkPrms[0].sdClock; 
    hSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)(pSdfmPrms->samplesBaseAddress);
    uint32_t sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW(pSdfmPrms->samplesBaseAddress);
    hSdfm->pSdfmInterface->sampleBufferBaseAdd = sampleOutputInterfaceGlobalAddr;
    hSdfm->iepInc = 1; /* Default IEP increment 1 */
    

#if (CONFIG_SDFM0_CLK_FROM_IEP != 0)
    /* IEP clock 300MHz, SD clk = 20Mhz
      Div = 300/20 = 15, one period time = 15 IEP cycles, high plus time = 7 IEP cycles  */
    uint32_t highPulseWidth = 6; /*7 - 1*/
    uint32_t periodTime = 14;  /* 15 - 1*/
    uint32_t syncStartTime = 0; /*clock generation start time.*/
    SDFM_configIepSyncMode(hSdfm, highPulseWidth, periodTime, syncStartTime);
    SDFM_enableIep(hSdfm);   
#endif
     
    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
#if (CONFIG_SDFM0_CLK_FROM_ECAP != 0)
    uint8_t ecap_divider = 0x0F; /*PRU clock at 300MHz: SD clock = 300/15=20Mhz*/
    SDFM_configEcap(hSdfm, ecap_divider);
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

#endif
   
   /*configure IEP count for one epwm period*/
    SDFM_configIepCount(hSdfm, pSdfmPrms->epwmOutFreq);

   /*configuration of sdfm parameters which are supported per axis, not for invidual channels.
    Channel0 perametrs value are used for all 3 channels of axis*/

    /*Phase delay calculation for ch0. With Load share mode also, phase delay calculation is enabled only for channel0 */
    if(pSdfmPrms->phaseDelay)
    {
        SDFM_measureClockPhaseDelay(hSdfm, pSdfmPrms->clkPrms[0].clkInv);
    }

    /*set Noraml current OSR */
    SDFM_setFilterOverSamplingRatio(hSdfm, pSdfmPrms->channelPrms[0].filterOsr);


    /*Enable Continuous mode*/
    if(pSdfmPrms->channelPrms[0].enableContinuousMode)
    {
        SDFM_enableContinuousNormalCurrent(hSdfm);
        /*When Continuous mode is enabled, configure the first sample starting point. 
        Sampling starts with some delay after IEP counter starts because there will be delay due to sdfm register configuration so we cannot trigger CMP event immediately
        triggring cmp event 5us late*/
        pSdfmPrms->channelPrms[0].firstSampTrigTime = 5;
    }

    /*GPIO pin configuration for zero cross*/
    SDFM_configGpioPins(hSdfm, pSdfmPrms->loadShare, pSdfmPrms->pruInsId);

    SDFM_setSampleTriggerTime(hSdfm, pSdfmPrms->channelPrms[0].firstSampTrigTime);
    if(pSdfmPrms->channelPrms[0].enSecondUpdate)
    {
        SDFM_enableDoubleSampling(hSdfm, pSdfmPrms->channelPrms[0].secondSampTrigTime);
    }
    else
    {
        SDFM_disableDoubleSampling(hSdfm);
    }
    
    /*enable epwm sync*/
    if(pSdfmPrms->channelPrms[0].enableEpwmSync)
    {
        SDFM_enableEpwmSync(hSdfm, pSdfmPrms->channelPrms[0].epwmSyncSource);
    }
     

    /*below configuration for all three channel*/
    for(SDFM_CH = 0; SDFM_CH < NUM_CH_SUPPORTED_PER_AXIS; SDFM_CH++)
    {

        /*set comparator osr or Over current osr*/
        SDFM_setCompFilterOverSamplingRatio(hSdfm, SDFM_CH, pSdfmPrms->compFilterPrms[SDFM_CH].comFilterOsr);

        /*set ACC source or filter type*/
        SDFM_configDataFilter(hSdfm, SDFM_CH, pSdfmPrms->channelPrms[SDFM_CH].accSource);

        /*set clock inversion & clock source for all three channel*/
        SDFM_selectClockSource(hSdfm, SDFM_CH, pSdfmPrms->clkPrms[SDFM_CH].clkSource);

        /*set clock inversion*/
        SDFM_setClockInversion(hSdfm, SDFM_CH, pSdfmPrms->clkPrms[SDFM_CH].clkInv);

        if(pSdfmPrms->fastDetectPrms[SDFM_CH][0])
        {
            /*Fast detect configuration */
            SDFM_configFastDetect(hSdfm, SDFM_CH, pSdfmPrms->fastDetectPrms[SDFM_CH]);
        }
        if(pSdfmPrms->compFilterPrms[SDFM_CH].enComparator)
        {
            SDFM_enableComparator(hSdfm, SDFM_CH); 
            /*set high and low thresholds value */
            SDFM_setCompFilterThresholds(hSdfm, SDFM_CH, pSdfmPrms->compFilterPrms[SDFM_CH].comThresholds);  
        }
        else
        {
            SDFM_disableComparator(hSdfm, SDFM_CH);
        }

        /*enabling Zero cross only for first channel of axis in load share mode*/

        if(pSdfmPrms->compFilterPrms[SDFM_CH].zeroCrossEn && ((pSdfmPrms->loadShare && SDFM_CH == 2)||(!pSdfmPrms->loadShare)))
        {
            SDFM_enableZeroCrossDetection(hSdfm, SDFM_CH, pSdfmPrms->compFilterPrms[SDFM_CH].zeroCrossTh);
        }

    }
    
    /* Enable (global) SDFM */
    SDFM_enable(hSdfm);

    *pHSdfm = hSdfm;

 return SDFM_ERR_NERR;
}
/*
 *  ======== initPruSdfm ========
 */
/* Initialize PRU core for SDFM */
int32_t initPruSdfm(
    PRUICSS_Handle pruIcssHandle,
    uint8_t pruInstId,
    SdfmPrms *pSdfmPrms,
    sdfm_handle *pHSdfm
)
{
    uint8_t sliceId;
    uint32_t pruIMem;
    PRUSDFM_PruFwImageInfo *pPruFwImageInfo;
    int32_t size;
    const uint32_t *sourceMem;          /* Source memory[ Array of uint32_t ] */
    uint32_t imemOffset;    /* Offset at which write will happen */
    uint32_t byteLen;                   /* Total number of bytes to be written */
    uint8_t pruId;
    int32_t status;
    
    /* Reset PRU */
    status = PRUICSS_resetCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) 
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }


    /* Calculate slice ID */
    sliceId = pruInstId - (uint8_t)pruInstId/ICSSG_NUM_SLICE * ICSSG_NUM_SLICE;
    /* Determine PRU DMEM address */
    /* Determine PRU FW image and PRU IMEM address */

    switch (pruInstId)
    {
        case PRUICSS_PRU0:
        case PRUICSS_PRU1:
            if(pSdfmPrms->loadShare)
            { 
                pPruFwImageInfo = &gPruFwImageInfo[1];
                pruIMem = PRUICSS_IRAM_PRU(sliceId);
            }
            else
            {
                pPruFwImageInfo = &gPruFwImageInfo[0];
                pruIMem = PRUICSS_IRAM_PRU(sliceId);
            }
            break;
        case PRUICSS_RTU_PRU0:
        case PRUICSS_RTU_PRU1:
            pPruFwImageInfo = &gPruFwImageInfo[2];
            pruIMem = PRUICSS_IRAM_RTU_PRU(sliceId);
            break;
        case PRUICSS_TX_PRU0:
        case PRUICSS_TX_PRU1:
            pPruFwImageInfo = &gPruFwImageInfo[3];
            pruIMem = PRUICSS_IRAM_TX_PRU(sliceId);
            if(pruInstId == PRUICSS_TX_PRU0)
            {
                PRUICSS_setConstantTblEntry(pruIcssHandle, pruInstId, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A4);
            }
            else
            {
                PRUICSS_setConstantTblEntry(pruIcssHandle, pruInstId, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A5);
            }
            break;
        default:
            pPruFwImageInfo = NULL;
            break;
    }

    if ((pPruFwImageInfo == NULL) ||
        (pPruFwImageInfo->pPruImemImg == NULL))
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }

    /* Write IMEM */
    imemOffset = 0;
    sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
    byteLen = pPruFwImageInfo->pruImemImgSz;
    size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
    if (size == 0)
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }

    /* Enable PRU */
    status = PRUICSS_enableCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) {
        return SDFM_ERR_INIT_PRU_SDFM;
    }
/* Translate PRU ID to SDFM API */
    if ((pruInstId == PRUICSS_PRU0) || (pruInstId == PRUICSS_RTU_PRU0) || (pruInstId == PRUICSS_TX_PRU0))
   {
        pruId = PRU_ID_0;
    }
    else if ((pruInstId == PRUICSS_PRU1) || (pruInstId == PRUICSS_RTU_PRU1) || (pruInstId == PRUICSS_TX_PRU1))
   {
        pruId = PRU_ID_1;
    }
    else 
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }

   
    status = initSdfmFw(pruId, pSdfmPrms, pHSdfm, pruIcssHandle);
    if (status != SDFM_ERR_NERR) 
    {
        return SDFM_ERR_INIT_PRU_SDFM;
    }
    return SDFM_ERR_NERR;

}

/*
 *  ======== Initialize SDFM parameters ========
 */
 
 void sdfmParamsConfig(uint8_t channel, SdfmPrms *gSdfmPrms)
{
    SdfmPrms gTestSdfmPrms = *gSdfmPrms;
    switch(channel)
    {
        case 0:
#if (CONFIG_SDFM0_CHANNEL0 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[channel].sdClock = CONFIG_SDFM0_CHANNEL0_MCLK;
            gTestSdfmPrms.clkPrms[channel].clkSource = CONFIG_SDFM0_CHANNEL0_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[channel].clkInv = CONFIG_SDFM0_CHANNEL0_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[channel].accSource = CONFIG_SDFM0_CHANNEL0_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[channel].filterOsr = CONFIG_SDFM0_CHANNEL0_NC_OSR;
            gTestSdfmPrms.channelPrms[channel].enableContinuousMode = CONFIG_SDFM0_CHANNEL0_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL0_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[channel].enSecondUpdate = CONFIG_SDFM0_CHANNEL0_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[channel].firstSampTrigTime = CONFIG_SDFM0_CHANNEL0_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL0_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[channel].secondSampTrigTime = CONFIG_SDFM0_CHANNEL0_SECOND_TRIGGER_POINT;          
#endif
#endif
            gTestSdfmPrms.channelPrms[channel].enableEpwmSync = CONFIG_SDFM0_CHANNEL0_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL0_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[channel].epwmSyncSource = CONFIG_SDFM0_CHANNEL0_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[channel].enComparator = CONFIG_SDFM0_CHANNEL0_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL0_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[channel].comFilterOsr = CONFIG_SDFM0_CHANNEL0_OC_OSR;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[0] = CONFIG_SDFM0_CHANNEL0_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[1] = CONFIG_SDFM0_CHANNEL0_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossEn = CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossTh = CONFIG_SDFM0_CHANNEL0_OC_ZC_TH;
#endif     
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[channel][0] = CONFIG_SDFM0_CHANNEL0_EN_FD;
#if(CONFIG_SDFM0_CHANNEL0_EN_FD != 0 )       
            gTestSdfmPrms.fastDetectPrms[channel][1] = CONFIG_SDFM0_CHANNEL0_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[channel][2] = CONFIG_SDFM0_CHANNEL0_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[channel][3] = CONFIG_SDFM0_CHANNEL0_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 1:
#if (CONFIG_SDFM0_CHANNEL1 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[channel].sdClock = CONFIG_SDFM0_CHANNEL1_MCLK;
            gTestSdfmPrms.clkPrms[channel].clkSource = CONFIG_SDFM0_CHANNEL1_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[channel].clkInv = CONFIG_SDFM0_CHANNEL1_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[channel].accSource = CONFIG_SDFM0_CHANNEL1_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[channel].filterOsr = CONFIG_SDFM0_CHANNEL1_NC_OSR;
            gTestSdfmPrms.channelPrms[channel].enableContinuousMode = CONFIG_SDFM0_CHANNEL1_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL1_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[channel].enSecondUpdate = CONFIG_SDFM0_CHANNEL1_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[channel].firstSampTrigTime = CONFIG_SDFM0_CHANNEL1_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL1_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[channel].secondSampTrigTime = CONFIG_SDFM0_CHANNEL1_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[channel].enableEpwmSync = CONFIG_SDFM0_CHANNEL1_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL1_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[channel].epwmSyncSource = CONFIG_SDFM0_CHANNEL1_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[channel].enComparator = CONFIG_SDFM0_CHANNEL1_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL1_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[channel].comFilterOsr = CONFIG_SDFM0_CHANNEL1_OC_OSR;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[0] = CONFIG_SDFM0_CHANNEL1_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[1] = CONFIG_SDFM0_CHANNEL1_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossEn = CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossTh = CONFIG_SDFM0_CHANNEL1_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[channel][0] = CONFIG_SDFM0_CHANNEL1_EN_FD;
#if(CONFIG_SDFM0_CHANNEL1_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[channel][1] = CONFIG_SDFM0_CHANNEL1_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[channel][2] = CONFIG_SDFM0_CHANNEL1_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[channel][3] = CONFIG_SDFM0_CHANNEL1_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 2:
#if (CONFIG_SDFM0_CHANNEL2 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[channel].sdClock = CONFIG_SDFM0_CHANNEL2_MCLK;
            gTestSdfmPrms.clkPrms[channel].clkSource = CONFIG_SDFM0_CHANNEL2_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[channel].clkInv = CONFIG_SDFM0_CHANNEL2_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[channel].accSource = CONFIG_SDFM0_CHANNEL2_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[channel].filterOsr = CONFIG_SDFM0_CHANNEL2_NC_OSR;
            gTestSdfmPrms.channelPrms[channel].enableContinuousMode = CONFIG_SDFM0_CHANNEL2_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL2_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[channel].enSecondUpdate = CONFIG_SDFM0_CHANNEL2_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[channel].firstSampTrigTime = CONFIG_SDFM0_CHANNEL2_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL2_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[channel].secondSampTrigTime = CONFIG_SDFM0_CHANNEL2_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[channel].enableEpwmSync = CONFIG_SDFM0_CHANNEL2_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL2_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[channel].epwmSyncSource = CONFIG_SDFM0_CHANNEL2_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[channel].enComparator = CONFIG_SDFM0_CHANNEL2_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL2_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[channel].comFilterOsr = CONFIG_SDFM0_CHANNEL2_OC_OSR;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[0] = CONFIG_SDFM0_CHANNEL2_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[channel].comThresholds[1] = CONFIG_SDFM0_CHANNEL2_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossEn = CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[channel].zeroCrossTh = CONFIG_SDFM0_CHANNEL2_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[channel][0] = CONFIG_SDFM0_CHANNEL2_EN_FD;
#if(CONFIG_SDFM0_CHANNEL2_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[channel][1] = CONFIG_SDFM0_CHANNEL2_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[channel][2] = CONFIG_SDFM0_CHANNEL2_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[channel][3] = CONFIG_SDFM0_CHANNEL2_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 3:
#if (CONFIG_SDFM0_CHANNEL3 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[0].sdClock = CONFIG_SDFM0_CHANNEL3_MCLK;
            gTestSdfmPrms.clkPrms[0].clkSource = CONFIG_SDFM0_CHANNEL3_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[0].clkInv = CONFIG_SDFM0_CHANNEL3_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[0].accSource = CONFIG_SDFM0_CHANNEL3_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[0].filterOsr = CONFIG_SDFM0_CHANNEL3_NC_OSR;
            gTestSdfmPrms.channelPrms[0].enableContinuousMode = CONFIG_SDFM0_CHANNEL3_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL3_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[0].enSecondUpdate = CONFIG_SDFM0_CHANNEL3_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[0].firstSampTrigTime = CONFIG_SDFM0_CHANNEL3_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL3_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[0].secondSampTrigTime = CONFIG_SDFM0_CHANNEL3_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[0].enableEpwmSync = CONFIG_SDFM0_CHANNEL3_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL3_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[0].epwmSyncSource = CONFIG_SDFM0_CHANNEL3_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[0].enComparator = CONFIG_SDFM0_CHANNEL3_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL3_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[0].comFilterOsr = CONFIG_SDFM0_CHANNEL3_OC_OSR;
            gTestSdfmPrms.compFilterPrms[0].comThresholds[0] = CONFIG_SDFM0_CHANNEL3_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[0].comThresholds[1] = CONFIG_SDFM0_CHANNEL3_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[0].zeroCrossEn = CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[0].zeroCrossTh = CONFIG_SDFM0_CHANNEL3_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[0][0] = CONFIG_SDFM0_CHANNEL3_EN_FD;
#if(CONFIG_SDFM0_CHANNEL3_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[0][1] = CONFIG_SDFM0_CHANNEL3_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[0][2] = CONFIG_SDFM0_CHANNEL3_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[0][3] = CONFIG_SDFM0_CHANNEL3_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 4:
#if (CONFIG_SDFM0_CHANNEL4 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[1].sdClock = CONFIG_SDFM0_CHANNEL4_MCLK;
            gTestSdfmPrms.clkPrms[1].clkSource = CONFIG_SDFM0_CHANNEL4_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[1].clkInv = CONFIG_SDFM0_CHANNEL4_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[1].accSource = CONFIG_SDFM0_CHANNEL4_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[1].filterOsr = CONFIG_SDFM0_CHANNEL4_NC_OSR;
            gTestSdfmPrms.channelPrms[1].enableContinuousMode = CONFIG_SDFM0_CHANNEL4_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL4_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[1].enSecondUpdate = CONFIG_SDFM0_CHANNEL4_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[1].firstSampTrigTime = CONFIG_SDFM0_CHANNEL4_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL4_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[1].secondSampTrigTime = CONFIG_SDFM0_CHANNEL4_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[1].enableEpwmSync = CONFIG_SDFM0_CHANNEL4_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL4_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[1].epwmSyncSource = CONFIG_SDFM0_CHANNEL4_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[1].enComparator = CONFIG_SDFM0_CHANNEL4_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL4_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[1].comFilterOsr = CONFIG_SDFM0_CHANNEL4_OC_OSR;
            gTestSdfmPrms.compFilterPrms[1].comThresholds[0] = CONFIG_SDFM0_CHANNEL4_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[1].comThresholds[1] = CONFIG_SDFM0_CHANNEL4_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[1].zeroCrossEn = CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[1].zeroCrossTh = CONFIG_SDFM0_CHANNEL4_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[1][0] = CONFIG_SDFM0_CHANNEL4_EN_FD;
#if(CONFIG_SDFM0_CHANNEL4_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[1][1] = CONFIG_SDFM0_CHANNEL4_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[1][2] = CONFIG_SDFM0_CHANNEL4_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[1][3] = CONFIG_SDFM0_CHANNEL4_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 5:
#if (CONFIG_SDFM0_CHANNEL5 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[2].sdClock = CONFIG_SDFM0_CHANNEL5_MCLK;
            gTestSdfmPrms.clkPrms[2].clkSource = CONFIG_SDFM0_CHANNEL5_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[2].clkInv = CONFIG_SDFM0_CHANNEL5_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[2].accSource = CONFIG_SDFM0_CHANNEL5_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[2].filterOsr = CONFIG_SDFM0_CHANNEL5_NC_OSR;
            gTestSdfmPrms.channelPrms[2].enableContinuousMode = CONFIG_SDFM0_CHANNEL5_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL5_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[2].enSecondUpdate = CONFIG_SDFM0_CHANNEL5_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[2].firstSampTrigTime = CONFIG_SDFM0_CHANNEL5_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL5_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[2].secondSampTrigTime = CONFIG_SDFM0_CHANNEL5_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[2].enableEpwmSync = CONFIG_SDFM0_CHANNEL5_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL5_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[2].epwmSyncSource = CONFIG_SDFM0_CHANNEL5_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[2].enComparator = CONFIG_SDFM0_CHANNEL5_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL5_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[2].comFilterOsr = CONFIG_SDFM0_CHANNEL5_OC_OSR;
            gTestSdfmPrms.compFilterPrms[2].comThresholds[0] = CONFIG_SDFM0_CHANNEL5_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[2].comThresholds[1] = CONFIG_SDFM0_CHANNEL5_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[2].zeroCrossEn = CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[2].zeroCrossTh = CONFIG_SDFM0_CHANNEL5_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[2][0] = CONFIG_SDFM0_CHANNEL5_EN_FD;
#if(CONFIG_SDFM0_CHANNEL5_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[2][1] = CONFIG_SDFM0_CHANNEL5_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[2][2] = CONFIG_SDFM0_CHANNEL5_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[2][3] = CONFIG_SDFM0_CHANNEL5_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 6:
#if (CONFIG_SDFM0_CHANNEL6 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[0].sdClock = CONFIG_SDFM0_CHANNEL6_MCLK;
            gTestSdfmPrms.clkPrms[0].clkSource = CONFIG_SDFM0_CHANNEL6_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[0].clkInv = CONFIG_SDFM0_CHANNEL6_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[0].accSource = CONFIG_SDFM0_CHANNEL6_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[0].filterOsr = CONFIG_SDFM0_CHANNEL6_NC_OSR;
            gTestSdfmPrms.channelPrms[0].enableContinuousMode = CONFIG_SDFM0_CHANNEL6_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL6_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[0].enSecondUpdate = CONFIG_SDFM0_CHANNEL6_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[0].firstSampTrigTime = CONFIG_SDFM0_CHANNEL6_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL6_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[0].secondSampTrigTime = CONFIG_SDFM0_CHANNEL6_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[0].enableEpwmSync = CONFIG_SDFM0_CHANNEL6_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL6_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[0].epwmSyncSource = CONFIG_SDFM0_CHANNEL6_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[0].enComparator = CONFIG_SDFM0_CHANNEL6_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL6_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[0].comFilterOsr = CONFIG_SDFM0_CHANNEL6_OC_OSR;
            gTestSdfmPrms.compFilterPrms[0].comThresholds[0] = CONFIG_SDFM0_CHANNEL6_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[0].comThresholds[1] = CONFIG_SDFM0_CHANNEL6_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[0].zeroCrossEn = CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[0].zeroCrossTh = CONFIG_SDFM0_CHANNEL6_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[0][0] = CONFIG_SDFM0_CHANNEL6_EN_FD;
#if(CONFIG_SDFM0_CHANNEL6_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[0][1] = CONFIG_SDFM0_CHANNEL6_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[0][2] = CONFIG_SDFM0_CHANNEL6_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[0][3] = CONFIG_SDFM0_CHANNEL6_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 7:
#if (CONFIG_SDFM0_CHANNEL7 != 0)
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[1].sdClock = CONFIG_SDFM0_CHANNEL7_MCLK;
            gTestSdfmPrms.clkPrms[1].clkSource = CONFIG_SDFM0_CHANNEL7_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[1].clkInv = CONFIG_SDFM0_CHANNEL7_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[1].accSource = CONFIG_SDFM0_CHANNEL7_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[1].filterOsr = CONFIG_SDFM0_CHANNEL7_NC_OSR;
            gTestSdfmPrms.channelPrms[1].enableContinuousMode = CONFIG_SDFM0_CHANNEL7_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL7_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[1].enSecondUpdate = CONFIG_SDFM0_CHANNEL7_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[1].firstSampTrigTime = CONFIG_SDFM0_CHANNEL7_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL7_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[1].secondSampTrigTime = CONFIG_SDFM0_CHANNEL7_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[1].enableEpwmSync = CONFIG_SDFM0_CHANNEL7_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL7_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[1].epwmSyncSource = CONFIG_SDFM0_CHANNEL7_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[1].enComparator = CONFIG_SDFM0_CHANNEL7_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL7_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[1].comFilterOsr = CONFIG_SDFM0_CHANNEL7_OC_OSR;
            gTestSdfmPrms.compFilterPrms[1].comThresholds[0] = CONFIG_SDFM0_CHANNEL7_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[1].comThresholds[1] = CONFIG_SDFM0_CHANNEL7_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[1].zeroCrossEn = CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[1].zeroCrossTh = CONFIG_SDFM0_CHANNEL7_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[1][0] = CONFIG_SDFM0_CHANNEL7_EN_FD;
#if(CONFIG_SDFM0_CHANNEL7_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[1][1] = CONFIG_SDFM0_CHANNEL7_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[1][2] = CONFIG_SDFM0_CHANNEL7_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[1][3] = CONFIG_SDFM0_CHANNEL7_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        case 8: 
#if (CONFIG_SDFM0_CHANNEL8 !=0 )
            /*Clock parameters*/
            gTestSdfmPrms.clkPrms[2].sdClock = CONFIG_SDFM0_CHANNEL8_MCLK;
            gTestSdfmPrms.clkPrms[2].clkSource = CONFIG_SDFM0_CHANNEL8_CLK_SOURCE;
            gTestSdfmPrms.clkPrms[2].clkInv = CONFIG_SDFM0_CHANNEL8_EN_CLK_INV;
            
            /*Normal current parameters*/
            gTestSdfmPrms.channelPrms[2].accSource = CONFIG_SDFM0_CHANNEL8_ACC_SOURCE;
            gTestSdfmPrms.channelPrms[2].filterOsr = CONFIG_SDFM0_CHANNEL8_NC_OSR;
            gTestSdfmPrms.channelPrms[2].enableContinuousMode = CONFIG_SDFM0_CHANNEL8_EN_CONT_MODE;
#if(CONFIG_SDFM0_CHANNEL8_EN_CONT_MODE == 0)
            gTestSdfmPrms.channelPrms[2].enSecondUpdate = CONFIG_SDFM0_CHANNEL8_EN_DOUBLE_UPDATE;
            gTestSdfmPrms.channelPrms[2].firstSampTrigTime = CONFIG_SDFM0_CHANNEL8_FIRST_TRIGGER_POINT;
#if(CONFIG_SDFM0_CHANNEL8_EN_DOUBLE_UPDATE != 0)
            gTestSdfmPrms.channelPrms[2].secondSampTrigTime = CONFIG_SDFM0_CHANNEL8_SECOND_TRIGGER_POINT;
#endif
#endif
            gTestSdfmPrms.channelPrms[2].enableEpwmSync = CONFIG_SDFM0_CHANNEL8_EN_EPWM_SYNC;
#if(CONFIG_SDFM0_CHANNEL8_EN_EPWM_SYNC != 0)
            gTestSdfmPrms.channelPrms[2].epwmSyncSource = CONFIG_SDFM0_CHANNEL8_EPWM_SOURCE;
#endif

            /*Over current parameters*/
            gTestSdfmPrms.compFilterPrms[2].enComparator = CONFIG_SDFM0_CHANNEL8_EN_COMP;
#if(CONFIG_SDFM0_CHANNEL8_EN_COMP  != 0)
            gTestSdfmPrms.compFilterPrms[2].comFilterOsr = CONFIG_SDFM0_CHANNEL8_OC_OSR;
            gTestSdfmPrms.compFilterPrms[2].comThresholds[0] = CONFIG_SDFM0_CHANNEL8_OC_HIGH_TH;
            gTestSdfmPrms.compFilterPrms[2].comThresholds[1] = CONFIG_SDFM0_CHANNEL8_OC_LOW_TH;
            gTestSdfmPrms.compFilterPrms[2].zeroCrossEn = CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS;
#if(CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS != 0 )
            gTestSdfmPrms.compFilterPrms[2].zeroCrossTh = CONFIG_SDFM0_CHANNEL8_OC_ZC_TH;
#endif
#endif

            /*fast detect parameters*/
            gTestSdfmPrms.fastDetectPrms[2][0] = CONFIG_SDFM0_CHANNEL8_EN_FD;
#if(CONFIG_SDFM0_CHANNEL8_EN_FD != 0 )
            gTestSdfmPrms.fastDetectPrms[2][1] = CONFIG_SDFM0_CHANNEL8_FD_WINDOW_SIZE;
            gTestSdfmPrms.fastDetectPrms[2][2] = CONFIG_SDFM0_CHANNEL8_FD_MAX_ZERO_COUNT;
            gTestSdfmPrms.fastDetectPrms[2][3] = CONFIG_SDFM0_CHANNEL8_FD_MIN_ZERO_COUNT;
#endif
#endif
            break;
        default:
            break;
    }

    *gSdfmPrms = gTestSdfmPrms;
}

void sdfmGlobalParamsConfig(SdfmPrms *gSdfmPrms)
{
    SdfmPrms gTestSdfmPrms = *gSdfmPrms;

    int32_t status;
   /* Configure Sdfm parameters */
    gTestSdfmPrms.loadShare = CONFIG_SDFM0_LOAD_SHARE;
    
    /*get iep & core clock values*/
    if(CONFIG_SDFM0_ICSSGx == ICSSG0_ID)
    {
        status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &gTestSdfmPrms.pruClock);
        DebugP_assert(status == SystemP_SUCCESS);
        status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_IEP_CLK, &gTestSdfmPrms.iepClock[0]);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    else
    {
        status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, &gTestSdfmPrms.pruClock);
        DebugP_assert(status == SystemP_SUCCESS);
        status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_IEP_CLK, &gTestSdfmPrms.iepClock[0]);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    
    gTestSdfmPrms.phaseDelay = CONFIG_SDFM0_PHASE_DELAY;

    *gSdfmPrms = gTestSdfmPrms;
}

