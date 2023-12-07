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

#include "tisdfm_pruss_intc_mapping.h"  /* INTC configuration */
#include "current_sense/sdfm/firmware/sdfm_bin.h"            /* SDFM image data */
#include "sdfm.h"
#include "current_sense/sdfm/include/sdfm_api.h"
/* PRU SDFM FW image info */
typedef struct PRUSDFM_PruFwImageInfo_s {
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDFM_PruFwImageInfo;

/* Number of PRU images */
#define PRU_SDFM_NUM_PRU_IMAGE  ( 3 )

/* PRU SDFM image info */
static PRUSDFM_PruFwImageInfo gPruFwImageInfo[PRU_SDFM_NUM_PRU_IMAGE] =
{
    {pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0)}, /* PRU FW */
    {NULL, 0}
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
        if (status != SystemP_SUCCESS) {
            return SDFM_ERR_INIT_ICSSG;
        }
    }
    else if (sliceId == ICSSG_SLICE_ID_1)
    {
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU1);
        if (status != SystemP_SUCCESS) {
            return SDFM_ERR_INIT_ICSSG;
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
void sdfm_configure_gpio_pin(sdfm_handle h_sdfm)
{
    /*ch0 GPIO configuration*/
    uint32_t gpioBaseAddrCh0Hi = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_HIGH_TH_CH0_BASE_ADDR);
    uint32_t pinNumCh0Hi       = GPIO_HIGH_TH_CH0_PIN;
    GPIO_setDirMode(gpioBaseAddrCh0Hi, pinNumCh0Hi, GPIO_HIGH_TH_CH0_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 0, gpioBaseAddrCh0Hi, pinNumCh0Hi, 0);

    uint32_t gpioBaseAddrCh0Lo = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LOW_TH_CH0_BASE_ADDR);
    uint32_t pinNumCh0Lo       = GPIO_LOW_TH_CH0_PIN;
    GPIO_setDirMode(gpioBaseAddrCh0Lo, pinNumCh0Lo, GPIO_LOW_TH_CH0_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 0, gpioBaseAddrCh0Lo, pinNumCh0Lo, 1);


    /*ch1 GPIO configuration*/
    uint32_t gpioBaseAddrCh1Hi = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_HIGH_TH_CH1_BASE_ADDR);
    uint32_t pinNumCh1Hi       = GPIO_HIGH_TH_CH1_PIN;
    GPIO_setDirMode(gpioBaseAddrCh1Hi, pinNumCh1Hi, GPIO_HIGH_TH_CH1_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 1, gpioBaseAddrCh1Hi, pinNumCh1Hi, 0);

    uint32_t gpioBaseAddrCh1Lo = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LOW_TH_CH1_BASE_ADDR);
    uint32_t pinNumCh1Lo       = GPIO_LOW_TH_CH1_PIN;
    GPIO_setDirMode(gpioBaseAddrCh1Lo, pinNumCh1Lo, GPIO_LOW_TH_CH1_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 1, gpioBaseAddrCh1Lo, pinNumCh1Lo, 1);


    /*ch2 GPIO configuration*/
    uint32_t gpioBaseAddrCh2Hi = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_HIGH_TH_CH2_BASE_ADDR);
    uint32_t pinNumCh2Hi       = GPIO_HIGH_TH_CH2_PIN;
    GPIO_setDirMode(gpioBaseAddrCh2Hi, pinNumCh2Hi, GPIO_HIGH_TH_CH2_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh2Hi, pinNumCh2Hi, 0);

    uint32_t gpioBaseAddrCh2Lo = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LOW_TH_CH2_BASE_ADDR);
    uint32_t pinNumCh2Lo       = GPIO_LOW_TH_CH2_PIN;
    GPIO_setDirMode(gpioBaseAddrCh2Lo, pinNumCh2Lo, GPIO_LOW_TH_CH2_DIR);
    SDFM_configComparatorGpioPins(h_sdfm, 2, gpioBaseAddrCh2Lo, pinNumCh2Lo, 1);


}
/* Initialize SDFM PRU FW */
int32_t init_sdfm_pru_fw(uint8_t pruId, SdfmPrms *pSdfmPrms, sdfm_handle *pHSdfm, void *pruss_cfg)
{
    sdfm_handle hSdfm;

    /* Initialize SDFM instance */
    hSdfm = SDFM_init(pruId);
    
    uint32_t i;
    i = SDFM_getFirmwareVersion(hSdfm);
    DebugP_log("\n\n\n");
    DebugP_log("SDFM firmware version \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");
    if (hSdfm == NULL)
    {
        return SDFM_ERR_INIT_SDFM;
    }

    uint8_t SDFM_CH;
    hSdfm->iep_clock = pSdfmPrms->iep_clock; 
    hSdfm->sdfm_clock = pSdfmPrms->sd_clock; 
    hSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)(pSdfmPrms->samplesBaseAddress);
    uint32_t sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW(pSdfmPrms->samplesBaseAddress);
    hSdfm->p_sdfm_interface->sampleBufferBaseAdd = sampleOutputInterfaceGlobalAddr;
    hSdfm->iep_inc = 1; /* Default IEP increment 1 */
    hSdfm->pruss_cfg = pruss_cfg;


    uint8_t acc_filter = 0; //SINC3 filter
    uint8_t ecap_divider = 0x0F; //IEP at 300MHz: SD clock = 300/15=20Mhz

    /*configure IEP count for one epwm period*/
    SDFM_configIepCount(hSdfm, pSdfmPrms->epwm_out_freq);

    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
    SDFM_configEcap(hSdfm, ecap_divider);

    /*set Noraml current OSR */
    SDFM_setFilterOverSamplingRatio(hSdfm, pSdfmPrms->FilterOsr);
     

    /*below configuration for all three channel*/
    for(SDFM_CH = 0; SDFM_CH < NUM_CH_SUPPORTED; SDFM_CH++)
    {
        SDFM_setEnableChannel(hSdfm, SDFM_CH);

        /*set comparator osr or Over current osr*/
        SDFM_setCompFilterOverSamplingRatio(hSdfm, SDFM_CH, pSdfmPrms->ComFilterOsr);

        /*set ACC source or filter type*/
        SDFM_configDataFilter(hSdfm, SDFM_CH, acc_filter);

        /*set clock inversion & clock source for all three channel*/
        SDFM_selectClockSource(hSdfm, SDFM_CH, pSdfmPrms->clkPrms[SDFM_CH]);

        /*set threshold values */
        SDFM_setCompFilterThresholds(hSdfm, SDFM_CH, pSdfmPrms->threshold_parms[SDFM_CH]);
        if(pSdfmPrms->en_fd)
        {
            /*Fast detect configuration */
            SDFM_configFastDetect(hSdfm, SDFM_CH, pSdfmPrms->fastDetect[SDFM_CH]);
        }
        if(pSdfmPrms->en_com)
        {
            SDFM_enableComparator(hSdfm, SDFM_CH);       
        }
        else
        {
            SDFM_disableComparator(hSdfm, SDFM_CH);
        }

    }
    
    /*GPIO pin configuration for threshold measurment*/
    sdfm_configure_gpio_pin(hSdfm);

    SDFM_setSampleTriggerTime(hSdfm, pSdfmPrms->firstSampTrigTime);
    if(pSdfmPrms->en_second_update)
    {
        SDFM_enableDoubleSampling(hSdfm, pSdfmPrms->secondSampTrigTime);
    }
    else
    {
        SDFM_disableDoubleSampling(hSdfm);
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
    void *pruss_cfg;

    pruss_cfg = (void *)(((PRUICSS_HwAttrs *)(pruIcssHandle->hwAttrs))->cfgRegBase);
    /* Reset PRU */
    status = PRUICSS_resetCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) {
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
            pPruFwImageInfo = &gPruFwImageInfo[0];
            pruIMem = PRUICSS_IRAM_PRU(sliceId);
            break;
        case PRUICSS_RTU_PRU0:
        case PRUICSS_RTU_PRU1:
            pPruFwImageInfo = &gPruFwImageInfo[1];
            pruIMem = PRUICSS_IRAM_RTU_PRU(sliceId);
            break;
        case PRUICSS_TX_PRU0:
        case PRUICSS_TX_PRU1:
            pPruFwImageInfo = NULL;
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
    if (pruInstId == PRUICSS_PRU0) {
        pruId = PRU_ID_0;
    }
    else if (pruInstId == PRUICSS_PRU1) {
        pruId = PRU_ID_1;
    }
    else {
        return SDFM_ERR_INIT_PRU_SDFM;
    }

    /* Initialize SDFM PRU FW */
    status = init_sdfm_pru_fw(pruId, pSdfmPrms, pHSdfm, pruss_cfg);
    if (status != SDFM_ERR_NERR) {
        return SDFM_ERR_INIT_PRU_SDFM;
    }
    return SDFM_ERR_NERR;

}


