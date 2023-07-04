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

#include "tisddf_pruss_intc_mapping.h"  /* INTC configuration */
#include "motor_control/current_sense/sdfm/firmware/sdfm_bin.h"            /* SDDF image data */
#include "sddf.h"
#include "motor_control/current_sense/sdfm/include/sddf_api.h"
/* PRU SDDF FW image info */
typedef struct PRUSDDF_PruFwImageInfo_s {
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDDF_PruFwImageInfo;

/* Number of PRU images */
#define PRU_SDDF_NUM_PRU_IMAGE  ( 3 )

/* PRU SDDF image info */
static PRUSDDF_PruFwImageInfo gPruFwImageInfo[PRU_SDDF_NUM_PRU_IMAGE] =
{
    {pru_SDDF_PRU0_image_0, sizeof(pru_SDDF_PRU0_image_0)}, /* PRU FW */
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
        return SDDF_ERR_INIT_ICSSG;
    }

    /* Disable slice PRU cores */
    if (sliceId == ICSSG_SLICE_ID_0)
    {
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU0);
        if (status != SystemP_SUCCESS) {
            return SDDF_ERR_INIT_ICSSG;
        }
    }
    else if (sliceId == ICSSG_SLICE_ID_1)
    {
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_PRU1);
        if (status != SystemP_SUCCESS) {
            return SDDF_ERR_INIT_ICSSG;
        }
    }
    else
    {
        return SDDF_ERR_INIT_ICSSG;
    }

    /* Reset slice memories */
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_PRU(sliceId));
    if (size == 0)
    {
        return SDDF_ERR_INIT_ICSSG;
    }
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_DATARAM(sliceId));
    if (size == 0)
    {
        return SDDF_ERR_INIT_ICSSG;
    }

    /* Set ICSS pin mux */
    PRUICSS_setSaMuxMode(pruIcssHandle, saMuxMode);

    /* Initialize ICSS INTC */
    status = PRUICSS_intcInit(pruIcssHandle, &gPruicssIntcInitdata);
    if (status != SystemP_SUCCESS) {
        return SDDF_ERR_INIT_ICSSG;
    }

    *pPruIcssHandle = pruIcssHandle;

    return SDDF_ERR_NERR;
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
/* Initialize SDDF PRU FW */
int32_t init_sdfm_pru_fw(uint8_t pruId, SddfPrms *pSddfPrms, sdfm_handle *pHSddf)
{
    sdfm_handle hSddf;

    /* Initialize SDDF instance */
    hSddf = SDFM_init(pruId);
    if (hSddf == NULL)
    {
        return SDDF_ERR_INIT_SDDF;
    }

    uint8_t SDFM_CH;
    hSddf->iep_clock = 300000000; //300MHz
    hSddf->sdfm_clock = 20000000; //20MHz
    hSddf->iep_inc = 1; // Default IEP increment 1

    uint8_t acc_osr = 13;
    uint8_t acc_filter = 0; //SINC3 filter
    uint8_t ecap_divider = 0x0F; //IEP at 300MHz: SD clock = 300/15=20Mhz

    /*configure IEP count for one epwm period*/
    SDFM_configIepCount(hSddf, pSddfPrms->epwm_out_freq);

    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
    SDFM_configEcap(hSddf, ecap_divider);

     /*set comparator osr or OC osr*/
    SDFM_setCompFilterOverSamplingRatio(hSddf, pSddfPrms->ComFilterOsr);

    /*set OC sample count for NC  & NC OSR */
    SDFM_setFilterOverSamplingRatio(hSddf, pSddfPrms->FilterOsr, pSddfPrms->ComFilterOsr);

    /*below configuration for all three channel*/
    for(SDFM_CH = 0; SDFM_CH < NUM_CH_SUPPORTED; SDFM_CH++)
    {
        SDFM_setEnableChannel(hSddf, SDFM_CH);

        SDFM_setAccOverSamplingRatio(hSddf, SDFM_CH, acc_osr);

        /*set ACC source or filter type*/
        SDFM_configDataFilter(hSddf, SDFM_CH, acc_filter);

        /*set clock inversion & clock source for all three channel*/
        SDFM_selectClockSource(hSddf, SDFM_CH, pSddfPrms->clkPrms[SDFM_CH]);

        /*set threshold values */
        SDFM_setCompFilterThresholds(hSddf, SDFM_CH, pSddfPrms->threshold_parms[SDFM_CH]);



        if(pSddfPrms->en_com)
        {
            SDFM_enableComparator(hSddf, SDFM_CH);
        }
        else
        {
            SDFM_disableComparator(hSddf, SDFM_CH);
        }

    }

    /*GPIO pin configuration for threshold measurment*/
    sdfm_configure_gpio_pin(hSddf);

    SDFM_setSampleReadingTime(hSddf, pSddfPrms->trigSampTime);

    /* Enable (global) SDDF */
    SDFM_enable(hSddf);

    pHSddf = &hSddf;

 return SDDF_ERR_NERR;
}

/*
 *  ======== initPruSddf ========
 */
/* Initialize PRU core for SDDF */
int32_t initPruSddf(
    PRUICSS_Handle pruIcssHandle,
    uint8_t pruInstId,
    SddfPrms *pSddfPrms,
    sdfm_handle *pHSddf
)
{
    uint8_t sliceId;
    uint32_t pruIMem;
    PRUSDDF_PruFwImageInfo *pPruFwImageInfo;
    int32_t size;
    const uint32_t *sourceMem;          /* Source memory[ Array of uint32_t ] */
    uint32_t imemOffset;    /* Offset at which write will happen */
    uint32_t byteLen;                   /* Total number of bytes to be written */
    uint8_t pruId;
    int32_t status;

    /* Reset PRU */
    status = PRUICSS_resetCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) {
        return SDDF_ERR_INIT_PRU_SDDF;
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
        return SDDF_ERR_INIT_PRU_SDDF;
    }

    /* Write IMEM */
    imemOffset = 0;
    sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
    byteLen = pPruFwImageInfo->pruImemImgSz;
    size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
    if (size == 0)
    {
        return SDDF_ERR_INIT_PRU_SDDF;
    }

    /* Enable PRU */
    status = PRUICSS_enableCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) {
        return SDDF_ERR_INIT_PRU_SDDF;
    }
/* Translate PRU ID to SDDF API */
    if (pruInstId == PRUICSS_PRU0) {
        pruId = PRU_ID_0;
    }
    else if (pruInstId == PRUICSS_PRU1) {
        pruId = PRU_ID_1;
    }
    else {
        return SDDF_ERR_INIT_PRU_SDDF;
    }

    /* Initialize SDDF PRU FW */
    status = init_sdfm_pru_fw(pruId, pSddfPrms, pHSddf);
    if (status != SDDF_ERR_NERR) {
        return SDDF_ERR_INIT_PRU_SDDF;
    }
    return SDDF_ERR_NERR;

}


