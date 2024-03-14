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
#include <drivers/hw_include/hw_types.h>
#include <drivers/pruicss.h>
#include <drivers/sciclient.h>

#include "tisddf_pruss_intc_mapping.h"  /* INTC configuration */
#include "current_sense/sdfm/firmware/sdfm_pru_bin.h"            /* SDDF image data */
#include "current_sense/sdfm/firmware/sdfm_rtu_bin.h"
#include "sddf.h"
#include "current_sense/sdfm/include/sdfm_api.h"

/* PRU SDDF FW image info */
typedef struct PRUSDDF_PruFwImageInfo_s {
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDDF_PruFwImageInfo;

/* TISCI ICSSG Core clock selection options */
static uint32_t gIcssCoreClkSel[ICSSGn_CORE_CLK_SEL_NUMSEL] = {
    TISCI_DEV_PRU_ICSSG0_CORE_CLK_PARENT_HSDIV4_16FFT_MAIN_2_HSDIVOUT0_CLK,
    TISCI_DEV_PRU_ICSSG0_CORE_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT9_CLK
};
/* TISCI ICSSG IEP clock selection options */
static uint32_t gIcssIepClkSel[ICSSGn_IEP_CLK_SEL_NUMSEL] = {
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_POSTDIV4_16FF_MAIN_2_HSDIVOUT5_CLK,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT6_CLK,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_BOARD_0_CP_GEMAC_CPTS0_RFT_CLK_OUT,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_BOARD_0_CPTS0_RFT_CLK_OUT,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_BOARD_0_MCU_EXT_REFCLK0_OUT,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_BOARD_0_EXT_REFCLK1_OUT,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_WIZ16B2M4CT_MAIN_0_IP1_LN0_TXMCLK,
    TISCI_DEV_PRU_ICSSG0_IEP_CLK_PARENT_K3_PLL_CTRL_WRAP_MAIN_0_CHIP_DIV1_CLK_CLK
};

/* Number of PRU images */
#define PRU_SDDF_NUM_PRU_IMAGE  ( 3 )

/* PRU SDDF image info */
static PRUSDDF_PruFwImageInfo gPruFwImageInfo[PRU_SDDF_NUM_PRU_IMAGE] =
{
    {pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0)}, /* PRU FW */
    {pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0)}, /* RTU FW */
    {NULL, 0} 
};

/* ICSS INTC configuration */
static const PRUICSS_IntcInitData gPruicssIntcInitdata = PRUICSS_INTC_INITDATA;


/*
 *  ======== cfgIcssgClkCfg ========
 */
/* Configure ICSSG clock selection */
int32_t cfgIcssgClkCfg(
    uint8_t icssInstId,
    uint8_t coreClkSel,         /* ICSSG internal Core clock source select, ICSSG_CORE_SYNC_REG:CORE_VBUSP_SYNC_EN */
    uint8_t icssCoreClkSel,     /* ICSSG external Core clock source select, CTRLMMR_ICSSGn_CLKSEL:CORE_CLKSEL, (used if coreClkSel==0) */
    uint64_t icssCoreClkFreqHz, /* ICSSG external Core clock frequency, (used if !=0 && coreClkSel==0) */
    uint8_t iepClkSel,          /* ICSSG internal IEP clock source select, ICSSG_IEPCLK_REG:IEP_OCP_CLK_EN */
    uint8_t icssIepClkSel,      /* ICSSG internal IEP clock source select, CTRLMMR_ICSSGn_CLKSEL:IEP_CLKSEL, (used if iepClkSel==0) */
    uint64_t icssIepClkFreqHz   /* ICSSG external IEP clock frequency, (used if !=0 && icssIepClkSel==0) */
)
{
    const PRUICSS_HwAttrs *hwAttrs;
    uint32_t instance;
    CSL_IcssCfgRegs *pIcssCfgRegs;
    uint64_t moduleId, clockId;
    uint32_t numParents = 0U;
    uint32_t parent;
    uint32_t regVal;
    uint32_t ret;
    
    hwAttrs = PRUICSS_getAttrs(icssInstId);
    instance = hwAttrs->instance;

    if (instance == PRUICSS_INSTANCE_ONE) {
        pIcssCfgRegs = (CSL_IcssCfgRegs *)CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE;

        /* Select ICSSG external Core clock source */
        if (coreClkSel == CORE_CLK_SEL_ICSSGn_CORE_CLK) {
            moduleId = TISCI_DEV_PRU_ICSSG0;
            clockId = TISCI_DEV_PRU_ICSSG0_CORE_CLK;
            ret = Sciclient_pmGetModuleClkNumParent(moduleId,
                clockId,
                &numParents,
                SystemP_WAIT_FOREVER);
                                                    
            if ((ret == CSL_PASS) && (numParents > 1U)) {
                /* Get TISCI parent clock ID */
                if (icssCoreClkSel < ICSSGn_CORE_CLK_SEL_NUMSEL) {
                   parent = gIcssCoreClkSel[icssCoreClkSel];
                }
                else {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }

                /* Configure Core clock mux */
                ret = Sciclient_pmSetModuleClkParent(moduleId,
                    clockId,
                    parent,
                    SystemP_WAIT_FOREVER);
                if (ret != CSL_PASS) {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
                
                if (icssCoreClkFreqHz != 0) {
                    /* Configure Clock frequency */
                    ret = Sciclient_pmSetModuleClkFreq(moduleId, clockId, icssCoreClkFreqHz, TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE, SystemP_WAIT_FOREVER);
                    if (ret != CSL_PASS) {
                        return SDDF_ERR_CFG_ICSSG_CLKCFG;
                    }
                }
            }
            else {
                return SDDF_ERR_CFG_ICSSG_CLKCFG;
            }
        }

        /* Select ICSSG internal Core clock source */
        regVal = HW_RD_REG32(&pIcssCfgRegs->CORE_SYNC_REG);
        regVal &= ~CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_MASK;
        regVal |= (coreClkSel << CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_SHIFT) & CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->CORE_SYNC_REG, regVal);

        /* Select ICSSG external IEP clock source */
        if (iepClkSel == IEP_CLK_SEL_ICSSGn_IEP_CLK) {
            moduleId = TISCI_DEV_PRU_ICSSG0;
            clockId = TISCI_DEV_PRU_ICSSG0_IEP_CLK;
            ret = Sciclient_pmGetModuleClkNumParent(moduleId,
                clockId,
                &numParents,
                SystemP_WAIT_FOREVER);
                                                    
            if ((ret == CSL_PASS) && (numParents > 1U)) {
                /* Get TISCI parent clock ID */
                if (icssIepClkSel < ICSSGn_IEP_CLK_SEL_NUMSEL) {
                   parent = gIcssIepClkSel[icssIepClkSel];
                }
                else {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
                
                /* Configure Core clock mux */
                ret = Sciclient_pmSetModuleClkParent(moduleId,
                    clockId,
                    parent,
                    SystemP_WAIT_FOREVER);
                if (ret != CSL_PASS) {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
                
                if (icssIepClkFreqHz != 0) {
                    /* Configure Clock frequency */
                    ret = Sciclient_pmSetModuleClkFreq(moduleId, clockId, icssIepClkFreqHz, TISCI_MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE, SystemP_WAIT_FOREVER);                
                    if (ret != CSL_PASS) {
                        return SDDF_ERR_CFG_ICSSG_CLKCFG;
                    }
                }
            }
            else {
                return SDDF_ERR_CFG_ICSSG_CLKCFG;
            }
        }

        /* Select ICSSG internal IEP clock source */
        regVal = HW_RD_REG32(&pIcssCfgRegs->IEPCLK);
        regVal &= ~CSL_ICSSCFG_IEPCLK_OCP_EN_MASK;
        regVal |= (iepClkSel << CSL_ICSSCFG_IEPCLK_OCP_EN_SHIFT) & CSL_ICSSCFG_IEPCLK_OCP_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->IEPCLK, regVal);
    }
    else if (instance == PRUICSS_INSTANCE_TWO) {
        pIcssCfgRegs = (CSL_IcssCfgRegs *)CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE;

        /* Select ICSSG external Core clock source */
        if (coreClkSel == CORE_CLK_SEL_ICSSGn_CORE_CLK) {
            moduleId = TISCI_DEV_PRU_ICSSG1;
            clockId = TISCI_DEV_PRU_ICSSG1_CORE_CLK;
            ret = Sciclient_pmGetModuleClkNumParent(moduleId,
                clockId,
                &numParents,
                SystemP_WAIT_FOREVER);
                                                    
            if ((ret == CSL_PASS) && (numParents > 1U)) {
                if (icssCoreClkSel == ICSSGn_CORE_CLK_SEL_MAIN_PLL2_HSDIV0_CLKOUT) {
                    parent = TISCI_DEV_PRU_ICSSG1_CORE_CLK_PARENT_HSDIV4_16FFT_MAIN_2_HSDIVOUT0_CLK;
                }
                else if (icssCoreClkSel == ICSSGn_CORE_CLK_SEL_MAIN_PLL0_HSDIV9_CLKOUT) {
                    parent = TISCI_DEV_PRU_ICSSG1_CORE_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT9_CLK;
                }
                else {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
                
                ret = Sciclient_pmSetModuleClkParent(moduleId,
                    clockId,
                    parent,
                    SystemP_WAIT_FOREVER);
                if (ret != CSL_PASS) {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
            }
            else {
                return SDDF_ERR_CFG_ICSSG_CLKCFG;
            }
        }

        /* Select ICSSG internal Core clock source */
        regVal = HW_RD_REG32(&pIcssCfgRegs->CORE_SYNC_REG);
        regVal &= ~CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_MASK;
        regVal |= (coreClkSel << CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_SHIFT) & CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->CORE_SYNC_REG, regVal);

        /* Select ICSSG external IEP clock source */
        if (iepClkSel == IEP_CLK_SEL_ICSSGn_IEP_CLK) {
            moduleId = TISCI_DEV_PRU_ICSSG1;
            clockId = TISCI_DEV_PRU_ICSSG1_IEP_CLK;
            ret = Sciclient_pmGetModuleClkNumParent(moduleId,
                clockId,
                &numParents,
                SystemP_WAIT_FOREVER);
                                                    
            if ((ret == CSL_PASS) && (numParents > 1U)) {
                if (icssIepClkSel == ICSSGn_IEP_CLK_SEL_MAIN_PLL2_HSDIV5_CLKOUT) {
                    parent = TISCI_DEV_PRU_ICSSG1_IEP_CLK_PARENT_POSTDIV4_16FF_MAIN_2_HSDIVOUT5_CLK;
                }
                else if (icssIepClkSel == ICSSGn_IEP_CLK_SEL_MAIN_PLL0_HSDIV6_CLKOUT) {
                    parent = TISCI_DEV_PRU_ICSSG1_IEP_CLK_PARENT_POSTDIV4_16FF_MAIN_0_HSDIVOUT6_CLK;
                }
                else {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
                
                ret = Sciclient_pmSetModuleClkParent(moduleId,
                    clockId,
                    parent,
                    SystemP_WAIT_FOREVER);
                if (ret != CSL_PASS) {
                    return SDDF_ERR_CFG_ICSSG_CLKCFG;
                }
            }
            else {
                return SDDF_ERR_CFG_ICSSG_CLKCFG;
            }
        }

        /* Select ICSSG internal IEP clock source */
        regVal = HW_RD_REG32(&pIcssCfgRegs->IEPCLK);
        regVal &= ~CSL_ICSSCFG_IEPCLK_OCP_EN_MASK;
        regVal |= (iepClkSel << CSL_ICSSCFG_IEPCLK_OCP_EN_SHIFT) & CSL_ICSSCFG_IEPCLK_OCP_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->IEPCLK, regVal);
    }
    else {
        return SDDF_ERR_CFG_ICSSG_CLKCFG;
    }
   
    return SDDF_ERR_NERR;
}

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
    const PRUICSS_HwAttrs *hwAttrs;
    CSL_IcssCfgRegs *pIcssCfgRegs;    
    uint32_t instance;
    uint32_t regVal;
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
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_RTU_PRU0);
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
        status = PRUICSS_disableCore(pruIcssHandle, PRUICSS_RTU_PRU1);
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
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_RTU_PRU(sliceId));
    if (size == 0)
    {
        return SDDF_ERR_INIT_ICSSG;
    }
    size = PRUICSS_initMemory(pruIcssHandle, PRUICSS_IRAM_TX_PRU(sliceId));
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

    /* Enable SDDF load sharing */
    hwAttrs = PRUICSS_getAttrs(icssInstId);
    instance = hwAttrs->instance;
    
    /* Determine ICSSG instance */
    if (icssInstId == PRUICSS_INSTANCE_ONE)
    {
        pIcssCfgRegs = (CSL_IcssCfgRegs *)CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE;
    }
    else if (instance == PRUICSS_INSTANCE_TWO) 
    {
        pIcssCfgRegs = (CSL_IcssCfgRegs *)CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE;
    }
    else 
    {
        return SDDF_ERR_INIT_ICSSG;
    }
    /* Configure selected ICSSG slice for SD load sharing */
    if (sliceId == ICSSG_SLICE_ID_0)
    {
        regVal = HW_RD_REG32(&pIcssCfgRegs->SDPRU0CLKDIV);
        regVal |= CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_SHARE_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->SDPRU0CLKDIV, regVal);
    }
    else if (sliceId == ICSSG_SLICE_ID_1)
    {
        regVal = HW_RD_REG32(&pIcssCfgRegs->SDPRU1CLKDIV);
        regVal |= CSL_ICSSCFG_SDPRU1CLKDIV_PRU1_SD_SHARE_EN_MASK;
        HW_WR_REG32(&pIcssCfgRegs->SDPRU1CLKDIV, regVal);
    }
    else
    {
        return SDDF_ERR_INIT_ICSSG;
    }

    *pPruIcssHandle = pruIcssHandle;
    
    return SDDF_ERR_NERR;
}

/*
 *  ======== initPruSddf ========
 */
/* Initialize PRU core for SDDF */
int32_t initPruSddf(
    PRUICSS_Handle pruIcssHandle,
    uint8_t pruInstId, 
    SdfmPrms *pSddfPrms, 
    sdfm_handle  *pHSddf
)
{
    uint8_t sliceId;
    uint32_t pruIMem;
    PRUSDDF_PruFwImageInfo *pPruFwImageInfo;
    int32_t size;
    const uint32_t *sourceMem;          /* Source memory[ Array of uint32_t ] */
    uint32_t imemOffset;
    uint32_t byteLen;                   /* Total number of bytes to be written */
    uint8_t pruId; // FL: remove Host FW init since FW changing
    int32_t status;

    /* Reset PRU */
    status = PRUICSS_resetCore(pruIcssHandle, pruInstId);
    if (status != SystemP_SUCCESS) {
        return SDDF_ERR_INIT_PRU_SDDF;
    }
    
    /* Calculate slice ID */
    sliceId = pruInstId - (uint8_t)pruInstId/ICSSG_NUM_SLICE * ICSSG_NUM_SLICE;
    /* Determine PRU DMEM address */
    PRUICSS_DATARAM(sliceId);
    /* Determine PRU FW image and PRU IMEM address */
    switch (pruInstId)
    {
        case PRUICSS_PRU0:
        case PRUICSS_PRU1:
            pPruFwImageInfo = &gPruFwImageInfo[0];
            pruIMem = PRUICSS_IRAM_PRU(sliceId);
           // dmemOffset = PRU_ICSSG_SDDF_CTRL_BASE;
            break;
        case PRUICSS_RTU_PRU0:
        case PRUICSS_RTU_PRU1:
            pPruFwImageInfo = &gPruFwImageInfo[1];
            pruIMem = PRUICSS_IRAM_RTU_PRU(sliceId);
           // dmemOffset = RTU_ICSSG_SDDF_CTRL_BASE;
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
    
    /* Write DMEM */
    //sourceMem = (uint32_t *)pPruFwImageInfo->pPruDmemImg;
   // byteLen = pPruFwImageInfo->pruDmemImgSz;
    //size = PRUICSS_writeMemory(pruIcssHandle, pruDMem, dmemOffset, sourceMem, byteLen);
    //if (size == 0)
   // {
    ///    return SDDF_ERR_INIT_PRU_SDDF;
    //}

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

    /* Translate PRU ID to SDFM API */
    if (sliceId == PRUICSS_PRU0) {
        pruId = PRU_ID_0;
    }
    else if (sliceId == PRUICSS_PRU1) {
        pruId = PRU_ID_1;
    }
    else {
        return SDDF_ERR_INIT_PRU_SDDF;
    }
    
    /* Initialize SDDF PRU FW */
    status = initSddf(pruId, pruInstId, pSddfPrms, pHSddf, pruIcssHandle);
    if (status != SDDF_ERR_NERR) {
        return SDDF_ERR_INIT_PRU_SDDF;
    }
    return SDDF_ERR_NERR;
}

/* Initialize SDDF PRU FW */
int32_t initSddf(
    uint8_t pruId, uint8_t pruInstId,
    SdfmPrms *pSdfmPrms, 
    sdfm_handle *pHSdfm, PRUICSS_Handle pruIcssHandle
)
{
    sdfm_handle hSdfm;
    
    /* Initialize SDFM instance */
    hSdfm = SDFM_init(pruIcssHandle, pruId, pruInstId);
    if (hSdfm == NULL) {
        return SDDF_ERR_INIT_SDDF;
    }
    uint32_t i;
    i = SDFM_getFirmwareVersion(hSdfm);
    DebugP_log("\n\n\n");
    DebugP_log("SDFM firmware version \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");
    if (hSdfm == NULL)
    {
        return SDDF_ERR_INIT_SDDF;
    }

    uint8_t SDFM_CH;
    hSdfm->iepClock = pSdfmPrms->iep_clock;
    hSdfm->sdfmClock = pSdfmPrms->sd_clock;
    hSdfm->sampleOutputInterface = (SDFM_SampleOutInterface *)(pSdfmPrms->samplesBaseAddress);
    uint32_t sampleOutputInterfaceGlobalAddr = CPU0_BTCM_SOCVIEW(pSdfmPrms->samplesBaseAddress);
    hSdfm->p_sdfm_interface->sampleBufferBaseAdd = sampleOutputInterfaceGlobalAddr;
    hSdfm->iepInc = 1; /* Default IEP increment 1 */


    uint8_t acc_filter = 0; //SINC3 filter
    uint8_t ecap_divider = 0x0F; //IEP at 300MHz: SD clock = 300/15=20Mhz

    /*configure IEP count for one epwm period*/
    SDFM_configIepCount(hSdfm, pSdfmPrms->epwm_out_freq);

    /*configure ecap as PWM code for generate 20 MHz sdfm clock*/
    SDFM_configEcap(hSdfm, ecap_divider);

    /*set Noraml current OSR */
    SDFM_setFilterOverSamplingRatio(hSdfm, pSdfmPrms->FilterOsr);
     

    /*below configuration for all three channel*/
    switch (pruInstId)
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
       
       for(int i = SDFM_CH; i< SDFM_CH + NUM_CH_SUPPORTED_PER_AXIS; i++)
       {
         SDFM_setEnableChannel(hSdfm, i);
       }

    for(SDFM_CH = 0; SDFM_CH < NUM_CH_SUPPORTED_PER_AXIS; SDFM_CH++)
    {
       

        /*set comparator osr or Over current osr*/
        SDFM_setCompFilterOverSamplingRatio(hSdfm, SDFM_CH, pSdfmPrms->ComFilterOsr);

        /*set ACC source or filter type*/
        SDFM_configDataFilter(hSdfm, SDFM_CH, acc_filter);

        /*set clock inversion & clock source for all three channel*/
        SDFM_selectClockSource(hSdfm, SDFM_CH, pSdfmPrms->clkPrms[SDFM_CH]);

        /*set threshold values */
        SDFM_setCompFilterThresholds(hSdfm, SDFM_CH, pSdfmPrms->threshold_parms[SDFM_CH]);
        
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
    ///sdfm_configure_gpio_pin(hSdfm);

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
    
    return SDDF_ERR_NERR;
}
