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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include<stdio.h>
#include<stdint.h>
#include<math.h>
#include <drivers/pruicss.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/tistdtypes.h>
#include <kernel/dpl/ClockP.h>
#include "endat3_periodic_trigger.h"
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#ifndef SOC_AM243X
/* ICSSM Interrupt Numbers */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSM0_PR1_HOST_INTR_PEND_0)
#endif
#else
/* ICSSG Interrupt Numbers */
#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG1_PR1_HOST_INTR_PEND_0)
#else
#define ICSS_PRU_ENDAT3_INT_NUM         (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_0)
#endif
#endif

#define IEP_CMP_EVENT                   (3)
/** \brief PRU interrupt event number (18 = 2 + 16) */
#define PRU_TRIGGER_HOST_ENDAT3_EVT     (2+16)    /* pr0_pru_mst_intr[2]_intr_req */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static HwiP_Object gEndat3HwiObject[CONFIG_ENDAT3_NUM_INSTANCES];
uint32_t gPruEndat3IrqCnt[CONFIG_ENDAT3_NUM_INSTANCES] = {0};

#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void endat3_config_iep(endat3_periodic_interface *endat3_periodic_interface);

static void endat3_interrupt_config(endat3_periodic_interface *endat3_periodic_interface);

void endat3_pru_irq_handler(void *pruicss_handle);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void endat3_config_iep(endat3_periodic_interface *endat3_periodic_interface)
{
    endat3_priv *priv = endat3_get_priv(endat3_periodic_interface->handle[CONFIG_ENDAT3_0]);
    void *pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    uint8_t temp;
    uint32_t event;
    uint32_t event_clear;
    uint32_t cmp_reg0;
    uint32_t cmp_reg1;
    uint64_t iep_reset_count = 0;

    /* Clear IEP */
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    event = HW_RD_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /* Enable IEP reset by cmp0 event */
    event |= IEP_CMP0_ENABLE;
    event |= IEP_RST_CNT_EN;
    event_clear |= 1;

    /* Set IEP counter to ZERO */
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG0, 0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_COUNT_REG1, 0);

    /* Configure CMP based on periodic_trigger_count of first handle */
    event |= (0x1 << (IEP_CMP_EVENT + 1));
    event_clear |= (0x1 << (IEP_CMP_EVENT));
    cmp_reg0 = (endat3_periodic_interface->periodic_trigger_count[CONFIG_ENDAT3_0] & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (endat3_periodic_interface->periodic_trigger_count[CONFIG_ENDAT3_0]>>32 & 0xffffffff);

    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + IEP_CMP_EVENT*8,  cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + IEP_CMP_EVENT*8,  cmp_reg1);

    /* Clear event */
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /* Enable event */
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    iep_reset_count = endat3_periodic_interface->iep_reset_count;

    /* Configure cmp0 registers */
    cmp_reg0 = (iep_reset_count & 0xffffffff) - IEP_DEFAULT_INC;
    cmp_reg1 = (iep_reset_count>>32 & 0xffffffff);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0, cmp_reg0);
    HW_WR_REG32((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1, cmp_reg1);

    /* Write IEP default increment and IEP start */
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0x0F;
    temp |= 0x10;
    temp |= IEP_COUNTER_EN;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);
}

static void endat3_interrupt_config(endat3_periodic_interface *endat3_periodic_interface)
{
    endat3_priv *priv = endat3_get_priv(endat3_periodic_interface->handle[CONFIG_ENDAT3_0]);
    void *pruicss_handle = (void *)(priv->pruicss_handle);
    int32_t status;
    HwiP_Params hwi_params;

    /* Register and enable PRU FW interrupt */
    HwiP_Params_init(&hwi_params);
    hwi_params.intNum   = ICSS_PRU_ENDAT3_INT_NUM;
    hwi_params.callback = &endat3_pru_irq_handler;
    hwi_params.args     = pruicss_handle;
    hwi_params.isPulse  = FALSE;
    hwi_params.isFIQ    = FALSE;
    status              = HwiP_construct(&gEndat3HwiObject[CONFIG_ENDAT3_0], &hwi_params);
    DebugP_assert(status == SystemP_SUCCESS);
}

int32_t endat3_config_periodic_mode(endat3_periodic_interface *endat3_periodic_interface)
{
    int32_t         status;
    endat3_priv   *priv;
    void            *pruicss_handle;

    /* NULL check on interface pointer and handle */
    if(endat3_periodic_interface == NULL || endat3_periodic_interface->handle[CONFIG_ENDAT3_0] == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = endat3_get_priv(endat3_periodic_interface->handle[CONFIG_ENDAT3_0]);
    pruicss_handle = (void *)(priv->pruicss_handle);

    /* Configure IEP */
    endat3_config_iep(endat3_periodic_interface);

#if(CONFIG_ENDAT3_0_PRUICSS_INSTANCE == 1)
    status = PRUICSS_intcInit(pruicss_handle, &icss1_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }
#else
    status = PRUICSS_intcInit(pruicss_handle, &icss0_intc_initdata);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }
#endif
    /* Configure Interrupts */
    endat3_interrupt_config(endat3_periodic_interface);
    return SystemP_SUCCESS;
}

int32_t endat3_stop_periodic_mode(endat3_periodic_interface *endat3_periodic_interface)
{
    endat3_priv *priv;
    void *pruicss_iep;
    uint8_t temp;

    /* NULL check on interface pointer and handle */
    if(endat3_periodic_interface == NULL || endat3_periodic_interface->handle[CONFIG_ENDAT3_0] == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = endat3_get_priv(endat3_periodic_interface->handle[CONFIG_ENDAT3_0]);
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);

    /* Stop IEP */
    temp = HW_RD_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;
    HW_WR_REG8((uint8_t *)pruicss_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    HwiP_destruct(&gEndat3HwiObject[CONFIG_ENDAT3_0]);

    return SystemP_SUCCESS;
}

/* PRU FW IRQ handler */
void endat3_pru_irq_handler(void *pruicss_handle)
{
    /* Increment IRQ count */
    gPruEndat3IrqCnt[CONFIG_ENDAT3_0]++;

    /* Clear interrupt at source */
    PRUICSS_clearEvent((PRUICSS_Handle)pruicss_handle, PRU_TRIGGER_HOST_ENDAT3_EVT);
}