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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/epwm.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <pruicss_pwm.h>
#include <drivers/pruicss.h>
#include "stdlib.h"
#include <drivers/pinmux.h>
#include "app_pruicss_pwm.h"
#include <board/ioexp/ioexp_tca6424.h>

/* Source Index of Icssg0 iep1 compare 0*/
#define TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX                (28U)
/* Source Index of Icssg1 iep1 compare 0*/
#define TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX                (28U)

/*---------------------------------------------------------------------------------------*/
/* TODO:                   sysconfig generation code starts here                         */
/*---------------------------------------------------------------------------------------*/

extern PRUICSS_Config gPruIcssConfig[];

uint32_t gPruIcssPwmConfigNum = 1;

PRUICSS_PWM_IEP_Attrs iepAttrs[] =
{
    {     
        .pruIcssIepClkFrequency = PRUICSS_IEP0_CLK_FREQ,
        .pruIcssIepClkPeriod = PRUICSS_IEP0_CLK_PERIOD_IN_NANOSECONDS,
        .iep0IncrementValue = PRUICSS_IEP_COUNT_INCREMENT_VALUE,
        .enableIep0 = 1U,
        .enableIEP0ShadowMode = 1U,
        .enableIep0ResetOnEpwm0_Sync = 1U,
        .enableIep0ResetOnEpwm3_Sync = 0U,
        .enableIep0ResetOnCompare0 = 1U,
        .enableIep1SlaveMode = 1U,
        .enableAutoClearCompareStatus = 1U
    }
};



/* PRUICSS driver configuration */
PRUICSS_PWM_Config gPruIcssPwmConfig[] =
{
    {
        .pruIcssHandle   = &gPruIcssConfig[CONFIG_PRU_ICSS0],
        .pwmAttrs        = {0},
        .iepAttrs        = &iepAttrs[CONFIG_PRUICSS_PWM_IEP_INSTANCE]
    },
};



/*-----------------------------------------------------------------------------------------*/
/*TODO:                    sysconfig generation code ends here                             */
/*-----------------------------------------------------------------------------------------*/

extern uint8_t gUpdateNextRisingEdgeCmpValue;

int32_t App_updateNextRisingEdgeCmpValue(PRUICSS_PWM_Handle handle)
{

    int status;
    int32_t retVal = SystemP_FAILURE;

    if((handle!=NULL))
    {
        /* compare0_val is calculated based on pwm period */
        uint32_t compare0_val = ((((handle->iepAttrs)->pruIcssIepClkFrequency)/((handle->iepAttrs)->pruIcssPwmFrequency))*((handle->iepAttrs)->iep0IncrementValue));

        /*div by 2 is done to update compare values half of the pwm period, as IEP cannot be configured in up-down mode*/
        compare0_val = (compare0_val)/2;

        uint32_t compare_val;
        uint8_t  currentPwmInstance, currentPwmSet;

        /*Next compare value which decides rising edge of PWM signal are computed & shadow compare register field is updated*/
        for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
        {
            for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
            {
            if(handle->pwmAttrs[currentPwmSet][currentPwmInstance].enable == 1 )
            {

                    compare_val = compare0_val - (((handle->pwmAttrs[currentPwmSet][currentPwmInstance].dutyCycle)*(compare0_val))/100);

                    compare_val = compare_val + (((handle->pwmAttrs[currentPwmSet][currentPwmInstance].riseEdgeDelay)*((handle->iepAttrs)->iep0IncrementValue))/((handle->iepAttrs)->pruIcssIepClkPeriod));

                    /*configure cmp  value of current pwm*/
                    status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, handle->pwmAttrs[currentPwmSet][currentPwmInstance].iepInstance, handle->pwmAttrs[currentPwmSet][currentPwmInstance].compareEvent, compare_val);
                    DebugP_assert(SystemP_SUCCESS == status);
            }

            }
        }

        /*configure cmp  value of current pwm*/
        status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST0, CMP_EVENT0, compare0_val + 1);
        DebugP_assert(SystemP_SUCCESS == status);

        status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST1, CMP_EVENT0, compare0_val + 1);
        DebugP_assert(SystemP_SUCCESS == status);
        
        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t App_sciclientCmpEventRouterIrqset(PRUICSS_PWM_Handle handle)
{
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        PRUICSS_HwAttrs const   *hwAttrs;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->pruIcssHandle->hwAttrs;

        int32_t                             retVal;
        struct tisci_msg_rm_irq_set_req     rmIrqReq;
        struct tisci_msg_rm_irq_set_resp    rmIrqResp;
        rmIrqReq.valid_params           = 0U;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.global_event           = 0U;

        if(hwAttrs->instance == 0)
        {
            rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG0;
            rmIrqReq.src_index              = TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX;
        }
        else if(hwAttrs->instance == 1)
        {
            rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG1;
            rmIrqReq.src_index              = TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX;
        }

        rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
        rmIrqReq.dst_host_irq           = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            retVal = SystemP_FAILURE;
            return retVal;
        }
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t App_sciclientCmpEventRouterIrqRelease(PRUICSS_PWM_Handle handle)
{
    struct tisci_msg_rm_irq_release_req     rmIrqReq;

    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        PRUICSS_HwAttrs const   *hwAttrs;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->pruIcssHandle->hwAttrs;

        rmIrqReq.valid_params           = 0U;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params          |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.global_event           = 0U;
        if(hwAttrs->instance == 0)
        {
            rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG0;
            rmIrqReq.src_index              = TISCI_PRU_ICSSG0_IEP1_CMP0_SRC_INDEX;
        }
        else if(hwAttrs->instance == 1)
        {
            rmIrqReq.src_id                 = TISCI_DEV_PRU_ICSSG1;
            rmIrqReq.src_index              = TISCI_PRU_ICSSG1_IEP1_CMP0_SRC_INDEX;
        }
        rmIrqReq.dst_id                 = TISCI_DEV_R5FSS0_CORE0;
        rmIrqReq.dst_host_irq           = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        retVal = Sciclient_rmIrqRelease(&rmIrqReq, SystemP_WAIT_FOREVER);
        if(0 != retVal)
        {
            retVal = SystemP_FAILURE;
            return retVal;
        }
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t App_pruicssIep1Compare0IrqSet(PRUICSS_PWM_Handle handle)
{
    HwiP_Params     hwiPrms;
    HwiP_Object     compHwiObject;

    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        retVal = App_sciclientCmpEventRouterIrqset(handle);
    }
    else
    {
        return retVal;
    }

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = CSLR_R5FSS0_CORE0_INTR_CMP_EVENT_INTROUTER0_OUTP_16;
    hwiPrms.callback = &App_pruIcssPwmHalfDoneIrq;
    retVal = HwiP_construct(&compHwiObject, &hwiPrms);
    return retVal;
}

int32_t App_epwm0Sync0IrqSet(PRUICSS_PWM_Handle handle, uint32_t epwmBaseAddr)
{

    HwiP_Params     hwiPrms;
    HwiP_Object     compHwiObject;

    AppEpwmSync0IrqArgs_t *AppEpwmSync0IrqArgs;

    AppEpwmSync0IrqArgs = (AppEpwmSync0IrqArgs_t*)malloc(sizeof(AppEpwmSync0IrqArgs_t));

    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        AppEpwmSync0IrqArgs->EpwmBaseAddr = epwmBaseAddr;
        AppEpwmSync0IrqArgs->handle = handle;
    }
    else
    {
        return retVal; 
    }

    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = CONFIG_EPWM0_INTR;
    hwiPrms.callback = &App_epwmSync0Irq;
    hwiPrms.args     = (void *)(AppEpwmSync0IrqArgs);
    hwiPrms.isPulse     = CONFIG_EPWM0_INTR_IS_PULSE;
    retVal = HwiP_construct(&compHwiObject, &hwiPrms);
    return retVal; 
}

void App_pruIcssPwmHalfDoneIrq(void *args)
{
    gUpdateNextRisingEdgeCmpValue  = 1;
}

void App_epwmSync0Irq(void *args)
{
    volatile uint16_t status;

    AppEpwmSync0IrqArgs_t *AppEpwmSync0IrqArgs = (AppEpwmSync0IrqArgs_t *)args;

    status = EPWM_etIntrStatus(AppEpwmSync0IrqArgs->EpwmBaseAddr);
    if(status & EPWM_ETFLG_INT_MASK)
    {
        EPWM_etIntrClear(AppEpwmSync0IrqArgs->EpwmBaseAddr);
    }

    /*Change state of all pwm signals to intial*/
    PRUICSS_PWM_changePwmSetToIntialState(AppEpwmSync0IrqArgs->handle, 0xF);

    /* compare0_val is calculated based on pwm period */
    uint32_t compare0_val = (((AppEpwmSync0IrqArgs->handle)->iepAttrs)->pruIcssIepClkFrequency)/((((AppEpwmSync0IrqArgs->handle)->iepAttrs)->pruIcssPwmFrequency)*((AppEpwmSync0IrqArgs->handle)->iepAttrs)->iep0IncrementValue);

    /*div by 2 is done to update compare values half of the pwm period, as IEP cannot be configured in up-down mode*/
    compare0_val = (compare0_val)/2;

    uint32_t compare_val;
    uint8_t  currentPwmInstance, currentPwmSet;

    /*Next compare value which decides fall edge of PWM signal are computed & shadow compare register field is updated*/
    for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
    {
        for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
        {
           if(((AppEpwmSync0IrqArgs->handle)->pwmAttrs[currentPwmSet][currentPwmInstance]).enable == 1)
           {

                compare_val = (((((AppEpwmSync0IrqArgs->handle)->pwmAttrs[currentPwmSet][currentPwmInstance]).dutyCycle)*(compare0_val))/100);

                compare_val = compare_val + (((AppEpwmSync0IrqArgs->handle->pwmAttrs[currentPwmSet][currentPwmInstance].fallEdgeDelay)*(((AppEpwmSync0IrqArgs->handle->iepAttrs)->iep0IncrementValue)))/((AppEpwmSync0IrqArgs->handle->iepAttrs)->pruIcssIepClkPeriod));

                /*configure cmp  value of current pwm*/
                status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(AppEpwmSync0IrqArgs->handle, (AppEpwmSync0IrqArgs->handle->pwmAttrs[currentPwmSet][currentPwmInstance]).iepInstance, (AppEpwmSync0IrqArgs->handle->pwmAttrs[currentPwmSet][currentPwmInstance]).compareEvent, compare_val);
           }

        }
    }
}
