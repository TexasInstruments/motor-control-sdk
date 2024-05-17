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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stddef.h>
#include <stdint.h>
#include <drivers/pruicss.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <pruicss_pwm/include/pruicss_pwm.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern int32_t gPruIcssPwmConfigNum;
extern PRUICSS_PWM_Config gPruIcssPwmConfig[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */



PRUICSS_PWM_Handle PRUICSS_PWM_open(uint32_t index, PRUICSS_Handle pruIcssHandle)
{
    PRUICSS_PWM_Handle  handle = NULL;

    /* Check index */
    if(index >= gPruIcssPwmConfigNum || (pruIcssHandle == NULL))
    {
        return NULL;
    }
    else
    {
        handle = (PRUICSS_PWM_Handle)(&gPruIcssPwmConfig[index]);
        handle->pruIcssHandle = pruIcssHandle;
    }

    return handle;
}

int32_t PRUICSS_PWM_setIepCounterLower_32bitValue(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint32_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_COUNT_REG_MAX))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0_COUNT_LO, value);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG0_COUNT_LO, value);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_setIepCounterUpper_32bitValue(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint32_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_COUNT_REG_MAX))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1_COUNT_HI, value);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_COUNT_REG1_COUNT_HI, value);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_configureIepShadowModeEnable(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG ),
                              CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_SHADOW_EN, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG ),
                              CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG_SHADOW_EN, enable);
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_configureIepCmp0ResetEnable(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG ),
                              CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG ),
                              CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN, enable);
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_configureIepCompareEnable(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint16_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_CMP_EVENTS_ENABLE_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP_EN, value);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP_CFG_REG_CMP_EN, value);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_setIepCompareEventLower_32bitValue(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (cmpEvent < PRUICSS_NUM_IEP_CMP_EVENTS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch(iepInstance)
        {
            case 0:
                if(cmpEvent<8)
                {
                    HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + cmpEvent*8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                else
                {
                    HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + cmpEvent*8 + 8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                break;
            case 1:
                if(cmpEvent<8)
                {
                    HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + cmpEvent*8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                else
                {
                    HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0 + cmpEvent*8 + 8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                break;
        }

    } 
    
    return retVal;
}



int32_t PRUICSS_PWM_setIepCompareEventUpper_32bitValue(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (cmpEvent < PRUICSS_NUM_IEP_CMP_EVENTS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch(iepInstance)
        {
            case 0:
                if(cmpEvent<8)
                {
                    HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1 + cmpEvent*8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                else
                {
                    HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1 + cmpEvent*8 + 8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                break;
            case 1:
                if(cmpEvent<8)
                {
                    HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + cmpEvent*8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                else
                {
                    HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1 + cmpEvent*8 + 8),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                }
                break;
        }

    }
    
    return retVal;    
}

int32_t PRUICSS_PWM_setPwmDebounceValue(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (value <= PRUICSS_PWM_DEBOUNCE_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_DEBOUNCE_VALUE, value);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_DEBOUNCE_VALUE, value);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_DEBOUNCE_VALUE, value);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_DEBOUNCE_VALUE, value);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_setPwmTripMask(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint16_t maskValue)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (maskValue <= PRUICSS_PWM_TRIP_MASK_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_MASK, maskValue);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_MASK, maskValue);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_MASK, maskValue);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_MASK, maskValue);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_getPwmTripMask(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint16_t *maskValuePtr)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (maskValuePtr != NULL))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                (*maskValuePtr) = HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_MASK);
                break;
            case 1:
                (*maskValuePtr) = HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_MASK);
                break;
            case 2:
                (*maskValuePtr) = HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_MASK);
                break;
             case 3:
                (*maskValuePtr) = HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_MASK);
                break;
        }
    }

    return retVal;    
}



int32_t PRUICSS_PWM_configurePwmCmp0TripResetEnable(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_CMP0_EN, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_CMP0_EN, enable);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_CMP0_EN, enable);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_CMP0_EN, enable);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_generatePwmTripReset(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_RESET, 1);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_RESET, 1);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_RESET, 1);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_RESET, 1);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_generatePwmOverCurrentErrorTrip(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_OVER_ERR_TRIP, 1);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_OVER_ERR_TRIP, 1);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_OVER_ERR_TRIP, 1);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_OVER_ERR_TRIP, 1);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_generatePwmPositionFeedbackErrorTrip(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_POS_ERR_TRIP, 1);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_POS_ERR_TRIP, 1);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_POS_ERR_TRIP, 1);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_POS_ERR_TRIP, 1);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_clearPwmTripResetStatus(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_RESET, 0);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_RESET, 0);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_RESET, 0);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_RESET, 0);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_clearPwmOverCurrentErrorTrip(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_OVER_ERR_TRIP, 0);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_OVER_ERR_TRIP, 0);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_OVER_ERR_TRIP, 0);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_OVER_ERR_TRIP, 0);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_clearPwmPositionFeedbackErrorTrip(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_POS_ERR_TRIP, 0);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_POS_ERR_TRIP, 0);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_POS_ERR_TRIP, 0);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_POS_ERR_TRIP, 0);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_getPwmTripTriggerCauseVector(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_VEC);
            case 1:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_VEC);
            case 2:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_VEC);
             case 3:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_VEC);
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_getPwmTripStatus(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (pwmSet)
        {
            case 0:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_MASK);
            case 1:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_MASK);
            case 2:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_MASK);
             case 3:
                return HW_RD_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_MASK);
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_clearPwmTripStatus(PRUICSS_PWM_Handle handle, uint8_t pwmSet)
{

    int32_t retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        retVal= PRUICSS_PWM_generatePwmTripReset(handle,pwmSet);
        if(retVal == SystemP_SUCCESS)
        {
            PRUICSS_PWM_clearPwmTripResetStatus(handle,pwmSet);
        }
    }

    return retVal; 
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);
        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_POS_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_POS_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_POS_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_POS_TRIP, action);
                        break;
                }
                break;
                

        }
    }

    return retVal; 
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);
        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0),
                               CSL_ICSSCFG_PWM0_0_PWM0_0_NEG_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0),
                               CSL_ICSSCFG_PWM1_0_PWM1_0_NEG_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0),
                               CSL_ICSSCFG_PWM2_0_PWM2_0_NEG_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_0),
                               CSL_ICSSCFG_PWM3_0_PWM3_0_NEG_TRIP, action);
                        break;
                }
                break;
                

        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);
        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_POS_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_POS_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_POS_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_POS_TRIP, action);
                        break;
                }
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);
        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1),
                               CSL_ICSSCFG_PWM0_1_PWM0_1_NEG_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1),
                               CSL_ICSSCFG_PWM1_1_PWM1_1_NEG_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1),
                               CSL_ICSSCFG_PWM2_1_PWM2_1_NEG_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_1),
                               CSL_ICSSCFG_PWM3_1_PWM3_1_NEG_TRIP, action);
                        break;
                }
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_POS_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_POS_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_POS_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_POS_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_POS_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_POS_TRIP, action);
                        break;
                }
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch(pwmSet)
        {
            case 0:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2),
                               CSL_ICSSCFG_PWM0_2_PWM0_2_NEG_TRIP, action);
                        break;
                }
                break;
            case 1:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2),
                               CSL_ICSSCFG_PWM1_2_PWM1_2_NEG_TRIP, action);
                        break;
                }
                break;
            case 2:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2),
                               CSL_ICSSCFG_PWM2_2_PWM2_2_NEG_TRIP, action);
                        break;
                }
                break;
            case 3:
                switch (state)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_NEG_INIT, action);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_NEG_ACT, action);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3_2),
                               CSL_ICSSCFG_PWM3_2_PWM3_2_NEG_TRIP, action);
                        break;
                }
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_PWM_configurePwmEfficiencyModeEnable(PRUICSS_PWM_Handle handle, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PIN_MX),CSL_ICSSCFG_PIN_MX_PWM_EFC_EN, enable);
    }
    return retVal;    
}

int32_t PRUICSS_PWM_enableIEP1Slave(PRUICSS_PWM_Handle handle, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_IEPCLK),
                      CSL_ICSSCFG_IEPCLK_IEP1_SLV_EN, 1);
    }
    return retVal;

}

int32_t PRUICSS_PWM_enableIEPResetOnEPWM0SyncOut(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM0_RST_CNT_EN, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP1_SLV_PWM_REG_PWM0_RST_CNT_EN, enable);
                break;
        }
    }

    return retVal;

}

int32_t PRUICSS_PWM_enableIEPResetOnEPWM3SyncOut(PRUICSS_PWM_Handle handle, uint8_t iepInstance, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((handle->pruIcssHandle)->hwAttrs);

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM3_RST_CNT_EN, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP1_SLV_PWM_REG_PWM3_RST_CNT_EN, enable);
                break;
        }
    }

    return retVal;

}


int32_t PRUICSS_PWM_attrsInit(PRUICSS_PWM_Handle handle)
{

    uint8_t compareEvent = CMP_EVENT1, iepInstance = PRUICSS_IEP_INST0, currentPwmInstance, currentPwmSet;

    int32_t retVal = SystemP_FAILURE;

    if(handle!=NULL){

        /*Intializes the pwm parameters with default values & maps compare events, output in intial, active, trip states and disables all pwm signals*/
        for(currentPwmSet=PRUICSS_PWM_SET0; currentPwmSet<=PRUICSS_PWM_SET1; currentPwmSet++)
        {
            for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
            {
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].iepInstance =  iepInstance;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].compareEvent = compareEvent;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgTripState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgActiveState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgInitialState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].fallEdgeDelay = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].riseEdgeDelay = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].dutyCycle = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].enable = 0;
                compareEvent++;
            }
        }

        compareEvent = CMP_EVENT1;
        iepInstance = PRUICSS_IEP_INST1;

        for(currentPwmSet=PRUICSS_PWM_SET2; currentPwmSet < PRUICSS_NUM_PWM_SETS; currentPwmSet++)
        {
            for(currentPwmInstance=0; currentPwmInstance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET; currentPwmInstance++)
            {
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].iepInstance =  iepInstance;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].compareEvent = compareEvent;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgTripState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgActiveState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].outputCfgInitialState = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].fallEdgeDelay = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].riseEdgeDelay = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].dutyCycle = 0;
                (handle->pwmAttrs)[currentPwmSet][currentPwmInstance].enable = 0;
                compareEvent++;
            }
        }

        retVal = SystemP_SUCCESS;
    }
    return retVal;  
}

int32_t PRUICSS_PWM_stateInit(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t instance, uint8_t outputCfgInitialState, uint8_t outputCfgActiveState, uint8_t outputCfgTripState)
{
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (instance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET))
    {
        (handle->pwmAttrs)[pwmSet][instance].outputCfgTripState = outputCfgTripState;
        (handle->pwmAttrs)[pwmSet][instance].outputCfgActiveState = outputCfgActiveState;
        (handle->pwmAttrs)[pwmSet][instance].outputCfgInitialState = outputCfgInitialState;
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_PWM_signalEnable(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t instance)
{
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (instance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET))
    {
        (handle->pwmAttrs)[pwmSet][instance].enable = 1;
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_PWM_config(PRUICSS_PWM_Handle handle, uint8_t pwmSet, uint8_t instance,  uint32_t dutyCycle, uint32_t riseEdgeDelay, uint32_t fallEdgeDelay)
{
    int32_t retVal = SystemP_FAILURE;
    /*Updates parameters of pwm signal with specified duty cycle, fall edge & rise edge delay*/
    if((handle!=NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (instance < PRUICSS_NUM_OF_PWMINSTANCES_PER_PWM_SET) && ((handle->pwmAttrs)[pwmSet][instance].enable == 1))
    {
        (handle->pwmAttrs)[pwmSet][instance].dutyCycle = dutyCycle;
        (handle->pwmAttrs)[pwmSet][instance].riseEdgeDelay = riseEdgeDelay;
        (handle->pwmAttrs)[pwmSet][instance].fallEdgeDelay = fallEdgeDelay;
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_PWM_stateConfig(PRUICSS_PWM_Handle handle)
{
    int status;
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        for(uint8_t currentPwmSet=0; currentPwmSet< PRUICSS_NUM_PWM_SETS; currentPwmSet++)
        {
            if((handle->pwmAttrs)[currentPwmSet][0].enable == 1)
            {

                /*configure PWM A0 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][0].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][0].outputCfgActiveState );
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][0].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }

            if((handle->pwmAttrs)[currentPwmSet][1].enable == 1)
            {
                /*configure PWM B0 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][1].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][1].outputCfgActiveState );
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][1].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }

            if((handle->pwmAttrs)[currentPwmSet][2].enable == 1)
            {
                /*configure PWM A1 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][2].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][2].outputCfgActiveState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][2].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }

            if((handle->pwmAttrs)[currentPwmSet][3].enable == 1)
            {
                /*configure PWM B1 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][3].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][3].outputCfgActiveState );
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][3].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }

            if((handle->pwmAttrs)[currentPwmSet][4].enable == 1)
            {
                /*configure PWM A2 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][4].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][4].outputCfgActiveState );
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][4].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }

            if((handle->pwmAttrs)[currentPwmSet][5].enable == 1)
            {
                /*configure PWM B2 signal of intial, active, trip states*/
                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_INTIAL_STATE, (handle->pwmAttrs)[currentPwmSet][5].outputCfgInitialState);
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_ACTIVE_STATE, (handle->pwmAttrs)[currentPwmSet][5].outputCfgActiveState );
                DebugP_assert(SystemP_SUCCESS == status);

                status = PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(handle, currentPwmSet, PRUICSS_PWM_TRIP_STATE, (handle->pwmAttrs)[currentPwmSet][5].outputCfgTripState);
                DebugP_assert(SystemP_SUCCESS == status);
            }
        }
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_PWM_changePwmSetToIntialState(PRUICSS_PWM_Handle handle, uint8_t pwmSetMask)
{
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        if(pwmSetMask & (1<<PRUICSS_PWM_SET0))
        {
            /*
            * generate trip reset status of pwm set 0
            */
            PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET0);
            /*
            * clear trip reset status of pwm set 0
            */
            PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET0);
        }    

        if(pwmSetMask & (1<<PRUICSS_PWM_SET1))
        {
            /*
            * generate trip reset status of pwm set 1
            */
            PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET1);
            /*
            * clear trip reset status of pwm set 1
            */
            PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET1);
        }  

        if(pwmSetMask & (1<<PRUICSS_PWM_SET2))
        {
            /*
            * generate trip reset status of pwm set 2
            */
            PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET2);
            /*
            * clear trip reset status of pwm set 2
            */
            PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET2);
        } 

        if(pwmSetMask & (1<<PRUICSS_PWM_SET3))
        {
            /*
            * generate trip reset status of pwm set 3
            */
            PRUICSS_PWM_generatePwmTripReset(handle,   PRUICSS_PWM_SET3);
            /*
            * clear trip reset status of pwm set 3
            */
            PRUICSS_PWM_clearPwmTripResetStatus(handle, PRUICSS_PWM_SET3);
        } 
        retVal = SystemP_SUCCESS;
    } 
    return retVal; 
}

int32_t PRUICSS_PWM_pruIcssPwmFrequencyInit(PRUICSS_PWM_Handle handle, uint32_t pruIcssPwmFrequency)
{
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        (handle->iepAttrs)->pruIcssPwmFrequency = pruIcssPwmFrequency;
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_PWM_iepConfig(PRUICSS_PWM_Handle handle)
{
    int status;
    int32_t retVal = SystemP_FAILURE;
    if((handle!=NULL))
    {
        /* compare0_val is calculated based on pwm period */
        uint32_t compare0_val = (float)((((handle->iepAttrs)->pruIcssIepClkFrequency *((handle->iepAttrs)->iep0IncrementValue)))/((handle->iepAttrs)->pruIcssPwmFrequency));

        /*Disable IEP0 counter*/
        status= PRUICSS_controlIepCounter((handle->pruIcssHandle), PRUICSS_IEP_INST0, 0);
        DebugP_assert(SystemP_SUCCESS == status);

        /*Intialize IEP0 count value*/
        status= PRUICSS_PWM_setIepCounterLower_32bitValue(handle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
        DebugP_assert(SystemP_SUCCESS == status);
        status= PRUICSS_PWM_setIepCounterUpper_32bitValue(handle, PRUICSS_IEP_INST0, 0xFFFFFFFF);
        DebugP_assert(SystemP_SUCCESS == status);

        if(((handle->iepAttrs)->enableIep0) == 1U)
        {

            /*Enable or disable shadow mode of IEP*/
            status=PRUICSS_PWM_configureIepShadowModeEnable(handle, PRUICSS_IEP_INST0, (handle->iepAttrs)->enableIEP0ShadowMode);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable or disable EPWM0 sync out to reset IEP*/
            status = PRUICSS_PWM_enableIEPResetOnEPWM0SyncOut(handle, PRUICSS_IEP_INST0, (handle->iepAttrs)->enableIep0ResetOnEpwm0_Sync);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable or disable EPWM3 sync out to reset IEP*/
            status = PRUICSS_PWM_enableIEPResetOnEPWM3SyncOut(handle, PRUICSS_IEP_INST0, (handle->iepAttrs)->enableIep0ResetOnEpwm3_Sync);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Enable cmp 0 reset of IEP0 counter*/
            status = PRUICSS_PWM_configureIepCmp0ResetEnable(handle, PRUICSS_IEP_INST0, (handle->iepAttrs)->enableIep0ResetOnCompare0);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Set IEP0 counter Increment value*/
            status = PRUICSS_setIepCounterIncrementValue((handle->pruIcssHandle), PRUICSS_IEP_INST0, (handle->iepAttrs)->iep0IncrementValue);
            DebugP_assert(SystemP_SUCCESS == status);

            /*Configure IEP0 compare 0 event to reset on (pruicss pwm period/2) with one clock cycle delay*/
            status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST0, CMP_EVENT0, ((compare0_val/2) +1));
            DebugP_assert(SystemP_SUCCESS == status);

            if((handle->iepAttrs)->enableIep1SlaveMode == 1U)
            {
                /*Disable IEP1 counter*/
                status= PRUICSS_controlIepCounter((handle->pruIcssHandle), PRUICSS_IEP_INST1, 0);
                DebugP_assert(SystemP_SUCCESS == status);

                /*Intialize IEP0 count value*/
                status= PRUICSS_PWM_setIepCounterLower_32bitValue(handle, PRUICSS_IEP_INST1, 0xFFFFFFFF);
                DebugP_assert(SystemP_SUCCESS == status);
                status= PRUICSS_PWM_setIepCounterUpper_32bitValue(handle, PRUICSS_IEP_INST1, 0xFFFFFFFF);
                DebugP_assert(SystemP_SUCCESS == status);

                /*Configure IEP1 compare 0 event to reset on (pruicss pwm period/2) with one clock cycle delay*/
                status = PRUICSS_PWM_setIepCompareEventUpper_32bitValue(handle, PRUICSS_IEP_INST1, CMP_EVENT0, ((compare0_val/2) +1));
                DebugP_assert(SystemP_SUCCESS == status);

                /*Enable  IEP1 slave mode*/
                status = PRUICSS_PWM_enableIEP1Slave(handle, (handle->iepAttrs)->enableIep1SlaveMode);
                DebugP_assert(SystemP_SUCCESS == status);

                /*Enable shadow mode of IEP1*/
                status=PRUICSS_PWM_configureIepShadowModeEnable(handle, PRUICSS_IEP_INST1, (handle->iepAttrs)->enableIEP0ShadowMode);
                DebugP_assert(SystemP_SUCCESS == status);

                /*Enable cmp 0 reset of IEP1 counter*/
                status = PRUICSS_PWM_configureIepCmp0ResetEnable(handle, PRUICSS_IEP_INST1, (handle->iepAttrs)->enableIep0ResetOnCompare0);
                DebugP_assert(SystemP_SUCCESS == status);

            }

            /*Enable IEP CMP flags to auto clear after state transition*/
            status = PRUICSS_PWM_configurePwmEfficiencyModeEnable(handle, (handle->iepAttrs)->enableAutoClearCompareStatus);
            DebugP_assert(SystemP_SUCCESS == status);
        }
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}