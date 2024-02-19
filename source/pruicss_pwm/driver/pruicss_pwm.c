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

int32_t PRUICSS_PWM_setIepCounterLower_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint32_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_COUNT_REG_MAX))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_setIepCounterUpper_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint32_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_COUNT_REG_MAX))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_configureIepCmp0ResetEnable(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_configureIepCompareEnable(PRUICSS_Handle handle, uint8_t iepInstance, uint16_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (value <= PRUICSS_IEP_CMP_EVENTS_ENABLE_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_setIepCompareEventLower_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (cmpEvent < PRUICSS_NUM_IEP_CMP_EVENTS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (iepInstance)
        {
            case 0:
                switch (cmpEvent)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0_CMP1_0,(uint32_t)value);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0_CMP2_0,(uint32_t)value);
                        break;
                    case 3:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP3_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP3_REG0_CMP3_0,(uint32_t)value);
                        break;
                    case 4:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP4_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP4_REG0_CMP4_0,(uint32_t)value);
                        break;
                    case 5:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP5_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP5_REG0_CMP5_0,(uint32_t)value);
                        break;
                    case 6:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP6_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP6_REG0_CMP6_0,(uint32_t)value);
                        break;
                    case 7:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP7_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP7_REG0_CMP7_0,(uint32_t)value);
                        break;
                    case 8:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP8_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP8_REG0_CMP8_0,(uint32_t)value);
                        break;
                    case 9:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP9_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP9_REG0_CMP9_0,(uint32_t)value);
                        break;
                    case 10:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP10_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP10_REG0_CMP10_0,(uint32_t)value);
                        break;
                    case 11:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP11_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP11_REG0_CMP11_0,(uint32_t)value);
                        break;
                    case 12:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP12_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP12_REG0_CMP12_0,(uint32_t)value);
                        break;
                    case 13:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP13_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP13_REG0_CMP13_0,(uint32_t)value);
                        break;
                    case 14:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP14_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP14_REG0_CMP14_0,(uint32_t)value);
                        break;
                    case 15:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP15_REG0),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP15_REG0_CMP15_0,(uint32_t)value);
                        break;
                }
                break; 
            case 1:
                switch (cmpEvent)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG0_CMP0_0,(uint32_t)value);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP1_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP1_REG0_CMP1_0,(uint32_t)value);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP2_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP2_REG0_CMP2_0,(uint32_t)value);
                        break;
                    case 3:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG0_CMP3_0,(uint32_t)value);
                        break;
                    case 4:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP4_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP4_REG0_CMP4_0,(uint32_t)value);
                        break;
                    case 5:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG0_CMP5_0,(uint32_t)value);
                        break;
                    case 6:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG0_CMP6_0,(uint32_t)value);
                        break;
                    case 7:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP7_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP7_REG0_CMP7_0,(uint32_t)value);
                        break;
                    case 8:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP8_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP8_REG0_CMP8_0,(uint32_t)value);
                        break;
                    case 9:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP9_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP9_REG0_CMP9_0,(uint32_t)value);
                        break;
                    case 10:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP10_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP10_REG0_CMP10_0,(uint32_t)value);
                        break;
                    case 11:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP11_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP11_REG0_CMP11_0,(uint32_t)value);
                        break;
                    case 12:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP12_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP12_REG0_CMP12_0,(uint32_t)value);
                        break;
                    case 13:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP13_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP13_REG0_CMP13_0,(uint32_t)value);
                        break;
                    case 14:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP14_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP14_REG0_CMP14_0,(uint32_t)value);
                        break;
                    case 15:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP15_REG0),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP15_REG0_CMP15_0,(uint32_t)value);
                        break;
                }
                break;
        }

    } 

    return retVal;
}



int32_t PRUICSS_PWM_setIepCompareEventUpper_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (cmpEvent < PRUICSS_NUM_IEP_CMP_EVENTS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (iepInstance)
        {
            case 0:
                switch (cmpEvent)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1_CMP0_1,(uint32_t)value);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG1_CMP1_1,(uint32_t)value);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG1_CMP2_1,(uint32_t)value);
                        break;
                    case 3:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP3_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP3_REG1_CMP3_1,(uint32_t)value);
                        break;
                    case 4:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP4_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP4_REG1_CMP4_1,(uint32_t)value);
                        break;
                    case 5:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP5_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP5_REG1_CMP5_1,(uint32_t)value);
                        break;
                    case 6:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP6_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP6_REG1_CMP6_1,(uint32_t)value);
                        break;
                    case 7:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP7_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP7_REG1_CMP7_1,(uint32_t)value);
                        break;
                    case 8:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP8_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP8_REG1_CMP8_1,(uint32_t)value);
                        break;
                    case 9:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP9_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP9_REG1_CMP9_1,(uint32_t)value);
                        break;
                    case 10:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP10_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP10_REG1_CMP10_1,(uint32_t)value);
                        break;
                    case 11:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP11_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP11_REG1_CMP11_1,(uint32_t)value);
                        break;
                    case 12:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP12_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP12_REG1_CMP12_1,(uint32_t)value);
                        break;
                    case 13:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP13_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP13_REG1_CMP13_1,(uint32_t)value);
                        break;
                    case 14:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP14_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP14_REG1_CMP14_1,(uint32_t)value);
                        break;
                    case 15:
                        HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP15_REG1),
                               CSL_ICSS_G_PR1_IEP0_SLV_CMP15_REG1_CMP15_1,(uint32_t)value);
                        break;
                }
                break; 
            case 1:
                switch (cmpEvent)
                {
                    case 0:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP0_REG1_CMP0_1,(uint32_t)value);
                        break;
                    case 1:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP1_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP1_REG1_CMP1_1,(uint32_t)value);
                        break;
                    case 2:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP2_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP2_REG1_CMP2_1,(uint32_t)value);
                        break;
                    case 3:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP3_REG1_CMP3_1,(uint32_t)value);
                        break;
                    case 4:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP4_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP4_REG1_CMP4_1,(uint32_t)value);
                        break;
                    case 5:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP5_REG1_CMP5_1,(uint32_t)value);
                        break;
                    case 6:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP6_REG1_CMP6_1,(uint32_t)value);
                        break;
                    case 7:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP7_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP7_REG1_CMP7_1,(uint32_t)value);
                        break;
                    case 8:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP8_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP8_REG1_CMP8_1,(uint32_t)value);
                        break;
                    case 9:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP9_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP9_REG1_CMP9_1,(uint32_t)value);
                        break;
                    case 10:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP10_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP10_REG1_CMP10_1,(uint32_t)value);
                        break;
                    case 11:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP11_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP11_REG1_CMP11_1,(uint32_t)value);
                        break;
                    case 12:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP12_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP12_REG1_CMP12_1,(uint32_t)value);
                        break;
                    case 13:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP13_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP13_REG1_CMP13_1,(uint32_t)value);
                        break;
                    case 14:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP14_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP14_REG1_CMP14_1,(uint32_t)value);
                        break;
                    case 15:
                        HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_CMP15_REG1),
                               CSL_ICSS_G_PR1_IEP1_SLV_CMP15_REG1_CMP15_1,(uint32_t)value);
                        break;
                }
                break;
        }

    }
    
    return retVal;    
}

int32_t PRUICSS_PWM_setPwmDebounceValue(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (value <= PRUICSS_PWM_DEBOUNCE_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_setPwmTripMask(PRUICSS_Handle handle, uint8_t pwmSet, uint16_t maskvalue)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (maskvalue <= PRUICSS_PWM_TRIP_MASK_MAX_VALUE))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (pwmSet)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0),
                               CSL_ICSSCFG_PWM0_PWM0_TRIP_MASK, maskvalue);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1),
                               CSL_ICSSCFG_PWM1_PWM1_TRIP_MASK, maskvalue);
                break;
            case 2:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2),
                               CSL_ICSSCFG_PWM2_PWM2_TRIP_MASK, maskvalue);
                break;
             case 3:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM3),
                               CSL_ICSSCFG_PWM3_PWM3_TRIP_MASK, maskvalue);
                break;
        }
    }

    return retVal;    
}

int32_t PRUICSS_PWM_configurePwmCmp0TripResetEnable(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_generatePwmTripReset(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_generatePwmOverCurrentErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_generatePwmPositionFeedbackErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_clearPwmTripResetStatus(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_clearPwmOverCurrentErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_clearPwmPositionFeedbackErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_getPwmTripTriggerCauseVector(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_getPwmTripStatus(PRUICSS_Handle handle, uint8_t pwmSet)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_clearPwmTripStatus(PRUICSS_Handle handle, uint8_t pwmSet)
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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pwmSet < PRUICSS_NUM_PWM_SETS) && (state < PRUICSS_NUM_PWM_STATES) && (action < PRUICSS_NUM_PWM_OUTPUT_ACTIONS))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_configurePwmEfficiencyModeEnable(PRUICSS_Handle handle, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_PIN_MX),CSL_ICSSCFG_PIN_MX_PWM_EFC_EN, enable);
    }
    return retVal;    
}

int32_t PRUICSS_PWM_enableIEP1Slave(PRUICSS_Handle handle, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_IEPCLK),
                      CSL_ICSSCFG_IEPCLK_IEP1_SLV_EN, 1);
    }
    return retVal;

}

int32_t PRUICSS_PWM_enableIEPResetOnEPWM0SyncOut(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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

int32_t PRUICSS_PWM_enableIEPResetOnEPWM3SyncOut(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable)
{

    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < PRUICSS_NUM_IEP_INSTANCES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

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