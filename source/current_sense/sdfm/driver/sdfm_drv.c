/*
 * Copyright (C) 2023-25 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   sdfm_drv.c
 *
 *  \brief  PRU SDFM (Sigma-Delta Filter Module) driver implementation.
 *
 *  \details
 *  This file implements the SDFM driver API functions for current sensing PRU SDFM
 *
 *  ## Driver Architecture
 *
 *  The SDFM driver manages communication between the ARM R5F core and PRU firmware
 *  that performs real-time sigma-delta filtering. The driver:
 *
 *  - Initializes the SDFM interface in PRU DMEM (shared memory)
 *  - Configures channel parameters (OSR, filter type, clock source)
 *  - Manages threshold comparators for over-current detection
 *  - Configures trigger/snoop modes for synchronized sampling
 *  - Provides data retrieval and status monitoring functions
 *
 *  ## Shared Memory Communication
 *
 *  The driver communicates with PRU firmware via the SDFM_Interface structure
 *  mapped to PRU DMEM. This structure contains:
 *
 *  - Control registers (enable, snoop mode, acknowledgments)
 *  - Channel configurations (all 9 channels)
 *  - Trigger configurations (for each PRU core)
 *  - Threshold parameters and status flags
 *  - GPIO parameters for zero-cross detection
 *
 *  The memory layout must exactly match the firmware's expectations to ensure
 *  correct operation.
 *
 *  ## Operating Modes
 *
 *  **Trigger Mode** (SDFM_enableTriggerModeForNormalCurrent):
 *  - IEP-based synchronized sampling at specific PWM phase angles
 *  - Configurable first and second sample points per PWM cycle
 *  - Used for precise motor control current measurements
 *
 *  **Snoop Mode** (SDFM_enableSnoopBasedNC):
 *  - Used when OC osr and normal current OSR is different
 *  - Clock should internal and not external
 *
 *  **Load-Share Mode** (SDFM_enableLoadShareMode):
 *  - Distributes 9 channels across 3 PRU cores (RTU_PRU, PRU, TX_PRU)
 *  - Each core handles 3 channels: CH0/3/6, CH1/4/7, CH2/5/8
 *  - Enables higher sampling rates through parallel processing
 *
 *  ## Clock Sources
 *
 *  The driver supports multiple clock sources for sigma-delta modulators:
 *
 *  - **IEP (Industrial Ethernet Peripheral)**: Programmable internal clock
 *  - **ECAP (Enhanced Capture)**: Configurable PWM-based clock generation
 *  - **PRU GPIO1**: Shift-out mode with programmable dividers
 *
 *  ## Implementation Notes
 *
 *  - All channel and PRU core indices are 0-based
 *  - PRU core indices: 0=PRU, 1=RTU_PRU, 2=TX_PRU
 *  - Channel numbers range from 0-8 (9 channels total)
 *  - IEP counter increment is always configured as 1 for consistent timing
 *
 *  ## Related Files
 *
 *  - sdfm_api.h: Public API declarations
 *  - sdfm_drv.h: Data structures and type definitions
 *  - icssg_sdfm.h: Firmware interface definitions
 */

#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <current_sense/sdfm/include/sdfm_drv.h>
#include <current_sense/sdfm/include/sdfm_api.h>
#include <current_sense/sdfm/firmware/icssg_sdfm.h>
#include <pruicss_pwm/include/pruicss_pwm.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>
#include <drivers/soc.h>
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern SDFM_Handle_Config gSdfmHandles[];
extern uint32_t gSdfmConfigNum;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* Initialize SDFM instance */
SDFM_Handle SDFM_init(uint32_t index, SDFM_Params sdfm_params)
{
    SDFM_Handle handle = NULL;
    if(sdfm_params.pruicss_handle == NULL || (index >= gSdfmConfigNum))
    {
        return NULL;
    }
    else
    {
        handle = (SDFM_Handle)(&gSdfmHandles[index]);
        /* Initialize SDFM interface address */
        if(sdfm_params.pru_slice == PRUICSS_PRU0)
        {              
            handle->sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(sdfm_params.pruicss_handle->hwAttrs))->pru0DramBase);
        }
        else if (sdfm_params.pru_slice == PRUICSS_PRU1)
        {
             handle->sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(sdfm_params.pruicss_handle->hwAttrs))->pru1DramBase);
        }
        else
        {
             return NULL;
        }
        /* Initialize SDFM handle parameters*/
        handle->pru_config.load_share_enable = sdfm_params.load_share_enable;
        handle->pru_config.pru_slice = sdfm_params.pru_slice;
        handle->pru_config.pru_clock = sdfm_params.pru_clock;
        handle->pru_config.iep_clock = sdfm_params.iep_clock;
        /* Always configure IEP counter increment value as 1. All cmp event configurations assume IEP counter increment by 1. */
        handle->pru_config.iep_instance = 1;
        handle->sdfm_interface->control[SDFM_PRU_CORE_INDX].enable_snoop_nc = sdfm_params.pru_core_config[SDFM_PRU_CORE_INDX].enable_snoop_mode;
        handle->sdfm_interface->control[SDFM_RTUPRU_CORE_INDX].enable_snoop_nc = sdfm_params.pru_core_config[SDFM_RTUPRU_CORE_INDX].enable_snoop_mode;
        handle->sdfm_interface->control[SDFM_TXPRU_CORE_INDX].enable_snoop_nc = sdfm_params.pru_core_config[SDFM_TXPRU_CORE_INDX].enable_snoop_mode;
        handle->pru_config.pwm_handle = sdfm_params.pwm_handle;
        handle->pru_config.pruicss_handle = sdfm_params.pruicss_handle;

        if(sdfm_params.load_share_enable == 1)
        {
            SDFM_enableLoadShareMode(handle, sdfm_params.pru_slice);
        }
        /* Enable SDFM mode */
        PRUICSS_setGpiMode(sdfm_params.pruicss_handle, sdfm_params.pru_slice, PRUICSS_GP_MUX_SEL_MODE_SD);
    }

    return handle;
}

/*Configuration of iep reset cycle time period */
int32_t SDFM_configIepCount(SDFM_Handle h_sdfm, uint32_t iep_reset_freq)
{
    if( h_sdfm == NULL || iep_reset_freq == 0)
    {
        return SystemP_FAILURE;
    }
    /*; max IEP0 count value for one epwm period*/
    uint32_t max_iep_cnt = (h_sdfm->pru_config.iep_clock/iep_reset_freq)*(h_sdfm->pru_config.iep_inc_value);
    h_sdfm->sdfm_interface->trigger_config[SDFM_PRU_CORE_INDX].max_iep_cnt_per_epwm_prd = max_iep_cnt;
    h_sdfm->sdfm_interface->trigger_config[SDFM_RTUPRU_CORE_INDX].max_iep_cnt_per_epwm_prd = max_iep_cnt;
    h_sdfm->sdfm_interface->trigger_config[SDFM_TXPRU_CORE_INDX].max_iep_cnt_per_epwm_prd = max_iep_cnt;

    return SystemP_SUCCESS;
}

/*ecap configuration for SD clock*/
int32_t SDFM_configEcap(SDFM_Handle h_sdfm, uint8_t ecap_divider)
{
    if(h_sdfm == NULL || ecap_divider == 0)
    {
        return SystemP_FAILURE;
    }
 
    void *pruicssEcap = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->ecapRegBase);;
    uint32_t rgval;
    uint32_t count;
    
    /*Set eCAP APWM mode*/ 
    rgval = HW_RD_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |= (0<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCI_EN_SHIFT) | (2<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCO_SEL_SHIFT) | (1<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_CAP_APWM_SHIFT) | (0<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_APWMPOL_SHIFT);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

    /*Set period count*/
    count = ecap_divider - 1;
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP1, count);

    /*Compute & set Duty Cycle count.
    Divide period count by 2, biased rounding.*/
    count = count + 1;
    count = count/2;
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP2, count);

    /*Clear counter phase and Reset eCAP PWM Counter  */
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CNTPHS, 0);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_TSCNT, 0);

    /* Enable eCAP APWM*/
    rgval = HW_RD_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |=  (1 << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_TSCNTSTP_SHIFT);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

    return SystemP_SUCCESS;

}

/*sdfm Hw osr configuration */
 int32_t SDFM_setCompFilterOverSamplingRatio(SDFM_Handle h_sdfm, uint8_t ch_id, uint16_t osr)
 {
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if(ch_id > SDFM_CHANNEL8 || ch_id < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    if(h_sdfm != NULL)
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((h_sdfm->pru_config.pruicss_handle)->hwAttrs);
        if(h_sdfm->pru_config.pru_slice == PRUICSS_PRU0)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0, osr - 1);
        }
        else if (h_sdfm->pru_config.pru_slice == PRUICSS_PRU1)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0, osr - 1);
        }
        else
        {
            return SystemP_FAILURE; 
        }
    }
    /*Over current OSR*/
    h_sdfm->sdfm_interface->channels[ch_id].over_current_osr = osr - 1;
    return retVal;
 }

/*sdfm high, low  threshold config */
int32_t SDFM_setCompFilterThresholds(SDFM_Handle h_sdfm, uint8_t ch_id, uint32_t *thresholdParms)
{
    if(thresholdParms == NULL || h_sdfm == NULL || ch_id > SDFM_CHANNEL8 || ch_id < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
   /* SD Over current high threshold */
    h_sdfm->sdfm_interface->channels[ch_id].threshold_config.high_threshold = thresholdParms[0];
    /* SD Over current low threshold */
    h_sdfm->sdfm_interface->channels[ch_id].threshold_config.low_threshold = thresholdParms[1];
    return SystemP_SUCCESS;
}

/*sdfm smapling time configuation */
int32_t SDFM_setSampleTriggerTime(SDFM_Handle h_sdfm, float samp_trig_time, uint8_t pru_core)
{
    if(pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0 || h_sdfm == NULL)
    {
        return SystemP_FAILURE;
    }  
    /*convert sample time into IEP count*/
    /*samp time in us */
    int32_t count = (h_sdfm->pru_config.iep_clock /1000000)*((float)samp_trig_time);
    h_sdfm->sdfm_interface->trigger_config[pru_core].first_samp_trig_time = count;
    return SystemP_SUCCESS;
}
/*Second normal current sampling configuration*/
int32_t SDFM_enableDoubleSampling(SDFM_Handle h_sdfm, float samp_trig_time, uint8_t pru_core)
{
    if(pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0 || h_sdfm == NULL)
    {
        return SystemP_FAILURE;
    }
    /*Enable double normal current sampling*/
    h_sdfm->sdfm_interface->trigger_config[pru_core].en_double_nc_sampling = 1;
    /*Second sample point*/
    int32_t count = (h_sdfm->pru_config.iep_clock /1000000)*((float)samp_trig_time);
    h_sdfm->sdfm_interface->trigger_config[pru_core].second_samp_trig_time = count;

    return SystemP_SUCCESS;
}

/*Disable Double update*/
int32_t SDFM_disableDoubleSampling(SDFM_Handle h_sdfm, uint8_t pru_core)
{
    if(pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0 || h_sdfm == NULL)
    {
        return SystemP_FAILURE;
    }
    h_sdfm->sdfm_interface->trigger_config[pru_core].en_double_nc_sampling = 0;
    return SystemP_SUCCESS;
}
/* Enable the channel specified by the channel number parameter*/
int32_t SDFM_setEnableChannel(SDFM_Handle h_sdfm, uint8_t channel_number)
{
    int32_t                 retVal = SystemP_FAILURE;
    if(channel_number > SDFM_CHANNEL8 || channel_number < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }

    if(h_sdfm != NULL)
    {
        retVal = SystemP_SUCCESS;
        h_sdfm->sdfm_interface->channels[channel_number].ch_id = channel_number;
        h_sdfm->sdfm_interface->channels[channel_number].enabled = 1;
        h_sdfm->sdfm_interface->active_channels_mask |= (1 << channel_number);
    }
    
    return retVal;
}
/* set SDFM channel acc source */
int32_t SDFM_configDataFilter(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t filter)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if(ch_id > SDFM_CHANNEL8 || ch_id < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    if(h_sdfm != NULL)
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((h_sdfm->pru_config.pruicss_handle)->hwAttrs);
        if(h_sdfm->pru_config.pru_slice == PRUICSS_PRU0)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC_SEL0, filter);
        }
        else if (h_sdfm->pru_config.pru_slice == PRUICSS_PRU1)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC_SEL0, filter);
        }
        else
        {
            return SystemP_FAILURE;
        }
        
    }

    h_sdfm->sdfm_interface->channels[ch_id].filter_type = filter;
    return retVal;
}

/*set  clock source for SDFM channel*/
int32_t SDFM_selectClockSource(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t clk_source)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if(ch_id > SDFM_CHANNEL8 || ch_id < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }

    if(h_sdfm != NULL)
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((h_sdfm->pru_config.pruicss_handle)->hwAttrs);
        if(h_sdfm->pru_config.pru_slice == PRUICSS_PRU0)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0, clk_source);
        }
        else if (h_sdfm->pru_config.pru_slice == PRUICSS_PRU1)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0, clk_source);
        }
        else
        {
            return SystemP_FAILURE;
        }
        
    }
    h_sdfm->sdfm_interface->channels[ch_id].clk_source = clk_source;
    return retVal;
}
/* set clock inversion for SDFM channel*/
int32_t SDFM_setClockInversion(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t clk_inv)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if(ch_id > SDFM_CHANNEL8 || ch_id < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    
    if(h_sdfm != NULL)
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)((h_sdfm->pru_config.pruicss_handle)->hwAttrs);
        if(h_sdfm->pru_config.pru_slice == PRUICSS_PRU0)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0, clk_inv);
        }
        else if (h_sdfm->pru_config.pru_slice == PRUICSS_PRU1)
        {
            HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (ch_id * 8)),
            CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0, clk_inv);
        }
        else
        {
            return SystemP_FAILURE;
        }
        
    }

    h_sdfm->sdfm_interface->channels[ch_id].clk_inv = clk_inv;

    return retVal;
}
/* Enable the comparator feature for a specified filter/channel */
int32_t SDFM_enableComparator(SDFM_Handle h_sdfm, uint8_t ch)
{
    uint8_t pwmSet = 0;
    uint16_t trip_mask; 
    int32_t  retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;

    if (ch > SDFM_CHANNEL8 || ch < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }

    if(ch < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (ch > SDFM_CHANNEL2 && ch < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (ch > SDFM_CHANNEL5 && ch <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        return SystemP_FAILURE;
    }

    retVal = PRUICSS_PWM_getPwmTripMask(pwm_handle, pwmSet, &trip_mask);
    if(retVal != SystemP_SUCCESS)
    {
        return retVal;
    }
    trip_mask |= 0x2; /*set the trip mask for over current trip*/

    retVal = PRUICSS_PWM_setPwmTripMask(pwm_handle, pwmSet, trip_mask);

    if(retVal != SystemP_SUCCESS)
    {
        return retVal;
    }
    h_sdfm->sdfm_interface->channels[ch].enable_comparator = 1;
    return retVal;
}

/* Disable the comparator feature for a specified filter/channel */
int32_t SDFM_disableComparator(SDFM_Handle h_sdfm, uint8_t ch)
{
    uint8_t pwmSet;
    uint16_t trip_mask; 
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;

    if (ch > SDFM_CHANNEL8 || ch < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }

    if(ch < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (ch > SDFM_CHANNEL2 && ch < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (ch > SDFM_CHANNEL5 && ch <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        return SystemP_FAILURE;
    }
    
    retVal = PRUICSS_PWM_getPwmTripMask(pwm_handle, pwmSet, &trip_mask);
    if(retVal != SystemP_SUCCESS)
    {
        return retVal;
    }
    trip_mask &= 0xFFFD; /*Clear the trip mask for over current trip*/

    retVal = PRUICSS_PWM_setPwmTripMask(pwm_handle, pwmSet, trip_mask);

    if(retVal != SystemP_SUCCESS)
    {
        return retVal;
    }
    h_sdfm->sdfm_interface->channels[ch].enable_comparator = 0;
    
    return retVal;
}
/*GPIO configuration*/
int32_t SDFM_configComparatorGpioPins(SDFM_Handle h_sdfm, uint8_t ch, uint32_t gpio_base_addr, uint32_t pin_number)
{

    if(ch > SDFM_CHANNEL8 || ch < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) gpio_base_addr);
    uint32_t reg_index = GPIO_GET_REG_INDEX(pin_number);
    uint32_t reg_val = GPIO_GET_BIT_MASK(pin_number);
    uint32_t clr_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].CLR_DATA;
    uint32_t set_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].SET_DATA;

    h_sdfm->sdfm_interface->channels[ch].gpio_params.write_val = reg_val;
    h_sdfm->sdfm_interface->channels[ch].gpio_params.set_val_addr = set_data_addr;
    h_sdfm->sdfm_interface->channels[ch].gpio_params.clr_val_addr = clr_data_addr;

    return SystemP_SUCCESS;
}

/* Get current (or latest) sample for the specified channel */
uint32_t SDFM_getFilterData(SDFM_Handle h_sdfm, uint8_t ch)
{
    if(ch > SDFM_CHANNEL8 || ch < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    else{
        return (uint32_t)(h_sdfm->sampleOutputInterface->sampleOutput[ch]);
    }
}

/*Configure normal current OSR for data filter*/
int32_t SDFM_setFilterOverSamplingRatio(SDFM_Handle h_sdfm, uint8_t ch, uint16_t nc_osr)
{ 
    uint8_t pru_core;
    
    if(ch < SDFM_CHANNEL3)
    {
        pru_core = 0;
    }
    else if (ch > SDFM_CHANNEL2 && ch < SDFM_CHANNEL6)
    {
        pru_core = 1;
    }
    else if (ch > SDFM_CHANNEL5 && ch <= SDFM_CHANNEL8)
    {
        pru_core = 2;
    }
    else
    {
        return SystemP_FAILURE;
    }
    
    if(h_sdfm->sdfm_interface->control[pru_core].enable_snoop_nc == 1)
    {
        /*IEP0 counts in normal current sampling period*/
        uint16_t count;
        uint32_t iep_freq = h_sdfm->pru_config.iep_clock;
        uint32_t sd_clock = h_sdfm->clk_config.sdfm_clock_value;
        count = (int)((float)nc_osr*((float)iep_freq/(float)sd_clock));
        h_sdfm->sdfm_interface->trigger_config[pru_core].nc_prd_iep_cnt = count;
    }
    else
    {
        /*Setting SDFM hardware OSR for normal current without snoop mode */
        SDFM_setCompFilterOverSamplingRatio(h_sdfm, ch, nc_osr);
    }

    h_sdfm->sdfm_interface->channels[ch].normal_current_osr = nc_osr;
    return SystemP_SUCCESS;
}
/*return firmware version */
uint32_t SDFM_getFirmwareVersion(SDFM_Handle h_sdfm)
{
   return h_sdfm->sdfm_interface->firmwareVersion >> SDFM_FW_VERSION_BIT_SHIFT;
}
/*Trigger based normal current */
int32_t SDFM_enableTriggerModeForNormalCurrent(SDFM_Handle h_sdfm, uint8_t pru_core)
{
    if(pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0)
    {
        return SystemP_FAILURE;
    }
    h_sdfm->sdfm_interface->trigger_config[pru_core].enable_trigger_mode = 1;
    return SystemP_SUCCESS;
}
/*FD block configuration */
int32_t SDFM_configFastDetect(SDFM_Handle h_sdfm, uint8_t ch, uint8_t *fdParms)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    uint8_t pwmSet = 0; // Initialize to prevent uninitialized use
    uint16_t trip_mask; 
    int32_t retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;

    if (ch > SDFM_CHANNEL8 || ch < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }

    if(ch < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (ch > SDFM_CHANNEL2 && ch < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (ch > SDFM_CHANNEL5 && ch <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        return SystemP_FAILURE;
    }

    h_sdfm->sdfm_interface->channels[ch].fd_enable = fdParms[0];
    h_sdfm->sdfm_interface->channels[ch].fd_window = fdParms[1];
    h_sdfm->sdfm_interface->channels[ch].fd_zero_max = fdParms[2];
    h_sdfm->sdfm_interface->channels[ch].fd_zero_min = fdParms[3];
      
    /*Configure one max to window size + 1 and one min to 0, so they never get set*/
    h_sdfm->sdfm_interface->channels[ch].fd_one_max = (fdParms[1] + 1) * 4 + 1;
    h_sdfm->sdfm_interface->channels[ch].fd_one_min = 0;
    
    hwAttrs = (PRUICSS_HwAttrs const *)((h_sdfm->pru_config.pruicss_handle)->hwAttrs);

    if(h_sdfm->pru_config.pru_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_FD_ZERO_MAX_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_zero_max);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_FD_ZERO_MIN_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_zero_min);

        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_EN_0, 1);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_ONE_MAX_LIMIT_0,  h_sdfm->sdfm_interface->channels[ch].fd_one_max);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_ONE_MIN_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_one_min);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_WINDOW_SIZE_0, h_sdfm->sdfm_interface->channels[ch].fd_window);
    }
    else if (h_sdfm->pru_config.pru_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_FD_ZERO_MAX_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_zero_max);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_FD_ZERO_MIN_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_zero_min);

        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_EN_0, 1);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_ONE_MAX_LIMIT_0,  h_sdfm->sdfm_interface->channels[ch].fd_one_max);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_ONE_MIN_LIMIT_0, h_sdfm->sdfm_interface->channels[ch].fd_one_min);
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (ch * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_WINDOW_SIZE_0, h_sdfm->sdfm_interface->channels[ch].fd_window);
    }
    else
    {
        return SystemP_FAILURE;
    } 

    retVal = PRUICSS_PWM_getPwmTripMask(pwm_handle, pwmSet, &trip_mask);
    if(retVal != SystemP_SUCCESS)
    {
        return retVal;
    }
    trip_mask |= (1<<(ch+2)); /*set the trip mask for fast detect trip*/

    retVal = PRUICSS_PWM_setPwmTripMask(pwm_handle, pwmSet, trip_mask);

    return retVal;
}

/*return status of PWM trip vector status bit*/
int32_t SDFM_getFastDetectErrorStatus(SDFM_Handle h_sdfm, uint8_t chNum) 
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    
    /*PWM trip vector */
    retVal = PRUICSS_PWM_getPwmTripTriggerCauseVector(pwm_handle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    else
    {

        retVal =  retVal >> 2;
        uint32_t temp;
        temp  = 1 << chNum;
        if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
        {
            return ((retVal) & (1 << SDFM_CHANNEL0)) ? 1 : 0;
        }
        else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
        {
            return ((retVal) & (1 << SDFM_CHANNEL1))? 1 : 0;
        }
        else 
        {
            return ((retVal) & (1 << SDFM_CHANNEL2)) ? 1 : 0;
        }
        
    }
    
}

/*Clear Trip status bit*/
int32_t SDFM_clearPwmTripStatus(SDFM_Handle h_sdfm, uint8_t chNum)
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;
    
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*clear trip status*/
    retVal = PRUICSS_PWM_generatePwmTripReset(pwm_handle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*clear trip reset status*/
    retVal = PRUICSS_PWM_clearPwmTripResetStatus(pwm_handle, pwmSet);

    return retVal;
}
/*Enable Load share mode*/
void SDFM_enableLoadShareMode(SDFM_Handle h_sdfm, uint8_t sliceId)
{
    void *pruicssCfg = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->cfgRegBase);
   
    uint32_t rgval;
    if(sliceId)
    {
       rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU1CLKDIV);
       rgval |= CSL_ICSSCFG_SDPRU1CLKDIV_PRU1_SD_SHARE_EN_MASK;
       HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU1CLKDIV, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU0CLKDIV);
        rgval |= CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU0CLKDIV, rgval);
    }

}
/*Measure Phase delay*/
int32_t SDFM_measureClockPhaseDelay(SDFM_Handle h_sdfm, uint16_t clkEdg, uint8_t chNum)
{
    if(chNum > SDFM_CHANNEL8 || chNum < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    /*enable phase delay measurement*/
    h_sdfm->sdfm_interface->channels[chNum].en_phase_delay = 1;
    /*waiting till measurment done */
    uint8_t ack = h_sdfm->sdfm_interface->channels[chNum].en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK;
    while(ack)
    {
       ack = h_sdfm->sdfm_interface->channels[chNum].en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK ;
    }

   uint16_t nEdge = h_sdfm->sdfm_interface->channels[chNum].clock_edge;
   float temp = h_sdfm->sdfm_interface->channels[chNum].clock_phase_delay;
   /*avg*/
    temp = temp/SDFM_PHASE_DELAY_CAL_LOOP_SIZE;
   /*check data reading edge(clk polarity) & nearest edge */
   if(nEdge == clkEdg)
   {
      /*PRU cycles for half SD clock period*/
      uint32_t pruCycles = ceil(((float)h_sdfm->pru_config.pru_clock)/(2*h_sdfm->sdfm_interface->channels[chNum].sdfm_clk));
      h_sdfm->sdfm_interface->channels[chNum].clock_phase_delay = pruCycles - temp;
   }
   else
   {
      /*PRU cycles for one SD clock period*/
      uint32_t pruCycles = ceil((float)(h_sdfm->pru_config.pru_clock/(h_sdfm->sdfm_interface->channels[chNum].sdfm_clk)));
      h_sdfm->sdfm_interface->channels[chNum].clock_phase_delay = pruCycles - temp;
   }

   return SystemP_SUCCESS;
}
float SDFM_getClockPhaseDelay(SDFM_Handle h_sdfm, uint8_t chNum)
{
    /*conversion from PRU cycle to ns */
    float phaseDelay =  ((float)h_sdfm->sdfm_interface->channels[chNum].clock_phase_delay * 1000000000)/h_sdfm->pru_config.pru_clock;
    return phaseDelay;
}
int32_t SDFM_getHighThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum)
{
    if(chNum > SDFM_CHANNEL8 || chNum < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    else
    {
        return h_sdfm->sdfm_interface->channels[chNum].threshold_config.high_th_status;
    }
}
int32_t SDFM_getLowThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum)
{
    if(chNum > SDFM_CHANNEL8 || chNum < SDFM_CHANNEL0)
    {
        return SystemP_FAILURE;
    }
    else
    {
        return  h_sdfm->sdfm_interface->channels[chNum].threshold_config.low_th_status;
    }
}

int32_t SDFM_clearOverCurrentError(SDFM_Handle h_sdfm, uint8_t chNum)
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle = h_sdfm->pru_config.pwm_handle;
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*Clear over current Error PWM trip*/
    retVal = PRUICSS_PWM_clearPwmOverCurrentErrorTrip(pwm_handle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    
    /*Clear PWM trip*/
    retVal = SDFM_clearPwmTripStatus(h_sdfm, chNum);
    return retVal;
}
int32_t SDFM_enableZeroCrossDetection(SDFM_Handle h_sdfm, uint8_t chNum, uint32_t zcThr)
{

    if(chNum <= SDFM_CHANNEL8 && chNum >= SDFM_CHANNEL0)
    {
        h_sdfm->sdfm_interface->channels[chNum].threshold_config.en_zero_cross = 1;
        h_sdfm->sdfm_interface->channels[chNum].threshold_config.zero_cross_threshold = zcThr;
    }
    else 
    {
        return SystemP_FAILURE;  
    }

    return SystemP_SUCCESS;
}
int32_t SDFM_getZeroCrossThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum)
{

    if(chNum <= SDFM_CHANNEL8 && chNum >= SDFM_CHANNEL0)
    {
        return h_sdfm->sdfm_interface->channels[chNum].threshold_config.zero_cross_th_status; 
    }
    else 
    {
        return SystemP_FAILURE;  
    }

}
int32_t SDFM_disableZeroCrossDetection(SDFM_Handle h_sdfm, uint8_t chNum)
{
    int32_t   retVal = SystemP_FAILURE;

    if(chNum <= SDFM_CHANNEL8 && chNum >= SDFM_CHANNEL0)
    {
        h_sdfm->sdfm_interface->channels[chNum].threshold_config.en_zero_cross = 0;
        retVal = SystemP_SUCCESS;
    }
    else
    {
        return SystemP_FAILURE;
    }

    return retVal;
}

int32_t SDFM_enableEpwmSync(SDFM_Handle h_sdfm, uint8_t epwmIns)
{
    void *pru_iep;
    int32_t   retVal = SystemP_FAILURE;

    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }
    
    if(pru_iep != NULL && (epwmIns == 0 || epwmIns == 3))
    {
        retVal = SystemP_SUCCESS;

        switch (epwmIns)
        {
            case 0:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM0_RST_CNT_EN, 1);
                break;
             case 3:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM3_RST_CNT_EN, 1);
                break;
        }
    }
    h_sdfm->enable_sync_with_epwm = 1;
    h_sdfm->sync_epwm_src = epwmIns;
    
    return retVal;
}

int32_t SDFM_disableEpwmSync(SDFM_Handle h_sdfm, uint8_t epwmIns)
{
    void *pru_iep;
    int32_t   retVal = SystemP_FAILURE;
    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }
    
    if(pru_iep != NULL && (epwmIns == 0 || epwmIns == 3))
    {
        retVal = SystemP_SUCCESS;

        switch (epwmIns)
        {
            case 0:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM0_RST_CNT_EN, 0);
                break;
             case 3:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM3_RST_CNT_EN, 0);
                break;
        }
    }
    h_sdfm->enable_sync_with_epwm = 0;
    h_sdfm->sync_epwm_src = 0;
    return retVal;
}

int32_t SDFM_configIepSyncMode(SDFM_Handle h_sdfm, uint32_t highPulseWidth, uint32_t periodTime, uint32_t syncStartTime)
{

    void        *pruIep;
    int32_t     retVal = SystemP_FAILURE;
    uint32_t    regVal ;

    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pruIep != NULL)
    {
       
        /*Set CMP1 period - SYNC0 trigger */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, syncStartTime);

        /*Set CMP2 period - SYNC1 trigger */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, syncStartTime);
               
        /*Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_EN_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_EN_SHIFT);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_SHIFT) | (0<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_IND_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, regVal);

        /*Set SYNC0/1 high pulse time in iep clok cycles  */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, highPulseWidth);

        /*Set SYNC0/1 period*/
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, periodTime);

        /*Set offset from cpm hit*/
        HW_WR_REG32( (uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

        /*Enable cmp1 and cmp2 for sync start trigger generation*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
        regVal |= (1<<SDFM_IEP_CMP1_EN_SHIFT)|(1<<SDFM_IEP_CMP2_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, regVal);
       
        /*Set default and compensation increment to 1*/
        regVal = HW_RD_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT );
        HW_WR_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

        retVal = SystemP_SUCCESS;

    }
    
    return retVal;
}

int32_t SDFM_enableIep(SDFM_Handle h_sdfm)
{
    void       *pruIep;
    int32_t    retVal = SystemP_FAILURE;
    uint32_t   regVal ;

    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pruIep != NULL)
    {
        /*IEP Counter increment value*/
        HW_WR_FIELD32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG,
        CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC, h_sdfm->pru_config.iep_inc_value);
        /*start iep0_timer*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        regVal |= 0x1;
        HW_WR_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);



        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t SDFM_configSync1Delay(SDFM_Handle h_sdfm, uint32_t delay)
{
    void      *pruIep;
    int32_t   retVal = SystemP_FAILURE;

    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pruIep != NULL)
    {
        /*Set delay between SYNC0 and SYNC1 in clock cycles */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, delay);

        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t SDFM_configClockFromGPO1(SDFM_Handle h_sdfm, uint8_t div0, uint8_t div1)
{
   
    uint32_t rgval;
    void *pruicssCfg =(void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->cfgRegBase);;
    int32_t   retVal = SystemP_SUCCESS;
    
    if( div0 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MAX || div1 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MAX )
    {
        retVal = SystemP_FAILURE;
        return retVal;
    }
    /*configure divider*/
    if(h_sdfm->pru_config.pru_slice == 1)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);

        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);
    }
    else if (h_sdfm->pru_config.pru_slice == 0)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
        
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
    }
    else
    {
        retVal = SystemP_FAILURE;
        return retVal;
    }
    
    /*enabling shift mode */
    if(h_sdfm->pru_config.pru_slice == 1)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MAX<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);
    }
    else if(h_sdfm->pru_config.pru_slice == 0)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MAX<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
    }
    else
    {
        retVal = SystemP_FAILURE;
        return retVal;
    }

    return retVal;
    
}

/*Enable snoop based NC sampling */
int32_t SDFM_enableSnoopBasedNC(SDFM_Handle h_sdfm, uint8_t pru_core)
{
    if(h_sdfm == NULL || pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0)
    {
        return SystemP_FAILURE;
    }
    /*Enable snoop based NC sampling */
    h_sdfm->sdfm_interface->control[pru_core].enable_snoop_nc = 1;

    return SystemP_SUCCESS;
}

/*Disable snoop basedNC sampling */
int32_t SDFM_disableSnoopBasedNC(SDFM_Handle h_sdfm, uint8_t pru_core)
{
    if(h_sdfm == NULL || pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0)
    {
        return SystemP_FAILURE;
    }
    /*Disable snoop basedNC sampling */
    h_sdfm->sdfm_interface->control[pru_core].enable_snoop_nc = 0;
    return SystemP_SUCCESS;
}
int32_t SDFM_selectIepCmpEvent(SDFM_Handle h_sdfm, uint8_t event, uint8_t pru_core)
{
    void      *pruIep;
    uint32_t  iep_cmp_reg;
    uint32_t  iep_cmp_status_reg;
    uint32_t regVal; 
    if(h_sdfm == NULL || pru_core > NUM_OF_PRU_CORE_PER_PRU_SLICE || pru_core < 0)
    {
        return SystemP_FAILURE;
    }
    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        /*Enable IEP0 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
        iep_cmp_reg = CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + (event * 8);
        iep_cmp_status_reg = CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG;
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        /*Enable IEP1 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
        iep_cmp_reg = CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + (event * 8);
        iep_cmp_status_reg = CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG;
    }
    else
    {
        return SystemP_FAILURE;
    }

    /*Configure the cmp event*/
    /* Read the current register value */
    regVal = HW_RD_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Set the CMP_EN bit (OR with the new value) */
    regVal |= ((uint32_t)1U << event) << CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), regVal);
    
    if(event > 7)
    {
        iep_cmp_reg += 8;
    } 

    /*Select IEP CMP event as trigger source */
    h_sdfm->sdfm_interface->trigger_config[pru_core].iep_cmp_event = event;
    h_sdfm->sdfm_interface->trigger_config[pru_core].iep_cmp_event_reg = iep_cmp_reg;
    h_sdfm->sdfm_interface->trigger_config[pru_core].iep_cmp_status_reg = iep_cmp_status_reg;
     
    return SystemP_SUCCESS;
}

int32_t SDFM_configIepCmp0ToResetIep(SDFM_Handle h_sdfm, uint32_t iep_reset_freq)
{
    void      *pruIep;
    int64_t    iep_count;
    uint32_t regVal; 
    if(h_sdfm == NULL || iep_reset_freq == 0)
    {
        return SystemP_FAILURE;
    }
    if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST0)
    {
        /*Enable IEP0 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(h_sdfm->pru_config.iep_instance == PRUICSS_IEP_INST1)
    {
        /*Enable IEP1 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(h_sdfm->pru_config.pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }
    iep_count = h_sdfm->pru_config.iep_clock / iep_reset_freq;
    /*Configure the cmp0 register to generate reset at required frequency*/
    HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, (iep_count & 0xffffffff) - 1);
    HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, (iep_count>>32) & 0xffffffff);

    /* Read the current register value */
    regVal = HW_RD_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG));

    /* Set the CMP_EN bit (OR with the new value) */
    regVal |= ((uint32_t)1U << 0) << CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), regVal);

    /*Select IEP CMP0 to reset iep counter */
    HW_WR_FIELD32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG),
                        CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN, 1);

    return SystemP_SUCCESS;
}

void SDFM_setSampleOutputInterfaceGlobalAddr(SDFM_Handle h_sdfm, uint32_t addr)
{
    h_sdfm->sdfm_interface->trigger_config[0].sample_buff_base_addr = addr;
    h_sdfm->sdfm_interface->trigger_config[1].sample_buff_base_addr = addr + 12;
    h_sdfm->sdfm_interface->trigger_config[2].sample_buff_base_addr = addr + 24;
}

/* SDFM global enable */
void SDFM_enable(SDFM_Handle h_sdfm, uint8_t pru_core)
{
    uint8_t sdfm_en_ack;

    /*Enable SDFM */
    h_sdfm->sdfm_interface->control[pru_core].enable = 1;

    /* wait for ACK */
    do {
        sdfm_en_ack = h_sdfm->sdfm_interface->control[pru_core].enable_ack;
    } while (sdfm_en_ack != BF_SDFM_EN_ENABLE);
}