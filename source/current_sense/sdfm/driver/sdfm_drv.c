/*
 * Copyright (C) 2023-2025 Texas Instruments Incorporated - http://www.ti.com/
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
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern SDFM_Config gSdfmHandle[];
extern uint32_t gSdfmConfigNum;

/* ========================================================================== */
/*                      Static Function Declarations                          */
/* ========================================================================== */
static int32_t SDFM_enableLoadShareMode(SDFM_Handle handle, uint8_t sliceId);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Initialize SDFM parameters structure with default values
 *
 * \param params Pointer to SDFM_Params structure to initialize
 */
void SDFM_paramsInit(SDFM_Params *params)
{
    if (params != NULL)
    {
        params->pruicss_handle = NULL;
        params->pwm_handle = NULL;
        params->sample_base_addr = 0U;
    }
}

/**
 * \brief Initialize SDFM instance
 *
 * \param index SDFM instance index
 * \param params Pointer to SDFM initialization parameters
 *
 * \return SDFM handle on success, NULL on failure
 */
SDFM_Handle SDFM_init(uint32_t index, SDFM_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    SDFM_Handle handle = NULL;
    SDFM_Priv *priv = NULL;
    const SDFM_Attrs *attrs = NULL;
    PRUICSS_HwAttrs *pruicss_hw_attrs = NULL;
    uint8_t ch_idx;

    /* Validate index and params */
    if ((index >= gSdfmConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        /* Get handle, priv, and attrs */
        handle = (SDFM_Handle)&gSdfmHandle[index];
        priv = handle->priv;
        attrs = handle->attrs;

        /* Validate priv and attrs pointers */
        if ((priv == NULL) || (attrs == NULL))
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Check if driver already open */
        if (priv->is_open == 1U)
        {
            status = SystemP_FAILURE;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Validate params structure fields */
        if ((params->pruicss_handle == NULL) ||
            (params->sample_base_addr == 0U))
        {
            status = SystemP_FAILURE;
        }

        /* Validate attrs structure fields */
        if ((attrs->instance >= gSdfmConfigNum) ||
            (attrs->pruicss_instance > 1U) ||
            (attrs->pruicss_slice > 1U) ||
            (attrs->load_share_enabled > 1U) ||
            (attrs->channel_mask == 0U) ||
            (attrs->channel_mask > SDFM_NINE_CH_MASK) ||
            (attrs->core_clk_freq == 0U) ||
            (attrs->iep_clk_freq == 0U) ||
            (attrs->iep_reset_freq == 0U) ||
            (attrs->iep_instance > 1U) ||
            (attrs->sdfm_clock_source > SDFM_EXTERNAL_CLOCK_SRC) ||
            (attrs->sdfm_sampling_freq == 0U) ||
            (attrs->enable_epwm_sync > 1U) ||
            (attrs->pru_core_mask == 0U))
        {
            status = SystemP_FAILURE;
        }

        /* Validate PRU core configuration for each enabled core */
        if (status == SystemP_SUCCESS)
        {
            for (ch_idx = 0U; ch_idx < NUM_OF_PRU_CORE_PER_PRU_SLICE; ch_idx++)
            {
                if (attrs->pru_core_mask & (1U << ch_idx))
                {
                    if ((attrs->pru_core_config[ch_idx].enable_trigger_mode > 1U) ||
                        (attrs->pru_core_config[ch_idx].en_double_nc_sampling > 1U) ||
                        (attrs->pru_core_config[ch_idx].enable_snoop_mode > 1U) ||
                        (attrs->pru_core_config[ch_idx].iep_cmp_event > SDFM_IEP_CMP_EVENT_MAX))
                    {
                        status = SystemP_FAILURE;
                        break;
                    }
                }
            }
        }

        /* Validate channel configuration for each enabled channel */
        if (status == SystemP_SUCCESS)
        {
            for (ch_idx = 0U; ch_idx < SDFM_NUM_OF_CH_PER_PRU_SLICE; ch_idx++)
            {
                if (attrs->channel_mask & (1U << ch_idx))
                {
                    if ((attrs->channels[ch_idx].ch_id >= SDFM_NUM_OF_CH_PER_PRU_SLICE) ||
                        (attrs->channels[ch_idx].enabled > 1U) ||
                        (attrs->channels[ch_idx].filter_type > SDFM_ACC_FILTER_MAX) ||
                        (attrs->channels[ch_idx].normal_current_osr < SDFM_OSR_MIN) ||
                        (attrs->channels[ch_idx].normal_current_osr > SDFM_OSR_MAX) ||
                        (attrs->channels[ch_idx].over_current_osr < SDFM_OSR_MIN) ||
                        (attrs->channels[ch_idx].over_current_osr > SDFM_OSR_MAX) ||
                        (attrs->channels[ch_idx].sdfm_clk == 0U) ||
                        (attrs->channels[ch_idx].enable_comparator > 1U) ||
                        (attrs->channels[ch_idx].fd_enable > 1U) ||
                        (attrs->channels[ch_idx].clk_source > SDFM_CLK_SOURCE_MAX) ||
                        (attrs->channels[ch_idx].clk_inv > 1U) ||
                        (attrs->channels[ch_idx].en_zero_cross > 1U))
                    {
                        status = SystemP_FAILURE;
                        break;
                    }

                    /* Validate fast detect configuration if enabled */
                    if (attrs->channels[ch_idx].fd_enable == 1U)
                    {
                        if ((attrs->channels[ch_idx].fd_window < SDFM_FD_WINDOW_SIZE_MIN) ||
                            (attrs->channels[ch_idx].fd_window > SDFM_FD_WINDOW_SIZE_MAX) ||
                            (attrs->channels[ch_idx].fd_zero_max > SDFM_FD_THRESHOLD_MAX) ||
                            (attrs->channels[ch_idx].fd_zero_min > SDFM_FD_THRESHOLD_MAX))
                        {
                            status = SystemP_FAILURE;
                            break;
                        }
                    }
                }
            }
        }
    }

    if (status != SystemP_SUCCESS)
    {
        return NULL;
    }
    /* Initialize PRU DMEM interface address based on PRU slice */
    pruicss_hw_attrs = (PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs);
    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        priv->sdfm_interface = (SDFM_Interface *)(pruicss_hw_attrs->pru0DramBase);
    }
    else 
    {
        priv->sdfm_interface = (SDFM_Interface *)(pruicss_hw_attrs->pru1DramBase);
    }

    /* Store handles in priv */
    priv->pruicss_handle = params->pruicss_handle;
    priv->pwm_handle = params->pwm_handle;

    /* Initialize sample output interface */
    priv->sampleOutputInterface = (SDFM_SampleOutInterface *)params->sample_base_addr;
    
    /* Enable SDFM mode */
    status = PRUICSS_setGpMuxSelect(params->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_SD);
    if(status != SystemP_SUCCESS)
    {
        return NULL;
    }

    /* Enable load share mode if configured */
    if (attrs->load_share_enabled == 1U)
    {
        SDFM_enableLoadShareMode(handle, attrs->pruicss_slice);
    }

    /* Mark driver as open */
    priv->is_open = 1U;

    return handle;
}

/**
 * \brief Deinitialize SDFM instance and release resources
 *
 * \param handle SDFM handle
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t SDFM_deinit(SDFM_Handle handle)
{
    SDFM_Priv *priv;

    /* Validate handle */
    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Check if driver is open */
    if (priv->is_open != 1U)
    {
        return SystemP_FAILURE;
    }

    /* Mark driver as closed */
    priv->is_open = 0U;

    return SystemP_SUCCESS;
}

const SDFM_Attrs* SDFM_getAttrs(SDFM_Handle handle)
{
    if (handle == NULL)
    {
        return NULL;
    }

    return handle->attrs;
}

SDFM_Priv* SDFM_getPriv(SDFM_Handle handle)
{
    if (handle == NULL)
    {
        return NULL;
    }

    return handle->priv;
}

/* Configuration of IEP reset cycle time period */
int32_t SDFM_configIepCount(SDFM_Handle handle, uint32_t iep_reset_freq)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint32_t max_iep_cnt;

    if (handle == NULL || iep_reset_freq == 0U)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Max IEP0 count value for one EPWM period */
    max_iep_cnt = (attrs->iep_clk_freq / iep_reset_freq) * IEP_DEFAULT_INC;
    priv->sdfm_interface->trigger_config[SDFM_PRU_CORE_INDEX].max_iep_cnt_per_epwm_prd = max_iep_cnt;
    priv->sdfm_interface->trigger_config[SDFM_RTUPRU_CORE_INDEX].max_iep_cnt_per_epwm_prd = max_iep_cnt;
    priv->sdfm_interface->trigger_config[SDFM_TXPRU_CORE_INDEX].max_iep_cnt_per_epwm_prd = max_iep_cnt;

    return SystemP_SUCCESS;
}

/* ECAP configuration for SD clock */
int32_t SDFM_configEcap(SDFM_Handle handle, uint8_t ecap_divider)
{
    SDFM_Priv *priv;
    void *pruicss_ecap;
    uint32_t rgval;
    uint32_t count;

    if (handle == NULL || ecap_divider == 0U)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_ecap = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->ecapRegBase);

    /* Set eCAP APWM mode */
    rgval = HW_RD_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |= (0U << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCI_EN_SHIFT) |
             (2U << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCO_SEL_SHIFT) |
             (1U << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_CAP_APWM_SHIFT) |
             (0U << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_APWMPOL_SHIFT);
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

    /* Set period count */
    count = ecap_divider - 1U;
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP1, count);

    /* Compute & set Duty Cycle count.
     * Divide period count by 2, biased rounding. */
    count = count + 1U;
    count = count / 2U;
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP2, count);

    /* Clear counter phase and Reset eCAP PWM Counter */
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CNTPHS, 0U);
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_TSCNT, 0U);

    /* Enable eCAP APWM */
    rgval = HW_RD_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |= (1U << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_TSCNTSTP_SHIFT);
    HW_WR_REG32((uint8_t *)pruicss_ecap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

    return SystemP_SUCCESS;

}

/* SDFM HW OSR configuration */
int32_t SDFM_setCompFilterOverSamplingRatio(SDFM_Handle handle, uint8_t channel, uint16_t osr)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    PRUICSS_HwAttrs const *hw_attrs;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    /* Validate OSR against hardware register limits */
    if ((osr < SDFM_OSR_MIN) || (osr > SDFM_OSR_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    hw_attrs = (PRUICSS_HwAttrs const *)((priv->pruicss_handle)->hwAttrs);
    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0, osr - 1U);
    }
    else if (attrs->pruicss_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0, osr - 1U);
    }
    else
    {
        return SystemP_FAILURE;
    }

    /* Over current OSR */
    priv->sdfm_interface->channels[channel].over_current_osr = osr - 1U;
    return SystemP_SUCCESS;
}

/* SDFM high, low threshold config */
int32_t SDFM_setCompFilterThresholds(SDFM_Handle handle, uint8_t channel, SDFM_ThresholdConfig threshold_config)
{
    SDFM_Priv *priv;

    if ((handle == NULL) || (channel > SDFM_CHANNEL8))
    {
        return SystemP_FAILURE;
    }

    /* Validate threshold values */
    if ((threshold_config.high_threshold > SDFM_THRESHOLD_MAX) ||
        (threshold_config.low_threshold > SDFM_THRESHOLD_MAX) ||
        (threshold_config.high_threshold <= threshold_config.low_threshold))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* SD Over current high threshold */
    priv->sdfm_interface->channels[channel].threshold_config.high_threshold = threshold_config.high_threshold;
    /* SD Over current low threshold */
    priv->sdfm_interface->channels[channel].threshold_config.low_threshold = threshold_config.low_threshold;
    return SystemP_SUCCESS;
}

/* SDFM sampling time configuration */
int32_t SDFM_setSampleTriggerTime(SDFM_Handle handle, float samp_trig_time, uint8_t pru_core)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint32_t count;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Convert sample time into IEP count */
    /* Sample time in us */
    count = (uint32_t)((attrs->iep_clk_freq / 1000000U) * samp_trig_time);
    priv->sdfm_interface->trigger_config[pru_core].first_samp_trig_time = count;
    return SystemP_SUCCESS;
}
/* Second normal current sampling configuration */
int32_t SDFM_enableDoubleSampling(SDFM_Handle handle, float samp_trig_time, uint8_t pru_core)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint32_t count;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Enable double normal current sampling */
    priv->sdfm_interface->trigger_config[pru_core].en_double_nc_sampling = 1U;
    /* Second sample point */
    count = (uint32_t)((attrs->iep_clk_freq / 1000000U) * samp_trig_time);
    priv->sdfm_interface->trigger_config[pru_core].second_samp_trig_time = count;

    return SystemP_SUCCESS;
}

/* Disable double update */
int32_t SDFM_disableDoubleSampling(SDFM_Handle handle, uint8_t pru_core)
{
    SDFM_Priv *priv;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->sdfm_interface->trigger_config[pru_core].en_double_nc_sampling = 0U;
    return SystemP_SUCCESS;
}
/* Enable the channel specified by the channel number parameter */
int32_t SDFM_setEnableChannel(SDFM_Handle handle, uint8_t channel_number)
{
    SDFM_Priv *priv;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (channel_number > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->sdfm_interface->channels[channel_number].ch_id = channel_number;
    priv->sdfm_interface->channels[channel_number].enabled = 1U;
    priv->sdfm_interface->active_channels_mask |= (1U << channel_number);

    return SystemP_SUCCESS;
}
/* set SDFM channel acc source */
int32_t SDFM_configDataFilter(SDFM_Handle handle, uint8_t channel, uint8_t filter)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    PRUICSS_HwAttrs const *hw_attrs;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    /* Validate filter selection against hardware register limits */
    if (filter > SDFM_ACC_FILTER_MAX)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    hw_attrs = (PRUICSS_HwAttrs const *)((priv->pruicss_handle)->hwAttrs);
    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC_SEL0, filter);
    }
    else if (attrs->pruicss_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC_SEL0, filter);
    }
    else
    {
        return SystemP_FAILURE;
    }

    priv->sdfm_interface->channels[channel].filter_type = filter;
    return SystemP_SUCCESS;
}

/* Set clock source for SDFM channel */
int32_t SDFM_selectClockSource(SDFM_Handle handle, uint8_t channel, uint8_t clk_source)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    PRUICSS_HwAttrs const *hw_attrs;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    /* Validate clock source selection against hardware register limits */
    if (clk_source > SDFM_CLK_SOURCE_MAX)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    hw_attrs = (PRUICSS_HwAttrs const *)((priv->pruicss_handle)->hwAttrs);
    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0, clk_source);
    }
    else if (attrs->pruicss_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0, clk_source);
    }
    else
    {
        return SystemP_FAILURE;
    }

    priv->sdfm_interface->channels[channel].clk_source = clk_source;
    return SystemP_SUCCESS;
}
/* Set clock inversion for SDFM channel */
int32_t SDFM_setClockInversion(SDFM_Handle handle, uint8_t channel, uint8_t clk_inv)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    PRUICSS_HwAttrs const *hw_attrs;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if ((channel > SDFM_CHANNEL8) || (clk_inv > SDFM_CLK_INV_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    hw_attrs = (PRUICSS_HwAttrs const *)((priv->pruicss_handle)->hwAttrs);
    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0, clk_inv);
    }
    else if (attrs->pruicss_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (channel * 8U)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0, clk_inv);
    }
    else
    {
        return SystemP_FAILURE;
    }

    priv->sdfm_interface->channels[channel].clk_inv = clk_inv;

    return SystemP_SUCCESS;
}
/* Enable the comparator feature for a specified filter/channel */
int32_t SDFM_enableComparator(SDFM_Handle handle, uint8_t channel)
{
    SDFM_Priv *priv;
    uint8_t pwm_set;
    uint16_t trip_mask;
    int32_t ret_val;

    /* Validate input parameters */
    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;

    if (channel < SDFM_CHANNEL3)
    {
        pwm_set = 0U;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1U;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2U;
    }
    else
    {
        return SystemP_FAILURE;
    }

    ret_val = PRUICSS_PWM_getPwmTripMask(priv->pwm_handle, pwm_set, &trip_mask);
    if (ret_val != SystemP_SUCCESS)
    {
        return ret_val;
    }
    trip_mask |= 0x2U; /* Set the trip mask for over current trip */

    ret_val = PRUICSS_PWM_setPwmTripMask(priv->pwm_handle, pwm_set, trip_mask);

    if (ret_val != SystemP_SUCCESS)
    {
        return ret_val;
    }
    priv->sdfm_interface->channels[channel].enable_comparator = 1U;
    return ret_val;
}

/* Disable the comparator feature for a specified filter/channel */
int32_t SDFM_disableComparator(SDFM_Handle handle, uint8_t channel)
{
    SDFM_Priv *priv;
    uint8_t pwm_set;
    uint16_t trip_mask;
    int32_t ret_val;

    /* Validate input parameters */
    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;

    if (channel < SDFM_CHANNEL3)
    {
        pwm_set = 0U;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1U;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2U;
    }
    else
    {
        return SystemP_FAILURE;
    }

    ret_val = PRUICSS_PWM_getPwmTripMask(priv->pwm_handle, pwm_set, &trip_mask);
    if (ret_val != SystemP_SUCCESS)
    {
        return ret_val;
    }
    trip_mask &= 0xFFFDU; /* Clear the trip mask for over current trip */

    ret_val = PRUICSS_PWM_setPwmTripMask(priv->pwm_handle, pwm_set, trip_mask);

    if (ret_val != SystemP_SUCCESS)
    {
        return ret_val;
    }
    priv->sdfm_interface->channels[channel].enable_comparator = 0U;

    return ret_val;
}
/* GPIO configuration */
int32_t SDFM_configComparatorGpioPins(SDFM_Handle handle, uint8_t channel, uint32_t gpio_base_addr, uint32_t pin_number)
{
    SDFM_Priv *priv;
    volatile CSL_GpioRegs *h_gpio;
    uint32_t reg_index;
    uint32_t reg_val;
    uint32_t clr_data_addr;
    uint32_t set_data_addr;

    /* Validate input parameters */
    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    if (gpio_base_addr == 0)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;

    h_gpio = (volatile CSL_GpioRegs *)((uintptr_t)gpio_base_addr);
    reg_index = GPIO_GET_REG_INDEX(pin_number);
    reg_val = GPIO_GET_BIT_MASK(pin_number);
    clr_data_addr = (uint32_t)&h_gpio->BANK_REGISTERS[reg_index].CLR_DATA;
    set_data_addr = (uint32_t)&h_gpio->BANK_REGISTERS[reg_index].SET_DATA;

    priv->sdfm_interface->channels[channel].gpio_params.write_val = reg_val;
    priv->sdfm_interface->channels[channel].gpio_params.set_val_addr = set_data_addr;
    priv->sdfm_interface->channels[channel].gpio_params.clr_val_addr = clr_data_addr;

    return SystemP_SUCCESS;
}

/* Get current (or latest) sample for the specified channel */
uint32_t SDFM_getFilterData(SDFM_Handle handle, uint8_t channel)
{
    SDFM_Priv *priv;

    if (handle == NULL)
    {
        return 0U;
    }

    if (channel > SDFM_CHANNEL8)
    {
        return 0U;
    }

    priv = handle->priv;

    return (uint32_t)(priv->sampleOutputInterface->sampleOutput[channel]);
}

/* Configure normal current OSR for data filter */
int32_t SDFM_setFilterOverSamplingRatio(SDFM_Handle handle, uint8_t channel, uint16_t nc_osr)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint8_t pru_core;
    uint16_t count;
    uint32_t iep_freq;
    uint32_t sd_clock;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Validate OSR against hardware register limits */
    if ((nc_osr < SDFM_OSR_MIN) || (nc_osr > SDFM_OSR_MAX))
    {
        return SystemP_FAILURE;
    }

    if (channel < SDFM_CHANNEL3)
    {
        pru_core = 0U;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pru_core = 1U;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pru_core = 2U;
    }
    else
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if (priv->sdfm_interface->control[pru_core].enable_snoop_nc == 1U)
    {
        /* IEP0 counts in normal current sampling period */
        iep_freq = attrs->iep_clk_freq;
        sd_clock = attrs->sdfm_sampling_freq;
        count = (uint16_t)((float)nc_osr * ((float)iep_freq / (float)sd_clock));
        priv->sdfm_interface->trigger_config[pru_core].nc_prd_iep_cnt = count;
    }
    else
    {
        /* Setting SDFM hardware OSR for normal current without snoop mode */
        SDFM_setCompFilterOverSamplingRatio(handle, channel, nc_osr);
    }

    priv->sdfm_interface->channels[channel].normal_current_osr = nc_osr - 1;
    return SystemP_SUCCESS;
}
/* Return firmware version */
uint32_t SDFM_getFirmwareVersion(SDFM_Handle handle)
{
    SDFM_Priv *priv;

    if (handle == NULL)
    {
        return 0U;
    }

    priv = handle->priv;

    return priv->sdfm_interface->firmwareVersion >> SDFM_FW_VERSION_BIT_SHIFT;
}
/* Trigger based normal current */
int32_t SDFM_enableTriggerModeForNormalCurrent(SDFM_Handle handle, uint8_t pru_core)
{
    SDFM_Priv *priv;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->sdfm_interface->trigger_config[pru_core].enable_trigger_mode = 1U;
    return SystemP_SUCCESS;
}
/* FD block configuration */
int32_t SDFM_configFastDetect(SDFM_Handle handle, uint8_t channel, SDFM_FastDetectConfig fast_detect_config)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    PRUICSS_HwAttrs const *hw_attrs;
    uint8_t pwm_set;
    uint16_t trip_mask;
    int32_t ret_val;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Validate fast detect parameters against hardware register limits */
    if ((fast_detect_config.fd_enable > 1U) ||
        (fast_detect_config.fd_window_size < SDFM_FD_WINDOW_SIZE_MIN) ||
        (fast_detect_config.fd_window_size > SDFM_FD_WINDOW_SIZE_MAX) ||
        (fast_detect_config.fd_zero_max > SDFM_FD_THRESHOLD_MAX) ||
        (fast_detect_config.fd_zero_max < SDFM_FD_THRESHOLD_MIN) ||
        (fast_detect_config.fd_zero_min > SDFM_FD_THRESHOLD_MAX) ||
        (fast_detect_config.fd_zero_min < SDFM_FD_THRESHOLD_MIN))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if (channel < SDFM_CHANNEL3)
    {
        pwm_set = 0U;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1U;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2U;
    }
    else
    {
        return SystemP_FAILURE;
    }

    priv->sdfm_interface->channels[channel].fd_enable = fast_detect_config.fd_enable;
    priv->sdfm_interface->channels[channel].fd_window = fast_detect_config.fd_window_size;
    priv->sdfm_interface->channels[channel].fd_zero_max = fast_detect_config.fd_zero_max - 1U;
    priv->sdfm_interface->channels[channel].fd_zero_min = fast_detect_config.fd_zero_min - 1U;

    /* Configure one max to window size + 1 and one min to 0, so they never get set */
    priv->sdfm_interface->channels[channel].fd_one_max = (fast_detect_config.fd_window_size + 1U) * 4U + 1U;
    priv->sdfm_interface->channels[channel].fd_one_min = 0U;

    hw_attrs = (PRUICSS_HwAttrs const *)((priv->pruicss_handle)->hwAttrs);

    if (attrs->pruicss_slice == PRUICSS_PRU0)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_FD_ZERO_MAX_LIMIT_0, priv->sdfm_interface->channels[channel].fd_zero_max);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0CLKSELREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_FD_ZERO_MIN_LIMIT_0, priv->sdfm_interface->channels[channel].fd_zero_min);

        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_EN_0, 1);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_ONE_MAX_LIMIT_0,  priv->sdfm_interface->channels[channel].fd_one_max);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_ONE_MIN_LIMIT_0, priv->sdfm_interface->channels[channel].fd_one_min);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_FD_WINDOW_SIZE_0, priv->sdfm_interface->channels[channel].fd_window);
    }
    else if (attrs->pruicss_slice == PRUICSS_PRU1)
    {
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_FD_ZERO_MAX_LIMIT_0, priv->sdfm_interface->channels[channel].fd_zero_max);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1CLKSELREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_FD_ZERO_MIN_LIMIT_0, priv->sdfm_interface->channels[channel].fd_zero_min);

        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_EN_0, 1);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_ONE_MAX_LIMIT_0,  priv->sdfm_interface->channels[channel].fd_one_max);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_ONE_MIN_LIMIT_0, priv->sdfm_interface->channels[channel].fd_one_min);
        HW_WR_FIELD32((hw_attrs->cfgRegBase + CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0 + (channel * 8)),
        CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_FD_WINDOW_SIZE_0, priv->sdfm_interface->channels[channel].fd_window);
    }
    else
    {
        return SystemP_FAILURE;
    }

    ret_val = PRUICSS_PWM_getPwmTripMask(priv->pwm_handle, pwm_set, &trip_mask);
    if(ret_val != SystemP_SUCCESS)
    {
        return ret_val;
    }
    trip_mask |= (1<<(channel+2)); /* Set the trip mask for fast detect trip */

    ret_val = PRUICSS_PWM_setPwmTripMask(priv->pwm_handle, pwm_set, trip_mask);

    return ret_val;
}

/* Return status of PWM trip vector status bit */
int32_t SDFM_getFastDetectErrorStatus(SDFM_Handle handle, uint8_t channel)
{
    uint8_t pwm_set;
    int32_t ret_val = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle;
    uint32_t temp;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(channel < SDFM_CHANNEL3)
    {
        pwm_set = 0;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2;
    }
    else
    {
        ret_val = SystemP_FAILURE;
    }

    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }

    pwm_handle = handle->priv->pwm_handle;

    /* PWM trip vector */
    ret_val = PRUICSS_PWM_getPwmTripTriggerCauseVector(pwm_handle, pwm_set);
    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }
    else
    {

        ret_val =  ret_val >> 2;
        temp  = 1 << channel;
        if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
        {
            return ((ret_val) & (1 << SDFM_CHANNEL0)) ? 1 : 0;
        }
        else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
        {
            return ((ret_val) & (1 << SDFM_CHANNEL1))? 1 : 0;
        }
        else
        {
            return ((ret_val) & (1 << SDFM_CHANNEL2)) ? 1 : 0;
        }

    }

}

/* Clear Trip status bit */
int32_t SDFM_clearPwmTripStatus(SDFM_Handle handle, uint8_t channel)
{
    uint8_t pwm_set;
    int32_t ret_val = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(channel < SDFM_CHANNEL3)
    {
        pwm_set = 0;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2;
    }
    else
    {
        ret_val = SystemP_FAILURE;
    }

    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }

    pwm_handle = handle->priv->pwm_handle;

    /* Clear trip status */
    ret_val = PRUICSS_PWM_generatePwmTripReset(pwm_handle, pwm_set);
    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }

    /* Clear trip reset status */
    ret_val = PRUICSS_PWM_clearPwmTripResetStatus(pwm_handle, pwm_set);

    return ret_val;
}
/* Enable Load share mode (static helper function) */
static int32_t SDFM_enableLoadShareMode(SDFM_Handle handle, uint8_t sliceId)
{
    SDFM_Priv *priv;
    void *pruicss_cfg;
    uint32_t rgval;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    if (sliceId != 0U)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_SDPRU1CLKDIV);
        rgval |= CSL_ICSSCFG_SDPRU1CLKDIV_PRU1_SD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_SDPRU1CLKDIV, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_SDPRU0CLKDIV);
        rgval |= CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_SDPRU0CLKDIV, rgval);
    }

    return SystemP_SUCCESS;
}
/* Measure Phase delay */
int32_t SDFM_measureClockPhaseDelay(SDFM_Handle handle, uint16_t clk_edg, uint8_t channel)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint16_t n_edge;
    float temp;
    uint8_t ack;
    uint32_t pru_cycles;
    uint32_t i;

    if(channel > SDFM_CHANNEL8 || clk_edg > 1)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Enable phase delay measurement */
    priv->sdfm_interface->channels[channel].en_phase_delay = 1;
    /* Waiting till measurement done with timeout */
    for(i = 0; i < SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT; i++)
    {
       ack = priv->sdfm_interface->channels[channel].en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK;
       if(!ack)
       {
           break;
       }
       ClockP_usleep(SDFM_DEFAULT_FW_WAIT_DELAY_US);
    }

    /* Check for timeout */
    if(i >= SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT)
    {
        return SystemP_TIMEOUT;
    }

   n_edge = priv->sdfm_interface->channels[channel].clock_edge;
   temp = priv->sdfm_interface->channels[channel].clock_phase_delay;
   /* Average */
    temp = temp/SDFM_PHASE_DELAY_CAL_LOOP_SIZE;
   /* Check data reading edge(clk polarity) & nearest edge */
   if(n_edge == clk_edg)
   {
      /* PRU cycles for half SD clock period */
      pru_cycles = ceil(((float)attrs->core_clk_freq)/(2*priv->sdfm_interface->channels[channel].sdfm_clk));
      priv->sdfm_interface->channels[channel].clock_phase_delay = pru_cycles - temp;
   }
   else
   {
      /* PRU cycles for one SD clock period */
      pru_cycles = ceil((float)(attrs->core_clk_freq/(priv->sdfm_interface->channels[channel].sdfm_clk)));
      priv->sdfm_interface->channels[channel].clock_phase_delay = pru_cycles - temp;
   }

   return SystemP_SUCCESS;
}
float SDFM_getClockPhaseDelay(SDFM_Handle handle, uint8_t channel)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    float phase_delay;

    if (handle == NULL || channel > SDFM_CHANNEL8)
    {
        return 0.0f;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Conversion from PRU cycle to ns */
    phase_delay =  ((float)priv->sdfm_interface->channels[channel].clock_phase_delay * SDFM_NANOSECONDS_PER_SECOND)/attrs->core_clk_freq;
    return phase_delay;
}
int32_t SDFM_getHighThresholdStatus(SDFM_Handle handle, uint8_t channel)
{
    if(channel > SDFM_CHANNEL8 || handle == NULL)
    {
        return SystemP_FAILURE;
    }
    else
    {
        return handle->priv->sdfm_interface->channels[channel].threshold_config.high_th_status;
    }
}
int32_t SDFM_getLowThresholdStatus(SDFM_Handle handle, uint8_t channel)
{
    if(channel > SDFM_CHANNEL8 || handle == NULL)
    {
        return SystemP_FAILURE;
    }
    else
    {
        return  handle->priv->sdfm_interface->channels[channel].threshold_config.low_th_status;
    }
}

int32_t SDFM_clearOverCurrentError(SDFM_Handle handle, uint8_t channel)
{
    uint8_t pwm_set;
    int32_t ret_val = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pwm_handle;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(channel < SDFM_CHANNEL3)
    {
        pwm_set = 0;
    }
    else if (channel > SDFM_CHANNEL2 && channel < SDFM_CHANNEL6)
    {
        pwm_set = 1;
    }
    else if (channel > SDFM_CHANNEL5 && channel <= SDFM_CHANNEL8)
    {
        pwm_set = 2;
    }
    else
    {
        ret_val = SystemP_FAILURE;
    }

    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }

    pwm_handle = handle->priv->pwm_handle;

    /* Clear over current Error PWM trip */
    ret_val = PRUICSS_PWM_clearPwmOverCurrentErrorTrip(pwm_handle, pwm_set);
    if(ret_val == SystemP_FAILURE)
    {
        return ret_val;
    }

    /* Clear PWM trip */
    ret_val = SDFM_clearPwmTripStatus(handle, channel);
    return ret_val;
}
int32_t SDFM_enableZeroCrossDetection(SDFM_Handle handle, uint8_t channel, uint32_t zc_thr)
{
    SDFM_Priv *priv;

    if ((handle == NULL) || (channel > SDFM_CHANNEL8))
    {
        return SystemP_FAILURE;
    }

    /* Validate zero cross threshold value */
    if (zc_thr > SDFM_THRESHOLD_MAX)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->sdfm_interface->channels[channel].threshold_config.en_zero_cross = 1;
    priv->sdfm_interface->channels[channel].threshold_config.zero_cross_threshold = zc_thr;

    return SystemP_SUCCESS;
}
int32_t SDFM_getZeroCrossThresholdStatus(SDFM_Handle handle, uint8_t channel)
{
    if(handle == NULL || channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    return handle->priv->sdfm_interface->channels[channel].threshold_config.zero_cross_th_status;
}
int32_t SDFM_disableZeroCrossDetection(SDFM_Handle handle, uint8_t channel)
{
    SDFM_Priv *priv;

    if(handle == NULL || channel > SDFM_CHANNEL8)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->sdfm_interface->channels[channel].threshold_config.en_zero_cross = 0;

    return SystemP_SUCCESS;
}

int32_t SDFM_enableEpwmSync(SDFM_Handle handle, uint8_t epwm_ins)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void *pru_iep;
    int32_t ret_val = SystemP_FAILURE;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(epwm_ins != 0 && epwm_ins != 3)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pru_iep != NULL)
    {
        ret_val = SystemP_SUCCESS;

        switch (epwm_ins)
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

    return ret_val;
}

int32_t SDFM_disableEpwmSync(SDFM_Handle handle, uint8_t epwm_ins)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void *pru_iep;
    int32_t ret_val = SystemP_FAILURE;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(epwm_ins != 0 && epwm_ins != 3)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pru_iep != NULL)
    {
        ret_val = SystemP_SUCCESS;

        switch (epwm_ins)
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

    return ret_val;
}

int32_t SDFM_configIepSyncMode(SDFM_Handle handle, uint32_t high_pulse_width, uint32_t period_time, uint32_t sync_start_time)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void        *pru_iep;
    int32_t     ret_val = SystemP_FAILURE;
    uint32_t    reg_val ;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        pru_iep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pru_iep != NULL)
    {

        /* Set CMP1 period - SYNC0 trigger */
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, sync_start_time);

        /* Set CMP2 period - SYNC1 trigger */
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, sync_start_time);

        /* Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable */
        reg_val = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG);
        reg_val |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_EN_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_EN_SHIFT);
        reg_val |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_SHIFT) | (0<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_IND_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, reg_val);

        /* Set SYNC0/1 high pulse time in IEP clock cycles */
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, high_pulse_width);

        /* Set SYNC0/1 period */
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, period_time);

        /* Set offset from cpm hit */
        HW_WR_REG32( (uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

        /* Enable cmp1 and cmp2 for sync start trigger generation */
        reg_val = HW_RD_REG8((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
        reg_val |= (1<<SDFM_IEP_CMP1_EN_SHIFT)|(1<<SDFM_IEP_CMP2_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, reg_val);

        /* Set default and compensation increment to 1 */
        reg_val = HW_RD_REG32((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        reg_val |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT );
        HW_WR_REG8((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, reg_val);

        ret_val = SystemP_SUCCESS;

    }

    return ret_val;
}

int32_t SDFM_enableIep(SDFM_Handle handle)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void       *pruIep;
    int32_t    retVal = SystemP_FAILURE;
    uint32_t   regVal ;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pruIep != NULL)
    {
        /* IEP Counter increment value */
        HW_WR_FIELD32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG,
        CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC, IEP_DEFAULT_INC);
        /* Start iep0_timer */
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        regVal |= 0x1;
        HW_WR_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t SDFM_configSync1Delay(SDFM_Handle handle, uint32_t delay)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void      *pruIep;
    int32_t   retVal = SystemP_FAILURE;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }

    if(pruIep != NULL)
    {
        /* Set delay between SYNC0 and SYNC1 in clock cycles */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, delay);

        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t SDFM_configClockFromGPO1(SDFM_Handle handle, uint8_t div0, uint8_t div1)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    uint32_t rgval;
    void *pruicssCfg;
    int32_t   retVal = SystemP_SUCCESS;

    /* Validate input parameters */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(div0 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MAX || div1 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MAX)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;
    attrs = handle->attrs;

    pruicssCfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    /* Configure divider */
    if(attrs->pruicss_slice == 1)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);

        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);

        /* Enabling shift mode */
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MAX<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);
    }
    else if (attrs->pruicss_slice == 0)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);

        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);

         /* Enabling shift mode */
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

/* Enable snoop based NC sampling */
int32_t SDFM_enableSnoopBasedNC(SDFM_Handle handle, uint8_t pru_core)
{
    SDFM_Priv *priv;

    if(handle == NULL || pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Enable snoop based NC sampling */
    priv->sdfm_interface->control[pru_core].enable_snoop_nc = 1;

    return SystemP_SUCCESS;
}

/* Disable snoop based NC sampling */
int32_t SDFM_disableSnoopBasedNC(SDFM_Handle handle, uint8_t pru_core)
{
    SDFM_Priv *priv;

    if(handle == NULL || pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Disable snoop based NC sampling */
    priv->sdfm_interface->control[pru_core].enable_snoop_nc = 0;
    return SystemP_SUCCESS;
}
int32_t SDFM_selectIepCmpEvent(SDFM_Handle handle, uint8_t event, uint8_t pru_core)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void      *pruIep;
    uint32_t  iep_cmp_reg;
    uint32_t  iep_cmp_status_reg;
    uint32_t regVal;

    if(handle == NULL || pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE || event > SDFM_IEP_CMP_EVENT_MAX)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        /* Enable IEP0 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
        iep_cmp_reg = CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + (event * 8);
        iep_cmp_status_reg = CSL_ICSS_G_PR1_IEP0_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG;
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        /* Enable IEP1 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
        iep_cmp_reg = CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0 + (event * 8);
        iep_cmp_status_reg = CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG;
    }
    else
    {
        return SystemP_FAILURE;
    }

    /* Configure the compare event*/
    /* Read the current register value */
    regVal = HW_RD_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG));
    /* Set the CMP_EN bit (OR with the new value) */
    regVal |= ((uint32_t)1U << event) << CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), regVal);

    /*
     * Adjust IEP compare register offset for events 8-15:
     *
     * The IEP compare event registers have a non-contiguous layout in memory:
     * - CMP0-CMP7  : Continuous
     * - Gap        : Offsets 0xB8-0xC0 are used for other registers (CSL_ICSS_G_PR1_IEP0_SLV_RXIPG0_REG, CSL_ICSS_G_PR1_IEP0_SLV_RXIPG1_REG)
     * - CMP8-CMP15 : Continuous
     *
     */
    if(event > SDFM_IEP_CMP_EVENT_CMP7)
    {
        iep_cmp_reg += SDFM_IEP_CMP_REG_GAP_SIZE;  /* Skip 8-byte gap (2 reserved registers) after CMP7 */
    }

   /* Select IEP CMP event as trigger source */
    priv->sdfm_interface->trigger_config[pru_core].iep_cmp_event = event;
    priv->sdfm_interface->trigger_config[pru_core].iep_cmp_event_reg = iep_cmp_reg;
    priv->sdfm_interface->trigger_config[pru_core].iep_cmp_status_reg = iep_cmp_status_reg;

    return SystemP_SUCCESS;
}

int32_t SDFM_configIepCmp0ToResetIep(SDFM_Handle handle, uint32_t iep_reset_freq)
{
    SDFM_Priv *priv;
    const SDFM_Attrs *attrs;
    void      *pruIep;
    int64_t    iep_count;
    uint32_t regVal;

    if(handle == NULL || iep_reset_freq == 0)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->iep_instance == PRUICSS_IEP_INST0)
    {
        /* Enable IEP0 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep0RegBase);
    }
    else if(attrs->iep_instance == PRUICSS_IEP_INST1)
    {
        /* Enable IEP1 as trigger source */
        pruIep = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->iep1RegBase);
    }
    else
    {
        return SystemP_FAILURE;
    }
    iep_count = attrs->iep_clk_freq / iep_reset_freq;
    /* Configure the cmp0 register to generate reset at required frequency */
    HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, (iep_count & 0xffffffff) - 1);
    HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG1, (iep_count>>32) & 0xffffffff);

    /* Read the current register value */
    regVal = HW_RD_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG));

    /* Set the CMP_EN bit (OR with the new value) */
    regVal |= ((uint32_t)1U << 0) << CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP_EN_SHIFT;

    /* Write back the modified value */
    HW_WR_REG32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), regVal);

    /* Select IEP CMP0 to reset iep counter */
    HW_WR_FIELD32(((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG),
                        CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN, 1);

    return SystemP_SUCCESS;
}

int32_t SDFM_setSampleOutputInterfaceGlobalAddr(SDFM_Handle handle, uint32_t addr)
{
    SDFM_Priv *priv;

    /* Validate input parameters */
    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (addr == 0)
    {
        return SystemP_FAILURE;
    }

    /* Assign variables after validation */
    priv = handle->priv;

    priv->sdfm_interface->trigger_config[0].sample_buff_base_addr = addr;
    priv->sdfm_interface->trigger_config[1].sample_buff_base_addr = addr + 12U;
    priv->sdfm_interface->trigger_config[2].sample_buff_base_addr = addr + 24U;

    return SystemP_SUCCESS;
}

/* SDFM global enable */
int32_t SDFM_enable(SDFM_Handle handle, uint8_t pru_core)
{
    SDFM_Priv *priv;
    uint8_t sdfm_en_ack;
    uint32_t i;

    if (handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if (pru_core >= NUM_OF_PRU_CORE_PER_PRU_SLICE)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Enable SDFM */
    priv->sdfm_interface->control[pru_core].enable = 1U;

    /* wait for ACK with timeout */
    for(i = 0; i < SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT; i++)
    {
        sdfm_en_ack = priv->sdfm_interface->control[pru_core].enable_ack;
        if(sdfm_en_ack == BF_SDFM_EN_ENABLE)
        {
            break;
        }
        ClockP_usleep(SDFM_DEFAULT_FW_WAIT_DELAY_US);
    }

    /* Check for timeout */
    if(i >= SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT)
    {
        return SystemP_TIMEOUT;
    }

    return SystemP_SUCCESS;
}