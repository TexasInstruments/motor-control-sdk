/*
 *  Copyright (C) 2024-2026 Texas Instruments Incorporated
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

#include <position_sense/bissc/include/bissc_api.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern uint32_t gBisscConfigNum;
extern bissc_config gBisscHandle[];

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \brief Calculate 4-bit CRC for BiSS-C control communication command
 *
 * \param ctrl_cmd      Control command bits
 * \param num_bits      Number of bits in the command
 *
 * \return Calculated 4-bit CRC value
 */
static uint8_t bissc_calc_ctrl_crc(uint32_t ctrl_cmd, uint8_t num_bits);

/**
 * \brief Configure the receiver for Three Channel Peripheral mode of PRU-ICSS
 *
 * \param handle        BiSS-C handle
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE if setting fails
 */
static int32_t bissc_config_endat_mode(bissc_handle handle);

/**
 * \brief Clear the channel specific frame size configuration registers
 *
 * \param handle        BiSS-C handle
 *
 */
static void bissc_config_clr_cfg0(bissc_handle handle);

/**
 * \brief Configure the primary core for load share mode
 *
 * \param handle        BiSS-C handle
 * \param mask          channel mask
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE if invalid mask is passed
 */
static int32_t bissc_config_primary_core_mask(bissc_handle handle, uint8_t mask);

/**
 * \brief Enable load share mode for BiSS-C receiver
 *
 * \param handle        BiSS-C handle
 *
 */
static void bissc_enable_load_share_mode(bissc_handle handle);

/**
 * \brief Configure IEP base address in PRU shared memory
 *
 * \param handle            BiSS-C handle
 * \param iep_base_address  IEP base address offset from PRU-ICSS base
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
static int32_t bissc_config_iep_base_address(bissc_handle handle, uint32_t iep_base_address);

/**
 * \brief Configure the channels to be used by BiSS-C receiver in load share mode
 *
 * \param handle        BiSS-C handle
 * \param mask          channel mask
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
static int32_t bissc_config_load_share(bissc_handle handle, uint8_t mask);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default bissc parameters structure */
const bissc_params gBisscDefaultParams =
{
    NULL,                               /* pruicss_handle */
    BISSC_DEFAULT_CMD_PROCESS_DELAY_US, /* cmd_process_delay_us */
    BISSC_DEFAULT_FW_WAIT_DELAY_US,     /* fw_wait_delay_us */
    BISSC_DEFAULT_MAX_WAIT_LOOP_COUNT,  /* max_wait_loop_count */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void bissc_params_init(bissc_params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = gBisscDefaultParams;
    }
}

bissc_handle bissc_init(uint32_t index, const bissc_params *params)
{
    int32_t             status = SystemP_SUCCESS;
    bissc_handle        handle = NULL;
    bissc_priv          *priv = NULL;
    const bissc_attrs   *attrs = NULL;
    bissc_pruicss_xchg  *pruicss_xchg = NULL;
    uint32_t            temp;
    void                *base_addr = NULL;
    uint8_t             ch_idx;

    if((index >= gBisscConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        handle = (bissc_handle)(&gBisscHandle[index]);

        /* Get the pointer to the priv and attrs */
        priv = handle->priv;
        attrs = handle->attrs;

        /* Input parameter validation */
        if ((priv == NULL) || (attrs == NULL))
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Validate params */
        if((params->pruicss_handle == NULL) ||
           (params->pruicss_handle->hwAttrs == NULL) ||
           (params->max_wait_loop_count == 0))
        {
            status = SystemP_FAILURE;
        }

        /* Validate attrs */
        if((attrs->instance >= gBisscConfigNum) ||
           (attrs->mode > BISSC_MODE_MULTI_CHANNEL_MULTI_PRU) ||
           (attrs->pruicss_instance > 1) ||
           (attrs->pruicss_slice > 1) ||
           (attrs->load_share_enabled > 1) ||
           (attrs->channel_mask == 0) ||
           (attrs->channel_mask > 7) ||
           (attrs->channel0_enabled > 1) ||
           (attrs->channel1_enabled > 1) ||
           (attrs->channel2_enabled > 1) ||
           (attrs->total_channels == 0) ||
           (attrs->total_channels > 3) ||
           (attrs->core_clk_freq == 0) ||
           (attrs->uart_clk_freq == 0) ||
           (attrs->iep_clk_freq == 0) ||
           (attrs->is_core_clk > 1) ||
           (attrs->iep_instance > 1) ||
           (attrs->iep_base_addr == NULL))
        {
            status = SystemP_FAILURE;
        }

        /* Validate IEP CMP and CAP event numbers for periodic trigger mode */
        if(status == SystemP_SUCCESS)
        {
            /* Validate IEP CMP event numbers and CAP event numbers */
            if(attrs->load_share_enabled)
            {
                for(ch_idx = 0; ch_idx < BISSC_NUM_CH_PER_SLICE_MAX; ch_idx++)
                {
                    if(attrs->channel_mask & (1U << ch_idx))
                    {
                        if((attrs->iep_cmp_event[ch_idx] >= BISSC_IEP_MAX_CMP_EVENT) ||
                           (attrs->iep_cap_event[ch_idx] >= BISSC_IEP_MAX_CAP_EVENT))
                        {
                            status = SystemP_FAILURE;
                            break;
                        }
                    }
                }
            }
            else
            {
                if((attrs->iep_cmp_event[0] >= BISSC_IEP_MAX_CMP_EVENT) ||
                   (attrs->iep_cap_event[0] >= BISSC_IEP_MAX_CAP_EVENT))
                {
                    status = SystemP_FAILURE;
                }
            }
        }

        /* Validate baud_rate - must be one of the supported BiSS-C frequencies */
        if((attrs->baud_rate != BISSC_FREQ_1MHZ) &&
           (attrs->baud_rate != BISSC_FREQ_2MHZ) &&
           (attrs->baud_rate != BISSC_FREQ_5MHZ) &&
           (attrs->baud_rate != BISSC_FREQ_8MHZ) &&
           (attrs->baud_rate != BISSC_FREQ_10MHZ))
        {
            status = SystemP_FAILURE;
        }

        /* TODO: Validate core_clk_freq, uart_clk_freq and iep_clk_freq */
    }

    if(status == SystemP_SUCCESS)
    {

        /* Get PRU slice from attrs */
        if(attrs->pruicss_slice == 1)
        {
            pruicss_xchg = (bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru1DramBase;
        }
        else /* if(attrs->pruicss_slice == 0) */
        {
            pruicss_xchg = (bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru0DramBase;
        }

        /* Initialize bissc_priv structure */
        priv->pruicss_xchg = pruicss_xchg;
        priv->pruicss_handle = params->pruicss_handle;
        priv->baud_rate = attrs->baud_rate;
        priv->cmd_process_delay_us = params->cmd_process_delay_us;
        priv->fw_wait_delay_us = params->fw_wait_delay_us;
        priv->max_wait_loop_count = params->max_wait_loop_count;
        status = bissc_hw_init(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /*Set IEP base address */
        base_addr = (void *)((PRUICSS_HwAttrs *)(handle->priv->pruicss_handle->hwAttrs))->baseAddr;
        temp = ((uint32_t)attrs->iep_base_addr) - ((uint32_t)base_addr);

        /* Initialize IEP base address in pruicss_xchg */
        status = bissc_config_iep_base_address(handle, temp);
    }
    if(status == SystemP_SUCCESS)
    {
        status = bissc_config_channel(handle, attrs->channel_mask, attrs->total_channels);
    }

    if((status == SystemP_SUCCESS) && (attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU))
    {
        status = bissc_config_load_share(handle, attrs->channel_mask);
    }

    if(status == SystemP_SUCCESS)
    {
        status = bissc_set_default_initialization(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        status = bissc_config_host_trigger(handle);
    }

    /* Configure IEP CMP and CAP events for enabled channels */
    if(status == SystemP_SUCCESS)
    {
        if(attrs->load_share_enabled)
        {
            /* Load share mode: Configure events for each enabled channel */
            for(ch_idx = 0; ch_idx < BISSC_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                /* Check if channel is enabled */
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    /* Configure IEP CMP event for this channel */
                    status = bissc_config_iep_cmp_event(handle, ch_idx, attrs->iep_cmp_event[ch_idx]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }

                    /* Configure IEP CAP event for this channel */
                    status = bissc_config_iep_cap_event(handle, ch_idx, attrs->iep_cap_event[ch_idx]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            /* Non-load share mode: Use index 0 always */
            /* Configure IEP CMP event */
            status = bissc_config_iep_cmp_event(handle, 0, attrs->iep_cmp_event[0]);

            if(status == SystemP_SUCCESS)
            {
                /* Configure IEP CAP event */
                status = bissc_config_iep_cap_event(handle, 0, attrs->iep_cap_event[0]);
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Mark handle as open after successful initialization */
        priv->is_open = 1;
    }

    if(status == SystemP_FAILURE)
    {
        handle = NULL;
    }

    return handle;
}

void bissc_deinit(bissc_handle handle)
{
    bissc_priv *priv;

    if((handle == NULL) || (handle->priv == NULL))
    {
        return;
    }

    priv = handle->priv;
    /* Mark as closed */
    priv->is_open = 0;
}

int32_t bissc_command_send(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        pruicss_xchg->cycle_trigger[0] = attrs->channel0_enabled ? 0x1 : 0;
        pruicss_xchg->cycle_trigger[1] = attrs->channel1_enabled ? 0x1 : 0;
        pruicss_xchg->cycle_trigger[2] = attrs->channel2_enabled ? 0x1 : 0;
    }
    else
    {
       pruicss_xchg->cycle_trigger[0] = 0x1;
    }

    return SystemP_SUCCESS;
}

int32_t bissc_command_wait(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint32_t            loop_count;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /*  Minimum and Maximum BiSS-C cycle time depends on various params as below:
        TCycle_min = TMA * (5 + DLEN + CRCLEN) + tLineDelay + tbusy_max + busy_s_max + tTO
        Instead wait for max of 5 ms as this can vary for different encoders and for daisy chain
    */
    loop_count = priv->max_wait_loop_count;

    /* Handle zero loop count case - would cause infinite loop */
    if(loop_count == 0)
    {
        return SystemP_FAILURE;
    }

    while(1)
    {
        if(attrs->load_share_enabled)
        {
            if((pruicss_xchg->cycle_trigger[0] == 0) && (pruicss_xchg->cycle_trigger[1] == 0) && (pruicss_xchg->cycle_trigger[2] == 0))
            {
                break;
            }
        }
        else if(pruicss_xchg->cycle_trigger[0] == 0)
        {
            break;
        }

        ClockP_usleep(priv->cmd_process_delay_us);
        loop_count--;
        if(loop_count == 0)
        {
            return SystemP_TIMEOUT;
        }

    }
    return SystemP_SUCCESS;
}

int32_t bissc_command_process(bissc_handle handle)
{
    int32_t ret = SystemP_FAILURE;
    bissc_priv *priv;

    /* Validate handle and priv pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    if(priv->is_continuous_mode == 0)
    {
        ret = bissc_command_send(handle);
        if(ret != SystemP_SUCCESS)
        {
            return ret;
        }
    }

    ret = bissc_command_wait(handle);
    return ret;
}

int32_t bissc_config_periodic_trigger_cmp_mode(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Configures bissc receiver in periodic trigger mode */
    if(attrs->load_share_enabled)
    {
        pruicss_xchg->opmode[0] = (attrs->channel0_enabled) ? BISSC_OPMODE_PERIODIC_CMP : pruicss_xchg->opmode[0];
        pruicss_xchg->opmode[1] = (attrs->channel1_enabled) ? BISSC_OPMODE_PERIODIC_CMP : pruicss_xchg->opmode[1];
        pruicss_xchg->opmode[2] = (attrs->channel2_enabled) ? BISSC_OPMODE_PERIODIC_CMP : pruicss_xchg->opmode[2];
    }
    else
    {
        pruicss_xchg->opmode[0] = BISSC_OPMODE_PERIODIC_CMP;
    }
    priv->is_continuous_mode = 1;
    return SystemP_SUCCESS;
}

static int32_t bissc_config_iep_base_address(bissc_handle handle, uint32_t iep_base_address)
{
    bissc_priv          *priv;
    bissc_pruicss_xchg  *pruicss_xchg;

    if(iep_base_address == 0)
    {
        return SystemP_FAILURE;
    }

    /* Configures IEP instance used for periodic trigger mode */
    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    pruicss_xchg->iep_base_address = iep_base_address;

    return SystemP_SUCCESS;
}

int32_t bissc_config_periodic_trigger_cap_mode(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Configures bissc receiver in periodic trigger CAP mode */
    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        pruicss_xchg->opmode[0] = (attrs->channel0_enabled) ? BISSC_OPMODE_PERIODIC_CAP : pruicss_xchg->opmode[0];
        pruicss_xchg->opmode[1] = (attrs->channel1_enabled) ? BISSC_OPMODE_PERIODIC_CAP : pruicss_xchg->opmode[1];
        pruicss_xchg->opmode[2] = (attrs->channel2_enabled) ? BISSC_OPMODE_PERIODIC_CAP : pruicss_xchg->opmode[2];
    }
    else
    {
        pruicss_xchg->opmode[0] = BISSC_OPMODE_PERIODIC_CAP;
    }
    priv->is_continuous_mode = 1;
    return SystemP_SUCCESS;
}

int32_t bissc_config_host_trigger(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Configures bissc receiver in host trigger mode */
    if(attrs->load_share_enabled)
    {
        pruicss_xchg->opmode[0] = (attrs->channel0_enabled) ? BISSC_OPMODE_HOST_TRIGGER : pruicss_xchg->opmode[0];
        pruicss_xchg->opmode[1] = (attrs->channel1_enabled) ? BISSC_OPMODE_HOST_TRIGGER : pruicss_xchg->opmode[1];
        pruicss_xchg->opmode[2] = (attrs->channel2_enabled) ? BISSC_OPMODE_HOST_TRIGGER : pruicss_xchg->opmode[2];
    }
    else
    {
        pruicss_xchg->opmode[0] = BISSC_OPMODE_HOST_TRIGGER;
    }
    priv->is_continuous_mode = 0;
    return SystemP_SUCCESS;
}

static void bissc_enable_load_share_mode(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    void                *pruicss_cfg;
    uint32_t            reg_val;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    if(attrs->pruicss_slice)
    {
        reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        reg_val |= CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, reg_val);
    }
    else
    {
        reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        reg_val |= CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, reg_val);
    }
}

static int32_t bissc_config_primary_core_mask(bissc_handle handle, uint8_t mask)
{
    bissc_priv *priv;
    priv = handle->priv;

    switch(mask)
    {
        case 1: /*only channel0 connected*/
            priv->pruicss_xchg->primary_core_mask = 0x1;
            break;
        case 2: /*channel1 connected*/
            priv->pruicss_xchg->primary_core_mask = 0x2;
            break;
        case 3: /*channel0 and channel1 connected*/
            priv->pruicss_xchg->primary_core_mask = 0x1;
            break;
        case 4: /*channel2 connected*/
            priv->pruicss_xchg->primary_core_mask = 0x4;
            break;
        case 5: /*channel0 and channel2 connnected*/
            priv->pruicss_xchg->primary_core_mask = 0x4;
            break;
        case 6: /*channel1 and channel2 connected*/
            priv->pruicss_xchg->primary_core_mask = 0X4;
            break;
        case 7: /*all three channel connected*/
            priv->pruicss_xchg->primary_core_mask = 0x4;
            break;
        default:
            return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

uint32_t bissc_get_total_channels(bissc_handle handle)
{
    const bissc_attrs *attrs;

    /* Validate handle and attrs pointer */
    if((handle == NULL) || (handle->attrs == NULL))
    {
        return 0;
    }

    attrs = handle->attrs;
    return attrs->total_channels;
}

uint32_t bissc_get_current_channel(bissc_handle handle, uint32_t ch_idx)
{
    bissc_priv *priv;

    /* Validate handle, priv pointer, and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ch_idx >= BISSC_NUM_CH_PER_SLICE_MAX))
    {
        return 0;
    }

    priv = handle->priv;
    return priv->channel[ch_idx];
}

int32_t bissc_update_clock_freq(bissc_handle handle, uint32_t frequency)
{
    bissc_priv *priv;

    /* Validate handle and priv pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Validate frequency - must be one of the supported BiSS-C frequencies */
    if((frequency != BISSC_FREQ_1MHZ) &&
       (frequency != BISSC_FREQ_2MHZ) &&
       (frequency != BISSC_FREQ_5MHZ) &&
       (frequency != BISSC_FREQ_8MHZ) &&
       (frequency != BISSC_FREQ_10MHZ))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->baud_rate = frequency;
    return SystemP_SUCCESS;
}

int32_t bissc_clock_config(bissc_handle handle, uint32_t frequency, uint32_t loop_count)
{
    bissc_clk_cfg   clk_cfg;
    int32_t         status = SystemP_FAILURE;

    /* Validate handle parameter */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(bissc_update_clock_freq(handle, frequency) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
    if(bissc_calc_clock(handle, &clk_cfg) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
    if(bissc_update_max_proc_delay(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
    if(bissc_hw_init(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
    status = bissc_wait_measure_proc_delay(handle, loop_count);
    return status;
}

int32_t bissc_clear_data_len(bissc_handle handle)
{
    bissc_priv  *priv;
    uint32_t    ch_num, enc_num;

    /* Validate handle and priv pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    for(ch_num = 0; ch_num < BISSC_NUM_CH_PER_SLICE_MAX; ch_num++)
    {
        priv->num_encoders[ch_num] = 0;
        for(enc_num = 0; enc_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; enc_num++)
        {
            priv->single_turn_len[ch_num][enc_num] = 0;
            priv->multi_turn_len[ch_num][enc_num] = 0;
            priv->data_len[ch_num][enc_num] = 0;
        }
    }
    return SystemP_SUCCESS;
}

int32_t bissc_get_pos(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    uint32_t            raw_data0, raw_data1, shift, sl_num, max, ch_num, num_encoders, ls_ch;
    uint32_t            ch;
    int32_t             ret;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    ch = 0;

    ret = bissc_command_process(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        ch = priv->channel[ch_num];

        if(attrs->load_share_enabled)
        {
            num_encoders = priv->num_encoders[ch];
            ls_ch = ch;
        }
        else
        {
            num_encoders = priv->num_encoders[0];
            ls_ch = 0;
        }

        for(sl_num = 0; sl_num < num_encoders; sl_num++)
        {
            raw_data0 = priv->pruicss_xchg->pos_data_res[sl_num].raw_data[ch].pos_data_word0;
            raw_data1 = priv->pruicss_xchg->pos_data_res[sl_num].raw_data[ch].pos_data_word1;

            /* Calculate max value for angle conversion using bit shift
             * NOTE: This bit shift is safe because bissc_update_data_len() validates that:
             *   - Without Safety: single_turn + multi_turn <= 56 bits
             *   - With Safety: single_turn + multi_turn <= 40 bits
             * Therefore, single_turn_len is guaranteed to be <= 56 bits, which is well within
             * the safe range for left shift on uint64_t (valid range: 0-63).
             * Shifting by 64 or more would cause undefined behavior, but this cannot occur
             * due to frame size validation in bissc_update_data_len(). */
            max = (1ULL << priv->single_turn_len[ls_ch][sl_num]);
            if(priv->has_safety[ls_ch][sl_num])
            {
                priv->raw_data                            = (uint64_t) raw_data0 << 32 | raw_data1;
                priv->enc_pos_data[ch].position[sl_num]   = (uint64_t) (priv->raw_data >> (BISSC_POS_CRC_LEN + BISSC_EW_LEN + BISSC_SAFETY_CRC_LEN));
                priv->enc_pos_data[ch].ew[sl_num]         = (priv->raw_data >> (BISSC_POS_CRC_LEN + BISSC_SAFETY_CRC_LEN)) & BISSC_EW_FIELD_MASK;
                priv->sign_of_life_cnt[ch][sl_num]        = (priv->raw_data >> (BISSC_SAFETY_CRC_LEN)) & BISSC_6BIT_FIELD_MASK;
                priv->rcv_safety_crc[ch][sl_num]          = priv->raw_data & BISSC_SAFETY_CRC_FIELD_MASK;
                priv->calc_safety_crc[ch][sl_num]         = priv->pruicss_xchg->safety_crc[sl_num][ch];
            }
            if((priv->data_len[ls_ch][sl_num] + BISSC_POS_CRC_LEN + BISSC_EW_LEN)  <= 32)
            {
                if(!(priv->has_safety[ls_ch][sl_num]))
                {
                    priv->raw_data                                = (uint64_t) raw_data0;
                    priv->enc_pos_data[ch].position[sl_num]       = (uint64_t) (raw_data0 >> (BISSC_POS_CRC_LEN + BISSC_EW_LEN));
                    priv->enc_pos_data[ch].ew[sl_num]             = ((raw_data0 >> (BISSC_POS_CRC_LEN )) & BISSC_EW_FIELD_MASK);
                    priv->enc_pos_data[ch].rcv_crc[sl_num]        = raw_data0 & BISSC_6BIT_FIELD_MASK;
                }
                priv->enc_pos_data[ch].num_of_turns[sl_num]   = priv->enc_pos_data[ch].position[sl_num] >> priv->single_turn_len[ls_ch][sl_num];
                priv->enc_pos_data[ch].angle[sl_num]          = (float)(priv->enc_pos_data[ch].position[sl_num] & (max - 1)) / max * (float)360;
                priv->enc_pos_data[ch].otf_crc[sl_num]        = priv->pruicss_xchg->pos_data_res[sl_num].pos_data_otf_crc[ch];
                priv->pd_crc_err_cnt[ch][sl_num]              = priv->pruicss_xchg->pos_data_res[sl_num].pd_crc_err_cnt[ch];
            }
            else
            {
                if(!(priv->has_safety[ls_ch][sl_num]))
                {
                    shift = ((priv->data_len[ls_ch][sl_num] + BISSC_POS_CRC_LEN + BISSC_EW_LEN) - 32);
                    priv->raw_data                                = (uint64_t) raw_data0 <<  shift | raw_data1;
                    priv->enc_pos_data[ch].position[sl_num]       = (uint64_t) (priv->raw_data >> (BISSC_POS_CRC_LEN + BISSC_EW_LEN));
                    priv->enc_pos_data[ch].ew[sl_num]             = ((priv->raw_data >> (BISSC_POS_CRC_LEN )) & BISSC_EW_FIELD_MASK);
                    priv->enc_pos_data[ch].rcv_crc[sl_num]        = priv->raw_data & BISSC_6BIT_FIELD_MASK;
                }
                priv->enc_pos_data[ch].num_of_turns[sl_num]   = priv->enc_pos_data[ch].position[sl_num] >> priv->single_turn_len[ls_ch][sl_num];
                priv->enc_pos_data[ch].angle[sl_num]          = (float)(priv->enc_pos_data[ch].position[sl_num] & (max - 1)) / max * (float)360;
                priv->enc_pos_data[ch].otf_crc[sl_num]        = priv->pruicss_xchg->pos_data_res[sl_num].pos_data_otf_crc[ch];
                priv->pd_crc_err_cnt[ch][sl_num]              = priv->pruicss_xchg->pos_data_res[sl_num].pd_crc_err_cnt[ch];
            }
        }
    }
    return SystemP_SUCCESS;
}

int32_t bissc_config_clock(bissc_handle handle, bissc_clk_cfg *clk_cfg)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    void                *pruicss_cfg;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint32_t            rx_reg_val;
    uint32_t            tx_reg_val;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) ||
       (clk_cfg == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    pruicss_xchg = priv->pruicss_xchg;

    /* Configure RX and TX CFG registers based on PRU slice */
    /* Polarity of Start bit is 0 for BiSS-C */
    if(attrs->pruicss_slice)
    {
        /* Slice 1 - Read-Modify-Write for RX CFG */
        rx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG);
        /*
         * NOTE: Using CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK instead of
         * CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SB_POL_MASK because of incorrect definition.
         */
        rx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SAMPLE_SIZE_MASK);
        rx_reg_val |= (clk_cfg->rx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_SHIFT) |
                      (clk_cfg->rx_div_attr);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG, rx_reg_val);

        /* Slice 1 - Read-Modify-Write for TX CFG */
        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, tx_reg_val);
    }
    else
    {
        /* Slice 0 - Read-Modify-Write for RX CFG */
        rx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG);
        rx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SAMPLE_SIZE_MASK);
        rx_reg_val |= (clk_cfg->rx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_SHIFT) |
                      (clk_cfg->rx_div_attr);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG, rx_reg_val);

        /* Slice 0 - Read-Modify-Write for TX CFG */
        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, tx_reg_val);
    }

    if(attrs->load_share_enabled)
    {
        bissc_enable_load_share_mode(handle);
    }

    /* Clock configuration has changed - Indicate fw to measure the delay again */
    pruicss_xchg->measure_proc_delay = BISSC_MEASURE_PROC_DELAY_ENABLE;

    return SystemP_SUCCESS;
}

int32_t bissc_config_channel(bissc_handle handle, uint8_t mask, uint8_t total_channels)
{
    bissc_priv          *priv;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint32_t            ch_num;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (total_channels > BISSC_NUM_CH_PER_SLICE_MAX) ||
       (mask == 0) ||
       (mask > 0x7))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    pruicss_xchg->channel = mask;

    /*  Below for loop iterates for enabled channel number of times.
        Updates channel in this manner:
        if ch0 only selected --> priv->channel[0] = 0;
        if ch1 only selected --> priv->channel[0] = 1;
        if ch2 only selected --> priv->channel[0] = 2;
        if ch0 & ch1 are selected --> priv->channel[0] = 0, priv->channel[1] = 1;
        if ch0 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 2;
        if ch1 & ch2 are selected --> priv->channel[0] = 1, priv->channel[1] = 2;
        if ch0, ch1 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 1, priv->channel[2] = 2;
    */
    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        if((mask & 1) && ch_num == 0)
            priv->channel[ch_num] = 0;
        else if((mask & 2) && (ch_num == 0 || ch_num == 1))
            priv->channel[ch_num] = 1;
        else if((mask & 4))
            priv->channel[ch_num] = 2;
    }
    return SystemP_SUCCESS;
}

static int32_t bissc_config_load_share(bissc_handle handle, uint8_t mask)
{
    if(bissc_config_primary_core_mask(handle, mask) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    bissc_enable_load_share_mode(handle);

    return SystemP_SUCCESS;
}

int32_t bissc_wait_for_fw_initialization(bissc_handle handle, uint32_t loop_count)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    uint32_t            i;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint8_t             mask;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;
    mask = attrs->channel_mask;

    /* Poll firmware initialization status with timeout */
    for(i = 0; i < loop_count; i++)
    {
        /* Load share mode: check status for all enabled channels */
        if(attrs->load_share_enabled)
        {
            /* Check initialization status based on channel_mask (1-7) */
            switch (mask)
            {
                /* Channel 0 connected */
                case 1:
                    if((pruicss_xchg->status[0] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* Channel 1 connected */
                case 2:
                    if((pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* Channel 0 and 1 connected */
                case 3:
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* Channel 2 connected */
                case 4:
                    if((pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* Channel 0 and 2 connected */
                case 5:
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* Channel 1 and 2 connected */
                case 6:
                    if((pruicss_xchg->status[1] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                /* All three channels connected */
                case 7:
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[1] & 1) && ( pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                default:
                    return SystemP_FAILURE;
            }
            ClockP_usleep(priv->fw_wait_delay_us);
        }
        /* Non-load share mode: check only status[0] */
        else if(pruicss_xchg->status[0] & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(priv->fw_wait_delay_us);
        }
    }
    /* Timeout occurred */
    if(i == loop_count)
    {
        return SystemP_TIMEOUT;
    }
    return SystemP_SUCCESS;
}

int32_t bissc_get_enc_proc_delay(bissc_handle handle)
{
    bissc_priv          *priv;
    uint32_t            i;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    for(i = 0; i < BISSC_NUM_CH_PER_SLICE_MAX; i++)
    {
        priv->proc_delay[i] = pruicss_xchg->proc_delay[i];
    }
    return SystemP_SUCCESS;
}

static void bissc_config_clr_cfg0(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    void                *pruicss_cfg;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    if(attrs->pruicss_slice)
    {
        if(attrs->channel0_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG, 0);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG, 0);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG, 0);
        }
    }
    else
    {
        if(attrs->channel0_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG, 0);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG, 0);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG, 0);
        }
    }
}

static int32_t bissc_config_endat_mode(bissc_handle handle)
{
    bissc_priv *priv;
    const bissc_attrs *attrs;
    int32_t status;

    priv = handle->priv;
    attrs = handle->attrs;

    status = PRUICSS_setGpMuxSelect(priv->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);
    return status;
}

int32_t bissc_calc_clock(bissc_handle handle, bissc_clk_cfg *clk_cfg)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    uint32_t            freq;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) || (clk_cfg == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    freq = priv->baud_rate;
    pruicss_xchg = priv->pruicss_xchg;

    clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE;
    freq = freq * BISSC_MHZ_TO_HZ;
    pruicss_xchg->fifo_bit_idx = BISSC_FIFO_BIT_IDX_8X_OS;
    if(attrs->is_core_clk == BISSC_SET_STATUS_FLAG)
    {
        clk_cfg->tx_div = (attrs->core_clk_freq / freq) - 1;
        clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE + 1))) - 1;
        clk_cfg->is_core_clk = BISSC_SET_STATUS_FLAG;
        if(attrs->core_clk_freq == BISSC_PRU_CORE_CLK_FREQ_200MHZ * BISSC_MHZ_TO_HZ)
        {
            if(freq == BISSC_FREQ_10MHZ * BISSC_MHZ_TO_HZ)
            {
                clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE_4X;
                clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE_4X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = BISSC_FIFO_BIT_IDX_4X_OS;
            }
        }
        else if(attrs->core_clk_freq == BISSC_PRU_CORE_CLK_FREQ_300MHZ * BISSC_MHZ_TO_HZ)
        {
            if(freq == BISSC_FREQ_5MHZ * BISSC_MHZ_TO_HZ)
            {
                clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE | BISSC_RX_ENABLE_FRACTIONAL_DIV;
                clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE + 1) * BISSC_CLOCK_FRACTIONAL_DIVIDER)) - 1;
                pruicss_xchg->fifo_bit_idx = BISSC_FIFO_BIT_IDX_8X_OS;
            }
            else if(freq == BISSC_FREQ_10MHZ * BISSC_MHZ_TO_HZ)
            {
                clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE_4X | BISSC_RX_ENABLE_FRACTIONAL_DIV;
                clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE_4X + 1) * BISSC_CLOCK_FRACTIONAL_DIVIDER)) - 1;
                pruicss_xchg->fifo_bit_idx = BISSC_FIFO_BIT_IDX_4X_OS;
            }
        }
    }
    else if(attrs->is_core_clk == BISSC_CLEAR_STATUS_FLAG)
    {
        clk_cfg->tx_div = (attrs->uart_clk_freq / freq) - 1;
        clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE + 1))) - 1;
        clk_cfg->is_core_clk = BISSC_CLEAR_STATUS_FLAG;
        if(attrs->uart_clk_freq == BISSC_PRU_UART_CLK_FREQ_160MHZ * BISSC_MHZ_TO_HZ)
        {
            if(freq == BISSC_FREQ_8MHZ * BISSC_MHZ_TO_HZ)
            {
                clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE_4X;
                clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (BISSC_RX_SAMPLE_SIZE_4X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = BISSC_FIFO_BIT_IDX_4X_OS;
            }
        }
    }
    else
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

int32_t bissc_hw_init(bissc_handle handle)
{
    bissc_clk_cfg clk_cfg;

    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL))
    {
        return SystemP_FAILURE;
    }

    if(bissc_calc_clock(handle, &clk_cfg) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    if(bissc_config_endat_mode(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    if(bissc_config_clock(handle, &clk_cfg) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    bissc_config_clr_cfg0(handle);

    return SystemP_SUCCESS;
}

int32_t bissc_update_max_proc_delay(bissc_handle handle)
{
    bissc_priv          *priv;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    if(priv->baud_rate == BISSC_FREQ_1MHZ)
    {
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_1MHZ;
    }
    else if(priv->baud_rate == BISSC_FREQ_2MHZ)
    {
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_2MHZ;
    }
    else if(priv->baud_rate == BISSC_FREQ_5MHZ)
    {
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_5MHZ;
    }
    else if(priv->baud_rate == BISSC_FREQ_8MHZ)
    {
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_8MHZ;
    }
    else if(priv->baud_rate == BISSC_FREQ_10MHZ)
    {
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_10MHZ;
    }
    return SystemP_SUCCESS;
}

int32_t bissc_wait_measure_proc_delay(bissc_handle handle, uint32_t loop_count)
{

    bissc_priv          *priv;
    uint32_t            i;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    for(i = 0; i < loop_count; i++)
    {
        if(pruicss_xchg->measure_proc_delay & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(priv->fw_wait_delay_us);
        }
    }
    if(i == loop_count)
    {
        return SystemP_TIMEOUT;
    }
    return SystemP_SUCCESS;
}

int32_t bissc_enable_safety(bissc_handle handle, uint32_t enc_num, uint32_t ls_ch)
{
    bissc_priv *priv;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Validate bounds */
    if((ls_ch >= BISSC_NUM_CH_PER_SLICE_MAX) || (enc_num >= BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->has_safety[ls_ch][enc_num] = 1;
    priv->pruicss_xchg->has_safety[ls_ch] |= (1 << enc_num);

    return SystemP_SUCCESS;
}

int32_t bissc_disable_safety(bissc_handle handle)
{
    bissc_priv  *priv;
    uint32_t    enc_num;
    uint32_t    ch_num;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    for(ch_num = 0; ch_num < BISSC_NUM_CH_PER_SLICE_MAX; ch_num++)
    {
        for(enc_num = 0; enc_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; enc_num++)
        {
            priv->has_safety[ch_num][enc_num] = 0;
        }
        priv->pruicss_xchg->has_safety[ch_num] = 0;
    }

    return SystemP_SUCCESS;
}

int32_t bissc_set_default_initialization(bissc_handle handle)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    uint32_t             ch_num;
    uint8_t              total_channels, ls_ch;
    bissc_pruicss_xchg  *pruicss_xchg;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Initialize parameters to default values */
    if(attrs->load_share_enabled)
        total_channels = attrs->total_channels;
    else
        total_channels = 1;
    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        if(attrs->load_share_enabled)
            ls_ch = priv->channel[ch_num];
        else
            ls_ch = 0;

        pruicss_xchg->enc_len[ls_ch].data_len[0]           = BISSC_POS_DATA_LEN_DEFAULT;
        priv->single_turn_len[ls_ch][0]                    = BISSC_POS_DATA_LEN_DEFAULT;
        priv->multi_turn_len[ls_ch][0]                     = 0;
        priv->data_len[ls_ch][0]                           = BISSC_POS_DATA_LEN_DEFAULT;
        pruicss_xchg->enc_len[ls_ch].num_encoders          = 1;
        priv->num_encoders[ls_ch]                          = 1;
        priv->multi_turn_len[ls_ch][0]                     = 0;
        pruicss_xchg->ctrl_cmd[ls_ch]                      = 0;
        priv->ctrl_enc_id[ls_ch]                           = 0;
        priv->ctrl_reg_data[ls_ch]                         = 0;
    }
    pruicss_xchg->pos_crc_len         = BISSC_POS_CRC_LEN;
    pruicss_xchg->ctrl_cmd_crc_len    = BISSC_CTRL_CMD_CRC_LEN;
    pruicss_xchg->rx_clk_freq         = priv->baud_rate;
    if(bissc_update_max_proc_delay(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        if(attrs->load_share_enabled)
            ls_ch = priv->channel[ch_num];
        else
            ls_ch = 0;

        /* Initialize encoder timeout with default value (40us converted to PRU cycles) */
        pruicss_xchg->encoder_timeout[ls_ch] = (uint32_t)((attrs->core_clk_freq / BISSC_MHZ_TO_HZ) * BISSC_DEFAULT_ENCODER_TIMEOUT_US);
    }
    pruicss_xchg->delay_100ms         = (uint32_t)((attrs->core_clk_freq / BISSC_MHZ_TO_HZ) * 100 * 1000);
    pruicss_xchg->icss_clk            = (uint64_t)(attrs->core_clk_freq);
    pruicss_xchg->valid_bit_idx       = BISSC_VALID_BIT_IDX;
    pruicss_xchg->measure_proc_delay  = 1;
    pruicss_xchg->execution_state[0]  = 0;
    pruicss_xchg->execution_state[1]  = 0;
    pruicss_xchg->execution_state[2]  = 0;
    pruicss_xchg->opmode[0]           = 1;
    pruicss_xchg->opmode[1]           = 1;
    pruicss_xchg->opmode[2]           = 1;
    priv->is_continuous_mode          = 0;
    return SystemP_SUCCESS;
}

static uint8_t bissc_calc_ctrl_crc(uint32_t ctrl_cmd, uint8_t num_bits)
{
    uint8_t     ff0 = 0, ff1 = 0, ff2 = 0, ff3 = 0, crc;
    uint32_t    msb, ex, i;

    /* NOTE: This is an internal API always called with valid num_bits values.
     * All call sites use constants or sums of constants that result in values < 32:
     *   - BISSC_CTS_BIT + BISSC_ENC_ID_LEN + BISSC_REG_ADDR_LEN = 1 + 3 + 7 = 11 bits
     *   - BISSC_REG_DATA_LEN = 8 bits
     * The bit shift operation (num_bits - 1) is safe because:
     *   - num_bits is always > 0 (no underflow)
     *   - num_bits is always < 32 (shift amount is valid for uint32_t)
     * This function is not exposed in the public API and is only called internally
     * by bissc_generate_ctrl_cmd() with controlled values. */
    msb = (1U << (num_bits - 1));
    for(i = 0; i < num_bits; i++)
    {
        if(ctrl_cmd & msb)          /*Check for the MSB(11th in this case)*/
            ex = ff3 ^ 1;
        else
            ex = ff3 ^ 0;
        ff3 = ff2;
        ff2 = ff1;
        ff1 = ff0 ^ ex;            /*4 bit CRC algorithm*/
        ff0 = ex;
        ctrl_cmd = ctrl_cmd << 1;
    }
    crc = ff3 << 3 | ff2 << 2 | ff1 << 1 | ff0;
    crc = ~ crc;
    return crc;
}

uint32_t bissc_generate_ctrl_cmd(bissc_handle handle,
                                 uint8_t ls_ch,
                                 uint8_t ctrl_write_status,
                                 uint32_t ctrl_reg_address,
                                 uint32_t ctrl_reg_data,
                                 uint32_t ctrl_enc_id)
{
    bissc_priv  *priv;
    uint32_t    ctrl_cmd = 0;
    uint8_t     crc;
    /* Validate handle, internal structure pointers, parameters */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= BISSC_NUM_CH_PER_SLICE_MAX) || (ctrl_write_status > 1) ||
       (ctrl_reg_address > BISSC_REG_ADDR_MASK) || (ctrl_reg_data > BISSC_REG_DATA_MASK) || (ctrl_enc_id > BISSC_ENC_ID_MASK))
    {
        return 0;
    }

    priv = handle->priv;
    priv->ctrl_write_status[ls_ch] = ctrl_write_status;
    priv->ctrl_reg_address[ls_ch] = ctrl_reg_address;
    priv->ctrl_reg_data[ls_ch] = ctrl_reg_data;
    priv->ctrl_enc_id[ls_ch] = ctrl_enc_id;
    /* CTS bit is 1 for control communication followed by 3-bit encoder ID which describes position of encoder in daisy chain*/
    ctrl_cmd = (BISSC_CTS_BIT << BISSC_ENC_ID_LEN) | (priv->ctrl_enc_id[ls_ch] & BISSC_ENC_ID_MASK);
    /* 7-bit register address given by user*/
    ctrl_cmd = (ctrl_cmd << BISSC_REG_ADDR_LEN) | (priv->ctrl_reg_address[ls_ch] & BISSC_REG_ADDR_MASK);
    /* 4-bit CRC over CTS + enc ID + reg address bits */
    crc = bissc_calc_ctrl_crc(ctrl_cmd, (BISSC_CTS_BIT + BISSC_ENC_ID_LEN + BISSC_REG_ADDR_LEN));
    ctrl_cmd = (ctrl_cmd << BISSC_CTRL_CMD_CRC_LEN) | (crc & BISSC_CTRL_CMD_CRC_MASK);
    if(priv->ctrl_write_status[ls_ch] == 1)
    {
        /* RWS bits Write Access*/
        ctrl_cmd = (ctrl_cmd << BISSC_RWS_LEN) | BISSC_CTRL_WRITE_ACCESS;
        /* calculate 4-bit CRC over 8-bit register data*/
        crc = bissc_calc_ctrl_crc(priv->ctrl_reg_data[ls_ch], BISSC_REG_DATA_LEN);
        /* 8-bit data, user wants to write at given register address*/
        ctrl_cmd = (ctrl_cmd << BISSC_REG_DATA_LEN) | (priv->ctrl_reg_data[ls_ch] & BISSC_REG_DATA_MASK);
        /* 4-bit CRC for given register data */
        ctrl_cmd = (ctrl_cmd << BISSC_CTRL_CMD_CRC_LEN) | (crc & BISSC_CTRL_CMD_CRC_MASK);
        /* 2-stop bits "SP" S: stop bit for one control communication command, P: stop bit for series of comtrol communication*/
        ctrl_cmd = ctrl_cmd << BISSC_CTRL_STOP_LEN;
    }
    else
    {
        /* RWS bits for Read Access*/
        ctrl_cmd = (ctrl_cmd << BISSC_RWS_LEN) | BISSC_CTRL_READ_ACCESS;
        /* Append 14 '0's at the end to receive CDS response*/
        ctrl_cmd = ctrl_cmd << (BISSC_REG_DATA_LEN + BISSC_CTRL_CMD_CRC_LEN + BISSC_CTRL_STOP_LEN);
    }
    priv->ctrl_enc_id[ls_ch] = 0;
    return ctrl_cmd;
}

int32_t bissc_update_data_len(bissc_handle handle, uint32_t single_turn_len[], uint32_t multi_turn_len[], uint32_t ch_num)
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint32_t            sl_num, ls_ch;
    uint32_t            total_frame_size;

    /* Validate handle, array parameters, internal structure pointers, and array bounds */
    if((handle == NULL) || (single_turn_len == NULL) || (multi_turn_len == NULL) || (ch_num >= BISSC_NUM_CH_PER_SLICE_MAX) ||
       (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        ls_ch = priv->channel[ch_num];
    }
    else
    {
        ls_ch = 0;
    }

    /* Validate all encoder configurations before making any state changes
     * This ensures atomicity - either all encoders are valid and configured,
     * or none are modified if any validation fails. */
    for(sl_num = 0; sl_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; sl_num++)
    {
        if(single_turn_len[sl_num])
        {
            /* Validate total frame size based on safety mode
             * BiSS-C uses 64-bit frames with following structure:
             * Without Safety: position_data + E/W(2) + CRC(6) = position_data + 8 bits
             * With Safety: position_data + E/W(2) + sign_of_life(6) + safety_CRC(16) = position_data + 24 bits
             */
            if(priv->has_safety[ls_ch][sl_num])
            {
                /* With Safety: position_data + 24 bits <= 64 bits */
                total_frame_size = single_turn_len[sl_num] + multi_turn_len[sl_num] +
                                   BISSC_EW_LEN + BISSC_SIGN_OF_LIFE_LEN + BISSC_SAFETY_CRC_LEN;

                if(total_frame_size > BISSC_MAX_FRAME_SIZE)
                {
                    return SystemP_FAILURE;
                }
            }
            else
            {
                /* Without Safety: position_data + 8 bits <= 64 bits */
                total_frame_size = single_turn_len[sl_num] + multi_turn_len[sl_num] +
                                   BISSC_EW_LEN + BISSC_POS_CRC_LEN;

                if(total_frame_size > BISSC_MAX_FRAME_SIZE)
                {
                    return SystemP_FAILURE;
                }
            }
        }
    }

    /* All validations passed, now update the driver state and PRU-ICSS exchange memory
     * This two-phase approach ensures we don't leave the driver in a partially configured
     * state if validation fails for any encoder. */
    priv->num_encoders[ls_ch] = 0;

    for(sl_num = 0; sl_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; sl_num++)
    {
        if(single_turn_len[sl_num])
        {
            priv->single_turn_len[ls_ch][sl_num] = single_turn_len[sl_num];
            priv->multi_turn_len[ls_ch][sl_num] = multi_turn_len[sl_num];
            priv->data_len[ls_ch][sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num];
            pruicss_xchg->enc_len[ls_ch].data_len[sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num];
            priv->num_encoders[ls_ch]++;
        }
    }
    pruicss_xchg->enc_len[0].num_encoders = priv->num_encoders[0];
    pruicss_xchg->enc_len[1].num_encoders = priv->num_encoders[1];
    pruicss_xchg->enc_len[2].num_encoders = priv->num_encoders[2];
    return SystemP_SUCCESS;
}

int32_t bissc_set_ctrl_cmd_and_process(bissc_handle handle, uint32_t ctrl_cmd[])
{
    bissc_priv          *priv;
    const bissc_attrs   *attrs;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint32_t            ch = 0, ch_num;
    int32_t             ret;

    /* Validate handle, array parameter, and internal structure pointers */
    if((handle == NULL) || (ctrl_cmd == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    pruicss_xchg->ctrl_cmd[0] = ctrl_cmd[0];
    pruicss_xchg->ctrl_cmd[1] = ctrl_cmd[1];
    pruicss_xchg->ctrl_cmd[2] = ctrl_cmd[2];
    if(attrs->load_share_enabled)
    {
        pruicss_xchg->ctrl_cmd_status[0] = attrs->channel0_enabled ? 1 : 0;
        pruicss_xchg->ctrl_cmd_status[1] = attrs->channel1_enabled ? 1 : 0;
        pruicss_xchg->ctrl_cmd_status[2] = attrs->channel2_enabled ? 1 : 0;
        while(pruicss_xchg->ctrl_cmd_status[0] + pruicss_xchg->ctrl_cmd_status[1] + pruicss_xchg->ctrl_cmd_status[2])
        {
            ret = bissc_command_process(handle);
            if(ret != SystemP_SUCCESS)
            {
                return ret;
            }
            ClockP_usleep(priv->fw_wait_delay_us);
        }
    }
    else
    {
        pruicss_xchg->ctrl_cmd_status[0] = 1;
        while(pruicss_xchg->ctrl_cmd_status[0] & 1)
        {
            ret = bissc_command_process(handle);
            if(ret != SystemP_SUCCESS)
            {
                return ret;
            }
            ClockP_usleep(priv->fw_wait_delay_us);
        }
    }

    /*supplying 2 extra cycles for ctrl communication stop bit */
    ret = bissc_command_process(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }
    ClockP_usleep(priv->fw_wait_delay_us);

    ret = bissc_command_process(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }
    ClockP_usleep(priv->fw_wait_delay_us);

    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        ch = priv->channel[ch_num];
        priv->enc_ctrl_data[ch].cmd_result  = pruicss_xchg->ctrl_res[ch].ctrl_cds_res;
        priv->enc_ctrl_data[ch].cmd_rcv_crc = pruicss_xchg->ctrl_res[ch].ctrl_rcvd_crc;
        priv->enc_ctrl_data[ch].cmd_otf_crc = pruicss_xchg->ctrl_res[ch].ctrl_otf_crc;
        priv->ctrl_crc_err_cnt[ch] = pruicss_xchg->ctrl_res[ch].ctrl_crc_err_cnt;
    }
    return SystemP_SUCCESS;
}

const bissc_attrs* bissc_get_attrs(bissc_handle handle)
{
    /* Validate handle and attrs pointer */
    if((handle == NULL) || (handle->attrs == NULL))
    {
        return NULL;
    }
    return handle->attrs;
}

bissc_priv* bissc_get_priv(bissc_handle handle)
{
    /* Validate handle and priv pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return NULL;
    }
    return handle->priv;
}

int32_t bissc_set_encoder_timeout(bissc_handle handle, uint32_t ch_num, uint32_t encoder_timeout)
{
    bissc_priv *priv;
    bissc_pruicss_xchg *pruicss_xchg;
    const bissc_attrs *attrs;
    uint32_t ls_ch;

    /* Input validation with internal structure pointers */
    if((handle == NULL) || (ch_num >= BISSC_NUM_CH_PER_SLICE_MAX) || (handle->priv == NULL) ||
       (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;
    attrs = handle->attrs;

    /* Determine load share channel index based on mode */
    if(attrs->load_share_enabled)
    {
        ls_ch = ch_num;
    }
    else
    {
        ls_ch = 0;
    }

    pruicss_xchg->encoder_timeout[ls_ch] = encoder_timeout;

    return SystemP_SUCCESS;
}

uint32_t bissc_get_encoder_timeout(bissc_handle handle, uint32_t ch_num)
{
    bissc_priv *priv;
    const bissc_attrs *attrs;
    uint32_t ls_ch;

    /* Input validation with internal structure pointers */
    if((handle == NULL) || (ch_num >= BISSC_NUM_CH_PER_SLICE_MAX) || (handle->priv == NULL) ||
       (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return 0;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Determine load share channel index based on mode */
    if(attrs->load_share_enabled)
    {
        ls_ch = ch_num;
    }
    else
    {
        ls_ch = 0;
    }

    return priv->pruicss_xchg->encoder_timeout[ls_ch];
}

int32_t bissc_config_iep_cap_event(bissc_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t             ret_val = SystemP_SUCCESS;
    const bissc_attrs   *attrs;
    bissc_priv          *priv;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint8_t             ch_index = 0;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) || (event_num >= BISSC_IEP_MAX_CAP_EVENT) || (channel >= BISSC_NUM_CH_PER_SLICE_MAX) ||
       (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        ch_index = channel;
    }
    else
    {
        /* Always 0 in single PRU mode. When load share mode is disabled.
        In single PRU mode firmware, the channel number is ignored and the firmware always reads data from DMEM using the channel 0 offset, regardless of which channels are connected.*/
        ch_index = 0;
    }

    /* Write cap event and capture register address in DMEM */
    pruicss_xchg->trigger_params[ch_index].iep_cap_event = event_num;
    pruicss_xchg->trigger_params[ch_index].iep_capture_reg = pruicss_xchg->iep_base_address + BISSC_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0  + BISSC_8_BYTE_REG_OFFSET*(event_num);

    /* CAP6 and CAP7 have 2 extra registers for fall capture values, add extra offset */
    /* CAP6 and CAP7 has 2 register bits each. So bit 8 needs to be used for CAP7. Only capture rise bits for CAP6 and CAP7 are used. */
    if(event_num > 6)
    {
        pruicss_xchg->trigger_params[ch_index].iep_cap_event += 1;
        pruicss_xchg->trigger_params[ch_index].iep_capture_reg += BISSC_8_BYTE_REG_OFFSET;
    }
    return ret_val;
}

int32_t bissc_config_iep_cmp_event(bissc_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t             ret_val = SystemP_SUCCESS;
    const bissc_attrs   *attrs;
    bissc_priv          *priv;
    bissc_pruicss_xchg  *pruicss_xchg;
    uint8_t             ch_index = 0;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) || (event_num >= BISSC_IEP_MAX_CMP_EVENT) || (channel >= BISSC_NUM_CH_PER_SLICE_MAX) ||
       (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    /* Determine channel index for DMEM access */
    if(attrs->load_share_enabled)
    {
        ch_index = channel;
    }
    else
    {
        /* Always 0 in single PRU mode. When load share mode is disabled.
        In single PRU mode firmware, the channel number is ignored and the firmware always reads data from DMEM using the channel 0 offset, regardless of which channels are connected.*/
        ch_index = 0;
    }

    /* Write CMP event number in DMEM */
    pruicss_xchg->trigger_params[ch_index].iep_cmp_event = event_num;

    return ret_val;
}
