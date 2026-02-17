/*
 *  Copyright (C) 2021-2026 Texas Instruments Incorporated
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

#include <position_sense/endat/include/endat_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern uint32_t gEndatConfigNum;
extern endat_config gEndatHandle[];
/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/*
 * Static internal functions
 *
 * Note: Handle NULL checks are NOT performed in these internal APIs.
 * It is the responsibility of the calling function (public APIs) to
 * ensure that the handle parameter is valid before calling these functions.
 * All public APIs perform handle NULL validation before invoking internal functions.
 */
static int32_t endat_recvd_organize(endat_handle handle, int32_t cmd, endat_data *endat_data);
static int32_t endat_recvd_format(endat_handle handle, int32_t cmd, endat_data *endat_data, endat_format_data *u);
static int32_t endat_addinfo_format(endat_handle handle, endat_data *endat_data, endat_format_data *u);
static int32_t endat_position_addinfo_format(endat_handle handle, endat_data *endat_data, endat_format_data *u);
static int32_t endat_addr_params_format(endat_handle handle, endat_data *endat_data, endat_format_data *u);
static int32_t endat_test_format(endat_handle handle, endat_data *endat_data, endat_format_data *u);
static uint32_t endat_make_crc_norm(uint32_t param8, uint32_t param16);
static uint32_t endat_make_crc_pos(uint32_t clocks, uint32_t error1, uint32_t error2, uint32_t addinfo, uint32_t addinfo2);
static int32_t endat_get_pos_res(endat_handle handle, int32_t *pos_res);
static int32_t endat_get_multi_turn_res(endat_handle handle, int32_t *multi_turn_res);
static int32_t endat_get_id(endat_handle handle);
static int32_t endat_get_sn(endat_handle handle);
static int32_t endat_get_command_set(endat_handle handle);
static int32_t endat_get_type(endat_handle handle);
static int32_t endat_get_step(endat_handle handle, int32_t *step);
static int32_t endat_config_global_rx_arm_cnt(endat_handle handle, uint16_t val);
static int32_t endat_set_continuous_mode(endat_handle handle);
static int32_t endat_clear_continuous_mode(endat_handle handle);
static int32_t endat_config_clr_cfg0(endat_handle handle);
static int32_t endat_config_endat_mode(endat_handle handle);
static int32_t endat_config_clock_reg(endat_handle handle, endat_clk_cfg *clk_cfg);
static int32_t endat_hw_init(endat_handle handle);
static int32_t endat_set_default_initialization(endat_handle handle);
static int32_t endat_config_timing_delays(endat_handle handle);
static int32_t endat_config_channel_info_addr(endat_handle handle, uint32_t ch_info_global_addr);
static int32_t endat_config_iep_base_addr(endat_handle handle);
static int32_t endat_config_syn_bits(endat_handle handle, uint8_t mask);
static int32_t endat_enable_load_share_mode(endat_handle handle);
static int32_t endat_config_primary_core_mask(endat_handle handle, uint8_t mask);
static int32_t endat_calculate_propagation_delay(endat_handle handle);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default EnDAT parameters structure */
const endat_params gEndatDefaultParams =
{
    NULL,                               /* pruicss_handle */
    ENDAT_DEFAULT_MAX_WAIT_LOOP_COUNT,  /* max_wait_loop_count */
    ENDAT_DEFAULT_CMD_PROCESS_DELAY_US, /* cmd_process_delay_us */
    ENDAT_DEFAULT_FW_WAIT_DELAY_US,     /* fw_wait_delay_us */
    NULL,                               /* channel_rx_info */
    0U,                                 /* ch_info_global_addr */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void endat_params_init(endat_params *params)
{
    /* Input parameter validation */
    if (params != NULL)
    {
        *params = gEndatDefaultParams;
    }
}

endat_handle endat_init(uint32_t index, const endat_params *params)
{
    int32_t status = SystemP_SUCCESS;
    endat_handle handle = NULL;
    endat_priv *priv = NULL;
    const endat_attrs *attrs = NULL;
    uint8_t ch_idx;
    uint32_t i;

    /* Parameter validation */
    if((index >= gEndatConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        handle = (endat_handle)&gEndatHandle[index];
        priv = handle->priv;
        attrs = handle->attrs;

        /* Validate internal structures */
        if((priv == NULL) || (attrs == NULL))
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Validate params */
        if((params->pruicss_handle == NULL) ||
           (params->pruicss_handle->hwAttrs == NULL) ||
           (params->max_wait_loop_count == 0) ||
           (params->channel_rx_info == NULL) ||
           (params->ch_info_global_addr == 0U))
        {
            status = SystemP_FAILURE;
        }

        /* Validate all attrs fields (range checking) */
        if((attrs->instance >= gEndatConfigNum) ||
           (attrs->mode > ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU) ||
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
           (attrs->iep_instance > 1) ||
           (attrs->iep_base_addr == NULL) ||
           (attrs->is_core_clk > 1))
        {
            status = SystemP_FAILURE;
        }

        /* Validate IEP event numbers for enabled channels only */
        if(status == SystemP_SUCCESS)
        {
            if(attrs->load_share_enabled)
            {
                for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
                {
                    /* Only validate IEP events for enabled channels */
                    if(attrs->channel_mask & (1U << i))
                    {
                        if((attrs->iep_cmp_event[i] >= ENDAT_IEP_CMP_EVENT_MAX) ||
                           (attrs->iep_cap_event[i] >= ENDAT_IEP_CAP_EVENT_MAX))
                        {
                            status = SystemP_FAILURE;
                            break;
                        }
                    }
                }
            }
            else
            {
                /* Non-load share mode: Use index 0 always */
                if((attrs->iep_cmp_event[0] >= ENDAT_IEP_CMP_EVENT_MAX) ||
                   (attrs->iep_cap_event[0] >= ENDAT_IEP_CAP_EVENT_MAX))
                {
                    status = SystemP_FAILURE;
                }
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Get PRU slice DRAM base address for pruicss_xchg */
        if(attrs->pruicss_slice == 1)
        {
            priv->pruicss_xchg = (endat_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru1DramBase;
        }
        else /* pruicss_slice == 0 */
        {
            priv->pruicss_xchg = (endat_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru0DramBase;
        }
        /* Copy params values to priv */
        priv->pruicss_handle = params->pruicss_handle;
        priv->cmd_process_delay_us = params->cmd_process_delay_us;
        priv->fw_wait_delay_us = params->fw_wait_delay_us;
        priv->max_wait_loop_count = params->max_wait_loop_count;
        priv->channel_rx_info = params->channel_rx_info;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Initialize priv and PRU exchange interface with defaults */
        status = endat_set_default_initialization(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure timing delays based on pru clock frequency */
        status = endat_config_timing_delays(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure IEP base address */
        status = endat_config_iep_base_addr(handle);
    }

    /* Configure IEP CMP and CAP events for enabled channels */
    if(status == SystemP_SUCCESS)
    {
        if(attrs->load_share_enabled)
        {
            for(ch_idx = 0; ch_idx < ENDAT_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                /* Check if channel is enabled */
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    /* Configure IEP CMP event for this channel */
                    status = endat_config_iep_cmp_event(handle, ch_idx, attrs->iep_cmp_event[ch_idx]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }

                    /* Configure IEP CAP event for this channel */
                    status = endat_config_iep_cap_event(handle, ch_idx, attrs->iep_cap_event[ch_idx]);
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
            /* Configure IEP CMP event for this channel */
            status = endat_config_iep_cmp_event(handle, 0, attrs->iep_cmp_event[0]);
            if(status == SystemP_SUCCESS)
            {
                /* Configure IEP CAP event for this channel */
                status = endat_config_iep_cap_event(handle, 0, attrs->iep_cap_event[0]);
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure channel info memory address */
        status = endat_config_channel_info_addr(handle, params->ch_info_global_addr);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure Recovery time parameters */
        status = endat_init_rt_measurement(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Initialize hardware, Enable Endat mode, configure init frequency and enable load share if enabled */
        status = endat_hw_init(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure channels */
        if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU || attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
        {
            status = endat_config_multi_channel_mask(handle, attrs->channel_mask, attrs->load_share_enabled);
        }
        else
        {
            for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1U << i))
                {
                    status = endat_config_channel(handle, i);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }
                }
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Mark as open */
        priv->is_open = 1;
    }

    if(status != SystemP_SUCCESS)
    {
        handle = NULL;
    }

    return handle;
}

void endat_deinit(endat_handle handle)
{
    endat_priv *priv;

    if((handle == NULL) || (handle->priv == NULL))
    {
        return;
    }

    priv = handle->priv;
    /* Mark as closed */
    priv->is_open = 0;
}

/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that functions,
 * endat_recvd_organize()
 * endat_recvd_format()
 * endat_recvd_validate()
 * endat_recvd_print()
 * will normally be called after endat_get_command
 */

static int32_t endat_recvd_organize(endat_handle handle, int32_t cmd, endat_data *endat_data)
{
    uint32_t word0, word1, word2, word3;
    uint32_t pos_bits, shift;
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_ch_rx_info_array *channel_rx_info;
    int32_t flags_idx;

    /* Get priv and attrs pointers */
    priv = handle->priv;
    attrs = handle->attrs;
    channel_rx_info = priv->channel_rx_info;

    /* Determine flags index: 0 for non-load-share, current_channel for load-share */
    flags_idx = attrs->load_share_enabled ? priv->current_channel : 0;

    memset(endat_data, 0, sizeof(*endat_data));

    word0 = channel_rx_info->ch[priv->current_channel].pos_word0;
    word1 = channel_rx_info->ch[priv->current_channel].pos_word1;
    word2 = channel_rx_info->ch[priv->current_channel].pos_word2;
    word3 = channel_rx_info->ch[priv->current_channel].pos_word3;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            endat_data->recvd1 = word0;
            break;

        case 6:
            shift = ENDAT_RX_46BITS % (sizeof(uint32_t) * 8);
            endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            break;

        case 1:
            pos_bits = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1;

            if(pos_bits <= sizeof(uint32_t) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(uint32_t) * 8);
                endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos_bits = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1 +
                       ENDAT_NUM_BITS_F2;

            if(pos_bits <= sizeof(uint32_t) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(uint32_t) * 8);
                endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            }
            if(priv->flags[flags_idx].info1 || priv->flags[flags_idx].info2)
            {
                endat_data->recvd2 = word2;
            }
            if(priv->flags[flags_idx].info1 && priv->flags[flags_idx].info2)
            {
                endat_data->recvd3 = word3;
            }
            break;

        default:
            return SystemP_FAILURE;
            break;
    }

    return SystemP_SUCCESS;
}

/* value to be reflected should be aligned to lsb, reflected value would be aligned to lsb */
static uint64_t endat_reflect_ull_nbits(uint64_t input, uint32_t n)
{
    uint32_t i;
    uint64_t val = 0;

    /* val initialized to 0 above to handle n = 0 case, otherwise garbage would be returned */
    for(i = 0; i < n; i++)
    {
        val <<= 1;

        if(input & ((uint64_t) 1 << i))
        {
            val |= 1;
        }
    }

    return val;
}

static int32_t endat_recvd_format(endat_handle handle, int32_t cmd, endat_data *endat_data, endat_format_data *u)
{
    uint64_t pos, rev;
    endat_priv *priv;
    const endat_attrs *attrs;
    uint8_t flags_idx;

    priv = handle->priv;
    attrs = handle->attrs;
    flags_idx = attrs->load_share_enabled ? priv->current_channel : 0;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            u->addr_params.params = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC)
                                    & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
            u->addr_params.address = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC +
                                      ENDAT_NUM_BITS_PARAMETER)) &
                                     ((1 << ENDAT_NUM_BITS_ADDRESS) - 1);
            u->addr_params.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC)
                                 - 1);
            break;

        case 6:
            u->test.value = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC) & (((
                                uint64_t)1 << 40) - 1);
            u->test.f1 = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC + 40)) & 1;
            u->test.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 1:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((uint64_t) 1 << priv->pos_res) - 1);       /* mask F1 */
            pos = endat_reflect_ull_nbits(pos, priv->pos_res);
            rev = (pos & (((uint64_t) 1 << priv->pos_res) - 1)) >>
                  priv->single_turn_res[priv->current_channel];
            pos = pos & (((uint64_t) 1 << priv->single_turn_res[priv->current_channel]) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((uint64_t) 1 << priv->pos_res) -
                         1);  /* mask F1/F2 */
            pos = endat_reflect_ull_nbits(pos, priv->pos_res);
            rev = (pos & (((uint64_t) 1 << priv->pos_res) - 1)) >>
                  priv->single_turn_res[priv->current_channel];
            pos = pos & (((uint64_t) 1 << priv->single_turn_res[priv->current_channel]) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res + 1)) & 1;
            u->position_addinfo.position.f2 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + priv->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);

            if(priv->flags[flags_idx].info1 || priv->flags[flags_idx].info2)
            {
                if(priv->flags[flags_idx].info2)
                {
                    u->position_addinfo.addinfo2.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo2.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
                else
                {
                    u->position_addinfo.addinfo1.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo1.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
            }

            if(priv->flags[flags_idx].info1 && priv->flags[flags_idx].info2)
            {
                u->position_addinfo.addinfo1.addinfo = (endat_data->recvd3 >>
                                                        ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                u->position_addinfo.addinfo1.crc = endat_data->recvd3 & ((
                                                       1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            }
            break;

        default:
            return SystemP_FAILURE;
            break;
    }

    return SystemP_SUCCESS;
}

int32_t endat_recvd_process(endat_handle handle, int32_t cmd, endat_format_data *u)
{
    endat_data endat_data;
    int32_t ret;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX) ||
       (u == NULL))
    {
        return SystemP_FAILURE;
    }

    ret = endat_recvd_organize(handle, cmd, &endat_data);

    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = endat_recvd_format(handle, cmd, &endat_data, u);

    return ret;
}

#define ENDAT_USE_OTF_CRC_STATUS

#ifndef ENDAT_USE_OTF_CRC_STATUS
static uint32_t endat_make_crc_norm(uint32_t param8, uint32_t param16)
{
    /* state of the 5 flip-flops */
    uint32_t ff[5];
    /* data bit array */
    uint32_t code[24];
    /* Auxiliary variable */
    uint32_t ex;
    /* determined CRC code */
    uint32_t crc = 0;
    /* controlled variable for looping */
    int32_t i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* read 8 bit parameter into code array and convert bit sequence */
    for(i = 0; i < 8; i++)
    {
        code[i] = (param8 & 0x0080) ? 1 : 0;
        param8 <<= 1;
    }

    /* read 16 bit parameter into code array and convert bit sequence */
    for(i = 8; i < 24; i++)
    {
        code[i] = (param16 & 0x8000) ? 1 : 0;
        param16 <<= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = 0; i < 24; i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}

static uint32_t endat_make_crc_pos(uint32_t clocks, uint32_t error1,
                                    uint32_t error2, uint32_t endat22,
                                    uint64_t highpos, uint64_t lowpos)
{
    /* state of the 5 flip-flops */
    uint32_t ff[5];
    /* data bit array */
    uint32_t code[66];
    /* Auxiliary variable */
    uint32_t ex;
    /* determined CRC code */
    uint32_t crc = 0;
    /* controlled variable for looping */
    int32_t i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* transfer alarm bits to code array */
    if(endat22)
    {
        code[0] = error1;
        code[1] = error2;
    }
    else
    {
        code[1] = error1;
    }

    /* transfer low pos bits to array */
    for(i = 2; i < 34; i++)
    {
        code[i] = (lowpos & 0x00000001L) ? 1 : 0;
        lowpos >>= 1;
    }

    /* transfer high pos bits to array */
    for(i = 34; i < 66; i++)
    {
        code[i] = (highpos & 0x00000001L) ? 1 : 0;
        highpos >>= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = (endat22 ? 0 : 1); i <= (clocks + 1); i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}
#endif

/* return crc status: bit0 - position/address params/test, bit1 - additional info1, bit2 - additional info2. return -EINVAL on failure */
uint32_t endat_recvd_validate(endat_handle handle, int32_t cmd, endat_format_data *u)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    int32_t flags_idx;
    uint32_t status = 0;
#ifdef ENDAT_USE_OTF_CRC_STATUS
    uint8_t val;
#else
    uint32_t crc;
    uint64_t highpos, lowpos;
    uint64_t test;
#endif

    /* Validate handle and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return 0;
    }

#ifndef ENDAT_USE_OTF_CRC_STATUS
    /* Validate u parameter when software CRC validation is used */
    if(u == NULL)
    {
        return 0;
    }
#endif

    /* Get priv and attrs pointers */
    priv = handle->priv;
    attrs = handle->attrs;

    /* Determine flags index: 0 for non-load-share, current_channel for load-share */
    flags_idx = attrs->load_share_enabled ? priv->current_channel : 0;

#ifdef ENDAT_USE_OTF_CRC_STATUS
    val = priv->channel_rx_info->ch[priv->current_channel].crc_status;

    if(priv->flags[flags_idx].info2)
    {
        status = val & 0x1;
        /* move bit 1 to bit 2 */
        status |= (val & 0x2) << 1;

        if(priv->flags[flags_idx].info1)
        {
            /* move bit 2 to bit 1 */
            status |= (val & 0x4) >> 1;
        }
    }

    else
    {
        /* either additional info1 only present or no additional info present */
        status = val & 0x3;
    }

#else

    switch(cmd)
    {
        case 1:
            lowpos = (u->position_addinfo.position.revolution << priv->single_turn_res[priv->current_channel] |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << priv->single_turn_res[priv->current_channel] |
                       u->position_addinfo.position.position) >> 32;
            crc = endat_make_crc_pos(priv->pos_res, u->position_addinfo.position.f1, 0, 0,
                                     highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 6:
            test = endat_reflect_ull_nbits(u->test.value, 40);
            lowpos = test & 0xFFFFFFFF;
            highpos = test >> 32;
            crc = endat_make_crc_pos(40, u->position_addinfo.position.f1, 0, 0, highpos,
                                     lowpos);

            if(u->test.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            lowpos = (u->position_addinfo.position.revolution << priv->single_turn_res[priv->current_channel] |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << priv->single_turn_res[priv->current_channel] |
                       u->position_addinfo.position.position) >> 32;
            crc = endat_make_crc_pos(priv->pos_res, u->position_addinfo.position.f1,
                                     u->position_addinfo.position.f2, 1, highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }

            if(priv->flags[flags_idx].info1)
            {
                crc = endat_make_crc_norm((u->position_addinfo.addinfo1.addinfo >> 16) & 0xFF,
                                          u->position_addinfo.addinfo1.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo1.crc == crc)
                {
                    status |= 0x1 << 1;
                }
            }

            if(priv->flags[flags_idx].info2)
            {
                crc = endat_make_crc_norm((u->position_addinfo.addinfo2.addinfo >> 16) & 0xFF,
                                          u->position_addinfo.addinfo2.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo2.crc == crc)
                {
                    status |= 0x1 << 2;
                }
            }

            break;

        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
            crc = endat_make_crc_norm(u->addr_params.address, u->addr_params.params);

            if(u->addr_params.crc == crc)
            {
                status = 0x1;
            }
            break;

        default:
            break;
    }

#endif

    return status;
}

/* ========================================================================== */
/*                       Command Build Helper Functions                       */
/* ========================================================================== */

/**
 * \brief Calculate the number of additional info fields for a channel
 *
 * \param flags Pointer to endat_flags structure for the channel
 *
 * \return Number of available info fields (0, 1, or 2)
 */
static inline uint32_t endat_calc_info_count(const endat_flags *flags)
{
    uint32_t count = 0;
    if(flags->info1)
    {
        count++;
    }
    if(flags->info2)
    {
        count++;
    }
    return count;
}

/**
 * \brief Apply additional info flags to command word
 *
 * \param cmd_word1 Pointer to command word1 to modify
 * \param flags Pointer to endat_flags structure
 */
static inline void endat_apply_addinfo_flags(volatile uint32_t *cmd_word1,
                                            const endat_flags *flags)
{
    if(flags->info1)
    {
        *cmd_word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
    }
    if(flags->info2)
    {
        *cmd_word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
    }
}

/**
 * \brief Calculate info counts for all channels in load-share mode
 *
 * \param priv Pointer to endat_priv with flags for all channels
 * \param info_array Pointer to array of size ENDAT_NUM_CH_PER_SLICE_MAX
 */
static inline void endat_calc_loadshare_info_counts(const endat_priv *priv,
                                                    uint32_t *info_array)
{
    uint32_t ch;
    for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
    {
        info_array[ch] = endat_calc_info_count(&priv->flags[ch]);
    }
}

/*
 * Check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called either after endat_get_command
 * or by diagnostic initialization code where it is only 2.1 commands used
 */
int32_t endat_command_build(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *endat_pruicss_xchg;
    uint32_t info[ENDAT_NUM_CH_PER_SLICE_MAX];
    uint32_t ch;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    /* Validate cmd_supplement for commands that require it */
    if((cmd == 2) || (cmd == 3) || (cmd == 4) || (cmd == 7) || (cmd >= 9 && cmd <= 11) || (cmd == 13) || (cmd == 14))
    {
        if(cmd_supplement == NULL)
        {
            return SystemP_FAILURE;
        }
    }

    priv = handle->priv;
    attrs = handle->attrs;
    endat_pruicss_xchg = priv->pruicss_xchg;

    /* first clear command parameters to be safe */
    memset(&endat_pruicss_xchg->cmd, 0, sizeof(endat_pruicss_xchg->cmd));

    switch(cmd)
    {
        case 1:
            /* ENDAT_CMD_SEND_POSITION_VALUES - EnDAT 2.1 position values */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                        endat_pruicss_xchg->cmd[ch].word1 = priv->pos_rx_bits_21_cmd[ch] | (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);
                    }
                }
            }
            else
            {
                /* Single-pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                endat_pruicss_xchg->cmd[0].word1 = priv->pos_rx_bits_21_cmd[priv->current_channel] | (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);
            }
            break;
        case 2:
            /* ENDAT_CMD_SEL_MEM_AREA - EnDAT 2.1 select memory area */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEL_MEM_AREA;
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->address[ch] & 0x80) >> 7) |
                                                             (((cmd_supplement->address[ch] << 1) & 0xFE) << 8);
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                            (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEL_MEM_AREA;
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address[0] & 0x80) >> 7) |
                                                    (((cmd_supplement->address[0] << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }

            break;

        case 3:
            /* ENDAT_CMD_RECEIVE_PARAMETERS - EnDAT 2.1 receive parameters */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->address[ch] & 0x80) >> 7) |
                                                             (((cmd_supplement->address[ch] << 1) & 0xFE) << 8);
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->data[ch] & 0x8000) >> 7) |
                                                             (((cmd_supplement->data[ch] << 1) & 0xFF00) << 8) |
                                                             (((cmd_supplement->data[ch] << 9) & 0xFE00) << 16);
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                            (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address[0] & 0x80) >> 7) |
                                                    (((cmd_supplement->address[0] << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->data[0] & 0x8000) >> 7) |
                                                    (((cmd_supplement->data[0] << 1) & 0xFF00) << 8) |
                                                    (((cmd_supplement->data[0] << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 4:
            /* ENDAT_CMD_SEND_PARAMETERS - EnDAT 2.1 send parameters */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_PARAMETERS;
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->address[ch] & 0x80) >> 7) |
                                                             (((cmd_supplement->address[ch] << 1) & 0xFE) << 8);
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                            (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_PARAMETERS;
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address[0] & 0x80) >> 7) |
                                                    (((cmd_supplement->address[0] << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 5:
            /* ENDAT_CMD_RECEIVE_RESET - EnDAT 2.1 receive reset */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_RECEIVE_RESET;
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                            (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_RESET;
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 6:
            /* ENDAT_CMD_SEND_TEST_VALUES - EnDAT 2.1 send test values */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_TEST_VALUES;
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                                            (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_TEST_VALUES;
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 7:
            /* ENDAT_CMD_RECEIVE_TEST_COMMAND - Test command */
            if(attrs->load_share_enabled)
            {
                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->address[ch] & 0x80) >> 7)
                                                           | (((cmd_supplement->address[ch] << 1) & 0xFE) << 8);
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
                    }
                }
            }
            else
            {
                /* Single-channel mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address[0] & 0x80) >> 7)
                                                   | (((cmd_supplement->address[0] << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 8:
            /* ENDAT_CMD_SEND_POSVAL_WITH_DATA - EnDAT 2.2 position with additional info */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single-channel mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 9:
            /* ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL - EnDAT 2.2 position with memory select */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                        if(cmd_supplement->has_block_address[ch])
                        {
                            endat_pruicss_xchg->cmd[ch].word1 |= (1 << 24);
                            endat_pruicss_xchg->cmd[ch].word2 = cmd_supplement->address[ch] |
                                                                (cmd_supplement->block[ch] << 24);
                        }
                        else
                        {
                            endat_pruicss_xchg->cmd[ch].word2 = cmd_supplement->address[ch];
                        }

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                if(cmd_supplement->has_block_address[0])
                {
                    endat_pruicss_xchg->cmd[0].word1 |= (1 << 24);
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address[0] |
                                                       (cmd_supplement->block[0] << 24);
                }
                else
                {
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address[0];
                }

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 10:
            /* ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM - EnDAT 2.2 position with parameter receive */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                        endat_pruicss_xchg->cmd[ch].word2 = cmd_supplement->address[ch];
                        /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                        endat_pruicss_xchg->cmd[ch].word2 |= ((cmd_supplement->data[ch] & 0xFF) << 16) |
                                                             (cmd_supplement->data[ch] & 0xFF00);

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address[0];
                /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                endat_pruicss_xchg->cmd[0].word2 |= ((cmd_supplement->data[0] & 0xFF) << 16) |
                                                    (cmd_supplement->data[0] & 0xFF00);

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 11:
            /* ENDAT_CMD_SEND_POSVAL_SEND_PARAM - EnDAT 2.2 position with parameter send */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                        endat_pruicss_xchg->cmd[ch].word2 = cmd_supplement->address[ch];

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address[0];

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 12:
            /* ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST - EnDAT 2.2 position with error reset */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 13:
            /* ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD - EnDAT 2.2 position with test command */
            if(attrs->load_share_enabled)
            {
                /* Calculate info counts for all channels using helper */
                endat_calc_loadshare_info_counts(priv, info);

                /* Build command for each channel */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                        endat_pruicss_xchg->cmd[ch].word1 = (priv->pos_rx_bits_22_cmd[ch] + info[ch] *
                                                             ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                            (ENDAT_TX_6BITS << 8) |
                                                            ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                        endat_pruicss_xchg->cmd[ch].word2 = cmd_supplement->address[ch];

                        /* Apply additional info flags using helper */
                        endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[ch].word1, &priv->flags[ch]);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                info[0] = endat_calc_info_count(&priv->flags[0]);

                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                endat_pruicss_xchg->cmd[0].word1 = (priv->pos_rx_bits_22_cmd[priv->current_channel] + info[0] *
                                                    ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                                   (ENDAT_TX_6BITS << 8) |
                                                   ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address[0];

                /* Apply additional info flags using helper */
                endat_apply_addinfo_flags(&endat_pruicss_xchg->cmd[0].word1, &priv->flags[0]);
            }

            break;

        case 14:
            /* ENDAT_CMD_RECEIVE_COMMUNICATION_CMD - EnDAT 2.2 communication command */
            if(attrs->load_share_enabled)
            {
                /* Build command for all channels */
                for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        endat_pruicss_xchg->cmd[ch].word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->address[ch] & 0x80) >> 7) |
                                                             (((cmd_supplement->address[ch] << 1) & 0xFE) << 8);
                        endat_pruicss_xchg->cmd[ch].word0 |= ((cmd_supplement->data[ch] & 0x8000) >> 7) |
                                                             (((cmd_supplement->data[ch] << 1) & 0xFF00) << 8) |
                                                             (((cmd_supplement->data[ch] << 9) & 0xFE00) << 16);
                        endat_pruicss_xchg->cmd[ch].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                            ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);
                    }
                }
            }
            else
            {
                /* Single pru mode */
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address[0] & 0x80) >> 7) |
                                                    (((cmd_supplement->address[0] << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->data[0] & 0x8000) >> 7) |
                                                    (((cmd_supplement->data[0] << 1) & 0xFF00) << 8) |
                                                    (((cmd_supplement->data[0] << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                                   ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);
            }
            break;

        default:
            cmd = SystemP_FAILURE;
            break;
    }

    return SystemP_SUCCESS;
}

int32_t endat_command_send(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;
    /*for load share mode set mask for all connected channels*/
    if(attrs->load_share_enabled)
    {
        pruicss_xchg->config[0].trigger = pruicss_xchg->config[0].channel==1?0x1:0;
        pruicss_xchg->config[1].trigger = pruicss_xchg->config[1].channel==2?0x1:0;
        pruicss_xchg->config[2].trigger = pruicss_xchg->config[2].channel==4?0x1:0;

    }
    else
    {
        pruicss_xchg->config[0].trigger = 0x1;
    }

    return SystemP_SUCCESS;
}

int32_t endat_command_wait(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;
    uint32_t loop_count;
    uint8_t all_cleared;
    uint32_t ch;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Wait for command completion with timeout
     * EnDAT cycle time depends on encoder parameters, frequency, and cable length
     * Use configurable timeout to handle different encoder configurations
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
            /* Wait until host trigger bits are cleared for all enabled channels */
            all_cleared = 1;
            for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
            {
                if(attrs->channel_mask & (1U << ch))
                {
                    if(pruicss_xchg->config[ch].trigger & 0x1)
                    {
                        all_cleared = 0;
                        break;
                    }
                }
            }
            if(all_cleared)
            {
                break;
            }
        }
        else
        {
            /* Single channel mode: wait for channel 0 trigger to clear */
            if((pruicss_xchg->config[0].trigger & 0x1) == 0)
            {
                break;
            }
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

int32_t endat_command_process(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    int32_t status;

    /* Validate handle */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    status = endat_command_build(handle, cmd, cmd_supplement);

    if(status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    status = endat_command_send(handle);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    status = endat_command_wait(handle);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    return SystemP_SUCCESS;
}

static int32_t endat_get_pos_res(endat_handle handle, int32_t *pos_res)
{
    endat_priv *priv;
    int32_t cmd;
    endat_cmd_supplement cmd_supplement;
    uint32_t word;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;
    int32_t status;

    if(pos_res == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word13 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_13;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_13;
        cmd_supplement.address[2] = APP_ENDAT_WORD_13;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    word = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    word &= (1 << ENDAT_NUM_BITS_VALID_PAGE0_WORD13) - 1;
    *pos_res = word;
    return SystemP_SUCCESS;
}

static int32_t endat_get_multi_turn_res(endat_handle handle, int32_t *multi_turn_res)
{
    endat_priv *priv;
    int32_t cmd;
    endat_cmd_supplement cmd_supplement;
    uint32_t word;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;
    int32_t status;

    if(multi_turn_res == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word1 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_1;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_1;
        cmd_supplement.address[2] = APP_ENDAT_WORD_1;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    word = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    word &= (1 << ENDAT_NUM_BITS_VALID_PAGE1_WORD1) - 1;
    *multi_turn_res = word;
    return SystemP_SUCCESS;
}

static int32_t endat_get_id(endat_handle handle)
{
    endat_priv *priv;
    int32_t cmd, status;
    endat_cmd_supplement cmd_supplement;
    uint32_t word0, word1, word2;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word8 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_8;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_8;
        cmd_supplement.address[2] = APP_ENDAT_WORD_8;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word0 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    /* send parameter for word9 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_9;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_9;
        cmd_supplement.address[2] = APP_ENDAT_WORD_9;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word1 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    /* send parameter for word10 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_10;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_10;
        cmd_supplement.address[2] = APP_ENDAT_WORD_10;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word2 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    priv->id.binary = word1 | word2 << 16;
    /* swap the two ascii's so that printing as string will give what is required */
    priv->id.ascii = ((word0 & 0xFF) << 8) | ((word0 & 0xFF00) >> 8);

    return SystemP_SUCCESS;
}

static int32_t endat_get_sn(endat_handle handle)
{
    endat_priv *priv;
    int32_t cmd, status;
    endat_cmd_supplement cmd_supplement;
    uint32_t word0, word1, word2;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word11 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_11;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_11;
        cmd_supplement.address[2] = APP_ENDAT_WORD_11;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word0 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    /* send parameter for word12 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_12;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_12;
        cmd_supplement.address[2] = APP_ENDAT_WORD_12;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word1 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    /* send parameter for word13 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_13;
    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_13;
        cmd_supplement.address[2] = APP_ENDAT_WORD_13;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    word2 = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    priv->sn.ascii_lsb = word0 & 0xFF;
    priv->sn.binary = ((word0 & 0xFF00) >> 8) | (word1 << 8) | ((
                          word2 & 0xFF) << 24);
    priv->sn.ascii_msb = (word2 & 0xFF00) >> 8;

    return SystemP_SUCCESS;
}

static int32_t endat_get_command_set(endat_handle handle)
{
    endat_priv *priv;
    int32_t cmd, status;
    endat_cmd_supplement cmd_supplement;
    uint32_t word;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 2 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word5 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_5;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_5;
        cmd_supplement.address[2] = APP_ENDAT_WORD_5;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    word = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    priv->cmd_set_2_2 = (word & 0x1) && !(word & 0x2);
    priv->has_safety[ch] = (word & 0x4) && !(word & 0x8);

    return SystemP_SUCCESS;
}

static int32_t endat_get_type(endat_handle handle)
{
    endat_priv *priv;
    int32_t cmd, status;
    endat_cmd_supplement cmd_supplement;
    uint32_t word;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;
    }
    
    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word13 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_14;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_14;
        cmd_supplement.address[2] = APP_ENDAT_WORD_14;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);

    word = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    priv->type[ch] = (word & (1 << 15)) ? ENDAT_ENCODER_TYPE_ROTARY: ENDAT_ENCODER_TYPE_LINEAR;

    return SystemP_SUCCESS;
}

static int32_t endat_get_step(endat_handle handle, int32_t *step)
{
    endat_priv *priv;
    int32_t cmd;
    endat_cmd_supplement cmd_supplement;
    uint32_t word;
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    const endat_attrs *attrs;
    int32_t status;

    if(step == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;
    attrs = handle->attrs;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2;
    cmd_supplement.address[0] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
        cmd_supplement.address[2] = ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_CMD_PROCESS_DELAY_12MS_US);

    /* send parameter for word4 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_4;

    if(attrs->load_share_enabled)
    {
        cmd_supplement.address[1] = APP_ENDAT_WORD_4;
        cmd_supplement.address[2] = APP_ENDAT_WORD_4;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);
    word = (channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);

    /* send parameter for word5 */
    cmd = 4;
    cmd_supplement.address[0] = APP_ENDAT_WORD_5;

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    /* delay copied from fw */
    ClockP_usleep(ENDAT_PARAM_READ_DELAY_2MS_US);
    word |= ((channel_rx_info->ch[ch].pos_word0 >> (ENDAT_NUM_BITS_POSITION_CRC))
             & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1)) << 16;

    *step = word;
    return SystemP_SUCCESS;
}

int32_t endat_get_encoder_info(endat_handle handle)
{
    endat_priv *priv;
    int32_t ret;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    ret = endat_get_pos_res(handle, &priv->pos_res);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = endat_get_multi_turn_res(handle, &priv->multi_turn_res[priv->current_channel]);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    if(!priv->multi_turn_res[priv->current_channel])
    {
        priv->single_turn_res[priv->current_channel] = priv->pos_res;
    }
    else
    {
        priv->multi_turn_res[priv->current_channel] = log2(priv->multi_turn_res[priv->current_channel]);
        priv->single_turn_res[priv->current_channel] = priv->pos_res - priv->multi_turn_res[priv->current_channel];
    }

    ret = endat_get_step(handle, &priv->step[priv->current_channel]);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }
    /* Calculate rx frame size for all three channels and store in different variables */

    priv->pos_rx_bits_21_cmd[priv->current_channel] = priv->pos_res + ENDAT_NUM_BITS_POSITION_CRC +
            ENDAT_NUM_BITS_F1;
    priv->pos_rx_bits_22_cmd[priv->current_channel] = priv->pos_rx_bits_21_cmd[priv->current_channel] + ENDAT_NUM_BITS_F2;

    ret = endat_get_id(handle);

    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = endat_get_sn(handle);

    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = endat_get_type(handle);

    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = endat_get_command_set(handle);

    return ret;
}

int32_t endat_get_prop_delay(endat_handle handle, uint32_t *prop_delay)
{
    endat_priv *priv;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) || (prop_delay == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    *prop_delay = priv->pruicss_xchg->ch[priv->current_channel].prop_delay;
    return SystemP_SUCCESS;
}

int32_t endat_addinfo_track(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    int32_t c7_c4, c3_c0;
    int32_t ch;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Get priv and attrs pointers */
    priv = handle->priv;
    attrs = handle->attrs;

    /* reset stops additional info's */
    if(cmd == 5)
    {
        if(attrs->load_share_enabled)
        {
            /* Reset flags for all enabled channels */
            for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
            {
                if(attrs->channel_mask & (1U << ch))
                {
                    priv->flags[ch].info1 = FALSE;
                    priv->flags[ch].info2 = FALSE;
                }
            }
        }
        else
        {
            /* Reset flags for */
            priv->flags[0].info1 = FALSE;
            priv->flags[0].info2 = FALSE;
        }
        return SystemP_SUCCESS;
    }

    if(cmd != 9)
    {
        return SystemP_SUCCESS;
    }

    /* Validate cmd_supplement for command 9 (MRS - Memory Read Select) */
    if(cmd_supplement == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Command 9: MRS (Memory Read Select) - Process address fields */
    if(attrs->load_share_enabled)
    {
        /* Process each enabled channel with its own address */
        for(ch = 0; ch < ENDAT_NUM_CH_PER_SLICE_MAX; ch++)
        {
            if(attrs->channel_mask & (1U << ch))
            {
                /* Check for Additional Info 1 */
                c7_c4 = (cmd_supplement->address[ch] & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
                        ENDAT_MRS_SHIFT_C7_C4;
                c3_c0 = cmd_supplement->address[ch] & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

                if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1)
                {
                    if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
                    {
                        priv->flags[ch].info1 = FALSE;
                    }
                    else
                    {
                        priv->flags[ch].info1 = TRUE;
                    }
                }

                /* Check for Additional Info 2 */
                c7_c4 = (cmd_supplement->address[ch] & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
                        ENDAT_MRS_SHIFT_C7_C4;
                c3_c0 = cmd_supplement->address[ch] & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

                if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2)
                {
                    if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
                    {
                        priv->flags[ch].info2 = FALSE;
                    }
                    else
                    {
                        priv->flags[ch].info2 = TRUE;
                    }
                }
            }
        }
    }
    else
    {
        /* Single-PRU mode: same MRS code is used for all channels  */

        /* Check for Additional Info 1 */
        c7_c4 = (cmd_supplement->address[0] & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
                ENDAT_MRS_SHIFT_C7_C4;
        c3_c0 = cmd_supplement->address[0] & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

        if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1)
        {
            if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
            {
                priv->flags[0].info1 = FALSE;
            }
            else
            {
                priv->flags[0].info1 = TRUE;
            }
        }

        /* Check for Additional Info 2 */
        c7_c4 = (cmd_supplement->address[0] & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
                ENDAT_MRS_SHIFT_C7_C4;
        c3_c0 = cmd_supplement->address[0] & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

        if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2)
        {
            if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
            {
                priv->flags[0].info2 = FALSE;
            }
            else
            {
                priv->flags[0].info2 = TRUE;
            }
        }
    }

    return SystemP_SUCCESS;
}

static int32_t endat_config_global_rx_arm_cnt(endat_handle handle, uint16_t val)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Configure only enabled channels based on channel mask */
    if(attrs->pruicss_slice == 1)
    {
       if(attrs->channel_mask & (1U << 0))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + 2, val);
       }
       if(attrs->channel_mask & (1U << 1))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG1_REG + 2, val);
       }
       if(attrs->channel_mask & (1U << 2))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG1_REG + 2, val);
       }
    }
    else
    {
       if(attrs->channel_mask & (1U << 0))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + 2, val);
       }
       if(attrs->channel_mask & (1U << 1))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG1_REG + 2, val);
       }
       if(attrs->channel_mask & (1U << 2))
       {
           HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG1_REG + 2, val);
       }
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_rx_arm_cnt(endat_handle handle, uint16_t val)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;
    int32_t ch;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    ch = priv->current_channel;
    if(attrs->pruicss_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + ch * 8 + 2, val);
    }
    else
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + ch * 8 + 2, val);
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_wire_delay(endat_handle handle, uint16_t val)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;
    int32_t ch;
    uint16_t regval;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    ch = priv->current_channel;
    if(attrs->pruicss_slice == 1)
    {
        regval = HW_RD_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG + ch *
                                   8);
    }
    else
    {
        regval = HW_RD_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG + ch *
                                   8);
    }

    /* clear wire delay bits, keep other bits as is */
    regval &= 0xF800;
    /* restrict wire delay to wire delay bits only */
    val &= 0x7FF;
    regval |= val;
    if(attrs->pruicss_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG + ch * 8, regval);
    }
    else
    {
        HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG + ch * 8, regval);
    }

    return SystemP_SUCCESS;
}

static int32_t endat_config_clock_reg(endat_handle handle, endat_clk_cfg *clk_cfg)
{
    void *pruicss_cfg;
    const endat_attrs *attrs;
    endat_priv *priv;
    uint32_t rx_reg_val;
    uint32_t tx_reg_val;

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Configure RX and TX CFG registers based on PRU slice */
    if(attrs->pruicss_slice)
    {
        /* Slice 1 - Read-Modify-Write for RX CFG */
        rx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG);
        rx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SAMPLE_SIZE_MASK);
        rx_reg_val |= (clk_cfg->rx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_SHIFT) |
                      (attrs->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_SHIFT) |
                      (0x1 << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SB_POL_SHIFT) |
                      (clk_cfg->rx_div_attr << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SAMPLE_SIZE_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG, rx_reg_val);

        /* Slice 1 - Read-Modify-Write for TX CFG */
        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_FRAC_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_SHIFT) |
                      (attrs->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, tx_reg_val);
    }
    else
    {
        /* Slice 0 - Read-Modify-Write for RX CFG */
        rx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG);
        rx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SAMPLE_SIZE_MASK);
        rx_reg_val |= (clk_cfg->rx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_SHIFT) |
                      (attrs->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_SHIFT) |
                      (0x1 << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_SHIFT) |
                      (clk_cfg->rx_div_attr << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SAMPLE_SIZE_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG, rx_reg_val);

        /* Slice 0 - Read-Modify-Write for TX CFG */
        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_FRAC_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_SHIFT) |
                      (attrs->is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, tx_reg_val);
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_tst_delay(endat_handle handle, uint16_t delay)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;
    int32_t ch;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);
    ch = priv->current_channel;
    if(attrs->pruicss_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + ch * 8, delay);
    }
    else
    {
      HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + ch * 8, delay);
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_rx_clock_disable(endat_handle handle, uint16_t val)
{
    endat_priv *priv;
    endat_pruicss_xchg *pruicss_xchg;
    int32_t ch;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }


    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;
    ch = priv->current_channel;
    pruicss_xchg->ch[ch].rx_clk_less = val;

    return SystemP_SUCCESS;
}

static int32_t endat_handle_prop_delay(endat_handle handle, uint32_t prop_delay)
{
    uint16_t clock_dis;
    uint16_t temp;
    endat_priv *priv;
    const endat_attrs *attrs;
    float ct;
    int32_t status;

    priv = handle->priv;
    attrs = handle->attrs;

    /* Validate endat_freq is not zero to avoid division by zero */
    if(priv->endat_freq == 0)
    {
        return SystemP_FAILURE;
    }

    /* One cycle period for endat clock is 1/endat_freq */
    ct = (float)1000000000 / priv->endat_freq;

    /* if propagation delay is more than half clock cycle time (2/endat frequency) then we have to reduce clock cycles for rx*/
    /*ASSUMPTION prop_delay is in ns*/
    if(prop_delay > (ct/2))
    {
        clock_dis = floor(prop_delay/ct);
        /* convert propagation delay into rx arm counts */
        temp = ((uint16_t)(((float)prop_delay * attrs->core_clk_freq )/1000000000)) * ENDAT_DELAY_COUNTER_INCREMENT;

        status = endat_config_rx_arm_cnt(handle, temp);
        if(status != SystemP_SUCCESS)
        {
            return status;
        }

        /* propagation delay/cycle_time */
        status = endat_config_rx_clock_disable(handle, clock_dis);
        if(status != SystemP_SUCCESS)
        {
            return status;
        }
    }
    else
    {
        /*Without propagation delay rx arm count is always equal to rx_en_cnt*/
        status = endat_config_rx_arm_cnt(handle, priv->rx_en_cnt);
        if(status != SystemP_SUCCESS)
        {
            return status;
        }

        status = endat_config_rx_clock_disable(handle, 0);
        if(status != SystemP_SUCCESS)
        {
            return status;
        }
    }

    return SystemP_SUCCESS;
}

/*function to set propagation delay, val always will be in ns*/
int32_t endat_config_propagation_delay(endat_handle handle, uint32_t val)
{
    int32_t ret;
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;
    int32_t ch;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /*handle propagation delay*/
    ret = endat_handle_prop_delay(handle, val);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    /*convert from ns to pru count*/
    val =  ((uint16_t)(((float)val * attrs->core_clk_freq)/1000000000));

    pruicss_xchg = priv->pruicss_xchg;
    ch = priv->current_channel;
    pruicss_xchg->ch[ch].prop_delay = val;

    return SystemP_SUCCESS;
}

static int32_t endat_set_continuous_mode(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        pruicss_xchg->config[0].trigger |= pruicss_xchg->config[0].channel==1?(0x1 << 7 | 0x1):0;
        pruicss_xchg->config[1].trigger |= pruicss_xchg->config[1].channel==2?(0x1 << 7 | 0x1):0;
        pruicss_xchg->config[2].trigger |= pruicss_xchg->config[2].channel==4?(0x1 << 7 | 0x1):0;
    }
    else
    {
        pruicss_xchg->config[0].trigger |= (0x1 << 7 | 0x1);
    }

    return SystemP_SUCCESS;
}

static int32_t endat_calc_clock(endat_handle handle, uint32_t freq, endat_clk_cfg *clk_cfg)
{
    uint32_t ns;
    uint64_t rx_source_freq;
    uint64_t tx_source_freq;
    const endat_attrs *attrs = handle->attrs;

    /* Validate freq is not zero to avoid division by zero */
    if(freq == 0)
    {
        return SystemP_FAILURE;
    }

    if(attrs->is_core_clk == 1)
    {
        rx_source_freq = attrs->core_clk_freq;
        tx_source_freq = attrs->core_clk_freq;
    }
    else
    {
        rx_source_freq = attrs->uart_clk_freq;
        tx_source_freq = attrs->uart_clk_freq;
    }

    /* Validate source frequencies are not zero */
    if((rx_source_freq == 0) || (tx_source_freq == 0))
    {
        return SystemP_FAILURE;
    }

    if(freq > 16000000 || (freq > 12000000 && freq < 16000000))
    {
        return SystemP_FAILURE;
    }

    if((freq != 16000000) && (rx_source_freq % (freq * ENDAT_RX_OVERSAMPLING_RATE))&&(tx_source_freq % (freq)))
    {
        return SystemP_FAILURE;
    }

    ns = ENDAT_DELAY_COUNTER_INCREMENT*(2*attrs->core_clk_freq/freq); /* rx arm >= 2 clock */

    /* should be divisible by 5 */
    if(ns % 5)
    {
        ns /= 5, ns += 1,  ns *= 5;
    }

    clk_cfg->tx_div = tx_source_freq / freq - 1;
    clk_cfg->rx_div = rx_source_freq / (freq * ENDAT_RX_OVERSAMPLING_RATE) - 1;
    clk_cfg->rx_en_cnt = ns;
    clk_cfg->rx_div_attr = ENDAT_RX_OVERSAMPLING_RATE - 1;

    if(freq == 16000000 && (attrs->is_core_clk != 1  && attrs->uart_clk_freq == 192000000))
    {
        clk_cfg->rx_div_attr |= ENDAT_RX_FRAC_DIV;
    }
    return SystemP_SUCCESS;
}

int32_t endat_config_clock(endat_handle handle, uint32_t freq)
{
    endat_clk_cfg clk_cfg;
    uint32_t delay;
    int32_t i;
    endat_priv *priv;
    const endat_attrs *attrs;
    int32_t ret;

    /* Validate core_clk_freq is not zero to avoid division by zero */
    /* Validate handle and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (handle->attrs->core_clk_freq == 0) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    ret = endat_calc_clock(handle, freq, &clk_cfg);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }
    priv->endat_freq = freq;
    priv->rx_en_cnt = clk_cfg.rx_en_cnt;
    ret = endat_config_clock_reg(handle, &clk_cfg);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    /*handle propagation delay*/
    if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    {
        for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
        {
            if(attrs->channel_mask & (1 << i))
            {
                priv->current_channel = i;
                delay = priv->pruicss_xchg->ch[i].prop_delay;
                /*convert into ns from pru count - using 64-bit to avoid overflow*/
                delay = (uint32_t)(((uint64_t)delay * 1000000000ULL) / attrs->core_clk_freq);

                ret = endat_handle_prop_delay(handle, delay);
                if(ret != SystemP_SUCCESS)
                {
                    return ret;
                }
            }
        }
    }
    else
    {
        delay = priv->pruicss_xchg->ch[priv->current_channel].prop_delay;
        /*convert into ns from pru count - using 64-bit to avoid overflow*/
        delay = (uint32_t)(((uint64_t)delay * 1000000000ULL) / attrs->core_clk_freq);
        ret = endat_handle_prop_delay(handle, delay);
        if(ret != SystemP_SUCCESS)
        {
            return ret;
        }
    }
    return SystemP_SUCCESS;
}

static int32_t endat_clear_continuous_mode(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(attrs->load_share_enabled)
    {
        pruicss_xchg->config[0].trigger &= ~(0x1 << 7);
        pruicss_xchg->config[1].trigger &= ~(0x1 << 7);
        pruicss_xchg->config[2].trigger &= ~(0x1 << 7);
    }
    else
    {
        pruicss_xchg->config[0].trigger &= ~(0x1 << 7);
    }

    return SystemP_SUCCESS;
}

int32_t endat_start_continuous_mode(endat_handle handle)
{
    int32_t cmd;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    cmd = endat_command_build(handle, 1, NULL);

    if(cmd != SystemP_SUCCESS)
    {
        return cmd;
    }

    if(endat_set_continuous_mode(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t endat_stop_continuous_mode(endat_handle handle)
{
    int32_t status;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    if(endat_clear_continuous_mode(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    status = endat_command_wait(handle);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_host_trigger(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(attrs->load_share_enabled)
    {
         pruicss_xchg->config[0].opmode=(pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_HOST_TRIGGER:0;
         pruicss_xchg->config[1].opmode=(pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_HOST_TRIGGER:0;
         pruicss_xchg->config[2].opmode=(pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_HOST_TRIGGER:0;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_HOST_TRIGGER;
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_periodic_trigger_cmp_mode(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(attrs->load_share_enabled)
    {
         pruicss_xchg->config[0].opmode=(pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[0].opmode;
         pruicss_xchg->config[1].opmode=(pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[1].opmode;
         pruicss_xchg->config[2].opmode=(pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[2].opmode;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_CMP_PERIODIC;
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_periodic_trigger_cap_mode(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(attrs->load_share_enabled)
    {
         pruicss_xchg->config[0].opmode = (pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[0].opmode;
         pruicss_xchg->config[1].opmode = (pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[1].opmode;
         pruicss_xchg->config[2].opmode = (pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[2].opmode;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_CAP_PERIODIC;
    }

    return SystemP_SUCCESS;
}

int32_t endat_config_channel(endat_handle handle, uint32_t ch)
{
    endat_priv *priv;
    endat_pruicss_xchg *pruicss_xchg;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) || (ch >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    pruicss_xchg->config[0].channel = 1 << ch;
    priv->current_channel = ch;

    return SystemP_SUCCESS;
}

int32_t endat_config_multi_channel_mask(endat_handle handle, uint8_t mask, uint8_t load_share_enabled)
{
    endat_priv *priv;
    int32_t status;

    /* Validate parameters and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (mask == 0) ||
       (mask > ENDAT_CHANNEL_MASK) ||
       (load_share_enabled > 1))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    if(load_share_enabled)
    {
        priv->pruicss_xchg->config[0].channel = mask & (1<<0);
        priv->pruicss_xchg->config[1].channel = mask & (1<<1);
        priv->pruicss_xchg->config[2].channel = mask & (1<<2);
        status = endat_config_primary_core_mask(handle, mask);/*select primary core for global configuration*/
        if(status != SystemP_SUCCESS)
        {
            return status;
        }

        status = endat_config_syn_bits(handle, mask); /* configure syn bits for synchronization before any global config*/
        if(status != SystemP_SUCCESS)
        {
            return status;
        }
    }
    else
    {
       priv->pruicss_xchg->config[0].channel = mask;
    }

    return SystemP_SUCCESS;
}

static int32_t endat_config_syn_bits(endat_handle handle, uint8_t mask)
{
    endat_priv *priv;

    priv = handle->priv;

    priv->pruicss_xchg->endat_ch0_syn_bit = (mask&(1<<0))?0x1:0;
    priv->pruicss_xchg->endat_ch1_syn_bit = (mask&(1<<1))?0x2:0;
    priv->pruicss_xchg->endat_ch2_syn_bit = (mask&(1<<2))?0x4:0;

    return SystemP_SUCCESS;
}

static int32_t endat_enable_load_share_mode(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;
    uint32_t reg_val;

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    if(attrs->pruicss_slice == 1)
    {
        reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        reg_val |= ENDAT_LOAD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, reg_val);
    }
    else
    {
        reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        reg_val |= ENDAT_LOAD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, reg_val);
    }

    return SystemP_SUCCESS;
}

static int32_t endat_config_primary_core_mask(endat_handle handle, uint8_t mask)
{
    endat_priv *priv;
    priv = handle->priv;

    switch(mask)
    {
        case 1:  /*only channel0 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x1;
            break;
        case 2: /*channel1 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x2;
            break;
        case 3:               /*channel0 and channel1 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x1;
            break;
        case 4:  /*channel2 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x4;
            break;
        case 5:               /*channel0 and channel2 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x4;
            break;
        case 6:                    /*channel1 and channel2 connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x4;
            break;
        case 7:                       /*all three channel connected*/
            priv->pruicss_xchg->endat_primary_core_mask = 0x4;
            break;
        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

uint8_t endat_multi_channel_detected(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return 0;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)  /* for loadshare mode*/
    {
        return (priv->pruicss_xchg->config[0].channel | priv->pruicss_xchg->config[1].channel | priv->pruicss_xchg->config[2].channel);
    }
    else
    {
        return priv->pruicss_xchg->config[0].channel;
    }
}

int32_t endat_multi_channel_set_cur(endat_handle handle, uint32_t ch)
{
    endat_priv *priv;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (ch >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->current_channel = ch;
    priv->pos_res =  priv->pos_rx_bits_21_cmd[ch] - (ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1);

    return SystemP_SUCCESS;
}

int32_t endat_wait_initialization(endat_handle handle, uint32_t timeout, uint8_t mask)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;
    int32_t i;
    int32_t status = SystemP_SUCCESS;
    uint8_t init_complete = 0;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (mask == 0) ||
       (mask > ENDAT_CHANNEL_MASK))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    for(i = 0; (i < timeout) && (init_complete == 0); i++)
    {
        if(attrs->load_share_enabled)  /* for loadshare mode*/
        {
            switch (mask)
            {
                case 1:  /*channel 0 connected*/
                    if((pruicss_xchg->config[0].status & 1))
                        init_complete = 1;
                    break;
                case 2: /*channel 1 connected*/
                    if((pruicss_xchg->config[1].status & 1))
                        init_complete = 1;
                    break;
                case 3:               /*channel 0 and 1 connected*/
                    if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[1].status & 1))
                        init_complete = 1;
                    break;
                case 4:  /*channel 2 connected*/
                    if((pruicss_xchg->config[2].status & 1))
                        init_complete = 1;
                    break;
                case 5:               /*channel 0 and 2 connected*/
                    if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[2].status & 1))
                        init_complete = 1;
                    break;
                case 6:                    /*channel 1 and 2 connected*/
                    if((pruicss_xchg->config[1].status & 1)&&(pruicss_xchg->config[2].status & 1))
                        init_complete = 1;
                    break;
                case 7:                       /*all three channel connected*/
                    if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[1].status & 1)&&( pruicss_xchg->config[2].status & 1))
                        init_complete = 1;
                    break;
            }
            if(init_complete == 0)
            {
                ClockP_usleep(priv->fw_wait_delay_us);
            }
        }
        else if(pruicss_xchg->config[0].status & 1)
        {
            init_complete = 1;
        }
        else
        {
            ClockP_usleep(priv->fw_wait_delay_us);
        }
    }

    if(init_complete == 0)
    {
        return SystemP_TIMEOUT;
    }
    /* Calculate propagation delay for all enabled channels */
    if(attrs->mode == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    {
       status = endat_calculate_propagation_delay(handle);
    }
    else
    {
        for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
        {
            if(attrs->channel_mask & (1 << i))
            {
                status = endat_multi_channel_set_cur(handle, i);
                if(status != SystemP_SUCCESS)
                {
                    return status;
                }
                status = endat_calculate_propagation_delay(handle);
                if(status != SystemP_SUCCESS)
                {
                    return status;
                }
            }
        }
    }

    return status;
}

static int32_t endat_config_clr_cfg0(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    void *pruicss_cfg;

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Clear configuration only for enabled channels based on channel mask */
    if(attrs->pruicss_slice)
    {
       if(attrs->channel_mask & (1U << 0))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG, 0);
       }
       if(attrs->channel_mask & (1U << 1))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG, 0);
       }
       if(attrs->channel_mask & (1U << 2))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG, 0);
       }
    }
    else
    {
       if(attrs->channel_mask & (1U << 0))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG, 0);
       }
       if(attrs->channel_mask & (1U << 1))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG, 0);
       }
       if(attrs->channel_mask & (1U << 2))
       {
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG, 0);
       }
    }

    return SystemP_SUCCESS;
}

static int32_t endat_config_endat_mode(endat_handle handle)
{
    int32_t status;
    endat_priv *priv;
    const endat_attrs *attrs;

    priv = handle->priv;
    attrs = handle->attrs;

    status = PRUICSS_setGpMuxSelect(priv->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);
    return status;
}

static int32_t endat_hw_init(endat_handle handle)
{
    endat_clk_cfg clk_cfg;
    const endat_attrs *attrs;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;

    /* Set initial clock to 200KHz */
    if(attrs->is_core_clk == 1)
    {
        clk_cfg.rx_div = attrs->core_clk_freq /(ENDAT_RX_OVERSAMPLING_RATE*(ENDAT_INIT_FREQ)) - 1;
    }
    else
    {
        clk_cfg.rx_div = attrs->uart_clk_freq /(ENDAT_RX_OVERSAMPLING_RATE*(ENDAT_INIT_FREQ)) - 1;
    }

    if(attrs->is_core_clk == 1)
    {
        clk_cfg.tx_div =attrs->core_clk_freq/ENDAT_INIT_FREQ - 1;
    }
    else
    {
        clk_cfg.tx_div = attrs->uart_clk_freq/ENDAT_INIT_FREQ - 1;
    }
    /* 2T */
    clk_cfg.rx_en_cnt = ENDAT_DELAY_COUNTER_INCREMENT*((2*attrs->core_clk_freq)/ENDAT_INIT_FREQ);
    /* sample size 8 */
    clk_cfg.rx_div_attr = ENDAT_RX_OVERSAMPLING_RATE - 1;

    /* Configure EnDAT mode */
    if(endat_config_endat_mode(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    /* Configure clock registers*/
    if(endat_config_clock_reg(handle, &clk_cfg) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    /* Configure global RX arm count */
    if(endat_config_global_rx_arm_cnt(handle, clk_cfg.rx_en_cnt) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    /* Enable load share mode if enabled */
    if(attrs->load_share_enabled)
    {
        if(endat_enable_load_share_mode(handle) != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
    }

    /* Clear configuration register 0 */
    if(endat_config_clr_cfg0(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

/**
 *  \brief      Set default initialization for EnDAT driver
 *
 *  \details    It sets up all default values and configures the firmware interface.
 *
 *  \param[in]  handle  EnDAT driver handle
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure
 */
static int32_t endat_set_default_initialization(endat_handle handle)
{
    uint32_t i = 0;
    endat_priv *priv;
    endat_pruicss_xchg *pruicss_xchg;

    priv = handle->priv;

    /* Initialize runtime state variables to defaults */
    priv->is_open = 0;
    priv->current_channel = 0;
    priv->pos_res = 0;
    priv->cmd_set_2_2 = 0;
    priv->raw_data = 0;
    priv->rx_en_cnt = 0;
    priv->endat_freq = ENDAT_INIT_FREQ;

    /* Initialize arrays to zero */
    for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
    {
        priv->multi_turn_res[i] = 0;
        priv->single_turn_res[i] = 0;
        priv->step[i] = 0;
        priv->pos_rx_bits_21_cmd[i] = 0;
        priv->pos_rx_bits_22_cmd[i] = 0;
        priv->type[i] = 0;
        priv->has_safety[i] = 0;
    }

    /* Initialize structures to zero */
    memset(&priv->flags, 0, sizeof(endat_flags));
    memset(&priv->id, 0, sizeof(endat_id));
    memset(&priv->sn, 0, sizeof(endat_sn));

    /* Configure PRU exchange interface */
    pruicss_xchg = priv->pruicss_xchg;

    for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
    {
        pruicss_xchg->config[i].channel = 1<<i;
        pruicss_xchg->config[i].trigger = 0;  /* Default to host trigger */
        pruicss_xchg->config[i].opmode = ENDAT_OPMODE_HOST_TRIGGER;   /* Default to host trigger mode */
        pruicss_xchg->config[i].status = 0;   /* Firmware will set this after init */
    }

    return SystemP_SUCCESS;
}

/**
 *  \brief      Configure timing delays for PRU firmware
 *
 *  \details    This function configures all timing delays used by the PRU firmware
 *              based on the core clock frequency from attrs.
 *
 *  \param[in]  handle  EnDAT driver handle
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure
 */
static int32_t endat_config_timing_delays(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    endat_pruicss_xchg *pruicss_xchg;

    priv = handle->priv;
    attrs = handle->attrs;

    pruicss_xchg = priv->pruicss_xchg;

    /* Configure timing delays based on PRU core clock frequency */
    pruicss_xchg->endat_delay_125ns = (uint32_t)(((float)attrs->core_clk_freq * 125.0f) / 1000000000.0f);
    pruicss_xchg->endat_delay_51us = (uint32_t)(((float)attrs->core_clk_freq * 51.0f) / 1000000.0f);
    pruicss_xchg->endat_delay_5us = (uint32_t)(((float)attrs->core_clk_freq * 5.0f) / 1000000.0f);
    pruicss_xchg->endat_delay_1ms = ((attrs->core_clk_freq / 1000) * 1);
    pruicss_xchg->endat_delay_2ms = ((attrs->core_clk_freq / 1000) * 2);
    pruicss_xchg->endat_delay_12ms = ((attrs->core_clk_freq / 1000) * 12);
    pruicss_xchg->endat_delay_50ms = ((attrs->core_clk_freq / 1000) * 50);
    pruicss_xchg->endat_delay_380ms = ((attrs->core_clk_freq / 1000) * 380);
    pruicss_xchg->endat_delay_900ms = ((attrs->core_clk_freq / 1000) * 900);
    pruicss_xchg->icss_clk = attrs->core_clk_freq;

    return SystemP_SUCCESS;
}

/**
 *  \brief      Configure IEP base address for periodic trigger mode
 *
 *  \details    This function sets up the IEP base address for
 *              periodic trigger mode operations.
 *
 *  \param[in]  handle  EnDAT driver handle
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure
 */
static int32_t endat_config_iep_base_addr(endat_handle handle)
{
    void *base_addr = NULL;
    endat_priv *priv;
    const endat_attrs *attrs;

    priv = handle->priv;
    attrs = handle->attrs;

    /* Calculate relative IEP address from PRUICSS base */
    base_addr = (void *)((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->baseAddr;
    priv->pruicss_xchg->endat_iep_base_addr = ((uint32_t)(attrs->iep_base_addr)) - ((uint32_t)base_addr);

    return SystemP_SUCCESS;
}

/**
 *  \brief      Configure channel info memory address in PRU exchange
 *
 *  \details    This function sets the global (SoC) address of the channel info
 *              structure.
 *
 *  \param[in]  handle               EnDAT driver handle
 *  \param[in]  ch_info_global_addr  Global (SoC) address of channel info structure
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure
 */
static int32_t endat_config_channel_info_addr(endat_handle handle, uint32_t ch_info_global_addr)
{
    endat_priv *priv;

    priv = handle->priv;

    /* Set channel info memory address */
    priv->pruicss_xchg->ch_info_memory_add = (uint64_t)ch_info_global_addr;

    return SystemP_SUCCESS;
}

int32_t endat_get_recovery_time(endat_handle handle, uint32_t *recovery_time)
{
    endat_priv *priv;
    const endat_attrs *attrs;

    /* Validate parameters and pointers used in this function */
    /* Validate core_clk_freq is not zero to avoid division by zero */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (recovery_time == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX) ||
       (handle->attrs->core_clk_freq == 0))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    /* Convert clock cycles to nanoseconds */
    *recovery_time = (uint32_t)(((float)priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.recovery_time * 1000000000.0f) / (float)attrs->core_clk_freq);

    return SystemP_SUCCESS;
}

int32_t endat_check_rt_error(endat_handle handle, int8_t *error_code)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    uint32_t rtDiff;
    uint32_t lastCounterValue;
    uint32_t currentCounterValue;

    /* Validate parameters and pointers used in this function */
    /* Validate core_clk_freq is not zero to avoid division by zero */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (error_code == NULL) ||
       (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX) ||
       (handle->attrs->core_clk_freq == 0))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    lastCounterValue =  priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.last_counter_value;
    currentCounterValue = priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.current_counter_value;

    /*Check if the counter is stuck by comparing current and last counter values*/
    if(currentCounterValue == lastCounterValue)
    {
       priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.is_counter_stuck = 1;
       *error_code = ENDAT_RT_COUNTER_STUCK_ERROR;
       return SystemP_SUCCESS;
    }
    else
    {
        priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.is_counter_stuck = 0;
    }

    /*handle the int overflow condition */
    if(lastCounterValue > currentCounterValue)
    {
        rtDiff = (ENDAT_MAX_RT_COUNTER_VALUE - lastCounterValue) + currentCounterValue;
    }
    else
    {
        rtDiff = currentCounterValue - lastCounterValue;
    }

    /* Convert clock cycles to nanoseconds */
    rtDiff = (uint32_t)(((float)rtDiff * 1000000000.0f) / (float)attrs->core_clk_freq);
    /* Check if the recovery time is within the short or long recovery time range */
    if ((rtDiff >= ENDAT_SHORT_RECOVERY_TIME_MIN) && (rtDiff <= ENDAT_SHORT_RECOVERY_TIME_MAX))
    {
        *error_code = ENDAT_RT_NO_ERROR;  /* Valid short recovery time */
        return SystemP_SUCCESS;
    }
    if ((rtDiff >= ENDAT_LONG_RECOVERY_TIME_MIN) && (rtDiff <= ENDAT_LONG_RECOVERY_TIME_MAX))
    {
        *error_code = ENDAT_RT_NO_ERROR;  /* Valid long recovery time */
        return SystemP_SUCCESS;
    }
    *error_code = ENDAT_RT_OUT_OF_RANGE_ERROR;  /*Error: out of expected range*/
    return SystemP_SUCCESS;
}

int32_t endat_init_rt_measurement(endat_handle handle)
{
    endat_priv *priv;
    const endat_attrs *attrs;
    uint32_t i;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->channel_rx_info == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
    {
        if(attrs->channel_mask & (1U << i))  /* Initialize only enabled channels */
        {
            priv->channel_rx_info->ch[i].recovery_time_parms.recovery_time = 0;
            priv->channel_rx_info->ch[i].recovery_time_parms.last_counter_value = 0;
            priv->channel_rx_info->ch[i].recovery_time_parms.starting_value = ENDAT_RT_COUNTER_STARTING_VALUE + ENDAT_RT_COUNTERS_STARTING_DIFFERENCE*(i); /* Assign a unique starting value for each channel; */
            priv->channel_rx_info->ch[i].recovery_time_parms.current_counter_value = priv->channel_rx_info->ch[i].recovery_time_parms.starting_value;
            priv->channel_rx_info->ch[i].recovery_time_parms.is_counter_stuck = 0;
        }
    }

    return SystemP_SUCCESS;
}

int32_t endat_enable_rt_measurement(endat_handle handle)
{
    endat_priv *priv;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) || (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->pruicss_xchg->ch[priv->current_channel].enable_rtm = 1;

    return SystemP_SUCCESS;
}

int32_t endat_disable_rt_measurement(endat_handle handle)
{
    endat_priv *priv;

    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) || (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    priv->pruicss_xchg->ch[priv->current_channel].enable_rtm = 0;

    return SystemP_SUCCESS;
}

int32_t endat_status_rt_measurement(endat_handle handle, uint32_t *status)
{
    endat_priv *priv;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->pruicss_xchg == NULL) || (status == NULL) || (handle->priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    *status = priv->pruicss_xchg->ch[priv->current_channel].enable_rtm;

    return SystemP_SUCCESS;
}

int32_t endat_config_iep_cap_event(endat_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t             ret_val = SystemP_SUCCESS;
    const endat_attrs   *attrs;
    endat_priv          *priv;
    endat_pruicss_xchg  *pruicss_xchg;
    uint8_t             ch_index = 0;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (event_num >= ENDAT_IEP_CAP_EVENT_MAX) ||
       (channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
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
    pruicss_xchg->trigger_params[ch_index].cap_event = event_num;
    pruicss_xchg->trigger_params[ch_index].iep_capture_reg =
        (uint32_t)pruicss_xchg->endat_iep_base_addr + ENDAT_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0  + ENDAT_8_BYTE_REG_OFFSET*(event_num);

    /* CAP6 and CAP7 have 2 extra registers for fall capture values, add extra offset */
    /* CAP6 and CAP7 has 2 register bits each. So bit 8 needs to be used for CAP7. Only capture rise bits for CAP6 and CAP7 are used. */
    if(event_num > 6)
    {
        pruicss_xchg->trigger_params[ch_index].cap_event += 1;
        pruicss_xchg->trigger_params[ch_index].iep_capture_reg += ENDAT_8_BYTE_REG_OFFSET;
    }
    return ret_val;
}

int32_t endat_config_iep_cmp_event(endat_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t             ret_val = SystemP_SUCCESS;
    const endat_attrs   *attrs;
    endat_priv          *priv;
    endat_pruicss_xchg  *pruicss_xchg;
    uint8_t             ch_index = 0;

    /* Validate parameters and pointers used in this function */
    if((handle == NULL) ||
      (handle->priv == NULL) ||
      (handle->attrs == NULL) ||
      (handle->priv->pruicss_xchg == NULL) ||
      (event_num >= ENDAT_IEP_CMP_EVENT_MAX) ||
      (channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
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

    /* write cmp event in DMEM */
    pruicss_xchg->trigger_params[ch_index].cmp_event = event_num;

    return ret_val;
}

static int32_t endat_calculate_propagation_delay(endat_handle handle)
{
    float delay_cycles;
    float endat_clock_period_cycles;

    /* Validate handle and pointers used in this function */
    /* Validate core_clk_freq is not zero to avoid division by zero */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->attrs->core_clk_freq == 0))
    {
        return SystemP_FAILURE;
    }

    /* Get accumulated propagation delay in PRU cycles */
    delay_cycles = handle->priv->pruicss_xchg->ch[handle->priv->current_channel].prop_delay;

    /* Calculate average of accumulated samples */
    delay_cycles = delay_cycles / (float)ENDAT_PROP_DELAY_NUM_SAMPLES;

    /* Calculate EnDAT clock period in PRU cycles */
    endat_clock_period_cycles = ((float)handle->attrs->core_clk_freq * (float)ENDAT_INIT_FREQ_CLOCK_PERIOD_NS) / (float)ENDAT_NS_PER_SECOND;

    /* Validate clock period is non-zero to prevent division by zero in fmodf */
    if(endat_clock_period_cycles <= 0.0f)
    {
        return SystemP_FAILURE;
    }

    /* Normalize delay to be less than one EnDAT clock period using fmodf */
    delay_cycles = fmodf(delay_cycles, endat_clock_period_cycles);

    /* Store calculated delay in PRU cycles with rounding to minimize truncation error */
    /* Adding 0.5 before truncation provides round-to-nearest behavior */
    handle->priv->pruicss_xchg->ch[handle->priv->current_channel].prop_delay = (uint32_t)(delay_cycles + 0.5f);

    return SystemP_SUCCESS;
}

const endat_attrs* endat_get_attrs(endat_handle handle)
{
    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->attrs == NULL))
    {
        return NULL;
    }

    return handle->attrs;
}

endat_priv* endat_get_priv(endat_handle handle)
{
    /* Validate handle and pointers used in this function */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return NULL;
    }

    return handle->priv;
}
