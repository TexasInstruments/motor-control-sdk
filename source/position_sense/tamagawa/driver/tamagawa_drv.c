/*
 *  Copyright (C) 2022-2026 Texas Instruments Incorporated
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

#include <position_sense/tamagawa/include/tamagawa_drv.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_icss.h>
#include <stdlib.h>

/* ========================================================================== */
/*                           External Declarations                            */
/* ========================================================================== */

extern uint32_t gTamagawaConfigNum;
extern tamagawa_config gTamagawaHandle[];

/* ========================================================================== */
/*                          Constant Definitions                              */
/* ========================================================================== */

const tamagawa_params gTamagawaDefaultParams = {
    NULL,                                   /* pruicss_handle */
    TAMAGAWA_DEFAULT_CMD_WAIT_DELAY_US,     /* cmd_wait_delay_us */
    TAMAGAWA_DEFAULT_MAX_WAIT_LOOP_COUNT,   /* max_wait_loop_count */
};

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* Internal functions */
static uint8_t tamagawa_crc(tamagawa_handle handle, uint8_t len, uint8_t ch);
static uint32_t tamagawa_reverse_bits(uint8_t data);
static uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data);
static void tamagawa_prepare_eeprom_command(tamagawa_handle handle, int32_t cmd, uint8_t ch);
static void tamagawa_eeprom_crc_reinit(tamagawa_handle handle);
static void tamagawa_config_clr_cfg0(tamagawa_handle handle);
static int32_t tamagawa_config_iep_base_address(tamagawa_handle handle, uint32_t iep_base_addr);

/* Load-share configuration functions */
static int32_t tamagawa_enable_load_share_mode(tamagawa_handle handle);
static int32_t tamagawa_config_primary_core_mask(tamagawa_handle handle, uint8_t mask);
static int32_t tamagawa_config_load_share(tamagawa_handle handle, uint8_t mask);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void tamagawa_params_init(tamagawa_params *params)
{
    if(params != NULL)
    {
        *params = gTamagawaDefaultParams;
    }
}

tamagawa_handle tamagawa_init(uint32_t index, const tamagawa_params *params)
{
    int32_t                 status = SystemP_SUCCESS;
    tamagawa_handle         handle = NULL;
    tamagawa_priv           *priv = NULL;
    const tamagawa_attrs    *attrs = NULL;
    uint8_t                 ch;
    uint32_t                iep_address;
    void                    *base_addr;

    if((index >= gTamagawaConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        handle = (tamagawa_handle)(&gTamagawaHandle[index]);

        /* Get the pointer to the priv and attrs */
        priv = handle->priv;
        attrs = handle->attrs;

        /* Input parameter validation */
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
           (params->max_wait_loop_count == 0))
        {
            status = SystemP_FAILURE;
        }

        /* Validate attrs */
        if((attrs->instance >= gTamagawaConfigNum) ||
           (attrs->mode > TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU) ||
           (attrs->pruicss_instance > 1) ||
           (attrs->pruicss_slice > 1) ||
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
           (attrs->load_share_enabled > 1) ||
           (attrs->iep_base_addr == NULL) ||
           (attrs->iep_instance > 1))
        {
            status = SystemP_FAILURE;
        }

        /* Validate IEP CMP and CAP event numbers for periodic trigger mode */
        if(status == SystemP_SUCCESS)
        {
            /* Validate IEP CMP event numbers and CAP event numbers */
            if(attrs->load_share_enabled)
            {
                for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
                {
                    if(attrs->channel_mask & (1U << ch))
                    {
                        if((attrs->iep_cmp_event[ch] >= TAMAGAWA_IEP_MAX_CMP_EVENT) ||
                           (attrs->iep_cap_event[ch] >= TAMAGAWA_IEP_MAX_CAP_EVENT))
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
                if((attrs->iep_cmp_event[0] >= TAMAGAWA_IEP_MAX_CMP_EVENT) ||
                   (attrs->iep_cap_event[0] >= TAMAGAWA_IEP_MAX_CAP_EVENT))
                {
                    status = SystemP_FAILURE;
                }
            }
        }

        /* Validate baud_rate - must be one of the supported Tamagawa frequencies */
        if((attrs->baud_rate != TAMAGAWA_FREQ_2_5_MHZ) &&
           (attrs->baud_rate != TAMAGAWA_FREQ_5_MHZ))
        {
            status = SystemP_FAILURE;
        }

        /* TODO: Validate core_clk_freq, uart_clk_freq and iep_clk_freq */
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set up PRU DRAM base address based on slice */
        if(attrs->pruicss_slice == 1)
        {
            priv->tamagawa_xchg = (tamagawa_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru1DramBase;
        }
        else
        {
            priv->tamagawa_xchg = (tamagawa_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru0DramBase;
        }

        /* Initialize priv structure */
        priv->pruicss_handle = params->pruicss_handle;
        priv->cmd_wait_delay_us = params->cmd_wait_delay_us;
        priv->max_wait_loop_count = params->max_wait_loop_count;

        /* Initialize sync offsets for load-share mode */
        if(attrs->mode == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
        {
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
            {
                /* Clear sync state offsets (RTU, PRU, TXPRU) */
                priv->tamagawa_xchg->execution_state[ch] = 0;
            }
        }

        /* Initialize clock and oversampling configuration */
        priv->clk_cfg.rx_clk_source = attrs->is_core_clk;
        priv->clk_cfg.tx_clk_source = attrs->is_core_clk;
        priv->clk_cfg.rx_os_rate = TAMAGAWA_RX_OVERSAMPLING_RATE;

        /* Clear CFG0 registers */
        tamagawa_config_clr_cfg0(handle);

        /* Configure channel mask */
        status = tamagawa_config_channel(handle, attrs->channel_mask);
    }
    /* Set the GP mux selection before any 3 channel interface registers configuration. */
    if(status == SystemP_SUCCESS)
    {
        status = PRUICSS_setGpMuxSelect(priv->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);
    }

    if(status == SystemP_SUCCESS)
    {
        /*Set IEP base address */
        base_addr = (void *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->baseAddr;
        iep_address = ((uint32_t)attrs->iep_base_addr) - ((uint32_t)base_addr);

        /* Initialize IEP base address in pruicss_xchg */
        status = tamagawa_config_iep_base_address(handle, iep_address);
    }

    /* Configure IEP CMP and CAP events for enabled channels */
    if(status == SystemP_SUCCESS)
    {
        if(attrs->load_share_enabled)
        {
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
            {
                /* Check if channel is enabled */
                if(attrs->channel_mask & (1U << ch))
                {
                    /* Configure IEP CMP event for this channel */
                    status = tamagawa_config_iep_cmp_event(handle, ch, attrs->iep_cmp_event[ch]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }

                    /* Configure IEP CAP event for this channel */
                    status = tamagawa_config_iep_cap_event(handle, ch, attrs->iep_cap_event[ch]);
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
            status = tamagawa_config_iep_cmp_event(handle, 0, attrs->iep_cmp_event[0]);

            if(status == SystemP_SUCCESS)
            {
                /* Configure IEP CAP event */
                status = tamagawa_config_iep_cap_event(handle, 0, attrs->iep_cap_event[0]);
            }
        }
    }

    if((status == SystemP_SUCCESS) && (attrs->mode == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU))
    {
        status = tamagawa_config_load_share(handle, attrs->channel_mask);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set baud rate from attrs configuration */
        status = tamagawa_set_baudrate(handle, attrs->baud_rate);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure default trigger mode to host trigger */
        status = tamagawa_config_host_trigger(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Mark handle as open */
        priv->is_open = 1;
    }

    if(status != SystemP_SUCCESS)
    {
        handle = NULL;
    }

    return handle;
}

void tamagawa_deinit(tamagawa_handle handle)
{
    /* Validate handle and priv structure pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return;
    }
    /* Mark handle as closed */
    handle->priv->is_open = 0;
}

const tamagawa_attrs* tamagawa_get_attrs(tamagawa_handle handle)
{
    /* Validate handle and attrs structure pointer */
    if((handle == NULL) || (handle->attrs == NULL))
    {
        return NULL;
    }
    return handle->attrs;
}

tamagawa_priv* tamagawa_get_priv(tamagawa_handle handle)
{
    /* Validate handle and priv structure pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return NULL;
    }
    return handle->priv;
}

int32_t tamagawa_parse(tamagawa_handle handle, int32_t cmd)
{
    uint32_t word0, word1, word2;
    uint8_t xchg_index;
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs *attrs;
    uint8_t ch;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Explicit validation of cmd parameter against valid DATA_ID range */
    if((cmd < 0) || (cmd > DATA_ID_D))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)
    {
        xchg_index = handle->priv->channel;
    }
    else
    {
        xchg_index = 0;
    }
    ch = handle->priv->channel;

    word0 = tamagawa_xchg_ptr->ch[ch].pos_word0;
    word1 = tamagawa_xchg_ptr->ch[ch].pos_word1;
    word2 = tamagawa_xchg_ptr->ch[ch].pos_word2;

    switch(cmd)
    {
        case DATA_ID_0:
            /* Data readout: data in one revolution */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abs = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 8) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word1 = word1 << 16;
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            /* Data frames: cf(8 bits) + sf(8 bits) + abm(3 frames with 8 bits data each) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abm = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 8) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word1 = word1 << 16;
            break;

        case DATA_ID_2:
            /* Data readout: encoder ID */
            /* Data frames: cf(8 bits) + sf(8 bits) + enid(8 bits) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.enid = (word0 >> 8) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word0) & 0xFF;
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + enid(8 bits) + abm(3 frames with 8 bits data each) + almc(8 bits) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abs = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 24) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.enid = (word1 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abm = ((word1 >> 8) & 0xFF) | ((word1) & 0xFF) << 8 | (((word2 >> 16) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.almc = (word2 >> 8) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word2) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word2 = word2 << 8;
            break;

        case DATA_ID_7:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abs = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 8) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word1 = word1 << 16;
            break;

        case DATA_ID_8:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abs = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 8) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word1 = word1 << 16;
            break;

        case DATA_ID_C:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.sf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.abs = ((word0 >> 8) & 0xFF) | ((word0) & 0xFF) << 8 | (((word1 >> 8) & 0xFF) << 16);
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg_ptr->ch[ch].pos_word1 = word1 << 16;
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Data frames: cf(1 frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.adf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.edf = (word0 >> 8) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word0) & 0xFF;
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Data frames: cf(1 frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.cf = (word0 >> 24) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.adf = (word0 >> 16) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.edf = (word0 >> 8) & 0xFF;
            handle->priv->tamagawa_interface[xchg_index].rx_frames_received.crc = (word0) & 0xFF;
            break;

        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

/**
 * \brief Calculate CRC for Tamagawa EEPROM interface data
 *
 * \details This internal function calculates the CRC for EEPROM commands (Read/Write)
 *          using the Tamagawa CRC polynomial (X^8+1).
 *
 * \param[in] handle  Tamagawa handle (validated by caller)
 * \param[in] len     Number of bytes to include in CRC calculation (2 or 3)
 * \param[in] ch      Channel number (0-2)
 *
 * \return Calculated 8-bit CRC value
 *
 * \note This is an internal function called only from \ref tamagawa_update_crc which
 *       validates all parameters. No NULL or bounds checking is performed here.
 */
static uint8_t tamagawa_crc(tamagawa_handle handle, uint8_t len, uint8_t ch)
{
    uint8_t crc = 0;
    uint8_t val;
    uint32_t i, j;
    uint32_t word0, word1, word2;
    uint8_t data[TAMAGAWA_CRC_DATA_ARRAY_SIZE];
    tamagawa_xchg *tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;

    word0 = tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word0;
    word1 = tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word1;
    word2 = tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word2;

    /* In EEPROM Write, we have CF, ADF and EDF fields as bits 24-31, 16-23 and 8-15 respectively */
    /* In EEPROM Read, we have CF and ADF fields as bits 24-31 and 16-23 respectively */
    /* Size of the array is 12 in order to use 12 8-bit integers for different frames */
    data[3] = word0 & 0xFF;
    data[2] = (word0 >> 8) & 0xFF;
    data[1] = (word0 >> 16) & 0xFF;
    data[0] = (word0 >> 24) & 0xFF;

    data[7] = word1 & 0xFF;
    data[6] = (word1 >> 8) & 0xFF;
    data[5] = (word1 >> 16) & 0xFF;
    data[4] = (word1 >> 24) & 0xFF;

    data[11] = word2 & 0xFF;
    data[10] = (word2 >> 8) & 0xFF;
    data[9] = (word2 >> 16) & 0xFF;
    data[8] = (word2 >> 24) & 0xFF;

    for(i = 0; i < len; i++)
    {
        for(j = 0; j < TAMAGAWA_BITS_PER_BYTE; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);
            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }
    }

    return crc;
}

int32_t tamagawa_update_crc(tamagawa_handle handle, int32_t cmd, uint8_t ch)
{
    uint32_t word0;
    tamagawa_xchg *tamagawa_xchg_ptr;

    /* Validate handle and internal structure pointers, channel bounds check */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->tamagawa_xchg == NULL) || (ch >= TAMAGAWA_MAX_CHANNELS_PER_SLICE))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    tamagawa_eeprom_crc_reinit(handle);
    word0 = tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word0;

    if(cmd == DATA_ID_6)
    {
        /* Bitwise OR the 32-bit integer with the CF, ADF and EDF values left shifted 24, 16 and 8 times respectively */
        word0 = (word0 | TAMAGAWA_CF_EEPROM_WRITE) << 24;
        word0 = word0 | (tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].adf << 16);
        word0 = word0 | (tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].edf << 8);
        tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(handle, TAMAGAWA_EEPROM_WRITE_CRC_BYTES, ch);
    }
    else
    {
        /* Bitwise OR the 32-bit integer with the CF and ADF values left shifted 24 and 16 times respectively */
        word0 = (word0 | TAMAGAWA_CF_EEPROM_READ) << 24;
        word0 = word0 | (tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].adf << 16);
        tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(handle, TAMAGAWA_EEPROM_READ_CRC_BYTES, ch);
    }

    tamagawa_eeprom_crc_reinit(handle);

    return SystemP_SUCCESS;
}

int32_t tamagawa_crc_verify(tamagawa_handle handle)
{
    uint32_t word0;
    uint8_t ch;
    tamagawa_xchg *tamagawa_xchg_ptr;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    ch = handle->priv->channel;
    word0 = tamagawa_xchg_ptr->ch[ch].cal_crc;

    if((word0 & 0xFF) == 1)
    {
        return SystemP_SUCCESS;
    }
    else
    {
        return SystemP_FAILURE;
    }
}

int32_t tamagawa_set_baudrate(tamagawa_handle handle, double baud_rate)
{
    uint16_t rx_div;
    uint16_t tx_div;
    tamagawa_clk_cfg clk_cfg;
    int32_t ret;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Validate baud_rate - must be one of the supported Tamagawa frequencies */
    if((baud_rate != TAMAGAWA_FREQ_2_5_MHZ) &&
       (baud_rate != TAMAGAWA_FREQ_5_MHZ))
    {
        return SystemP_FAILURE;
    }

    clk_cfg = handle->priv->clk_cfg;

    if(clk_cfg.rx_clk_source == 1)
    {
        rx_div = handle->attrs->core_clk_freq / ((clk_cfg.rx_os_rate + 1) * (baud_rate));
    }
    else
    {
        rx_div = handle->attrs->uart_clk_freq / ((clk_cfg.rx_os_rate + 1) * (baud_rate));
    }

    if(clk_cfg.tx_clk_source == 1)
    {
        tx_div = handle->attrs->core_clk_freq / baud_rate;
    }
    else
    {
        tx_div = handle->attrs->uart_clk_freq / baud_rate;
    }

    clk_cfg.tx_div = tx_div - 1;
    clk_cfg.rx_div = rx_div - 1;

    /* Configure RX auto arm counter for 1us delay */
    clk_cfg.rx_en_cnt = TAMAGAWA_DELAY_COUNTER_INCREMENT * (handle->attrs->core_clk_freq / 1000000);

    ret = tamagawa_config_clock(handle, &clk_cfg);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    ret = tamagawa_config_global_rx_arm_cnt(handle, clk_cfg.rx_en_cnt);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_command_build(tamagawa_handle handle, int32_t cmd)
{
    uint8_t ch;
    uint8_t xchg_index;
    const tamagawa_attrs *attrs;
    tamagawa_xchg *tamagawa_xchg_ptr;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Explicit validation of cmd parameter against valid DATA_ID range */
    if((cmd < 0) || (cmd > DATA_ID_D))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;

    /* First clear command parameters to be safe */
    memset(&tamagawa_xchg_ptr->cmd, 0, sizeof(tamagawa_xchg_ptr->cmd));

    switch(cmd)
    {
         case DATA_ID_0:
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Data readout: data in one revolution */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x20) | (0x40 << 8);
                        /* Number of expected Rx frames is 6 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 6;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (6 << 8);
                    }
                }
            }
            else
            {
                /* Data readout: data in one revolution */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x20) | (0x40 << 8);
                /* Number of expected Rx frames is 6 */
                handle->priv->tamagawa_interface[0].rx_frames = 6;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (6 << 8);
            }
            break;

        case DATA_ID_1:
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Data readout: multi-turn data */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x28) | (0xC0 << 8);
                        /* Number of expected Rx frames is 6 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 6;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (6 << 8);
                    }
                }
            }
            else
            {
                /* Data readout: multi-turn data */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x28) | (0xC0 << 8);
                /* Number of expected Rx frames is 6 */
                handle->priv->tamagawa_interface[0].rx_frames = 6;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (6 << 8);
            }
            break;

        case DATA_ID_2:
            if(attrs->load_share_enabled)
            {

                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Data readout: encoder ID */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x24) | (0xC0 << 8);
                        /* Number of expected Rx frames is 4 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 4;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 4 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (4 << 8);
                    }
                }
            }
            else
            {
                /* Data readout: encoder ID */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x24) | (0xC0 << 8);
                /* Number of expected Rx frames is 4 */
                handle->priv->tamagawa_interface[0].rx_frames = 4;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 4 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (4 << 8);
            }
            break;

        case DATA_ID_3:
            if(attrs->load_share_enabled)
            {

                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x2C) | (0x40 << 8);
                        /* Number of expected Rx frames is 11 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 11;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 11 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (0xB << 8);
                    }
                }
            }
            else
            {
                /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x2C) | (0x40 << 8);
                /* Number of expected Rx frames is 11 */
                handle->priv->tamagawa_interface[0].rx_frames = 11;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 11 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (0xB << 8);
            }
            break;

        case DATA_ID_7:
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Reset */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x2E) | (0xC0 << 8);
                        /* Number of expected Rx frames is 6 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 6;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (6 << 8);
                    }
                }
            }
            else
            {
                /* Reset */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x2E) | (0xC0 << 8);
                /* Number of expected Rx frames is 6 */
                handle->priv->tamagawa_interface[0].rx_frames = 6;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (6 << 8);
            }
            break;

        case DATA_ID_8:
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Reset */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x21) | (0xC0 << 8);
                        /* Number of expected Rx frames is 6 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 6;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (6 << 8);
                    }
                }
            }
            else
            {
                /* Reset */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x21) | (0xC0 << 8);
                /* Number of expected Rx frames is 6 */
                handle->priv->tamagawa_interface[0].rx_frames = 6;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (6 << 8);
            }
            break;

        case DATA_ID_C:
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Reset */
                        /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                        tamagawa_xchg_ptr->cmd[xchg_index].word0 = (0x23) | (0x40 << 8);
                        /* Number of expected Rx frames is 6 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 6;
                        /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (1) | (6 << 8);
                    }
                }
            }
            else
            {
                /* Reset */
                /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
                tamagawa_xchg_ptr->cmd[0].word0 = (0x23) | (0x40 << 8);
                /* Number of expected Rx frames is 6 */
                handle->priv->tamagawa_interface[0].rx_frames = 6;
                /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
                tamagawa_xchg_ptr->cmd[0].word1 = (1) | (6 << 8);
            }
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF and CRC data */
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
            {
                if(attrs->channel_mask & (1 << ch))
                {
                    tamagawa_prepare_eeprom_command(handle, cmd, ch);
                }
            }
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Number of expected Rx frames is 4 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 4;
                        /* Number of Tx frames being sent to the encoder is 3, and the number of Rx frames to be received is 4 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (3) | (4 << 8);
                    }
                }
            }
            else
            {
                /* Number of expected Rx frames is 4 */
                handle->priv->tamagawa_interface[0].rx_frames = 4;
                /* Number of Tx frames being sent to the encoder is 3, and the number of Rx frames to be received is 4 */
                tamagawa_xchg_ptr->cmd[0].word1 = (3) | (4 << 8);
            }
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF, EDF and CRC data */
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
            {
                if(attrs->channel_mask & (1 << ch))
                {
                    tamagawa_prepare_eeprom_command(handle, cmd, ch);
                }
            }
            if(attrs->load_share_enabled)
            {
                for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
                {
                    if(handle->attrs->channel_mask & (1 << xchg_index))
                    {
                        /* Number of expected Rx frames is 4 */
                        handle->priv->tamagawa_interface[xchg_index].rx_frames = 4;
                        /* Number of Tx frames being sent to the encoder is 4, and the number of Rx frames to be received is 4 */
                        tamagawa_xchg_ptr->cmd[xchg_index].word1 = (4) | (4 << 8);
                    }
                }
            }
            else
            {
                /* Number of expected Rx frames is 4 */
                handle->priv->tamagawa_interface[0].rx_frames = 4;
                /* Number of Tx frames being sent to the encoder is 4, and the number of Rx frames to be received is 4 */
                tamagawa_xchg_ptr->cmd[0].word1 = (4) | (4 << 8);
            }
            break;

        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_command_send(tamagawa_handle handle)
{
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs    *attrs = NULL;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    attrs = handle->attrs;
    if(attrs->load_share_enabled)
    {
        tamagawa_xchg_ptr->config[0].trigger = (attrs->channel0_enabled)?TAMAGAWA_ENABLE_CYCLE_TRIGGER:TAMAGAWA_DISABLE_CYCLE_TRIGGER;
        tamagawa_xchg_ptr->config[1].trigger = (attrs->channel1_enabled)?TAMAGAWA_ENABLE_CYCLE_TRIGGER:TAMAGAWA_DISABLE_CYCLE_TRIGGER;
        tamagawa_xchg_ptr->config[2].trigger = (attrs->channel2_enabled)?TAMAGAWA_ENABLE_CYCLE_TRIGGER:TAMAGAWA_DISABLE_CYCLE_TRIGGER;
    }
    else
    {
        tamagawa_xchg_ptr->config[0].trigger = TAMAGAWA_ENABLE_CYCLE_TRIGGER;
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_command_wait(tamagawa_handle handle)
{
    tamagawa_xchg *tamagawa_xchg_ptr;
    tamagawa_priv *priv;
    uint32_t loop_count;
    const tamagawa_attrs *attrs;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    tamagawa_xchg_ptr = priv->tamagawa_xchg;
    loop_count = priv->max_wait_loop_count;
    attrs = handle->attrs;

    /* Handle zero loop count case - would cause infinite loop */
    if(loop_count == 0)
    {
        return SystemP_FAILURE;
    }

    /* Wait until the trigger value is cleared by firmware with timeout */
    while(1)
    {

        if(attrs->load_share_enabled)
        {
            if((tamagawa_xchg_ptr->config[0].trigger == TAMAGAWA_DISABLE_CYCLE_TRIGGER) &&
            (tamagawa_xchg_ptr->config[1].trigger == TAMAGAWA_DISABLE_CYCLE_TRIGGER) &&
            (tamagawa_xchg_ptr->config[2].trigger == TAMAGAWA_DISABLE_CYCLE_TRIGGER))
            {
                break;
            }
        }
        else if(tamagawa_xchg_ptr->config[0].trigger == TAMAGAWA_DISABLE_CYCLE_TRIGGER)
        {
            break;
        }

        /* Add delay and decrement loop counter */
        ClockP_usleep(priv->cmd_wait_delay_us);

        loop_count--;
        if(loop_count == 0)
        {
            return SystemP_TIMEOUT;
        }
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_command_process(tamagawa_handle handle, int32_t cmd)
{
    int32_t ret = SystemP_SUCCESS;
    uint8_t xchg_index;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Build command in PRU interface buffer */
    ret = tamagawa_command_build(handle, cmd);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    /* Trigger sending the command in PRU */
    ret = tamagawa_command_send(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    /* Wait until PRU finishes transaction */
    ret = tamagawa_command_wait(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    /* In case of EEPROM commands, reset the command ID for all channels back to 0 */
    if(cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[xchg_index].cmd = 0;
            }
        }
    }

    return SystemP_SUCCESS;
}

/**
 * \brief Reverse the bit order of an 8-bit value
 *
 * \details This internal function reverses the bit order of an 8-bit data value.
 *          It is used to prepare data for transmission over the Tamagawa interface,
 *          which requires bit-reversed data format.
 *
 * \param[in] data  8-bit value to be bit-reversed
 *
 * \return 32-bit value containing the bit-reversed result in lower 8 bits
 *
 * \note This is an internal function called from \ref tamagawa_prepare_eeprom_tx_data.
 *       The function returns uint32_t to match the calling context, but only the
 *       lower 8 bits contain meaningful data.
 */
static uint32_t tamagawa_reverse_bits(uint8_t data)
{
    uint32_t number_of_bits = sizeof(data) * TAMAGAWA_BITS_PER_BYTE;
    uint32_t reversed_num = 0;
    uint32_t current_bit_pos;

    for(current_bit_pos = 0; current_bit_pos < number_of_bits; current_bit_pos++)
    {
        if((data & (1 << current_bit_pos)))
        {
            reversed_num |= 1 << ((number_of_bits - 1) - current_bit_pos);
        }
    }

    return reversed_num;
}

/**
 * \brief Prepares EEPROM TX data by adding start/stop bits and reversing data
 *
 * \details This function performs returns modified eeprom_tx_data by:
 *          1. Left shifting the existing eeprom_tx_data by 9 bits
 *          2. Adding reversed 8-bit data
 *          3. Left shifting by 1 bit and adding stop bit
 *
 *          IMPORTANT: This function modifies eeprom_tx_data by building
 *          up the complete TX frame incrementally. The caller must pass the
 *          returned value back in subsequent calls to build multi-byte frames.
 *
 * \param[in] eeprom_tx_data  Current accumulated TX data
 * \param[in] data            8-bit data to be added (CF, ADF, EDF, or CRC)
 *
 * \return Modified eeprom_tx_data with new data appended
 */
static uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data)
{
    /* Takes 8-bit data of CF, ADF, EDF and CRC, reverses it and adds start and stop bits */
    uint32_t reversed_num = tamagawa_reverse_bits(data);

    /* Left shift and add start bit with reversed data */
    eeprom_tx_data = eeprom_tx_data << 9;
    eeprom_tx_data = eeprom_tx_data | reversed_num;

    /* Left shift and add stop bit */
    eeprom_tx_data = eeprom_tx_data << 1;
    eeprom_tx_data = eeprom_tx_data | 1;

    return eeprom_tx_data;
}

static void tamagawa_prepare_eeprom_command(tamagawa_handle handle, int32_t cmd, uint8_t ch)
{
    uint64_t eeprom_tx_data = 0;
    tamagawa_xchg *tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;

    if(cmd == DATA_ID_6)
    {
        /* Command preparation for EEPROM Write
         * Note: Each call to tamagawa_prepare_eeprom_tx_data() modifies eeprom_tx_data
         * by appending the new data. The returned value must be passed back
         * in subsequent calls to build the complete multi-byte frame: CF + ADF + EDF + CRC */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, TAMAGAWA_CF_EEPROM_WRITE);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].edf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].crc);
    }
    else
    {
        /* Command preparation for EEPROM Read
         * Note: Each call to tamagawa_prepare_eeprom_tx_data() modifies eeprom_tx_data
         * by appending the new data. The returned value must be passed back
         * in subsequent calls to build the complete multi-byte frame: CF + ADF + CRC */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, TAMAGAWA_CF_EEPROM_READ);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].crc);
        /* Left shift twice to load the data byte-wise into TX FIFO correctly */
        eeprom_tx_data <<= 2;
    }

    /* Assign the prepared TX data for EEPROM commands to the interface */
    tamagawa_xchg_ptr->tamagawa_eeprom_interface[ch].eeprom_tx_data = eeprom_tx_data;
}

int32_t tamagawa_config_clock(tamagawa_handle handle, tamagawa_clk_cfg *clk_cfg)
{
    void *pruicss_cfg;
    const tamagawa_attrs *attrs;
    uint32_t rx_reg_val;
    uint32_t tx_reg_val;

    /* Validate handle and internal structure pointers, clk_cfg parameter */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (clk_cfg == NULL))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(handle->priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Configure RX and TX CFG registers based on PRU slice */
    if(attrs->pruicss_slice)
    {
        /* Slice 1 */
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
                      (clk_cfg->rx_clk_source << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_SHIFT) |
                      (clk_cfg->rx_os_rate << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_SAMPLE_SIZE_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG, rx_reg_val);

        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_FRAC_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->tx_clk_source << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, tx_reg_val);
    }
    else
    {
        /* Slice 0 */
        rx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG);
        rx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_FRAC_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SB_POL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SAMPLE_SIZE_MASK);
        rx_reg_val |= (clk_cfg->rx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->rx_clk_source << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_SHIFT) |
                      (clk_cfg->rx_os_rate << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_SAMPLE_SIZE_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG, rx_reg_val);

        tx_reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        tx_reg_val &= ~(CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_MASK |
                        CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_FRAC_MASK);
        tx_reg_val |= (clk_cfg->tx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_SHIFT) |
                      (clk_cfg->tx_clk_source << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_SHIFT);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, tx_reg_val);
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_config_global_rx_arm_cnt(tamagawa_handle handle, uint16_t rx_en_cnt)
{
    void *pruicss_cfg;
    const tamagawa_attrs *attrs;

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
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(handle->priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Write RX Global Auto Arm Counter for all channels based on PRU slice */
    if(attrs->pruicss_slice)
    {
        /* Slice 1 */
        if(attrs->channel0_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + 2, rx_en_cnt);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG1_REG + 2, rx_en_cnt);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG1_REG + 2, rx_en_cnt);
        }
    }
    else
    {
        /* Slice 0 */
        if(attrs->channel0_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + 2, rx_en_cnt);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG1_REG + 2, rx_en_cnt);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG1_REG + 2, rx_en_cnt);
        }
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_config_host_trigger(tamagawa_handle handle)
{
    uint8_t xchg_index;
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs    *attrs = NULL;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                tamagawa_xchg_ptr->config[xchg_index].opmode = TAMAGAWA_OPMODE_HOST_TRIGGER;
            }
        }
    }
    else
    {
        tamagawa_xchg_ptr->config[0].opmode = TAMAGAWA_OPMODE_HOST_TRIGGER;
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_config_periodic_trigger_cmp_mode(tamagawa_handle handle)
{
    uint8_t xchg_index;
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs    *attrs = NULL;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                tamagawa_xchg_ptr->config[xchg_index].opmode = TAMAGAWA_OPMODE_PERIODIC_CMP;
            }
        }
    }
    else
    {
        tamagawa_xchg_ptr->config[0].opmode = TAMAGAWA_OPMODE_PERIODIC_CMP;
    }

    return SystemP_SUCCESS;
}

/**
 *  \brief      Configure IEP base address for periodic trigger
 *
 *  \details    This function writes the IEP base address to DMEM (PRU Data Memory) so that
 *              the firmware can access IEP registers for periodic trigger mode operation.
 *              The address is stored as an offset from the PRU-ICSS base address.
 *
 *              This function is called during \ref tamagawa_init.
 *
 *  \param[in]  handle          Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  iep_base_addr   IEP base address as offset from PRU-ICSS base address
 *
 *  \retval     SystemP_SUCCESS  IEP base address configured successfully
 *  \retval     SystemP_FAILURE  On zero iep_base_addr
 *
 *  \note       This function writes to the tamagawa_xchg structure in DMEM.
 */
static int32_t tamagawa_config_iep_base_address(tamagawa_handle handle, uint32_t iep_base_addr)
{
    tamagawa_xchg *tamagawa_xchg_ptr;

    if(iep_base_addr == 0)
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    tamagawa_xchg_ptr->iep_base_addr = iep_base_addr;

    return SystemP_SUCCESS;
}

int32_t tamagawa_config_periodic_trigger_cap_mode(tamagawa_handle handle)
{
    uint8_t xchg_index;
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs *attrs = NULL;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                tamagawa_xchg_ptr->config[xchg_index].opmode = TAMAGAWA_OPMODE_PERIODIC_CAP;
            }
        }
    }
    else
    {
        tamagawa_xchg_ptr->config[0].opmode = TAMAGAWA_OPMODE_PERIODIC_CAP;
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_config_channel(tamagawa_handle handle, uint8_t mask)
{
    uint8_t xchg_index;
    tamagawa_xchg *tamagawa_xchg_ptr;
    const tamagawa_attrs *attrs;

    /* Validate handle and internal structure pointers, mask bounds check */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) ||
       (handle->priv->tamagawa_xchg == NULL) || (mask == 0) || (mask > 0x07))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    tamagawa_xchg_ptr = handle->priv->tamagawa_xchg;

    if(attrs->load_share_enabled)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                tamagawa_xchg_ptr->config[xchg_index].channel = mask;
            }
        }
    }
    else
    {
        tamagawa_xchg_ptr->config[0].channel = mask;
    }

    /* For single-channel mode, store the specific channel index in priv->channel */
    if(attrs->total_channels == 1)
    {
        if(attrs->channel0_enabled)
        {
            handle->priv->channel = 0;
        }
        else if(attrs->channel1_enabled)
        {
            handle->priv->channel = 1;
        }
        else if(attrs->channel2_enabled)
        {
            handle->priv->channel = 2;
        }
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_update_data_id(tamagawa_handle handle, int32_t cmd)
{
    uint8_t xchg_index;
    const tamagawa_attrs *attrs;
    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->tamagawa_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    if(attrs->load_share_enabled)
    {
        for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
        {
            if(handle->attrs->channel_mask & (1 << xchg_index))
            {
                handle->priv->tamagawa_interface[xchg_index].data_id = cmd;
            }
        }
    }
    else
    {
        handle->priv->tamagawa_interface[0].data_id = cmd;
    }

    if(cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        if(attrs->load_share_enabled)
        {
            for(xchg_index = 0; xchg_index < TAMAGAWA_MAX_CHANNELS_PER_SLICE; xchg_index++)
            {
                if(handle->attrs->channel_mask & (1 << xchg_index))
                {
                    handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[xchg_index].cmd = cmd;
                }
            }
        }
        else
        {
            handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[0].cmd = cmd;
        }
    }

    return SystemP_SUCCESS;
}

int32_t tamagawa_update_adf(tamagawa_handle handle, uint32_t val, uint8_t ch)
{
    /* Validate handle and internal structure pointers, channel and ADF value bounds check */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->priv->tamagawa_xchg == NULL) ||
       (ch >= TAMAGAWA_MAX_CHANNELS_PER_SLICE) ||
       (val > TAMAGAWA_MAX_EEPROM_ADDRESS))
    {
        return SystemP_FAILURE;
    }

    handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].adf = val;

    return SystemP_SUCCESS;
}

int32_t tamagawa_update_edf(tamagawa_handle handle, uint32_t val, uint8_t ch)
{
    /* Validate handle and internal structure pointers, channel and EDF value bounds check */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->priv->tamagawa_xchg == NULL) ||
       (ch >= TAMAGAWA_MAX_CHANNELS_PER_SLICE) ||
       (val > TAMAGAWA_MAX_EEPROM_WRITE_DATA))
    {
        return SystemP_FAILURE;
    }

    handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].edf = val;

    return SystemP_SUCCESS;
}

/**
 * \brief Reset EEPROM CRC calculation workspace for all channels
 *
 * \details This internal function resets the word0, word1, and word2 fields in the
 *          EEPROM interface structure to zero for all channels. These fields are used
 *          as workspace during CRC calculation in \ref tamagawa_crc.
 *
 * \param[in] handle  Tamagawa handle (validated by caller)
 *
 * \note This is an internal function called from \ref tamagawa_update_crc before
 *       and after CRC calculation. No validation is performed as the caller
 *       (\ref tamagawa_update_crc) has already validated the handle.
 */
static void tamagawa_eeprom_crc_reinit(tamagawa_handle handle)
{
    uint8_t ch;

    for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
    {
        if(handle->attrs->channel_mask & (1 << ch))
        {
            handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = 0;
            handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word1 = 0;
            handle->priv->tamagawa_xchg->tamagawa_eeprom_interface[ch].word2 = 0;
        }
    }
}

int32_t tamagawa_multi_channel_set_cur(tamagawa_handle handle, uint8_t ch)
{
    /* Validate handle and internal structure pointers, channel bounds check */
    if((handle == NULL) || (handle->priv == NULL) || (ch >= TAMAGAWA_MAX_CHANNELS_PER_SLICE))
    {
        return SystemP_FAILURE;
    }

    handle->priv->channel = ch;

    return SystemP_SUCCESS;
}

/**
 * \brief Clear PRU Three Channel Peripheral Interface CFG0 registers for enabled Tamagawa channels
 *
 * \details This internal function clears the PRU Three Channel Peripheral Interface Channel CFG0
 *          registers for all enabled Tamagawa channels. Clearing these registers ensures a clean
 *          hardware state before initializing the Tamagawa encoder interface.
 *
 *          Purpose:
 *          - Called during \ref tamagawa_init to reset hardware configuration
 *          - Clears only the enabled channels (based on attrs configuration)
 *          - Handles both PRU slice 0 and slice 1 configurations
 *
 *          Hardware registers cleared:
 *          - PRU0_ED_CH0_CFG0_REG / PRU1_ED_CH0_CFG0_REG (if channel 0 enabled)
 *          - PRU0_ED_CH1_CFG0_REG / PRU1_ED_CH1_CFG0_REG (if channel 1 enabled)
 *          - PRU0_ED_CH2_CFG0_REG / PRU1_ED_CH2_CFG0_REG (if channel 2 enabled)
 *
 * \param[in] handle  Tamagawa handle
 *
 * \note This is an internal function called only from \ref tamagawa_init.
 */
static void tamagawa_config_clr_cfg0(tamagawa_handle handle)
{
    void *pruicss_cfg;
    const tamagawa_attrs *attrs;

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(handle->priv->pruicss_handle->hwAttrs))->cfgRegBase);
    attrs = handle->attrs;

    /* Clear CFG0 registers only for enabled channels based on PRU slice */
    if(attrs->pruicss_slice)
    {
        /* Slice 1 */
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
        /* Slice 0 */
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

int32_t tamagawa_config_iep_cmp_event(tamagawa_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    const tamagawa_attrs *attrs;
    tamagawa_priv *priv;
    tamagawa_xchg *tamagawa_xchg_ptr;
    uint8_t ch_index = 0;

    /* Validate handle and internal structure pointers, event and channel bounds check */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) ||
       (handle->priv->tamagawa_xchg == NULL) || (event_num >= TAMAGAWA_IEP_MAX_CMP_EVENT) ||
       (channel >= TAMAGAWA_MAX_CHANNELS_PER_SLICE))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    priv = handle->priv;
    tamagawa_xchg_ptr = priv->tamagawa_xchg;

    /* Determine channel index for DMEM access */
    if(attrs->load_share_enabled)
    {
        ch_index = channel;
    }
    else
    {
        /* Always 0 in single PRU mode. When load share mode is disabled.
         * In single PRU mode firmware, the channel number is ignored and the firmware
         * always reads data from DMEM using the channel 0 offset, regardless of which
         * channels are connected. */
        ch_index = 0;
    }

    /* Write IEP CMP event number to DMEM for PRU firmware access */
    tamagawa_xchg_ptr->trigger_params[ch_index].iep_cmp_event = event_num;

    return ret_val;
}

int32_t tamagawa_config_iep_cap_event(tamagawa_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    const tamagawa_attrs *attrs;
    tamagawa_priv *priv;
    tamagawa_xchg *tamagawa_xchg_ptr;
    uint8_t ch_index = 0;

    /* Validate handle and internal structure pointers, event and channel bounds check */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) ||
       (handle->priv->tamagawa_xchg == NULL) || (event_num >= TAMAGAWA_IEP_MAX_CAP_EVENT) ||
       (channel >= TAMAGAWA_MAX_CHANNELS_PER_SLICE))
    {
        return SystemP_FAILURE;
    }

    attrs = handle->attrs;
    priv = handle->priv;
    tamagawa_xchg_ptr = priv->tamagawa_xchg;

    if(attrs->load_share_enabled)
    {
        ch_index = channel;
    }
    else
    {
        /* Always 0 in single PRU mode. When load share mode is disabled.
         * In single PRU mode firmware, the channel number is ignored and the firmware
         * always reads data from DMEM using the channel 0 offset, regardless of which
         * channels are connected. */
        ch_index = 0;
    }

    /* Write cap event and capture register address in DMEM */
    tamagawa_xchg_ptr->trigger_params[ch_index].iep_cap_event = event_num;
    tamagawa_xchg_ptr->trigger_params[ch_index].iep_capture_reg = tamagawa_xchg_ptr->iep_base_addr + TAMAGAWA_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0  + TAMAGAWA_8_BYTE_REG_OFFSET*(event_num);

    /* CAP6 and CAP7 have 2 extra registers for fall capture values, add extra offset */
    /* CAP6 and CAP7 has 2 register bits each. So bit 8 needs to be used for CAP7. Only capture rise bits for CAP6 and CAP7 are used. */
    if(event_num > 6)
    {
        tamagawa_xchg_ptr->trigger_params[ch_index].iep_cap_event += 1;
        tamagawa_xchg_ptr->trigger_params[ch_index].iep_capture_reg += TAMAGAWA_8_BYTE_REG_OFFSET;
    }
    return ret_val;
}

/**
 * \brief Enable load-share mode for Tamagawa encoder
 *
 * \details This internal function enables the load-share mode by setting the ENDAT_SHARE_EN
 *          register bit based on the PRU slice configuration. This allows multiple PRU cores
 *          to coordinate and share encoder channel processing with synchronized global reinit.
 *
 * \param[in] handle  Tamagawa handle
 *
 * \return SystemP_SUCCESS on success
 *
 */
static int32_t tamagawa_enable_load_share_mode(tamagawa_handle handle)
{
    tamagawa_priv           *priv;
    const tamagawa_attrs    *attrs;
    void                    *pruicss_cfg;
    uint32_t                reg_val;

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

    return SystemP_SUCCESS;
}

/**
 * \brief Configure primary core mask for load-share mode
 *
 * \details This internal function sets the primary core mask in shared memory based on the
 *          channel mask configuration. The primary core is responsible for executing
 *          global reinit operations that affect all PRU cores.
 *
 * \param[in] handle  Tamagawa handle
 * \param[in] mask    Channel mask indicating enabled channels
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 *
 */
static int32_t tamagawa_config_primary_core_mask(tamagawa_handle handle, uint8_t mask)
{
    tamagawa_priv *priv;

    /* Validate handle parameter */
    if((mask == 0) || (mask > 7))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    switch (mask)
    {
        case 1: /*only channel0 connected*/
            priv->tamagawa_xchg->primary_core_mask = 0x1;
            break;
        case 2: /*channel1 connected*/
            priv->tamagawa_xchg->primary_core_mask = 0x2;
            break;
        case 3: /*channel0 and channel1 connected*/
            priv->tamagawa_xchg->primary_core_mask = 0x1;
            break;
        case 4: /*channel2 connected*/
            priv->tamagawa_xchg->primary_core_mask = 0x4;
            break;
        case 5: /*channel0 and channel2 connnected*/
            priv->tamagawa_xchg->primary_core_mask = 0x4;
            break;
        case 6: /*channel1 and channel2 connected*/
            priv->tamagawa_xchg->primary_core_mask = 0X4;
            break;
        case 7: /*all three channel connected*/
            priv->tamagawa_xchg->primary_core_mask = 0x4;
            break;
    }
    return SystemP_SUCCESS;
}

/**
 * \brief Configure load-share mode for Tamagawa encoder
 *
 * \details This internal function configures the load-share mode by setting up the
 *          primary core mask and enabling load-share hardware. This enables multiple
 *          PRU cores to coordinate encoder processing with synchronized operations.
 *
 * \param[in] handle  Tamagawa handle
 * \param[in] mask    Channel mask indicating enabled channels
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 *
 * \note This is an internal function called from \ref tamagawa_init when
 *       attrs->mode == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU.
 */
static int32_t tamagawa_config_load_share(tamagawa_handle handle, uint8_t mask)
{
    /* Validate handle parameter */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(tamagawa_config_primary_core_mask(handle, mask) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    if(tamagawa_enable_load_share_mode(handle)!= SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}
