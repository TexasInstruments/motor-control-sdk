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

#include <position_sense/nikon/include/nikon_api.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* SysConfig-generated arrays (defined in ti_drivers_config.c) */
extern nikon_config gNikonHandle[];
extern uint32_t gNikonConfigNum;

/* ========================================================================== */
/*                      Internal Function Declarations                        */
/* ========================================================================== */

/* NOTE: These internal static functions do not validate handle parameter or pointers.
 * Caller must ensure that pointers are not NULL. */

/**
 * \brief Calculate 3-bit CRC for Nikon communication frame
 *
 * \details Internal function for CRC calculation used by command generation functions.
 *
 * \param handle        Nikon handle from \ref nikon_init
 * \param cmd           Command bits for CRC calculation
 *
 * \return Calculated 3-bit CRC value with frame padding
 */
static uint32_t nikon_calc_3bitcrc(nikon_handle handle, uint32_t cmd);

/**
 *  \brief Generate Memory Data Frame (MDF) for EEPROM access
 *
 *  \details    Internal function that does not validate handle parameter.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  mem_idx         Memory data index (0-3)
 *  \param[in]  mdf_idx         MDF frame index (0 to NUM_MDF_MAX-1)
 */
static void nikon_generate_mdf(nikon_handle handle, uint32_t mem_idx, uint32_t mdf_idx);

/**
 * \brief Clear CFG0 registers for all channels
 *
 * \param handle        Nikon handle from \ref nikon_init
 */
static void nikon_config_clr_cfg0(nikon_handle handle);

/**
 * \brief Configure encoder clock settings
 *
 * \param handle        Nikon handle from \ref nikon_init
 * \param clk_cfg       Clock configuration structure
 */
static void nikon_config_clock(nikon_handle handle, nikon_clk_cfg *clk_cfg);

/**
 * \brief Enable load share mode in PRU-ICSS
 *
 * \param handle        Nikon handle from \ref nikon_init
 */
static void nikon_enable_load_share_mode(nikon_handle handle);

/**
 * \brief Configure primary core mask for multi-channel operation
 *
 * \param handle        Nikon handle from \ref nikon_init
 * \param mask          Channel mask
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
static int32_t nikon_config_primary_core_mask(nikon_handle handle, uint8_t mask);

/**
 *  \brief Initialize hardware and configure clocks
 *
 *  \details    This function initializes the PRU-ICSS hardware interface for Nikon communication,
 *              including clock configuration. Called internally by \ref nikon_init during initialization.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
static int32_t nikon_hw_init(nikon_handle handle);

/**
 * \brief Set default initialization values in PRU-ICSS exchange structure
 *
 * \param handle        Nikon handle from \ref nikon_init
 * \param icss_clk      ICSS core clock frequency in Hz
 */
static void nikon_set_default_initialization(nikon_handle handle, uint64_t icss_clk);

/**
 * \brief Configure channel mapping based on channel mask
 *
 * \param handle        Nikon handle from \ref nikon_init
 * \param mask          Channel mask
 */
static void nikon_config_channel(nikon_handle handle, uint32_t mask);

/**
 *  \brief Parse and extract alarm bits from encoder response
 *
 *  \details    This function parses the alarm field (ALM) from the encoder response and populates
 *              the alarm bits structure. Called internally by \ref nikon_get_pos for commands
 *              that return alarm information.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  enc_num         Encoder number (0 to NUM_ENCODERS_MAX-1)
 *  \param[in]  ch              Channel number (0 to NIKON_NUM_CH_PER_SLICE_MAX-1)
 */
static void nikon_get_alm_bits(nikon_handle handle, uint32_t enc_num, uint32_t ch);

/**
 *  \brief Parse and extract PM alarm bits from encoder response (Nikon 3.0 only)
 *
 *  \details    This function parses the PM alarm field from the encoder response and populates
 *              the PM alarm bits structure. Called internally by \ref nikon_get_pos for Nikon 3.0
 *              encoders that return PM alarm information.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  enc_num         Encoder number (0 to NUM_ENCODERS_MAX-1)
 *  \param[in]  ch              Channel number (0 to NIKON_NUM_CH_PER_SLICE_MAX-1)
 */
static void nikon_get_pm_alm_bits(nikon_handle handle, uint32_t enc_num, uint32_t ch);

/**
 * \brief Configure IEP base address in PRU shared memory
 *
 *  \param[in]  handle              Nikon handle from \ref nikon_init
 *  \param[in]  iep_base_address    IEP base address offset from PRU-ICSS base
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
static int32_t nikon_config_iep_base_address(nikon_handle handle, uint32_t iep_base_address);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default nikon parameters structure */
const nikon_params gNikonDefaultParams =
{
    NULL,                                   /* pruicss_handle */
    NIKON_DEFAULT_CMD_PROCESS_DELAY_US,     /* cmd_process_delay_us */
    NIKON_DEFAULT_FW_WAIT_DELAY_US,         /* fw_wait_delay_us */
    NIKON_DEFAULT_MAX_WAIT_LOOP_COUNT,      /* max_wait_loop_count */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t nikon_command_send(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;

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
        pruicss_xchg->cycle_trigger[0] = (attrs->channel0_enabled) ? NIKON_ENABLE_CYCLE_TRIGGER : NIKON_DISABLE_CYCLE_TRIGGER;
        pruicss_xchg->cycle_trigger[1] = (attrs->channel1_enabled) ? NIKON_ENABLE_CYCLE_TRIGGER : NIKON_DISABLE_CYCLE_TRIGGER;
        pruicss_xchg->cycle_trigger[2] = (attrs->channel2_enabled) ? NIKON_ENABLE_CYCLE_TRIGGER : NIKON_DISABLE_CYCLE_TRIGGER;
    }
    else
    {
        pruicss_xchg->cycle_trigger[0] = NIKON_ENABLE_CYCLE_TRIGGER;
    }

    return SystemP_SUCCESS;
}

int32_t nikon_update_enc_len(nikon_handle handle, uint32_t num_encoders, uint32_t single_turn_len[], uint32_t multi_turn_len[], uint32_t ch)
{
    uint32_t enc_num;
    uint32_t ls_ch = 0;
    nikon_priv *priv;
    const nikon_attrs *attrs;

    /* Validate handle, internal structure pointers, array parameters and array bounds */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (single_turn_len == NULL) ||
       (multi_turn_len == NULL) ||
       (ch >= NIKON_NUM_CH_PER_SLICE_MAX) ||
       (num_encoders == 0) ||
       (num_encoders > NUM_ENCODERS_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(attrs->load_share_enabled)
    {
        ls_ch = ch;
    }

    /* Validate all encoder configurations before making any state changes
     * This ensures that either all encoders are valid and configured,
     * or none are modified if any validation fails. */
    for(enc_num = 0; enc_num < num_encoders; enc_num++)
    {
        /* Validate that single turn + multi turn length does not exceed maximum ABS data length
         * Nikon protocol supports maximum 40-bit absolute position data
         * Note: single_turn_len of 0 is allowed to indicate encoder not connected */
        if((single_turn_len[enc_num] + multi_turn_len[enc_num]) > NIKON_MAX_ABS_LEN)
        {
            return SystemP_FAILURE;
        }
    }

    /* All validations passed - now update the configuration */
    priv->num_encoders[ls_ch] = num_encoders;
    for(enc_num = 0; enc_num < num_encoders; enc_num++)
    {
        priv->single_turn_len[ch][enc_num] = single_turn_len[enc_num];
        priv->multi_turn_len[ch][enc_num] = multi_turn_len[enc_num];
        priv->data_len[ch][enc_num] = priv->single_turn_len[ch][enc_num] + priv->multi_turn_len[ch][enc_num];
    }

    return SystemP_SUCCESS;
}

uint64_t nikon_reverse_bits(uint64_t bits, uint32_t num_bits)
{
    uint32_t temp;
    uint32_t i;
    uint64_t res = 0;
    for(i = 0; i < num_bits; i++)
    {
        temp = bits & 1;
        res = res | (temp << ((num_bits - 1) - i));
        bits = bits >> 1;
    }
    return res;
}

int32_t nikon_get_current_channel(nikon_handle handle, uint32_t ch_idx, uint32_t *channel)
{
    nikon_priv *priv;

    /* Validate handle, internal structure pointer, output pointer, and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (channel == NULL) || (ch_idx >= NIKON_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    *channel = priv->channel[ch_idx];

    return SystemP_SUCCESS;
}

int32_t nikon_update_enc_addr(nikon_handle handle, uint32_t enc_addr, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate parameters, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX) || (enc_addr > NIKON_ENC_ADDR_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->eax[ls_ch] = (uint32_t)nikon_reverse_bits(enc_addr, NIKON_ENC_ADDR_LEN);

    return SystemP_SUCCESS;
}

int32_t nikon_update_eeprom_addr(nikon_handle handle, uint8_t addr, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate handle, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->mem_data[ls_ch][NIKON_MEM_ADDRESS_INDEX] = (uint32_t)nikon_reverse_bits(addr, NIKON_EEPROM_ADDR_LEN);

    return SystemP_SUCCESS;
}

int32_t nikon_update_eeprom_data(nikon_handle handle, uint16_t data, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate handle, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->mem_data[ls_ch][NIKON_MEM_DATA_LOW_INDEX] = (uint32_t)nikon_reverse_bits((data & 0xFFU), NIKON_EEPROM_DATA_BYTE_LEN);
    priv->mem_data[ls_ch][NIKON_MEM_DATA_HIGH_INDEX] = (uint32_t)nikon_reverse_bits(((data & 0xFF00U) >> NIKON_BYTE_SHIFT), NIKON_EEPROM_DATA_BYTE_LEN);

    return SystemP_SUCCESS;
}

int32_t nikon_update_eeprom_bank(nikon_handle handle, uint8_t bank, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate handle, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    priv->mem_data[ls_ch][NIKON_MEM_BANK_INDEX] = (uint32_t)nikon_reverse_bits(bank, NIKON_EEPROM_BANK_LEN);

    return SystemP_SUCCESS;
}

int32_t nikon_update_id_code(nikon_handle handle, uint32_t data, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate parameters, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX) || (data > 0xFFFFFFU))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Use lower 24 bits only */
    priv->mem_data[ls_ch][0] = (uint32_t)nikon_reverse_bits((data & 0xFFU), NIKON_EEPROM_DATA_BYTE_LEN);
    priv->mem_data[ls_ch][1] = (uint32_t)nikon_reverse_bits(((data & 0xFF00U) >> NIKON_BYTE_SHIFT), NIKON_EEPROM_DATA_BYTE_LEN);
    /* 0xFF0000: Third byte mask (bits 16-23) to extract upper 8 bits of 24-bit ID code */
    priv->mem_data[ls_ch][2] = (uint32_t)nikon_reverse_bits(((data & 0xFF0000U) >> (NIKON_BYTE_SHIFT * 2)), NIKON_EEPROM_DATA_BYTE_LEN);

    return SystemP_SUCCESS;
}

int32_t nikon_update_velocity_coefficient(nikon_handle handle, uint32_t data, uint32_t ls_ch)
{
    nikon_priv *priv;

    /* Validate parameters, internal structure pointer and array bounds */
    if((handle == NULL) || (handle->priv == NULL) || (ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX) || (data > 0x7FFFF))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    /* Use lower 19 bits only */
    priv->mem_data[ls_ch][0] = (uint32_t)nikon_reverse_bits((data & 0xFFU), NIKON_EEPROM_DATA_BYTE_LEN);
    priv->mem_data[ls_ch][1] = (uint32_t)nikon_reverse_bits(((data & 0xFF00U) >> NIKON_BYTE_SHIFT), NIKON_EEPROM_DATA_BYTE_LEN);
    /* Use only lower 3 bits for data_low as velocity coefficient is 19 bits long. Move it to most significant bits */
    priv->mem_data[ls_ch][2] = (uint32_t)nikon_reverse_bits(((data & NIKON_VEL_COEFF_THIRD_BYTE_MASK) >> (NIKON_BYTE_SHIFT * 2)), NIKON_EEPROM_DATA_BYTE_LEN);

    return SystemP_SUCCESS;
}

static uint32_t nikon_calc_3bitcrc(nikon_handle handle, uint32_t cmd)
{
    uint8_t ff0 = 0;
    uint8_t ff1 = 0;
    uint8_t ff2 = 0;
    uint8_t crc;
    uint32_t res;
    uint32_t msb;
    uint32_t ex;
    uint32_t i;
    nikon_priv *priv;

    priv = handle->priv;

    /* Initialize with all 1s.
     * Bits are shifted out as frame fields are added. */
    res = 0xFFFFFFFFUL;
    res = res << NIKON_START_BIT_LEN;
    res = (res << NIKON_SYNC_CODE_LEN) | priv->sync_code;

    /* 3bit crc should be calculated for fc + ea + command code or fc + memData/memAddress */
    res = (res << (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN)) | cmd;
    msb = (1UL << (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN - 1));

    for(i = 0; i < (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN); i++)
    {
        /* Check for the MSB(9th in this case) */
        if(cmd & msb)
        {
            ex = ff2 ^ 1;
        }
        else
        {
            ex = ff2 ^ 0;
        }
        ff2 = ff1;

        /* 3 bit CRC algorithm */
        ff1 = ff0 ^ ex;
        ff0 = ex;
        cmd = cmd << 1;
    }

    crc = (ff2 << 2) | (ff1 << 1) | ff0;
    res = (res << NIKON_TX_CRC_LEN) | crc;
    /* Stop bit value (single '1' bit) */
    res = (res << NIKON_STOP_BIT_LEN) | NIKON_STOP_BIT_VALUE;
    /* 0x3: Two '1' bits for t3 delay compensation (binary '11') */
    res = (res << 2) | 0x3;

    return res;
}

int32_t nikon_generate_cdf(nikon_handle handle, uint32_t cmd)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    uint32_t cdf_cmd;
    uint32_t pru_num;
    uint32_t ls_ch;
    uint32_t res;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(((attrs->protocol_version == NIKON_PROTOCOL_V2_1) && (((cmd > CMD_22) && (cmd < CMD_27)) || (cmd > CMD_30))) || ((cmd > CMD_30) && (cmd < CMD_1_VEL)) || (cmd >= CMD_CODE_NUM))
    {
        return SystemP_FAILURE;
    }

    /*
     * Padding of 12 1's has be provided at the beginning to compensate for 2 micro-seconds
     * t2 delay and 2 1's t3 delay compensation start bit = 0, fc = 00/11, encoder adress = EA[0:2],
     * command code = CC[0:4], crc = CRC[2:0], stop bit = 1
     */

    priv->fc = 0;

    if((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd >= CMD_1_VEL) && (cmd <= CMD_18_VEL))
    {
        if(cmd == CMD_1_VEL)
        {
            cmd = CMD_1;
        }
        else if(cmd == CMD_5_VEL)
        {
            cmd = CMD_5;
        }
        else if(cmd == CMD_8_POS)
        {
            cmd = CMD_8;
        }
        else if(cmd == CMD_9_POS)
        {
            cmd = CMD_9;
        }
        else if(cmd == CMD_10_POS)
        {
            cmd = CMD_10;
        }
        else if(cmd == CMD_11_POS)
        {
            cmd = CMD_11;
        }
        else if(cmd == CMD_12_POS)
        {
            cmd = CMD_12;
        }
        else if(cmd == CMD_13_BANK)
        {
            cmd = CMD_13;
            priv->fc = NIKON_FRAME_CODE_BANK;
        }
        else if(cmd == CMD_14_BANK)
        {
            cmd = CMD_14;
            priv->fc = NIKON_FRAME_CODE_BANK;
        }
        else if(cmd == CMD_16_VEL)
        {
            cmd = CMD_16;
            priv->fc = NIKON_FRAME_CODE_BANK;
        }
        else if(cmd == CMD_18_VEL)
        {
            cmd = CMD_18;
            priv->fc = NIKON_FRAME_CODE_BANK;
        }
    }

    for(pru_num = 0; pru_num < attrs->total_channels; pru_num++)
    {
        if(attrs->load_share_enabled)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = attrs->total_channels;
        }

        res = cmd & NIKON_CMD_CODE_MASK;
        res = (uint32_t)nikon_reverse_bits(res, NIKON_COMMAND_CODE_LEN);

        cdf_cmd = priv->fc;
        cdf_cmd = cdf_cmd << NIKON_ENC_ADDR_LEN | priv->eax[ls_ch];
        cdf_cmd = cdf_cmd << NIKON_COMMAND_CODE_LEN | res;
        priv->tx_cdf[ls_ch] = nikon_calc_3bitcrc(handle, cdf_cmd);
    }

    return SystemP_SUCCESS;
}

static void nikon_generate_mdf(nikon_handle handle, uint32_t mem_idx, uint32_t mdf_idx)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    uint32_t mdf_cmd;
    uint32_t pru_num;
    uint32_t ls_ch;

    /*
     * Padding of 12 1's has be provided at the beginning to compensate for 2 micro-seconds
     * t2 delay and 2 1's t3 delay compensation start bit = 0, fc = FC[0:1], memory adress or
     * memory data = MEM[0:7], crc = CRC[2:0], stop bit = 1
     */

    priv = handle->priv;
    attrs = handle->attrs;

    for(pru_num = 0; pru_num < attrs->total_channels; pru_num++)
    {
        if(attrs->load_share_enabled)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = attrs->total_channels;
        }

        mdf_cmd = priv->fc;
        mdf_cmd = mdf_cmd << (NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN) | priv->mem_data[ls_ch][mem_idx];
        priv->tx_mdf[ls_ch][mdf_idx] = nikon_calc_3bitcrc(handle, mdf_cmd);
    }
    priv->fc = 0;
}

int32_t nikon_command_wait(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint32_t loop_count;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /*
     * Minimum and Maximum NIKON cycle time depends on various params as below:
     * TCycle_max = TMA * (number of RX frames * 18) + TX frame(32) + delay between TX and RX +
     * (delay between Multi Transmission commands * (maximum encoder address delay t6))
     * + Delay between two EEPROM access commands(30 milli-seconds)
     * TCycle_min = TMA * (number of Rx frames * 18) + TX frame(32) + delay between TX and RX
     * Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
     * Default max_wait_loop_count is 35, with cmd_process_delay_us of 1000us this gives 35ms timeout.
     * This can vary for different encoders, different commands and multi transmission connection.
     */
    loop_count = priv->max_wait_loop_count;
    while(1)
    {
        if(attrs->load_share_enabled)
        {
            if((pruicss_xchg->cycle_trigger[0] == NIKON_DISABLE_CYCLE_TRIGGER) &&
            (pruicss_xchg->cycle_trigger[1] == NIKON_DISABLE_CYCLE_TRIGGER) &&
            (pruicss_xchg->cycle_trigger[2] == NIKON_DISABLE_CYCLE_TRIGGER))
            {
                break;
            }
        }
        else if(pruicss_xchg->cycle_trigger[0] == NIKON_DISABLE_CYCLE_TRIGGER)
        {
            break;
        }
        if(!priv->is_continuous_mode)
        {
            ClockP_usleep(priv->cmd_process_delay_us);
            loop_count--;
            if(loop_count == 0)
            {
                return SystemP_TIMEOUT;
            }
        }
    }

    return SystemP_SUCCESS;

}

int32_t nikon_command_process(nikon_handle handle)
{
    int32_t ret = SystemP_FAILURE;
    nikon_priv *priv;

    /* Validate handle and internal structure pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;

    if(priv->is_continuous_mode == NIKON_CLEAR_STATUS_FLAG)
    {
        ret = nikon_command_send(handle);
        if(ret != SystemP_SUCCESS)
        {
            return ret;
        }
    }

    ret = nikon_command_wait(handle);
    return ret;
}

int32_t nikon_config_periodic_trigger_cmp_mode(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t pru_num;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Configures Nikon receiver in periodic trigger CMP mode */
    if(attrs->load_share_enabled)
    {
        for(pru_num = 0; pru_num < NIKON_NUM_CH_PER_SLICE_MAX; pru_num++)
        {
            if(attrs->channel_mask & (1U << pru_num))
            {
                pruicss_xchg->opmode[pru_num] = NIKON_CONFIG_PERIODIC_TRIGGER_CMP_MODE;
            }
        }
    }
    else
    {
        pruicss_xchg->opmode[0] = NIKON_CONFIG_PERIODIC_TRIGGER_CMP_MODE;
    }
    priv->is_continuous_mode = NIKON_SET_STATUS_FLAG;

    return SystemP_SUCCESS;
}

int32_t nikon_config_host_trigger(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t pru_num;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Configures Nikon receiver in host trigger mode */
    if(attrs->load_share_enabled)
    {
        for(pru_num = 0; pru_num < NIKON_NUM_CH_PER_SLICE_MAX; pru_num++)
        {
            if(attrs->channel_mask & (1U << pru_num))
            {
                pruicss_xchg->opmode[pru_num] = NIKON_CONFIG_HOST_TRIGGER_MODE;
            }
        }
    }
    else
    {
        pruicss_xchg->opmode[0] = NIKON_CONFIG_HOST_TRIGGER_MODE;
    }
    priv->is_continuous_mode = NIKON_CLEAR_STATUS_FLAG;

    return SystemP_SUCCESS;
}

int32_t nikon_config_periodic_trigger_cap_mode(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t pru_num;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    /* Configures Nikon receiver in periodic trigger CAP mode */
    if(attrs->load_share_enabled)
    {
        for(pru_num = 0; pru_num < NIKON_NUM_CH_PER_SLICE_MAX; pru_num++)
        {
            if(attrs->channel_mask & (1U << pru_num))
            {
                pruicss_xchg->opmode[pru_num] = NIKON_CONFIG_PERIODIC_TRIGGER_CAP_MODE;
            }
        }
    }
    else
    {
        pruicss_xchg->opmode[0] = NIKON_CONFIG_PERIODIC_TRIGGER_CAP_MODE;
    }
    priv->is_continuous_mode = NIKON_SET_STATUS_FLAG;

    return SystemP_SUCCESS;
}

static int32_t nikon_config_iep_base_address(nikon_handle handle, uint32_t iep_base_address)
{
    nikon_priv          *priv;
    nikon_pruicss_xchg  *pruicss_xchg;

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

int32_t nikon_config_iep_cap_event(nikon_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    const nikon_attrs *attrs;
    nikon_priv *priv;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t ch_index = 0;

    /* Validate handle, internal structure pointers and parameters */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL) || (event_num >= NIKON_IEP_MAX_CAP_EVENT) || (channel >= NIKON_NUM_CH_PER_SLICE_MAX))
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
    pruicss_xchg->trigger_params[ch_index].iep_capture_reg = pruicss_xchg->iep_base_address + NIKON_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0 + NIKON_8_BYTE_REG_OFFSET*(event_num);

    /* CAP6 and CAP7 have 2 extra registers for fall capture values, add extra offset */
    /* CAP6 and CAP7 has 2 register bits each. So bit 8 needs to be used for CAP7. Only capture rise bits for CAP6 and CAP7 are used. */
    if(event_num > 6)
    {
        pruicss_xchg->trigger_params[ch_index].iep_cap_event += 1;
        pruicss_xchg->trigger_params[ch_index].iep_capture_reg += NIKON_8_BYTE_REG_OFFSET;
    }

    return ret_val;
}

int32_t nikon_config_iep_cmp_event(nikon_handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    const nikon_attrs *attrs;
    nikon_priv *priv;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t ch_index = 0;

    /* Validate handle, internal structure pointers and parameters */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL) || (event_num >= NIKON_IEP_MAX_CMP_EVENT) || (channel >= NIKON_NUM_CH_PER_SLICE_MAX))
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

static void nikon_config_clr_cfg0(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    void *pruicss_cfg;

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

static void nikon_config_clock(nikon_handle handle, nikon_clk_cfg *clk_cfg)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    void *pruicss_cfg;
    uint32_t rx_reg_val;
    uint32_t tx_reg_val;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Configure RX and TX CFG registers based on PRU slice */
    /* Polarity of Start bit is 0 for Nikon */
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
}

int32_t nikon_calc_clock(nikon_handle handle, nikon_clk_cfg *clk_cfg)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    double freq;
    nikon_pruicss_xchg *pruicss_xchg;

    /* Validate handle, internal structure pointers and clk_cfg parameters */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL) || (clk_cfg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    freq = priv->baud_rate;
    pruicss_xchg = priv->pruicss_xchg;
    freq = freq * MHZ_TO_HZ;
    clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE;
    pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_8X_OS;
    if(attrs->is_core_clk == NIKON_SET_STATUS_FLAG)
    {
        clk_cfg->tx_div = (attrs->core_clk_freq / freq) - 1;
        clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE + 1))) - 1;
        clk_cfg->is_core_clk = NIKON_SET_STATUS_FLAG;
        if(attrs->core_clk_freq == PRU_CORE_CLK_FREQ_200MHZ * MHZ_TO_HZ)
        {
            if(((uint8_t)(priv->baud_rate) % NIKON_FREQ_6_67MHZ) < 1)
            {
                clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_6X;
                clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE_6X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_6X_OS;
            }
        }
        else if(attrs->core_clk_freq == PRU_CORE_CLK_FREQ_300MHZ * MHZ_TO_HZ)
        {
            if(((uint8_t)(priv->baud_rate) % NIKON_FREQ_6_67MHZ) < 1)
            {
                clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_6X | NIKON_RX_ENABLE_FRACTIONAL_DIV;
                clk_cfg->rx_div = (attrs->core_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE_6X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_6X_OS;
            }
        }
    }
    else
    {
        clk_cfg->tx_div = (attrs->uart_clk_freq / freq) - 1;
        clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE + 1))) - 1;
        clk_cfg->is_core_clk = NIKON_CLEAR_STATUS_FLAG;
        if(attrs->uart_clk_freq == PRU_UART_CLK_FREQ_160MHZ * MHZ_TO_HZ)
        {
            if(priv->baud_rate == NIKON_FREQ_8MHZ)
            {
                clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_4X;
                clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE_4X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_4X_OS;
            }
            else if(priv->baud_rate == NIKON_FREQ_16MHZ)
            {
                clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_5X;
                clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE_5X + 1))) - 1;
                pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_5X_OS;
            }
        }
        else if(attrs->uart_clk_freq == PRU_UART_CLK_FREQ_192MHZ * MHZ_TO_HZ)
        {
            if(priv->baud_rate == NIKON_FREQ_16MHZ)
            {
                clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE | NIKON_RX_ENABLE_FRACTIONAL_DIV;
                clk_cfg->rx_div = (attrs->uart_clk_freq / (freq * (NIKON_RX_SAMPLE_SIZE + 1) * NIKON_CLOCK_FRACTIONAL_DIVIDER)) - 1;
                pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_8X_OS;
            }
        }
    }
    if(priv->baud_rate == NIKON_FREQ_2_5MHZ)
    {
       priv->pruicss_xchg->multi_transmission_delay = ((freq * NIKON_MT_DELAY_2_5MHZ_MULTIPLIER) / MHZ_TO_HZ);
    }
    else if(priv->baud_rate == NIKON_FREQ_4MHZ)
    {
       priv->pruicss_xchg->multi_transmission_delay = ((freq * NIKON_MT_DELAY_4MHZ_MULTIPLIER) / MHZ_TO_HZ);
    }
    else if((priv->baud_rate == NIKON_FREQ_16MHZ) || (priv->baud_rate == NIKON_FREQ_8MHZ))
    {
       priv->pruicss_xchg->multi_transmission_delay = ((freq * NIKON_MT_DELAY_8_16MHZ_MULTIPLIER) / MHZ_TO_HZ);
    }
    else if(((uint8_t)(priv->baud_rate) % NIKON_FREQ_6_67MHZ) < 1)
    {
       priv->pruicss_xchg->multi_transmission_delay = ((attrs->core_clk_freq * NIKON_MT_DELAY_6_67MHZ_MULTIPLIER) / MHZ_TO_HZ);
    }
    else
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

static void nikon_enable_load_share_mode(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    void *pruicss_cfg;
    uint32_t reg_val;

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

static int32_t nikon_config_primary_core_mask(nikon_handle handle, uint8_t mask)
{
    nikon_priv *priv;

    priv = handle->priv;

    switch (mask)
    {
        case 1: /* Only Channel 0 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL0_MASK;
            break;
        case 2: /* Channel 1 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL1_MASK;
            break;
        case 3: /* Channel 0 and channel 1 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL0_MASK;
            break;
        case 4: /* Channel 2 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 5: /* Channel 0 and channel 2 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 6: /* Channel 1 and channel 2 connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 7: /* All three channels connected */
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

static int32_t nikon_hw_init(nikon_handle handle)
{
    nikon_clk_cfg clk_cfg;
    int32_t status;


    status = nikon_calc_clock(handle, &clk_cfg);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    nikon_config_clock(handle, &clk_cfg);
    nikon_config_clr_cfg0(handle);

    return SystemP_SUCCESS;
}

int32_t nikon_update_clock_freq(nikon_handle handle, float_t frequency)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    if(((uint8_t)frequency % NIKON_FREQ_6_67MHZ) < 1)
    {
        frequency = (float_t)20/3;
    }

    priv->baud_rate = frequency;
    priv->pruicss_xchg->rx_clk_freq = priv->baud_rate;

    if(nikon_hw_init(handle) != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    if(attrs->load_share_enabled)
    {
        nikon_enable_load_share_mode(handle);
    }

    return SystemP_SUCCESS;
}

static void nikon_set_default_initialization(nikon_handle handle, uint64_t icss_clk)
{
    nikon_priv *priv;
    nikon_pruicss_xchg *pruicss_xchg;
    uint8_t pru_num;

    priv = handle->priv;
    pruicss_xchg = priv->pruicss_xchg;

    /* Initialize parameters to default values */
    pruicss_xchg->pos_crc_len         = NIKON_POS_CRC_LEN;
    pruicss_xchg->rx_clk_freq         = priv->baud_rate;
    pruicss_xchg->delay_1us           = (icss_clk / MHZ_TO_HZ);
    pruicss_xchg->icss_clk            = icss_clk;
    pruicss_xchg->valid_bit_idx       = NIKON_BASE_VALID_BIT_IDX;
    for(pru_num = 0; pru_num < NIKON_NUM_CH_PER_SLICE_MAX; pru_num++)
    {
        /* Default encoder address assumed as 0 */
        priv->eax[pru_num]                         = 0;
        pruicss_xchg->pru_sync_status[pru_num]     = NIKON_CLEAR_STATUS_FLAG;
        pruicss_xchg->opmode[pru_num]              = NIKON_CONFIG_HOST_TRIGGER_MODE;
    }
    priv->is_continuous_mode          = NIKON_CLEAR_STATUS_FLAG;
    /* Sync code for Rx is 010 */
    priv->sync_code                   = 2;
    /* Frame code for CDF is 00 */
    priv->fc                          = 0;
    /* Incorrect bank error */
    priv->bank_error                  = 0;
}

static void nikon_config_channel(nikon_handle handle, uint32_t mask)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint32_t ch_num;

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    pruicss_xchg->channel = mask;

    /*
     * Below for loop iterates for enabled channel number of times.
     * Updates channel in this manner:
     * if ch0 only selected --> priv->channel[0] = 0;
     * if ch1 only selected --> priv->channel[0] = 1;
     * if ch2 only selected --> priv->channel[0] = 2;
     * if ch0 & ch1 are selected --> priv->channel[0] = 0, priv->channel[1] = 1;
     * if ch0 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 2;
     * if ch1 & ch2 are selected --> priv->channel[0] = 1, priv->channel[1] = 2;
     * if ch0, ch1 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 1, priv->channel[2] = 2;
     */
    for(ch_num = 0; ch_num < attrs->total_channels && ch_num < NIKON_NUM_CH_PER_SLICE_MAX; ch_num++)
    {
        if((mask & NIKON_CHANNEL0_MASK) && ch_num == 0)
        {
            priv->channel[ch_num] = 0;
        }
        else if((mask & NIKON_CHANNEL1_MASK) && (ch_num == 0 || ch_num == 1))
        {
            priv->channel[ch_num] = 1;
        }
        else if((mask & NIKON_CHANNEL2_MASK))
        {
            priv->channel[ch_num] = 2;
        }
    }
}

nikon_handle nikon_init(uint32_t index, const nikon_params *params)
{
    int32_t                     status = SystemP_SUCCESS;
    nikon_handle                handle = NULL;
    nikon_priv                  *priv = NULL;
    const nikon_attrs           *attrs = NULL;
    nikon_pruicss_xchg          *pruicss_xchg = NULL;
    uint32_t                    temp;
    void                        *base_addr = NULL;
    uint8_t                     ch_idx;

    /* Validate index and params - gNikonHandle and gNikonConfigNum are generated by SysConfig */
    if((index >= gNikonConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Get handle from SysConfig-generated array */
        handle = (nikon_handle)(&gNikonHandle[index]);

        priv = handle->priv;
        attrs = handle->attrs;

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

        /* Validation of attrs */
        if((attrs->instance >= gNikonConfigNum) ||
           (attrs->mode > NIKON_MODE_MULTI_CHANNEL_MULTI_PRU) ||
           (attrs->pruicss_instance > 1) ||
           (attrs->pruicss_slice > 1) ||
           (attrs->load_share_enabled > 1) ||
           (attrs->channel_mask == 0) || (attrs->channel_mask > 7) ||
           (attrs->channel0_enabled > 1) ||
           (attrs->channel1_enabled > 1) ||
           (attrs->channel2_enabled > 1) ||
           (attrs->total_channels == 0) || (attrs->total_channels > 3) ||
           (attrs->core_clk_freq == 0) ||
           (attrs->uart_clk_freq == 0) ||
           (attrs->iep_clk_freq == 0) ||
           (attrs->is_core_clk > 1) ||
           (attrs->iep_instance > 1) ||
           (attrs->iep_base_addr == NULL))
        {
            status = SystemP_FAILURE;
        }

        /* Validate baud_rate */
        if(status == SystemP_SUCCESS)
        {
            if((attrs->baud_rate != NIKON_FREQ_2_5MHZ) &&
               (attrs->baud_rate != NIKON_FREQ_4MHZ) &&
               (attrs->baud_rate != NIKON_FREQ_8MHZ) &&
               (attrs->baud_rate != NIKON_FREQ_16MHZ) &&
               (((uint8_t)attrs->baud_rate % NIKON_FREQ_6_67MHZ) >= 1))
            {
                status = SystemP_FAILURE;
            }
        }

        /* Validate protocol_version */
        if(status == SystemP_SUCCESS)
        {
            if((attrs->protocol_version != NIKON_PROTOCOL_V2_1) &&
               (attrs->protocol_version != NIKON_PROTOCOL_V3_0))
            {
                status = SystemP_FAILURE;
            }
        }

        /* Validate IEP CMP and CAP event numbers for periodic trigger mode */
        if(status == SystemP_SUCCESS)
        {
            /* Validate IEP CMP event numbers and CAP event numbers */
            if(attrs->load_share_enabled)
            {
                for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
                {
                    if(attrs->channel_mask & (1U << ch_idx))
                    {
                        if((attrs->iep_cmp_event[ch_idx] >= NIKON_IEP_MAX_CMP_EVENT) ||
                        (attrs->iep_cap_event[ch_idx] >= NIKON_IEP_MAX_CAP_EVENT))
                        {
                            status = SystemP_FAILURE;
                            break;
                        }
                    }
                }
            }
            else
            {
                if((attrs->iep_cmp_event[0] >= NIKON_IEP_MAX_CMP_EVENT) ||
                (attrs->iep_cap_event[0] >= NIKON_IEP_MAX_CAP_EVENT))
                {
                    status = SystemP_FAILURE;
                }
            }
        }
    }

    /* Get PRU slice DRAM base */
    if(status == SystemP_SUCCESS)
    {
        if(attrs->pruicss_slice == 1)
        {
            pruicss_xchg = (nikon_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru1DramBase;
        }
        else
        {
            pruicss_xchg = (nikon_pruicss_xchg *)((PRUICSS_HwAttrs *)(params->pruicss_handle->hwAttrs))->pru0DramBase;
        }

        /* Initialize nikon_priv from attrs and params */
        priv->pruicss_xchg = pruicss_xchg;
        priv->pruicss_handle = params->pruicss_handle;
        priv->baud_rate = attrs->baud_rate;
        priv->cmd_process_delay_us = params->cmd_process_delay_us;
        priv->fw_wait_delay_us = params->fw_wait_delay_us;
        priv->max_wait_loop_count = params->max_wait_loop_count;

        /* Set GP mux */
        status = PRUICSS_setGpMuxSelect(params->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Hardware initialization */
        status = nikon_hw_init(handle);
    }

    if(status == SystemP_SUCCESS)
    {
        /*Set IEP base address */
        base_addr = (void *)((PRUICSS_HwAttrs *)(handle->priv->pruicss_handle->hwAttrs))->baseAddr;
        temp = ((uint32_t)attrs->iep_base_addr) - ((uint32_t)base_addr);

        /* Initialize IEP base address in pruicss_xchg */
        status = nikon_config_iep_base_address(handle, temp);
    }

    /* Configure IEP CMP and CAP events for enabled channels */
    if(status == SystemP_SUCCESS)
    {
        if(attrs->load_share_enabled)
        {
            for(ch_idx = 0; ch_idx < NIKON_NUM_CH_PER_SLICE_MAX; ch_idx++)
            {
                /* Check if channel is enabled */
                if(attrs->channel_mask & (1U << ch_idx))
                {
                    /* Configure IEP CMP event for this channel */
                    status = nikon_config_iep_cmp_event(handle, ch_idx, attrs->iep_cmp_event[ch_idx]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }

                    /* Configure IEP CAP event for this channel */
                    status = nikon_config_iep_cap_event(handle, ch_idx, attrs->iep_cap_event[ch_idx]);
                    if(status != SystemP_SUCCESS)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            /* Configure IEP CMP event */
            status = nikon_config_iep_cmp_event(handle, 0, attrs->iep_cmp_event[0]);

            if(status == SystemP_SUCCESS)
            {
                /* Configure IEP CAP event */
                status = nikon_config_iep_cap_event(handle, 0, attrs->iep_cap_event[0]);
            }
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Channel configuration */
        nikon_config_channel(handle, attrs->channel_mask);
    }

    if((status == SystemP_SUCCESS) && (attrs->mode == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU))
    {
        /* Load share configuration if enabled */
        status = nikon_config_load_share(handle, attrs->channel_mask);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Default initialization */
        nikon_set_default_initialization(handle, attrs->core_clk_freq);
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

void nikon_deinit(nikon_handle handle)
{
    nikon_priv *priv;

    if((handle == NULL) || (handle->priv == NULL))
    {
        return;
    }

    priv = handle->priv;
    /* Mark as closed */
    priv->is_open = 0;
}

void nikon_params_init(nikon_params *params)
{
    /* Input parameter validation */
    if(params != NULL)
    {
        *params = gNikonDefaultParams;
    }
}

const nikon_attrs* nikon_get_attrs(nikon_handle handle)
{
    /* Validate handle and attrs pointer */
    if((handle == NULL) || (handle->attrs == NULL))
    {
        return NULL;
    }
    return handle->attrs;
}

nikon_priv* nikon_get_priv(nikon_handle handle)
{
    /* Validate handle and priv pointer */
    if((handle == NULL) || (handle->priv == NULL))
    {
        return NULL;
    }
    return handle->priv;
}

int32_t nikon_config_load_share(nikon_handle handle, uint8_t mask)
{
    int32_t status;

    if((handle == NULL) ||
       (handle->priv == NULL) ||
       (handle->attrs == NULL) ||
       (handle->priv->pruicss_xchg == NULL) ||
       (handle->priv->pruicss_handle == NULL) ||
       (handle->priv->pruicss_handle->hwAttrs == NULL) ||
       (mask == 0) ||
       (mask > 7))
    {
        return SystemP_FAILURE;
    }

    status = nikon_config_primary_core_mask(handle, mask);
    if(status != SystemP_SUCCESS)
    {
        return status;
    }

    nikon_enable_load_share_mode(handle);

    return SystemP_SUCCESS;
}

static void nikon_get_alm_bits(nikon_handle handle, uint32_t enc_num, uint32_t ch)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    uint32_t alm;

    priv = handle->priv;
    attrs = handle->attrs;
    alm = priv->alm_field[ch][enc_num];

    if(attrs->protocol_version == NIKON_PROTOCOL_V3_0)
    {
        priv->alm_bits[ch][enc_num].inc_err_s = alm & 1;
        alm = alm >> 1;
        priv->alm_bits[ch][enc_num].busy_s = alm & 1;
        alm = alm >> 1;
        priv->alm_bits[ch][enc_num].ps_err_s = alm & 1;
        alm = alm >> 1;
        priv->alm_bits[ch][enc_num].st_err_s = alm & 1;
        alm = alm >> 1;
        priv->alm_bits[ch][enc_num].ov_spd_s = alm & 1;
        alm = alm >> 1;
    }
    else
    {
        alm >>= 5;
    }

    priv->alm_bits[ch][enc_num].inc_err_m = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ov_temp = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].mem_busy = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].busy = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ps_err = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].st_err = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].mem_err = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ov_spd = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ov_flow = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].mt_err = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].batt = alm & 1;
}

static void nikon_get_pm_alm_bits(nikon_handle handle, uint32_t enc_num, uint32_t ch)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    uint32_t pm_alm;

    priv = handle->priv;
    attrs = handle->attrs;
    pm_alm = priv->pm_alm_field[ch][enc_num];

    if(attrs->protocol_version == NIKON_PROTOCOL_V3_0)
    {
        pm_alm >>= 2;
        priv->pm_alm_bits[ch][enc_num].ifw_2 = pm_alm & 1;
        pm_alm >>= 1;
        priv->pm_alm_bits[ch][enc_num].ifw_1 = pm_alm & 1;
        pm_alm >>= 3;
        priv->pm_alm_bits[ch][enc_num].incw_2 = pm_alm & 1;
        pm_alm >>= 1;
        priv->pm_alm_bits[ch][enc_num].incw_1 = pm_alm & 1;
    }
}

int32_t nikon_wait_for_encoder_detection(nikon_handle handle)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    uint32_t pru_num;
    uint32_t ls_ch;
    int32_t ret;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;

    for(pru_num = 0; pru_num < attrs->total_channels; pru_num++)
    {
        if(attrs->load_share_enabled)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = attrs->total_channels;
        }
        priv->eax[ls_ch] = NIKON_ENC_ADDR_MASK;
        priv->pruicss_xchg->num_encoders[ls_ch] = priv->num_encoders[ls_ch];
        priv->pruicss_xchg->rx_frame_size[ls_ch] = 0xfff;
    }

    priv->pruicss_xchg->num_rx_frames = NIKON_NUM_RX_FRAMES_FOUR;
    nikon_generate_cdf(handle, CMD_4);

    for(pru_num = 0; pru_num < attrs->total_channels; pru_num++)
    {
        if(attrs->load_share_enabled)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = attrs->total_channels;
        }

        priv->pruicss_xchg->cdf_frame[ls_ch] = priv->tx_cdf[ls_ch];
        priv->eax[ls_ch] = 0;
    }

    ret = nikon_command_process(handle);
    if(ret != SystemP_SUCCESS)
    {
        return ret;
    }

    return SystemP_SUCCESS;
}

int32_t nikon_get_pos(nikon_handle handle, uint32_t cmd)
{
    nikon_priv *priv;
    const nikon_attrs *attrs;
    nikon_pruicss_xchg *pruicss_xchg;
    uint64_t max;
    uint32_t multiturn_mask;
    uint32_t enc_num;
    uint32_t ls_ch;
    uint32_t ch;
    uint32_t ch_num;
    uint32_t loop_cnt;
    uint32_t pru_num;
    uint32_t mdf_num;
    int32_t ret;

    /* Validate handle and internal structure pointers */
    if((handle == NULL) || (handle->priv == NULL) || (handle->attrs == NULL) || (handle->priv->pruicss_xchg == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_xchg = priv->pruicss_xchg;

    if(((attrs->protocol_version == NIKON_PROTOCOL_V2_1) && (((cmd > CMD_22) && (cmd < CMD_27)) || (cmd > CMD_30))) || ((cmd > CMD_30) && (cmd < CMD_1_VEL)) || (cmd >= CMD_CODE_NUM))
    {
        return SystemP_FAILURE;
    }

    ch = priv->channel[0];
    pruicss_xchg->is_memory_access = 0;
    pruicss_xchg->num_mdf = 0;

    if((cmd == CMD_0) || (cmd == CMD_4) || ((cmd >= CMD_27) && (cmd <= CMD_30)))
    {
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_FOUR;
    }
    else if((cmd == CMD_21) || (cmd == CMD_22))
    {
       priv->num_rx_frames = NIKON_NUM_RX_FRAMES_TWO;
    }
    else if((cmd == CMD_23) || (cmd == CMD_24))
    {
       priv->num_rx_frames = NIKON_NUM_RX_FRAMES_FIVE;
    }
    else if((cmd == CMD_25) || (cmd == CMD_26) || (cmd == CMD_1_VEL) || (cmd == CMD_5_VEL))
    {
       priv->num_rx_frames = NIKON_NUM_RX_FRAMES_SIX;
    }
    else if(cmd == CMD_13)
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_READ_ACCESS;
        pruicss_xchg->num_mdf = 1;
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_THREE;

        /* MDF2 for memory address needs to be sent after CDF */
        priv->fc = 3;
        nikon_generate_mdf(handle, 2, 0);
    }
    else if(cmd == CMD_13_BANK)
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_READ_ACCESS;
        pruicss_xchg->num_mdf = 2;
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_FOUR;
        /* Reset bank error */
        priv->bank_error = 0;

        /* MDF3 for bank number needs to be sent after CDF */
        priv->fc = 0;
        nikon_generate_mdf(handle, 3, 0);

        /* MDF2 for memory address needs to be sent after MDF3 */
        priv->fc = 3;
        nikon_generate_mdf(handle, 2, 1);
    }
    else if(cmd == CMD_14)
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_WRITE_ACCESS;
        pruicss_xchg->num_mdf = 3;
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_THREE;

        /* Send MDF0 (Data lower 8 bits), MDF1 (Data upper 8 bits) and MDF2 (memory address) after CDF*/
        for(mdf_num = 0; mdf_num < NUM_MDF_MAX-1; mdf_num++)
        {
            priv->fc = (uint32_t)nikon_reverse_bits(mdf_num + 1 , NIKON_FRAME_CODE_LEN);
            nikon_generate_mdf(handle, mdf_num, mdf_num);
        }
    }
    else if(cmd == CMD_14_BANK)
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_WRITE_ACCESS;
        pruicss_xchg->num_mdf = 4;
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_FOUR;
        /* Reset bank error */
        priv->bank_error = 0;

        /* Send MDF0 (Data lower 8 bits), MDF1 (Data upper 8 bits) after CDF*/
        for(mdf_num = 0; mdf_num < NUM_MDF_MAX-2; mdf_num++)
        {
            priv->fc = (uint32_t)nikon_reverse_bits(mdf_num + 1 , NIKON_FRAME_CODE_LEN);
            nikon_generate_mdf(handle, mdf_num, mdf_num);
        }

        /* Send MDF3 (bank number) and MDF2 (memory address) */
        priv->fc = 0;
        nikon_generate_mdf(handle, 3, 2);

        priv->fc = (uint32_t)nikon_reverse_bits(3, NIKON_FRAME_CODE_LEN);
        nikon_generate_mdf(handle, 2, 3);
    }
    else if((cmd == CMD_18) || (cmd == CMD_18_VEL) || (cmd == CMD_19) || (cmd == CMD_20))
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_WRITE_ACCESS;
        pruicss_xchg->num_mdf = 3;
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_THREE;

        for(mdf_num = 0; mdf_num < NUM_MDF_MAX-1; mdf_num++)
        {
            priv->fc = (uint32_t)nikon_reverse_bits(mdf_num + 1 , NIKON_FRAME_CODE_LEN);
            nikon_generate_mdf(handle, mdf_num, mdf_num);
        }
    }
    else
    {
        priv->num_rx_frames = NIKON_NUM_RX_FRAMES_THREE;
    }

    pruicss_xchg->num_rx_frames = priv->num_rx_frames;

    for(pru_num = 0; pru_num < attrs->total_channels; pru_num++)
    {
        if(attrs->load_share_enabled)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = attrs->total_channels;
        }
        priv->pruicss_xchg->num_encoders[ls_ch] = ((cmd == CMD_4) || (cmd == CMD_5) || (cmd == CMD_5_VEL) ||(cmd == CMD_6) || (cmd == CMD_7) || (cmd == CMD_22) || (cmd == CMD_24) || (cmd == CMD_26) ||(cmd == CMD_28) || (cmd == CMD_30)) ? priv->num_encoders[ls_ch] : 1;
        priv->num_enc_access[ls_ch] = priv->pruicss_xchg->num_encoders[ls_ch];
        pruicss_xchg->cdf_frame[ls_ch] = priv->tx_cdf[ls_ch];
        pruicss_xchg->rx_frame_size[ls_ch] = (pruicss_xchg->num_encoders[ls_ch] == 1) ? (priv->num_rx_frames * (NIKON_RX_ONE_FRAME_LEN + NIKON_START_BIT_LEN + NIKON_STOP_BIT_LEN)) : 0xfff;

        for(mdf_num = 0; mdf_num < pruicss_xchg->num_mdf; mdf_num++)
        {
            pruicss_xchg->mdf_frame[ls_ch][mdf_num] = priv->tx_mdf[ls_ch][mdf_num];
        }

    }

    if((cmd >= CMD_8) && (cmd <= CMD_12))
    {
        loop_cnt = NIKON_NUM_OF_CYCLE_FOR_RESET;
        do
        {
            ret = nikon_command_process(handle);
            if(ret != SystemP_SUCCESS)
            {
                return ret;
            }
        } while(loop_cnt--);
    }
    else
    {
        ret = nikon_command_process(handle);
        if(ret != SystemP_SUCCESS)
        {
            return ret;
        }
    }

    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        ch = priv->channel[ch_num];
        ls_ch = 0;
        if(attrs->load_share_enabled)
        {
            ls_ch = ch;
        }
        for(enc_num = 0; enc_num < pruicss_xchg->num_encoders[ls_ch]; enc_num++)
        {
            switch(cmd)
            {
                case CMD_0:
                case CMD_4:
                    priv->abs_len   = NIKON_MAX_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].abs[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | ((priv->pos_data_info[ch].raw_data3[enc_num]) >> NIKON_POS_CRC_LEN);
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_40BIT;
                    multiturn_mask = (1ULL << (priv->abs_len - priv->single_turn_len[ch][enc_num])) - 1;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] & multiturn_mask;
                    priv->pos_data_info[ch].multi_turn[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].multi_turn[enc_num], (priv->abs_len - priv->single_turn_len[ch][enc_num]));
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> (priv->abs_len - priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_40BIT;
                    break;

                case CMD_1:
                case CMD_2:
                case CMD_5:
                case CMD_6:
                    priv->abs_len   = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | ((priv->pos_data_info[ch].raw_data2[enc_num]) >> NIKON_POS_CRC_LEN);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    break;
                case CMD_1_VEL:
                case CMD_5_VEL:
                    priv->abs_len   = NIKON_MAX_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->pos_data_info[ch].raw_data4[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[3][ch];
                    priv->pos_data_info[ch].raw_data5[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[4][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN) | (priv->pos_data_info[ch].raw_data2[enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].abs[enc_num] << 8) | (priv->pos_data_info[ch].raw_data3[enc_num] >> 8);
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_40BIT;
                    multiturn_mask = (1ULL << (priv->abs_len - priv->single_turn_len[ch][enc_num])) - 1;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] & multiturn_mask;
                    priv->pos_data_info[ch].multi_turn[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].multi_turn[enc_num], (priv->abs_len - priv->single_turn_len[ch][enc_num]));
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> (priv->abs_len - priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_40BIT;
                    /* Extract velocity data */
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)(((priv->pos_data_info[ch].raw_data3[enc_num] & 0xFF) << 24) |
                                                                (priv->pos_data_info[ch].raw_data4[enc_num] << 8) |
                                                                ((priv->pos_data_info[ch].raw_data5[enc_num]) >> NIKON_POS_CRC_LEN));
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].velocity[enc_num], NIKON_VEL_LEN);
                    break;
                case CMD_3:
                case CMD_7:
                case CMD_8:
                case CMD_9:
                case CMD_10:
                case CMD_11:
                case CMD_12:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->alm_field[ch][enc_num] = priv->pos_data_info[ch].raw_data1[enc_num];
                    nikon_get_alm_bits(handle, enc_num, ch);
                    if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
                    {
                        priv->pm_alm_field[ch][enc_num] = ((priv->pos_data_info[ch].raw_data2[enc_num]) >> NIKON_POS_CRC_LEN);
                        nikon_get_pm_alm_bits(handle, enc_num, ch);
                    }
                    break;
                case CMD_8_POS:
                case CMD_9_POS:
                case CMD_10_POS:
                case CMD_11_POS:
                case CMD_12_POS:
                    priv->abs_len   = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | ((priv->pos_data_info[ch].raw_data2[enc_num]) >> NIKON_POS_CRC_LEN);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];

                    break;
                case CMD_13:
                case CMD_14:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    /* At EEPROM temperature address, DTB[0:9] (temperature data) is encoded */
                    if((uint32_t)nikon_reverse_bits(((priv->pos_data_info[ch].raw_data2[enc_num]) >> NIKON_POS_CRC_LEN), NIKON_EEPROM_ADDR_LEN) == NIKON_EEPROM_TEMP_ADDRESS)
                    {
                        priv->temperature[ch][enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] & 0xFFFF) >> NIKON_TEMP_DATA_SHIFT;  /* 10 bit temperature information */
                        priv->temperature[ch][enc_num] = (uint32_t)nikon_reverse_bits(priv->temperature[ch][enc_num], NIKON_DB_BITS_LEN);
                        priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num] * NIKON_TEMPERATURE_SCALE_FACTOR;
                    }
                    break;
                case CMD_13_BANK:
                case CMD_14_BANK:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    if((priv->pos_data_info[ch].raw_data3[enc_num] & 0x8000) != 0)
                    {
                        priv->bank_error = 1;
                    }
                    break;
                case CMD_15:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->temperature[ch][enc_num]             = (priv->pos_data_info[ch].raw_data1[enc_num] >> (NIKON_RX_ONE_FRAME_LEN - NIKON_DB_BITS_LEN)) & NIKON_DB_BITS_MASK;
                    priv->temperature[ch][enc_num]             = nikon_reverse_bits(priv->temperature[ch][enc_num], NIKON_DB_BITS_LEN);
                    priv->temperature[ch][enc_num]             = priv->temperature[ch][enc_num] * NIKON_TEMPERATURE_SCALE_FACTOR;
                    break;

                case CMD_16:
                case CMD_17:
                case CMD_18:
                case CMD_19:
                case CMD_20:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->identification_code[ch]              = ((pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch]) << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | (pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN);
                    priv->identification_code[ch]              = (uint32_t)nikon_reverse_bits(priv->identification_code[ch], NIKON_ID_CODE_LEN);
                    break;
                case CMD_16_VEL:
                case CMD_18_VEL:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->velocity_coefficient[ch]             = ((pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch]) << 3) | ((pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] & 0xE000) >> 13);
                    priv->velocity_coefficient[ch]             = (uint32_t)nikon_reverse_bits(priv->velocity_coefficient[ch], NIKON_VEL_COEFFICIENT_LEN);
                    break;
                case CMD_21:
                case CMD_22:
                    priv->abs_len                              = NIKON_MIN_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].abs[enc_num]       = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch] & 0x1FF;    /*ABS[0:8] is encoded in info field for command 21 and 22*/
                    priv->pos_data_info[ch].abs[enc_num]       = ((priv->pos_data_info[ch].raw_data1[enc_num]) >> NIKON_POS_CRC_LEN) | (priv->pos_data_info[ch].abs[enc_num] << 8);
                    priv->pos_data_info[ch].abs[enc_num]       = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num]       = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_17BIT;
                    break;
                case CMD_23:
                case CMD_24:
                    priv->abs_len   = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->pos_data_info[ch].raw_data4[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[3][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | (priv->pos_data_info[ch].raw_data2[enc_num] >> 8);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    /* Extract velocity data */
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)(((priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 24) |
                                                                (priv->pos_data_info[ch].raw_data3[enc_num] << 8) |
                                                                ((priv->pos_data_info[ch].raw_data4[enc_num] & 0xFF00) >> NIKON_POS_CRC_LEN));
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].velocity[enc_num], NIKON_VEL_LEN);
                    break;
                case CMD_25:
                case CMD_26:
                    priv->abs_len   = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->pos_data_info[ch].raw_data4[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[3][ch];
                    priv->pos_data_info[ch].raw_data5[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[4][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | (priv->pos_data_info[ch].raw_data2[enc_num] >> 8);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    /* Extract velocity data */
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)(((priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 24) |
                                                                (priv->pos_data_info[ch].raw_data3[enc_num] << 8) |
                                                                ((priv->pos_data_info[ch].raw_data4[enc_num] & 0xFF00) >> 8));
                    priv->pos_data_info[ch].velocity[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].velocity[enc_num], NIKON_VEL_LEN);
                    /* Extract acceleration data */
                    priv->pos_data_info[ch].acc[enc_num] = (uint32_t)(((priv->pos_data_info[ch].raw_data4[enc_num] & 0xFF) << 8) | ((priv->pos_data_info[ch].raw_data5[enc_num] & 0xFF00) >> NIKON_POS_CRC_LEN));
                    priv->pos_data_info[ch].acc[enc_num] = (uint32_t)nikon_reverse_bits(priv->pos_data_info[ch].acc[enc_num], NIKON_ACC_LEN);
                    break;
                case CMD_27:
                case CMD_28:
                    priv->abs_len                              = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->alm_field[ch][enc_num] = ((priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 8) | ((priv->pos_data_info[ch].raw_data3[enc_num]) >> NIKON_POS_CRC_LEN);
                    nikon_get_alm_bits(handle, enc_num, ch);
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> 8;
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    break;

                case CMD_29:
                case CMD_30:
                    priv->abs_len                              = NIKON_AVG_ABS_LEN;
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> 8;
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & NIKON_MASK_24BIT;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    priv->temperature[ch][enc_num] = (priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 2;
                    /* Use 8 bits from raw_data2 and 2 most significant bits from raw_data3 for temperature*/
                    priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num] | (((priv->pos_data_info[ch].raw_data3[enc_num]) >> 14) & 0x3);
                    priv->temperature[ch][enc_num] = (uint32_t)nikon_reverse_bits(priv->temperature[ch][enc_num], NIKON_DB_BITS_LEN);
                    priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num] * NIKON_TEMPERATURE_SCALE_FACTOR;
                    break;
            default:
                break;
            }

            if((cmd == CMD_21) || (cmd == CMD_22))
            {
                priv->enc_info[ch].enc_status[enc_num] = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_CMD_21_22_IF_DATA_LEN)) & NIKON_ENC_STATUS_BIT_MASK;
                priv->enc_info[ch].enc_addr[enc_num]   = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_CMD_21_22_IF_DATA_LEN + NIKON_ENC_STATUS_BIT_LEN)) & NIKON_ENC_ADDR_MASK;
            }
            else
            {
                priv->enc_info[ch].enc_status[enc_num] =  priv->pos_data_info[ch].raw_data0[enc_num] & NIKON_ENC_STATUS_MASK;
                priv->enc_info[ch].enc_status[enc_num]  = (uint32_t)nikon_reverse_bits(priv->enc_info[ch].enc_status[enc_num], NIKON_ENC_STATUS_LEN);
                priv->enc_info[ch].enc_cmd[enc_num]    = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_ENC_STATUS_LEN + NIKON_FIXED_BIT_LEN)) & NIKON_CMD_CODE_MASK;
                priv->enc_info[ch].enc_cmd[enc_num]   = (uint32_t)nikon_reverse_bits(priv->enc_info[ch].enc_cmd[enc_num], NIKON_COMMAND_CODE_LEN);
                priv->enc_info[ch].enc_addr[enc_num]   = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_ENC_STATUS_LEN + NIKON_COMMAND_CODE_LEN + NIKON_FIXED_BIT_LEN)) & NIKON_ENC_ADDR_MASK;
            }

            priv->enc_info[ch].enc_addr[enc_num]  = (uint32_t)nikon_reverse_bits(priv->enc_info[ch].enc_addr[enc_num], NIKON_ENC_ADDR_LEN);
            max = (1ULL << priv->single_turn_len[ch][enc_num]);
            priv->pos_data_info[ch].angle[enc_num] = (float)(priv->pos_data_info[ch].abs[enc_num] & (max - 1))/max * NIKON_DEGREES_PER_REVOLUTION;
            priv->pos_data_info[ch].rcv_crc[enc_num]          = pruicss_xchg->pos_data_res[enc_num].crc.pos_rcv_crc[ch];
            priv->pos_data_info[ch].otf_crc[enc_num]          = pruicss_xchg->pos_data_res[enc_num].crc.pos_otf_crc[ch];
            priv->pos_data_info[ch].crc_err_cnt[enc_num]      = pruicss_xchg->pos_data_res[enc_num].crc.pd_crc_err_cnt[ch];
        }
    }
    return SystemP_SUCCESS;
}
