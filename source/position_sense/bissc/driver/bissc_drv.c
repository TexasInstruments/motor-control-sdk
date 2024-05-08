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

#include <position_sense/bissc/include/bissc_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
static struct bissc_priv bissc_priv;

void bissc_command_send(struct bissc_priv *priv)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    if(priv->load_share)
    {
        pruicss_xchg->cycle_trigger[0] = (pruicss_xchg->channel & 0x1)?0x1:0;
        pruicss_xchg->cycle_trigger[1] = (pruicss_xchg->channel & 0x2)?0x1:0;
        pruicss_xchg->cycle_trigger[2] = (pruicss_xchg->channel & 0x4)?0x1:0;
    }
    else
    {
       pruicss_xchg->cycle_trigger[0] = 0x1;
    }
}

int32_t bissc_command_wait(struct bissc_priv *priv)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    uint32_t timeout;
    /*  Minimum and Maximum BiSSC cycle time depends on various params as below:
        TCycle_min = TMA ∗ (5 + DLEN + CRCLEN) + tLineDelay + tbusy_max + busy_s_max + tTO
        Instead wait for max of 5 ms as this can vary for different encoders and for daisy chain
    */
    timeout = BISSC_MAX_CYCLE_TIMEOUT;
    while(1)
    {
        if(priv->load_share)
        {
            if((pruicss_xchg->cycle_trigger[0] == 0 ) && (pruicss_xchg->cycle_trigger[1] == 0) && (pruicss_xchg->cycle_trigger[2] == 0))
            {
                break;
            }
        }
        else if(pruicss_xchg->cycle_trigger[0] == 0)
        {
            break;
        }
        if(!priv->is_continuous_mode)
        {
            ClockP_usleep(1000);
            timeout--;
            if(timeout == 0)
            {
                return SystemP_FAILURE;
            }
        }
    }
    return SystemP_SUCCESS;
}

int32_t bissc_command_process(struct bissc_priv *priv)
{
    int32_t ret = SystemP_FAILURE;
    bissc_command_send(priv);
    ret = bissc_command_wait(priv);
    return ret;
}

void bissc_config_periodic_trigger(struct bissc_priv *priv)
{
    /* Configures bissc receiver in periodic trigger mode */
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    if(priv->load_share)
    {
        pruicss_xchg->opmode[0] = 0;
        pruicss_xchg->opmode[1] = 0;
        pruicss_xchg->opmode[2] = 0;
    }
    else
    {
        pruicss_xchg->opmode[0] = 0;
    }
    priv->is_continuous_mode = 0x1;
}

void bissc_config_host_trigger(struct bissc_priv *priv)
{
    /* Configures bissc receiver in host trigger mode */
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;

    if(priv->load_share)
    {
        pruicss_xchg->opmode[0] = 0x1;
        pruicss_xchg->opmode[1] = 0x1;
        pruicss_xchg->opmode[2] = 0x1;
    }
    else
    {
        pruicss_xchg->opmode[0] = 0x1;
    }
    priv->is_continuous_mode = 0x0;
}

void bissc_enable_load_share_mode(struct bissc_priv *priv)
{
   void *pruicss_cfg = priv->pruicss_cfg;
    uint32_t rgval;
    if(priv->pruicss_slicex)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER);
        rgval |= CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER);
        rgval |= CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, rgval);
    }
}

void bissc_config_primary_core_mask(struct bissc_priv *priv, uint8_t mask)
{
    switch (mask)
    {
        case 1: /*only channel0 connected*/
            priv->pruicss_xchg->primary_core_mask=0x1;
            break;
        case 2: /*channel1 connected*/
            priv->pruicss_xchg->primary_core_mask=0x2;
            break;
        case 3: /*channel0 and channel1 connected*/
            priv->pruicss_xchg->primary_core_mask=0x1;
            break;
        case 4: /*channel2 connected*/
            priv->pruicss_xchg->primary_core_mask=0x4;
            break;
        case 5: /*channel0 and channel2 connnected*/
            priv->pruicss_xchg->primary_core_mask=0x4;
            break;
        case 6: /*channel1 and channel2 connected*/
            priv->pruicss_xchg->primary_core_mask=0X4;
            break;
        case 7: /*all three channel connected*/
            priv->pruicss_xchg->primary_core_mask=0x4;
            break;
    }
}

uint32_t bissc_get_totalchannels(struct bissc_priv *priv)
{
    return priv->totalchannels;
}

uint32_t bissc_get_current_channel(struct bissc_priv *priv, uint32_t ch_idx)
{
    return priv->channel[ch_idx];
}

void bissc_update_clock_freq(struct bissc_priv *priv, uint32_t frequency)
{
    priv->baud_rate = frequency;
}

void bissc_clear_data_len(struct bissc_priv *priv)
{
    uint32_t ch_num, enc_num;
    for(ch_num = 0; ch_num < NUM_ED_CH_MAX; ch_num++)
    {
        priv->num_encoders[ch_num] = 0;
        for(enc_num = 0; enc_num < NUM_ENCODERS_MAX; enc_num++)
        {
            priv->single_turn_len[ch_num][enc_num] = 0;
            priv->multi_turn_len[ch_num][enc_num] = 0;
            priv->data_len[ch_num][enc_num] = 0;
        }
    }
}

int32_t bissc_get_pos(struct bissc_priv *priv)
{
    uint32_t   raw_data0, raw_data1, shift, sl_num, max, ch_num, numencoders, ls_ch;
    int32_t    ch = 0;
    if(bissc_command_process(priv) < 0)
    {
        return SystemP_FAILURE;
    }
    for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
    {
        ch = priv->channel[ch_num];
        if(priv->load_share)
        {
            numencoders = priv->num_encoders[ch];
            ls_ch = ch;
        }
        else
        {
            numencoders = priv->num_encoders[0];
            ls_ch = 0;
        }
        for(sl_num = 0; sl_num < numencoders; sl_num++)
        {
            raw_data0 = priv->pruicss_xchg->pos_data_res[sl_num].raw_data[ch].pos_data_word0;
            raw_data1 = priv->pruicss_xchg->pos_data_res[sl_num].raw_data[ch].pos_data_word1;
            max = pow(2, priv->single_turn_len[ls_ch][sl_num]);
            if((priv->data_len[ls_ch][sl_num] + BISSC_POS_CRC_LEN + BISSC_EW_LEN)  <= 32)
            {
                priv->raw_data                                = (uint64_t) raw_data0;
                priv->enc_pos_data[ch].position[sl_num]       = (uint64_t) (raw_data0 >> (BISSC_POS_CRC_LEN + BISSC_EW_LEN));
                priv->enc_pos_data[ch].ew[sl_num]             = ((raw_data0 >> (BISSC_POS_CRC_LEN )) & 0x03);
                priv->enc_pos_data[ch].num_of_turns[sl_num]   = priv->enc_pos_data[ch].position[sl_num] >> priv->single_turn_len[ls_ch][sl_num];
                priv->enc_pos_data[ch].angle[sl_num]          = (float)(priv->enc_pos_data[ch].position[sl_num] & (max - 1)) / max * (float)360;
                priv->enc_pos_data[ch].rcv_crc[sl_num]        = raw_data0 & 0x3F;
                priv->enc_pos_data[ch].otf_crc[sl_num]        = priv->pruicss_xchg->pos_data_res[sl_num].pos_data_otf_crc[ch];
                priv->pd_crc_err_cnt[ch][sl_num]              = priv->pruicss_xchg->pos_data_res[sl_num].pd_crc_err_cnt[ch];
            }
            else
            {
                shift = ((priv->data_len[ls_ch][sl_num] + BISSC_POS_CRC_LEN + BISSC_EW_LEN) - 32);
                priv->raw_data                                = (uint64_t) raw_data0 <<  shift | raw_data1;
                priv->enc_pos_data[ch].position[sl_num]       = (uint64_t) (priv->raw_data >> (BISSC_POS_CRC_LEN + BISSC_EW_LEN));
                priv->enc_pos_data[ch].ew[sl_num]             = ((priv->raw_data >> (BISSC_POS_CRC_LEN )) & 0x03);
                priv->enc_pos_data[ch].num_of_turns[sl_num]   = priv->enc_pos_data[ch].position[sl_num] >> priv->single_turn_len[ls_ch][sl_num];
                priv->enc_pos_data[ch].angle[sl_num]          = (float)(priv->enc_pos_data[ch].position[sl_num] & (max - 1)) / max * (float)360;
                priv->enc_pos_data[ch].rcv_crc[sl_num]        = priv->raw_data & 0x3F;
                priv->enc_pos_data[ch].otf_crc[sl_num]        = priv->pruicss_xchg->pos_data_res[sl_num].pos_data_otf_crc[ch];
                priv->pd_crc_err_cnt[ch][sl_num]              = priv->pruicss_xchg->pos_data_res[sl_num].pd_crc_err_cnt[ch];
            }
        }
    }
    return SystemP_SUCCESS;
}

void bissc_config_clock(struct bissc_priv *priv,
                        struct bissc_clk_cfg *clk_cfg)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    /* Set PRU1_ED_RX_SB_POL polarity bit to 0 for bissc, required for ICSSG (don't care for ICSSM) */
    if(priv->pruicss_slicex)
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, ((clk_cfg->rx_div << 16) |
            (clk_cfg->is_core_clk << 4) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, clk_cfg->tx_div << 16 |
            (clk_cfg->is_core_clk << 4));
    }
    else
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0RXCFGREGISTER, ((clk_cfg->rx_div << 16) |
            (clk_cfg->is_core_clk << 4) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, clk_cfg->tx_div << 16 |
                (clk_cfg->is_core_clk << 4));
    }
    if(priv->load_share)
        bissc_enable_load_share_mode(priv);
    /* Clock configuration has changed - Indicate fw to measure the delay again */
    pruicss_xchg->measure_proc_delay = 1;

}

void bissc_config_channel(struct bissc_priv *priv, int32_t mask, int32_t totalch)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    int32_t ch_num;
    pruicss_xchg->channel = mask;
    priv->totalchannels = totalch;
    /*  Below for loop iterates for enabled channel number of times.
        Updates channel in this manner:
        if ch0 only selected --> priv->channel[0] = 0;
        if ch1 only selected --> priv->channel[0] = 1;
        if ch2 only selected --> priv->channel[0] = 2;
        if ch0 & ch1 are selected --> priv->channel[0] = 0, priv->channel[1] = 1;
        if ch0 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 2;
        if ch1 & ch2 are selected --> priv->channel[0] = 1, priv->channel[1] = 2;
        if ch0, ch1 & ch2 are selected --> priv->channel[0] = 0, priv->channel[1] = 1, priv->channel[1] = 2;
    */
    for(ch_num = 0; ch_num < totalch; ch_num++)
    {
        if((mask & 1) && ch_num == 0)
            priv->channel[ch_num] = 0;
        else if((mask & 2) && (ch_num == 0 || ch_num == 1))
            priv->channel[ch_num] = 1;
        else if((mask & 4))
            priv->channel[ch_num] = 2;
    }
}

void bissc_config_load_share(struct bissc_priv *priv, int32_t mask)
{
    priv->load_share = 1; /*Enable load-share*/
    bissc_config_primary_core_mask(priv, mask);
    bissc_enable_load_share_mode(priv);
}

int32_t bissc_wait_for_fw_initialization(struct bissc_priv *priv, uint32_t timeout, uint8_t mask)
{
    int32_t i;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    for(i = 0; i < timeout; i++)
    {
         if(priv->load_share)  /* for loadshare mode*/
        {
            switch (mask) {
                case 1: /*channel 0 connected*/
                    if((pruicss_xchg->status[0] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 2: /*channel 1 connected*/
                    if((pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 3: /*channel 0 and 1 connected*/
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 4: /*channel 2 connected*/
                    if((pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 5: /*channel 0 and 2 connnected*/
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 6: /*channel 1 and 2 connected*/
                    if((pruicss_xchg->status[1] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 7: /*all three channel connected*/
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[1] & 1) && ( pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
            }
            ClockP_usleep(1000 * 1);
        }
        else if(pruicss_xchg->status[0] & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(1000 * 1);
        }
    }
    if(i == timeout)
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

void bissc_get_enc_proc_delay(struct bissc_priv *priv)
{
    int32_t i;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    for(i = 0; i < NUM_ED_CH_MAX; i++)
    {
        priv->proc_delay[i] = pruicss_xchg->proc_delay[i];
    }
}

void bissc_config_clr_cfg0(struct bissc_priv *priv)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    if(priv->pruicss_slicex)
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER, 0);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER, 0);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER, 0);
    }
    else
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER, 0);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER, 0);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER, 0);
    }
}

void bissc_config_endat_mode(struct bissc_priv *priv)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    if(priv->pruicss_slicex)
    {
        HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSSCFG_GPCFG1 + 3, 4);
    }
    else
    {
        HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSSCFG_GPCFG0 + 3, 4);
    }
}

int32_t bissc_calc_clock(struct bissc_priv *priv, struct bissc_clk_cfg *clk_cfg)
{
    uint32_t freq = priv->baud_rate;
    clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE;

    if((freq == BISSC_FREQ_1MHZ) || (freq == BISSC_FREQ_2MHZ) || (freq == BISSC_FREQ_8MHZ))
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->uart_clk_freq / freq) - 1;
        clk_cfg->rx_div = (priv->uart_clk_freq / (freq * 8)) - 1;
        clk_cfg->is_core_clk = 0;
    }
    else if(freq == BISSC_FREQ_5MHZ)
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->core_clk_freq / freq) - 1;
        clk_cfg->is_core_clk = 1;
        if(priv->core_clk_freq == (300 * 1000 * 1000))
        {
            /* For 300MHz using fraction factor */
            clk_cfg->rx_div = ((priv->core_clk_freq / (freq * 8 * 1.5)) - 1);
            clk_cfg->rx_div_attr = clk_cfg->rx_div_attr | BISSC_RX_ENABLE_FRACTIONAL_DIV;
        }
        else
        {
            clk_cfg->rx_div = ((priv->core_clk_freq / (freq * 8)) - 1);
        }
    }
    else if(freq == BISSC_FREQ_10MHZ)
    {
        freq = freq* 1000 * 1000;
        clk_cfg->tx_div = (priv->core_clk_freq / freq) - 1;
        clk_cfg->is_core_clk = 1;
        clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE_10MHZ;
        if(priv->core_clk_freq == (300 * 1000 * 1000))
        {
            /* For 300MHz using fraction factor */
            clk_cfg->rx_div = ((priv->core_clk_freq / (freq * 4 * 1.5)) - 1);
            clk_cfg->rx_div_attr = clk_cfg->rx_div_attr | BISSC_RX_ENABLE_FRACTIONAL_DIV;
        }
        else
        {
            clk_cfg->rx_div = ((priv->core_clk_freq / (freq * 4)) - 1);
        }
    }
    else
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

void bissc_hw_init(struct bissc_priv *priv)
{
    struct bissc_clk_cfg clk_cfg;
    bissc_calc_clock(priv, &clk_cfg);
    bissc_config_endat_mode(priv);
    bissc_config_clock(priv, &clk_cfg);
    bissc_config_clr_cfg0(priv);
}


struct bissc_priv *bissc_init(PRUICSS_Handle gPruIcssXHandle,
                              int32_t slice,
                              uint32_t frequency,
                              uint32_t core_clk_freq,
                              uint32_t uart_clk_freq)
{
    struct bissc_pruicss_xchg *pruicss_xchg;
    void *pruicss_cfg;
    void *pruicss_iep;
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    if(slice)
        pruicss_xchg =  (struct bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru1DramBase;
    else
        pruicss_xchg =  (struct bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru0DramBase;
    bissc_priv.pruicss_xchg = pruicss_xchg;
    bissc_priv.pruicss_cfg = pruicss_cfg;
    bissc_priv.pruicss_iep = pruicss_iep;
    bissc_priv.pruicss_slicex = slice;
    bissc_priv.baud_rate = frequency;
    bissc_priv.core_clk_freq = core_clk_freq;
    bissc_priv.uart_clk_freq = uart_clk_freq;
    bissc_hw_init(&bissc_priv);
    return &bissc_priv;
}

void bissc_update_max_proc_delay(struct bissc_priv *priv)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    pruicss_xchg->fifo_bit_idx = 4;                          /* 8x over clock - middle bit*/
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
        pruicss_xchg->fifo_bit_idx = 2;                          /* 4x over clock - middle bit*/
    }
}

int32_t bissc_wait_measure_proc_delay(struct bissc_priv *priv, uint32_t timeout)
{
    int32_t i;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    for(i = 0; i < timeout; i++)
    {
        if(pruicss_xchg->measure_proc_delay & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(1000);
        }
    }
    if(i == timeout)
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

void bissc_set_default_initialization(struct bissc_priv *priv, uint64_t icssgclk)
{
    int8_t ch_num, totalchns, ls_ch;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    /* Initialize parameters to default values */
    if(priv->load_share)
        totalchns = priv->totalchannels;
    else
        totalchns = 1;
    for( ch_num = 0; ch_num < totalchns; ch_num++)
    {
        if(priv->load_share)
            ls_ch = priv->channel[ch_num];
        else
            ls_ch = 0;
        pruicss_xchg->enc_len[ls_ch].data_len[0]           = BISSC_POS_DATA_LEN_DEFAULT;
        priv->data_len[ls_ch][0]                           = BISSC_POS_DATA_LEN_DEFAULT;
        priv->single_turn_len[ls_ch][0]                    = BISSC_POS_DATA_LEN_DEFAULT;
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
    bissc_update_max_proc_delay(priv);
    pruicss_xchg->delay_40us          = ((icssgclk*40)/1000000);
    pruicss_xchg->delay_100ms         = (icssgclk / 10); /*((icssgclk*100000) / 1000000)*/
    pruicss_xchg->icssg_clk           = icssgclk;
    pruicss_xchg->valid_bit_idx       = 24;
    pruicss_xchg->measure_proc_delay  = 1;
    pruicss_xchg->execution_state[0]  = 0;
    pruicss_xchg->execution_state[1]  = 0;
    pruicss_xchg->execution_state[2]  = 0;
    pruicss_xchg->opmode[0]           = 1;
    pruicss_xchg->opmode[1]           = 1;
    pruicss_xchg->opmode[2]           = 1;
    priv->is_continuous_mode          = 0;
    priv->cmp3                        = ((icssgclk*100)/1000000);     /* 100usec delay */
}

static uint8_t bissc_calc_ctrl_crc(uint32_t ctrl_cmd, uint8_t num_bits)
{
    uint8_t ff0 = 0, ff1 = 0, ff2 = 0, ff3 = 0, crc;
    uint32_t msb, ex, i;
    msb = pow(2, (num_bits - 1));
    for(i = 0; i < num_bits; i++)
    {
        if( ctrl_cmd & msb)          /*Check for the MSB(11th in this case)*/
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

uint32_t bissc_generate_ctrl_cmd(struct bissc_priv *priv,
                                 int8_t ls_ch,
                                 uint32_t ctrl_write_status,
                                 uint32_t ctrl_reg_address,
                                 uint32_t ctrl_reg_data,
                                 uint32_t ctrl_enc_id)
{
    uint32_t ctrl_cmd = 0;
    uint8_t crc;
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

void bissc_update_data_len(struct bissc_priv *priv, uint32_t single_turn_len[], uint32_t multi_turn_len[], int32_t ch_num)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    uint32_t sl_num, ls_ch;
    if(priv->load_share)
        ls_ch = priv->channel[ch_num];
    else
        ls_ch = 0;
    for( sl_num = 0; sl_num < NUM_ENCODERS_MAX; sl_num++)
    {
        if(single_turn_len[sl_num])
        {
            priv->single_turn_len[ls_ch][sl_num] =  single_turn_len[sl_num];
            priv->multi_turn_len[ls_ch][sl_num] =  multi_turn_len[sl_num];
            priv->data_len[ls_ch][sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num];
            pruicss_xchg->enc_len[ls_ch].data_len[sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num];
            priv->num_encoders[ls_ch]++;
        }
    }
    pruicss_xchg->enc_len[0].num_encoders = priv->num_encoders[0];
    pruicss_xchg->enc_len[1].num_encoders = priv->num_encoders[1];
    pruicss_xchg->enc_len[2].num_encoders = priv->num_encoders[2];
}

int32_t bissc_set_ctrl_cmd_and_process(struct bissc_priv *priv, uint32_t ctrl_cmd[])
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    pruicss_xchg->ctrl_cmd[0] = ctrl_cmd[0];
    pruicss_xchg->ctrl_cmd[1] = ctrl_cmd[1];
    pruicss_xchg->ctrl_cmd[2] = ctrl_cmd[2];
    int32_t ch = 0, ch_num;
    if(priv->load_share)
    {
        pruicss_xchg->ctrl_cmd_status[0] = (pruicss_xchg->channel & 0x1) ? 1:0;
        pruicss_xchg->ctrl_cmd_status[1] = (pruicss_xchg->channel & 0x2) ? 1:0;
        pruicss_xchg->ctrl_cmd_status[2] = (pruicss_xchg->channel & 0x4) ? 1:0;
        while(pruicss_xchg->ctrl_cmd_status[0] + pruicss_xchg->ctrl_cmd_status[1] + pruicss_xchg->ctrl_cmd_status[2])
        {
            if(bissc_command_process(priv) < 0)
            {
                return SystemP_FAILURE;
            }
            ClockP_usleep(1000);
        }
    }
    else
    {
        pruicss_xchg->ctrl_cmd_status[0] = 1;
        while(pruicss_xchg->ctrl_cmd_status[0] & 1)
        {
            if(bissc_command_process(priv) < 0)
            {
                return SystemP_FAILURE;
            }
            ClockP_usleep(1000);
        }
    }
    /*supplying 2 extra cycles for ctrl communication stop bit */
    bissc_command_process(priv);
    ClockP_usleep(1000);
    bissc_command_process(priv);
    ClockP_usleep(1000);
    for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
    {
        ch = priv->channel[ch_num];
        priv->enc_ctrl_data[ch].cmd_result  = pruicss_xchg->ctrl_res[ch].ctrl_cds_res;
        priv->enc_ctrl_data[ch].cmd_rcv_crc = pruicss_xchg->ctrl_res[ch].ctrl_rcvd_crc;
        priv->enc_ctrl_data[ch].cmd_otf_crc = pruicss_xchg->ctrl_res[ch].ctrl_otf_crc;
        priv->ctrl_crc_err_cnt[ch] = pruicss_xchg->ctrl_res[ch].ctrl_crc_err_cnt;
    }
    return SystemP_SUCCESS;
}
