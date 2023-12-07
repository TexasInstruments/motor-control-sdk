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
    /*Wait for max of 5 ms*/
    timeout = BISSC_MAX_CYCLE_TIMEOUT;
    while(1)
    {
        if(priv->load_share){
            if((pruicss_xchg->cycle_trigger[0] == 0 ) && (pruicss_xchg->cycle_trigger[1] == 0) && (pruicss_xchg->cycle_trigger[2] == 0)){
                break;
            }
        }
        else if(pruicss_xchg->cycle_trigger[0] == 0)
        {
            break;
        }
        ClockP_usleep(1000);
        timeout--;
        if(timeout == 0)
        {
            return SystemP_FAILURE;
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

int32_t bissc_get_pos_res(struct bissc_priv *priv)
{
    uint32_t   raw_data0, raw_data1, shift, sl_num, max, ch_num, numslaves, ls_ch;
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
            numslaves = priv->num_slaves[ch];
            ls_ch = ch;
        }
        else
        {
            numslaves = priv->num_slaves[0];
            ls_ch = 0;
        }
        for(sl_num = 0; sl_num < numslaves; sl_num++)
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
            (clk_cfg->is_ocp << 4) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, clk_cfg->tx_div << 16 | 
            (clk_cfg->is_ocp << 4));
    }
    else
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0RXCFGREGISTER, ((clk_cfg->rx_div << 16) | 
            (clk_cfg->is_ocp << 4) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, clk_cfg->tx_div << 16 |
                (clk_cfg->is_ocp << 4));
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
    for(ch_num = 0; ch_num < totalch; ch_num++){
        if((mask & 1) && ch_num == 0)
            priv->channel[ch_num] = 0;
        else if((mask & 2) && (ch_num == 0 || ch_num == 1))
            priv->channel[ch_num] = 1;
        else if((mask & 4))
            priv->channel[ch_num] = 2;
    }
}
void bissc_config_load_share(struct bissc_priv *priv, int32_t mask, int32_t loadshare)
{
    priv->load_share = loadshare;
    bissc_config_primary_core_mask(priv, mask);
    bissc_enable_load_share_mode(priv);
}
void bissc_enable_load_share_mode(struct bissc_priv *priv)
{
   void *pruss_cfg = priv->pruicss_cfg;
    //HW_WR_REG32(0x30026104) |= 0x0800;
    uint32_t rgval;
    if(priv->pruicss_slicex)
    {
       rgval = HW_RD_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER);
       rgval |= CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT_SHARE_EN_MASK;
      HW_WR_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER);
        rgval |= CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT_SHARE_EN_MASK;
      HW_WR_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, rgval);
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
int32_t bissc_wait_for_fw_initialization(struct bissc_priv *priv, uint32_t timeout, uint8_t mask)
{
    int32_t i;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    for(i = 0; i < timeout; i++)
    {
         if(priv->load_share)  /* for loadshare mode*/
        {
            switch (mask) {
                case 1:  /*channel 0 connected*/
                    if((pruicss_xchg->status[0] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 2: /*channel 1 connected*/
                    if((pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 3:               /*channel 0 and 1 connected*/
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[1] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 4:  /*channel 2 connected*/
                    if((pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 5:               /*channel 0 and 2 connnected*/
                    if((pruicss_xchg->status[0] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 6:                    /*channel 1 and 2 connected*/
                    if((pruicss_xchg->status[1] & 1) && (pruicss_xchg->status[2] & 1))
                        return SystemP_SUCCESS;
                    break;
                case 7:                       /*all three channel connected*/
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
int32_t bissc_calc_clock(uint32_t freq, struct bissc_clk_cfg *clk_cfg)
{
    clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE;
    if((freq == 1) || (freq == 5))
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (BISSC_INPUT_CLOCK_OCP_FREQUENCY / freq) - 1;
        clk_cfg->rx_div = (BISSC_INPUT_CLOCK_OCP_FREQUENCY / (freq * 8)) - 1;
        clk_cfg->is_ocp = 1;
    }
    else if((freq == 2) || (freq == 8))
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (BISSC_INPUT_CLOCK_FREQUENCY_UART / freq) - 1;
        clk_cfg->rx_div = (BISSC_INPUT_CLOCK_FREQUENCY_UART / (freq * 8)) - 1;
        clk_cfg->is_ocp = 0;
    }
    else if(freq == 10)
    {
        freq = freq* 1000 * 1000;
        clk_cfg->tx_div = (BISSC_INPUT_CLOCK_OCP_FREQUENCY / freq) - 1;
        clk_cfg->rx_div = (BISSC_INPUT_CLOCK_OCP_FREQUENCY / (freq * 4)) - 1;
        clk_cfg->is_ocp = 1;
        clk_cfg->rx_div_attr = BISSC_RX_SAMPLE_SIZE_10MHZ;
    }    
    else 
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

static void bissc_hw_init(struct bissc_priv *priv, uint32_t frequency)
{
    struct bissc_clk_cfg clk_cfg;
    bissc_calc_clock(frequency, &clk_cfg);
    bissc_config_endat_mode(priv);
    bissc_config_clock(priv, &clk_cfg);
    bissc_config_clr_cfg0(priv);
}


struct bissc_priv *bissc_init(PRUICSS_Handle gPruIcssXHandle, int32_t slice, uint32_t frequency)
{
    struct bissc_pruicss_xchg *pruicss_xchg;
    void *pruicss_cfg;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    if(slice)
        pruicss_xchg =  (struct bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru1DramBase;
    else
         pruicss_xchg =  (struct bissc_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru0DramBase;
    bissc_priv.pruicss_xchg = pruicss_xchg;
    bissc_priv.pruicss_cfg = pruicss_cfg;
    bissc_priv.pruicss_slicex = slice;
    bissc_hw_init(&bissc_priv, frequency);
    return &bissc_priv;    
}

void bissc_update_max_proc_delay(struct bissc_priv *priv,  uint32_t frequency) 
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    pruicss_xchg->fifo_bit_idx = 4;                          /* 8x over clock - middle bit*/
    if(frequency == 1)
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_1MHZ;
    else if(frequency == 2)
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_2MHZ;
    else if(frequency == 5)
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_5MHZ;
    else if(frequency == 8)
        pruicss_xchg->max_proc_delay = BISSC_MAX_PROC_DELAY_8MHZ;
    else if(frequency == 10)
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

void bissc_set_default_initialization(struct bissc_priv *priv, uint32_t frequency, uint64_t icssgclk)
{
    int8_t pru_num, totalprus, ls_ch;
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    /* Initialize parameters to default values */
    if(priv->load_share)
        totalprus = priv->totalchannels;
    else
        totalprus = 1;
    for( pru_num = 0; pru_num < totalprus; pru_num++)
    {
        if(priv->load_share)
            ls_ch = priv->channel[pru_num];
        else
            ls_ch = 0;
        pruicss_xchg->enc_len[ls_ch].data_len[0]           = BISSC_POS_DATA_LEN_DEFAULT;
        priv->data_len[ls_ch][0]                           = BISSC_POS_DATA_LEN_DEFAULT;
        priv->single_turn_len[ls_ch][0]                    = BISSC_POS_DATA_LEN_DEFAULT;
        pruicss_xchg->enc_len[ls_ch].num_slaves            = 1;
        priv->num_slaves[ls_ch]                            = 1;
        priv->multi_turn_len[ls_ch][0]                     = 0;
        pruicss_xchg->ctrl_cmd[ls_ch]                      = 0;
    }
    pruicss_xchg->pos_crc_len         = BISSC_POS_CRC_LEN;
    pruicss_xchg->ctrl_cmd_crc_len    = BISSC_CTRL_CMD_CRC_LEN;
    pruicss_xchg->rx_clk_freq         = frequency;
    bissc_update_max_proc_delay(priv, frequency); 
    pruicss_xchg->delay_40us          = ((icssgclk*40)/1000000);
    pruicss_xchg->delay_100ms         = (icssgclk / 10); /*((icssgclk*100000) / 1000000)*/   
    pruicss_xchg->icssg_clk           = icssgclk; 
    pruicss_xchg->valid_bit_idx       = 24;            
    pruicss_xchg->measure_proc_delay  = 1; 
    pruicss_xchg->execution_state[0]  = 0;
    pruicss_xchg->execution_state[1]  = 0;
    pruicss_xchg->execution_state[2]  = 0;
}

void bissc_update_data_len(struct bissc_priv *priv, uint32_t single_turn_len[], uint32_t multi_turn_len[], int32_t pru_num)
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    uint32_t sl_num, ls_ch;
    if(priv->load_share)
        ls_ch = priv->channel[pru_num];
    else
        ls_ch = 0;
    for( sl_num = 0; sl_num < NUM_SLAVE_MAX; sl_num++)
    {
        if(single_turn_len[sl_num])
        {
            priv->single_turn_len[ls_ch][sl_num] =  single_turn_len[sl_num];
            priv->multi_turn_len[ls_ch][sl_num] =  multi_turn_len[sl_num];
            priv->data_len[ls_ch][sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num]; 
            pruicss_xchg->enc_len[ls_ch].data_len[sl_num] = single_turn_len[sl_num] + multi_turn_len[sl_num];
            priv->num_slaves[ls_ch]++;
        }
    }
    pruicss_xchg->enc_len[0].num_slaves = priv->num_slaves[0];
    pruicss_xchg->enc_len[1].num_slaves = priv->num_slaves[1];
    pruicss_xchg->enc_len[2].num_slaves = priv->num_slaves[2];
}

int32_t bissc_set_ctrl_cmd_and_process(struct bissc_priv *priv, uint32_t ctrl_cmd[])
{
    struct bissc_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    pruicss_xchg->ctrl_cmd[0] = ctrl_cmd[0];
    pruicss_xchg->ctrl_cmd[1] = ctrl_cmd[1];
    pruicss_xchg->ctrl_cmd[2] = ctrl_cmd[2];
    int32_t ch = 0, ch_num;
    if(priv->load_share){
        pruicss_xchg->ctrl_cmd_status[0] = (pruicss_xchg->channel & 0x1)?1:0;
        pruicss_xchg->ctrl_cmd_status[1] = (pruicss_xchg->channel & 0x2)?1:0;
        pruicss_xchg->ctrl_cmd_status[2] = (pruicss_xchg->channel & 0x4)?1:0;
       while(pruicss_xchg->ctrl_cmd_status[0] + pruicss_xchg->ctrl_cmd_status[1] + pruicss_xchg->ctrl_cmd_status[2])
        {
        if(bissc_command_process(priv) < 0)
        {
            return SystemP_FAILURE;
        }
        ClockP_usleep(1000);
        }
    }
    else{
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
