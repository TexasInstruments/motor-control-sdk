/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <position_sense/nikon/include/nikon_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
static struct nikon_priv nikon_priv;

void nikon_command_send(struct nikon_priv *priv)
{
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    if(priv->load_share)
    {
        pruicss_xchg->cycle_trigger[0] = (pruicss_xchg->channel & NIKON_CHANNEL0_MASK)?NIKON_ENABLE_CYCLE_TRIGGER:NIKON_DISABLE_CYCLE_TRIGGER;
        pruicss_xchg->cycle_trigger[1] = (pruicss_xchg->channel & NIKON_CHANNEL1_MASK)?NIKON_ENABLE_CYCLE_TRIGGER:NIKON_DISABLE_CYCLE_TRIGGER;
        pruicss_xchg->cycle_trigger[2] = (pruicss_xchg->channel & NIKON_CHANNEL2_MASK)?NIKON_ENABLE_CYCLE_TRIGGER:NIKON_DISABLE_CYCLE_TRIGGER;
    }
    else
    {
        pruicss_xchg->cycle_trigger[0] = NIKON_ENABLE_CYCLE_TRIGGER;
    }
}

void nikon_update_enc_len(struct nikon_priv *priv, uint32_t num_encoders, uint32_t single_turn_len[], uint32_t multi_turn_len[], uint32_t ch)
{
    int32_t enc_num;
    int32_t ls_ch = 0;
    if(priv->load_share)
    {
        ls_ch = ch;
    }
    priv->num_encoders[ls_ch] = num_encoders;
    for(enc_num = 0; enc_num < num_encoders; enc_num++)
    {
        priv->single_turn_len[ch][enc_num] = single_turn_len[enc_num];
        priv->multi_turn_len[ch][enc_num] = multi_turn_len[enc_num];
        priv->data_len[ch][enc_num] = priv->single_turn_len[ch][enc_num] + priv->multi_turn_len[ch][enc_num];
    }
}

int32_t nikon_reverse_bits(uint64_t bits, int32_t num_bits)
{
    int32_t temp;
    int32_t i;
    uint64_t res = 0;
    for(i = 0; i < num_bits; i++)
    {
        temp = bits & 1;
        res = res | (temp << ((num_bits - 1) - i));
        bits = bits >> 1;
    }
    return res;
}

uint32_t nikon_get_current_channel(struct nikon_priv *priv, uint32_t ch_idx)
{
    return priv->channel[ch_idx];
}

uint32_t nikon_get_totalchannels(struct nikon_priv *priv)
{
    return priv->totalchannels;
}

void nikon_update_enc_addr(struct nikon_priv *priv, uint32_t enc_addr, uint32_t ls_ch)
{
    priv->eax[ls_ch] = nikon_reverse_bits(enc_addr, NIKON_ENC_ADDR_LEN);
}

void nikon_update_eeprom_addr(struct nikon_priv *priv, uint32_t addr)
{
    priv->mem_data[2] = addr;
}

void nikon_update_eeprom_data(struct nikon_priv *priv, uint32_t data_high, uint32_t data_low)
{
    priv->mem_data[0] = data_high;
    priv->mem_data[1] = data_low;
}

void nikon_update_id_code(struct nikon_priv *priv, uint32_t data_high, uint32_t data_mid, uint32_t data_low)
{
    priv->mem_data[0] = data_high;
    priv->mem_data[1] = data_mid;
    priv->mem_data[2] = data_low;
}

static uint32_t nikon_calc_3bitcrc(struct nikon_priv *priv, uint32_t cmd)
{
    int8_t ff0 = 0;
    int8_t ff1 = 0;
    int8_t ff2 = 0;
    int8_t crc;
    uint32_t res;
    uint32_t msb;
    uint32_t ex;
    uint32_t i;
    res = pow(2, 32) - 1;               /*Load 32 1's */
    res = res << NIKON_START_BIT_LEN;
    res = (res << NIKON_SYNC_CODE_LEN) | priv->sync_code;
    /*3bit crc should be calculated for fc + ea + command code or fc + memData/memAddress*/
    res = (res << (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN)) | cmd;
    msb = pow(2, (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN +NIKON_COMMAND_CODE_LEN -1));
    for(i = 0; i < (NIKON_FRAME_CODE_LEN + NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN); i++)
    {
        if(cmd & msb)          /*Check for the MSB(9th in this case)*/
        {
            ex = ff2 ^ 1;
        }
        else
        {
            ex = ff2 ^ 0;
        }
        ff2 = ff1;
        ff1 = ff0 ^ ex;            /*3 bit CRC algorithm*/
        ff0 = ex;
        cmd = cmd << 1;
    }
    crc = ff2 << 2 | ff1 << 1 | ff0;
    res = res << NIKON_TX_CRC_LEN | crc;
    res = res << NIKON_STOP_BIT_LEN | 0x1;
    res = (res << 2) | 0x3;
    return res;
}

void nikon_generate_cdf(struct nikon_priv *priv, int32_t cmd)
{
    /* padding of 12 1's has be provided at the beggining to compensate for 2micro sec t2 delay and 2 1's t3 delay compensation
        start bit = 0, fc = 00, encoder adress = EA[0:2], command code = CC[0:4], crc = CRC[2:0], stop bit = 1*/
    int32_t cdf_cmd;
    int32_t pru_num;
    int32_t ls_ch;
    uint32_t res = 0;
    for(pru_num = 0; pru_num < priv->totalchannels; pru_num++)
    {
        if(priv->load_share)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = priv->totalchannels;
        }
        res = cmd & NIKON_CMD_CODE_MASK;
        res = nikon_reverse_bits(res, NIKON_COMMAND_CODE_LEN);
        cdf_cmd = priv->fc;
        cdf_cmd = cdf_cmd << NIKON_ENC_ADDR_LEN | priv->eax[ls_ch];
        cdf_cmd = cdf_cmd << NIKON_COMMAND_CODE_LEN | res;
        priv->tx_cdf[ls_ch] = nikon_calc_3bitcrc(priv, cdf_cmd);
    }
}

static void nikon_generate_mdf(struct nikon_priv *priv, int32_t mem_idx)
{
    /*  padding of 12 1's has be provided at the beggining to compensate for 2micro sec t2 delay 
        and 2 1's t3 delay compensation start bit = 0, fc = FC[0:1], memory adress or 
        memory data = MEM[0:7], crc = CRC[2:0], stop bit = 1
    */
    int32_t mdf_cmd;
    mdf_cmd = priv->fc;
    mdf_cmd = mdf_cmd << (NIKON_ENC_ADDR_LEN + NIKON_COMMAND_CODE_LEN) | priv->mem_data[mem_idx];
    priv->tx_mdf = nikon_calc_3bitcrc(priv, mdf_cmd);
    priv->fc = 0;
}

int32_t nikon_command_wait(struct nikon_priv *priv)
{
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    uint32_t timeout;
    uint32_t pru_num;
    /*  Minimum and Maximum NIKON cycle time depends on various params as below:
        TCycle_max = TMA ∗ (number of RX frames * 18) + TX frame(32) + delay between TX and RX + 
        (delay between Multi Transmission commands * (maximum encoder address delay t6))
        + Delay between two EEPROM access commands(30 milisec)
        TCycle_min = TMA * (number of Rx frames * 18) + TX frame(32) + delay between TX and RX
        Instead wait for max of 35 ms as this can vary for different encoders, different commands
        and multi transmission connection.
    */
    timeout = NIKON_MAX_CYCLE_TIMEOUT;
    while(1)
    {
        if(priv->load_share)
        {
            if((pruicss_xchg->cycle_trigger[0] == NIKON_DISABLE_CYCLE_TRIGGER ) && 
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
            ClockP_usleep(NIKON_1MILLISEC_SLEEP_TIME);
            timeout--;
            if(timeout == 0)
            {
                return SystemP_FAILURE;
            }
        }
    }
    if(priv->load_share)
    {
        for(pru_num = 0; pru_num < NUM_ED_CH_MAX; pru_num++)
        {
            pruicss_xchg->pru_sync_status[pru_num] = NIKON_CLEAR_STATUS_FLAG;
        }
    }
    return SystemP_SUCCESS;
}

static int32_t nikon_command_process(struct nikon_priv *priv)
{
    int32_t ret = SystemP_FAILURE;
    nikon_command_send(priv);
    ret = nikon_command_wait(priv);
    return ret;
}

void nikon_config_periodic_trigger(struct nikon_priv *priv)
{
    /* Configures Nikon in periodic trigger mode */
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    int8_t pru_num;
    pruicss_xchg->num_rx_frames = NIKON_MAX_NUM_RX_FRAMES;
    if(priv->load_share)
    {
        for(pru_num = 0; pru_num < NUM_ED_CH_MAX; pru_num++)
        {
            pruicss_xchg->cdf_frame[pru_num] = priv->tx_cdf[pru_num];
            pruicss_xchg->num_encoders[pru_num] = priv->num_encoders[pru_num];
            pruicss_xchg->rx_frame_size[pru_num] = (pruicss_xchg->num_encoders[pru_num] == 1) 
                ? (priv->num_rx_frames * (NIKON_RX_ONE_FRAME_LEN + NIKON_START_BIT_LEN + NIKON_STOP_BIT_LEN)) : 0xfff;
            pruicss_xchg->opmode[pru_num] = NIKON_CONFIG_PERIODIC_TRIGGER_MODE;
        }
    }
    else
    {
        pruicss_xchg->cdf_frame[0] = priv->tx_cdf[0];
        pruicss_xchg->num_encoders[0] = priv->num_encoders[0];
        pruicss_xchg->rx_frame_size[0] = (pruicss_xchg->num_encoders[0] == 1) 
            ? (priv->num_rx_frames * (NIKON_RX_ONE_FRAME_LEN + NIKON_START_BIT_LEN + NIKON_STOP_BIT_LEN)) : 0xfff;
        pruicss_xchg->opmode[0] = NIKON_CONFIG_PERIODIC_TRIGGER_MODE;
    }
    priv->is_continuous_mode = NIKON_SET_STATUS_FLAG;
}

void nikon_config_host_trigger(struct nikon_priv *priv)
{
    /* Configures Nikon receiver in host trigger mode */
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    int8_t pru_num;
    if(priv->load_share)
    {
        for(pru_num = 0; pru_num < NUM_ED_CH_MAX; pru_num++)
        {
            pruicss_xchg->opmode[pru_num] = NIKON_CONFIG_HOST_TRIGGER_MODE;
        }
    }
    else
    {
        pruicss_xchg->opmode[0] = NIKON_CONFIG_HOST_TRIGGER_MODE;
    }
    priv->is_continuous_mode = NIKON_CLEAR_STATUS_FLAG;
}

static void nikon_config_clr_cfg0(struct nikon_priv *priv)
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

static void nikon_config_clock(struct nikon_priv *priv,
                        struct nikon_clk_cfg *clk_cfg)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    /* Set PRU1_ED_RX_SB_POL polarity bit to 0 for nikon, required for ICSSG (don't care for ICSSM) */
    if(priv->pruicss_slicex)
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, 
            ((clk_cfg->rx_div << CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_SHIFT) |
            (clk_cfg->is_core_clk << CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_CLK_SEL_SHIFT) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, 
            clk_cfg->tx_div << CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_SHIFT |
            (clk_cfg->is_core_clk << CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_SHIFT));
    }
    else
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0RXCFGREGISTER, 
            ((clk_cfg->rx_div << CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_SHIFT) |
            (clk_cfg->is_core_clk << CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_CLK_SEL_SHIFT) | (clk_cfg->rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, 
            clk_cfg->tx_div << CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_SHIFT |
            (clk_cfg->is_core_clk << CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_SHIFT));
    }
}

static int32_t nikon_calc_clock(struct nikon_priv *priv, struct nikon_clk_cfg *clk_cfg)
{
    double freq = priv->baud_rate;
    clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE;
    if(freq == NIKON_FREQ_2_5MHZ)
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->core_clk_freq / freq) - 1;
        clk_cfg->rx_div = (priv->core_clk_freq / (freq * 8)) - 1;
        clk_cfg->is_core_clk = NIKON_SET_STATUS_FLAG;
        priv->pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_8X_OS;   /* Middle bit for 8x oversampling */
        priv->pruicss_xchg->multi_transmission_delay = ((freq*3)/1000000);
    }
    else if((freq == NIKON_FREQ_4MHZ) || (freq == NIKON_FREQ_8MHZ))
    {
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->uart_clk_freq / freq) - 1;
        clk_cfg->rx_div = (priv->uart_clk_freq / (freq * 8)) - 1;
        clk_cfg->is_core_clk = NIKON_CLEAR_STATUS_FLAG;
        priv->pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_8X_OS;   /* Middle bit for 8x oversampling */
        priv->pruicss_xchg->multi_transmission_delay = (freq == NIKON_FREQ_8MHZ) ? ((freq*1.5)/1000000) : ((freq*2)/1000000);
    }
    else if(freq == NIKON_FREQ_16MHZ)
    {
        clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_16MHZ;
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->uart_clk_freq / freq) - 1;
        clk_cfg->rx_div = (priv->uart_clk_freq / (freq * 4)) - 1;
        clk_cfg->is_core_clk = NIKON_CLEAR_STATUS_FLAG;
        priv->pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_4X_OS;    /* Middle bit for 4x oversampling */
        priv->pruicss_xchg->multi_transmission_delay = ((freq*1.5)/1000000);
    }
    else if(((uint8_t)freq % NIKON_FREQ_6_67MHZ) < 1)
    {
        clk_cfg->rx_div_attr = NIKON_RX_SAMPLE_SIZE_6_67MHZ;
        freq = freq * 1000 * 1000;
        clk_cfg->tx_div = (priv->core_clk_freq / freq) - 1;
        clk_cfg->rx_div = (priv->core_clk_freq / (freq * 6)) - 1;
        clk_cfg->is_core_clk = NIKON_SET_STATUS_FLAG;
        priv->pruicss_xchg->fifo_bit_idx = NIKON_FIFO_BIT_IDX_6X_OS;   /* Middle bit for 6x oversampling */
        priv->pruicss_xchg->multi_transmission_delay = ((priv->core_clk_freq*2)/1000000);
    }
    else
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

static void nikon_enable_load_share_mode(struct nikon_priv *priv)
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

static void nikon_config_primary_core_mask(struct nikon_priv *priv, uint8_t mask)
{
    switch (mask)
    {
        case 1: /*only channel0 connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL0_MASK;
            break;
        case 2: /*channel1 connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL1_MASK;
            break;
        case 3: /*channel0 and channel1 connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL0_MASK;
            break;
        case 4: /*channel2 connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 5: /*channel0 and channel2 connnected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 6: /*channel1 and channel2 connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
        case 7: /*all three channel connected*/
            priv->pruicss_xchg->primary_core_mask = NIKON_CHANNEL2_MASK;
            break;
    }
}

static void nikon_hw_init(struct nikon_priv *priv)
{
    struct nikon_clk_cfg clk_cfg;
    nikon_calc_clock(priv, &clk_cfg);
    nikon_config_clock(priv, &clk_cfg);
    nikon_config_clr_cfg0(priv);
}

void nikon_update_clock_freq(struct nikon_priv *priv, float_t frequency)
{
    if(((uint8_t)frequency % NIKON_FREQ_6_67MHZ) < 1)
    {
        frequency = (float_t)20/3;
    }
    priv->baud_rate = frequency;
    priv->pruicss_xchg->rx_clk_freq = priv->baud_rate;
    nikon_hw_init(priv);
    if(priv->load_share)
    {
        nikon_enable_load_share_mode(priv);
    }
}

static void nikon_set_default_initialization(struct nikon_priv *priv, uint64_t icssgclk)
{
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    int8_t pru_num;
    /* Initialize parameters to default values */
    pruicss_xchg->pos_crc_len         = NIKON_POS_CRC_LEN;
    pruicss_xchg->rx_clk_freq         = priv->baud_rate;
    pruicss_xchg->delay_10us          = ((icssgclk*10) / 1000000);
    pruicss_xchg->delay_300us         = ((icssgclk*300) / 1000000);
    pruicss_xchg->delay_30ms          = ((icssgclk*3) / 100); /*((icssgclk*30000) / 1000000)*/
    pruicss_xchg->icssg_clk           = icssgclk;
    pruicss_xchg->valid_bit_idx       = NIKON_BASE_VALID_BIT_IDX;
    for(pru_num = 0; pru_num < NUM_ED_CH_MAX; pru_num++)
    {
        priv->eax[pru_num]                         = 0;                  /*Default encoder address assumed as 0*/
        pruicss_xchg->pru_sync_status[pru_num]     = NIKON_CLEAR_STATUS_FLAG;
        pruicss_xchg->opmode[pru_num]              = NIKON_CONFIG_HOST_TRIGGER_MODE;
    }
    priv->is_continuous_mode          = NIKON_CLEAR_STATUS_FLAG;
    priv->cmp3                        = ((icssgclk*100) / 1000000);   /* 100usec delay */
    priv->sync_code                   = 2;                  /*Syn code for Rx is 010*/
    priv->fc                          = 0;                  /*Frame code for CDF is 00*/
}

static void nikon_config_channel(struct nikon_priv *priv, int32_t mask, int32_t totalch)
{
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
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

struct nikon_priv *nikon_init(PRUICSS_Handle gPruIcssXHandle,
                              int32_t slice,
                              float_t frequency,
                              uint32_t core_clk_freq,
                              uint32_t uart_clk_freq,
                              uint32_t mask,
                              uint32_t totalch)
{
    struct nikon_pruicss_xchg *pruicss_xchg;
    void *pruicss_cfg;
    void *pruicss_iep;
    pruicss_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    if(slice)
    {
        pruicss_xchg =  (struct nikon_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru1DramBase;
    }
    else
    {
        pruicss_xchg =  (struct nikon_pruicss_xchg *)((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru0DramBase;
    }
    nikon_priv.pruicss_xchg = pruicss_xchg;
    nikon_priv.pruicss_cfg = pruicss_cfg;
    nikon_priv.pruicss_iep = pruicss_iep;
    nikon_priv.pruicss_slicex = slice;
    nikon_priv.baud_rate = frequency;
    nikon_priv.core_clk_freq = core_clk_freq;
    nikon_priv.uart_clk_freq = uart_clk_freq;
    PRUICSS_setGpMuxSelect(gPruIcssXHandle, slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);
    nikon_hw_init(&nikon_priv);
    nikon_config_channel(&nikon_priv, mask, totalch);
    nikon_set_default_initialization(&nikon_priv, core_clk_freq);
    return &nikon_priv;
}

void nikon_config_load_share(struct nikon_priv *priv, int32_t mask)
{
    priv->load_share = NIKON_SET_STATUS_FLAG; /*Enable load-share*/
    nikon_config_primary_core_mask(priv, mask);
    nikon_enable_load_share_mode(priv);
}

static void nikon_get_alm_bits(struct nikon_priv *priv, int32_t enc_num, int32_t ch)
{
    int32_t alm;
    alm = priv->alm_field[ch][enc_num]; /*discard reserved bits*/
    priv->alm_bits[ch][enc_num].inc_err = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ov_temp = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].mem_busy = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].busy = alm & 1;
    alm = alm >> 1;
    priv->alm_bits[ch][enc_num].ps_err = alm & 1;
    alm =alm >> 1;
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

int32_t nikon_wait_for_encoder_detection(struct nikon_priv *priv)
{
    int32_t pru_num;
    int32_t ls_ch;
    for(pru_num = 0; pru_num < priv->totalchannels; pru_num++)
    {
        if(priv->load_share)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = priv->totalchannels;
        }
        priv->eax[ls_ch] = NIKON_ENC_ADDR_MASK;
        priv->pruicss_xchg->num_encoders[ls_ch] = priv->num_encoders[ls_ch];
        priv->pruicss_xchg->rx_frame_size[ls_ch] = 0xfff;
    }
    priv->pruicss_xchg->num_rx_frames = NIKON_MAX_NUM_RX_FRAMES;
    nikon_generate_cdf(priv, CMD_4);
    for(pru_num = 0; pru_num < NUM_ED_CH_MAX; pru_num++)
    {
        priv->pruicss_xchg->cdf_frame[pru_num] = priv->tx_cdf[pru_num];
        priv->eax[pru_num] = 0;
    }
    if(nikon_command_process(priv) < 0)
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

int32_t nikon_get_pos(struct nikon_priv *priv, int8_t cmd)
{
    struct nikon_pruicss_xchg *pruicss_xchg = priv->pruicss_xchg;
    uint64_t max;
    uint32_t multiturn_mask;
    uint32_t enc_num;
    uint32_t ls_ch;
    uint32_t ch;
    uint32_t ch_num;
    uint32_t loop_cnt;
    uint32_t pru_num;
    uint32_t mdf_num;

    ch = priv->channel[0];

    if((cmd == CMD_0) || (cmd == CMD_4) || ((cmd >= CMD_27) && (cmd <= CMD_30)))
    {
        priv->num_rx_frames = NIKON_MAX_NUM_RX_FRAMES;
    }
    else if((cmd == CMD_21) || (cmd == CMD_22))
    {
       priv->num_rx_frames = NIKON_MIN_NUM_RX_FRAMES;
    }
    else if(cmd == CMD_13)
    {
        pruicss_xchg->is_memory_access = NIKON_EEPROM_READ_ACCESS;
        priv->num_rx_frames = NIKON_AVG_NUM_RX_FRAMES;
        priv->mem_data[2] = nikon_reverse_bits(priv->mem_data[2], NIKON_EEPROM_ADDR_LEN);
        priv->fc = 3;                           /* MDF2 for memory address */
        nikon_generate_mdf(priv, 2);
        priv->pruicss_xchg->mdf_frame[2] = priv->tx_mdf;
    }
    else if((cmd == CMD_14) || (cmd == CMD_18) || (cmd == CMD_19) || (cmd == CMD_20))
    {
        if(cmd == CMD_14)
        {
            priv->mem_data[2] = nikon_reverse_bits(priv->mem_data[2], NIKON_EEPROM_ADDR_LEN);
        }
        pruicss_xchg->is_memory_access = NIKON_EEPROM_WRITE_ACCESS;
        priv->num_rx_frames = NIKON_AVG_NUM_RX_FRAMES;
        for(mdf_num = 0; mdf_num < NUM_MDF_CMD_MAX; mdf_num++)
        {
            priv->fc = nikon_reverse_bits(mdf_num + 1 , NIKON_FRAME_CODE_LEN);
            nikon_generate_mdf(priv, mdf_num);
            priv->pruicss_xchg->mdf_frame[mdf_num] = priv->tx_mdf;
        }
    }
    else
    {
        priv->num_rx_frames = NIKON_AVG_NUM_RX_FRAMES;
    }
    pruicss_xchg->num_rx_frames = priv->num_rx_frames;
    for(pru_num = 0; pru_num < priv->totalchannels; pru_num++)
    {
        if(priv->load_share)
        {
            ls_ch = priv->channel[pru_num];
        }
        else
        {
            ls_ch = 0;
            pru_num = priv->totalchannels;
        }
        priv->pruicss_xchg->num_encoders[ls_ch] = ((cmd == CMD_4) || (cmd == CMD_5) || (cmd == CMD_6) || (cmd == CMD_7) || (cmd == CMD_22) || (cmd == CMD_28) || (cmd == CMD_30)) ? priv->num_encoders[ls_ch] : 1;
        priv->num_enc_access[ls_ch] = priv->pruicss_xchg->num_encoders[ls_ch];
        pruicss_xchg->cdf_frame[ls_ch] = priv->tx_cdf[ls_ch];
        pruicss_xchg->rx_frame_size[ls_ch] = (pruicss_xchg->num_encoders[ls_ch] == 1) ? (priv->num_rx_frames * (NIKON_RX_ONE_FRAME_LEN + NIKON_START_BIT_LEN + NIKON_STOP_BIT_LEN)) : 0xfff;
    }
    if((cmd >= CMD_8) && (cmd <=CMD_12))
    {
        loop_cnt = NIKON_NUM_OF_CYCLE_FOR_RESET;
        do
        {
            if(nikon_command_process(priv) < 0)
            {
                return SystemP_FAILURE;
            }
        }while (loop_cnt--);
        ClockP_usleep(NIKON_30_MILLI_SEC_DELAY);   /* 30msec delay after 8th cycle to print the changes occured*/
    }
    if(nikon_command_process(priv) < 0)
    {
        return SystemP_FAILURE;
    }
    for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
    {
        ch = priv->channel[ch_num];
        ls_ch = 0;
        if(priv->load_share)
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
                    max = pow(2, priv->abs_len);
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch] >> NIKON_POS_CRC_LEN;
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].abs[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | priv->pos_data_info[ch].raw_data3[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    multiturn_mask = pow(2, (priv->abs_len - priv->single_turn_len[ch][enc_num])) - 1;
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] & multiturn_mask;
                    priv->pos_data_info[ch].multi_turn[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].multi_turn[enc_num], (priv->abs_len - priv->single_turn_len[ch][enc_num]));
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> (priv->abs_len - priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->single_turn_len[ch][enc_num]);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    break;

                case CMD_1:
                case CMD_2:
                case CMD_5:
                case CMD_6:
                    priv->abs_len   = NIKON_AVG_ABS_LEN;
                    max = pow(2, priv->abs_len);
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN;
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
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
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN;
                    priv->alm_field[ch][enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->alm_field[ch][enc_num] = priv->alm_field[ch][enc_num] >> 13;
                    nikon_get_alm_bits(priv, enc_num, ch);                           /*ALM[0:10] are status flags, ALM[11:15] are reserved and ALM[16:23] are '0' */
                    break;

                case CMD_13:
                case CMD_14:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN;
                    priv->pos_data_info[ch].raw_data2[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].raw_data2[enc_num], NIKON_EEPROM_ADDR_LEN);
                    if(priv->pos_data_info[ch].raw_data2[enc_num] == 0xF9)                             /*At EEPROM address F9 DTB[0:7] (temperature) is encoded */
                    {
                        priv->temperature[ch][enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] >> 6) & 0xFF;  /* 8bit temperature information */
                        priv->temperature[ch][enc_num] = nikon_reverse_bits(priv->temperature[ch][enc_num], 8);
                        priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num];
                    }
                    break;

                case CMD_15:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN;
                    priv->temperature[ch][enc_num]             = (priv->pos_data_info[ch].raw_data1[enc_num] >> (NIKON_RX_ONE_FRAME_LEN - NIKON_DB_BITS_LEN)) & NIKON_DB_BITS_MASK;
                    priv->temperature[ch][enc_num]             = nikon_reverse_bits(priv->temperature[ch][enc_num], NIKON_DB_BITS_LEN);
                    priv->temperature[ch][enc_num]             = priv->temperature[ch][enc_num] * 0.25;
                    break;

                case CMD_16:
                case CMD_17:
                case CMD_18:
                case CMD_19:
                case CMD_20:
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->identification_code[ch]              = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->identification_code[ch]              = (priv->identification_code[ch] << (NIKON_RX_ONE_FRAME_LEN - NIKON_POS_CRC_LEN)) | (pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch] >> NIKON_POS_CRC_LEN);
                    break;

                case CMD_21:
                case CMD_22:
                    priv->abs_len                              = NIKON_MIN_ABS_LEN;
                    max                                        = pow(2, priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num]       = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch] & 0x1FF;          /*ABS[0:8] is encoded in infofield for command 21 and 22*/
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch] >> 9;       /*Discard abs bits*/
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch] >> NIKON_POS_CRC_LEN;
                    priv->pos_data_info[ch].abs[enc_num]       = priv->pos_data_info[ch].raw_data1[enc_num] | (priv->pos_data_info[ch].abs[enc_num] << 8);
                    priv->pos_data_info[ch].abs[enc_num]       = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num]       = priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    break;

                case CMD_27:
                case CMD_28:
                    priv->abs_len                              = NIKON_AVG_ABS_LEN;
                    max                                        = pow(2, priv->abs_len);
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch] >> NIKON_POS_CRC_LEN;
                    priv->alm_field[ch][enc_num] = ((priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 8 ) | (priv->pos_data_info[ch].raw_data3[enc_num]);
                    priv->alm_field[ch][enc_num] = priv->alm_field[ch][enc_num] >> 5;                   /*ALM[0:10] are status flags and ALM[11:15] are reserved*/
                    nikon_get_alm_bits(priv, enc_num, ch);
                    priv->pos_data_info[ch].abs[enc_num] = (priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN ) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> 8;
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    break;

                case CMD_29:
                case CMD_30:
                    priv->abs_len                              = NIKON_AVG_ABS_LEN;
                    max                                        = pow(2, priv->abs_len);
                    priv->pos_data_info[ch].raw_data0[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.info_field[ch];
                    priv->pos_data_info[ch].raw_data1[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[0][ch];
                    priv->pos_data_info[ch].raw_data2[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[1][ch];
                    priv->pos_data_info[ch].raw_data3[enc_num] = pruicss_xchg->pos_data_res[enc_num].raw_data.data_field[2][ch] >> NIKON_POS_CRC_LEN;
                    priv->pos_data_info[ch].abs[enc_num] = (uint64_t)(priv->pos_data_info[ch].raw_data1[enc_num] << NIKON_RX_ONE_FRAME_LEN ) | priv->pos_data_info[ch].raw_data2[enc_num];
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> 8;
                    priv->pos_data_info[ch].abs[enc_num] = nikon_reverse_bits(priv->pos_data_info[ch].abs[enc_num], priv->abs_len);
                    priv->pos_data_info[ch].abs[enc_num] = priv->pos_data_info[ch].abs[enc_num] & (max - 1);
                    priv->pos_data_info[ch].multi_turn[enc_num] = priv->pos_data_info[ch].abs[enc_num] >> priv->single_turn_len[ch][enc_num];
                    priv->temperature[ch][enc_num] = (priv->pos_data_info[ch].raw_data2[enc_num] & 0xFF) << 2;
                    priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num] | ((priv->pos_data_info[ch].raw_data3[enc_num] >> 6) & 0x3);
                    priv->temperature[ch][enc_num] = nikon_reverse_bits(priv->temperature[ch][enc_num], NIKON_DB_BITS_LEN);
                    priv->temperature[ch][enc_num] = priv->temperature[ch][enc_num] * 0.25;
                    break;

            default:
                break;
            }
            if((cmd == CMD_21) || (cmd == CMD_22))
            {
                priv->enc_info[ch].enc_status[enc_num] =  priv->pos_data_info[ch].raw_data0[enc_num] & 0x1;
                priv->enc_info[ch].enc_addr[enc_num]   = (priv->pos_data_info[ch].raw_data0[enc_num] >> 1) & NIKON_ENC_ADDR_MASK;
            }
            else
            {
                priv->enc_info[ch].enc_status[enc_num] =  priv->pos_data_info[ch].raw_data0[enc_num] & NIKON_ENC_STATUS_MASK;
                priv->enc_info[ch].enc_cmd[enc_num]    = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_ENC_STATUS_LEN + NIKON_FIXED_BIT_LEN)) & NIKON_CMD_CODE_MASK;
                priv->enc_info[ch].enc_addr[enc_num]   = (priv->pos_data_info[ch].raw_data0[enc_num] >> (NIKON_ENC_STATUS_LEN + NIKON_COMMAND_CODE_LEN + NIKON_FIXED_BIT_LEN)) & NIKON_ENC_ADDR_MASK;
            }
            priv->enc_info[ch].enc_addr[enc_num]  = nikon_reverse_bits(priv->enc_info[ch].enc_addr[enc_num], NIKON_ENC_ADDR_LEN);
            priv->enc_info[ch].enc_cmd[enc_num]   = nikon_reverse_bits(priv->enc_info[ch].enc_cmd[enc_num], NIKON_COMMAND_CODE_LEN);
            max = pow(2, priv->single_turn_len[ch][enc_num]);
            priv->pos_data_info[ch].angle[enc_num] = (float)(priv->pos_data_info[ch].abs[enc_num] & (max - 1))/max * (float)360;
            priv->pos_data_info[ch].rcv_crc[enc_num]          = pruicss_xchg->pos_data_res[enc_num].crc.pos_rcv_crc[ch];
            priv->pos_data_info[ch].otf_crc[enc_num]          = pruicss_xchg->pos_data_res[enc_num].crc.pos_otf_crc[ch];
            priv->pos_data_info[ch].crc_err_cnt[enc_num]      = pruicss_xchg->pos_data_res[enc_num].crc.pd_crc_err_cnt[ch];
        }
    }
    return SystemP_SUCCESS;
}
