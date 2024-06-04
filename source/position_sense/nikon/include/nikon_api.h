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

#ifndef NIKON_API_H_
#define NIKON_API_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "nikon_drv.h"

/**
 *  \defgroup NIKON_API_MODULE APIs for Nikon A-Format Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for Nikon A-Format encoder communication protocol
 *
 *  @{
 */


/**
 *  \brief      Wait till PRU finishes Nikon transaction
 *
 *  \param[in]  priv     cookie returned by \ref nikon_init
 *
 *
 */
int32_t nikon_command_wait(struct nikon_priv *priv);

/**
 *  \brief      Get single cycle Nikon position data
 *
 *  \param[in]  priv     cookie returned by \ref nikon_init
 *  \param[in]  cmd      command code specified by user
 *
 */
int32_t nikon_get_pos(struct nikon_priv *priv, int8_t cmd);

/**
 *  \brief      Configure Nikon clock
 *
 *  \param[in]  priv    cookie returned by \ref nikon_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 */
void nikon_config_clock(struct nikon_priv *priv,
                        struct nikon_clk_cfg *clk_cfg);

/**
 *  \brief      Initialize Nikon firmware interface address and get the pointer
 *              to struct nikon_priv instance
 *
 *  \param[in]  gPruIcssXHandle      Nikon firmware interface address
 *  \param[in]  slice                ICSS PRU SLICE
 *  \param[in]  frequency            Input frequency
 *  \param[in]  core_clk_freq        Core clock frequency
 *  \param[in]  uart_clk_freq        Uart clock frequency
 *  \param[in]  mask                 Mask for the selected channels
 *  \param[in]  totalch              Total number of selected channels
 *  \retval     priv            pointer to struct nikon_priv instance
 *
 */
struct nikon_priv *nikon_init(PRUICSS_Handle gPruIcssXHandle,
                              int32_t slice,
                              float_t frequency,
                              uint32_t core_clk_freq,
                              uint32_t uart_clk_freq,
                              uint32_t mask,
                              uint32_t totalch);

/**
 *  \brief      Clear the channel specific frame size cfg registers.
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 */
void nikon_config_clr_cfg0(struct nikon_priv *priv);
/**
 *  \brief      Calculate Rx and Tx divisors for given frequency.
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  clk_cfg         pointer to structure containing clock configuration data.
 */
int32_t nikon_calc_clock(struct nikon_priv *priv, struct nikon_clk_cfg *clk_cfg);
/**
 *  \brief      Generate CDF command to be passed in Tx fifo.
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  cmd             command code requested by the user.
 */
void nikon_generate_cdf(struct nikon_priv *priv, int32_t cmd);

/**
 *  \brief      Reverse the bits (LSB to be sent out first) provided as parameters.
 *
 *  \param[in]  bits        field to be reversed as specified by the user.
 *  \param[in]  num_bits    width of field to be reversed.
 *
 *  \retval     res         return the reversed field.
 */
int32_t nikon_reverse_bits(uint64_t bits, int32_t num_bits);

/**
 *  \brief      Calculate 3 bit otf crc and generate tx command.
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  cmd             cdf or mdf command to be included in 3 bit crc calculation.
 */
uint32_t nikon_calc_3bitcrc(struct nikon_priv *priv, uint32_t cmd);
/**
 *  \brief      Configure Load Share mode for Nikon receiver
 *
 *  \param[in]  priv    cookie returned by \ref nikon_init
 *  \param[in]  mask    channel mask
 *
 */
void nikon_config_load_share(struct nikon_priv *priv, int32_t mask);

/**
 *  \brief      Configure periodic trigger operation mode
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 */
void nikon_config_periodic_trigger(struct nikon_priv *priv);
/**
 *  \brief      Configure host trigger operation mode
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 */
void nikon_config_host_trigger(struct nikon_priv *priv);
/**
 *  \brief      Detect whether the connected encoder is of selected frequency
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 */
int32_t nikon_wait_for_encoder_detection(struct nikon_priv *priv);
/**
 *  \brief      Update number of encoders connected in bus or one-to-one and
 *              update data lengths of those encoders
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  num_encoders    number of encoders connected to each channel
 *  \param[in]  single_turn_len    Encoder's single turn resolution
 *  \param[in]  multi_turn_len     Encoder's multi turn resolution
 *  \param[in]  ch              channel in use
 */
void nikon_update_enc_len(struct nikon_priv *priv,
                          uint32_t num_encoders,
                          uint32_t single_turn_len[],
                          uint32_t multi_turn_len[],
                          uint32_t ch);
/**
 *  \brief      Update the operating baud rate as user speciefied baud rate
 *              through UART menu
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  frequency       frequency as specified by the user
 */
void nikon_update_clock_freq(struct nikon_priv *priv, float_t frequency);
/**
 *  \brief      Retrives the current channel in use
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  ch_idx          index of current channel to be selected
 *  \retval     channel[ch_idx] current channel in use
 */
uint32_t nikon_get_current_channel(struct nikon_priv *priv, uint32_t ch_idx);
/**
 *  \brief      Retrives total number of channels configured
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \retval     totalchannels   total number of channels configured
 */
uint32_t nikon_get_totalchannels(struct nikon_priv *priv);
/**
 *  \brief      Update encoder address as specified by the user
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  enc_addr        encoder address specified by the user
 *  \param[in]  ls_ch           channel in use in load share
 */
void nikon_update_enc_addr(struct nikon_priv *priv, uint32_t enc_addr, uint32_t ls_ch);
/**
 *  \brief      Access encoder's EEPROM location specified by the user
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  addr            EEPROM address specified by the user
 */
void nikon_update_eeprom_addr(struct nikon_priv *priv, uint32_t addr);
/**
 *  \brief      Write data at specified EEPROM location
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  data_high       upper byte of data to write at EEPROM location
 *  \param[in]  data_low        lower byte of data to write at EEPROM location
 */
void nikon_update_eeprom_data(struct nikon_priv *priv, uint32_t data_high, uint32_t data_low);
/**
 *  \brief      Assign the specified 24bits as Identification code of encoder
 *
 *  \param[in]  priv            cookie returned by \ref nikon_init
 *  \param[in]  data_high       Upper byte to assign as ID code[23 : 16]
 *  \param[in]  data_mid        Middle byte to assign as ID code[15 : 8]
 *  \param[in]  data_low        Lower byte to assign as ID code[7 : 0]
 */
void nikon_update_id_code(struct nikon_priv *priv, uint32_t data_high, uint32_t data_mid, uint32_t data_low);
/** @} */
#ifdef __cplusplus
}
#endif

#endif
