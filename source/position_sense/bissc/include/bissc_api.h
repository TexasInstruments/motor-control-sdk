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

#ifndef BISSC_API_H_
#define BISSC_API_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "bissc_drv.h"

/**
 *  \defgroup BISSC_API_MODULE APIs for BiSSC Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for BiSSC encoder communication protocol
 *
 *  @{
 */

/**
 *  \brief      Send the BiSSC command and wait till firmware acknowledges
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 *
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 */
int32_t bissc_command_process(struct bissc_priv *priv);

/**
 *  \brief      Trigger sending the BiSSC command in PRU
 *
 *  \param[in]  priv     cookie returned by \ref bissc_init
 *
 *
 */
void bissc_command_send(struct bissc_priv *priv);

/**
 *  \brief      Wait till PRU finishes BiSSC transaction
 *
 *  \param[in]  priv     cookie returned by \ref bissc_init
 *
 *
 */
int32_t bissc_command_wait(struct bissc_priv *priv);

/**
 *  \brief      Get single cycle BiSS-C position data
 *
 *  \param[in]  priv     cookie returned by \ref bissc_init
 *
 */
int32_t bissc_get_pos(struct bissc_priv *priv);

/**
 *  \brief      Configure EnDat clock
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 */
void bissc_config_clock(struct bissc_priv *priv,
                        struct bissc_clk_cfg *clk_cfg);

/**
 *  \brief      Select channel to be used by BiSSC master
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  mask    channel mask
 *  \param[in]  totalch    total number of channels in use 
 *
 */
void bissc_config_channel(struct bissc_priv *priv, int32_t mask, int32_t totalch);
/**
 *  \brief      Configure the channels to be used by BiSSC master
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  mask    channel mask
 *
 */
void bissc_config_load_share(struct bissc_priv *priv, int32_t mask);
/**
 *  \brief      Enable load share mode for BiSSC master
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *
 */
void bissc_enable_load_share_mode(struct bissc_priv *priv);
/**
 *  \brief      Configure the primary core for load share mode
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  mask    channel mask
 *
 */
void bissc_config_primary_core_mask(struct bissc_priv *priv, uint8_t mask);
/**
 *  \brief      Wait for BiSSC master firmware to initialize
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  timeout timeout to wait for initialization
 *  \param[in]  mask    channel mask
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 */
int32_t bissc_wait_for_fw_initialization(struct bissc_priv *priv, uint32_t timeout, uint8_t mask);

/**
 *  \brief      Initialize BiSSC hardware interface
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *
 */
void bissc_hw_init(struct bissc_priv *priv);

/**
 *  \brief      Initialize BiSSC firmware interface address and get the pointer
 *              to struct bissc_priv instance
 *
 *  \param[in]  gPruIcssXHandle      BiSSC firmware interface address
 *  \param[in]  slice                ICSS PRU SLICE
 *  \param[in]  frequency            Input frequency
 *  \param[in]  core_clk_freq        Core clock frequency
 *  \param[in]  uart_clk_freq        Uart clock frequency
 *
 *  \retval     priv            pointer to struct bissc_priv instance
 *
 */
struct bissc_priv *bissc_init(PRUICSS_Handle gPruIcssXHandle, 
                              int32_t slice, 
                              uint32_t frequency,
                              uint32_t core_clk_freq,
                              uint32_t uart_clk_freq);

/**
 *  \brief      Update max processing time and bit index to poll in fifo data
 *
 *  \param[in]  priv        cookie returned by \ref bissc_init
 *
 */
void bissc_update_max_proc_delay(struct bissc_priv *priv); 
/**
 *  \brief      Wait for BiSSC master firmware to measure processing time
 *
 *  \param[in]  priv    cookie returned by \ref bissc_init
 *  \param[in]  timeout timeout to wait for initialization
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 */
int32_t bissc_wait_measure_proc_delay(struct bissc_priv *priv, uint32_t timeout);

/**
 *  \brief      Set default configuration parameters for BiSSC Master firmware
 *
 *  \param[in]  priv        cookie returned by \ref bissc_init
 *  \param[in]  icssgclk    ICSSG core clock for firmware reference
 *
 */
void bissc_set_default_initialization(struct bissc_priv *priv, uint64_t icssgclk);
/**
 *  \brief      Update data length with encoder bit width for BiSSC Master firmware
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 *  \param[in]  single_turn_len    Encoder's single turn resolution
 *  \param[in]  multi_turn_len     Encoder's multi turn resolution
 *  \param[in]  num_pru         number(index) of PRU in use
 */
void bissc_update_data_len(struct bissc_priv *priv, uint32_t single_turn_len[], uint32_t multi_turn_len[], int32_t num_pru);

/**
 *  \brief      Set control command and process the ctrl communication read/write
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 *  \param[in]  ctrl_cmd        Hex equivalent of control command 
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 */
int32_t bissc_set_ctrl_cmd_and_process(struct bissc_priv *priv, uint32_t ctrl_cmd[]);
/**
 *  \brief      Configure the master for EnDat mode
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 */
void bissc_config_endat_mode(struct bissc_priv *priv);
/**
 *  \brief      Clear the channel specific frame size cfg registers.
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 */
void bissc_config_clr_cfg0(struct bissc_priv *priv);
/**
 *  \brief      Get measured processing delay of individual channel.
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 */
void bissc_get_enc_proc_delay(struct bissc_priv *priv);
/**
 *  \brief      Calculate Rx and Tx divisors for given frequency.
 *
 *  \param[in]  priv            cookie returned by \ref bissc_init
 *  \param[in]  clk_cfg         pointer to structure containing clock configuration data.
 */
int32_t bissc_calc_clock(struct bissc_priv *priv, struct bissc_clk_cfg *clk_cfg);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
