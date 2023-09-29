/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#ifndef ENDAT_API_H_
#define ENDAT_API_H_

#ifdef __cplusplus
extern "C" {
#endif

 /**
 * \defgroup POSITION_SENSE_API APIs for Position Sense
 *
 * This module contains APIs for device drivers for various position sense encoders supported in this SDK.
 */

/**
 *  \defgroup ENDAT_API_MODULE APIs for ENDAT Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for EnDAT encoder communication protocol
 *
 *  @{
 */

/**
 *  \brief      process raw recieved data and format based on the command
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[out] u       pointer to union for storing formatted data based on cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_recvd_process(struct endat_priv *priv, int32_t cmd,
                        union endat_format_data *u);

/**
 *  \brief      CRC result indicated in return value as follows,             <br>
 *              0th bit high    position/address/params/test CRC success     <br>
 *              0th bit low     position/address/params/test CRC failure     <br>
 *              1st bit high    if addinfo 1 present, addinfo 1 CRC success  <br>
 *              1st bit low     if addinfo 1 present, addinfo 1 CRC failure  <br>
 *              2nd bit high    if addinfo 2 present, addinfo 2 CRC success  <br>
 *              2nd bit low     if addinfo 2 present, addinfo 2 CRC failure
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[in]  u       pointer to union having formatted data based on cmd
 *
 *  \retval     status  position/address/params/test CRC status
 *
 */
uint32_t endat_recvd_validate(struct endat_priv *priv, int32_t cmd,
                              union endat_format_data *u);

/**
 *  \brief      send the EnDat command and wait till firmware acknowledges
 *
 *  \param[in]  priv            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_command_process(struct endat_priv *priv, int32_t cmd,
                          struct cmd_supplement *cmd_supplement);

/**
 *  \brief      setup the EnDat command in the PRU interface buffer
 *
 *  \param[in]  priv            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_command_build(struct endat_priv *priv, int32_t cmd,
                        struct cmd_supplement *cmd_supplement);

/**
 *  \brief      trigger sending the EnDat command in PRU
 *
 *  \param[in]  priv     cookie returned by endat_init
 *
 *
 */
void endat_command_send(struct endat_priv *priv);

/**
 *  \brief  wait till PRU finishes EnDat transaction
 *
 *  \param[in]  priv     cookie returned by endat_init
 *
 *
 */
void endat_command_wait(struct endat_priv *priv);


/**
 *  \brief  read RT from DMEM
 *
 *  \param[in]  priv     cookie returned by endat_init
 *
 *   \retval  Value of Recovery Time in nanoseconds
 *
 */
uint32_t endat_get_recovery_time(struct endat_priv *priv);

/**
 *  \brief       update priv with position resolution, id, serial number, encoder
 *               type and supported command set
 *
 *  \param[in]   priv    cookie returned by endat_init
 *
 *  \retval      0 for success, -EINVAL for failure
 *
 */
int32_t endat_get_encoder_info(struct endat_priv *priv);

/**
 *  \brief  get propagation delay automatically estimated by the firmware
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     delay   estimated propogation delay
 *
 */
uint32_t endat_get_prop_delay(struct endat_priv *priv);

/**
 *  \brief  track presence of additional information in priv
 *
 *  \param[in]  priv           cookie returned by endat_init
 *  \param[in]  cmd            EnDat command number
 *  \param[in]  cmd_supplement Supplement information needed to setup EnDat cmd
 *
 *
 */
void endat_addinfo_track(struct endat_priv *priv, int32_t cmd,
                         struct cmd_supplement *cmd_supplement);

/**
 *  \brief  configure EnDat clock
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 *
 */
void endat_config_clock(struct endat_priv *priv,
                        struct endat_clk_cfg *clk_cfg);

/**
 *  \brief  configure tST delay
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  delay   tST delay value
 *
 *
 */
void endat_config_tst_delay(struct endat_priv *priv, uint16_t delay);

/**
 *  \brief  configure rx arm counter
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     rx arm counter value in ns
 *
 *
 */
void endat_config_rx_arm_cnt(struct endat_priv *priv, uint16_t val);

/**
 *  \brief  configure wire delay for the selected channel
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     wire delay in ns
 *
 *
 */
void endat_config_wire_delay(struct endat_priv *priv, uint16_t val);

/**
 *  \brief  configure clocks to be disabled at the end of rx to account for tD
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  val     number of clocks to be disabled
 *
 *
 */
void endat_config_rx_clock_disable(struct endat_priv *priv,
                                   uint16_t val);

/**
 *  \brief       start continuous mode
 *
 *  \param[in]   priv   cookie returned by endat_init
 *
 *  \retval      0       success, -EINVAL for failure
 *
 */
int32_t endat_start_continuous_mode(struct endat_priv *priv);

/**
 *  \brief       stop continuous mode
 *
 *  \param[in]   priv   cookie returned by endat_init
 *
 *
 */
void endat_stop_continuous_mode(struct endat_priv *priv);

/**
 *  \brief      configure EnDat master for host trigger mode
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *
 */
void endat_config_host_trigger(struct endat_priv *priv);

/**
 *  \brief      configure EnDat master in periodic trigger mode
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *
 */
void endat_config_periodic_trigger(struct endat_priv *priv);
/*  brief     set syn_bits of all connected channels for synchronization before global_TX_init
 *
 *  \param[in]  priv    cookie returned by endat_init
 * \param[in]   mask    channels mask
 *
 */
void endat_config_syn_bits(struct endat_priv *priv, uint8_t mask);
/**
 *  \brief     set a core as primay core for global configuration, clk configuration and TX_GLOBAL_INIT
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]   mask    channels mask
 *
 *
 */
void endat_config_primary_core_mask(struct endat_priv *priv, uint8_t mask);
/**
 *  \brief     enable load share mode if encoders has diffent make
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 */
void endat_enable_load_share_mode(struct endat_priv *priv);
/**
 *  \brief      select channel to be used by EnDat master
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  ch      channel to be selected
 *
 *
 */
void endat_config_channel(struct endat_priv *priv, int32_t ch);
/**
 *  \brief      select mask of channels to be used in multi channel configuration by EnDat master
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  mask    channel mask
 *  \param[in]  loadshare  value for loadshare mode enable.
 *
 */
void endat_config_multi_channel_mask(struct endat_priv *priv,
                                     uint8_t mask,
                                     uint8_t loadshare);
/**
 *  \brief      select channels detected in multi channel configuration by EnDat master.    <br>
 *              required to be invoked only if firmware indicates initialization failure    <br>
 *              to know the channels that has been detected. Initialization success implies <br>
 *              that all channels indicated has been detected.
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     mask    mask of the detected channels
 *
 */
uint8_t endat_multi_channel_detected(struct endat_priv *priv);

/**
 *  \brief      In multi channel configuration, select channel before receive processing in <br>
 *              multi channel configuration. After receive is complete, select each channel <br>
 *              and invoke rx API's to parse data recieved in each channel.
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  ch      channel number to be selected
 *
 *
 */
void endat_multi_channel_set_cur(struct endat_priv *priv, int32_t ch);
/**
 *  \brief      wait for EnDat master firmware to initialize
 *
 *  \param[in]  priv    cookie returned by endat_init
 *  \param[in]  timeout timeout to wait for initialization
 *  \param[in]  mask    channel mask
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_wait_initialization(struct endat_priv *priv, uint32_t timeout, uint8_t mask);

/**
 *  \brief      Initialize EnDat firmware interface address and get the pointer
 *              to struct endat_priv instance
 *
 *  \param[in]  pruss_xchg      EnDat firmware interface address
 *  \param[in]  pruss_cfg       ICSS PRU config base address
 *  \param[in]  pruss_iep       ICSS PRU iep base address
 *  \param[in]  slice           ICSS PRU SLICE
 *
 *  \retval     priv            pointer to struct endat_priv instance
 *
 */
struct endat_priv *endat_init(struct endat_pruss_xchg *pruss_xchg,
                              void *pruss_cfg, void *pruss_iep, int32_t slice);

/**
 *  \brief      Read EnDat 2.2 angular position in steps for rotary encoders      <br>
 *              (prior to invoking this, 2.2 position command has to be completed)<br>
 *              This checks the CRC status (updated on-the-fly by firmware),      <br>
 *              checks the group alarm errors (F1/F2 bits) and the angle value    <br>
 *              recieved in reverse are reversed in a highly optimized way using  <br>
 *              inline assembly.                                                  <br>
 *              Assumptions (that holds good for quite a few encoders):           <br>
 *              1. Total resolution >= 25                                         <br>
 *              2. Single turn resolution <= 30
 *
 *  \param[in]  priv    cookie returned by endat_init
 *
 *  \retval     angle for angular position in steps on success, -1 for failure
 *
 */
int32_t endat_get_2_2_angle(struct endat_priv *priv);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
