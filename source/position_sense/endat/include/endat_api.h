/*
 *  Copyright (C) 2021-25 Texas Instruments Incorporated
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
 *  \brief      Initialize EnDat firmware interface address and get the pointer
 *              to struct Endat_Handle instance
 *  
 *  \param[in]  index           Index to use in the global handle array
 *  \param[in]  endat_params    Structure containing initialization parameters
 *
 *  \retval     handle          Pointer to struct Endat_Handle instance, or NULL if parameters are invalid
 *
 */
Endat_Handle endat_init(uint32_t index, Endat_Params endat_params);
/**
 *  \brief      process raw recieved data and format based on the command
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[out] u       pointer to union for storing formatted data based on cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_recvd_process(Endat_Handle handle, int32_t cmd,
                        Endat_FormatData *u);

/**
 *  \brief      CRC result indicated in return value as follows,             <br>
 *              0th bit high    position/address/params/test CRC success     <br>
 *              0th bit low     position/address/params/test CRC failure     <br>
 *              1st bit high    if addinfo 1 present, addinfo 1 CRC success  <br>
 *              1st bit low     if addinfo 1 present, addinfo 1 CRC failure  <br>
 *              2nd bit high    if addinfo 2 present, addinfo 2 CRC success  <br>
 *              2nd bit low     if addinfo 2 present, addinfo 2 CRC failure
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  cmd     EnDat command number
 *  \param[in]  u       pointer to union having formatted data based on cmd
 *
 *  \retval     status  position/address/params/test CRC status
 *
 */
uint32_t endat_recvd_validate(Endat_Handle handle, int32_t cmd,
                              Endat_FormatData *u);

/**
 *  \brief      send the EnDat command and wait till firmware acknowledges
 *
 *  \param[in]  handle            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_command_process(Endat_Handle handle, int32_t cmd,
                          Endat_CmdSupplement *cmd_supplement);

/**
 *  \brief      setup the EnDat command in the PRU interface buffer
 *
 *  \param[in]  handle            cookie returned by endat_init
 *  \param[in]  cmd             EnDat command number
 *  \param[in]  cmd_supplement  Supplement information needed to setup EnDat cmd
 *
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_command_build(Endat_Handle handle, int32_t cmd,
                        Endat_CmdSupplement *cmd_supplement);

/**
 *  \brief      trigger sending the EnDat command in PRU
 *
 *  \param[in]  handle     cookie returned by endat_init
 *
 *
 */
void endat_command_send(Endat_Handle handle);

/**
 *  \brief  wait till PRU finishes EnDat transaction
 *
 *  \param[in]  handle     cookie returned by endat_init
 *
 *
 */
void endat_command_wait(Endat_Handle handle);


/**
 *  \brief  read recovery time parameters from memory
 *
 *  \param[in]  handle     cookie returned by endat_init
 *
 *   \retval  Value of Recovery Time in nanoseconds
 *
 */
uint32_t endat_get_recovery_time(Endat_Handle handle);

/**
 *  \brief       update handle with position resolution, id, serial number, encoder
 *               type and supported command set
 *
 *  \param[in]   handle    cookie returned by endat_init
 *
 *  \retval      0 for success, -EINVAL for failure
 *
 */
int32_t endat_get_encoder_info(Endat_Handle handle);

/**
 *  \brief  get propagation delay automatically estimated by the firmware
 *
 *  \param[in]  handle    cookie returned by endat_init
 *
 *  \retval     delay   estimated propogation delay
 *
 */
uint32_t endat_get_prop_delay(Endat_Handle handle);

/**
 *  \brief  track presence of additional information in handle
 *
 *  \param[in]  handle           cookie returned by endat_init
 *  \param[in]  cmd            EnDat command number
 *  \param[in]  cmd_supplement Supplement information needed to setup EnDat cmd
 *
 *
 */
void endat_addinfo_track(Endat_Handle handle, int32_t cmd,
                         Endat_CmdSupplement *cmd_supplement);

/**
 *  \brief  configure EnDat clock
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  clk_cfg pointer to structure containing clock configuration data
 *
 *
 */
void endat_config_clock(Endat_Handle handle,
                        Endat_ClkCfg_Internal *clk_cfg);

/**
 *  \brief  configure tST delay
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  delay   tST delay value
 *
 *
 */
void endat_config_tst_delay(Endat_Handle handle, uint16_t delay);

/**
 *  \brief  configure rx arm counter
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  val     rx arm counter value in ns
 *
 *
 */
void endat_config_rx_arm_cnt(Endat_Handle handle, uint16_t val);

/**
 *  \brief  configure wire delay for the selected channel
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  val     wire delay in ns
 *
 *
 */
void endat_config_wire_delay(Endat_Handle handle, uint16_t val);

/**
 *  \brief  configure clocks to be disabled at the end of rx to account for tD
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  val     number of clocks to be disabled
 *
 *
 */
void endat_config_rx_clock_disable(Endat_Handle handle,
                                   uint16_t val);

/**
 *  \brief       start continuous mode
 *
 *  \param[in]   handle   cookie returned by endat_init
 *
 *  \retval      0       success, -EINVAL for failure
 *
 */
int32_t endat_start_continuous_mode(Endat_Handle handle);

/**
 *  \brief       stop continuous mode
 *
 *  \param[in]   handle   cookie returned by endat_init
 *
 *
 */
void endat_stop_continuous_mode(Endat_Handle handle);

/**
 *  \brief      configure EnDat master for host trigger mode
 *
 *  \param[in]  handle    cookie returned by endat_init
 *
 *
 */
void endat_config_host_trigger(Endat_Handle handle);

/**
 *  \brief      configure EnDat master in periodic trigger mode
 *
 *  \param[in]  handle    cookie returned by endat_init
 *
 *
 */
void endat_config_periodic_trigger_cmp_mode(Endat_Handle handle);
/*  brief     set syn_bits of all connected channels for synchronization before global_TX_init
 *
 *  \param[in]  handle    cookie returned by endat_init
 * \param[in]   mask    channels mask
 *
 */
void endat_config_syn_bits(Endat_Handle handle, uint8_t mask);
/**
 *  \brief     set a core as primay core for global configuration, clk configuration and TX_GLOBAL_INIT
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]   mask    channels mask
 *
 *
 */
void endat_config_primary_core_mask(Endat_Handle handle, uint8_t mask);
/**
 *  \brief     enable load share mode if encoders has diffent make
 *
 *  \param[in]  handle    cookie returned by endat_init
 *
 */
void endat_enable_load_share_mode(Endat_Handle handle);
/**
 *  \brief      select channel to be used by EnDat master
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  ch      channel to be selected
 *
 *
 */
void endat_config_channel(Endat_Handle handle, int32_t ch);
/**
 *  \brief      select mask of channels to be used in multi channel configuration by EnDat master
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  mask    channel mask
 *  \param[in]  loadshare  value for loadshare mode enable.
 *
 */
void endat_config_multi_channel_mask(Endat_Handle handle,
                                     uint8_t mask,
                                     uint8_t loadshare);
/**
 *  \brief      select channels detected in multi channel configuration by EnDat master.    <br>
 *              required to be invoked only if firmware indicates initialization failure    <br>
 *              to know the channels that has been detected. Initialization success implies <br>
 *              that all channels indicated has been detected.
 *
 *  \param[in]  handle    cookie returned by endat_init
 *
 *  \retval     mask    mask of the detected channels
 *
 */
uint8_t endat_multi_channel_detected(Endat_Handle handle);

/**
 *  \brief      In multi channel configuration, select channel before receive processing in <br>
 *              multi channel configuration. After receive is complete, select each channel <br>
 *              and invoke rx API's to parse data recieved in each channel.
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  ch      channel number to be selected
 *
 *
 */
void endat_multi_channel_set_cur(Endat_Handle handle, int32_t ch);
/**
 *  \brief      wait for EnDat master firmware to initialize
 *
 *  \param[in]  handle    cookie returned by endat_init
 *  \param[in]  timeout timeout to wait for initialization
 *  \param[in]  mask    channel mask
 *  \retval     0 for success, -EINVAL for failure
 *
 */
int32_t endat_wait_initialization(Endat_Handle handle, uint32_t timeout, uint8_t mask);

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
 *  \param[in]  handle    cookie returned by endat_init
 *
 *  \retval     angle for angular position in steps on success, -1 for failure
 *
 */
int32_t endat_get_2_2_angle(Endat_Handle handle);
/**
 *  \brief      Initialize the recovery time parameters 
 *             
 *   \param[in]  handle    cookie returned by endat_init
 *
 */

void endat_init_rt_measurement (Endat_Handle handle); 
/**
 *  \brief      Validate the recovery time parameters 
 *             
 *  \param[in]  handle  cookie returned by endat_init
 * 
 *  \retval  return 0 if measured recovery time is within the recovery time range else return error 
 */

int8_t endat_check_rt_error(Endat_Handle handle);
/**
 *  \brief      Disable the recovery time measurement 
 *             
 *  \param[in]  handle  cookie returned by endat_init
 * 
 */
void endat_disable_rt_measurement (Endat_Handle handle);
/**
 *  \brief      Enable the recovery time measurement 
 *             
 *  \param[in]  handle  cookie returned by endat_init
 * 
 */
void endat_enable_rt_measurement (Endat_Handle handle);
/**
 *  \brief      read the status of recovery time measurement 
 *             
 *  \param[in]  handle  cookie returned by endat_init
 * 
 *  \retval  return 1 if recovery time measurement is enabled else return 0
 */
uint32_t endat_status_rt_measurement (Endat_Handle handle);

/**
 *  \brief      Enable IEP reset on CMP0 event
 *
 *  \param[in]  handle  cookie returned by endat_init
 *  \param[in]  iep_reset_count  IEP reset count value for CMP0
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_enable_iep_reset_on_cmp0(Endat_Handle handle, uint64_t iep_reset_count);

/**
 *  \brief      Enable IEP counter
 * 
 *   Note: It configure IEP counter increment values from  
 *   handle->pru_cfg.iep_increment which is configured during endat_init.
 *
 *  \param[in]  handle  cookie returned by endat_init 
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_enable_iep_counter(Endat_Handle handle);

/**
 *  \brief      Disable IEP counter
 *
 *  \param[in]  handle  cookie returned by endat_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_disable_iep_counter(Endat_Handle handle);

/**
 *  \brief      Configure periodic trigger CAP mode
 *
 *  This function configures the EnDAT channels to operate in periodic trigger mode
 *  with IEP CAP events.
 *
 *  \param[in]  handle  cookie returned by endat_init
 *
 */
void endat_config_periodic_trigger_cap_mode(Endat_Handle handle);

/**
 *  \brief      Configure IEP CMP event for periodic trigger
 *
 *  This function configures an IEP compare event for periodic trigger mode.
 *  It sets the compare register value and enables the specified CMP event (0-15).
 *
 *  \param[in]  handle          cookie returned by endat_init
 *  \param[in]  channel         EnDAT channel number (0-2)
 *  \param[in]  trigger_point   IEP counter value at which the CMP event triggers
 *  \param[in]  event_num       CMP event number (0-15)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_config_iep_cmp_event(Endat_Handle handle, uint8_t channel, uint64_t trigger_point, uint8_t event_num);

/**
 *  \brief      Configure IEP CAP event for periodic trigger
 *
 *  This function configures an IEP capture event for periodic trigger mode.
 *  It enables the specified CAP event (0-7) and configures the capture register
 *  address and event number in the PRU shared memory for firmware access.
 *
 *  \param[in]  handle      cookie returned by endat_init
 *  \param[in]  channel     EnDAT channel number (0-2)
 *  \param[in]  event_num   CAP event number (0-7)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_config_iep_cap_event(Endat_Handle handle, uint8_t channel, uint8_t event_num);

/**
 *  \brief      Disable IEP CMP event
 *
 *  This function disables a previously configured IEP compare event.
 *  It clears the CMP_EN bit for the specified CMP event (0-15).
 *
 *  \param[in]  handle      cookie returned by endat_init
 *  \param[in]  event_num   CMP event number (0-15) to disable
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_disable_iep_cmp_event(Endat_Handle handle, uint8_t event_num);

/**
 *  \brief      Disable IEP CAP event
 *
 *  This function disables a previously configured IEP capture event.
 *  It clears the CAP_EN bit for the specified CAP event (0-7).
 *
 *  \param[in]  handle      cookie returned by endat_init
 *  \param[in]  event_num   CAP event number (0-7) to disable
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 */
int32_t endat_disable_iep_cap_event(Endat_Handle handle, uint8_t event_num);
/** @} */

#ifdef __cplusplus
}
#endif

#endif
