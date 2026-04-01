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

#ifndef NIKON_API_H_
#define NIKON_API_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "nikon_drv.h"

/* ========================================================================== */
/*                              API Declarations                              */
/* ========================================================================== */

/**
 *  \defgroup NIKON_API_MODULE APIs for Nikon A-Format Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for Nikon A-Format encoder communication protocol
 *
 * ## Validation Strategy
 *
 * Nikon driver APIs use following validation approach:
 * - **Public API validation**: All public APIs validate the handle parameter for NULL and perform
 *   array bounds checking for index parameters (ch, ls_ch, ch_idx)
 * - **Internal function validation**: Internal static functions assume valid parameters. The caller
 *   is responsible for ensuring parameters are valid before calling internal functions
 * - **Internal structure validation**: Each API validates the internal structure pointers it accesses
 *   (e.g., attrs, priv, pruicss_xchg, pruicss_handle) for NULL before dereferencing
 * - **Error state handling**: Error in \ref nikon_get_pos may leave internal state partially modified.
 *   Subsequent calls will overwrite these values. Caller is responsible for explicit state cleanup if
 *   needed.
 *
 * ## Typical API Call Sequence
 *
 * 1. Initialize params: \ref nikon_params_init
 * 2. Configure pruicss_handle in params
 * 3. Initialize driver: \ref nikon_init (validates all attrs and params)
 * 4. Configure operation mode:
 *    - Host trigger mode: \ref nikon_config_host_trigger
 *    - Periodic CMP mode: \ref nikon_config_periodic_trigger_cmp_mode
 *    - Periodic CAP mode: \ref nikon_config_periodic_trigger_cap_mode
 * 5. Detect encoder: \ref nikon_wait_for_encoder_detection
 * 6. Update encoder settings: \ref nikon_update_enc_len, \ref nikon_update_enc_addr
 * 7. Get position data: \ref nikon_get_pos
 * 8. De-initialize: \ref nikon_deinit
 *
 *  @{
 */


/**
 *  \brief      Trigger sending the Nikon command to PRU
 *
 *  \details    This function sets the cycle_trigger flag(s) in the PRU-ICSS exchange
 *              structure to initiate a Nikon transaction. In load share mode, it sets
 *              individual trigger flags for each enabled channel.
 *
 *  \param[in]  handle   Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t nikon_command_send(nikon_handle handle);

/**
 *  \brief      Wait till PRU finishes Nikon transaction
 *
 *  \details    This function polls the cycle_trigger flag(s) in PRU-ICSS exchange structure
 *              to detect when the PRU firmware has completed the Nikon transaction. In host
 *              trigger mode (non-continuous mode), it adds a delay between poll iterations
 *              to prevent excessive CPU usage, and implements a timeout mechanism.
 *
 *              This function internally calls:
 *              - ClockP_usleep(): Delay configured via nikon_params.cmd_process_delay_us before calling \ref nikon_init
 *                (default: 1000 micro-seconds) between poll iterations (in host trigger mode only)
 *
 *  \param[in]  handle   Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on successful completion
 *  \retval     SystemP_TIMEOUT when communication timeout occurs (encoder not responding or
 *              transaction takes longer than configured timeout)
 *  \retval     SystemP_FAILURE on NULL handle or invalid internal structures
 *
 *  \note       Timeout is calculated as: max_wait_loop_count * cmd_process_delay_us / 1000 (in ms)
 *              With defaults (max_wait_loop_count=35, cmd_process_delay_us=1000us), timeout is 35ms.
 *              Both parameters can be configured in \ref nikon_params before calling \ref nikon_init.
 *
 */
int32_t nikon_command_wait(nikon_handle handle);

/**
 *  \brief      Send the Nikon command and wait till firmware acknowledges
 *
 *  \details    This function processes a Nikon command transaction. The behavior
 *              differs based on the operating mode:
 *
 *              **Host Trigger Mode**:
 *              - Calls \ref nikon_command_send to trigger the command in PRU
 *              - Calls \ref nikon_command_wait to wait for PRU completion
 *
 *              **Periodic Trigger Mode (CMP or CAP)**:
 *              - Skips calling \ref nikon_command_send (PRU triggers automatically)
 *              - Only calls \ref nikon_command_wait to wait for PRU completion
 *
 *              In periodic modes, the PRU firmware automatically initiates Nikon
 *              transactions based on IEP timer events, so explicit command sending
 *              by the host is not required.
 *
 *  \param[in]  handle   Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_TIMEOUT when communication timeout occurs (forwarded from \ref nikon_command_wait)
 *  \retval     SystemP_FAILURE on error (NULL handle or \ref nikon_command_send failure)
 *
 */
int32_t nikon_command_process(nikon_handle handle);

/**
 *  \brief      Get single cycle Nikon position data
 *
 *  \details    This function sends a command to the encoder and retrieves position/status data.
 *              The command determines what data is returned (position, alarms, temperature, etc.).
 *              After successful command processing, this function parses the raw data from PRU
 *              firmware and stores results in priv->pos_data_info[], priv->enc_info[], and
 *              protocol-specific fields.
 *
 *              This function internally calls:
 *              - \ref nikon_command_process : Send command and wait for firmware acknowledgment
 *
 *  \param[in]  handle   Nikon handle from \ref nikon_init
 *  \param[in]  cmd      Command code from nikon_cmd enum. Valid commands:
 *                       - For Nikon V2.1: CMD_0 to CMD_22, CMD_27 to CMD_30
 *                       - For Nikon V3.0: CMD_0 to CMD_30, CMD_1_VEL to CMD_18_VEL
 *                       NOTE: CMD_23 to CMD_26 and CMD_1_VEL to CMD_18_VEL are Nikon 3.0 only.
 *                       Commands ENCODER_ADR_CHANGE, START_CONTINUOUS_CMP_MODE, START_CONTINUOUS_CAP_MODE,
 *                       UPDATE_CLOCK_FREQ, and UPDATE_ENC_LEN are invalid for this API.
 *
 *  \retval     SystemP_SUCCESS for success
 *  \retval     SystemP_TIMEOUT when communication timeout occurs (forwarded from \ref nikon_command_process)
 *  \retval     SystemP_FAILURE for invalid handle, invalid command or communication failure
 *
 *  \note       Results are stored in handle->priv and can be accessed via \ref nikon_get_priv
 *
 *  \note       Error Handling: On communication failure or timeout, this function may leave internal state
 *              partially modified (pruicss_xchg->is_memory_access, pruicss_xchg->num_mdf,
 *              priv->num_rx_frames). Subsequent calls will overwrite these values. If explicit state
 *              cleanup is needed after errors, the caller is responsible for resetting these fields.
 */
int32_t nikon_get_pos(nikon_handle handle, uint32_t cmd);

/**
 *  \brief      Initialize a Nikon instance
 *
 *  \details    This function initializes a Nikon instance by setting up the firmware
 *              interface and configuring hardware based on SysConfig parameters.
 *              The driver is automatically configured in host trigger mode after initialization.
 *
 *  \param[in]  index       Index of Nikon handle to use in the gNikonHandle array
 *  \param[in]  params      Pointer to structure containing Nikon parameters.
 *
 *  \retval     handle      Pointer to initialized nikon_handle instance
 *  \retval     NULL        On validation failure (invalid index, NULL params, NULL priv/attrs,
 *                          invalid params, invalid attrs values, or failed hardware initialization)
 *
 */
nikon_handle nikon_init(uint32_t index, const nikon_params *params);

/**
 *  \brief      De-initialize Nikon interface
 *
 *  \details    Closes the Nikon interface and marks it as not open.
 *              Should be called when done using the Nikon interface.
 *
 *  \param[in]  handle      Nikon handle from \ref nikon_init
 *
 */
void nikon_deinit(nikon_handle handle);

/**
 *  \brief      Initialize Nikon params structure with defaults
 *
 *  \details    Populates the provided params structure with default values.
 *              Use this before calling \ref nikon_init to get default parameters.
 *
 *  \param[out] params      Pointer to params structure to initialize
 *
 */
void nikon_params_init(nikon_params *params);

/**
 *  \brief      Get pointer to Nikon attributes
 *
 *  \details    Returns a pointer to the compile-time attributes structure.
 *
 *  \param[in]  handle      Nikon handle from \ref nikon_init
 *
 *  \retval     attrs       Pointer to attrs structure, or NULL if handle invalid
 *
 */
const nikon_attrs* nikon_get_attrs(nikon_handle handle);

/**
 *  \brief      Get pointer to Nikon private data
 *
 *  \details    Returns a pointer to the runtime private data structure.
 *
 *  \param[in]  handle      Nikon handle from \ref nikon_init
 *
 *  \retval     priv        Pointer to priv structure, or NULL if handle invalid
 *
 */
nikon_priv* nikon_get_priv(nikon_handle handle);

/**
 *  \brief      Calculate Rx and Tx divisors for given frequency.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  clk_cfg         pointer to structure containing clock configuration data.
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_calc_clock(nikon_handle handle, nikon_clk_cfg *clk_cfg);

/**
 *  \brief      Generate CDF command to be sent to encoder. Update encoder ID appropriately with \ref nikon_update_enc_addr API.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  cmd             command code requested by the user. Valid commands:
 *                              - For Nikon V2.1: CMD_0 to CMD_22, CMD_27 to CMD_30
 *                              - For Nikon V3.0: CMD_0 to CMD_30, CMD_1_VEL to CMD_18_VEL
 *                              NOTE: CMD_23 to CMD_26 and CMD_1_VEL to CMD_18_VEL are Nikon 3.0 only.
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure:
 *                              - NULL handle
 *                              - NULL handle->priv
 *                              - NULL handle->attrs
 *                              - For Nikon V2.1: (cmd > CMD_22 and cmd < CMD_27) or cmd > CMD_30
 *                              - (cmd > CMD_30 and cmd < CMD_1_VEL) or cmd >= CMD_CODE_NUM
 */
int32_t nikon_generate_cdf(nikon_handle handle, uint32_t cmd);

/**
 *  \brief      Reverse the bits (LSB to be sent out first) provided as parameters.
 *
 *  \param[in]  bits        field to be reversed as specified by the user.
 *  \param[in]  num_bits    width of field to be reversed.
 *
 *  \retval     res         return the reversed field.
 */
uint64_t nikon_reverse_bits(uint64_t bits, uint32_t num_bits);

/**
 *  \brief      Configure Load Share mode for Nikon receiver
 *
 *  \param[in]  handle  Nikon handle from \ref nikon_init
 *  \param[in]  mask    channel mask
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_config_load_share(nikon_handle handle, uint8_t mask);

/**
 *  \brief      Configure periodic trigger CMP mode operation
 *
 *  \details    Configures the Nikon firmware to use IEP CMP (compare) events for periodic triggering.
 *              Position data is sampled automatically when IEP counter reaches the configured
 *              CMP event compare value.
 *
 *              **Configuration requirements:**
 *              - IEP hardware CMP registers must be configured separately
 *              - Use \ref nikon_config_iep_cmp_event to set event number in firmware. This function
 *                is called inside \ref nikon_init by default.
 *              - CMP event range: 0-15
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_config_periodic_trigger_cmp_mode(nikon_handle handle);

/**
 *  \brief      Configure host trigger operation mode
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_config_host_trigger(nikon_handle handle);

/**
 *  \brief      Configure periodic trigger CAP mode operation
 *
 *  \details    Configures the Nikon firmware to use IEP CAP (capture) events for periodic triggering.
 *              Position data is sampled automatically when an external signal triggers
 *              the IEP capture event.
 *
 *              **Configuration requirements:**
 *              - IEP hardware CAP registers must be configured separately
 *              - External signal to IEP capture input should be configured
 *              - Use \ref nikon_config_iep_cap_event to set event number in firmware. This function
 *                is called inside \ref nikon_init by default.
 *              - CAP event range: 0-7
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_config_periodic_trigger_cap_mode(nikon_handle handle);

/**
 *  \brief      Configure IEP CAP event for periodic trigger (DMEM configuration only)
 *
 *  \details    This function configures the IEP capture event information in PRU shared
 *              memory (DMEM) for firmware access. It writes the capture register address
 *              and event number to trigger_params structure. This function does NOT configure
 *              IEP hardware registers.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  channel         Channel number (0-2 for ch0, ch1, ch2). Used in load share mode,
 *                              ignored in single PRU mode (always uses index 0).
 *  \param[in]  event_num       IEP CAP event number (valid range: 0-7)
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on NULL handle or invalid parameters
 *
 *  \note       This function only configures firmware DMEM, not IEP hardware.
 *              Application must separately configure IEP CAP hardware registers.
 */
int32_t nikon_config_iep_cap_event(nikon_handle handle, uint8_t channel, uint8_t event_num);

/**
 *  \brief      Configure IEP CMP event for periodic trigger (DMEM configuration only)
 *
 *  \details    This function configures the IEP compare event information in PRU shared
 *              memory (DMEM) for firmware access. It writes the event number to trigger_params
 *              structure. This function does NOT configure IEP hardware registers.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  channel         Channel number (0-2 for ch0, ch1, ch2). Used in load share mode,
 *                              ignored in single PRU mode (always uses index 0).
 *  \param[in]  event_num       IEP CMP event number (valid range: 0-15)
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on NULL handle or invalid parameters
 *
 *  \note       This function only configures firmware DMEM, not IEP hardware.
 *              Application must separately configure IEP CMP hardware registers.
 */
int32_t nikon_config_iep_cmp_event(nikon_handle handle, uint8_t channel, uint8_t event_num);

/**
 *  \brief      Detect whether the connected encoder is of selected frequency
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_TIMEOUT when communication timeout occurs (forwarded from \ref nikon_command_process)
 *  \retval     SystemP_FAILURE on error (invalid handle or communication failure)
 */
int32_t nikon_wait_for_encoder_detection(nikon_handle handle);

/**
 *  \brief      Update number of encoders connected in bus or one-to-one and
 *              update data lengths of those encoders
 *
 *  \details    This function configures encoder resolution parameters for a specific channel.
 *              It validates and stores single-turn and multi-turn bit lengths for each encoder
 *              in the bus configuration. The function performs atomic validation of all encoders
 *              before updating any state - if any encoder configuration is invalid, no changes
 *              are made.
 *
 *              Validation performed:
 *              - All encoder configurations must satisfy: single_turn_len + multi_turn_len <= NIKON_MAX_ABS_LEN (40 bits)
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  num_encoders    Number of encoders connected to the channel (1 to NUM_ENCODERS_MAX).
 *                              Must not be 0.
 *  \param[in]  single_turn_len Array containing single-turn resolution in bits for each encoder.
 *                              Array must have at least num_encoders elements.
 *                              A value of 0 indicates encoder not connected.
 *  \param[in]  multi_turn_len  Array containing multi-turn resolution in bits for each encoder.
 *                              Array must have at least num_encoders elements.
 *  \param[in]  ch              Channel index (0 to NIKON_NUM_CH_PER_SLICE_MAX-1)
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure:
 *                              - NULL handle
 *                              - NULL single_turn_len or multi_turn_len array pointers
 *                              - ch >= NIKON_NUM_CH_PER_SLICE_MAX
 *                              - num_encoders is 0 or exceeds NUM_ENCODERS_MAX
 *                              - Any encoder's (single_turn_len + multi_turn_len) > NIKON_MAX_ABS_LEN (40 bits)
 *
 *  \note       Arrays must contain at least num_encoders elements. Only the first num_encoders
 *              elements are accessed. Passing arrays with fewer elements will result in undefined behavior.
 */
int32_t nikon_update_enc_len(nikon_handle handle,
                              uint32_t num_encoders,
                              uint32_t single_turn_len[],
                              uint32_t multi_turn_len[],
                              uint32_t ch);

/**
 *  \brief      Update the operating baud rate to the specified frequency
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  frequency       Frequency in MHz as specified by the user
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_update_clock_freq(nikon_handle handle, float_t frequency);

/**
 *  \brief      Retrieves the current channel in use
 *
 *  \details    This function returns the physical channel number at the specified index
 *              in the internal channel array. For example, if channels 0 and 2 are enabled,
 *              ch_idx=0 returns 0, ch_idx=1 returns 2.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  ch_idx          Index into the channel array (0 to NIKON_NUM_CH_PER_SLICE_MAX-1)
 *  \param[out] channel         Pointer to store the physical channel number (0, 1, or 2)
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on NULL handle, NULL channel pointer, or invalid ch_idx
 */
int32_t nikon_get_current_channel(nikon_handle handle, uint32_t ch_idx, uint32_t *channel);

/**
 *  \brief      Update encoder address as specified by the user. Default initial address is 0.
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  enc_addr        encoder address specified by the user
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure:
 *                              - NULL handle
 *                              - NULL handle->priv
 *                              - ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX
 *                              - enc_addr > NIKON_ENC_ADDR_MAX
 */
int32_t nikon_update_enc_addr(nikon_handle handle, uint32_t enc_addr, uint32_t ls_ch);

/**
 *  \brief      Access encoder's EEPROM location specified by the user
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  addr            EEPROM address (Bits [7:0] in addr)
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_update_eeprom_addr(nikon_handle handle, uint8_t addr, uint32_t ls_ch);

/**
 *  \brief      Write data at specified EEPROM location
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  data            data to write at EEPROM location (Bits [15:0] in data)
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_update_eeprom_data(nikon_handle handle, uint16_t data, uint32_t ls_ch);

/**
 *  \brief      Assign the specified 24bits as Identification code of encoder
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  data            data to assign as ID code (Bits [23:0] in data will be used)
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure:
 *                              - NULL handle
 *                              - NULL handle->priv
 *                              - ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX
 *                              - data > 0xFFFFFF (exceeds 24-bit width)
 */
int32_t nikon_update_id_code(nikon_handle handle, uint32_t data, uint32_t ls_ch);

/**
 *  \brief      Assign the specified 19 bits as velocity coefficient of encoder
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  data            data to assign as velocity coefficient (Bits [18:0] in data will be used)
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure:
 *                              - NULL handle
 *                              - NULL handle->priv
 *                              - ls_ch >= NIKON_NUM_CH_PER_SLICE_MAX
 *                              - data > 0x7FFFF (exceeds 19-bit width)
 */
int32_t nikon_update_velocity_coefficient(nikon_handle handle, uint32_t data, uint32_t ls_ch);

/**
 *  \brief Configure bank for memory operations in Nikon 3.0
 *
 *  \param[in]  handle          Nikon handle from \ref nikon_init
 *  \param[in]  bank            Bank number to be set (Bits [7:0] in bank)
 *  \param[in]  ls_ch           channel in use in load share or 0 in case of single channel mode
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t nikon_update_eeprom_bank(nikon_handle handle, uint8_t bank, uint32_t ls_ch);

/** @} */
#ifdef __cplusplus
}
#endif

#endif
