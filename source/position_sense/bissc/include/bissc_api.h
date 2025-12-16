/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "bissc_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \defgroup BISSC_API_MODULE APIs for BiSS-C Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for BiSS-C encoder communication protocol
 *
 *  \par Validation Strategy
 *  BiSS-C driver APIs use a simplified validation approach for optimal performance:
 *  - **Handle validation**: All public APIs validate the handle parameter for NULL
 *  - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *  - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *    are validated once during bissc_init() and assumed valid in subsequent API calls
 *  - This strategy reduces overhead in time-critical data path functions
 *
 *  @{
 */

/**
 *  \brief      Initialize the parameters data structure with defaults
 *
 *  \details    This function copies default BiSS-C parameters from gBisscDefaultParams
 *              to the provided params structure.
 *
 *  \param[out] params  Initialized parameters
 *
 *  \note       On NULL params, function returns without performing any operation
 */
void bissc_params_init(bissc_params *params);

/**
 *  \brief      Initialize a BiSS-C instance
 *
 *  \details    This function initializes a BiSS-C instance by setting up the firmware
 *              interface and configuring hardware based on SysConfig parameters.
 *
 *              This function internally calls the following APIs:
 *              - \ref bissc_hw_init : Initialize hardware interface
 *              - \ref bissc_config_channel : Configure channel mask and total channels
 *              - bissc_config_load_share (internal) : Configure load share mode (if multi-channel multi-PRU mode is enabled, applicable for PRU-ICSSG only)
 *              - \ref bissc_set_default_initialization : Set default configuration parameters
 *              - \ref bissc_config_host_trigger : Configure host trigger mode
 *
 *  \param[in]  index           Index of BiSS-C handle to use in the gBisscHandle array
 *  \param[in]  bissc_params    Pointer to structure containing BiSS-C parameters
 *
 *  \retval     handle          Pointer to initialized bissc_handle instance
 *  \retval     NULL            On validation failure (invalid index, NULL params, NULL priv/attrs,
 *                              invalid pruicss_handle, failed hardware initialization)
 *
 */
bissc_handle bissc_init(uint32_t index, const bissc_params *bissc_params);

/**
 *  \brief      De-initialize a BiSS-C instance
 *
 *  \details    This function de-initializes the BiSS-C instance by marking the handle
 *              as closed (is_open = 0). It does not free memory or disable PRU cores.
 *
 *  \param[in]  bissc_handle     Handle to BiSS-C instance
 *
 *  \note       On NULL handle or NULL priv, function returns without performing any operation
 */
void bissc_deinit(bissc_handle bissc_handle);

/**
 *  \brief      Send the BiSS-C command and wait till firmware acknowledges
 *
 *  \details    This function internally calls the following APIs:
 *              - \ref bissc_command_send : Trigger sending the BiSS-C command in PRU
 *              - \ref bissc_command_wait : Wait till PRU finishes BiSS-C transaction
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 */
int32_t bissc_command_process(bissc_handle handle);

/**
 *  \brief      Trigger sending the BiSS-C command in PRU
 *
 *  \details    This function sets the cycle_trigger flag(s) in the PRU-ICSS exchange
 *              structure to initiate a BiSS-C transaction. In load share mode, it sets
 *              individual trigger flags for each enabled channel. In non-load share mode,
 *              it sets a single trigger flag.
 *
 *  \param[in]  handle     BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_command_send(bissc_handle handle);

/**
 *  \brief      Wait till PRU finishes BiSS-C transaction
 *
 *  \details    This function polls the cycle_trigger flag(s) in PRU-ICSS exchange structure
 *              to detect when the PRU firmware has completed the BiSS-C transaction. In host
 *              trigger mode (non-continuous mode), it adds a delay between poll iterations
 *              to prevent excessive CPU usage, and implements a timeout mechanism.
 *
 *              This function internally calls:
 *              - ClockP_usleep(): Delay configured via bissc_params.cmd_process_delay_us before calling \ref bissc_init
 *                (default: 1000 microseconds) between poll iterations (in host trigger mode only)
 *
 *  \param[in]  handle     BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on successful completion
 *  \retval     SystemP_FAILURE on timeout (configured via bissc_params.max_wait_loop_count before calling \ref bissc_init, default: 5ms)
 *
 */
int32_t bissc_command_wait(bissc_handle handle);

/**
 *  \brief      Get single cycle BiSS-C position data
 *
 *  \details    This function internally calls the following APIs:
 *              - \ref bissc_command_process : Send command and wait for firmware acknowledgment
 *              After successful command processing, this function extracts position data,
 *              error/warning bits, CRC values, and calculates angle and number of turns
 *              from the raw data received from the encoder.
 *
 *  \param[in]  handle     BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 */
int32_t bissc_get_pos(bissc_handle handle);

/**
 *  \brief      Configure BiSS-C clock
 *
 *  \details    This function configures the PRU-ICSS clock registers based on the provided
 *              clock configuration structure. After clock configuration, it conditionally calls:
 *              - bissc_enable_load_share_mode (internal) : Enable load share mode (if load sharing is enabled)
 *
 *  \param[in]  handle      BiSS-C handle
 *  \param[in]  clk_cfg     pointer to structure containing clock configuration data
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation or hardware access failure
 */
int32_t bissc_config_clock(bissc_handle handle, bissc_clk_cfg *clk_cfg);

/**
 *  \brief      Select channel to be used by BiSS-C receiver
 *
 *  \details    This function configures the channel mask in PRU firmware and builds
 *              an internal channel array mapping enabled channels to their indices.
 *              For example, if channels 0 and 2 are enabled, priv->channel[] will be
 *              {0, 2}. This mapping is used throughout the driver for channel operations.
 *
 *  \param[in]  handle          BiSS-C handle
 *  \param[in]  mask            channel mask (bit 0=CH0, bit 1=CH1, bit 2=CH2)
 *  \param[in]  total_channels  total number of channels in use
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_config_channel(bissc_handle handle, uint8_t mask, uint8_t total_channels);


/**
 *  \brief      Wait for BiSS-C receiver firmware to initialize
 *
 *  \details    This function polls the firmware initialization status flags in the
 *              PRU-ICSS exchange structure to detect when PRU firmware has completed
 *              initialization. It checks the status flags based on the channel mask
 *              to ensure all enabled channels are initialized.
 *
 *              This function internally calls:
 *              - ClockP_usleep(): Delay configured via bissc_params.fw_wait_delay_us (before calling \ref bissc_init)
 *                (default: 1000 microseconds) between poll iterations to prevent excessive CPU usage
 *
 *  \param[in]  handle          BiSS-C handle
 *  \param[in]  loop_count      timeout value in iterations
 *  \retval     SystemP_SUCCESS when all specified channels are initialized
 *  \retval     SystemP_FAILURE on timeout
 *
 */
int32_t bissc_wait_for_fw_initialization(bissc_handle handle, uint32_t loop_count);

/**
 *  \brief      Initialize BiSS-C hardware interface
 *
 *  \details    This function internally calls the following APIs:
 *              - \ref bissc_calc_clock : Calculate Rx and Tx divisors for the configured frequency
 *              - bissc_config_endat_mode (internal) : Configure the receiver for EnDat mode
 *              - \ref bissc_config_clock : Configure the PRU-ICSS clock registers
 *              - bissc_config_clr_cfg0 (internal) : Clear the channel specific frame size configuration registers
 *
 *  \param[in]  handle    BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation or hardware initialization failure
 */
int32_t bissc_hw_init(bissc_handle handle);


/**
 *  \brief      Update maximum processing delay value
 *
 *  \details    This function sets the maximum encoder processing delay value in the
 *              PRU-ICSS exchange structure based on the configured baud rate.
 *
 *  \param[in]  handle        BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_update_max_proc_delay(bissc_handle handle);

/**
 *  \brief      Wait for BiSS-C receiver firmware to measure processing time
 *
 *  \details    This function polls the measure_proc_delay flag in PRU-ICSS exchange
 *              structure to detect when firmware has completed encoder processing delay
 *              measurement.
 *
 *              This function internally calls:
 *              - ClockP_usleep(): Delay configured via bissc_params.fw_wait_delay_us (before calling \ref bissc_init)
 *                (default: 1000 microseconds) between poll iterations to prevent excessive CPU usage
 *
 *  \param[in]  handle       BiSS-C handle
 *  \param[in]  loop_count   timeout value in iterations
 *  \retval     SystemP_SUCCESS when measurement completes
 *  \retval     SystemP_FAILURE on timeout
 *
 */
int32_t bissc_wait_measure_proc_delay(bissc_handle handle, uint32_t loop_count);

/**
 *  \brief      Set default configuration parameters for BiSS-C receiver firmware
 *
 *  \details    This function internally calls the following APIs:
 *              - \ref bissc_update_max_proc_delay: Update maximum processing delay based on configured frequency
 *
 *  \param[in]  handle        BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_set_default_initialization(bissc_handle handle);

/**
 *  \brief      Update data length with encoder bit width for BiSS-C receiver firmware
 *
 *  \details    This function configures encoder resolution parameters for a specific channel.
 *              It updates the PRU firmware with single-turn and multi-turn bit lengths for
 *              each encoder in the daisy chain, calculates total data length including CRC
 *              bits, and stores these values in both the private structure and PRU-ICSS
 *              exchange memory for firmware access.
 *
 *              **Validation Strategy:**
 *              This function uses a two-phase approach to ensure atomic configuration:
 *              - **Phase 1 (Validation)**: All encoder configurations are validated against
 *                frame size constraints without modifying any driver state
 *              - **Phase 2 (Configuration)**: Only after all validations pass, the driver
 *                state and PRU-ICSS exchange memory are updated
 *
 *              This ensures that if any encoder configuration is invalid, the driver remains
 *              in its previous valid state rather than being left partially configured.
 *
 *              **Frame Size Constraints:**
 *              This function validates that the total frame size does not exceed 64 bits:
 *              - Without Safety: Position Data + E/W(2) + CRC(6) <= 64 bits
 *                  Therefore: single_turn + multi_turn <= 56 bits
 *              - With Safety: Position Data + E/W(2) + sign-of-life(6) + safety CRC(16) <= 64 bits
 *                  Therefore: single_turn + multi_turn <= 40 bits
 *
 *  \param[in]  handle              BiSS-C handle
 *  \param[in]  single_turn_len     Encoder's single turn resolution array
 *  \param[in]  multi_turn_len      Encoder's multi turn resolution array
 *  \param[in]  ch_num              channel number(index) in use
 *
 *  \retval     SystemP_SUCCESS on success
 *  \retval     SystemP_FAILURE on validation failure (NULL parameters, invalid ch_num, or frame size exceeds 64 bits)
 *
 *  \note       The single_turn_len and multi_turn_len arrays must contain
 *              \ref BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX elements. Passing arrays
 *              with fewer elements will result in undefined behavior.
 */
int32_t bissc_update_data_len(bissc_handle handle, uint32_t single_turn_len[], uint32_t multi_turn_len[], uint32_t ch_num);

/**
 *  \brief      Set control command and process the ctrl communication read/write
 *
 *  \details    This function sends control commands to the encoder and polls for completion.
 *              It sets the ctrl_cmd and ctrl_cmd_status fields in PRU-ICSS exchange structure,
 *              then polls until firmware completes the control communication transaction.
 *              A delay configured via bissc_params.fw_wait_delay_us (before calling \ref bissc_init) (default: 1000 microseconds)
 *              is used between poll iterations and after control communication stop bits to ensure
 *              proper timing for encoder control communication protocol compliance. After completion,
 *              it reads back the control data from the encoder response.
 *
 *              This function internally calls the following APIs:
 *              - \ref bissc_command_process : Send command and wait for firmware acknowledgment (called multiple times)
 *              - ClockP_usleep(): Delay between poll iterations and after stop bits
 *
 *  \param[in]  handle            BiSS-C handle
 *  \param[in]  ctrl_cmd          Hex equivalent of control command array (per channel)
 *  \retval     SystemP_SUCCESS for success, SystemP_FAILURE for failure
 *
 *  \note       The ctrl_cmd array must contain \ref BISSC_NUM_CH_PER_SLICE_MAX
 *              elements. Passing an array with fewer elements will result in
 *              undefined behavior.
 */
int32_t bissc_set_ctrl_cmd_and_process(bissc_handle handle, uint32_t ctrl_cmd[]);


/**
 *  \brief      Get measured processing delay of individual channel
 *
 *  \details    This function copies the encoder processing delay values measured by
 *              the PRU firmware from the PRU-ICSS exchange structure to the driver's
 *              private structure.
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_get_enc_proc_delay(bissc_handle handle);

/**
 *  \brief      Calculate Rx and Tx divisors for given frequency
 *
 *  \details    This function calculates the Tx and Rx clock divisors required to achieve
 *              the configured baud rate from either core clock or UART clock source. It
 *              also sets the FIFO bit index for oversampling and determines valid baud
 *              rates based on the clock source and divisor limits. Returns error if the
 *              requested frequency cannot be achieved with available clock sources.
 *
 *  \param[in]  handle            BiSS-C handle
 *  \param[out] clk_cfg           pointer to structure to store calculated clock configuration
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t bissc_calc_clock(bissc_handle handle, bissc_clk_cfg *clk_cfg);

/**
 *  \brief      Configure periodic trigger operation mode
 *
 *  \details    This function sets the operation mode to \ref BISSC_OPMODE_PERIODIC in the
 *              PRU-ICSS exchange structure. In this mode, the PRU firmware automatically
 *              triggers position data retrieval at regular intervals based on IEP timer
 *              configuration.
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_config_periodic_trigger(bissc_handle handle);

/**
 *  \brief      Configure host trigger operation mode
 *
 *  \details    This function sets the operation mode to \ref BISSC_OPMODE_HOST_TRIGGER in the
 *              PRU-ICSS exchange structure. In this mode, position data is retrieved only
 *              when explicitly triggered by the host via \ref bissc_command_send.
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_config_host_trigger(bissc_handle handle);

/**
 *  \brief      Generate control communication Hex equivalent command
 *
 *  \details    This function generates a control communication command according to BiSS-C protocol.
 *
 *              This function internally calls the following APIs:
 *              - bissc_calc_ctrl_crc (internal) : Calculate 4-bit CRC values for command and data portions
 *
 *  \param[in]  handle              BiSS-C handle
 *  \param[in]  ls_ch               channel in use for load share
 *  \param[in]  ctrl_write_status   status for control communication write access
 *  \param[in]  ctrl_reg_address    address of encoder's register for control communication access
 *  \param[in]  ctrl_reg_data       data to write in encoder's register in control communication
 *  \param[in]  ctrl_enc_id         ID of encoder based on it's place in daisy chain
 *  \retval     ctrl_cmd            Hex equivalent control communication 32 bit command
 *  \retval     0                   On NULL handle or invalid ls_ch (>= \ref BISSC_NUM_CH_PER_SLICE_MAX)
 */
uint32_t bissc_generate_ctrl_cmd(bissc_handle handle,
                                 uint8_t ls_ch,
                                 uint8_t ctrl_write_status,
                                 uint32_t ctrl_reg_address,
                                 uint32_t ctrl_reg_data,
                                 uint32_t ctrl_enc_id);
/**
 *  \brief      Retrieves the current channel in use
 *
 *  \details    This function returns the physical channel number at the specified index
 *              in the internal channel array. For example, if channels 0 and 2 are enabled,
 *              ch_idx=0 returns 0, ch_idx=1 returns 2.
 *
 *  \param[in]  handle          BiSS-C handle
 *  \param[in]  ch_idx          index into the channel array
 *  \retval     channel[ch_idx] physical channel number (0, 1, or 2)
 *  \retval     0               On NULL handle or invalid ch_idx (>= \ref BISSC_NUM_CH_PER_SLICE_MAX)
 */
uint32_t bissc_get_current_channel(bissc_handle handle, uint32_t ch_idx);

/**
 *  \brief      Retrieves total number of channels configured
 *
 *  \details    This function returns the total number of enabled channels from the
 *              attrs structure.
 *
 *  \param[in]  handle          BiSS-C handle
 *  \retval     total_channels  total number of channels configured (1-3)
 *  \retval     0               On NULL handle or NULL attrs
 */
uint32_t bissc_get_total_channels(bissc_handle handle);

/**
 *  \brief      Clears all the encoder resolution parameters
 *
 *  \details    This function resets all encoder resolution data to zero in the private
 *              structure, including num_encoders, single_turn_len, multi_turn_len, and
 *              data_len arrays for all channels. This is typically called before
 *              reconfiguring encoder parameters.
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_clear_data_len(bissc_handle handle);

/**
 *  \brief      Update the operating baud rate as specified by the user
 *
 *  \details    This function updates the baud_rate field in the private structure
 *              with the specified frequency value. This value is used later by
 *              bissc_calc_clock() to calculate appropriate clock divisors.
 *
 *  \param[in]  handle          BiSS-C handle
 *  \param[in]  frequency       Frequency in MHz (valid values: 1, 2, 5, 8, 10)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation or invalid frequency
 */
int32_t bissc_update_clock_freq(bissc_handle handle, uint32_t frequency);

/**
 *  \brief      Configure BiSS-C clock frequency
 *
 *  \details    This function updates the clock frequency, recalculates clock divisors,
 *              reinitializes the hardware with new clock settings, and waits for the
 *              firmware to complete encoder processing delay measurement.
 *
 *              This function internally calls the following APIs:
 *              - \ref bissc_update_clock_freq : Update the baud rate in the private structure
 *              - \ref bissc_calc_clock : Calculate Rx and Tx divisors for the new frequency
 *              - \ref bissc_update_max_proc_delay : Update maximum processing delay for the new frequency
 *              - \ref bissc_hw_init : Reinitialize hardware with new clock settings
 *              - \ref bissc_wait_measure_proc_delay : Wait for firmware to measure processing delay
 *
 *  \param[in]  handle          BiSS-C handle
 *  \param[in]  frequency       Desired clock frequency in MHz (valid values: 1/2/5/8/10)
 *  \param[in]  loop_count      loop_count used when calling \ref bissc_wait_measure_proc_delay
 *
 *  \retval     SystemP_SUCCESS on successful clock configuration and delay measurement
 *  \retval     SystemP_FAILURE if clock calculation fails or delay measurement times out
 *
 */
int32_t bissc_clock_config(bissc_handle handle, uint32_t frequency, uint32_t loop_count);

/**
 *  \brief      Enable Safety for connected BiSS-C encoder
 *
 *  \details    This function enables safety mode for a specific encoder on a specific channel.
 *              It sets the has_safety flag in both the private structure and the PRU-ICSS
 *              exchange structure. When safety is enabled, the encoder provides additional
 *              CRC and error checking per the BiSS-C safety specification.
 *
 *  \param[in]  handle            BiSS-C handle
 *  \param[in]  enc_num         encoder number(index) in daisy chain
 *  \param[in]  ch_num          channel number(index)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_enable_safety(bissc_handle handle, uint32_t enc_num, uint32_t ch_num);

/**
 *  \brief      Disable Safety for connected BiSS-C encoder
 *
 *  \details    This function disables safety mode for all encoders on all channels by
 *              clearing the has_safety flags in both the private structure and the
 *              PRU-ICSS exchange structure.
 *
 *  \param[in]  handle            BiSS-C handle
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on validation failure
 */
int32_t bissc_disable_safety(bissc_handle handle);

/**
 *  \brief      Get pointer to BiSS-C attributes structure
 *
 *  \details    This function provides access to the read-only attributes structure
 *              containing configuration parameters set during initialization, such as
 *              PRU-ICSS instance, channel configuration, load share settings, and
 *              clock configuration. Returns NULL if handle is invalid.
 *
 *  \param[in]  handle            BiSS-C handle
 *  \retval     attrs             Pointer to const bissc_attrs structure, NULL if handle is invalid
 */
const bissc_attrs* bissc_get_attrs(bissc_handle handle);

/**
 *  \brief      Get pointer to BiSS-C private data structure
 *
 *  \details    This function provides access to the private data structure containing
 *              runtime state information, encoder parameters, processing delays, and
 *              pointers to PRU-ICSS resources. This function should be used with caution.
 *              Returns NULL if handle is invalid.
 *
 *  \param[in]  handle            BiSS-C handle
 *  \retval     priv              Pointer to bissc_priv structure, NULL if handle is invalid
 */
bissc_priv* bissc_get_priv(bissc_handle handle);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
