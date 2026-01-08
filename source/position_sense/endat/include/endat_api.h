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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "endat_drv.h"

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
 *  \par Validation Strategy
 *  EnDAT driver APIs use a simplified validation approach for optimal performance:
 *  - **Handle validation**: All public APIs validate the handle parameter for NULL
 *  - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *  - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *    are validated once during endat_init() and assumed valid in subsequent API calls
 */

/**
 *  \brief      Initialize the parameters data structure with defaults
 *
 *  \details    This function initializes an endat_params structure with default values
 *              for all configuration parameters. Applications should call this function
 *              before modifying specific parameters and passing to \ref endat_init.
 *
 *              Default values set by this function:
 *              - pruicss_handle: NULL (must be set by application)
 *              - max_wait_loop_count: 1000
 *              - cmd_process_delay_us: 1000 us (1 ms)
 *              - fw_wait_delay_us: 1000 us (1 ms)
 *              - channel_rx_info: NULL (must be set by application)
 *              - ch_info_global_addr: 0 (must be set by application)
 *
 *  \param[out] params  Pointer to parameters structure to initialize
 *
 *  \retval     SystemP_SUCCESS     Parameters initialized successfully
 *  \retval     SystemP_FAILURE     params is NULL
 */
int32_t endat_params_init(endat_params *params);

/**
 *  \brief      Initialize an EnDAT instance
 *
 *  \details    This function initializes an EnDAT instance by setting up the firmware
 *              interface and configuring hardware based on SysConfig parameters.
 *
 *              Prerequisites:
 *              - Application must allocate memory for channel_rx_info array
 *              - Application must set params->channel_rx_info to local address
 *              - Application must set params->ch_info_global_addr to SoC global address
 *              - Both addresses must point to the same physical memory
 *
 *              This function internally calls:
 *              - endat_hw_init : Initialize hardware interface
 *              - endat_config_channel : Configure channel mask
 *              - endat_set_default_initialization : Set default configuration
 *              - endat_config_host_trigger : Configure host trigger mode
 *
 *              Validation performed:
 *              - Validates index is within valid range (< gEndatConfigNum)
 *              - Validates params pointer is not NULL
 *              - Validates pruicss_handle in params is not NULL
 *              - Validates channel_rx_info in params is not NULL
 *              - Validates ch_info_global_addr in params is not 0
 *              - Validates max_wait_loop_count is greater than 0
 *              - Validates all attrs fields are within valid ranges
 *              - Validates priv and attrs pointers in handle are not NULL
 *
 *  \param[in]  index           Index of EnDAT handle in gEndatHandle array (0 to CONFIG_ENDAT_NUM_INSTANCES-1)
 *  \param[in]  params          Pointer to structure containing EnDAT initialization parameters
 *
 *  \retval     handle          Pointer to initialized endat_handle instance
 *  \retval     NULL            On validation failure or initialization error
 *
 *  \note       The attrs structure is populated by SysConfig and validated during this call.
 *              After successful initialization, handle->priv->is_open will be set to 1.
 */
endat_handle endat_init(uint32_t index, const endat_params *params);

/**
 *  \brief      Deinitialize an EnDAT instance
 *
 *  \details    This function marks the EnDAT instance as closed and releases resources.
 *              After calling this function, the handle should not be used for further
 *              operations until reinitialized with \ref endat_init.
 *
 *              Operations performed:
 *              - Sets priv->is_open to 0 to mark handle as closed
 *              - No hardware resources are released (PRU firmware continues running)
 *
 *  \param[in]  handle          EnDAT handle to deinitialize
 *
 *  \retval     SystemP_SUCCESS     Handle deinitialized successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       This function does not stop PRU firmware or release PRUICSS resources.
 */
int32_t endat_deinit(endat_handle handle);

/**
 *  \brief      Process raw received data and format based on the command
 *
 *  \details    This function processes the raw data received from the encoder after
 *              a command transaction and formats it into a structured format based on
 *              the command type. It extracts position data, additional information,
 *              and other encoder responses from the PRU interface buffer.
 *
 *              This function should be called after:
 *              - \ref endat_command_wait completes successfully
 *              - Raw data is available in the PRU interface buffer
 *
 *              Supported commands include:
 *              - Position commands (2.1, 2.2)
 *              - Memory read/write commands
 *              - Parameter access commands
 *              - Test commands
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  cmd         EnDAT command number identifying the transaction type
 *  \param[out] u           Pointer to union for storing formatted data based on cmd
 *                          (must not be NULL)
 *
 *  \retval     SystemP_SUCCESS     Data processed and formatted successfully
 *  \retval     SystemP_FAILURE     Handle is NULL or invalid command
 *
 *  \note       Handle parameter is validated for NULL. Internal structures
 *              are validated once during \ref endat_init and assumed valid.
 *  \note       The output union format varies based on the command type.
 *              Caller must interpret the union correctly for the given command.
 *  \note       In multi-channel mode, call \ref endat_multi_channel_set_cur to select
 *              the target channel before calling this function.
 */
int32_t endat_recvd_process(endat_handle handle, int32_t cmd,
                        endat_format_data *u);

/**
 *  \brief      Validate CRC for received data
 *
 *  \details    This function validates the CRC (Cyclic Redundancy Check) for the
 *              received data after processing. 
 *
 *              This function should be called after:
 *              - \ref endat_recvd_process has formatted the received data
 *
 *              Return value bit encoding:
 *              - Bit 0: Position/address/params/test CRC (1=success, 0=failure)
 *              - Bit 1: Additional info 1 CRC if present (1=success, 0=failure)
 *              - Bit 2: Additional info 2 CRC if present (1=success, 0=failure)
 *
 *              Example: Return value of 0x7 (0b111) indicates all CRCs passed.
 *                       Return value of 0x5 (0b101) indicates main and addinfo2
 *                       passed but addinfo1 failed.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  cmd         EnDAT command number identifying the transaction type
 *  \param[in]  u           Pointer to union having formatted data based on cmd
 *                          (must not be NULL)
 *
 *  \retval     status      CRC validation status (bitfield as described above)
 *                          - Bit 0 set: Primary data CRC valid
 *                          - Bit 1 set: Additional info 1 CRC valid (if present)
 *                          - Bit 2 set: Additional info 2 CRC valid (if present)
 *
 *  \note       Handle parameter is validated for NULL. Internal structures
 *              are validated once during \ref endat_init and assumed valid.
 *  \note       Not all commands have additional information fields. Unused bits
 *              should be ignored for commands without additional info.
 */
uint32_t endat_recvd_validate(endat_handle handle, int32_t cmd,
                              endat_format_data *u);

/**
 *  \brief      Send EnDAT command and wait for firmware completion
 *
 *  \details    This is a high-level function that combines command building, sending,
 *              and waiting for completion. It is a convenience wrapper that internally
 *              calls the following functions in sequence:
 *              - \ref endat_command_build : Set up command in PRU interface buffer
 *              - \ref endat_command_send : Trigger PRU to execute the command
 *              - \ref endat_command_wait : Wait for PRU to complete the transaction
 *
 *              This function blocks until the firmware completes the command transaction
 *              or a timeout occurs. 
 *
 *              Typical usage sequence:
 *              1. Call this function to execute command
 *              2. Call \ref endat_recvd_process to parse received data
 *              3. Call \ref endat_recvd_validate to verify CRC
 *
 *  \param[in]  handle          EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  cmd             EnDAT command number (e.g., position, memory access)
 *  \param[in]  cmd_supplement  Pointer to supplement information needed for command
 *                              setup (can be NULL for commands without supplements)
 *
 *  \retval     SystemP_SUCCESS     Command processed successfully
 *  \retval     SystemP_FAILURE     Handle is NULL or invalid parameters
 *
 *  \note       This function blocks until command completion. Use individual
 *              build/send/wait functions for non-blocking operation.
 *  \note       Handle parameter is validated for NULL. Internal structures
 *              are validated once during \ref endat_init and assumed valid.
 */
int32_t endat_command_process(endat_handle handle, int32_t cmd,
                          endat_cmd_supplement *cmd_supplement);

/**
 *  \brief      Set up EnDAT command in PRU interface buffer
 *
 *  \details    This function prepares an EnDAT command in the PRU shared interface
 *              buffer without triggering execution. It encodes the command parameters,
 *              address, data, and other supplement information into the format expected
 *              by the PRU firmware.
 *
 *              This function is the first step in a three-phase command execution:
 *              1. \ref endat_command_build : Prepare command (this function)
 *              2. \ref endat_command_send : Trigger PRU execution
 *              3. \ref endat_command_wait : Wait for completion
 *
 *              Use this function when you need fine-grained control over command
 *              timing. For simpler use cases, \ref endat_command_process combines
 *              all three phases.
 *
 *              Command types supported:
 *              - Position commands (2.1, 2.2)
 *              - Memory read/write commands
 *              - Parameter selection commands
 *              - Test mode commands
 *
 *  \param[in]  handle          EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  cmd             EnDAT command number specifying the operation type
 *  \param[in]  cmd_supplement  Pointer to supplement information (address, data, etc.)
 *                              required for the command. Can be NULL for simple commands.
 *
 *  \retval     SystemP_SUCCESS     Command built successfully in PRU buffer
 *  \retval     SystemP_FAILURE     Handle is NULL or invalid command parameters
 *
 *  \note       This function does not trigger command execution. Call
 *              \ref endat_command_send after building to execute.
 *  \note       Handle parameter is validated for NULL. Internal structures
 *              are validated once during \ref endat_init and assumed valid.
 */
int32_t endat_command_build(endat_handle handle, int32_t cmd,
                        endat_cmd_supplement *cmd_supplement);

/**
 *  \brief      Trigger PRU to execute the prepared EnDAT command
 *
 *  \details    This function signals the PRU firmware to begin executing the command
 *              that was previously set up in the interface buffer by \ref endat_command_build.
 *              It sets the appropriate trigger bit in the PRU control register to start
 *              the EnDAT transaction on the physical interface.
 *
 *              This function is the second step in a three-phase command execution:
 *              1. \ref endat_command_build : Prepare command
 *              2. \ref endat_command_send : Trigger PRU execution (this function)
 *              3. \ref endat_command_wait : Wait for completion
 *
 *              This function returns immediately without waiting for the command to
 *              complete. Use \ref endat_command_wait to wait for completion.
 *
 *              Prerequisites:
 *              - Command must be built using \ref endat_command_build first
 *              - PRU firmware must be initialized and ready
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Command sent successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       This function does not wait for command completion. Always call
 *              \ref endat_command_wait after this function to ensure the command
 *              completes before processing results.
 */
int32_t endat_command_send(endat_handle handle);

/**
 *  \brief      Wait for PRU to complete EnDAT transaction
 *
 *  \details    This function blocks until the PRU firmware completes the EnDAT
 *              command transaction that was triggered by \ref endat_command_send.
 *              It polls the PRU interface busy flag until the firmware signals
 *              completion or until the configured timeout expires.
 *
 *              This function is the third step in a three-phase command execution:
 *              1. \ref endat_command_build : Prepare command
 *              2. \ref endat_command_send : Trigger PRU execution
 *              3. \ref endat_command_wait : Wait for completion (this function)
 *
 *              The maximum wait time is controlled by the fw_wait_delay_us parameter
 *              configured during initialization. The function performs busy-wait
 *              polling for optimal timing accuracy.
 *
 *              After this function returns, the received data is available in the
 *              PRU interface buffer and can be processed using \ref endat_recvd_process.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Transaction completed successfully
 *  \retval     SystemP_FAILURE     On NULL handle, internal structure validation failure,
 *                                  zero loop count configuration, or firmware timeout
 *
 */
int32_t endat_command_wait(endat_handle handle);

/**
 *  \brief      Get encoder recovery time from memory
 *
 *  \details    This function retrieves the recovery time (tD) parameter from the
 *              PRU interface memory. 
 *
 *              The recovery time value is:
 *              - Read from the PRU interface buffer
 *              - measured and stored by firmware during latest command execution 
 *           
 *
 *              Prerequisites:
 *              - \ref endat_command_process must have been called successfully
 * 
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[out] recovery_time  Pointer to store recovery time in nanoseconds
 *
 *  \retval     SystemP_SUCCESS     Transaction completed successfully
 *  \retval     SystemP_FAILURE     On NULL handle, NULL recovery_time pointer, or internal
 *                                  structure validation failure
 *
 */
int32_t endat_get_recovery_time(endat_handle handle, uint32_t *recovery_time);

/**
 *  \brief      Query and store encoder information in handle
 *
 *  \details    This function retrieves comprehensive encoder information by executing
 *              a series of EnDAT memory read commands and stores the results in the
 *              driver handle's private data structure. This function should be called
 *              after successful initialization to populate encoder characteristics.
 *
 *              Information retrieved and stored:
 *              - Single-turn position resolution (bits)
 *              - Multi-turn position resolution (bits)
 *              - Encoder manufacturer ID
 *              - Encoder serial number
 *              - Encoder type code
 *              - Supported command set (EnDAT 2.1 or 2.2)
 *
 *              This function internally calls:
 *              - \ref endat_command_process : To execute memory read commands
 *              - \ref endat_recvd_process : To parse encoder responses
 *
 *              The retrieved information is stored in the handle's priv structure
 *              and can be accessed directly after this function succeeds.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Encoder information retrieved and stored successfully
 *  \retval     SystemP_FAILURE     Handle is NULL or command execution failed
 *
 *  \note       This function must be called after \ref endat_init and before any command is sent
 *              to ensure encoder characteristics are known.
 */
int32_t endat_get_encoder_info(endat_handle handle);

/**
 *  \brief      Get propagation delay measured by firmware
 *
 *  \details    This function retrieves the cable propagation delay that was
 *              automatically estimated by the PRU firmware during initialization.
 *              The propagation delay accounts for signal transmission time through
 *              the cable between the controller and encoder.
 *
 *              The propagation delay is:
 *              - Automatically measured during firmware initialization
 *              - Used to compensate timing for accurate communication
 *   
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[out] prop_delay  Pointer to store propagation delay in pru cycle count 
 *
 *  \retval     SystemP_SUCCESS     Propagation delay retrieved successfully
 *  \retval     SystemP_FAILURE     On NULL handle, NULL prop_delay pointer, or read failure
 *
 *  \note       The propagation delay is measured once during initialization.
 */
int32_t endat_get_prop_delay(endat_handle handle, uint32_t *prop_delay);

/**
 *  \brief      Track additional information presence in encoder response
 *
 *  \details    This function updates the driver's internal tracking of which
 *              additional information fields are present in encoder responses.
 *              EnDAT encoders can include up to two additional information words
 *              (addinfo1, addinfo2) along with position data, and this function
 *              helps the driver know when to expect and process these fields.
 *
 *
 *              The tracking information is stored in the handle and used by:
 *              - \ref endat_recvd_process : To parse the correct number of fields
 *              - \ref endat_recvd_validate : To validate CRC for all present fields
 *
 *
 *  \param[in]  handle          EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  cmd             EnDAT command number
 *  \param[in]  cmd_supplement  Pointer to command supplement containing addinfo flags
 *
 *  \retval     SystemP_SUCCESS     Additional info tracking updated successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Not all EnDAT commands support additional information.
 */
int32_t endat_addinfo_track(endat_handle handle, int32_t cmd,
                         endat_cmd_supplement *cmd_supplement);

/**
 *  \brief      Configure propagation delay for EnDAT channel
 *
 *  \details    This function configures the propagation delay compensation for the
 *              currently selected EnDAT channel.
 *
 *              The delay value is specified in nanoseconds and is internally converted
 *              to delay register counter cycles. This function:
 *              - Converts nanosecond delay to delay register counter value
 *              - Updates the propagation delay tracking
 *              - Configures the channel-specific delay register
 *
 *
 *  \param[in]  handle  EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  val     Propagation delay value in nanoseconds
 *
 *  \retval     SystemP_SUCCESS     Propagation delay configured successfully
 *  \retval     SystemP_FAILURE     On NULL handle or internal configuration error
 *
 *  \note       In multi-channel mode, call endat_multi_channel_set_cur() first to
 *              select the target channel before calling this function.
 *  \note       The delay value is applied to the current_channel in the driver state.
 */
int32_t endat_config_propagation_delay(endat_handle handle, uint32_t val);

/**
 *  \brief      Configure EnDAT clock frequency
 *
 *  \details    This function configures the clock frequency used for EnDAT serial
 *              communication.
 *
 *              The clock configuration includes:
 *              - Clock frequency divider settings
 *              - Timing compensation parameters
 *
 *              This function:
 *              - Updates PRU 3 channel interface clock registers
 *              
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  freq        Clock frequency in Hz
 *
 *  \retval     SystemP_SUCCESS     Clock configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL or invalid clock frequency
 *
 *  \note       Clock frequency changes take effect immediately. Ensure no
 *              transactions are in progress before calling this function.
 *  \note       To get accurate clock frequency, the divisor should be an integer
 *              for both RX and TX clock.
 *
 *  \note       The RX clock frequency is always configured as 8x the endat clock.
 */  
int32_t endat_config_clock(endat_handle handle, uint32_t freq);

/**
 *  \brief      Configure tST delay 
 *
 *  \details    This function configures the tST (start delay) timing parameter
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  delay       tST delay value in delay register counter units
 *
 *  \retval     SystemP_SUCCESS     tST delay configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Incorrect tST delay can cause communication errors or CRC failures.
 *  \note       delay register counter units (5 ns)
 */
int32_t endat_config_tst_delay(endat_handle handle, uint16_t delay);

/**
 *  \brief      Configure RX arm counter timing
 *
 *  \details    This function configures the receiver arm counter, which determines
 *              when the PRU firmware begins sampling the incoming data from the encoder.
 *              Proper configuration ensures the receiver is ready before data arrives
 *              and samples at the optimal point in each bit period.
 *
 *              The RX arm counter controls:
 *              - When the receiver starts looking for incoming data
 *
 *              The value is typically calculated based on:
 *              - Clock frequency
 *              - Propagation delay
 *              - Encoder timing specifications
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  val         RX arm counter value in delay register counter units
 *
 *  \retval     SystemP_SUCCESS     RX arm counter configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       This parameter affects data sampling timing. Incorrect values
 *              can cause bit errors or CRC failures.
 *  \note       This function operates on the currently selected channel. Use
 *              \ref endat_config_channel to select channel before calling.
 *  \note       delay register counter units (5 ns)
 */
int32_t endat_config_rx_arm_cnt(endat_handle handle, uint16_t val);

/**
 *  \brief      Configure cable/wire delay compensation for selected channel
 *
 *  \details    This function configures the cable delay compensation for the currently
 *              selected EnDAT channel. 
 *
 *         
 *
 *
 *              In multi-channel configurations, different channels may have
 *              different cable lengths requiring individual compensation.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  val         Wire delay in delay register counter units
 *
 *  \retval     SystemP_SUCCESS     Wire delay configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       This function operates on the currently selected channel. Use
 *              \ref endat_config_channel to select channel before calling.
 *  \note       Incorrect wire delay can cause timing errors and CRC failures.
 *  \note       delay register counter units (5 ns)
 */
int32_t endat_config_wire_delay(endat_handle handle, uint16_t val);

/**
 *  \brief      Configure RX clock disable count for tD timing
 *
 *  \details    This function configures the number of clock cycles to be disabled
 *              at the end of the receive phase to account for tD (cable delay time).
 *
 *
 *              The value represents how many clock cycles should be suppressed
 *              at the end of reception to maintain proper timing margins.
 *
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  val         Number of endat clock cycles to disable at end of RX
 *
 *  \retval     SystemP_SUCCESS     RX clock disable configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Incorrect configuration may violate encoder timing requirements
 *              and cause protocol errors.
 */
int32_t endat_config_rx_clock_disable(endat_handle handle,
                                   uint16_t val);

/**
 *  \brief      Start continuous position update mode
 *
 *  \details    This function starts continuous mode operation where the encoder
 *              automatically sends position updates at regular intervals without
 *              requiring individual command transactions. 
 *
 *              In continuous mode:
 *              - Position data is automatically transmitted by encoder
 *              - No mode commands are sent for each update
 *              - Position data is continuously available in PRU buffer
 *
 *              This function configures:
 *              - PRU firmware for continuous mode operation
 *              - Encoder to enable continuous transmission
 *
 *
 *              Use \ref endat_stop_continuous_mode to return to command mode.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Continuous mode started successfully
 *  \retval     SystemP_FAILURE     Handle is NULL or mode transition failed
 *
 *  \note       Encoder must support continuous mode (EnDAT 2.2 required).
 *  \note       In continuous mode, command transactions cannot be executed.
 *              Stop continuous mode first to send commands.
 */
int32_t endat_start_continuous_mode(endat_handle handle);

/**
 *  \brief      Stop continuous position update mode
 *
 *  \details    This function stops continuous mode operation and returns the encoder
 *              and PRU firmware to normal command mode.
 *
 *              This function:
 *              - Disables continuous transmission in the encoder
 *              - Reconfigures PRU firmware for command mode operation
 *
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Continuous mode stopped successfully
 *  \retval     SystemP_FAILURE     On NULL handle, internal structure validation failure,
 *                                  or firmware command completion timeout
 *
 *  \note       Use \ref endat_start_continuous_mode to restart continuous mode.
 */
int32_t endat_stop_continuous_mode(endat_handle handle);

/**
 *  \brief      Configure EnDAT for host trigger mode
 *
 *  \details    This function configures the EnDAT to operate in host trigger
 *              mode, where position updates are initiated by explicit host commands
 *              rather than by periodic timers or external hardware triggers.
 *
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Host trigger mode configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 */
int32_t endat_config_host_trigger(endat_handle handle);

/**
 *  \brief      Configure EnDAT for periodic trigger using IEP compare mode
 *
 *  \details    This function configures the EnDAT to operate with periodic
 *              triggers generated by IEP (Industrial Ethernet Peripheral) compare
 *              events. In this mode, position updates are automatically triggered
 *              at precise intervals defined by IEP timer compare registers.
 *
 *           
 *              Operation:
 *              - IEP timer continuously counts
 *              - When counter matches compare value, position update triggers
 *             
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Periodic trigger compare mode configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 */
int32_t endat_config_periodic_trigger_cmp_mode(endat_handle handle);

/**
 *  \brief      Select active channel for EnDAT operation
 *
 *  \details    This function selects which physical encoder channel the EnDAT
 *              will use for subsequent operations. In single-channel mode, this function
 *              sets the active channel. In multi-channel configurations, it determines
 *              which channel will be used for command and configuration operations.
 *
 *              After calling this function:
 *              - All command transactions target the selected channel
 *              - Configuration operations apply to the selected channel
 *              - Position data is read from the selected channel
 *
 *
 *              In multi-channel scenarios, use \ref endat_multi_channel_set_cur to
 *              switch between channels when processing data from multiple encoders.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  ch          Channel number to select (0-2 depending on connected hardware)
 *
 *  \retval     SystemP_SUCCESS     Channel configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL or invalid channel number
 *
 */
int32_t endat_config_channel(endat_handle handle, uint32_t ch);

/**
 *  \brief      Configure channel mask for multi-channel operation
 *
 *  \details    This function configures which encoder channels are active in a
 *              multi-channel configuration. The channel mask enables simultaneous
 *              operation of multiple encoder channels with coordinated timing and
 *              optional load sharing for heterogeneous encoders.
 *
 *              The mask parameter specifies active channels:
 *              - Bit 0: Channel 0
 *              - Bit 1: Channel 1
 *              - Bit 2: Channel 2
 *              - Example: 0x03 (0b011) enables channels 0 and 1
 *              - Example: 0x07 (0b111) enables all three channels
 *
 *              The loadshare parameter controls load sharing mode:
 *              - 0: Load share disabled 
 *              - 1: Load share enabled 
 *
 *
 *              This function should be called during initialization to set up
 *              the multi-channel configuration before loading the firmware and starting transactions.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  mask        Channel mask (bit 0 = channel 0, bit 1 = channel 1, etc.)
 *  \param[in]  loadshare   Load share mode enable (0 = disabled, 1 = enabled)
 *
 *  \retval     SystemP_SUCCESS     Multi-channel mask configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Use loadshare=1 when encoders have different characteristics.
 *  \note       See \ref endat_enable_load_share_mode for more on load sharing.
 */
int32_t endat_config_multi_channel_mask(endat_handle handle,
                                     uint8_t mask,
                                     uint8_t loadshare);

/**
 *  \brief      Get detected channels in multi-channel configuration
 *
 *  \details    This function retrieves the mask of encoder channels that were
 *              successfully detected by the firmware during initialization. 
 *
 *
 *              Return value interpretation:
 *              - Bit 0: Channel 0 detected (1) or not detected (0)
 *              - Bit 1: Channel 1 detected (1) or not detected (0)
 *              - Bit 2: Channel 2 detected (1) or not detected (0)
 *              - Example: 0x05 (0b101) means channels 0 and 2 detected, channel 1 failed
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     mask        Bitmask of successfully detected channels
 *                          (bit 0 = channel 0, bit 1 = channel 1, etc.)
 *  \retval     0           On NULL handle or no channels detected
 *
 *  \note       This function reads the detection status from PRU firmware memory.
 *  \note       Only needed for diagnostic purposes or after initialization failure.
 */
uint8_t endat_multi_channel_detected(endat_handle handle);

/**
 *  \brief      Set priv->current channel for multi-channel data processing
 *
 *  \details    In multi-channel configurations, this function selects which channel's
 *              data should be processed when calling send and receive processing APIs. After a
 *              multi-channel transaction completes, call this function to iterate through
 *              each channel and process its data separately.
 *
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[in]  ch          Channel number to select for data processing (0-2)
 *
 *  \retval     SystemP_SUCCESS     Current channel set successfully
 *  \retval     SystemP_FAILURE     handle is NULL or invalid channel number
 *
 *  \note       Must be called before processing each channel's data in multi-channel mode.
 */
int32_t endat_multi_channel_set_cur(endat_handle handle, uint32_t ch);

/**
 *  \brief      Wait for EnDAT firmware initialization to complete
 *
 *  \details    This function blocks until the PRU firmware completes its initialization
 *              sequence for the specified encoder channels, or until the timeout expires.
 *              Firmware initialization includes encoder detection, timing calibration,
 *              and establishment of communication.
 *
 *              The mask parameter specifies which channels to wait for:
 *              - Bit 0: Wait for channel 0
 *              - Bit 1: Wait for channel 1
 *              - Bit 2: Wait for channel 2
 *              - Example: 0x03 waits for channels 0 and 1
 *
 *              This function is typically called immediately after firmware load done and PRU core is enabled
 *
 *              If initialization fails, use \ref endat_multi_channel_detected to
 *              determine which channels failed to initialize.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \param[in]  timeout     Timeout for iterations. Each iteration of the wait loop
 *                          uses priv->fw_wait_delay_us microseconds.
 *  \param[in]  mask        Channel mask indicating which channels to wait for
 *                          (bit 0 = channel 0, bit 1 = channel 1, etc.)
 *
 *  \retval     SystemP_SUCCESS     All specified channels initialized successfully
 *  \retval     SystemP_FAILURE     Handle is NULL, timeout occurred, or initialization failed
 *
 *  \note       This function blocks until initialization completes or timeout.
 */
int32_t endat_wait_initialization(endat_handle handle, uint32_t timeout, uint8_t mask);
/**
 *  \brief      Initialize recovery time measurement subsystem
 *
 *  \details    This function initializes the recovery time measurement feature,
 *              which monitors the encoder's recovery time (tD) parameter during
 *              operation.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Recovery time measurement initialized successfully
 *  \retval     SystemP_FAILURE     On NULL handle or internal structure validation failure
 *
 *  \note       Call this function once during initialization.
 */
int32_t endat_init_rt_measurement (endat_handle handle);

/**
 *  \brief      Validate measured recovery time against acceptable range
 *
 *  \details    This function checks whether the most recently measured recovery time
 *              falls within the acceptable range defined by the encoder's specifications
 *              and configured limits.
 *
 *              The function compares measured recovery time against:
 *              - Short recovery time range (2.45-3.75 μs)
 *              - Long recovery time range (18.5-30.0 μs)
 *
 *              Error codes returned via error_code parameter:
 *              - ENDAT_RT_NO_ERROR (0x0): Recovery time is within acceptable range
 *              - ENDAT_RT_OUT_OF_RANGE_ERROR (0x1): Recovery time is outside valid ranges
 *              - ENDAT_RT_COUNTER_STUCK_ERROR (0x2): Recovery time counter is not incrementing
 *
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[out] error_code  Pointer to store recovery time error code
 *
 *  \retval     SystemP_SUCCESS     Recovery time check completed successfully
 *  \retval     SystemP_FAILURE     On NULL handle, NULL error_code pointer, or internal
 *                                  structure validation failure
 *
 */
int32_t endat_check_rt_error(endat_handle handle, int8_t *error_code);

/**
 *  \brief      Disable recovery time measurement
 *
 *  \details    This function disables the recovery time counter feature,
 *              stopping the firmware from updating the encoder
 *              recovery time (tD) counter during transactions.
 *
 *              Use \ref endat_enable_rt_measurement to re-enable measurement.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Recovery time measurement disabled successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Measurement must have been initialized with \ref endat_init_rt_measurement.
 */
int32_t endat_disable_rt_measurement(endat_handle handle);

/**
 *  \brief      Enable recovery time counter
 *
 *  \details    This function enables the recovery time counter feature,
 *              instructing the firmware to monitor and record encoder recovery
 *              time (tD) during each transaction. The measured values can be
 *              validated using \ref endat_check_rt_error.
 *
 *              After enabling:
 *              - Firmware update recovery time counter on each transaction
 *              - \ref endat_check_rt_error provides valid validation results
 *              - \ref endat_status_rt_measurement returns 1
 *
 *              Prerequisites:
 *              - \ref endat_init_rt_measurement must be called first
 *
 *              Use \ref endat_disable_rt_measurement to stop measurement.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Recovery time measurement enabled successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Recovery time counter must be initialized before enabling.
 */
int32_t endat_enable_rt_measurement (endat_handle handle);
/**
 *  \brief      Get recovery time counter enable status
 *
 *  \details    This function returns the current enable/disable status of the
 *              recovery time counter.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *  \param[out] status      Pointer to store measurement enable status (1 = enabled, 0 = disabled)
 *
 *  \retval     SystemP_SUCCESS     Status retrieved successfully
 *  \retval     SystemP_FAILURE     On NULL handle, NULL status pointer, or internal
 *                                  structure validation failure
 *
 */
int32_t endat_status_rt_measurement(endat_handle handle, uint32_t *status);
/**
 *  \brief      Configure EnDAT for periodic trigger using IEP capture mode
 *
 *  \details    This function configures the EnDAT to operate with periodic
 *              triggers generated by IEP (Industrial Ethernet Peripheral) capture
 *              events.
 *
 *              Use \ref endat_config_iep_cap_event to configure specific capture
 *              events and assign them to channels.
 *
 *  \param[in]  handle      EnDAT driver handle obtained from \ref endat_init
 *
 *  \retval     SystemP_SUCCESS     Periodic trigger capture mode configured successfully
 *  \retval     SystemP_FAILURE     handle is NULL
 *
 *  \note       Configure specific capture events using \ref endat_config_iep_cap_event.
 */
int32_t endat_config_periodic_trigger_cap_mode(endat_handle handle);

/**
 *  \brief      Configure IEP CAP event for periodic trigger (DMEM configuration only)
 *
 *  \details    This function configures the IEP capture event information in PRU shared
 *              memory (DMEM) for firmware access. It writes the capture register address
 *              and event number to trigger_params structure. This function does NOT configure
 *              IEP hardware registers.
 *
 *  \param[in]  handle      EnDAT handle
 *  \param[in]  channel     EnDAT channel number (0-2). Used in load share mode,
 *                          ignored in single PRU mode (always uses index 0).
 *  \param[in]  event_num   CAP event number (0-7)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 * 
 *  \note       This is configured during the endat_init call. 
 */
int32_t endat_config_iep_cap_event(endat_handle handle, uint8_t channel, uint8_t event_num);

/**
 *  \brief      Configure IEP CMP event for periodic trigger (DMEM configuration only)
 *
 *  \details    This function configures the IEP compare event information in PRU shared
 *              memory (DMEM) for firmware access. It writes the event number to trigger_params
 *              structure. This function does NOT configure IEP hardware registers.
 *
 *  \param[in]  handle      EnDAT handle
 *  \param[in]  channel     EnDAT channel number (0-2). Used in load share mode,
 *                          ignored in single PRU mode (always uses index 0).
 *  \param[in]  event_num   CMP event number (0-15)
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE otherwise
 * 
 *  \note       This is configured during the endat_init call. 
 */
int32_t endat_config_iep_cmp_event(endat_handle handle, uint8_t channel, uint8_t event_num);

/**
 *  \brief      Get pointer to EnDAT attributes (compile-time configuration)
 *
 *  \details    This function provides access to the compile-time configuration
 *              parameters populated from SysConfig. The returned pointer is const
 *              to prevent modification of configuration data.
 *
 *  \param[in]  handle  EnDAT driver handle obtained from endat_init()
 *
 *  \retval     Pointer to const endat_attrs structure, or NULL if handle is invalid
 *
 *  \note       This function performs NULL validation on the handle parameter only.
 *              The attrs pointer is validated once during endat_init().
 */
const endat_attrs* endat_get_attrs(endat_handle handle);

/**
 *  \brief      Get pointer to EnDAT private data (runtime state)
 *
 *  \details    This function provides access to the runtime state and results
 *              data for the EnDAT instance. The returned pointer can be used
 *              to access encoder information, position data, and other runtime
 *              state variables.
 *
 *  \param[in]  handle  EnDAT driver handle obtained from endat_init()
 *
 *  \retval     Pointer to endat_priv structure, or NULL if handle is invalid
 *
 *  \note       This function performs NULL validation on the handle parameter only.
 *              The priv pointer is validated once during endat_init().
 */
endat_priv* endat_get_priv(endat_handle handle);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
