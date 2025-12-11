/*
 *  Copyright (C) 2022-2025 Texas Instruments Incorporated
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

#ifndef TAMAGAWA_DRV_H_
#define TAMAGAWA_DRV_H_

/**
 *  \defgroup TAMAGAWA_API_MODULE APIs for Tamagawa Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for Tamagawa encoder communication protocol
 *
 *  \par Validation Strategy
 *  Tamagawa driver APIs use a simplified validation approach for optimal performance:
 *  - **Handle validation**: All public APIs validate the handle parameter for NULL
 *  - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *  - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *    are validated once during \ref tamagawa_init and assumed valid in subsequent API calls
 *  - This strategy reduces overhead in time-critical data path functions.
 *
 *  @{
 */

/**
 *  \ingroup TAMAGAWA_API_MODULE
 *  \defgroup TAMAGAWA_INTERFACE_MODULE Tamagawa PRU-ICSS Interface Structures
 *
 *  This module contains structures that define the PRU-ICSS firmware interface
 *  for Tamagawa encoder communication. These structures are mapped to PRU DRAM
 *  and provide the communication interface between ARM and PRU cores.
 *
 *  @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief  Maximum number of channels supported per PRU slice
 */
#define TAMAGAWA_MAX_CHANNELS (3)

/** \brief Single PRU - Single channel configuration mode
 *
 *  Only one channel (ch0, ch1, or ch2) is used with a single PRU core.
 *  This is the simplest configuration for single encoder applications.
 */
#define TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU    (0U)

/** \brief Single PRU - Multichannel configuration mode
 *
 *  Multiple channels (up to 3: ch0, ch1, ch2) are managed by a single PRU core.
 *  All channels share the same PRU core resources without load sharing.
 */
#define TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU     (1U)

/**
 *  \brief  Tamagawa operation mode: Periodic trigger
 *
 *  In periodic mode, the PRU firmware automatically triggers position readout
 *  at regular intervals configured by IEP timer.
 */
#define TAMAGAWA_OPMODE_PERIODIC                    (0x0U)

/**
 *  \brief  Tamagawa operation mode: Host trigger
 *
 *  In host trigger mode, the R5F host processor explicitly triggers
 *  each position readout by setting the trigger bit.
 */
#define TAMAGAWA_OPMODE_HOST_TRIGGER                (0x1U)

/** \brief Allowed Tamagawa communication frequency: 2.5 MHz */
#define TAMAGAWA_FREQ_2_5_MHZ                       (2500000U)

/** \brief Allowed Tamagawa communication frequency: 5 MHz */
#define TAMAGAWA_FREQ_5_MHZ                         (5000000U)

/**
 *  \brief  Maximum EEPROM address that can be used for EEPROM Read/Write
 */
#define TAMAGAWA_MAX_EEPROM_ADDRESS (127)

/**
 *  \brief  Maximum value that can be written to EEPROM
 */
#define TAMAGAWA_MAX_EEPROM_WRITE_DATA (255)

/**
 *  \brief  RX oversampling rate. Set 7 for 8x oversampling.
 */
#define TAMAGAWA_RX_OVERSAMPLING_RATE (7)

/**
 *  \brief  Delay counter increment value. 5 ns based on 200 MHz clock,
 *          as Three Channel Peripheral interface needs this value in
 *          200 MHz clock units.
 */
#define TAMAGAWA_DELAY_COUNTER_INCREMENT (5)

/**
 *  \brief  Number of bytes in CRC calculation for EEPROM Write (CF + ADF + EDF)
 */
#define TAMAGAWA_EEPROM_WRITE_CRC_BYTES (3)

/**
 *  \brief  Number of bytes in CRC calculation for EEPROM Read (CF + ADF)
 */
#define TAMAGAWA_EEPROM_READ_CRC_BYTES (2)

/**
 *  \brief  Number of bits in a byte
 */
#define TAMAGAWA_BITS_PER_BYTE (8)

/**
 *  \brief  CRC calculation array size for storing frame data
 */
#define TAMAGAWA_CRC_DATA_ARRAY_SIZE (12)

/**
 *  \brief  Default command process delay in microseconds
 *
 *  This delay is used in command wait loops to prevent busy-waiting and
 *  allow timeout detection. Can be overridden via tamagawa_params.
 */
#define TAMAGAWA_DEFAULT_CMD_WAIT_DELAY_US (100)

/**
 *  \brief  Default maximum wait loop count
 *
 *  Maximum number of wait loop iterations in \ref tamagawa_command_wait to detect
 *  communication failures. The actual timeout is: max_wait_loop_count × cmd_wait_delay_us.
 *  With defaults (50 × 100 us = 5000 us). Can be overridden via tamagawa_params.
 */
#define TAMAGAWA_DEFAULT_MAX_WAIT_LOOP_COUNT (50U)

/**
 *  \brief  Tamagawa EEPROM Control Field value for Write operation
 */
#define TAMAGAWA_CF_EEPROM_WRITE (0x32U)

/**
 *  \brief  Tamagawa EEPROM Control Field value for Read operation
 */
#define TAMAGAWA_CF_EEPROM_READ (0xEAU)

/**
 *  \brief  Tamagawa Data ID codes
 */
typedef enum data_id_e
{
    DATA_ID_0,  /**< Data readout: data in one revolution */
    DATA_ID_1,  /**< Data readout: multi-turn data */
    DATA_ID_2,  /**< Data readout: encoder ID */
    DATA_ID_3,  /**< Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
    DATA_ID_6,  /**< EEPROM write */
    DATA_ID_7,  /**< Reset */
    DATA_ID_8,  /**< Reset */
    DATA_ID_C,  /**< Reset */
    DATA_ID_D,  /**< EEPROM read */
    PERIODIC_TRIGGER_CMD, /**< Periodic trigger command */
    DATA_ID_NUM /**< Number of Data ID codes */
} data_id;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining Tamagawa per channel interface
 *
 *    \details  Firmware per channel interface containing received data words
 *              and calculated CRC for each channel
 */
typedef struct tamagawa_ch_info_s
{
    volatile uint32_t pos_word0;
        /**< Word 0 for receiving RX data */
    volatile uint32_t pos_word1;
        /**< Word 1 for receiving RX data */
    volatile uint32_t pos_word2;
        /**< Word 2 for receiving RX data */
    volatile uint32_t cal_crc;
        /**< Word for receiving the calculated CRC */
} tamagawa_ch_info;

/**
 *    \brief    Structure defining Tamagawa command interface
 *
 *    \details  Firmware command interface used to send commands to the encoder
 */
typedef struct tamagawa_cmd_s
{
    volatile uint32_t word0;
        /**< Command word 0: [Byte 0] control field */
    volatile uint32_t word1;
        /**< Command word 1: [Byte 0] No. of TX frames, [Byte 1] No. of RX frames */
} tamagawa_cmd;

/**
 *    \brief    Structure defining Tamagawa configuration interface
 *
 *    \details  Firmware configuration interface for operation mode and channel selection
 */
typedef struct tamagawa_fw_config_s
{
    volatile uint8_t opmode;
        /**< Operation mode selection: 0 - periodic trigger, 1 - host trigger */
    volatile uint8_t channel;
        /**< Channel mask (1 << channel), 0 < channel < 3. This must be selected before running firmware.
             Once initialization is complete, it will reflect the detected channels in the selected mask.
             Multi-channel can have up to 3 channels selected, single channel only one */
    volatile uint8_t trigger;
        /**< Command trigger: Set LSB to send command, will be cleared upon command completion.
             Note that command must be set up before trigger */
    volatile uint8_t status;
} tamagawa_fw_config;

/**
 * \brief Tamagawa received frame data
 *
 * \details This structure contains all the parsed fields from a received Tamagawa frame
 */
typedef struct tamagawa_rx_frames_s
{
    uint32_t abs;
        /**< Data in one revolution (absolute position) */
    uint32_t abm;
        /**< Multi-turn data (absolute multi-turn) */
    uint8_t cf;
        /**< Control Frame */
    uint8_t sf;
        /**< Status Frame */
    uint8_t enid;
        /**< Encoder ID */
    uint8_t almc;
        /**< Encoder error */
    uint8_t adf;
        /**< EEPROM address */
    uint8_t edf;
        /**< EEPROM data */
    uint8_t crc;
        /**< CRC */
} tamagawa_rx_frames;

/**
 * \brief Tamagawa main interface structure
 *
 * \details This structure contains the main firmware interface parameters including
 *          channel configuration, clock settings, and received data
 */
typedef struct tamagawa_interface_s
{
    uint8_t ch_mask;
        /**< Mask for what channels are required */
    volatile uint32_t rx_div_factor;
        /**< RX divide factor */
    volatile uint32_t tx_div_factor;
        /**< TX divide factor */
    volatile uint32_t oversample_rate;
        /**< Oversampling rate */
    uint32_t version;
        /**< Firmware version */
    uint8_t data_id;
        /**< Data ID code */
    tamagawa_rx_frames rx_frames_received;
        /**< Received data frames */
    uint8_t tx_frames;
        /**< Number of TX frames */
    uint8_t rx_frames;
        /**< Number of RX frames */
} tamagawa_interface;

/**
 * \brief Tamagawa channel configuration structure
 *
 * \details Configuration for individual channel enable/disable
 */
typedef struct tamagawa_channel_config_s
{
    uint8_t ch0;
        /**< Configuration for channel 0 */
    uint8_t ch1;
        /**< Configuration for channel 1 */
    uint8_t ch2;
        /**< Configuration for channel 2 */
} tamagawa_channel_config;

/**
 * \brief Tamagawa EEPROM interface structure
 *
 * \details Structure containing EEPROM command parameters and TX data preparation
 */
typedef struct tamagawa_eeprom_interface_s
{
    volatile uint32_t cmd;
        /**< Holds the value of command ID for EEPROM commands */
    volatile uint32_t adf;
        /**< Holds the value of ADF (EEPROM address) for EEPROM commands */
    volatile uint32_t edf;
        /**< Holds the value of EDF (EEPROM data) for EEPROM Write command */
    volatile uint32_t crc;
        /**< Holds the value of CRC for EEPROM commands */
    volatile uint32_t word0;
        /**< Used for CRC calculation */
    volatile uint32_t word1;
        /**< Used for CRC calculation */
    volatile uint32_t word2;
        /**< Used for CRC calculation */
    uint64_t eeprom_tx_data;
        /**< Used to store the bits for TX in EEPROM read/write */
} tamagawa_eeprom_interface;

/**
 *    \brief    Structure defining complete Tamagawa PRU-ICSS exchange interface
 *
 *    \details  This is the top-level structure mapped to PRU DRAM that contains
 *              all firmware interfaces for configuration, command, per-channel data,
 *              and EEPROM operations
 */
typedef struct tamagawa_xchg_s
{
    tamagawa_fw_config config;
        /**< Firmware configuration interface */
    tamagawa_cmd cmd;
        /**< Command interface */
    tamagawa_ch_info ch[TAMAGAWA_MAX_CHANNELS];
        /**< Per-channel interface array (3 channels) */
    tamagawa_interface tamagawa_interface;
        /**< Main Tamagawa interface */
    tamagawa_eeprom_interface tamagawa_eeprom_interface[TAMAGAWA_MAX_CHANNELS];
        /**< Tamagawa interface for EEPROM commands (per channel) */
} tamagawa_xchg;

/**
 * \brief   Tamagawa clock configuration structure
 *
 * \details Used to configure the Tamagawa clock dividers and source selection
 */
typedef struct tamagawa_clk_cfg_s
{
    /** RX divide factor */
    uint16_t rx_div;
    /** TX divide factor */
    uint16_t tx_div;
    /** RX oversample rate */
    uint16_t rx_os_rate;
    /** RX clock source (0: UART clock, 1: Core clock) */
    uint8_t rx_clk_source;
    /** TX clock source (0: UART clock, 1: Core clock) */
    uint8_t tx_clk_source;
    /** RX enable counter */
    uint16_t rx_en_cnt;
} tamagawa_clk_cfg;

/**
 * \brief   Tamagawa attributes structure
 *
 * \details Contains compile-time configuration parameters from SysConfig.
 *          This structure is read-only and populated by SysConfig-generated code.
 */
typedef struct tamagawa_attrs_s
{
    /** Tamagawa instance index */
    uint8_t instance;
    /**< Tamagawa configuration mode.
     *   0 = TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU (one channel, one PRU)
     *   1 = TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU (multiple channels, one PRU) */
    uint8_t mode;
    /** PRU-ICSS instance (0 or 1) */
    uint8_t pruicss_instance;
    /** PRU slice (0 = PRU0, 1 = PRU1) */
    uint8_t pruicss_slice;
    /** Channel mask indicating enabled channels (bit 0: Ch0, bit 1: Ch1, bit 2: Ch2) */
    uint8_t channel_mask;
    /** Channel 0 enabled flag (0 or 1) */
    uint8_t channel0_enabled;
    /** Channel 1 enabled flag (0 or 1) */
    uint8_t channel1_enabled;
    /** Channel 2 enabled flag (0 or 1) */
    uint8_t channel2_enabled;
    /** Total number of channels enabled (1-3) */
    uint8_t total_channels;
    /** Baud rate in Hz (typically 2500000 or 5000000) */
    uint32_t baud_rate;
    /** PRU core clock frequency in Hz */
    uint32_t core_clk_freq;
    /** PRU UART clock frequency in Hz */
    uint32_t uart_clk_freq;
    /** IEP clock frequency in Hz */
    uint32_t iep_clk_freq;
    /** Clock source selection (0: UART clock, 1: Core clock) */
    uint8_t is_core_clk;
} tamagawa_attrs;

/**
 * \brief   Tamagawa private runtime state structure
 *
 * \details Contains runtime state information for a Tamagawa instance.
 *          This structure is initialized by the driver during \ref tamagawa_init.
 */
typedef struct tamagawa_priv_s
{
    /** Flag to track if instance is initialized (1 = open, 0 = closed) */
    uint8_t is_open;
    /** Currently selected channel ID (0-2) */
    uint8_t channel;
    /** Pointer to PRU-ICSS firmware interface structure (mapped to PRU DRAM) */
    tamagawa_xchg *tamagawa_xchg;
    /** Tamagawa clock configuration */
    tamagawa_clk_cfg clk_cfg;
    /** PRU-ICSS handle
      *  Copied from params in \ref tamagawa_init. */
    PRUICSS_Handle pruicss_handle;
    /** Command process delay in microseconds (for timeout handling)
     *  Copied from params in \ref tamagawa_init. */
    uint32_t cmd_wait_delay_us;
    /** Maximum wait loop iteration count.
     *  Used in \ref tamagawa_command_wait to detect communication failures.
     *  Actual timeout = max_wait_loop_count × cmd_wait_delay_us microseconds
     *  Copied from params in \ref tamagawa_init. */
    uint32_t max_wait_loop_count;
} tamagawa_priv;

/**
 * \brief   Tamagawa configuration structure (handle)
 *
 * \details This structure combines the read-only attributes (from SysConfig)
 *          with the runtime private state. It serves as the Tamagawa handle.
 */
typedef struct tamagawa_config_s
{
    /** Pointer to private runtime state */
    tamagawa_priv *priv;
    /** Pointer to read-only attributes (from SysConfig) */
    const tamagawa_attrs *attrs;
} tamagawa_config;

/**
 * \brief   Tamagawa handle type
 *
 * \details Opaque pointer to a Tamagawa configuration structure.
 *          Returned by \ref tamagawa_init and used in all Tamagawa APIs.
 */
typedef tamagawa_config *tamagawa_handle;

/**
 * \brief   Tamagawa initialization parameters structure
 *
 * \details Contains runtime parameters passed to \ref tamagawa_init.
 *          Initialize with \ref tamagawa_params_init before use.
 */
typedef struct tamagawa_params_s
{
    /** PRU-ICSS handle (must be valid, obtained from PRUICSS_open) */
    PRUICSS_Handle pruicss_handle;
    /** Command process delay in microseconds (used in command wait timeout loop)
     *  Default: 100 us */
    uint32_t cmd_wait_delay_us;
    /** Maximum wait loop iteration count.
     *  Used in \ref tamagawa_command_wait to detect communication failures.
     *  Actual timeout = max_wait_loop_count × cmd_wait_delay_us microseconds.
     *  Default: 50 (gives 50 × 100 us = 5000 us with default cmd_wait_delay_us) */
    uint32_t max_wait_loop_count;
} tamagawa_params;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief      Initialize Tamagawa parameters structure with default values
 *
 *  \details    This function initializes a \ref tamagawa_params structure with default
 *              values. Call this function before setting custom parameters and passing
 *              to \ref tamagawa_init.
 *
 *  \param[out] params          Pointer to \ref tamagawa_params structure to initialize
 *
 */
void tamagawa_params_init(tamagawa_params *params);

/**
 *  \brief      Initialize a Tamagawa instance
 *
 *  \details    This function initializes a Tamagawa instance by setting up the firmware
 *              interface and configuring hardware based on \ref tamagawa_attrs and \ref tamagawa_params.
 *              The function validates all input parameters and initializes the PRU-ICSS interface.
 *
 *              This function performs the following operations:
 *              - Validates index against \ref gTamagawaConfigNum
 *              - Validates PRUICSS handle is not NULL
 *              - Validates PRU slice value (0 or 1)
 *              - Validates IEP comparator event (0-15)
 *              - Validates clock frequencies (must be positive)
 *              - Validates IEP instance (0 or 1)
 *              - Validates clock source selection (0 or 1)
 *              - Validates oversampling rate (0-7)
 *              - Sets up PRU DRAM base address for firmware interface
 *              - Initializes register offsets based on PRU slice
 *              - Marks handle as open
 *
 *              Internal API calls (in order):
 *              - tamagawa_config_clr_cfg0() - Clears PRU Tamagawa CFG0 registers
 *              - \ref tamagawa_config_channel() - Configures channel mask for both single and multi-channel modes
 *              - \ref tamagawa_config_host_trigger() - Sets default trigger mode to host trigger
 *              - \ref tamagawa_set_baudrate() - Configures communication baud rate from attrs configuration
 *
 *  \param[in]  index            Index of Tamagawa handle to use in the \ref gTamagawaHandle array
 *  \param[in]  params           Pointer to structure containing Tamagawa parameters. Use \ref tamagawa_params_init
 *                               to initialize with defaults before setting custom values. Must not be NULL.
 *
 *  \retval     handle           Pointer to initialized tamagawa_handle instance
 *  \retval     NULL             On validation failure (invalid index, NULL params, invalid configuration, or internal API call failure)
 *
 *  \note       Validation strategy: This function performs strict validation on all input parameters.
 *              After successful initialization, internal structures (attrs, priv, pruicss_handle) are
 *              assumed to be valid and are not rechecked in subsequent API calls for performance reasons.
 *  \note       Channel configuration, trigger mode, and baud rate are automatically set during initialization based on
 *              SysConfig parameters. Applications do not need to call configuration functions separately unless
 *              they need to change the mode after initialization.
 */
tamagawa_handle tamagawa_init(uint32_t index, const tamagawa_params *params);

/**
 *  \brief      Deinitialize Tamagawa interface and release resources
 *
 *  \details    This function deinitializes a Tamagawa instance by marking it as closed.
 *              It does not free memory as the handle is statically allocated via SysConfig.
 *              After calling this function, the handle should not be used until reinitialized
 *              with \ref tamagawa_init.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \note       NULL check: If handle is NULL, function returns without performing any operation.
 */
void tamagawa_deinit(tamagawa_handle handle);

/**
 *  \brief      Process a Tamagawa command (build, send, and wait for completion)
 *
 *  \details    This function combines command setup, triggering, and waiting for
 *              completion in a single call. It internally calls:
 *              - \ref tamagawa_command_build : Setup command in PRU interface buffer
 *              - \ref tamagawa_command_send : Trigger PRU to send command
 *              - \ref tamagawa_command_wait : Wait for PRU to complete transaction
 *
 *              For EEPROM commands (DATA_ID_6, DATA_ID_D), the function also resets the
 *              command ID for all channels after completion. The multi-channel mask for
 *              EEPROM operations is determined automatically from the enabled channels
 *              configured in the attrs structure.
 *
 *  \param[in]  handle           Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  cmd              Tamagawa command number (see \ref tamagawa_data_id)
 *
 *  \retval     SystemP_SUCCESS  Command processed successfully
 *  \retval     SystemP_FAILURE  Command processing failed (invalid handle, invalid command, or timeout)
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_command_process(tamagawa_handle handle, int32_t cmd);

/**
 *  \brief      Build a Tamagawa command in the PRU interface
 *
 *  \details    This function sets up the Tamagawa command parameters in the PRU interface
 *              based on the command type. The command is not sent until \ref tamagawa_command_send
 *              is called. For EEPROM commands (DATA_ID_6, DATA_ID_D), this function prepares
 *              the TX data with CF, ADF, EDF, and CRC fields. The multi-channel mask for
 *              EEPROM operations is determined automatically from the enabled channels
 *              configured in the attrs structure.
 *
 *  \param[in]  handle           Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  cmd              Tamagawa command number (see \ref tamagawa_data_id)
 *
 *  \retval     SystemP_SUCCESS  Command built successfully
 *  \retval     SystemP_FAILURE  Command build failed (invalid handle, unsupported command)
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_command_build(tamagawa_handle handle, int32_t cmd);

/**
 *  \brief      Trigger sending the Tamagawa command in PRU
 *
 *  \details    This function triggers the PRU firmware to send the command that was previously
 *              set up using \ref tamagawa_command_build. It sets the trigger bit in the PRU
 *              interface, which signals the firmware to start the transaction.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     SystemP_SUCCESS    Command trigger successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_command_send(tamagawa_handle handle);

/**
 *  \brief      Wait until PRU finishes Tamagawa transaction
 *
 *  \details    This function waits in a polling loop until the PRU firmware clears the trigger
 *              bit, indicating that the command has been completed. This is a blocking call with
 *              timeout protection.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     SystemP_SUCCESS    Command completed successfully
 *  \retval     SystemP_FAILURE    NULL handle provided or timeout occurred (configured via
 *                                 tamagawa_params.max_wait_loop_count before calling \ref tamagawa_init,
 *                                 default: 5000 us = 50 loops × 100 us/loop)
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_command_wait(tamagawa_handle handle);

/**
 *  \brief      Configure global RX auto arm counter for Tamagawa interface
 *
 *  \details    This function configures the global RX auto arm counter register for all three
 *              channels. The counter value determines the timing for RX enable.
 *
 *  \param[in]  handle      Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  rx_en_cnt   Value to be set in global RX auto arm counter register
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_config_global_rx_arm_cnt(tamagawa_handle handle, uint16_t  rx_en_cnt);

/**
 *  \brief      Configure Tamagawa clock dividers and source selection
 *
 *  \details    This function configures the PRU Three Channel Peripheral Interface's RX and TX clock
 *              dividers and clock source selection. It writes to PRUx_ED_RXCFG and PRUx_ED_TXCFG registers.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  clk_cfg   Pointer to structure containing clock configuration data
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle or NULL clk_cfg provided
 *
 *  \note       NULL check: Strict check on handle and clk_cfg. After successful init,
 *              internal structures are assumed valid and not rechecked.
 */
int32_t tamagawa_config_clock(tamagawa_handle handle, tamagawa_clk_cfg *clk_cfg);

/**
 *  \brief      Configure Tamagawa interface for host trigger mode
 *
 *  \details    This function sets the operation mode to host trigger mode (opmode = 1).
 *              In this mode, commands are sent when explicitly triggered by the host using
 *              \ref tamagawa_command_send.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_config_host_trigger(tamagawa_handle handle);

/**
 *  \brief      Configure Tamagawa interface for periodic trigger mode
 *
 *  \details    This function sets the operation mode to periodic trigger mode (opmode = 0).
 *              In this mode, commands are sent periodically by the PRU firmware based on
 *              IEP timer configuration.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_config_periodic_trigger(tamagawa_handle handle);

/**
 *  \brief      Configure channel mask for Tamagawa interface
 *
 *  \details    This function configures the channel mask for both single-channel and multi-channel modes.
 *              In single-channel mode, it also stores the specific channel index in priv->channel
 *              based on the channel enable flags (channel0_enabled, channel1_enabled, channel2_enabled).
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  mask      Channel mask (valid range 1-7 for up to 3 channels)
 *                        - Bit 0: Channel 0
 *                        - Bit 1: Channel 1
 *                        - Bit 2: Channel 2
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle or invalid mask (mask == 0 or mask > 0x07)
 *
 *  \note       NULL check: Strict check on handle. Mask bounds checked (1-7).
 */
int32_t tamagawa_config_channel(tamagawa_handle handle, uint8_t mask);

/**
 *  \brief      Set current channel for receive processing in multi-channel mode
 *
 *  \details    In multi-channel configuration, after the receive transaction is complete,
 *              call this function to select each channel before invoking RX parsing APIs
 *              to process the data received on that channel.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  ch        Channel number to be selected (0-2, see \ref TAMAGAWA_MAX_CHANNELS)
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle or invalid channel (ch >= TAMAGAWA_MAX_CHANNELS)
 *
 *  \note       NULL check: Strict check on handle. Channel bounds checked (0-2).
 */
int32_t tamagawa_multi_channel_set_cur(tamagawa_handle handle, uint8_t ch);

/**
 *  \brief      Update the current requested command ID in Tamagawa interface
 *
 *  \details    This function updates the data_id field in the Tamagawa interface structure.
 *              For EEPROM commands (DATA_ID_6, DATA_ID_D), it also sets the command ID for
 *              all three channels in the EEPROM interface.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  cmd       Tamagawa command number (see \ref tamagawa_data_id)
 *
 *  \retval     SystemP_SUCCESS    Update successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_update_data_id(tamagawa_handle handle, int32_t cmd);

/**
 *  \brief      Update the ADF (EEPROM address) field for EEPROM command
 *
 *  \details    This function updates the ADF (Address Field) in the Tamagawa EEPROM interface
 *              for the specified channel. This is used for both EEPROM Read (DATA_ID_D) and
 *              EEPROM Write (DATA_ID_6) commands.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  val       ADF value to be updated (valid range 0-127, see \ref TAMAGAWA_MAX_EEPROM_ADDRESS)
 *  \param[in]  ch        Channel number that is currently selected (0-2, see \ref TAMAGAWA_MAX_CHANNELS)
 *
 *  \retval     SystemP_SUCCESS    Update successful
 *  \retval     SystemP_FAILURE    NULL handle, invalid channel (ch >= TAMAGAWA_MAX_CHANNELS),
 *                                 or invalid ADF value (val > TAMAGAWA_MAX_EEPROM_ADDRESS)
 *
 *  \note       NULL check: Strict check on handle. Channel and ADF value bounds checked.
 */
int32_t tamagawa_update_adf(tamagawa_handle handle, uint32_t val, uint8_t ch);

/**
 *  \brief      Update the EDF (EEPROM data) field for EEPROM Write command
 *
 *  \details    This function updates the EDF (Encoder Data Field) in the Tamagawa EEPROM
 *              interface for the specified channel. This is used only for EEPROM Write
 *              (DATA_ID_6) commands.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  val       EDF value to be updated (valid range 0-255, see \ref TAMAGAWA_MAX_EEPROM_WRITE_DATA)
 *  \param[in]  ch        Channel number that is currently selected (0-2, see \ref TAMAGAWA_MAX_CHANNELS)
 *
 *  \retval     SystemP_SUCCESS    Update successful
 *  \retval     SystemP_FAILURE    NULL handle, invalid channel (ch >= TAMAGAWA_MAX_CHANNELS),
 *                                 or invalid EDF value (val > TAMAGAWA_MAX_EEPROM_WRITE_DATA)
 *
 *  \note       NULL check: Strict check on handle. Channel and EDF value bounds checked.
 */
int32_t tamagawa_update_edf(tamagawa_handle handle, uint32_t val, uint8_t ch);

/**
 *  \brief      Parse the data in Tamagawa interface received from the encoder
 *
 *  \details    This function extracts and parses the received frame data based on the command
 *              type. It populates the rx_frames_received structure in the Tamagawa interface
 *              with the parsed data (CF, SF, ABS, ABM, ENID, ALMC, ADF, EDF, CRC).
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  cmd       Tamagawa command number (see \ref tamagawa_data_id)
 *
 *  \retval     SystemP_SUCCESS    Data parsed successfully
 *  \retval     SystemP_FAILURE    Parsing failed (invalid handle or unsupported command)
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_parse(tamagawa_handle handle, int32_t cmd);

/**
 *  \brief      Verify the CRC computed by PRU firmware against encoder CRC
 *
 *  \details    This function checks the CRC verification result computed by the PRU firmware.
 *              The firmware calculates CRC and stores the result in cal_crc field. A value
 *              of 1 indicates successful CRC verification.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     SystemP_SUCCESS    CRC verification passed
 *  \retval     SystemP_FAILURE    CRC verification failed or NULL handle
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 */
int32_t tamagawa_crc_verify(tamagawa_handle handle);

/**
 *  \brief      Update CRC field for EEPROM command
 *
 *  \details    This function calculates and updates the CRC field for EEPROM commands
 *              based on CF, ADF, and EDF values. It internally calls the CRC calculation
 *              function. For EEPROM Write (DATA_ID_6), CRC is calculated over 3 bytes
 *              (CF + ADF + EDF). For EEPROM Read (DATA_ID_D), CRC is calculated over
 *              2 bytes (CF + ADF).
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  cmd       Tamagawa command number (DATA_ID_6 or DATA_ID_D)
 *  \param[in]  ch        Channel number that is currently selected (0-2, see \ref TAMAGAWA_MAX_CHANNELS)
 *
 *  \retval     SystemP_SUCCESS    CRC update successful
 *  \retval     SystemP_FAILURE    NULL handle or invalid channel (ch >= TAMAGAWA_MAX_CHANNELS)
 *
 *  \note       NULL check: Strict check on handle. Channel bounds checked (0-2).
 */
int32_t tamagawa_update_crc(tamagawa_handle handle, int32_t cmd, uint8_t ch);

/**
 *  \brief      Set Tamagawa communication baud rate
 *
 *  \details    This function calculates and configures the RX/TX division factors and
 *              oversampling rate based on the specified baud rate. It internally calls:
 *              - \ref tamagawa_config_clock : Configure clock dividers
 *              - \ref tamagawa_config_global_rx_arm_cnt : Configure RX auto arm counter
 *
 *              The function also updates the firmware interface with the calculated
 *              division factors and oversampling rate.
 *
 *  \param[in]  handle      Tamagawa handle returned by \ref tamagawa_init
 *  \param[in]  baud_rate   Baud rate of the Tamagawa encoder in Hz (2.5 MHz or 5 MHz)
 *
 *  \retval     SystemP_SUCCESS    Configuration successful
 *  \retval     SystemP_FAILURE    NULL handle provided
 *
 *  \note       NULL check: Strict check on handle. After successful init, internal structures
 *              are assumed valid and not rechecked.
 *  \note       The RX auto arm counter is configured for 1us delay. This may need adjustment
 *              based on specific encoder requirements.
 */
int32_t tamagawa_set_baudrate(tamagawa_handle handle, double baud_rate);

/**
 *  \brief      Get pointer to Tamagawa attributes structure
 *
 *  \details    This function returns a const pointer to the attributes structure containing
 *              compile-time configuration parameters from SysConfig. The attrs structure is
 *              read-only and should not be modified.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     attrs     Pointer to const tamagawa_attrs structure
 *  \retval     NULL      If handle is NULL
 *
 *  \note       NULL check: Strict check on handle. Returns NULL on NULL handle.
 */
const tamagawa_attrs* tamagawa_get_attrs(tamagawa_handle handle);

/**
 *  \brief      Get pointer to Tamagawa private structure
 *
 *  \details    This function returns a pointer to the private structure containing runtime
 *              state information. This is typically used for advanced operations or debugging.
 *
 *  \param[in]  handle    Tamagawa handle returned by \ref tamagawa_init
 *
 *  \retval     priv      Pointer to tamagawa_priv structure
 *  \retval     NULL      If handle is NULL
 *
 *  \note       NULL check: Strict check on handle. Returns NULL on NULL handle.
 *  \warning    Modifying priv structure fields directly can lead to undefined behavior.
 *              Use provided APIs for configuration changes.
 */
tamagawa_priv* tamagawa_get_priv(tamagawa_handle handle);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* TAMAGAWA_DRV_H_ */
