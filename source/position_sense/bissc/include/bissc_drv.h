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


#ifndef BISSC_DRV_H_
#define BISSC_DRV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <position_sense/bissc/include/bissc_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/** \brief Delay in microseconds for command processing (1000 us = 1 ms)
 *
 *  Default value for delay used in \ref bissc_command_wait polling loop to avoid excessive CPU usage.
 *  Different value can be configured by changing cmd_process_delay_us of \ref bissc_params before calling \ref bissc_init.
 */
#define BISSC_DEFAULT_CMD_PROCESS_DELAY_US      (1000U)

/** \brief Delay in microseconds between firmware initialization status checks (1000 us = 1 ms)
 *
 *  Default value for delay used in \ref bissc_wait_for_fw_initialization, \ref bissc_wait_measure_proc_delay and \ref bissc_set_ctrl_cmd_and_process.
 *  Different value can be configured by changing fw_wait_delay_us of \ref bissc_params before calling \ref bissc_init.
 */
#define BISSC_DEFAULT_FW_WAIT_DELAY_US          (1000U)

/** \brief Maximum BiSS-C cycle timeout in milliseconds
 *
 *  BiSS-C cycle time formula: TCycle_min = TMA * (5 + DLEN + CRCLEN) + tLineDelay + tbusy_max + busy_s_max + tTO
 *  A timeout of 5ms is used as it accommodates various encoders and daisy chain configurations.
 *  This timeout is the default value used in \ref bissc_command_wait to detect communication failures.
 *  Different value can be configured by changing max_cycle_timeout_ms of \ref bissc_params before calling \ref bissc_init.
 */
#define BISSC_DEFAULT_MAX_CYCLE_TIMEOUT         (5U)

/** \brief Single PRU - Single channel configuration mode
 *
 *  Only one channel (ch0, ch1, or ch2) is used with a single PRU core.
 *  This is the simplest configuration for single encoder applications.
 */
#define BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU    (0U)

/** \brief Single PRU - Multichannel configuration mode
 *
 *  Multiple channels (up to 3: ch0, ch1, ch2) are managed by a single PRU core.
 *  All channels share the same PRU core resources without load sharing.
 */
#define BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU     (1U)

/** \brief Multichannel - Load Share configuration mode
 *
 *  Multiple channels are distributed across multiple PRU cores for load
 *  sharing. Each channel is handled by a dedicated PRU core.
 *  Mapping: RTU-PRU - Channel 0, PRU - Channel 1, TX-PRU - Channel 2.
 *
 */
#define BISSC_MODE_MULTI_CHANNEL_MULTI_PRU      (2U)

/** \brief Maximum encoder processing delay at 1 MHz in valid bits/clock cycles (40 cycles) */
#define BISSC_MAX_PROC_DELAY_1MHZ           40

/** \brief Maximum encoder processing delay at 2 MHz in valid bits/clock cycles (80 cycles) */
#define BISSC_MAX_PROC_DELAY_2MHZ           80

/** \brief Maximum encoder processing delay at 5 MHz in valid bits/clock cycles (200 cycles) */
#define BISSC_MAX_PROC_DELAY_5MHZ           200

/** \brief Maximum encoder processing delay at 8 MHz in valid bits/clock cycles (320 cycles) */
#define BISSC_MAX_PROC_DELAY_8MHZ           320

/** \brief Maximum encoder processing delay at 10 MHz in valid bits/clock cycles (400 cycles) */
#define BISSC_MAX_PROC_DELAY_10MHZ          400

/** \brief CTS (Control Transfer Status) bit value for control communication */
#define BISSC_CTS_BIT                       1

/** \brief Number of Encoder ID bits in control communication (3 bits) */
#define BISSC_ENC_ID_LEN                    3

/** \brief Bit mask for extracting Encoder ID (0x7 = 0b111) */
#define BISSC_ENC_ID_MASK                   0x7

/** \brief Number of bits for Register address in control communication (7 bits) */
#define BISSC_REG_ADDR_LEN                  7

/** \brief Bit mask for extracting Register address (0x7F = 0b1111111) */
#define BISSC_REG_ADDR_MASK                 0x7F

/** \brief Number of RWS bits (3 bits) */
#define BISSC_RWS_LEN                       3

/** \brief Bit mask for extracting RWS bits (0x7 = 0b111) */
#define BISSC_RWS_MASK                      0x7

/** \brief RWS value for read access of encoder register (0x5 = 0b101) */
#define BISSC_CTRL_READ_ACCESS              0x5

/** \brief RWS value for write access of encoder register (0x3 = 0b011) */
#define BISSC_CTRL_WRITE_ACCESS             0x3

/** \brief Number of Register data bits (8 bits = 1 byte) */
#define BISSC_REG_DATA_LEN                  8

/** \brief Bit mask for extracting Register data (0xFF = 0b11111111) */
#define BISSC_REG_DATA_MASK                 0xFF

/** \brief Number of stop bits for control communication (2 bits: P and S)
 *
 *  P: stop bit indicating end of one control communication frame
 *  S: stop bit indicating end of sequential control communication sequence
 */
#define BISSC_CTRL_STOP_LEN                 2

/** \brief Rx divisor value for 8x oversampling (divisor = 8, register value = 7)
 *
 *  8x oversampling samples each bit 8 times for robust clock recovery and data sampling.
 *  The register is programmed with (divisor - 1), so 8x oversampling uses value 7.
 */
#define BISSC_RX_SAMPLE_SIZE                7

/** \brief Rx divisor value for 6x oversampling (divisor = 6, register value = 5) */
#define BISSC_RX_SAMPLE_SIZE_6X             5

/** \brief Rx divisor value for 4x oversampling (divisor = 4, register value = 3) */
#define BISSC_RX_SAMPLE_SIZE_4X             3

/** \brief Number of position data CRC bits (6-bit CRC polynomial)
 *
 *  6-bit CRC protects position data integrity during transmission.
 */
#define BISSC_POS_CRC_LEN                   6

/** \brief Number of Error and Warning bits (2 bits total) */
#define BISSC_EW_LEN                        2

/** \brief Number of control command CRC bits (4-bit CRC polynomial)
 *
 *  4-bit CRC protects control communication command and response data.
 */
#define BISSC_CTRL_CMD_CRC_LEN              4

/** \brief Bit mask for extracting control command CRC (0xF = 0b1111) */
#define BISSC_CTRL_CMD_CRC_MASK             0xF

/** \brief Default position data length in bits (12 bits)
 *
 *  Used as a safe default value before actual encoder resolution is configured.
 *  Prevents garbage data interpretation during initialization.
 */
#define BISSC_POS_DATA_LEN_DEFAULT          12

/** \brief Number of Safety CRC bits for BiSS Safety protocol (16-bit CRC)
 *
 *  BiSS Safety uses a 16-bit CRC along with sign-of-life counter.
 */
#define BISSC_SAFETY_CRC_LEN                16

/** \brief Number of Sign-of-Life counter bits in BiSS Safety protocol (6 bits)
 *
 *  The 6-bit sign-of-life counter increments with each transaction to detect
 *  communication interruptions in BiSS Safety mode.
 */
#define BISSC_SIGN_OF_LIFE_LEN              6

/** \brief Maximum total frame size in bits (64 bits)
 *
 *  BiSS-C protocol uses 64-bit data frames. Total frame includes:
 *  - Without Safety: Position Data + E/W(2) + CRC(6) <= 64 bits
 *  - With Safety: Position Data + E/W(2) + sign-of-life(6) + safety CRC(16) <= 64 bits
 */
#define BISSC_MAX_FRAME_SIZE                64

/** \brief Enable fractional clock divider (1.5x) for Rx clock (bit 15 = 1)
 *
 *  Fractional divider allows achieving 1.5x clock division for frequencies
 *  that cannot be achieved with integer division.
 */
#define BISSC_RX_ENABLE_FRACTIONAL_DIV      (1<<15)

/** \brief Allowed BiSS-C communication frequency: 1 MHz */
#define BISSC_FREQ_1MHZ                     1

/** \brief Allowed BiSS-C communication frequency: 2 MHz */
#define BISSC_FREQ_2MHZ                     2

/** \brief Allowed BiSS-C communication frequency: 5 MHz */
#define BISSC_FREQ_5MHZ                     5

/** \brief Allowed BiSS-C communication frequency: 8 MHz */
#define BISSC_FREQ_8MHZ                     8

/** \brief Allowed BiSS-C communication frequency: 10 MHz */
#define BISSC_FREQ_10MHZ                    10

/** \brief Middle bit index for 8x oversampling (index 4 out of 0-7)
 *
 *  For 8x oversampling, bit 4 is the center sample point used for received data.
 */
#define BISSC_FIFO_BIT_IDX_8X_OS            4

/** \brief Middle bit index for 6x oversampling (index 3 out of 0-5) */
#define BISSC_FIFO_BIT_IDX_6X_OS            3

/** \brief Middle bit index for 4x oversampling (index 2 out of 0-3) */
#define BISSC_FIFO_BIT_IDX_4X_OS            2

/** \brief General purpose macro for clearing any status flag (value = 0) */
#define BISSC_CLEAR_STATUS_FLAG             0x0

/** \brief General purpose macro for setting any status flag (value = 1) */
#define BISSC_SET_STATUS_FLAG               0x1

/** \brief PRU-ICSS Core Clock frequency: 200 MHz (in MHz units)
 *
 *  Used on devices like AM243x with 200 MHz PRU-ICSS core clock.
 */
#define BISSC_PRU_CORE_CLK_FREQ_200MHZ            200

/** \brief PRU-ICSS Core Clock frequency: 300 MHz (in MHz units)
 *
 *  Used on devices like AM263x with 300 MHz PRU-ICSS core clock.
 */
#define BISSC_PRU_CORE_CLK_FREQ_300MHZ            300

/** \brief PRU-ICSS UART Clock frequency: 160 MHz (in MHz units) */
#define BISSC_PRU_UART_CLK_FREQ_160MHZ            160

/** \brief PRU-ICSS UART Clock frequency: 192 MHz (in MHz units) */
#define BISSC_PRU_UART_CLK_FREQ_192MHZ            192

/** \brief Valid bit index (value = 24)
 */
#define BISSC_VALID_BIT_IDX                       (24U)

/** \brief Clock fractional divider factor (value = 1.5)
 *
 *  Fractional divider applied in BiSS-C clock calculations for
 *  achieving precise baud rates with oversampling.
 */
#define BISSC_CLOCK_FRACTIONAL_DIVIDER            (1.5)

/** \brief MHz to Hz conversion factor (value = 1000000)
 *
 *  Multiplication factor for converting frequency values from MHz to Hz.
 *  Used in clock configuration calculations.
 */
#define BISSC_MHZ_TO_HZ                          (1000000U)

/** \brief Error/Warning field bit mask (value = 0x03)
 *
 *  2-bit mask for extracting Error/Warning (E/W) field from BiSS-C position data.
 */
#define BISSC_EW_FIELD_MASK                      (0x03U)

/** \brief 6-bit field mask (value = 0x3F) */
#define BISSC_6BIT_FIELD_MASK                    (0x3FU)

/** \brief Safety CRC field mask (value = 0xFFFF)
 *
 *  16-bit mask for extracting safety CRC from BiSS-C Safety protocol data.
 *  BiSS Safety adds 16-bit CRC for enhanced data integrity.
 */
#define BISSC_SAFETY_CRC_FIELD_MASK              (0xFFFFU)

/** \brief BiSS-C Periodic/Continuous operation mode
 *
 *  In periodic mode, the PRU firmware automatically triggers BiSS-C transactions
 *  at regular intervals using IEP timer events, without host CPU intervention.
 */
#define BISSC_OPMODE_PERIODIC               (0x0U)

/** \brief BiSS-C Host trigger operation mode
 *
 *  In host trigger mode, each BiSS-C transaction must be explicitly triggered
 *  by the host CPU via \ref bissc_command_send or \ref bissc_command_process.
 */
#define BISSC_OPMODE_HOST_TRIGGER           (0x1U)

/** \brief Disable encoder processing delay measurement (value = 0x0) */
#define BISSC_MEASURE_PROC_DELAY_DISABLE    (0x0U)

/** \brief Enable encoder processing delay measurement (value = 0x1) */
#define BISSC_MEASURE_PROC_DELAY_ENABLE     (0x1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining BiSS-C clock configuration for selected frequency
 *
 *    \details  Contains clock divisors and configuration calculated by \ref bissc_calc_clock
 *              for achieving the desired baud rate. These values are written to PRU-ICSS
 *              registers during hardware initialization.
 *
 */
typedef struct bissc_clk_cfg_s
{
    uint16_t  rx_div;
    /**< Rx clock divisor (value-1 written to register). Determines receive sample rate.
     *   Formula: rx_clk = source_clk / ((rx_div + 1) * rx_oversampling)
     *   Example: For 8x oversampling at 1 MHz from 200 MHz core: rx_div = 24 */

    uint16_t  tx_div;
    /**< Tx clock divisor (value-1 written to register). Determines transmit baud rate.
     *   Formula: tx_clk = source_clk / (tx_div + 1)
     *   Example: For 1 MHz from 200 MHz core: tx_div = 199 */

    uint16_t  rx_div_attr;
    /**< Rx oversampling rate and fractional divider configuration.
     *   Bits [14:0]: Oversampling divisor (7=8x, 5=6x, 3=4x)
     *   Bit [15]: Fractional divider enable (1=enable 1.5x fractional division) */

    uint16_t  is_core_clk;
    /**< Clock source selection for BiSS-C communication.
     *   0 = Use UART clock (160/192 MHz)
     *   1 = Use Core clock (200/300 MHz) */
} bissc_clk_cfg;

/**
 *    \brief    Structure defining BiSS-C Position data results
 *
 *    \details  Contains position data received from encoder(s), including raw position value,
 *              calculated angle, number of turns (multi-turn), error/warning flags, and CRC values
 *              for data integrity verification. This structure is populated by
 *              \ref bissc_get_pos API after a successful encoder transaction.
 */
typedef struct bissc_position_info_s
{
    uint64_t          position[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Raw position data (combined single-turn and multi-turn bits) from each encoder */
    float             angle[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Single turn angle for each encoder */
    uint32_t          num_of_turns[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Multi turn count for each encoder */
    uint8_t           ew[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Error and Warning bits (2 bits) for each encoder */
    uint8_t           rcv_crc[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Received 6-bit CRC from encoder for position data */
    uint8_t           otf_crc[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Calculated on-the-fly 6-bit CRC for position data verification */
} bissc_position_info;

/**
 *    \brief    Structure defining BiSS-C Channel specific control communication(ctrl) results
 *
 *    \details  Contains results from BiSS-C control communication.
 *              Includes the data received from the  encoder, along with CRC
 *              values for error checking.
 *              Populated by \ref bissc_set_ctrl_cmd_and_process API.
 *
 */
typedef struct bissc_control_info_s
{
    uint8_t           cmd_result;
    /**< Control communication data received from encoder (8 bits) */
    uint8_t           cmd_rcv_crc;
    /**< Received 4-bit CRC from encoder for control communication */
    uint8_t           cmd_otf_crc;
    /**< Calculated on-the-fly 4-bit CRC for control communication verification */
} bissc_control_info;

/**
 *    \brief    Structure defining BiSS-C initialization parameters
 *
 *    \details  Parameters passed to \ref bissc_init to initialize a BiSS-C instance.
 *              Use \ref bissc_params_init to populate with default values.
 *
 */
typedef struct bissc_params_s
{
    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS Handle obtained from PRUICSS_open(). */

    uint32_t cmd_process_delay_us;
    /**< Delay in microseconds for command processing polling loop.
     *   Used in \ref bissc_command_wait to avoid excessive CPU usage.
     *   Default: 1000 us (1 ms) */

    uint32_t fw_wait_delay_us;
    /**< Delay in microseconds between firmware status checks.
     *   Used in \ref bissc_wait_for_fw_initialization, \ref bissc_wait_measure_proc_delay
     *   and \ref bissc_set_ctrl_cmd_and_process.
     *   Default: 1000 us (1 ms) */

    uint32_t max_cycle_timeout_ms;
    /**< Maximum BiSS-C cycle timeout in milliseconds.
     *   Used in \ref bissc_command_wait to detect communication failures.
     *   Default: 5 ms */
} bissc_params;

/**
 *    \brief    BiSS-C private data structure (runtime state and configuration)
 *
 *    \details  Contains runtime state information including encoder parameters, position/control
 *              data results, CRC error counts, safety data, and pointers to PRU-ICSS shared
 *              memory (bissc_pruicss_xchg). This structure is initialized during \ref bissc_init
 *              and should be accessed via \ref bissc_get_priv API.
 *
 */
typedef struct bissc_priv_s
{
    uint8_t is_open;
    /**< Initialization state flag.
     *   0 = Driver closed/not initialized
     *   1 = Driver successfully initialized and open */

    uint32_t data_len[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Total resolution (single-turn + multi-turn) in bits for each encoder.
     *   Indexed by [channel][encoder_in_daisy_chain] */

    uint32_t single_turn_len[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Single-turn resolution in bits for each encoder.
     *   Indexed by [channel][encoder_in_daisy_chain] */

    uint32_t multi_turn_len[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Multi-turn resolution in bits for each encoder.
     *   Indexed by [channel][encoder_in_daisy_chain] */

    uint32_t channel[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Array mapping enabled channel indices to physical channel numbers (0, 1, or 2).
     *   Example: If only ch1 is enabled, channel[0] = 1
     *   Example: If ch0 and ch2 are enabled, channel[0] = 0  and channel[1] = 2*/

    bissc_pruicss_xchg *pruicss_xchg;
    /**< Pointer to PRU-ICSS shared memory interface structure.
     *   This is the primary communication structure between ARM R5F and PRU firmware */

    uint8_t has_safety[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< BiSS Safety protocol enable flag for each encoder.
     *   0 = Standard BiSS-C only
     *   1 = BiSS Safety enabled (includes 16-bit CRC and sign-of-life counter) */

    uint8_t sign_of_life_cnt[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< 6-bit sign-of-life counter from BiSS Safety protocol.
     *   Increments with each transaction to detect communication interruptions */

    uint16_t rcv_safety_crc[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Received 16-bit safety CRC from encoder (BiSS Safety protocol) */

    uint16_t calc_safety_crc[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Calculated 16-bit safety CRC by PRU firmware for comparison.
     *   Mismatch indicates data corruption */

    uint8_t is_continuous_mode;
    /**< Operation mode flag.
     *   0 = Host trigger mode (manual transaction triggering)
     *   1 = Periodic/continuous mode (IEP timer-driven automatic triggering) */

    uint64_t raw_data;
    /**< Data storage for raw 64-bit data received from encoder.
     *   Used during position data parsing in \ref bissc_get_pos */

    bissc_position_info enc_pos_data[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Parsed position data results (angle, turns, CRC) for each channel.
     *   Updated by \ref bissc_get_pos after each successful transaction */

    bissc_control_info enc_ctrl_data[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Control communication results (register data, CRC) for each channel.
     *   Updated by \ref bissc_set_ctrl_cmd_and_process after register read/write */

    uint32_t pd_crc_err_cnt[BISSC_NUM_CH_PER_SLICE_MAX][BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Cumulative position data CRC error count for each encoder.
     *   Incremented by firmware when received CRC != calculated CRC */

    uint32_t ctrl_crc_err_cnt[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Cumulative control communication CRC error count for each channel.
     *   Incremented when control response CRC validation fails */

    uint32_t num_encoders[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Number of encoders connected in daisy chain on each channel.
     *   Configured via bissc_update_data_len() */

    uint8_t ctrl_write_status[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Control communication read/write status for each channel. */

    uint32_t ctrl_reg_address[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Target register address for control communication (0x00-0x7F).
     *   Set by bissc_generate_ctrl_cmd() */

    uint32_t ctrl_reg_data[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Register data for control communication */

    uint32_t ctrl_enc_id[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Encoder ID for control communication in daisy chain */

    uint16_t  proc_delay[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Measured encoder processing delay in clock cycles for each channel.
     *   Automatically measured by PRU firmware during initialization */

    uint32_t baud_rate;
    /**< BiSS-C communication baud rate in MHz (1, 2, 5, 8, or 10).
     *   Configured via bissc_update_clock_freq() or from SysConfig */

    uint32_t cmd_process_delay_us;
    /**< Delay in microseconds for command processing polling loop.
     *   Used in \ref bissc_command_wait to avoid excessive CPU usage */

    uint32_t fw_wait_delay_us;
    /**< Delay in microseconds between firmware status checks.
     *   Used in firmware initialization and control communication functions */

    uint32_t max_cycle_timeout_ms;
    /**< Maximum BiSS-C cycle timeout in milliseconds.
     *   Used in \ref bissc_command_wait to detect communication failures */

    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS driver handle obtained from PRUICSS_open().
     *   Used for accessing PRU-ICSS hardware resources */
} bissc_priv;

/**
 * \brief   Structure defining BiSS-C attributes (compile-time/SysConfig configuration data)
 *
 */
typedef struct bissc_attrs_s
{
    uint8_t instance;
    /**< BiSS-C instance index (0, 1, ...) for multi-instance configurations.
     *   Used to distinguish between multiple BiSS-C instances in the system */

    uint8_t mode;
    /**< BiSS-C channel configuration mode.
     *   0 = BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU (one channel, one PRU)
     *   1 = BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU (multiple channels, one PRU)
     *   2 = BISSC_MODE_MULTI_CHANNEL_MULTI_PRU (multiple channels, load-shared across PRUs) */

    uint8_t pruicss_instance;
    /**< PRU-ICSS hardware instance number (0 or 1).
     *   0 = PRU-ICSSG0/PRU-ICSSM0
     *   1 = PRU-ICSSG1/PRU-ICSSM1 */

    uint8_t pruicss_slice;
    /**< PRU-ICSS slice selection (0 or 1).
     *   Each PRU-ICSS has 2 slices, each with its own set of PRU cores.
     *   0 = Slice 0 (contains PRU0/RTU-PRU0/TX-PRU0 on PRU-ICSSG, PRU0 on PRU-ICSSM)
     *   1 = Slice 1 (contains PRU1/RTU-PRU1/TX-PRU1 on PRU-ICSSG, PRU1 on PRU-ICSSM) */

    uint8_t load_share_enabled;
    /**< Load share mode enable flag.
     *   0 = Disabled (single PRU handles all channels)
     *   1 = Enabled (channels distributed across RTU-PRU, PRU, and TX-PRU in PRU-ICSSG only) */

    uint8_t pru_id;
    /**< PRU core ID for channel assignment.
     *   0 = PRUICSS_PRU0 (handles any channel in single PRU mode, and handles channel 1 in load share mode)
     *   1 = PRUICSS_PRU1 */

    uint8_t rtu_pru_id;
    /**< RTU-PRU core ID for channel assignment in load share mode.
     *   2 = PRUICSS_RTU_PRU0 (handles channel 0)
     *   3 = PRUICSS_RTU_PRU1 */

    uint8_t tx_pru_id;
    /**< TX-PRU core ID for channel assignment in load share mode.
     *   4 = PRUICSS_TX_PRU0 (handles channel 2)
     *   5 = PRUICSS_TX_PRU1 */

    uint8_t channel_mask;
    /**< Bit mask indicating which channels are enabled (0-7).
     *   Bit 0 (0x1): Channel 0 enabled
     *   Bit 1 (0x2): Channel 1 enabled
     *   Bit 2 (0x4): Channel 2 enabled
     *   Example: 0x5 = channels 0 and 2 enabled */

    uint8_t channel0_enabled;
    /**< Channel 0 enable flag (1=enabled, 0=disabled) */

    uint8_t channel1_enabled;
    /**< Channel 1 enable flag (1=enabled, 0=disabled) */

    uint8_t channel2_enabled;
    /**< Channel 2 enable flag (1=enabled, 0=disabled) */

    uint8_t total_channels;
    /**< Total number of enabled channels (1, 2, or 3).
     *   Calculated as: channel0_enabled + channel1_enabled + channel2_enabled */

    uint16_t baud_rate;
    /**< BiSS-C communication baud rate in MHz (configured from SysConfig).
     *   Valid values: 1, 2, 5, 8, or 10 MHz */

    uint32_t core_clk_freq;
    /**< PRU-ICSS Core Clock frequency in Hz (not MHz). */

    uint32_t uart_clk_freq;
    /**< PRU-ICSS UART Clock frequency in Hz (not MHz). */

    uint32_t iep_clk_freq;
    /**< PRU-ICSS IEP (Industrial Ethernet Peripheral) timer clock frequency in Hz. */

    uint16_t is_core_clk;
    /**< Clock source selection for BiSS-C communication.
     *   0 = Use UART clock as source (uart_clk_freq)
     *   1 = Use Core clock as source (core_clk_freq) */
} bissc_attrs;

/**
 * \brief   Structure defining BiSS-C configuration handle
 *
 * \details This structure combines pointers to both runtime state (priv) and compile-time
 *          configuration (attrs). The handle is returned by bissc_init() and passed to all
 *          BiSS-C driver APIs to identify the specific BiSS-C instance being operated on.
 *
 */
typedef struct bissc_config_s
{
    bissc_priv *priv;
    /**< Pointer to BiSS-C private data (runtime state and results).
     *   Contains encoder parameters, position data, control communication results,
     *   and all runtime operational state maintained by the driver */

    const bissc_attrs *attrs;
    /**< Pointer to BiSS-C attributes (read-only configuration from SysConfig).
     *   Contains compile-time configuration including PRU instance, channels,
     *   clock frequencies, and operation mode settings */
} bissc_config;

/**
 * \brief   BiSS-C handle type
 *
 * \details Opaque handle to a BiSS-C instance. Obtained from bissc_init() and used
 *          in all subsequent BiSS-C API calls.
 */
typedef bissc_config *bissc_handle;

#ifdef __cplusplus
}
#endif

#endif
