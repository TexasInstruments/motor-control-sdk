/*
 *  Copyright (C) 2021-2026 Texas Instruments Incorporated
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


#ifndef ENDAT_DRV_H_
#define ENDAT_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <position_sense/endat/include/endat_interface.h>

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/**    \brief    EnDAT operation mode: Single channel with single PRU core */
#define ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU                (0U)

/**    \brief    EnDAT operation mode: Multi-channel with single PRU core */
#define ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU                 (1U)

/**    \brief    EnDAT operation mode: Multi-channel with multiple PRU cores (load share mode) */
#define ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU                  (2U)

/**    \brief    MRS code for encoder manufacturer parameters - Page 0 (12 words) */
#define ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0     (0xA1U)

/**    \brief    MRS code for encoder manufacturer parameters - Page 1 (16 words) */
#define ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1     (0xA3U)

/**    \brief    MRS code for encoder manufacturer parameters - Page 2 (16 words) */
#define ENDAT_MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2     (0xA5U)

/**    \brief    Load share mode enable mask for PRU firmware configuration */
#define ENDAT_LOAD_SHARE_EN_MASK                            (0x00000800U)

/**    \brief    RX oversampling rate for EnDAT receiver clock */
#define ENDAT_RX_OVERSAMPLING_RATE                          (8U)

/**    \brief    EnDAT enable RX fractional divider */
#define ENDAT_RX_FRAC_DIV                                   (1 << 15)

/**    \brief    TX bits for 30-bit transmission (adjusted +1 for early transmission) */
#define ENDAT_TX_30BITS                                     (31U)

/**    \brief    TX bits for 6-bit transmission (adjusted +1 for early transmission) */
#define ENDAT_TX_6BITS                                      (7U)

/**    \brief    RX bits for 29-bit reception */
#define ENDAT_RX_29BITS                                     (29U)

/**    \brief    RX bits for 46-bit reception (EnDAT 2.1 test values) */
#define ENDAT_RX_46BITS                                     (46U)

/**    \brief    EnDAT word index constants for parameter access */
#define APP_ENDAT_WORD_0                                    (0U)
#define APP_ENDAT_WORD_1                                    (1U)
#define APP_ENDAT_WORD_2                                    (2U)
#define APP_ENDAT_WORD_3                                    (3U)
#define APP_ENDAT_WORD_4                                    (4U)
#define APP_ENDAT_WORD_5                                    (5U)
#define APP_ENDAT_WORD_6                                    (6U)
#define APP_ENDAT_WORD_7                                    (7U)
#define APP_ENDAT_WORD_8                                    (8U)
#define APP_ENDAT_WORD_9                                    (9U)
#define APP_ENDAT_WORD_10                                   (10U)
#define APP_ENDAT_WORD_11                                   (11U)
#define APP_ENDAT_WORD_12                                   (12U)
#define APP_ENDAT_WORD_13                                   (13U)
#define APP_ENDAT_WORD_14                                   (14U)
#define APP_ENDAT_WORD_15                                   (15U)
#define APP_ENDAT_WORD_16                                   (16U)
#define APP_ENDAT_WORD_17                                   (17U)
#define APP_ENDAT_WORD_18                                   (18U)
#define APP_ENDAT_WORD_19                                   (19U)

/**    \brief    Number of CRC bits for position data */
#define ENDAT_NUM_BITS_POSITION_CRC                         (5U)

/**    \brief    Number of bits for F1 error flag */
#define ENDAT_NUM_BITS_F1                                   (1U)

/**    \brief    Number of bits for F2 warning flag */
#define ENDAT_NUM_BITS_F2                                   (1U)

/**    \brief    Number of bits for parameter data */
#define ENDAT_NUM_BITS_PARAMETER                            (16U)

/**    \brief    Number of bits for address field */
#define ENDAT_NUM_BITS_ADDRESS                              (8U)

/**    \brief    EnDAT initialization frequency in Hz (200 kHz) */
#define ENDAT_INIT_FREQ                                     (200000U)

/**    \brief    EnDAT clock cycle period at initialization frequency (ns)
 *             Clock period = 1 / 200kHz = 5000ns */
#define ENDAT_INIT_FREQ_CLOCK_PERIOD_NS                     (5000U)

/**    \brief    Number of propagation delay samples for averaging */
#define ENDAT_PROP_DELAY_NUM_SAMPLES                        (8U)

/**    \brief    Nanoseconds per second conversion factor */
#define ENDAT_NS_PER_SECOND                                 (1000000000U)

/**    \brief    Minimum short recovery time in nanoseconds (2.45 us) */
#define ENDAT_SHORT_RECOVERY_TIME_MIN                       (2450U)

/**    \brief    Maximum short recovery time in nanoseconds (3.75 us) */
#define ENDAT_SHORT_RECOVERY_TIME_MAX                       (3750U)

/**    \brief    Minimum long recovery time in nanoseconds (18.5 us) */
#define ENDAT_LONG_RECOVERY_TIME_MIN                        (18500U)

/**    \brief    Maximum long recovery time in nanoseconds (30.0 us) */
#define ENDAT_LONG_RECOVERY_TIME_MAX                        (30000U)

/**    \brief    Maximum valid recovery time counter value (2^32 - 1) */
#define ENDAT_MAX_RT_COUNTER_VALUE                          (0xFFFFFFFFU)

/**    \brief    Recovery time out of range error code */
#define ENDAT_RT_OUT_OF_RANGE_ERROR                         (0x1U)

/**    \brief    Recovery time counter stuck error code */
#define ENDAT_RT_COUNTER_STUCK_ERROR                        (0x2U)

/**    \brief    No recovery time error */
#define ENDAT_RT_NO_ERROR                                   (0x0U)

/**    \brief    Recovery time counter starting value */
#define ENDAT_RT_COUNTER_STARTING_VALUE                     (100U)

/**    \brief    Minimum difference between starting values of recovery time counters */
#define ENDAT_RT_COUNTERS_STARTING_DIFFERENCE         (100U)

/**    \brief    Command processing delay (12 ms) - copied from firmware timing requirements */
#define ENDAT_CMD_PROCESS_DELAY_12MS_US                     (12000U)

/**    \brief    Parameter read delay (2 ms) - copied from firmware timing requirements */
#define ENDAT_PARAM_READ_DELAY_2MS_US                       (2000U)

/**
 *  \brief  EnDAT operation mode: Periodic trigger mode using IEP compare event
 *
 *  In periodic mode, the PRU firmware automatically triggers position readout
 *  at regular intervals configured by IEP timer compare events.
 */
#define ENDAT_OPMODE_CMP_PERIODIC                           (0x0U)

/**
 *  \brief  EnDAT operation mode: Host trigger
 *
 *  In host trigger mode, the R5F host processor explicitly triggers
 *  each position readout by setting the trigger bit in firmware interface.
 */
#define ENDAT_OPMODE_HOST_TRIGGER                           (0x1U)

/**
 *  \brief  EnDAT operation mode: Periodic trigger mode using IEP capture event
 *
 *  The PRU firmware automatically triggers position readout at regular
 *  intervals based on IEP timer capture events.
 */
#define ENDAT_OPMODE_CAP_PERIODIC                           (0x2U)

/**    \brief    Delay counter increment value  */
#define ENDAT_DELAY_COUNTER_INCREMENT                       (5U)

/**    \brief    Maximum IEP compare event number (0-15) */
#define ENDAT_IEP_CMP_EVENT_MAX                             (16U)

/**    \brief    Maximum IEP capture event number (0-7) */
#define ENDAT_IEP_CAP_EVENT_MAX                             (8U)

/**    \brief    EnDat channel mask */
#define ENDAT_CHANNEL_MASK                                  (7U)

/** \brief Delay in microseconds for command processing (1000 us = 1 ms)
 *
 *  Default value for delay used in command processing polling loop to avoid excessive CPU usage.
 *  Different value can be configured by changing cmd_process_delay_us of \ref endat_params before calling \ref endat_init.
 */
#define ENDAT_DEFAULT_CMD_PROCESS_DELAY_US      (1000U)

/** \brief Delay in microseconds between firmware initialization status checks (1000 us = 1 ms)
 *
 *  Default value for delay used in firmware initialization and command processing functions.
 *  Different value can be configured by changing fw_wait_delay_us of \ref endat_params before calling \ref endat_init.
 */
#define ENDAT_DEFAULT_FW_WAIT_DELAY_US          (1000U)

/** \brief Maximum wait loop count for EnDAT cycle timeout
 *
 *  Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
 *  Default value is 1000, which with default cmd_process_delay_us of 1000us results in 1000ms timeout.
 *  This accommodates various encoders and operating modes.
 *
 *  Different value can be configured by changing max_wait_loop_count of \ref endat_params before calling \ref endat_init.
 *  Value must be greater than 0.
 */
#define ENDAT_DEFAULT_MAX_WAIT_LOOP_COUNT       (1000U)

/**    \brief    IEP configuration register size in bytes
 *
 *  \note FIXME: Remove these definitions once they are available in cslr_common.h
 */
#define ENDAT_CFG_REG_SIZE                                        (4U)

/**
 * \brief 8-byte register offset for IEP registers
 */
#define ENDAT_8_BYTE_REG_OFFSET                                   (8U)

/**    \brief    IEP capture register 0 address */
#define ENDAT_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0                     (CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG + 2U*ENDAT_CFG_REG_SIZE)

/** \brief Validate EnDAT 2.1 command codes (1-7) */
#define VALID_2_1_CMD(x) (((x) == 1) || ((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 5) || ((x) == 6) || ((x) == 7) )
/** \brief Validate EnDAT 2.2 command codes (8-14) */
#define VALID_2_2_CMD(x) (((x) == 8) || ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 12) || ((x) == 13) || ((x) == 14))

/** \brief Number of dummy bits in additional info */
#define ENDAT_NUM_ADDITIONAL_INFO_DUMMY_BIT                       (1U)
/** \brief Number of bytes in additional info field */
#define ENDAT_NUM_BYTES_ADDITIONAL_INFO                           (3U)

/** \brief Encoder status warning bit shift position */
#define ENDAT_STATUS_WARN_SHIFT                                   (7U)
/** \brief Encoder status RM (Request Master) bit shift position */
#define ENDAT_STATUS_RM_SHIFT                                     (6U)
/** \brief Encoder status busy bit shift position */
#define ENDAT_STATUS_BUSY_SHIFT                                   (5U)
/** \brief Encoder status warning bit mask */
#define ENDAT_STATUS_WARN_MASK                                    (1U << ENDAT_STATUS_WARN_SHIFT)
/** \brief Encoder status RM bit mask */
#define ENDAT_STATUS_RM_MASK                                      (1U << ENDAT_STATUS_RM_SHIFT)
/** \brief Encoder status busy bit mask */
#define ENDAT_STATUS_BUSY_MASK                                    (1U << ENDAT_STATUS_BUSY_SHIFT)
/** \brief Encoder information field mask (bits 0-4) */
#define ENDAT_INFORMATION_MASK                                    (0x1FU)

/** \brief EnDAT 2.1 F1 error flag bit shift position */
#define ENDAT_21_F1_SHIFT                                         (31U)
/** \brief EnDAT 2.1 F1 error flag bit mask */
#define ENDAT_21_F1_MASK                                          (1U << ENDAT_21_F1_SHIFT)
/** \brief EnDAT 2.2 F1 error flag bit shift position */
#define ENDAT_22_F1_SHIFT                                         ENDAT_21_F1_SHIFT
/** \brief EnDAT 2.2 F1 error flag bit mask */
#define ENDAT_22_F1_MASK                                          (1U << ENDAT_22_F1_SHIFT)
/** \brief F2 warning flag bit shift position */
#define ENDAT_F2_SHIFT                                            (30U)
/** \brief F2 warning flag bit mask */
#define ENDAT_F2_MASK                                             (1U << ENDAT_F2_SHIFT)

/** \brief Number of RX bits for additional info field */
#define ENDAT_ADDITIONAL_INFO_RX_BITS                             (30U)

/** \brief Section 2 memory address constant */
#define ENDAT_SECTION2_MEMORY                                     (0xBFU)

/** \brief Number of valid bits in page 0 word 13 */
#define ENDAT_NUM_BITS_VALID_PAGE0_WORD13                         (6U)

/** \brief Number of valid bits in page 1 word 1 */
#define ENDAT_NUM_BITS_VALID_PAGE1_WORD1                          (16U)

/** \brief MRS (Mode Register Select) C7-C4 bits shift value */
#define ENDAT_MRS_SHIFT_C7_C4                                     (4U)
/** \brief MRS C7-C4 value to select additional info 1 */
#define ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1               (0x4U)
/** \brief MRS C7-C4 value to select additional info 2 */
#define ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2               (0x5U)
/** \brief MRS mask for selecting additional info */
#define ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO                     (0xFU << ENDAT_MRS_SHIFT_C7_C4)
/** \brief MRS value to stop additional info transmission */
#define ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO                        (0xFU)
/** \brief MRS mask for stopping additional info */
#define ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO                       (ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)

/** \brief Extract multi-turn position from combined position value */
#define ENDAT_GET_POS_MULTI_TURN(pos, handle)                     (((pos) & (((uint64_t) 1 << (handle)->pos_res) - 1)) >> (handle)->single_turn_res[(handle)->current_channel])
/** \brief Extract single-turn position from combined position value */
#define ENDAT_GET_POS_SINGLE_TURN(pos, handle)                    ((pos) & (((uint64_t) 1 << (handle)->single_turn_res[(handle)->current_channel]) - 1))

/* ========================================================================== */
/*                         Enum Definitions                                   */
/* ========================================================================== */

/**    \brief    Encoder types */
enum endat_encoder_type_e
{
    ENDAT_ENCODER_TYPE_LINEAR,
    ENDAT_ENCODER_TYPE_ROTARY
} endat_encoder_type;
/* ========================================================================== */
/*                         Structure Definitions                              */
/* ========================================================================== */

/**
 *  \brief EnDAT clock configuration
 *
 *  \details This structure contains calculated clock divider values and configured the endat clock. These values are
 *           computed based on the endat clock and 3 channel source clock frequency.
 */
typedef struct endat_clk_cfg_s
{
    uint16_t rx_div;
    /**< RX clock divider value.
     *   Divides the PRU clock to generate the receiver sampling clock */

    uint16_t tx_div;
    /**< TX clock divider value.
     *   Divides the PRU clock to generate the transmitter baud rate */

    uint16_t rx_en_cnt;
    /**< RX enable counter for switch to RX mode.
     *   Timing after TX to start RX sampling */

    uint16_t rx_div_attr;
    /**< RX over sampling rate.
     *  Number of samples taken per bit period
     *  Bit  [2:0] : Oversampling divisor (7 = 8x, 5 = 6x, 3 = 4x)
     *  Bits [3]   : Start bit polarity (0 or 1)
     *  Bit  [15]  : Fractional divider enable (1=enable 1.5x fractional division) */
} endat_clk_cfg;

/**
 *  \brief EnDAT encoder status flags
 *
 *  \details This structure contains bitfield flags indicating the availability
 *           of additional information from the encoder. These flags are set by
 *           the encoder in its response and indicate whether Info1 and Info2
 *           data words are available.
 *
 */
typedef struct endat_flags_s
{
    uint32_t info1 : 1;
    /**< Info1 availability flag.
     *   0 = Additional info1 not available
     *   1 = Additional info1 available  */

    uint32_t info2 : 1;
    /**< Info2 availability flag.
     *   0 = Additional info2 not available
     *   1 = Additional info2 available */
} endat_flags;

/**
 *  \brief EnDAT encoder identification data
 *
 */
typedef struct endat_id_s
{
    uint32_t ascii;
    uint32_t binary;
} endat_id;

/**
 *  \brief EnDAT encoder serial number
 *
 */
typedef struct endat_sn_s
{
    uint32_t ascii_msb;
    /**< Serial number ASCII most significant bytes. */

    uint32_t binary;

    uint32_t ascii_lsb;
    /**< Serial number ASCII least significant bytes. */
} endat_sn;

/**
 *  \brief EnDAT command supplement parameters
 *
 *  \details This structure contains additional parameters required for specific
 *           EnDAT commands. Not all fields are used for every command - the
 *           relevant fields depend on the command type being executed.
 *
 *           Used for commands that require memory addressing, data transmission,
 *           or periodic trigger configuration.
 */
typedef struct endat_cmd_supplement_s
{
    uint32_t cmd_type;
    /**< Command type. */
    uint32_t address[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Memory address parameter for MRS (Memory Read Select) commands. */

    uint32_t data[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Data value for encoder write commands. */

    uint32_t block[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Block address for command*/

    uint8_t has_block_address[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Flag indicating if cmd has block address. */

    uint32_t frequency;
    /**< Frequency parameter for clock configuration commands. */

    uint32_t delay;
    /**< used to take delay from user delay parameter*/

    uint8_t enable_rt;
    /**<Enable/disable Recovery time counter */

    uint8_t selected_channel;
    /**< Selected channel for multi-channel operations.
     *   0 = channel 0
     *   1 = channel 1
     *   2 = channel 2 */

    uint64_t iep_reset_count;
    /**< IEP timer reset count value for periodic trigger mode.
     *   Determines the period of automatic position updates */

    uint32_t periodic_mode_cmd;
    /**< Command for periodic mode configuration.*/

    uint64_t ch_trigger_count[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Per-channel trigger count for periodic mode.
     *   Array indexed by channel (0-2)
     *   Specifies trigger intervals for each encoder channel  */

    uint64_t iep_sync0_period;
    /**< IEP SYNC OUT0 period  */
} endat_cmd_supplement;

/**
 *  \brief EnDAT raw received data
 *
 *  \details Raw data words received from encoder before parsing.
 *           Content depends on the command type that was executed.
 */
typedef struct endat_data_s
{
    uint64_t recvd1;
    uint32_t recvd2;
    uint32_t recvd3;
} endat_data;

/**
 *  \brief EnDAT position data
 *
 *  \details Parsed position information from encoder response
 */
typedef struct endat_position_s
{
    uint64_t position;
    uint64_t revolution;
    uint8_t f1;
    /**< F1 error flag (0 = no error, 1 = error) */
    uint8_t f2;
    /**< F2 warning flag (0 = no warning, 1 = warning) */
    uint8_t crc;
    /**< Received CRC value for position data */
} endat_position;

/**
 *  \brief EnDAT additional information data
 *
 *  \details Additional information word (addinfo1 or addinfo2) from encoder
 */
typedef struct endat_addinfo_s
{
    uint32_t addinfo;
    /**< Additional information */
    uint8_t crc;
    /**< Received CRC value for additional information */
} endat_addinfo;

/**
 *  \brief EnDAT position with additional information
 *
 */
typedef struct endat_position_addinfo_s
{
    endat_position position;
    endat_addinfo addinfo1;
    endat_addinfo addinfo2;
} endat_position_addinfo;

/**
 *  \brief EnDAT memory read response data
 *
 *  \details Response from MRS (Memory Read Select) command
 */
typedef struct endat_addr_params_s
{
    uint8_t address;
    /**< Memory address */
    uint16_t params;
    /**< 16-bit parameter value read from encoder memory */
    uint8_t crc;
    /**< Received CRC value for the parameter data */
} endat_addr_params;

/**
 *  \brief EnDAT test response data
 *
 */
typedef struct endat_test_values_s
{
    uint64_t value;
    uint8_t f1;
    uint8_t crc;
} endat_test_values;

/**
 *  \brief EnDAT formatted response data
 *
 */
typedef union endat_format_data_u
{
    endat_position_addinfo position_addinfo;
    /**< Position command response  */

    endat_addr_params addr_params;
    /**< Memory read command response */

    endat_test_values test;
    /**< Test response */
} endat_format_data;

/**
 *  \brief EnDAT driver attributes (compile-time configuration)
 *
 *  \details This structure contains compile-time configuration parameters populated
 *           from SysConfig during code generation. These values define the hardware
 *           configuration and operational mode of the EnDAT driver instance.
 *
 *           This structure is typically declared as const and placed in read-only
 *           memory, as these parameters do not change during driver operation.
 */
typedef struct endat_attrs_s
{
    uint8_t instance;
    /**< EnDAT driver instance index.
     *   Identifies this specific EnDAT instance in multi-instance configurations
     *   Valid range: 0 to (CONFIG_ENDAT_NUM_INSTANCES - 1)
     *   Used to index into the global gEndatHandle array */

    uint8_t mode;
    /**< EnDAT operational mode defining channel and PRU configuration.
     *   Determines how channels are mapped to PRU cores:
     *   - 0 = ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU: Single channel on one PRU
     *   - 1 = ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU: Multiple channels on one PRU
     *   - 2 = ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU: Multiple channels across multiple PRUs
     *   Configured via SysConfig based on system requirements */

    uint8_t pruicss_instance;
    /**< PRU-ICSS hardware instance number.
     *   Specifies which PRU-ICSS subsystem to use
     *   Valid values: 0 or 1
     *   Depends on SoC capabilities and hardware design */

    uint8_t pruicss_slice;
    /**< PRU slice (core) selection within the PRU-ICSS instance.
     *   Valid values:
     *   - 0 = PRU0
     *   - 1 = PRU1
     *   Determines which PRU core executes the EnDAT firmware */

    uint8_t load_share_enabled;
    /**< Load share mode enable flag.
     *   Enables distribution of encoder channels across multiple PRU cores
     *   Valid values:
     *   - 0 = Load sharing disabled (all channels on single PRU)
     *   - 1 = Load sharing enabled (channels distributed across PRUs)
     *   Only applicable in MULTI_CHANNEL_MULTI_PRU mode */

    uint8_t channel_mask;
    /**< Enabled channels bitmask.
     *   Bit-field indicating which encoder channels are active:
     *   - Bit 0: Channel 0 enabled
     *   - Bit 1: Channel 1 enabled
     *   - Bit 2: Channel 2 enabled
     *   Valid range: 0x1 - 0x7 (at least one channel must be enabled)
     *   Used by firmware to determine which channels to process */

    uint8_t channel0_enabled;
    /**< Channel 0 enable flag.
     *   Valid values:
     *   - 0 = Channel 0 disabled
     *   - 1 = Channel 0 enabled
     *   Must match bit 0 of channel_mask */

    uint8_t channel1_enabled;
    /**< Channel 1 enable flag.
     *   Valid values:
     *   - 0 = Channel 1 disabled
     *   - 1 = Channel 1 enabled
     *   Must match bit 1 of channel_mask */

    uint8_t channel2_enabled;
    /**< Channel 2 enable flag.
     *   Valid values:
     *   - 0 = Channel 2 disabled
     *   - 1 = Channel 2 enabled
     *   Must match bit 2 of channel_mask */

    uint8_t total_channels;
    /**< Total number of enabled encoder channels.
     *   Sum of channel0_enabled + channel1_enabled + channel2_enabled
     *   Valid range: 1 - 3
     *   Used for iteration and validation in multi-channel operations */

    uint32_t core_clk_freq;
    /**< PRU core clock frequency in Hz.
     *   Typically 200 MHz or 300 MHz depending on SoC
     *   Used for timing calculations and clock divider configuration
     *   Obtained from PRU module configuration in SysConfig */

    uint32_t uart_clk_freq;
    /**< PRU UART peripheral clock frequency in Hz. */

    uint32_t iep_clk_freq;
    /**< IEP (Industrial Ethernet Peripheral) timer clock frequency in Hz. */

    uint16_t is_core_clk;
    /**< Clock source selection for TX/RX clock.
     *   Valid values:
     *   - 0 = Use UART clock for communication
     *   - 1 = Use core clock for communication
     *   Affects baud rate calculation and timing precision */

    uint8_t iep_instance;
    /**< IEP timer instance selection.
     *   Valid values:
     *   - 0 = IEP0
     *   - 1 = IEP1
     *   Only used in periodic trigger mode
     *   Depends on SoC capabilities and resource availability */

    uint8_t iep_cmp_event[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< IEP compare event numbers for periodic trigger mode (per channel).
     *   Array indexed by channel (0-2)
     *   Each entry specifies which IEP compare event triggers position updates
     *   Valid range per entry: 0 - 15
     *   Used to generate periodic encoder read requests */

    uint8_t iep_cap_event[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< IEP capture event numbers for periodic trigger mode (per channel).
     *   Array indexed by channel (0-2)
     *   Each entry specifies which IEP capture event timestamps encoder responses
     *   Valid range per entry: 0 - 7
     *   Used for accurate timing measurement in periodic mode */

    void *iep_base_addr;
    /**< IEP timer base address for register access.
     *   Memory-mapped base address of IEP peripheral
     *   Used by driver to configure compare/capture events in periodic trigger mode
     *   Obtained from SoC memory map */
} endat_attrs;

/**
 *  \brief EnDAT driver initialization parameters
 *
 *  \details This structure contains runtime parameters required for EnDAT driver initialization.
 *           The structure should be initialized using \ref endat_params_init before passing
 *           to \ref endat_init.
 *
 *           Application must allocate memory for channel RX info and provide both local and
 *           global addresses:
 *           - channel_rx_info: Local address pointer for driver access
 *           - ch_info_global_addr: SoC global address for firmware access
 *           Both must point to the same physical memory location.
 *
 *           These parameters control timing, delays, and PRU-ICSS handle association.
 *           Default values are provided via \ref endat_params_init.
 */
typedef struct endat_params_s
{
    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS Handle obtained from PRUICSS_open().
     *   Must be valid and non-NULL. */

    uint32_t max_wait_loop_count;
    /**< Maximum wait loop count for EnDAT cycle timeout detection.
     *   Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
     *   Used in \ref endat_command_wait for timeout detection.
     *   Must be greater than 0.
     *   Default: 1000 */

    uint32_t cmd_process_delay_us;
    /**< Delay in microseconds for command processing polling loop.
     *   Used in \ref endat_command_wait to avoid excessive CPU usage during polling.
     *   Default: 1000 us (1 ms) */

    uint32_t fw_wait_delay_us;
    /**< Delay in microseconds between firmware status checks.
     *   Used in \ref endat_wait_initialization function.
     *   Default: 1000 us (1 ms) */

    endat_ch_rx_info_array *channel_rx_info;
    /**< Pointer to local channel RX info memory allocated by application.
     *
     *   This memory buffer is where PRU firmware writes command responses and
     *   received encoder data. The application reads position values, additional
     *   information, and command results from this structure.
     *
     *   Must point to valid memory of type endat_ch_rx_info_array.
     *   Must not be NULL.
     *
     *   Memory layout: Contains arrays indexed by channel for position data,
     *   CRC values, additional information fields, and command responses.
     *   See endat_ch_rx_info_array structure for detailed field descriptions. */

    uint32_t ch_info_global_addr;
    /**< SoC global address for channel RX info memory (firmware write buffer).
     *
     *   This is the physical global address that PRU firmware uses to write
     *   command responses and encoder data. Must be the SoC global address
     *   corresponding to the channel_rx_info local pointer.
     *
     *   Both channel_rx_info and ch_info_global_addr must point to the same
     *   physical memory, but use different address spaces:
     *   - channel_rx_info: CPU local address for application read access
     *   - ch_info_global_addr: SoC global address for PRU firmware write access
     *
     *   Must not be 0.
     *
     *   Example: If channel_rx_info points to memory at local address 0x70000000,
     *   ch_info_global_addr might be 0x50000000 (depends on SoC memory mapping). */
} endat_params;

/**
 *    \brief    EnDAT driver private data (runtime state)
 *
 *    \details  This structure contains per-instance runtime state including encoder configuration,
 *              data results, CRC error tracking, and pointers to PRU-ICSS shared memory (endat_pruicss_xchg).
 *              This structure is initialized during \ref endat_init and updated during driver operation.
 *
 */
typedef struct endat_priv_s
{
    uint8_t is_open;
    /**< Initialization state flag.
     *   0 = Driver closed/not initialized
     *   1 = Driver successfully initialized and open */

    int32_t current_channel;
    /**< Currently selected channel index for multi-channel configurations.
     *   Valid range: 0 to (total_channels - 1) */

    int32_t pos_res;
    /**< Total position resolution in bits (single-turn + multi-turn).*/

    int32_t multi_turn_res[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Multi-turn resolution in bits for each channel.
     *   Indexed by [channel]. Obtained from encoder during initialization via MRS commands */

    int32_t single_turn_res[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Single-turn resolution in bits for each channel.
     *   Indexed by [channel]. Obtained from encoder during initialization via MRS commands */

    int32_t step[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Step value for position calculation for each channel.
     *   Indexed by [channel]. Used for angle conversion calculations */

    uint32_t pos_rx_bits_21_cmd[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Number of position RX bits for EnDAT 2.1 command for each channel.
     *   Indexed by [channel]. Determined during encoder initialization */

    uint32_t pos_rx_bits_22_cmd[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Number of position RX bits for EnDAT 2.2 command for each channel.
     *   Indexed by [channel]. Determined during encoder initialization */

    int32_t type[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Encoder type for each channel (linear or rotary).
     *   Indexed by [channel]. Values: 0 = ENDAT_ENCODER_TYPE_LINEAR, 1 = ENDAT_ENCODER_TYPE_ROTARY
     *   Obtained from encoder parameters during initialization */

    int32_t has_safety[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< EnDAT Safety protocol enable flag for each channel.
     *   0 = Standard EnDAT only
     *   1 = EnDAT Safety enabled
     *   Indexed by [channel] */

    endat_id id;
    /**< Encoder identification data  */

    endat_sn sn;
    /**< Encoder serial number data */

    uint32_t cmd_set_2_2;
     /**< EnDAT 2.2 command set support flag.
      *   0 = Encoder supports EnDAT 2.1 only
      *   1 = Encoder supports EnDAT 2.2 command set
      *   Determined during encoder identification */

    endat_flags flags[ENDAT_NUM_CH_PER_SLICE_MAX];
     /**< Status flags structure containing info1 and info2 flags for each channel.
      *   Indexed by [channel]. Used to indicate availability of additional information from encoder.
      *   In load-share mode, each channel has independent info1/info2 state.
      *   In single-channel mode, only flags[current_channel] is used */

    int32_t raw_data;
    /**< Temporary storage for raw data during processing. */

    uint16_t rx_en_cnt;
    /**< RX enable counter for clock configuration.
     *   Determines sampling window timing for received data */

    endat_pruicss_xchg *pruicss_xchg;
    /**< Pointer to PRU-ICSS shared memory interface structure.
     *   This is the primary communication structure between ARM R5F and PRU firmware.
     *   Allocated in PRU DRAM and mapped during \ref endat_init */

    endat_ch_rx_info_array *channel_rx_info;
    /**< Pointer to channel RX info array holding received encoder data for all channels.
     *   Allocated internally during \ref endat_init from the global gEndatChRxInfoArray.
     *   Updated by PRU firmware after each transaction */

    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS driver handle obtained from PRUICSS_open().
     *   Used for PRU firmware loading and management.
     *   Copied from params in \ref endat_init */

    uint32_t cmd_process_delay_us;
    /**< Delay in microseconds for command processing polling loop.
     *   Used in command wait functions to avoid excessive CPU usage.
     *   Copied from params in \ref endat_init */

    uint32_t fw_wait_delay_us;
    /**< Delay in microseconds between firmware status checks.
     *   Used during firmware initialization and command processing.
     *   Copied from params in \ref endat_init */

    uint32_t max_wait_loop_count;
    /**< Maximum wait loop count for EnDAT cycle timeout detection.
     *   Used in command wait functions for timeout calculation.
     *   Copied from params in \ref endat_init */

    uint32_t endat_freq;
    /**< Configured EnDAT communication clock frequency in Hz.
     */
} endat_priv;

/**
 *  \brief EnDAT driver configuration (handle structure)
 *
 *  \details This structure combines runtime state (priv) and compile-time configuration
 *           (attrs) to form the complete driver handle. This is the primary data structure
 *           that represents an initialized EnDAT driver instance.
 *
 *           The structure separates concerns between:
 *           - Runtime state (priv): Dynamic data that changes during operation
 *           - Compile-time configuration (attrs): Static parameters from SysConfig
 *
 *           Application code typically uses endat_handle (pointer to this structure)
 *           and does not directly access these fields.
 */
typedef struct endat_config_s
{
    endat_priv *priv;
    /**< Pointer to runtime private data structure.
     *   Contains encoder configuration, received data, error tracking,
     *   and pointers to PRU shared memory interface
     *   Allocated and initialized by \ref endat_init
     *   Updated during driver operation as data is exchanged with encoders */

    const endat_attrs *attrs;
    /**< Pointer to compile-time attributes structure.
     *   Contains hardware configuration (PRU instance, channels, clocks, IEP)
     *   Generated by SysConfig and placed in read-only memory
     *   Remains constant throughout driver lifetime */
} endat_config;

/**
 *  \brief EnDAT driver handle type
 *
 *  \details Opaque handle to an EnDAT driver instance. This is a pointer to the
 *           endat_config structure but is treated as opaque by application code.
 *
 *           A handle is obtained from \ref endat_init during driver initialization
 *           and must be passed to all subsequent EnDAT driver API calls.
 *           The handle remains valid until \ref endat_deinit is called.
 *
 *           Multiple handles can exist simultaneously for multi-instance configurations.
 */
typedef endat_config *endat_handle;

#include "endat_api.h"

#ifdef __cplusplus
}
#endif

#endif
