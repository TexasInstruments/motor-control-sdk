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

#ifndef BISSC_INTERFACE_H_
#define BISSC_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "bissc_drv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum number of BiSS-C channels per PRU-ICSS slice (3 channels: ch0, ch1, ch2)
 *
 *  Each slice supports up to 3 independent BiSS-C channels for connecting multiple encoders.
 */
#define BISSC_NUM_CH_PER_SLICE_MAX              3

/** \brief Maximum number of BiSS-C encoders in daisy chain configuration per channel (3 encoders)
 *
 *  BiSS-C supports daisy-chaining multiple encoders on a single channel.
 *  Each encoder in the chain has a unique ID (0-2) and contributes its position data sequentially.
 */
#define BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX   3

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining BiSS-C Raw data received from encoder.
 *
 *    \details  BiSS-C position data can exceed 32 bits.
 *              The data is split into two 32-bit words for storage and processing.
 */
typedef struct bissc_raw_data_s
{
    volatile uint32_t   pos_data_word0;
    /**< Initial 32 bits of position data received from encoder. */

    volatile uint32_t   pos_data_word1;
    /**< 32 bits of position data received after initial 32 bits (valid only if total data length > 32 bits). */
} bissc_raw_data;

/**
 *    \brief    Structure defining BiSS-C position data lengths of connected encoders
 *
 *    \details  Specifies number of encoders in daisy chain and resolution (data length in bits)
 *              for each encoder. Used by firmware to correctly parse incoming position data.
 */
typedef struct bissc_enc_len_s
{
    volatile uint8_t num_encoders;
    /**< Number of encoders connected in daisy chain.
     *   Each encoder transmits its position data sequentially in the BiSS-C frame */

    volatile uint8_t data_len[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Position data length in bits for each encoder in the daisy chain.
     *   Includes single-turn + multi-turn resolution.
     *   Index corresponds to encoder position in chain (0=first, 1=second, 2=third) */
} bissc_enc_len;

/**
 *    \brief    Structure defining BiSS-C position data per channel
 *
 *    \details  Contains raw data, CRC error counts, and calculated CRC for each encoder
 *              in the daisy chain. Updated by PRU firmware after each BiSS-C transaction.
 */
typedef struct bissc_pos_data_res_s
{
    bissc_raw_data raw_data[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Raw position data received from each encoder in daisy chain.
     *   Stored as two 32-bit words before parsing into angle/turns */

    volatile uint32_t   pd_crc_err_cnt[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< Cumulative position data CRC error count for each encoder.
     *   Incremented by PRU firmware whenever received CRC doesn't match calculated CRC */

    volatile uint8_t    pos_data_otf_crc[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX];
    /**< On-the-fly calculated 6-bit CRC for position data of each encoder.
     *   Computed by PRU firmware during data reception for comparison with received CRC */
} bissc_pos_data_res;

/**
 *    \brief    Structure defining BiSS-C Channel specific control communication results
 *
 *    \details  Contains results from encoder register read/write operations including
 *              the register data (CDS response), CRC error count, and CRC validation values.
 *              Updated by PRU firmware after control communication completes.
 */
typedef struct bissc_ctrl_res_s
{
    volatile uint32_t ctrl_crc_err_cnt;
    /**< Cumulative control communication CRC error count for this channel.
     *   Incremented whenever control response CRC validation fails */

    volatile uint8_t ctrl_cds_res;
    /**< CDS response. */

    volatile uint8_t ctrl_rcvd_crc;
    /**< 4-bit CRC received from encoder with the control communication response. */

    volatile uint8_t ctrl_otf_crc;
    /**< On-the-fly calculated 4-bit CRC for control communication response.
     *   Computed by PRU firmware for comparison with received CRC */
} bissc_ctrl_res;

/**
 *    \brief    Structure defining BiSS-C periodic trigger configuration
 *
 *    \details  Contains IEP event configuration for periodic trigger mode
 */
typedef struct bissc_periodic_trigger_cfg_s
{
    uint8_t iep_cmp_event;
    /**< IEP compare event number for periodic CMP mode */

    uint8_t iep_cap_event;
    /**< IEP capture event number for periodic CAP mode */

    uint16_t reserved;
    /**< Reserved for alignment */

    uint32_t iep_capture_reg;
    /**< IEP capture register address for periodic CAP mode */
} bissc_periodic_trigger_cfg;

/**
 *    \brief    Structure defining BiSS-C PRU-ICSS shared memory interface
 *
 *    \details  This is the primary communication structure between ARM R5F CPU and PRU firmware.
 *              Located in PRU Data RAM, it contains configuration parameters, command/control
 *              fields, and results. Both ARM and PRU access this structure for data exchange.
 */
typedef struct bissc_pruicss_xchg_s
{
    volatile uint8_t pos_crc_len;
    /**< Position data CRC polynomial length in bits (typically 6 bits for BiSS-C).
     *   Configured by ARM, used by PRU firmware for CRC calculation */

    volatile uint8_t rx_clk_freq;
    /**< BiSS-C communication clock frequency in MHz (1, 2, 5, 8, or 10).
     *   Used by PRU firmware for timing calculations and delay measurements */

    volatile uint8_t ctrl_cmd_crc_len;
    /**< Control command CRC polynomial length in bits (4 bits for BiSS-C).
     *   Used by PRU firmware for control communication CRC calculation */

    volatile uint8_t  channel;
    /**< Channel enable bitmask (bit 0=ch0, bit 1=ch1, bit 2=ch2).
     *   Indicates which channels are active for this BiSS-C instance */

    volatile uint8_t status[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< PRU firmware initialization status for each channel.
     *   ARM polls this after loading firmware to confirm readiness */

    volatile uint8_t primary_core_mask;
    /**< Primary PRU core mask for load share mode synchronization (0x1, 0x2, or 0x4).
     *   Indicates which channel's PRU acts as primary coordinator */

    volatile uint8_t cycle_trigger[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< BiSS-C transaction trigger and completion status for each channel.
     *   ARM writes 1 to trigger, PRU clears to 0 when transaction completes
     *   ARM polls this in \ref bissc_command_wait to detect completion */

    volatile uint8_t measure_proc_delay;
    /**< Processing delay measurement control flag.
     *   1 = ARM requests PRU to measure encoder delay after configuration change
     *   0 = Measurement complete, PRU has updated proc_delay[] */

    volatile uint8_t opmode[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Operation mode for each channel.
     *   0 (BISSC_OPMODE_PERIODIC) = Automatic periodic triggering via IEP timer
     *   1 (BISSC_OPMODE_HOST_TRIGGER) = Manual trigger by ARM via cycle_trigger */

    bissc_enc_len  enc_len[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Encoder resolution configuration for each channel.
     *   Specifies number of encoders and data length (bits) for parsing position data */

    volatile uint8_t   valid_bit_idx;
    /**< Valid bit index */

    volatile uint8_t   fifo_bit_idx;
    /**< Middle bit index for oversampling (4=8x, 3=6x, 2=4x oversampling).
     *   PRU samples this bit position for most reliable data recovery */

    volatile uint8_t   ctrl_cmd_status[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Control communication transaction status for each channel.
     *   ARM writes 1 to initiate control communication, PRU clears to 0 when complete */

    volatile uint16_t  proc_delay[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Measured encoder processing delay in clock cycles for each channel.
     *   Automatically measured by PRU firmware, read by ARM via \ref bissc_get_enc_proc_delay */

    volatile uint32_t ctrl_cmd[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Control communication command for each channel (32-bit hex value).
     *   Generated by \ref bissc_generate_ctrl_cmd on ARM, executed by PRU firmware */

    volatile uint32_t max_proc_delay;
    /**< Maximum allowed processing delay in clock cycles for current frequency.
     *   Used by PRU firmware to validate measured delay is within spec */

    bissc_pos_data_res pos_data_res[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Position data results for each channel (raw data, CRC, error counts).
     *   Updated by PRU firmware after each transaction, read by ARM via \ref bissc_get_pos */

    bissc_ctrl_res ctrl_res[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Control communication results for each channel (register data, CRC).
     *   Updated by PRU firmware, read by ARM via \ref bissc_set_ctrl_cmd_and_process */

    volatile uint8_t   execution_state[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< PRU firmware execution state for load share mode synchronization.
     *   Used internally by firmware to coordinate multi-PRU operations */

    volatile uint64_t  register_backup[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< PRU register backup storage for context switching (ch0, ch1, ch2).
     *   Used by firmware for saving/restoring state during channel operations */

    volatile uint8_t   bissc_re_measure_proc_delay;
    /**< Flag to request re-measurement of processing delay.
     *   Set to 1 by ARM when frequency changes, cleared by PRU after measurement */

    volatile uint8_t   has_safety[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< BiSS Safety protocol enable bitmask for each channel.
     *   Each bit represents an encoder in daisy chain: bit 0=enc0, bit 1=enc1, bit 2=enc2
     *   1 = Safety enabled (includes 16-bit CRC and sign-of-life counter) */

    volatile uint16_t safety_crc[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX][BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Calculated 16-bit safety CRC for BiSS Safety protocol.
     *   Indexed by [encoder][channel], computed by PRU firmware */

    volatile uint32_t encoder_timeout;
    /**< Encoder timeout delay in PRU clock cycles (configurable via SysConfig).
     *   Calculated as: (core_clk_freq / 1000000) * encoder_timeout_us
     *   Default: 40 microseconds, configurable range: 1-100 microseconds
     *   Used by firmware for BiSS-C timeout detection */

    volatile uint32_t delay_100ms;
    /**< Maximum interframe delay in PRU clock cycles equivalent to 100 milliseconds.
     *   Calculated as: (core_clk_freq / 1000000) * 100 * 1000
     *   Used by firmware for maximum time between BiSS-C frames */

    volatile uint64_t icss_clk;
    /**< PRU-ICSS core clock frequency in Hz (not MHz).
     *   Example: 200000000 for 200 MHz, 300000000 for 300 MHz
     *   Used by firmware for precise timing and delay calculations */

    volatile uint32_t iep_base_address;
    /**< IEP base address used for periodic trigger mode */

    bissc_periodic_trigger_cfg trigger_params[BISSC_NUM_CH_PER_SLICE_MAX];
    /**< Periodic trigger configuration parameters for each channel (ch0, ch1, ch2).
     *   Contains IEP event numbers and capture register addresses */

} bissc_pruicss_xchg;

#ifdef __cplusplus
}
#endif

#endif
