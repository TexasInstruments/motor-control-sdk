/*
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated
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

#ifndef NIKON_INTERFACE_H_
#define NIKON_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum number of channels per PRU-ICSS slice */
#define NIKON_NUM_CH_PER_SLICE_MAX          (3U)
/* Maximum number of Nikon Encoders connected in bus connection*/
#define NUM_ENCODERS_MAX                    (8U)
/* Maximum number of Received Data Field Frames */
#define NUM_DATA_FIELDS_MAX                 (5U)
/* Maximum number of Memory Data Field Frames */
#define NUM_MDF_MAX                         (4U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining Nikon Position data results.
 *
 *    \details  IF frame, Data field 0 to Data field 4. The number of valid
 *              data fields depends on the command issued (1 to NUM_DATA_FIELDS_MAX).
 */
typedef struct nikon_raw_data_s
{
    volatile uint16_t    info_field[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Information field received from the encoder */
    volatile uint16_t    data_field[NUM_DATA_FIELDS_MAX][NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Data fields received from the encoder (DF0 to DF4) */
} nikon_raw_data;

/**
 *    \brief    Structure defining Nikon CRC data
 *
 *    \details  Contains on-the-fly calculated CRC, received CRC from encoder,
 *              and cumulative CRC error count per channel.
 */
typedef struct nikon_crc_s
{
    volatile uint32_t pd_crc_err_cnt[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Position data CRC error count */
    volatile uint8_t pos_otf_crc[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Position data on-the-fly calculated CRC bits */
    volatile uint8_t pos_rcv_crc[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Position data received CRC bits */
} nikon_crc;

/**
 *    \brief    Structure defining Nikon position data result (raw data + CRC)
 *
 *    \details  Contains raw data frames received from the encoder and the
 *              corresponding CRC information (received CRC, calculated CRC,
 *              and error count).
 */
typedef struct nikon_pos_data_res_s
{
    nikon_raw_data raw_data;
    /**< Raw data received from encoder */
    nikon_crc crc;
    /**< Calculated CRC, Received CRC and CRC error count */
} nikon_pos_data_res;
/**
 *    \brief    Structure defining Nikon periodic trigger configuration
 *
 *    \details  Contains IEP event numbers and capture register addresses for periodic trigger mode
 */
typedef struct nikon_periodic_trigger_cfg_s
{
    uint8_t iep_cmp_event;
    /**< IEP compare event number for periodic CMP mode */

    uint8_t iep_cap_event;
    /**< IEP capture event number for periodic CAP mode */

    uint16_t reserved;
    /**< Reserved for alignment */

    uint32_t iep_capture_reg;
    /**< IEP capture register address for periodic CAP mode */
} nikon_periodic_trigger_cfg;

/**
 *    \brief    Structure defining Nikon interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
typedef struct nikon_pruicss_xchg_s
{
    volatile uint8_t cycle_trigger[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Nikon cycle trigger/complete status */
    volatile uint8_t channel;
    /**< Channel configuration */
    volatile uint8_t num_encoders[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Number of Encoders connected */
    volatile uint8_t pos_crc_len;
    /**< Position data CRC length */
    volatile uint16_t rx_frame_size[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Rx frame size to be configured*/
    volatile uint8_t valid_bit_idx;
    /**< Channel Bit Index */
    volatile uint8_t fifo_bit_idx;
    /**< Fifo Bit Index(middle bit) */
    volatile uint8_t rx_clk_freq;
    /**< Clock frequency */
    volatile uint8_t is_memory_access;
    /**< Status for memory write*/
    volatile uint8_t num_rx_frames;
    /**< Number of Rx frames to be receive */
    volatile uint8_t multi_transmission_delay;
    /**< t5(m)-t6-t5(m-1) delay between 2 consecutive
     * responses of encoders connected in bus*/
    volatile uint8_t pru_sync_status[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< status flag for synchronization of PRUs in load share*/
    volatile uint8_t primary_core_mask;
    /**< Primary core mask incase of load share */
    volatile uint8_t opmode[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Operation mode per channel:
     *   0 = periodic trigger CMP mode (NIKON_CONFIG_PERIODIC_TRIGGER_CMP_MODE)
     *   1 = host trigger mode (NIKON_CONFIG_HOST_TRIGGER_MODE)
     *   2 = periodic trigger CAP mode (NIKON_CONFIG_PERIODIC_TRIGGER_CAP_MODE) */
    volatile uint32_t cdf_frame[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Command to be transmitted to encoder */
    volatile uint32_t mdf_frame[NIKON_NUM_CH_PER_SLICE_MAX][NUM_MDF_MAX];
    /**< Memory data frames to be transmitted to encoder */
    volatile uint32_t num_mdf;
    /**< Number of MDFs to be sent */
    volatile uint32_t delay_1us;
    /**< Nikon Minimum delay between memory access commands */
    volatile uint64_t icss_clk;
    /**< ICSS core clock frequency */
    nikon_pos_data_res pos_data_res[NUM_ENCODERS_MAX];
    /**< Results extracted from raw data received */
    uint32_t iep_base_address;
    /**< IEP register base address offset from PRU-ICSS base, used for periodic trigger mode */
    nikon_periodic_trigger_cfg trigger_params[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Periodic trigger configuration parameters for each channel (ch0, ch1, ch2).
     *   Contains IEP event numbers and capture register addresses */
} nikon_pruicss_xchg;

#ifdef __cplusplus
}
#endif

#endif
