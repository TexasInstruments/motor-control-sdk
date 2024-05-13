/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
/* Maximum number of 3-ch peripheral interface Channels*/
#define NUM_ED_CH_MAX                       3
/* Maximum number of Nikon Encoders connected in bus connection*/
#define NUM_ENCODERS_MAX                    3
/* Maximum number of Received data field frames */
#define NUM_DATA_FIELDS_MAX                 3
/* Maximum number of Memory data field frames */
#define NUM_MDF_MAX                         3


/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 *    \brief    Structure defining Nikon Position data results.
 *
 *    \details  IF frame, Data field 0, Data field 1, Data field 2.
 */
struct  raw_data
{
    volatile uint16_t    info_field[NUM_ED_CH_MAX];
    /**<Information Field receive from the encoder*/
    volatile uint16_t    data_field[NUM_DATA_FIELDS_MAX][NUM_ED_CH_MAX];
    /**<Data fields receive from the encoder*/
};
struct crc
{
    volatile uint32_t pd_crc_err_cnt[NUM_ED_CH_MAX];
    /**< Position data crc error count*/
    volatile uint8_t pos_otf_crc[NUM_ED_CH_MAX];
    /**< Position data otf crc bits*/
    volatile uint8_t pos_rcv_crc[NUM_ED_CH_MAX];
    /**< Position data receive crc bits*/

};
struct pos_data_res
{
    struct   raw_data raw_data;
    /**< Raw data receive from encoder*/
    struct crc crc;
    /**< Calculated CRC, Received CRC and CRC error count */
};
/**
 *    \brief    Structure defining Nikon interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
struct nikon_pruicss_xchg
{
    volatile uint8_t cycle_trigger[NUM_ED_CH_MAX];
    /**< Nikon cycle trigger/complete status */
    volatile uint8_t channel;
    /**< Channel configuration */


    volatile uint8_t num_encoders[NUM_ED_CH_MAX];
    /**< Number of Encoders connected */
    volatile uint8_t pos_crc_len;
    /**< Position data CRC length */

    volatile uint16_t rx_frame_size[NUM_ED_CH_MAX];
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

    volatile uint8_t pru_sync_status[NUM_ED_CH_MAX];
    /**< status flag for synchronization of PRUs in load share*/
    volatile uint8_t primary_core_mask;
    /**< Primary core mask incase of load share */

    volatile uint8_t opmode[NUM_ED_CH_MAX];
    /**< operation mode status: '0' for periodic trigger
     * and '1' for host trigger */

    volatile uint32_t cdf_frame[NUM_ED_CH_MAX];
    /**< Command to be transmitted to Tx*/
    volatile uint32_t mdf_frame[NUM_MDF_MAX];
    /**< Memory data frames MDF 0,1 indexes for data
     * and MDF 2 index for EEPROM address*/


    struct   pos_data_res   pos_data_res[NUM_ENCODERS_MAX];
    /**< Results extracted from raw data receivedd */
    volatile uint32_t delay_10us;
    /**< Nikon Minimum delay between memory access commands */
    volatile uint32_t delay_300us;
    /**< Nikon Max membusy duration */

    volatile uint32_t delay_30ms;
    /**< Nikon max interframe delay  */

    volatile uint64_t icssg_clk;
    /**< ICSSG core clock frequency  */
};

#ifdef __cplusplus
}
#endif

#endif
