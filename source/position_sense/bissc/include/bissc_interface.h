/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
/*                           Macros                                           */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */
/**
 *    \brief    Structure defining BiSSC Raw data received from encoder.
 *
 */
struct  raw_data
{
    volatile uint32_t   pos_data_word0;
    /**< Initial (<=32) position bits received */
    volatile uint32_t   pos_data_word1;
    /**< position bits received after the initial 32 bits (if applicable) */ 
};
/**
 *    \brief    Structure defining BiSSC Position data lengths of conected encoder.
 *
 *    \details  Number of encoders connected in daisy chain, and their position data lengths(resolution). 
 */
struct  enc_len
{
    volatile uint8_t num_encoders;
    /**< Number of Encoders connected */
    volatile uint8_t data_len[3];
    /**< Data length of individual encoder connected in daisy chain*/
};
/**
 *    \brief    Structure defining BiSSC Position data results.
 *
 *    \details  raw data, position data crc error count, 6-bit otf crc. 
 */
struct  pos_data_res
{
    struct  raw_data raw_data[3];
    /**< Structure defining BiSSC Raw data received from encoder*/
    volatile uint32_t   pd_crc_err_cnt[3];
    /**< Position data CRC error count */
    volatile uint8_t    pos_data_otf_crc[3];
    /**< position data otf  crc bits*/
};
/**
 *    \brief    Structure defining BiSSC Channel specific control communication(ctrl) results
 *
 *    \details  ctrl crc error count,cds results, ctrl 4 bit received crc and ctrl otf crc. 
 *
 */
struct ctrl_res
{
    volatile uint32_t ctrl_crc_err_cnt; 
    /**< control communication CRC error count */
    volatile uint8_t ctrl_cds_res;
     /**< control communication result*/
    volatile uint8_t ctrl_rcvd_crc;
    /**< control communication crc received*/
    volatile uint8_t ctrl_otf_crc;
    /**< control communication otf crc computed*/
};
/**
 *    \brief    Structure defining BiSSC interface
 *
 *    \details  Firmware config, command and channel interface
 *
 */
struct bissc_pruicss_xchg
{
    volatile uint8_t pos_crc_len;
    /**< Position data CRC length */
    volatile uint8_t rx_clk_freq;
    /**< Clock frequency */
    volatile uint8_t ctrl_cmd_crc_len;    
    /**< Control command CRC length */
    volatile uint8_t  channel;
    /**< Channel configuration */

    volatile uint8_t status[3];
    /**< Firmware initialization status */
    volatile uint8_t primary_core_mask;
    /**< Primary core mask incase of load share */
    volatile uint8_t cycle_trigger[3];
    /**< BiSSC cycle complete status */
    volatile uint8_t measure_proc_delay;
    /**< measure processing delay - do this for every config change */
    volatile uint8_t opmode[3];
    /**< operation mode status: '0' for periodic trigger and '1' for host trigger */

    struct  enc_len  enc_len[3];
    /**< position data lengths(resolution) of BiSSC encoders connected */

    volatile uint8_t   valid_bit_idx;
    /**< channel Bit Index */
    volatile uint8_t   fifo_bit_idx;
    /**< Fifo Bit Index(middle bit) */
    volatile uint8_t   ctrl_cmd_status[3];
    /**< control communication status */

    volatile uint16_t  proc_delay[3];
    /**< automatically estimated processing delay */


    volatile uint32_t ctrl_cmd[3];
    /**< Hex equivalent Control command */
    
    volatile uint32_t max_proc_delay;
    /**< maximum processing delay for selected frequency  */

    struct  pos_data_res pos_data_res[3];
    /**< Channel specific position data results*/

    struct ctrl_res ctrl_res[3];
    /**< Channel specific control communication results*/
    
    volatile uint8_t   execution_state[3];
    /**< Execution state for different channels in load share   */
    
    volatile uint64_t  register_backup[3];
    /**< Backup registers for ch0 - ch2 */

    volatile uint8_t   bissc_re_measure_proc_delay;
    
    volatile uint32_t delay_40us;
    /**< BiSS-C Timeout delay  */
    
    volatile uint32_t delay_100ms;
    /**< BiSS-C max interframe delay  */
    
    volatile uint64_t icssg_clk;
    /**< ICSSG core clock frequency  */
};

#ifdef __cplusplus
}
#endif

#endif
