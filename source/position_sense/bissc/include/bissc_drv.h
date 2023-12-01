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


#ifndef BISSC_DRV_H_
#define BISSC_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <position_sense/bissc/include/bissc_interface.h>
#define BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU (0U)
#define BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU (1U)
#define BISSC_MODE_MULTI_CHANNEL_MULTI_PRU (2U)

#define BISSC_NUM_BITS_POSITION_CRC 6
#define BISSC_NUM_BITS_EW           2
#define BISSC_NUM_BITS_ADDRESS      8

#define BISSC_MAX_CYCLE_TIMEOUT     5
#define NUM_ED_CH_MAX               3
#define NUM_SLAVE_MAX               3
/* Max processing delay at 1 MHz is 40 us as per spec - values are in valid bits*/
#define BISSC_MAX_PROC_DELAY_1MHZ           40
#define BISSC_MAX_PROC_DELAY_2MHZ           80
#define BISSC_MAX_PROC_DELAY_5MHZ           200
#define BISSC_MAX_PROC_DELAY_8MHZ           320
#define BISSC_MAX_PROC_DELAY_10MHZ          400

#define BISSC_RX_SAMPLE_SIZE                7       /* 8x over clock */
#define BISSC_RX_SAMPLE_SIZE_10MHZ          3       /* 4x over clock */
#define BISSC_POS_CRC_LEN                   6
#define BISSC_EW_LEN                        2
#define BISSC_CTRL_CMD_CRC_LEN              4
#define BISSC_POS_DATA_LEN_DEFAULT          12

#define BISSC_INPUT_CLOCK_UART_FREQUENCY    192000000
#define BISSC_INPUT_CLOCK_OCP_FREQUENCY    200000000
#define BISSC_INPUT_CLOCK_FREQUENCY_UART    BISSC_INPUT_CLOCK_UART_FREQUENCY
/**
 *    \brief    Structure defining EnDat clock configuration for selected frequency
 *
 *    \details  Rx, Tx divisors for selected frequency, oversampling rate, ocp/uart clock source status. 
 *
 */
struct bissc_clk_cfg
{
    uint16_t  rx_div;
    /**< Rx divisor for selected frequency*/
    uint16_t  tx_div;
    /**< Tx divisor for selected frequency*/
    uint16_t  rx_div_attr;
    /**< Rx oversampling rate*/
    uint16_t  is_ocp;
    /**< status for ocp clock source*/
};
/**
 *    \brief    Structure defining BiSSC Position data results
 *
 *    \details  position data result(raw data, angle and number of turns), error-warning, 6-bit received crc, 6-bit otf crc. 
 */
struct bissc_position_info
{
    uint64_t          position[NUM_SLAVE_MAX];
    /**< Position data results from each encoder connected in daisy chain*/
    float             angle[NUM_SLAVE_MAX];
    /**< Single turn result(Angle) for each encoder connected in daisy chain*/
    uint32_t          num_of_turns[NUM_SLAVE_MAX];
    /**< Multi turn result(No. of rotations) for each encoder connected in daisy chain*/
    uint8_t           ew[NUM_SLAVE_MAX];
    /**< Error and Warning result for each encoder connected in daisy chain*/
    uint8_t           rcv_crc[NUM_SLAVE_MAX];
    /**< received 6-bit crc for each encoder connected in daisy chain*/
    uint8_t           otf_crc[NUM_SLAVE_MAX];
    /**< Calculated otf 6-bit crc for each encoder connected in daisy chain*/
};
/**
 *    \brief    Structure defining BiSSC Channel specific control communication(ctrl) results
 *
 *    \details  cds results, ctrl 4 bit received crc and ctrl otf crc. 
 *
 */
struct bissc_control_info
{
    uint8_t           cmd_result;
    /**< Control communication result received from the encoder*/
    uint8_t           cmd_rcv_crc;
    /**< Control Communication received 4-bit crc*/
    uint8_t           cmd_otf_crc;
    /**< Control Communication calculated otf crc*/
};
/**
 *    \brief    Initialize BiSS-C firmware interface address and get the pointer to struct bissc_priv instance
 *
 *    \details  contains ICSS slice, load share, data lengths, channel, safety specific information and a pointer to bissc_pruicss_xchg structure. 
 *
 */
struct bissc_priv
{   
    int32_t pruicss_slicex;
    /**< PRU ICSS slice number*/
    int32_t load_share;
    /**< Load share flag*/
    int32_t data_len[NUM_ED_CH_MAX][NUM_SLAVE_MAX];
    /**< Resolution of each encoder connected in daisy chain config to each channel*/
    int32_t single_turn_len[NUM_ED_CH_MAX][NUM_SLAVE_MAX];
    /**< Single turn resolution of each encoder connected in daisy chain config to each channel*/
    int32_t multi_turn_len[NUM_ED_CH_MAX][NUM_SLAVE_MAX];
    /**< Multi turn resolution of each encoder connected in daisy chain config to each channel*/
    int32_t channel[NUM_ED_CH_MAX];
    /**< Array of all configured channel*/
    struct bissc_pruicss_xchg *pruicss_xchg;
    /**< Structure defining BiSSC interface*/
    int32_t has_safety;
    /**< status for safety support*/
    void *pruicss_cfg;
    /**< PRU-ICSS cfg registers base offset*/
    int64_t raw_data;
    /**< Raw data received from encoder*/
    struct bissc_position_info enc_pos_data[NUM_ED_CH_MAX];
    /**< Structure containing encoders position data results for each channel*/
    struct bissc_control_info enc_ctrl_data[NUM_ED_CH_MAX];
    /**< Structure containing encoders control communication results for each PRU*/
    int32_t pd_crc_err_cnt[NUM_ED_CH_MAX][NUM_SLAVE_MAX];
    /**< position data crc error count for each encoder connected in daisy chain to each channel*/
    int32_t ctrl_crc_err_cnt[NUM_ED_CH_MAX];
    /**< control communication crc error count for each encoder connected to each PRU in load share*/
    int32_t num_slaves[NUM_ED_CH_MAX];
    /**< Number of slaves connected in daisy chain to each PRU in load share*/
    int32_t totalchannels;
    /**< Total number of channels configured*/
};


#include "bissc_api.h"

#ifdef __cplusplus
}
#endif

#endif
