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
/* Single PRU - Single channel configuration */
#define BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU (0U)
/* Single PRU - Multichannel configuration */
#define BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU (1U)
/* Multichannel - Load Share configuration */
#define BISSC_MODE_MULTI_CHANNEL_MULTI_PRU (2U)

/*  Minimum and Maximum BiSSC cycle time depends on various params as below:
    TCycle_min = TMA ∗ (5 + DLEN + CRCLEN) + tLineDelay + tbusy_max + busy_s_max + tTO
    Instead wait for max of 5 ms as this can vary for different encoders and for daisy chain
*/
#define BISSC_MAX_CYCLE_TIMEOUT             5
/* Maximum number of Endat Channels*/
#define NUM_ED_CH_MAX                       3
/* Maximum number of BiSS-C Encoders connected in Daisy chain*/
#define NUM_ENCODERS_MAX                    3
/* Max processing delay as per spec - values are in valid bits/clock cycles*/
#define BISSC_MAX_PROC_DELAY_1MHZ           40
#define BISSC_MAX_PROC_DELAY_2MHZ           80
#define BISSC_MAX_PROC_DELAY_5MHZ           200
#define BISSC_MAX_PROC_DELAY_8MHZ           320
#define BISSC_MAX_PROC_DELAY_10MHZ          400
#define BISSC_CTS_BIT                       1       /* CTS bit for control communication */
#define BISSC_ENC_ID_LEN                    3       /* Number of Encoder ID bits */
#define BISSC_ENC_ID_MASK                   0x7     /* Mask for encoder ID*/
#define BISSC_REG_ADDR_LEN                  7       /* Number of bits for Register address */
#define BISSC_REG_ADDR_MASK                 0x7F    /* Mask for Register address */
#define BISSC_RWS_LEN                       3       /* Number of RWS bits */
#define BISSC_RWS_MASK                      0x7     /* Mask for RWS bits */
#define BISSC_CTRL_READ_ACCESS              0x5     /* RWS = "101" for read access of register */
#define BISSC_CTRL_WRITE_ACCESS             0x3     /* RWS = "011" for write access of register */
#define BISSC_REG_DATA_LEN                  8       /* Number of Register data bits */
#define BISSC_REG_DATA_MASK                 0xFF    /* Mask for Register data */
#define BISSC_CTRL_STOP_LEN                 2       /* Number of stop bits PS, P: stop bit for one frame, S: stop bit for sequential control communication */
#define BISSC_RX_SAMPLE_SIZE                7       /* 8x over clock */
#define BISSC_RX_SAMPLE_SIZE_10MHZ          3       /* 4x over clock */
#define BISSC_POS_CRC_LEN                   6       /* Number of position data CRC bits */
#define BISSC_EW_LEN                        2       /* Number of Error and Warning bits */
#define BISSC_CTRL_CMD_CRC_LEN              4       /* Number of CTRL cmd CRC bits */
#define BISSC_CTRL_CMD_CRC_MASK             0xF     /* Mask for CTRL cmd CRC bits */
#define BISSC_POS_DATA_LEN_DEFAULT          12      /* Default data length instead of garbage*/
#define BISSC_SAFETY_CRC_LEN                16      /* Number of Safety CRC bits */
#define BISSC_RX_ENABLE_FRACTIONAL_DIV      (1<<15) /* Enable fractional divider 1.5 for RX */
/* Allowed frequencies in MHz for BiSSC */
#define BISSC_FREQ_1MHZ                     1
#define BISSC_FREQ_2MHZ                     2
#define BISSC_FREQ_5MHZ                     5
#define BISSC_FREQ_8MHZ                     8
#define BISSC_FREQ_10MHZ                    10

/**
 *    \brief    Structure defining EnDat clock configuration for selected frequency
 *
 *    \details  Rx, Tx divisors for selected frequency, oversampling rate, core_clk/uart clock source status.
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
    uint16_t  is_core_clk;
    /**< Status for core_clk clock source*/
};
/**
 *    \brief    Structure defining BiSSC Position data results
 *
 *    \details  position data result(raw data, angle and number of turns), error-warning, 6-bit received crc, 6-bit otf crc.
 */
struct bissc_position_info
{
    uint64_t          position[NUM_ENCODERS_MAX];
    /**< Position data results from each encoder connected in daisy chain*/
    float             angle[NUM_ENCODERS_MAX];
    /**< Single turn result(Angle) for each encoder connected in daisy chain*/
    uint32_t          num_of_turns[NUM_ENCODERS_MAX];
    /**< Multi turn result(No. of rotations) for each encoder connected in daisy chain*/
    uint8_t           ew[NUM_ENCODERS_MAX];
    /**< Error and Warning result for each encoder connected in daisy chain*/
    uint8_t           rcv_crc[NUM_ENCODERS_MAX];
    /**< Received 6-bit crc for each encoder connected in daisy chain*/
    uint8_t           otf_crc[NUM_ENCODERS_MAX];
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
    int32_t data_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Resolution of each encoder connected in daisy chain config to each channel*/
    int32_t single_turn_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Single turn resolution of each encoder connected in daisy chain config to each channel*/
    int32_t multi_turn_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Multi turn resolution of each encoder connected in daisy chain config to each channel*/
    int32_t channel[NUM_ED_CH_MAX];
    /**< Array of all configured channel*/
    struct bissc_pruicss_xchg *pruicss_xchg;
    /**< Structure defining BiSSC interface*/
    int32_t has_safety[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Status for safety support*/
    int32_t sign_of_life_cnt[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< 6-bit sign of life counter, it'll give number of cycles triggered since power on*/
    int32_t rcv_safety_crc[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Received 16-bit safety crc*/
    int32_t calc_safety_crc[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Calculated 16-bit safety crc*/
    int32_t is_continuous_mode;
    /**< Continuous Mode is opted*/
    void *pruicss_cfg;
    /**< PRU-ICSS cfg registers base offset*/
    int64_t raw_data;
    /**< Raw data received from encoder*/
    struct bissc_position_info enc_pos_data[NUM_ED_CH_MAX];
    /**< Structure containing encoders position data results for each channel*/
    struct bissc_control_info enc_ctrl_data[NUM_ED_CH_MAX];
    /**< Structure containing encoders control communication results for each PRU*/
    int32_t pd_crc_err_cnt[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Position data crc error count for each encoder connected in daisy chain to each channel*/
    int32_t ctrl_crc_err_cnt[NUM_ED_CH_MAX];
    /**< Control communication crc error count for each encoder connected to each PRU in load share*/
    int32_t num_encoders[NUM_ED_CH_MAX];
    /**< Number of encoders connected in daisy chain to each PRU in load share*/
    int8_t ctrl_write_status[NUM_ED_CH_MAX];
    /**< Control communication Read or Write status*/
    uint32_t ctrl_reg_address[NUM_ED_CH_MAX];
    /**< Control communication register address*/
    uint32_t ctrl_reg_data[NUM_ED_CH_MAX];
    /**< Control communication register data*/
    uint32_t ctrl_enc_id[NUM_ED_CH_MAX];
    /**< Encoder ID for control communication*/
    int32_t totalchannels;
    /**< Total number of channels configured*/
    uint16_t  proc_delay[NUM_ED_CH_MAX];
    /**< Measured Processing delay of individual channel*/
    uint32_t baud_rate;
    /**< Input baudrate*/
    uint32_t core_clk_freq;
    /**< Core clock frequency*/
    uint32_t uart_clk_freq;
    /**< UART clock frequency*/
    void *pruicss_iep;
    /**< ICSS IEP base address*/
    int64_t cmp3;
    /**< IEP CMP3 reg used in periodic trigger mode*/
};

#ifdef __cplusplus
}
#endif

#endif
