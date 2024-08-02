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


#ifndef NIKON_DRV_H_
#define NIKON_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <position_sense/nikon/include/nikon_interface.h>

/*  Minimum and Maximum NIKON cycle time depends on various params as below:
    TCycle_max = TMA ∗ (number of RX frames * 18) + TX frame(32)
    + delay between TX and RX + (delay between Multi Transmission commands
    * (maximum encoder address delay t6))
    + Delay between two EEPROM access commands(30milisec)
    TCycle_min = TMA * (number of Rx frames * 18) + TX frame(32)
    + delay between TX and RX
    Instead wait for max of 35 ms as this can vary for different encoders,
    different commands and multi transmission connection.
*/
/* Single PRU - Single channel configuration */
#define NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU (0U)
/* Single PRU - Multichannel configuration */
#define NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU (1U)
/* Multichannel - Load Share configuration */
#define NIKON_MODE_MULTI_CHANNEL_MULTI_PRU (2U)

/* 35 milisec as max cycle timeout (more than 30 mili sec delay is required
between two cycles in memory access commands) */
#define NIKON_MAX_CYCLE_TIMEOUT             35
/* Maximum number of 3-ch peripheral interface Channels*/
#define NUM_ED_CH_MAX                       3
/* Maximum number of Nikon Encoders connected in bus connection*/
#define NUM_ENCODERS_MAX                    3
/* Maximum number of Memory Data Frames to be sent on Tx*/
#define NUM_MDF_CMD_MAX                     3

#define NIKON_RX_SAMPLE_SIZE_16MHZ          3       /* 4x over sample rate */
#define NIKON_RX_SAMPLE_SIZE_6_67MHZ        5       /* 6x over sample rate */
#define NIKON_RX_SAMPLE_SIZE                7       /* 8x over sample rate */
/* Allowed frequencies in MHz for NIKON */
#define NIKON_FREQ_2_5MHZ                   2.5     /* 2.5 MHz frequency */
#define NIKON_FREQ_4MHZ                     4       /* 4 MHz frequency */
#define NIKON_FREQ_6_67MHZ                  6       /* 6.67 MHz frequency */
#define NIKON_FREQ_8MHZ                     8       /* 8 MHz frequency */
#define NIKON_FREQ_16MHZ                    16      /* 16 MHz frequency */
/* Number of Rx data CRC bits */
#define NIKON_POS_CRC_LEN                   8
/* Default data length instead of garbage*/
#define NIKON_POS_DATA_LEN_DEFAULT          17

/* Enable cycle trigger for firmware*/
#define NIKON_ENABLE_CYCLE_TRIGGER          0x1
 /* Disable cycle trigger */
#define NIKON_DISABLE_CYCLE_TRIGGER         0x0
/* Configure firmware in continuous mode*/
#define NIKON_CONFIG_PERIODIC_TRIGGER_MODE  0x0
/* Configure firmware in host trigger mode*/
#define NIKON_CONFIG_HOST_TRIGGER_MODE      0x1
/* General Macro for clearing any status flag */
#define NIKON_CLEAR_STATUS_FLAG             0x0
/* General Macro for setting any status flag */
#define NIKON_SET_STATUS_FLAG               0x1

#define NIKON_CHANNEL0_MASK                 0x1     /* Mask for channel 0 */
#define NIKON_CHANNEL1_MASK                 0x2     /* Mask for channel 1 */
#define NIKON_CHANNEL2_MASK                 0x4     /* Mask for channel 2 */

/* 1000usec sleep for timeout count*/
#define NIKON_1MILLISEC_SLEEP_TIME          1000
/* 30milli sec delay for EEPROM update */
#define NIKON_30_MILLI_SEC_DELAY            30000

/* Middle bit indexes for Given Oversampling rates */
#define NIKON_FIFO_BIT_IDX_8X_OS            4       /* 8x Oversampling */
#define NIKON_FIFO_BIT_IDX_6X_OS            3       /* 6x Oversampling */
#define NIKON_FIFO_BIT_IDX_4X_OS            2       /* 4x Oversampling */

#define NIKON_BASE_VALID_BIT_IDX            24      /* Base valid bit index */

/* Lengths of the Specified fields */
#define NIKON_DB_BITS_LEN                   10      /* temperature bits*/
#define NIKON_RX_ONE_FRAME_LEN              16      /* Rx frame*/
#define NIKON_EEPROM_ADDR_LEN               8       /* EEPROM memory address*/
#define NIKON_COMMAND_CODE_LEN              5       /* command code */
#define NIKON_ENC_STATUS_LEN                4       /* encoder status field */
#define NIKON_ENC_ADDR_LEN                  3       /* encoder address */
#define NIKON_SYNC_CODE_LEN                 3       /* sync code */
#define NIKON_TX_CRC_LEN                    3       /* 3bit Tx CRC */
#define NIKON_FRAME_CODE_LEN                2       /* frame code */
#define NIKON_START_BIT_LEN                 1       /* start bit */
#define NIKON_STOP_BIT_LEN                  1       /* stop bit */
#define NIKON_FIXED_BIT_LEN                 1       /* fix bit in info field*/

/* Status for request or access */
#define NIKON_EEPROM_READ_ACCESS            1       /* eeprom read access */
#define NIKON_EEPROM_WRITE_ACCESS           2       /* eeprom write access */
#define NIKON_ENABLE_ID_CODE_WRITE          2       /* ID code write request */

/* Maximum possible parameters */
#define NIKON_MAX_NUM_RX_FRAMES             4       /* Rx frames */
#define NIKON_MAX_ABS_LEN                   40      /* ABS data length */
#define NIKON_MAX_NUM_DATA_FIELDS           3       /* Rx Data fields */
/* Common parameters for most of the commands */
#define NIKON_AVG_NUM_RX_FRAMES             3       /* Rx frames */
#define NIKON_AVG_ABS_LEN                   24      /* ABS data length*/
/*Minimum possible parameters */
#define NIKON_MIN_NUM_RX_FRAMES             2       /* Rx frames */
#define NIKON_MIN_ABS_LEN                   17      /* ABS data length*/

/* Masks for speecified fields */
#define NIKON_DB_BITS_MASK                  0x3FF   /* temperature bits(DB) */
#define NIKON_ENC_STATUS_MASK               0xF     /* encoder status field */
#define NIKON_CMD_CODE_MASK                 0x1F    /* command code field*/
#define NIKON_ENC_ADDR_MASK                 0x7     /* encoder address field */

/* Number of cycles required to perform operation or reset specific commands */
#define NIKON_NUM_OF_CYCLE_FOR_RESET        7
/**
*    \brief    Command codes[4:0]
*/
enum cmd_code
{
    CMD_0,                  /**< ABS full 40bit data request*/
    CMD_1,                  /**< ABS lower 24bit data request */
    CMD_2,                  /**< ABS upper 24bit data request */
    CMD_3,                  /**< Encoder status request*/
    CMD_4,                  /**< ABS full 40bit data request (MT) */
    CMD_5,                  /**< ABS lower 24bit data request (MT) */
    CMD_6,                  /**< ABS upper 24bit data request (MT)*/
    CMD_7,                  /**< Encoder status request (MT) */
    CMD_8,                  /**< Status flag clear request */
    CMD_9,                  /**< Multiple turn data clear request */
    CMD_10,                 /**< Status+ Multiple turn data clear request*/
    CMD_11,                 /**< Encoder address setting I (one-to-one) */
    CMD_12,                 /**< Single turn data zero preset*/
    CMD_13,                 /**< EEPROM read request*/
    CMD_14,                 /**< EEPROM write request */
    CMD_15,                 /**< Temperature data (8bit) request */
    CMD_16,                 /**< Identification code read I*/
    CMD_17,                 /**< Identification code read II (one-to-one) */
    CMD_18,                 /**< Identification code write I*/
    CMD_19,                 /**< Identification code write II (one-to-one) */
    CMD_20,                 /**< Encoder address setting II*/
    CMD_21,                 /**< ABS lower 17bit data request*/
    CMD_22,                 /**< ABS lower 17bit data request (MT) */
    CMD_27 = 27,            /**< ABS lower 24bit data + Status request*/
    CMD_28,                 /**< ABS lower 24bit data + Status request (MT) */
    CMD_29,                 /**< ABS lower 24bit data
                                 + Temperature data request*/
    CMD_30,                 /**< ABS lower 24bit data
                                 + Temperatuure data request (MT) */
    /**< Update Encoder address(EAX) in APP local context*/
    ENCODER_ADR_CHANGE,
    START_CONTINUOUS_MODE,  /**< Start periodic trigger mode*/
    UPDATE_CLOCK_FREQ,      /**< Update operating baud rate as specified by user*/
    UPDATE_ENC_LEN,         /**< Update encoder's single turn and multi turn resolution*/
    CMD_CODE_NUM
};

/**
 *    \brief    Structure defining 3-ch peripheral interface
 *              clock configuration for selected frequency
 *
 *    \details  Rx, Tx divisors for selected frequency, oversampling rate,
 *              ocp/uart clock source status.
 *
 */
struct nikon_clk_cfg
{
    uint16_t  rx_div;
    /**< Rx divisor for selected frequency*/
    uint16_t  tx_div;
    /**< Tx divisor for selected frequency*/
    uint16_t  rx_div_attr;
    /**< Rx oversampling rate*/
    uint16_t  is_core_clk;
    /**< status for core clock source*/
};
struct pos_data_info
{
    uint32_t raw_data0[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - IF0 */
    uint32_t raw_data1[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF0 */
    uint32_t raw_data2[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF1 */
    uint32_t raw_data3[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF2 */
    uint32_t rcv_crc[NUM_ENCODERS_MAX];
    /**< 8-bit receive position sense crc*/
    uint32_t otf_crc[NUM_ENCODERS_MAX];
    /**< 8-bit calculated otf crc*/
    uint32_t crc_err_cnt[NUM_ENCODERS_MAX];
    /**< Position data crc error count*/
    uint64_t abs[NUM_ENCODERS_MAX];
    /**< Absolute data(position data) receivedd from the encoder */
    uint32_t multi_turn[NUM_ENCODERS_MAX];
    /**< Total number of complete rotations(360) */
    float angle[NUM_ENCODERS_MAX];
    /**< Angle of encoder shaft*/
};
struct enc_info
{
    uint32_t enc_status[NUM_ENCODERS_MAX];
    /**< Encoder status*/
    uint32_t enc_cmd[NUM_ENCODERS_MAX];
    /**< Command acknowledge by encoder*/
    uint32_t enc_addr[NUM_ENCODERS_MAX];
    /**<Encoder address acknowledged by the encoder*/
};
/**
 *    \brief    Structure defining Alarm bits receivedd by the encoder
 *
 *    \details  Alarm for battery voltage beyond a specific band, over flow,
 *              over speed, over temperature, Memory , single turn
 *              and multi turn errors and busy flags.
 *
 */
struct alm_bits
{
    uint8_t batt;
    /**<  Alarm occurs when the internal battery voltage
     *    drop below the specified value*/
    uint8_t mt_err;
    /**< The higher voltage between the built-in backup capacitor in the
         encoder or the external battery becomes 2.45V (TYP) or less*/
    uint8_t ov_flow;
    /**< OvFlow='1' is outputted when "-32768 to +32767" is exceeded
        as an amount of rotations. The multi-turn counter operates as a cyclic
        counter of 0 to 65535 even after overflow*/
    uint8_t ov_spd;
    /**< Alarm occurs when Over speed is detected through
     * optical incremental signal or multi turn signal*/
    uint8_t mem_err;
    /**< This flag is turned on when an error occurs while accessing the
         EEPROM in the encoder. */
    uint8_t st_err;
    /**< To monitor the conformance between the “ABS block”
     *   and the “INC block".*/
    uint8_t ps_err;
    /**< To monitor the conformance between (1)“multi turn calculation block”
     * and (2)“single turn calculation block.” When comparing (1) and (2),if
     * (difference between (1) and (2)) ≧ (single turn) an alarm is turned on*/
    uint8_t busy;
    /**< This flag is turned on during the process
     *  to determine a single turn absolute value*/
    uint8_t mem_busy;
    /**< This flag shows that access to the EEPROM in the encoder is under way.
     * After the access is completed, the flag returns to “0.”*/
    uint8_t ov_temp;
    /**< It issues warning when the temperature sensor’s output
     * on board becomes beyond the specified value*/
    uint8_t inc_err;
    /**< When a signal failure (amplitude, level, etc.) in the incremental
     * signal phase A/ phase B is detected, this flag outputs ‘1.’*/
};
/**
 *    \brief    Initialize NIKON firmware interface address
 *              and get the pointer to struct nikon_priv instance
 *
 *    \details  contains ICSS slice, load share, data lengths, channel,
 *              safety specific information and a pointer to
 *              nikon_pruicss_xchg structure.
 *
 */
struct nikon_priv
{
    uint32_t pruicss_slicex;
    /**< PRU ICSS slice number*/
    uint32_t load_share;
    /**< Load share flag*/
    uint32_t data_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Resolution of encoder*/
    uint32_t num_encoders[NUM_ED_CH_MAX];
    /**< Number of encoders connected in bus to each PRU in load share*/
    uint32_t num_enc_access[NUM_ED_CH_MAX];
    /**< Number of encoders to access based on provided command code*/
    uint32_t single_turn_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Single turn resolution*/
    uint32_t multi_turn_len[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< Multi turn resolution*/
    uint32_t channel[NUM_ED_CH_MAX];
    /**< Arrray of all configured channel*/
    struct nikon_pruicss_xchg *pruicss_xchg;
    /**< Structure defining NIKON interface*/
    uint32_t has_safety;
    /**< Status for safety support*/
    void *pruicss_cfg;
    /**< PRU-ICSS cfg registers base offset*/
    uint32_t tx_cdf[NUM_ED_CH_MAX];
    /**< Command data frame to be transmitted to encoder*/
    uint32_t tx_mdf;
    /**< Memory data frame to be transmitted to encoder*/
    uint32_t num_rx_frames;
    /**< Number of Rx frames to be receive */
    uint32_t totalchannels;
    /**< Total number of channels configured*/
    float_t baud_rate;
    /**< Input baudrate*/
    uint32_t core_clk_freq;
    /**< Core clock frequency*/
    uint32_t uart_clk_freq;
    /**< UART clock frequency*/
    uint32_t eax[NUM_ED_CH_MAX];
    /**<Encoder address from the user*/
    uint32_t fc;
    /**<Frame code specified by the user*/
    uint32_t sync_code;
    /**<Synchronization code*/
    uint32_t tx_crc;
    /**<Tx 3 bit crc*/
    uint32_t mem_data[NUM_MDF_CMD_MAX];
    /**<Memory data in indexes 0,1 and memory address in index 2*/
    struct   pos_data_info  pos_data_info[NUM_ED_CH_MAX];
    /**<ABS, ALM, EEPROM or Identification code information extracted
     * from the data receive */
    struct   enc_info   enc_info[NUM_ED_CH_MAX];
    /**< Encoder's information(Encoder address, Encoder status and
     *   command given to the encoder) extracted from the data receive */
    uint32_t temperature[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**<Temperature */
    uint32_t identification_code[NUM_ED_CH_MAX];
    /**<Identification code of the current encoder */
    uint32_t alm_field[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**<ALM field receivedd from the encoder */
    struct alm_bits alm_bits[NUM_ED_CH_MAX][NUM_ENCODERS_MAX];
    /**< ALM bits receivedd from the encoder */
    uint32_t abs_len;
    /**< Length of absolute data receivedd from encoder */
    uint32_t is_continuous_mode;
    /**< Flag for continuous mode triggered */
    void *pruicss_iep;
    /**< ICSS IEP base address*/
    uint64_t cmp3;
    /**< IEP CMP3 reg used in periodic trigger mode*/

};

#ifdef __cplusplus
}
#endif

#endif
