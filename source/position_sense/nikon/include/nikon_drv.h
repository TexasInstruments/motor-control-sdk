/*
 *  Copyright (C) 2024-2026 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <drivers/pruicss.h>
#include <position_sense/nikon/include/nikon_interface.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Single PRU - Single channel configuration mode
 *
 *  Only one channel (ch0, ch1, or ch2) is used with a single PRU core.
 */
#define NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU    (0U)

/** \brief Single PRU - Multichannel configuration mode
 *
 *  Multiple channels (up to 3: ch0, ch1, ch2) are managed by a single PRU core.
 *  All channels share the same PRU core resources without load sharing.
 */
#define NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU     (1U)

/** \brief Multichannel - Load Share configuration mode
 *
 *  Multiple channels are distributed across multiple PRU cores (PRU, RTU-PRU, TX-PRU).
 */
#define NIKON_MODE_MULTI_CHANNEL_MULTI_PRU      (2U)

#define NIKON_RX_SAMPLE_SIZE_4X                 (3U)    /* 4x over sample rate */
#define NIKON_RX_SAMPLE_SIZE_5X                 (4U)    /* 5x over sample rate */
#define NIKON_RX_SAMPLE_SIZE_6X                 (5U)    /* 6x over sample rate */
#define NIKON_RX_SAMPLE_SIZE                    (7U)    /* 8x over sample rate */
#define NIKON_RX_ENABLE_FRACTIONAL_DIV          (1<<15) /* Enable fractional divider 1.5 for RX */

/** \brief Allowed Nikon communication frequencies in MHz
 */
#define NIKON_FREQ_2_5MHZ                       (2.5)     /* 2.5 MHz frequency */
#define NIKON_FREQ_4MHZ                         (4)       /* 4 MHz frequency */
#define NIKON_FREQ_6_67MHZ                      (6)       /* 6.67 MHz frequency */
#define NIKON_FREQ_8MHZ                         (8)       /* 8 MHz frequency */
#define NIKON_FREQ_16MHZ                        (16)      /* 16 MHz frequency */

#define PRU_CORE_CLK_FREQ_200MHZ                (200)
#define PRU_CORE_CLK_FREQ_300MHZ                (300)
#define PRU_UART_CLK_FREQ_160MHZ                (160)
#define PRU_UART_CLK_FREQ_192MHZ                (192)

/* Number of Rx data CRC bits */
#define NIKON_POS_CRC_LEN                       (8)
/* Default data length instead of garbage*/
#define NIKON_POS_DATA_LEN_DEFAULT              (17)

/* Enable cycle trigger for firmware*/
#define NIKON_ENABLE_CYCLE_TRIGGER              (0x1)
 /* Disable cycle trigger */
#define NIKON_DISABLE_CYCLE_TRIGGER             (0x0)
/* Configure firmware in periodic CMP trigger mode */
#define NIKON_CONFIG_PERIODIC_TRIGGER_CMP_MODE  (0x0)
/* Configure firmware in host trigger mode */
#define NIKON_CONFIG_HOST_TRIGGER_MODE          (0x1)
/* Configure firmware in periodic CAP trigger mode */
#define NIKON_CONFIG_PERIODIC_TRIGGER_CAP_MODE  (0x2)

/* General Macro for clearing any status flag */
#define NIKON_CLEAR_STATUS_FLAG                 (0x0)
/* General Macro for setting any status flag */
#define NIKON_SET_STATUS_FLAG                   (0x1)

#define NIKON_CHANNEL0_MASK                     (0x1)   /* Mask for channel 0 */
#define NIKON_CHANNEL1_MASK                     (0x2)   /* Mask for channel 1 */
#define NIKON_CHANNEL2_MASK                     (0x4)   /* Mask for channel 2 */

/* IEP event limits for periodic trigger mode */
#define NIKON_IEP_MAX_CAP_EVENT                 (0x8U)  /* Maximum CAP event number (0-7) */
#define NIKON_IEP_MAX_CMP_EVENT                 (0x10U) /* Maximum CMP event number (0-15) */

/* IEP register offsets for periodic trigger mode */
#define NIKON_CFG_REG_SIZE                      (4U)
#define NIKON_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0   (CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG + 2U*NIKON_CFG_REG_SIZE)
#define NIKON_8_BYTE_REG_OFFSET                 (8U)

/* 1000usec sleep for timeout count*/
#define NIKON_1MILLISEC_SLEEP_TIME              (1000U)
/* 300 usec delay for EEPROM read operation */
#define NIKON_EEPROM_READ_WAIT_US               (300U)
/* 30milli sec delay for EEPROM write operation */
#define NIKON_EEPROM_WRITE_WAIT_US              (30000U)

/* Middle bit indexes for Given Oversampling rates */
#define NIKON_FIFO_BIT_IDX_8X_OS                (4)     /* 8x Oversampling */
#define NIKON_FIFO_BIT_IDX_6X_OS                (3)     /* 6x Oversampling */
#define NIKON_FIFO_BIT_IDX_5X_OS                (2)     /* 5x Oversampling */
#define NIKON_FIFO_BIT_IDX_4X_OS                (2)     /* 4x Oversampling */

#define NIKON_BASE_VALID_BIT_IDX                (24)    /* Base valid bit index */

/* Lengths of the Specified fields */
#define NIKON_DB_BITS_LEN                       (10)    /* Temperature bits */
#define NIKON_RX_ONE_FRAME_LEN                  (16)    /* Rx frame */
#define NIKON_EEPROM_ADDR_LEN                   (8)     /* EEPROM memory address */
#define NIKON_EEPROM_BANK_LEN                   (8)     /* EEPROM memory BANK */
#define NIKON_EEPROM_DATA_BYTE_LEN              (8)     /* EEPROM memory Data Byte Length (High/Low) */
#define NIKON_COMMAND_CODE_LEN                  (5)     /* command code */
#define NIKON_ENC_STATUS_LEN                    (4)     /* encoder status field */
#define NIKON_ENC_STATUS_BIT_LEN                (1)     /* encoder status bit field for CMD_21/CMD_22 */
#define NIKON_ENC_ADDR_LEN                      (3)     /* encoder address */
#define NIKON_ENC_ADDR_MAX                      (7)     /* maximum encoder address value (3-bit: 0-7) */
#define NIKON_SYNC_CODE_LEN                     (3)     /* sync code */
#define NIKON_TX_CRC_LEN                        (3)     /* 3bit Tx CRC */
#define NIKON_FRAME_CODE_LEN                    (2)     /* frame code */
#define NIKON_START_BIT_LEN                     (1)     /* start bit */
#define NIKON_STOP_BIT_LEN                      (1)     /* stop bit */
#define NIKON_FIXED_BIT_LEN                     (1)     /* fix bit in info field */
#define NIKON_VEL_LEN                           (32)    /* velocity data length in bits */
#define NIKON_ACC_LEN                           (16)    /* acceleration data length in bits */
#define NIKON_ID_CODE_LEN                       (24)    /* ID code data length in bits */
#define NIKON_VEL_COEFFICIENT_LEN               (19)    /* velocity coefficient data length in bits */
#define NIKON_CMD_21_22_IF_DATA_LEN             (9)     /* Data bits in IF for CMD_21/CMD_22 */

/* Status for request or access */
#define NIKON_EEPROM_READ_ACCESS                (1)     /* eeprom read access */
#define NIKON_EEPROM_WRITE_ACCESS               (2)     /* eeprom write access */
#define NIKON_ENABLE_ID_CODE_WRITE              (2)     /* ID code write request */

/* Maximum possible parameters */
#define NIKON_MAX_NUM_RX_FRAMES                 (6)     /* Rx frames */
#define NIKON_MAX_ABS_LEN                       (40)    /* ABS data length */
#define NIKON_MAX_NUM_DATA_FIELDS               (3)     /* Rx Data fields */
/* Common parameters for most of the commands */
#define NIKON_AVG_NUM_RX_FRAMES                 (3)     /* Rx frames */
#define NIKON_AVG_ABS_LEN                       (24)    /* ABS data length */
/*Minimum possible parameters */
#define NIKON_MIN_NUM_RX_FRAMES                 (2)     /* Rx frames */
#define NIKON_MIN_ABS_LEN                       (17)    /* ABS data length */

/* Pre-calculated bit masks */
#define NIKON_MASK_40BIT                        ((1ULL << NIKON_MAX_ABS_LEN) - 1)  /* 0xFFFFFFFFFF - mask for 40-bit data */
#define NIKON_MASK_24BIT                        ((1ULL << NIKON_AVG_ABS_LEN) - 1)  /* 0xFFFFFF - mask for 24-bit data */
#define NIKON_MASK_17BIT                        ((1ULL << NIKON_MIN_ABS_LEN) - 1)  /* 0x1FFFF - mask for 17-bit data */

#define NIKON_NUM_RX_FRAMES_TWO                 (2U)
#define NIKON_NUM_RX_FRAMES_THREE               (3U)
#define NIKON_NUM_RX_FRAMES_FOUR                (4U)
#define NIKON_NUM_RX_FRAMES_FIVE                (5U)
#define NIKON_NUM_RX_FRAMES_SIX                 (6U)

/* Masks for specified fields */
#define NIKON_DB_BITS_MASK                      (0x3FF) /* temperature bits(DB) */
#define NIKON_ENC_STATUS_MASK                   (0xF)   /* encoder status field */
#define NIKON_ENC_STATUS_BIT_MASK               (0x1)   /* encoder status field for CMD_21/CMD_22*/
#define NIKON_CMD_CODE_MASK                     (0x1F)  /* command code field*/
#define NIKON_ENC_ADDR_MASK                     (0x7)   /* encoder address field */

/* Nikon protocol versions */
#define NIKON_PROTOCOL_V2_1                     (0)
#define NIKON_PROTOCOL_V3_0                     (1)

/* Frame codes for Nikon 3.0 memory operations */
#define NIKON_FRAME_CODE_NO_BANK                (0x0)
#define NIKON_FRAME_CODE_BANK                   (0x3)

/* Number of cycles required to perform operation or reset specific commands */
#define NIKON_NUM_OF_CYCLE_FOR_RESET            (7)

#define NIKON_MEM_DATA_LOW_INDEX                (0U)
#define NIKON_MEM_DATA_HIGH_INDEX               (1U)
#define NIKON_MEM_ADDRESS_INDEX                 (2U)
#define NIKON_MEM_BANK_INDEX                    (3U)

/* Macro to define multiplication factor to convert MHz into Hz*/
#define MHZ_TO_HZ                               (1000000)

/** \brief Temperature scale factor */
#define NIKON_TEMPERATURE_SCALE_FACTOR          (0.25f)

/** \brief EEPROM address containing temperature data */
#define NIKON_EEPROM_TEMP_ADDRESS               (0xF9U)

/** \brief Shift to extract 10-bit temperature from 16-bit data field */
#define NIKON_TEMP_DATA_SHIFT                   (6U)

/** \brief Degrees per full revolution for angle calculation */
#define NIKON_DEGREES_PER_REVOLUTION            (360.0f)

/** \brief Byte shift amount (8 bits) */
#define NIKON_BYTE_SHIFT                        (8U)

/** \brief Stop bit value in Nikon frame */
#define NIKON_STOP_BIT_VALUE                    (0x1U)

/** \brief Frame code for memory address MDF */
#define NIKON_FRAME_CODE_MEMORY_ADDR            (3U)

/** \brief Default sync code for RX (010 binary) */
#define NIKON_DEFAULT_SYNC_CODE                 (2U)

/** \brief Default frame code for CDF (00 binary) */
#define NIKON_DEFAULT_FRAME_CODE                (0U)

/* ========================================================================== */
/*                       Default Parameter Values                             */
/* ========================================================================== */

/** \brief Default command processing delay in micro-seconds (1 ms) */
#define NIKON_DEFAULT_CMD_PROCESS_DELAY_US      (1000U)

/** \brief Default firmware wait delay in micro-seconds (1 ms) */
#define NIKON_DEFAULT_FW_WAIT_DELAY_US          (1000U)

/** \brief Default maximum wait loop count for Nikon cycle timeout
 *
 *  Minimum and Maximum NIKON cycle time depends on various params as below:
 *  TCycle_max = TMA * (number of RX frames * 18) + TX frame(32)
 *  + delay between TX and RX + (delay between Multi Transmission commands
 *  * (maximum encoder address delay t6))
 *  + Delay between two EEPROM access commands(30milli-seconds)
 *  TCycle_min = TMA * (number of Rx frames * 18) + TX frame(32)
 *  + delay between TX and RX
 *  Instead wait for max of 35 ms as this can vary for different encoders,
 *  different commands and multi transmission connection.
 *
 *  Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
 *  Default value is 35, which with default cmd_process_delay_us of 1000us results in 35ms timeout.
 *
 *  Different value can be configured by changing max_wait_loop_count of \ref nikon_params before calling \ref nikon_init.
 *  Value must be greater than 0.
 */
#define NIKON_DEFAULT_MAX_WAIT_LOOP_COUNT       (35U)

/** \brief Multi-transmission delay multiplier for 2.5 MHz */
#define NIKON_MT_DELAY_2_5MHZ_MULTIPLIER        (3U)

/** \brief Multi-transmission delay multiplier for 4 MHz */
#define NIKON_MT_DELAY_4MHZ_MULTIPLIER          (2U)

/** \brief Multi-transmission delay multiplier for 8/16 MHz */
#define NIKON_MT_DELAY_8_16MHZ_MULTIPLIER       (1.5f)

/** \brief Multi-transmission delay multiplier for 6.67 MHz */
#define NIKON_MT_DELAY_6_67MHZ_MULTIPLIER       (2U)

/** \brief Clock fractional divider factor (value = 1.5)
 *
 *  Fractional divider applied in Nikon clock calculations for
 *  achieving precise baud rates with oversampling.
 */
#define NIKON_CLOCK_FRACTIONAL_DIVIDER          (1.5f)

/** \brief Velocity coefficient third byte mask (bits 16-18)
 *
 *  Used to extract lower 3 bits from the third byte when processing
 *  19-bit velocity coefficient data (bits 0-18).
 */
#define NIKON_VEL_COEFF_THIRD_BYTE_MASK         (0x070000U)

/**
 *  \brief    Nikon command codes [4:0]
 *
 *  \details  Command code enum for various Nikon encoder operations.
 *            Values 0-30 are standard command codes sent to encoder.
 *            Values 31+ are internal driver control commands.
 */
typedef enum nikon_cmd_e
{
    CMD_0 = 0,                      /**< ABS full 40bit data request */
    CMD_1,                          /**< ABS lower 24bit data request */
    CMD_2,                          /**< ABS upper 24bit data request */
    CMD_3,                          /**< Encoder status request */
    CMD_4,                          /**< ABS full 40bit data request (MT) */
    CMD_5,                          /**< ABS lower 24bit data request (MT) */
    CMD_6,                          /**< ABS upper 24bit data request (MT) */
    CMD_7,                          /**< Encoder status request (MT) */
    CMD_8,                          /**< Status flag clear request */
    CMD_9,                          /**< Multiple turn data clear request */
    CMD_10,                         /**< Status + Multiple turn data clear request */
    CMD_11,                         /**< Encoder address setting I (one-to-one) */
    CMD_12,                         /**< Single turn data zero preset */
    CMD_13,                         /**< EEPROM read request */
    CMD_14,                         /**< EEPROM write request */
    CMD_15,                         /**< Temperature data (8bit) request */
    CMD_16,                         /**< Identification code read I */
    CMD_17,                         /**< Identification code read II (one-to-one) */
    CMD_18,                         /**< Identification code write I */
    CMD_19,                         /**< Identification code write II (one-to-one) */
    CMD_20,                         /**< Encoder address setting II */
    CMD_21,                         /**< ABS lower 17bit data request */
    CMD_22,                         /**< ABS lower 17bit data request (MT) */
    CMD_23,                         /**< ABS lower 24bit + velocity request (Individual) (Nikon 3.0 only) */
    CMD_24,                         /**< ABS lower 24bit + velocity request (Multiple) (Nikon 3.0 only) */
    CMD_25,                         /**< ABS lower 24bit + velocity + acceleration request (Individual) (Nikon 3.0 only) */
    CMD_26,                         /**< ABS lower 24bit + velocity + acceleration request (Multiple) (Nikon 3.0 only) */
    CMD_27,                         /**< ABS lower 24bit data + Status request */
    CMD_28,                         /**< ABS lower 24bit data + Status request (MT) */
    CMD_29,                         /**< ABS lower 24bit data + Temperature data request */
    CMD_30,                         /**< ABS lower 24bit data + Temperature data request (MT) */
    ENCODER_ADR_CHANGE = 31,        /**< Update Encoder address(EAX) in APP local context */
    START_CONTINUOUS_CMP_MODE,      /**< Start periodic trigger CMP mode */
    START_CONTINUOUS_CAP_MODE,      /**< Start periodic trigger CAP mode */
    UPDATE_CLOCK_FREQ,              /**< Update operating baud rate as specified by user */
    UPDATE_ENC_LEN,                 /**< Update encoder's single turn and multi turn resolution */
    CMD_1_VEL,                      /**< ABS full 40bit data + velocity data request (Nikon 3.0 only) */
    CMD_5_VEL,                      /**< ABS full 40bit data + velocity data request (MT) (Nikon 3.0 only) */
    CMD_8_POS,                      /**< ABS lower 24bit data request (Nikon 3.0 only) */
    CMD_9_POS,                      /**< ABS lower 24bit data request (Nikon 3.0 only) */
    CMD_10_POS,                     /**< ABS lower 24bit data request (Nikon 3.0 only) */
    CMD_11_POS,                     /**< ABS lower 24bit data request (Nikon 3.0 only) */
    CMD_12_POS,                     /**< ABS lower 24bit data request (Nikon 3.0 only) */
    CMD_13_BANK,                    /**< EEPROM read request with bank (Nikon 3.0 only) */
    CMD_14_BANK,                    /**< EEPROM write request with bank (Nikon 3.0 only) */
    CMD_16_VEL,                     /**< Velocity coefficient read (Nikon 3.0 only) */
    CMD_18_VEL,                     /**< Velocity coefficient write (Nikon 3.0 only) */
    CMD_CODE_NUM                    /**< Total number of command codes */
} nikon_cmd;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *    \brief    Structure defining 3-ch peripheral interface clock configuration
 *
 *    \details  Contains clock divisors and configuration for Nikon communication at the
 *              selected frequency. Calculated by \ref nikon_calc_clock based on the
 *              target baud rate and available clock source (Core or UART clock).
 *
 */
typedef struct nikon_clk_cfg_s
{
    uint16_t  rx_div;
    /**< Rx clock divisor (value-1 written to register). Determines receive sample rate.
     *   Formula: rx_clk = source_clk / ((rx_div + 1) * rx_oversampling)
     *   Example: For 8x oversampling at 4 MHz from 200 MHz core: rx_div = (200/(4*8))-1 = 5 */

    uint16_t  tx_div;
    /**< Tx clock divisor (value-1 written to register). Determines transmit bit rate.
     *   Formula: tx_clk = source_clk / (tx_div + 1)
     *   Example: For 4 MHz from 200 MHz core: tx_div = (200/4)-1 = 49 */

    uint16_t  rx_div_attr;
    /**< Rx oversampling rate, start bit polarity and fractional divider configuration.
     *   Bits [2:0] : Oversampling divisor (7 = 8x, 5 = 6x, 3 = 4x)
     *   Bits [3]   : Start bit polarity (0 or 1)
     *   Bit  [15]  : Fractional divider enable (1=enable 1.5x fractional division) */

    uint16_t  is_core_clk;
    /**< Clock source selection flag.
     *   0 = Use UART clock as source (160/192 MHz)
     *   1 = Use Core clock as source (200/300 MHz) */
} nikon_clk_cfg;

typedef struct nikon_position_info_s
{
    uint32_t raw_data0[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - IF0 */
    uint32_t raw_data1[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF0 */
    uint32_t raw_data2[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF1 */
    uint32_t raw_data3[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF2 */
    uint32_t raw_data4[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF3 */
    uint32_t raw_data5[NUM_ENCODERS_MAX];
    /**< Raw data receive from encoder - DF4 */
    uint32_t rcv_crc[NUM_ENCODERS_MAX];
    /**< 8-bit receive position sense crc */
    uint32_t otf_crc[NUM_ENCODERS_MAX];
    /**< 8-bit calculated otf crc*/
    uint32_t crc_err_cnt[NUM_ENCODERS_MAX];
    /**< Position data crc error count */
    uint64_t abs[NUM_ENCODERS_MAX];
    /**< Absolute data(position data) received from the encoder */
    uint32_t multi_turn[NUM_ENCODERS_MAX];
    /**< Total number of complete rotations(360) */
    float angle[NUM_ENCODERS_MAX];
    /**< Angle of encoder shaft */
    int32_t velocity[NUM_ENCODERS_MAX];
    /**< Velocity data (signed, Nikon 3.0 only).
     *   32-bit signed integer velocity value received from the encoder */
    int16_t acc[NUM_ENCODERS_MAX];
    /**< Acceleration data (signed, Nikon 3.0 only).
     *   16-bit signed integer acceleration value received from the encoder */
} nikon_position_info;

typedef struct nikon_encoder_info_s
{
    uint32_t enc_status[NUM_ENCODERS_MAX];
    /**< Encoder status*/
    uint32_t enc_cmd[NUM_ENCODERS_MAX];
    /**< Command acknowledge by encoder*/
    uint32_t enc_addr[NUM_ENCODERS_MAX];
    /**<Encoder address acknowledged by the encoder*/
} nikon_encoder_info;

/**
 *    \brief    Structure defining Predictive Maintenance Alarm bits received by the encoder (Nikon 3.0 only)
 *
 *    \details  Alarm for incremental signal and LED forward current value
 *
 */
typedef struct nikon_pm_alarm_bits_s
{
    uint8_t incw_1;
    /**<  Alarm occurs when deterioration is observed in incremental signal of sensor unit 1*/
    uint8_t incw_2;
    /**<  Alarm occurs when deterioration is observed in incremental signal of sensor unit 2*/
    uint8_t ifw_1;
    /**<  Alarm occurs when LED deterioration is observed based on forward current value in sensor unit 1*/
    uint8_t ifw_2;
    /**<  Alarm occurs when LED deterioration is observed based on forward current value in sensor unit 2*/
} nikon_pm_alarm_bits;

/**
 *    \brief    Structure defining Alarm bits received by the encoder
 *
 *    \details  Alarm for battery voltage beyond a specific band, over flow,
 *              over speed, over temperature, Memory , single turn
 *              and multi turn errors and busy flags.
 *
 */
typedef struct nikon_alarm_bits_s
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
    /**< To monitor the conformance between the "ABS block"
     *   and the "INC block".*/
    uint8_t ps_err;
    /**< To monitor the conformance between (1)"multi turn calculation block"
     * and (2)"single turn calculation block." When comparing (1) and (2), if
     * (difference between (1) and (2)) ≧ (single turn) an alarm is turned on*/
    uint8_t busy;
    /**< This flag is turned on during the process
     *  to determine a single turn absolute value*/
    uint8_t mem_busy;
    /**< This flag shows that access to the EEPROM in the encoder is under way.
     * After the access is completed, the flag returns to "0."*/
    uint8_t ov_temp;
    /**< It issues warning when the temperature sensor's output
     * on board becomes beyond the specified value*/
    uint8_t inc_err_m;
    /**< When a signal failure (amplitude, level, etc.) in the incremental
     * signal phase A/ phase B is detected, this flag outputs '1.'*/
    uint8_t ov_spd_s;
    /* Over speed (Nikon 3.0) */
    uint8_t st_err_s;
    /* Single turn error (Nikon 3.0) */
    uint8_t ps_err_s;
    /* Position error (Nikon 3.0) */
    uint8_t busy_s;
    /* Busy (Nikon 3.0) */
    uint8_t inc_err_s;
    /* Increment error (Nikon 3.0) */
} nikon_alarm_bits;

/**
 *    \brief    Structure defining Nikon initialization parameters
 *
 *    \details  Parameters passed to \ref nikon_init to initialize a Nikon instance.
 *              Use \ref nikon_params_init to populate with default values.
 *
 */
typedef struct nikon_params_s
{
    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS Handle obtained from PRUICSS_open() */

    uint32_t cmd_process_delay_us;
    /**< Delay in micro-seconds for command processing polling loop.
     *   Used in \ref nikon_command_wait to avoid excessive CPU usage.
     *   Default: 1000 us (1 ms) */

    uint32_t fw_wait_delay_us;
    /**< Delay in micro-seconds between firmware status checks.
     *   Default: 1000 us (1 ms) */

    uint32_t max_wait_loop_count;
    /**< Maximum wait loop count for Nikon cycle timeout detection.
     *   Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
     *   Used in \ref nikon_command_wait to detect communication failures.
     *   Must be greater than 0.
     *   Default: 35 (with default cmd_process_delay_us of 1000us results in 35ms timeout) */

} nikon_params;

/**
 *    \brief    Structure defining Nikon compile-time attributes (from SysConfig)
 *
 *    \details  Contains configuration parameters that are determined at compile-time
 *              via SysConfig or static configuration. These values do not change
 *              during runtime.
 */
typedef struct nikon_attrs_s
{
    uint8_t instance;
    /**< Nikon instance index (0, 1, ...) for multi-instance configurations */

    uint8_t mode;
    /**< Nikon channel configuration mode:
     *   0 = NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU
     *   1 = NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU
     *   2 = NIKON_MODE_MULTI_CHANNEL_MULTI_PRU (load share) */

    uint8_t pruicss_instance;
    /**< PRU-ICSS hardware instance number (0 or 1) */

    uint8_t pruicss_slice;
    /**< PRU-ICSS slice selection (0 or 1) */

    uint8_t load_share_enabled;
    /**< Load share mode enable flag (0 = disabled, 1 = enabled) */

    uint8_t channel_mask;
    /**< Bit mask indicating enabled channels (0x1=ch0, 0x2=ch1, 0x4=ch2) */

    uint8_t channel0_enabled;
    /**< Channel 0 enable flag (0 or 1) */

    uint8_t channel1_enabled;
    /**< Channel 1 enable flag (0 or 1) */

    uint8_t channel2_enabled;
    /**< Channel 2 enable flag (0 or 1) */

    uint8_t total_channels;
    /**< Total number of enabled channels (1-3) */

    uint16_t baud_rate;
    /**< Nikon communication frequency in MHz (2.5, 4, 6.67, 8, 16) */

    uint32_t core_clk_freq;
    /**< PRU-ICSS Core Clock frequency in Hz */

    uint32_t uart_clk_freq;
    /**< PRU-ICSS UART Clock frequency in Hz */

    uint32_t iep_clk_freq;
    /**< PRU-ICSS IEP timer clock frequency in Hz */

    uint16_t is_core_clk;
    /**< Clock source selection (0 = UART clock, 1 = Core clock) */

    uint8_t protocol_version;
    /**< Nikon protocol version (NIKON_PROTOCOL_V2_1 or NIKON_PROTOCOL_V3_0) */

    uint8_t iep_instance;
    /**< IEP instance (0 or 1) for periodic trigger mode */

    uint8_t iep_cmp_event[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< IEP CMP event numbers for periodic CMP trigger mode (per channel) */

    uint8_t iep_cap_event[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< IEP CAP event numbers for periodic CAP trigger mode (per channel) */

    void *iep_base_addr;
    /**< IEP register base address for periodic trigger mode */

} nikon_attrs;

/**
 *    \brief    Nikon private data structure (runtime state and configuration)
 *
 *    \details  Contains runtime state information including encoder parameters, position/control
 *              data results, CRC error counts, and pointers to PRU-ICSS shared
 *              memory (nikon_pruicss_xchg). This structure is initialized during \ref nikon_init
 *              and should be accessed via \ref nikon_get_priv API.
 *
 */
typedef struct nikon_priv_s
{
    uint8_t is_open;
    /**< Initialization state flag (0 = closed, 1 = open) */

    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS handle from params */

    uint32_t cmd_process_delay_us;
    /**< Command processing delay from params */

    uint32_t fw_wait_delay_us;
    /**< Firmware wait delay from params */

    uint32_t max_wait_loop_count;
    /**< Maximum wait loop count for Nikon cycle timeout detection.
     *   Actual timeout (ms) = max_wait_loop_count * cmd_process_delay_us / 1000
     *   Used in \ref nikon_command_wait to detect communication failures.
     *   Must be greater than 0.
     *   Copied from params in \ref nikon_init. */

    uint32_t data_len[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< Resolution of encoder */

    uint32_t num_encoders[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Number of encoders connected in bus to each PRU in load share */

    uint32_t num_enc_access[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Number of encoders to access based on provided command code */

    uint32_t single_turn_len[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< Single turn resolution */

    uint32_t multi_turn_len[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< Multi turn resolution */

    uint32_t channel[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Array of all configured channels */

    nikon_pruicss_xchg *pruicss_xchg;
    /**< Structure defining NIKON interface */

    uint32_t tx_cdf[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Command data frame to be transmitted to encoder */

    uint32_t tx_mdf[NIKON_NUM_CH_PER_SLICE_MAX][NUM_MDF_MAX];
    /**< Memory data frame to be transmitted to encoder */

    uint32_t num_rx_frames;
    /**< Number of Rx frames to be receive */

    float_t baud_rate;
    /**< Input baudrate */

    uint32_t eax[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Encoder address from the user */

    uint32_t fc;
    /**< Frame code specified by the user */

    uint32_t sync_code;
    /**< Synchronization code */

    uint32_t tx_crc;
    /**< Tx 3 bit crc */

    uint32_t mem_data[NIKON_NUM_CH_PER_SLICE_MAX][NUM_MDF_MAX];
    /**< Memory data in indexes 0, 1 and memory address in index 2 */

    nikon_position_info pos_data_info[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< ABS, ALM, EEPROM or Identification code information extracted
     *   from the data receive */

    nikon_encoder_info enc_info[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Encoder's information(Encoder address, Encoder status and
     *   command given to the encoder) extracted from the data receive */

    uint32_t temperature[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< Temperature */

    uint32_t identification_code[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Identification code of the current encoder (ID bits are stored in lower 24 bits of this 32 bit variable) */

    uint32_t velocity_coefficient[NIKON_NUM_CH_PER_SLICE_MAX];
    /**< Velocity Coefficient of the current encoder (Velocity coefficient bits are stored in lower 19 bits of this 32 bit variable) */

    uint32_t alm_field[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< ALM field received from the encoder */

    uint32_t pm_alm_field[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< PM ALM field received from the encoder */

    nikon_alarm_bits alm_bits[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< ALM bits received from the encoder */

    nikon_pm_alarm_bits pm_alm_bits[NIKON_NUM_CH_PER_SLICE_MAX][NUM_ENCODERS_MAX];
    /**< PM ALM bits received from the encoder */

    uint32_t abs_len;
    /**< Length of absolute data received from encoder */

    uint32_t is_continuous_mode;
    /**< Flag for continuous mode triggered */

    uint8_t bank_error;
    /**< Incorrect bank error indication in response (Nikon 3.0 only) */

} nikon_priv;

/**
 *    \brief    Nikon configuration structure (internal use)
 *
 *    \details  Combines pointers to runtime state (priv) and compile-time attributes (attrs).
 *              Used internally by the driver. Applications use the opaque \ref nikon_handle.
 */
typedef struct nikon_config_s
{
    nikon_priv *priv;
    /**< Pointer to private runtime state */

    const nikon_attrs *attrs;
    /**< Pointer to compile-time attributes */

} nikon_config;

/**
 *    \brief    Nikon handle type (opaque pointer)
 *
 *    \details  Opaque handle returned by \ref nikon_init and used by all Nikon APIs.
 *              Provides access to both runtime state (priv) and compile-time config (attrs).
 */
typedef nikon_config *nikon_handle;

#ifdef __cplusplus
}
#endif

#endif
