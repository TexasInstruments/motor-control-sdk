/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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

#ifndef ENDAT3_DRV_H_
#define ENDAT3_DRV_H_

/**
 *  \defgroup ENDAT3_API_MODULE APIs for EnDAT3 Encoder
 *  \ingroup POSITION_SENSE_API
 *
 *  The EnDAT3 API module provides functions for initializing and communicating with
 *  EnDAT3-compatible encoders. It supports both host-triggered and periodic communication,
 *  and provides interfaces for command transmission and response parsing.
 *
 *  \section endat3_param_validation Parameter Validation Strategy
 *
 *  The EnDAT3 driver implements following parameter validation approach:
 *
 *  - **Initialization-time validation**: During \ref endat3_init, all configuration
 *    parameters are thoroughly validated including: handle, attrs (configuration structure),
 *    params (including pruicss_handle), and all their member fields. This ensures the
 *    driver is properly initialized with valid configuration before any operations begin.
 *
 *  - **Runtime validation**: All public APIs validate the handle parameter and internal
 *    structure pointers (priv, endat3_interface, attrs, pruicss_handle) for NULL before
 *    dereferencing to prevent undefined behavior.
 *
 *  - **Array bounds checking**: All array accesses include explicit bounds checking to
 *    prevent buffer overruns. Array indices are validated before use, and array sizes
 *    are checked against defined limits.
 *
 *  \section endat3_usage_flow Typical Usage Flow
 *
 *  The following sequence demonstrates typical usage of the EnDAT3 driver:
 *
 *  **1. Initialization:**
 *  - Call \ref endat3_params_init() to initialize parameters structure
 *  - Configure params.pruicss_handle with PRU-ICSS instance from PRUICSS_open()
 *  - Call \ref endat3_init() to initialize the driver and hardware
 *
 *  **2. Operating Mode Configuration:**
 *  - Default operating mode is HOST_TRIGGER
 *  - Can change operating mode with \ref endat3_set_operating_mode() (HOST_TRIGGER or PERIODIC)
 *  - For periodic mode, trigger is done automatically by PRU firmware at configured intervals.
 *
 *  **3. Command Execution (Host Trigger Mode):**
 *  - Call \ref endat3_set_expected_tx_frame_count() to specify number of frames
 *  - Call \ref endat3_send_command() to prepare command (e.g., ENDAT3_REQ_DATA0)
 *  - Call \ref endat3_release_start_trigger() to initiate communication
 *  - Wait for encoder processing (use \ref endat3_is_busy() to poll status)
 *  - Call \ref endat3_receive_response() to fetch and validate response
 *  - Check return value == ENDAT3_SUCCESS before reading data
 *
 *  **4. Data Retrieval:**
 *  - Use \ref endat3_get_hpf_status() to check High Priority Frame status
 *  - Use \ref endat3_get_hpf_data() to retrieve position/encoder data
 *  - Use \ref endat3_is_hpf_data_valid() to verify data validity
 *  - Optionally use \ref endat3_get_lph_status(), \ref endat3_get_lpf_data() for low priority data
 *
 *  **5. Error Handling:**
 *  - Most APIs return ENDAT3_SUCCESS (0) on success or negative error codes on failure (ENDAT3_ERR_*).
 *  - Few APIs do not return ENDAT3_SUCCESS or ENDAT3_ERR_* codes. Refer to API documentation for
 *    specific details.
 *
 *  **6. Cleanup:**
 *  - Call \ref endat3_deinit() when finished to release resources
 *  \{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <drivers/pruicss.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                               */
/* ========================================================================== */

/**
 * \name EnDat3 PRU Operating Modes
 * \{
 */
/** \brief Single channel, single PRU mode (non-load-share) */
#define ENDAT3_MODE_SINGLE_CHANNEL_SINGLE_PRU   (0U)
/** \} */

/**
 *  \brief  EnDat3 Operation Mode: Periodic trigger with CMP event
 *
 *  In periodic CMP mode, the PRU firmware automatically triggers position readout
 *  at regular intervals using IEP CMP (Compare) events.
 */
#define ENDAT3_OPMODE_PERIODIC_CMP              (0x0U)

/**
 *  \brief  EnDat3 Operation Mode: Host trigger
 *
 *  In host trigger mode, the R5F host processor explicitly triggers
 *  each position readout by setting the trigger bit.
 */
#define ENDAT3_OPMODE_HOST_TRIGGER              (0x1U)

/**
 *  \brief  EnDat3 Operation Mode: Periodic trigger with CAP event
 *
 *  In periodic CAP mode, the PRU firmware automatically triggers position readout
 *  at regular intervals using IEP CAP (Capture) events.
 */
#define ENDAT3_OPMODE_PERIODIC_CAP              (0x2U)

/* EnDAT3 Driver Return Codes
 * Values for APIs to return ENDAT3_SUCCESS (0) on success
 * or detailed ENDAT3_ERR_* codes on failure.
 * Error codes range from ENDAT3_ERR_INVALID_INPUT (-1) to ENDAT3_ERR_INVALID_OPMODE (-8).
 */
#define ENDAT3_SUCCESS                          (0)     /**< Success code */
#define ENDAT3_ERR_INVALID_INPUT                (-1)    /**< Invalid input parameter (NULL pointer, invalid value, out of bounds) */
#define ENDAT3_ERR_BUSY                         (-2)    /**< Encoder busy (transfer in progress) */
#define ENDAT3_ERR_RX_FAIL                      (-3)    /**< Reception failure */
#define ENDAT3_ERR_LPH_CRC_FAIL                 (-4)    /**< LPH (Low Priority Header) CRC mismatch */
#define ENDAT3_ERR_HPF_CRC_FAIL                 (-5)    /**< HPF (High Priority Frame) CRC mismatch */
#define ENDAT3_ERR_LPF_CRC_FAIL                 (-6)    /**< LPF (Low Priority Frame) CRC mismatch */
#define ENDAT3_ERR_SAMPLING_ERROR               (-7)    /**< Encoder sampling/timing error */
#define ENDAT3_ERR_INVALID_OPMODE               (-8)    /**< Invalid operating mode (mode > 2) */

/* Encoder busy/error status defines */
#define ENCODER_IDLE                            (0x0)  /**< Encoder idle, ready for communication */
#define ENCODER_BUSY                            (0x1)  /**< Encoder busy, transfer in progress */
#define ENCODER_ERROR                           (0x2)  /**< Encoder error detected during transfer */

/* Frame processing constants */
#define ENDAT3_FRAME_SIZE_BYTES                 (4U)    /**< EnDAT3 frame size in bytes */
#define ENDAT3_CHANNEL_MASK_MAX                 (0x7U)  /**< Maximum valid channel mask (ch0|ch1|ch2) */
#define ENDAT3_MAX_BG_CMD_INDEX                 (3U)    /**< Exclusive upper bound for background command index (valid range: 0-2) */

/* Core clock frequency reference values (MHz) */
#define ENDAT3_CORE_CLK_200MHZ                  (200000000U)  /**< 200 MHz core clock frequency */
#define ENDAT3_CORE_CLK_300MHZ                  (300000000U)  /**< 300 MHz core clock frequency */

/** \brief Reference frequency for delay calculations (1 MHz) */
#define REFERENCE_PRU_FREQ_HZ                   (1000000ULL)
/** \brief 5 MHz reference for sampling delay (gives 60 cycles @ 300MHz, 40 cycles @ 200MHz) */
#define REFERENCE_PRU_FREQ_HZ_FOR_SAMPLING      (5000000ULL)

/** These values are derived from the original 300 MHz PRU frequency implementation
 *  and are scaled to actual PRU frequency at runtime using the formula:
 *  actual_cycles = (reference_cycles * actual_freq) / REFERENCE_PRU_FREQ_HZ
 *  \{
 */
#define ENDAT3_TX_START_DELAY_1_REF_CYCLES      (218U)          /**< TX start delay 1 reference (65535 / 300) */
#define ENDAT3_TX_START_DELAY_2_REF_CYCLES      (1748U)         /**< TX start delay 2 reference (524288 / 300) */
#define ENDAT3_TX_START_DELAY_3_REF_CYCLES      (438U)          /**< TX start delay 3 reference (131328 / 300) */
#define ENDAT3_DELAY_10MS_REF_CYCLES            (10486U)        /**< 10ms delay reference (3145728 / 300) */
/** \} */

#define ENDAT3_BAUD_RATE_12_5_MBPS              (0U)            /**< 12.5 Mbps mode selector */
#define ENDAT3_BAUD_RATE_25_MBPS                (1U)            /**< 25 Mbps mode selector */

/** Clock configuration constants
 * \{
 */
#define ENDAT3_BAUD_RATE_12_5_MBPS_VALUE        (12500000U)     /**< 12.5 Mbps baud rate value in Hz */
#define ENDAT3_BAUD_RATE_25_MBPS_VALUE          (25000000U)     /**< 25 Mbps baud rate value in Hz */
/** \} */

/** IEP (Industrial Ethernet Peripheral) event limits for periodic trigger mode
 * \{
 */
#define ENDAT3_IEP_MAX_CAP_EVENT                (0x8U)          /**< Maximum IEP CAP events (0-7) */
#define ENDAT3_IEP_MAX_CMP_EVENT                (0x10U)         /**< Maximum IEP CMP events (0-15) */
/** \} */

/** IEP register offsets for periodic trigger mode
 * \{
 */
#define ENDAT3_CFG_REG_SIZE                     (4U)            /**< IEP configuration register size in bytes */
#define ENDAT3_CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0  (CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG + 2U*ENDAT3_CFG_REG_SIZE)  /**< IEP CAP0 register 0 offset */
#define ENDAT3_8_BYTE_REG_OFFSET                (8U)            /**< 8-byte register offset for IEP CMP/CAP registers */
/** \} */

#define ENDAT3_OVERSAMPLE_RATE_8X               (8U)            /**< 8x oversampling rate for RX clock */
#define ENDAT3_SB_POLARITY                      (0U)            /**< Start bit polarity configuration */
#define ENDAT3_TX_CLOCK_MULTIPLIER              (2U)            /**< TX clock multiplier: TX clock = 2x baud rate */

/** Buffer and data size definitions
 * \{
 */
#define HPF_DATA_SIZE                           (6)     /**< High Priority Frame data size in bytes */
#define LPF_DATA_SIZE                           (6)     /**< Low Priority Frame data size in bytes */
#define MAX_LPF_COUNT                           (8)     /**< Maximum number of LPF frames */
#define LUT_SIZE                                (64)    /**< Lookup table size (256 bytes / 4 bytes per uint32_t) */
#define RX_BUFFER_SIZE                          (64)    /**< Receive buffer size */
#define TX_BUFFER_SIZE                          (24)    /**< Transmit buffer size */
#define BG_DATA_SIZE                            (6)     /**< Background data array size */
/** \} */

/* Frame layout definitions for RX buffer */
#define ENDAT3_HPF_TOTAL_SIZE                   (8)     /**< HPF total frame size: 6 data + 1 status + 1 CRC */
#define ENDAT3_LPH_TOTAL_SIZE                   (4)     /**< LPH total frame size: 2 data + 1 status + 1 CRC */
#define ENDAT3_LPF_TOTAL_SIZE                   (8)     /**< LPF total frame size: 6 data + 1 FID/status + 1 CRC */
#define ENDAT3_HPF_OFFSET                       (0)     /**< HPF offset in RX buffer */
#define ENDAT3_LPH_OFFSET                       (8)     /**< LPH offset in RX buffer (after HPF) */
#define ENDAT3_LPF_OFFSET                       (12)    /**< LPF offset in RX buffer (after HPF + LPH) */
#define ENDAT3_MIN_RX_BUFFER_SIZE               (20)    /**< Minimum RX buffer size: HPF(8) + LPH(4) + LPF(8) */

/* Reset command data values */
#define ENDAT3_RESET_HARD                       (0x01)  /**< Hard reset command data */
#define ENDAT3_RESET_SOFT                       (0x00)  /**< Soft reset command data */

/* Clear command flag bits */
#define ENDAT3_CLEAR_F                          (0x01)  /**< Clear F (error) flag */
#define ENDAT3_CLEAR_W                          (0x02)  /**< Clear W (warning) flag */
#define ENDAT3_CLEAR_REF                        (0x04)  /**< Clear reference flag */

/* Data rate configuration values */
#define ENDAT3_RATE_12_5MBPS                    (0x00)  /**< 12.5 Mbps data rate */
#define ENDAT3_RATE_25MBPS                      (0x01)  /**< 25 Mbps data rate */

/* Bus initialization command types */
#define ENDAT3_BUSINIT_RESET_ADDR               (0x01)  /**< Bus init with address reset */
#define ENDAT3_BUSINIT_KEEP_ADDR                (0x00)  /**< Bus init keeping addresses */

/* Protection/Authentication modes */
#define ENDAT3_PROTECT_QUERY                    (0x01)  /**< Query current access levels */
#define ENDAT3_PROTECT_SET_READ                 (0x02)  /**< Set access level for read operations */
#define ENDAT3_PROTECT_SET_WRITE                (0x03)  /**< Set access level for write operations */

/* Password bit masks for splitting 32-bit password */
#define ENDAT3_PASSWORD_HIGH_MASK               (0xFFFF0000)    /**< Upper 16 bits mask */
#define ENDAT3_PASSWORD_LOW_MASK                (0x0000FFFF)    /**< Lower 16 bits mask */
#define ENDAT3_PASSWORD_HIGH_SHIFT              (16)            /**< Shift for upper 16 bits */

#define ENDAT3_PRE_LENGTH                       (25)            /**< Minimum preamble length in halfbits */
#define ENDAT3_POST_LENGTH                      (4)             /**< Postamble length in halfbits */

/**
 * \brief Low Priority Header status values
 *
 * Status values that can appear in the LPH status byte.
 */
typedef enum endat3_lph_status_e {
    LPH_STATUS_IDLE     = 0,  /**< IDLE state */
    LPH_STATUS_RX_START = 1,  /**< RX_START state - beginning of reception */
    LPH_STATUS_RX_LAST  = 2,  /**< RX_LAST state - last reception frame */
    LPH_STATUS_BUSY     = 3   /**< BUSY state - processing in progress */
} endat3_lph_status;

/**
 * \brief EnDAT3 Background Request (BGREQ) OpCodes
 *
 * Operation codes for background processing operations.
 */
typedef enum endat3_bg_req_opcode_e {
    ENDAT3_BGREQ_NOP          = 0x01,  /**< No operation */
    ENDAT3_BGREQ_READ         = 0x02,  /**< Read from encoder memory */
    ENDAT3_BGREQ_WRITE        = 0x03,  /**< Write to encoder memory */
    ENDAT3_BGREQ_RECONFIGURE  = 0x04,  /**< Reconfigure parameters */
    ENDAT3_BGREQ_AUTH         = 0x80,  /**< Authentication */
    ENDAT3_BGREQ_PROTECT      = 0x81,  /**< Set protection */
    ENDAT3_BGREQ_RESERVED     = 0x82,  /**< Reserved for future use */
    ENDAT3_BGREQ_SETPASS      = 0x83,  /**< Set password */
    ENDAT3_BGREQ_LOCATE       = 0x84   /**< Locate function */
} endat3_bg_req_opcode;

/**
 * \brief EnDAT3 Foreground Request (REQ.CODE) OpCodes
 *
 * Operation codes for foreground request commands.
 */
typedef enum endat3_req_code_e {
    ENDAT3_REQ_DATA0      = 0x00,  /**< Activate LPF send list 0 */
    ENDAT3_REQ_DATA1      = 0x01,  /**< Activate LPF send list 1 */
    ENDAT3_REQ_DATA2      = 0x02,  /**< Activate LPF send list 2 */
    ENDAT3_REQ_DATA3      = 0x03,  /**< Activate LPF send list 3 */
    ENDAT3_REQ_DATA4      = 0x04,  /**< Activate LPF send list 4 */
    ENDAT3_REQ_DATA5      = 0x05,  /**< Activate LPF send list 5 */
    ENDAT3_REQ_DATA6      = 0x06,  /**< Activate LPF send list 6 */
    ENDAT3_REQ_DATA7      = 0x07,  /**< Activate LPF send list 7 */
    ENDAT3_REQ_DATA       = 0x08,  /**< Data with BGD */
    ENDAT3_REQ_DATANOP    = 0x09,  /**< Data without BGD */
    ENDAT3_REQ_RESET      = 0x0B,  /**< Encoder reset */
    ENDAT3_REQ_CLEAR      = 0x0C,  /**< Resetting of states */
    ENDAT3_REQ_ECHO       = 0x0E,  /**< Echo for measuring propagation time */
    ENDAT3_REQ_RATE       = 0x10,  /**< Set data transfer rate */
    ENDAT3_REQ_HELLO      = 0x22,  /**< Switch to EnDat 3 mode */
    ENDAT3_REQ_RESERVED   = 0x40,  /**< Reserved */
    ENDAT3_REQ_BUSBC      = 0x80,  /**< Bus command for broadcast */
    ENDAT3_REQ_BUSP2P     = 0x81,  /**< Bus command for point-to-point communication */
    ENDAT3_REQ_BUSINIT    = 0x82,  /**< Initialization of a bus setup */
    ENDAT3_REQ_FORCE      = 0x90   /**< Forced dynamic sampling */
} endat3_req_code;

/**
 * \brief EnDAT3 Menu Options
 *
 * Menu options for user interface commands.
 */
typedef enum endat3_menu_option_e {
    ENDAT3_MENU_DATA0 = 1,    /**< Start with 1 */
    ENDAT3_MENU_DATA1,        /**< DATA1 command */
    ENDAT3_MENU_DATA2,        /**< DATA2 command */
    ENDAT3_MENU_DATA3,        /**< DATA3 command */
    ENDAT3_MENU_DATA4,        /**< DATA4 command */
    ENDAT3_MENU_DATA5,        /**< DATA5 command */
    ENDAT3_MENU_DATA6,        /**< DATA6 command */
    ENDAT3_MENU_DATA7,        /**< DATA7 command */
    ENDAT3_MENU_DATA,         /**< DATA command */
    ENDAT3_MENU_DATANOP,      /**< DATANOP command */
    ENDAT3_MENU_RESET,        /**< RESET command */
    ENDAT3_MENU_CLEAR,        /**< CLEAR command */
    ENDAT3_MENU_ECHO,         /**< ECHO command */
    ENDAT3_MENU_RATE,         /**< RATE command */
    ENDAT3_MENU_HELLO,        /**< HELLO command */
    ENDAT3_MENU_RESERVED,     /**< RESERVED command */
    ENDAT3_MENU_BUSBC,        /**< BUSBC command */
    ENDAT3_MENU_BUSP2P,       /**< BUSP2P command */
    ENDAT3_MENU_BUSINIT,      /**< BUSINIT command */
    ENDAT3_MENU_FORCE         /**< FORCE command */
} endat3_menu_option;
/**
 * \brief HPF STATUS bits
 *
 * Individual bits in the HPF status byte.
 */
typedef enum endat3_hpf_status_bits_e {
    ENDAT3_HPF_STATUS_F       = 0x01,  /**< Bit 0: Collective error bit */
    ENDAT3_HPF_STATUS_W       = 0x02,  /**< Bit 1: Collective warning bit */
    ENDAT3_HPF_STATUS_HPFV    = 0x04,  /**< Bit 2: Validity of HPF data */
    ENDAT3_HPF_STATUS_RM      = 0x08,  /**< Bit 3: Availability of absolute value */
    ENDAT3_HPF_STATUS_ERR_REQ = 0x10   /**< Bit 4: Request code not supported */
} endat3_hpf_status_bits;

/**
 * \brief LPH Status Flags
 *
 * Status flags for the Low Priority Header (LPH).
 */
typedef enum endat3_lph_status_flags_e {
    ENDAT3_LPH_STATUS_IDLE        = 0x0,  /**< Idle state, no background operation in progress */
    ENDAT3_LPH_STATUS_RX_START    = 0x1,  /**< First frame of a multi-frame background response */
    ENDAT3_LPH_STATUS_RX_LAST     = 0x2,  /**< Last frame of a multi-frame background response */
    ENDAT3_LPH_STATUS_BUSY        = 0x3,  /**< Background operation in progress */
    ENDAT3_LPH_BG_ERR_EXEC        = 0x4,  /**< Error during background operation execution */
    ENDAT3_LPH_BG_BUSY            = 0x8,  /**< Background processor is busy */
    ENDAT3_LPH_BG_RTX_ERROR       = 0x10, /**< Background transmit/receive error */
    ENDAT3_LPH_STATE_MASK         = 0x3   /**< Mask for extracting LPH state (bits 0-1) */
} endat3_lph_status_flags;

/**
 * \brief EnDat3 Error Codes
 *
 * Error codes used in foreground and background communication.
 */
typedef enum endat3_error_code_e {
    /* General errors */
    ENDAT3_ERR_UNKNOWN                 = 0x0000,  /**< The cause of the error is unknown */

    /* Foreground errors (0x0001-0x0FFF) */
    ENDAT3_FGERR_RECONFIGURE           = 0x0001,  /**< Device is in configuration as a result of RECONFIGURE */
    ENDAT3_FGERR_ECHO                  = 0x0002,  /**< An ECHO is being responded to */
    ENDAT3_FGERR_INVALID_FID           = 0x0100,  /**< An invalid FID was configured */
    ENDAT3_FGERR_DUPLICATE_FID         = 0x0101,  /**< FID was selected more than once during the cycle */
    ENDAT3_FGERR_INVALID_DATA          = 0x0200,  /**< LPF is supported, but invalid data were delivered internally */
    ENDAT3_FGERR_INT_TRM               = 0x0201,  /**< LPF is supported but currently unavailable */
    ENDAT3_FGERR_NO_SENSOR_DATA        = 0x0300,  /**< Sensor box data not available */

    /* Background errors - Usage errors (0x1100-0x11FF) */
    ENDAT3_BGERR_USAGE                 = 0x1100,  /**< Generic operator error */
    ENDAT3_BGERR_USAGE_OPCODE          = 0x1101,  /**< Invalid or unsupported command code */
    ENDAT3_BGERR_USAGE_ARGUMENTS       = 0x1102,  /**< Invalid arguments */
    ENDAT3_BGERR_USAGE_SEQUENCE        = 0x1103,  /**< Invalid command sequence */
    ENDAT3_BGERR_USAGE_ACCESS_DENIED   = 0x1104,  /**< Access denied; insufficient user level */
    ENDAT3_BGERR_USAGE_MEM_ADDRESS     = 0x1105,  /**< Access to invalid address */
    ENDAT3_BGERR_USAGE_NO_BG           = 0x1106,  /**< Encoder does not support background processing */

    /* Background errors - Internal errors (0x1200-0x12FF) */
    ENDAT3_BGERR_INTERNAL              = 0x1200,  /**< Generic exception error in the encoder */
    ENDAT3_BGERR_INTERNAL_MEMORY       = 0x1201,  /**< Exception error when accessing memory */
    ENDAT3_BGERR_INTERNAL_CONFIG       = 0x1202   /**< Exception error: configuration invalid */
} endat3_error_code;

/**
 * \brief Background command request parameters
 *
 * Structure containing parameters for background command requests.
 * Used with \ref endat3_handle_background_command_request.
 */
typedef struct endat3_bg_cmd_params_s {
    uint8_t  index;      /**< Background data array index to start writing (valid range: 0-2, must be < \ref ENDAT3_MAX_BG_CMD_INDEX) */
    uint8_t  frame_cnt;  /**< Number of frames for this command */
    uint8_t  op_code;    /**< Background operation code */
    uint32_t addr_msb;   /**< Address MSB (bits [23:16]) */
    uint32_t addr_lsb;   /**< Address LSB (bits [15:0]) */
    uint32_t data;       /**< Operation-specific data */
} endat3_bg_cmd_params;

/**
 *    \brief    Structure defining ENDAT3 clock configuration for selected frequency
 *
 *    \details  Contains clock divisors and configuration calculated internally
 *              for achieving the desired baud rate. These values are written to PRU-ICSS
 *              registers during hardware initialization.
 *
 */
typedef struct endat3_clock_config_s {
    uint16_t  rx_div;
    /**< Rx clock divisor (value-1 written to register). Determines receive sample rate. */
    uint16_t  tx_div;
    /**< Tx clock divisor (value-1 written to register). Determines transmit baud rate. */
    uint16_t  rx_div_attr;
    /**< Rx oversampling rate, start bit polarity and fractional divider configuration.
     *   Bits [2:0] : Oversampling divisor (7 = 8x, 5 = 6x, 3 = 4x)
     *   Bit  [3]   : Start bit polarity (0 or 1)
     *   Bit  [15]  : Fractional divider enable (1=enable 1.5x fractional division) */
    uint16_t  is_core_clk;
    /**< Clock source selection for ENDAT3 communication.
     *   0 = Use UART clock (160/192 MHz)
     *   1 = Use Core clock (200/300 MHz) */
} endat3_clock_config;

/**
 * \brief EnDAT3 initialization parameters
 *
 * \details Parameters passed to \ref endat3_init to initialize an EnDAT3 instance.
 *          Use \ref endat3_params_init to populate with default values.
 */
typedef struct endat3_params_s {
    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS Handle obtained from PRUICSS_open(). Must be valid. */
} endat3_params;

/**
 * \brief EnDAT3 attributes (compile-time/SysConfig configuration data)
 *
 * \details Contains read-only configuration parameters set during initialization
 *          or generated by SysConfig. These values do not change after init.
 */
typedef struct endat3_attrs_s {
    uint8_t instance;
    /**< EnDAT3 instance index (0, 1, ...) for multi-instance configurations.
     *   Used to distinguish between multiple EnDAT3 instances in the system */

    uint8_t mode;
    /**< EnDAT3 configuration mode.
     *   0 = ENDAT3_MODE_SINGLE_CHANNEL_SINGLE_PRU (one channel, one PRU) */

    uint8_t pruicss_instance;
    /**< PRU-ICSS hardware instance number (0 or 1).
     *   0 = PRU-ICSSG0/PRU-ICSSM0
     *   1 = PRU-ICSSG1/PRU-ICSSM1 */

    uint8_t pruicss_slice;
    /**< PRU-ICSS slice selection (0 or 1).
     *   Each PRU-ICSS has 2 slices, each with its own set of PRU cores.
     *   0 = Slice 0 (contains PRU0/RTU-PRU0/TX-PRU0 on PRU-ICSSG, PRU0 on PRU-ICSSM)
     *   1 = Slice 1 (contains PRU1/RTU-PRU1/TX-PRU1 on PRU-ICSSG, PRU1 on PRU-ICSSM) */

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
    /**< EnDAT3 communication baud rate in Mbps (configured from SysConfig).
     *   Value: 0 for 12.5 Mbps, 1 for 25 Mbps */

    uint32_t core_clk_freq;
    /**< PRU-ICSS Core Clock frequency in Hz (not MHz). */

    uint32_t iep_clk_freq;
    /**< PRU-ICSS IEP (Industrial Ethernet Peripheral) timer clock frequency in Hz. */

    uint8_t iep_instance;
    /**< IEP instance number used for periodic trigger mode (0=IEP0, 1=IEP1) */

    uint8_t iep_cmp_event;
    /**< IEP compare event number (0-15) used for periodic CMP trigger mode */

    uint8_t iep_cap_event;
    /**< IEP capture event number (0-7) used for periodic CAP trigger mode */

    void *iep_base_addr;
    /**< IEP base address for IEP timer configuration in periodic trigger mode */
} endat3_attrs;

/**
 * \brief High Priority Frame structure
 *
 * The High Priority Frame (HPF) contains position data and critical status information.
 */
typedef struct endat3_hpf_s {
    uint8_t data[HPF_DATA_SIZE];    /**< 48-bit (6-byte) data payload */
    uint8_t status;                 /**< Status byte with error/warning flags */
    uint8_t crc;                    /**< CRC checksum */
} endat3_hpf;

/**
 * \brief Low Priority Header structure
 *
 * The Low Priority Header (LPH) contains information about following Low Priority Frames.
 */
typedef struct endat3_lph_s {
    uint8_t status;     /**< Status byte */
    uint8_t num_lpf;    /**< Number of LPF frames (0-\ref MAX_LPF_COUNT) */
    uint8_t reserved;   /**< Reserved byte for alignment */
    uint8_t crc;        /**< CRC checksum */
} endat3_lph;

/**
 * \brief Low Priority Frame structure
 *
 * Low Priority Frames (LPF) contain additional data and diagnostic information.
 */
typedef struct endat3_lpf_s {
    uint8_t status;                 /**< Status byte with FID (Frame ID) */
    uint8_t data[LPF_DATA_SIZE];    /**< 48-bit (6-byte) data payload */
    uint8_t crc;                    /**< CRC checksum  */
} endat3_lpf;

/**
 *    \brief    Structure defining ENDAT3 periodic trigger configuration
 *
 *    \details  Contains IEP event configuration for periodic trigger mode
 */
typedef struct endat3_periodic_trigger_cfg_s
{
    uint8_t iep_cmp_event;
    /**< IEP compare event number for periodic CMP mode */

    uint8_t iep_cap_event;
    /**< IEP capture event number for periodic CAP mode */

    uint16_t reserved;
    /**< Reserved for alignment */

    uint32_t iep_capture_reg;
    /**< IEP capture register address for periodic CAP mode */
} endat3_periodic_trigger_cfg;

/**
 * \brief EnDAT3 Interface structure
 *
 * Main structure for storing communication buffers and status information.
 */
typedef struct endat3_interface_s {
    /* Communication buffers */
    volatile uint32_t lut[LUT_SIZE];             /**< Lookup table */
    volatile uint8_t rx_buffer[RX_BUFFER_SIZE];  /**< Receive buffer */

    /* Frame structures */
    volatile endat3_hpf hpf;                     /**< High Priority Frame */
    volatile endat3_lph lph;                     /**< Low Priority Header */
    volatile endat3_lpf lpf[MAX_LPF_COUNT];      /**< Low Priority Frames array */

    /* Status flags */
    volatile uint8_t connected;                  /**< Connection status */
    volatile uint8_t busy;                       /**< Transfer in progress */
    volatile uint8_t comm_cycle_flag;            /**< Communication cycle flag */
    volatile uint8_t reserved1;                  /**< Reserved for alignment */

    /* Frame control */
    volatile uint32_t expected_tx_frames_count;  /**< Expected number of TX frames */
    volatile uint32_t current_tx_frame_offset;   /**< Current TX frame offset */

    /* Data buffers */
    volatile uint8_t tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer */
    volatile uint32_t bg_data[BG_DATA_SIZE];     /**< Background data */

    /* Timing and operation codes */
    volatile uint32_t propagation_time;          /**< Measured propagation time */
    volatile uint32_t foreground_op_code;        /**< Foreground operation code */
    volatile uint32_t background_op_code;        /**< Background operation code */

    /* Periodic trigger configuration */
    volatile uint8_t opmode_config;              /**< Operating mode: 0=periodic CMP, 1=host, 2=periodic CAP */
    volatile uint8_t reserved2;                  /**< Reserved for alignment */
    volatile uint8_t reserved3;                  /**< Reserved for alignment */
    volatile uint8_t reserved4;                  /**< Reserved for alignment */

    /* Delay Cycle Configuration (frequency-independent timing) */
    /* These values are calculated by R5F based on actual PRU frequency */
    volatile uint32_t delay_tx_start_1;          /**< TX start delay 1 in PRU cycles */
    volatile uint32_t delay_tx_start_2;          /**< TX start delay 2 in PRU cycles */
    volatile uint32_t delay_tx_start_3;          /**< TX start delay 3 in PRU cycles */
    volatile uint32_t delay_sampling;            /**< Sampling delay in PRU cycles */
    volatile uint32_t delay_10ms;                /**< 10ms delay in PRU cycles */

    /* Trigger Control */
    volatile uint8_t start_trigger;              /**< Start trigger: released by R5F core based on operating mode */

    /* Channel Enable Mask */
    volatile uint8_t channel_enable_mask;        /**< Channel enable mask: bit 0=CH0, bit 1=CH1, bit 2=CH2 */

    volatile uint8_t reserved5;                  /**< Reserved for alignment */
    volatile uint8_t reserved6;                  /**< Reserved for alignment */

    volatile uint32_t iep_base_address;
    /**< IEP base address used for periodic trigger mode */

    volatile endat3_periodic_trigger_cfg trigger_params;
    /**< Periodic trigger configuration parameters.
     *   Contains IEP event numbers and capture register addresses */

} endat3_interface;

/**
 * \brief EnDAT3 private data structure (runtime state and configuration)
 *
 * \details Contains runtime state information and pointers to PRU-ICSS resources.
 *          This structure is initialized during \ref endat3_init and should be
 *          accessed via \ref endat3_get_priv API.
 */
typedef struct endat3_priv_s {
    uint8_t             is_open;
    /**< Initialization state flag.
     *   0 = Driver closed/not initialized
     *   1 = Driver successfully initialized and open */
    PRUICSS_Handle      pruicss_handle;
    /**< PRUICSS_Handle for PRU-ICSS instance */
    uint32_t            *base_mem_addr;
    /**< Base Memory Address for endat3 channel configuration */
    endat3_interface    *endat3_interface;
    /**< Pointer to interface communication structure */
} endat3_priv;

/**
 * \brief EnDAT3 configuration handle structure
 *
 * \details This structure combines pointers to both runtime state (priv) and compile-time
 *          configuration (attrs). The handle is returned by \ref endat3_init() and passed to all
 *          EnDAT3 driver APIs to identify the specific EnDAT3 instance being operated on.
 */
typedef struct endat3_config_s {
    endat3_priv *priv;
    /**< Pointer to EnDAT3 private data (runtime state).
     *   Contains encoder parameters, interface pointers, and runtime operational state */

    const endat3_attrs *attrs;
    /**< Pointer to EnDAT3 attributes (read-only configuration from SysConfig).
     *   Contains compile-time configuration including PRU instance, mode, and core settings */
} endat3_config;

/**
 * \brief EnDAT3 Driver Handle
 *
 * \details Opaque handle to an EnDAT3 instance. Obtained from \ref endat3_init() and used
 *          in all subsequent EnDAT3 API calls.
 */
typedef endat3_config *endat3_handle;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* ========================================================================== */
/*                      Initialization and Configuration APIs                 */
/* ========================================================================== */

/**
 * \brief Initialize EnDAT3 parameters with default values
 *
 * \details Populates the endat3_params structure with default values.
 *          Must be called before passing params to \ref endat3_init.
 *
 * \param[out] params  Pointer to params structure to initialize
 *
 * \note On NULL params, function returns without performing any operation
 */
void endat3_params_init(endat3_params *params);

/**
 * \brief Initialize an EnDAT3 instance
 *
 * \details Initializes an EnDAT3 driver instance with the specified configuration.
 *          This function validates all parameters, sets up the firmware interface,
 *          and configures hardware based on both SysConfig attributes and runtime params.
 *
 * \note This function internally calls: PRUICSS_setGpMuxSelect(), endat3_config_clr_cfg0(),
 *       endat3_calculate_clock(), endat3_config_clock(), endat3_set_delay_cycles(),
 *       endat3_set_channel_mask(), endat3_generate_memory_image(),
 *       endat3_config_iep_base_address(), \ref endat3_config_iep_cmp_event(),
 *       \ref endat3_config_iep_cap_event(), and \ref endat3_set_operating_mode().
 *
 * \param[in]  index           Index of EnDAT3 (0 to gEndat3ConfigNum - 1)
 * \param[in]  endat3_params   Pointer to structure containing EnDAT3 initialization parameters
 *
 * \retval     handle          Pointer to initialized endat3_handle instance
 * \retval     NULL            On validation failure including:
 *                             - Invalid index (>= gEndat3ConfigNum)
 *                             - NULL params or params->pruicss_handle
 *                             - NULL attrs/priv in config array
 *                             - Invalid attrs fields
 *                             - Only ENDAT3_BAUD_RATE_12_5_MBPS supported (12.5 Mbps)
 *                             - Only 200MHz or 300MHz core_clk_freq supported
 *                             - Hardware initialization failure (PRUICSS_setGpMuxSelect, clock config, etc.)
 */
endat3_handle endat3_init(uint32_t index, const endat3_params *endat3_params);

/**
 * \brief De-initialize an EnDAT3 instance
 *
 * \details De-initializes the EnDAT3 instance by marking the handle as closed
 *          (is_open = 0). It does not free memory or disable PRU cores.
 *
 * \param[in]  handle   Handle to EnDAT3 instance
 *
 * \note On NULL handle, function returns without performing any operation.
 */
void endat3_deinit(endat3_handle handle);

/**
 * \brief Get pointer to EnDAT3 attributes structure
 *
 * \details Provides access to the read-only attributes structure containing
 *          configuration parameters set during initialization or generated by SysConfig.
 *
 * \param[in]  handle   EnDAT3 handle
 *
 * \retval     attrs    Pointer to const endat3_attrs structure
 * \retval     NULL     If handle is NULL
 */
const endat3_attrs* endat3_get_attrs(endat3_handle handle);

/**
 * \brief Get pointer to EnDAT3 private data structure
 *
 * \details Provides access to the private data structure containing runtime state
 *          information.
 *
 * \param[in]  handle   EnDAT3 handle
 *
 * \retval     priv     Pointer to endat3_priv structure
 * \retval     NULL     If handle is NULL
 */
endat3_priv* endat3_get_priv(endat3_handle handle);

/* ========================================================================== */
/*                      Command send/receive APIs                             */
/* ========================================================================== */

/**
 * \brief Send EnDAT3 command
 *
 * Prepares and sends an EnDAT3 command with the specified command code
 * and number of frames. Validates input parameters and checks for buffer
 * overflow conditions before transmission.
 *
 * \param handle EnDAT3 handle
 * \param cmd Command code
 * \param frames Number of frames to transmit (1-6, limited by TX_BUFFER_SIZE/4)
 *
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) on error: invalid handle, NULL pointers, buffer overflow, or invalid parameters
 *
 * \note **CRITICAL**: Always check return value before calling \ref endat3_set_busy() or waiting for response
 *
 * \note Validation performed:
 *       - Handle NULL check
 *       - Frame count validation via endat3_prepare_request() (1 to ENDAT3_MAX_TX_FRAMES)
 *       - Background data array validation (NULL check)
 *
 * \note Internal function used: endat3_prepare_request()
 */
int32_t endat3_send_command(endat3_handle handle, uint8_t cmd, uint8_t frames);

/**
 * \brief Receive EnDAT3 response
 *
 * Receives and processes EnDAT3 response frames, verifying CRC integrity
 * and parsing the data into appropriate structures.
 *
 * \param handle EnDAT3 handle
 *
 * \return ENDAT3_SUCCESS (0) on successful reception with valid CRC
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle or priv is NULL
 * \return ENDAT3_ERR_BUSY (-2) if encoder is busy (transfer in progress)
 * \return ENDAT3_ERR_LPH_CRC_FAIL (-4) on LPH CRC mismatch
 * \return ENDAT3_ERR_HPF_CRC_FAIL (-5) on HPF CRC mismatch
 * \return ENDAT3_ERR_LPF_CRC_FAIL (-6) on LPF CRC mismatch
 * \return ENDAT3_ERR_SAMPLING_ERROR (-7) on encoder sampling/timing error (busy==ENCODER_ERROR)
 *
 * \note **IMPORTANT**: Check return value == ENDAT3_SUCCESS before reading HPF/LPH/LPF data
 *
 * \note Validation performed:
 *       - Handle NULL check
 *       - Busy status checking via endat3_wait_rx_complete()
 *       - RX buffer validation (internal)
 *       - CRC validation for HPF, LPH, and LPF (when applicable)
 */
int32_t endat3_receive_response(endat3_handle handle);

/* ========================================================================== */
/*                            Error Handling APIs                             */
/* ========================================================================== */

/**
 * \brief Get error description string
 *
 * Returns a human-readable description for the given EnDAT3 error code.
 * Useful for logging and debugging error conditions.
 *
 * \param error_code The error code to get description for
 * \return Pointer to error description string (never NULL, always returns valid string)
 *
 * \code
 * endat3_error_code error;
 * if(endat3_get_error_code(handle, &error) == ENDAT3_SUCCESS)
 * {
 *     const char* desc = endat3_get_error_description(error);
 *     DebugP_log("Error: %s\r\n", desc);
 * }
 * \endcode
 */
const char* endat3_get_error_description(endat3_error_code error_code);

/**
 * \brief Get recommended action for error code
 *
 * Returns a recommended action string for the given EnDAT3 error code.
 * Helps users understand what to do when an error occurs.
 *
 * \param error_code The error code to get recommended action for
 * \return Pointer to recommended action string (never NULL, always returns valid string)
 *
 * \code
 * endat3_error_code error;
 * if(endat3_get_error_code(handle, &error) == ENDAT3_SUCCESS)
 * {
 *     const char* action = endat3_get_error_action(error);
 *     DebugP_log("Recommended action: %s\r\n", action);
 * }
 * \endcode
 */
const char* endat3_get_error_action(endat3_error_code error_code);

/**
 * \brief Handle background command request
 *
 * Processes and prepares a background command request for transmission.
 * Handles various background operations like READ, WRITE, AUTH, PROTECT, etc.
 * Formats the command data according to the operation code and parameters.
 *
 * \param handle EnDAT3 handle
 * \param params Pointer to background command parameters structure (endat3_bg_cmd_params)
 * \param mode Pointer to store mode value (optional, can be NULL if not needed).
 *             Only populated when op_code is ENDAT3_BGREQ_PROTECT and mode is not NULL.
 * \param acc_level_desc Pointer to store access level description string (optional, can be NULL if not needed).
 *                     Only populated when op_code is ENDAT3_BGREQ_PROTECT and acc_level_desc is not NULL.
 *
 * \retval ENDAT3_SUCCESS (0) on success
 * \retval ENDAT3_ERR_INVALID_INPUT (-1) on validation failure (NULL handle, NULL params, or index >= ENDAT3_MAX_BG_CMD_INDEX)
 *
 * \note Validation performed:
 *       - Handle NULL check
 *       - Params pointer NULL check
 *       - Index bounds check (index must be 0-2)
 *       - Individual parameter members (op_code, addr_msb, addr_lsb, data, frame_cnt) are NOT validated
 *
 * \note Output parameters (mode, acc_level_desc) are only meaningful for ENDAT3_BGREQ_PROTECT operation
 */
int32_t endat3_handle_background_command_request(endat3_handle handle, const endat3_bg_cmd_params *params, uint8_t *mode, const char **acc_level_desc);

/* ========================================================================== */
/*                    HPF (High Priority Frame) Access APIs                   */
/* ========================================================================== */

/**
 * \brief Get HPF status byte
 *
 * Retrieves the status byte from the High Priority Frame, which contains
 * critical status flags including F, W, HPFV, RM, and ERR_REQ bits.
 *
 * \param handle EnDAT3 handle
 * \param status Pointer to store HPF status byte
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or status is NULL
 *
 * \code
 * uint8_t status;
 * if(endat3_get_hpf_status(handle, &status) == ENDAT3_SUCCESS)
 * {
 *     if(status & ENDAT3_HPF_STATUS_F)
 * {
 *         // Handle error condition
 *     }
 * }
 * \endcode
 */
int32_t endat3_get_hpf_status(endat3_handle handle, uint8_t *status);

/**
 * \brief Get HPF data array
 *
 * Retrieves the 6-byte data payload from the High Priority Frame.
 * The data is copied to the provided buffer.
 *
 * \param handle EnDAT3 handle
 * \param data Buffer to store HPF data (must be at least 6 bytes)
 * \return Number of bytes copied (6) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle or data is NULL
 *
 * \note On failure (NULL handle or data pointer), returns ENDAT3_ERR_INVALID_INPUT immediately without accessing any memory.
 *       Always check return value before using the data buffer.
 *
 * \code
 * uint8_t hpf_data[6];
 * int32_t bytes = endat3_get_hpf_data(handle, hpf_data);
 * if(bytes == 6)
 * {
 *     // Process HPF data
 * }
 * \endcode
 */
int32_t endat3_get_hpf_data(endat3_handle handle, uint8_t *data);

/**
 * \brief Get HPF CRC value
 *
 * Retrieves the CRC checksum from the High Priority Frame.
 *
 * \param handle EnDAT3 handle
 * \param crc Pointer to store HPF CRC value
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or crc is NULL
 */
int32_t endat3_get_hpf_crc(endat3_handle handle, uint8_t *crc);

/**
 * \brief Get HPF data as 64-bit value
 *
 * Retrieves the HPF data payload as a single 64-bit unsigned integer.
 * Useful for position data extraction. Data is packed in little-endian format.
 *
 * \param handle EnDAT3 handle
 * \param data Pointer to store HPF data as uint64_t
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or data is NULL
 *
 * \code
 * uint64_t position;
 * if(endat3_get_hpf_data_64_bit(handle, &position) == ENDAT3_SUCCESS)
 * {
 *     uint32_t single_turn = position & 0x1FFF;  // Extract 13-bit single turn
 * }
 * \endcode
 */
int32_t endat3_get_hpf_data_64_bit(endat3_handle handle, uint64_t *data);

/**
 * \brief Check if HPF data is valid
 *
 * Checks the HPFV bit in the HPF status to determine if the data is valid.
 *
 * \param handle EnDAT3 handle
 * \param is_valid Pointer to store result (1 if HPF data is valid, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or is_valid is NULL
 */
int32_t endat3_is_hpf_data_valid(endat3_handle handle, uint8_t *is_valid);

/**
 * \brief Check if HPF has error flag set
 *
 * Checks the F bit in the HPF status to determine if an error is present.
 *
 * \param handle EnDAT3 handle
 * \param has_error Pointer to store result (1 if error flag is set, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or has_error is NULL
 */
int32_t endat3_has_hpf_error(endat3_handle handle, uint8_t *has_error);

/**
 * \brief Check if HPF has warning flag set
 *
 * Checks the W bit in the HPF status to determine if a warning is present.
 *
 * \param handle EnDAT3 handle
 * \param has_warning Pointer to store result (1 if warning flag is set, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or has_warning is NULL
 */
int32_t endat3_has_hpf_warning(endat3_handle handle, uint8_t *has_warning);

/**
 * \brief Check if absolute value is available
 *
 * Checks the RM bit in the HPF status to determine if absolute position
 * value is available.
 *
 * \param handle EnDAT3 handle
 * \param has_absolute Pointer to store result (1 if absolute value is available, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or has_absolute is NULL
 */
int32_t endat3_has_absolute_value(endat3_handle handle, uint8_t *has_absolute);

/* ========================================================================== */
/*                    LPH (Low Priority Header) Access APIs                   */
/* ========================================================================== */

/**
 * \brief Get LPH status byte
 *
 * Retrieves the status byte from the Low Priority Header, which contains
 * the communication state and error flags.
 *
 * \param handle EnDAT3 handle
 * \param status Pointer to store LPH status byte
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or status is NULL
 */
int32_t endat3_get_lph_status(endat3_handle handle, uint8_t *status);

/**
 * \brief Get number of LPF frames
 *
 * Retrieves the number of Low Priority Frames indicated in the LPH.
 *
 * \param handle EnDAT3 handle
 * \param num_lpf Pointer to store number of LPF frames (0 to MAX_LPF_COUNT)
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or num_lpf is NULL
 */
int32_t endat3_get_lph_lpf_count(endat3_handle handle, uint8_t *num_lpf);

/**
 * \brief Get LPH CRC value
 *
 * Retrieves the CRC checksum from the Low Priority Header.
 *
 * \param handle EnDAT3 handle
 * \param crc Pointer to store LPH CRC value
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or crc is NULL
 */
int32_t endat3_get_lph_crc(endat3_handle handle, uint8_t *crc);

/**
 * \brief Get LPH communication state
 *
 * Extracts the communication state from the LPH status byte (bits 0-1).
 *
 * \param handle EnDAT3 handle
 * \param state Pointer to store LPH state (IDLE, RX_START, RX_LAST, BUSY)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or state is NULL
 *
 * \code
 * endat3_lph_status state;
 * if(endat3_get_lph_state(handle, &state) == ENDAT3_SUCCESS)
 * {
 *     switch(state)
 *     {
 *         case LPH_STATUS_IDLE:
 *             // Ready for new command
 *             break;
 *         case LPH_STATUS_BUSY:
 *             // Background operation in progress
 *             break;
 *     }
 * }
 * \endcode
 */
int32_t endat3_get_lph_state(endat3_handle handle, endat3_lph_status *state);

/**
 * \brief Check if background operation has error
 *
 * Checks the BG.ERR_EXEC bit in the LPH status.
 *
 * \param handle EnDAT3 handle
 * \param has_error Pointer to store result (1 if background error is present, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or has_error is NULL
 */
int32_t endat3_has_bg_error(endat3_handle handle, uint8_t *has_error);

/**
 * \brief Check if background processor is busy
 *
 * Checks the BG.BUSY bit in the LPH status.
 *
 * \param handle EnDAT3 handle
 * \param is_busy Pointer to store result (1 if background processor is busy, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or is_busy is NULL
 */
int32_t endat3_is_bg_busy(endat3_handle handle, uint8_t *is_busy);

/**
 * \brief Check if background RTX error occurred
 *
 * Checks the BG.RTX_ERROR bit in the LPH status.
 *
 * \param handle EnDAT3 handle
 * \param has_error Pointer to store result (1 if background RTX error occurred, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or has_error is NULL
 */
int32_t endat3_has_bg_rtx_error(endat3_handle handle, uint8_t *has_error);

/* ========================================================================== */
/*                    LPF (Low Priority Frame) Access APIs                    */
/* ========================================================================== */

/**
 * \brief Get LPF status byte for specific frame
 *
 * Retrieves the status byte from a specific Low Priority Frame.
 *
 * \param handle EnDAT3 handle
 * \param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \param status Pointer to store LPF status byte
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle, status is NULL or index is invalid
 */
int32_t endat3_get_lpf_status(endat3_handle handle, uint8_t index, uint8_t *status);

/**
 * \brief Get LPF data array for specific frame
 *
 * Retrieves the 6-byte data payload from a specific Low Priority Frame.
 *
 * \param handle EnDAT3 handle
 * \param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \param data Buffer to store LPF data (must be at least 6 bytes)
 * \return Number of bytes copied (6) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL, data is NULL, or index >= MAX_LPF_COUNT(8)
 *
 * \note Validation performed:
 *       - Handle and data buffer validation
 *       - Index bounds checking (index < MAX_LPF_COUNT)
 *
 * \code
 * uint8_t lpf_data[6];
 * int32_t bytes = endat3_get_lpf_data(handle, 0, lpf_data);
 * if(bytes == 6)
 * {
 *     // Process first LPF data
 * }
 * \endcode
 */
int32_t endat3_get_lpf_data(endat3_handle handle, uint8_t index, uint8_t *data);

/**
 * \brief Get LPF CRC value for specific frame
 *
 * Retrieves the CRC checksum from a specific Low Priority Frame for validation.
 *
 * \param handle EnDAT3 handle
 * \param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \param crc Pointer to store LPF CRC byte
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle, crc is NULL or index is invalid
 */
int32_t endat3_get_lpf_crc(endat3_handle handle, uint8_t index, uint8_t *crc);

/**
 * \brief Get LPF FID (Frame ID) for specific frame
 *
 * Extracts the Frame ID from the LPF status byte.
 *
 * \param handle EnDAT3 handle
 * \param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \param fid Pointer to store FID value (0-255)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle, fid is NULL or index is invalid
 */
int32_t endat3_get_lpf_fid(endat3_handle handle, uint8_t index, uint8_t *fid);

/* ========================================================================== */
/*                    Communication Control APIs                              */
/* ========================================================================== */

/**
 * \brief Get connection status
 *
 * Checks if the encoder is connected and communicating.
 *
 * \param handle EnDAT3 handle
 * \param is_connected Pointer to store connection status (1 if connected, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or is_connected is NULL
 */
int32_t endat3_is_connected(endat3_handle handle, uint8_t *is_connected);

/**
 * \brief Get busy status
 *
 * Checks if a transfer is currently in progress. Returns 1 only when encoder state
 * is ENCODER_BUSY. Both ENCODER_IDLE and ENCODER_ERROR states return 0.
 *
 * \param handle EnDAT3 handle
 * \param is_busy Pointer to store busy status (1 if state == ENCODER_BUSY, 0 otherwise)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or is_busy is NULL
 */
int32_t endat3_is_busy(endat3_handle handle, uint8_t *is_busy);

/**
 * \brief Set or clear busy status
 *
 * Sets or clears the busy flag. Set busy=1 to mark a transfer as in progress;
 * set busy=0 to clear the busy state when no transfer is active.
 *
 * \param handle EnDAT3 handle
 * \param busy Busy state to set (1 for busy, 0 for not busy)
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL, busy > 1, or internal structures are NULL
 *
 * \note Check return value to ensure state was set properly
 */
int32_t endat3_set_busy(endat3_handle handle, uint8_t busy);

/**
 * \brief Get expected TX frame count
 *
 * Retrieves the number of frames expected to be transmitted.
 *
 * \param handle EnDAT3 handle
 * \param count Pointer to store expected TX frame count
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or count is NULL
 */
int32_t endat3_get_expected_tx_frame_count(endat3_handle handle, uint32_t *count);

/**
 * \brief Set expected TX frame count
 *
 * Sets the number of frames expected to be transmitted.
 *
 * \param handle EnDAT3 handle
 * \param count Expected frame count
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *
 * \note Check return value - incorrect frame count causes communication errors
 */
int32_t endat3_set_expected_tx_frame_count(endat3_handle handle, uint32_t count);

/**
 * \brief Get propagation time
 *
 * Retrieves the measured propagation time from ECHO command.
 * The value is in PRU clock cycles.
 *
 * \param handle EnDAT3 handle
 * \param prop_time Pointer to store propagation time in PRU clock cycles
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or prop_time is NULL
 *
 * \code
 * uint32_t prop_time_cycles;
 * if(endat3_get_propagation_time(handle, &prop_time_cycles) == ENDAT3_SUCCESS)
 * {
 *     // Convert to nanoseconds (assuming 200MHz PRU clock)
 *     uint32_t prop_time_ns = (prop_time_cycles * 1000) / 200;
 * }
 * \endcode
 */
int32_t endat3_get_propagation_time(endat3_handle handle, uint32_t *prop_time);

/* ========================================================================== */
/*                    Command and Data APIs                                   */
/* ========================================================================== */

/**
 * \brief Get foreground operation code
 *
 * Retrieves the current foreground operation code.
 *
 * \param handle EnDAT3 handle
 * \param opcode Pointer to store foreground operation code
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or opcode is NULL
 */
int32_t endat3_get_foreground_op_code(endat3_handle handle, uint32_t *opcode);

/**
 * \brief Set foreground operation code
 *
 * Sets the foreground operation code for the next command.
 *
 * \param handle EnDAT3 handle
 * \param opcode Operation code to set (use endat3_req_code enum values)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *         **CRITICAL**: Check return value before calling \ref endat3_send_command() - wrong opcode sends wrong command
 *
 * \code
 * if(endat3_set_foreground_op_code(handle, ENDAT3_REQ_DATA0) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to set foreground opcode\r\n");
 *     return;
 * }
 * \endcode
 */
int32_t endat3_set_foreground_op_code(endat3_handle handle, uint32_t opcode);

/**
 * \brief Get background operation code
 *
 * Retrieves the current background operation code.
 *
 * \param handle EnDAT3 handle
 * \param opcode Pointer to store background operation code
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle or opcode is NULL
 */
int32_t endat3_get_background_op_code(endat3_handle handle, uint32_t *opcode);

/**
 * \brief Set background operation code
 *
 * Sets the background operation code for the next command.
 *
 * \param handle EnDAT3 handle
 * \param opcode Operation code to set (use endat3_bg_req_opcode enum values)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *         **CRITICAL**: Check return value before calling \ref endat3_send_command() - wrong opcode sends wrong background operation
 *
 * \code
 * if(endat3_set_background_op_code(handle, ENDAT3_BGREQ_READ) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to set background opcode\r\n");
 *     return;
 * }
 * \endcode
 */
int32_t endat3_set_background_op_code(endat3_handle handle, uint32_t opcode);

/**
 * \brief Get background data word
 *
 * Retrieves a specific word from the background data array.
 *
 * \param handle EnDAT3 handle
 * \param index Data word index (0 to BG_DATA_SIZE-1)
 * \param data Pointer to store the retrieved data value
 * \return ENDAT3_SUCCESS (0) on success, ENDAT3_ERR_INVALID_INPUT (-1) if handle, data is NULL, or index is invalid
 */
int32_t endat3_get_bg_data(endat3_handle handle, uint8_t index, uint32_t *data);

/**
 * \brief Set background data word
 *
 * Sets a specific word in the background data array.
 *
 * \param handle EnDAT3 handle
 * \param index Data word index (0 to BG_DATA_SIZE-1)
 * \param data Data value to set
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL or index >= BG_DATA_SIZE(6)
 *
 * \note Check return value - wrong background data sends incorrect command parameters
 *
 * \code
 * // Set RESET command data
 * if(endat3_set_bg_data(handle, 0, ENDAT3_RESET_HARD) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to set background data\r\n");
 *     return;
 * }
 * \endcode
 */
int32_t endat3_set_bg_data(endat3_handle handle, uint8_t index, uint32_t data);

/**
 * \brief Get all background data
 *
 * Retrieves all background data words into the provided buffer.
 *
 * \param handle EnDAT3 handle
 * \param data Buffer to store background data (must be at least BG_DATA_SIZE words = 6 words = 24 bytes)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL or data buffer is NULL
 *
 * \note Check return value to ensure data was retrieved successfully
 */
int32_t endat3_get_all_bg_data(endat3_handle handle, uint32_t *data);

/**
 * \brief Set all background data
 *
 * Sets all background data words from the provided buffer. Copies exactly BG_DATA_SIZE (6) words
 * (24 bytes) from the source buffer to the internal background data array.
 *
 * \param handle EnDAT3 handle
 * \param data Buffer containing background data (must be at least BG_DATA_SIZE words = 6 words = 24 bytes)
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL, data buffer is NULL, or internal structures are NULL
 *
 * \note This function copies a fixed size of BG_DATA_SIZE (6) words regardless of actual data content
 * \note Check return value to ensure data was set successfully
 */
int32_t endat3_set_all_bg_data(endat3_handle handle, const uint32_t *data);

/* ========================================================================== */
/*                    Buffer Access APIs                                      */
/* ========================================================================== */

/**
 * \brief Get RX buffer pointer
 *
 * Provides read-only access to the receive buffer.
 *
 * \param handle EnDAT3 handle
 * \return Pointer to RX buffer (RX_BUFFER_SIZE = 64 bytes), or NULL if handle is NULL
 *
 * \warning Do not modify the returned buffer. Use for read-only access.
 */
const volatile uint8_t* endat3_get_rx_buffer(endat3_handle handle);

/**
 * \brief Get TX buffer pointer
 *
 * Provides read-only access to the transmit buffer.
 *
 * \param handle EnDAT3 handle
 * \return Pointer to TX buffer (TX_BUFFER_SIZE = 24 bytes), or NULL if handle is NULL
 *
 * \warning Do not modify the returned buffer. Use for read-only access.
 */
const volatile uint8_t* endat3_get_tx_buffer(endat3_handle handle);

/**
 * \brief Copy data to TX buffer
 *
 * Safely copies data to the transmit buffer with bounds checking.
 *
 * \param handle EnDAT3 handle
 * \param data Source data buffer
 * \param length Number of bytes to copy (1-24)
 * \return Number of bytes copied on success (1-24)
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL, data is NULL, length is 0, or length > TX_BUFFER_SIZE
 */
int32_t endat3_set_tx_buffer(endat3_handle handle, const uint8_t *data, uint32_t length);

/* ========================================================================== */
/*                    Utility and Helper APIs                                 */
/* ========================================================================== */

/**
 * \brief Get complete HPF structure
 *
 * Retrieves the entire High Priority Frame structure.
 *
 * \param handle EnDAT3 handle
 * \param hpf Pointer to HPF structure to fill
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle or hpf is NULL
 *
 * \code
 * endat3_hpf hpf;
 * if(endat3_get_hpf_frame(handle, &hpf) == 0)
 * {
 *     // Access hpf.status, hpf.data, hpf.crc
 * }
 * \endcode
 */
int32_t endat3_get_hpf_frame(endat3_handle handle, endat3_hpf *hpf);

/**
 * \brief Get complete LPH structure
 *
 * Retrieves the entire Low Priority Header structure.
 *
 * \param handle EnDAT3 handle
 * \param lph Pointer to LPH structure to fill
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle or lph is NULL
 */
int32_t endat3_get_lph_frame(endat3_handle handle, endat3_lph *lph);

/**
 * \brief Get complete LPF structure
 *
 * Retrieves the entire Low Priority Frame structure for a specific index.
 *
 * \param handle EnDAT3 handle
 * \param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \param lpf Pointer to LPF structure to fill
 * \return ENDAT3_SUCCESS (0) on success
 * \return ENDAT3_ERR_INVALID_INPUT (-1) if handle/lpf is NULL or index is invalid
 */
int32_t endat3_get_lpf_frame(endat3_handle handle, uint8_t index, endat3_lpf *lpf);

/**
 * \brief Extract error code from HPF or LPF
 *
 * Extracts the error code from HPF data (when HPFV=0) or LPF data
 * (when BG.ERR_EXEC is set).
 *
 * \param handle EnDAT3 handle
 * \param error_code Pointer to store error code value
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or error_code is NULL
 *
 * \code
 * endat3_error_code error;
 * if(endat3_get_error_code(handle, &error) == ENDAT3_SUCCESS)
 * {
 *     if(error != ENDAT3_ERR_UNKNOWN)
 *     {
 *         const char* desc = endat3_get_error_description(error);
 *         DebugP_log("Error: %s\r\n", desc);
 *     }
 * }
 * \endcode
 */
int32_t endat3_get_error_code(endat3_handle handle, endat3_error_code *error_code);

/**
 * \brief Get interface pointer (advanced use)
 *
 * Provides direct access to the endat3_interface structure for advanced
 * users who need full control. Use with caution.
 *
 * \param handle EnDAT3 handle
 * \return Pointer to endat3_interface structure, or NULL if handle is NULL
 *
 * \warning Direct manipulation of the interface structure can lead to
 *          undefined behavior. Use the provided APIs whenever possible.
 */
endat3_interface* endat3_get_interface(endat3_handle handle);

/**
 * \brief Set operating mode (host trigger or periodic trigger)
 *
 * Configures the firmware operating mode for the EnDAT3 interface.
 * This determines whether the encoder is triggered by host command,
 * or sampled automatically when IEP counter reaches the configured
 * CMP event compare value, or sampled automatically when an external
 * signal triggers the IEP capture event.
 *
 *  **Configuration requirements for \ref ENDAT3_OPMODE_PERIODIC_CMP :**
 *  - IEP hardware CMP registers must be configured separately
 *  - Use \ref endat3_config_iep_cmp_event to set event number in firmware. This function
 *    is called inside \ref endat3_init by default.
 *  - CMP event range: 0-15
 *
 *  **Configuration requirements for \ref ENDAT3_OPMODE_PERIODIC_CAP :**
 *  - IEP hardware CAP registers must be configured separately
 *  - External signal to IEP capture input should be configured
 *  - Use \ref endat3_config_iep_cap_event to set event number in firmware. This function
 *    is called inside \ref endat3_init by default.
 *  - CAP event range: 0-7
 *
 * \param handle EnDAT3 handle
 * \param opmode Operating mode: 0 = periodic CMP trigger, 1 = host trigger, 2 = periodic CAP trigger
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *         ENDAT3_ERR_INVALID_OPMODE (-8) if opmode > 2
 *         **CRITICAL**: Check return value - wrong operating mode causes complete communication failure
 *
 * \code
 * // Set to host trigger mode
 * if(endat3_set_operating_mode(handle, ENDAT3_OPMODE_HOST_TRIGGER) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to set operating mode\r\n");
 *     return;
 * }
 *
 * // Set to periodic trigger CMP mode
 * if(endat3_set_operating_mode(handle, ENDAT3_OPMODE_PERIODIC_CMP) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to set operating mode\r\n");
 *     return;
 * }
 * \endcode
 */
int32_t endat3_set_operating_mode(endat3_handle handle, uint8_t opmode);

/**
 * \brief Get current operating mode
 *
 * Retrieves the current firmware operating mode.
 *
 * \param handle EnDAT3 handle
 * \param opmode Pointer to store operating mode (0 = periodic CMP, 1 = host trigger, 2 = periodic CAP)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or opmode is NULL
 */
int32_t endat3_get_operating_mode(endat3_handle handle, uint8_t *opmode);

/**
 * \brief Configure IEP CAP event number in PRU DMEM
 *
 * \param handle EnDAT3 handle
 * \param event_num CAP event number (0-7) to use for periodic triggering
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL or event_num > 7
 *
 * \note This only updates the DMEM configuration. The operating mode must be set to
 *       \ref ENDAT3_OPMODE_PERIODIC_CAP separately via endat3_set_operating_mode(). This
 *       function does NOT configure IEP hardware registers.
 */
int32_t endat3_config_iep_cap_event(endat3_handle handle, uint8_t event_num);

/**
 * \brief Configure IEP CMP event number in PRU DMEM
 *
 * \param handle EnDAT3 handle
 * \param event_num CMP event number (0-15) to use for periodic triggering
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL or event_num > 15
 *
 * \note This only updates the DMEM configuration. The operating mode must be set to
 *       \ref ENDAT3_OPMODE_PERIODIC_CMP separately via endat3_set_operating_mode(). This
 *       function does NOT configure IEP hardware registers.
 */
int32_t endat3_config_iep_cmp_event(endat3_handle handle, uint8_t event_num);

/**
 * \brief Release start trigger to firmware
 *
 * Signals the firmware to begin processing by setting the start_trigger flag.
 * The firmware will process the command based on the current operating mode:
 * - In host mode: processes the command immediately
 * - In periodic mode: waits for the next IEP CMP/CAP event
 *
 * \param handle EnDAT3 handle
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *         **CRITICAL**: Check return value - if this fails, command will never be processed by firmware
 *
 * \code
 * // Release trigger to firmware
 * if(endat3_release_start_trigger(handle) != ENDAT3_SUCCESS)
 * {
 *     DebugP_log("ERROR: Failed to release start trigger\r\n");
 *     return;
 * }
 * \endcode
 */
int32_t endat3_release_start_trigger(endat3_handle handle);

/**
 * \brief Clear start trigger flag
 *
 * Clears the start_trigger flag after the firmware has processed the command.
 * This prepares the interface for the next command.
 *
 * \param handle EnDAT3 handle
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle is NULL
 *
 * \note Check return value to ensure trigger was cleared properly
 */
int32_t endat3_clear_start_trigger(endat3_handle handle);

/**
 * \brief Get current start trigger status
 *
 * Retrieves the current state of the start_trigger flag.
 *
 * \param handle EnDAT3 handle
 * \param trigger_status Pointer to store trigger status (1 if set, 0 if cleared)
 * \return ENDAT3_SUCCESS (0) on success
 *         ENDAT3_ERR_INVALID_INPUT (-1) if handle or trigger_status is NULL
 */
int32_t endat3_get_start_trigger_status(endat3_handle handle, uint8_t *trigger_status);

/** \} */ /* End of ENDAT3_API_MODULE */

#ifdef __cplusplus
}
#endif

#endif /* ENDAT3_DRV_H_ */