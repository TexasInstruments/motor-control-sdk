/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

/* EnDAT3 Driver Error Codes */
#define ENDAT3_ERR_HPF_CRC_FAIL     -5
#define ENDAT3_ERR_LPH_CRC_FAIL     -4
#define ENDAT3_ERR_TIMEOUT          -3
#define ENDAT3_ERR_RX_FAIL          -2
#define ENDAT3_ERR_INVALID_PARAM    -1

/* Encoder busy/error status defines */
#define ENCODER_IDLE 0x0
#define ENCODER_BUSY 0x1
#define ENCODER_ERROR 0x2

/* Default clock configuration values */
#define ENDAT3_DIV_FACTOR_NORMAL        11      /* Divide factor for normal clock (300/25=12) */
#define ENDAT3_DIV_FACTOR_OVERSAMPLED   2       /* Divide factor for over sampled clock 300/(12.5*8)=3 */
#define ENDAT3_PRU_CLOCK_TYPE           0x10    /* PRU clock type */
#define ENDAT3_UART_CLOCK_TYPE          0x0     /* UART clock type */
#define ENDAT3_OVERSAMPLE_RATE          7       /* Over sample rate 8 (actual = 7+1) */

/* Start bit polarity configuration */
#define ENDAT3_START_BIT_POL_0          0x0     /* Start bit polarity 0 */
#define ENDAT3_START_BIT_POL_1          0x8     /* Start bit polarity 1 */
#define DEFAULT_SB_POLARITY             ENDAT3_START_BIT_POL_0

/* Register configuration values */
#define ENDAT3_ENABLE_BIT               (0x1 << 26)     /* EnDat3 enable bit */
#define ENDAT3_CTR_EN                   (1 << 3)        /* Counter enable bit */

/* Buffer and data size definitions */
#define HPF_DATA_SIZE                   6       /* High Priority Frame data size in bytes */
#define LPF_DATA_SIZE                   6       /* Low Priority Frame data size in bytes */
#define MAX_LPF_COUNT                   8       /* Maximum number of LPF frames */
#define TEMP_BUFFER_SIZE                16      /* Temporary buffer size */
#define LUT_SIZE                        80      /* Lookup table size */
#define RX_BUFFER_SIZE                  64      /* Receive buffer size */
#define TX_BUFFER_SIZE                  24      /* Transmit buffer size */
#define BG_DATA_SIZE                    6       /* Background data array size */
#define TEMP2_BUFFER_SIZE               4       /* Additional temporary storage size */

/* Reset command data values */
#define ENDAT3_RESET_HARD               0x01    /**< Hard reset command data */
#define ENDAT3_RESET_SOFT               0x00    /**< Soft reset command data */

/* Clear command flag bits */
#define ENDAT3_CLEAR_F                  0x01    /**< Clear F (error) flag */
#define endat3_CLEAR_F                  0x01    /**< Clear F (error) flag - alternate naming */
#define ENDAT3_CLEAR_W                  0x02    /**< Clear W (warning) flag */
#define ENDAT3_CLEAR_REF                0x04    /**< Clear reference flag */

/* Data rate configuration values */
#define ENDAT3_RATE_12_5MBPS            0x01    /**< 12.5 Mbps data rate */
#define ENDAT3_RATE_25MBPS              0x02    /**< 25 Mbps data rate */

/* Bus initialization command types */
#define ENDAT3_BUSINIT_RESET_ADDR       0x01    /**< Bus init with address reset */
#define ENDAT3_BUSINIT_KEEP_ADDR        0x00    /**< Bus init keeping addresses */

/* Configuration computation macros */
#define ENDAT3_COMPUTE_TX_CFG(clock_type, load_share, div_factor) \
    ((clock_type) | ((load_share) << 11) | ((div_factor) << 16))

#define ENDAT3_COMPUTE_RX_CFG(clock_type, div_factor, oversample, sb_pol) \
    ((clock_type) | ((div_factor) << 16) | (oversample) | (sb_pol))
/**
 * \name Protocol Constants
 * \{
 */
#define ENDAT3_PRE_LENGTH     25  /**< Minimum preamble length in halfbits */
#define ENDAT3_POST_LENGTH    4   /**< Postamble length in halfbits */
#define ENDAT3_MAX_CHANNELS   3   /**< Maximum number of supported channels per PRU Slice */
#define ENDAT3_RX_OVERSAMPLING_RATE 8 /**< Oversampling rate for RX */
/** \} */

/**
 * \brief Clock configuration structure
 * 
 * Defines the configuration parameters for the EnDAT3 clock settings.
 */
typedef struct {
    uint32_t rx_clk_source;   /**< RX clock source */
    uint32_t tx_clk_source;   /**< TX clock source */
    uint32_t tx_div;          /**< TX clock divider */
    uint32_t rx_div;          /**< RX clock divider */
    uint32_t rx_os_rate;      /**< RX oversampling rate */
} endat3_clk_cfg_t;

/**
 * \brief EnDat3 Clock Configuration Structure
 * 
 * This structure contains clock configuration parameters
 * for EnDat3 timing setup.
 */
typedef struct {
    uint32_t div_factor_normal;         /**< Divide factor for normal clock (300/25=12) */
    uint32_t div_factor_oversampled;    /**< Divide factor for over sampled clock 300/(12.5*8)=3 */
    uint32_t pru_clock_type;            /**< PRU clock type configuration */
    uint32_t uart_clock_type;           /**< UART clock type configuration */
    uint32_t clock_type;                /**< Selected clock type */
    uint32_t oversample_rate;           /**< Over sample rate (actual rate = value + 1) */
    uint32_t sb_polarity; /**< Start bit polarity (0 or 1) */
} endat3_ClockConfig_t;
/**
 * \brief EnDat3 Register Configuration Structure
 * 
 * This structure contains all register configuration values
 * for EnDat3 PRU-ICSS setup.
 */
typedef struct {
    uint32_t endat3_enable;             /**< EnDat3 enable bit configuration */
    uint32_t tx_config;                 /**< TX configuration register value */
    uint32_t rx_config;                 /**< RX configuration register value */
    uint32_t counter_enable;            /**< Counter enable configuration */
    uint32_t load_share_mode;           /**< Load share mode flag */
} endat3_RegisterConfig_t;

/**
 * \brief Complete EnDat3 Configuration Structure
 * 
 * This structure combines clock and register configurations
 * for complete EnDat3 setup.
 */
typedef struct {
    endat3_ClockConfig_t clock_config;      /**< Clock configuration parameters */
    endat3_RegisterConfig_t reg_config;     /**< Register configuration parameters */
} endat3_Config_t;

/**
 * \brief High Priority Frame structure
 * 
 * The High Priority Frame (HPF) contains position data and critical status information.
 */
typedef struct {
    uint8_t data[HPF_DATA_SIZE];          /**< 48-bit data payload */
    uint8_t status;           /**< Status byte */
    uint8_t crc;              /**< CRC checksum */
} endat3_hpf_t;

/**
 * \brief Low Priority Header structure
 * 
 * The Low Priority Header (LPH) contains information about following Low Priority Frames.
 */
typedef struct {
    uint8_t status;           /**< Status byte */
    uint8_t num_lpf ;             /**< Number of LPF frames */
    uint8_t reserved;         /**< Reserved byte */
    uint8_t crc;              /**< CRC checksum */
} endat3_lph_t;

/**
 * \brief Low Priority Frame structure
 * 
 * Low Priority Frames (LPF) contain additional data and diagnostic information.
 */
typedef struct {
    uint8_t status;           /**< Status byte with FID */
    uint8_t data[LPF_DATA_SIZE];          /**< 48-bit data payload */
    uint8_t crc;              /**< CRC checksum */
} endat3_lpf_t;

/**
 * \brief EnDAT3 Interface structure
 * 
 * Main structure for storing communication buffers and status information.
 */
typedef struct 
{
    /* Communication buffers */
    uint32_t LUT[LUT_SIZE];                 /**< Lookup table */
    uint8_t rx_buffer[RX_BUFFER_SIZE];            /**< Receive buffer */
    
    /* Frame structures */
    endat3_hpf_t hpf;                 /**< High Priority Frame */
    endat3_lph_t lph;                 /**< Low Priority Header */
    endat3_lpf_t lpf[MAX_LPF_COUNT];  /**< Low Priority Frames array */
    
    /* Status flags */
    uint8_t connected;                /**< Connection status */
    uint8_t busy;                     /**< Transfer in progress */
    uint8_t comm_cycle_flag;          /**< Communication cycle flag */
    uint8_t reserved1;                /**< Reserved for alignment */
    
    /* Frame control */
    uint32_t expected_tx_frames_count; /**< Expected number of TX frames */
    uint32_t current_tx_frame_offset; /**< Current TX frame offset */
    
    /* Data buffers */
    uint8_t tx_buffer[TX_BUFFER_SIZE];            /**< Transmit buffer */
    uint32_t bg_data[BG_DATA_SIZE];              /**< Background data */
    
    /* Timing and operation codes */
    uint32_t propagation_time;        /**< Measured propagation time */
    uint32_t foreground_op_code;      /**< Foreground operation code */
    uint32_t background_op_code;      /**< Background operation code */
    
    /* Periodic trigger configuration */
    uint8_t opmode_config;            /**< Operating mode: 0=periodic, 1=host */
    uint8_t reserved2;                /**< Reserved for alignment */
    uint8_t reserved3;                /**< Reserved for alignment */
    uint8_t reserved4;                /**< Reserved for alignment */

    /* Delay Cycle Configuration (frequency-independent timing) */
    /* These values are calculated by R5F based on actual PRU frequency */
    uint32_t delay_tx_start_1;        /**< TX start delay 1 in PRU cycles */
    uint32_t delay_tx_start_2;        /**< TX start delay 2 in PRU cycles */
    uint32_t delay_tx_start_3;        /**< TX start delay 3 in PRU cycles */
    uint32_t delay_sampling;          /**< Sampling delay in PRU cycles */
    uint32_t delay_10ms;              /**< 10ms delay in PRU cycles */

    /* Trigger Control */
    uint8_t start_trigger;            /**< Start trigger: released by R5F core based on operating mode */
} endat3_Interface;

/**
 * \brief Low Priority Header status values
 * 
 * Status values that can appear in the LPH status byte.
 */
typedef enum {
    LPH_STATUS_IDLE     = 0,  /**< IDLE state */
    LPH_STATUS_RX_START = 1,  /**< RX_START state - beginning of reception */
    LPH_STATUS_RX_LAST  = 2,  /**< RX_LAST state - last reception frame */
    LPH_STATUS_BUSY     = 3   /**< BUSY state - processing in progress */
} LPH_Status_t;

/**
 * \brief EnDAT3 Background Request (BGREQ) OpCodes
 * 
 * Operation codes for background processing operations.
 */
typedef enum {
    endat3_BGREQ_NOP          = 0x01,  /**< No operation */
    endat3_BGREQ_READ         = 0x02,  /**< Read from encoder memory */
    endat3_BGREQ_WRITE        = 0x03,  /**< Write to encoder memory */
    endat3_BGREQ_RECONFIGURE  = 0x04,  /**< Reconfigure parameters */
    endat3_BGREQ_AUTH         = 0x80,  /**< Authentication */
    endat3_BGREQ_PROTECT      = 0x81,  /**< Set protection */
    endat3_BGREQ_RESERVED     = 0x82,  /**< Reserved for future use */
    endat3_BGREQ_SETPASS      = 0x83,  /**< Set password */
    endat3_BGREQ_LOCATE       = 0x84   /**< Locate function */
} endat3_BgReqOpCode_t;

/**
 * \brief EnDAT3 Foreground Request (REQ.CODE) OpCodes
 * 
 * Operation codes for foreground request commands.
 */
typedef enum {
    endat3_REQ_DATA0      = 0x00,  /**< Activate LPF send list 0 */
    endat3_REQ_DATA1      = 0x01,  /**< Activate LPF send list 1 */
    endat3_REQ_DATA2      = 0x02,  /**< Activate LPF send list 2 */
    endat3_REQ_DATA3      = 0x03,  /**< Activate LPF send list 3 */
    endat3_REQ_DATA4      = 0x04,  /**< Activate LPF send list 4 */
    endat3_REQ_DATA5      = 0x05,  /**< Activate LPF send list 5 */
    endat3_REQ_DATA6      = 0x06,  /**< Activate LPF send list 6 */
    endat3_REQ_DATA7      = 0x07,  /**< Activate LPF send list 7 */
    endat3_REQ_DATA       = 0x08,  /**< Data with BGD */
    endat3_REQ_DATANOP    = 0x09,  /**< Data without BGD */
    endat3_REQ_RESET      = 0x0B,  /**< Encoder reset */
    endat3_REQ_CLEAR      = 0x0C,  /**< Resetting of states */
    endat3_REQ_ECHO       = 0x0E,  /**< Echo for measuring propagation time */
    endat3_REQ_RATE       = 0x10,  /**< Set data transfer rate */
    endat3_REQ_HELLO      = 0x22,  /**< Switch to EnDat 3 mode */
    endat3_REQ_RESERVED   = 0x40,  /**< Reserved */
    endat3_REQ_BUSBC      = 0x80,  /**< Bus command for broadcast */
    endat3_REQ_BUSP2P     = 0x81,  /**< Bus command for point-to-point communication */
    endat3_REQ_BUSINIT    = 0x82,  /**< Initialization of a bus setup */
    endat3_REQ_FORCE      = 0x90   /**< Forced dynamic sampling */
} endat3_ReqCode_t;

/**
 * \brief EnDAT3 Menu Options
 * 
 * Menu options for user interface commands.
 */
typedef enum {
    ENDAT3_MENU_DATA0 = 1,    /**< Start with 1 for better user experience */
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
} endat3_MenuOption_t;
/**
 * \brief HPF STATUS bits
 * 
 * Individual bits in the HPF status byte.
 */
typedef enum {
    endat3_HPF_STATUS_F       = 0x01,  /**< Bit 0: Collective error bit */
    endat3_HPF_STATUS_W       = 0x02,  /**< Bit 1: Collective warning bit */
    endat3_HPF_STATUS_HPFV    = 0x04,  /**< Bit 2: Validity of HPF data */
    endat3_HPF_STATUS_RM      = 0x08,  /**< Bit 3: Availability of absolute value */
    endat3_HPF_STATUS_ERR_REQ = 0x10   /**< Bit 4: Request code not supported */
} endat3_HpfStatusBits_t;

/**
 * \brief LPH Status Flags
 * 
 * Status flags for the Low Priority Header (LPH).
 */
typedef enum {
    endat3_LPH_STATUS_IDLE        = 0x0,  /**< Idle state, no background operation in progress */
    endat3_LPH_STATUS_RX_START    = 0x1,  /**< First frame of a multi-frame background response */
    endat3_LPH_STATUS_RX_LAST     = 0x2,  /**< Last frame of a multi-frame background response */
    endat3_LPH_STATUS_BUSY        = 0x3,  /**< Background operation in progress */
    endat3_LPH_BG_ERR_EXEC        = 0x4,  /**< Error during background operation execution */
    endat3_LPH_BG_BUSY            = 0x8,  /**< Background processor is busy */
    endat3_LPH_BG_RTX_ERROR       = 0x10, /**< Background transmit/receive error */
    endat3_LPH_STATE_MASK         = 0x3   /**< Mask for extracting LPH state (bits 0-1) */
} endat3_LphStatus_t;

/**
 * \brief EnDAT3 private structure
 * 
 * Main driver handle structure that contains all the context for an EnDAT3 interface.
 */
typedef struct endat3_priv {
    PRUICSS_Handle icssHandle;     /**< PRUICSS_Handle for icssg0 or icssg1 instance */
    uint32_t icssCore;             /**< PRUICSS core identifier (Check PRUICSS_PRU0 and other available macros) */
    uint32_t *baseMemAddr;         /**< Base Memory Address for endat3 channel configuration */
    endat3_Interface *endat3Interface; /**< Pointer to interface communication structure */
    uint32_t resolution;           /**< Encoder resolution */
    uint32_t channel;              /**< Current channel */
    uint32_t baudrate;             /**< Communication baudrate */
    void *pruicss_cfg;               /**< pruicss CFG space pointer */
    void *pruicss_iep;               /**< pruicss IEP pointer */
    uint64_t pru_clock; /**<PRU CORE Clock*/
    uint8_t rx_clock_source; /*3 channel Peripheral RX clock source*/
    uint8_t tx_clock_source; /*3 channel Peripheral TX clock source*/
} endat3_priv_t;
/**
 * \brief Extended clock configuration structure for legacy compatibility
 */
typedef struct {
    uint32_t rx_div;        /**< RX clock divider */
    uint32_t tx_div;        /**< TX clock divider */
    uint32_t rx_div_attr;   /**< RX divider attributes */
} endat_clk_cfg_ext_t;

/**
 * \brief EnDAT3 Driver Handle
 * 
 * Opaque pointer to the EnDAT3 private structure.
 */
typedef struct endat3_priv *endat3_Handle;

/**
 * \brief EnDat3 Error Codes
 * 
 * Error codes used in foreground and background communication.
 */
typedef enum {
    /* General errors */
    endat3_ERR_UNKNOWN                 = 0x0000,  /**< The cause of the error is unknown */

    /* Foreground errors (0x0001-0x0FFF) */
    endat3_FGERR_RECONFIGURE           = 0x0001,  /**< Device is in configuration as a result of RECONFIGURE */
    endat3_FGERR_ECHO                  = 0x0002,  /**< An ECHO is being responded to */
    endat3_FGERR_INVALID_FID           = 0x0100,  /**< An invalid FID was configured */
    endat3_FGERR_DUPLICATE_FID         = 0x0101,  /**< FID was selected more than once during the cycle */
    endat3_FGERR_INVALID_DATA          = 0x0200,  /**< LPF is supported, but invalid data were delivered internally */
    endat3_FGERR_INT_TRM               = 0x0201,  /**< LPF is supported but currently unavailable */
    endat3_FGERR_NO_SENSOR_DATA        = 0x0300,  /**< Sensor box data not available */

    /* Background errors - Usage errors (0x1100-0x11FF) */
    endat3_BGERR_USAGE                 = 0x1100,  /**< Generic operator error */
    endat3_BGERR_USAGE_OPCODE          = 0x1101,  /**< Invalid or unsupported command code */
    endat3_BGERR_USAGE_ARGUMENTS       = 0x1102,  /**< Invalid arguments */
    endat3_BGERR_USAGE_SEQUENCE        = 0x1103,  /**< Invalid command sequence */
    endat3_BGERR_USAGE_ACCESS_DENIED   = 0x1104,  /**< Access denied; insufficient user level */
    endat3_BGERR_USAGE_MEM_ADDRESS     = 0x1105,  /**< Access to invalid address */
    endat3_BGERR_USAGE_NO_BG           = 0x1106,  /**< Encoder does not support background processing */

    /* Background errors - Internal errors (0x1200-0x12FF) */
    endat3_BGERR_INTERNAL              = 0x1200,  /**< Generic exception error in the encoder */
    endat3_BGERR_INTERNAL_MEMORY       = 0x1201,  /**< Exception error when accessing memory */
    endat3_BGERR_INTERNAL_CONFIG       = 0x1202   /**< Exception error: configuration invalid */
} endat3_ErrorCode_t;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \ brief Open EnDAT3 handle for the specified core
 * 
 * This function initializes and returns a handle to the EnDAT3 interface
 * associated with the specified PRU-ICSS core. It supports both load-share
 * and non-load-share modes.
 *
 * \ param icssHandle PRUICSS_Handle for the ICSS instance
 * \ param icssCore Core to map in ICSSG instance
 * \ param pruMode 0 for load share mode disabled, 1 for load share mode enabled
 *
 * \ return endat3_Handle on success, NULL on error
 */
endat3_Handle endat3_open(PRUICSS_Handle icssHandle, uint32_t icssCore, uint8_t pruMode);

/**
 * \ brief Send EnDAT3 command
 * 
 * Prepares and sends an EnDAT3 command with the specified command code
 * and number of frames. Validates input parameters and checks for buffer
 * overflow conditions before transmission.
 *
 * \ param priv Private structure pointer
 * \ param cmd Command code
 * \ param frames Number of frames to transmit
 *
 * \ return SystemP_SUCCESS on success, SystemP_FAILURE on error
 *         (invalid handle, buffer overflow, or invalid data array)
 */
int32_t endat3_send_command(endat3_Handle priv, uint8_t cmd, uint8_t frames);

/**
 * \ brief Receive EnDAT3 response
 * 
 * Receives and processes EnDAT3 response frames, verifying CRC integrity
 * and parsing the data into appropriate structures.
 *
 * \ param priv Private structure pointer
 *
 * \ return 0 on success, negative on error, 1 for special status
 */
int32_t endat3_receive_response(endat3_Handle priv);

/**
 * \ brief Calculate CRC for EnDAT3 frame
 * 
 * Calculates the CRC-8 checksum for EnDAT3 frames using the
 * standard EnDAT3 CRC algorithm.
 *
 * \ param data Data buffer
 * \ param len Data length
 *
 * \ return Calculated CRC value
 */
uint8_t endat3_calculate_crc(uint8_t *data, uint32_t len);

/**
 * \ brief Configure EnDAT3 for host trigger mode
 * 
 * Sets up the EnDAT3 interface for host-triggered operation,
 * where communication is initiated by the host processor.
 *
 * \ param priv Private structure pointer
 */
void endat3_config_host_trigger(endat3_Handle priv);

/**
 * \ brief Configure EnDAT3 for periodic trigger mode
 * 
 * Sets up the EnDAT3 interface for periodic triggering,
 * where communication occurs at regular intervals.
 *
 * \ param priv Private structure pointer
 */
void endat3_config_periodic_trigger(endat3_Handle priv);

/**
 * \ brief Generates memory image
 * 
 * Creates a memory image for the PRU firmware based on the
 * current EnDAT3 configuration.
 *
 * \ param endat3Handle EnDAT3 handle
  * \ param icssgHandle PRUICSS handle
 */
void endat3_generate_memory_image(endat3_Handle endat3Handle, PRUICSS_Handle icssgHandle);

/**
 * \ brief Process endat3 frame with variable length
 * 
 * Verifies the integrity of an endat3 frame by calculating its CRC
 * and comparing it with the received CRC byte. The CRC byte is assumed
 * to be the last byte in the buffer.
 *
 * \ param priv endat3 handle for the interface
 * \ param buffer Data buffer containing the complete frame including CRC byte
 * \ param length Total length of the buffer including CRC byte
 * \ return uint8_t 1 if the calculated CRC matches the received CRC (frame is valid),
 *                 0 if CRC check failed or buffer is too short
 */
uint8_t endat3_process_frame(endat3_Handle priv, uint8_t *buffer, uint32_t length);


/**
 * \brief Initialize EnDat3 configuration with default values
 *
 * This function initializes the EnDat3 configuration structure with default
 * values for all clock and register parameters. The load share mode can be
 * specified as a parameter.
 *
 * \param config         [OUT] Pointer to EnDat3 configuration structure to initialize
 * \param load_share_mode [IN]  Load share mode flag (0 or 1)
 *
 * \return void
 *
 * \pre config must point to a valid endat3_Config_t structure
 * \post config structure is initialized with default values
 */
void endat3_initConfig(endat3_Config_t *config, uint32_t load_share_mode);

/**
 * \brief Configure EnDat3 PRU registers using configuration structure
 *
 * This function configures the necessary PRU-ICSS registers for EnDat3 operation
 * using values from the provided configuration structure. It writes to the
 * GPCFG1, EDPRU1TXCFGREGISTER, and EDPRU1RXCFGREGISTER registers.
 *
 * \param pru_cfg_base [IN] Pointer to PRU configuration register base address
 * \param config       [IN] Pointer to EnDat3 configuration structure
 * \param pruSlice
 * \return void
 *
 * \pre pru_cfg_base must point to valid PRU configuration register space
 * \pre config must point to a valid, initialized endat3_Config_t structure
 * \post PRU registers are configured for EnDat3 operation
 */
void endat3_configurePruRegisters(void *pru_cfg_base, const endat3_Config_t *config, uint32_t pruSlice);

/**
 * \brief Create EnDat3 configuration and configure PRU registers (convenience function)
 *
 * This is a convenience function that initializes the configuration structure
 * with default values and configures the PRU registers in one call. It combines
 * the functionality of endat3_initConfig() and endat3_configurePruRegisters().
 *
 * \param pru_cfg_base    [IN] Pointer to PRU configuration register base address
 * \param load_share_mode [IN] Load share mode flag (0 or 1)
 * \param pruSlice
 * \return void
 *
 * \pre pru_cfg_base must point to valid PRU configuration register space
 * \post PRU registers are configured for EnDat3 operation with default values
 */
void endat3_configureWithDefaults(void *pru_cfg_base, uint32_t load_share_mode,uint32_t pruSlice);

/**
 * \brief Calculate and set frequency-independent delay cycles in firmware interface
 *
 * This function calculates delay values in PRU cycles based on the actual PRU
 * core frequency and stores them in the endat3_Interface structure. The firmware
 * loads these values at runtime, making the delays frequency-independent.
 *
 * \param handle         [IN] EnDAT3 handle containing interface pointer
 * \param pru_freq_hz    [IN] PRU core frequency in Hz (e.g., 300000000 for 300MHz)
 *
 * \return SystemP_SUCCESS on success, SystemP_FAILURE if handle is invalid
 *
 * \note This function should be called after endat3_open() and before starting communication.
 *       The delay values are calculated based on protocol timing requirements:
 *       - delay_tx_start_1: Short delay after clock reset (~65us)
 *       - delay_tx_start_2: Long delay after HELLO transmission (~524ms)
 *       - delay_tx_start_3: Delay between frame transmissions (~131ms)
 *       - delay_sampling: Sampling delay before RX enable (~37 cycles minimum)
 *       - delay_10ms: Special timing delay for EEPROM writes (10ms)
 *
 * \code
 * endat3_Handle handle = endat3_open(pruicss_handle, PRUICSS_PRU0, 1);
 * endat3_setDelayCycles(handle, 300000000);  // 300MHz PRU
 * \endcode
 */
int32_t endat3_setDelayCycles(endat3_Handle handle, uint64_t pru_freq_hz);

/**
 * \ brief Configure ENDAT mode for the specified PRU-ICSS slice
 * 
 * This function configures the ENDAT communication mode by writing to the
 * appropriate PRU-ICSS general purpose configuration register based on the
 * specified slice.
 * 
 * \ param priv         Handle to the endat3 instance containing PRU-ICSS configuration
 * \ param pruicss_slicex  PRU-ICSS slice selector:
 *                       - 0: Configure PRU slice 0 (GPCFG0_REG)
 *                       - 1: Configure PRU slice 1 (GPCFG1_REG)
 * 
 * \ return None
 * 
 * \ note This function writes value 4 to the upper byte (offset +3) of the
 *       respective GPCFG register to enable ENDAT mode.
 */
/**
 * \ brief Configure EnDAT mode in PRU configuration registers
 *
 * Configures the PRU-ICSS configuration registers to enable EnDAT mode
 * operation for the specified PRU slice.
 *
 * \ param priv endat3 handle
 * \ param pruicss_slicex PRU slice selection (0 or 1)
 * \ return void
 */
void endat3_config_endat_mode(endat3_Handle priv, uint8_t pruicss_slicex);

/**
 * \ brief Enable load share mode for EnDAT3
 *
 * Enables load share mode in the PRU-ICSS configuration, allowing multiple
 * PRU cores to share the EnDAT3 communication interface.
 *
 * \ param pruCfg Pointer to PRU configuration register base address
 * \ param pruSlice PRU slice selector (0 or 1)
 * \ return void
 */
void endat3_enable_load_share_mode(void *pruCfg, uint32_t pruSlice);

/**
 * \ brief Get error description string
 * 
 * Returns a human-readable description for the given EnDAT3 error code.
 * Useful for logging and debugging error conditions.
 *
 * \ param error_code The error code to get description for
 * \ return Pointer to error description string
 * 
 * \ code
 * endat3_ErrorCode_t error = endat3_getErrorCode(handle);
 * const char* desc = endat3_getErrorDescription(error);
 * DebugP_log("Error: %s\r\n", desc);
 * \ endcode
 */
const char* endat3_getErrorDescription(endat3_ErrorCode_t error_code);

/**
 * \ brief Get recommended action for error code
 * 
 * Returns a recommended action string for the given EnDAT3 error code.
 * Helps users understand what to do when an error occurs.
 *
 * \ param error_code The error code to get recommended action for
 * \ return Pointer to recommended action string
 * 
 * \ code
 * endat3_ErrorCode_t error = endat3_getErrorCode(handle);
 * const char* action = endat3_getErrorAction(error);
 * DebugP_log("Recommended action: %s\r\n", action);
 * \ endcode
 */
const char* endat3_getErrorAction(endat3_ErrorCode_t error_code);

/**
 * \ brief Handle background command request
 * 
 * Processes and prepares a background command request for transmission.
 * Handles various background operations like READ, WRITE, AUTH, PROTECT, etc.
 * Formats the command data according to the operation code and parameters.
 *
 * \ param endat3Handle EnDAT3 handle
 * \ param index Background data array index to start writing
 * \ param frame_cnt Number of frames for this command
 * \ param op_code Background operation code (endat3_BgReqOpCode_t)
 * \ param addr_msb Address MSB (for memory operations)
 * \ param addr_lsb Address LSB (for memory operations)
 * \ param data Operation-specific data
 * \ return void
 * 
 * \ code
 * // Example: READ operation
 * endat3_handle_background_command_request(handle, 0, 4, 
 *     endat3_BGREQ_READ, 0x00, 0x0000, 1);  // Read 1 word from address 0x000000
 * \ endcode
 */
void endat3_handle_background_command_request(endat3_Handle endat3Handle, int index, int frame_cnt, uint8_t op_code, uint32_t addr_msb, uint32_t addr_lsb, uint32_t data);

/* ========================================================================== */
/*                    HPF (High Priority Frame) Access APIs                   */
/* ========================================================================== */

/**
 * \ brief Get HPF status byte
 * 
 * Retrieves the status byte from the High Priority Frame, which contains
 * critical status flags including F, W, HPFV, RM, and ERR_REQ bits.
 *
 * \ param handle EnDAT3 handle
 * \ return HPF status byte, or 0 if handle is NULL
 * 
 * \ code
 * uint8_t status = endat3_getHpfStatus(handle);
 * if (status & endat3_HPF_STATUS_F) {
 *     // Handle error condition
 * }
 * \ endcode
 */
uint8_t endat3_getHpfStatus(endat3_Handle handle);

/**
 * \ brief Get HPF data array
 * 
 * Retrieves the 6-byte data payload from the High Priority Frame.
 * The data is copied to the provided buffer.
 *
 * \ param handle EnDAT3 handle
 * \ param data Buffer to store HPF data (must be at least 6 bytes)
 * \ return Number of bytes copied (6) on success, -1 if handle or data is NULL
 * 
 * \ code
 * uint8_t hpf_data[6];
 * int32_t bytes = endat3_getHpfData(handle, hpf_data);
 * if (bytes == 6) {
 *     // Process HPF data
 * }
 * \ endcode
 */
int32_t endat3_getHpfData(endat3_Handle handle, uint8_t *data);

/**
 * \ brief Get HPF CRC value
 * 
 * Retrieves the CRC checksum from the High Priority Frame.
 *
 * \ param handle EnDAT3 handle
 * \ return HPF CRC value, or 0 if handle is NULL
 */
uint8_t endat3_getHpfCrc(endat3_Handle handle);

/**
 * \ brief Get HPF data as 64-bit value
 * 
 * Retrieves the HPF data payload as a single 64-bit unsigned integer.
 * Useful for position data extraction. Data is packed in little-endian format.
 *
 * \ param handle EnDAT3 handle
 * \ return HPF data as uint64_t, or 0 if handle is NULL
 * 
 * \ code
 * uint64_t position = endat3_getHpfDataAsU64(handle);
 * uint32_t single_turn = position & 0x1FFF;  // Extract 13-bit single turn
 * \ endcode
 */
uint64_t endat3_getHpfDataAsU64(endat3_Handle handle);

/**
 * \ brief Check if HPF data is valid
 * 
 * Checks the HPFV bit in the HPF status to determine if the data is valid.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if HPF data is valid (HPFV bit set), 0 otherwise
 */
uint8_t endat3_isHpfDataValid(endat3_Handle handle);

/**
 * \ brief Check if HPF has error flag set
 * 
 * Checks the F bit in the HPF status to determine if an error is present.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if error flag is set, 0 otherwise
 */
uint8_t endat3_hasHpfError(endat3_Handle handle);

/**
 * \ brief Check if HPF has warning flag set
 * 
 * Checks the W bit in the HPF status to determine if a warning is present.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if warning flag is set, 0 otherwise
 */
uint8_t endat3_hasHpfWarning(endat3_Handle handle);

/**
 * \ brief Check if absolute value is available
 * 
 * Checks the RM bit in the HPF status to determine if absolute position
 * value is available.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if absolute value is available (RM bit set), 0 otherwise
 */
uint8_t endat3_hasAbsoluteValue(endat3_Handle handle);

/* ========================================================================== */
/*                    LPH (Low Priority Header) Access APIs                   */
/* ========================================================================== */

/**
 * \ brief Get LPH status byte
 * 
 * Retrieves the status byte from the Low Priority Header, which contains
 * the communication state and error flags.
 *
 * \ param handle EnDAT3 handle
 * \ return LPH status byte, or 0 if handle is NULL
 */
uint8_t endat3_getLphStatus(endat3_Handle handle);

/**
 * \ brief Get number of LPF frames
 * 
 * Retrieves the number of Low Priority Frames indicated in the LPH.
 *
 * \ param handle EnDAT3 handle
 * \ return Number of LPF frames (0-15), or 0 if handle is NULL
 */
uint8_t endat3_getLphnum_lpf (endat3_Handle handle);

/**
 * \ brief Get LPH CRC value
 * 
 * Retrieves the CRC checksum from the Low Priority Header.
 *
 * \ param handle EnDAT3 handle
 * \ return LPH CRC value, or 0 if handle is NULL
 */
uint8_t endat3_getLphCrc(endat3_Handle handle);

/**
 * \ brief Get LPH communication state
 * 
 * Extracts the communication state from the LPH status byte (bits 0-1).
 *
 * \ param handle EnDAT3 handle
 * \ return LPH_Status_t enum value (IDLE, RX_START, RX_LAST, BUSY)
 * 
 * \ code
 * LPH_Status_t state = endat3_getLphState(handle);
 * switch(state) {
 *     case LPH_STATUS_IDLE:
 *         // Ready for new command
 *         break;
 *     case LPH_STATUS_BUSY:
 *         // Background operation in progress
 *         break;
 * }
 * \ endcode
 */
LPH_Status_t endat3_getLphState(endat3_Handle handle);

/**
 * \ brief Check if background operation has error
 * 
 * Checks the BG.ERR_EXEC bit in the LPH status.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if background error is present, 0 otherwise
 */
uint8_t endat3_hasBgError(endat3_Handle handle);

/**
 * \ brief Check if background processor is busy
 * 
 * Checks the BG.BUSY bit in the LPH status.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if background processor is busy, 0 otherwise
 */
uint8_t endat3_isBgBusy(endat3_Handle handle);

/**
 * \ brief Check if background RTX error occurred
 * 
 * Checks the BG.RTX_ERROR bit in the LPH status.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if background transmit/receive error occurred, 0 otherwise
 */
uint8_t endat3_hasBgRtxError(endat3_Handle handle);

/* ========================================================================== */
/*                    LPF (Low Priority Frame) Access APIs                    */
/* ========================================================================== */

/**
 * \ brief Get LPF status byte for specific frame
 * 
 * Retrieves the status byte from a specific Low Priority Frame.
 *
 * \ param handle EnDAT3 handle
 * \ param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \ return LPF status byte, or 0 if handle is NULL or index is invalid
 */
uint8_t endat3_getLpfStatus(endat3_Handle handle, uint8_t index);

/**
 * \ brief Get LPF data array for specific frame
 * 
 * Retrieves the 6-byte data payload from a specific Low Priority Frame.
 *
 * \ param handle EnDAT3 handle
 * \ param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \ param data Buffer to store LPF data (must be at least 6 bytes)
 * \ return Number of bytes copied (6) on success, -1 if handle/data is NULL or index is invalid
 * 
 * \ code
 * uint8_t lpf_data[6];
 * int32_t bytes = endat3_getLpfData(handle, 0, lpf_data);
 * if (bytes == 6) {
 *     // Process first LPF data
 * }
 * \ endcode
 */
int32_t endat3_getLpfData(endat3_Handle handle, uint8_t index, uint8_t *data);

/**
 * \ brief Get LPF CRC value for specific frame
 * 
 * Retrieves the CRC checksum from a specific Low Priority Frame.
 *
 * \ param handle EnDAT3 handle
 * \ param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \ return LPF CRC value, or 0 if handle is NULL or index is invalid
 */
uint8_t endat3_getLpfCrc(endat3_Handle handle, uint8_t index);

/**
 * \ brief Get LPF FID (Frame ID) for specific frame
 * 
 * Extracts the Frame ID from the LPF status byte.
 *
 * \ param handle EnDAT3 handle
 * \ param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \ return FID value (0-255), or 0 if handle is NULL or index is invalid
 */
uint8_t endat3_getLpfFid(endat3_Handle handle, uint8_t index);

/* ========================================================================== */
/*                    Communication Control APIs                              */
/* ========================================================================== */

/**
 * \ brief Get connection status
 * 
 * Checks if the encoder is connected and communicating.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if connected, 0 otherwise
 */
uint8_t endat3_isConnected(endat3_Handle handle);

/**
 * \ brief Get busy status
 * 
 * Checks if a transfer is currently in progress.
 *
 * \ param handle EnDAT3 handle
 * \ return  1 if busy, 0 otherwise
 */
uint8_t endat3_isBusy(endat3_Handle handle);

/**
 * \ brief Set busy status
 * 
 * Sets the busy flag to indicate a transfer is in progress.
 *
 * \ param handle EnDAT3 handle
 * \ param busy Busy state to set (1 for busy, 0 for not busy)
 * \ return 0 on success, -1 if handle is NULL
 */
int32_t endat3_setBusy(endat3_Handle handle, uint8_t busy);

/**
 * \ brief Get expected TX frame count
 * 
 * Retrieves the number of frames expected to be transmitted.
 *
 * \ param handle EnDAT3 handle
 * \ return Expected TX frame count, or 0 if handle is NULL
 */
uint32_t endat3_getExpectedTxFrameCount(endat3_Handle handle);

/**
 * \ brief Set expected TX frame count
 * 
 * Sets the number of frames expected to be transmitted.
 *
 * \ param handle EnDAT3 handle
 * \ param count Expected frame count
 * \ return 0 on success, -1 if handle is NULL
 */
int32_t endat3_setExpectedTxFrameCount(endat3_Handle handle, uint32_t count);

/**
 * \ brief Get propagation time
 * 
 * Retrieves the measured propagation time from ECHO command.
 * The value is in PRU clock cycles.
 *
 * \ param handle EnDAT3 handle
 * \ return Propagation time in PRU clock cycles, or 0 if handle is NULL
 * 
 * \ code
 * uint32_t prop_time_cycles = endat3_getPropagationTime(handle);
 * // Convert to nanoseconds (assuming 200MHz PRU clock)
 * uint32_t prop_time_ns = (prop_time_cycles * 1000) / 200;
 * \ endcode
 */
uint32_t endat3_getPropagationTime(endat3_Handle handle);

/* ========================================================================== */
/*                    Command and Data APIs                                   */
/* ========================================================================== */

/**
 * \ brief Get foreground operation code
 * 
 * Retrieves the current foreground operation code.
 *
 * \ param handle EnDAT3 handle
 * \ return Foreground operation code, or 0 if handle is NULL
 */
uint32_t endat3_getForegroundOpCode(endat3_Handle handle);

/**
 * \ brief Set foreground operation code
 * 
 * Sets the foreground operation code for the next command.
 *
 * \ param handle EnDAT3 handle
 * \ param opcode Operation code to set (use endat3_ReqCode_t enum values)
 * \ return 0 on success, -1 if handle is NULL
 * 
 * \ code
 * endat3_setForegroundOpCode(handle, endat3_REQ_DATA0);
 * \ endcode
 */
int32_t endat3_setForegroundOpCode(endat3_Handle handle, uint32_t opcode);

/**
 * \ brief Get background operation code
 * 
 * Retrieves the current background operation code.
 *
 * \ param handle EnDAT3 handle
 * \ return Background operation code, or 0 if handle is NULL
 */
uint32_t endat3_getBackgroundOpCode(endat3_Handle handle);

/**
 * \ brief Set background operation code
 * 
 * Sets the background operation code for the next command.
 *
 * \ param handle EnDAT3 handle
 * \ param opcode Operation code to set (use endat3_BgReqOpCode_t enum values)
 * \ return 0 on success, -1 if handle is NULL
 * 
 * \ code
 * endat3_setBackgroundOpCode(handle, endat3_BGREQ_READ);
 * \ endcode
 */
int32_t endat3_setBackgroundOpCode(endat3_Handle handle, uint32_t opcode);

/**
 * \ brief Get background data word
 * 
 * Retrieves a specific word from the background data array.
 *
 * \ param handle EnDAT3 handle
 * \ param index Data word index (0-5)
 * \ return Background data word, or 0 if handle is NULL or index is invalid
 */
uint32_t endat3_getBgData(endat3_Handle handle, uint8_t index);

/**
 * \ brief Set background data word
 * 
 * Sets a specific word in the background data array.
 *
 * \ param handle EnDAT3 handle
 * \ param index Data word index (0-5)
 * \ param data Data value to set
 * \ return 0 on success, -1 if handle is NULL or index is invalid
 * 
 * \ code
 * // Set RESET command data
 * endat3_setBgData(handle, 0, endat3_RESET_HARD);
 * \ endcode
 */
int32_t endat3_setBgData(endat3_Handle handle, uint8_t index, uint32_t data);

/**
 * \ brief Get all background data
 * 
 * Retrieves all background data words into the provided buffer.
 *
 * \ param handle EnDAT3 handle
 * \ param data Buffer to store background data (must be at least 6 words)
 * \ return 0 on success, -1 if handle or data is NULL
 */
int32_t endat3_getAllBgData(endat3_Handle handle, uint32_t *data);

/**
 * \ brief Set all background data
 * 
 * Sets all background data words from the provided buffer.
 *
 * \ param handle EnDAT3 handle
 * \ param data Buffer containing background data (must be at least 6 words)
 * \ return 0 on success, -1 if handle or data is NULL
 */
int32_t endat3_setAllBgData(endat3_Handle handle, const uint32_t *data);

/* ========================================================================== */
/*                    Buffer Access APIs                                      */
/* ========================================================================== */

/**
 * \ brief Get RX buffer pointer
 * 
 * Provides read-only access to the receive buffer.
 *
 * \ param handle EnDAT3 handle
 * \ return Pointer to RX buffer (64 bytes), or NULL if handle is NULL
 * 
 * \ warning Do not modify the returned buffer. Use for read-only access.
 */
const uint8_t* endat3_getRxBuffer(endat3_Handle handle);

/**
 * \ brief Get TX buffer pointer
 * 
 * Provides read-only access to the transmit buffer.
 *
 * \ param handle EnDAT3 handle
 * \ return Pointer to TX buffer (24 bytes), or NULL if handle is NULL
 * 
 * \ warning Do not modify the returned buffer. Use for read-only access.
 */
const uint8_t* endat3_getTxBuffer(endat3_Handle handle);

/**
 * \ brief Copy data to TX buffer
 * 
 * Safely copies data to the transmit buffer with bounds checking.
 *
 * \ param handle EnDAT3 handle
 * \ param data Source data buffer
 * \ param length Number of bytes to copy (max 24)
 * \ return Number of bytes copied, or -1 on error
 */
int32_t endat3_setTxBuffer(endat3_Handle handle, const uint8_t *data, uint32_t length);

/* ========================================================================== */
/*                    Utility and Helper APIs                                 */
/* ========================================================================== */

/**
 * \ brief Get complete HPF structure
 * 
 * Retrieves the entire High Priority Frame structure.
 *
 * \ param handle EnDAT3 handle
 * \ param hpf Pointer to HPF structure to fill
 * \ return 0 on success, -1 if handle or hpf is NULL
 * 
 * \ code
 * endat3_hpf_t hpf;
 * if (endat3_getHpfFrame(handle, &hpf) == 0) {
 *     // Access hpf.status, hpf.data, hpf.crc
 * }
 * \ endcode
 */
int32_t endat3_getHpfFrame(endat3_Handle handle, endat3_hpf_t *hpf);

/**
 * \ brief Get complete LPH structure
 * 
 * Retrieves the entire Low Priority Header structure.
 *
 * \ param handle EnDAT3 handle
 * \ param lph Pointer to LPH structure to fill
 * \ return 0 on success, -1 if handle or lph is NULL
 */
int32_t endat3_getLphFrame(endat3_Handle handle, endat3_lph_t *lph);

/**
 * \ brief Get complete LPF structure
 * 
 * Retrieves the entire Low Priority Frame structure for a specific index.
 *
 * \ param handle EnDAT3 handle
 * \ param index LPF frame index (0 to MAX_LPF_COUNT-1)
 * \ param lpf Pointer to LPF structure to fill
 * \ return 0 on success, -1 if handle/lpf is NULL or index is invalid
 */
int32_t endat3_getLpfFrame(endat3_Handle handle, uint8_t index, endat3_lpf_t *lpf);

/**
 * \ brief Extract error code from HPF or LPF
 * 
 * Extracts the error code from HPF data (when HPFV=0) or LPF data
 * (when BG.ERR_EXEC is set).
 *
 * \ param handle EnDAT3 handle
 * \ return Error code (endat3_ErrorCode_t), or 0 if no error
 * 
 * \ code
 * endat3_ErrorCode_t error = endat3_getErrorCode(handle);
 * if (error != endat3_ERR_UNKNOWN) {
 *     const char* desc = endat3_getErrorDescription(error);
 *     DebugP_log("Error: %s\r\n", desc);
 * }
 * \ endcode
 */
endat3_ErrorCode_t endat3_getErrorCode(endat3_Handle handle);

/**
 * \ brief Get interface pointer (advanced use)
 * 
 * Provides direct access to the endat3Interface structure for advanced
 * users who need full control. Use with caution.
 *
 * \ param handle EnDAT3 handle
 * \ return Pointer to endat3_Interface structure, or NULL if handle is NULL
 * 
 * \ warning Direct manipulation of the interface structure can lead to
 *          undefined behavior. Use the provided APIs whenever possible.
 */
endat3_Interface* endat3_getInterface(endat3_Handle handle);

/**
 * \ brief Set operating mode (host trigger or periodic trigger)
 * 
 * Configures the firmware operating mode for the EnDAT3 interface.
 * This determines whether the encoder is triggered by host commands
 * or by periodic IEP timer events.
 *
 * \ param handle EnDAT3 handle
 * \ param opmode Operating mode: 0 = periodic trigger, 1 = host trigger
 * \ return 0 on success, -1 if handle is NULL
 * 
 * \ code
 * // Set to host trigger mode (default)
 * endat3_setOperatingMode(handle, 1);
 * 
 * // Set to periodic trigger mode
 * endat3_setOperatingMode(handle, 0);
 * \ endcode
 */
int32_t endat3_setOperatingMode(endat3_Handle handle, uint8_t opmode);

/**
 * \ brief Get current operating mode
 * 
 * Retrieves the current firmware operating mode.
 *
 * \ param handle EnDAT3 handle
 * \ return Operating mode: 0 = periodic trigger, 1 = host trigger, or -1 if handle is NULL
 */
int32_t endat3_getOperatingMode(endat3_Handle handle);

/**
 * \ brief Release start trigger to firmware
 * 
 * Signals the firmware to begin processing by setting the start_trigger flag.
 * The firmware will process the command based on the current operating mode:
 * - In host mode: processes the command immediately
 * - In periodic mode: waits for the next IEP CMP3 event
 *
 * \ param handle EnDAT3 handle
 * \ return 0 on success, -1 if handle is NULL
 * 
 * \ code
 * // Release trigger to firmware
 * endat3_releaseStartTrigger(handle);
 * \ endcode
 */
int32_t endat3_releaseStartTrigger(endat3_Handle handle);

/**
 * \ brief Clear start trigger flag
 * 
 * Clears the start_trigger flag after the firmware has processed the command.
 * This prepares the interface for the next command.
 *
 * \ param handle EnDAT3 handle
 * \ return 0 on success, -1 if handle is NULL
 */
int32_t endat3_clearStartTrigger(endat3_Handle handle);

/**
 * \ brief Get current start trigger status
 * 
 * Retrieves the current state of the start_trigger flag.
 *
 * \ param handle EnDAT3 handle
 * \ return 1 if trigger is set, 0 if cleared, -1 if handle is NULL
 */
int32_t endat3_getStartTriggerStatus(endat3_Handle handle);

/** \ } */ /* End of endat3_INTERFACE_ACCESS_API */


#ifdef __cplusplus
}
#endif

#endif /* endat3_DRV_H_ */