/*
 * Copyright (C) 2023-2026 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   sdfm_drv.h
 *
 *  \brief  SDFM driver data structures, macros, and type definitions.
 *
 *  \details
 *  This header file defines the core data structures, macros, and types used by
 *  the SDFM (Sigma-Delta Filter Module) driver for current sensing on TI AM243x
 *  processors using PRU-ICSS.
 *
 *  ## Pointer Validation
 *
 *  All APIs validate internal structure pointers.
 *
 *  ## Architecture
 *
 *  The SDFM driver uses a firmware/driver split architecture:
 *  - **PRU Firmware**: Runs real-time sigma-delta filtering on PRU-ICSS cores
 *  - **ARM R5F Driver**: Manages configuration, control, and data retrieval
 *  - **Shared Memory**: DMEM interface for communication between ARM and PRU
 *
 *  ## Operating Modes
 *
 *  **Single PRU Mode**: All 9 channels processed by one PRU core
 *  - Suitable for lower channel counts or less demanding timing
 *  - Simpler configuration
 *
 *  **Load-Share Mode**: 9 channels distributed across 3 PRU cores
 *  - CH0-CH2 on RTU_PRU (pru_core index 1)
 *  - CH3-CH5 on PRU     (pru_core index 0)
 *  - CH6-CH8 on TX_PRU  (pru_core index 2)
 *  - Enables higher sampling rates and parallel processing
 *
 *  **Trigger Mode**: IEP-based synchronized sampling at specific PWM phase points
 *  - Precise timing control for motor control applications
 *  - Configurable first and second sample trigger times
 *  - Uses IEP compare events for synchronization
 *
 *  **Snoop Mode**: Continuous sampling with over-current monitoring
 *  - Lower latency for fault detection
 *  - Independent OSR for normal current vs over-current
 *
 *  ## Key Data Structures
 *
 *  - **SDFM_Params**: Initialization parameters from SysConfig (compile-time config)
 *  - **SDFM_Config**: Runtime driver handle containing PRU interface and state
 *  - **SDFM_Interface**: Firmware interface structure in PRU DMEM (shared memory)
 *  - **SDFM_ChannelConfig**: Per-channel runtime configuration
 *  - **SDFM_ChannelAttrs**: Per-channel compile-time attributes from SysConfig
 *
 *  ## Memory Layout
 *
 *  The SDFM_Interface structure is mapped to PRU DMEM and must match the firmware's
 *  expected memory layout exactly. This enables zero-copy communication between
 *  ARM R5F and PRU cores.
 *
 *  ## Usage Pattern
 *
 *  1. Configure via SysConfig (generates SDFM_Params)
 *  2. Call SDFM_init() to create handle and initialize firmware interface
 *  3. Configure channels, thresholds, triggers via API functions
 *  4. Enable SDFM via SDFM_enable()
 *  5. Read samples via SDFM_getFilterData()
 *  6. Monitor status via threshold and fast-detect status functions
 *
 *  ## Related Files
 *
 *  - sdfm_api.h: Public API function declarations
 *  - sdfm_drv.c: Driver implementation
 *  - icssg_sdfm.h: Firmware interface definitions
 */

#ifndef _SDFM_DRV_H_
#define _SDFM_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/soc.h>
#include <drivers/pruicss.h>
#include <pruicss_pwm/include/pruicss_pwm.h>
#include  <math.h>


/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/* IEP counter configuration */
#define IEP_DEFAULT_INC                 0x1  /**< Default IEP counter increment value */

/* SDFM channel and core configuration */
#define NUM_CH_SUPPORTED_PER_AXIS        ( 3 )   /**< Number of channels per PRU core in load-share mode */
#define SDFM_NUM_OF_CH_PER_PRU_SLICE     ( 9 )   /**< Total number of SDFM channels per PRU slice */
#define NUM_OF_PRU_CORE_PER_PRU_SLICE    ( 3 )   /**< Number of PRU cores per slice (PRU, RTU, TX) */

/* Channel masks for load-share mode */
#define SDFM_NINE_CH_MASK                ( 0x1FF )  /**< Mask for all 9 channels */
#define SDFM_CH_MASK_FOR_CH0_CH3_CH6     ( 0x49 )   /**< Mask for channels 0,3,6 (RTU_PRU) */
#define SDFM_CH_MASK_FOR_CH1_CH4_CH7     ( 0x92 )   /**< Mask for channels 1,4,7 (PRU) */
#define SDFM_CH_MASK_FOR_CH2_CH5_CH8     ( 0x124 )  /**< Mask for channels 2,5,8 (TX_PRU) */

/* SDFM Channel IDs */
#define SDFM_CHANNEL0    ( 0 )  /**< SDFM channel 0 */
#define SDFM_CHANNEL1    ( 1 )  /**< SDFM channel 1 */
#define SDFM_CHANNEL2    ( 2 )  /**< SDFM channel 2 */
#define SDFM_CHANNEL3    ( 3 )  /**< SDFM channel 3 */
#define SDFM_CHANNEL4    ( 4 )  /**< SDFM channel 4 */
#define SDFM_CHANNEL5    ( 5 )  /**< SDFM channel 5 */
#define SDFM_CHANNEL6    ( 6 )  /**< SDFM channel 6 */
#define SDFM_CHANNEL7    ( 7 )  /**< SDFM channel 7 */
#define SDFM_CHANNEL8    ( 8 )  /**< SDFM channel 8 */

/* SDFM control flags */
#define BF_SDFM_EN_ENABLE               ( 1 )  /**< SDFM enable acknowledgment value */

#define NUM_FD_FIELDS                   ( 4 )  /**< Number of fast-detect configuration fields */
#define SDFM_FW_VERSION_BIT_SHIFT       ( 32 )  /**< Bit shift to extract release version from 64-bit firmware version field */

/* Phase delay compensation */
#define SDFM_PHASE_DELAY_ACK_BIT_MASK   ( 1 )  /**< Phase delay acknowledgment bit mask */
#define SDFM_PHASE_DELAY_CAL_LOOP_SIZE  ( 8 )  /**< Phase delay calibration loop size */

/* Time conversion constants */
#define SDFM_NANOSECONDS_PER_SECOND     ( 1000000000U )  /**< Nanoseconds in one second (for phase delay conversion) */

/* IEP comparator event configuration */
#define SDFM_IEP_CMP1_EN_SHIFT     ( 2 )  /**< IEP CMP1 enable bit shift */
#define SDFM_IEP_CMP2_EN_SHIFT     ( 3 )  /**< IEP CMP2 enable bit shift */
#define SDFM_IEP_CMP_EVENT_MAX     ( 15U )  /**< Maximum IEP compare event number (0-15) */
#define SDFM_IEP_CMP_EVENT_CMP7    ( 7U )   /**< IEP compare event CMP7 (boundary before register gap) */
#define SDFM_IEP_CMP_REG_GAP_SIZE  ( 8U )   /**< Register offset gap after CMP7 (2 reserved registers * 4 bytes) */

/* PRU core indices
 * In load-share mode, the channel-to-core mapping is:
 *   Channels 0-2 -> RTU_PRU (index 1)
 *   Channels 3-5 -> PRU     (index 0)
 *   Channels 6-8 -> TX_PRU  (index 2)
 */
#define SDFM_PRU_CORE_INDEX          0U  /**< PRU core index (channels 3-5 in load-share mode) */
#define SDFM_RTUPRU_CORE_INDEX       1U  /**< RTU PRU core index (channels 0-2 in load-share mode) */
#define SDFM_TXPRU_CORE_INDEX        2U  /**< TX PRU core index (channels 6-8 in load-share mode) */

/* Hardware register limits (from ICSSG_PRU0_SD_CLK_SEL_REG0 and ICSSG_PRU0_SD_SAMPLE_SIZE_REG0) */
#define SDFM_OSR_MIN                 ( 4U )   /**< Minimum user-facing OSR value (register: OSR-1 = 3) */
#define SDFM_OSR_MAX                 ( 256U ) /**< Maximum user-facing OSR value (register: OSR-1 = 255) */
#define SDFM_ACC_FILTER_MAX          ( 2U )   /**< Maximum accumulator filter selection (0=acc3, 1=acc2, 2=acc1) */
#define SDFM_CLK_SOURCE_MAX          ( 2U )   /**< Maximum clock source selection (0=sd8_clk, 1=sd\<i\>_clk, 2=group clk) */
#define SDFM_CLK_INV_MAX             ( 1U )   /**< Maximum clock inversion value (0=normal, 1=inverted) */
#define SDFM_FD_WINDOW_SIZE_MIN      ( 0U )   /**< Minimum FD window size (4 samples, register value 0) */
#define SDFM_FD_WINDOW_SIZE_MAX      ( 6U )   /**< Maximum FD window size (28 samples, register value 6) */
#define SDFM_FD_THRESHOLD_MAX        ( 28U )  /**< Maximum FD threshold value (maps to count 28, 0x00=1 to 0x1B=28) */
#define SDFM_FD_THRESHOLD_MIN        ( 1U )   /**< Minimum threshold value  */
#define SDFM_THRESHOLD_MAX           ( 16777216U ) /**< Maximum threshold value for high/low/zero-cross thresholds (2^24) */
#define SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT    ( 5U ) /**< Maximum wait loop count for firmware acknowledgment timeout */
#define SDFM_DEFAULT_FW_WAIT_DELAY_US       ( 1000U ) /**< Delay in microseconds between firmware acknowledgment checks (1000 us = 1 ms) */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 * \brief SDFM clock source enumeration
 */
typedef enum SDFM_ClockSource_e {
    SDFM_CLOCK_SOURCE_IEP = 0,   /**< IEP as clock source */
    SDFM_CLOCK_SOURCE_ECAP = 1,  /**< eCAP as clock source */
    SDFM_CLOCK_SOURCE_PRU_GPIO1 = 2, /**< PRU GPIO1 as clock source */
    SDFM_EXTERNAL_CLOCK_SRC = 3 /**< External clock source */
} SDFM_ClockSource;

/**
 *  \brief SDFM threshold configuration structure
 *
 *  Structure to configure the high and low threshold values for over-current detection
 */
typedef struct SDFM_ThresholdConfig_s
{
    uint32_t high_threshold;  /**< High threshold value for over-current detection. Valid range: 0 to 16777216. Must be greater than low_threshold. */
    uint32_t low_threshold;   /**< Low threshold value for over-current detection. Valid range: 0 to 16777216. Must be less than high_threshold. */
} SDFM_ThresholdConfig;

/**
 *  \brief SDFM fast detect configuration structure
 *
 *  Structure to configure fast detect parameters for rapid error detection in SDFM channels
 */
typedef struct SDFM_FastDetectConfig_s
{
    uint8_t fd_enable;       /**< Fast detect enable: 0 = disabled, 1 = enabled */
    uint8_t fd_window_size;  /**< Fast detect window size. Valid range: 0-6 (maps to 4-28 samples: window_size * 4 + 4) */
    uint8_t fd_zero_max;     /**< Zero max threshold for fast detect. Valid range: 1-28 (number of zero crossings) */
    uint8_t fd_zero_min;     /**< Zero min threshold for fast detect. Valid range: 1-28 (number of zero crossings) */
} SDFM_FastDetectConfig;

/**
 *    \brief    Structure defining SDFM triggered mode trigger times
 *
 *    \details  Firmware trigger fields exposed through PRU data <br>
 *              memory - used by driver to start sampling and    <br>
 *              generate optional output event after receiving   <br>
 *              input trigger
 */
typedef struct SDFM_CfgTrigger_s
{
    /**< enable trigger mode */
    volatile  uint8_t  enable_trigger_mode;
    /**< enable double update */
    volatile uint8_t   en_double_nc_sampling;
    /**< First sample starting point */
    volatile uint32_t first_samp_trig_time;
    /**< Second sample starting point */
    volatile uint32_t second_samp_trig_time;
    /**< IEP0 counts in normal current sampling period */
    volatile uint64_t nc_prd_iep_cnt;
    /**< Max IEP0 counts in one EPWM period */
    volatile uint32_t max_iep_cnt_per_epwm_prd;
    /**< CMP event number */
    volatile uint8_t iep_cmp_event;
    /**< CMP event register address */
    volatile uint32_t iep_cmp_event_reg;
    /**< IEP CMP status register address */
    volatile uint32_t iep_cmp_status_reg;
    /**< Host output sample buffer base address for this channel */
    volatile uint32_t   sample_buff_base_addr;
    /**< reserved variable */
    volatile uint32_t   reserved;
} SDFM_CfgTrigger;


/**
 *    \brief    Structure defining SDFM base address and values to toggle GPIO pins
 *
 *    \details  Used to toggle the gpio based on different threshold conditions
 *
 */
typedef struct SDFM_GpioParams_s{
    volatile uint32_t write_val;      /**< GPIO pin bit mask value */
    volatile uint32_t set_val_addr;   /**< GPIO set data register address */
    volatile uint32_t clr_val_addr;   /**< GPIO clear data register address */
} SDFM_GpioParams;


/**
 *    \brief    Structure defining SDFM thresholds parameters
 *
 *    \details  High, Low & zero cross thresholds for SDFM channel  <br>
 *
 */
typedef struct SDFM_ThresholdParms_s
{
    /**< High threshold value */
    volatile uint32_t    high_threshold;
    /**< Low threshold value */
    volatile uint32_t    low_threshold;
    /**< High Threshold status */
    volatile uint8_t     high_th_status;
    /**< Low Threshold status */
    volatile uint8_t     low_th_status;
    /**< Zero cross enable bit */
    volatile uint8_t    en_zero_cross;
    /**< Zero cross threshold status */
    volatile uint8_t    zero_cross_th_status;
    /**< Zero cross threshold value */
    volatile uint32_t    zero_cross_threshold;
} SDFM_ThresholdParms;
/**
 *    \brief    Structure defining configuration for a single SDFM channel
 *
 *    \details  Contains all configuration parameters specific to one SDFM channel
 */
typedef struct SDFM_ChannelConfig_s
{
    /**< Channel ID (0-8) */
    volatile uint8_t   ch_id;

    /**< Enable/disable status for this channel */
    volatile uint8_t   enabled;

    /**< Filter type - sinc1, sinc2, sinc3 */
    volatile uint8_t   filter_type;

    /**< Normal current Over Sampling Rate (OSR) */
    volatile uint8_t   normal_current_osr;

    /**< Over current Over Sampling Rate (OSR) */
    volatile uint8_t   over_current_osr;

    /**< SDFM clock frequency for this channel */
    volatile uint32_t  sdfm_clk;

    /**< Enable comparator for this channel */
    volatile uint8_t  enable_comparator;

    /**< Enable fast detect for this channel */
    volatile uint8_t   fd_enable;

    /**< Fast detect window size */
    volatile uint8_t   fd_window;

    /**< Fast detect max count of zero */
    volatile uint8_t   fd_zero_max;

    /**< Fast detect min count of zero */
    volatile uint8_t   fd_zero_min;

    /**< Fast detect max count of one */
    volatile uint8_t   fd_one_max;

    /**< Fast detect min count of one */
    volatile uint8_t   fd_one_min;

    /**< Clock source for this channel */
    volatile uint32_t  clk_source;

    /**< Clock inversion for this channel */
    volatile uint8_t   clk_inv;

    /**< Enable phase delay calculation */
    volatile uint8_t   en_phase_delay;

    /**< Clock phase delay */
    volatile uint16_t  clock_phase_delay;

    /**< Nearest clock edge type: 0 = rising edge, 1 = falling edge */
    volatile uint16_t  clock_edge;

    /**< Threshold configuration */
    SDFM_ThresholdParms  threshold_config;

    /**< GPIO parameters for this channel */
    SDFM_GpioParams     gpio_params;

} SDFM_ChannelConfig;

/**
 *    \brief    Structure defining SDFM control settings
 *
 *    \details  Contains control parameters for SDFM operation
 */
typedef struct SDFM_Control_s
{
    /**< SDFM Enable */
    volatile uint8_t enable;

    /**< SDFM Enable Ack */
    volatile uint8_t enable_ack;

    /**< Enable snoop based Normal current sampling */
    volatile uint8_t enable_snoop_nc;
} SDFM_Control;
/**
 *    \brief    Structure defining SDFM interface components that will be placed in DMEM for firmware interaction
 *
 *    \details  Contains control settings, channel configurations, and trigger settings in a layout
 *              that exactly matches the firmware's expected memory layout
 */
typedef struct SDFM_Interface_s
{
    /**< Global SDFM control settings for each PRU core */
    SDFM_Control control[NUM_OF_PRU_CORE_PER_PRU_SLICE];

    /**< Channel mask indicating which channels are active */
    volatile uint16_t active_channels_mask;

    /**< Firmware version */
    volatile uint64_t firmwareVersion;

    /**< Trigger configuration */
    SDFM_CfgTrigger trigger_config[NUM_OF_PRU_CORE_PER_PRU_SLICE];

    /**< Channel-specific configurations - array of 9 channels */
    SDFM_ChannelConfig channels[SDFM_NUM_OF_CH_PER_PRU_SLICE];


} SDFM_Interface;
/**
 *    \brief    Structure defining SDFM sample output address
 *
 *    \details  SDFM sample output address, used by application/driver to read output samples
 *
 */
typedef struct SDFM_SampleOutInterface_s
{
   uint32_t sampleOutput[SDFM_NUM_OF_CH_PER_PRU_SLICE];
} SDFM_SampleOutInterface;

/**
 *    \brief    Structure defining SDFM interface with channel-based organization
 *
 *    \details  Firmware configuration, control, data and trigger interface with all channel settings organized separately
 */
typedef struct SDFM_Priv_s
{
    /**< Initialization state flag.
    *   0 = Driver closed/not initialized
    *   1 = Driver successfully initialized and open */
    uint8_t is_open;

    /**< Pointer to SDFM interface in DMEM */
    SDFM_Interface *sdfm_interface;

    /**< Pointer to sample output interface for reading channel samples */
    SDFM_SampleOutInterface *sampleOutputInterface;

    /**< PRU ICSS Handle */
    PRUICSS_Handle pruicss_handle;

    /**< PRU PWM Handle */
    PRUICSS_PWM_Handle pwm_handle;

} SDFM_Priv;

/**
 *    \brief    Structure defining SDFM PRU core specific attributes (compile-time/SysConfig configuration data)
 *
 *    \details  Configuration data for a single PRU core (PRU, RTU-PRU, or TX-PRU) including
 *              trigger mode settings, double sampling, and IEP compare event configuration
 */
typedef struct SDFM_PruCoreAttrs_s
{
    /**< enable trigger mode */
    volatile  uint8_t  enable_trigger_mode;
    /**< enable double update */
    volatile uint8_t   en_double_nc_sampling;
    /**< First sample starting point */
    volatile uint32_t first_samp_trig_time;
    /**< Second sample starting point */
    volatile uint32_t second_samp_trig_time;
    /**< CMP event number */
    volatile uint8_t iep_cmp_event;
    /**< Enable snoop mode */
    uint8_t enable_snoop_mode;

} SDFM_PruCoreAttrs;

/**
 *    \brief    Structure defining SDFM channel specific attributes (compile-time/SysConfig configuration data)
 *
 *    \details  Configuration-only version of SDFM_ChannelConfig for initialization from SysConfig
 */
typedef struct SDFM_ChannelAttrs_s
{
    /**< Channel ID (0-8) */
    volatile uint8_t   ch_id;

    /**< Enable/disable status for this channel */
    volatile uint8_t   enabled;

    /**< Filter type - sinc1, sinc2, sinc3 */
    volatile uint8_t   filter_type;

    /**< Normal current Over Sampling Rate (OSR) */
    volatile uint16_t   normal_current_osr;

    /**< Over current Over Sampling Rate (OSR) */
    volatile uint16_t   over_current_osr;

    /**< SDFM clock frequency for this channel */
    volatile uint32_t  sdfm_clk;

    /**< Enable comparator for this channel */
    volatile uint8_t  enable_comparator;

    /**< Enable fast detect for this channel */
    volatile uint8_t   fd_enable;

    /**< Fast detect window size */
    volatile uint8_t   fd_window;

    /**< Fast detect max count of zero */
    volatile uint8_t   fd_zero_max;

    /**< Fast detect min count of zero */
    volatile uint8_t   fd_zero_min;

    /**< Clock source for this channel */
    volatile uint32_t  clk_source;

    /**< Clock inversion for this channel */
    volatile uint8_t   clk_inv;

    /**< High threshold value */
    volatile uint32_t    high_threshold;

    /**< Low threshold value */
    volatile uint32_t    low_threshold;

    /**<  Zero cross enable bit*/
    volatile uint8_t    en_zero_cross;

    /**< Zero Cross Threshold*/
    volatile uint32_t    zero_cross_threshold;

} SDFM_ChannelAttrs;

/**
 *    \brief    Structure defining SDFM attributes (compile-time/SysConfig configuration)
 *
 *    \details  Contains all compile-time configuration parameters for an SDFM instance,
 *              typically generated by SysConfig. Includes PRU instance selection, channel
 *              configuration, clock frequencies, and operation mode settings.
 */
typedef struct SDFM_Attrs_s
{
    /**< SDFM instance index */
    uint8_t instance;

    /**< ICSS instance (0 for ICSSG0, 1 for ICSSG1) */
    uint8_t pruicss_instance;

    /**< PRU slice being used (0 or 1) */
    uint8_t pruicss_slice;

    /**< Load share mode enable flag.
     *   0 = Disabled (single PRU handles all channels)
     *   1 = Enabled (channels distributed across RTU-PRU, PRU, and TX-PRU in PRU-ICSSG only) */
    uint8_t load_share_enabled;

    /**< Channel mask indicating which of the 9 channels are enabled */
    uint16_t channel_mask;

    /**< Array indicating enable status for each of the 9 channels */
    uint8_t channel_enabled[SDFM_NUM_OF_CH_PER_PRU_SLICE];

    /**< PRU-ICSS Core Clock frequency in Hz (not MHz) */
    uint32_t core_clk_freq;

    /**< PRU-ICSS IEP (Industrial Ethernet Peripheral) timer clock frequency in Hz */
    uint32_t iep_clk_freq;

    /**< IEP reset frequency in Hz for trigger mode operation */
    uint32_t iep_reset_freq;

    /**< IEP instance selection (PRUICSS_IEP_INST0 or PRUICSS_IEP_INST1) */
    uint8_t iep_instance;

    /**< SDFM clock source selection (IEP, eCAP, PRU GPIO, or external) */
    uint8_t  sdfm_clock_source;

    /**< SDFM sampling frequency in Hz */
    uint32_t sdfm_sampling_freq;

    /**< PRU core enable mask (bit 0: PRU, bit 1: RTU-PRU, bit 2: TX-PRU) */
    uint8_t pru_core_mask;

    /**< Enable ePWM sync for normal current sampling */
    uint8_t enable_epwm_sync;

    /**< ePWM sync source selection (EPWM0 or EPWM3) */
    uint8_t epwm_sync_source;

    /**< PRU core specific configuration for each of the 3 cores */
    SDFM_PruCoreAttrs pru_core_config[NUM_OF_PRU_CORE_PER_PRU_SLICE];

    /**< Channel specific attributes for all 9 channels */
    SDFM_ChannelAttrs channels[SDFM_NUM_OF_CH_PER_PRU_SLICE];

} SDFM_Attrs;

/**
 *    \brief    Structure defining SDFM configuration handle
 *
 *    \details  This structure combines pointers to both runtime state (priv) and compile-time
 *              configuration (attrs). The handle is returned by SDFM_init() and passed to all
 *              SDFM driver APIs to identify the specific SDFM instance being operated on.
 *
 */
typedef struct sdfm_config_s
{
    SDFM_Priv *priv;
    /**< Pointer to SDFM private data (runtime state and results).
     *   Contains PRU interface, channel parameters, and all runtime operational state maintained by the driver */

    const SDFM_Attrs *attrs;
    /**< Pointer to SDFM attributes (read-only configuration from SysConfig).
     *   Contains compile-time configuration including PRU instance, channels,
     *   clock frequencies, and operation mode settings */
} SDFM_Config;

/**
 *    \brief    SDFM handle type
 *
 *    \details  Opaque handle to an SDFM instance. Obtained from SDFM_init() and used
 *              in all subsequent SDFM API calls.
 */
typedef SDFM_Config *SDFM_Handle;
/**
 *    \brief    Structure defining SDFM initialization parameters
 *
 *    \details  Contains runtime handles and addresses needed for SDFM driver initialization.
 *              Passed to SDFM_init() to provide PRU-ICSS handle, PWM handle, and sample buffer address.
 */
typedef struct SDFM_Params_s
{
    /**< PRU ICSS Handle */
    PRUICSS_Handle pruicss_handle;

    /**< PRU PWM Handle (optional, can be NULL if not using PWM) */
    PRUICSS_PWM_Handle pwm_handle;

    /**< Output samples base address in PRU shared memory */
    uint32_t sample_base_addr;
} SDFM_Params;

#include "sdfm_api.h"

#ifdef __cplusplus
}
#endif

#endif
