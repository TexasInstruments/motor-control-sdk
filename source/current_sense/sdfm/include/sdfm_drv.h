/*
 * Copyright (C) 2023-25 Texas Instruments Incorporated - http://www.ti.com/
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
 *  - CH0,CH3,CH6 on RTU_PRU
 *  - CH1,CH4,CH7 on PRU
 *  - CH2,CH5,CH8 on TX_PRU
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
#define SDFM_FW_VERSION_BIT_SHIFT       ( 32 )  /**< SDFM firmware version bit shift */

/* Phase delay compensation */
#define SDFM_PHASE_DELAY_ACK_BIT_MASK   ( 1 )  /**< Phase delay acknowledgment bit mask */
#define SDFM_PHASE_DELAY_CAL_LOOP_SIZE  ( 8 )  /**< Phase delay calibration loop size */

/* IEP comparator event configuration */
#define SDFM_IEP_CMP1_EN_SHIFT     ( 2 )  /**< IEP CMP1 enable bit shift */
#define SDFM_IEP_CMP2_EN_SHIFT     ( 3 )  /**< IEP CMP2 enable bit shift */

/* PRU core indices */
#define SDFM_PRU_CORE_INDEX          0U  /**< PRU core index */
#define SDFM_RTUPRU_CORE_INDEX       1U  /**< RTU PRU core index */
#define SDFM_TXPRU_CORE_INDEX        2U  /**< TX PRU core index */

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 * \brief SDFM clock source enumeration
 */
typedef enum SDFM_ClockSource_e {
    SDFM_CLOCK_SOURCE_IEP = 0,   /**< IEP as clock source */
    SDFM_CLOCK_SOURCE_ECAP = 1,  /**< eCAP as clock source */
    SDFM_CLOCK_SOURCE_PRUGPIO1 = 2, /**< PRU GPIO1 as clock source */
    SDFM_EXTERNAL_CLOCK_SRC = 3 /**< External clock source */
} SDFM_ClockSource;

/**
 *    \brief    Structure defining SDFM clock configuration parameters.
 *
 *    \details  Firmware SD clock configuration interface exposed through PRU data <br>
 *              memory - used by driver to configure firmware parameters
 */
typedef struct SDFM_CfgSdClk_s
{
    /**< Clock source selection (IEP, eCAP, or PRU GPIO1) */
    volatile SDFM_ClockSource clock_source;
    
    /**< Sdfm Clock frequency value in Hz */
    volatile uint64_t sdfm_clock_value;
    
    /**< Sdfm Clock divider value */
    volatile uint64_t sdfm_source_clock_value;
    
} SDFM_CfgSdClk;

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
    /**<Second sample starting point*/
    volatile uint32_t second_samp_trig_time;
    /**< IEP0 counts in normal current sampling period*/
    volatile uint64_t nc_prd_iep_cnt;
    /**< max IEP0 counts in one epwm period*/
    volatile uint32_t max_iep_cnt_per_epwm_prd;
    /**< CMP event number */
    volatile uint8_t iep_cmp_event;
    /**< CMP event register address */
    volatile uint32_t iep_cmp_event_reg;
    /**<IEP cmp status register address */
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
    volatile uint32_t write_val;
    volatile uint32_t set_val_addr;
    volatile uint32_t clr_val_addr;
} SDFM_GpioParams;


/**
 *    \brief    Structure defining SDFM thresholds parametrs
 *
 *    \details  High, Low & zero cross thresholds for sdfm channel  <br>
 *
 */
typedef struct SDFM_ThresholdParms_s
{
    /**< High threshold value */
    volatile uint32_t    high_threshold;
    /**< Low threshold value */
    volatile uint32_t    low_threshold;
    /**<  High Threshold status*/
    volatile uint8_t     high_th_status;
    /**<  High Threshold status*/
    volatile uint8_t     low_th_status;
    /**<  Zero cross enable bit*/
    volatile uint8_t    en_zero_cross;
    /**<  Zero cross Threshold status */
    volatile uint8_t    zero_cross_th_status;
    /**< Zero Cross Threshold*/
    volatile uint32_t    zero_cross_threshold;
}SDFM_ThresholdParms;
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

    /**< Nearest clock edge status of data */
    volatile uint16_t  clock_edge;

    /**< Threshold configuration */
    SDFM_ThresholdParms  threshold_config;

    /**< GPIO parameters for this channel */
    SDFM_GpioParams     gpio_params;

} SDFM_ChannelConfig;

/**
 *    \brief    Structure defining SDFM PRU configuration
 *
 *    \details  Contains configuration parameters for PRU including ID, clock settings, and load sharing
 */
typedef struct SDFM_CfgPru_s
{
    /**< PRU Slice */
    volatile uint8_t pru_slice;
    
    /**< PRU ICSS Handle */
    PRUICSS_Handle pruicss_handle;
    
    /**< PRU PWM Handle */
    PRUICSS_PWM_Handle pwm_handle;
    
    /**< PRU core clock frequency in Hz */
    volatile uint32_t pru_clock;
    
    /**< Enable load sharing between PRUs */
    volatile uint8_t load_share_enable;
    
   /**< IEP Instance (0 for IEP0, 1 for IEP1) */
   volatile uint8_t iep_instance;
    
   /**< Increment value of IEP counter */
   volatile uint8_t iep_inc_value;

   /**< PRU iep clock frequency in Hz */
   volatile uint32_t iep_clock;
    
} SDFM_CfgPru;

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
   uint32_t sampleOutput[NUM_CH_SUPPORTED_PER_AXIS];
}SDFM_SampleOutInterface;

/**
 *    \brief    Structure defining SDFM interface with channel-based organization
 *
 *    \details  Firmware configuration, control, data and trigger interface with all channel settings organized separately
 */
typedef struct SDFM_Config_s {
    /**< PRU configuration */
    SDFM_CfgPru pru_config;
    
    /**< Global SD clock configuration */
    SDFM_CfgSdClk clk_config;

    /**< Pointer to SDFM interface in DMEM */
    SDFM_Interface *sdfm_interface;
    
    /**< synchronization with EPWM */
    volatile uint8_t enable_sync_with_epwm;

    /**< EPWM source for synchronization */
    volatile uint8_t sync_epwm_src;

    SDFM_SampleOutInterface *sampleOutputInterface;
    
} SDFM_Config;
/**
 *    \brief    Handle to the SDFM driver object
 *
 */
typedef struct SDFM_Config_s *SDFM_Handle;

/**
 *    \brief    Structure defining SDFM PRU core specific attributes (compile-time/SysConfig configuration data)
 *
 */
typedef struct SDFM_PruCoreAttrs_s
{
    /**< enable trigger mode */
    volatile  uint8_t  enable_trigger_mode;
    /**< enable double update */
    volatile uint8_t   en_double_nc_sampling;
    /**< First sample starting point */
    volatile uint32_t first_samp_trig_time;
    /**<Second sample starting point*/
    volatile uint32_t second_samp_trig_time;
    /**< CMP event number */
    volatile uint8_t iep_cmp_event;
    /**< enable snoop mode */
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
 *    \brief    Structure defining SDFM initialization parameters.
 *
 */
typedef struct SDFM_Params_s
{
    PRUICSS_Handle pruicss_handle; /**< PRU ICSS Handle */
    PRUICSS_PWM_Handle pwm_handle; /**< PRU PWM Handle */
    SDFM_PruCoreAttrs pru_core_config[NUM_OF_PRU_CORE_PER_PRU_SLICE];
    SDFM_ChannelAttrs channels[SDFM_NUM_OF_CH_PER_PRU_SLICE];
    uint8_t load_share_enable; /**< Enable load sharing between PRUs */
    uint8_t icss_instance;   /**< ICSS instance (0 for ICSSG0, 1 for ICSSG1) */
    uint8_t pru_slice; /**< PRUx slice being used */
    uint8_t iep_instance;     /**< IEP instance (0 for IEP0, 1 for IEP1) */
    uint32_t iep_reset_freq; /**< IEP reset frequency in Hz */
    uint32_t pru_clock; /**< PRU core clock frequency in Hz */
    uint32_t iep_clock; /**< IEP clock frequency in Hz */
    uint16_t enable_channel_mask; /**< SDFM channel mask to indicate active channels */
    uint8_t enable_pru_core_mask; /**< PRU core mask to indicate which PRU cores are enabled */
    uint8_t enable_epwm_sync; /**< Enable ePWM sync for normal current sampling */
    uint8_t epwm_sync_source; /**< ePWM sync source selection */
    uint32_t sample_base_addr; /**< output samples base address*/
    uint8_t enable_phase_delay; /**< Enable phase delay measurement */
}SDFM_Params;

#include "sdfm_api.h"

#ifdef __cplusplus
}
#endif

#endif
