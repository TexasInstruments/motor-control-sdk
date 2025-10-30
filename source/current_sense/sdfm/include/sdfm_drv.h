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


/** \brief SD channel control, channel disable/enable */
#define DEF_SD_CH_CTRL_CH_EN        ( 0 )       /* default all chs disabled */
#define BF_CH_EN_MASK               ( 0x1 )
#define SDFM_CH_CTRL_CH_EN_BF_CH0_EN_SHIFT   ( 0 )
#define SDFM_CH_CTRL_CH_EN_BF_CH0_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH0_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH1_EN_SHIFT   ( 1 )
#define SDFM_CH_CTRL_CH_EN_BF_CH1_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH1_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH2_EN_SHIFT   ( 2 )
#define SDFM_CH_CTRL_CH_EN_BF_CH2_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH2_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH3_EN_SHIFT   ( 3 )
#define SDFM_CH_CTRL_CH_EN_BF_CH3_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH3_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH4_EN_SHIFT   ( 4 )
#define SDFM_CH_CTRL_CH_EN_BF_CH4_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH4_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH5_EN_SHIFT   ( 5 )
#define SDFM_CH_CTRL_CH_EN_BF_CH5_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH5_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH6_EN_SHIFT   ( 6 )
#define SDFM_CH_CTRL_CH_EN_BF_CH6_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH6_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH7_EN_SHIFT   ( 7 )
#define SDFM_CH_CTRL_CH_EN_BF_CH7_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH7_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH8_EN_SHIFT   ( 8 )
#define SDFM_CH_CTRL_CH_EN_BF_CH8_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH8_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_BF_CH9_EN_SHIFT   ( 9 )
#define SDFM_CH_CTRL_CH_EN_BF_CH9_EN_MASK    ( BF_CH_EN_MASK << SDFM_CH_CTRL_CH_EN_BF_CH9_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_SHIFT             ( SDFM_CH_CTRL_CH_EN_BF_CH0_EN_SHIFT )
#define SDFM_CH_CTRL_CH_EN_MASK \
    ( SDFM_CH_CTRL_CH_EN_BF_CH0_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH1_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH2_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH3_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH4_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH5_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH6_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH7_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH8_EN_MASK | \
      SDFM_CH_CTRL_CH_EN_BF_CH9_EN_MASK )

#define SDFM_MAIN_FILTER_MASK    ( 1 )
#define SDFM_MAIN_FILTER_SHIFT          ( 0 )

#define SDFM_MAIN_INTERRUPT_MASK     ( 1 )
#define SDFM_MAIN_INTERRUPT_SHIFT           ( 1 )

/**    \brief    reinitialize PRU SDFM */
#define SDFM_RECFG_REINIT               ( SDFM_RECFG_BF_RECFG_REINIT_MASK )
/**    \brief    reconfigure SD clock */
#define SDFM_RECFG_CLK                  ( SDFM_RECFG_BF_RECFG_CLK_MASK )
/**    \brief    reconfigure SD OSR */
#define SDFM_RECFG_OSR                  ( SDFM_RECFG_BF_RECFG_OSR_MASK )
/**    \brief    reconfigure Trigger mode sample time */
#define SDFM_RECFG_TRIG_SAMP_TIME       ( SDFM_RECFG_BF_RECFG_TRIG_SAMPLE_TIME_MASK )
/**    \brief    reconfigure Trigger mode sample count */
#define SDFM_RECFG_TRIG_SAMP_CNT        ( SDFM_RECFG_BF_RECFG_TRIG_SAMPLE_CNT_MASK )
/**    \brief    reconfigure SD channel disable/enable */
#define SDFM_RECFG_CH_EN                ( 1<<6 )
/**    \brief    reconfigure SD channel disable/enable */
#define SDFM_RECFG_FD                   ( SDFM_RECFG_BF_RECFG_FD_MASK )
/**    \brief    reconfigure Trigger mode output sample buffer */
#define SDFM_RECFG_TRIG_OUT_SAMP_BUF    ( SDFM_RECFG_BF_RECFG_TRIG_OUT_SAMP_BUF_MASK )
/**    \brief IEP_CFG*/
#define IEP_DEFAULT_INC                 0x1



/* SDFM output buffer size in 32-bit words */
#define ICSSG_SD_SAMP_CH_BUF_SZ          ( 128 )
#define NUM_CH_SUPPORTED_PER_AXIS        ( 3 )
#define SDFM_NINE_CH_MASK                ( 0x1FF )
#define SDFM_CH_MASK_FOR_CH0_CH3_CH6     ( 0x49 )
#define SDFM_CH_MASK_FOR_CH1_CH4_CH7     ( 0x92 )
#define SDFM_CH_MASK_FOR_CH2_CH5_CH8     ( 0x124 )
#define SDFM_NUM_OF_CH_PER_PRU_SLICE     (9)
#define NUM_OF_PRU_CORE_PER_PRU_SLICE    ( 3 )

/*SDFM Channel IDs*/
#define SDFM_CHANNEL0    (0)
#define SDFM_CHANNEL1    (1)
#define SDFM_CHANNEL2    (2)
#define SDFM_CHANNEL3    (3)
#define SDFM_CHANNEL4    (4)
#define SDFM_CHANNEL5    (5)
#define SDFM_CHANNEL6    (6)
#define SDFM_CHANNEL7    (7)
#define SDFM_CHANNEL8    (8)

#define BF_SDFM_EN_ENABLE               (1)

/*SDFM firmware version mask*/
#define SDFM_FW_VERSION_BIT_SHIFT       (32)

/*Fast detect ERROR mask*/
#define SDFM_FD_ERROR_MASK_FOR_TRIP_VEC      ( 0x3800000 )

#define SDFM_PHASE_DELAY_ACK_BIT_MASK   (1)
#define SDFM_PHASE_DELAY_CAL_LOOP_SIZE  (8)

#define SDFM_IEP_CMP1_EN_SHIFT     (2)
#define SDFM_IEP_CMP2_EN_SHIFT     (3)

#define SDFM_PRU_CORE_INDX          0U
#define SDFM_RTUPRU_CORE_INDX       1U
#define SDFM_TXPRU_CORE_INDX        2U

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
    volatile uint64_t   sampleBufferBaseAdd;

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
    volatile uint8_t     highThStatus;
    /**<  High Threshold status*/
    volatile uint8_t     lowThStatus;
    /**<  Zero cross enable bit*/
    volatile uint8_t    zeroCrossEn;
    /**<  Zero cross Threshold status */
    volatile uint8_t    zeroCrossThstatus;
    /**< Zero Cross Threshold*/
    volatile uint32_t    zeroCrossTh;
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
    volatile uint32_t  sdfmClock;
    
    /**< Enable comparator for this channel */
    volatile uint8_t  enable_comparator;
    
    /**< Enable fast detect for this channel */
    volatile uint8_t   enFastDetect;
    
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
typedef struct SDFM_Handle_Config_s {
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
    
} SDFM_Handle_Config;
/**
 *    \brief    Handle to the SDFM driver object
 *
 */
typedef struct SDFM_Handle_Config_s *SDFM_Handle;

/**
 *    \brief    Structure defining SDFM initialization parameters.
 *
 */
typedef struct SDFM_Params_s
{
    PRUICSS_Handle pruicss_handle; /**< PRU ICSS Handle */
    PRUICSS_PWM_Handle pwm_handle; /**< PRU PWM Handle */
    uint32_t load_share_enable; /**< Enable load sharing between PRUs */
    uint8_t enable_snoop_mode[NUM_OF_PRU_CORE_PER_PRU_SLICE]; /**< Enable snoop mode for normal current sampling */
    SDFM_CfgTrigger trigger_config[NUM_OF_PRU_CORE_PER_PRU_SLICE];
    SDFM_ChannelConfig channels[SDFM_NUM_OF_CH_PER_PRU_SLICE];
    uint32_t pru_slice_value; /**< PRUx slice being used */
    uint32_t iep_instance;     /**< IEP instance (0 for IEP0, 1 for IEP1) */
    uint32_t iep_inc_value; /**< Increment value of IEP counter */
    uint32_t iep_reset_freq; /**< IEP reset frequency in Hz */
    uint32_t pru_clock; /**< PRU core clock frequency in Hz */
    uint32_t iep_clock; /**< IEP clock frequency in Hz */
    uint16_t sdfm_channel_mask; /**< SDFM channel mask to indicate active channels */
    uint8_t sdfm_enable_pru_core_mask; /**< PRU core mask to indicate which PRU cores are enabled */
    uint8_t sdfm_enable_epwm_sync; /**< Enable ePWM sync for normal current sampling */
    uint8_t sdfm_epwm_sync_source; /**< ePWM sync source selection */
    uint32_t samplesBaseAddress; /**< output samples base address*/
    uint8_t sdfm_enable_phase_delay; /**< Enable phase delay measurement */
    uint8_t sdfm_phase_delay; /**< Measured phase delay */
}SDFM_Params;

#include "sdfm_api.h"

#ifdef __cplusplus
}
#endif

#endif
