/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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
#include  <math.h>


/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */


/** \brief ICSSG DMEM0/1 base addresses */
#define PRU_ICSSG_DRAM0_SLV_RAM     ( CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE )
#define PRU_ICSSG_DRAM1_SLV_RAM     ( CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE )

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

/*SDFM firmware version mask*/
#define SDFM_FW_VERSION_BIT_SHIFT       (32)

/*Fast detect ERROR mask*/
#define SDFM_FD_ERROR_MASK_FOR_TRIP_VEC      ( 0x3800000 )

#define SDFM_PHASE_DELAY_ACK_BIT_MASK   (1)
#define SDFM_PHASE_DELAY_CAL_LOOP_SIZE  (8)

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 *    \brief    Structure defining SDFM clock configuration parameters.
 *
 *    \details  Firmware SD clock configuration interface exposed through PRU data <br>
 *              memory - used by driver to configure firmware parameters
 */
typedef struct SDFM_CfgSdClk_s
{
    /**< clock count to generate SD clock (eCAP PWM) with desired frequency  */
    volatile uint8_t  sd_prd_clocks;
    /**< invert SD clock post clock selection mux  */
    volatile uint8_t  sd_clk_inv;
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
    /**< enable continuous mode */
    volatile  uint8_t  en_continuous_mode;
    /**< enable double update */
    volatile uint8_t    en_double_nc_sampling;
    /**< First sample starting point */
    volatile uint32_t first_samp_trig_time;
    /**<Second sample starting point*/
    volatile uint32_t second_samp_trig_time;
    /**< IEP0 counts in normal current sampling period*/
    volatile uint32_t nc_prd_iep_cnt;
} SDFM_CfgTrigger;

/**
 *    \brief    Structure defining SDFM IEP configuration
 *
 *    \details  Increment value of IEP counter (IEP0 default increment=1) <br>
 *              IEP CMP0 count for simulated EPWM period  <br>
 */
typedef struct SDFM_CfgIep_s
{
    /**< bit-field containing flags indicating configurations to be executed, non-zero to enable */
    volatile uint8_t iep_inc_value;

    /**< IEP CMP0 count for simulated EPWM period */
    volatile  uint32_t  cnt_epwm_prd;

}SDFM_CfgIep;


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
 *    \brief    Structure defining SDFM channel control fields
 *
 *    \details  Used by driver to enable / disable individual SD <br>
 *              channels.
 */
typedef struct SDFM_ChCtrl_s
{
    /**< stores the channel ids for different selected channel */
    volatile uint32_t    sdfm_ch_id;
    /**< bit-field to enable comparators for individual SDFM channels, BitN:ChN, non-zero to enable */
    volatile uint16_t    enable_comparator;
    /**< bit-field to enable fast detect  for individual SDFM channels, BitN:ChN, non-zero to enable */
    volatile uint8_t    enFastDetect;
     /**< enable phase delay calcualtion */
    volatile uint8_t    en_phase_delay;
    /**< Clock phase delay */
     volatile uint16_t   clock_phase_delay;
    /**<nearest clock edge status of data*/
    volatile uint16_t    clock_edge;

} SDFM_ChCtrl;

/**
 *    \brief    Structure defining clk source for sdfm ch
 *
 *    \details clk source for channel <br>
 *             iversion of clock
 */
typedef struct SDFM_ClkSourceParms_s
{
    /** < Channle clock source */
    volatile uint32_t clk_source;
    /**<clock inversion*/
    volatile uint8_t  clk_inv;
}SDFM_ClkSourceParms;

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
 *    \brief    Structure defining SDFM configuration interface
 *
 *    \details  Firmware configuration interface exposed through PRU data <br>
 *              memory - used by driver to configure firmware parameters
 */
typedef struct SDFM_Cfg_s
{
    /** < Channle id */
    volatile uint8_t   ch_id;
    /**< Filter type - sinc1, sinc2, sinc3 */
    volatile uint8_t    filter_type;
    /**< Accumulator Over Sampling Rate (OSR) */
    volatile uint8_t    osr;
    /**< sdfm threshold parms*/
    SDFM_ThresholdParms  sdfm_threshold_parms;
    /**< Fast detect window size*/
    volatile uint8_t   fd_window;
    /**< Fast detect max count of zero*/
    volatile uint8_t   fd_zero_max;
    /**< Fast detect min count of zero*/
    volatile uint8_t   fd_zero_min;
    /**< Fast detect max count of one*/
    volatile uint8_t   fd_one_max;
    /**< Fast detect min count of one*/
    volatile uint8_t   fd_one_min;
    /**< sdfm ch clock parms*/
    SDFM_ClkSourceParms  sdfm_clk_parms;
    /**< array to store the params of gpios for zero cross threshold*/
    SDFM_GpioParams        sdfm_gpio_params;
} SDFM_Cfg;

/**
 *    \brief    Structure defining SDFM control fields
 *
 *    \details  Firmware control & status interface exposed through PRU data    <br>
 *              memory - used by driver to enable SDFM operations and to select <br>
 *              between continuous and triggered mode <br>
 */
typedef struct SDFM_Ctrl_s
{
    /**< SDFM Enable */
    volatile uint8_t sdfm_en;
    /**< SDFM Enable Ack */
    volatile uint8_t  sdfm_en_ack;
    /**< SDFM PRU ID*/
    volatile uint8_t  sdfm_pru_id;
} SDFM_Ctrl;

typedef struct SDFM_Interface_s{
    /**< control interface  */
    SDFM_Ctrl       sdfm_ctrl;
    /**<iep configuration interface */
    SDFM_CfgIep    sdfm_cfg_iep_ptr;
    /**< SD modulator clock, eCAP PWM period register value */
    SDFM_CfgSdClk     sd_clk;
     /**< channel control interface */
    SDFM_ChCtrl    sdfm_ch_ctrl;
    /**< sdfm channel configuration interface pointer*/
    SDFM_Cfg        sdfm_cfg_ptr[NUM_CH_SUPPORTED_PER_AXIS];
    /*<sdfm time sampling interface pointer */
    SDFM_CfgTrigger    sdfm_cfg_trigger;
    /**< host output sample buffer base address */
    volatile uint32_t   sampleBufferBaseAdd;
    /**<firmware version */
    volatile uint64_t  firmwareVersion;
}SDFM_Interface;

typedef struct SDFM_SampleOutInterface_s
{
   uint32_t sampleOutput[NUM_CH_SUPPORTED_PER_AXIS];
}SDFM_SampleOutInterface;
/**
 *    \brief    Structure defining SDFM interface
 *
 *    \details  Firmware configuration, control, data and trigger interface exposed through PRU
 *
 */
typedef struct SDFM_s {
    /**< PRU ID */
    PRUICSS_Handle gPruIcssHandle;
    uint8_t pruId;
    uint32_t sdfmClock;
    uint32_t iepClock;
    uint32_t pruCoreClk;
    uint8_t  iepInc;
    SDFM_Interface * p_sdfm_interface;
    SDFM_SampleOutInterface *sampleOutputInterface;
    void *pruss_cfg;
} SDFM;


#include "sdfm_api.h"

#ifdef __cplusplus
}
#endif

#endif
