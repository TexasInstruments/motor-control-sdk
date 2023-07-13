/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ICSS_SDDF_H_
#define _ICSS_SDDF_H_

/* Number of SDDF channels per PRU */
#define ICSSG_NUM_SD_CH         ( 9 )
/* Number of SDDF channels supported by PRU FW */
#define ICSSG_NUM_SD_CH_FW      ( 3 )

/* ICSSG INTC events */
/* Compile-time Host event for SDDF samples available.
   Ideally Host would provide this to FW via pseudo-register in DMEM. */
#define PRU_TRIGGER_HOST_SDDF_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define RTU_TRIGGER_HOST_SDDF_EVT   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */


/*
    Firmware registers
*/

/* FW register base addresses */
#define PRU_ICSSG_SDDF_CTRL_BASE                    ( 0x0000 )
#define PRU_ICSSG_SDDF_CFG_BASE                     ( 0x0002 )
#define RTU_ICSSG_SDDF_CTRL_BASE                    ( 0x0080 )
#define RTU_ICSSG_SDDF_CFG_BASE                     ( 0x0082 )

#if defined (SDDF_PRU_CORE)
/* SDDF Control */
#define ICSSG_SDDF_CTRL_BASE                        ( PRU_ICSSG_SDDF_CTRL_BASE )
/* SDDF Configuration */
#define ICSSG_SDDF_CFG_BASE                         ( PRU_ICSSG_SDDF_CFG_BASE )
#elif defined (SDDF_RTU_CORE)
/* SDDF Control */
#define ICSSG_SDDF_CTRL_BASE                        ( RTU_ICSSG_SDDF_CTRL_BASE )
/* SDDF Configuration */
#define ICSSG_SDDF_CFG_BASE                         ( RTU_ICSSG_SDDF_CFG_BASE )
#else
#endif

/* FW register sizes (in bytes) */
/* SDDF Control */
#define FW_REG_SDDF_CTRL_SZ                         ( 1 )
#define FW_REG_SDDF_STAT_SZ                         ( 1 )

/* SDDF Configuration */
#define FW_REG_SDDF_CFG_IEP_CFG_SZ                  ( 1 )
#define FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD_SZ     ( 4 )
#define FW_REG_SDDF_CFG_SD_CH_ID_SZ                 ( 4 )
#define FW_REG_SDDF_CFG_SD_CLK_SZ                   ( 2 )
#define FW_REG_SDFM_CFG_OSR_SZ                      ( 1 )
#define FW_REG_SDFM_CFG_EN_COMP_SZ                  ( 2 )
#define FW_REG_SDFM_CFG_OC_HIGH_THR_SZ              ( 2 )
#define FW_REG_SDFM_CFG_OC_LOW_THR_SZ               ( 2 )
#define FW_REG_SDFM_CFG_ZC_ENABLE_SZ                ( 2 )
#define FW_REG_SDFM_CFG_ZC_THR_SZ                   ( 4 )
#define FW_REG_SDFM_CFG_ZC_TRIP_STATUS_SZ           ( 1 )
#define FW_REG_SDFM_CFG_ZC_PREV_VAL_SZ              ( 4 )
#define FW_REG_SDDF_CFG_OSR_SZ                      ( 1 )
#define FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME_SZ         ( 4 )
#define FW_REG_SDDF_CFG_TRIG_SAMPLE_CNT_SZ          ( 2 )
#define FW_REG_SDDF_CFG_NC_PRD_IEP_CNT_SZ           ( 2 )
#define FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_SZ           ( 2 )
#define FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD_SZ     ( 4 )
#define FW_REG_SDDF_CFG_OUT_SAMP_BUF_SZ             ( 4 )
#define FW_REG_SDFM_CFG_GPIO_VALUE_SZ               ( 4 )
#define FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ            ( 4 )
#define FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ            ( 4 )
#define FW_REG_SDFM_CFG_CURR_VAL_SZ                 ( 4 )

/* FW register offsets from base (in bytes) */
/* SDDF Control */
#define FW_REG_SDDF_CTRL_OFFSET                     ( 0x00 )
#define FW_REG_SDDF_STAT_OFFSET                     ( 0x01 )

/* SDDF Configuration */
#define FW_REG_SDDF_CFG_IEP_CFG_OFFSET              ( 0x00 )
#define FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET ( 0x06 )
#define FW_REG_SDDF_CFG_SD_CLK_OFFSET               ( 0x0A )
#define FW_REG_SDDF_CFG_SD_CH_ID_OFFSET             ( 0x0E )
#define FW_REG_SDFM_CFG_SD_EN_COMP_OFFSET           ( 0x12 )
#define FW_REG_SDFM_CFG_SD_ZC_ENABLE_OFFSET         ( 0x16 )
#define FW_REG_SDFM_CFG_ZC_START_SZ                 ( 0x18 )

/*SDFM channel offsets*/
/*ch0 offset*/
#define FW_REG_SDDF_CFG_CH0_CH_ID_OFFSET            ( 0x1A )
#define FW_REG_SDDF_CFG_CH0_FILTER_TYPE_OFFSET      ( 0x1B )
#define FW_REG_SDDF_CFG_CH0_OSR_OFFSET              ( 0x1C )
#define FW_REG_SDFM_CFG_OC_HIGH_THR_CH0_OFFSET      ( 0x1E )
#define FW_REG_SDFM_CFG_OC_LOW_THR_CH0_OFFSET       ( 0x22 )
#define FW_REG_SDFM_CFG_OC_ZC_THR_CH0_OFFSET        ( 0x26 )
#define FW_REG_SDFM_CFG_ZC_TRIP_STATUS_CH0_OFFSET   ( 0x2A )
#define SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET             ( 0x2E )
#define FW_REG_SDDF_CFG_CH0_CLOCK_SOURCE_OFFSET     ( 0x32 )
#define FW_REG_SDDF_CFG_CH0_CLOCK_INVERSION_OFFSET  ( 0x36 )
#define SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET      ( 0x3A )
#define SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET   ( 0x3E )
#define SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET   ( 0x42 )
#define SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET       ( 0x46 )
#define SDFM_CFG_LOW_THR_CH0_SET_VAL_ADDR_OFFSET    ( 0x4A )
#define SDFM_CFG_LOW_THR_CH0_CLR_VAL_ADDR_OFFSET    ( 0x4E )
#define SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET        ( 0x52 )
#define SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET     ( 0x56 )
#define SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET     ( 0x5A )

/*ch1 offsets*/
#define FW_REG_SDDF_CFG_CH1_CH_ID_OFFSET            ( 0x5E )
#define FW_REG_SDDF_CFG_CH1_FILTER_TYPE_OFFSET      ( 0x5F )
#define FW_REG_SDDF_CFG_CH1_OSR_OFFSET              ( 0x60 )
#define FW_REG_SDFM_CFG_OC_HIGH_THR_CH1_OFFSET      ( 0x62 )
#define FW_REG_SDFM_CFG_OC_LOW_THR_CH1_OFFSET       ( 0x66 )
#define FW_REG_SDFM_CFG_OC_ZC_THR_CH1_OFFSET        ( 0x6A )
#define FW_REG_SDFM_CFG_ZC_TRIP_STATUS_CH1_OFFSET   ( 0x6E )
#define SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET             ( 0x72 )
#define FW_REG_SDDF_CFG_CH1_CLOCK_SOURCE_OFFSET     ( 0x76 )
#define FW_REG_SDDF_CFG_CH1_CLOCK_INVERSION_OFFSET  ( 0x7A )
#define SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET      ( 0x7E )
#define SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET   ( 0x82 )
#define SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET   ( 0x86 )
#define SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET       ( 0x8A )
#define SDFM_CFG_LOW_THR_CH1_SET_VAL_ADDR_OFFSET    ( 0x8E )
#define SDFM_CFG_LOW_THR_CH1_CLR_VAL_ADDR_OFFSET    ( 0x92 )
#define SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET        ( 0x96 )
#define SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET     ( 0x9A )
#define SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET     ( 0x9E )


/*ch2 offsets*/
#define FW_REG_SDDF_CFG_CH2_CH_ID_OFFSET            ( 0xA2 )
#define FW_REG_SDDF_CFG_CH2_FILTER_TYPE_OFFSET      ( 0xA3 )
#define FW_REG_SDDF_CFG_CH2_OSR_OFFSET              ( 0xA4 )
#define FW_REG_SDFM_CFG_OC_HIGH_THR_CH2_OFFSET      ( 0xA6 )
#define FW_REG_SDFM_CFG_OC_LOW_THR_CH2_OFFSET       ( 0xAA )
#define FW_REG_SDFM_CFG_OC_ZC_THR_CH2_OFFSET        ( 0xAE )
#define FW_REG_SDFM_CFG_ZC_TRIP_STATUS_CH2_OFFSET   ( 0xB2 )
#define SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET             ( 0xB6 )
#define FW_REG_SDDF_CFG_CH2_CLOCK_SOURCE_OFFSET     ( 0xBA )
#define FW_REG_SDDF_CFG_CH2_CLOCK_INVERSION_OFFSET  ( 0xBE )
#define SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET      ( 0xC2 )
#define SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET   ( 0xC6 )
#define SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET   ( 0xCA )
#define SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET       ( 0xCE )
#define SDFM_CFG_LOW_THR_CH2_SET_VAL_ADDR_OFFSET    ( 0xD2 )
#define SDFM_CFG_LOW_THR_CH2_CLR_VAL_ADDR_OFFSET    ( 0xD6 )
#define SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET        ( 0xDA)
#define SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET     ( 0xDE )
#define SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET     ( 0xE2 )

/*sample timing offset*/
#define FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME_OFFSET     ( 0xE6 )
#define FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_OFFSET       ( 0xEA )
#define FW_REG_SDDF_CFG_NC_PRD_IEP_CNT_OFFSET       ( 0xEC )
#define FW_REG_SDDF_CFG_SAMPLE_COUNT                ( 0xEE )
/*Sample output offset*/
#define FW_REG_SDDF_CFG_OUT_SAMP_BUF_OFFSET         ( 0xF2 )


/*Debug Offset */
#define DUBG_OFFSET                                   (0x110)

/* FW register addresses */

/* Sigma Delta Filter Control */
#define FW_REG_SDDF_CTRL                            ( ICSSG_SDDF_CTRL_BASE + FW_REG_SDDF_CTRL_OFFSET )
/* Sigma Delta Filter Status */
#define FW_REG_SDDF_STAT                            ( ICSSG_SDDF_CTRL_BASE + FW_REG_SDDF_STAT_OFFSET )

/* Sigma Delta Filter Configuration, IEP configuration */
#define FW_REG_SDDF_CFG_IEP_CFG                     ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_IEP_CFG_OFFSET )
/* Sigma Delta Filter Configuration, IEP CMP0 count for simulated EPWM period */
#define FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD        ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET )
/* Sigma Delta Filter Configuration, SD channel IDs */
#define FW_REG_SDDF_CFG_SD_CH_ID                    ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_SD_CH_ID_OFFSET )
/* Sigma Delta Filter Configuration, SD clock */
#define FW_REG_SDDF_CFG_SD_CLK                      ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_SD_CLK_OFFSET )
/* Sigma Delta Filter Configuration, OSR */
#define FW_REG_SDDF_CFG_OSR                         ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_OSR_OFFSET )
/* Sigma Delta Filter Configuration, positive threshold for OC detect */
#define FW_REG_SDDF_CFG_OC_POS_THR                  ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_OC_POS_THR_OFFSET )
/* Sigma Delta Filter Configuration, negative threshold for OC detect */
#define FW_REG_SDDF_CFG_OC_NEG_THR                  ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_OC_NEG_THR_OFFSET )
/* Sigma Delta Filter Configuration, Trigger Sample Time */
#define FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME            ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME_OFFSET )
/* Sigma Delta Filter Configuration, Trigger Sample Count */
#define FW_REG_SDDF_CFG_OC_PRD_IEP_CNT             ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_OFFSET )
/* Sigma Delta Filter Configuration, NC sampling period IEP counts */
#define FW_REG_SDDF_CFG_NC_PRD_IEP_CNT              ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_NC_PRD_IEP_CNT_OFFSET )
/* Sigma Delta Filter Configuration, Host Sample Output Buffer */
#define FW_REG_SDDF_CFG_OUT_SAMP_BUF                ( ICSSG_SDDF_CFG_BASE + FW_REG_SDDF_CFG_OUT_SAMP_BUF_OFFSET )

/*
    Firmware register bit fields
*/

/* SDDF_CTRL */
#define BF_SDDF_EN_MASK                             ( 0x1 )
#define BF_PRU_ID_MASK                              ( 0x3 )
#define SDDF_CTRL_BF_SDDF_EN_SHIFT                  ( 0 )
#define SDDF_CTRL_BF_SDDF_EN_MASK                   ( BF_SDDF_EN_MASK << SDDF_CTRL_BF_SDDF_EN_SHIFT )
#define SDDF_CTRL_BF_PRU_ID_SHIFT                   ( 1 )
#define SDDF_CTRL_BF_PRU_ID_MASK                    ( BF_PRU_ID_MASK << SDDF_CTRL_BF_PRU_ID_SHIFT )
/* SDDF_EN bit field */
#define BF_SDDF_EN_DISABLE                          ( 0 )
#define BF_SDDF_EN_ENABLE                           ( 1 )
/* PRU ID bit field */
#define BF_PRU_ID_0                                 ( 0 )
#define BR_PRU_ID_1                                 ( 1 )
#define BF_PRU_ID_UNINIT                            ( 2 )

/* SDDF_STAT */
#define BF_SDDF_EN_ACK_MASK                         ( 0x1 )
#define BF_PRU_ID_ACK_MASK                          ( 0x3 )
#define SDDF_STAT_BF_SDDF_EN_ACK_SHIFT              ( 0 )
#define SDDF_STAT_BF_SDDF_EN_ACK_MASK               ( BF_SDDF_EN_ACK_MASK << SDDF_STAT_BF_SDDF_EN_ACK_SHIFT )
#define SDDF_STAT_BF_PRU_ID_ACK_SHIFT               ( 1 )
#define SDDF_STAT_BF_PRU_ID_ACK_MASK                ( BF_PRU_ID_ACK_MASK << SDDF_STAT_BF_PRU_ID_ACK_SHIFT )

/* IEP_CFG */
#define BF_IEP_DEFAULT_INC_MASK                     ( 0xF )
#define IEP_CFG_BF_IEP_DEFAULT_INC_SHIFT            ( 0 )
#define IEP_CFG_BF_IEP_DEFAULT_INC_MASK             ( BF_IEP_DEFAULT_INC_MASK << IEP_CFG_BF_IEP_DEFAULT_INC_SHIFT )

/* IEP_CFG_EPWM_PRD */
#define BF_CMP0_CNT_EPWM_PRD_MASK                   ( 0xFFFFFFFF )
#define IEP_CFG_BF_CMP0_CNT_EPWM_PRD_SHIFT          ( 0 )
#define IEP_CFG_BF_CMP0_CNT_EPWM_PRD_MASK           ( BF_CMP0_CNT_EPWM_PRD_MASK << IEP_CFG_BF_CMP0_CNT_EPWM_PRD_SHIFT )

/* SDDF_CFG_SD_CH_ID */
#define BF_SD_CH0_ID_MASK                           ( 0xF )
#define BF_SD_CH1_ID_MASK                           ( 0xF )
#define BF_SD_CH2_ID_MASK                           ( 0xF )
#define SDDF_CFG_BF_SD_CH0_ID_SHIFT                 ( 0 )
#define SDDF_CFG_BF_SD_CH1_ID_SHIFT                 ( 4 )
#define SDDF_CFG_BF_SD_CH2_ID_SHIFT                 ( 8 )
#define SDDF_CFG_BF_SD_CH0_ID_MASK                  ( BF_SD_CH0_ID_MASK << SDDF_CFG_BF_SD_CH0_ID_SHIFT )
#define SDDF_CFG_BF_SD_CH1_ID_MASK                  ( BF_SD_CH1_ID_MASK << SDDF_CFG_BF_SD_CH1_ID_SHIFT )
#define SDDF_CFG_BF_SD_CH2_ID_MASK                  ( BF_SD_CH2_ID_MASK << SDDF_CFG_BF_SD_CH2_ID_SHIFT )

/* SDDF_CFG_SD_CLK */
#define BF_SD_PRD_CLOCKS_MASK                       ( 0xFF )
#define BF_SD_CLK_INV_MASK                          ( 0x1 )
#define SDDF_CFG_SD_CLK_BF_SD_PRD_CLOCKS_SHIFT      ( 0 )
#define SDDF_CFG_SD_CLK_BF_SD_PRD_CLOCKS_MASK       ( BF_SD_PRD_CLOCKS_MASK << SDDF_CFG_SD_CLK_BF_SD_PRD_CLOCKS_SHIFT )
#define SDDF_CFG_SD_CLK_BF_SD_CLK_INV_SHIFT         ( 8 )
#define SDDF_CFG_SD_CLK_BF_SD_CLK_INV_MASK          ( BF_SD_CLK_INV_MASK << SDDF_CFG_SD_CLK_BF_SD_CLK_INV_SHIFT )

/* SDDF_CFG_OSR */
#define BF_OC_OSR_MASK                              ( 0xFF )
#define SDDF_CFG_OC_OSR_BF_OSR_SHIFT                ( 0 )
#define SDDF_CFG_OC_OSR_BF_OSR_MASK                 ( BF_OC_OSR_MASK << SDDF_CFG_OC_OSR_BF_OSR_SHIFT )

/* SDDF_CFG_OC_POS_THR */
#define BF_OC_POS_THR_MASK                          ( 0xFF )
#define SDDF_CFG_OC_POS_THR_SHIFT                   ( 0 )
#define SDDF_CFG_OC_POS_THR_MASK                    ( BF_OC_POS_THR_MASK << SDDF_CFG_OC_POS_THR_SHIFT )

/* SDDF_CFG_OC_NEG_THR */
#define BF_OC_NEG_THR_MASK                          ( 0xFF )
#define SDDF_CFG_OC_NEG_THR_SHIFT                   ( 0 )
#define SDDF_CFG_OC_NEG_THR_MASK                    ( BF_OC_NEG_THR_MASK << SDDF_CFG_OC_NEG_THR_SHIFT )

/* SDFM_CFG_SD_EN_COMP */
#define SDFM_CFG_BF_SD_CH0_EN_COMP_BIT                  ( 0x00 )
#define SDFM_CFG_BF_SD_CH1_EN_COMP_BIT                  ( 0x01 )
#define SDFM_CFG_BF_SD_CH2_EN_COMP_BIT                  ( 0x02 )

/* SDFM_CFG_ZC_ENABLE */
#define SDFM_CFG_BF_SD_CH0_ZC_ENABLE_BIT                  ( 0x00 )
#define SDFM_CFG_BF_SD_CH1_ZC_ENABLE_BIT                  ( 0x01 )
#define SDFM_CFG_BF_SD_CH2_ZC_ENABLE_BIT                  ( 0x02 )


/* SDFM_CFG_ZC_START */
#define SDFM_CFG_BF_SD_CH0_ZC_START_BIT                  ( 0x00 )
#define SDFM_CFG_BF_SD_CH1_ZC_START_BIT                  ( 0x01 )
#define SDFM_CFG_BF_SD_CH2_ZC_START_BIT                  ( 0x02 )


/* SDFM_CFG_TRIP_STATUS */
#define SDFM_CFG_BF_SD_TRIP_STATUS_HIGH                  ( 0x01 )
#define SDFM_CFG_BF_SD_TRIP_STATUS_LOW                   ( 0x00 )


/* SDDF_CFG_TRIG_SAMP_TIME */
#define BF_TRIG_SAMP_TIME_MASK                      ( 0xFFFF )
#define SDDF_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_SHIFT \
    ( 0 )
#define SDDF_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_MASK  \
    ( BF_TRIG_SAMP_TIME_MASK << SDDF_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_SHIFT )

/* SDDF_CFG_TRIG_SAMPLE_CNT */
#define BF_TRIG_SAMP_CNT_MASK                       ( 0xFFFF )
#define SDDF_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_SHIFT   \
    ( 0 )
#define SDDF_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_MASK    \
    ( BF_TRIG_SAMP_CNT_MASK << SDDF_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_SHIFT )

/* SDDF_CFG_NC_PRD_IEP_CNT */
#define BF_NC_PRD_IEP_CNT_MASK                      ( 0xFF )
#define SDDF_CFG_NC_PRD_IEP_CNT_SHIFT               ( 0 )
#define SDDF_CFG_NC_PRD_IEP_CNT_MASK                ( BF_NC_PRD_IEP_CNT_MASK << SDDF_CFG_NC_PRD_IEP_CNT_SHIFT )

/* SDDF_CFG_NC_OUT_SAMP_BUF */
#define BF_NC_OUT_SAMP_BUF_MASK                     ( 0xFFFF )
#define SDDF_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_SHIFT \
    ( 0 )
#define SDDF_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_MASK \
    ( BF_NC_OUT_SAMP_BUF_MASK << SDDF_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_SHIFT )

#endif