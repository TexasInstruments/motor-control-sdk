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

#ifndef _ICSS_SDFM_H_
#define _ICSS_SDFM_H_

/* Number of SDFM channels per PRU */
#define ICSSG_NUM_SD_CH         ( 9 )
/* Number of SDFM channels supported by PRU FW */
#define ICSSG_NUM_SD_CH_FW      ( 3 )

/* ICSSG INTC events */
/* Compile-time Host event for SDFM samples available.
   Ideally Host would provide this to FW via pseudo-register in DMEM. */
#define PRU_TRIGGER_HOST_SDFM_EVT   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define RTU_TRIGGER_HOST_SDFM_EVT   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */


/*
    Firmware registers
*/

/* FW register base addresses */
#define PRU0_DMEM                                   ( 0x0000 )



/* Base address for SDFM control parameters in DMEM */
#define ICSSG_SDFM_CTRL_BASE                        ( PRU0_DMEM )
/* Base address for SDFM Configuration parameters in DMEM */
#define ICSSG_SDFM_CFG_BASE                         ( PRU0_DMEM + 0x0002)


/* FW register sizes (in bytes) */
/* SDFM ENABLE   */
#define SDFM_EN_SZ                       ( 1 )
/* SDFM ENABLE ACK*/
#define SDFM_EN_ACK_SZ                   ( 1 )
#define SDFM_PRU_ID_SZ                   ( 1 )

/* SDFM Configuration */
#define SDFM_CFG_IEP_CFG_SZ                  ( 1 )
#define SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_SZ     ( 4 )

#define SDFM_CFG_SD_CH_ID_SZ                 ( 4 )
#define SDFM_CFG_EN_COMP_SZ                  ( 2 )

#define SDFM_CFG_SD_CLK_SZ                   ( 2 )

#define SDFM_CFG_OSR_SZ                      ( 1 )
#define SDFM_CFG_OC_HIGH_THR_SZ              ( 4 )
#define SDFM_CFG_OC_LOW_THR_SZ               ( 4 )



#define SDFM_CFG_TRIG_SAMPLE_TIME_SZ         ( 4 )
#define SDFM_CFG_TRIG_SAMPLE_CNT_SZ          ( 2 )
#define SDFM_CFG_NC_PRD_IEP_CNT_SZ           ( 2 )
#define SDFM_CFG_OC_PRD_IEP_CNT_SZ           ( 2 )

#define SDFM_CFG_OUT_SAMP_BUF_SZ             ( 4 )
#define SDFM_CFG_GPIO_VALUE_SZ               ( 4 )
#define SDFM_CFG_GPIO_SET_ADDR_SZ            ( 4 )
#define SDFM_CFG_GPIO_CLR_ADDR_SZ            ( 4 )
#define SDFM_CFG_CURR_VAL_SZ                 ( 4 )

/* FW register offsets from base (in bytes) */
/* SDFM Control */
#define SDFM_EN_OFFSET                     ( 0x00 )
#define SDFM_EN_ACK_OFFSET                 ( 0x01 )
#define SDFM_PRU_ID_OFFSET                 ( 0x02 )

/* SDFM IEP Configuration */
#define SDFM_CFG_IEP_CFG_OFFSET              ( 0x04 )
#define SDFM_CFG_IEP_INC_OFFSET              ( 0x04 )
#define SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET ( 0x08 )

/* SDFM Clock Configuration*/
#define SDFM_CFG_SD_CLK_OFFSET               ( 0x0C )
#define SDFM_CFG_SD_CLK_INV_OFFSET           ( 0x0D )

/* SDFM Configuration*/
#define SDFM_CFG_SD_CH_ID_OFFSET             ( 0x10 )
#define SDFM_CFG_SD_EN_COMP_OFFSET           ( 0x14 )
#define SDFM_CFG_SD_EN_FD_OFFSET             ( 0x16 )
#define SDFM_CFG_SD_EN_PHASE_DELAY           ( 0x17 )
#define SDFM_CFG_SD_CLOCK_PHASE_DELAY        ( 0x18 )



/*SDFM channel offsets*/
/*Ch0 offset*/
#define SDFM_CFG_CH0_CH_ID_OFFSET            ( 0x1C )
#define SDFM_CFG_CH0_FILTER_TYPE_OFFSET      ( 0x1D )
#define SDFM_CFG_CH0_OSR_OFFSET              ( 0x1E )

#define SDFM_CFG_OC_HIGH_THR_CH0_OFFSET      ( 0x20 )
#define SDFM_CFG_OC_LOW_THR_CH0_OFFSET       ( 0x24 )

#define SDFM_CFG_CH0_FD_WD_REG_OFFSET        ( 0x2C)
#define SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET  ( 0x2D)
#define SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET  ( 0x2E)
#define SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET   ( 0x2F)
#define SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET   ( 0x30)

#define SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET     ( 0x34 )
#define SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET  ( 0x38 )

#define SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET      ( 0x3C )
#define SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET   ( 0x40 )
#define SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET   ( 0x44 )
#define SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET       ( 0x48 )
#define SDFM_CFG_LOW_THR_CH0_SET_VAL_ADDR_OFFSET    ( 0x4C )
#define SDFM_CFG_LOW_THR_CH0_CLR_VAL_ADDR_OFFSET    ( 0x50 )


/*Ch1 offsets*/
#define SDFM_CFG_CH1_CH_ID_OFFSET            ( 0x60 )
#define SDFM_CFG_CH1_FILTER_TYPE_OFFSET      ( 0x61 )
#define SDFM_CFG_CH1_OSR_OFFSET              ( 0x62 )

#define SDFM_CFG_OC_HIGH_THR_CH1_OFFSET      ( 0x64 )
#define SDFM_CFG_OC_LOW_THR_CH1_OFFSET       ( 0x68 )

#define SDFM_CFG_CH1_FD_WD_REG_OFFSET        ( 0x70 )
#define SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET  ( 0x71 )
#define SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET  ( 0x72 )
#define SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET   ( 0x73 )
#define SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET   ( 0x74 )

#define SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET     ( 0x78 )
#define SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET  ( 0x7C )

#define SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET      ( 0x80 )
#define SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET   ( 0x84 )
#define SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET   ( 0x88 )
#define SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET       ( 0x8C )
#define SDFM_CFG_LOW_THR_CH1_SET_VAL_ADDR_OFFSET    ( 0x90 )
#define SDFM_CFG_LOW_THR_CH1_CLR_VAL_ADDR_OFFSET    ( 0x94 )



/*Ch2 offsets*/
#define SDFM_CFG_CH2_CH_ID_OFFSET            ( 0xA4 )
#define SDFM_CFG_CH2_FILTER_TYPE_OFFSET      ( 0xA5 )
#define SDFM_CFG_CH2_OSR_OFFSET              ( 0xA6 )

#define SDFM_CFG_OC_HIGH_THR_CH2_OFFSET      ( 0xA8 )
#define SDFM_CFG_OC_LOW_THR_CH2_OFFSET       ( 0xAC )

#define SDFM_CFG_CH2_FD_WD_REG_OFFSET        ( 0xB4 )
#define SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET  ( 0xB5 )
#define SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET  ( 0xB6 )
#define SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET   ( 0xB7 )
#define SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET   ( 0xB8 )


#define SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET     ( 0xBC )
#define SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET  ( 0xC0 )

#define SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET      ( 0xC4 )
#define SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET   ( 0xC8 )
#define SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET   ( 0xCC )
#define SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET       ( 0xD0 )
#define SDFM_CFG_LOW_THR_CH2_SET_VAL_ADDR_OFFSET    ( 0xD4 )
#define SDFM_CFG_LOW_THR_CH2_CLR_VAL_ADDR_OFFSET    ( 0xD8 )


/*Sample timing offset*/
#define SDFM_CFG_EN_CONT_NC_MODE                    ( 0xE8 )
#define SDFM_CFG_EN_DOUBLE_UPDATE                   ( 0xEA )
#define FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME      ( 0xEC )
#define FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME     ( 0xF0 )
#define SDFM_CFG_NC_PRD_IEP_CNT_OFFSET              ( 0xF4)

/* Output sample buffer base address offset*/
#define SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET         ( 0xF8 )

/*Firmware version offset*/
#define SDFM_FIRMWARE_VERSION_OFFSET                  (0xFC)

/*Local store offset for Phase delay: 2 byte */
#define SDFM_CFG_DELAY_STORE_OFFSET                   ( 0x110 )


/*Phase delay offset */
/*Debug */
#define SDFM_DUBUG_OFFSET         ( 0x10F )

/*Output sample offset*/
#define SDFM_CFG_OUT_SAMP_BUF_OFFSET                   (0x00)
/*
    Firmware register bit fields
*/

/* SDFM_CTRL */
#define BF_SDFM_EN_MASK                             ( 0x1 )
#define BF_PRU_ID_MASK                              ( 0x3 )
#define SDFM_CTRL_BF_SDFM_EN_SHIFT                  ( 0 )
#define SDFM_CTRL_BF_SDFM_EN_MASK                   ( BF_SDFM_EN_MASK << SDFM_CTRL_BF_SDFM_EN_SHIFT )
#define SDFM_CTRL_BF_PRU_ID_SHIFT                   ( 1 )
#define SDFM_CTRL_BF_PRU_ID_MASK                    ( BF_PRU_ID_MASK << SDFM_CTRL_BF_PRU_ID_SHIFT )
/* SDFM_EN bit field */
#define BF_SDFM_EN_DISABLE                          ( 0 )
#define BF_SDFM_EN_ENABLE                           ( 1 )
/* PRU ID bit field */
#define BF_PRU_ID_0                                 ( 0 )
#define BR_PRU_ID_1                                 ( 1 )
#define BF_PRU_ID_UNINIT                            ( 2 )

/* SDFM_STAT */
#define BF_SDFM_EN_ACK_MASK                         ( 0x1 )
#define BF_PRU_ID_ACK_MASK                          ( 0x3 )
#define SDFM_STAT_BF_SDFM_EN_ACK_SHIFT              ( 0 )
#define SDFM_STAT_BF_SDFM_EN_ACK_MASK               ( BF_SDFM_EN_ACK_MASK << SDFM_STAT_BF_SDFM_EN_ACK_SHIFT )
#define SDFM_STAT_BF_PRU_ID_ACK_SHIFT               ( 1 )
#define SDFM_STAT_BF_PRU_ID_ACK_MASK                ( BF_PRU_ID_ACK_MASK << SDFM_STAT_BF_PRU_ID_ACK_SHIFT )

/* IEP_CFG */
#define BF_IEP_DEFAULT_INC_MASK                     ( 0xF )
#define IEP_CFG_BF_IEP_DEFAULT_INC_SHIFT            ( 0 )
#define IEP_CFG_BF_IEP_DEFAULT_INC_MASK             ( BF_IEP_DEFAULT_INC_MASK << IEP_CFG_BF_IEP_DEFAULT_INC_SHIFT )

/* IEP_CFG_EPWM_PRD */
#define BF_CMP0_CNT_EPWM_PRD_MASK                   ( 0xFFFFFFFF )
#define IEP_CFG_BF_CMP0_CNT_EPWM_PRD_SHIFT          ( 0 )
#define IEP_CFG_BF_CMP0_CNT_EPWM_PRD_MASK           ( BF_CMP0_CNT_EPWM_PRD_MASK << IEP_CFG_BF_CMP0_CNT_EPWM_PRD_SHIFT )

/* SDFM_CFG_SD_CH_ID */
#define BF_SD_CH0_ID_MASK                           ( 0xF )
#define BF_SD_CH1_ID_MASK                           ( 0xF )
#define BF_SD_CH2_ID_MASK                           ( 0xF )
#define SDFM_CFG_BF_SD_CH0_ID_SHIFT                 ( 0 )
#define SDFM_CFG_BF_SD_CH1_ID_SHIFT                 ( 4 )
#define SDFM_CFG_BF_SD_CH2_ID_SHIFT                 ( 8 )
#define SDFM_CFG_BF_SD_CH0_ID_MASK                  ( BF_SD_CH0_ID_MASK << SDFM_CFG_BF_SD_CH0_ID_SHIFT )
#define SDFM_CFG_BF_SD_CH1_ID_MASK                  ( BF_SD_CH1_ID_MASK << SDFM_CFG_BF_SD_CH1_ID_SHIFT )
#define SDFM_CFG_BF_SD_CH2_ID_MASK                  ( BF_SD_CH2_ID_MASK << SDFM_CFG_BF_SD_CH2_ID_SHIFT )

/* SDFM_CFG_SD_CLK */
#define BF_SD_PRD_CLOCKS_MASK                       ( 0xFF )
#define BF_SD_CLK_INV_MASK                          ( 0x1 )
#define SDFM_CFG_SD_CLK_BF_SD_PRD_CLOCKS_SHIFT      ( 0 )
#define SDFM_CFG_SD_CLK_BF_SD_PRD_CLOCKS_MASK       ( BF_SD_PRD_CLOCKS_MASK << SDFM_CFG_SD_CLK_BF_SD_PRD_CLOCKS_SHIFT )
#define SDFM_CFG_SD_CLK_BF_SD_CLK_INV_SHIFT         ( 8 )
#define SDFM_CFG_SD_CLK_BF_SD_CLK_INV_MASK          ( BF_SD_CLK_INV_MASK << SDFM_CFG_SD_CLK_BF_SD_CLK_INV_SHIFT )

/* SDFM_CFG_OSR */
#define BF_OC_OSR_MASK                              ( 0xFF )
#define SDFM_CFG_OC_OSR_BF_OSR_SHIFT                ( 0 )
#define SDFM_CFG_OC_OSR_BF_OSR_MASK                 ( BF_OC_OSR_MASK << SDFM_CFG_OC_OSR_BF_OSR_SHIFT )

/* SDFM_CFG_OC_POS_THR */
#define BF_OC_POS_THR_MASK                          ( 0xFF )
#define SDFM_CFG_OC_POS_THR_SHIFT                   ( 0 )
#define SDFM_CFG_OC_POS_THR_MASK                    ( BF_OC_POS_THR_MASK << SDFM_CFG_OC_POS_THR_SHIFT )

/* SDFM_CFG_OC_NEG_THR */
#define BF_OC_NEG_THR_MASK                          ( 0xFF )
#define SDFM_CFG_OC_NEG_THR_SHIFT                   ( 0 )
#define SDFM_CFG_OC_NEG_THR_MASK                    ( BF_OC_NEG_THR_MASK << SDFM_CFG_OC_NEG_THR_SHIFT )

/* SDFM_CFG_SD_EN_COMP */
#define SDFM_CFG_EN_COMP_BIT                            ( 0x00 )
#define SDFM_CFG_BF_SD_CH0_EN_COMP_BIT                  ( 0x01 )
#define SDFM_CFG_BF_SD_CH1_EN_COMP_BIT                  ( 0x02 )
#define SDFM_CFG_BF_SD_CH2_EN_COMP_BIT                  ( 0x03 )

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


/* SDFM_CFG_TRIG_SAMP_TIME */
#define BF_TRIG_SAMP_TIME_MASK                      ( 0xFFFF )
#define SDFM_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_SHIFT \
    ( 0 )
#define SDFM_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_MASK  \
    ( BF_TRIG_SAMP_TIME_MASK << SDFM_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_SHIFT )

/* SDFM_CFG_TRIG_SAMPLE_CNT */
#define BF_TRIG_SAMP_CNT_MASK                       ( 0xFFFF )
#define SDFM_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_SHIFT   \
    ( 0 )
#define SDFM_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_MASK    \
    ( BF_TRIG_SAMP_CNT_MASK << SDFM_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_SHIFT )

/* SDFM_CFG_NC_PRD_IEP_CNT */
#define BF_NC_PRD_IEP_CNT_MASK                      ( 0xFF )
#define SDFM_CFG_NC_PRD_IEP_CNT_SHIFT               ( 0 )
#define SDFM_CFG_NC_PRD_IEP_CNT_MASK                ( BF_NC_PRD_IEP_CNT_MASK << SDFM_CFG_NC_PRD_IEP_CNT_SHIFT )

/* SDFM_CFG_NC_OUT_SAMP_BUF */
#define BF_NC_OUT_SAMP_BUF_MASK                     ( 0xFFFF )
#define SDFM_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_SHIFT \
    ( 0 )
#define SDFM_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_MASK \
    ( BF_NC_OUT_SAMP_BUF_MASK << SDFM_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_SHIFT )


#endif