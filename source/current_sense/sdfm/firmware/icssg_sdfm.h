/*
 * Copyright (c) 2025, Texas Instruments Incorporated
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

/**
 *  \file   icssg_sdfm.h
 *
 *  \brief  SDFM firmware/driver shared memory layout definitions.
 *
 *  \details
 *  This header defines the DMEM memory offsets used for communication between
 *  the R5F host driver and the PRU SDFM firmware. It includes INTC event
 *  definitions, channel configuration offsets, trigger mode offsets, and
 *  local storage offsets for each PRU core type (RTU, PRU, TX_PRU).
 *
 *  The memory layout must match exactly between firmware and driver to ensure
 *  correct operation.
 */

/* ICSSG INTC system event numbers (base event + PRU-specific offset of 18) */
#define ICSS_SDFM_TRIGGER_EVNT_CH0   ( 3+18 )   
#define ICSS_SDFM_TRIGGER_EVNT_CH1   ( 4+18 )   
#define ICSS_SDFM_TRIGGER_EVNT_CH2   ( 5+18 )    
#define ICSS_SDFM_TRIGGER_EVNT_CH3   ( 6+18 )   
#define ICSS_SDFM_TRIGGER_EVNT_CH4   ( 7+18 )   
#define ICSS_SDFM_TRIGGER_EVNT_CH5   ( 8+18 )   
#define ICSS_SDFM_TRIGGER_EVNT_CH6   ( 9+18 )  
#define ICSS_SDFM_TRIGGER_EVNT_CH7   ( 10+18 )    
#define ICSS_SDFM_TRIGGER_EVNT_CH8   ( 11+18 ) 

/* Number of SDFM channels per PRU */
#if defined (SDFM_LOAD_SHARE_MODE)
#define ICSS_PRU_MAX_NUM_OF_SD_CH         ( 3 )
#if defined (SDFM_PRU_CORE)
#define ICSS_PRU_SD_FIRST_CH              ( 3 )
#elif defined (SDFM_RTU_CORE)
#define ICSS_PRU_SD_FIRST_CH              ( 0 )
#elif defined (SDFM_TXPRU_CORE)
#define ICSS_PRU_SD_FIRST_CH              ( 6 )
#endif
#else
#define ICSS_PRU_SD_FIRST_CH              ( 0 )
#define ICSS_PRU_MAX_NUM_OF_SD_CH         ( 9 )
#endif

/* DMEM MEMORY OFFSETS BETWEEN CHANNELS */
#define ICSSG_SDFM_CH_MEM_OFFSET        ( 0x3C )    /* 60 bytes offset between channels */

/* DMEM MEMORY OFFSETS */

/* SDFM Control */
#if defined (SDFM_PRU_CORE)
#define SDFM_EN_OFFSET                     ( 0x00 )
#define SDFM_EN_ACK_OFFSET                 ( 0x01 )
#define SDFM_EN_NC_USING_SNOOP_REG_OFFSET  ( 0x02 )
#endif
#if defined (SDFM_RTU_CORE)
#define SDFM_EN_OFFSET                     ( 0x03 )
#define SDFM_EN_ACK_OFFSET                 ( 0x04 )
#define SDFM_EN_NC_USING_SNOOP_REG_OFFSET  ( 0x05 )
#endif
#if defined (SDFM_TXPRU_CORE)
#define SDFM_EN_OFFSET                     ( 0x06 )
#define SDFM_EN_ACK_OFFSET                 ( 0x07 )
#define SDFM_EN_NC_USING_SNOOP_REG_OFFSET  ( 0x08 )
#endif

/* common offsets for all channels */
#define SDFM_CFG_SD_CH_MASK_OFFSET           ( 0x0A )
#define SDFM_FIRMWARE_VERSION_OFFSET         ( 0x10 )

/* Trigger mode offsets */
#if defined (SDFM_PRU_CORE)
#define SDFM_CFG_EN_NC_TRIGGER_MODE                 ( 0x18 )
#define SDFM_CFG_EN_DOUBLE_UPDATE                   ( 0x19 )
#define FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME      ( 0x1C )
#define FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME     ( 0x20 )
#define SDFM_CFG_NC_PRD_IEP_CNT_OFFSET              ( 0x28 )
#define SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET        ( 0x30 )
#define SDFM_CFG_SD_CMP_EVENT_NUM_OFFSET            ( 0x34 )
#define SDFM_CFG_NC_PRD_IEP_REG_OFFSET              ( 0x38 )
#define SDFM_CFG_NC_PRD_IEP_CMP_STATUS_REG_OFFSET   ( 0x3C )
#define SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET       ( 0x40 )
#endif
#if defined (SDFM_RTU_CORE)
#define SDFM_CFG_EN_NC_TRIGGER_MODE                 ( 0x48 )
#define SDFM_CFG_EN_DOUBLE_UPDATE                   ( 0x49 )
#define FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME      ( 0x4C )
#define FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME     ( 0x50 )
#define SDFM_CFG_NC_PRD_IEP_CNT_OFFSET              ( 0x58 )
#define SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET        ( 0x60 )
#define SDFM_CFG_SD_CMP_EVENT_NUM_OFFSET            ( 0x64 )
#define SDFM_CFG_NC_PRD_IEP_REG_OFFSET              ( 0x68 )
#define SDFM_CFG_NC_PRD_IEP_CMP_STATUS_REG_OFFSET   ( 0x6C )
#define SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET       ( 0x70 )
#endif
#if defined (SDFM_TXPRU_CORE)
#define SDFM_CFG_EN_NC_TRIGGER_MODE                 ( 0x78 )
#define SDFM_CFG_EN_DOUBLE_UPDATE                   ( 0x79 )
#define FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME      ( 0x7C )
#define FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME     ( 0x80 )
#define SDFM_CFG_NC_PRD_IEP_CNT_OFFSET              ( 0x88 )
#define SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET        ( 0x90 )
#define SDFM_CFG_SD_CMP_EVENT_NUM_OFFSET            ( 0x94 )
#define SDFM_CFG_NC_PRD_IEP_REG_OFFSET              ( 0x98 )
#define SDFM_CFG_NC_PRD_IEP_CMP_STATUS_REG_OFFSET   ( 0x9C )
#define SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET       ( 0xA0 )
#endif

/* SDFM Channel offsets */
#if defined (SDFM_LOAD_SHARE_MODE)
#if defined (SDFM_RTU_CORE)
/* Channel 0 offsets*/
#define SDFM_CH0_ID_OFFSET                       (0xA8)
#define SDFM_CH0_ENABLE_OFFSET                   (0xA9)
#define SDFM_CFG_CH0_FILTER_TYPE_OFFSET          (0xAA)
#define SDFM_CH0_NC_OSR_OFFSET                   (0xAB)
#define SDFM_CH0_OC_OSR_OFFSET                   (0xAC)
#define SDFM_CH0_CLK_OFFSET                      (0xB0)
#define SDFM_CFG_SD_CH0_EN_COMP_OFFSET           (0xB4)
#define SDFM_CFG_CH0_FD_WD_REG_OFFSET            (0xB6)
#define SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET      (0xB7)
#define SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET      (0xB8)
#define SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET       (0xB9)
#define SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET       (0xBA)
#define SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET         (0xBC)
#define SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET      (0xC0)
#define SDFM_CFG_SD_CH0_EN_PHASE_DELAY           (0xC1)
#define SDFM_CFG_SD_CH0_CLOCK_PHASE_DELAY        (0xC2)
#define SDFM_CFG_SD_CH0_CLOCK_EDGE               (0xC4)
#define SDFM_CFG_OC_HIGH_THR_CH0_OFFSET          (0xC8)
#define SDFM_CFG_OC_LOW_THR_CH0_OFFSET           (0xCC)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET   (0xD0)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET    (0xD1)
#define SDFM_CFG_ZC_THR_EN_CH0_OFFSET            (0xD2)
#define SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET        (0xD3)
#define SDFM_CFG_ZC_THR_CH0_OFFSET               (0xD4)
#define SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET     (0xD8)
#define SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET  (0xDC)
#define SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET  (0xE0)

/* Channel 1 offsets*/
#define SDFM_CH1_ID_OFFSET                       (0xE4)
#define SDFM_CH1_ENABLE_OFFSET                   (0xE5)
#define SDFM_CFG_CH1_FILTER_TYPE_OFFSET          (0xE6)
#define SDFM_CH1_NC_OSR_OFFSET                   (0xE7)
#define SDFM_CH1_OC_OSR_OFFSET                   (0xE8)
#define SDFM_CH1_CLK_OFFSET                      (0xEC)
#define SDFM_CFG_SD_CH1_EN_COMP_OFFSET           (0xF0)
#define SDFM_CFG_CH1_FD_WD_REG_OFFSET            (0xF2)
#define SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET      (0xF3)
#define SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET      (0xF4)
#define SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET       (0xF5)
#define SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET       (0xF6)
#define SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET         (0xF8)
#define SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET      (0xFC)
#define SDFM_CFG_SD_CH1_EN_PHASE_DELAY           (0xFD)
#define SDFM_CFG_SD_CH1_CLOCK_PHASE_DELAY        (0xFE)
#define SDFM_CFG_SD_CH1_CLOCK_EDGE               (0x100)
#define SDFM_CFG_OC_HIGH_THR_CH1_OFFSET          (0x104)
#define SDFM_CFG_OC_LOW_THR_CH1_OFFSET           (0x108)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET   (0x10C)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET    (0x10D)
#define SDFM_CFG_ZC_THR_EN_CH1_OFFSET            (0x10E)
#define SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET        (0x10F)
#define SDFM_CFG_ZC_THR_CH1_OFFSET               (0x110)
#define SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET     (0x114)
#define SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET  (0x118)
#define SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET  (0x11C)
/* Channel 2 offsets*/
#define SDFM_CH2_ID_OFFSET                       (0x120)
#define SDFM_CH2_ENABLE_OFFSET                   (0x121)
#define SDFM_CFG_CH2_FILTER_TYPE_OFFSET          (0x122)
#define SDFM_CH2_NC_OSR_OFFSET                   (0x123)
#define SDFM_CH2_OC_OSR_OFFSET                   (0x124)
#define SDFM_CH2_CLK_OFFSET                      (0x128)
#define SDFM_CFG_SD_CH2_EN_COMP_OFFSET           (0x12C)
#define SDFM_CFG_CH2_FD_WD_REG_OFFSET            (0x12E)
#define SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET      (0x12F)
#define SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET      (0x130)
#define SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET       (0x131)
#define SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET       (0x132)
#define SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET         (0x134)
#define SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET      (0x138)
#define SDFM_CFG_SD_CH2_EN_PHASE_DELAY           (0x139)
#define SDFM_CFG_SD_CH2_CLOCK_PHASE_DELAY        (0x13A)
#define SDFM_CFG_SD_CH2_CLOCK_EDGE               (0x13C)
#define SDFM_CFG_OC_HIGH_THR_CH2_OFFSET          (0x140)
#define SDFM_CFG_OC_LOW_THR_CH2_OFFSET           (0x144)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET   (0x148)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET    (0x149)
#define SDFM_CFG_ZC_THR_EN_CH2_OFFSET            (0x14A)
#define SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET        (0x14B)
#define SDFM_CFG_ZC_THR_CH2_OFFSET               (0x14C)
#define SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET     (0x150)
#define SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET  (0x154)
#define SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET  (0x158)
#endif
#if defined (SDFM_PRU_CORE)
/* Channel 0 offsets - starting at 0x15C */
#define SDFM_CH0_ID_OFFSET                       (0x15C)
#define SDFM_CH0_ENABLE_OFFSET                   (0x15D)
#define SDFM_CFG_CH0_FILTER_TYPE_OFFSET          (0x15E)
#define SDFM_CH0_NC_OSR_OFFSET                   (0x15F)
#define SDFM_CH0_OC_OSR_OFFSET                   (0x160)
#define SDFM_CH0_CLK_OFFSET                      (0x164)
#define SDFM_CFG_SD_CH0_EN_COMP_OFFSET           (0x168)
#define SDFM_CFG_CH0_FD_WD_REG_OFFSET            (0x16A)
#define SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET      (0x16B)
#define SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET      (0x16C)
#define SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET       (0x16D)
#define SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET       (0x16E)
#define SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET         (0x170)
#define SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET      (0x174)
#define SDFM_CFG_SD_CH0_EN_PHASE_DELAY           (0x175)
#define SDFM_CFG_SD_CH0_CLOCK_PHASE_DELAY        (0x176)
#define SDFM_CFG_SD_CH0_CLOCK_EDGE               (0x178)
#define SDFM_CFG_OC_HIGH_THR_CH0_OFFSET          (0x17C)
#define SDFM_CFG_OC_LOW_THR_CH0_OFFSET           (0x180)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET   (0x184)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET    (0x185)
#define SDFM_CFG_ZC_THR_EN_CH0_OFFSET            (0x186)
#define SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET        (0x187)
#define SDFM_CFG_ZC_THR_CH0_OFFSET               (0x188)
#define SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET     (0x18C)
#define SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET  (0x190)
#define SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET  (0x194)

/* Channel 1 offsets - starting at 0x198 */
#define SDFM_CH1_ID_OFFSET                       (0x198)
#define SDFM_CH1_ENABLE_OFFSET                   (0x199)
#define SDFM_CFG_CH1_FILTER_TYPE_OFFSET          (0x19A)
#define SDFM_CH1_NC_OSR_OFFSET                   (0x19B)
#define SDFM_CH1_OC_OSR_OFFSET                   (0x19C)
#define SDFM_CH1_CLK_OFFSET                      (0x1A0)
#define SDFM_CFG_SD_CH1_EN_COMP_OFFSET           (0x1A4)
#define SDFM_CFG_CH1_FD_WD_REG_OFFSET            (0x1A6)
#define SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET      (0x1A7)
#define SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET      (0x1A8)
#define SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET       (0x1A9)
#define SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET       (0x1AA)
#define SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET         (0x1AC)
#define SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET      (0x1B0)
#define SDFM_CFG_SD_CH1_EN_PHASE_DELAY           (0x1B1)
#define SDFM_CFG_SD_CH1_CLOCK_PHASE_DELAY        (0x1B2)
#define SDFM_CFG_SD_CH1_CLOCK_EDGE               (0x1B4)
#define SDFM_CFG_OC_HIGH_THR_CH1_OFFSET          (0x1B8)
#define SDFM_CFG_OC_LOW_THR_CH1_OFFSET           (0x1BC)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET   (0x1C0)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET    (0x1C1)
#define SDFM_CFG_ZC_THR_EN_CH1_OFFSET            (0x1C2)
#define SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET        (0x1C3)
#define SDFM_CFG_ZC_THR_CH1_OFFSET               (0x1C4)
#define SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET     (0x1C8)
#define SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET  (0x1CC)
#define SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET  (0x1D0)

/* Channel 2 offsets - starting at 0x1D4 */
#define SDFM_CH2_ID_OFFSET                       (0x1D4)
#define SDFM_CH2_ENABLE_OFFSET                   (0x1D5)
#define SDFM_CFG_CH2_FILTER_TYPE_OFFSET          (0x1D6)
#define SDFM_CH2_NC_OSR_OFFSET                   (0x1D7)
#define SDFM_CH2_OC_OSR_OFFSET                   (0x1D8)
#define SDFM_CH2_CLK_OFFSET                      (0x1DC)
#define SDFM_CFG_SD_CH2_EN_COMP_OFFSET           (0x1E0)
#define SDFM_CFG_CH2_FD_WD_REG_OFFSET            (0x1E2)
#define SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET      (0x1E3)
#define SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET      (0x1E4)
#define SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET       (0x1E5)
#define SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET       (0x1E6)
#define SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET         (0x1E8)
#define SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET      (0x1EC)
#define SDFM_CFG_SD_CH2_EN_PHASE_DELAY           (0x1ED)
#define SDFM_CFG_SD_CH2_CLOCK_PHASE_DELAY        (0x1EE)
#define SDFM_CFG_SD_CH2_CLOCK_EDGE               (0x1F0)
#define SDFM_CFG_OC_HIGH_THR_CH2_OFFSET          (0x1F4)
#define SDFM_CFG_OC_LOW_THR_CH2_OFFSET           (0x1F8)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET   (0x1FC)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET    (0x1FD)
#define SDFM_CFG_ZC_THR_EN_CH2_OFFSET            (0x1FE)
#define SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET        (0x1FF)
#define SDFM_CFG_ZC_THR_CH2_OFFSET               (0x200)
#define SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET     (0x204)
#define SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET  (0x208)
#define SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET  (0x20C)
#endif
#if defined (SDFM_TXPRU_CORE)
/* Channel 0 offsets - starting at 0x210 */
#define SDFM_CH0_ID_OFFSET                       (0x210)
#define SDFM_CH0_ENABLE_OFFSET                   (0x211)
#define SDFM_CFG_CH0_FILTER_TYPE_OFFSET          (0x212)
#define SDFM_CH0_NC_OSR_OFFSET                   (0x213)
#define SDFM_CH0_OC_OSR_OFFSET                   (0x214)
#define SDFM_CH0_CLK_OFFSET                      (0x218)
#define SDFM_CFG_SD_CH0_EN_COMP_OFFSET           (0x21C)
#define SDFM_CFG_CH0_FD_WD_REG_OFFSET            (0x21E)
#define SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET      (0x21F)
#define SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET      (0x220)
#define SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET       (0x221)
#define SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET       (0x222)
#define SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET         (0x224)
#define SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET      (0x228)
#define SDFM_CFG_SD_CH0_EN_PHASE_DELAY           (0x229)
#define SDFM_CFG_SD_CH0_CLOCK_PHASE_DELAY        (0x22A)
#define SDFM_CFG_SD_CH0_CLOCK_EDGE               (0x22C)
#define SDFM_CFG_OC_HIGH_THR_CH0_OFFSET          (0x230)
#define SDFM_CFG_OC_LOW_THR_CH0_OFFSET           (0x234)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET   (0x238)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET    (0x239)
#define SDFM_CFG_ZC_THR_EN_CH0_OFFSET            (0x23A)
#define SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET        (0x23B)
#define SDFM_CFG_ZC_THR_CH0_OFFSET               (0x23C)
#define SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET     (0x240)
#define SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET  (0x244)
#define SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET  (0x248)

/* Channel 1 offsets - starting at 0x24C */
#define SDFM_CH1_ID_OFFSET                       (0x24C)
#define SDFM_CH1_ENABLE_OFFSET                   (0x24D)
#define SDFM_CFG_CH1_FILTER_TYPE_OFFSET          (0x24E)
#define SDFM_CH1_NC_OSR_OFFSET                   (0x24F)
#define SDFM_CH1_OC_OSR_OFFSET                   (0x250)
#define SDFM_CH1_CLK_OFFSET                      (0x254)
#define SDFM_CFG_SD_CH1_EN_COMP_OFFSET           (0x258)
#define SDFM_CFG_CH1_FD_WD_REG_OFFSET            (0x25A)
#define SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET      (0x25B)
#define SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET      (0x25C)
#define SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET       (0x25D)
#define SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET       (0x25E)
#define SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET         (0x260)
#define SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET      (0x264)
#define SDFM_CFG_SD_CH1_EN_PHASE_DELAY           (0x265)
#define SDFM_CFG_SD_CH1_CLOCK_PHASE_DELAY        (0x266)
#define SDFM_CFG_SD_CH1_CLOCK_EDGE               (0x268)
#define SDFM_CFG_OC_HIGH_THR_CH1_OFFSET          (0x26C)
#define SDFM_CFG_OC_LOW_THR_CH1_OFFSET           (0x270)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET   (0x274)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET    (0x275)
#define SDFM_CFG_ZC_THR_EN_CH1_OFFSET            (0x276)
#define SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET        (0x277)
#define SDFM_CFG_ZC_THR_CH1_OFFSET               (0x278)
#define SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET     (0x27C)
#define SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET  (0x280)
#define SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET  (0x284)

/* Channel 2 offsets - starting at 0x288 */
#define SDFM_CH2_ID_OFFSET                       (0x288)
#define SDFM_CH2_ENABLE_OFFSET                   (0x289)
#define SDFM_CFG_CH2_FILTER_TYPE_OFFSET          (0x28A)
#define SDFM_CH2_NC_OSR_OFFSET                   (0x28B)
#define SDFM_CH2_OC_OSR_OFFSET                   (0x28C)
#define SDFM_CH2_CLK_OFFSET                      (0x290)
#define SDFM_CFG_SD_CH2_EN_COMP_OFFSET           (0x294)
#define SDFM_CFG_CH2_FD_WD_REG_OFFSET            (0x296)
#define SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET      (0x297)
#define SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET      (0x298)
#define SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET       (0x299)
#define SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET       (0x29A)
#define SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET         (0x29C)
#define SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET      (0x2A0)
#define SDFM_CFG_SD_CH2_EN_PHASE_DELAY           (0x2A1)
#define SDFM_CFG_SD_CH2_CLOCK_PHASE_DELAY        (0x2A2)
#define SDFM_CFG_SD_CH2_CLOCK_EDGE               (0x2A4)
#define SDFM_CFG_OC_HIGH_THR_CH2_OFFSET          (0x2A8)
#define SDFM_CFG_OC_LOW_THR_CH2_OFFSET           (0x2AC)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET   (0x2B0)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET    (0x2B1)
#define SDFM_CFG_ZC_THR_EN_CH2_OFFSET            (0x2B2)
#define SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET        (0x2B3)
#define SDFM_CFG_ZC_THR_CH2_OFFSET               (0x2B4)
#define SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET     (0x2B8)
#define SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET  (0x2BC)
#define SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET  (0x2C0)
#endif
#else
/* Channel 0 offsets */
#define SDFM_CH0_ID_OFFSET                       (0xA8)
#define SDFM_CH0_ENABLE_OFFSET                   (0xA9)
#define SDFM_CFG_CH0_FILTER_TYPE_OFFSET          (0xAA)
#define SDFM_CH0_NC_OSR_OFFSET                   (0xAB)
#define SDFM_CH0_OC_OSR_OFFSET                   (0xAC)
#define SDFM_CH0_CLK_OFFSET                      (0xB0)
#define SDFM_CFG_SD_CH0_EN_COMP_OFFSET           (0xB4)
#define SDFM_CFG_CH0_FD_WD_REG_OFFSET            (0xB6)
#define SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET      (0xB7)
#define SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET      (0xB8)
#define SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET       (0xB9)
#define SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET       (0xBA)
#define SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET         (0xBC)
#define SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET      (0xC0)
#define SDFM_CFG_SD_CH0_EN_PHASE_DELAY           (0xC1)
#define SDFM_CFG_SD_CH0_CLOCK_PHASE_DELAY        (0xC2)
#define SDFM_CFG_SD_CH0_CLOCK_EDGE               (0xC4)
#define SDFM_CFG_OC_HIGH_THR_CH0_OFFSET          (0xC8)
#define SDFM_CFG_OC_LOW_THR_CH0_OFFSET           (0xCC)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET   (0xD0)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET    (0xD1)
#define SDFM_CFG_ZC_THR_EN_CH0_OFFSET            (0xD2)
#define SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET        (0xD3)
#define SDFM_CFG_ZC_THR_CH0_OFFSET               (0xD4)
#define SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET     (0xD8)
#define SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET  (0xDC)
#define SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET  (0xE0)

/* Channel 1 offsets*/
#define SDFM_CH1_ID_OFFSET                       (0xE4)
#define SDFM_CH1_ENABLE_OFFSET                   (0xE5)
#define SDFM_CFG_CH1_FILTER_TYPE_OFFSET          (0xE6)
#define SDFM_CH1_NC_OSR_OFFSET                   (0xE7)
#define SDFM_CH1_OC_OSR_OFFSET                   (0xE8)
#define SDFM_CH1_CLK_OFFSET                      (0xEC)
#define SDFM_CFG_SD_CH1_EN_COMP_OFFSET           (0xF0)
#define SDFM_CFG_CH1_FD_WD_REG_OFFSET            (0xF2)
#define SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET      (0xF3)
#define SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET      (0xF4)
#define SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET       (0xF5)
#define SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET       (0xF6)
#define SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET         (0xF8)
#define SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET      (0xFC)
#define SDFM_CFG_SD_CH1_EN_PHASE_DELAY           (0xFD)
#define SDFM_CFG_SD_CH1_CLOCK_PHASE_DELAY        (0xFE)
#define SDFM_CFG_SD_CH1_CLOCK_EDGE               (0x100)
#define SDFM_CFG_OC_HIGH_THR_CH1_OFFSET          (0x104)
#define SDFM_CFG_OC_LOW_THR_CH1_OFFSET           (0x108)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET   (0x10C)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET    (0x10D)
#define SDFM_CFG_ZC_THR_EN_CH1_OFFSET            (0x10E)
#define SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET        (0x10F)
#define SDFM_CFG_ZC_THR_CH1_OFFSET               (0x110)
#define SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET     (0x114)
#define SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET  (0x118)
#define SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET  (0x11C)
/* Channel 2 offsets*/
#define SDFM_CH2_ID_OFFSET                       (0x120)
#define SDFM_CH2_ENABLE_OFFSET                   (0x121)
#define SDFM_CFG_CH2_FILTER_TYPE_OFFSET          (0x122)
#define SDFM_CH2_NC_OSR_OFFSET                   (0x123)
#define SDFM_CH2_OC_OSR_OFFSET                   (0x124)
#define SDFM_CH2_CLK_OFFSET                      (0x128)
#define SDFM_CFG_SD_CH2_EN_COMP_OFFSET           (0x12C)
#define SDFM_CFG_CH2_FD_WD_REG_OFFSET            (0x12E)
#define SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET      (0x12F)
#define SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET      (0x130)
#define SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET       (0x131)
#define SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET       (0x132)
#define SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET         (0x134)
#define SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET      (0x138)
#define SDFM_CFG_SD_CH2_EN_PHASE_DELAY           (0x139)
#define SDFM_CFG_SD_CH2_CLOCK_PHASE_DELAY        (0x13A)
#define SDFM_CFG_SD_CH2_CLOCK_EDGE               (0x13C)
#define SDFM_CFG_OC_HIGH_THR_CH2_OFFSET          (0x140)
#define SDFM_CFG_OC_LOW_THR_CH2_OFFSET           (0x144)
#define SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET   (0x148)
#define SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET    (0x149)
#define SDFM_CFG_ZC_THR_EN_CH2_OFFSET            (0x14A)
#define SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET        (0x14B)
#define SDFM_CFG_ZC_THR_CH2_OFFSET               (0x14C)
#define SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET     (0x150)
#define SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET  (0x154)
#define SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET  (0x158)
#endif

/* DMEM memory offsets for local uses */
#if defined (SDFM_PRU_CORE) 
/*Zero cross local storage*/
#define SDFM_CFG_BF_SD_CH0_ZC_START_OFFSET               (0x2E0)
#define SDFM_CFG_BF_SD_CH1_ZC_START_OFFSET               (0x2E1)
#define SDFM_CFG_BF_SD_CH2_ZC_START_OFFSET               (0x2E2)

#define SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET               ( 0x2E4)
#define SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET               ( 0x2E8)
#define SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET               ( 0x2EC)

/*Local output sample buffer offset */
#define SDFM_LOCAL_OUTPUT_SAMPLE_BUFFER_OFFSET        (0x2F0)

/*Debug offset*/
#define SDFM_DEBUG_OFFSET         ( 0x300 )
#endif
#if defined (SDFM_RTU_CORE)
/*Zero cross local storage*/
#define SDFM_CFG_BF_SD_CH0_ZC_START_OFFSET               (0x310)
#define SDFM_CFG_BF_SD_CH1_ZC_START_OFFSET               (0x312)
#define SDFM_CFG_BF_SD_CH2_ZC_START_OFFSET               (0x313)

#define SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET               ( 0x314)
#define SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET               ( 0x318)
#define SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET               ( 0x31C)

/*Local output sample buffer offset */
#define SDFM_LOCAL_OUTPUT_SAMPLE_BUFFER_OFFSET        (0x320)

/*Debug */
#define SDFM_DEBUG_OFFSET         ( 0x330 )
#endif
#if defined (SDFM_TXPRU_CORE)
/*Zero cross local storage*/
#define SDFM_CFG_BF_SD_CH0_ZC_START_OFFSET               (0x340)
#define SDFM_CFG_BF_SD_CH1_ZC_START_OFFSET               (0x341)
#define SDFM_CFG_BF_SD_CH2_ZC_START_OFFSET               (0x342)

#define SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET               ( 0x344)
#define SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET               ( 0x348)
#define SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET               ( 0x34C)

/*Local output sample buffer offset */
#define SDFM_LOCAL_OUTPUT_SAMPLE_BUFFER_OFFSET        (0x350)

/*Debug */
#define SDFM_DEBUG_OFFSET         ( 0x360 )
#endif


/* Local defines */
/* Defines for comparator  */
#define SDFM_CFG_BF_SD_CH0_EN_COMP_BIT                  ( 0x01 )
#define SDFM_CFG_BF_SD_CH1_EN_COMP_BIT                  ( 0x02 )
#define SDFM_CFG_BF_SD_CH2_EN_COMP_BIT                  ( 0x03 )
#define SDFM_CFG_BF_SD_CH3_EN_COMP_BIT                  ( 0x04 )
#define SDFM_CFG_BF_SD_CH4_EN_COMP_BIT                  ( 0x05 )
#define SDFM_CFG_BF_SD_CH5_EN_COMP_BIT                  ( 0x06 )
#define SDFM_CFG_BF_SD_CH6_EN_COMP_BIT                  ( 0x07 )
#define SDFM_CFG_BF_SD_CH7_EN_COMP_BIT                  ( 0x08 )
#define SDFM_CFG_BF_SD_CH8_EN_COMP_BIT                  ( 0x09 )

/* Zero cross fields */
#define SDFM_CFG_BF_SD_CH0_ZC_EN_BIT         ( 0 )
#define SDFM_CFG_BF_SD_CH1_ZC_EN_BIT         ( 1 )
#define SDFM_CFG_BF_SD_CH2_ZC_EN_BIT         ( 2 )

/* Register sizes (in bytes) */
#define SDFM_ONE_BYTE                       ( 1 )
#define SDFM_TWO_BYTE                       ( 2 )
#define SDFM_THREE_BYTE                     ( 3 )
#define SDFM_FOUR_BYTE                      ( 4 )

/* Output sample offset */
#define SDFM_CFG_OUT_SAMP_BUF_OFFSET                   (0x00)

#endif
