/*
 *  Copyright (C) 2021-2025 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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


#ifndef _HDSL_DIAGNOSTIC_H_
#define _HDSL_DIAGNOSTIC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TXPRU_IRAM_SIZE                     (6*1024) /*6 kB*/

#define HDSL_EXTRA_EDGE_LOOKUP_SIZE         (8U)

/*Bit 7 will be set in QM when link is establised*/
#define QM_LINK_ESTABLISHED                 (0x80)
#define QM_LINK_ESTABLISHED_AND_VALUE_15    (0x8F)

/* for DDR trace*/
#define DDR_START_OFFSET                    (0x80000000)
#define DDR_LIMIT                           (0x3FFFFFFF)
#define DDR_END_OFFSET                      (0xBFFFFFF)

#define NUM_RESOURCES                       (5000)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE                 (UDMA_GET_TRPD_TR15_SIZE(1U))

/** \brief TSR input event: ICSSG_0_EDC1_SYNC0 ICSSG0 IEP1 sync event 0 Pulse (event 27) */
#define SYNCEVENT_INTRTR_IN_27              (27)

/** \brief TSR output event: SYNC0_OUT Pin - Selectable timesync event 24 Edge (4+(24*4) = 0x64) */
#define SYNCEVT_RTR_SYNC24_EVT              (0x64)

/** \brief TSR output event: SYNC1_OUT Pin - Selectable timesync event 25 Edge (4+(25*4) = 0x68) */
#define SYNCEVT_RTR_SYNC25_EVT              (0x68)

/** \brief TSR output event: SYNC2_OUT Pin - Selectable timesync event 26 Edge (4+(26*4) = 0x6C) */
#define SYNCEVT_RTR_SYNC26_EVT              (0x6C)

/** \brief TSR output event: SYNC3_OUT Pin - Selectable timesync event 27 Edge (4+(27*4) = 0x70) */
#define SYNCEVT_RTR_SYNC27_EVT              (0x70)

/** \brief TSR output event: ICSSG0_PR1_EDC1_LATCH0_IN PRU_ICSSG0 (4+(10*4) = 0x2C) */
#define SYNCEVT_RTR_SYNC10_EVT              (0x2C)

/** \brief TSR output event: ICSSG0_PR1_EDC1_LATCH1_IN PRU_ICSSG0 (4+(11*4) = 0x30) */
#define SYNCEVT_RTR_SYNC11_EVT              (0x30)

/** @} */

/* IEP (Industrial Ethernet Peripheral) Configuration Values */

/** \brief IEP global config: enable IEP + set default increment + set CMP increment to 1 */
#define IEP_GLOBAL_CFG_ENABLE_WITH_INCR     (0x111U)

/** \brief IEP SYNC control: enable SYNC and SYNC0 output */
#define IEP_SYNC_CTRL_ENABLE_SYNC0          (0x03U)

/** \brief IEP SYNC control: enable cyclic mode for periodic sync pulses */
#define IEP_SYNC_CTRL_CYCLIC_MODE           (0x20U)

/** \brief IEP CMP config: enable CMP1 for sync start time */
#define IEP_CMP_CFG_CMP1_ENABLE             (0x04U)

/* GPIO and Hardware Configuration Values */

/** \brief GPIO register address for PRG0_PRU1_GPI9 configuration (AM243x/AM64x) */
#define PRG0_PRU1_GPI9_CONFIG_REG           (0x000F41D4U)

/** \brief GPIO configuration value for input mode with pull */
#define GPIO_INPUT_MODE_CONFIG              (0x00050001U)

/** \brief Internal MUX selection value for routing to TimeSync XBAR */
#define ICSSM1_INPUT_INTR_SEL_ALL           (0xFFU)

/* Encoder ID Parsing Bit Masks and Shifts */

/** \brief Mask for acceleration bits (lower 4 bits) in encoder ID */
#define ENC_ID_ACC_BITS_MASK                (0x00FU)

/** \brief Mask for position bits (bits 4-9) in encoder ID */
#define ENC_ID_POS_BITS_MASK                (0x3F0U)

/** \brief Shift to extract position bits from encoder ID */
#define ENC_ID_POS_BITS_SHIFT               (4U)

/** \brief Flag bit indicating bipolar position encoding (bit 10) */
#define ENC_ID_BIPOLAR_FLAG                 (0x400U)

/** \brief Offset added to acceleration bits from encoder ID */
#define HDSL_ENC_ID_ACC_BITS_OFFSET         (8U)

/* Bit Masks for HDSL Status Registers */

/** \brief Mask for lower nibble (4 bits) of status registers */
#define HDSL_LOWER_NIBBLE_MASK              (0x0FU)

/** \brief Mask for upper nibble (4 bits) of status registers */
#define HDSL_UPPER_NIBBLE_MASK              (0xF0U)

/** \brief Shift to extract upper nibble to lower position */
#define HDSL_UPPER_NIBBLE_SHIFT             (4U)

#ifdef __cplusplus
}
#endif

#endif /* _HDSL_DIAGNOSTIC_H_ */
