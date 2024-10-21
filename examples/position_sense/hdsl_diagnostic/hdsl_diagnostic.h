/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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


#ifndef HDSL_APP_H_
#define HDSL_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <drivers/hw_include/hw_types.h>

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))
#define HWREGB(x)                                                              \
        (*((volatile uint8_t *)(x)))
#define HWREGH(x)                                                              \
        (*((volatile uint16_t *)(x)))

/* for DDR trace*/
#define DDR_START_OFFSET 0x80000000
#define DDR_LIMIT 0x3FFFFFFF
#define DDR_END_OFFSET 0xBFFFFFF

#define NUM_RESOURCES 5000 //4210752 (0x3fff_ffff div 0xff)
/* UDMA TR packet descriptor memory size - with one TR */
#define UDMA_TEST_TRPD_SIZE             (UDMA_GET_TRPD_TR15_SIZE(1U))
#ifndef SOC_AM261X
/*TSR configuration:*/

/*inEvent value:*/
/* ICSSG_0_EDC1_SYNC0 ICSSG0 IEP1 sync event 0 Pulse */
#define SYNCEVENT_INTRTR_IN_27 27

/*outEvent values:*/
/*SYNC0_OUT Pin Selectable timesync event 24 Edge (4+(24*4)) */
#define SYNCEVT_RTR_SYNC28_EVT 0x64
/* SYNC1_OUT Pin Selectable timesync event 25 Edge (4+(25*4)) */
#define SYNCEVT_RTR_SYNC29_EVT 0x68
/* SYNC2_OUT Pin Selectable timesync event 26 Edge (4+(26*4)) */
#define SYNCEVT_RTR_SYNC30_EVT 0x6C
/* SYNC3_OUT Pin Selectable timesync event 27 Edge (4+(27*4)) */
#define SYNCEVT_RTR_SYNC31_EVT 0x70
/* ICSSG0_PR1_EDC1_LATCH0_IN PRU_ICSSG0 (4+(10*4)) */
#define SYNCEVT_RTR_SYNC10_EVT 0x2C

#endif

#ifdef __cplusplus
}
#endif

#endif
