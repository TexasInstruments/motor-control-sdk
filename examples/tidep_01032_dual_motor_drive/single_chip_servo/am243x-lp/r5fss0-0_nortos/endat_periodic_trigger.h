/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef _ENDAT_H_
#define _ENDAT_H_

#include<stdint.h>

struct endat_periodic_interface
{
  void *pruss_iep;
  void *pruss_dmem;
  void *pruss_cfg;
  uint8_t load_share;
  uint64_t cmp3;
  uint64_t cmp5;
  uint64_t cmp6;
};

///#define IEP_DEFAULT_INC    0x1;
#define IEP_DEFAULT_INC_EN  0x4;
#define IEP_COUNTER_EN      0x1;
#define IEP_RST_CNT_EN      0x1;
#define IEP_CMP0_ENABLE     0x1 << 1;

#define IEP_CMP3_EVNT (0x1 << 3 )
#define IEP_CMP5_EVNT (0x1 << 5 )
#define IEP_CMP6_EVNT (0x1 << 6 )

#define PRU_TRIGGER_HOST_SDFM_EVT0   ( 2+16 )    /* pr0_pru_mst_intr[2]_intr_req */
#define PRU_TRIGGER_HOST_SDFM_EVT1   ( 3+16 )    /* pr0_pru_mst_intr[3]_intr_req */
#define PRU_TRIGGER_HOST_SDFM_EVT2   ( 4+16 )   /* pr0_pru_mst_intr[4]_intr_req */


uint32_t endat_config_periodic_mode(struct endat_periodic_interface *endat_periodic_interface, PRUICSS_Handle handle);

void endat_stop_periodic_continuous_mode(struct endat_periodic_interface *endat_periodic_interface);

static void pruEnDatIrqHandler0(void *handle);
static void pruEnDatIrqHandler1(void *handle);
static void pruEnDatIrqHandler2(void *handle);


#define IEP_TIM_CAP_CMP_EVENT	7
#define SYNC1_OUT_EVENT         13
#define SYNC0_OUT_EVENT         14

/* SYS_EVT_16-31 can be used for generating interrupts for IPC with hosts/prus etc */
#define PRU_ARM_EVENT00         16
#define PRU_ARM_EVENT01         17
#define PRU_ARM_EVENT02         18
#define PRU_ARM_EVENT03         19
#define PRU_ARM_EVENT04         20
#define PRU_ARM_EVENT05         21
#define PRU_ARM_EVENT06         22
#define PRU_ARM_EVENT07         23
#define PRU_ARM_EVENT08         24
#define PRU_ARM_EVENT09         25
#define PRU_ARM_EVENT10         26
#define PRU_ARM_EVENT11         27
#define PRU_ARM_EVENT12         28
#define PRU_ARM_EVENT13         29
#define PRU_ARM_EVENT14         30
#define PRU_ARM_EVENT15         31

#define PRU0_RX_ERR32_EVENT     33
#define PORT1_TX_UNDERFLOW	    39
#define PORT1_TX_OVERFLOW	    40
#define MII_LINK0_EVENT         41
#define PORT1_RX_EOF_EVENT      42
#define PRU1_RX_ERR32_EVENT     45
#define PORT2_TX_UNDERFLOW	    51
#define PORT2_TX_OVERFLOW	    53
#define PORT2_RX_EOF_EVENT	    54
#define MII_LINK1_EVENT         53

#define CHANNEL0                0
#define CHANNEL1                1
#define CHANNEL2                2
#define CHANNEL3                3
#define CHANNEL4                4
#define CHANNEL5                5
#define CHANNEL6                6
#define CHANNEL7                7
#define CHANNEL8                8
#define CHANNEL9                9

#define PRU0                    0
#define PRU1                    1
#define PRU_EVTOUT0             2
#define PRU_EVTOUT1             3
#define PRU_EVTOUT2             4
#define PRU_EVTOUT3             5
#define PRU_EVTOUT4             6
#define PRU_EVTOUT5             7
#define PRU_EVTOUT6             8
#define PRU_EVTOUT7             9

#define PRU0_HOSTEN_MASK            ((uint32_t)0x0001)
#define PRU1_HOSTEN_MASK            ((uint32_t)0x0002)
#define PRU_EVTOUT0_HOSTEN_MASK     ((uint32_t)0x0004)
#define PRU_EVTOUT1_HOSTEN_MASK     ((uint32_t)0x0008)
#define PRU_EVTOUT2_HOSTEN_MASK     ((uint32_t)0x0010)
#define PRU_EVTOUT3_HOSTEN_MASK     ((uint32_t)0x0020)
#define PRU_EVTOUT4_HOSTEN_MASK     ((uint32_t)0x0040)
#define PRU_EVTOUT5_HOSTEN_MASK     ((uint32_t)0x0080)
#define PRU_EVTOUT6_HOSTEN_MASK     ((uint32_t)0x0100)
#define PRU_EVTOUT7_HOSTEN_MASK     ((uint32_t)0x0200)

#define SYS_EVT_POLARITY_LOW        0
#define SYS_EVT_POLARITY_HIGH       1

#define SYS_EVT_TYPE_PULSE          0
#define SYS_EVT_TYPE_EDGE           1

#define PRUICSS_INTC_INITDATA {   \
{   IEP_TIM_CAP_CMP_EVENT, PRU_ARM_EVENT02, PRU_ARM_EVENT03, PRU_ARM_EVENT04, PRU_ARM_EVENT05, PRU_ARM_EVENT06, \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* initializing member [6-15] for Misra C standards */ \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,     /* initializing member [16-31] for Misra C standards */ \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,     /* initializing member [32-47] for Misra C standards */ \
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},    /* initializing member [48-63] for Misra C standards */ \
{ {IEP_TIM_CAP_CMP_EVENT,   CHANNEL1, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {PRU_ARM_EVENT02,         CHANNEL2, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {PRU_ARM_EVENT03,         CHANNEL3, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {PRU_ARM_EVENT04,         CHANNEL4, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {PRU_ARM_EVENT05,         CHANNEL5, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {PRU_ARM_EVENT06,         CHANNEL6, SYS_EVT_POLARITY_HIGH, SYS_EVT_TYPE_PULSE},\
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [6] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [7] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [8] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [9] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [10] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [11] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [12] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [13] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [14] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [15] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [16] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [17] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [18] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [19] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [20] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [21] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [22] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [23] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [24] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [25] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [26] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [27] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [28] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [29] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [30] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [31] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [32] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [33] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [34] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [35] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [36] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [37] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [38] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [39] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [40] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [41] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [42] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [43] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [44] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [45] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [46] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [47] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [48] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [49] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [50] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [51] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [52] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [53] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [54] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [55] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [56] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [57] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [58] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [59] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [60] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [61] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}, /* initializing member [62] for Misra C standards */ \
  {0xFF,0xFF,0xFF,0xFF}}, /* initializing member [63] for Misra C standards */ \
  { {CHANNEL1, PRU1}, {CHANNEL2, PRU_EVTOUT0}, {CHANNEL3, PRU_EVTOUT1},\
    {CHANNEL4, PRU_EVTOUT2}, {CHANNEL5, PRU_EVTOUT3}, {CHANNEL6, PRU_EVTOUT4}, \
    {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF}, {0xFF, 0xFF} }, /* Initializing members [6,7,8,9] of array for Misra C standards */ \
  (PRU1_HOSTEN_MASK | PRU_EVTOUT0_HOSTEN_MASK | PRU_EVTOUT1_HOSTEN_MASK | PRU_EVTOUT2_HOSTEN_MASK | PRU_EVTOUT3_HOSTEN_MASK | PRU_EVTOUT4_HOSTEN_MASK) /* PRU_EVTOUT0 */ \
}


#endif /* _ENDAT_H_ */
