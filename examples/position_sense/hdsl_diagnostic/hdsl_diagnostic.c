/*
 *  Copyright (C) 2021-2026 Texas Instruments Incorporated
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

/**
 *  \file   hdsl_diagnostic.c
 *
 *  \brief  HDSL diagnostic application demonstrating position encoder communication
 *
 *  \section hdsl_load_share HDSL Load Share Mode Configuration
 *
 *  The HDSL driver supports two operating modes based on PRU-ICSS core clock frequency:
 *
 *  **Non-Load Share Mode (225 MHz):**
 *  - Supported on: AM243x (PRU-ICSSG) and AM261x (PRU-ICSSM)
 *  - Single channel operation only
 *  - All encoder channels use the PRU core (PRU0 or PRU1 based on selected slice)
 *
 *  **Load Share Mode (300 MHz):**
 *  - Supported on: AM243x (PRU-ICSSG) only - NOT available on AM261x
 *  - Multi-channel operation (up to 3 HDSL encoder channels)
 *  - Channels distributed across different PRU cores for load balancing:
 *    * Channel 0 uses RTU_PRU
 *    * Channel 1 uses PRU
 *    * Channel 2 uses TX_PRU
 *
 *  **Driver Validation includes:**
 *
 *  - Handle Parameter Validation.
 *  - Array Bounds and Index Validation.
 *  - Internal Structure Validation.
 *  - Pointer Parameter Validation.
 *
 *  **Application Responsibilities:**
 *  1. Always check return value of HDSL_open() before proceeding
 *  2. Do not modify driver internal structures or SysConfig-generated data
 *  3. Ensure handles remain valid for the lifetime of usage
 *  4. Call HDSL_close() to properly cleanup before handle reuse
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/hw_types.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/ClockP.h>

#include <drivers/pruicss.h>
#ifndef SOC_AM261X
#include <drivers/udma.h>
#endif
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "hdsl_diagnostic.h"
#include <position_sense/hdsl/include/hdsl_drv.h>

#ifdef HDSL_AM64xE1_TRANSCEIVER
#include <board/ioexp/ioexp_tca6424.h>
#endif

/* PRU core clock frequency = 225 MHz */
#define PRU_CORE_CLOCK_FREQ_225M 225000000
/* PRU core clock frequency = 300 MHz */
#define PRU_CORE_CLOCK_FREQ_300M 300000000

#define PRU_CORE_CLOCK_FREQ CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
/* PRU core clock frequency = 225 MHz */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#include <position_sense/hdsl/firmware/freerun_225_mhz/hdsl_receiver_freerun_225_mhz_pru1_bin.h>
#include <position_sense/hdsl/firmware/sync_225_mhz/hdsl_receiver_sync_225_mhz_pru1_bin.h>
#else
#include <position_sense/hdsl/firmware/freerun_225_mhz/hdsl_receiver_freerun_225_mhz_pru0_bin.h>
#include <position_sense/hdsl/firmware/sync_225_mhz/hdsl_receiver_sync_225_mhz_pru0_bin.h>
#endif
#else
/*  PRU core clock frequency = 300 MHz */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#include <position_sense/hdsl/firmware/multichannel_ch0/hdsl_receiver_multichannel_rtu_pru1_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch0_sync_mode/hdsl_receiver_multichannel_sync_mode_rtu_pru1_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch1/hdsl_receiver_multichannel_pru1_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch1_sync_mode/hdsl_receiver_multichannel_sync_mode_pru1_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch2/hdsl_receiver_multichannel_tx_pru1_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch2_sync_mode/hdsl_receiver_multichannel_sync_mode_tx_pru1_bin.h>
#else
#include <position_sense/hdsl/firmware/multichannel_ch0/hdsl_receiver_multichannel_rtu_pru0_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch0_sync_mode/hdsl_receiver_multichannel_sync_mode_rtu_pru0_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch1/hdsl_receiver_multichannel_pru0_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch1_sync_mode/hdsl_receiver_multichannel_sync_mode_pru0_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch2/hdsl_receiver_multichannel_tx_pru0_bin.h>
#include <position_sense/hdsl/firmware/multichannel_ch2_sync_mode/hdsl_receiver_multichannel_sync_mode_tx_pru0_bin.h>
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if (CONFIG_HDSL0_CHANNEL0_ENABLED + CONFIG_HDSL0_CHANNEL1_ENABLED  + CONFIG_HDSL0_CHANNEL2_ENABLED > 1)
#define HDSL_MULTI_CHANNEL
#endif

/* Timing and delay constants */
#define HDSL_SYNC_START_TIME                    (10000U)
#define HDSL_INIT_DELAY_US                      (5000U)
#define SYNC_PULSE_WAIT_CLK_CYCLES              (5505)
/** Maximum wait time in iterations for HDSL operations (20000 iterations) */
#define MAX_WAIT                                (20000)
/*Timeout in micro-seconds for short message read/write*/
#define SHORT_MSG_TIMEOUT                       (1000)
/*Timeout in micro-seconds for long message read/write*/
#define LONG_MSG_TIMEOUT                        (200000)
/* Sleep in micro-seconds to be used when polling QM during intialization */
#define HDSL_QM_POLL_SLEEP_SEC                  (1)

/* max cycle time for transmission of dsl frame*/
#define MAX_SYNC_CYCLE_TIME                     (27)
/* min cycle time for transmission of dsl frame*/
#define MIN_SYNC_CYCLE_TIME                     (12)

/*Register Addresses for Short Messages (Parameter Channel)*/
#define ENCODER_STATUS0_REG_ADDRESS             (0x40)
#define ENCODER_RSSI_REG_ADDRESS                (0x7C)
#define ENCODER_PING_REG_ADDRESS                (0x7F)

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
/* Memory Trace is triggered for each H-Frame. SYS_EVENT_21 is triggered for each
   H-Frame from PRU. SYS_EVENT_21 is mapped to PRU_ICSSG0_PR1_HOST_INTR_PEND_3 of R5F
   in INTC Mapping. */

/*Event number for SYS_EVENT_21 (pr1_pru_mst_intr<5>_int_req) */
#define HDSL_MEMORY_TRACE_ICSS_INTC_EVENT_NUM   (21U)

/* R5F Interrupt number for Memory Traces */
#define HDSL_MEMORY_TRACE_R5F_IRQ_NUM           (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3)
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static int32_t hdsl_sync_calculation(HDSL_Handle handle);
static void hdsl_iep_init(void);
static int32_t hdsl_enable_sync_signal(uint8_t es, uint32_t period);

static void hdsl_process_request(HDSL_Handle handle, uint32_t menu);

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
static void hdsl_pruicss_init(void);
static void hdsl_init(void);
static void hdsl_pruicss_load_run_fw(void);
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_300M)
static void hdsl_pruicss_init_300m(void);
static void hdsl_pruicss_load_run_fw_300m(void);
static void hdsl_init_300m(void);
#endif

static void hdsl_read_pc_short_msg(HDSL_Handle handle);
static void hdsl_write_pc_short_msg(HDSL_Handle handle);

static void hdsl_display_menu(void);

static void hdsl_direct_read_rid0_length4(HDSL_Handle handle);
static void hdsl_direct_read_rid81_length8(HDSL_Handle handle);
static void hdsl_direct_read_rid81_length2(HDSL_Handle handle);
static void hdsl_indirect_write_rid0_length8_offset0(HDSL_Handle handle);
static void hdsl_indirect_write_rid0_length8(HDSL_Handle handle);

static uint32_t hdsl_get_menu(void);

static uint32_t hdsl_read_encoder_resolution(HDSL_Handle handle);

#ifdef HDSL_AM64xE1_TRANSCEIVER
static void hdsl_i2c_io_expander(void *args);
#endif

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)

static void hdsl_udma_trpd_init(Udma_ChHandle gChHandle,
                                uint8_t *trpdMem,
                                const void *destBuf,
                                const void *srcBuf,
                                uint32_t length);

static void hdsl_udma_copy(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);

static void hdsl_isr_fxn(void);

static void hdsl_traces_into_memory(HDSL_Handle handle);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * Handle 2D array structure: [instance][channel]
 * Each channel has its own runtime state (priv)
 * In load share mode: enabled channels (0, 1, 2) have valid priv entries at their respective indices
 * In non-load share mode: always use index 0 (gAppHdslHandle[instance][0]) regardless of which
 *                         physical channel number (0, 1, or 2) is configured. Only index [0] contains
 *                         valid handle; indices [1] and [2] are unused.
 */

/* Runtime handle array for HDSL instances */
HDSL_Handle gAppHdslHandle[CONFIG_HDSL_NUM_INSTANCES][HDSL_NUM_CH_PER_SLICE_MAX];

PRUICSS_Handle gPruIcssXHandle;

/* PRU-ICSS Interrupt Controller (INTC) configuration data - generated by SysConfig based on PRU-ICSS instance */
#if !defined(SOC_AM261X)
#if (CONFIG_HDSL0_PRUICSS_INSTANCE == 1)
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif
#endif

uint8_t gFirstEnabledChannel;

HDSL_CopyTable *gCopyTable;

#ifdef HDSL_AM64xE1_TRANSCEIVER
static TCA6424_Config  gTCA6424_Config;
#endif

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)

Udma_ChHandle   gChHandle;

/* To store user input to start memory copy*/
volatile uint8_t gStartCopy;

/* To store user input number of count of memory copy*/
uint16_t gTraceCount;

/* To store error count during memory copy*/
uint32_t gTraceErrorCount;

/* To save log h-frame count during memory copy*/
uint32_t gHFrameCountArr[NUM_RESOURCES];

/* Location for copying HDSL Interface structure */
HDSL_Interface gHdslInterfaceTrace[NUM_RESOURCES] __attribute__((aligned(128), section(".hdslInterface_mem")));

HwiP_Object gPRUHwiObject;

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
#endif


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)

static void hdsl_udma_trpd_init(Udma_ChHandle gChHandle,
                                uint8_t *trpd_mem,
                                const void *dest_buf,
                                const void *src_buf,
                                uint32_t length)
{
    CSL_UdmapTR15  *p_tr;
    uint32_t        cq_ring_num = Udma_chGetCqRingNum(gChHandle);
    static          uint32_t init_done = 0;

    if(init_done == 0)
    {
        /* Make TRPD with TR15 TR type */
        UdmaUtils_makeTrpdTr15(trpd_mem, 1U, cq_ring_num);

        /* Setup TR */
        p_tr = UdmaUtils_getTrpdTr15Pointer(trpd_mem, 0U);
        p_tr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
        p_tr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
        p_tr->icnt0    = length;
        p_tr->icnt1    = 1U;
        p_tr->icnt2    = 1U;
        p_tr->icnt3    = 1U;
        p_tr->dim1     = p_tr->icnt0;
        p_tr->dim2     = (p_tr->icnt0 * p_tr->icnt1);
        p_tr->dim3     = (p_tr->icnt0 * p_tr->icnt1 * p_tr->icnt2);
        p_tr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(src_buf, 0U, NULL);
        p_tr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
        p_tr->dicnt0   = length;
        p_tr->dicnt1   = 1U;
        p_tr->dicnt2   = 1U;
        p_tr->dicnt3   = 1U;
        p_tr->ddim1    = p_tr->dicnt0;
        p_tr->ddim2    = (p_tr->dicnt0 * p_tr->dicnt1);
        p_tr->ddim3    = (p_tr->dicnt0 * p_tr->dicnt1 * p_tr->dicnt2);
        p_tr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(dest_buf, 0U, NULL);
        /* Perform cache writeback */
        CacheP_wb(trpd_mem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

        init_done = 1;
    }
    else
    {
        p_tr = UdmaUtils_getTrpdTr15Pointer(trpd_mem, 0U);
        p_tr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(dest_buf, 0U, NULL);
        /* Perform cache writeback */
        CacheP_wb(trpd_mem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
    }
    return;
}

static void hdsl_udma_copy(uint8_t *src_buf, uint8_t *dest_buf, uint32_t length)
{
    int32_t         ret_val = UDMA_SOK;
    uint64_t        p_desc;
    uint32_t        tr_resp_status;
    uint8_t        *trpd_mem = &gUdmaTestTrpdMem[0U];
    uint64_t        trpd_mem_phy = (uint64_t) Udma_defaultVirtToPhyFxn(trpd_mem, 0U, NULL);

    if((src_buf == NULL) || (dest_buf == NULL))
    {
        gTraceErrorCount++;
        return;
    }

    /* Init TR packet descriptor */
    hdsl_udma_trpd_init(gChHandle, trpd_mem, dest_buf, src_buf, length);

    /* Submit TRPD to channel */
    ret_val = Udma_ringQueueRaw(Udma_chGetFqRingHandle(gChHandle), trpd_mem_phy);
    DebugP_assert(UDMA_SOK == ret_val);

    /* Wait for return descriptor in completion ring - this marks transfer completion */
    while(1)
    {
        ret_val = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(gChHandle), &p_desc);
        if(UDMA_SOK == ret_val)
        {
            /* Check TR response status */
            CacheP_inv(trpd_mem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
            tr_resp_status = UdmaUtils_getTrpdTr15Response(trpd_mem, 1U, 0U);
            DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == tr_resp_status);
            break;
        }
    }

    /* Validate data in destination memory */
    CacheP_inv(dest_buf, length, CacheP_TYPE_ALLD);
}

static void hdsl_isr_fxn(void)
{
    static uint64_t h_frames_count = 0;
    static uint32_t temp = 0;
    uint8_t         *src_buf;
    uint8_t         *dest_buf;
    uint32_t        length;
    void            *src_loc;
    int32_t         status;

    if(gAppHdslHandle[CONFIG_HDSL0][0] == NULL)
    {
        gTraceErrorCount++;
        return;
    }

    status = HDSL_get_src_loc(gAppHdslHandle[CONFIG_HDSL0][0], &src_loc);
    if(status != SystemP_SUCCESS)
    {
        gTraceErrorCount++;
        return;
    }
    else
    {
        src_buf = (uint8_t *)src_loc;
    }

    status = HDSL_get_length(gAppHdslHandle[CONFIG_HDSL0][0], &length);
    if(status != SystemP_SUCCESS)
    {
        gTraceErrorCount++;
        return;
    }

    PRUICSS_clearEvent(gPruIcssXHandle, HDSL_MEMORY_TRACE_ICSS_INTC_EVENT_NUM);

    /* No of h-frames count */
    h_frames_count++;

    if((gStartCopy == 1) && (temp < gTraceCount))
    {

        /* Init buffers and TR packet descriptor */
        dest_buf = (uint8_t *)&gHdslInterfaceTrace[temp];

        /* start UDMA copying data from src to dest */
        hdsl_udma_copy(src_buf, dest_buf, length);

        gHFrameCountArr[temp] = h_frames_count;
        temp++;
    }
    else
    {
        gStartCopy = 0;
        temp = 0;
    }
}

static void hdsl_traces_into_memory(HDSL_Handle handle)
{
    uint32_t i = 0;
    int32_t status;
    uint32_t length;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_traces_into_memory() failed due to NULL handle");
        return;
    }

    status = HDSL_get_length(handle, &length);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_get_length() did not return success\r\n");
        return;
    }

    DebugP_log("\r\n sizeof(hdslInterface)_count = %u", length);
    DebugP_log("\r\n Start address of memory trace location = %x", &gHdslInterfaceTrace[0]);
    DebugP_log("\r\n End address of memory trace location = %x", &gHdslInterfaceTrace[NUM_RESOURCES-1]);
    DebugP_log("\r\n No of HDSL-Interface-Register-Structure to copy = %u", gTraceCount);

    gStartCopy = 1;

    while(gStartCopy)
    {
        ClockP_sleep(1);
    }

    for(i=0; i< gTraceCount; i++)
    {
        DebugP_log("\r\n %u h_frame_count = %u ", i, gHFrameCountArr[i]);
    }
}

#endif

static void hdsl_iep_init(void)
{
    PRUICSS_setIepCounterIncrementValue(gPruIcssXHandle, 0, 1);
    PRUICSS_controlIepCounter(gPruIcssXHandle, 0, 1);
    PRUICSS_setIepClkSrc(gPruIcssXHandle, 1);
}

static int32_t hdsl_enable_sync_signal(uint8_t es, uint32_t period)
{
    uint32_t read_value;
    uint32_t start_time = HDSL_SYNC_START_TIME;
#ifdef SOC_AM261X
    /* For AM261X, ICSSM0 IEP0 is used to generate SYNC_OUT0 pulse as an input to ICSSM1 LATCH0*/
    uint32_t iep_base = CSL_ICSS_M_ICSSM_0_IEP0_U_BASE;
#else
    uint32_t iep_base = (uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep1RegBase);
#endif

    /* Enable IEP. Enable the Counter and set the DEFAULT_INC and CMP_INC to 1. */
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, IEP_GLOBAL_CFG_ENABLE_WITH_INCR);

    /* Enable SYNC0 and program pulse width */
    /* Enable SYNC and SYNC0 */
    read_value = HW_RD_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    read_value |= IEP_SYNC_CTRL_ENABLE_SYNC0;
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, read_value);

    /* Enable cyclic mode */
    read_value = HW_RD_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG);
    read_value |= IEP_SYNC_CTRL_CYCLIC_MODE;
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC_CTRL_REG, read_value);

    /* Configure pulse width and period */
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC_PWIDTH_REG, (period)/2);
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_SYNC0_PERIOD_REG, (period));

    /* Program CMP1 */
    read_value = HW_RD_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    read_value |= IEP_CMP_CFG_CMP1_ENABLE;
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, read_value);

    /* NOTE: Ensure this start time is in future */
    HW_WR_REG32(iep_base + CSL_ICSS_PR1_IEP0_SLV_CMP1_REG0, start_time);

    /* On AM243x, Time Sync Router is used.
     * On AM261x, Time Sync XBAR  is used. */
#ifdef SOC_AM261X
    /*Changing internal MUXING for enabling ICSSM1 XBAR instance settings */
    HW_WR_REG32(((uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->baseAddr) + CSL_MSS_CTRL_ICSSM1_INPUT_INTR_SEL), ICSSM1_INPUT_INTR_SEL_ALL);
#else
    /* Time Sync Router(TSR) Configuration */
    uint32_t in_event;
    uint32_t out_event_latch;
    uint32_t out_event_gpio;
    in_event = SYNCEVENT_INTRTR_IN_27;
    out_event_latch = SYNCEVT_RTR_SYNC10_EVT;
    out_event_gpio = SYNCEVT_RTR_SYNC30_EVT;

    HW_WR_REG32(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + out_event_latch, (in_event | 0x10000));
    HW_WR_REG32(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + out_event_gpio, (in_event | 0x10000));
    HW_WR_REG32(CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + SYNCEVT_RTR_SYNC28_EVT, (in_event | 0x10000));
#endif
    return SystemP_SUCCESS;
}

static int32_t hdsl_sync_calculation(HDSL_Handle handle)
{
    int32_t status;
    uint8_t es;
    uint16_t wait_before_start;
    uint32_t counter, period, index;
    volatile uint32_t cap6_rise0, cap6_rise1, cap6_fall0, cap6_fall1;
    HDSL_Priv *priv;
    priv = HDSL_get_priv(handle);

    if((handle == NULL) || (priv == NULL))
    {
        DebugP_log("\r\n\n|ERROR: hdsl_sync_calculation() failed due to NULL handle/priv");
        return SystemP_FAILURE;
    }

    /* Extra edge lookup table for SYNC timing calculations */
    static const uint8_t extra_edge_arr[HDSL_EXTRA_EDGE_LOOKUP_SIZE] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};

    uint32_t minm_bits, cycle_per_bit, max_stuffing, stuffing_size, cycle_per_overclock_bit, minm_extra_size, sync_param_mem_start;
    uint32_t cycles_left, additional_bits, minm_cycles, time_rest, extra_edge, extra_size, num_of_stuffing, extra_size_remainder, stuffing_remainder, bottom_up_cycles;

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
        minm_bits = 112; cycle_per_bit = 24; max_stuffing = 26; stuffing_size = 6; cycle_per_overclock_bit = 3; minm_extra_size = 4; sync_param_mem_start = 0xDC;
#endif
#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_300M)
        minm_bits = 112; cycle_per_bit = 32; max_stuffing = 26; stuffing_size = 6; cycle_per_overclock_bit = 4; minm_extra_size = 4; sync_param_mem_start = 0xDC;
#endif

    /* Measurement of SYNC period starts */
    status = HDSL_get_sync_ctrl(handle, &es);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_get_sync_ctrl() did not return success\r\n");
        return SystemP_FAILURE;
    }

    /* Validate that ES value is non-zero before using in calculations */
    if(es == 0)
    {
        DebugP_log("\r\n ERROR: ES=0, cannot calculate sync parameters\r\n");
        return SystemP_FAILURE;
    }
#ifdef SOC_AM261X
    /* For AM261X, ICSSM0 IEP0 is used to generate SYNC_OUT0 pulse as an input to ICSSM1 LATCH0*/
    uint32_t capr6_reg0 = (uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase) + CSL_ICSS_M_PR1_IEP0_SLV_CAPR6_REG0;
    uint32_t capf6_reg0 = (uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase) + CSL_ICSS_M_PR1_IEP0_SLV_CAPF6_REG0;
#else
    uint32_t capr6_reg0 = (uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep1RegBase) + CSL_ICSS_G_PR1_IEP1_SLV_CAPR6_REG0;
    uint32_t capf6_reg0 = (uint32_t)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep1RegBase) + CSL_ICSS_G_PR1_IEP1_SLV_CAPF6_REG0;
#endif
    cap6_rise0 = HW_RD_REG32(capr6_reg0);
    cap6_fall0 = HW_RD_REG32(capf6_reg0);
    cap6_rise1 = cap6_rise0;
    cap6_fall1 = cap6_fall0;
    counter = 0;
    for(index = 0; index < 2; index++)
    {
        cap6_rise0 = cap6_rise1;
        cap6_fall0 = cap6_fall1;
        while(((cap6_fall0 == cap6_fall1) || (cap6_rise0 == cap6_rise1)) && (counter <= MAX_WAIT))
        {
            cap6_rise1 = HW_RD_REG32(capr6_reg0);
            cap6_fall1 = HW_RD_REG32(capf6_reg0);
            counter++;
        }
    }

    /* Validate SYNC period measurement succeeded */
    if(counter > MAX_WAIT)
    {
        DebugP_log("\r\n ERROR: SYNC period measurement timeout (counter=%u, max=%u)\r\n", counter, MAX_WAIT);
        return SystemP_FAILURE;
    }
    period = cap6_rise1 - cap6_rise0;
    /*measure of SYNC period ends*/
    minm_cycles = minm_bits * es * cycle_per_bit;
    cycles_left = period - minm_cycles;
    time_rest = (cycles_left % cycle_per_bit) / cycle_per_overclock_bit;
    additional_bits = cycles_left / cycle_per_bit;

    /* Bounds check for array access */
    if(time_rest >= HDSL_EXTRA_EDGE_LOOKUP_SIZE)
    {
        DebugP_log("\r\n ERROR: time_rest value out of range: %u\r\n", time_rest);
        return SystemP_FAILURE;
    }
    extra_edge = extra_edge_arr[time_rest];
    num_of_stuffing = additional_bits / stuffing_size;
    extra_size = additional_bits % stuffing_size;
    extra_size = extra_size + minm_extra_size * es;
    if(num_of_stuffing > es * max_stuffing)
    {
        extra_size = extra_size + (((num_of_stuffing) - (max_stuffing * es)) * stuffing_size);
        num_of_stuffing = es * max_stuffing;
    }
    extra_size_remainder = extra_size % es;
    extra_size = extra_size / es;
    stuffing_remainder = num_of_stuffing % es;
    num_of_stuffing = num_of_stuffing / es;
    bottom_up_cycles = (minm_cycles - minm_extra_size * es * cycle_per_bit);
    bottom_up_cycles = bottom_up_cycles + (stuffing_size * (es * num_of_stuffing + stuffing_remainder))*cycle_per_bit;
    bottom_up_cycles = bottom_up_cycles + ((es * extra_size  + extra_size_remainder) * cycle_per_bit ) + time_rest * cycle_per_overclock_bit;
    wait_before_start = (84 * cycle_per_bit) + ((8 - time_rest)*cycle_per_overclock_bit) + (num_of_stuffing * stuffing_size * cycle_per_bit);
    if(stuffing_remainder != 0)
    {
        wait_before_start = wait_before_start+(stuffing_size * cycle_per_bit);
    }
    wait_before_start=wait_before_start+SYNC_PULSE_WAIT_CLK_CYCLES;
    if(extra_size < 4 || extra_size > 9)
    {
        DebugP_log("\r\n ERROR: ES or period selected is invalid ");
    }
    DebugP_log("\r\n ********************************************************************");
    DebugP_log("\r\n SYNC MODE: period = %d", period);
    DebugP_log("\r\n SYNC MODE: ES = %d", es);
    DebugP_log("\r\n SYNC MODE: counter = %d", counter);
    DebugP_log("\r\n SYNC MODE: wait_before_start = %d", wait_before_start);
    DebugP_log("\r\n SYNC MODE: bottom_up_cycles = %d", bottom_up_cycles);
    DebugP_log("\r\n SYNC MODE: extra_size = %d", extra_size);
    DebugP_log("\r\n SYNC MODE: time_rest = %d", time_rest);
    DebugP_log("\r\n SYNC MODE: extra_edge = %d", extra_edge);
    DebugP_log("\r\n SYNC MODE: num_of_stuffing = %d", num_of_stuffing);
    DebugP_log("\r\n SYNC MODE: extra_size_remainder = %d", extra_size_remainder);
    DebugP_log("\r\n SYNC MODE: stuffing_remainder = %d", stuffing_remainder);
    DebugP_log("\r\n ********************************************************************");

    sync_param_mem_start = sync_param_mem_start + (uint32_t)priv->base_mem_addr;

    HW_WR_REG8(sync_param_mem_start, extra_size);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG8(sync_param_mem_start, num_of_stuffing);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG8(sync_param_mem_start, extra_edge);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG8(sync_param_mem_start, time_rest);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG8(sync_param_mem_start, extra_size_remainder);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG8(sync_param_mem_start, stuffing_remainder);
    sync_param_mem_start = sync_param_mem_start + 1;
    HW_WR_REG16(sync_param_mem_start, wait_before_start);

    return SystemP_SUCCESS;
}

static void hdsl_process_request(HDSL_Handle handle, uint32_t menu)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t val[3];
    uint8_t ureg, ureg1;
    float pos[3];
    uint64_t turn[3];
    uint32_t i;
    uint64_t mask_value;
    uint32_t res_value;
    uint32_t multi_turn_value;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_process_request() failed due to NULL handle");
        return;
    }

    switch(menu)
    {
        case MENU_SAFE_POSITION:

            status = HDSL_get_pos(handle, 0, &val[0]);
            if(status == SystemP_SUCCESS)
            {

                DebugP_log("\r\n Fast Position read successfully");
            }

            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_pos(handle, 1, &val[1]);
            }
            if(status == SystemP_SUCCESS)
            {
                DebugP_log("\r\n Safe Position 1 read successfully");
            }

            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_pos(handle, 2, &val[2]);
            }
            if(status == SystemP_SUCCESS)
            {
                DebugP_log("\r\n Safe Position 2 read successfully");
            }

            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_mask(handle, &mask_value);
            }
            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_res(handle, &res_value);
            }
            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_multi_turn(handle, &multi_turn_value);
            }

            if(status == SystemP_SUCCESS)
            {
                for(i = 0; i < 3; i++)
                {
                    pos[i] = (float)(val[i] & mask_value) / (float)(mask_value + 1) * (float)360;
                }
            }

            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_rssi(handle, &ureg);
            }
            if(status == SystemP_SUCCESS)
            {
                status = HDSL_get_qm(handle, &ureg1);
            }

            if(status == SystemP_SUCCESS)
            {
                if(multi_turn_value)
                {
                    for(i = 0; i < 3; i++)
                    {
                        turn[i] = val[i] & ~mask_value;
                        turn[i] >>= res_value;
                    }
                    DebugP_log("\r\n Angle: %10.6f\tTurn: %llu\t", pos[0], turn[0]);
                    DebugP_log("\r\n SafePos1: %10.6f\tTurn: %llu", pos[1], turn[1]);
                    DebugP_log("\r\n SafePos2: %10.6f\tTurn: %llu", pos[2], turn[2]);
                    DebugP_log("\r\n RSSI: %u\t QM:  %u", ureg, ureg1 );

                }
                else
                {
                    DebugP_log("\r\n Angle: %10.6f", pos[0]);
                }
            }
            break;
        case MENU_QUALITY_MONITORING:
            {
                uint8_t qm_value;
                status = HDSL_get_qm(handle, &qm_value);
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n Quality monitoring value: %u", qm_value);
                }
            }
            break;
        case MENU_EVENTS:
            {
                uint16_t events_value;
                uint8_t safe_events_value;
                uint16_t status_value;

                status = HDSL_get_events(handle, &events_value);
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n EVENT_H and EVENT_L: 0x%x", events_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    status = HDSL_get_safe_events(handle, &safe_events_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n EVENT_S: 0x%x", safe_events_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    status = HDSL_get_online_status_d(handle, &status_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ONLINE_STATUS_D: 0x%x", status_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    status = HDSL_get_online_status_1(handle, &status_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ONLINE_STATUS_1: 0x%x", status_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    status = HDSL_get_online_status_2(handle, &status_value);
                }
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ONLINE_STATUS_2: 0x%x", status_value);
                }
            }
            break;
        case MENU_SUMMARY:
            {
                uint8_t sum_value;
                status = HDSL_get_sum(handle, &sum_value);
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n Summarized slave status: 0x%x", sum_value);
                }
            }
            break;
        case MENU_ACC_ERR_CNT:
            {
                uint8_t count_value;
                status = HDSL_get_acc_err_cnt(handle, &count_value);
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n Acceleration error counter: %u", count_value);
                }
            }
            break;
        case MENU_RSSI:
            {
                uint8_t rssi_value;
                status = HDSL_get_rssi(handle, &rssi_value);
                if(status == SystemP_SUCCESS)
                {
                    DebugP_log("\r\n RSSI: %u", rssi_value);
                }
            }
            break;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
        case MENU_HDSL_REG_INTO_MEMORY:
            hdsl_traces_into_memory(handle);
            break;
#endif
        case MENU_PC_SHORT_MSG_WRITE:
            hdsl_write_pc_short_msg(handle);
            break;
        case MENU_PC_SHORT_MSG_READ:
            hdsl_read_pc_short_msg(handle);
            break;
        case MENU_DIRECT_READ_RID0_LENGTH4:
            hdsl_direct_read_rid0_length4(handle);
            break;
        case MENU_DIRECT_READ_RID81_LENGTH8:
            hdsl_direct_read_rid81_length8(handle);
            break;
        case MENU_DIRECT_READ_RID81_LENGTH2:
            hdsl_direct_read_rid81_length2(handle);
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8:
            hdsl_indirect_write_rid0_length8(handle);
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0:
            hdsl_indirect_write_rid0_length8_offset0(handle);
            break;

        default:
            DebugP_log( "\r\n ERROR: invalid request");
            break;
    }

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: API call failed in menu option %d", menu);
    }
}

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
static void hdsl_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_HDSL0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    /* Configure PRU-ICSS Interrupt Controller */
#if !defined(SOC_AM261X)
#if (CONFIG_HDSL0_PRUICSS_INSTANCE == 1)
    PRUICSS_intcInit(gPruIcssXHandle, &icss1_intc_initdata);
#else
    PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
#endif
#endif

    /* Configure C28 to IEP depending upon IEP instance usage */
#ifdef SOC_AM261X
    /* IEP0 base is used for AM261x */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_HDSL0_PRUICSS_SLICE, PRUICSS_CONST_TBL_ENTRY_C28, 0x02E0);
#else
    /* IEP1 base is used for AM243x */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_HDSL0_PRUICSS_SLICE, PRUICSS_CONST_TBL_ENTRY_C28, 0x02F0);
#endif

    /* Enable cycle counter */
    PRUICSS_configureCycleCounter(gPruIcssXHandle, pru_id, 1);
}
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_300M)
static void hdsl_pruicss_init_300m(void)
{
    uint32_t u_status = 0;

#if (CONFIG_HDSL0_CHANNEL0_ENABLED != 1) && (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    DebugP_log("\r\n Channel 2 can be enabled only if channel 0 is enabled because of the code overlay scheme needed in TX-PRU. See \"Overlay Scheme for TX-PRU\" section in \"HDSL Protocol Design\" of SDK documentation for more details.");
    DebugP_assert(0);
#endif

    /* Disable PRU core */
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    uint8_t rtu_pru_id = CONFIG_HDSL0_PRUICSS_RTU_PRU_ID;
    PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;
    PRUICSS_disableCore(gPruIcssXHandle, pru_id);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    uint8_t tx_pru_id = CONFIG_HDSL0_PRUICSS_TX_PRU_ID;
    PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
#endif

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_HDSL0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    /* Configure PRU-ICSS Interrupt Controller */
#if !defined(SOC_AM261X)
#if (CONFIG_HDSL0_PRUICSS_INSTANCE == 1)
    PRUICSS_intcInit(gPruIcssXHandle, &icss1_intc_initdata);
#else
    PRUICSS_intcInit(gPruIcssXHandle, &icss0_intc_initdata);
#endif
#endif
    /* configure C28 to IEP1 base */

#if (CONFIG_HDSL0_PRUICSS_INSTANCE == 1)
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTU_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A38);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A40);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TX_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A58);
#endif
#else
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTU_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A30);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A20);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TX_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0A50);
#endif
#endif /* CONFIG_HDSL0_PRUICSS_SLICE == 1 */
#else
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTU_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0238);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0240);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TX_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0258);
#endif
#else
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTU_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0230);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0220);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TX_PRU0, PRUICSS_CONST_TBL_ENTRY_C28, 0x0250);
#endif
#endif /* CONFIG_HDSL0_PRUICSS_SLICE == 1 */
#endif /* CONFIG_HDSL0_PRUICSS_INSTANCE == 1 */

    /* Configure C24 and enable cycle counter */

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    /* RTU_PRU Core */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, rtu_pru_id, PRUICSS_CONST_TBL_ENTRY_C24, 0x0000);
    PRUICSS_configureCycleCounter(gPruIcssXHandle, rtu_pru_id, 1);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    /* PRU Core */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, pru_id, PRUICSS_CONST_TBL_ENTRY_C24, 0x0007);
    PRUICSS_configureCycleCounter(gPruIcssXHandle, pru_id, 1);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    /* TX_PRU Core */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, tx_pru_id, PRUICSS_CONST_TBL_ENTRY_C24, 0x000E);
    PRUICSS_configureCycleCounter(gPruIcssXHandle, tx_pru_id, 1);
#endif
}
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
static void hdsl_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;

#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_FREE_RUN)
    /* Free Run Mode */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = HdslFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwarePru1_0);
#else
    const uint32_t *pru_firmware = HdslFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwarePru0_0);
#endif
#else
    /* Sync Mode */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = HdslFirmwareSyncPru1_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwareSyncPru1_0);
#else
    const uint32_t *pru_firmware = HdslFirmwareSyncPru0_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwareSyncPru0_0);
#endif
#endif
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_HDSL0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
}
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_300M)
static void hdsl_pruicss_load_run_fw_300m(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_FREE_RUN)
    /* Free Run Mode */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = HdslFirmwareRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(HdslFirmwareRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_HDSL0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = HdslFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwarePru1_0);
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = HdslFirmwareTxPru1_0;
    const uint8_t *tx_pru_firmware_1 = HdslFirmwareTxPru1_1;
    const uint8_t *tx_pru_firmware_2 = HdslFirmwareTxPru1_2;
    uint32_t tx_pru_firmware_size = sizeof(HdslFirmwareTxPru1_0);
    uint8_t tx_pru_id = CONFIG_HDSL0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = HdslFirmwareRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(HdslFirmwareRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_HDSL0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = HdslFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwarePru0_0);
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = HdslFirmwareTxPru0_0;
    const uint8_t *tx_pru_firmware_1 = HdslFirmwareTxPru0_1;
    const uint8_t *tx_pru_firmware_2 = HdslFirmwareTxPru0_2;
    uint32_t tx_pru_firmware_size = sizeof(HdslFirmwareTxPru0_0);
    uint8_t tx_pru_id = CONFIG_HDSL0_PRUICSS_TX_PRU_ID;
#endif
#endif
#else
    /* Sync Mode */
#if (CONFIG_HDSL0_PRUICSS_SLICE == 1)
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = HdslFirmwareSyncRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(HdslFirmwareSyncRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_HDSL0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = HdslFirmwareSyncPru1_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwareSyncPru1_0);
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = HdslFirmwareSyncTxPru1_0;
    const uint8_t *tx_pru_firmware_1 = HdslFirmwareSyncTxPru1_1;
    const uint8_t *tx_pru_firmware_2 = HdslFirmwareSyncTxPru1_2;
    uint32_t tx_pru_firmware_size = sizeof(HdslFirmwareSyncTxPru1_0);
    uint8_t tx_pru_id = CONFIG_HDSL0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = HdslFirmwareSyncRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(HdslFirmwareSyncRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_HDSL0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = HdslFirmwareSyncPru0_0;
    uint32_t pru_firmware_size = sizeof(HdslFirmwareSyncPru0_0);
    uint8_t pru_id = CONFIG_HDSL0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = HdslFirmwareSyncTxPru0_0;
    const uint8_t *tx_pru_firmware_1 = HdslFirmwareSyncTxPru0_1;
    const uint8_t *tx_pru_firmware_2 = HdslFirmwareSyncTxPru0_2;
    uint32_t tx_pru_firmware_size = sizeof(HdslFirmwareSyncTxPru0_0);
    uint8_t tx_pru_id = CONFIG_HDSL0_PRUICSS_TX_PRU_ID;
#endif
#endif
#endif

#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    uint32_t tx_pru_fw_size = 0;
#endif

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    /* Disable RTU-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    /* Disable TX-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    /* Load firmware to RTU-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_HDSL0_PRUICSS_SLICE), 0,
                                  (uint32_t *)rtu_pru_firmware, rtu_pru_firmware_size);
    DebugP_assert(0 != u_status);
#endif

#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_HDSL0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);
#endif

#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    /* Load firmware to TX-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_HDSL0_PRUICSS_SLICE), 0,
                                  (uint32_t *)tx_pru_firmware, tx_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /*
    NOTE: As this array is typecasted into a structure with 32-bit variables,
    32b alignment is required. This is done using linker.
    Validate alignment at runtime to catch linker configuration issues.
    */
    DebugP_assert(((uint32_t)tx_pru_firmware_2 & 0x3U) == 0U);

    gCopyTable = (HDSL_CopyTable *)tx_pru_firmware_2;
    tx_pru_fw_size = (gCopyTable->size1 > gCopyTable->size2)?(tx_pru_firmware_size + gCopyTable->size1):(tx_pru_firmware_size + gCopyTable->size2);
    DebugP_assert(tx_pru_fw_size <= TXPRU_IRAM_SIZE);

    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_HDSL0_PRUICSS_SLICE), 0x1500, (uint32_t *)tx_pru_firmware_1, gCopyTable->size1 + gCopyTable->size2);

    if(gCopyTable->load_addr1 < gCopyTable->load_addr2)
    {
        PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_HDSL0_PRUICSS_SLICE), gCopyTable->run_addr1, (uint32_t *) ((uint8_t *)tx_pru_firmware_1), gCopyTable->size1);
    }
    else
    {
        PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_HDSL0_PRUICSS_SLICE), gCopyTable->run_addr1, (uint32_t *) ((uint8_t *)tx_pru_firmware_1 + gCopyTable->size2), gCopyTable->size1);
    }

    /* Call HDSL_config_copy_table API with channel 0 specific handle */
    status = HDSL_config_copy_table(gAppHdslHandle[CONFIG_HDSL0][0], gCopyTable);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_config_copy_table() did not return success");
        return;
    }
#endif

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    /* Reset RTU-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    /* Reset TX-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    /* Enable RTU-PRU core to run the firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    /* Enable PRU core to run the firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    /* Enable TX-PRU core to run the firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
}
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)
static void hdsl_init(void)
{
    int32_t         status;
    uint8_t         es;
    uint32_t        period;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    HwiP_Params     hwi_prms;
    uint32_t        intr_num = HDSL_MEMORY_TRACE_R5F_IRQ_NUM;
#endif

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    /* Register PRU interrupt */
    HwiP_Params_init(&hwi_prms);
    hwi_prms.intNum   = intr_num;
    hwi_prms.callback = (void*)&hdsl_isr_fxn;
    HwiP_construct(&gPRUHwiObject, &hwi_prms);
#endif
    hdsl_iep_init();
    ClockP_usleep(HDSL_INIT_DELAY_US);

#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_FREE_RUN)
    es = 0;
#endif
#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_SYNC)
    es = 1;
#endif

    status = HDSL_set_sync_ctrl(gAppHdslHandle[CONFIG_HDSL0][0], es);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_set_sync_ctrl() did not return success");
        return;
    }

    if(es != 0)
    {
        DebugP_log("\r\nSYNC MODE\n");
        DebugP_log("\r\nEnter period for SYNC PULSE in unit of cycles(1 cycle = 4.44ns):");
        DebugP_scanf("%d", &period);

        /* Check Sync period condition
        *
        * Tsync = Cycle time for input SYNC pulse signal
        * Tmin  = MIN_SYNC_CYCLE_TIME
        * Tmax  = MAX_SYNC_CYCLE_TIME
        * Tsync = period / (PRU core clock freq)  =  period / 225)
        *
        * ES <= Tsync/Tmin  and  ES >= Tsync/Tmax
        */

        DebugP_assert((es <= (period/(MIN_SYNC_CYCLE_TIME * 225))) && (es >= (period/(MAX_SYNC_CYCLE_TIME * 225))));

        status = hdsl_enable_sync_signal(es, period);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: hdsl_enable_sync_signal() did not return success");
            return;
        }
        status = HDSL_generate_memory_image(gAppHdslHandle[CONFIG_HDSL0][0]);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("\r\n FAIL: HDSL_generate_memory_image() did not return success for channel 0");
            return;
        }
        status = hdsl_sync_calculation(gAppHdslHandle[CONFIG_HDSL0][0]);
        DebugP_assert(status == SystemP_SUCCESS);
    }
    else
    {
        DebugP_log( "\r\nFREE RUN MODE\n");
        ClockP_sleep(5);
        status = HDSL_generate_memory_image(gAppHdslHandle[CONFIG_HDSL0][0]);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("\r\n FAIL: HDSL_generate_memory_image() did not return success for channel 0");
            return;
        }
    }
}
#endif

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_300M)
static void hdsl_init_300m(void)
{
    int32_t         status;
    uint32_t        i;
    uint8_t         es = 0;
    const HDSL_Attrs *attrs;

    if(gAppHdslHandle[CONFIG_HDSL0][0] == NULL)
    {
        DebugP_log("\r\n FAIL: hdsl_init_300m() failed due to NULL handle");
        return;
    }

    attrs = HDSL_get_attrs(gAppHdslHandle[CONFIG_HDSL0][0]);
    if(attrs == NULL)
    {
        DebugP_log("\r\n FAIL: hdsl_init_300m() failed due to NULL attrs");
        return;
    }

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    HwiP_Params     hwi_prms;
    uint32_t        intr_num = HDSL_MEMORY_TRACE_R5F_IRQ_NUM;
#endif
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    /* Register PRU interrupt */
    HwiP_Params_init(&hwi_prms);
    hwi_prms.intNum   = intr_num;
    hwi_prms.callback = (void*)&hdsl_isr_fxn;
    HwiP_construct(&gPRUHwiObject, &hwi_prms);
#endif

    hdsl_iep_init();

    ClockP_usleep(HDSL_INIT_DELAY_US);

#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_SYNC)
    uint32_t period;
    DebugP_log("\r\nSYNC MODE\n");
    DebugP_log("\r\nEnter ES and period for SYNC PULSE in unit of cycles(1 cycle = 3.33ns):\r\n");
    DebugP_scanf("%d",&es);
    DebugP_scanf("%d",&period);

    for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
    {
        if(attrs->channel_mask & (1 << i))
        {
            status = HDSL_set_sync_ctrl(gAppHdslHandle[CONFIG_HDSL0][i], es);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\n FAIL: HDSL_set_sync_ctrl() did not return success for channel %u", i);
                return;
            }
        }
    }

    /* Check Sync period condition
     *
     * Tsync = Cycle time for input SYNC pulse signal
     * Tmin  = MIN_SYNC_CYCLE_TIME
     * Tmax  = MAX_SYNC_CYCLE_TIME
     * Tsync = period / (PRU core clock freq)  =  period / 300)
     *
     * ES <= Tsync/Tmin  and  ES >= Tsync/Tmax
     */

    DebugP_assert((es <= (period/(MIN_SYNC_CYCLE_TIME * 300))) && (es >= (period/(MAX_SYNC_CYCLE_TIME * 300))));
    status = hdsl_enable_sync_signal(es, period);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: hdsl_enable_sync_signal() did not return success");
        return;
    }

    for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
    {
        if(attrs->channel_mask & (1 << i))
        {
            status = HDSL_generate_memory_image(gAppHdslHandle[CONFIG_HDSL0][i]);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\n FAIL: HDSL_generate_memory_image() did not return success for channel %u", i);
                return;
            }
            status = hdsl_sync_calculation(gAppHdslHandle[CONFIG_HDSL0][i]);
            DebugP_assert(status == SystemP_SUCCESS);
        }
    }

#endif
#if (CONFIG_HDSL0_MODE == HDSL_OPERATIONAL_MODE_FREE_RUN)
    DebugP_log( "\r\nFREE RUN MODE\n");

    for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
    {
        if(attrs->channel_mask & (1 << i))
        {
            status = HDSL_set_sync_ctrl(gAppHdslHandle[CONFIG_HDSL0][i], es);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\n FAIL: HDSL_set_sync_ctrl() did not return success for channel %u", i);
                return;
            }
            status = HDSL_generate_memory_image(gAppHdslHandle[CONFIG_HDSL0][i]);
            if(SystemP_SUCCESS != status)
            {
                DebugP_log("\r\n FAIL: HDSL_generate_memory_image() did not return success for channel %u", i);
                return;
            }
        }
    }
#endif
}
#endif

static void hdsl_read_pc_short_msg(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_data;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_read_pc_short_msg() failed due to NULL handle");
        return;
    }

    status = HDSL_read_pc_short_msg(handle, ENCODER_RSSI_REG_ADDRESS, &pc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }

    DebugP_log("\r\n Parameter channel short message read  : Slave RSSI (address 0x7C) = %x (should be 0x7 which indicates best signal strength)", pc_data);

    status = HDSL_read_pc_short_msg(handle, ENCODER_STATUS0_REG_ADDRESS, &pc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }

    DebugP_log("\r\n Parameter channel short message read  : Address 0x40 = %x (should be 0x1)", pc_data);
}

static void hdsl_write_pc_short_msg(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_data;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_write_pc_short_msg() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel short message write : 0xab to PING register (address 0x7F)");

    status = HDSL_write_pc_short_msg(handle, ENCODER_PING_REG_ADDRESS, 0xab, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_short_msg() did not return success");
        return;
    }

    status = HDSL_read_pc_short_msg(handle, ENCODER_PING_REG_ADDRESS, &pc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }
    DebugP_log("\r\n Parameter channel short message read  : PING register (address 0x7F) = %x (should be 0xab) ", pc_data);


    DebugP_log("\r\n Parameter channel short message write : 0xcd to PING register (address 0x7F)");

    status = HDSL_write_pc_short_msg(handle, ENCODER_PING_REG_ADDRESS, 0xcd, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_short_msg() did not return success");
        return;
    }

    status = HDSL_read_pc_short_msg(handle, ENCODER_PING_REG_ADDRESS, &pc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }
    DebugP_log("\r\n Parameter channel short message read  : PING register (address 0x7F) = %x (should be 0xcd) ", pc_data);
}

static void hdsl_display_menu(void)
{
    DebugP_log("\r\n");
    DebugP_log("\r\n |------------------------------------------------------------------------------|");
    DebugP_log("\r\n |                                    MENU                                      |");
    DebugP_log("\r\n |------------------------------------------------------------------------------|");
    DebugP_log("\r\n | %2d : Safe Position                                                           |", MENU_SAFE_POSITION);
    DebugP_log("\r\n | %2d : Quality Monitoring                                                      |", MENU_QUALITY_MONITORING);
    DebugP_log("\r\n | %2d : Events                                                                  |", MENU_EVENTS);
    DebugP_log("\r\n | %2d : Summarized Slave Status                                                 |", MENU_SUMMARY);
    DebugP_log("\r\n | %2d : Acceleration Error Counter                                              |", MENU_ACC_ERR_CNT);
    DebugP_log("\r\n | %2d : RSSI                                                                    |", MENU_RSSI);
    DebugP_log("\r\n | %2d : Parameter Channel Short Message Write                                   |", MENU_PC_SHORT_MSG_WRITE);
    DebugP_log("\r\n | %2d : Parameter Channel Short Message Read                                    |", MENU_PC_SHORT_MSG_READ);
    DebugP_log("\r\n | %2d : Parameter Channel Long Message Read                                     |", MENU_DIRECT_READ_RID0_LENGTH4);
    DebugP_log("\r\n |      Access on RID 0x0, direct read access with length 4                     |");
    DebugP_log("\r\n | %2d : Parameter Channel Long Message Read                                     |", MENU_DIRECT_READ_RID81_LENGTH8);
    DebugP_log("\r\n |      Access on RID 0x81, direct read access with length 8                    |");
    DebugP_log("\r\n | %2d : Parameter Channel Long Message Read                                     |", MENU_DIRECT_READ_RID81_LENGTH2);
    DebugP_log("\r\n |      Access on RID 0x81, direct read access with length 2 and offset 3       |");
    DebugP_log("\r\n | %2d : Parameter Channel Long Message Write                                    |", MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0);
    DebugP_log("\r\n |      Access on RID 0x0, indirect write, length 8, with offset 0              |");
    DebugP_log("\r\n | %2d : Parameter Channel Long Message Write                                    |", MENU_INDIRECT_WRITE_RID0_LENGTH8);
    DebugP_log("\r\n |      Access on RID 0x0; indirect write, length 8, without offset value       |");
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    DebugP_log("\r\n | %2d : HDSL registers into Memory                                              |", MENU_HDSL_REG_INTO_MEMORY);
#endif
    DebugP_log("\r\n |------------------------------------------------------------------------------|\n");
    DebugP_log("\r\n Enter value: ");
}

static void hdsl_direct_read_rid0_length4(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_buf0, pc_buf1, pc_buf2, pc_buf3;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_direct_read_rid0_length4() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel long message read : RID 0, Length 4");

    /* Set the parameter channel buffers to 0xff */
    status = HDSL_write_pc_buffer(handle, 0, 0xff);
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 1, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 2, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 3, 0xff);
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_buffer() did not return success");
        return;
    }

    status = HDSL_read_pc_long_msg(handle, 0, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_4, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_get_pc_long_msg_error() did not return success");
        return;
    }

    if(enc_error != 0)
    {
        DebugP_log("\r\n FAIL: Encoder reported parameter error for long message");
        return;
    }

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 0\r\n");
        return;
    }

    if(pc_buf0 == 'R')
    {
        status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 1\r\n");
            return;
        }
        if(pc_buf1 == 'O')
        {
            status = HDSL_read_pc_buffer(handle, 2, &pc_buf2);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 2\r\n");
                return;
            }
            if(pc_buf2 == 'O')
            {
                status = HDSL_read_pc_buffer(handle, 3, &pc_buf3);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 3\r\n");
                    return;
                }
                if(pc_buf3 == 'T')
                {
                    DebugP_log("\r\n PASS : Read \"ROOT\"");
                }
                else
                {
                    DebugP_log("\r\n FAIL: PC_BUFFER3 != T (It is %u)", pc_buf3);
                }
            }
            else
            {
                DebugP_log("\r\n FAIL: PC_BUFFER2 != O (It is %u)", pc_buf2);
            }
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != O (It is %u)", pc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != R (It is %u)", pc_buf0);
    }
}

static void hdsl_direct_read_rid81_length8(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_buf0, pc_buf1, pc_buf2, pc_buf3, pc_buf4, pc_buf5, pc_buf6, pc_buf7;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_direct_read_rid81_length8() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel long message read : RID 0x81, Length 8");

    /* Set the parameter channel buffers to 0xff */
    status = HDSL_write_pc_buffer(handle, 0, 0xff);
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 1, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 2, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 3, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 4, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 5, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 6, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 7, 0xff);
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_buffer() did not return success");
        return;
    }

    status = HDSL_read_pc_long_msg(handle, 0x81, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_get_pc_long_msg_error() did not return success");
        return;
    }

    if(enc_error != 0)
    {
        DebugP_log("\r\n FAIL: Encoder reported parameter error for long message");
        return;
    }

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 0\r\n");
        return;
    }

    if(pc_buf0 == 'R')
    {
        status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 1\r\n");
            return;
        }
        if(pc_buf1 == 'E')
        {
            status = HDSL_read_pc_buffer(handle, 2, &pc_buf2);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 2\r\n");
                return;
            }
            if(pc_buf2 == 'S')
            {
                status = HDSL_read_pc_buffer(handle, 3, &pc_buf3);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 3\r\n");
                    return;
                }
                if(pc_buf3 == 'O')
                {
                    status = HDSL_read_pc_buffer(handle, 4, &pc_buf4);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 4\r\n");
                        return;
                    }
                    if(pc_buf4 == 'L')
                    {
                        status = HDSL_read_pc_buffer(handle, 5, &pc_buf5);
                        if(status != SystemP_SUCCESS)
                        {
                            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 5\r\n");
                            return;
                        }
                        if(pc_buf5 == 'U')
                        {
                            status = HDSL_read_pc_buffer(handle, 6, &pc_buf6);
                            if(status != SystemP_SUCCESS)
                            {
                                DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 6\r\n");
                                return;
                            }
                            if(pc_buf6 == 'T')
                            {
                                status = HDSL_read_pc_buffer(handle, 7, &pc_buf7);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 7\r\n");
                                    return;
                                }
                                if(pc_buf7 == 'N')
                                {
                                    DebugP_log("\r\n PASS : Read \"RESOLUTN\"");
                                }
                                else
                                {
                                    DebugP_log("\r\n FAIL: PC_BUFFER7 != N (It is %u)", pc_buf7);
                                }
                            }
                            else
                            {
                                DebugP_log("\r\n FAIL: PC_BUFFER6 != T (It is %u)", pc_buf6);
                            }
                        }
                        else
                        {
                            DebugP_log("\r\n FAIL: PC_BUFFER5 != U (It is %u)", pc_buf5);
                        }
                    }
                    else
                    {
                        DebugP_log("\r\n FAIL: PC_BUFFER4 != L (It is %u)", pc_buf4);
                    }

                }
                else
                {
                    DebugP_log("\r\n FAIL: PC_BUFFER3 != O (It is %u)", pc_buf3);
                }
            }
            else
            {
                DebugP_log("\r\n FAIL: PC_BUFFER2 != S (It is %u)", pc_buf2);
            }
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != E (It is %u)", pc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != R (It is %u)", pc_buf0);
    }
}

static void hdsl_direct_read_rid81_length2(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_buf0, pc_buf1;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_direct_read_rid81_length2() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel long message read : RID 0x81, Offset 3, Length 2");

    /* Set the parameter channel buffers to 0xaa */
    status = HDSL_write_pc_buffer(handle, 0, 0xaa);
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 1, 0xaa);
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_buffer() did not return success");
        return;
    }

    status = HDSL_read_pc_long_msg(handle, 0x81, HDSL_LONG_MSG_ADDR_WITH_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_2, 3, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_get_pc_long_msg_error() did not return success");
        return;
    }

    if(enc_error != 0)
    {
        DebugP_log("\r\n FAIL: Encoder reported parameter error for long message");
        return;
    }

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 0\r\n");
        return;
    }

    if(pc_buf0 == 0x00)
    {
        status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 1\r\n");
            return;
        }
        if(pc_buf1 == 0x0f)
        {
            DebugP_log("\r\n PASS : Read 15");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x0f (It is %u)", pc_buf1);
        }

    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x00 (It is %u)", pc_buf0);
    }
}

static void hdsl_indirect_write_rid0_length8_offset0(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_buf0, pc_buf1;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_indirect_write_rid0_length8_offset0() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel long message write : RID 0x0, Offset 0, Length 8");

    status = HDSL_write_pc_long_msg(handle, 0x0, HDSL_LONG_MSG_ADDR_WITH_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_long_msg() did not return success");
        return;
    }

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_get_pc_long_msg_error() did not return success");
        return;
    }

    if(enc_error != 0)
    {
        DebugP_log("\r\n Encoder reported parameter error for long message (expected for parameters used in this API call)");
    }

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 0\r\n");
        return;
    }

    if(pc_buf0 == 0x41)
    {
        status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 1\r\n");
            return;
        }
        if(pc_buf1 == 0x10)
        {
            DebugP_log("\r\n PC_BUFFER0 = 0x41, PC_BUFFER1 = 0x10 (Write access not possible)");
            DebugP_log("\r\n PASS ");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x10 (It is %u)", pc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x41 (It is %u)", pc_buf0);
    }

}

static void hdsl_indirect_write_rid0_length8(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint8_t pc_buf0, pc_buf1;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_indirect_write_rid0_length8() failed due to NULL handle");
        return;
    }

    DebugP_log("\r\n Parameter channel long message write : RID 0x0, Length 8");

    status = HDSL_write_pc_long_msg(handle, 0x0, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_long_msg() did not return success");
        return;
    }

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_get_pc_long_msg_error() did not return success");
        return;
    }

    if(enc_error != 0)
    {
        DebugP_log("\r\n Encoder reported parameter error for long message (expected for parameters used in this API call)");
    }

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 0\r\n");
        return;
    }

    if(pc_buf0 == 0x41)
    {
        status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r\n FAIL: HDSL_read_pc_buffer() did not return success for buffer 1\r\n");
            return;
        }
        if(pc_buf1 == 0x10)
        {
            DebugP_log("\r\n PC_BUFFER0 = 0x41, PC_BUFFER1 = 0x10 (Write access not possible) ");
            DebugP_log("\r\n PASS ");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x10 (It is %u)", pc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x41 (It is %u)", pc_buf0);
    }
}

/**
 * \brief Read menu selection from user input
 *
 * Reads a menu command from user via DebugP_scanf. Validates the input against
 * MENU_LIMIT. In debug mode for single channel (only on AM243x), also handles
 * MENU_HDSL_REG_INTO_MEMORY option and prompts for trace count.
 *
 * \return Menu command ID on success
 * \retval MENU_SAFE_POSITION if invalid input is provided (out of range)
 * \retval MENU_INVALID if trace count validation fails in debug mode
 */
static uint32_t hdsl_get_menu(void)
{
    uint32_t cmd;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    if(DebugP_scanf("%u\n", &cmd) < 0 || (cmd >= MENU_LIMIT))
#else
    if(DebugP_scanf("%u\n", &cmd) < 0 || (cmd >= MENU_LIMIT) || (cmd == MENU_HDSL_REG_INTO_MEMORY))
#endif
    {
        DebugP_log("\r\n WARNING: invalid option, Safe position selected");
        cmd = MENU_SAFE_POSITION;
    }

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)

    if(cmd == MENU_HDSL_REG_INTO_MEMORY)
    {
       DebugP_log("\r\n| How many traces you want to copy : ");
       if(DebugP_scanf("%u\n", &gTraceCount) < 0 || gTraceCount >= NUM_RESOURCES)
       {
           DebugP_log("\r\n| WARNING: invalid data\n|\n|\n");
           return MENU_INVALID;
       }
    }
#endif
    return cmd;
}

#ifdef HDSL_AM64xE1_TRANSCEIVER
static void hdsl_i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    /* P20 = LED 3 bits, pin, 2 bits port.*/
    uint32_t            ioIndex = 0x10;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);

    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    if(status == SystemP_SUCCESS)
    {
        /* Set output to HIGH before config so that LED start with On state */
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);


    }
    TCA6424_close(&gTCA6424_Config);
}
#endif

/**
 * \brief Reads encoder resolution from HDSL encoder parameter channel
 *
 * Queries the encoder via parameter channel long message (RID 0x81, indirect addressing)
 * to read the 32-bit resolution value stored in bytes 0-3. Returns the log2 of this value
 * which represents the number of single-turn position bits.
 *
 * \param[in] handle  Handle to HDSL driver instance
 *
 * \return Resolution in bits (log2 of raw encoder resolution value)
 *
 * \note On failure, this function calls DebugP_assert() which will halt execution.
 *       Failure can occur if parameter channel communication times out or if the
 *       encoder does not respond. Ensure encoder link is established (QM = 15)
 *       before calling this function.
 */
static uint32_t hdsl_read_encoder_resolution(HDSL_Handle handle)
{
    int32_t status = SystemP_FAILURE;
    uint32_t resolution = 0;
    uint8_t pc_buf0, pc_buf1, pc_buf2, pc_buf3;
    uint32_t raw_resolution;
    double log_result;
    uint8_t enc_error = 0;

    if(handle == NULL)
    {
        DebugP_log("\r\n\n|ERROR: hdsl_read_encoder_resolution() failed due to NULL handle");
        return 0;
    }

    /* Set the parameter channel buffers to 0xff */
    status = HDSL_write_pc_buffer(handle, 0, 0xff);
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 1, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 2, 0xff);
    }
    if(status == SystemP_SUCCESS)
    {
        status = HDSL_write_pc_buffer(handle, 3, 0xff);
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_buffer() did not return success");
        return 0;  /* Return 0 resolution on error */
    }

    /* Parameter channel long message read with RID 0x81, Offset 5, Length 4
     * for reading resolution */

    status = HDSL_read_pc_long_msg(handle, 0x81, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_4, 0, LONG_MSG_TIMEOUT);

    DebugP_assert(SystemP_SUCCESS == status);

    /* Check if encoder reported parameter error */
    status = HDSL_get_pc_long_msg_error(handle, &enc_error);
    DebugP_assert(SystemP_SUCCESS == status);
    DebugP_assert(enc_error == 0);

    status = HDSL_read_pc_buffer(handle, 0, &pc_buf0);
    DebugP_assert(SystemP_SUCCESS == status);
    status = HDSL_read_pc_buffer(handle, 1, &pc_buf1);
    DebugP_assert(SystemP_SUCCESS == status);
    status = HDSL_read_pc_buffer(handle, 2, &pc_buf2);
    DebugP_assert(SystemP_SUCCESS == status);
    status = HDSL_read_pc_buffer(handle, 3, &pc_buf3);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Assemble raw resolution value from 4 bytes */
    raw_resolution = (pc_buf0 << 24) | (pc_buf1 << 16) | (pc_buf2 << 8) | pc_buf3;

    /* Validate resolution is non-zero */
    DebugP_assert(raw_resolution != 0);

    /* Cast log2 result with proper rounding to avoid precision loss */
    log_result = log2((double)raw_resolution);

    /* Validate result is in valid range for encoder resolution (0-32 bits) */
    DebugP_assert(log_result >= 0.0 && log_result <= 32.0);

    /* Round to nearest integer */
    resolution = (uint32_t)(log_result + 0.5);

    return resolution;
}

/**
 * \brief Main diagnostic function for HDSL encoder interface testing
 *
 * This function performs the following key operations:
 * 1. System Initialization:
 *    - Opens UART driver for console communication via Drivers_open()
 *    - Initializes board-specific drivers via Board_driversOpen()
 *    - Optionally enables UDMA for memory tracing in debug mode (AM243x only)
 *
 * 2. Hardware Configuration:
 *    - Opens PRU-ICSS peripheral handle via PRUICSS_open()
 *    - Configures GPIO pins for booster pack encoder power enable (if applicable)
 *    - Sets SA MUX mode for correct signal routing to HDSL interface (AM243x only, if applicable)
 *    - Configures GPIO42 for HDSL transceiver mode selection (AM243x only, if applicable)
 *
 * 3. HDSL Driver Initialization:
 *    - Initializes HDSL parameters structure via HDSL_params_init()
 *    - Creates HDSL handles for enabled channels (0, 1, 2) via HDSL_open()
 *    - Loads and runs appropriate PRU firmware (225MHz or 300MHz variants)
 *
 * 4. Encoder Link Establishment:
 *    For each enabled channel:
 *    - Polls HDSL_get_master_qm() waiting for link establishment (QM bit 7 set)
 *    - Waits until quality monitoring value reaches 15 (optimal link quality)
 *    - Times out with error messages if encoder not detected
 *
 * 5. Encoder Parameter Discovery:
 *    For each channel:
 *    - Reads and displays quality monitoring, edges, cable delay, RSSI
 *    - Retrieves encoder ID bytes via HDSL_get_enc_id() to determine:
 *      * Acceleration bits (lower 4 bits + 8)
 *      * Position bits (bits 4-9 + acceleration bits)
 *      * Position type (bipolar vs unipolar, bit 10)
 *    - Reads encoder resolution from parameter channel via hdsl_read_encoder_resolution()
 *    - Calculates single-turn and multi-turn bit counts
 *    - Stores resolution, multi-turn, and mask values
 *
 * 6. Interactive Diagnostic Loop:
 *    - Presents menu-driven interface via hdsl_display_menu()
 *    - Processes user selections via hdsl_process_request()
 *    - Supports operations like:
 *      * Reading safe position, quality monitoring, events
 *      * Parameter channel short/long message read/write
 *      * Encoder resolution queries
 *      * Memory trace capture (only in debug build for AM243x)
 *
 */
void hdsl_diagnostic_main(void *arg)
{
    int32_t             status;
    uint32_t            val, acc_bits, pos_bits, i, res_value, multi_turn_value, menu;
    uint8_t             ureg, enc_id0, enc_id1, enc_id2;
    HDSL_Params         params;
    const HDSL_Attrs    *attrs;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    int32_t     ret_val = UDMA_SOK;
#endif
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_) && !defined(SOC_AM261X)
    /* UDMA initialization */
    gChHandle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */
    /* Channel enable */
    ret_val = Udma_chEnable(gChHandle);
    if(UDMA_SOK != ret_val)
    {
        DebugP_log("\r\n FAIL: Udma_chEnable() did not return success, exiting hdsl_diagnostic_main()\r\n");
        return;
    }
#endif
    /*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_HDSL0_BOOSTER_PACK_ENABLE && CONFIG_HDSL0_CHANNEL0_ENABLED)
        GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
        GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
    /*B17 pin High for Enabling ch2 in booster pack */
#if (CONFIG_HDSL0_BOOSTER_PACK_ENABLE && CONFIG_HDSL0_CHANNEL2_ENABLED)
        GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
        GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    if(gPruIcssXHandle == NULL)
    {
        DebugP_log("\r\n FAIL: PRUICSS_open() did not return valid handle, exiting hdsl_diagnostic_main()\r\n");
        return;
    }
#ifndef HDSL_AM64xE1_TRANSCEIVER
#ifdef CONFIG_HDSL0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. This is required to remap EnDAT signals correctly via Interface card.*/
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif
#if (CONFIG_HDSL0_BOOSTER_PACK_ENABLE == 0)
    /*Configure GPIO42 for HDSL mode.*/
    GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN, CONFIG_GPIO0_DIR);
    GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
#endif
#else
    /*Configure GPIO42 for HDSL mode. New transceiver card needs the pin to be configured as input*/
    HW_WR_REG32(PRG0_PRU1_GPI9_CONFIG_REG, GPIO_INPUT_MODE_CONFIG);   /* PRG0_PRU1_GPI9 as input */
    hdsl_i2c_io_expander(NULL);
#endif

    /* Initialize HDSL handle */
    DebugP_log( "\n\n Hiperface DSL Diagnostic\n");
    HDSL_params_init(&params);
    params.pruicss_handle = gPruIcssXHandle;

#if (PRU_CORE_CLOCK_FREQ == PRU_CORE_CLOCK_FREQ_225M)

    hdsl_pruicss_init();

    /* In non-load share mode,
     *  - Ther is no need to configure params.channel
     *  - gAppHdslHandle[CONFIG_HDSLx][0] should be used always, irrespective of channel used
     */
    gAppHdslHandle[CONFIG_HDSL0][0] = HDSL_open(CONFIG_HDSL0, &params);

    if(gAppHdslHandle[CONFIG_HDSL0][0] == NULL)
    {
        DebugP_log("\r\n FAIL: HDSL_open() did not return valid handle, exiting hdsl_diagnostic_main()\r\n");
        return;
    }

    hdsl_init();

    /* Initialize HDSL hardware: configure PRU-ICSS clock dividers (RX/TX), set GP MUX for
     * EnDAT mode, and clear channel CFG0 registers. This configures the low-level PRU
     * peripheral for HDSL communication at the configured core clock frequency.
     * Must be called after HDSL_open() and before loading PRU firmware. */

    status = HDSL_hw_init(gAppHdslHandle[CONFIG_HDSL0][0]);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_hw_init() did not return success, exiting hdsl_diagnostic_main()\r\n");
        return;
    }

    hdsl_pruicss_load_run_fw();
#else

    hdsl_pruicss_init_300m();

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    params.channel = 0;
    gAppHdslHandle[CONFIG_HDSL0][0] = HDSL_open(CONFIG_HDSL0, &params);

    if(gAppHdslHandle[CONFIG_HDSL0][0] == NULL)
    {
        DebugP_log("\r\n FAIL: HDSL_open() did not return valid handle, exiting hdsl_diagnostic_main()\r\n");
        return;
    }
#endif
#if (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    params.channel = 1;
    gAppHdslHandle[CONFIG_HDSL0][1] = HDSL_open(CONFIG_HDSL0, &params);

    if(gAppHdslHandle[CONFIG_HDSL0][1] == NULL)
    {
        DebugP_log("\r\n FAIL: HDSL_open() did not return valid handle, exiting hdsl_diagnostic_main()\r\n");
        return;
    }
#endif
#if (CONFIG_HDSL0_CHANNEL2_ENABLED == 1)
    params.channel = 2;
    gAppHdslHandle[CONFIG_HDSL0][2] = HDSL_open(CONFIG_HDSL0, &params);

    if(gAppHdslHandle[CONFIG_HDSL0][2] == NULL)
    {
        DebugP_log("\r\n FAIL: HDSL_open() did not return valid handle, exiting hdsl_diagnostic_main()\r\n");
        return;
    }
#endif

    hdsl_init_300m();

#if (CONFIG_HDSL0_CHANNEL0_ENABLED == 1)
    gFirstEnabledChannel = 0;
#elif (CONFIG_HDSL0_CHANNEL1_ENABLED == 1)
    gFirstEnabledChannel = 1;
#else /* (CONFIG_HDSL0_CHANNEL2_ENABLED == 1) */
    gFirstEnabledChannel = 2;
#endif
    /* Initialize HDSL hardware: Set GP MUX for EnDAT mode, configure PRU-ICSS clock dividers (RX/TX),
     * enable load share mode (mandatory for 300 MHz core clock operation), and clear channel CFG0
     * registers. This configures the low-level PRU peripheral for HDSL communication at 300 MHz.
     * Must be called after HDSL_open() (which internally configures the channel mask) and before
     * loading PRU firmware.
     * NOTE: In load share mode, call this API only once per PRU slice (not per channel) as the
     * configuration is shared across all channels on the same PRU slice. */
    status = HDSL_hw_init(gAppHdslHandle[CONFIG_HDSL0][gFirstEnabledChannel]);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\n FAIL: HDSL_hw_init() did not return success, exiting hdsl_diagnostic_main()\r\n");
        return;
    }

    hdsl_pruicss_load_run_fw_300m();
#endif

    /* Get attrs from first handle (shared across all channels) */
    attrs = HDSL_get_attrs(gAppHdslHandle[CONFIG_HDSL0][0]);

    if(attrs == NULL)
    {
        DebugP_log("\r\n FAIL: HDSL_get_attrs() did not return valid attrs, exiting hdsl_diagnostic_main()\r\n");
        return;
    }

    DebugP_log( "\r\n HDSL setup finished for PRU-ICSS instance %u slice %u\n\n", attrs->pruicss_instance, attrs->pruicss_slice);

    /* Poll each enabled channel to establish encoder link and read parameters */
    if(attrs->load_share_enabled)
    {
        for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
        {

            if(!(attrs->channel_mask & (1 << i)))
            {
                continue;
            }

            while(1)
            {
                status = HDSL_get_master_qm(gAppHdslHandle[CONFIG_HDSL0][i], &ureg);
                DebugP_assert(status == SystemP_SUCCESS);

                if((ureg & QM_LINK_ESTABLISHED) != 0)
                    break;

                DebugP_log( "\r\n Hiperface DSL encoder not detected on channel %u", i);
                ClockP_sleep(HDSL_QM_POLL_SLEEP_SEC);
            }

            /* Wait until QM is 15 */
            while(1)
            {
                status = HDSL_get_master_qm(gAppHdslHandle[CONFIG_HDSL0][i], &ureg);
                DebugP_assert(status == SystemP_SUCCESS);

                if(ureg == QM_LINK_ESTABLISHED_AND_VALUE_15)
                    break;

                DebugP_log( "\r\n QM is not 15 for channel %u", i);
                ClockP_sleep(HDSL_QM_POLL_SLEEP_SEC);
            }

            DebugP_log( "\r\n");
            DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
            DebugP_log( "\r |            Hiperface DSL Diagnostic : Channel %u                               |\n", i);
            DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
            DebugP_log( "\r |                                                                               |\n");
            DebugP_log( "\r | Quality monitoring value: %u                                                  |\n", ureg & HDSL_LOWER_NIBBLE_MASK);
            status = HDSL_get_edges(gAppHdslHandle[CONFIG_HDSL0][i], &ureg);
            DebugP_assert(status == SystemP_SUCCESS);
            DebugP_log( "\r | Edges: 0x%x                                                                    |", ureg);
            status = HDSL_get_delay(gAppHdslHandle[CONFIG_HDSL0][i], &ureg);
            DebugP_assert(status == SystemP_SUCCESS);
            DebugP_log("\r\n | Cable delay: %u                                                                |", ureg & HDSL_LOWER_NIBBLE_MASK);
            DebugP_log("\r\n | RSSI: %u                                                                       |", (ureg & HDSL_UPPER_NIBBLE_MASK) >> HDSL_UPPER_NIBBLE_SHIFT);
            /* Read encoder parameters: ID, acceleration bits, position bits, and resolution */
            status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][i], 0, &enc_id0);
            DebugP_assert(status == SystemP_SUCCESS);
            status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][i], 1, &enc_id1);
            DebugP_assert(status == SystemP_SUCCESS);
            status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][i], 2, &enc_id2);
            DebugP_assert(status == SystemP_SUCCESS);
            val = enc_id0 | (enc_id1 << 8) | (enc_id2 << 16);

            acc_bits = val & ENC_ID_ACC_BITS_MASK;
            acc_bits += HDSL_ENC_ID_ACC_BITS_OFFSET;
            pos_bits = (val & ENC_ID_POS_BITS_MASK) >> ENC_ID_POS_BITS_SHIFT;
            pos_bits += acc_bits;
            DebugP_log("\r\n | Encoder ID: 0x%x", val);
            DebugP_log( "(");
            DebugP_log( "Acceleration bits: %u, ", acc_bits);
            DebugP_log( "Position bits: %u,", pos_bits);
            DebugP_log( "%s", val & ENC_ID_BIPOLAR_FLAG ? " Bipolar position" : " Unipolar position");
            DebugP_log(")|");
            res_value = hdsl_read_encoder_resolution(gAppHdslHandle[CONFIG_HDSL0][i]);
            multi_turn_value = pos_bits - res_value;
            status = HDSL_set_res(gAppHdslHandle[CONFIG_HDSL0][i], res_value);
            DebugP_assert(status == SystemP_SUCCESS);
            status = HDSL_set_multi_turn(gAppHdslHandle[CONFIG_HDSL0][i], multi_turn_value);
            DebugP_assert(status == SystemP_SUCCESS);
            status = HDSL_set_mask(gAppHdslHandle[CONFIG_HDSL0][i], (1ULL << res_value) - 1);
            DebugP_assert(status == SystemP_SUCCESS);

            if(multi_turn_value)
            {
                DebugP_log( "\r\n | Single-turn bits: %u, Multi-turn bits: %u                                     |", pos_bits - multi_turn_value, multi_turn_value);
            }
            else
            {
                DebugP_log( "\r\n | Single-turn bits: %u                                                          |", pos_bits);
            }
            DebugP_log("\r\n |-------------------------------------------------------------------------------|");
        }
    }
    else
    {
        while(1)
        {
            status = HDSL_get_master_qm(gAppHdslHandle[CONFIG_HDSL0][0], &ureg);
            DebugP_assert(status == SystemP_SUCCESS);

            if((ureg & QM_LINK_ESTABLISHED) != 0)
                break;

            DebugP_log( "\r\n Hiperface DSL encoder not detected");
            ClockP_sleep(HDSL_QM_POLL_SLEEP_SEC);
        }

        /* Wait until QM is 15 */
        while(1)
        {
            status = HDSL_get_master_qm(gAppHdslHandle[CONFIG_HDSL0][0], &ureg);
            DebugP_assert(status == SystemP_SUCCESS);

            if(ureg == QM_LINK_ESTABLISHED_AND_VALUE_15)
                break;

            DebugP_log( "\r\n QM is not 15");
            ClockP_sleep(HDSL_QM_POLL_SLEEP_SEC);
        }

        DebugP_log( "\r\n");
        DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
        DebugP_log( "\r |            Hiperface DSL Diagnostic : Channel                                 |\n");
        DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
        DebugP_log( "\r |                                                                               |\n");
        DebugP_log( "\r | Quality monitoring value: %u                                                  |\n", ureg & HDSL_LOWER_NIBBLE_MASK);
        status = HDSL_get_edges(gAppHdslHandle[CONFIG_HDSL0][0], &ureg);
        DebugP_assert(status == SystemP_SUCCESS);
        DebugP_log( "\r | Edges: 0x%x                                                                    |", ureg);
        status = HDSL_get_delay(gAppHdslHandle[CONFIG_HDSL0][0], &ureg);
        DebugP_assert(status == SystemP_SUCCESS);
        DebugP_log("\r\n | Cable delay: %u                                                                |", ureg & HDSL_LOWER_NIBBLE_MASK);
        DebugP_log("\r\n | RSSI: %u                                                                       |", (ureg & HDSL_UPPER_NIBBLE_MASK) >> HDSL_UPPER_NIBBLE_SHIFT);
        /* Read encoder parameters: ID, acceleration bits, position bits, and resolution */
        status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][0], 0, &enc_id0);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][0], 1, &enc_id1);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HDSL_get_enc_id(gAppHdslHandle[CONFIG_HDSL0][0], 2, &enc_id2);
        DebugP_assert(status == SystemP_SUCCESS);
        val = enc_id0 | (enc_id1 << 8) | (enc_id2 << 16);

        acc_bits = val & ENC_ID_ACC_BITS_MASK;
        acc_bits += HDSL_ENC_ID_ACC_BITS_OFFSET;
        pos_bits = (val & ENC_ID_POS_BITS_MASK) >> ENC_ID_POS_BITS_SHIFT;
        pos_bits += acc_bits;
        DebugP_log("\r\n | Encoder ID: 0x%x", val);
        DebugP_log( "(");
        DebugP_log( "Acceleration bits: %u, ", acc_bits);
        DebugP_log( "Position bits: %u,", pos_bits);
        DebugP_log( "%s", val & ENC_ID_BIPOLAR_FLAG ? " Bipolar position" : " Unipolar position");
        DebugP_log(")|");
        res_value = hdsl_read_encoder_resolution(gAppHdslHandle[CONFIG_HDSL0][0]);
        multi_turn_value = pos_bits - res_value;
        status = HDSL_set_res(gAppHdslHandle[CONFIG_HDSL0][0], res_value);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HDSL_set_multi_turn(gAppHdslHandle[CONFIG_HDSL0][0], multi_turn_value);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HDSL_set_mask(gAppHdslHandle[CONFIG_HDSL0][0], (1ULL << res_value) - 1);
        DebugP_assert(status == SystemP_SUCCESS);

        if(multi_turn_value)
        {
            DebugP_log( "\r\n | Single-turn bits: %u, Multi-turn bits: %u                                     |", pos_bits - multi_turn_value, multi_turn_value);
        }
        else
        {
            DebugP_log( "\r\n | Single-turn bits: %u                                                          |", pos_bits);
        }
        DebugP_log("\r\n |-------------------------------------------------------------------------------|");
    }

    /* Enter interactive diagnostic loop - process user menu selections */
    while(1)
    {
        hdsl_display_menu();
        menu = hdsl_get_menu();

        if(attrs->load_share_enabled)
        {
            for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1 << i))
                {
                    DebugP_log( "|\r\n Channel %u ", i);
                    hdsl_process_request(gAppHdslHandle[CONFIG_HDSL0][i], menu);
                }
            }
        }
        else
        {
            hdsl_process_request(gAppHdslHandle[CONFIG_HDSL0][0], menu);
        }
    }

    /* Close HDSL handles before exiting */

    if(attrs->load_share_enabled)
    {
        for(i = 0; i < HDSL_NUM_CH_PER_SLICE_MAX; i++)
        {
            if(attrs->channel_mask & (1 << i))
            {
                HDSL_close(gAppHdslHandle[CONFIG_HDSL0][i]);
            }
        }
    }
    else
    {
        HDSL_close(gAppHdslHandle[CONFIG_HDSL0][0]);
    }

    Board_driversClose();
    Drivers_close();
}
