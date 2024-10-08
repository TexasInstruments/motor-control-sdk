/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <drivers/udma.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include "hdsl_diagnostic.h"
#include <position_sense/hdsl/include/hdsl_drv.h>
#include <position_sense/hdsl/include/pruss_intc_mapping.h>

/* PRU clock frequency =225Mhz   */
#define PRU_CLK_FREQ_225M 225000000
/*  PRU clock frequency =300Mhz   */
#define PRU_CLK_FREQ_300M 300000000

#if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_225M)
#include <position_sense/hdsl/firmware/hdsl_master_icssg_freerun_225_mhz_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_sync_225_mhz_bin.h>
/* Divide factor for normal clock (default value for 225 MHz=23) */
#define DIV_FACTOR_NORMAL 23
/* Divide factor for oversampled clock (default value for 225 MHz=2) */
#define DIV_FACTOR_OVERSAMPLED 2
#endif

#if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_300M)
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch0_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch1_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch2_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch0_sync_mode_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch1_sync_mode_bin.h>
#include <position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch2_sync_mode_bin.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if (CONFIG_HDSL0_CHANNEL0 + CONFIG_HDSL0_CHANNEL1  + CONFIG_HDSL0_CHANNEL2 > 1)
#define HDSL_MULTI_CHANNEL
#endif

#define TXPRU_IRAM_SIZE             (6*1024) /*6 kB*/
#define SYNC_PULSE_WAIT_CLK_CYCLES  5505

/* Divide factor for normal clock (default value for 300 MHz=31) */
#define DIV_FACTOR_NORMAL 31
/* Divide factor for oversampled clock (default value for 300 MHz=3) */
#define DIV_FACTOR_OVERSAMPLED 3

#ifdef HDSL_AM64xE1_TRANSCEIVER
#include <board/ioexp/ioexp_tca6424.h>
#endif

#define PRUICSS_PRUx  PRUICSS_PRU1
/* Oversample rate 8*/
#define OVERSAMPLE_RATE 7

/* max cycle time for transmission of dsl frame*/
#define MAX_SYNC_CYCLE_TIME 27

/* min cycle time for transmission of dsl frame*/
#define MIN_SYNC_CYCLE_TIME 12

#define HDSL_EN (0x1 << 26)
/* OCP as clock, div 32 */
#define HDSL_TX_CFG (0x10 | (DIV_FACTOR_NORMAL << 16))
/* OCP as clock, div 4, 8x OSR */
#define HDSL_RX_CFG (0x10 | (DIV_FACTOR_OVERSAMPLED << 16) | OVERSAMPLE_RATE | 0x08)
#define CTR_EN (1 << 3)
#define MAX_WAIT 20000

/*Timeout in micro-seconds for short message read/write*/
#define SHORT_MSG_TIMEOUT (1000)

/*Timeout in micro-seconds for long message read/write*/
#define LONG_MSG_TIMEOUT (200000)

/*Register Addresses for Short Messages (Parameter Channel)*/

#define ENCODER_STATUS0_REG_ADDRESS (0x40)
#define ENCODER_RSSI_REG_ADDRESS    (0x7C)
#define ENCODER_PING_REG_ADDRESS    (0x7F)

/*Bit 7 will be set in QM when link is establised*/
#define QM_LINK_ESTABLISHED                 (0x80)
#define QM_LINK_ESTABLISHED_AND_VALUE_15    (0x8F)


#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
/* Memory Trace is triggered for each H-Frame. SYS_EVENT_21 is triggered for each
   H-Frame from PRU. SYS_EVENT_21 is mapped to PRU_ICSSG0_PR1_HOST_INTR_PEND_3 of R5F
   in INTC Mapping. */

/*Event number for SYS_EVENT_21 (pr1_pru_mst_intr<5>_int_req) */
#define HDSL_MEMORY_TRACE_ICSS_INTC_EVENT_NUM (21U)

/* R5F Interrupt number for Memory Traces */
#define HDSL_MEMORY_TRACE_R5F_IRQ_NUM  (CSLR_R5FSS0_CORE0_INTR_PRU_ICSSG0_PR1_HOST_INTR_PEND_3)
#endif

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

void sync_calculation(HDSL_Handle hdslHandle);

void process_request(HDSL_Handle hdslHandle,int32_t menu);

void hdsl_pruss_init(void);

void hdsl_pruss_init_300m(void);

void hdsl_pruss_load_run_fw_300m(HDSL_Handle hdslHandle);

void hdsl_init(void);

void hdsl_init_300m(void);

void TC_read_pc_short_msg(HDSL_Handle hdslHandle);

void TC_write_pc_short_msg(HDSL_Handle hdslHandle);

static void display_menu(void);

void direct_read_rid0_length4(HDSL_Handle hdslHandle);

void direct_read_rid81_length8(HDSL_Handle hdslHandle);

void direct_read_rid81_length2(HDSL_Handle hdslHandle);

void indirect_write_rid0_length8_offset0(HDSL_Handle hdslHandle);

void indirect_write_rid0_length8(HDSL_Handle hdslHandle);

static int get_menu(void);

uint32_t read_encoder_resolution(HDSL_Handle hdslHandle);

#ifdef HDSL_AM64xE1_TRANSCEIVER
static void hdsl_i2c_io_expander(void *args);
#endif

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);

void udma_copy(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length);

static void HDSL_IsrFxn();

void traces_into_memory(HDSL_Handle hdslHandle);
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

HDSL_Handle gHdslHandleCh[HDSL_MAX_CHANNELS];

PRUICSS_Handle  gPruIcss0Handle;
PRUICSS_IntcInitData gPruss0_intc_initdata = PRU_ICSS0_INTC_INITDATA;
PRUICSS_Handle gPruIcss1Handle;
PRUICSS_IntcInitData gPruss1_intc_initdata = PRU_ICSS1_INTC_INITDATA;

HDSL_CopyTable *copyTable;

static char gUart_buffer[256];
static void *gPru_cfg;
void *gPru_dramx;
void *gPru_dramx_0;
void *gPru_dramx_1;
void *gPru_dramx_2;
int32_t get_pos=1;

uint32_t gMulti_turn, gRes;
uint64_t gMask;
uint8_t gPc_data;
uint8_t gPc_addrh, gPc_addrl, gPc_offh, gPc_offl, gPc_buf0, gPc_buf1, gPc_buf2, gPc_buf3, gPc_buf4, gPc_buf5, gPc_buf6, gPc_buf7;

#ifdef HDSL_AM64xE1_TRANSCEIVER
static TCA6424_Config  gTCA6424_Config;
#endif

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)

Udma_ChHandle   chHandle;

/* To store user input to start memory copy*/
volatile uint8_t start_copy;

/* To store user input number of count of memory copy*/
uint16_t trace_count;

/* To save log h-frame count during memory copy*/
uint32_t h_frame_count_arr[NUM_RESOURCES];

/* Location for copying HDSL Interface structure */
HDSL_Interface gHdslInterfaceTrace[NUM_RESOURCES] __attribute__((aligned(128), section(".hdslInterface_mem")));

HwiP_Object gPRUHwiObject;

/* UDMA TRPD Memory */
uint8_t gUdmaTestTrpdMem[UDMA_TEST_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)

static void App_udmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *trpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length)
{
    CSL_UdmapTR15  *pTr;
    uint32_t        cqRingNum = Udma_chGetCqRingNum(chHandle);
    static          uint32_t initDone = 0;

    if(initDone == 0)
    {
        /* Make TRPD with TR15 TR type */
        UdmaUtils_makeTrpdTr15(trpdMem, 1U, cqRingNum);

        /* Setup TR */
        pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
        pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOL, CSL_UDMAP_TR_FLAGS_EOL_MATCH_SOL_EOL);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);  /* This will come back in TR response */
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
        pTr->flags   |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
        pTr->icnt0    = length;
        pTr->icnt1    = 1U;
        pTr->icnt2    = 1U;
        pTr->icnt3    = 1U;
        pTr->dim1     = pTr->icnt0;
        pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
        pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
        pTr->addr     = (uint64_t) Udma_defaultVirtToPhyFxn(srcBuf, 0U, NULL);
        pTr->fmtflags = 0x00000000U;    /* Linear addressing, 1 byte per elem */
        pTr->dicnt0   = length;
        pTr->dicnt1   = 1U;
        pTr->dicnt2   = 1U;
        pTr->dicnt3   = 1U;
        pTr->ddim1    = pTr->dicnt0;
        pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
        pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
        pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);
        /* Perform cache writeback */
        CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);

        initDone = 1;
    }
    else
    {
        pTr = UdmaUtils_getTrpdTr15Pointer(trpdMem, 0U);
        pTr->daddr    = (uint64_t) Udma_defaultVirtToPhyFxn(destBuf, 0U, NULL);
        /* Perform cache writeback */
        CacheP_wb(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
    }


    return;
}

void udma_copy(uint8_t *srcBuf, uint8_t *destBuf, uint32_t length)
{
    int32_t         retVal = UDMA_SOK;
    uint64_t        pDesc;
    uint32_t        trRespStatus;
    uint8_t        *trpdMem = &gUdmaTestTrpdMem[0U];
    uint64_t        trpdMemPhy = (uint64_t) Udma_defaultVirtToPhyFxn(trpdMem, 0U, NULL);

    /* Init TR packet descriptor */
    App_udmaTrpdInit(chHandle, trpdMem, destBuf, srcBuf, length);

    /* Submit TRPD to channel */
    retVal = Udma_ringQueueRaw(Udma_chGetFqRingHandle(chHandle), trpdMemPhy);
    DebugP_assert(UDMA_SOK == retVal);

    /* Wait for return descriptor in completion ring - this marks transfer completion */
    while(1)
    {
        retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
        if(UDMA_SOK == retVal)
        {
            /* Check TR response status */
            CacheP_inv(trpdMem, UDMA_TEST_TRPD_SIZE, CacheP_TYPE_ALLD);
            trRespStatus = UdmaUtils_getTrpdTr15Response(trpdMem, 1U, 0U);
            DebugP_assert(CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE == trRespStatus);
            break;
        }
    }

    /* Validate data in destination memory */
    CacheP_inv(destBuf, length, CacheP_TYPE_ALLD);
}

static void HDSL_IsrFxn()
{
    static uint64_t h_frames_count = 0;
    static uint32_t temp = 0;
    uint8_t         *srcBuf;
    uint8_t         *destBuf;
    uint32_t        length;

    srcBuf = (uint8_t*)HDSL_get_src_loc(gHdslHandleCh[0]);
    length = HDSL_get_length(gHdslHandleCh[0]);
    PRUICSS_clearEvent(gPruIcss0Handle, HDSL_MEMORY_TRACE_ICSS_INTC_EVENT_NUM);

    /* No of h-frames count */
    h_frames_count++;

    if((start_copy == 1) && (temp < trace_count))
    {

        /* Init buffers and TR packet descriptor */
        destBuf = (uint8_t*)&gHdslInterfaceTrace[temp];

        /* start UDMA copying data from src to dest */
        udma_copy(srcBuf,destBuf,length);

        h_frame_count_arr[temp] = h_frames_count;
        temp++;
    }
    else
    {
        start_copy = 0;
        temp = 0;
    }
}

void traces_into_memory(HDSL_Handle hdslHandle)
{
    int32_t i= 0;
    uint32_t length;
    length = HDSL_get_length(hdslHandle);

    DebugP_log("\r\n sizeof(hdslInterface)_count = %u", length);
    DebugP_log("\r\n Start address of memory trace location = %x", &gHdslInterfaceTrace[0]);
    DebugP_log("\r\n End address of memory trace location = %x", &gHdslInterfaceTrace[NUM_RESOURCES-1]);
    DebugP_log("\r\n No of HDSL-Interface-Register-Structure to copy = %u", trace_count);

    start_copy = 1;

    while(start_copy)
    {
        ClockP_sleep(1);
    }

    for(i=0; i< trace_count; i++)
    {
        DebugP_log("\r\n %u h_frame_count = %u ",i, h_frame_count_arr[i]);
    }
}

#endif

void sync_calculation(HDSL_Handle hdslHandle)
{
    uint8_t ES;
    uint16_t wait_before_start;
    uint32_t counter, period, index;
    volatile uint32_t cap6_rise0, cap6_rise1, cap6_fall0, cap6_fall1;
    uint8_t EXTRA_EDGE_ARR[8] = {0x00 ,0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};
    #if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_225M)
        uint32_t minm_bits = 112, cycle_per_bit = 24, max_stuffing = 26, stuffing_size = 6, cycle_per_overclock_bit =3, minm_extra_size = 4, sync_param_mem_start = 0xDC;
        uint32_t cycles_left, additional_bits, minm_cycles, time_gRest, extra_edge, extra_size, num_of_stuffing, extra_size_remainder, stuffing_remainder, bottom_up_cycles;
    #endif
    #if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_300M)
        uint32_t minm_bits = 112, cycle_per_bit = 32, max_stuffing = 26, stuffing_size = 6, cycle_per_overclock_bit =4, minm_extra_size = 4, sync_param_mem_start = 0xDC;
        uint32_t cycles_left, additional_bits, minm_cycles, time_gRest, extra_edge, extra_size, num_of_stuffing, extra_size_remainder, stuffing_remainder, bottom_up_cycles;
    #endif
    /*measure of SYNC period starts*/

    ES=HDSL_get_sync_ctrl(hdslHandle);
    uint32_t CAPR6_REG0 = CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CAPR6_REG0;
    uint32_t CAPF6_REG0 = CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_IEP1_SLV_REGS_BASE + CSL_ICSS_G_PR1_IEP0_SLV_CAPF6_REG0;
    cap6_rise0 = HW_RD_REG32(CAPR6_REG0);
    cap6_fall0 = HW_RD_REG32(CAPF6_REG0);
    cap6_rise1 = cap6_rise0;
    cap6_fall1 = cap6_fall0;
    counter = 0;
    for(index = 0 ; index <2 ; index++)
    {
        cap6_rise0 = cap6_rise1;
        cap6_fall0 = cap6_fall1;
        while(((cap6_fall0 == cap6_fall1) || (cap6_rise0 == cap6_rise1)) && (counter <= MAX_WAIT))
        {
            cap6_rise1 = HW_RD_REG32(CAPR6_REG0);
            cap6_fall1 = HW_RD_REG32(CAPF6_REG0);
            counter++;
        }
    }
    DebugP_assert(counter <= MAX_WAIT);
    period = cap6_rise1 - cap6_rise0;
    /*measure of SYNC period ends*/
    minm_cycles = minm_bits * ES * cycle_per_bit;
    cycles_left = period - minm_cycles;
    time_gRest = (cycles_left % cycle_per_bit) / cycle_per_overclock_bit;
    additional_bits = cycles_left / cycle_per_bit;
    extra_edge = EXTRA_EDGE_ARR[time_gRest];
    num_of_stuffing = additional_bits / stuffing_size;
    extra_size = additional_bits % stuffing_size;
    extra_size = extra_size + minm_extra_size * ES;
    if(num_of_stuffing > ES * max_stuffing)
    {
        extra_size = extra_size + (((num_of_stuffing) - (max_stuffing * ES)) * stuffing_size);
        num_of_stuffing = ES * max_stuffing;
    }
    extra_size_remainder = extra_size % ES;
    extra_size = extra_size / ES;
    stuffing_remainder = num_of_stuffing % ES;
    num_of_stuffing = num_of_stuffing / ES;
    bottom_up_cycles = (minm_cycles - minm_extra_size * ES * cycle_per_bit);
    bottom_up_cycles = bottom_up_cycles + (stuffing_size * (ES * num_of_stuffing + stuffing_remainder))*cycle_per_bit;
    bottom_up_cycles = bottom_up_cycles + ((ES * extra_size  + extra_size_remainder) * cycle_per_bit ) + time_gRest * cycle_per_overclock_bit;
    wait_before_start = (84 * cycle_per_bit)+((8 - time_gRest)*cycle_per_overclock_bit)+(num_of_stuffing * stuffing_size * cycle_per_bit);
    if(stuffing_remainder != 0)
    {
        wait_before_start = wait_before_start+(stuffing_size * cycle_per_bit);
    }
    wait_before_start=wait_before_start+SYNC_PULSE_WAIT_CLK_CYCLES;
    if(extra_size < 4 || extra_size > 9)
    {
        DebugP_log("\r\n ERROR: ES or period selected is Invalid ");
    }
    DebugP_log("\r\n ********************************************************************");
    DebugP_log("\r\n SYNC MODE: period = %d", period);
    DebugP_log("\r\n SYNC MODE: ES = %d", ES);
    DebugP_log("\r\n SYNC MODE: counter = %d", counter);
    DebugP_log("\r\n SYNC MODE: wait_before_start = %d", wait_before_start);
    DebugP_log("\r\n SYNC MODE: bottom_up_cycles = %d", bottom_up_cycles);
    DebugP_log("\r\n SYNC MODE: extra_size = %d", extra_size);
    DebugP_log("\r\n SYNC MODE: temp_gRest = %d", time_gRest);
    DebugP_log("\r\n SYNC MODE: extra_edge = %d", extra_edge);
    DebugP_log("\r\n SYNC MODE: num_of_stuffing = %d", num_of_stuffing);
    DebugP_log("\r\n SYNC MODE: extra_size_remainder = %d", extra_size_remainder);
    DebugP_log("\r\n SYNC MODE: stuffing_remainder = %d", stuffing_remainder);
    DebugP_log("\r\n ********************************************************************");
    #if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_225M)
        sync_param_mem_start =sync_param_mem_start + (uint32_t)gPru_dramx;
    #endif
    #if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_300M)
        sync_param_mem_start =sync_param_mem_start + (uint32_t)hdslHandle->baseMemAddr;
    #endif
    HWREGB(sync_param_mem_start) = extra_size;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = num_of_stuffing;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = extra_edge;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = time_gRest;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = extra_size_remainder;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGB(sync_param_mem_start) = stuffing_remainder;
    sync_param_mem_start = sync_param_mem_start + 1;
    HWREGH(sync_param_mem_start) = wait_before_start;

}

void process_request(HDSL_Handle hdslHandle,int32_t menu)
{
    uint64_t ret_status=0;
    uint64_t val0, val1, val2;
    uint8_t ureg, ureg1;
    float pos0, pos1, pos2;
    uint64_t turn0, turn1, turn2;

    switch(menu)
    {
        case MENU_SAFE_POSITION:

            val0 = HDSL_get_pos(hdslHandle,0);
            if(val0 != -1)
            {

                DebugP_log("\r\n Fast Position read successfully");
            }

            val1 = HDSL_get_pos(hdslHandle,1);
            if(val1 != -1)
            {
                DebugP_log("\r\n Safe Position 1 read successfully");
            }

            val2 = HDSL_get_pos(hdslHandle,2);
            if(val2 != -1)
            {
                DebugP_log("\r\n Safe Position 2 read successfully");
            }

            pos0 = (float)(val0 & hdslHandle->mask) / (float)(hdslHandle->mask + 1) * (float)360;
            pos1 = (float)(val1 & hdslHandle->mask) / (float)(hdslHandle->mask + 1) * (float)360;
            pos2 = (float)(val2 & hdslHandle->mask) / (float)(hdslHandle->mask + 1) * (float)360;

            ureg = HDSL_get_rssi(hdslHandle);
            ureg1 = HDSL_get_qm(hdslHandle);

            if (hdslHandle->multi_turn)
            {

                turn0 = val0 & ~hdslHandle->mask;
                turn0 >>= hdslHandle->res;

                turn1 = val1 & ~hdslHandle->mask;
                turn1 >>= hdslHandle->res;

                turn2 = val2 & ~hdslHandle->mask;
                turn2 >>= hdslHandle->res;
                DebugP_log("\r\n Angle: %10.6f\tTurn: %llu\t", pos0, turn0);
                DebugP_log("\r\n SafePos1: %10.6f\tTurn: %llu", pos1, turn1);
                DebugP_log("\r\n SafePos2: %10.6f\tTurn: %llu", pos2, turn2);
                DebugP_log("\r\n RSSI: %u\t QM:  %u", ureg, ureg1 );

            }
            else
            {
                DebugP_log("\r\n Angle: %10.6f", pos0);
            }
            break;
        case MENU_QUALITY_MONITORING:
            ret_status= HDSL_get_qm(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n Quality monitoring value: %u", ret_status);
            }
            break;
        case MENU_EVENTS:
            ret_status=HDSL_get_events(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n EVENT_H and EVENT_L: 0x%x", ret_status);
            }
            ret_status=HDSL_get_safe_events(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n EVENT_S: 0x%x", ret_status);
            }
            ret_status=HDSL_get_online_status_d(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n ONLINE_STATUS_D: 0x%x", ret_status);
            }
            ret_status=HDSL_get_online_status_1(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n ONLINE_STATUS_1: 0x%x", ret_status);
            }
            ret_status=HDSL_get_online_status_2(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n ONLINE_STATUS_2: 0x%x", ret_status);
            }
            break;
        case MENU_SUMMARY:
            ret_status= HDSL_get_sum(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n Summarized slave status: 0x%x", ret_status);
            }
            break;
        case MENU_ACC_ERR_CNT:
            ret_status= HDSL_get_acc_err_cnt(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n Acceleration error counter: %u", ret_status);
            }
            break;
        case MENU_RSSI:
            ret_status=HDSL_get_rssi(hdslHandle);
            if(ret_status!=-1)
            {
                DebugP_log("\r\n RSSI: %u", ret_status);
            }
            break;
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
        case MENU_HDSL_REG_INTO_MEMORY:
            traces_into_memory(hdslHandle);
            break;
#endif
        case MENU_PC_SHORT_MSG_WRITE:
            TC_write_pc_short_msg(hdslHandle);
            break;
        case MENU_PC_SHORT_MSG_READ:
            TC_read_pc_short_msg(hdslHandle);
            break;
        case MENU_DIRECT_READ_RID0_LENGTH4:
            direct_read_rid0_length4(hdslHandle);
            break;
        case MENU_DIRECT_READ_RID81_LENGTH8:
            direct_read_rid81_length8(hdslHandle);
            break;
        case MENU_DIRECT_READ_RID81_LENGTH2:
            direct_read_rid81_length2(hdslHandle);
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8:
            indirect_write_rid0_length8(hdslHandle);
            break;
        case MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0:
            indirect_write_rid0_length8_offset0(hdslHandle);
            break;

        default:
            DebugP_log( "\r\n ERROR: invalid request");
            break;
    }
}

void hdsl_pruss_init(void)
{
    PRUICSS_disableCore(gPruIcss0Handle, gHdslHandleCh[0]->icssCore);

    /* clear ICSS0 PRU data RAM */
    gPru_dramx = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_PRUx));
    memset(gPru_dramx, 0, (4 * 1024));

    gPru_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->cfgRegBase);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_GPCFG1, HDSL_EN);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, HDSL_TX_CFG);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, HDSL_RX_CFG);

    PRUICSS_intcInit(gPruIcss0Handle, &gPruss0_intc_initdata);

    PRUICSS_intcInit(gPruIcss1Handle, &gPruss1_intc_initdata);

    /* configure C28 to PRU_ICSS_CTRL and C29 to EDMA + 0x1000 */
    /*6.4.14.1.1 ICSSG_PRU_CONTROL RegisterPRU_ICSSG0_PR1_PDSP0_IRAM 00B0 2400h*/
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x0240);
    /*IEP1 base */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C29, 0x0002F000);

    /* enable cycle counter */
    HW_WR_REG32((void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE), CTR_EN);
}

void hdsl_pruss_init_300m(void)
{
#if (CONFIG_HDSL0_CHANNEL0 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_RTU_PRU1);    // ch0
#endif

#if (CONFIG_HDSL0_CHANNEL1 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);        // ch1
#endif

#if (CONFIG_HDSL0_CHANNEL2 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_TX_PRU1);        // ch2
#endif
    /* Clear PRU_DRAM0 and PRU_DRAM1 memory */

    gPru_dramx_0 = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_RTU_PRU1));
    gPru_dramx_1 = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_PRU1));
    gPru_dramx_2 = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_TX_PRU1));
    memset(gPru_dramx_0, 0, (4 * 1024));
    memset(gPru_dramx_1, 0, (4 * 1024));
    memset(gPru_dramx_2, 0, (4 * 1024));
    memset((void *) CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE, 0, (16 * 1024));
    memset((void *) CSL_PRU_ICSSG0_DRAM1_SLV_RAM_BASE, 0, (16 * 1024));

    gPru_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->cfgRegBase);

    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_GPCFG1, HDSL_EN);
    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, HDSL_TX_CFG);
    HW_WR_REG32(gPru_cfg + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, HDSL_RX_CFG);
    PRUICSS_intcInit(gPruIcss0Handle, &gPruss0_intc_initdata);

    /* configure C28 to PRU_ICSS_CTRL and C29 to EDMA + 0x1000 */
    /*6.4.14.1.1 ICSSG_PRU_CONTROL RegisterPRU_ICSSG0_PR1_PDSP0_IRAM 00B0 2400h*/
    HWREG(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0) = 0xF0000238; // Address = 0x30023828
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C28, 0x0240);
    HWREG(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0) = 0xF0000258; // Address = 0x30025828
    /*IEP1 base */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C29, 0x0002F000);

    HWREG(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0) = 0x0000; // RTU Core
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRU1, PRUICSS_CONST_TBL_ENTRY_C24, 0x0007);        // PRU Core
    HWREG(CSL_PRU_ICSSG0_DRAM0_SLV_RAM_BASE + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_REGS_BASE + CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0) = 0x000E; // TX_PRU Core

    /* enable cycle counter */
    HW_WR_REG32((void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_REGS_BASE), CTR_EN);   // RTU_PRU Core
    HW_WR_REG32((void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE), CTR_EN);
    HW_WR_REG32((void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_REGS_BASE), CTR_EN);        // TX_PRU Core

}

void hdsl_pruss_load_run_fw(HDSL_Handle hdslHandle)
{
#if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_225M)
        PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
    if(HDSL_get_sync_ctrl(hdslHandle) == 0)
    {
        /*free run*/
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                            0, (uint32_t *) Hiperface_DSL2_0_RTU_0,
                            sizeof(Hiperface_DSL2_0_RTU_0));
    }
    else
    {
        /*sync_mode*/
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                        0, (uint32_t *) Hiperface_DSL_SYNC2_0_RTU_0,
                        sizeof(Hiperface_DSL_SYNC2_0_RTU_0));
    }
        PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
        /*Run firmware*/
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
#endif
}

void hdsl_pruss_load_run_fw_300m(HDSL_Handle hdslHandle)
{
#if (CONFIG_HDSL0_CHANNEL2 == 1)
    uint32_t txpruFwSize = 0;
#endif
#if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_300M)
#if (CONFIG_HDSL0_CHANNEL0 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_RTU_PRU1);    // ch0
#endif

#if (CONFIG_HDSL0_CHANNEL1 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRU1);        // ch1
#endif

#if (CONFIG_HDSL0_CHANNEL2 == 1)
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_TX_PRU1);        // ch2
#endif

    /* Enable Load Share mode */
    gPru_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->cfgRegBase);
    hdsl_enable_load_share_mode(gPru_cfg,PRUICSS_PRUx);

    if(HDSL_get_sync_ctrl(hdslHandle) == 0)
    {
        /*free run*/
#if (CONFIG_HDSL0_CHANNEL0 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_RTU_PRU(1),
                        0, (uint32_t *) Hiperface_DSL2_0_RTU_0,
                        sizeof(Hiperface_DSL2_0_RTU_0));
#endif
#if (CONFIG_HDSL0_CHANNEL1 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(1),
                        0, (uint32_t *) Hiperface_DSL2_0_PRU_0,
                        sizeof(Hiperface_DSL2_0_PRU_0));
#endif
#if (CONFIG_HDSL0_CHANNEL2 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1),
                        0, (uint32_t *) Hiperface_DSL2_0_TX_PRU_0,
                        sizeof(Hiperface_DSL2_0_TX_PRU_0));

            /*
            NOTE: As this array is typecasted into a structure with 32-bit variables,
            32b alignment is required. This is done using linker.
            */
            copyTable = (HDSL_CopyTable *)&Hiperface_DSL2_0_TX_PRU_2;
            txpruFwSize = (copyTable->size1 > copyTable->size2)?(sizeof(Hiperface_DSL2_0_TX_PRU_0) + copyTable->size1):(sizeof(Hiperface_DSL2_0_TX_PRU_0) + copyTable->size2);
            DebugP_assert(txpruFwSize <= TXPRU_IRAM_SIZE);

            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_DATARAM(1), 0x1500, (uint32_t *)Hiperface_DSL2_0_TX_PRU_1, copyTable->size1 + copyTable->size2);
            if(copyTable->loadAddr1 < copyTable->loadAddr2)
            {
                PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1), copyTable->runAddr1, (uint32_t *) ((uint8_t *)Hiperface_DSL2_0_TX_PRU_1), copyTable->size1);
            }
            else
            {
                PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1), copyTable->runAddr1, (uint32_t *) ((uint8_t *)Hiperface_DSL2_0_TX_PRU_1 + copyTable->size2), copyTable->size1);
            }

            HDSL_config_copy_table(hdslHandle, copyTable);
#endif
    }
    else
    {
        /*Sync mode*/
#if (CONFIG_HDSL0_CHANNEL0 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_RTU_PRU(1),
                        0, (uint32_t *) Hiperface_DSL_SYNC2_0_RTU_0,
                        sizeof(Hiperface_DSL_SYNC2_0_RTU_0));
#endif
#if (CONFIG_HDSL0_CHANNEL1 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(1),
                        0, (uint32_t *) Hiperface_DSL_SYNC2_0_PRU_0,
                        sizeof(Hiperface_DSL_SYNC2_0_PRU_0));
#endif
#if (CONFIG_HDSL0_CHANNEL2 == 1)
            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1),
                       0, (uint32_t *) Hiperface_DSL_SYNC2_0_TX_PRU_0,
                       sizeof(Hiperface_DSL_SYNC2_0_TX_PRU_0));

            /*
            NOTE: As this array is typecasted into a structure with 32-bit variables,
            32b alignment is required. This is done using linker.
            */
            copyTable = (HDSL_CopyTable *)&Hiperface_DSL_SYNC2_0_TX_PRU_2;
            txpruFwSize = (copyTable->size1 > copyTable->size2)?(sizeof(Hiperface_DSL_SYNC2_0_TX_PRU_0) + copyTable->size1):(sizeof(Hiperface_DSL_SYNC2_0_TX_PRU_0) + copyTable->size2);
            DebugP_assert(txpruFwSize <= TXPRU_IRAM_SIZE);

            PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_DATARAM(1), 0x1500, (uint32_t *)Hiperface_DSL_SYNC2_0_TX_PRU_1, copyTable->size1 + copyTable->size2);
            if(copyTable->loadAddr1 < copyTable->loadAddr2)
            {
                PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1), copyTable->runAddr1, (uint32_t *) ((uint8_t *)Hiperface_DSL_SYNC2_0_TX_PRU_1), copyTable->size1);
            }
            else
            {
                PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_TX_PRU(1), copyTable->runAddr1, (uint32_t *) ((uint8_t *)Hiperface_DSL_SYNC2_0_TX_PRU_1 + copyTable->size2), copyTable->size1);
            }

            HDSL_config_copy_table(hdslHandle, copyTable);
#endif
    }




#if (CONFIG_HDSL0_CHANNEL0 == 1)
        PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_RTU_PRU1);
#endif
#if (CONFIG_HDSL0_CHANNEL1 == 1)
        PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRU1);
#endif
#if (CONFIG_HDSL0_CHANNEL2 == 1)
        PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_TX_PRU1);
#endif
        /*Run firmware*/
#if (CONFIG_HDSL0_CHANNEL0 == 1)
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_RTU_PRU1);
#endif
#if (CONFIG_HDSL0_CHANNEL1 == 1)
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRU1);
#endif
#if (CONFIG_HDSL0_CHANNEL2 == 1)
        PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_TX_PRU1);
#endif
#endif
}

void hdsl_init(void)
{
    uint8_t         ES;
    uint32_t        period;
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    HwiP_Params     hwiPrms;
    uint32_t        intrNum = HDSL_MEMORY_TRACE_R5F_IRQ_NUM;
#endif

    hdsl_pruss_init();

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    /* Register PRU interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = (void*)&HDSL_IsrFxn;
    HwiP_construct(&gPRUHwiObject, &hwiPrms);
#endif

    HDSL_iep_init(gHdslHandleCh[0]);
    ClockP_usleep(5000);
    if(CONFIG_HDSL0_MODE==0)
    {
        ES=0;
    }
    else
    {
        ES=1;
    }
    HDSL_set_sync_ctrl(gHdslHandleCh[0], ES);
    if(ES != 0)
    {
        DebugP_log("\r\nSYNC MODE\n");
        DebugP_log("\r\nEnter period for SYNC PULSE in unit of cycles(1 cycle = 4.44ns):");
        DebugP_scanf("%d",&period);
        HDSL_enable_sync_signal(ES,period);
        HDSL_generate_memory_image(gHdslHandleCh[0]);
        sync_calculation(gHdslHandleCh[0]);
    }
    else
    {
        DebugP_log( "\r\nFREE RUN MODE\n");
        ClockP_sleep(5);
        HDSL_generate_memory_image(gHdslHandleCh[0]);
    }
}

void hdsl_init_300m(void)
{
    uint8_t         ES = 0;
    uint32_t        period;
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    HwiP_Params     hwiPrms;
    uint32_t        intrNum = HDSL_MEMORY_TRACE_R5F_IRQ_NUM;
#endif

    hdsl_pruss_init_300m();

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    /* Register PRU interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = (void*)&HDSL_IsrFxn;
    HwiP_construct(&gPRUHwiObject, &hwiPrms);
#endif

    if(CONFIG_HDSL0_CHANNEL0==1)
    {
        HDSL_iep_init(gHdslHandleCh[0]);
    }
    else if(CONFIG_HDSL0_CHANNEL1==1)
    {
        HDSL_iep_init(gHdslHandleCh[1]);
    }
    else if(CONFIG_HDSL0_CHANNEL2==1)
    {
        HDSL_iep_init(gHdslHandleCh[2]);
    }

    ClockP_usleep(5000);

    if(CONFIG_HDSL0_MODE == 1)
    {
        DebugP_log("\r\nSYNC MODE\n");
        DebugP_log("\r\nEnter ES and period for SYNC PULSE in unit of cycles(1 cycle = 3.33ns):\r\n");
        DebugP_scanf("%d",&ES);
        DebugP_scanf("%d",&period);

        if(CONFIG_HDSL0_CHANNEL0==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[0], ES);
        }
        if(CONFIG_HDSL0_CHANNEL1==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[1], ES);
        }
        if(CONFIG_HDSL0_CHANNEL2==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[2], ES);
        }

        /*  Check Sync period condition

        (Tsync= Cycle time for input SYNC pulse signal ,
        Tmin=MIN_SYNC_CYCLE_TIME, Tmax=MAX_SYNC_CYCLE_TIME
        Tsync=period/(PRU clock freq)  =  period/300)

        ES <= Tsync/Tmin  and  ES >= Tsync/Tmax     */

        if ((ES > (period/(MIN_SYNC_CYCLE_TIME*300))) || (ES < (period/(MAX_SYNC_CYCLE_TIME*300))))
        {
            DebugP_log("\r\n FAIL: HDSL ES or period value you entered is not valid");
            while(1)
                ;
        }
        HDSL_enable_sync_signal(ES,period);
        if (CONFIG_HDSL0_CHANNEL0==1)
        {

            HDSL_generate_memory_image(gHdslHandleCh[0]);
            sync_calculation(gHdslHandleCh[0]);
        }
        if (CONFIG_HDSL0_CHANNEL1==1)
        {

            HDSL_generate_memory_image(gHdslHandleCh[1]);
            sync_calculation(gHdslHandleCh[1]);
        }
        if (CONFIG_HDSL0_CHANNEL2==1)
        {

            HDSL_generate_memory_image(gHdslHandleCh[2]);
            sync_calculation(gHdslHandleCh[2]);
        }
    }
    else
    {
        DebugP_log( "\r\nFREE RUN MODE\n");

        if(CONFIG_HDSL0_CHANNEL0==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[0], ES);
            HDSL_generate_memory_image(gHdslHandleCh[0]);
        }
        if(CONFIG_HDSL0_CHANNEL1==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[1], ES);
            HDSL_generate_memory_image(gHdslHandleCh[1]);
        }
        if(CONFIG_HDSL0_CHANNEL2==1)
        {
            HDSL_set_sync_ctrl(gHdslHandleCh[2], ES);
            HDSL_generate_memory_image(gHdslHandleCh[2]);
        }
    }
}

void TC_read_pc_short_msg(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;
    status = HDSL_read_pc_short_msg(hdslHandle,ENCODER_RSSI_REG_ADDRESS, &gPc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }

    DebugP_log("\r\n Parameter channel short message read  : Slave RSSI (address 0x7C) = %x (should be 0x7 which indicates best signal strength)", gPc_data);

    status = HDSL_read_pc_short_msg(hdslHandle,ENCODER_STATUS0_REG_ADDRESS, &gPc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }

    DebugP_log("\r\n Parameter channel short message read  : Address 0x40 = %x (should be 0x1)", gPc_data);
}

void TC_write_pc_short_msg(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;

    DebugP_log("\r\n Parameter channel short message write : 0xab to PING register (address 0x7F)");

    status = HDSL_write_pc_short_msg(hdslHandle,ENCODER_PING_REG_ADDRESS, 0xab, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_short_msg() did not return success");
        return;
    }

    status = HDSL_read_pc_short_msg(hdslHandle,ENCODER_PING_REG_ADDRESS, &gPc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }
    DebugP_log("\r\n Parameter channel short message read  : PING register (address 0x7F) = %x (should be 0xab) ", gPc_data);


    DebugP_log("\r\n Parameter channel short message write : 0xcd to PING register (address 0x7F)");

    status = HDSL_write_pc_short_msg(hdslHandle,ENCODER_PING_REG_ADDRESS, 0xcd, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_write_pc_short_msg() did not return success");
        return;
    }

    status = HDSL_read_pc_short_msg(hdslHandle,ENCODER_PING_REG_ADDRESS, &gPc_data, SHORT_MSG_TIMEOUT);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_short_msg() did not return success");
        return;
    }
    DebugP_log("\r\n Parameter channel short message read  : PING register (address 0x7F) = %x (should be 0xcd) ", gPc_data);
}

static void display_menu(void)
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
#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    DebugP_log("\r\n | %2d : HDSL registers into Memory                                              |", MENU_HDSL_REG_INTO_MEMORY);
#endif
    DebugP_log("\r\n |------------------------------------------------------------------------------|\n");
    DebugP_log("\r\n Enter value: ");
}

void direct_read_rid0_length4(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;

    DebugP_log("\r\n Parameter channel long message read : RID 0, Length 4");

    /* Set the parameter channel buffers to 0xff */
    HDSL_write_pc_buffer(hdslHandle, 0, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 1, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 2, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 3, 0xff);

    status = HDSL_read_pc_long_msg(hdslHandle, 0, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_4, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);

    if(gPc_buf0 == 'R')
    {
        gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
        if(gPc_buf1 == 'O')
        {
            gPc_buf2 = HDSL_read_pc_buffer(hdslHandle, 2);
            if(gPc_buf2 == 'O')
            {
                gPc_buf3 = HDSL_read_pc_buffer(hdslHandle, 3);
                if(gPc_buf3 == 'T')
                {
                    DebugP_log("\r\n PASS : Read \"ROOT\"");
                }
                else
                {
                    DebugP_log("\r\n FAIL: PC_BUFFER3 != T (It is %u)", gPc_buf3);
                }
            }
            else
            {
                DebugP_log("\r\n FAIL: PC_BUFFER2 != O (It is %u)", gPc_buf2);
            }
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != O (It is %u)", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != R (It is %u)", gPc_buf0);
    }
}

void direct_read_rid81_length8(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;

    DebugP_log("\r\n Parameter channel long message read : RID 0x81, Length 8");

    /* Set the parameter channel buffers to 0xff */
    HDSL_write_pc_buffer(hdslHandle, 0, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 1, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 2, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 3, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 4, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 5, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 6, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 7, 0xff);

    status = HDSL_read_pc_long_msg(hdslHandle, 0x81, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);
    if(gPc_buf0 == 'R')
    {
        gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
        if(gPc_buf1 == 'E')
        {
            gPc_buf2 = HDSL_read_pc_buffer(hdslHandle, 2);
            if(gPc_buf2 == 'S')
            {
                gPc_buf3 = HDSL_read_pc_buffer(hdslHandle, 3);
                if(gPc_buf3 == 'O')
                {
                    gPc_buf4 = HDSL_read_pc_buffer(hdslHandle, 4);
                    if(gPc_buf4 == 'L')
                    {
                        gPc_buf5 = HDSL_read_pc_buffer(hdslHandle, 5);
                        if(gPc_buf5 == 'U')
                        {
                            gPc_buf6 = HDSL_read_pc_buffer(hdslHandle, 6);
                            if(gPc_buf6 == 'T')
                            {
                                gPc_buf7 = HDSL_read_pc_buffer(hdslHandle, 7);
                                if(gPc_buf7 == 'N')
                                {
                                    DebugP_log("\r\n PASS : Read \"RESOLUTN\"");
                                }
                                else
                                {
                                    DebugP_log("\r\n FAIL: PC_BUFFER7 != N (It is %u)", gPc_buf7);
                                }
                            }
                            else
                            {
                                DebugP_log("\r\n FAIL: PC_BUFFER6 != T (It is %u)", gPc_buf6);
                            }
                        }
                        else
                        {
                            DebugP_log("\r\n FAIL: PC_BUFFER5 != U (It is %u)", gPc_buf5);
                        }
                    }
                    else
                    {
                        DebugP_log("\r\n FAIL: PC_BUFFER4 != L (It is %u)", gPc_buf4);
                    }

                }
                else
                {
                    DebugP_log("\r\n FAIL: PC_BUFFER3 != O (It is %u)", gPc_buf3);
                }
            }
            else
            {
                DebugP_log("\r\n FAIL: PC_BUFFER2 != S (It is %u)", gPc_buf2);
            }
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != E (It is %u)", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != R (It is %u)", gPc_buf0);
    }
}

void direct_read_rid81_length2(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;

    DebugP_log("\r\n Parameter channel long message read : RID 0x81, Offset 3, Length 2");

    /* Set the parameter channel buffers to 0xaa */
    HDSL_write_pc_buffer(hdslHandle, 0, 0xaa);
    HDSL_write_pc_buffer(hdslHandle, 1, 0xaa);

    status = HDSL_read_pc_long_msg(hdslHandle, 0x81, HDSL_LONG_MSG_ADDR_WITH_OFFSET, HDSL_LONG_MSG_ADDR_DIRECT, HDSL_LONG_MSG_LENGTH_2, 3, LONG_MSG_TIMEOUT);

    if(SystemP_SUCCESS != status)
    {
        DebugP_log("\r\n FAIL: HDSL_read_pc_long_msg() did not return success");
        return;
    }

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);

    if(gPc_buf0 == 0x00)
    {
        gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
        if(gPc_buf1 == 0x0f)
        {
            DebugP_log("\r\n PASS : Read 15");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x0f (It is %u)", gPc_buf1);
        }

    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x00 (It is %u)", gPc_buf0);
    }
}

void indirect_write_rid0_length8_offset0(HDSL_Handle hdslHandle)
{
    DebugP_log("\r\n Parameter channel long message write : RID 0x0, Offset 0, Length 8");

    HDSL_write_pc_long_msg(hdslHandle, 0x0, HDSL_LONG_MSG_ADDR_WITH_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    /*FIXME: Add error check*/

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);
    if(gPc_buf0 == 0x41)
    {
        gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
        if(gPc_buf1 == 0x10)
        {
            DebugP_log("\r\n PASS ");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x10 (It is %u)", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x41 (It is %u)", gPc_buf0);
    }

}

void indirect_write_rid0_length8(HDSL_Handle hdslHandle)
{

    DebugP_log("\r\n Parameter channel long message write : RID 0x0, Length 8");

    HDSL_write_pc_long_msg(hdslHandle, 0x0, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_8, 0, LONG_MSG_TIMEOUT);

    /*FIXME: Add error check*/

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);
    if(gPc_buf0 == 0x41)
    {
        gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
        if(gPc_buf1 == 0x10)
        {
            DebugP_log("\r\n PASS ");
        }
        else
        {
            DebugP_log("\r\n FAIL: PC_BUFFER1 != 0x10 (It is %u)", gPc_buf1);
        }
    }
    else
    {
        DebugP_log("\r\n FAIL: PC_BUFFER0 != 0x41 (It is %u)", gPc_buf0);
    }
}

static int get_menu(void)
{
    uint32_t cmd;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    if(DebugP_scanf("%d\n", &cmd) < 0 || (cmd >= MENU_LIMIT))
#else
    if(DebugP_scanf("%d\n", &cmd) < 0 || (cmd >= MENU_LIMIT) || (cmd == MENU_HDSL_REG_INTO_MEMORY))
#endif
    {
        DebugP_log("\r\n WARNING: invalid option, Safe position selected");
        cmd = MENU_SAFE_POSITION;
    }

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)

    if (cmd == MENU_HDSL_REG_INTO_MEMORY)
    {
       DebugP_log("\r\n| How many traces you want to copy : ");
       if(DebugP_scanf("%u\n", &trace_count) < 0 || trace_count >= NUM_RESOURCES)
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

uint32_t read_encoder_resolution(HDSL_Handle hdslHandle)
{
    int32_t status = SystemP_FAILURE;
    uint32_t resolution = 0;

    /* Set the parameter channel buffers to 0xff */
    HDSL_write_pc_buffer(hdslHandle, 0, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 1, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 2, 0xff);
    HDSL_write_pc_buffer(hdslHandle, 3, 0xff);

    /* Parameter channel long message read with RID 0x81, Offset 5, Length 4
     * for reading resolution */

    status = HDSL_read_pc_long_msg(hdslHandle, 0x81, HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET, HDSL_LONG_MSG_ADDR_INDIRECT, HDSL_LONG_MSG_LENGTH_4, 0, LONG_MSG_TIMEOUT);

    DebugP_assert(SystemP_SUCCESS == status);

    gPc_buf0 = HDSL_read_pc_buffer(hdslHandle, 0);
    gPc_buf1 = HDSL_read_pc_buffer(hdslHandle, 1);
    gPc_buf2 = HDSL_read_pc_buffer(hdslHandle, 2);
    gPc_buf3 = HDSL_read_pc_buffer(hdslHandle, 3);

    resolution = log2((gPc_buf0 << 24) | (gPc_buf1 << 16) | (gPc_buf2 << 8) | (gPc_buf3));

    return resolution;
}

void hdsl_diagnostic_main(void *arg)
{
    uint32_t    val, acc_bits, pos_bits;
    uint8_t     ureg;
    uint8_t     chMask = 0;

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    int32_t     retVal = UDMA_SOK;
#endif

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

#if !defined(HDSL_MULTI_CHANNEL) && defined(_DEBUG_)
    /* UDMA initialization */
    chHandle = gConfigUdma0BlkCopyChHandle[0];  /* Has to be done after driver open */
    /* Channel enable */
    retVal = Udma_chEnable(chHandle);
    DebugP_assert(UDMA_SOK == retVal);
#endif

    /*C16 pin High for Enabling ch0 in booster pack */
    #if(CONFIG_HDSL0_BOOSTER_PACK && CONFIG_HDSL0_CHANNEL0)
        GPIO_setDirMode(ENC0_EN_BASE_ADDR, ENC0_EN_PIN, ENC0_EN_DIR);
        GPIO_pinWriteHigh(ENC0_EN_BASE_ADDR, ENC0_EN_PIN);
    #endif
    /*B17 pin High for Enabling ch2 in booster pack */
    #if(CONFIG_HDSL0_BOOSTER_PACK && CONFIG_HDSL0_CHANNEL2)
        GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
        GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
    #endif

    #ifndef HDSL_AM64xE1_TRANSCEIVER
        /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. This is required to remap EnDAT signals correctly via Interface card.*/
        HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x80));

    #if (CONFIG_HDSL0_BOOSTER_PACK == 0)
        /*Configure GPIO42 for HDSL mode.*/
        GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN, CONFIG_GPIO0_DIR);
        GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
    #endif
    #else
        /* Configure g_mux_en to 0 in ICSSG_SA_MX_REG Register. */
        HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x00));
        /*Configure GPIO42 for HDSL mode. New transceiver card needs the pin to be configured as input*/
        HW_WR_REG32(0x000F41D4, 0x00050001);   /* PRG0_PRU1_GPI9 as input */
        hdsl_i2c_io_expander(NULL);
    #endif
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    // initialize hdsl handle
    DebugP_log( "\n\n Hiperface DSL diagnostic\n");
    #if (CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ==PRU_CLK_FREQ_225M)
    gHdslHandleCh[0] = HDSL_open(gPruIcss0Handle, PRUICSS_PRUx, 0);
    hdsl_init();
    hdsl_pruss_load_run_fw(gHdslHandleCh[0]);
    #else
#if (CONFIG_HDSL0_CHANNEL0==1)
    gHdslHandleCh[0] = HDSL_open(gPruIcss0Handle, PRUICSS_RTU_PRU1, 1);
    DebugP_assert(gHdslHandleCh[0] != NULL);
    chMask |= CHANNEL_0_ENABLED;
#endif
#if (CONFIG_HDSL0_CHANNEL1==1)
    gHdslHandleCh[1] = HDSL_open(gPruIcss0Handle, PRUICSS_PRU1, 1);
    DebugP_assert(gHdslHandleCh[1] != NULL);
    chMask |= CHANNEL_1_ENABLED;
#endif
#if (CONFIG_HDSL0_CHANNEL2==1)
    gHdslHandleCh[2] = HDSL_open(gPruIcss0Handle, PRUICSS_TX_PRU1, 1);
    DebugP_assert(gHdslHandleCh[2] != NULL);
    chMask |= CHANNEL_2_ENABLED;
#endif
    hdsl_init_300m();
#if (CONFIG_HDSL0_CHANNEL0==1)
    HDSL_config_channel_mask(gHdslHandleCh[0], chMask);
#endif
#if (CONFIG_HDSL0_CHANNEL1==1)
    HDSL_config_channel_mask(gHdslHandleCh[1], chMask);
#endif
#if (CONFIG_HDSL0_CHANNEL2==1)
    HDSL_config_channel_mask(gHdslHandleCh[2], chMask);
#endif

    #if (CONFIG_HDSL0_CHANNEL0==1)
        hdsl_pruss_load_run_fw_300m(gHdslHandleCh[0]);
    #elif (CONFIG_HDSL0_CHANNEL1==1)
        hdsl_pruss_load_run_fw_300m(gHdslHandleCh[1]);
    #elif (CONFIG_HDSL0_CHANNEL2==1)
        hdsl_pruss_load_run_fw_300m(gHdslHandleCh[2]);
    #endif
    #endif
    DebugP_log( "\r\n HDSL setup finished\n");

    #if (CONFIG_HDSL0_CHANNEL0==1)
    /* Channel 0 starts here */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[0]);

        if((ureg & QM_LINK_ESTABLISHED) != 0)
            break;

        DebugP_log( "\r\n Hiperface DSL encoder not detected\n");
        ClockP_usleep(10000);
    }

    /* Wait until QM is 15 */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[0]);

        if(ureg == QM_LINK_ESTABLISHED_AND_VALUE_15)
            break;

        DebugP_log( "\r\n QM is not 15 \n");
        ClockP_usleep(10000);
    }

    DebugP_log( "\r\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |            Hiperface DSL Diagnostic : Channel 0                               |\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |                                                                               |\n");
    DebugP_log( "\r | Quality monitoring value: %u                                                  |\n", ureg & 0xF);
    ureg = HDSL_get_edges(gHdslHandleCh[0]);
    DebugP_log( "\r | Edges: 0x%x                                                                    |", ureg);
    ureg = HDSL_get_delay(gHdslHandleCh[0]);
    DebugP_log("\r\n | Cable delay: %u                                                                |", ureg & 0xF);
    DebugP_log("\r\n | RSSI: %u                                                                       |", (ureg & 0xF0) >> 4);
    val =HDSL_get_enc_id(gHdslHandleCh[0], 0) | (HDSL_get_enc_id(gHdslHandleCh[0], 1) << 8) |
              (HDSL_get_enc_id(gHdslHandleCh[0], 2) << 16);
    acc_bits = val & 0xF;
    acc_bits += 8;
    pos_bits = (val & 0x3F0) >> 4;
    pos_bits += acc_bits;
    DebugP_log("\r\n | Encoder ID: 0x%x", val);
    DebugP_log( "(");
    DebugP_log( "Acceleration bits: %u, ", acc_bits);
    DebugP_log( "Position bits: %u,", pos_bits);
    DebugP_log( "%s", val & 0x400 ? " Bipolar position" : " Unipolar position");
    DebugP_log(")|");
    gHdslHandleCh[0]->res = read_encoder_resolution(gHdslHandleCh[0]);
    gHdslHandleCh[0]->multi_turn = pos_bits - gHdslHandleCh[0]->res;
    gHdslHandleCh[0]->mask = pow(2, gHdslHandleCh[0]->res) - 1;
    if(gHdslHandleCh[0]->multi_turn)
    {
        DebugP_log( "\r\n | Single-turn bits: %u, Multi-turn bits: %u                                     |", pos_bits - gHdslHandleCh[0]->multi_turn, gHdslHandleCh[0]->multi_turn);
    }
    else
    {
        DebugP_log( "\r\n | Single-turn bits: %u                                                          |", pos_bits);
    }
    DebugP_log("\r\n |-------------------------------------------------------------------------------|");


    #endif
    #if (CONFIG_HDSL0_CHANNEL1==1)

    /* Channel 1 starts here */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[1]);

        if((ureg & QM_LINK_ESTABLISHED) != 0)
            break;

        DebugP_log( "\r\n Hiperface DSL encoder not detected\n");
        ClockP_usleep(10000);
    }

    /* Wait until QM is 15 */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[1]);

        if(ureg == QM_LINK_ESTABLISHED_AND_VALUE_15)
            break;

        DebugP_log( "\r\n QM is not 15 \n");
        ClockP_usleep(10000);
    }

    DebugP_log( "\r\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |            Hiperface DSL Diagnostic : Channel 1                               |\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |                                                                               |\n");
    DebugP_log( "\r | Quality monitoring value: %u                                                  |\n", ureg & 0xF);
    ureg = HDSL_get_edges(gHdslHandleCh[1]);
    DebugP_log( "\r | Edges: 0x%x                                                                    |", ureg);
    ureg = HDSL_get_delay(gHdslHandleCh[1]);
    DebugP_log("\r\n | Cable delay: %u                                                                |", ureg & 0xF);
    DebugP_log("\r\n | RSSI: %u                                                                       |", (ureg & 0xF0) >> 4);
    val =HDSL_get_enc_id(gHdslHandleCh[1], 0) | (HDSL_get_enc_id(gHdslHandleCh[1], 1) << 8) |
                (HDSL_get_enc_id(gHdslHandleCh[1], 2) << 16);
    acc_bits = val & 0xF;
    acc_bits += 8;
    pos_bits = (val & 0x3F0) >> 4;
    pos_bits += acc_bits;
    DebugP_log("\r\n | Encoder ID: 0x%x", val);
    DebugP_log( "(");
    DebugP_log( "Acceleration bits: %u, ", acc_bits);
    DebugP_log( "Position bits: %u,", pos_bits);
    DebugP_log( "%s", val & 0x400 ? " Bipolar position" : " Unipolar position");
    DebugP_log(")|");
    gHdslHandleCh[1]->res = read_encoder_resolution(gHdslHandleCh[1]);
    gHdslHandleCh[1]->multi_turn = pos_bits - gHdslHandleCh[1]->res;
    gHdslHandleCh[1]->mask = pow(2, gHdslHandleCh[1]->res) - 1;
    if(gHdslHandleCh[1]->multi_turn)
    {
        DebugP_log( "\r\n | Single-turn bits: %u, Multi-turn bits: %u                                     |", pos_bits - gHdslHandleCh[1]->multi_turn, gHdslHandleCh[1]->multi_turn);
    }
    else
    {
        DebugP_log( "\r\n | Single-turn bits: %u                                                          |", pos_bits);
    }
    DebugP_log("\r\n |-------------------------------------------------------------------------------|");
    #endif
    #if (CONFIG_HDSL0_CHANNEL2==1)

    /* Channel 2 starts here */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[2]);

        if((ureg & QM_LINK_ESTABLISHED) != 0)
            break;

        DebugP_log( "\r\n Hiperface DSL encoder not detected\n");
        ClockP_usleep(10000);
    }

    /* Wait until QM is 15 */
    while(1)
    {
        ureg = HDSL_get_master_qm(gHdslHandleCh[2]);

        if(ureg == QM_LINK_ESTABLISHED_AND_VALUE_15)
            break;

        DebugP_log( "\r\n QM is not 15 \n");
        ClockP_usleep(10000);
    }

    DebugP_log( "\r\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |            Hiperface DSL Diagnostic : Channel 2                               |\n");
    DebugP_log( "\r |-------------------------------------------------------------------------------|\n");
    DebugP_log( "\r |                                                                               |\n");
    DebugP_log( "\r | Quality monitoring value: %u                                                  |\n", ureg & 0xF);
    ureg = HDSL_get_edges(gHdslHandleCh[2]);
    DebugP_log( "\r | Edges: 0x%x                                                                    |", ureg);
    ureg = HDSL_get_delay(gHdslHandleCh[2]);
    DebugP_log("\r\n | Cable delay: %u                                                                |", ureg & 0xF);
    DebugP_log("\r\n | RSSI: %u                                                                       |", (ureg & 0xF0) >> 4);
    val =HDSL_get_enc_id(gHdslHandleCh[2], 0) | (HDSL_get_enc_id(gHdslHandleCh[2], 1) << 8) |
                (HDSL_get_enc_id(gHdslHandleCh[2], 2) << 16);
    acc_bits = val & 0xF;
    acc_bits += 8;
    pos_bits = (val & 0x3F0) >> 4;
    pos_bits += acc_bits;
    DebugP_log("\r\n | Encoder ID: 0x%x", val);
    DebugP_log( "(");
    DebugP_log( "Acceleration bits: %u, ", acc_bits);
    DebugP_log( "Position bits: %u,", pos_bits);
    DebugP_log( "%s", val & 0x400 ? " Bipolar position" : " Unipolar position");
    DebugP_log(")|");
    gHdslHandleCh[2]->res = read_encoder_resolution(gHdslHandleCh[2]);
    gHdslHandleCh[2]->multi_turn = pos_bits - gHdslHandleCh[2]->res;
    gHdslHandleCh[2]->mask = pow(2, gHdslHandleCh[2]->res) - 1;
    if(gHdslHandleCh[2]->multi_turn)
    {
        DebugP_log( "\r\n | Single-turn bits: %u, Multi-turn bits: %u                                     |", pos_bits - gHdslHandleCh[2]->multi_turn, gHdslHandleCh[2]->multi_turn);
    }
    else
    {
        DebugP_log( "\r\n | Single-turn bits: %u                                                          |", pos_bits);
    }
    DebugP_log("\r\n |-------------------------------------------------------------------------------|");
    #endif
    while(1)
    {
        int32_t menu;
        display_menu();
        menu = get_menu();
        if (CONFIG_HDSL0_CHANNEL0==1)
        {
            DebugP_log( "|\r\n Channel 0 ");
            process_request(gHdslHandleCh[0], menu);
            DebugP_log( "\r%s", gUart_buffer);
        }

        if (CONFIG_HDSL0_CHANNEL1==1)
        {
            DebugP_log( "|\r\n Channel 1");
            process_request(gHdslHandleCh[1], menu);
            DebugP_log( "\r%s", gUart_buffer);
        }
        if (CONFIG_HDSL0_CHANNEL2==1)
        {
           DebugP_log( "|\r\n Channel 2");
           process_request(gHdslHandleCh[2], menu);
           DebugP_log( "\r%s", gUart_buffer);
        }
    }
    Board_driversClose();
    Drivers_close();
}
