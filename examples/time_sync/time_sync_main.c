/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss.h>
#include <drivers/pinmux.h>
#include <stdlib.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <time_sync.h>
#include <rtupru0_load_bin.h>

const uint8_t gIepIncrementValue = 5;

/** \brief Global handle for PRUICSS0 instance */
PRUICSS_Handle gPruIcss0Handle;
/** \brief Global handle for PRUICSS1 instance */
PRUICSS_Handle gPruIcss1Handle;

static TCA6424_Config  gTCA6424_Config;

extern uint32_t gpioBaseAddr, pinNum;

/* Structure to hold time synchronization parameters */
#ifndef TIME_TRANSMITTER_RECEIVER
TimesyncParams timesyncParams0;
TimesyncHandle timesyncHandle0;
#endif
TimesyncParams timesyncParams1;
TimesyncHandle timesyncHandle1;
/* Structure to hold time synchronization debug information */
#ifndef TIME_TRANSMITTER_RECEIVER
TimesyncDebug timesyncDebug0;
#endif
TimesyncDebug timesyncDebug1;

/* Processing delay macros which include propagation delay and CPU latency (in nanoseconds) */
/* Refer to Time Sync Design SDK documentation for calculation methodology */
#define TIMESYNC_PROCESSING_DELAY_TRANSMITTER_RECEIVER  (400U) /* Value applicable in release mode of transmitter and receiver example */
#define TIMESYNC_PROCESSING_DELAY_RECEIVER              (750U) /* Value applicable in debug mode of receiver example */

/* TIMESYNC router configuration register offsets and values */
#define TIMESYNC_EVENT_ROUTER_REG_SIZE         (4U)
#define TIMESYNC_EVENT_ROUTER_OUT12_OFFSET     (12U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)
#define TIMESYNC_EVENT_ROUTER_OUT8_OFFSET      (8U * TIMESYNC_EVENT_ROUTER_REG_SIZE + 4U)
#define TIMESYNC_EVENT_ROUTER_IN25_TO_OUT12    (0x00010019U)  /* PRU_ICSSG0_PR1_EDC0_SYNC0_OUT_0 to PRG1_IEP0_LATCH_IN0 */
#define TIMESYNC_EVENT_ROUTER_IN4_TO_OUT8      (0x00010004U)  /* PINFUNCTION_PRG0_IEP0_LATCH_IN0 to PRG0_IEP0_LATCH_IN0 */

/* IEP configuration values */
#define IEP_SYNC_PULSE_WIDTH                   (200U)
#define IEP_SYNC_DISABLED                      (0x0000U)
#define IEP_CMP_EVENTS_ENABLE_ALL              (0x1FFFEU)

/* Helper macro to access IEP registers */
#define IEP_REG_ADDR(pruHandle, regOffset) \
    ((uint32_t)(((PRUICSS_HwAttrs *)(pruHandle->hwAttrs))->iep0RegBase) + (regOffset))

#if defined(am243x_evm)
static void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;

    /* Initialize the TCA6424 params */
    TCA6424_Params_init(&tca6424Params);

    /* Open the TCA6424 device */
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    /* If the open is successful, set the IO expander pin high and
     * configure it as output
     */
    if(status == SystemP_SUCCESS)
    {
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        uint32_t ioIndex = 0x0a;
        status = TCA6424_setOutput(
                    &gTCA6424_Config,
                    ioIndex,
                    TCA6424_OUT_STATE_HIGH);

        /* Configure the pin as output */
        status += TCA6424_config(
                    &gTCA6424_Config,
                    ioIndex,
                    TCA6424_MODE_OUTPUT);
    }

    /* Close the TCA6424 device */
    TCA6424_close(&gTCA6424_Config);
}
#endif

void pru_icss_with_time_sync_main(void *args)
{
    Drivers_open();

    int32_t status;
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    /* Call the board specific function to configure the IO expander */
#if defined(am243x_evm)
    i2c_io_expander(NULL);
#endif
    /* Initialize the PRUICSS DMEM memory */
    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    DebugP_assert(status != 0);
    /* Load the RTU0 G0 firmware for sync generation*/
    status = PRUICSS_loadFirmware(gPruIcss0Handle, PRUICSS_RTU_PRU0, RTUPRU0_Firmware_0, sizeof(RTUPRU0_Firmware_0));
    DebugP_assert(SystemP_SUCCESS == status);

#ifdef TIME_TRANSMITTER_RECEIVER
    gPruIcss1Handle = PRUICSS_open(CONFIG_PRU_ICSS1);
    /* Initialize the PRUICSS DMEM memory */
    status = PRUICSS_initMemory(gPruIcss1Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    DebugP_assert(status != 0);
    /* Load the RTU G1 firmware for sync generation*/
    status = PRUICSS_loadFirmware(gPruIcss1Handle, PRUICSS_RTU_PRU0, RTUPRU0_Firmware_0, sizeof(RTUPRU0_Firmware_0));
    DebugP_assert(SystemP_SUCCESS == status);
    /*Connect TIMESYNC_INTRTR0_IN25(PRU_ICSSG0_PR1_EDC0_SYNC0_OUT_0) to TIMESYNC_INTRTR0_OUT12(PRG1_IEP0_LATCH_IN0)*/
    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT12_OFFSET), TIMESYNC_EVENT_ROUTER_IN25_TO_OUT12);
    timesyncHandle1 = timesync_init(&timesyncParams1, IEP_REG_ADDR(gPruIcss1Handle, 0));
    timesyncHandle1->iepIncrementValue = gIepIncrementValue;
#ifdef ENABLE_DEBUG_LOGS
    timesyncHandle1->timesyncDebugPtr = &timesyncDebug1;
#endif
    timesyncHandle1->processingDelay = TIMESYNC_PROCESSING_DELAY_TRANSMITTER_RECEIVER;
#endif

#ifdef TIME_RECEIVER
    /*TIME SYNC router configuration */
    /*Connect TIMESYNC_INTRTR0_IN4(PINFUNCTION_PRG0_IEP0_LATCH_IN0) to TIMESYNC_INTRTR0_OUT8 (PRG0_IEP0_LATCH_IN0)*/
    HW_WR_REG32((CSL_TIMESYNC_EVENT_INTROUTER0_CFG_BASE + TIMESYNC_EVENT_ROUTER_OUT8_OFFSET), TIMESYNC_EVENT_ROUTER_IN4_TO_OUT8);
    timesyncHandle0 = timesync_init(&timesyncParams0, IEP_REG_ADDR(gPruIcss0Handle, 0));
    timesyncHandle0->iepIncrementValue = gIepIncrementValue;
#ifdef ENABLE_DEBUG_LOGS
    timesyncHandle0->timesyncDebugPtr = &timesyncDebug0;
#endif
    timesyncHandle0->processingDelay = TIMESYNC_PROCESSING_DELAY_RECEIVER;
#endif
    /*disable IEP0 of ICSSG0 */
    PRUICSS_controlIepCounter(gPruIcss0Handle, 0, 0);
    /*Initialize IEP0 counter value*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0), 0x0);
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1), 0x0);
    /*Configure non zero compare1 value, compare 1 value is initialized with 1000 in sync generation firmware*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0), SYNC_PERIOD_IN_NS);
    /*clear compare status*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG), 0xFFFF);
    /*configure sync pulse width and disable sync*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG), IEP_SYNC_PULSE_WIDTH);
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG), IEP_SYNC_DISABLED);
    /*enable all compare events of ICSSG0*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss0Handle, CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), IEP_CMP_EVENTS_ENABLE_ALL);
#ifdef TIME_TRANSMITTER_RECEIVER
    /*disable IEP0 of ICSSG1*/
    PRUICSS_controlIepCounter(gPruIcss1Handle, 0, 0);
    /*Initialize IEP0 counter value*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0), 0x0);
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1), 0x0);
    /*Configure non zero compare1 value, compare 1 value is initialized with 1000 in sync generation firmware*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0), SYNC_PERIOD_IN_NS);
    /*clear compare status*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_CMP_STATUS_REG), 0xFFFF);
    /*configure sync pulse width and disable sync*/
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG), IEP_SYNC_PULSE_WIDTH);
    HW_WR_REG32(IEP_REG_ADDR(gPruIcss1Handle, CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG), IEP_SYNC_DISABLED);
    /* Enable IEP0 with increment of 5 for ICSSG1*/
    PRUICSS_setIepCounterIncrementValue(gPruIcss1Handle, 0, gIepIncrementValue);
    PRUICSS_controlIepCounter(gPruIcss1Handle, 0, 1);
#endif
    /* Enable IEP0 with increment of 5 for ICSSG0*/
    PRUICSS_setIepCounterIncrementValue(gPruIcss0Handle, 0, gIepIncrementValue);
    PRUICSS_controlIepCounter(gPruIcss0Handle, 0, 1);


#ifdef TIME_RECEIVER
#if (defined(_DEBUG_) != 1)
    DebugP_log("This example is supported only in debug mode\n");
    while(1)
    {
        ClockP_usleep(1);
    }
#else
    while(1)
    {
#ifdef ENABLE_DEBUG_GPIO
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
#endif
       timesync_run(timesyncHandle0);
#ifdef ENABLE_DEBUG_GPIO
    GPIO_pinWriteLow(gpioBaseAddr, pinNum);
#endif
    }
#endif
#endif
#ifdef TIME_TRANSMITTER_RECEIVER
    /* Wait for offset to be stable */
    while(1)
    {
#ifdef ENABLE_DEBUG_GPIO
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
#endif
       timesync_run(timesyncHandle1);
#ifdef ENABLE_DEBUG_GPIO
    GPIO_pinWriteLow(gpioBaseAddr, pinNum);
#endif
    }
#endif
    /* Close the board drivers */
    Board_driversClose();
    /* Close the PRUICSS driver */
    Drivers_close();
}