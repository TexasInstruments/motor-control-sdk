/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
 * \file  bissc_diagnostic.c
 *
 * \brief BiSS-C encoder diagnostic application
 *
 * This application provides comprehensive diagnostic and testing capabilities for
 * BiSS-C encoders, including single-shot position reads, control communication,
 * continuous position monitoring, and periodic trigger mode.
 *
 * \par Dual Slice Support (tested only on AM261x with Single PRU Single Channel mode):
 * This diagnostic application supports dual PRU slice operation when compiled with
 * BISSC_DUAL_PRU_SLICE_ENABLE defined. In dual slice mode:
 * - Two independent BiSS-C instances (CONFIG_BISSC0, CONFIG_BISSC1) run simultaneously
 * - Each instance operates on a different PRU slice (PRU0 or PRU1)
 * - Both instances must use the same PRU-ICSS instance (validated at runtime)
 * - Both instances share common PRU-ICSS level resources
 *
 * \par Instance vs Slice:
 * - Instance: Software driver instance (CONFIG_BISSC0, CONFIG_BISSC1) with independent
 *   configuration, state, and driver handle. Configured via SysConfig.
 * - Slice: Hardware PRU slice number (0 or 1) within PRU-ICSS where firmware executes.
 *   CONFIG_BISSCx_PRUICSS_SLICE specifies which slice each instance uses.
 *
 * \par Driver Handle Array:
 * The global handle array gAppBisscHandle[CONFIG_BISSC_NUM_INSTANCES] stores driver
 * handles for all configured instances:
 * - gAppBisscHandle[CONFIG_BISSC0]: Handle for first instance (always present)
 * - gAppBisscHandle[CONFIG_BISSC1]: Handle for second instance (dual handle mode, tested only on AM261x with Single PRU Single Channel mode)
 * - CONFIG_BISSC_NUM_INSTANCES: Number of instances (1 or 2), defined by SysConfig
 * ASSUMPTIONS: Loop (with index i) is used to call functions with handle as argument.
 *      - i = 0 will use CONFIG_BISSC0
 *      - i = 1 will use CONFIG_BISSC1 when BISSC_DUAL_PRU_SLICE_ENABLE is defined
 *
 * \par Shared Resources:
 * When both instances use the same PRU-ICSS instance, they share:
 * - IEP Timer: Used for periodic trigger mode, configured via first instance
 * - PRU-ICSS INTC: Interrupt controller shared across slices
 *
 * \par First instance (CONFIG_BISSC0) is used for shared resources:
 * Several operations use gAppBisscHandle[CONFIG_BISSC0] to access shared PRU-ICSS
 * resources. This approach works correctly because validation in bissc_pruicss_init()
 * ensures both instances use the same PRU-ICSS instance.
 *
 * \par Application Flow:
 * 1. Initialize PRU-ICSS and disable cores
 * 2. Initialize BiSS-C driver for each instance (bissc_init)
 * 3. Get encoder resolution parameters
 * 4. Load and run PRU firmware for each slice
 * 5. Wait for firmware initialization and measure processing delays
 * 6. Enter interactive menu for encoder operations
 * 7. Deinitialize all instances on exit
 *
 * \par Periodic Trigger Mode:
 * In periodic mode, the IEP timer automatically triggers BiSS-C transactions:
 * - Each instance/channel can have different trigger times
 * - All instances share the same IEP reset count (period)
 * - Configured via bissc_periodic_interface structure
 * - See bissc_periodic_trigger.c for detailed IEP configuration
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>

#if defined(SOC_AM243X)
#include <drivers/sciclient.h>
#endif

#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <position_sense/bissc/include/bissc_drv.h>
#include <position_sense/bissc/include/bissc_api.h>
#include "bissc_periodic_trigger.h"

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
/* Multi-channel single PRU mode */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include  <bissc_receiver_multi_pru1_bin.h>
#else
#include  <bissc_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
/* Multi-channel multi-PRU mode (load share mode) */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include <bissc_receiver_multi_rtu_pru1_bin.h>
#include <bissc_receiver_multi_pru1_bin.h>
#include <bissc_receiver_multi_tx_pru1_bin.h>
#else
#include <bissc_receiver_multi_rtu_pru0_bin.h>
#include <bissc_receiver_multi_pru0_bin.h>
#include <bissc_receiver_multi_tx_pru0_bin.h>
#endif
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
/* Single channel single PRU mode */
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#include  <bissc_receiver_pru1_bin.h>
#else
#include  <bissc_receiver_pru0_bin.h>
#endif
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
#if !defined(SOC_AM261X) || (CONFIG_BISSC1_MODE != BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#error "Dual handle example using PRU0 and PRU1 is tested only with BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other combinations, update code and remove this line."
#endif

/* Single channel single PRU mode for second slice */
#if (CONFIG_BISSC1_PRUICSS_SLICE == 1)
#include  <bissc_receiver_pru1_bin.h>
#else
#include  <bissc_receiver_pru0_bin.h>
#endif

#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define WAIT_5_SECOND                       (5000)
#define WAIT_2_SECOND                       (2000)

#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

#define BISSC_CMD_EXIT_APP                  (0)
#define BISSC_CMD_ENC_LEN_UPDATE            (1)
#define BISSC_CMD_ENC_FREQ_UPDATE           (2)
#define BISSC_CMD_ENC_SEND_POS              (3)
#define BISSC_CMD_ENC_CTRL_CMD              (4)
#define BISSC_CMD_ENC_LOOP_OVER_CYC         (5)
#define BISSC_CMD_PERIODIC_TRIGGER_CMP      (6)
#define BISSC_CMD_PERIODIC_TRIGGER_CAP      (7)
#define BISSC_ENABLE_SAFETY                 (8)

#define BISSC_POSITION_LOOP_STOP            0
#define BISSC_POSITION_LOOP_START           1

#define BISSC_CLOCK_CONFIG_DELAY_SEC        (2U)
#define BISSC_CTRL_REG_ADDR_MASK            (0x7FU)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* BiSS-C Driver Handle */
bissc_handle gAppBisscHandle[CONFIG_BISSC_NUM_INSTANCES] = {NULL};

/* BiSS-C Periodic Interface Struct Instance */
bissc_periodic_interface gBisscPeriodicInterface;

/* Global variable to track position loop status */
volatile int32_t gBisscPositionLoopStatus;

/* Task related global variables */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void bissc_pruicss_init(void);
static void bissc_pruicss_load_run_fw(void);
static void bissc_display_fw_version(void);
static void bissc_display_menu(void);
static void bissc_get_enc_data_len(bissc_handle handle);
static void bissc_print_res(bissc_handle handle);
static int32_t bissc_get_command(void);
static void bissc_position_loop_decide_termination(void *args);
static int32_t bissc_loop_task_create(void);
static void bissc_process_periodic_command(bissc_handle handle[CONFIG_BISSC_NUM_INSTANCES], uint64_t trigger_count[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX], uint64_t iep_reset_count, uint8_t is_cap_mode);
void bissc_main(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void bissc_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    uint8_t rtu_pru_id = CONFIG_BISSC0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    uint8_t tx_pru_id = CONFIG_BISSC0_PRUICSS_TX_PRU_ID;
#endif
#else
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#endif

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);

#ifdef CONFIG_BISSC0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /*
     * These checks are applicable only if both BiSS-C instances
     * use same PRU-ICSSG instance. If different instances are used,
     * these checks can be removed.
     */
#if (CONFIG_BISSC0_PRUICSS_INSTANCE != CONFIG_BISSC1_PRUICSS_INSTANCE)
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple BiSS-C handles in this example */
    DebugP_log("\r\n PRU-ICSS instance should be set uniformly for both BiSS-C instances in SysConfig.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_BISSC0_G_MUX_EN) && !defined(CONFIG_BISSC1_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both BiSS-C instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_BISSC1_G_MUX_EN) && !defined(CONFIG_BISSC0_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both BiSS-C instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif
#endif

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_BISSC0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    pru_id = CONFIG_BISSC1_PRUICSS_PRU_ID;

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_BISSC1_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    /* Disable PRU cores for second slice */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
}

static void bissc_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = BiSSFirmwareMultiMakeRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(BiSSFirmwareMultiMakeRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_BISSC0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = BiSSFirmwareMultiMakePru1_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwareMultiMakePru1_0);
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = BiSSFirmwareMultiMakeTxPru1_0;
    uint32_t tx_pru_firmware_size = sizeof(BiSSFirmwareMultiMakeTxPru1_0);
    uint8_t tx_pru_id = CONFIG_BISSC0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = BiSSFirmwareMultiMakeRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(BiSSFirmwareMultiMakeRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_BISSC0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = BiSSFirmwareMultiMakePru0_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwareMultiMakePru0_0);
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = BiSSFirmwareMultiMakeTxPru0_0;
    uint32_t tx_pru_firmware_size = sizeof(BiSSFirmwareMultiMakeTxPru0_0);
    uint8_t tx_pru_id = CONFIG_BISSC0_PRUICSS_TX_PRU_ID;
#endif
#endif
#elif (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = BiSSFirmwareMultiPru1_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwareMultiPru1_0);
#else
    const uint32_t *pru_firmware = BiSSFirmwareMultiPru0_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwareMultiPru0_0);
#endif
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#else
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = BiSSFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwarePru1_0);
#else
    const uint32_t *pru_firmware = BiSSFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(BiSSFirmwarePru0_0);
#endif
    uint8_t pru_id = CONFIG_BISSC0_PRUICSS_PRU_ID;
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
    /* Disable RTU-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to RTU-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_BISSC0_PRUICSS_SLICE), 0,
                                  (uint32_t *)rtu_pru_firmware, rtu_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset RTU-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable RTU-PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_BISSC0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
    /* Disable TX-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to TX-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_BISSC0_PRUICSS_SLICE), 0,
                                  (uint32_t *)tx_pru_firmware, tx_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset TX-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable TX-PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_BISSC0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /* Load and run firmware for second slice */
#if (CONFIG_BISSC1_PRUICSS_SLICE == 1)
    pru_firmware = BiSSFirmwarePru1_0;
    pru_firmware_size = sizeof(BiSSFirmwarePru1_0);
#else
    pru_firmware = BiSSFirmwarePru0_0;
    pru_firmware_size = sizeof(BiSSFirmwarePru0_0);
#endif
    pru_id = CONFIG_BISSC1_PRUICSS_PRU_ID;

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_BISSC1_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

#endif
}

static void bissc_display_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version(s) depending on configuration */
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwareMultiPru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwareMultiPru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#elif (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_BISSC0_CHANNEL0_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwareMultiMakeRtuPru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwareMultiMakeRtuPru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware for channel 0 (RTU-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_BISSC0_CHANNEL1_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwareMultiMakePru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwareMultiMakePru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware for channel 1 (PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_BISSC0_CHANNEL2_ENABLED == 1)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwareMultiMakeTxPru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwareMultiMakeTxPru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware for channel 2 (TX-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#elif (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (CONFIG_BISSC0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    /* Print firmware version for second slice */
#if (CONFIG_BISSC1_PRUICSS_SLICE == 1)
    version = *((uint32_t *)BiSSFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)BiSSFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nBiSS-C firmware for second slice \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
}

static void bissc_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select input parameters                          |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 1 : Number of position data bits                                             |");
    DebugP_log("\r\n| 2 : Select clock frequency in MHz(1/2/5/8/10)                                |");
    DebugP_log("\r\n| 3 : Encoder send position values                                             |");
    DebugP_log("\r\n| 4 : Control Communication - Register Read/Write                              |");
    DebugP_log("\r\n| 5 : Loop over BiSS-C cycles                                                  |");
    DebugP_log("\r\n| 6 : Start periodic continuous mode (CMP trigger)                             |");
    DebugP_log("\r\n| 7 : Start periodic continuous mode (CAP trigger)                             |");
    DebugP_log("\r\n| 8 : Enable safety mode                                                       |");
    DebugP_log("\r\n| 0 : Exit the application                                                     |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| Enter value:\r\n");
}

static void bissc_get_enc_data_len(bissc_handle handle)
{
    int32_t ret;
    uint32_t single_turn_len[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX], multi_turn_len[BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX], enc_num = 0, total_channels, ch_num;
    const bissc_attrs *attrs = bissc_get_attrs(handle);

    ret = bissc_clear_data_len(handle);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Failed to clear data length\n");
        return;
    }

    if(attrs->load_share_enabled)
        total_channels = bissc_get_total_channels(handle);
    else
        total_channels = 1;

    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        for(enc_num = 0; enc_num < BISSC_NUM_ENCODERS_IN_DAISY_CHAIN_MAX; enc_num++)
        {
            single_turn_len[enc_num] = 0;
            multi_turn_len[enc_num] = 0;
        }

        /* BiSS-C Frame Size Constraints:
         * BiSS-C protocol uses 64-bit data frames. Total frame size must not exceed 64 bits.
         *
         * Frame structure WITHOUT Safety mode:
         * Position Data + E/W(2) + CRC(6) <= 64 bits
         *
         *   Therefore: single_turn + multi_turn <= 56 bits
         *
         * Frame structure WITH Safety mode:
         *   Position Data + E/W(2) + sign-of-life(6) + safety CRC(16) <= 64 bits
         *   Therefore: single_turn + multi_turn <= 40 bits
         *
         * Examples:
         *   - Without Safety: 32-bit single-turn + 12-bit multi-turn = 44 bits (valid, <= 56)
         *   - With Safety: 32-bit single-turn + 8-bit multi-turn = 40 bits (valid, <= 40)
         *   - Without Safety: 40-bit single-turn + 20-bit multi-turn = 60 bits (invalid, > 56)
         */

        DebugP_log("\r\n=======================================================================\n");
        DebugP_log("BiSS-C Frame Size Constraints:\n");
        DebugP_log("  - Without Safety: single_turn + multi_turn <= 56 bits\n");
        DebugP_log("  - With Safety: single_turn + multi_turn <= 40 bits\n");
        DebugP_log("=======================================================================\n");
        DebugP_log("\r\nPlease enter encoder lengths connected to Channel %u:\n", handle->priv->channel[ch_num]);
        DebugP_log("\r\nPlease enter 1st encoder single turn length\n");
        DebugP_scanf("%u\n", &single_turn_len[0]);
        if(single_turn_len[0])
        {
            DebugP_log("\r\nPlease enter 1st encoder multiturn length, 0 if not multiturn\n");
            DebugP_scanf("%u\n", &multi_turn_len[0]);
        }
        DebugP_log("\r\nPlease enter 0 as data length if daisy chain is not used\n");
        DebugP_log("\r\nPlease enter 2nd encoder single turn length\n");
        DebugP_scanf("%u\n", &single_turn_len[1]);
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 2nd encoder multiturn length, 0 if not multiturn\n");
            DebugP_scanf("%u\n", &multi_turn_len[1]);
        }
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 3rd encoder single turn length\n");
            DebugP_scanf("%u\n", &single_turn_len[2]);
            if(single_turn_len[2])
            {
                DebugP_log("\r\nPlease enter 3rd encoder multiturn length, 0 if not multiturn\n");
                DebugP_scanf("%u\n", &multi_turn_len[2]);
            }
        }
        ret = bissc_update_data_len(handle, single_turn_len, multi_turn_len, ch_num);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r\n[BiSS-C] ERROR: Invalid encoder configuration for channel %u\n", ch_num);
            return;
        }
    }
}

static void bissc_print_res(bissc_handle handle)
{
    const bissc_attrs *attrs = bissc_get_attrs(handle);
    uint32_t ch_num, ch, ls_ch;

    for(ch_num = 0; ch_num < attrs->total_channels; ch_num++)
    {
        ch = bissc_get_current_channel(handle, ch_num);
        if(attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
            ls_ch = ch;
        else
            ls_ch = 0;
        if((attrs->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU) || (attrs->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU))
            DebugP_log("%s", (ch_num != (attrs->total_channels-1))?"\r":" & ");
        else
            DebugP_log("\r");
        if(handle->priv->data_len[ls_ch][1])
        {
            if(handle->priv->data_len[ls_ch][2])
            {
                if(handle->priv->multi_turn_len[ls_ch][2])
                {
                    DebugP_log("Channel:%u - Enc3: MT rev:%u, Angle:%.12f, Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc_1:%u ", ch, handle->priv->enc_pos_data[ch].num_of_turns[2], handle->priv->enc_pos_data[ch].angle[2], handle->priv->enc_pos_data[ch].num_of_turns[1], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][2], handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%u - Enc3: Angle:%.12f, Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[2], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][2], handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
            }
            else
            {
                if(handle->priv->multi_turn_len[ls_ch][1])
                {
                    DebugP_log("Channel:%u - Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].num_of_turns[1], handle->priv->enc_pos_data[ch].angle[1],
                    handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][1], handle->priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%u - Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[1], handle->priv->enc_pos_data[ch].angle[0], handle->priv->pd_crc_err_cnt[ch][1],
                    handle->priv->pd_crc_err_cnt[ch][0]);
                }
            }
        }
        else
        {
            if(handle->priv->multi_turn_len[ls_ch][0])
            {
                DebugP_log("Channel:%u - Enc1: MT rev:%u, Angle:%.12f, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].num_of_turns[0], handle->priv->enc_pos_data[ch].angle[0],
                handle->priv->pd_crc_err_cnt[ch][0]);
            }
            else
            {
                DebugP_log("Channel:%u - Enc1: Angle:%.12f, crc error count enc1:%u ", ch, handle->priv->enc_pos_data[ch].angle[0],
                handle->priv->pd_crc_err_cnt[ch][0]);
            }
        }
    }
}

static int32_t bissc_get_command()
{
    int32_t cmd;
    DebugP_scanf("%d\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if( cmd < BISSC_CMD_EXIT_APP || cmd > BISSC_ENABLE_SAFETY )
    {
        DebugP_log("\r\n| WARNING: invalid option try again\n");
        return SystemP_FAILURE;
    }
    return cmd;
}

static void bissc_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gBisscPositionLoopStatus = BISSC_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t bissc_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params task_params;

    TaskP_Params_init(&task_params);
    task_params.name = "bissc_position_loop_decide_termination";
    task_params.stackSize = TASK_STACK_SIZE;
    task_params.stack = (uint8_t *)gTaskFxnStack;
    task_params.priority = TASK_PRIORITY;
    task_params.taskMain = (TaskP_FxnMain)bissc_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &task_params);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: TaskP_construct() for bissc_position_loop_decide_termination failed\n");
    }

    return status;
}

static void bissc_process_periodic_command(bissc_handle handle[CONFIG_BISSC_NUM_INSTANCES], uint64_t trigger_count[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX], uint64_t iep_reset_count, uint8_t is_cap_mode)
{
    /* Any function call failure will lead to exit of bissc_process_periodic_command function */
    int32_t ret;
    uint32_t i;
    uint32_t pos_fail_cnt[CONFIG_BISSC_NUM_INSTANCES] = {0}, pos_total_cnt = 0;
    const bissc_attrs *attrs;

    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {  
        if(is_cap_mode)
        {
            if(bissc_config_periodic_trigger_cap_mode(handle[i]) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to configure periodic trigger in CAP mode\n");
                return;
            }
        }
        else
        {
            if(bissc_config_periodic_trigger_cmp_mode(handle[i]) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to configure periodic trigger in CMP mode\n");
                return;
            }
        }
    }

    if(bissc_loop_task_create() != SystemP_SUCCESS)
    {
        return;
    }
    if(is_cap_mode == 0)
    {
        for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
        {
            attrs = bissc_get_attrs(handle[i]);

            if(attrs->load_share_enabled)
            {
                if(attrs->channel0_enabled)
                {
                    gBisscPeriodicInterface.periodic_trigger_count[i][0] = trigger_count[i][0];
                }
                if(attrs->channel1_enabled)
                {
                    gBisscPeriodicInterface.periodic_trigger_count[i][1] = trigger_count[i][1];
                }
                if(attrs->channel2_enabled)
                {
                    gBisscPeriodicInterface.periodic_trigger_count[i][2] = trigger_count[i][2];
                }
            }
            else
            {
                gBisscPeriodicInterface.periodic_trigger_count[i][0] = trigger_count[i][0];
            }
        }
    }
    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        gBisscPeriodicInterface.handle[i] = handle[i];
    }
    gBisscPeriodicInterface.iep_reset_count = iep_reset_count;
    gBisscPeriodicInterface.is_cap_mode = is_cap_mode;

    if(bissc_config_periodic_mode(&gBisscPeriodicInterface) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: bissc_config_periodic_mode failed\r\n|\r\n|\n");
        return;
    }

    gBisscPositionLoopStatus = BISSC_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| Press Enter to stop the continuous mode\r\n|");

    while(1)
    {
        pos_total_cnt++;
        if(gBisscPositionLoopStatus == BISSC_POSITION_LOOP_STOP)
        {
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                DebugP_log("\r\n Failed %u out of %u times for BiSS-C instance %u\n", pos_fail_cnt[i], pos_total_cnt, i);
            }
            if (bissc_stop_periodic_mode(&gBisscPeriodicInterface) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: bissc_stop_periodic_mode failed\r\n|\r\n|\n");

            }
            return;
        }
        else
        {
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                ret = bissc_get_pos(handle[i]);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Position data measurement failed for BiSS-C instance %u\n", i);
                    pos_fail_cnt[i]++;
                    continue;
                }
                bissc_print_res(handle[i]);
            }
        }
    }
    return;
}

/**
 * \brief   BiSS-C diagnostic application main function
 *
 * \details This function implements the main diagnostic application flow for BiSS-C
 *          encoder interface. It initializes the BiSS-C driver, loads and starts PRU
 *          firmware, and provides an interactive menu-driven interface for various
 *          encoder operations including:
 *          - Position data acquisition (single-shot and continuous)
 *          - Control communication (register read/write)
 *          - Frequency configuration
 *          - Safety mode operations
 *          - Periodic trigger mode configuration
 *
 *          Flow:
 *          1. Initialize SoC drivers and board drivers
 *          2. Enable booster pack power pins if configured
 *          3. Initialize PRU-ICSS subsystem
 *          4. Initialize BiSS-C driver
 *          5. Get encoder resolution parameters from user
 *          6. Load and run PRU firmware(s) and configure host trigger mode (default)
 *          7. Validate encoder processing delays (multi-channel mode)
 *          8. Enter interactive menu loop for encoder operations
 *          9. De-initialize on exit
 *
 *          Trigger Modes:
 *          - Host Trigger Mode (default): Each BiSS-C transaction is initiated by the host (R5F)
 *            via API calls. This mode is configured by default.
 *          - Periodic Trigger Mode: BiSS-C transactions are automatically triggered by IEP timer
 *            at regular intervals. This mode can be enabled through the interactive menu.
 *
 *          NOTE on driver APIs:
 *          BiSS-C driver APIs use a simplified validation approach for optimal performance:
 *          - **Handle validation**: All public APIs validate the handle parameter for NULL
 *          - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *          - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *            are validated once during bissc_init() and assumed valid in subsequent API calls
 *          - This strategy reduces overhead in time-critical data path functions
 *
 * \param[in]   args    Unused
 */

void bissc_main(void *args)
{
    int32_t ret;
    uint32_t ch = 0, ls_ch = 0, ch_num, enc_num = 0, total_channels;
    uint32_t i = 0;
    const bissc_attrs *attrs[CONFIG_BISSC_NUM_INSTANCES] = {NULL};
    bissc_priv *priv[CONFIG_BISSC_NUM_INSTANCES] = {NULL};
    bissc_params bissc_params_instance;

    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                           */
    /* ========================================================================== */
    Drivers_open();          /* Open SoC drivers */
    Board_driversOpen();     /* Open board-specific drivers */
    /*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_BISSC0_BOOSTER_PACK)
#if (CONFIG_BISSC0_CHANNEL0)
    GPIO_setDirMode(ENC0_EN_BASE_ADDR, ENC0_EN_PIN, ENC0_EN_DIR);
    GPIO_pinWriteHigh(ENC0_EN_BASE_ADDR, ENC0_EN_PIN);
#endif
#if (CONFIG_BISSC0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
#endif

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS and BiSS-C driver                              */
    /* ========================================================================== */

    /* Get and display BiSS-C firmware version from PRU firmware image */
    bissc_display_fw_version();

    /* Initialize PRU-ICSS instance, initialize DRAM, and disable PRU cores */
    bissc_pruicss_init();

    /* Run the loop for all BiSS-C instances defined in SysConfig
     * ASSUMPTIONS:
     *      - i = 0 will use CONFIG_BISSC0
     *      - i = 1 will use CONFIG_BISSC1 when BISSC_DUAL_PRU_SLICE_ENABLE is defined
     */
    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        /* Initialize BiSS-C parameters with defaults and set PRU-ICSS handle */
        bissc_params_init(&bissc_params_instance);
        /* Default delay values are used:
         *   - cmd_process_delay_us = 1000us (delay for command processing polling loop)
         *   - fw_wait_delay_us = 1000us (delay for firmware status checks)
         *   - max_wait_loop_count = 5 (maximum wait loop count, 5ms with default cmd_process_delay_us)
         * If needed, these can be modified before calling bissc_init():
         *   bissc_params_instance.cmd_process_delay_us = <custom_value>;
         *   bissc_params_instance.fw_wait_delay_us = <custom_value>;
         *   bissc_params_instance.max_wait_loop_count = <custom_value>;
         */
        bissc_params_instance.pruicss_handle = gPruIcssXHandle;

        /* Initialize BiSS-C driver instance
         * This calls: bissc_hw_init(), bissc_config_channel(), bissc_config_load_share(),
         * bissc_set_default_initialization() and bissc_config_host_trigger().
         */
        gAppBisscHandle[i] = bissc_init(i, &bissc_params_instance);

        DebugP_log("\r\n|------------------------------------------------------------------------------|");
        DebugP_log("\r\n BiSS-C Instance %u", i);

        if(gAppBisscHandle[i] == NULL)
        {
            DebugP_log("\r\nERROR: BiSS-C initialization failed\n");
            return;
        }

        /* Get pointer to BiSS-C attrs and priv */
        attrs[i] = bissc_get_attrs(gAppBisscHandle[i]);
        priv[i] = bissc_get_priv(gAppBisscHandle[i]);

        if(attrs[i] == NULL || priv[i] == NULL)
        {
            DebugP_log("\r\nERROR: BiSS-C initialization failed\n");
            return;
        }

        /* Display HW instances used, operation mode and enabled channels */
        DebugP_log("\r\n PRU-ICSS instance: %u, PRU-ICSS slice number: %u\n", attrs[i]->pruicss_instance, attrs[i]->pruicss_slice);
        if(attrs[i]->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
        {
            /* Multi-channel multi-PRU (load share) mode: Each channel uses a separate PRU core */
            DebugP_log("\r\nBiSS-C Load Share Demo application is running......\n");
            for(ch_num = 0; ch_num < attrs[i]->total_channels; ch_num++)
            {
                DebugP_log("\r\nChannel %u is enabled\n", priv[i]->channel[ch_num]);
            }
        }
        else if(attrs[i]->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
        {
            /* Multi-channel single PRU mode: Multiple channels handled by one PRU core */
            DebugP_log("\r\nBiSS-C Multi channel, Single PRU Demo application is running......\n");
            for(ch_num = 0; ch_num < attrs[i]->total_channels; ch_num++)
            {
                DebugP_log("\r\nChannel %u is enabled\n", priv[i]->channel[ch_num]);
            }
        }
        else
        {
            /* Single channel single PRU mode: One channel on one PRU core */
            DebugP_log("\r\nBiSS-C Single channel, Single PRU Demo application is running......\n");
            DebugP_log("\r\nChannel %u is enabled\n", priv[i]->channel[0]);
        }

        /* ========================================================================== */
        /* STEP 3: Get encoder resolution parameters from user                        */
        /* ========================================================================== */
        /* Prompt user for encoder resolution (single-turn and multi-turn bit lengths)
        * Calls: bissc_clear_data_len() once and bissc_update_data_len() per channel */
        bissc_get_enc_data_len(gAppBisscHandle[i]);

        DebugP_log("\r\n|------------------------------------------------------------------------------|\n\n");
    }
    /* ========================================================================== */
    /* STEP 4: Load and run PRU firmware                                          */
    /* ========================================================================== */
    /* Load PRU firmware image, set default initialization parameters, wait for
     * firmware init. */
    bissc_pruicss_load_run_fw();

    DebugP_log("\r\n PRU-ICSS firmware loading is complete\n");

    /* Run the loop for all BiSS-C instances defined in SysConfig
     * ASSUMPTIONS:
     *      - i = 0 will use CONFIG_BISSC0
     *      - i = 1 will use CONFIG_BISSC1 when BISSC_DUAL_PRU_SLICE_ENABLE is defined
     */
    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        /* check initialization ack from firmware, with a timeout of 5 second */
        ret = bissc_wait_for_fw_initialization(gAppBisscHandle[i], WAIT_5_SECOND);

        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r\nERROR: BiSS-C initialization failed for BiSS-C instance %u\n", i);
            DebugP_log("\r\nCheck whether encoder is connected and ensure proper connections\n");
            DebugP_log("\r\nExit %s due to failed firmware initialization\n", __func__);
            goto deinit;
        }

        /* =============================================================================== */
        /* STEP 5: Get and validate encoder processing delays                              */
        /* =============================================================================== */
        /* In multi-channel single PRU mode, all encoders must have the same processing
        * delay for proper synchronization. If delays differ, operation is not supported. */

        /* Copy measured processing delays from PRU firmware to private driver structure */
        ret = bissc_get_enc_proc_delay(gAppBisscHandle[i]);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r\nERROR: Failed to get encoder processing delay for BiSS-C instance %u\n", i);
            goto deinit;
        }

        if(attrs[i]->mode == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
        {
            if(attrs[i]->total_channels > 1)
            {
                if(priv[i]->proc_delay[priv[i]->channel[0]] == priv[i]->proc_delay[priv[i]->channel[1]])
                {
                    if(attrs[i]->total_channels > 2)
                    {
                        if(priv[i]->proc_delay[priv[i]->channel[1]] !=  priv[i]->proc_delay[priv[i]->channel[2]])
                        {
                            DebugP_log("\r\n Encoders connected accross channels for BiSS-C instance %u have different processing delays, multi channel configuration is not supported\n", i);
                            goto deinit;
                        }
                    }
                }
                else
                {
                    DebugP_log("\r\n Encoders connected accross channels for BiSS-C instance %u have different processing delays, multi channel configuration is not supported\n", i);
                    goto deinit;
                }
            }
        }


        /* Display encoder detection status and processing delays */
        DebugP_log("\r\nBiSS-C encoder/encoders detected and running at frequency %u MHz for BiSS-C instance %u\n", attrs[i]->baud_rate, i);
        for(ch_num = 0; ch_num < attrs[i]->total_channels; ch_num++)
        {
            DebugP_log("\r\nProcessing Delay in clock cycles for BiSS-C instance %u channel %u : %u\n", i, priv[i]->channel[ch_num], priv[i]->proc_delay[priv[i]->channel[ch_num]]);
        }

    }
    /* ========================================================================== */
    /* STEP 6: Interactive menu loop for encoder operations                       */
    /* ========================================================================== */

#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
    DebugP_log("\r\n| In host trigger mode, same command will be run on both BiSS-C instances      |");
    DebugP_log("\r\n| one after the other.                                                         |\n\n");
#endif

    while(1)
    {
        int32_t cmd, ret;
        uint64_t trigger_count[CONFIG_BISSC_NUM_INSTANCES][BISSC_NUM_CH_PER_SLICE_MAX]={0}, iep_reset_count = 0;
        uint8_t is_cap_mode = 0;
        uint32_t freq, ctrl_cmd[BISSC_NUM_CH_PER_SLICE_MAX]={0};
        uint32_t loop_cnt;
        uint32_t safety = 0;
        uint32_t ctrl_write_status, ctrl_reg_address, ctrl_reg_data = 0, ctrl_enc_id = 0;
        /* Display menu and get user command */
        bissc_display_menu();
        cmd = bissc_get_command();

        if(cmd < BISSC_CMD_EXIT_APP)
        {
            /* Invalid command, show menu again */
            continue;
        }
        else if(cmd == BISSC_CMD_EXIT_APP)
        {
            /* Exit application */
            DebugP_log("\r\tExiting application!\n");
            break;
        }
        else if(cmd == BISSC_CMD_ENC_LEN_UPDATE)
        {
           /* Update encoder resolution parameters
            * Calls: bissc_clear_data_len() once and bissc_update_data_len() per channel */
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                bissc_get_enc_data_len(gAppBisscHandle[i]);
            }
        }
        else if(cmd == BISSC_CMD_ENC_FREQ_UPDATE)
        {
            /* Change BiSS-C communication frequency (1/2/5/8/10 MHz)
             * Calls: bissc_clock_config() which internally calls multiple functions */
            DebugP_log("\r\nPlease enter frequency in MHz:\n");
            DebugP_scanf("%u\n", &freq);

            /* Validate frequency (only 1, 2, 5, 8, 10 MHz supported) */
            if(!((freq == BISSC_FREQ_1MHZ) || (freq == BISSC_FREQ_2MHZ) || (freq == BISSC_FREQ_5MHZ) || (freq == BISSC_FREQ_8MHZ) || (freq == BISSC_FREQ_10MHZ)))
            {
                DebugP_log("\r\nClock divisors will not be possible. Please provide valid frequency: 1/2/5/8/10\n");
                continue;
            }

            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                /* Reconfigure clock frequency and wait for processing delay measurement */
                ret = bissc_clock_config(gAppBisscHandle[i], freq, WAIT_5_SECOND);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\nERROR: Processing time measurement failed for BiSS-C instance %u\n", i);
                    DebugP_log("\r\nCheck whether encoder is connected and ensure proper connections\n");
                    DebugP_log("\r\nExiting application!\n");
                    break;
                }
            }
            ClockP_sleep(BISSC_CLOCK_CONFIG_DELAY_SEC);
        }
        else if(cmd == BISSC_CMD_ENC_SEND_POS)
        {
            /* Get single-shot position data from encoder(s)
             * Calls: bissc_command_process() which internally calls
             *        bissc_command_send() and bissc_command_wait() */
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                ret = bissc_get_pos(gAppBisscHandle[i]);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Position data measurement failed \n");
                }
                for(ch_num = 0; ch_num < attrs[i]->total_channels; ch_num++)
                {
                    ch = bissc_get_current_channel(gAppBisscHandle[i], ch_num);
                    if(attrs[i]->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                        ls_ch = ch;
                    else
                        ls_ch = 0;
                    DebugP_log("\r\n Channel %u:\n", ch);
                    if(gAppBisscHandle[i]->priv->multi_turn_len[ls_ch][0])
                    {
                        if(gAppBisscHandle[i]->priv->has_safety[ls_ch][0])
                        {
                            DebugP_log("\r\n Encoder-1 Multiturn rev: %u, Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[0],
                            gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[0], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][0], gAppBisscHandle[i]->priv->calc_safety_crc[ch][0], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[0], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][0]);
                        }
                        else
                        {
                            DebugP_log("\r\n Encoder-1 Multiturn rev: %u, Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[0],
                            gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[0], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[0], gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[0], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[0]);
                        }
                    }
                    else
                    {
                        if(gAppBisscHandle[i]->priv->has_safety[ls_ch][0])
                        {
                            DebugP_log("\r\n Encoder-1 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                            gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[0], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][0], gAppBisscHandle[i]->priv->calc_safety_crc[ch][0], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[0], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][0]);
                        }
                        else
                        {
                            DebugP_log("\r\n Encoder-1 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[0], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[0],
                            gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[0], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[0]);
                        }
                    }
                    if(gAppBisscHandle[i]->priv->has_safety[ls_ch][0])
                    {
                        DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->rcv_safety_crc[ch][0] == gAppBisscHandle[i]->priv->calc_safety_crc[ch][0]) ? "success" : "failure", gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][0]);
                    }
                    else
                    {
                        DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[0] == gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[0]) ? "success" : "failure",
                        gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][0]);
                    }
                    if(gAppBisscHandle[i]->priv->data_len[ls_ch][1])
                    {
                        if(gAppBisscHandle[i]->priv->multi_turn_len[ls_ch][1])
                        {
                            if(gAppBisscHandle[i]->priv->has_safety[ls_ch][1])
                            {
                                DebugP_log("\r\n Encoder-2 Multiturn rev: %u, Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[1],
                                gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[1], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][1], gAppBisscHandle[i]->priv->calc_safety_crc[ch][1], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[1], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][1]);
                            }
                            else
                            {
                                DebugP_log("\r\n Encoder-2 Multiturn rev: %u, Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[1],
                                gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[1], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[1], gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[1], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[1]);
                            }
                        }
                        else
                        {
                            if(gAppBisscHandle[i]->priv->has_safety[ls_ch][1])
                            {
                                DebugP_log("\r\n Encoder-2 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                                gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[1], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][1], gAppBisscHandle[i]->priv->calc_safety_crc[ch][1], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[1], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][1]);
                            }
                            else
                            {
                                DebugP_log("\r\n Encoder-2 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[1], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[1],
                                gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[1], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[1]);
                            }
                        }
                        if(gAppBisscHandle[i]->priv->has_safety[ls_ch][1])
                        {
                            DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->rcv_safety_crc[ch][1] == gAppBisscHandle[i]->priv->calc_safety_crc[ch][1]) ? "success" : "failure", gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][1]);
                        }
                        else
                        {
                            DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[1] == gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[1]) ? "success" : "failure",
                            gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][1]);
                        }
                        if(gAppBisscHandle[i]->priv->data_len[ls_ch][2])
                        {
                            if(gAppBisscHandle[i]->priv->multi_turn_len[ls_ch][2])
                            {
                                if(gAppBisscHandle[i]->priv->has_safety[ls_ch][2])
                                {
                                    DebugP_log("\r\n Encoder-3 Multiturn rev: %u, Angle:  %.12f, received safety crc: 0x%x, calculated safety crc: 0x%x, e_w: 0x%x, sign-of-life counter: %d\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[2],
                                    gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[2], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][2], gAppBisscHandle[i]->priv->calc_safety_crc[ch][2], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[2], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][2]);
                                }
                                else
                                {
                                    DebugP_log("\r\n Encoder-3 Multiturn rev: %u, Angle:  %.12f, crc: 0x%x, otf crc: 0x%x, e_w: 0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].num_of_turns[2],
                                    gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[2], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[2], gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[2], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[2]);
                                }
                            }
                            else
                            {
                                if(gAppBisscHandle[i]->priv->has_safety[ls_ch][2])
                                {
                                    DebugP_log("\r\n Encoder-3 Singleturn Angle:  %.12f, received safety crc:0x%x, calculated safety crc:0x%x, e_w:0x%x, sign-of-life counter: %d\n",
                                    gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[2], gAppBisscHandle[i]->priv->rcv_safety_crc[ch][2], gAppBisscHandle[i]->priv->calc_safety_crc[ch][2], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[2], gAppBisscHandle[i]->priv->sign_of_life_cnt[ch][2]);
                                }
                                else
                                {
                                    DebugP_log("\r\n Encoder-3 Singleturn Angle:  %.12f, crc:0x%x, otf crc:0x%x, e_w:0x%x\n", gAppBisscHandle[i]->priv->enc_pos_data[ch].angle[2], gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[2],
                                    gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[2], gAppBisscHandle[i]->priv->enc_pos_data[ch].ew[2]);
                                }
                            }
                            if(gAppBisscHandle[i]->priv->has_safety[ls_ch][2])
                            {
                                DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->rcv_safety_crc[ch][2] == gAppBisscHandle[i]->priv->calc_safety_crc[ch][2]) ? "success" : "failure",gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][2]);
                            }
                            else
                            {
                                DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (gAppBisscHandle[i]->priv->enc_pos_data[ch].rcv_crc[2] == gAppBisscHandle[i]->priv->enc_pos_data[ch].otf_crc[2]) ? "success" : "failure" ,
                                gAppBisscHandle[i]->priv->pd_crc_err_cnt[ch][2]);
                            }
                        }
                    }
                }
            }
        }
        else if(cmd == BISSC_CMD_ENC_CTRL_CMD)
        {
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                DebugP_log("\r\nPlease enter control communication details for BiSS-C instance %u:\n", i);
                total_channels = bissc_get_total_channels(gAppBisscHandle[i]);
                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ctrl_reg_data = 0;
                    ctrl_enc_id = 0;
                    if(attrs[i]->mode == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                    {
                        ls_ch = bissc_get_current_channel(gAppBisscHandle[i], ch_num);
                        DebugP_log("\r\n Channel %u: ",ls_ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        total_channels = 1;
                    }
                    DebugP_log("\r\n Enter type of access(0: Read & 1: Write): ");
                    DebugP_scanf("%x\n", &ctrl_write_status);
                    while(1)
                    {
                        DebugP_log("\r\n Enter Register Address(in hex): ");
                        DebugP_scanf("%x\n", &ctrl_reg_address);
                        if(ctrl_reg_address > BISSC_CTRL_REG_ADDR_MASK)
                            DebugP_log("\r\n Please enter a 7-bit address\n");
                        else
                            break;
                    }
                    if(ctrl_write_status == 1)
                    {
                        DebugP_log("\r\n Enter Data(in hex) to write at 0x%x Register\n", ctrl_reg_address);
                        DebugP_scanf("%x\n", &ctrl_reg_data);
                    }
                    DebugP_log("\r\n Enter Encoder ID(0: if daisy chain is not in use)\n");
                    DebugP_scanf("%x\n", &ctrl_enc_id);
                    ctrl_cmd[ls_ch] = bissc_generate_ctrl_cmd(gAppBisscHandle[i], ls_ch, ctrl_write_status, ctrl_reg_address, ctrl_reg_data, ctrl_enc_id);
                }
                ret = bissc_set_ctrl_cmd_and_process(gAppBisscHandle[i], ctrl_cmd);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Control communication failed for BiSS-C instance %u:\n", i);
                }
                for(ch_num = 0; ch_num < attrs[i]->total_channels; ch_num++)
                {
                    ch = bissc_get_current_channel(gAppBisscHandle[i], ch_num);
                    DebugP_log("\r\n Channel %u:\n", ch);
                    DebugP_log("\r\n Control communication result: 0x%x, crc: 0x%x, otf crc: 0x%x, status: %s\n",gAppBisscHandle[i]->priv->enc_ctrl_data[ch].cmd_result,
                    gAppBisscHandle[i]->priv->enc_ctrl_data[ch].cmd_rcv_crc, gAppBisscHandle[i]->priv->enc_ctrl_data[ch].cmd_otf_crc,
                    (gAppBisscHandle[i]->priv->enc_ctrl_data[ch].cmd_rcv_crc == gAppBisscHandle[i]->priv->enc_ctrl_data[ch].cmd_otf_crc) ? "success" : "failure");

                    DebugP_log("\r\n CTRL CRC error count: %u\n", gAppBisscHandle[i]->priv->ctrl_crc_err_cnt[ch]);
                }
            }
        }
        else if(cmd == BISSC_CMD_ENC_LOOP_OVER_CYC)
        {
            DebugP_log("\r\n Enter number of BiSS-C cycles:\n");
            DebugP_scanf("%u\n", &loop_cnt);
            if(loop_cnt)
            {
                do
                {
                    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
                    {
                        ret = bissc_get_pos(gAppBisscHandle[i]);
                        if(ret != SystemP_SUCCESS)
                        {
                            DebugP_log("\r\n ERROR: Position data measurement failed for BiSS-C instance %u:\n", i);
                        }
                        bissc_print_res(gAppBisscHandle[i]);
                    }
                    loop_cnt--;
                }
                while(loop_cnt);
            }
            else
            {
                DebugP_log("\r\nPlease enter non-zero value\n");
            }
        }
        else if(cmd == BISSC_CMD_PERIODIC_TRIGGER_CMP)
        {
            is_cap_mode = 0;
#if defined(BISSC_DUAL_PRU_SLICE_ENABLE)
            DebugP_log("\r\n IEP is common for both BiSS-C instances. Same IEP periodic cycle will be used for both instances.\n");
#endif
            DebugP_log("\r| Enter IEP cycle count(must be greater than BiSS cycle time including timeout period, in IEP cycles): ");
            DebugP_scanf("%llu\n", &iep_reset_count);
            if(iep_reset_count <= BISSC_IEP_COUNTER_INCREMENT)
            {
                DebugP_log("\r\n| ERROR: Invalid value entered\n");
                continue;
            }

            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                if(attrs[i]->load_share_enabled)
                {
                    if(attrs[i]->channel0_enabled)  /* Channel 0 */
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) for channel 0: \n");
                        DebugP_scanf("%llu\n", &trigger_count[i][0]);
                        if((trigger_count[i][0] > iep_reset_count) || (trigger_count[i][0] <= BISSC_IEP_COUNTER_INCREMENT))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }

                    if(attrs[i]->channel1_enabled)  /* Channel 1 */
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) for channel 1: \n");
                        DebugP_scanf("%llu\n", &trigger_count[i][1]);
                        if((trigger_count[i][1] > iep_reset_count) || (trigger_count[i][1] <= BISSC_IEP_COUNTER_INCREMENT))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }
                    if(attrs[i]->channel2_enabled)  /* Channel 2 */
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) for channel 2: \n");
                        DebugP_scanf("%llu\n", &trigger_count[i][2]);
                        if((trigger_count[i][2] > iep_reset_count) || (trigger_count[i][2] <= BISSC_IEP_COUNTER_INCREMENT))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }

                }
                else
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
                    DebugP_scanf("%llu\n", &trigger_count[i][0]);
                    if((trigger_count[i][0] > iep_reset_count) || (trigger_count[i][0] <= BISSC_IEP_COUNTER_INCREMENT))
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        continue;
                    }
                }
            }
            DebugP_log("\r\n Switching to periodic trigger mode\n");

            /* Switching to periodic mode using bissc_config_periodic_trigger_cmp_mode() is done
             * inside bissc_process_periodic_command */

            bissc_process_periodic_command(gAppBisscHandle, trigger_count, iep_reset_count, is_cap_mode);

            /* Switch back to host trigger mode for menu-driven operation */
            DebugP_log("\r\n Switching to host trigger mode\n");

            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                if(bissc_config_host_trigger(gAppBisscHandle[i]) != SystemP_SUCCESS)
                {
                    /* NOTE: If this fails, driver may remain in periodic mode causing subsequent
                    * host-triggered commands to fail. */
                    DebugP_log("\r| ERROR: Failed to revert to host trigger for BiSS-C instance %u\n", i);
                }
            }
        }
        else if(cmd == BISSC_ENABLE_SAFETY)
        {
            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                /* Clear all previously enabled safety flags */
                ret = bissc_disable_safety(gAppBisscHandle[i]);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Failed to disable safety for BiSS-C instance %u\n", i);
                    continue;
                }
                total_channels = bissc_get_total_channels(gAppBisscHandle[i]);
                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    if(attrs[i]->load_share_enabled)
                    {
                        ls_ch = bissc_get_current_channel(gAppBisscHandle[i], ch_num);
                    }
                    else
                    {
                        total_channels = 1;
                        ls_ch = 0;
                    }
                    DebugP_log("\r\n Enter 0 to Disable Safety or 1 to Enable Safety for Channel %u:\n", gAppBisscHandle[i]->priv->channel[ch_num]);
                    for(enc_num = 0; enc_num < gAppBisscHandle[i]->priv->num_encoders[ls_ch]; enc_num++)
                    {
                        DebugP_log("\r\nPlease enter encoder %d safety status\n", enc_num);
                        DebugP_scanf("%d", &safety);
                        if(safety == 1)
                        {
                            ret = bissc_enable_safety(gAppBisscHandle[i], enc_num, ls_ch);
                            if(ret != SystemP_SUCCESS)
                            {
                                DebugP_log("\r| ERROR: Failed to enable safety for encoder %d, channel %u for BiSS-C instance %u\n", enc_num, ls_ch, i);
                            }
                        }
                    }
                }
            }
        }
        else if(cmd == BISSC_CMD_PERIODIC_TRIGGER_CAP)
        {
            is_cap_mode = 1;
#if defined(SOC_AM243X) 
            DebugP_log("\r| Enter IEP SYNC0 period (in IEP cycles, used for CAP mode):");
            DebugP_scanf("%llu\n", &iep_reset_count);
            if(iep_reset_count <= BISSC_IEP_COUNTER_INCREMENT)
            {
                DebugP_log("\r\n| ERROR: Invalid value entered\n");
                continue;
            }       
#else
            DebugP_log("\r| Periodic CAP mode cycle time will be equal to EPWM frequency. NOTE: In SysConfig, EPWM and EPWM to IEP LATCH XBAR configuration must be done. \n|\n|\n|\n");
            
#endif
            DebugP_log("\r\n Switching to periodic trigger mode \n");

            /* Switching to periodic mode using bissc_config_periodic_trigger_cap_mode() is done
             * inside bissc_process_periodic_command */

            bissc_process_periodic_command(gAppBisscHandle, trigger_count, iep_reset_count, is_cap_mode);

            /* Switch back to host trigger mode for menu-driven operation */
            DebugP_log("\r\n Switching to host trigger mode\n");

            for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
            {
                if(bissc_config_host_trigger(gAppBisscHandle[i]) != SystemP_SUCCESS)
                {
                    /* NOTE: If this fails, driver may remain in periodic mode causing subsequent
                    * host-triggered commands to fail. */
                    DebugP_log("\r| ERROR: Failed to revert to host trigger for BiSS-C instance %u\n", i);
                }
            }
        }
        
    }
deinit:

    for(i = 0; i < CONFIG_BISSC_NUM_INSTANCES; i++)
    {
        bissc_deinit(gAppBisscHandle[i]);
    }

    Board_driversClose();
    Drivers_close();
    return;
}
