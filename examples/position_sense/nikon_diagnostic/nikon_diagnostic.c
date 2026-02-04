/*
 *  Copyright (C) 2024-2026 Texas Instruments Incorporated
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
 * \file  nikon_diagnostic.c
 *
 * \brief Nikon encoder diagnostic application
 *
 * \details This application provides comprehensive diagnostic and testing capabilities for
 *          Nikon encoders, including position reads, EEPROM operations, continuous position monitoring,
 *          and periodic trigger mode.
 *
 * \par Application Flow:
 * 1. Initialize SoC drivers and board drivers
 * 2. Wait for encoder power-up (per encoder specification)
 * 3. Display firmware version information
 * 4. Initialize PRU-ICSS subsystem
 * 5. Initialize Nikon driver with default parameters
 * 6. Get encoder resolution parameters from user
 * 7. Load and run PRU firmware(s) - driver configures the default host trigger mode
 * 8. Wait for encoder detection
 * 9. Enter interactive menu loop for encoder operations
 * 10. De-initialize on exit
 *
 * \par Trigger Modes:
 * - **Host Trigger Mode (default)**: Each encoder transaction is initiated by the host (R5F)
 *   via API calls. The driver configures this mode (as default) during initialization.
 * - **Periodic Trigger Mode**: encoder transactions are automatically triggered by IEP timer
 *   at regular intervals. This mode can be enabled through the interactive menu.
 *
 * \par Supported Operations:
 * - Position data acquisition
 * - Bus mode operation
 * - Multi-turn and single-turn data
 * - Encoder status and alarm monitoring
 * - EEPROM read/write operations
 * - Temperature reading
 * - Identification code operations
 * - Velocity and acceleration data (Nikon 3.0)
 * - Encoder address configuration
 * - Frequency configuration
 * - Continuous/periodic mode operation
 *
 * \par Driver Validation Strategy:
 * Nikon driver APIs use following validation approach:
 * - **Handle validation**: All public APIs validate the handle parameter for NULL
 * - **Array bounds checking**: APIs with array/index parameters perform bounds validation
 * - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *   are validated for NULL before dereferencing to prevent undefined behavior
 * - **Error handling**: Functions returning data return 0 on validation failures. Functions returning
 *   status use SystemP_SUCCESS/SystemP_FAILURE. Caller is responsible for explicit state cleanup if
 *   needed after errors (e.g., nikon_get_pos may leave internal state partially modified on failure).
 * - Check API documentation for more details.
 *
 * \par Dual Slice Support (tested only on AM261x with Single PRU Single Channel mode):
 * This diagnostic application supports dual PRU slice operation when compiled with
 * NIKON_DUAL_PRU_SLICE_ENABLE defined. In dual slice mode:
 * - Two independent Nikon instances (CONFIG_NIKON0, CONFIG_NIKON1) run simultaneously
 * - Each instance operates on a different PRU slice (PRU0 or PRU1)
 * - Both instances must use the same PRU-ICSS instance (validated at runtime)
 * - Both instances share common PRU-ICSS level resources
 *
 * \par Instance vs Slice:
 * - Instance: Software driver instance (CONFIG_NIKON0, CONFIG_NIKON1) with independent
 *   configuration, state, and driver handle. Configured via SysConfig.
 * - Slice: Hardware PRU-ICSS slice number (0 or 1) within PRU-ICSS where firmware executes.
 *   CONFIG_NIKONx_PRUICSS_SLICE specifies which slice each instance uses.
 *
 * \par Driver Handle Array:
 * The global handle array gAppNikonHandle[CONFIG_NIKON_NUM_INSTANCES] stores driver
 * handles for all configured instances:
 * - gAppNikonHandle[CONFIG_NIKON0]: Handle for first instance (always present)
 * - gAppNikonHandle[CONFIG_NIKON1]: Handle for second instance (tested only on AM261x with Single PRU Single Channel mode)
 * - CONFIG_NIKON_NUM_INSTANCES: Number of instances (defined by SysConfig)
 * ASSUMPTIONS: Loop (with index i) is used to call functions with handle as argument.
 *      - i = 0 will use CONFIG_NIKON0
 *      - i = 1 will use CONFIG_NIKON1 when NIKON_DUAL_PRU_SLICE_ENABLE is defined
 *
 * \par Shared Resources:
 * When both instances use the same PRU-ICSS instance, they share:
 * - IEP Timer: Used for periodic trigger mode, configured via first instance
 * - PRU-ICSS INTC: Interrupt controller shared across slices
 *
 * \par First instance (CONFIG_NIKON0) is used for shared resources:
 * Several operations use gAppNikonHandle[CONFIG_NIKON0] to access shared PRU-ICSS
 * resources. This approach works correctly because validation in nikon_pruicss_init()
 * ensures both instances use the same PRU-ICSS instance.
 *
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
#include <position_sense/nikon/include/nikon_drv.h>
#include <position_sense/nikon/include/nikon_api.h>
#include "nikon_periodic_trigger.h"

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
/* Multi-channel single PRU mode */
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
#include  <nikon_receiver_multi_pru1_bin.h>
#else
#include  <nikon_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
/* Multi-channel multi-PRU mode (load share mode) */
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
#include <nikon_receiver_multi_rtu_pru1_bin.h>
#include <nikon_receiver_multi_pru1_bin.h>
#include <nikon_receiver_multi_tx_pru1_bin.h>
#else
#include <nikon_receiver_multi_rtu_pru0_bin.h>
#include <nikon_receiver_multi_pru0_bin.h>
#include <nikon_receiver_multi_tx_pru0_bin.h>
#endif
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
/* Single channel single PRU mode */
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
#include  <nikon_receiver_pru1_bin.h>
#else
#include  <nikon_receiver_pru0_bin.h>
#endif
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
/* Single channel mode firmware */
#if (CONFIG_NIKON1_PRUICSS_SLICE == 1)
#include  <nikon_receiver_pru1_bin.h>
#else
#include  <nikon_receiver_pru0_bin.h>
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

#define NIKON_CMD_EXIT_APP                  (0)
#define NIKON_CMD_ENC_LEN_UPDATE            (1)
#define NIKON_CMD_ENC_FREQ_UPDATE           (2)
#define NIKON_CMD_ENC_SEND_POS              (3)
#define NIKON_CMD_ENC_LOOP_OVER_CYC         (4)
#define NIKON_CMD_PERIODIC_TRIGGER          (5)

#define NIKON_POSITION_LOOP_STOP            0
#define NIKON_POSITION_LOOP_START           1

/* Macro for 0.5 seconds delay - value in micro-seconds */
#define NIKON_POWER_UP_DELAY (0.5 * 1000 * 1000)

#define NIKON_PERIODIC_MODE_POLL_SLEEP_US   (1)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* Nikon Driver Handle */
nikon_handle gAppNikonHandle[CONFIG_NIKON_NUM_INSTANCES] = {NULL};

/* Nikon Periodic Interface Struct Instance */
nikon_periodic_interface gNikonPeriodicInterface;

/* Global variable to track position loop status */
volatile int32_t gNikonPositionLoopStatus;

/* IRQ count from periodic trigger (defined in nikon_periodic_trigger.c) */
extern volatile uint32_t gPruNikonIrqCnt[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX];

/* Task related global variables */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void nikon_pruicss_init(void);
static void nikon_pruicss_load_run_fw(void);
static void nikon_get_enc_data_len(nikon_handle handle);
static void nikon_display_fw_version(void);
static void nikon_display_menu(nikon_handle handle, uint32_t instance);
static uint32_t nikon_get_command(nikon_handle handle);
static int32_t nikon_handle_command(nikon_handle handle, uint32_t cmd);
static void nikon_position_loop_decide_termination(void *args);
static int32_t nikon_loop_task_create(void);
static int32_t nikon_process_periodic_command(nikon_handle handle[CONFIG_NIKON_NUM_INSTANCES], uint64_t trigger_count[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX], uint64_t iep_reset_count, uint8_t is_cap_mode);
void nikon_main(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void nikon_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    uint8_t rtu_pru_id = CONFIG_NIKON0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    uint8_t tx_pru_id = CONFIG_NIKON0_PRUICSS_TX_PRU_ID;
#endif
#else
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#endif

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    if(gPruIcssXHandle == NULL)
    {
        DebugP_log("\r\n ERROR: PRUICSS_open failed - NULL handle returned\n");
        DebugP_assert(0);
    }

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
#if !defined(SOC_AM261X) || (CONFIG_NIKON0_MODE != NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU) || (CONFIG_NIKON1_MODE != NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    DebugP_log("Dual handle example using PRU0 and PRU1 is tested only with NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other combinations, update code and remove this check.");
    DebugP_assert(0);
#endif
    /*
     * These checks are applicable only if both Nikon instances
     * use same PRU-ICSSG instance. If different instances are used,
     * these checks can be removed.
     */
#if (CONFIG_NIKON0_PRUICSS_INSTANCE != CONFIG_NIKON1_PRUICSS_INSTANCE)
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Nikon handles in this example */
    DebugP_log("\r\n PRU-ICSS instance should be set uniformly for both Nikon instances in SysConfig.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_NIKON0_G_MUX_EN) && !defined(CONFIG_NIKON1_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both Nikon instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_NIKON1_G_MUX_EN) && !defined(CONFIG_NIKON0_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both Nikon instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif
#endif

#ifdef CONFIG_NIKON0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Clear PRU-ICSS DATA RAM for slice*/
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_NIKON0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    pru_id = CONFIG_NIKON1_PRUICSS_PRU_ID;

    /* Clear PRU-ICSS DATA RAM for slice*/
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_NIKON1_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    /*
     * Set the constant table C28 for TXPRU
     * configuring the constant table C28 to point to the TX counter
     * register (CNTR). The counter is needed in firmware for adding waits and time stamps.
     */
#if (CONFIG_NIKON0_PRUICSS_INSTANCE == 1)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, tx_pru_id, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, tx_pru_id, PRUICSS_CONST_TBL_ENTRY_C28, 0xA50);
#endif
#else
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, tx_pru_id, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, tx_pru_id, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif
#endif
#endif
#endif
}

static void nikon_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = NikonFirmwareMultiMakeRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(NikonFirmwareMultiMakeRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_NIKON0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = NikonFirmwareMultiMakePru1_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwareMultiMakePru1_0);
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = NikonFirmwareMultiMakeTxPru1_0;
    uint32_t tx_pru_firmware_size = sizeof(NikonFirmwareMultiMakeTxPru1_0);
    uint8_t tx_pru_id = CONFIG_NIKON0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = NikonFirmwareMultiMakeRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(NikonFirmwareMultiMakeRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_NIKON0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = NikonFirmwareMultiMakePru0_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwareMultiMakePru0_0);
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = NikonFirmwareMultiMakeTxPru0_0;
    uint32_t tx_pru_firmware_size = sizeof(NikonFirmwareMultiMakeTxPru0_0);
    uint8_t tx_pru_id = CONFIG_NIKON0_PRUICSS_TX_PRU_ID;
#endif
#endif
#elif (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = NikonFirmwareMultiPru1_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwareMultiPru1_0);
#else
    const uint32_t *pru_firmware = NikonFirmwareMultiPru0_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwareMultiPru0_0);
#endif
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#else
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = NikonFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwarePru1_0);
#else
    const uint32_t *pru_firmware = NikonFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(NikonFirmwarePru0_0);
#endif
    uint8_t pru_id = CONFIG_NIKON0_PRUICSS_PRU_ID;
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
    /* Disable RTU-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to RTU-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_NIKON0_PRUICSS_SLICE), 0,
                                  (uint32_t *)rtu_pru_firmware, rtu_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset RTU-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable RTU-PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
    /* Disable TX-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to TX-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_NIKON0_PRUICSS_SLICE), 0,
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
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_NIKON1_PRUICSS_SLICE == 1)
    pru_firmware = NikonFirmwarePru1_0;
    pru_firmware_size = sizeof(NikonFirmwarePru1_0);
#else
    pru_firmware = NikonFirmwarePru0_0;
    pru_firmware_size = sizeof(NikonFirmwarePru0_0);
#endif
    pru_id = CONFIG_NIKON1_PRUICSS_PRU_ID;

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON1_PRUICSS_SLICE), 0,
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

static void nikon_get_enc_data_len(nikon_handle handle)
{
    int32_t ret;
    uint32_t ch_num;
    uint32_t ch;
    uint32_t enc_num;
    uint32_t num_encoders;
    uint32_t single_turn_len[NUM_ENCODERS_MAX];
    uint32_t multi_turn_len[NUM_ENCODERS_MAX];
    uint32_t total_channels;
    const nikon_attrs *attrs = nikon_get_attrs(handle);

    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\n| ERROR: NULL handle/attrs");
        return;
    }

    total_channels = attrs->total_channels;

    for(ch_num = 0; ch_num < total_channels; ch_num++)
    {
        for(enc_num = 0; enc_num < NUM_ENCODERS_MAX; enc_num++)
        {
            single_turn_len[enc_num] = 0;
            multi_turn_len[enc_num] = 0;
        }

        ret = nikon_get_current_channel(handle, ch_num, &ch);
        DebugP_assert(ret == SystemP_SUCCESS);

        DebugP_log("\r\nPlease enter encoder length connected to Channel %d:\n", ch);

        /* Retry loop for encoder 1 input validation */
        while(1)
        {
            DebugP_log("\r\nPlease enter single turn length for encoder 1: ");
            DebugP_scanf("%u\n", &single_turn_len[0]);
            DebugP_log("\r\nPlease enter multi turn length for encoder 1 (0, if not a multi turn encoder): ");
            DebugP_scanf("%u\n", &multi_turn_len[0]);

            /* Validate first encoder data */
            if(single_turn_len[0] > NIKON_MAX_ABS_LEN)
            {
                DebugP_log("\r\n| ERROR: Invalid single turn length for encoder 1. Valid range: 0-%u\n", NIKON_MAX_ABS_LEN);
                continue;
            }
            if(multi_turn_len[0] > NIKON_MAX_ABS_LEN)
            {
                DebugP_log("\r\n| ERROR: Invalid multi turn length for encoder 1. Valid range: 0-%u\n", NIKON_MAX_ABS_LEN);
                continue;
            }
            if((single_turn_len[0] + multi_turn_len[0]) > NIKON_MAX_ABS_LEN)
            {
                DebugP_log("\r\n| ERROR: Total encoder length exceeds maximum. Single + Multi turn must be <= %u bits\n", NIKON_MAX_ABS_LEN);
                continue;
            }

            /* Validation passed, break out of retry loop */
            break;
        }

        num_encoders = 1;

        for(enc_num = 1; enc_num < NUM_ENCODERS_MAX; enc_num++)
        {
            /* Nested retry loop for single turn length input */
            while(1)
            {
                DebugP_log("\r\nPlease enter single turn length for encoder %d (0, if not connected): ", (enc_num + 1));
                DebugP_scanf("%u\n", &single_turn_len[enc_num]);

                /* Validate single turn length */
                if(single_turn_len[enc_num] > NIKON_MAX_ABS_LEN)
                {
                    DebugP_log("\r\n| ERROR: Invalid single turn length for encoder %d. Valid range: 0-%u\n", (enc_num + 1), NIKON_MAX_ABS_LEN);
                    continue;  /* Retry same encoder */
                }

                /* Valid input, exit retry loop */
                break;
            }

            if(single_turn_len[enc_num] == 0)
            {
                break;
            }

            /* Nested retry loop for multi turn length input */
            while(1)
            {
                DebugP_log("\r\nPlease enter multi turn length for encoder %d (0, if not a multi turn encoder): ", (enc_num + 1));
                DebugP_scanf("%u\n", &multi_turn_len[enc_num]);

                /* Validate multi turn length */
                if(multi_turn_len[enc_num] > NIKON_MAX_ABS_LEN)
                {
                    DebugP_log("\r\n| ERROR: Invalid multi turn length for encoder %d. Valid range: 0-%u\n", (enc_num + 1), NIKON_MAX_ABS_LEN);
                    continue;  /* Retry same encoder */
                }

                /* Validate total length */
                if((single_turn_len[enc_num] + multi_turn_len[enc_num]) > NIKON_MAX_ABS_LEN)
                {
                    DebugP_log("\r\n| ERROR: Total encoder length exceeds maximum. Single + Multi turn must be <= %u bits\n", NIKON_MAX_ABS_LEN);
                    continue;  /* Retry same encoder */
                }

                /* Valid input, exit retry loop */
                break;
            }

            num_encoders++;
        }

        ret = nikon_update_enc_len(handle, num_encoders, single_turn_len, multi_turn_len, ch);
        DebugP_assert(ret == SystemP_SUCCESS);
    }
}

static void nikon_display_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version(s) depending on configuration */
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwareMultiPru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwareMultiPru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware \t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#elif (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_NIKON0_CHANNEL0_ENABLED == 1)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwareMultiMakeRtuPru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwareMultiMakeRtuPru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware for channel 0 (RTU-PRU)\t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_NIKON0_CHANNEL1_ENABLED == 1)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwareMultiMakePru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwareMultiMakePru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware for channel 1 (PRU)\t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_NIKON0_CHANNEL2_ENABLED == 1)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwareMultiMakeTxPru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwareMultiMakeTxPru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware for channel 2 (TX-PRU)\t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#elif (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware for Nikon instance 0\t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif

#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_NIKON0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)NikonFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)NikonFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nNikon firmware for Nikon instance 1\t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
}

static void nikon_display_menu(nikon_handle handle, uint32_t instance)
{
    const nikon_attrs *attrs = nikon_get_attrs(handle);

    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\n| ERROR: NULL handle/attrs");
        return;
    }

    DebugP_log("\r\n|-------------------------------------------------------------------------------------- |");
    DebugP_log("\r\n|                      Select input parameters for instance %u                           |", instance);
    DebugP_log("\r\n|-------------------------------------------------------------------------------------- |");
    if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
    {
        DebugP_log("\r\n| 0 : ABS full 40 bit data request                                                      |");
        DebugP_log("\r\n| 1 : ABS lower 24bit data request / ABS full 40bit data + velocity data request        |");
        DebugP_log("\r\n| 2 : ABS upper 24bit data request                                                      |");
        DebugP_log("\r\n| 3 : Encoder status Request                                                            |");
        DebugP_log("\r\n| 4 : ABS full 40 bit data request(MT)                                                  |");
        DebugP_log("\r\n| 5 : ABS lower 24bit data request(MT) / ABS full 40bit data + velocity data request(MT)|");
        DebugP_log("\r\n| 6 : ABS upper 24bit data request(MT)                                                  |");
        DebugP_log("\r\n| 7 : Encoder status Request(MT)                                                        |");
        DebugP_log("\r\n| 8 : Status flag clear request / ABS lower 24bit data request                          |");
        DebugP_log("\r\n| 9 : Multiple turn data clear request / ABS lower 24bit data request                   |");
        DebugP_log("\r\n| 10: Status + Multiple turn data clear request / ABS lower 24bit data request          |");
        DebugP_log("\r\n| 11: Encoder address setting I (one-to-one connection) / ABS lower 24bit data request  |");
        DebugP_log("\r\n| 12: Single turn data zero preset / ABS lower 24bit data request                       |");
        DebugP_log("\r\n| 13: EEPROM read request                                                               |");
        DebugP_log("\r\n| 14: EEPROM write request                                                              |");
        DebugP_log("\r\n| 15: Temperature data request                                                          |");
        DebugP_log("\r\n| 16: Identification code read I / Velocity coefficient read                            |");
        DebugP_log("\r\n| 17: Identification code read II(one-to-one connection)                                |");
        DebugP_log("\r\n| 18: Identification code write I / Velocity coefficient write                          |");
        DebugP_log("\r\n| 19: Identification code write II(one-to-one connection)                               |");
        DebugP_log("\r\n| 20: Encoder address setting II                                                        |");
        DebugP_log("\r\n| 21: ABS lower 17bit data request                                                      |");
        DebugP_log("\r\n| 22: ABS lower 17bit data request(MT)                                                  |");
        DebugP_log("\r\n| 23: ABS lower 24bit + velocity request                                                |");
        DebugP_log("\r\n| 24: ABS lower 24bit + velocity request(MT)                                            |");
        DebugP_log("\r\n| 25: ABS lower 24bit + velocity + acceleration                                         |");
        DebugP_log("\r\n| 26: ABS lower 24bit + velocity + acceleration(MT)                                     |");
        DebugP_log("\r\n| 27: ABS lower 24bit + status request                                                  |");
        DebugP_log("\r\n| 28: ABS lower 24bit + status request(MT)                                              |");
        DebugP_log("\r\n| 29: ABS lower 24bit + Temperature data request                                        |");
        DebugP_log("\r\n| 30: ABS lower 24bit + Temperature data request(MT)                                    |");
    }
    else
    {
        DebugP_log("\r\n| 0 : ABS full 40 bit data request                                                      |");
        DebugP_log("\r\n| 1 : ABS lower 24bit data request                                                      |");
        DebugP_log("\r\n| 2 : ABS upper 24bit data request                                                      |");
        DebugP_log("\r\n| 3 : Encoder status Request                                                            |");
        DebugP_log("\r\n| 4 : ABS full 40 bit data request(MT)                                                  |");
        DebugP_log("\r\n| 5 : ABS lower 24bit data request(MT)                                                  |");
        DebugP_log("\r\n| 6 : ABS upper 24bit data request(MT)                                                  |");
        DebugP_log("\r\n| 7 : Encoder status Request(MT)                                                        |");
        DebugP_log("\r\n| 8 : Status flag clear request                                                         |");
        DebugP_log("\r\n| 9 : Multiple turn data clear request                                                  |");
        DebugP_log("\r\n| 10: Status + Multiple turn data clear request                                         |");
        DebugP_log("\r\n| 11: Encoder address setting I (one-to-one connection)                                 |");
        DebugP_log("\r\n| 12: Single turn data zero preset                                                      |");
        DebugP_log("\r\n| 13: EEPROM read request                                                               |");
        DebugP_log("\r\n| 14: EEPROM write request                                                              |");
        DebugP_log("\r\n| 15: Temperature data request                                                          |");
        DebugP_log("\r\n| 16: Identification code read I                                                        |");
        DebugP_log("\r\n| 17: Identification code read II(one-to-one connection)                                |");
        DebugP_log("\r\n| 18: Identification code write I                                                       |");
        DebugP_log("\r\n| 19: Identification code write II(one-to-one connection)                               |");
        DebugP_log("\r\n| 20: Encoder address setting II                                                        |");
        DebugP_log("\r\n| 21: ABS lower 17bit data request                                                      |");
        DebugP_log("\r\n| 22: ABS lower 17bit data request(MT)                                                  |");
        DebugP_log("\r\n| 27: ABS lower 24bit + status request                                                  |");
        DebugP_log("\r\n| 28: ABS lower 24bit + status request(MT)                                              |");
        DebugP_log("\r\n| 29: ABS lower 24bit + Temperature data request                                        |");
        DebugP_log("\r\n| 30: ABS lower 24bit + Temperature data request(MT)                                    |");
    }
    DebugP_log("\r\n| Below options are user application specific for cmd prepare                           |");
    DebugP_log("\r\n| 31: Update Encoder address(EAX) in APP local context                                  |");
    DebugP_log("\r\n| 32: Start Continuous Mode (IEP CMP based trigger)                                     |");
    DebugP_log("\r\n| 33: Start Continuous Mode (IEP CAP based trigger)                                     |");
    DebugP_log("\r\n| 34: Select clock frequency in MHz(2.5/4/6.67/8/16)                                    |");
    DebugP_log("\r\n| 35: Update Encoder's Single Turn and Multi turn Resolution                            |");
    DebugP_log("\r\n|---------------------------------------------------------------------------------------|");
    DebugP_log("\r\n| enter value:\r\n");
}

static uint32_t nikon_get_command(nikon_handle handle)
{
    uint32_t cmd;
    const nikon_attrs *attrs = nikon_get_attrs(handle);

    if((handle == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\n| ERROR: NULL handle/attrs, nikon_get_command() will return 0");
        return 0;
    }

    DebugP_scanf("%u\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if(((attrs->protocol_version == NIKON_PROTOCOL_V2_1) && ((cmd > CMD_22 && cmd < CMD_27))) || (cmd >= CMD_CODE_NUM))
    {
        DebugP_log("\r\n| WARNING: Invalid option, nikon_get_command() will return 0\n");
        return 0;
    }
    return cmd;
}

static void nikon_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gNikonPositionLoopStatus = NIKON_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t nikon_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params task_params;

    TaskP_Params_init(&task_params);
    task_params.name = "nikon_position_loop_decide_termination";
    task_params.stackSize = TASK_STACK_SIZE;
    task_params.stack = (uint8_t *)gTaskFxnStack;
    task_params.priority = TASK_PRIORITY;
    task_params.taskMain = (TaskP_FxnMain)nikon_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &task_params);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: TaskP_construct() for nikon_position_loop_decide_termination failed\n");
    }

    return status;
}

/* NOTE: Validation error for any module instance will lead to failure of this function */
static int32_t nikon_process_periodic_command(nikon_handle handle[CONFIG_NIKON_NUM_INSTANCES], uint64_t trigger_count[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX], uint64_t iep_reset_count, uint8_t is_cap_mode)
{
    int32_t status;
    int32_t ret;
    uint32_t i;
    uint32_t ch_num;
    uint32_t ch;
    uint32_t enc_num;
    uint32_t ls_ch;
    uint32_t pos_fail_cnt[CONFIG_NIKON_NUM_INSTANCES] = {0};
    uint32_t pos_total_cnt = 0;
    nikon_priv *priv[CONFIG_NIKON_NUM_INSTANCES] = {NULL};
    uint32_t total_channels = 0;
    const nikon_attrs *attrs[CONFIG_NIKON_NUM_INSTANCES] = {NULL};
    uint32_t prev_irq_cnt[CONFIG_NIKON_NUM_INSTANCES] = {0};
    uint32_t curr_irq_cnt;
    uint32_t irq_ch_idx[CONFIG_NIKON_NUM_INSTANCES] = {0};

    /* CMD_4 is used in this example for 40-bit ABS data with multi-transmission. */
    uint32_t periodic_cmd = CMD_4;

    if((handle == NULL) || (trigger_count == NULL))
    {
        DebugP_log("\r\n\n| ERROR: NULL handle[] or trigger_count[]\n");
        return SystemP_FAILURE;
    }

    if(is_cap_mode > 1)
    {
        DebugP_log("\r\n\n| ERROR: Invalid is_cap_mode value\n");
        return SystemP_FAILURE;
    }

#if defined(SOC_AM243X)
    /* For AM243x, check iep_reset_count for 0 in both modes */
    if(iep_reset_count == 0)
    {
        DebugP_log("\r\n\n| ERROR: Invalid iep_reset_count value\n");
        return SystemP_FAILURE;
    }
#else
    if((is_cap_mode == 0) && (iep_reset_count == 0))
    {
        /* For AM26x, check iep_reset_count for 0 only in CMP mode.
         * In CAP mode, iep_reset_count is not used.
         */
        DebugP_log("\r\n\n| ERROR: Invalid iep_reset_count value\n");
        return SystemP_FAILURE;
    }
#endif
    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        priv[i] = nikon_get_priv(handle[i]);
        attrs[i] = nikon_get_attrs(handle[i]);

        if((handle[i] == NULL) || (priv[i] == NULL) || (attrs[i] == NULL))
        {
            DebugP_log("\r\n\n| ERROR: NULL handle/priv/attrs\n");
            return SystemP_FAILURE;
        }

        if(is_cap_mode == 0)
        {
            if(attrs[i]->load_share_enabled)
            {
                if(((attrs[i]->channel0_enabled) && (trigger_count[i][0] > iep_reset_count)) ||
                   ((attrs[i]->channel1_enabled) && (trigger_count[i][1] > iep_reset_count)) ||
                   ((attrs[i]->channel2_enabled) && (trigger_count[i][2] > iep_reset_count)))
                {
                    DebugP_log("\r\n\n| ERROR: Channel trigger count exceeds IEP reset count for instance %u\n", i);
                    return SystemP_FAILURE;
                }
            }
            else
            {
                if(trigger_count[i][0] > iep_reset_count)
                {
                    DebugP_log("\r\n\n| ERROR: Channel trigger count exceeds IEP reset count for instance %u\n", i);
                    return SystemP_FAILURE;
                }
            }
        }
    }

    DebugP_log("\r\n| Using command %u (CMD_%u) for periodic mode\n", periodic_cmd, periodic_cmd);

    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        /*
         * Send command once in host trigger mode, to ensure that command data is
         * populated in PRU shared memory as required.
         * ASSUMPTION: Host trigger mode is active when this function is called.
         */
        ret = nikon_generate_cdf(handle[i], periodic_cmd);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: Failed to generate command data frame for periodic mode for handle %d\n", i);
            return SystemP_FAILURE;
        }

        ret = nikon_get_pos(handle[i], periodic_cmd);
        if(ret != SystemP_SUCCESS)
        {
            if(ret == SystemP_TIMEOUT)
            {
                DebugP_log("\r\n ERROR: ABS measurement timed out for handle %d\n", i);
            }
            else
            {
                DebugP_log("\r\n ERROR: ABS measurement failed for handle %d\n", i);
            }
            return SystemP_FAILURE;
        }

        /* Configure periodic trigger mode */
        if(is_cap_mode)
        {
            ret = nikon_config_periodic_trigger_cap_mode(handle[i]);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to configure periodic trigger CAP mode for handle %d\n", i);
                return SystemP_FAILURE;
            }
        }
        else
        {
            ret = nikon_config_periodic_trigger_cmp_mode(handle[i]);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Failed to configure periodic trigger CMP mode for handle %d\n", i);
                return SystemP_FAILURE;
            }
        }
    }

    memset(&gNikonPeriodicInterface, 0, sizeof(gNikonPeriodicInterface));

    if(nikon_loop_task_create() != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    if(is_cap_mode == 0)
    {
        for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
        {
            if(attrs[i]->load_share_enabled)
            {
                if(attrs[i]->channel0_enabled)
                {
                    gNikonPeriodicInterface.periodic_trigger_count[i][0] = trigger_count[i][0];
                }
                if(attrs[i]->channel1_enabled)
                {
                    gNikonPeriodicInterface.periodic_trigger_count[i][1] = trigger_count[i][1];
                }
                if(attrs[i]->channel2_enabled)
                {
                    gNikonPeriodicInterface.periodic_trigger_count[i][2] = trigger_count[i][2];
                }
            }
            else
            {
                gNikonPeriodicInterface.periodic_trigger_count[i][0] = trigger_count[i][0];
            }
        }
    }

    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        gNikonPeriodicInterface.handle[i] = handle[i];
    }
    gNikonPeriodicInterface.iep_reset_count = iep_reset_count;
    gNikonPeriodicInterface.is_cap_mode = is_cap_mode;

    status = nikon_config_periodic_mode(&gNikonPeriodicInterface);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Failed to configure periodic mode\n");
        return SystemP_FAILURE;
    }

    /* Determine IRQ channel index for each instance (based on load share mode) */
    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        if(attrs[i]->load_share_enabled)
        {
            /* In load share mode, use channel index for first enabled channel */
            if(attrs[i]->channel0_enabled)
            {
                irq_ch_idx[i] = 0;
            }
            else if(attrs[i]->channel1_enabled)
            {
                irq_ch_idx[i] = 1;
            }
            else if(attrs[i]->channel2_enabled)
            {
                irq_ch_idx[i] = 2;
            }
        }
        else
        {
            irq_ch_idx[i] = 0;
        }

        /* Initialize previous IRQ count with current value */
        prev_irq_cnt[i] = gPruNikonIrqCnt[i][irq_ch_idx[i]];
    }

    gNikonPositionLoopStatus = NIKON_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press Enter to stop the continuous mode\r\n|");
    while(1)
    {
        if(gNikonPositionLoopStatus == NIKON_POSITION_LOOP_STOP)
        {
            for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
            {
                DebugP_log("\r\n Failed %u out of %u times for Nikon instance %u\n", pos_fail_cnt[i], pos_total_cnt, i);
            }

            ret = nikon_stop_periodic_mode(&gNikonPeriodicInterface);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r| WARNING: Failed to stop periodic mode\n");
                return SystemP_FAILURE;
            }
            return SystemP_SUCCESS;
        }
        else
        {
            /* Wait for IRQ count to increment for at least one instance before reading position */
            for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
            {
                /* Wait for IRQ count to increment */
                while(1)
                {
                    curr_irq_cnt = gPruNikonIrqCnt[i][irq_ch_idx[i]];
                    if(gNikonPositionLoopStatus == NIKON_POSITION_LOOP_STOP)
                    {
                        break;
                    }
                    /* Break as soon as IRQ count increments to avoid missing IRQs at high rates */
                    if(curr_irq_cnt != prev_irq_cnt[i])
                    {
                        break;
                    }
                    ClockP_usleep(NIKON_PERIODIC_MODE_POLL_SLEEP_US);
                }

                /* Check stop condition before updating prev_irq_cnt */
                if(gNikonPositionLoopStatus == NIKON_POSITION_LOOP_STOP)
                {
                    break;
                }

                prev_irq_cnt[i] = curr_irq_cnt;
            }

            /* If stop was requested during IRQ wait, continue for proper cleanup */
            if(gNikonPositionLoopStatus == NIKON_POSITION_LOOP_STOP)
            {
                continue;
            }

            pos_total_cnt++;

            /* Start with \r to overwrite the same line for all instances */
            DebugP_log("\r");

            for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
            {
                total_channels = attrs[i]->total_channels;

                ret = nikon_get_pos(handle[i], periodic_cmd);
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: ABS measurement timed out for instance %u\n", i);
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: ABS measurement failed for instance %u\n", i);
                    }
                    pos_fail_cnt[i]++;
                    continue;
                }

                /* For multi-instance, add instance prefix */
                if(CONFIG_NIKON_NUM_INSTANCES > 1)
                {
                    DebugP_log("Inst[%u]:", i);
                }

                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle[i], ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel for instance %u\n", i);
                        continue;
                    }

                    /* Separator between channels */
                    if(ch_num > 0)
                    {
                        DebugP_log(", ");
                    }

                    if(attrs[i]->load_share_enabled)
                    {
                        ls_ch = ch;
                    }
                    else
                    {
                        ls_ch = 0;
                    }

                    for(enc_num = 0; enc_num < priv[i]->num_enc_access[ls_ch]; enc_num++)
                    {
                        DebugP_log("Ch%d-Enc%d: ", ch, enc_num + 1);
                        if(priv[i]->multi_turn_len[ch][enc_num])
                        {
                            DebugP_log("MT:%u, ", priv[i]->pos_data_info[ch].multi_turn[enc_num]);
                        }
                        DebugP_log("Ang:%.4f, CRC_Err:%u", priv[i]->pos_data_info[ch].angle[enc_num], priv[i]->pos_data_info[ch].crc_err_cnt[enc_num]);
                    }
                }

                /* Separator between instances */
                if(CONFIG_NIKON_NUM_INSTANCES > 1 && ((i + 1) < CONFIG_NIKON_NUM_INSTANCES))
                {
                    DebugP_log(" | ");
                }
            }
        }
    }

    return SystemP_SUCCESS;
}

static int32_t nikon_handle_command(nikon_handle handle, uint32_t cmd)
{
    int32_t             ret;
    uint32_t            enc_addr = 0;
    uint32_t            ch_num;
    uint32_t            enc_num;
    uint32_t            pru_num;
    float_t             freq;
    uint32_t            ls_ch;
    uint32_t            total_channels;
    uint32_t            ch;
    uint32_t            addr = 0;
    uint32_t            data = 0;
    uint32_t            bank = 0;
    uint32_t            cmd_type;
    uint32_t            case_failed = 0;
    nikon_priv          *priv = nikon_get_priv(handle);
    const nikon_attrs   *attrs = nikon_get_attrs(handle);

    if((handle == NULL) || (priv == NULL) || (attrs == NULL))
    {
        return SystemP_FAILURE;
    }

    total_channels = attrs->total_channels;

    /* Get first channel for single channel operations */
    ret = nikon_get_current_channel(handle, 0, &ch);
    if(ret != SystemP_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
        return SystemP_FAILURE;
    }

    switch(cmd)
    {
        case CMD_0:
        case CMD_4:
            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: ABS measurement timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: ABS measurement failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }
                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    if(cmd == CMD_4)
                    {
                        DebugP_log("\r\n Encoder %d: \n", enc_num);
                    }
                    DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x, Data Field2: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num], priv->pos_data_info[ch].raw_data3[enc_num]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u,\n", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                    DebugP_log("\r\n ABS: 0x%llx, Multi Turn Rev: %d, ", priv->pos_data_info[ch].abs[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                    DebugP_log("Angle: %.12f \n", priv->pos_data_info[ch].angle[enc_num]);
                }
            }
            break;

        case CMD_1:
        case CMD_2:
        case CMD_5:
        case CMD_6:

            if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1 || cmd == CMD_5))
            {
                while(1)
                {
                    DebugP_log("\r\n Enter \"1\" to request ABS lower 24bit data, or enter \"2\" to request ABS 40bit data + velocity data : ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        break;
                    }
                    else if(cmd_type == 2)
                    {
                        cmd = (cmd == CMD_1)?CMD_1_VEL:CMD_5_VEL;
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: ABS measurement timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: ABS measurement failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }
                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    if(cmd == CMD_5 || cmd == CMD_6 || cmd == CMD_5_VEL)
                    {
                        DebugP_log("\r\n Encoder %d: \n", enc_num);
                    }

                    DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x\n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);

                    if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1_VEL || cmd == CMD_5_VEL))
                    {
                        DebugP_log("\r\n Data Field2: 0x%x, Data Field3: 0x%x, Data Field4: 0x%x\n", priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].raw_data4[enc_num], priv->pos_data_info[ch].raw_data5[enc_num]);
                    }

                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                    DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                    if((priv->abs_len >= priv->single_turn_len[ch][enc_num]) && cmd != CMD_2 && cmd != CMD_6)
                    {
                        DebugP_log("\r\n Angle: %.12f , Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                    }
                    if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1_VEL || cmd == CMD_5_VEL))
                    {
                        DebugP_log("\r\n Velocity: %d \n", priv->pos_data_info[ch].velocity[enc_num]);
                    }
                }
            }
            break;

        case CMD_3:
        case CMD_7:
        case CMD_8:
        case CMD_9:
        case CMD_10:
        case CMD_11:
        case CMD_12:

            if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd >= CMD_8 && cmd <= CMD_12))
            {
                while(1)
                {
                    DebugP_log("\r\n Enter \"1\" to request operation (clear or set as selected above), or enter \"2\" to request ABS lower 24bit data : ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        break;
                    }
                    else if(cmd_type == 2)
                    {
                        if(cmd == CMD_8)
                        {
                            cmd = CMD_8_POS;
                        }
                        else if(cmd == CMD_9)
                        {
                            cmd = CMD_9_POS;
                        }
                        else if(cmd == CMD_10)
                        {
                            cmd = CMD_10_POS;
                        }
                        else if(cmd == CMD_11)
                        {
                            cmd = CMD_11_POS;
                        }
                        else if(cmd == CMD_12)
                        {
                            cmd = CMD_12_POS;
                        }
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);

            if(cmd >= CMD_8_POS && cmd <= CMD_12_POS)
            {
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: ABS measurement timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: ABS measurement failed\n");
                    }
                    return SystemP_FAILURE;
                }
                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    if(attrs->load_share_enabled)
                    {
                        ls_ch = ch;
                    }
                    else
                    {
                        ls_ch = 0;
                    }
                    for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                    {
                        DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x\n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                        DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                        if((priv->abs_len >= priv->single_turn_len[ch][enc_num]))
                        {
                            DebugP_log("\r\n Angle: %.12f , Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                        }
                    }
                }
            }
            else
            {
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Encoder's operation request failed \n");
                    return SystemP_FAILURE;
                }

                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    if(attrs->load_share_enabled)
                    {
                        ls_ch = ch;
                    }
                    else
                    {
                        ls_ch = 0;
                    }
                    for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                    {
                        if(cmd == CMD_7)
                        {
                            DebugP_log("\r\n Encoder %d: \n", enc_num);
                        }
                        DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                        DebugP_log("\r\n ALM: 0x%x \n", priv->alm_field[ch][enc_num]);
                        DebugP_log("\r\n Batt: %u, MtErr: %u, OverFlow: %u, OverSpeed M: %u, Memory Error: %u, Single Turn Error M: %u \n", priv->alm_bits[ch][enc_num].batt, priv->alm_bits[ch][enc_num].mt_err, priv->alm_bits[ch][enc_num].ov_flow, priv->alm_bits[ch][enc_num].ov_spd, priv->alm_bits[ch][enc_num].mem_err, priv->alm_bits[ch][enc_num].st_err);
                        DebugP_log("\r\n PS Error M: %u, Busy M: %u, Memory Busy: %u, Over Temperature: %u, Increment Error M: %u \n", priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err_m);
                        DebugP_log("\r\n OverSpeed S: %u, Single Turn Error S: %u, PS Error S: %u, Busy S: %u, Increment Error S: %u \n", priv->alm_bits[ch][enc_num].ov_spd_s, priv->alm_bits[ch][enc_num].st_err_s, priv->alm_bits[ch][enc_num].ps_err_s, priv->alm_bits[ch][enc_num].busy_s, priv->alm_bits[ch][enc_num].inc_err_s);
                        if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
                        {
                            DebugP_log("\r\n PM ALM: 0x%x \n", priv->pm_alm_field[ch][enc_num]);
                            DebugP_log("\r\n INCW1: %u, INCW2: %u, IFW1:%u, IFW2: %u", priv->pm_alm_bits[ch][enc_num].incw_1, priv->pm_alm_bits[ch][enc_num].incw_2, priv->pm_alm_bits[ch][enc_num].ifw_1, priv->pm_alm_bits[ch][enc_num].ifw_2);
                        }
                    }
                }
            }
            break;

        case CMD_13:
            case_failed = 0;  /* Reset flag for this case */

            if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
            {
                while(1)
                {
                    DebugP_log("\r\n Read with bank? (0 for without bank / 1 for with bank) ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        cmd = CMD_13_BANK;

                        for(pru_num = 0; pru_num < total_channels; pru_num++)
                        {
                            ret = nikon_get_current_channel(handle, pru_num, &ch);
                            if(ret != SystemP_SUCCESS)
                            {
                                DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                                case_failed = 1;
                                break;
                            }
                            if(attrs->load_share_enabled)
                            {
                                ls_ch = ch;
                                DebugP_log("\r\n Channel %d: \n", ch);
                            }
                            else
                            {
                                ls_ch = 0;
                                pru_num = total_channels;
                            }

                            while (1)
                            {
                                DebugP_log("\r\n Enter bank number (in hex): ");
                                DebugP_scanf("%x", &bank);

                                if(bank > 0xFF)
                                {
                                    DebugP_log("\r\n Please enter a valid bank value\n");
                                }
                                else
                                {
                                    break;
                                }
                            }

                            nikon_update_eeprom_bank(handle, (bank & 0xFF), ls_ch);
                        }
                        break;
                    }
                    else if(cmd_type == 0)
                    {
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            if(case_failed)
            {
                break;  /* Exit switch case if first for loop failed */
            }

            for(pru_num = 0; pru_num < total_channels; pru_num++)
            {
                ret = nikon_get_current_channel(handle, pru_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                    case_failed = 1;
                    break;
                }
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                    DebugP_log("\r\nChannel %d: ", ch);
                }
                else
                {
                    ls_ch = 0;
                    pru_num = total_channels;
                }

                while(1)
                {
                    DebugP_log("\r\n Enter Memory location(in hex) to read(00h to FFh is valid): ");
                    DebugP_scanf("%x", &addr);

                    if(addr > 0xFF)
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit value\n");
                    }
                    else
                    {
                        nikon_update_eeprom_addr(handle, (addr & 0xFF), ls_ch);
                        break;
                    }
                }
            }

            if(case_failed)
            {
                break;  /* Exit switch case if second for loop failed */
            }

            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: EEPROM Read access request timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: EEPROM Read access request failed\n");
                }
                return SystemP_FAILURE;
            }
            else
            {
                /* 300 micro-seconds sleep - wait for read data to be determined*/
                ClockP_usleep(NIKON_EEPROM_READ_WAIT_US);
                ret = nikon_get_pos(handle, cmd);

                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: EEPROM Read access request timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: EEPROM Read access request failed\n");
                    }
                    return SystemP_FAILURE;
                }
            }

            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(CMD_13_BANK == cmd)
                {
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Bank: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], (uint16_t)nikon_reverse_bits((uint16_t)(priv->pos_data_info[ch].raw_data1[0]), NIKON_RX_ONE_FRAME_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8), NIKON_EEPROM_BANK_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF)), NIKON_EEPROM_ADDR_LEN));

                    if(priv->bank_error == 1)
                    {
                        DebugP_log("\r\n ERROR: Invalid bank number was specified \n");
                    }
                }
                else
                {
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], (uint16_t)nikon_reverse_bits((uint16_t)(priv->pos_data_info[ch].raw_data1[0]), NIKON_RX_ONE_FRAME_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8), NIKON_EEPROM_ADDR_LEN));
                }

                DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                if((cmd == CMD_13) && ((uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8), NIKON_EEPROM_ADDR_LEN) == 0xF9))
                {
                    DebugP_log("\r\n Temperature: %d \n", priv->temperature[ch][0]);
                }
            }
            break;

        case CMD_14:
            case_failed = 0;  /* Reset flag for this case */

            if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
            {
                while(1)
                {
                    DebugP_log("\r\n Read with bank? (0 for without bank / 1 for with bank) ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        cmd = CMD_14_BANK;

                        for(pru_num = 0; pru_num < total_channels; pru_num++)
                        {
                            ret = nikon_get_current_channel(handle, pru_num, &ch);
                            if(ret != SystemP_SUCCESS)
                            {
                                DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                                case_failed = 1;
                                break;
                            }
                            if(attrs->load_share_enabled)
                            {
                                ls_ch = ch;
                                DebugP_log("\r\n Channel %d: \n", ch);
                            }
                            else
                            {
                                ls_ch = 0;
                                pru_num = total_channels;
                            }

                            while (1)
                            {
                                DebugP_log("\r\n Enter bank number (in hex): ");
                                DebugP_scanf("%x", &bank);

                                if(bank > 0xFF)
                                {
                                    DebugP_log("\r\n Please enter a valid bank value\n");
                                }
                                else
                                {
                                    break;
                                }
                            }

                            nikon_update_eeprom_bank(handle, (bank & 0xFF), ls_ch);
                        }
                        break;
                    }
                    else if(cmd_type == 0)
                    {
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            if(case_failed)
            {
                break;  /* Exit switch case if first for loop failed */
            }

            for(pru_num = 0; pru_num < total_channels; pru_num++)
            {
                ret = nikon_get_current_channel(handle, pru_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                    case_failed = 1;
                    break;
                }
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                    DebugP_log("\r\nChannel %d: ", ch);
                }
                else
                {
                    ls_ch = 0;
                    pru_num = total_channels;
                }

                while(1)
                {
                    if(cmd == CMD_14_BANK)
                    {
                        DebugP_log("\r\n Enter Memory location(in hex) to write (00h to FFh is valid): ");
                    }
                    else
                    {
                        DebugP_log("\r\n Enter Memory location(in hex) to write (00h to EFh is valid): ");
                    }

                    DebugP_scanf("%x", &addr);
                    DebugP_log("\r\n Enter data(Bits [15:0] in hex) to write at Memory location 0x%x: ", addr);
                    DebugP_scanf("%x", &data);

                    if((data > 0xFFFF) || (addr > 0xFF) || ((cmd == CMD_14) && (addr > 0xEF)))
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit value\n");
                    }
                    else
                    {
                        nikon_update_eeprom_addr(handle, (addr & 0xFF), ls_ch);
                        nikon_update_eeprom_data(handle, (data & 0xFFFF), ls_ch);
                        break;
                    }
                }
            }

            if(case_failed)
            {
                break;  /* Exit switch case if second for loop failed */
            }

            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: EEPROM Write access request timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: EEPROM Write access request failed\n");
                }
                return SystemP_FAILURE;
            }
            else
            {
                /* 30 milli-seconds sleep - wait for write operation to finish*/
                ClockP_usleep(NIKON_EEPROM_WRITE_WAIT_US);
            }

            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);

                if(CMD_14_BANK == cmd)
                {
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Bank: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], (uint16_t)nikon_reverse_bits((uint16_t)(priv->pos_data_info[ch].raw_data1[0]), NIKON_RX_ONE_FRAME_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8), NIKON_EEPROM_BANK_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF)), NIKON_EEPROM_ADDR_LEN));

                    if(priv->bank_error == 1)
                    {
                        DebugP_log("\r\n ERROR: Invalid bank number was specified \n");
                    }
                }
                else
                {
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], (uint16_t)nikon_reverse_bits((uint16_t)(priv->pos_data_info[ch].raw_data1[0]), NIKON_RX_ONE_FRAME_LEN), (uint8_t)nikon_reverse_bits((uint8_t)((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8), NIKON_EEPROM_ADDR_LEN));
                }
                DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
            }
            break;

        case CMD_15:
            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: Encoder's temperature request timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: Encoder's temperature request failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                DebugP_log("\r\n Info Field: 0x%x, Temperature: %u \n", priv->pos_data_info[ch].raw_data0[0], priv->temperature[ch][0]);
                DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
            }
            break;

        case CMD_16:
        case CMD_17:
            if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_16))
            {
                while(1)
                {
                    DebugP_log("\r\n Enter \"1\" to request identification code read, or enter \"2\" to request velocity coefficient read : ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        break;
                    }
                    else if(cmd_type == 2)
                    {
                        cmd = CMD_16_VEL;
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);

            if(cmd == CMD_16_VEL)
            {
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: Velocity coefficient read request timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Velocity coefficient read request failed\n");
                    }
                    return SystemP_FAILURE;
                }

                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    DebugP_log("\r\n Info Field: 0x%x, Velocity coefficient (Bits [18:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->velocity_coefficient[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
            }
            else
            {
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: Identification code read request timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Identification code read request failed\n");
                    }
                    return SystemP_FAILURE;
                }
                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    DebugP_log("\r\n Info Field: 0x%x, Identification Code (Bits [23:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
            }
            break;

        case CMD_18:
        case CMD_19:
        case CMD_20:
            case_failed = 0;  /* Reset flag for this case */

            if ((attrs->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_18))
            {
                while(1)
                {
                    DebugP_log("\r\n Enter \"1\" to request identification code write, or enter \"2\" to request velocity coefficient write : ");
                    DebugP_scanf("%u", &cmd_type);

                    if(cmd_type == 1)
                    {
                        break;
                    }
                    else if(cmd_type == 2)
                    {
                        cmd = CMD_18_VEL;
                        break;
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Invalid option, try again \n");
                    }
                }
            }

            if(cmd == CMD_18_VEL)
            {
                for(pru_num = 0; pru_num < total_channels; pru_num++)
                {
                    ret = nikon_get_current_channel(handle, pru_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                        case_failed = 1;
                        break;
                    }
                    if(attrs->load_share_enabled)
                    {
                        ls_ch = ch;
                        DebugP_log("\r\nChannel %d: ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = total_channels;
                    }

                    while(1)
                    {
                        DebugP_log("\r\n Enter data(Bits [18:0] in hex) to assign as Velocity coefficient: ");
                        DebugP_scanf("%x", &data);
                        if((data > 0x7FFFF))
                        {
                            DebugP_log("\r\n Please enter valid value \n");
                        }
                        else
                        {
                            nikon_update_velocity_coefficient(handle, data, ls_ch);
                            break;
                        }
                    }
                }

                if(case_failed)
                {
                    break;  /* Exit switch case if for loop failed */
                }

                ret = nikon_generate_cdf(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                    return SystemP_FAILURE;
                }
                ret = nikon_get_pos(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: Encoder's Velocity coefficient code write access timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: Encoder's Velocity coefficient code write access failed\n");
                    }
                    return SystemP_FAILURE;
                }
                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    DebugP_log("\r\n Info Field: 0x%x, Velocity coefficient (Bits [18:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->velocity_coefficient[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
            }
            else
            {
                for(pru_num = 0; pru_num < total_channels; pru_num++)
                {
                    ret = nikon_get_current_channel(handle, pru_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel. Command execution will be skipped for all channels.\n");
                        case_failed = 1;
                        break;
                    }
                    if(attrs->load_share_enabled)
                    {
                        ls_ch = ch;
                        DebugP_log("\r\nChannel %d: ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = total_channels;
                    }

                    while(1)
                    {
                        DebugP_log("\r\n Enter data(Bits [23:0] in hex) to assign as identification code: ");
                        DebugP_scanf("%x", &data);

                        if((data > 0xFFFFFF))
                        {
                            DebugP_log("\r\n Please enter a valid value \n");
                        }
                        else
                        {
                            nikon_update_id_code(handle, data, ls_ch);
                            break;
                        }
                    }
                }

                if(case_failed)
                {
                    break;  /* Exit switch case if for loop failed */
                }

                ret = nikon_generate_cdf(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                    return SystemP_FAILURE;
                }
                ret = nikon_get_pos(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    if(cmd == CMD_20)
                    {
                        if(ret == SystemP_TIMEOUT)
                        {
                            DebugP_log("\r\n ERROR: Encoder's address setting timed out\n");
                        }
                        else
                        {
                            DebugP_log("\r\n ERROR: Encoder's address setting failed\n");
                        }
                    }
                    else
                    {
                        if(ret == SystemP_TIMEOUT)
                        {
                            DebugP_log("\r\n ERROR: Encoder's identification code write access timed out\n");
                        }
                        else
                        {
                            DebugP_log("\r\n ERROR: Encoder's identification code write access failed\n");
                        }
                    }
                    return SystemP_FAILURE;
                }

                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    DebugP_log("\r\n Info Field: 0x%x, Identification Code (Bits [23:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
            }
            break;

        case CMD_21:
        case CMD_22:
            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: 17bit ABS measurement timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: 17bit ABS measurement failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }
                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    if(cmd == CMD_22)
                    {
                        DebugP_log("\r\n Encoder %d: \n", enc_num);
                    }
                    DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, ABS: 0x%llx \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].abs[enc_num]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x\n", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num]);

                    if(priv->abs_len >= priv->single_turn_len[ch][enc_num])
                    {
                        DebugP_log("\r\n Angle: %.12f\n", priv->pos_data_info[ch].angle[enc_num]);
                    }
                }
            }
            break;
        case CMD_23:
        case CMD_24:
        case CMD_25:
        case CMD_26:
            if (attrs->protocol_version == NIKON_PROTOCOL_V3_0)
            {
                ret = nikon_generate_cdf(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                    return SystemP_FAILURE;
                }
                ret = nikon_get_pos(handle, cmd);
                if(ret != SystemP_SUCCESS)
                {
                    if(ret == SystemP_TIMEOUT)
                    {
                        DebugP_log("\r\n ERROR: ABS measurement and velocity/acceleration request timed out\n");
                    }
                    else
                    {
                        DebugP_log("\r\n ERROR: ABS measurement and velocity/acceleration request failed\n");
                    }
                    return SystemP_FAILURE;
                }

                for(ch_num = 0; ch_num < total_channels; ch_num++)
                {
                    ret = nikon_get_current_channel(handle, ch_num, &ch);
                    if(ret != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                        return SystemP_FAILURE;
                    }
                    DebugP_log("\r\n Channel %d: \n", ch);
                    if(attrs->load_share_enabled)
                    {
                        ls_ch = ch;
                    }
                    else
                    {
                        ls_ch = 0;
                    }
                    for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                    {
                        if(cmd == CMD_24 || cmd == CMD_26)
                        {
                            DebugP_log("\r\n Encoder %d: \n", enc_num);
                        }
                        DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x\n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);

                        if(cmd == CMD_25 || cmd == CMD_26)
                        {
                            DebugP_log("\r\n Data Field2: 0x%x, Data Field3: 0x%x Data Field4: 0x%x\n", priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].raw_data4[enc_num], priv->pos_data_info[ch].raw_data5[enc_num]);
                        }
                        else
                        {
                            DebugP_log("\r\n Data Field2: 0x%x, Data Field3: 0x%x\n", priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].raw_data4[enc_num]);
                        }

                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                        DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                        if(priv->abs_len >= priv->single_turn_len[ch][enc_num])
                        {
                            DebugP_log("\r\n Angle: %.12f , Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                        }

                        DebugP_log("\r\n Velocity: %d \n", priv->pos_data_info[ch].velocity[enc_num]);

                        if(cmd == CMD_25 || cmd == CMD_26)
                        {
                            DebugP_log("\r\n Acceleration: %d \n", priv->pos_data_info[ch].acc[enc_num]);
                        }

                    }
                }
            }
            break;
        case CMD_27:
        case CMD_28:
            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's status request timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's status request failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }
                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    if(cmd == CMD_28)
                    {
                        DebugP_log("\r\n Encoder %d: \n", enc_num);
                    }
                    DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x, Data Field2: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num], priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].abs[enc_num]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                    DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                    if(priv->abs_len >= priv->single_turn_len[ch][enc_num])
                    {
                        DebugP_log("\r\n Angle: %.12f, Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                    }
                    DebugP_log("\r\n ALM: 0x%x \n", priv->alm_field[ch][enc_num]);
                    DebugP_log("\r\n Batt: %u, MtErr: %u, OverFlow: %u, OverSpeed M: %u, Memory Error: %u, Single Turn Error M: %u \n", priv->alm_bits[ch][enc_num].batt, priv->alm_bits[ch][enc_num].mt_err, priv->alm_bits[ch][enc_num].ov_flow, priv->alm_bits[ch][enc_num].ov_spd, priv->alm_bits[ch][enc_num].mem_err, priv->alm_bits[ch][enc_num].st_err);
                    DebugP_log("\r\n PS Error M: %u, Busy M: %u, Memory Busy: %u, Over Temperature: %u, Increment Error M: %u \n", priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err_m);
                    DebugP_log("\r\n OverSpeed S: %u, Single Turn Error S: %u, PS Error S: %u, Busy S: %u, Increment Error S: %u \n", priv->alm_bits[ch][enc_num].ov_spd_s, priv->alm_bits[ch][enc_num].st_err_s, priv->alm_bits[ch][enc_num].ps_err_s, priv->alm_bits[ch][enc_num].busy_s, priv->alm_bits[ch][enc_num].inc_err_s);
                }
            }
            break;

        case CMD_29:
        case CMD_30:
            ret = nikon_generate_cdf(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to generate command data frame\n");
                return SystemP_FAILURE;
            }
            ret = nikon_get_pos(handle, cmd);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's temperature request timed out\n");
                }
                else
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's temperature request failed\n");
                }
                return SystemP_FAILURE;
            }
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                ret = nikon_get_current_channel(handle, ch_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                DebugP_log("\r\n Channel %d: \n", ch);
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }
                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    if(cmd == CMD_30)
                    {
                        DebugP_log("\r\n Encoder %d: \n", enc_num);
                    }
                    DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x, Data Field2: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num], priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].abs[enc_num]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                    DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                    if(priv->abs_len >= priv->single_turn_len[ch][enc_num])
                    {
                        DebugP_log("\r\n Angle: %.12f, Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
                    }
                    DebugP_log("\r\n Temperature: %u \n", priv->temperature[ch][enc_num]);
                }
            }
            break;

        case ENCODER_ADR_CHANGE:
            for(pru_num = 0; pru_num < total_channels; pru_num++)
            {
                ret = nikon_get_current_channel(handle, pru_num, &ch);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: Failed to get current channel\n");
                    return SystemP_FAILURE;
                }
                if(attrs->load_share_enabled)
                {
                    ls_ch = ch;
                    DebugP_log("\r\nChannel %d: ", ch);
                }
                else
                {
                    ls_ch = 0;
                    pru_num = total_channels;
                }
                while(1)
                {
                    DebugP_log("\r\n Current encoder address : %u", (uint32_t)nikon_reverse_bits(priv->eax[ls_ch], NIKON_ENC_ADDR_LEN));
                    DebugP_log("\r\n Please enter the encoder address : ");
                    DebugP_scanf("%u", &enc_addr);
                    if(enc_addr > NIKON_ENC_ADDR_MAX)
                    {
                        DebugP_log("\r\n Please enter a 3-bit value(0-%u)\n", NIKON_ENC_ADDR_MAX);
                    }
                    else
                    {
                        break;
                    }
                }
                nikon_update_enc_addr(handle, enc_addr, ls_ch);
            }
            break;

        case UPDATE_CLOCK_FREQ:
            DebugP_log("\r\nNOTE: Source clock selection for Nikon (PRU-ICSS Core Clock or PRU-ICSS UART Clock) is not changed in this option\n");
            DebugP_log("\r\nPlease enter frequency in MHz:\n");
            DebugP_scanf("%f\n", &freq);
            if(!((freq == NIKON_FREQ_2_5MHZ) || (freq == NIKON_FREQ_4MHZ) || (((uint8_t)freq % NIKON_FREQ_6_67MHZ) < 1) || (freq == NIKON_FREQ_8MHZ) || (freq == NIKON_FREQ_16MHZ)))
            {
                DebugP_log("\r\n CLK divisors will not be possible. Please provide valid freq: 2.5/4/6.67/8/16 \n");
                return SystemP_FAILURE;
            }
            ret = nikon_update_clock_freq(handle, freq);
            if(ret != SystemP_SUCCESS)
            {
                DebugP_log("\r\n ERROR: Failed to update clock frequency\n");
            }
            break;

        case UPDATE_ENC_LEN:
            nikon_get_enc_data_len(handle);
            break;

        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

/**
 * \brief   Nikon diagnostic application main function
 *
 * \details This function implements the main diagnostic application flow for Nikon
 *          encoder interface. It initializes the Nikon driver, loads and starts PRU
 *          firmware, and provides an interactive menu-driven interface for various
 *          encoder operations including:
 *          - Position data acquisition (single-shot and continuous)
 *          - EEPROM read/write operations
 *          - Frequency configuration
 *          - Encoder address configuration
 *          - Temperature reading
 *          - Identification code operations
 *          - Velocity coefficient operations (Nikon 3.0)
 *          - Periodic trigger mode configuration
 *          - Dual PRU slice operation for AM261x devices
 *
 *          The function follows this sequence:
 *          1. Initialize system drivers and board drivers
 *          2. Wait for encoder power-up stabilization
 *          3. Display firmware version information
 *          4. Initialize PRU-ICSS hardware and Nikon driver
 *          5. Configure encoder resolution parameters
 *          6. Load PRU firmware and wait for encoder detection
 *          7. Enter interactive command loop for encoder operations
 *          8. Clean up and de-initialize on exit
 *
 * \param[in]   args    Unused parameter
 */
void nikon_main(void *args)
{
    int32_t             ret;
    uint32_t            i;
    uint32_t            ch_num;
    uint32_t            pru_num;
    uint64_t            iep_reset_count = 0;
    uint64_t            trigger_count[CONFIG_NIKON_NUM_INSTANCES][NIKON_NUM_CH_PER_SLICE_MAX]={0};
    uint32_t            total_channels;
    uint32_t            cmd[CONFIG_NIKON_NUM_INSTANCES];
    uint8_t             is_cmd_continuous = 0;
    uint8_t             is_cap_mode = 0;
    nikon_params        nikon_params_instance;
    nikon_priv          *priv[CONFIG_NIKON_NUM_INSTANCES] = {NULL};
    const nikon_attrs   *attrs[CONFIG_NIKON_NUM_INSTANCES] = {NULL};

    /* ========================================================================== */
    /* INITIALIZATION PHASE                                                       */
    /* ========================================================================== */

    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                           */
    /* ========================================================================== */
    Drivers_open();
    Board_driversOpen();

    /* ========================================================================== */
    /* STEP 2: Wait for encoder power-up stabilization                            */
    /* ========================================================================== */
    /* Allow encoder power supply to stabilize before communication (per encoder datasheet) */
    ClockP_usleep(NIKON_POWER_UP_DELAY);

    /* ========================================================================== */
    /* STEP 3: Display firmware version information                               */
    /* ========================================================================== */
    nikon_display_fw_version();

    /* ========================================================================== */
    /* STEP 4: Initialize PRU-ICSS hardware subsystem                             */
    /* ========================================================================== */
    nikon_pruicss_init();

    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        DebugP_log("\r\n|------------------------------------------------------------------------------|");
        DebugP_log("\r\n| Nikon Instance %u                                                             |", i);

        /* ========================================================================== */
        /* STEP 5: Initialize Nikon driver parameters                                 */
        /* ========================================================================== */

        nikon_params_init(&nikon_params_instance);
        nikon_params_instance.pruicss_handle = gPruIcssXHandle;

        /* ========================================================================== */
        /* STEP 6: Initialize Nikon driver instance                                   */
        /* ========================================================================== */
        /* This internally calls: nikon_hw_init(), nikon_config_channel(),            */
        /* nikon_config_load_share(), nikon_set_default_initialization().             */
        /* NOTE: Default host trigger mode is set.                                    */
        gAppNikonHandle[i] = nikon_init(i, &nikon_params_instance);
        attrs[i] = nikon_get_attrs(gAppNikonHandle[i]);
        priv[i] = nikon_get_priv(gAppNikonHandle[i]);

        if((gAppNikonHandle[i] == NULL) || (attrs[i] == NULL) || (priv[i] == NULL))
        {
            DebugP_log("\r\nERROR: Nikon initialization failed\n");
            return;
        }

        /* ========================================================================== */
        /* STEP 7: Display HW instances used, operation mode and enabled channels     */
        /* ========================================================================== */
        total_channels = attrs[i]->total_channels;
        DebugP_log("\r\n| NIKON Protocol Version selected\t: %s                                  |", (attrs[i]->protocol_version == NIKON_PROTOCOL_V2_1)?"2.1":"3.0");
        DebugP_log("\r\n| PRU-ICSS instance: %u, PRU-ICSS slice number: %u                               |", attrs[i]->pruicss_instance, attrs[i]->pruicss_slice);

        if(attrs[i]->mode == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
        {
            DebugP_log("\r\n| Nikon Load Share Demo application is running......                           |");
            for(pru_num = 0; pru_num < total_channels; pru_num++)
            {
                DebugP_log("\r\n| Channel %d is enabled                                                         |", priv[i]->channel[pru_num]);
            }
        }
        else if(attrs[i]->mode == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
        {
            DebugP_log("\r\n| Nikon Multi channel, Single PRU Demo application is running......            |");
            for(ch_num = 0; ch_num < total_channels; ch_num++)
            {
                DebugP_log("\r\n| Channel %d is enabled                                                         |", priv[i]->channel[ch_num]);
            }
        }
        else
        {
            DebugP_log("\r\n| Nikon Single channel, Single PRU Demo application is running......           |");
            DebugP_log("\r\n| Channel %d is enabled                                                         |", priv[i]->channel[0]);
        }
        DebugP_log("\r\n|------------------------------------------------------------------------------|\n\n");

        /* ========================================================================== */
        /* STEP 8: Get encoder resolution configuration from user                     */
        /* ========================================================================== */
        nikon_get_enc_data_len(gAppNikonHandle[i]);
    }

    /* ========================================================================== */
    /* STEP 9: Load and run PRU firmware for encoder detection                    */
    /* ========================================================================== */
    DebugP_log("\r\nRunning CDF4(Multi Transmission command) with maximum encoder address for detecting connected encoder\n");

    nikon_pruicss_load_run_fw();

    /* ========================================================================== */
    /* STEP 10: Wait for encoder detection and verify communication               */
    /* ========================================================================== */
    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        ret = nikon_wait_for_encoder_detection(gAppNikonHandle[i]);
        if(ret != SystemP_SUCCESS)
        {
            if(ret == SystemP_TIMEOUT)
            {
                DebugP_log("\r\nERROR: NIKON encoder detection timed out for instance %u\n", i);
                DebugP_log("\r\n Encoder is not responding within the configured timeout period\n");
            }
            else
            {
                DebugP_log("\r\nERROR: NIKON initialization failed for instance %u\n", i);
            }
            DebugP_log("\r\n Check whether encoder of selected frequency is connected and ensure proper connections\n");
            DebugP_log("\r\n Exit %s due to failed firmware initialization\n", __func__);
            goto deinit;
        }

        if(((uint8_t)attrs[i]->baud_rate == 6) || ((uint8_t)attrs[i]->baud_rate == 2))
        {
            DebugP_log("\r\nNIKON encoder/encoders detected and running at frequency %fMHz for instance %u\n", (float)attrs[i]->baud_rate, i);
        }
        else
        {
            DebugP_log("\r\nNIKON encoder/encoders detected and running at frequency %dMHz for instance %u\n", attrs[i]->baud_rate, i);
        }
    }
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
    DebugP_log("\r\n| In host trigger mode, command will be run on both Nikon instances one-by-one |\n\n");
#endif

    /* ========================================================================== */
    /*          MAIN COMMAND LOOP - Interactive encoder operations                */
    /* ========================================================================== */
    /* Process user commands for encoder operations including position reads,     */
    /* EEPROM access, configuration changes, and periodic mode control            */
    while(1)
    {
        is_cmd_continuous = 0;

        for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
        {
            /* Display menu and get user command */
            nikon_display_menu(gAppNikonHandle[i], i);
            cmd[i] = nikon_get_command(gAppNikonHandle[i]);

            if((cmd[i] == START_CONTINUOUS_CMP_MODE) || (cmd[i] == START_CONTINUOUS_CAP_MODE))
            {
                is_cmd_continuous = 1;
                is_cap_mode = (cmd[i] == START_CONTINUOUS_CAP_MODE) ? 1 : 0;

                /* is_cmd_continuous and is_cap_mode are used for continuous mode processing below.
                   cmd[i] is used below only when is_cmd_continuous = 0.*/
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
            DebugP_log("\r\n| Continuous mode command will be executed on all encoders \n\n");
#endif
                break;
            }
        }

        /* Handle periodic mode */
        if(is_cmd_continuous)
        {
            if(is_cap_mode == 0)
            {
#if defined(NIKON_DUAL_PRU_SLICE_ENABLE)
                DebugP_log("\r\n IEP is common for both Nikon instances. Same IEP periodic cycle will be used for both instances.\n");
#endif
                DebugP_log("\r| Enter IEP cycle count(must be greater than Nikon cycle time, in IEP cycles): ");
                DebugP_scanf("%llu\n", &iep_reset_count);
                if((iep_reset_count == 0) || (iep_reset_count <= NIKON_IEP_COUNTER_INCREMENT))
                {
                    DebugP_log("\r\n| WARNING: invalid value entered\n");
                    continue;
                }

                for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
                {
                    DebugP_log("\n\r| Enter IEP trigger time for instance %u: \n", i);

                    if(attrs[i]->load_share_enabled)
                    {
                        if(attrs[i]->channel0_enabled)
                        {
                            DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel0: \n");
                            DebugP_scanf("%llu\n", &trigger_count[i][0]);
                            if((trigger_count[i][0] > iep_reset_count) || (trigger_count[i][0] <= NIKON_IEP_COUNTER_INCREMENT))
                            {
                                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                                continue;
                            }
                        }
                        if(attrs[i]->channel1_enabled)
                        {
                            DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel1: \n");
                            DebugP_scanf("%llu\n", &trigger_count[i][1]);
                            if((trigger_count[i][1] > iep_reset_count) || (trigger_count[i][1] <= NIKON_IEP_COUNTER_INCREMENT))
                            {
                                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                                continue;
                            }
                        }
                        if(attrs[i]->channel2_enabled)
                        {
                            DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel2: \n");
                            DebugP_scanf("%llu\n", &trigger_count[i][2]);
                            if((trigger_count[i][2] > iep_reset_count) || (trigger_count[i][2] <= NIKON_IEP_COUNTER_INCREMENT))
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
                        if((trigger_count[i][0] > iep_reset_count) || (trigger_count[i][0] <= NIKON_IEP_COUNTER_INCREMENT))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }
                }
                DebugP_log("\r\n| Switching to periodic trigger CMP mode\n");
            }
            else /* (is_cap_mode == 1) */
            {
#if defined(SOC_AM243X)
                DebugP_log("\r| Enter IEP SYNC0 period (in IEP cycles, used for CAP mode):");
                DebugP_scanf("%llu\n", &iep_reset_count);
                if((iep_reset_count == 0) || (iep_reset_count <= NIKON_IEP_COUNTER_INCREMENT) || (iep_reset_count > UINT32_MAX))
                {
                    DebugP_log("\r\n| ERROR: invalid value entered. 0 is not allowed and maximum value allowed is %u\n", UINT32_MAX);
                    continue;
                }
#else
                DebugP_log("\r| Periodic CAP mode cycle time will be equal to EPWM frequency. NOTE: In SysConfig, EPWM and EPWM to IEP LATCH XBAR configuration must be done. \n|\n|\n|\n");
#endif
                DebugP_log("\r\n| Switching to periodic trigger CAP mode\n");
            }

            ret = nikon_process_periodic_command(gAppNikonHandle, trigger_count, iep_reset_count, is_cap_mode);
            if(ret != SystemP_SUCCESS)
            {
                    DebugP_log("\r| ERROR: nikon_process_periodic_command() failed\n");
            }

            DebugP_log("\r| Revert to host trigger\n");
            for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
            {
                ret = nikon_config_host_trigger(gAppNikonHandle[i]);
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Failed to revert to host trigger\n");
                }
            }
        }
        else
        {
            /* Handle commands other than periodic mode */
            for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
            {
                DebugP_log("\r\n| Nikon instance %u: Command %u", i, cmd[i]);
                ret = nikon_handle_command(gAppNikonHandle[i], cmd[i]);;
                if(ret != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: nikon_handle_command() failed for instance %u\n", i);
                }
            }
        }
    }

    /* ========================================================================== */
    /* CLEANUP AND DEINITIALIZATION                                              */
    /* ========================================================================== */
deinit:
    /* De-initialize Nikon driver and other drivers */
    for(i = 0; i < CONFIG_NIKON_NUM_INSTANCES; i++)
    {
        nikon_deinit(gAppNikonHandle[i]);
    }

    Board_driversClose();
    Drivers_close();
    return;
}
