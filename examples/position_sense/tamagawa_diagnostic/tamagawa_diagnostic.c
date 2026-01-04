/*
 *  Copyright (C) 2022-2025 Texas Instruments Incorporated
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
 * \file  tamagawa_diagnostic.c
 *
 * \brief Tamagawa encoder diagnostic application
 *
 * This application provides comprehensive diagnostic and testing capabilities for
 * Tamagawa encoders, including single-shot position reads, EEPROM operations,
 * reset commands, and periodic trigger mode.
 *
 * \par Dual Slice Support (tested only on AM261x with Single PRU Single Channel mode):
 * This diagnostic application supports dual PRU slice operation when compiled with
 * TAMAGAWA_DUAL_PRU_SLICE_ENABLE defined. In dual slice mode:
 * - Two independent Tamagawa instances (CONFIG_TAMAGAWA0, CONFIG_TAMAGAWA1) run simultaneously
 * - Each instance operates on a different PRU slice (PRU0 or PRU1)
 * - Both instances must use the same PRU-ICSS instance (validated at runtime)
 * - Both instances share common PRU-ICSS level resources
 *
 * \par Instance vs Slice:
 * - Instance: Software driver instance (CONFIG_TAMAGAWA0, CONFIG_TAMAGAWA1) with independent
 *   configuration, state, and driver handle. Configured via SysConfig.
 * - Slice: Hardware PRU-ICSS slice number (0 or 1) within PRU-ICSS where firmware executes.
 *   CONFIG_TAMAGAWAx_PRUICSS_SLICE specifies which slice each instance uses.
 *
 * \par Driver Handle Array:
 * The global handle array gAppTamagawaHandle[CONFIG_TAMAGAWA_NUM_INSTANCES] stores driver
 * handles for all configured instances:
 * - gAppTamagawaHandle[CONFIG_TAMAGAWA0]: Handle for first instance (always present)
 * - gAppTamagawaHandle[CONFIG_TAMAGAWA1]: Handle for second instance (tested only on AM261x with Single PRU Single Channel mode)
 * - CONFIG_TAMAGAWA_NUM_INSTANCES: Number of instances (defined by SysConfig)
 * ASSUMPTIONS: Loop (with index i) is used to call functions with handle as argument.
 *      - i = 0 will use CONFIG_TAMAGAWA0
 *      - i = 1 will use CONFIG_TAMAGAWA1 when TAMAGAWA_DUAL_PRU_SLICE_ENABLE is defined
 *
 * \par Shared Resources:
 * When both instances use the same PRU-ICSS instance, they share:
 * - IEP Timer: Used for periodic trigger mode, configured via first instance
 * - PRU-ICSS INTC: Interrupt controller shared across slices
 *
 * \par First instance (CONFIG_TAMAGAWA0) is used for shared resources:
 * Several operations use gAppTamagawaHandle[CONFIG_TAMAGAWA0] to access shared PRU-ICSS
 * resources. This approach works correctly because validation in tamagawa_pruicss_init()
 * ensures both instances use the same PRU-ICSS instance.
 *
 * \par Periodic Trigger Mode:
 * In periodic mode, the IEP timer automatically triggers Tamagawa transactions:
 * - Each instance/channel can have different trigger times
 * - All instances share the same IEP reset count (period)
 * - Configured via tamagawa_periodic_interface structure
 * - See tamagawa_periodic_trigger.c for detailed IEP configuration
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
#include <position_sense/tamagawa/include/tamagawa_drv.h>
#include "tamagawa_periodic_trigger.h"

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU)
/* Single channel mode firmware */
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_single_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_single_channel_pru0_bin.h>
#endif
#endif

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
/* Multi-channel mode firmware */
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_multi_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_multi_channel_pru0_bin.h>
#endif
#endif

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
/* Multi-channel load-share mode firmware */
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_multi_rtu_pru1_bin.h>
#include <tamagawa_receiver_multi_pru1_bin.h>
#include <tamagawa_receiver_multi_tx_pru1_bin.h>
#else
#include <tamagawa_receiver_multi_rtu_pru0_bin.h>
#include <tamagawa_receiver_multi_pru0_bin.h>
#include <tamagawa_receiver_multi_tx_pru0_bin.h>
#endif
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if !defined(SOC_AM261X) || (CONFIG_TAMAGAWA1_MODE != TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#error "Dual handle example using PRU0 and PRU1 is tested only with TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other combinations, update code and remove this line."
#endif

/* Single channel mode firmware */
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_single_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_single_channel_pru0_bin.h>
#endif

#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TASK_STACK_SIZE                         (4096)
#define TASK_PRIORITY                           (6)

#define TAMAGAWA_POSITION_LOOP_STOP             (0)
#define TAMAGAWA_POSITION_LOOP_START            (1)

#define TAMAGAWA_PERIODIC_MODE_CMD              (DATA_ID_0)
#define TAMAGAWA_PERIODIC_MODE_LOG_SLEEP_US     (100)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* Tamagawa Driver Handle */
tamagawa_handle gAppTamagawaHandle[CONFIG_TAMAGAWA_NUM_INSTANCES] = {NULL};

/* Tamagawa Periodic Interface Struct Instance */
tamagawa_periodic_interface gTamagawaPeriodicInterface;

/* Global variable to track position loop status */
volatile int32_t gTamagawaPositionLoopStatus;

/* Task related global variables */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void tamagawa_pruicss_init(void);
static void tamagawa_pruicss_load_run_fw(void);
static void tamagawa_display_fw_version(void);
static void tamagawa_display_menu(void);
static void tamagawa_display_result(tamagawa_handle handle, int32_t cmd);
static int32_t tamagawa_handle_rx(tamagawa_handle handle, int32_t cmd);
static int32_t tamagawa_get_command(uint8_t *adf, uint8_t *edf);
static void tamagawa_position_loop_decide_termination(void *args);
static int32_t tamagawa_loop_task_create(void);
static void tamagawa_process_periodic_command(tamagawa_handle handle[], int32_t process_dataid_cmd);
void tamagawa_main(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void tamagawa_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    uint8_t rtu_pru_id = CONFIG_TAMAGAWA0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    uint8_t tx_pru_id = CONFIG_TAMAGAWA0_PRUICSS_TX_PRU_ID;
#endif
#else
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#endif

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);

#ifdef CONFIG_TAMAGAWA0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /*
     * These checks are applicable only if both Tamagawa instances
     * use same PRU-ICSSG instance. If different instances are used,
     * these checks can be removed.
     */
#if (CONFIG_TAMAGAWA0_PRUICSS_INSTANCE != CONFIG_TAMAGAWA1_PRUICSS_INSTANCE)
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple Tamagawa handles in this example */
    DebugP_log("\r\n PRU-ICSS instance should be set uniformly for both Tamagawa instances in SysConfig.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_TAMAGAWA0_G_MUX_EN) && !defined(CONFIG_TAMAGAWA1_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both Tamagawa instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_TAMAGAWA1_G_MUX_EN) && !defined(CONFIG_TAMAGAWA0_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both Tamagawa instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif
#endif

    /* Clear PRU-ICSS DATA RAM for slice*/
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_TAMAGAWA0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    pru_id = CONFIG_TAMAGAWA1_PRUICSS_PRU_ID;

    /* Clear PRU-ICSS DATA RAM for slice*/
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_TAMAGAWA1_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
}

static void tamagawa_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = TamagawaFirmwareMultiMakeRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(TamagawaFirmwareMultiMakeRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_TAMAGAWA0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = TamagawaFirmwareMultiMakePru1_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwareMultiMakePru1_0);
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = TamagawaFirmwareMultiMakeTxPru1_0;
    uint32_t tx_pru_firmware_size = sizeof(TamagawaFirmwareMultiMakeTxPru1_0);
    uint8_t tx_pru_id = CONFIG_TAMAGAWA0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = TamagawaFirmwareMultiMakeRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(TamagawaFirmwareMultiMakeRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_TAMAGAWA0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = TamagawaFirmwareMultiMakePru0_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwareMultiMakePru0_0);
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = TamagawaFirmwareMultiMakeTxPru0_0;
    uint32_t tx_pru_firmware_size = sizeof(TamagawaFirmwareMultiMakeTxPru0_0);
    uint8_t tx_pru_id = CONFIG_TAMAGAWA0_PRUICSS_TX_PRU_ID;
#endif
#endif
#elif (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = TamagawaFirmwareMultiPru1_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwareMultiPru1_0);
#else
    const uint32_t *pru_firmware = TamagawaFirmwareMultiPru0_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwareMultiPru0_0);
#endif
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#else
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = TamagawaFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwarePru1_0);
#else
    const uint32_t *pru_firmware = TamagawaFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(TamagawaFirmwarePru0_0);
#endif
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;
#endif

#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
    /* Disable RTU-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to RTU-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_TAMAGAWA0_PRUICSS_SLICE), 0,
                                  (uint32_t *)rtu_pru_firmware, rtu_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset RTU-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable RTU-PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
    /* Disable TX-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to TX-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_TAMAGAWA0_PRUICSS_SLICE), 0,
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
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
    pru_firmware = TamagawaFirmwarePru1_0;
    pru_firmware_size = sizeof(TamagawaFirmwarePru1_0);
#else
    pru_firmware = TamagawaFirmwarePru0_0;
    pru_firmware_size = sizeof(TamagawaFirmwarePru0_0);
#endif

    pru_id = CONFIG_TAMAGAWA1_PRUICSS_PRU_ID;

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA1_PRUICSS_SLICE), 0,
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

static void tamagawa_display_result(tamagawa_handle handle, int32_t cmd)
{
    tamagawa_priv *priv;
    uint8_t xchg_index;
    const tamagawa_attrs *attrs;
    /* NULL check on handle */
    if(handle == NULL)
    {
        DebugP_log("\r\n ERROR: NULL handle in tamagawa_display_result\n");
        return;
    }

    priv = tamagawa_get_priv(handle);
    attrs = tamagawa_get_attrs(handle);
    if(attrs->load_share_enabled)
    {
        xchg_index = priv->channel;
    }
    else
    {
        xchg_index = 0;
    }


    /* Prints the position value returned by the encoder for a particular command ID */
    switch(cmd)
    {
        case DATA_ID_7:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abs, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_8:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abs, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_C:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abs, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_0:
            /* Data readout: data in one revolution */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abs, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABM: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abm, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nENID: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.enid, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tENID: 0x%x\tABM: 0x%x\tALMC: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.abs, priv->tamagawa_interface[xchg_index].rx_frames_received.enid, priv->tamagawa_interface[xchg_index].rx_frames_received.abm, priv->tamagawa_interface[xchg_index].rx_frames_received.almc, priv->tamagawa_interface[xchg_index].rx_frames_received.sf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.edf, priv->tamagawa_interface[xchg_index].rx_frames_received.adf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_interface[xchg_index].rx_frames_received.edf, priv->tamagawa_interface[xchg_index].rx_frames_received.adf, priv->tamagawa_interface[xchg_index].rx_frames_received.cf, priv->tamagawa_interface[xchg_index].rx_frames_received.crc);
            break;

        default:
            DebugP_log("\r\n| ERROR: unknown Data ID\n");
            break;
    }
}

static int32_t tamagawa_handle_rx(tamagawa_handle handle, int32_t cmd)
{
    tamagawa_priv *priv;

    /* NULL check on handle */
    if(handle == NULL)
    {
        DebugP_log("\r\n ERROR: NULL handle\n");
        return SystemP_FAILURE;
    }
    priv = tamagawa_get_priv(handle);

    DebugP_log("\r\n Parsing process started\n");
    /* Case of parsing failure */
    if(tamagawa_parse(handle, cmd) == SystemP_FAILURE)
    {
        DebugP_log("\r\n ERROR: Parsing failure\n");
        return SystemP_FAILURE;
    }

    /* Case of successful parsing, display the results after CRC check*/
    DebugP_log("\r\n Channel is  %x \n", priv->channel);
    DebugP_log("\r\n data id is %x \n", cmd);

    if(tamagawa_crc_verify(handle) == SystemP_SUCCESS)
    {
        DebugP_log("\r\n CRC success \n");
        tamagawa_display_result(handle, cmd);
    }
    else
    {
        DebugP_log("\r\n CRC Failure \n");
    }

    return SystemP_SUCCESS;
}

static int32_t tamagawa_get_command(uint8_t *adf, uint8_t *edf)
{
    int32_t cmd;
    uint32_t val;
    uint32_t i, j;
    const tamagawa_attrs *attrs;

    /* Check to make sure that the command issued is correct */
    if(DebugP_scanf("%d\n", &cmd) < 0)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* If the command is 9, start periodic trigger CMP mode with DATA ID as 0*/
    if(cmd == PERIODIC_TRIGGER_CMP_CMD)
    {
        gTamagawaPeriodicInterface.is_cap_mode = 0;  /* CMP mode */

        DebugP_log("\r| Enter IEP reset cycle count (must be greater than Tamagawa cycle time including timeout period, in IEP cycles):");
        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }

        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);
            if(attrs->load_share_enabled)
            {
                for(j = 0; j < TAMAGAWA_MAX_CHANNELS_PER_SLICE; j++)
                {
                    if((attrs->channel_mask & (1 << j)))
                    {

                        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles) for ch %u for Tamagawa instance %u: ", j, i);
                        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.periodic_trigger_count[i][j]) < 0 )
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            return SystemP_FAILURE;
                        }

                        if(gTamagawaPeriodicInterface.periodic_trigger_count[i][j] > gTamagawaPeriodicInterface.iep_reset_count)
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles) for Tamagawa instance %u: ", i);

                if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.periodic_trigger_count[i][0]) < 0 )
                {
                    DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                    return SystemP_FAILURE;
                }

                if(gTamagawaPeriodicInterface.periodic_trigger_count[i][0] > gTamagawaPeriodicInterface.iep_reset_count)
                {
                    DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                    return SystemP_FAILURE;
                }
            }
        }
    }
    /* If the command is 10, start periodic trigger CAP mode with DATA ID as 0*/
    else if(cmd == PERIODIC_TRIGGER_CAP_CMD)
    {
        gTamagawaPeriodicInterface.is_cap_mode = 1;  /* CAP mode */
#if defined(SOC_AM243X)
        DebugP_log("\r| Enter IEP SYNC0 period (in IEP cycles, used for CAP mode):");
        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }
#else
        DebugP_log("\r| Periodic CAP mode cycle time will be equal to EPWM frequency. NOTE: In SysConfig, EPWM and EPWM to IEP LATCH XBAR configuration must be done. \n|\n|\n|\n");
#endif
    }
    /* Check to make sure that the command issued is correct */
    if(cmd >= DATA_ID_NUM)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* In case of EEPROM commands, take input for Address field for different channels selected*/
    if((cmd == DATA_ID_D) || (cmd == DATA_ID_6))
    {
        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);
            uint8_t ch = 0;
            for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE ; ch++)
            {
                if(attrs->channel_mask & (1 << ch))
                {
                    if(attrs->total_channels == 1)
                    {
                        DebugP_log("\r\n| Enter EEPROM address (hex value) for Tamagawa instance %u: ", i);
                    }
                    else
                    {
                        DebugP_log("\r\n| Enter EEPROM address (hex value) for ch %d for Tamagawa instance %u: ", ch, i);
                    }
                    if(DebugP_scanf("%x\n", &val) < 0)
                    {
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: invalid EEPROM address\n|\n");
                        break;
                    }

                    if(val > TAMAGAWA_MAX_EEPROM_ADDRESS)
                    {
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: invalid EEPROM address\n|\n");
                        break;
                    }

                    *adf = (uint8_t)val;
                    if(tamagawa_update_adf(gAppTamagawaHandle[i], val, ch) != SystemP_SUCCESS)
                    {
                        /* If EEPROM address update fails, command cannot proceed with correct address.
                        * Mark command as invalid to prevent execution with wrong address. */
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: tamagawa_update_adf failed for Tamagawa instance %u\n|\n", i);
                        break;
                    }
                }
            }
        }

    }
    /* In case of EEPROM Write, take input for Address field for different channels selected*/
    if(cmd == DATA_ID_6)
    {
        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);
            uint8_t ch = 0;
            for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE ; ch++)
            {
                if(attrs->channel_mask & (1 << ch))
                {
                    if(attrs->total_channels == 1)
                    {
                        DebugP_log("\r\n| Enter EEPROM data (hex value) for Tamagawa instance %u: ", i);
                    }
                    else
                    {
                        DebugP_log("\r\n| Enter EEPROM data (hex value) for ch %d for Tamagawa instance %u: ", ch, i);
                    }
                    if(DebugP_scanf("%x\n", &val) < 0)
                    {
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: invalid EEPROM data\n|\n");
                        break;
                    }

                    if(val > TAMAGAWA_MAX_EEPROM_WRITE_DATA)
                    {
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: invalid EEPROM data\n|\n");
                        break;
                    }

                    *edf = (uint8_t)val;
                    if(tamagawa_update_edf(gAppTamagawaHandle[i], val, ch) != SystemP_SUCCESS)
                    {
                        /* If EEPROM data update fails, write command cannot proceed with correct data.
                        * Mark command as invalid to prevent execution with wrong data. */
                        cmd = DATA_ID_NUM;
                        DebugP_log("\r\n| ERROR: tamagawa_update_edf failed for Tamagawa instance %u\n|\n", i);
                        break;
                    }
                }
            }
        }
    }

    if(cmd == DATA_ID_NUM)
    {
        return SystemP_FAILURE;
    }

    return cmd;
}

static void tamagawa_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select DATA ID Code                              |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 0 : Data readout, Absolute (Data ID 0)                                       |");
    DebugP_log("\r\n| 1 : Data readout, Multi-turn (Data ID 1)                                     |");
    DebugP_log("\r\n| 2 : Data readout, Encoder-ID (Data ID 2)                                     |");
    DebugP_log("\r\n| 3 : Data readout, Absolute & Multi-turn (Data ID 3)                          |");
    DebugP_log("\r\n| 4 : Writing to EEPROM (Data ID 6)                                            |");
    DebugP_log("\r\n| 5 : Reset (Data ID 7)                                                        |");
    DebugP_log("\r\n| 6 : Reset (Data ID 8)                                                        |");
    DebugP_log("\r\n| 7 : Reset (Data ID C)                                                        |");
    DebugP_log("\r\n| 8 : Readout from EEPROM (Data ID D)                                          |");
    DebugP_log("\r\n| 9 : Start periodic continuous mode (CMP mode)                                |");
    DebugP_log("\r\n| 10: Start periodic continuous mode (CAP mode)                                |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|\n|\n");
    DebugP_log("\r\n| enter value: ");
}

static void tamagawa_display_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version(s) depending on configuration */
#if (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwareMultiPru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwareMultiPru0_0 + 1);
#endif
    DebugP_log("\r\nTamagawa firmware for Tamagawa instance 0\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#elif (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_TAMAGAWA0_CHANNEL0_ENABLED == 1)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwareMultiMakeRtuPru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwareMultiMakeRtuPru0_0 + 1);
#endif
    DebugP_log("\r\nTAMAGAWA firmware for channel 0 (RTU-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL1_ENABLED == 1)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwareMultiMakePru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwareMultiMakePru0_0 + 1);
#endif
    DebugP_log("\r\nTAMAGAWA firmware for channel 1 (PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_TAMAGAWA0_CHANNEL2_ENABLED == 1)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwareMultiMakeTxPru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwareMultiMakeTxPru0_0 + 1);
#endif
    DebugP_log("\r\nTAMAGAWA firmware for channel 2 (TX-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#elif (CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif
    DebugP_log("\r\nTamagawa firmware for Tamagawa instance 0\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif
    DebugP_log("\r\nTamagawa firmware for Tamagawa instance 1\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
}

static void tamagawa_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gTamagawaPositionLoopStatus = TAMAGAWA_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t tamagawa_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "tamagawa_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)tamagawa_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: TaskP_construct() for tamagawa_position_loop_decide_termination failed\n");
    }

    return status;
}

static void tamagawa_process_periodic_command(tamagawa_handle handle[], int32_t process_dataid_cmd)
{
    /* NOTE:
     * - Any function call failure will lead to exit of tamagawa_process_periodic_command function
     * - Switch back to host trigger mode is outside this function
     */
    uint32_t i;
    const tamagawa_attrs *attrs;

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        if(gTamagawaPeriodicInterface.is_cap_mode)
        {
            if(tamagawa_config_periodic_trigger_cap_mode(handle[i]) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: tamagawa_config_periodic_trigger_cap_mode failed for Tamagawa instance %u\r\n|\r\n|\n", i);
                return;
            }
        }
        else
        {
            if(tamagawa_config_periodic_trigger_cmp_mode(handle[i]) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: tamagawa_config_periodic_trigger_cmp_mode failed for Tamagawa instance %u\r\n|\r\n|\n", i);
                return;
            }

        }
    }

    if(tamagawa_loop_task_create() != SystemP_SUCCESS)
    {
        return;
    }

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        gTamagawaPeriodicInterface.handle[i] = handle[i];
    }
    /* Assuming that periodic_trigger_count[] and iep_reset_count values are set in tamagawa_get_command() */


    if(tamagawa_config_periodic_mode(&gTamagawaPeriodicInterface) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_config_periodic_mode failed\r\n|\r\n|\n");
        return;
    }

    gTamagawaPositionLoopStatus = TAMAGAWA_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| Press Enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        if(tamagawa_update_data_id(handle[i], process_dataid_cmd) != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: tamagawa_update_data_id failed for Tamagawa instance %u\r\n|\r\n|\n", i);
            return;
        }
    }

    /* In case of EEPROM commands, calculate the CRC for the different channels selected */
    if((process_dataid_cmd == DATA_ID_6) || (process_dataid_cmd == DATA_ID_D))
    {
        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            attrs = tamagawa_get_attrs(handle[i]);
            uint8_t ch = 0;
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
            {
                if(attrs->channel_mask & (1 << ch))
                {
                    if(tamagawa_update_crc(handle[i], process_dataid_cmd, ch) != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: tamagawa_update_crc failed for channel %d of Tamagawa instance %u\r\n|\r\n|\n", ch, i);
                        return;
                    }
                }
            }
        }
    }

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        if(tamagawa_command_process(handle[i], process_dataid_cmd) != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: tamagawa_command_process failed for Tamagawa instance %u\r\n|\r\n|\n", i);
            return;
        }
    }

    while(1)
    {
        if(gTamagawaPositionLoopStatus == TAMAGAWA_POSITION_LOOP_STOP)
        {
            if(tamagawa_stop_periodic_mode(&gTamagawaPeriodicInterface) != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: tamagawa_stop_periodic_mode failed\r\n|\r\n|\n");
            }
            return;
        }
        else
        {
            for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
            {
                attrs = tamagawa_get_attrs(handle[i]);
                if(attrs->total_channels > 1)
                {
                    DebugP_log("\r\n Multi-channel mode is enabled for Tamagawa instance %u\n\n", i);

                    uint8_t ch;
                    for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
                    {
                        if(attrs->channel_mask & (1 << ch))
                        {
                            if(tamagawa_multi_channel_set_cur(handle[i], ch) != SystemP_SUCCESS)
                            {
                                DebugP_log("\r| ERROR: tamagawa_multi_channel_set_cur failed for channel %d of Tamagawa instance %u\r\n|\r\n|\n", ch, i);
                                return;
                            }
                            DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                            if(tamagawa_handle_rx(handle[i], process_dataid_cmd) != SystemP_SUCCESS)
                            {
                                DebugP_log("\r| ERROR: tamagawa_handle_rx failed for channel %d of Tamagawa instance %u\r\n|\r\n|\n", ch, i);
                                return;
                            }
                        }
                    }
                }
                else
                {
                    DebugP_log("\r\n Single-channel mode is enabled for Tamagawa instance %u\n\n", i);
                    if(tamagawa_handle_rx(handle[i], process_dataid_cmd) != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: tamagawa_handle_rx failed for Tamagawa instance %u\r\n|\r\n|\n", i);
                        return;
                    }
                }
            }

            ClockP_usleep(TAMAGAWA_PERIODIC_MODE_LOG_SLEEP_US);
        }
    }
    return;
}

/**
 * \brief   Tamagawa diagnostic application main function
 *
 * \details This function implements the main diagnostic application flow for Tamagawa
 *          encoder interface. It initializes the Tamagawa driver, loads and starts PRU
 *          firmware, and provides an interactive menu-driven interface for various
 *          encoder operations including:
 *          - Position data acquisition (single-shot and continuous)
 *          - Absolute position data readout (DATA_ID_0, DATA_ID_1)
 *          - Encoder ID readout (DATA_ID_2)
 *          - Multi-turn data with encoder status (DATA_ID_3)
 *          - EEPROM read/write operations (DATA_ID_6, DATA_ID_D)
 *          - Reset commands (DATA_ID_7, DATA_ID_8, DATA_ID_C)
 *          - Periodic trigger mode configuration
 *
 *          Flow:
 *          1. Initialize SoC drivers and board drivers
 *          2. Enable booster pack power pins if configured
 *          3. Initialize PRU-ICSS subsystem
 *          4. Initialize Tamagawa driver
 *          5. Load and run PRU firmware
 *          6. Display firmware version and selected channels
 *          7. Enter interactive menu loop for encoder operations
 *          8. Process user commands and display results
 *          9. De-initialize on exit
 *
 *          NOTE on driver APIs:
 *          Tamagawa driver APIs use a simplified validation approach for optimal performance:
 *          - **Handle validation**: All public APIs validate the handle parameter for NULL
 *          - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *          - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *            are validated once during tamagawa_init() and assumed valid in subsequent API calls
 *          - This strategy reduces overhead in time-critical data path functions.
 *
 *          Supported encoder operations:
 *          - Single-shot position readout using host trigger mode
 *          - Continuous position sampling using periodic trigger mode with IEP timer
 *          - EEPROM access for encoder configuration (address 0-127, data 0-255)
 *          - CRC verification for data integrity
 *          - Multi-channel support (up to 3 channels per PRU slice)
 *          - Dual PRU slice operation for AM261x devices
 *
 * \param[in]   args    Unused parameter (required by task creation API)
 */
void tamagawa_main(void *args)
{
    uint32_t i;
    const tamagawa_attrs *attrs;
    tamagawa_params tamagawa_params;
    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                          */
    /* ========================================================================== */
    Drivers_open();          /* Open SoC drivers */
    Board_driversOpen();     /* Open board-specific drivers */

/* Set pin high for Enabling ch0 in booster pack */
#if (CONFIG_TAMAGAWA0_BOOSTER_PACK && CONFIG_TAMAGAWA0_CHANNEL0_ENABLED)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
/* Set pin high for Enabling ch2 in booster pack */
#if (CONFIG_TAMAGAWA0_BOOSTER_PACK && CONFIG_TAMAGAWA0_CHANNEL2_ENABLED)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS and Tamagawa driver                           */
    /* ========================================================================== */

    /* Initialize PRU-ICSS instance, initialize DRAM, and disable PRU cores */
    tamagawa_pruicss_init();
    DebugP_log("\r\n\nTamagawa PRU-ICSS init done\n\n");

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        DebugP_log("\r\n|------------------------------------------------------------------------------|");
        DebugP_log("\r\n Tamagawa Instance %u", i);

        /* Initialize Tamagawa parameters with defaults and set PRU-ICSS handle */
        tamagawa_params_init(&tamagawa_params);
        tamagawa_params.pruicss_handle = gPruIcssXHandle;

        /* Default delay values are used:
         *   - cmd_wait_delay_us = 100 us (delay for command wait loop)
         *   - max_wait_loop_count = 50 (number of wait loop iterations in tamagawa_command_wait())
         * If needed, these can be modified before calling tamagawa_init():
         *   tamagawa_params.cmd_wait_delay_us = <custom_value>;
         *   tamagawa_params.max_wait_loop_count = <custom_value>;
         */

        /* Initialize Tamagawa driver instance
         * This calls: tamagawa_config_clr_cfg0(), tamagawa_config_channel(), tamagawa_set_baudrate(),
         * and tamagawa_config_host_trigger() */
        gAppTamagawaHandle[i] = tamagawa_init(i, &tamagawa_params);
        if(gAppTamagawaHandle[i] == NULL)
        {
            DebugP_log("\r\nERROR: Tamagawa initialization failed for instance %u\n", i);
            return;
        }

        /* Display HW instances used, operation mode and enabled channels */
        attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);
        DebugP_log("\r\n PRU-ICSS instance: %u, PRU-ICSS slice number: %u\n", attrs->pruicss_instance, attrs->pruicss_slice);
        if(attrs->mode == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
        {
            /* Multi-channel single PRU mode: Multiple channels handled by one PRU core */
            DebugP_log("\r\nTamagawa Multi channel, Single PRU Demo application is running......\n");
        }
        else
        {
            /* Single channel single PRU mode: One channel on one PRU core */
            DebugP_log("\r\nTamagawa Single channel, Single PRU Demo application is running......\n");
        }

        DebugP_log("\r\nChannel(s) selected: %s %s %s \n\n\n",
                    attrs->channel_mask & (1 << 0) ? "0" : "",
                    attrs->channel_mask & (1 << 1) ? "1" : "",
                    attrs->channel_mask & (1 << 2) ? "2" : "");
        DebugP_log("\r\n|------------------------------------------------------------------------------|\n\n");
    }

    /* ========================================================================== */
    /* STEP 3: Load and run PRU firmware                                          */
    /* ========================================================================== */

    /* Display Tamagawa firmware version from PRU firmware image */
    tamagawa_display_fw_version();

    /* Load PRU firmware image to instruction RAM and enable PRU cores */
    tamagawa_pruicss_load_run_fw();

    DebugP_log("\r\nTamagawa PRU-ICSS firmware loaded and running\n\n\n");

    /* ========================================================================== */
    /* STEP 4: Interactive menu loop for encoder operations                      */
    /* ========================================================================== */

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    DebugP_log("\r\n| In host trigger mode, same command will be run on both Tamagawa instances    |");
    DebugP_log("\r\n| one after the other.                                                         |\n\n");
#endif

    while(1)
    {
        /*
         * Initialized to zero to remove the compiler warning about the variable being uninitialized.
         */
        uint8_t adf = 0, edf = 0;
        int32_t cmd;

        /* Display menu and get user command input */
        tamagawa_display_menu();
        cmd = tamagawa_get_command(&adf, &edf);

        /* Skip iteration if invalid command */
        if(cmd == SystemP_FAILURE)
        {
            continue;
        }

        /* Handle periodic trigger mode - continuous position sampling using IEP timer */
        if(cmd == PERIODIC_TRIGGER_CMP_CMD || cmd == PERIODIC_TRIGGER_CAP_CMD)
        {

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
            DebugP_log("\r\n IEP is common for both Tamagawa instances. Same IEP periodic cycle will be used for both instances.\n");
#endif

            DebugP_log("\r\n\n Switching to periodic trigger mode");

            /* Switching to periodic mode using tamagawa_config_periodic_trigger() is done
             * inside tamagawa_process_periodic_command */

            /* Process continuous position readout using DATA_ID_0 */
            tamagawa_process_periodic_command(gAppTamagawaHandle, TAMAGAWA_PERIODIC_MODE_CMD);

            /* Switch back to host trigger mode for menu-driven operation */
            DebugP_log("\r\n\n Switching to host trigger mode");

            for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
            {
                if(tamagawa_config_host_trigger(gAppTamagawaHandle[i]) != SystemP_SUCCESS)
                {
                    /* NOTE: If this fails, driver may remain in periodic mode causing subsequent
                     * host-triggered commands to fail. */
                    DebugP_log("\r| ERROR: tamagawa_config_host_trigger failed for Tamagawa instance %u\r\n|\r\n|\n", i);
                }
            }
        }

        /* Validate DATA_ID command is within valid range */
        if(cmd < 0 || cmd >= DATA_ID_NUM)
        {
            continue;
        }

        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            /* Update DATA_ID for encoder command (configures which data to retrieve from encoder) */
            if(tamagawa_update_data_id(gAppTamagawaHandle[i], cmd) != SystemP_SUCCESS)
            {
                /* NOTE: If DATA_ID update fails, subsequent command processing would use wrong/stale DATA_ID.
                * Skip this command iteration to prevent incorrect encoder operation. */
                DebugP_log("\r\n| ERROR: tamagawa_update_data_id failed for Tamagawa instance %u\n", i);
                continue;
            }

            /* For EEPROM read/write commands (DATA_ID_6, DATA_ID_D), calculate and update CRC.
            * CRC is required to ensure data integrity when accessing encoder EEPROM. */
            if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
            {
                attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);
                uint8_t ch = 0;
                uint8_t crc_failed = 0;
                for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE ; ch++)
                {
                    if(attrs->channel_mask & (1 << ch))
                    {
                        if(tamagawa_update_crc(gAppTamagawaHandle[i], cmd, ch) != SystemP_SUCCESS)
                        {
                            /* NOTE: If CRC calculation fails, EEPROM command would proceed without proper CRC verification.
                            * Mark as failed to skip command execution and prevent data corruption. */
                            DebugP_log("\r\n| ERROR: tamagawa_update_crc failed for channel %d of Tamagawa instance %u\n", ch, i);
                            crc_failed = 1;
                        }
                    }
                }
                if(crc_failed)
                {
                    continue;
                }
            }
        }

        /* Execute Tamagawa command transaction with encoder.
         * This triggers PRU firmware to send command to encoder and wait for response. */


        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            if(tamagawa_command_process(gAppTamagawaHandle[i], cmd) != SystemP_SUCCESS)
            {
                /* NOTE: If command processing fails, no valid data is available to parse.
                 * Skip data processing for this slice to prevent parsing invalid/stale data. */
                DebugP_log("\r\n| ERROR: tamagawa_command_process failed for Tamagawa instance %u\n", i);
                continue;
            }
        }

        for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
        {
            attrs = tamagawa_get_attrs(gAppTamagawaHandle[i]);

            /* Parse and display received encoder data based on channel configuration */
            if(attrs->total_channels > 1)
            {
                /* Multi-channel mode: Process each enabled channel separately */
                DebugP_log("\r\n Multi-channel mode is enabled for Tamagawa instance %u\n\n", i);
                uint8_t ch;
                for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS_PER_SLICE; ch++)
                {
                    if(attrs->channel_mask & (1 << ch))
                    {
                        if(tamagawa_multi_channel_set_cur(gAppTamagawaHandle[i], ch) != SystemP_SUCCESS)
                        {
                            /* NOTE: If channel selection fails, subsequent RX parsing would use wrong channel.
                            * Skip this channel to prevent incorrect data interpretation. */
                            DebugP_log("\r\n| ERROR: tamagawa_multi_channel_set_cur failed for channel %d of Tamagawa instance %u\n", ch, i);
                            continue;
                        }
                        DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d (Tamagawa instance %u)\n", ch, i);
                        if(tamagawa_handle_rx(gAppTamagawaHandle[i], cmd) != SystemP_SUCCESS)
                        {
                            DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed for channel %d of Tamagawa instance %u\n", ch, i);
                        }
                    }
                }
            }
            else
            {
                /* Single-channel mode: Process single channel data directly */
                DebugP_log("\r\n Single-channel mode is enabled for Tamagawa instance %u\n\n", i);
                if(tamagawa_handle_rx(gAppTamagawaHandle[i], cmd) != SystemP_SUCCESS)
                {
                    DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed for Tamagawa instance %u\n", i);
                }
            }
        }
    }

    for(i = 0; i < CONFIG_TAMAGAWA_NUM_INSTANCES; i++)
    {
        tamagawa_deinit(gAppTamagawaHandle[i]);
    }

    Board_driversClose();
    Drivers_close();
}
