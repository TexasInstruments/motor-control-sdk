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
 * \file  endat_diagnostic.c
 *
 * \brief EnDAT 2.2 encoder diagnostic application
 *
 * This application provides comprehensive diagnostic and testing capabilities for
 * EnDAT 2.2 encoders, including position reads, parameter access via MRS commands,
 * continuous monitoring, recovery time measurement, and periodic trigger mode.
 *
 * \par Key Features:
 * - Support for EnDAT 2.1 and 2.2 protocol commands
 * - Single-channel and load-share (multi-channel) operation modes
 * - MRS (Memory Range Selection) parameter read/write operations
 * - Recovery time testing with error detection
 * - Position value validation and safety information handling
 * - IEP timer-based periodic trigger mode for real-time updates
 *
 * \par Application Flow:
 * 1. Initialize PRU-ICSS and disable cores
 * 2. Initialize EnDAT driver (endat_init)
 * 3. Load and run PRU firmware
 * 4. Get encoder configuration and resolution
 * 5. Run diagnostic tests via interactive menu
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
#include <position_sense/endat/include/endat_drv.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "endat_periodic_trigger.h"

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
#if CONFIG_ENDAT0_PRUICSS_SLICE == 1
#include <endat_receiver_multi_pru1_bin.h>
#else
#include <endat_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if CONFIG_ENDAT0_PRUICSS_SLICE == 1
#if CONFIG_ENDAT0_CHANNEL0_ENABLED
#include <endat_receiver_multi_rtu_pru1_bin.h>
#endif
#if CONFIG_ENDAT0_CHANNEL1_ENABLED
#include <endat_receiver_multi_pru1_bin.h>
#endif
#if CONFIG_ENDAT0_CHANNEL2_ENABLED
#include <endat_receiver_multi_tx_pru1_bin.h>
#endif
#else
#if CONFIG_ENDAT0_CHANNEL0_ENABLED
#include <endat_receiver_multi_rtu_pru0_bin.h>
#endif
#if CONFIG_ENDAT0_CHANNEL1_ENABLED
#include <endat_receiver_multi_pru0_bin.h>
#endif
#if CONFIG_ENDAT0_CHANNEL2_ENABLED
#include <endat_receiver_multi_tx_pru0_bin.h>
#endif
#endif
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
#if CONFIG_ENDAT0_PRUICSS_SLICE == 1
#include <endat_receiver_pru1_bin.h>
#else
#include <endat_receiver_pru0_bin.h>
#endif
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
#if CONFIG_ENDAT1_PRUICSS_SLICE == 1
#include <endat_receiver_pru1_bin.h>
#else
#include <endat_receiver_pru0_bin.h>
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Timeout and Task Configuration */
#define WAIT_5_SECOND                       (5000)
#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

/* Delay Values (in microseconds) */
#define POSITION_LOOP_DISPLAY_DELAY_US      (100U)   /* Delay between position display updates */
#define POSITION_LOOP_2_1_DELAY_US          (500U)   /* Delay for EnDAT 2.1 position loop display */
#define POSITION_VAL2_READY_DELAY_MULT      (3U)     /* Multiplier for position value 2 ready wait (us * 3) */
#define ENDAT_PERIODIC_MODE_POLL_SLEEP_US   (1)

/* Position Loop Control */
#define ENDAT_POSITION_LOOP_STOP            (0)
#define ENDAT_POSITION_LOOP_START           (1)

/* MRS Position Value 2 Word Addresses */
#define MRS_POS_VAL2_WORD1                  (0x42U)
#define MRS_POS_VAL2_WORD2                  (0x43U)
#define MRS_POS_VAL2_WORD3                  (0x44U)

#if defined(SOC_AM261X) || defined(SOC_AM263X) || defined(SOC_AM263PX)
/* Translate the TCM local view addr to SoC view addr */
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5SS0_CORE0_TCMB_U_BASE+(x - CSL_MSS_TCMB_RAM_BASE))
#else
/* Translate the TCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))
#endif

/* Command Validation Macros */
#define VALID_CONT_MODE_CMD(x) ((x) == 101 || (x) == 104 || (x) == 107 || (x) == 111 || (x) == 200 || (x) == 201)

#define VALID_HOST_CMD(x) ((x == 100) || ((x) == 102) || ((x) == 103) || ((x) == 105) || \
                           ((x) == 106) || ((x) == 108) || ((x) == 109) || ((x) == 110) || ((x)== 112))

#define HAVE_COMMAND_SUPPLEMENT(x) (((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 7) || \
                                    ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 13) || ((x) == 14) || \
                                    ((x) == 100) || ((x) == 101) || ((x)== 103) || ((x) == 105) || ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109)  || ((x) == 200) || ((x) == 201) || ((x) == 112))
#define ENDAT_POSITION_CMD(x) (((x) == 1) || ((x) == 8) ||((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 13))


/* Size of the PRU instruction memory in bytes.*/
#define PRU_IRAM_SIZE   ( 12 * 1024 )    /* 12KB */

/* Size of the RTU PRU instruction memory in bytes. */
#define RTUPRU_IRAM_SIZE   ( 8 * 1024 )    /* 8KB */

/* Size of the TX PRU instruction memory in bytes.*/
#define TXPRU_IRAM_SIZE   ( 6 * 1024 )    /* 6KB */

/* ========================================================================== */
/*                         Structure Definitions                              */
/* ========================================================================== */

typedef union endat_position_type_u
{
    float angle;
    uint64_t length;
} endat_position_type;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* EnDAT Driver Handle Array - indexed by instance (0 to CONFIG_ENDAT_NUM_INSTANCES-1) */
endat_handle gAppEndatHandle[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};

/* EnDAT Periodic Interface Struct Instance */
endat_periodic_interface gEndatPeriodicInterface;

/* Multi-instance Position Data Arrays [instance][channel] */
static endat_format_data gAppEndatFormatDataMtrCtrl[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint32_t gAppEndatMtrCtrlCrcErr[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint32_t gAppEndat22CrcPositionErrCnt[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint32_t gAppEndat22CrcAddinfo1ErrCnt[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint64_t gAppEndat22PosVal2[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint32_t gAppEndat22LoopMrs[CONFIG_ENDAT_NUM_INSTANCES];

/* Propagation Delay Arrays [instance][channel] */
static uint32_t gAppEndatPropDelay[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];
static uint32_t gAppEndatPropDelayMax[CONFIG_ENDAT_NUM_INSTANCES];

/* Global variable to track position loop status */
volatile int32_t gEndatPositionLoopStatus;

/* IRQ count from periodic trigger (defined in endat_periodic_trigger.c) */
extern volatile uint32_t gPruEndatIrqCnt[CONFIG_ENDAT_NUM_INSTANCES][ENDAT_NUM_CH_PER_SLICE_MAX];

volatile uint64_t gPositionLoopIrqCount = 0;

/* Task related global variables */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* Utility buffers */
static char gUartBuffer[256];
static char gPrintfDumpBuffer[21];

/* EnDAT channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) endat_ch_rx_info_array gEndatChInfo[CONFIG_ENDAT_NUM_INSTANCES];

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/* Utility functions */
char *uint64_to_str(uint64_t x);

/* PRU-ICSS initialization and firmware loading */

/**
 * \brief   Initialize PRU-ICSS for EnDAT operation
 *
 * \details Opens PRU-ICSS instance, configures G_MUX, sets constant tables,
 *          clears data RAM, and disables PRU cores in preparation for firmware loading.
 *
 *          Implemented configurations:
 *          - Instance 0: All modes implemented (single-channel single-PRU,
 *            multi-channel single-PRU, multi-channel multi-PRU load-share)
 *          - Instance 1 (dual slice): Only single-channel single-PRU mode
 *            implemented. Tested on AM261x platform.
 *
 * \note    In dual slice mode (ENDAT_DUAL_PRU_SLICE_ENABLE), this function
 *          initializes both PRU slices. Instance 0 uses CONFIG_ENDAT0_* settings,
 *          Instance 1 uses CONFIG_ENDAT1_* settings. Only single-channel single-PRU
 *          configuration is implemented for Instance 1.
 */
static void endat_pruicss_init(void);
/**
 * \brief   Load and run PRU firmware for EnDAT encoder interface
 *
 * \details This function loads the appropriate PRU firmware binary into PRUICSS
 *          instruction memory and starts the PRU core(s). Firmware selection is
 *          based on the EnDAT mode (single/multi-channel, load-share) and PRU
 *          slice configuration.
 *
 *          Implemented configurations:
 *          - Instance 0: All modes implemented (single-channel single-PRU,
 *            multi-channel single-PRU, multi-channel multi-PRU load-share)
 *          - Instance 1 (dual slice): Only single-channel single-PRU mode
 *            implemented. Tested on AM261x platform.
 *
 * \note    This function does NOT call endat_wait_initialization(). Caller must
 *          wait for firmware initialization separately after calling this function.
 *
 * \note    In dual slice mode (ENDAT_DUAL_PRU_SLICE_ENABLE), this function is
 *          called for both instances. Instance 0 uses CONFIG_ENDAT0_* settings,
 *          Instance 1 uses CONFIG_ENDAT1_* settings. Only single-channel single-PRU
 *          configuration is implemented for Instance 1.
 *
 * \param   handle    EnDAT driver handle for the instance
 *
 * \retval  SystemP_SUCCESS    Firmware loaded and started successfully
 * \retval  SystemP_FAILURE    Firmware load/start failed
 */
static void endat_pruicss_load_run_fw(void);

/**
 * \brief   Display PRU firmware version information
 *
 * \details Reads and displays the firmware version from the loaded PRU firmware
 *          binary. Version format: major.minor.patch (release/internal).
 *
 *          Implemented configurations:
 *          - Instance 0 single-channel single-PRU: Displays firmware version
 *          - Instance 0 multi-channel single-PRU: Displays firmware version
 *          - Instance 0 multi-channel multi-PRU: Displays version for each enabled
 *            channel (RTU-PRU for channel 0, PRU for channel 1, TX-PRU for channel 2)
 *          - Instance 1 (dual slice): Displays firmware version for single-channel
 *            single-PRU configuration only
 */
static void endat_display_fw_version(void);
/**
 * \brief   Display interactive diagnostic command menu
 */
static void endat_print_menu(endat_handle handle);
/* Get command from user */
static int32_t endat_get_command(endat_handle handle);

static void endat_recvd_print(endat_handle handle, int32_t cmd, endat_format_data *endat_data, int32_t crc);
static void endat_display_raw_data(endat_handle handle, int32_t cmd);
static void endat_print_encoder_info(endat_handle handle);
static void endat_print_position_header(endat_handle handle, uint32_t continuous, uint32_t is_2_2);
static void endat_print_position_loop(endat_handle handle, uint32_t continuous, uint32_t is_2_2, uint32_t ch);

/* Position command processing */
void endat_position_loop(void);
static void endat_process_2_1_position_command(endat_handle handle);
static void endat_process_2_2_position_command(endat_handle handle);
static uint16_t endat_handle_2_2_position_command(endat_handle handle, uint32_t cmd, endat_cmd_supplement *cmd_supplement, uint32_t a0);

/* Command processing and handling */
static int32_t endat_get_command_supplement(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement);
static int32_t endat_handle_user(endat_handle handle[CONFIG_ENDAT_NUM_INSTANCES], endat_cmd_supplement cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES]);
static void endat_process_host_command(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement);
static int32_t endat_process_continuous_mode_command(endat_handle handle[CONFIG_ENDAT_NUM_INSTANCES], int32_t cmd, endat_cmd_supplement cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES]);
static void endat_handle_rx(endat_handle handle, int32_t cmd);

/* Position loop control */
static int32_t endat_calc_position_period(uint32_t freq);
static uint32_t endat_get_position_loop_chars(uint32_t multi_turn_res, uint32_t continuous, uint32_t is_2_2);
static int32_t endat_loop_task_create(void);
static void endat_position_loop_decide_termination(void *args);
static void endat_loop_timer_create(int32_t us);

/* Testing and diagnostics */
static uint32_t endat_do_sanity_tst_delay(uint32_t delay);

/* ========================================================================== */
/*                       Function Definitions                                 */
/* ========================================================================== */

char * uint64_to_str (uint64_t x)
{
    char *b = gPrintfDumpBuffer + sizeof(gPrintfDumpBuffer);
    *(--b) = '\0';
    do
    {
        *(--b) = '0' + (x % 10);
        x /= 10;
    } while (x);

    return b;
}

static void endat_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    uint8_t rtu_pru_id = CONFIG_ENDAT0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    uint8_t tx_pru_id = CONFIG_ENDAT0_PRUICSS_TX_PRU_ID;
#endif
#else
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#endif

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    if(gPruIcssXHandle == NULL)
    {
        DebugP_log("\r\n ERROR: PRUICSS_open failed - NULL handle returned\n");
        DebugP_assert(0);
    }

#ifdef CONFIG_ENDAT0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)

#if !defined(SOC_AM261X) || (CONFIG_ENDAT0_MODE != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU) || (CONFIG_ENDAT1_MODE != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    DebugP_log("Dual handle example using PRU0 and PRU1 is tested only with ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU mode on AM261x. For enabling other combinations, update code and remove this check.");
    DebugP_assert(0);
#endif
    /*
     * These checks are applicable only if both EnDAT instances
     * use same PRU-ICSSG instance. If different instances are used,
     * these checks can be removed.
     */
#if (CONFIG_ENDAT0_PRUICSS_INSTANCE != CONFIG_ENDAT1_PRUICSS_INSTANCE)
    /* ASSUMPTION: Same PRU-ICSS instance is used for multiple EnDAT handles in this example */
    DebugP_log("\r\n PRU-ICSS instance should be set uniformly for both EnDAT instances in SysConfig.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_ENDAT0_G_MUX_EN) && !defined(CONFIG_ENDAT1_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both EnDAT instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif

#if defined(CONFIG_ENDAT1_G_MUX_EN) && !defined(CONFIG_ENDAT0_G_MUX_EN)
    DebugP_log("\r\n G_MUX_EN should be set uniformly for both EnDAT instances in SysConfig. G_MUX_EN is a PRU-ICSSG instance level configuration.");
    DebugP_assert(0);
#endif
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    /*
     * Set the constant table C28 for tx pru
     * configuring the constant table C28 to point to the TX counter
     * register (CNTR). The counter is needed in firmware for adding waits and time stamps.
     */
#if CONFIG_ENDAT0_PRUICSS_INSTANCE == 1
#if CONFIG_ENDAT0_PRUICSS_SLICE == 1
    status = PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_ENDAT0_PRUICSS_TX_PRU_ID, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
    DebugP_assert(SystemP_SUCCESS == status);
#else
    status = PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_ENDAT0_PRUICSS_TX_PRU_ID, PRUICSS_CONST_TBL_ENTRY_C28, 0xA50);
    DebugP_assert(SystemP_SUCCESS == status);
#endif /* CONFIG_ENDAT0_PRUICSS_SLICE == 1 */
#else
#if CONFIG_ENDAT0_PRUICSS_SLICE == 1
    status = PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_ENDAT0_PRUICSS_TX_PRU_ID, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
    DebugP_assert(SystemP_SUCCESS == status);
#else
    status = PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_ENDAT0_PRUICSS_TX_PRU_ID, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
    DebugP_assert(SystemP_SUCCESS == status);
#endif /* CONFIG_ENDAT0_PRUICSS_SLICE == 1 */
#endif /* CONFIG_ENDAT0_PRUICSS_INSTANCE == 1 */
#endif /* CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU */

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_ENDAT0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    pru_id = CONFIG_ENDAT1_PRUICSS_PRU_ID;

    /* Clear PRU-ICSS DATA RAM based on slice */
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_ENDAT1_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    /* Disable PRU cores for second slice */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
}

static void endat_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = EnDatFirmwareMultiMakeRtuPru1_0;
    uint32_t rtu_pru_firmware_size = sizeof(EnDatFirmwareMultiMakeRtuPru1_0);
    uint8_t rtu_pru_id = CONFIG_ENDAT0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = EnDatFirmwareMultiMakePru1_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwareMultiMakePru1_0);
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = EnDatFirmwareMultiMakeTxPru1_0;
    uint32_t tx_pru_firmware_size = sizeof(EnDatFirmwareMultiMakeTxPru1_0);
    uint8_t tx_pru_id = CONFIG_ENDAT0_PRUICSS_TX_PRU_ID;
#endif
#else
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    const uint32_t *rtu_pru_firmware = EnDatFirmwareMultiMakeRtuPru0_0;
    uint32_t rtu_pru_firmware_size = sizeof(EnDatFirmwareMultiMakeRtuPru0_0);
    uint8_t rtu_pru_id = CONFIG_ENDAT0_PRUICSS_RTU_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    const uint32_t *pru_firmware = EnDatFirmwareMultiMakePru0_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwareMultiMakePru0_0);
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    const uint32_t *tx_pru_firmware = EnDatFirmwareMultiMakeTxPru0_0;
    uint32_t tx_pru_firmware_size = sizeof(EnDatFirmwareMultiMakeTxPru0_0);
    uint8_t tx_pru_id = CONFIG_ENDAT0_PRUICSS_TX_PRU_ID;
#endif
#endif
#elif (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = EnDatFirmwareMultiPru1_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwareMultiPru1_0);
#else
    const uint32_t *pru_firmware = EnDatFirmwareMultiPru0_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwareMultiPru0_0);
#endif
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#else
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    const uint32_t *pru_firmware = EnDatFirmwarePru1_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwarePru1_0);
#else
    const uint32_t *pru_firmware = EnDatFirmwarePru0_0;
    uint32_t pru_firmware_size = sizeof(EnDatFirmwarePru0_0);
#endif
    uint8_t pru_id = CONFIG_ENDAT0_PRUICSS_PRU_ID;
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED == 1)
    /*Validate firmware size*/
    if(rtu_pru_firmware_size > RTUPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: RTU Firmware binary size (%d) exceeds available IRAM size (%d)\n", rtu_pru_firmware_size, RTUPRU_IRAM_SIZE);
    }

    /* Disable RTU-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to RTU-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(CONFIG_ENDAT0_PRUICSS_SLICE), 0,
                                  (uint32_t *)rtu_pru_firmware, rtu_pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset RTU-PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable RTU-PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, rtu_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED == 1)
    /*Validate firmware size*/
    if(pru_firmware_size > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", pru_firmware_size, PRU_IRAM_SIZE);
    }

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_ENDAT0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED == 1)
    /*Validate firmware size*/
    if(tx_pru_firmware_size > TXPRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: TX PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", tx_pru_firmware_size, TXPRU_IRAM_SIZE);
    }

    /* Disable TX-PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, tx_pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to TX-PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_TX_PRU(CONFIG_ENDAT0_PRUICSS_SLICE), 0,
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
    /*Validate firmware size*/
    if(pru_firmware_size > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", pru_firmware_size, PRU_IRAM_SIZE);
    }

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_ENDAT0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
  /* End Instance 0 firmware loading */

/* ========================================================================== */
/* Load firmware for Instance 1 (CONFIG_ENDAT1) - Dual Slice Mode           */
/* ========================================================================== */
/*
 * NOTE: Dual slice mode (Instance 1) only implements single-channel single-PRU
 *       configuration. This mode has been tested on AM261x platform.
 *       Multi-channel and multi-PRU modes are NOT implemented for dual slice.
 */
#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* Load and run firmware for second slice */
#if (CONFIG_ENDAT1_PRUICSS_SLICE == 1)
    pru_firmware = EnDatFirmwarePru1_0;
    pru_firmware_size = sizeof(EnDatFirmwarePru1_0);
#else
    pru_firmware = EnDatFirmwarePru0_0;
    pru_firmware_size = sizeof(EnDatFirmwarePru0_0);
#endif
    pru_id = CONFIG_ENDAT1_PRUICSS_PRU_ID;

    /*Validate firmware size*/
    if(pru_firmware_size > PRU_IRAM_SIZE)
    {
        DebugP_log("ERROR: PRU Firmware binary size (%d) exceeds available IRAM size (%d)\n", pru_firmware_size, PRU_IRAM_SIZE);
    }

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_ENDAT1_PRUICSS_SLICE), 0,
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

static void endat_display_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version(s) depending on configuration */
#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwareMultiPru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwareMultiPru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#elif (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#if (CONFIG_ENDAT0_CHANNEL0_ENABLED)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwareMultiMakeRtuPru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwareMultiMakeRtuPru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware for channel 0 (RTU-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_ENDAT0_CHANNEL1_ENABLED)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwareMultiMakePru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwareMultiMakePru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware for channel 1 (PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#if (CONFIG_ENDAT0_CHANNEL2_ENABLED)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwareMultiMakeTxPru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwareMultiMakeTxPru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware for channel 2 (TX-PRU)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
#elif (CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (CONFIG_ENDAT0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif

#if defined(ENDAT_DUAL_PRU_SLICE_ENABLE)
    /* Print firmware version for second slice */
#if (CONFIG_ENDAT1_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDatFirmwarePru1_0 + 2);
#else
    version = *((uint32_t *)EnDatFirmwarePru0_0 + 2);
#endif
    DebugP_log("\r\nEnDAT firmware for second slice \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
#endif
}

static void endat_print_menu(endat_handle handle)
{
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);

    if((handle == NULL) || (priv == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs\n");
        return;
    }

    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|              Select value for the encoder command from following             |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| 1 : Encoder send position values                                             |\n");
    DebugP_log("\r| 2 : Selection of memory area                                                 |\n");
    DebugP_log("\r| 3 : Encoder receive parameter                                                |\n");
    DebugP_log("\r| 4 : Encoder send parameter                                                   |\n");
    DebugP_log("\r| 5 : Encoder receive reset                                                    |\n");
    DebugP_log("\r| 6 : Encoder send test values                                                 |\n");
    DebugP_log("\r| 7 : Encoder receive test command                                             |\n");
    if(priv->cmd_set_2_2)
    {
        DebugP_log("\r| 8 : Encoder to send position + AI(s)                                         |\n");
        DebugP_log("\r| 9 : Encoder to send position + AI(s) and receive selection of memory area    |\n");
        DebugP_log("\r| 10: Encoder to send position + AI(s) and receive parameter                   |\n");
        DebugP_log("\r| 11: Encoder to send position + AI(s) and send parameter                      |\n");
        DebugP_log("\r| 12: Encoder to send position + AI(s) and receive error reset                 |\n");
        DebugP_log("\r| 13: Encoder to send position + AI(s) and receive test command                |\n");
        DebugP_log("\r| 14: Encoder receive communication command                                    |\n");
    }
    DebugP_log("\r|                                                                              |\n");
    DebugP_log("\r|100: Configure clock                                                          |\n");
    DebugP_log("\r|101: Simulate motor control 2.1 position loop                                 |\n");
    DebugP_log("\r|102: Toggle raw data display                                                  |\n");
    DebugP_log("\r|103: Configure tST delay                                                      |\n");
    DebugP_log("\r|104: Start continuous mode                                                    |\n");
    DebugP_log("\r|105: Configure rx arm counter (account tD)                                    |\n");
    DebugP_log("\r|106: Configure rx clock disable time (for tD)                                 |\n");

    if(priv->cmd_set_2_2)
    {
        DebugP_log("\r|107: Simulate motor control 2.2 position loop %s                        |\n",
            (attrs->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU) ? "        " : "(safety)");
    }

    DebugP_log("\r|108: Configure propagation delay (tD)                                         |\n");
    if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        DebugP_log("\r|109: Configure wire delay                                                 |\n");
    }

    DebugP_log("\r|110: Recovery Time (RT)                                                       |\n");
    DebugP_log("\r|111: Simulate motor control 2.1 position loop for long time                   |\n");
    DebugP_log("\r|112: Start/Stop Recovery Time counter                                         |\n");
    DebugP_log("\r|200: Start periodic continuous mode (CMP trigger)                             |\n");
    DebugP_log("\r|201: Start periodic continuous mode (CAP trigger)                             |\n");

    DebugP_log("\r|------------------------------------------------------------------------------|\n\r|\n");
    DebugP_log("\r| enter value: ");
}

static int32_t endat_get_command(endat_handle handle)
{
    volatile int32_t cmd = SystemP_FAILURE;
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);

    if((handle == NULL) || (priv == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs\n");
        DebugP_log("\r| WARNING: EnDat 2.1 send position values command will be sent\n");
        return SystemP_FAILURE;
    }

    if(DebugP_scanf("%d", &cmd) < 0)
    {
        cmd = SystemP_FAILURE;
    }

    if(VALID_2_1_CMD(cmd) || (priv->cmd_set_2_2 && VALID_2_2_CMD(cmd))
            || VALID_HOST_CMD(cmd) || VALID_CONT_MODE_CMD(cmd))
    {
        if(attrs->mode != ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU && (cmd == 109))
        {
            DebugP_log("\r| WARNING: command 109 is only available in multi-channel single PRU mode, EnDat 2.1 send position values command will be sent\n");
            return 1;
        }

        if(!priv->cmd_set_2_2 && (cmd == 107))
        {
            DebugP_log("\r| WARNING: invalid command, EnDat 2.1 send position values command will be sent\n");
            return 1;
        }
        return cmd;
    }

    DebugP_log("\r| WARNING: invalid command, EnDat 2.1 send position values command will be sent\n");
    return 1;
}

static void endat_recvd_print(endat_handle handle, int32_t cmd, endat_format_data *u, int32_t crc)
{
    endat_priv *priv = endat_get_priv(handle);
    uint32_t current_channel;
    uint32_t addinfo, byte1;
    uint64_t max;
    endat_position_type position;
    uint8_t flag_idx;
    const endat_attrs *attrs = endat_get_attrs(handle);

    if((handle == NULL) || (priv == NULL) || (attrs == NULL) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs or invalid current_channel\n");
        return;
    }

    current_channel = priv->current_channel;
    max = (uint64_t)1 << priv->single_turn_res[current_channel];
    flag_idx = (attrs->load_share_enabled) ? current_channel : 0;

    /* Calculate position value (angle for rotary, length for linear encoders)
     * Note: This would give wrong values if cmd is not position related, but that
     * is okay as then this value won't be used */
    if(priv->type[current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
    {
        position.angle = ((float) u->position_addinfo.position.position) /
                         (float)max * (float)360;
    }
    else
    {
        position.length = u->position_addinfo.position.position * priv->step[current_channel];
    }


    DebugP_log("\r|\n\r|\n");

    switch(cmd)
    {
        case 2:
        case 3:
        case 5:
        case 7:
        case 14:
            DebugP_log("\r| crc: %s\n", crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("| crc: %x\n", u->addr_params.crc);
            break;

        case 4:
            DebugP_log("\r| parameter: 0x%x, crc: %s\n", u->addr_params.params,
                        crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->addr_params.crc);
            break;

        case 6:
            DebugP_log("\r| test value: 0x%02x%08x, crc: %s\n",
                        (uint32_t)((u->test.value & 0xFF00000000) >> 32),
                        (uint32_t)(u->test.value & 0xFFFFFFFF), crc & 0x1 ? "success" : "failure");
            DebugP_logInfo("\r| crc: %x\n", u->test.crc);
            break;

        case 1:
            if(priv->multi_turn_res[current_channel])
            {
                sprintf(gUartBuffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type[current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                {
                    sprintf(gUartBuffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUartBuffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUartBuffer);

            DebugP_log("f1: %x, crc: %s\n", u->position_addinfo.position.f1,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            if(priv->multi_turn_res[current_channel])
            {
                sprintf(gUartBuffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type[current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                {
                    sprintf(gUartBuffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUartBuffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUartBuffer);

            DebugP_log("f1/f2: %x/%x, crc: %s\n", u->position_addinfo.position.f1,
                        u->position_addinfo.position.f2,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);

            if(priv->flags[flag_idx].info1)
            {
                addinfo = u->position_addinfo.addinfo1.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| WRN: %x  RM: %x  Busy: %x  I4-I0: %x\n",
                            (byte1 & ENDAT_STATUS_WARN_MASK) >> ENDAT_STATUS_WARN_SHIFT,
                            (byte1 & ENDAT_STATUS_RM_MASK) >> ENDAT_STATUS_RM_SHIFT,
                            (byte1 & ENDAT_STATUS_BUSY_MASK) >> ENDAT_STATUS_BUSY_SHIFT,
                            byte1 & ENDAT_INFORMATION_MASK);
                DebugP_log("\r|\n| Additional Information 1: 0x%x, crc: %s\n", addinfo,
                            crc & 0x2 ? "success" : "failure");
                DebugP_logInfo("| addinfo1 crc: %x\n",
                                u->position_addinfo.addinfo1.crc);
            }

            if(priv->flags[flag_idx].info2)
            {
                addinfo = u->position_addinfo.addinfo2.addinfo;
                byte1 = (addinfo >> 16) & ((1 << 8) - 1);

                DebugP_log("\r|\n| Additional Information 2: 0x%x, crc: %s\n", addinfo,
                            crc & 0x4 ? "success" : "failure");
                DebugP_logInfo("| addinfo2 crc: %x\n",
                                u->position_addinfo.addinfo2.crc);
            }

            break;

        default:
            DebugP_log("\r|\n| ERROR: print requested for invalid command\n");
            break;
    }

    DebugP_log("\r|\n\r|\n");
}

static void endat_display_raw_data(endat_handle handle, int32_t cmd)
{
    int32_t ch;
    endat_ch_rx_info_array *channel_rx_info;
    endat_priv *priv = endat_get_priv(handle);

    if((handle == NULL) || (priv == NULL) || (handle->priv->channel_rx_info == NULL) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/channel_rx_info or invalid current_channel\n");
        return;
    }

    ch = priv->current_channel;
    channel_rx_info = priv->channel_rx_info;

    switch(cmd)
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
            DebugP_log("\r|\n| raw data: %x %x %x %x\n|\n",
                        channel_rx_info->ch[ch].pos_word0, channel_rx_info->ch[ch].pos_word1,
                        channel_rx_info->ch[ch].pos_word2, channel_rx_info->ch[ch].pos_word3);
            break;

        default:
            DebugP_log("\r|\n| Nothing raw to display - this is not a valid command\n|\n");
            break;
    }
}

/**
 * \brief Get command supplement data from user input
 *
 * Prompts user for additional parameters required by specific EnDAT commands
 * (MRS address, parameters, block address, etc.). This function should be called
 * after endat_get_command() validates the command.
 *
 * \param handle    EnDAT driver handle
 * \param cmd       Command code
 * \param cmd_supplement Pointer to structure to store supplement data
 *
 * \return Command code on success, negative error code on failure
 */
static int32_t endat_get_command_supplement(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    uint32_t i;
    uint8_t loop_count;
    const endat_attrs *attrs = endat_get_attrs(handle);
    uint8_t selected_ch;

    if((handle == NULL) || (attrs == NULL) || (cmd_supplement == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/attrs/cmd_supplement\n");
        return SystemP_FAILURE;
    }

    /* Clear previous command supplement data to avoid stale values */
    memset(cmd_supplement, 0, sizeof(*cmd_supplement));
    loop_count = ENDAT_NUM_CH_PER_SLICE_MAX;

    cmd_supplement->cmd_type = cmd;
    switch(cmd)
    {
        case 2:
            DebugP_log("\n\r| Enter MRS code (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        DebugP_log("\n\r| for channel %d: ", i);
                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid MRS code|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid MRS code|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid MRS code|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid MRS code|\n");
                    return SystemP_FAILURE;
                }
            }

            break;

        case 9:
            DebugP_log("\n\r| Enter MRS code (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        DebugP_log("\n\r| for channel %d: ", i);

                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid MRS code|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid MRS code|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] == ENDAT_SECTION2_MEMORY)
                        {
                            DebugP_log("\n\r| Enter block address (hex value): ");

                            if(DebugP_scanf("%x", &cmd_supplement->block[i]) < 0)
                            {
                                DebugP_log("\r| ERROR: invalid block address|\n");
                                return SystemP_FAILURE;
                            }

                            /* better compare it with number of blocks information available in eeprom */
                            if(cmd_supplement->block[i] > 0xFF)
                            {
                                DebugP_log("\r| ERROR: invalid block address|\n");
                                return SystemP_FAILURE;
                            }

                            cmd_supplement->has_block_address[i] = TRUE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid MRS code|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid MRS code|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] == ENDAT_SECTION2_MEMORY)
                {
                    DebugP_log("\n\r| Enter block address (hex value): ");

                    if(DebugP_scanf("%x", &cmd_supplement->block[0]) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid block address|\n");
                        return SystemP_FAILURE;
                    }

                    /* better compare it with number of blocks information available in eeprom */
                    if(cmd_supplement->block[0] > 0xFF)
                    {
                        DebugP_log("\r| ERROR: invalid block address|\n");
                        return SystemP_FAILURE;
                    }

                    cmd_supplement->has_block_address[0] = TRUE;
                }
            }

            break;

        case 3:
        case 10:
            DebugP_log("\n\r| Enter parameter address (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        DebugP_log("\n\r| for channel %d: ", i);

                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid parameter address|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid parameter address|\n");
                            return SystemP_FAILURE;
                        }

                        DebugP_log("\n\r| Enter parameter (hex value): ");

                        if(DebugP_scanf("%x", &cmd_supplement->data[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid parameter|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->data[i] > 0xFFFF)
                        {
                            DebugP_log("\r| ERROR: invalid parameter|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid parameter address|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid parameter address|\n");
                    return SystemP_FAILURE;
                }

                DebugP_log("\n\r| Enter parameter (hex value): ");

                if(DebugP_scanf("%x", &cmd_supplement->data[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid parameter|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->data[0] > 0xFFFF)
                {
                    DebugP_log("\r| ERROR: invalid parameter|\n");
                    return SystemP_FAILURE;
                }
            }

            break;

        case 4:
        case 11:
            DebugP_log("\n\r| Enter parameter address (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        DebugP_log("\n\r| for channel %d: ", i);

                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid parameter address|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid parameter address|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid parameter address|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid parameter address|\n");
                    return SystemP_FAILURE;
                }
            }

            break;

        case 7:
        case 13:
            DebugP_log("\n\r| Enter port address (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        DebugP_log("\n\r| for channel %d: ", i);

                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid port address|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid port address|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid port address|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid port address|\n");
                    return SystemP_FAILURE;
                }
            }

            break;

        case 14:
            DebugP_log("\n\r| Enter encoder address (hex value): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        if(loop_count > 1)
                        {
                            DebugP_log("\n\r| for channel %d: ", i);
                        }

                        if(DebugP_scanf("%x", &cmd_supplement->address[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid encoder address|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->address[i] > 0xFF)
                        {
                            DebugP_log("\r| ERROR: invalid encoder address|\n");
                            return SystemP_FAILURE;
                        }

                        DebugP_log("\n\r| Enter instruction (hex value): ");

                        if(DebugP_scanf("%x", &cmd_supplement->data[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid instruction|\n");
                            return SystemP_FAILURE;
                        }

                        if(cmd_supplement->data[i] > 0xFFFF)
                        {
                            DebugP_log("\r| ERROR: invalid instruction|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%x", &cmd_supplement->address[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid encoder address|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->address[0] > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid encoder address|\n");
                    return SystemP_FAILURE;
                }

                DebugP_log("\n\r| Enter instruction (hex value): ");

                if(DebugP_scanf("%x", &cmd_supplement->data[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid instruction|\n");
                    return SystemP_FAILURE;
                }

                if(cmd_supplement->data[0] > 0xFFFF)
                {
                    DebugP_log("\r| ERROR: invalid instruction|\n");
                    return SystemP_FAILURE;
                }
            }

            break;

        case 100:
        case 101:
        case 107:
            DebugP_log("\n\r| Enter frequency in Hz: ");

            if(DebugP_scanf("%u", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid frequency|\n");
                return SystemP_FAILURE;
            }

            break;

        case 103:
        case 105:
        case 106:
        case 108:
        case 109:
            /* Channel-specific delay configuration commands
             * 103: tST delay counter value
             * 105: RX arm counter
             * 106: RX clock disable time (tD)
             * 108: Propagation delay
             * 109: Wire delay
             */
            switch(cmd)
            {
                case 103:
                    DebugP_log("\n\r| Enter tST delay counter value in ns: ");
                    break;
                case 105:
                    DebugP_log("\n\r| Enter rx arm counter in ns: ");
                    break;
                case 106:
                    DebugP_log("\n\r| Enter rx clock disable time (for tD) in ns: ");
                    break;
                case 108:
                    DebugP_log("\n\r| Enter propagation delay in ns: ");
                    break;
                case 109:
                    DebugP_log("\n\r| Enter wire delay in ns: ");
                    break;
            }

            /* Get the delay value */
            if(DebugP_scanf("%u", &cmd_supplement->delay) < 0)
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }
            /* For multi-channel mode, select the target channel first */
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                DebugP_log("\n\r| Select Channel: ");
                if(DebugP_scanf("%hhu", &selected_ch) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel|\n");
                    return SystemP_FAILURE;
                }

                if(!((attrs->channel_mask) & (1 << selected_ch)))
                {
                    DebugP_log("\r| ERROR: invalid channel|\n");
                    return SystemP_FAILURE;
                }
                cmd_supplement->selected_channel = selected_ch;
            }

            break;

        case 112:
            DebugP_log("\n\r| Enter 1 to enable recovery time measurement and 0 to disable recovery time measurement: ");

            if(DebugP_scanf("%hhu", &cmd_supplement->enable_rt) < 0)
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }
            if(cmd_supplement->enable_rt > 1)
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }

            /* For multi-channel mode, select the target channel first */
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {

                DebugP_log("\n\r| Select Channel: ");
                if(DebugP_scanf("%hhu", &selected_ch) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel|\n");
                    return SystemP_FAILURE;
                }

                if(!((attrs->channel_mask) & (1 << selected_ch)))
                {
                    DebugP_log("\r| ERROR: invalid channel|\n");
                    return SystemP_FAILURE;
                }

                /* Store selected channel in cmd_supplement for use in host command processing */
                cmd_supplement->selected_channel = selected_ch;
            }
            break;

        case 200:
            DebugP_log("\n\r| Enter IEP reset cycle count (must be greater than EnDat cycle time including timeout period, in IEP cycles): ");
            if(DebugP_scanf("%llu", &cmd_supplement->iep_reset_count) < 0)
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }
            if((cmd_supplement->iep_reset_count == 0) || (cmd_supplement->iep_reset_count <= ENDAT_IEP_COUNTER_INCREMENT))
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }

            DebugP_log("\n\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
            if(attrs->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                for(i = 0; i < loop_count; i++)
                {
                    if(attrs->channel_mask & (1 << i))
                    {
                        if(loop_count > 1)
                        {
                            DebugP_log("\n\r| for channel %d: ", i);
                        }

                        if(DebugP_scanf("%llu", &cmd_supplement->ch_trigger_count[i]) < 0)
                        {
                            DebugP_log("\r| ERROR: invalid value|\n");
                            return SystemP_FAILURE;
                        }
                    }
                }
            }
            else
            {
                if(DebugP_scanf("%llu", &cmd_supplement->ch_trigger_count[0]) < 0)
                {
                    DebugP_log("\r| ERROR: invalid value|\n");
                    return SystemP_FAILURE;
                }
            }
            break;

        case 201:
#if defined(SOC_AM243X)
            DebugP_log("\n\r| Enter IEP SYNC OUT0 cycle period (in IEP cycles): ");
            if(DebugP_scanf("%llu", &cmd_supplement->iep_sync0_period) < 0)
            {
                DebugP_log("\r| ERROR: invalid value|\n");
                return SystemP_FAILURE;
            }
            if((cmd_supplement->iep_sync0_period == 0) || (cmd_supplement->iep_sync0_period <= ENDAT_IEP_COUNTER_INCREMENT) || (cmd_supplement->iep_sync0_period > UINT32_MAX))
            {
                DebugP_log("\r| ERROR: invalid value. 0 is not allowed and maximum value allowed is %u|\n", UINT32_MAX);
                return SystemP_FAILURE;
            }
#else
            DebugP_log("\r| Periodic cap mode cycle time will be equal to EPWM SYNC OUT frequency. \r\n NOTE: In SysConfig, EPWM and EPWM to IEP LATCH XBAR configuration must be done|\n");
#endif
            break;

        default:
            DebugP_log("\r| ERROR: no command data required for the command\n");
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

static int32_t endat_handle_user(endat_handle handle[CONFIG_ENDAT_NUM_INSTANCES], endat_cmd_supplement cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES])
{
    int32_t cmd;
    uint32_t i = 0;
    int32_t ret = SystemP_SUCCESS;

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
        DebugP_log("\r|------------------ EnDat Module %d Command Menu ------------------|\n", i);
#else
        DebugP_log("\r|------------------ EnDat Command Menu ------------------|\n");
#endif
        endat_print_menu(handle[i]);

#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
        DebugP_log("\r| Enter command for EnDat Module %d: ", i);
#else
        DebugP_log("\r| Enter command: ");
#endif
        cmd = endat_get_command(handle[i]);

        if(cmd == SystemP_FAILURE)
        {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
            DebugP_log("\r| ERROR: endat_get_command() failed for instance %d\n", i);
#else
            DebugP_log("\r| ERROR: endat_get_command() failed\n");
#endif
            ret = SystemP_FAILURE;
        }

        cmd_supplement[i].cmd_type = cmd;

        if(HAVE_COMMAND_SUPPLEMENT(cmd))
        {
            cmd = endat_get_command_supplement(handle[i], cmd, &cmd_supplement[i]);
            if(cmd == SystemP_FAILURE)
            {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                DebugP_log("\r| ERROR: Failed to get command supplement data for instance %d\n", i);
#else
                DebugP_log("\r| ERROR: Failed to get command supplement data\n");
#endif
                ret = SystemP_FAILURE;
                continue;
            }
        }
    }

    return ret;
}


static uint32_t endat_do_sanity_tst_delay(uint32_t delay)
{
    /* 0xFFFF is also a multiple of 5 */
    if(delay > 0xFFFFU)
    {
        DebugP_log("\r| ERROR: delay greater than %uns, enter lesser value\n|\n|\n",
                    0xFFFFU);
        return delay;
    }

    if(delay % 5)
    {
        delay += 5, delay /= 5, delay *= 5;
        DebugP_log("\r| WARNING: delay not multiple of 5ns, rounding to %uns\n|\n|\n",
                    delay);
    }

    return delay;
}

/**
 * \brief   Calculate position loop period in microseconds
 *
 * \param   freq    Position loop frequency in Hz
 *
 * \retval  >0      Period in microseconds
 * \retval  -1      Error (invalid frequency)
 *
 * \note    Returns negative value on error for backward compatibility with usage pattern
 */
static int32_t endat_calc_position_period(uint32_t freq)
{
    uint32_t i;
    uint32_t is_multi_ch = 0;
    const endat_attrs *attrs[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        attrs[i] = endat_get_attrs(gAppEndatHandle[i]);

        if((gAppEndatHandle[i] == NULL) || (attrs[i] == NULL))
        {
            DebugP_log("\r\n\nERROR: NULL handle/attrs for EnDAT instance %u, exiting endat_calc_position_period()\n", i);
            return -1;
        }
    }

    /* Check if any instance uses multi-channel mode */
    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            is_multi_ch = 1;
            break;
        }
    }

    /* Validate frequency limits - 16KHz max due to timer limitations */
    if(freq > 16000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 16KHz\n|\n|\n");
        return -1;
    }
    else if(is_multi_ch && freq > 8000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 8KHz in multi channel configuration\n|\n|\n");
        return -1;
    }

    /* Calculate period in microseconds */
    return 1000000 / freq;
}

static void endat_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gEndatPositionLoopStatus = ENDAT_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static void endat_process_2_1_position_command(endat_handle handle)
{
    uint32_t crc;
    const endat_attrs *attrs = endat_get_attrs(handle);
    endat_priv *priv = endat_get_priv(handle);
    uint32_t i;
    uint32_t instance;
    int32_t status;

    if((handle == NULL) || (priv == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs\n");
        return;
    }

    instance = attrs->instance;

    status = endat_command_process(handle, 1, NULL);
    if(status != SystemP_SUCCESS)
    {
        if(status == SystemP_TIMEOUT)
        {
            DebugP_log("\r| ERROR: endat_command_process timed out\n");
        }
        else
        {
            DebugP_log("\r| ERROR: endat_command_process failed with status %d\n", status);
        }
        return;
    }

    if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    {
        for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
        {
            if(attrs->channel_mask & (1 << i))
            {
                status = endat_multi_channel_set_cur(handle, i);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                    return;
                }
                status = endat_recvd_process(handle, 1, &gAppEndatFormatDataMtrCtrl[instance][i]);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
                    return;
                }
                crc = endat_recvd_validate(handle, 1, &gAppEndatFormatDataMtrCtrl[instance][i]);

                if(!(crc & 0x1))
                {
                    gAppEndatMtrCtrlCrcErr[instance][i]++;
                }
            }
        }

    }
    else
    {
        if(priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX)
        {
            DebugP_log("\r| ERROR: Invalid current_channel\n");
            return;
        }

        status = endat_recvd_process(handle, 1, &gAppEndatFormatDataMtrCtrl[instance][priv->current_channel]);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
            return;
        }
        crc = endat_recvd_validate(handle, 1, &gAppEndatFormatDataMtrCtrl[instance][priv->current_channel]);
        if(!(crc & 0x1))
        {
            gAppEndatMtrCtrlCrcErr[instance][priv->current_channel]++;
        }
    }
}

static uint16_t endat_handle_2_2_position_command(endat_handle handle, uint32_t cmd, endat_cmd_supplement *cmd_supplement, uint32_t a0)
{
    uint32_t crc;
    uint32_t instance;
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);
    uint8_t flag_idx;
    int32_t status;

    if((handle == NULL) || (priv == NULL) || (attrs == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs\n");
        return 0;
    }

    instance = attrs->instance;
    flag_idx = (attrs->load_share_enabled) ? a0 : 0;

    status = endat_recvd_process(handle, cmd, &gAppEndatFormatDataMtrCtrl[instance][a0]);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
        return 0;
    }
    crc = endat_recvd_validate(handle, cmd, &gAppEndatFormatDataMtrCtrl[instance][a0]);

    if(!(crc & 0x1))
    {
        gAppEndat22CrcPositionErrCnt[instance][a0]++;
    }

    if(priv->flags[flag_idx].info1 && !(crc & 0x2))
    {
        gAppEndat22CrcAddinfo1ErrCnt[instance][a0]++;
    }

    status = endat_addinfo_track(handle, cmd, cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: endat_addinfo_track failed with status %d\n", status);
    }

    return gAppEndatFormatDataMtrCtrl[instance][a0].position_addinfo.addinfo1.addinfo & 0xFFFF;
}

static void endat_process_2_2_position_command(endat_handle handle)
{
    uint32_t cmd, i;
    endat_cmd_supplement cmd_supplement;
    uint16_t pos_word;
    uint32_t instance;
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);
    int32_t status;

    if((handle == NULL) || (priv == NULL) || (attrs == NULL) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs or invalid current_channel\n");
        return;
    }

    instance = attrs->instance;

    if(((attrs->mode != ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU) && priv->has_safety[priv->current_channel]))
    {
        cmd = 9;
        if(attrs->load_share_enabled)
        {
            /* Load-share mode: populate addresses for enabled channels */
            if(attrs->channel0_enabled) cmd_supplement.address[0] = gAppEndat22LoopMrs[instance];
            if(attrs->channel1_enabled) cmd_supplement.address[1] = gAppEndat22LoopMrs[instance];
            if(attrs->channel2_enabled) cmd_supplement.address[2] = gAppEndat22LoopMrs[instance];
        }
        else
        {
            /* Single/multi-channel single-PRU mode: populate only index 0 */
            cmd_supplement.address[0] = gAppEndat22LoopMrs[instance];
        }
    }
    else
    {
        cmd = 8;
    }

    status = endat_command_process(handle, cmd, &cmd_supplement);
    if(status != SystemP_SUCCESS)
    {
        if(status == SystemP_TIMEOUT)
        {
            DebugP_log("\r| ERROR: endat_command_process timed out\n");
        }
        else
        {
            DebugP_log("\r| ERROR: endat_command_process failed with status %d\n", status);
        }
        return;
    }

    if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    {
        for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
        {
            if(attrs->channel_mask & (1 << i))
            {
                status = endat_multi_channel_set_cur(handle, i);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: endat_multi_channel_set_cur failed with status %d\n", status);
                    return;
                }
                pos_word = endat_handle_2_2_position_command(handle, cmd, &cmd_supplement, i);

                if (!priv->has_safety[i] || attrs->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
                {
                    continue;
                }

                /* WORD3 in addinfo1 */
                if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD1)
                {
                    gAppEndat22PosVal2[instance][i] &= 0xFFFF0000FFFFFFFF;
                    gAppEndat22PosVal2[instance][i] |= (unsigned long long)pos_word << 32;
                    gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD2;
                    /* WORD1 in addinfo1 */
                }
                else if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD2)
                {
                    gAppEndat22PosVal2[instance][i] &= 0xFFFFFFFFFFFF0000;
                    gAppEndat22PosVal2[instance][i] |= (uint64_t)pos_word;
                    gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD3;
                    /* WORD2 in addinfo1 */
                }
                else if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD3)
                {
                    gAppEndat22PosVal2[instance][i] &= 0xFFFFFFFF0000FFFF;
                    gAppEndat22PosVal2[instance][i] |= (unsigned long long)pos_word << 16;
                    gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD1;
                }
            }
        }
    }
    else
    {
        if(priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX)
        {
            DebugP_log("\r| ERROR: Invalid current_channel\n");
            return;
        }

        pos_word = endat_handle_2_2_position_command(handle, cmd, &cmd_supplement, priv->current_channel);
        if(!priv->has_safety[priv->current_channel])
        {
            return;
        }

        /* WORD3 in addinfo1 */
        if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD1)
        {
            gAppEndat22PosVal2[instance][priv->current_channel] &= 0xFFFF0000FFFFFFFF;
            gAppEndat22PosVal2[instance][priv->current_channel] |= (unsigned long long)pos_word << 32;
            gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD2;
            /* WORD1 in addinfo1 */
        }
        else if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD2)
        {
            gAppEndat22PosVal2[instance][priv->current_channel] &= 0xFFFFFFFFFFFF0000;
            gAppEndat22PosVal2[instance][priv->current_channel] |= (uint64_t)pos_word;
            gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD3;
            /* WORD2 in addinfo1 */
        }
        else if(gAppEndat22LoopMrs[instance] == MRS_POS_VAL2_WORD3)
        {
            gAppEndat22PosVal2[instance][priv->current_channel] &= 0xFFFFFFFF0000FFFF;
            gAppEndat22PosVal2[instance][priv->current_channel] |= (unsigned long long)pos_word << 16;
            gAppEndat22LoopMrs[instance] = MRS_POS_VAL2_WORD1;
        }
    }
}

/**
 * \brief   Position loop IRQ handler for continuous position reading (commands 101 and 107)
 *
 * \details This function is called by the timer interrupt handler during continuous
 *          position reading mode. It increments gPositionLoopIrqCount to track IRQ events.
 *          After every IRQ hit and increment of gPositionLoopIrqCount, the corresponding
 *          EnDat command (101 for 2.1 or 107 for 2.2) is executed to read position data
 *          from the encoder.
 *
 *          Used by:
 *          - Command 101: EnDat 2.1 continuous position reading
 *          - Command 107: EnDat 2.2 continuous position reading
 */
void endat_position_loop(void)
{
    gPositionLoopIrqCount++;
}

/* Function to print the appropriate header based on channel configurations */
static void endat_print_position_header(endat_handle handle, uint32_t continuous, uint32_t is_2_2)

{
    const endat_attrs *attrs = endat_get_attrs(handle);
    endat_priv *priv = endat_get_priv(handle);

    if((handle == NULL) || (priv == NULL) || (attrs == NULL) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs or invalid current_channel\n");
        return;
    }

    if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    {
        /*For multi-channel, we print a general header that accommodates all channels*/
        /*For multi-channel, we print a general header that accommodates all channels*/
        if(continuous)
        {
            DebugP_log("         position,    [revolution],  f1");
            if(is_2_2)
            {
                DebugP_log(", f2");
            }
            DebugP_log("\r\n| ");
        }
        else
        {
            DebugP_log("         position,    [revolution], crc errors, f1");
            if(is_2_2)
            {
                DebugP_log(", f2");
            }
            DebugP_log("\r\n| ");
        }
    }
    else
    {
        /* For single channel, we check the specific channel configuration */
        if(priv->multi_turn_res[priv->current_channel])
        {
            if(continuous)
            {
                DebugP_log("         position,       revolution, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position,       revolution, crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            if(continuous)
            {
                DebugP_log("         position, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
            else
            {
                DebugP_log("         position, crc errors, f1");
                if(is_2_2)
                {
                    DebugP_log(", f2");
                }
                DebugP_log("\r\n| ");
            }
        }
    }
}

static int32_t endat_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "endat_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)endat_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r endat_position_loop_decide_termination task creation failed\n");
    }

    return status ;
}

static void endat_loop_timer_create(int32_t us)
{
    TimerP_Params timerParams;

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_TIMER0_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER0_INPUT_CLK_HZ;
    timerParams.periodInUsec   = us;
    timerParams.oneshotMode    = 0;
    timerParams.enableOverflowInt = 1;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

    return ;
}

static uint32_t endat_get_position_loop_chars(uint32_t multi_turn_res, uint32_t continuous, uint32_t is_2_2)
{
    uint32_t i = 0;

    /* Base character count depends on multi-turn resolution */
    if(multi_turn_res)
    {
        i = 34;
    }
    else
    {
        i = 16;
    }

    /* Add space for CRC errors in non-continuous mode */
    if(!continuous)
    {
        i += 12;
    }

    /* Add space for F1 field */
    i += 4;

    /* Add space for F2 field in 2.2 mode */
    if(!continuous && is_2_2)
    {
        i += 4;
    }

    return i;
}

/* Modified version of endat_print_position_loop to handle channel-specific data sources */
static void endat_print_position_loop(endat_handle handle, uint32_t continuous, uint32_t is_2_2, uint32_t ch)
{
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);
    uint32_t instance;
    uint32_t current_channel;
    uint64_t max;
    endat_position_type position;

    if((handle == NULL) || (priv == NULL) || (attrs == NULL) || (ch >= ENDAT_NUM_CH_PER_SLICE_MAX) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs or ch >= ENDAT_NUM_CH_PER_SLICE_MAX or current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX\n");
        return;
    }

    instance = attrs->instance;
    current_channel = priv->current_channel;
    max = (uint64_t)1 << priv->single_turn_res[current_channel];

    if(priv->type[current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
    {
        position.angle = ((float)
                          gAppEndatFormatDataMtrCtrl[instance][ch].position_addinfo.position.position) /
                         (float)max * (float)360;
    }
    else
    {
        position.length =
            gAppEndatFormatDataMtrCtrl[instance][ch].position_addinfo.position.position * priv->step[current_channel];
    }

    /* max value is 2x48, has 15 digits, so 16 is safe */
    if(priv->multi_turn_res[current_channel])
    {
        sprintf(gUartBuffer, "%16.12f, %16s", position.angle,
                uint64_to_str(gAppEndatFormatDataMtrCtrl[instance][ch].position_addinfo.position.revolution));
    }
    else
    {
        if(priv->type[current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
        {
            sprintf(gUartBuffer, "%16.12f", position.angle);
        }
        else
        {
            sprintf(gUartBuffer, "%16s", uint64_to_str(position.length));
        }
    }

    DebugP_log("\r%s", gUartBuffer);

    if(!continuous)
    {
        if(is_2_2)
        {
            DebugP_log(", %10u", gAppEndat22CrcPositionErrCnt[instance][ch]);
        }
        else
        {
            DebugP_log(", %10u", gAppEndatMtrCtrlCrcErr[instance][ch]);
        }
    }

    DebugP_log(",%3u",
                gAppEndatFormatDataMtrCtrl[instance][ch].position_addinfo.position.f1);

    if(!continuous && is_2_2)
    {
        DebugP_log(",%3u",
                    gAppEndatFormatDataMtrCtrl[instance][ch].position_addinfo.position.f2);
    }
}

static void endat_process_host_command(endat_handle handle, int32_t cmd, endat_cmd_supplement *cmd_supplement)
{
    const endat_attrs *attrs = endat_get_attrs(handle);
    endat_priv *priv = endat_get_priv(handle);
    int32_t ret;
    int32_t status;
    int32_t i;
    uint32_t val;
    float ct;

    if((handle == NULL) ||
       (priv == NULL) ||
       (handle->priv->channel_rx_info == NULL) ||
       (attrs == NULL) ||
       (cmd_supplement == NULL) ||
       (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/channel_rx_info/attrs/cmd_supplement or invalid current_channel\n");
        return;
    }

    /* clock configuration */
    if(cmd == 100)
    {
        if(endat_config_clock(handle, cmd_supplement->frequency) != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: clock configuration failed\n|\n|\n");
            return;
        }

        /* Compensate the wire delay for multi channel single PRU mode */
        if((attrs->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU))
        {
            for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1 << i))
                {
                    status = endat_multi_channel_set_cur(handle, i);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                        return;
                    }

                    /* Calculate wire delay compensation based on propagation delay difference */
                    val = gAppEndatPropDelayMax[attrs->instance] - gAppEndatPropDelay[attrs->instance][i];

                    /* Convert ns to delay counter value - multiplication first for better precision */
                    val = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)((((float)val * attrs->core_clk_freq) / 1000000000)));
                    status = endat_config_wire_delay(handle, val);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: endat_config_wire_delay failed with status %d\n", status);
                    }
                }
            }
        }
        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->delay = 2000;
        }
        else
        {
            cmd_supplement->delay = 0;
        }
        /* control loop */
        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1 << i))
                {
                    cmd_supplement->selected_channel = i;
                    endat_process_host_command(handle, 103, cmd_supplement);
                }
           }
        }
        else
        {
            endat_process_host_command(handle, 103, cmd_supplement);
        }
    }
    else if(cmd == 102)
    {
       priv->raw_data ^= 1;
    }
    else if(cmd == 103)
    {
        /* convert tst delay from ns to tst counts*/
        cmd_supplement->delay = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->delay * attrs->core_clk_freq)/1000000000));

        val = endat_do_sanity_tst_delay(cmd_supplement->delay);

        if(val <= 0xFFFFU)
        {
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                    return;
                }
            }
            status = endat_config_tst_delay(handle, (uint16_t) val);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_config_tst_delay failed with status %d\n", status);
            }
        }
    }
    else if(cmd == 105)
    {
        /* convert rx arm delay from ns to rx arm count*/
        cmd_supplement->delay = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->delay * attrs->core_clk_freq)/1000000000));

        /* reuse tST delay sanity check */
        val = endat_do_sanity_tst_delay(cmd_supplement->delay);

        if(val <= 0xFFFFU)
        {
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                    return;
                }
            }
            status = endat_config_rx_arm_cnt(handle, (uint16_t)val);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_config_rx_arm_cnt failed with status %d\n", status);
            }
        }
    }
    else if(cmd == 106)
    {
        /* endat cycle period in ns */
        ct = (float)1000000000 / priv->endat_freq;
        val = floor(cmd_supplement->delay / ct);

        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                return;
            }
        }

        status = endat_config_rx_clock_disable(handle, val);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: endat_config_rx_clock_disable failed with status %d\n", status);
        }
    }
    else if(cmd == 108)
    {
        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                return;
            }
        }
        ret = endat_config_propagation_delay(handle, cmd_supplement->delay);
        if(ret != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: Failed to configure propagation delay\n");
        }
    }
    else if(cmd == 109)
    {
        /* convert from ns to wire delay count*/
        cmd_supplement->delay = ENDAT_DELAY_COUNTER_INCREMENT*((uint16_t)(((float)cmd_supplement->delay * attrs->core_clk_freq)/1000000000));
        /* reuse tST delay sanity check */
        val = endat_do_sanity_tst_delay(cmd_supplement->delay);

        if(val <= 0xFFFFU)
        {
            if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                    return;
                }
            }
            status = endat_config_wire_delay(handle, val);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_config_wire_delay failed with status %d\n", status);
            }
        }
    }
    else if(cmd == 110)
    {
        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            for(i = 0; i < ENDAT_NUM_CH_PER_SLICE_MAX; i++)
            {
                if(attrs->channel_mask & (1 << i))
                {
                    status = endat_multi_channel_set_cur(handle, i);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                        continue;
                    }
                    DebugP_log("\n Channel: %d \n", i);
                    DebugP_log("\t");

                    status = endat_get_recovery_time(handle, &val);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: endat_get_recovery_time failed with status %d\n", status);
                        continue;
                    }
                    DebugP_log("\r Recovery Time: %10u ns \n", val);
                    if(priv->current_channel < ENDAT_NUM_CH_PER_SLICE_MAX)
                    {
                        DebugP_log("\r Current value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.current_counter_value);
                        DebugP_log("\r Previous value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.last_counter_value);
                        DebugP_log("\r Starting value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.starting_value);
                    }
                }
            }
        }
        else
        {
            status = endat_get_recovery_time(handle, &val);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_get_recovery_time failed with status %d\n", status);
                return;
            }
            DebugP_log("\r Recovery Time: %10u ns \n", val);
            DebugP_log("\r Current value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.current_counter_value);
            DebugP_log("\r Previous value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.last_counter_value);
            DebugP_log("\r Starting value of RT counter: %10u \n", priv->channel_rx_info->ch[priv->current_channel].recovery_time_parms.starting_value);
        }
    }
    else if(cmd == 112)
    {
        if(attrs->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            status = endat_multi_channel_set_cur(handle, cmd_supplement->selected_channel);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                return;
            }
        }
        if(cmd_supplement->enable_rt == 1)
        {
            status = endat_enable_rt_measurement(handle);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_enable_rt_measurement failed with status %d\n", status);
            }
        }
        else
        {
            status = endat_disable_rt_measurement(handle);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_disable_rt_measurement failed with status %d\n", status);
            }
        }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
}

/* NOTE: Validation error for any module instance will lead to failure of this function */
static int32_t endat_process_continuous_mode_command(endat_handle handle[CONFIG_ENDAT_NUM_INSTANCES], int32_t cmd, endat_cmd_supplement cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES])
{
    static int32_t timer_init;
    int32_t periodic_cmd[CONFIG_ENDAT_NUM_INSTANCES], status;
    uint32_t char_count;
    endat_cmd_supplement periodic_cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES];
    uint32_t i = 0, j = 0;
    const endat_attrs *attrs[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};
    endat_priv *priv[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};
    int8_t is_cap_mode;
    uint64_t irq_count;
    uint64_t multi_turn, single_turn, position_read, max;
    endat_position_type position;
    int32_t us, ret;
    uint32_t prev_irq_cnt[CONFIG_ENDAT_NUM_INSTANCES] = {0};
    uint32_t curr_irq_cnt;
    uint32_t irq_ch_idx[CONFIG_ENDAT_NUM_INSTANCES] = {0};

    if(cmd == 200)
    {
        if(cmd_supplement[CONFIG_ENDAT0].iep_reset_count == 0)
        {
            DebugP_log("\r\n\n| ERROR: Invalid iep_reset_count value\n");
            return SystemP_FAILURE;
        }
    }
#if defined(SOC_AM243X)
    else if(cmd == 201)
    {
        if(cmd_supplement[CONFIG_ENDAT0].iep_sync0_period == 0)
        {
            DebugP_log("\r\n\n| ERROR: Invalid iep_sync0_period value\n");
            return SystemP_FAILURE;
        }
    }
#endif

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        attrs[i] = endat_get_attrs(handle[i]);
        priv[i] = endat_get_priv(handle[i]);

        if((handle[i] == NULL) || (priv[i] == NULL) || (attrs[i] == NULL))
        {
            DebugP_log("\r\n\nERROR: NULL handle/priv/attrs for EnDAT instance %u\n", i);
            return SystemP_FAILURE;
        }

        if(cmd == 200)
        {
            if(attrs[i]->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
            {
                if(((attrs[i]->channel0_enabled) && (cmd_supplement[i].ch_trigger_count[0] > cmd_supplement[CONFIG_ENDAT0].iep_reset_count)) ||
                   ((attrs[i]->channel1_enabled) && (cmd_supplement[i].ch_trigger_count[1] > cmd_supplement[CONFIG_ENDAT0].iep_reset_count)) ||
                   ((attrs[i]->channel2_enabled) && (cmd_supplement[i].ch_trigger_count[2] > cmd_supplement[CONFIG_ENDAT0].iep_reset_count)))
                {
                    DebugP_log("\r\n\nERROR: Channel trigger count exceeds IEP reset count for EnDAT instance %u\n", i);
                    return SystemP_FAILURE;
                }
            }
            else
            {
                if(cmd_supplement[i].ch_trigger_count[0] > cmd_supplement[CONFIG_ENDAT0].iep_reset_count)
                {
                    DebugP_log("\r\n\nERROR: Channel trigger count exceeds IEP reset count for EnDAT instance %u\n", i);
                    return SystemP_FAILURE;
                }
            }
        }
    }

    if((cmd != 101) && (cmd != 104) && (cmd != 107) && (cmd != 111) && (cmd != 200) && (cmd != 201))
    {
        DebugP_log("\r\n\nERROR: Invalid cmd in endat_process_continuous_mode_command()\n");
        return SystemP_FAILURE;
    }

    /* Get periodic command FIRST, before task creation, to avoid UART contention */
    if(cmd == 200 || cmd == 201)
    {
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
            DebugP_log("\r| Enter position command for periodic mode EnDat Module %d: ", i);
#else
            DebugP_log("\r| Enter position command for periodic mode: ");
#endif
            /* Get periodic command for continuous mode */
            periodic_cmd[i] = endat_get_command(handle[i]);

            if(periodic_cmd[i] == SystemP_FAILURE)
            {
                DebugP_log("\r| ERROR: endat_get_command() failed for EnDAT instance %u\n", i);
                return SystemP_FAILURE;
            }

            if(ENDAT_POSITION_CMD(periodic_cmd[i]))
            {
                if(HAVE_COMMAND_SUPPLEMENT(periodic_cmd[i]))
                {
                    status = endat_get_command_supplement(handle[i], periodic_cmd[i], &periodic_cmd_supplement[i]);
                    if(status == SystemP_FAILURE)
                    {
                        DebugP_log("\r| ERROR: Failed to get command supplement for periodic command for EnDAT instance %u\n", i);
                        return SystemP_FAILURE;
                    }
                }
            }
            else
            {
                DebugP_log("\r| ERROR: Invalid command for periodic mode for EnDAT instance %u\r\n|\r\n|\n", i);
                return SystemP_FAILURE;
            }
        }
    }

    /* Common task creation for all continuous mode commands */
    if(endat_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return SystemP_FAILURE;
    }

    /* Set position loop status for all commands */
    gEndatPositionLoopStatus = ENDAT_POSITION_LOOP_START;

    if(cmd == 200 || cmd == 201)
    {
        is_cap_mode = (cmd == 201) ? 1 : 0;
        memset(&gEndatPeriodicInterface, 0, sizeof(endat_periodic_interface));
        gEndatPeriodicInterface.is_cap_mode = is_cap_mode;
        /*ASSUMPTION: IEP reset count is same for all instances*/
        if(is_cap_mode)
        {
            gEndatPeriodicInterface.iep_reset_count = cmd_supplement[CONFIG_ENDAT0].iep_sync0_period;
        }
        else
        {
            gEndatPeriodicInterface.iep_reset_count = cmd_supplement[CONFIG_ENDAT0].iep_reset_count;
        }

        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            /*
            * Send command once in host trigger mode, to ensure that command data is
            * populated in PRU shared memory as required.
            * ASSUMPTION: Host trigger mode is active when this function is called.
            */
            ret = endat_command_process(handle[i], periodic_cmd[i], &periodic_cmd_supplement[i]);
            if(ret != SystemP_SUCCESS)
            {
                if(ret == SystemP_TIMEOUT)
                {
                    DebugP_log("\r| ERROR: Command processing timed out, endat_command_process failed for EnDAT instance %u\r\n|\r\n|\n", i);
                }
                else
                {
                    DebugP_log("\r| ERROR: Failed to process command, endat_command_process failed for EnDAT instance %u\r\n|\r\n|\n", i);
                }
                return SystemP_FAILURE;
            }

            gEndatPeriodicInterface.handle[i] = handle[i];
            if(is_cap_mode)
            {
                status = endat_config_periodic_trigger_cap_mode(handle[i]);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: CAP trigger config failed for EnDAT instance %u: %d\n", i, status);
                    return SystemP_FAILURE;
                }
            }
            else
            {
                status = endat_config_periodic_trigger_cmp_mode(handle[i]);
                if(status != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: CMP trigger config failed for EnDAT instance %u: %d\n", i, status);
                    return SystemP_FAILURE;
                }
                if(attrs[i]->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
                {
                    if(attrs[i]->channel0_enabled)
                    {
                        gEndatPeriodicInterface.periodic_trigger_count[i][0] = cmd_supplement[i].ch_trigger_count[0];
                    }
                    if(attrs[i]->channel1_enabled)
                    {
                        gEndatPeriodicInterface.periodic_trigger_count[i][1] = cmd_supplement[i].ch_trigger_count[1];
                    }
                    if(attrs[i]->channel2_enabled)
                    {
                        gEndatPeriodicInterface.periodic_trigger_count[i][2] = cmd_supplement[i].ch_trigger_count[2];
                    }
                }
                else
                {
                    gEndatPeriodicInterface.periodic_trigger_count[i][0] = cmd_supplement[i].ch_trigger_count[0];
                }
            }
        }

        status = endat_config_periodic_mode(&gEndatPeriodicInterface);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("\r| ERROR: Failed to configure periodic %s mode|\n", is_cap_mode ? "CAP" : "CMP");
            return SystemP_FAILURE;
        }

        /* Determine IRQ channel index for each instance (based on load share mode) */
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            if(attrs[i]->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
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
            prev_irq_cnt[i] = gPruEndatIrqCnt[i][irq_ch_idx[i]];
        }

        DebugP_log("\r|\n\r| Press enter to stop the continuous mode\r\n|\r\n");

        while(1)
        {
            if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
            {
                if(endat_stop_periodic_mode(&gEndatPeriodicInterface) != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
                return SystemP_SUCCESS;
            }
            else
            {
                /* Wait for IRQ count to increment for at least one instance before reading position */
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    /* Wait for IRQ count to increment */
                    while(1)
                    {
                        curr_irq_cnt = gPruEndatIrqCnt[i][irq_ch_idx[i]];
                        if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
                        {
                            break;
                        }
                        /* Break as soon as IRQ count increments to avoid missing IRQs at high rates */
                        if(curr_irq_cnt != prev_irq_cnt[i])
                        {
                            break;
                        }
                        
                        ClockP_usleep(ENDAT_PERIODIC_MODE_POLL_SLEEP_US);
                    }

                    /* Check stop condition before updating prev_irq_cnt */
                    if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
                    {
                        break;
                    }

                    prev_irq_cnt[i] = curr_irq_cnt;
                }

                /* If stop was requested during IRQ wait, continue for proper cleanup */
                if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
                {
                    continue;
                }

                char_count = 0;

                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {

#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                    DebugP_log("\r| --- EnDat Module %d --- |\r\n", i);
#endif
                    endat_print_position_header(handle[i], 0, VALID_2_2_CMD(periodic_cmd[i]));

                    if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
                    {
                        for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                        {
                            if(attrs[i]->channel_mask & (1 << j))
                            {
                                status = endat_multi_channel_set_cur(handle[i], j);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                                    continue;
                                }
                                status = endat_recvd_process(handle[i], periodic_cmd[i], &gAppEndatFormatDataMtrCtrl[i][j]);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
                                    continue;
                                }
                                char_count += endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, VALID_2_2_CMD(periodic_cmd[i]));
                                DebugP_log("\r\n| Ch-%d: \n", j);
                                endat_print_position_loop(handle[i], 0, VALID_2_2_CMD(periodic_cmd[i]), j);
                                DebugP_log("\n| ");
                                char_count += 3;
                            }
                        }
                    }
                    else
                    {
                        status = endat_recvd_process(handle[i], periodic_cmd[i], &gAppEndatFormatDataMtrCtrl[i][0]);
                        if(status != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
                            continue;
                        }
                        char_count = endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, VALID_2_2_CMD(periodic_cmd[i]));
                        endat_print_position_loop(handle[i], 0, VALID_2_2_CMD(periodic_cmd[i]), 0);
                        DebugP_log("\n| ");
                    }
                }
                while(char_count--)
                {
                    DebugP_log("%c", 8);
                }
            }
        }
    }
    /* Command 101: Simulate motor control 2.1 position loop */
    else if(cmd == 101)
    {
        us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return SystemP_FAILURE;
        }

        gPositionLoopIrqCount = 0;
        irq_count = 0;

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

        DebugP_log("\r|\r\n| Press enter to stop the position display|\n");

        while(1)
        {
            if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
                return SystemP_SUCCESS;
            }
            else
            {
                char_count = 0;
                irq_count = gPositionLoopIrqCount;
                /* Wait till timer IRQ hit */
                while(irq_count == gPositionLoopIrqCount)
                {
                    ;
                }
                /* Process all instances */
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {

                    endat_process_2_1_position_command(handle[i]);

#if(CONFIG_ENDAT_NUM_INSTANCES > 1)
                        DebugP_log("\r| --- EnDat Module %d ---|\r\n", i);
#endif

                    endat_print_position_header(handle[i], 0, 0);

                    if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
                    {
                        for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                        {
                            if(attrs[i]->channel_mask & (1 << j))
                            {
                                status = endat_multi_channel_set_cur(handle[i], j);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Ch set failed: %d\n", status);
                                    continue;
                                }
                                char_count += endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, 0);
                                DebugP_log("\r\n| Ch-%d: \n", j);
                                endat_print_position_loop(handle[i], 0, 0, j);
                                DebugP_log("\n| ");
                                char_count += 3;
                            }
                        }
                    }
                    else
                    {
                        char_count = endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, 0);
                        endat_print_position_loop(handle[i], 0, 0, priv[i]->current_channel);
                        DebugP_log("\n| ");
                    }
                }

                while(char_count--)
                {
                    DebugP_log("%c", 8);
                }
            }
        }
    }
    /* Command 104: Start continuous mode */
    else if(cmd == 104)
    {
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            status = endat_start_continuous_mode(handle[i]);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_start_continuous_mode failed for instance %d with status %d\n", i, status);
                return SystemP_FAILURE;
            }
        }

        DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n");

        while(1)
        {
            if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
            {
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    status = endat_stop_continuous_mode(handle[i]);
                    if(status != SystemP_SUCCESS)
                    {
                        if(status == SystemP_TIMEOUT)
                        {
                            DebugP_log("\r| ERROR: endat_stop_continuous_mode timed out for instance %d \n", i);
                        }
                        else
                        {
                            DebugP_log("\r| ERROR: endat_stop_continuous_mode failed for instance %d with status %d\n", i, status);
                        }
                    }
                }
                return SystemP_SUCCESS;
            }
            else
            {
                char_count = 0;

                /* Process all instances */
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                    DebugP_log("\r| --- EnDat Module %d ---|\r\n", i);
#endif
                    endat_print_position_header(handle[i], 1, 0);

                    if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
                    {
                        for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                        {
                            if(attrs[i]->channel_mask & (1 << j))
                            {
                                status = endat_multi_channel_set_cur(handle[i], j);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Ch set failed for EnDAT instance %u ch %u: %d\n", i, j, status);
                                    continue;
                                }
                                status = endat_recvd_process(handle[i], 1, &gAppEndatFormatDataMtrCtrl[i][j]);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Recvd process failed for EnDAT instance %u ch %u: %d\n", i, j, status);
                                    continue;
                                }
                                char_count += endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, 0);
                                DebugP_log("\r\n| Ch-%d: \n", j);
                                endat_print_position_loop(handle[i], 1, 0, j);
                                DebugP_log("\n| ");
                                char_count += 3;
                            }
                        }
                    }
                    else
                    {
                        status = endat_recvd_process(handle[i], 1, &gAppEndatFormatDataMtrCtrl[i][priv[i]->current_channel]);
                        if(status != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: Recvd process failed for EnDAT instance %u: %d\n", i, status);
                            continue;
                        }
                        char_count = endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 1, 0);
                        endat_print_position_loop(handle[i], 1, 0, priv[i]->current_channel);
                        DebugP_log("\n| ");
                    }
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(POSITION_LOOP_DISPLAY_DELAY_US);
                while(char_count--)
                {
                    DebugP_log("%c", 8);
                }
            }
        }
    }
    /* Command 107: Simulate motor control 2.2 position loop */
    else if(cmd == 107)
    {
        us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return SystemP_FAILURE;
        }

        gPositionLoopIrqCount = 0;

        /* This setup is similar to command 101 */
        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        /* reset additional info's if present */
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            status = endat_command_process(handle[i], 5, NULL);
            if(status != SystemP_SUCCESS)
            {
                if(status == SystemP_TIMEOUT)
                {
                    DebugP_log("\r| ERROR: Command processing timed out, endat_command_process failed for instance %d with status %d\n", i, status);
                }
                else
                {
                    DebugP_log("\r| ERROR: endat_command_process failed for instance %d with status %d\n", i, status);
                }
            }
            status = endat_addinfo_track(handle[i], 5, NULL);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_addinfo_track failed for instance %d with status %d\n", i, status);
            }
            gAppEndat22LoopMrs[i] = MRS_POS_VAL2_WORD1;
        }

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        gEndatPositionLoopStatus = ENDAT_POSITION_LOOP_START;

        /* Wait for proper position value 2 to be ready from the beginning */
        ClockP_usleep(us * POSITION_VAL2_READY_DELAY_MULT);

        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            if(attrs[i]->mode != ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
            {
                for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                {
                    if(attrs[i]->channel_mask & (1 << j))
                    {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                        DebugP_log("\r|\n| EnDat Module: %d,  Channel: %d, encoder does not support safety, position value 2 would not be displayed\n|\n ", i, j);
#else
                        DebugP_log("\r|\n| Channel: %d, encoder does not support safety, position value 2 would not be displayed\n|\n ", j);
#endif
                    }
                }
            }
        }

        DebugP_log("\r|\n\r| Press enter to stop the position display\n\r|\n");

        while(1)
        {
            if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    /* reset additional info's if present */
                    status = endat_command_process(handle[i], 5, NULL);
                    if(status != SystemP_SUCCESS)
                    {
                        if(status == SystemP_TIMEOUT)
                        {
                            DebugP_log("\r| ERROR: Command processing timed out, endat_command_process failed for instance %d with status %d\n", i, status);
                        }
                        else
                        {
                            DebugP_log("\r| ERROR: endat_command_process failed for instance %d with status %d\n", i, status);
                        }
                    }
                    status = endat_addinfo_track(handle[i], 5, NULL);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: endat_addinfo_track failed for instance %d with status %d\n", i, status);
                    }
                }
                return SystemP_SUCCESS;
            }
            else
            {
                char_count = 0;

                irq_count = gPositionLoopIrqCount;
                /* Wait till timer IRQ hit */
                while(irq_count == gPositionLoopIrqCount)
                {
                    ;
                }

                /* Process all instances */
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    endat_process_2_2_position_command(handle[i]);

                    if(CONFIG_ENDAT_NUM_INSTANCES > 1)
                    {
                        DebugP_log("\r|\n\r| --- EnDat Module %d ---|\r\n", i);
                    }

                    endat_print_position_header(handle[i], 0, 1);

                    if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
                    {
                        for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                        {
                            if(attrs[i]->channel_mask & (1 << j))
                            {
                                status = endat_multi_channel_set_cur(handle[i], j);
                                if(status != SystemP_SUCCESS)
                                {
                                    DebugP_log("\r| ERROR: Ch set failed for EnDAT instance %u ch %u: %d\n", i, j, status);
                                    continue;
                                }
                                char_count += endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, 1);
                                DebugP_log("\r\n| Ch-%d: \n", j);
                                endat_print_position_loop(handle[i], 0, 1, j);

                                if(priv[i]->has_safety[priv[i]->current_channel])
                                {
                                    max = (uint64_t)1 << priv[i]->single_turn_res[priv[i]->current_channel];

                                    multi_turn = ENDAT_GET_POS_MULTI_TURN(gAppEndat22PosVal2[i][j], priv[i]);
                                    single_turn = ENDAT_GET_POS_SINGLE_TURN(gAppEndat22PosVal2[i][j], priv[i]);

                                    if(priv[i]->type[priv[i]->current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                                    {
                                        position.angle = (float)single_turn / (float)max * (float)360;
                                    }
                                    else
                                    {
                                        position.length = single_turn * priv[i]->step[priv[i]->current_channel];
                                    }

                                    DebugP_log(", ");

                                    if(priv[i]->multi_turn_res[priv[i]->current_channel])
                                    {
                                        sprintf(gUartBuffer, "%16.12f, %16s", position.angle, uint64_to_str(multi_turn));
                                    }
                                    else
                                    {
                                        if(priv[i]->type[priv[i]->current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                                        {
                                            sprintf(gUartBuffer, "%16.12f", position.angle);
                                        }
                                        else
                                        {
                                            sprintf(gUartBuffer, "%16s", uint64_to_str(position.length));
                                        }
                                    }

                                    DebugP_log("%s", gUartBuffer);
                                    DebugP_log(",    %10u", gAppEndat22CrcAddinfo1ErrCnt[i][j]);
                                }

                                DebugP_log("\n| ");
                                char_count += 3;
                            }
                        }
                    }
                    else
                    {
                        char_count = endat_get_position_loop_chars(priv[i]->multi_turn_res[priv[i]->current_channel], 0, 1);
                        endat_print_position_loop(handle[i], 0, 1, priv[i]->current_channel);

                        if(priv[i]->has_safety[priv[i]->current_channel])
                        {
                            max = (uint64_t)1 << priv[i]->single_turn_res[priv[i]->current_channel];

                            multi_turn = ENDAT_GET_POS_MULTI_TURN(gAppEndat22PosVal2[i][0], priv[i]);
                            single_turn = ENDAT_GET_POS_SINGLE_TURN(gAppEndat22PosVal2[i][0], priv[i]);

                            if(priv[i]->type[priv[i]->current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                            {
                                position.angle = (float)single_turn / (float)max * (float)360;
                            }
                            else
                            {
                                position.length = single_turn * priv[i]->step[priv[i]->current_channel];
                            }

                            DebugP_log(", ");

                            if(priv[i]->multi_turn_res[priv[i]->current_channel])
                            {
                                sprintf(gUartBuffer, "%16.12f, %16s", position.angle, uint64_to_str(multi_turn));
                            }
                            else
                            {
                                if(priv[i]->type[priv[i]->current_channel] == ENDAT_ENCODER_TYPE_ROTARY)
                                {
                                    sprintf(gUartBuffer, "%16.12f", position.angle);
                                }
                                else
                                {
                                    sprintf(gUartBuffer, "%16s", uint64_to_str(position.length));
                                }
                            }

                            DebugP_log("%s", gUartBuffer);
                            DebugP_log(",    %10u", gAppEndat22CrcAddinfo1ErrCnt[i][0]);

                            if(priv[i]->multi_turn_res[priv[i]->current_channel])
                            {
                                char_count += 2 + 46 + 3;
                            }
                            else
                            {
                                char_count += 2 + 28 + 3;
                            }
                        }

                        DebugP_log("\n| ");
                    }
                }

                while(char_count--)
                {
                    DebugP_log("%c", 8);
                }
            }
        }
    }
    /* Command 111: Simulate motor control 2.1 position loop for long time */
    else if(cmd == 111)
    {
        DebugP_log("\r|press enter to stop the long time continuous mode\n|");

        position_read = 0;
        /*clear CRC error count */
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            memset(gAppEndatMtrCtrlCrcErr[i], 0, sizeof(gAppEndatMtrCtrlCrcErr[i]));
        }
        while(1)
        {
            if(gEndatPositionLoopStatus == ENDAT_POSITION_LOOP_STOP)
            {
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    if(CONFIG_ENDAT_NUM_INSTANCES > 1)
                    {
                        DebugP_log("\r| --- EnDat Module %d ---|\r\n", i);
                    }
                    if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
                    {
                        for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                        {
                            if(attrs[i]->channel_mask & (1 << j))
                            {
                                DebugP_log("\r -------Channel %u ------\n", j);
                                DebugP_log("\r position command sent = %llu \n", position_read);
                                DebugP_log("\r CRC failures encountered = %u \n", gAppEndatMtrCtrlCrcErr[i][j]);
                                DebugP_log(" ");
                                DebugP_log("\r");
                            }
                        }
                    }
                    else
                    {
                        DebugP_log("\r position command sent = %llu \n", position_read);
                        DebugP_log("\r CRC failures encountered = %u \n", gAppEndatMtrCtrlCrcErr[i][priv[i]->current_channel]);
                        DebugP_log(" ");
                        DebugP_log("\r");
                    }
                }
                return SystemP_SUCCESS;
            }
            else
            {
                position_read++;

                /* Process position command */
                for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
                {
                    endat_process_2_1_position_command(handle[i]);
                }
                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(POSITION_LOOP_2_1_DELAY_US);
            }
        }
    }
    return SystemP_SUCCESS;
}

static void endat_handle_rx(endat_handle handle, int32_t cmd)
{
    const endat_priv *priv = endat_get_priv(handle);
    uint32_t crc;
    endat_format_data endat_format_data;
    int32_t status;

    if((handle == NULL) || (priv == NULL))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv\n");
        return;
    }

    /* Display raw data if enabled */
    if(priv->raw_data)
    {
        endat_display_raw_data(handle, cmd);
    }

    /* Process, validate, and print received data */
    status = endat_recvd_process(handle, cmd, &endat_format_data);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: Recvd process failed: %d\n", status);
        return;
    }
    crc = endat_recvd_validate(handle, cmd, &endat_format_data);
    endat_recvd_print(handle, cmd, &endat_format_data, crc);
}

static void endat_print_encoder_info(endat_handle handle)
{
    endat_priv *priv = endat_get_priv(handle);
    const endat_attrs *attrs = endat_get_attrs(handle);
    uint32_t ch;
    const char *encoder_type;

    if((handle == NULL) || (priv == NULL) || (attrs == NULL) || (priv->current_channel >= ENDAT_NUM_CH_PER_SLICE_MAX))
    {
        DebugP_log("\r\n\nERROR: NULL handle/priv/attrs or invalid current_channel\n");
        return;
    }

    ch = priv->current_channel;
    encoder_type = (priv->type[ch] == ENDAT_ENCODER_TYPE_ROTARY) ? "rotary" : "linear";

    /* Print encoder type */
    DebugP_log("\rEnDat 2.%d %s encoder \n",
                priv->cmd_set_2_2 ? 2 : 1,
                encoder_type);

    /* Print encoder ID */
    DebugP_log("\rID: %u %s \n",
                priv->id.binary, (char *)&priv->id.ascii);

    /* Print serial number */
    DebugP_log("\rSN: %c %u %c \n",
                (char)priv->sn.ascii_msb, priv->sn.binary, (char)priv->sn.ascii_lsb);

    /* Print position resolution */
    DebugP_log("\rPosition: %d bits \n", priv->pos_res);

    if(priv->type[ch] == ENDAT_ENCODER_TYPE_ROTARY)
    {
        DebugP_log("\rSingleturn: %d, Multiturn: %d\n",
                    priv->single_turn_res[ch],
                    priv->multi_turn_res[ch]);
    }

    /* Print resolution */
    DebugP_log("\rResolution: %d %s \n",
                priv->step[ch],
                (priv->type[ch] == ENDAT_ENCODER_TYPE_ROTARY) ? "M/rev" : "nm");

    /* Print propagation delay */
    DebugP_log("\rPropagation delay: %dns \n",
                gAppEndatPropDelay[attrs->instance][ch]);
}

/**
 * \brief   EnDAT diagnostic application main function
 *
 * \details This function implements the main diagnostic application flow for EnDAT
 *          encoder interface. It initializes the EnDAT driver, loads and starts PRU
 *          firmware, and provides an interactive menu-driven interface for various
 *          encoder operations including:
 *          - Position data acquisition (single-shot and continuous)
 *          - Additional information read/write
 *          - Encoder parameter configuration
 *          - Memory operations
 *          - Test command operations
 *          - Propagation delay configuration
 *
 *          Flow:
 *          1. Initialize SoC drivers and board drivers
 *          2. Enable booster pack power pins if configured
 *          3. Display firmware version
 *          4. Initialize PRU-ICSS subsystem
 *          5. Initialize EnDAT driver for all instances
 *          6. Load and run PRU firmware(s)
 *          7. Read encoder information at low frequency
 *          8. Calculate and store propagation delays
 *          9. Configure default operating frequency based on encoder type
 *          10. Enter interactive menu loop for encoder operations
 *          11. De-initialize on exit
 *
 *          Encoder Type Detection:
 *          - EnDAT 2.2 encoders: Default frequency 8 MHz (AM243X) or 5 MHz (other SoCs)
 *              - \note Safety command is not supported for multi-channel single-PRU mode.
 *                      This limitation exists because EnDAT 2.2 commands with additional info
 *                      have variable RX length depending on the configuration. If one channel
 *                      enables additional info while another doesn't, the RX lengths differ,
 *                      which cannot be handled in multi-channel single-PRU firmware where all
 *                      channels share the same PRU core and timing constraints.
 *          - EnDAT 2.1 encoders: Default frequency 1 MHz
 *
 *          Clock Configuration:
 *          - Assumes UART clock source for peripheral clock
 *          - ICSSG devices: 192 MHz UART clock
 *          - ICSSM devices: 160 MHz UART clock
 *          - If default clock not achievable, use command 100 to set custom frequency
 *
 *          Multi-Channel Support:
 *          - Single channel single PRU: One encoder per PRU core
 *          - Multi-channel single PRU: Multiple encoders on one PRU core
 *          - Multi-channel multi-PRU (load share): Each channel on separate PRU core
 *          - Propagation delay compensation applied for multi-channel configurations
 *
 *          NOTE on driver APIs:
 *          EnDAT driver APIs use a following validation approach:
 *          - **Handle validation**: All public APIs validate the handle parameter for NULL
 *          - **Array bounds checking**: APIs with array parameters or index parameters perform bounds validation
 *          - **Internal structure validation**: Internal structures (attrs, priv, pruicss_xchg, pruicss_handle)
 *            are validated for NULL before dereferencing to prevent undefined behavior
 *
 * \param[in]   args    Unused
 */

void endat_main(void *args)
{
    uint8_t ch_num;
    uint32_t i = 0;
    uint32_t j = 0;
    uint32_t prop_delay_cnt;
    const endat_attrs *attrs[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};
    endat_priv *priv[CONFIG_ENDAT_NUM_INSTANCES] = {NULL};
    endat_params endat_params_instance[CONFIG_ENDAT_NUM_INSTANCES];
    endat_cmd_supplement cmd_supplement[CONFIG_ENDAT_NUM_INSTANCES];
    int32_t status;
    int32_t cmd;
    int8_t rt_error = 0;
    uint8_t is_cmd_continuous = 0;
    uint32_t rt_status = 0;

    /* ========================================================================== */
    /* STEP 1: Initialize SoC drivers and board drivers                           */
    /* ========================================================================== */
    Drivers_open();  /*Open SoC drivers*/
    Board_driversOpen(); /*Open board-specific drivers*/

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS and ENDAT driver                              */
    /* ========================================================================== */

    /* Get and display ENDAT firmware version from PRU firmware image */
    endat_display_fw_version();

    /* Initialize PRU-ICSS instance, initialize DRAM, and disable PRU cores */
    endat_pruicss_init();

    /* Run the loop for all ENDAT instances defined in SysConfig
     * ASSUMPTIONS:
     *      - i = 0 will use CONFIG_ENDAT0
     *      - i = 1 will use CONFIG_ENDAT1 when ENDAT_DUAL_PRU_SLICE_ENABLE is defined
     */
    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        /* Initialize EnDAT parameters with defaults and set PRU-ICSS handle */
        endat_params_init(&endat_params_instance[i]);
        /* Default values are set by endat_params_init():
         *   - cmd_process_delay_us = 1000us (delay for command processing polling loop)
         *   - fw_wait_delay_us = 1000us (delay for firmware status checks)
         *   - max_wait_loop_count = 1000 (maximum wait loop count, 1000ms with default cmd_process_delay_us)
         *   - channel_rx_info = NULL (must be set before endat_init())
         *   - ch_info_global_addr = 0 (must be set before endat_init())
         * If needed, delay values can be modified before calling endat_init():
         *   endat_params_instance[i].cmd_process_delay_us = <custom_value>;
         *   endat_params_instance[i].fw_wait_delay_us = <custom_value>;
         *   endat_params_instance[i].max_wait_loop_count = <custom_value>;
         */
        endat_params_instance[i].pruicss_handle = gPruIcssXHandle;

        /* Set channel RX info memory - pointer to channel information structure array */
        endat_params_instance[i].channel_rx_info = (endat_ch_rx_info_array *)(&gEndatChInfo[i]);
        /* Translate the TCMB local view addr to SoC view addr for firmware access.
         * ASSUMPTION: Application is running on R5FSS0_CORE0 and gEndatChInfo is defined in R5FSS0_CORE0 BTCM memory region.
         * Note: Need to update CPU0_BTCM_SOCVIEW macro if it is not R5FSS0_CORE0 or BTCM memory.
         */
        endat_params_instance[i].ch_info_global_addr = (uint32_t)(CPU0_BTCM_SOCVIEW((uint32_t)(endat_params_instance[i].channel_rx_info)));

        /* Initialize EnDAT driver instance
         * This calls: endat_hw_init(), endat_config_channel(), endat_config_load_share(), endat_config_multi_channel_mask()
         * endat_set_default_initialization() and endat_config_host_trigger().
         */
        gAppEndatHandle[i] = endat_init(i, &endat_params_instance[i]);
        /* Get pointer to EnDAT attrs and priv */
        attrs[i] = endat_get_attrs(gAppEndatHandle[i]);
        priv[i] = endat_get_priv(gAppEndatHandle[i]);

        DebugP_log("\r\n|------------------------------------------------------------------------------|");
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
        DebugP_log("\r\n EnDAT Instance %u", i);
#endif

        if((gAppEndatHandle[i] == NULL) || (attrs[i] == NULL) || (priv[i] == NULL))
        {
            DebugP_log("\r\nERROR: EnDAT initialization failed for EnDAT instance %u\n", i);
            return;
        }

        /* Display HW instances used, operation mode and enabled channels */
        DebugP_log("\r\nPRU-ICSS instance: %u, PRU-ICSS slice number: %u\n", attrs[i]->pruicss_instance, attrs[i]->pruicss_slice);
        if(attrs[i]->mode == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
        {
            /* Multi-channel multi-PRU (load share) mode: Each channel uses a separate PRU core */
            DebugP_log("\r\nEnDAT Load Share Demo application is running......\n");
            for(ch_num = 0; ch_num < ENDAT_NUM_CH_PER_SLICE_MAX; ch_num++)
            {
                if(attrs[i]->channel_mask & (1 << ch_num))
                {
                    DebugP_log("\r\nChannel %u is enabled\n", ch_num);
                }
            }
        }
        else if(attrs[i]->mode == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)
        {
            /* Multi-channel single PRU mode: Multiple channels handled by one PRU core */
            DebugP_log("\r\nEnDAT Multi channel, Single PRU Demo application is running......\n");
            for(ch_num = 0; ch_num < ENDAT_NUM_CH_PER_SLICE_MAX; ch_num++)
            {
                if(attrs[i]->channel_mask & (1 << ch_num))
                {
                    DebugP_log("\r\nChannel %u is enabled\n", ch_num);
                }
            }
        }
        else
        {
            /* Single channel single PRU mode: One channel on one PRU core */
            DebugP_log("\r\nEnDAT Single channel, Single PRU Demo application is running......\n");
            DebugP_log("\r\nChannel %u is enabled\n", priv[i]->current_channel);
        }

        DebugP_log("\r\n|------------------------------------------------------------------------------|\n\n");
    }

    /* ========================================================================== */
    /* STEP 3: Load and run PRU firmware                                          */
    /* ========================================================================== */
    /* Load PRU firmware image to IRAM, reset and enable PRU cores.
     * The firmware selection depends on configuration mode:
     * - Single channel single PRU: Load single channel firmware
     * - Multi-channel single PRU: Load multi-channel firmware
     * - Multi-channel multi-PRU: Load separate firmware for RTU-PRU, PRU, TX-PRU
     * After loading, wait for firmware initialization acknowledgment.
     */

    DebugP_log("\r\nPRU-ICSS firmware loading started\n");

    endat_pruicss_load_run_fw();

    DebugP_log("\r\nPRU-ICSS firmware loading is complete\n");

    /* Load firmware for all EnDAT instances */
    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        /* Wait for firmware initialization acknowledgment */
        status = endat_wait_initialization(gAppEndatHandle[i], WAIT_5_SECOND, attrs[i]->channel_mask);

        if(status != SystemP_SUCCESS)
        {
            if(status == SystemP_TIMEOUT)
            {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                DebugP_log("\rERROR: EnDAT initialization timeout for instance %d -\n\n", i);
#else
                DebugP_log("\rERROR: EnDAT initialization timeout -\n\n");
#endif 
            }
            else
            {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
                DebugP_log("\rERROR: EnDAT initialization failed for instance %d -\n\n", i);
#else
                DebugP_log("\rERROR: EnDAT initialization failed -\n\n");
#endif 
            }

        if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {
            ch_num = endat_multi_channel_detected(gAppEndatHandle[i]) & (attrs[i]->channel_mask);
            ch_num ^= (attrs[i]->channel_mask);
            DebugP_log("\r\tunable to detect encoder in channel %s %s %s\n",
                        ch_num & (1 << 0) ? "0" : "",
                        ch_num & (1 << 1) ? "1" : "",
                        ch_num & (1 << 2) ? "2" : "");
        }
        else
        {
            DebugP_log("\r\tcheck whether encoder is connected and ensure proper connections\n");
        }

            DebugP_log("\r\nExit %s due to failed firmware initialization\n", __func__);
            goto deinit;
        }
    }

    /* ========================================================================== */
    /* STEP 4: Read encoder information and calculate propagation delays          */
    /* ========================================================================== */
    /* For each EnDAT instance:
     * 1. Set clock to initialization frequency (200 kHz) for reliable communication
     * 2. Read encoder information (manufacturer, type, serial number, resolution, etc.)
     * 3. Measure propagation delay from PRU firmware
     * 4. Convert propagation delay from clock cycles to nanoseconds
     * 5. For multi-channel mode: Find maximum propagation delay for wire delay compensation
     * 6. Display encoder information for each channel
     *
     * Propagation delay is critical for:
     * - Accurate timing of encoder transactions
     * - Wire delay compensation in multi-channel configurations
     * - Ensuring all channels are synchronized properly
     */

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {

        cmd_supplement[i].frequency = ENDAT_INIT_FREQ;
        endat_process_host_command(gAppEndatHandle[i], 100, &cmd_supplement[i]);
        if(attrs[i]->mode == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
        {

            status = endat_get_encoder_info(gAppEndatHandle[i]);
            if(status != SystemP_SUCCESS)
            {
                if(status == SystemP_TIMEOUT)
                {
                    DebugP_log("\rEnDat initialization timeout during channel information read for EnDAT instance %u\n", i);
                }
                else
                {
                    DebugP_log("\rEnDat initialization failed during channel information read for EnDAT instance %u\n", i);
                }
                DebugP_log("\rexit %s due to failed initialization for EnDAT instance %u\n", __func__, i);
                goto deinit;
            }

            status = endat_get_prop_delay(gAppEndatHandle[i], &prop_delay_cnt);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\rEnDat get propagation delay failed for EnDAT instance %u\n", i);
                DebugP_log("\rexit %s due to failed propagation delay read for EnDAT instance %u\n", __func__, i);
                goto deinit;
            }
            gAppEndatPropDelay[i][priv[i]->current_channel] = prop_delay_cnt*((float)(1000000000)/attrs[i]->core_clk_freq);

            DebugP_log("\r\nCHANNEL %d \n", priv[i]->current_channel);
            endat_print_encoder_info(gAppEndatHandle[i]);
        }
        else
        {
            for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
            {
                if(attrs[i]->channel_mask & (1 << j))
                {
                    status = endat_multi_channel_set_cur(gAppEndatHandle[i], j);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\rCh %d switch failed: %d\n", j, status);
                        goto deinit;
                    }

                    status = endat_get_encoder_info(gAppEndatHandle[i]);
                    if(status != SystemP_SUCCESS)
                    {
                        if(status == SystemP_TIMEOUT)
                        {
                            DebugP_log("\rEnDat initialization timeout during channel information read\n");
                        }
                        else
                        {
                            DebugP_log("\rEnDat initialization failed during channel information read\n");
                        }
                        DebugP_log("\rexit %s due to failed initialization\n", __func__);
                        goto deinit;
                    }

                    status = endat_get_prop_delay(gAppEndatHandle[i], &prop_delay_cnt);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\rEnDat get propagation delay failed for channel %d\n", j);
                        DebugP_log("\rexit %s due to failed propagation delay read\n", __func__);
                        goto deinit;
                    }
                    gAppEndatPropDelay[i][priv[i]->current_channel] = prop_delay_cnt*((float)(1000000000)/attrs[i]->core_clk_freq);

                    DebugP_log("\r\nCHANNEL %d\n", j);
                    endat_print_encoder_info(gAppEndatHandle[i]);
                }
            }
            /* Find max propagation delay among all channels to compensate wire delay */
            gAppEndatPropDelayMax[i] = gAppEndatPropDelay[i][0] > gAppEndatPropDelay[i][1] ?
                                   gAppEndatPropDelay[i][0] : gAppEndatPropDelay[i][1];
            gAppEndatPropDelayMax[i] = gAppEndatPropDelayMax[i] > gAppEndatPropDelay[i][2] ?
                                   gAppEndatPropDelayMax[i] : gAppEndatPropDelay[i][2];

        }
    }

    /* ========================================================================== */
    /* STEP 5: Configure EnDAT operating frequency based on encoder type          */
    /* ========================================================================== */
    /* Configure operating frequency for each EnDAT instance using command 100.
     * Frequency selection is based on detected encoder type:
     * - EnDAT 2.2 encoders: 8 MHz (AM243X) or 5 MHz (other SoCs)
     * - EnDAT 2.1 encoders: 1 MHz
     *
     * ASSUMPTIONS:
     * - All encoders connected to same PRU slice are of same type (2.1 or 2.2)
     * - 3-channel peripheral clock source is UART
     * - ICSSG devices: 192 MHz UART clock
     * - ICSSM devices: 160 MHz UART clock
     *
     * NOTE:
     * Use command 100 from the interactive menu to manually change frequency if needed.
     *
     * The configuration includes:
     * - Clock divisor calculation based on source clock and target frequency
     * - Propagation delay compensation for multi-channel configurations
     * - Wire delay configuration to synchronize channels
     * - tST (safety time) configuration: 2us for frequencies >= 1 MHz, 0 otherwise
     */

    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        if(priv[i]->cmd_set_2_2)
        {
#if defined(SOC_AM261X)
            cmd_supplement[i].frequency = 5 * 1000 * 1000;
#else
            cmd_supplement[i].frequency = 8 * 1000 * 1000;
#endif
        }
        else
        {
            cmd_supplement[i].frequency = 1 * 1000 * 1000;
        }

#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
        DebugP_log("\r\nEnDAT Instance %u: Configuring clock frequency to %u Hz (EnDAT %s)\n",
                   i, cmd_supplement[i].frequency, priv[i]->cmd_set_2_2 ? "2.2" : "2.1");
#else
        DebugP_log("\r\nConfiguring clock frequency to %u Hz (EnDAT %s)\n",
                   cmd_supplement[i].frequency, priv[i]->cmd_set_2_2 ? "2.2" : "2.1");
#endif

        endat_process_host_command(gAppEndatHandle[i], 100, &cmd_supplement[i]);

#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
        DebugP_log("\r\nEnDAT Instance %u: Clock configuration completed\n", i);
#else
        DebugP_log("\r\nClock configuration completed\n");
#endif
    }

    DebugP_log("\r\n|--------------------------------------------------------------------------------|\n");
    DebugP_log("\r| NOTE: If target frequency is not achievable with the current clock source,     |\n");
    DebugP_log("\r|       Use command 100 from the menu to manually set a different frequency.     |\n");
    DebugP_log("\r|--------------------------------------------------------------------------------|\n\n");

    /* ========================================================================== */
    /* STEP 6: Interactive menu loop for encoder operations                       */
    /* ========================================================================== */
    /* Enter the main application loop to process user commands.
     * The application supports two types of commands:
     * 1. Host commands (100-106): Configuration and control commands
     *    - Clock frequency configuration
     *    - Propagation delay settings
     *    - Wire delay configuration
     *    - Motor control simulation modes
     * 2. Encoder commands (1-14): Communication with encoder
     *    - Position read (single-shot and continuous)
     *    - Additional information read
     *    - Parameter read/write
     *    - Memory operations
     *    - Test commands
     *
     * For dual PRU slice configurations:
     * - Commands are executed sequentially on both instances
     * - Each instance can have independent settings via cmd_supplement array
     * - Continuous mode commands run simultaneously on all instances
     */
    while(1)
    {
        j = 0;
        rt_error = 0;
        is_cmd_continuous = 0;
        rt_status = 0;

        status = endat_handle_user(gAppEndatHandle, cmd_supplement);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: endat_handle_user() failed, skipping command execution\r\n|\r\n|\n");
            continue;
        }

        /*Check if any of the Encoder instance has Continuous mode Encoder communication command*/
        is_cmd_continuous = 0;
        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
            cmd = cmd_supplement[i].cmd_type;
            if(cmd == SystemP_FAILURE)
            {
                DebugP_log("\r| ERROR: Invalid command for instance %d, skipping command execution for this instance\r\n|\r\n|\n", i);
                continue;
            }
            if(VALID_CONT_MODE_CMD(cmd))
            {
                is_cmd_continuous = 1;
                break;
            }
        }

        /*Continuous mode*/
        /*ASSUMPTION: All ENCODERS assuming INSTANCE 0 configuration are used for all instances*/
        if(is_cmd_continuous == 1 && VALID_CONT_MODE_CMD(cmd_supplement[CONFIG_ENDAT0].cmd_type))
        {
            DebugP_log("\r\n| Command %d will be executed on all encoders |\n\n", cmd);
            status = endat_process_continuous_mode_command(gAppEndatHandle, cmd, cmd_supplement);

            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: Continuous mode command failed.\n");
                while(gEndatPositionLoopStatus != ENDAT_POSITION_LOOP_STOP)
                {
                    ;
                }
            }

            /* Restore host trigger for periodic mode commands (200/201) on any exit */
            if((cmd == 200 || cmd == 201))
            {
                for(j = 0; j < CONFIG_ENDAT_NUM_INSTANCES; j++)
                {
                    status = endat_config_host_trigger(gAppEndatHandle[j]);
                    if(status != SystemP_SUCCESS)
                    {
                        DebugP_log("\r| ERROR: Host trigger restore failed for instance %d: %d\n", j, status);
                    }
                }
            }

            DebugP_log("\r|\n\r|\n");
            continue;
        }
        if(is_cmd_continuous == 1 && (!VALID_CONT_MODE_CMD(cmd_supplement[CONFIG_ENDAT0].cmd_type)))
        {
            DebugP_log("\r\n| Configure continuous mode command for instance 0 `CONFIG_ENDAT0'. Command will be executed on all ENCODERS |\n\n");
            continue;
        }

        for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
        {
#if (CONFIG_ENDAT_NUM_INSTANCES > 1)
            DebugP_log("\r\n| Command execution for Endat Module: %d |\n", i);
#endif
            cmd = cmd_supplement[i].cmd_type;

            /*Host command*/
            if(VALID_HOST_CMD(cmd))
            {
                endat_process_host_command(gAppEndatHandle[i], cmd, &cmd_supplement[i]);
                DebugP_log("\r|\n\r|\n");
                continue;
            }
            status = endat_command_process(gAppEndatHandle[i], cmd, &cmd_supplement[i]);
            if(status != SystemP_SUCCESS)
            {
                if(status == SystemP_TIMEOUT)
                {
                    DebugP_log("\r| ERROR: Command %d timeout occurred for EnDAT instance %u\n", cmd, i);
                }
                else
                {
                    DebugP_log("\r| ERROR: Failed to process command, endat_command_process failed for EnDAT instance %u\r\n|\r\n|\n", i);
                }
                continue;
            }

            if(attrs[i]->mode != ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU)
            {
                DebugP_log("\r|\n");

                for(j = 0; j < ENDAT_NUM_CH_PER_SLICE_MAX; j++)
                {
                    if(attrs[i]->channel_mask & (1 << j))
                    {
                        status = endat_multi_channel_set_cur(gAppEndatHandle[i], j);
                        if(status != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: Ch set failed for EnDAT instance %u ch %u: %d\n", i, j, status);
                            continue;
                        }
                        DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", j);
                        endat_handle_rx(gAppEndatHandle[i], cmd);
                        /* Recovery Time validation */
                        if(endat_status_rt_measurement(gAppEndatHandle[i], &rt_status) == SystemP_SUCCESS && rt_status == 1)
                        {
                            if(endat_check_rt_error(gAppEndatHandle[i], &rt_error) == SystemP_SUCCESS && rt_error != ENDAT_RT_NO_ERROR)
                            {
                                if(rt_error == ENDAT_RT_COUNTER_STUCK_ERROR)
                                {
                                    DebugP_log("\r Error: Counter for Channel %d is stuck.\n", j);
                                }
                                else
                                {
                                    DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", j);
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                endat_handle_rx(gAppEndatHandle[i], cmd);
                /* Recovery Time validation */
                if(endat_status_rt_measurement(gAppEndatHandle[i], &rt_status) == SystemP_SUCCESS && rt_status == 1)
                {
                    if(endat_check_rt_error(gAppEndatHandle[i], &rt_error) == SystemP_SUCCESS && rt_error != ENDAT_RT_NO_ERROR)
                    {
                        if(rt_error == ENDAT_RT_COUNTER_STUCK_ERROR)
                        {
                            DebugP_log("\r Error: Counter for Channel %d is stuck. \n", priv[i]->current_channel);
                        }
                        else
                        {
                            DebugP_log("\r Error: Channel %d - Recovery time out of expected range. \n", priv[i]->current_channel);
                        }
                    }
                }
            }

            /* this cannot be done except as last in loop; additional info becomes applicable from next command onwards only */
            status = endat_addinfo_track(gAppEndatHandle[i], cmd, &cmd_supplement[i]);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat_addinfo_track failed for instance %d with status %d\n", i, status);
            }
        }

    }
deinit:
    /* ========================================================================== */
    /* STEP 7: Deinitialize ENDAT driver and other drivers                        */
    /* ========================================================================== */
    for(i = 0; i < CONFIG_ENDAT_NUM_INSTANCES; i++)
    {
        endat_deinit(gAppEndatHandle[i]);
    }

    Board_driversClose();
    Drivers_close();
    return;
}
