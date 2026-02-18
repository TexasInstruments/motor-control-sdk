/*
 * Copyright (C) 2025-2026 Texas Instruments Incorporated
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

/**
 * \file endat3_diagnostic.c
 *
 * \brief EnDat3 diagnostic application demonstrating encoder communication
 *
 * \details This application provides a comprehensive diagnostic interface for testing
 *          and validating EnDat3 encoder communication. It demonstrates:
 *          - Host trigger and periodic trigger modes
 *          - Position data acquisition and display
 *          - Memory parameter read/write operations
 *          - Error detection
 *
 * ## Application Flow
 *
 * The application follows this sequence:
 *
 * **STEP 1: System Initialization**
 * - Initialize SoC drivers
 * - Set up PRU-ICSS instance
 *
 * **STEP 2: PRU-ICSS and EnDat3 Driver Initialization**
 * - Initialize PRU-ICSS subsystem (endat3_pruicss_init)
 * - Open EnDat3 driver instance using SysConfig-generated parameters
 * - Validate driver initialization
 *
 * **STEP 3: Firmware Loading**
 * - Load EnDat3 PRU firmware to appropriate PRU core
 * - Verify firmware version and compatibility
 * - Enable PRU core execution
 *
 * **STEP 4: Interactive Diagnostic Menu**
 * - Present user menu for encoder operations
 * - Execute selected diagnostic command
 *      - Position Data Acquisition
 *          - Support continuous position fetch (host trigger mode)
 *          - Support periodic position fetch (IEP timer-driven)
 *          - Display position, status, and error information
 *      - Memory Operations
 *          - Read/write encoder memory parameters
 *          - Display operation results
 *
 * **STEP 5: Cleanup and Exit**
 * - Deinitialize EnDat3 driver
 * - Close SoC drivers
 *
 * ## Trigger Modes
 *
 * The application supports two trigger modes:
 *
 * - **Host Trigger Mode**: Application explicitly triggers each encoder transaction
 *   via API calls. Provides precise control but requires active CPU involvement.
 *
 * - **Periodic Mode**: PRU-ICSS IEP timer automatically triggers encoder transactions
 *   at configured intervals. Reduces CPU load for continuous position monitoring.
 *
 * ## Configuration Validation
 *
 * The driver validates all configuration parameters during endat3_init():
 * - ENDAT3 instance and mode
 * - PRU-ICSS instance and slice selection
 * - Channel mask and enabled channels
 * - Baud rate and clock configuration
 *
 * Once initialized, the driver assumes these parameters remain valid.
 *
 * ## Error Handling Strategy
 *
 * The application follows these error handling principles:
 * - All driver API calls check return values for detailed error codes
 * - Encoder-level errors (HPF, LPH) displayed to user
 * - Communication failures result in menu return, not application exit
 *
 * ## Driver Validation Strategy
 *
 * The EnDAT3 driver uses an optimized validation approach:
 * - **Handle validation**: All public APIs validate the handle parameter for NULL
 * - **Array bounds checking**: APIs with array/index parameters validate bounds before access
 * - **Internal structure validation**: Internal structures (attrs, priv, endat3_interface,
 *   pruicss_handle) are validated for NULL before dereferencing to prevent undefined behavior
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <position_sense/endat3/include/endat3_drv.h>
#include <drivers/hw_include/hw_types.h>
#include <math.h>
#include <stdbool.h>
#include <drivers/soc.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss.h>
#include "endat3_periodic_trigger.h"

#if(CONFIG_ENDAT3_0_PRUICSS_PRU_ID == 1)
#include <position_sense/endat3/firmware/single_channel/endat3_receiver_pru1_bin.h>
#else
#include <position_sense/endat3/firmware/single_channel/endat3_receiver_pru0_bin.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Continuous position loop stop signal value */
#define ENDAT3_POSITION_LOOP_STOP       (0U)
/** \brief Continuous position loop start/running signal value */
#define ENDAT3_POSITION_LOOP_START      (1U)

/** \brief Bit mask for 30-bit position data (EnDAT3 standard position resolution) */
#define POSITION_MASK_30BIT             (0x3FFFFFFF)
/** \brief Full rotation angle in degrees (360.0) for angle calculation */
#define ANGLE_FULL_ROTATION             (360.0f)
/** \brief Position data bit resolution (30 bits for EnDAT3 encoders) */
#define ANGLE_BIT_RESOLUTION            (30U)
/** \brief Maximum position value (2^30) for position data range */
#define POSITION_MAX_VALUE              (1UL << ANGLE_BIT_RESOLUTION)

/** \brief Delay in microseconds between IRQ counter polling in periodic mode (1 us) */
#define PERIODIC_MODE_POLL_SLEEP_US     (1U)
/** \brief Delay in microseconds between position reads in continuous mode (1 ms) */
#define CONTINUOUS_MODE_DELAY_US        (1000U)
/** \brief Delay when switching between operating modes (100 ms) */
#define MODE_SWITCH_DELAY_US            (100000U)
/** \brief Standard 1 second delay in microseconds */
#define DELAY_1_SEC                     (1000000U)
/** \brief 302 millisecond delay for encoder stabilization */
#define DELAY_302_MILLISEC              (302000U)

/** \brief Fixed value for DATANOP test command (0x0000) */
#define DATANOP_FIXED_VALUE             (0x0000)
/** \brief Fixed value for HELLO test command (0x2222) */
#define HELLO_FIXED_VALUE               (0x2222)

/** \brief Stack size in bytes for continuous mode task (4 KB) */
#define TASK_STACK_SIZE                 (4096U)
/** \brief Priority level for continuous mode task in FreeRTOS */
#define TASK_PRIORITY                   (6U)

/* Command Type Defines */
/** \brief Command type: Foreground command with immediate response */
#define ENDAT3_CMD_TYPE_FOREGROUND      (0U)
/** \brief Command type: Background command executed during next position read */
#define ENDAT3_CMD_TYPE_BACKGROUND      (1U)
/** \brief Command type: Continuous position reading mode (loop with host trigger mode) */
#define ENDAT3_CMD_TYPE_CONTINUOUS      (2U)
/** \brief Command type: Periodic mode with IEP CMP based trigger */
#define ENDAT3_CMD_TYPE_PERIODIC_CMP    (3U)
/** \brief Command type: Periodic mode with IEP CAP based trigger */
#define ENDAT3_CMD_TYPE_PERIODIC_CAP    (4U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* ENDAT3 Driver Handle */
endat3_handle gAppEndat3Handle[CONFIG_ENDAT3_NUM_INSTANCES] = {NULL};

/* ENDAT3 Periodic Interface Struct Instance */
endat3_periodic_interface gEndat3PeriodicInterface;

static volatile int32_t gEndat3PositionLoopStatus;

/* IRQ count from periodic trigger (defined in endat3_periodic_trigger.c) */
extern volatile uint32_t gPruEndat3IrqCnt[CONFIG_ENDAT3_NUM_INSTANCES];

/* Task management for periodic mode */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static void endat3_display_position_info(endat3_handle handle);
static void endat3_display_error_status(endat3_handle handle);
static void endat3_pruicss_init(void);
static void endat3_pruicss_load_run_fw(void);
static void endat3_continuous_position_fetch(endat3_handle handle);
static void endat3_position_loop_decide_termination(void *args);
static int32_t endat3_loop_task_create(void);
static int32_t endat3_process_periodic_command(endat3_handle handle[], uint8_t is_cap_mode);
static void endat3_display_fw_version(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Display position information from encoder
 *
 * \param handle Pointer to ENDAT3 Handle
 */
static void endat3_display_position_info(endat3_handle handle)
{
    uint8_t hpf_status;
    uint8_t hpf_data[HPF_DATA_SIZE];
    uint8_t hpf_crc;
    uint8_t lph_status;
    uint8_t lpf_status;
    uint8_t lpf_data[LPF_DATA_SIZE];
    uint8_t lpf_crc;
    uint8_t has_error;
    uint8_t has_warning;
    uint8_t is_valid;
    uint8_t has_absolute;

    DebugP_log("\r\n Position Information:");

    /* Get HPF status */
    if(endat3_get_hpf_status(handle, &hpf_status) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n Error: Failed to get HPF status");
        return;
    }
    DebugP_log("\r\n HPF Status: 0x%02X", hpf_status);

    if(endat3_has_hpf_error(handle, &has_error) == ENDAT3_SUCCESS && has_error)
    {
        DebugP_log("\r\n - Error detected (F bit set)");
    }

    if(endat3_has_hpf_warning(handle, &has_warning) == ENDAT3_SUCCESS && has_warning)
    {
        DebugP_log("\r\n - Warning detected (W bit set)");
    }

    /* Check if HPF data is valid */
    if(endat3_is_hpf_data_valid(handle, &is_valid) != ENDAT3_SUCCESS || !is_valid)
    {
        DebugP_log("\r\n - HPF data invalid (HPFV bit not set)");
        return; /* Don't display invalid data */
    }

    /* Check if absolute value is available */
    if(endat3_has_absolute_value(handle, &has_absolute) == ENDAT3_SUCCESS && !has_absolute)
    {
        DebugP_log("\r\n - Absolute value not available (RM bit not set)");
    }

    /* Get HPF data */
    if(endat3_get_hpf_data(handle, hpf_data) == HPF_DATA_SIZE)
    {
        DebugP_log("\r\n Position Data: 0x%02X%02X%02X%02X",
                   hpf_data[3], hpf_data[2], hpf_data[1], hpf_data[0]);
    }

    /* Get HPF CRC */
    if(endat3_get_hpf_crc(handle, &hpf_crc) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n CRC: 0x%02X", hpf_crc);
    }

    DebugP_log("\r\n Additional Information:");

    /* Get LPH status */
    if(endat3_get_lph_status(handle, &lph_status) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n LPH Status (communication Status): 0x%x", lph_status);
    }

    /* Get LPF status and data */
    if(endat3_get_lpf_status(handle, 0, &lpf_status) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n LPF Status: 0x%x", lpf_status);
    }

    if(endat3_get_lpf_data(handle, 0, lpf_data) == LPF_DATA_SIZE)
    {
        DebugP_log("\r\n LPF Data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                   lpf_data[5], lpf_data[4], lpf_data[3],
                   lpf_data[2], lpf_data[1], lpf_data[0]);
    }

    /* Get LPF CRC */
    if(endat3_get_lpf_crc(handle, 0, &lpf_crc) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n LPF CRC: 0x%02X", lpf_crc);
    }

    /* Display additional position data if available */
    if(hpf_data[4] || hpf_data[5])
    {
        DebugP_log("\r\n Extended Position: 0x%02X%02X",
                   hpf_data[5], hpf_data[4]);
    }
}

/**
 * \brief Display error status information
 *
 * \param handle Pointer to ENDAT3 Handle
 */
static void endat3_display_error_status(endat3_handle handle)
{
    uint8_t hpf_status, lph_status;
    endat3_error_code error_code;
    const char* description;
    const char* action;
    uint8_t has_warning;

    DebugP_log("\r\n HPF and LPH Status:");

    /* Use APIs to get status */
    if(endat3_get_hpf_status(handle, &hpf_status) != ENDAT3_SUCCESS ||
        endat3_get_lph_status(handle, &lph_status) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n Error: Failed to get status");
        return;
    }

    DebugP_log("\r\n HPF Status: 0x%02X", hpf_status);
    DebugP_log("\r\n LPH Status: 0x%02X", lph_status);

    /* Extract error code */
    if(endat3_get_error_code(handle, &error_code) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n Error: Failed to get error code");
        return;
    }

    /* Check for warning */
    if(endat3_has_hpf_warning(handle, &has_warning) == ENDAT3_SUCCESS && has_warning)
    {
        DebugP_log("\r\n Warning detected (W bit set in HPF status)");
    }

    /* Display error information if error code is present */
    if(error_code != ENDAT3_ERR_UNKNOWN)
    {
        DebugP_log("\r\n Error Code: 0x%04X", error_code);

        /* Get description and recommended action */
        description = endat3_get_error_description(error_code);
        action = endat3_get_error_action(error_code);

        /* Display error information */
        DebugP_log("\r\n Description: %s", description);
        DebugP_log("\r\n Recommended action: %s", action);

        /* Special handling for FGERR_RECONFIGURE */
        if(error_code == ENDAT3_FGERR_RECONFIGURE)
        {
            DebugP_log("\r\n NOTE: If this error occurs without a RECONFIGURE command or active");
            DebugP_log("\r\n background BUSY state, the encoder may be defective.");
        }

        /* Special handling for access denied errors */
        if(error_code == ENDAT3_BGERR_USAGE_ACCESS_DENIED)
        {
            DebugP_log("\r\n Access protection error: Current user level is insufficient.");
            DebugP_log("\r\n You may need to authenticate with the AUTH command using a higher");
            DebugP_log("\r\n user level (OEM2, OEM1, or MANUFACTURER) and the correct password.");
        }
    }
}

/**
 * \brief Initialize PRU-ICSS subsystem for EnDat3 operation
 *
 * \details Performs PRU-ICSS initialization including:
 *          - Open PRU-ICSS Handle
 *          - Configure SA-MUX Mode, if needed (only in PRU-ICSSG)
 *          - Clear PRU Data RAM
 *          - Disable PRU core
 *
 * This function must be called before loading PRU firmware and opening ENDAT3 Handle.
 */
static void endat3_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;
    uint8_t pru_id = CONFIG_ENDAT3_0_PRUICSS_PRU_ID;

    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    if(gPruIcssXHandle == NULL)
    {
        DebugP_log("\r\n ERROR: PRUICSS_open failed - NULL handle returned\n");
        DebugP_assert(0);
    }

    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
#ifdef CONFIG_ENDAT3_0_G_MUX_EN
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif

    /* Clear PRU-ICSS DATA RAM for slice*/
    u_status = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(CONFIG_ENDAT3_0_PRUICSS_SLICE));
    DebugP_assert(0 != u_status);

    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
}

/**
 * \brief Load and execute EnDat3 PRU firmware
 *
 * \details This function handles the complete firmware loading sequence:
 *          - Disable PRU core
 *          - Write firmware binary to PRU instruction RAM
 *          - Validate load success
 *          - Reset PRU core
 *          - Enable PRU core to begin execution
 *
 * The firmware binary is selected based on SysConfig settings:
 * - PRU slice assignment
 */
static void endat3_pruicss_load_run_fw(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t u_status = 0;
    const uint32_t *pru_firmware = NULL;
    uint32_t pru_firmware_size = 0;
    uint8_t pru_id = CONFIG_ENDAT3_0_PRUICSS_PRU_ID;

#if(CONFIG_ENDAT3_0_PRUICSS_SLICE == 1)
    pru_firmware = EnDat3FirmwarePru1_0;
    pru_firmware_size = sizeof(EnDat3FirmwarePru1_0);
#else
    pru_firmware = EnDat3FirmwarePru0_0;
    pru_firmware_size = sizeof(EnDat3FirmwarePru0_0);
#endif

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_ENDAT3_0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pru_firmware, pru_firmware_size);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);
}

/**
 * \brief Continuously fetch position data from encoder
 *
 * \param handle EnDAT3 handle for the encoder channel
 */
static void endat3_continuous_position_fetch(endat3_handle handle)
{
    int32_t status = 0;
    uint32_t position = 0;
    float angle = 0.0f;
    uint16_t cmd = ENDAT3_REQ_DATA0;
    int32_t error;
    const uint8_t *rx_buffer;
    uint8_t is_busy;

    /* Create task to monitor stop condition */
    if(endat3_loop_task_create() != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n ERROR: Task creation failed\r\n");
        gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_STOP;
        return;
    }

    gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_START;

    DebugP_log("\r\n Starting continuous position fetch from encoder...\r\n");
    DebugP_log("\r\n Press Enter to stop continuous mode\r\n");

    while(1)
    {
        /* Check if user requested to stop */
        if(gEndat3PositionLoopStatus == ENDAT3_POSITION_LOOP_STOP)
        {
            DebugP_log("\r\n Continuous mode stopped, returning to menu\r\n");
            return;
        }

        /* Set expected TX frames count */
        error = endat3_set_expected_tx_frame_count(handle, 1);
        if(error != ENDAT3_SUCCESS)
        {
            DebugP_log("\r\nERROR: endat3_set_expected_tx_frame_count() failed with error code: %d", error);
            if(error == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            break;
        }

        /* Send DATA0 command to encoder */
        error = endat3_send_command(handle, cmd, 1);
        if(error != ENDAT3_SUCCESS)
        {
            DebugP_log("\r\nERROR: endat3_send_command() failed with error code: %d", error);
            if(error == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            break;
        }

        /* Set busy flag and wait for transmission to complete */
        error = endat3_set_busy(handle, ENCODER_BUSY);
        if(error != ENDAT3_SUCCESS)
        {
            DebugP_log("\r\nERROR: endat3_set_busy() failed with error code: %d", error);
            if(error == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            break;
        }
        while(endat3_is_busy(handle, &is_busy) == ENDAT3_SUCCESS && is_busy);

        /* Receive response from encoder */
        status = endat3_receive_response(handle);

        /* Start with \r to overwrite the same line */
        DebugP_log("\r");

        /* Process response if successful */
        if(status == ENDAT3_SUCCESS)
        {
            /* Get RX buffer */
            rx_buffer = endat3_get_rx_buffer(handle);
            if(rx_buffer == NULL)
            {
                DebugP_log("\r\nERROR: endat3_get_rx_buffer() returned NULL - Invalid handle or interface\r\n");
                break;
            }
            position = (rx_buffer[0]) |
                      ((rx_buffer[1]) << 8) |
                      ((rx_buffer[2]) << 16) |
                      ((rx_buffer[3]) << 24);

            /* Mask upper 2 bits to get 30-bit position value */
            position &= POSITION_MASK_30BIT;

            /* Convert position to angle in degrees (30-bit resolution = 2^30 = 1073741824) */
            angle = ((float)position * ANGLE_FULL_ROTATION) / POSITION_MAX_VALUE;

            /* Print new values */
            DebugP_log("\r Position: 0x%x, Angle: %f degrees", position, angle);
        }
        else
        {
            DebugP_log("\r\nERROR: endat3_receive_response() failed with error code: %d", status);
            if(status == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            break;
        }

        /* Small delay to allow task to check for user input */
        ClockP_usleep(CONTINUOUS_MODE_DELAY_US);
    }
}

/**
 * \brief Task function to monitor user input for stopping periodic mode
 *
 * \param args Pointer to arguments (unused)
 */
static void endat3_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

/**
 * \brief Create task for monitoring periodic mode termination
 */
static int32_t endat3_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "endat3_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)endat3_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\r\nTask creation failed\r\n");
    }

    return status;
}

/**
 * \brief Process periodic trigger command
 *
 * This function implements the complete periodic trigger mode workflow
 * NOTE:
 * - Switch back to host trigger mode is outside this function
 */
static int32_t endat3_process_periodic_command(endat3_handle handle[], uint8_t is_cap_mode)
{
    int32_t status;
    uint8_t current_opmode;
    const uint8_t* rx_buffer;
    uint32_t position;
    float angle;
    uint16_t cmd;
    uint8_t is_busy;
    uint8_t is_valid;
    uint32_t pos_fail_cnt[CONFIG_ENDAT3_NUM_INSTANCES] = {0}, pos_total_cnt = 0;
    uint32_t prev_irq_cnt[CONFIG_ENDAT3_NUM_INSTANCES] = {0};
    uint32_t curr_irq_cnt;

    if((handle == NULL) || handle[CONFIG_ENDAT3_0] == NULL)
    {
        DebugP_log("\r\n\n| ERROR: NULL handle\n");
        return SystemP_FAILURE;
    }

    if(is_cap_mode > 1)
    {
        DebugP_log("\r\n\n| ERROR: Invalid is_cap_mode value\n");
        return SystemP_FAILURE;
    }

    memset(&gEndat3PeriodicInterface, 0, sizeof(endat3_periodic_interface));

    /* Get IEP timer configuration from user and update gEndat3PeriodicInterface */
    if(is_cap_mode)
    {
#if defined(SOC_AM243X)
        DebugP_log("\r| Enter IEP SYNC0 period (must be greater than EnDat3 cycle time including timeout period, in IEP cycles):");
        if(DebugP_scanf("%llu\n", &gEndat3PeriodicInterface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\r\n|\r\n|\r\n|\r\n");
            return SystemP_FAILURE;
        }

        if((gEndat3PeriodicInterface.iep_reset_count == 0) || (gEndat3PeriodicInterface.iep_reset_count <= ENDAT3_IEP_COUNTER_INCREMENT) || (gEndat3PeriodicInterface.iep_reset_count > UINT32_MAX))
        {
            DebugP_log("\r| ERROR:  ERROR: invalid value entered. 0 is not allowed and maximum value allowed is %u\r\n|\r\n|\r\n|\r\n", UINT32_MAX);
            return SystemP_FAILURE;
        }
#else
        DebugP_log("\r| Periodic CAP mode cycle time will be equal to EPWM SYNC OUT frequency. NOTE: In SysConfig, EPWM and EPWM to IEP LATCH XBAR configuration must be done. \n|\n|\n|\n");
#endif
    }
    else
    {
        DebugP_log("\r\n| Enter IEP reset cycle count (must be greater than EnDat3 cycle time including timeout period, in IEP cycles): ");
        if(DebugP_scanf("%llu\n", &gEndat3PeriodicInterface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\r\n|\r\n|\r\n|\r\n");
            return SystemP_FAILURE;
        }

        if((gEndat3PeriodicInterface.iep_reset_count == 0) || (gEndat3PeriodicInterface.iep_reset_count <= ENDAT3_IEP_COUNTER_INCREMENT))
        {
            DebugP_log("\r| ERROR: invalid value\r\n|\r\n|\r\n|\r\n");
            return SystemP_FAILURE;
        }

        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles): ");
        if(DebugP_scanf("%llu\n", &gEndat3PeriodicInterface.periodic_trigger_count[CONFIG_ENDAT3_0]) < 0 )
        {
            DebugP_log("\r| ERROR: invalid value\r\n|\r\n|\r\n|\r\n");
            return SystemP_FAILURE;
        }

        if((gEndat3PeriodicInterface.periodic_trigger_count[CONFIG_ENDAT3_0] > gEndat3PeriodicInterface.iep_reset_count) ||
           (gEndat3PeriodicInterface.periodic_trigger_count[CONFIG_ENDAT3_0] <= ENDAT3_IEP_COUNTER_INCREMENT))
        {
            DebugP_log("\r| ERROR: Trigger time (%u) must be <= Reset cycle (%u) and > %u\r\n", gEndat3PeriodicInterface.periodic_trigger_count[CONFIG_ENDAT3_0], gEndat3PeriodicInterface.iep_reset_count, ENDAT3_IEP_COUNTER_INCREMENT);
            DebugP_log("\r| ERROR: invalid value\r\n|\r\n|\r\n|\r\n");
            return SystemP_FAILURE;
        }
    }

    gEndat3PeriodicInterface.handle[CONFIG_ENDAT3_0] = handle[CONFIG_ENDAT3_0];
    gEndat3PeriodicInterface.is_cap_mode = is_cap_mode;

    if(is_cap_mode)
    {
        /* Set firmware to periodic trigger CAP mode (opmode = 2) */
        DebugP_log("\r\n| Setting firmware to periodic trigger CAP mode...");
        status = endat3_set_operating_mode(handle[CONFIG_ENDAT3_0], ENDAT3_OPMODE_PERIODIC_CAP);

        if(status != ENDAT3_SUCCESS)
        {
            DebugP_log("\r\n| ERROR: endat3_set_operating_mode() failed with error code: %d", status);
            if(status == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            return SystemP_FAILURE;
        }
    }
    else
    {
        /* Set firmware to periodic trigger CMP mode (opmode = 0) */
        DebugP_log("\r\n| Setting firmware to periodic trigger CMP mode...");
        status = endat3_set_operating_mode(handle[CONFIG_ENDAT3_0], ENDAT3_OPMODE_PERIODIC_CMP);

        if(status != ENDAT3_SUCCESS)
        {
            DebugP_log("\r\n| ERROR: endat3_set_operating_mode() failed with error code: %d", status);
            if(status == ENDAT3_ERR_INVALID_INPUT)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            return SystemP_FAILURE;
        }
    }

    /* Create task to monitor stop condition */
    if(endat3_loop_task_create() != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\r\n");
        DebugP_log("Task_create() failed!\n");
        gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_STOP;
        return SystemP_FAILURE;
    }

    /* Configure frame information before releasing trigger
     *
     * Send command once in host trigger mode, to ensure that command data is
     * populated in PRU shared memory as required.
     * ASSUMPTION: Host trigger mode is active when this function is called.
     */

    cmd = ENDAT3_REQ_DATA0;
    DebugP_log("\r\n| Configuring frame information for periodic mode based on ENDAT3_REQ_DATA0 command...");

    if(endat3_set_expected_tx_frame_count(handle[CONFIG_ENDAT3_0], 1) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to set expected frame count\r\n");
        return SystemP_FAILURE;
    }

    if(endat3_send_command(handle[CONFIG_ENDAT3_0], cmd, 1) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to send command\r\n");
        return SystemP_FAILURE;
    }

    if(endat3_set_busy(handle[CONFIG_ENDAT3_0], ENCODER_BUSY) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to set encoder busy state\r\n");
        return SystemP_FAILURE;
    }

    /* Release start trigger */
    if(endat3_release_start_trigger(handle[CONFIG_ENDAT3_0]) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to release start trigger\r\n");
        return SystemP_FAILURE;
    }

    /* Configure and start periodic mode */
    if(endat3_config_periodic_mode(&gEndat3PeriodicInterface) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r| ERROR: endat3_config_periodic_mode failed\r\n|\r\n|\r\n");
        return SystemP_FAILURE;
    }

    DebugP_log("\r\n| Periodic mode configured successfully");
    if(is_cap_mode)
    {
#if defined(SOC_AM243X)
        DebugP_log("\r\n| Reset Cycle   : %u IEP cycles", gEndat3PeriodicInterface.iep_reset_count);
#endif
    }
    else
    {
        DebugP_log("\r\n| Reset Cycle   : %u IEP cycles", gEndat3PeriodicInterface.iep_reset_count);
        DebugP_log("\r\n| Trigger Time  : %u IEP cycles", gEndat3PeriodicInterface.periodic_trigger_count[CONFIG_ENDAT3_0]);
    }

    /* Verify operating mode was set correctly */
    if(endat3_get_operating_mode(handle[CONFIG_ENDAT3_0], &current_opmode) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: Failed to get operating mode\r\n");
        return SystemP_FAILURE;
    }
    DebugP_log("\r\n| Current operating mode: %u (0 = Periodic CMP, 1 = Host trigger, 2 = Periodic CAP)", current_opmode);

    gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_START;

    DebugP_log("\r\n|\n\r\n| Firmware will now trigger automatically on IEP Compare Event");
    DebugP_log("\r\n| Waiting for periodic triggers...");
    DebugP_log("\r\n| Press Enter to stop the continuous mode\r\n|\r\n|");

    /* Initialize previous IRQ count with current value */
    prev_irq_cnt[CONFIG_ENDAT3_0] = gPruEndat3IrqCnt[CONFIG_ENDAT3_0];

    /* Main periodic loop - continuously display position data */
    while(1)
    {
        if(gEndat3PositionLoopStatus == ENDAT3_POSITION_LOOP_STOP)
        {
            /* Stop periodic mode and restore host trigger */
            DebugP_log("\r\n| Stopping periodic mode...");
            DebugP_log("\r\n Failed %u out of %u times\n", pos_fail_cnt[CONFIG_ENDAT3_0], pos_total_cnt);

            /* Stop IEP timer */
            if(endat3_stop_periodic_mode(&gEndat3PeriodicInterface) != ENDAT3_SUCCESS)
            {
                DebugP_log("\r| ERROR: endat3_stop_periodic_mode failed\r\n|\r\n|\r\n");
                return SystemP_FAILURE;
            }
            return SystemP_SUCCESS;
        }
        else
        {

            /* Wait for IRQ count to increment before reading position */
            while(1)
            {
                curr_irq_cnt = gPruEndat3IrqCnt[CONFIG_ENDAT3_0];
                if(gEndat3PositionLoopStatus == ENDAT3_POSITION_LOOP_STOP)
                {
                    break;
                }
                /* Break as soon as IRQ count increments to avoid missing IRQs at high rates */
                if(curr_irq_cnt != prev_irq_cnt[CONFIG_ENDAT3_0])
                {
                    break;
                }
                ClockP_usleep(PERIODIC_MODE_POLL_SLEEP_US);
            }

            /* Check stop condition before updating prev_irq_cnt */
            if(gEndat3PositionLoopStatus == ENDAT3_POSITION_LOOP_STOP)
            {
                continue;
            }

            prev_irq_cnt[CONFIG_ENDAT3_0] = curr_irq_cnt;

            pos_total_cnt++;

            while(endat3_is_busy(handle[CONFIG_ENDAT3_0], &is_busy) == ENDAT3_SUCCESS && is_busy);
            status = endat3_receive_response(handle[CONFIG_ENDAT3_0]);

            /* Start with \r to overwrite the same line */
            DebugP_log("\r");

            /* Process response if successful */
            if(status == ENDAT3_SUCCESS)
            {
                /* Display position info if valid */
                if(endat3_is_hpf_data_valid(handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                {
                    /* Extract position data */
                    rx_buffer = endat3_get_rx_buffer(handle[CONFIG_ENDAT3_0]);

                    if(rx_buffer == NULL)
                    {
                        DebugP_log("\r\nERROR: endat3_get_rx_buffer() returned NULL - Invalid handle or interface\r\n");
                        pos_fail_cnt[CONFIG_ENDAT3_0]++;
                        gEndat3PositionLoopStatus = ENDAT3_POSITION_LOOP_STOP;
                        continue;
                    }

                    position = (rx_buffer[0]) |
                               ((rx_buffer[1]) << 8) |
                               ((rx_buffer[2]) << 16) |
                               ((rx_buffer[3]) << 24);

                    /* Mask upper 2 bits to get 30-bit position value */
                    position &= POSITION_MASK_30BIT;

                    /* Convert position to angle in degrees */
                    angle = ((float)position * ANGLE_FULL_ROTATION) / POSITION_MAX_VALUE;

                    /* Print new values */
                    DebugP_log("\r Position: 0x%x, Angle: %f degrees", position, angle);
                }
            }
            else
            {
                pos_fail_cnt[CONFIG_ENDAT3_0]++;
            }
        }
    }
    return SystemP_SUCCESS;
}

static void endat3_display_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version */
#if(CONFIG_ENDAT3_0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)EnDat3FirmwarePru1_0 + 1);
#else
    version = *((uint32_t *)EnDat3FirmwarePru0_0 + 1);
#endif
    DebugP_log("\r\nENDAT3 Firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
}

void endat3_diagnostic_main(void *args)
{
    uint16_t cmd = 0;
    int32_t status = 0;
    uint8_t cmd_type = 0;
    uint16_t req_data = 0;
    int32_t menu_option = 0;
    uint8_t op_code;
    uint32_t addr_msb = 0, addr_lsb = 0, data = 0;
    uint8_t user_level = 0;
    uint8_t words = 0;
    uint8_t mode = 0, acclevel = 0;
    uint32_t password = 0;
    int32_t reset_type_choice;
    uint16_t clear_flags = 0;
    int32_t clear_f = 0, clear_w = 0, clear_ref = 0;
    uint32_t rate_choice;
    uint32_t mode_input;
    uint32_t acclevel_input;
    uint8_t ret_mode;
    const char *ret_acclevel_desc;
    endat3_bg_cmd_params bg_cmd_params;
    uint32_t expected_frames;
    uint8_t echo_data[HPF_DATA_SIZE];
    uint32_t reset_data;
    uint16_t reset_type;
    uint32_t clear_data;
    uint32_t rate_data;
    uint16_t rate_type;
    uint32_t busbc_data;
    uint32_t busp2p_data;
    uint32_t businit_data;
    uint16_t businit_type;
    uint8_t lph_status;
    endat3_params params;
    endat3_lph_status lph_state;
    uint8_t is_busy;
    uint8_t is_valid;

    /* ========================================================================== */
    /* STEP 1: Initialize system drivers and board peripherals                    */
    /* ========================================================================== */
    /* Open UART console and other board drivers */
    Drivers_open();
    Board_driversOpen();

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS subsystem and memory mapping                   */
    /* ========================================================================== */
    /* Configure PRU-ICSS instance, load shared memory structures */
    endat3_pruicss_init();

    /* ========================================================================== */
    /* STEP 3: Initialize EnDat3 driver and configure interface parameters        */
    /* ========================================================================== */
    /* Initialize EnDat3 parameters and driver */
    endat3_params_init(&params);

    /* Set PRUICSS handle - must be set before calling endat3_init */
    params.pruicss_handle = gPruIcssXHandle;

    /* Encoder operating mode is set to ENDAT3_OPMODE_HOST_TRIGGER in endat3_init */
    gAppEndat3Handle[CONFIG_ENDAT3_0] = endat3_init(CONFIG_ENDAT3_0, &params);
    if(gAppEndat3Handle[CONFIG_ENDAT3_0] == NULL)
    {
        DebugP_log("\r\nERROR: endat3_init() failed\r\n");
        DebugP_log("Please verify SysConfig settings and ensure PRU-ICSS is properly initialized\r\n");
        goto deinit;
    }

    /* ========================================================================== */
    /* STEP 4: Load and execute PRU firmware for EnDat3 protocol                  */
    /* ========================================================================== */
    /* Load firmware binary to PRU instruction memory and start execution */
    endat3_pruicss_load_run_fw();

    endat3_display_fw_version();

    /* ========================================================================== */
    /* STEP 5: Configure trigger initialization                                   */
    /* ========================================================================== */
    /* Clear any previous trigger state */
    status = endat3_clear_start_trigger(gAppEndat3Handle[CONFIG_ENDAT3_0]);
    if(status != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\nERROR: endat3_clear_start_trigger() failed with error code: %d", status);
        if(status == ENDAT3_ERR_INVALID_INPUT)
        {
            DebugP_log(" - Invalid parameters\r\n");
        }
        goto deinit;
    }

    /* It can therefore take up to 300 ms to respond to a HELLO command for initial startup */
    ClockP_usleep(DELAY_1_SEC);

    /* Release start trigger to firmware for initial communication */
    status = endat3_release_start_trigger(gAppEndat3Handle[CONFIG_ENDAT3_0]);
    if(status != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\nERROR: endat3_release_start_trigger() failed with error code: %d", status);
        if(status == ENDAT3_ERR_INVALID_INPUT)
        {
            DebugP_log(" - Invalid parameters\r\n");
        }
        goto deinit;
    }

    ClockP_usleep(DELAY_1_SEC);

    /* ========================================================================== */
    /* STEP 6: Interactive diagnostic menu loop - Process encoder commands        */
    /* ========================================================================== */
    /* Main command loop allowing user to select foreground, background, continuous, or periodic modes */
    while(1)
    {
        DebugP_log("\r\n ========================================");
        DebugP_log("\r\n Select command type:");
        DebugP_log("\r\n 0: Foreground Communication");
        DebugP_log("\r\n 1: Background Communication");
        DebugP_log("\r\n 2: Continuous Position Fetch");
        DebugP_log("\r\n 3: Periodic Trigger Mode (IEP CMP based trigger)");
        DebugP_log("\r\n 4: Periodic Trigger Mode (IEP CAP based trigger)");
        DebugP_log("\r\n ========================================");
        DebugP_scanf("%d", &cmd_type);

        if(cmd_type == ENDAT3_CMD_TYPE_PERIODIC_CMP)
        {
            /* Process periodic trigger mode with dedicated function */
            status = endat3_process_periodic_command(gAppEndat3Handle, 0);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_process_periodic_command() failed ");
            }

            /* Set firmware back to host trigger mode */
            DebugP_log("\r\n| Setting firmware to host trigger mode...");

            status = endat3_set_operating_mode(gAppEndat3Handle[CONFIG_ENDAT3_0], ENDAT3_OPMODE_HOST_TRIGGER);
            if(status != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_set_operating_mode() failed with error code: %d", status);
                if(status == ENDAT3_ERR_INVALID_INPUT)
                {
                    DebugP_log(" - Invalid parameters\r\n");
                }
                break;
            }

            /* Wait for mode switch to complete */
            ClockP_usleep(MODE_SWITCH_DELAY_US);

            DebugP_log("\r\n| Periodic mode stopped, returning to host trigger mode\r\n");
        }
        else if(cmd_type == ENDAT3_CMD_TYPE_PERIODIC_CAP)
        {
            /* Process periodic trigger mode with dedicated function */
            status = endat3_process_periodic_command(gAppEndat3Handle, 1);
            if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_process_periodic_command() failed ");
            }

            /* Set firmware back to host trigger mode */
            DebugP_log("\r\n| Setting firmware to host trigger mode...");

            status = endat3_set_operating_mode(gAppEndat3Handle[CONFIG_ENDAT3_0], ENDAT3_OPMODE_HOST_TRIGGER);
            if(status != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_set_operating_mode() failed with error code: %d", status);
                if(status == ENDAT3_ERR_INVALID_INPUT)
                {
                    DebugP_log(" - Invalid parameters\r\n");
                }
                break;
            }

            /* Wait for mode switch to complete */
            ClockP_usleep(MODE_SWITCH_DELAY_US);

            DebugP_log("\r\n| Periodic mode stopped, returning to host trigger mode\r\n");
        }
        else
        {
            if(cmd_type == ENDAT3_CMD_TYPE_CONTINUOUS)
            {
                /* Continuous position fetch mode */
                DebugP_log("\r\n Starting continuous position fetch mode with ENDAT3_REQ_DATA0...");
                endat3_continuous_position_fetch(gAppEndat3Handle[CONFIG_ENDAT3_0]);
            }
            else if(cmd_type == ENDAT3_CMD_TYPE_FOREGROUND)
            {
                DebugP_log("\r\n Select command for endat3 foreground communication:");
                DebugP_log("\r\n 1: DATA0 (Activate LPF send list 0)");
                DebugP_log("\r\n 2: DATA1 (Activate LPF send list 1)");
                DebugP_log("\r\n 3: DATA2 (Activate LPF send list 2)");
                DebugP_log("\r\n 4: DATA3 (Activate LPF send list 3)");
                DebugP_log("\r\n 5: DATA4 (Activate LPF send list 4)");
                DebugP_log("\r\n 6: DATA5 (Activate LPF send list 5)");
                DebugP_log("\r\n 7: DATA6 (Activate LPF send list 6)");
                DebugP_log("\r\n 8: DATA7 (Activate LPF send list 7)");
                DebugP_log("\r\n 9: DATA (General data with BGD)");
                DebugP_log("\r\n 10: DATANOP (Data without BGD)");
                DebugP_log("\r\n 11: RESET (Encoder reset)");
                DebugP_log("\r\n 12: CLEAR (Resetting of states)");
                DebugP_log("\r\n 13: ECHO (Echo for measuring propagation time)");
                DebugP_log("\r\n 14: RATE (Set data transfer rate)");
                DebugP_log("\r\n 15: HELLO (Switch to EnDat 3 mode)");

                DebugP_scanf("%d", &menu_option);

                /* Map menu option to actual command code */
                switch(menu_option)
                {
                    case ENDAT3_MENU_DATA0:
                        cmd = ENDAT3_REQ_DATA0;
                        break;
                    case ENDAT3_MENU_DATA1:
                        cmd = ENDAT3_REQ_DATA1;
                        break;
                    case ENDAT3_MENU_DATA2:
                        cmd = ENDAT3_REQ_DATA2;
                        break;
                    case ENDAT3_MENU_DATA3:
                        cmd = ENDAT3_REQ_DATA3;
                        break;
                    case ENDAT3_MENU_DATA4:
                        cmd = ENDAT3_REQ_DATA4;
                        break;
                    case ENDAT3_MENU_DATA5:
                        cmd = ENDAT3_REQ_DATA5;
                        break;
                    case ENDAT3_MENU_DATA6:
                        cmd = ENDAT3_REQ_DATA6;
                        break;
                    case ENDAT3_MENU_DATA7:
                        cmd = ENDAT3_REQ_DATA7;
                        break;
                    case ENDAT3_MENU_DATA:
                        cmd = ENDAT3_REQ_DATA;
                        break;
                    case ENDAT3_MENU_DATANOP:
                        cmd = ENDAT3_REQ_DATANOP;
                        DebugP_log("\r\n DATANOP uses fixed value 0x%04X", DATANOP_FIXED_VALUE);
                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, DATANOP_FIXED_VALUE);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_RESET:
                        cmd = ENDAT3_REQ_RESET;
                        DebugP_log("\r\n Select reset type:");
                        DebugP_log("\r\n 1: Hard Reset (0xBBBB)");
                        DebugP_log("\r\n 2: Other (Enter custom value)");

                        DebugP_scanf("%d", &reset_type_choice);

                        if(reset_type_choice == 1)
                        {
                            req_data = ENDAT3_RESET_HARD;
                        } else
                        {
                            DebugP_log("\r\n Enter custom reset type (hex):");
                            DebugP_scanf("%x", &req_data);
                        }
                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, req_data);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_CLEAR:
                        cmd = ENDAT3_REQ_CLEAR;
                        clear_flags = 0;
                        clear_f = 0;
                        clear_w = 0;
                        clear_ref = 0;

                        DebugP_log("\r\n Reset errors (F, SF.Status.F1, F2)? (0/1):");
                        DebugP_scanf("%d", &clear_f);
                        if(clear_f)
                            clear_flags |= ENDAT3_CLEAR_F;

                        DebugP_log("\r\n Reset warning (W)? (0/1):");
                        DebugP_scanf("%d", &clear_w);
                        if(clear_w)
                            clear_flags |= ENDAT3_CLEAR_W;

                        DebugP_log("\r\n Clear absolute value (REF)? (0/1):");
                        DebugP_scanf("%d", &clear_ref);
                        if(clear_ref)
                            clear_flags |= ENDAT3_CLEAR_REF;

                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, clear_flags);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_ECHO:
                        cmd = ENDAT3_REQ_ECHO;
                        DebugP_log("\r\n Enter echo data (0x0-0xffff):");
                        DebugP_scanf("%x", &req_data);
                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, req_data);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_RATE:
                        cmd = ENDAT3_REQ_RATE;
                        DebugP_log("\r\n Select data transfer rate:");
                        DebugP_log("\r\n 1: 12.5 Mbps");

                        DebugP_scanf("%u", &rate_choice);

                        if(rate_choice == 1)
                        {
                            req_data = ENDAT3_RATE_12_5MBPS;
                        }
                        else if(rate_choice == 2)
                        {
                            req_data = ENDAT3_RATE_25MBPS;
                        }
                        else
                        {
                            DebugP_log("\r\n Invalid selection, using 12.5 Mbps");
                            req_data = ENDAT3_RATE_12_5MBPS;
                        }
                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, req_data);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_HELLO:
                        cmd = ENDAT3_REQ_HELLO;
                        DebugP_log("\r\n HELLO uses fixed value 0x%04X", HELLO_FIXED_VALUE);
                        /* Set background data */
                        status = endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, HELLO_FIXED_VALUE);
                        if(status != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\nERROR: endat3_set_bg_data() failed with error code: %d - Invalid handle or interface\r\n", status);
                            break;
                        }
                        break;
                    default:
                        DebugP_log("\r\n Invalid option, using DATA0");
                        cmd = ENDAT3_REQ_DATA0;
                        break;
                }

                /* Use APIs to set operation codes */
                status = endat3_set_background_op_code(gAppEndat3Handle[CONFIG_ENDAT3_0], 0);
                if(status != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: endat3_set_background_op_code() failed with error code: %d", status);
                    if(status == ENDAT3_ERR_INVALID_INPUT)
                    {
                        DebugP_log(" - Invalid parameters\r\n");
                    }
                    break;
                }
                status = endat3_set_foreground_op_code(gAppEndat3Handle[CONFIG_ENDAT3_0], cmd);
                if(status != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: endat3_set_foreground_op_code() failed with error code: %d", status);
                    if(status == ENDAT3_ERR_INVALID_INPUT)
                    {
                        DebugP_log(" - Invalid parameters\r\n");
                    }
                    break;
                }
                status = endat3_set_expected_tx_frame_count(gAppEndat3Handle[CONFIG_ENDAT3_0], 1);
                if(status != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: endat3_set_expected_tx_frame_count() failed with error code: %d", status);
                    if(status == ENDAT3_ERR_INVALID_INPUT)
                    {
                        DebugP_log(" - Invalid parameters\r\n");
                    }
                    break;
                }
            }
            else if(cmd_type == ENDAT3_CMD_TYPE_BACKGROUND)
            {
                cmd = ENDAT3_REQ_DATA0; /* Sendlist 0 needs to be selected for Background communication commands*/
                DebugP_log("\r\n Select background request operation:");
                DebugP_log("\r\n 1: No Operation (NOP)");
                DebugP_log("\r\n 2: Read from encoder memory");
                DebugP_log("\r\n 3: Write to encoder memory");
                DebugP_log("\r\n 4: Reconfigure parameters");
                DebugP_log("\r\n 5: Authentication");
                DebugP_log("\r\n 6: Set protection");
                DebugP_log("\r\n 7: Set password");
                DebugP_log("\r\n 8: Locate function");

                DebugP_scanf("%d", &op_code);

                switch(op_code)
                {
                    case 1: /* NOP */
                        op_code = ENDAT3_BGREQ_NOP;
                        break;
                    case 2: /* READ */
                        op_code = ENDAT3_BGREQ_READ;
                        DebugP_log("\r\n Enter number of words to read:");
                        DebugP_scanf("%d", &words);
                        data = words;
                        DebugP_log("\r\n Enter address (MSB byte):");
                        DebugP_scanf("%x", &addr_msb);
                        DebugP_log("\r\n Enter address (LSB 2 bytes):");
                        DebugP_scanf("%x", &addr_lsb);
                        break;
                    case 3: /* WRITE */
                        op_code = ENDAT3_BGREQ_WRITE;
                        DebugP_log("\r\n Enter address (MSB byte):");
                        DebugP_scanf("%x", &addr_msb);
                        DebugP_log("\r\n Enter address (LSB 2 bytes):");
                        DebugP_scanf("%x", &addr_lsb);
                        DebugP_log("\r\n Enter data word to write:");
                        DebugP_scanf("%x", &data);
                        break;
                    case 4: /* RECONFIGURE */
                        op_code = ENDAT3_BGREQ_RECONFIGURE;
                        break;
                    case 5: /* AUTH */
                        op_code = ENDAT3_BGREQ_AUTH;
                        DebugP_log("\r\n Enter user level (0-255):");
                        DebugP_scanf("%d", &user_level);
                        addr_msb = user_level;
                        DebugP_log("\r\n Enter password (hex, up to 32 bits):");
                        DebugP_scanf("%x", &password);
                        addr_lsb = (password >> ENDAT3_PASSWORD_HIGH_SHIFT) & ENDAT3_PASSWORD_LOW_MASK;
                        data = password & ENDAT3_PASSWORD_LOW_MASK;
                        break;
                    case 6: /* PROTECT */
                        op_code = ENDAT3_BGREQ_PROTECT;
                        DebugP_log("\r\n PROTECT - Controls the protection of memory areas");
                        DebugP_log("\r\n Enter address (MSB byte):");
                        DebugP_scanf("%x", &addr_msb);
                        DebugP_log("\r\n Enter address (LSB 2 bytes):");
                        DebugP_scanf("%x", &addr_lsb);

                        DebugP_log("\r\n Enter protection mode:");
                        DebugP_log("\r\n 1: QUERY (0x%02X) - Query the current access levels", ENDAT3_PROTECT_QUERY);
                        DebugP_log("\r\n 2: SET_READ (0x%02X) - Set the access level for read-accesses", ENDAT3_PROTECT_SET_READ);
                        DebugP_log("\r\n 3: SET_WRITE (0x%02X) - Set the access level for write-accesses", ENDAT3_PROTECT_SET_WRITE);
                        DebugP_log("\r\n (Other values will be rejected with an error)");
                        DebugP_scanf("%u", &mode_input);

                        if(mode_input < 1 || mode_input > 3)
                        {
                            DebugP_log("\r\n Invalid mode value. Defaulting to QUERY (1)");
                            mode = ENDAT3_PROTECT_QUERY;
                        }
                        else
                        {
                            mode = (uint8_t)mode_input;
                        }

                        DebugP_log("\r\n Enter access level:");
                        DebugP_log("\r\n 0: USER - Default level, active after power-on or RESET");
                        DebugP_log("\r\n 1: OEM2 - For independent after-market users (machine manufacturers)");
                        DebugP_log("\r\n 2: OEM1 - For OEMs (motor manufacturers)");
                        DebugP_log("\r\n 3: MANUFACTURER - For encoder manufacturer access only");
                        DebugP_scanf("%u", &acclevel_input);

                        if(acclevel_input > 3)
                        {
                            DebugP_log("\r\n Invalid access level. Defaulting to USER (0)");
                            acclevel = 0;
                        }
                        else
                        {
                            acclevel = (uint8_t)acclevel_input;
                        }

                        data = ((uint32_t)mode << 24) | ((uint32_t)acclevel << 16);
                        break;
                    case 7: /* SETPASS */
                        op_code = ENDAT3_BGREQ_SETPASS;
                        DebugP_log("\r\n Enter user level (0-255):");
                        DebugP_scanf("%d", &user_level);
                        addr_msb = user_level;
                        DebugP_log("\r\n Enter new password (hex, up to 32 bits):");
                        DebugP_scanf("%x", &password);
                        addr_lsb = (password >> ENDAT3_PASSWORD_HIGH_SHIFT) & ENDAT3_PASSWORD_LOW_MASK;
                        data = password & ENDAT3_PASSWORD_LOW_MASK;
                        break;
                    case 8: /* LOCATE */
                        op_code = ENDAT3_BGREQ_LOCATE;
                        DebugP_log("\r\n Enter control value:");
                        DebugP_scanf("%x", &data);
                        break;
                    default:
                        DebugP_log("\r\n Invalid selection, using NOP");
                        op_code = ENDAT3_BGREQ_NOP;
                        break;
                }

                /* Use APIs to set operation codes */
                status = endat3_set_foreground_op_code(gAppEndat3Handle[CONFIG_ENDAT3_0], 0);
                if(status != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: endat3_set_foreground_op_code() failed with error code: %d", status);
                    if(status == ENDAT3_ERR_INVALID_INPUT)
                    {
                        DebugP_log(" - Invalid parameters\r\n");
                    }
                    break;
                }
                status = endat3_set_background_op_code(gAppEndat3Handle[CONFIG_ENDAT3_0], op_code);
                if(status != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: endat3_set_background_op_code() failed with error code: %d", status);
                    if(status == ENDAT3_ERR_INVALID_INPUT)
                    {
                        DebugP_log(" - Invalid parameters\r\n");
                    }
                    break;
                }

                /* Get LPH status and state */
                if(endat3_get_lph_state(gAppEndat3Handle[CONFIG_ENDAT3_0], &lph_state) != ENDAT3_SUCCESS)
                {
                    DebugP_log("\r\nERROR: Failed to get LPH state\r\n");
                    break;
                }

                switch(lph_state)
                {
                    case LPH_STATUS_IDLE: /* 0 */
                        /* Cycle stage: 1->2->3->0 */
                        {
                            bg_cmd_params.index = 0;
                            bg_cmd_params.frame_cnt = 4;
                            bg_cmd_params.op_code = op_code;
                            bg_cmd_params.addr_msb = addr_msb;
                            bg_cmd_params.addr_lsb = addr_lsb;
                            bg_cmd_params.data = data;
                            if(op_code == ENDAT3_BGREQ_PROTECT)
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, &ret_mode, &ret_acclevel_desc);
                                /* Display mode information with access level descriptions */
                                switch(ret_mode)
                                {
                                    case ENDAT3_PROTECT_QUERY:
                                        DebugP_log("\r\n PROTECT Mode: QUERY (0x%02X) - Querying current access levels", ENDAT3_PROTECT_QUERY);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_READ:
                                        DebugP_log("\r\n PROTECT Mode: SET_READ (0x%02X) - Setting read access level", ENDAT3_PROTECT_SET_READ);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_WRITE:
                                        DebugP_log("\r\n PROTECT Mode: SET_WRITE (0x%02X) - Setting write access level", ENDAT3_PROTECT_SET_WRITE);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    default:
                                        DebugP_log("\r\n PROTECT Mode: Unknown (0x%02X) - This will be rejected by the encoder", ret_mode);
                                        break;
                                }
                            }
                            else
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, NULL, NULL);
                            }
                        }
                        break;
                    case LPH_STATUS_RX_START: /* 1 */
                        /* Cycle stage: 2->0->1->2->3->0 */
                        /* Set background data */
                        {
                            if(endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, 0) != ENDAT3_SUCCESS)
                            {
                                DebugP_log("\r\nERROR: Failed to set background data\r\n");
                                break;
                            }
                            if(endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 1, 0) != ENDAT3_SUCCESS)
                            {
                                DebugP_log("\r\nERROR: Failed to set background data\r\n");
                                break;
                            }
                            bg_cmd_params.index = 2;
                            bg_cmd_params.frame_cnt = 6;
                            bg_cmd_params.op_code = op_code;
                            bg_cmd_params.addr_msb = addr_msb;
                            bg_cmd_params.addr_lsb = addr_lsb;
                            bg_cmd_params.data = data;
                            if(op_code == ENDAT3_BGREQ_PROTECT)
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, &ret_mode, &ret_acclevel_desc);

                                /* Display mode information with access level descriptions */
                                switch(ret_mode)
                                {
                                    case ENDAT3_PROTECT_QUERY:
                                        DebugP_log("\r\n PROTECT Mode: QUERY (0x%02X) - Querying current access levels", ENDAT3_PROTECT_QUERY);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_READ:
                                        DebugP_log("\r\n PROTECT Mode: SET_READ (0x%02X) - Setting read access level", ENDAT3_PROTECT_SET_READ);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_WRITE:
                                        DebugP_log("\r\n PROTECT Mode: SET_WRITE (0x%02X) - Setting write access level", ENDAT3_PROTECT_SET_WRITE);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    default:
                                        DebugP_log("\r\n PROTECT Mode: Unknown (0x%02X) - This will be rejected by the encoder", ret_mode);
                                        break;
                                }
                            }
                            else
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, NULL, NULL);
                            }
                        }
                        break;
                    case LPH_STATUS_RX_LAST: /* 2 */
                        /* Cycle stage: 0->1->2->3->0 */
                        /* Set background data */
                        {
                            if(endat3_set_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, 0) != ENDAT3_SUCCESS)
                            {
                                DebugP_log("\r\nERROR: Failed to set background data\r\n");
                                break;
                            }
                            bg_cmd_params.index = 1;
                            bg_cmd_params.frame_cnt = 5;
                            bg_cmd_params.op_code = op_code;
                            bg_cmd_params.addr_msb = addr_msb;
                            bg_cmd_params.addr_lsb = addr_lsb;
                            bg_cmd_params.data = data;
                            if(op_code == ENDAT3_BGREQ_PROTECT)
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, &ret_mode, &ret_acclevel_desc);
                                /* Display mode information with access level descriptions */
                                switch(ret_mode)
                                {
                                    case ENDAT3_PROTECT_QUERY:
                                        DebugP_log("\r\n PROTECT Mode: QUERY (0x%02X) - Querying current access levels", ENDAT3_PROTECT_QUERY);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_READ:
                                        DebugP_log("\r\n PROTECT Mode: SET_READ (0x%02X) - Setting read access level", ENDAT3_PROTECT_SET_READ);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    case ENDAT3_PROTECT_SET_WRITE:
                                        DebugP_log("\r\n PROTECT Mode: SET_WRITE (0x%02X) - Setting write access level", ENDAT3_PROTECT_SET_WRITE);
                                        DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                                        DebugP_log("\r\n - Access Level: %s", ret_acclevel_desc);
                                        break;
                                    default:
                                        DebugP_log("\r\n PROTECT Mode: Unknown (0x%02X) - This will be rejected by the encoder", ret_mode);
                                        break;
                                }
                            }
                            else
                            {
                                endat3_handle_background_command_request(gAppEndat3Handle[CONFIG_ENDAT3_0], &bg_cmd_params, NULL, NULL);
                            }
                        }
                        break;
                    case LPH_STATUS_BUSY: /* 3 */
                        /* Communication Busy */
                        DebugP_log("\r\n Communication Busy - Try again later");
                        break;
                    default:
                        /* Invalid LPH status */
                        DebugP_log("\r\n Wrong LPH_STATUS value: %d captured", lph_state);
                        break;
                }
            }
            else
            {
                DebugP_log("\r\n Invalid command\r\b");
                continue;
            }

            /* Get expected TX frame count */
            if(endat3_get_expected_tx_frame_count(gAppEndat3Handle[CONFIG_ENDAT3_0], &expected_frames) != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\n Error: Failed to get expected TX frame count");
                break;
            }
            status = endat3_send_command(gAppEndat3Handle[CONFIG_ENDAT3_0], cmd, expected_frames);
            if(status != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_send_command() failed with error code: %d", status);
                if(status == ENDAT3_ERR_INVALID_INPUT)
                {
                    DebugP_log(" - Invalid parameters\r\n");
                }
                break;
            }
            status = endat3_set_busy(gAppEndat3Handle[CONFIG_ENDAT3_0], ENCODER_BUSY);
            if(status != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\nERROR: endat3_set_busy() failed with error code: %d - Invalid handle or interface\r\n", status);
                break;
            }
            while(endat3_is_busy(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_busy) == ENDAT3_SUCCESS && is_busy);
            status = endat3_receive_response(gAppEndat3Handle[CONFIG_ENDAT3_0]);

            if(status == ENDAT3_ERR_SAMPLING_ERROR)
            {
                DebugP_log("\r\n Sampling ERROR");
            }

            if(status == ENDAT3_SUCCESS) /* Successful response */
            {
                /* Check for errors */
                endat3_display_error_status(gAppEndat3Handle[CONFIG_ENDAT3_0]);

                /* Display information based on the specific command */
                switch(cmd)
                {
                    case ENDAT3_REQ_DATA0:
                    case ENDAT3_REQ_DATA1:
                    case ENDAT3_REQ_DATA2:
                    case ENDAT3_REQ_DATA3:
                    case ENDAT3_REQ_DATA4:
                    case ENDAT3_REQ_DATA5:
                    case ENDAT3_REQ_DATA6:
                    case ENDAT3_REQ_DATA7:
                        DebugP_log("\r\n Send list %d activated", cmd);
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            endat3_display_position_info(gAppEndat3Handle[CONFIG_ENDAT3_0]);
                        }
                        break;
                    case ENDAT3_REQ_DATA:
                        DebugP_log("\r\n DATA command executed");
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            endat3_display_position_info(gAppEndat3Handle[CONFIG_ENDAT3_0]);
                        }
                        break;
                    case ENDAT3_REQ_DATANOP:
                        DebugP_log("\r\n DATANOP command executed (no BGD)");
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            endat3_display_position_info(gAppEndat3Handle[CONFIG_ENDAT3_0]);
                        }
                        break;
                    case ENDAT3_REQ_ECHO:
                        /* Display echo response and propagation time */
                        DebugP_log("\r\n Echo Response:");
                        /* For ECHO, HPF data is specifically marked as invalid (HPFV=0) */

                        /* Get HPF data */
                        if(endat3_get_hpf_data(gAppEndat3Handle[CONFIG_ENDAT3_0], echo_data) == HPF_DATA_SIZE)
                        {
                            DebugP_log("\r\n - Echo Code: 0x%04X", echo_data[0]);
                            DebugP_log("\r\n - Echo Req Data: 0x%x%x", echo_data[5], echo_data[4]);
                        }
                        else
                        {
                            DebugP_log("\r\n - ERROR: Failed to get echo data\r\n");
                        }
                        break;
                    case ENDAT3_REQ_HELLO:
                        /* Display hello response */
                        DebugP_log("\r\n EnDat3 Hello Response:");
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            DebugP_log("\r\n - Successfully switched to EnDat3 mode, now all commands can be executed");
                        }
                        else
                        {
                            DebugP_log("\r\n - Switch to EnDat3 mode in progress");
                            DebugP_log("\r\n (May take up to 300ms to complete)");
                        }
                        break;
                    case ENDAT3_REQ_RESET:
                        DebugP_log("\r\n Encoder Reset Response:");
                        /* Get background data */
                        reset_type = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &reset_data) == ENDAT3_SUCCESS)
                        {
                            reset_type = (uint16_t)reset_data;
                        }
                        if(reset_type == ENDAT3_RESET_HARD)
                        {
                            DebugP_log("\r\n - Hard Reset initiated (0xBBBB)");
                            DebugP_log("\r\n - Expect device to restart (up to 300ms)");
                        }
                        else
                        {
                            DebugP_log("\r\n - Custom Reset Type: 0x%04X", reset_type);
                        }
                        ClockP_usleep(DELAY_302_MILLISEC);
                        /* Release start trigger */
                        if(endat3_release_start_trigger(gAppEndat3Handle[CONFIG_ENDAT3_0]) != ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\n| ERROR: Failed to release start trigger\r\n");
                        }
                        break;
                    case ENDAT3_REQ_CLEAR:
                        DebugP_log("\r\n State Reset Response:");
                        /* Get background data */
                        clear_flags = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &clear_data) == ENDAT3_SUCCESS)
                        {
                            clear_flags = (uint16_t)clear_data;
                        }
                        if(clear_flags & ENDAT3_CLEAR_F)
                        {
                            DebugP_log("\r\n - Errors (F) reset requested");
                        }
                        if(clear_flags & ENDAT3_CLEAR_W)
                        {
                            DebugP_log("\r\n - Warnings (W) reset requested");
                        }
                        if(clear_flags & ENDAT3_CLEAR_REF)
                        {
                            DebugP_log("\r\n - Absolute value (REF) clear requested");
                        }
                        break;
                    case ENDAT3_REQ_RATE:
                        DebugP_log("\r\n Data Rate Configuration Response:");
                        /* Get background data */
                        rate_type = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &rate_data) == ENDAT3_SUCCESS)
                        {
                            rate_type = (uint16_t)rate_data;
                        }
                        if(rate_type == ENDAT3_RATE_12_5MBPS)
                        {
                            DebugP_log("\r\n - Switching to 12.5 Mbps requested");
                        }
                        else if(rate_type == ENDAT3_RATE_25MBPS)
                        {
                            DebugP_log("\r\n - Switching to 25 Mbps requested");
                        }
                        else
                        {
                            DebugP_log("\r\n - Custom rate type: 0x%04X", rate_type);
                        }
                        DebugP_log("\r\n - Note: Rate switching occurs after 2ms of no communication");
                        DebugP_log("\r\n and may take up to 300ms to complete");
                        ClockP_usleep(DELAY_302_MILLISEC);
                        break;
                    case ENDAT3_REQ_FORCE:
                        DebugP_log("\r\n Forced Dynamic Sampling Response:");
                        DebugP_log("\r\n - Check Safety Frame for IgF1 and IgF2 bits");
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            endat3_display_position_info(gAppEndat3Handle[CONFIG_ENDAT3_0]);
                        }
                        break;
                    case ENDAT3_REQ_BUSBC:
                        DebugP_log("\r\n Bus Broadcast Command Response:");
                        /* Get background data */
                        busbc_data = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &busbc_data) == ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\n - Broadcast Address: 0x%04X", (uint16_t)busbc_data);
                        }
                        break;
                    case ENDAT3_REQ_BUSP2P:
                        DebugP_log("\r\n Bus Point-to-Point Command Response:");
                        /* Get background data */
                        busp2p_data = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &busp2p_data) == ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\n - P2P Address: 0x%04X", (uint16_t)busp2p_data);
                        }
                        break;
                    case ENDAT3_REQ_BUSINIT:
                        DebugP_log("\r\n Bus Initialization Response:");
                        /* Get background data */
                        businit_type = 0;
                        if(endat3_get_bg_data(gAppEndat3Handle[CONFIG_ENDAT3_0], 0, &businit_data) == ENDAT3_SUCCESS)
                        {
                            businit_type = (uint16_t)businit_data;
                        }
                        if(businit_type == ENDAT3_BUSINIT_RESET_ADDR)
                        {
                            DebugP_log("\r\n - Bus address reset to 0x00 (0x8282)");
                        }
                        else
                        {
                            DebugP_log("\r\n - Custom BUSINIT type: 0x%04X", businit_type);
                        }
                        break;
                    default:
                        /* Generic response display */
                        DebugP_log("\r\n Command 0x%02X Response:", cmd);
                        /* Check if HPF data is valid */
                        if(endat3_is_hpf_data_valid(gAppEndat3Handle[CONFIG_ENDAT3_0], &is_valid) == ENDAT3_SUCCESS && is_valid)
                        {
                            endat3_display_position_info(gAppEndat3Handle[CONFIG_ENDAT3_0]);
                        }
                        break;
                }

                /* Get LPH status */
                if((cmd_type == 1) &&
                    (endat3_get_lph_status(gAppEndat3Handle[CONFIG_ENDAT3_0], &lph_status) == ENDAT3_SUCCESS) &&
                    (lph_status != 0))
                {
                    DebugP_log("\r\n LPH STATUS ERROR");
                }
            }
            else
            {
                DebugP_log("\r\n status=%d, Error during communication", status);
                if(status == ENDAT3_ERR_SAMPLING_ERROR)
                {
                    DebugP_log("\r\n Sampling ERROR");
                }
                else
                {
                    DebugP_log("\r\n CRC ERROR");
                }
            }
        }
    }

    /* ========================================================================== */
    /* STEP 7: Cleanup and graceful shutdown                                      */
    /* ========================================================================== */

deinit:
    /* Close drivers */
    endat3_deinit(gAppEndat3Handle[CONFIG_ENDAT3_0]);
    Board_driversClose();
    Drivers_close();
}
