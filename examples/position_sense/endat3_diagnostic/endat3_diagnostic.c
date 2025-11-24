/*
 * Copyright (C) 2025 Texas Instruments Incorporated
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

/* ========================================================================== */
/* Include Files                                                              */
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

#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#if (CONFIG_ENDAT3_0_PRUICSS_PRUx == 1)
#include <position_sense/endat3/firmware/single_channel/endat3_receiver_pru1_bin.h>
#else
#include <position_sense/endat3/firmware/single_channel/endat3_receiver_pru0_bin.h>
#endif

/* ========================================================================== */
/* Macros & Typedefs                                                          */
/* ========================================================================== */

#define ENDAT3_SUCCESSFUL_RESPONSE   1
#define ENDAT3_POSITION_LOOP_STOP    0
#define ENDAT3_POSITION_LOOP_START   1
#define POSITION_MASK_30BIT          0x3FFFFFFF
#define ANGLE_FULL_ROTATION          360.0f
#define ANGLE_BIT_RESOLUTION         30
#define DISPLAY_LINE_LENGTH          47
#define CONTINUOUS_MODE_DELAY_US     1000
#define PERIODIC_MODE_STARTUP_US     100000
#define PERIODIC_MODE_LOOP_DELAY_US  10000
#define DATANOP_FIXED_VALUE          0x0000
#define HELLO_FIXED_VALUE            0x2222
#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY (6)
#define DELAY_1_SECOND (1000000)
#define DELAY_302_MILLISEC (302000)
/* PRU-ICSS Defines */
#define PRU_CORE_CLK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define PRUICSS_PRUx CONFIG_ENDAT3_0_PRUICSS_PRUx
#define PRUICSS_TX_PRUx PRUICSS_PRUx + 4
#define PRUICSS_RTU_PRUx PRUICSS_PRUx + 2
/* Command Type Defines */
#define ENDAT3_CMD_TYPE_FOREGROUND (0)
#define ENDAT3_CMD_TYPE_BACKGROUND (1)
#define ENDAT3_CMD_TYPE_CONTINUOUS (2)
#define ENDAT3_CMD_TYPE_PERIODIC (3)

/* Global handles - gPruIcssXHandle is now extern from endat3_periodic_trigger.h */
endat3_Handle gEndat3HandleCh[ENDAT3_MAX_CHANNELS];

/* Add PRU memory pointers */
static void *gPru_cfg;

/* Task management for periodic mode */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;
static int32_t endat3_position_loop_status;

/* ========================================================================== */
/* Function Prototypes                                                        */
/* ========================================================================== */

void endat3_display_position_info(endat3_Handle priv);
void endat3_display_error_status(endat3_Handle priv);
int32_t endat3_pruicss_load_run_fw(void);
static void endat3_pruicss_init(void);
void endat3_continuous_position_fetch(endat3_Handle handle);
static void endat3_position_loop_decide_termination(void *args);
static int32_t endat3_loop_task_create(void);
static void endat3_process_periodic_command(void);

/* ========================================================================== */
/* Function Definitions                                                       */
/* ========================================================================== */

/**
 * \brief Display position information from encoder
 *
 * \param priv Pointer to private data structure
 */
void endat3_display_position_info(endat3_Handle priv)
{
    DebugP_log("\r\n Position Information:");

    /* Get HPF status */
    uint8_t hpf_status;
    if (endat3_getHpfStatus(priv, &hpf_status) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n Error: Failed to get HPF status");
        return;
    }
    DebugP_log("\r\n HPF Status: 0x%02X", hpf_status);

    if (endat3_hasHpfError(priv))
    {
        DebugP_log("\r\n - Error detected (F bit set)");
    }

    if (endat3_hasHpfWarning(priv))
    {
        DebugP_log("\r\n - Warning detected (W bit set)");
    }

    /* Check if HPF data is valid */
    if (!endat3_isHpfDataValid(priv))
    {
        DebugP_log("\r\n - HPF data invalid (HPFV bit not set)");
        return; /* Don't display invalid data */
    }

    /* Check if absolute value is available */
    if (!endat3_hasAbsoluteValue(priv))
    {
        DebugP_log("\r\n - Absolute value not available (RM bit not set)");
    }

    /* Get HPF data */
    uint8_t hpf_data[6];
    if (endat3_getHpfData(priv, hpf_data) == 6)
    {
        DebugP_log("\r\n Position Data: 0x%02X%02X%02X%02X",
                   hpf_data[3], hpf_data[2], hpf_data[1], hpf_data[0]);
    }

    /* Get HPF CRC */
    uint8_t hpf_crc;
    if (endat3_getHpfCrc(priv, &hpf_crc) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n CRC: 0x%02X", hpf_crc);
    }

    DebugP_log("\r\n Additional Information:");

    /* Get LPH status */
    uint8_t lph_status;
    if (endat3_getLphStatus(priv, &lph_status) == ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n LPH Status (communication Status): 0x%x", lph_status);
    }

    /* Get LPF status and data */
    uint8_t lpf_status = endat3_getLpfStatus(priv, 0);
    DebugP_log("\r\n LPF Status: 0x%x", lpf_status);

    uint8_t lpf_data[6];
    if (endat3_getLpfData(priv, 0, lpf_data) == 6)
    {
        DebugP_log("\r\n LPF Data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                   lpf_data[5], lpf_data[4], lpf_data[3],
                   lpf_data[2], lpf_data[1], lpf_data[0]);
    }

    /* Get LPF CRC */
    uint8_t lpf_crc = endat3_getLpfCrc(priv, 0);
    DebugP_log("\r\n LPF CRC: 0x%02X", lpf_crc);

    /* Display additional position data if available */
    if (hpf_data[4] || hpf_data[5])
    {
        DebugP_log("\r\n Extended Position: 0x%02X%02X",
                   hpf_data[5], hpf_data[4]);
    }
}

/**
 * \brief Display error status information
 *
 * \param priv Pointer to private data structure
 */
void endat3_display_error_status(endat3_Handle priv)
{
    DebugP_log("\r\n HPF and LPH Status:");

    /* Use APIs to get status */
    uint8_t hpf_status, lph_status;
    if (endat3_getHpfStatus(priv, &hpf_status) != ENDAT3_SUCCESS ||
        endat3_getLphStatus(priv, &lph_status) != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n Error: Failed to get status");
        return;
    }

    DebugP_log("\r\n HPF Status: 0x%02X", hpf_status);
    DebugP_log("\r\n LPH Status: 0x%02X", lph_status);

    /* Extract error code */
    endat3_ErrorCode_t error_code = endat3_getErrorCode(priv);

    /* Check for warning */
    if (endat3_hasHpfWarning(priv))
    {
        DebugP_log("\r\n Warning detected (W bit set in HPF status)");
    }

    /* Display error information if error code is present */
    if (error_code != ENDAT3_ERR_UNKNOWN)
    {
        DebugP_log("\r\n Error Code: 0x%04X", error_code);

        /* Get description and recommended action */
        const char* description = endat3_getErrorDescription(error_code);
        const char* action = endat3_getErrorAction(error_code);

        /* Display error information */
        DebugP_log("\r\n Description: %s", description);
        DebugP_log("\r\n Recommended action: %s", action);

        /* Special handling for FGERR_RECONFIGURE */
        if (error_code == ENDAT3_FGERR_RECONFIGURE)
        {
            DebugP_log("\r\n NOTE: If this error occurs without a RECONFIGURE command or active");
            DebugP_log("\r\n background BUSY state, the encoder may be defective.");
        }

        /* Special handling for access denied errors */
        if (error_code == ENDAT3_BGERR_USAGE_ACCESS_DENIED)
        {
            DebugP_log("\r\n Access protection error: Current user level is insufficient.");
            DebugP_log("\r\n You may need to authenticate with the AUTH command using a higher");
            DebugP_log("\r\n user level (OEM2, OEM1, or MANUFACTURER) and the correct password.");
        }
    }
}

static void endat3_pruicss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
#ifdef CONFIG_ENDAT3_0_G_MUX_EN
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));

    /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_PRUx));

    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
}

int32_t endat3_pruicss_load_run_fw()
{
    int32_t status = ENDAT3_SUCCESS;
    uint32_t size;

    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(ENDAT3_SUCCESS == status);
#if (CONFIG_ENDAT3_0_PRUICSS_PRUx == 0)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                               0, (uint32_t *) EnDat3FirmwarePru0_0,
                               sizeof(EnDat3FirmwarePru0_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                                   0, (uint32_t *) EnDat3FirmwarePru1_0,
                                   sizeof(EnDat3FirmwarePru1_0));
#endif
    DebugP_assert(size);

    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(ENDAT3_SUCCESS == status);

    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(ENDAT3_SUCCESS == status);

    return status;
}

/**
 * \brief Continuously fetch position data from encoder
 *
 * \param handle EnDAT3 handle for the encoder channel
 */
void endat3_continuous_position_fetch(endat3_Handle handle)
{
    int32_t status = 0;
    uint32_t position = 0;
    float angle = 0.0f;
    uint16_t cmd = ENDAT3_REQ_DATA0;

    /* Create task to monitor stop condition */
    if (endat3_loop_task_create() != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n ERROR: Task creation failed\r\n");
        endat3_position_loop_status = ENDAT3_POSITION_LOOP_STOP;
        return;
    }

    endat3_position_loop_status = ENDAT3_POSITION_LOOP_START;

    DebugP_log("\r\n Starting continuous position fetch from encoder...\r\n");
    DebugP_log("\r\n Press Enter to stop continuous mode\r\n");

    while (1)
    {
        /* Check if user requested to stop */
        if (endat3_position_loop_status == ENDAT3_POSITION_LOOP_STOP)
        {
            DebugP_log("\r\n Continuous mode stopped, returning to menu\r\n");
            return;
        }

        /* Set expected TX frames count */
        if (endat3_setExpectedTxFrameCount(handle, 1) != ENDAT3_SUCCESS)
        {
            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
            DebugP_log("\r\nERROR: endat3_setExpectedTxFrameCount() failed with error code: %d", error);
            if (error == ENDAT3_ERR_INVALID_HANDLE)
            {
                DebugP_log(" - Invalid handle or interface\r\n");
            }
            break;
        }

        /* Send DATA0 command to encoder */
        if (endat3_send_command(handle, cmd, 1) != ENDAT3_SUCCESS)
        {
            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
            DebugP_log("\r\nERROR: endat3_send_command() failed with error code: %d", error);
            if (error == ENDAT3_ERR_INVALID_HANDLE)
            {
                DebugP_log(" - Invalid handle or interface\r\n");
            }
            else if (error == ENDAT3_ERR_INVALID_PARAM)
            {
                DebugP_log(" - Invalid parameters\r\n");
            }
            break;
        }

        /* Set busy flag and wait for transmission to complete */
        if (endat3_setBusy(handle, ENCODER_BUSY) != ENDAT3_SUCCESS)
        {
            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
            DebugP_log("\r\nERROR: endat3_setBusy() failed with error code: %d", error);
            if (error == ENDAT3_ERR_INVALID_HANDLE)
            {
                DebugP_log(" - Invalid handle or interface\r\n");
            }
            break;
        }
        while (endat3_isBusy(handle));

        /* Receive response from encoder */
        status = endat3_receive_response(handle);

        /* Process response if successful */
        if (status == ENDAT3_SUCCESSFUL_RESPONSE)
        {
            /* Get RX buffer */
            const uint8_t* rx_buffer = endat3_getRxBuffer(handle);
            if (rx_buffer == NULL)
            {
                int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                DebugP_log("\r\nERROR: endat3_getRxBuffer() failed with error code: %d", error);
                if (error == ENDAT3_ERR_INVALID_HANDLE)
                {
                    DebugP_log(" - Invalid handle or interface\r\n");
                }
                break;
            }
            position = (rx_buffer[0]) |
                      ((rx_buffer[1]) << 8) |
                      ((rx_buffer[2]) << 16) |
                      ((rx_buffer[3]) << 24);

            /* Mask upper 2 bits to get 30-bit position value */
            position &= POSITION_MASK_30BIT;

            /* Convert position to angle in degrees (30-bit resolution = 2^30 = 1073741824) */
            angle = (position * ANGLE_FULL_ROTATION) / (1 << ANGLE_BIT_RESOLUTION);

            /* Print new values */
            DebugP_log("\r Position: 0x%x, Angle: %f degrees", position, angle);
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

    while (1)
    {
        DebugP_scanf("%c", &c);
        endat3_position_loop_status = ENDAT3_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

/**
 * \brief Create task for monitoring periodic mode termination
 *
 * \return ENDAT3_SUCCESS on success, ENDAT3_ERR_RX_FAIL on failure
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

    if (status != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\nTask creation failed\n");
    }

    return status;
}

/**
 * \brief Process periodic trigger command
 *
 * This function implements the complete periodic trigger mode workflow
 */
static void endat3_process_periodic_command(void)
{
    int32_t status;
    uint32_t cmp0_val, cmp3_val;
    uint16_t cmd = ENDAT3_REQ_DATA0;

    /* Get IEP timer configuration from user */
    DebugP_log("\r\n| Enter IEP reset cycle count (must be greater than EnDat3 cycle time including timeout period, in IEP cycles): ");
    if (DebugP_scanf("%u", &cmp0_val) < 0)
    {
        DebugP_log("\r\n| ERROR: invalid value\n|\n|\n|\n");
        return;
    }
    DebugP_log("\r\n| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
    if (DebugP_scanf("%u", &cmp3_val) < 0)
    {
        DebugP_log("\r\n| ERROR: invalid value\n|\n|\n|\n");
        return;
    }

    /* Validate CMP3 <= CMP0 */
    if (cmp3_val > cmp0_val)
    {
        DebugP_log("\r\n| ERROR: Trigger time (CMP3=%u) must be <= Reset cycle (CMP0=%u)\r\n", cmp3_val, cmp0_val);
        DebugP_log("\r\n| Please enter valid values where CMP3 <= CMP0\r\n|\r\n|\n");
        return;
    }

    /* Create task to monitor stop condition */
    if (endat3_loop_task_create() != ENDAT3_SUCCESS)
    {
        DebugP_log("\r\n| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        endat3_position_loop_status = ENDAT3_POSITION_LOOP_STOP;
        return;
    }

    /* Set up periodic interface structure */
    struct endat3_periodic_interface endat3_periodic_interface;
    endat3_periodic_interface.pruss_cfg = gPru_cfg;
    endat3_periodic_interface.pruss_iep = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);
    endat3_periodic_interface.pruss_dmem = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru0DramBase);
    endat3_periodic_interface.load_share = 0;
    endat3_periodic_interface.cmp3 = cmp3_val;
    endat3_periodic_interface.cmp0 = cmp0_val;

    /* Configure and start periodic mode */
    status = endat3_config_periodic_mode(&endat3_periodic_interface, gPruIcssXHandle);
    if (status != 1)
    {
        DebugP_log("\r\n| ERROR: Failed to configure periodic mode\r\n|\r\n|\n");
        return;
    }

    DebugP_log("\r\n| Periodic mode configured successfully");
    DebugP_log("\r\n| CMP0 (Reset Cycle): %u IEP cycles", cmp0_val);
    DebugP_log("\r\n| CMP3 (Trigger Time): %u IEP cycles", cmp3_val);
    DebugP_log("\r\n| IRQ Count at start: %u", gPruEnDat3IrqCnt0);

    /* Set firmware to periodic trigger mode (opmode = 0) */
    DebugP_log("\r\n| Setting firmware to periodic trigger mode...");
    int32_t opmode_result = endat3_setOperatingMode(gEndat3HandleCh[0], 0);  /* 0 = periodic mode */
    if (opmode_result != ENDAT3_SUCCESS)
    {
        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
        DebugP_log("\r\n| ERROR: endat3_setOperatingMode() failed with error code: %d", error);
        if (error == ENDAT3_ERR_INVALID_HANDLE)
        {
            DebugP_log(" - Invalid handle or interface\r\n|\r\n|\n");
        }
        else
        {
            DebugP_log(" - Failed to set operating mode to periodic\r\n|\r\n|\n");
        }
        return;
    }
    /* Configure frame information before releasing trigger */
    DebugP_log("\r\n| Configuring frame information for periodic mode...");
    endat3_setExpectedTxFrameCount(gEndat3HandleCh[0], 1);
    endat3_send_command(gEndat3HandleCh[0], cmd, 1);
    endat3_setBusy(gEndat3HandleCh[0], ENCODER_BUSY);
    /* Release start trigger for periodic mode operation */
    endat3_releaseStartTrigger(gEndat3HandleCh[0]);

    /* Wait for periodic mode to start */
    ClockP_usleep(PERIODIC_MODE_STARTUP_US);

    /* Verify operating mode was set correctly */
    int32_t current_opmode = endat3_getOperatingMode(gEndat3HandleCh[0]);
    DebugP_log("\r\n| Current operating mode: %d (0=periodic, 1=host)", current_opmode);

    endat3_position_loop_status = ENDAT3_POSITION_LOOP_START;

    DebugP_log("\r\n|\n\r\n| Firmware will now trigger automatically on IEP CMP3 events");
    DebugP_log("\r\n| Waiting for periodic triggers...");
    DebugP_log("\r\n| press enter to stop the continuous mode\r\n|\r\n|");

    /* Main periodic loop - continuously display position data */
    while (1)
    {
        if (endat3_position_loop_status == ENDAT3_POSITION_LOOP_STOP)
        {
            /* Stop periodic mode and restore host trigger */
            DebugP_log("\r\n| Stopping periodic mode...");

            /* Stop IEP timer */
            endat3_stop_periodic_continuous_mode(&endat3_periodic_interface);

            /* Set firmware back to host trigger mode */
            DebugP_log("\r\n| Setting firmware to host trigger mode...");

            status = endat3_setOperatingMode(gEndat3HandleCh[0], 1);  /* 1 = host trigger mode */
            if (status != ENDAT3_SUCCESS)
            {
                int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                DebugP_log("\r\nERROR: endat3_setOperatingMode() failed with error code: %d", error);
                if (error == ENDAT3_ERR_INVALID_HANDLE)
                {
                    DebugP_log(" - Invalid handle or interface\r\n");
                }
                return;
            }

            /* Wait for mode switch to complete */
            ClockP_usleep(100000);

            DebugP_log("\r\n| Periodic mode stopped, returning to host trigger mode\r\n");
            return;
        }
        else
        {
            /* In periodic mode, continuously try to receive data */
            while (endat3_isBusy(gEndat3HandleCh[0]));
            status = endat3_receive_response(gEndat3HandleCh[0]);

            /* Process response if successful */
            if (status == ENDAT3_SUCCESSFUL_RESPONSE)
            {
                /* Display position info if valid */
                if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                {
                    /* Extract position data */
                    const uint8_t* rx_buffer = endat3_getRxBuffer(gEndat3HandleCh[0]);
                    if (rx_buffer == NULL)
                    {
                        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                        DebugP_log("\r\nERROR: endat3_getRxBuffer() failed with error code: %d", error);
                        if (error == ENDAT3_ERR_INVALID_HANDLE)
                        {
                            DebugP_log(" - Invalid handle or interface\r\n");
                        }
                        break;
                    }
                    uint32_t position = (rx_buffer[0]) |
                                      ((rx_buffer[1]) << 8) |
                                      ((rx_buffer[2]) << 16) |
                                      ((rx_buffer[3]) << 24);
                    
                    /* Mask upper 2 bits to get 30-bit position value */
                    position &= POSITION_MASK_30BIT;

                    /* Convert position to angle in degrees */
                    float angle = (position * ANGLE_FULL_ROTATION) / (1 << ANGLE_BIT_RESOLUTION);

                    /* Print new values */
                    DebugP_log("\r Position: 0x%x, Angle: %f degrees", position, angle);
                }
            }

            /* Small delay to prevent CPU hogging */
            ClockP_usleep(PERIODIC_MODE_LOOP_DELAY_US);
        }
    }
}

/**
 * \brief Main diagnostic function
 *
 * \param args Pointer to private data structure
 */
void endat3_diagnostic_main(void *args)
{
    int32_t i;
    uint16_t cmd;
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
    uint8_t channel_mask = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Initialize PRUICSS */
    /*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_ENDAT3_0_BOOSTER_PACK && CONFIG_ENDAT3_0_CHANNEL0)
    GPIO_setDirMode(ENC0_EN_BASE_ADDR, ENC0_EN_PIN, ENC0_EN_DIR);
    GPIO_pinWriteHigh(ENC0_EN_BASE_ADDR, ENC0_EN_PIN);
#endif

    /*B17 pin High for Enabling ch2 in booster pack */
#if (CONFIG_ENDAT3_0_BOOSTER_PACK && CONFIG_ENDAT3_0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif

    /* Initialize EnDAT interface */
    endat3_pruicss_init();

    /* Set channel mask based on configured channel */
    channel_mask = 0;
#if (CONFIG_ENDAT3_0_CHANNEL0)
    channel_mask |= (1 << 0);
#endif
#if (CONFIG_ENDAT3_0_CHANNEL1)
    channel_mask |= (1 << 1);
#endif
#if (CONFIG_ENDAT3_0_CHANNEL2)
    channel_mask |= (1 << 2);
#endif

    gEndat3HandleCh[0] = endat3_open(gPruIcssXHandle, PRUICSS_PRUx, CONFIG_ENDAT3_0_MODE, CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ, channel_mask, CONFIG_ENDAT3_0_BAUD_RATE);
    if (gEndat3HandleCh[0] == NULL)
    {
        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
        DebugP_log("\r\nERROR: endat3_open() failed with error code: %d\n", error);
        switch (error)
        {
            case ENDAT3_ERR_CLOCK_CONFIG:
                DebugP_log("  Clock configuration failed - check PRU frequency and baud rate settings\n");
                break;
            case ENDAT3_ERR_INVALID_CORE:
                DebugP_log("  Invalid PRU core specified for the selected mode\n");
                break;
            case ENDAT3_ERR_DELAY_CONFIG:
                DebugP_log("  Delay cycle configuration failed - check handle and interface initialization\n");
                break;
            case ENDAT3_ERR_CHANNEL_CONFIG:
                DebugP_log("  Channel mask configuration failed - check channel mask settings\n");
                break;
            default:
                DebugP_log("  Unknown error occurred\n");
                break;
        }
        goto deinit;
    }

    gPru_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);

    /* Load and run firmware */
    i = endat3_pruicss_load_run_fw();

    if (i < 0)
    {
        DebugP_log("\r\nERROR: Endat3 initialization failed \n");
        DebugP_log("\r\ncheck whether encoder of selected frequency is connected and ensure proper connections\n");
        goto deinit;
    }

    /* Set firmware to host trigger mode by default */
    i = endat3_setOperatingMode(gEndat3HandleCh[0], 1);  /* 1 = host trigger mode */
    if (i != ENDAT3_SUCCESS)
    {
        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
        DebugP_log("\r\nERROR: endat3_setOperatingMode() failed with error code: %d", error);
        if (error == ENDAT3_ERR_INVALID_HANDLE)
        {
            DebugP_log(" - Invalid handle or interface\r\n");
        }
        goto deinit;
    }

    /* Clear any previous trigger state */
    i = endat3_clearStartTrigger(gEndat3HandleCh[0]);
    if (i != ENDAT3_SUCCESS)
    {
        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
        DebugP_log("\r\nERROR: endat3_clearStartTrigger() failed with error code: %d", error);
        if (error == ENDAT3_ERR_INVALID_HANDLE)
        {
            DebugP_log(" - Invalid handle or interface\r\n");
        }
        goto deinit;
    }
    /* It can therefore take up to 300 ms to respond to a HELLO command for initial startup */
    ClockP_usleep(DELAY_1_SECOND);

    /* Release start trigger to firmware for initial communication */
    i = endat3_releaseStartTrigger(gEndat3HandleCh[0]);
    if (i != ENDAT3_SUCCESS)
    {
        int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
        DebugP_log("\r\nERROR: endat3_releaseStartTrigger() failed with error code: %d", error);
        if (error == ENDAT3_ERR_INVALID_HANDLE)
        {
            DebugP_log(" - Invalid handle or interface\r\n");
        }
        goto deinit;
    }

    ClockP_usleep(DELAY_1_SECOND);

    while (1)
    {
        DebugP_log("\r\n ========================================");
        DebugP_log("\r\n Select command type:");
        DebugP_log("\r\n 0: Foreground Communication");
        DebugP_log("\r\n 1: Background Communication");
        DebugP_log("\r\n 2: Continuous Position Fetch");
        DebugP_log("\r\n 3: Periodic Trigger Mode");
        DebugP_log("\r\n ========================================");
        DebugP_scanf("%d", &cmd_type);

        if (cmd_type == ENDAT3_CMD_TYPE_PERIODIC)
        {
            /* Process periodic trigger mode with dedicated function */
            endat3_process_periodic_command();
        }
        else
        {
            if (cmd_type == ENDAT3_CMD_TYPE_CONTINUOUS)
            {
                /* Continuous position fetch mode */
                DebugP_log("\r\n Starting continuous position fetch mode...");
                endat3_continuous_position_fetch(gEndat3HandleCh[0]);
            }
            else if (cmd_type == ENDAT3_CMD_TYPE_FOREGROUND)
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
                switch (menu_option)
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
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, DATANOP_FIXED_VALUE) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d", error);
                            if (error == ENDAT3_ERR_INVALID_HANDLE)
                            {
                                DebugP_log(" - Invalid handle or interface\r\n");
                            }
                            break;
                        }
                        break;
                    case ENDAT3_MENU_RESET:
                        cmd = ENDAT3_REQ_RESET;
                        DebugP_log("\r\n Select reset type:");
                        DebugP_log("\r\n 1: Hard Reset (0xBBBB)");
                        DebugP_log("\r\n 2: Other (Enter custom value)");

                        DebugP_scanf("%d", &reset_type_choice);

                        if (reset_type_choice == 1)
                        {
                            req_data = ENDAT3_RESET_HARD;
                        } else
                        {
                            DebugP_log("\r\n Enter custom reset type (hex):");
                            DebugP_scanf("%x", &req_data);
                        }
                        /* Set background data */
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, req_data) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d - Invalid handle or interface\r\n", error);
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
                        if (clear_f)
                            clear_flags |= ENDAT3_CLEAR_F;

                        DebugP_log("\r\n Reset warning (W)? (0/1):");
                        DebugP_scanf("%d", &clear_w);
                        if (clear_w)
                            clear_flags |= ENDAT3_CLEAR_W;

                        DebugP_log("\r\n Clear absolute value (REF)? (0/1):");
                        DebugP_scanf("%d", &clear_ref);
                        if (clear_ref)
                            clear_flags |= ENDAT3_CLEAR_REF;

                        /* Set background data */
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, clear_flags) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d - Invalid handle or interface\r\n", error);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_ECHO:
                        cmd = ENDAT3_REQ_ECHO;
                        DebugP_log("\r\n Enter echo data (0x0-0xffff):");
                        DebugP_scanf("%x", &req_data);
                        /* Set background data */
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, req_data) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d - Invalid handle or interface\r\n", error);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_RATE:
                        cmd = ENDAT3_REQ_RATE;
                        DebugP_log("\r\n Select data transfer rate:");
                        DebugP_log("\r\n 1: 12.5 Mbps");

                        int32_t rate_choice;
                        DebugP_scanf("%d", &rate_choice);

                        if (rate_choice == 1)
                        {
                            req_data = ENDAT3_RATE_12_5MBPS;
                        }
                        else if (rate_choice == 2)
                        {
                            req_data = ENDAT3_RATE_25MBPS;
                        }
                        else
                        {
                            DebugP_log("\r\n Invalid selection, using 12.5 Mbps");
                            req_data = ENDAT3_RATE_12_5MBPS;
                        }
                        /* Set background data */
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, req_data) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d - Invalid handle or interface\r\n", error);
                            break;
                        }
                        break;
                    case ENDAT3_MENU_HELLO:
                        cmd = ENDAT3_REQ_HELLO;
                        DebugP_log("\r\n HELLO uses fixed value 0x%04X", HELLO_FIXED_VALUE);
                        /* Set background data */
                        if (endat3_setBgData(gEndat3HandleCh[0], 0, HELLO_FIXED_VALUE) != ENDAT3_SUCCESS)
                        {
                            int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                            DebugP_log("\r\nERROR: endat3_setBgData() failed with error code: %d - Invalid handle or interface\r\n", error);
                            break;
                        }
                        break;
                    default:
                        DebugP_log("\r\n Invalid option, using DATA0");
                        cmd = ENDAT3_REQ_DATA0;
                        break;
                }

                /* Use APIs to set operation codes */
                if (endat3_setBackgroundOpCode(gEndat3HandleCh[0], 0) != ENDAT3_SUCCESS)
                {
                    int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                    DebugP_log("\r\nERROR: endat3_setBackgroundOpCode() failed with error code: %d", error);
                    if (error == ENDAT3_ERR_INVALID_HANDLE)
                    {
                        DebugP_log(" - Invalid handle or interface\r\n");
                    }
                    break;
                }
                if (endat3_setForegroundOpCode(gEndat3HandleCh[0], cmd) != ENDAT3_SUCCESS)
                {
                    int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                    DebugP_log("\r\nERROR: endat3_setForegroundOpCode() failed with error code: %d", error);
                    if (error == ENDAT3_ERR_INVALID_HANDLE)
                    {
                        DebugP_log(" - Invalid handle or interface\r\n");
                    }
                    break;
                }
                if (endat3_setExpectedTxFrameCount(gEndat3HandleCh[0], 1) != ENDAT3_SUCCESS)
                {
                    int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                    DebugP_log("\r\nERROR: endat3_setExpectedTxFrameCount() failed with error code: %d", error);
                    if (error == ENDAT3_ERR_INVALID_HANDLE)
                    {
                        DebugP_log(" - Invalid handle or interface\r\n");
                    }
                    break;
                }
            }
            else
            {
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

                switch (op_code)
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

                        int32_t mode_input;
                        DebugP_log("\r\n Enter protection mode:");
                        DebugP_log("\r\n 1: QUERY (0x%02X) - Query the current access levels", ENDAT3_PROTECT_QUERY);
                        DebugP_log("\r\n 2: SET_READ (0x%02X) - Set the access level for read-accesses", ENDAT3_PROTECT_SET_READ);
                        DebugP_log("\r\n 3: SET_WRITE (0x%02X) - Set the access level for write-accesses", ENDAT3_PROTECT_SET_WRITE);
                        DebugP_log("\r\n (Other values will be rejected with an error)");
                        DebugP_scanf("%d", &mode_input);

                        if (mode_input < 1 || mode_input > 3)
                        {
                            DebugP_log("\r\n Invalid mode value. Defaulting to QUERY (1)");
                            mode = ENDAT3_PROTECT_QUERY;
                        }
                        else
                        {
                            mode = (uint8_t)mode_input;
                        }

                        int32_t acclevel_input;
                        DebugP_log("\r\n Enter access level:");
                        DebugP_log("\r\n 0: USER - Default level, active after power-on or RESET");
                        DebugP_log("\r\n 1: OEM2 - For independent after-market users (machine manufacturers)");
                        DebugP_log("\r\n 2: OEM1 - For OEMs (motor manufacturers)");
                        DebugP_log("\r\n 3: MANUFACTURER - For encoder manufacturer access only");
                        DebugP_scanf("%d", &acclevel_input);

                        if (acclevel_input < 0 || acclevel_input > 3)
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
                if (endat3_setForegroundOpCode(gEndat3HandleCh[0], 0) != ENDAT3_SUCCESS)
                {
                    int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                    DebugP_log("\r\nERROR: endat3_setForegroundOpCode() failed with error code: %d", error);
                    if (error == ENDAT3_ERR_INVALID_HANDLE)
                    {
                        DebugP_log(" - Invalid handle or interface\r\n");
                    }
                    break;
                }
                if (endat3_setBackgroundOpCode(gEndat3HandleCh[0], op_code) != ENDAT3_SUCCESS)
                {
                    int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                    DebugP_log("\r\nERROR: endat3_setBackgroundOpCode() failed with error code: %d", error);
                    if (error == ENDAT3_ERR_INVALID_HANDLE)
                    {
                        DebugP_log(" - Invalid handle or interface\r\n");
                    }
                    break;
                }

                /* Get LPH status and state */
                LPH_Status_t lph_state = endat3_getLphState(gEndat3HandleCh[0]);

                switch (lph_state)
                {
                    case LPH_STATUS_IDLE: /* 0 */
                        /* Cycle stage: 1->2->3->0 */
                        endat3_handle_background_command_request(gEndat3HandleCh[0],0, 4, op_code, addr_msb, addr_lsb, data);
                        break;
                    case LPH_STATUS_RX_START: /* 1 */
                        /* Cycle stage: 2->0->1->2->3->0 */
                        /* Set background data */
                        endat3_setBgData(gEndat3HandleCh[0], 0, 0);
                        endat3_setBgData(gEndat3HandleCh[0], 1, 0);
                        endat3_handle_background_command_request(gEndat3HandleCh[0],2, 6, op_code, addr_msb, addr_lsb, data);
                        break;
                    case LPH_STATUS_RX_LAST: /* 2 */
                        /* Cycle stage: 0->1->2->3->0 */
                        /* Set background data */
                        endat3_setBgData(gEndat3HandleCh[0], 0, 0);
                        endat3_handle_background_command_request(gEndat3HandleCh[0],1, 5, op_code, addr_msb, addr_lsb, data);
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

            /* Get expected TX frame count */
            uint32_t expected_frames;
            if (endat3_getExpectedTxFrameCount(gEndat3HandleCh[0], &expected_frames) != ENDAT3_SUCCESS)
            {
                DebugP_log("\r\n Error: Failed to get expected TX frame count");
                break;
            }
            if (endat3_send_command(gEndat3HandleCh[0], cmd, expected_frames) != ENDAT3_SUCCESS)
            {
                int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                DebugP_log("\r\nERROR: endat3_send_command() failed with error code: %d", error);
                if (error == ENDAT3_ERR_INVALID_HANDLE)
                {
                    DebugP_log(" - Invalid handle or interface\r\n");
                }
                else if (error == ENDAT3_ERR_INVALID_PARAM)
                {
                    DebugP_log(" - Invalid parameters\r\n");
                }
                break;
            }
            if (endat3_setBusy(gEndat3HandleCh[0],ENCODER_BUSY) != ENDAT3_SUCCESS)
            {
                int32_t error = endat3_getLastError(gEndat3HandleCh[0]);
                DebugP_log("\r\nERROR: endat3_setBusy() failed with error code: %d - Invalid handle or interface\r\n", error);
                break;
            }
            while (endat3_isBusy(gEndat3HandleCh[0]));
            status = endat3_receive_response(gEndat3HandleCh[0]);

            if (status == ENDAT3_ERR_SAMPLING_ERROR)
            {
                DebugP_log("\r\n Sampling ERROR");
            }

            if (status == ENDAT3_SUCCESSFUL_RESPONSE) /* Successful response */
            {
                /* Check for errors */
                endat3_display_error_status(gEndat3HandleCh[0]);

                /* Display information based on the specific command */
                switch (cmd)
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
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                        {
                            endat3_display_position_info(gEndat3HandleCh[0]);
                        }
                        break;
                    case ENDAT3_REQ_DATA:
                        DebugP_log("\r\n DATA command executed");
                        /* Check if HPF data is valid */
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                        {
                            endat3_display_position_info(gEndat3HandleCh[0]);
                        }
                        break;
                    case ENDAT3_REQ_DATANOP:
                        DebugP_log("\r\n DATANOP command executed (no BGD)");
                        /* Check if HPF data is valid */
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                        {
                            endat3_display_position_info(gEndat3HandleCh[0]);
                        }
                        break;
                    case ENDAT3_REQ_ECHO:
                        /* Display echo response and propagation time */
                        DebugP_log("\r\n Echo Response:");
                        /* For ECHO, HPF data is specifically marked as invalid (HPFV=0) */

                        /* Get HPF data */
                        uint8_t echo_data[6];
                        if (endat3_getHpfData(gEndat3HandleCh[0], echo_data) == 0)
                        {
                            DebugP_log("\r\n - Echo Code: 0x%04X", echo_data[0]);
                            DebugP_log("\r\n - Echo Req Data: 0x%x%x", echo_data[5], echo_data[4]);
                        }
                        break;
                    case ENDAT3_REQ_HELLO:
                        /* Display hello response */
                        DebugP_log("\r\n EnDat3 Hello Response:");
                        /* Check if HPF data is valid */
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
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
                        uint32_t reset_data;
                        uint16_t reset_type = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &reset_data) == ENDAT3_SUCCESS)
                        {
                            reset_type = (uint16_t)reset_data;
                        }
                        if (reset_type == ENDAT3_RESET_HARD)
                        {
                            DebugP_log("\r\n - Hard Reset initiated (0xBBBB)");
                            DebugP_log("\r\n - Expect device to restart (up to 300ms)");
                        }
                        else
                        {
                            DebugP_log("\r\n - Custom Reset Type: 0x%04X", reset_type);
                        }
                        ClockP_usleep(DELAY_302_MILLISEC);
                        break;
                    case ENDAT3_REQ_CLEAR:
                        DebugP_log("\r\n State Reset Response:");
                        /* Get background data */
                        uint32_t clear_data;
                        uint16_t clear_flags = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &clear_data) == ENDAT3_SUCCESS)
                        {
                            clear_flags = (uint16_t)clear_data;
                        }
                        if (clear_flags & ENDAT3_CLEAR_F)
                        {
                            DebugP_log("\r\n - Errors (F) reset requested");
                        }
                        if (clear_flags & ENDAT3_CLEAR_W)
                        {
                            DebugP_log("\r\n - Warnings (W) reset requested");
                        }
                        if (clear_flags & ENDAT3_CLEAR_REF)
                        {
                            DebugP_log("\r\n - Absolute value (REF) clear requested");
                        }
                        break;
                    case ENDAT3_REQ_RATE:
                        DebugP_log("\r\n Data Rate Configuration Response:");
                        /* Get background data */
                        uint32_t rate_data;
                        uint16_t rate_type = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &rate_data) == ENDAT3_SUCCESS)
                        {
                            rate_type = (uint16_t)rate_data;
                        }
                        if (rate_type == ENDAT3_RATE_12_5MBPS)
                        {
                            DebugP_log("\r\n - Switching to 12.5 Mbps requested");
                        }
                        else if (rate_type == ENDAT3_RATE_25MBPS)
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
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                        {
                            endat3_display_position_info(gEndat3HandleCh[0]);
                        }
                        break;
                    case ENDAT3_REQ_BUSBC:
                        DebugP_log("\r\n Bus Broadcast Command Response:");
                        /* Get background data */
                        uint32_t busbc_data = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &busbc_data) == ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\n - Broadcast Address: 0x%04X", (uint16_t)busbc_data);
                        }
                        break;
                    case ENDAT3_REQ_BUSP2P:
                        DebugP_log("\r\n Bus Point-to-Point Command Response:");
                        /* Get background data */
                        uint32_t busp2p_data = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &busp2p_data) == ENDAT3_SUCCESS)
                        {
                            DebugP_log("\r\n - P2P Address: 0x%04X", (uint16_t)busp2p_data);
                        }
                        break;
                    case ENDAT3_REQ_BUSINIT:
                        DebugP_log("\r\n Bus Initialization Response:");
                        /* Get background data */
                        uint32_t businit_data;
                        uint16_t businit_type = 0;
                        if (endat3_getBgData(gEndat3HandleCh[0], 0, &businit_data) == ENDAT3_SUCCESS)
                        {
                            businit_type = (uint16_t)businit_data;
                        }
                        if (businit_type == ENDAT3_BUSINIT_RESET_ADDR)
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
                        if (endat3_isHpfDataValid(gEndat3HandleCh[0]))
                        {
                            endat3_display_position_info(gEndat3HandleCh[0]);
                        }
                        break;
                }

                /* Get LPH status */
                uint8_t lph_status;
                if ((cmd_type == 1) &&
                    (endat3_getLphStatus(gEndat3HandleCh[0], &lph_status) == ENDAT3_SUCCESS) &&
                    (lph_status != 0))
                {
                    DebugP_log("\r\n LPH STATUS ERROR");
                }
            }
            else
            {
                DebugP_log("\r\n status=%d, Error during communication", status);
                if (status == ENDAT3_ERR_SAMPLING_ERROR)
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

deinit:
    Board_driversClose();
    Drivers_close();
}
