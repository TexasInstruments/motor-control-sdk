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

#if defined(SOC_AM243X) || defined(SOC_AM64X)
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

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_TAMAGAWA1_MODE == TAMAGAWA_MODE_SINGLE_CHANNEL_SINGLE_PRU)
/* Single channel mode firmware */
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_single_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_single_channel_pru0_bin.h>
#endif
#endif

#if (CONFIG_TAMAGAWA1_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
/* Multi-channel mode firmware */
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
#include <tamagawa_receiver_multi_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_multi_channel_pru0_bin.h>
#endif
#endif
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TASK_STACK_SIZE                         (4096)
#define TASK_PRIORITY                           (6)

#define TAMAGAWA_POSITION_LOOP_STOP             (0)
#define TAMAGAWA_POSITION_LOOP_START            (1)

#define TAMAGAWA_PERIODIC_MODE_LOG_SLEEP_US     (100)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* PRU-ICSS Driver Handle */
PRUICSS_Handle gPruIcssXHandle = NULL;

/* Tamagawa Driver Handle */
tamagawa_handle gAppTamagawaHandle[CONFIG_TAMAGAWA_NUM_INSTANCES] = {NULL};

/* Tamagawa Periodic Interface Struct Instance */
tamagawa_periodic_interface gTamagawaPeriodicInterface = {0};

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
static void tamagawa_get_fw_version(void);
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
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;

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

    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

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
    const uint32_t *pruFirmware = NULL;
    uint32_t pruFirmwareSize = 0;
    uint8_t pru_id = CONFIG_TAMAGAWA0_PRUICSS_PRU_ID;

#if CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1
    pruFirmware = TamagawaFirmwarePru1_0;
    pruFirmwareSize = sizeof(TamagawaFirmwarePru1_0);
#else
    pruFirmware = TamagawaFirmwarePru0_0;
    pruFirmwareSize = sizeof(TamagawaFirmwarePru0_0);
#endif

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA0_PRUICSS_SLICE), 0,
                                  (uint32_t *)pruFirmware, pruFirmwareSize);
    DebugP_assert(0 != u_status);

    /* Reset PRU core */
    status = PRUICSS_resetCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enable PRU core to run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);


#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1
    pruFirmware = TamagawaFirmwarePru1_0;
    pruFirmwareSize = sizeof(TamagawaFirmwarePru1_0);
#else
    pruFirmware = TamagawaFirmwarePru0_0;
    pruFirmwareSize = sizeof(TamagawaFirmwarePru0_0);
#endif

    pru_id = CONFIG_TAMAGAWA1_PRUICSS_PRU_ID;

    /* Disable PRU core */
    status = PRUICSS_disableCore(gPruIcssXHandle, pru_id);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Load firmware to PRU instruction RAM */
    u_status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA1_PRUICSS_SLICE), 0,
                                  (uint32_t *)pruFirmware, pruFirmwareSize);
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
    /* NULL check on handle */
    if(handle == NULL)
    {
        DebugP_log("\r\n ERROR: NULL handle in tamagawa_display_result\n");
        return;
    }

    priv = tamagawa_get_priv(handle);

    /* Prints the position value returned by the encoder for a particular command ID */
    switch(cmd)
    {
        case DATA_ID_7:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_8:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_C:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_0:
            /* Data readout: data in one revolution */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABM: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nENID: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tENID: 0x%x\tABM: 0x%x\tALMC: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.almc, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, priv->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
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
        return SystemP_SUCCESS;
    }
    else
    {
        DebugP_log("\r\n CRC Failure \n");
        return SystemP_FAILURE;
    }
}

static int32_t tamagawa_get_command(uint8_t *adf, uint8_t *edf)
{
    int32_t cmd;
    uint32_t val;
    /* Check to make sure that the command issued is correct */
    if(DebugP_scanf("%d\n", &cmd) < 0)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* If the command is 9, start periodic trigger with DATA ID as 0*/
    if(cmd == PERIODIC_TRIGGER_CMD)
    {
        DebugP_log("\r| Enter IEP reset cycle count (must be greater than Tamagawa cycle time including timeout period, in IEP cycles):");
        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }

        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles): ");
        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.periodic_trigger_count[CONFIG_TAMAGAWA0]) < 0 )
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }

        if(gTamagawaPeriodicInterface.periodic_trigger_count[CONFIG_TAMAGAWA0] > gTamagawaPeriodicInterface.iep_reset_count)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)

        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles) for second slice: ");
        if(DebugP_scanf("%u\n", &gTamagawaPeriodicInterface.periodic_trigger_count[CONFIG_TAMAGAWA1]) < 0 )
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }

        if(gTamagawaPeriodicInterface.periodic_trigger_count[CONFIG_TAMAGAWA1] > gTamagawaPeriodicInterface.iep_reset_count)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return SystemP_FAILURE;
        }
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
        uint8_t ch = 0;
        for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
        {
            if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->total_channels == 1)
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) : ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) for ch %d : ", ch);
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
                if(tamagawa_update_adf(gAppTamagawaHandle[CONFIG_TAMAGAWA0], val, ch) != SystemP_SUCCESS)
                {
                    /* If EEPROM address update fails, command cannot proceed with correct address.
                     * Mark command as invalid to prevent execution with wrong address. */
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: tamagawa_update_adf failed\n|\n");
                    break;
                }
            }
        }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
        {
            if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->total_channels == 1)
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) for second slice: ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM address (hex value) for ch %d in second slice: ", ch);
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
                if(tamagawa_update_adf(gAppTamagawaHandle[CONFIG_TAMAGAWA1], val, ch) != SystemP_SUCCESS)
                {
                    /* If EEPROM address update fails, command cannot proceed with correct address.
                     * Mark command as invalid to prevent execution with wrong address. */
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: tamagawa_update_adf failed for second slice\n|\n");
                    break;
                }
            }
        }
#endif
    }
    /* In case of EEPROM Write, take input for Address field for different channels selected*/
    if(cmd == DATA_ID_6)
    {
        uint8_t ch = 0;
        for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
        {
            if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->total_channels == 1)
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) : ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) for ch %d : ", ch);
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
                if(tamagawa_update_edf(gAppTamagawaHandle[CONFIG_TAMAGAWA0], val, ch) != SystemP_SUCCESS)
                {
                    /* If EEPROM data update fails, write command cannot proceed with correct data.
                     * Mark command as invalid to prevent execution with wrong data. */
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: tamagawa_update_edf failed\n|\n");
                    break;
                }
            }
        }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
        {
            if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->total_channels == 1)
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) for second slice: ");
                }
                else
                {
                    DebugP_log("\r\n| Enter EEPROM data (hex value) for ch %d in second slice: ", ch);
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
                if(tamagawa_update_edf(gAppTamagawaHandle[CONFIG_TAMAGAWA1], val, ch) != SystemP_SUCCESS)
                {
                    /* If EEPROM data update fails, write command cannot proceed with correct data.
                     * Mark command as invalid to prevent execution with wrong data. */
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: tamagawa_update_edf failed for second slice\n|\n");
                    break;
                }
            }
        }
#endif
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
    DebugP_log("\r\n| 9 : Start periodic continuous mode                                           |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|\n|\n");
    DebugP_log("\r\n| enter value: ");
}

static void tamagawa_get_fw_version(void)
{
    uint32_t version;
    /* Prints the firmware version, depending on Single or Multi-channel configuration */
#if (CONFIG_TAMAGAWA0_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif
    DebugP_log("\r\nTamagawa firmware \t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
#if (CONFIG_TAMAGAWA1_PRUICSS_SLICE == 1)
    version = *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    version = *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif
    DebugP_log("\r\nTamagawa firmware (second slice)\t: %x.%x.%x (%s)\n\n", (version >> 24) & 0x7F, (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
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
    /* Any function call failure will lead to exit of tamagawa_process_periodic_command function */
    if(tamagawa_config_periodic_trigger(handle[CONFIG_TAMAGAWA0]) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_config_periodic_trigger failed\r\n|\r\n|\n");
        return;
    }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    if(tamagawa_config_periodic_trigger(handle[CONFIG_TAMAGAWA1]) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_config_periodic_trigger failed for second slice\r\n|\r\n|\n");
        return;
    }
#endif

    if(tamagawa_loop_task_create() != SystemP_SUCCESS)
    {
        return;
    }

    gTamagawaPeriodicInterface.handle[CONFIG_TAMAGAWA0] = handle[CONFIG_TAMAGAWA0];
    /* Assuming that periodic_trigger_count[CONFIG_TAMAGAWA0] and iep_reset_count values are set in tamagawa_get_command() */
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    gTamagawaPeriodicInterface.handle[CONFIG_TAMAGAWA1] = handle[CONFIG_TAMAGAWA1];
    /* Assuming that periodic_trigger_count[CONFIG_TAMAGAWA1] and iep_reset_count values are set in tamagawa_get_command() */
#endif

    if(tamagawa_config_periodic_mode(&gTamagawaPeriodicInterface) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_config_periodic_mode failed\r\n|\r\n|\n");
        return;
    }

    gTamagawaPositionLoopStatus = TAMAGAWA_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| Press Enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");

    if(tamagawa_update_data_id(handle[CONFIG_TAMAGAWA0], process_dataid_cmd) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_update_data_id failed\r\n|\r\n|\n");
        return;
    }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    if(tamagawa_update_data_id(handle[CONFIG_TAMAGAWA1], process_dataid_cmd) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_update_data_id failed for second slice\r\n|\r\n|\n");
        return;
    }
#endif

    /* In case of EEPROM commands, calculate the CRC for the different channels selected */
    if((process_dataid_cmd == DATA_ID_6) || (process_dataid_cmd == DATA_ID_D))
    {
        uint8_t ch = 0;
        for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
        {
            if(handle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
            {
                if(tamagawa_update_crc(handle[CONFIG_TAMAGAWA0], process_dataid_cmd, ch) != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: tamagawa_update_crc failed for channel %d\r\n|\r\n|\n", ch);
                    return;
                }
            }
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
        {
            if(handle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
            {
                if(tamagawa_update_crc(handle[CONFIG_TAMAGAWA1], process_dataid_cmd, ch) != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: tamagawa_update_crc failed for channel %d in second slice \r\n|\r\n|\n", ch);
                    return;
                }
            }
        }
#endif
    }

    if(tamagawa_command_process(handle[CONFIG_TAMAGAWA0], process_dataid_cmd) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_command_process failed\r\n|\r\n|\n");
        return;
    }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)

    if(tamagawa_command_process(handle[CONFIG_TAMAGAWA1], process_dataid_cmd) != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: tamagawa_command_process failed for second slice\r\n|\r\n|\n");
        return;
    }

#endif

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
            if(handle[CONFIG_TAMAGAWA0]->attrs->total_channels > 1)
            {
                DebugP_log("\r\n Multi-channel mode is enabled\n\n");

                uint8_t ch;
                for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
                {
                    if(handle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
                    {
                        if(tamagawa_multi_channel_set_cur(handle[CONFIG_TAMAGAWA0], ch) != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: tamagawa_multi_channel_set_cur failed for channel %d\r\n|\r\n|\n", ch);
                            return;
                        }
                        DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                        if(tamagawa_handle_rx(handle[CONFIG_TAMAGAWA0], process_dataid_cmd) != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: tamagawa_handle_rx failed for channel %d\r\n|\r\n|\n", ch);
                            return;
                        }
                    }
                }
            }
            else
            {
                DebugP_log("\r\n Single-channel mode is enabled\n\n");
                if(tamagawa_handle_rx(handle[CONFIG_TAMAGAWA0], process_dataid_cmd) != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: tamagawa_handle_rx failed\r\n|\r\n|\n");
                    return;
                }
            }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
            if(handle[CONFIG_TAMAGAWA1]->attrs->total_channels > 1)
            {
                DebugP_log("\r\n Multi-channel mode is enabled in second slice\n\n");

                uint8_t ch;
                for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
                {
                    if(handle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
                    {
                        if(tamagawa_multi_channel_set_cur(handle[CONFIG_TAMAGAWA1], ch) != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: tamagawa_multi_channel_set_cur failed for channel %d in second slice\r\n|\r\n|\n", ch);
                            return;
                        }
                        DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                        if(tamagawa_handle_rx(handle[CONFIG_TAMAGAWA1], process_dataid_cmd) != SystemP_SUCCESS)
                        {
                            DebugP_log("\r| ERROR: tamagawa_handle_rx failed for channel %d in second slice\r\n|\r\n|\n", ch);
                            return;
                        }
                    }
                }
            }
            else
            {
                DebugP_log("\r\n Single-channel mode is enabled in second slice\n\n");
                if(tamagawa_handle_rx(handle[CONFIG_TAMAGAWA1], process_dataid_cmd) != SystemP_SUCCESS)
                {
                    DebugP_log("\r| ERROR: tamagawa_handle_rx failed for second slice\r\n|\r\n|\n");
                    return;
                }
            }
#endif
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

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
/* Set pin high for Enabling ch0 in booster pack */
#if (CONFIG_TAMAGAWA1_BOOSTER_PACK && CONFIG_TAMAGAWA1_CHANNEL0_ENABLED)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
/* Set pin high for Enabling ch2 in booster pack */
#if (CONFIG_TAMAGAWA1_BOOSTER_PACK && CONFIG_TAMAGAWA1_CHANNEL2_ENABLED)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
#endif

    /* ========================================================================== */
    /* STEP 2: Initialize PRU-ICSS and Tamagawa driver                           */
    /* ========================================================================== */

    /* Initialize PRU-ICSS instance, initialize DRAM, and disable PRU cores */
    tamagawa_pruicss_init();
    DebugP_log("\r\n\nTamagawa PRU-ICSS init done\n\n");

    /* Initialize Tamagawa parameters with defaults and set PRU-ICSS handle */
    tamagawa_params_init(&tamagawa_params);
    tamagawa_params.pruicss_handle = gPruIcssXHandle;
    /* Default delay values are used:
     *   - cmd_wait_delay_us = 10us (delay for command wait loop)
     *   - max_wait_loop_count = 5 (number of wait loop iterations in tamagawa_command_wait())
     * If needed, these can be modified before calling tamagawa_init():
     *   tamagawa_params.cmd_wait_delay_us = <custom_value>;
     *   tamagawa_params.max_wait_loop_count = <custom_value>;
     */

    /* Initialize Tamagawa driver instance
     * This calls: tamagawa_config_clr_cfg0(), tamagawa_config_channel(), tamagawa_set_baudrate(),
     * and tamagawa_config_host_trigger() */
    gAppTamagawaHandle[CONFIG_TAMAGAWA0] = tamagawa_init(CONFIG_TAMAGAWA0, &tamagawa_params);
    if(gAppTamagawaHandle[CONFIG_TAMAGAWA0] == NULL)
    {
        DebugP_log("\r\nERROR: Tamagawa initialization failed\n");
        return;
    }

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* Initialize Tamagawa parameters with defaults for second instance */
    tamagawa_params_init(&tamagawa_params);
    tamagawa_params.pruicss_handle = gPruIcssXHandle;
    /* Default delay values are used:
     *   - cmd_wait_delay_us = 10us (delay for command wait loop)
     *   - max_wait_loop_count = 5 (number of wait loop iterations in tamagawa_command_wait())
     * If needed, these can be modified before calling tamagawa_init():
     *   tamagawa_params.cmd_wait_delay_us = <custom_value>;
     *   tamagawa_params.max_wait_loop_count = <custom_value>;
     */

    /* Initialize Tamagawa driver instance for second slice */
    gAppTamagawaHandle[CONFIG_TAMAGAWA1] = tamagawa_init(CONFIG_TAMAGAWA1, &tamagawa_params);
    if(gAppTamagawaHandle[CONFIG_TAMAGAWA1] == NULL)
    {
        DebugP_log("\r\nERROR: Tamagawa initialization failed for second slice\n");
        return;
    }
#endif

    /* Get and display Tamagawa firmware version from PRU firmware image */
    tamagawa_get_fw_version();

    /* Display HW instances used, operation mode and enabled channels */
    DebugP_log("\r\n PRU-ICSS instance: %u, PRU-ICSS slice number: %u\n", CONFIG_TAMAGAWA0_PRUICSS_INSTANCE, CONFIG_TAMAGAWA0_PRUICSS_SLICE);
    if(CONFIG_TAMAGAWA0_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
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
                gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << 0) ? "0" : "",
                gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << 1) ? "1" : "",
                gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << 2) ? "2" : "");

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* Display HW instances used, operation mode and enabled channels for second slice */
    DebugP_log("\r\n For second slice, PRU-ICSS instance: %u, PRU-ICSS slice number: %u\n", CONFIG_TAMAGAWA1_PRUICSS_INSTANCE, CONFIG_TAMAGAWA1_PRUICSS_SLICE);
    if(CONFIG_TAMAGAWA1_MODE == TAMAGAWA_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        /* Multi-channel single PRU mode: Multiple channels handled by one PRU core */
        DebugP_log("\r\nTamagawa Multi channel, Single PRU Demo application is running in second slice......\n");
    }
    else
    {
        /* Single channel single PRU mode: One channel on one PRU core */
        DebugP_log("\r\nTamagawa Single channel, Single PRU Demo application is running in second slice......\n");
    }

    DebugP_log("\r\nChannel(s) selected in second slice: %s %s %s \n\n\n",
                gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << 0) ? "0" : "",
                gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << 1) ? "1" : "",
                gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << 2) ? "2" : "");
#endif

    /* ========================================================================== */
    /* STEP 3: Load and run PRU firmware                                         */
    /* ========================================================================== */
    /* Load PRU firmware image to instruction RAM and enable PRU cores */
    tamagawa_pruicss_load_run_fw();

    DebugP_log("\r\nTamagawa PRU-ICSS firmware loaded and running\n\n\n");

    /* ========================================================================== */
    /* STEP 4: Interactive menu loop for encoder operations                      */
    /* ========================================================================== */
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
        if(cmd == PERIODIC_TRIGGER_CMD)
        {
            DebugP_log("\r\n\n Switching to periodic trigger mode");

            /* Process continuous position readout using DATA_ID_0 */
            tamagawa_process_periodic_command(gAppTamagawaHandle, DATA_ID_0);

            /* Switch back to host trigger mode for menu-driven operation */
            DebugP_log("\r\n\n Switching to host trigger mode");
            if(tamagawa_config_host_trigger(gAppTamagawaHandle[CONFIG_TAMAGAWA0]) != SystemP_SUCCESS)
            {
                /* NOTE: If this fails, driver may remain in periodic mode causing subsequent
                 * host-triggered commands to fail. */
                DebugP_log("\r| ERROR: tamagawa_config_host_trigger failed\r\n|\r\n|\n");
            }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
            if(tamagawa_config_host_trigger(gAppTamagawaHandle[CONFIG_TAMAGAWA1]) != SystemP_SUCCESS)
            {
                /* NOTE: If this fails, driver may remain in periodic mode causing subsequent
                 * host-triggered commands to fail. */
                DebugP_log("\r| ERROR: tamagawa_config_host_trigger failed for second slice\r\n|\r\n|\n");
            }
#endif
        }

        /* Validate DATA_ID command is within valid range */
        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        /* Update DATA_ID for encoder command (configures which data to retrieve from encoder) */
        if(tamagawa_update_data_id(gAppTamagawaHandle[CONFIG_TAMAGAWA0], cmd) != SystemP_SUCCESS)
        {
            /* NOTE: If DATA_ID update fails, subsequent command processing would use wrong/stale DATA_ID.
             * Skip this command iteration to prevent incorrect encoder operation. */
            DebugP_log("\r\n| ERROR: tamagawa_update_data_id failed\n");
            continue;
        }

        /* For EEPROM read/write commands (DATA_ID_6, DATA_ID_D), calculate and update CRC.
         * CRC is required to ensure data integrity when accessing encoder EEPROM. */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint8_t ch = 0;
            uint8_t crc_failed = 0;
            for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
                {
                    if(tamagawa_update_crc(gAppTamagawaHandle[CONFIG_TAMAGAWA0], cmd, ch) != SystemP_SUCCESS)
                    {
                        /* NOTE: If CRC calculation fails, EEPROM command would proceed without proper CRC verification.
                         * Mark as failed to skip command execution and prevent data corruption. */
                        DebugP_log("\r\n| ERROR: tamagawa_update_crc failed for channel %d\n", ch);
                        crc_failed = 1;
                    }
                }
            }
            if(crc_failed)
            {
                continue;
            }

        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        if(tamagawa_update_data_id(gAppTamagawaHandle[CONFIG_TAMAGAWA1], cmd) != SystemP_SUCCESS)
        {
            /* NOTE: If DATA_ID update fails, subsequent command processing would use wrong/stale DATA_ID.
             * Skip this command iteration to prevent incorrect encoder operation. */
            DebugP_log("\r\n| ERROR: tamagawa_update_data_id failed for second slice\n");
            continue;
        }

        /* In case of EEPROM commands, calculate the CRC for the different channels selected */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint8_t ch = 0;
            uint8_t crc_failed = 0;
            for(ch = 0 ; ch < TAMAGAWA_MAX_CHANNELS ; ch++)
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
                {
                    if(tamagawa_update_crc(gAppTamagawaHandle[CONFIG_TAMAGAWA1], cmd, ch) != SystemP_SUCCESS)
                    {
                        /* NOTE: If CRC calculation fails, EEPROM command would proceed without proper CRC verification.
                         * Mark as failed to skip command execution and prevent data corruption. */
                        DebugP_log("\r\n| ERROR: tamagawa_update_crc failed for channel %d in second slice\n", ch);
                        crc_failed = 1;
                    }
                }
            }
            if(crc_failed)
            {
                continue;
            }

        }
#endif

        /* Execute Tamagawa command transaction with encoder.
         * This triggers PRU firmware to send command to encoder and wait for response. */
        if(tamagawa_command_process(gAppTamagawaHandle[CONFIG_TAMAGAWA0], cmd) != SystemP_SUCCESS)
        {
            /* NOTE: If command processing fails, no valid data is available to parse.
             * Skip data processing for this slice to prevent parsing invalid/stale data. */
            DebugP_log("\r\n| ERROR: tamagawa_command_process failed\n");
            continue;
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        if(tamagawa_command_process(gAppTamagawaHandle[CONFIG_TAMAGAWA1], cmd) != SystemP_SUCCESS)
        {
            /* NOTE: If command processing fails, no valid data is available to parse.
             * Skip data processing for this slice to prevent parsing invalid/stale data. */
            DebugP_log("\r\n| ERROR: tamagawa_command_process failed for second slice\n");
            continue;
        }
#endif

        /* Parse and display received encoder data based on channel configuration */
        if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->total_channels > 1)
        {
            /* Multi-channel mode: Process each enabled channel separately */
            DebugP_log("\r\n Multi-channel mode is enabled\n\n");
            uint8_t ch;
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA0]->attrs->channel_mask & (1 << ch))
                {
                    if(tamagawa_multi_channel_set_cur(gAppTamagawaHandle[CONFIG_TAMAGAWA0], ch) != SystemP_SUCCESS)
                    {
                        /* NOTE: If channel selection fails, subsequent RX parsing would use wrong channel.
                         * Skip this channel to prevent incorrect data interpretation. */
                        DebugP_log("\r\n| ERROR: tamagawa_multi_channel_set_cur failed for channel %d\n", ch);
                        continue;
                    }
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    if(tamagawa_handle_rx(gAppTamagawaHandle[CONFIG_TAMAGAWA0], cmd) != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed for channel %d\n", ch);
                    }
                }
            }
        }
        else
        {
            /* Single-channel mode: Process single channel data directly */
            DebugP_log("\r\n Single-channel mode is enabled\n\n");
            if(tamagawa_handle_rx(gAppTamagawaHandle[CONFIG_TAMAGAWA0], cmd) != SystemP_SUCCESS)
            {
                DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed\n");
            }
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->total_channels > 1)
        {
            DebugP_log("\r\n Multi-channel mode is enabled in second slice\n\n");
            uint8_t ch;
            for(ch = 0; ch < TAMAGAWA_MAX_CHANNELS; ch++)
            {
                if(gAppTamagawaHandle[CONFIG_TAMAGAWA1]->attrs->channel_mask & (1 << ch))
                {
                    if(tamagawa_multi_channel_set_cur(gAppTamagawaHandle[CONFIG_TAMAGAWA1], ch) != SystemP_SUCCESS)
                    {
                        /* NOTE: If channel selection fails, subsequent RX parsing would use wrong channel.
                         * Skip this channel to prevent incorrect data interpretation. */
                        DebugP_log("\r\n| ERROR: tamagawa_multi_channel_set_cur failed for channel %d in second slice\n", ch);
                        continue;
                    }
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    if(tamagawa_handle_rx(gAppTamagawaHandle[CONFIG_TAMAGAWA1], cmd) != SystemP_SUCCESS)
                    {
                        DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed for channel %d in second slice\n", ch);
                    }
                }
            }

        }
        else
        {
            DebugP_log("\r\n Single-channel mode is enabled in second slice\n\n");
            if(tamagawa_handle_rx(gAppTamagawaHandle[CONFIG_TAMAGAWA1], cmd) != SystemP_SUCCESS)
            {
                DebugP_log("\r\n| ERROR: tamagawa_handle_rx failed for second slice\n");
            }
        }
#endif
    }

    tamagawa_deinit(gAppTamagawaHandle[CONFIG_TAMAGAWA0]);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_deinit(gAppTamagawaHandle[CONFIG_TAMAGAWA1]);
#endif
    Board_driversClose();
    Drivers_close();
}
