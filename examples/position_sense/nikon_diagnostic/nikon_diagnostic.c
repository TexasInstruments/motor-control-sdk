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

#define PRUICSS_SLICEx CONFIG_NIKON0_PRUICSS_PRUx

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (PRUICSS_SLICEx == 1)
#include  <nikon_receiver_multi_pru1_bin.h>
#else
#include  <nikon_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU )
#if (PRUICSS_SLICEx == 1)
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
#if (PRUICSS_SLICEx == 1)
#include  <nikon_receiver_pru1_bin.h>
#else
#include  <nikon_receiver_pru0_bin.h>
#endif
#endif
#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

#define NIKON_POSITION_LOOP_STOP            0
#define NIKON_POSITION_LOOP_START           1

#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define ICSS_PRU_UART_CLOCK CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ

/* Macro for 0.5 seconds delay - value in microseconds */
#define NIKON_POWER_UP_DELAY (0.5 * 1000 * 1000)

/** \brief Global Structure pointer holding Nikon handle */
struct nikon_priv *priv;

uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

PRUICSS_Handle gPruIcssXHandle;
TaskP_Object gTaskObject;

static uint32_t nikon_position_loop_status;
uint32_t totalchannels = 0;
uint32_t mask = 0;

static void nikon_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    uint32_t size;
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
#ifdef CONFIG_NIKON0_G_MUX_EN
    /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
    /* clear ICSS0 PRUx data RAM */
    size = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_SLICEx));
    DebugP_assert(size);
    if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_RTUPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
    /*
    * Set the constant table C28 for tx pru
    * configuring the constant table C28 to point to the TX counter
    * register (CNTR). The counter is needed in firmware for adding waits and time stemps.
    */
#if CONFIG_NIKON0_PRUICSSx == 1
#if PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA50);
#endif /* PRUICSS_SLICEx == 1 */
#else
#if PRUICSS_SLICEx == 1
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
#else
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x250);
#endif /* PRUICSS_SLICEx == 1 */
#endif /* CONFIG_NIKON0_PRUICSSx == 1 */

    }
    status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);

}

int32_t nikon_pruicss_load_run_fw(struct nikon_priv *priv, uint8_t mask)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t size;
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU) /*enable loadshare mode*/
#if(CONFIG_NIKON0_CHANNEL0)
    status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#if (PRUICSS_SLICEx == 1)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeRtuPru1_0,
                                                        sizeof(NikonFirmwareMultiMakeRtuPru1_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeRtuPru0_0,
                                                        sizeof(NikonFirmwareMultiMakeRtuPru0_0));
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_NIKON0_CHANNEL1)
    status=PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx );
    DebugP_assert(SystemP_SUCCESS == status);
#if (PRUICSS_SLICEx == 1)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) NikonFirmwareMultiMakePru1_0,
                                                      sizeof(NikonFirmwareMultiMakePru1_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) NikonFirmwareMultiMakePru0_0,
                                                      sizeof(NikonFirmwareMultiMakePru0_0));
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_NIKON0_CHANNEL2)
    status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#if (PRUICSS_SLICEx == 1)
    size = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeTxPru1_0,
                                                        sizeof(NikonFirmwareMultiMakeTxPru1_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeTxPru0_0,
                                                        sizeof(NikonFirmwareMultiMakeTxPru0_0));
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (PRUICSS_SLICEx == 1)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmwareMultiPru1_0,
                                sizeof(NikonFirmwareMultiPru1_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmwareMultiPru0_0,
                                sizeof(NikonFirmwareMultiPru0_0));
#endif
#else
#if (PRUICSS_SLICEx == 1)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmwarePru1_0,
                                sizeof(NikonFirmwarePru1_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_NIKON0_PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmwarePru0_0,
                                sizeof(NikonFirmwarePru0_0));
#endif
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, CONFIG_NIKON0_PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
    status = nikon_wait_for_encoder_detection(priv);
    return status;
}
void nikon_get_enc_data_len(struct nikon_priv *priv)
{
    uint32_t ch_num;
    uint32_t ch;
    uint32_t enc_num;
    uint32_t num_encoders;
    uint32_t single_turn_len[NUM_ENCODERS_MAX];
    uint32_t multi_turn_len[NUM_ENCODERS_MAX];
    for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
    {
        for(enc_num = 0; enc_num < NUM_ENCODERS_MAX; enc_num++)
        {
            single_turn_len[enc_num] = 0;
            multi_turn_len[enc_num] = 0;
        }

        ch = nikon_get_current_channel(priv, ch_num);
        DebugP_log("\r\nPlease enter encoder length connected to Channel %d:\n", ch);

        DebugP_log("\r\nPlease enter single turn length for encoder 1: ");
        DebugP_scanf("%u\n", &single_turn_len[0]);
        DebugP_log("\r\nPlease enter multi turn length for encoder 1 (0, if not a multi turn encoder): ");
        DebugP_scanf("%u\n", &multi_turn_len[0]);
        num_encoders = 1;

        for(enc_num = 1; enc_num < NUM_ENCODERS_MAX; enc_num++)
        {
            DebugP_log("\r\nPlease enter single turn length for encoder %d (0, if not connected): ", (enc_num + 1));
            DebugP_scanf("%u\n", &single_turn_len[enc_num]);

            if(single_turn_len[enc_num] == 0)
            {
                break;
            }
            DebugP_log("\r\nPlease enter multi turn length for encoder %d (0, if not a multi turn encoder): ", (enc_num + 1));
            DebugP_scanf("%u\n", &multi_turn_len[enc_num]);

            num_encoders++;
        }

        nikon_update_enc_len(priv, num_encoders, single_turn_len, multi_turn_len, ch);
    }
}

uint32_t nikon_get_fw_version(void)
{
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (PRUICSS_SLICEx == 1)
    return *((uint32_t *)NikonFirmwareMultiPru1_0 + 2);
#else
    return *((uint32_t *)NikonFirmwareMultiPru0_0 + 2);
#endif
#endif
#if (CONFIG_NIKON0_CHANNEL0) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#if (PRUICSS_SLICEx == 1)
    return *((uint32_t *)NikonFirmwareMultiMakeRtuPru1_0 + 2);
#else
    return *((uint32_t *)NikonFirmwareMultiMakeRtuPru0_0 + 2);
#endif
#endif
#if (CONFIG_NIKON0_CHANNEL1) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#if (PRUICSS_SLICEx == 1)
    return *((uint32_t *)NikonFirmwareMultiMakePru1_0 + 2);
#else
    return *((uint32_t *)NikonFirmwareMultiMakePru0_0 + 2);
#endif
#endif
#if (CONFIG_NIKON0_CHANNEL2) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#if (PRUICSS_SLICEx == 1)
    return *((uint32_t *)NikonFirmwareMultiMakeTxPru1_0 + 2);
#else
    return *((uint32_t *)NikonFirmwareMultiMakeTxPru0_0 + 2);
#endif
#endif
#if (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#if (PRUICSS_SLICEx == 1)
    return *((uint32_t *)NikonFirmwarePru1_0 + 2);
#else
    return *((uint32_t *)NikonFirmwarePru0_0 + 2);
#endif
#endif
}

static void nikon_display_menu(void)
{
    DebugP_log("\r\n|-------------------------------------------------------------------------------------- |");
    DebugP_log("\r\n|                             Select input parameters                                   |");
    DebugP_log("\r\n|-------------------------------------------------------------------------------------- |");
    if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
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
    DebugP_log("\r\n| 32: Start Continuous Mode                                                             |");
    DebugP_log("\r\n| 33: Select clock frequency in MHz(2.5/4/6.67/8/16)                                    |");
    DebugP_log("\r\n| 34: Update Encoder's Single Turn and Multi turn Resolution                            |");
    DebugP_log("\r\n|---------------------------------------------------------------------------------------|");
    DebugP_log("\r\n| enter value:\r\n");
}

static uint32_t nikon_get_command()
{
    uint32_t cmd;
    DebugP_scanf("%u\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if(((priv->protocol_version == NIKON_PROTOCOL_V2_1) && ((cmd > CMD_22 && cmd < CMD_27))) || (cmd >= CMD_CODE_NUM))
    {
        DebugP_log("\r\n| WARNING: Invalid option, try again\n");
        return SystemP_FAILURE;
    }
    return cmd;
}

static void nikon_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        nikon_position_loop_status = NIKON_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t nikon_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "nikon_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)nikon_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\rnikon_position_loop_decide_termination creation failed\n");
    }

    return status;
}

static void nikon_process_periodic_command(struct nikon_priv *priv, int64_t iep_reset_count, int64_t ch0_trigger_count, int64_t ch1_trigger_count, int64_t ch2_trigger_count)
{
    int32_t status;
    int32_t ret;
    uint32_t ch_num;
    uint32_t ch;
    uint32_t enc_num;
    uint32_t ls_ch;
    uint32_t pos_fail_cnt = 0;
    uint32_t pos_total_cnt = 0;
    struct nikon_periodic_interface nikon_periodic_interface;
    nikon_generate_cdf(priv, CMD_4);
    nikon_config_periodic_trigger(priv);

    if(nikon_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return;
    }

    nikon_periodic_interface_init(priv, &nikon_periodic_interface, iep_reset_count, ch0_trigger_count, ch1_trigger_count, ch2_trigger_count);

    status = nikon_config_periodic_mode(&nikon_periodic_interface, gPruIcssXHandle);
    DebugP_assert(0 != status);
    nikon_position_loop_status = NIKON_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|");
    while(1)
    {
        pos_total_cnt++;
        if(nikon_position_loop_status == NIKON_POSITION_LOOP_STOP)
        {
            nikon_stop_periodic_mode(&nikon_periodic_interface);
            nikon_config_host_trigger(priv);
            DebugP_log("\r\n Failed %u out of %u times\n", pos_fail_cnt, pos_total_cnt);
            return;
        }
        else
        {
            ret = nikon_get_pos(priv, CMD_4);
            if(ret < 0)
            {
                DebugP_log("\r\n ERROR: 40bit ABS measurement failed\n");
                pos_fail_cnt++;
                continue;
            }
            for(ch_num = 0; ch_num < totalchannels; ch_num++)
            {
                ch = nikon_get_current_channel(priv, ch_num);
                if(totalchannels > 1)
                {
                    DebugP_log("%s", (ch_num != (totalchannels-1))?"\r":" & ");
                }
                else
                {
                    DebugP_log("\r");
                }

                if(CONFIG_NIKON0_LOAD_SHARE_MODE)
                {
                    ls_ch = ch;
                }
                else
                {
                    ls_ch = 0;
                }

                for(enc_num = 0; enc_num < priv->num_enc_access[ls_ch]; enc_num++)
                {
                    DebugP_log("Channel:%d-Encoder %d: ", ch, enc_num+1);
                    if(priv->multi_turn_len[ch][enc_num])
                    {
                        DebugP_log("MT:%u, ", priv->pos_data_info[ch].multi_turn[enc_num]);
                    }
                    DebugP_log("Angle:%.12f, CRC Error Count:%u",priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                }
            }
        }
    }
}

void nikon_main(void *args)
{
    int32_t ret;
    uint32_t enc_addr = 0;
    uint32_t ch_num;
    uint32_t enc_num;
    uint32_t pru_num;
    uint32_t ls_ch;
    float_t freq;
    int64_t iep_reset_count=0;
    int64_t ch0_trigger_count=0;
    int64_t ch1_trigger_count=0;
    int64_t ch2_trigger_count=0;
    uint64_t icssClk;
    uint64_t uartClk;
    uint32_t version;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
    /*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_NIKON0_BOOSTER_PACK)
#if (CONFIG_NIKON0_CHANNEL0)
    GPIO_setDirMode(ENC0_EN_BASE_ADDR, ENC0_EN_PIN, ENC0_EN_DIR);
    GPIO_pinWriteHigh(ENC0_EN_BASE_ADDR, ENC0_EN_PIN);
#endif
#if (CONFIG_NIKON0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif
#endif

    /* As per encoder specification, add 0.5 seconds delay after powering up to ensure that encoder is in normal operation state */
    ClockP_usleep(NIKON_POWER_UP_DELAY);

    version = nikon_get_fw_version();

    DebugP_log("\r\nNIKON firmware \t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F,
                (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
    DebugP_log("\r\nNIKON Protocol Version selected\t: %s", (CONFIG_NIKON0_PROTOCOL_VERSION == NIKON_PROTOCOL_V2_1)?"2.1":"3.0");

    nikon_pruicss_init();

    mask = CONFIG_NIKON0_CHANNEL0<<0 | CONFIG_NIKON0_CHANNEL1<<1 | CONFIG_NIKON0_CHANNEL2<<2;

    totalchannels = (CONFIG_NIKON0_CHANNEL0 + CONFIG_NIKON0_CHANNEL1 + CONFIG_NIKON0_CHANNEL2);

    DebugP_log("\r\n");

    icssClk = ICSS_PRU_CORE_CLOCK;
    uartClk = ICSS_PRU_UART_CLOCK;

    priv = nikon_init(gPruIcssXHandle, PRUICSS_SLICEx, CONFIG_NIKON0_BAUDRATE, (uint32_t)icssClk, (uint32_t)uartClk, CONFIG_NIKON0_TX_RX_FIFO_CLOCK_SOURCE, mask, totalchannels, CONFIG_NIKON0_PROTOCOL_VERSION);

    if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        nikon_config_load_share(priv, mask);
        DebugP_log("\r\nNikon Load Share Demo application is running......\n");
        for(pru_num = 0; pru_num < totalchannels; pru_num++)
        {
            DebugP_log("\r\nChannel %d is enabled\n", priv->channel[pru_num]);
        }
    }
    else if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        DebugP_log("\r\nNikon Multi channel, Single PRU Demo application is running......\n");
        for(ch_num = 0; ch_num < totalchannels; ch_num++)
        {
            DebugP_log("\r\nChannel %d is enabled\n", priv->channel[ch_num]);
        }
    }
    else
    {
        DebugP_log("\r\nNikon Single channel, Single PRU Demo application is running......\n");
        DebugP_log("\r\nChannel %d is enabled\n", priv->channel[0]);
    }
    nikon_get_enc_data_len(priv);
    DebugP_log("\r\nRunning CDF4(Multi Transmission command) with maximum encoder address for detecting connected encoder\n");
    ret = nikon_pruicss_load_run_fw(priv, mask);
    if(ret < 0)
    {
        DebugP_log("\r\nERROR: NIKON initialization failed \n");
        DebugP_log("\r\ncheck whether encoder of selected frequency is connected and ensure proper connections\n");
        DebugP_log("\r\nexit %s due to failed firmware initialization\n", __func__);
        goto deinit;
    }
    if(((uint8_t)CONFIG_NIKON0_BAUDRATE == 6) || ((uint8_t)CONFIG_NIKON0_BAUDRATE == 2))
    {
        DebugP_log("\r\nNIKON encoder/encoders detected and running at frequency %fMHz\n", (float)CONFIG_NIKON0_BAUDRATE);
    }
    else
    {
        DebugP_log("\r\nNIKON encoder/encoders detected and running at frequency %dMHz\n", CONFIG_NIKON0_BAUDRATE);
    }

    totalchannels = nikon_get_totalchannels(priv);

    while(1)
    {
        uint32_t cmd;
        int32_t ret;
        int32_t ch;
        uint32_t addr = 0;
        uint32_t data = 0;
        uint32_t bank = 0;
        uint32_t cmd_type;
        ch = nikon_get_current_channel(priv, 0);
        nikon_display_menu();
        cmd = nikon_get_command();

        switch(cmd)
        {
            case CMD_0:
            case CMD_4:
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: 40bit ABS measurement failed\n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                            DebugP_log("\r\n Encoder %d: \n",enc_num);
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

                if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1 || cmd == CMD_5))
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

                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: ABS measurement failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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

                        if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1_VEL || cmd == CMD_5_VEL))
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
                        if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_1_VEL || cmd == CMD_5_VEL))
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

                if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd >= CMD_8 && cmd <= CMD_12))
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

                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);

                if(cmd >= CMD_8_POS && cmd <= CMD_12_POS)
                {
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: ABS measurement failed \n");
                        continue;
                    }
                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: Encoder's operation request failed \n");
                        continue;
                    }

                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                                DebugP_log("\r\n Encoder %d: \n",enc_num);
                            }
                            DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);
                            DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                            DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                            DebugP_log("\r\n ALM: 0x%x \n", priv->alm_field[ch][enc_num]);
                            DebugP_log("\r\n Batt: %u, MtErr: %u, OverFlow: %u, OverSpeed M: %u, Memory Error: %u, Single Turn Error M: %u \n", priv->alm_bits[ch][enc_num].batt, priv->alm_bits[ch][enc_num].mt_err, priv->alm_bits[ch][enc_num].ov_flow, priv->alm_bits[ch][enc_num].ov_spd, priv->alm_bits[ch][enc_num].mem_err, priv->alm_bits[ch][enc_num].st_err);
                            DebugP_log("\r\n PS Error M: %u, Busy M: %u, Memory Busy: %u, Over Temperature: %u, Increment Error M: %u \n",priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err_m);
                            DebugP_log("\r\n OverSpeed S: %u, Single Turn Error S: %u, PS Error S: %u, Busy S: %u, Increment Error S: %u \n",priv->alm_bits[ch][enc_num].ov_spd_s, priv->alm_bits[ch][enc_num].st_err_s, priv->alm_bits[ch][enc_num].ps_err_s, priv->alm_bits[ch][enc_num].busy_s, priv->alm_bits[ch][enc_num].inc_err_s);
                            if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                            {
                                DebugP_log("\r\n PM ALM: 0x%x \n", priv->pm_alm_field[ch][enc_num]);
                                DebugP_log("\r\n INCW1: %u, INCW2: %u, IFW1:%u, IFW2: %u", priv->pm_alm_bits[ch][enc_num].incw_1, priv->pm_alm_bits[ch][enc_num].incw_2, priv->pm_alm_bits[ch][enc_num].ifw_1, priv->pm_alm_bits[ch][enc_num].ifw_2);
                            }
                        }
                    }
                }
                break;

            case CMD_13:
                if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                {
                    while(1)
                    {
                        DebugP_log("\r\n Read with bank? (0 for without bank / 1 for with bank) ");
                        DebugP_scanf("%u", &cmd_type);

                        if(cmd_type == 1)
                        {
                            cmd = CMD_13_BANK;

                            for(pru_num = 0; pru_num < totalchannels; pru_num++)
                            {
                                ch = nikon_get_current_channel(priv, pru_num);
                                if(priv->load_share)
                                {
                                    ls_ch = ch;
                                    DebugP_log("\r\n Channel %d: \n",ch);
                                }
                                else
                                {
                                    ls_ch = 0;
                                    pru_num = nikon_get_totalchannels(priv);
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

                                nikon_update_eeprom_bank(priv, (bank & 0xFF), ls_ch);
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

                for(pru_num = 0; pru_num < totalchannels; pru_num++)
                {
                    ch = nikon_get_current_channel(priv, pru_num);
                    if(priv->load_share)
                    {
                        ls_ch = ch;
                        DebugP_log("\r\nChannel %d: ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = nikon_get_totalchannels(priv);
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
                            nikon_update_eeprom_addr(priv, (addr & 0xFF), ls_ch);
                            break;
                        }
                    }
                }

                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: EEPROM Read access request failed \n");
                    continue;
                }
                else
                {
                    /* 300 microseconds sleep - wait for read data to be determined*/
                    ClockP_usleep(300);
                    ret = nikon_get_pos(priv, cmd);

                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: EEPROM Read access request failed \n");
                        continue;
                    }
                }

                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
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
                if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                {
                    while(1)
                    {
                        DebugP_log("\r\n Read with bank? (0 for without bank / 1 for with bank) ");
                        DebugP_scanf("%u", &cmd_type);

                        if(cmd_type == 1)
                        {
                            cmd = CMD_14_BANK;

                            for(pru_num = 0; pru_num < totalchannels; pru_num++)
                            {
                                ch = nikon_get_current_channel(priv, pru_num);
                                if(priv->load_share)
                                {
                                    ls_ch = ch;
                                    DebugP_log("\r\n Channel %d: \n",ch);
                                }
                                else
                                {
                                    ls_ch = 0;
                                    pru_num = nikon_get_totalchannels(priv);
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

                                nikon_update_eeprom_bank(priv, (bank & 0xFF), ls_ch);
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

                for(pru_num = 0; pru_num < totalchannels; pru_num++)
                {
                    ch = nikon_get_current_channel(priv, pru_num);
                    if(priv->load_share)
                    {
                        ls_ch = ch;
                        DebugP_log("\r\nChannel %d: ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = nikon_get_totalchannels(priv);
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
                            nikon_update_eeprom_addr(priv, (addr & 0xFF), ls_ch);
                            nikon_update_eeprom_data(priv, (data & 0xFFFF), ls_ch);
                            break;
                        }
                    }
                }

                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: EEPROM Write access request failed \n");
                    continue;
                }
                else
                {
                    /* 30 miliseconds sleep - wait for write operation to finish*/
                    ClockP_usleep(30*1000);
                }

                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);

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
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: Encoder's temperature request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    DebugP_log("\r\n Info Field: 0x%x, Temperature: %u \n", priv->pos_data_info[ch].raw_data0[0], priv->temperature[ch][0]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
                break;

            case CMD_16:
            case CMD_17:
                if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_16))
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

                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);

                if(cmd == CMD_16_VEL)
                {
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: Velocity coefficient read request failed \n");
                        continue;
                    }

                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        DebugP_log("\r\n Info Field: 0x%x, Velocity coefficient (Bits [18:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->velocity_coefficient[ch]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    }
                }
                else
                {
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: Identification code read request failed \n");
                        continue;
                    }
                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        DebugP_log("\r\n Info Field: 0x%x, Identification Code (Bits [23:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    }
                }
                break;

            case CMD_18:
            case CMD_19:
            case CMD_20:

                if ((priv->protocol_version == NIKON_PROTOCOL_V3_0) && (cmd == CMD_18))
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
                    for(pru_num = 0; pru_num < totalchannels; pru_num++)
                    {
                        ch = nikon_get_current_channel(priv, pru_num);
                        if(priv->load_share)
                        {
                            ls_ch = ch;
                            DebugP_log("\r\nChannel %d: ", ch);
                        }
                        else
                        {
                            ls_ch = 0;
                            pru_num = nikon_get_totalchannels(priv);
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
                                nikon_update_velocity_coefficient(priv, data, ls_ch);
                                break;
                            }
                        }
                    }

                    nikon_generate_cdf(priv, cmd);
                    ret = nikon_get_pos(priv, cmd);
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: Encoder's Velocity coefficient code write access failed \n");
                        continue;
                    }
                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        DebugP_log("\r\n Info Field: 0x%x, Velocity coefficient (Bits [18:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->velocity_coefficient[ch]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    }
                }
                else
                {
                    for(pru_num = 0; pru_num < totalchannels; pru_num++)
                    {
                        ch = nikon_get_current_channel(priv, pru_num);
                        if(priv->load_share)
                        {
                            ls_ch = ch;
                            DebugP_log("\r\nChannel %d: ", ch);
                        }
                        else
                        {
                            ls_ch = 0;
                            pru_num = nikon_get_totalchannels(priv);
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
                                nikon_update_id_code(priv, data, ls_ch);
                                break;
                            }
                        }
                    }

                    nikon_generate_cdf(priv, cmd);
                    ret = nikon_get_pos(priv, cmd);
                    if(ret < 0)
                    {
                        if(cmd == CMD_20)
                        {
                            DebugP_log("\r\n ERROR: Encoder's address setting failed \n");
                        }
                        else
                        {
                            DebugP_log("\r\n ERROR: Encoder's identification code write access failed \n");
                        }
                        continue;
                    }

                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        DebugP_log("\r\n Info Field: 0x%x, Identification Code (Bits [23:0]): 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    }
                }
                break;

            case CMD_21:
            case CMD_22:
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: 17bit ABS measurement failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                            DebugP_log("\r\n Encoder %d: \n",enc_num);
                        }
                        DebugP_log("\r\n Info Field: 0x%x,  Data Field0: 0x%x, ABS: 0x%llx \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].abs[enc_num]);
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
                if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                {
                    nikon_generate_cdf(priv, cmd);
                    ret = nikon_get_pos(priv, cmd);
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: ABS measurement and velocity/acceleration request failed\n");
                        continue;
                    }

                    for(ch_num = 0; ch_num < totalchannels; ch_num++)
                    {
                        ch = nikon_get_current_channel(priv, ch_num);
                        DebugP_log("\r\n Channel %d: \n",ch);
                        if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's status request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                            DebugP_log("\r\n Encoder %d: \n",enc_num);
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
                        DebugP_log("\r\n PS Error M: %u, Busy M: %u, Memory Busy: %u, Over Temperature: %u, Increment Error M: %u \n",priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err_m);
                        DebugP_log("\r\n OverSpeed S: %u, Single Turn Error S: %u, PS Error S: %u, Busy S: %u, Increment Error S: %u \n",priv->alm_bits[ch][enc_num].ov_spd_s, priv->alm_bits[ch][enc_num].st_err_s, priv->alm_bits[ch][enc_num].ps_err_s, priv->alm_bits[ch][enc_num].busy_s, priv->alm_bits[ch][enc_num].inc_err_s);
                    }
                }
                break;

            case CMD_29:
            case CMD_30:
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: 24bit ABS and encoder's temperature request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    if(CONFIG_NIKON0_LOAD_SHARE_MODE)
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
                            DebugP_log("\r\n Encoder %d: \n",enc_num);
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
                for(pru_num = 0; pru_num < totalchannels; pru_num++)
                {
                    ch = nikon_get_current_channel(priv, pru_num);
                    if(priv->load_share)
                    {
                        ls_ch = ch;
                        DebugP_log("\r\nChannel %d: ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = nikon_get_totalchannels(priv);
                    }
                    while(1)
                    {
                        DebugP_log("\r\n Current encoder address : %u", (uint32_t)nikon_reverse_bits(priv->eax[ls_ch], NIKON_ENC_ADDR_LEN));
                        DebugP_log("\r\n Please enter the encoder address : ");
                        DebugP_scanf("%d", &enc_addr);
                        if(enc_addr > 7)
                        {
                            DebugP_log("\r\n Please enter a 3-bit value(0-7)\n");
                        }
                        else
                        {
                            break;
                        }
                    }
                    nikon_update_enc_addr(priv, enc_addr, ls_ch);
                }
                break;

            case START_CONTINUOUS_MODE:
                DebugP_log("\r| Enter IEP cycle count(must be greater than Nikon cycle time, in IEP cycles): ");
                DebugP_scanf("%lld\n", &iep_reset_count);
                if(iep_reset_count <= IEP_DEFAULT_INC)
                {
                    DebugP_log("\r\n| WARNING: invalid value entered\n");
                    continue;
                }
                if(CONFIG_NIKON0_LOAD_SHARE_MODE)
                {
                    if(CONFIG_NIKON0_CHANNEL0)
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel0: \n");
                        DebugP_scanf("%lld\n", &ch0_trigger_count);
                        if((ch0_trigger_count > iep_reset_count) || (ch0_trigger_count <= IEP_DEFAULT_INC))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }
                    if(CONFIG_NIKON0_CHANNEL1)
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel1: \n");
                        DebugP_scanf("%lld\n", &ch1_trigger_count);
                        if((ch1_trigger_count > iep_reset_count) || (ch1_trigger_count <= IEP_DEFAULT_INC))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }
                    if(CONFIG_NIKON0_CHANNEL2)
                    {
                        DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel2: \n");
                        DebugP_scanf("%lld\n", &ch2_trigger_count);
                        if((ch2_trigger_count > iep_reset_count) || (ch2_trigger_count <= IEP_DEFAULT_INC))
                        {
                            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                            continue;
                        }
                    }

                }
                else
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
                    DebugP_scanf("%lld\n", &ch0_trigger_count);
                    if((ch0_trigger_count > iep_reset_count) || (ch0_trigger_count <= IEP_DEFAULT_INC))
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        continue;
                    }
                }
                nikon_process_periodic_command(priv, iep_reset_count, ch0_trigger_count, ch1_trigger_count, ch2_trigger_count);
                nikon_command_wait(priv);
                break;

            case UPDATE_CLOCK_FREQ:
                DebugP_log("\r\nPlease enter frequency in MHz:\n");
                DebugP_scanf("%f\n", &freq);
                if(!((freq == NIKON_FREQ_2_5MHZ) || (freq == NIKON_FREQ_4MHZ) || (((uint8_t)freq % NIKON_FREQ_6_67MHZ) < 1) || (freq == NIKON_FREQ_8MHZ) || (freq == NIKON_FREQ_16MHZ)))
                {
                    DebugP_log("\r\n CLK divisors will not be possible. Please provide valid freq: 2.5/4/6.67/8/16 \n");
                    continue;
                }
                nikon_update_clock_freq(priv, freq);
                break;

            case UPDATE_ENC_LEN:
                nikon_get_enc_data_len(priv);
                break;

            default:
                break;
        }
    }
deinit:

    Board_driversClose();
    Drivers_close();
    return;
}
