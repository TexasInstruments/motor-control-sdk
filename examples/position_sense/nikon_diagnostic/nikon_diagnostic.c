/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <drivers/sciclient.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <position_sense/nikon/include/nikon_drv.h>
#include <position_sense/nikon/include/nikon_api.h>
#include "nikon_periodic_trigger.h"

#define PRUICSS_SLICEx PRUICSS_PRUx

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#include  <position_sense/nikon/firmware/nikon_receiver_multi_bin.h>
#endif

#if (CONFIG_NIKON0_CHANNEL0) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#include <position_sense/nikon/firmware/nikon_receiver_multi_RTU_bin.h>
#endif

#if (CONFIG_NIKON0_CHANNEL1) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#include <position_sense/nikon/firmware/nikon_receiver_multi_PRU_bin.h>
#endif

#if (CONFIG_NIKON0_CHANNEL2) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
#include <position_sense/nikon/firmware/nikon_receiver_multi_TXPRU_bin.h>
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#include <position_sense/nikon/firmware/nikon_receiver_bin.h>
#endif

#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)
#define NIKON_POSITION_LOOP_STOP            0
#define NIKON_POSITION_LOOP_START           1
struct nikon_priv *priv;
/** \brief Global Structure pointer holding PRU-ICSSG memory Map. */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

PRUICSS_Handle gPruIcssXHandle;
TaskP_Object gTaskObject;

static int32_t nikon_position_loop_status;
int32_t totalchannels = 0;
int32_t mask = 0;

static void nikon_pruicss_init(void)
{
    int32_t status = SystemP_FAILURE;
    int32_t size;
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
     /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
    status = PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    DebugP_assert(SystemP_SUCCESS == status);
    /* clear ICSS0 PRUx data RAM */
    size = PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_PRUx));
    DebugP_assert(size);
    if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
        /*Set in constant table C28 for  tx pru*/
        if(CONFIG_PRU_ICSS0)
        {
            /*ICSSG_PRU_CONTROL registers offset for ICSSG1 is 0xA58 */
            PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0xA58);
        }
        else
        {
            /*ICSSG_PRU_CONTROL registers offset for ICSSG0 is 0x258 */
            PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);
        }
    }
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
}

int32_t nikon_pruicss_load_run_fw(struct nikon_priv *priv, uint8_t mask)
{
    int32_t status = SystemP_SUCCESS;
    int32_t size;
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU) /*enable loadshare mode*/
#if(CONFIG_NIKON0_CHANNEL0)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeRTU_0,
                                                        sizeof(NikonFirmwareMultiMakeRTU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_NIKON0_CHANNEL1)
    status=PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx );
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) NikonFirmwareMultiMakePRU_0,
                                                      sizeof(NikonFirmwareMultiMakePRU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_NIKON0_CHANNEL2)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) NikonFirmwareMultiMakeTXPRU_0,
                                                        sizeof(NikonFirmwareMultiMakeTXPRU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#if(CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmwareMulti_0,
                                sizeof(NikonFirmwareMulti_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                                0, (uint32_t *) NikonFirmware_0,
                                sizeof(NikonFirmware_0));
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
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
        DebugP_log("\r\nPlease enter 1st encoder single turn length\n");
        DebugP_scanf("%u\n", &single_turn_len[0]);
        num_encoders = 1;
        DebugP_log("\r\nPlease enter 1st encoder multi turn length (zero if not a multi turn encoder)\n");
        DebugP_scanf("%u\n", &multi_turn_len[0]);
        DebugP_log("\r\nPlease enter 2nd encoder single turn length (zero if not connected)\n");
        DebugP_scanf("%u\n", &single_turn_len[1]);
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 2nd encoder multi turn length (zero if not a multi turn encoder)\n");
            DebugP_scanf("%u\n", &multi_turn_len[1]);
            num_encoders = 2;
            DebugP_log("\r\nPlease enter 3rd encoder single turn length (zero if not connected)\n");
            DebugP_scanf("%u\n", &single_turn_len[2]);
            if(single_turn_len[2])
            {
                DebugP_log("Please enter 3rd encoder multi turn length (zero if not a multi turn encoder)\n");
                DebugP_scanf("%u\n", &multi_turn_len[2]);
                num_encoders = 3;
            }
        }
        nikon_update_enc_len(priv, num_encoders, single_turn_len, multi_turn_len, ch);
    }
}

uint64_t nikon_get_fw_version(void)
{
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
    return *((unsigned long *)NikonFirmwareMulti_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL0) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((unsigned long *)NikonFirmwareMultiMakeRTU_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL1) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((unsigned long *)NikonFirmwareMultiMakePRU_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL2) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((unsigned long *)NikonFirmwareMultiMakeTXPRU_0 + 2);
#endif
#if (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    return *((unsigned long *)NikonFirmware_0 + 2);
#endif
}

static void nikon_display_menu(void)
{
    DebugP_log("\r\n|--------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select input parametes                             |");
    DebugP_log("\r\n|--------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 0 :   ABS full 40 bit data request                                             |");
    DebugP_log("\r\n| 1 :   ABS lower 24bit data request                                             |");
    DebugP_log("\r\n| 2 :   ABS upper 24bit data request                                             |");
    DebugP_log("\r\n| 3 :   Encoder status Request                                                   |");
    DebugP_log("\r\n| 4 :   ABS full 40 bit data request(MT)                                         |");
    DebugP_log("\r\n| 5 :   ABS lower 24bit data request(MT)                                         |");
    DebugP_log("\r\n| 6 :   ABS upper 24bit data request(MT)                                         |");
    DebugP_log("\r\n| 7 :   Encoder status Request(MT)                                               |");
    DebugP_log("\r\n| 8 :   Status flag clear request                                                |");
    DebugP_log("\r\n| 9 :   Multiple turn data clear request                                         |");
    DebugP_log("\r\n| 10:   Status+ Multiple turn data clear request                                 |");
    DebugP_log("\r\n| 11:   Encoder address setting I (one-to-one connection)                        |");
    DebugP_log("\r\n| 12:   Single turn data zero preset                                             |");
    DebugP_log("\r\n| 13:   EEPROM read request                                                      |");
    DebugP_log("\r\n| 14:   EEPROM write request                                                     |");
    DebugP_log("\r\n| 15:   Temperature data request                                                 |");
    DebugP_log("\r\n| 16:   Identification code read I                                               |");
    DebugP_log("\r\n| 17:   Identification code read II(one-to-one connection)                       |");
    DebugP_log("\r\n| 18:   Identification code write I                                              |");
    DebugP_log("\r\n| 19:   Identification code write II(one-to-one connection)                      |");
    DebugP_log("\r\n| 20:   Encoder address setting II                                               |");
    DebugP_log("\r\n| 21:   ABS lower 17bit data request                                             |");
    DebugP_log("\r\n| 22:   ABS lower 17bit data request(MT)                                         |");
    DebugP_log("\r\n| 27:   ABS lower 24bit + status request                                         |");
    DebugP_log("\r\n| 28:   ABS lower 24bit + status request(MT)                                     |");
    DebugP_log("\r\n| 29:   ABS lower 24bit + Temperature data request                               |");
    DebugP_log("\r\n| 30:   ABS lower 24bit + Temperature data request(MT)                           |");
    DebugP_log("\r\n| Below options are user application specific for cmd prepare                    |");
    DebugP_log("\r\n| 31:   Update Encoder address(EAX) in APP local context                         |");
    DebugP_log("\r\n| 32:   Start Continuous Mode                                                    |");
    DebugP_log("\r\n| 33:   Select clock frequency in MHz(2.5/4/6.67/8/16)                           |");
    DebugP_log("\r\n| 34:   Update Encoder's single turn and Multi turn Resoltions                   |");
    DebugP_log("\r\n|--------------------------------------------------------------------------------|");
    DebugP_log("\r\n| enter value:\r\n");
}
static int nikon_get_command()
{
    int cmd;
    DebugP_scanf("%d\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if((cmd > CMD_22 && cmd < CMD_27) || (cmd >= CMD_CODE_NUM))
    {
        DebugP_log("\r\n| WARNING: invalid option try again\n");
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

    return status ;
}
static void nikon_process_periodic_command(struct nikon_priv *priv, int64_t cmp3)
{
    int32_t status;
    int32_t ret;
    int32_t ch_num;
    int32_t ch;
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

    nikon_periodic_interface_init(priv, &nikon_periodic_interface, cmp3);

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
                DebugP_log("Channel:%d - Encoder1: ",ch);
                if(priv->multi_turn_len[ch][0])
                {
                    DebugP_log("MT rev:%u, ", priv->pos_data_info[ch].multi_turn[0]);
                }
                DebugP_log("Angle:%.12f, crc error count:%u",priv->pos_data_info[ch].angle[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                if(priv->single_turn_len[ch][1])
                {
                    DebugP_log(", Encoder2: ");
                    if(priv->multi_turn_len[ch][1])
                    {
                        DebugP_log("MT rev:%u, ", priv->pos_data_info[ch].multi_turn[1]);
                    }
                    DebugP_log("Angle:%.12f, crc error count:%u",priv->pos_data_info[ch].angle[1], priv->pos_data_info[ch].crc_err_cnt[1]);
                    if(priv->single_turn_len[ch][2])
                    {
                        DebugP_log("Encoder3: ");
                        if(priv->multi_turn_len[ch][2])
                        {
                            DebugP_log("MT rev:%u, ", priv->pos_data_info[ch].multi_turn[2]);
                        }
                        DebugP_log("Angle:%.12f, crc error count:%u",priv->pos_data_info[ch].angle[2], priv->pos_data_info[ch].crc_err_cnt[2]);
                    }
                }

            }
        }
    }
}

void nikon_main(void *args)
{
    int32_t i;
    int32_t enc_addr = 0;
    int32_t ch_num;
    int32_t enc_num;
    int32_t pru_num;
    int32_t ls_ch;
    float_t freq;
    int64_t cmp3;
    uint64_t icssgclk;
    uint64_t uartclk;
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

    i = nikon_get_fw_version();

    DebugP_log("\n\n");
    DebugP_log("NIKON firmware \t: %x.%x.%x (%s)\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    nikon_pruicss_init();

    i = CONFIG_NIKON0_CHANNEL0 & 0;

    i += CONFIG_NIKON0_CHANNEL1;

    i += CONFIG_NIKON0_CHANNEL2<<1;

    if(i < 0 || i > 2)
    {
        DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
        i = 0;
    }

    mask = CONFIG_NIKON0_CHANNEL0<<0 | CONFIG_NIKON0_CHANNEL1<<1 | CONFIG_NIKON0_CHANNEL2<<2;

    totalchannels = (CONFIG_NIKON0_CHANNEL0 + CONFIG_NIKON0_CHANNEL1 + CONFIG_NIKON0_CHANNEL2);

    DebugP_log("\r\n");

    /* Read the ICSSG configured clock frequency. */
    if(gPruIcssXHandle->hwAttrs->instance)
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, &icssgclk);
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_UCLK_CLK, &uartclk);
    }
    else
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &icssgclk);
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_UCLK_CLK, &uartclk);
    }

    priv = nikon_init(gPruIcssXHandle, PRUICSS_PRUx, CONFIG_NIKON0_BAUDRATE, (uint32_t)icssgclk, (uint32_t)uartclk, mask, totalchannels);

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
    i = nikon_pruicss_load_run_fw(priv, mask);
    if(i < 0)
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
        int8_t cmd;
        int32_t ret;
        int32_t ch;
        uint32_t addr = 0;
        uint32_t data_high = 0;
        uint32_t data_mid = 0;
        uint32_t data_low = 0;
        ch = nikon_get_current_channel(priv, 0);
        nikon_display_menu();
        cmd = nikon_get_command();
        if(cmd < CMD_0)
        {
            continue;
        }
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
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: 24bit ABS meaasurement failed \n");
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
                        if(cmd == CMD_5 || cmd == CMD_6)
                        {
                            DebugP_log("\r\n Encoder %d: \n",enc_num);
                        }
                        DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1: 0x%x\n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u\n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u, ", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num], priv->enc_info[ch].enc_cmd[enc_num]);
                        DebugP_log("ABS: 0x%llx \n", priv->pos_data_info[ch].abs[enc_num]);
                        if((priv->abs_len >= priv->single_turn_len[ch][enc_num])&& cmd != CMD_2 && cmd != CMD_6)
                        {
                            DebugP_log("\r\n Angle: %.12f , Multi Turn Rev: %u \n", priv->pos_data_info[ch].angle[enc_num], priv->pos_data_info[ch].multi_turn[enc_num]);
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
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: Encoder's Status request failed \n");
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
                        DebugP_log("ALM: 0x%x\n", priv->alm_field[ch][enc_num]);
                        DebugP_log("\r\n Batt: %u, MtErr: %u, OverFlow: %u, OverSpeed: %u, Memory Error: %u, Single Turn Error: %u \n", priv->alm_bits[ch][enc_num].batt, priv->alm_bits[ch][enc_num].mt_err, priv->alm_bits[ch][enc_num].ov_flow, priv->alm_bits[ch][enc_num].ov_spd, priv->alm_bits[ch][enc_num].mem_err, priv->alm_bits[ch][enc_num].st_err);
                        DebugP_log("\r\n PS Error: %u, Busy: %u, Memory Busy: %u, Over Temperature: %u, Increment Error: %u \n",priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err);
                    }
                }
                break;

            case CMD_13:
                while(1)
                {
                    DebugP_log("\r\n Enter Memory location to read(00h to FFh are available): ");
                    DebugP_scanf("%x", &addr);
                    if(addr > 0xFF)
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit address\n");
                    }
                    else
                    {
                        break;
                    }
                }
                nikon_update_eeprom_addr(priv, addr);
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: EEPROM Read access request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->pos_data_info[ch].raw_data1[0], priv->pos_data_info[ch].raw_data2[0]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    if(addr == 0xF9)
                    {
                        DebugP_log("\r\n Temperature: %d \n", priv->temperature[ch][0]);
                    }
                }
                break;

            case CMD_14:
                while(1)
                {
                    DebugP_log("\r\n Enter Memory location to write(00h to EFh are available): ");
                    DebugP_scanf("%x", &addr);
                    DebugP_log("\r\n Enter upper byte of data to write at Memory location 0x%x: ", addr);
                    DebugP_scanf("%x", &data_high);
                    DebugP_log("\r\n Enter lower byte of data to write in Memory location 0x%x: ", addr);
                    DebugP_scanf("%x", &data_low);
                    if((data_high > 0xFF) || (data_low > 0xFF) || (addr > 0xEF))
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit value\n");
                    }
                    else
                    {
                        break;
                    }
                }
                nikon_update_eeprom_addr(priv, addr);
                nikon_update_eeprom_data(priv, data_high, data_low);
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: EEPROM Write access request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->pos_data_info[ch].raw_data1[0], priv->pos_data_info[ch].raw_data2[0]);
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
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: Identification code read request failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    DebugP_log("\r\n Info Field: 0x%x, Identification Code: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                }
                break;

            case CMD_18:
            case CMD_19:
            case CMD_20:
                while(1)
                {
                    DebugP_log("\r\n Enter upper byte of data to assign as identification code: ");
                    DebugP_scanf("%x", &data_high);
                    DebugP_log("\r\n Enter middle byte of data to assign as identification code: ");
                    DebugP_scanf("%x", &data_mid);
                    DebugP_log("\r\n Enter lower byte of data to assign as identification code: ");
                    DebugP_scanf("%x", &data_low);
                    if((data_high > 0xFF) || (data_mid > 0xFF) || (data_low > 0xFF))
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit value \n");
                    }
                    else
                    {
                        break;
                    }
                }
                nikon_update_id_code(priv, data_high, data_mid, data_low);
                nikon_generate_cdf(priv, cmd);
                ret = nikon_get_pos(priv, cmd);
                if(ret < 0)
                {
                    DebugP_log("\r\n ERROR: Encoder's identification code write access failed \n");
                    continue;
                }
                for(ch_num = 0; ch_num < totalchannels; ch_num++)
                {
                    ch = nikon_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: \n",ch);
                    DebugP_log("\r\n Info Field: 0x%x, Identification Code: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], priv->identification_code[ch]);
                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
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
                        DebugP_log("\r\n Info Field: 0x%x, ABS: 0x%llx \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].abs[enc_num]);
                        DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[enc_num], priv->pos_data_info[ch].otf_crc[enc_num], priv->pos_data_info[ch].crc_err_cnt[enc_num]);
                        DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x\n", priv->enc_info[ch].enc_addr[enc_num], priv->enc_info[ch].enc_status[enc_num]);

                        if(priv->abs_len >= priv->single_turn_len[ch][enc_num])
                        {
                            DebugP_log("\r\n Angle: %.12f\n", priv->pos_data_info[ch].angle[enc_num]);
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
                        DebugP_log("\r\n Batt: %u, MtErr: %u, OverFlow: %u, OverSpeed: %u, Memory Error: %u, Single Turn Error: %u \n", priv->alm_bits[ch][enc_num].batt, priv->alm_bits[ch][enc_num].mt_err, priv->alm_bits[ch][enc_num].ov_flow, priv->alm_bits[ch][enc_num].ov_spd, priv->alm_bits[ch][enc_num].mem_err, priv->alm_bits[ch][enc_num].st_err);
                        DebugP_log("\r\n PS Error: %u, Busy: %u, Memory Busy: %u, Over Temperature: %u, Increment Error: %u \n",priv->alm_bits[ch][enc_num].ps_err, priv->alm_bits[ch][enc_num].busy, priv->alm_bits[ch][enc_num].mem_busy, priv->alm_bits[ch][enc_num].ov_temp, priv->alm_bits[ch][enc_num].inc_err);
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
                        DebugP_log("\r\n Info Field: 0x%x, Data Field0: 0x%x, Data Field1; 0x%x, Data Field2: 0x%x \n", priv->pos_data_info[ch].raw_data0[enc_num], priv->pos_data_info[ch].raw_data1[enc_num], priv->pos_data_info[ch].raw_data2[enc_num], priv->pos_data_info[ch].raw_data3[enc_num], priv->pos_data_info[ch].abs[enc_num]);
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
                        DebugP_log("\r\nChannel %d - ", ch);
                    }
                    else
                    {
                        ls_ch = 0;
                        pru_num = nikon_get_totalchannels(priv);
                    }
                    DebugP_log(" Please enter the encoder address : ");
                    DebugP_scanf("%d", &enc_addr);
                    if(enc_addr > 7)
                    {
                        DebugP_log("\r\n Please enter a 3-bit value(0-7)\n");
                        continue;
                    }
                    nikon_update_enc_addr(priv, enc_addr, ls_ch);
                }
                break;

            case START_CONTINUOUS_MODE:
                DebugP_log("\r| Enter IEP cycle count(must be greater than Nikon cycle time in nano seconds): ");
                if(DebugP_scanf("%lld\n", &cmp3) < 0)
                {
                    DebugP_log("\r\n| WARNING: invalid value entered\n");
                    continue;
                }
                nikon_process_periodic_command(priv, cmp3);
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
