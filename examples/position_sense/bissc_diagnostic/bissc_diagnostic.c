/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include <position_sense/bissc/include/bissc_drv.h>
#include <position_sense/bissc/include/bissc_api.h>
#include "bissc_periodic_trigger.h"

#define PRUICSS_SLICEx PRUICSS_PRUx

#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
#include  <position_sense/bissc/firmware/bissc_receiver_multi_bin.h>
#endif

#if (CONFIG_BISSC0_CHANNEL0) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
#include <position_sense/bissc/firmware/bissc_receiver_multi_RTU_bin.h>
#endif

#if (CONFIG_BISSC0_CHANNEL1) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
#include <position_sense/bissc/firmware/bissc_receiver_multi_PRU_bin.h>
#endif

#if (CONFIG_BISSC0_CHANNEL2) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
#include <position_sense/bissc/firmware/bissc_receiver_multi_TXPRU_bin.h>
#endif

#if (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
#include <position_sense/bissc/firmware/bissc_receiver_bin.h>
#endif

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
#define BISSC_CMD_PERIODIC_TRIGGER          (6)

#define BISSC_POSITION_LOOP_STOP            0
#define BISSC_POSITION_LOOP_START           1

struct bissc_priv *priv;
/** \brief Global Structure pointer holding PRU-ICSSG memory Map. */
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
static int32_t bissc_position_loop_status;
int32_t totalchannels = 0, mask = 0;

TaskP_Object gTaskObject;
PRUICSS_Handle gPruIcssXHandle;

static void bissc_pruicss_init(void)
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
    if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
        DebugP_assert(SystemP_SUCCESS == status);
    }
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
}

int32_t bissc_pruicss_load_run_fw(struct bissc_priv *priv, uint8_t mask)
{
    int32_t status = SystemP_FAILURE, size;
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU) /*enable loadshare mode*/
#if(CONFIG_BISSC0_CHANNEL0)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) BiSSFirmwareMultiMakeRTU_0,
                                                        sizeof(BiSSFirmwareMultiMakeRTU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_BISSC0_CHANNEL1)
    status=PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx );
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) BiSSFirmwareMultiMakePRU_0,
                                                      sizeof(BiSSFirmwareMultiMakePRU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#if(CONFIG_BISSC0_CHANNEL2)
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    size = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) BiSSFirmwareMultiMakeTXPRU_0,
                                                        sizeof(BiSSFirmwareMultiMakeTXPRU_0));
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
#else
    status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                                0, (uint32_t *) BiSSFirmwareMulti_0,
                                sizeof(BiSSFirmwareMulti_0));
#else
    size = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                                0, (uint32_t *) BiSSFirmware_0,
                                sizeof(BiSSFirmware_0));
#endif
    DebugP_assert(size);
    status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
    /*Run firmware */
    status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
    DebugP_assert(SystemP_SUCCESS == status);
#endif
    /* check initialization ack from firmware, with a timeout of 5 second */
    status = bissc_wait_for_fw_initialization(priv, WAIT_5_SECOND, mask);
    return status;
}

uint64_t bissc_get_fw_version(void)
{
#if (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    return *((unsigned long *)BiSSFirmwareMulti_0 + 2);
#endif
#if (CONFIG_BISSC0_CHANNEL0) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
    return *((unsigned long *)BiSSFirmwareMultiMakeRTU_0 + 2);
#endif
#if (CONFIG_BISSC0_CHANNEL1) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
    return *((unsigned long *)BiSSFirmwareMultiMakePRU_0 + 2);
#endif
#if (CONFIG_BISSC0_CHANNEL2) && (CONFIG_BISSC0_LOAD_SHARE_MODE)
    return *((unsigned long *)BiSSFirmwareMultiMakeTXPRU_0 + 2);
#endif
#if (CONFIG_BISSC0_MODE == BISSC_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    return *((unsigned long *)BiSSFirmware_0 + 2);
#endif
}

static int32_t bissc_clock_config(uint32_t frequency, struct bissc_priv *priv)
{
    struct bissc_clk_cfg clk_cfg;
    int32_t status = SystemP_FAILURE;
    bissc_update_clock_freq(priv, frequency);
    if(bissc_calc_clock(priv, &clk_cfg) < 0)
    {
        return SystemP_FAILURE;
    }
    DebugP_logInfo("\r| clock config values - tx_div: %u\trx_div: %u\trx_div_attr: %x\n",
                    clk_cfg.tx_div, clk_cfg.rx_div, clk_cfg.rx_div_attr);
    bissc_update_max_proc_delay(priv);
    bissc_hw_init(priv);
    status = bissc_wait_measure_proc_delay(priv, WAIT_5_SECOND);
    return status;
}

static void bissc_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select input parametes                           |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 1 : Number of position data bits                                             |");
    DebugP_log("\r\n| 2 : Select clock frequency in MHz(1/2/5/8/10)                                |");
    DebugP_log("\r\n| 3 : Encoder send position values                                             |");
    DebugP_log("\r\n| 4 : Control Communication - Register Read/Write                              |");
    DebugP_log("\r\n| 5 : Loop over BiSS-C cycles                                                  |");
    DebugP_log("\r\n| 6 : Start continuous mode                                                    |");
    DebugP_log("\r\n| 0 : Exit the application                                                     |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| enter value:\r\n");
}

void bissc_get_enc_data_len(struct bissc_priv *priv)
{
    int32_t ch_num, totalchns;
    uint32_t  single_turn_len[3] = {0}, multi_turn_len[3] = {0};
    bissc_clear_data_len(priv);
    if(priv->load_share)
        totalchns = bissc_get_totalchannels(priv);
    else
        totalchns = 1;

    for(ch_num = 0; ch_num < totalchns; ch_num++)
    {
        DebugP_log("\r\nPlease enter encoder lengths connected to Channel %d:\n", priv->channel[ch_num]);
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
        bissc_update_data_len(priv, single_turn_len, multi_turn_len, ch_num);
    }
}

static void bissc_print_res(struct bissc_priv *priv)
{
    int32_t ch_num, ch, ls_ch;
    for( ch_num = 0; ch_num < totalchannels; ch_num++)
    {
        ch = bissc_get_current_channel(priv, ch_num);
        if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
            ls_ch = ch;
        else
            ls_ch = 0;
        if((CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU) || (CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU))
            DebugP_log("%s", (ch_num != (totalchannels-1))?"\r":" & ");
        else
            DebugP_log("\r");
        if(priv->data_len[ls_ch][1])
        {
            if(priv->data_len[ls_ch][2])
            {
                if(priv->multi_turn_len[ls_ch][2])
                {
                    DebugP_log("Channel:%d - Enc3: MT rev:%u, Angle:%.12f, Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc_1:%u ",ch, priv->enc_pos_data[ch].num_of_turns[2], priv->enc_pos_data[ch].angle[2],priv->enc_pos_data[ch].num_of_turns[1], priv->enc_pos_data[ch].angle[1],
                    priv->enc_pos_data[ch].num_of_turns[0], priv->enc_pos_data[ch].angle[0], priv->pd_crc_err_cnt[ch][2], priv->pd_crc_err_cnt[ch][1], priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%d - Enc3: Angle:%.12f, Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc3:%u, crc error count enc2:%u, crc error count enc1:%u ",ch, priv->enc_pos_data[ch].angle[2], priv->enc_pos_data[ch].angle[1],
                    priv->enc_pos_data[ch].angle[0], priv->pd_crc_err_cnt[ch][2], priv->pd_crc_err_cnt[ch][1], priv->pd_crc_err_cnt[ch][0]);
                }
            }
            else
            {
                if(priv->multi_turn_len[ls_ch][1])
                {
                    DebugP_log("Channel:%d - Enc2: MT rev:%u, Angle:%.12f, Enc1: MT rev:%u, Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ",ch,priv->enc_pos_data[ch].num_of_turns[1], priv->enc_pos_data[ch].angle[1],
                    priv->enc_pos_data[ch].num_of_turns[0], priv->enc_pos_data[ch].angle[0],priv->pd_crc_err_cnt[ch][1], priv->pd_crc_err_cnt[ch][0]);
                }
                else
                {
                    DebugP_log("Channel:%d - Enc2: Angle:%.12f, Enc1: Angle:%.12f, crc error count enc2:%u, crc error count enc1:%u ",ch, priv->enc_pos_data[ch].angle[1], priv->enc_pos_data[ch].angle[0], priv->pd_crc_err_cnt[ch][1],
                    priv->pd_crc_err_cnt[ch][0]);
                }
            }
        }
        else
        {
            if(priv->multi_turn_len[ls_ch][0])
            {
                DebugP_log("Channel:%d - Enc1: MT rev:%u, Angle:%.12f, crc error count enc1:%u ",ch, priv->enc_pos_data[ch].num_of_turns[0], priv->enc_pos_data[ch].angle[0],
                priv->pd_crc_err_cnt[ch][0]);
            }
            else
            {
                DebugP_log("Channel:%d - Enc1: Angle:%.12f, crc error count enc1:%u ",ch, priv->enc_pos_data[ch].angle[0],
                priv->pd_crc_err_cnt[ch][0]);
            }
        }
    }
}

static int bissc_get_command()
{
    uint32_t cmd;
    DebugP_scanf("%d\n", &cmd);
    /* Check to make sure that the command issued is correct */
    if( cmd < BISSC_CMD_EXIT_APP || cmd > BISSC_CMD_PERIODIC_TRIGGER)
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
        bissc_position_loop_status = BISSC_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

static int32_t bissc_loop_task_create(void)
{
    uint32_t status;
    TaskP_Params taskParams;

    TaskP_Params_init(&taskParams);
    taskParams.name = "bissc_position_loop_decide_termination";
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)gTaskFxnStack;
    taskParams.priority = TASK_PRIORITY;
    taskParams.taskMain = (TaskP_FxnMain)bissc_position_loop_decide_termination;
    status = TaskP_construct(&gTaskObject, &taskParams);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("\rbissc_position_loop_decide_termination creation failed\n");
    }

    return status ;
}

static void bissc_process_periodic_command(struct bissc_priv *priv, int64_t cmp3, int64_t cmp0)
{
    int32_t status, ret;
    uint32_t pos_fail_cnt = 0, pos_total_cnt = 0;
    struct bissc_periodic_interface bissc_periodic_interface;
    bissc_config_periodic_trigger(priv);

    if(bissc_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return;
    }
    bissc_periodic_interface_init(priv, &bissc_periodic_interface, cmp3, cmp0);
    status = bissc_config_periodic_mode(&bissc_periodic_interface, gPruIcssXHandle);
    DebugP_assert(0 != status);
    bissc_position_loop_status = BISSC_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|");

    while(1)
    {
        pos_total_cnt++;
        if(bissc_position_loop_status == BISSC_POSITION_LOOP_STOP)
        {
            bissc_stop_periodic_mode(&bissc_periodic_interface);
            bissc_config_host_trigger(priv);
            DebugP_log("\r\n Failed %u out of %u times\n", pos_fail_cnt, pos_total_cnt);
            return;
        }
        else
        {
            ret = bissc_get_pos(priv);
            if(ret < 0)
            {
                DebugP_log("\r\n ERROR: Position data measurement failed \n");
                pos_fail_cnt++;
                continue;
            }
            bissc_print_res(priv);
        }
    }
}
void bissc_main(void *args)
{
    int32_t i, totalchns, ch_num, ls_ch = 0;
    uint64_t icssgclk;
    uint64_t uartclk;
    int32_t ch = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
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

    i = bissc_get_fw_version();

    DebugP_log("\n\n");
    DebugP_log("BiSS-C firmware \t: %x.%x.%x (%s)\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    bissc_pruicss_init();

    i = CONFIG_BISSC0_CHANNEL0 & 0;

    i += CONFIG_BISSC0_CHANNEL1;

    i += CONFIG_BISSC0_CHANNEL2<<1;

    if(i < 0 || i > 2)
    {
        DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
        i = 0;
    }

    mask = CONFIG_BISSC0_CHANNEL0<<0 | CONFIG_BISSC0_CHANNEL1<<1 | CONFIG_BISSC0_CHANNEL2<<2;

    totalchannels = (CONFIG_BISSC0_CHANNEL0 + CONFIG_BISSC0_CHANNEL1 + CONFIG_BISSC0_CHANNEL2);

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

    priv = bissc_init(gPruIcssXHandle, PRUICSS_PRUx, CONFIG_BISSC0_BAUDRATE, (uint32_t)icssgclk, (uint32_t)uartclk);
    bissc_config_channel(priv, mask, totalchannels);
    if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        bissc_config_load_share(priv, mask);
    }

    bissc_set_default_initialization(priv, icssgclk);
    if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
    {
        DebugP_log("\r\nBiSS-C Load Share Demo application is running......\n");
        for(ch_num = 0; ch_num < totalchannels; ch_num++)
        {
            DebugP_log("\r\nChannel %d is enabled\n", priv->channel[ch_num]);
        }
    }
    else if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        DebugP_log("\r\nBiSS-C Multi channel, Single PRU Demo application is running......\n");
        for(ch_num = 0; ch_num < totalchannels; ch_num++)
        {
            DebugP_log("\r\nChannel %d is enabled\n", priv->channel[ch_num]);
        }
    }
    else
    {
        DebugP_log("\r\nBiSS-C Single channel, Single PRU Demo application is running......\n");
        DebugP_log("\r\nChannel %d is enabled\n", priv->channel[0]);
    }

    bissc_get_enc_data_len(priv);
    i = bissc_pruicss_load_run_fw(priv, mask);
    if(i < 0)
    {
        DebugP_log("\r\nERROR: BiSS-C initialization failed \n");
        DebugP_log("\r\ncheck whether encoder is connected and ensure proper connections\n");
        DebugP_log("\r\nexit %s due to failed firmware initialization\n", __func__);
        goto deinit;
    }

    bissc_get_enc_proc_delay(priv);
    if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_SINGLE_PRU)
    {
        if(priv->totalchannels > 1)
        {
            if(priv->proc_delay[priv->channel[0]] == priv->proc_delay[priv->channel[1]])
            {
                if(priv->totalchannels > 2)
                {
                    if(priv->proc_delay[priv->channel[1]] !=  priv->proc_delay[priv->channel[2]])
                    {
                        DebugP_log("\r\n Encoders connected accross channels have different processing delays, multi channel configuration is not supported\n");
                        goto deinit;
                    }
                }
            }
            else
            {
                DebugP_log("\r\n Encoders connected accross channels have different processing delays, multi channel configuration is not supported\n");
                goto deinit;
            }
        }
    }

    DebugP_log("\r\nBiSS-C encoder/encoders detected and running at frequency %dMHz\n", CONFIG_BISSC0_BAUDRATE);
    for( ch_num = 0; ch_num < totalchannels; ch_num++)
    {
        DebugP_log("\r\nProcessing Delay in clock cycles for Channel %d : %u\n",priv->channel[ch_num], priv->proc_delay[priv->channel[ch_num]]);
    }

    while(1)
    {
        int32_t cmd, ret;
        int64_t cmp3, cmp0;
        uint32_t freq, ctrl_cmd[3]={0};
        uint32_t loop_cnt;
        uint32_t ctrl_write_status, ctrl_reg_address, ctrl_reg_data = 0, ctrl_enc_id = 0;
        bissc_display_menu();
        cmd = bissc_get_command();
        if(cmd < BISSC_CMD_EXIT_APP)
        {
            continue;
        }
        else if(cmd == BISSC_CMD_EXIT_APP)
        {
            DebugP_log("\r\tGood bye!\n");
            break;
        }
        else if(cmd == BISSC_CMD_ENC_LEN_UPDATE)
        {
           bissc_get_enc_data_len(priv);
        }
        else if(cmd == BISSC_CMD_ENC_FREQ_UPDATE)
        {
            DebugP_log("\r\nPlease enter frequency in MHz:\n");
            DebugP_scanf("%u\n", &freq);
            if(!((freq == BISSC_FREQ_1MHZ) || (freq == BISSC_FREQ_2MHZ) || (freq == BISSC_FREQ_5MHZ) || (freq == BISSC_FREQ_8MHZ) || (freq == BISSC_FREQ_10MHZ)))
            {
                DebugP_log("\r\n CLK divisors will not be possible. Please provide valid freq: 1/2/5/8/10 \n");
                continue;
            }
            ret = bissc_clock_config(freq, priv);
            if(ret < 0)
            {
                DebugP_log("\r\n ERROR: Processing time measurement failed \n");
                DebugP_log("\r\n check whether encoder is connected and ensure proper connections \n");
                DebugP_log("\r\n Good bye!\n");
                break;
            }
            ClockP_sleep(2);
        }
        else if(cmd == BISSC_CMD_ENC_SEND_POS)
        {
            ret = bissc_get_pos(priv);
            if(ret < 0)
            {
                DebugP_log("\r\n ERROR: Position data measurement failed \n");
            }
            for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
            {
                ch = bissc_get_current_channel(priv, ch_num);
                if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                    ls_ch = ch;
                else
                    ls_ch = 0;
                DebugP_log("\r\n Channel %d:\n", ch);
                if(priv->multi_turn_len[ls_ch][0])
                {
                    DebugP_log("\r\n Encoder-1 Multiturn rev: %u, Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].num_of_turns[0],
                    priv->enc_pos_data[ch].angle[0], priv->enc_pos_data[ch].rcv_crc[0], priv->enc_pos_data[ch].otf_crc[0], priv->enc_pos_data[ch].ew[0]);
                }
                else
                {
                    DebugP_log("\r\n Encoder-1 Singleturn Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].angle[0], priv->enc_pos_data[ch].rcv_crc[0],
                    priv->enc_pos_data[ch].otf_crc[0], priv->enc_pos_data[ch].ew[0]);
                }
                DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (priv->enc_pos_data[ch].rcv_crc[0] == priv->enc_pos_data[ch].otf_crc[0]) ? "success" : "failure" ,
                priv->pd_crc_err_cnt[ch][0]);

                if(priv->data_len[ls_ch][1])
                {
                    if(priv->multi_turn_len[ls_ch][1])
                    {
                        DebugP_log("\r\n Encoder-2 Multiturn rev: %u, Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].num_of_turns[1],
                        priv->enc_pos_data[ch].angle[1], priv->enc_pos_data[ch].rcv_crc[1], priv->enc_pos_data[ch].otf_crc[1], priv->enc_pos_data[ch].ew[1]);
                    }
                    else
                    {
                        DebugP_log("\r\n Encoder-2 Singleturn Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].angle[1], priv->enc_pos_data[ch].rcv_crc[1],
                        priv->enc_pos_data[ch].otf_crc[1], priv->enc_pos_data[ch].ew[1]);
                    }
                    DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (priv->enc_pos_data[ch].rcv_crc[1] == priv->enc_pos_data[ch].otf_crc[1]) ? "success" : "failure" ,
                    priv->pd_crc_err_cnt[ch][1]);

                    if(priv->data_len[ls_ch][2])
                    {
                        if(priv->multi_turn_len[ls_ch][2])
                        {
                            DebugP_log("\r\n Encoder-3 Multiturn rev: %u, Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].num_of_turns[2],
                            priv->enc_pos_data[ch].angle[2], priv->enc_pos_data[ch].rcv_crc[2], priv->enc_pos_data[ch].otf_crc[2], priv->enc_pos_data[ch].ew[2]);
                        }
                        else
                        {
                            DebugP_log("\r\n Encoder-3 Singleturn Angle:  %.12f, crc: %x, otf crc: %x, e_w: %x\n", priv->enc_pos_data[ch].angle[2], priv->enc_pos_data[ch].rcv_crc[2],
                            priv->enc_pos_data[ch].otf_crc[2], priv->enc_pos_data[ch].ew[2]);
                        }
                        DebugP_log("\r\n CRC Status: %s, crc error count: %u\n", (priv->enc_pos_data[ch].rcv_crc[2] == priv->enc_pos_data[ch].otf_crc[2]) ? "success" : "failure" ,
                        priv->pd_crc_err_cnt[ch][2]);
                    }
                }
            }
        }
        else if(cmd == BISSC_CMD_ENC_CTRL_CMD)
        {
            DebugP_log("\r\nPlease enter control communication details:\n");
            totalchns = bissc_get_totalchannels(priv);
            for(ch_num = 0; ch_num < totalchns; ch_num++)
            {
                ctrl_reg_data = 0;
                ctrl_enc_id = 0;
                if(CONFIG_BISSC0_MODE == BISSC_MODE_MULTI_CHANNEL_MULTI_PRU)
                {
                    ls_ch = bissc_get_current_channel(priv, ch_num);
                    DebugP_log("\r\n Channel %d: ",ls_ch);
                }
                else
                {
                    ls_ch = 0;
                    totalchns = 1;
                }
                DebugP_log("\r\n Enter type of access(0: Read & 1: Write): ");
                DebugP_scanf("%x\n", &ctrl_write_status);
                while(1)
                {
                    DebugP_log("\r\n Enter Register Address: ");
                    DebugP_scanf("%x\n", &ctrl_reg_address);
                    if(ctrl_reg_address > 0x7F)
                        DebugP_log("\r\n Please enter a 7-bit address\n");
                    else
                        break;
                }
                if(ctrl_write_status == 1)
                {
                    DebugP_log("\r\n Enter Data to write at 0x%x Register\n", ctrl_reg_address);
                    DebugP_scanf("%x\n", &ctrl_reg_data);
                }
                if(priv->num_encoders[ls_ch] > 1)
                {
                    DebugP_log("\r\n Enter Encoder ID\n");
                    DebugP_scanf("%x\n", &ctrl_enc_id);
                }
                ctrl_cmd[ls_ch] = bissc_generate_ctrl_cmd(priv, ls_ch, ctrl_write_status, ctrl_reg_address, ctrl_reg_data, ctrl_enc_id);
            }
            ret = bissc_set_ctrl_cmd_and_process(priv, ctrl_cmd);
            if(ret < 0)
            {
                DebugP_log("\r\n ERROR: Control communication failed \n");
            }
            for(ch_num = 0; ch_num < priv->totalchannels; ch_num++)
            {
                ch = bissc_get_current_channel(priv, ch_num);
                DebugP_log("\r\n Channel %d:\n", ch);
                DebugP_log("\r\n Control communication result: %x, crc: %x, otf crc: %x, status: %s\n",priv->enc_ctrl_data[ch].cmd_result,
                priv->enc_ctrl_data[ch].cmd_rcv_crc, priv->enc_ctrl_data[ch].cmd_otf_crc,
                (priv->enc_ctrl_data[ch].cmd_rcv_crc == priv->enc_ctrl_data[ch].cmd_otf_crc) ? "success" : "failure");

                DebugP_log("\r\n CTRL CRC error count: %u\n", priv->ctrl_crc_err_cnt[ch]);
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
                    ret = bissc_get_pos(priv);
                    if(ret < 0)
                    {
                        DebugP_log("\r\n ERROR: Position data measurement for first encoder failed \n");
                    }
                    bissc_print_res(priv);
                    loop_cnt--;
                }while(loop_cnt);
            }
            else
            {
                DebugP_log("Please enter non-zero value\n");
            }
        }
        else if(cmd == BISSC_CMD_PERIODIC_TRIGGER)
        {
            DebugP_log("\r| Enter IEP cycle count(must be greater than BiSS cycle time including timeout period, in IEP cycles): ");
            DebugP_scanf("%lld\n", &cmp0);
            if(cmp0 <= IEP_DEFAULT_INC)
            {
                DebugP_log("\r\n| WARNING: invalid value entered\n");
                continue;
            }
            DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP cycle count, in IEP cycles): ");
            DebugP_scanf("%lld\n", &cmp3);
            if((cmp3 > cmp0) || (cmp3 <= IEP_DEFAULT_INC))
            {
                DebugP_log("\r\n| WARNING: invalid value entered\n");
                continue;
            }
            bissc_process_periodic_command(priv, cmp3, cmp0);
        }
    }
deinit:

    Board_driversClose();
    Drivers_close();
    return;
}
