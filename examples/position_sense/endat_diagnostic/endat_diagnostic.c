/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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
#include <position_sense/endat/include/endat_drv.h>

#include "endat_periodic_trigger.h"

#if PRU_ICSSGx_PRU_SLICE
#define PRUICSS_PRUx PRUICSS_PRU1
#define PRUICSS_TXPRUx PRUICSS_TX_PRU1
#define PRUICSS_RTUPRUx PRUICSS_RTU_PRU1
#else
#define PRUICSS_PRUx PRUICSS_PRU0
#define PRUICSS_TXPRUx PRUICSS_TX_PRU0
#define PRUICSS_RTUPRUx PRUICSS_RTU_PRU0
#endif
#define PRUICSS_SLICEx PRU_ICSSGx_PRU_SLICE

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
#include  <position_sense/endat/firmware/endat_master_multi_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_RTU_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_PRU_bin.h>
#endif

#if (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
#include <position_sense/endat/firmware/endat_master_multi_TXPRU_bin.h>
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
#include <position_sense/endat/firmware/endat_master_bin.h>
#endif



#define WAIT_5_SECOND  (5000)
#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY   (6)

#define ENDAT_RX_SAMPLE_SIZE    7
#define ENDAT_RX_SESQUI_DIV (1 << 15)

#define MRS_POS_VAL2_WORD1  0x42
#define MRS_POS_VAL2_WORD2  0x43
#define MRS_POS_VAL2_WORD3  0x44

/* Translate the TCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))


static union endat_format_data gEndat_format_data_mtrctrl[3];
static uint32_t gEndat_mtrctrl_crc_err[3];
static uint32_t gEndat_2_2_crc_position_err_cnt[3];
static uint32_t gEndat_2_2_crc_addinfo1_err_cnt[3];
static uint64_t gEndat_2_2_pos_val2[3];
static int32_t gEndat_2_2_loop_mrs;
static void (*endat_fn_position_loop)(unsigned int);

uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;
#define VALID_PERIODIC_CMD(x) ((x) == 200)

#define VALID_HOST_CMD(x) ((x == 100) || ((x) == 101) || ((x) == 102) || ((x) == 103) || ((x) == 104) || ((x) == 105) || \
                           ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109) || ((x) == 110) || ((x) == 111))

#define HAVE_COMMAND_SUPPLEMENT(x) (((x) == 2) || ((x) == 3) || ((x) == 4) || ((x) == 7) || \
                                    ((x) == 9) || ((x) == 10) || ((x) == 11) || ((x) == 13) || ((x) == 14) || \
                                    ((x) == 100) || ((x) == 101) || ((x)== 103) || ((x) == 105) || ((x) == 106) || ((x) == 107) || ((x) == 108) || ((x) == 109)  || ((x) == 200))

#define ENDAT_INPUT_CLOCK_UART_FREQUENCY 192000000
/* use uart clock only to start with */
#define ENDAT_INPUT_CLOCK_FREQUENCY ENDAT_INPUT_CLOCK_UART_FREQUENCY

#define ENDAT_POSITION_LOOP_STOP    0
#define ENDAT_POSITION_LOOP_START   1

   
union position
{
    float angle;
    uint64_t length;
};

struct endat_priv *priv;

#define PRUSS_PRU1_CTRL_CTPPR1      (CSL_ICSS_G_PR1_PDSP1_IRAM_REGS_BASE + 0x2C)

#define ENDAT_MULTI_CH0 (1 << 0)
#define ENDAT_MULTI_CH1 (1 << 1)
#define ENDAT_MULTI_CH2 (1 << 2)
static uint32_t gEndat_prop_delay[3];
static uint32_t gEndat_prop_delay_max;
static uint8_t gEndat_is_multi_ch;
static uint8_t gEndat_multi_ch_mask;
static uint8_t  gEndat_is_load_share_mode;
static char gUart_buffer[256];

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcssXHandle;

/* buffer to handle long long printf */
char gPrintf_dump_buffer[21];

/* This function is used to convert uint64_t
 * variable tp a string as printf doesn't support
 * printing 64-bit variables
 */

/* EnDat channel Info, written by PRU cores */
__attribute__((section(".gEnDatChInfo"))) struct endatChRxInfo gEndatChInfo;
char * uint64_to_str (uint64_t x)
{
    char *b = gPrintf_dump_buffer + sizeof(gPrintf_dump_buffer);
    *(--b) = '\0';
    do
    {
        *(--b) = '0' + (x % 10);
        x /= 10;
    } while (x);

    return b;
}


static void endat_pruss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
     /* Configure g_mux_en to 1 in ICSSG_SA_MX_REG Register. */
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
    if(gEndat_is_load_share_mode)
    {
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_RTUPRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));
        /*Set in constant table C29 for  tx pru*/
        PRUICSS_setConstantTblEntry(gPruIcssXHandle, PRUICSS_TXPRUx, PRUICSS_CONST_TBL_ENTRY_C28, 0x258);

    }
     /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_SLICEx));
    if(gEndat_is_load_share_mode)
    {
        PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
        PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
    }
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);


}

void endat_pre_init(void)
{
    endat_pruss_init();
}

uint32_t endat_pruss_load_run_fw(struct endat_priv *priv)
{

    uint32_t status = SystemP_FAILURE;

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU /*enable loadshare mode*/



            status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status=PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_RTU_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) EnDatFirmwareMultiMakeRTU_0,
                                                        sizeof(EnDatFirmwareMultiMakeRTU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_RTUPRUx);
            DebugP_assert(SystemP_SUCCESS == status);


            status=PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx );
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                                      0, (uint32_t *) EnDatFirmwareMultiMakePRU_0,
                                                      sizeof(EnDatFirmwareMultiMakePRU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
            DebugP_assert(SystemP_SUCCESS == status);


           status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
             DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_writeMemory(gPruIcssXHandle,  PRUICSS_IRAM_TX_PRU(PRUICSS_SLICEx),
                                                        0, (uint32_t *) EnDatFirmwareMultiMakeTXPRU_0,
                                                        sizeof(EnDatFirmwareMultiMakeTXPRU_0));
            DebugP_assert(0 != status);
            status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_TXPRUx);
            DebugP_assert(SystemP_SUCCESS == status);
            status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_TXPRUx);
            DebugP_assert(SystemP_SUCCESS == status);


        status=endat_wait_initialization(priv, WAIT_5_SECOND, gEndat_multi_ch_mask);


#else

        status = PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

#if(CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU)

            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                0, (uint32_t *) EnDatFirmwareMulti_0,
                                sizeof(EnDatFirmwareMulti_0));

#else

            status = PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_SLICEx),
                                0, (uint32_t *) EnDatFirmware_0,
                                sizeof(EnDatFirmware_0));
#endif
        DebugP_assert(0 != status);

        status = PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

         /*Run firmware */
        status = PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
        DebugP_assert(SystemP_SUCCESS == status);

        /* check initialization ack from firmware, with a timeout of 5 second */
        status = endat_wait_initialization(priv, WAIT_5_SECOND, gEndat_multi_ch_mask);
#endif

    return status;
}

uint64_t endat_get_fw_version(void)
{
#if CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_SINGLE_PRU
    return *((unsigned long *)EnDatFirmwareMulti_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL0) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakeRTU_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL1) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakePRU_0 + 2);
#endif

#if (CONFIG_ENDAT0_CHANNEL2) && (CONFIG_ENDAT0_MODE == ENDAT_MODE_MULTI_CHANNEL_MULTI_PRU)
    return *((unsigned long *)EnDatFirmwareMultiMakeTXPRU_0 + 2);
#endif

#if CONFIG_ENDAT0_MODE == ENDAT_MODE_SINGLE_CHANNEL_SINGLE_PRU
    return *((unsigned long *)EnDatFirmware_0 + 2);
#endif

}

static void endat_print_menu(void)
{
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
                    gEndat_is_multi_ch || gEndat_is_load_share_mode ? "        " : "(safety)");
    }

    DebugP_log("\r|108: Configure propagation delay (tD)                                         |\n");

    if((gEndat_is_multi_ch) || (gEndat_is_load_share_mode))
    {
        DebugP_log("\r|109: Configure wire delay                                                     |\n");
    }

    DebugP_log("\r|110: Recovery Time (RT)                                                       |\n");
    DebugP_log("\r|111: Simulate motor control 2.1 position loop for long time                   |\n");
    DebugP_log("\r|200: Start periodic continuous mode                                           |\n");

    DebugP_log("\r|------------------------------------------------------------------------------|\n\r|\n");
    DebugP_log("\r| enter value: ");
}

static inline int32_t endat_get_command(void)
{
    volatile int32_t cmd = -1;


    if(DebugP_scanf("%d", &cmd) < 0)
    {
        cmd = -1;
    }

    if(VALID_2_1_CMD(cmd) || (priv->cmd_set_2_2 && VALID_2_2_CMD(cmd))
            || VALID_HOST_CMD(cmd) || VALID_PERIODIC_CMD(cmd))
    {
        return cmd;
    }

    DebugP_log("\r| WARNING: invalid command, EnDat 2.1 send position values command will be sent\n");
    return 1;
}

static void endat_recvd_print(int32_t cmd, struct endat_priv *priv,
                              union endat_format_data *u, int32_t crc)
{
    uint32_t addinfo, byte1;
    uint64_t max = pow(2, priv->single_turn_res);
    union position position;

    /* this would give wrong values if cmd is not position related, but that is okay as then this value won't be used */
    if(priv->type == rotary)
    {
        position.angle = ((float) u->position_addinfo.position.position) /
                         (float)max * (float)360;
    }

    else
    {
        position.length = u->position_addinfo.position.position * priv->step;
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
            if(priv->multi_turn_res)
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

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
            if(priv->multi_turn_res)
            {
                sprintf(gUart_buffer, "\r| position: %.12f, revolution: %s, ",
                        position.angle, uint64_to_str(u->position_addinfo.position.revolution));
            }
            else
            {
                if(priv->type == rotary)
                {
                    sprintf(gUart_buffer, "\r| position: %.12f ", position.angle);
                }
                else
                {
                    sprintf(gUart_buffer, "\r| position: %s ", uint64_to_str(position.length));
                }
            }

            DebugP_log("%s", gUart_buffer);

            DebugP_log("f1/f2: %x/%x, crc: %s\n", u->position_addinfo.position.f1,
                        u->position_addinfo.position.f2,
                        crc & 0x1 ? "success" : "failure");

            DebugP_logInfo("| crc: %x\n", u->position_addinfo.position.crc);

            if(priv->flags.info1)
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

            if(priv->flags.info2)
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

static void endat_display_raw_data(int32_t cmd, struct endat_priv *priv)
{
    int32_t ch = priv->channel;
    struct endatChRxInfo *endatChRxInfo = priv->endatChRxInfo;

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
                        endatChRxInfo->ch[ch].posWord0, endatChRxInfo->ch[ch].posWord1,
                        endatChRxInfo->ch[ch].posWord2, endatChRxInfo->ch[ch].posWord3);
            break;

        default:
            DebugP_log("\r|\n| Nothing raw to display - this is not a valid command\n|\n");
            break;
    }
}

/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called after endat_get_command
 */
static int32_t endat_get_command_supplement(int32_t cmd,
                                        struct cmd_supplement *cmd_supplement)
{
    /* erase previous cmd supplements */
    memset(cmd_supplement, 0, sizeof(*cmd_supplement));

    switch(cmd)
    {
        case 2:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 9:
            DebugP_log("\r| enter MRS code (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid MRS code\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address == ENDAT_SECTION2_MEMORY)
            {
                DebugP_log("\r| enter block address (hex value): ");

                if(DebugP_scanf("%x\n", &cmd_supplement->block) < 0)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                /* better compare it with number of blocks information available in eeprom */
                if(cmd_supplement->block > 0xFF)
                {
                    DebugP_log("\r| ERROR: invalid block address\n|\n|\n|\n");
                    return -EINVAL;
                }

                cmd_supplement->has_block_address = TRUE;
            }

            break;

        case 3:
        case 10:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter parameter (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid parameter\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 4:
        case 11:
            DebugP_log("\r| enter parameter address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid parameter address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 7:
        case 13:
            DebugP_log("\r| enter port address (hex value):");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("| ERROR: invalid port address\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 14:
            DebugP_log("\r| enter encoder address (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->address > 0xFF)
            {
                DebugP_log("\r| ERROR: invalid encoder address\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter instruction (hex value): ");

            if(DebugP_scanf("%x\n", &cmd_supplement->data) < 0)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            if(cmd_supplement->data > 0xFFFF)
            {
                DebugP_log("\r| ERROR: invalid instruction\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 100:
        case 101:
        case 107:
            DebugP_log("\r| enter frequency in Hz: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid frequency\n|\n|\n|\n");
                return -EINVAL;
            }

            break;

        case 103:
            DebugP_log("\r| enter tST delay counter value in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &priv->channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<priv->channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
            break;

        case 105:
            DebugP_log("\r| enter rx arm counter in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &priv->channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<priv->channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
            break;


        case 106:
            DebugP_log("\r| enter rx clock disable time (for tD) in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &priv->channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<priv->channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
            break;

        case 108:
            DebugP_log("\r| enter propagation delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                DebugP_log("\r| Select Channel: ");
                if(DebugP_scanf("%u\n", &priv->channel) < 0)
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }

                if(!((gEndat_multi_ch_mask) & (1<<priv->channel)))
                {
                    DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                    return -EINVAL;
                }
            }

            break;

        case 109:
            DebugP_log("\r| enter channel number: ");

            if(DebugP_scanf("%x\n", &cmd_supplement->address) < 0)
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }

            if(!(gEndat_multi_ch_mask & 1 << cmd_supplement->address))
            {
                DebugP_log("\r| ERROR: invalid channel\n|\n|\n|\n");
                return -EINVAL;
            }

            DebugP_log("\r| enter wire delay in ns: ");

            if(DebugP_scanf("%u\n", &cmd_supplement->frequency) < 0)
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }

            break;
        case 200:
        
           
            DebugP_log("\r| Enter IEP reset cycle count (must be greater than EnDat cycle time including timeout period, in IEP cycles): ");
            if(DebugP_scanf("%u\n", &cmd_supplement->cmp0))
            {
                DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                return -EINVAL;
            }
           
            if(gEndat_is_load_share_mode)
            {
   
                if(gEndat_multi_ch_mask & (1<<0))
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel0: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->cmp3) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }

                if(gEndat_multi_ch_mask & (1<<1))
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel1: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->cmp5) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }
                if(gEndat_multi_ch_mask & (1<<2))
                {
                    DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles) Channel2: \n");
                    if(DebugP_scanf("%u\n", &cmd_supplement->cmp6) < 0)
                    {
                        DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                        return -EINVAL;
                    }
                }
                
            }
            else
            {
                DebugP_log("\r| Enter IEP trigger time (must be less than or equal to IEP reset cycle, in IEP cycles): ");
                if(DebugP_scanf("%u\n", &cmd_supplement->cmp3) < 0)
                {
                    DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
                    return -EINVAL;
                }
            }
            break;

        default:
            cmd = -EINVAL;
            DebugP_log("\r| ERROR: no command data required for the command\n");
            break;
    }

    return cmd;
}

static int32_t endat_handle_user(struct cmd_supplement *cmd_supplement)
{
    int32_t cmd;

    endat_print_menu();
    cmd = endat_get_command();

    if(HAVE_COMMAND_SUPPLEMENT(cmd))
    {
        return endat_get_command_supplement(cmd, cmd_supplement);
    }

    return cmd;
}

static int32_t endat_calc_clock(uint32_t freq, struct endat_clk_cfg *clk_cfg)
{
    uint32_t ns;

    if(freq > 16000000 || (freq > 12000000 && freq < 16000000))
    {
        DebugP_log("\r| ERROR: frequency above 16MHz, between 12 & 16MHz not allowed\n|\n|\n");
        return -1;
    }

    if((freq != 16000000) && (ENDAT_INPUT_CLOCK_FREQUENCY % (freq * 8)))
        DebugP_log("\r| WARNING: exact clock divider is not possible, frequencies set would be tx: %u\trx: %u\n",
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / freq),
                    ENDAT_INPUT_CLOCK_FREQUENCY / (ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8)));

    ns = 2 * 1000000000 / freq; /* rx arm >= 2 clock */

    /* should be divisible by 5 */
    if(ns % 5)
    {
        ns /= 5, ns += 1,  ns *= 5;
    }

    clk_cfg->tx_div = ENDAT_INPUT_CLOCK_FREQUENCY / freq - 1;
    clk_cfg->rx_div = ENDAT_INPUT_CLOCK_FREQUENCY / (freq * 8) - 1;
    clk_cfg->rx_en_cnt = ns;
    clk_cfg->rx_div_attr = ENDAT_RX_SAMPLE_SIZE;

    if(freq == 16000000)
    {
        clk_cfg->rx_div_attr |= ENDAT_RX_SESQUI_DIV;
    }

    DebugP_logInfo("\r| clock config values - tx_div: %u\trx_div: %u\trx_en_cnt: %u\trx_div_attr: %x\n",
                    clk_cfg->tx_div, clk_cfg->rx_div, clk_cfg->rx_en_cnt, clk_cfg->rx_div_attr);

    return 0;
}

static uint32_t endat_do_sanity_tst_delay(uint32_t delay)
{
    /* (unsigned short)~0 is also a multiple of 5 */
    if(delay > (unsigned short)~0)
    {
        DebugP_log("\r| ERROR: delay greater than %uns, enter lesser value\n|\n|\n",
                    (unsigned short)~0);
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

static int32_t endat_position_loop_status;

/* position period in microsecond */
static int32_t endat_calc_position_period(uint32_t freq)
{
    /* 16KHz limitation due to the timer */
    if(freq > 16000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 16KHz\n|\n|\n");
        return -1;
    }
    else if((gEndat_is_multi_ch || gEndat_is_load_share_mode)&& freq > 8000)
    {
        DebugP_log("\r| ERROR: enter frequency less than or equal 8KHz in multi channel configuration\n|\n|\n");
        return -1;
    }

    return 1000000 / freq;
}

static void endat_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        endat_position_loop_status = ENDAT_POSITION_LOOP_STOP;
        break;
    }
    TaskP_exit();
}

void endat_process_position_command(uint32_t a0)
{
    uint32_t crc;

    endat_command_process(priv, 1, NULL);
    endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(priv, 1, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_mtrctrl_crc_err[a0]++;
    }
}

uint16_t _endat_process_2_2_position_command(int32_t cmd,
        struct cmd_supplement *cmd_supplement, uint32_t a0)
{
    uint32_t crc;

    endat_command_process(priv, cmd, cmd_supplement);
    endat_recvd_process(priv, cmd, &gEndat_format_data_mtrctrl[a0]);
    crc = endat_recvd_validate(priv, cmd, &gEndat_format_data_mtrctrl[a0]);

    if(!(crc & 0x1))
    {
        gEndat_2_2_crc_position_err_cnt[a0]++;
    }

    if(priv->flags.info1 && !(crc & 0x2))
    {
        gEndat_2_2_crc_addinfo1_err_cnt[a0]++;
    }

    endat_addinfo_track(priv, cmd, cmd_supplement);

    return gEndat_format_data_mtrctrl[a0].position_addinfo.addinfo1.addinfo & 0xFFFF;
}

void endat_process_2_2_position_command(uint32_t a0)
{
    uint32_t cmd;
    struct cmd_supplement cmd_supplement;
    uint16_t pos_word;

    if(((!gEndat_is_multi_ch || !gEndat_is_load_share_mode) && priv->has_safety) )
    {
        cmd = 9, cmd_supplement.address = gEndat_2_2_loop_mrs;
    }
    else
    {
        cmd = 8;
    }

    pos_word = _endat_process_2_2_position_command(cmd, &cmd_supplement, a0);

    if((gEndat_is_multi_ch) || (!priv->has_safety) || (gEndat_is_load_share_mode))
    {
        return;
    }

    /* WORD3 in addinfo1 */
    if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD1)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFF0000FFFFFFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 32;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD2;
        /* WORD1 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD2)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFFFFFF0000;
        gEndat_2_2_pos_val2[a0] |= (uint64_t)pos_word;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD3;
        /* WORD2 in addinfo1 */
    }
    else if(gEndat_2_2_loop_mrs == MRS_POS_VAL2_WORD3)
    {
        gEndat_2_2_pos_val2[a0] &= 0xFFFFFFFF0000FFFF;
        gEndat_2_2_pos_val2[a0] |= (unsigned long long)pos_word << 16;
        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;
    }
}

void endat_position_loop(uint32_t a0)
{
    if(endat_fn_position_loop != NULL)
    {
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_fn_position_loop(j);
                }
            }
        }
        else
        {
            endat_fn_position_loop(0);
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
        DebugP_log("\rTask2 creation failed\n");
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

static int32_t endat_get_position_loop_chars(struct endat_priv *priv,
        int32_t continuous, int32_t is_2_2)
{
    int32_t i;

    if(priv->multi_turn_res)
    {
        i = 34;
    }
    else
    {
        i = 16;
    }

    if(!continuous)
    {
        i += 12;
    }

    i += 4;

    if(!continuous && is_2_2)
    {
        i += 4;
    }

    return i;
}

static void endat_print_position_loop(struct endat_priv *priv, int32_t continuous,
                                      int32_t is_2_2, int32_t ch)
{
    uint64_t max = pow(2, priv->single_turn_res);
    union position position;

    if(priv->type == rotary)
    {
        position.angle = ((float)
                          gEndat_format_data_mtrctrl[ch].position_addinfo.position.position) /
                         (float)max * (float)360;
    }
    else
    {
        position.length =
            gEndat_format_data_mtrctrl[ch].position_addinfo.position.position * priv->step;
    }

    /* max value is 2x48, has 15 digits, so 16 is safe */
    if(priv->multi_turn_res)
    {
        sprintf(gUart_buffer, "%16.12f, %16s", position.angle,
                uint64_to_str(gEndat_format_data_mtrctrl[ch].position_addinfo.position.revolution));
    }
    else
    {
        if(priv->type == rotary)
        {
            sprintf(gUart_buffer, "%16.12f", position.angle);
        }
        else
        {
            sprintf(gUart_buffer, "%16s", uint64_to_str(position.length));
        }
    }

    DebugP_log("\r%s", gUart_buffer);

    if(!continuous)
    {
        if(is_2_2)
        {
            DebugP_log(", %10u", gEndat_2_2_crc_position_err_cnt[ch]);
        }
        else
        {
            DebugP_log(", %10u", gEndat_mtrctrl_crc_err[ch]);
        }
    }

    DebugP_log(",%3u",
                gEndat_format_data_mtrctrl[ch].position_addinfo.position.f1);

    if(!continuous && is_2_2)
    {
        DebugP_log(",%3u",
                    gEndat_format_data_mtrctrl[ch].position_addinfo.position.f2);
    }
}

static void endat_print_position_loop_channel_info(struct endat_priv *priv,
        int32_t is_2_2)
{
    int32_t k, j, i = endat_get_position_loop_chars(priv, 0, is_2_2);

    /* add 3 extra to account for spacing b/n channels */
    i += 3;
    /* find mid point */
    i /= 2;
    /* account for "CHANNEL x" display */
    i -= 4;

    for(k = i, j = 0; j < 3; j++, k = i)
        if(gEndat_multi_ch_mask & 1 << j)
        {
            while(k--)
            {
                DebugP_log("%c", ' ');
            }

            DebugP_log("\rCHANNEL %d", j);

            /* 3 - extra to account for spacing b/n channels, 9 - "CHANNEL x" length */
            k = endat_get_position_loop_chars(priv, 0, is_2_2) + 3 - i - 9;

            while(k--)
            {
                DebugP_log("%c", ' ');
            }
        }
}

static void endat_handle_prop_delay(struct endat_priv *priv,
                                    uint16_t prop_delay)
{
    float ct = (priv->rx_en_cnt)/2; /*one endat clock cycle time = 1/endat frequency = 2*rx_en_cnt*/
    /* if propagation delay is more than half clock cycle time (2/endat frequency) then we have to reduce clock cycles for rx*/
    if(prop_delay > (ct/2))
    {
        uint16_t dis = round(prop_delay/ct);
        endat_config_rx_arm_cnt(priv, prop_delay);
        /* propagation delay/cycle_time */
        endat_config_rx_clock_disable(priv, dis);
    }
    else
    {
        endat_config_rx_arm_cnt(priv, priv->rx_en_cnt);
        endat_config_rx_clock_disable(priv, 0);
    }
}

static void endat_process_periodic_command(int32_t cmd,
                                       struct cmd_supplement *cmd_supplement, struct endat_priv *priv)
{
    if(cmd == 200)
    {
        endat_config_periodic_trigger(priv);
        int32_t status;
        int32_t pos_cmd = 1;
        DebugP_assert(endat_command_process(priv, pos_cmd, NULL) >= 0);
        priv->cmp0 = cmd_supplement->cmp0;
        if(gEndat_is_load_share_mode)
        {
            priv->cmp3 = gEndat_multi_ch_mask & (1<<0) ? cmd_supplement->cmp3 : 0;
            priv->cmp5 = gEndat_multi_ch_mask & (1<<1) ? cmd_supplement->cmp5 : 0;
            priv->cmp6 = gEndat_multi_ch_mask & (2<<1) ? cmd_supplement->cmp6 : 0;
        }
        else
        {
            priv->cmp3 = cmd_supplement->cmp3; 
        }

        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
            DebugP_log("Task_create() failed!\n");
            return;
        }

        struct endat_periodic_interface endat_periodic_interface;
        endat_periodic_interface.pruss_iep = priv->pruss_iep;
        endat_periodic_interface.pruss_dmem = priv->pruss_xchg;
        endat_periodic_interface.load_share = priv->load_share;
        endat_periodic_interface.cmp3 = priv->cmp3;
        endat_periodic_interface.cmp5 = priv->cmp5;
        endat_periodic_interface.cmp6 = priv->cmp6;
        endat_periodic_interface.cmp0 = priv->cmp0;
        
        status = endat_config_periodic_mode(&endat_periodic_interface, gPruIcssXHandle);
        DebugP_assert(0 != status);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;
    
        if(priv->multi_turn_res)
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position,       revolution, f1\r\n| ");
        }
        else
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");
        }
    
        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                endat_stop_periodic_continuous_mode(&endat_periodic_interface);
                endat_config_host_trigger(priv);
                return;
            }
            else
            {
                int32_t i;

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;
                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(priv, 0, 0);
                            DebugP_log("| ");
                            DebugP_log("\r|\n\r| Channel: %10u\r\n| ", j);
                            endat_print_position_loop(priv, 1, 0, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {        
                    endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[0]);

                    i = endat_get_position_loop_chars(priv, 1, 0);
                    endat_print_position_loop(priv, 1, 0, 0);
                }
                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    
    
    }
}
static void endat_process_host_command(int32_t cmd,
                                       struct cmd_supplement *cmd_supplement, struct endat_priv *priv)
{
    struct endat_clk_cfg clk_cfg;
    static int32_t timer_init;

    /* clock configuration */
    if(cmd == 100)
    {
        if(endat_calc_clock(cmd_supplement->frequency, &clk_cfg) < 0)
        {
            return;
        }

        endat_config_clock(priv, &clk_cfg);

        priv->rx_en_cnt = clk_cfg.rx_en_cnt;

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            uint16_t d;

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_handle_prop_delay(priv, gEndat_prop_delay[priv->channel]);
                    d = gEndat_prop_delay_max - gEndat_prop_delay[j];
                    endat_config_wire_delay(priv, d);
                }
            }
        }
        else
        {
            endat_handle_prop_delay(priv, gEndat_prop_delay[priv->channel]);
        }

        /* set tST to 2us if frequency > 1MHz, else turn it off */
        if(cmd_supplement->frequency >= 1000000)
        {
            cmd_supplement->frequency = 2000;
        }
        else
        {
            cmd_supplement->frequency = 0;
        }

        /* control loop */
        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;
            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    endat_process_host_command(103, cmd_supplement, priv);
                }
           }
        }
        else
        {
            endat_process_host_command(103, cmd_supplement, priv);
        }

    }
    else if(cmd == 101)
    {
        int32_t us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }

        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing position loop as related Task creation failed\n|\n|\n");
            DebugP_log("\rTask_create() failed!\n");
            return;
        }

        endat_fn_position_loop = endat_process_position_command;

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        if(priv->multi_turn_res)
        {
            DebugP_log("\r|\r\n| press enter to stop the position display|\n");

            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 0);
                DebugP_log("\r\n|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position,       revolution, crc errors, f1\n");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("\r|         position,       revolution, crc errors, f1|\r\n ");
            }
        }
        else
        {
            DebugP_log("|\n| press enter to stop the position display\n|\n");

            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 0);
                DebugP_log("\r\n|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position, crc errors, f1\n");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("|         position, crc errors, f1\n| ");
            }
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);
                return;
            }
            else
            {
                int32_t i;

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            i += endat_get_position_loop_chars(priv, 0, 0);
                            DebugP_log("| ");
                            endat_print_position_loop(priv, 0, 0, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(priv, 0, 0);
                    endat_print_position_loop(priv, 0, 0, 0);
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(500);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }

    else if(cmd == 102)
    {
        priv->raw_data ^= 1;
    }
    else if(cmd == 103)
    {
        uint32_t delay;

        delay = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(delay <= (uint16_t)~0)
        {
            endat_config_tst_delay(priv, (uint16_t) delay);
        }
    }
    else if(cmd == 104)
    {

        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
            DebugP_log("Task_create() failed!\n");
            return;
        }

        endat_start_continuous_mode(priv);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        if(priv->multi_turn_res)
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position,       revolution, f1\r\n| ");
        }
        else
        {
            DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                endat_stop_continuous_mode(priv);
                return;
            }
            else
            {
                int32_t i;

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;
                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[j]);
                            i += endat_get_position_loop_chars(priv, 0, 0);
                            DebugP_log("| ");
                            DebugP_log("\r|\n\r| Channel: %10u\r\n| ", j);
                            endat_print_position_loop(priv, 1, 0, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {        
                    endat_recvd_process(priv, 1, &gEndat_format_data_mtrctrl[0]);

                    i = endat_get_position_loop_chars(priv, 1, 0);
                    endat_print_position_loop(priv, 1, 0, 0);
                }
                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }
    else if(cmd == 105)
    {
        uint32_t val;

        /* reuse tST delay sanity check */
        val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val <= (uint16_t)~0)
        {
            endat_config_rx_arm_cnt(priv, (uint16_t)val);
        }
    }
    else if(cmd == 106)
    {
        uint16_t dis = cmd_supplement->frequency * 2 / priv->rx_en_cnt;

        endat_config_rx_clock_disable(priv, dis);
    }
    else if(cmd == 108)
    {
        /* reuse tST delay sanity check */
        uint32_t val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        if(val > (uint16_t)~0)
        {
            return;
        }
        endat_handle_prop_delay(priv, (uint16_t)val);

    }
    else if(cmd == 109)
    {
        /* reuse tST delay sanity check */
        uint32_t val = endat_do_sanity_tst_delay(cmd_supplement->frequency);

        endat_multi_channel_set_cur(priv, cmd_supplement->address);
        endat_config_wire_delay(priv, val);
    }
    else if(cmd == 107)
    {
        int32_t us = endat_calc_position_period(cmd_supplement->frequency);

        if(us < 0)
        {
            return;
        }
        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing position loop as related Task creation failed\n|\n|\n");
            DebugP_log("\rTask_create() failed!\n");
            return;
        }

        endat_fn_position_loop = endat_process_2_2_position_command;

        if(!timer_init)
        {
            endat_loop_timer_create(us);
            timer_init = 1;
        }

        /* reset additional info's if present */
        endat_command_process(priv, 5, NULL);
        endat_addinfo_track(priv, 5, NULL);

        gEndat_2_2_loop_mrs = MRS_POS_VAL2_WORD1;

        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        /* so that proper position value 2 is displayed from the begining */
        ClockP_usleep(us * 3);

        if((!gEndat_is_multi_ch && !priv->has_safety) || (!gEndat_is_load_share_mode && !priv->has_safety))
        {
            DebugP_log("\r|\n| encoder does not support safety, position value 2 would not be displayed\n|\n");
        }

        DebugP_log("\r|\n\r| press enter to stop the position display\n\r|\n");

        if(priv->multi_turn_res)
        {
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 1);
                DebugP_log("\n\r|\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("\r| ");
                        DebugP_log("        position,       revolution, crc errors, f1, f2");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\r\n");
            }
            else
            {
                DebugP_log("\r|         position,       revolution, crc errors, f1, f2");

                if(priv->has_safety)
                {
                    DebugP_log(",      position(2),    revolution(2), crc errors(2)");
                }
                DebugP_log("\r\n| ");
            }
        }
        else
        {
            if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
            {
                int32_t j;

                DebugP_log("\r|\n\r|");
                endat_print_position_loop_channel_info(priv, 1);
                DebugP_log("\r\n|\r\n");

                for(j = 0; j < 3; j++)
                {
                    if(gEndat_multi_ch_mask & 1 << j)
                    {
                        DebugP_log("| ");
                        DebugP_log("        position, crc errors, f1");
                        DebugP_log(" ");
                    }
                }
                DebugP_log("\n");
            }
            else
            {
                DebugP_log("|         position, crc errors, f1, f2 ");

                if(priv->has_safety)
                {
                    DebugP_log(",      position(2), crc errors(2)");
                }
                DebugP_log("\r\n| ");
            }
        }

        while(1)
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {
                TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

                /* reset additional info1 */
                endat_command_process(priv, 5, NULL);
                endat_addinfo_track(priv, 5, NULL);
                return;
            }
            else
            {
                int32_t i;

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(i = 0, j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            i += endat_get_position_loop_chars(priv, 0, 1);
                            DebugP_log("| ");
                            endat_print_position_loop(priv, 0, 1, j);
                            DebugP_log(" ");
                            DebugP_log("\n");
                            i += 3;
                        }
                    }
                }
                else
                {
                    i = endat_get_position_loop_chars(priv, 0, 1);
                    endat_print_position_loop(priv, 0, 1, 0);
                }

                if((!gEndat_is_multi_ch && priv->has_safety) || (!gEndat_is_load_share_mode && priv->has_safety))
                {
                    uint64_t multi_turn, single_turn;
                    union position position2;
                    uint64_t max = pow(2, priv->single_turn_res);

                    multi_turn = ENDAT_GET_POS_MULTI_TURN(gEndat_2_2_pos_val2[0], priv);
                    single_turn = ENDAT_GET_POS_SINGLE_TURN(gEndat_2_2_pos_val2[0], priv);

                    if(priv->type == rotary)
                    {
                        position2.angle = (float)single_turn / (float)max * (float)360;
                    }
                    else
                    {
                        position2.length = single_turn * priv->step;
                    }

                    DebugP_log(", ");

                    if(priv->multi_turn_res)
                    {
                        sprintf(gUart_buffer, "%16.12f, %16s", position2.angle, uint64_to_str(multi_turn));
                    }
                    else
                    {
                        if(priv->type == rotary)
                        {
                            sprintf(gUart_buffer, "%16.12f", position2.angle);
                        }
                        else
                        {
                            sprintf(gUart_buffer, "%16s", uint64_to_str(position2.length));
                        }
                    }

                    DebugP_log("%s", gUart_buffer);

                    DebugP_log(",    %10u", gEndat_2_2_crc_addinfo1_err_cnt[0]);
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(100);

                if((!gEndat_is_multi_ch && priv->has_safety) || (!gEndat_is_load_share_mode && priv->has_safety))
                {
                    if(priv->multi_turn_res)
                    {
                        i += 2 + 46 + 3;
                    }
                    else
                    {
                        i += 2 + 28 + 3;
                    }
                }

                while(i--)
                {
                    DebugP_log("%c", 8);
                }
            }
    }
    else if(cmd == 110)
    {
        uint32_t recovery_time;
        if(gEndat_is_multi_ch||gEndat_is_load_share_mode)
        {
            int32_t j;
            for( j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    DebugP_log("channel: %d",priv->channel);
                    DebugP_log("\t");
                    recovery_time = endat_get_recovery_time(priv);
                    DebugP_log("Recovery Time: %d ns", recovery_time);
                    DebugP_log("\n");
                }
            }
        }
        else
        {
            recovery_time = endat_get_recovery_time(priv);
            DebugP_log("Recovery Time: %d ns", recovery_time);
            DebugP_log("\n");
        }

    }
    else if(cmd == 111)
    {   
        DebugP_log("\r|press enter to stop the long time continuous mode\n|");
        if(endat_loop_task_create() != SystemP_SUCCESS)
        {
            DebugP_log("\r| ERROR: OS not allowing position loop as related Task creation failed\n|\n|\n");
            DebugP_log("\rTask_create() failed!\n");
            return;
        }

        endat_position_loop_status = ENDAT_POSITION_LOOP_START;

        uint64_t poaition_read = 0;
        /*clear CRC error count */
        memset(gEndat_mtrctrl_crc_err, 0, 3);
        while(1)
        {
            if(endat_position_loop_status == ENDAT_POSITION_LOOP_STOP)
            {

                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                           DebugP_log("\r -------Channel %u ------\n", j);
                           DebugP_log("\r position command sent = %u \n", poaition_read);
                           DebugP_log("\r CRC failures encountered = %u \n", gEndat_mtrctrl_crc_err[j]);
                           DebugP_log(" ");
                           DebugP_log("\r");
                        }
                    }
                }
                else
                {
                    DebugP_log("\r position command sent = %u \n", poaition_read);
                    DebugP_log("\r CRC failures encountered = %u \n", gEndat_mtrctrl_crc_err[0]);
                    DebugP_log(" ");
                    DebugP_log("\r");
                }
                
                return;
            }
            else
            {
                poaition_read++;
                if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
                {
                    int32_t j;

                    for(j = 0; j < 3; j++)
                    {
                        if(gEndat_multi_ch_mask & 1 << j)
                        {
                            endat_multi_channel_set_cur(priv, j);
                            endat_process_position_command(j);
                        }
                    }
                }
                else
                {
                    endat_process_position_command(0);
                }

                /* increase sleep value if glitches in display to be prevented (and would result in slower position display freq) */
                ClockP_usleep(500);

            }
        }
    }
    else
    {
        DebugP_log("\r| ERROR: non host command being requested to be handled as host command\n|\n|\n");
    }
}

static void endat_handle_rx(struct endat_priv *priv, int32_t cmd)
{
    uint32_t crc;
    union endat_format_data endat_format_data;

    if(priv->raw_data)
    {
        endat_display_raw_data(cmd, priv);
    }

    endat_recvd_process(priv, cmd,  &endat_format_data);
    crc = endat_recvd_validate(priv, cmd, &endat_format_data);
    endat_recvd_print(cmd, priv, &endat_format_data, crc);

    return;
}

static void endat_print_encoder_info(struct endat_priv *priv)
{
    DebugP_log("EnDat 2.%d %s encoder\tID: %u %s\tSN: %c %u %c\n\n",
                priv->cmd_set_2_2 ? 2 : 1,
                (priv->type == rotary) ? "rotary" : "linear",
                priv->id.binary, (char *)&priv->id.ascii,
                (char)priv->sn.ascii_msb, priv->sn.binary, (char)priv->sn.ascii_lsb);
    DebugP_log("\rPosition: %d bits ", priv->pos_res);

    if(priv->type == rotary)
    {
        DebugP_log("(singleturn: %d, multiturn: %d) ", priv->single_turn_res,
                    priv->multi_turn_res);
    }

    DebugP_log("[resolution: %d %s]", priv->step,
                priv->type == rotary ? "M/rev" : "nm");
    DebugP_log("\r\n\nPropagation delay: %dns",
                gEndat_prop_delay[priv->channel]);
    DebugP_log("\n\n\n");
}

void endat_main(void *args)
{
    int32_t i;
    struct cmd_supplement cmd_supplement;

    uint64_t icssgclk;

    void *pruss_cfg;
    void *pruss_iep;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

/*C16 pin High for Enabling ch0 in booster pack */
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL0)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
/*B17 pin High for Enabling ch2 in booster pack */
#if(CONFIG_ENDAT0_BOOSTER_PACK && CONFIG_ENDAT0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif

    i = endat_get_fw_version();

    DebugP_log("\n\n\n");
    DebugP_log("EnDat firmware \t: %x.%x.%x (%s)\n\n", (i >> 24) & 0x7F,
                (i >> 16) & 0xFF, i & 0xFFFF, i & (1 << 31) ? "internal" : "release");

    gEndat_is_multi_ch = CONFIG_ENDAT0_MODE & 1;
    gEndat_is_load_share_mode = CONFIG_ENDAT0_MODE & 2;
    endat_pre_init();


    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {

        gEndat_multi_ch_mask=(CONFIG_ENDAT0_CHANNEL0<<0|CONFIG_ENDAT0_CHANNEL1<<1|CONFIG_ENDAT0_CHANNEL2<<2);

        DebugP_log("\r\nchannels %s %s %s selected\n",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH0 ? "0" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH1 ? "1" : "",
                    gEndat_multi_ch_mask & ENDAT_MULTI_CH2 ? "2" : "");

        if(!gEndat_multi_ch_mask)
        {
            DebugP_log("\r\nERROR: please select channels to be used in multi channel configuration -\n\n");
            DebugP_log("\rexit %s as no channel selected in multichannel configuration\n",
                          __func__);
            return;
        }
    }
    else
    {

        i = CONFIG_ENDAT0_CHANNEL0 & 0;

        i += CONFIG_ENDAT0_CHANNEL1;

        i += CONFIG_ENDAT0_CHANNEL2<<1;

        if(i < 0 || i > 2)
        {
           DebugP_log("\r\nWARNING: invalid channel selected, defaulting to Channel 0\n");
           i = 0;
        }
    }

    DebugP_log("\r\n\n");

    /*Translate the TCM local view addr to globel view addr */
    uint64_t gEndatChInfoGlobalAddr = CPU0_BTCM_SOCVIEW((uint64_t)&gEndatChInfo);


    pruss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    pruss_iep  = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);

    #if PRU_ICSSGx_PRU_SLICE
        priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru1DramBase, &gEndatChInfo, gEndatChInfoGlobalAddr, pruss_cfg, pruss_iep, PRUICSS_SLICEx);

    #else
        priv = endat_init((struct endat_pruss_xchg *)((PRUICSS_HwAttrs *)(
                          gPruIcssXHandle->hwAttrs))->pru0DramBase, &gEndatChInfo, gEndatChInfoGlobalAddr,  pruss_cfg, pruss_iep, PRUICSS_SLICEx);
    #endif




    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        endat_config_multi_channel_mask(priv, gEndat_multi_ch_mask, gEndat_is_load_share_mode);
    }
    else
    {
        endat_config_channel(priv, i);
    }

    endat_config_host_trigger(priv);
    /* Read the ICSSG configured clock frequency. */
    if(gPruIcssXHandle->hwAttrs->instance)
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, &icssgclk);
    }
    else
    {
        SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &icssgclk);
    }

    /* Configure Delays based on the ICSSG frequency*/
    /* Count = ((required delay * icssgclk)/1000) */
    priv->pruss_xchg->endat_delay_125ns = ((icssgclk*125)/1000000000);
    priv->pruss_xchg->endat_delay_51us = ((icssgclk*51)/1000000 );
    priv->pruss_xchg->endat_delay_5us = ((icssgclk*5)/1000000);
    priv->pruss_xchg->endat_delay_1ms = ((icssgclk/1000) * 1);
    priv->pruss_xchg->endat_delay_2ms = ((icssgclk/1000) * 2);
    priv->pruss_xchg->endat_delay_12ms = ((icssgclk/1000) * 12);
    priv->pruss_xchg->endat_delay_50ms = ((icssgclk/1000) * 50);
    priv->pruss_xchg->endat_delay_380ms = ((icssgclk/1000) * 380);
    priv->pruss_xchg->endat_delay_900ms = ((icssgclk/1000) * 900);
    priv->pruss_xchg->icssg_clk = icssgclk;
    

    i = endat_pruss_load_run_fw(priv);

    if(i < 0)
    {
        DebugP_log("\rERROR: EnDat initialization failed -\n\n");

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            uint8_t tmp;

            tmp = endat_multi_channel_detected(priv) & gEndat_multi_ch_mask;
            tmp ^= gEndat_multi_ch_mask;
            DebugP_log("\r\tunable to detect encoder in channel %s %s %s\n",
                        tmp & ENDAT_MULTI_CH0 ? "0" : "",
                        tmp & ENDAT_MULTI_CH1 ? "1" : "",
                        tmp & ENDAT_MULTI_CH2 ? "2" : "");
        }
        else
        {
            DebugP_log("\r\tcheck whether encoder is connected and ensure proper connections\n");
        }

        DebugP_log("\rexit %s due to failed firmware initialization\n", __func__);
        return;
    }

    /* read encoder info at low frequency so that cable length won't affect */
    cmd_supplement.frequency = 200 * 1000;
    endat_process_host_command(100, &cmd_supplement, priv);

    if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
    {
        int32_t j;

        for(j = 0; j < 3; j++)
        {
            if(gEndat_multi_ch_mask & 1 << j)
            {
                endat_multi_channel_set_cur(priv, j);
                if(endat_get_encoder_info(priv) < 0)
                {
                    DebugP_log("\rEnDat initialization channel %d failed\n", j);
                    DebugP_log("\rexit %s due to failed initialization\n", __func__);
                    return;
                }
                /*convert cnt to time in ns ((cnt*1000000000)/icssgclk) before use*/
                gEndat_prop_delay[priv->channel] = endat_get_prop_delay(priv)*((float)(1000000000)/icssgclk);
                DebugP_log("\n\t\t\t\tCHANNEL %d\n\n", j);
                endat_print_encoder_info(priv);
            }
        }

        gEndat_prop_delay_max = gEndat_prop_delay[0] > gEndat_prop_delay[1] ?
                               gEndat_prop_delay[0] : gEndat_prop_delay[1];
        gEndat_prop_delay_max = gEndat_prop_delay_max > gEndat_prop_delay[2] ?
                               gEndat_prop_delay_max : gEndat_prop_delay[2];

    }
    else
    {
        if(endat_get_encoder_info(priv) < 0)
        {
            DebugP_log("\rEnDat initialization failed\n");
            DebugP_log("\rexit %s due to failed initialization\n", __func__);
            return;
        }
        /*convert cnt to time in ns ((cnt*1000000000)/icssgclk) before use*/
        gEndat_prop_delay[priv->channel] = endat_get_prop_delay(priv)*((float)(1000000000)/icssgclk);

        endat_print_encoder_info(priv);
    }

    /* default frequency - 8MHz for 2.2 encoders, 1MHz for 2.1 encoders */
    if(priv->cmd_set_2_2)
    {
        cmd_supplement.frequency = 8 * 1000 * 1000;
    }
    else
    {
        cmd_supplement.frequency = 1 * 1000 * 1000;
    }

    endat_process_host_command(100, &cmd_supplement, priv);

    while(1)
    {
        int32_t cmd;

        cmd = endat_handle_user(&cmd_supplement);

        if(cmd < 0)
        {
            continue;
        }

        if(VALID_HOST_CMD(cmd))
        {
            endat_process_host_command(cmd, &cmd_supplement, priv);
            DebugP_log("\r|\n\r|\n");
            continue;
        }

        if(VALID_PERIODIC_CMD(cmd))
        {
            endat_process_periodic_command(cmd, &cmd_supplement, priv);
            DebugP_log("\r|\n\r|\n");
            continue;
        }

        if(endat_command_process(priv, cmd, &cmd_supplement) < 0)
        {
            continue;
        }

        if(gEndat_is_multi_ch || gEndat_is_load_share_mode)
        {
            int32_t j;

            DebugP_log("\r|\n");

            for(j = 0; j < 3; j++)
            {
                if(gEndat_multi_ch_mask & 1 << j)
                {
                    endat_multi_channel_set_cur(priv, j);
                    DebugP_log("\r|\n|\t\t\t\tCHANNEL %d\n", j);
                    endat_handle_rx(priv, cmd);
                }
            }
        }
        else
        {
            endat_handle_rx(priv, cmd);
        }

        /* this cannot be done except as last in loop; additional info becomes applicable from next command onwards only */
        endat_addinfo_track(priv, cmd, &cmd_supplement);
    }

    Board_driversClose();
    Drivers_close();
}

