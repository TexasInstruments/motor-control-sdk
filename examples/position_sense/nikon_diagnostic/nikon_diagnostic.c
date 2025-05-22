/*
 *  Copyright (C) 2024-25 Texas Instruments Incorporated
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

#if defined(SOC_AM243X) || defined(SOC_AM64X)
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

#define PRUICSS_SLICEx PRUICSS_PRUx

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
#if (PRUICSS_PRUx == 1)
#include  <nikon_receiver_multi_pru1_bin.h>
#else
#include  <nikon_receiver_multi_pru0_bin.h>
#endif
#endif

#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_MULTI_PRU )
#if (PRUICSS_PRUx == 1)
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
#if (PRUICSS_PRUx == 1)
#include  <nikon_receiver_pru1_bin.h>
#else
#include  <nikon_receiver_pru0_bin.h>
#endif
#endif
#define TASK_STACK_SIZE                     (4096)
#define TASK_PRIORITY                       (6)

#define NIKON_POSITION_LOOP_STOP            0
#define NIKON_POSITION_LOOP_START           1

#if defined(SOC_AM263PX) || defined(SOC_AM263X)
#define ICSS_PRU_CORE_CLOCK 200000000
#define ICSS_PRU_UART_CLOCK 192000000
#define NIKON_CH1_RX_GPIO_NUM   0x400
#else
#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define ICSS_PRU_UART_CLOCK CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ
#endif

/*Use soc driver instead it when available */
#if SOC_AM263PX
/**
 *  \anchor TCA6416_Mode
 *  \name IO pin mode - Input or Output
 *  @{
 */
/** \brief Configure IO pin as input */
#define TCA6416_MODE_INPUT              (0U)
/** \brief Configure IO pin as output */
#define TCA6416_MODE_OUTPUT             (1U)
/** @} */

/**
 *  \anchor TCA6416_OutState
 *  \name IO pin output state - HIGH or LOW
 *  @{
 */
/** \brief Configure IO pin output as LOW */
#define TCA6416_OUT_STATE_LOW           (0U)
/** \brief Configure IO pin output as HIGH */
#define TCA6416_OUT_STATE_HIGH          (1U)
/** @} */


#define TCA6416_REG_INPUT_PORT_0        (0x00U)
#define TCA6416_REG_INPUT_PORT_1        (0x01U)
#define TCA6416_REG_OUTPUT_PORT_0       (0x02U)
#define TCA6416_REG_OUTPUT_PORT_1       (0x03U)
#define TCA6416_REG_POL_INV_PORT_0      (0x04U)
#define TCA6416_REG_POL_INV_PORT_1      (0x05U)
#define TCA6416_REG_CONFIG_PORT_0       (0x06U)
#define TCA6416_REG_CONFIG_PORT_1       (0x07U)
#endif

/** \brief Global Structure pointer holding Nikon handle */
struct nikon_priv *priv;

uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

PRUICSS_Handle gPruIcssXHandle;
TaskP_Object gTaskObject;

static uint32_t nikon_position_loop_status;
uint32_t totalchannels = 0;
uint32_t mask = 0;

#if defined(SOC_AM263PX)
I2C_Handle          i2cHandle;

int32_t TCA6416_open()
{
    int32_t status = SystemP_SUCCESS;

    i2cHandle = I2C_getHandle(CONFIG_I2C0);

    return (status);
}

int32_t TCA6416_config(uint32_t ioIndex, uint32_t mode)
{

    int32_t         status = SystemP_SUCCESS;
    I2C_Transaction i2cTransaction;
    uint32_t        port, portPin, i2cAddress;
    uint8_t         buffer[2U] = {0};

    i2cAddress  = 0x20;

    if(status == SystemP_SUCCESS)
    {
        /* Each port contains 8 IOs */
        port        = 0;
        portPin     = ioIndex;

        /* Set config register address - needed for next read */
        I2C_Transaction_init(&i2cTransaction);
        buffer[0] = TCA6416_REG_CONFIG_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);

        /* Read config register value */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = buffer;
        i2cTransaction.readCount    = 1;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);

        /* Set output or input mode to particular IO pin - read/modify/write */
        I2C_Transaction_init(&i2cTransaction);
        if(TCA6416_MODE_INPUT == mode)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6416_REG_CONFIG_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 2;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);
    }

    return (status);
}

int32_t TCA6416_setOutput(uint32_t ioIndex, uint32_t state)
{
    int32_t         status = SystemP_SUCCESS;
    I2C_Transaction i2cTransaction;
    uint32_t        port, portPin, i2cAddress;
    uint8_t         buffer[2U] = {0};

    i2cAddress  = 0x20;

    if(status == SystemP_SUCCESS)
    {
        /* Each port contains 8 IOs */
        port        = 0;
        portPin     = ioIndex;

        /* Set output prt register address - needed for next read */
        I2C_Transaction_init(&i2cTransaction);
        buffer[0] = TCA6416_REG_OUTPUT_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);

        /* Read config register value */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = buffer;
        i2cTransaction.readCount    = 1;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);

        /* Set output or input mode to particular IO pin - read/modify/write */
        I2C_Transaction_init(&i2cTransaction);
        if(TCA6416_OUT_STATE_HIGH == state)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6416_REG_OUTPUT_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 2;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);
    }

    return (status);
}

void lp_bp_mux_mode_config()
{
    int32_t status = SystemP_FAILURE;
    status = TCA6416_open();
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure pins 6 and 7 as outputs */
    status = TCA6416_config(6, TCA6416_MODE_OUTPUT);
    DebugP_assert(status == SystemP_SUCCESS);
    status = TCA6416_config(7, TCA6416_MODE_OUTPUT);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Set value 1 in pin 7 - BP Mux 0 */
    status = TCA6416_setOutput(7, TCA6416_OUT_STATE_HIGH);
    DebugP_assert(status == SystemP_SUCCESS);

     /* Set value 1 in pin 6 - BP Mux 1 */
    status = TCA6416_setOutput(6, TCA6416_OUT_STATE_HIGH);
    DebugP_assert(status == SystemP_SUCCESS);
}
#endif

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

#if defined(SOC_AM263PX) ||  defined(SOC_AM263X)
#if defined(SOC_AM263PX)
    lp_bp_mux_mode_config();
#endif
    /* Set bits for input pins in ICSSM_PRU0_GPIO_OUT_CTRL register */
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU0_GPIO_OUT_CTRL, NIKON_CH1_RX_GPIO_NUM);
    status = SOC_moduleSetClockFrequency(SOC_RcmPeripheralId_ICSSM0_UART0, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, ICSS_PRU_UART_CLOCK);
    DebugP_assertNoLog(status == SystemP_SUCCESS);
#endif

}

int32_t nikon_pruicss_load_run_fw(struct nikon_priv *priv, uint8_t mask)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t size;
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
        DebugP_log("\r\nPlease enter 1st encoder single turn length: ");
        DebugP_scanf("%u\n", &single_turn_len[0]);
        num_encoders = 1;
        DebugP_log("\r\nPlease enter 1st encoder multi turn length (zero if not a multi turn encoder): ");
        DebugP_scanf("%u\n", &multi_turn_len[0]);
        DebugP_log("\r\nPlease enter 2nd encoder single turn length (zero if not connected): ");
        DebugP_scanf("%u\n", &single_turn_len[1]);
        if(single_turn_len[1])
        {
            DebugP_log("\r\nPlease enter 2nd encoder multi turn length (zero if not a multi turn encoder): ");
            DebugP_scanf("%u\n", &multi_turn_len[1]);
            num_encoders = 2;
            DebugP_log("\r\nPlease enter 3rd encoder single turn length (zero if not connected): ");
            DebugP_scanf("%u\n", &single_turn_len[2]);
            if(single_turn_len[2])
            {
                DebugP_log("Please enter 3rd encoder multi turn length (zero if not a multi turn encoder): ");
                DebugP_scanf("%u\n", &multi_turn_len[2]);
                num_encoders = 3;
            }
        }
        nikon_update_enc_len(priv, num_encoders, single_turn_len, multi_turn_len, ch);
    }
}

uint32_t nikon_get_fw_version(void)
{
#if (CONFIG_NIKON0_MODE == NIKON_MODE_MULTI_CHANNEL_SINGLE_PRU)
    return *((uint32_t *)NikonFirmwareMulti_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL0) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((uint32_t *)NikonFirmwareMultiMakeRTU_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL1) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((uint32_t *)NikonFirmwareMultiMakePRU_0 + 2);
#endif
#if (CONFIG_NIKON0_CHANNEL2) && (CONFIG_NIKON0_LOAD_SHARE_MODE)
    return *((uint32_t *)NikonFirmwareMultiMakeTXPRU_0 + 2);
#endif
#if (CONFIG_NIKON0_MODE == NIKON_MODE_SINGLE_CHANNEL_SINGLE_PRU)
    return *((uint32_t *)NikonFirmware_0 + 2);
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
        DebugP_log("\r\n| 23: ABS lower 24bit + velocity request (Individual)                                   |");
        DebugP_log("\r\n| 24: ABS lower 24bit + velocity request (Multiple)                                     |");
        DebugP_log("\r\n| 25: ABS lower 24bit + velocity + acceleration (Individual)                            |");
        DebugP_log("\r\n| 26: ABS lower 24bit + velocity + acceleration (Multiple)                              |");
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

static void nikon_process_periodic_command(struct nikon_priv *priv, int64_t cmp0, int64_t cmp3)
{
    int32_t status;
    int32_t ret;
    uint32_t ch_num;
    uint32_t ch;
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

    nikon_periodic_interface_init(priv, &nikon_periodic_interface, cmp0, cmp3);

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
    int32_t ret;
    uint32_t enc_addr = 0;
    uint32_t ch_num;
    uint32_t enc_num;
    uint32_t pru_num;
    uint32_t ls_ch;
    float_t freq;
    int64_t cmp3;
    int64_t cmp0;
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

    version = nikon_get_fw_version();

    DebugP_log("\r\nNIKON firmware \t: %x.%x.%x (%s)\n", (version >> 24) & 0x7F,
                (version >> 16) & 0xFF, version & 0xFFFF, version & (1 << 31) ? "internal" : "release");
    DebugP_log("\r\nNIKON Protocol Version selected\t: %s", (NIKON_PROTOCOL_VERSION == NIKON_PROTOCOL_V2_1)?"2.1":"3.0");

    nikon_pruicss_init();

    mask = CONFIG_NIKON0_CHANNEL0<<0 | CONFIG_NIKON0_CHANNEL1<<1 | CONFIG_NIKON0_CHANNEL2<<2;

    totalchannels = (CONFIG_NIKON0_CHANNEL0 + CONFIG_NIKON0_CHANNEL1 + CONFIG_NIKON0_CHANNEL2);

    DebugP_log("\r\n");

    icssClk = ICSS_PRU_CORE_CLOCK;
    uartClk = ICSS_PRU_UART_CLOCK;

    priv = nikon_init(gPruIcssXHandle, PRUICSS_PRUx, CONFIG_NIKON0_BAUDRATE, (uint32_t)icssClk, (uint32_t)uartClk, TX_RX_FIFO_CLOCK_SOURCE, mask, totalchannels, NIKON_PROTOCOL_VERSION);

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
        uint8_t access_with_bank = 0;
        uint8_t cmd_type;
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
                        DebugP_scanf("%c", &cmd_type);

                        if(cmd_type == '1')
                        {
                            break;
                        }
                        else if(cmd_type == '2')
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
                        DebugP_scanf("%c", &cmd_type);

                        if(cmd_type == '1')
                        {
                            break;
                        }
                        else if(cmd_type == '2')
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
                access_with_bank = 0;
                if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                {
                    while(1)
                    {
                        DebugP_log("\r\n Read with bank? (y/n) ");
                        DebugP_scanf("%c", &cmd_type);

                        if((cmd_type == 'y') || (cmd_type == 'Y'))
                        {
                            DebugP_log("\r\n Enter bank number: ");
                            DebugP_scanf("%x", &bank);

                            if(bank > 0xFF)
                            {
                                DebugP_log("\r\n Please enter a valid bank value\n");
                            }
                            else
                            {
                                nikon_update_bank(priv, (bank & 0xFF));
                                access_with_bank = 1;
                                break;
                            }
                        }
                        else if((cmd_type == 'n') || (cmd_type == 'N'))
                        {
                            break;
                        }
                        else
                        {
                            DebugP_log("\r\n ERROR: Invalid option, try again \n");
                        }
                    }
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
                        break;
                    }
                }

                nikon_update_eeprom_addr(priv, (addr & 0xFF));
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
                    if(access_with_bank == 1)
                    {
                        DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Bank: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], nikon_reverse_bits(priv->pos_data_info[ch].raw_data1[0], NIKON_RX_ONE_FRAME_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8, NIKON_EEPROM_BANK_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF), NIKON_EEPROM_ADDR_LEN));
                        if(priv->bank_error == 1)
                        {
                            DebugP_log("\r\n ERROR: Invalid bank number was specified \n");
                        }
                    }
                    else
                    {
                        DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], nikon_reverse_bits(priv->pos_data_info[ch].raw_data1[0], NIKON_RX_ONE_FRAME_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8, NIKON_EEPROM_ADDR_LEN));
                    }

                    DebugP_log("\r\n Received CRC: 0x%x, On-the-fly CRC: 0x%x, CRC Error Count: %u \n", priv->pos_data_info[ch].rcv_crc[0], priv->pos_data_info[ch].otf_crc[0], priv->pos_data_info[ch].crc_err_cnt[0]);
                    DebugP_log("\r\n Encoder Address: %u, Encoder Status: 0x%x, Command to Encoder: %u\n", priv->enc_info[ch].enc_addr[0], priv->enc_info[ch].enc_status[0], priv->enc_info[ch].enc_cmd[0]);
                    if((access_with_bank == 0) && (addr == 0xF9))
                    {
                        DebugP_log("\r\n Temperature: %d \n", priv->temperature[ch][0]);
                    }
                }
                break;

            case CMD_14:
                access_with_bank = 0;
                if (priv->protocol_version == NIKON_PROTOCOL_V3_0)
                {
                    while(1)
                    {
                        DebugP_log("\r\n Write with bank? (y/n) ");
                        DebugP_scanf("%c", &cmd_type);

                        if((cmd_type == 'y') || (cmd_type == 'Y'))
                        {
                            DebugP_log("\r\n Enter bank number: ");
                            DebugP_scanf("%x", &bank);

                            if(bank > 0xFF)
                            {
                                DebugP_log("\r\n Please enter a valid bank value\n");
                            }
                            else
                            {
                                access_with_bank = 1;
                                nikon_update_bank(priv, (bank & 0xFF));
                                break;
                            }
                        }
                        else if((cmd_type == 'n') || (cmd_type == 'N'))
                        {
                            break;
                        }
                        else
                        {
                            DebugP_log("\r\n ERROR: Invalid option, try again \n");
                        }
                    }
                }

                while(1)
                {
                    DebugP_log("\r\n Enter Memory location(in hex) to write(for bank read, 00h to FFh is valid; for non-bank read, 00h to EFh is valid): ");
                    DebugP_scanf("%x", &addr);
                    DebugP_log("\r\n Enter data(Bits [15:0] in hex) to write at Memory location 0x%x: ", addr);
                    DebugP_scanf("%x", &data);

                    if((data > 0xFFFF) || (addr > 0xFF) || ((access_with_bank == 0) && (addr > 0xEF)))
                    {
                        DebugP_log("\r\n Please enter a valid 8 bit value\n");
                    }
                    else
                    {
                        break;
                    }
                }
                nikon_update_eeprom_addr(priv, (addr & 0xFF));
                nikon_update_eeprom_data(priv, (data & 0xFFFF));

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

                    if(access_with_bank == 1)
                    {
                        DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Bank: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], nikon_reverse_bits(priv->pos_data_info[ch].raw_data1[0], NIKON_RX_ONE_FRAME_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8, NIKON_EEPROM_BANK_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF), NIKON_EEPROM_ADDR_LEN));

                        if(priv->bank_error == 1)
                        {
                            DebugP_log("\r\n ERROR: Invalid bank number was specified \n");
                        }
                    }
                    else
                    {
                        DebugP_log("\r\n Info Field: 0x%x, EEPROM Data: 0x%x, EEPROM Address: 0x%x \n", priv->pos_data_info[ch].raw_data0[0], nikon_reverse_bits(priv->pos_data_info[ch].raw_data1[0], NIKON_RX_ONE_FRAME_LEN), nikon_reverse_bits((priv->pos_data_info[ch].raw_data2[0] & 0xFF00) >> 8, NIKON_EEPROM_ADDR_LEN));
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
                        DebugP_scanf("%c", &cmd_type);

                        if(cmd_type == '1')
                        {
                            break;
                        }
                        else if(cmd_type == '2')
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
                        DebugP_scanf("%c", &cmd_type);

                        if(cmd_type == '1')
                        {
                            break;
                        }
                        else if(cmd_type == '2')
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
                            break;
                        }
                    }
                    nikon_update_velocity_coefficient(priv, data);
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
                            break;
                        }
                    }
                    nikon_update_id_code(priv, data);
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
                        DebugP_log("\r\n ERROR: Velocity/acceleration request failed\n");
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
                if(DebugP_scanf("%lld\n", &cmp0) < 0)
                {
                    DebugP_log("\r\n| WARNING: invalid value entered\n");
                    continue;
                }
                DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP cycle count, in nano seconds): ");
                DebugP_scanf("%lld\n", &cmp3);
                if((cmp3 > cmp0) || (cmp3 <= IEP_DEFAULT_INC))
                {
                    DebugP_log("\r\n| WARNING: invalid value entered\n");
                    continue;
                }
                nikon_process_periodic_command(priv, cmp0, cmp3);
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
