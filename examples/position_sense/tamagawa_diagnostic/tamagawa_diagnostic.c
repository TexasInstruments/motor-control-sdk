/**
 * tamagawa_diagnostic.c
 *
 * Copyright (c) 2022-24, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */



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
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include <position_sense/tamagawa/include/tamagawa_drv.h>
#include "tamagawa_periodic_trigger.h"

#define TASK_STACK_SIZE (4096)
#define TASK_PRIORITY   (6)

/*Use soc driver instead it when available */
#if SOC_AM261X
/**
 *  \anchor TCA6408_Mode
 *  \name IO pin mode - Input or Output
 *  @{
 */
/** \brief Configure IO pin as input */
#define TCA6408_MODE_INPUT              (0U)
/** \brief Configure IO pin as output */
#define TCA6408_MODE_OUTPUT             (1U)
/** @} */

/**
 *  \anchor TCA6408_OutState
 *  \name IO pin output state - HIGH or LOW
 *  @{
 */
/** \brief Configure IO pin output as LOW */
#define TCA6408_OUT_STATE_LOW           (0U)
/** \brief Configure IO pin output as HIGH */
#define TCA6408_OUT_STATE_HIGH          (1U)
/** @} */

#define TCA6408_REG_INPUT_PORT_0        (0x00U)
#define TCA6408_REG_OUTPUT_PORT_0       (0x01U)
#define TCA6408_REG_POL_INV_PORT_0      (0x02U)
#define TCA6408_REG_CONFIG_PORT_0       (0x03U)

#define GPIO9_BIT_FOR_INPUT  (0x200)
#endif
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

#define TAMAGAWA_POSITION_LOOP_STOP    0
#define TAMAGAWA_POSITION_LOOP_START   1

#define  ICSSM_PRU_CORE_CLOCK 200000000

#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) == 1)
#include <position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h>
#endif
#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) > 1)
#include <position_sense/tamagawa/firmware/tamagawa_master_multi_channel_bin.h>
#endif

static uint8_t gTamagawa_multi_ch_mask;
static uint32_t gTamagawa_is_multi_ch;
struct tamagawa_priv *priv;

/** \brief Global Structure pointer holding PRU-ICSSx memory Map. */
PRUICSS_Handle gPruIcssXHandle;
void *gPru_dramx;

#if SOC_AM261X
I2C_Handle          i2cHandle;

int32_t TCA6408_open()
{
    int32_t status = SystemP_SUCCESS;

    i2cHandle = I2C_getHandle(CONFIG_I2C0);

    return (status);
}

int32_t TCA6408_config(uint32_t ioIndex, uint32_t mode)
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
        buffer[0] = TCA6408_REG_CONFIG_PORT_0 + port;
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
        if(TCA6408_MODE_INPUT == mode)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6408_REG_CONFIG_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 2;
        i2cTransaction.targetAddress = i2cAddress;
        status += I2C_transfer(i2cHandle, &i2cTransaction);
    }

    return (status);
}

int32_t TCA6408_setOutput(uint32_t ioIndex, uint32_t state)
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
        buffer[0] = TCA6408_REG_OUTPUT_PORT_0 + port;
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
        if(TCA6408_OUT_STATE_HIGH == state)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6408_REG_OUTPUT_PORT_0 + port;
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
    status = TCA6408_open();
    DebugP_assert(status == SystemP_SUCCESS);

    /* Configure pins 4, 5 and 7 as outputs */
    status = TCA6408_config(4, TCA6408_MODE_OUTPUT);
    DebugP_assert(status == SystemP_SUCCESS);
    status = TCA6408_config(5, TCA6408_MODE_OUTPUT);
    DebugP_assert(status == SystemP_SUCCESS);
    status = TCA6408_config(7, TCA6408_MODE_OUTPUT);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Set value 1 in pin 7 - BP Mux 0 */
    status = TCA6408_setOutput(7, TCA6408_OUT_STATE_HIGH);
    DebugP_assert(status == SystemP_SUCCESS);

     /* Set value 0 in pin 5 - BP Mux 1 */
    status = TCA6408_setOutput(5, TCA6408_OUT_STATE_LOW);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Set value 1 in pin 4 - Mux Enable */
    status = TCA6408_setOutput(4, TCA6408_OUT_STATE_HIGH);
    DebugP_assert(status == SystemP_SUCCESS);

}
#endif
void tamagawa_pruicss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    /* PRUICSS_PRUx holds value 0 or 1 depending on whether we are using PRU0 or PRU1 slice */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(PRUICSS_PRUx));
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
#if(SOC_AM243X || SOC_AM64X)
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif

#ifdef SOC_AM261X
    lp_bp_mux_mode_config();

    /* Set bits for input pins in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers */
#if (PRUICSS_PRUx == 1)
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM1_PRU0_GPIO_OUT_CTRL, GPIO9_BIT_FOR_INPUT);
#else
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM0_PRU0_GPIO_OUT_CTRL, GPIO9_BIT_FOR_INPUT);
#endif

    
#endif
}

void tamagawa_pruicss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcssXHandle, PRUICSS_PRUx);
    /*Load firmware. Set buffer = write to Pru memory */
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),0, (uint32_t *) TamagawaFirmware_0,sizeof(TamagawaFirmware_0));
    PRUICSS_resetCore(gPruIcssXHandle, PRUICSS_PRUx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcssXHandle, PRUICSS_PRUx);
}

void tamagawa_display_result(struct tamagawa_priv *priv, int32_t cmd)
{
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


static void tamagawa_handle_rx(struct tamagawa_priv *priv, int32_t cmd)
{
    DebugP_log("\r\n Parsing process started\n");
    /* Case of parsing failure */
    if (tamagawa_parse(cmd, priv) == -1)
    {
        DebugP_log("\r\n ERROR: Parsing failure\n");
        return;
    }
    /* Case of successful parsing, display the results after CRC check*/
    DebugP_log("\r\n Channel is  %x \n",priv->channel);
    DebugP_log("\r\n data id is %x \n",cmd);
    if (tamagawa_crc_verify(priv) == 1)
    {
        DebugP_log("\r\n CRC success \n");
        tamagawa_display_result(priv, cmd);
        return;
    }
    else
    {
        DebugP_log("\r\n CRC Failure \n");
    }

    return;
}

static enum data_id tamagawa_get_command(uint8_t *adf, uint8_t *edf)
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
        if(DebugP_scanf("%u\n", &priv->cmp0))
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            return cmd;
        }
        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles): ");
        if(DebugP_scanf("%u\n", &priv->cmp3) >= 0)
        {
            return cmd;
        }else{
            cmd = DATA_ID_0;
            DebugP_log("\r\n| WARNING: invalid value entered, Data readout Data ID 0 will be sent with host trigger mode\n");
        }
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
        uint32_t ch = 0;
        for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
        {
            if(gTamagawa_multi_ch_mask & 1 << ch)
            {
                if(gTamagawa_is_multi_ch != 1)
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

                if(val > MAX_EEPROM_ADDRESS)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM address\n|\n");
                    break;
                }

                *adf = (uint8_t)val;
                tamagawa_update_adf(priv, val, ch);
            }
        }
    }
    /* In case of EEPROM Write, take input for Address field for different channels selected*/
    if(cmd == DATA_ID_6)
    {
        uint32_t ch = 0;
        for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
        {
            if(gTamagawa_multi_ch_mask & 1 << ch)
            {
                if(gTamagawa_is_multi_ch != 1)
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

                if(val > MAX_EEPROM_WRITE_DATA)
                {
                    cmd = DATA_ID_NUM;
                    DebugP_log("\r\n| ERROR: invalid EEPROM data\n|\n");
                    break;
                }

                *edf = (uint8_t)val;
                tamagawa_update_edf(priv, val, ch);
            }
        }
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


uint32_t tamagawa_get_fw_version(void)
{
    /* Returns the firmware version, depending on Single or Multi-channel configuration */
    return *((uint32_t *)TamagawaFirmware_0 + 1);
}

static int32_t tamagawa_position_loop_status;

static void tamagawa_position_loop_decide_termination(void *args)
{
    char c;

    while(1)
    {
        DebugP_scanf("%c", &c);
        tamagawa_position_loop_status = TAMAGAWA_POSITION_LOOP_STOP;
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
        DebugP_log("\rTask2 creation failed\n");
    }

    return status ;
}

static void tamagawa_process_periodic_command(enum data_id process_dataid_cmd)
{
    int32_t status;

    tamagawa_config_periodic_trigger(priv);

    if(tamagawa_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return;
    }

    struct tamagawa_periodic_interface tamagawa_periodic_interface;
    tamagawa_periodic_interface.pruss_cfg = priv->pruss_cfg;
    tamagawa_periodic_interface.pruss_iep = priv->pruss_iep;
    tamagawa_periodic_interface.pruss_dmem = priv->tamagawa_xchg;
    tamagawa_periodic_interface.cmp3 = priv->cmp3;
     tamagawa_periodic_interface.cmp0 = priv->cmp0;

    status = tamagawa_config_periodic_mode(&tamagawa_periodic_interface, gPruIcssXHandle);
    DebugP_assert(0 != status);
    tamagawa_position_loop_status = TAMAGAWA_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");

    while(1)
    {
        if(tamagawa_position_loop_status == TAMAGAWA_POSITION_LOOP_STOP)
        {
            tamagawa_stop_periodic_continuous_mode(&tamagawa_periodic_interface);
            tamagawa_config_host_trigger(priv);
            return;
        }
        else
        {
            tamagawa_update_data_id(priv, process_dataid_cmd);

            /* In case of EEPROM commands, calculate the CRC for the different channels selected */
            if((process_dataid_cmd == DATA_ID_6) || (process_dataid_cmd == DATA_ID_D))
            {
                uint32_t ch = 0;
                for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
                {
                    if(gTamagawa_multi_ch_mask & 1 << ch)
                    {
                        tamagawa_update_crc(priv, process_dataid_cmd, ch);
                    }
                }

            }

            tamagawa_command_process(priv, process_dataid_cmd, gTamagawa_multi_ch_mask);

            if(gTamagawa_is_multi_ch)
            {
                DebugP_log("\r\n Multi-channel mode is enabled\n\n");

                uint32_t ch;
                for(ch = 0; ch < MAX_CHANNELS; ch++)
                {
                    if(gTamagawa_multi_ch_mask & 1 << ch)
                    {
                        tamagawa_multi_channel_set_cur(priv, ch);
                        DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                        tamagawa_handle_rx(priv, process_dataid_cmd);
                    }
                }
            }
            else
            {
                DebugP_log("\r\n Single-channel mode is enabled\n\n");
                tamagawa_handle_rx(priv, process_dataid_cmd);
            }
        }
    }
}

void tamagawa_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

/*C16 pin High for Enabling ch0 in booster pack */
#if (CONFIG_TAMAGAWA0_BOOSTER_PACK && CONFIG_TAMAGAWA0_CHANNEL0)
    GPIO_setDirMode(ENC1_EN_BASE_ADDR, ENC1_EN_PIN, ENC1_EN_DIR);
    GPIO_pinWriteHigh(ENC1_EN_BASE_ADDR, ENC1_EN_PIN);
#endif
/*B17 pin High for Enabling ch2 in booster pack */
#if (CONFIG_TAMAGAWA0_BOOSTER_PACK && CONFIG_TAMAGAWA0_CHANNEL2)
    GPIO_setDirMode(ENC2_EN_BASE_ADDR, ENC2_EN_PIN, ENC2_EN_DIR);
    GPIO_pinWriteHigh(ENC2_EN_BASE_ADDR, ENC2_EN_PIN);
#endif

    void *pruicss_cfg;
    void *pruicss_iep;
    uint32_t slice_value = 1;
    uint32_t selected_ch;

    tamagawa_pruicss_init();

    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->cfgRegBase);
    pruicss_iep  = (void *)(((PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->iep0RegBase);

    if(PRUICSS_PRUx == 0)
    {
        slice_value = 0;
    }

    /* Initialize the priv structure according to the PRUx slice selected */
#if PRUICSS_PRUx
    priv = tamagawa_init((struct tamagawa_xchg *)(
        (PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru1DramBase, pruicss_cfg,pruicss_iep,slice_value);
#else
    priv = tamagawa_init((struct tamagawa_xchg *)(
        (PRUICSS_HwAttrs *)(gPruIcssXHandle->hwAttrs))->pru0DramBase, pruicss_cfg, pruicss_iep, slice_value);
#endif

/* Get core clock value */
#if defined(SOC_AM263X) || defined(SOC_AM261X)
    /* fixed 200MHz pru core clock for ICSSM*/
    priv->pru_clock = ICSSM_PRU_CORE_CLOCK;
#else
    uint32_t status;
#if(PRUICSSx)
    status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG1, TISCI_DEV_PRU_ICSSG1_CORE_CLK, &priv->pru_clock);
    DebugP_assert(status == SystemP_SUCCESS);
#else
    status = SOC_moduleGetClockFrequency(TISCI_DEV_PRU_ICSSG0, TISCI_DEV_PRU_ICSSG0_CORE_CLK, &priv->pru_clock);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
#endif
    tamagawa_set_baudrate(priv, CONFIG_TAMAGAWA0_BAUDRATE);

    DebugP_log("\r\n\nTamagawa PRU-ICSS init done\n\n");

    /* Set the value of gTamagawa_multi_ch_mask based on the channels selected */
    gTamagawa_multi_ch_mask = (CONFIG_TAMAGAWA0_CHANNEL0<<0|CONFIG_TAMAGAWA0_CHANNEL1<<1|CONFIG_TAMAGAWA0_CHANNEL2<<2);

    if (CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2 > 1)
    {
        gTamagawa_is_multi_ch = 1;
    }

    if(gTamagawa_is_multi_ch)
    {
        tamagawa_config_multi_channel_mask(priv, gTamagawa_multi_ch_mask);
    }

    else
    {
        if (CONFIG_TAMAGAWA0_CHANNEL0 == 1)
        {
            selected_ch = 0;
        }

        if (CONFIG_TAMAGAWA0_CHANNEL1 == 1)
        {
            selected_ch = 1;
        }

        if (CONFIG_TAMAGAWA0_CHANNEL2 == 1)
        {
            selected_ch = 2;
        }
        tamagawa_config_channel(priv, selected_ch);
    }

    uint32_t firmware_ver;
    firmware_ver = tamagawa_get_fw_version();

    DebugP_log("\r\nTamagawa firmware \t: %x.%x.%x (%s)\n\n", (firmware_ver >> 24) & 0x7F,
                (firmware_ver >> 16) & 0xFF, firmware_ver & 0xFFFF, firmware_ver & (1 << 31) ? "internal" : "release");

    DebugP_log("\r\nChannel(s) selected: %s %s %s \n\n\n",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH0 ? "0" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH1 ? "1" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH2 ? "2" : "");

    /*Updating the channel mask in interface*/
    priv->tamagawa_xchg->tamagawa_interface.ch_mask =gTamagawa_multi_ch_mask;

    tamagawa_config_host_trigger(priv);

    tamagawa_pruicss_load_run_fw();
    DebugP_log("\r\nTamagawa PRU-ICSS firmware loaded and running\n\n\n");

    while(1)
    {
        /*
         * Initialized to zero to remove the compiler warning about the variable being uninitialized.
         */
        uint8_t adf = 0, edf = 0;
        enum data_id cmd;

        tamagawa_display_menu();
        cmd = tamagawa_get_command(&adf, &edf);
      

        if(cmd == PERIODIC_TRIGGER_CMD)
        {
            /*Takes which data_id to process in input arguments*/
            tamagawa_process_periodic_command(DATA_ID_0);
        }

        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        tamagawa_update_data_id(priv, cmd);

        /* In case of EEPROM commands, calculate the CRC for the different channels selected */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint32_t ch = 0;
            for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_update_crc(priv, cmd, ch);
                }
            }

        }

        tamagawa_command_process(priv, cmd, gTamagawa_multi_ch_mask);

        if(gTamagawa_is_multi_ch)
        {
            DebugP_log("\r\n Multi-channel mode is enabled\n\n");

            uint32_t ch;
            for(ch = 0; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_multi_channel_set_cur(priv, ch);
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    tamagawa_handle_rx(priv, cmd);
                }
            }
        }
        else
        {
            DebugP_log("\r\n Single-channel mode is enabled\n\n");
            tamagawa_handle_rx(priv, cmd);
        }
    }

    Board_driversClose();
    Drivers_close();
}
