/**
 * tamagawa_diagnostic.c
 *
 * Copyright (c) 2022-25, Texas Instruments Incorporated
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

#if defined(SOC_AM243X) || defined(SOC_AM64X)
#include <drivers/sciclient.h>
#endif

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
uint32_t gTaskFxnStack[TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));
TaskP_Object gTaskObject;

#define TAMAGAWA_POSITION_LOOP_STOP    0
#define TAMAGAWA_POSITION_LOOP_START   1

#define ICSS_PRU_CORE_CLOCK CONFIG_PRU_ICSS0_CORE_CLK_FREQ_HZ
#define ICSS_PRU_UART_CLOCK CONFIG_PRU_ICSS0_UART_CLK_FREQ_HZ
#define ICSS_PRU_IEP_CLOCK  CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ

#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) == 1)
#if CONFIG_TAMAGAWA0_PRUICSS_PRUx == 1
#include <tamagawa_receiver_single_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_single_channel_pru0_bin.h>
#endif
#endif
#if ((CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2) > 1)
#if CONFIG_TAMAGAWA0_PRUICSS_PRUx == 1
#include <tamagawa_receiver_multi_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_multi_channel_pru0_bin.h>
#endif
#endif

#define TAMAGAWA0_PRUICSS_SLICEx CONFIG_TAMAGAWA0_PRUICSS_PRUx

#if (CONFIG_TAMAGAWA0_PRUICSS_PRUx == 1)
#define TAMAGAWA0_PRUICSS_PRUx PRUICSS_PRU1
#else
#define TAMAGAWA0_PRUICSS_PRUx PRUICSS_PRU0
#endif

/*define macros for dual channels*/
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)

#define TAMAGAWA1_PRUICSS_SLICEx CONFIG_TAMAGAWA1_PRUICSS_PRUx

#if (TAMAGAWA1_PRUICSS_SLICEx == 1)
#define TAMAGAWA1_PRUICSS_PRUx PRUICSS_PRU1
#else
#define TAMAGAWA1_PRUICSS_PRUx PRUICSS_PRU0
#endif

Tamagawa_Handle gTamagawaHandle2;

#if TAMAGAWA1_PRUICSS_SLICEx == 1
#include <tamagawa_receiver_single_channel_pru1_bin.h>
#else
#include <tamagawa_receiver_single_channel_pru0_bin.h>
#endif

uint8_t gTamagawa1_multi_ch_mask;
uint8_t gTamagawa1_is_multi_ch;

struct tamagawa_periodic_interface tamagawa1_periodic_interface;
#endif

uint8_t gTamagawa_multi_ch_mask;
uint32_t gTamagawa_is_multi_ch;
Tamagawa_Handle gTamagawaHandle1;

/** \brief Global Structure pointer holding PRU-ICSSx memory Map. */
PRUICSS_Handle gPruIcssXHandle;

struct tamagawa_periodic_interface tamagawa_periodic_interface;

void tamagawa_pruicss_init(void)
{
    gPruIcssXHandle = PRUICSS_open(CONFIG_PRU_ICSS0);
    /* TAMAGAWA0_PRUICSS_SLICEx holds value 0 or 1 depending on whether we are using PRU0 or PRU1 slice */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(TAMAGAWA0_PRUICSS_SLICEx));
    PRUICSS_disableCore(gPruIcssXHandle, TAMAGAWA0_PRUICSS_SLICEx);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    /* CONFIG_TAMAGAWA1_PRUICSS_SLICEx holds value 0 or 1 depending on whether we are using PRU0 or PRU1 slice */
    PRUICSS_initMemory(gPruIcssXHandle, PRUICSS_DATARAM(TAMAGAWA1_PRUICSS_SLICEx));
    PRUICSS_disableCore(gPruIcssXHandle, TAMAGAWA1_PRUICSS_SLICEx);
#endif

#ifdef CONFIG_TAMAGAWA0_G_MUX_EN
    PRUICSS_setSaMuxMode(gPruIcssXHandle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
#endif

}

void tamagawa_pruicss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcssXHandle, TAMAGAWA0_PRUICSS_SLICEx);
    /*Load firmware. Set buffer = write to Pru memory */
#if TAMAGAWA0_PRUICSS_SLICEx == 1
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA0_PRUICSS_PRUx),0, (uint32_t *) TamagawaFirmwarePru1_0,sizeof(TamagawaFirmwarePru1_0));
#else
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(CONFIG_TAMAGAWA0_PRUICSS_PRUx),0, (uint32_t *) TamagawaFirmwarePru0_0,sizeof(TamagawaFirmwarePru0_0));
#endif
    PRUICSS_resetCore(gPruIcssXHandle, TAMAGAWA0_PRUICSS_SLICEx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcssXHandle, TAMAGAWA0_PRUICSS_SLICEx);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    PRUICSS_disableCore(gPruIcssXHandle, TAMAGAWA1_PRUICSS_SLICEx);
    /*Load firmware. Set buffer = write to Pru memory */
#if TAMAGAWA1_PRUICSS_SLICEx == 1
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(TAMAGAWA1_PRUICSS_SLICEx),0, (uint32_t *) TamagawaFirmwarePru1_0,sizeof(TamagawaFirmwarePru1_0));
#else
    PRUICSS_writeMemory(gPruIcssXHandle, PRUICSS_IRAM_PRU(TAMAGAWA1_PRUICSS_SLICEx),0, (uint32_t *) TamagawaFirmwarePru0_0,sizeof(TamagawaFirmwarePru0_0));
#endif
    PRUICSS_resetCore(gPruIcssXHandle, TAMAGAWA1_PRUICSS_SLICEx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcssXHandle, TAMAGAWA1_PRUICSS_SLICEx);

#endif
}

void tamagawa_display_result(Tamagawa_Handle handle, int32_t cmd)
{
    /* Prints the position value returned by the encoder for a particular command ID */
    switch(cmd)
    {
        case DATA_ID_7:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_8:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_C:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_0:
            /* Data readout: data in one revolution */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABM: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nENID: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tENID: 0x%x\tABM: 0x%x\tALMC: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abs, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.enid, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.abm, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.almc, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.sf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.edf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.adf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.cf, handle->tamagawa_xchg->tamagawa_interface.rx_frames_received.crc);
            break;

        default:
            DebugP_log("\r\n| ERROR: unknown Data ID\n");
            break;
    }
}


static void tamagawa_handle_rx(Tamagawa_Handle handle, int32_t cmd)
{
    DebugP_log("\r\n Parsing process started\n");
    /* Case of parsing failure */
    if (tamagawa_parse(cmd, handle) == -1)
    {
        DebugP_log("\r\n ERROR: Parsing failure\n");
        return;
    }
    /* Case of successful parsing, display the results after CRC check*/
    DebugP_log("\r\n Channel is  %x \n",handle->channel);
    DebugP_log("\r\n data id is %x \n",cmd);
    if (tamagawa_crc_verify(handle) == 1)
    {
        DebugP_log("\r\n CRC success \n");
        tamagawa_display_result(handle, cmd);
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
        if(DebugP_scanf("%u\n", &tamagawa_periodic_interface.iep_reset_count) < 0)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            DebugP_log("\r\n| WARNING: invalid Data \n");
            return -1;
        }
        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles): ");
        if(DebugP_scanf("%u\n", &tamagawa_periodic_interface.periodic_trigger_count) < 0 )
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            DebugP_log("\r\n| WARNING: invalid Data \n");
            return -1;
        }
        if(tamagawa_periodic_interface.periodic_trigger_count > tamagawa_periodic_interface.iep_reset_count)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            DebugP_log("\r\n| WARNING: invalid Data\n");
            return -1;
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        DebugP_log("\r| Enter IEP trigger time(must be less than or equal to IEP reset cycle, in IEP cycles) for 2nd channel: ");
        if(DebugP_scanf("%u\n", &tamagawa1_periodic_interface.periodic_trigger_count) < 0 )
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            DebugP_log("\r\n| WARNING: invalid Data\n");
            return -1;
        }
        if(tamagawa1_periodic_interface.periodic_trigger_count > tamagawa_periodic_interface.iep_reset_count)
        {
            DebugP_log("\r| ERROR: invalid value\n|\n|\n|\n");
            DebugP_log("\r\n| WARNING: invalid Data \n");
            return -1;
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
                tamagawa_update_adf(gTamagawaHandle1, val, ch);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE) 
                tamagawa_update_adf(gTamagawaHandle2, val, ch);
#endif
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
                tamagawa_update_edf(gTamagawaHandle1, val, ch);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE) 
                tamagawa_update_edf(gTamagawaHandle2, val, ch);
#endif
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
#if TAMAGAWA0_PRUICSS_SLICEx == 1
    return *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    return *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif

}
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
uint32_t tamagawa1_get_fw_version(void)
{
    /* Returns the firmware version, depending on Single or Multi-channel configuration */
#if TAMAGAWA1_PRUICSS_SLICEx == 1
    return *((uint32_t *)TamagawaFirmwarePru1_0 + 1);
#else
    return *((uint32_t *)TamagawaFirmwarePru0_0 + 1);
#endif

}
#endif
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

void tamagawa_process_periodic_command(Tamagawa_Handle handle1, enum data_id process_dataid_cmd
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        , Tamagawa_Handle handle2
#endif
    )
    {
    int32_t status;
    
    tamagawa_config_periodic_trigger(handle1);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_config_periodic_trigger(handle2);
#endif
    
    if(tamagawa_loop_task_create() != SystemP_SUCCESS)
    {
        DebugP_log("\r| ERROR: OS not allowing continuous mode as related Task creation failed\r\n|\r\n|\n");
        DebugP_log("Task_create() failed!\n");
        return;
    }
    
    status = tamagawa_config_periodic_mode(&tamagawa_periodic_interface, gPruIcssXHandle, handle1->instance_index);
    DebugP_assert(0 != status);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    status = tamagawa_config_periodic_mode(&tamagawa1_periodic_interface, gPruIcssXHandle, handle2->instance_index);
    DebugP_assert(0 != status);
#endif
    tamagawa_position_loop_status = TAMAGAWA_POSITION_LOOP_START;

    DebugP_log("\r|\n\r| press enter to stop the continuous mode\r\n|\r\n|         position, f1\r\n| ");

    tamagawa_update_data_id(handle1, process_dataid_cmd);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_update_data_id(handle2, process_dataid_cmd);
#endif
    
    /* In case of EEPROM commands, calculate the CRC for the different channels selected */
    if((process_dataid_cmd == DATA_ID_6) || (process_dataid_cmd == DATA_ID_D))
    {
        uint32_t ch = 0;
        for(ch = 0; ch < MAX_CHANNELS; ch++)
        {
            if(gTamagawa_multi_ch_mask & (1 << ch))
            {
                tamagawa_update_crc(handle1, process_dataid_cmd, ch);
            }
        }
        
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        for(ch = 0; ch < MAX_CHANNELS; ch++)
        {
            if(gTamagawa1_multi_ch_mask & (1 << ch))
            {
                tamagawa_update_crc(handle2, process_dataid_cmd, ch);
            }
        }
#endif
    }

    tamagawa_command_process(handle1, process_dataid_cmd, gTamagawa_multi_ch_mask);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_command_process(handle2, process_dataid_cmd, gTamagawa1_multi_ch_mask);
#endif
    
        while(1)
        {
            if(tamagawa_position_loop_status == TAMAGAWA_POSITION_LOOP_STOP)
            {
                tamagawa_stop_periodic_continuous_mode(&tamagawa_periodic_interface);
                tamagawa_config_host_trigger(handle1);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
                tamagawa_config_host_trigger(handle2);
#endif
                return;
            }
            else
            {
                if(gTamagawa_is_multi_ch)
                {
                    DebugP_log("\r\n Multi-channel mode is enabled\n\n");
    
                    uint32_t ch;
                    for(ch = 0; ch < MAX_CHANNELS; ch++)
                    {
                        if(gTamagawa_multi_ch_mask & (1 << ch))
                        {
                            tamagawa_multi_channel_set_cur(handle1, ch);
                            DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                            tamagawa_handle_rx(handle1, process_dataid_cmd);
                        }
                    }
                }
                else
                {
                    DebugP_log("\r\n Single-channel mode is enabled\n\n");
                    tamagawa_handle_rx(handle1, process_dataid_cmd);
                }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
                if(gTamagawa1_is_multi_ch)
                {
                    DebugP_log("\r\n 2nd Slice Multi-channel mode is enabled\n\n");
    
                    uint32_t ch;
                    for(ch = 0; ch < MAX_CHANNELS; ch++)
                    {
                        if(gTamagawa1_multi_ch_mask & (1 << ch))
                        {
                            tamagawa_multi_channel_set_cur(handle2, ch);
                            DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                            tamagawa_handle_rx(handle2, process_dataid_cmd);
                        }
                    }
                }
                else
                {
                    DebugP_log("\r\n 2nd Slice Single-channel mode is enabled\n\n");
                    tamagawa_handle_rx(handle2, process_dataid_cmd);
                }
#endif
                ClockP_usleep(100);
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

    uint32_t selected_ch;

    tamagawa_pruicss_init();

    Tamagawa_Params tamagawa_params;

    /* Initialize the gTamagawaHandle1 structure according to the PRUx slice selected */
    tamagawa_params.pru_cfg.iep_cmp_event = IEP_CMP_EVENT;
    tamagawa_params.pru_cfg.pruicss_handle = gPruIcssXHandle;
    tamagawa_params.pru_cfg.pru_clock = ICSS_PRU_CORE_CLOCK;
    tamagawa_params.pru_cfg.uart_clock = ICSS_PRU_UART_CLOCK;
    tamagawa_params.pru_cfg.iep_clock = ICSS_PRU_IEP_CLOCK;
    tamagawa_params.pru_cfg.iep_instance = TAMAGAWA_PERIODIC_MODE_IEP_INSTANCE;

    tamagawa_params.clk_cfg.rx_clk_source = CONFIG_TAMAGAWA0_TX_RX_FIFO_CLOCK_SOURCE;
    tamagawa_params.clk_cfg.tx_clk_source = CONFIG_TAMAGAWA0_TX_RX_FIFO_CLOCK_SOURCE;
    tamagawa_params.clk_cfg.rx_os_rate = TAMAGAWA_RX_OVERSAMPLING_RATE;
#if CONFIG_TAMAGAWA0_PRUICSS_PRUx
        tamagawa_params.pru_cfg.pru_slice = PRUICSS_PRU1;
        gTamagawaHandle1 = tamagawa_init(CONFIG_TAMAGAWA0, tamagawa_params);
#else
        tamagawa_params.pru_cfg.pru_slice = PRUICSS_PRU0;
        gTamagawaHandle1 = tamagawa_init(CONFIG_TAMAGAWA0, tamagawa_params);
#endif
    if(gTamagawaHandle1 == NULL)
    {
        DebugP_log("tamagawa_init failed\n");
        return;
    }
    
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    Tamagawa_Params tamagawa_params1;

    tamagawa_params1.pru_cfg.iep_cmp_event = DUAL_CH_IEP_CMP_EVENT;
    tamagawa_params1.pru_cfg.pruicss_handle = gPruIcssXHandle;
    tamagawa_params1.pru_cfg.pru_clock = ICSS_PRU_CORE_CLOCK;
    tamagawa_params1.pru_cfg.uart_clock = ICSS_PRU_UART_CLOCK;
    tamagawa_params1.pru_cfg.iep_clock = ICSS_PRU_IEP_CLOCK;
    tamagawa_params1.pru_cfg.iep_instance = TAMAGAWA_PERIODIC_MODE_IEP_INSTANCE;

    tamagawa_params1.clk_cfg.rx_clk_source = CONFIG_TAMAGAWA1_TX_RX_FIFO_CLOCK_SOURCE;
    tamagawa_params1.clk_cfg.tx_clk_source = CONFIG_TAMAGAWA1_TX_RX_FIFO_CLOCK_SOURCE;
    tamagawa_params1.clk_cfg.rx_os_rate = TAMAGAWA_RX_OVERSAMPLING_RATE;
#if TAMAGAWA1_PRUICSS_SLICEx == 1
    tamagawa_params1.pru_cfg.pru_slice = PRUICSS_PRU1;
    gTamagawaHandle2 = tamagawa_init(CONFIG_TAMAGAWA1, tamagawa_params1);
#else
    tamagawa_params1.pru_cfg.pru_slice = PRUICSS_PRU0;
    gTamagawaHandle2 = tamagawa_init(CONFIG_TAMAGAWA1, tamagawa_params1);
#endif
    
    if(gTamagawaHandle2 == NULL)
    {
        DebugP_log("tamagawa1_init failed\n");
        return;
    }
#endif
    DebugP_log("\r\n\nTamagawa PRU-ICSS init done\n\n");

    /* Set the value of gTamagawa_multi_ch_mask based on the channels selected */
    gTamagawa_multi_ch_mask = (CONFIG_TAMAGAWA0_CHANNEL0<<0|CONFIG_TAMAGAWA0_CHANNEL1<<1|CONFIG_TAMAGAWA0_CHANNEL2<<2);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    gTamagawa1_multi_ch_mask = (CONFIG_TAMAGAWA1_CHANNEL0<<0|CONFIG_TAMAGAWA1_CHANNEL1<<1|CONFIG_TAMAGAWA1_CHANNEL2<<2);
#endif

    if (CONFIG_TAMAGAWA0_CHANNEL0 + CONFIG_TAMAGAWA0_CHANNEL1 + CONFIG_TAMAGAWA0_CHANNEL2 > 1)
    {
        gTamagawa_is_multi_ch = 1;
    }

    if(gTamagawa_is_multi_ch)
    {
        tamagawa_config_multi_channel_mask(gTamagawaHandle1, gTamagawa_multi_ch_mask);
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
        tamagawa_config_channel(gTamagawaHandle1, selected_ch);
    }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    if (CONFIG_TAMAGAWA1_CHANNEL0 + CONFIG_TAMAGAWA1_CHANNEL1 + CONFIG_TAMAGAWA1_CHANNEL2 > 1)
    {
        gTamagawa1_is_multi_ch = 1;
    }

    if(gTamagawa1_is_multi_ch)
    {
        tamagawa_config_multi_channel_mask(gTamagawaHandle2, gTamagawa1_multi_ch_mask);
    }
    else
    {
        uint32_t tamagawa1_selected_ch;
        if (CONFIG_TAMAGAWA1_CHANNEL0 == 1)
        {
            tamagawa1_selected_ch = 0;
        }
        if (CONFIG_TAMAGAWA1_CHANNEL1 == 1)
        {
            tamagawa1_selected_ch = 1;
        }
        if (CONFIG_TAMAGAWA1_CHANNEL2 == 1)
        {
            tamagawa1_selected_ch = 2;
        }
        tamagawa_config_channel(gTamagawaHandle2, tamagawa1_selected_ch);
    }
#endif
    
    uint32_t firmware_ver;
    firmware_ver = tamagawa_get_fw_version();

    DebugP_log("\r\nTamagawa firmware \t: %x.%x.%x (%s)\n\n", (firmware_ver >> 24) & 0x7F,
                (firmware_ver >> 16) & 0xFF, firmware_ver & 0xFFFF, firmware_ver & (1 << 31) ? "internal" : "release");

    DebugP_log("\r\nChannel(s) selected: %s %s %s \n\n\n",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH0 ? "0" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH1 ? "1" : "",
                gTamagawa_multi_ch_mask & TAMAGAWA_MULTI_CH2 ? "2" : "");

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    firmware_ver = tamagawa1_get_fw_version();
    DebugP_log("\r\nTamagawa firmware \t: %x.%x.%x (%s)\n\n", (firmware_ver >> 24) & 0x7F,
                (firmware_ver >> 16) & 0xFF, firmware_ver & 0xFFFF, firmware_ver & (1 << 31) ? "internal" : "release");

    DebugP_log("\r\nChannel(s) selected on second slice: %s %s %s \n\n\n",
                gTamagawa1_multi_ch_mask & TAMAGAWA_MULTI_CH0 ? "0" : "",
                gTamagawa1_multi_ch_mask & TAMAGAWA_MULTI_CH1 ? "1" : "",
                gTamagawa1_multi_ch_mask & TAMAGAWA_MULTI_CH2 ? "2" : "");
#endif

    tamagawa_config_host_trigger(gTamagawaHandle1);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_config_host_trigger(gTamagawaHandle2);
#endif
    tamagawa_pruicss_load_run_fw();
    DebugP_log("\r\nTamagawa PRU-ICSS firmware loaded and running\n\n\n");

    tamagawa_set_baudrate(gTamagawaHandle1, CONFIG_TAMAGAWA0_BAUDRATE);

#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
    tamagawa_set_baudrate(gTamagawaHandle2, CONFIG_TAMAGAWA1_BAUDRATE);
#endif

    while(1)
    {
        /*
         * Initialized to zero to remove the compiler warning about the variable being uninitialized.
         */
        uint8_t adf = 0, edf = 0;
        enum data_id cmd;

        tamagawa_display_menu();
        cmd = tamagawa_get_command(&adf, &edf);

        if(cmd < 0)
        {
            continue;
        }

        if(cmd == PERIODIC_TRIGGER_CMD)
        {
            /*Takes which data_id to process in input arguments*/            
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
            tamagawa_process_periodic_command(gTamagawaHandle1, DATA_ID_0, gTamagawaHandle2);
#else
            tamagawa_process_periodic_command(gTamagawaHandle1, DATA_ID_0);
#endif
        }

        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        tamagawa_update_data_id(gTamagawaHandle1, cmd);

        /* In case of EEPROM commands, calculate the CRC for the different channels selected */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint32_t ch = 0;
            for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_update_crc(gTamagawaHandle1, cmd, ch);
                }
            }

        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        tamagawa_update_data_id(gTamagawaHandle2, cmd);

        /* In case of EEPROM commands, calculate the CRC for the different channels selected */
        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            uint32_t T1_ch = 0;
            for(T1_ch = 0 ; T1_ch < MAX_CHANNELS ; T1_ch++)
            {
                if(gTamagawa1_multi_ch_mask & 1 << T1_ch)
                {
                    tamagawa_update_crc(gTamagawaHandle2, cmd, T1_ch);
                }
            }

        }
#endif

        tamagawa_command_process(gTamagawaHandle1, cmd, gTamagawa_multi_ch_mask);
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        tamagawa_command_process(gTamagawaHandle2, cmd, gTamagawa1_multi_ch_mask);
#endif

        if(gTamagawa_is_multi_ch)
        {
            DebugP_log("\r\n Multi-channel mode is enabled\n\n");

            uint32_t ch;
            for(ch = 0; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_multi_channel_set_cur(gTamagawaHandle1, ch);
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    tamagawa_handle_rx(gTamagawaHandle1, cmd);
                }
            }
        }
        else
        {
            DebugP_log("\r\n Single-channel mode is enabled\n\n");
            tamagawa_handle_rx(gTamagawaHandle1, cmd);
        }
#if defined(TAMAGAWA_DUAL_PRU_SLICE_ENABLE)
        if(gTamagawa1_is_multi_ch)
        {
            DebugP_log("\r\n Dual PRU Slice Multi-channel mode is enabled\n\n");
            uint32_t ch;
            for(ch = 0; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa1_multi_ch_mask & 1 << ch)
                {
                    tamagawa_multi_channel_set_cur(gTamagawaHandle2, ch);
                    DebugP_log("\r\n\r|\n|\t\t\t\tCHANNEL %d\n", ch);
                    tamagawa_handle_rx(gTamagawaHandle2, cmd);
                }
            }
            
        }
        else
        {
            DebugP_log("\r\n Dual PRU Slice Single-channel mode is enabled\n\n");
            tamagawa_handle_rx(gTamagawaHandle2, cmd);
        }
#endif
    }

    Board_driversClose();
    Drivers_close();
}
