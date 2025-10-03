/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <position_sense/tamagawa/include/tamagawa_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <stdlib.h>

extern uint32_t gTamagawaConfigNum;
extern Tamagawa_Config gTamagawaHandle[];

int32_t tamagawa_parse(int32_t cmd, Tamagawa_Handle handle)
{
    /* Parses the data in tamagawa interface received from the Encoder*/
    uint32_t word0, word1, word2;
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    word0 = tamagawa_xchg->ch[handle->channel].pos_word0;
    word1 = tamagawa_xchg->ch[handle->channel].pos_word1;
    word2 = tamagawa_xchg->ch[handle->channel].pos_word2;

    switch(cmd)
    {
        case DATA_ID_0:
            /* Data readout: data in one revolution */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            /* Data frames: cf(8 bits) + sf(8 bits) + abm(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf =(word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf =(word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abm = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc =(word1) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_2:
            /* Data readout: encoder ID */
            /* Data frames: cf(8 bits) + sf(8 bits) + enid(8 bits)+ crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.enid = (word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + enid(8 bits) + abm(3 frames with 8 bits data each) + almc(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>24) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.enid =(word1 >> 16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abm = ((word1>>8)& 0xFF) | ((word1)& 0xFF)<<8 | (((word2>>16) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.almc =(word2 >> 8) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word2) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word2=word2<<8;
            break;

        case DATA_ID_7:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_8:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_C:
            /* Reset */
            /* Data frames: cf(8 bits) + sf(8 bits) + abs(3 frames with 8 bits data each) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.sf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.abs = ((word0>>8)& 0xFF) | ((word0)& 0xFF)<<8 | (((word1>>8) & 0xFF) << 16);
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word1) & 0xFF;
            tamagawa_xchg->ch[handle->channel].pos_word1=word1<<16;
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Data frames: cf(1frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.adf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.edf =(word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Data frames: cf(1frame) + adf(8 bits) + edf(8 bits) + crc(8 bits) */
            tamagawa_xchg->tamagawa_interface.rx_frames_received.cf = (word0>>24) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.adf = (word0>>16) & 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.edf =(word0>>8)& 0xFF;
            tamagawa_xchg->tamagawa_interface.rx_frames_received.crc = (word0)& 0xFF;
            break;

        default:
            return -1;
    }

    return 0;
}

static uint8_t tamagawa_crc(Tamagawa_Handle handle, uint8_t len, uint32_t ch)
{
    /* Calculates the CRC for the currently selected channel. Uses the CRC function (X^8+1) */
    uint8_t crc = 0;
    uint8_t val;
    uint32_t  i, j;
    uint32_t word0, word1, word2;

    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    word0 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word0;
    word1 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word1;
    word2 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word2;

    /* In EEPROM Write, we have CF, ADF and EDF fields as bits 24-31, 16-23 and 8-15 respectively */
    /* In EEPROM Write, we have CF and ADF fields as bits 24-31 and 16-23 respectively */
    /* Size of the array is 12 in order to use 12 8 bit integers for different frames */
    uint8_t data[12];
    data[3]=word0 & 0xFF;
    data[2]=(word0 >> 8) & 0xFF;
    data[1]=(word0 >> 16) & 0xFF;
    data[0]=(word0 >> 24) & 0xFF;

    data[7]=word1 & 0xFF;
    data[6]=(word1 >> 8) & 0xFF;
    data[5]=(word1 >> 16) & 0xFF;
    data[4]=(word1 >> 24) & 0xFF;

    data[11]=word2 & 0xFF;
    data[10]=(word2 >> 8) & 0xFF;
    data[9]=(word2 >> 16) & 0xFF;
    data[8]=(word2 >> 24) & 0xFF;


    for(i = 0; i < len; i++)
    {
        for(j = 0; j < 8; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);
            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }
    }

    return crc;
}

void tamagawa_update_crc(Tamagawa_Handle handle, int32_t cmd, uint32_t ch)
{
    /* Passes the values of CF(Control Field), ADF(address of EEPROM) and EDF(data for EEPROM) to the CRC calculator fucntion and update the CRC field */
    uint32_t word0;
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;
    tamagawa_eeprom_crc_reinit(handle);
    word0 = tamagawa_xchg->tamagawa_eeprom_interface[ch].word0;

    if (cmd == DATA_ID_6)
    {
        /* Bitwise OR the 32 bit integer with the CF, ADF and EDF values left shifted 24, 16 and 8 times respectively, in order to get the correct sequence of bits for CRC calculation*/
        word0 = (word0|0x32)<<24;
        word0 = word0 |  (tamagawa_xchg->tamagawa_eeprom_interface[ch].adf<<16);
        word0 = word0 | (tamagawa_xchg->tamagawa_eeprom_interface[ch].edf<<8);
        tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(handle, 3, ch);
    }
    else
    {
        /* Bitwise OR the 32 bit integer with the CF and ADF values left shifted 24 and 16 times respectively, in order to get the correct sequence of bits for CRC calculation*/
        word0 = (word0|0xEA)<<24;
        word0 = word0 | (tamagawa_xchg->tamagawa_eeprom_interface[ch].adf<<16);
        tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = word0;
        tamagawa_xchg->tamagawa_eeprom_interface[ch].crc = tamagawa_crc(handle, 2, ch);
    }

    tamagawa_eeprom_crc_reinit(handle);
}

int32_t tamagawa_crc_verify(Tamagawa_Handle handle)
{
    /* Verifies the CRC computed with the encoder crc */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;
    uint32_t word0;
    word0 = tamagawa_xchg->ch[handle->channel].cal_crc;

    if ((word0 & 0xFF)==1)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

void tamagawa_set_baudrate(Tamagawa_Handle handle, double baudrate)
{
    /* Updates the values for oversample rate and division factor for Tx and Rx based on the baud rate selected */
    uint16_t rx_div;
    uint16_t tx_div;
    Tamagawa_ClkCfg clk_cfg = handle->clk_cfg;
    if(clk_cfg.rx_clk_source == 1)
    {
        rx_div = handle->pru_cfg.pru_clock/((clk_cfg.rx_os_rate + 1)*(baudrate));
    }
    else
    {
        rx_div = handle->pru_cfg.uart_clock/((clk_cfg.rx_os_rate + 1)*(baudrate));
    }

    if(clk_cfg.tx_clk_source == 1)
    {
        tx_div = handle->pru_cfg.pru_clock/baudrate;
    }
    else
    {
        tx_div = handle->pru_cfg.uart_clock/baudrate;
    }
   
    clk_cfg.tx_div = tx_div - 1;
    clk_cfg.rx_div = rx_div - 1;

    /*Configure RX auto arm counter for 1us, FIXME: need to add value based on tamagawa freq. */
    clk_cfg.rx_en_cnt = TAMAGAWA_DELAY_COUNTER_INCREMENT*(handle->pru_cfg.pru_clock/1000000);
    tamagawa_config_clock(handle, &clk_cfg);
    tamagawa_config_global_rx_arm_cnt(handle, clk_cfg.rx_en_cnt);
    /*write in DMEM*/
    handle->tamagawa_xchg->tamagawa_interface.rx_div_factor = rx_div - 1;
    handle->tamagawa_xchg->tamagawa_interface.tx_div_factor = tx_div - 1;
    handle->tamagawa_xchg->tamagawa_interface.oversample_rate = clk_cfg.rx_os_rate;
    
}

int32_t tamagawa_command_build(Tamagawa_Handle handle, int32_t cmd,  uint8_t gTamagawa_multi_ch_mask)
{
    /* Sets up the tamagawa command in the PRU interface buffer */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    /*first clear command parameters to be safe */
    memset(&tamagawa_xchg->cmd, 0, sizeof(tamagawa_xchg->cmd));
    uint32_t ch;
    switch(cmd)
    {
        case DATA_ID_0:
            /* Data readout: data in one revolution */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x20)|(0x40<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x28)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x24)|(0xC0<<8);
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (1)|(4<<8);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x2C)|(0x40<<8);
            /* Number of expected Rx frames is 11 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 11;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 11 */
            tamagawa_xchg->cmd.word1 = (1)|(0xB<<8);
            break;

        case DATA_ID_7:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x2E)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_8:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x21)|(0xC0<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_C:
            /* Reset */
            /* After reversing the Control Field and adding the start and stop bits, update the Tx data such that it can be loaded byte-wise  */
            tamagawa_xchg->cmd.word0 = (0x23)|(0x40<<8);
            /* Number of expected Rx frames is 6 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 6;
            /* Number of Tx frames being sent to the encoder is 1, and the number of Rx frames to be received is 6 */
            tamagawa_xchg->cmd.word1 = (1)|(6<<8);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF and CRC data */
            for(ch = 0 ; ch < MAX_CHANNELS ; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_prepare_eeprom_command(handle, cmd, ch);
                }
            }
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 3, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (3)|(4<<8);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            /* Loop through all the selected channels and prepare the EEPROM Read Tx data based on the CF, ADF, EDF and CRC data */
            for(ch = 0 ; ch < MAX_CHANNELS; ch++)
            {
                if(gTamagawa_multi_ch_mask & 1 << ch)
                {
                    tamagawa_prepare_eeprom_command(handle, cmd, ch);
                }
            }
            /* Number of expected Rx frames is 4 */
            tamagawa_xchg->tamagawa_interface.rx_frames = 4;
            /* Number of Tx frames being sent to the encoder is 4, and the number of Rx frames to be received is 4 */
            tamagawa_xchg->cmd.word1 = (4)|(4<<8);
            break;

        default:
            cmd = -1;
            break;
    }

    return cmd;
}

void tamagawa_command_send(Tamagawa_Handle handle)
{
    /* Triggers sending the tamagawa command in PRU */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;
    /* Set the trigger value as 1 */
    tamagawa_xchg->config.trigger = 0x1;
}

void tamagawa_command_wait(Tamagawa_Handle handle)
{
    /* Waits till PRU finishes tamagawa transaction */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;
    /* Wait until the trigger value is 1 */
    while(tamagawa_xchg->config.trigger & 0x1)
        ;
}

int32_t tamagawa_command_process(Tamagawa_Handle handle, int32_t cmd, uint8_t gTamagawa_multi_ch_mask)
{
    /* Sends the tamagawa command and waits till firmware acknowledges */

    /* Use tamagawa_command_build to setup the tamagawa command in the PRU interface buffer */
    cmd = tamagawa_command_build(handle, cmd, gTamagawa_multi_ch_mask);

    if(cmd < 0)
    {
        return cmd;
    }

    /* Trigger sending the tamagawa command in PRU */
    tamagawa_command_send(handle);
    /* Wait till PRU finishes tamagawa transaction */
    tamagawa_command_wait(handle);

    /* In case of EEPROM commands, reset the command id for all the channels back to 0 */
    if (cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        handle->tamagawa_xchg->tamagawa_eeprom_interface[0].cmd = 0;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[1].cmd = 0;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[2].cmd = 0;
    }

    return cmd;
}

uint32_t tamagawa_reverse_bits(int8_t data)
{
    /* Reverses the bits of the 8 bit data*/
    /* Loops through all the bits of input data. If the bit at i-th position is set for the input data, then set the bit at (N – 1 – i)-th position in reversed number.
        Here, N is the number of bits in the input data.*/
    uint32_t number_of_bits = sizeof(data) * 8;
    uint32_t reversed_num = 0;
    uint32_t current_bit_pos;
    for (current_bit_pos = 0; current_bit_pos < number_of_bits; current_bit_pos++) {
        if ((data & (1 << current_bit_pos)))
            reversed_num |= 1 << ((number_of_bits - 1) - current_bit_pos);
    }
    return reversed_num;
}

uint64_t tamagawa_prepare_eeprom_tx_data(uint64_t eeprom_tx_data, volatile uint32_t data)
{
    /* Takes the 8 bit data of CF(Control Field), ADF(address of EEPROM), EDF(data for EEPROM) and CRC, reverses it and adds the start and the stop bits */

    /* Call tamagawa_reverse_bits function in order to reverse the bits of the data */
    uint32_t reversed_num  = tamagawa_reverse_bits(data);
    /* Perform left shift of the 64 bit integer holding the EEPROM Tx data 9 times and bitwise-OR it with the reversed number to add the start bit and the reversed data*/
    eeprom_tx_data = eeprom_tx_data<<9;
    eeprom_tx_data = eeprom_tx_data|reversed_num;
    /* Perform left shift of the integer holding the EEPROM Tx data 1 more time and bitwise-OR it with 1 in order to add the stop bit */
    eeprom_tx_data = eeprom_tx_data<<1;
    eeprom_tx_data = eeprom_tx_data|1;
    return eeprom_tx_data;
}

void tamagawa_prepare_eeprom_command(Tamagawa_Handle handle, int32_t cmd,uint32_t ch)
{
    /* Prepares the required EEPROM command from the CF(Control Field), ADF(address of EEPROM), EDF(data for EEPROM) and CRC */
    uint64_t eeprom_tx_data = 0;
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;
    if (cmd== DATA_ID_6)
    {
        /* Command preparation for EEPROM Write */
        /* Control Field value for EEPROM Write is 0x32 */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, 0x32);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].edf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].crc);
    }
    else
    {
        /* Command preparation for EEPROM Read */
        /* Control Field value for EEPROM Read is 0xEA */
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, 0xEA);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].adf);
        eeprom_tx_data = tamagawa_prepare_eeprom_tx_data(eeprom_tx_data, tamagawa_xchg->tamagawa_eeprom_interface[ch].crc);
        /* 64 bit integer is left shifted twice to load the data byte wise into the Tx FIFO correctly */
        eeprom_tx_data <<= 2;
    }
    /* Assign the value of the prepared Tx data for EEPROM commands to the 64 bit integer eeprom_tx_data for the currently selected channel */
    tamagawa_xchg->tamagawa_eeprom_interface[ch].eeprom_tx_data = eeprom_tx_data;
}

void tamagawa_config_clock(Tamagawa_Handle handle, Tamagawa_ClkCfg *clk_cfg)
{
    /* Configures the tamagawa clock */
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    /* Configure the PRUx Rx CFG register by writing the Rx Divide Factor and Oversampling rate */
    HW_WR_REG32((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_RXCFG, (uint32_t)(clk_cfg->rx_div << 16 | clk_cfg->rx_clk_source << 4 | clk_cfg->rx_os_rate));
    /* Configure the PRUx Tx CFG register by writing the Tx Divide Factor  */
    HW_WR_REG32((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_TXCFG , (uint32_t)(clk_cfg->tx_div << 16 | clk_cfg->tx_clk_source << 4));
}

void tamagawa_config_global_rx_arm_cnt(Tamagawa_Handle handle, uint16_t  rx_en_cnt)
{
    /* Configures the global RX auto arm counter for tamagawa master */
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    /* Write the value to the PRUx RX Global Auto Arm Counter register */
    HW_WR_REG16((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG1 + 2, rx_en_cnt);
    HW_WR_REG16((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG1 + 2, rx_en_cnt);
    HW_WR_REG16((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG1 + 2, rx_en_cnt);
}

void tamagawa_config_host_trigger(Tamagawa_Handle handle)
{
    /* Configures tamagawa master for host trigger mode */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    tamagawa_xchg->config.opmode = 0x1;
}

void tamagawa_config_periodic_trigger(Tamagawa_Handle handle)
{
    /* Configures tamagawa master in periodic trigger mode */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    tamagawa_xchg->config.opmode = 0;
}

void tamagawa_config_channel(Tamagawa_Handle handle, uint32_t ch)
{
    /* Selects the channel to be used by tamagawa master */
    Tamagawa_Xchg *tamagawa_xchg = handle->tamagawa_xchg;

    tamagawa_xchg->config.channel = 1 << ch;
    handle->channel = ch;
    tamagawa_xchg->tamagawa_interface.ch_mask = 1 << ch;
}

void tamagawa_config_multi_channel_mask(Tamagawa_Handle handle, uint8_t mask)
{
    /* Selects mask of channels to be used in multi channel configuration */
    handle->tamagawa_xchg->config.channel = mask;
    handle->tamagawa_xchg->tamagawa_interface.ch_mask = mask;
}

void tamagawa_update_data_id(Tamagawa_Handle handle, int32_t cmd)
{
    /* Updates the data id according to the command selected */
    handle->tamagawa_xchg->tamagawa_interface.data_id= cmd;
    if(cmd == DATA_ID_6 || cmd == DATA_ID_D)
    {
        handle->tamagawa_xchg->tamagawa_eeprom_interface[0].cmd = cmd;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[1].cmd = cmd;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[2].cmd = cmd;
    }
}

void tamagawa_update_adf(Tamagawa_Handle handle, uint32_t val, uint32_t ch)
{
    /* Updates the ADF field for the currently selected channel for EEPROM commands */
    handle->tamagawa_xchg->tamagawa_eeprom_interface[ch].adf = val;
}

void tamagawa_update_edf(Tamagawa_Handle handle, uint32_t val, uint32_t ch)
{
    /* Updates the EDF field for the currently selected channel for EEPROM Write command */
    handle->tamagawa_xchg->tamagawa_eeprom_interface[ch].edf = val;
}

void tamagawa_eeprom_crc_reinit(Tamagawa_Handle handle)
{
    /* Resets the integers used to calculate the CRC back to 0 for all the channels */
    uint32_t ch = 0;
    for(ch = 0; ch < MAX_CHANNELS; ch++)
    {
        handle->tamagawa_xchg->tamagawa_eeprom_interface[ch].word0 = 0;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[ch].word1 = 0;
        handle->tamagawa_xchg->tamagawa_eeprom_interface[ch].word2 = 0;
    }
}

uint8_t tamagawa_multi_channel_detected(Tamagawa_Handle handle)
{
    /* Detects the channels that have been selected in multi-channel configuration */
    return handle->tamagawa_xchg->config.channel;
}

void tamagawa_multi_channel_set_cur(Tamagawa_Handle handle, uint32_t ch)
{
    /* Set the channel that is currently being processed */
    handle->channel = ch;
}

static inline void tamagawa_config_clr_cfg0(Tamagawa_Handle handle)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 0 */
    HW_WR_REG32((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0, 0);
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 1 */
    HW_WR_REG32((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0, 0);
    /* Clear the CFG0 registers for the selected PRUx slice for Channel 2 */
    HW_WR_REG32((uint32_t)(pruicss_cfg) + handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0, 0);
}
Tamagawa_Handle tamagawa_init(uint32_t index, Tamagawa_Params tamagawa_params)
{
    Tamagawa_Handle handle = NULL;
    
    handle = (Tamagawa_Handle)(&gTamagawaHandle[index]);

    if(index >= gTamagawaConfigNum)
    {
        return NULL;
    }
    
    /* Configure the handle */
    if(tamagawa_params.pru_cfg.pruicss_handle == NULL) {
        /* Return NULL if required PRU handle is NULL */
        return NULL;
    }
    
    if(tamagawa_params.pru_cfg.pru_slice == 1)
    {
        handle->tamagawa_xchg = (Tamagawa_Xchg *)((PRUICSS_HwAttrs *)(tamagawa_params.pru_cfg.pruicss_handle->hwAttrs))->pru1DramBase;
    }
    else if(tamagawa_params.pru_cfg.pru_slice == 0)
    {
        handle->tamagawa_xchg = (Tamagawa_Xchg *)((PRUICSS_HwAttrs *)(tamagawa_params.pru_cfg.pruicss_handle->hwAttrs))->pru0DramBase;
    }
    else
    {
        /* Return NULL if invalid pru slice is provided */
        return NULL;
    }
   /* Check valid range for IEP comparator (0-15), clocks must be positive */
   if((tamagawa_params.pru_cfg.iep_cmp_event < 0 || tamagawa_params.pru_cfg.iep_cmp_event > 15) ||
      (tamagawa_params.pru_cfg.pru_clock <= 0) ||
      (tamagawa_params.pru_cfg.uart_clock <= 0) || 
      (tamagawa_params.pru_cfg.iep_clock <= 0) || 
      (tamagawa_params.pru_cfg.iep_instance < 0 || tamagawa_params.pru_cfg.iep_instance > 1))
    {
        /* Return NULL if invalid parameters */
        return NULL;
    }
    handle->pru_cfg = tamagawa_params.pru_cfg;

    if((tamagawa_params.clk_cfg.tx_clk_source > 1 || tamagawa_params.clk_cfg.tx_clk_source < 0) || 
       (tamagawa_params.clk_cfg.rx_clk_source > 1 || tamagawa_params.clk_cfg.rx_clk_source < 0) || 
       (tamagawa_params.clk_cfg.rx_os_rate < 0 || tamagawa_params.clk_cfg.rx_os_rate > 7))
    {
        /* Return NULL if invalid clock parameters */
        return NULL;
    }
    
    handle->clk_cfg = tamagawa_params.clk_cfg;

    handle->instance_index = index;

    /* If the slice value is 0, it denotes that PRU0 is selected. Assign the register offsets for PRU0 */
    if(tamagawa_params.pru_cfg.pru_slice == 0)
    {
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_GPCFGx = CSL_ICSS_PR1_CFG_SLV_GPCFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_RXCFG = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_TXCFG = CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG;
    }
    /* If the slice value is 1, it denotes that PRU1 is selected. Assign the register offsets for PRU1 */
    else /* tamagawa_params.pru_cfg.pru_slice == 1 */
    {
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG0 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH0_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH1_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_CH2_CFG1 = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG1_REG;
        handle->register_offset_val.ICSS_CFG_GPCFGx = CSL_ICSS_PR1_CFG_SLV_GPCFG1_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_RXCFG = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG;
        handle->register_offset_val.ICSS_CFG_PRUx_ED_TXCFG = CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG;
    }
    
    /* Clear CFG0 registers */
    tamagawa_config_clr_cfg0(handle);

    return handle;
}
