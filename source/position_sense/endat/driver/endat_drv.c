/*
 *  Copyright (C) 2021-25 Texas Instruments Incorporated
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

#include <position_sense/endat/include/endat_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <stdlib.h> 

extern uint32_t gEndatConfigNum;
extern Endat_Config gEndatHandle[];

/*
 * check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that functions,
 * endat_recvd_organize()
 * endat_recvd_format()
 * endat_recvd_validate()
 * endat_recvd_print()
 * will normally be called after endat_get_command
 */

static int32_t endat_recvd_organize(int32_t cmd, Endat_Handle handle,
                                Endat_Data *endat_data)
{
    uint32_t word0, word1, word2, word3;
    uint32_t pos_bits, shift;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;
    memset(endat_data, 0, sizeof(*endat_data));

    word0 = channel_rx_info->ch[handle->current_channel].posWord0;
    word1 = channel_rx_info->ch[handle->current_channel].posWord1;
    word2 = channel_rx_info->ch[handle->current_channel].posWord2;
    word3 = channel_rx_info->ch[handle->current_channel].posWord3;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            endat_data->recvd1 = word0;
            break;

        case 6:
            shift = ENDAT_RX_46BITS % (sizeof(unsigned) * 8);
            endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            break;

        case 1:
            pos_bits = handle->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1;

            if(pos_bits <= sizeof(unsigned) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(unsigned) * 8);
                endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos_bits = handle->pos_res + ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1 +
                       ENDAT_NUM_BITS_F2;

            if(pos_bits <= sizeof(unsigned) * 8)
            {
                endat_data->recvd1 = word0;
            }
            else
            {
                shift = pos_bits % (sizeof(unsigned) * 8);
                endat_data->recvd1 = (uint64_t) word0 << shift | word1;
            }
            if(handle->flags.info1 || handle->flags.info2)
            {
                endat_data->recvd2 = word2;
            }
            if(handle->flags.info1 && handle->flags.info2)
            {
                endat_data->recvd3 = word3;
            }
            break;

        default:
            return -EINVAL;
            break;
    }

    return 0;
}

/* value to be reflected should be aligned to lsb, relected value would be aligned to lsb */
static uint64_t endat_reflect_ull_nbits(uint64_t input,
        uint32_t n)
{
    uint32_t i;
    uint64_t val = 0;

    /* val initialized to 0 above to handle n = 0 case, otherwise garbage would be returned */
    for(i = 0; i < n; i++)
    {
        val <<= 1;

        if(input & ((uint64_t) 1 << i))
        {
            val |= 1;
        }
    }

    return val;
}

static int64_t endat_recvd_format(int32_t cmd, Endat_Handle handle,
                              Endat_Data *endat_data, Endat_FormatData *u)
{
    uint64_t pos, rev;

    switch(cmd)
    {
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 14:
            u->addr_params.params = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC)
                                    & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
            u->addr_params.address = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC +
                                      ENDAT_NUM_BITS_PARAMETER)) &
                                     ((1 << ENDAT_NUM_BITS_ADDRESS) - 1);
            u->addr_params.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC)
                                 - 1);
            break;

        case 6:
            u->test.value = (endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC) & (((
                                uint64_t)1 << 40) - 1);
            u->test.f1 = (endat_data->recvd1 >> (ENDAT_NUM_BITS_POSITION_CRC + 40)) & 1;
            u->test.crc = endat_data->recvd1 & ((1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 1:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((uint64_t) 1 << handle->pos_res) - 1);       /* mask F1 */
            pos = endat_reflect_ull_nbits(pos, handle->pos_res);
            rev = (pos & (((uint64_t) 1 << handle->pos_res) - 1)) >>
                  handle->single_turn_res[handle->current_channel];
            pos = pos & (((uint64_t) 1 << handle->single_turn_res[handle->current_channel]) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + handle->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            pos = endat_data->recvd1 >> ENDAT_NUM_BITS_POSITION_CRC;
            pos = pos & (((uint64_t) 1 << handle->pos_res) -
                         1);  /* mask F1/F2 */
            pos = endat_reflect_ull_nbits(pos, handle->pos_res);
            rev = (pos & (((uint64_t) 1 << handle->pos_res) - 1)) >>
                  handle->single_turn_res[handle->current_channel];
            pos = pos & (((uint64_t) 1 << handle->single_turn_res[handle->current_channel]) - 1);
            u->position_addinfo.position.position = pos;
            u->position_addinfo.position.revolution = rev;
            u->position_addinfo.position.f1 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + handle->pos_res + 1)) & 1;
            u->position_addinfo.position.f2 = (endat_data->recvd1 >>
                                               (ENDAT_NUM_BITS_POSITION_CRC + handle->pos_res)) & 1;
            u->position_addinfo.position.crc = endat_data->recvd1 & ((
                                                   1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);

            if(handle->flags.info1 || handle->flags.info2)
            {
                if(handle->flags.info2)
                {
                    u->position_addinfo.addinfo2.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo2.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
                else
                {
                    u->position_addinfo.addinfo1.addinfo = (endat_data->recvd2 >>
                                                            ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                    u->position_addinfo.addinfo1.crc = endat_data->recvd2 & ((
                                                           1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
                }
            }

            if(handle->flags.info1 && handle->flags.info2)
            {
                u->position_addinfo.addinfo1.addinfo = (endat_data->recvd3 >>
                                                        ENDAT_NUM_BITS_POSITION_CRC) & ((1 << 24) - 1);
                u->position_addinfo.addinfo1.crc = endat_data->recvd3 & ((
                                                       1 << ENDAT_NUM_BITS_POSITION_CRC) - 1);
            }
            break;

        default:
            return -EINVAL;
            break;
    }

    return 0;
}

int32_t endat_recvd_process(Endat_Handle handle, int32_t cmd,
                        Endat_FormatData *u)
{
    Endat_Data endat_data;
    int32_t ret;

    ret = endat_recvd_organize(cmd, handle, &endat_data);

    if(ret < 0)
    {
        return ret;
    }

    return endat_recvd_format(cmd, handle, &endat_data, u);
}

#define ENDAT_USE_OTF_CRC_STATUS

#ifndef ENDAT_USE_OTF_CRC_STATUS
static uint32_t make_crc_norm(uint32_t param8, uint32_t param16)
{
    /* state of the 5 flip-flops */
    uint32_t ff[5];
    /* data bit array */
    uint32_t code[24];
    /* Auxiliary variable */
    uint32_t ex;
    /* dtetermined CRC code */
    uint32_t crc = 0;
    /* controlled variable for looping */
    int32_t i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* read 8 bit parameter into code array and convert bit sequence */
    for(i = 0; i < 8; i++)
    {
        code[i] = (param8 & 0x0080) ? 1 : 0;
        param8 <<= 1;
    }

    /* read 16 bit parameter into code array and convert bit sequence */
    for(i = 8; i < 24; i++)
    {
        code[i] = (param16 & 0x8000) ? 1 : 0;
        param16 <<= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = 0; i < 24; i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}

static uint32_t make_crc_pos(uint32_t clocks, uint32_t error1,
                                 uint32_t error2, uint32_t endat22,
                                 uint64_t highpos, uint64_t lowpos)
{
    /* state of the 5 flip-flops */
    uint32_t ff[5];
    /* data bit array */
    uint32_t code[66];
    /* Auxiliary variable */
    uint32_t ex;
    /* dtetermined CRC code */
    uint32_t crc = 0;
    /* controlled variable for looping */
    int32_t i;

    /* set all flip-flops to 1 */
    for(i = 0; i < 5; i++)
    {
        ff[i] = 1;
    }

    /* transfer alarm bits to code array */
    if(endat22)
    {
        code[0] = error1;
        code[1] = error2;
    }
    else
    {
        code[1] = error1;
    }

    /* transfer low pos bits to array */
    for(i = 2; i < 34; i++)
    {
        code[i] = (lowpos & 0x00000001L) ? 1 : 0;
        lowpos >>= 1;
    }

    /* transfer high pos bits to array */
    for(i = 34; i < 66; i++)
    {
        code[i] = (highpos & 0x00000001L) ? 1 : 0;
        highpos >>= 1;
    }

    /* calculate crc analog to described h/w generator */
    for(i = (endat22 ? 0 : 1); i <= (clocks + 1); i++)
    {
        ex = ff[4] ^ code[i];
        ff[4] = ff[3];
        ff[3] = ff[2] ^ ex;
        ff[2] = ff[1];
        ff[1] = ff[0] ^ ex;
        ff[0] = ex;
    }

    /* store crc in variable */
    for(i = 4; i >= 0; i--)
    {
        ff[i] = ff[i] ? 0 : 1;
        crc <<= 1;
        crc |= ff[i];
    }

    return crc;
}
#endif

/* return crc status: bit0 - position/address params/test, bit1 - additional info1, bit2 - additional info2. return -EINVAL on failure */
uint32_t endat_recvd_validate(Endat_Handle handle, int32_t cmd,
                              Endat_FormatData *u)
{
    uint32_t status = 0;
#ifdef ENDAT_USE_OTF_CRC_STATUS
    uint8_t val;
#else
    uint32_t crc;
    uint64_t highpos, lowpos;
    uint64_t test;
#endif

#ifdef ENDAT_USE_OTF_CRC_STATUS
    val = handle->channel_rx_info->ch[handle->current_channel].crcStatus;

    if(handle->flags.info2)
    {
        status = val & 0x1;
        /* move bit 1 to bit 2 */
        status |= (val & 0x2) << 1;

        if(handle->flags.info1)
        {
            /* move bit 2 to bit 1 */
            status |= (val & 0x4) >> 1;
        }
    }

    else
    {
        /* either additional info1 only present or no additional info present */
        status = val & 0x3;
    }

#else

    switch(cmd)
    {
        case 1:
            lowpos = (u->position_addinfo.position.revolution << handle->single_turn_res[handle->current_channel] |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << handle->single_turn_res[handle->current_channel] |
                       u->position_addinfo.position.position) >> 32;
            crc = make_crc_pos(handle->pos_res, u->position_addinfo.position.f1, 0, 0,
                               highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 6:
            test = endat_reflect_ull_nbits(u->test.value, 40);
            lowpos = test & 0xFFFFFFFF;
            highpos = test >> 32;
            crc = make_crc_pos(40, u->position_addinfo.position.f1, 0, 0, highpos,
                               lowpos);

            if(u->test.crc == crc)
            {
                status = 0x1;
            }
            break;

        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            lowpos = (u->position_addinfo.position.revolution << handle->single_turn_res[handle->current_channel] |
                      u->position_addinfo.position.position) & 0xFFFFFFFF;
            highpos = (u->position_addinfo.position.revolution << handle->single_turn_res[handle->current_channel] |
                       u->position_addinfo.position.position) >> 32;
            crc = make_crc_pos(handle->pos_res, u->position_addinfo.position.f1,
                               u->position_addinfo.position.f2, 1, highpos, lowpos);

            if(u->position_addinfo.position.crc == crc)
            {
                status = 0x1;
            }

            if(handle->flags.info1)
            {
                crc = make_crc_norm((u->position_addinfo.addinfo1.addinfo >> 16) & 0xFF,
                                    u->position_addinfo.addinfo1.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo1.crc == crc)
                {
                    status |= 0x1 << 1;
                }
            }

            if(handle->flags.info2)
            {
                crc = make_crc_norm((u->position_addinfo.addinfo2.addinfo >> 16) & 0xFF,
                                    u->position_addinfo.addinfo2.addinfo & 0xFFFF);

                if(u->position_addinfo.addinfo2.crc == crc)
                {
                    status |= 0x1 << 2;
                }
            }

            break;

        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
            crc = make_crc_norm(u->addr_params.address, u->addr_params.params);

            if(u->addr_params.crc == crc)
            {
                status = 0x1;
            }
            break;

        default:
            break;
    }

#endif

    return status;
}

/*
 * XXX: check 2.2 command case with 2.2 capability in encoder, can live w/o as endat_get_command
 * will handle and it is assumed that this function will be called either after endat_get_command
 * or by diagnostic initialization code where it is only 2.1 commands used
 */
int32_t endat_command_build(Endat_Handle handle, int32_t cmd,
                        Endat_CmdSupplement *cmd_supplement)
{
    uint32_t info;
    Endat_PruicssXchg *endat_pruicss_xchg = handle->pruicss_xchg;

    info = 0, handle->flags.info1 ? info++ : 0, handle->flags.info2 ? info++ : 0 ;

    /*first clear command parameters to be safe */
    memset(&endat_pruicss_xchg->cmd, 0, sizeof(endat_pruicss_xchg->cmd));

    switch(cmd)
    {
        case 1:
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                endat_pruicss_xchg->cmd[0].word1 = handle->pos_rx_bits_21_cmd[0] | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                endat_pruicss_xchg->cmd[1].word1 = handle->pos_rx_bits_21_cmd[1] | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                endat_pruicss_xchg->cmd[2].word1 = handle->pos_rx_bits_21_cmd[2] | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);
            } /* command build for ch0, ch1 and ch2 in load share mode*/
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSITION_VALUES;
                endat_pruicss_xchg->cmd[0].word1 = handle->pos_rx_bits_21_cmd[handle->current_channel] | (ENDAT_TX_6BITS << 8) |
                              ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION) << 16);
            }
            break;
        case 2:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEL_MEM_AREA;
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
             /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEL_MEM_AREA;
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[1].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEL_MEM_AREA;
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[2].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }

            break;

        case 3:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
            endat_pruicss_xchg->cmd[0].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

             /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[1].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_RECEIVE_PARAMETERS;
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[2].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 4:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_PARAMETERS;
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruicss_xchg->cmd[0].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

            /*build separate command for all three channel in loadshare mode */
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_PARAMETERS;
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[1].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_PARAMETERS;
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[2].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 5:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_RESET;
            endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

            /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_RECEIVE_RESET;
                endat_pruicss_xchg->cmd[1].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_RECEIVE_RESET;
                endat_pruicss_xchg->cmd[2].word1 = ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 6:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_TEST_VALUES;
            endat_pruicss_xchg->cmd[0].word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

            /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_TEST_VALUES;
                endat_pruicss_xchg->cmd[1].word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_TEST_VALUES;
                endat_pruicss_xchg->cmd[2].word1 = ENDAT_RX_46BITS | (ENDAT_TX_6BITS << 8) |
                                          (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 7:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruicss_xchg->cmd[0].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

            /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[1].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_RECEIVE_TEST_COMMAND;
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[2].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           (ENDAT_CMDTYP_NO_SUPPLEMENT << 16);
            }
            break;

        case 8:

            /*build separate command for all three channel in loadshare mode*/
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                /*
                * Though this is not 2.1 command, fw expects it to be ENDAT_CMDTYP_2_1. Ideally macro should
                *  have been named ENDAT_CMDTYPE_HAVE_2_2_SUPPLEMENT instead of ENDAT_CMDTYP_2_[12] for readability
                */
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22)
                                               << 16);

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                /*
                * Though this is not 2.1 command, fw expects it to be ENDAT_CMDTYP_2_1. Ideally macro should
                *  have been named ENDAT_CMDTYPE_HAVE_2_2_SUPPLEMENT instead of ENDAT_CMDTYP_2_[12] for readability
                */
                endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22)
                                           << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                /*
                * Though this is not 2.1 command, fw expects it to be ENDAT_CMDTYP_2_1. Ideally macro should
                *  have been named ENDAT_CMDTYPE_HAVE_2_2_SUPPLEMENT instead of ENDAT_CMDTYP_2_[12] for readability
                */
                endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22)
                                           << 16);


            }
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_WITH_DATA;
                /*
                * Though this is not 2.1 command, fw expects it to be ENDAT_CMDTYP_2_1. Ideally macro should
                *  have been named ENDAT_CMDTYPE_HAVE_2_2_SUPPLEMENT instead of ENDAT_CMDTYP_2_[12] for readability
                */
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22)
                                              << 16);
            }



            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                }

            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                }
            }

            break;

        case 9:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;
                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL;
            }
            if(cmd_supplement->has_block_address)
            {
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                                ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                               (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                        16) | (1 << 24);
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address |
                                               (cmd_supplement->block << 24);

                    endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16) | (1 << 24);
                    endat_pruicss_xchg->cmd[1].word2 = cmd_supplement->address |
                                              (cmd_supplement->block << 24);

                    endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16) | (1 << 24);
                    endat_pruicss_xchg->cmd[2].word2 = cmd_supplement->address |
                                              (cmd_supplement->block << 24);

                }
                else
                {
                    endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                        ENDAT_ADDITIONAL_INFO_RX_BITS) |
                       (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                16) | (1 << 24);
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address |
                       (cmd_supplement->block << 24);
                }
            }

            else
            {
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16);
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;

                    endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16);
                    endat_pruicss_xchg->cmd[1].word2 = cmd_supplement->address;

                    endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) |
                                              (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                                                      16);
                    endat_pruicss_xchg->cmd[2].word2 = cmd_supplement->address;

                 }
                else
                {
                    endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                        ENDAT_ADDITIONAL_INFO_RX_BITS) |
                       (ENDAT_TX_6BITS << 8) | ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) <<
                               16);
                    endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;
                }


            }

            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                 if(handle->pru_cfg.load_share_enable)
                 {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                 }
            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                 if(handle->pru_cfg.load_share_enable)
                 {

                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                 }
            }

            break;

        case 10:

            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;
                /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                endat_pruicss_xchg->cmd[0].word2 |= ((cmd_supplement->data & 0xFF) << 16) |
                                           (cmd_supplement->data & 0xFF00);

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[1].word2 = cmd_supplement->address;
                /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                endat_pruicss_xchg->cmd[1].word2 |= ((cmd_supplement->data & 0xFF) << 16) |
                                           (cmd_supplement->data & 0xFF00);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[2].word2 = cmd_supplement->address;
                /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                endat_pruicss_xchg->cmd[2].word2 |= ((cmd_supplement->data & 0xFF) << 16) |
                                           (cmd_supplement->data & 0xFF00);

            }
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;
                /* data MSByte should be @((char *)word2 + 1) and LSByte @((char *)word2 + 2) */
                endat_pruicss_xchg->cmd[0].word2 |= ((cmd_supplement->data & 0xFF) << 16) |
                                           (cmd_supplement->data & 0xFF00);
            }


            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                }

            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                }
            }

            break;

        case 11:
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[1].word2 = cmd_supplement->address;

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[2].word2 = cmd_supplement->address;

            }
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_SEND_PARAM;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;
            }

            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                }
            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                }
            }

            break;

        case 12:
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
            }
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
            }

            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                }
            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                }
            }

            break;

        case 13:
            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[0] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;

                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                endat_pruicss_xchg->cmd[1].word1 = (handle->pos_rx_bits_22_cmd[1] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[1].word2 = cmd_supplement->address;

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                endat_pruicss_xchg->cmd[2].word1 = (handle->pos_rx_bits_22_cmd[2] + info *
                                           ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                          ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[2].word2 = cmd_supplement->address;
            }
            else
            {
                endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD;
                endat_pruicss_xchg->cmd[0].word1 = (handle->pos_rx_bits_22_cmd[handle->current_channel] + info *
                                               ENDAT_ADDITIONAL_INFO_RX_BITS) | (ENDAT_TX_6BITS << 8) |
                                              ((ENDAT_CMDTYP_POSITION | ENDAT_CMDTYP_ENDAT22) << 16);
                endat_pruicss_xchg->cmd[0].word2 = cmd_supplement->address;
            }

            if(handle->flags.info1)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO1) << 16;
                }
            }

            if(handle->flags.info2)
            {
                endat_pruicss_xchg->cmd[0].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                if(handle->pru_cfg.load_share_enable)
                {
                    endat_pruicss_xchg->cmd[1].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;

                    endat_pruicss_xchg->cmd[2].word1 |= (ENDAT_CMDTYP_HAS_ADDINFO2) << 16;
                }
            }

            break;

        case 14:
            endat_pruicss_xchg->cmd[0].word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
            endat_pruicss_xchg->cmd[0].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
            endat_pruicss_xchg->cmd[0].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);

            if(handle->pru_cfg.load_share_enable)
            {
                endat_pruicss_xchg->cmd[1].word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[1].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[1].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);

                endat_pruicss_xchg->cmd[2].word0 = ENDAT_CMD_RECEIVE_COMMUNICATION_CMD;
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->address & 0x80) >> 7)
                                           | (((cmd_supplement->address << 1) & 0xFE) << 8);
                endat_pruicss_xchg->cmd[2].word0 |= ((cmd_supplement->data & 0x8000) >> 7)
                                           | (((cmd_supplement->data << 1) & 0xFF00) << 8) |
                                           (((cmd_supplement->data << 9) & 0xFE00) << 16);
                endat_pruicss_xchg->cmd[2].word1 =  ENDAT_RX_29BITS | (ENDAT_TX_30BITS << 8) |
                                           ((ENDAT_CMDTYP_NO_SUPPLEMENT | ENDAT_CMDTYP_ENDAT22) << 16);
            }
            break;

        default:
            cmd = -EINVAL;
            break;
    }

    return cmd;
}

void endat_command_send(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    /*for load share mode set mask for all connected channels*/
    if(handle->pru_cfg.load_share_enable)
    {
        pruicss_xchg->config[0].trigger = pruicss_xchg->config[0].channel==1?0x1:0;
        pruicss_xchg->config[1].trigger = pruicss_xchg->config[1].channel==2?0x1:0;
        pruicss_xchg->config[2].trigger = pruicss_xchg->config[2].channel==4?0x1:0;

    }
    else
    {
        pruicss_xchg->config[0].trigger = 0x1;
    }

}

void endat_command_wait(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    if(handle->pru_cfg.load_share_enable)
    {  /*waiting till host trigger bits has not clear for all enabled channels*/
        while((pruicss_xchg->config[0].trigger&0x1) || (pruicss_xchg->config[1].trigger &0x1) || (pruicss_xchg->config[2].trigger&0x1))
           ;
    }
    else
    {
        while(pruicss_xchg->config[0].trigger & 0x1)
           ;
    }
}

int32_t endat_command_process(Endat_Handle handle, int32_t cmd,
                          Endat_CmdSupplement *cmd_supplement)
{
    cmd = endat_command_build(handle, cmd, cmd_supplement);

    if(cmd < 0)
    {
        return cmd;
    }

    endat_command_send(handle);
    endat_command_wait(handle);
    return cmd;
}

int32_t endat_get_2_2_angle(Endat_Handle handle)
{
    int32_t pos;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;
    int32_t ch = handle->current_channel;


    if(!(channel_rx_info->ch[ch].crcStatus & ENDAT_CRC_DATA))
    {
        return -1;
    }

    pos = channel_rx_info->ch[ch].posWord0;

#ifdef __TI_ARM__
    pos = __rbit(pos);
#else
    asm("rbit %0,%1" : "=r"(pos) : "r"(pos));
#endif

    /* verify F1 = 0 & F2 = 1 */
    if((pos & 0x3) != 0x2)
    {
        return -1;
    }

    /* discard F1, F2 */
    pos >>= 2;

    /* mask non-angular bits */
    pos &= (1 << handle->single_turn_res[handle->current_channel]) - 1;
    return pos;
}

static int32_t endat_get_pos_res(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(1000 * 12);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_13;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    return word &= (1 << ENDAT_NUM_BITS_VALID_PAGE0_WORD13) - 1;
}

static int32_t endat_get_multi_turn_res(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word1 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_1;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    return word &= (1 << ENDAT_NUM_BITS_VALID_PAGE1_WORD1) - 1;
}

static int32_t endat_get_id(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word0, word1, word2;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word8 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_8;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word0 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word9 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_9;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word1 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word10 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_10;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word2 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    handle->id.binary = word1 | word2 << 16;
    /* swap the two ascii's so that printing as string will give what is required */
    handle->id.ascii = ((word0 & 0xFF) << 8) | ((word0 & 0xFF00) >> 8);

    return 0;
}

static int32_t endat_get_sn(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word0, word1, word2;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 1 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word11 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_11;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word0 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word12 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_12;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word1 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_13;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    word2 = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
            & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    handle->sn.ascii_lsb = word0 & 0xFF;
    handle->sn.binary = ((word0 & 0xFF00) >> 8) | (word1 << 8) | ((
                          word2 & 0xFF) << 24);
    handle->sn.ascii_msb = (word2 & 0xFF00) >> 8;

    return 0;
}

static int32_t endat_get_command_set(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 2 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE2;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word5 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_5;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    handle->cmd_set_2_2 = (word & 0x1) && !(word & 0x2);
    handle->has_safety[ch] = (word & 0x4) && !(word & 0x8);

    return 0;
}

static int32_t endat_get_type(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE0;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw, absence of delay here resulted in wrong values for pos_res */
    ClockP_usleep(1000 * 12);

    /* send parameter for word13 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_14;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);

    word = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);
    handle->type[ch] = (word & (1 << 15)) ? rotary : linear;

    return 0;
}

static int32_t endat_get_step(Endat_Handle handle)
{
    int32_t cmd;
    Endat_CmdSupplement cmd_supplement;
    uint32_t word;
    int32_t ch = handle->current_channel;
    Endat_ChRxInfoArray *channel_rx_info = handle->channel_rx_info;

    /* select memory area encoder manufacturer page 0 */
    cmd = 2, cmd_supplement.address = MRS_CODE_PARAM_ENCODER_MANUFACTURER_PAGE1;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 12);

    /* send parameter for word4 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_4;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);
    word = (channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC)) & ((
                1 << ENDAT_NUM_BITS_PARAMETER) - 1);

    /* send parameter for word5 */
    cmd = 4, cmd_supplement.address = APP_ENDAT_WORD_5;

    if(endat_command_process(handle, cmd, &cmd_supplement) < 0)
    {
        return -EINVAL;
    }

    /* delay copied from fw */
    ClockP_usleep(1000 * 2);
    word |= ((channel_rx_info->ch[ch].posWord0 >> (ENDAT_NUM_BITS_POSITION_CRC))
             & ((1 << ENDAT_NUM_BITS_PARAMETER) - 1)) << 16;

    return word;
}

int32_t endat_get_encoder_info(Endat_Handle handle)
{
    int32_t ret;

    handle->pos_res = endat_get_pos_res(handle);

    if(handle->pos_res < 0)
    {
        return handle->pos_res;
    }

    handle->multi_turn_res[handle->current_channel] = endat_get_multi_turn_res(handle);

    if(handle->multi_turn_res[handle->current_channel] < 0)
    {
        return handle->multi_turn_res[handle->current_channel];
    }

    else if(!handle->multi_turn_res[handle->current_channel])
    {
        handle->single_turn_res[handle->current_channel] = handle->pos_res;
    }

    else
    {
        handle->multi_turn_res[handle->current_channel] = log2(handle->multi_turn_res[handle->current_channel]);
        handle->single_turn_res[handle->current_channel] = handle->pos_res - handle->multi_turn_res[handle->current_channel];
    }

    handle->step[handle->current_channel] = endat_get_step(handle);

    if(handle->step[handle->current_channel] < 0)
    {
        return handle->step[handle->current_channel];
    }
    /*calaculate rx frame size for all three channels and store in different variables*/
  
    handle->pos_rx_bits_21_cmd[handle->current_channel] = handle->pos_res + ENDAT_NUM_BITS_POSITION_CRC +
            ENDAT_NUM_BITS_F1;
    handle->pos_rx_bits_22_cmd[handle->current_channel] = handle->pos_rx_bits_21_cmd[handle->current_channel] + ENDAT_NUM_BITS_F2;

    ret = endat_get_id(handle);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_sn(handle);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_type(handle);

    if(ret)
    {
        return ret;
    }

    ret = endat_get_command_set(handle);

    if(ret)
    {
        return ret;
    }

    return 0;
}

uint32_t endat_get_prop_delay(Endat_Handle handle)
{
    return handle->pruicss_xchg->ch[handle->current_channel].propDelay;
}

void endat_addinfo_track(Endat_Handle handle, int32_t cmd,
                         Endat_CmdSupplement *cmd_supplement)
{
    int32_t c7_c4, c3_c0;

    /* reset stops additional info's */
    if(cmd == 5)
    {
        handle->flags.info1 = FALSE, handle->flags.info2 = FALSE;
    }

    if(cmd != 9)
    {
        return;
    }

    c7_c4 = (cmd_supplement->address & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
            ENDAT_MRS_SHIFT_C7_C4;
    c3_c0 = cmd_supplement->address & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

    if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO1)
    {
        if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
        {
            handle->flags.info1 = FALSE;
        }

        else
        {
            handle->flags.info1 = TRUE;
        }
    }

    c7_c4 = (cmd_supplement->address & ENDAT_MRS_MASK_SELECT_ADDITIONAL_INFO) >>
            ENDAT_MRS_SHIFT_C7_C4;
    c3_c0 = cmd_supplement->address & ENDAT_MRS_MASK_STOP_ADDITIONAL_INFO;

    if(c7_c4 == ENDAT_MRS_VAL_C7_C4_SELECT_ADDITIONAL_INFO2)
    {
        if(c3_c0 == ENDAT_MRS_VAL_STOP_ADDITIONAL_INFO)
        {
            handle->flags.info2 = FALSE;
        }

        else
        {
            handle->flags.info2 = TRUE;
        }
    }
}

static void endat_config_global_rx_arm_cnt(Endat_Handle handle,
        uint16_t val)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    if(handle->pru_cfg.pru_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + 2, val);
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG1_REG + 2, val);
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG1_REG + 2, val);
    }
    else
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + 2, val);
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG1_REG + 2, val);
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG1_REG + 2, val);
    }
}

void endat_config_rx_arm_cnt(Endat_Handle handle, uint16_t val)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    int32_t ch = handle->current_channel;
    if(handle->pru_cfg.pru_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + ch * 8 + 2, val);
    }
    else
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + ch * 8 + 2, val);
    }
}

void endat_config_wire_delay(Endat_Handle handle, uint16_t val)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    int32_t ch = handle->current_channel;
    uint16_t regval;
    if(handle->pru_cfg.pru_slice == 1)
    {
        regval = HW_RD_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG + ch *
                                   8);
    }
    else
    {
        regval = HW_RD_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG + ch *
                                   8);
    }

    /* clear wire delay bits, keep other bits as is */
    regval &= 0xF800;
    /* restrict wire delay to wire delay bits only */
    val &= 0x7FF;
    regval |= val;
    if(handle->pru_cfg.pru_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG + ch * 8, regval);
    }
    else
    {
        HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG + ch * 8, regval);
    }
}

void endat_config_clock(Endat_Handle handle,
                        Endat_ClkCfg_Internal *clk_cfg)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    /* Set PRU1_ED_RX_SB_POL polarity bit, required for ICSSG (don't care for ICSSM) */
    if(handle->pru_cfg.pru_slice == 1)
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG, clk_cfg->rx_div << 16 | handle->clk_cfg->rx_clock_source << 4| 0x8 |
            clk_cfg->rx_div_attr);
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, clk_cfg->tx_div << 16 | handle->clk_cfg->tx_clock_source << 4);
    }
    else
    {
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG, clk_cfg->rx_div << 16 | handle->clk_cfg->rx_clock_source << 4 | 0x8 |
            clk_cfg->rx_div_attr);
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, clk_cfg->tx_div << 16 | handle->clk_cfg->tx_clock_source << 4);
    }
    
    if(handle->pru_cfg.load_share_enable)
    {
        endat_enable_load_share_mode(handle);
    }
}

void endat_config_tst_delay(Endat_Handle handle, uint16_t delay)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    int32_t ch = handle->current_channel;
    if(handle->pru_cfg.pru_slice == 1)
    {
       HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG1_REG + ch * 8, delay);
    }
    else
    {
      HW_WR_REG16((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG1_REG + ch * 8, delay);
    }

}

void endat_config_rx_clock_disable(Endat_Handle handle,
                                   uint16_t val)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    int32_t ch = handle->current_channel;
    pruicss_xchg->ch[ch].rxClkLess = val;
}

static void endat_set_continuous_mode(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    if(handle->pru_cfg.load_share_enable)
    {
        pruicss_xchg->config[0].trigger |= pruicss_xchg->config[0].channel==1?(0x1 << 7 | 0x1):0;
        pruicss_xchg->config[1].trigger |= pruicss_xchg->config[1].channel==2?(0x1 << 7 | 0x1):0;
        pruicss_xchg->config[2].trigger |= pruicss_xchg->config[2].channel==4?(0x1 << 7 | 0x1):0;      
    }
    else
    {
        pruicss_xchg->config[0].trigger |= (0x1 << 7 | 0x1);
    }

}

static void endat_clear_continuous_mode(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;

    if(handle->pru_cfg.load_share_enable)
    {
        pruicss_xchg->config[0].trigger &= ~(0x1 << 7);
        pruicss_xchg->config[1].trigger &= ~(0x1 << 7);
        pruicss_xchg->config[2].trigger &= ~(0x1 << 7);
    }
    else
    {
        pruicss_xchg->config[0].trigger &= ~(0x1 << 7);
    }

}

int32_t endat_start_continuous_mode(Endat_Handle handle)
{
    int32_t cmd;

    cmd = endat_command_build(handle, 1, NULL);

    if(cmd < 0)
    {
        return cmd;
    }

    endat_set_continuous_mode(handle);

    return cmd;
}

void endat_stop_continuous_mode(Endat_Handle handle)
{
    endat_clear_continuous_mode(handle);
    endat_command_wait(handle);
}

void endat_config_host_trigger(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(handle->pru_cfg.load_share_enable)
    {
         pruicss_xchg->config[0].opmode=(pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_HOST_TRIGGER:0;
         pruicss_xchg->config[1].opmode=(pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_HOST_TRIGGER:0;
         pruicss_xchg->config[2].opmode=(pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_HOST_TRIGGER:0;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_HOST_TRIGGER;
    }
}

void endat_config_periodic_trigger_cmp_mode(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(handle->pru_cfg.load_share_enable)
    {
         pruicss_xchg->config[0].opmode=(pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[0].opmode;
         pruicss_xchg->config[1].opmode=(pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[1].opmode;
         pruicss_xchg->config[2].opmode=(pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_CMP_PERIODIC:pruicss_xchg->config[2].opmode;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_CMP_PERIODIC;
    }
}

void endat_config_periodic_trigger_cap_mode(Endat_Handle handle)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;
    /*for loadshare mode trigger set based on connected channels*/
    if(handle->pru_cfg.load_share_enable)
    {
         pruicss_xchg->config[0].opmode = (pruicss_xchg->config[0].channel&(1<<0))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[0].opmode;
         pruicss_xchg->config[1].opmode = (pruicss_xchg->config[1].channel&(1<<1))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[1].opmode;
         pruicss_xchg->config[2].opmode = (pruicss_xchg->config[2].channel&(1<<2))?ENDAT_OPMODE_CAP_PERIODIC:pruicss_xchg->config[2].opmode;
    }
    else
    {
        pruicss_xchg->config[0].opmode = ENDAT_OPMODE_CAP_PERIODIC;
    }
}

void endat_config_channel(Endat_Handle handle, int32_t ch)
{
    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;

    pruicss_xchg->config[0].channel = 1 << ch;
    handle->current_channel = ch;
}

void endat_config_multi_channel_mask(Endat_Handle handle,
                                     uint8_t mask,
                                     uint8_t loadshare)
{
    handle->pru_cfg.load_share_enable = loadshare;

    if(loadshare)
    {
        handle->pruicss_xchg->config[0].channel = mask&(1<<0);
        handle->pruicss_xchg->config[1].channel = mask&(1<<1);
        handle->pruicss_xchg->config[2].channel = mask&(1<<2);
        endat_config_primary_core_mask(handle, mask);/*select primary core for global configuration*/
        endat_config_syn_bits(handle, mask); /* configure syn bits for synchronization before any global config*/
        endat_enable_load_share_mode(handle); /*ENABLE SHARE MODE*/
    }
    else
    {
       handle->pruicss_xchg->config[0].channel = mask;
    }
}

void endat_config_syn_bits(Endat_Handle handle, uint8_t mask)
{
    handle->pruicss_xchg->endat_ch0_syn_bit=mask&(1<<0)?0x1:0;
    handle->pruicss_xchg->endat_ch1_syn_bit=mask&(1<<1)?0x2:0;
    handle->pruicss_xchg->endat_ch2_syn_bit=mask&(1<<2)?0x4:0;
}
void endat_enable_load_share_mode(Endat_Handle handle)
{
   void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    //HW_WR_REG32(0x30026104) |= 0x0800;
    uint32_t rgval;
    if(handle->pru_cfg.pru_slice == 1)
    {
       rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
       rgval |= ENDAT_LOAD_SHARE_EN_MASK;
      HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        rgval |= ENDAT_LOAD_SHARE_EN_MASK;
      HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, rgval);
    }

}
void endat_config_primary_core_mask(Endat_Handle handle, uint8_t mask)
{
    switch (mask)
    {

        case 1:  /*only channel0 connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x1;
                        break;
        case 2: /*channel1 connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x2;
                        break;
        case 3:               /*channel0 and channel1 connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x1;
                        break;
        case 4:  /*channel2 connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x4;
                        break;
        case 5:               /*channel0 and channel2 connnected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x4;
                        break;
        case 6:                    /*channel1 and channel2 connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0X4;
                        break;
        case 7:                       /*all three channel connected*/
            handle->pruicss_xchg->endat_primary_core_mask=0x4;
                        break;

     }

}
uint8_t endat_multi_channel_detected(Endat_Handle handle)
{      if(handle->pru_cfg.load_share_enable)  /* for loadshare mode*/
        return (handle->pruicss_xchg->config[0].channel|handle->pruicss_xchg->config[1].channel|handle->pruicss_xchg->config[2].channel);
       else
       return handle->pruicss_xchg->config[0].channel;

}

void endat_multi_channel_set_cur(Endat_Handle handle, int32_t ch)
{
    handle->current_channel = ch;
    handle->pos_res =  handle->pos_rx_bits_21_cmd[ch] - (ENDAT_NUM_BITS_POSITION_CRC + ENDAT_NUM_BITS_F1);
}

int32_t endat_wait_initialization(Endat_Handle handle, uint32_t timeout, uint8_t mask)
{
    int32_t i;

    Endat_PruicssXchg *pruicss_xchg = handle->pruicss_xchg;

    for(i = 0; i < timeout; i++)
    {


        if(handle->pru_cfg.load_share_enable)  /* for loadshare mode*/
        {
            switch (mask) {

             case 1:  /*channel 0 connected*/
                     if((pruicss_xchg->config[0].status & 1))
                         return 0;

                    break;
             case 2: /*channel 1 connected*/
                       if((pruicss_xchg->config[1].status & 1))
                          return 0;
                    break;
             case 3:               /*channel 0 and 1 connected*/
                    if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[1].status & 1))
                          return 0;
                    break;
             case 4:  /*channel 2 connected*/
                       if((pruicss_xchg->config[2].status & 1))
                           return 0;
                    break;
             case 5:               /*channel 0 and 2 connnected*/
                 if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[2].status & 1))
                           return 0;
                    break;
             case 6:                    /*channel 1 and 2 connected*/
                 if((pruicss_xchg->config[1].status & 1)&&(pruicss_xchg->config[2].status & 1))
                            return 0;
                    break;
             case 7:                       /*all three channel connected*/
                 if((pruicss_xchg->config[0].status & 1)&&(pruicss_xchg->config[1].status & 1)&&( pruicss_xchg->config[2].status & 1))
                            return 0;
                    break;
            }
            ClockP_usleep(1000 * 1);

        }
        else if(pruicss_xchg->config[0].status & 1)
        {
            break;
        }
        else
        {
            ClockP_usleep(1000 * 1);
        }
    }

    if(i == timeout)
    {
        return -1;
    }

  return 0;
}

static inline void endat_config_clr_cfg0(Endat_Handle handle)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    if(handle->pru_cfg.pru_slice)
    {
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG, 0);
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG, 0);
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG, 0);
    }
    else
    {
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG, 0);
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG, 0);
       HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG, 0);
    }
}

static inline void endat_config_endat_mode(Endat_Handle handle)
{
    void *pruicss_cfg = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->cfgRegBase;
    if(handle->pru_cfg.pru_slice == 1)
    {
       HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG1_REG + 3, 4);
    }
    else
    {
       HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG0_REG + 3, 4);
    }
}

static void endat_hw_init(Endat_Handle handle)
{
    Endat_ClkCfg_Internal clk_cfg;
    /* set initial clock to 200KHz */
    if(handle->clk_cfg->rx_clock_source == 1)
    {
        clk_cfg.rx_div = handle->pru_cfg.pru_clock /(ENDAT_RX_OVERSAMPLING_RATE*(ENDAT_INIT_FREQ)) - 1;
    }
    else
    {
        clk_cfg.rx_div = handle->pru_cfg.uart_clock /(ENDAT_RX_OVERSAMPLING_RATE*(ENDAT_INIT_FREQ)) - 1;
    }

    if(handle->clk_cfg->tx_clock_source == 1)
    {
        clk_cfg.tx_div =handle->pru_cfg.pru_clock/ENDAT_INIT_FREQ - 1;
    }
    else
    {
        clk_cfg.tx_div = handle->pru_cfg.uart_clock/ENDAT_INIT_FREQ - 1;
    }
    /* 2T */
    clk_cfg.rx_en_cnt = ENDAT_DELAY_COUNTER_INCREMENT*((2*handle->pru_cfg.pru_clock)/ENDAT_INIT_FREQ);
    /* sample size 8 */
    clk_cfg.rx_div_attr = ENDAT_RX_OVERSAMPLING_RATE - 1;

    endat_config_endat_mode(handle);
    endat_config_clock(handle, &clk_cfg);
    endat_config_global_rx_arm_cnt(handle, clk_cfg.rx_en_cnt);
    endat_config_clr_cfg0(handle);
}

Endat_Handle endat_init(uint32_t index, Endat_Params endat_params)
{
    Endat_Handle handle = NULL;
    void *base_addr = NULL;

    if(index >=  gEndatConfigNum)
    {
        return NULL;
    }
    
    handle = (Endat_Handle)(&gEndatHandle[index]);

    handle->instance_index = index;
    
    if(endat_params.pru_cfg.pruicss_handle == NULL) {
        /* Return NULL if required PRU handle is NULL */
        return NULL;
    }
    
    if(endat_params.pru_cfg.pru_slice == 1)
    {
        handle->pruicss_xchg = (Endat_PruicssXchg *)((PRUICSS_HwAttrs *)(endat_params.pru_cfg.pruicss_handle->hwAttrs))->pru1DramBase;
    }
    else if(endat_params.pru_cfg.pru_slice == 0)
    {
        handle->pruicss_xchg = (Endat_PruicssXchg *)((PRUICSS_HwAttrs *)(endat_params.pru_cfg.pruicss_handle->hwAttrs))->pru0DramBase;
    }
    else
    {
        /* Return NULL if invalid pru slice is provided */
        return NULL;
    }
   /* Check valid range for IEP comparator (0-15), clocks must be positive */
   if((endat_params.pru_cfg.pru_clock <= 0) ||
      (endat_params.pru_cfg.uart_clock <= 0) || 
      (endat_params.pru_cfg.iep_clock <= 0) || 
      (endat_params.pru_cfg.iep_instance < 0 || endat_params.pru_cfg.iep_instance > 1))
    {
        /* Return NULL if invalid parameters */
        return NULL;
    }
    handle->pru_cfg = endat_params.pru_cfg;

    if((endat_params.endat_clk_config->tx_clock_source > 1 || endat_params.endat_clk_config->tx_clock_source < 0) || 
       (endat_params.endat_clk_config->rx_clock_source > 1 || endat_params.endat_clk_config->rx_clock_source < 0) || 
       (endat_params.endat_clk_config->rx_os_rate < 0 || endat_params.endat_clk_config->rx_os_rate > 7))
    {
        /* Return NULL if invalid clock parameters */
        return NULL;
    }
    /* Configure Delays based on the PRU ICSS frequency*/
    handle->pruicss_xchg->endat_delay_125ns = ((endat_params.pru_cfg.pru_clock*125)/1000000000);
    handle->pruicss_xchg->endat_delay_51us = ((endat_params.pru_cfg.pru_clock*51)/1000000 );
    handle->pruicss_xchg->endat_delay_5us = ((endat_params.pru_cfg.pru_clock*5)/1000000);
    handle->pruicss_xchg->endat_delay_1ms = ((endat_params.pru_cfg.pru_clock/1000) * 1);
    handle->pruicss_xchg->endat_delay_2ms = ((endat_params.pru_cfg.pru_clock/1000) * 2);
    handle->pruicss_xchg->endat_delay_12ms = ((endat_params.pru_cfg.pru_clock/1000) * 12);
    handle->pruicss_xchg->endat_delay_50ms = ((endat_params.pru_cfg.pru_clock/1000) * 50);
    handle->pruicss_xchg->endat_delay_380ms = ((endat_params.pru_cfg.pru_clock/1000) * 380);
    handle->pruicss_xchg->endat_delay_900ms = ((endat_params.pru_cfg.pru_clock/1000) * 900);
    handle->pruicss_xchg->icss_clk = endat_params.pru_cfg.pru_clock;
    
    handle->clk_cfg = endat_params.endat_clk_config;
    handle->channel_rx_info = endat_params.channel_rx_info;
   
    /*Write Configured memory address to DMEM */
    handle->pruicss_xchg->ch_info_memory_add = endat_params.ch_info_global_addr;

    /*Set IEP base address */
    base_addr = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->baseAddr;

    handle->pruicss_xchg->endat_iep_base_addr = ((uint32_t)endat_params.pru_cfg.iep_base_addr) - ((uint32_t)base_addr);
    endat_hw_init(handle);
    return handle;
}

uint32_t endat_get_recovery_time(Endat_Handle handle)
{
    return handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.recoveryTime*((float)(1000000000)/handle->pru_cfg.pru_clock);
}

int8_t endat_check_rt_error(Endat_Handle handle)
{
    uint32_t rtDiff;
    uint32_t lastCounterValue;
    uint32_t currentCounterValue;

    lastCounterValue =  handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue;
    currentCounterValue = handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue;

    /*Check if the counter is stuck by comparing current and last counter values*/
    if(currentCounterValue == lastCounterValue)
    {
       handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.isCounterStuck = 1 ;
       return RT_COUNTER_STUCK_ERROR;
    }
    else
    {
        handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.isCounterStuck = 0 ;
    }

    /*handle the int overflow candition */
    if(lastCounterValue > currentCounterValue)
    {
        rtDiff = (MAX_RT_COUNTER_VALUE - lastCounterValue) + currentCounterValue;
    }
    else
    {
        rtDiff = currentCounterValue - lastCounterValue;
    }

    /*convert into us */
    rtDiff = rtDiff*((float)(1000000000)/handle->pru_cfg.pru_clock);
    /* Check if the recovery time is within the short or long recovery time range */
    if ((rtDiff >= SHORT_RECOVERY_TIME_MIN) && (rtDiff <= SHORT_RECOVERY_TIME_MAX)) {
        return RT_NO_ERROR;  /* Valid short recovery time */
    }
    if ((rtDiff >= LONG_RECOVERY_TIME_MIN) && (rtDiff <= LONG_RECOVERY_TIME_MAX)) {
        return RT_NO_ERROR;  /* Valid long recovery time */
    }
    return RT_OUT_OF_RANGE_ERROR;  /*Error: out of expected range*/
}

void endat_init_rt_measurement (Endat_Handle handle) 
{
    handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.recoveryTime = 0;
    handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.lastCounterValue = 0;
    handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue = RT_COUNTER_STARTING_VALUE + 100*(handle->current_channel); /* Assign a unique starting value for each channel; */
    handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.currentCounterValue = handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.startingValue;
    handle->channel_rx_info->ch[handle->current_channel].recoveryTimeParms.isCounterStuck = 0;
}
void endat_enable_rt_measurement (Endat_Handle handle)
{
    handle->pruicss_xchg->ch[handle->current_channel].enableRTM = 1;
}
void endat_disable_rt_measurement (Endat_Handle handle)
{
    handle->pruicss_xchg->ch[handle->current_channel].enableRTM = 0;
}
uint32_t endat_status_rt_measurement (Endat_Handle handle)
{
    return handle->pruicss_xchg->ch[handle->current_channel].enableRTM;
}

int32_t endat_config_iep_cap_event(Endat_Handle handle, uint8_t channel, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint32_t reg0;
    void *base_addr = (void *)((PRUICSS_HwAttrs *)(handle->pru_cfg.pruicss_handle->hwAttrs))->baseAddr;

    if(event_num > 7 || pru_iep == NULL)
    {
        ret_val = SystemP_FAILURE;
        return ret_val;
    }
    else
    {
        /* Configure the cap event */
        /* Read the current register value */
        reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));
        /* Set the CAP_EN bit (OR with the new value) */
        reg0 |= ((uint32_t)1U << event_num);
        /* Write back the modified value */
        HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);

        /* Read the current register value */
        reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));
        /* Set the CAP_EN bit (OR with the new value) */
        reg0 |= ((uint32_t)1U << event_num);
        /* Write back the modified value */
        HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
    }
    
    if(handle->pru_cfg.load_share_enable)
    {
        /*write cap even and address in dmem */
        handle->pruicss_xchg->trigger_params[channel].iep_event_number = event_num;
        handle->pruicss_xchg->trigger_params[channel].iep_capture_reg = (uint32_t)pru_iep - ((uint32_t)base_addr) + CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0 + 8*(event_num);
        if(event_num > 6)
        {
            handle->pruicss_xchg->trigger_params[channel].iep_capture_reg += 8;
        }
    }
    else{
        /* write cap event and address in dmem */
        /* Always 0 in single PRU mode. When load share mode is disabled.
        In single PRU mode firmware, the channel number is ignored and the firmware always reads data from DMEM using the channel 0 offset, regardless of which channels are connected.*/
        handle->pruicss_xchg->trigger_params[0].iep_event_number = event_num;
        handle->pruicss_xchg->trigger_params[0].iep_capture_reg = (uint32_t)pru_iep - ((uint32_t)base_addr) + CSL_ICSS_PR1_IEP0_SLV_CAP0_REG0 + 8*(event_num);
        if(event_num > 6)
        {
            handle->pruicss_xchg->trigger_params[0].iep_capture_reg += 8;
        }
    }
    
    return ret_val;
}
int32_t endat_config_iep_cmp_event(Endat_Handle handle, uint8_t channel, uint64_t trigger_point, uint8_t event_num)
{
   
    int32_t ret_val = SystemP_SUCCESS;
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint32_t reg0;
    uint32_t reg1;

    if(event_num > 15 || pru_iep == NULL)
    {
        ret_val = SystemP_FAILURE;
        return ret_val;
    }
    else
    {
        /* Configure the cmp event */
        /* Read the current register value */
        reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
        /* Set the CMP_EN bit (OR with the new value) */
        reg0 |= ((uint32_t)1U << event_num) << 1;
        /* Write back the modified value */
        HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);
        reg0 = (trigger_point & 0xFFFFFFFF) - handle->pru_cfg.iep_increment;
        reg1 = (trigger_point >> 32 & 0xFFFFFFFF);
        if(event_num > 7)
        {
            HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*8 + 8),  reg0);
            HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*8 + 8),  reg1);
        }
        else
        {
            HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0 + event_num*8),  reg0);
            HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1 + event_num*8),  reg1);
        }
        /* Write in dmem */
        if(handle->pru_cfg.load_share_enable)
        {
            handle->pruicss_xchg->trigger_params[channel].iep_event_number = event_num;
        }
        else
        {
            /* Always 0 in single PRU mode. When load share mode is disabled.
            In single PRU mode firmware, the channel number is ignored and the firmware always reads data from DMEM using the channel 0 offset, regardless of which channels are connected.*/
            handle->pruicss_xchg->trigger_params[0].iep_event_number = event_num;
        }
    }

    return ret_val;
}

int32_t endat_disable_cmp_event(Endat_Handle handle, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint32_t reg0;

    if(event_num > 15 || pru_iep == NULL)
    {
        ret_val = SystemP_FAILURE;
        return ret_val;
    }
    else
    {
        /* Disable the cmp event */
        /* Read the current register value */
        reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG));
        /* Clear the CMP_EN bit (AND with the negated new value) */
        reg0 &= ~(((uint32_t)1U << event_num) << IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);
        /* Write back the modified value */
        HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG), reg0);
    }

    return ret_val;
}

int32_t endat_disable_cap_event(Endat_Handle handle, uint8_t event_num)
{
    int32_t ret_val = SystemP_SUCCESS;
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint32_t reg0;

    if(event_num > 7 || pru_iep == NULL)
    {
        ret_val = SystemP_FAILURE;
        return ret_val;
    }
    else
    {
        /* Disable the cap event */
        /* Read the current register value */
        reg0 = HW_RD_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG));
        /* Clear the CAP_EN bit (AND with the negated new value) */
        reg0 &= ~((uint32_t)1U << event_num);
        /* Write back the modified value */
        HW_WR_REG32(((uint8_t *)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CAP_CFG_REG), reg0);
    }

    return ret_val;
}
int32_t endat_enable_iep_reset_on_cmp0(Endat_Handle handle, uint64_t iep_reset_count)
{
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint16_t event;
    uint32_t event_clear;
    uint32_t reg0;
    uint32_t reg1;

    if(handle == NULL || pru_iep == NULL)
    {
        return SystemP_FAILURE;
    }

    reg0 = (iep_reset_count & 0xFFFFFFFF) - handle->pru_cfg.iep_increment;
    reg1 = (iep_reset_count >> 32 & 0xFFFFFFFF);

    HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG0),  reg0);
    HW_WR_REG32((uint8_t*)pru_iep + (CSL_ICSS_PR1_IEP0_SLV_CMP0_REG1),  reg1);

    /* Read CMP CFG register */
    event = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG);
    event_clear = HW_RD_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG);

    /* Enable IEP reset by CMP0 event */
    event |= (1 << IEP_SLV_CMP_CFG_REG_CMP_EN_SHIFT);  /* CMP0 enable bit */
    event |= (1 << IEP_SLV_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT);  /* Reset counter enable bit */
    event_clear |= 1;

    /* Clear event */
    HW_WR_REG32((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_STATUS_REG, event_clear);
    /* Enable event */
    HW_WR_REG16((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_CMP_CFG_REG, event);

    return SystemP_SUCCESS;
}

int32_t endat_enable_iep_counter(Endat_Handle handle)
{
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint8_t temp;

    if(handle == NULL || pru_iep == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Write IEP default increment & IEP start */
    temp = HW_RD_REG8((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0x0F;
    temp |=  (handle->pru_cfg.iep_increment << CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT);  /* Default increment enable */
    temp |= 0x01;  /* Counter enable */
    HW_WR_REG8((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    return SystemP_SUCCESS;
}

int32_t endat_disable_iep_counter(Endat_Handle handle)
{
    void *pru_iep = handle->pru_cfg.iep_base_addr;
    uint8_t temp;

    if(handle == NULL || pru_iep == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Clear IEP counter enable bit */
    temp = HW_RD_REG8((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG);
    temp &= 0xFE;  /* Clear counter enable bit (bit 0) */
    HW_WR_REG8((uint8_t*)pru_iep + CSL_ICSS_PR1_IEP0_SLV_GLOBAL_CFG_REG, temp);

    return SystemP_SUCCESS;
}
