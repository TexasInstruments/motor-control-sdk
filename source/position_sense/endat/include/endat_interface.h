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

#ifndef ENDAT_INTERFACE_H_
#define ENDAT_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros                                           */
/* ========================================================================== */

/**    \brief    2.1 send position value */
#define ENDAT_CMD_SEND_POSITION_VALUES  (0x1C >> 1)
/**    \brief    2.1 select memory area */
#define ENDAT_CMD_SEL_MEM_AREA          (0x38 >> 1)
/**    \brief    2.1 receive paramter */
#define ENDAT_CMD_RECEIVE_PARAMETERS    (0x70 >> 1)
/**    \brief    2.1 send paramter */
#define ENDAT_CMD_SEND_PARAMETERS       (0x8C >> 1)
/**    \brief    2.1 receive reset */
#define ENDAT_CMD_RECEIVE_RESET         (0xA8 >> 1)
/**    \brief    2.1 send test values */
#define ENDAT_CMD_SEND_TEST_VALUES      (0x54 >> 1)
/**    \brief    2.1 receive test command */
#define ENDAT_CMD_RECEIVE_TEST_COMMAND  (0xC4 >> 1)

/**    \brief    2.2 send position value with addinfo(s) */
#define ENDAT_CMD_SEND_POSVAL_WITH_DATA         (0xE0 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & select memory area */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_MEMSEL    (0x24 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive parameter */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_PARAM     (0x6C >> 1)
/**    \brief    2.2 send position value with addinfo(s) & send parameter */
#define ENDAT_CMD_SEND_POSVAL_SEND_PARAM        (0x90 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive test command */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_TESTCMD   (0xD8 >> 1)
/**    \brief    2.2 send position value with addinfo(s) & receive error reset */
#define ENDAT_CMD_SEND_POSVAL_RECEIVE_ERR_RST   (0xB4 >> 1)
/**    \brief    2.2 receive communication command */
#define ENDAT_CMD_RECEIVE_COMMUNICATION_CMD     (0x48 >> 1)

/**    \brief    command has no command supplement */
#define ENDAT_CMDTYP_NO_SUPPLEMENT  0x1
/**    \brief    position command */
#define ENDAT_CMDTYP_POSITION       (0x1 << 1)
/**    \brief    command belongs to EnDat 2.2 command set */
#define ENDAT_CMDTYP_ENDAT22        (0x1 << 2)
/**    \brief    2.2 position command with additional info 1 */
#define ENDAT_CMDTYP_HAS_ADDINFO1   (0x1 << 3)
/**    \brief    2.2 position command with additional info 2 */
#define ENDAT_CMDTYP_HAS_ADDINFO2   (0x1 << 4)

/**    \brief    position/data CRC status mask */
#define ENDAT_CRC_DATA      (0x1 << 0)
/**    \brief    additional info 1/2 CRC status mask (if either one present) <br>
                 if both present, indicates additional info 2 CRC status mask */
#define ENDAT_CRC_ADDINFOX  (0x1 << 1)
/**    \brief    additional info 1 CRC status mask (if both present) */
#define ENDAT_CRC_ADDINFO1  (0x1 << 2)


/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 *    \brief    Structure defining per channel CRC information
 *
 *    \details  Firmware per channel CRC information interface
 */
typedef struct Endat_CrcInfo_s
{
     volatile uint8_t  errCntData;
     /**< CRC position/data error count (will wraparound after 255) */
     volatile uint8_t  errCntAddinfox;
     /**< CRC additional info1/2 error count (will wraparound after 255) */
     volatile uint8_t  errCntAddinfo1;
     /**< CRC additional info1 error count (will wraparound after 255)     <br>
          applicable only when both additional info's are present */
     volatile uint8_t   resvdInt1;
     /**< reserved */
}Endat_CrcInfo;
/**
 *  \brief    Structure defining EnDat channel Info
 *  \details  Firmware per channel interface 
 * 
 * 
*/
typedef struct Endat_ChInfo_s
{
     volatile uint8_t  numClkPulse;
     /**< position bits excluding SB, error, CRC (updated upon initialization) */
     volatile uint8_t  endat22Stat;
     /**< encoder command set type, 1 - 2.2 supported, 0 - 2.2 not supported  */
     volatile uint16_t rxClkLess;
     /**< receive clocks to be reduced to handle propagation delay (to be  <br>
          updated by host, if applicable) */
     volatile uint32_t   propDelay;
     /**< automatically estimated propagation delay */
     Endat_CrcInfo crc;
     /**<Crc information*/
     volatile uint32_t   resvdInt0;
     /**< reserved */

}Endat_ChInfo;
/**
 * \brief    Structure defining EnDat channel Rx Info 
 * \details  Firmware per channel interface for store Rx data (command response)
 * 
 * 
 * 
*/
typedef struct Endat_ChRXInfo_s
{
     volatile uint32_t   posWord0;
     /**< Initial (<=32) position bits received including error bits */
     volatile uint32_t   posWord1;
     /**< position bits received after the initial 32 bits (if applicable) */
     volatile uint32_t   posWord2;
     /**< additional info 1/2 (will be additional info 2 if both present) */
     volatile uint32_t   posWord3;
     /**< additional info 1 (if both additional 1 & 2 present) */
     volatile uint8_t    crcStatus;
     /**< CRC status,
         bit0: 1 - position/data success, 0 - position/data failure       <br>
         bit1: 1 - additional info1 success, 0 - additioanl info1 failure <br>
         bit2: 1 - additional info2 success, 0 - additioanl info2 failure */
     volatile uint8_t resvdInt2;
     /**< reserved */
     volatile uint16_t resvdInt3;
     /**< reserved */
     volatile uint32_t   recoveryTime;
     /*< Recovery Time */
   

}Endat_ChRxInfo;

/**
 *    \brief    Structure defining EnDat command interface
 *
 *    \details  Firmware command interface
 */
struct endat_pruss_cmd
{
    volatile uint32_t   word0;
    /**< command,                                                         <br>
         [Byte 0] bit 7: 0(dummy), bit 6-1: command, bit 0: address bit 7 <br>
         [Byte 1] bit 7-1: address bit 6-0, bit 0: parameter bit 15       <br>
         [Byte 2] bit 7-0: parameter bit 14-7                             <br>
         [Byte 2] bit 7-1: parameter bit 6-0, bit 0: 0(dummy) */
    volatile uint32_t   word1;
    /**< command parameters,                                              <br>
         [Byte 0] receive bits, includes SB & dummy (for additional info) for PRU <br>
         [Byte 1] transmit bit                                            <br>
         [Byte 2] attributes,                                             <br>
          bit0: 1 - no command supplement, 0 - command supplement present <br>
          bit1: 1 - position command, 0 - not position command            <br>
          bit2: 1 - EnDat 2.2 command, 0 - EnDat 2.1 command              <br>
          bit3: 1 - additional info1 present, 0 - no additional info1     <br>
          bit4: 1 - additional info2 present, 0 - no additional info2     <br>
         [Byte 3] 1 - block address selected, 0 - block address not selected */
    volatile uint32_t   word2;
    /**< command supplement,                                              <br>
         [Byte 0] address                                                 <br>
         [Byte 1] parameter MSByte                                        <br>
         [Byte 2] parameter LSByte                                        <br>
         [Byte 3] block address */

};

/**
 *    \brief    Structure defining EnDat configuration interface
 *
 *    \details  Firmware configuration interface
 */
struct endat_pruss_config
{
    volatile uint8_t  opmode;
    /**< operation mode selection: 0 - periodic trigger, 1 - host trigger */
    volatile uint8_t  channel;
    /**< channel mask (1 << channel), 0 < channel < 3. This has to be      <br>
         selected before running firmware. Once initialization is complete,<br>
         it will reflect the detected channels in the selected mask.       <br>
         Multichannel can have upto 3 selected, while single channel only one */
    volatile uint8_t  trigger;
    /**< command trigger. Set LSB to send cmd, will be cleared upon cmd    <br>
         completion. Set/clear MSB to start/stop continuous clock mode.    <br>
         To start continuous mode LSB also has to be set. Note that cmd    <br>
         has to be setup before trigger */
    volatile uint8_t  status;
    /**< initialization status: 1 - upon successful. Wait around 5 seconds <br>
         after firmware has started running to confirm status */
};

/**
 *    \brief    Structure defining EnDat interface
 *
 *    \details  Firmware config, command interface
 *
 */
struct endat_pruss_xchg
{
     struct endat_pruss_config   config[3];
     /**< config interface */
     struct endat_pruss_cmd      cmd[3];
     /**< command interface */
     Endat_ChInfo ch[3];
     /**<channel interface */
     uint64_t endatChInfoMemoryAdd;
     uint16_t endat_rx_clk_config;
     uint16_t endat_tx_clk_config;
     uint32_t endat_rx_clk_cnten;
     uint32_t endat_delay_125ns;
     uint32_t endat_delay_5us;
     uint32_t endat_delay_51us;
     uint32_t endat_delay_1ms;
     uint32_t endat_delay_2ms;
     uint32_t endat_delay_12ms;
     uint32_t endat_delay_50ms;
     uint32_t endat_delay_380ms;
     uint32_t endat_delay_900ms;
     volatile uint8_t endat_primary_core_mask;
     volatile uint8_t endat_ch0_syn_bit;
     volatile uint8_t endat_ch1_syn_bit;
     volatile uint8_t endat_ch2_syn_bit;
     uint64_t icssg_clk;
};
/**
 *    \brief    Structure defining EnDat channel Rx information 
 *
 *    \details   
 *
 */
struct endatChRxInfo
{
    Endat_ChRxInfo ch[3];
};
#ifdef __cplusplus
}
#endif

#endif
