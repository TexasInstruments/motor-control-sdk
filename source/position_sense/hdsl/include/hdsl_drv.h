/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 *  "AS IS" AND ANY EXPS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#ifndef HDSL_DRV_H_
#define HDSL_DRV_H_

/**
 *  \defgroup HDSL_API_MODULE APIs for HDSL Encoder
 *  \ingroup POSITION_SENSE_API
 *
 * Here is the list of APIs used for HDSL Encoder communication protocol
 *
 *  @{
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>

#include <kernel/dpl/DebugP.h>

#include <drivers/pruicss.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


#define HDSL_MAX_CHANNELS           (3U)
#define CHANNEL_0_ENABLED           (1<<0)
#define CHANNEL_1_ENABLED           (1<<1)
#define CHANNEL_2_ENABLED           (1<<2)

/**
 *  \anchor HDSL_LongMessageAddrTypes
 *  \name HDSL Long Message Addressing Types
 *
 *
 *  @{
 */
/** \brief Direct addressing of long messages */
#define HDSL_LONG_MSG_ADDR_DIRECT   (0U)
/** \brief Indirect addressing of long messages */
#define HDSL_LONG_MSG_ADDR_INDIRECT (1U)
/** @} */

/**
 *  \anchor HDSL_LongMessageAddrOffsetModes
 *  \name HDSL Long Message Addressing with/without offset
 *
 *
 *  @{
 */
/** \brief Addressing of long messages without offset */
#define HDSL_LONG_MSG_ADDR_WITHOUT_OFFSET   (0U)
/** \brief Addressing of long messages with offset */
#define HDSL_LONG_MSG_ADDR_WITH_OFFSET (1U)
/** @} */


/**
 *  \anchor HDSL_LongMessageLengths
 *  \name HDSL Long Message Data Lengths
 *
 *
 *  @{
 */
/** \brief No data bytes */
#define HDSL_LONG_MSG_LENGTH_0   (0U)
/** \brief 2 data bytes */
#define HDSL_LONG_MSG_LENGTH_2   (1U)
/** \brief 4 data bytes */
#define HDSL_LONG_MSG_LENGTH_4   (2U)
/** \brief 8 data bytes */
#define HDSL_LONG_MSG_LENGTH_8   (3U)
/** @} */

#define MAX_WAIT 20000

#define HDSL_ICSSG0_INST        0U
#define HDSL_ICSSG1_INST        1U

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))
#define HWREGB(x)                                                              \
        (*((volatile uint8_t *)(x)))
#define HWREGH(x)                                                              \
        (*((volatile uint16_t *)(x)))
/*TSR configuration:*/

/*inEvent value:*/
/* ICSSG_0_EDC1_SYNC0 ICSSG0 IEP1 sync event 0 Pulse */
#define SYNCEVENT_INTRTR_IN_27 27

/*outEvent values:*/
/*SYNC0_OUT Pin Selectable timesync event 24 Edge (4+(24*4)) */
#define SYNCEVT_RTR_SYNC28_EVT 0x64
/* SYNC1_OUT Pin Selectable timesync event 25 Edge (4+(25*4)) */
#define SYNCEVT_RTR_SYNC29_EVT 0x68
/* SYNC2_OUT Pin Selectable timesync event 26 Edge (4+(26*4)) */
#define SYNCEVT_RTR_SYNC30_EVT 0x6C
/* SYNC3_OUT Pin Selectable timesync event 27 Edge (4+(27*4)) */
#define SYNCEVT_RTR_SYNC31_EVT 0x70
/* ICSSG0_PR1_EDC1_LATCH0_IN PRU_ICSSG0 (4+(10*4)) */
#define SYNCEVT_RTR_SYNC10_EVT 0x2C

enum {
    MENU_SAFE_POSITION,
    MENU_QUALITY_MONITORING,
    MENU_EVENTS,
    MENU_SUMMARY,
    MENU_ACC_ERR_CNT,
    MENU_RSSI,
    MENU_PC_SHORT_MSG_WRITE,
    MENU_PC_SHORT_MSG_READ,
    MENU_DIRECT_READ_RID0_LENGTH4,
    MENU_DIRECT_READ_RID81_LENGTH8,
    MENU_DIRECT_READ_RID81_LENGTH2,
    MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0,
    MENU_INDIRECT_WRITE_RID0_LENGTH8,
    MENU_HDSL_REG_INTO_MEMORY,
    MENU_LIMIT,
    MENU_INVALID,
};

typedef struct HDSL_Config_s         *HDSL_Handle;

/* ========================================================================== */
/*                  Structure Declarations & Definitions                      */
/* ========================================================================== */

/**
 * \anchor  HDSL_Interface
 * \name    HDSL Master Register Interface Parameters Structure
 * @{
 */
typedef struct {
    volatile uint8_t SYS_CTRL;          /**< System control */
    volatile uint8_t SYNC_CTRL;         /**< Synchronization control */
    volatile uint8_t resvd0;            /**< Reserved 0 */
    volatile uint8_t MASTER_QM;         /**< Quality monitoring */
    volatile uint8_t EVENT_H;           /**< High bytes event */
    volatile uint8_t EVENT_L;           /**< Low bytes event */
    volatile uint8_t MASK_H;            /**< High byte event mask */
    volatile uint8_t MASK_L;            /**< Low byte event mask */
    volatile uint8_t MASK_SUM;          /**< Summary mask */
    volatile uint8_t EDGES;             /**< Cable bit sampling time control */
    volatile uint8_t DELAY;             /**< Run time delay of system cable and signal strength */
    volatile uint8_t VERSION;           /**< Version */
    volatile uint8_t resvd1;            /**< Reserved 1 */
    volatile uint8_t ENC_ID2;           /**< Encoder ID, byte 2 */
    volatile uint8_t ENC_ID1;           /**< Encoder ID, byte 1 */
    volatile uint8_t ENC_ID0;           /**< Encoder ID, byte 0 */
    volatile uint8_t POS4;              /**< Fast position, byte 4 */
    volatile uint8_t POS3;              /**< Fast position, byte 3 */
    volatile uint8_t POS2;              /**< Fast position, byte 2 */
    volatile uint8_t POS1;              /**< Fast position, byte 1 */
    volatile uint8_t POS0;              /**< Fast position, byte 0 */
    volatile uint8_t VEL2;              /**< Speed, byte 2 */
    volatile uint8_t VEL1;              /**< Speed, byte 1 */
    volatile uint8_t VEL0;              /**< Speed, byte 0 */
    volatile uint8_t resvd2;            /**< Reserved 2 */
    volatile uint8_t VPOS4;             /**< Safe position, byte 4 */
    volatile uint8_t VPOS3;             /**< Safe position, byte 3 */
    volatile uint8_t VPOS2;             /**< Safe position, byte 2 */
    volatile uint8_t VPOS1;             /**< Safe position, byte 1 */
    volatile uint8_t VPOS0;             /**< Safe position, byte 0 */
    volatile uint8_t VPOSCRC_H;         /**< CRC of Safe position, byte 1 */
    volatile uint8_t VPOSCRC_L;         /**< CRC of Safe position, byte 0 */
    volatile uint8_t PC_BUFFER0;        /**< Parameters channel buffer, byte 0 */
    volatile uint8_t PC_BUFFER1;        /**< Parameters channel buffer, byte 1 */
    volatile uint8_t PC_BUFFER2;        /**< Parameters channel buffer, byte 2 */
    volatile uint8_t PC_BUFFER3;        /**< Parameters channel buffer, byte 3 */
    volatile uint8_t PC_BUFFER4;        /**< Parameters channel buffer, byte 4 */
    volatile uint8_t PC_BUFFER5;        /**< Parameters channel buffer, byte 5 */
    volatile uint8_t PC_BUFFER6;        /**< Parameters channel buffer, byte 6 */
    volatile uint8_t PC_BUFFER7;        /**< Parameters channel buffer, byte 7 */
    volatile uint8_t PC_ADD_H;          /**< Long message address, byte 1 */
    volatile uint8_t PC_ADD_L;          /**< Long message address, byte 0 */
    volatile uint8_t PC_OFF_H;          /**< Long message address offset, byte 1 */
    volatile uint8_t PC_OFF_L;          /**< Long message address offset, byte 0 */
    volatile uint8_t PC_CTRL;           /**< Parameters channel control */
    volatile uint8_t PIPE_S;            /**< Sensor hub channel status */
    volatile uint8_t PIPE_D;            /**< Sensor hub channel data */
    volatile uint8_t PC_DATA;           /**< Short message parameters channel data */
    volatile uint8_t resvd3;            /**< Reserved 3 */
    volatile uint8_t resvd4;            /**< Reserved 4 */
    volatile uint8_t resvd5;            /**< Reserved 5 */
    volatile uint8_t resvd6;            /**< Reserved 6 */
    volatile uint8_t resvd7;            /**< Reserved 7 */
    volatile uint8_t SAFE_CTRL;         /**< Safe System Control */
    volatile uint8_t SAFE_SUM;          /**< Summarized slave status */
    volatile uint8_t S_PC_DATA;         /**< Response of Short message parameters channel Read for safe1 channel */
    volatile uint8_t ACC_ERR_CNT;       /**< Fast position error counter */
    volatile uint8_t resvd8;            /**< Reserved 8 */
    volatile uint8_t resvd9;            /**< Reserved 9 */
    volatile uint8_t resvd10;           /**< Reserved 10 */
    volatile uint8_t resvd11;           /**< Reserved 11 */
    volatile uint8_t EVENT_S;           /**< Safe Events */
    volatile uint8_t MASK_S;            /**< Safe Event Mask */
    volatile uint8_t DUMMY;             /**< Dummy, no data */
    volatile uint8_t SLAVE_REG_CTRL;    /**< Short message control */
    volatile uint8_t ACC_ERR_CNT_THRESH;/**< Fast position error counter threshold */
    volatile uint8_t resvd12;           /**< Reserved 12 */
    volatile uint8_t resvd13;           /**< Reserved 13 */
    /*Safe 2 Interface */
    volatile uint8_t VERSION2;          /**< Version in Safe Channel 2 */
    volatile uint8_t ENC2_ID;           /**< Encoder ID in Safe Channel 2 */
    volatile uint8_t STATUS2;           /**< Safe Channel 2 Status */
    volatile uint8_t VPOS24;            /**< Safe Position 2, byte 4 */
    volatile uint8_t VPOS23;            /**< Safe Position 2, byte 3 */
    volatile uint8_t VPOS22;            /**< Safe Position 2, byte 2 */
    volatile uint8_t VPOS21;            /**< Safe Position 2, byte 1 */
    volatile uint8_t VPOS20;            /**< Safe Position 2, byte 0 */
    volatile uint8_t VPOSCRC2_H;        /**< CRC of Safe Position 2, byte 1 */
    volatile uint8_t VPOSCRC2_L;        /**< CRC of Safe Position 2, byte 0 */
    volatile uint8_t POSTX;             /**< Position transmission status */
    volatile uint8_t resvd14;           /**< Reserved 14 */
	/* Online Status*/
	volatile uint8_t ONLINE_STATUS_D_H; /**< Online Status D, high byte*/
    volatile uint8_t ONLINE_STATUS_D_L; /**< Online Status D, low byte*/
	volatile uint8_t ONLINE_STATUS_1_H; /**< Online Status 1, high byte*/
    volatile uint8_t ONLINE_STATUS_1_L; /**< Online Status 1, low byte*/
	volatile uint8_t ONLINE_STATUS_2_H; /**< Online Status 2, high byte*/
    volatile uint8_t ONLINE_STATUS_2_L; /**< Online Status 2, low byte*/
} HDSL_Interface;
/** @} */

/**
 * \anchor  HDSL_Config
 * \name    HDSL Config Parameters Structure
 * @{
 */
typedef struct HDSL_Config_s {
    PRUICSS_Handle icssgHandle;
    /**< PRUICSS_Handle for icssg0 or icssg1 instance*/
    uint32_t icssCore;
    /**< PRUICSS core identifier
     * Check PRUICSS_PRU0 and check other available macros
    */
    uint32_t *baseMemAddr; // icssgHandle->hwAttrs->baseAddr + PRUICSS_DATARAM(PRUICSS_PRUx)
    /**< Base Memory Address for HDSL channel configuration */
    HDSL_Interface *hdslInterface;
    /**< HDSL master memory interface structure */
    uint32_t multi_turn;
    /**< HDSL ----- */
    uint32_t res;
    /**< HDSL ----- */
    uint64_t mask;
    /**< HDSL ----- */

    // intc_initdata // ** - needs to be common for all channels and fixed (configure for all 3 channels in starting)

} HDSL_Config;
/** @} */

/**
 * \anchor  HDSL_CopyTable
 * \name    HDSL Copy Table for overlay scheme used when channel 2 is enabled.
 * @{
 */

typedef struct HDSL_CopyTable_s {
    uint32_t reserved1;
    uint32_t loadAddr1; /**< Load Address of Part 1 of firmware */
    uint32_t runAddr1;  /**< Run Address of Part 1 of firmware */
    uint32_t size1;     /**< Size of Part 1 of firmware */
    uint32_t reserved2;
    uint32_t loadAddr2; /**< Load Address of Part 2 of firmware */
    uint32_t runAddr2;  /**< Run Address of Part 2 of firmware */
    uint32_t size2;     /**< Size of Part 2 of firmware */
} HDSL_CopyTable;
/** @} */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
/**
 *  \brief     enable load share mode for multi-channel HDSL
 *
 *  \param[in]  pruCfg    Cfg base register address
 *  \param[in]  pruSlice   PRU slice, 1 for PRU1 and 0 for PRU0
 *
 */
void hdsl_enable_load_share_mode(void *pruCfg ,uint32_t pruSlice);

/**
 *  \brief      Open HDSL handle for the specified core
 *              (interrupt mapping should already be completed)
 *
 *  \param[in]  icssgHandle PRUICSS_Handle for the ICSS instance
 *  \param[in]  icssCore    Core to map in ICSSG instance
 *  \param[in]  pruMode    0 for load share mode disabled, 1 for load share mode enabled
 *  \retval     HDSL_Handle
 *
 */
HDSL_Handle HDSL_open(PRUICSS_Handle icssgHandle, uint32_t icssCore, uint8_t pruMode);

/**
 *  \brief      Initialize IEP and Use OCP as IEP CLK src
 *
 *  \param[in]  hdslHandle
 *
 */
void HDSL_iep_init(HDSL_Handle hdslHandle);

/**
 *  \brief      Enable IEP            <br>
 *              *Enable SYNC0 and program pulse width   <br>
 *              Enable cyclic mod   <br>
 *              Program CMP1     <br>
 *              TSR configuration <br>
 *
 *  \param[in]  ES
 *  \param[in]  period
 *  \retval     1 for successful enable sync signal
 *
 */
int HDSL_enable_sync_signal(uint8_t ES, uint32_t period);

/**
 *  \brief      Calculate fast position,safe position1,safe position2
 *
 *  \param[in]  hdslHandle
 *  \param[in]  position_id
 *  \retval     position value in integer for successful position return, -1 for error in position return
 *
 */
uint64_t HDSL_get_pos(HDSL_Handle hdslHandle, int position_id);

/**
 *  \brief      Getting quality monitoring value
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer QM value
 */
uint8_t HDSL_get_qm(HDSL_Handle hdslHandle);

/**
*  \brief       Taking values of High bytes event (EVENT_H) and Low bytes event(EVENT_L)
 *
 *  \param[in]  hdslHandle
 *  \retval     16 bit integer concatenated values of both EVENT_H,EVENT_L
 *
 */
uint16_t HDSL_get_events(HDSL_Handle hdslHandle);

/**
*  \brief       Taking values of Safe Event (EVENT_S) register
 *
 *  \param[in]  hdslHandle

 *  \retval       8 bit integer values of EVENT_S
 *
 */
uint8_t HDSL_get_safe_events(HDSL_Handle hdslHandle);


/**
*  \brief       Taking values of Online Status D (ONLINE_STATUS_D) register
 *
 *  \param[in]  hdslHandle

 *  \retval       16 bit integer value of ONLINE_STATUS_D
 *
 */
uint16_t HDSL_get_online_status_d(HDSL_Handle hdslHandle);

/**
*  \brief       Taking values of Online Status D (ONLINE_STATUS_D) register
 *
 *  \param[in]  hdslHandle

 *  \retval       16 bit integer value of ONLINE_STATUS_D
 *
 */
uint16_t HDSL_get_online_status_1(HDSL_Handle hdslHandle);

/**
*  \brief       Taking values of Online Status D (ONLINE_STATUS_D) register
 *
 *  \param[in]  hdslHandle

 *  \retval       16 bit integer value of ONLINE_STATUS_D
 *
 */
uint16_t HDSL_get_online_status_2(HDSL_Handle hdslHandle);

/**
*  \brief       Getting Summarized slave status
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of summarized status
 *
 */
uint8_t HDSL_get_sum(HDSL_Handle hdslHandle);

/**
 *  \brief      Acceleration error counter
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of acceleration error counter
 *
 */
uint8_t HDSL_get_acc_err_cnt(HDSL_Handle hdslHandle);

/**
*  \brief       Read RSSI value
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit RSSI integer value
 *
 */
uint8_t HDSL_get_rssi(HDSL_Handle hdslHandle);

/**
 *  \brief  Trigger a short message write operation using parameters channel
 *
 *  \param[in]  hdslHandle
 *  \param[in]  addr    Address
 *  \param[in]  data    Data
 *  \param[in]  timeout Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_TIMEOUT in case of timeout
 *
 */
int32_t HDSL_write_pc_short_msg(HDSL_Handle hdslHandle, uint8_t addr, uint8_t data, uint64_t timeout);

/**
 *  \brief      Trigger a short message read operation using parameters channel
 *
 *  \param[in]  hdslHandle
 *  \param[in]  addr    Address
 *  \param[in]  data    Pointer to data buffer where read data will be stored
 *  \param[in]  timeout Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_TIMEOUT in case of timeout
 *
 */
int32_t HDSL_read_pc_short_msg(HDSL_Handle hdslHandle, uint8_t addr, uint8_t *data, uint64_t timeout);

/**
 *  \brief  Trigger a long message write operation using parameters channel.
 *          Call \ref HDSL_write_pc_buffer before this to write the data to be sent using long message.
 *
 *  \param[in]  hdslHandle
 *  \param[in]  addr            10 bit address for long message
 *  \param[in]  offsetEnable    Addressing with offset enable/disable from \ref HDSL_LongMessageAddrOffsetModes
 *  \param[in]  addrType        Addressing Type from \ref HDSL_LongMessageAddrTypes
 *  \param[in]  length          Length from \ref HDSL_LongMessageLengths
 *  \param[in]  offset          15 bit address offset for long message (if offset is enabled in offsetEnable parameter)
 *  \param[in]  timeout         Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_FAILURE in case of error, SystemP_TIMEOUT in case of timeout
 *
 */
int32_t HDSL_write_pc_long_msg(HDSL_Handle hdslHandle, uint16_t addr, uint8_t offsetEnable, uint8_t addrType, uint8_t length, uint16_t offset, uint64_t timeout);

/**
 *  \brief      Trigger a long message read operation using parameters channel
 *              If this API returns SystemP_SUCCESS, call \ref HDSL_read_pc_buffer after this to read the data received using long message.
 *  \param[in]  hdslHandle
 *  \param[in]  addr            10 bit address for long message
 *  \param[in]  offsetEnable    Addressing with offset enable/disable from \ref HDSL_LongMessageAddrOffsetModes
 *  \param[in]  addrType        Addressing Type from \ref HDSL_LongMessageAddrTypes
 *  \param[in]  length          Length from \ref HDSL_LongMessageLengths
 *  \param[in]  offset          15 bit address offset for long message
 *  \param[in]  timeout         Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_FAILURE in case of error, SystemP_TIMEOUT in case of timeout
 *
 */
int32_t HDSL_read_pc_long_msg(HDSL_Handle hdslHandle, uint16_t addr, uint8_t offsetEnable, uint8_t addrType, uint8_t length, uint16_t offset, uint64_t timeout);

/**
 *  \brief      Write Parameters channel buffer for different bytes(bytes 0-7)
 *
 *  \param[in]  hdslHandle
 *  \param[in]  buff_off
 *  \param[in]  data
 *
 */
void HDSL_write_pc_buffer(HDSL_Handle hdslHandle, uint8_t buff_off, uint8_t data);

/**
 *  \brief      Returns Parameters channel buffer for different bytes(bytes 0-7)
 *
 *  \param[in]  hdslHandle
 *  \param[in]  buff_off
 *  \retval     8 bit integer value of PC_BUFFER from hdsl interface
 *
 */
uint8_t HDSL_read_pc_buffer(HDSL_Handle hdslHandle, uint8_t buff_off);

/**
 *  \brief      Returns Synchronization control value
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of SYNC_CTRL from hdsl interface
 *
 */

uint8_t HDSL_get_sync_ctrl(HDSL_Handle hdslHandle);

/**
 *  \brief      Writes Synchronization control value
 *
 *  \param[in]  hdslHandle
 *  \param[in]  val
 *
 */
void HDSL_set_sync_ctrl(HDSL_Handle hdslHandle, uint8_t val);

/**
 *  \brief      Returns Quality monitoring value
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of MASTER_QM from hdsl interface
 *
 */
uint8_t HDSL_get_master_qm(HDSL_Handle hdslHandle);

/**
 *  \brief      Returns Cable bit sampling time control
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of EDGES from hdsl interface
 *
 */
uint8_t HDSL_get_edges(HDSL_Handle hdslHandle);

/**
 *  \brief      Returns Run time delay of system cable and signal strength
 *
 *  \param[in]  hdslHandle
 *  \retval     8 bit integer value of DELAY from hdsl interface
 *
 */
uint8_t HDSL_get_delay(HDSL_Handle hdslHandle);

/**
 *  \brief      Read encoder id bytes(byte no. 0-2)
 *
 *  \param[in]  hdslHandle
 *  \param[in]  byte
 *  \retval     8 bit encoder bytes data from hdsl interface
 *
 */
uint8_t HDSL_get_enc_id(HDSL_Handle hdslHandle, int byte);

/**
 *  \brief      Generates memory image
 *
 *  \param[in]  hdslHandle
 *
 */
void HDSL_generate_memory_image(HDSL_Handle hdslHandle);

/**
 *  \brief      Get memory location for HDSL interface struct
 *
 *  \param[in]  hdslHandle
 *  \retval     Pointer containing base memeory address
 *
 */
void* HDSL_get_src_loc(HDSL_Handle hdslHandle);

/**
 *  \brief      Get size of memory used by HDSL interface struct
 *
 *  \param[in]  hdslHandle
 *  \retval     Size of the struct
 *
 */
uint32_t HDSL_get_length(HDSL_Handle hdslHandle);

/**
 *  \brief      Configure the copy table entries for two overlayed firmware parts for channel 2
 *
 *  \param[in]  hdslHandle
 *  \param[in]  copyTable
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_FAILURE in case of error
 *
 */
int32_t HDSL_config_copy_table(HDSL_Handle hdslHandle, HDSL_CopyTable *copyTable);


/**
 *  \brief      Configure the channel mask
 *
 *  \param[in]  hdslHandle
 *  \param[in]  channelMask
 *
 *  \return     SystemP_SUCCESS in case of success, SystemP_FAILURE in case of error
 *
 */
int32_t HDSL_config_channel_mask(HDSL_Handle hdslHandle, uint8_t channelMask);

#ifdef __cplusplus
}
#endif

/** @} */
#endif
