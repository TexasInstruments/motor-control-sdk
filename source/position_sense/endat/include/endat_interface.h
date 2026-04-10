/*
 *  Copyright (C) 2021-2026 Texas Instruments Incorporated
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
/**    \brief    2.1 receive parameter */
#define ENDAT_CMD_RECEIVE_PARAMETERS    (0x70 >> 1)
/**    \brief    2.1 send parameter */
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

/* Maximum number of EnDat Encoders connected with one PRU Slice*/
#define ENDAT_NUM_CH_PER_SLICE_MAX                    (3)

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 *  \brief EnDAT CRC error tracking information
 *
 *  \details This structure maintains CRC error counters for different data fields
 *           received from the encoder. Counters are maintained by PRU firmware and
 *           wrap around after 255 errors.
 */
typedef struct endat_crc_info_s
{
    volatile uint8_t err_cnt_data;
    /**< CRC error count for position/data field.
     *   Increments on each CRC failure for position data
     *   Wraps around to 0 after 255 errors */

    volatile uint8_t err_cnt_addinfox;
    /**< CRC error count for additional information (info1 or info2).
     *   Used when only one additional info field is present
     *   Wraps around to 0 after 255 errors */

    volatile uint8_t err_cnt_addinfo1;
    /**< CRC error count specifically for additional info1.
     *   Only applicable when both addinfo1 and addinfo2 are present
     *   Wraps around to 0 after 255 errors */

    volatile uint8_t resvd_int1;
    /**< Reserved  */
} endat_crc_info;
/**
 *  \brief EnDAT channel configuration and status information
 *
 *  \details This structure contains per-channel configuration and runtime status
 *           information shared between the host and PRU firmware. It includes
 *           encoder capabilities, propagation delay compensation, and CRC error tracking.
 *
 *           This structure is part of the PRU-ICSS shared memory interface and is
 *           accessed by both the ARM host and PRU firmware cores.
 */
typedef struct endat_ch_info_s
{
    volatile uint8_t num_clk_pulse;
    /**< Number of clock pulses for position data transfer.
     *   Excludes start bit, error bits, and CRC bits
     *   Updated during encoder initialization based on encoder resolution
     *   Used by PRU firmware to determine receive window timing */

    volatile uint8_t endat22_stat;
    /**< EnDAT 2.2 command set support status.
     *   0 = Encoder supports EnDAT 2.1 only
     *   1 = Encoder supports EnDAT 2.2 command set
     *   Determined during encoder identification */

    volatile uint16_t rx_clk_less;
    /**< RX clock reduction for propagation delay compensation.
     *   Number of receive clock cycles to subtract to account for
     *   cable propagation delay and encoder processing time */

    volatile uint32_t prop_delay;
    /**< Automatically estimated propagation delay in PRU clock cycles.
     *   Measured by PRU firmware during initialization */

    endat_crc_info crc;
    /**< CRC error tracking information for this channel. */

    volatile uint32_t enable_rtm;
    /**< Recovery time counter flag.
     *   0 = Recovery time counter disabled
     *   1 = Recovery time counter enabled
     *   Controls recovery time counter  */
} endat_ch_info;
/**
 *  \brief EnDAT channel recovery time parameters
 *
 *  \details This structure contains parameters for encoder
 *           recovery time.
 */
typedef struct endat_ch_rt_info_s
{
    volatile uint32_t recovery_time;
    /**< Measured recovery time in PRU clock cycles. */

    volatile uint32_t current_counter_value;
    /**< update counter value after current measurement.
     *   Updated by PRU firmware for each transaction */

    volatile uint32_t last_counter_value;
    /**< Recovery Time counter value until current measurement. */

    volatile uint32_t starting_value;
    /**< Initial recovery time counter value at start of measurement.
     *   Reference point for recovery time calculation */

    volatile uint8_t is_counter_stuck;
    /**< Counter stuck detection flag.
     *   0 = Counter incrementing normally
     *   1 = Counter appears to be stuck  */
} endat_ch_rt_info;
/**
 *  \brief EnDAT channel received data structure
 *
 *  \details This structure stores the raw received data from an encoder for a single
 *           channel. It contains the position words, additional information words,
 *           CRC validation status, and recovery time information.
 *
 *           This structure is written by PRU firmware after each encoder transaction
 *           and read by the host driver for data processing and validation.
 */
typedef struct endat_ch_rx_info_s
{
    volatile uint32_t pos_word0;
    /**< First position data word (up to 32 bits). */

    volatile uint32_t pos_word1;
    /**< Second position data word (if applicable).
     *   Contains position bits beyond the first 32 bits
     *   Used for high-resolution encoders (>32 bit position)
     *   Zero if not applicable */

    volatile uint32_t pos_word2;
    /**< Additional information word 1 or 2.
     *   If both addinfo1 and addinfo2 are present, this contains addinfo2
     *   If only one addinfo is present, this contains that addinfo
     *   Content depends on encoder and command type */

    volatile uint32_t pos_word3;
    /**< Additional information word 1 (when both present).
     *   Only valid if both addinfo1 and addinfo2 are received
     *   Zero if not applicable */

    volatile uint8_t crc_status;
    /**< CRC validation status bitfield.
     *   Bit 0: Position/data CRC status (1=pass, 0=fail)
     *   Bit 1: Additional info1 CRC status (1=pass, 0=fail)
     *   Bit 2: Additional info2 CRC status (1=pass, 0=fail)
     *   Updated by PRU firmware after CRC calculation */

    endat_ch_rt_info recovery_time_parms;
    /**< Recovery time parameters for this channel. */
} endat_ch_rx_info;

/**
 *  \brief EnDAT command interface
 *
 *  \details This structure defines the command interface for sending EnDAT commands
 *           from the host to the PRU firmware. It contains the packed command word,
 *           command parameters, and command supplement data.
 *
 *           The host writes to this structure to initiate encoder commands, and the
 *           PRU firmware reads and executes the commands.
 */
typedef struct endat_pruicss_cmd_s
{
    volatile uint32_t word0;
    /**< Packed command word containing command code, address, and parameter bits.
     *   Byte 0 bit 7:   Dummy bit (0)
     *   Byte 0 bit 6-1: EnDAT command code
     *   Byte 0 bit 0:   Address bit 7
     *   Byte 1 bit 7-1: Address bits 6-0
     *   Byte 1 bit 0:   Parameter bit 15
     *   Byte 2 bit 7-0: Parameter bits 14-7
     *   Byte 3 bit 7-1: Parameter bits 6-0
     *   Byte 3 bit 0:   Dummy bit (0) */

    volatile uint32_t word1;
    /**< Command parameters and attributes.
     *   Byte 0: Number of receive bits (includes start bit and dummy bits for addinfo)
     *   Byte 1: Number of transmit bits
     *   Byte 2: Command attributes bitfield:
     *           bit 0: Command supplement flag (1=no supplement, 0=supplement present)
     *           bit 1: Position command flag (1=position, 0=not position)
     *           bit 2: EnDAT version flag (1=2.2 command, 0=2.1 command)
     *           bit 3: Additional info1 present flag (1=present, 0=not present)
     *           bit 4: Additional info2 present flag (1=present, 0=not present)
     *   Byte 3: Block address selection flag (1=selected, 0=not selected) */

    volatile uint32_t word2;
    /**< Command supplement data for MRS (Memory Read Select) commands.
     *   Byte 0: Memory address
     *   Byte 1: Parameter MSB (most significant byte)
     *   Byte 2: Parameter LSB (least significant byte)
     *   Byte 3: Block address */
} endat_pruicss_cmd;

/**
 *  \brief EnDAT configuration interface
 *
 *  \details This structure defines the configuration and control interface for
 *           the EnDAT. It contains operational mode settings,
 *           channel selection, command triggers, and initialization status.
 *
 *           This structure is bidirectional - the host writes configuration
 *           and trigger bits, while the PRU firmware writes status information.
 */
typedef struct endat_pruicss_config_s
{
    volatile uint8_t opmode;
    /**< Operational mode selection.
     *   0 = Periodic trigger mode with IEP compare event
     *   1 = Host trigger mode (software-initiated commands)
     *   2 = Periodic trigger mode with IEP capture event
     *   Set by host before starting firmware operations */

    volatile uint8_t channel;
    /**< Channel mask for encoder selection.
     *   Bit 0: Channel 0 enable (1 << 0)
     *   Bit 1: Channel 1 enable (1 << 1)
     *   Bit 2: Channel 2 enable (1 << 2)
     *   Must be set before running firmware
     *   After initialization, reflects detected/active channels
     *   Multi-channel mode: up to 3 channels (0x1-0x7)
     *   Single-channel mode: exactly 1 channel (0x1, 0x2, or 0x4) */

    volatile uint8_t trigger;
    /**< Command trigger and continuous mode control.
     *   Bit 0 (LSB): Command trigger (1=send command, cleared by firmware on completion)
     *   Bit 7 (MSB): Continuous clock mode (1=start, 0=stop)
     *   Note: Command must be set up in endat_pruicss_cmd before setting trigger bit
     *   For continuous mode, both LSB and MSB must be set to start */

    volatile uint8_t status;
    /**< Firmware initialization status.
     *   0 = Initialization in progress or failed
     *   1 = Initialization successful, ready for commands
     *   Written by PRU firmware after initialization sequence
     *   Host should wait approximately 5 seconds after firmware start to verify status */
} endat_pruicss_config;

/**
 *  \brief EnDAT periodic trigger mode configuration
 *
 *  \details This structure contains configuration parameters for periodic trigger
 *           mode operation. In periodic mode, the IEP timer automatically triggers
 *           encoder position reads at regular intervals without host intervention.
 */
typedef struct endat_periodic_trigger_cfg_s
{
    uint8_t cmp_event;
    /**< IEP compare event number for this channel (0-15).
     *   Used when opmode=0 (IEP compare trigger mode)
     *   Specifies which IEP compare event triggers position updates
     *   The selected event triggers automatic position readout */

    uint8_t cap_event;
    /**< IEP capture event number for this channel (0-7).
     *   Used when opmode=2 (IEP capture trigger mode) */

    uint16_t reserved;
    /**< Reserved for alignment */

    uint32_t iep_capture_reg;
    /**< IEP capture register value for capture mode.
     *   Only used when opmode=2 (IEP capture mode) */
} endat_periodic_trigger_cfg;

/**
 *  \brief EnDAT exchange interface
 *
 *  \details This structure defines the shared memory interface between the host
 *           processor and PRU firmware for EnDAT communication. It contains
 *           configuration, command, and data exchange structures along with
 *           timing parameters required by the firmware.
 *
 *           This structure is mapped to PRU shared memory and accessed by both
 *           the host driver and PRU firmware for bidirectional communication.
 */
typedef struct endat_pruicss_xchg_s
{
    endat_pruicss_config config[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Per-channel configuration interface.
     *   Contains operational parameters for each of the 3 channels */

    endat_pruicss_cmd cmd[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Per-channel command interface for EnDAT protocol commands.
     *   Host writes commands, PRU executes and writes status
     *   Bidirectional communication structure */

    endat_ch_info ch[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Per-channel status and data exchange interface.
     *   Contains received encoder data, error flags, and channel status */

    uint64_t ch_info_memory_add;
    /**< Global memory address of channel RX info structure.
     *   Allows PRU direct access to write received data to host memory
     *   Set by host during initialization */

    uint64_t reserved;
    /**< Reserved for alignment */

    uint32_t endat_delay_125ns;
    /**< PRU counts for 125 nanosecond delay.
     *   Calculated based on PRU clock frequency */

    uint32_t endat_delay_5us;
    /**<  PRU counts for 5 microsecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_51us;
    /**< PRU counts for 51 microsecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_1ms;
    /**< PRU counts for 1 millisecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_2ms;
    /**< PRU counts for 2 millisecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_12ms;
    /**< PRU counts for 12 millisecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_50ms;
    /**< PRU counts for 50 millisecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_380ms;
    /**< PRU counts for 380 millisecond delay. Calculated based on PRU clock frequency. */

    uint32_t endat_delay_900ms;
    /**< PRU counts for 900 millisecond delay. Calculated based on PRU clock frequency. */

    volatile uint8_t endat_primary_core_mask;
    /**< Bitmask indicating which PRU core is primary in load-share mode to execute global reinit.
     *
     *   bitmask values:
     *   0x1 - RTU is primary
     *   0x2 - PRU is primary
     *   0x4 - TXPRU is primary
     */

    volatile uint8_t endat_ch0_syn_bit;
    /**< Synchronization bit for channel 0.
     *   Used in load share mode to ensure channel 0 is ready for global reinit */

    volatile uint8_t endat_ch1_syn_bit;
    /**< Synchronization bit for channel 1.
     *   Used in load share mode to ensure channel 1 is ready for global reinit */

    volatile uint8_t endat_ch2_syn_bit;
    /**< Synchronization bit for channel 2.
     *   Used in load share mode to ensure channel 2 is ready for global reinit */

    uint64_t icss_clk;
    /**< PRU-ICSS core clock frequency in Hz.
     *   Used by firmware for timing calculations */

    uint32_t endat_iep_base_addr;
    /**< IEP (Industrial Ethernet Peripheral) timer base address.
     *   Used for periodic trigger mode */

    endat_periodic_trigger_cfg trigger_params[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Per-channel periodic trigger configuration. */
} endat_pruicss_xchg;
/**
 *  \brief EnDAT channel receive information array
 *
 *  \details This structure contains an array of receive information structures
 *           for all EnDAT channels. It holds the decoded encoder data received
 *           from each channel including position, additional info, CRC status,
 *           and error flags.
 *
 *           This structure is allocated by the application and passed to the
 *           driver during initialization. The PRU firmware writes received
 *           encoder data directly to this structure.
 */
typedef struct endat_ch_rx_info_array_s
{
    endat_ch_rx_info ch[ENDAT_NUM_CH_PER_SLICE_MAX];
    /**< Array of per-channel receive information structures.
     *   Index 0 = Channel 0, Index 1 = Channel 1, Index 2 = Channel 2
     *   Each element contains decoded position, addinfo, and status data */
} endat_ch_rx_info_array;

#ifdef __cplusplus
}
#endif

#endif
