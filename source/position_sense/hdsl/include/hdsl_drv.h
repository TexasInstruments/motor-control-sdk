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

/**
 *  \section HDSL_VALIDATION_STRATEGY Driver Validation Strategy
 *
 *  \subsection hdsl_validation_policy Validation Policy
 *
 *  **Handle Parameter Validation:**
 *  - All public APIs validate the handle parameter for NULL
 *  - Returns appropriate error value if handle is invalid:
 *    - Functions returning int32_t status: SystemP_FAILURE
 *    - Functions returning pointers: NULL
 *    - Functions returning void: early return
 *  - This catches programming errors where uninitialized handles are used
 *
 *  **Array Bounds and Index Validation:**
 *  - APIs with array parameters or index parameters perform bounds checking
 *  - Examples: buff_off (0-7), byte (0-2), position_id (0-2)
 *  - Prevents buffer overruns and out-of-bounds memory access
 *
 *  **Internal Structure Validation:**
 *  - All APIs validate internal structure pointers before dereferencing them
 *  - Each function validates only the pointers it uses.
 *  - Provides protection against NULL pointer dereferences
 *
 *  **Pointer Parameter Validation:**
 *  - Output pointer parameters (position, data, copy_table) are checked for NULL
 *  - Ensures safe dereferencing before writing output data
 *
 *  This validation strategy:
 *  - Validates all pointers used in each function before dereferencing
 *  - Provides error detection and graceful failure handling
 *
 *  \subsection hdsl_validation_assumptions Safety Guidelines
 *  1. Always call \ref HDSL_open successfully before using other APIs
 *  2. Check return values of all API calls for error detection
 *  3. Do not modify internal driver structures (priv, attrs) directly
 *  4. Ensure SysConfig-generated configuration is correct
 *
 *  The validation strategy ensures graceful failure even if these
 *  guidelines are not followed, returning appropriate error codes rather than
 *  causing undefined behavior.
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

/** \brief Maximum number of HDSL channels supported per PRU-ICSS slice */
#define HDSL_NUM_CH_PER_SLICE_MAX   (3U)

/**
 *  \anchor HDSL_OperationalModes
 *  \name HDSL Operational Modes (FREE_RUN vs SYNC)
 *
 *  @{
 */
/** \brief Free-run operational mode - encoder operates continuously without external synchronization */
#define HDSL_OPERATIONAL_MODE_FREE_RUN         (0U)

/** \brief Synchronized operational mode - encoder synchronized to external SYNC pulse */
#define HDSL_OPERATIONAL_MODE_SYNC             (1U)
/** @} */

/**
 *  \anchor HDSL_PruIcssTypes
 *  \name HDSL PRU-ICSS Types (PRU-ICSSG on AM243x, or PRU-ICSSM on AM261x)
 *
 *  @{
 */
/** \brief PRU-ICSSG on AM243x */
#define HDSL_PRU_ICSSG              (0U)

/** \brief PRU-ICSSM on AM261x */
#define HDSL_PRU_ICSSM              (1U)
/** @} */

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
#define HDSL_LONG_MSG_ADDR_WITH_OFFSET      (1U)
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

/**
 *  \brief Menu options for HDSL diagnostic application
 */
typedef enum HDSL_MenuOption_e
{
    MENU_SAFE_POSITION,                               /**< Read safe position data */
    MENU_QUALITY_MONITORING,                          /**< Read quality monitoring value */
    MENU_EVENTS,                                      /**< Read event registers */
    MENU_SUMMARY,                                     /**< Read summary information */
    MENU_ACC_ERR_CNT,                                 /**< Read acceleration error count */
    MENU_RSSI,                                        /**< Read RSSI value */
    MENU_PC_SHORT_MSG_WRITE,                          /**< Write short message via parameters channel */
    MENU_PC_SHORT_MSG_READ,                           /**< Read short message via parameters channel */
    MENU_DIRECT_READ_RID0_LENGTH4,                    /**< Direct read register ID 0, 4-byte length */
    MENU_DIRECT_READ_RID81_LENGTH8,                   /**< Direct read register ID 81, 8-byte length */
    MENU_DIRECT_READ_RID81_LENGTH2,                   /**< Direct read register ID 81, 2-byte length */
    MENU_INDIRECT_WRITE_RID0_LENGTH8_OFFSET0,         /**< Indirect write register ID 0, 8-byte length, offset 0 */
    MENU_INDIRECT_WRITE_RID0_LENGTH8,                 /**< Indirect write register ID 0, 8-byte length */
    MENU_HDSL_REG_INTO_MEMORY,                        /**< Copy HDSL registers into memory */
    MENU_LIMIT,                                       /**< Sentinel value for menu limit */
    MENU_INVALID,                                     /**< Invalid menu option */
} HDSL_MenuOption;

/* Forward declarations */
typedef struct HDSL_Object_s         *HDSL_Handle;

/* ========================================================================== */
/*                  Structure Declarations                                    */
/* ========================================================================== */

/**
 * \name    HDSL Master Register Interface Parameters Structure
 * @{
 */
typedef struct HDSL_Interface_s {
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
 * \name    HDSL Copy Table for overlay scheme used when channel 2 is enabled.
 * @{
 */

typedef struct HDSL_CopyTable_s {
    uint32_t reserved1;
    uint32_t load_addr1; /**< Load Address of Part 1 of firmware */
    uint32_t run_addr1;  /**< Run Address of Part 1 of firmware */
    uint32_t size1;      /**< Size of Part 1 of firmware */
    uint32_t reserved2;
    uint32_t load_addr2; /**< Load Address of Part 2 of firmware */
    uint32_t run_addr2;  /**< Run Address of Part 2 of firmware */
    uint32_t size2;      /**< Size of Part 2 of firmware */
} HDSL_CopyTable;
/** @} */

/**
 * \name    HDSL Initialization Parameters Structure
 * @{
 */
/**
 *    \brief    Structure defining HDSL initialization parameters
 *
 *    \details  Parameters passed to \ref HDSL_open to initialize an HDSL instance.
 *              Use \ref HDSL_params_init to populate with default values.
 *
 */
typedef struct HDSL_Params_s
{
    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS Handle obtained from PRUICSS_open().
     *   Must not be NULL. */

    uint32_t channel;
    /**< HDSL channel number (0, 1, or 2).
     *   In load share mode: Specifies which channel to initialize (0=CH0, 1=CH1, 2=CH2).
     *   In non-load share mode: This field is ignored. Only one channel can be enabled,
     *   and the driver automatically uses handle index 0 regardless of this value. */

} HDSL_Params;
/** @} */

/**
 * \name    HDSL Private Data Structure (Runtime State)
 * @{
 */
/**
 *    \brief    HDSL private data structure (runtime state and configuration)
 *
 *    \details  Contains runtime state information including encoder parameters,
 *              position data, and pointers to PRU-ICSS shared memory.
 *              This structure is initialized during \ref HDSL_open and should
 *              be accessed via \ref HDSL_get_priv API.
 *
 * \note       Architecture: SysConfig generates `HDSL_Priv gHdslPriv[instances][channels]`
 *             - **Per-channel runtime state** - each channel has its own priv
 *             - 2D array: gHdslPriv[instance][channel]
 *             - Contains channel-specific runtime data (position, resolution, etc.)
 *             - Different from attrs which is shared across all channels of an instance
 *
 */
typedef struct HDSL_Priv_s
{
    uint8_t is_open;
    /**< Initialization state flag.
     *   0 = Driver closed/not initialized
     *   1 = Driver successfully initialized and open */

    uint32_t *base_mem_addr;
    /**< Base Memory Address for HDSL channel configuration (internal use only).
     *   Points to PRU data RAM base address + offset for the channel.
     *   Applications must not dereference this pointer directly.
     *   Used internally by driver for PRU communication. */

    HDSL_Interface *hdsl_interface;
    /**< Pointer to HDSL master memory interface structure.
     *   This is the primary communication structure between ARM R5F and PRU firmware */

    uint32_t multi_turn;
    /**< Multi-turn resolution configuration */

    uint32_t res;
    /**< Single-turn resolution configuration */

    uint64_t mask;
    /**< Position data mask for extracting valid position bits */

    PRUICSS_Handle pruicss_handle;
    /**< PRU-ICSS driver handle obtained from PRUICSS_open().
     *   Used for accessing PRU-ICSS hardware resources.
     *   Copied from params in \ref HDSL_open. */

} HDSL_Priv;
/** @} */

/**
 * \name    HDSL Attributes Structure (Compile-time/SysConfig Configuration)
 * @{
 */
/**
 * \brief   Structure defining HDSL attributes (compile-time/SysConfig configuration data)
 *
 * \details This structure contains read-only configuration data that is typically
 *          populated by SysConfig or at compile time. It defines the hardware
 *          configuration including PRU instance, channels, and operation mode.
 *
 * \note    HDSL Architecture and Data Structures:
 *
 *          SysConfig generates a 1D array: `HDSL_Attrs gHdslAttrs[instances]`
 *          - **One attrs structure per instance, shared by all channels**
 *          - Contains per-channel enable flags (channel0_enabled, channel1_enabled, channel2_enabled)
 *          - Contains channel_mask indicating which channels are enabled
 *          - All channels of an instance use the same attrs structure
 *
 *          Related structures:
 *          - `gHdslHandle[instance][channel]` - 2D array of per-channel handles
 *          - `gHdslPriv[instance][channel]` - 2D array of per-channel runtime state
 *          - All handles for instance i point to the same `gHdslAttrs[i]`
 *
 *          **Non-Load Share Mode (225 MHz core clock):**
 *          - Supported on AM243x (ICSSG) and AM261x (ICSSM)
 *          - **Single channel operation only** - exactly one channel must be enabled
 *          - gHdslAttrs[i] contains:
 *            * Exactly one of channel0_enabled/channel1_enabled/channel2_enabled set to 1
 *            * The enabled channel uses PRU0/PRU1 for firmware execution
 *          - Application accesses gHdslHandle[instance][0] regardless of which channel is enabled
 *
 *          **Load Share Mode (300 MHz core clock):**
 *          - Supported on AM243x (ICSSG) only; NOT supported on AM261x
 *          - **Multi-channel operation** - up to 3 channels can be enabled simultaneously
 *          - gHdslAttrs[i] contains:
 *            * channel_mask with bits set for enabled channels (e.g., 0x7 for all three)
 *            * Channel distribution: CH0 uses RTU_PRU0/RTU_PRU1, CH1 uses PRU0/PRU1, CH2 uses TX_PRU0/TX_PRU1
 *          - Application accesses gHdslHandle[instance][channel] for each enabled channel
 *
 */
typedef struct HDSL_Attrs_s
{
    uint8_t instance;
    /**< HDSL instance index (0, 1, ...) for multi-instance configurations.
     *   Used to distinguish between multiple HDSL instances in the system */

    uint8_t pruicss_instance;
    /**< PRU-ICSS hardware instance number (0 or 1).
     *   0 = PRU-ICSSG0/PRU-ICSSM0
     *   1 = PRU-ICSSG1/PRU-ICSSM1 */

    uint8_t pruicss_type;
    /**< PRU-ICSS peripheral type.
     *   HDSL_PRU_ICSSG (0) = PRU-ICSSG (AM243x)
     *   HDSL_PRU_ICSSM (1) = PRU-ICSSM (AM261x)
     *   This identifies the underlying PRU-ICSS hardware variant. */

    uint8_t pruicss_slice;
    /**< PRU-ICSS slice selection (0 or 1).
     *   Each PRU-ICSS has 2 slices, each with its own set of PRU cores.
     *   0 = Slice 0 (contains PRU0/RTU-PRU0/TX-PRU0 on PRU-ICSSG, PRU0 on PRU-ICSSM)
     *   1 = Slice 1 (contains PRU1/RTU-PRU1/TX-PRU1 on PRU-ICSSG, PRU1 on PRU-ICSSM) */

    uint8_t mode;
    /**< Operational mode selection.
     *   0 = HDSL_OPERATIONAL_MODE_FREE_RUN (encoder operates continuously without external synchronization)
     *   1 = HDSL_OPERATIONAL_MODE_SYNC (encoder synchronized to external SYNC pulse) */

    uint8_t load_share_enabled;
    /**< Load share mode enable flag.
     *   0 = Disabled @ 225 MHz (single PRU handles all channels, AM243x/AM261x)
     *       Application must always use index 0 for handle access regardless of channel number
     *   1 = Enabled @ 300 MHz (channels distributed across RTU-PRU/PRU/TX-PRU, AM243x PRU-ICSSG only)
     *       Application uses channel number as array index for handle access */

    uint8_t channel_mask;
    /**< Bit mask indicating which channels are enabled (0-7).
     *   Bit 0 (0x1): Channel 0 enabled
     *   Bit 1 (0x2): Channel 1 enabled
     *   Bit 2 (0x4): Channel 2 enabled
     *   Example: 0x5 = channels 0 and 2 enabled
     *   This attrs structure is shared by all channels of this instance. */

    uint8_t channel0_enabled;
    /**< Channel 0 enable flag (1=enabled, 0=disabled) */

    uint8_t channel1_enabled;
    /**< Channel 1 enable flag (1=enabled, 0=disabled) */

    uint8_t channel2_enabled;
    /**< Channel 2 enable flag (1=enabled, 0=disabled) */

    uint8_t total_channels;
    /**< Total number of enabled channels for this instance (1, 2, or 3).
     *   Calculated as: channel0_enabled + channel1_enabled + channel2_enabled
     *   This attrs structure is shared by all channels of this instance. */

    uint32_t core_clk_freq;
    /**< PRU-ICSS core clock frequency in Hz.
     *   Typically 225 MHz or 300 MHz depending on SoC and configuration */

    uint32_t iep_clk_freq;
    /**< PRU-ICSS IEP (Industrial Ethernet Peripheral) clock frequency in Hz */

} HDSL_Attrs;
/** @} */

/**
 * \name    HDSL Configuration Handle Structure
 * @{
 */
/**
 * \brief   Structure defining HDSL configuration handle
 *
 * \details This structure combines pointers to both runtime state (priv) and compile-time
 *          configuration (attrs). The handle is returned by \ref HDSL_open() and passed to all
 *          HDSL driver APIs to identify the specific HDSL instance and channel being operated on.
 *
 * \note    Architecture: SysConfig generates `HDSL_Object gHdslHandle[instances][channels]`
 *          - **Per-channel handle** - each channel has its own handle
 *          - 2D array: gHdslHandle[instance][channel]
 *          - Each gHdslHandle[i][j] contains:
 *            * priv: Points to gHdslPriv[i][j] (per-channel runtime state)
 *            * attrs: Points to gHdslAttrs[i] (shared configuration for all channels of instance i)
 *          - If load share mode is disabled, application should use gHdslHandle[instance][0] always
 *          - If load share mode is enabled, application should use gHdslHandle[instance][channel]
 *            to access specific channels
 *
 */
typedef struct HDSL_Object_s
{
    HDSL_Priv *priv;
    /**< Pointer to HDSL private data (runtime state and results).
     *   Points to per-channel priv gHdslPriv[instance][channel], if load share mode is enabled.
     *   Points to priv gHdslPriv[instance][0], if load share mode is disabled.
     *   Contains encoder parameters, position data, and all runtime
     *   operational state maintained by the driver for this specific channel */

    const HDSL_Attrs *attrs;
    /**< Pointer to HDSL attributes (read-only configuration from SysConfig).
     *   Points to shared attrs: gHdslAttrs[instance].
     *   Contains compile-time configuration including PRU instance, channels,
     *   and operation mode settings shared by all channels of this instance */

} HDSL_Object;
/** @} */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief      Initialize HDSL parameters to default values
 *
 *  \details    Populates the \ref HDSL_Params structure with default initialization values.
 *              This function should be called before \ref HDSL_open to ensure all
 *              parameters are properly initialized.
 *
 *  \param[out] params    Pointer to \ref HDSL_Params structure to initialize
 *
 */
void HDSL_params_init(HDSL_Params *params);

/**
 *  \brief      Initialize an HDSL channel with SysConfig integration
 *
 *  \details    This function initializes an HDSL channel by setting up the firmware
 *              interface and configuring hardware based on SysConfig parameters.
 *
 *              **Handle Structure:**
 *              - Returns gHdslHandle[instance][channel_index], if load share mode is enabled.
 *              - Returns gHdslHandle[instance][0], if load share mode is disabled.
 *              - Each handle contains:
 *                * priv: Points to per-channel runtime state gHdslPriv[instance][channel_index],
 *                  if load share mode is enabled and gHdslPriv[instance][0], if load share mode
 *                  is disabled
 *                * attrs: Points to shared configuration gHdslAttrs[instance]
 *
 *              **Load Share Mode (300 MHz):**
 *              - Multiple channels can be enabled (up to 3 per instance)
 *              - params->channel specifies which channel to initialize (0, 1, or 2)
 *              - Each enabled channel must be initialized separately with its own HDSL_open call
 *              - Returns handle from gHdslHandle[instance][params->channel]
 *              - Channels distributed: CH0->RTU_PRU, CH1->PRU, CH2->TX_PRU
 *
 *              **Non-Load Share Mode (225 MHz):**
 *              - Only ONE channel can be enabled per instance
 *              - params->channel is validated against enabled channel but driver returns gHdslHandle[instance][0]
 *              - All physical channels (0, 1, 2) use the same PRU core
 *              - Application always uses handle with index 0 (gHdslHandle[instance][0])
 *
 *              **Internal Operations:**
 *              - Validates all input parameters (instance, channel from params, attrs, priv)
 *              - Validates non-load share mode has exactly 1 channel enabled
 *              - Validates the specified channel is enabled in attrs->channel_mask
 *              - Validates pruicss_handle from params
 *              - Validates all attrs fields (instance, channels, PRU settings)
 *              - Validates hardware-specific constraints:
 *                * Core clock: Must be either 225 MHz or 300 MHz (no other frequencies supported)
 *                * ICSSM: Must use 225 MHz and load share must be disabled
 *                * 225 MHz: Load share must be disabled and only 1 channel allowed
 *                * 300 MHz: Load share must be enabled
 *                * Load share mode with channel 2: Channel 0 must also be enabled due
 *                  to TX_PRU instruction memory limitation.
 *              - Configures PRU data RAM base address and channel-specific offsets
 *              - Initializes priv->hdsl_interface pointer to PRU shared memory
 *              - Configures channel mask in PRU firmware
 *              - Copies pruicss_handle to priv for runtime use
 *              - Sets priv->is_open flag to 1 on success
 *
 *  \param[in]  instance     HDSL instance index (0, 1, ...)
 *  \param[in]  params       Pointer to \ref HDSL_Params structure containing:
 *                           - pruicss_handle: PRU-ICSS handle from PRUICSS_open()
 *                           - channel: Channel number (0, 1, 2).
 *                             In load share mode: specifies which channel to initialize
 *                             In non-load share mode: validated but return is always gHdslHandle[instance][0]
 *
 *  \retval     handle       Pointer to initialized HDSL_Handle (HDSL_Object*)
 *                           - Load share: gHdslHandle[instance][params->channel]
 *                           - Non-load share: gHdslHandle[instance][0]
 *  \retval     NULL         On validation failure (invalid instance/channel, disabled channel,
 *                           NULL params, NULL priv/attrs, invalid pruicss_handle,
 *                           out of range configuration values, non-load share with > 1 channel enabled)
 *
 */
HDSL_Handle HDSL_open(uint32_t instance, const HDSL_Params *params);

/**
 *  \brief      Close an HDSL instance
 *
 *  \details    Cleans up an HDSL instance by clearing the is_open flag. After calling this function,
 *              the handle should not be used until reinitialized with \ref HDSL_open.
 *
 *  \param[in]  handle  HDSL handle obtained from \ref HDSL_open
 *
 */
void HDSL_close(HDSL_Handle handle);

/**
 *  \brief      Get pointer to read-only HDSL attributes structure
 *
 *  \details    Returns a const pointer to the attributes (compile-time configuration)
 *              associated with the specified HDSL handle. The returned pointer points
 *              to immutable data and should not be modified by the application.
 *
 *  \param[in]  handle  HDSL handle obtained from \ref HDSL_open
 *
 *  \retval     attrs        Pointer to const \ref HDSL_Attrs structure (read-only, do not modify)
 *  \retval     NULL         If handle is NULL
 *
 *  \note       The returned pointer is const-qualified to prevent accidental modification
 *              of the attributes structure which should remain unchanged during runtime.
 */
const HDSL_Attrs* HDSL_get_attrs(HDSL_Handle handle);

/**
 *  \brief      Get pointer to HDSL private data structure
 *
 *  \details    Returns a pointer to the private data (runtime state) associated
 *              with the specified HDSL handle. This can be used to check the is_open
 *              flag or access other runtime state information.
 *
 *  \param[in]  handle  HDSL handle obtained from \ref HDSL_open
 *
 *  \retval     priv         Pointer to \ref HDSL_Priv structure
 *  \retval     NULL         If handle is NULL
 *
 */
HDSL_Priv* HDSL_get_priv(HDSL_Handle handle);

/**
 *  \brief      Initialize HDSL hardware configuration and clock settings for a PRU slice
 *
 *  \details    Performs low-level hardware initialization of the PRU-ICSS peripheral for
 *              HDSL encoder communication. This function must be called after \ref HDSL_open
 *              and before loading/running the PRU firmware. The configuration is applied to
 *              the PRU slice (PRU0 or PRU1) specified by attrs->pruicss_slice.
 *
 *              This function configures:
 *              - GP MUX selection for EnDAT mode (hardware interface used for HDSL)
 *              - RX/TX clock dividers calculated from core clock frequency
 *              - Clock source selection (always uses core clock, not UART clock)
 *              - RX oversampling at 8x rate with start bit polarity = 1
 *              - Load share mode enable (when attrs->load_share_enabled = 1)
 *              - Channel-specific CFG0 registers cleared for all enabled channels
 *
 *              Clock configuration calculates dividers to achieve target communication frequency:
 *              - RX clock rate: 75 MHz (9.375 MHz  * 8x oversampling)
 *              - TX clock rate: 9.375 MHz (HDSL interface frequency)
 *              - Core clock: 225 MHz (non-load share) or 300 MHz (load share)
 *
 *              **Load Share Mode Operation:**
 *              Load share mode distributes channels across multiple PRU cores (RTU_PRU, PRU, TX_PRU)
 *              for multi-channel operation. Required for reliable operation at 300 MHz core clock.
 *              When enabled, hardware resources are shared across all channels on the PRU slice.
 *
 *              **Important**: In load share mode, call this function only once per PRU slice,
 *              not per channel. The hardware configuration registers are shared. Use any valid
 *              handle from the slice to configure the shared resources.
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *                       In non-load share mode: always use handle at index 0 (gAppHdslHandle[instance][0])
 *                       In load share mode: use any valid handle from the enabled channels
 *
 *  \return     SystemP_SUCCESS on successful hardware initialization
 *  \return     SystemP_FAILURE if handle is invalid or GP MUX configuration fails
 *
 *  \note       Must be called after HDSL_open and before loading PRU firmware
 *  \note       Configures the hardware registers based on the
 *              PRU slice specified in attrs (PRU0 or PRU1 based on attrs->pruicss_slice)
 *  \note       In load share mode: call once per slice, clears CFG0 for all enabled channels
 *  \note       In non-load share mode: call once using handle with index 0 (only one channel enabled per slice)
 */
int32_t HDSL_hw_init(HDSL_Handle handle);

/**
 *  \brief      Calculate fast position, safe position 1, or safe position 2
 *
 *  \param[in]  handle          HDSL handle obtained from \ref HDSL_open
 *  \param[in]  position_id     Position selector: 0=Fast position, 1=Safe position 1, 2=Safe position 2
 *  \param[out] position        Pointer to store 40-bit position value (must not be NULL)
 *
 *  \retval     SystemP_SUCCESS  Position read successfully
 *  \retval     SystemP_FAILURE  Invalid handle, NULL position pointer, or invalid position_id
 *
 */
int32_t HDSL_get_pos(HDSL_Handle handle, uint32_t position_id, uint64_t *position);

/**
 *  \brief      Get quality monitoring value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] qm       Pointer to store 8-bit QM value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or qm pointer is NULL
 */
int32_t HDSL_get_qm(HDSL_Handle handle, uint8_t *qm);

/**
 *  \brief      Get values of High bytes event (EVENT_H) and Low bytes event (EVENT_L)
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] events   Pointer to store 16-bit concatenated values of EVENT_H and EVENT_L
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or events pointer is NULL
 */
int32_t HDSL_get_events(HDSL_Handle handle, uint16_t *events);

/**
 *  \brief      Get values of Safe Event (EVENT_S) register
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] events   Pointer to store 8-bit EVENT_S value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or events pointer is NULL
 */
int32_t HDSL_get_safe_events(HDSL_Handle handle, uint8_t *events);


/**
 *  \brief      Get values of Online Status D (ONLINE_STATUS_D) register
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] status   Pointer to store 16-bit ONLINE_STATUS_D value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or status pointer is NULL
 */
int32_t HDSL_get_online_status_d(HDSL_Handle handle, uint16_t *status);

/**
 *  \brief      Get values of Online Status 1 (ONLINE_STATUS_1) register
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] status   Pointer to store 16-bit ONLINE_STATUS_1 value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or status pointer is NULL
 */
int32_t HDSL_get_online_status_1(HDSL_Handle handle, uint16_t *status);

/**
 *  \brief      Get values of Online Status 2 (ONLINE_STATUS_2) register
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] status   Pointer to store 16-bit ONLINE_STATUS_2 value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or status pointer is NULL
 */
int32_t HDSL_get_online_status_2(HDSL_Handle handle, uint16_t *status);

/**
 *  \brief      Get summarized slave status
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] sum      Pointer to store 8-bit summarized status value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or sum pointer is NULL
 */
int32_t HDSL_get_sum(HDSL_Handle handle, uint8_t *sum);

/**
 *  \brief      Get acceleration error counter
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] count    Pointer to store 8-bit acceleration error counter value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or count pointer is NULL
 */
int32_t HDSL_get_acc_err_cnt(HDSL_Handle handle, uint8_t *count);

/**
 *  \brief      Read RSSI value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] rssi     Pointer to store 8-bit RSSI value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or rssi pointer is NULL
 */
int32_t HDSL_get_rssi(HDSL_Handle handle, uint8_t *rssi);

/**
 *  \brief  Trigger a short message write operation using parameters channel
 *          After the required registers are written for write operation, the firmware takes < 250 us
 *          for completing short message write operation, i.e. FRES bit will be unset for < 250 us.
 *
 *  \param[in]  handle  HDSL handle obtained from \ref HDSL_open
 *  \param[in]  addr    Address (must be 6-bit: 0x00 to 0x3F)
 *  \param[in]  data    Data
 *  \param[in]  timeout Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success
 *  \return     SystemP_FAILURE if handle is NULL, addr > 0x3F, or internal structures are NULL
 *  \return     SystemP_TIMEOUT in case of timeout
 *
 *  \note       Applications should use reasonable timeout values. Extremely large values
 *              (close to UINT64_MAX) are not recommended as they may cause immediate timeout
 *              due to potential arithmetic overflow in the timeout calculation.
 *
 */
int32_t HDSL_write_pc_short_msg(HDSL_Handle handle, uint8_t addr, uint8_t data, uint64_t timeout);

/**
 *  \brief      Trigger a short message read operation using parameters channel.
 *              After the required registers are written for read operation, the firmware takes < 250 us
 *              for completing short message read operation, i.e. FRES bit will be unset for < 250 us.
 *
 *  \param[in]  handle  HDSL handle obtained from \ref HDSL_open
 *  \param[in]  addr    Address (must be 6-bit: 0x00 to 0x3F)
 *  \param[in]  data    Pointer to data buffer where read data will be stored
 *  \param[in]  timeout Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS in case of success
 *  \return     SystemP_FAILURE if handle is NULL, addr > 0x3F, data is NULL, or internal structures are NULL
 *  \return     SystemP_TIMEOUT in case of timeout
 *
 *  \note       Applications should use reasonable timeout values. Extremely large values
 *              (close to UINT64_MAX) are not recommended as they may cause immediate timeout
 *              due to potential arithmetic overflow in the timeout calculation.
 *
 */
int32_t HDSL_read_pc_short_msg(HDSL_Handle handle, uint8_t addr, uint8_t *data, uint64_t timeout);

/**
 *  \brief  Trigger a long message write operation using parameters channel.
 *
 *  \details    **Required Workflow:**
 *              1. Call \ref HDSL_write_pc_buffer to write data bytes (0-7) to PC_BUFFER registers
 *              2. Call this function to trigger the long message write operation
 *              3. Firmware completes the write within < 3.5 ms (FREL bit unset during operation)
 *              4. If SystemP_SUCCESS is returned, optionally call \ref HDSL_get_pc_long_msg_error
 *                 to check if encoder accepted the parameters or reported an error
 *
 *              This function performs internal operations:
 *              - Configures PC_ADD_L, PC_ADD_H registers with address and control bits
 *              - Configures PC_OFF_L, PC_OFF_H registers with offset value
 *              - Sets PC_CTRL register to trigger the operation
 *              - Waits for ONLINE_STATUS_D_L FREL bit transitions
 *
 *                Note: SystemP_SUCCESS return value does not indicate that the encoder accepted the parameters.
 *                Use \ref HDSL_get_pc_long_msg_error to check if encoder accepted the parameters or reported an error.
 *
 *  \param[in]  handle          HDSL handle obtained from \ref HDSL_open
 *  \param[in]  addr            10 bit address for long message (0-0x3FF)
 *  \param[in]  offset_enable   Addressing with offset enable/disable from \ref HDSL_LongMessageAddrOffsetModes (0 or 1)
 *  \param[in]  addr_type       Addressing Type from \ref HDSL_LongMessageAddrTypes (0 or 1)
 *  \param[in]  length          Length from \ref HDSL_LongMessageLengths (0-3)
 *  \param[in]  offset          15 bit address offset for long message (0-0x7FFF, if offset is enabled in offset_enable parameter)
 *  \param[in]  timeout         Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS if communication completed (check \ref HDSL_get_pc_long_msg_error to check if encoder accepted the parameters or reported an error)
 *  \return     SystemP_FAILURE if input parameters are invalid
 *  \return     SystemP_TIMEOUT if FREL transitions did not complete within timeout
 *
 *  \note       Applications should use reasonable timeout values. Extremely large values
 *              (close to UINT64_MAX) are not recommended as they may cause immediate timeout
 *              due to potential arithmetic overflow in the timeout calculation.
 *
 */
int32_t HDSL_write_pc_long_msg(HDSL_Handle handle, uint16_t addr, uint8_t offset_enable, uint8_t addr_type, uint8_t length, uint16_t offset, uint64_t timeout);

/**
 *  \brief      Trigger a long message read operation using parameters channel
 *
 *  \details    **Required Workflow:**
 *              1. Call this function to trigger the long message read operation
 *              2. Wait for SystemP_SUCCESS return value
 *              3. If SystemP_SUCCESS is returned, optionally call \ref HDSL_get_pc_long_msg_error
 *                 to check if encoder accepted the parameters or reported an error
 *              4. Call \ref HDSL_read_pc_buffer to read data bytes (0-7) from PC_BUFFER registers
 *
 *              \note Firmware completes the read within < 3.5 ms (FREL bit unset during operation).
 *
 *              This function performs internal operations:
 *              - Configures PC_ADD_L, PC_ADD_H registers with address and control bits
 *              - Configures PC_OFF_L, PC_OFF_H registers with offset value
 *              - Sets PC_CTRL register to trigger the operation
 *              - Waits for ONLINE_STATUS_D_L FREL bit transitions
 *
 *                Note: SystemP_SUCCESS return value does not indicate that the encoder accepted the parameters.
 *                Use \ref HDSL_get_pc_long_msg_error to check if encoder accepted the parameters or reported an error.
 *
 *  \param[in]  handle          HDSL handle obtained from \ref HDSL_open
 *  \param[in]  addr            10 bit address for long message (0-0x3FF)
 *  \param[in]  offset_enable   Addressing with offset enable/disable from \ref HDSL_LongMessageAddrOffsetModes (0 or 1)
 *  \param[in]  addr_type       Addressing Type from \ref HDSL_LongMessageAddrTypes (0 or 1)
 *  \param[in]  length          Length from \ref HDSL_LongMessageLengths (0-3)
 *  \param[in]  offset          15 bit address offset for long message (0-0x7FFF)
 *  \param[in]  timeout         Timeout in microseconds
 *
 *  \return     SystemP_SUCCESS if communication completed (check \ref HDSL_get_pc_long_msg_error to check if encoder accepted the parameters or reported an error)
 *  \return     SystemP_FAILURE if input parameters are invalid
 *  \return     SystemP_TIMEOUT if FREL transitions did not complete within timeout
 *
 *  \note       Applications should use reasonable timeout values. Extremely large values
 *              (close to UINT64_MAX) are not recommended as they may cause immediate timeout
 *              due to potential arithmetic overflow in the timeout calculation.
 *
 */
int32_t HDSL_read_pc_long_msg(HDSL_Handle handle, uint16_t addr, uint8_t offset_enable, uint8_t addr_type, uint8_t length, uint16_t offset, uint64_t timeout);

/**
 *  \brief      Write Parameters channel buffer for different bytes(bytes 0-7)
 *
 *  \details    Writes data to PC_BUFFER registers (PC_BUFFER0 through PC_BUFFER7) in HDSL interface.
 *              This function must be called before \ref HDSL_write_pc_long_msg to prepare the data
 *              payload for transmission.
 *
 *              **Typical usage workflow:**
 *              1. Call HDSL_write_pc_buffer() for each data byte (buff_off 0-7) to write
 *              2. Call \ref HDSL_write_pc_long_msg to trigger the transmission
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  buff_off     Buffer offset (0-7)
 *  \param[in]  data         Data byte to write
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle or buff_off > 7)
 *
 *  \note       This function performs NULL check on handle and bounds check on buff_off.
 *              Returns SystemP_FAILURE if buff_off is out of range (0-7).
 *              Internal structures (priv, hdsl_interface) are guaranteed valid after
 *              successful \ref HDSL_open and not rechecked for performance.
 *
 */
int32_t HDSL_write_pc_buffer(HDSL_Handle handle, uint8_t buff_off, uint8_t data);

/**
 *  \brief      Returns Parameters channel buffer for different bytes (bytes 0-7)
 *
 *  \details    Reads data from PC_BUFFER registers (PC_BUFFER0 through PC_BUFFER7) in HDSL interface.
 *              This function must be called after \ref HDSL_read_pc_long_msg returns SystemP_SUCCESS
 *              to retrieve the received data payload.
 *
 *              **Typical usage workflow:**
 *              1. Call \ref HDSL_read_pc_long_msg to trigger the read operation
 *              2. Validate that SystemP_SUCCESS is returned
 *              3. Call HDSL_read_pc_buffer() for each data byte (buff_off 0-7) to read
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  buff_off Buffer offset (0-7)
 *  \param[out] data     Pointer to store the 8-bit PC_BUFFER value (must not be NULL)
 *
 *  \retval     SystemP_SUCCESS  Data read successfully
 *  \retval     SystemP_FAILURE  Invalid handle, NULL data pointer, or invalid buff_off > 7
 *
 *  \note       Internal structures (priv, hdsl_interface) are guaranteed valid after
 *              successful \ref HDSL_open and not rechecked for performance.
 */
int32_t HDSL_read_pc_buffer(HDSL_Handle handle, uint8_t buff_off, uint8_t *data);

/**
 *  \brief      Check encoder error status from long message parameter channel operation
 *
 *  \details    Reads the error status bit (bit 5) from PC_ADD_H register to determine if
 *              the encoder reported an error for the last long message operation.
 *
 *              This function should be called after \ref HDSL_write_pc_long_msg or
 *              \ref HDSL_read_pc_long_msg returns SystemP_SUCCESS to check if the encoder
 *              accepted the parameters or responded with an error.
 *
 *              **Typical Usage:**
 *              1. Call \ref HDSL_write_pc_long_msg or \ref HDSL_read_pc_long_msg
 *              2. If SystemP_SUCCESS is returned (communication succeeded)
 *              3. Call this function to check if encoder reported parameter error
 *              4. Handle error condition if error status is 1
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] error    Pointer to store encoder error status (0 = no error, 1 = error)
 *
 *  \retval     SystemP_SUCCESS  Error status read successfully
 *  \retval     SystemP_FAILURE  Invalid handle (NULL) or error pointer (NULL)
 *
 */
int32_t HDSL_get_pc_long_msg_error(HDSL_Handle handle, uint8_t *error);

/**
 *  \brief      Get synchronization control value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] ctrl     Pointer to store 8-bit SYNC_CTRL value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or ctrl pointer is NULL
 */
int32_t HDSL_get_sync_ctrl(HDSL_Handle handle, uint8_t *ctrl);

/**
 *  \brief      Writes Synchronization control value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  val      Synchronization control value to write
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 */
int32_t HDSL_set_sync_ctrl(HDSL_Handle handle, uint8_t val);

/**
 *  \brief      Get quality monitoring value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] qm       Pointer to store 8-bit MASTER_QM value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or qm pointer is NULL
 */
int32_t HDSL_get_master_qm(HDSL_Handle handle, uint8_t *qm);

/**
 *  \brief      Get cable bit sampling time control
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] edges    Pointer to store 8-bit EDGES value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or edges pointer is NULL
 */
int32_t HDSL_get_edges(HDSL_Handle handle, uint8_t *edges);

/**
 *  \brief      Get run time delay of system cable and signal strength
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] delay    Pointer to store 8-bit DELAY value
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or delay pointer is NULL
 */
int32_t HDSL_get_delay(HDSL_Handle handle, uint8_t *delay);

/**
 *  \brief      Set Parameters channel address registers
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  pc_addrh     High byte of PC address
 *  \param[in]  pc_addrl     Low byte of PC address
 *  \param[in]  pc_offh      High byte of PC offset
 *  \param[in]  pc_offl      Low byte of PC offset
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 *
 */
int32_t HDSL_set_pc_addr(HDSL_Handle handle, uint8_t pc_addrh, uint8_t pc_addrl, uint8_t pc_offh, uint8_t pc_offl);

/**
 *  \brief      Set Parameters channel control register
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  value        Control value to write
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 *
 */
int32_t HDSL_set_pc_ctrl(HDSL_Handle handle, uint8_t value);

/**
 *  \brief      Read encoder ID bytes from HDSL encoder
 *
 *  \details    Retrieves one of the three 8-bit encoder ID bytes (ENC_ID0, ENC_ID1, ENC_ID2)
 *              provided by the HDSL encoder. The encoder ID is a 24-bit value that uniquely
 *              identifies the encoder model or configuration. This function reads a single
 *              byte specified by the byte parameter.
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  byte     Encoder ID byte index: Valid range is 0-2
 *                       - 0: ENC_ID0 (bits 0-7 of encoder ID)
 *                       - 1: ENC_ID1 (bits 8-15 of encoder ID)
 *                       - 2: ENC_ID2 (bits 16-23 of encoder ID)
 *  \param[out] enc_id   Pointer to store 8-bit encoder ID byte data
 *
 *  \retval     SystemP_SUCCESS  Encoder ID byte read successfully
 *  \retval     SystemP_FAILURE  Invalid handle (NULL), byte index out of range (>2), or enc_id pointer (NULL)
 */
int32_t HDSL_get_enc_id(HDSL_Handle handle, uint32_t byte, uint8_t *enc_id);

/**
 *  \brief      Generate lookup tables (LUTs) in PRU-ICSS memory for HDSL firmware operation
 *
 *  \details    This function initializes all required lookup tables in PRU-ICSS data memory by
 *              internally calling specialized generation routines:
 *              - 5b6b and 3b4b encoding/decoding tables (via hdsl_enc_dec_lut)
 *              - Bit count tables (via hdsl_generate_bit_cnt_lut)
 *              - CRC5 and CRC16 calculation tables (via hdsl_generate_crc5_lut and hdsl_generate_crc16_lut)
 *              - Bit-to-byte conversion tables (via hdsl_generate_bit_to_byte_lut)
 *              - RSSI (Received Signal Strength Indication) tables (via hdsl_generate_rssi_lut)
 *              - Extra edge detection tables (via hdsl_generate_extra_edge_lut)
 *              - Register interface configuration (via hdsl_configure_register_if)
 *
 *              This function must be called once after \ref HDSL_open and before starting
 *              position encoder communication. The LUTs are generated in the PRU firmware's
 *              data memory region and are required for proper HDSL protocol operation.
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *
 *  \retval     SystemP_SUCCESS  LUT generation completed successfully
 *  \retval     SystemP_FAILURE  Invalid handle (NULL)
 */
int32_t HDSL_generate_memory_image(HDSL_Handle handle);

/**
 *  \brief      Get base memory address of HDSL interface structure in PRU-ICSS memory
 *
 *  \details    Returns a pointer to the HDSL interface structure (HDSL_Interface) located in
 *              PRU-ICSS data memory. This structure contains all real-time communication
 *              registers and buffers shared between the driver and PRU firmware.
 *
 *  \param[in]  handle    HDSL handle obtained from \ref HDSL_open
 *  \param[out] src_loc   Pointer to store address of HDSL_Interface structure in PRU memory
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or src_loc pointer is NULL
 *
 *  \warning    Direct memory access through this pointer bypasses API safety checks.
 *              Use with caution. Prefer using standard HDSL_get_* and HDSL_set_* APIs.
 */
int32_t HDSL_get_src_loc(HDSL_Handle handle, void **src_loc);

/**
 *  \brief      Get size of HDSL interface structure in bytes
 *
 *  \details    Returns sizeof(HDSL_Interface), which is the size of HDSL firmware
 *              interface structure in PRU-ICSS data memory.
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] length   Pointer to store size of HDSL_Interface structure in bytes
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or length pointer is NULL
 */
int32_t HDSL_get_length(HDSL_Handle handle, uint32_t *length);

/**
 *  \brief      Configure the copy table entries for overlayed firmware parts for channel 2
 *
 *  \details    This function configures the PRU firmware memory copy table for channel 2
 *              operation in multi-channel load share mode. The copy table defines two
 *              overlayed firmware parts that are loaded and executed sequentially.
 *
 *              The copy_table structure specifies:
 *              - part1_load_addr: Load address in PRU memory for firmware part 1
 *              - part1_run_addr: Execution address for firmware part 1
 *              - part1_size: Size in bytes of firmware part 1
 *              - part2_load_addr: Load address in PRU memory for firmware part 2
 *              - part2_run_addr: Execution address for firmware part 2
 *              - part2_size: Size in bytes of firmware part 2
 *
 *              This configuration is required for TX_PRU (channel 2) in multi-channel
 *              load share mode where firmware sections are overlayed to optimize memory usage.
 *
 *  \param[in]  handle       HDSL handle obtained from \ref HDSL_open
 *  \param[in]  copy_table   Pointer to HDSL_CopyTable structure containing firmware part configuration
 *
 *  \retval     SystemP_SUCCESS  Copy table configured successfully
 *  \retval     SystemP_FAILURE  Invalid handle (NULL) or copy_table pointer (NULL)
 */
int32_t HDSL_config_copy_table(HDSL_Handle handle, const HDSL_CopyTable *copy_table);

/**
 *  \brief      Set encoder single-turn resolution value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  res      Single-turn resolution value in bits
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 */
int32_t HDSL_set_res(HDSL_Handle handle, uint32_t res);

/**
 *  \brief      Get encoder single-turn resolution value
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] res      Pointer to store single-turn resolution value in bits
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or res pointer is NULL
 */
int32_t HDSL_get_res(HDSL_Handle handle, uint32_t *res);

/**
 *  \brief      Set encoder multi-turn resolution value
 *
 *  \param[in]  handle       HDSL handle obtained from \ref HDSL_open
 *  \param[in]  multi_turn   Multi-turn resolution value in bits
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 */
int32_t HDSL_set_multi_turn(HDSL_Handle handle, uint32_t multi_turn);

/**
 *  \brief      Get encoder multi-turn resolution value
 *
 *  \param[in]  handle       HDSL handle obtained from \ref HDSL_open
 *  \param[out] multi_turn   Pointer to store multi-turn resolution value in bits
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or multi_turn pointer is NULL
 */
int32_t HDSL_get_multi_turn(HDSL_Handle handle, uint32_t *multi_turn);

/**
 *  \brief      Set encoder position data mask
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[in]  mask     Position data mask for extracting valid position bits
 *
 *  \retval     SystemP_SUCCESS on successful write
 *  \retval     SystemP_FAILURE on invalid parameters (NULL handle)
 */
int32_t HDSL_set_mask(HDSL_Handle handle, uint64_t mask);

/**
 *  \brief      Get encoder position data mask
 *
 *  \param[in]  handle   HDSL handle obtained from \ref HDSL_open
 *  \param[out] mask     Pointer to store position data mask for extracting valid position bits
 *
 *  \return     SystemP_SUCCESS on success
 *  \return     SystemP_FAILURE if handle is invalid or mask pointer is NULL
 */
int32_t HDSL_get_mask(HDSL_Handle handle, uint64_t *mask);

#ifdef __cplusplus
}
#endif

/** @} */
#endif
