/*
 *  Copyright (C) 2022-2025 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <position_sense/hdsl/include/hdsl_drv.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/tistdtypes.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief FRES bit position in ONLINE_STATUS_1_L register */
#define ONLINE_STATUS_1_L_FRES          (1<<0)

/** \brief FREL bit value in ONLINE_STATUS_D_L register */
#define ONLINE_STATUS_D_L_FREL          (1)

/** \brief Enable bit for long message transaction in PC_ADD_H register (bit 7)
 *  \details Must be set to indicate a long message parameter channel operation */
#define PC_ADD_H_LONG_MSG_ENABLE        (1<<7)

/** \brief Write operation bit for long message in PC_ADD_H register (bit 6 = 0)
 *  \details Cleared for write operations, set for read operations */
#define PC_ADD_H_LONG_MSG_WRITE         (0<<6)

/** \brief Read operation bit for long message in PC_ADD_H register (bit 6 = 1)
 *  \details Set for read operations, cleared for write operations */
#define PC_ADD_H_LONG_MSG_READ          (1<<6)

/** \brief Error status bit in PC_ADD_H register (bit 5) */
#define PC_ADD_H_LONG_MSG_ERROR         (1<<5)

/** \brief Bit shift position for offset enable field in PC_ADD_H register */
#define PC_ADD_H_OFFSET_EN_SHIFT        (5)

/** \brief Bit shift position for address type field in PC_ADD_H register */
#define PC_ADD_H_ADDR_TYPE_SHIFT        (4)

/** \brief Bit shift position for length field in PC_ADD_H register */
#define PC_ADD_H_LENGTH_SHIFT           (2)

/** \brief Mask for extracting high 2 bits of 10-bit address from PC_ADD_H register */
#define PC_ADD_H_ADDR_HIGH_MASK         (0x0300)

/** \brief Mask for extracting low 8 bits of 10-bit address from PC_ADD_L register */
#define PC_ADD_L_ADDR_LOW_MASK          (0x00FF)

/** \brief Enable bit for offset in PC_OFF_H register (bit 7)
 *  \details Indicates if offset addressing is enabled for long message */
#define PC_OFF_H_LONG_MSG_ENABLE        (1<<7)

/** \brief Mask for extracting high 7 bits of 15-bit offset from PC_OFF_H register */
#define PC_OFF_H_OFFSET_HIGH_MASK       (0x7F00)

/** \brief Mask for extracting low 8 bits of 15-bit offset from PC_OFF_L register */
#define PC_OFF_L_OFFSET_LOW_MASK        (0x00FF)

/** \brief Enable value for PC_CTRL register to start parameter channel operation */
#define PC_CTRL_ENABLE                  (0x01)

/** \brief Memory offset for firmware part 1 load start address in PRU data memory */
#define PART1_LOAD_START_OFFSET		    (0x76)

/** \brief Memory offset for firmware part 1 run start address in PRU data memory */
#define PART1_RUN_START_OFFSET		    (0x78)

/** \brief Memory offset for firmware part 1 size in PRU data memory */
#define PART1_SIZE_OFFSET				(0x7A)

/** \brief Memory offset for firmware part 2 load start address in PRU data memory */
#define PART2_LOAD_START_OFFSET		    (0x7C)

/** \brief Memory offset for firmware part 2 run start address in PRU data memory */
#define PART2_RUN_START_OFFSET		    (0x7E)

/** \brief Memory offset for firmware part 2 size in PRU data memory */
#define PART2_SIZE_OFFSET				(0x80)

/** \brief Memory offset for channel mask configuration in PRU data memory */
#define CHANNEL_MASK_OFFSET             (0x83)

/** \brief PRU data memory base offset for RTU_PRU (channel 0) in load share mode */
#define HDSL_DMEM_OFFSET_RTU_PRU       (0x0000U)

/** \brief PRU data memory base offset for PRU (channel 1) in load share mode */
#define HDSL_DMEM_OFFSET_PRU           (0x0700U)

/** \brief PRU data memory base offset for TX_PRU (channel 2) in load share mode */
#define HDSL_DMEM_OFFSET_TX_PRU        (0x0E00U)

/** \brief Mask for extracting Quality Monitor (QM) value (lower 4 bits) */
#define HDSL_QM_VALUE_MASK              (0xFU)

/** \brief Mask for extracting Acceleration Error Counter value (lower 5 bits) */
#define HDSL_ACC_ERR_CNT_MASK           (0x1FU)

/** \brief Mask for extracting RSSI (Received Signal Strength Indication) value (upper 4 bits) */
#define HDSL_RSSI_MASK                  (0xF0U)

/** \brief Bit shift for extracting RSSI value from combined byte */
#define HDSL_RSSI_SHIFT                 (4U)

/** \brief Read bit flag in short message address byte (bit 7)
 *  \details Set for read operations, cleared for write operations */
#define HDSL_SHORT_MSG_READ_BIT         (1U<<7)

/** \brief Maximum position ID value for HDSL_get_pos (0=Fast, 1=Safe1, 2=Safe2) */
#define HDSL_MAX_POSITION_ID            (2U)

/** \brief Maximum parameter channel buffer index (PC_BUFFER0 through PC_BUFFER7) */
#define HDSL_PC_BUFFER_MAX_INDEX        (7U)

/** \brief Number of encoder ID bytes available (ENC_ID0, ENC_ID1, ENC_ID2) */
#define HDSL_NUM_ENC_ID_BYTES           (3U)

/** \brief MHz to Hz conversion factor (value = 1000000)
 *
 *  Multiplication factor for converting frequency values from MHz to Hz.
 *  Used in clock configuration calculations.
 */
#define HDSL_MHZ_TO_HZ                  (1000000U)

/** \brief Oversample rate for HDSL receiver (8x oversampling) */
#define HDSL_OVERSAMPLE_RATE_8X         (8U)

/** \brief Default RX clock rate for HDSL (75 MHz = 9.375 MHz * 8)
 *  \details Calculated as data rate (9.375 MHz) multiplied by oversample rate (8x) */
#define HDSL_DEFAULT_RX_CLOCK_RATE      (75 * HDSL_MHZ_TO_HZ)

/** \brief Start bit polarity configuration value
 *  \details Start bit polarity is set to 1 for HDSL protocol */
#define HDSL_SB_POLARITY                (1U)

/* ========================================================================== */
/*                       Internal Functions Note                              */
/* ========================================================================== */
/**
 *  \details After successful HDSL_open(), the priv pointer returned by HDSL_get_priv()
 *           is guaranteed to be valid and non-NULL for the lifetime of the handle.
 *           Driver functions rely on this guarantee and do not perform redundant NULL
 *           checks on priv for performance. Applications must not call driver APIs
 *           with handles that failed to open (NULL handles are checked at API entry).
 */

/* ========================================================================== */
/*                       Global Variables                                     */
/* ========================================================================== */
/**
 *  \brief  Global variables for HDSL configuration
 *
 *  \details These are extern declarations. The actual definitions and initialization
 *           are generated by SysConfig in ti_drivers_config.c.
 *
 *           **SysConfig generates:**
 *           - uint32_t gHdslConfigNum: Number of HDSL instances configured
 *           - HDSL_Object gHdslHandle[instances][3]: 2D array of HDSL_Object structures
 *           - HDSL_Priv gHdslPriv[instances][3]: 2D array for per-channel runtime state
 *           - HDSL_Attrs gHdslAttrs[instances]: 1D array for shared compile-time config
 *
 *           **Structure Pattern:**
 *           - Attrs is shared across all channels of an instance (1D array)
 *           - Priv is per-channel (2D array) for individual runtime state
 *           - Handle is per-channel (2D array) for direct access
 *             gHdslHandle[i][j].priv  = &gHdslPriv[i][j]   (per-channel)
 *             gHdslHandle[i][j].attrs = &gHdslAttrs[i]     (shared)
 *
 *           **Non-Load Share Mode (225 MHz - AM243x/AM261x):**
 *           Single channel operation only:
 *             - gHdslAttrs[i]: Shared attrs with one channel enabled (channel0/1/2_enabled)
 *             - gHdslHandle[i][0]: Index 0 is used always
 *             - PRU core determined at runtime based on pruicss_slice (PRU0 or PRU1)
 *           Application initializes only the enabled channel (ch = 0, 1, or 2).
 *
 *           **Load Share Mode (300 MHz - AM243x ICSSG only):**
 *           Multi-channel operation with load distribution:
 *             - gHdslAttrs[i]: Shared attrs with channel enable flags (channel0/1/2_enabled)
 *             - gHdslHandle[i][0/1/2]: Each enabled channel has its own priv
 *             - PRU core determined at runtime:
 *               * Channel 0 -> RTU_PRU (based on slice)
 *               * Channel 1 -> PRU (based on slice)
 *               * Channel 2 -> TX_PRU (based on slice)
 *           Application initializes each enabled channel separately.
 *
 *           **Example: Load share with CH0 and CH2 enabled:**
 *             gHdslAttrs[0]: channel0_enabled=1, channel1_enabled=0, channel2_enabled=1
 *             gHdslHandle[0][0] = valid (uses gHdslPriv[0][0], RTU_PRU determined at runtime)
 *             gHdslHandle[0][1] = available but CH1 not enabled (init will fail)
 *             gHdslHandle[0][2] = valid (uses gHdslPriv[0][2], TX_PRU determined at runtime)
 */
extern uint32_t gHdslConfigNum;

extern HDSL_Object gHdslHandle[][HDSL_NUM_CH_PER_SLICE_MAX];

/**
 *  \brief  Default HDSL initialization parameters
 *
 *  \details This structure contains the default values for HDSL initialization.
 *           Use HDSL_Params_init() to copy these defaults into a HDSL_Params structure.
 */
const HDSL_Params gHdslDefaultParams = {
    NULL,  /* pruicss_handle - Must be set by application */
    0,     /* channel - Default to channel 0 */
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void HDSL_params_init(HDSL_Params *params)
{
    if(params != NULL)
    {
        *params = gHdslDefaultParams;
    }
}

HDSL_Handle HDSL_open(uint32_t instance, const HDSL_Params *params)
{
    int32_t             status = SystemP_SUCCESS;
    HDSL_Handle         handle = NULL;
    HDSL_Priv           *priv = NULL;
    const HDSL_Attrs    *attrs = NULL;
    uint32_t            channel_idx;
    uint32_t            offset = 0;
    /* Validate instance and params */
    if((instance >= gHdslConfigNum) || (params == NULL))
    {
        status = SystemP_FAILURE;
    }

    /* Validate params fields: channel and pruicss_handle */
    if((status == SystemP_SUCCESS) &&
       ((params->channel >= HDSL_NUM_CH_PER_SLICE_MAX) || (params->pruicss_handle == NULL)))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Get attrs to determine load share mode (all channels share same attrs) */
        attrs = gHdslHandle[instance][0].attrs;

        if(attrs == NULL)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Validate non-load share mode configuration */
        if(attrs->load_share_enabled == 0)
        {
            /* Non-load share mode: Only 1 channel can be enabled */
            if(attrs->total_channels != 1)
            {
                status = SystemP_FAILURE;
            }

            /* In non-load share mode, always use index 0 regardless of params->channel.
             * Even if channel 1 or 2 is configured, the handle is stored at index 0. */
            channel_idx = 0;
        }
        else
        {
            /* Load share mode: Use channel from params */
            channel_idx = params->channel;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        handle = (HDSL_Handle)(&gHdslHandle[instance][channel_idx]);
        priv = handle->priv;
        attrs = handle->attrs;

        /* Validate that this channel is enabled (not marked invalid) */
        if((priv == NULL) || (attrs == NULL) || (attrs->instance == 0xFF))
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Validate attrs fields */
        if((attrs->instance >= gHdslConfigNum) ||
           (attrs->pruicss_instance > 1) ||
           (attrs->pruicss_type > HDSL_PRU_ICSSM) ||
           (attrs->pruicss_slice > 1) ||
           (attrs->mode > HDSL_OPERATIONAL_MODE_SYNC) ||
           (attrs->load_share_enabled > 1) ||
           (attrs->channel_mask == 0) ||
           (attrs->channel_mask > 7) ||
           (attrs->channel0_enabled > 1) ||
           (attrs->channel1_enabled > 1) ||
           (attrs->channel2_enabled > 1) ||
           (attrs->total_channels == 0) ||
           (attrs->total_channels > HDSL_NUM_CH_PER_SLICE_MAX) ||
           (attrs->core_clk_freq == 0) ||
           (attrs->iep_clk_freq == 0))
        {
            status = SystemP_FAILURE;
        }
    }

    /* Validate that the requested channel is enabled */
    if(status == SystemP_SUCCESS)
    {
        if((channel_idx == 0 && attrs->channel0_enabled == 0) ||
            (channel_idx == 1 && attrs->channel1_enabled == 0) ||
            (channel_idx == 2 && attrs->channel2_enabled == 0))
        {
            status = SystemP_FAILURE;
        }
    }

    /* Validate ICSSM hardware constraints for HDSL */
    if(status == SystemP_SUCCESS)
    {
        /* HDSL based on ICSSM only supports 225 MHz and load share must be disabled */
        if(attrs->pruicss_type == HDSL_PRU_ICSSM)
        {
            if((attrs->core_clk_freq != 225000000) || (attrs->load_share_enabled != 0))
            {
                status = SystemP_FAILURE;
            }
        }
    }

    /* Validate core clock and load share mode correlation */
    if(status == SystemP_SUCCESS)
    {
        /* At 225 MHz: load share must be disabled and only 1 channel allowed */
        if(attrs->core_clk_freq == 225000000)
        {
            if((attrs->load_share_enabled != 0) || (attrs->total_channels != 1))
            {
                status = SystemP_FAILURE;
            }
        }
        /* At 300 MHz: load share must be enabled */
        else if(attrs->core_clk_freq == 300000000)
        {
            if(attrs->load_share_enabled != 1)
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            /* Only 225 MHz and 300 MHz are supported core clock frequencies */
            status = SystemP_FAILURE;
        }
    }

    /* Validate load share mode channel dependencies */
    if(status == SystemP_SUCCESS)
    {
        /* In load share mode, if channel 2 is enabled, channel 0 must also be enabled
            * due to TX_PRU instruction memory limitation (code overlay scheme) */
        if(attrs->load_share_enabled && attrs->channel2_enabled && (!(attrs->channel0_enabled)))
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Store PRUICSS handle in priv */
        priv->pruicss_handle = params->pruicss_handle;

        /* Configure base memory address based on PRU core */
        PRUICSS_HwAttrs *hw_attrs = (PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs);

        if(attrs->load_share_enabled == 0)
        {
            /* Set base memory address */
            if(attrs->pruicss_slice)
            {
                priv->base_mem_addr = (uint32_t *)(hw_attrs->pru1DramBase);
            }
            else
            {
                priv->base_mem_addr = (uint32_t *)(hw_attrs->pru0DramBase);
            }
        }
        else
        {
            /* Load share mode: Select PRU core based on channel index */
            /* Channel 0 -> RTU_PRU, Channel 1 -> PRU, Channel 2 -> TX_PRU */

            if(channel_idx == 0)
            {
                /* Channel 0 uses RTU_PRU */
                offset = HDSL_DMEM_OFFSET_RTU_PRU;
            }
            else if(channel_idx == 1)
            {
                /* Channel 1 uses PRU */
                offset = HDSL_DMEM_OFFSET_PRU;
            }
            else if(channel_idx == 2)
            {
                /* Channel 2 uses TX_PRU */
                offset = HDSL_DMEM_OFFSET_TX_PRU;
            }

            if(attrs->pruicss_slice)
            {
                priv->base_mem_addr = (uint32_t *)((hw_attrs->pru1DramBase) + offset);
            }
            else
            {
                priv->base_mem_addr = (uint32_t *)((hw_attrs->pru0DramBase) + offset);
            }
        }

        /* Initialize HDSL interface pointer */
        priv->hdsl_interface = (HDSL_Interface *)(priv->base_mem_addr);

        /* Configure channel mask in PRU memory */
        HW_WR_REG8((uint8_t *)priv->base_mem_addr + CHANNEL_MASK_OFFSET, attrs->channel_mask);

        /* Initialize multi_turn, res and mask to 0 (can be configured later by application) */
        priv->multi_turn = 0;
        priv->res = 0;
        priv->mask = 0;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Mark handle as open after successful initialization */
        priv->is_open = 1;
    }
    else
    {
        handle = NULL;
    }

    return handle;
}

void HDSL_close(HDSL_Handle handle)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return;
    }

    priv = HDSL_get_priv(handle);

    /* Reset is_open in priv structure */
    priv->is_open = 0;

    return;
}

const HDSL_Attrs* HDSL_get_attrs(HDSL_Handle handle)
{
    if(handle == NULL)
    {
        return NULL;
    }

    return handle->attrs;
}

HDSL_Priv* HDSL_get_priv(HDSL_Handle handle)
{
    if(handle == NULL)
    {
        return NULL;
    }

    return handle->priv;
}

int32_t HDSL_get_pos(HDSL_Handle handle, uint32_t position_id, uint64_t *position)
{
    HDSL_Interface *hdsl_interface_struct;
    HDSL_Priv *priv;

    /* Validate inputs */
    if((handle == NULL) || (position == NULL))
    {
        return SystemP_FAILURE;
    }

    if(position_id > HDSL_MAX_POSITION_ID)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    hdsl_interface_struct = priv->hdsl_interface;

    switch(position_id)
    {
        case 0:
            /* Fast Position */
            *position = hdsl_interface_struct->POS0 | (hdsl_interface_struct->POS1 << 8) |
                       (hdsl_interface_struct->POS2 << 16) | (hdsl_interface_struct->POS3 << 24);
            *position |= (uint64_t)hdsl_interface_struct->POS4 << 32;
            break;

        case 1:
            /* Safe Position 1 */
            *position = hdsl_interface_struct->VPOS0 | (hdsl_interface_struct->VPOS1 << 8) |
                       (hdsl_interface_struct->VPOS2 << 16) | (hdsl_interface_struct->VPOS3 << 24);
            *position |= (uint64_t)hdsl_interface_struct->VPOS4 << 32;
            break;

        case 2:
            /* Safe Position 2 */
            *position = hdsl_interface_struct->VPOS20 | (hdsl_interface_struct->VPOS21 << 8) |
                       (hdsl_interface_struct->VPOS22 << 16) | (hdsl_interface_struct->VPOS23 << 24);
            *position |= (uint64_t)hdsl_interface_struct->VPOS24 << 32;
            break;

        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_get_qm(HDSL_Handle handle, uint8_t *qm)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (qm != NULL))
    {
        priv = HDSL_get_priv(handle);

        *qm = priv->hdsl_interface->MASTER_QM & HDSL_QM_VALUE_MASK;
        status = SystemP_SUCCESS;
    }

    return status;
}


int32_t HDSL_get_events(HDSL_Handle handle, uint16_t *events)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (events != NULL))
    {
        priv = HDSL_get_priv(handle);

        *events = priv->hdsl_interface->EVENT_L | (priv->hdsl_interface->EVENT_H << 8);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_safe_events(HDSL_Handle handle, uint8_t *events)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (events != NULL))
    {
        priv = HDSL_get_priv(handle);

        *events = priv->hdsl_interface->EVENT_S;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_online_status_d(HDSL_Handle handle, uint16_t *status)
{
    int32_t ret_status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (status != NULL))
    {
        priv = HDSL_get_priv(handle);

        *status = priv->hdsl_interface->ONLINE_STATUS_D_L | (priv->hdsl_interface->ONLINE_STATUS_D_H << 8);
        ret_status = SystemP_SUCCESS;
    }

    return ret_status;
}

int32_t HDSL_get_online_status_1(HDSL_Handle handle, uint16_t *status)
{
    int32_t ret_status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (status != NULL))
    {
        priv = HDSL_get_priv(handle);

        *status = priv->hdsl_interface->ONLINE_STATUS_1_L | (priv->hdsl_interface->ONLINE_STATUS_1_H << 8);
        ret_status = SystemP_SUCCESS;
    }

    return ret_status;
}

int32_t HDSL_get_online_status_2(HDSL_Handle handle, uint16_t *status)
{
    int32_t ret_status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (status != NULL))
    {
        priv = HDSL_get_priv(handle);

        *status = priv->hdsl_interface->ONLINE_STATUS_2_L | (priv->hdsl_interface->ONLINE_STATUS_2_H << 8);
        ret_status = SystemP_SUCCESS;
    }

    return ret_status;
}

int32_t HDSL_get_sum(HDSL_Handle handle, uint8_t *sum)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (sum != NULL))
    {
        priv = HDSL_get_priv(handle);

        *sum = priv->hdsl_interface->SAFE_SUM;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_acc_err_cnt(HDSL_Handle handle, uint8_t *count)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (count != NULL))
    {
        priv = HDSL_get_priv(handle);

        *count = (uint8_t)(priv->hdsl_interface->ACC_ERR_CNT & HDSL_ACC_ERR_CNT_MASK);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_rssi(HDSL_Handle handle, uint8_t *rssi)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (rssi != NULL))
    {
        priv = HDSL_get_priv(handle);

        *rssi = (priv->hdsl_interface->DELAY & HDSL_RSSI_MASK) >> HDSL_RSSI_SHIFT;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_write_pc_short_msg(HDSL_Handle handle, uint8_t addr, uint8_t data, uint64_t timeout)
{
    uint64_t end;
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    end = ClockP_getTimeUsec() + timeout;

    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    priv->hdsl_interface->S_PC_DATA = data;
    priv->hdsl_interface->SLAVE_REG_CTRL =  addr;
    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 0)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    return SystemP_SUCCESS;
}

int32_t HDSL_read_pc_short_msg(HDSL_Handle handle, uint8_t addr, uint8_t *data, uint64_t timeout)
{
    uint64_t end;
    HDSL_Priv *priv;

    if((handle == NULL) || (data == NULL))
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    end = ClockP_getTimeUsec() + timeout;

    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    priv->hdsl_interface->S_PC_DATA = 0;
    priv->hdsl_interface->SLAVE_REG_CTRL = (addr | HDSL_SHORT_MSG_READ_BIT);
    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 0)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    while((priv->hdsl_interface->ONLINE_STATUS_1_L & ONLINE_STATUS_1_L_FRES) != 1)
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }
    *data = priv->hdsl_interface->S_PC_DATA;
    return SystemP_SUCCESS;
}

int32_t HDSL_write_pc_long_msg(HDSL_Handle handle, uint16_t addr, uint8_t offsetEnable, uint8_t addrType, uint8_t length, uint16_t offset, uint64_t timeout)
{
    uint64_t end;
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Validate parameters */
    if((addr > 0x3FF) ||                              /* 10-bit address */
       (offsetEnable > 1) ||                          /* Boolean: 0 or 1 */
       (addrType > 1) ||                              /* DIRECT=0, INDIRECT=1 */
       (length > HDSL_LONG_MSG_LENGTH_8) ||           /* 0-3 valid */
       (offset > 0x7FFF))                             /* 15-bit offset */
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    end = ClockP_getTimeUsec() + timeout;

    while(!(priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    /*
        Setting PC_ADD_L
        Bits 7:0 contain bits 7:0 of 10 bit address for long message
    */
    priv->hdsl_interface->PC_ADD_L = (addr & PC_ADD_L_ADDR_LOW_MASK);

    /*
        Setting PC_ADD_H
        Bit 7 should always be set for long message
        Bit 6 is unset for write operation
        Bit 5 is to enable/disable offset
        Bit 4 is to select direct/indirect addressing
        Bits 3:2 define the length of the message
        Bits 1:0 contain bits 9:8 of 10 bit address for long message
    */
    priv->hdsl_interface->PC_ADD_H = (PC_ADD_H_LONG_MSG_ENABLE) |
                                          (PC_ADD_H_LONG_MSG_WRITE) |
                                          (offsetEnable << PC_ADD_H_OFFSET_EN_SHIFT) |
                                          (addrType << PC_ADD_H_ADDR_TYPE_SHIFT) |
                                          (length << PC_ADD_H_LENGTH_SHIFT) |
                                          ((addr & PC_ADD_H_ADDR_HIGH_MASK) >> 8);

    /*
        Setting PC_OFF_L
        Bits 7:0 contain bits 7:0 of 15 bit offset value
    */
    priv->hdsl_interface->PC_OFF_L = (offset & PC_OFF_L_OFFSET_LOW_MASK);

    /*
        Setting PC_OFF_H
        Bit 7 should always be set for long message
        Bits 6:0 contain bits 14:8 of 15 bit offset value
    */
    priv->hdsl_interface->PC_OFF_H = (PC_OFF_H_LONG_MSG_ENABLE) |
                                          ((offset & PC_OFF_H_OFFSET_HIGH_MASK) >> 8);

    /* Setting PC_CTRL */
    priv->hdsl_interface->PC_CTRL =  PC_CTRL_ENABLE;

    while((priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    while(!(priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    /* Checking for error */
    if(priv->hdsl_interface->PC_ADD_H & PC_ADD_H_LONG_MSG_ERROR)
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_read_pc_long_msg(HDSL_Handle handle, uint16_t addr, uint8_t offsetEnable, uint8_t addrType, uint8_t length, uint16_t offset, uint64_t timeout)
{
    uint64_t end;
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Validate parameters */
    if((addr > 0x3FF) ||                                    /* 10-bit address */
       (offsetEnable > HDSL_LONG_MSG_ADDR_WITH_OFFSET) ||   /* Boolean: 0 or 1 */
       (addrType > HDSL_LONG_MSG_ADDR_INDIRECT) ||          /* DIRECT = 0, INDIRECT = 1 */
       (length > HDSL_LONG_MSG_LENGTH_8) ||                 /* 0-3 valid */
       (offset > 0x7FFF))                                   /* 15-bit offset */
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    end = ClockP_getTimeUsec() + timeout;

    while(!(priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    /*
        Setting PC_ADD_L
        Bits 7:0 contain bits 7:0 of 10 bit address for long message
    */
    priv->hdsl_interface->PC_ADD_L = (addr & PC_ADD_L_ADDR_LOW_MASK);

    /*
        Setting PC_ADD_H
        Bit 7 should always be set for long message
        Bit 6 is set for read operation
        Bit 5 is to enable/disable offset
        Bit 4 is to select direct/indirect addressing
        Bits 3:2 define the length of the message
        Bits 1:0 contain bits 9:8 of 10 bit address for long message
    */
    priv->hdsl_interface->PC_ADD_H = (PC_ADD_H_LONG_MSG_ENABLE) |
                                          (PC_ADD_H_LONG_MSG_READ) |
                                          (offsetEnable << PC_ADD_H_OFFSET_EN_SHIFT) |
                                          (addrType << PC_ADD_H_ADDR_TYPE_SHIFT) |
                                          (length << PC_ADD_H_LENGTH_SHIFT) |
                                          ((addr & PC_ADD_H_ADDR_HIGH_MASK) >> 8);

    /*
        Setting PC_OFF_L
        Bits 7:0 contain bits 7:0 of 15 bit offset value
    */
    priv->hdsl_interface->PC_OFF_L = (offset & PC_OFF_L_OFFSET_LOW_MASK);

    /*
        Setting PC_OFF_H
        Bit 7 should always be set for long message
        Bits 6:0 contain bits 14:8 of 15 bit offset value
    */
    priv->hdsl_interface->PC_OFF_H = (PC_OFF_H_LONG_MSG_ENABLE) |
                                          ((offset & PC_OFF_H_OFFSET_HIGH_MASK) >> 8);

    /* Setting PC_CTRL */
    priv->hdsl_interface->PC_CTRL =  PC_CTRL_ENABLE;

    while((priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    while(!(priv->hdsl_interface->ONLINE_STATUS_D_L & (1<<ONLINE_STATUS_D_L_FREL)))
    {
        if(ClockP_getTimeUsec() > end)
        {
            return SystemP_TIMEOUT;
        }
    }

    /* Checking for error */

    if(priv->hdsl_interface->PC_ADD_H & PC_ADD_H_LONG_MSG_ERROR)
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_write_pc_buffer(HDSL_Handle handle, uint8_t buff_off, uint8_t data)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    /* Bounds check on buff_off parameter (0-7) */
    if(buff_off > HDSL_PC_BUFFER_MAX_INDEX)
    {
        return SystemP_FAILURE;
    }

    switch(buff_off)
    {
        case 0:
            priv->hdsl_interface->PC_BUFFER0 = data;
            break;
        case 1:
            priv->hdsl_interface->PC_BUFFER1 = data;
            break;
        case 2:
            priv->hdsl_interface->PC_BUFFER2 = data;
            break;
        case 3:
            priv->hdsl_interface->PC_BUFFER3 = data;
            break;
        case 4:
            priv->hdsl_interface->PC_BUFFER4 = data;
            break;
        case 5:
            priv->hdsl_interface->PC_BUFFER5 = data;
            break;
        case 6:
            priv->hdsl_interface->PC_BUFFER6 = data;
            break;
        case 7:
            priv->hdsl_interface->PC_BUFFER7 = data;
            break;
        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_read_pc_buffer(HDSL_Handle handle, uint8_t buff_off, uint8_t *data)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    if(data == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    /* Bounds check on buff_off parameter (0-7) */
    if(buff_off > HDSL_PC_BUFFER_MAX_INDEX)
    {
        return SystemP_FAILURE;
    }

    switch(buff_off)
    {
        case 0:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER0);
            break;
        case 1:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER1);
            break;
        case 2:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER2);
            break;
        case 3:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER3);
            break;
        case 4:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER4);
            break;
        case 5:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER5);
            break;
        case 6:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER6);
            break;
        case 7:
            *data = (uint8_t)(priv->hdsl_interface->PC_BUFFER7);
            break;
        default:
            return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_get_sync_ctrl(HDSL_Handle handle, uint8_t *ctrl)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (ctrl != NULL))
    {
        priv = HDSL_get_priv(handle);

        *ctrl = (uint8_t)(priv->hdsl_interface->SYNC_CTRL);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_set_sync_ctrl(HDSL_Handle handle, uint8_t val)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->hdsl_interface->SYNC_CTRL = val;
    return SystemP_SUCCESS;
}

int32_t HDSL_get_master_qm(HDSL_Handle handle, uint8_t *qm)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (qm != NULL))
    {
        priv = HDSL_get_priv(handle);

        *qm = (uint8_t)priv->hdsl_interface->MASTER_QM;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_edges(HDSL_Handle handle, uint8_t *edges)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (edges != NULL))
    {
        priv = HDSL_get_priv(handle);

        *edges = (uint8_t)priv->hdsl_interface->EDGES;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_set_pc_addr(HDSL_Handle handle, uint8_t pc_addrh, uint8_t pc_addrl, uint8_t pc_offh, uint8_t pc_offl)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->hdsl_interface->PC_ADD_L = pc_addrl;
    priv->hdsl_interface->PC_ADD_H = pc_addrh;
    priv->hdsl_interface->PC_OFF_L = pc_offl;
    priv->hdsl_interface->PC_OFF_H = pc_offh;

    return SystemP_SUCCESS;
}

int32_t HDSL_set_pc_ctrl(HDSL_Handle handle, uint8_t value)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->hdsl_interface->PC_CTRL = value;
    return SystemP_SUCCESS;
}

int32_t HDSL_get_delay(HDSL_Handle handle, uint8_t *delay)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (delay != NULL))
    {
        priv = HDSL_get_priv(handle);

        *delay = (uint8_t)priv->hdsl_interface->DELAY;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_enc_id(HDSL_Handle handle, uint32_t byte, uint8_t *enc_id)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (enc_id != NULL))
    {
        priv = HDSL_get_priv(handle);

        /* Bounds check on byte parameter (0-2) */
        if(byte < HDSL_NUM_ENC_ID_BYTES)
        {
            switch(byte)
            {
                case 0:
                    *enc_id = (uint8_t)priv->hdsl_interface->ENC_ID0;
                    status = SystemP_SUCCESS;
                    break;
                case 1:
                    *enc_id = (uint8_t)priv->hdsl_interface->ENC_ID1;
                    status = SystemP_SUCCESS;
                    break;
                case 2:
                    *enc_id = (uint8_t)priv->hdsl_interface->ENC_ID2;
                    status = SystemP_SUCCESS;
                    break;
                default:
                    break;
            }
        }
    }

    return status;
}

int32_t HDSL_get_src_loc(HDSL_Handle handle, void **src_loc)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (src_loc != NULL))
    {
        priv = HDSL_get_priv(handle);

        /* returns HDSL interface struct memory location */
        *src_loc = (void *)priv->hdsl_interface;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_get_length(HDSL_Handle handle, uint32_t *length)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (length != NULL))
    {
        priv = HDSL_get_priv(handle);

        *length = sizeof(*(priv->hdsl_interface));
        status = SystemP_SUCCESS;
    }

    return status;
}


int32_t HDSL_config_copy_table(HDSL_Handle handle, const HDSL_CopyTable *copy_table)
{
    HDSL_Priv *priv;

    if((handle == NULL) || (copy_table == NULL))
    {
        return SystemP_FAILURE;
    }

    /* Validate that all addresses and sizes fit in 16 bits */
    if((copy_table->load_addr1 > 0xFFFFU) || (copy_table->run_addr1 > 0xFFFFU) ||
       (copy_table->size1 > 0xFFFFU) || (copy_table->load_addr2 > 0xFFFFU) ||
       (copy_table->run_addr2 > 0xFFFFU) || (copy_table->size2 > 0xFFFFU))
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART1_LOAD_START_OFFSET, (uint16_t)copy_table->load_addr1);
    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART1_RUN_START_OFFSET, (uint16_t)copy_table->run_addr1);
    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART1_SIZE_OFFSET, (uint16_t)copy_table->size1);
    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART2_LOAD_START_OFFSET, (uint16_t)copy_table->load_addr2);
    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART2_RUN_START_OFFSET, (uint16_t)copy_table->run_addr2);
    HW_WR_REG16((uint8_t *)priv->base_mem_addr + PART2_SIZE_OFFSET, (uint16_t)copy_table->size2);

    return SystemP_SUCCESS;
}

int32_t HDSL_hw_init(HDSL_Handle handle)
{
    int32_t             status;
    HDSL_Priv           *priv;
    const HDSL_Attrs    *attrs;
    void                *pruicss_cfg;
    uint32_t            reg_val;
    uint16_t            rx_div;
    /**< Rx clock divisor (value-1 written to register). Determines receive sample rate.
     *   Formula: rx_clk = source_clk / ((rx_div + 1) * rx_oversampling) */
    uint16_t            tx_div;
    /**< Tx clock divisor (value-1 written to register). Determines transmit baud rate.
     *   Formula: tx_clk = source_clk / (tx_div + 1) */
    uint16_t            rx_div_attr;
    /**< Rx oversampling rate, start bit polarity and fractional divider configuration.
     *   Bits [2:0] : Oversampling divisor (7 = 8x, 5 = 6x, 3 = 4x)
     *   Bits [3]   : Start bit polarity (0 or 1)
     *   Bit  [15]  : Fractional divider enable (1=enable 1.5x fractional division) */
    uint16_t            is_core_clk;
    /**< Clock source selection for HDSL communication.
     *   0 = Use UART clock
     *   1 = Use Core clock */

    /* Validate handle */
    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = handle->priv;
    attrs = handle->attrs;
    pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(priv->pruicss_handle->hwAttrs))->cfgRegBase);

    /* Set GP MUX to EnDAT mode for position sensing */
    status = PRUICSS_setGpMuxSelect(priv->pruicss_handle, attrs->pruicss_slice, PRUICSS_GP_MUX_SEL_MODE_ENDAT);

    if(status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    /* HDSL always uses Core clock (no UART clock support) */
    /* For 8x oversampling, use value of 7. Use "1" as start bit for receive */
    rx_div_attr = (HDSL_SB_POLARITY << 3) | (HDSL_OVERSAMPLE_RATE_8X - 1);
    is_core_clk = 1;  /* Always use core clock */

    tx_div = (attrs->core_clk_freq * (HDSL_OVERSAMPLE_RATE_8X) / (HDSL_DEFAULT_RX_CLOCK_RATE)) - 1;
    rx_div = (attrs->core_clk_freq / (HDSL_DEFAULT_RX_CLOCK_RATE)) - 1;

    if(attrs->pruicss_slice)
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG,
        ((rx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_DIV_FACTOR_SHIFT) |
         (is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG_PRU1_ED_RX_CLK_SEL_SHIFT) |
         (rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG,
        (tx_div << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_DIV_FACTOR_SHIFT) |
        (is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ED_TX_CLK_SEL_SHIFT));
    }
    else
    {
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG,
        ((rx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_DIV_FACTOR_SHIFT) |
         (is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG_PRU0_ED_RX_CLK_SEL_SHIFT) |
         (rx_div_attr)));
        HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG,
        (tx_div << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_DIV_FACTOR_SHIFT) |
        (is_core_clk << CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ED_TX_CLK_SEL_SHIFT));
    }

    /* Enable load share mode */
    if(attrs->load_share_enabled)
    {
        if(attrs->pruicss_slice)
        {
           reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
           reg_val |= CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ENDAT_SHARE_EN_MASK;
           HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, reg_val);
        }
        else
        {
            reg_val = HW_RD_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
            reg_val |= CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ENDAT_SHARE_EN_MASK;
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, reg_val);
        }
    }

    /* Clear channel specific CFG0 registers */
    if(attrs->pruicss_slice)
    {
        if(attrs->channel0_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG, 0);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG, 0);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG, 0);
        }
    }
    else
    {
        if(attrs->channel0_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG, 0);
        }
        if(attrs->channel1_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG, 0);
        }
        if(attrs->channel2_enabled)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG, 0);
        }
    }

    return SystemP_SUCCESS;
}

int32_t HDSL_set_res(HDSL_Handle handle, uint32_t res)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->res = res;
    return SystemP_SUCCESS;
}

int32_t HDSL_get_res(HDSL_Handle handle, uint32_t *res)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (res != NULL))
    {
        priv = HDSL_get_priv(handle);

        *res = priv->res;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_set_multi_turn(HDSL_Handle handle, uint32_t multi_turn)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->multi_turn = multi_turn;
    return SystemP_SUCCESS;
}

int32_t HDSL_get_multi_turn(HDSL_Handle handle, uint32_t *multi_turn)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (multi_turn != NULL))
    {
        priv = HDSL_get_priv(handle);

        *multi_turn = priv->multi_turn;
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t HDSL_set_mask(HDSL_Handle handle, uint64_t mask)
{
    HDSL_Priv *priv;

    if(handle == NULL)
    {
        return SystemP_FAILURE;
    }

    priv = HDSL_get_priv(handle);

    priv->mask = mask;
    return SystemP_SUCCESS;
}

int32_t HDSL_get_mask(HDSL_Handle handle, uint64_t *mask)
{
    int32_t status = SystemP_FAILURE;
    HDSL_Priv *priv;

    if((handle != NULL) && (mask != NULL))
    {
        priv = HDSL_get_priv(handle);

        *mask = priv->mask;
        status = SystemP_SUCCESS;
    }

    return status;
}
