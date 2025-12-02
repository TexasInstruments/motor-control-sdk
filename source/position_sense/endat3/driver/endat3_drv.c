/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#include <string.h>
#include <position_sense/endat3/include/endat3_drv.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_icss.h>
#include <kernel/dpl/ClockP.h>
#include <stdint.h>
#include <stddef.h>

/* ========================================================================== */
/*                    Internal Function Forward Declarations                  */
/* ========================================================================== */

/* External function defined in endat3_lut.c */
void endat3_generate_memory_image(endat3_Handle endat3Handle, PRUICSS_Handle icssgHandle);

/* Static internal functions */
static void endat3_initConfig(endat3_Config_t *config, uint32_t load_share_mode, const endat3_ClockConfig_t *clock_config);
static void endat3_configurePruRegisters(void *pru_cfg_base, const endat3_Config_t *config, uint32_t pru_slice);
static void endat3_configure_tx_rx_clocks(void *pru_cfg_base, uint32_t load_share_mode, uint32_t pru_slice, const endat3_ClockConfig_t *clock_config);
static int32_t endat3_set_delay_cycles(endat3_Handle handle, uint64_t pru_freq_hz);
static int32_t endat3_setChannelMask(endat3_Handle handle, uint8_t channel_mask);
static void endat3_config_endat_mode(endat3_Handle priv, uint8_t pruicss_slicex);
static void endat3_enable_load_share_mode(void *pru_cfg, uint32_t pru_slice);
static uint8_t endat3_calculate_crc(uint8_t *data, uint32_t len);
static int32_t endat3_process_frame(endat3_Handle priv, uint8_t *buffer, uint32_t length);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENDAT3_CRC_POLY    0x1A7
#define ENDAT3_CRC_INIT    0xFF

#define ENDAT3_PREAMBLE_SIZE 4
#define ENDAT3_PREAMBLE_PATTERN {0xCC, 0xCD, 0x94, 0x01}

/* EnDAT3 Memory Map Offsets for Load Share Mode
 * RTU_PRU core:   0x0000 - 0x06FF
 * PRU core:       0x0700 - 0x0DFF
 * TX_PRU core:    0x0E00 - 0x1500
 */
#define DMEM_BASE_OFFSET_RTU_PRU    0x0000
#define DMEM_BASE_OFFSET_PRU        0x0700
#define DMEM_BASE_OFFSET_TX_PRU     0x0E00

/* Maximum TX frames supported by protocol */
#define ENDAT3_MAX_TX_FRAMES        255

/* ========================================================================== */
/*                         Private Function Prototypes                        */
/* ========================================================================== */

static int32_t endat3_prepare_request(endat3_Handle priv, 
                                  uint8_t cmd, 
                                  uint32_t *data_array, 
                                  uint8_t num_frames);
static int32_t endat3_wait_rx_complete(endat3_Handle priv);
static void endat3_parse_frames(endat3_Handle priv, uint8_t *rx_buffer);

/* Global handle storage */
endat3_priv_t endat3Config0;
endat3_priv_t endat3Config1;
endat3_priv_t endat3Config2;

/* ========================================================================== */
/*                          Public Functions                                  */
/* ========================================================================== */

int32_t endat3_getLastError(endat3_Handle handle)
{
    if (handle == NULL)
    {
        return ENDAT3_ERR_INVALID_HANDLE;
    }
    return handle->last_error;
}

int32_t endat3_initClockConfig(endat3_ClockConfig_t *clock_config, uint64_t pru_freq_hz, uint32_t baud_rate)
{
    uint32_t target_tx_clock_hz;
    uint32_t target_rx_clock_hz;
    uint32_t oversample_rate;

    if (clock_config == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    oversample_rate = 8;
    clock_config->oversample_rate = oversample_rate - 1;

    if (baud_rate == 0)
    {
        /* 12.5 Mbps: TX=25MHz, RX=100MHz */
        target_tx_clock_hz = ENDAT3_TX_CLOCK_FREQ_12_5_MBPS;
        target_rx_clock_hz = (ENDAT3_TX_CLOCK_FREQ_12_5_MBPS / 2) * oversample_rate;
    }
    else
    {
        /* 25 Mbps: TX=50MHz, RX=200MHz */
        target_tx_clock_hz = ENDAT3_TX_CLOCK_FREQ_25_MBPS;
        target_rx_clock_hz = (ENDAT3_TX_CLOCK_FREQ_25_MBPS / 2) * oversample_rate;
    }

    /* div_factor = (pru_freq / target_clock) - 1 */
    clock_config->div_factor_normal = (uint32_t)((pru_freq_hz / target_tx_clock_hz) - 1);
    clock_config->div_factor_oversampled = (uint32_t)((pru_freq_hz / target_rx_clock_hz) - 1);

    clock_config->pru_clock_type = 0x10;
    clock_config->uart_clock_type = 0x0;
    clock_config->clock_type = 0x10;
    clock_config->sb_polarity = ENDAT3_SB_POLARITY;

    return ENDAT3_SUCCESS;
}

endat3_Handle endat3_open(PRUICSS_Handle icssHandle, uint32_t icssCore, uint8_t pruMode, uint64_t pru_freq_hz, uint8_t channel_mask, uint32_t baud_rate)
{
    endat3_Handle endat3Handle = NULL;
    void *pru_cfg;
    uint32_t pru_slice = 0;
    endat3_ClockConfig_t clock_config;

    if (endat3_initClockConfig(&clock_config, pru_freq_hz, baud_rate) != ENDAT3_SUCCESS)
    {
        /* Cannot set handle->last_error here as handle not yet determined */
        return NULL;
    }

    if (pruMode == 0)
    {
        endat3Handle = &endat3Config0;
        if (icssCore == 0)
        {
            endat3Handle->baseMemAddr = (uint32_t *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase);
            pru_slice = 0;
        }
        else if (icssCore == 1)
        {
            endat3Handle->baseMemAddr = (uint32_t *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase);
            pru_slice = 1;
        }
        else
        {
            /* Invalid icssCore value for single channel mode */
            endat3Handle = NULL;
        }
    }
    else
    {
        /* Load share mode - use memory map offsets defined in macros */
        /* Note: pruMode=1 (load-share) is only supported on AM243x/AM64x with ICSSG.
         * AM261x with ICSSM does not support load-share mode and will return NULL. */

        /* Handle Slice 1 cores */
        if (icssCore == ENDAT3_PRUICSS_RTU_PRU1)
        {
            endat3Handle = &endat3Config0;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_RTU_PRU);
            pru_slice = 1;
        }
        else if (icssCore == ENDAT3_PRUICSS_PRU1)
        {
            endat3Handle = &endat3Config1;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_PRU);
            pru_slice = 1;
        }
        else if (icssCore == ENDAT3_PRUICSS_TX_PRU1)
        {
            endat3Handle = &endat3Config2;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_TX_PRU);
            pru_slice = 1;
        }
        /* Handle Slice 0 cores */
        else if (icssCore == ENDAT3_PRUICSS_RTU_PRU0)
        {
            endat3Handle = &endat3Config0;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_RTU_PRU);
            pru_slice = 0;
        }
        else if (icssCore == ENDAT3_PRUICSS_PRU0)
        {
            endat3Handle = &endat3Config1;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_PRU);
            pru_slice = 0;
        }
        else if (icssCore == ENDAT3_PRUICSS_TX_PRU0)
        {
            endat3Handle = &endat3Config2;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_TX_PRU);
            pru_slice = 0;
        }
        else
        {
            /* No valid handle to set error on */
            endat3Handle = NULL;
        }
    }

    if (endat3Handle != NULL)
    {
        endat3Handle->icssHandle   = icssHandle;
        endat3Handle->icssCore      = icssCore;
        endat3Handle->endat3Interface = (endat3_Interface *) endat3Handle->baseMemAddr;
        endat3Handle->pruicss_cfg = (void *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->cfgRegBase);
        endat3Handle->last_error = 0;  /* Initialize error code */

        /* Generate memory image */
        endat3_generate_memory_image(endat3Handle, icssHandle);

        /* Set delay cycles based on PRU frequency */
        if (endat3_set_delay_cycles(endat3Handle, pru_freq_hz) != ENDAT3_SUCCESS)
        {
            endat3Handle->last_error = ENDAT3_ERR_DELAY_CONFIG;
            return NULL;
        }

        /* Set channel mask */
        if (endat3_setChannelMask(endat3Handle, channel_mask) != ENDAT3_SUCCESS)
        {
            endat3Handle->last_error = ENDAT3_ERR_CHANNEL_CONFIG;
            return NULL;
        }

        /* Configure EnDAT mode */
        endat3_config_endat_mode(endat3Handle, pru_slice);

        /* Configure TX/RX clocks */
        pru_cfg = (void *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->cfgRegBase);
        endat3_configure_tx_rx_clocks(pru_cfg, pruMode, pru_slice, &clock_config);
    }

    return endat3Handle;
}

static int32_t endat3_prepare_request(endat3_Handle priv, uint8_t cmd, uint32_t *data_array, uint8_t num_frames)
{
    uint8_t i, k;
    uint8_t temp;
    uint8_t frame_length;

    /* Validate input parameters - buffer overflow protection */
    if (priv == NULL || priv->endat3Interface == NULL)
    {
        if (priv != NULL) priv->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_HANDLE; /* Invalid handle */
    }

    if (num_frames > ENDAT3_MAX_TX_FRAMES || num_frames * 4 > sizeof(priv->endat3Interface->tx_buffer))
    {
        priv->last_error = ENDAT3_ERR_INVALID_PARAM;
        return ENDAT3_ERR_INVALID_PARAM; /* Buffer overflow protection - num_frames exceeds buffer capacity */
    }

    if (data_array == NULL)
    {
        priv->last_error = ENDAT3_ERR_INVALID_PARAM;
        return ENDAT3_ERR_INVALID_PARAM; /* Invalid data array pointer */
    }

    /* Process frames, each 4 bytes long */
    for (k = 0; k < num_frames; k++)
    {
        /* Create request frame */
        priv->endat3Interface->tx_buffer[k * 4 + 0] = cmd;                     /* Command code */
        priv->endat3Interface->tx_buffer[k * 4 + 1] = data_array[k] & 0xFF;    /* LSB of data[k] */
        priv->endat3Interface->tx_buffer[k * 4 + 2] = (data_array[k] >> 8) & 0xFF;  /* MSB of data[k] */
        priv->endat3Interface->tx_buffer[k * 4 + 3] = endat3_calculate_crc(priv->endat3Interface->tx_buffer + k * 4, 3);

        /* Reverse byte order of the entire buffer (4 bytes: cmd, LSB, MSB, CRC) */
        frame_length = 4;  /* Total frame length: command(1) + data(2) + CRC(1) */

        /* Swap bytes from outside toward the center */
        for (i = 0; i < frame_length / 2; i++)
        {
            temp = priv->endat3Interface->tx_buffer[k * 4 + i];
            priv->endat3Interface->tx_buffer[k * 4 + i] = priv->endat3Interface->tx_buffer[k * 4 + frame_length - 1 - i];
            priv->endat3Interface->tx_buffer[k * 4 + frame_length - 1 - i] = temp;
        }
    }

    return ENDAT3_SUCCESS;
}


static int32_t endat3_wait_rx_complete(endat3_Handle priv)
{
    /* Return sampling error if encoder detected an error condition */
    if (priv->endat3Interface->busy == ENCODER_ERROR)
    {
        return ENDAT3_ERR_SAMPLING_ERROR;
    }
    /* Return timeout while a transfer is still in progress */
    if (priv->endat3Interface->busy == ENCODER_BUSY)
    {
        return ENDAT3_ERR_TIMEOUT;
    }
    return ENDAT3_SUCCESS;
}

static void endat3_parse_frames(endat3_Handle priv, uint8_t *rx_buffer)
{
    /* Parse HPF */
    memcpy(priv->endat3Interface->hpf.data, rx_buffer, 6);
    priv->endat3Interface->hpf.status = rx_buffer[6];
    priv->endat3Interface->hpf.crc = rx_buffer[7];

    /* Parse LPH */
    priv->endat3Interface->lph.status = rx_buffer[8];
    priv->endat3Interface->lph.num_lpf  = rx_buffer[9];
    priv->endat3Interface->lph.crc = rx_buffer[11];

    /* Parse LPF[0] */
    priv->endat3Interface->lpf[0].status = rx_buffer[12];  /* Copy status byte */

    /* Copy data payload byte by byte */
    for (int j = 0; j < 6; j++)
    {
        priv->endat3Interface->lpf[0].data[j] = rx_buffer[13 + j];
    }

    priv->endat3Interface->lpf[0].crc = rx_buffer[19];  /* Copy CRC byte */
}


/* ========================================================================== */
/*                          Public Functions                                  */
/* ========================================================================== */

int32_t endat3_send_command(endat3_Handle priv, uint8_t cmd, uint8_t frames)
{
    int32_t status;

    /* Prepare request frame */
    status = endat3_prepare_request(priv, cmd, priv->endat3Interface->bg_data, frames);

    if (status != ENDAT3_SUCCESS)
    {
        return status;
    }

    return ENDAT3_SUCCESS;
}

int32_t endat3_receive_response(endat3_Handle priv)
{
    int32_t status;

    /* Wait for RX complete */
    status = endat3_wait_rx_complete(priv);
    if (status != ENDAT3_SUCCESS)
    {
        return status;  /* Propagate specific error code (e.g., ENDAT3_ERR_SAMPLING_ERROR) */
    }

    /* Parse received frames */
    endat3_parse_frames(priv, priv->endat3Interface->rx_buffer);

    /* HPF CRC check */
    status = endat3_process_frame(priv, priv->endat3Interface->rx_buffer + 0, 8);
    if (status != ENDAT3_SUCCESS)
    {
        return ENDAT3_ERR_HPF_CRC_FAIL;
    }

    /* LPH CRC check */
    status = endat3_process_frame(priv, priv->endat3Interface->rx_buffer + 8, 4);
    if (status != ENDAT3_SUCCESS)
    {
        return ENDAT3_ERR_LPH_CRC_FAIL;
    }

    /* check for SENDLIST 0 to calculate CRC for LPF only */
    if (priv->endat3Interface->tx_buffer[3] == 0)
    {
        /* LPF CRC check for SENDLIST 0 */
        status = endat3_process_frame(priv, priv->endat3Interface->rx_buffer + 12, 8);
        if (status != ENDAT3_SUCCESS)
        {
            return ENDAT3_ERR_LPF_CRC_FAIL;
        }
    }

    return ENDAT3_SUCCESSFUL_RESPONSE;  /* Success - all CRC checks passed */
}

static uint16_t endat3_reflect_general (uint16_t value, uint16_t width)
{
    uint16_t result = 0;
    uint16_t i;
    
    for (i = 0; i < width; i++)
    {
        if ((value & (1 << i)) != 0)
        {
            result |= (1 << (width - 1 - i));
        }
    }
    return result;
}

static uint16_t endat3_compute_crc(const uint8_t *bytes, size_t length,
                          uint16_t poly, uint16_t crcsize,
                          uint16_t initialValue, uint8_t inputReflected,
                          uint8_t resultReflected, uint16_t finalXor)
{
    /* Calculate bit manipulation constants */
    uint16_t bitsToShift = crcsize - 8;       /* How many bits to shift input byte */
    uint16_t bitmask = 1 << (crcsize - 1);    /* Mask for MSB check */
    uint16_t finalmask = (1 << crcsize) - 1;  /* Mask for final value */
    uint16_t crc = initialValue;
    size_t j;
    int i;
    uint8_t curByte;
    
    /* Process each input byte */
    for (j = 0; j < length; j++)
    {
        /* Reflect input byte if needed */
        curByte = inputReflected ? (uint8_t)endat3_reflect_general(bytes[j], 8) : bytes[j];
        
        /* XOR the byte into the MSB of CRC register */
        crc ^= (uint16_t)(curByte << bitsToShift);
        
        /* Process each bit */
        for (i = 0; i < 8; i++)
        {
            if ((crc & bitmask) != 0)
            {
                /* MSB is set, shift left and XOR with polynomial */
                crc = (uint16_t)((crc << 1) ^ poly);
            }
            else
            {
                /* MSB not set, just shift left */
                crc <<= 1;
            }
        }
        
        /* Apply final mask to keep only relevant bits */
        crc = crc & finalmask;
    }
    
    /* Reflect result if needed */
    if (resultReflected)
    {
        crc = endat3_reflect_general (crc, crcsize);
    }
    
    /* Apply final XOR */
    crc ^= finalXor;

    return crc & finalmask;
}

static uint8_t endat3_calculate_crc(uint8_t *data, uint32_t len)
{
    return (uint8_t)endat3_compute_crc(
        data,
        len,
        0xA7,    /* polynomial */
        8,       /* crcsize */
        0xFF,    /* initialValue */
        1,       /* inputReflected  */
        1,       /* resultReflected  */
        0x00     /* finalXor */
    );
}

static int32_t endat3_process_frame(endat3_Handle priv, uint8_t *buffer, uint32_t length)
{
    uint8_t received_crc;
    uint8_t calculated_crc;

    /* A valid frame must contain at least one data byte and a CRC byte */
    if (length <= 1)
    {
        return ENDAT3_ERR_INVALID_PARAM;   /* Invalid frame length */
    }

    received_crc = buffer[length - 1];
    calculated_crc = endat3_calculate_crc(buffer, length - 1);

    return (received_crc == calculated_crc) ? ENDAT3_SUCCESS : ENDAT3_ERR_HPF_CRC_FAIL;
}

/* Clear CFG0 registers for enabled channels - resets encoder channel configuration */
static void endat3_config_clr_cfg0(endat3_Handle priv, uint8_t pruicss_slicex)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    uint8_t channel_mask = priv->endat3Interface->channel_enable_mask;

    if (pruicss_slicex)
    {
        /* Clear CFG0 registers only for enabled channels */
        if (channel_mask & ENDAT3_CH0_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH0_CFG0_REG, 0);
        }
        if (channel_mask & ENDAT3_CH1_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH1_CFG0_REG, 0);
        }
        if (channel_mask & ENDAT3_CH2_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_CH2_CFG0_REG, 0);
        }
    }
    else
    {
        /* Clear CFG0 registers only for enabled channels */
        if (channel_mask & ENDAT3_CH0_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH0_CFG0_REG, 0);
        }
        if (channel_mask & ENDAT3_CH1_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH1_CFG0_REG, 0);
        }
        if (channel_mask & ENDAT3_CH2_MASK)
        {
            HW_WR_REG32((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_CH2_CFG0_REG, 0);
        }
    }
}

static void endat3_config_endat_mode(endat3_Handle priv, uint8_t pruicss_slicex)
{
    void *pruicss_cfg = priv->pruicss_cfg;

    if (pruicss_slicex)
    {
        HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG1_REG + 3, 4);
    }
    else
    {
        HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG0_REG + 3, 4);
    }

    /* Clear CFG0 registers for all channels - CRITICAL for proper operation */
    endat3_config_clr_cfg0(priv, pruicss_slicex);
}

static void endat3_enable_load_share_mode(void *pru_cfg, uint32_t pru_slice)
{
    uint32_t regVal;

    if (pru_slice == 1)
    {
        regVal = HW_RD_REG32((uint8_t *)pru_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG);
        regVal |= CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG_PRU1_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pru_cfg + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, regVal);
    }
    else
    {
        regVal = HW_RD_REG32((uint8_t *)pru_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG);
        regVal |= CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG_PRU0_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pru_cfg + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, regVal);
    }
}

static void endat3_initConfig(endat3_Config_t *config, uint32_t load_share_mode, const endat3_ClockConfig_t *clock_config)
{
    if (config == NULL || clock_config == NULL)
    {
        return;
    }

    config->clock_config.div_factor_normal = clock_config->div_factor_normal;
    config->clock_config.div_factor_oversampled = clock_config->div_factor_oversampled;
    config->clock_config.pru_clock_type = clock_config->pru_clock_type;
    config->clock_config.uart_clock_type = clock_config->uart_clock_type;
    config->clock_config.clock_type = clock_config->clock_type;
    config->clock_config.oversample_rate = clock_config->oversample_rate;
    config->clock_config.sb_polarity = clock_config->sb_polarity;

    config->reg_config.endat3_enable = ENDAT3_ENABLE_BIT;
    config->reg_config.counter_enable = ENDAT3_CTR_EN;
    config->reg_config.load_share_mode = load_share_mode;

    config->reg_config.tx_config = ENDAT3_COMPUTE_TX_CFG(
        config->clock_config.clock_type,
        config->reg_config.load_share_mode,
        config->clock_config.div_factor_normal
    );

    config->reg_config.rx_config = ENDAT3_COMPUTE_RX_CFG(
        config->clock_config.clock_type,
        config->clock_config.div_factor_oversampled,
        config->clock_config.oversample_rate,
        config->clock_config.sb_polarity
    );
}

static void endat3_configurePruRegisters(void *pru_cfg_base, const endat3_Config_t *config, uint32_t pru_slice)
{
    if (pru_cfg_base == NULL || config == NULL)
    {
        return;
    }

    if (pru_slice == 1)
    {
        /* Enable EnDat3 functionality in GPCFG1 register */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_GPCFG1_REG, config->reg_config.endat3_enable);

        /* Configure PRU1 TX parameters */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_TX_CFG_REG, config->reg_config.tx_config);

        /* Configure PRU1 RX parameters */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_PRU1_ED_RX_CFG_REG, config->reg_config.rx_config);
    }
    else
    {
        /* Enable EnDat3 functionality in GPCFG0 register */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_GPCFG0_REG, config->reg_config.endat3_enable);

        /* Configure PRU0 TX parameters */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_TX_CFG_REG, config->reg_config.tx_config);

        /* Configure PRU0 RX parameters */
        HW_WR_REG32((uint8_t *)pru_cfg_base + CSL_ICSS_PR1_CFG_SLV_PRU0_ED_RX_CFG_REG, config->reg_config.rx_config);
    }
}

static void endat3_configure_tx_rx_clocks(void *pru_cfg_base, uint32_t load_share_mode, uint32_t pru_slice, const endat3_ClockConfig_t *clock_config)
{
    endat3_Config_t config;

    /* Initialize configuration with application-provided clock settings */
    endat3_initConfig(&config, load_share_mode, clock_config);

    /* Configure PRU registers */
    endat3_configurePruRegisters(pru_cfg_base, &config, pru_slice);
}

static int32_t endat3_set_delay_cycles(endat3_Handle handle, uint64_t pru_freq_hz)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* Reference values scaled to 1MHz base frequency:
     * ENDAT3_TX_START_DELAY_1 = 218 cycles (65535 / 300)
     * ENDAT3_TX_START_DELAY_2 = 1748 cycles (524288 / 300)
     * ENDAT3_TX_START_DELAY_3 = 438 cycles (131328 / 300)
     * SAMPLING_DELAY_COUNT = 37 cycles
     * DELAY_10MS = 10486 cycles (3145728 / 300)
     *
     * Note: Original values were for 300MHz PRU frequency
     */
    /* Scale delay values based on actual PRU frequency relative to 1MHz reference */
    /* Formula: actual_cycles = (reference_cycles * actual_freq) / reference_freq */

    handle->endat3Interface->delay_tx_start_1 = (uint32_t)((218ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    handle->endat3Interface->delay_tx_start_2 = (uint32_t)((1748ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    handle->endat3Interface->delay_tx_start_3 = (uint32_t)((438ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    /* delay_sampling: Minimum 37 cycles (encoder-dependent, keep conservative) */
    handle->endat3Interface->delay_sampling = 37;
    /* delay_10ms: Scale from 10486 cycles at 1MHz */
    handle->endat3Interface->delay_10ms = (uint32_t)((10486ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    return ENDAT3_SUCCESS;
}

static int32_t endat3_setChannelMask(endat3_Handle handle, uint8_t channel_mask)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* Set channel enable mask in firmware interface structure
     * Bit 0: Channel 0 enable
     * Bit 1: Channel 1 enable
     * Bit 2: Channel 2 enable
     *
     * In non-load share mode, the firmware reads this mask at initialization
     * and uses it to determine which channels to enable for operations.
     *
     * In load share mode, this value is ignored as each PRU core handles
     * a dedicated channel.
     */
    handle->endat3Interface->channel_enable_mask = channel_mask & ENDAT3_ALL_CH_MASK;

    return ENDAT3_SUCCESS;
}

void endat3_handle_background_command_request(endat3_Handle endat3Handle, int index, int frame_cnt, uint8_t op_code, uint32_t addr_msb, uint32_t addr_lsb, uint32_t data)
{
    switch (op_code)
    {
        case ENDAT3_BGREQ_NOP:
            /* NOP operation: Just need to send arbitrary data */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFF;          /* Arbitrary data */
            endat3Handle->endat3Interface->bg_data[index + 1] = (data >> 8) & 0xFF;   /* Arbitrary data */
            endat3Handle->endat3Interface->bg_data[index + 2] = ENDAT3_BGREQ_NOP;     /* OpCode */
            break;

        case ENDAT3_BGREQ_READ:
            /* READ operation: addr_msb = address[23:16], addr_lsb = address[15:0], data = num_words */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFF;         /* num_words */
            endat3Handle->endat3Interface->bg_data[index + 1] = addr_lsb & 0xFFFF;   /* address[15:0] */
            endat3Handle->endat3Interface->bg_data[index + 2] = (addr_msb & 0xFF) | (ENDAT3_BGREQ_READ << 8);  /* OpCode + address[23:16] */
            break;

        case ENDAT3_BGREQ_WRITE:
            /* WRITE operation: addr_msb = address[23:16], addr_lsb = address[15:0], data = value to write */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFFFF;       /* Word to write */
            endat3Handle->endat3Interface->bg_data[index + 1] = addr_lsb & 0xFFFF;   /* address[15:0] */
            endat3Handle->endat3Interface->bg_data[index + 2] = (addr_msb & 0xFF) | (ENDAT3_BGREQ_WRITE << 8);  /* OpCode + address[23:16] */
            break;

        case ENDAT3_BGREQ_RECONFIGURE:
            /* RECONFIGURE operation: Simply sends the reconfigure command */
            endat3Handle->endat3Interface->bg_data[index + 0] = 0x0000;              /* Reserved (0) */
            endat3Handle->endat3Interface->bg_data[index + 1] = 0x0000;              /* Reserved (0) */
            endat3Handle->endat3Interface->bg_data[index + 2] = ENDAT3_BGREQ_RECONFIGURE << 8;  /* OpCode */
            break;

        case ENDAT3_BGREQ_AUTH:
            /* AUTH operation: addr_msb = usrlevel, addr_lsb/data = password */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFFFF;       /* Password part 2 */
            endat3Handle->endat3Interface->bg_data[index + 1] = addr_lsb & 0xFFFF;   /* Password part 1 */
            endat3Handle->endat3Interface->bg_data[index + 2] = (addr_msb & 0xFF) | (ENDAT3_BGREQ_AUTH << 8);  /* OpCode + User level */
            break;

        case ENDAT3_BGREQ_PROTECT:
        {
            /*
             * PROTECT operation with 2-byte bg_data entries:
             * bg_data[index+0]: Bytes 0-1 (acclevel, mode)
             * bg_data[index+1]: Bytes 2-3 (address[15:0])
             * bg_data[index+2]: Bytes 4-5 (address[23:16], OpCode=0x81)
             */
            uint8_t mode = (data >> 24) & 0xFF;
            uint8_t acclevel = (data >> 16) & 0xFF;
            const char* acclevelDesc;

            /* Pack the first 2 bytes: acclevel (byte 0) and mode (byte 1) */
            endat3Handle->endat3Interface->bg_data[index + 0] = (mode << 8) | acclevel;

            /* Pack the middle 2 bytes: address[15:0] (bytes 2-3) */
            endat3Handle->endat3Interface->bg_data[index + 1] = addr_lsb & 0xFFFF;

            /* Pack the last 2 bytes: address[23:16] (byte 4) and OpCode=0x81 (byte 5) */
            endat3Handle->endat3Interface->bg_data[index + 2] = (0x81 << 8) | (addr_msb & 0xFF);

            /* Get description for access level */
            switch (acclevel)
            {
                case 0:
                    acclevelDesc = "USER";
                    break;
                case 1:
                    acclevelDesc = "OEM2";
                    break;
                case 2:
                    acclevelDesc = "OEM1";
                    break;
                case 3:
                    acclevelDesc = "MANUFACTURER";
                    break;
                default:
                    acclevelDesc = "Unknown";
                    break;
            }

            /* Display mode information with access level descriptions */
            switch (mode)
            {
                case ENDAT3_PROTECT_QUERY:
                    DebugP_log("\r\n PROTECT Mode: QUERY (0x%02X) - Querying current access levels", ENDAT3_PROTECT_QUERY);
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                case ENDAT3_PROTECT_SET_READ:
                    DebugP_log("\r\n PROTECT Mode: SET_READ (0x%02X) - Setting read access level", ENDAT3_PROTECT_SET_READ);
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                case ENDAT3_PROTECT_SET_WRITE:
                    DebugP_log("\r\n PROTECT Mode: SET_WRITE (0x%02X) - Setting write access level", ENDAT3_PROTECT_SET_WRITE);
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                default:
                    DebugP_log("\r\n PROTECT Mode: Unknown (0x%02X) - This will be rejected by the encoder", mode);
                    break;
            }
            break;
        }

        case ENDAT3_BGREQ_SETPASS:
            /* SETPASS operation: addr_msb = usrlevel, addr_lsb/data = password */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFFFF;       /* Password part 2 */
            endat3Handle->endat3Interface->bg_data[index + 1] = addr_lsb & 0xFFFF;   /* Password part 1 */
            endat3Handle->endat3Interface->bg_data[index + 2] = (addr_msb & 0xFF) | (ENDAT3_BGREQ_SETPASS << 8);  /* OpCode + User level */
            break;

        case ENDAT3_BGREQ_LOCATE:
            /* LOCATE operation: addr_msb = not used, addr_lsb = not used, data = ctrl */
            endat3Handle->endat3Interface->bg_data[index + 0] = data & 0xFF;         /* ctrl */
            endat3Handle->endat3Interface->bg_data[index + 1] = 0x0000;              /* Reserved (0) */
            endat3Handle->endat3Interface->bg_data[index + 2] = ENDAT3_BGREQ_LOCATE << 8;  /* OpCode */
            break;

        default:
            /* Invalid operation code, use NOP */
            DebugP_log("\r\n Invalid background request operation code: 0x%02X", op_code);
            endat3Handle->endat3Interface->bg_data[index + 0] = 0x0000;              /* Arbitrary data */
            endat3Handle->endat3Interface->bg_data[index + 1] = 0x0000;              /* Arbitrary data */
            endat3Handle->endat3Interface->bg_data[index + 2] = ENDAT3_BGREQ_NOP << 8;  /* OpCode */
            break;
    }

    /* BG_DATAT=0 for last frame in which we receive BG response in LPFs */
    endat3Handle->endat3Interface->bg_data[index + 3] = 0x0;
    /* Set the frame count for all operations */
    endat3Handle->endat3Interface->expected_tx_frames_count = frame_cnt;
}

const char* endat3_getErrorDescription(endat3_ErrorCode_t error_code)
{
    switch (error_code)
    {
        case ENDAT3_ERR_UNKNOWN:
            return "Unknown error cause";
        case ENDAT3_FGERR_RECONFIGURE:
            return "Device in configuration due to RECONFIGURE";
        case ENDAT3_FGERR_ECHO:
            return "ECHO response";
        case ENDAT3_FGERR_INVALID_FID:
            return "Invalid FID configured";
        case ENDAT3_FGERR_DUPLICATE_FID:
            return "FID selected multiple times in cycle";
        case ENDAT3_FGERR_INVALID_DATA:
            return "Invalid data delivered internally";
        case ENDAT3_FGERR_INT_TRM:
            return "LPF supported but unavailable (value not formed in time)";
        case ENDAT3_FGERR_NO_SENSOR_DATA:
            return "Sensor box data not available";
        case ENDAT3_BGERR_USAGE:
            return "Generic operator error";
        case ENDAT3_BGERR_USAGE_OPCODE:
            return "Invalid or unsupported command code";
        case ENDAT3_BGERR_USAGE_ARGUMENTS:
            return "Invalid arguments";
        case ENDAT3_BGERR_USAGE_SEQUENCE:
            return "Invalid command sequence";
        case ENDAT3_BGERR_USAGE_ACCESS_DENIED:
            return "Access denied; insufficient user level";
        case ENDAT3_BGERR_USAGE_MEM_ADDRESS:
            return "Access to invalid address";
        case ENDAT3_BGERR_USAGE_NO_BG:
            return "Encoder doesn't support background processing";
        case ENDAT3_BGERR_INTERNAL:
            return "Generic exception error in encoder";
        case ENDAT3_BGERR_INTERNAL_MEMORY:
            return "Exception error when accessing memory";
        case ENDAT3_BGERR_INTERNAL_CONFIG:
            return "Exception error: configuration invalid";
        default:
            if ((error_code >= 0x0001) && (error_code <= 0x0FFF))
                return "Unknown foreground error";
            else if ((error_code >= 0x1100) && (error_code <= 0x11FF))
                return "Unknown usage error";
            else if ((error_code >= 0x1200) && (error_code <= 0x12FF))
                return "Unknown internal error";
            else
                return "Unrecognized error code";
    }
}

const char* endat3_getErrorAction(endat3_ErrorCode_t error_code)
{
    switch (error_code)
    {
        case ENDAT3_ERR_UNKNOWN:
            return "Try again";
        case ENDAT3_FGERR_RECONFIGURE:
            return "Try again after configuration completes";
        case ENDAT3_FGERR_ECHO:
            return "No action needed";
        case ENDAT3_FGERR_INVALID_FID:
        case ENDAT3_FGERR_DUPLICATE_FID:
        case ENDAT3_FGERR_NO_SENSOR_DATA:
        case ENDAT3_BGERR_USAGE:
        case ENDAT3_BGERR_USAGE_OPCODE:
        case ENDAT3_BGERR_USAGE_ARGUMENTS:
        case ENDAT3_BGERR_USAGE_SEQUENCE:
        case ENDAT3_BGERR_USAGE_MEM_ADDRESS:
        case ENDAT3_BGERR_USAGE_NO_BG:
            return "Correct the application";
        case ENDAT3_FGERR_INVALID_DATA:
        case ENDAT3_FGERR_INT_TRM:
        case ENDAT3_BGERR_INTERNAL:
        case ENDAT3_BGERR_INTERNAL_MEMORY:
        case ENDAT3_BGERR_INTERNAL_CONFIG:
            return "Try again";
        case ENDAT3_BGERR_USAGE_ACCESS_DENIED:
            return "Authenticate with proper level and try again";
        default:
            if ((error_code >= 0x0001) && (error_code <= 0x0FFF))
                return "Refer to documentation or try again";
            else if ((error_code >= 0x1100) && (error_code <= 0x11FF))
                return "Correct the application";
            else if ((error_code >= 0x1200) && (error_code <= 0x12FF))
                return "Try again";
            else
                return "Treat as unknown error and try again";
    }
}

/* ========================================================================== */
/*                    HPF (High Priority Frame) Access APIs                   */
/* ========================================================================== */

int32_t endat3_getHpfStatus(endat3_Handle handle, uint8_t *status)
{
    if (handle == NULL || handle->endat3Interface == NULL || status == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *status = handle->endat3Interface->hpf.status;
    return ENDAT3_SUCCESS;
}

int32_t endat3_getHpfData(endat3_Handle handle, uint8_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* Manual copy instead of memcpy - more explicit and returns bytes copied */
    for (int i = 0; i < 6; i++)
    {
        data[i] = handle->endat3Interface->hpf.data[i];
    }

    return 6; /* Return number of bytes copied */
}

int32_t endat3_getHpfCrc(endat3_Handle handle, uint8_t *crc)
{
    if (handle == NULL || handle->endat3Interface == NULL || crc == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *crc = handle->endat3Interface->hpf.crc;
    return ENDAT3_SUCCESS;
}

int32_t endat3_getHpfDataAsU64(endat3_Handle handle, uint64_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    uint64_t result = 0;
    uint8_t *hpf_data = handle->endat3Interface->hpf.data;

    /* Pack 6 bytes into uint64_t (little-endian) */
    for (int i = 0; i < 6; i++)
    {
        result |= ((uint64_t)hpf_data[i]) << (i * 8);
    }

    *data = result;
    return ENDAT3_SUCCESS;
}

int32_t endat3_isHpfDataValid(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->hpf.status & ENDAT3_HPF_STATUS_HPFV) != 0;
}

int32_t endat3_hasHpfError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->hpf.status & ENDAT3_HPF_STATUS_F) != 0;
}

int32_t endat3_hasHpfWarning(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->hpf.status & ENDAT3_HPF_STATUS_W) != 0;
}

int32_t endat3_hasAbsoluteValue(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->hpf.status & ENDAT3_HPF_STATUS_RM) != 0;
}

/* ========================================================================== */
/*                    LPH (Low Priority Header) Access APIs                   */
/* ========================================================================== */

int32_t endat3_getLphStatus(endat3_Handle handle, uint8_t *status)
{
    if (handle == NULL || handle->endat3Interface == NULL || status == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *status = handle->endat3Interface->lph.status;
    return ENDAT3_SUCCESS;
}

int32_t endat3_getLphnum_lpf(endat3_Handle handle, uint8_t *num_lpf)
{
    if (handle == NULL || handle->endat3Interface == NULL || num_lpf == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *num_lpf = handle->endat3Interface->lph.num_lpf;
    return ENDAT3_SUCCESS;
}

int32_t endat3_getLphCrc(endat3_Handle handle, uint8_t *crc)
{
    if (handle == NULL || handle->endat3Interface == NULL || crc == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *crc = handle->endat3Interface->lph.crc;
    return ENDAT3_SUCCESS;
}

int32_t endat3_getLphState(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (int32_t)(handle->endat3Interface->lph.status & ENDAT3_LPH_STATE_MASK);
}

int32_t endat3_hasBgError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->lph.status & ENDAT3_LPH_BG_ERR_EXEC) != 0;
}

int32_t endat3_isBgBusy(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->lph.status & ENDAT3_LPH_BG_BUSY) != 0;
}

int32_t endat3_hasBgRtxError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (handle->endat3Interface->lph.status & ENDAT3_LPH_BG_RTX_ERROR) != 0;
}

/* ========================================================================== */
/*                    LPF (Low Priority Frame) Access APIs                    */
/* ========================================================================== */

int32_t endat3_getLpfStatus(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return handle->endat3Interface->lpf[index].status;
}

int32_t endat3_getLpfData(endat3_Handle handle, uint8_t index, uint8_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL || index >= MAX_LPF_COUNT)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* Manual copy instead of memcpy - more explicit and returns bytes copied */
    for (int i = 0; i < 6; i++)
    {
        data[i] = handle->endat3Interface->lpf[index].data[i];
    }

    return 6; /* Return number of bytes copied */
}

int32_t endat3_getLpfCrc(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return handle->endat3Interface->lpf[index].crc;
}

int32_t endat3_getLpfFid(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* FID is typically in the status byte */
    return handle->endat3Interface->lpf[index].status;
}

/* ========================================================================== */
/*                    Communication Control APIs                              */
/* ========================================================================== */

int32_t endat3_isConnected(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return handle->endat3Interface->connected != 0;
}

int32_t endat3_isBusy(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    if ((handle->endat3Interface->busy) == ENCODER_ERROR)
    {
        return ENCODER_IDLE;
    }

    return handle->endat3Interface->busy != 0;
}

int32_t endat3_setBusy(endat3_Handle handle, uint8_t busy)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->busy = busy ? 1 : 0;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getExpectedTxFrameCount(endat3_Handle handle, uint32_t *count)
{
    if (handle == NULL || handle->endat3Interface == NULL || count == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *count = handle->endat3Interface->expected_tx_frames_count;
    return ENDAT3_SUCCESS;
}

int32_t endat3_setExpectedTxFrameCount(endat3_Handle handle, uint32_t count)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->expected_tx_frames_count = count;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getPropagationTime(endat3_Handle handle, uint32_t *prop_time)
{
    if (handle == NULL || handle->endat3Interface == NULL || prop_time == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *prop_time = handle->endat3Interface->propagation_time;
    return ENDAT3_SUCCESS;
}

/* ========================================================================== */
/*                    Command and Data APIs                                   */
/* ========================================================================== */

int32_t endat3_getForegroundOpCode(endat3_Handle handle, uint32_t *opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL || opcode == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *opcode = handle->endat3Interface->foreground_op_code;
    return ENDAT3_SUCCESS;
}

int32_t endat3_setForegroundOpCode(endat3_Handle handle, uint32_t opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->foreground_op_code = opcode;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getBackgroundOpCode(endat3_Handle handle, uint32_t *opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL || opcode == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *opcode = handle->endat3Interface->background_op_code;
    return ENDAT3_SUCCESS;
}

int32_t endat3_setBackgroundOpCode(endat3_Handle handle, uint32_t opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->background_op_code = opcode;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getBgData(endat3_Handle handle, uint8_t index, uint32_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL || index >= 6)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    *data = handle->endat3Interface->bg_data[index];
    return ENDAT3_SUCCESS;
}

int32_t endat3_setBgData(endat3_Handle handle, uint8_t index, uint32_t data)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= 6)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->bg_data[index] = data;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getAllBgData(endat3_Handle handle, uint32_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    memcpy(data, handle->endat3Interface->bg_data, sizeof(handle->endat3Interface->bg_data));

    return ENDAT3_SUCCESS;
}

int32_t endat3_setAllBgData(endat3_Handle handle, const uint32_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    memcpy(handle->endat3Interface->bg_data, data, sizeof(handle->endat3Interface->bg_data));

    return ENDAT3_SUCCESS;
}

/* ========================================================================== */
/*                    Buffer Access APIs                                      */
/* ========================================================================== */

const uint8_t* endat3_getRxBuffer(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return NULL;
    }

    return handle->endat3Interface->rx_buffer;
}

const uint8_t* endat3_getTxBuffer(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return NULL;
    }

    return handle->endat3Interface->tx_buffer;
}

int32_t endat3_setTxBuffer(endat3_Handle handle, const uint8_t *data, uint32_t length)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    /* Limit to TX buffer size (24 bytes) */
    if (length > 24)
    {
        length = 24;
    }

    memcpy(handle->endat3Interface->tx_buffer, data, length);

    return (int32_t)length;
}

/* ========================================================================== */
/*                    Utility and Helper APIs                                 */
/* ========================================================================== */

int32_t endat3_getHpfFrame(endat3_Handle handle, endat3_hpf_t *hpf)
{
    if (handle == NULL || handle->endat3Interface == NULL || hpf == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    memcpy(hpf, &handle->endat3Interface->hpf, sizeof(endat3_hpf_t));

    return ENDAT3_SUCCESS;
}

int32_t endat3_getLphFrame(endat3_Handle handle, endat3_lph_t *lph)
{
    if (handle == NULL || handle->endat3Interface == NULL || lph == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    memcpy(lph, &handle->endat3Interface->lph, sizeof(endat3_lph_t));

    return ENDAT3_SUCCESS;
}

int32_t endat3_getLpfFrame(endat3_Handle handle, uint8_t index, endat3_lpf_t *lpf)
{
    if (handle == NULL || handle->endat3Interface == NULL || lpf == NULL || index >= MAX_LPF_COUNT)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    memcpy(lpf, &handle->endat3Interface->lpf[index], sizeof(endat3_lpf_t));

    return ENDAT3_SUCCESS;
}

endat3_ErrorCode_t endat3_getErrorCode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_UNKNOWN;
    }

    uint32_t error_code = 0;

    /* Check if HPFV bit is NOT set in HPF status (indicates error in HPF data) */
    if ((handle->endat3Interface->hpf.status & ENDAT3_HPF_STATUS_HPFV) == 0)
    {
        /* Extract error code from HPF data bytes 0-1 */
        error_code = (handle->endat3Interface->hpf.data[1] << 8) |
                     handle->endat3Interface->hpf.data[0];

        return (endat3_ErrorCode_t)error_code;
    }

    /* Check if BG.ERR_EXEC bit is set in LPH status */
    if ((handle->endat3Interface->lph.status & ENDAT3_LPH_BG_ERR_EXEC) == ENDAT3_LPH_BG_ERR_EXEC)
    {
        /* Extract error code from LPF data bytes 0-1 */
        error_code = (handle->endat3Interface->lpf[0].data[1] << 8) |
                     handle->endat3Interface->lpf[0].data[0];

        return (endat3_ErrorCode_t)error_code;
    }

    return ENDAT3_ERR_UNKNOWN;
}

endat3_Interface* endat3_getInterface(endat3_Handle handle)
{
    if (handle == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return NULL;
    }

    return handle->endat3Interface;
}

int32_t endat3_setOperatingMode(endat3_Handle handle, uint8_t opmode)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->opmode_config = opmode;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getOperatingMode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (int32_t)handle->endat3Interface->opmode_config;
}

int32_t endat3_releaseStartTrigger(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->start_trigger = 1;

    return ENDAT3_SUCCESS;
}

int32_t endat3_clearStartTrigger(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        if (handle != NULL) handle->last_error = ENDAT3_ERR_INVALID_HANDLE;
        return ENDAT3_ERR_INVALID_PARAM;
    }

    handle->endat3Interface->start_trigger = 0;

    return ENDAT3_SUCCESS;
}

int32_t endat3_getStartTriggerStatus(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return ENDAT3_ERR_INVALID_PARAM;
    }

    return (int32_t)handle->endat3Interface->start_trigger;
}
