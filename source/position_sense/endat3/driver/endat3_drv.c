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
#include <kernel/dpl/ClockP.h>
#include <stdint.h>
#include <stddef.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define endat3_CRC_POLY    0x1A7
#define endat3_CRC_INIT    0xFF

#define endat3_PREAMBLE_SIZE 4
#define endat3_PREAMBLE_PATTERN {0xCC, 0xCD, 0x94, 0x01}

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
/*                          Private Functions                                 */
/* ========================================================================== */

endat3_Handle endat3_open(PRUICSS_Handle icssHandle, uint32_t icssCore, uint8_t pruMode)
{
    endat3_Handle endat3Handle;

   if (pruMode == 0)
    {
        endat3Handle = &endat3Config0;
        if (icssCore==0)
        {
            endat3Handle->baseMemAddr = (uint32_t *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase);
        }
        else
        {
            endat3Handle->baseMemAddr = (uint32_t *)(((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase);
        }
    }
    else
    {
        /*
        endat3 memory map:
        RTU_PRU core:   0x0000 - 0x06FF
        PRU core:       0x0700 - 0x0DFF
        TX_PRU core:    0x0E00 - 0x1500
        */
        uint32_t DMEM_BASE_OFFSET_RTU_PRU = 0;
        uint32_t DMEM_BASE_OFFSET_PRU = 0x700;
        uint32_t DMEM_BASE_OFFSET_TX_PRU = 0xE00;
        
        // Handle Slice 1 cores
        if(icssCore == PRUICSS_RTU_PRU1)
        {
            endat3Handle = &endat3Config0;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_RTU_PRU);
        }
        else if(icssCore == PRUICSS_PRU1)
        {
            endat3Handle = &endat3Config1;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_PRU);
        }
        else if(icssCore == PRUICSS_TX_PRU1)
        {
            endat3Handle = &endat3Config2;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru1DramBase) + DMEM_BASE_OFFSET_TX_PRU);
        }
        // Handle Slice 0 cores
        else if(icssCore == PRUICSS_RTU_PRU0)
        {
            endat3Handle = &endat3Config0;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_RTU_PRU);
        }
        else if(icssCore == PRUICSS_PRU0)
        {
            endat3Handle = &endat3Config1;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_PRU);
        }
        else if(icssCore == PRUICSS_TX_PRU0)
        {
            endat3Handle = &endat3Config2;
            endat3Handle->baseMemAddr = (uint32_t *)((((PRUICSS_HwAttrs *)(icssHandle->hwAttrs))->pru0DramBase) + DMEM_BASE_OFFSET_TX_PRU);
        }
        else
        {
            endat3Handle = NULL;
        }
    }
    if (endat3Handle != NULL)
    {
        endat3Handle->icssHandle   = icssHandle;
        endat3Handle->icssCore      = icssCore;
        endat3Handle->endat3Interface = (endat3_Interface *) endat3Handle->baseMemAddr;
    }

    return endat3Handle;
}

/**
 * @brief Prepare endat3 request frames with command, data array, and CRCs
 * 
 * This function creates multiple request frames with the same command code
 * but potentially different data values. Each frame's byte order is reversed
 * after CRC calculation.
 *
 * @param priv endat3 handle
 * @param cmd Command code (same for all frames)
 * @param data_array Array of 16-bit data values, one for each frame
 * @param num_frames Number of frames to prepare (typically 4)
 * @return int32_t SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
static int32_t endat3_prepare_request(endat3_Handle priv, uint8_t cmd, uint32_t *data_array, uint8_t num_frames)
{
    uint8_t i, k;
    uint8_t temp;
    uint8_t frame_length;
    
    /* Validate input parameters - buffer overflow protection */
    if (priv == NULL || priv->endat3Interface == NULL) {
        return SystemP_FAILURE; /* Invalid handle */
    }
    
    if (num_frames * 4 > sizeof(priv->endat3Interface->tx_buffer)) {
        return SystemP_FAILURE; /* Buffer overflow protection - num_frames exceeds buffer capacity */
    }
    
    if (data_array == NULL) {
        return SystemP_FAILURE; /* Invalid data array pointer */
    }
    
    /* Process frames, each 4 bytes long */
    for (k = 0; k < num_frames; k++)
    {
        /* Create request frame */
        priv->endat3Interface->tx_buffer[k*4+0] = cmd;                     /* Command code */
        priv->endat3Interface->tx_buffer[k*4+1] = data_array[k] & 0xFF;    /* LSB of data[k] */
        priv->endat3Interface->tx_buffer[k*4+2] = (data_array[k] >> 8) & 0xFF;  /* MSB of data[k] */
        priv->endat3Interface->tx_buffer[k*4+3] = endat3_calculate_crc(priv->endat3Interface->tx_buffer + k*4, 3);
        
        /* Reverse byte order of the entire buffer (4 bytes: cmd, LSB, MSB, CRC) */
        frame_length = 4;  /* Total frame length: command(1) + data(2) + CRC(1) */
        
        /* Swap bytes from outside toward the center */
        for (i = 0; i < frame_length / 2; i++) {
            temp = priv->endat3Interface->tx_buffer[k*4 + i];
            priv->endat3Interface->tx_buffer[k*4 + i] = priv->endat3Interface->tx_buffer[k*4 + frame_length - 1 - i];
            priv->endat3Interface->tx_buffer[k*4 + frame_length - 1 - i] = temp;
        }
    }
    
    return SystemP_SUCCESS;
}


/**
 * @brief Wait for receive completion
 *
 * @param priv Private structure pointer
 *
 * @return SystemP_SUCCESS on success, negative on timeout
 */
static int32_t endat3_wait_rx_complete(endat3_Handle priv)
{
    if (priv->endat3Interface->busy==ENCODER_ERROR)    
    {
        return SystemP_FAILURE;
    }
    return SystemP_SUCCESS;
}

/**
 * @brief Parse received frames
 *
 * Extracts data, status, and CRC information from received buffer into
 * the appropriate interface structures.
 *
 * @param priv Private structure pointer
 * @param rx_buffer Receive buffer containing frame data
 */
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
    priv->endat3Interface->lpf[0].status = rx_buffer[12];  // Copy status byte

    // Copy data payload byte by byte
    for (int j = 0; j < 6; j++) 
    {
        priv->endat3Interface->lpf[0].data[j] = rx_buffer[13 + j];
    }

    priv->endat3Interface->lpf[0].crc = rx_buffer[19];  // Copy CRC byte
}


/* ========================================================================== */
/*                          Public Functions                                  */
/* ========================================================================== */
/**
 * @brief Send EnDAT3 command
 * 
 * Prepares and initiates transmission of an EnDAT3 command with the specified
 * number of frames.
 *
 * @param priv EnDAT3 handle
 * @param cmd Command code to send
 * @param frames Number of frames to transmit
 * @return int32_t 0 on success, negative on error
 */
int32_t endat3_send_command(endat3_Handle priv, uint8_t cmd, uint8_t frames) 
{
    int32_t status;
    
    /* Prepare request frame */
    status = endat3_prepare_request(priv, cmd, priv->endat3Interface->bg_data, frames);
    
    if (status != SystemP_SUCCESS) {
        return status;
    }
    
    
    return SystemP_SUCCESS;
}

int32_t endat3_receive_response(endat3_Handle priv) 
{

    /* Wait for RX complete */
    int32_t status = endat3_wait_rx_complete(priv);
    if(status < 0) {
        return status;
    }    
    /* Parse received frames */
    endat3_parse_frames(priv, priv->endat3Interface->rx_buffer);
 
    /* HPF CRC check */
   status= endat3_process_frame(priv,priv->endat3Interface->rx_buffer + 0,8);
  if (status==0) return ENDAT3_ERR_HPF_CRC_FAIL;
  
  /* LPH CRC check */
  status =  endat3_process_frame(priv,priv->endat3Interface->rx_buffer + 8,4);
  if (status==0) return ENDAT3_ERR_LPH_CRC_FAIL;

  /* check for SENDLIST 0 to calculate CRC for LPF only */
  if (priv->endat3Interface->tx_buffer[3]==0)
  {
  /* LPF CRC check for SENDLIST 0 */
  status =  endat3_process_frame(priv,priv->endat3Interface->rx_buffer + 12,8);
  }
  
  if(status==0) return SystemP_SUCCESS;  
  return 1;
}


/**
 * @brief Reflect 8-bit value (reverse bit order)
 * 
 * @param value Input value to reflect
 * @return uint8_t Bit-reflected value
 */
static uint8_t endat3_reflect8 (uint8_t value)
{
    uint8_t result = 0;
    uint8_t i;
    
    for (i = 0; i < 8; i++)
    {
        if ((value & (1 << i)) != 0)
        {
            result |= (1 << (7 - i));
        }
    }
    return result;
}

/**
 * @brief Reflect bits in a value with arbitrary width
 * 
 * @param value Input value to reflect
 * @param width Bit width of the value
 * @return uint16_t Reflected value
 */
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

/**
 * @brief Calculate CRC with custom parameters
 * 
 * @param bytes Buffer containing input data
 * @param length Length of input data in bytes
 * @param poly CRC polynomial
 * @param crcsize CRC size in bits
 * @param initialValue Initial value for CRC register
 * @param inputReflected Whether input bytes should be reflected (1/0)
 * @param resultReflected Whether final CRC result should be reflected (1/0)
 * @param finalXor Value to XOR with final CRC result
 * @return uint16_t Calculated CRC value
 */
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
        curByte = inputReflected ? endat3_reflect8 (bytes[j]) : bytes[j];
        
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

/**
 * @brief Calculate CRC-8 for endat3 protocol
 * 
 * @param data Pointer to data buffer
 * @param len Number of bytes to calculate CRC on
 * @return uint8_t Calculated CRC value
 */
uint8_t endat3_calculate_crc(uint8_t *data, uint32_t len)
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

/**
 * @brief Process endat3 frame with variable length
 * 
 * @param priv endat3 handle
 * @param buffer Data buffer
 * @param length Length of buffer
 * @return uint8_t 1 if CRC check passed, 0 otherwise
 */
uint8_t endat3_process_frame(endat3_Handle priv, uint8_t *buffer, uint32_t length)
{
    uint8_t received_crc;
    uint8_t calculated_crc;
    
    if (length <= 1) {
        return SystemP_SUCCESS; /* Using 0 instead of 0 */
    }
    
    received_crc = buffer[length - 1];
    calculated_crc = endat3_calculate_crc(buffer, length - 1);
    
    return (received_crc == calculated_crc) ? 1 : 0; /* Using 1/0 instead of 1/0 */
}

/**
 * @brief Configure EnDAT mode in PRU configuration registers
 * 
 * @param priv endat3 handle
 * @param pruicss_slicex PRU slice selection (0 or 1)
 */
void endat3_config_endat_mode(endat3_Handle priv, uint8_t pruicss_slicex)
{
    void *pruicss_cfg = priv->pruicss_cfg;
    if(pruicss_slicex)
    {
       HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG1_REG + 3, 4);
    }
    else
    {
       HW_WR_REG8((uint8_t *)pruicss_cfg + CSL_ICSS_PR1_CFG_SLV_GPCFG0_REG + 3, 4);
    }
}
void endat3_enable_load_share_mode(void *pruCfg, uint32_t pruSlice)
{
    uint32_t regVal;
    if(pruSlice == 1)
    {
        regVal = HW_RD_REG32((uint8_t *)pruCfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_MASK);
        regVal |= CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruCfg + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, regVal);
    }
    else
    {
        regVal = HW_RD_REG32((uint8_t *)pruCfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_MASK);
        regVal |= CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruCfg + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, regVal);
    }
}


/**
 * \brief Initialize EnDat3 configuration with default values
 *
 * \param config Pointer to EnDat3 configuration structure
 * \param load_share_mode Load share mode flag (0 or 1)
 *
 * \return void
 */
void endat3_initConfig(endat3_Config_t *config, uint32_t load_share_mode)
{
    if (config == NULL) {
        return;
    }
    
    /* Initialize clock configuration just */
    config->clock_config.div_factor_normal = ENDAT3_DIV_FACTOR_NORMAL;
    config->clock_config.div_factor_oversampled = ENDAT3_DIV_FACTOR_OVERSAMPLED;
    config->clock_config.pru_clock_type = ENDAT3_PRU_CLOCK_TYPE;
    config->clock_config.uart_clock_type = ENDAT3_UART_CLOCK_TYPE;
    config->clock_config.clock_type = ENDAT3_PRU_CLOCK_TYPE;
    config->clock_config.oversample_rate = ENDAT3_OVERSAMPLE_RATE;
    config->clock_config.sb_polarity= DEFAULT_SB_POLARITY;
    
    /* Initialize register configuration */
    config->reg_config.endat3_enable = ENDAT3_ENABLE_BIT;
    config->reg_config.counter_enable = ENDAT3_CTR_EN;
    config->reg_config.load_share_mode = load_share_mode;
    
    /* Compute TX and RX configurations */
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

/**
 * \brief Configure EnDat3 PRU registers using configuration structure
 *
 * This function configures the necessary PRU-ICSS registers for EnDat3 operation
 * using values from the provided configuration structure.
 *
 * \param pru_cfg_base Pointer to PRU configuration register base address
 * \param config Pointer to EnDat3 configuration structure
 *
 * \return void
 */
void endat3_configurePruRegisters(void *pru_cfg_base, const endat3_Config_t *config, uint32_t pruSlice)
{
    if (pru_cfg_base == NULL || config == NULL) {
        return;
    }
    if(pruSlice == 1)
    {
    /* Enable EnDat3 functionality in GPCFG1 register */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_GPCFG1, config->reg_config.endat3_enable);
    
    /* Configure PRU1 TX parameters */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_EDPRU1TXCFGREGISTER, config->reg_config.tx_config);
    
    /* Configure PRU1 RX parameters */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_EDPRU1RXCFGREGISTER, config->reg_config.rx_config);
    }
    else
    {
        /* Enable EnDat3 functionality in GPCFG0 register */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_GPCFG0, config->reg_config.endat3_enable);
    
    /* Configure PRU0 TX parameters */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_EDPRU0TXCFGREGISTER, config->reg_config.tx_config);
    
    /* Configure PRU0 RX parameters */
    HW_WR_REG32(pru_cfg_base + CSL_ICSSCFG_EDPRU0RXCFGREGISTER, config->reg_config.rx_config);
    }
}

/**
 * \brief Create EnDat3 configuration and configure PRU registers (convenience function)
 *
 * This is a convenience function that initializes the configuration structure
 * with default values and configures the PRU registers in one call.
 *
 * \param pru_cfg_base Pointer to PRU configuration register base address
 * \param load_share_mode Load share mode flag (0 or 1)
 *
 * \return void
 */
void endat3_configureWithDefaults(void *pru_cfg_base, uint32_t load_share_mode, uint32_t pruSlice)
{
    endat3_Config_t config;

    /* Initialize configuration with default values */
    endat3_initConfig(&config, load_share_mode);

    /* Configure PRU registers */
    endat3_configurePruRegisters(pru_cfg_base, &config, pruSlice);
}

int32_t endat3_setDelayCycles(endat3_Handle handle, uint64_t pru_freq_hz)
{
    if (handle == NULL || handle->endat3Interface == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Reference values from endat3_params.h at 300MHz PRU frequency:
     * ENDAT3_TX_START_DELAY_1 = 0xFFFF = 65535 cycles
     * ENDAT3_TX_START_DELAY_2 = 0x80000 = 524288 cycles
     * ENDAT3_TX_START_DELAY_3 = 0x20100 = 131328 cycles
     * SAMPLING_DELAY_COUNT = 37 cycles
     * DELAY_10MS = 0x300000 = 3145728 cycles (10ms at 300MHz)
     */

    #define REFERENCE_PRU_FREQ_HZ 300000000ULL  /* 300 MHz reference frequency */

    /* Scale delay values based on actual PRU frequency relative to 300MHz reference */
    /* Formula: actual_cycles = (reference_cycles * actual_freq) / reference_freq */

    handle->endat3Interface->delay_tx_start_1 = (uint32_t)((65535ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    handle->endat3Interface->delay_tx_start_2 = (uint32_t)((524288ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    handle->endat3Interface->delay_tx_start_3 = (uint32_t)((131328ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    /* delay_sampling: Minimum 37 cycles (encoder-dependent, keep conservative) */
    handle->endat3Interface->delay_sampling = 37;
    /* delay_10ms: Scale from 3145728 cycles at 300MHz */
    handle->endat3Interface->delay_10ms = (uint32_t)((3145728ULL * pru_freq_hz) / REFERENCE_PRU_FREQ_HZ);
    return SystemP_SUCCESS;
}

void endat3_handle_background_command_request(endat3_Handle endat3Handle, int index, int frame_cnt, uint8_t op_code, uint32_t addr_msb, uint32_t addr_lsb, uint32_t data)
{
    switch (op_code) {
        case endat3_BGREQ_NOP:
            // NOP operation: Just need to send arbitrary data
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFF;          // Arbitrary data
            endat3Handle->endat3Interface->bg_data[index+1] = (data >> 8) & 0xFF;   // Arbitrary data
            endat3Handle->endat3Interface->bg_data[index+2] = endat3_BGREQ_NOP;     // OpCode
            break;

        case endat3_BGREQ_READ:
            // READ operation: addr_msb = address[23:16], addr_lsb = address[15:0], data = num_words
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFF;         // num_words
            endat3Handle->endat3Interface->bg_data[index+1] = addr_lsb & 0xFFFF;   // address[15:0]
            endat3Handle->endat3Interface->bg_data[index+2] = (addr_msb & 0xFF) | (endat3_BGREQ_READ << 8);  // OpCode + address[23:16]
            break;

        case endat3_BGREQ_WRITE:
            // WRITE operation: addr_msb = address[23:16], addr_lsb = address[15:0], data = value to write
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFFFF;       // Word to write
            endat3Handle->endat3Interface->bg_data[index+1] = addr_lsb & 0xFFFF;   // address[15:0]
            endat3Handle->endat3Interface->bg_data[index+2] = (addr_msb & 0xFF) | (endat3_BGREQ_WRITE << 8);  // OpCode + address[23:16]
            break;

        case endat3_BGREQ_RECONFIGURE:
            // RECONFIGURE operation: Simply sends the reconfigure command
            endat3Handle->endat3Interface->bg_data[index+0] = 0x0000;              // Reserved (0)
            endat3Handle->endat3Interface->bg_data[index+1] = 0x0000;              // Reserved (0)
            endat3Handle->endat3Interface->bg_data[index+2] = endat3_BGREQ_RECONFIGURE << 8;  // OpCode
            break;

        case endat3_BGREQ_AUTH:
            // AUTH operation: addr_msb = usrlevel, addr_lsb/data = password
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFFFF;       // Password part 2
            endat3Handle->endat3Interface->bg_data[index+1] = addr_lsb & 0xFFFF;   // Password part 1
            endat3Handle->endat3Interface->bg_data[index+2] = (addr_msb & 0xFF) | (endat3_BGREQ_AUTH << 8);  // OpCode + User level
            break;

        case endat3_BGREQ_PROTECT:
        {
            // PROTECT operation with 2-byte bg_data entries:
            // bg_data[index+0]: Bytes 0-1 (acclevel, mode)
            // bg_data[index+1]: Bytes 2-3 (address[15:0])
            // bg_data[index+2]: Bytes 4-5 (address[23:16], OpCode=0x81)

            uint8_t mode = (data >> 24) & 0xFF;
            uint8_t acclevel = (data >> 16) & 0xFF;
            const char* acclevelDesc;

            // Pack the first 2 bytes: acclevel (byte 0) and mode (byte 1)
            endat3Handle->endat3Interface->bg_data[index+0] = (mode << 8) | acclevel;

            // Pack the middle 2 bytes: address[15:0] (bytes 2-3)
            endat3Handle->endat3Interface->bg_data[index+1] = addr_lsb & 0xFFFF;

            // Pack the last 2 bytes: address[23:16] (byte 4) and OpCode=0x81 (byte 5)
            endat3Handle->endat3Interface->bg_data[index+2] = (0x81 << 8) | (addr_msb & 0xFF);

            // Get description for access level
            switch(acclevel) {
                case 0: acclevelDesc = "USER"; break;
                case 1: acclevelDesc = "OEM2"; break;
                case 2: acclevelDesc = "OEM1"; break;
                case 3: acclevelDesc = "MANUFACTURER"; break;
                default: acclevelDesc = "Unknown"; break;
            }

            // Display mode information with access level descriptions
            switch(mode) {
                case 1:
                    DebugP_log("\r\n PROTECT Mode: QUERY (0x01) - Querying current access levels");
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                case 2:
                    DebugP_log("\r\n PROTECT Mode: SET_READ (0x02) - Setting read access level");
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                case 3:
                    DebugP_log("\r\n PROTECT Mode: SET_WRITE (0x03) - Setting write access level");
                    DebugP_log("\r\n - Address: 0x%02X%04X", addr_msb, addr_lsb);
                    DebugP_log("\r\n - Access Level: %d (%s)", acclevel, acclevelDesc);
                    break;
                default:
                    DebugP_log("\r\n PROTECT Mode: Unknown (0x%02X) - This will be rejected by the encoder", mode);
                    break;
            }
            break;
        }

        case endat3_BGREQ_SETPASS:
            // SETPASS operation: addr_msb = usrlevel, addr_lsb/data = password
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFFFF;       // Password part 2
            endat3Handle->endat3Interface->bg_data[index+1] = addr_lsb & 0xFFFF;   // Password part 1
            endat3Handle->endat3Interface->bg_data[index+2] = (addr_msb & 0xFF) | (endat3_BGREQ_SETPASS << 8);  // OpCode + User level
            break;

        case endat3_BGREQ_LOCATE:
            // LOCATE operation: addr_msb = not used, addr_lsb = not used, data = ctrl
            endat3Handle->endat3Interface->bg_data[index+0] = data & 0xFF;         // ctrl
            endat3Handle->endat3Interface->bg_data[index+1] = 0x0000;              // Reserved (0)
            endat3Handle->endat3Interface->bg_data[index+2] = endat3_BGREQ_LOCATE << 8;  // OpCode
            break;

        default:
            // Invalid operation code, use NOP
            DebugP_log("\r\n Invalid background request operation code: 0x%02X", op_code);
            endat3Handle->endat3Interface->bg_data[index+0] = 0x0000;              // Arbitrary data
            endat3Handle->endat3Interface->bg_data[index+1] = 0x0000;              // Arbitrary data
            endat3Handle->endat3Interface->bg_data[index+2] = endat3_BGREQ_NOP << 8;  // OpCode
            break;
    }
    // BG_DATAT=0 for last frame in which we receive BG response in LPFs
    endat3Handle->endat3Interface->bg_data[index+3]=0x0;
    // Set the frame count for all operations
    endat3Handle->endat3Interface->expected_tx_frames_count = frame_cnt;
}

/**
 * @brief Get string description for EnDat3 error code
 *
 * @param error_code The error code to get description for
 * @return const char* String description of the error code
 */
const char* endat3_getErrorDescription(endat3_ErrorCode_t error_code)
{
    switch(error_code) {
        case endat3_ERR_UNKNOWN:
            return "Unknown error cause";
        case endat3_FGERR_RECONFIGURE:
            return "Device in configuration due to RECONFIGURE";
        case endat3_FGERR_ECHO:
            return "ECHO response";
        case endat3_FGERR_INVALID_FID:
            return "Invalid FID configured";
        case endat3_FGERR_DUPLICATE_FID:
            return "FID selected multiple times in cycle";
        case endat3_FGERR_INVALID_DATA:
            return "Invalid data delivered internally";
        case endat3_FGERR_INT_TRM:
            return "LPF supported but unavailable (value not formed in time)";
        case endat3_FGERR_NO_SENSOR_DATA:
            return "Sensor box data not available";
        case endat3_BGERR_USAGE:
            return "Generic operator error";
        case endat3_BGERR_USAGE_OPCODE:
            return "Invalid or unsupported command code";
        case endat3_BGERR_USAGE_ARGUMENTS:
            return "Invalid arguments";
        case endat3_BGERR_USAGE_SEQUENCE:
            return "Invalid command sequence";
        case endat3_BGERR_USAGE_ACCESS_DENIED:
            return "Access denied; insufficient user level";
        case endat3_BGERR_USAGE_MEM_ADDRESS:
            return "Access to invalid address";
        case endat3_BGERR_USAGE_NO_BG:
            return "Encoder doesn't support background processing";
        case endat3_BGERR_INTERNAL:
            return "Generic exception error in encoder";
        case endat3_BGERR_INTERNAL_MEMORY:
            return "Exception error when accessing memory";
        case endat3_BGERR_INTERNAL_CONFIG:
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

/**
 * @brief Get recommended action for EnDat3 error code
 *
 * @param error_code The error code to get recommended action for
 * @return const char* String with recommended action
 */
const char* endat3_getErrorAction(endat3_ErrorCode_t error_code)
{
    switch(error_code) {
        case endat3_ERR_UNKNOWN:
            return "Try again";
        case endat3_FGERR_RECONFIGURE:
            return "Try again after configuration completes";
        case endat3_FGERR_ECHO:
            return "No action needed";
        case endat3_FGERR_INVALID_FID:
        case endat3_FGERR_DUPLICATE_FID:
        case endat3_FGERR_NO_SENSOR_DATA:
        case endat3_BGERR_USAGE:
        case endat3_BGERR_USAGE_OPCODE:
        case endat3_BGERR_USAGE_ARGUMENTS:
        case endat3_BGERR_USAGE_SEQUENCE:
        case endat3_BGERR_USAGE_MEM_ADDRESS:
        case endat3_BGERR_USAGE_NO_BG:
            return "Correct the application";
        case endat3_FGERR_INVALID_DATA:
        case endat3_FGERR_INT_TRM:
        case endat3_BGERR_INTERNAL:
        case endat3_BGERR_INTERNAL_MEMORY:
        case endat3_BGERR_INTERNAL_CONFIG:
            return "Try again";
        case endat3_BGERR_USAGE_ACCESS_DENIED:
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

uint8_t endat3_getHpfStatus(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->hpf.status;
}

int32_t endat3_getHpfData(endat3_Handle handle, uint8_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL) {
        return SystemP_FAILURE;
    }
    
    /* Manual copy instead of memcpy - more explicit and returns bytes copied */
    for (int i = 0; i < 6; i++) {
        data[i] = handle->endat3Interface->hpf.data[i];
    }
    
    return 6; /* Return number of bytes copied */
}

uint8_t endat3_getHpfCrc(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->hpf.crc;
}

uint64_t endat3_getHpfDataAsU64(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    
    uint64_t result = 0;
    uint8_t *data = handle->endat3Interface->hpf.data;
    
    /* Pack 6 bytes into uint64_t (little-endian) */
    for (int i = 0; i < 6; i++) {
        result |= ((uint64_t)data[i]) << (i * 8);
    }
    
    return result;
}

uint8_t endat3_isHpfDataValid(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->hpf.status & endat3_HPF_STATUS_HPFV) != 0;
}

uint8_t endat3_hasHpfError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->hpf.status & endat3_HPF_STATUS_F) != 0;
}

uint8_t endat3_hasHpfWarning(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->hpf.status & endat3_HPF_STATUS_W) != 0;
}

uint8_t endat3_hasAbsoluteValue(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->hpf.status & endat3_HPF_STATUS_RM) != 0;
}

/* ========================================================================== */
/*                    LPH (Low Priority Header) Access APIs                   */
/* ========================================================================== */

uint8_t endat3_getLphStatus(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->lph.status;
}

uint8_t endat3_getLphnum_lpf (endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->lph.num_lpf ;
}

uint8_t endat3_getLphCrc(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->lph.crc;
}

LPH_Status_t endat3_getLphState(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return LPH_STATUS_IDLE;
    }
    return (LPH_Status_t)(handle->endat3Interface->lph.status & endat3_LPH_STATE_MASK);
}

uint8_t endat3_hasBgError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->lph.status & endat3_LPH_BG_ERR_EXEC) != 0;
}

uint8_t endat3_isBgBusy(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->lph.status & endat3_LPH_BG_BUSY) != 0;
}

uint8_t endat3_hasBgRtxError(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return (handle->endat3Interface->lph.status & endat3_LPH_BG_RTX_ERROR) != 0;
}

/* ========================================================================== */
/*                    LPF (Low Priority Frame) Access APIs                    */
/* ========================================================================== */

uint8_t endat3_getLpfStatus(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->lpf[index].status;
}

int32_t endat3_getLpfData(endat3_Handle handle, uint8_t index, uint8_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL || index >= MAX_LPF_COUNT) {
        return SystemP_FAILURE;
    }
    
    /* Manual copy instead of memcpy - more explicit and returns bytes copied */
    for (int i = 0; i < 6; i++) {
        data[i] = handle->endat3Interface->lpf[index].data[i];
    }
    
    return 6; /* Return number of bytes copied */
}

uint8_t endat3_getLpfCrc(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->lpf[index].crc;
}

uint8_t endat3_getLpfFid(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= MAX_LPF_COUNT) {
        return SystemP_SUCCESS;
    }
    /* FID is typically in the status byte */
    return handle->endat3Interface->lpf[index].status;
}

/* ========================================================================== */
/*                    Communication Control APIs                              */
/* ========================================================================== */

uint8_t endat3_isConnected(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    return handle->endat3Interface->connected != 0;
}

uint8_t endat3_isBusy(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return 0;
    }
    if ((handle->endat3Interface->busy)==ENCODER_ERROR) return ENCODER_IDLE;
    return handle->endat3Interface->busy != 0;
}

int32_t endat3_setBusy(endat3_Handle handle, uint8_t busy)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->busy = busy ? 1 : 0;
    return SystemP_SUCCESS;
}

uint32_t endat3_getExpectedTxFrameCount(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->expected_tx_frames_count;
}

int32_t endat3_setExpectedTxFrameCount(endat3_Handle handle, uint32_t count)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->expected_tx_frames_count = count;
    return SystemP_SUCCESS;
}

uint32_t endat3_getPropagationTime(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->propagation_time;
}

/* ========================================================================== */
/*                    Command and Data APIs                                   */
/* ========================================================================== */

uint32_t endat3_getForegroundOpCode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->foreground_op_code;
}

int32_t endat3_setForegroundOpCode(endat3_Handle handle, uint32_t opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->foreground_op_code = opcode;
    return SystemP_SUCCESS;
}

uint32_t endat3_getBackgroundOpCode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->background_op_code;
}

int32_t endat3_setBackgroundOpCode(endat3_Handle handle, uint32_t opcode)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->background_op_code = opcode;
    return SystemP_SUCCESS;
}

uint32_t endat3_getBgData(endat3_Handle handle, uint8_t index)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= 6) {
        return SystemP_SUCCESS;
    }
    return handle->endat3Interface->bg_data[index];
}

int32_t endat3_setBgData(endat3_Handle handle, uint8_t index, uint32_t data)
{
    if (handle == NULL || handle->endat3Interface == NULL || index >= 6) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->bg_data[index] = data;
    return SystemP_SUCCESS;
}

int32_t endat3_getAllBgData(endat3_Handle handle, uint32_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL) {
        return SystemP_FAILURE;
    }
    memcpy(data, handle->endat3Interface->bg_data, sizeof(uint32_t) * 6);
    return SystemP_SUCCESS;
}

int32_t endat3_setAllBgData(endat3_Handle handle, const uint32_t *data)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL) {
        return SystemP_FAILURE;
    }
    memcpy(handle->endat3Interface->bg_data, data, sizeof(uint32_t) * 6);
    return SystemP_SUCCESS;
}

/* ========================================================================== */
/*                    Buffer Access APIs                                      */
/* ========================================================================== */

const uint8_t* endat3_getRxBuffer(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return NULL;
    }
    return handle->endat3Interface->rx_buffer;
}

const uint8_t* endat3_getTxBuffer(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return NULL;
    }
    return handle->endat3Interface->tx_buffer;
}

int32_t endat3_setTxBuffer(endat3_Handle handle, const uint8_t *data, uint32_t length)
{
    if (handle == NULL || handle->endat3Interface == NULL || data == NULL) {
        return SystemP_FAILURE;
    }
    
    /* Limit to TX buffer size (24 bytes) */
    if (length > 24) {
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
    if (handle == NULL || handle->endat3Interface == NULL || hpf == NULL) {
        return SystemP_FAILURE;
    }
    memcpy(hpf, &handle->endat3Interface->hpf, sizeof(endat3_hpf_t));
    return SystemP_SUCCESS;
}

int32_t endat3_getLphFrame(endat3_Handle handle, endat3_lph_t *lph)
{
    if (handle == NULL || handle->endat3Interface == NULL || lph == NULL) {
        return SystemP_FAILURE;
    }
    memcpy(lph, &handle->endat3Interface->lph, sizeof(endat3_lph_t));
    return SystemP_SUCCESS;
}

int32_t endat3_getLpfFrame(endat3_Handle handle, uint8_t index, endat3_lpf_t *lpf)
{
    if (handle == NULL || handle->endat3Interface == NULL || lpf == NULL || index >= MAX_LPF_COUNT) {
        return SystemP_FAILURE;
    }
    memcpy(lpf, &handle->endat3Interface->lpf[index], sizeof(endat3_lpf_t));
    return SystemP_SUCCESS;
}

endat3_ErrorCode_t endat3_getErrorCode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return endat3_ERR_UNKNOWN;
    }
    
    uint32_t error_code = 0;
    
    /* Check if HPFV bit is NOT set in HPF status (indicates error in HPF data) */
    if ((handle->endat3Interface->hpf.status & endat3_HPF_STATUS_HPFV) == 0) {
        /* Extract error code from HPF data bytes 0-1 */
        error_code = (handle->endat3Interface->hpf.data[1] << 8) |
                     handle->endat3Interface->hpf.data[0];
        return (endat3_ErrorCode_t)error_code;
    }
    
    /* Check if BG.ERR_EXEC bit is set in LPH status */
    if ((handle->endat3Interface->lph.status & endat3_LPH_BG_ERR_EXEC) == endat3_LPH_BG_ERR_EXEC) {
        /* Extract error code from LPF data bytes 0-1 */
        error_code = (handle->endat3Interface->lpf[0].data[1] << 8) |
                     handle->endat3Interface->lpf[0].data[0];
        return (endat3_ErrorCode_t)error_code;
    }
    
    return endat3_ERR_UNKNOWN;
}

endat3_Interface* endat3_getInterface(endat3_Handle handle)
{
    if (handle == NULL) {
        return NULL;
    }
    return handle->endat3Interface;
}

int32_t endat3_setOperatingMode(endat3_Handle handle, uint8_t opmode)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->opmode_config = opmode;
    return SystemP_SUCCESS;
}

int32_t endat3_getOperatingMode(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    return (int32_t)handle->endat3Interface->opmode_config;
}

int32_t endat3_releaseStartTrigger(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->start_trigger = 1;
    return SystemP_SUCCESS;
}

int32_t endat3_clearStartTrigger(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    handle->endat3Interface->start_trigger = 0;
    return SystemP_SUCCESS;
}

int32_t endat3_getStartTriggerStatus(endat3_Handle handle)
{
    if (handle == NULL || handle->endat3Interface == NULL) {
        return SystemP_FAILURE;
    }
    return (int32_t)handle->endat3Interface->start_trigger;
}
