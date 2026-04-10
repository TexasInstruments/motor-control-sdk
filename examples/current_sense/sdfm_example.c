/*
 *  Copyright (C) 2023-2025 Texas Instruments Incorporated
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

/**
 *  \file   sdfm_example.c
 *
 *  \brief  SDFM example initialization and configuration functions.
 *
 *  \details
 *  This file provides helper functions for SDFM (Sigma-Delta Filter Module)
 *  initialization on ICSSG PRU cores for current sensing applications.
 *
 *  Architecture:
 *  - PRU firmware runs real-time SDFM filtering on PRU-ICSS cores
 *  - ARM R5F manages initialization, configuration, and data processing
 *  - Supports single PRU mode (9 channels on one core) or load-share mode
 *    (3 channels each on RTU_PRU, PRU, and TX_PRU cores)
 *
 *  Key Functions:
 *  - sdfmPruicssInit()           : Initialize ICSSG subsystem and PRU cores
 *  - sdfmLoadFirmware()          : Load PRU firmware into cores
 *  - sdfmConfigureAndEnable()    : Configure SDFM parameters and enable firmware
 *  - sdfmConfigGpioPins()        : Internal GPIO setup for zero-cross detection (static helper)
 *
 *  Firmware Loading:
 *  - Single PRU mode: Loads SDFM_PRU0/1_image_0 to PRU core
 *  - Load-share mode: Loads separate firmware to RTU_PRU, PRU, and TX_PRU
 *  - Firmware binaries are statically linked from firmware/ directory
 *
 */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <stdint.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/pruicss.h>
#include <drivers/sciclient.h>

#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_txpru1_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru1_bin.h"            /* SDFM image data */
#else
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_pru0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_rtu0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/multi_axis_load_share/sdfm_txpru0_bin.h"            /* SDFM image data */
#include "current_sense/sdfm/firmware/single_axis_single_pru/sdfm_pru0_bin.h"            /* SDFM image data */
#endif

#include "sdfm_example.h"
#include "current_sense/sdfm/include/sdfm_api.h"

#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
#define SDFM_PRU_CORE         PRUICSS_PRU1
#define SDFM_RTU_CORE         PRUICSS_RTU_PRU1
#define SDFM_TXPRU_CORE       PRUICSS_TX_PRU1
#else
#define SDFM_PRU_CORE         PRUICSS_PRU0
#define SDFM_RTU_CORE         PRUICSS_RTU_PRU0
#define SDFM_TXPRU_CORE       PRUICSS_TX_PRU0
#endif

/* Number of PRU images */
#define PRU_SDFM_NUM_PRU_IMAGE  ( 4 )

/* ICSS INTC configuration */
#if CONFIG_SDFM0_ICSSGx == 1
/* These variables are defined in the generated SysCfg code */
extern PRUICSS_IntcInitData icss1_intc_initdata;
#else
extern PRUICSS_IntcInitData icss0_intc_initdata;
#endif

/* PRU SDFM FW image info */
typedef struct PRUSDFM_PruFwImageInfo_s
{
    const uint32_t *pPruImemImg;
    const uint32_t pruImemImgSz;
} PRUSDFM_PruFwImageInfo;

/* PRU SDFM image info */
static PRUSDFM_PruFwImageInfo gPruFwImageInfo[PRU_SDFM_NUM_PRU_IMAGE] =
{
#if CONFIG_SDFM0_SLICE == PRUICSS_PRU1
    {SDFM_PRU1_image_0, sizeof(SDFM_PRU1_image_0)}, /* single PRU FW binary */
    {pru_SDFM_PRU1_image_0, sizeof(pru_SDFM_PRU1_image_0)}, /* load share PRU FW binary */
    {pru_SDFM_RTU1_image_0, sizeof(pru_SDFM_RTU1_image_0)}, /*load share RTU FW binary */
    {pru_SDFM_TXPRU1_image_0, sizeof(pru_SDFM_TXPRU1_image_0)} /*load share TXPRU binary*/
#else
    {SDFM_PRU0_image_0, sizeof(SDFM_PRU0_image_0)}, /* single PRU FW binary */
    {pru_SDFM_PRU0_image_0, sizeof(pru_SDFM_PRU0_image_0)}, /* load share PRU FW binary */
    {pru_SDFM_RTU0_image_0, sizeof(pru_SDFM_RTU0_image_0)}, /*load share RTU FW binary */
    {pru_SDFM_TXPRU0_image_0, sizeof(pru_SDFM_TXPRU0_image_0)} /*load share TXPRU binary*/
#endif
};

int32_t sdfmPruicssInit(PRUICSS_Handle *pruicss_handle, uint8_t pruicss_instance, uint8_t pruicss_slice, uint8_t load_share_enabled)
{
    PRUICSS_Handle pru_icss_handle;
    int32_t size;
    int32_t status;

    /* Open ICSS PRU instance */
    pru_icss_handle = PRUICSS_open(pruicss_instance);
    if (pru_icss_handle == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Disable slice PRU cores */
    if (pruicss_slice == PRUICSS_PRU0)
    {
        status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_PRU0);
        if (status != SystemP_SUCCESS)
        {
            return status;
        }

        if(load_share_enabled)
        {
            status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_RTU_PRU0);
            if (status != SystemP_SUCCESS)
            {
                return status;
            }

            status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_TX_PRU0);
            if (status != SystemP_SUCCESS)
            {
                return status;
            }

        }
    }
    else if (pruicss_slice == PRUICSS_PRU1)
    {
        status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_PRU1);
        if (status != SystemP_SUCCESS)
        {
            return status;
        }

        if(load_share_enabled)
        {
            status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_RTU_PRU1);
            if (status != SystemP_SUCCESS)
            {
                return status;
            }

            status = PRUICSS_disableCore(pru_icss_handle, PRUICSS_TX_PRU1);
            if (status != SystemP_SUCCESS)
            {
                return status;
            }

        }
    }
    else
    {
        return SystemP_FAILURE;
    }

    /* Reset slice memories */
    size = PRUICSS_initMemory(pru_icss_handle, PRUICSS_IRAM_PRU(pruicss_slice));
    if (size == 0)
    {
        return SystemP_FAILURE;
    }
    if(load_share_enabled)
    {
        size = PRUICSS_initMemory(pru_icss_handle, PRUICSS_IRAM_RTU_PRU(pruicss_slice));
        if (size == 0)
        {
            return SystemP_FAILURE;
        }
        size = PRUICSS_initMemory(pru_icss_handle, PRUICSS_IRAM_TX_PRU(pruicss_slice));
        if (size == 0)
        {
            return SystemP_FAILURE;
        }
    }
    size = PRUICSS_initMemory(pru_icss_handle, PRUICSS_DATARAM(pruicss_slice));
    if (size == 0)
    {
        return SystemP_FAILURE;
    }

    /* Set ICSS pin mux */
#ifdef CONFIG_SDFM0_G_MUX_EN
    status = PRUICSS_setSaMuxMode(pru_icss_handle, PRUICSS_SA_MUX_MODE_SD_ENDAT);
    if (status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }
#endif
    /* Initialize ICSS INTC */
#if CONFIG_SDFM0_ICSSGx == 1
    status = PRUICSS_intcInit(pru_icss_handle, &icss1_intc_initdata);
#else
    status = PRUICSS_intcInit(pru_icss_handle, &icss0_intc_initdata);
#endif
    if (status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    *pruicss_handle = pru_icss_handle;

    return SystemP_SUCCESS;
}
/*
 *  ======== SDFM_configGpioPins ========
 *  Internal helper function to configure GPIO pins for zero-cross detection.
 *  Called from initSdfmFw() for each enabled channel with zero-cross enabled.
 */
static void sdfmConfigGpioPins(SDFM_Handle handle, uint8_t channel)
{
    switch(channel)
    {
        case 0:
#if (CONFIG_SDFM0_CHANNEL0_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH0_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH0_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH0_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 1:
#if (CONFIG_SDFM0_CHANNEL1_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH1_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH1_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH1_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 2:
#if (CONFIG_SDFM0_CHANNEL2_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH2_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH2_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH2_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 3:
#if (CONFIG_SDFM0_CHANNEL3_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH3_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH3_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH3_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 4:
#if (CONFIG_SDFM0_CHANNEL4_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH4_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH4_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH4_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 5:
#if (CONFIG_SDFM0_CHANNEL5_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH5_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH5_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH5_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 6:
#if (CONFIG_SDFM0_CHANNEL6_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH6_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH6_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH6_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 7:
#if (CONFIG_SDFM0_CHANNEL7_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH7_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH7_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH7_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        case 8:
#if (CONFIG_SDFM0_CHANNEL8_OC_EN_ZERO_CROSS != 0)
            {
                uint32_t gpio_base_addr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_ZC_TH_CH8_BASE_ADDR);
                uint32_t pin_num = GPIO_ZC_TH_CH8_PIN;
                GPIO_setDirMode(gpio_base_addr, pin_num, GPIO_ZC_TH_CH8_DIR);
                SDFM_configComparatorGpioPins(handle, channel, gpio_base_addr, pin_num);
            }
#endif
            break;
        default:
            break;
    }
}

/**
 *  \brief Configure SDFM parameters and enable SDFM firmware
 *
 *  This function performs comprehensive SDFM configuration including:
 *  - Enabling all configured SDFM channels
 *  - Setting up sample output buffer address translation (TCM to SoC global view)
 *  - Configuring IEP count for EPWM synchronization
 *  - Configuring operation modes (snoop/trigger) for each PRU core
 *  - Setting up trigger timing and double sampling if enabled
 *  - Enabling EPWM synchronization or configuring IEP CMP0
 *  - Configuring filter parameters for all enabled channels:
 *    - Overcurrent comparator filter OSR
 *    - Normal current filter OSR
 *    - Data filter type (SINC3/SINC2/SINC1)
 *    - Clock source and inversion
 *  - Configuring overcurrent comparator and thresholds
 *  - Configuring fast detect for quick overcurrent detection
 *  - Configuring zero-cross detection and GPIO output (including phase delay measurement)
 *  - Enabling SDFM firmware on all configured PRU cores
 *
 *  \param[in]  handle  SDFM handle
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 *
 *  \note IMPORTANT: SDFM_enable() is called at the end of this function after all
 *        configuration is complete. Before calling SDFM_enable(), all configuration
 *        must be done, including threshold values, memory addresses,comparator
 *        events, filter parameters, etc. Once SDFM_enable() is executed, the firmware
 *        starts sampling immediately using the configured parameters. Therefore, ensure
 *        all configuration is properly set up before calling SDFM_enable().
 */
int32_t sdfmConfigureAndEnable(SDFM_Handle handle)
{
    const SDFM_Attrs *attrs;
    SDFM_Priv *priv;
    uint8_t channel;
    uint32_t i;
    uint32_t local_addr, global_addr;
    int32_t status;
    SDFM_ThresholdConfig threshold_config;
    SDFM_FastDetectConfig fast_detect_config;

    /* Get attrs and priv using accessor functions */
    attrs = SDFM_getAttrs(handle);
    priv = SDFM_getPriv(handle);

    if((handle == NULL) || (attrs == NULL) || (priv == NULL))
    {
        DebugP_log("\r\n ERROR: NULL handle/attrs/priv");
        return SystemP_FAILURE;
    }

    /* Enable all configured SDFM channels */
    for(i = 0; i < SDFM_NUM_OF_CH_PER_PRU_SLICE; i++)
    {
        if(attrs->channel_mask & (1 << i))
        {
            status = SDFM_setEnableChannel(handle, i);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }
        }
    }

    /*
     * Configure sample output buffer address translation (TCM local to SoC global view)
     *
     * The sample output buffer is allocated in R5F TCM (Tightly Coupled Memory):
     * - R5F uses core-local view address to access the buffer directly
     * - PRU firmware uses SoC global view address to write samples via ICSSG memory interface
     *
     * Address Translation Requirements:
     * - CPU0_BTCM_SOCVIEW: Used for R5FSS0_CORE0 (default for r5fss0-0_freertos)
     * - CPU1_BTCM_SOCVIEW: Required if running on R5FSS1_CORE0 (r5fss1-0_freertos)
     * - CPU0_ATCM_SOCVIEW: Required if buffer allocated in ATCM instead of BTCM
     *
     * The macro translates local TCM address (0x00000000-0x0007FFFF) to SoC view:
     * - R5FSS0 BTCM: 0x70000000-0x7007FFFF
     * - R5FSS1 BTCM: 0x70100000-0x7017FFFF
     *
     * \note Update the address translation macro if using different R5F core or memory region
     */
    local_addr = (uint32_t)priv->sampleOutputInterface;
    global_addr = CPU0_BTCM_SOCVIEW(local_addr);
    status = SDFM_setSampleOutputInterfaceGlobalAddr(handle, global_addr);
    if(status != SystemP_SUCCESS)
    {
        return SystemP_FAILURE;
    }

    /* Configure IEP count for snoop mode (one EPWM period) */
    if(attrs->pru_core_config[SDFM_PRU_CORE_INDEX].enable_snoop_mode || attrs->pru_core_config[SDFM_RTUPRU_CORE_INDEX].enable_snoop_mode || attrs->pru_core_config[SDFM_TXPRU_CORE_INDEX].enable_snoop_mode)
    {
        status = SDFM_configIepCount(handle, attrs->iep_reset_freq);
        if(status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
    }

    /* Configure operation mode (snoop/trigger) for each enabled PRU core */
    for(i = 0; i < NUM_OF_PRU_CORE_PER_PRU_SLICE; i++)
   {
        if(attrs->pru_core_mask & (1 << i))
        {

            if(attrs->pru_core_config[i].enable_snoop_mode)
            {
                status = SDFM_enableSnoopBasedNC(handle, i);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
            }
            if(attrs->pru_core_config[i].enable_trigger_mode == 1)
            {
                status = SDFM_enableTriggerModeForNormalCurrent(handle, i);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
                status = SDFM_setSampleTriggerTime(handle, attrs->pru_core_config[i].first_samp_trig_time, i);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
                if(attrs->pru_core_config[i].en_double_nc_sampling)
                {
                    status = SDFM_enableDoubleSampling(handle, attrs->pru_core_config[i].second_samp_trig_time, i);
                    if(status != SystemP_SUCCESS)
                    {
                        return SystemP_FAILURE;
                    }
                }
                else
                {
                    status = SDFM_disableDoubleSampling(handle, i);
                    if(status != SystemP_SUCCESS)
                    {
                        return SystemP_FAILURE;
                    }
                }

                status = SDFM_selectIepCmpEvent(handle, attrs->pru_core_config[i].iep_cmp_event, i);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
            }
        }
    }

    /* Configure EPWM synchronization if enabled */
    if(attrs->enable_epwm_sync)
    {
        status = SDFM_enableEpwmSync(handle, attrs->epwm_sync_source);
        if(status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
    }
    else
    {
        /* Configure IEP for trigger mode if any PRU core uses trigger mode */
        if(attrs->pru_core_config[0].enable_trigger_mode == 1|| attrs->pru_core_config[1].enable_trigger_mode == 1|| attrs->pru_core_config[2].enable_trigger_mode == 1)
        {
            status = SDFM_configIepCmp0ToResetIep(handle, attrs->iep_reset_freq);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }
        }
    }

    /* Measure phase delay for channel 0 if enabled */
#if CONFIG_SDFM0_PHASE_DELAY
        DebugP_log("\r\n Current Firmware only supports phase delay measurement on channel 0");
        if(attrs->channel_mask & (1 << 0))
        {
            status = SDFM_measureClockPhaseDelay(handle, attrs->channels[0].clk_inv, 0);
            if(status == SystemP_TIMEOUT)
            {
                DebugP_log("\r\nSDFM_measureClockPhaseDelay timeout error\n");
                return SystemP_FAILURE;
            }
            else if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nSDFM_measureClockPhaseDelay failed\n");
                return SystemP_FAILURE;
            }
        }
        else
        {
           DebugP_log("\r\n Phase delay measurement skipped as channel 0 is not enabled");
        }
#endif

    /* Configure filter parameters for all enabled channels */
    for(channel  = 0; channel  < SDFM_NUM_OF_CH_PER_PRU_SLICE; channel ++)
    {
        if(attrs->channel_mask & (1 << channel))
        {
            /* Configure overcurrent comparator filter OSR */
            status = SDFM_setCompFilterOverSamplingRatio(handle, channel , attrs->channels[channel].over_current_osr);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }

            /* Configure normal current filter OSR */
            status = SDFM_setFilterOverSamplingRatio(handle, channel , attrs->channels[channel].normal_current_osr);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }

            /* Configure data filter type (SINC3/Fast Response) */
            status = SDFM_configDataFilter(handle, channel, attrs->channels[channel].filter_type);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }

            /* Configure clock source and inversion for channel */
            status = SDFM_selectClockSource(handle, channel, attrs->channels[channel].clk_source);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }
            status = SDFM_setClockInversion(handle, channel, attrs->channels[channel].clk_inv);
            if(status != SystemP_SUCCESS)
            {
                return SystemP_FAILURE;
            }

            /* Configure overcurrent comparator if enabled */
            if(attrs->channels[channel].enable_comparator == 1)
            {
                status = SDFM_enableComparator(handle, channel);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
                /* Set high and low threshold values */
                threshold_config.high_threshold = attrs->channels[channel].high_threshold;
                threshold_config.low_threshold = attrs->channels[channel].low_threshold;
                status = SDFM_setCompFilterThresholds(handle, channel, threshold_config);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
            }

            /* Configure fast detect for quick overcurrent detection */
            if(attrs->channels[channel].fd_enable == 1)
            {
                fast_detect_config.fd_enable = attrs->channels[channel].fd_enable;
                fast_detect_config.fd_window_size = attrs->channels[channel].fd_window;
                fast_detect_config.fd_zero_max = attrs->channels[channel].fd_zero_max;
                fast_detect_config.fd_zero_min = attrs->channels[channel].fd_zero_min;
                status = SDFM_configFastDetect(handle, channel, fast_detect_config);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
            }

            /* Configure zero-cross detection if enabled */
            if(attrs->channels[channel].en_zero_cross == 1)
            {
                status = SDFM_enableZeroCrossDetection(handle, channel, attrs->channels[channel].zero_cross_threshold);
                if(status != SystemP_SUCCESS)
                {
                    return SystemP_FAILURE;
                }
                /* Configure GPIO pins for zero-cross output */
                sdfmConfigGpioPins(handle, channel);
            }
        }

    }

    /* Enable SDFM firmware on all enabled PRU cores to start sampling */
    for(i = 0; i < NUM_OF_PRU_CORE_PER_PRU_SLICE; i++)
    {
        if(attrs->pru_core_mask & (1 << i))
        {
            status = SDFM_enable(handle, i);
            if(status == SystemP_TIMEOUT)
            {
                DebugP_log("\r\nSDFM_enable timeout error for PRU core %d\n", i);
                return SystemP_FAILURE;
            }
            else if(status != SystemP_SUCCESS)
            {
                DebugP_log("\r\nSDFM_enable failed for PRU core %d\n", i);
                return SystemP_FAILURE;
            }
        }
    }

    return SystemP_SUCCESS;
}
/**
 *  \brief Load SDFM firmware into PRU cores and enable them
 *
 *  Loads appropriate firmware images based on load-share mode configuration.
 *  For single PRU mode, loads one firmware. For load-share mode, loads
 *  separate firmware to RTU PRU, PRU, and TX PRU cores.
 *
 *  \param pruIcssHandle  [in] PRU-ICSS handle
 *
 *  \return SystemP_SUCCESS on success, SystemP_FAILURE on failure
 */
int32_t sdfmLoadFirmware(PRUICSS_Handle pruIcssHandle)
{
    uint32_t pruIMem;
    PRUSDFM_PruFwImageInfo *pPruFwImageInfo;
    int32_t size;
    const uint32_t *sourceMem;          /* Source memory (array of uint32_t) */
    uint32_t imemOffset;                /* Offset at which write will happen */
    uint32_t byteLen;                   /* Total number of bytes to be written */
    int32_t status;

    /* Load firmware images into PRU cores */
    imemOffset = 0;
#if(CONFIG_SDFM0_LOAD_SHARE == 1)
    {
#if CONFIG_SDFM0_CHANNEL0 || CONFIG_SDFM0_CHANNEL1 || CONFIG_SDFM0_CHANNEL2
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_RTU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
        pPruFwImageInfo = &gPruFwImageInfo[2];
        pruIMem = PRUICSS_IRAM_RTU_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SystemP_FAILURE;
        }
        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_RTU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
#endif
#if CONFIG_SDFM0_CHANNEL3 || CONFIG_SDFM0_CHANNEL4 || CONFIG_SDFM0_CHANNEL5
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
        pPruFwImageInfo = &gPruFwImageInfo[1];
        pruIMem = PRUICSS_IRAM_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SystemP_FAILURE;
        }

        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
#endif
#if CONFIG_SDFM0_CHANNEL6 || CONFIG_SDFM0_CHANNEL7 || CONFIG_SDFM0_CHANNEL8
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_TXPRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
        pPruFwImageInfo = &gPruFwImageInfo[3];
        pruIMem = PRUICSS_IRAM_TX_PRU(CONFIG_SDFM0_SLICE);
        if(SDFM_TXPRU_CORE == PRUICSS_TX_PRU0)
        {
            PRUICSS_setConstantTblEntry(pruIcssHandle, SDFM_TXPRU_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A4);
        }
        else
        {
            PRUICSS_setConstantTblEntry(pruIcssHandle, SDFM_TXPRU_CORE, PRUICSS_CONST_TBL_ENTRY_C28, 0x2A5);
        }
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SystemP_FAILURE;
        }

        /* Enable PRU core */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_TXPRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
#endif
    }
#else
    {
        status = PRUICSS_resetCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS)
        {
            return SystemP_FAILURE;
        }
        pPruFwImageInfo = &gPruFwImageInfo[0];
        pruIMem = PRUICSS_IRAM_PRU(CONFIG_SDFM0_SLICE);
        /* Write IMEM */
        sourceMem = (uint32_t *)pPruFwImageInfo->pPruImemImg;
        byteLen = pPruFwImageInfo->pruImemImgSz;
        size = PRUICSS_writeMemory(pruIcssHandle, pruIMem, imemOffset, sourceMem, byteLen);
        if (size == 0)
        {
            return SystemP_FAILURE;
        }

        /* Enable PRU */
        status = PRUICSS_enableCore(pruIcssHandle, SDFM_PRU_CORE);
        if (status != SystemP_SUCCESS) {
            return SystemP_FAILURE;
        }
    }
#endif
    return SystemP_SUCCESS;

}
