/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#ifndef _SDDF_H_
#define _SDDF_H_

#include <stdint.h>
#include <drivers/pruicss.h>
#include "current_sense/sdfm/include/sddf_api.h"

/* Status codes */
#define SDDF_ERR_NERR               (  0 )  /* no error */
#define SDDF_ERR_CFG_PIN_MUX        ( -1 )  /* pin mux configuration error */
#define SDDF_ERR_CFG_ICSSG_CLKCFG   ( -2 )  /* ICSSG clock configuration error */
#define SDDF_ERR_INIT_ICSSG         ( -3 )  /* initialize ICSSG error */
#define SDDF_ERR_CFG_MCU_INTR       ( -4 )  /* interrupt configuration error */
#define SDDF_ERR_INIT_PRU_SDDF      ( -5 )  /* initialize PRU for SDDF error */
#define SDDF_ERR_INIT_SDDF          ( -6 )  /* initialize SDDF error */

/* Bit for SDDF configuration mask */
#define SDDF_CFG_CLK                    ( 1<<0 )
#define SDDF_CFG_OSR                    ( 1<<1 )
#define SDDF_CFG_TRIG_SAMP_TIME         ( 1<<2 )
#define SDDF_CFG_TRIG_SAMP_CNT          ( 1<<3 )
#define SDDF_CFG_CH_EN                  ( 1<<4 )
#define SDDF_CFG_FD                     ( 1<<5 )
#define SDDF_CFG_TRIG_OUT_SAMP_BUF      ( 1<<6 )

/* SDDF mode */
#define SDDF_MODE_TRIG                  ( 0 )
#define SDDF_MODE_CONT                  ( 1 )

/* ICSSG Core clock source selection options */
#define CORE_CLK_SEL_ICSSGn_CORE_CLK    ( 0 )   /* Mux Output */
#define CORE_CLK_SEL_ICSSGn_ICLK        ( 1 )   /* ICSSGn_ICLK = MAIN_SYSCLK0/2 = 250 MHz */
/* ICSSG Core clock selections in case Mux Output selected */
#define ICSSGn_CORE_CLK_SEL_MAIN_PLL2_HSDIV0_CLKOUT ( 0 )   /* 225 or 300 MHz, default 225 MHz */
#define ICSSGn_CORE_CLK_SEL_MAIN_PLL0_HSDIV9_CLKOUT ( 1 )   /* 200, 250, or 333 MHz, default 200 MHz */
#define ICSSGn_CORE_CLK_SEL_NUMSEL                  ( 2 )
/* ICSSG Core clock frequency in case Mux Output selected.
  Set to 0 in case clock frequency configuration not desired. */
#define ICSSGn_CORE_CLK_FREQ_225MHZ     ( 225000000UL ) /* MAIN PLL2 HSDIV0, 225 MHz */
#define ICSSGn_CORE_CLK_FREQ_300MHZ     ( 300000000UL ) /* MAIN PLL2 HSDIV0, 300 MHz */
#define ICSSGn_CORE_CLK_FREQ_200MHZ     ( 200000000UL ) /* MAIN PLL0 HSDIV9, 200 MHz */
#define ICSSGn_CORE_CLK_FREQ_250MHZ     ( 250000000UL ) /* MAIN PLL0 HSDIV9, 250 MHz */
#define ICSSGn_CORE_CLK_FREQ_333MHZ     ( 333333333UL ) /* MAIN PLL0 HSDIV9, 333 MHz */
#define ICSSGn_CORE_CLK_FREQ_NOCFG      ( 0UL )         /* No clock frequency reconfig */
//#define ICSSGn_CORE_CLK_FREQ            ( ICSSGn_CORE_CLK_FREQ_NOCFG )
#define ICSSGn_CORE_CLK_FREQ            ( ICSSGn_CORE_CLK_FREQ_300MHZ )
//#define ICSSGn_CORE_CLK_FREQ            ( ICSSGn_CORE_CLK_FREQ_333MHZ )

/* ICSSG IEP clock source selection options */
#define IEP_CLK_SEL_ICSSGn_IEP_CLK      ( 0 )   /* Mux Output */
#define IEP_CLK_SEL_CORE_CLK            ( 1 )   /* CORE_CLK */
/* ICSSG IEP clock selections in case Mux output selected */
#define ICSSGn_IEP_CLK_SEL_MAIN_PLL2_HSDIV5_CLKOUT  ( 0 )   /* Default 225 MHz */
#define ICSSGn_IEP_CLK_SEL_MAIN_PLL0_HSDIV6_CLKOUT  ( 1 )   /* 200 or 250 MHz, default 200 MHz */
#define ICSSGn_IEP_CLK_SEL_CPSW0_CPTS_RFT_CLK       ( 2 )
#define ICSSGn_IEP_CLK_SEL_CPTS_RFT_CLK             ( 3 )
#define ICSSGn_IEP_CLK_SEL_MCU_EXT_REFCLK0          ( 4 )
#define ICSSGn_IEP_CLK_SEL_EXT_REFCLK1              ( 5 )
#define ICSSGn_IEP_CLK_SEL_SERDES0_IP1_LN0_TXMCLK   ( 6 )
#define ICSSGn_IEP_CLK_SEL_SYSCLK0                  ( 7 )
#define ICSSGn_IEP_CLK_SEL_NUMSEL                   ( 8 )
/* ICSSG IEP clock frequency in case Mux Output selected.
   Set to 0 in case clock frequency configuration not desired. */
#define ICSSGn_IEP_CLK_FREQ_200MHZ      ( 200000000UL ) /* MAIN PLL0 HSDIV6, 200 MHz */
#define ICSSGn_IEP_CLK_FREQ_250MHZ      ( 250000000UL ) /* MAIN PLL0 HSDIV6, 250 MHz */
#define ICSSGn_IEP_CLK_FREQ_NOCFG       ( 0UL )          /* No clock frequency reconfig */
#define ICSSGn_IEP_CLK_FREQ             ( ICSSGn_IEP_CLK_FREQ_NOCFG )

/* Default ICSS pin mux setting */
#define PRUICSS_G_MUX_EN_DEF            ( 0x0 ) /* ICSSG_SA_MX_REG:G_MUX_EN */

/* Translate the TCM local view addr to SoC view addr */
#define CPU0_ATCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_ATCM_BASE+(x))
#define CPU1_ATCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_ATCM_BASE+(x))
#define CPU0_BTCM_SOCVIEW(x) (CSL_R5FSS0_CORE0_BTCM_BASE+(x - CSL_R5FSS0_BTCM_BASE))
#define CPU1_BTCM_SOCVIEW(x) (CSL_R5FSS1_CORE0_BTCM_BASE+(x - CSL_R5FSS1_BTCM_BASE))

#define ICSSG_SLICE_ID_0   ( 0 )    /* ICSSG slide ID 0 */
#define ICSSG_SLICE_ID_1   ( 1 )    /* ICSSG slide ID 1 */
#define ICSSG_NUM_SLICE    ( 2 )    /* ICSSG number of slices */

/*!
 *  @brief    PRUICSS Instance IDs
 */
typedef enum PRUICSS_MaxInstances_s
{
   PRUICSS_INSTANCE_ONE=0,
   PRUICSS_INSTANCE_TWO=1,
   PRUICSS_INSTANCE_MAX=2
} PRUICSS_MaxInstances;

/* SDDF configuration parameters */
typedef struct SddfPrms_s {
    float trigSampTime;
    uint32_t epwm_out_freq;
    SDFM_ThresholdParms   threshold_parms[NUM_CH_SUPPORTED];
    SDFM_ClkSourceParms   clkPrms[3];
    uint16_t ComFilterOsr;
    uint16_t FilterOsr;
    uint8_t en_com;
} SddfPrms;


/* Initialize ICSSG */
int32_t initIcss(
    uint8_t icssInstId,
    uint8_t sliceId,
    uint8_t saMuxMode,
    PRUICSS_Handle *pPruIcssHandle
);

/* Initialize PRU core for SDDF */
int32_t initPruSddf(
    PRUICSS_Handle pruIcssHandle,
    uint8_t pruInstId,
    SddfPrms *pSddfPrms,
    sdfm_handle *pHSddf
);


#endif /* _SDDF_H_ */
