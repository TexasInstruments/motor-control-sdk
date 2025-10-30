/*
 * Copyright (C) 2023-25 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SDFM_API_H_
#define _SDFM_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "../firmware/icssg_sdfm.h"
#include <current_sense/sdfm/include/sdfm_drv.h>
#include <drivers/pruicss.h>

 /**
 * \defgroup CURRENT_SENSE_API APIs for Current Sense
 *
 * This module contains APIs for device drivers for current sense supported in this SDK.
 */

/**
 *  \defgroup SDFM_API_MODULE APIs for SDFM
 *  \ingroup CURRENT_SENSE_API
 *
 * Here is the list of APIs used for Sigma Delta interface
 *
 *  @{
 */

/* Number of ICSSG PRUs */
#define NUM_PRU     ( 2 )

/* PRU IDs */
#define PRU_ID_0    ( 0 )   /* PRU 0 ID */
#define PRU_ID_1    ( 1 )   /* PRU 1 ID */

#define PRUx_DMEM_BASE_ADD     (0x00)
#define RTUx_DMEM_BASE_ADD     (0x200)
#define TXPRUx_DMEM_BASE_ADD   (0x400)
/* Number of SD channels */
#define NUM_SD_CH   ( ICSSG_NUM_SD_CH )
/* ICSSG INTC event */
#define SDFM_EVT    ( TRIGGER_HOST_SDFM_EVT )


/**
 *
 *  \brief  Initialize SDFM instance
 *  
 *  \param[in]  index           SDFM Instance index
 *  \param[in]  sdfm_params     SDFM initialization parameters
 *
 *  \retval SDFM_Handle         SDFM instance handle
 *
 */
SDFM_Handle SDFM_init(uint32_t index, SDFM_Params sdfm_params);

/**
 *
 *  \brief  Configure iep increment & iep count in one epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  iep_reset_freq  IEP reset frequency
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configIepCount(SDFM_Handle h_sdfm, uint32_t iep_reset_freq);

/**
 *
 *  \brief  Configure ecap parameters for generate SD clock
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ecap_divider    ecap divider for sdfm clock
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configEcap(SDFM_Handle h_sdfm, uint8_t ecap_divider);

/**
 *
 *  \brief  Configure comparator filter (over current) sampling ratio
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  osr             comparator filter/Over current sampling ratio
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 *  \note This API is used to configure the sampling ratio for the over-current
 *        filter. It should be called only when the snoop mode is enabled.
 *        When the snoop mode is disabled, the sampling ratio for the over-current
 *        filter is set through \ref SDFM_setFilterOverSamplingRatio.
 *
 */
int32_t SDFM_setCompFilterOverSamplingRatio(SDFM_Handle h_sdfm, uint8_t ch_id, uint16_t osr);

/**
 *
 *  \brief  configuration of SDFM threshold values
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  thresholdParms  thresholds parametrs (High and Low)
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_setCompFilterThresholds(SDFM_Handle h_sdfm, uint8_t ch_id, uint32_t *thresholdParms);

/**
 *
 *  \brief  configuration of single sample trigger time one Epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  samp_trig_time  first sample trigger time in one pwm cycle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_setSampleTriggerTime(SDFM_Handle h_sdfm, float samp_trig_time, uint8_t pru_core);

/**
 *
 *
 * \brief  configuration and enable second normal current sample starting time one Epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  samp_trig_time  second sample trigger time in one PWM cycle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_enableDoubleSampling(SDFM_Handle h_sdfm, float samp_trig_time, uint8_t pru_core);

/**
 *
 *
 * \brief  Disable double normal current update/sampling
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_disableDoubleSampling(SDFM_Handle h_sdfm, uint8_t pru_core);

/**
 *
 *  \brief  Enable the channel specified by the channel number parameter
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  channel_number  channel number
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_setEnableChannel(SDFM_Handle h_sdfm, uint8_t channel_number);

/**
 *
 *  \brief  configuration of SDFM channel Acc source (sync filter type). <br>
 *          Current SDFM firmware implementation support only SYNC3 filter <br>
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  filter          filter type (acc source)
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configDataFilter(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t filter);

/**
 *
 *  \brief  configuration of SDFM channel clock source 
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  clk_source      channel clock source type 
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_selectClockSource(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t clk_source);

/**
 *
 *  \brief  configuration of SDFM channel clock inversion  
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  clk_inv         channel clock inversion 
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_setClockInversion(SDFM_Handle h_sdfm, uint8_t ch_id, uint8_t clk_inv);

/**
 *
 *  \brief  This API enables the Comparator for the selected channel.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_enableComparator(SDFM_Handle h_sdfm, uint8_t ch);

/**
 *
 *  \brief  This API disables the Comparator for the selected channel.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_disableComparator(SDFM_Handle h_sdfm, uint8_t ch);

/**
 *
 *  \brief  configure GPIO pin number and address for associate Channel Number
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *  \param[in]  gpio_base_addr  GPIO base address
 *  \param[in]  pin_number      GPIO PIN number
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configComparatorGpioPins(SDFM_Handle h_sdfm, uint8_t ch, uint32_t gpio_base_addr, uint32_t pin_number);

/**
 *
 *  \brief  get sample data from DMEM
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *
 *  \retval uint32_t            Current sample value
 *
 */
uint32_t SDFM_getFilterData(SDFM_Handle h_sdfm, uint8_t ch);

/**
 *
 *  \brief  Configure normal current OSR.
 *
 *          If Snoop mode is used, it configures IEP count for normal current
 *          sampling.
 *          \note h_sdfm->snoop_mode must be set to 1 before calling this API for snoop mode configuration 
 *
 *          If Snoop mode is not used, it configures SD HW OSR equal to
 *          matNC OSR.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *  \param[in]  nc_osr          Normal current osr value
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_setFilterOverSamplingRatio(SDFM_Handle h_sdfm, uint8_t ch, uint16_t nc_osr);

/**
 *
 *  \brief  Return Firmware version
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *  \retval uint32_t            Release version of firmware
 *
 */
uint32_t SDFM_getFirmwareVersion(SDFM_Handle h_sdfm);

/**
 *  \brief  Enable trigger mode for normal current sampling
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableTriggerModeForNormalCurrent(SDFM_Handle h_sdfm, uint8_t pru_core);

/**
 *  \brief  This API Configure Fast detect block fields.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch              current ch number
 *  \param[in]  fdParms         array of fast detect fields. {window size, zero max count, zero min count, one max count, one min count}
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_configFastDetect(SDFM_Handle h_sdfm, uint8_t ch, uint8_t *fdParms);

/**
 *  \brief This API returns the fast detect error status for specified SDFM channel number.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             Status of fd error: 1 means error available & 0 means no error, SystemP_FAILURE on not expected API parameters
 *
 */
int32_t SDFM_getFastDetectErrorStatus(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *  \brief  Clear PWM trip status of the corresponding PWM trip zone block for specified SDFM channel number.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error or not expected API parameters
 */
int32_t SDFM_clearPwmTripStatus(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API enables load share mode
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  sliceId         slice ID
 *
 */
void SDFM_enableLoadShareMode(SDFM_Handle h_sdfm, uint8_t sliceId);

/**
 *
 *  \brief  Measure Clock phase compensation
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  clkEdg          Clock polarity: 1 -> falling edge, 0 -> raising edge
 *  \param[in]  chNum           SDFM channel number
 * 
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_measureClockPhaseDelay(SDFM_Handle h_sdfm, uint16_t clkEdg, uint8_t chNum);

/**
 *
 *  \brief  This API returns Clock phase compensation
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number
 *
 *  \retval float               Phase delay in nano sec, SystemP_FAILURE on error
 */
float SDFM_getClockPhaseDelay(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API returns High threshold Status for specified SDFM channel number
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             Status of over current error for High Threshold, SystemP_FAILURE on error
 */
int32_t SDFM_getHighThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API returns Low threshold Status for specified SDFM channel number
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             Status of Over current error for Low threshold, SystemP_FAILURE on error
 */
int32_t SDFM_getLowThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API clears Overcurrent error bit of corresponding PWM register for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on not expected API parameters
 */
int32_t SDFM_clearOverCurrentError(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API enables zero cross detection for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *  \param[in]  zcThr           zero cross threshold
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableZeroCrossDetection(SDFM_Handle h_sdfm, uint8_t chNum, uint32_t zcThr);

/**
 *
 *  \brief  This API returns Zero cross Status for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             Status of zero cross, SystemP_FAILURE on error
 */
int32_t SDFM_getZeroCrossThresholdStatus(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API disbales zero cross detection for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_disableZeroCrossDetection(SDFM_Handle h_sdfm, uint8_t chNum);

/**
 * 
 * \brief This API enables EPWM synchronization with SDFM
 * \param[in]  h_sdfm          SDFM handle
 * \param[in]  epwmIns         epwm instance: Only epwm0/epwm3 support synchronization with sdfm 
 * 
 * \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableEpwmSync(SDFM_Handle h_sdfm, uint8_t epwmIns);

/**
 * 
 * \brief This API disbale EPWM synchronization with SDFM
 * \param[in]  h_sdfm          SDFM handle
 * \param[in]  epwmIns         epwm instance: Only epwm0/epwm3 support synchronization with sdfm
 * 
 * \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_disableEpwmSync(SDFM_Handle h_sdfm, uint8_t epwmIns);

/**
 * 
 * \brief     This API configures IEP SYNC0 and SYNC1 registers to generate free running clock
 * \param[in]  h_sdfm          SDFM handle
 * \param[in]  highPulseWidth   Number of clock cycles SYNC0/1 will be high.
 *                             0h = 1 clock cycle.
 *                             1h = 2 clock cycles.
 *                             Nh: N+1 clock cycles.
 * \param[in]  periodTime      Period between the rising edges of SYNC0
 *                             1h = 2 clk cycles period
 *                             Nh = N+1 clk cycles period
 * \param[in]  syncStartTime  SYNC0 and SYNC1 activation time
 * 
 * \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_configIepSyncMode(SDFM_Handle h_sdfm, uint32_t highPulseWidth, uint32_t periodTime, uint32_t syncStartTime);

/**
 * 
 * \brief This API enables IEP counter
 * 
 * \param[in]  h_sdfm          SDFM handle
 * 
 * \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableIep(SDFM_Handle h_sdfm);

/**
 *  \brief  Defines clock cycles from the start of SYNC0 to the start of SYNC1
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  delay           Delay before the start of SYNC1
 *                          
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_configSync1Delay(SDFM_Handle h_sdfm, uint32_t delay);

/***
 *  \brief  This API configures PRU GPO mode as shift out mode (ICSSG_GPCFG0_REG[14] PRU<n>_GPO_MODE = 1h) and  
 *          shift out mode's clock divisors to output the SD clock on PR<k>_PRUx_GPO1 pin.
 *  \brief  PRU0_GPO_DIV0 and PRU0_GPO_DIV1 configuration values 
 *  \brief  0x0:  for divisor 1
 *  \brief  0x1:  for divisor 1.5
 *  \brief  0x2:  for divisor 2
 *  \brief  0x3:  for divisor 2.5
 *       .
 *       .
 *       .
 *  \brief  0xID: for divisor 15.5
 *  \brief  0xIE: for divisor 16
 *  \brief  0x1F: reserved
 * 
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  div0            PRUx_GPO_DIV0 value
 *  \param[in]  div1            PRUx_GPO_DIV1 value
 * 
 * \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on not expected API parameters
*/
int32_t SDFM_configClockFromGPO1(SDFM_Handle h_sdfm, uint8_t div0, uint8_t div1);

/**
 *  \brief  Enable snoop based normal current sampling for a specific PRU core
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *  
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableSnoopBasedNC(SDFM_Handle h_sdfm, uint8_t pru_core);

/**
 *  \brief  Disable snoop based normal current sampling for a specific PRU core
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *  
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_disableSnoopBasedNC(SDFM_Handle h_sdfm, uint8_t pru_core);

/**
 *
 *  \brief  Set sample output interface global address
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  addr            sample output interface global address
 *
 */

void SDFM_setSampleOutputInterfaceGlobalAddr(SDFM_Handle h_sdfm, uint32_t addr);

/**
 *
 *  \brief  Select IEP comparator event for a specific PRU core
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  event           IEP comparator event
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_selectIepCmpEvent(SDFM_Handle h_sdfm, uint8_t event, uint8_t pru_core);

/**
 *
 *  \brief  Configure IEP CMP0 to reset IEP counter
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  iep_reset_freq  IEP reset frequency
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configIepCmp0ToResetIep(SDFM_Handle h_sdfm, uint32_t iep_reset_freq);
/**
 *
 *  \brief  SDFM global enable for a specific PRU core
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0, RTU - 1, TXPRU - 2
 *
 */
void SDFM_enable(SDFM_Handle h_sdfm, uint8_t pru_core);

/** @} */

#ifdef __cplusplus
}
#endif

#endif