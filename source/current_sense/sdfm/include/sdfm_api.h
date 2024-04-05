/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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

/* SDFM handle */
typedef SDFM *sdfm_handle;

/**
 *
 *  \brief  Initialize SDFM instance
 *  
 *  \param[in]  pruIcssHandle   PRU-ICSS handle 
 *  \param[in]  pruId          Pru slice id
 *  \param[in]  coreId          Pru core id
 *
 *  \retval sdfm  SDFM instance handle
 *
 */
sdfm_handle SDFM_init(PRUICSS_Handle pruIcssHandle, uint8_t pruId, uint8_t coreId);

/**
 *
 *  \brief  Configure iep increment  & iep count in one epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  epwm_out_freq        epwm output frequency
 *
 *
 *
 */
void SDFM_configIepCount(sdfm_handle h_sdfm, uint32_t epwm_out_freq);

/**
 *
 *  \brief  Configure ecap parameters for generate 20MHz SD clock
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ecap_divider    ecap divider for sdfm clock
 *
 *
 *
 */
void SDFM_configEcap(sdfm_handle h_sdfm, uint8_t ecap_divider);

/**
 *
 *  \brief  Enable the channel specified by the channel number parameter
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in] channel_number    channel number
 *
 *
 *
 *
 */
void SDFM_setEnableChannel(sdfm_handle h_sdfm, uint8_t channel_number);

/**
 *
 *  \brief  Configure comparator filter (over current) sampling ratio
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  osr             comparator filter/Over current sampling ratio
 *
 *
 *
 */
void SDFM_setCompFilterOverSamplingRatio(sdfm_handle h_sdfm, uint8_t ch_id, uint16_t osr);

/**
 *
 *  \brief  SDFM global enable
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *
 *
 */
void SDFM_enable(sdfm_handle h_sdfm);

/**
 *
 *  \brief  configuration of SDFM threshold values
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  thresholdParms   thresholds parametrs (High and Low )
 *
 *
 *
 */
void SDFM_setCompFilterThresholds(sdfm_handle h_sdfm, uint8_t ch_id, uint32_t *thresholdParms);

/**
 *
 *  \brief  configuration of single sample trigger time one Epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in] samp_trig_time     first sample trigger time in one pwm cycle
 *
 *
 *
 */
void SDFM_setSampleTriggerTime(sdfm_handle h_sdfm, float samp_trig_time);

/**
 *
 *
 * \brief  configuration and enable second normal current sample starting time one Epwm cycle
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in] samp_trig_time     second sample trigger time in one PWM cycle
 *
 *
 *
*/
void SDFM_enableDoubleSampling(sdfm_handle h_sdfm, float samp_trig_time);

/**
 *
 *
 * \brief  Disable double normal current update/sampling
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *
 *
*/
void SDFM_disableDoubleSampling(sdfm_handle h_sdfm);


/**
 *
 *  \brief  configuration of SDFM channel Acc source (sync filter type). <br>
 *          Current SDFM firmware implementation support only SYNC3 filter <br>
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  filter          filter type (acc source)
 *
 *
 *
 */
void SDFM_configDataFilter(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t filter);

/**
 *
 *  \brief  configuration of SDFM channel clock source 
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  clk_source       channel clock source type 
 *
 *
 *
 */
void SDFM_selectClockSource(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t clk_source);

/**
 *
 *  \brief  configuration of SDFM channel clock inversion  
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  clk_source       channel clock inversion 
 *
 *
 *
 */
void SDFM_setClockInversion(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t clk_inv);

/**
 *
 *  \brief  This API enables the Comparator for the selected channel.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch           current ch number
 *
 *
 *
 *
 */
void SDFM_enableComparator(sdfm_handle h_sdfm, uint8_t ch);

/**
 *
 *  \brief  This API disables the Comparator for the selected channel.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch           current ch number
 *
 *
 *
 *
 */
void SDFM_disableComparator(sdfm_handle h_sdfm, uint8_t ch);

/**
 *
 *  \brief  configure GPIO pin number and address for associate Channel Number
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch           current ch number
 *  \param[in]  gpio_base_addr  GPIO bas address
 *  \param[in]  pin_number      GPIO PIN number
 *
 *
 *
 */
void SDFM_configComparatorGpioPins(sdfm_handle h_sdfm, uint8_t ch,uint32_t gpio_base_addr, uint32_t pin_number);

/**
 *
 *  \brief  get sample data from DMEM
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch           current ch number
 *
 *
 *  \retval current sample value
 *
 */
uint32_t SDFM_getFilterData(sdfm_handle h_sdfm,uint8_t ch);
/**
 *
 *  \brief  Configure iep count for normal current sampling
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  nc_osr          Normal current osr value
 *
 *
 */
void SDFM_setFilterOverSamplingRatio(sdfm_handle h_sdfm, uint16_t nc_osr);
/**
 *
 *  \brief  Return Firmware version
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *  \retval     firmwareVersion    release vesrion of firmware
 *
 */
uint32_t SDFM_getFirmwareVersion(sdfm_handle h_sdfm);

/**
 *  \brief  This API Configure Fast detect block fields.
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch           current ch number
 *  \param[in]  fdParms   array of fast detect fields. {window size, zero max count, zero min count, one max count, one min count}
 *
 *
 *
 */
void SDFM_configFastDetect(sdfm_handle h_sdfm, uint8_t ch, uint8_t *fdParms);

/**
 *  \brief This API returns the fast detect error status for specified SDFM channel number.
 *
 *  \param[in]  h_sdfm       SDFM handle
 *  \param[in]  chNum        SDFM channel number : Channel0-Channel8
 *
 *  \retval    stauts of fd error: 1 means error available & 0 means no error, SystemP_FAILURE on not expected API parameters
 *
 */
int32_t SDFM_getFastDetectErrorStatus(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *  \brief  Clear PWM trip status of the corresponding PWM trip zone block for specified SDFM channel number.
 *
 *  \param[in]  h_sdfm       SDFM handle
 *  \param[in]  chNum        SDFM channel number : Channel0-Channel8
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on error or not expected API parameters
 */
int32_t SDFM_clearPwmTripStatus(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API enables continuous normal current sampling
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *
 */
void SDFM_enableContinuousNormalCurrent(sdfm_handle h_sdfm);

/**
 *
 *  \brief  This API enables load share mode
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  sliceId         slice ID
 *
 */
void SDFM_enableLoadShareMode(sdfm_handle h_sdfm, uint8_t sliceId);

/**
 *
 *  \brief  Measure Clock phase compensation
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  clEdg           Clock polarity: 1 -> falling edge, 0 -> raising edge
 *
 * 
 */
void SDFM_measureClockPhaseDelay(sdfm_handle h_sdfm, uint16_t clEdg);

/**
 *
 *  \brief  This API returns Clock phase compensation
 *
 *  \param[in]  h_sdfm          SDFM handle
 *
 *  \retval    Phase delay      in nano sec
 * 
 */
float SDFM_getClockPhaseDelay(sdfm_handle h_sdfm);
/**
 *
 *  \brief  This API returns Low threshold Status for specified SDFM channel number
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval     Status of Over current error for Low threshold
 */
uint8_t SDFM_getLowThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API returns high threshold Status for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval     Status of over current error for High Threshold
 */
uint8_t SDFM_getHighThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API clears Overcurrent error bit of corresponding PWM register for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval     SystemP_SUCCESS on success, SystemP_FAILURE on not expected API parameters
 */
int32_t SDFM_clearOverCurrentError(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API enables zero cross detection for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *  \param[in]  zcThr            zero cross threshold
 *
 */
void SDFM_enableZeroCrossDetection(sdfm_handle h_sdfm, uint8_t chNum, uint32_t zcThr);

/**
 *
 *  \brief  This API returns Zero cross Status for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *  \retval     Status of zero cross
 */
uint8_t SDFM_getZeroCrossThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum);

/**
 *
 *  \brief  This API disbales zero cross detection for specified SDFM channel number
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  chNum           SDFM channel number : Channel0-Channel8
 *
 *
 */
void SDFM_disableZeroCrossDetection(sdfm_handle h_sdfm, uint8_t chNum);

/**
 * 
 * \brief This API enables EPWM synchronization with SDFM
 * \param[in]  h_sdfm          SDFM handle
 * \param[in]  epwmIns         epwm instance: Only epwm0/epwm3 support synchronization with sdfm 
 * 
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t SDFM_enableEpwmSync(sdfm_handle h_sdfm, uint8_t epwmIns);

/**
 * 
 * \brief This API disbale EPWM synchronization with SDFM
 * \param[in]  h_sdfm          SDFM handle
 * \param[in]  epwmIns         epwm instance: Only epwm0/epwm3 support synchronization with sdfm
 * 
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t SDFM_disableEpwmSync(sdfm_handle h_sdfm, uint8_t epwmIns);
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
 * \retval     SystemP_SUCCESS on success, SystemP_FAILURE on not expected API parameters
*/
int32_t SDFM_configClockFromGPO1(sdfm_handle h_sdfm, uint8_t div0, uint8_t div1);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
