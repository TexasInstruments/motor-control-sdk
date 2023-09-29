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
 *  \param[in]  pru_id          PRU (SD) ID
 *
 *  \retval sdfm  SDFM instance handle
 *
 */
sdfm_handle SDFM_init(uint8_t pru_id);

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
void SDFM_setCompFilterThresholds(sdfm_handle h_sdfm, uint8_t ch_id, SDFM_ThresholdParms thresholdParms);

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
 *  \brief  configuration of SDFM channel clock source & clock inversion
 *
 *  \param[in]  h_sdfm          SDFM handle
 *  \param[in]  ch_id           current ch number
 *  \param[in]  clkPrams        channel clock source type & clock inversion
 *
 *
 *
 */
void SDFM_selectClockSource(sdfm_handle h_sdfm, uint8_t ch_id, SDFM_ClkSourceParms clkPrams);

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
 *  \param[in]  threshold_type  Threshold type: High(0), Low(1)
 *
 *
 *
 */
void SDFM_configComparatorGpioPins(sdfm_handle h_sdfm, uint8_t ch,uint32_t gpio_base_addr, uint32_t pin_number, uint32_t threshold_type);

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


/** @} */

#ifdef __cplusplus
}
#endif

#endif
