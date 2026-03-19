/*
 * Copyright (C) 2023-2026 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *  \file   sdfm_api.h
 *
 *  \brief  Public API declarations for SDFM (Sigma-Delta Filter Module) driver.
 *
 *  \details
 *  This header file contains all public API function declarations for the SDFM driver.
 *
 *  ## Pointer Validation
 *
 *  All APIs validate internal structure pointers.
 *
 *  ## API Categories
 *
 *  ### Initialization and Configuration
 *  - SDFM_init(): Initialize SDFM instance with parameters from SysConfig
 *  - SDFM_setEnableChannel(): Enable specific SDFM channels
 *  - SDFM_enable(): Global enable for SDFM on specific PRU core
 *
 *  ### Clock Configuration
 *  - SDFM_selectClockSource(): - 0: Use pr\<k\>_pru\<n\>_sd8_clk (common SDFM clock pin for all channels)
 *                              - 1: Use pr\<k\>_pru\<n\>_sd\<i\>_clk (channel-specific clock)
 *                              - 2: Use group clocks:
 *                                      - pr\<k\>_pru\<n\>_sd0_clk for channels 0, 1, and 2
 *                                      - pr\<k\>_pru\<n\>_sd3_clk for channels 3, 4, and 5
 *                                      - pr\<k\>_pru\<n\>_sd6_clk for channels 6, 7, and 8
 *  - SDFM_setClockInversion(): Configure clock polarity
 *  - SDFM_configEcap(): Configure ECAP for SD clock generation
 *  - SDFM_configClockFromGPO1(): Configure PRU GPO shift-out mode for clock
 *  - SDFM_configIepSyncMode(): Configure IEP SYNC0/SYNC1 for free-running clock
 *
 *  ### Filter Configuration
 *  - SDFM_configDataFilter(): Select filter type (SINC1/SINC2/SINC3)
 *  - SDFM_setFilterOverSamplingRatio(): Configure normal current OSR
 *  - SDFM_setCompFilterOverSamplingRatio(): Configure over-current OSR
 *
 *  ### Threshold and Comparator Configuration
 *  - SDFM_setCompFilterThresholds(): Set high/low over-current thresholds
 *  - SDFM_enableComparator(): Enable over-current comparator
 *  - SDFM_disableComparator(): Disable over-current comparator
 *  - SDFM_enableZeroCrossDetection(): Enable zero-crossing detection
 *  - SDFM_disableZeroCrossDetection(): Disable zero-crossing detection
 *  - SDFM_configComparatorGpioPins(): Associate GPIO pins for threshold events
 *
 *  ### Fast Detect Configuration
 *  - SDFM_configFastDetect(): Configure fast detect parameters for error detection
 *
 *  ### Trigger Mode Configuration
 *  - SDFM_enableTriggerModeForNormalCurrent(): Enable IEP-based triggered sampling
 *  - SDFM_setSampleTriggerTime(): Set first sample trigger time in PWM cycle
 *  - SDFM_enableDoubleSampling(): Enable second sample in PWM cycle
 *  - SDFM_disableDoubleSampling(): Disable second sample
 *  - SDFM_selectIepCmpEvent(): Select IEP comparator event for trigger
 *  - SDFM_configIepCount(): Configure IEP count for PWM period
 *  - SDFM_configIepCmp0ToResetIep(): Configure IEP CMP0 to reset counter
 *
 *  ### Snoop Mode Configuration
 *  - SDFM_enableSnoopBasedNC(): Enable snoop mode for a PRU core
 *  - SDFM_disableSnoopBasedNC(): Disable snoop mode for a PRU core
 *
 *  ### EPWM Synchronization
 *  - SDFM_enableEpwmSync(): Enable EPWM synchronization
 *  - SDFM_disableEpwmSync(): Disable EPWM synchronization
 *
 *  ### Data Retrieval
 *  - SDFM_getFilterData(): Read current sample from specified channel
 *
 *  ### Status and Monitoring
 *  - SDFM_getHighThresholdStatus(): Check high threshold violation
 *  - SDFM_getLowThresholdStatus(): Check low threshold violation
 *  - SDFM_getZeroCrossThresholdStatus(): Check zero-crossing event
 *  - SDFM_getFastDetectErrorStatus(): Check fast detect error
 *  - SDFM_clearOverCurrentError(): Clear over-current error flag
 *  - SDFM_clearPwmTripStatus(): Clear PWM trip status
 *
 *  ### Phase Compensation
 *  - SDFM_measureClockPhaseDelay(): Measure clock phase delay between channels
 *  - SDFM_getClockPhaseDelay(): Get measured phase delay value
 *
 *  ### Utility Functions
 *  - SDFM_getFirmwareVersion(): Get PRU firmware version
 *  - SDFM_setSampleOutputInterfaceGlobalAddr(): Set sample output buffer address
 *  - SDFM_enableIep(): Enable IEP counter
 *  - SDFM_configSync1Delay(): Configure SYNC1 delay relative to SYNC0
 *
 *  ## Usage Example
 *
 *  \code
 *  // 1. Initialize SDFM with SysConfig parameters
 *  SDFM_Handle hSdfm = SDFM_init(CONFIG_SDFM0, gSdfmParams);
 *
 *  // 2. Enable channels
 *  SDFM_setEnableChannel(hSdfm, SDFM_CH0);
 *  SDFM_setEnableChannel(hSdfm, SDFM_CH1);
 *
 *  // 3. Configure thresholds
 *  uint32_t thresholds[2] = {HIGH_THRESHOLD, LOW_THRESHOLD};
 *  SDFM_setCompFilterThresholds(hSdfm, SDFM_CH0, thresholds);
 *
 *  // 4. Enable SDFM
 *  SDFM_enable(hSdfm, SDFM_PRU_CORE_INDX);
 *
 *  // 5. Read samples
 *  uint32_t sample = SDFM_getFilterData(hSdfm, SDFM_CH0);
 *  \endcode
 *
 *  ## Related Files
 *
 *  - sdfm_drv.h: Data structures, macros, and type definitions
 *  - sdfm_drv.c: Driver implementation
 *  - icssg_sdfm.h: Firmware interface definitions
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

/**
 *
 *  \brief  Initialize SDFM parameters structure with default values
 *
 *  \param[out]  params         Pointer to SDFM_Params structure to initialize
 *
 */
void SDFM_paramsInit(SDFM_Params *params);

/**
 *
 *  \brief  Initialize SDFM instance
 *
 *  \param[in]  index           SDFM Instance index
 *  \param[in]  params          Pointer to SDFM initialization parameters
 *
 *  \retval SDFM_Handle         SDFM instance handle on success, NULL on failure
 *
 */
SDFM_Handle SDFM_init(uint32_t index, SDFM_Params *params);

/**
 *
 *  \brief  Deinitialize SDFM instance
 *
 *  \param[in]  handle          SDFM handle
 *
 */
void SDFM_deinit(SDFM_Handle handle);

/**
 *
 *  \brief  Get SDFM attributes (compile-time configuration)
 *
 *  \param[in]  handle          SDFM handle
 *
 *  \retval const SDFM_Attrs*   Pointer to SDFM attributes, NULL if handle is invalid
 *
 */
const SDFM_Attrs* SDFM_getAttrs(SDFM_Handle handle);

/**
 *
 *  \brief  Get SDFM private data (runtime state)
 *
 *  \param[in]  handle          SDFM handle
 *
 *  \retval SDFM_Priv*          Pointer to SDFM private data, NULL if handle is invalid
 *
 */
SDFM_Priv* SDFM_getPriv(SDFM_Handle handle);

/**
 *
 *  \brief  Configure IEP counter reset cycle time period
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  iep_reset_freq  IEP counter reset frequency. Typically equal to EPWM output frequency, used to synchronize IEP counter with EPWM cycle
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or iep_reset_freq is 0
 *
 */
int32_t SDFM_configIepCount(SDFM_Handle handle, uint32_t iep_reset_freq);

/**
 *
 *  \brief  Configure eCAP parameters for SD clock generation
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  ecap_divider    ecap divider for sdfm clock
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or ecap_divider is 0
 *
 */
int32_t SDFM_configEcap(SDFM_Handle handle, uint8_t ecap_divider);

/**
 *
 *  \brief  Configure comparator filter (over current) sampling ratio
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  osr             comparator filter/Over current sampling ratio. Valid range: 4-256
 *
 *  \note The OSR value is validated against the range 4-256 and decremented by 1
 *        before writing to hardware (user OSR 4-256 -> register 3-255).
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error (invalid osr range)
 *
 *  \note This API is used to configure the sampling ratio for the over-current
 *        filter. It should be called only when the snoop mode is enabled.
 *        When the snoop mode is disabled, the sampling ratio for the over-current
 *        filter is set through \ref SDFM_setFilterOverSamplingRatio.
 *
 */
int32_t SDFM_setCompFilterOverSamplingRatio(SDFM_Handle handle, uint8_t channel, uint16_t osr);

/**
 *
 *  \brief  Configure SDFM comparator filter threshold values
 *
 *  \param[in]  handle           SDFM handle
 *  \param[in]  channel          SDFM channel number (0-8)
 *  \param[in]  threshold_config Threshold configuration structure
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL), channel > 8,
 *                              or threshold values are invalid (high_threshold <= low_threshold or exceeds max)
 *
 */
int32_t SDFM_setCompFilterThresholds(SDFM_Handle handle, uint8_t channel, SDFM_ThresholdConfig threshold_config);

/**
 *
 *  \brief  Configure the first sample trigger time within one EPWM cycle
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  samp_trig_time  first sample trigger time in one pwm cycle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or pru_core >= 3
 *
 */
int32_t SDFM_setSampleTriggerTime(SDFM_Handle handle, float samp_trig_time, uint8_t pru_core);

/**
 *
 *
 * \brief  Configure and enable second normal current sample trigger time within one EPWM cycle
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  samp_trig_time  second sample trigger time in one PWM cycle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or pru_core >= 3
 *
 */
int32_t SDFM_enableDoubleSampling(SDFM_Handle handle, float samp_trig_time, uint8_t pru_core);

/**
 *
 *
 * \brief  Disable double normal current sampling
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or pru_core >= 3
 *
 */
int32_t SDFM_disableDoubleSampling(SDFM_Handle handle, uint8_t pru_core);

/**
 *
 *  \brief  Enable the channel specified by the channel number parameter
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel_number  channel number (0-8)
 *
 *  \retval SystemP_SUCCESS     Channel enabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel_number > 8
 *
 */
int32_t SDFM_setEnableChannel(SDFM_Handle handle, uint8_t channel_number);

/**
 *
 *  \brief  Configure SDFM channel accumulator filter type (SINC1/SINC2/SINC3)
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  filter          filter type (acc source). Valid values: 0=SINC3, 1=SINC2, 2=SINC1
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL), channel > 8, or filter > 2
 *
 */
int32_t SDFM_configDataFilter(SDFM_Handle handle, uint8_t channel, uint8_t filter);

/**
 *
 *  \brief  Configure SDFM channel clock source
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  clk_source      channel clock source type. Valid range: 0-2
 *                              - 0: Use pr\<k\>_pru\<n\>_sd8_clk (common SDFM clock pin for all channels)
 *                              - 1: Use pr\<k\>_pru\<n\>_sd\<i\>_clk (channel-specific clock)
 *                              - 2: Use group clocks:
 *                                      - pr\<k\>_pru\<n\>_sd0_clk for channels 0, 1, and 2
 *                                      - pr\<k\>_pru\<n\>_sd3_clk for channels 3, 4, and 5
 *                                      - pr\<k\>_pru\<n\>_sd6_clk for channels 6, 7, and 8
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL), channel > 8, or clk_source > 2
 *
 */
int32_t SDFM_selectClockSource(SDFM_Handle handle, uint8_t channel, uint8_t clk_source);

/**
 *
 *  \brief  Configure SDFM channel clock inversion
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  clk_inv         channel clock inversion
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel > 8
 *
 */
int32_t SDFM_setClockInversion(SDFM_Handle handle, uint8_t channel, uint8_t clk_inv);

/**
 *
 *  \brief  This API enables the Comparator for the selected channel.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval SystemP_SUCCESS     Comparator enabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel > 8
 *
 */
int32_t SDFM_enableComparator(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API disables the Comparator for the selected channel.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval SystemP_SUCCESS     Comparator disabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel > 8
 *
 */
int32_t SDFM_disableComparator(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  configure GPIO pin number and address for associate Channel Number
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  gpio_base_addr  GPIO base address
 *  \param[in]  pin_number      GPIO PIN number
 *
 *  \retval SystemP_SUCCESS     GPIO pins configured successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel > 8
 *
 */
int32_t SDFM_configComparatorGpioPins(SDFM_Handle handle, uint8_t channel, uint32_t gpio_base_addr, uint32_t pin_number);

/**
 *
 *  \brief  get sample data from DMEM
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval uint32_t            Current sample value. Returns 0 on failure (invalid handle or channel).
 *
 *  \note A return value of 0 can also represent a valid zero current sample.
 *
 */
uint32_t SDFM_getFilterData(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  Configure normal current OSR.
 *
 *          If Snoop mode is used, it configures IEP count for normal current
 *          sampling.
 *          \note Snoop mode must be enabled via SDFM_enableSnoopBasedNC() before calling this API for snoop mode configuration
 *
 *          If Snoop mode is not used, it configures SD HW OSR equal to
 *          matNC OSR.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  nc_osr          Normal current osr value. Valid range: 4-256
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error (invalid osr range)
 *
 *  \note The OSR value is validated against the range 4-256 and decremented by 1
 *        before writing to hardware (user OSR 4-256 -> register 3-255).
 */
int32_t SDFM_setFilterOverSamplingRatio(SDFM_Handle handle, uint8_t channel, uint16_t nc_osr);

/**
 *
 *  \brief  Return Firmware version
 *
 *  \param[in]  handle          SDFM handle
 *
 *  \retval uint32_t            Release version of firmware. Returns 0 on failure (invalid handle).
 *
 *  \note A return value of 0 can also represent a valid firmware version.
 *
 */
uint32_t SDFM_getFirmwareVersion(SDFM_Handle handle);

/**
 *  \brief  Enable trigger mode for normal current sampling
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableTriggerModeForNormalCurrent(SDFM_Handle handle, uint8_t pru_core);

/**
 *  \brief  Configure SDFM fast detect block parameters
 *
 *  This API configures the fast detect feature for rapid error detection in SDFM channels.
 *  Fast detect monitors zero crossings within a window to quickly identify sensor faults.
 *
 *  \param[in]  handle              SDFM handle
 *  \param[in]  channel             SDFM channel number (0-8)
 *  \param[in]  fast_detect_config  Fast detect configuration structure
 *
 *  \retval SystemP_SUCCESS     Configuration successful
 *  \retval SystemP_FAILURE     Invalid handle (NULL), channel > 8,
 *                              or parameter values out of valid range
 *
 */
int32_t SDFM_configFastDetect(SDFM_Handle handle, uint8_t channel, SDFM_FastDetectConfig fast_detect_config);

/**
 *  \brief This API returns the fast detect error status for specified SDFM channel number.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             Status of fd error: 1 means error available & 0 means no error, SystemP_FAILURE on not expected API parameters
 *
 */
int32_t SDFM_getFastDetectErrorStatus(SDFM_Handle handle, uint8_t channel);

/**
 *  \brief  Clear PWM trip status of the corresponding PWM trip zone block for specified SDFM channel number.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error or not expected API parameters
 */
int32_t SDFM_clearPwmTripStatus(SDFM_Handle handle, uint8_t channel);


/**
 *
 *  \brief  Measure Clock phase compensation
 *
 *  This function triggers PRU firmware to measure the clock phase delay for the specified channel
 *  and waits for the measurement to complete with timeout protection.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  clk_edg         Clock polarity: 1 -> falling edge, 0 -> rising edge
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval SystemP_SUCCESS     Phase delay measurement completed successfully
 *  \retval SystemP_FAILURE     Invalid parameters
 *  \retval SystemP_TIMEOUT     Firmware acknowledgment timeout
 *
 *  \note This is a blocking function with timeout protection. Maximum wait time is
 *        (SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT * SDFM_DEFAULT_FW_WAIT_DELAY_US) microseconds.
 *        Returns SystemP_TIMEOUT if firmware does not acknowledge within timeout period.
 */
int32_t SDFM_measureClockPhaseDelay(SDFM_Handle handle, uint16_t clk_edg, uint8_t channel);

/**
 *
 *  \brief  This API returns Clock phase compensation
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval float               Phase delay in nano sec, SystemP_FAILURE on error
 */
float SDFM_getClockPhaseDelay(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API returns High threshold Status for specified SDFM channel number
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             Status of over current error for High Threshold, SystemP_FAILURE on error
 */
int32_t SDFM_getHighThresholdStatus(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API returns Low threshold Status for specified SDFM channel number
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             Status of Over current error for Low threshold, SystemP_FAILURE on error
 */
int32_t SDFM_getLowThresholdStatus(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API clears Overcurrent error bit of corresponding PWM register for specified SDFM channel number
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on not expected API parameters
 */
int32_t SDFM_clearOverCurrentError(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API enables zero cross detection for specified SDFM channel number
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *  \param[in]  zc_thr          zero cross threshold
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_enableZeroCrossDetection(SDFM_Handle handle, uint8_t channel, uint32_t zc_thr);

/**
 *
 *  \brief  This API returns Zero cross Status for specified SDFM channel number
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval int32_t             Status of zero cross, SystemP_FAILURE on error
 */
int32_t SDFM_getZeroCrossThresholdStatus(SDFM_Handle handle, uint8_t channel);

/**
 *
 *  \brief  This API disables zero cross detection for specified SDFM channel number
 *  \param[in]  handle          SDFM handle
 *  \param[in]  channel         SDFM channel number (0-8)
 *
 *  \retval SystemP_SUCCESS     Zero cross detection disabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or channel > 8
 */
int32_t SDFM_disableZeroCrossDetection(SDFM_Handle handle, uint8_t channel);

/**
 *
 * \brief This API enables EPWM synchronization with SDFM
 * \param[in]  handle          SDFM handle
 * \param[in]  epwm_ins        epwm instance: Only epwm0/epwm3 support synchronization with sdfm
 *
 * \retval SystemP_SUCCESS     EPWM synchronization enabled successfully
 * \retval SystemP_FAILURE     Invalid handle (NULL) or unsupported epwm_ins (only 0 or 3 supported)
 */
int32_t SDFM_enableEpwmSync(SDFM_Handle handle, uint8_t epwm_ins);

/**
 *
 * \brief This API disables EPWM synchronization with SDFM
 * \param[in]  handle          SDFM handle
 * \param[in]  epwm_ins        epwm instance: Only epwm0/epwm3 support synchronization with sdfm
 *
 * \retval SystemP_SUCCESS     EPWM synchronization disabled successfully
 * \retval SystemP_FAILURE     Invalid handle (NULL) or unsupported epwm_ins (only 0 or 3 supported)
 */
int32_t SDFM_disableEpwmSync(SDFM_Handle handle, uint8_t epwm_ins);

/**
 *
 * \brief     This API configures IEP SYNC0 and SYNC1 registers to generate free running clock
 * \param[in]  handle            SDFM handle
 * \param[in]  high_pulse_width  Number of clock cycles SYNC0/1 will be high.
 *                               0h = 1 clock cycle.
 *                               1h = 2 clock cycles.
 *                               Nh: N+1 clock cycles.
 * \param[in]  period_time       Period between the rising edges of SYNC0
 *                               1h = 2 clk cycles period
 *                               Nh = N+1 clk cycles period
 * \param[in]  sync_start_time   SYNC0 and SYNC1 activation time
 *
 * \retval SystemP_SUCCESS       IEP SYNC mode configured successfully
 * \retval SystemP_FAILURE       Invalid handle (NULL)
 */
int32_t SDFM_configIepSyncMode(SDFM_Handle handle, uint32_t high_pulse_width, uint32_t period_time, uint32_t sync_start_time);

/**
 *
 * \brief This API enables IEP counter
 *
 * \param[in]  handle          SDFM handle
 *
 * \retval SystemP_SUCCESS     IEP counter enabled successfully
 * \retval SystemP_FAILURE     Invalid handle (NULL)
 */
int32_t SDFM_enableIep(SDFM_Handle handle);

/**
 *  \brief  Defines clock cycles from the start of SYNC0 to the start of SYNC1
 *  \param[in]  handle          SDFM handle
 *  \param[in]  delay           Delay before the start of SYNC1
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */
int32_t SDFM_configSync1Delay(SDFM_Handle handle, uint32_t delay);

/***
 *  \brief  This API configures PRU GPO mode as shift out mode (ICSSG_GPCFG0_REG[14] PRU<n>_GPO_MODE = 1h) and
 *          shift out mode's clock divisors to output the SD clock on PR\<k\>_PRUx_GPO1 pin.
 *  \brief  PRU0_GPO_DIV0 and PRU0_GPO_DIV1 configuration values
 *  \brief  0x0:  for divisor 1
 *  \brief  0x1:  for divisor 1.5
 *  \brief  0x2:  for divisor 2
 *  \brief  0x3:  for divisor 2.5
 *       .
 *       .
 *       .
 *  \brief  0x1D: for divisor 15.5
 *  \brief  0x1E: for divisor 16
 *  \brief  0x1F: reserved
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  div0            PRUx_GPO_DIV0 value
 *  \param[in]  div1            PRUx_GPO_DIV1 value
 *
 * \retval SystemP_SUCCESS     GPO1 clock configured successfully
 * \retval SystemP_FAILURE     Invalid handle (NULL), div0 out of range, or div1 out of range
 */
int32_t SDFM_configClockFromGPO1(SDFM_Handle handle, uint8_t div0, uint8_t div1);

/**
 *  \brief  Enable snoop based normal current sampling for a specific PRU core
 *  \param[in]  handle          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     Snoop mode enabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or pru_core >= 3
 */
int32_t SDFM_enableSnoopBasedNC(SDFM_Handle handle, uint8_t pru_core);

/**
 *  \brief  Disable snoop based normal current sampling for a specific PRU core
 *  \param[in]  handle          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     Snoop mode disabled successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL) or pru_core >= 3
 */
int32_t SDFM_disableSnoopBasedNC(SDFM_Handle handle, uint8_t pru_core);

/**
 *
 *  \brief  Set sample output interface global address
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  addr            Sample output interface global address
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 */

int32_t SDFM_setSampleOutputInterfaceGlobalAddr(SDFM_Handle handle, uint32_t addr);

/**
 *
 *  \brief  Select IEP comparator event for a specific PRU core
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  event           IEP comparator event (0-15)
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     IEP compare event selected successfully
 *  \retval SystemP_FAILURE     Invalid handle (NULL), pru_core >= 3, or event > 15
 *
 */
int32_t SDFM_selectIepCmpEvent(SDFM_Handle handle, uint8_t event, uint8_t pru_core);

/**
 *
 *  \brief  Configure IEP CMP0 to reset IEP counter
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  iep_reset_freq  IEP counter reset frequency. Typically equal to EPWM output frequency, used to synchronize IEP counter with EPWM cycle
 *
 *  \retval int32_t             SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t SDFM_configIepCmp0ToResetIep(SDFM_Handle handle, uint32_t iep_reset_freq);
/**
 *
 *  \brief  SDFM global enable for a specific PRU core
 *
 *  This function enables SDFM operation on the specified PRU core and waits for
 *  firmware acknowledgment with timeout protection.
 *
 *  \param[in]  handle          SDFM handle
 *  \param[in]  pru_core        PRU core ID (0-2): PRU - 0 (Ch 3-5 in load-share), RTU - 1 (Ch 0-2 in load-share), TXPRU - 2 (Ch 6-8 in load-share)
 *
 *  \retval SystemP_SUCCESS     SDFM enabled successfully
 *  \retval SystemP_FAILURE     Invalid handle or pru_core parameter
 *  \retval SystemP_TIMEOUT     Firmware acknowledgment timeout
 *
 *  \note This is a blocking function with timeout protection. Maximum wait time is
 *        (SDFM_DEFAULT_MAX_WAIT_LOOP_COUNT * SDFM_DEFAULT_FW_WAIT_DELAY_US) microseconds.
 *        Returns SystemP_TIMEOUT if firmware does not acknowledge within timeout period.
 */
int32_t SDFM_enable(SDFM_Handle handle, uint8_t pru_core);

/** @} */

#ifdef __cplusplus
}
#endif

#endif