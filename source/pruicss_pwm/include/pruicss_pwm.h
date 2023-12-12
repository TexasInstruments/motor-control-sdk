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

 /**
 * \defgroup PRUICSS_PWM_API APIs for PRUICSS PWM
 *
 * This module contains APIs for device driver pruicss_pwm supported in this SDK.
 * 
 * PRUICSS has one pwm module, which has four pwm sets (PWM0, PWM1, PWM2, PWM3)
 * Each Set has six signals (A0,A1,A2,B0,B1,B2)
 * With Reference to Technical Reference Manual, Pwm six signals(A0,A1,A2,B0,B1,B2) Naming convention is slightly different
 * 
 *          PWMn==============>PWMn_0=========>PWMn_0_POS (alias signal A0)
 *                  |                   |=====>PWMn_0_NEG (alias signal A1)
 *                  |
 *                  | 
 *                  |   
 *                  |=========>PWMn_1=========>PWMn_1_POS (alias signal A2)
 *                  |                   |=====>PWMn_1_NEG (alias signal B0)
 *                  |
 *                  | 
 *                  |           
 *                  |=========>PWMn_2=========>PWMn_2_POS (alias signal B1)  
 *                                      |=====>PWMn_2_NEG (alias signal B2) 
 * 
 * Each Set has one trip zone output OR logic block
 * Each trip zone block has nine trip_error signals (trip_e1_[0:2], trip_e2, trip_e3[0:2], trip_e4, trip_e5) as input 
 * And one PWMn_TZ_OUT output signal which makes transition to safe or trip state from current state
 *        
 *                                                                   ________________
 *             tripn_e1_[2:0](Debounce Trip)----------------------->|                |
 *                                                                  |                |
 *             tripn_e2(Debounce Error Trip_in)-------------------->|                |              _____________
 *                                                                  |                |              |           |
 *             trip_e3[2:0](SD Fast detect Error Trip)------------->|   Tripzone     |------------->|PWMn_TZ_OUT|
 *                                                                  |    output      |              |___________| 
 *                                                                  |     logic      |
 *             trip_e4(SD over current Error Trip)----------------->|                |
 *                                                                  |                |
 *             trip_e5(Position Error trip)------------------------>|                |
 *                                                                  |________________|               
 * 
 * 
 * 
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifndef PRUICSS_PWM_H_
#define PRUICSS_PWM_H_

#include <stddef.h>
#include <stdint.h>
#include <drivers/pruicss.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/**
 *  
 *  \anchor PRUICSS_PWM_SET
 *  \name PRUICSS PWM SET
 * 
 * @{
 */
#define PRUICSS_NUM_PWM_SETS                                (0x4U)

#define PRUICSS_PWM_SET0                                    (0x0U)

#define PRUICSS_PWM_SET1                                    (0x1U)   

#define PRUICSS_PWM_SET2                                    (0x2U)    

#define PRUICSS_PWM_SET3                                    (0x3U)
/** @} */


/**
 *  
 *  \anchor PRUICSS_PWM_STATE
 *  \name PRUICSS PWM STATE
 * 
 * @{
 */
#define PRUICSS_NUM_PWM_STATES                              (0x3U)

#define PRUICSS_PWM_INTIAL_STATE                            (0x0U)

#define PRUICSS_PWM_ACTIVE_STATE                            (0x1U)

#define PRUICSS_PWM_TRIP_STATE                              (0x2U)
/** @} */

/**
 *  
 *  \anchor PRUICSS_PWM_OUTPUT_ACTION
 *  \name PRUICSS PWM OUTPUT ACTION
 * 
 * @{
 */
#define PRUICSS_NUM_PWM_OUTPUT_ACTIONS                       (0x3U)

#define PRUICSS_PWM_OUTPUT_TOGGLE                            (0x0U)

#define PRUICSS_PWM_OUTPUT_LOW                               (0x1U)

#define PRUICSS_PWM_OUTPUT_HIGH                              (0x2U)
/** @} */

/**
 *  
 *  \anchor PRUICSS_IEP_INSTANCE
 *  \name PRUICSS IEP INSTANCE
 * 
 * @{
 */
#define PRUICSS_NUM_IEP_INSTANCES                           (0x2U)

#define PRUICSS_IEP_INST0                                   (0x0U)

#define PRUICSS_IEP_INST1                                   (0x1U)
/** @} */


/**
 * \brief PRUICSS IEP count register maximum value
 */
#define PRUICSS_IEP_COUNT_REG_MAX                           (0xFFFFFFFFU)

/**
 *  
 *  \anchor PRUICSS_IEP_COMPARE_EVENT
 *  \name PRUICSS IEP COMPARE EVENT
 * 
 * @{
 */
#define PRUICSS_NUM_IEP_CMP_EVENTS                          (0x10U)

#define CMP_EVENT0                                          (0x0U)

#define CMP_EVENT1                                          (0x1U)

#define CMP_EVENT2                                          (0x2U)

#define CMP_EVENT3                                          (0x3U)

#define CMP_EVENT4                                          (0x4U)

#define CMP_EVENT5                                          (0x5U)

#define CMP_EVENT6                                          (0x6U)

#define CMP_EVENT7                                          (0x7U)

#define CMP_EVENT8                                          (0x8U)

#define CMP_EVENT9                                          (0x9U)

#define CMP_EVENT10                                         (0xAU)

#define CMP_EVENT11                                         (0xBU)

#define CMP_EVENT12                                         (0xCU)

#define CMP_EVENT13                                         (0xDU)

#define CMP_EVENT14                                         (0xEU)

#define CMP_EVENT15                                         (0xFU)
/** @} */

/**
 * \brief Number of PRUICSS IEP compare events enable field maximum value 
 */
#define PRUICSS_IEP_CMP_EVENTS_ENABLE_MAX_VALUE             (0x0000FFFFU)

/**
 * \brief PRUICSS PWM Debounce maximum value
 */
#define PRUICSS_PWM_DEBOUNCE_MAX_VALUE                      (0xFFU)

/**
 * \brief PRUICSS PWM mask field maximum value
 */
#define PRUICSS_PWM_TRIP_MASK_MAX_VALUE                     (0x000001FFU)



/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 * \brief  This API writes Lower_32bit Value of IEP counter in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Iep count register lower 32bit value
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_setIepCounterLower_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint32_t value);

/**
 * \brief  This API writes Upper_32bit Value of IEP counter in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Iep count register upper 32bit value
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_setIepCounterUpper_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint32_t value);

/**
 * \brief  This API sets enables/disables of IEP counter reset on compare 0 event in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   enable      0 for disable, 1 for enable
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_configureIepCmp0ResetEnable(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable);

/**
 * \brief  This API sets enables/disables compare events in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Value to store in compare enable field of compare config register
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_configureIepCompareEnable(PRUICSS_Handle handle, uint8_t iepInstance, uint16_t value);

/**
 * \brief  This API writes Lower_32bit Value of compare event in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Compare register lower 32bit value
 * \param   cmpEvent    Compare event number. maximum value allowed is 15
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_setIepCompareEventLower_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value);

/**
 * \brief  This API writes Upper_32bit Value of compare event in IEP module.
 *      
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   iepInstance 0 for IEP0, 1 for IEP1
 * \param   value       Compare register upper 32bit Value
 * \param   cmpEvent    Compare event number. maximum value allowed is 15
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 *
 */
int32_t PRUICSS_PWM_setIepCompareEventUpper_32bitValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t cmpEvent, uint32_t value);

/**
 * \brief  This API updates Debounce Value for specified pwm set
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   value       Pwmset debounce value
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_setPwmDebounceValue(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t value);

/**
 * \brief  This API updates TripMask Value for specified pwm set
 * 
 * \brief  Software TripMask
 * \brief  0x0: PWM0_POS_ERR_TRIP (trip_e5)
 * \brief  0x1: PWM0_OVER_ERR_TRIP (trip_e4)
 * \brief  0x2: PWM0_0_SD_SHORT_ERR_TRIP (trip_e3_0)
 * \brief  0x3: PWM0_1_SD_SHORT_ERR_TRIP (trip_e3_1)
 * \brief  0x4: PWM0_2_SD_SHORT_ERR_TRIP (trip_e3_2)
 * \brief  0x5: PWM0_DEBOUNCE_TRIP_IN (trip_e2)
 * \brief  0x6: PWM0_0_DEBOUNCE_TRIP (trip_e1_0)
 * \brief  0x7: PWM0_1_DEBOUNCE_TRIP (trip_e1_1)
 * \brief  0x8: PWM0_2_DEBOUNCE_TRIP (trip_e1_2)
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   maskvalue   Pwmset maskvalue
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_setPwmTripMask(PRUICSS_Handle handle, uint8_t pwmSet, uint16_t maskvalue);

/**
 * \brief  This API enables/disables TripReset on Compare_0 Event for specified pwm set
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   enable      0 for disable, 1 for enable
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_configurePwmCmp0TripResetEnable(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t enable);

/**
 * \brief  This API generates Software Trip Reset by writing 1 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_generatePwmTripReset(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API generates Software Over current error trip by writing 1 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_generatePwmOverCurrentErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API generates Software Position Feedback Error Trip by writing 1 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_generatePwmPositionFeedbackErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API clears Software Trip Reset by writing 0 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_clearPwmTripResetStatus(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API clears Software Over current error trip by writing 0 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_clearPwmOverCurrentErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API clears Software Position Feedback Error Trip by writing 0 to bit field for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_clearPwmPositionFeedbackErrorTrip(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API returns Trip trigger cause vector for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  Trip trigger cause vector on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_getPwmTripTriggerCauseVector(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API returns Trip status for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  Trip status on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_getPwmTripStatus(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API clears Trip status and makes state transition to Intial state as follows (Active->Intial) 
 *  or (Trip->Intial) for specified pwm set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_clearPwmTripStatus(PRUICSS_Handle handle, uint8_t pwmSet);

/**
 * \brief  This API updates output action for specified state of A0 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA0(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API updates output action for specified state of A1 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA1(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API updates output action for specified state of A2 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalA2(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API updates output action for specified state of B0 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB0(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API updates output action for specified state of B1 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB1(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API updates output action for specified state of B2 signal for specified PWM set 
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   pwmSet      0 for PWM0, 1 for PWM1, 2 for PWM2, 3 for PWM3
 * \param   state       0 for Intial, 1 for Active, 2 for Safe(alias Trip) state
 * \param   action      0 for Toggle, 1 for Low, 2 for High
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_actionOnOutputCfgPwmSignalB2(PRUICSS_Handle handle, uint8_t pwmSet, uint8_t state, uint8_t action);

/**
 * \brief  This API enables Efficiency mode
 * In Efficiency mode Pwm state machine will go from Idle to
 * Active and the same time Pwm output will get updated during this
 * state transition And Iep Cmp flags will get auto HW cleared
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   enable       0 for disable, 1 for enable
 * \return  SystemP_SUCCESS on success, SystemP_FAILURE on error
 * 
*/
int32_t PRUICSS_PWM_configurePwmEfficiencyModeEnable(PRUICSS_Handle handle, uint8_t enable);

/**
 * \brief   This function enables IEP1 counter follow IEP0 counter
 *  when Enabled IEP1 counter acts as slave
 *  IEP1 counter[63:0] is from IEP0 during 64-bit mode
 *  IEP1 counter[31:0] is from IEP0 during 32-bit mode
 * 
 * \param   handle      PRUICSS_Handle returned from PRUICSS_open()
 * \param   enable      0 for disable, 1 for enable
 * \return  SystemP_SUCCESS in case of success, SystemP_FAILURE otherwise
 */
int32_t PRUICSS_PWM_enableIEP1Slave(PRUICSS_Handle handle, uint8_t enable);

/** @} */

#ifdef __cplusplus
}
#endif

#endif/* #ifndef PRUICSS_PWM_H_ */