; Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
; OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
; SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
; LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

;************************************************************************************
;   File:     		main.asm
;   Description: 	Updates IEP compare value to PRUICSS PWM signals
;************************************************************************************

;************************************************************************************
;   Note:	  If the PRU core is paused, it is required to reset.
;************************************************************************************

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
; icss_constant_defines.inc: Defines symbols corresponding to Constant Table Entries
    .include "icss_constant_defines.inc"
    .include "icss_iep_macros.inc"
    .include "icss_cfg_regs.inc"

    .cdecls C,  NOLIST
%{
#include "drivers/pruicss/g_v0/cslr_icss_g.h"
%}


    .asg	R0.b0,		XFR2_VBUS_IPC_SPAD_SHIFT
    .asg    R0.b1,      TEMP_BYTE
    .asg    R0.w2,      TEMP_WORD
    .asg    R1,			TEMP_REG1
    ;R2 - R4 		s_axis_xPwmCmpUpOrDownIncrementValues
    .asg    R5,     axis_xPwm0CmpLowValue
    .asg    R6,     axis_xPwm0CmpHighValue
    .asg    R7,     axis_xPwm1CmpLowValue
    .asg    R8,     axis_xPwm1CmpHighValue
    .asg    R9,     axis_xPwm2CmpLowValue
    .asg    R10,    axis_xPwm2CmpHighValue
    .asg    R11,    axis_xPwm3CmpLowValue
    .asg    R12,    axis_xPwm3CmpHighValue
    .asg    R13,    axis_xPwm4CmpLowValue
    .asg    R14,    axis_xPwm4CmpHighValue
    .asg    R15,    axis_xPwm5CmpLowValue
    .asg    R16,    axis_xPwm5CmpHighValue
    ;IEP0_CMP13 used as even axis scheduler
    .asg    R17,    SCHEDULER_CMP_LOW_VALUE
    .asg    R18,    SCHEDULER_CMP_HIGH_VALUE
	;Increment values after updating compare values axis 1 to axis 3
	.asg    R19.w0, AXIS0_AXIS1_INC_VALUE
	.asg    R19.w2, AXIS1_AXIS2_INC_VALUE
	.asg    R20.w0, AXIS2_AXIS0_INC_VALUE
	;R20.w2 to R23 unused
    ;Flag to check whether compare up value configured or not
    .asg    R24,    axis_xCmpUpConfigured
    .asg	R24.b0, axis0PwmxCmpUpConfigured
    .asg	R24.b1, axis1PwmxCmpUpConfigured
    .asg	R24.b2, axis2PwmxCmpUpConfigured
    .asg    "R24.b0 , 0", axis0Pwm0CmpUpConfigured
    .asg    "R24.b0 , 1", axis0Pwm1CmpUpConfigured
    .asg    "R24.b0 , 2", axis0Pwm2CmpUpConfigured
    .asg    "R24.b0 , 3", axis0Pwm3CmpUpConfigured
    .asg    "R24.b0 , 4", axis0Pwm4CmpUpConfigured
    .asg    "R24.b0 , 5", axis0Pwm5CmpUpConfigured
    .asg    "R24.b1 , 0", axis1Pwm0CmpUpConfigured
    .asg    "R24.b1 , 1", axis1Pwm1CmpUpConfigured
    .asg    "R24.b1 , 2", axis1Pwm2CmpUpConfigured
    .asg    "R24.b1 , 3", axis1Pwm3CmpUpConfigured
    .asg    "R24.b1 , 4", axis1Pwm4CmpUpConfigured
    .asg    "R24.b1 , 5", axis1Pwm5CmpUpConfigured
    .asg    "R24.b2 , 0", axis2Pwm0CmpUpConfigured
    .asg    "R24.b2 , 1", axis2Pwm1CmpUpConfigured
    .asg    "R24.b2 , 2", axis2Pwm2CmpUpConfigured
    .asg    "R24.b2 , 3", axis2Pwm3CmpUpConfigured
    .asg    "R24.b2 , 4", axis2Pwm4CmpUpConfigured
    .asg    "R24.b2 , 5", axis2Pwm5CmpUpConfigured
	.asg     R25,     	  PRUICSS_PWM_PARAMS_DMEM_ADDR

;************************ IEP Resources allocation ********************************
;IEP compare registers usage.
;IEP0_CMP1 to IEP0_CMP12 used by axis 0 and axis 1.
;IEP1_CMP1 to IEP1_CMP6 used by axis 2.
;IEP0_CMP13 is used to update compare values from axis 0 to 2.
;**********************************************************************************

;6 byte read from DMEM takes  (2 + 2) cycles
s_axis_xPwmCmpValues    	 			.struct
axisPwm0CmpUpOrDownIncVal		.uhalf
axisPwm1CmpUpOrDownIncVal		.uhalf
axisPwm2CmpUpOrDownIncVal		.uhalf
axisPwm3CmpUpOrDownIncVal		.uhalf
axisPwm4CmpUpOrDownIncVal		.uhalf
axisPwm5CmpUpOrDownIncVal		.uhalf
s_axispwmCmpValues_len        		.endstruct

axis_xPwmCmpValuesITR   .sassign  R2,  s_axis_xPwmCmpValues

;Compare event used by scheduler to update PRUICSS PWM compare values
SCHEDULER_CMP_EVENT									.set    13
;IEP constant table entry used by scheduler
SCHEDULER_IEP_CONST                                 .set    ICSS_IEP_CONST
;Compare Value used to schedule compare updates of axis 0 to axis 2
;TODO: need to add this offset in iep include file
ICSS_IEP_CMP13_REG									.set	0xE8
SCHEDULER_CMP_REG_OFFSET_VALUE                      .set    ICSS_IEP_CMP13_REG

;********************************************************************************
;IEP CMP VALUES OFFSET
;********************************************************************************
;Compare values low and high axis 0, axis 1, axis 2 offset
AXIS0_IEP0_CMP_VALUES_OFFSET						.set	ICSS_IEP_CMP1_REG
AXIS1_IEP0_CMP_VALUES_OFFSET						.set	ICSS_IEP_CMP7_REG
AXIS2_IEP1_CMP_VALUES_OFFSET						.set	ICSS_IEP_CMP1_REG

;6 Compare register values size (6 compare values per axis* 8)
AXISX_CMP_REG_VALUES_SIZE							.set 	0x30

;TODO: neeed to add these offsets in cfg include file
;********************************************************************************
;PWM STATE CONFIGURATION OFFSET
;********************************************************************************
ICSS_CFG_1_PWM0											.set	0x30
ICSS_CFG_1_PWM1											.set	0x34
ICSS_CFG_1_PWM2											.set	0x38
ICSS_CFG_1_PWM3											.set	0x3C
ICSS_CFG_1_PWM0_0										.set	0x40
ICSS_CFG_1_PWM0_1										.set	0x44
ICSS_CFG_1_PWM0_2										.set	0x48
ICSS_CFG_1_PWM1_0										.set	0x4C
ICSS_CFG_1_PWM1_1										.set	0x50
ICSS_CFG_1_PWM1_2										.set	0x54
ICSS_CFG_1_PWM2_0										.set	0x58
ICSS_CFG_1_PWM2_1										.set	0x5C
ICSS_CFG_1_PWM2_2										.set	0x60
ICSS_CFG_1_PWM3_0										.set	0x64
ICSS_CFG_1_PWM3_1										.set	0x68
ICSS_CFG_1_PWM3_2										.set	0x6C

;********************************************************************************
;DMEM0 OFFSETS
;********************************************************************************
;Compare values up and down increment values dmem offset of axis 0, axis 1, axis 2
PWM_PARAMS_DMEM0_OFFSET                         .set    0x200
AXIS0_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x0
AXIS0_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0xC
AXIS1_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x18
AXIS1_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0x24
AXIS2_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x30
AXIS2_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0x3C
AXIS0_AXIS1_INC_VAL_OFFSET						.set	0x48
AXIS1_AXIS2_INC_VAL_OFFSET						.set	0x4A
AXIS2_AXIS0_INC_VAL_OFFSET						.set	0x4C
SCHEDULER_INITIAL_VAL_OFFSET					.set	0x50

;********************************************************************************
;Additional IEP & PWM macros
;********************************************************************************
IEP_INST0											.set    0
IEP_INST1											.set    1
PWM_SET0											.set    0
PWM_SET1											.set    1
PWM_SET2											.set    2
PWM_SET3											.set    3
PWM_0                                               .set    0
PWM_1                                               .set    1
PWM_2                                               .set    2
PWM_3                                               .set    3
PWM_4                                               .set    4
PWM_5                                               .set    5
PWM_A_B_STATES_INIT_VALUE							.set 	0x00A9
AXIS_X_CMP_UP_CONFIGURED_VALUE                      .set    0x003F3F3F

;************************************************************************************
;   Macro: m_wait_until_schedular_compare_event_is_hit
;
;   Description : Wait until scheduler compare event is set
;
;   Parameters: None
;
;   PEAK cycles:
;           4 cycles
;************************************************************************************
m_wait_until_schedular_compare_event_is_hit	.macro
wait?:
    lbco    &TEMP_REG1, SCHEDULER_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 2
    qbbc    wait?, TEMP_REG1, SCHEDULER_CMP_EVENT
    .endm

;************************************************************************************
;   Macro: m_clr_flag_if_equal
;
;   Description : macro used to clear bit index in PRU reg byte(flag) 
;		          when scheduler low_high_value is equal to cmp_low_high_value
;
;   Parameters:
;           pru_reg_byte   - PRU register byte
;           pwm_num        - bit index in PRU register byte to clear
;           low_value  	   - low value to compare with sch_low_value	
;           high_value     - high value to compare with sch_high_value	
;           sch_low_value  - sch_low_value
;			sch_high_value - sch_high_value
;
;   PEAK cycles:
;           3 cycles
;************************************************************************************
m_clr_flag_if_equal	.macro		pru_reg_byte, pwm_num, low_value, high_value, sch_low_value, sch_high_value
	qbne	skip_clr?, low_value, sch_low_value
	qbne	skip_clr?, high_value, sch_high_value
	clr		pru_reg_byte, pru_reg_byte, pwm_num
skip_clr?
	.endm

;************************************************************************************
;   Macro: m_compute_axis_x_compare_up_values_and_clear_flag
;
;   Description :
;           macro used to compute and clear flag for axis_x pwm compare up values
;
;   Parameters:
;           axis_x_pwm_x_flag - axis_x pwm flag
;
;   PEAK cycles:
;           30 cycles(5 cycles per PWM signal)
;************************************************************************************
m_compute_axis_x_compare_up_values_and_clear_flag		.macro	axis_x_pwm_x_flag
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm0CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm0CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm0CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_0_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_0, axis_xPwm0CmpLowValue, axis_xPwm0CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm1CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm1CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm1CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_1_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_1, axis_xPwm1CmpLowValue, axis_xPwm1CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm2CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm2CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm2CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_2_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_2, axis_xPwm2CmpLowValue, axis_xPwm2CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm3CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm3CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm3CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_3_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_3, axis_xPwm3CmpLowValue, axis_xPwm3CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm4CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm4CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm4CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_4_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_4, axis_xPwm4CmpLowValue, axis_xPwm4CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	;Change IEP compare low reg values with compare up increment value + scheduler value
	ADD		axis_xPwm5CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm5CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
	;if axis_x_Pwm5CmpLowValue is equal to scheduler value, then skip setting axis_x_pwm_5_flag to 1
	m_clr_flag_if_equal	 axis_x_pwm_x_flag, PWM_5, axis_xPwm5CmpLowValue, axis_xPwm5CmpHighValue, SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_HIGH_VALUE
	.endm

;******************************************************
;	Macro: m_compute_axis_x_compare_down_values_when_flag_set
;
;	Description :
;			Compute axis compare down values when flag is set
;
;	Parameters:
;   		axis_x_pwm_x_flag: flag to check if compare up value is configured or not
;
;   PEAK cycles:
;           18 cycles(3 cycles per PWM signal)
;
;******************************************************
m_compute_axis_x_compare_down_values_when_flag_set		.macro	axis_x_pwm_x_flag
	;if pwm0 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_0?, axis_x_pwm_x_flag, PWM_0
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm0CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm0CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_0?:
	;if pwm 1 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_1?, axis_x_pwm_x_flag, PWM_1
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm1CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm1CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_1?:
	;if pwm 2 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_2?, axis_x_pwm_x_flag, PWM_2
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm2CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm2CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_2?:
	;if pwm 3 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_3?, axis_x_pwm_x_flag, PWM_3
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm3CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm3CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_3?:
	;if pwm 4 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_4?, axis_x_pwm_x_flag, PWM_4
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm4CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm4CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_4?:
	;if pwm 5 compare up value is not configured then skip updating compare down value
	QBBC	skip_axisx_5?, axis_x_pwm_x_flag, PWM_5
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm5CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm5CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_5?:
	.endm

;***************************************************************************
;	Macro: m_generate_software_trip_reset_event
;
;	Description :
;			Generates a software trip/reset event for the specified PWM
;
;	Parameters:
;   		PWM_SET: PWM to set trip/reset event
;			TEMP_REG1: temporary register to be used
;
;   PEAK cycles:
;           7 cycles
;****************************************************************************
m_generate_software_trip_reset_event		.macro PWM_SET, TEMP_REG1 
	.if   PWM_SET = 0
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	OR       TEMP_REG1.b2, TEMP_REG1.b2, 0x4
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	.elseif PWM_SET = 1
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	OR       TEMP_REG1.b2, TEMP_REG1.b2, 0x4
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	.elseif PWM_SET = 2
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	OR       TEMP_REG1.b2, TEMP_REG1.b2, 0x4
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	.elseif PWM_SET = 3
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	OR       TEMP_REG1.b2, TEMP_REG1.b2, 0x4
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	.endif
	.endm

;************************************************************************************
;   Macro:   m_clear_software_trip_reset_event
;
;   Description:
;   		Clears software trip/reset event for the specified PWM
;
;	Parameters:
;   		PWM_SET: PWM to set trip/reset event
;			TEMP_REG1: temporary register to be used
;
;   PEAK cycles:
;           7 cycles
;************************************************************************************
m_clear_software_trip_reset_event		.macro PWM_SET, TEMP_REG1 
	.if   PWM_SET = 0
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	AND       TEMP_REG1.b2, TEMP_REG1.b2, 0xFB
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	.elseif PWM_SET = 1
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	AND       TEMP_REG1.b2, TEMP_REG1.b2, 0xFB
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	.elseif PWM_SET = 2
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	AND       TEMP_REG1.b2, TEMP_REG1.b2, 0xFB
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	.elseif PWM_SET = 3
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	AND       TEMP_REG1.b2, TEMP_REG1.b2, 0xFB
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	.endif
	.endm


;************************************************************************************
;   Macro:   m_enable_iep_clock_sync
;
;   Description:
;   		Enables the sync mode for the IEP, when enabled core clk is given as input
;           to IEP module, bypassing IEP clk
;
;	Parameters:
;   		TEMP_REG1: temporary register to be used
;
;   PEAK cycles:
;           7 cycles
;************************************************************************************
m_enable_iep_clock_sync				.macro	TEMP_REG1
	lbco        &TEMP_REG1, ICSS_CFG_CONST, ICSS_CFG_IEPCLK, 4
	OR           TEMP_REG1, TEMP_REG1, 1
	sbco		&TEMP_REG1, ICSS_CFG_CONST, ICSS_CFG_IEPCLK, 4
	.endm 

;************************************************************************************
;   Macro:   m_lock_iep1_with_iep0
;
;   Description:
;   		Locks IEP1 with IEP0 counter, IEP1[63:0] is same as IEP0[63:0]
;
;	Parameters:
;   		TEMP_REG1: temporary register to be used
;
;   PEAK cycles:
;           7 cycles
;************************************************************************************
m_lock_iep1_with_iep0				.macro	TEMP_REG1
	lbco        &TEMP_REG1, ICSS_CFG_CONST, ICSS_CFG_IEPCLK, 4
	OR           TEMP_REG1, TEMP_REG1, 2
	sbco		&TEMP_REG1, ICSS_CFG_CONST, ICSS_CFG_IEPCLK, 4
	.endm 

;********
;* MAIN *
;********

main:
;----------------------------------------------------------------------------
;   Clear the register space
;   Before begining with the application, make sure all the registers are set
;   to 0. PRU has 32 - 4 byte registers: R0 to R31, with R30 and R31 being special
;   registers for output and input respectively.
;----------------------------------------------------------------------------
; Give the starting address and number of bytes to clear.
	zero	&r0, 120
	;Intialize DMEM0 base address
	LDI     PRUICSS_PWM_PARAMS_DMEM_ADDR, PWM_PARAMS_DMEM0_OFFSET 
	; disable iep0 timer
	m_set_iep_global_cfg_reg   	0, TEMP_REG1, 0x30
	;clear compare status (0 to 15)
	m_set_iep_cmp_status_reg    0, TEMP_REG1, 0xFFFF
	; disable iep1 timer
	m_set_iep_global_cfg_reg   	1, TEMP_REG1, 0x30
	;clear compare status (0 to 15)
	m_set_iep_cmp_status_reg    1, TEMP_REG1, 0xFFFF

icss_pwm_init:
	;Configure PWMx_B0(-) signal as complimentary signal to PWMx_A0(+)
	ldi     TEMP_REG1,  PWM_A_B_STATES_INIT_VALUE
	;intialize PWM0_A0 and PWM0_B0 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0_0, 4
	;intialize PWM0_A1 and PWM0_B1 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0_1, 4
	;intialize PWM0_A2 and PWM0_B2 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0_2, 4
	;intialize PWM1_A0 and PWM1_B0 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1_0, 4
	;intialize PWM1_A1 and PWM1_B1 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1_1, 4
	;intialize PWM1_A2 and PWM1_B2 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1_2, 4
	;intialize PWM2_A0 and PWM2_B0 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2_0, 4
	;intialize PWM2_A1 and PWM2_B1 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2_1, 4
	;intialize PWM2_A2 and PWM2_B2 channels intial state make signal low, active to toggle, trip states to make signal high
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2_2, 4
	;generate software based trip reset event, to change PWM0, PWM1, PWM2 channels state to intial
	m_generate_software_trip_reset_event		PWM_SET0, TEMP_REG1
	m_generate_software_trip_reset_event		PWM_SET1, TEMP_REG1
	m_generate_software_trip_reset_event		PWM_SET2, TEMP_REG1
	;clear software based trip reset event
	m_clear_software_trip_reset_event			PWM_SET0, TEMP_REG1
	m_clear_software_trip_reset_event			PWM_SET1, TEMP_REG1
	m_clear_software_trip_reset_event			PWM_SET2, TEMP_REG1

icss_iep1_init:
	; set starting count value of IEP0 and IEP1 counter
	m_set_iep_count_reg0       	0, TEMP_REG1, 0x00000000
	m_set_iep_count_reg1       	0, TEMP_REG1, 0x00000000
	m_set_iep_count_reg0       	1, TEMP_REG1, 0x00000000
	m_set_iep_count_reg1       	1, TEMP_REG1, 0x00000000
	;Intialize compare values of axis_x
	zero        &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	LDI			axis_xPwm0CmpLowValue,  0x00000001
	LDI			axis_xPwm1CmpLowValue,  0x00000001
	LDI			axis_xPwm2CmpLowValue,  0x00000001
	LDI			axis_xPwm3CmpLowValue,  0x00000001
	LDI			axis_xPwm4CmpLowValue,  0x00000001
	LDI			axis_xPwm5CmpLowValue,  0x00000001
	;initialize axis 0 compare register values
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;initialize axis 1 compare register values
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco		&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	;initialize axis 2 compare register values
	sbco    	&axis_xPwm0CmpLowValue, ICSS_IEP1_0_CONST,  AXIS2_IEP1_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Initailize scheduler value to schedule compare updates for axis 0
	lbbo    	&SCHEDULER_CMP_LOW_VALUE,  PRUICSS_PWM_PARAMS_DMEM_ADDR,   SCHEDULER_INITIAL_VAL_OFFSET, 4
	LDI		     SCHEDULER_CMP_HIGH_VALUE,  0
	sbco    	&SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8
	;Initialize axis0 - axis 1 increment value
	lbbo    	&AXIS0_AXIS1_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_AXIS1_INC_VAL_OFFSET, 2
	;Initialize axis1 - axis 2 increment value
	lbbo    	&AXIS1_AXIS2_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_AXIS2_INC_VAL_OFFSET, 2
	;Initialize axis2 - axis 0 increment value
	lbbo    	&AXIS2_AXIS0_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_AXIS0_INC_VAL_OFFSET, 2
	;enable compare 1 to 12, disable shadow mode and keep IEP0 counter free running
	m_set_iep_cmp_cfg_reg		IEP_INST0, TEMP_REG1, 0x1FFFE
	;enable compare 1 to 6, disable shadow mode and keep IEP1 counter free running
	m_set_iep_cmp_cfg_reg		IEP_INST1, TEMP_REG1, 0x1FFFE
	;enable sync mode
	m_enable_iep_clock_sync		TEMP_REG1	
	;Lock IEP 1 with IEP0
	m_lock_iep1_with_iep0		TEMP_REG1
	;enable IEP0, with increment as 3
	m_set_iep_global_cfg_reg   	IEP_INST0, TEMP_REG1, 0x31

;******************************************************
;Configure even axis 0 compare up values
;******************************************************
config_axis0_compare_up_increment_values:
	;set compare up configured flag of all pwms of axis_x
	LDI32	axis_xCmpUpConfigured, AXIS_X_CMP_UP_CONFIGURED_VALUE
	;load axis 0 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_and_clear_flag		axis0PwmxCmpUpConfigured
	;increment scheduler low value by axis 0 and axis 1 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS0_AXIS1_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP0 compare 1 to 6 register values of IEP0 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure odd axis (1) compare up values
;******************************************************
config_axis1_compare_up_increment_values:
	;load axis 1 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_and_clear_flag		axis1PwmxCmpUpConfigured
	;increment scheduler low value by axis 1 and axis 2 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS1_AXIS2_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP0 compare 7 to 12 register values of axis 1 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco	&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis 2 compare up values
;******************************************************
config_axis2_compare_up_increment_values:
	;load axis 2 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 2 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_and_clear_flag		axis2PwmxCmpUpConfigured
	;increment scheduler low value by axis 2 and axis 0 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS2_AXIS0_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP1 compare 1 to 6 register values of axis 2 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, ICSS_IEP1_0_CONST, AXIS2_IEP1_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis 0 compare down values
;******************************************************
config_axis0_compare_down_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 0 compare down increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_set		axis0PwmxCmpUpConfigured
	;increment scheduler low value by axis 0 and axis 1 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS0_AXIS1_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP0 compare 1 to 6 register values of IEP0 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,	AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST, SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis (1) compare down values
;******************************************************
config_axis1_compare_down_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 1 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 1 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_set		axis1PwmxCmpUpConfigured
	;increment scheduler low value by axis 1 and axis 2 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS1_AXIS2_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP0 compare 7 to 12 register values of axis 1 at once which also clears compare status
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco		&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE, SCHEDULER_IEP_CONST,	SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis 2 compare down values
;******************************************************
config_axis2_compare_down_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 2 compare down increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 2 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_set		axis2PwmxCmpUpConfigured
	;increment scheduler low value by axis 2 and axis 0 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS2_AXIS0_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_schedular_compare_event_is_hit
	;Update IEP1 compare 1 to 6 register values of IEP1 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, ICSS_IEP1_0_CONST, AXIS2_IEP1_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST, SCHEDULER_CMP_REG_OFFSET_VALUE, 8
	;jump back to update axis 0 compare down increment values
	qba	config_axis0_compare_up_increment_values

