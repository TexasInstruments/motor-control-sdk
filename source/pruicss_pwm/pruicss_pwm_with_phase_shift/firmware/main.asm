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
    ;R2 - R7 		s_axis_xPwmCmpUpOrDownIncrementValues
    .asg    R8,     axis_xPwm0CmpLowValue
    .asg    R9,     axis_xPwm0CmpHighValue
    .asg    R10,     axis_xPwm1CmpLowValue
    .asg    R11,     axis_xPwm1CmpHighValue
    .asg    R12,     axis_xPwm2CmpLowValue
    .asg    R13,    axis_xPwm2CmpHighValue
    .asg    R14,    axis_xPwm3CmpLowValue
    .asg    R15,    axis_xPwm3CmpHighValue
    .asg    R16,    axis_xPwm4CmpLowValue
    .asg    R17,    axis_xPwm4CmpHighValue
    .asg    R18,    axis_xPwm5CmpLowValue
    .asg    R19,    axis_xPwm5CmpHighValue
    ;IEP0_CMP13 used as even axis scheduler
    .asg    R20,    SCHEDULER_CMP_LOW_VALUE
    .asg    R21,    SCHEDULER_CMP_HIGH_VALUE
	;Increment values after updating compare values axis 1 to axis 3
	.asg    R22, AXIS0_AXIS1_INC_VALUE
	.asg    R23, AXIS1_AXIS2_INC_VALUE
	.asg    R24, AXIS2_AXIS0_INC_VALUE
    ;Flag to check whether compare up value configured or not
    .asg    R25,    axis_xSkipCmpUpAndDown
    .asg	R25.b0, axis0PwmxSkipCmpUpAndDown
    .asg	R25.b1, axis1PwmxSkipCmpUpAndDown
    .asg	R25.b2, axis2PwmxSkipCmpUpAndDown
    .asg    "R26.b0 , 0", axis0Pwm0SkipCmpUpAndDown
    .asg    "R26.b0 , 1", axis0Pwm1SkipCmpUpAndDown
    .asg    "R26.b0 , 2", axis0Pwm2SkipCmpUpAndDown
    .asg    "R26.b0 , 3", axis0Pwm3SkipCmpUpAndDown
    .asg    "R26.b0 , 4", axis0Pwm4SkipCmpUpAndDown
    .asg    "R26.b0 , 5", axis0Pwm5SkipCmpUpAndDown
    .asg    "R26.b1 , 0", axis1Pwm0SkipCmpUpAndDown
    .asg    "R26.b1 , 1", axis1Pwm1SkipCmpUpAndDown
    .asg    "R26.b1 , 2", axis1Pwm2SkipCmpUpAndDown
    .asg    "R26.b1 , 3", axis1Pwm3SkipCmpUpAndDown
    .asg    "R26.b1 , 4", axis1Pwm4SkipCmpUpAndDown
    .asg    "R26.b1 , 5", axis1Pwm5SkipCmpUpAndDown
    .asg    "R26.b2 , 0", axis2Pwm0SkipCmpUpAndDown
    .asg    "R26.b2 , 1", axis2Pwm1SkipCmpUpAndDown
    .asg    "R26.b2 , 2", axis2Pwm2SkipCmpUpAndDown
    .asg    "R26.b2 , 3", axis2Pwm3SkipCmpUpAndDown
    .asg    "R26.b2 , 4", axis2Pwm4SkipCmpUpAndDown
    .asg    "R26.b2 , 5", axis2Pwm5SkipCmpUpAndDown
	.asg     R27,     	  PRUICSS_PWM_PARAMS_DMEM_ADDR

;************************ IEP Resources allocation ********************************
;IEP compare registers usage.
;IEP0_CMP1 to IEP0_CMP12 used by axis 0 and axis 1.
;IEP1_CMP1 to IEP1_CMP6 used by axis 2.
;IEP0_CMP13 is used to update compare values from axis 0 to 2.
;**********************************************************************************

;6 byte read from DMEM takes  (2 + 2) cycles
s_axis_xPwmCmpValues    	 			.struct
axisPwm0CmpUpOrDownIncVal		.ulong
axisPwm1CmpUpOrDownIncVal		.ulong
axisPwm2CmpUpOrDownIncVal		.ulong
axisPwm3CmpUpOrDownIncVal		.ulong
axisPwm4CmpUpOrDownIncVal		.ulong
axisPwm5CmpUpOrDownIncVal		.ulong
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
PWM_PARAMS_DMEM0_OFFSET                         .set    0x600
AXIS0_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x0
AXIS0_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0x18
AXIS1_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x30
AXIS1_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0x48
AXIS2_CMP_UP_INC_VAL_DMEM0_OFFSET				.set	0x60
AXIS2_CMP_DOWN_INC_VAL_DMEM0_OFFSET				.set	0x78
AXIS0_AXIS1_INC_VAL_OFFSET						.set	0x90
AXIS1_AXIS2_INC_VAL_OFFSET						.set	0x94
AXIS2_AXIS0_INC_VAL_OFFSET						.set	0x98
SCHEDULER_INITIAL_VAL_OFFSET					.set	0x9C
STOP_AXIS_X_PWM_OFFSET							.set    0xA0
OFF_PWM_OFFSET									.set    0xA4
WAIT_TO_EXIT_FROM_OFFSTATE_OFFSET				.set    0xA5

;********************************************************************************
;Firmware version
;********************************************************************************
;  bit31		release or internal version
FIRMWARE_VERSION_RELEASE	.set	1
FIRMWARE_VERSION_INTERNAL	.set	0
;  bit30..24		version number
FIRMWARE_VERSION_REVISION	.set		0x01
;  bit23..16		major number
FIRMWARE_VERSION_MAJOR	.set			0x00
;  bit15..0		minor number
FIRMWARE_VERSION_MINOR	.set			0x0000
;  Release version
ICSS_FIRMWARE_RELEASE	.set ((FIRMWARE_VERSION_INTERNAL << 31) | (FIRMWARE_VERSION_REVISION << 24) | (FIRMWARE_VERSION_MAJOR << 16) | (FIRMWARE_VERSION_MINOR << 0))

;********************************************************************************
;Additional IEP & PWM macros
;********************************************************************************
IEP_INST0														.set    0
IEP_INST1														.set    1
PWM_SET0														.set    0
PWM_SET1														.set    1
PWM_SET2														.set    2
PWM_SET3														.set    3
PWM_0                                               			.set    0
PWM_1                                               			.set    1
PWM_2                                               			.set    2
PWM_3                                               			.set    3
PWM_4                                               			.set    4
PWM_5                                               			.set    5
AXIS_X_SKIP_CMP_UP_AND_DOWN_INIT_VAL                			.set    0x00000000
FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT		.set    0x1

;************************************************************************************
;   Macro: m_wait_until_scheduler_compare_event_is_hit
;
;   Description : Wait until scheduler compare event is set
;
;   Parameters: None
;
; 	Pseudo code:
; 				 while (!(IEP_CMP_STATUS_REG & (1 << SCHEDULER_CMP_EVENT))) {
;   			   // Wait until the bit is set
; 				 }	
;
;   PEAK cycles:
;           4 cycles
;************************************************************************************
m_wait_until_scheduler_compare_event_is_hit	.macro
wait?:
    lbco    &TEMP_REG1, SCHEDULER_IEP_CONST, ICSS_IEP_CMP_STATUS_REG, 2
    qbbc    wait?, TEMP_REG1, SCHEDULER_CMP_EVENT
    .endm

;************************************************************************************
;   Macro: m_compute_axis_x_compare_up_values_when_flag_clear
;
;   Description :
;           macro used to compute compare up values when flag is clear
;
;   Parameters:
;           axis_x_pwm_x_flag - axis_x pwm flag
;
; 	Pseudo code:
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_0))) {
; 			    axis_xPwm0CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal;
; 			    axis_xPwm0CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_1))) {
; 			    axis_xPwm1CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal;
; 			    axis_xPwm1CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_2))) {
; 			    axis_xPwm2CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal;
; 			    axis_xPwm2CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_3))) {
; 			    axis_xPwm3CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal;
; 			    axis_xPwm3CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_4))) {
; 			    axis_xPwm4CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal;
; 			    axis_xPwm4CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_5))) {
; 			    axis_xPwm5CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal;
; 			    axis_xPwm5CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
;
;   PEAK cycles:
;           30 cycles(5 cycles per PWM signal)
;************************************************************************************
m_compute_axis_x_compare_up_values_when_flag_clear		.macro	axis_x_pwm_x_flag
	;if pwm0 flag is set then skip updating compare up value
	QBBS	skip_axisx_0_up?, axis_x_pwm_x_flag, PWM_0
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm0CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm0CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_0_up?:
	;if pwm1 flag is set then skip updating compare up value
	QBBS	skip_axisx_1_up?, axis_x_pwm_x_flag, PWM_1
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm1CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm1CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_1_up?:
	;if pwm2 flag is set then skip updating compare up value
	QBBS	skip_axisx_2_up?, axis_x_pwm_x_flag, PWM_2
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm2CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm2CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_2_up?:
	;if pwm3 flag is set then skip updating compare up value
	QBBS	skip_axisx_3_up?, axis_x_pwm_x_flag, PWM_3
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm3CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm3CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_3_up?:
	;if pwm4 flag is set then skip updating compare up value
	QBBS	skip_axisx_4_up?, axis_x_pwm_x_flag, PWM_4
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm4CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm4CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_4_up?:
	;if pwm5 flag is set then skip updating compare up value
	QBBS	skip_axisx_5_up?, axis_x_pwm_x_flag, PWM_5
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm5CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm5CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_5_up?:
	.endm

;******************************************************
;	Macro: m_compute_axis_x_compare_down_values_when_flag_clear
;
;	Description :
;			Computes axis compare down values when flag is clear
;
;	Parameters:
;   		axis_x_pwm_x_flag: flag to check if compare up value is configured or not
;
; 	Pseudo code:
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_0))) {
; 			    axis_xPwm0CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal;
; 			    axis_xPwm0CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_1))) {
; 			    axis_xPwm1CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal;
; 			    axis_xPwm1CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_2))) {
; 			    axis_xPwm2CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal;
; 			    axis_xPwm2CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_3))) {
; 			    axis_xPwm3CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal;
; 			    axis_xPwm3CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_4))) {
; 			    axis_xPwm4CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal;
; 			    axis_xPwm4CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
; 			if (!(axis_x_pwm_x_flag & (1 << PWM_5))) {
; 			    axis_xPwm5CmpLowValue = SCHEDULER_CMP_LOW_VALUE + axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal;
; 			    axis_xPwm5CmpHighValue = SCHEDULER_CMP_HIGH_VALUE + carry;
; 			}
;
;   PEAK cycles:
;           18 cycles(3 cycles per PWM signal)
;******************************************************
m_compute_axis_x_compare_down_values_when_flag_clear	.macro	axis_x_pwm_x_flag
	;if pwm0 flag is set then skip updating compare down value
	QBBS	skip_axisx_0_down?, axis_x_pwm_x_flag, PWM_0
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm0CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm0CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_0_down?:
	;if pwm1 flag is set then skip updating compare down value
	QBBS	skip_axisx_1_down?, axis_x_pwm_x_flag, PWM_1
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm1CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm1CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm1CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_1_down?:
	;if pwm2 flag is set then skip updating compare down value
	QBBS	skip_axisx_2_down?, axis_x_pwm_x_flag, PWM_2
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm2CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm2CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm2CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_2_down?:
	;if pwm3 flag is set then skip updating compare down value
	QBBS	skip_axisx_3_down?, axis_x_pwm_x_flag, PWM_3
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm3CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm3CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm3CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_3_down?:
	;if pwm4 flag is set then skip updating compare down value
	QBBS	skip_axisx_4_down?, axis_x_pwm_x_flag, PWM_4
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm4CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm4CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm4CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_4_down?:
	;if pwm5 flag is set then skip updating compare down value
	QBBS	skip_axisx_5_down?, axis_x_pwm_x_flag, PWM_5
	;Change IEP compare low reg values with compare down increment value + scheduler value
	ADD		axis_xPwm5CmpLowValue, SCHEDULER_CMP_LOW_VALUE, axis_xPwmCmpValuesITR.axisPwm5CmpUpOrDownIncVal
	;add carry value to compare high values
	ADC     axis_xPwm5CmpHighValue, SCHEDULER_CMP_HIGH_VALUE, 0
skip_axisx_5_down?:
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
;	Pseudo code:
;	 		if (PWM_SET == 0) {
;	 		    TEMP_REG1 = read from ICSS_CFG_1_PWM0;
;	 		    TEMP_REG1.b2 |= 0x4;
;	 		    write TEMP_REG1 to ICSS_CFG_1_PWM0;
;	 		} else if (PWM_SET == 1) {
;	 		    TEMP_REG1 = read from ICSS_CFG_1_PWM1;
;	 		    TEMP_REG1.b2 |= 0x4;
;	 		    write TEMP_REG1 to ICSS_CFG_1_PWM1;
;	 		} else if (PWM_SET == 2) {
;	 		    TEMP_REG1 = read from ICSS_CFG_1_PWM2;
;	 		    TEMP_REG1.b2 |= 0x4;
;	 		    write TEMP_REG1 to ICSS_CFG_1_PWM2;
;	 		} else if (PWM_SET == 3) {
;	 		    TEMP_REG1 = read from ICSS_CFG_1_PWM3;
;	 		    TEMP_REG1.b2 |= 0x4;
;	 		    write TEMP_REG1 to ICSS_CFG_1_PWM3;
;	 		}
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

;***************************************************************************
;	Macro: m_clear_position_error_trip
;
;	Description :
;			clear position error but don't changes pwm from trip state, this is requied
;			to generate trip reset and pwm to initial
;
;	Parameters:
;   		PWM_SET: PWM to set trip/reset event
;			TEMP_REG1: temporary register to be used
;
; 	Pseudo code:
; 			if (PWM_SET == 0) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM0;
; 			    TEMP_REG1.b2 &= 0xFB;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM0;
; 			} else if (PWM_SET == 1) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM1;
; 			    TEMP_REG1.b2 &= 0xFB;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM1;
; 			} else if (PWM_SET == 2) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM2;
; 			    TEMP_REG1.b2 &= 0xFB;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM2;
; 			} else if (PWM_SET == 3) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM3;
; 			    TEMP_REG1.b2 &= 0xFB;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM3;
; 			}
;
;   PEAK cycles:
;           7 cycles
;****************************************************************************
m_clear_position_error_trip		.macro PWM_SET, TEMP_REG1
	.if   PWM_SET = 0
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	CLR       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	.elseif PWM_SET = 1
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	CLR       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	.elseif PWM_SET = 2
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	CLR       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	.elseif PWM_SET = 3
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	CLR       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	.endif
	.endm

;***************************************************************************
;	Macro: m_generate_position_error_trip
;
;	Description:
;			Generates position error and changes pwm to trip status
;
;	Parameters:
;   		PWM_SET: PWM to set trip/reset event
;			TEMP_REG1: temporary register to be used
;
; 	Pseudo code:
; 			if (PWM_SET == 0) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM0;
; 			    SET bit 8 of TEMP_REG1;
; 			    SET bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM0;
; 			} else if (PWM_SET == 1) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM1;
; 			    SET bit 8 of TEMP_REG1;
; 			    SET bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM1;
; 			} else if (PWM_SET == 2) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM2;
; 			    SET bit 8 of TEMP_REG1;
; 			    SET bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM2;
; 			} else if (PWM_SET == 3) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM3;
; 			    SET bit 8 of TEMP_REG1;
; 			    SET bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM3;
; 			}
;
;   PEAK cycles:
;           7 cycles
;****************************************************************************
m_generate_position_error_trip		.macro PWM_SET, TEMP_REG1
	.if   PWM_SET = 0
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	SET       TEMP_REG1, TEMP_REG1, 8
	SET       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM0, 4
	.elseif PWM_SET = 1
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	SET       TEMP_REG1, TEMP_REG1, 8
	SET       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM1, 4
	.elseif PWM_SET = 2
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	SET       TEMP_REG1, TEMP_REG1, 8
	SET       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM2, 4
	.elseif PWM_SET = 3
	lbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	SET       TEMP_REG1, TEMP_REG1, 8
	SET       TEMP_REG1, TEMP_REG1, 20
	sbco    &TEMP_REG1, ICSS_CFG_1_CONST, ICSS_CFG_1_PWM3, 4
	.endif
	.endm

;************************************************************************************
;   Macro:   m_clear_software_trip_reset_event
;
;   Description:
;   		Clears software trip/reset event for the specified PWM
;
; 	Pseudo code:
; 			if (PWM_SET == 0) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM0;
; 			    CLR bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM0;
; 			} else if (PWM_SET == 1) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM1;
; 			    CLR bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM1;
; 			} else if (PWM_SET == 2) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM2;
; 			    CLR bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM2;
; 			} else if (PWM_SET == 3) {
; 			    TEMP_REG1 = read from ICSS_CFG_1_PWM3;
; 			    CLR bit 20 of TEMP_REG1;
; 			    write TEMP_REG1 to ICSS_CFG_1_PWM3;
; 			}
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
; 	Pseudo code:
; 			TEMP_REG1 = read from ICSS_CFG_IEPCLK;
; 			TEMP_REG1 |= 1;
; 			write TEMP_REG1 to ICSS_CFG_IEPCLK;
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
; 	Pseudo code:
; 			TEMP_REG1 = read from ICSS_CFG_IEPCLK;
; 			TEMP_REG1 |= 2;
; 			write TEMP_REG1 to ICSS_CFG_IEPCLK;
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
	.word 	ICSS_FIRMWARE_RELEASE
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

icss_iep1_init:
	; set starting count value of IEP0 and IEP1 counter
	m_set_iep_count_reg0       	0, TEMP_REG1, 0x00000000
	m_set_iep_count_reg1       	0, TEMP_REG1, 0x00000000
	m_set_iep_count_reg0       	1, TEMP_REG1, 0x00000000
	m_set_iep_count_reg1       	1, TEMP_REG1, 0x00000000
	;Initialize axis0 - axis 1 increment value
	lbbo    	&AXIS0_AXIS1_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_AXIS1_INC_VAL_OFFSET, 4
	;Initialize axis1 - axis 2 increment value
	lbbo    	&AXIS1_AXIS2_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_AXIS2_INC_VAL_OFFSET, 4
	;Initialize axis2 - axis 0 increment value
	lbbo    	&AXIS2_AXIS0_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_AXIS0_INC_VAL_OFFSET, 4
	;enable compare 1 to 12, disable shadow mode and keep IEP0 counter free running
	m_set_iep_cmp_cfg_reg		IEP_INST0, TEMP_REG1, 0x1FFFE
	;enable compare 1 to 6, disable shadow mode and keep IEP1 counter free running
	m_set_iep_cmp_cfg_reg		IEP_INST1, TEMP_REG1, 0x1FFFE
	;enable sync mode
	m_enable_iep_clock_sync		TEMP_REG1	
	;Lock IEP 1 with IEP0
	m_lock_iep1_with_iep0		TEMP_REG1
	;Initialize axis0 - axis 1 increment value
	lbbo    	&AXIS0_AXIS1_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_AXIS1_INC_VAL_OFFSET, 4
	;Initialize axis1 - axis 2 increment value
	lbbo    	&AXIS1_AXIS2_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_AXIS2_INC_VAL_OFFSET, 4
	;Initialize axis2 - axis 0 increment value
	lbbo    	&AXIS2_AXIS0_INC_VALUE, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_AXIS0_INC_VAL_OFFSET, 4
go_to_trip_state:
	;generate position error
	m_generate_position_error_trip		PWM_SET0, TEMP_REG1
	m_generate_position_error_trip		PWM_SET1, TEMP_REG1
	m_generate_position_error_trip		PWM_SET2, TEMP_REG1
	m_clear_position_error_trip			PWM_SET0, TEMP_REG1
	m_clear_position_error_trip			PWM_SET1, TEMP_REG1
	m_clear_position_error_trip			PWM_SET2, TEMP_REG1
	;let R5F know, that PWM signals are ready to move from off state
	LDI      TEMP_REG1,  0
	sbbo	&TEMP_REG1, PRUICSS_PWM_PARAMS_DMEM_ADDR, OFF_PWM_OFFSET, 1
	;wait until command from R5F is given to regenerate PWM signals
wait_to_regenerate_pwm_signals:
	lbbo	&TEMP_REG1, PRUICSS_PWM_PARAMS_DMEM_ADDR, WAIT_TO_EXIT_FROM_OFFSTATE_OFFSET, 1
	qbeq     wait_to_regenerate_pwm_signals, TEMP_REG1.b0, 0
go_initial_state:
	;generate software based trip reset event, to change PWM0, PWM1, PWM2 channels state to intial
	m_generate_software_trip_reset_event		PWM_SET0, TEMP_REG1
	m_generate_software_trip_reset_event		PWM_SET1, TEMP_REG1
	m_generate_software_trip_reset_event		PWM_SET2, TEMP_REG1
	;clear software based trip reset event
	m_clear_software_trip_reset_event			PWM_SET0, TEMP_REG1
	m_clear_software_trip_reset_event			PWM_SET1, TEMP_REG1
	m_clear_software_trip_reset_event			PWM_SET2, TEMP_REG1

load_iep_counter_value:
	;load counter high value
	lbco	  &TEMP_REG1, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG + 4, 4
	;load counter low value and load counter high value again
	lbco      &SCHEDULER_CMP_LOW_VALUE, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 8
	qbeq      skip_reloading, SCHEDULER_CMP_HIGH_VALUE, TEMP_REG1
	;reload counter high and counter low values again
	lbco      &SCHEDULER_CMP_LOW_VALUE, ICSS_IEP_CONST, ICSS_IEP_COUNT_REG, 8
skip_reloading:
	;read scheduler value to schedule compare updates for axis 0
	lbbo    &TEMP_REG1,  PRUICSS_PWM_PARAMS_DMEM_ADDR,   SCHEDULER_INITIAL_VAL_OFFSET, 4
	;add IEP counter value and update schedular value
	ADD		SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_CMP_LOW_VALUE,  TEMP_REG1
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	sbco    &SCHEDULER_CMP_LOW_VALUE, SCHEDULER_IEP_CONST,	    SCHEDULER_CMP_REG_OFFSET_VALUE, 8
	;intialize axis_xSkipCmppUpAndDown value
	LDI32	axis_xSkipCmpUpAndDown, AXIS_X_SKIP_CMP_UP_AND_DOWN_INIT_VAL
	;when ever software based trip reset event is generated to change pwm signals to initial state,
	;the first compare hit is not making PWM's to go into active state this is observed only when IEP is running
	;Below code fixes it 
	;TODO: need to find reason and remove below
	MOV			axis_xPwm0CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm1CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm2CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm3CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm4CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm5CmpLowValue,   SCHEDULER_CMP_LOW_VALUE
	MOV			axis_xPwm0CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	MOV			axis_xPwm1CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	MOV			axis_xPwm2CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	MOV			axis_xPwm3CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	MOV			axis_xPwm4CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	MOV			axis_xPwm5CmpHighValue,  SCHEDULER_CMP_HIGH_VALUE
	;initialize axis 0 compare register values
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;initialize axis 1 compare register values
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco		&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	;initialize axis 2 compare register values
	sbco    	&axis_xPwm0CmpLowValue, ICSS_IEP1_0_CONST,  AXIS2_IEP1_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE

;******************************************************
;Configure even axis 0 compare up values
;******************************************************
config_axis0_compare_up_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;if pwm signals should be move to off state then reset firmware
	lbbo	&TEMP_REG1, PRUICSS_PWM_PARAMS_DMEM_ADDR, OFF_PWM_OFFSET, 1
	qbeq    go_to_trip_state, TEMP_REG1.b0, 1
	.if     FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT = 0
	;load axis_xSkipCmppUpAndDown value from STOP_AXIS_X_PWM_OFFSET to decide whether to stop or run PWM signals
	lbbo	&axis_xSkipCmpUpAndDown, PRUICSS_PWM_PARAMS_DMEM_ADDR, STOP_AXIS_X_PWM_OFFSET, 4
	.endif
	;load axis 0 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_when_flag_clear		axis0PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 0 and axis 1 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS0_AXIS1_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
	;Update IEP0 compare 1 to 6 register values of IEP0 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure odd axis (1) compare up values
;******************************************************
config_axis1_compare_up_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 1 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_when_flag_clear		axis1PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 1 and axis 2 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS1_AXIS2_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
	;Update IEP0 compare 7 to 12 register values of axis 1 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco	&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis 2 compare up values
;******************************************************
config_axis2_compare_up_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 2 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_CMP_UP_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 2 compare up value and clear flag accordingly
	m_compute_axis_x_compare_up_values_when_flag_clear		axis2PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 2 and axis 0 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS2_AXIS0_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
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
	;if pwm signals should be move to off state then reset firmware
	lbbo	&TEMP_REG1, PRUICSS_PWM_PARAMS_DMEM_ADDR, OFF_PWM_OFFSET, 1
	qbeq    go_to_trip_state, TEMP_REG1.b0, 1
	.if     FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT = 1
	;load axis_xSkipCmppUpAndDown value from STOP_AXIS_X_PWM_OFFSET to decide whether to stop or run PWM signals
	lbbo	&axis_xSkipCmpUpAndDown, PRUICSS_PWM_PARAMS_DMEM_ADDR, STOP_AXIS_X_PWM_OFFSET, 4
	.endif
	;load axis 0 compare down increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS0_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 0 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_clear		axis0PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 0 and axis 1 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS0_AXIS1_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
	;Update IEP0 compare 1 to 6 register values of IEP0 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,	AXIS0_IEP0_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis (1) compare down values
;******************************************************
config_axis1_compare_down_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 1 compare up increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS1_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 1 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_clear		axis1PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 1 and axis 2 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS1_AXIS2_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
	;Update IEP0 compare 7 to 12 register values of axis 1 at once which also clears compare status
	sbco    	&axis_xPwm0CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET, 8
	sbco		&axis_xPwm1CmpLowValue, SCHEDULER_IEP_CONST,		AXIS1_IEP0_CMP_VALUES_OFFSET + 16, AXISX_CMP_REG_VALUES_SIZE - 8
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST,		SCHEDULER_CMP_REG_OFFSET_VALUE, 8

;******************************************************
;Configure even axis 2 compare down values
;******************************************************
config_axis2_compare_down_increment_values:
	;clear compare low and high values
	zero    &axis_xPwm0CmpLowValue, AXISX_CMP_REG_VALUES_SIZE
	;load axis 2 compare down increment values from DMEM offset
	lbbo	&axis_xPwmCmpValuesITR.axisPwm0CmpUpOrDownIncVal, PRUICSS_PWM_PARAMS_DMEM_ADDR, AXIS2_CMP_DOWN_INC_VAL_DMEM0_OFFSET, s_axispwmCmpValues_len
	;Compute axis 2 compare down values when flag is set
	m_compute_axis_x_compare_down_values_when_flag_clear		axis2PwmxSkipCmpUpAndDown
	;increment scheduler low value by axis 2 and axis 0 increment value
	ADD		SCHEDULER_CMP_LOW_VALUE, SCHEDULER_CMP_LOW_VALUE, AXIS2_AXIS0_INC_VALUE
	;add carry value to scheduler high value
	ADC		SCHEDULER_CMP_HIGH_VALUE, SCHEDULER_CMP_HIGH_VALUE, 0
	;Wait until scheduler compare event is hit
	m_wait_until_scheduler_compare_event_is_hit
	;Update IEP1 compare 1 to 6 register values of IEP1 of axis 0 at once which also clears compare status
	sbco    &axis_xPwm0CmpLowValue, ICSS_IEP1_0_CONST, AXIS2_IEP1_CMP_VALUES_OFFSET, AXISX_CMP_REG_VALUES_SIZE
	;Update scheduler compare value which will clear compare status
	sbco    &SCHEDULER_CMP_LOW_VALUE,  SCHEDULER_IEP_CONST, SCHEDULER_CMP_REG_OFFSET_VALUE, 8
	;jump back to update axis 0 compare down increment values
	qba	config_axis0_compare_up_increment_values

