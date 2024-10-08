
;
; Copyright (C) 2023 Texas Instruments Incorporated
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
;   Redistributions of source code must retain the above copyright
;   notice, this list of conditions and the following disclaimer.
;
;   Redistributions in binary form must reproduce the above copyright
;   notice, this list of conditions and the following disclaimer in the
;   documentation and/or other materials provided with the
;   distribution.
;
;   Neither the name of Texas Instruments Incorporated nor the names of
;   its contributors may be used to endorse or promote products derived
;   from this software without specific prior written permission.
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
;

;************************************************************************************
;*   File:     bissc_icss_reg_defs.h                                                *
;*                                                                                  *
;*   Brief:    Defining R30, R31 register bits as specific status bits for different*
;*		       channels.  							    							*
;*             Defining the Clock Modes with their respective values 		    	*
;************************************************************************************

	.include "../../../../mcu_plus_sdk/source/pru_io/firmware/common/icss_regs.inc"
	.include "../../../../mcu_plus_sdk/source/pru_io/firmware/common/icss_cfg_regs.inc"

	.asg	R30.t24,	BISSC_CH0_RX_EN
	.asg	R31.t27,	BISSC_CH0_RX_CLR_OVF
	.asg	R31.t24,	BISSC_CH0_RX_CLR_VALID
	.asg	R31.t27,	BISSC_CH0_RX_OVF
	.asg	R31.t24,	BISSC_CH0_RX_VALID

	.asg	R30.t25,	BISSC_CH1_RX_EN
	.asg	R31.t28,	BISSC_CH1_RX_CLR_OVF
	.asg	R31.t25,	BISSC_CH1_RX_CLR_VALID
	.asg	R31.t28,	BISSC_CH1_RX_OVF
	.asg	R31.t25,	BISSC_CH1_RX_VALID

	.asg	R30.t26,	BISSC_CH2_RX_EN
	.asg	R31.t29,	BISSC_CH2_RX_CLR_OVF
	.asg	R31.t26,	BISSC_CH2_RX_CLR_VALID
	.asg	R31.t29,	BISSC_CH2_RX_OVF
	.asg	R31.t26,	BISSC_CH2_RX_VALID

	.asg	R31.t18,	BISSC_TX_CHANNEL_GO
	.asg	R31.t19,	BISSC_TX_GLOBAL_REINIT
	.asg	R31.t20,	BISSC_TX_GLOBAL_GO

	.asg	R31.t5,		BISSC_CH0_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t5,		BISSC_CH0_TX_BUSY
	.asg	R31.t0,		BISSC_CH0_TX_OVERUN
	.asg	R31.t1,		BISSC_CH0_TX_UNDERRUN

	.asg	R31.t13,	BISSC_CH1_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t13,	BISSC_CH1_TX_BUSY
	.asg	R31.t8,		BISSC_CH1_TX_OVERUN
	.asg	R31.t9,		BISSC_CH1_TX_UNDERRUN

	.asg	R31.t21,	BISSC_CH2_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t21,	BISSC_CH2_TX_BUSY
	.asg	R31.t16,	BISSC_CH2_TX_OVERUN
	.asg	R31.t17,	BISSC_CH2_TX_UNDERRUN

; Channel select bits R30[17:16]
BISSC_TX_CH0_SEL					.set	0
BISSC_TX_CH1_SEL					.set	1
BISSC_TX_CH2_SEL					.set	2

; CLK MODE bits R30[20:19]
BISSC_TX_CLK_MODE_FREERUN_STOPLOW	.set	(0 << 3)
BISSC_TX_CLK_MODE_FREERUN_STOPHIGH	.set	(1 << 3)
BISSC_TX_CLK_MODE_FREERUN			.set	(2 << 3)
BISSC_TX_CLK_MODE_STOPHIGH_AFTER_TX	.set	(3 << 3)

; BISSC
