; Copyright (C) 2022-2025 Texas Instruments Incorporated
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

	.if	!$isdefed("__tamagawa_icss_reg_defs_h")
__tamagawa_icss_reg_defs_h	.set	1

	.include "pru_io/firmware/common/icss_regs.inc"
	.include "pru_io/firmware/common/icss_cfg_regs.inc"

	.asg	R30.t24,	TAMAGAWA_CH0_RX_EN
	.asg	R31.t27,	TAMAGAWA_CH0_RX_CLR_OVF
	.asg	R31.t24,	TAMAGAWA_CH0_RX_CLR_VALID
	.asg	R31.t27,	TAMAGAWA_CH0_RX_OVF
	.asg	R31.t24,	TAMAGAWA_CH0_RX_VALID

	.asg	R30.t25,	TAMAGAWA_CH1_RX_EN
	.asg	R31.t28,	TAMAGAWA_CH1_RX_CLR_OVF
	.asg	R31.t25,	TAMAGAWA_CH1_RX_CLR_VALID
	.asg	R31.t28,	TAMAGAWA_CH1_RX_OVF
	.asg	R31.t25,	TAMAGAWA_CH1_RX_VALID

	.asg	R30.t26,	TAMAGAWA_CH2_RX_EN
	.asg	R31.t29,	TAMAGAWA_CH2_RX_CLR_OVF
	.asg	R31.t26,	TAMAGAWA_CH2_RX_CLR_VALID
	.asg	R31.t29,	TAMAGAWA_CH2_RX_OVF
	.asg	R31.t26,	TAMAGAWA_CH2_RX_VALID


; Channel select bits R30[17:16]
TAMAGAWA_TX_CH0_SEL	.set					0
TAMAGAWA_TX_CH1_SEL	.set					1
TAMAGAWA_TX_CH2_SEL	.set					2


	.asg	R31.t18,	TAMAGAWA_TX_CHANNEL_GO
	.asg	R31.t19,	TAMAGAWA_TX_GLOBAL_REINIT
	.asg	R31.t20,	TAMAGAWA_TX_GLOBAL_GO

	.asg	R31.t5,	TAMAGAWA_CH0_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t5,	TAMAGAWA_CH0_TX_BUSY
	.asg	R31.t0,	TAMAGAWA_CH0_TX_OVERUN
	.asg	R31.t1,	TAMAGAWA_CH0_TX_UNDERRUN

	.asg	R31.t13,	TAMAGAWA_CH1_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t13,	TAMAGAWA_CH1_TX_BUSY
	.asg	R31.t8,	TAMAGAWA_CH1_TX_OVERUN
	.asg	R31.t9,	TAMAGAWA_CH1_TX_UNDERRUN

	.asg	R31.t21,	TAMAGAWA_CH2_TX_GLOBAL_REINIT_ACTIVE
	.asg	R31.t21,	TAMAGAWA_CH2_TX_BUSY
	.asg	R31.t16,	TAMAGAWA_CH2_TX_OVERUN
	.asg	R31.t17,	TAMAGAWA_CH2_TX_UNDERRUN


TAMAGAWA_RTU_PRU_BIT_ID			   .set  0
TAMAGAWA_PRU_BIT_ID			   	   .set  1
TAMAGAWA_TX_PRU_BIT_ID			   .set  2

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
TAMAGAWA_CHANNEL_BIT_ID			   .set	 TAMAGAWA_RTU_PRU_BIT_ID 
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
TAMAGAWA_CHANNEL_BIT_ID			   .set	 TAMAGAWA_PRU_BIT_ID 
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
TAMAGAWA_CHANNEL_BIT_ID			   .set	 TAMAGAWA_TX_PRU_BIT_ID 
	.endif

;=============================================================================
; PRU Event Numbers for Host Interrupt Triggering 
;=============================================================================
; These event numbers are used by PRU firmware to trigger host interrupts
; after completing encoder operations in periodic mode.
;
; Event number mapping follows the same pattern as EnDAT:
; - SLICE1 Multi-PRU: RTU=34, PRU=35, TXPRU=36
; - SLICE1 Single-PRU: PRU=34
; - SLICE0 Multi-PRU: RTU=37, PRU=38, TXPRU=39
; - SLICE0 Single-PRU: PRU=37
;
; Application side uses (event - 16) for R5F interrupt mapping
;=============================================================================

	.if	$isdefed("SLICE1")
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") | $isdefed("ENABLE_MULTI_MAKE_PRU") | $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	; Multi-channel load share using multiple PRUs
TAMAGAWA_RTU_TRIGGER_HOST_EVT              .set	34			;( (0x20 | 2), pr0_pru_mst_intr[2]_intr_req )
TAMAGAWA_PRU_TRIGGER_HOST_EVT              .set	35			;( (0x20 | 3), pr0_pru_mst_intr[3]_intr_req )
TAMAGAWA_TXPRU_TRIGGER_HOST_EVT            .set	36			;( (0x20 | 4), pr0_pru_mst_intr[4]_intr_req )
	.else
	; Single PRU
TAMAGAWA_PRU_TRIGGER_HOST_EVT              .set	34			;( (0x20 | 2), pr0_pru_mst_intr[2]_intr_req )
	.endif
	.else
	; "SLICE0"
	.if $isdefed("ENABLE_MULTI_MAKE_RTU") | $isdefed("ENABLE_MULTI_MAKE_PRU") | $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	; Multi-channel load share using multiple PRUs
TAMAGAWA_RTU_TRIGGER_HOST_EVT              .set	37			;( (0x20 | 5), pr0_pru_mst_intr[5]_intr_req )
TAMAGAWA_PRU_TRIGGER_HOST_EVT              .set	38			;( (0x20 | 6), pr0_pru_mst_intr[6]_intr_req )
TAMAGAWA_TXPRU_TRIGGER_HOST_EVT            .set	39			;( (0x20 | 7), pr0_pru_mst_intr[7]_intr_req )
	.else
	; Single PRU
TAMAGAWA_PRU_TRIGGER_HOST_EVT              .set	37			;( (0x20 | 5), pr0_pru_mst_intr[5]_intr_req )
	.endif
	.endif
	.endif
