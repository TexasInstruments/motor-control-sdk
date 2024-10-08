
; Copyright (C) 2021-2023 Texas Instruments Incorporated
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

	.include "memory.inc"
	.include "defines.inc"
	.include "macros.inc"
	;.sect	".text"
	.ref transport_init_done
	.ref PUSH_FIFO_2B_8x
	.ref PUSH_FIFO_3_8x
	.ref WAIT_TX_FIFO_FREE
	.ref datalink_transport_on_v_frame_done
	.ref datalink_transport_on_v_frame_done_2
	.ref transport_layer_processing_1_done
	.ref transport_layer_processing_2_done
	.ref datalink_abort
	.ref datalink_abort2
	.ref qm_add
	.ref transport_layer_done
	.ref transport_layer_send_msg_done
	.ref transport_layer_recv_msg_done
	;.ref transport_layer_assemble_msg_done
	.global transport_layer_send_msg
	;.global transport_layer_assemble_msg
	.global transport_layer_recv_msg
	.global transport_on_h_frame
	.global transport_on_v_frame
	.global transport_on_v_frame_2
	.global transport_layer_processing_1
	.global transport_layer_processing_2
	.global transport_layer
	.global calc_speed
	.global calc_16bit_crc
	.global calc_fastpos

	; part 2 code starts here

	.sect	".text:part2"

;----------------------------------------------------
;transport_on_v_frame_2
; Cycle Budget : 140 cycles
; extracts safe position from secondary channel
; verifies the summary from encoder
; NOT allowed to use REG_TMP11
;----------------------------------------------------
transport_on_v_frame_2:
;save REG_FNC.w0 content
	mov		REG_TMP11.w1, REG_FNC.w0

; retrieve the 8 bytes for secondary channel from VPOS2_TEMP
    lbco        &REG_TMP1, MASTER_REGS_CONST, VPOS2_TEMP, 8
; error checks for secondary channel
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
; retrieve H_FRAME.flags from H_FRAME_FLAGS_TEMP
    lbco        &REG_TMP0.w2, MASTER_REGS_CONST, H_FRAME_FLAGS_TEMP, 2
;channel 2 transmission error?
	qbbs	    transport_on_v_frame_dont_update_qm_secondary_channel, REG_TMP0.w2, FLAG_ERR_SEC
;checking for crc error in channel 2
; retrieve CRC_SEC from CRC_SEC_TEMP

    lbco        &REG_TMP0.w2, MASTER_REGS_CONST, CRC_SEC_TEMP, 2
	qbeq		check_for_slave_error_on_secondary_channel, REG_TMP0.w2, 0
; set SCE2 bit in ONLINE_STATUS_2
	set		    REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_SCE2
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
transport_on_v_frame_dont_update_qm_secondary_channel:
	qba		transport_on_v_frame_2_exit
check_for_slave_error_on_secondary_channel:
; clear SCE2 bit in ONLINE_STATUS_2
	clr		    REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_SCE2
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
; No QM updates for CRC check success with safe channel 2
    lbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
;check for special character: K27.7 is sent in first byte of secondary vertical channel if slave error occured
; assumption: r21.b2 or CHANNEL.ch_sech.b2 contains the first byte of secondary vertical channel
	qbne		transport_on_v_frame_no_vpos2_error, REG_TMP2.b2, K27_7
    ; set VPOS2 bit in ONLINE_STATUS_2
	set		    REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_VPOS2
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
    qba         transport_on_v_frame_vpos2_error_exit
transport_on_v_frame_no_vpos2_error:
	clr		    REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_VPOS2
transport_on_v_frame_vpos2_error_exit:
transport_on_v_frame_2_exit:
; store the data from secondary channel
; swap bytes in each 32 bit register REG_TMP1 and REG_TMP2
	xin     160, &REG_TMP1, 8

; update ONLINE_STATUS_2_SUM2 in ONLINE_STATUS_2
    qbbs        online_status_2_sum2_set, REG_TMP2.b0, STATUS2_TEST2
    qbbs        online_status_2_sum2_set, REG_TMP2.b0, STATUS2_ERR2
    clr         REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_SUM2
    qba         online_status_2_sum2_not_set
online_status_2_sum2_set:
    set         REG_TMP0.b0, REG_TMP0.b0, ONLINE_STATUS_2_SUM2
online_status_2_sum2_not_set:
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 1
; Store STATUS2, VPOS24, VPOS23 and VPOS22
	sbco	&REG_TMP2.b0, MASTER_REGS_CONST, STATUS2, 4
; Store VPOS21, VPOS20, VPOSCRC2_H and VPOSCRC2_L
	sbco	&REG_TMP1.b0, MASTER_REGS_CONST, VPOS21, 4
	qbbc        no_qm_sub, REG_TMP2.b0, STATUS2_TEST2
	QM_SUB		8
no_qm_sub:
; generate interrupt PRU0_ARM_IRQ2
	ldi	    r31.w0, PRU0_ARM_IRQ2
; update the position and crc to DMEM
; and raise interrupt
	;Update ONLINE_STATUS_1 to DMEM
    lbco	&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H_TEMP, 1
	sbco	&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 1
 	.if $defined("HDSL_MULTICHANNEL")
 	CALL2 WAIT_TX_FIFO_FREE
	LOOP push_1B_0 ,2
	PUSH_FIFO_CONST  0xff
push_1B_0:
	qbeq			free_run_mode2, EXTRA_SIZE, 0
	PUSH_FIFO_CONST		0xff
	RESET_CYCLCNT
free_run_mode2:
	.endif
;Update EVENT_S in DMEM for EVENT_S_VPOS and EVENT_S_SCE events
	lbco	&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S_TEMP, 1
	lbco	&REG_TMP0.w1, MASTER_REGS_CONST, EVENT_S, 2
	qbbc	no_VPOS_update_in_event_reg,REG_TMP0.b0,EVENT_S_VPOS
	set REG_TMP0.b1,REG_TMP0.b1,EVENT_S_VPOS
no_VPOS_update_in_event_reg:
	qbbc		update_events_no_int4_VPOS, REG_TMP0.b2, EVENT_S_VPOS
; generate interrupt_s
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int4_VPOS:
	qbbc	no_SCE_update_in_event_reg,REG_TMP0.b0,EVENT_S_SCE
	set REG_TMP0.b1,REG_TMP0.b1,EVENT_S_SCE
no_SCE_update_in_event_reg:
	qbbc		update_events_no_int4_SCE, REG_TMP0.b2, EVENT_S_SCE
; generate interrupt_s
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int4_SCE:
	sbco	&REG_TMP0.b1, MASTER_REGS_CONST, EVENT_S, 1
	lbco	&REG_TMP0.b0, MASTER_REGS_CONST, VPOS_TEMP, 7
    sbco	&REG_TMP0.b0, MASTER_REGS_CONST, VPOS4, 7
; generate interrupt PRU0_ARM_IRQ1
	ldi		    r31.w0, PRU0_ARM_IRQ1
transport_skip_vpos_update:

; Set POSTX to 3
    ldi         REG_TMP0.b0, 0x3
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, POSTX, 1

; Load summary
    lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_SUM_TEMP, 1
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_SUM, 1
;check SUMMARY and MASK_SUM
    lbco		&REG_TMP1.b1, MASTER_REGS_CONST, MASK_SUM, 1
	and		    REG_TMP1.b0, REG_TMP0.b0, REG_TMP1.b1
	qbeq		summary_no_int, REG_TMP1.b0, 0x00
;set event and generate interrupt
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	;set		REG_TMP0.w0, REG_TMP0.w0, REG_FNC.b0
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_SUM
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int7, REG_TMP0.w2, REG_TMP0.w2
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int7:
summary_no_int:

; Update SUM and SSUM bits in ONLINE_STATUS registers
    lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_SUM, 1
	lbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 3
	qbeq		online_status_sum_clear, REG_TMP0.b0, 0x00
    set         REG_TMP2.b0, REG_TMP2.b0, ONLINE_STATUS_D_SUM
    set         REG_TMP2.b2, REG_TMP2.b2, ONLINE_STATUS_1_SSUM
;set SSUM in EVENT_S and generate interrupt_s
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 2
	set         REG_TMP0.b0, REG_TMP0.b0, EVENT_S_SSUM
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int17, REG_TMP0.b1, EVENT_S_SSUM
; generate interrupt_s
	ldi			r31.w0, PRU0_ARM_IRQ4
update_events_no_int17:
	qba 		online_status_sum_save
online_status_sum_clear:
    clr         REG_TMP2.b0, REG_TMP2.b0, ONLINE_STATUS_D_SUM
    clr         REG_TMP2.b2, REG_TMP2.b2, ONLINE_STATUS_1_SSUM
online_status_sum_save:
    sbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 3

;restore REG_FNC.w0 content
	mov		REG_FNC.w0, REG_TMP11.w1
	jmp		datalink_transport_on_v_frame_done_2

;----------------------------------------------------
;Function: transport_on_h_frame (RET_ADDR)
;calculates the acceleration, velocity and fast position
; 38+9+52 = 99 cycles
;input:
;output:
;modifies:
;----------------------------------------------------
;TODO: reduce processing time by 24 cycles
transport_on_h_frame:

; Set POSTX to 0
    ldi         REG_TMP0.b0, 0x0
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, POSTX, 1

;check for byte error in acceleration channel
	qbbs		transport_acc_err_inc, H_FRAME.flags, FLAG_ERR_ACC
;crc error verification
	;CALL1		calc_acc_crc
;5 bits CRC of acceleration channel are flipped -> flip back
	xor		H_FRAME.acc, H_FRAME.acc, 0x1f
	ldi		REG_TMP2, (LUT_CRC5+PDMEM00)
	lbbo		&REG_TMP0.b1, REG_TMP2, H_FRAME_acc1, 1
	lsl		REG_TMP0.b1, REG_TMP0.b1, 3
	xor		REG_TMP0.b1, REG_TMP0.b1, H_FRAME_acc0
	lbbo		&REG_FNC.b0, REG_TMP2, REG_TMP0.b1, 1
	qbne		transport_acc_err_inc, REG_FNC.b0, 0
;check for special character: K29.7 is sent two times if slave error occured
	ldi		REG_TMP0.w0, DOUBLE_K29_7
    qbne		delta_delta_position, H_FRAME.acc, REG_TMP0.w0
transport_acc_err_inc:
;update the acc_err_cnt register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, ACC_ERR_CNT, 1
	add		REG_TMP0.b0, REG_TMP0.b0, 1
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ACC_ERR_CNT, 1
;update align phase:
	ldi ALIGN_PHASE,0
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, ALIGN_PH, 1
	.if $defined("HDSL_CHECK_ALIGNMENT_PHASE")
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST,CURRENT_ALIGN_PHASE, 1
	.endif
;reset if it is too large
	lbco		&REG_TMP0.b1, MASTER_REGS_CONST, ACC_ERR_CNT_THRESH, 1
	qbgt		transport_on_h_frame_no_reset, REG_TMP0.b0, REG_TMP0.b1
	jmp		datalink_abort
transport_on_h_frame_no_reset:
;save return addr
	mov		REG_TMP11.w0, RET_ADDR0
	;CALL		estimator_acc; Instead of calling the API, copy the code here to save PRU cycles.
;----------------------------------------------------
;Function: estimator_acc (RET_ADDR)
;This function estimates the FPOS and VEL
;15+15+22=52 cycles
;input:
;
;output:
;	FAST_POSH, FAST_POSL, SPEED
;modifies:
;	FAST_POSH, FAST_POSL, SPEED, REG_TMP1, REG_TMP0
;----------------------------------------------------
;Estimating the acceleration
;ith acc = (delta_a4* 1 + delta_a3 * 2 + delta_a2 * 4 + delta_a1 * 8 + delta_a0 * 16) / 32
	lsl	  	REG_TMP0.w0, DELTA_ACC3, 1
	lsl		REG_TMP0.w2, DELTA_ACC2, 2
	lsl		REG_TMP1.w0, DELTA_ACC1, 3
	lsl		REG_TMP1.w2, DELTA_ACC0, 4
	add		REG_TMP0.w0, REG_TMP0.w0, REG_TMP0.w2
	add		REG_TMP1.w0, REG_TMP1.w2, REG_TMP1.w0
	add		REG_TMP0.w0, REG_TMP0.w0, REG_TMP1.w0
	add		REG_TMP0.w0, REG_TMP0.w0, DELTA_ACC4
;divide with 32
	lsr		REG_TMP0.w0, REG_TMP0.w0, 5
;sign extend delta acceleration to 16 bit -> acc size
	qbbc		estimator_acc_sign_extend_dacc1, REG_TMP0.w0, 10
	or		REG_TMP0.b1, REG_TMP0.b1, 0xf8
estimator_acc_sign_extend_dacc1:
; TODO: calcuate MAXACC, cap acc
;add estimated delta acc to LAST_ACC
	add		REG_FNC.w0, LAST_ACC, REG_TMP0.w0
;check if estimated acc is neg. or pos.
	;CALL1		calc_speed; Instead of calling the API, copy the code here to save PRU cycles.
;sign extend acceleration to  24 bit -> speed size
	xor		REG_TMP0, REG_TMP0, REG_TMP0
	qbbc		calc_speed_extend_acc1, REG_FNC.w0, 10
	ldi		REG_TMP0.w1, 0xfff8
calc_speed_extend_acc1:
	.if $defined("HDSL_MULTICHANNEL")
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0x00
	PUSH_FIFO_CONST  0x00
	.endif
	or		REG_TMP0.w0, REG_TMP0.w0, REG_FNC.w0
	add		SPEED.w0, SPEED.w0, REG_TMP0.w0
	adc		SPEED.b2, SPEED.b2, REG_TMP0.b2
;updating the delta acceleration regs
	mov		DELTA_ACC4, DELTA_ACC3
	mov		DELTA_ACC3, DELTA_ACC2
	mov		DELTA_ACC2, DELTA_ACC1
	mov		DELTA_ACC1, DELTA_ACC0
	sub		DELTA_ACC0, REG_TMP0.w0, LAST_ACC
	mov		LAST_ACC, REG_TMP0.w0

    CALL1		calc_fastpos
;restore return addr
	mov		RET_ADDR0, REG_TMP11.w0
; Update ONLINE_STATUS_D_H for POS error
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
	set 		REG_TMP0,REG_TMP0,ONLINE_STATUS_D_POS
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
	qba		transport_on_h_frame_exit
delta_delta_position:

;reset ACC_ERR_CNT
	sbco		&REG_FNC.b0, MASTER_REGS_CONST, ACC_ERR_CNT, 1
;shift out crc bits
	lsr		REG_FNC.w0, H_FRAME.acc, 5
; learn highest abs. acc
	;CALL1		calc_speed; Instead of calling the API, copy the code here to save PRU cycles.
;sign extend acceleration to  24 bit -> speed size
	xor		REG_TMP0, REG_TMP0, REG_TMP0
	qbbc		calc_speed_extend_acc0, REG_FNC.w0, 10
	ldi		REG_TMP0.w1, 0xfff8
calc_speed_extend_acc0:
	.if $defined("HDSL_MULTICHANNEL")
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0x00
	PUSH_FIFO_CONST  0x00
	.endif
	or		REG_TMP0.w0, REG_TMP0.w0, REG_FNC.w0
	add		SPEED.w0, SPEED.w0, REG_TMP0.w0
	adc		SPEED.b2, SPEED.b2, REG_TMP0.b2
;updating the delta acceleration regs
	mov		DELTA_ACC4, DELTA_ACC3
	mov		DELTA_ACC3, DELTA_ACC2
	mov		DELTA_ACC2, DELTA_ACC1
	mov		DELTA_ACC1, DELTA_ACC0
	sub		DELTA_ACC0, REG_TMP0.w0, LAST_ACC
	mov		LAST_ACC, REG_TMP0.w0

	CALL1		calc_fastpos
transport_on_h_frame_exit:
;calculate rel. pos and store
	lbco		&REG_TMP0, MASTER_REGS_CONST, REL_POS0, 4
;sign extend speed to 32 bits and add it to REL_POS
	mov		REG_TMP1, SPEED
    ldi     REG_TMP1.b3, 0
	qbbc	calc_relpos_extend_vel, SPEED, 23
	ldi		REG_TMP1.b3, 0xff
calc_relpos_extend_vel:
	add		REG_TMP0.w0, REG_TMP0.w0, REG_TMP1.w0
	adc		REG_TMP0.w2, REG_TMP0.w2, REG_TMP1.w2
	sbco		&REG_TMP0, MASTER_REGS_CONST, REL_POS0, 4
	;store fast pos. and velocity
    mov     REG_TMP0, FAST_POSH
    mov     REG_TMP1, SPEED
    xin     160, &REG_TMP0, 8
	sbco	&REG_TMP0, MASTER_REGS_CONST, POS4, SIZE_FAST_POS+3
; Set POSTX to 2
    ldi         REG_TMP0.b0, 0x2
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, POSTX, 1
;store last FAST_POS
	sbco		&FAST_POSL, MASTER_REGS_CONST, LAST_FAST_POS0, SIZE_FAST_POS

	RET

;--------------------------------------------------------------------------------------------------
;Function: calc_fastpos (RET_ADDR1)
;This function sign extends speed and adds it to fast position
;22 cycles
;input:SPEED, FAST_POSH, FAST_POSL
;
;output: FAST_POSH, FAST_POSL
;
;modifies:FAST_POSH, FAST_POSL
;
;--------------------------------------------------------------------------------------------------
calc_fastpos:
;sign extend speed to NUM_POS_BITS=NUM_MT_BITS+NUM_ST_BITS and add it to FAST_POS
	ldi		REG_TMP1.b0, 0
	qbbc		calc_fastpos_extend_vel, SPEED, 23
	ldi		REG_TMP1.b0, 0xff
calc_fastpos_extend_vel:
	mov		REG_TMP0, SPEED
	mov		REG_TMP0.b3, REG_TMP1.b0
;calculating fast position
	add		FAST_POSL, FAST_POSL, REG_TMP0.b0
	lsr		REG_TMP0, REG_TMP0, 8
	mov		REG_TMP0.b3, REG_TMP1.b0
	adc		FAST_POSH, FAST_POSH, REG_TMP0
;load mask for fast position
	lbco		&REG_TMP1, MASTER_REGS_CONST, MASK_POS, 4
	and		FAST_POSH, FAST_POSH, REG_TMP1
	RET1
;--------------------------------------------------------------------------------------------------
;Function: calc_speed (RET_ADDR1)
;This function sign extends acceleration and adds it to speed
;15 cycles
;input:
;	SPEED
;	REG_FNC.w0: acceleration
;
;output: SPEED
;
;modifies: SPEED
;
;--------------------------------------------------------------------------------------------------
calc_speed:
;sign extend acceleration to  24 bit -> speed size
	xor		REG_TMP0, REG_TMP0, REG_TMP0
	qbbc		calc_speed_extend_acc, REG_FNC.w0, 10
	ldi		REG_TMP0.w1, 0xfff8
calc_speed_extend_acc:
	or		REG_TMP0.w0, REG_TMP0.w0, REG_FNC.w0
	add		SPEED.w0, SPEED.w0, REG_TMP0.w0
	adc		SPEED.b2, SPEED.b2, REG_TMP0.b2
;updating the delta acceleration regs
	mov		DELTA_ACC4, DELTA_ACC3
	mov		DELTA_ACC3, DELTA_ACC2
	mov		DELTA_ACC2, DELTA_ACC1
	mov		DELTA_ACC1, DELTA_ACC0
	sub		DELTA_ACC0, REG_TMP0.w0, LAST_ACC
	mov		LAST_ACC, REG_TMP0.w0
	RET1

;--------------------------------------------------------------------------------------------------
;Function: calc_acc_crc (RET_ADD1)
;This function checks the crc for the acceleration channel
;11 cycles
;input:
;	H_frame.acc
;output:
;	REG_FNC.b0: 0 indicates that crc check was successfully
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
calc_acc_crc:
;5 bits CRC of acceleration channel are flipped -> flip back
	xor		H_FRAME.acc, H_FRAME.acc, 0x1f
	ldi		REG_TMP2, (LUT_CRC5+PDMEM00)
	lbbo		&REG_TMP0.b1, REG_TMP2, H_FRAME_acc1, 1
	lsl		REG_TMP0.b1, REG_TMP0.b1, 3
	xor		REG_TMP0.b1, REG_TMP0.b1, H_FRAME_acc0
	lbbo		&REG_FNC.b0, REG_TMP2, REG_TMP0.b1, 1
	RET1
;--------------------------------------------------------------------------------------------------
;Function: demp_data_symbols (RET_ADDR1)
;Demaps data symbols (5bits) and converts them to data nibbles (4bits)
;input:
;	REG_FNC.b0: Number of data symbols
;	r1.b1: source
;	r1.b0: destination
;output:
;	*r1.b0[0:REG_FNC.b0]: demapped data
;modifies:
;--------------------------------------------------------------------------------------------------
demap_data_symbols:

; common code starts here
	.sect ".text"

;----------------------------------------------------
;transport_layer_recv_msg
;Handles Hiperface DSL messages receiving
;?? cycles
;input:
;output:
;modifies:
;----------------------------------------------------
transport_layer_recv_msg:
;check if we reset protocol
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYS_CTRL, 1
;ERROR: qbbc datalink_abort2 is not working, though no compiler error
	qbbc		transport_layer_no_prst, REG_TMP0.b0, SYS_CTRL_PRST

	jmp		datalink_abort
transport_layer_no_prst:
;check if we reset protocol by reading SAFE_CTRL register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
	qbbc		transport_layer_no_safe_prst, REG_TMP0.b0, SAFE_CTRL_PRST
	jmp		datalink_abort
transport_layer_no_safe_prst:
;are we already receiving a long message?
	qbeq		transport_layer_check_for_new_msg, LONG_MSG_RECV.bits_left, 0

	and		REG_TMP2.b0, CHANNEL.ch_paral, 0x0f
;last byte (crc low)? -> then flip it
	qblt		transport_layer_recving_long_msg_dont_flip_nibble, LONG_MSG_RECV.bits_left, 8
	xor		REG_TMP2.b0, REG_TMP2.b0, 0x0f
transport_layer_recving_long_msg_dont_flip_nibble:
;calculate running crc
	ldi		REG_TMP0, (LUT_CRC16+PDMEM00)
	lsr		REG_TMP1.b0, LONG_MSG_RECV_CRC_H, 4
	xor		REG_TMP1.b0, REG_TMP1.b0, REG_TMP2.b0
	lsl		REG_TMP1.w0, REG_TMP1.b0, 1
	lbbo		&REG_TMP1.w0, REG_TMP0, REG_TMP1.w0, 2
	lsl		LONG_MSG_RECV.crc, LONG_MSG_RECV.crc, 4
	xor		LONG_MSG_RECV.crc, LONG_MSG_RECV.crc, REG_TMP1.w0

;are we receiving crc?
	qbge		transport_layer_recving_long_msg_crc, LONG_MSG_RECV.bits_left, 16
;still receiving data
	qbbs		transport_layer_recving_long_msg_data_low_nibble, LONG_MSG_RECV.bits_left, 2

transport_layer_recving_long_msg_data_high_nibble:

	lsl		REG_TMP0.b0, REG_TMP2.b0, 4
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, LONG_MSG_RECV.ptr, 1
	qba		transport_layer_recving_long_msg_data_nibble_end
transport_layer_recving_long_msg_data_low_nibble:

	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, LONG_MSG_RECV.ptr, 1
	or		REG_TMP0.b0, REG_TMP0.b0, REG_TMP2.b0
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, LONG_MSG_RECV.ptr, 1
	add		LONG_MSG_RECV.ptr, LONG_MSG_RECV.ptr, 1
transport_layer_recving_long_msg_data_nibble_end:

	qba		transport_layer_recving_long_msg_end
transport_layer_recving_long_msg_crc:
;we are receiving crc
	qbne		transport_layer_recving_long_msg_end, LONG_MSG_RECV.bits_left, 4
;set long msg channel to unbusy
; Set EVENT_FREL in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_FREL
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int8, REG_TMP0.w2, EVENT_FREL
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int8:
; Set ONLINE_STATUS_D_FREL in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_FREL-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
;check for crc error
	qbeq		transport_layer_recving_long_msg_end, LONG_MSG_RECV.crc, 0
; Set EVENT_ANS in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_ANS
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int9, REG_TMP0.w2, EVENT_ANS
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int9:
; Set ANS in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
transport_layer_recving_long_msg_end:
; Clear ANS in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	clr		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	sub		LONG_MSG_RECV.bits_left, LONG_MSG_RECV.bits_left, 4
	qba		transport_layer_recv_msg_end
transport_layer_check_for_new_msg:
;check only position ch_para[35:39], if found at this position, we can use subsequent nibbles and have full short message available
	lsr		REG_TMP0.b0, CHANNEL.ch_parah, 3
	and		REG_TMP0.b0, REG_TMP0.b0, 0x1f
	qbeq		transport_layer_recv_no_msg, REG_TMP0.b0, S_PAR_IDLE
	qbbs		transport_layer_recv_msg_end, H_FRAME.flags, FLAG_WAIT_IDLE
;check for special character

	qbbs		transport_layer_recv_msg_check_for_nak, REG_TMP0.b0, 4

;set flag to signalize that we need to wait for next S_PAR_IDLE again, so we do not parse data multiple times
	set		H_FRAME.flags, H_FRAME.flags, FLAG_WAIT_IDLE
;reassemble 32 bits of msg, 5bits->4bits
	and		REG_TMP11.b3, REG_TMP0.b0, 0x1f
	lsl		REG_TMP11.b3, REG_TMP11.b3, 4
	lsl		REG_TMP0.b0, CHANNEL.ch_parah, 2
	lsr		REG_TMP0.b1, CHANNEL.ch_paral, 30
	or		REG_TMP0.b0, REG_TMP0.b0, REG_TMP0.b1
	and		REG_TMP0.b0, REG_TMP0.b0, 0x0f
	or		REG_TMP11.b3, REG_TMP11.b3, REG_TMP0.b0
	.if $defined("HDSL_MULTICHANNEL")
	CALL2 WAIT_TX_FIFO_FREE
	CALL3 PUSH_FIFO_3_8x
	.endif
	mov		REG_TMP2, CHANNEL.ch_paral
	ldi		REG_TMP1.b0, &REG_TMP11.b0
	loop		transport_layer_reassemble_msg_loop, 3
	and		REG_TMP0.b1, REG_TMP2.b0, 0x0f
	lsr		REG_TMP0.b2, REG_TMP2, 1
	and		REG_TMP0.b2, REG_TMP0.b2, 0xf0
	or		REG_TMP0.b0, REG_TMP0.b1, REG_TMP0.b2
	mvib		*REG_TMP1.b0++, REG_TMP0.b0
	lsr		REG_TMP2, REG_TMP2, 10
transport_layer_reassemble_msg_loop:
;identify message type

	qbbs		transport_layer_received_long_msg, REG_TMP11.b3, 7
transport_layer_received_short_msg:
;check crc
	ldi		REG_TMP1.b0, &r12.b0
;read or write?

	qbbs		transport_layer_short_msg_recv_read, REG_TMP11.b3, 6
;received write ack
	ldi		REG_FNC.b0, 1
	CALL1		(calc_16bit_crc);-DYNAMIC_CODE_OFFSET-CODE_TRANSPORT_LAYER*CODE_SIZE)
	qbne		transport_layer_recv_msg_check_for_nak, REG_FNC.w0, REG_TMP11.w1
;we received a valid msg -> set ACK flag
	or		SHORT_MSG.timeout, SHORT_MSG.timeout, (1<<FLAG_ACK)
;set short msg channel to unbusy
; Set EVENT_S_FRES in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_S_FRES
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int100, REG_TMP0.b1, EVENT_S_FRES
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int100:
; Set ONLINE_STATUS_1_FRES in ONLINE_STATUS_1 register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_1_FRES-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	qba		transport_layer_recv_msg_check_for_nak
transport_layer_short_msg_recv_read:
;received read answer
	ldi		REG_FNC.b0, 2
	CALL1		(calc_16bit_crc);-DYNAMIC_CODE_OFFSET-CODE_TRANSPORT_LAYER*CODE_SIZE)
	qbne		transport_layer_recv_msg_check_for_nak, REG_FNC.w0, REG_TMP11.w0
;save to master register
	sbco		&REG_TMP11.b2, MASTER_REGS_CONST, S_PC_DATA, 1
;we received a valid msg -> set ACK flag
	or		SHORT_MSG.timeout, SHORT_MSG.timeout, (1<<FLAG_ACK)
;set short msg channel to unbusy
; Set EVENT_S_FRES in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_S_FRES
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int10, REG_TMP0.b1, EVENT_S_FRES
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int10:
; Set ONLINE_STATUS_1_FRES in ONLINE_STATUS_1 register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_1_FRES-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	qba		transport_layer_recv_msg_check_for_nak
transport_layer_received_long_msg:
;process long message
;calculate number of bits we still need to receive
	lsr		REG_TMP0.b0, REG_TMP11.b3, 2
	and		REG_TMP0.b0, REG_TMP0.b0, 0x03
	ldi		LONG_MSG_RECV.bits_left, 0
	qbeq		transport_layer_received_long_msg_no_data, REG_TMP0.b0, 0
	ldi		LONG_MSG_RECV.bits_left, 8
	lsl		LONG_MSG_RECV.bits_left, LONG_MSG_RECV.bits_left, REG_TMP0.b0
transport_layer_received_long_msg_no_data:
;we already read 32 bits bits (PC_ADD_H/L + CRC)
;store already received  bits (32) to master registers
	mov		REG_TMP0, REG_TMP11
	xin     160, &REG_TMP0, 4
;check if we have LOFF
	qbbc		transport_layer_received_long_msg_no_loffset, REG_TMP11.b3, LOFF
	sbco		&REG_TMP0, MASTER_REGS_CONST, PC_ADD_H, 2
	sbco		&REG_TMP11.b0, MASTER_REGS_CONST, PC_BUFFER0, 2
;set ptr
	ldi		LONG_MSG_RECV.ptr, 0x20
	qba		transport_layer_received_long_msg_loffset_end
transport_layer_received_long_msg_no_loffset:
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, PC_ADD_H, 2
;lower two bytes data or crc?
;if crc then dont save to PC_BUFFER

	qbeq		transport_layer_received_long_msg_no_loffset_crc, LONG_MSG_RECV.bits_left, 0
;set ptr
	ldi		LONG_MSG_RECV.ptr, 0x22
	sbco		&REG_TMP0.b2, MASTER_REGS_CONST, PC_BUFFER0, 2
	qba		transport_layer_received_long_msg_loffset_end
transport_layer_received_long_msg_no_loffset_crc:
;long message is complete
	xor		REG_TMP11.b0, REG_TMP11.b0, 0xff
; Set EVENT_FREL in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_FREL
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int11, REG_TMP0.w2, EVENT_FREL
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int11:
; Set ONLINE_STATUS_D_FREL in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_FREL-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	clr		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
transport_layer_received_long_msg_loffset_end:

;calculate CRC for already recevied bits
	ldi		REG_FNC.b0, 4
	ldi		r1.b0, &r12.b0
	CALL1		(calc_16bit_crc);-DYNAMIC_CODE_OFFSET-CODE_TRANSPORT_LAYER*CODE_SIZE)
	xor		LONG_MSG_RECV.crc, REG_FNC.w0, 0xff
;raise error if long message complete and error
	qbne		transport_layer_resend_msg_end, LONG_MSG_RECV.bits_left, 0
	qbeq		transport_layer_resend_msg_end, LONG_MSG_RECV.crc, 0

; Set EVENT_ANS in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_ANS
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int12, REG_TMP0.w2, EVENT_ANS
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int12:
; Set ANS in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	qba		transport_layer_resend_msg_end
transport_layer_recv_no_msg:
;reset flag
	clr		H_FRAME.flags, H_FRAME.flags, FLAG_WAIT_IDLE
transport_layer_recv_msg_check_for_nak:
;check for NAK
	and		REG_TMP0.b0, H_FRAME.s_par, 0x1f
	qbeq		transport_layer_resend_msg, REG_TMP0.b0, S_PAR_SNAK
;check for S_PAR_LNAK
	qbne		transport_layer_recv_msg_check_for_nak_no_lnak, REG_TMP0.b0, S_PAR_LNAK
;raise flag
; Set EVENT_ANS in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_ANS
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int13, REG_TMP0.w2, EVENT_ANS
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int13:
; Set ANS in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
    qba         transport_layer_recv_msg_check_for_init_no_init
transport_layer_recv_msg_check_for_nak_no_lnak:
;check for S_PAR_INIT
	qbne		transport_layer_recv_msg_check_for_init_no_init, REG_TMP0.b0, S_PAR_INIT
; Moving the events register update to transport_update_events_and_online_status
; Set MIN bit to indicate acknowledgment of message initialization
; set MIN bit in EVENT_L and EVENT_S registers
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_MIN
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int19, REG_TMP0.w2, EVENT_MIN
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int19:
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_MIN
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int20, REG_TMP0.b1, EVENT_S_MIN
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int20:
    lbco		&REG_TMP0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 3
    set         REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
    set         REG_TMP0.b2, REG_TMP0.b2, (ONLINE_STATUS_1_MIN-8)
	sbco		&REG_TMP0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 3
    qba         transport_layer_min_update_done
transport_layer_recv_msg_check_for_init_no_init:
transport_layer_min_unset:
	lbco		&REG_TMP0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 3
    clr         REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_MIN-8)
    clr         REG_TMP0.b2, REG_TMP0.b2, (ONLINE_STATUS_1_MIN-8)
	sbco		&REG_TMP0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 3
transport_layer_min_update_done:
;check for timeout - count only down when bitsleft = 0 amnd timeout != 0
	qbne		transport_layer_resend_msg_end, SHORT_MSG.bits_left, 0
;lower 6 bits is timeout value - MSB is ACK flag
	qbbs		transport_layer_resend_msg_end, SHORT_MSG.timeout, FLAG_ACK
	sub		SHORT_MSG.timeout, SHORT_MSG.timeout, 1
	qbne		transport_layer_resend_msg_end, SHORT_MSG.timeout, 0
transport_layer_resend_msg:
;resend message
	ldi		SHORT_MSG.timeout, 64
	ldi		SHORT_MSG.bits_left, 24
	qbbc		transport_layer_resend_msg_read, SHORT_MSG.addr, (1<<MSG_READ)
	add		SHORT_MSG.bits_left, SHORT_MSG.bits_left, 8
transport_layer_resend_msg_read:
transport_layer_resend_msg_end:
transport_layer_recv_msg_end:
	jmp		transport_layer_recv_msg_done
;----------------------------------------------------
;transport_layer_send_msg
;Handles Hiperface DSL messages sending
;Short message are priotized over long messages
;54 cycles
;input:
;output:
;modifies:
;----------------------------------------------------
transport_layer_send_msg:
;TODO: reduce cycles
; Skip message processing until one v-frame is complete
	qbbc			transport_layer_send_msg_end, H_FRAME.flags, FLAG_NORMAL_FLOW
	ldi		SEND_PARA, M_PAR_IDLE
;check if we discard any messages and reset parameter channel
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYS_CTRL, 1
	qbbc		transport_layer_send_msg_no_reset_sys_ctrl, REG_TMP0.b0, SYS_CTRL_MRST
;clear flag
	clr		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_MRST
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYS_CTRL, 1
;reset
	ldi		SEND_PARA, M_PAR_INIT
	zero		&SHORT_MSG, (6)
	clr		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
	qba		transport_layer_send_msg_end
transport_layer_send_msg_no_reset_sys_ctrl:
;check if we discard any messages and reset parameter channel
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
	qbbc		transport_layer_send_msg_no_reset_safe_ctrl, REG_TMP0.b0, SAFE_CTRL_MRST
;clear flag
	clr		REG_TMP0.b0, REG_TMP0.b0, SAFE_CTRL_MRST
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
;reset
	ldi		SEND_PARA, M_PAR_INIT
	zero		&SHORT_MSG, (6)
	clr		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
	qba		transport_layer_send_msg_end
transport_layer_send_msg_no_reset_safe_ctrl:
;do not send new message if we are not finished with message

	qbbc		transport_layer_check_for_new_short_msg, H_FRAME.flags, FLAG_PARA_BUSY
	sub		SHORT_MSG.bits_left, SHORT_MSG.bits_left, 4

;check if we still need to calculate crc or if we send crc
	qblt		transport_layer_send_dont_send_crc, SHORT_MSG.bits_left, 12
;send crc
	lsr		SEND_PARA, SHORT_MSG.crc, SHORT_MSG.bits_left
	and		SEND_PARA, SEND_PARA, 0x0f

;last nibble?
	qbne		transport_layer_send_not_last_nibble, SHORT_MSG.bits_left, 0
;it is the last nibble
;set long msg channel to unbusy
	clr		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
transport_layer_send_not_last_nibble:
	qba		transport_layer_send_msg_end
transport_layer_send_dont_send_crc:
;are we sending short or long message?

	qbbc		transport_layer_send_sending_short_msg, SHORT_MSG.addr, 7
;we are still sending long message
;For long message, data is ptr to memory buffer
	lbco		&SEND_PARA, MASTER_REGS_CONST, SHORT_MSG.data, 1
;check if we send higher or lower nibble of byte
;if bits_left%8 != 0 -> we send lower nibble
	qbbc		transport_layer_sned_long_msg_lower_nibble, SHORT_MSG.bits_left, 2
	lsr		SEND_PARA, SEND_PARA, 4
	qba		transport_layer_sned_long_msg_nibble_end
transport_layer_sned_long_msg_lower_nibble:
	and		SEND_PARA, SEND_PARA, 0x0f
	add		SHORT_MSG.data, SHORT_MSG.data, 1
transport_layer_sned_long_msg_nibble_end:
	qba		transport_layer_send_sending_calc_crc
transport_layer_send_sending_short_msg:
;we are still sending short message
	lsr		SEND_PARA, SHORT_MSG32, SHORT_MSG.bits_left
;no special char -> only 4 bits
	and		SEND_PARA, SEND_PARA, 0x0f
transport_layer_send_sending_calc_crc:
;calculate running CRC
	ldi		REG_TMP2, (LUT_CRC16+PDMEM00)
	lsr		REG_TMP0.b0, SHORT_MSG_CRC_H, 4
	xor		REG_TMP0.b0, REG_TMP0.b0, SEND_PARA
	lsl		REG_TMP0.w0, REG_TMP0.b0, 1
	lbbo		&REG_TMP0.w0, REG_TMP2, REG_TMP0.w0, 2
	lsl		SHORT_MSG.crc, SHORT_MSG.crc, 4
	xor		SHORT_MSG.crc, SHORT_MSG.crc, REG_TMP0.w0
;flip lower crc byte (only once)
	qbne		transport_layer_send_sending_msg_crc_dont_flip, SHORT_MSG.bits_left, 16
	xor		SHORT_MSG_CRC_L, SHORT_MSG_CRC_L, 0xff
transport_layer_send_sending_msg_crc_dont_flip:
	qba		transport_layer_send_msg_end
transport_layer_check_for_new_short_msg:
;check for SLAVE_REG_CTRL if we read/write data

	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SLAVE_REG_CTRL, 1
	qbeq		transport_layer_no_short_msg, REG_TMP0.b0, 0x3f
; Clear ONLINE_STATUS_1_FRES in ONLINE_STATUS_1 register
	lbco		&REG_TMP0.b2, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	clr		    REG_TMP0.b2, REG_TMP0.b2, (ONLINE_STATUS_1_FRES-8)
    sbco		&REG_TMP0.b2, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
;reset
	ldi		REG_TMP0.b1, 0x3f
	sbco		&REG_TMP0.b1, MASTER_REGS_CONST, SLAVE_REG_CTRL, 1
;only 6 bit address
	and		SHORT_MSG.addr, REG_TMP0.b0, 0x3f
;reset CRC
	ldi		SHORT_MSG.crc, 0

;MSB in Master Register identifies mode (r/w)
	qbbc		transport_layer_short_msg_write, REG_TMP0.b0, 7
;we read slave register
	or		SHORT_MSG.addr, SHORT_MSG.addr, (MSG_READ<<6)
;read short messages contain less content -> move addr part to data
	mov		SHORT_MSG.data, SHORT_MSG.addr
	ldi		SHORT_MSG.bits_left, (8*3)
	qba		transport_layer_short_msg_dir_end
transport_layer_short_msg_write:
;we write slave register -> DIR=0
;load data from S_PC_DATA for writing
	lbco		&REG_TMP0.b1, MASTER_REGS_CONST, S_PC_DATA, 1
	mov		SHORT_MSG.data, REG_TMP0.b1
;we write slave register -> DIR=0
	ldi		SHORT_MSG.bits_left, (8*4)
transport_layer_short_msg_dir_end:
	ldi		SHORT_MSG.timeout, 64
;set para channel to busy
	set		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
	qba		transport_layer_send_msg_end
transport_layer_no_short_msg:
;check for new long msg
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, PC_CTRL, 1
	qbbc		transport_layer_send_msg_end, REG_TMP0.b0, 0
	clr		REG_TMP0.b0, REG_TMP0.b0, 0
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, PC_CTRL, 1
;set para channel to busy
	set		H_FRAME.flags, H_FRAME.flags, FLAG_PARA_BUSY
; Clear ONLINE_STATUS_D_FREL in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	clr		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_FREL-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	lbco		&REG_TMP1, MASTER_REGS_CONST, PC_ADD_H, 4
; Bit 7 should be set for long message in PC_ADD_H
	set			REG_TMP1.b0, REG_TMP1.b0, 7
	mov		SHORT_MSG.addr, REG_TMP1.b0
	ldi		SHORT_MSG.bits_left, 16


;using PC_OFF?
	qbbc		transport_layer_assemble_long_msg_no_pc_off, REG_TMP1.b0, LOFF
; Bit 7 should be set for long message in PC_OFF_H
	set			REG_TMP1.b2, REG_TMP1.b2, 7
	add		SHORT_MSG.bits_left, SHORT_MSG.bits_left, 16
transport_layer_assemble_long_msg_no_pc_off:
;save ADDR(+OFF) to memory buffer

	lsr		REG_TMP0.b0, SHORT_MSG.bits_left, 3
	sbco		&REG_TMP1, MASTER_REGS_CONST, LONG_MSG_BUFFER, b0
;skip LLEN on read operation
	qbbs		transport_layer_assemble_long_msg_no_llen, SHORT_MSG.addr, 6
;get LLEN
	lsr		REG_TMP1.b0, REG_TMP1.b0, 2
	and		REG_TMP1.b0, REG_TMP1.b0, 0x03
	qbeq		transport_layer_assemble_long_msg_no_llen, REG_TMP1.b0, 0
	ldi		REG_TMP1.b1, 8
	lsl		REG_TMP1.b0, REG_TMP1.b1, REG_TMP1.b0;counting no. of bits length we need to send
	add		SHORT_MSG.bits_left, SHORT_MSG.bits_left, REG_TMP1.b0
	mov		REG_TMP0.b2, REG_TMP0.b0
	lsr		REG_TMP0.b0, REG_TMP1.b0, 3
	lbco		&REG_TMP1, MASTER_REGS_CONST, PC_BUFFER0, b0
	add		REG_TMP0.b2, REG_TMP0.b2, LONG_MSG_BUFFER
	;Store the data at LONG_MSG_BUFFER+4
	sbco		&REG_TMP1, MASTER_REGS_CONST, REG_TMP0.b2, b0
transport_layer_assemble_long_msg_no_llen:
;crc
	add		SHORT_MSG.bits_left, SHORT_MSG.bits_left, 16
;load ptr for memory buffer
	ldi		SHORT_MSG.data, LONG_MSG_BUFFER
	ldi		SHORT_MSG.crc, 0


transport_layer_send_msg_end:

; Check for Low QM value and update event registers if required
; Check EVENT_UPDATE_PENDING_QMLW to process a low QM value event update
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_UPDATE_PENDING, 1
    qbbc        transport_layer_no_qmlw_event, REG_TMP0.b0, EVENT_UPDATE_PENDING_QMLW
; Set EVENT_QMLW in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_QMLW
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int3, REG_TMP0.w2, EVENT_QMLW
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int3:
; Set EVENT_S_QMLW in EVENT_S register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 2
	set		    REG_TMP0.b0, REG_TMP0.b0, EVENT_S_QMLW
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int16, REG_TMP0.b1, EVENT_S_QMLW
; generate interrupt_s
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int16:
;Clear EVENT_UPDATE_PENDING_QMLW
	clr         REG_TMP0.w0, REG_TMP0.w0, EVENT_UPDATE_PENDING_QMLW
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_UPDATE_PENDING, 1
transport_layer_no_qmlw_event:

; Update QMLW bits in ONLINE_STATUS registers
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, MASTER_QM, 1
    and	        REG_TMP0.b0, REG_TMP0.b0, 0x7f
; Set QMLW if value is < 14
    qble	    transport_layer_online_status_qm_not_low, REG_TMP0.b0, 14
; Set QMLW bits
	lbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6
    set         REG_TMP0.w0, REG_TMP0.w0, ONLINE_STATUS_D_QMLW
    set         REG_TMP0.w2, REG_TMP0.w2, ONLINE_STATUS_1_QMLW
    set         REG_TMP1.w0, REG_TMP1.w0, ONLINE_STATUS_2_QMLW
	sbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6
    qba         transport_layer_online_status_qm_update_done
transport_layer_online_status_qm_not_low:
; Clear QMLW bits
	lbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6
    clr         REG_TMP0.w0, REG_TMP0.w0, ONLINE_STATUS_D_QMLW
    clr         REG_TMP0.w2, REG_TMP0.w2, ONLINE_STATUS_1_QMLW
    clr         REG_TMP1.w0, REG_TMP1.w0, ONLINE_STATUS_2_QMLW
	sbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6
transport_layer_online_status_qm_update_done:
	.if $defined("HDSL_MULTICHANNEL")
	CALL3 PUSH_FIFO_2B_8x
	.endif
; update POS bits in ONLINE_STATUS_D and EVENT
    lbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	qbbc		no_update_for_POS_bit,REG_TMP1.b0,ONLINE_STATUS_D_POS
	; Set EVENT_POS in EVENT register
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_POS
no_update_for_POS_bit:
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int14, REG_TMP0.w2, EVENT_POS
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int14:
    sbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
	jmp		transport_layer_send_msg_done


;--------------------------------------------------
;v_frame calculations
;102+20=122 cycles
;extracts safe position
;verifies fast position
;NOT allowed to use REG_TMP11
;--------------------------------------------------
transport_on_v_frame:
;save REG_FNC.w0 content
	mov		REG_TMP11.w1, REG_FNC.w0
;inc NUM_VERT_FRAMES -> could be replaced with PA STATS on ICSS_G
    .if 0
	lbco	&REG_TMP1, MASTER_REGS_CONST, NUM_VERT_FRAMES0, 4
	add		REG_TMP1, REG_TMP1, 1
	sbco	&REG_TMP1, MASTER_REGS_CONST, NUM_VERT_FRAMES0, 4
    .endif
;store CRC in Master  Registers
	mov		REG_TMP1.b0, VERT_L.b1
	mov		REG_TMP1.b1, VERT_L.b0
; Store the required data for vertical channel in temporary memory.
; It will be stored to DMEM in transport_on_v_frame_2
	sbco		&REG_TMP1, MASTER_REGS_CONST, VPOSCRC_TEMP, 2
;transmission error?
	qbbs		transport_on_v_frame_dont_update_qm, H_FRAME.flags, FLAG_ERR_VERT
    lbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 1
    and         REG_TMP2.b0, REG_TMP2.b0, (~((1<<ONLINE_STATUS_1_SCE) | (1<<ONLINE_STATUS_1_VPOS)) & 0xFF)
;checking for crc error
	qbeq		check_for_slave_error_on_v_frame, CRC_VERT, 0
; Set EVENT_S_SCE in EVENT register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_SCE
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S_TEMP, 1
; Set ONLINE_STATUS_1_SCE in ONLINE_STATUS_1 register
    set         REG_TMP2.b0, REG_TMP2.b0, ONLINE_STATUS_1_SCE
    sbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H_TEMP, 1
	QM_SUB		6
transport_on_v_frame_dont_update_qm:
;update CRC error count
	.if 0
	lbco		&REG_TMP1, MASTER_REGS_CONST, NUM_VERT_ERR0, 4
	add		REG_TMP1, REG_TMP1, 1
	sbco		&REG_TMP1, MASTER_REGS_CONST, NUM_VERT_ERR0, 4
	.endif
	qba		transport_on_v_frame_exit
check_for_slave_error_on_v_frame:

;CRC was correct -> add 1 to QM
;Note: QM_ADD uses REG_TMP1
	QM_ADD		1

;check for special character: K29.7 is sent in first byte of vertical channel if slave error occured
    mov		REG_TMP0.b0, VERT_H.b2
	mov		REG_TMP0.b1, VERT_H.b1
	mov		REG_TMP0.b2, VERT_H.b0
	mov		REG_TMP0.b3, VERT_L.b3
	mov		REG_TMP1.b0, VERT_L.b2
; Store the required data for vertical channel in memory
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, VPOS_TEMP, 5
	;VPOS data stored from high to low byte in order of VERT_H.b2,VERT_H.b1,VERT_H.b0,VERT_L.b3,VERT_L.b2
	qbne		transport_on_v_frame_check_pos, VERT_H.b2, K29_7
; Set EVENT_S_VPOS in EVENT register, due to encoder internal error
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_VPOS
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S_TEMP, 1
; Set ONLINE_STATUS_1_VPOS in ONLINE_STATUS_1 register
    set         REG_TMP2.b0, REG_TMP2.b0, ONLINE_STATUS_1_VPOS
    sbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H_TEMP, 1
	qba		transport_on_v_frame_exit
transport_on_v_frame_check_pos:
	sbco		&REG_TMP2.b0, MASTER_REGS_CONST, ONLINE_STATUS_1_H_TEMP, 1

transport_on_v_frame_realign:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff
	PUSH_FIFO_CONST  0xff
	CALL1 re_align_algo
	PUSH_FIFO_CONST  0xff

transport_on_v_frame_no_pos_mismatch:
; Store the required data for secondary channel in temporary memory.
; It will be processed in transport_on_v_frame_2
; store H_FRAME.flags
	sbco	&H_FRAME.flags, MASTER_REGS_CONST, H_FRAME_FLAGS_TEMP, 2
; store CRC_SEC
	sbco	&CRC_SEC, MASTER_REGS_CONST, CRC_SEC_TEMP, 2
; store the 8 bytes from secondary channel
	sbco	&CHANNEL.ch_secl, MASTER_REGS_CONST, VPOS2_TEMP, 8
	jmp no_first_push_for_exit
transport_on_v_frame_exit:
;we are in RX0
;reset rel. pos
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff
	PUSH_FIFO_CONST  0xff
	CALL1 re_align_algo
	PUSH_FIFO_CONST  0xff
Wait_and_Push_2_byte:
no_first_push_for_exit:
    .if $defined("HDSL_MULTICHANNEL")
	qbeq			free_run_mode1, EXTRA_SIZE, 0
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	RESET_CYCLCNT
free_run_mode1:
	.endif
	ldi		REG_TMP0, 0
	sbco		&REG_TMP0, MASTER_REGS_CONST, REL_POS0, 4
;store last FAST_POS
	sbco		&FAST_POSL, MASTER_REGS_CONST, LAST_FAST_POS0, SIZE_FAST_POS
; Store summary
    sbco    &VERT_H.b3, MASTER_REGS_CONST, SAFE_SUM_TEMP, 1
;restore REG_FNC.w0 content
	mov		REG_FNC.w0, REG_TMP11.w1
;reset vertical/secondary channel crc
	ldi		CRC, 0
;reset flags
	and		H_FRAME_flags_l, H_FRAME_flags_l, FLAG_ERRORS
	jmp		datalink_transport_on_v_frame_done



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The actual realignment algorithm pseudocode
;;This is a simplified version without the estimator and without handling invalid acc values
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;     global fastpos, vel, relpos, xreg, ALIGN_PHASE, uvel, upos, avel, epos, oldvpos, curvpos, dvpos, pq_state

;    if acc_valid == 1:
;        vel = vel + acc_update
;    else:
;        # restart alignment process
;        ALIGN_PHASE = 0
;
;    fastpos = fastpos + vel
;
;    # Start of a new VPOS cycle, reset relpos
;    if vpos_part == 0:
;        relpos = vel
;    else:
;        relpos = relpos + vel
;
;    uvel = uvel + maxacc
;    upos = upos + uvel
;
;    avel = avel + dvpos
;    epos = curvpos + (avel>>3)
;
;    # End of a VPOS cycle, we have a new complete VPOS
;    if vpos_part == 7:
;        if ALIGN_PHASE == 0:
;            print("Executing Align Phase 0.")
;            if vpos_valid == True:
;                xreg = vpos_update + relpos
;                fastpos = xreg
;                ALIGN_PHASE = 1
;        else :
;            if ALIGN_PHASE == 1:
;                print("Executing Align Phase 1.")
;                ALIGN_PHASE = 2
;                xreg = vpos_update - xreg
;                vel = vel + (xreg>>3)
;                xreg = xreg*2
;                fastpos = fastpos + xreg
;            if ALIGN_PHASE == 2:
;                print("Executing Align Phase 2.")
;                ALIGN_PHASE = 3
;            else:
;                if ALIGN_PHASE == 3:
;                    if relpos+vpos_update != fastpos:
;                        print("Check Align Phase 3: failed", relpos+vpos_update, '!=', fastpos)
;                        ALIGN_PHASE = 0
;                    else:
;                        print("Check Align Phase 3: succeeded", relpos+vpos_update, '==', fastpos, "whole last 8 packets were synchronous")
;
;    if ALIGN_PHASE == 3:
;        print("Aligned Fastpos:", fastpos, "\tVel:", vel, "\tMaxdev:", "0")
;    else:
;        print("Estimated Fastpos:", epos, "\tVel:", dvpos>>3, "\tMaxdev:", upos>>8)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
re_align_algo:
	lbco 		&r0.b0, MASTER_REGS_CONST,VPOS_TEMP, 1
	qbeq		alignment_check_failed,r0.b0,K29_7
	lbco 		&ALIGN_PHASE, MASTER_REGS_CONST, ALIGN_PH, 1
	.if $defined("HDSL_CHECK_ALIGNMENT_PHASE")
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, CURRENT_ALIGN_PHASE, 1
	.endif
	qbeq		align_ph0,ALIGN_PHASE,0
	qbeq		align_ph1,ALIGN_PHASE,1
	qbeq		align_ph2,ALIGN_PHASE,2
	qbeq		align_ph3,ALIGN_PHASE,3
	jmp alignment_check_failed
;Align phase 0
align_ph0:
	;XREG= vpos_update+relpos
	;FAST_POS=XREG
	lbco		&REG_TMP0, MASTER_REGS_CONST, REL_POS0, 4
	lbco		&REG_TMP1, MASTER_REGS_CONST, VPOS_TEMP, 5
	add		FAST_POSL, REG_TMP2.b0, REG_TMP0.b0
	adc		FAST_POSH.b0, REG_TMP1.b3, REG_TMP0.b1
	adc		FAST_POSH.b1, REG_TMP1.b2, REG_TMP0.b2
	adc		FAST_POSH.b2, REG_TMP1.b1, REG_TMP0.b3
;sign extend relative position to 40 bits
	qbbc	estimator_vpos_add_relpos_positive, REG_TMP0, 31
	adc		FAST_POSH.b3, REG_TMP1.b0, 0xFF
    qba     estimator_vpos_add_relpos_done
estimator_vpos_add_relpos_positive:
	adc		FAST_POSH.b3, REG_TMP1.b0, 0
estimator_vpos_add_relpos_done:
	;Store XREG
	ldi ALIGN_PHASE,1
	sbco	&FAST_POSL,MASTER_REGS_CONST,XREG,5
	jmp 	phase_completed
;Align phase 1
align_ph1:
	ldi ALIGN_PHASE,2
	lbco		&REG_TMP0, MASTER_REGS_CONST, XREG, 5
	lbco		&REG_TMP1.b1, MASTER_REGS_CONST, VPOS_TEMP, 5
	;;XREG=vpos_update-XREG
	sub		REG_TMP0.b0, REG_TMP2.b1, REG_TMP0.b0
	suc		REG_TMP0.b1, REG_TMP2.b0, REG_TMP0.b1
	suc		REG_TMP0.b2, REG_TMP1.b3, REG_TMP0.b2
	suc		REG_TMP0.b3, REG_TMP1.b2, REG_TMP0.b3
	suc		REG_TMP1.b0, REG_TMP1.b1, REG_TMP1.b0
	sbco	&REG_TMP0,MASTER_REGS_CONST,XREG,5
;;vel = vel + (xreg>>3)
;we use only first 3 bytes of XREG as vel is of 3 bytes and XREG will be of 3 byte order in this state.
	lsr REG_TMP0,REG_TMP0,3
	add		SPEED.w0, SPEED.w0, REG_TMP0.w0
	adc		SPEED.b2, SPEED.b2, REG_TMP0.b2
	lbco	&REG_TMP0,MASTER_REGS_CONST,XREG,5
;; xreg=xreg<<1
;;1. Perform left shift of 5th byte of XREG by 1
;;2. copy XREG's 7th bit of 4th byte to 0th bit of 5th byte
;;3. Perform left shift on first 4 bytes of XREG
	lsl REG_TMP1.b0,REG_TMP1.b0,1
	qbbs xreg_31_bit_set,REG_TMP0,31
	clr REG_TMP1,REG_TMP1,0
	qba xreg_shift_complete
xreg_31_bit_set:
	set REG_TMP1,REG_TMP1,0
xreg_shift_complete:
	lsl REG_TMP0,REG_TMP0,1
	sbco	&REG_TMP0,MASTER_REGS_CONST,XREG,5
;;FPOS=FPOS+XREG
	add		FAST_POSL, FAST_POSL, REG_TMP0.b0
	adc		FAST_POSH.b0, FAST_POSH.b0, REG_TMP0.b1
	adc		FAST_POSH.b1, FAST_POSH.b1, REG_TMP0.b2
	adc		FAST_POSH.b2, FAST_POSH.b2, REG_TMP0.b3
	adc		FAST_POSH.b3, FAST_POSH.b3, REG_TMP1.b0
	jmp 	phase_completed
;Align phase 2
align_ph2:
	ldi ALIGN_PHASE,3
	jmp 	phase_completed
;Align phase 3
align_ph3:
;;;relpos+vpos_update
	lbco		&REG_TMP0, MASTER_REGS_CONST, REL_POS0, 4
	lbco		&REG_TMP1.b1, MASTER_REGS_CONST, VPOS_TEMP, 5
	add		REG_TMP1.b0, REG_TMP2.b1, REG_TMP0.b0
	adc		REG_TMP0.b0, REG_TMP2.b0, REG_TMP0.b1
	adc		REG_TMP0.b1, REG_TMP1.b3, REG_TMP0.b2
	adc		REG_TMP0.b2, REG_TMP1.b2, REG_TMP0.b3
;sign extend relative position to 40 bits
	qbbc	estimator_vpos_add_relpos_positive_1, REG_TMP0, 31
	adc		REG_TMP0.b3, REG_TMP1.b1, 0xFF
    qba     estimator_vpos_add_relpos_done_1
estimator_vpos_add_relpos_positive_1:
	adc		REG_TMP0.b3, REG_TMP1.b1, 0
estimator_vpos_add_relpos_done_1:
	lbco		&REG_TMP2, MASTER_REGS_CONST, MASK_POS, 4
check_1st_byte:
;if relpos+vpos_update != fastpos:
	qbne		alignment_check_failed, REG_TMP1.b0, FAST_POSL
	xor 	REG_TMP0,REG_TMP0,FAST_POSH
	and 	REG_TMP2,REG_TMP2,REG_TMP0
	qbne 	alignment_check_failed,REG_TMP2,0
update_online_status_D:
;Update ONLINE_STATUS_D_H for clearing ONLINE_STATUS_D_POS bit
    lbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
transport_layer_no_pos_event:
    clr         REG_TMP1.b0, REG_TMP1.b0, ONLINE_STATUS_D_POS
transport_layer_pos_update_done:
    sbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
	qba 		alignment_done
alignment_check_failed:
	ldi ALIGN_PHASE,0
	;sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, ALIGN_PH, 1
	.if $defined("HDSL_CHECK_ALIGNMENT_PHASE")
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, CURRENT_ALIGN_PHASE, 1
	.endif
	lbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
    set 		REG_TMP1.b0,REG_TMP1.b0,ONLINE_STATUS_D_POS
    sbco        &REG_TMP1.b0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 1
phase_completed:
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, ALIGN_PH, 1
alignment_done:
not_7th_hframe:
	RET1
