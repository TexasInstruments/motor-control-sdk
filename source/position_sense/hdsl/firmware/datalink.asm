
;
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
	.ref re_align_algo
	.ref transport_on_h_frame
	.ref transport_on_v_frame
	.ref transport_on_v_frame_2
    .ref transport_layer_processing_1
    .ref transport_layer_processing_2
	.ref calc_acc_crc
	.ref calc_speed
	.ref calc_fastpos
	.ref transport_layer
	.ref transport_layer_send_msg
	.ref transport_layer_recv_msg
	.ref load_code
	.ref datalink_reset
	.global PUSH_FIFO_2B_8x
	.global PUSH_FIFO_3_8x
	.global WAIT_TX_FIFO_FREE
	.global transport_layer_send_msg_done
	.global transport_layer_recv_msg_done
	.global transport_layer_done
	.global datalink_abort
	.global datalink_abort2
	.global qm_add
	.global datalink_transport_on_v_frame_done
	.global datalink_transport_on_v_frame_done_2
    .global transport_layer_processing_1_done
    .global transport_layer_processing_2_done
    .if !$defined("HDSL_MULTICHANNEL")
    .global update_events
    .endif
	.global datalink_init
	.global qm_add
	.global calc_rssi
	.global send_stuffing
	.global send_header
	.global send_header_modified
	.global send_header_300m
	.global send_trailer
	.global wait_delay
	.global datalink_loadfw
	.global recv_dec
	.global main
	.global datalink_wait_vsynch
	.global send_01
	.global int_div
	.global sync_pulse
	.global check_test_pattern
	.global datalink_abort_jmp
	.global receive

;Initialize connection and state machine here
datalink_init:
;--------------------------------------------------------------------------------------------------
	; part 2 code starts here

	.sect ".text:part2"
	jmp	main
;--------------------------------------------------------------------------------------------------
;--------------------------------------------------------------------------------------------------
;State RX 0-7
datalink_wait_vsynch:
	ldi ALIGN_PHASE,0
	sbco 		&ALIGN_PHASE, MASTER_REGS_CONST, ALIGN_PH, 1
	zero			&SPEED, (4*8)
	ldi			LOOP_CNT.b1, 7
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=1,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0400 | M_PAR_INIT)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL			recv_dec
;check for timeout
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qbeq			datalink_abort_jmp, LOOP_CNT.b1, 0
	CALL1			send_stuffing
;check for error in parameter channel (byte 1)
	qbbs			datalink_wait_vsynch, BYTE_ERROR, BYTE_CH_PARAMETER
;try again if vsync is not set
	qbbc			datalink_wait_vsynch, H_FRAME.s_par, 7
	;qba			datalink_wait_vsynch
;received vsync - now set msync for synchronization
	set			H_FRAME.flags, H_FRAME.flags, FLAG_MSYNC
;message can be sent now
; Set EVENT_S_FRES in EVENT_S register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_FRES
;save events
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int0, REG_TMP0.b1, EVENT_S_FRES
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int0:
; Set ONLINE_STATUS_1_FRES in ONLINE_STATUS_1 register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_1_FRES-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_1_L), 1
; Set EVENT_FREL in EVENT_L register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_FREL
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int1, REG_TMP0.w2, EVENT_FREL
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int1:
; Set ONLINE_STATUS_D_FREL in ONLINE_STATUS_D register
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
	set		    REG_TMP0.b0, REG_TMP0.b0, (ONLINE_STATUS_D_FREL-8)
    sbco		&REG_TMP0.b0, MASTER_REGS_CONST, (ONLINE_STATUS_D_L), 1
;--------------------------------------------------------------------------------------------------
;State RX0-RX7
	ldi			LOOP_CNT.w2, 8
	ldi			SEND_PARA, M_PAR_IDLE
	ldi			CRC, 0
datalink_rx0_7:
	ldi			REG_FNC.b1, 0x4
;check if we set MSYNC
	qbbc			datalink_rx0_7_no_msync, H_FRAME.flags, FLAG_MSYNC
	set			REG_FNC.b1, REG_FNC.b1, 1
datalink_rx0_7_no_msync:
;flip MSYNC in rx7
	qbne			datalink_rx0_7_not_rx7, LOOP_CNT.b2, 1
	ldi			REG_TMP0, (1<<FLAG_MSYNC)
	xor			H_FRAME.flags, H_FRAME.flags, REG_TMP0
datalink_rx0_7_not_rx7:
;reset BYTE_ERROR - but do not clear bit which shows if byte0 error occurred once in v frame (MSB)
	and			BYTE_ERROR, BYTE_ERROR, 0x80
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=1,vsync=1,reserved=0
	mov			REG_FNC.b0, SEND_PARA
	qbeq           modified_header, MODIFIED_HEADER_STARTED, 1
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	ldi            MODIFIED_HEADER_STARTED, 1
	qba            header_send_done
modified_header:
	CALL			send_header_modified
	and             R0,R0,R0
header_send_done:
	CALL			recv_dec
;check for vsync and update quality monitor - should only be set in rx7
	qbbs			datalink_rx0_7_vsync_continue, BYTE_ERROR, BYTE_CH_PARAMETER
	qbbc			datalink_rx0_7_vsync_not_set, H_FRAME.s_par, 7
	qbeq			datalink_rx0_7_vsync_inc_qm, LOOP_CNT.b2, 1
	QM_SUB			4
	qba			datalink_rx0_7_vsync_continue
datalink_rx0_7_vsync_not_set:
	qbne			datalink_rx0_7_vsync_continue, LOOP_CNT.b2, 1
	QM_SUB			6
	qba			datalink_rx0_7_vsync_continue
datalink_rx0_7_vsync_inc_qm:
	QM_ADD			1
datalink_rx0_7_vsync_continue:
	;reset flags
	clr			H_FRAME.flags, H_FRAME.flags, FLAG_ERR_ACC
	CALL1			send_stuffing

	.if !$defined("HDSL_MULTICHANNEL") ;Single channel(225m) starts here
;sending sync and 2 bits of sample early to buy processing time for h frame processing
	qbeq			modified_header_early_data_push_free_run, EXTRA_SIZE, 0
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x2f
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	RESET_CYCLCNT
	qba modified_header_early_data_push_done
modified_header_early_data_push_free_run:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x2f
modified_header_early_data_push_done:
     ;READ_CYCLCNT		r19
;go to H-Frame callback on transport layer (max. 120-50=70 cycles)
 	CALL			transport_on_h_frame
	sub			LOOP_CNT.b2, LOOP_CNT.b2, 1
	qblt			datalink_rx0_7, LOOP_CNT.b2, 0
;V-Frame ends here
;reset BYTE_ERROR
	ldi			BYTE_ERROR, 0x00
;debug cnt
	add			LOOP_CNT.b3, LOOP_CNT.b3, 1
	ldi			LOOP_CNT.b2, 8
	set			H_FRAME.flags, H_FRAME.flags, FLAG_NORMAL_FLOW
	qba			datalink_rx0_7
	.else ;Single channel ends here
	;Multi-channel starts here

;sending sync and 2 bits of sample early to buy processing time for h frame processing
	;;PUSH 8 bytes for 1 byte data (0x2f) in FIFO
	;;last 6 bytes not being pushed in continous manner(some PRU code being executed and then PUSH a byte to stop FIFO underrun issue)

modified_header_early_data_push_done:
;go to H-Frame callback on transport layer (max. 120-50=70 cycles)
 	CALL			transport_on_h_frame
	sub		LOOP_CNT.b2, LOOP_CNT.b2, 1
	qbeq 	hframe_7_fifo_push,LOOP_CNT.b2,7
	qblt 	Hframe_fifo_push, LOOP_CNT.b2, 0
	jmp 	Vframe_fifo_push

Hframe_fifo_push:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff
	PUSH_FIFO_CONST  0x00
	LOOP push_2b_0_0 ,4
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff

push_2b_0_0:
	qbeq			free_run_mode_0, EXTRA_SIZE, 0
	PUSH_FIFO_CONST		0xff
free_run_mode_0:
	RESET_CYCLCNT
	qba			generate_h_frame_interrupt
;V-Frame ends here
;reset BYTE_ERROR
Vframe_fifo_push:
	PUSH_FIFO_CONST  0xff
	ldi			BYTE_ERROR, 0x00
	ldi			LOOP_CNT.b2, 8
	set			H_FRAME.flags, H_FRAME.flags, FLAG_NORMAL_FLOW
	qba			generate_h_frame_interrupt
hframe_7_fifo_push:
	qbbc			Hframe_fifo_push, H_FRAME.flags, FLAG_NORMAL_FLOW
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff
	PUSH_FIFO_CONST  0x00
generate_h_frame_interrupt:

; signal event mst_intr[0] and PRU0_ARM_IRQ3
	ldi     r31.w0, 32+0
	ldi     r31.w0, PRU0_ARM_IRQ3
	qba datalink_rx0_7
 	.endif ;Multichannel
;--------------------------------------------------------------------------------------------------

;Reroute data link abort to avoid branching error.
datalink_abort_jmp:
    jmp datalink_abort
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
;Function: switch_clk (RET_ADDR1)
;
;input:
;	REG_FNC.w0: new clk_div
;	REG_FNC.b2: #of cycles to wait before switching
;output:
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
switch_clk:
	.if !$defined("HDSL_MULTICHANNEL")
	WAIT_CLK_LOW		REG_TMP0
	WAIT_CLK_HIGH		REG_TMP0
	sub			REG_FNC.b2, REG_FNC.b2, 1
	qbne			switch_clk, REG_FNC.b2, 1
	WAIT_CLK_LOW		REG_TMP0
	;ldi     		REG_SCRATCH, P0EDTXCFG+2
	sbco			&REG_FNC.w0, ICSS_CFGx, EDTXCFG+2, 2
	WAIT_CLK_HIGH		REG_TMP0
	.endif
	RET1



; common code starts here
	.sect ".text"



;--------------------------------------------------------------------------------------------------
;Function: receive
;This functions receives data without deocoding.
;output:
;	r20-r19: data
;--------------------------------------------------------------------------------------------------
receive:
	ldi			LOOP_CNT.b0, 32
	CALL1		wait_delay
	ldi			REG_TMP0.w0, 0
	ldi			REG_TMP11, (PDMEM00+0x5a4);LUT_B2B)
	zero		&r18, (4*5)
datalink_receive_signal_0_31_1:
	qbbc			datalink_receive_signal_0_31_1, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			datalink_receive_signal_0_31_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r20, r20, LOOP_CNT.b0
datalink_receive_signal_0_31_received_0_1:
	mov			REG_TMP0.b1, REG_TMP0.b0
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1		calc_rssi
	qbne		datalink_receive_signal_0_31_1, LOOP_CNT.b0, 0
;receive next bits
	ldi			LOOP_CNT.b0, 29
datalink_receive_signal_32_60_1:
	qbbc			datalink_receive_signal_32_60_1, r31, RX_VALID_FLAG					;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			datalink_receive_signal_32_60_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r19, r19, LOOP_CNT.b0
datalink_receive_signal_32_60_received_0_1:
	mov			REG_TMP0.b1, REG_TMP0.b0
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			datalink_receive_signal_32_60_1, LOOP_CNT.b0, 1
datalink_receive_signal_last_1:
	qbbc			datalink_receive_signal_last_1, r31, RX_VALID_FLAG					;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	qbbc			datalink_receive_signal_last_received_0_1, REG_TMP0.w0, SAMPLE_EDGE
	set			r19, r19, 0
datalink_receive_signal_last_received_0_1:
	lsl			r19, r19, 3
	CLEAR_VAL
; same delay code as in learn
	ldi			REG_TMP1, (74*CYCLES_BIT+9);-7=14iters;-35  ;-34; -9 for 100 m
	READ_CYCLCNT		REG_TMP0
	qble	    receive_skip_wait, REG_TMP0, REG_TMP1
	sub			REG_TMP0, REG_TMP1, REG_TMP0
	;sub 		REG_TMP0,REG_TMP0,28  ;28- 13
	add			r0,r0,1
	WAIT		REG_TMP0
receive_skip_wait:
	TX_EN
; datalink_receive_signal_no_delay_wait_1:
	CALL1			send_trailer
	CALL1			send_stuffing
	RET

;--------------------------------------------------------------------------------------------------
;Function: send_01 (RET_ADDR1)
;This function sends 01 pattern in RESET and SYNC state
;input:
;	REG_FNC.b2: last two bits of parameter channel
;output:
;modifies:
;--------------------------------------------------------------------------------------------------
send_01:
;send 01 pattern
;2 para bits, 1 switch bit, 5 slave bit
	or			REG_FNC.b2, REG_FNC.b2, 0x15;0bPPS10101
    .if $defined("HDSL_MULTICHANNEL")
	mov FIFO_L,REG_FNC.b2
	loop dd1,4
	CALL3 PUSH_FIFO_2B_8x
dd1:
    .else
	PUSH_FIFO		REG_FNC.b2
    .endif
;56+12 line delay slave bits
	ldi			REG_TMP0.b0, 8
send_header_send_01_pattern_loop:
	;;PUSH 8 bytes for 1 byte data (0x55) in FIFO

    .if $defined("HDSL_MULTICHANNEL")
	ldi FIFO_L,0x55
	loop aaa9,4
	CALL3 PUSH_FIFO_2B_8x
aaa9:
    .else
    CALL2 WAIT_TX_FIFO_FREE
    PUSH_FIFO_CONST		0x55
    .endif
	sub			REG_TMP0.b0, REG_TMP0.b0, 1
	qbne			send_header_send_01_pattern_loop, REG_TMP0.b0, 0
;send last 0101 (4 bits)
	CALL2 WAIT_TX_FIFO_FREE
;overclock(8x)
	PUSH_FIFO_CONST		0x00
	ldi			REG_TMP0, (9*(CLKDIV_NORMAL+1)-9)
    .if !$defined("HDSL_MULTICHANNEL")
	WAIT			REG_TMP0
	TX_CLK_DIV		CLKDIV_FAST, REG_TMP0
    .endif
	PUSH_FIFO_CONST		0xff
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
;push TRAILER
	;PUSH 8 bytes for 1 byte data (0x03) in FIFO
	CALL2 WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
	ldi FIFO_L,0x03
	loop aaa8,4
	CALL3 PUSH_FIFO_2B_8x
aaa8:
    .else
    PUSH_FIFO_CONST		0x03
    .endif
    .if $defined("FREERUN_300_MHZ") | $defined("SYNC_300_MHZ")
	ldi			REG_TMP0, (6*(CLKDIV_FAST+1)-8+2)
    .else
    ldi			REG_TMP0, (6*(CLKDIV_FAST+1)-8)
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
    WAIT			REG_TMP0
	TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP0
    ;wait to have same timing as send_trailer
	ldi			REG_TMP0, 30
    WAIT			REG_TMP0
    .endif

;reset cyclecount
	RESET_CYCLCNT

	RET1


;--------------------------------------------------------------------------------------------------
;Function: recv_dec_10b (RET_ADDR1)
;This function receive 10 bits and decodes it while receiving
;input:
;	LOOP_CNT.b0: numberof iterations for 6b (usually 6, except for vertical channel)
;	REG_FNC.b1: byte number
;output:
;	REG_FNC.b0: decoded 8b data
;	DISPARITY
;	SPECIAL_CHARACTER
;	BYTE_ERROR
;modifies:
;	REG_TMP0, LOOP_CNT.b0
;--------------------------------------------------------------------------------------------------
recv_dec_10b:
;save RET1
	mov			REG_TMP2.w0, RET_ADDR1
	ldi			REG_FNC.w2, 0
;receive first 6 bits (abcdei)
recv_dec_10b_6b:
	qbbc			recv_dec_10b_6b, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			recv_dec_10b_6b_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, LOOP_CNT.b0
recv_dec_10b_6b_received_0:
recv_dec_10b_6b_skip_acc:
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			recv_dec_10b_6b, LOOP_CNT.b0, 0
;decode
	ldi			REG_TMP11, (PDMEM00+LUT_5b6b_DEC)
	lbbo			&REG_FNC.b0, REG_TMP11, REG_FNC.w2, 1
;error checks:
;abcd should not be equal
	and			REG_TMP0.b1, REG_FNC.w2, 0x3c
	qbeq			recv_dec_10b_6b_error, REG_TMP0.b1, 0x3c
	qbeq			recv_dec_10b_6b_error, REG_TMP0.b1, 0x00
	qba			recv_dec_10b_6b_no_error
recv_dec_10b_6b_error:
	set			BYTE_ERROR, BYTE_ERROR, REG_FNC.b1
recv_dec_10b_6b_no_error:

;get next 3 bits (fgh) and check for errors / special character
	lsl			REG_FNC.w2, REG_FNC.w2, 3
	ldi			LOOP_CNT.b0, 3
recv_dec_10b_4b_0_2:
	qbbc			recv_dec_10b_4b_0_2, r31, RX_VALID_FLAG ;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			recv_dec_10b_4b_0_2_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, LOOP_CNT.b0
recv_dec_10b_4b_0_2_received_0:
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			recv_dec_10b_4b_0_2, LOOP_CNT.b0, 0
;check for errors / special character
	ldi			REG_TMP1.b0, 0
	clr			SPECIAL_CHARACTER, SPECIAL_CHARACTER, REG_FNC.b1
;K=(c==d==e==i)
	and			REG_TMP0.b0, REG_FNC.w2, 0x78
	qbeq			recv_dec_10b_special_character0, REG_TMP0.b0, 0x78
	qbeq			recv_dec_10b_special_character0, REG_TMP0.b0, 0x00
	qba			recv_dec_10b_no_special_character0
recv_dec_10b_special_character0:
	set			SPECIAL_CHARACTER, SPECIAL_CHARACTER, REG_FNC.b1
recv_dec_10b_no_special_character0:
	and			REG_TMP0.b0, REG_FNC.w2, 0x1f
	qbeq			recv_dec_10b_eifgh_error, REG_TMP0.b0, 0x1f
	qbeq			recv_dec_10b_eifgh_error, REG_TMP0.b0, 0x00
	qba			recv_dec_10b_eifgh_no_error
recv_dec_10b_eifgh_error:
	set			BYTE_ERROR, BYTE_ERROR, REG_FNC.b1
recv_dec_10b_eifgh_no_error:
;get next 1 bits (j)
	lsl			REG_FNC.w2, REG_FNC.w2, 1
recv_dec_10b_4b_3:
	qbbc			recv_dec_10b_4b_3, r31, RX_VALID_FLAG ;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	qbbc			recv_dec_10b_4b_3_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, 0
recv_dec_10b_4b_3_received_0:
	lsr			REG_TMP1.b1, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b1, REG_TMP0.b0
	CALL1			calc_rssi
recv_dec_10b_4b_not_last_byte:
;decode
	and			REG_TMP0.w2, REG_FNC.w2, 0x0f
	ldi			REG_TMP11, (PDMEM00+LUT_3b4b_DEC)
	lbbo			&REG_TMP0.b0, REG_TMP11, REG_TMP0.w2, 1
	or			REG_FNC.b0, REG_FNC.b0, REG_TMP0.b0
;check for special character and flip
;K=(c==d==e==i) v (P13*e'*i*g*h*j) v (P31*e*i'*g'*h'*j')
	and			REG_TMP0.b0, REG_FNC.w2, 0x37
	xor			REG_TMP0.b0, REG_TMP0.b0, 0x20
	qbeq			recv_dec_10b_special_character1, REG_TMP0.b0, 0x37
	qbeq			recv_dec_10b_special_character1, REG_TMP0.b0, 0x00
	qba			recv_dec_10b_no_special_character1
recv_dec_10b_special_character1:
	set			SPECIAL_CHARACTER, SPECIAL_CHARACTER, REG_FNC.b1
recv_dec_10b_no_special_character1:
	xor			REG_FNC.b0, REG_FNC.b0, REG_TMP1.b0
;restore RET1
	mov			RET_ADDR1, REG_TMP2.w0
	RET1
;--------------------------------------------------------------------------------------------------
;Function: srecv_dec_10b (RET_ADDR1)
;This function receive 10 bits and decodes it while receiving
;input:
;	LOOP_CNT.b0: numberof iterations for 6b (usually 6, except for vertical channel)
;	REG_FNC.b1: byte number
;output:
;	REG_FNC.b0: decoded 8b data
;	DISPARITY
;	SPECIAL_CHARACTER
;	BYTE_ERROR
;modifies:
;	REG_TMP0, LOOP_CNT.b0
;--------------------------------------------------------------------------------------------------
srecv_dec_10b:
;save RET1
	mov			REG_TMP2.w0, RET_ADDR1
	ldi			REG_FNC.w2, 0
;receive first 6 bits (abcdei)
srecv_dec_10b_6b:
	qbbc			srecv_dec_10b_6b, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			srecv_dec_10b_6b_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, LOOP_CNT.b0
srecv_dec_10b_6b_received_0:


srecv_dec_10b_6b_skip_acc:
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			srecv_dec_10b_6b, LOOP_CNT.b0, 0
;decode
	ldi			REG_TMP11, (PDMEM00+LUT_5b6b_DEC)
	lbbo			&REG_FNC.b0, REG_TMP11, REG_FNC.w2, 1
;error checks:
;abcd should not be equal
	and			REG_TMP0.b1, REG_FNC.w2, 0x3c
	qbeq			srecv_dec_10b_6b_error, REG_TMP0.b1, 0x3c
	qbeq			srecv_dec_10b_6b_error, REG_TMP0.b1, 0x00
	qba			srecv_dec_10b_6b_no_error
srecv_dec_10b_6b_error:
	set			BYTE_ERROR, BYTE_ERROR, REG_FNC.b1
srecv_dec_10b_6b_no_error:

;get next 3 bits (fgh) and check for errors / special character
	lsl			REG_FNC.w2, REG_FNC.w2, 3
	ldi			LOOP_CNT.b0, 3
srecv_dec_10b_4b_0_2:
	qbbc			srecv_dec_10b_4b_0_2, r31, RX_VALID_FLAG ;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 1
	qbbc			srecv_dec_10b_4b_0_2_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, LOOP_CNT.b0
srecv_dec_10b_4b_0_2_received_0:
;get edges
	lsr			REG_TMP1.b0, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b0, REG_TMP0.b0
	CALL1			calc_rssi
	qbne			srecv_dec_10b_4b_0_2, LOOP_CNT.b0, 0

	and			REG_TMP0.b0, REG_FNC.w2, 0x1f
	qbeq			srecv_dec_10b_eifgh_error, REG_TMP0.b0, 0x1f
	qbeq			srecv_dec_10b_eifgh_error, REG_TMP0.b0, 0x00
	qba			srecv_dec_10b_eifgh_no_error
srecv_dec_10b_eifgh_error:
	set			BYTE_ERROR, BYTE_ERROR, REG_FNC.b1
srecv_dec_10b_eifgh_no_error:
;get next 1 bits (j)
	lsl			REG_FNC.w2, REG_FNC.w2, 1
srecv_dec_10b_4b_3:
	qbbc			srecv_dec_10b_4b_3, r31, RX_VALID_FLAG ;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	qbbc			srecv_dec_10b_4b_3_received_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_FNC.w2, REG_FNC.w2, 0
srecv_dec_10b_4b_3_received_0:
	lsr			REG_TMP1.b1, REG_TMP0.w0, 1
	xor			CUR_EDGES, REG_TMP1.b1, REG_TMP0.b0
	CALL1			calc_rssi
srecv_dec_10b_4b_not_last_byte:
;decode
	and			REG_TMP0.w2, REG_FNC.w2, 0x0f
	ldi			REG_TMP11, (PDMEM00+LUT_3b4b_DEC)
	lbbo			&REG_TMP0.b0, REG_TMP11, REG_TMP0.w2, 1
	or			REG_FNC.b0, REG_FNC.b0, REG_TMP0.b0
;restore RET1
	mov			RET_ADDR1, REG_TMP2.w0
;;check for special character and flip for 4 cases (when 6b==110000 and 4b= 0110,1010,0101,1001)
	and REG_TMP0.b2,REG_FNC.w2,0xf      ;REG_FNC.w2 stores 10b encoded data
	lsr			REG_TMP0.b0,REG_FNC.w2,4
	and			REG_TMP0.b0,REG_TMP0.b0,0x3f
	qbne		check_for_special_character_done,	REG_TMP0.b0,0x30
	qbeq 		flip_first_3bits_of_STATUS2, REG_TMP0.b2,0x6
	qbeq 		flip_first_3bits_of_STATUS2, REG_TMP0.b2,0xa
	qbeq 		flip_first_3bits_of_STATUS2, REG_TMP0.b2,0x5
	qbeq 		flip_first_3bits_of_STATUS2, REG_TMP0.b2,0x9
	qba 		check_for_special_character_done
flip_first_3bits_of_STATUS2:
	xor REG_FNC.b0,REG_FNC.b0,0xe0
check_for_special_character_done:
	RET1

;--------------------------------------------------------------------------------------------------
;Function: recv_dec (RET_ADDR)
;This function receive data and decodes it while receiving
;input:
;output:
;	H_FRAME: decoded H-Frame data
;modifies:
;	REG_TMP0, REG_FNC, LOOP_CNT.b0
;--------------------------------------------------------------------------------------------------
recv_dec:
	ldi			LOOP_CNT.b0, 6
	ZERO			&H_FRAME, 6
	CALL1			wait_delay
	ldi			REG_TMP0.w2, 0
;receive 10 bits from channel and decode, then receive next channel
;we cant receive 6 zeroes in a row due to encoding!
;receive vertical channel
recv_dec_first:
	ldi			REG_FNC.b1, 0
	CALL1		recv_dec_10b
	mov			H_FRAME.vert, REG_FNC.b0
;put data to CHANNEL - vertical channel
	lsl			CHANNEL.ch_verth, CHANNEL.ch_verth, 8
	lsr			REG_TMP0, CHANNEL.ch_vertl, 24
	or			CHANNEL.ch_verth, CHANNEL.ch_verth, REG_TMP0
	lsl			CHANNEL.ch_vertl, CHANNEL.ch_vertl, 8
	or			CHANNEL.ch_vertl, CHANNEL.ch_vertl, H_FRAME.vert
;calc running crc for vertical channel
	xor			CRC_VERT_H, CRC_VERT_H, H_FRAME.vert
	qbne		recv_dec_vertical_not_rx7, LOOP_CNT.b2, 1
;last byte of vertical channel: crc -> flip
	xor			CRC_VERT_H, CRC_VERT_H, 0xff
recv_dec_vertical_not_rx7:
	ldi			REG_TMP0, (LUT_CRC16+PDMEM00)
	lsl			REG_TMP1.w0, CRC_VERT_H, 1
	lbbo			&REG_TMP0.w0, REG_TMP0, REG_TMP1.w0, 2
	xor			REG_TMP0.b1, REG_TMP0.b1, CRC_VERT_L
	mov			CRC_VERT, REG_TMP0.w0
;receive parameter channel
	ldi			REG_FNC.b1, 1
	ldi			LOOP_CNT.b0, 6
	CALL1			recv_dec_10b
	mov			H_FRAME.s_par, REG_FNC.b0
;put data to CHANNEL - parameter channel, only lower 5 bits contain data information
	lsl			CHANNEL.ch_parah, CHANNEL.ch_parah, 5
	lsr			REG_TMP0, CHANNEL.ch_paral, 27
	or			CHANNEL.ch_parah, CHANNEL.ch_parah, REG_TMP0
	lsl			CHANNEL.ch_paral, CHANNEL.ch_paral, 5
	and			REG_TMP0, H_FRAME.s_par, 0x1f
	or			CHANNEL.ch_paral, CHANNEL.ch_paral, REG_TMP0
;receive pipe channel
	ldi			REG_FNC.b1, 2
	ldi			LOOP_CNT.b0, 6
	CALL1			recv_dec_10b
	mov			H_FRAME.pipe, REG_FNC.b0
	; Store PIPE data
	sbco	&H_FRAME.pipe, MASTER_REGS_CONST, PIPE_D, 1
;receive acceleration channel
	ldi			REG_FNC.b1, 3
	ldi			LOOP_CNT.b0, 6
	CALL1			recv_dec_10b
	mov			H_FRAME.acc, REG_FNC.b0
	ldi			REG_FNC.b1, 4
	ldi			LOOP_CNT.b0, 6
	CALL1		recv_dec_10b
	lsl			H_FRAME.acc, H_FRAME.acc, 8
	or			H_FRAME.acc, H_FRAME.acc, REG_FNC.b0
;one switch bit - always opposite of last bit
	ldi			REG_TMP0.w2, 0
datalink_receive_signal_swb_0:
	qbbc			datalink_receive_signal_swb_0, r31, RX_VALID_FLAG ;changed here from 24 to 26
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	qbbc			datalink_receive_signal_swb_received_0_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_TMP0.w2, REG_TMP0.w2, 0
datalink_receive_signal_swb_received_0_0:
;check for error in vertical channel
	qbbc			datalink_receive_signal_no_error_0, BYTE_ERROR, BYTE_CH_VERTICAL
	qbbs			datalink_receive_signal_no_error_0, BYTE_ERROR, 7
	set			BYTE_ERROR, BYTE_ERROR, 7
	qbbs			datalink_receive_signal_no_error_0, H_FRAME.flags, FLAG_ERR_VERT
	set			H_FRAME.flags, H_FRAME.flags, FLAG_ERR_VERT
	QM_SUB			6
	qba			recv_dec_vertical_no_special_character
datalink_receive_signal_no_error_0:
;vertical channel is only allowed to have K29.7 as special character
	qbbc			recv_dec_vertical_no_special_character, SPECIAL_CHARACTER, BYTE_CH_VERTICAL
	qbeq			recv_dec_vertical_no_special_character, H_FRAME.vert, K29_7
	QM_SUB			2
recv_dec_vertical_no_special_character:
;check for error in parameter channel
	qbbc			datalink_receive_signal_no_error_1, BYTE_ERROR, BYTE_CH_PARAMETER
	QM_SUB			1
	qba			recv_dec_para_no_special_character
datalink_receive_signal_no_error_1:
;check for special character: parameter channel is not allowed to have that
	qbbc			recv_dec_para_no_special_character, SPECIAL_CHARACTER, BYTE_CH_PARAMETER
	QM_SUB			2
recv_dec_para_no_special_character:
;check for error in pipe channel
	qbbc			datalink_receive_signal_no_error_2, BYTE_ERROR, BYTE_CH_PIPE
	QM_SUB			1
datalink_receive_signal_no_error_2:
;check for error in acceleration channel
	and			REG_TMP0, BYTE_ERROR, (0x03<<3)
	qbeq			datalink_receive_signal_no_error_3_4, REG_TMP0, 0
	set			H_FRAME.flags, H_FRAME.flags, FLAG_ERR_ACC
	QM_SUB			2
	qba			recv_dec_acc_no_special_character
datalink_receive_signal_no_error_3_4:
;vertical channel is only allowed to have K29.7 as special character
	qbbc			recv_dec_acc_check_special_character, SPECIAL_CHARACTER, BYTE_CH_ACC_H
	qbne			recv_dec_acc_special_character, H_FRAME_acc1, K29_7
recv_dec_acc_check_special_character:
	qbbc			recv_dec_acc_no_special_character, SPECIAL_CHARACTER, BYTE_CH_ACC_L
	qbeq			recv_dec_acc_no_special_character, H_FRAME_acc0, K29_7
recv_dec_acc_special_character:
	QM_SUB			2
recv_dec_acc_no_special_character:
;receive secondary channel
	ldi			REG_FNC.b1, 5
	ldi			LOOP_CNT.b0, 6
	CALL1			srecv_dec_10b
;switch to TX
	TX_EN
;wait 61+1sw+12delay bits - slave delay
	ldi			REG_TMP1, (74*CYCLES_BIT+9)  ; -9 for 100m
	READ_CYCLCNT		REG_TMP0
	qble	    datalink_receive_signal_no_delay_wait_0, REG_TMP0, REG_TMP1
	sub			REG_TMP0, REG_TMP1, REG_TMP0
	WAIT		REG_TMP0

datalink_receive_signal_no_delay_wait_0:
;send TRAILER
	CALL1			send_trailer
;put data to CHANNEL - secondary channel
	mov			H_FRAME.secondary, REG_FNC.b0
	lsl			CHANNEL.ch_sech, CHANNEL.ch_sech, 8
	lsr			REG_TMP0, CHANNEL.ch_secl, 24
	or			CHANNEL.ch_sech, CHANNEL.ch_sech, REG_TMP0
	lsl			CHANNEL.ch_secl, CHANNEL.ch_secl, 8
	or			CHANNEL.ch_secl, CHANNEL.ch_secl, H_FRAME.secondary
;check for error in secondary channel
	qbbc			datalink_receive_signal_no_error_5, BYTE_ERROR, BYTE_CH_SECONDARY
	qbbs			datalink_receive_signal_no_error_5, H_FRAME.flags, FLAG_ERR_SEC
	set			H_FRAME.flags, H_FRAME.flags, FLAG_ERR_SEC
	QM_SUB			8
datalink_receive_signal_no_error_5:
;calc running crc for secondary channel
	xor			CRC_SEC_H, CRC_SEC_H, H_FRAME.secondary
	qbne			recv_dec_secondary_not_rx7, LOOP_CNT.b2, 1
;last byte of secondary channel: crc -> flip
	xor			CRC_SEC_H, CRC_SEC_H, 0xff
recv_dec_secondary_not_rx7:
	ldi			REG_TMP0, (LUT_CRC16+PDMEM00)
	lsl			REG_TMP1.w0, CRC_SEC_H, 1
	lbbo			&REG_TMP0.w0, REG_TMP0, REG_TMP1.w0, 2
	xor			REG_TMP0.b1, REG_TMP0.b1, CRC_SEC_L
	mov			CRC_SEC, REG_TMP0.w0
	RET
;--------------------------------------------------------------------------------------------------
;Function: send_header (RET_ADDR)
;This function sends the header (data before slave answer)
;input:
;	REG_FNC.w0: parameter channel input
;output:
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
	.if !$defined("HDSL_MULTICHANNEL")
send_header:
;push SYNC and first 2 bits of SAMPLE

	PUSH_FIFO_CONST		0x2f
	;check if we have an EXTRA period
;if we have a EXTRA period: do TX FIFO synchronization here to gain processing time
	qbeq			send_header_no_extra_wait, EXTRA_SIZE, 0
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	RESET_CYCLCNT

send_header_modified:
send_header_no_extra_wait:
;calculate EQUALIZATION
	add			DISPARITY, DISPARITY, EXTRA_SIZE
	qbbs			send_header_disp_neg, DISPARITY, 7
send_header_disp_pos:
	qblt			send_header_disp_pos0, DISPARITY, 1
	ldi			REG_TMP11.b0, 0x60 ; 0x6a
	qba			send_header_end_disp
send_header_disp_pos0:
	qblt			send_header_disp_pos1, DISPARITY, 3
	ldi			REG_TMP11.b0, 0x62
	sub			DISPARITY, DISPARITY, 2
	qba			send_header_end_disp
send_header_disp_pos1:
	ldi			REG_TMP11.b0, 0x60
	sub			DISPARITY, DISPARITY, 4
	qba			send_header_end_disp
send_header_disp_neg:
	not			REG_TMP11.b0, DISPARITY
	add			REG_TMP11.b0, REG_TMP11.b0, 1
	qblt			send_header_disp_neg0, REG_TMP11.b0, 1
	ldi			REG_TMP11.b0, 0x60; 0x6a
	qba			send_header_end_disp
send_header_disp_neg0:
	qblt			send_header_disp_neg1, REG_TMP11.b0, 3
	ldi			REG_TMP11.b0, 0x6d
	add			DISPARITY, DISPARITY, 2
	qba			send_header_end_disp
send_header_disp_neg1:
	ldi			REG_TMP11.b0, 0x6f
	add			DISPARITY, DISPARITY, 4
send_header_end_disp:
;reset eCAP1 INT
	ldi			REG_TMP1.w0, (ECAP+ECAP_ECCLR)
	ldi			REG_TMP1.w2, 0xffff
	sbco			&REG_TMP1.w2, PWMSS1_CONST, REG_TMP1.w0, 2
	;sbco			&REG_TMP1.w2, PWMSS2_CONST, REG_TMP1.w0, 2
;HINT: we have some processing time here (140 cycles)
;go to V-Frame callback on transport layer
	qbbc			datalink_transport_no_v_frame_2, H_FRAME.flags, FLAG_NORMAL_FLOW
; Check if it V-Frame is complete (and currently the header for first H-Frame is
; being sent) -> state rx0 since we wait for processing v-frame in beginning of
; next frame and not at end of frame in rx7. If yes, start the V-frame
; processing. It will be completed in next H-Frame (State rx1). Therefore the
; V-frame processing is split into two parts : transport_on_v_frame and
; transport_on_v_frame_2.
	qbne			datalink_transport_no_v_frame, LOOP_CNT.b2, 8
	jmp			transport_on_v_frame
datalink_transport_on_v_frame_done:

datalink_transport_no_v_frame:
; Check if the first H-Frame is complete (and currently the header for second H-Frame
; is being sent). If yes, perform the remaining part of V-frame processing
	qbne			datalink_transport_no_v_frame_2, LOOP_CNT.b2, 7
 	qbbc			dont_push_for_non7_hframe_1, H_FRAME.flags, FLAG_NORMAL_FLOW

dont_push_for_non7_hframe_1:
    jmp             transport_on_v_frame_2

datalink_transport_on_v_frame_done_2:
datalink_transport_no_v_frame_2:

;check if we have an EXTRA period
	qbeq			send_header_no_extra, EXTRA_SIZE, 0

;********************************************************************************;
	;extra value decide starts
	qbeq        num_pulses_is_one3, NUM_PULSES, 1
	mov EXTRA_SIZE_SELF, EXTRA_SIZE
	ldi EXTRA_EDGE_SELF, 0
	qba         extra_value_decided
num_pulses_is_one3:
	mov EXTRA_SIZE_SELF, EXTRA_SIZE_COMP
	mov EXTRA_EDGE_SELF, EXTRA_EDGE_COMP
extra_value_decided:
	;extra value decide ends
	;remainder increament logic starts
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYNC_EXTRA_REMAINDER, 1 ;reload ES
	qbgt        remainder_increament_done1, REG_TMP0.b0, NUM_PULSES
	add         EXTRA_SIZE_SELF, EXTRA_SIZE_SELF, 1
remainder_increament_done1:
	;remainder increament logic ends
	qbne        extra_edge_calculation_for_self_done1, EXTRA_EDGE_SELF, 0
	sub         EXTRA_SIZE_SELF, EXTRA_SIZE_SELF, 1
	ldi         EXTRA_EDGE_SELF, 0xFF
extra_edge_calculation_for_self_done1:
	qba         send_header_extra_not_too_large

send_header_extra_not_too_large:
;limit STUFFING
	qbge			send_header_no_cap_stuffing, NUM_STUFFING, MAX_STUFFING
	ldi			NUM_STUFFING, MAX_STUFFING
send_header_no_cap_stuffing:
	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP0, c25, 0, 4
	add         REG_TMP0,REG_TMP0,4
	READ_CYCLCNT		REG_TMP1
	;lbco        &REG_TMP1, c1, 0x10, 4
	sbco        &REG_TMP1, c25, REG_TMP0, 4
	sbco        &REG_TMP0, c25, 0, 4
	.endif

;read cyclecount
	READ_CYCLCNT		REG_TMP1
	.if $defined(EXT_SYNC_ENABLE)
	qbeq           modified_header_wait, MODIFIED_HEADER_STARTED, 1
	ldi			   REG_TMP0, (9*(CLKDIV_NORMAL+1)-9-4-10)
	qba            modified_header_wait_done
modified_header_wait:
	ldi			REG_TMP0, (11*(CLKDIV_NORMAL+1))
modified_header_wait_done:
	.else
	ldi			REG_TMP0, (9*(CLKDIV_NORMAL+1)-9-4-4)
	.endif
	sub			REG_TMP0, REG_TMP0, REG_TMP1
	WAIT			REG_TMP0
;reset ECAP INT

send_header_extra_no_wait:
	TX_CLK_DIV		CLKDIV_FAST, REG_TMP0
	sub			REG_TMP1.b0, EXTRA_SIZE_SELF, 1

send_header_extra_loop:

	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	sub			REG_TMP1.b0, REG_TMP1.b0, 1
	qbne			send_header_extra_loop, REG_TMP1.b0, 0
	ldi			REG_TMP0, (11*(CLKDIV_FAST+1)-0)
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO		EXTRA_EDGE_SELF

send_header_extra_no_edge:
;reset clock to normal frequency
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO		REG_TMP11.b0
;skip synch pulse measurement if we generate pulse ourself
	WAIT			REG_TMP0
send_header_no_wait_after_synch:
	TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP0

	.if $defined(EXT_SYNC_ENABLE)
;**********************************************************************************************;
;pseudo code:
;we just pushed the extra edge so now is the time to take the time capture.
;from the experiments 12 cycle latency for read so adjust for that.
;also one more thing to consider is that EXTRA_EDGE is not always 0xFF.
;It can take values, FF, FC,F8, F0, E0, C0, 80 etc. so we need to adjust for
;that also to get the exact timing of extra edge fall.
;
;iep_time = read_iep();
;iep_time = iep_time + 12 ; //latency adjustment_done
;if(EXTRA_EDGE == 0xFF)
;{
;	;do nothing
;}
;if(EXTRA_EDGE == 0xFE)
;{
;	;each bit in extra edge is 3 pru cycle, simnce here
;	;extra edge came one bit early, so on from other possibilities.
;	iep_time = iep_time - 3;
;}
;if(EXTRA_EDGE == 0xFC)
;{
;	iep_time = iep_time - 6;
;}
;if(EXTRA_EDGE == 0xF8)
;{
;	iep_time = iep_time - 9;
;}
;if(EXTRA_EDGE == 0xF0)
;{
;	iep_time = iep_time - 12;
;}
;if(EXTRA_EDGE == 0xE0)
;{
;	iep_time = iep_time - 15;
;}
;if(EXTRA_EDGE == 0xC0)
;{
;	iep_time = iep_time - 18;
;}
;if(EXTRA_EDGE == 0x80)
;{
;	iep_time = iep_time - 21;
;}
;extra_edge_fall_time = iep_time;


	qbne        num_pulses_is_not_one1, NUM_PULSES, 1 ;not the last frame of period

	lbco        &REG_TMP0, c1, 0x10, 4
	add         REG_TMP0, REG_TMP0, 12  ;read offset

	qbeq        adjustment_done, EXTRA_EDGE_SELF, 0xFF
is_val_FE:
	qbne        is_val_FC, EXTRA_EDGE_SELF, 0xFE
	sub         REG_TMP0, REG_TMP0, 3
	qba         adjustment_done
is_val_FC:
	qbne        is_val_F8, EXTRA_EDGE_SELF, 0xFC
	sub         REG_TMP0, REG_TMP0, 6
	qba         adjustment_done
is_val_F8:
	qbne        is_val_F0, EXTRA_EDGE_SELF, 0xF8
	sub         REG_TMP0, REG_TMP0, 9
	qba         adjustment_done
is_val_F0:
	qbne        is_val_E0, EXTRA_EDGE_SELF, 0xF0
	sub         REG_TMP0, REG_TMP0, 12
	qba         adjustment_done
is_val_E0:
	qbne        is_val_C0, EXTRA_EDGE_SELF, 0xE0
	sub         REG_TMP0, REG_TMP0, 15
	qba         adjustment_done
is_val_C0:
	qbne        is_val_80, EXTRA_EDGE_SELF, 0xC0
	sub         REG_TMP0, REG_TMP0, 18
	qba         adjustment_done
is_val_80:
	qbne        adjustment_done,EXTRA_EDGE_SELF, 0x80
	sub         REG_TMP0, REG_TMP0, 21
	qba         adjustment_done
adjustment_done:
	sbco		&REG_TMP0, MASTER_REGS_CONST, EXTRA_EDGE_TIMESTAMP, 4
num_pulses_is_not_one1:
	qba             send_header_extra_drive_cycle_check_end
;**********************************************************************************************;
	.endif

send_header_extra_drive_cycle_check_end:
	qba			send_header_encode
send_header_no_extra:
;push last bit of SAMPLE, 3 bits of CYCLE RESET and 4 bits EQUALIZATION
    PUSH_FIFO		REG_TMP11.b0
send_header_encode:
;encode data
	ldi			REG_TMP11, (PDMEM00+LUT_5b6b_ENC)
	lbbo			&REG_FNC.b3, REG_TMP11, REG_FNC.b0, 1
	LOOKUP_BITCNT		REG_TMP0, REG_FNC.b3
	ldi			REG_TMP11, (PDMEM00+LUT_3b4b_ENC)
	lbbo			&REG_FNC.b2, REG_TMP11, REG_FNC.b1, 1
	LOOKUP_BITCNT		REG_TMP1, REG_FNC.b2
;check if subblock polarity is 0
	qbeq			send_header_encode_first_subblock_end, REG_TMP0.b0, 3
;calculate outcoming disparity (due to LUT structure, we lookup always neg. encodings->#0>=#1)
;calculate how many 0s are there more than 1s
	lsl			REG_TMP0.b0, REG_TMP0.b0, 1
	rsb			REG_TMP0.b0, REG_TMP0.b0, 6
;check line disparity for first block
	qbbc			send_header_encode_first_subblock_pos, DISPARITY, 7
	xor			REG_FNC.b3, REG_FNC.b3, 0x3f
	add			DISPARITY, DISPARITY, REG_TMP0.b0
	qba			send_header_encode_first_subblock_end
send_header_encode_first_subblock_pos:
	sub			DISPARITY, DISPARITY, REG_TMP0.b0
send_header_encode_first_subblock_end:
;check line disparity for second block
	qbbc			send_header_encode_sec_subblock_pos, DISPARITY, 7
;do not flip if subblock polarity is 0
	qbeq			send_header_encode_sec_subblock_pos, REG_TMP1.b0, 2
	xor			REG_FNC.b2, REG_FNC.b2, 0x0f
	qba			send_header_encode_sec_subblock_end
send_header_encode_sec_subblock_pos:
send_header_encode_sec_subblock_end:
;put together
	lsl			REG_FNC.b3, REG_FNC.b3, 2
	lsr			REG_TMP0.b0, REG_FNC.b2, 2
	or			REG_FNC.b3, REG_FNC.b3, REG_TMP0.b0
	lsl			REG_FNC.b2, REG_FNC.b2, 6
; transport_layer_send_msg sends short/long message (if pending) and also checks for QMLW/POS errors
	jmp			transport_layer_send_msg

transport_layer_send_msg_done:
;encoding end
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO		REG_FNC.b3
	;check if we receive or send 01 pattern
	qbeq			send_header_send_01_pattern, REG_FNC.b0, M_PAR_RESET
	qbeq			send_header_send_01_pattern, REG_FNC.b0, M_PAR_SYNC
	qba			send_header_dont_send_01
send_header_send_01_pattern:
;send 01 pattern
	CALL1			send_01
	qba			send_header_end
send_header_dont_send_01:
;send last 2 parameter bits
	CALL2 WAIT_TX_FIFO_FREE
;overclock
	qbbs			send_header_dont_send_01_send_1, REG_FNC.b2, 7
	PUSH_FIFO_CONST		0x00
	qba			send_header_dont_send_01_send_next
send_header_dont_send_01_send_1:
	PUSH_FIFO_CONST		0xff
send_header_dont_send_01_send_next:
	RESET_CYCLCNT
	jmp comp_logic_starts

comp_logic_done_1:
	qbeq			send_header_end_1, REG_FNC.b0, M_PAR_RESET
	qbeq			send_header_end_1, REG_FNC.b0, M_PAR_SYNC

	qbbc			transport_layer_recv_msg_done, H_FRAME.flags, FLAG_NORMAL_FLOW
;HINT: we have processing time here (~168 cycles)
	jmp			transport_layer_recv_msg
transport_layer_recv_msg_done:

	READ_CYCLCNT		REG_TMP1
	ldi			REG_TMP0, (9*(CLKDIV_NORMAL+1)-9)
	sub			REG_TMP0, REG_TMP0, REG_TMP1
	WAIT			REG_TMP0
	TX_CLK_DIV		CLKDIV_FAST, r0
	qbbs			send_header_dont_send_01_send_11, REG_FNC.b2, 6
	PUSH_FIFO_CONST		0x00
	ldi			LAST_BIT_SENT, 0
	.if $defined(EXT_SYNC_ENABLE)
	ldi REG_TMP2.b0, 0x1F
	;ldi     	REG_SCRATCH, P0EDRXCFG
	sbco &REG_TMP2.b0, ICSS_CFGx, EDRXCFG, 1
	.endif
	qba			send_header_dont_send_01_send_next1
send_header_dont_send_01_send_11:
	PUSH_FIFO_CONST		0xff
	ldi			LAST_BIT_SENT, 1
	.if $defined(EXT_SYNC_ENABLE)
	ldi REG_TMP2.b0, 0x17
	;ldi     	REG_SCRATCH, P0EDRXCFG
	sbco &REG_TMP2.b0, ICSS_CFGx, EDRXCFG, 1
	.endif
send_header_dont_send_01_send_next1:

send_header_sync_wait:
send_header_end_1:

send_header_end:
	RET
	.endif
;--------------------------------------------------------------------------------------------------
;Function: send_header_300m (RET_ADDR)
;This function sends the header (data before slave answer) for 300m PRU clock
;input:
;	REG_FNC.w0: parameter channel input
;output:
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
 	.if $defined("HDSL_MULTICHANNEL")
send_header_300m:
;push SYNC and first 2 bits of SAMPLE
	ldi FIFO_L,0x2f
	;PUSH 8 bytes for 1 byte data (0x2f) in FIFO
	loop aaa,4
	CALL3 PUSH_FIFO_2B_8x
aaa:
;check if we have an EXTRA period
;if we have a EXTRA period: do TX FIFO synchronization here to gain processing time
	qbeq			send_header_no_extra_wait, EXTRA_SIZE, 0
	RESET_CYCLCNT

send_header_modified:
send_header_no_extra_wait:

;calculate EQUALIZATION
	add			DISPARITY, DISPARITY, EXTRA_SIZE
	qbbs			send_header_disp_neg, DISPARITY, 7
send_header_disp_pos:
	qblt			send_header_disp_pos0, DISPARITY, 1
	ldi			REG_TMP11.b0, 0x60 ; 0x6a
	qba			send_header_end_disp
send_header_disp_pos0:
	qblt			send_header_disp_pos1, DISPARITY, 3
	ldi			REG_TMP11.b0, 0x62
	sub			DISPARITY, DISPARITY, 2
	qba			send_header_end_disp
send_header_disp_pos1:
	ldi			REG_TMP11.b0, 0x60
	sub			DISPARITY, DISPARITY, 4
	qba			send_header_end_disp
send_header_disp_neg:
	not			REG_TMP11.b0, DISPARITY
	add			REG_TMP11.b0, REG_TMP11.b0, 1
	qblt			send_header_disp_neg0, REG_TMP11.b0, 1
	ldi			REG_TMP11.b0, 0x60; 0x6a
	qba			send_header_end_disp
send_header_disp_neg0:
	qblt			send_header_disp_neg1, REG_TMP11.b0, 3
	ldi			REG_TMP11.b0, 0x6d
	add			DISPARITY, DISPARITY, 2
	qba			send_header_end_disp
send_header_disp_neg1:
	ldi			REG_TMP11.b0, 0x6f
	add			DISPARITY, DISPARITY, 4
send_header_end_disp:
;reset eCAP1 INT
	ldi			REG_TMP1.w0, (ECAP+ECAP_ECCLR)
	ldi			REG_TMP1.w2, 0xffff
	sbco			&REG_TMP1.w2, PWMSS1_CONST, REG_TMP1.w0, 2
	;sbco			&REG_TMP1.w2, PWMSS2_CONST, REG_TMP1.w0, 2
;HINT: we have some processing time here (140 cycles)
;go to V-Frame callback on transport layer
	qbbc			datalink_transport_no_v_frame_2, H_FRAME.flags, FLAG_NORMAL_FLOW
; Check if it V-Frame is complete (and currently the header for first H-Frame is
; being sent) -> state rx0 since we wait for processing v-frame in beginning of
; next frame and not at end of frame in rx7. If yes, start the V-frame
; processing. It will be completed in next H-Frame (State rx1). Therefore the
; V-frame processing is split into two parts : transport_on_v_frame and
; transport_on_v_frame_2.
	qbne			datalink_transport_no_v_frame, LOOP_CNT.b2, 8
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0x00
	PUSH_FIFO_CONST  0xff
	jmp			transport_on_v_frame
datalink_transport_on_v_frame_done:

datalink_transport_no_v_frame:
; Check if the first H-Frame is complete (and currently the header for second H-Frame
; is being sent). If yes, perform the remaining part of V-frame processing
	qbne			datalink_transport_no_v_frame_2, LOOP_CNT.b2, 7
 	qbbc			dont_push_for_non7_hframe_1, H_FRAME.flags, FLAG_NORMAL_FLOW
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST  0xff
	PUSH_FIFO_CONST  0xff
dont_push_for_non7_hframe_1:
    jmp             transport_on_v_frame_2

datalink_transport_on_v_frame_done_2:
datalink_transport_no_v_frame_2:

;check if we have an EXTRA period
	qbeq			send_header_no_extra, EXTRA_SIZE, 0

;********************************************************************************;
	;extra value decide starts
	qbeq        num_pulses_is_one3, NUM_PULSES, 1
	mov EXTRA_SIZE_SELF, EXTRA_SIZE
	ldi EXTRA_EDGE_SELF, 0
	qba         extra_value_decided
num_pulses_is_one3:
	mov EXTRA_SIZE_SELF, EXTRA_SIZE_COMP
	mov EXTRA_EDGE_SELF, EXTRA_EDGE_COMP
extra_value_decided:
	;extra value decide ends
	;remainder increament logic starts
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYNC_EXTRA_REMAINDER, 1 ;reload ES
	qbgt        remainder_increament_done1, REG_TMP0.b0, NUM_PULSES
	add         EXTRA_SIZE_SELF, EXTRA_SIZE_SELF, 1
remainder_increament_done1:
	;remainder increament logic ends
	qbne        extra_edge_calculation_for_self_done1, EXTRA_EDGE_SELF, 0
	sub         EXTRA_SIZE_SELF, EXTRA_SIZE_SELF, 1
	ldi         EXTRA_EDGE_SELF, 0xFF
extra_edge_calculation_for_self_done1:
	qba         send_header_extra_not_too_large

send_header_extra_not_too_large:
;limit STUFFING
	qbge			send_header_no_cap_stuffing, NUM_STUFFING, MAX_STUFFING
	ldi			NUM_STUFFING, MAX_STUFFING
send_header_no_cap_stuffing:
	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP0, c25, 0, 4
	add         REG_TMP0,REG_TMP0,4
	READ_CYCLCNT		REG_TMP1
	;lbco        &REG_TMP1, c1, 0x10, 4
	sbco        &REG_TMP1, c25, REG_TMP0, 4
	sbco        &REG_TMP0, c25, 0, 4
	.endif

;read cyclecount
	READ_CYCLCNT		REG_TMP1
	.if $defined(EXT_SYNC_ENABLE)
	qbeq           modified_header_wait, MODIFIED_HEADER_STARTED, 1
	ldi			   REG_TMP0, (9*(CLKDIV_NORMAL+1)-9-4-10)
	qba            modified_header_wait_done
modified_header_wait:
	ldi			REG_TMP0, (11*(CLKDIV_NORMAL+1))
modified_header_wait_done:
	.else
	ldi			REG_TMP0, (9*(CLKDIV_NORMAL+1)-9-4-4)
	.endif
	sub			REG_TMP0, REG_TMP0, REG_TMP1

;reset ECAP INT
send_header_extra_no_wait:
;	TX_CLK_DIV		CLKDIV_FAST, REG_TMP0
	sub			REG_TMP1.b0, EXTRA_SIZE_SELF, 1

send_header_extra_loop:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	sub			REG_TMP1.b0, REG_TMP1.b0, 1
	qbne			send_header_extra_loop, REG_TMP1.b0, 0

	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO		EXTRA_EDGE_SELF
send_header_extra_no_edge:
;reset clock to normal frequency

	.if $defined(EXT_SYNC_ENABLE)
	mov FIFO_L,REG_TMP11.b0
	CALL3 PUSH_FIFO_2B_8x

	.else
	mov FIFO_L,REG_TMP11.b0
	loop dd2,4
	CALL3 PUSH_FIFO_2B_8x
dd2:
    .endif

send_header_no_wait_after_synch:

	.if $defined(EXT_SYNC_ENABLE)
;**********************************************************************************************;
;pseudo code:
;we just pushed the extra edge so now is the time to take the time capture.
;from the experiments 12 cycle latency for read so adjust for that.
;also one more thing to consider is that EXTRA_EDGE is not always 0xFF.
;It can take values, FF, FC,F8, F0, E0, C0, 80 etc. so we need to adjust for
;that also to get the exact timing of extra edge fall.
;
;iep_time = read_iep();
;iep_time = iep_time + 12 ; //latency adjustment_done
;if(EXTRA_EDGE == 0xFF)
;{
;	;do nothing
;}
;if(EXTRA_EDGE == 0xFE)
;{
;	;each bit in extra edge is 3 pru cycle, simnce here
;	;extra edge came one bit early, so on from other possibilities.
;	iep_time = iep_time - 3;
;}
;if(EXTRA_EDGE == 0xFC)
;{
;	iep_time = iep_time - 6;
;}
;if(EXTRA_EDGE == 0xF8)
;{
;	iep_time = iep_time - 9;
;}
;if(EXTRA_EDGE == 0xF0)
;{
;	iep_time = iep_time - 12;
;}
;if(EXTRA_EDGE == 0xE0)
;{
;	iep_time = iep_time - 15;
;}
;if(EXTRA_EDGE == 0xC0)
;{
;	iep_time = iep_time - 18;
;}
;if(EXTRA_EDGE == 0x80)
;{
;	iep_time = iep_time - 21;
;}
;extra_edge_fall_time = iep_time;


	qbne        num_pulses_is_not_one1, NUM_PULSES, 1 ;not the last frame of period

	lbco        &REG_TMP0, c1, 0x10, 4
	add         REG_TMP0, REG_TMP0, 12   ;read offset

	ldi			REG_TMP11, (PDMEM00+LUT_EE)
	lbbo			&REG_TMP11.b0, REG_TMP11, EXTRA_EDGE_SELF, 1
	sub			REG_TMP0, REG_TMP0, REG_TMP11.b0
adjustment_done:
	sbco		&REG_TMP0, MASTER_REGS_CONST, EXTRA_EDGE_TIMESTAMP, 4
num_pulses_is_not_one1:
	qba             send_header_extra_drive_cycle_check_end
;**********************************************************************************************;
	.endif

send_header_extra_drive_cycle_check_end:
	qba			send_header_encode
send_header_no_extra:
;push last bit of SAMPLE, 3 bits of CYCLE RESET and 4 bits EQUALIZATION
	.if !$defined(EXT_SYNC_ENABLE)
	mov FIFO_L,REG_TMP11.b0
	CALL3 PUSH_FIFO_2B_8x
    .endif
send_header_encode:

;encode data
	ldi			REG_TMP11, (PDMEM00+LUT_5b6b_ENC)
	lbbo			&REG_FNC.b3, REG_TMP11, REG_FNC.b0, 1
	LOOKUP_BITCNT		REG_TMP0, REG_FNC.b3
	ldi			REG_TMP11, (PDMEM00+LUT_3b4b_ENC)
	lbbo			&REG_FNC.b2, REG_TMP11, REG_FNC.b1, 1
	LOOKUP_BITCNT		REG_TMP1, REG_FNC.b2
;check if subblock polarity is 0
	qbeq			send_header_encode_first_subblock_end, REG_TMP0.b0, 3
;calculate outcoming disparity (due to LUT structure, we lookup always neg. encodings->#0>=#1)
;calculate how many 0s are there more than 1s
	lsl			REG_TMP0.b0, REG_TMP0.b0, 1
	rsb			REG_TMP0.b0, REG_TMP0.b0, 6
;check line disparity for first block
	qbbc			send_header_encode_first_subblock_pos, DISPARITY, 7
	xor			REG_FNC.b3, REG_FNC.b3, 0x3f
	add			DISPARITY, DISPARITY, REG_TMP0.b0
	qba			send_header_encode_first_subblock_end
send_header_encode_first_subblock_pos:
	sub			DISPARITY, DISPARITY, REG_TMP0.b0
send_header_encode_first_subblock_end:
;check line disparity for second block
	qbbc			send_header_encode_sec_subblock_pos, DISPARITY, 7
;do not flip if subblock polarity is 0
	qbeq			send_header_encode_sec_subblock_pos, REG_TMP1.b0, 2
	xor			REG_FNC.b2, REG_FNC.b2, 0x0f
	qba			send_header_encode_sec_subblock_end
send_header_encode_sec_subblock_pos:
send_header_encode_sec_subblock_end:
;put together
	lsl			REG_FNC.b3, REG_FNC.b3, 2
	lsr			REG_TMP0.b0, REG_FNC.b2, 2
	or			REG_FNC.b3, REG_FNC.b3, REG_TMP0.b0
	lsl			REG_FNC.b2, REG_FNC.b2, 6
	CALL3 PUSH_FIFO_3_8x
	jmp			transport_layer_send_msg

transport_layer_send_msg_done:
;encoding end

	;check if we receive or send 01 pattern
	qbeq			send_header_send_01_pattern, REG_FNC.b0, M_PAR_RESET
	qbeq			send_header_send_01_pattern, REG_FNC.b0, M_PAR_SYNC
	qba			send_header_dont_send_01
send_header_send_01_pattern:

	mov FIFO_L,REG_FNC.b3
	loop dd3,4
	CALL3 PUSH_FIFO_2B_8x
dd3:
;send 01 pattern
	CALL1			send_01
	qba			send_header_end
send_header_dont_send_01:

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP0, c25, 0, 4
	add         REG_TMP0,REG_TMP0,4
	qble        log_done1, REG_TMP0,255
	sbco        &REG_FNC, c25, REG_TMP0, 4
	sbco        &REG_TMP0, c25, 0, 4
log_done1:
	.endif
	qbeq			send_header_end_1, REG_FNC.b0, M_PAR_RESET
	qbeq			send_header_end_1, REG_FNC.b0, M_PAR_SYNC
;HINT: we have processing time here (~168 cycles)
	;check if we reset protocol
	lbco		&FIFO_L, MASTER_REGS_CONST, SYS_CTRL, 1
	qbbc		SYS_CTRL_PRST_cleared,FIFO_L, SYS_CTRL_PRST
	jmp		No_long_short_msg
SYS_CTRL_PRST_cleared:
;check if we reset protocol by reading SAFE_CTRL register
	lbco		&FIFO_L, MASTER_REGS_CONST, SAFE_CTRL, 1
	qbbc		SAFE_CTRL_PRST_cleared, FIFO_L, SAFE_CTRL_PRST
	jmp		No_long_short_msg

SAFE_CTRL_PRST_cleared:
	mov FIFO_L,CHANNEL.ch_parah
	lsr		FIFO_L, FIFO_L, 3
	and		FIFO_L, FIFO_L, 0x1f
	qbeq		No_long_short_msg, FIFO_L, S_PAR_IDLE
	qbbs		No_long_short_msg, H_FRAME.flags, FLAG_WAIT_IDLE
	qbbs		No_long_short_msg, FIFO_L, 4
	mov FIFO_L,REG_FNC.b3
	ldi			LEARN_STATE_STARTED , 3
	jmp comp_logic_starts
comp_logic_done1:
	jmp Push_done

No_long_short_msg:
	mov FIFO_L,REG_FNC.b3
	ldi			LEARN_STATE_STARTED , 2

	jmp comp_logic_starts
Push_done:
	jmp			transport_layer_recv_msg

comp_logic_done:
transport_layer_recv_msg_done:
;send last 2 parameter bits
    CALL2 WAIT_TX_FIFO_FREE
;overclock
	qbbs			send_header_dont_send_01_send_1, REG_FNC.b2, 7
	PUSH_FIFO_CONST		0x00
	qba			send_header_dont_send_01_send_next
send_header_dont_send_01_send_1:
	PUSH_FIFO_CONST		0xff
send_header_dont_send_01_send_next:
	RESET_CYCLCNT
	qbbs			send_header_dont_send_01_send_11, REG_FNC.b2, 6
	PUSH_FIFO_CONST		0x00
	ldi			LAST_BIT_SENT, 0
	.if $defined(EXT_SYNC_ENABLE)
	ldi REG_TMP2.b0, 0x1F
	;ldi     	REG_SCRATCH, P0EDRXCFG
	sbco &REG_TMP2.b0, ICSS_CFGx, EDRXCFG, 1
	.endif
	qba			send_header_dont_send_01_send_next1
send_header_dont_send_01_send_11:
	PUSH_FIFO_CONST		0xff
	ldi			LAST_BIT_SENT, 1
	.if $defined(EXT_SYNC_ENABLE)
	ldi REG_TMP2.b0, 0x17
	;ldi     	REG_SCRATCH, P0EDRXCFG
	sbco &REG_TMP2.b0, ICSS_CFGx, EDRXCFG, 1
	.endif
send_header_dont_send_01_send_next1:
send_header_sync_wait:
send_header_end_1:
send_header_end:
	RET
	.endif ;MULTICHANNEL



;--------------------------------------------------------------------------------------------------
;Function: send_stuffing (RET_ADDR)
;This function sends the trailer (data after slave answer: STUFFING)
;input:
;output:
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
send_stuffing:
	.if $defined(EXT_SYNC_ENABLE)
	mov			REG_TMP11, RET_ADDR1
	;do calc

	;stuffing value decide starts
	qbeq        num_pulses_is_one, NUM_PULSES, 1
	mov			REG_FNC.b3, NUM_STUFFING
	qba         stuffing_value_decided
num_pulses_is_one:
	mov			REG_FNC.b3, NUM_STUFFING_COMP
stuffing_value_decided:
	;stuffing value decide ends

	;remainder increament logic starts
	lbco		&REG_TMP0.b0, MASTER_REGS_CONST, SYNC_STUFFING_REMAINDER, 1 ;reload ES
	qbgt        remainder_increament_done, REG_TMP0.b0, NUM_PULSES
	add			REG_FNC.b3, REG_FNC.b3, 1
remainder_increament_done:
	;remainder increament logic ends

	;ES reload starts
	sub			NUM_PULSES, NUM_PULSES, 1
	qbne        num_pulses_non_zero, NUM_PULSES, 0
	lbco		&NUM_PULSES, MASTER_REGS_CONST, SYNC_CTRL, 1 ;reload ES
num_pulses_non_zero:
	;ES reload ends

	qbeq		send_stuffing_no_stuffing, REG_FNC.b3, 0
;check if we have stuffing
	READ_CYCLCNT		REG_TMP0
	qbeq        learn_state_started, LEARN_STATE_STARTED, 1
	rsb			REG_TMP2, REG_TMP0, (5*(CLKDIV_NORMAL+1)+4);(6*(CLKDIV_NORMAL+1)+4)
	qba calculation_for_wait_done
learn_state_started:
	;lbco        &REG_TMP1, c1, 0x10, 4
	rsb			REG_TMP2, REG_TMP0, (4*(CLKDIV_NORMAL+1)+4);(6*(CLKDIV_NORMAL+1)+4)
calculation_for_wait_done:
	nop
	.else
	mov			REG_TMP11, RET_ADDR1
	qbeq			send_stuffing_no_stuffing, NUM_STUFFING, 0
;check if we have stuffing
	READ_CYCLCNT		REG_TMP0
	rsb			REG_TMP2, REG_TMP0, (5*(CLKDIV_NORMAL+1)+4);(6*(CLKDIV_NORMAL+1)+4)
	mov			REG_FNC.b3, NUM_STUFFING
	.endif;EXT_SYNC_ENABLE

	.if $defined("HDSL_MULTICHANNEL")
	ldi FIFO_L,0x2c
	loop aaa4,3
	CALL3 PUSH_FIFO_2B_8x
aaa4:
	.else
	PUSH_FIFO_CONST		0x0b
	WAIT			REG_TMP2
	.endif

;send first 4 zeroes with double frequency
;synchronize with clock
	.if !$defined("HDSL_MULTICHANNEL")
send_stuffing_sync_clk:
	;ldi     		REG_SCRATCH, P0EDTXCFG
	lbco			&REG_TMP1, ICSS_CFGx, EDTXCFG, 4
	.if $defined(CHANNEL_2)
	qbbc			send_stuffing_sync_clk, REG_TMP1, 10
	.endif
	.if $defined(CHANNEL_1)
	qbbc			send_stuffing_sync_clk, REG_TMP1, 9
	.endif
	.if $defined(CHANNEL_0)
	qbbc			send_stuffing_sync_clk, REG_TMP1, 8
	.endif

	.endif
send_stuffing_first:
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV	        CLKDIV_DOUBLE, REG_TMP1
;wait 4 cycles
	ldi			REG_FNC.w0, CLKDIV_NORMAL
	ldi			REG_FNC.b2, 4
	CALL1		switch_clk
    .endif ;HDSL_MULTICHANNEL

	sub			REG_FNC.b3, REG_FNC.b3, 1
	qbeq			send_stuffing_no_stuffing, REG_FNC.b3, 0

	.if $defined("HDSL_MULTICHANNEL")
send_stuffing_loop_8x:
	ldi FIFO_L,0x2c
	loop aaa5,3
	CALL3 PUSH_FIFO_2B_8x
aaa5:
	sub			REG_FNC.b3, REG_FNC.b3, 1
	qbne			send_stuffing_loop_8x, REG_FNC.b3, 1
	ldi FIFO_L,0x2c
	loop aaa6,3
	CALL3 PUSH_FIFO_2B_8x
aaa6:
	.else

send_stuffing_loop:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x0b
;wait 4 cycles
	ldi			REG_FNC.w0, CLKDIV_DOUBLE
	ldi			REG_FNC.b2, 4
	CALL1			switch_clk
;wait 4 cycles
	ldi			REG_FNC.w0, CLKDIV_NORMAL
	ldi			REG_FNC.b2, 3
	CALL1			switch_clk
	sub			REG_FNC.b3, REG_FNC.b3, 1
	qbne			send_stuffing_loop, REG_FNC.b3, 0

	.endif
send_stuffing_no_stuffing:
	mov			RET_ADDR1, REG_TMP11
	RET1
;--------------------------------------------------------------------------------------------------
;Function: send_trailer (RET_ADDR)
;This function sends the trailer (data after slave answer: TRAILER)
;gain: 5*24-6=114 cycles
;input:
;output:
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
send_trailer:
;already overclocked
;reduce trailer length by a few ns to compensate for a slightly transmission start delay < 30ns
;so: first reduce from higher clockrate to lower clock rate for 1 cycle, then set to normal clockrate again
;send TRAILER
	;PUSH 8 bytes for 1 byte data (0x03) in FIFO
    .if $defined("HDSL_MULTICHANNEL")
	NOP_n 9
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
	PUSH_FIFO_CONST		0x00
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff

    .else
    PUSH_FIFO_CONST		0x03
	TX_CHANNEL
    .endif ;HDSL_MULTICHANNEL
	;determine DELAY Master Register (also used as 2 dummy cycles)
	lsl			REG_TMP1.b0, RSSI, 4
	or			REG_TMP1.b0, REG_TMP1.b0, SLAVE_DELAY

;  additional delay here shortens the the first trailer byte
    .if $defined("HDSL_MULTICHANNEL")
	NOP_n 7
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
	TX_CLK_DIV		CLKDIV_SLOW, REG_TMP0
    .endif
;reset DISPARITY
	ldi			DISPARITY, 0
	;store RSSI/DELAY (also used as 2 dummy cycles)
	sbco			&REG_TMP1.b0, MASTER_REGS_CONST, DELAY, 1
;update qm when rssi < 2
	qble			send_trailer_dont_update_qm, RSSI, 2
;save return addr
	mov			REG_TMP11.w0, RET_ADDR1
	qm_sub			4
;restore return addr
	mov			RET_ADDR1, REG_TMP11.w0
send_trailer_dont_update_qm:
    .if !$defined("HDSL_MULTICHANNEL")
	TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP0
    .endif
;syn with clock before resetting counter
	.if !$defined("HDSL_MULTICHANNEL")
	WAIT_CLK_LOW		REG_TMP0
	.endif
	RESET_CYCLCNT
	RET1
;--------------------------------------------------------------------------------------------------
datalink_abort:
	qbbs			datalink_abort_no_wait, r30, RX_ENABLE ;changed here from 24 to 26
	WAIT_TX_DONE
datalink_abort_no_wait:
	lbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
	add			REG_TMP0.b0, REG_TMP0.b0, 1
	sbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
; Trigger a reset in all channels

	.if $defined(CHANNEL_2)
;reset PRST bit in SYS_CTRL
	ldi 	REG_TMP1, DMEM_CH0_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi 	REG_TMP1, DMEM_CH1_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	.if $defined(CHANNEL_1)
	ldi 	REG_TMP1, DMEM_CH0_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi 	REG_TMP1, DMEM_CH2_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	.if $defined(CHANNEL_0)
	ldi 	REG_TMP1, DMEM_CH1_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi 	REG_TMP1, DMEM_CH2_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	jmp			datalink_reset

;--------------------------------------------------------------------------------------------------
;Function: qm_add (RET_ADDR1)
;Adds value to Quality Monitor and resets connection if necessary
; 15+9=24 cycles
;input:
;	REG_FNC.b0: value
;modifies:
;	REG_TMP1, REG_FNC
;--------------------------------------------------------------------------------------------------
qm_add:
	.if 1
	and	QM, QM, 0x7f
;check if negative (bit 7 indicates there is a link -> check bit 6)
	qbbc	qm_add_no_reset, QM, 6
	ldi	QM, 0
;update MASTER_QM
	sbco	&QM, MASTER_REGS_CONST, MASTER_QM, 1
	qba	datalink_abort
qm_add_no_reset:
;max value is 15
	qbge	qm_add_no_capping, QM, QM_MAX
	ldi	QM, QM_MAX
qm_add_no_capping:
;check if we need to set event flag
	qble	qm_add_below_not_14, QM, 14
; Defer the events register update to later
; Set EVENT_UPDATE_PENDING_QMLW to indicate a low QM value
	lbco		&REG_TMP1.b0, MASTER_REGS_CONST, EVENT_UPDATE_PENDING, 1
	set         REG_TMP1.b0, REG_TMP1.b0, EVENT_UPDATE_PENDING_QMLW
	sbco		&REG_TMP1.b0, MASTER_REGS_CONST, EVENT_UPDATE_PENDING, 1
qm_add_below_not_14:
	or	QM, QM, (1<<7)
;update MASTER_QM
	sbco	&QM, MASTER_REGS_CONST, MASTER_QM, 1
qm_add_end:
	.endif
	RET1
;--------------------------------------------------------------------------------------------------
;Function: wait_delay (RET_ADDR1)
;functions waits to compensate slave delay if necessary
;input:
;modifies:
;--------------------------------------------------------------------------------------------------
wait_delay:
	WAIT_TX_DONE
    .if $defined("HDSL_MULTICHANNEL")
	NOP_n 12
    .endif  ;HDSL_MULTICHANNEL
	.if $defined(EXT_SYNC_ENABLE)
	NOP_n 1
	.endif ;EXT_SYNC_ENABLE
; same code as in learn
; with 4 or 3 bit encoder does not respond after time, starts working with 2 set it to 1
wait_on_rx_transtion_in_wait_delay:
        RX_EN
 	;measure passed time during receive and wait appropriately at the end -> reset cyclecount
	RESET_CYCLCNT

;reset RSSI
	ldi			RSSI, 12
; wait for slave delay in bits
	qbeq		wait_delay_no_wait, SLAVE_DELAY, 0
	mov			REG_TMP1.b0, SLAVE_DELAY
wait_delay_recv_wire:
	qbbc			wait_delay_recv_wire, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
	CLEAR_VAL
	sub				REG_TMP1.b0,REG_TMP1.b0,1
	qbne			wait_delay_recv_wire, REG_TMP1.b0, 0

wait_delay_no_wait:
	RET1

;--------------------------------------------------------------------------------------------------
;Function: calc_rssi (RET_ADDR1)
;Calculates RSSI based on current EDGES
;input: CUR_EDGES
;output: RSSI
;--------------------------------------------------------------------------------------------------
calc_rssi:
	ldi			REG_TMP0, (PDMEM00 + 0x5e4);LUT_RSSI)
	lbbo			&REG_TMP0, REG_TMP0, CUR_EDGES, 1
;store new rssi only if value is smaller
	qblt			calc_rssi_discard, REG_TMP0.b0, RSSI
	mov			RSSI, REG_TMP0.b0
calc_rssi_discard:
	RET1

;--------------------------------------------------------------------------------------------------
;Function: comp_logic_starts
;Calculates 1)overhead adjustment per frame
;			2)compensation for any diff between the sync pulse captured from latch and  extra
;			   edge fall time computed by PRU
;input: NUM_PULSES,EXTRA_EDGE,EXTRA_SIZE,NUM_STUFFING
;output: Modified EXTRA_SIZE_COMP and NUM_STUFFING_COMP
;
;--------------------------------------------------------------------------------------------------


;HINT: we have some processing time here (~74 cycles) ;6-19:sorry, not anymore, used for ext. sync suppport

;***********************************************************************************************************;
;pseudocode:
;This code takes care of two things
;1)overhead adjustment per frame
;2)compensation for any diff between the sync pulse captured from latch and  extra
;edge fall time computed by PRU

;/*push in fifo M_PAR data to get time for processing.
;store the default values of extra and stuffing calculated in the beginning, to this we will
;make adjustments as per need*/
;EXTRA_EDGE_COMP = EXTRA_EDGE
;EXTRA_SIZE_COMP = EXTRA_SIZE
;NUM_STUFFING_COMP = NUM_STUFFING

;/*note all the time here is in terms of PRU cycle(4.44ns) unless specified*/
;sync_pulse_rise_time = read_latch1_iep1_capture_register();
;extra_edge_fall_time = read(); //it was read and stored in memory earliar
;diff_for_compensation = (sync_pulse_rise_time - extra_edge_fall_time)/3; // we have also converted cycles to bit here, by dividing by 3, note these overclocked bits we are talking about
;overhead_adjustment = 8 - TIME_REST_COMP; //coming from last cycle, overhead of child is (8 - parents TIME_REST)
;total_adjustment = (diff_for_compensation + overhead_adjustment); // we take care of sign etc.
;EXTRA_EDGE_COMP = EXTRA_EDGE_COMP (-/+) total_adjustment; //extra size may vary here, if extra_edge is not able to compensate fully
;if(EXTRA_SIZE_COMP < 4)
;{
;	EXTRA_SIZE_COMP = EXTRA_SIZE_COMP + 1;
;	NUM_STUFFING_COMP = NUM_STUFFING_COMP - 1;
;}
;if(EXTRA_SIZE_COMP > 9)
;{
;	EXTRA_SIZE_COMP = EXTRA_SIZE_COMP - 1;
;	NUM_STUFFING_COMP = NUM_STUFFING_COMP + 1;
;}
;in next cycle these _COMP values will be pushed to fifo and new values will be calculated and so on.
;one point to note here is if you need to adjust num of stuffing you need to do it from one cycle before.
;since stuffng for current cycle is not pushed yet, we can do that.
;that's one strategic importance of keeping this code here.

comp_logic_starts:
	.if $defined(EXT_SYNC_ENABLE)
	;compensation logic for diff between sync signal and extra edge starts;
	qbne        num_pulses_is_not_one2, NUM_PULSES, 1 ;not the last frame of period
	.if $defined("HDSL_MULTICHANNEL")
	CALL3 PUSH_FIFO_2B_8x
	.endif

	mov         EXTRA_EDGE_COMP, EXTRA_EDGE
	mov         EXTRA_SIZE_COMP, EXTRA_SIZE
	mov         NUM_STUFFING_COMP, NUM_STUFFING

	lbco		&REG_TMP0, MASTER_REGS_CONST, EXTRA_EDGE_TIMESTAMP, 4
	lbco        &REG_TMP1, IEP1_BASE_CONST, 0x50, 4
	qbge        extra_edge_ahead, REG_TMP1 ,REG_TMP0
	mov         REG_TMP2, REG_TMP0
	sub         REG_TMP0, REG_TMP1, REG_TMP0


	ldi         REG_TMP2, MAX_ALLOWED_CYCLE_DIFF
	qble        cycle_diff_more_than_max_allowed, REG_TMP0 ,REG_TMP2
	qbge        no_capping1, REG_TMP0 ,CAPPING_CYCLE_DIFF
	ldi         REG_TMP0 , CAPPING_CYCLE_DIFF
	;qbge        compensation_not_needed_this_cycle, REG_TMP0 ,2

no_capping1:
	mov         REG_TMP1, REG_FNC ;taking backup
	mov			REG_FNC.w0, REG_TMP0
	ldi			REG_FNC.w2, 3;3
	CALL1		int_div
	mov         REG_TMP0.b0, REG_FNC.b2
	qbeq        no_reminder1, REG_FNC.w0, 0
	add         REG_TMP0.b0, REG_TMP0.b0,1
no_reminder1:
	mov         REG_FNC, REG_TMP1 ;reloading orig value
	qba         self_overhead_more1
extra_edge_ahead:
	mov         REG_TMP2, REG_TMP0
	sub         REG_TMP0, REG_TMP0, REG_TMP1

	ldi         REG_TMP2, MAX_ALLOWED_CYCLE_DIFF
	qble        cycle_diff_more_than_max_allowed, REG_TMP0 ,REG_TMP2
	qbge        no_capping2, REG_TMP0 ,CAPPING_CYCLE_DIFF
	ldi         REG_TMP0 , CAPPING_CYCLE_DIFF
no_capping2:
	mov         REG_TMP1, REG_FNC ;taking backup
	mov			REG_FNC.w0, REG_TMP0
	ldi			REG_FNC.w2, 3;3
	CALL1		int_div
	mov         REG_TMP0.b0, REG_FNC.b2
	qbeq        no_reminder2, REG_FNC.w0, 0
	add         REG_TMP0.b0, REG_TMP0.b0, 1
no_reminder2:
	mov         REG_FNC, REG_TMP1 ;reloading orig value
	qba         child_overhead_more1
cycle_diff_more_than_max_allowed:
	;halt ;enable to debug jitter out of bound issues
	ldi REG_TMP0.b0, 0 ;this is the case of iep wraparound between two readings. happens once in 3-4 hours, let's not waste cycles here
	                   ;by taking care of wraparound calculation, let's just ignore this reading.

child_overhead_more1:
	.if $defined("HDSL_MULTICHANNEL")
	.endif
	ldi         REG_TMP1.b0, 8
	sub         REG_TMP1.b0, REG_TMP1.b0, TIME_REST_COMP
	qbne        time_rest_comp_not_8_1, REG_TMP1.b0, 8
	ldi         REG_TMP1.b0, 0
time_rest_comp_not_8_1:
	mov         TIME_REST_COMP, TIME_REST

	add         TIME_REST_COMP, TIME_REST_COMP, 16
	sub         EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 2
	sub         TIME_REST_COMP, TIME_REST_COMP, REG_TMP0.b0
	sub         TIME_REST_COMP, TIME_REST_COMP, REG_TMP1.b0
	qba         check_time_rest_size_violation1
self_overhead_more1:
	.if $defined("HDSL_MULTICHANNEL")
	.endif
	ldi         REG_TMP1.b0, 8
	sub         REG_TMP1.b0, REG_TMP1.b0, TIME_REST_COMP
	qbne        time_rest_comp_not_8_2, REG_TMP1.b0, 8
	ldi         REG_TMP1.b0, 0
time_rest_comp_not_8_2:
	mov         TIME_REST_COMP, TIME_REST

	add         TIME_REST_COMP, TIME_REST_COMP, 8
	sub         EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 1
	add         TIME_REST_COMP, TIME_REST_COMP, REG_TMP0.b0

	sub         TIME_REST_COMP, TIME_REST_COMP, REG_TMP1.b0

check_time_rest_size_violation1:
	qbge        comp_done1, TIME_REST_COMP, 7
	add         EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 1
	sub         TIME_REST_COMP, TIME_REST_COMP, 8
	qbge        comp_done1, TIME_REST_COMP, 7
	add         EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 1
	sub         TIME_REST_COMP, TIME_REST_COMP, 8
comp_done1:
	mov			REG_TMP0.b1, TIME_REST_COMP
	ldi			EXTRA_EDGE_COMP, 0
	.if $defined("HDSL_MULTICHANNEL")
	CALL3 PUSH_FIFO_2B_8x
	.endif

	qbeq		extra_edge_bit_setting_loop_end1, REG_TMP0.b1, 0
	ldi			REG_TMP0.b2, 7
extra_edge_bit_setting1:
	set			EXTRA_EDGE_COMP, EXTRA_EDGE_COMP, REG_TMP0.b2
	sub			REG_TMP0.b1, REG_TMP0.b1, 1
	sub			REG_TMP0.b2, REG_TMP0.b2, 1
	qblt		extra_edge_bit_setting1, REG_TMP0.b1, 0
extra_edge_bit_setting_loop_end1:
	mov         REG_TMP0.b0, EXTRA_SIZE_COMP
	qbeq      	check_if_extra_remainder, EXTRA_EDGE_COMP, 0
	add         REG_TMP0.b0, REG_TMP0.b0, 1
check_if_extra_remainder:
	lbco		&REG_TMP0.b1, MASTER_REGS_CONST, SYNC_EXTRA_REMAINDER, 1 ;reload ES
	qbeq      	send_header_extra_too_small1, REG_TMP0.b1, 0
	add         REG_TMP0.b0, REG_TMP0.b0, 1 ;there is an extra remainder
send_header_extra_too_small1:
	qble		send_header_extra_not_too_small1, REG_TMP0.b0, 4 ;too small extra window
	add			EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 6
	sub			NUM_STUFFING_COMP, NUM_STUFFING_COMP, 1
send_header_extra_not_too_small1:
	qbge		extra_size_validation_done1, REG_TMP0.b0, 9  ;too large extra window
	sub			EXTRA_SIZE_COMP, EXTRA_SIZE_COMP, 6
	add			NUM_STUFFING_COMP, NUM_STUFFING_COMP, 1
	jmp extra_size_validation_done1

num_pulses_is_not_one2:
	loop extra_size_validation_done1,2
	.if $defined("HDSL_MULTICHANNEL")
	CALL3 PUSH_FIFO_2B_8x
	.endif
extra_size_validation_done1:
	.if !$defined("HDSL_MULTICHANNEL")
	jmp comp_logic_done_1
	.endif
	;compensation logic for diff between sync signal and extra edge ends;
	.else ;free run mode starts
	.if $defined("HDSL_MULTICHANNEL")
	loop push_2B_c,2
	CALL3 PUSH_FIFO_2B_8x
push_2B_c:
	.else
	jmp comp_logic_done_1
	.endif
	.endif
	.if $defined("HDSL_MULTICHANNEL")
comp_logic_ends:

	qbeq	comp2,LEARN_STATE_STARTED,2
	qbeq	comp1,LEARN_STATE_STARTED,3
	ldi			LEARN_STATE_STARTED , 1
	jmp comp_logic_done

comp1:
	ldi			LEARN_STATE_STARTED , 1
	jmp transport_layer_recv_msg
comp2:
	CALL2 WAIT_TX_FIFO_FREE
	CALL3 PUSH_FIFO_3_8x
	ldi			LEARN_STATE_STARTED , 1
	jmp transport_layer_recv_msg
	.endif
;***********************************************************************************************************;


PUSH_FIFO_2B_8x:
    ;mov FIFO_L,dat
    qbbs L2_01, FIFO_L, 7 ; check bit 0
    CALL2 WAIT_TX_FIFO_FREE
    ldi   r30.b0, 0x0 ; Push oversampled 0s on the wire
    qba    L3_01
L2_01:
    CALL2 WAIT_TX_FIFO_FREE
    ldi   r30.b0, 0xff ; Push oversampled 1s on the wire
L3_01:
    lsl   FIFO_L,FIFO_L, 1 ; Drop least significant bit

    qbbs L2_11, FIFO_L, 7 ; check bit 0
    ldi   r30.b0, 0x0 ; Push oversampled 0s on the wire
    qba    L3_11
L2_11:
    ldi   r30.b0, 0xff ; Push oversampled 1s on the wire
L3_11:
    lsl   FIFO_L,FIFO_L, 1 ; Drop least significant bit

    RET3

WAIT_TX_FIFO_FREE:
l2:
	.if $defined(CHANNEL_2)
	qbbs		l2, r31, TX_GLOBAL_TX_GO
	qbbs		l2, r31, TX_GLOBAL_REINIT
	.endif
	.if $defined(CHANNEL_1)
	qbbs		l2, r31, 12
	qbbs		l2, r31, 11
	.endif
	.if $defined(CHANNEL_0)
	qbbs		l2, r31, 4 ;fill level 4
	qbbs		l2, r31, 3 ;fill level 2
	.endif
	RET2

PUSH_FIFO_3_8x:
    ;mov FIFO_L,dat
    qbbs L2_05, FIFO_L, 7 ; check bit 0

    ldi   r30.b0, 0x0 ; Push oversampled 0s on the wire
    qba    L3_05
L2_05:

    ldi   r30.b0, 0xff ; Push oversampled 1s on the wire
L3_05:
    lsl   FIFO_L,FIFO_L, 1 ; Drop least significant bit

	CALL2 WAIT_TX_FIFO_FREE

	loop L4_15,3
    qbbs L2_15, FIFO_L, 7 ; check bit 0

    ldi   r30.b0, 0x0 ; Push oversampled 0s on the wire
    qba    L3_15
L2_15:
    ldi   r30.b0, 0xff ; Push oversampled 1s on the wire

L3_15:
    lsl   FIFO_L,FIFO_L, 1 ; Drop least significant bit

L4_15:
    RET3

;--------------------------------------------------------------------------------------------------
;Function: int_div (RET_ADDR1)
;integer divides
;input:
;	REG_FNC.w0: Number
;	REG_FNC.w2: Divisor
;output:
;	REG_FNC.w2: Result
;	REG_FNC.w0: Rest
;modifies:
;--------------------------------------------------------------------------------------------------
int_div:
	ldi			REG_TMP0, 0
int_div_loop:
	qbgt			int_div_end, REG_FNC.w0, REG_FNC.w2
	sub			REG_FNC.w0, REG_FNC.w0, REG_FNC.w2
	add			REG_TMP0, REG_TMP0, 1
	qba			int_div_loop
int_div_end:
	mov			REG_FNC.w2, REG_TMP0
	RET1
