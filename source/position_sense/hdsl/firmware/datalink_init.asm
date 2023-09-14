
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
	.ref transport_init
	.ref qm_add
	.ref calc_rssi
	.ref send_stuffing
	.ref datalink_wait_vsynch
	.if $defined("HDSL_MULTICHANNEL")
	.ref send_header_300m
	.else
	.ref send_header
	.endif
	.ref send_header_modified
	.ref send_trailer
	.ref wait_delay
	.ref datalink_loadfw
	.ref recv_dec
	.ref transport_on_h_frame
	.ref sync_pulse
	.ref check_test_pattern
	.ref datalink_abort_jmp
	.ref receive
	.ref datalink_abort
	.global datalink_reset
	.global datalink_init_start


	.sect	".text"

relocatable0:

datalink_init_start:
datalink_reset:
;State RESET
	zero			&r0, 124
;send 2 times

;setup ICSS encoder peripheral for Hiperface DSL
	ldi			DISPARITY, 0x00
	TX_EN
	SET_TX_CH0
	REINIT_TX
	TX_FRAME_SIZE		0, REG_TMP0
    .if $defined("HDSL_MULTICHANNEL")
	TX_CLK_DIV 		CLKDIV_FAST, REG_TMP0
    .else
    TX_CLK_DIV 		CLKDIV_NORMAL, REG_TMP0
    .endif

; set the VERSION and VERSION2 register
    ldi     REG_TMP0.b0, ICSS_FIRMWARE_RELEASE
    sbco	&REG_TMP0.b0, MASTER_REGS_CONST, VERSION, 1
    sbco	&REG_TMP0.b0, MASTER_REGS_CONST, VERSION2, 1

	zero			&H_FRAME, (4*2)
;init transport layer here
	CALL			transport_init
;QualityMonitor is initialized with 8
	ldi			QM, 8
;free running mode frame size is 108
	ldi			EXTRA_SIZE, 0
	ldi			NUM_STUFFING, 0
;reset PRST bit in SYS_CTRL
	lbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
	clr			REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
;reset SAFE_CTRL register
    zero        &REG_TMP0.b0, 1
	sbco        &REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
; Initialize ONLINE_STATUS_D, ONLINE_STATUS_1 and ONLINE_STATUS_2
; In ONLINE_STATUS_D high, bit 2 is FIX0, bit 4 is FIX1 and bit 5 is FIX0
; In ONLINE_STATUS_D low, bit 0 is FIX0 and bit 3 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_D_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 2
; In ONLINE_STATUS_1 high, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX1
; In ONLINE_STATUS_1 low, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_1_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 2
; In ONLINE_STATUS_2 high, bit 1 is FIX0, bit 3 is FIX0, bit 4 is FIX1 and bit7 is FIX1
; In ONLINE_STATUS_2 low, bits 0, 1, 3, 4, 5 are FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_2_HIGH_BIT4_FIX1)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 2
;check for SPOL and configure eCAP accordingly
	ldi			REG_TMP1, (ECAP+ECAP_ECCTL1)
	lbco			&REG_TMP2, PWMSS1_CONST, REG_TMP1, 4
	clr			REG_TMP2, REG_TMP2, 0
	qbbc			datalink_reset_spol_rising_edge, REG_TMP0, SYS_CTRL_SPOL
	set			REG_TMP2, REG_TMP2, 1
datalink_reset_spol_rising_edge:
	sbco			&REG_TMP2, PWMSS1_CONST, REG_TMP1, 4
;wait for pusle synch if register != 0
	lbco			&NUM_PULSES, MASTER_REGS_CONST, SYNC_CTRL, 1
	qbeq			datalink_no_sync, NUM_PULSES, 0
; remove this once external sync pulse is supported
	.if !$defined(EXT_SYNC_ENABLE)
	qba			datalink_no_sync
	.endif


;*************************************************************************************************************************;
;synchronize with SYNC Pulse here
	CALL1			sync_pulse
;*************************************************************************************************************************;

;***************************************************************************************************/
;wait logic for aligning with sync pulse for the very first  time
;pseudocode:
	;/*idea is that we should wait here and start such that first time extra edge alignes with sync pulse rise edge */
	;R20 has the pulse time in cycles. all the time here is i terms of pru cycles(4.44ns) unless specified
	;num_of_bits = 8*3 + EXTRA_SIZE //we push two times 0x00 and then actual 8 bits of frame start
	;/*each bit is 24 cycles in normal clock and 3 cycles in overclock, now extra edge is overclocked bits*
	;num_of_overclock_bits = num_of_bits*8 + TIME_REST
	;num_of_cycles = num_of_overclock_bits*3
	;wait_time = R20 - num_of_cycles
	;also we add 43 cycles to wait_time, this is coming from experimental data, we saw we were still ahead by 43 cycles
	lbco	   &R12, MASTER_REGS_CONST, SYNC_PARAM1, 4
	ldi         REG_TMP2, 0
	lbco	    &REG_TMP2, MASTER_REGS_CONST, WAIT_BEFORE_START, 2
	mov         EXTRA_EDGE_COMP, EXTRA_EDGE
	mov         EXTRA_SIZE_COMP, EXTRA_SIZE
	mov         TIME_REST_COMP, TIME_REST
	mov         NUM_STUFFING_COMP, NUM_STUFFING
;***************************************************************************************************/


sync_calc_time_rest_not_0:
	CALL1			sync_pulse
	WAIT			REG_TMP2
datalink_no_sync:
;init state machine here
;--------------------------------------------------------------------------------------------------
;State RESET
;--------------------------------------------------------------------------------------------------
datalink_reset2:
	;push dummy values to TX FIFO to gain processing time
    ;Push 8 bytes for single byte 0x00
    ;push first 4 bytes to fill fifo to max level then trigger channel for transmitting data
    ;later push further bytes in continous fifo load way
    .if $defined("HDSL_MULTICHANNEL")
	LOOP push_1b_0,4
	PUSH_FIFO_CONST			0x00
push_1b_0:
    TX_CHANNEL
   	LOOP push_2b_0,6
    WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
push_2b_0:

    .else
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
    .endif
	;send RESET 2 times to reset protocol
	ldi   LOOP_CNT_0 ,0
RESET_LOOP:
	;loop			datalink_reset2_end, 4
	;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_RESET)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL1			send_stuffing
	add 			LOOP_CNT_0, LOOP_CNT_0, 1
	qbne RESET_LOOP,LOOP_CNT_0,2
datalink_reset2_end:

;--------------------------------------------------------------------------------------------------
;State SYNC
;--------------------------------------------------------------------------------------------------
	ldi   LOOP_CNT_0 ,0
SYNC_LOOP:

datalink_sync:
	;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_SYNC)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL1			send_stuffing
	;TX_CHANNEL
datalink_sync_end:
	add 			LOOP_CNT_0, LOOP_CNT_0, 1
	qbne SYNC_LOOP,LOOP_CNT_0,16
;--------------------------------------------------------------------------------------------------
;State LEARN
; DLS response window is 1 switch bit + 61 slave answer and 12 delay bits
; at 100 meter cable we get a response delay of up to 11 bits, so we need to read max 72 bits for
; full test pattern
; VAL flag on last bit comes late. Need to rearrange code for last bit processing
; at 0 meter cable we get a response delay of 0 bits, need to make sure we dont sample to early
;--------------------------------------------------------------------------------------------------
;M_PAR_START is repeated 9 times
	;loop			datalink_learn_end, 9
	ldi			LEARN_STATE_STARTED , 1			;state indicator
	ldi			LOOP_CNT.b1, 9			;9

datalink_learn:
	;;WAIT_TX_FIFO_FREE
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_START)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
; indication of TX_DONE comes about 53ns after wire timing
	WAIT_TX_DONE
    .if $defined("FREERUN_300_MHZ")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
    .endif
 	.if $defined("HDSL_MULTICHANNEL")
	NOP_2
	NOP_2
    .endif

; measured starting point at 0 cable length
; first 8 bits will be all ones is delay from encoder and transceiver
; second 8 bits is oversampled DSL bit which is 0 on test pattern
; channel enable will always hit a high state which allows for save EDGE detection
; and SLAVE_DELAY determination
; the measured offset used below is 4 bits (OVS) * (13.333 / 4.444) + 4.444 ns.
        loop	wait_on_rx_transtion_in_learn_state, 1 ; (4*7) for 100m
        add		r0,r0,0
wait_on_rx_transtion_in_learn_state:
; now receive starts in save high state. First VAL comes after 180ns. Following ones in DSL
; bit times of 106.66 ns. Make sure code before next VAL is less than 106 ns!!!
	RX_EN

; measue the time of receive window, 2 cycles
; compensation value should be
;  108 bits
;      - 12 cycles (53 ns)
;      - 2 cycles (RESET_CYCLCNT)
;      - time to switch to TX and start sending trailer
	RESET_CYCLCNT

;read 61+11 bits
;Channel is already tiggered. First 0-1 transition will be from bit 1 to bit 2 of test pattern
;response from encoder.
;DSL data will be stored to r20-r18
	ldi			LOOP_CNT.b0, 72
	zero			&r18, (4*3)
	ldi			r1.b0, &r21.b0
; reset flag that indicates first rising edge detected in this DSL frame
	ldi 		REG_TMP11.b0, 0
; this loop executes one DSL byte in oversample mode
datalink_learn_recv_oloop:
	ldi			REG_TMP1.b1, 0
	ldi			LOOP_CNT.b2, 8

; this loop executes one DSL bit in 8x oversample mode
datalink_learn_recv_loop:
; this is after 7 cycles ~30 ns on first VAL
	qbbc			datalink_learn_recv_loop, r31, RX_VALID_FLAG
	POP_FIFO		REG_TMP0.b0
; after clearing VAL there needs to be 2 PRU cycles before we can poll for VAL again!
	CLEAR_VAL

; for each frame, detect the SAMPLE_EDGE,
; detect first falling edge which is received byte < 255
    	qbne			datalink_learn_recv_loop_not_first,REG_TMP11.b0, 0
    	qbeq			datalink_learn_recv_loop_not_first,REG_TMP0.b0,0xff
; result is in SAMPLE_EDGE and gives the sampling bit number
	FIND_EDGE		REG_TMP0.b0, REG_TMP2
; bits are counted from LSB to MSB with SET cmd
; data bits are moved MSB to LSB, hence we need to reverse the bit position
	RSB				SAMPLE_EDGE,SAMPLE_EDGE, 7

; at the 100 meter boundary do not move to next sample with
; this is 10 bits delay, and SAMPLE_EDGE with wrap around
	qbne			datalink_learn_recv_loop_100m, LOOP_CNT.b0, 64
    	qbne		    datalink_learn_recv_loop_100m, LOOP_CNT.b2, 6
	qbge			datalink_learn_recv_loop_100m, SAMPLE_EDGE, 3
; cap SAMPLE_EDGE to last bit position at 100 meter
	ldi				SAMPLE_EDGE, 0
datalink_learn_recv_loop_100m:
; temp save
	set				R22, R22, SAMPLE_EDGE
; do it only once
	ldi				REG_TMP11.b0, 1
datalink_learn_recv_loop_not_first:
	sub			LOOP_CNT.b2, LOOP_CNT.b2, 1
	qbbc		datalink_learn_recv_loop_0, REG_TMP0.w0, SAMPLE_EDGE
	set			REG_TMP1.b1, REG_TMP1.b1, LOOP_CNT.b2
datalink_learn_recv_loop_0:
;TODO: get EDGES
	mov			REG_TMP0.b1, REG_TMP0.b0
; on last byte do only 7 bits
	qbne		datalink_learn_skip_one_bit, LOOP_CNT.b0, 8
	qbne		datalink_learn_recv_loop, LOOP_CNT.b2, 1
    	qba 		datalink_learn_skip_one_bit_1
datalink_learn_skip_one_bit:
	qbne			datalink_learn_recv_loop, LOOP_CNT.b2, 0
	mvib			*--r1.b0, REG_TMP1.b1
	sub			LOOP_CNT.b0, LOOP_CNT.b0, 8
	qbne			datalink_learn_recv_oloop, LOOP_CNT.b0, 0

datalink_learn_skip_one_bit_1:
; this code section minimizes time between last VAL and TX_EN

; pre-load register to save time on last bit
;	ldi			REG_TMP2, (74*CYCLES_BIT-9) ; 100 m
    .if $defined("FREERUN_300_MHZ")
	ldi			r3, (74*CYCLES_BIT+9)
    .else
    ldi			r3, (74*CYCLES_BIT+9)
    .endif

datalink_learn_recv_loop_last_bit:

	qbbc			datalink_learn_recv_loop_last_bit, r31, RX_VALID_FLAG

; now finisch with last bit sample and store
	POP_FIFO		REG_TMP0.b0
	sub			LOOP_CNT.b2, LOOP_CNT.b2, 1
	qbbc		datalink_learn_recv_loop_final, REG_TMP0.b0, SAMPLE_EDGE
	set			REG_TMP1.b1, REG_TMP1.b1, LOOP_CNT.b2
datalink_learn_recv_loop_final:
	mov			REG_TMP0.b1, REG_TMP0.b0
    	mvib		*--r1.b0, REG_TMP1.b1

; this delay code should handle the case where both values are equal
; WAIT macro takes n+2 cycles, which is 3 for n = 1
; in case of n = 0 we skip 5 cycles wich causes the encoder to go out of sync!!!

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif

	READ_CYCLCNT	REG_TMP2
; avoid wrap around, need to skip on equal as wait does not work for 0.
;	qble	    datalink_learn_skip_wait, REG_TMP2, r3
	qble	    datalink_abort2, REG_TMP2, r3
	sub			REG_TMP11, r3, REG_TMP2
	MOV			REG_TMP2.b0, REG_TMP11.b0
; WAIT subracts -1 from parameter before compare. On 0 it wraps around!!!
	WAIT		REG_TMP11

datalink_learn_skip_wait:
	TX_EN
    .if $defined("HDSL_MULTICHANNEL")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	nop
    .endif
;send TRAILER 0x03 (skipping first 2 bits of logic 0 to avoid some extra delays)
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
	LOOP push_3b_0,3
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
push_3b_0:
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
    PUSH_FIFO_CONST		0x03
	TX_CHANNEL
    .endif
;	2 dummy cycles
	NOP_2
; test: we are in oversample mode (3 PRU clocks per bit)
; extra NOPs should make it shorter
	NOP_2
	NOP_2
	NOP_2
    .if $defined("FREERUN_300_MHZ")
	NOP_2
	NOP_2
	NOP_2
	NOP_2
	NOP_2
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV		CLKDIV_SLOW, REG_TMP2
    .endif
;reset DISPARITY
	ldi			DISPARITY, 0
	;2 dummy cycles
	NOP_2
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP2
    .endif

;reset cycle count
	RESET_CYCLCNT
datalink_learn_pattern:
	.if $defined(EXT_SYNC_ENABLE)
	.else
    WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
;add stuffing to gain processing time
	;PUSH 8 bytes for 1 byte data (0x2c) in FIFO
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00

	;PUSH 8 bytes for 1 byte data (0xb2) in FIFO
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0x00

;	PUSH 8 bytes for 1 byte data (0xcb) in FIFO
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0x00
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
;add stuffing to gain processing time
	PUSH_FIFO_CONST		0x2c
	WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xb2
	PUSH_FIFO_CONST		0xcb
    .endif  ;HDSL_MULTICHANNEL

    .endif  ;EXT_SYNC_ENABLE
;extensive search for test pattern
	ldi			LOOP_CNT.b3, 0
	qbne		datalink_learn_delay, LOOP_CNT.b1, 1

datalink_learn_delay:

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif
;shift data and remove first switch bit
	add			LOOP_CNT.b3, LOOP_CNT.b3, 1
	lsl			r20, r20, 1
	lsr			REG_TMP0.b0, r19.b3, 7
	or			r20, r20, REG_TMP0.b0
	lsl			r19, r19, 1
	lsr			REG_TMP0.b0, r18.b3, 7
	or			r19.b0, r19.b0, REG_TMP0.b0
	lsl			r18, r18, 1
;check pattern

	CALL1		check_test_pattern
	qbeq		datalink_abort2, LOOP_CNT.b3, 14
	qbne		datalink_learn_delay, REG_FNC.b0, 1
datalink_learn_end_test:
; SLAVE_DELAY has no switch bit
    	mov			SLAVE_DELAY, LOOP_CNT.b3
;send STUFFING
	.if $defined(EXT_SYNC_ENABLE)
	CALL1			send_stuffing
	.endif
datalink_learn_end:
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt		datalink_learn, LOOP_CNT.b1, 0
;we need a rel. jump here
	qba			datalink_learn2_before
;--------------------------------------------------------------------------------------------------
datalink_abort2:
	qbbs			datalink_abort2_no_wait, r30, RX_ENABLE						;changed here from 24 to 26
	WAIT_TX_DONE
    .if $defined("FREERUN_300_MHZ")
	LOOP no_operation_2cycle,9
	NOP_2
no_operation_2cycle:
    .endif
datalink_abort3:
datalink_abort2_no_wait:
	lbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
	add			REG_TMP0.b0, REG_TMP0.b0, 1
	sbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
; Set EVENT_PRST in EVENT_H register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_H, 4
	set		REG_TMP0.w0, REG_TMP0.w0, EVENT_PRST
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_H, 2
	qbbc		update_events_no_int15, REG_TMP0.w2, EVENT_PRST
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ
update_events_no_int15:
; Set EVENT_S_PRST in EVENT_S register
	lbco		&REG_TMP0, MASTER_REGS_CONST, EVENT_S, 2
	set		REG_TMP0.b0, REG_TMP0.b0, EVENT_S_PRST
;save events
	sbco		&REG_TMP0.w0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int22, REG_TMP0.b1, EVENT_S_PRST
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int22:
;we need rel. jump here
	qba			datalink_reset
;--------------------------------------------------------------------------------------------------
;M_PAR_LEARN does not seem to have further meaning...
datalink_learn2_before:
	ldi			LOOP_CNT.b1, 9; 16
datalink_learn2:
    .if !$defined("HDSL_MULTICHANNEL")
	WAIT_TX_FIFO_FREE
    .endif
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_LEARN)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL			receive
	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,12
	sbco        &R18, c25, REG_TMP2, 12
	sbco        &REG_TMP2, c25, 0, 4
	.endif
	;check test pattern
	CALL1			check_test_pattern
	qbne			datalink_abort3, REG_FNC.b0, 1
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt			datalink_learn2, LOOP_CNT.b1, 0
datalink_learn2_end:
;--------------------------------------------------------------------------------------------------
;State LINK CHECK
;this is repeated 16 times
	ldi			LOOP_CNT.b1, 16
datalink_line_check:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_CHECK)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL			receive
;check test pattern
	CALL1			check_test_pattern
	qbne			datalink_abort2, REG_FNC.b0, 1
	sub			LOOP_CNT.b1, LOOP_CNT.b1, 1
	qblt			datalink_line_check, LOOP_CNT.b1, 0
	;qba datalink_line_check
datalink_line_check_end:
;--------------------------------------------------------------------------------------------------
;State ID REQ
;save delay to master registers after all checks were successful
	sbco			&SLAVE_DELAY, MASTER_REGS_CONST, DELAY, 1
datalink_id_req:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_IDREQ)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	CALL			recv_dec
;--------------------------------------------------------------------------------------------------
;State ID STORE
datalink_id_store:
;check if 40 bits is palindrome (bytewise! not bitwise!), if not repeat this step
;-> vert == acc.b0 && par == acc.b1 && pipe.nibble0 == pipe.nibble1
;second half of palindrome is actually our ENC_ID!
	qbne			datalink_abort2, H_FRAME.vert, H_FRAME_acc0
	qbne			datalink_abort2, H_FRAME.s_par, H_FRAME_acc1
	and			REG_TMP0.b0,  H_FRAME.pipe, 0xf
	lsr			REG_TMP0.b1,  H_FRAME.pipe, 4
	qbne			datalink_abort2, REG_TMP0.b0, REG_TMP0.b1;
;now store the encoder ID
	mov			REG_TMP0.w0, H_FRAME.acc
	mov			REG_TMP0.b2, H_FRAME.pipe
;now in memory for master registers
;big endian format
	mov			REG_TMP1.b0, REG_TMP0.b2
	mov			REG_TMP1.b1, REG_TMP0.b1
	mov			REG_TMP1.b2, REG_TMP0.b0
	sbco			&REG_TMP1, MASTER_REGS_CONST, ENC_ID2, 3
;Safe QM + set LINK bit, which means we have established a link
;just add 0 to QM to update
	QM_ADD			0
;Synchronization with Drive Cycle is enabled here!
	set			H_FRAME.flags, H_FRAME.flags, FLAG_DRIVE_SYNC
;--------------------------------------------------------------------------------------------------
;State ID COMPUTE
datalink_id_compute:
;decode the ENC_ID here
;num of acc bits is always +8
	and			NUM_ACC_BITS, REG_TMP0, 0x0f
	add			NUM_ACC_BITS, NUM_ACC_BITS, 8
;num pos bits is +num acc bits
	lsr			NUM_ST_BITS, REG_TMP0, 4
	and			NUM_ST_BITS, NUM_ST_BITS, 0x3f
	add			NUM_ST_BITS, NUM_ST_BITS, NUM_ACC_BITS
	sub			NUM_ST_BITS, NUM_ST_BITS, NUM_MT_BITS
;finding the mask for position
	add			REG_TMP0.b0, NUM_ST_BITS, NUM_MT_BITS
	rsb			REG_TMP0.b0, REG_TMP0.b0, 40
	ldi32			REG_TMP1, 0xffffffff
	lsr			REG_TMP1, REG_TMP1, REG_TMP0.b0
	sbco			&REG_TMP1, MASTER_REGS_CONST, MASK_POS, 4
	;qba datalink_id_req
	CALL1		send_stuffing
	jmp         datalink_wait_vsynch

