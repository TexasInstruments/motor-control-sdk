
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
	.ref PUSH_FIFO_2B_8x
	.ref WAIT_TX_FIFO_FREE
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
	.ref datalink_abort_jmp
	.ref receive
	.ref datalink_abort
	.global datalink_reset
	.global datalink_init_start


	; part 1 code starts here

	.sect	".text:part1"

;--------------------------------------------------------------------------------------------------
;Function: check_test_pattern (RET_ADDR1)
;This function checks if the test pattern was received
;input:
;	r18-r20: data
;output:
;	REG_FNC.b0: 1 if true
;modifies:
;	REG_TMP0, REG_FNC
;--------------------------------------------------------------------------------------------------
check_test_pattern:
;load test pattern and mask from memory

	lbco			&REG_TMP0, MASTER_REGS_CONST, TEST_PATTERN0, 12
;rm switch bit
	and			REG_TMP11, r19, REG_TMP2
	ldi			REG_TMP2, 0xff8
	and			REG_TMP2, r19, REG_TMP2
	lsl			REG_TMP2, REG_TMP2, 1
	or			REG_TMP11, REG_TMP2, REG_TMP11
;if found go to next step
	qbne			check_test_pattern_false, r20, REG_TMP0
	qbne			check_test_pattern_false, REG_TMP11, REG_TMP1
check_test_pattern_true:
	ldi			REG_FNC.b0, 1
	RET1
check_test_pattern_false:
	ldi			REG_FNC.b0, 0
	RET1

;--------------------------------------------------------------------------------------------------
;Function: sync_pulse (RET_ADDR1)
;functions bussy waits for sync pulse
;input:
;modifies:
;--------------------------------------------------------------------------------------------------
;stores sync pulse period in R20 in unit of cycles
sync_pulse:
	lbco        &REG_TMP1, c1, IEP_CAPR6_RISE, 4
wait_next_pulse:
	lbco        &R20, c1, IEP_CAPR6_RISE, 4
	QBEQ		wait_next_pulse, R20, REG_TMP1
	SUB         R20, R20, REG_TMP1
	RET1


	; common code starts here

	.sect	".text"

datalink_init_start:
datalink_reset:

; Synchronization and loading overlaid part of firmware for TXPRU (Channel 2) is needed,
; only if channel 0 and 2 are enabled


	.if !$defined(CHANNEL_2)
; For channel 2, we always need to check for synchronization, so this code is not needed
	LBCO   	&REG_TMP0.b0, MASTER_REGS_CONST, CHANNEL_MASK, 1
	qbne 	skip_overlay_load1, REG_TMP0.b0, ((1<<0) | (1<<2))
	.endif

; Set sync bit and wait for all channels
	SET_SYNC_BIT REG_TMP0
	CALL WAIT_SYNC_SET_ALL

	.if $defined(CHANNEL_0)

; Following part of code on channel 0 loads the part 1 of overlaid firmware for
; TXPRU (Channel 2)

; Disable TXPRU1 before writing into IMEM
	LDI32	REG_TMP0, TXPRU1_CTRL
	LBBO  	&REG_TMP1, REG_TMP0, 0, 4
	clr   	REG_TMP1, REG_TMP1, 1
	SBBO  	&REG_TMP1, REG_TMP0, 0, 4

; Load the load address, run address and size of part 1 of overlaid firmware
; for TXPRU (Channel 2)
	ZERO 	&REG_TMP0, 12
	LBCO   	&REG_TMP0.w0, MASTER_REGS_CONST, PART1_LOAD_START, 2
	LBCO   	&REG_TMP1.w0, MASTER_REGS_CONST, PART1_RUN_START, 2
	; Add IMEM base address to run address
	LDI32	REG_TMP2, TXPRU1_IMEM_BASE
	ADD		REG_TMP1.w0, REG_TMP1.w0, REG_TMP2.w0
	ADC		REG_TMP1.w2, REG_TMP1.w2, REG_TMP2.w2
	LBCO   	&REG_TMP2.w0, MASTER_REGS_CONST, PART1_SIZE, 2
    LDI    	REG_TMP2.w2, 0x0
memcpy_loop1:
    LBBO	&SPEED.b0, REG_TMP0,  REG_TMP2.w2, 32
    SBBO	&SPEED.b0, REG_TMP1,  REG_TMP2.w2, 32
    ADD 	REG_TMP2.w2,  REG_TMP2.w2, 32
    QBLE	memcpy_loop1,  REG_TMP2.w0, REG_TMP2.w2

	ZERO	&SPEED, (4*8)

; Enable TXPRU1 before writing into IMEM
	LDI32	REG_TMP0, TXPRU1_CTRL
	LBBO  	&REG_TMP1, REG_TMP0, 0, 4
	set   	REG_TMP1, REG_TMP1, 1
	SBBO  	&REG_TMP1, REG_TMP0, 0, 4

; Clear sync bit on RTUPRU (Channel 0).
; PRU (Channel 1) and TXPRU (Channel 2) will pend on this bit clear.
	CLEAR_SYNC_BIT REG_TMP1

	.endif

	.if $defined(CHANNEL_1)
; Wait for RTUPRU (Channel 0) to clear sync bit and clear sync bit on PRU (Channel 1)
; which signals completion of overlay part 1 load
	WAIT_SYNC_CLEAR_CH0 REG_TMP0, REG_TMP1
	CLEAR_SYNC_BIT REG_TMP1
	.endif

	.if $defined(CHANNEL_2)
; Wait for RTUPRU (Channel 0) to clear sync bit and clear sync bit on TXPRU (Channel 2)
; which signals completion of overlay part 1 load
	WAIT_SYNC_CLEAR_CH0 REG_TMP0, REG_TMP1
	CLEAR_SYNC_BIT REG_TMP1
	.endif

skip_overlay_load1:

	jmp datalink_reset_after_fw_load
	; part 1 code starts here

	.if $defined(CHANNEL_2)
	.sect	".text:part1"
	.else
	.sect	".text"
	.endif
datalink_reset_after_fw_load:
;State RESET

; Clear all registers
	zero			&r0, 124

;setup ICSS encoder peripheral for Hiperface DSL
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
    ldi     REG_TMP0.b1, ICSS_BUILD_VERSION
    sbco	&REG_TMP0.b0, MASTER_REGS_CONST, VERSION, 1
    sbco	&REG_TMP0.b0, MASTER_REGS_CONST, VERSION2, 1
    sbco	&REG_TMP0.b1, MASTER_REGS_CONST, BUILD_VERSION, 1

;init transport layer here
;Initialize transport layer here
transport_init:
;resert short msg ctrl
	ldi		REG_TMP0.b0, 0x3f
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, SLAVE_REG_CTRL, 1
;initialize acc_err_cnt to 0
	sbco		&SPEED.b0, MASTER_REGS_CONST, ACC_ERR_CNT, 1
	sbco		&SPEED.b0, MASTER_REGS_CONST, POS4, 8
;reset rel. pos
	sbco		&SPEED, MASTER_REGS_CONST, REL_POS0, 4
transport_init_abs_err_loop:
	ldi 	REG_TMP0.b0, 0
	sbco 	&REG_TMP0.b0, MASTER_REGS_CONST, ALIGN_PH, 1
exit_transport_init:
;QualityMonitor is initialized with 8
	ldi			QM, 8
;reset PRST bit in SYS_CTRL
	lbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
	clr			REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbco			&REG_TMP0, MASTER_REGS_CONST, SYS_CTRL, 1
;reset SAFE_CTRL register
    zero        &REG_TMP0.b0, 1
	sbco        &REG_TMP0.b0, MASTER_REGS_CONST, SAFE_CTRL, 1
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
	sbco		&REG_TMP0.b0, MASTER_REGS_CONST, EVENT_S, 1
	qbbc		update_events_no_int22, REG_TMP0.b1, EVENT_S_PRST
; generate interrupt
	ldi		r31.w0, PRU0_ARM_IRQ4
update_events_no_int22:
; Initialize ONLINE_STATUS_D, ONLINE_STATUS_1 and ONLINE_STATUS_2
; In ONLINE_STATUS_D high, bit 2 is FIX0, bit 4 is FIX1 and bit 5 is FIX0
; In ONLINE_STATUS_D low, bit 0 is FIX0 and bit 3 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1 and PRST bit
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_D_HIGH_BIT4_FIX1) | (1<<ONLINE_STATUS_D_PRST)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 2
; In ONLINE_STATUS_1 high, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX1
; In ONLINE_STATUS_1 low, bit 1 is FIX0, bit 3 is FIX0 and bit 4 is FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1 and PRST bit
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_1_HIGH_BIT4_FIX1) | (1<<ONLINE_STATUS_1_PRST)
	sbco        &REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_1_H, 2
; In ONLINE_STATUS_2 high, bit 1 is FIX0, bit 3 is FIX0, bit 4 is FIX1 and bit7 is FIX1
; In ONLINE_STATUS_2 low, bits 0, 1, 3, 4, 5 are FIX0
	lbco        &REG_TMP0.w0, MASTER_REGS_CONST, ONLINE_STATUS_2_H, 2
    ; clearing bits
    ldi        REG_TMP0.w0, 0
    ; setting bits with fix1 and PRST bit
    or         REG_TMP0.w0, REG_TMP0.w0, (1<<ONLINE_STATUS_2_HIGH_BIT4_FIX1) | (1<<ONLINE_STATUS_2_PRST)
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
	ldi FIFO_L,0x0
	loop aaa10,6
	CALL3 PUSH_FIFO_2B_8x
aaa10:
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
;send m_par_reset 8b/10b: 5b/6b and 3b/4b, first=0,vsync=0,reserved=0
	ldi			REG_FNC.w0, (0x0000 | M_PAR_START)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
; indication of TX_DONE comes about 53ns after wire timing
	WAIT_TX_DONE
	.if $defined("HDSL_MULTICHANNEL")
	NOP_n 11
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
    .if $defined("FREERUN_300_MHZ") | $defined("SYNC_300_MHZ")
	ldi			r3, (74*CYCLES_BIT+9)
    .else
    ldi			r3, (74*CYCLES_BIT+9)
    .endif

datalink_learn_recv_loop_last_bit:

	qbbc			datalink_learn_recv_loop_last_bit, r31, RX_VALID_FLAG

; now finish with last bit sample and store
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
	NOP_n 9
    .endif
;send TRAILER 0x03 (skipping first 2 bits of logic 0 to avoid some extra delays)
    .if $defined("HDSL_MULTICHANNEL")
	PUSH_FIFO_CONST		0x00
	TX_CHANNEL
	LOOP push_3b_0,2
	PUSH_FIFO_CONST		0x00
	CALL2 WAIT_TX_FIFO_FREE
push_3b_0:
	PUSH_FIFO_CONST		0x00
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff
    .else
    PUSH_FIFO_CONST		0x03
	TX_CHANNEL
    .endif
; test: we are in oversample mode (3 PRU clocks per bit)
; extra NOPs should make it shorter
    .if $defined("HDSL_MULTICHANNEL")
	NOP_n 8
    .endif
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV		CLKDIV_SLOW, REG_TMP2
    .endif
;reset DISPARITY
	ldi			DISPARITY, 0
	;2 dummy cycles
	nop
    .if !$defined("HDSL_MULTICHANNEL")
    TX_CLK_DIV		CLKDIV_NORMAL, REG_TMP2
    .endif

;reset cycle count
	RESET_CYCLCNT
datalink_learn_pattern:
	.if $defined(EXT_SYNC_ENABLE)
	.else
    CALL2 WAIT_TX_FIFO_FREE
    .if $defined("HDSL_MULTICHANNEL")
;add stuffing to gain processing time
	;PUSH 8 bytes for 1 byte data (0x2c) in FIFO
	ldi FIFO_L,0x2c
	loop aaa1,4
	CALL3 PUSH_FIFO_2B_8x
aaa1:

	;PUSH 8 bytes for 1 byte data (0xb2) in FIFO
	ldi FIFO_L,0xb2
	loop aaa2,4
	CALL3 PUSH_FIFO_2B_8x
aaa2:
	CALL2 WAIT_TX_FIFO_FREE
	PUSH_FIFO_CONST		0xff
	PUSH_FIFO_CONST		0xff

    .else
;add stuffing to gain processing time
	PUSH_FIFO_CONST		0x2c
	CALL2 WAIT_TX_FIFO_FREE
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
	.if !$defined(EXT_SYNC_ENABLE)
	;	PUSH 6 bit stuffing in FIFO
	ldi FIFO_L,0x2c
	loop aaa3,3
	CALL3 PUSH_FIFO_2B_8x
aaa3:
	.endif
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
datalink_abort3:
datalink_abort2_no_wait:
	lbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
	add			REG_TMP0.b0, REG_TMP0.b0, 1
	sbco			&REG_TMP0.b0, MASTER_REGS_CONST, NUM_RESETS, 1
;we need rel. jump here

	.if $defined(CHANNEL_2)
;reset PRST bit in SYS_CTRL
	ldi32 	REG_TMP1, DMEM_CH0_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi32 	REG_TMP1, DMEM_CH1_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	.if $defined(CHANNEL_1)
	ldi32 	REG_TMP1, DMEM_CH0_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi32 	REG_TMP1, DMEM_CH2_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	.if $defined(CHANNEL_0)
	ldi32 	REG_TMP1, DMEM_CH1_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1

	ldi32 	REG_TMP1, DMEM_CH2_START
	lbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	set		REG_TMP0.b0, REG_TMP0.b0, SYS_CTRL_PRST
	sbbo	&REG_TMP0, REG_TMP1, SYS_CTRL, 1
	.endif

	qba			datalink_reset
;--------------------------------------------------------------------------------------------------
;M_PAR_LEARN does not seem to have further meaning...
datalink_learn2_before:
	ldi			LOOP_CNT.b1, 9; 16
datalink_learn2:
    .if !$defined("HDSL_MULTICHANNEL")
	CALL2 WAIT_TX_FIFO_FREE
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

; Clear PRST bits in ONLINE_STATUS registers
	lbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6
    clr         REG_TMP0.w0, REG_TMP0.w0, ONLINE_STATUS_D_PRST
    clr         REG_TMP0.w2, REG_TMP0.w2, ONLINE_STATUS_1_PRST
    clr         REG_TMP1.w0, REG_TMP1.w0, ONLINE_STATUS_2_PRST
	sbco		&REG_TMP0, MASTER_REGS_CONST, ONLINE_STATUS_D_H, 6

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
	ldi 		DELTA_ACC0, 0
	;qba datalink_id_req
	CALL1		send_stuffing

; Synchronization and loading overlaid part of firmware for TXPRU (Channel 2) is needed,
; only if channel 0 and 2 are enabled

	.if !$defined(CHANNEL_2)
; For channel 2, we always need to check for synchronization, so this code is not needed
	LBCO   	&REG_TMP0.b0, MASTER_REGS_CONST, CHANNEL_MASK, 1
	qbne 	datalink_wait_vsynch, REG_TMP0.b0, ((1<<0) | (1<<2))
	.endif

	.if $defined(CHANNEL_0)

; Following part of code on channel 0 loads the part 2 of overlaid firmware for
; TXPRU (Channel 2)
datalink_loadfw_start:

	ldi			REG_FNC.w0, (0x0000 | M_PAR_IDREQ)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	WAIT_TX_DONE

	READ_IEPCNT	REG_TMP1
	LDI 		REG_TMP2, (74*CYCLES_BIT + 25)
	ADD 		REG_TMP1, REG_TMP1, REG_TMP2

	SBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

; Set sync bit and wait for all channels
	SET_SYNC_BIT REG_TMP0
	CALL WAIT_SYNC_SET_ALL

; Disable TXPRU1 before writing into IMEM
	LDI32	REG_TMP0, TXPRU1_CTRL
	LBBO  	&REG_TMP1, REG_TMP0, 0, 4
	clr   	REG_TMP1, REG_TMP1, 1
	SBBO  	&REG_TMP1, REG_TMP0, 0, 4

	LBBO  &REG_TMP1, REG_TMP0, 0, 4
	clr   REG_TMP1, REG_TMP1, 1
	SBBO  &REG_TMP1, REG_TMP0, 0, 4

; Load the load address, run address and size of part 2 of overlaid firmware
; for TXPRU (Channel 2)
	ZERO 	&REG_TMP0, 12
	LBCO   	&REG_TMP0.w0, MASTER_REGS_CONST, PART2_LOAD_START, 2
	LBCO   	&REG_TMP1.w0, MASTER_REGS_CONST, PART2_RUN_START, 2
	; Add IMEM base address to run address
	LDI32	REG_TMP2, TXPRU1_IMEM_BASE
	ADD		REG_TMP1.w0, REG_TMP1.w0, REG_TMP2.w0
	ADC		REG_TMP1.w2, REG_TMP1.w2, REG_TMP2.w2
	LBCO   	&REG_TMP2.w0, MASTER_REGS_CONST, PART2_SIZE, 2
    LDI    	REG_TMP2.w2, 0x0
memcpy_loop2:
    LBBO	&SPEED.b0, REG_TMP0,  REG_TMP2.w2, 32
    SBBO	&SPEED.b0, REG_TMP1,  REG_TMP2.w2, 32
    ADD 	REG_TMP2.w2,  REG_TMP2.w2, 32
    QBLE	memcpy_loop2,  REG_TMP2.w0, REG_TMP2.w2

	ZERO	&SPEED, (4*8)

; Enable TXPRU1 before writing into IMEM
	LDI32	REG_TMP0, TXPRU1_CTRL
	LBBO  	&REG_TMP1, REG_TMP0, 0, 4
	set   	REG_TMP1, REG_TMP1, 1
	SBBO  	&REG_TMP1, REG_TMP0, 0, 4

; Clear Synchronization bit on RTUPRU (Channel 0).
; PRU (Channel 1) and TXPRU (Channel 2) will pend on this bit clear.
	CLEAR_SYNC_BIT REG_TMP1

	LBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

datalink_loadfw_wait_for_rx_completion:
	READ_IEPCNT	REG_TMP2
	qble datalink_loadfw_wait_for_rx_completion, REG_TMP1, REG_TMP2

	TX_EN
;send TRAILER
	CALL1			send_trailer
	CALL1			send_stuffing

	jmp         datalink_wait_vsynch
	.endif

	.if $defined(CHANNEL_1)
datalink_loadfw_start:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_IDREQ)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	WAIT_TX_DONE

	READ_IEPCNT	REG_TMP1
	LDI 		REG_TMP2, (74*CYCLES_BIT + 25)
	ADD 		REG_TMP1, REG_TMP1, REG_TMP2

	SBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

; Set sync bit and wait for all channels
	SET_SYNC_BIT REG_TMP0
	CALL WAIT_SYNC_SET_ALL
; Wait for RTUPRU (Channel 0) to clear sync bit and clear sync bit on PRU (Channel 1)
; which signals completion of overlay part 2 load
	WAIT_SYNC_CLEAR_CH0 REG_TMP0, REG_TMP1
	CLEAR_SYNC_BIT REG_TMP1

	LBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

datalink_loadfw_wait_for_rx_completion:
	READ_IEPCNT	REG_TMP2
	qble datalink_loadfw_wait_for_rx_completion, REG_TMP1, REG_TMP2

	TX_EN
;send TRAILER
	CALL1			send_trailer
	CALL1			send_stuffing

	jmp         datalink_wait_vsynch

	.endif

	.if $defined(CHANNEL_2)
datalink_loadfw_start:
	ldi			REG_FNC.w0, (0x0000 | M_PAR_IDREQ)
	.if $defined("HDSL_MULTICHANNEL")
	CALL			send_header_300m
	.else
	CALL			send_header
	.endif
	WAIT_TX_DONE

	READ_IEPCNT	REG_TMP1
	LDI 		REG_TMP2, (74*CYCLES_BIT + 25)
	ADD 		REG_TMP1, REG_TMP1, REG_TMP2

	SBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

	jmp 		datalink_loadfw_continue

;  Jumping to .text only for channel 2
	.sect ".text"

datalink_loadfw_continue:

; Set sync bit and wait for all channels
	SET_SYNC_BIT REG_TMP0
	CALL WAIT_SYNC_SET_ALL

; Wait for RTUPRU (Channel 0) to clear sync bit and clear sync bit on TXPRU (Channel 2)
; which signals completion of overlay part 2 load
	WAIT_SYNC_CLEAR_CH0 REG_TMP0, REG_TMP1
	CLEAR_SYNC_BIT REG_TMP1

	LBCO 		&REG_TMP1, MASTER_REGS_CONST, LOADFW_TIMESTAMP, 4

datalink_loadfw_wait_for_rx_completion:
	READ_IEPCNT	REG_TMP2
	qble datalink_loadfw_wait_for_rx_completion, REG_TMP1, REG_TMP2

	TX_EN
;send TRAILER
	CALL1			send_trailer
	CALL1			send_stuffing

	jmp         datalink_wait_vsynch

	.endif

WAIT_SYNC_SET_ALL:
	LBCO   	&REG_TMP1.b1, MASTER_REGS_CONST, CHANNEL_MASK, 1
	QBBC 	wait_for_channel1?, REG_TMP1.b1, 0
wait_for_channel0?:
	LDI 	REG_TMP0, DMEM_CH0_START
	LBBO	&REG_TMP1.b0, REG_TMP0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel0?, REG_TMP1.b0, 1
wait_for_channel1?:
	QBBC 	wait_for_channel2?, REG_TMP1.b1, 1
	LDI 	REG_TMP0, DMEM_CH1_START
	LBBO	&REG_TMP1.b0, REG_TMP0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel1?, REG_TMP1.b0, 1
wait_for_channel2?:
	QBBC 	wait_sync_clear_all_end?, REG_TMP1.b1, 2
	LDI 	REG_TMP0, DMEM_CH2_START
	LBBO	&REG_TMP1.b0, REG_TMP0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel2?, REG_TMP1.b0, 1
wait_sync_clear_all_end?:
	RET
