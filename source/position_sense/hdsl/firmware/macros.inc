
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

CALL .macro func
    jal     RET_ADDR0, func
    .endm

RET .macro
    JMP     RET_ADDR0
    .endm

CALL1 .macro func
    jal     RET_ADDR1, func
    .endm

RET1 .macro
    JMP     RET_ADDR1
    .endm
CALL2 .macro func
    jal     RET_ADDR2, func
    .endm

RET2 .macro
    JMP     RET_ADDR2
    .endm

CALL3 .macro func
    jal     RET_ADDR3, func
    .endm

RET3 .macro
    JMP     RET_ADDR3
    .endm


QM_ADD .macro val
    ldi     REG_FNC.b0, (val & 0xff)
    add     QM, QM, REG_FNC.b0
    CALL1       qm_add
    .endm

QM_SUB .macro val
    ldi     REG_FNC.b0, (val & 0xff)
    sub     QM, QM, REG_FNC.b0
    CALL1       qm_add
    .endm

FIND_EDGE .macro src, tmp
    LOOKUP_BITCNT       tmp, src
    add         SAMPLE_EDGE, tmp, 4
    WRAP_AROUND     SAMPLE_EDGE, 8
    .endm

WRAP_AROUND .macro reg, threshold
    qbgt        l?, reg, threshold
    sub         reg, reg, threshold;threshold/2
    ;ldi            reg, (threshold-1)
l?:
    .endm

LOOKUP_BITCNT .macro dst, src
    ldi         dst, (PDMEM00+LUT_BITCNT)
    lbbo        &dst, dst, src, 1
    .endm

SAMPLE .macro dst, src
    ldi         dst, (PDMEM00+LUT_BITCNT)
    lbbo        &dst, dst, src, 1
    qblt        l?, dst, 4
    ldi         dst, 0
    jmp         end?
l?:
    ldi         dst, 1
end?:
    .endm

WAIT .macro delay
    lsr delay, delay, 1
l?: sub delay,delay,1
    qblt l?,delay,0
    .endm

WAIT_X24 .macro reg, offset
    lsl     reg, reg, 3
    sub     reg, reg, offset
l?:
    and     r0, r0, r0
    sub     reg, reg, 1
    qblt    l?, reg, 0
    .endm

WAIT_BITS .macro bits, reg
    ldi32       reg, (CYCLES_BIT/2*bits-1)
l?:
    sub     reg, reg, 1
    qblt    l?, reg, 0
    .endm

TX_EN .macro
	.if $defined(CHANNEL_2)
	clr		r30.b3, r30.b3, 2
	.endif
	.if $defined(CHANNEL_1)
	clr		r30.b3, r30.b3, 1
	.endif
	.if $defined(CHANNEL_0)
	clr		r30.b3, r30.b3, 0
	.endif
	.endm

;4 cycles
;tx delay is always 0
TX_FRAME_SIZE .macro bits, tmp0
	ldi		tmp0, (bits<<11)
	.if $defined(CHANNEL_2)
	;ldi     	REG_SCRATCH, P0EDCH2CFG0
	sbco		&tmp0, ICSS_CFGx, EDCH2CFG0, 2
	.endif
	.if $defined(CHANNEL_1)
	;ldi     	REG_SCRATCH, P0EDCH1CFG0
	sbco		&tmp0, ICSS_CFGx, EDCH1CFG0, 2
	.endif
	.if $defined(CHANNEL_0)
	;ldi     	REG_SCRATCH, P0EDCH0CFG0
	sbco		&tmp0, ICSS_CFGx, EDCH0CFG0, 2
	.endif
	.endm

;4 cycles
TX_CLK_DIV .macro div, tmp0
    ldi         tmp0.w2, div
    ;ldi        REG_SCRATCH, P0EDTXCFG+2
    sbco        &tmp0.w2, ICSS_CFGx, EDTXCFG+2, 2
    .endm

TX_CLK_DIV_WAIT .macro div,tmp0
    nop
    nop
    nop
    .endm

WAIT_TX_DONE .macro
    ;WAIT_TX_FIFO_CLEAR
l?:
	qbbs		l?, r31, TX_GLOBAL_REINIT_ACTIVE
	.endm



SET_TX_CH0 .macro
	and				r30.b2, r30.b2, 0xfc ;clear the channel select bits
	.if !$defined(CHANNEL_0)
	set				r30.b2, r30.b2, CHANNEL_NUM
	.endif
	.endm

CLKMODE .macro
    clr         r30.b2, r30.b2, 3
    ;set            r30.b2, r30.b2, 4
    ;ldi                r30.b2, 0x00
    ;ldi            r30.b2, 0x08
    ;ldi            r30.b2, 0x10
    ;ldi            r30.b2, 0x18
    .endm

REINIT_TX .macro
	set			r31, r31, TX_GLOBAL_REINIT
	TX_EN
	set			r31, r31, TX_GLOBAL_REINIT
l?:
    ;qbbs       l?, r31, 21
    loop    loop1?, 50
    add R0, R0, 0
loop1?:

    CLKMODE
    .endm

REINIT_TX_CNT .macro reg
	ldi			reg, 6
	set			r31, r31, TX_GLOBAL_REINIT
	TX_EN
	set			r31, r31, TX_GLOBAL_REINIT
l?:
	add			reg, reg, 2
	qbbs		l?, r31, TX_GLOBAL_REINIT_ACTIVE
	CLKMODE
	.endm

TX_CHANNEL .macro
	set			r31, r31, TX_CHANNEL_GO
	.endm

PUSH_FIFO_CONST .macro dat
    ldi         r30.b0, dat
    .endm

PUSH_FIFO .macro dat
    mov         r30.b0, dat
    .endm

POP_FIFO .macro dst
	.if $defined(CHANNEL_2)
	mov			dst, r31.b2
	.endif
	.if $defined(CHANNEL_1)
	mov			dst, r31.b1
	.endif
	.if $defined(CHANNEL_0)
	mov			dst, r31.b0

	.if $defined(EXT_SYNC_ENABLE_DEBUG)
	lbco        &REG_TMP2, c25, 0, 4
	add         REG_TMP2,REG_TMP2,4
	sbco        &dst, c25, REG_TMP2, 1
	sbco        &REG_TMP2, c25, 0, 4
	.endif

	.endif
	.endm

RX_EN .macro
	.if $defined(CHANNEL_2)
	set				r30.b3, r30.b3, 2
	.endif
	.if $defined(CHANNEL_1)
	set				r30.b3, r30.b3, 1
	.endif
	.if $defined(CHANNEL_0)
	set				r30.b3, r30.b3, 0
	.endif
	.endm

REINIT_RX .macro
	set			r31, r31, TX_GLOBAL_REINIT
l?:
	qbbs		l?, r31, TX_GLOBAL_REINIT_ACTIVE
	RX_EN
	CLKMODE
	.endm

CLEAR_VAL .macro
	set			r31, r31, RX_VALID_FLAG
	.endm

CLEAR_OVF .macro
	set			r31, r31, RX_OVERFLOW_FLAG
	.endm

;CLEAR_VAL_OVF .macro
    ;ldi            r31.b3, 0x24
    ;.endm


RESET_CYCLCNT .macro
    sbco        &r0, PRU_CTRL_CONST, PRU_CYCLCNT, 4
    .endm

READ_CYCLCNT .macro dst
    lbco        &dst, PRU_CTRL_CONST, PRU_CYCLCNT, 4
    .endm

; Reads lower 32 bits of IEP0 Timer
READ_IEPCNT .macro dst
    lbco        &dst, IEP_CONST, IEP_CNT, 4
    .endm

WAIT_CLK_HIGH .macro tmp
l?:
	;ldi     		REG_SCRATCH, P0EDTXCFG
	lbco			&tmp, ICSS_CFGx, EDTXCFG, 4
	;//here 8 for endat0, 9 for endat1 and 10 for endat2
	.if $defined(CHANNEL_2)
	qbbc			l?, tmp, 10
	.endif
	.if $defined(CHANNEL_1)
	qbbc			l?, tmp, 9
	.endif
	.if $defined(CHANNEL_0)
	qbbc			l?, tmp, 8
	.endif
	.endm

WAIT_CLK_LOW .macro tmp
l?:
	;ldi     		REG_SCRATCH, P0EDTXCFG
	lbco			&tmp, ICSS_CFGx, EDTXCFG, 4
	;//here 8 for endat0, 9 for endat1 and 10 for endat2
	.if $defined(CHANNEL_2)
	qbbs			l?, tmp, 10
	.endif
	.if $defined(CHANNEL_1)
	qbbs			l?, tmp, 9
	.endif
	.if $defined(CHANNEL_0)
	qbbs			l?, tmp, 8
	.endif
	.endm

RESET_ECAP_INT .macro tmp0, tmp1
    ldi         tmp0, (ECAP+ECAP_ECCLR)
    ldi         tmp1, 0xffff
    sbco            &tmp1, PWMSS1_CONST, tmp0, 2
    .endm

NOP_n .macro dat
    loop lw?,dat
    and         r0, r0, r0
lw?:
    .endm

; Set Sync bit
SET_SYNC_BIT .macro tmp0
	LDI 	tmp0.b0, 0x1
	SBCO	&tmp0.b0, MASTER_REGS_CONST, CHANNEL_SYNC, 1
	.endm

; Clear Sync bit
CLEAR_SYNC_BIT .macro tmp0
	LDI 	tmp0.b0, 0x0
	SBCO	&tmp0.b0, MASTER_REGS_CONST, CHANNEL_SYNC, 1
	.endm

WAIT_SYNC_CH0 .macro tmp0, tmp1
wait_for_channel0?:
	LDI 	tmp0, DMEM_CH0_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel0?, tmp1.b0, 1
	.endm

WAIT_SYNC_CH1 .macro tmp0, tmp1
wait_for_channel1?:
	LDI 	tmp0, DMEM_CH1_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel1?, tmp1.b0, 1
	.endm

WAIT_SYNC_CH2 .macro tmp0, tmp1
wait_for_channel2?:
	LDI 	tmp0, DMEM_CH2_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel2?, tmp1.b0, 1
	.endm

WAIT_SYNC_CLEAR_CH0 .macro tmp0, tmp1
wait_for_channel0?:
	LDI 	tmp0, DMEM_CH0_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel0?, tmp1.b0, 0
	.endm

WAIT_SYNC_CLEAR_CH1 .macro tmp0, tmp1
wait_for_channel1?:
	LDI 	tmp0, DMEM_CH1_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel1?, tmp1.b0, 0
	.endm

WAIT_SYNC_CLEAR_CH2 .macro tmp0, tmp1
wait_for_channel2?:
	LDI 	tmp0, DMEM_CH2_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel2?, tmp1.b0, 0
	.endm

WAIT_SYNC_CLEAR_ALL .macro tmp0, tmp1
	LBCO   	&tmp1.b1, MASTER_REGS_CONST, CHANNEL_MASK, 1
	QBBC 	wait_for_channel1?, tmp1.b1, 0
wait_for_channel0?:
	LDI 	tmp0, DMEM_CH0_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel0?, tmp1.b0, 0
wait_for_channel1?:
	QBBC 	wait_for_channel2?, tmp1.b1, 1
	LDI 	tmp0, DMEM_CH1_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel1?, tmp1.b0, 0
wait_for_channel2?:
	QBBC 	wait_sync_clear_all_end?, tmp1.b1, 2
	LDI 	tmp0, DMEM_CH2_START
	LBBO	&tmp1.b0, tmp0, CHANNEL_SYNC, 1
    QBNE  	wait_for_channel2?, tmp1.b0, 0	
wait_sync_clear_all_end?:	
	.endm
