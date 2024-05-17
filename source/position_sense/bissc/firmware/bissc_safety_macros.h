;
; Copyright (C) 2024 Texas Instruments Incorporated
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
;*	 File:	bissc_safety_macros.h												*
;*																					*
;*	 Brief: Macros Receiveing raw data (position data + ew + sign of life counter)  *
;* 	 Note:  This macro is designed to receive BiSS-C SCD(single cycle data) which	*
;* 			supports 16-bit CRC safety												*
;************************************************************************************


	.include "bissc_icss_reg_defs.h"
	.include "bissc_params.h"
	.include "bissc_interface.h"
	.include "firmware_version.h"



; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_SAFETY
; 	Receive and Downample the raw data bits.
; Registers:
;	Rx: Holds the raw data received - caller has to zero or provide value to continue
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be data length + ew length + sign of life counter.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry and Wait for valid bit then check the Middle Bit of Rx fifo and store the same in Rx and Ry.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 10(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY	.macro	Rx, cnt, bit_idx
	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP_SAFETY?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
WB2_SAFETY?:
	AND 	SCRATCH.b0, R31.b3, CH_MASK
	QBNE	WB2_SAFETY?, SCRATCH.b0, CH_MASK ;  wait for valid
	QBBS	BISSC_SET_CRC_BIT_CHx_SAFETY?,	R31,	bit_idx ;  Check the Mid bit of received oversampled data
	QBA		BISSC_SKIP_CRC_BIT_CHx_SAFETY?
BISSC_SET_CRC_BIT_CHx_SAFETY?:
	OR		Rx,	Rx,	1
BISSC_SKIP_CRC_BIT_CHx_SAFETY?:
	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
	NOP
	NOP
	NOP
BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP_SAFETY?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_SAFETY
; 	Receive and downsaple the raw data for single cycle data.
; Registers:
;	Ra, Rb: Holds the raw data received - caller has to zero or provide value to continue
;	SCRATCH3.b3: rx_size
;	RAW_DATA_LEN: position data length with error warning length + sign of life counter length and 16bit crc length.
;
;  PseudoCode:
;	(start code)
;       1.Add safety crc length to usual raw data length.
;       2.Receive position data Ra and Rb with respect to the data length and Register length by calling onto
;		  M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY.
;		3.Store the raw data to its respective offsets.
;	(end code)
;
;	 Worst case peak cycle usage:
; REVISIT: More optimization may be required or can be done at least in M_INIT_CRC_FLIP_FLOPS - defer those till a requirement comes
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_SAFETY	.macro	Ra, Rb, encoder_offset
    ADD     RAW_DATA_LEN, RAW_DATA_LEN, BISSC_SAFETY_CRC_LEN						; pos data + e+ w
	; receive data
	QBGE	BISSC_RX_LE_32_BITS_SAFETY?,	RAW_DATA_LEN,	32
    SUB     SCRATCH3.b3, RAW_DATA_LEN, 32
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY	Ra,	SCRATCH3.b3, FIFO_BIT_IDX
	;SUB		SCRATCH3.b3,	RAW_DATA_LEN,   SCRATCH3.b3
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY	Rb, 32, FIFO_BIT_IDX
	JMP		BISSC_RX_CRC_SAFETY?

BISSC_RX_LE_32_BITS_SAFETY?:
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY	Rb, RAW_DATA_LEN, FIFO_BIT_IDX
BISSC_RX_CRC_SAFETY?:
	ADD 	ENCODER_OFFSET, SCRATCH2.b0, SCRATCH2.b3
	SBCO	&Ra, PRUx_DMEM,	ENCODER_OFFSET,	8
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_MC
; 	Receive and Downample the raw data along with safety CRC for multi channel.
; Registers:
;	Rx, Ry, Rz: Holds the raw data received for all channels - caller has to zero or provide value to continue
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 6 as we are receiving the CRC.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry, Rz, and Wait for valid bit then check the Middle Bit of Rx fifo for all channels and store the same in Rx, Ry, Rz.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 18(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY_MC	.macro	Rx, Ry, Rz, cnt, bit_idx
	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP_SAFETY?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL		Rz,	Rz,	1
WB2_SAFETY?:
	AND		SCRATCH.b0,	R31.b3, CH_MASK
	QBNE	WB2_SAFETY?,	SCRATCH.b0,	CH_MASK ;  wait for valid

	QBBS	BISSC_SET_CRC_BIT_CH0_SAFETY?,	R31.b0,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH0_SAFETY?
BISSC_SET_CRC_BIT_CH0_SAFETY?:
	OR		Rx,	Rx,	1
BISSC_SKIP_CRC_BIT_CH0_SAFETY?:

	QBBS    BISSC_SET_CRC_BIT_CH1_SAFETY?,	R31.b1,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH1_SAFETY?
BISSC_SET_CRC_BIT_CH1_SAFETY?:
	OR		Ry,	Ry,	1
BISSC_SKIP_CRC_BIT_CH1_SAFETY?:

	QBBS    BISSC_SET_CRC_BIT_CH2_SAFETY?,	R31.b2,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH2_SAFETY?
BISSC_SET_CRC_BIT_CH2_SAFETY?:
	OR		Rz,	Rz,	1
BISSC_SKIP_CRC_BIT_CH2_SAFETY?:

	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP_SAFETY?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_MC
; 	Receive and downsaple the raw data with safety CRC support for multichannel.
; Registers:
;	Ra, Rb, Rc, Rd, Re, Rf: Holds the raw data received - caller has to zero or provide value to continue
;	SCRATCH3.b3: rx_size
;	RAW_DATA_LEN: position data length with error warning length + sign of life counter and 16bit safety CRC lengths
;
;  PseudoCode:
;	(start code)
;		1.Add 16-bit crc length to usual raw data length
;       2.Receive position data Ra and Rb with respect to the raw data length and Register length by calling onto
;		  M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY_MC.
;		3.Store the position data and crc to thier respective offsets and check for CRC validation between BISSC_RCV_CRC_0 and SCRATCH3.b3.
;       6.Increment the error counter if CRC not matched.
;	(end code)
;
;	 Worst case peak cycle usage:
; REVISIT: More optimization may be required or can be done at least in M_INIT_CRC_FLIP_FLOPS - defer those till a requirement comes
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_SAFETY_MC	.macro	Ra, Rb, Rc, Rd, Re, Rf, encoder_offset

    ADD     RAW_DATA_LEN, RAW_DATA_LEN, BISSC_SAFETY_CRC_LEN						; pos data + e+ w
	; receive data
	QBGE	BISSC_RX_LE_32_BITS_SAFETY?,	RAW_DATA_LEN,	32
    SUB     SCRATCH3.b3, RAW_DATA_LEN, 32
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY_MC	Ra,	Rc, Re, SCRATCH3.b3, FIFO_BIT_IDX
	;SUB		SCRATCH3.b3,	RAW_DATA_LEN,   SCRATCH3.b3
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY_MC	Rb, Rd, Rf, 32, FIFO_BIT_IDX
	JMP		BISSC_RX_CRC_SAFETY?

BISSC_RX_LE_32_BITS_SAFETY?:
	M_OTF_RECEIVE_AND_DOWNSAMPLE_SAFETY_MC	Ra, Rc, Re, RAW_DATA_LEN, FIFO_BIT_IDX
BISSC_RX_CRC_SAFETY?:
	SBCO	&Ra,	PRUx_DMEM,	encoder_offset,	24
	.endm


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_BISSC_SAFETY_POST_PROCESSING
; 	Calculate 16-bit safety CRC post receiving the position data + ew + sign of life counter.
; Registers:
;	SCRATCH3: Holds channel wise DMEM offsets with channel number
;	RAW_DATA1_0, RAW_DATA2_0: holds lower and upper position data words
;	SCRATCH2: Holds the data under processing(crc calculation).
; 	SCRATCH:  Holds CRC polynomial
;	SCRATCH1.b0: Holds LMBD value of SCRATCH2
; 	SCRATCH1.b3: Holds Number bits to be loaded from lower word
;
;  PseudoCode:
;	(start code)
;       1.Load the currently selected channel data
;		2.Load Upper word in processing register if total data length < 32 else load lower word.
;       3.Load CRC Polynomial and adjust is MSB according to the LMBD of processing register.
;		4.Perform XOR of CRC polynomial and processing register only if processing reg contents > CRC polynomial.
;		5.Else load next 16-bits from lower register.
; 		6.Repeat 3-4 untill contents of lower register is 0.
;		7.Invert the CRC in processing register and compare with the received CRC, increment the CRC error count if both does not match.
; 		8.Increment the dmem offsets and channel in use.
; 		9.Repeat steps 1-8 for all configured channels.
;	(end code)
;
;	 Worst case peak cycle usage: 64
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_BISSC_SAFETY_POST_PROCESSING .macro
	LDI 	SAFETY_ENCODER.b0, BISSC_POS_DATA_RES_ENCODER0_OFFSET
	LDI 	SAFETY_ENCODER.b1, BISSC_ENC0_SAFETY_CRC_OFFSET
	LDI 	SAFETY_ENCODER.b2, 0
BISSC_NEXT_ENCODER_SAFETY?:
	QBBC 	BISSC_CALC_NEXT_ENC_CRC?, STATUS_REG1, SAFETY_ENCODER.b2
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	ADD 	SCRATCH3.b0, 	SAFETY_ENCODER.b1, BISSC_CH0_SAFETY_CRC_OFFSET
	ADD 	SCRATCH3.b1, 	SAFETY_ENCODER.b0, BISSC_POS_DATA_WORD_CH0_OFFSET
	LDI 	SCRATCH3.b2, 	0
	ADD 	SCRATCH3.b3, 	SAFETY_ENCODER.b0, BISSC_POS_CRC_ERR_COUNT_CH0_OFFSET
BISSC_MULTI_CHANNEL_SAFETY?:
	.else
	QBBC 	BISSC_CH0_SAFETY?, CH_MASK, 0	;load channel 0 offset
	ADD 	SCRATCH3.b0, 	SAFETY_ENCODER.b1, BISSC_CH0_SAFETY_CRC_OFFSET
	ADD 	SCRATCH3.b1, 	SAFETY_ENCODER.b0, BISSC_POS_DATA_WORD_CH0_OFFSET
	LDI 	SCRATCH3.b2, 	0
	ADD 	SCRATCH3.b3, 	SAFETY_ENCODER.b0, BISSC_POS_CRC_ERR_COUNT_CH0_OFFSET
BISSC_CH0_SAFETY?:
	QBBC 	BISSC_CH1_SAFETY?, CH_MASK, 1	;load channel 1 offset
	ADD 	SCRATCH3.b0, 	SAFETY_ENCODER.b1, BISSC_CH1_SAFETY_CRC_OFFSET
	ADD 	SCRATCH3.b1, 	SAFETY_ENCODER.b0, BISSC_POS_DATA_WORD_CH1_OFFSET
	LDI 	SCRATCH3.b2, 	1
	ADD 	SCRATCH3.b3, 	SAFETY_ENCODER.b0, BISSC_POS_CRC_ERR_COUNT_CH1_OFFSET
BISSC_CH1_SAFETY?:
	QBBC 	BISSC_SKIP_SAFETY?, CH_MASK, 2	;load channel 2 offset
	ADD 	SCRATCH3.b0, 	SAFETY_ENCODER.b1, BISSC_CH2_SAFETY_CRC_OFFSET
	ADD 	SCRATCH3.b1, 	SAFETY_ENCODER.b0, BISSC_POS_DATA_WORD_CH2_OFFSET
	LDI 	SCRATCH3.b2, 	2
	ADD 	SCRATCH3.b3, 	SAFETY_ENCODER.b0, BISSC_POS_CRC_ERR_COUNT_CH2_OFFSET
BISSC_SKIP_SAFETY?:
	.endif
	QBBC 	BISSC_END_SAFETY_LOOP?, CH_MASK, SCRATCH3.b2 ;skip crc calculation if current channel is not configured
	LBCO 	&RAW_DATA1_0, PRUx_DMEM, SCRATCH3.b1, 8
	LDI32 	SCRATCH, BISSC_SAFETY_CRC_POLY
	MOV 	SCRATCH2, RAW_DATA1_0						;load upper word of raw data into local register
	LDI 	SCRATCH1.b3, 32								;bits to loaded from next lower word
	QBNE 	BISSC_SKIP_UPPER_WORD?, SCRATCH2, 0			;if upper word is zero then load lower word
	MOV 	SCRATCH2, RAW_DATA2_0
	LDI 	SCRATCH2.w0, 0
	LDI 	SCRATCH1.b3, 0
BISSC_SKIP_UPPER_WORD?:
	MOV 	BISSC_RCV_CRC.w0, RAW_DATA2_0.w0
	LDI 	RAW_DATA2_0.w0, 0 							;clear the 16-bit crc received
	LMBD 	SCRATCH1.b0, SCRATCH2, 1
	QBGT 	BISSC_LOAD_NEXT_WORD?, SCRATCH1.b0, BISSC_SAFETY_CRC_LEN	;load next word(RAW_DATA2_0.w2) if lmbd < crc length
	SUB 	SCRATCH1.b0, SCRATCH1.b0, BISSC_SAFETY_CRC_LEN	;subtract 16bit crc length from lmbd to get shift value
	LSL 	SCRATCH, SCRATCH, SCRATCH1.b0					;shift the crc polynomial to match leading 1's of raw data and polynomial
BISSC_SAFETY_CRC_LOOP?:
	XOR 	SCRATCH2, SCRATCH2, SCRATCH
BISSC_LOAD_NEXT_WORD?:

	LMBD 	SCRATCH1.b0, SCRATCH2, 1
	QBEQ 	BISSC_LMBD_IS_0?, SCRATCH1.b0, 32				;if the contents of input reg is 0, lmbd value will be 32
	QBLE 	BISSC_SKIP_NEXT_WORD?, SCRATCH1.b0, BISSC_SAFETY_CRC_LEN	;load next word if lmbd < crc length
BISSC_LMBD_IS_0?:
	QBEQ 	BISSC_END_SAFETY_LOOP?, RAW_DATA1_0, 0	;if upper word is 0 do not load any word
	LSL 	SCRATCH2, SCRATCH2, 16					;append 16 bits if the lmbd value is less than 16
	LSR 	RAW_DATA1_1, RAW_DATA2_0, (32 - 16)		;extract upper 16 bits from lower register
	OR 		SCRATCH2, SCRATCH2, RAW_DATA1_1			;get those 16 bit in upper reg
	LSL		RAW_DATA2_0, RAW_DATA2_0, 16			;retain lower 16 bits to upper position
	LMBD	SCRATCH1.b0, SCRATCH2, 1
	QBNE 	BISSC_LOWER_WORD_DATA?, RAW_DATA2_0, 0
	QBNE 	BISSC_LOWER_WORD_DATA?, SCRATCH1.b3, 0				;if lower register content and number of bits to be loaded is 0 end crc calculation
	LSR 	SCRATCH2, SCRATCH2, 16
	QBA 	BISSC_END_SAFETY_LOOP?
BISSC_LOWER_WORD_DATA?:
	SUB 	SCRATCH1.b3, SCRATCH1.b3, 16 		;bits taken from lower register
BISSC_SKIP_NEXT_WORD?:
	QBNE 	BISSC_DATA_GT_CRC_POLY?, SCRATCH1.b0, BISSC_SAFETY_CRC_LEN
	QBNE 	BISSC_DATA_GT_CRC_POLY?, SCRATCH1.b3, 0				;if crc length == lmbd and number of bits to be loaded from lower reg is 0
	LDI32 	SCRATCH, BISSC_SAFETY_CRC_POLY			;perform XOR to get data < crc polynomial and end crc calculation
	XOR 	SCRATCH2, SCRATCH2, SCRATCH
	QBA 	BISSC_END_SAFETY_LOOP?
BISSC_DATA_GT_CRC_POLY?:
	SUB 	SCRATCH1.b0, SCRATCH1.b0, BISSC_SAFETY_CRC_LEN	;continue crc calculation if crc polynomial < data
	LDI32 	SCRATCH, BISSC_SAFETY_CRC_POLY
	QBGT 	BISSC_DATA_LT_CRC_POLY?, SCRATCH2, SCRATCH
	LSL 	SCRATCH, SCRATCH, SCRATCH1.b0
	QBA 	BISSC_SAFETY_CRC_LOOP?
BISSC_DATA_LT_CRC_POLY?:
	QBNE	BISSC_SAFETY_CRC_LOOP?, SCRATCH1.b3, 0			;continue crc calculation untill all bits from lower regiter is covered
BISSC_END_SAFETY_LOOP?:
	NOT 	SCRATCH2.w0, SCRATCH2.w0						;Received CRC is inverted
	SBCO 	&SCRATCH2, PRUx_DMEM, SCRATCH3.b0, 2
	QBEQ 	BISSC_SAFETY_CRC_SUCCESS?, SCRATCH2.w0, BISSC_RCV_CRC.w0
	LBCO 	&SCRATCH, PRUx_DMEM, SCRATCH3.b3, 4
	ADD 	SCRATCH, SCRATCH, 1
	SBCO 	&SCRATCH, PRUx_DMEM, SCRATCH3.b3, 4
BISSC_SAFETY_CRC_SUCCESS?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	ADD 	SCRATCH3.b0, SCRATCH3.b0, 2
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 8
	ADD 	SCRATCH3.b2, SCRATCH3.b2, 1
	ADD 	SCRATCH3.b3, SCRATCH3.b3, 4
	QBGE 	BISSC_MULTI_CHANNEL_SAFETY?, SCRATCH3.b2, 2
	.endif
BISSC_CALC_NEXT_ENC_CRC?:
	ADD		SAFETY_ENCODER.b0, SAFETY_ENCODER.b0, (BISSC_POS_DATA_RES_ENCODER1_OFFSET - BISSC_POS_DATA_RES_ENCODER0_OFFSET)
	ADD 	SAFETY_ENCODER.b1, SAFETY_ENCODER.b1, (BISSC_ENC1_SAFETY_CRC_OFFSET - BISSC_ENC0_SAFETY_CRC_OFFSET)
	ADD 	SAFETY_ENCODER.b2, SAFETY_ENCODER.b2, 1
	QBGE 	BISSC_NEXT_ENCODER_SAFETY?, SAFETY_ENCODER.b2, 2
	.endm