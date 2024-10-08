
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

	.sect	".text"
	.global update_events
	.global demp_data_symbols
	.global calc_acc_crc
	.global calc_16bit_crc
	.global load_code

;--------------------------------------------------------------------------------------------------
;Function: calc_16bit_crc (RET_ADD1)
;This function checks the crc for the acceleration channel (polynomial is x^5 + x^2 + 1)
;11*REG_FNC.b2+3 cycles -> 69 cycles for 6 bytes data
;input:
;	REG_FNC.b0: num bytes
;	r1.b0: pointer to register + 1 byte, counting down from there
;output:
;	REG_FNC.w0: 16 bit CRC
;modifies:
;	REG_TMP0, REG_FNC, r1
;--------------------------------------------------------------------------------------------------
calc_16bit_crc:
	ldi		REG_TMP2, (LUT_CRC16+PDMEM00)
	mov		REG_TMP0.b2, REG_FNC.b0
	ldi		REG_FNC.w0, 0
calc_16bit_crc_loop:
	ldi		REG_TMP0.w0, 0
	mvib		REG_TMP0.b0, *--r1.b0
	xor		REG_TMP0.b0, REG_FNC.b1, REG_TMP0.b0
	lsl		REG_TMP0.w0, REG_TMP0.w0, 1
	lbbo		&REG_TMP0.w0, REG_TMP2, REG_TMP0.w0, 2
	lsl		REG_FNC.w0, REG_FNC.w0, 8
	xor		REG_FNC.w0, REG_TMP0.w0, REG_FNC.w0
	sub		REG_TMP0.b2, REG_TMP0.b2, 1
	qblt		calc_16bit_crc_loop, REG_TMP0.b2, 0
;CRC_L is flipped
	xor		REG_FNC.b0, REG_FNC.b0, 0xff
	RET1
