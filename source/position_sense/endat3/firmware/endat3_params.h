;
; Copyright (C) 2025 Texas Instruments Incorporated
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
;*   File:     endat3_params.h                                            		    *
;*                                                                                  *
;*   Brief:    Register naming definitions for EnDat3 firmware and other offsets    *
;************************************************************************************

;******************************************************************************
; Register Aliases
;******************************************************************************
; General-purpose registers
	.asg r0, TEMP0
	.asg r1, TEMP1
	.asg r2, TEMP2
	.asg r3, TEMP3
	.asg r4, TEMP4
    .asg r0, TX_DATA_REG0
	.asg r1, TX_DATA_REG1
	.asg r2, TX_DATA_REG2
	.asg r3, TX_DATA_REG3
	.asg r4, TX_DATA_REG4
	.asg r5, DMEM_OFFSET
	.asg r15, ADD_DELAY
	.asg r22, TEMP_REG1

; Protocol handling registers
	.asg r1, RX_BUFFER_OFFSET
	.asg r2.b0, first_data_half_bit
	.asg r2.b1, long_short_status  ;1=long, 0=short;
	.asg r3.b0, long_symbol_count
	.asg r29.b0, TX_FRAMES_LEFT
	.asg r29.b1, CURR_TX_FRAME_MEM_OFFSET

; Error handling registers
	.asg r24.b0, ERROR_MASK_REG
	.asg r24.b1, ERROR_STATUS_REG

; Lookup table registers
	.asg r8, PREAMBLE_START_REG
	.asg r9, PREAMBLE_BASE
	.asg r10, DEC_OFFSET_REG
	.asg r12, OFFLOAD_REG
	.asg r14, DECODED_DATA_REG

; Manchester encoding registers
    .asg r27, BYTE_REV_RESULT
    .asg r28, BYTE_REV_INPUT
    .asg r8, ENCODE_INPUT_REG
    .asg r10, ENCODE_OUTPUT_REG
    .asg r3, ENCODED_DATA_HIGH
    .asg r4, ENCODED_DATA_LOW

; Data processing registers
    .asg r17, SAMPLE_OFFSET_COUNTER
    .asg r21.b0, LUT_SAMPLE_REG
    .asg r21.b2, TEMP_REG
    .asg r23, BIT_CAPTURE_REG
    .asg r27.b0, OVS_DATA_REG

;******************************************************************************
; Shared DRAM Memory Offsets (Absolute Addresses)
;******************************************************************************
PREAMBLE_START .set                     0x10000
PREAMBLE_DEC .set                       0x10020
ENDAT3_DEC_OFFS .set                    0x1208C
OFFLOAD_DATA_OFFS .set                  0x1508C

;******************************************************************************
; Command and Status Constants
;******************************************************************************
WRITE_BG_OPCODE .set 0x3
RESET_CMD .set 0xB
SAMPLING_ERROR_FLAG .set 0x2

;******************************************************************************
; Protocol Constants
;******************************************************************************
FIXED_TX_PREAMBLE .set 0x1999B29
FIXED_HELLO_CMD_DATA .set 0x22222269
RX_FIXED_PREAMBLE_HIGH .set 0x2
RX_FIXED_PREAMBLE_LOW .set 0xC9
ERROR_MASK .set 0x80
POSTAMBLE_PATTERN .set 0xFD
MIN_LONG_SYMB_COUNT .set 7
SAMPLING_DELAY_COUNT .set 37
ENDAT3_TX_START_DELAY_1 .set 0xffff
ENDAT3_TX_START_DELAY_2 .set 0x80000
ENDAT3_TX_START_DELAY_3 .set 0x20100
DELAY_10MS  .set 0x300000 ;TODO: Will work for 300MHz correctly, change this logic for to support all PRU frequency
