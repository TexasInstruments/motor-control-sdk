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
; NOTE: Each register now has a single, clear purpose
;       Overlapping aliases have been reassigned to unused registers

	.asg	r0,             TEMP0                           ; r0: General purpose
	.asg	r1,             TEMP1                           ; r1: General purpose
	.asg	r2,             TEMP2                           ; r2: General purpose
	.asg	r3,             TEMP3                           ; r3: General purpose
	.asg	r4,             TEMP4                           ; r4: General purpose
	.asg	r5,             DMEM_OFFSET                     ; r5: Data memory offset
	.asg	r6,             RX_BUFFER_OFFSET                ; r6: RX buffer offset
	.asg	r7.b0,          LONG_SYMBOL_COUNT               ; r7: Long symbol count and protocol state
	.asg	r7.b1,          FIRST_DATA_HALF_BIT
	.asg	r7.b2,          LONG_SHORT_STATUS
	.asg	r8,             PREAMBLE_START_REG              ; r8: Preamble start register
	.asg	r9,             PREAMBLE_BASE                   ; r9: Preamble base
	.asg	r10,            DEC_OFFSET_REG                  ; r10: Decode offset register
	.asg	r11,            ENCODED_DATA_LOW                ; r11: Encoded data low
	.asg	r12,            OFFLOAD_REG                     ; r12: Offload register
	.asg	r13,            ENCODE_INPUT_REG                ; r13: Encode input register
	.asg	r14,            DECODED_DATA_REG                ; r14: Decoded data register
	.asg	r15,            ADD_DELAY                       ; r15: Additional delay
	.asg	r16,            ENCODE_OUTPUT_REG               ; r16: Encode output register
	.asg	r17,            SAMPLE_OFFSET_COUNTER           ; r17: Sample offset counter
	.asg	r18.b0,         OVS_DATA_REG                    ; r18: Oversampling data register
	.asg	r19,            ENCODED_DATA_HIGH               ; r19: Encoded data high
	.asg	r21.b0,         LUT_SAMPLE_REG                  ; r21: LUT sample and temp
	.asg	r21.b2,         TEMP_REG
	.asg	r22,            TEMP_REG1                       ; r22: Temporary register 1
	.asg	r23,            BIT_CAPTURE_REG                 ; r23: Bit capture register
	.asg	r24.b0,         ERROR_MASK_REG                  ; r24: Error handling
	.asg	r24.b1,         ERROR_STATUS_REG
	.asg	r27,            BYTE_REV_RESULT                 ; r27: Byte reverse result
	.asg	r28,            BYTE_REV_INPUT                  ; r28: Byte reverse input
	.asg	r29.b0,         TX_FRAMES_LEFT                  ; r29: TX frame control
	.asg	r29.b1,         CURR_TX_FRAME_MEM_OFFSET

;******************************************************************************
; Shared DRAM Memory Offsets (Absolute Addresses)
;******************************************************************************
PREAMBLE_START                  .set    0x10000
PREAMBLE_DEC                    .set    0x10020
ENDAT3_DEC_OFFS                 .set    0x1208C
OFFLOAD_DATA_OFFS               .set    0x1508C

;******************************************************************************
; Command and Status Constants
;******************************************************************************
WRITE_BG_OPCODE                 .set    0x3
RESET_CMD                       .set    0xB
SAMPLING_ERROR_FLAG             .set    0x2

;******************************************************************************
; Protocol Constants
;******************************************************************************
FIXED_TX_PREAMBLE               .set    0x1999B29
FIXED_HELLO_CMD_DATA            .set    0x22222269
RX_FIXED_PREAMBLE_HIGH          .set    0x2
RX_FIXED_PREAMBLE_LOW           .set    0xC9
ERROR_MASK                      .set    0x80
POSTAMBLE_PATTERN               .set    0xFD
MIN_LONG_SYMB_COUNT             .set    7
;******************************************************************************
; Periodic Trigger Configuration Constants
;******************************************************************************
IEP_CMP3_EVENT_FLAG             .set    3       ; Bit 3 for CMP3 event
IEP_CMP0_EVENT_FLAG             .set    0       ; Bit 0 for CMP0 event
PRU_TRIGGER_HOST_ENDAT3_EVT0    .set    18      ; (2+16) - Interrupt event
; Note: IEP register offsets are defined in icss_iep_regs.inc:
; ICSS_IEP_CMP_STATUS_REG = 0x0074
; ICSS_IEP_CMP_CFG_REG = 0x0070
; ICSS_IEP_CMP0_REG = 0x0078
; ICSS_IEP_CMP3_REG = 0x0090

;******************************************************************************