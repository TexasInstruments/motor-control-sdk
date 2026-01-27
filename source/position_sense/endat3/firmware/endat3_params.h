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
; DESIGN PRINCIPLE: Each register name maps to ONE unique physical register
; No overlapping aliases - clear 1-to-1 mapping for all registers
; Registers are listed in ascending order by physical register number
;******************************************************************************

;------------------------------------------------------------------------------
; r0-r4: TX Pipeline Registers - Descriptive Naming
;------------------------------------------------------------------------------
	.asg	r0,             TEMP0                           ; r0: General purpose temporary register 0
	.asg	r1,             TX_CMD_DATA                     ; r1: TX command data input
	.asg	r2,             TX_PREAMBLE                     ; r2: TX preamble storage
	.asg	r3,             TX_ENCODED_HIGH                 ; r3: TX encoded data high 32 bits
	.asg	r4,             TX_ENCODED_LOW                  ; r4: TX encoded data low 32 bits

;------------------------------------------------------------------------------
; r5-r6: Memory and Data Path Registers
;------------------------------------------------------------------------------
	.asg	r5,             DMEM_OFFSET                     ; r5: Data memory base pointer
	.asg	r6,             RX_BUFFER_OFFSET                ; r6: RX buffer write offset

;------------------------------------------------------------------------------
; r7: Protocol State Registers (byte-packed)
;------------------------------------------------------------------------------
	.asg	r7.b0,          LONG_SYMBOL_COUNT               ; r7.b0: Long symbol counter
	.asg	r7.b1,          FIRST_DATA_HALF_BIT             ; r7.b1: First half-bit value
	.asg	r7.b2,          LONG_SHORT_STATUS               ; r7.b2: Symbol length indicator

;------------------------------------------------------------------------------
; r8-r10: Available for future use
;------------------------------------------------------------------------------

;------------------------------------------------------------------------------
; r11: TX Encoding Input Register
;------------------------------------------------------------------------------
	.asg	r11,            ENCODE_INPUT_REG                ; r11: TX encode 8-bit input

	.asg	r12,            DMEM_BASE                		; r12: Base address in DMEM

	;------------------------------------------------------------------------------
; r13: TX Encoding Output Register
;------------------------------------------------------------------------------
	.asg	r13,            ENCODE_OUTPUT_REG               ; r13: TX encode 16-bit output

;------------------------------------------------------------------------------
; r14-r15: RX and Timing Registers
;------------------------------------------------------------------------------
	.asg	r14,            DECODED_DATA_REG                ; r14: RX decoded data destination
	.asg	r15,            ADD_DELAY                       ; r15: Delay counter for timing loops

;------------------------------------------------------------------------------
; r16: Channel Enable Mask Register
;------------------------------------------------------------------------------
	.asg	r16.b0,         ENDAT_ENABLE_CHx                ; r16.b0: Channel enable mask (bit 0=CH0, bit 1=CH1, bit 2=CH2)

;------------------------------------------------------------------------------
; r17-r18: RX Sampling Registers
;------------------------------------------------------------------------------
	.asg	r17,            SAMPLE_OFFSET_COUNTER           ; r17: RX sample debug offset counter
	.asg	r18.b0,         OVS_DATA_REG                    ; r18.b0: RX oversampling data storage

;------------------------------------------------------------------------------
; r19-r20: General Purpose Temporary Registers
;------------------------------------------------------------------------------
	.asg	r19,            TEMP1                           ; r19: General purpose temporary register 1
	.asg	r20,            TEMP2                           ; r20: General purpose temporary register 2

;------------------------------------------------------------------------------
; r21-r24: RX Sampling, LUT, and Error Detection Registers
;------------------------------------------------------------------------------
	.asg	r21.b0,         LUT_SAMPLE_REG                  ; r21.b0: LUT lookup result
	.asg	r21.b2,         TEMP_REG                        ; r21.b2: Temporary byte storage
	.asg	r22,            TEMP_REG1                       ; r22: General temporary storage
	.asg	r23,            BIT_CAPTURE_REG                 ; r23: RX bit accumulation buffer
	.asg	r24.b0,         ERROR_MASK_REG                  ; r24.b0: Error detection mask
	.asg	r24.b1,         ERROR_STATUS_REG                ; r24.b1: Error status flag

;------------------------------------------------------------------------------
; r25-r26: Unused - Available for future use
;------------------------------------------------------------------------------

;------------------------------------------------------------------------------
; r27-r28: Byte Reversal Registers (TX encoding operations)
;------------------------------------------------------------------------------
	.asg	r27,            BYTE_REV_RESULT                 ; r27: Byte reversal output
	.asg	r28,            BYTE_REV_INPUT                  ; r28: Byte reversal input

;------------------------------------------------------------------------------
; r29: TX Frame Management Registers (byte-packed)
;------------------------------------------------------------------------------
	.asg	r29.b0,         TX_FRAMES_LEFT                  ; r29.b0: TX frame counter
	.asg	r29.b1,         CURR_TX_FRAME_MEM_OFFSET        ; r29.b1: TX frame memory offset

;******************************************************************************
; Free Register Summary - Available for Future Development
;******************************************************************************
; This section documents all unused register space for easy reference when
; adding new features or optimizations to the EnDAT3 firmware.
;
; FREE BYTE FIELDS (partial register usage):
;   r7.b3                  : 1 byte   - Protocol state register
;   r16.b1, r16.b2, r16.b3 : 3 bytes  - Channel enable register
;   r18.b1, r18.b2, r18.b3 : 3 bytes  - RX sampling register
;   r21.b1, r21.b3         : 2 bytes  - LUT/temp register
;   r24.b2, r24.b3         : 2 bytes  - Error detection register
;   r29.b2, r29.b3         : 2 bytes  - TX frame management register
;
; FREE FULL REGISTERS:
;   r8, r9, r10            : 12 bytes - Available for future use
;   r12                    : 4 bytes  - Available for future use
;   r25, r26               : 8 bytes  - Available for future use
;
; TOTAL FREE SPACE: 37 bytes
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
	.if	$isdefed("SLICE1")
PRU_TRIGGER_HOST_ENDAT3_EVT		.set	34			;( (0x20 | 2), pr0_pru_mst_intr[2]_intr_req )
	.else
	; "SLICE0"
PRU_TRIGGER_HOST_ENDAT3_EVT		.set	37			;( (0x20 | 5), pr0_pru_mst_intr[5]_intr_req )
	.endif
; Note: IEP register offsets are defined in icss_iep_regs.inc:
;******************************************************************************