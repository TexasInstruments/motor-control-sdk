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
;*   File:     endat3_macros.inc                                            		*
;*                                                                                  *
;*   Brief:    EnDat3 Three Channel Peripheral Interface Macros			*
;
; This file contains macros to implement the EnDat3 protocol for industrial 
; position sensing applications. The macros provide functionality for data 
; encoding/decoding, transmission, reception, error handling, and channel 
; management for multi-channel configurations.
;
; Register Offset Handling:
; - Uses labels from icss_cfg_regs.inc for ICSS configuration registers
; - Loads register offsets into TEMP1 register before using in sbco instructions
; - Supports dynamic PRU selection via compilation flags (PRU0/PRU1/RTU/TX variants)
; - Offsets are computed at assembly time and loaded at runtime
;
; Macro Categories:
; - Encoding: Manchester encoding, byte reversal, data preprocessing
; - Transmission: TX frame setup, clock configuration, FIFO management
; - Reception: RX frame setup, preamble detection, Manchester decoding
; - Channel Control: TX/RX enable, channel selection, clock mode configuration
;************************************************************************************
	.include "endat3_icss_reg_defs.h"
	.include "endat3_params.h"
	.include "firmware_version.h"
;******************************************************************************
; Byte Reversal and Encoding Macros
;
; These macros implement the EnDat3 data encoding scheme, which uses Manchester
; encoding for robust communication in industrial environments.
;******************************************************************************

;------------------------------------------------------------------------------
; REVERSE_BYTE
; 
; Reverses the bit order in a byte using PRU's hardware acceleration.
; Used to convert between little-endian and big-endian formats.
;
; Parameters:
;   data - Register containing the byte to reverse
;
; Returns:
;   data - The same register now containing the bit-reversed byte
;------------------------------------------------------------------------------
REVERSE_BYTE .macro data
	mov    BYTE_REV_INPUT, data         ; Copy input data to transfer register
    xout   CRC_XID,&BYTE_REV_INPUT,4          ; Send to hardware bit reversal unit
    xin    CRC_XID,&BYTE_REV_RESULT,4         ; Receive the reversed result
    mov 	data, BYTE_REV_RESULT        ; Store result back to input register
    .endm

;------------------------------------------------------------------------------
; FN_MANCHESTER_ENCODE_8BIT
; 
; Encodes an 8-bit value into a 16-bit Manchester-encoded pattern.
; Manchester encoding represents '0' as '01' and '1' as '10'.
;
; Parameters:
;   data - Register containing 8-bit data to encode
;   temp - Register to receive 16-bit encoded result
;
; Returns:
;   temp - The register now containing the Manchester-encoded data
;------------------------------------------------------------------------------
FN_MANCHESTER_ENCODE_8BIT .macro data,temp
    ; Input: data: 8-bit data to encode
    ; Output:temp: 16-bit Manchester encoded data
    zero    &temp, 4                   ; Clear output register

    ; Encode each bit
    loop    MANCH_8BIT_LOOP?, 8        ; Process 8 bits sequentially
    lsl     temp, temp, 2              ; Make room for next encoded bit pair
    qbbc    ENCODE_8BIT_ZERO?, data, 7 ; Check if current bit is 0 (MSB first)

    ; Encode '1' as '10'
    or      temp, temp, 2              ; Set pattern '10' for bit value '1'
    jmp     NEXT_8BIT?

ENCODE_8BIT_ZERO?:
    ; Encode '0' as '01'
    or      temp, temp, 1              ; Set pattern '01' for bit value '0'

NEXT_8BIT?:
    lsl     data, data, 1              ; Shift to next input bit
MANCH_8BIT_LOOP?:
    .endm

;------------------------------------------------------------------------------
; ENCODE_TX_DATA
; 
; Encodes a full 32-bit data word for transmission using Manchester encoding.
; Processes the data in 8-bit chunks, most significant byte first.
;
; Note: Takes data from TEMP1 and stores encoded result in ENCODED_DATA_HIGH
; and ENCODED_DATA_LOW registers.
;
; Fixed Register Usage:
;   Input:  TEMP1 - 32-bit data to encode
;   Output: ENCODED_DATA_HIGH - High 32 bits of encoded data
;           ENCODED_DATA_LOW - Low 32 bits of encoded data
;   Temp:   ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
;------------------------------------------------------------------------------
ENCODE_TX_DATA .macro
	REVERSE_BYTE TEMP1                  ; Reverse the bit order of input data
    
    ; Process first byte (MSB)
	mov ENCODE_INPUT_REG.b0, TEMP1.b3   ; Extract most significant byte
	FN_MANCHESTER_ENCODE_8BIT ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
	mov ENCODED_DATA_HIGH.w2, ENCODE_OUTPUT_REG.w0 ; Store encoded result
	
    ; Process second byte
	mov ENCODE_INPUT_REG.b0, TEMP1.b2   ; Extract second byte
	FN_MANCHESTER_ENCODE_8BIT ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
	mov ENCODED_DATA_HIGH.w0, ENCODE_OUTPUT_REG.w0 ; Store encoded result
	
    ; Process third byte
	mov ENCODE_INPUT_REG.b0, TEMP1.b1   ; Extract third byte
	FN_MANCHESTER_ENCODE_8BIT ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
	mov ENCODED_DATA_LOW.w2, ENCODE_OUTPUT_REG.w0  ; Store encoded result
	
    ; Process fourth byte (LSB)
	mov ENCODE_INPUT_REG.b0, TEMP1.b0   ; Extract least significant byte
	FN_MANCHESTER_ENCODE_8BIT ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
	mov ENCODED_DATA_LOW.w0, ENCODE_OUTPUT_REG.w0  ; Store encoded result
    .endm

;------------------------------------------------------------------------------
; TX_PREPROCESSING
; 
; Prepares encoded data for transmission by:
; 1. Shifting data to create space for the postamble
; 2. Aligning data across registers
; 3. Adding the postamble pattern
;
; Fixed Register Usage:
;   Input:  TEMP2 - Preamble data
;           TEMP3 - High 32 bits of encoded data
;           TEMP4 - Low 32 bits of encoded data
;   Temp:   TEMP0 - Temporary register for calculations
;   Output: TEMP2, TEMP3, TEMP4 - Preprocessed data ready for transmission
;------------------------------------------------------------------------------
TX_PREPROCESSING .macro
    ; Make 4 bits space for adding postamble at end
    ; Adjust TEMP2 by shift 4 positions left
	lsl TEMP2, TEMP2, 4                 ; Shift preamble data left by 4 bits
	and TEMP0.b0, TEMP3.b3, 0xf0        ; Extract high nibble from TEMP3 MSB
	lsr TEMP0.b0, TEMP0.b0, 4           ; Shift to low nibble position
	or TEMP2.b0, TEMP2.b0, TEMP0.b0     ; Combine with TEMP2

    ; Adjust TEMP3 by shift 4 positions left
	lsl TEMP3, TEMP3, 4                 ; Shift data left by 4 bits
	and TEMP0.b0, TEMP4.b3, 0xf0        ; Extract high nibble from TEMP4 MSB
	lsr TEMP0.b0, TEMP0.b0, 4           ; Shift to low nibble position
	or TEMP3.b0, TEMP3.b0, TEMP0.b0     ; Combine with TEMP3
    
    ; Adjust TEMP4 by shift 4 positions left and add postamble
	lsl TEMP4, TEMP4, 4                 ; Shift data left by 4 bits
	ldi TEMP0.b0, 0x9                   ; Postamble pattern (1001)
	or TEMP4.b0, TEMP4.b0, TEMP0.b0     ; Append postamble to data
    .endm

;------------------------------------------------------------------------------
; SEND_TX
; 
; Sends the prepared data to the EnDat3 interface FIFO for transmission.
; Handles data in the proper sequence and manages FIFO flow control.
;
; Fixed Register Usage:
;   Input: TEMP2 - Preamble data
;           TEMP3 - High 32 bits of data
;           TEMP4 - Low 32 bits of data with postamble
;
; Note: Sends data in the following order:
;   1. Preamble (from TEMP2)
;   2. Data (from TEMP3)
;   3. Data with postamble (from TEMP4)
;------------------------------------------------------------------------------
SEND_TX .macro
    ; Push data from TEMP2,TEMP3,TEMP4 registers to TX FIFO
    ; Send preamble first (TEMP2 MSB to LSB)
	PUSH_FIFO TEMP2.b3                 ; Push most significant byte
	PUSH_FIFO TEMP2.b2
	PUSH_FIFO TEMP2.b1
	TX_CHANNEL                         ; Trigger channel for transmission
	WAIT_TX_FIFO_FREE                  ; Ensure FIFO has space
	PUSH_FIFO TEMP2.b0                 ; Push least significant byte
	
    ; Send data middle section (TEMP3)
	WAIT_TX_FIFO_FREE                  ; Ensure FIFO has space
	PUSH_FIFO TEMP3.b3                 ; Push most significant byte
	PUSH_FIFO TEMP3.b2
	WAIT_TX_FIFO_FREE                  ; Ensure FIFO has space
	PUSH_FIFO TEMP3.b1
	PUSH_FIFO TEMP3.b0                 ; Push least significant byte
	
    ; Send data end section with postamble (TEMP4)
	WAIT_TX_FIFO_FREE                  ; Ensure FIFO has space
	PUSH_FIFO TEMP4.b3                 ; Push most significant byte
	PUSH_FIFO TEMP4.b2
	WAIT_TX_FIFO_FREE                  ; Ensure FIFO has space
	PUSH_FIFO TEMP4.b1
	PUSH_FIFO TEMP4.b0                 ; Push least significant byte (with postamble)
    .endm
;******************************************************************************
; Channel Enable/Disable Macros
;
; These macros configure and control the EnDat3 interface channels.
;******************************************************************************

;------------------------------------------------------------------------------
; TX_EN
; 
; Enables transmit mode for the configured EnDat3 channel(s).
; Clears the appropriate bits in R30 based on which channel is defined.
;------------------------------------------------------------------------------
TX_EN .macro
	.if $defined(ENABLE_MULTI_MAKE_TXPRU)
	clr		r30.b3, r30.b3, 2          ; Enable TX for channel 2
	.endif
	.if $defined(ENABLE_MULTI_MAKE_PRU)
	clr		r30.b3, r30.b3, 1          ; Enable TX for channel 1
	.endif
	.if $defined(ENABLE_MULTI_MAKE_RTU)
	clr		r30.b3, r30.b3, 0          ; Enable TX for channel 0
	.endif
	.endm

;------------------------------------------------------------------------------
; RX_EN
; 
; Enables receive mode for the configured EnDat3 channel(s).
; Sets the appropriate bits in R30 based on which channel is defined.
;------------------------------------------------------------------------------
RX_EN .macro
	.if $defined(ENABLE_MULTI_MAKE_TXPRU)
	set		r30.b3, r30.b3, 2          ; Enable RX for channel 2
	.endif
	.if $defined(ENABLE_MULTI_MAKE_PRU)
	set		r30.b3, r30.b3, 1          ; Enable RX for channel 1
	.endif
	.if $defined(ENABLE_MULTI_MAKE_RTU)
	set		r30.b3, r30.b3, 0          ; Enable RX for channel 0
	.endif
	.endm

;******************************************************************************
; FIFO Control and Frame Configuration Macros
;
; These macros handle the configuration and management of the EnDat3 interface
; FIFOs and frame parameters.
;******************************************************************************

;------------------------------------------------------------------------------
; PUSH_FIFO
; 
; Pushes a byte to the EnDat3 TX FIFO.
; Supports multi-core functionality with TX FIFO load on b0, b1, b2 for all cores.
;
; Parameters:
;   dat - The byte to push to the FIFO
;
; Multi-Core Behavior (Channel Order: 0, 1, 2):
;   - ENABLE_MULTI_MAKE_RTU: Uses r30.b0 for RTU (channel 0)
;   - ENABLE_MULTI_MAKE_PRU: Uses r30.b1 for PRU (channel 1)
;   - ENABLE_MULTI_MAKE_TXPRU: Uses r30.b2 for TX PRU (channel 2)
;------------------------------------------------------------------------------
PUSH_FIFO .macro dat
    .if $defined(ENABLE_MULTI_MAKE_RTU)
    mov     r30.b0, dat                ; RTU: Move data to TX FIFO register (channel 0)
    .endif
    .if $defined(ENABLE_MULTI_MAKE_PRU)
    mov     r30.b1, dat                ; PRU: Move data to TX FIFO register (channel 1)
    .endif
    .if $defined(ENABLE_MULTI_MAKE_TXPRU)
    mov     r30.b2, dat                ; TX PRU: Move data to TX FIFO register (channel 2)
    .endif
    .endm

;------------------------------------------------------------------------------
; WAIT
; 
; Creates a programmable delay using register arithmetic.
; Used for timing critical operations and protocol timing requirements.
;
; Parameters:
;   delay - Register containing delay count value (will be modified)
;------------------------------------------------------------------------------
WAIT .macro delay
    lsr delay, delay, 1                ; Divide delay by 2 for efficiency
l?: 
    sub delay, delay, 1                  ; Decrement counter
    qblt l?, delay, 0                    ; Loop until counter reaches zero
    .endm

;------------------------------------------------------------------------------
; RX_FRAME_SIZE
; 
; Configures the receive frame size for the EnDat3 interface.
; Sets maximum frame size for the appropriate channel.
;------------------------------------------------------------------------------
RX_FRAME_SIZE .macro
    ldi     TEMP0.w0, 0x0FFF           ; Set maximum frame size (4095 bits)
    
    ; Select the appropriate channel configuration register
    .if $defined(ENABLE_MULTI_MAKE_RTU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 0 config register + offset
    .endif
    .if $defined(ENABLE_MULTI_MAKE_PRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 1 config register + offset
    .endif
    .if $defined(ENABLE_MULTI_MAKE_TXPRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 2 config register + offset
    .endif
    .endm

;------------------------------------------------------------------------------
; TX_FRAME_SIZE
; 
; Configures the transmit frame size for the EnDat3 interface.
;
; Parameters:
;   bits - Number of bits to transmit
;   tmp0 - Temporary register for calculations
;------------------------------------------------------------------------------
TX_FRAME_SIZE .macro bits, tmp0
    ldi     tmp0, (bits<<11)           ; Prepare frame size value
    
    ; Select the appropriate channel register
    .if $defined(ENABLE_MULTI_MAKE_TXPRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 2
    .endif
    .if $defined(ENABLE_MULTI_MAKE_PRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 1
    .endif
    .if $defined(ENABLE_MULTI_MAKE_RTU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 0
    .endif
    .endm

;------------------------------------------------------------------------------
; TX_CLK_DIV
; 
; Sets the transmit clock divider for the EnDat3 interface.
; Controls the transmission bit rate.
;
; Parameters:
;   div - Clock divider value
;   tmp0 - Temporary register for calculations
;------------------------------------------------------------------------------
TX_CLK_DIV .macro div, tmp0
    ldi     tmp0.w2, div               ; Load divider value
    
    ; Select the appropriate PRU TXCFG register
    .if $defined(ENABLE_MULTI_MAKE_TXPRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_TXCFG+2   ; Load offset
    sbco    &tmp0.w2, ICSS_CFG, TEMP1.w0, 2  ; Configure clock divider for TX PRU
    .endif
    .if $defined(ENABLE_MULTI_MAKE_PRU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_TXCFG+2   ; Load offset
    sbco    &tmp0.w2, ICSS_CFG, TEMP1.w0, 2  ; Configure clock divider for PRU
    .endif
    .if $defined(ENABLE_MULTI_MAKE_RTU)
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_TXCFG+2   ; Load offset
    sbco    &tmp0.w2, ICSS_CFG, TEMP1.w0, 2  ; Configure clock divider for RTU
    .endif
    .endm

;******************************************************************************
; Status and Reset Macros
;
; These macros handle interface status flags and reset operations.
;******************************************************************************

;------------------------------------------------------------------------------
; CLEAR_VAL
; 
; Clears the RX valid flag to acknowledge receipt of data.
; This allows the hardware to present the next bit.
;------------------------------------------------------------------------------
CLEAR_VAL .macro
	set		r31, r31, RX_VALID_FLAG    ; Set bit to clear valid flag
	.endm

;------------------------------------------------------------------------------
; REINIT_TX
; 
; Reinitializes the TX hardware for the EnDat3 interface.
; Handles the reset sequence to ensure clean transmission.
;------------------------------------------------------------------------------
REINIT_TX .macro
	set		r31, r31, TX_GLOBAL_REINIT  ; Signal TX reset start
	TX_EN                               ; Enable TX mode
	set		r31, r31, TX_GLOBAL_REINIT  ; Re-signal TX reset
l?:
    loop    loop1?, 50                  ; Delay loop for reset to complete
    add     TEMP0, TEMP0, 0             ; NOP operation
loop1?:
    CLKMODE                             ; Set clock mode after reset
    .endm

;******************************************************************************
; FIFO Management and Synchronization Macros
;
; These macros handle FIFO status checking and synchronization operations.
;******************************************************************************

;------------------------------------------------------------------------------
; WAIT_TX_FIFO_FREE
; 
; Waits until the TX FIFO has space available for new data.
; Prevents FIFO overflow by checking status flags.
;------------------------------------------------------------------------------
WAIT_TX_FIFO_FREE .macro
l?:
    ; Check FIFO status based on active channel
	.if $defined(ENABLE_MULTI_MAKE_TXPRU)
	qbbs    l?, r31, TX_GLOBAL_TX_GO    ; Wait if transmission active
	qbbs    l?, r31, TX_GLOBAL_REINIT   ; Wait if reset in progress
	.endif
	.if $defined(ENABLE_MULTI_MAKE_PRU)
	qbbs    l?, r31, 12                 ; Wait if fill level >= 4
	qbbs    l?, r31, 11                 ; Wait if fill level >= 2
	.endif
	.if $defined(ENABLE_MULTI_MAKE_RTU)
	qbbs    l?, r31, 4                  ; Wait if fill level >= 4
	qbbs    l?, r31, 3                  ; Wait if fill level >= 2
	.endif
	.endm

;------------------------------------------------------------------------------
; WAIT_TX_DONE
; 
; Waits until TX transmission is fully completed.
; Ensures all data has been transmitted before proceeding.
;------------------------------------------------------------------------------
WAIT_TX_DONE .macro
l?:
	qbbs    l?, r31, TX_GLOBAL_REINIT_ACTIVE  ; Wait while TX is active
	.endm

;------------------------------------------------------------------------------
; SET_TX_CH
; 
; Configures the EnDat3 interface to use channel 0.
; Clears channel select bits and sets appropriate bit for non-channel 0 config.
;------------------------------------------------------------------------------
SET_TX_CH .macro
	and     r30.b2, r30.b2, 0xfc        ; Clear channel select bits
	.if !$defined(ENABLE_MULTI_MAKE_RTU)
	set     r30.b2, r30.b2, CHANNEL_NUM  ; Set bit for selected channel
	.endif
	.endm

;------------------------------------------------------------------------------
; CLKMODE
; 
; Sets the clock mode for the EnDat3 interface.
; Configures clock to appropriate mode for transmission/reception.
;------------------------------------------------------------------------------
CLKMODE .macro
    clr     r30.b2, r30.b2, 3           ; Clear clock mode bit 0
    set     r30.b2, r30.b2, 4           ; Set clock mode bit 1
    .endm

;------------------------------------------------------------------------------
; TX_CHANNEL
; 
; Triggers the TX channel to begin transmission.
;------------------------------------------------------------------------------
TX_CHANNEL .macro
	set     r31, r31, TX_CHANNEL_GO     ; Set channel go bit to start transmission
	.endm

;******************************************************************************
; Data Reception Macros
;
; These macros handle receiving and decoding data from the EnDat3 interface.
;******************************************************************************

;------------------------------------------------------------------------------
; ERROR_IN_SAMPLING
; 
; Handles error conditions during data sampling.
; Sets error status flag and optionally halts processing.
;------------------------------------------------------------------------------
ERROR_IN_SAMPLING .macro 
    ldi     ERROR_STATUS_REG, 0          ; Clear error status register to indicate error
    .endm

;------------------------------------------------------------------------------
; RECEIVE_PREAMBLE
; 
; Receives and processes preamble bits from the EnDat3 interface.
; Detects long and short symbols in the preamble pattern.
;
; Parameters:
;   loop_cnt - Number of bits to process
;------------------------------------------------------------------------------
RECEIVE_PREAMBLE .macro loop_cnt
   loop     bit_sample_done?,loop_cnt   ; Loop for specified number of bits
receive_preamble_bit?:
	qbbc    receive_preamble_bit?, r31, RX_VALID_FLAG  ; Wait for valid bit
    
    mov     OVS_DATA_REG, r31.b0         ; Save raw oversampled data
	lsl     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 2  ; Make room for new bits
    add     TEMP0, TEMP0, 0               ; NOP operation
    
    ; Use lookup table to decode bit pattern
    lbbo    &LUT_SAMPLE_REG, PREAMBLE_BASE, r31.b0, 1  ; LUT based sampling decision
    CLEAR_VAL                          ; Clear valid flag for next bit

    ; Store oversampled data for debugging
    sbco    &OVS_DATA_REG, PRUx_DMEM, SAMPLE_OFFSET_COUNTER, 1
	add     SAMPLE_OFFSET_COUNTER, SAMPLE_OFFSET_COUNTER, 1  ; Update counter
    
    ; Check for sampling error
    qbne    no_error_in_sampling?, LUT_SAMPLE_REG, 0xff 
    ERROR_IN_SAMPLING                   ; Handle error if detected
no_error_in_sampling?:
    
    ; Determine if this is a long or short symbol
    qbeq    long_symbol?, LUT_SAMPLE_REG, 0xC0  ; Check for long symbol pattern 1
    qbeq    long_symbol?, LUT_SAMPLE_REG, 0x0   ; Check for long symbol pattern 2
    ldi     LONG_SHORT_STATUS, 0         ; Set status to indicate short symbol
    qba     symbol_update_done?
long_symbol?:
    ldi     LONG_SHORT_STATUS, 1         ; Set status to indicate long symbol
symbol_update_done?:

    ; Extract decoded bit from LUT result and save to bit capture register
    lsr     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 6  ; Shift to get decoded bit
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, LUT_SAMPLE_REG  ; Store bit
bit_sample_done?:  
    .endm

;------------------------------------------------------------------------------
; RECEIVE_MANCHESTER_DATA
; 
; Receives and decodes Manchester-encoded data from the EnDat3 interface.
; Handles error detection and bit reconstruction from Manchester encoding.
;
; Parameters:
;   loop_cnt - Number of bytes to receive (each byte is 8 bits)
;------------------------------------------------------------------------------
RECEIVE_MANCHESTER_DATA .macro loop_cnt
   loop     bit_sample_done?,loop_cnt   ; Loop for specified number of bytes
receive_manchester_first_half_bit?:
	qbbc    receive_manchester_first_half_bit?, r31, RX_VALID_FLAG  ; Wait for valid bit
	
    lsr     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 1  ; Make room for next bit
    
    ; Get bit pattern from lookup table
    lbbo    &LUT_SAMPLE_REG, PREAMBLE_BASE, r31.b0, 1  ; LUT-based sampling decision
    CLEAR_VAL                          ; Clear valid flag for next bit
    
    ; Check for sampling error
    qbne    no_error_in_sampling1?, LUT_SAMPLE_REG, 0xff
    ERROR_IN_SAMPLING                   ; Handle error if detected
no_error_in_sampling1?:
    
    ; Check for Manchester encoding violations (same consecutive half-bits)
    xor     TEMP_REG, LUT_SAMPLE_REG, FIRST_DATA_HALF_BIT
    and     ERROR_STATUS_REG, ERROR_MASK_REG, TEMP_REG  ; ERROR_MASK_REG=0x80

    ; Reconstruct original data bit from Manchester encoding
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, FIRST_DATA_HALF_BIT

    ; Prepare for next bit
    lsl     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 1
    mov     FIRST_DATA_HALF_BIT, LUT_SAMPLE_REG  ; Save for next cycle
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RESET_FIFO_SETTING
; 
; Resets the EnDat3 interface FIFO settings to default values.
; Used during initialization and after error conditions.
;------------------------------------------------------------------------------
RESET_FIFO_SETTING .macro
    ; Reset channels once
	REINIT_TX                          ; Reinitialize TX hardware
    
    ; Initialize tx frame size as 0
	TX_FRAME_SIZE		0, TEMP0        ; Set frame size to 0
	.endm