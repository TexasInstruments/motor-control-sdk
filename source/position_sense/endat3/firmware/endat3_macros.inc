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
; Parameters:
;   input_data     - 32-bit register containing data to encode
;   output_high    - 32-bit register to store high 32 bits of encoded result
;   output_low     - 32-bit register to store low 32 bits of encoded result
;   work_reg1      - 32-bit work register for encoding operations
;   work_reg2      - 32-bit work register for encoding operations
;
; Example:
;   ENCODE_TX_DATA TX_CMD_DATA, TX_ENCODED_HIGH, TX_ENCODED_LOW, ENCODE_INPUT_REG, ENCODE_OUTPUT_REG
;------------------------------------------------------------------------------
ENCODE_TX_DATA .macro input_data, output_high, output_low, work_reg1, work_reg2
    ; Reverse byte order of input data (preserve original input_data)
    mov     work_reg1, input_data
    REVERSE_BYTE work_reg1

    ; Process first byte (MSB)
	mov     work_reg2.b0, work_reg1.b3
	FN_MANCHESTER_ENCODE_8BIT work_reg2, TEMP0
	mov     output_high.w2, TEMP0.w0

    ; Process second byte
	mov     work_reg2.b0, work_reg1.b2
	FN_MANCHESTER_ENCODE_8BIT work_reg2, TEMP0
	mov     output_high.w0, TEMP0.w0

    ; Process third byte
	mov     work_reg2.b0, work_reg1.b1
	FN_MANCHESTER_ENCODE_8BIT work_reg2, TEMP0
	mov     output_low.w2, TEMP0.w0

    ; Process fourth byte (LSB)
	mov     work_reg2.b0, work_reg1.b0
	FN_MANCHESTER_ENCODE_8BIT work_reg2, TEMP0
	mov     output_low.w0, TEMP0.w0
    .endm

;------------------------------------------------------------------------------
; TX_PREPROCESSING
;
; Prepares encoded data for transmission by:
; 1. Shifting data to create space for the postamble
; 2. Aligning data across registers
; 3. Adding the postamble pattern (0x9)
;
; Parameters:
;   preamble_reg   - 32-bit register containing preamble data (modified in-place)
;   data_high_reg  - 32-bit register containing high encoded data (modified in-place)
;   data_low_reg   - 32-bit register containing low encoded data (modified in-place)
;   work_reg       - 32-bit work register for temporary calculations
;
; Example:
;   TX_PREPROCESSING TX_PREAMBLE, TX_ENCODED_HIGH, TX_ENCODED_LOW, TEMP0
;------------------------------------------------------------------------------
TX_PREPROCESSING .macro preamble_reg, data_high_reg, data_low_reg, work_reg
    ; Make 4 bits space for adding postamble at end
    ; Adjust preamble register by shifting 4 positions left
	lsl     preamble_reg, preamble_reg, 4
	and     work_reg.b0, data_high_reg.b3, 0xf0
	lsr     work_reg.b0, work_reg.b0, 4
	or      preamble_reg.b0, preamble_reg.b0, work_reg.b0

    ; Adjust data_high register by shifting 4 positions left
	lsl     data_high_reg, data_high_reg, 4
	and     work_reg.b0, data_low_reg.b3, 0xf0
	lsr     work_reg.b0, work_reg.b0, 4
	or      data_high_reg.b0, data_high_reg.b0, work_reg.b0

    ; Adjust data_low register by shifting 4 positions left and add postamble
	lsl     data_low_reg, data_low_reg, 4
	ldi     work_reg.b0, 0x9
	or      data_low_reg.b0, data_low_reg.b0, work_reg.b0
    .endm

;------------------------------------------------------------------------------
; SEND_TX
;
; Sends the prepared data to the EnDat3 interface FIFO for transmission.
; Handles data in the proper sequence and manages FIFO flow control.
;
; Parameters:
;   preamble_reg   - 32-bit register containing preamble data
;   data_high_reg  - 32-bit register containing high 32 bits of data
;   data_low_reg   - 32-bit register containing low 32 bits of data with postamble
;
; Note: Sends data in the following order (MSB to LSB):
;   1. Preamble bytes (from preamble_reg)
;   2. High data bytes (from data_high_reg)
;   3. Low data bytes with postamble (from data_low_reg)
;
; Example:
;   SEND_TX TX_PREAMBLE, TX_ENCODED_HIGH, TX_ENCODED_LOW
;------------------------------------------------------------------------------
SEND_TX .macro preamble_reg, data_high_reg, data_low_reg
    ; Send preamble bytes (MSB to LSB)
	PUSH_FIFO preamble_reg.b3
	PUSH_FIFO preamble_reg.b2
	PUSH_FIFO preamble_reg.b1
	TX_CHANNEL
	WAIT_TX_FIFO_FREE
	PUSH_FIFO preamble_reg.b0

    ; Send high data bytes
	WAIT_TX_FIFO_FREE
	PUSH_FIFO data_high_reg.b3
	PUSH_FIFO data_high_reg.b2
	WAIT_TX_FIFO_FREE
	PUSH_FIFO data_high_reg.b1
	PUSH_FIFO data_high_reg.b0

    ; Send low data bytes with postamble
	WAIT_TX_FIFO_FREE
	PUSH_FIFO data_low_reg.b3
	PUSH_FIFO data_low_reg.b2
	WAIT_TX_FIFO_FREE
	PUSH_FIFO data_low_reg.b1
	PUSH_FIFO data_low_reg.b0
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
; Determines channel from runtime mask and clears the appropriate bit in R30.
;------------------------------------------------------------------------------
TX_EN .macro
	; Determine channel from mask at runtime
	qbbs	tx_ch0?, ENDAT_ENABLE_CHx, 0
	qbbs	tx_ch1?, ENDAT_ENABLE_CHx, 1
	qbbs	tx_ch2?, ENDAT_ENABLE_CHx, 2
	qba		tx_done?
tx_ch0?:
	clr		r30.b3, r30.b3, 0              ; Enable TX for channel 0
	qba		tx_done?
tx_ch1?:
	clr		r30.b3, r30.b3, 1              ; Enable TX for channel 1
	qba		tx_done?
tx_ch2?:
	clr		r30.b3, r30.b3, 2              ; Enable TX for channel 2
tx_done?:
	.endm

;------------------------------------------------------------------------------
; RX_EN
;
; Enables receive mode for the configured EnDat3 channel(s).
; Determines channel from runtime mask and sets the appropriate bit in R30.
;------------------------------------------------------------------------------
RX_EN .macro
	; Determine channel from mask at runtime
	qbbs	rx_ch0?, ENDAT_ENABLE_CHx, 0
	qbbs	rx_ch1?, ENDAT_ENABLE_CHx, 1
	qbbs	rx_ch2?, ENDAT_ENABLE_CHx, 2
	qba		rx_done?
rx_ch0?:
	set		r30.b3, r30.b3, 0              ; Enable RX for channel 0
	qba		rx_done?
rx_ch1?:
	set		r30.b3, r30.b3, 1              ; Enable RX for channel 1
	qba		rx_done?
rx_ch2?:
	set		r30.b3, r30.b3, 2              ; Enable RX for channel 2
rx_done?:
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
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    mov     r30.b0, dat                ; RTU: Move data to TX FIFO register (channel 0)
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    mov     r30.b1, dat                ; PRU: Move data to TX FIFO register (channel 1)
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    mov     r30.b2, dat                ; TX PRU: Move data to TX FIFO register (channel 2)
    .else
    mov     r30.b0, dat                ; Move data to TX FIFO for channel 0
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
; Sets maximum frame size for all channels.
;------------------------------------------------------------------------------
RX_FRAME_SIZE .macro
    ldi     TEMP0.w0, 0x0FFF           ; Set maximum frame size (4095 bits)

    ; Select the appropriate channel configuration register
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 0 config register + offset
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 1 config register + offset
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0 + 2   ; Load offset
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2   ; Channel 2 config register + offset
    .else
    ; Single channel mode: determine channel from mask at runtime
    qbbs    rx_frame_ch0?, ENDAT_ENABLE_CHx, 0
    qbbs    rx_frame_ch1?, ENDAT_ENABLE_CHx, 1
    qbbs    rx_frame_ch2?, ENDAT_ENABLE_CHx, 2
    qba     rx_frame_done?
rx_frame_ch0?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0 + 2
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2
    qba     rx_frame_done?
rx_frame_ch1?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0 + 2
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2
    qba     rx_frame_done?
rx_frame_ch2?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0 + 2
    sbco    &TEMP0, ICSS_CFG, TEMP1.w0, 2
rx_frame_done?:
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
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 2
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 1
    .elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0   ; Load offset
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2  ; Configure channel 0
    .else
    ; Single channel mode: determine channel from mask at runtime
    qbbs    tx_frame_ch0?, ENDAT_ENABLE_CHx, 0
    qbbs    tx_frame_ch1?, ENDAT_ENABLE_CHx, 1
    qbbs    tx_frame_ch2?, ENDAT_ENABLE_CHx, 2
    qba     tx_frame_done?
tx_frame_ch0?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2
    qba     tx_frame_done?
tx_frame_ch1?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2
    qba     tx_frame_done?
tx_frame_ch2?:
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0
    sbco    &tmp0, ICSS_CFG, TEMP1.w0, 2
tx_frame_done?:
    .endif
    .endm

;------------------------------------------------------------------------------
; TX_CLK_DIV
;
; Sets the transmit clock divider for the EnDat3 interface.
; Controls the transmission bit rate. TXCFG is a single register per PRU.
;
; Parameters:
;   div - Clock divider value
;   tmp0 - Temporary register for calculations
;------------------------------------------------------------------------------
TX_CLK_DIV .macro div, tmp0
    ldi     tmp0.w2, div               ; Load divider value
    ldi     TEMP1.w0, ICSS_CFG_PRUx_ENDAT3_TXCFG+2   ; Load offset
    sbco    &tmp0.w2, ICSS_CFG, TEMP1.w0, 2  ; Configure clock divider
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
	.if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	qbbs    l?, r31, TX_GLOBAL_TX_GO    ; Wait if transmission active
	qbbs    l?, r31, TX_GLOBAL_REINIT   ; Wait if reset in progress
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	qbbs    l?, r31, 12                 ; Wait if fill level >= 4
	qbbs    l?, r31, 11                 ; Wait if fill level >= 2
	.elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
	qbbs    l?, r31, 4                  ; Wait if fill level >= 4
	qbbs    l?, r31, 3                  ; Wait if fill level >= 2
	.else
	; Determine channel from mask at runtime
	qbbs    wait_fifo_ch2?, ENDAT_ENABLE_CHx, 2
	qbbs    wait_fifo_ch1?, ENDAT_ENABLE_CHx, 1
	; Default to channel 0
	qbbs    l?, r31, 4                  ; Wait if fill level >= 4
	qbbs    l?, r31, 3                  ; Wait if fill level >= 2
	qba     wait_fifo_done?
wait_fifo_ch1?:
	qbbs    l?, r31, 12                 ; Wait if fill level >= 4
	qbbs    l?, r31, 11                 ; Wait if fill level >= 2
	qba     wait_fifo_done?
wait_fifo_ch2?:
	qbbs    l?, r31, TX_GLOBAL_TX_GO    ; Wait if transmission active
	qbbs    l?, r31, TX_GLOBAL_REINIT   ; Wait if reset in progress
wait_fifo_done?:
	.endif
	.endm

;------------------------------------------------------------------------------
; WAIT_TX_DONE
;
; Waits until TX transmission is fully completed.
; Ensures all data has been transmitted before proceeding.
; Channel-aware: checks the appropriate TX_GLOBAL_REINIT_ACTIVE bit for active channel
;------------------------------------------------------------------------------
WAIT_TX_DONE .macro
    qbbs    wait_tx_ch0?, ENDAT_ENABLE_CHx, 0
    qbbs    wait_tx_ch1?, ENDAT_ENABLE_CHx, 1
    qbbs    wait_tx_ch2?, ENDAT_ENABLE_CHx, 2
    qba     wait_tx_done?
wait_tx_ch0?:
	qbbs    wait_tx_ch0?, r31, CH0_TX_GLOBAL_REINIT_ACTIVE
    qba     wait_tx_done?
wait_tx_ch1?:
	qbbs    wait_tx_ch1?, r31, CH1_TX_GLOBAL_REINIT_ACTIVE
    qba     wait_tx_done?
wait_tx_ch2?:
	qbbs    wait_tx_ch2?, r31, CH2_TX_GLOBAL_REINIT_ACTIVE
wait_tx_done?:
	.endm

;------------------------------------------------------------------------------
; SET_TX_CH
; 
; Configures the EnDat3 interface to use channel 0.
; Clears channel select bits and sets appropriate bit for non-channel 0 config.
;------------------------------------------------------------------------------
SET_TX_CH .macro
	and     r30.b2, r30.b2, 0xfc        ; Clear channel select bits
	.if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	set     r30.b2, r30.b2, 1           ; Set bit 1 for channel 2 (0b10)
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	set     r30.b2, r30.b2, 0           ; Set bit 0 for channel 1 (0b01)
	.elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
	; Channel 0: bits already cleared, nothing to set (0b00)
	.else
	qbbs    set_ch2?, ENDAT_ENABLE_CHx, 2
	qbbs    set_ch1?, ENDAT_ENABLE_CHx, 1
	; Default to channel 0 (bits already cleared)
	qba     ch_set_done?
set_ch1?:
	set     r30.b2, r30.b2, 0           ; Set bit 0 for channel 1 (0b01)
	qba     ch_set_done?
set_ch2?:
	set     r30.b2, r30.b2, 1           ; Set bit 1 for channel 2 (0b10)
ch_set_done?:
	.endif
	.endm

;------------------------------------------------------------------------------
; LOAD_CH_MASK
;
; Loads the channel enable mask from DMEM into ENDAT_ENABLE_CHx register.
;------------------------------------------------------------------------------
LOAD_CH_MASK .macro
    ldi     DMEM_OFFSET, ENDAT3_CHANNEL_ENABLE_MASK_OFFSET
    lbbo    &ENDAT_ENABLE_CHx, DMEM_BASE, DMEM_OFFSET, 1     ; Load channel enable mask into r16.b0
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
;------------------------------------------------------------------------------
; RECEIVE_PREAMBLE / RECEIVE_MANCHESTER_DATA - Wrapper macros
; Automatically selects the correct channel-specific macro based on ENDAT_ENABLE_CHx
;------------------------------------------------------------------------------
RECEIVE_PREAMBLE .macro loop_cnt
    qbbs    use_ch0_preamble?, ENDAT_ENABLE_CHx, 0
    qbbs    use_ch1_preamble?, ENDAT_ENABLE_CHx, 1
    qbbs    use_ch2_preamble?, ENDAT_ENABLE_CHx, 2
    qba     preamble_done?
use_ch0_preamble?:
    RECEIVE_PREAMBLE_CH0 loop_cnt
    qba     preamble_done?
use_ch1_preamble?:
    RECEIVE_PREAMBLE_CH1 loop_cnt
    qba     preamble_done?
use_ch2_preamble?:
    RECEIVE_PREAMBLE_CH2 loop_cnt
preamble_done?:
    .endm

RECEIVE_MANCHESTER_DATA .macro loop_cnt
    qbbs    use_ch0_manchester?, ENDAT_ENABLE_CHx, 0
    qbbs    use_ch1_manchester?, ENDAT_ENABLE_CHx, 1
    qbbs    use_ch2_manchester?, ENDAT_ENABLE_CHx, 2
    qba     manchester_done?
use_ch0_manchester?:
    RECEIVE_MANCHESTER_DATA_CH0 loop_cnt
    qba     manchester_done?
use_ch1_manchester?:
    RECEIVE_MANCHESTER_DATA_CH1 loop_cnt
    qba     manchester_done?
use_ch2_manchester?:
    RECEIVE_MANCHESTER_DATA_CH2 loop_cnt
manchester_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_PREAMBLE_CH0 - Channel 0 specific preamble reception
;------------------------------------------------------------------------------
RECEIVE_PREAMBLE_CH0 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_preamble_bit?:
	qbbc    receive_preamble_bit?, r31, CH0_RX_VALID_FLAG
	lsl     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 2
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b0, 1
    set		r31, r31, CH0_RX_VALID_FLAG

    qbne    no_error_in_sampling?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
    qba error_detected
no_error_in_sampling?:

    qbeq    long_symbol?, LUT_SAMPLE_REG, 0xC0
    qbeq    long_symbol?, LUT_SAMPLE_REG, 0x0
    ldi     LONG_SHORT_STATUS, 0
    qba     symbol_update_done?
long_symbol?:
    ldi     LONG_SHORT_STATUS, 1
symbol_update_done?:

    lsr     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 6
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, LUT_SAMPLE_REG
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_PREAMBLE_CH1 - Channel 1 specific preamble reception
;------------------------------------------------------------------------------
RECEIVE_PREAMBLE_CH1 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_preamble_bit?:
	qbbc    receive_preamble_bit?, r31, CH1_RX_VALID_FLAG
	lsl     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 2
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b1, 1
    set		r31, r31, CH1_RX_VALID_FLAG

    qbne    no_error_in_sampling?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
    qba error_detected
no_error_in_sampling?:

    qbeq    long_symbol?, LUT_SAMPLE_REG, 0xC0
    qbeq    long_symbol?, LUT_SAMPLE_REG, 0x0
    ldi     LONG_SHORT_STATUS, 0
    qba     symbol_update_done?
long_symbol?:
    ldi     LONG_SHORT_STATUS, 1
symbol_update_done?:

    lsr     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 6
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, LUT_SAMPLE_REG
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_PREAMBLE_CH2 - Channel 2 specific preamble reception
;------------------------------------------------------------------------------
RECEIVE_PREAMBLE_CH2 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_preamble_bit?:
	qbbc    receive_preamble_bit?, r31, CH2_RX_VALID_FLAG
	lsl     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 2
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b2, 1
    set		r31, r31, CH2_RX_VALID_FLAG

    qbne    no_error_in_sampling?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
    qba error_detected
no_error_in_sampling?:

    qbeq    long_symbol?, LUT_SAMPLE_REG, 0xC0
    qbeq    long_symbol?, LUT_SAMPLE_REG, 0x0
    ldi     LONG_SHORT_STATUS, 0
    qba     symbol_update_done?
long_symbol?:
    ldi     LONG_SHORT_STATUS, 1
symbol_update_done?:

    lsr     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 6
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, LUT_SAMPLE_REG
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_MANCHESTER_DATA_CH0 - Channel 0 specific Manchester data reception
;------------------------------------------------------------------------------
RECEIVE_MANCHESTER_DATA_CH0 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_manchester_first_half_bit?:
	qbbc    receive_manchester_first_half_bit?, r31, CH0_RX_VALID_FLAG
    lsr     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 1
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b0, 1
    set		r31, r31, CH0_RX_VALID_FLAG

    qbne    no_error_in_sampling1?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
no_error_in_sampling1?:

    xor     TEMP_REG, LUT_SAMPLE_REG, FIRST_DATA_HALF_BIT
    and     ERROR_STATUS_REG, ERROR_MASK_REG, TEMP_REG
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, FIRST_DATA_HALF_BIT
    lsl     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 1
    mov     FIRST_DATA_HALF_BIT, LUT_SAMPLE_REG
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_MANCHESTER_DATA_CH1 - Channel 1 specific Manchester data reception
;------------------------------------------------------------------------------
RECEIVE_MANCHESTER_DATA_CH1 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_manchester_first_half_bit?:
	qbbc    receive_manchester_first_half_bit?, r31, CH1_RX_VALID_FLAG
    lsr     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 1
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b1, 1
    set		r31, r31, CH1_RX_VALID_FLAG

    qbne    no_error_in_sampling1?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
no_error_in_sampling1?:

    xor     TEMP_REG, LUT_SAMPLE_REG, FIRST_DATA_HALF_BIT
    and     ERROR_STATUS_REG, ERROR_MASK_REG, TEMP_REG
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, FIRST_DATA_HALF_BIT
    lsl     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 1
    mov     FIRST_DATA_HALF_BIT, LUT_SAMPLE_REG
bit_sample_done?:
    .endm

;------------------------------------------------------------------------------
; RECEIVE_MANCHESTER_DATA_CH2 - Channel 2 specific Manchester data reception
;------------------------------------------------------------------------------
RECEIVE_MANCHESTER_DATA_CH2 .macro loop_cnt
   loop     bit_sample_done?,loop_cnt
receive_manchester_first_half_bit?:
	qbbc    receive_manchester_first_half_bit?, r31, CH2_RX_VALID_FLAG
    lsr     BIT_CAPTURE_REG, BIT_CAPTURE_REG, 1
    lbco    &LUT_SAMPLE_REG, PRUx_DMEM, r31.b2, 1
    set		r31, r31, CH2_RX_VALID_FLAG

    qbne    no_error_in_sampling1?, LUT_SAMPLE_REG, 0xff
    ldi     ERROR_STATUS_REG, 0
no_error_in_sampling1?:

    xor     TEMP_REG, LUT_SAMPLE_REG, FIRST_DATA_HALF_BIT
    and     ERROR_STATUS_REG, ERROR_MASK_REG, TEMP_REG
    or      BIT_CAPTURE_REG.b0, BIT_CAPTURE_REG.b0, FIRST_DATA_HALF_BIT
    lsl     LUT_SAMPLE_REG, LUT_SAMPLE_REG, 1
    mov     FIRST_DATA_HALF_BIT, LUT_SAMPLE_REG
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