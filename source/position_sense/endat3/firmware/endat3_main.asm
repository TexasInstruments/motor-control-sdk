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
;   File:     endat3_main.asm                                            		*
;                                                                                  *
;   Brief:    EnDat3 Protocol Implementation - Main Firmware Entry Point		*
;
; This firmware implements the EnDat3 protocol for position sensing applications.
; It handles bidirectional communication with EnDat3 encoders including:
; - Initialization and hello sequence
; - Transmitting commands to encoders
; - Receiving and decoding position data and status information
; - Error detection and handling
; - Periodic trigger support using IEP timer (CMP3 event)
;
; Operating Modes:
; - Periodic Mode (opmode=0): Automatic timer-driven reads via IEP CMP3 event
; - Host Mode (opmode=1): Traditional host-triggered reads
;
; Register Aliases:
; - Uses labels from icss_cfg_regs.inc for ICSS configuration registers
; - Dynamically selects PRU-specific register offsets based on compilation flags
; - Converts symbol values to constants for use in sbco instructions
;
; Compilation Flags:
; - PRU0/RTU_PRU0/TX_PRU0: Compiles for PRU0 with PRU0-specific register offsets
; - PRU1/RTU_PRU1/TX_PRU1: Compiles for PRU1 with PRU1-specific register offsets
;************************************************************************************

;******************************************************************************
	.include "endat3_icss_reg_defs.h"
	.include "endat3_params.h"
	.include "endat3_interface.h"
	.include "firmware_version.h"
	.include "endat3_macros.inc"
	.global endat3_init
	.retain
	.retainrefs

;******************************************************************************
; Periodic Trigger Configuration Constants
;******************************************************************************
ENDAT3_OPMODE_CONFIG_OFFSET         .set    0x0214  ; Operating mode offset
IEP_CMP3_EVENT_FLAG                 .set    3       ; Bit 3 for CMP3 event
IEP_CMP0_EVENT_FLAG                 .set    0       ; Bit 0 for CMP0 event
PRU_TRIGGER_HOST_ENDAT3_EVT0        .set    18      ; (2+16) - Interrupt event
; Note: IEP register offsets are defined in icss_iep_regs.inc:
; ICSS_IEP_CMP_STATUS_REG = 0x0074
; ICSS_IEP_CMP_CFG_REG = 0x0070
; ICSS_IEP_CMP0_REG = 0x0078
; ICSS_IEP_CMP3_REG = 0x0090

;******************************************************************************
; Main Entry Point
; 
; Sets up firmware release version and initializes the PRU for EnDat3 operation
;******************************************************************************
endat3_init:
    .word   ICSS_FIRMWARE_RELEASE_1, ICSS_FIRMWARE_RELEASE_2
endat_main:
    ; Clear all registers
    zero    &TEMP0, 120                  ; Clear R0-R29
    ; Initialize system
	.if	$isdefed("PRU0") | $isdefed("RTU_PRU0") | $isdefed("TX_PRU0")
	.asg	c24,	PRU0_DMEM
	.asg	c25,	PRU1_DMEM
	.asg	PRU0_DMEM,		PRUx_DMEM
	.asg	PRU0_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU0_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT3_CH0_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH1_CFG1, ICSS_CFG_PRUx_ENDAT3_CH1_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH2_CFG1, ICSS_CFG_PRUx_ENDAT3_CH2_CFG1
	.asg    ICSS_CFG_PRU0_ENDAT_CH0_CFG0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_CH1_CFG0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_CH2_CFG0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0
	.asg    ICSS_CFG_PRU0_ENDAT_TXCFG,    ICSS_CFG_PRUx_ENDAT3_TXCFG
	.asg    ICSS_CFG_PRU0_ENDAT_RXCFG,    ICSS_CFG_PRUx_ENDAT3_RXCFG
	.endif

	.if	$isdefed("PRU1") | $isdefed("RTU_PRU1") | $isdefed("TX_PRU1")
	.asg	c24,	PRU1_DMEM
	.asg	c25,	PRU0_DMEM
	.asg	PRU0_DMEM,		PRUx_DMEM
	.asg	PRU1_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU1_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT3_CH0_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH1_CFG1, ICSS_CFG_PRUx_ENDAT3_CH1_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH2_CFG1, ICSS_CFG_PRUx_ENDAT3_CH2_CFG1
	.asg    ICSS_CFG_PRU1_ENDAT_CH0_CFG0, ICSS_CFG_PRUx_ENDAT3_CH0_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_CH1_CFG0, ICSS_CFG_PRUx_ENDAT3_CH1_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_CH2_CFG0, ICSS_CFG_PRUx_ENDAT3_CH2_CFG0
	.asg    ICSS_CFG_PRU1_ENDAT_TXCFG,    ICSS_CFG_PRUx_ENDAT3_TXCFG
	.asg    ICSS_CFG_PRU1_ENDAT_RXCFG,    ICSS_CFG_PRUx_ENDAT3_RXCFG
	.endif
;******************************************************************************
; System Initialization
;
; Configures the EnDat3 interface by:
; - Enabling TX channel
; - Setting channel configuration
; - Initializing registers and memory
;******************************************************************************
fn_init_system:
	; Configure TX Channel
	TX_EN                                           ; Enable TX mode 
	SET_TX_CH                                      ; Select dedicated channel
	
	; Reset channel and initialize
	REINIT_TX                                       ; Reinitialize TX hardware
	TX_FRAME_SIZE		0, TEMP0                    ; Set TX frame size to 0
	TX_EN                                           ; Re-enable TX mode
	SET_TX_CH                                      ; Re-select dedicated channel 

;******************************************************************************
; Hello Sequence
;
; Sends initial "Hello" message to establish communication with the encoder.
; This is the first step in the EnDat3 handshake process.
;******************************************************************************
init_seq_hello:
	; Reset clock and prepare for transmission
	RESET_FIFO_SETTING                               ; Reset clock settings for TX
	; Load delay from interface structure (frequency-independent)
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_TX_START_1_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_tx_start_1 from R5F-calculated value
	WAIT ADD_DELAY
	
	; Prepare and send hello sequence
	ldi32 TEMP2,FIXED_TX_PREAMBLE                   ; Load predefined preamble pattern
	ldi32 TEMP1,FIXED_HELLO_CMD_DATA                ; Load hello command data
	ENCODE_TX_DATA                                  ; Manchester encode the data
	TX_PREPROCESSING                                ; Prepare data for transmission (add postamble, etc)
	SEND_TX                                         ; Transmit the data
	
	; Wait after hello transmission to pass Rx (no need to store rx for hello command used for encoder startup sequence)
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_TX_START_2_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_tx_start_2 from R5F-calculated value
	WAIT ADD_DELAY
wait_for_hello_command:

;******************************************************************************
; Unified Start Trigger Interface
;
; Waits for the R5F core to signal that a command is ready to send.
; Works for both host-triggered and periodic-triggered modes.
; The trigger is released by R5F core based on operating mode.
;******************************************************************************
	; Clear start trigger flag to indicate ready for command
	ldi TEMP2.b0,0
	ldi32 DMEM_OFFSET,START_TRIGGER_OFFSET
	sbbo &TEMP2.b0,DMEM_OFFSET,0,1                  ; Write 0 to start trigger flag

wait_for_start_trigger:
	; Wait for R5F core to set start trigger flag to 1
	ldi32 DMEM_OFFSET,START_TRIGGER_OFFSET
	lbbo &TEMP2.b0,DMEM_OFFSET,0,1                  ; Read start trigger flag
	qbne wait_for_start_trigger,TEMP2.b0,1          ; Loop until flag == 1
	
	; Reset system for new transmission
	ZERO    &TEMP0, 116                             ; Clear registers
	RESET_FIFO_SETTING

	; Send confirmation message
	ldi32 TEMP2,FIXED_TX_PREAMBLE                   ; Load preamble
	ldi32 TEMP1,FIXED_HELLO_CMD_DATA                ; Load data
	ENCODE_TX_DATA                                  ; Manchester encode the data
	TX_PREPROCESSING                                ; Prepare data for transmission (add postamble, etc)
	SEND_TX                                         ; Transmit the data

	; Wait after transmission to pass Rx as we dont need to store rx for HELLO command for encoder wakeup
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_TX_START_2_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_tx_start_2 from R5F-calculated value
	WAIT ADD_DELAY

	; Clear start trigger flag for next command
	ldi TEMP2.b0,0
	ldi32 DMEM_OFFSET,START_TRIGGER_OFFSET
	sbbo &TEMP2.b0,DMEM_OFFSET,0,1

;******************************************************************************
; Operating Mode Check
;
; Determines whether to use periodic trigger mode or host trigger mode
; based on ENDAT3_OPMODE_CONFIG_OFFSET in DMEM
;******************************************************************************
check_operating_mode:
	; Read operating mode from DMEM
	ldi32 DMEM_OFFSET, ENDAT3_OPMODE_CONFIG_OFFSET
	lbbo &TEMP2.b0, DMEM_OFFSET, 0, 1              ; Read opmode (0=periodic, 1=host)
	qbne wait_for_host_trigger1, TEMP2.b0, 0       ; If opmode != 0, use host trigger

;******************************************************************************
; Periodic Trigger Mode
;
; Waits for IEP CMP3 event to trigger encoder reads automatically.
; Generates interrupts to host after each read.
; Checks for operating mode changes to allow switching back to host mode.
;******************************************************************************
handle_periodic_trigger_mode:
	; Check if operating mode has changed to host mode
	ldi32 DMEM_OFFSET, ENDAT3_OPMODE_CONFIG_OFFSET
	lbbo &TEMP2.b0, DMEM_OFFSET, 0, 1              ; Read opmode (0=periodic, 1=host)
	qbne check_operating_mode, TEMP2.b0, 0         ; If opmode changed to 1, exit periodic mode
	
	; Get compare event status from IEP (offset 0x0074)
	lbco &TEMP0, ICSS_IEP, ICSS_IEP_CMP_STATUS_REG, 4
	
	; Check if CMP3 event is set (bit 3)
	qbbc handle_periodic_trigger_mode, TEMP0, IEP_CMP3_EVENT_FLAG
	
	; Clear CMP3 event flag by writing 1 to bit 3
	set TEMP0, TEMP0, IEP_CMP3_EVENT_FLAG
	sbco &TEMP0, ICSS_IEP, ICSS_IEP_CMP_STATUS_REG, 4
	
	; Jump directly to process_command (skip host trigger wait)
	qba process_command_periodic

;******************************************************************************
; Command Processing
;
; Processes commands received from the host. Reads command parameters,
; prepares transmission frames, and sends commands to the encoder.
;******************************************************************************
new_request_start:
	; Check operating mode before waiting for trigger
	ldi32 DMEM_OFFSET, ENDAT3_OPMODE_CONFIG_OFFSET
	lbbo &TEMP2.b0, DMEM_OFFSET, 0, 1              ; Read opmode (0=periodic, 1=host)
	qbeq process_command, TEMP2.b0, 0               ; If periodic mode, skip host trigger wait

wait_for_host_trigger1:
	; Wait for new command trigger (host mode only)
	ldi32 DMEM_OFFSET,HOST_TRIGGER_STATUS_FLAG
	lbbo &TEMP2.b0,DMEM_OFFSET,0,1
	qbne wait_for_host_trigger1,TEMP2.b0,1          ; Loop until flag == 1

process_command:
	; Clear status flag for host mode
	ldi TEMP2.b0,0
	ldi32 DMEM_OFFSET,HOST_TRIGGER_STATUS_FLAG
	sbbo &TEMP2.b0,DMEM_OFFSET,0,1

process_command_periodic:
	; Get number of frames to transmit
	ldi32 DMEM_OFFSET,TX_FRAMES_EXPECTED_OFFSET     ; Get frame count address
	lbbo &TX_FRAMES_LEFT,DMEM_OFFSET,0,1            ; Load frame count
	ldi CURR_TX_FRAME_MEM_OFFSET,0                  ; Reset buffer offset

;******************************************************************************
; Data Transmission
;
; Sends encoded data frames to the encoder. Handles timing requirements
; and prepares data for transmission with proper encoding and framing.
;******************************************************************************
send_encoded_tx_pattern:
	; Wait between next transmissions to follow protocol timings
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_TX_START_3_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_tx_start_3 from R5F-calculated value
	WAIT ADD_DELAY
	
data_id_x:
	; Check if special timing is needed
	ldi32 DMEM_OFFSET,BG_OPCODE_OFFSET
	lbbo &TEMP1,DMEM_OFFSET,0,1                     ; Read opcode
	qbne no_wait_for_10ms,TEMP1.b0,WRITE_BG_OPCODE  ; Skip if not write opcode
	qbne no_wait_for_10ms,TX_FRAMES_LEFT,1          ; Skip if not last frame
	
wait_10ms:
	; Special timing delay (10ms) - load from interface structure
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_10MS_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_10ms from R5F-calculated value
	WAIT ADD_DELAY
	
no_wait_for_10ms:
	; Prepare for data transmission
	ZERO    &TEMP0, 116                             ; Clear registers
	RESET_FIFO_SETTING
	; Prepare and send data frame
	ldi32 TEMP2,FIXED_TX_PREAMBLE                   ; Load preamble pattern
	ldi32 DMEM_OFFSET,DMEM_BASE_TX_BUFFER_OFFSET    ; Get buffer address
	lbbo &TEMP1,DMEM_OFFSET,CURR_TX_FRAME_MEM_OFFSET,4 ; Load data from memory
	add CURR_TX_FRAME_MEM_OFFSET,CURR_TX_FRAME_MEM_OFFSET,4 ; Update offset
	
	; Encode and send the data
	ENCODE_TX_DATA                                  ; Manchester encode the data
	TX_PREPROCESSING                                ; Prepare data for transmission (add postamble, etc)
	SEND_TX                                         ; Transmit the data
;******************************************************************************
; Main Processing Loop
;
; Handles frame counting and transitions to receiving mode when all frames
; have been transmitted.
;******************************************************************************
rx_main_loop:
	; Check if this is the last frame to transmit
	qbeq one_tx_frame_left, TX_FRAMES_LEFT,1        ; If last frame, prepare for rx
	sub TX_FRAMES_LEFT,TX_FRAMES_LEFT,1             ; Decrement frame counter
	qba send_encoded_tx_pattern                     ; Continue sending frames

;******************************************************************************
; Receive Mode Setup
;
; Prepares the system to receive data from the encoder after transmitting
; the command. Sets up receive registers and enables rx mode.
;******************************************************************************
one_tx_frame_left:
    ; Clear registers and prepare for reception
    zero			&TEMP0, 116                     ; Clear registers
    sbco &TEMP0,c25,TEMP0,120                       ; Clear memory area
    
    ; Setup lookup tables and decoding resources
    ldi32   PREAMBLE_START_REG,(PREAMBLE_START)     ; Load preamble start address
	ldi32   PREAMBLE_BASE,(PREAMBLE_DEC)            ; Load preamble decode table address
	ldi32   DEC_OFFSET_REG,(ENDAT3_DEC_OFFS)        ; Load decoder offset
    ldi32   OFFLOAD_REG,(OFFLOAD_DATA_OFFS)         ; Load offload address
    ldi32	DECODED_DATA_REG,DECODED_DATA_OFFSET    ; Set decoded data destination
	
	; Wait for TX to complete before enabling rx
	WAIT_TX_DONE                                    ; Wait until TX is completely done

	; Small delay before enabling rx depending upon encoder types
	; Load sampling delay from interface structure (frequency-independent)
	ldi32 DMEM_OFFSET,ENDAT3_DELAY_SAMPLING_OFFSET
	lbbo &ADD_DELAY,DMEM_OFFSET,0,4                  ; Load delay_sampling from R5F-calculated value
	loop sampling_delay,ADD_DELAY  ; Use dynamic sampling delay
	add TEMP0,TEMP0,0
sampling_delay:

	; Configure and enable rx mode
	RX_FRAME_SIZE                                   ; Configure rx frame size
	RX_EN                                           ; Enable rx hardware

;******************************************************************************
; Preamble Detection
;
; Detects and validates the preamble pattern from the encoder response.
; Ensures proper synchronization before proceeding to data reception.
;******************************************************************************
;Check for preamble (delimiter)
    ldi ERROR_MASK_REG,ERROR_MASK                   ; Set error detection mask
    ldi ERROR_STATUS_REG,1                          ; Initialize error status
	ldi first_data_half_bit,0                       ; Clear first bit storage
    
    ; Receive first bit of preamble
    RECEIVE_PREAMBLE 1                              ; Get first preamble bit
    ;;Now first_data_half_bit contain start of preamble (first half bit)
    
start_bits:
    ; Continue receiving preamble bits
    RECEIVE_PREAMBLE 1                              ; Get next bit
	qbeq long_symbol_detected,long_short_status,1   ; Check if it's a long symbol

short_symbol_detected:
    ; Process short symbol (pattern recognition)
    qbgt error_detected, long_symbol_count,MIN_LONG_SYMB_COUNT ; Check long symbol count
    RECEIVE_PREAMBLE 5                              ; Get 5 more bits
	
	; Mask and analyze the bit pattern
    and TEMP_REG1.b0,BIT_CAPTURE_REG.b0,0xff        ; Mask lower bits
    and TEMP_REG1.b1,BIT_CAPTURE_REG.b1,0xf         ; Mask upper bits
    lsr TEMP_REG1,TEMP_REG1,1                       ; Right shift
	
	; Check for errors and validate pattern
    qbeq error_detected,ERROR_STATUS_REG,0          ; Error check
    qbne error_detected, TEMP_REG1.b0,RX_FIXED_PREAMBLE_LOW  ; Pattern check low
    qbne error_detected, TEMP_REG1.b1,RX_FIXED_PREAMBLE_HIGH ; Pattern check high
	
	; Preamble is valid, prepare for data reception
    and first_data_half_bit,BIT_CAPTURE_REG.b0,0x1  ; Save first data half bit
    lsl first_data_half_bit,first_data_half_bit,7   ; Shift to position
	ldi BIT_CAPTURE_REG,0                           ; Clear bit capture register
	ldi32 TEMP1,0                                   ; Clear TEMP1
    qba receive_data_frames                         ; Start receiving data
    
long_symbol_detected:
    ; Track long symbols (part of preamble detection)
    add long_symbol_count,long_symbol_count,1       ; Increment long symbol counter
    qba start_bits                                  ; Continue preamble detection

;******************************************************************************
; Data Reception and Decoding
;
; Receives Manchester-encoded data bits from the encoder and stores
; the decoded values in memory. Continues until postamble or error is detected.
;******************************************************************************
;Receive of DATA frames here
receive_data_frames:
    ; Receive and decode 8 bits of Manchester data
    RECEIVE_MANCHESTER_DATA 8                       ; Get 8 bits (16 half-bits)
    
    ; Store the decoded byte
    sbbo &BIT_CAPTURE_REG.b0,DECODED_DATA_REG, RX_BUFFER_OFFSET,1 ; Save byte
    add RX_BUFFER_OFFSET,RX_BUFFER_OFFSET,1         ; Update buffer offset
    
    ; Check for errors in received data
    qbeq error_detected_in_data_frames,ERROR_STATUS_REG,0 ; Check for Manchester error
    qba receive_data_frames                         ; Continue receiving data

;******************************************************************************
; Response Processing
;
; Processes the received response including error detection, postamble 
; detection, and handling of special commands.
;******************************************************************************
error_detected_in_data_frames:
    ; Check if this is postamble or actual error
    qbeq rx_sampling_done, BIT_CAPTURE_REG.b0,POSTAMBLE_PATTERN ; Check for postamble
    ; If not postamble, error occurred in sampling
	qba error_detected                                     ; Handle error
rx_sampling_done:
    ; Successful reception complete
	ldi TEMP2.b0,0                                  ; Clear status
	ldi32 DMEM_OFFSET,HOST_TRIGGER_STATUS_FLAG      ; Get status flag address
	sbbo &TEMP2.b0,DMEM_OFFSET,0,1                  ; Clear flag
	
	; Check operating mode for next action
	ldi32 DMEM_OFFSET, ENDAT3_OPMODE_CONFIG_OFFSET
	lbbo &TEMP2.b0, DMEM_OFFSET, 0, 1              ; Read opmode
	qbne host_trigger_next_cmd, TEMP2.b0, 0        ; If host mode, wait for next host trigger

	; Periodic mode: Generate interrupt and loop back
	ldi R31.w0, PRU_TRIGGER_HOST_ENDAT3_EVT0        ; Generate interrupt to host
	qba handle_periodic_trigger_mode                ; Loop back for next periodic trigger
host_trigger_next_cmd:
	; Check if this was a reset command
    ldi DMEM_OFFSET,TX_FRAME_ID_OFFSET              ; Get command ID address
    lbbo &TEMP2.b0,DMEM_OFFSET,0,1                  ; Read command ID
    qbeq init_seq_hello,TEMP2.b0,RESET_CMD          ; If reset command, restart hello sequence
	qba check_operating_mode                        ; Otherwise check operating mode for next command

;******************************************************************************
; Error Handling
;
; Handles various error conditions that can occur during transmission,
; reception, or decoding. Checks operating mode to determine next action.
;******************************************************************************
error_detected:
    ; Report error to host
	ldi TEMP2.b0,SAMPLING_ERROR_FLAG                ; Set error flag value
	ldi32 DMEM_OFFSET,HOST_TRIGGER_STATUS_FLAG      ; Get status flag address
	sbbo &TEMP2.b0,DMEM_OFFSET,0,1                  ; Set error flag
	
	; Check operating mode for next action
	ldi32 DMEM_OFFSET, ENDAT3_OPMODE_CONFIG_OFFSET
	lbbo &TEMP2.b0, DMEM_OFFSET, 0, 1              ; Read opmode
	qbne host_trigger_error_cmd, TEMP2.b0, 0       ; If host mode, wait for next host trigger
	
	; Periodic mode: Generate interrupt and loop back
	ldi R31.w0, PRU_TRIGGER_HOST_ENDAT3_EVT0        ; Generate interrupt to host
	qba handle_periodic_trigger_mode                ; Loop back for next periodic trigger
	
host_trigger_error_cmd:
	; Host mode: Update frame counter and prepare for next command
	sub TX_FRAMES_LEFT,TX_FRAMES_LEFT,1             ; Update frame counter
	qba new_request_start                           ; Restart request cycle