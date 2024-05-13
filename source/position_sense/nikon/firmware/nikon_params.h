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
;*   File:     nikon_params.h                                                       *
;*                                                                                  *
;*   Brief:    Defining Register usage and setting Nikon specific parameters        *
;************************************************************************************


    ; register usage
	; R0		- Use for HW config
	; R1-R8	 	- Flipflops
	; R9.w0		- Captured RAW data for channel 0
	; R9.w2 	- Captured RAW data for channel 1
	; R10.w0 	- Captured RAW data for channel 2
	; R11.b0	- 8 bit calculated On-the-fly CRC for channel 0
	; R11.b1 	- 8 bit calculated On-the-fly CRC for channel 1
	; R11.b2 	- 8 bit calculated On-the-fly CRC for channel 2
	; R12.b0 	- 8 bit receivedd CRC for channel 0
	; R12.b1 	- 8 bit receivedd CRC for channel 1
	; R12.b2 	- 8 bit receivedd CRC for channel 2
	; R13.b0	- Valid bit index for selected channel
	; R13.b1 	- FIFO bit index for selected channel
	; R13.b2 	- Channel mask for channel in use
	; R13.b3	- EEPROM access status
	; R14 	 	- Actual CDF command sent on wire
	; R14		- EX: used as temporary reg in otf calculation
	; R15	 	- Memory data frame command sent on wire for EEPROM access
	; R15.b0	- Number of encoders connected to one channel
	; R15.b1	- Number of Rx frames to be received
	; R15.b2	- Current the value of channel in use
	; R15.b3	- Holds Rx clock frrequency
	; R16.b0 	- Offset of current encoder in BUS configuration.
	; R16.b1 	- Offset of RAW DATA from current encoder on current channel.
	; R17-R20	- Used as scratch registers
	; R21-R28	- Free
	; R29	 	- Link register
	; unused Register: R21-R28
	.asg 	R0, 	HW_CONFIG
	.asg    R1, 	FF0
    .asg    R2, 	FF1
    .asg    R3, 	FF2
    .asg    R4, 	FF3
    .asg    R5, 	FF4
    .asg    R6, 	FF5
    .asg    R7, 	FF6
    .asg    R8, 	FF7
	.asg 	R9, 	RAW_DATA
	.asg 	R9.w0, 	RAW_DATA0
	.asg 	R9.w2, 	RAW_DATA1
	.asg 	R10.w0, RAW_DATA2
	.asg 	R10.b2, TX_MODE
	.asg 	R11, 	NIKON_OTF_CRC
	.asg 	R12,    NIKON_RCV_CRC
	.asg 	R13.b0, VALID_BIT_IDX
	.asg 	R13.b1, FIFO_BIT_IDX
	.asg 	R13.b2, CH_MASK
	.asg 	R13.b3, EEPROM_ACCESS_STATUS
	.asg 	R14, 	CDF_FRAME
	.asg 	R14, 	EX
	.asg 	R15, 	MDF_FRAME
	.asg 	R15.b0, NUM_ENCODERS
	.asg 	R15.b1, NUM_FRAMES
	.asg 	R15.b2, CH_IN_USE
	.asg 	R15.b3, RX_FREQ
	.asg 	R16.b0, ENCODER_OFFSET
	.asg 	R16.b1, RAW_DATA_OFFSET
	.asg 	R16.b2, LS_SYNC_STATE
	.asg 	R16.b3, PRIMARY_CORE
	.asg	R17, 	SCRATCH
	.asg	R18, 	SCRATCH1
	.asg 	R19, 	SCRATCH2
	.asg 	R20, 	SCRATCH3
	.asg 	R29, 	LINK_REG

NIKON_16MHZ_FREQ 						.set 	16			;16MHz frequency
NIKON_RX_CRC_POLY 						.set 	0x11D		;Polynomial for 8-bit crc calculation
TX_SINGLE_SHOT 							.set 	(31 << 3)	;18 bit cdf or mdf with 13 1's to compensate delays
TX_CONTINUOUS_MODE 						.set 	(0 << 3)	;'0' as Tx frame size will configure Tx in continuous mode

POLLING_MASK    						.set    0x1C    	;Mask to poll the required bits in Tx Continuous FIFO loading
ONE_BYTE_FIFO_LEVEL 					.set 	0x04  		;Mask to check whether the Tx FIFO level is at 1 byte
TWO_BYTES_FIFO_LEVEL    				.set    0x08  		;Mask to check whether the Tx FIFO level is at 2 bytes

DATA_SIZE_IN_EACH_FRAME					.set 	16			;Length of Data in each Rx frame, except start bit and stop bit.
NIKON_AUTO_ARM_DELAY 					.set 	0x0A		;Enable Rx after a certain period, by loading auto arm delay as non zero value.
NUM_OF_NIKON_CYC_FOR_EEPROM_ACCESS 		.set 	2			;Number of cycles to be executed for accessing EEPROM
NIKON_RX_CRC_LEN						.set 	8			;length of CRC received in last Rx frame

EEPROM_READ_CMD 						.set 	1			;status flag for EEPROM read command
EEPROM_WRITE_CMD 						.set 	2			;status flag for EEPROM write command
NON_EEPROM_CMD							.set 	0			;status flag for Non-EEPROM command
NIKON_PRU_TRIGGER_HOST_EVT				.set	34			;( pr0_pru_mst_intr[2]_intr_req )