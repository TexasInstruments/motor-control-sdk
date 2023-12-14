
;
; Copyright (C) 2023 Texas Instruments Incorporated
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
;*   File:     bissc_params.h                                                       *
;*                                                                                  *
;*   Brief:    Defining Register usage and setting BissC specific parameters        *
;************************************************************************************


    ; register usage
	; R1-R6 - flipflops for otf crc.
	; R7 - holds received crc for position data and used as ex for otf calculation.
	; R8.b0 - valid bit,rx enable, rx valid, R8.b1 - holds channel mask, R8.w2 - stores raw data length (pos_data_length + ew_length + pos_crc_length). 
	; R9.b0 - holds fifo bit index(middle bit) of oversampled data, R9.b1 - holds base offset for channel in use, R9.w2 - holds CFG0 offset for channel in use.
	; R10 - flipflops for control communication otf crc.
	; R11.b1 - holds 4-bit control communication crc, R11.b2 - holds cds data, R11.b3 - holds ctrl otf crc.
	; R12.b0 - control command bit pointer, R12.b1 - control cycle counter.
	; R12.b2 - holds the channel number for channel in use, R12.b3 - holds encoder offset for encoder connected in daisy chain.
	; R13-R14 - holds raw data for channel 0, R15-R16 - holds raw data for channel 1, R17-R18 - holds raw data for channel 2.
	; R19 - holds cds backup for one biss-c cycle.
	; R20 - Used as flag register.
	; R21 - holds hex equivalent control command.
	; R22 - holds daisy chain configurations.
	; R23.b0 - used for execution status for each PRU in load share. R23.b1 - holds primary core mask.
	; R23.w2 -it is used as status register.
	; R24-R27 - scratch.
	; R29 - holds call and return (branch) address.
	
	; unused Register: R11.b0, R28

	.asg	R1, 		FF0
	.asg	R2, 		FF1 
	.asg 	R3, 		FF2
	.asg 	R4, 		FF3 
	.asg 	R5, 		FF4 
	.asg 	R6, 		FF5
	.asg 	R7, 		EX
	.asg 	R7, 		BISSC_RCV_CRC
	.asg	R8.b0, 		VALID_BIT_IDX
	.asg	R8.b0,		BISSC_RX_EN
	.asg	R8.b0, 		BISSC_RX_VALID
	.asg	R8.b1,		CH_MASK
	.asg 	R8.w2, 		RAW_DATA_LEN			;length(used for data loop) including ew and crc
	.asg 	R9.b0, 		FIFO_BIT_IDX
	.asg	R9.b1, 		BASE_OFFSET
	.asg	R9.w2, 		BISSC_ICSSG_CHx_CFG0
	.asg	R10,		BISSC_CTRL_FF		
	.asg 	R11.b1, 	BISSC_CTRL_CRC
	.asg 	R11.b2, 	BISSC_CTRL_CMD_RES
	.asg	R11.b3,		BISSC_CTRL_OTF_CRC
	.asg 	R12.b0, 	BISSC_CMD_BIT_PTR
	.asg 	R12.b1, 	BISSC_CTRL_CYCLE_CNTR
	.asg 	R12.b2,	 	BISSC_ENABLE_CHx_IN_USE
	.asg 	R12.b3, 	ENCODER_OFFSET
	.asg	R13,		RAW_DATA1_0				;Includes Position data, ew and CRC
	.asg	R14,		RAW_DATA2_0
	.asg	R15,		RAW_DATA1_1				;Includes Position data, ew and CRC
	.asg	R16,		RAW_DATA2_1
	.asg	R17,		RAW_DATA1_2				;Includes Position data, ew and CRC
	.asg	R18,		RAW_DATA2_2
	.asg	R19, 		BISSC_CDS_BACKUP
	.asg	R20,		BISSC_FLAGS_REG
	.asg	R21, 		BISSC_CTRL_CMD
	.asg 	R22, 		DAISY_CHAIN	
	.asg	R23.b0, 	LS_SYNC_STATE
	.asg 	R23.b1, 	PRIMARY_CORE			
	.asg	R23.b2, 	STATUS_REG1
	.asg	R23.b3, 	STATUS_REG2
	.asg	R24,		SCRATCH
	.asg	R25,		SCRATCH1
	.asg	R26,		SCRATCH2
	.asg 	R27, 		SCRATCH3
	.asg 	R29, 		LINK_REG

    
BISSC_RX_CRCBITS 				.set    6			;Pos data CRC len
BISSC_SB_CDS_LEN	            .set    2			;Start bit + CDS bit len
BISSC_EW_LEN		            .set	2			;Error warning len

; Time units below in nano sec

BISSC_MAX_TBISS_TIMEOUT 		.set 	40000		;max biss timeout
BISSC_MIN_TBISS_TIMEOUT 		.set 	12500		;min biss timeout
BISSC_TLINEDELAY  				.set 	40000		;max Line delay

BISSC_BUSY_TBUSY_S  			.set 	40000	 	;max processing time for single cycle data
BISSC_BUSY_TbUSY_R  			.set 	20000000	;max processing time for register access

BISSC_CONFIG_CH0				.set	0x1			;Config Endat channel 0
BISSC_CONFIG_CH1				.set	(0x1 << 1)	;Config Endat channel 1
BISSC_CONFIG_CH2				.set	(0x1 << 2)	;Config Endat channel 2

BISSC_CH0_RX_VALID_BIT_OFFSET	.set	24			;RX valid bit offset channel 0
BISSC_CH1_RX_VALID_BIT_OFFSET	.set	25			;RX valid bit offset channel 1
BISSC_CH2_RX_VALID_BIT_OFFSET	.set	26			;RX valid bit offset channel 2

BISSC_CH0_VALID_BIT_IDX			.set 	24			;RX valid bit index channel 0
BISSC_CH1_VALID_BIT_IDX			.set 	25			;RX valid bit index channel 1
BISSC_CH2_VALID_BIT_IDX			.set 	26			;RX valid bit index channel 2

BISSC_MAX_FRAME_SIZE			.set	256			;Max frame size for Processing delay measurement
BISSC_MAX_WAIT_FOR_ENC_DETECT	.set	10000		;Max wait count for encoder detected