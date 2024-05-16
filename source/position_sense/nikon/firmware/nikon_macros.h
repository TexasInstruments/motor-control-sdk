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
;*	 File:	nikon_macros.h															*
;*																					*
;*	 Brief: Macros for calculating On the Fly CRC of position data and returning	*
;*	 		the CRC status with RAW data											*
;*	       	Macros for call and return(branching) 									*
;*          Macro for enabling pru cycle counter 									*
;* 			Macro for disabling pru cycle counter 									*
;*  		Macro for configuring Tx in singleshot mode 							*
;* 			Macro for configuring Tx in continuous mode for EEPROM Read.			*
;* 			MAcro for configuring Tx in continuous mode for EEPROM Write			*
;*			Macro for synchronizing PRUs in load share.								*
;************************************************************************************


	.include "nikon_icss_reg_defs.h"
	.include "nikon_params.h"
	.include "firmware_version.h"
	.global  NIKON_WAIT_FOR_FILL_LEVEL

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macros: CALL, CALL2
; 	Branch to defined function and stores the return address.
; Macros: RET, RET2
;	Returns to address stored in defined registers.
; Registers:
;	LINK_REG.w0: Holds the address of function specified in CALL macro.
;	LINK_REG.w2: Holds the address of function specified in CALL2 macro.
;
;  PseudoCode:
;	 (start code)
;		1.Jump to specified function for CALL or CALL2 macros and stores the return
;		  address in LINK_REG.w0 or LINK_REG.w2.
;		2.Returns to address stored in LINK_REG.w0 or LINK_REG.w2 when RET or RET2 macro is called.
;	 (end code)
;
;	 Worst case peak cycle usage: 1
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
CALL	.macro function
	JAL	LINK_REG.w0,	function
	.endm
RET	.macro
	JMP	LINK_REG.w0
	.endm

CALL2	.macro function
	JAL	LINK_REG.w2,	function
	.endm
RET2	.macro
	JMP	LINK_REG.w2
	.endm
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_ENABLE_PRU_CYCLE_COUNTER, M_DISABLE_PRU_CYCLE_COUNTER
; 	Enable or Disable the PRU cycle counter.
; Registers:
;	SCRATCH1: Holds PRU control configurations.
;
;  PseudoCode:
;	 (start code)
;		1.Load the ICSS_PRU_CTRL_CONST to SCRATCH2 and SET the 3rd bit to enable PRU cycle counter or
;		  CLR the 3rd bit to disable PRU cycle counter.
;		2.Store the updated SCRATCH2 at ICSS_PRU_CTRL_CONST offset in shared memory.
;	 (end code)
;
;	 Worst case peak cycle usage: 13
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_ENABLE_PRU_CYCLE_COUNTER .macro
	LBCO	&SCRATCH1, ICSS_PRU_CTRL_CONST, 0, 4
	SET 	SCRATCH1, SCRATCH1, 3
	SBCO	&SCRATCH1, ICSS_PRU_CTRL_CONST, 0, 4
  .endm
M_DISABLE_PRU_CYCLE_COUNTER .macro
	LBCO	&SCRATCH1, ICSS_PRU_CTRL_CONST, 0, 4
	CLR 	SCRATCH1, SCRATCH1, 3
	SBCO	&SCRATCH1, ICSS_PRU_CTRL_CONST, 0, 4
  .endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC
; 	Receive and Downsample the position data and do not invoke for CRC bits
;	(single channel or load share).
; Registers:
;	Rx: Holds the raw data receive - caller has to zero or provide value to continue
;	ff[0-7]: Flipflops for otf crc calculation.
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 16 or 8 as we are returning
;	     only one RX frame at a time.
;	ex: Temporary variable
;
;  PseudoCode:
;		(start code)
;		1.Left shift the Rx and Wait for valid bit
;		  then check the Middle Bit of Rx fifo and store the same in Rx.
;		2.Do the On the fly crc algorithm for receive bit.
;		3.Loop through the steps 1-2 for cnt iterations.
;		(end code)
;
;	 Worst case peak cycle usage: 16(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	.macro	Rx, cnt, bit_idx, valid_bit, ff0, ff1, ff2, ff3, ff4, ff5, ff6, ff7, ex

    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?, cnt
    LSL     Rx, Rx, 1
NIKON_WB1?:
    QBBC	NIKON_WB1?,	R31, valid_bit ;  wait for valid
    MOV     SCRATCH1, R31
	QBBS	NIKON_RCV_HIGH?, SCRATCH1, bit_idx	;  Check the Mid bit of receive oversampled data
	MOV		ex, ff7	; if code[i] = 0, ex = ff7 (ex = ff7 ^ 0)
	JMP		NIKON_RCV_LOW?
NIKON_RCV_HIGH?:
	NOT		ex,	ff7	; if code[i] = 1, ex = !ff7 (ex = ff7 ^ 1)
	OR		Rx,	Rx,	1
NIKON_RCV_LOW?:
	SET		R31,	R31,    valid_bit ;  clear valid bit
	MOV		ff7,	ff6
	MOV		ff6,	ff5
	MOV		ff5,	ff4
    XOR     ff4,    ff3,    ex
    XOR     ff3,    ff2,    ex
    XOR     ff2,    ff1,    ex
	MOV		ff1,	ff0
    MOV     ff0,    ex
NIKON_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE
; 	Receive and Downsample the CRC bits(single channel or load share).
; Registers:
;	Rx: Holds the raw data receive - caller has to zero or provide value to continue
;	Ry: Holds receive CRC
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 8 as we are receiving the CRC.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry and Wait for valid bit then check the Middle Bit of Rx fifo and
;		  store the same in Rx and Ry.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 10(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE	.macro	Rx, Ry, cnt, bit_idx, valid_bit
    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
NIKON_WB2?:
	QBBC	NIKON_WB2?,	R31,	valid_bit ;  wait for valid
    MOV     SCRATCH1, R31
	QBBC	NIKON_CRC_LOW?,	SCRATCH1,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_CRC_HIGH?:
	OR		Rx,	Rx,	1
	OR		Ry,	Ry,	1
NIKON_CRC_LOW?:
	SET		R31,	R31, valid_bit ;  clear valid bit
NIKON_RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_CALC_CRC
; 	Store and return the calculated On-the-fly CRC.
; Registers:
;	ff[0-7]: Holds calculated CRC bits separately
;	crc: Returns the overall CRC computation value.
;
;  	PseudoCode:
;      (start code)
;       1.Clear the crc register
;		2.Check for individual bit(bit-0) from ff[0-7]
;		3.Set the bits in crc register if the 0th bit of respective numbered flipflop is high.
;       4.Repeat step 2 and 3 for 8 ff.
;       (end code)
;
;	Worst case peak cycle usage: 17
;
 ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_CALC_CRC 	.macro 	crc, ff0, ff1, ff2, ff3, ff4, ff5, ff6, ff7

    LDI		crc,	0
	QBBC	NIKON_CRC_BIT1?,	ff0,	0
	SET		crc,	crc,	0
NIKON_CRC_BIT1?:
	QBBC	NIKON_CRC_BIT2?,	ff1,	0
	SET		crc,	crc,	1
NIKON_CRC_BIT2?:
	QBBC	NIKON_CRC_BIT3?,	ff2,	0
	SET		crc,	crc,	2
NIKON_CRC_BIT3?:
	QBBC	NIKON_CRC_BIT4?,	ff3,	0
	SET		crc,	crc,	3
NIKON_CRC_BIT4?:
	QBBC	NIKON_CRC_BIT5?,	ff4,	0
	SET		crc,	crc,	4
NIKON_CRC_BIT5?:
	QBBC	NIKON_CRC_BIT6?,	ff5,	0
	SET		crc,	crc,	5
NIKON_CRC_BIT6?:
	QBBC	NIKON_CRC_BIT7?,	ff6,	0
	SET		crc,	crc,	6
NIKON_CRC_BIT7?:
	QBBC	NIKON_CRC_END?,	    ff7,	0
	SET		crc,	crc,	7
NIKON_CRC_END?:
	.endm


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE
; 	Receive and Downsample the raw data then calculate the CRC(single channel or load share).
; Registers:
;	Ra: Holds the raw data receive - caller has to zero or provide value to continue
; 	num_frames: Number of frames to receive from the encoder
; 	frame_cnt: 	Counter to track the current frame number
; 	raw_data_offset: Offset of raw data memory location.
;	SCRATCH2.b1: RX data size in each frame except CRC byte.
;	ff[0-7].b0: Flipflops for otf crc calculation.
;	EX.b0: temporary variable for otf calculation
;	NIKON_RCV_CRC.b0: used for returning the receive crc results.
;   NIKON_OTF_CRC.b0: used to hold otf crc calculated.
; 	FIFO_BIT_IDX: used to hold fifo bit index.
; 	VALID_BIT_IDX: used to hold valid bit index.
;
;  PseudoCode:
;	(start code)
;       1.Increment the current frame count, raw data offset and Poll for start bit.
; 		2.Load SCRATCH2.b1 with the frame size - crc length if current frame is last frame.
; 		3.Receive position data in Ra with respect to the frame size and calculate On-the-fly CRC by
;		  calling onto M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC.
;		4.Receive crc by calling onto M_OTF_RECEIVE_AND_DOWNSAMPLE
;		  if the current frame is last frame else skip to this step.
; 		5.Poll for stop bit.
; 		6.Store the receive data in memory offset.
;		7.Repeat step(1-6) for current frame is the last frame.
; 		8.Store OTF crc in a local register by calling onto  M_CALC_CRC.
; 		9.Increment the CRC Error Count if the otf crc is not equal to receive crc and
;		  store the otf crc, receive crc, CRC Error Count.
;	(end code)
;
;	 Worst case peak cycle usage:
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE	.macro	Ra, num_frames, frame_cnt, raw_data_offset
NIKON_START_RECIEVE?:
    ADD     frame_cnt, frame_cnt, 1
    QBNE    NIKON_START_BIT?, frame_cnt, num_frames		;Last RX frame contains 8 bit data and 8 bit crc
    LDI     SCRATCH2.b1, NIKON_RX_CRC_LEN				;Loop for first 8 bit in M_OTF_RECEIVE_DOWNSAPLE_AND_CRC and next 8 bit in M_OTF_RECEIVE_AND_DOWNSAMPLE

NIKON_START_BIT?:
    QBBC    NIKON_START_BIT?, R31, VALID_BIT_IDX		;Poll for start bit at beginning of each RX frame
    MOV     SCRATCH1, R31
    SET     R31, R31, VALID_BIT_IDX
    QBBS    NIKON_START_BIT?, SCRATCH1, FIFO_BIT_IDX
    ;must maintain 3 cycles before checking next valid bit
NIKON_DATA_RCV?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC 	Ra, SCRATCH2.b1, FIFO_BIT_IDX, VALID_BIT_IDX, FF0.b0, FF1.b0, FF2.b0, FF3.b0, FF4.b0, FF5.b0, FF6.b0, FF7.b0, EX.b0
	;Receive the data bits and performe otf crc algorithm.
    QBNE    NIKON_STOP_BIT?, frame_cnt, num_frames
	;Recieve crc bits from last RX frame.
	M_OTF_RECEIVE_AND_DOWNSAMPLE 	Ra, NIKON_RCV_CRC.b0, SCRATCH2.b1, FIFO_BIT_IDX, VALID_BIT_IDX
	;Receive crc bits to compare with otf crc
	NOP
NIKON_STOP_BIT?:
    QBBC    NIKON_STOP_BIT?, R31, VALID_BIT_IDX			;Poll for stop bit at end of each RX frame
    MOV     SCRATCH1, R31
    SET     R31, R31, VALID_BIT_IDX
    QBBC    NIKON_STOP_BIT?, SCRATCH1, FIFO_BIT_IDX
	SBCO 	&Ra, PRUx_DMEM, raw_data_offset, 2			;Store 16 bit data from each frame excluding start bit and stop bit.
	ADD 	raw_data_offset, raw_data_offset, 6			;Increment to the offset of next RX frame memory location
    ZERO    &Ra, 4

    QBNE    NIKON_START_RECIEVE?, frame_cnt, num_frames	;Poll for start bit of next frame if current frame is not last frame.
	M_CALC_CRC 	NIKON_OTF_CRC.b0, FF0.b0, FF1.b0, FF2.b0, FF3.b0, FF4.b0, FF5.b0, FF6.b0, FF7.b0 ;Store the clculated On-the-fly CRC
	QBEQ 	NIKON_CRC_SUCCESS?, NIKON_RCV_CRC.b0, NIKON_OTF_CRC.b0
	LBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
	ADD 	SCRATCH1, SCRATCH1, 1						;Increment the crc error counter if otf crc and receive crc does not match
	SBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
NIKON_CRC_SUCCESS?:
	SBCO 	&NIKON_OTF_CRC.b0, PRUx_DMEM, SCRATCH3.b3, 1;Store otf crc and receive crc
	ADD 	SCRATCH3.b3, SCRATCH3.b3, 3
	SBCO 	&NIKON_RCV_CRC.b0, PRUx_DMEM, SCRATCH3.b3, 1

	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC
; 	Receive and Downsample the position data and do not invoke for CRC bits(Multi Channel).
; Registers:
;	Rx, Ry, Rz: Holds the raw data receive for each channel - caller has to zero or provide value to continue
;	ff[0-7]: Flipflops for otf crc calculation.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 16 or 8 as we are returning only one RX frame at a time.
;	ex: Temporary variable
;	CH_MASK: Mask for all selected channels
; 	SCRATCH1.b0: to hold "and" product for channel mask and valid bit indexes for 3 channels.
;
;
;  PseudoCode:
;		(start code)
;		1.Left shift the Rx, Ry, Rz and Wait for all channel valid bit to be set
;		  then check the Middle Bit of Rx fifo of all channels and store the same in Rx, Ry, Rz.
;		2.Do the On the fly crc algorithm for receive bits of all channels.
;		3.Loop through the steps 1-2 for cnt iterations.
;		(end code)
;
;	 Worst case peak cycle usage: 24(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC	.macro	Rx, Ry, Rz, cnt, bit_idx, ff0, ff1, ff2, ff3, ff4, ff5, ff6, ff7, ex

    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?, cnt
    LSL     Rx, Rx, 1
	LSL 	Ry, Ry, 1
	LSL	 	Rz, Rz, 1
NIKON_WB1?:
    AND 	SCRATCH1.b0, R31.b3, CH_MASK
	QBNE	NIKON_WB1?,	SCRATCH1.b0, CH_MASK ;  wait for valid
	QBBS	NIKON_SET_BIT_CH0?, R31.b0, bit_idx	;  Check the Mid bit of receive oversampled data
	MOV		ex.b0, 	ff7.b0	; if code[i] = 0, ex = ff7 (ex = ff7 ^ 0)
	JMP		NIKON_SKIP_BIT_CH0?
NIKON_SET_BIT_CH0?:
	NOT		ex.b0,	ff7.b0	; if code[i] = 1, ex = !ff7 (ex = ff7 ^ 1)
	OR		Rx,		Rx,		1
NIKON_SKIP_BIT_CH0?:
	QBBS	NIKON_SET_BIT_CH1?, R31.b1, bit_idx	;  Check the Mid bit of receive oversampled data
	MOV		ex.b1, 	ff7.b1	; if code[i] = 0, ex = ff7 (ex = ff7 ^ 0)
	JMP		NIKON_SKIP_BIT_CH1?
NIKON_SET_BIT_CH1?:
	NOT		ex.b1,	ff7.b1	; if code[i] = 1, ex = !ff7 (ex = ff7 ^ 1)
	OR		Ry,		Ry,		1
NIKON_SKIP_BIT_CH1?:
	QBBS	NIKON_SET_BIT_CH2?,  R31.b2, bit_idx	;  Check the Mid bit of receive oversampled data
	MOV		ex.b2, 	ff7.b2	; if code[i] = 0, ex = ff7 (ex = ff7 ^ 0)
	JMP		NIKON_SKIP_BIT_CH2?
NIKON_SET_BIT_CH2?:
	NOT		ex.b2,	ff7.b2	; if code[i] = 1, ex = !ff7 (ex = ff7 ^ 1)
	OR		Rz,		Rz,		1
NIKON_SKIP_BIT_CH2?:
	MOV 	R31.b3, SCRATCH1.b0 ;  clear valid bit
	MOV		ff7,	ff6
	MOV		ff6,	ff5
	MOV		ff5,	ff4
    XOR     ff4,    ff3,    ex
    XOR     ff3,    ff2,    ex
    XOR     ff2,    ff1,    ex
	MOV		ff1,	ff0
    MOV     ff0,    ex
NIKON_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_MC
; 	Receive and Downsample the CRC bits(Multi Channel).
; Registers:
;	Rx, Ry, Rz: Holds the raw data receive from all channels- caller has to zero or provide value to continue
;	Ra: Holds receive CRC from all channels
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 6 as we are receiving the CRC.
; 	CH_MASK: Mask for all selected channels
; 	SCRATCH1.b0: to hold "and" product for channel mask and valid bit indexes for 3 channels.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry, Rz, Ra and Wait for all channels valid bits to be set
;		  then check the Middle Bit of Rx fifo and store the same in Rx, Ry, Rz and Ra.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 17(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_MC	.macro	Rx, Ry, Rz, Ra, cnt, bit_idx
    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL 	Rz, Rz, 1
	LSL 	Ra, Ra, 1 ; this will do for all 3 channels
NIKON_WB2?:
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
	QBNE	NIKON_WB2?,	SCRATCH1.b0,	CH_MASK ;  wait for valid
	MOV 	SCRATCH1, R31
	QBBC	NIKON_SKIP_CRC_BIT_CH0?,	SCRATCH1.b0,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_CRC_BIT_CH0?:
	OR		Rx,	Rx,	1
	OR		Ra.b0,	Ra.b0,	1
NIKON_SKIP_CRC_BIT_CH0?:
	QBBC	NIKON_SKIP_CRC_BIT_CH1?,	SCRATCH1.b1,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_CRC_BIT_CH1?:
	OR		Ry,	Ry,	1
	OR		Ra.b1,	Ra.b1,	1
NIKON_SKIP_CRC_BIT_CH1?:
	QBBC	NIKON_SKIP_CRC_BIT_CH2?,	SCRATCH1.b2,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_CRC_BIT_CH2?:
	OR		Rz,	Rz,	1
	OR		Ra.b2,	Ra.b2,	1
NIKON_SKIP_CRC_BIT_CH2?:
	NOP						;  incase all channel fifo bit is '0' we will clear the valid bit within 3 PRU cycles
	MOV 	R31.b3, CH_MASK ;  clear valid bit
	NOP
NIKON_RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_CALC_CRC_MC
; 	Store and return the calculated CRC(Multi Channel).
; Registers:
;	ff[0-7]: Holds calculated CRC bits separately for all channels
;	crc: Returns the overall CRC computation values of all channels.
;
;  	PseudoCode:
;      (start code)
;       1.Clear the crc register
; 		2.Left Shift all the flipflops to have last channel flipflops in ff[0-7].b3
;		3.Set the bits in crc.b0 register if the 0th bit of respective numbered ff[0-7].b3 is high.
;       4.Repeat step 2 and 3 for 8 ff.
;		5.Loop step 2-4 for all channels.
;       (end code)
;
;	Worst case peak cycle usage: 27(each channel)
;
 ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_CALC_CRC_MC 	.macro 	crc, ff0, ff1, ff2, ff3, ff4, ff5, ff6, ff7
	ZERO 	&crc, 	4

	LOOP 	NIKON_CRC_CH0_END?, 	3

	LSL 	ff0, 	ff0, 	8
	LSL 	ff1, 	ff1, 	8
	LSL 	ff2, 	ff2, 	8
	LSL 	ff3, 	ff3, 	8
	LSL 	ff4, 	ff4, 	8
	LSL 	ff5, 	ff5, 	8
	LSL 	ff6, 	ff6, 	8
	LSL 	ff7, 	ff7, 	8

	; ch2 would be initially in b0, at the end of the loop it would have reached b2, finally ch[0-2] would end up @b[0-2] as reqd.
	; no-op in first interation
	LSL		crc, 	crc, 	8
	QBBC	NIKON_CRC_CH0_BIT1?,	ff0.b3,	0
	SET		crc.b0,	crc.b0,	0
NIKON_CRC_CH0_BIT1?:
	QBBC	NIKON_CRC_CH0_BIT2?,	ff1.b3,	0
	SET		crc.b0,	crc.b0,	1
NIKON_CRC_CH0_BIT2?:
	QBBC	NIKON_CRC_CH0_BIT3?,	ff2.b3,	0
	SET		crc.b0,	crc.b0,	2
NIKON_CRC_CH0_BIT3?:
	QBBC	NIKON_CRC_CH0_BIT4?,	ff3.b3,	0
	SET		crc.b0,	crc.b0,	3
NIKON_CRC_CH0_BIT4?:
	QBBC	NIKON_CRC_CH0_BIT5?,	ff4.b3,	0
	SET		crc.b0,	crc.b0,	4
NIKON_CRC_CH0_BIT5?:
	QBBC	NIKON_CRC_CH0_BIT6?,	ff5.b3,	0
	SET		crc.b0,	crc.b0,	5
NIKON_CRC_CH0_BIT6?:
	QBBC	NIKON_CRC_CH0_BIT7?,	ff6.b3,	0
	SET		crc.b0,	crc.b0,	6
NIKON_CRC_CH0_BIT7?:
	QBBC	NIKON_CRC_CH0_END?,		ff7.b3,	0
	SET		crc.b0,	crc.b0,	7
NIKON_CRC_CH0_END?:
	.endm


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_MC
; 	Receive and Downsample the raw data then calculate the CRC(Multi Channel).
; Registers:
;	Ra, Rb, Rc: Holds the raw data receive - caller has to zero or provide value to continue
; 	num_frames: Number of frames to receive from the encoder
; 	frame_cnt: 	Counter to track the current frame number
; 	raw_data_offset: Offset of raw data memory location.
;	SCRATCH2.b1: RX data size in each frame except CRC byte.
;	ff[0-7]: Flipflops for otf crc calculation for all channels.
;	EX: temporary variables for otf calculation for all channels.
;	NIKON_RCV_CRC: used for returning the receive crc results for all channels.
;   NIKON_OTF_CRC: used to hold otf crc calculated for all channels.
; 	FIFO_BIT_IDX: used to hold fifo bit index.
; 	CH_MASK: Mask for all selected channels
; 	SCRATCH1.b0: to hold "and" product for channel mask and valid bit indexes for 3 channels.
;
;  PseudoCode:
;	(start code)
;       1.Increment the current frame count, raw data offset and Poll for start bit of all channels.
; 		2.Load SCRATCH2.b1 with the frame size - crc length if current frame is last frame.
; 		3.Receive position data in Ra, Rb, Rc, for all channels with respect to the frame size and calculate On-the-fly CRC by calling onto
;		  M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC.
;		4.Receive crc for all channels by calling onto M_OTF_RECEIVE_AND_DOWNSAMPLE_MC if the current frame is last frame else skip to this step.
; 		5.Poll for stop bit for all channels.
; 		6.Store the receive data in memory offset of all channels.
;		7.Repeat step(1-6) for current frame is the last frame.
; 		8.Store OTF crc for all channels in a local register by calling onto  M_CALC_CRC_MC.
; 		9.Increment the CRC Error Count if the otf crc is not equal to receive crc and store the otf crc, receive crc, CRC Error Count for all channels.
;	(end code)
;
;	 Worst case peak cycle usage:
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_MC	.macro	Ra, Rb, Rc, num_frames, frame_cnt, raw_data_offset
NIKON_START_RECIEVE?:
    ADD     frame_cnt, frame_cnt, 1
    QBNE    NIKON_START_BIT?, frame_cnt, num_frames		;Last RX frame contains 8 bit data and 8 bit crc
    LDI     SCRATCH2.b1, NIKON_RX_CRC_LEN				;Loop for first 8 bit in M_OTF_RECEIVE_DOWNSAPLE_AND_CRC_MC and next 8 bit in M_OTF_RECEIVE_AND_DOWNSAMPLE_MC

NIKON_START_BIT?:										;Poll for start bit at beginning of each RX frame
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
    QBNE    NIKON_START_BIT?, SCRATCH1.b0, CH_MASK		;Check for the valid bit of all channels together
    MOV     SCRATCH1, R31
	MOV 	R31.b3, CH_MASK
    QBBC 	NIKON_IS_CH1_FIFO_SB?, CH_MASK, 0
	QBBC	NIKON_SB_FOUND?, SCRATCH1.b0, FIFO_BIT_IDX
NIKON_IS_CH1_FIFO_SB?:
	QBBC 	NIKON_IS_CH2_FIFO_SB?, CH_MASK, 1
	QBBC	NIKON_SB_FOUND?, SCRATCH1.b1, FIFO_BIT_IDX
NIKON_IS_CH2_FIFO_SB?:
	QBBC 	NIKON_START_BIT?, CH_MASK, 2
	QBBS	NIKON_START_BIT?, SCRATCH1.b2, FIFO_BIT_IDX
NIKON_SB_FOUND?:

    ;must maintain 3 cycles before checking next valid bit
NIKON_DATA_RCV?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC 	Ra, Rb, Rc, SCRATCH2.b1, FIFO_BIT_IDX, FF0, FF1, FF2, FF3, FF4, FF5, FF6, FF7, EX
    ;Receive the data bits and performe otf crc algorithm for all channels.
	QBNE    NIKON_STOP_BIT?, frame_cnt, num_frames
	;Recieve crc bits from last RX frame of all channels.
	M_OTF_RECEIVE_AND_DOWNSAMPLE_MC 	Ra, Rb, Rc, NIKON_RCV_CRC, SCRATCH2.b1, FIFO_BIT_IDX
	;Receive crc bits of all channels to compare with otf crc
	NOP

NIKON_STOP_BIT?:
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
    QBNE    NIKON_STOP_BIT?, SCRATCH1.b0, CH_MASK				;Poll for stop bit at end of each RX frame
    MOV     SCRATCH1, R31
	MOV 	R31.b3, CH_MASK
    QBBC 	NIKON_IS_CH1_FIFO_STOPBIT?, CH_MASK, 0
	QBBS	NIKON_STOP_BIT_FOUND?, SCRATCH1.b0, FIFO_BIT_IDX
NIKON_IS_CH1_FIFO_STOPBIT?:
	QBBC 	NIKON_IS_CH2_FIFO_STOPBIT?, CH_MASK, 1
	QBBS	NIKON_STOP_BIT_FOUND?, SCRATCH1.b1, FIFO_BIT_IDX
NIKON_IS_CH2_FIFO_STOPBIT?:
	QBBC 	NIKON_STOP_BIT?, CH_MASK, 2
	QBBC	NIKON_STOP_BIT?, SCRATCH1.b2, FIFO_BIT_IDX
NIKON_STOP_BIT_FOUND?:
	SBCO 	&Ra, PRUx_DMEM, raw_data_offset, 6					;Store 16(x3 for all channels) bit data from each frame excluding start bit and stop bit.
	ADD 	raw_data_offset, raw_data_offset, 6					;Increment to the offset of next RX frame memory location of ch0
    ZERO    &Ra, 6
    QBNE    NIKON_START_RECIEVE?, frame_cnt, num_frames			;Poll for start bit of next frame if current frame is not last frame.
	M_CALC_CRC_MC 	NIKON_OTF_CRC, FF0, FF1, FF2, FF3, FF4, FF5, FF6, FF7	;Store the clculated On-the-fly CRC for all channels
	QBEQ 	NIKON_CH0_CRC_SUCCESS?, NIKON_RCV_CRC.b0, NIKON_OTF_CRC.b0
	LBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
	ADD 	SCRATCH1, SCRATCH1, 1									;Increment the crc error counter if otf crc and receive crc of ch0 does not match
	SBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
NIKON_CH0_CRC_SUCCESS?:
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 4
	QBEQ 	NIKON_CH1_CRC_SUCCESS?, NIKON_RCV_CRC.b1, NIKON_OTF_CRC.b1
	LBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
	ADD 	SCRATCH1, SCRATCH1, 1									;Increment the crc error counter if otf crc and receive crc of ch1 does not match
	SBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
NIKON_CH1_CRC_SUCCESS?:
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 4
	QBEQ 	NIKON_CH2_CRC_SUCCESS?, NIKON_RCV_CRC.b2, NIKON_OTF_CRC.b2
	LBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
	ADD 	SCRATCH1, SCRATCH1, 1									;Increment the crc error counter if otf crc and receive crc of ch2 does not match
	SBCO 	&SCRATCH1, PRUx_DMEM, SCRATCH3.b1, 4
NIKON_CH2_CRC_SUCCESS?:
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 4
	SBCO 	&NIKON_OTF_CRC, PRUx_DMEM, SCRATCH3.b1, 3				;Store otf crc and receive crc for all channels
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 3
	SBCO 	&NIKON_RCV_CRC, PRUx_DMEM, SCRATCH3.b1, 3

	.endm

;************************************************************************************
;*   Note:  These macros are optimize version of M_OTF_RECEIVE to support 			*
;*			16MHz operating frequency with less peak cycle usage					*
;************************************************************************************

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_16MHZ
; 	Receive and Downsample the Rx data bits(single channel or load share).
; Registers:
;	Rx: Holds the raw data receive - caller has to zero or provide value to continue
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be 16 for each RX frame.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx and Wait for valid bit then check the Middle Bit of Rx fifo and
;		  store the same in Rx and Ry.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 9(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ	.macro	Rx, cnt, bit_idx, valid_bit
    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_16MHZ_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
NIKON_WB2_16MHZ?:
	QBBC	NIKON_WB2_16MHZ?,	R31,	valid_bit ;  wait for valid
    MOV     SCRATCH1, R31
	QBBC	NIKON_LOW_16MHZ?,	SCRATCH1,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_HIGH_16MHZ?:
	OR		Rx,	Rx,	1
NIKON_LOW_16MHZ?:
	SET		R31,	R31, valid_bit ;  clear valid bit
    NOP
    NOP
    NOP
    ;must maintain at least 3 cycles between valid bit check and valid bit clear
NIKON_RX_RECEIVE_DOWNSAMPLE_16MHZ_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_16MHZ
; 	Receive and Downsample the raw data (single channel or load share).
; Registers:
;	Ra: Holds the raw data receive - caller has to zero or provide value to continue
; 	num_frames: Number of frames to receive from the encoder
; 	frame_cnt: 	Counter to track the current frame number
; 	raw_data_offset: Offset of raw data memory location.
;	SCRATCH2.b1: RX data size in each frame(16 - bits).
; 	FIFO_BIT_IDX: used to hold fifo bit index.
; 	VALID_BIT_IDX: used to hold valid bit index.
;
;  PseudoCode:
;	(start code)
;       1.Increment the current frame count, raw data offset and Poll for start bit.
; 		2.Receive position data in Ra with respect to the frame size(16 bits each frame) by
;		  calling onto M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ.
; 		3.Poll for stop bit.
; 		4.Store the receive data in memory offset.
;		5.Repeat step(1-4) for current frame is the last frame.
;	(end code)
;
;	 Worst case peak cycle usage:
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_16MHZ	.macro	Ra, num_frames, frame_cnt, raw_data_offset
NIKON_START_RECIEVE_16MHZ?:
    ADD     frame_cnt, frame_cnt, 1
NIKON_START_BIT_16MHZ?:
    QBBC    NIKON_START_BIT_16MHZ?, R31, VALID_BIT_IDX		;Poll for start bit at beginning of each RX frame
    MOV     SCRATCH1, R31
    SET     R31, R31, VALID_BIT_IDX
	NOP
	NOP
    QBBS    NIKON_START_BIT_16MHZ?, SCRATCH1, FIFO_BIT_IDX
    ;must maintain 3 cycles before checking next valid bit
NIKON_DATA_RCV_16MHZ?:
	;Receive the data bits and performe otf algorithm.
	M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ 	Ra, SCRATCH2.b1, FIFO_BIT_IDX, VALID_BIT_IDX
	;Receive crc bits to compare with otf crc
	NOP
NIKON_STOP_BIT_16MHZ?:
    QBBC    NIKON_STOP_BIT_16MHZ?, R31, VALID_BIT_IDX			;Poll for stop bit at end of each RX frame
    MOV     SCRATCH1, R31
    SET     R31, R31, VALID_BIT_IDX
	NOP
	NOP
    QBBC    NIKON_STOP_BIT_16MHZ?, SCRATCH1, FIFO_BIT_IDX
	SBCO 	&Ra, PRUx_DMEM, raw_data_offset, 2			;Store 16 bit data from each frame excluding start bit and stop bit.
	ADD 	raw_data_offset, raw_data_offset, 6			;Increment to the offset of next RX frame memory location
    ZERO    &Ra, 4

    QBNE    NIKON_START_RECIEVE_16MHZ?, frame_cnt, num_frames	;Poll for start bit of next frame if current frame is not last frame.
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_16MHZ_MC
; 	Receive and Downsample the Rx data(Multi Channel).
; Registers:
;	Rx, Ry, Rz: Holds the raw data receive from all channels- caller has to zero or provide value to continue
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo.
;	cnt: Number of bits to receive which should be data length + ew length + sign of life counter.
; 	CH_MASK: Mask for all selected channels
; 	SCRATCH1.b0: to hold "and" product for channel mask and valid bit indexes for 3 channels.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry, Rz and Wait for all channels valid bits to be set
;		  then check the Middle Bit of Rx fifo and store the same in Rx, Ry, Rz.
;		2.Clear the valid bit
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 13(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ_MC	.macro	Rx, Ry, Rz, cnt, bit_idx
    LOOP    NIKON_RX_RECEIVE_DOWNSAMPLE_16MHZ_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL 	Rz, Rz, 1
NIKON_WB2_16MHZ?:
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
	QBNE	NIKON_WB2_16MHZ?,	SCRATCH1.b0,	CH_MASK ;  wait for valid
	QBBC	NIKON_SKIP_BIT_16MHZ_CH0?,	R31.b0,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_BIT_16MHZ_CH0?:
	OR		Rx,	Rx,	1
NIKON_SKIP_BIT_16MHZ_CH0?:
	QBBC	NIKON_SKIP_BIT_16MHZ_CH1?,	R31.b1,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_BIT_16MHZ_CH1?:
	OR		Ry,	Ry,	1
NIKON_SKIP_BIT_16MHZ_CH1?:
	QBBC	NIKON_SKIP_BIT_16MHZ_CH2?,	R31.b2,	bit_idx ;  Check the Mid bit of receive oversampled data
NIKON_SET_BIT_16MHZ_CH2?:
	OR		Rz,	Rz,	1
NIKON_SKIP_BIT_16MHZ_CH2?:
	MOV 	R31.b3, SCRATCH1.b0 ;  clear valid bit
NIKON_RX_RECEIVE_DOWNSAMPLE_16MHZ_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_16MHZ_MC
; 	Receive and Downsample the raw data for operating frequency 16MHz(Multi Channel).
; Registers:
;	Ra, Rb, Rc: Holds the raw data receive - caller has to zero or provide value to continue
; 	num_frames: Number of frames to receive from the encoder
; 	frame_cnt: 	Counter to track the current frame number
; 	raw_data_offset: Offset of raw data memory location.
;	SCRATCH2.b1: RX data size in each frame.
; 	FIFO_BIT_IDX: used to hold fifo bit index.
; 	CH_MASK: Mask for all selected channels
; 	SCRATCH1.b0: to hold "and" product for channel mask and valid bit indexes for 3 channels.
;
;  PseudoCode:
;	(start code)
;       1.Increment the current frame count, raw data offset and Poll for start bit of all channels.
; 		2.Receive position data in Ra, Rb, Rc, for all channels with respect to the frame size by calling onto
;		  M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ_MC.
; 		3.Poll for stop bit for all channels.
; 		4.Store the receive data in memory offset of all channels.
;		5.Repeat step(1-4) for current frame is the last frame.
;	(end code)
;
;	 Worst case peak cycle usage:
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_16MHZ_MC	.macro	Ra, Rb, Rc, num_frames, frame_cnt, raw_data_offset
NIKON_START_RECIEVE_16MHZ?:
    ADD     frame_cnt, frame_cnt, 1
NIKON_START_BIT_16MHZ?:										;Poll for start bit at beginning of each RX frame
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
    QBNE    NIKON_START_BIT_16MHZ?, SCRATCH1.b0, CH_MASK		;Check for the valid bit of all channels together
    MOV     SCRATCH1, R31
	MOV 	R31.b3, CH_MASK
    QBBC 	NIKON_IS_CH1_FIFO_SB_16MHZ?, CH_MASK, 0
	QBBC	NIKON_SB_FOUND_16MHZ?, SCRATCH1.b0, FIFO_BIT_IDX
NIKON_IS_CH1_FIFO_SB_16MHZ?:
	QBBC 	NIKON_IS_CH2_FIFO_SB_16MHZ?, CH_MASK, 1
	QBBC	NIKON_SB_FOUND_16MHZ?, SCRATCH1.b1, FIFO_BIT_IDX
NIKON_IS_CH2_FIFO_SB_16MHZ?:
	QBBC 	NIKON_START_BIT_16MHZ?, CH_MASK, 2
	QBBS	NIKON_START_BIT_16MHZ?, SCRATCH1.b2, FIFO_BIT_IDX
NIKON_SB_FOUND_16MHZ?:

    ;must maintain 3 cycles before checking next valid bit
NIKON_DATA_RCV_16MHZ?:
	M_OTF_RECEIVE_AND_DOWNSAMPLE_16MHZ_MC 	Ra, Rb, Rc, SCRATCH2.b1, FIFO_BIT_IDX
	;Receive crc bits of all channels to compare with otf crc
	NOP

NIKON_STOP_BIT_16MHZ?:
	AND 	SCRATCH1.b0, R31.b3, CH_MASK
    QBNE    NIKON_STOP_BIT_16MHZ?, SCRATCH1.b0, CH_MASK				;Poll for stop bit at end of each RX frame
    MOV     SCRATCH1, R31
	MOV 	R31.b3, CH_MASK
    QBBC 	NIKON_IS_CH1_FIFO_STOPBIT_16MHZ?, CH_MASK, 0
	QBBS	NIKON_STOP_BIT_FOUND_16MHZ?, SCRATCH1.b0, FIFO_BIT_IDX
NIKON_IS_CH1_FIFO_STOPBIT_16MHZ?:
	QBBC 	NIKON_IS_CH2_FIFO_STOPBIT_16MHZ?, CH_MASK, 1
	QBBS	NIKON_STOP_BIT_FOUND_16MHZ?, SCRATCH1.b1, FIFO_BIT_IDX
NIKON_IS_CH2_FIFO_STOPBIT_16MHZ?:
	QBBC 	NIKON_STOP_BIT_16MHZ?, CH_MASK, 2
	QBBC	NIKON_STOP_BIT_16MHZ?, SCRATCH1.b2, FIFO_BIT_IDX
NIKON_STOP_BIT_FOUND_16MHZ?:
	SBCO 	&Ra, PRUx_DMEM, raw_data_offset, 6					;Store 16(x3 for all channels) bit data from each frame excluding start bit and stop bit.
	ADD 	raw_data_offset, raw_data_offset, 6					;Increment to the offset of next RX frame memory location of ch0
    ZERO    &Ra, 6
    QBNE    NIKON_START_RECIEVE_16MHZ?, frame_cnt, num_frames			;Poll for start bit of next frame if current frame is not last frame.
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: NIKON_TX_SEND
; 	Configure Tx in continuous mode and send EEPROM Read or Write command and MDF0-2 over Tx fifo or
;	Configure Tx in singleshot mode and send non EEPROM command CDF over Tx Fifo.
; Registers:
;	cdf: Holds command data frame configured by the user.
;   mdf: Holds memory data frame configured by the user.
;	SCRATCH2.b2: Holds tx frame size.
; 	SCRATCH.w0: Holds cfg registers offsets and delay count.
; 	TX_MODE: Holds Tx mode (singleshot or continuous mode)
;	EEPROM_ACCESS_STATUS: Holds the flag for EEPROM Read or Write.
;
;
;  PseudoCode:
;	(start code)
;       1.Load Tx frame size as TX_MODE in cfg0 register.
;       2.Select the configured channel.
;		3.Load 4 bytes of command data frame to tx fifo.
; 		4.Set the Tx channel go bit inorder to send Tx fifo on wire(encoder).
; 		5.Skip step 6-11 if a non eeprom command is triggered.
; 		6.Track the fill level through R31.bx[2-4], and send one byte each time.
; 		7.Enable PRU cycle counter and poll for 10usec.
;		8.Send 1's till PRU counter counts 10usec.
;		9.Then send MDF2 in case of EEPROM read
; 		10.Send MDF0 and repeat step 6, 7 and 9 for MDF1 and MDF2 in case of EEPROM write.
;		11.Do a global reinit after certain(10usec) delay and
; 		   Repeat 1-7 after 300usec for Read or 30milisec for write delay.
;	(end code)
;
;	 Worst case peak cycle usage:
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
NIKON_TX_SEND 	.macro cdf, mdf
NIKON_START_TX_SEND?:
	QBBC 	NIKON_CH1_TX_SIZE?, CH_MASK, 0
	LDI     SCRATCH.w0, ICSS_CFG_PRUx_NIKON_CH0_CFG0+1
	LBCO	&SCRATCH2.b2,	ICSS_CFG,	SCRATCH.w0,	1
    ;In order to clear the previous value of Tx frame size
    AND     SCRATCH2.b2 , SCRATCH2.b2 , 0x7
	OR 		SCRATCH2.b2, SCRATCH2.b2, TX_MODE
	SBCO 	&SCRATCH2.b2, ICSS_CFG, SCRATCH.w0, 1
	LDI		R30.w2,	(NIKON_TX_CLK_MODE_FREERUN_STOPHIGH | NIKON_TX_CH0_SEL)
NIKON_CH1_TX_SIZE?:
	QBBC 	NIKON_CH2_TX_SIZE?, CH_MASK, 1
	LDI 	SCRATCH.w0, ICSS_CFG_PRUx_NIKON_CH1_CFG0+1
	LBCO	&SCRATCH2.b2,	ICSS_CFG,	SCRATCH.w0,	1
    ;In order to clear the previous value of Tx frame size
    AND     SCRATCH2.b2 , SCRATCH2.b2 , 0x7
	OR 		SCRATCH2.b2, SCRATCH2.b2, TX_MODE
	SBCO 	&SCRATCH2.b2, ICSS_CFG, SCRATCH.w0, 1
	LDI		R30.w2,	(NIKON_TX_CLK_MODE_FREERUN_STOPHIGH | NIKON_TX_CH1_SEL)
NIKON_CH2_TX_SIZE?:
	QBBC 	NIKON_SKIP_TX_SIZE?, CH_MASK, 2
	LDI 	SCRATCH.w0, ICSS_CFG_PRUx_NIKON_CH2_CFG0+1
	LBCO	&SCRATCH2.b2,	ICSS_CFG,	SCRATCH.w0,	1
    ;In order to clear the previous value of Tx frame size
    AND     SCRATCH2.b2 , SCRATCH2.b2 , 0x7
	OR 		SCRATCH2.b2, SCRATCH2.b2, TX_MODE
	SBCO 	&SCRATCH2.b2, ICSS_CFG, SCRATCH.w0, 1
	LDI		R30.w2,	(NIKON_TX_CLK_MODE_FREERUN_STOPHIGH | NIKON_TX_CH2_SEL)
NIKON_SKIP_TX_SIZE?:
NIKON_SEND_WAIT_TILL_TX_BUSY?:
	LDI     SCRATCH.w0, ICSS_CFG_PRUx_NIKON_TXCFG
	LBCO	&SCRATCH2.b0,	ICSS_CFG,	SCRATCH.w0,		1
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	AND 	SCRATCH2.b0, 	SCRATCH2.b0, 	0xE0
	QBNE 	NIKON_SEND_WAIT_TILL_TX_BUSY?, 	SCRATCH2.b0, 	0
	.else
	QBBC 	NIKON_CH1_TX_BUSY?, CH_MASK, 0
	;Determines when you can assert tx go to issue a new TX frame
    QBBS    NIKON_SEND_WAIT_TILL_TX_BUSY?,	SCRATCH2.b0,	5
NIKON_CH1_TX_BUSY?:
	QBBC 	NIKON_CH2_TX_BUSY?, CH_MASK, 1
    QBBS    NIKON_SEND_WAIT_TILL_TX_BUSY?,	SCRATCH2.b0,	6
NIKON_CH2_TX_BUSY?:
	QBBC 	NIKON_SKIP_TX_BUSY?, CH_MASK, 2
	;Determines when you can assert tx go to issue a new TX frame
    QBBS    NIKON_SEND_WAIT_TILL_TX_BUSY?,	SCRATCH2.b0,	7
	.endif
NIKON_SKIP_TX_BUSY?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	MOV 	SCRATCH3.b0, R30.b2
	LOOP 	NIKON_SEND_CDF_MULTI_CHANNEL?, 3
	.endif
	AND 	SCRATCH2.b0, R30.b2, 0x3
	QBBC 	NIKON_SKIP_CDF_CHx?, CH_MASK, SCRATCH2.b0
	;Load pre-computed command into TX FIFO
	MOV 	R30.b0, cdf.b3
	MOV 	R30.b0, cdf.b2
	MOV 	R30.b0, cdf.b1
	MOV 	R30.b0, cdf.b0
NIKON_SKIP_CDF_CHx?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	SUB 	R30.b2, R30.b2, 1
NIKON_SEND_CDF_MULTI_CHANNEL?:
	SET 	R31, NIKON_TX_GLOBAL_GO
	.else
	SET 	R31, NIKON_TX_CHANNEL_GO
	.endif
	QBEQ 	NIKON_SKIP_TX_SEND?, EEPROM_ACCESS_STATUS, NON_EEPROM_CMD
	SUB 	SCRATCH2.b3, SCRATCH2.b3, 1
	QBEQ 	NIKON_NEXT_MDF_CMD?, EEPROM_ACCESS_STATUS, EEPROM_READ_CMD
	LDI 	SCRATCH3.b2, NIKON_MEMORY_DATA0_FRAME_OFFSET
	LDI 	SCRATCH2.b2, 3
NIKON_NEXT_MDF_CMD?:
	M_ENABLE_PRU_CYCLE_COUNTER
	LBCO 	&SCRATCH, PRUx_DMEM, NIKON_CONFIG_DELAY_10US_OFFSET, 4
	ZERO 	&SCRATCH1, 4
	SBCO 	&SCRATCH1, ICSS_PRU_CTRL_CONST, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
NIKON_SKIP_WAIT_TILL_FILL?:

	.if $isdefed("ENABLE_MULTI_CHANNEL")
	MOV 	R30.b2, 	SCRATCH3.b0
	LOOP 	NIKON_SEND_DELAY_MULTI_CHANNEL?, 3
	.endif
	AND 	SCRATCH3.b1, R30.b2, 0x3
	QBBC 	NIKON_SKIP_DELAY_CHx?, CH_MASK, SCRATCH3.b1
	NIKON_WAIT_FOR_FILL_LEVEL
    LDI     R30.b0, 0xFF
NIKON_SKIP_DELAY_CHx?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	SUB 	R30.b2, 	R30.b2, 	1
NIKON_SEND_DELAY_MULTI_CHANNEL?
	.endif										;1's to compensate for 10usec delay
	LBCO 	&SCRATCH1, ICSS_PRU_CTRL_CONST, PRUx_CNTL_CYCLE_COUNT_OFFSET, 4
	QBLE 	NIKON_SKIP_WAIT_TILL_FILL?, SCRATCH, SCRATCH1
	M_DISABLE_PRU_CYCLE_COUNTER
	QBNE 	NIKON_LOAD_READ_MDF?, EEPROM_ACCESS_STATUS, EEPROM_WRITE_CMD
	LBCO 	&mdf, PRUx_DMEM, SCRATCH3.b2, 4
	QBA 	NIKON_SKIP_MDF_LOAD?
NIKON_LOAD_READ_MDF?:
	LBCO 	&mdf, PRUx_DMEM, NIKON_MEMORY_ADDR_FRAME_OFFSET, 4
NIKON_SKIP_MDF_LOAD?:
	LDI 	SCRATCH2.b0, 	0
NIKON_SKIP_MDF_CHx?:
	ADD 	SCRATCH2.b0, SCRATCH2.b0, 1
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	MOV 	R30.b2, SCRATCH3.b0
	LOOP	NIKON_SEND_MDF_EACH_CHx?, 3
	.endif
	AND 	SCRATCH3.b1, R30.b2, 0x3
	QBBC 	NIKON_SKIP_MDF_THIS_CHx?, CH_MASK, SCRATCH3.b1
	NIKON_WAIT_FOR_FILL_LEVEL
    MOV     R30.b0, mdf.b3
NIKON_SKIP_MDF_THIS_CHx?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
	SUB 	R30.b2, R30.b2, 1
NIKON_SEND_MDF_EACH_CHx?:
	.endif
	LSL 	mdf, mdf, 8
	QBNE	NIKON_SKIP_MDF_CHx?, SCRATCH2.b0, 4
	QBNE 	NIKON_SKIP_NEXT_MDF_CMD?, EEPROM_ACCESS_STATUS, EEPROM_WRITE_CMD
	SUB 	SCRATCH2.b2, SCRATCH2.b2, 1
	ADD 	SCRATCH3.b2, SCRATCH3.b2, 4
	QBNE 	NIKON_NEXT_MDF_CMD?, SCRATCH2.b2, 0									;In case of EEPROM write load MDF0-2 with 10usec delay in between
NIKON_SKIP_NEXT_MDF_CMD?:
	QBEQ 	NIKON_SKIP_TX_SEND?, SCRATCH2.b3, 0
	LBCO 	&SCRATCH, PRUx_DMEM, NIKON_CONFIG_DELAY_10US_OFFSET, 4
NIKON_WAIT_LOOP?:
	SUB 	SCRATCH, SCRATCH, 1
	QBNE 	NIKON_WAIT_LOOP?, SCRATCH, 0												;first nikon cycle data is undetermined, start the next cycle after certain delay.
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	M_NIKON_LS_WAIT_FOR_SYNC
	QBBC 	NIKON_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 0
    SET     R31, NIKON_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	M_NIKON_LS_WAIT_FOR_SYNC
	QBBC 	NIKON_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 1
	SET 	R31, NIKON_TX_GLOBAL_REINIT
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	M_NIKON_LS_WAIT_FOR_SYNC
	QBBC 	NIKON_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 2
	SET 	R31, NIKON_TX_GLOBAL_REINIT
	.else
	SET 	R31, NIKON_TX_GLOBAL_REINIT
	.endif
NIKON_SKIP_GLOBAL_REINIT1?:
	QBNE 	NIKON_DELAY_FOR_EEPROM_WRITE?, EEPROM_ACCESS_STATUS, EEPROM_READ_CMD
	LBCO 	&SCRATCH, PRUx_DMEM, NIKON_CONFIG_DELAY_300US_OFFSET, 4				;maximum of 300 microsec required for retrieving from EEPROM.
	QBA 	NIKON_SKIP_DELAY_FOR_WRITE?
NIKON_DELAY_FOR_EEPROM_WRITE?:
	LBCO 	&SCRATCH, PRUx_DMEM, NIKON_CONFIG_DELAY_30MS_OFFSET, 4				;maximum of 30 milisec required for modifying in EEPROM.
NIKON_SKIP_DELAY_FOR_WRITE?:
	LSR 	SCRATCH, SCRATCH, 1
NIKON_WAIT_LOOP1?:
	SUB 	SCRATCH, SCRATCH, 1
	QBNE 	NIKON_WAIT_LOOP1?, SCRATCH, 0
	SBCO 	&SCRATCH, PRUx_DMEM, NIKON_LS_RTU_SYNC_STATUS_OFFSET, 3
	QBNE 	NIKON_START_TX_SEND?, SCRATCH2.b3, 0						;start second cycle to get determined data from EEPROM.
NIKON_SKIP_TX_SEND?:

	LDI 	SCRATCH.b0,  NON_EEPROM_CMD
	SBCO 	&SCRATCH.b0, PRUx_DMEM, NIKON_MEM_ACCESS_STATUS_OFFSET, 1
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_NIKON_LS_WAIT_FOR_SYNC
; 	Loop till all channels are synchronized in load share mode.
; Registers:
;	LS_SYNC_STATE: Holds the execution status of individual PRU in load share.
;	SCRATCH.b1: Holds the combine execution status of all PRUs in load share.
;	SCRATCH.b0: Holds channel mask.
;
;  PseudoCode:
;		(start code)
;		1.Set LS_SYNC_STATE for the PRU which enters this macro.
;		2.Store the execution state at specific PRU offset.
;		3.Load the execution state back for all PRUs(in use) and perform 'or' operation for all execution states in SCRATCH1.b1.
;		4.Check whether execution states and channel mask is equal.
;		5.Repeat step 3 and 4 till all PRUs(in use) execution state is set.
;		(end code)
;
;	 Worst case peak cycle usage: 14
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_NIKON_LS_WAIT_FOR_SYNC 	.macro
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI 	LS_SYNC_STATE, 1
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_RTU_SYNC_STATUS_OFFSET, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI 	LS_SYNC_STATE, (1<<1)
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_PRU_SYNC_STATUS_OFFSET, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI 	LS_SYNC_STATE, (1<<2)
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_TXPRU_SYNC_STATUS_OFFSET, 1
	.endif
	LBCO 	&SCRATCH.b0,  PRUx_DMEM,  NIKON_CHANNEL_CONFIG_OFFSET , 1
NIKON_IS_SYNCED?:
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_RTU_SYNC_STATUS_OFFSET, 1
	MOV		SCRATCH.b1, LS_SYNC_STATE
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_PRU_SYNC_STATUS_OFFSET, 1
	OR		SCRATCH.b1, SCRATCH.b1, LS_SYNC_STATE
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, NIKON_LS_TXPRU_SYNC_STATUS_OFFSET, 1
	OR		SCRATCH.b1, SCRATCH.b1, LS_SYNC_STATE
	QBNE	NIKON_IS_SYNCED?, SCRATCH.b1, SCRATCH.b0
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: NIKON_WAIT_FOR_FILL_LEVEL
;	Wait till Tx fifo fill level is One byte.
; Registers:
;			SCRATCH3.b1 - Holds selected channel R30.b2.
;			SCRATCH2.b1 - Holds masked Tx fifo fill level from R31.bx[2-4].
;			R31.bx 		- Current Tx fifo fill level.
; PseudoCode:
;		(start code)
;		1.Check the channel selected.
;		2.Load the Tx fifo fill level bits R31.bx[2-4].
;		3.Repeat step 1 and 2 untill Tx fifo fill level is 1 byte.
; 		4.Repeat step 1-3 for all configured channels.
;		(end code)
;	 Worst case peak cycle usage: 9
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

NIKON_WAIT_FOR_FILL_LEVEL 	.macro
NIKON_WAIT_TILL_FILL?:
	QBNE	NIKON_CH1_FILL?, SCRATCH3.b1, 0
    AND     SCRATCH2.b1, R31.b0, POLLING_MASK
	QBLT    NIKON_WAIT_TILL_FILL?, SCRATCH2.b1, ONE_BYTE_FIFO_LEVEL
NIKON_CH1_FILL?:
	QBNE	NIKON_CH2_FILL?, SCRATCH3.b1, 1
	AND 	SCRATCH2.b1, R31.b1, POLLING_MASK
	QBLT    NIKON_WAIT_TILL_FILL?, SCRATCH2.b1, ONE_BYTE_FIFO_LEVEL
NIKON_CH2_FILL?:
	QBNE	NIKON_SKIP_CH2_FILL?, SCRATCH3.b1, 2
	AND 	SCRATCH2.b1, R31.b2, POLLING_MASK
	QBLT    NIKON_WAIT_TILL_FILL?, SCRATCH2.b1, ONE_BYTE_FIFO_LEVEL
NIKON_SKIP_CH2_FILL?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: NIKON_16MHZ_POST_PROCESSING
; 	For 16MHz as operating frequency calculate 8 bit CRC on received data
; Registers:
;	RAW_DATA: Holds the 8-bit CRC and resultant value for every iteration.
;	SCRATCH3: Holds offsets for DMEM locations
;	SCRATCH:  Holds CRC polynomial
; 	NUM_FRAMES: Holds the count of Rx frame utilized in CRC calculations
; 	CH_IN_USE: Holds current channel in use
;
;  PseudoCode:
;		(start code)
;		1.Load the first Rx frames for first encoder and first channel
;		2.Perform XOR with CRC polynomial by alinging MSB using LMBD
;		3.Repeat Step 2 untill RAW_DATA content is len than CRC polynomial
;		4.Load next frame untill NUM_FRAME is last frame
;		5.If current frame is last frame RAW_DATA have calculated CRC.
; 		6.Compare received CRC with calculated CRC and increment error count if does not match.
; 		7.Repeat Step 1 - 6 for all selected channel by incrementing channel in use.
;		8.Repeat Step 1 - 7 for all encoders connected in bus by decrementing number of encoders remains.
;		(end code)
;
;	 Worst case peak cycle usage: 40
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

NIKON_16MHZ_POST_PROCESSING 	.macro
	;16MHz Post Processing
	ZERO 	&RAW_DATA, 4
	LDI 	ENCODER_OFFSET, NIKON_POSITION_DATA_ENC0_RES_OFFSET
	LBCO 	&NUM_ENCODERS, PRUx_DMEM, NIKON_NUMBER_OF_ENCODERS_OFFSET, 1
NIKON_CALC_CRC_NEXT_ENC?:
	.if $isdefed("ENABLE_MULTI_CHANNEL")
;Increment channel in use in case of multi channel
	LDI 	CH_IN_USE, 0
	.else
	QBBC 	NIKON_CH1_CALC_CRC?, CH_MASK, 0
	LDI 	CH_IN_USE, 0
NIKON_CH1_CALC_CRC?:
	QBBC 	NIKON_CH2_CALC_CRC?, CH_MASK, 1
	LDI 	CH_IN_USE, 1
NIKON_CH2_CALC_CRC?:
	QBBC 	NIKON_SKIP_CALC_CRC?, CH_MASK, 2
	LDI 	CH_IN_USE, 2
NIKON_SKIP_CALC_CRC?:
	.endif
NIKON_CALC_CRC_NEXT_CH?:
	LBCO 	&NUM_FRAMES, PRUx_DMEM, NIKON_NUMBER_OF_RX_FRAMES_OFFSET, 1
	QBBC 	NIKON_SKIP_CALC_CRC_THIS_CH?, CH_MASK, CH_IN_USE
	QBNE 	NIKON_THIS_CH_IS_CH1?, CH_IN_USE, 0										;load channel 0 offsets
	ADD 	RAW_DATA_OFFSET, ENCODER_OFFSET, NIKON_INFO_FIELD_CH0_OFFSET
	ADD 	SCRATCH3.b1, ENCODER_OFFSET, NIKON_POSITION_DATA_OTF_CRC_CH0_OFFSET
	ADD 	SCRATCH3.b2, ENCODER_OFFSET, NIKON_POS_DATA_CH0_CRC_ERROR_COUNT
NIKON_THIS_CH_IS_CH1?:
	QBNE 	NIKON_THIS_CH_IS_CH2?, CH_IN_USE, 1										;load channel 1 offsets
	ADD 	RAW_DATA_OFFSET, ENCODER_OFFSET, NIKON_INFO_FIELD_CH1_OFFSET
	ADD 	SCRATCH3.b1, ENCODER_OFFSET, NIKON_POSITION_DATA_OTF_CRC_CH1_OFFSET
	ADD 	SCRATCH3.b2, ENCODER_OFFSET, NIKON_POS_DATA_CH1_CRC_ERROR_COUNT
NIKON_THIS_CH_IS_CH2?:
	QBNE 	NIKON_SKIP_THIS_CH?, CH_IN_USE, 2										;load channel 2 offsets
	ADD 	RAW_DATA_OFFSET, ENCODER_OFFSET, NIKON_INFO_FIELD_CH2_OFFSET
	ADD 	SCRATCH3.b1, ENCODER_OFFSET, NIKON_POSITION_DATA_OTF_CRC_CH2_OFFSET
	ADD		SCRATCH3.b2, ENCODER_OFFSET, NIKON_POS_DATA_CH2_CRC_ERROR_COUNT
NIKON_SKIP_THIS_CH?:
	LBCO 	&RAW_DATA, PRUx_DMEM, RAW_DATA_OFFSET, 2
	ADD 	RAW_DATA_OFFSET, RAW_DATA_OFFSET, 6
	SUB 	NUM_FRAMES, NUM_FRAMES, 1
	LMBD 	SCRATCH3.b0, RAW_DATA, 1
	QBGT 	NIKON_LOAD_NEXT_WORD?, SCRATCH3.b0, NIKON_RX_CRC_LEN 	;lmbd is less than crc length, load next frame.
	SUB 	SCRATCH3.b0, SCRATCH3.b0, NIKON_RX_CRC_LEN
	LDI 	SCRATCH, NIKON_RX_CRC_POLY
	LSL 	SCRATCH, SCRATCH, SCRATCH3.b0
NIKON_CONTINUE_CALC_CRC?:
	XOR 	RAW_DATA, RAW_DATA, SCRATCH
NIKON_LOAD_NEXT_WORD?:
	LMBD 	SCRATCH3.b0, RAW_DATA, 1
	QBEQ 	NIKON_LMBD_IS_0?, SCRATCH3.b0, 32			;lmbd will result in 32 if input reg contents are 0
	QBLE 	NIKON_SKIP_NEXT_WORD?, SCRATCH3.b0, NIKON_RX_CRC_LEN 	;do not load next frame ,if lmbd >= crc len
NIKON_LMBD_IS_0?:
	QBEQ 	NIKON_END_CAL_CRC?, NUM_FRAMES, 0
	LSL 	RAW_DATA, RAW_DATA, 16						;left shift the current content into second word (RAW_DATA.w2)
	LBCO 	&RAW_DATA.w0, PRUx_DMEM, RAW_DATA_OFFSET, 2 ;load next frame into lower word (RAW_DATA.w0)
	ADD 	RAW_DATA_OFFSET, RAW_DATA_OFFSET, 6			;Increment the offset to next frame dmem offset
	SUB 	NUM_FRAMES, NUM_FRAMES, 1					;decrement number of frames
	QBNE 	NIKON_SKIP_NEXT_WORD?, NUM_FRAMES, 0		;if current frame is last frame then clear crc bits
	MOV 	NIKON_RCV_CRC.b0, RAW_DATA.b0				;store received crc in local register
	LDI 	RAW_DATA.b0, 0								;clear the crc byte
NIKON_SKIP_NEXT_WORD?:
	LMBD 	SCRATCH3.b0, RAW_DATA, 1
	QBNE 	NIKON_DATA_GT_CRC_POLY?, SCRATCH3.b0, NIKON_RX_CRC_LEN
	QBNE 	NIKON_DATA_GT_CRC_POLY?, NUM_FRAMES, 0		;if current frame is last frame and lmbd == crc length
	LDI 	SCRATCH, NIKON_RX_CRC_POLY					;then end the crc calculation after a iteration(len(raw_data) < crc_len)
	XOR 	RAW_DATA, RAW_DATA, SCRATCH
	QBA 	NIKON_END_CAL_CRC?
NIKON_DATA_GT_CRC_POLY?:
	SUB 	SCRATCH3.b0, SCRATCH3.b0, NIKON_RX_CRC_LEN	;subtract crc length from lmbd to get shift value for crc poly
	LDI 	SCRATCH, NIKON_RX_CRC_POLY
	QBGT 	NIKON_DATA_LT_CRC_POLY?, RAW_DATA, SCRATCH
	LSL 	SCRATCH, SCRATCH, SCRATCH3.b0				;shift crc polynomial to match the leading 1's indexes
	QBA 	NIKON_CONTINUE_CALC_CRC?
NIKON_DATA_LT_CRC_POLY?:
	QBNE 	NIKON_CONTINUE_CALC_CRC?, NUM_FRAMES, 0		;continue the CRC calculation untill all Rx frames are used
NIKON_END_CAL_CRC?:
	SBCO 	&RAW_DATA, PRUx_DMEM, SCRATCH3.b1, 1
	ADD 	SCRATCH3.b1, SCRATCH3.b1, 3
	SBCO 	&NIKON_RCV_CRC.b0, PRUx_DMEM, SCRATCH3.b1, 1
	MOV 	NIKON_OTF_CRC.b0, RAW_DATA.b0
	QBEQ 	NIKON_SKIP_CALC_CRC_THIS_CH?, NIKON_OTF_CRC.b0, NIKON_RCV_CRC.b0
	LBCO 	&SCRATCH, PRUx_DMEM, SCRATCH3.b2, 4
	ADD 	SCRATCH, SCRATCH, 1
	SBCO 	&SCRATCH, PRUx_DMEM, SCRATCH3.b2, 4
NIKON_SKIP_CALC_CRC_THIS_CH?:
	ADD 	CH_IN_USE , CH_IN_USE, 1
	QBNE 	NIKON_CALC_CRC_NEXT_CH?, CH_IN_USE, 3
	SUB 	NUM_ENCODERS, NUM_ENCODERS, 1
	ADD 	ENCODER_OFFSET, ENCODER_OFFSET, (NIKON_POSITION_DATA_ENC1_RES_OFFSET - NIKON_POSITION_DATA_ENC0_RES_OFFSET)
	QBNE 	NIKON_CALC_CRC_NEXT_ENC?, NUM_ENCODERS, 0

	.endm
