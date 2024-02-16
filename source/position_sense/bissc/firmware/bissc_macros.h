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
;*	 File:	bissc_macros.h															*
;*																					*
;*	 Brief: Macros for calculating On the Fly CRC of position data and returning	*
;*	 		the CRC status with RAW data											*
;*	       	Macros for call and return(branching) 									*
;*          Macro for enabling pru cycle counter 									*
;*	       	Macro for Measuring processing delay 	
;*			Macro for synchronizing PRUs in load share.								*
;************************************************************************************


	.include "bissc_icss_reg_defs.h"
	.include "bissc_params.h"
	.include "bissc_interface.h"
	.include "firmware_version.h"

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
;		2.Returns to address stored in SCRTACH5.w0 or SCRATCH.w2 when RET or RET2 macro is called.
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
; Macro: M_ENABLE_PRU_CYCLE_COUNTER
; 	Enable PRU cycle counter.
; Registers: 
;	SCRATCH2: Holds PRU control configurations.
;
;  PseudoCode:
;	 (start code)
;		1.Load the ICSS_PRU_CTRL_CONST to SCRATCH2 and SET the 3rd bit to enable PRU cycle counter.
;		2.Store the updated SCRATCH2 at ICSS_PRU_CTRL_CONST offset in shared memory.
;	 (end code)
;
;	 Worst case peak cycle usage: 13
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_ENABLE_PRU_CYCLE_COUNTER .macro
	LBCO	&SCRATCH2, ICSS_PRU_CTRL_CONST, 0, 4
	SET 	SCRATCH2, SCRATCH2, 3
	SBCO	&SCRATCH2, ICSS_PRU_CTRL_CONST, 0, 4
  .endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_BISSC_LS_WAIT_FOR_SYNC
; 	Loop till all channels are synchronized in load share mode.
; Registers: 
;	LS_SYNC_STATE: Holds the execution status of individual PRU in load share.
;	SCRATCH1.b1: Holds the combine execution status of all PRUs in load share.
;	SCRATCH2.b0: Holds channel mask.
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
M_BISSC_LS_WAIT_FOR_SYNC 	.macro
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LDI 	LS_SYNC_STATE, 1
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_RTU_STATE, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LDI 	LS_SYNC_STATE, (1<<1)
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_PRU_STATE, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LDI 	LS_SYNC_STATE, (1<<2)
	SBCO 	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_TXPRU_STATE, 1
	.endif
	LBCO 	&SCRATCH1.b0,  PRUx_DMEM,  BISSC_CHANNEL_CONFIG_OFFSET , 1
BISSC_IS_SYNCED?:
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_RTU_STATE, 1
	MOV		SCRATCH1.b1, LS_SYNC_STATE
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_PRU_STATE, 1
	OR		SCRATCH1.b1, SCRATCH1.b1, LS_SYNC_STATE
	LBCO	&LS_SYNC_STATE, PRUx_DMEM, BISSC_LS_EXEC_TXPRU_STATE, 1
	OR		SCRATCH1.b1, SCRATCH1.b1, LS_SYNC_STATE
	QBNE	BISSC_IS_SYNCED?, SCRATCH1.b1, SCRATCH1.b0
	.endm
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_BISSC_CLK_CONFIG
; 	Configure Rx, Tx frame sizes at ICSS_CFG_PRUx_BISSC_CHx_CFGx offset specific to channels in use.
; Registers: 
;	mask: Holds the channel mask.
;	reg: Holds the frame sizes.
;	BISSC_ICSSG_CHx_CFG0: Holds the confiugation register offset as per channel.
;
;  PseudoCode:
;		(start code)
;		1.Check the channel mask bits for respective numbered channel.
;		2.Hold the offset for channel in use.
;		3.Store the reg for the enabled channel at specified offset.
;		(end code)
;
;	 Worst case peak cycle usage: 18
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_BISSC_CLK_CONFIG .macro mask, reg
	QBBC	BISSC_CLK_CFG_CH1?, mask, 0
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH0_CFG0
	SBCO 	&reg, ICSS_CFG, BISSC_ICSSG_CHx_CFG0,  4 	
BISSC_CLK_CFG_CH1?:
	QBBC	BISSC_CLK_CFG_CH2?, mask, 1
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH1_CFG0
	SBCO 	&reg, ICSS_CFG, BISSC_ICSSG_CHx_CFG0,  4 	
BISSC_CLK_CFG_CH2?:
	QBBC	BISSC_SKIP_CLK_CFG?, mask, 2
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH2_CFG0
	SBCO 	&reg, ICSS_CFG, BISSC_ICSSG_CHx_CFG0,  4 
BISSC_SKIP_CLK_CFG?:
	.endm
;*************************************************************************************************************************************
;	 Macro: M_MEASURE_MAX_PROC_TIME
;			 Measure the Processing Delay and compensate in the Rx frame size.
;	 Registers:
;			SCRATCH3.w2 - Holds Rx frame size
;			SCRATCH3.w0 - Holds Tx frame size
;			SCRATCH.w2  - Holds Processing delay for one BissC Cycle
;			SCRATCH.w0  - Holds Maximum time to wait for encoder to respond then holds maximum processing delay acceptable. 
;			SCRATCH2.w0 - Stores the average processing delay for 8 BissC cycles
;			SCRATCH3.b0 - Holds processing delay remeasurement status
;	PseudoCode:
;		(start code)
;		1.Check for the channel in use and load the register with channel specific parameters.
;		2.Load the clock mode for selected channels by writing to R30.w2
;		3.Load the Rx frame size to maximum and decrement upon detecting the valid bits and Tx frame size to 0.
;		4.Increment the SCRATCH.w2 until start bit is detected then add it to SCRATCH2.w0.
;		5.If it exceeds the maximum processing delay allowed, then set proc delay remeasure flag and restart the loop from first iteration. 
;		6.Loop the steps 1-4 for 8 times and compensate the average in Rx frame size.
;		7.If step 5 is encountered the repeat step 6.
;		(end code)
;	
;   Worst case peak cycle usage:
;**************************************************************************************************************************************

M_MEASURE_MAX_PROC_TIME .macro
	QBBC	BISSC_IS_CH1_IN_USE?, CH_MASK, 0
	LDI 	BISSC_ENABLE_CHx_IN_USE, 0
	LDI 	VALID_BIT_IDX, 	BISSC_CH0_VALID_BIT_IDX
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH0_CFG0
	QBA		BISSC_PROC_MEASURE_CHN_SEL_CMPLT?
BISSC_IS_CH1_IN_USE?:
	QBBC 	BISSC_IS_CH2_IN_USE?, CH_MASK, 1
	LDI		BISSC_ENABLE_CHx_IN_USE, 1
	LBCO 	&FIFO_BIT_IDX, PRUx_DMEM, BISSC_FIFO_BIT_IDX_OFFSET, 1
	ADD		FIFO_BIT_IDX, FIFO_BIT_IDX, 8
	LDI 	VALID_BIT_IDX, 	BISSC_CH1_VALID_BIT_IDX
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH1_CFG0
	QBA 	BISSC_PROC_MEASURE_CHN_SEL_CMPLT?
BISSC_IS_CH2_IN_USE?:
	QBBC 	BISSC_END_PROC_MEASURE_PER_CH?, CH_MASK, 2
	LDI 	BISSC_ENABLE_CHx_IN_USE, 2
	LBCO 	&FIFO_BIT_IDX, PRUx_DMEM, BISSC_FIFO_BIT_IDX_OFFSET, 1
	ADD		FIFO_BIT_IDX, FIFO_BIT_IDX, 16
	LDI 	VALID_BIT_IDX, 	BISSC_CH2_VALID_BIT_IDX
	LDI 	BISSC_ICSSG_CHx_CFG0, ICSS_CFG_PRUx_BISSC_CH2_CFG0
BISSC_PROC_MEASURE_CHN_SEL_CMPLT?:
BISSC_PROC_MEASURE_AGAIN?:
	LDI 	SCRATCH2.w0, 0
	LOOP	BISSC_PROC_DELAY_END?,	8
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	M_BISSC_LS_WAIT_FOR_SYNC
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	M_BISSC_LS_WAIT_FOR_SYNC
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	M_BISSC_LS_WAIT_FOR_SYNC
	.endif	
    QBNE    BISSC_IS_CH1_SEL?, BISSC_ENABLE_CHx_IN_USE,	0
	LDI		R30.w2,	(BISSC_TX_CLK_MODE_FREERUN_STOPHIGH | BISSC_TX_CH0_SEL)
	LDI 	R30.b0, 0
BISSC_IS_CH1_SEL?:
    QBNE    BISSC_IS_CH2_SEL?, BISSC_ENABLE_CHx_IN_USE,	1
	LDI		R30.w2,	(BISSC_TX_CLK_MODE_FREERUN_STOPHIGH | BISSC_TX_CH1_SEL)
	LDI 	R30.b0, 0
BISSC_IS_CH2_SEL?:
    QBNE    BISSC_SKIP_CH_SEL?, BISSC_ENABLE_CHx_IN_USE,	2
	LDI		R30.w2,	(BISSC_TX_CLK_MODE_FREERUN_STOPHIGH | BISSC_TX_CH2_SEL)
	LDI 	R30.b0, 0
BISSC_SKIP_CH_SEL?:
	; loading rx frame size to maximum bits we can receive from BiSSC encoder
    LDI     SCRATCH3.w2, BISSC_MAX_FRAME_SIZE
	LDI		SCRATCH3.w0, 0 
	; store Rx and Tx frame size to ICSS_CFG_PRUx_ED_CHx for all configured channels
    SBCO	&SCRATCH3, ICSS_CFG, BISSC_ICSSG_CHx_CFG0,	4	
BISSC_PROC_WAIT_TILL_TX_CH_BUSY?
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	QBBC 	BISSC_SKIP_TX_CH_BUSY?, PRIMARY_CORE, 0
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_BISSC_TXCFG
	LBCO	&SCRATCH3.b0, ICSS_CFG,	SCRATCH1.w0, 1	
	QBBS	BISSC_PROC_WAIT_TILL_TX_CH_BUSY?, SCRATCH3.b0,	5
	LDI		SCRATCH3.b0, 0
	SBCO	&SCRATCH3.b0, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	LDI 	R30.b0, 0
	SET	    R31, BISSC_TX_GLOBAL_GO
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	QBBC 	BISSC_SKIP_TX_CH_BUSY?, PRIMARY_CORE, 1
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_BISSC_TXCFG
	LBCO	&SCRATCH3.b0, ICSS_CFG, SCRATCH1.w0,	1
	QBBS	BISSC_PROC_WAIT_TILL_TX_CH_BUSY?, SCRATCH3.b0, 6
	LDI		SCRATCH3.b0, 0
	SBCO	&SCRATCH3.b0, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	LDI 	R30.b0, 0
	SET	    R31, BISSC_TX_GLOBAL_GO
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	QBBC 	BISSC_SKIP_TX_CH_BUSY?, PRIMARY_CORE, 2
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_BISSC_TXCFG
	LBCO	&SCRATCH3.b0,	ICSS_CFG,	SCRATCH1.w0,	1
	QBBS	BISSC_PROC_WAIT_TILL_TX_CH_BUSY?,	SCRATCH3.b0,	7
	LDI		SCRATCH3.b0, 0
	SBCO	&SCRATCH3.b0, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	LDI 	R30.b0, 0
	SET	    R31, BISSC_TX_GLOBAL_GO
	.else
	LDI     SCRATCH1.w0, ICSS_CFG_PRUx_BISSC_TXCFG
	LBCO	&SCRATCH3.b0, ICSS_CFG, SCRATCH1.w0, 1
	QBBS	BISSC_PROC_WAIT_TILL_TX_CH_BUSY?, SCRATCH3.b0, 5
	LDI 	R30.b0, 0
	SET	    R31, BISSC_TX_GLOBAL_GO
	.endif
BISSC_SKIP_TX_CH_BUSY?:
    SET     R30, R30, BISSC_RX_EN
	;Max number to come out of infinite loop of waiting for encoder detected.
	;Using emperical number as spec did not mentioned about anything 
	LDI     SCRATCH.w0, BISSC_MAX_WAIT_FOR_ENC_DETECT
	LDI     SCRATCH.w2, 0

BISSC_ACK_BIT?:
    ;  wait for valid bit
    QBBC    BISSC_ACK_BIT?,  R31, VALID_BIT_IDX
	ADD     SCRATCH.w2,SCRATCH.w2,1
    QBLT	BISSC_END_PROC_ENC_NOT_DETECTED?, SCRATCH.w2, SCRATCH.w0
	MOV		SCRATCH1, R31
	;Set the valid bit for clearing flag so that next bit can come.
	SET     R31, R31, BISSC_RX_VALID
	;must maintain 3 cycles before checking next valid bit
	NOP
	NOP
	QBBS	BISSC_ACK_BIT?, SCRATCH1, FIFO_BIT_IDX
	SUB		SCRATCH3.w2, SCRATCH3.w2, 1
	LBCO    &SCRATCH.w0, PRUx_DMEM, BISSC_MAX_PROC_DELAY_OFFSET, 2
	LDI     SCRATCH.w2,0

	;start bit
BISSC_START_BIT?:
    ;  wait for valid bit
    QBBC    BISSC_START_BIT?,  R31, VALID_BIT_IDX
	ADD     SCRATCH.w2,SCRATCH.w2,1
	QBLT	BISSC_END_PROC_MAX_PROC_TIME_EXCEEDED?, SCRATCH.w2, SCRATCH.w0
	MOV		SCRATCH1, R31
	;Set the valid bit for clearing flag so that next bit can come.
	SET     R31, R31, BISSC_RX_VALID
	;must maintain 3 cycles before checking next valid bit
	NOP
	NOP
	QBBC	BISSC_START_BIT?, SCRATCH1, FIFO_BIT_IDX
	SUB		SCRATCH3.w2, SCRATCH3.w2, 1
	;Accumulate proc time for this iteration
	ADD		SCRATCH2.w0, SCRATCH2.w0, SCRATCH.w2
	ZERO 	&SCRATCH1, 4
	SBCO	&SCRATCH1, PRUx_DMEM, BISSC_LS_EXEC_RTU_STATE, 4
	;loop here for all data bits
BISSC_LOOP_OVER_BITS?:
    ;  wait for valid bit
    QBBC    BISSC_LOOP_OVER_BITS?,  R31, VALID_BIT_IDX
	SET     R31, R31, BISSC_RX_VALID
	SUB		SCRATCH3.w2, SCRATCH3.w2, 1
	;must maintain 3 cycles before checking next valid bit
	NOP
	NOP
	QBNE	BISSC_LOOP_OVER_BITS?, SCRATCH3.w2, 0

BISSC_PROC_WAIT_FOR_ALL_CORES?:
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LBCO 	&SCRATCH3, PRUx_DMEM, BISSC_LS_EXEC_RTU_STATE, 1
	QBNE	BISSC_PROC_WAIT_FOR_ALL_CORES?, SCRATCH3.b0, 0
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 0
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
BISSC_SKIP_GLOBAL_REINIT1?:
	LBCO 	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBEQ	BISSC_PROC_MEASURE_AGAIN?, SCRATCH2.b3, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LBCO 	&SCRATCH3, PRUx_DMEM, BISSC_LS_EXEC_PRU_STATE, 1
	QBNE	BISSC_PROC_WAIT_FOR_ALL_CORES?, SCRATCH3.b0, 0
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 1
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
BISSC_SKIP_GLOBAL_REINIT1?:
	LBCO 	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBEQ	BISSC_PROC_MEASURE_AGAIN?, SCRATCH2.b3, 1
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LBCO 	&SCRATCH3, PRUx_DMEM, BISSC_LS_EXEC_TXPRU_STATE, 1
	QBNE	BISSC_PROC_WAIT_FOR_ALL_CORES?, SCRATCH3.b0, 0
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT1?, PRIMARY_CORE, 2
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
BISSC_SKIP_GLOBAL_REINIT1?:
	LBCO 	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBEQ	BISSC_PROC_MEASURE_AGAIN?, SCRATCH2.b3, 1
	.else
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	.endif

	LBCO     &SCRATCH3, PRUx_DMEM, BISSC_CONFIG_DELAY_100MS_OFFSET, 4
BISSC_DELAY_LOOP?:
	SUB		SCRATCH3, SCRATCH3, 1
	QBNE	BISSC_DELAY_LOOP?, SCRATCH3, 0	
	NOP
BISSC_PROC_DELAY_END?:
	LSR		SCRATCH2.w0, SCRATCH2.w0, 3
	QBA		BISSC_END_PROC_MEASURE?
BISSC_END_PROC_ENC_NOT_DETECTED?:
	
	ZERO 	&SCRATCH1, 4
	SBCO	&SCRATCH1, PRUx_DMEM, BISSC_LS_EXEC_RTU_STATE, 4
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT2?, PRIMARY_CORE, 0
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT2?, PRIMARY_CORE, 1
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	M_BISSC_LS_WAIT_FOR_SYNC
	QBBC	BISSC_SKIP_GLOBAL_REINIT2?, PRIMARY_CORE, 2
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	.else
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	.endif
BISSC_SKIP_GLOBAL_REINIT2?:
	;Incase encoder is not detected you will reach here.
	HALT

BISSC_END_PROC_MAX_PROC_TIME_EXCEEDED?:
	.if $isdefed("ENABLE_MULTI_MAKE_RTU")
	LBCO    &SCRATCH, PRUx_DMEM, BISSC_CONFIG_DELAY_100MS_OFFSET, 4	;100ms delay loop
BISSC_DELAY_LOOP2?:
	SUB		SCRATCH, SCRATCH, 1
	QBNE	BISSC_DELAY_LOOP2?, SCRATCH, 0
	LDI 	SCRATCH2.b3, 1
	SBCO	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBA 	BISSC_PROC_WAIT_FOR_ALL_CORES?
	.elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
	LBCO    &SCRATCH, PRUx_DMEM, BISSC_CONFIG_DELAY_100MS_OFFSET, 4	;100ms delay loop
BISSC_DELAY_LOOP3?:
	SUB		SCRATCH, SCRATCH, 1
	QBNE	BISSC_DELAY_LOOP3?, SCRATCH, 0
	LDI 	SCRATCH2.b3, 1
	SBCO	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBA 	BISSC_PROC_WAIT_FOR_ALL_CORES?
	.elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
	LBCO    &SCRATCH, PRUx_DMEM, BISSC_CONFIG_DELAY_100MS_OFFSET, 4	;100ms delay loop
BISSC_DELAY_LOOP4?:
	SUB		SCRATCH, SCRATCH, 1
	QBNE	BISSC_DELAY_LOOP4?, SCRATCH, 0
	LDI 	SCRATCH2.b3, 1
	SBCO	&SCRATCH2.b3, PRUx_DMEM, BISSC_RE_MEASURE_PROC_DELAY, 1
	QBA 	BISSC_PROC_WAIT_FOR_ALL_CORES?
	.else
	SET 	R31, R31, 19 ;BISSC_TX_GLOBAL_REINIT ; Set TX_EN low
	; rx_en = 0 : Disable RX mode
	LDI 	R30.b3, 0
	LDI		SCRATCH2.w0, 0
	;Delay loop
	LBCO    &SCRATCH, PRUx_DMEM, BISSC_CONFIG_DELAY_100MS_OFFSET, 4	;100ms delay loop
BISSC_DELAY_LOOP1?:
	SUB		SCRATCH, SCRATCH, 1
	QBNE	BISSC_DELAY_LOOP1?, SCRATCH, 0
	QBA		BISSC_PROC_MEASURE_AGAIN?
	.endif
BISSC_END_PROC_MEASURE?:
	QBEQ 	BISSC_STORE_CH0_PROC_DELAY?, BISSC_ENABLE_CHx_IN_USE, 0
	QBEQ	BISSC_STORE_CH1_PROC_DELAY?, BISSC_ENABLE_CHx_IN_USE, 1
	SBCO	&SCRATCH2.w0, PRUx_DMEM, BISSC_CH2_PROC_DELAY_OFFSET, 2
	JMP		BISSC_END_PROC_MEASURE_PER_CH?
BISSC_STORE_CH0_PROC_DELAY?:
	SBCO	&SCRATCH2.w0, PRUx_DMEM, BISSC_CH0_PROC_DELAY_OFFSET, 2
	JMP		BISSC_IS_CH1_IN_USE?
BISSC_STORE_CH1_PROC_DELAY?:
	SBCO	&SCRATCH2.w0, PRUx_DMEM, BISSC_CH1_PROC_DELAY_OFFSET, 2
	JMP		BISSC_IS_CH2_IN_USE?
BISSC_END_PROC_MEASURE_PER_CH?:
	LDI		SCRATCH1.b1, 0
	SBCO	&SCRATCH1.b1, PRUx_DMEM, BISSC_MEAS_PROC_DELAY_OFFSET, 1
	.endm



; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC
; 	Receive and Downample the position data and do not invoke for CRC bits.
; Registers: 
;	Rx: Holds the raw data received - caller has to zero or provide value to continue
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo. 
;	cnt: Number of bits to receive which should be less than less than 32 as we are returning only one register Rx
;	ex: Temporary variable
;
;  PseudoCode:
;		(start code)
;		1.Left shift the Rx and Wait for valid bit then check the Middle Bit of Rx fifo and store the same in Rx.
;		2.Do the On the fly crc algorithm for received bit.
;		3.Loop through the steps 1-2 for cnt iterations.
;		(end code)
;
;	 Worst case peak cycle usage: 14(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	.macro	Rx, cnt, bit_idx, ff0, ff1, ff2, ff3, ff4, ff5, ex

	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
WB1?:
	AND 	SCRATCH.b0, R31.b3, CH_MASK
	QBNE	WB1?, SCRATCH.b0, CH_MASK ;  wait for valid
	QBBS	BISSC_SET_POS_BIT_CHx?, R31, bit_idx	;  Check the Mid bit of received oversampled data
	MOV		ex,	ff5	; if code[i] = 0, ex = ff5 (ex = ff5 ^ 0)
	JMP		BISSC_SKIP_POS_BIT_CHx?
BISSC_SET_POS_BIT_CHx?:
	NOT		ex,	ff5	; if code[i] = 1, ex = !ff5 (ex = ff5 ^ 1)
	OR		Rx,	Rx,	1
BISSC_SKIP_POS_BIT_CHx?:
	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
	MOV		ff5,	ff4
	MOV		ff4,	ff3
	MOV		ff3,	ff2
	MOV		ff2,	ff1
	XOR		ff1,	ff0,	ex
	MOV		ff0,	ex
BISSC_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE
; 	Receive and Downample the CRC bits.
; Registers: 
;	Rx: Holds the raw data received - caller has to zero or provide value to continue
;	Ry: Holds received CRC
;	valid_bit: Holds the valid bit for selected channel.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo. 
;	cnt: Number of bits to receive which should be 6 as we are receiving the CRC.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry and Wait for valid bit then check the Middle Bit of Rx fifo and store the same in Rx and Ry.
;		2.Clear the valid bit 
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 10(each iteration)
; 
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE	.macro	Rx, Ry, cnt, bit_idx
	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
WB2?:
	AND 	SCRATCH.b0, R31.b3, CH_MASK
	QBNE	WB2?, SCRATCH.b0, CH_MASK ;  wait for valid
	QBBS	BISSC_SET_CRC_BIT_CHx?,	R31,	bit_idx ;  Check the Mid bit of received oversampled data
	QBA		BISSC_SKIP_CRC_BIT_CHx?
BISSC_SET_CRC_BIT_CHx?:
	OR		Rx,	Rx,	1
	OR		Ry,	Ry,	1
BISSC_SKIP_CRC_BIT_CHx?:
	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_CALC_CRC
; 	Store and return the calculated CRC.
; Registers: 
;	ff[0-5]: Holds calculated CRC bits separately
;	crc: Returns the overall CRC computation value. 
;
;  	PseudoCode:
;      (start code)
;       1.Load crc with 0
;		2.Check for individual bit(bit-0) from ff[0-5]
;		3.Set the bits required to be high in crc
;       4.Repeat step 2 and 3 for 6 ff.
;       (end code)
;
;	Worst case peak cycle usage: 13
; 
 ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_CALC_CRC	.macro	crc, ff0, ff1, ff2, ff3, ff4, ff5
	LDI		crc,	0
	QBBS	BISSC_CRC_BIT1?,	ff0,	0
	SET		crc,	crc,	0
BISSC_CRC_BIT1?:
	QBBS	BISSC_CRC_BIT2?,	ff1,	0
	SET		crc,	crc,	1
BISSC_CRC_BIT2?:
	QBBS	BISSC_CRC_BIT3?,	ff2,	0
	SET		crc,	crc,	2
BISSC_CRC_BIT3?:
	QBBS	BISSC_CRC_BIT4?,	ff3,	0
	SET		crc,	crc,	3
BISSC_CRC_BIT4?:
	QBBS	BISSC_CRC_BIT5?,	ff4,	0
	SET		crc,	crc,	4
BISSC_CRC_BIT5?:
	QBBS	BISSC_CRC_END?,		ff5,	0
	SET		crc,	crc,	5 
BISSC_CRC_END?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE
; 	Receive and downsaple the raw data then calculate the CRC.
; Registers: 
;	Ra, Rb: Holds the raw data received - caller has to zero or provide value to continue
;	SCRATCH3.b3: rx_size 
;	RAW_DATA_LEN: position data length with crc and error warning length
;	ff[0-5]: FF0to3_0, FF4_0, FF5_0 - otf crc calculation. 
;	BISSC_RCV_CRC: used for ex and returns the otf crc results.
;
;  PseudoCode:
;	(start code)
;       1.Load ff[0-5] to 0 .
;       2.Receive position data Ra and Rb with respect to the data length and Register length by calling onto 
;		  M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC.
;		3.Receive CRC bits into Rb and BISSC_RCV_CRC_0.
;		4.Load the Calculated CRC result in SCRATCH3.b3 by calling onto M_CALC_CRC.
;		5.Store the position data and crc to thier respective offsets and check for CRC validation between BISSC_RCV_CRC_0 and SCRATCH3.b3.
;       6.Increment the error counter if CRC not matched.
;	(end code)
;
;	 Worst case peak cycle usage: 920
; REVISIT: More optimization may be required or can be done at least in M_INIT_CRC_FLIP_FLOPS - defer those till a requirement comes
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE	.macro	Ra, Rb, encoder_offset
	ZERO	&FF0, 6
	; initialize CRC flip-flops, i.e. ff[0-5] = 0
	SUB		SCRATCH3.b3, RAW_DATA_LEN, 6						; pos data + e+ w
	; receive data
	QBGE	BISSC_RX_LE_32_BITS?,	SCRATCH3.b3,	32
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra,	32, FIFO_BIT_IDX, FF0.b0, FF0.b1, FF0.b2, FF0.b3, FF1.b0, FF1.b1, EX.b0
	SUB		SCRATCH3.b3,	RAW_DATA_LEN,	(32 + 6)	; note taking care of CRC bits
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Rb, SCRATCH3.b3, FIFO_BIT_IDX, FF0.b0, FF0.b1, FF0.b2, FF0.b3, FF1.b0, FF1.b1, EX.b0
	LDI		BISSC_RCV_CRC.b0,	0	; clear crc_recvd
	LDI		SCRATCH3.b3,	6	; CRC length, assuming 6 bit crc
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb,BISSC_RCV_CRC.b0, SCRATCH3.b3, FIFO_BIT_IDX
	JMP		BISSC_RX_CRC?

BISSC_RX_LE_32_BITS?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC	Ra, SCRATCH3.b3, FIFO_BIT_IDX, FF0.b0, FF0.b1, FF0.b2, FF0.b3, FF1.b0, FF1.b1, EX.b0
	LDI		BISSC_RCV_CRC.b0,	0	; clear crc_recvd
	RSB		SCRATCH3.b3,	SCRATCH3.b3,	32
	MIN		SCRATCH3.b3,	SCRATCH3.b3,	6	; CRC length, assuming 6 bit crc
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Ra, BISSC_RCV_CRC.b0, SCRATCH3.b3,  FIFO_BIT_IDX
	QBEQ	BISSC_RX_CRC?,	SCRATCH3.b3,	6
	RSB		SCRATCH3.b3,	SCRATCH3.b3,	6	; CRC length, assuming 6 bit crc
	M_OTF_RECEIVE_AND_DOWNSAMPLE	Rb, BISSC_RCV_CRC.b0, SCRATCH3.b3, FIFO_BIT_IDX
	
BISSC_RX_CRC?:
	ADD 	ENCODER_OFFSET, SCRATCH2.b0, SCRATCH2.b3
	SBCO	&Ra, PRUx_DMEM,	ENCODER_OFFSET,	8
	ADD 	ENCODER_OFFSET, SCRATCH2.b0, SCRATCH2.b1
	M_CALC_CRC	SCRATCH3.b3, FF0.b0, FF0.b1, FF0.b2, FF0.b3, FF1.b0, FF1.b1

	QBEQ	BISSC_CRC_SUCCESS?,	SCRATCH3.b3,	BISSC_RCV_CRC.b0	; crc: calculated - SCRATCH3.b3, received - BISSC_RCV_CRC_0
	LBCO	&SCRATCH, PRUx_DMEM, ENCODER_OFFSET, 4
	ADD		SCRATCH, SCRATCH, 1
	SBCO	&SCRATCH, PRUx_DMEM,	ENCODER_OFFSET , 4

BISSC_CRC_SUCCESS?:
	ADD		ENCODER_OFFSET, SCRATCH2.b0, SCRATCH2.b2
	SBCO	&SCRATCH3.b3,	PRUx_DMEM,	ENCODER_OFFSET , 1
	.endm


; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC
; 	Receive and Downample the position data for multi channel and do not invoke for CRC bits.
; Registers: 
;	Rx, Ry, Rz: Holds the raw data received for all channels- caller has to zero or provide value to continue.
;	ff0 - ff5: flip flops for otf crc calculation.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo. 
;	cnt: Number of bits to receive which should be less than less than 32 as we are returning only one register per channel.
;	ex: Temporary variable.
;
;  PseudoCode:
;		(start code)
;		1.Left shift the Rx, Ry, Rz and Wait for valid bit then check the Middle Bit of Rx fifo for all channels and store the same in Rx, Ry, Rz.
;		2.Do the On the fly crc algorithm for received bits.
;		3.Loop through the steps 1-2 for cnt iterations.
;		(end code)
;
;	 Worst case peak cycle usage: 22(each iteration)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC	.macro	Rx, Ry, Rz, cnt, bit_idx, ff0, ff1, ff2, ff3, ff4, ff5, ex
	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL		Rz,	Rz,	1
WB1?:
	AND		SCRATCH.b0,	R31.b3, CH_MASK
	QBNE	WB1?,	SCRATCH.b0,	CH_MASK ;  wait for valid

	QBBS	BISSC_SET_POS_BIT_CH0?,	R31.b0, bit_idx ;  Check the Mid bit of received oversampled data
	MOV		ex.b0,	ff5.b0	; if code[i] = 0, ex = ff5 (ex = ff4 ^ 0)
	JMP		BISSC_SKIP_POS_BIT_CH0?
BISSC_SET_POS_BIT_CH0?:
	NOT		ex.b0,	ff5.b0	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Rx,	Rx,	1
BISSC_SKIP_POS_BIT_CH0?:

	QBBS	BISSC_SET_POS_BIT_CH1?,	R31.b1,	bit_idx ;  Check the Mid bit of received oversampled data
	MOV		ex.b1,	ff5.b1	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		BISSC_SKIP_POS_BIT_CH1?
BISSC_SET_POS_BIT_CH1?:
	NOT		ex.b1,	ff5.b1	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Ry,	Ry,	1
BISSC_SKIP_POS_BIT_CH1?:

	QBBS	BISSC_SET_POS_BIT_CH2?,	R31.b2,	bit_idx ;  Check the Mid bit of received oversampled data
	MOV		ex.b2,	ff5.b2	; if code[i] = 0, ex = ff4 (ex = ff4 ^ 0)
	JMP		BISSC_SKIP_POS_BIT_CH2?
BISSC_SET_POS_BIT_CH2?:
	NOT		ex.b2,	ff5.b2	; if code[i] = 1, ex = !ff4 (ex = ff4 ^ 1)
	OR		Rz,	Rz,	1
BISSC_SKIP_POS_BIT_CH2?:

	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
	MOV		ff5,	ff4
	MOV		ff4,	ff3
	MOV		ff3,	ff2
	MOV		ff2,	ff1
	XOR		ff1,	ff0,	ex
	MOV		ff0,	ex
BISSC_RX_RECEIVE_DOWNSAMPLE_CRC_LOOP?:
	.endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_DOWNSAMPLE_MC
; 	Receive and Downample the CRC bits for multi channel.
; Registers: 
;	Rx, Ry, Rz: Holds the raw data received for all channels - caller has to zero or provide value to continue
;	Ra: Holds received CRC for all channels.
; 	bit_idx: Holds the bit index(middle bit of oversampled data) of rx fifo. 
;	cnt: Number of bits to receive which should be 6 as we are receiving the CRC.
;
;  PseudoCode:
;	 (start code)
;		1.Left shift Rx, Ry, Rz, Ra and Wait for valid bit then check the Middle Bit of Rx fifo for all channels and store the same in Rx, Ry, Rz and Ra.
;		2.Clear the valid bit 
;		3.Loop through the steps 1-2 for cnt iterations.
;	 (end code)
;
;	 Worst case peak cycle usage: 18(each iteration)
; 
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_AND_DOWNSAMPLE_MC	.macro	Rx, Ry, Rz, Ra, cnt, bit_idx
	LOOP    BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP?,	cnt ;  Run loop for receive bits
	LSL		Rx,	Rx,	1
	LSL		Ry,	Ry,	1
	LSL		Rz,	Rz,	1
	LSL		Ra,	Ra,	1 ; this will do for all 3 channels
WB2?:
	AND		SCRATCH.b0,	R31.b3, CH_MASK
	QBNE	WB2?,	SCRATCH.b0,	CH_MASK ;  wait for valid

	QBBS	BISSC_SET_CRC_BIT_CH0?,	R31.b0,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH0?
BISSC_SET_CRC_BIT_CH0?:
	OR		Rx,	Rx,	1
	OR		Ra.b0,	Ra.b0,	1
BISSC_SKIP_CRC_BIT_CH0?:

	QBBS    BISSC_SET_CRC_BIT_CH1?,	R31.b1,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH1?
BISSC_SET_CRC_BIT_CH1?:
	OR		Ry,	Ry,	1
	OR		Ra.b1,	Ra.b1,	1
BISSC_SKIP_CRC_BIT_CH1?:

	QBBS    BISSC_SET_CRC_BIT_CH2?,	R31.b2,	bit_idx ;  Check the Mid bit of received oversampled data
	JMP		BISSC_SKIP_CRC_BIT_CH2?
BISSC_SET_CRC_BIT_CH2?:
	OR		Rz,	Rz,	1
	OR		Ra.b2,	Ra.b2,	1
BISSC_SKIP_CRC_BIT_CH2?:

	MOV		R31.b3,	SCRATCH.b0 ;  clear valid bit
BISSC_RX_RECEIVE_DOWNSAMPLE_LOOP?:
	.endm



; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_CALC_CRC_MC
; 	Store and return the calculated CRC for all channels.
; Registers: 
;	ff[0-5]: Holds calculated CRC bits separately for all channels.
;	crc: Returns the overall CRC computation value. 
;
;  	PseudoCode:
;      (start code)
;       1.Load crc with 0
;		2.Left shift ff0-5 and crc by 1 byte, ch2 data will be available in b3 for ff0-5,
;		  ch2 data will overflow and ch1 data will be available in b3 for next iteration.
;		3.Check for individual bit(bit-0) from ff[0-5]
;		4.Set the bits required to be high in crc
;       5.Repeat step 2 and 4 for 6 ff.
;       (end code)
;
;	Worst case peak cycle usage:1+20(each iteration)
; 
 ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_CALC_CRC_MC	.macro	crc, ff0, ff1, ff2, ff3, ff4, ff5
	ZERO	&crc,	4

	LOOP	BISSC_CRC_CH0_END?,	3

	LSL		ff0, ff0,	8
	LSL		ff1, ff1,	8
	LSL		ff2, ff2,	8
	LSL		ff3, ff3,	8
	LSL		ff4, ff4,	8
	LSL 	ff5, ff5, 	8

	; ch2 would be initially in b0, at the end of the loop it would have reached b2, finally ch[0-2] would end up @b[0-2] as reqd.
	; no-op in first interation
	LSL		crc, crc, 8

	QBBS	BISSC_CRC_CH0_BIT1?,	ff0.b3,	0
	SET		crc.b0,	crc.b0,	0
BISSC_CRC_CH0_BIT1?:
	QBBS	BISSC_CRC_CH0_BIT2?,	ff1.b3,	0
	SET		crc.b0,	crc.b0,	1
BISSC_CRC_CH0_BIT2?:
	QBBS	BISSC_CRC_CH0_BIT3?,	ff2.b3,	0
	SET		crc.b0,	crc.b0,	2
BISSC_CRC_CH0_BIT3?:
	QBBS	BISSC_CRC_CH0_BIT4?,	ff3.b3,	0
	SET		crc.b0,	crc.b0,	3
BISSC_CRC_CH0_BIT4?:
	QBBS	BISSC_CRC_CH0_BIT5?,	ff4.b3,	0
	SET		crc.b0,	crc.b0,	4
BISSC_CRC_CH0_BIT5?:
	QBBS	BISSC_CRC_CH0_END?,		ff5.b3,	0
	SET		crc.b0,	crc.b0,	5
BISSC_CRC_CH0_END?:
	.endm
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_OTF_RECEIVE_MC
; 	Receive and downsaple the raw data then calculate the CRC for multichannel.
; Registers: 
;	Ra, Rb, Rc, Rd, Re, Rf: Holds the raw data received - caller has to zero or provide value to continue
;	SCRATCH3.b3: rx_size 
;	RAW_DATA_LEN: position data length with crc and error warning length
;	ff[0-5]: FF0to3_0, FF4_0, FF5_0 - otf crc calculation. 
;	BISSC_RCV_CRC_0: used for ex and returns the otf crc results.
;
;  PseudoCode:
;	(start code)
;       1.Load ff[0-5] to 0 .
;       2.Receive position data Ra and Rb with respect to the data length and Register length by calling onto 
;		  M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC.
;		3.Receive CRC bits into Rb and BISSC_RCV_CRC_0.
;		4.Load the Calculated CRC result in SCRATCH3.b3 by calling onto M_CALC_CRC.
;		5.Store the position data and crc to thier respective offsets and check for CRC validation between BISSC_RCV_CRC_0 and SCRATCH3.b3.
;       6.Increment the error counter if CRC not matched.
;	(end code)
;
;	 Worst case peak cycle usage: 1530
; REVISIT: More optimization may be required or can be done at least in M_INIT_CRC_FLIP_FLOPS - defer those till a requirement comes
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_OTF_RECEIVE_MC	.macro	Ra, Rb, Rc, Rd, Re, Rf, encoder_offset

	; initialize CRC flip-flops, i.e. ff[0-5] = 0
	ZERO	&FF0, 24
	SUB		SCRATCH3.b3, RAW_DATA_LEN, 6						; pos data + e+ w
	; receive data
	QBGE	BISSC_RX_LE_32_BITS?,	SCRATCH3.b3,	32
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC	Ra, Rc, Re,	32, FIFO_BIT_IDX, FF0, FF1, FF2, FF3, FF4, FF5, EX
	SUB		SCRATCH3.b3,	RAW_DATA_LEN,	(32 + 6)	; note taking care of CRC bits
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC	Rb, Rd, Rf, SCRATCH3.b3, FIFO_BIT_IDX, FF0, FF1, FF2, FF3, FF4, FF5, EX
	LDI		BISSC_RCV_CRC,	0	; clear crc_recvd
	M_OTF_RECEIVE_AND_DOWNSAMPLE_MC	Rb, Rd, Rf, BISSC_RCV_CRC, 6, FIFO_BIT_IDX
	JMP		BISSC_RX_CRC?

BISSC_RX_LE_32_BITS?:
	M_OTF_RECEIVE_DOWNSAMPLE_AND_CRC_MC	Ra, Rc, Re, SCRATCH3.b3, FIFO_BIT_IDX, FF0, FF1, FF2, FF3, FF4, FF5, EX
	RSB		SCRATCH3.b3,	SCRATCH3.b3,	32
	MIN		SCRATCH3.b3,	SCRATCH3.b3,	6	; CRC length, assuming 6 bit crc
	LDI 	BISSC_RCV_CRC, 		0
	M_OTF_RECEIVE_AND_DOWNSAMPLE_MC	Ra, Rc, Re, BISSC_RCV_CRC, SCRATCH3.b3, FIFO_BIT_IDX
	QBEQ	BISSC_RX_CRC?,	SCRATCH3.b3,	6
	RSB		SCRATCH3.b3,	SCRATCH3.b3,	6	; CRC length, assuming 6 bit crc
	M_OTF_RECEIVE_AND_DOWNSAMPLE_MC	Rb, Rd, Rf, BISSC_RCV_CRC, SCRATCH3.b3, FIFO_BIT_IDX
	
BISSC_RX_CRC?: 
	SBCO	&Ra,	PRUx_DMEM,	encoder_offset,	24
	ADD		encoder_offset, encoder_offset, 24
	M_CALC_CRC_MC	SCRATCH, FF0, FF1, FF2, FF3, FF4, FF5

	QBEQ	BISSC_CH0_CRC_SUCCESS?,	SCRATCH.b0, BISSC_RCV_CRC.b0	; crc: calculated - SCRATCH, received - BISSC_RCV_CRC
	LBCO	&SCRATCH3, PRUx_DMEM, encoder_offset, 4
	ADD		SCRATCH3, SCRATCH3, 1
	SBCO	&SCRATCH3, PRUx_DMEM,	encoder_offset , 4
BISSC_CH0_CRC_SUCCESS?:
	ADD		encoder_offset, encoder_offset, 4
	QBEQ	BISSC_CH2_CRC_SUCCESS?,	SCRATCH.b1, BISSC_RCV_CRC.b1	; crc: calculated - SCRATCH, received - BISSC_RCV_CRC
	LBCO	&SCRATCH3, PRUx_DMEM, encoder_offset, 4
	ADD		SCRATCH3, SCRATCH3, 1
	SBCO	&SCRATCH3, PRUx_DMEM,	encoder_offset , 4
BISSC_CH2_CRC_SUCCESS?:
	ADD		encoder_offset, encoder_offset, 4
	QBEQ	BISSC_CRC_SUCCESS?,	SCRATCH.b2, BISSC_RCV_CRC.b2	; crc: calculated - SCRATCH, received - BISSC_RCV_CRC
	LBCO	&SCRATCH3, PRUx_DMEM, encoder_offset, 4
	ADD		SCRATCH3, SCRATCH3, 1
	SBCO	&SCRATCH3, PRUx_DMEM,	encoder_offset , 4
BISSC_CRC_SUCCESS?:
	ADD		encoder_offset, encoder_offset, 4
	SBCO	&SCRATCH,	PRUx_DMEM,	encoder_offset , 4
	.endm
