; Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
;
; Redistributions of source code must retain the above copyright
; notice, this list of conditions and the following disclaimer.
;
; Redistributions in binary form must reproduce the above copyright
; notice, this list of conditions and the following disclaimer in the
; documentation and/or other materials provided with the
; distribution.
;
; Neither the name of Texas Instruments Incorporated nor the names of
; its contributors may be used to endorse or promote products derived
; from this software without specific prior written permission.
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

;************************************************************************************
;   File:     main.asm
;
;   Brief:    Template asm file example
;************************************************************************************

; CCS/makefile specific settings
    .retain     ; Required for building .out with assembly file
    .retainrefs ; Required for building .out with assembly file

    .global     main
    .sect       ".text"

;************************************* includes *************************************
; icss_constant_defines.inc: Defines symbols corresponding to Constant Table Entries
    .include "icss_constant_defines.inc"
    .include "icss_iep_macros.inc"
    .include "icss_tm_macros.inc"


    .cdecls C,  NOLIST
%{
#include "drivers/pruicss/g_v0/cslr_icss_g.h"
%}

    .asg	R2,	   TEMP_REG
    .asg    R3,    CMP1_REG0
    .asg    R4,    CMP1_REG1
    .asg    R5,    SYNC_PERIOD

    .if	$isdefed("PRU0")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_PRU0_PR1_TASKS_MGR_PRU0_MMR_REGS_BASE, TM_BASE
    .elseif	$isdefed("PRU1")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_PRU1_PR1_TASKS_MGR_PRU1_MMR_REGS_BASE, TM_BASE
    .elseif	$isdefed("RTU_PRU0")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_RTU0_PR1_TASKS_MGR_RTU0_MMR_REGS_BASE, TM_BASE
    .elseif	$isdefed("RTU_PRU1")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_RTU1_PR1_TASKS_MGR_RTU1_MMR_REGS_BASE, TM_BASE
    .elseif	$isdefed("TX_PRU0")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_PRU_TX0_PR1_TASKS_MGR_PRU_TX0_MMR_REGS_BASE, TM_BASE
    .elseif	$isdefed("TX_PRU1")
    .asg     CSL_ICSS_G_PR1_TASKS_MGR_PRU_TX1_PR1_TASKS_MGR_PRU_TX1_MMR_REGS_BASE, TM_BASE
    .endif


tmpRegStruct        .sassign  R6, Struct_4_Reg
SYNC_PERIOD_IN_NS .set  (100*1000) ; 100us
IEP_INSTANCE        .set  0
;IEP0 cmp1 hit event is 17
TASK_MANAGER_EVENT  .set  17

;********
;* MAIN *
;********

main:

init:
;----------------------------------------------------------------------------
;   Clear the register space
;   Before begining with the application, make sure all the registers are set
;   to 0. PRU has 32 - 4 byte registers: R0 to R31, with R30 and R31 being special
;   registers for output and input respectively.
;----------------------------------------------------------------------------

; Give the starting address and number of bytes to clear.
    zero	&r0, 120

task_manager_init:
    ; disable task manager
    tsen   0
    m_pru_tm_rst    tmpRegStruct, TM_BASE
    ; Task manager configuration
    ; Mode: GP mode
    ; Priority: T1_S1 (highest) > T1_S2 > T1_S3 > T1_S4 > T1_S0 (Lowest)
    ; Priority: T2_S0 (highest) > T2_S1 > T2_S2 > T2_S3 > T2_S4 (Lowest)
    ;****************************************************************************************************
    ; Subtask       State       Function         Event
    ;****************************************************************************************************
    ; T1_S0         Enabled    TM_IEP0_CMP1_TASK      17 (CMP1)
    ; Rest of the tasks are disabled.
    ;****************************************************************************************************
    m_pru_tm_set_cfg_gpmode_enable tmpRegStruct,     TM_BASE   ; 5 cyc. Macro - Setup Task Manager in GP mode.
    ; configure task manager for iep_tasks, TS2 is highest priority and can pre-empt TS1
    m_pru_tm_ts1_pc_set tmpRegStruct, TM_BASE,  CMP1_TASK, FN_NOP_TASK, FN_NOP_TASK, FN_NOP_TASK, FN_NOP_TASK    ; 11 cyc
    ;m_pru_tm_disable_task tmpRegStruct, TM_BASE, TS1_S0
    m_pru_tm_disable_task tmpRegStruct, TM_BASE, TS1_S1
    m_pru_tm_disable_task tmpRegStruct, TM_BASE, TS1_S2
    m_pru_tm_disable_task tmpRegStruct, TM_BASE, TS1_S3
    m_pru_tm_disable_task tmpRegStruct, TM_BASE, TS1_S4
    ; set TS1_S0  trigger to IEP0 cmp1 hit event = 17
    ; Note : TS1_S1 cannot preempt TS1_S0
    ;        TS1_S0 cannot preempt TS1_S1
    m_pru_tm_set_cfg_gpmode_ts1_mux_set tmpRegStruct, TM_BASE, TASK_MANAGER_EVENT, 0, 0, 0, 0
    ; enable Task manager
    tsen   1
    LDI32	CMP1_REG0,   SYNC_PERIOD_IN_NS
    LDI32   SYNC_PERIOD, SYNC_PERIOD_IN_NS
    ;add compare1 with sync period
	ADD		CMP1_REG0, CMP1_REG0, SYNC_PERIOD
	ADC     CMP1_REG1, CMP1_REG1, 0


L1:
	;wait for compare 1
	qba	L1

CMP1_TASK:    .asmfunc
	;loop count
	;wait for 1us (configure iep clk and icss clk in sync mode)
	LDI		TEMP_REG, 200
	loop endloop, TEMP_REG
	nop
endloop:
	;clear sync
	m_set_iep_sync_ctrl_reg     IEP_INSTANCE, TEMP_REG, 0x0
	;configure compare 1
	sbco    &CMP1_REG0, ICSS_IEP_CONST, ICSS_IEP_CMP1_REG, 8
    ;enable sync
    m_set_iep_sync_ctrl_reg     IEP_INSTANCE, TEMP_REG, 0x3
    ;add compare1 with sync period
	ADD		CMP1_REG0, CMP1_REG0, SYNC_PERIOD
	ADC     CMP1_REG1, CMP1_REG1, 0

yeild_task:
    ;exit from task
    xin     TM_YIELD_XID, &R29.b3, 1
    NOP
    NOP
    .endasmfunc

FN_NOP_TASK:    .asmfunc
    ; m_pru_tm_yield
    xin     TM_YIELD_XID, &R29.b3, 1
    NOP
    NOP                                                       ; 4 cycles
    .endasmfunc

