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
;*   File:  tamagawa_sync_macros.h                                                  *
;*                                                                                  *
;*   Brief: Macros for synchronizing PRUs in load share mode for Tamagawa encoder   *    
;************************************************************************************

;************************************* includes *************************************
	.include "tamagawa_icss_reg_defs.h"
    .include "tamagawa_interface.h"

    .if !$isdefed("__tamagawa_sync_macros_h")
__tamagawa_sync_macros_h .set 1

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_TAMAGAWA_LS_WAIT_FOR_SYNC
;   Loop till all channels are synchronized in load share mode.
; Registers:
;   SCRATCH.b0: Used for sync state values (loaded from DMEM as needed)
;   SCRATCH.b1: Holds the combined execution status of all PRUs in load share
;   TAMAGAWA_ENABLED_CHANNELS: Original channel mask stored in register
;
;  PseudoCode:
;       (start code)
;       1.Set sync state for the PRU which enters this macro.
;       2.Store the execution state at specific PRU offset in DMEM.
;       3.Load the execution state back for all PRUs(in use) and perform 'or' operation for all execution states in SCRATCH.b1.
;       4.Check whether execution states and channel mask is equal.
;       5.Repeat step 3 and 4 till all PRUs(in use) execution state is set.
;       (end code)
;
;    Worst case peak cycle usage: 14
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
M_TAMAGAWA_LS_WAIT_FOR_SYNC    .macro
    LDI     SCRATCH.b0, (1<<TAMAGAWA_CHANNEL_BIT_ID)
    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
    LDI     SCRATCH1.w0, TAMAGAWA_LS_EXEC_RTU_STATE
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
    LDI     SCRATCH1.w0, TAMAGAWA_LS_EXEC_PRU_STATE
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
    LDI     SCRATCH1.w0, TAMAGAWA_LS_EXEC_TX_PRU_STATE
    .endif
    SBCO    &SCRATCH.b0, PRUx_DMEM, SCRATCH1.w0, 1
TAMAGAWA_IS_SYNCED?:
    LBCO    &SCRATCH.b0, PRUx_DMEM, TAMAGAWA_LS_EXEC_RTU_STATE, 3
    MOV     SCRATCH.b3, SCRATCH.b0
    OR      SCRATCH.b3, SCRATCH.b3, SCRATCH.b1
    OR      SCRATCH.b3, SCRATCH.b3, SCRATCH.b2
    QBNE    TAMAGAWA_IS_SYNCED?, SCRATCH.b3, TAMAGAWA_ENABLED_CHANNELS
    .endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_TAMAGAWA_LS_CLEAR
;   Clear all PRU synchronization status locations in load share mode.
; Registers:
;   SCRATCH: Temporary register used to store zero value for clearing.
;
;  PseudoCode:
;    (start code)
;       1.Load 32-bit zero value into SCRATCH register.
;       2.Store 3 bytes of zero starting at TAMAGAWA_LS_EXEC_RTU_STATE to clear
;         all three sync status locations (RTU, PRU, TXPRU) simultaneously.
;    (end code)
;
;    Worst case peak cycle usage: 3
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_TAMAGAWA_LS_CLEAR    .macro
    LDI     SCRATCH, 0
    SBCO    &SCRATCH.b0, PRUx_DMEM, TAMAGAWA_LS_EXEC_RTU_STATE, 3
    .endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_TAMAGAWA_LS_WAIT_FOR_SYNC_CLEAR
;   Wait until all PRU synchronization status locations are cleared in load share mode.
; Registers:
;   SCRATCH: Temporary register used to read sync status values.
;
;  PseudoCode:
;    (start code)
;       1.Initialize SCRATCH register with zero.
;       2.Read 3 bytes starting at TAMAGAWA_LS_EXEC_RTU_STATE containing
;         all sync status locations (RTU, PRU, TXPRU).
;       3.Compare with zero to check if all sync status locations are cleared.
;       4.Loop back to step 2 if any sync status location is still set.
;       5.Continue execution when all sync status locations are cleared.
;    (end code)
;
;    Worst case peak cycle usage: Variable (depends on sync clear completion time)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_TAMAGAWA_LS_WAIT_FOR_SYNC_CLEAR  .macro
    LDI   SCRATCH, 0
TAMAGAWA_IS_SYNC_CLEARED?: 
    LBCO    &SCRATCH.b0, PRUx_DMEM, TAMAGAWA_LS_EXEC_RTU_STATE, 3
    QBNE    TAMAGAWA_IS_SYNC_CLEARED?, SCRATCH, 0
    .endm

; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;
; Macro: M_TAMAGAWA_LS_GLOBAL_REINIT
;   Execute global reinit with synchronization across all PRU cores in load share mode.
;   Only primary core triggers the actual global reinit signal.
; Registers:
;   SCRATCH.b1: Used to load primary core mask from DMEM when needed
;
;  PseudoCode:
;    (start code)
;       1.Wait for all PRUs to reach this synchronization point.
;       2.Load primary core mask from DMEM and check if this PRU is designated as primary.
;       3.If primary core, execute SET R31, TAMAGAWA_TX_GLOBAL_REINIT to assert TX_EN low.
;       4.Primary core clears sync flags to allow other PRUs to proceed.
;       5.All PRUs wait for sync clear before continuing.
;    (end code)
;
;    Worst case peak cycle usage: Variable (depends on synchronization timing)
;
; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ; ;

M_TAMAGAWA_LS_GLOBAL_REINIT    .macro
    .if $isdefed("ENABLE_MULTI_MAKE_RTU") | $isdefed("ENABLE_MULTI_MAKE_PRU") | $isdefed("ENABLE_MULTI_MAKE_TXPRU") 
    M_TAMAGAWA_LS_WAIT_FOR_SYNC
    LDI     SCRATCH1.w0, TAMAGAWA_PRIMARY_CORE_MASK_OFFSET
    LBCO    &SCRATCH.b1, PRUx_DMEM, SCRATCH1.w0, 1
    QBBC    TAMAGAWA_SKIP_GLOBAL_REINIT?, SCRATCH.b1, TAMAGAWA_CHANNEL_BIT_ID
    SET     R31, TAMAGAWA_TX_GLOBAL_REINIT     ; Set TX_EN low for global reinit
    M_TAMAGAWA_LS_CLEAR
TAMAGAWA_SKIP_GLOBAL_REINIT?:
    M_TAMAGAWA_LS_WAIT_FOR_SYNC_CLEAR
    .else
    ; Single-core mode: no synchronization needed
    SET     R31, TAMAGAWA_TX_GLOBAL_REINIT
    .endif
    .endm

    .endif ; __tamagawa_sync_macros_h