; Copyright (c) 2023, Texas Instruments Incorporated
; All rights reserved.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;
;  *  Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;
;  *  Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;
;  *  Neither the name of Texas Instruments Incorporated nor the names of
;     its contributors may be used to endorse or promote products derived
;     from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
;  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
;  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
;  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
;  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
;  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
;  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
;  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
;  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
;  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;  file: sdfm_macros.h
;
;
    .if !$defined("__sdfm_macros_h")
__sdfm_macros_h .set    1
   
    .include "sdfm.h"

;************************************************************************************
;
;   Macro: M_WRITE_C24_BLK_INDEX
;
;   Write C24 block index for local PRU DMEM
;   
;   PEAK cycles:
;        3 cycles
;   Pseudo code:
;       ICSSG_PRU_CTBIR0[0-7] = blk_index;
;
;   Parameters:
;      blk_ind : local DMEM base address for pru constant entry 24 block index 
;
;   Returns:
;      None
;
;************************************************************************************
M_WRITE_C24_BLK_INDEX .macro blk_index
    ; Set DMEM (C24) block offset
    LDI     TEMP_REG0.b0, blk_index
    SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_CTRL, PRUx_CNTLSELF_CONST_IDX0_REG, 1
    ; delay for update to land?
    NOP                                                                      
    .endm

;************************************************************************************
;
;   Macro: M_SET_SD_HW_REG_BASE_PTR
;
;   Set SD HW registers base pointer
;
;   PEAK cycles:
;        5 cycles
;
;   Pseudo code:
;       (start code)
;       if(slice_id==0)
;       {
;           base_ptr = PRUx_CFG_BASE + 0x48;
;       }
;       else
;       {
;           base_ptr = PRUx_CFG_BASE + 0x94;
;       }
;       (endcode) 
;   Parameters:
;      base_ptr : R24
;
;   Returns:
;      None
;
;************************************************************************************
M_SET_SD_HW_REG_BASE_PTR  .macro  base_ptr
    ;load PRU slice number
    LBBO    &TEMP_REG0.b0, SDFM_CFG_BASE_PTR_REG, SDFM_PRU_ID_OFFSET,  SDFM_PRU_ID_SZ
    ; Check slice ID 0 or 1
    QBEQ    pru_id1?, TEMP_REG0.b0, 1
pru_id0?:
    LDI32   base_ptr, PRUx_CFG_BASE+ICSSG_CFG_PRU0_SD0_CLK
    QBA     set_sd_hw_reg_base_ptr_end?
pru_id1?:
    LDI32   base_ptr, PRUx_CFG_BASE+ICSSG_CFG_PRU1_SD0_CLK
set_sd_hw_reg_base_ptr_end?:
    .endm

;************************************************************************************
;   Macro: M_ACC3_PROCESS
;
;   Calculates Sinc3 sample value from ACC3 & Sinc3 variables
; 
;   PEAK cycles:
;        7 cycles
;
;   Pseudo code:
;       (start code)
;       cn3 = dn0 - dn1;
;       dn1 = dn0;
;       cn4 = cn3 - dn3;
;       dn3 = cn3;
;       cn5 = cn4 - dn5
;       dn5 = cn4
;       cn5 = cn5 & 0x0FFFFFFF
;       (endcode) 
;   Parameters:
;      DN1, DN3, DN5 : Sinc3 differntiator state variables
;
;   Result:
;      CN5 : Output sample
;
;    Uses:
;       CN3, CN4, CN5
;
;************************************************************************************
M_ACC3_PROCESS  .macro  DN1, DN3, DN5
    RSB     CN3, DN1, DN0
    MOV     DN1, DN0
    RSB     CN4, DN3, CN3
    MOV     DN3, CN3
    RSB     CN5, DN5, CN4
    MOV     DN5, CN4
    AND     CN5, CN5, MASK_REG  ; apply limit
    .endm

;************************************************************************************
;************************************************************************************
;   Macro: M_ACC2_PROCESS
;
;   Calculates Sinc2 sample value from ACC2 & Sinc2 variables
; 
;   PEAK cycles:
;        5 cycles
;
;   Pseudo code:
;       (start code)
;       cn3 = dn0 - dn1;
;       dn1 = dn0;
;       cn4 = cn3 - dn3;
;       dn3 = cn3;
;       cn5 = cn5 & 0x0FFFFFFF
;       (endcode) 
;   Parameters:
;      DN1, DN3: Sinc2 differntiator state variables
;
;   Result:
;      CN5 : Output sample
;
;    Uses:
;       CN3, CN4, CN5
;
;************************************************************************************
M_ACC2_PROCESS  .macro  DN1, DN3
    RSB     CN3, DN1, DN0
    MOV     DN1, DN0
    RSB     CN4, DN3, CN3
    MOV     DN3, CN3
    AND     CN5, CN4, MASK_REG  ; apply limit
    .endm

;***********************************************************************************
;************************************************************************************
;   Macro: M_ACC1_PROCESS
;
;   Calculates Sinc1 sample value from ACC1 & Sinc1 variables
; 
;   PEAK cycles:
;        3 cycles
;
;   Pseudo code:
;       (start code)
;       cn3 = dn0 - dn1;
;       dn1 = dn0;
;       cn5 = cn3 & 0x0FFFFFFF
;       (endcode) 
;   Parameters:
;      DN1 : Sinc1 differntiator state variables
;
;   Result:
;      CN5 : Output sample
;
;    Uses:
;       CN3, CN5
;
;************************************************************************************
M_ACC1_PROCESS  .macro  DN1
    RSB     CN3, DN1, DN0
    MOV     DN1, DN0
    AND     CN5, CN3, MASK_REG  ; apply limit
    .endm

;***********************************************************************************
;
;   Macro: M_PRU_TM_ENABLE
;
;   Enable task manager
;
;   PEAK cycles:
;        1 cycle
;
;   Pseudo code:
;      .word   0x32800000
;   Parameters:
;      None
;
;   Returns:
;      None
;
;************************************************************************************
M_PRU_TM_ENABLE .macro
    tsen 1
    .endm

;************************************************************************************
;
;   Macro: M_PRU_TM_DISABLE
;
;   Disable task manager
;
;   PEAK cycles:
;        5 cycles
;
;   Pseudo code:
;     .word 0x32000000
;   Parameters:
;      None
;
;   Returns:
;      None
;
;************************************************************************************
M_PRU_TM_DISABLE .macro
    tsen 0
    .endm


;************************************************************************************
;
;   Macro: M_SDFM_PHASE_DELAY_FOR_RAISING_EDGE
;
;   Calculate number of PRU cycles between data raising edge  and upcoming nearest clock raising edge 
;
;   Invokes:
;       None
;
;   Parameters:
;      None
;
;   Results: TEMP_REG1.w0 -> PRU cycles
;            TEMP_REG1.w2 -> Raising Edge status 
;            TEMP_REG2    -> MAX PRU cyles between clk & data edge 
;
;
;************************************************************************************
M_SDFM_PHASE_DELAY_FOR_RAISING_EDGE .macro
; waiting for one on SD data line 
        wbc  R31.b0, 1
        ;waiting for rising edge of sd data
        wbs  R31.b0, 1
        QBBS   DELAY_1_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_2_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_3_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_4_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_5_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_6_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_7_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_8_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_9_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_10_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_11_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_12_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_13_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_14_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_15_PRU_CYCLE,  R31.b2, 0
        QBBS   DELAY_16_PRU_CYCLE,  R31.b2, 0
DELAY_1_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 1 
        MAX  TEMP_REG2, TEMP_REG2,   1
        JMP  DELAY_DONE   
DELAY_2_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 2 
        MAX  TEMP_REG2, TEMP_REG2,   2
        JMP  DELAY_DONE   
DELAY_3_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 3 
        MAX  TEMP_REG2, TEMP_REG2,   3
        JMP  DELAY_DONE   
DELAY_4_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 4 
        MAX  TEMP_REG2, TEMP_REG2,   4
        JMP  DELAY_DONE   
DELAY_5_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 5 
        MAX  TEMP_REG2, TEMP_REG2,   5
        JMP  DELAY_DONE   
DELAY_6_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 6 
        MAX  TEMP_REG2, TEMP_REG2,   6
        JMP  DELAY_DONE   
DELAY_7_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 7 
        MAX  TEMP_REG2, TEMP_REG2,   7
        JMP  DELAY_DONE   
DELAY_8_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 8 
        MAX  TEMP_REG2, TEMP_REG2,   8
        JMP  DELAY_DONE   
DELAY_9_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 9 
        MAX  TEMP_REG2, TEMP_REG2,   9
        JMP  DELAY_DONE   
DELAY_10_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 10
        MAX  TEMP_REG2, TEMP_REG2,   10 
        JMP  DELAY_DONE   
DELAY_11_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 11 
        MAX  TEMP_REG2, TEMP_REG2,   11
        JMP  DELAY_DONE   
DELAY_12_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 12 
        MAX  TEMP_REG2, TEMP_REG2,   12
        JMP  DELAY_DONE   
DELAY_13_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 13
        MAX  TEMP_REG2, TEMP_REG2,   13 
        JMP  DELAY_DONE   
DELAY_14_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 14
        MAX  TEMP_REG2, TEMP_REG2,   14 
        JMP  DELAY_DONE   
DELAY_15_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 15
        MAX  TEMP_REG2, TEMP_REG2,   15 
        JMP  DELAY_DONE   
DELAY_16_PRU_CYCLE:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 16
        MAX  TEMP_REG2, TEMP_REG2,   16  
DELAY_DONE:
        LDI  TEMP_REG1.b2, 0  ; status of edge is 0 means rising edge
       .endm
  
;************************************************************************************
;
;   Macro: M_SDFM_PHASE_DELAY_FOR_FALLING_EDGE
;
;  Calculate number of PRU cycles between data raising edge  and upcoming nearest clock falling edge 
;
;   Invokes:
;       None
;
;   Parameters:
;      None
;
;   Results: TEMP_REG1.w0 -> PRU cycles
;            TEMP_REG1.w2 -> falling edage status 
;            TEMP_REG2    -> MAX PRU cyles between clk & data edge 
;
;
;************************************************************************************ 
M_SDFM_PHASE_DELAY_FOR_FALLING_EDGE .macro
; waiting for one on SD data line 
        wbc  R31.b0, 1
        ;waiting for rising edge of sd data
        wbs  R31.b0, 1
        QBBC   DELAY_1_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_2_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_3_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_4_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_5_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_6_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_7_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_8_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_9_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_10_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_11_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_12_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_13_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_14_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_15_PRU_CYCLE1,  R31.b2, 0
        QBBC   DELAY_16_PRU_CYCLE1,  R31.b2, 0
DELAY_1_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 1 
        MAX  TEMP_REG2, TEMP_REG2,   1
        JMP  DELAY_DONE1   
DELAY_2_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 2
        MAX  TEMP_REG2, TEMP_REG2,   2 
        JMP  DELAY_DONE1   
DELAY_3_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 3
        MAX  TEMP_REG2, TEMP_REG2,   3 
        JMP  DELAY_DONE1   
DELAY_4_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 4
        MAX  TEMP_REG2, TEMP_REG2,   4 
        JMP  DELAY_DONE1   
DELAY_5_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 5
        MAX  TEMP_REG2, TEMP_REG2,   5 
        JMP  DELAY_DONE1   
DELAY_6_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 6
        MAX  TEMP_REG2, TEMP_REG2,   6 
        JMP  DELAY_DONE1   
DELAY_7_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 7
        MAX  TEMP_REG2, TEMP_REG2,   7 
        JMP  DELAY_DONE1   
DELAY_8_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 8
        MAX  TEMP_REG2, TEMP_REG2,   8 
        JMP  DELAY_DONE1   
DELAY_9_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 9
        MAX  TEMP_REG2, TEMP_REG2,   9 
        JMP  DELAY_DONE1   
DELAY_10_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 10 
        MAX  TEMP_REG2, TEMP_REG2,   10
        JMP  DELAY_DONE1   
DELAY_11_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 11
        MAX  TEMP_REG2, TEMP_REG2,   11 
        JMP  DELAY_DONE1   
DELAY_12_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 12
        MAX  TEMP_REG2, TEMP_REG2,   12 
        JMP  DELAY_DONE1   
DELAY_13_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 13
        MAX  TEMP_REG2, TEMP_REG2,   13 
        JMP  DELAY_DONE1   
DELAY_14_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 14
        MAX  TEMP_REG2, TEMP_REG2,   14 
        JMP  DELAY_DONE1   
DELAY_15_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 15
        MAX  TEMP_REG2, TEMP_REG2,   15 
        JMP  DELAY_DONE1   
DELAY_16_PRU_CYCLE1:
        ADD  TEMP_REG1.w0,  TEMP_REG1.w0, 16
        MAX  TEMP_REG2, TEMP_REG2,   16  
DELAY_DONE1:
        LDI  TEMP_REG1.b2, 1   ; status of edge is 1 means falling edge
       .endm

   .endif  ; __sdfm_macros_h