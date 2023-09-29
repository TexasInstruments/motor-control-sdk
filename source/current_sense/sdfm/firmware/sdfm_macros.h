;
; sdfm_macros.h
;
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

        .if !$defined("__sdfm_macros_h")
__sdfm_macros_h .set    1

        .include "sdfm.h"

;
; Macros
;
;

WRITE_C24_BLK_INDEX .macro blk_index
        ; Set DMEM (C24) block offset
        LDI     TEMP_REG0.b0, blk_index
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_CTRL, PRUx_CNTLSELF_CONST_IDX0_REG, 1
        NOP ; delay for update to land?
        .endm


; Set SD HW registers base pointer
SET_SD_HW_REG_BASE_PTR  .macro  base_ptr
        ; Load TR0.b0 <- FW_REG_SDFM_CTRL
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_PRU_ID_OFFSET,  SDFM_PRU_ID_SZ
        ; Check PRU ID 0 or 1
        QBEQ    pru_id1?, TEMP_REG0.b0, 1
pru_id0?:
        LDI32   base_ptr, PRUx_CFG_BASE+ICSSG_CFG_PRU0_SD0_CLK
        QBA     set_sd_hw_reg_base_ptr_end?
pru_id1?:
        LDI32   base_ptr, PRUx_CFG_BASE+ICSSG_CFG_PRU1_SD0_CLK
set_sd_hw_reg_base_ptr_end?:
        .endm

; Configure Triggered mode sample count
;CFG_TRIG_MODE_SAMP_CNT  .macro  samp_cnt
        ; Load samp_cnt <- SDFM_CFG_TRIG_SAMPLE_CNT
  ;      LBCO    &samp_cnt, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDFM_CFG_TRIG_SAMPLE_CNT, FW_REG_SDFM_CFG_TRIG_SAMPLE_CNT_SZ
   ;     .endm

; Wait until shadow flag of the channel is set & clear the flag
;   args    - ch_idx    : SD channel index {0...ICSSG_NUM_SD_CH-1}
;;   updates - TEMP_REG0.w2  : channel sample buffer offset
;
M_WAIT_SHADOW_FLAG_AND_CLR  .macro  ch_idx
        ; Place ch_idx in R30[29-26], channel_select
        LSL     TEMP_REG1.b0, ch_idx, 2
        SET     TEMP_REG1.b0.t1
        MOV     R30.b3, TEMP_REG1.b0
        NOP

wait_for_shadow_update_cont?:
        ; R31[24], shadow_update_flag for ICSS
        ; R31[28], shadow_update_flag for ICSSG
        QBBC    wait_for_shadow_update_cont?, R31, 28
        ; R31[24], shadow_update_flag_clr for ICSS/ICSSG
        SET     R31, R31.t24
        .endm

        .endif  ; __sdfm_macros_h

; Calculates Sinc3 sample value from ACC3 & Sinc3 variables
;   args    - DN1, DN3, DN5 : Sinc3 differntiator state variables
;   expects - ACC3 value in DN0, mask in MASK_REG
;   uses    - CN3, CN4, CN5
;   result  - CN5           : Output sample
;
M_ACC3_PROCESS  .macro  DN1, DN3, DN5
        RSB     CN3, DN1, DN0
        MOV     DN1, DN0
        RSB     CN4, DN3, CN3
        MOV     DN3, CN3
        RSB     CN5, DN5, CN4
        MOV     DN5, CN4
        AND     CN5, CN5, MASK_REG  ; apply limit
        .endm

;Enable task manager
M_PRU_TM_ENABLE .macro
    tsen 1
    .endm

;Disable task manager
M_PRU_TM_DISABLE .macro
    tsen 0
    .endm