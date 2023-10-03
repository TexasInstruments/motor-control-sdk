;
; sdfm.h
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

        .if !$defined("__sdfm_h")
__sdfm_h    .set    1

;
; Substitution symbols
;
        .asg    C0, CT_PRU_ICSSG_INTC           ; Constant Table, PRU_ICSSG INTC
        .asg    C1, CT_PRU_ICSSG_IEP1           ; Constant Table, PRU_ICSSG IEP1
        .asg    C3, CT_PRU_ICSSG_ECAP           ; Constant Table, PRU_ICSSG ECAP0
        .asg    C4, CT_PRU_ICSSG_CFG            ; Constant Table, PRU_ICSSG CFG
        .asg    C8, CT_PRU_ICSSG_IEP0_0x100     ; Constant Table, PRU_ICSSG IEP0_0x100
        .asg    C10, CT_PRU_ICSSG_TM            ; Constant Table, PRU_ICSSG TM
        .asg    C11, CT_PRU_ICSSG_CTRL          ; Constant Table, PRU Control
        .asg    C24, CT_PRU_ICSSG_LOC_DMEM      ; Constant Table, local PRU DMEM
        .asg    C26, CT_PRU_ICSSG_IEP0          ; Constant Table, PRU_ICSSG IEP0
	    .asg	CT_PRU_ICSSG_LOC_DMEM,		PRUx_DMEM  ;DMEM base address

        .asg    R1, TEMP_REG0                       ; temporary register 0
        .asg    R2, TEMP_REG1                       ; temporary register 1
        .asg    R3, TEMP_REG2                       ; temporary register 2
        .asg    R4, TEMP_REG3                       ; temporary register 3

        .asg    R23, SDFM_CFG_BASE_PTR_REG      ; SDFM CFG FW registers base pointer register
        .asg    R24, SD_HW_BASE_PTR_REG         ; SD hardware base pointer register
        .asg    R25.w0, RET_ADDR_REG            ; function return register

        .asg    R5, DN0                         ; SD integrator 3 (ACC3) output
        .asg    R6, CN3                         ; SDFM differentiator 1 output
        .asg    R7, CN4                         ; SDFM differentiator 2 output
        .asg    R8, CN5                         ; SDFM differentiator 3 output
        .asg    R21, MASK_REG                    ; integrator & differentiator output mask

        .asg    R26.w0, COMPARATOR_EN           ; SD comparator enable for different channels
        .asg    R26.w2, ZERO_CROSS_EN           ; SD Zero Crossing enable for different channels
        .asg    R19, OC_HIGH_THR              ; SD OC High threshold
        .asg    R27, OC_LOW_THR              ; SD OC Low threshold


        .asg    R29, GPIO_TGL_ADDR              ; Address to write to for the GPIO toggle


       .asg    R1, T0_CTXT_BASE_REG            ; base PRU register for T0 context
    .asg    R9, T1_S0_CTXT_BASE_REG         ; base PRU register for T1_S0 context

        .asg    R9, ACC3_DN1_CH0                ; Ch X (0...8), differentiator 1 state
        .asg    R10, ACC3_DN3_CH0               ; Ch X (0...8), differentiator 2 state
        .asg    R11, ACC3_DN5_CH0               ; Ch X (0...8), differentiator 3 state
        .asg    R12, ACC3_DN1_CH1               ; CH Y (0...8), differentiator 1 state
        .asg    R13, ACC3_DN3_CH1               ; CH Y (0...8), differentiator 2 state
        .asg    R14, ACC3_DN5_CH1               ; CH Y (0...8), differentiator 3 state
        .asg    R15, ACC3_DN1_CH2               ; CH Z (0...8), differentiator 1 state
        .asg    R16, ACC3_DN3_CH2               ; CH Z (0...8), differentiator 2 state
        .asg    R17, ACC3_DN5_CH2               ; CH Z (0...8), differentiator 3 state

        .asg    R22.b0, SD_CH0_ID                ; SD channel0 ID
        .asg    R22.b1, SD_CH1_ID                ; SD Channel1 ID
        .asg    R22.b2, SD_CH2_ID                ; SD Channel2 ID

        .asg    R20, OUT_SAMP_BUF_REG           ; address of local interleaved NC output sample buffer

        .asg    R28.b0,  SAMP_CNT_REG             ; NC sample count
        .asg    R28.b1,  SAMP_NAME                ; First/second sample number
        .asg    R28.b2,  NC_OUTPUT_SAMP         ;
        .asg    R28.b3,  EN_DOUBLE_UPDATE

;Fast detect registers(using only in SDFM init)

        .asg R19.b0,  FAST_TZ_OUT_REG
        .asg R19.b1,  FAST_WINDOW_REG
        .asg R19.b2,  FAST_ONE_MAX_REG
        .asg R19.b3,  FAST_ONE_MIN_REG
        .asg R27.b0,  FAST_ZERO_MAX_REG
        .asg R27.b1,  FAST_ZERO_MIN_REG


;
; Symbolic constants for ICSSG/PRU HW
;

; DMEM
PRUx_DSELF_BASE                 .set (0x00000000)   ;  Own Data RAM (8kB)

; SPAD Bank IDs for Xfer instructions
;
BANK0                           .set 10
BANK1                           .set 11
BANK2                           .set 12
BANK3                           .set 13

; PRU_ICSSG_INTC events
;
SYS_EVT_IEP_TIM_CAP_CMP_PEND    .set 7              ; IEP0 tim_cap_cmp_pend

; PRU_ICSSG_INTC
;
ICSSG_INTC_HIPIR1               .set 0x0904             ; Host Int 1 Prioritized Interrupt Register
ICSSG_INTC_HIPIR                .set ICSSG_INTC_HIPIR1  ; using Host Interrupt 1
ICSSG_INTC_SICR                 .set 0x0024             ; Sys Interrupt Indexed Deassert
NONE_HINT_BIT                   .set 31                 ; ICSSG_PRI_HINT_REG:NONE_HINT_0 bit number

; PRU_ICSSG_PRU_CTRL
;
ICSSG_CNTLSELF_BASE             .set 0
PRUx_CNTL_CONST_IDX0_OFFSET     .set 0x0020         ;  Constant Table Block Index Reg 0
PRUx_CNTLSELF_CONST_IDX0_REG    .set (ICSSG_CNTLSELF_BASE + PRUx_CNTL_CONST_IDX0_OFFSET)


; ICSSG_PRU_CTBIR0:C24_BLK_INDEX, PRU Constant Entry C24 Block Index
C24_BLK_INDEX_FW_REGS_VAL       .set 0
C24_BLK_INDEX_OUT_SAMP_BUF_VAL  .set 8

; PRU_ICSSG_CFG
;
PRUx_CFG_BASE                   .set (0x00026000)
ICSSG_CFG_GPCFG0                .set 0x0008 ;  GP IO Configuration Register 0
ICSSG_CFG_GPCFG1                .set 0x000C ;  GP IO Configuration Register 1
ICSSG_CFG_SPPC                  .set 0x0034 ;  Scratch PAD priority and config
ICSSG_CFG_PRU0_SD0_CLK          .set 0x48
ICSSG_CFG_PRU1_SD0_CLK          .set 0x94
ICSSG_CFG_PWM0                   .set 0x130 ; PWM0 configuration register offset 

;
; ICSSG_GPCFGn_REG:PR1_PRUn_GP_MUX_SEL, Controls the icss_wrap mux sel
;   n: {0,1}, PRU ID
PR1_PRUn_GP_MUX_SEL_SHIFT       .set 26
PR1_PRUn_GP_MUX_SEL_MASK        .set 0xF
PR1_PRUn_GP_MUX_SEL_VAL         .set 0011b

; ICSSG_SPP_REG:XFR_SHIFT_EN, Shift enable using R0[4:0] to define the number of 32-bit offset for XIN and XOUT operations
XFR_SHIFT_EN_BN                 .set 1
RTU_XFR_SHIFT_EN                .set 3

; ICSSG_PRUn_SD_CLK_SEL_REGi
;   n: {0,1}, PRU ID
;   i: {0...8}, SD Channel Number
; ICSSG_PRUn_SD_CLK_SEL_REGi:PRUn_SD_CLK_SELi, Selects the clock source
PRUn_SD_CLK_SELi_SHIFT          .set 0
PRUn_SD_CLK_SELi_MASK           .set 11b
PRUn_SD_CLK_SELi_VAL            .set 00b
; ICSSG_PRUn_SD_CLK_SEL_REGi:PRUn_SD_CLK_INVi, Optional clock inversion post clock selection mux
PRUn_SD_CLK_INVi_SHIFT          .set 2
PRUn_SD_CLK_INVi_MASK           .set 1b
; ICSSG_PRUn_SD_CLK_SEL_REGi:PRUn_SD_ACC_SELi, Selects to ACC source
PRUn_SD_ACC_SELi_SHIFT          .set 4
PRUn_SD_ACC_SELi_MASK           .set 11b
PRUn_SD_ACC_SELi_VAL            .set 00b
; ICSSG_PRUn_SD_CLK_SEL_REGi:PRUn_FD_ZERO_MIN_LIMIT_i
PRUn_FD_ZERO_MIN_LIMIT_i_SHIFT  .set 11
PRUn_FD_ZERO_MIN_LIMIT_i_MASK   .set 0x1F
; ICSSG_PRUn_SD_CLK_SEL_REGi:PRUn_FD_ZERO_MAX_LIMIT_i
PRUn_FD_ZERO_MAX_LIMIT_i_SHIFT  .set 17
PRUn_FD_ZERO_MAX_LIMIT_i_MASK   .set 0x1F

;MACRO FOR TASK MANAGER
COMP4_EVENT_NUMBER        .set  20
COMP_EVENT_FOUR_SIFT          .set  8

; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi
;   n: {0,1}, PRU ID
;   i: {0...8}, SD Channel Number
; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi:PRUn_SD_SAMPLE_SIZEi, Over Sample Rate
PRUn_SD_SAMPLE_SIZEi_SHIFT      .set 0
PRUn_SD_SAMPLE_SIZEi_MASK       .set 0xFF
; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi:PRUn_FD_WINDOW_SIZE_i
PRUn_FD_WINDOW_SIZE_i_SHIFT     .set 8
PRUn_FD_WINDOW_SIZE_i_MASK      .set 111b
; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi:PRUn_FD_ONE_MIN_LIMIT_i
PRUn_FD_ONE_MIN_LIMIT_i_SHIFT   .set 11
PRUn_FD_ONE_MIN_LIMIT_i_MASK    .set 0x1F
; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi:PRUn_FD_ONE_MAX_LIMIT_i
PRUn_FD_ONE_MAX_LIMIT_i_SHIFT   .set 17
PRUn_FD_ONE_MAX_LIMIT_i_MASK    .set 0x1F
; ICSSG_PRUn_SD_SAMPLE_SIZE_REGi:PRUn_FD_EN_i
PRUn_FD_EN_i_BN                 .set 23

; PRU_ICSSG_IEP
;
ICSSG_IEP_GLOBAL_CFG_REG        .set 0x0000 ; Global Configuration Register
ICSSG_IEP_COUNT_REG0            .set 0x0010 ; 64-bit Count Value Low Register
ICSSG_IEP_COUNT_REG1            .set 0x0014 ; 64-bit Count Value High Register
ICSSG_IEP_CMP_CFG_REG           .set 0x0070 ; Compare Configuration Register
ICSSG_IEP_CMP_STATUS_REG        .set 0x0074 ; Compare Status Register
ICSSG_IEP_CMP0_REG0             .set 0x0078 ; Compare 0 Low Register
ICSSG_IEP_CMP0_REG1             .set 0x007C ; Compare 0 High Register
ICSSG_IEP_CMP1_REG0             .set 0x0080 ; Compare 1 Low Register
ICSSG_IEP_CMP1_REG1             .set 0x0084 ; Compare 1 High Register
ICSSG_IEP_CMP2_REG0             .set 0x0088 ; Compare 1 Low Register
ICSSG_IEP_CMP2_REG1             .set 0x008C ; Compare 1 High Register
ICSSG_IEP_CMP3_REG0             .set 0x0090 ; Compare 1 Low Register
ICSSG_IEP_CMP3_REG1             .set 0x0094 ; Compare 1 High Register
ICSSG_IEP_CMP4_REG0             .set 0x0098 ; compare 4 low Register
ICSSG_IEP_CMP4_REG1             .set 0x009C ; compare 4 High Register
ICSSG_IEP_PWM_REG               .set 0x0008 ; PWM Sync Out Register, offset from 0x100
; ICSSG_IEP_GLOBAL_CFG_REG:CNT_ENABLE_BN
CNT_ENABLE_BN                   .set 0
; ICSSG_IEP_GLOBAL_CFG_REG:DEFAULT_INC
DEFAULT_INC_BN                  .set 4
; ICSSG_IEP_CMP_STATUS_REG:CMP_STATUS
CMP_STATUS_CMP1_BN              .set 1

; PRU_ICSSG_ECAP
;
ICSSG_eCAP_TSCNT                .set 0x0000 ; 32b time stamp counter
ICSSG_eCAP_CNTPHS               .set 0x0004 ; counter phase offset value
ICSSG_eCAP_CAP1                 .set 0x0008 ; 32b capture 1 reg
ICSSG_eCAP_CAP2                 .set 0x000C ; 32b capture 2 reg
ICSSG_eCAP_ECCTL1               .set 0x0028 ; capture control reg 1
; ICSSG_ECCTL2_ECCTL1
; ICSSG_ECCTL2_ECCTL1:APWMPOL
APWMPOL_SHIFT                   .set 26
APWMPOL_MASK                    .set 1b
APWMPOL_VAL                     .set 0b
; ICSSG_ECCTL2_ECCTL1:CAP_APWM
CAP_APWM_SHIFT                  .set 25
CAP_APWM_MASK                   .set 1b
CAP_APWM_VAL                    .set 1b
; ICSSG_ECCTL2_ECCTL1:SYNCO_SEL
SYNCO_SEL_SHIFT                 .set 22
SYNCO_SEL_MASK                  .set 11b
SYNCO_SEL_VAL                   .set 10b
; ICSSG_ECCTL2_ECCTL1:SYNCI_EN
SYNCI_EN_SHIFT                  .set 21
SYNCI_EN_MASK                   .set 1b
SYNCI_EN_VAL                    .set 0b
; ICSSG_ECCTL2_ECCTL1:TSCNTSTP
TSCNTSTP_BN                     .set 20

; PRU_ICSSG Tasks Manager
;
TM_CFG_PRU0_BASE                .set 0x0002A000
TM_CFG_RTU0_BASE                .set 0x0002A100
TM_CFG_PRU1_BASE                .set 0x0002A200
TM_CFG_RTU1_BASE                .set 0x0002A300
TM_CFG_TX_PRU0_BASE             .set 0x0002A400
TM_CFG_TX_PRU1_BASE             .set 0x0002A500
TASKS_MGR_TS1_PC_S0             .set 0x08
TASKS_MGR_TS1_PC_S1             .set 0x0C
TASKS_MGR_TS1_GEN_CFG1          .set 0x38

TM_YIELD_XID                    .set 252

;IEP_CFG
IEP_DEFAULT_INC                 .set 0x1

    .endif  ; __sdfm_h
