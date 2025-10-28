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
; define symboles
; Free registers: R23, R24
    .asg    C4, CT_PRU_ICSSG_CFG            ; Constant Table, PRU_ICSSG CFG
    .asg    C10, CT_PRU_ICSSG_TM            ; Constant Table, PRU_ICSSG TM
       
    .asg    R1, TEMP_REG0                       ; temporary register 0
    .asg    R2, TEMP_REG1                       ; temporary register 1
    .asg    R3, TEMP_REG2                       ; temporary register 2
    .asg    R4, TEMP_REG3                       ; temporary register 3

    .asg    R25.w0, RET_ADDR_REG            ; function return register

    .asg    R5, DN0                         ; SD integrator 3 (ACC3) output
    .asg    R6, CN3                         ; SDFM differentiator 1 output
    .asg    R7, CN4                         ; SDFM differentiator 2 output
    .asg    R8, CN5                         ; SDFM differentiator 3 output
    .asg    R21, MASK_REG                   ; integrator & differentiator output mask

    .asg    R26.w0, COMPARATOR_EN           ; SD comparator enable for different channels
    .asg    R26.b2, ZERO_CROSS_EN           ; SD Zero Crossing enable for different channels
    .asg    R26.b3, NC_SINC_FILTER_TYPE     ; SINC filter type for NC 
    .asg    R19, OC_HIGH_THR                ; SD OC High threshold
    .asg    R27, OC_LOW_THR                 ; SD OC Low threshold

    .asg    R29, GPIO_TGL_ADDR              ; Address to write to for the GPIO toggle

    .asg    R9, ACC3_DN1_CH0                ; Ch X (0...8), differentiator 1 state
    .asg    R10, ACC3_DN3_CH0               ; Ch X (0...8), differentiator 2 state
    .asg    R11, ACC3_DN5_CH0               ; Ch X (0...8), differentiator 3 state
    .asg    R12, ACC3_DN1_CH1               ; CH Y (0...8), differentiator 1 state
    .asg    R13, ACC3_DN3_CH1               ; CH Y (0...8), differentiator 2 state
    .asg    R14, ACC3_DN5_CH1               ; CH Y (0...8), differentiator 3 state
    .asg    R15, ACC3_DN1_CH2               ; CH Z (0...8), differentiator 1 state
    .asg    R16, ACC3_DN3_CH2               ; CH Z (0...8), differentiator 2 state
    .asg    R17, ACC3_DN5_CH2               ; CH Z (0...8), differentiator 3 state

    .asg    R22.b0, NC_SAMPLE_COUNT          ; min no. of continuous sample for sin filter
    .asg    R20, OUT_SAMP_BUF_REG            ; address of local interleaved NC output sample buffer

    .asg    R28.b0,  SAMP_CNT_REG             ; NC sample count
    .asg    R28.b1,  SAMP_NAME                ; First/second sample number
    .asg    R28.b2,  SD_CHANNEL_MASK          ; NC channel mask, FIXME: for nine channels 
    .asg    R28.b3,  EN_DOUBLE_UPDATE         ; Enable double update mode
    .asg    R22.b1,  EN_NC_TRIGGER_MODE       ; Enable continuous NC mode
    .asg    R22.w2,  NC_SAMPLE_DONE           ; Enable Snoop mode

; SPAD Bank IDs for Xfer instructions
;
BANK0                           .set 10
BANK1                           .set 11
BANK2                           .set 12
BANK3                           .set 13

    .if $isdefed("SDFM_LOAD_SHARE_MODE")
    .if $isdefed("SDFM_RTU_CORE")
ICSSG_CFG_PWMx                   .set 0x130 ; PWM0 configuration register offset 
    .elseif $isdefed("SDFM_PRU_CORE")
ICSSG_CFG_PWMx                   .set 0x134 ; PWM1 configuration register offset 
    .elseif $isdefed("SDFM_TXPRU_CORE")
ICSSG_CFG_PWMx                   .set 0x138 ; PWM2 configuration register offset 
    .endif ; SDFM_TXPRU_CORE
    .else
ICSSG_CFG_PWMx                   .set 0x130 ; PWM0 configuration register offset 
    .endif ;SDFM_LOAD_SHARE_MODE

; PRU_ICSSG Tasks Manager
;
TASKS_MGR_TS1_PC_S0             .set 0x08
TASKS_MGR_TS1_PC_S1             .set 0x0C
TASKS_MGR_TS1_GEN_CFG1          .set 0x38
CMP_EVENT_BIT_SHIFT             .set 8
IEP_DEFAULT_INC                 .set 1
TASK_IEP0_STARTING_EVT          .set 16
TASK_IEP1_STARTING_EVT          .set 40
PRU_ICSS_IEP1_BASE              .set (0x2f000UL)
PRU_ICSS_IEP0_BASE              .set (0x2e000UL)

;Mask for phase delay
SDFM_11_MASK                    .set  0x00010002
SDFM_01_MASK                    .set  0x00000002
TM_YIELD_XID                    .set 252

;SD_CH_ID
    .if $isdefed("SDFM_LOAD_SHARE_MODE")
    .if $isdefed("SDFM_RTU_CORE")
; Load Sharing: RTUn
SD_CH0                       .set 0000b
SD_CH1                       .set 0001b
SD_CH2                       .set 0010b
   .elseif $isdefed("SDFM_PRU_CORE")
; Load Sharing: PRUn
SD_CH0                      .set 0011b
SD_CH1                       .set 0100b
SD_CH2                       .set 0101b
   .elseif $isdefed("SDFM_TXPRU_CORE")
SD_CH0                      .set 00110b
SD_CH1                       .set 0111b
SD_CH2                       .set 1000b    
   .endif
   .else
SD_CH0                       .set 0000b
SD_CH1                       .set 0001b
SD_CH2                       .set 0010b 
SD_CH3                       .set 0011b
SD_CH4                       .set 0100b
SD_CH5                       .set 0101b
SD_CH6                       .set 0110b
SD_CH7                       .set 0111b
SD_CH8                       .set 1000b
   .endif

   .endif  ; __sdfm_h
