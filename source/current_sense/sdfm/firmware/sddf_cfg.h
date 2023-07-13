;
; sddf_cfg.h
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

        .if !$defined("__sddf_cfg_h")
__sddf_cfg_h    .set    1

        .cdecls C,NOLIST
%{
        #include "icssg_sddf.h"
%}

;
; Symbolic constants for FW configuration
;

; Provided by compiler
;SDDF_PRU_CORE                   .set 1 ; build code for PRUn
;SDDF_RTU_CORE                   .set 1 ; build code for RTUn

; SDDF_CTRL & SDDF_STAT
SDDF_EN                         .set BF_SDDF_EN_ENABLE
PRU_ID                          .set BF_PRU_ID_0
SDDF_EN_ACK                     .set BF_SDDF_EN_DISABLE
PRU_ID_ACK                      .set BF_PRU_ID_UNINIT

; IEP_CFG
IEP_DEFAULT_INC                 .set 0x1

; IEP_CFG_EPWM_PRD
CMP0_CNT_EPWM_PRD               .set 0x927C

; SDDF_CFG_SD_CH_ID
    .if $isdefed("SDDF_RTU_CORE")
; Load Sharing: RTUn
SD_CH0_ID                       .set 0000b
SD_CH1_ID                       .set 0001b
SD_CH2_ID                       .set 0010b
    .elseif $isdefed("SDDF_PRU_CORE")
; Load Sharing: PRUn
;SD_CH0_ID                       .set 0011b
;SD_CH1_ID                       .set 0100b
;SD_CH2_ID                       .set 0101b
SD_CH0_ID                       .set 0000b
SD_CH1_ID                       .set 0001b
SD_CH2_ID                       .set 0010b
    .elseif $isdefed("SDDF_TX_PRU_CORE")
; Load Sharing: TX_PRUn
SD_CH0_ID                       .set 0110b
SD_CH1_ID                       .set 0111b
SD_CH2_ID                       .set 1000b
    .endif

; SDDF_CFG_SD_CLK
;SD_CLK_INV                      .set 0b
;SD_PRD_CLOCKS                   .set 0x0F

; SDDF_CFG_OSR
;OC_OSR                          .set 0x0D
;OC_OSR                          .set 0x0E

; SDDF_CFG_OC_POS_THR
;OC_POS_THR                      .set 0x0AB8

; SDDF_CFG_OC_NEG_THR
;OC_NEG_THR                      .set 0x0001

; SDDF_CFG_TRIG_SAMP_TIME
;TRIG_SAMP_TIME                  .set 0x439E

; SDDF_CFG_TRIG_SAMP_CNT
;TRIG_SAMP_CNT                   .set 0x0003

; SDDF_CFG_NC_PRD_IEP_CNT
;NC_PRD_IEP_CNT                  .set 0x03C0

; SDDF_CFG_OUT_SAMP_BUF
    .if $isdefed("SDDF_RTU_CORE")
; Load Sharing: RTUn
;NC_OUT_SAMP_BUF                 .set 0x78100000 ; R5F_0_0 BTCM, SoC view
    .elseif $isdefed("SDDF_PRU_CORE")
; Load Sharing: PRUn
;NC_OUT_SAMP_BUF                 .set 0x7810000C ; R5F_0_0 BTCM, SoC view
    .elseif $isdefed("SDDF_TX_PRU_CORE")
; Load Sharing: TX_PRUn
;NC_OUT_SAMP_BUF                 .set 0x78100018 ; R5F_0_0 BTCM, SoC view
    .endif

    .endif  ; __sddf_cfg_h
