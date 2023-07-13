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

        .cdecls C,NOLIST
%{
        #include "icssg_sddf.h"
%}
        .include "sddf_cfg.h"


; Initial FW register values
; SDDF_EN=0, PRU_ID=uninitialized
INIT_SDDF_CTRL                  .set    (SDDF_EN << SDDF_CTRL_BF_SDDF_EN_SHIFT) | (PRU_ID << SDDF_CTRL_BF_PRU_ID_SHIFT)
; SDDF_EN_ACK=0, PRU_ID_ACK=uninitialized
INIT_SDDF_STAT                  .set    (SDDF_EN_ACK << SDDF_STAT_BF_SDDF_EN_ACK_SHIFT) | (PRU_ID_ACK << SDDF_STAT_BF_PRU_ID_ACK_SHIFT)
; IEP0 default increment=1
INIT_IEP_CFG                    .set    (IEP_DEFAULT_INC << IEP_CFG_BF_IEP_DEFAULT_INC_SHIFT)
; IEP0 CMP0 count to simulate EPWM (FOC loop) period:
;   - IEP frequency = 300 MHz
;   - IEP Default Increment = 1
;   - Simulated EPWM frequency = 8e3
; CMP0 = 300e6/1/8e3 = 37500 = 0x927C
; CMP3 = 300e6/16e3  = 18,750 = 0x493E
INIT_IEP_CFG_CMP0_CNT_EPWM_PRD  .set    (CMP0_CNT_EPWM_PRD << IEP_CFG_BF_CMP0_CNT_EPWM_PRD_SHIFT)
; SD Ch0 ID=0, Ch1 ID=1, Ch2 ID=2
INIT_SDDF_CFG_SD_CH_ID          .set    (SD_CH0_ID << SDDF_CFG_BF_SD_CH0_ID_SHIFT) | (SD_CH1_ID << SDDF_CFG_BF_SD_CH1_ID_SHIFT) | (SD_CH2_ID << SDDF_CFG_BF_SD_CH2_ID_SHIFT)
; SD_PRD_CLOCKS=10 => 20 MHz SD clock, SD_CLK_INV==0 => No clock inversion
INIT_SDDF_CFG_SD_CLK            .set    (SD_PRD_CLOCKS << SDDF_CFG_SD_CLK_BF_SD_PRD_CLOCKS_SHIFT) | (SD_CLK_INV << SDDF_CFG_SD_CLK_BF_SD_CLK_INV_SHIFT)
; SD OC OSR=(13+1)=14
INIT_SDDF_CFG_OSR               .set    (OC_OSR << SDDF_CFG_OC_OSR_BF_OSR_SHIFT)
; SD OC positive threshold = (OC OSR)^(OC SINC ORDER) = 14^3 = 2744
INIT_SDDF_CFG_OC_POS_THR        .set    (OC_POS_THR << SDDF_CFG_OC_POS_THR_SHIFT)
; SD OC negative threshold = 1
INIT_SDDF_CFG_OC_NEG_THR        .set    (OC_NEG_THR << SDDF_CFG_OC_NEG_THR_SHIFT)
; For SD samp freq==20e6, OSR==64, EPWM (FOC) loop freq = 8e3, sddf_setup_time=0 (TBD) &
; IEP freq = 300e6, IEP counter increment=1:
;   (1/2*(1/8e3) - 3/2*(64/20e6) - 0)*300e6*1 = 17310
;   ()
;
;for 16KHz  (1/2*(1/16e3)-3/2*(64/20e6)-0)*300e6 = 7935
;for
INIT_SDDF_CFG_TRIG_SAMP_TIME    .set    (TRIG_SAMP_TIME << SDDF_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_SHIFT)
; Generate 3 NC samples after trigger
INIT_SDDF_CFG_TRIG_SAMP_CNT     .set    (TRIG_SAMP_CNT << SDDF_CFG_TRIG_SAMP_CNT_BF_TRIG_SAMP_CNT_SHIFT)
; IEP0 counts in NC sampling period:
;   - IEP0 count =
;       (NC OSR * SD clock period)*(IEP0 frequency * IEP default count) =
;       (NC OSR * IEP0 frequency * IEP default count)/(SD clock frequency) =
;       64*300e6*1/20e6 = 960
; for NC OSR = 128
; 128*15 = 1920
; 256*15 = 3840
INIT_SDDF_CFG_NC_PRD_IEP_CNT    .set    (NC_PRD_IEP_CNT << SDDF_CFG_NC_PRD_IEP_CNT_SHIFT)
INIT_SDDF_CFG_OUT_SAMP_BUF      .set    (NC_OUT_SAMP_BUF << SDDF_CFG_NC_OUT_SAMP_BUF_BF_NC_OUT_SAMP_BUF_SHIFT)

                                ; SDDF Firmware Registers
                                .sect   ".fwRegs"
                                .retain ".fwRegs"
                                .retainrefs ".fwRegs"
                                .byte   INIT_SDDF_CTRL
                                .byte   INIT_SDDF_STAT
                                .byte   INIT_IEP_CFG
                                .byte   0   ; RESERVED
                                .word   INIT_IEP_CFG_CMP0_CNT_EPWM_PRD
                                .word   INIT_SDDF_CFG_SD_CH_ID
                                .short  INIT_SDDF_CFG_SD_CLK
                                .byte   INIT_SDDF_CFG_OSR
                                .byte   0   ; RESERVED
                                .short  INIT_SDDF_CFG_OC_POS_THR
                                .short  INIT_SDDF_CFG_OC_NEG_THR
                                .word   INIT_SDDF_CFG_TRIG_SAMP_TIME
                                .short  INIT_SDDF_CFG_TRIG_SAMP_CNT
                                .short  INIT_SDDF_CFG_NC_PRD_IEP_CNT
                                .word   INIT_SDDF_CFG_OUT_SAMP_BUF
