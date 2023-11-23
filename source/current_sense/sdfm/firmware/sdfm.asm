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
        #include "icssg_sdfm.h"
%}
        .include "sdfm.h"
        .include "sdfm_macros.h"
        .include "firmware_version.h"




; Compile-time Host event for SDFM samples available
; R31 event interface mapping, add pru<n>_r31_vec_valid to system event number, <sysevt> + 1<<5
    .if $isdefed("SDFM_PRU_CORE")
TRIGGER_HOST_SDFM_IRQ   .set PRU_TRIGGER_HOST_SDFM_EVT + 16
    .elseif $isdefed("SDFM_RTU_CORE")
TRIGGER_HOST_SDFM_IRQ   .set RTU_TRIGGER_HOST_SDFM_EVT + 16
    .endif

;SPAD Bank for SD Ch context storage
BANK_CTXT_NC               .set BANK0

;differentiator state located in BANK locations 9-17
NUM_REGS_DIFF_STATE    .set  9  ; Number of PRU registers for differentiator state
OUT_SAMP_MASK           .set 0x0FFFFFFF ; 28-bit mask applied to Integrator & Differentiator output

;Required sample for stable NC sample = NC_SAMP_CNT - 1
NC_SAMP_CNT .set 4

;--------------------------------------Unsed Registers-------------------------------------;
;registers R20 - R24
;R20: contain address of NC local output
;R21; output mask
;R22.w0 channel ID
;R24 PRUx CFG base address
;-------------------------------------------------------------------------------------------;



        ; local interleaved NC output sample buffer
OUT_SAMP_BUF:   .usect  ".outSamps", ICSSG_NUM_SD_CH_FW*4, 4
        .retain         ".outSamps"
        .retainrefs     ".outSamps"

        .def    SDFM_ENTRY  ; global entry point
        .ref    dbg_setup_pinmux

        .sect   ".text"
        .retain ".text"
        .retainrefs ".text"
SDFM_ENTRY:
        ; Clear registers R0-R30
        ZERO    &R0, 124
        LDI32  TEMP_REG0, ICSS_FIRMWARE_RELEASE_1
        LDI32  TEMP_REG1, ICSS_FIRMWARE_RELEASE_2
        SBCO   &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, SDFM_FIRMWARE_VERSION_OFFSET, 8
        ; Disable Task Manager
        ;.word 0x32000000
        M_PRU_TM_DISABLE

        ; Clear Task Manager status which is sticky after debug halt
        LDI     TEMP_REG0.w0, 0x0fff
        SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, 0, 2
        XIN     TM_YIELD_XID, &R0.b3,1
        LDI     TEMP_REG0.w0, 0
        SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, 0, 2


        ; Write C24 block index for access to FW registers
        WRITE_C24_BLK_INDEX C24_BLK_INDEX_FW_REGS_VAL

PHASE_DELAY_CAL:
        ;check phase delay measurment active
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_EN_PHASE_DELAY, 1
        QBBC    SKIP_PHASE_DELAY_CAL, TEMP_REG0.b0, 0
        JAL     RET_ADDR_REG, SDFM_CLOCK_PHASE_COMPENSATION
        ;acknowledge 
        CLR     TEMP_REG0, TEMP_REG0, 0
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_EN_PHASE_DELAY, 1       
SKIP_PHASE_DELAY_CAL:



;
; Check SDFM global enable & set SDFM global enable acknowledge to inform R5 core.
; If SDFM global enable not set, wait for SDFM global enable from R5.
;
check_sdfm_en:
        ;Check for phase delay measurment
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_EN_PHASE_DELAY, 1
        QBBS    PHASE_DELAY_CAL, TEMP_REG0.b0, 0
        ; Check SDFM global enable
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_EN_OFFSET, SDFM_EN_SZ
        QBBC    check_sdfm_en, TEMP_REG0.b0, 0                      ; If SDFM_EN not set, wait to set sdfm enable

        ; Set SDFM global enable acknowledge
        SET     TEMP_REG0, TEMP_REG0, 0                             ; Set SDFM_EN_ACK
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_EN_ACK_OFFSET, SDFM_EN_ACK_SZ


;
; Perform initialization
;
init_sdfm:
        ; Enable XIN/XOUT shifting.
        ; Used for context & SD state save/restore in TM tasks.
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_CFG, ICSSG_CFG_SPPC, 1
    .if $isdefed("SDFM_PRU_CORE")
        SET     TEMP_REG0, TEMP_REG0, XFR_SHIFT_EN_BN   ; ICSSG_SPP_REG:XFR_SHIFT_EN=1
    .elseif $isdefed("SDFM_RTU_CORE")
        SET     TEMP_REG0, TEMP_REG0, RTU_XFR_SHIFT_EN   ; ICSSG_SPP_REG:RTU_XFR_SHIFT_EN=1
    .endif
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_CFG, ICSSG_CFG_SPPC, 1

        ; Initialize Task Manager
        JAL     RET_ADDR_REG, tm_init

        ; Enable Task Manager
        ;.word   0x32800000
        M_PRU_TM_ENABLE

    .if $isdefed("SDFM_PRU_CORE") ; no IEP on RTU
        ; Initialize IEP0
        JAL     RET_ADDR_REG, iep0_init
    .endif


        ; Initialize SD mode
        LDI32   TEMP_REG1, PR1_PRUn_GP_MUX_SEL_VAL<<PR1_PRUn_GP_MUX_SEL_SHIFT
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM, SDFM_PRU_ID_OFFSET,  SDFM_PRU_ID_SZ   ; Load TR0.b0 <-  SDFM_CTRL
        QBEQ    init_pru_id1, TEMP_REG0.b0, 1                          ; Check PRU ID 0 or 1
init_pru_id0:
        SBCO    &TEMP_REG1, CT_PRU_ICSSG_CFG, ICSSG_CFG_GPCFG0, 4                           ; Initialize PRU0 SD mode
        QBA     init_sdfm_cont
init_pru_id1:
        SBCO    &TEMP_REG1, CT_PRU_ICSSG_CFG, ICSSG_CFG_GPCFG1, 4                           ; Initialize PRU1 SD mode



init_sdfm_cont:

        ; Set base point to SD HW registers ; (PRUx_CFG_BASE address)
        SET_SD_HW_REG_BASE_PTR SD_HW_BASE_PTR_REG

        ; Read Connected Channel numbers
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_CH_ID_OFFSET,  SDFM_CFG_SD_CH_ID_SZ
        AND    TEMP_REG1, TEMP_REG0, BF_SD_CH0_ID_MASK
        MOV    SD_CH0_ID, TEMP_REG1.b0
        LSR    TEMP_REG0, TEMP_REG0, 4
        AND    TEMP_REG1, TEMP_REG0, BF_SD_CH1_ID_MASK
        MOV    SD_CH1_ID, TEMP_REG1.b0
        LSR    TEMP_REG0, TEMP_REG0, 4
        AND    TEMP_REG1, TEMP_REG0, BF_SD_CH2_ID_MASK
        MOV    SD_CH2_ID, TEMP_REG1.b0

        ; Reset SDFM state
        JAL     RET_ADDR_REG, reset_sdfm_state

        ; Initialize SD clock
        JAL     RET_ADDR_REG, init_sd_clock

        ; Configure OSR for SD channels
        JAL     RET_ADDR_REG, config_oc_osr

        ; Configure SD channels. For all channels, initialize:
        ;   ACC select: ACC3
        ;   Clock inversion
        ;   Clock source: pr1_pru<n>_pru_r31_in[16] Primary Input
        JAL     RET_ADDR_REG, config_sd_ch

        ; Global enable SD HW,
        ; reset SD channel HW
        JAL     RET_ADDR_REG, reset_sd_ch_hw
        SET     R30.t25 ; R30[25] channel_en = 1, all channels enabled

        ;Configure pwm TZ block
        JAL   RET_ADDR_REG, config_pwm_trip

        ; Initialize dedicated registers:
        ;   MASK register,
        ;   Local NC output sample buffer address,
        ;   Clear NC sample count.
        LDI32   MASK_REG, OUT_SAMP_MASK
        LDI32   OUT_SAMP_BUF_REG, OUT_SAMP_BUF
        LDI  SAMP_CNT_REG,  0
        LBCO  &EN_DOUBLE_UPDATE,  CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_EN_DOUBLE_UPDATE,  1
        ;NC continuous mode status 
        LBCO  &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_EN_CONT_NC_MODE,1
        LSL   TEMP_REG0.b0, TEMP_REG0.b0,1 
        OR    EN_DOUBLE_UPDATE, EN_DOUBLE_UPDATE, TEMP_REG0.b0

        LDI  SAMP_NAME, 0



        .if $isdefed("SDFM_PRU_CORE") ; no IEP on RTU
        ; Start IEP
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        SET     TEMP_REG0, TEMP_REG0, CNT_ENABLE_BN ; ICSSG_IEP_GLOBAL_CFG_REG:CNT_ENABLE=1
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
       .endif

       LBCO   &COMPARATOR_EN, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_EN_COMP_OFFSET,  SDFM_CFG_EN_COMP_SZ

       QBBS    ts0_oc_loop, COMPARATOR_EN, SDFM_CFG_EN_COMP_BIT
       ;waiting loop if OC is disable
wait_loop:
       JMP   wait_loop


;------------------------------------Over Current---------------------------------------------------------;
;1) Select channel & read data from shadow register
;2) Does SINC3 differentiation
;3) Check sampled data value with High & Low threshold
;4) Toggle GPIO based on comparison
;---------------------------------------------------------------------------------------------------------;
ts0_oc_loop:

        QBBC    comp_ch0_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH0_EN_COMP_BIT
        ;Switch to SD HW to Ch0 & Enable channels
        MOV     TEMP_REG1.b0, SD_CH0_ID            ;Select channel 0
        LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2      ;R30[26-29] channel select bits
        SET     TEMP_REG1.b0.t1                    ;R30[25] global channel enable bit
        MOV     R30.b3, TEMP_REG1.b0
        NOP

        ; R31[28], check shadow_update_flag for Ch0
        QBBC    comp_ch0_end, R31, 28

        .if $isdefed("DEBUG_CODE")
        ;Debug code  :GPIO HIGH
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        .endif


        ;R31[24], ; clear shadow update flag for ch0
        SET     R31, R31.t24
        ; Load reg R31[0-27] SD HW ACC3 output sample
        AND     DN0, R31, MASK_REG
        ;Execute Sinc3 Differentiation
        M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0


        .if $isdefed("DEBUG_CODE")
        ;store data
         SBBO    &CN5, OUT_SAMP_BUF_REG, 0, 4
        .endif



        ;Comparator for Ch0
        MOV     TEMP_REG2, CN5
        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBCO    &OC_HIGH_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_HIGH_THR_CH0_OFFSET,  SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBCO    &OC_LOW_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_LOW_THR_CH0_OFFSET,  SDFM_CFG_OC_LOW_THR_SZ

        QBGE    over_threshold_start_ch0, OC_HIGH_THR, TEMP_REG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch0, OC_HIGH_THR, TEMP_REG2
low_threshold_ch0_check:
        ;Check if the sample value is greater than the low threshold
        QBGE    below_threshold_end_ch0, OC_LOW_THR, TEMP_REG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch0, OC_LOW_THR, TEMP_REG2
over_threshold_start_ch0:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch0_check

over_threshold_end_ch0:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch0_check

below_threshold_end_ch0:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA comp_ch0_end

below_threshold_start_ch0:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
comp_ch0_end:


        .if $isdefed("DEBUG_CODE")
        ;GPIO LOW
         LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
         LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        .endif




        QBBC    comp_ch1_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH1_EN_COMP_BIT
        ; Switch to SD HW to Ch1 & Enable channels
        MOV     TEMP_REG1.b0,  SD_CH1_ID         ;Select channel 1
        LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2    ;R30[26-29] channel select bits
        SET     TEMP_REG1.b0.t1                  ;R30[25] global channel enable bit
        MOV     R30.b3, TEMP_REG1.b0
        NOP

        ; R31[28], check  shadow_update_flag for ch1
        QBBC    comp_ch1_end, R31, 28


        .if $isdefed("DEBUG_CODE")
        ;GPIO HIGH
         LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
         LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        .endif


        ; R31[24], shadow_update_flag_clr for ch1
        SET     R31, R31.t24
        ; Load reg R31[0-27] SD HW ACC3 output sample
        AND     DN0, R31, MASK_REG
        ;Execute Sinc3 Differentiation
        M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1


        .if $isdefed("DEBUG_CODE")
        ;store data
        SBBO    &CN5, OUT_SAMP_BUF_REG, 4, 4
        .endif



        ;Comparator for Ch0
        MOV     TEMP_REG2, CN5

        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBCO    &OC_HIGH_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_HIGH_THR_CH1_OFFSET,  SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBCO    &OC_LOW_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_LOW_THR_CH1_OFFSET,  SDFM_CFG_OC_LOW_THR_SZ

        ;Check if the sample value is greater than the high threshold
        QBGE    over_threshold_start_ch1, OC_HIGH_THR, TEMP_REG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch1, OC_HIGH_THR, TEMP_REG2
low_threshold_ch1_check:
        ;Check if the sample value is greater than the low threshold
        QBGE    below_threshold_end_ch1, OC_LOW_THR, TEMP_REG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch1, OC_LOW_THR, TEMP_REG2
over_threshold_start_ch1:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch1_check

over_threshold_end_ch1:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch1_check

below_threshold_end_ch1:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH1_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA     comp_ch1_end

below_threshold_start_ch1:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH1_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
comp_ch1_end:

        .if $isdefed("DEBUG_CODE")
        ;GPIO LOW
         LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
         LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
         .endif




        QBBC    comp_ch2_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH2_EN_COMP_BIT
        ; Switch to SD HW to Ch2 & Enable Channels
        MOV     TEMP_REG1.w0,  SD_CH2_ID       ;Select channel 2
        LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2  ;R30[26-29] channel select bits
        SET     TEMP_REG1.b0.t1                ;R30[25] global channel enable bit
        MOV     R30.b3, TEMP_REG1.b0
        NOP

        ; R31[28], check shadow_update_flag for Ch2
        QBBC    ts0_oc_loop, R31, 28


         .if $isdefed("DEBUG_CODE")
         ; GPIO HIGH
         LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
         LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        .endif


        ; R31[24], shadow_update_flag_clr for Ch2
        SET     R31, R31.t24
        ; Load reg R31[0-27] SD HW ACC3 output sample
        AND     DN0, R31, MASK_REG
        ; Execute Sinc3 Differentiation
        M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2


        .if $isdefed("DEBUG_CODE")
        ;store data
        SBBO    &CN5, OUT_SAMP_BUF_REG, 8, 4
        .endif


        ;Comparator for Ch2
        MOV     TEMP_REG2, CN5

        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBCO    &OC_HIGH_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_HIGH_THR_CH2_OFFSET,  SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBCO    &OC_LOW_THR, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_OC_LOW_THR_CH2_OFFSET,  SDFM_CFG_OC_LOW_THR_SZ

        ;Check if the sample value is greater than the high threshold
        QBGE    over_threshold_start_ch2, OC_HIGH_THR, TEMP_REG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch2, OC_HIGH_THR, TEMP_REG2
        ;Check if the sample value is greater than the low threshold
low_threshold_ch2_check:
        QBGE    below_threshold_end_ch2, OC_LOW_THR, TEMP_REG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch2, OC_LOW_THR, TEMP_REG2
over_threshold_start_ch2:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch2_check

over_threshold_end_ch2:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch2_check

below_threshold_end_ch2:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH2_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        QBA comp_ch2_end

below_threshold_start_ch2:
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH2_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_CLR_ADDR_SZ

comp_ch2_end:



        .if $isdefed("DEBUG_CODE")
         ; GPIO LOW
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        .endif

        .if $isdefed("DEBUG_CODE")
         ; Write local interleaved output samples to Host buffer address
        LBBO    &TEMP_REG3, OUT_SAMP_BUF_REG, 0, ICSSG_NUM_SD_CH_FW*4
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET,4
        SBBO    &TEMP_REG3,  TEMP_REG0,  SDFM_CFG_OUT_SAMP_BUF_OFFSET, ICSSG_NUM_SD_CH_FW*4
         ; Trigger interrupt
        LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ
        .endif

       QBA     ts0_oc_loop


;--------------------------------Normal Current---------------------------------------------------;
;Normal current
;1)Retore & save registers
;2)Clear CMP event
;3)Select channel, enable snoop mode, read data & does diffrentiation
;4) Update cmp register for next NC sample
;  a)Check sample count: 4 times continuous NC sampling for stable data
;  a)Update Cmp_reg based on first sample point, second sample point & sample count
;5)at every 4th sample of NC, firmware stores sampled in DMEM & triggers R5 interrupt
;6)restore & save registers
;7)clear task
;--------------------------------------------------------------------------------------------------;
ts1_nc_loop:
        ; Save/restore context
        ; restore differentiator state(R1-R18) for NC
        ; save current registers values for OC
        LDI   R0.b0, 0
        MOV     R18.b0, R30.b3                  ; save T0 SD channel select
        xchg    BANK_CTXT_NC, &R1, 4*18

        ; Clear IEP0 CMP4 event
        LDI		TEMP_REG0.b0, 0x10
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_STATUS_REG, 1

        .if $isdefed("DEBUG_CODE")
        ;Debug code  :GPIO HIGH
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_SET_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_SET_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ
        ;debug code end
       .endif

        ;select ch0, enable all channel, set SD snoop=1 & sample_counter_select=1
        LDI     R30.w2, (0<<10 | 1<<9 | 1<<6 | 1<<5)
        NOP
        ; Snoop read Ch0 sample_counter,
        ; wait for ChX sample count+1.
        AND     TEMP_REG1, R31, 0xFF ; snoop read LSB Ch0 sample_counter
wait_sample_count_incr_oc:
        AND     TEMP_REG2, R31, 0xFF ; snoop read LSB Ch0 sample_counter
        QBEQ    wait_sample_count_incr_oc, TEMP_REG2, TEMP_REG1

        .if $isdefed("DEBUG_CODE")
        ;GPIO LOW
        LBCO    &GPIO_TGL_ADDR, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_CLR_VAL_ADDR_OFFSET,  SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBCO    &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET,  SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TEMP_REG3, GPIO_TGL_ADDR, 0,  SDFM_CFG_GPIO_VALUE_SZ

        ;Store sample counter values
        LDI    TEMP_REG1, SDFM_DUBUG_OFFSET
        LSL    TEMP_REG3, SAMP_CNT_REG, 1
        ADD     TEMP_REG1, TEMP_REG3, TEMP_REG1
        SBCO    &TEMP_REG2, CT_PRU_ICSSG_LOC_DMEM, TEMP_REG1, 1
       .endif

        ;Snoop read Ch0 ACC3
        CLR     R30.t21                 ; set SD sample_counter_select=0
        NOP
        AND     DN0, R31, MASK_REG      ; DN0 = Ch0 SD HW ACC3 output sample
        CLR     R30.t22                 ; set SD snoop=0

        ;select ch1, enable all channel & set SD snoop=1
        LDI     R30.w2, (1<<10 | 1<<9 | 1<<6)
        NOP
        ; Snoop read ACC3 for SD Ch1
        AND     TEMP_REG1, R31, MASK_REG    ; TEMP_REG0 = Ch1 SD HW ACC3 output sample
        CLR     R30.t22                     ; set SD snoop=0


        ;select ch2, enable all channel & set SD snoop=1
        LDI     R30.w2, (2<<10 | 1<<9 | 1<<6)
        NOP
        ; Snoop read ACC3 for SD Ch2
        AND     TEMP_REG2, R31, MASK_REG    ; TEMP_REG1 = Ch2 SD HW ACC3 output sample
        CLR     R30.t22                     ; set SD snoop=0

        ; Execute SINC3 differentiation for Ch0
        M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0
        ; Save NC output sample to local output sample buffer
        SBBO    &CN5, OUT_SAMP_BUF_REG, 0, 4

         ; Execute SINC3 differentiation for Ch1
        MOV     DN0, TEMP_REG1 ; DN0 = Ch1 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1
        ; Save output sample to local output sample buffer
        SBBO    &CN5, OUT_SAMP_BUF_REG, 4, 4

        ; Execute SINC3 differentiation for Ch2
        MOV     DN0, TEMP_REG2 ; DN0 = Ch2 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2
        ; Save output sample to local output sample buffer
        SBBO    &CN5, OUT_SAMP_BUF_REG, 8, 4
        
        ;continuous mode check
        QBBC    TRIGGER_MODE, EN_DOUBLE_UPDATE, 1

        ;update IEP0 CMP4
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_NC_PRD_IEP_CNT_OFFSET, 4
        LBCO    &TEMP_REG1, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4 ;
        ADD     TEMP_REG0, TEMP_REG1, TEMP_REG0
        ;read iep counter maximum value
        LDI     TEMP_REG1, 0
        LBCO    &TEMP_REG1, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET, 4
        QBLE    UPDATE_CMP_FOR_IEP_RESET,  TEMP_REG0, TEMP_REG1
        ;update Cmp4 with old value + next sample time value
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
        JMP     END_CMP_UPDATE
UPDATE_CMP_FOR_IEP_RESET:
        ;Update cmp4 according to iep reset
        SUB      TEMP_REG0, TEMP_REG0, TEMP_REG1
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
END_CMP_UPDATE:
        JMP  END_RESET_NC_FRAME

TRIGGER_MODE:
       ;Check NC sample count
        QBLE    RESET_NC_FRAME, SAMP_CNT_REG, NC_SAMP_CNT-1
        ; NC sample count < NC_SAMP_CNT-1
        ; Add configured IEP count for NC OSR
        ; IEP0 CMP4_reg = cmp4_reg + NC_OSR*IEP_CLOCK* SD_cycle
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_NC_PRD_IEP_CNT_OFFSET,  4
        LBCO    &TEMP_REG1, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4 ;
        ADD     TEMP_REG0, TEMP_REG1, TEMP_REG0
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4

        ADD     SAMP_CNT_REG, SAMP_CNT_REG, 1   ; increment NC sample count

        QBA     NRESET_NC_FRAME
        
RESET_NC_FRAME:
        ; Set IEP CMP$ value: IEP_CMP4_REG1:REG0 = 0:TRIG_SAMPLE_TIME
         QBBS    FIRST_NC_SAMPLE, SAMP_NAME, 0
         QBBC    FIRST_NC_SAMPLE, EN_DOUBLE_UPDATE, 0 ;check double update is enable
         LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME, 4
         SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP4
         SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
         LDI    SAMP_NAME, 1 ;clear sample name for first sample
         QBA     END_RESET_NC_FRAME
FIRST_NC_SAMPLE:
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME, 4
        SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP4
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
        LDI    SAMP_NAME, 0 ; update for Second sample
END_RESET_NC_FRAME:
        LDI     SAMP_CNT_REG, 0 ; reset NC sample count
        ; Write local interleaved output samples to Host buffer address
        LBBO    &TEMP_REG3, OUT_SAMP_BUF_REG, 0, ICSSG_NUM_SD_CH_FW*4
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET,4
        SBBO    &TEMP_REG3,  TEMP_REG0,  SDFM_CFG_OUT_SAMP_BUF_OFFSET, ICSSG_NUM_SD_CH_FW*4
        ;Trigger interrupt
        LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ
        ;;SBCO    &NC_OUTPUT_SAMP,  CT_PRU_ICSSG_LOC_DMEM,  SDFM_DUBUG_OFFSET, 4

NRESET_NC_FRAME:

        ; Save/restore context
        ; save differentiator state(R9-R17) for NC
        ; restore NC registers
        LDI   R0.b0, 0
        xchg    BANK_CTXT_NC, &R1, 4*18
        MOV     R30.b3, R18.b0                  ; restore T0 SD channel select

        XIN     TM_YIELD_XID, &R0.b3, 1  ; exit task after two instructions/cycles
        NOP
        NOP


;
; Initialize Task Manager
;
tm_init:
; Configure Task Manager tasks
;
        ; TM general purpose mode
        ; Enable  T1_S1: IEP0 CMP4 task
        LDI     TEMP_REG0.b0, (1b<<3|0b<<2|11b<<0)           ; enable  T1_s1
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_TM, 0, 1

        ;set T1_S1 address
        LDI     TEMP_REG0.w0, $CODE(ts1_nc_loop)
        SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_PC_S1, 2

        ; Set Task triggers
        ; set T1_S1 trigger to IEP0 CMP4 event = 20
        LDI     TEMP_REG0.w0, (COMP4_EVENT_NUMBER<<COMP_EVENT_FOUR_SIFT)
        SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_GEN_CFG1, 2

        JMP     RET_ADDR_REG

; Initialize IEP0.
;
iep0_init:
        ; Disable IEP0 counter
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        AND     TEMP_REG0.b0, TEMP_REG0.b0, 0xFE
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1

        ; Set IEP0 counter to zero
        LDI     TEMP_REG0, 0
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_COUNT_REG1, 4
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_COUNT_REG0, 4

        ; Clear IEP0  CMP4 events
        LDI		TEMP_REG0.b0, 0x10
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_STATUS_REG, 1

        ; Write IEP0 default increment
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        AND     TEMP_REG0.b0, TEMP_REG0.b0, 0x0F
        OR      TEMP_REG0.b0, TEMP_REG0.b0, IEP_DEFAULT_INC<<DEFAULT_INC_BN
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1

        ;Enable IEP0 counter on EPWM0 SYNC0 event

        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0_0x100, ICSSG_IEP_PWM_REG, 1
        SET     TEMP_REG0.b0.t0 ; IEP_PWM_REG:PWM0_RST_CNT_EN = 1, enable IEP0 counter reset on EPWM0 SYNCO event
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0_0x100, ICSSG_IEP_PWM_REG, 1


        ; Initialize Trigger sample time for OC
        ; Set IEP0 CMP4 value: IEP0_CMP4_REG1:REG0 = 0:TRIG_SAMPLE_TIME
        LDI     TEMP_REG0, 0
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG1, 4
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_LOC_DMEM,  FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME,  4
        LDI     TEMP_REG1, SDFM_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_MASK
        AND     TEMP_REG0, TEMP_REG1, TEMP_REG0
        SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP0
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
        ; Enable IEP0 CMP4
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_CFG_REG, 1  ; TR0 <- Byte0 ICSSG_CMP_CFG_REG
        SET     TEMP_REG0.t5    ; CMP_EN[5]=1 => CMP4 enabled
        SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_CFG_REG, 1  ; TR0 -> ICSSG_CMP_CFG_REG Byte0



        JMP     RET_ADDR_REG


;
; Reset Scratchpad for NC registers
;
; Arguments: None
;
reset_sdfm_state:
       LDI     R0.b0, 0
       ZERO    &R1, 4*18
       XOUT    BANK_CTXT_NC, &R1, 4*18 ;clear ScratchPad registers for NC
       JMP     RET_ADDR_REG


;
; Configure OC OSR, FD_ONE Min/Max, for SD channels
;
; Arguments:
;   CT_PRU_ICSSG_LOC_DMEM: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;   SD_HW_BASE_PTR_REG: base address of SD HW configuration registers (&ICSSG_PRUn_SD_CLK_SEL_REG0)
;
config_oc_osr:
        ;Load fast detect enable bits
        LBCO   &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_EN_FD_OFFSET, 1
        LDI    TEMP_REG0, 0
        ; Load TR1.w0 <- SDFM_CFG_SD_CH_ID = ;LDI      TEMP_REG1.w0, 0x210 =001100010000
        LBCO    &TEMP_REG1.w0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_CH_ID_OFFSET,  SDFM_CFG_SD_CH_ID_SZ

        LOOP    config_osr_loop_end, ICSSG_NUM_SD_CH_FW ; loop over SD channels
        AND     TEMP_REG2.b0, TEMP_REG1.b0, BF_SD_CH0_ID_MASK   ; LS byte is ID for Ch0
        QBNE            SDFM_SKIP0_CH0, TEMP_REG2.b0,	SD_CH0_ID
        ; Load TR0.b0 <-  SDFM_CFG_OSR load osr from DMEM for ch0
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_OSR_OFFSET,  SDFM_CFG_OSR_SZ

        QBBC   SDFM_SKIP0_CH2, TEMP_REG3.b0, 0
       ;configure fast detect one count and enable fast detect
        LBCO   &FAST_WINDOW_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH0_FD_WD_REG_OFFSET, 1
        LBCO   &FAST_ONE_MIN_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH0_FD_ONE_MIN_REG_OFFSET, 1
        LBCO   &FAST_ONE_MAX_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH0_FD_ONE_MAX_REG_OFFSET, 1

        MOV     TEMP_REG0.b1, FAST_WINDOW_REG
        LSL     TEMP_REG0.b2, FAST_ONE_MIN_REG, 3
        OR      TEMP_REG0.b1, TEMP_REG0.b2, TEMP_REG0.b1
        LSL     TEMP_REG0.b2, FAST_ONE_MAX_REG, 1
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41;clear max & min threshold hit
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x80;enable fast detec

        JMP    SDFM_SKIP0_CH2
SDFM_SKIP0_CH0:
        QBNE            SDFM_SKIP0_CH1, TEMP_REG2.b0,	SD_CH1_ID
        ; Load TR0.b0 <-  SDFM_CFG_OSR load osr from DMEM for ch1
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_OSR_OFFSET,  SDFM_CFG_OSR_SZ

         
        QBBC   SDFM_SKIP0_CH2, TEMP_REG3.b0, 1
        ;configure fast detect one count and enable fast detect
        LBCO   &FAST_WINDOW_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH1_FD_WD_REG_OFFSET, 1
        LBCO   &FAST_ONE_MIN_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH1_FD_ONE_MIN_REG_OFFSET, 1
        LBCO   &FAST_ONE_MAX_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH1_FD_ONE_MAX_REG_OFFSET, 1

        MOV     TEMP_REG0.b1, FAST_WINDOW_REG
        LSL     TEMP_REG0.b2, FAST_ONE_MIN_REG, 3
        OR      TEMP_REG0.b1, TEMP_REG0.b2, TEMP_REG0.b1
        LSL     TEMP_REG0.b2, FAST_ONE_MAX_REG, 1
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41;clear max & MIN threshold hit
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x80;enable fast detect
    

        JMP    SDFM_SKIP0_CH2
SDFM_SKIP0_CH1:
        QBNE            SDFM_SKIP0_CH2, TEMP_REG2.b0,	SD_CH2_ID
        ; Load TR0.b0 <-  SDFM_CFG_OSR load osr from DMEM for ch2
        LBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_OSR_OFFSET,  SDFM_CFG_OSR_SZ
        
        QBBC   SDFM_SKIP0_CH2, TEMP_REG3.b0, 2
        ;configure fast detect one count and enable fast detect
        LBCO   &FAST_WINDOW_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH2_FD_WD_REG_OFFSET, 1
        LBCO   &FAST_ONE_MIN_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH2_FD_ONE_MIN_REG_OFFSET, 1
        LBCO   &FAST_ONE_MAX_REG, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_CH2_FD_ONE_MAX_REG_OFFSET, 1

        MOV     TEMP_REG0.b1, FAST_WINDOW_REG
        LSL     TEMP_REG0.b2, FAST_ONE_MIN_REG, 3
        OR      TEMP_REG0.b1, TEMP_REG0.b2, TEMP_REG0.b1
        LSL     TEMP_REG0.b2, FAST_ONE_MAX_REG, 1
        ;clear max & min threshold hit
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41
        ;fast detect is enabled at a later stage
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x80
        

SDFM_SKIP0_CH2:

        LSL     TEMP_REG2, TEMP_REG2, 3                         ; (ChID*8) bytes to Byte0 PRUn_SD_SAMPLE_SIZE_REG(ChID)
        ADD     TEMP_REG2, TEMP_REG2, 4                         ; 4 bytes added for Byte0 PRUn_SD_SAMPLE_SIZE_REG0

        SBBO    &TEMP_REG0, SD_HW_BASE_PTR_REG, TEMP_REG2, 3 ; TR0.b0 -> PRUn_SD_SAMPLE_SIZE_REG(ChID)
        LSR     TEMP_REG1, TEMP_REG1, 4                         ; LS byte is ID for Ch(i+1) ; FL fix me
config_osr_loop_end:

        JMP     RET_ADDR_REG


;
; Configure SD channels. For SD channels, initialize:
;   ACC select = ACC3
;   Clock inversion
;   Clock source: pr1_pru<n>_pru_r31_in[16] Primary Input
;
;   CT_PRU_ICSSG_LOC_DMEM: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;   SD_HW_BASE_PTR_REG: base address of SD HW configuration registers (&ICSSG_PRUn_SD_CLK_SEL_REG0)
;
;
config_sd_ch:
        ;load fast detect enable bit
        LBCO   &TEMP_REG3, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_EN_FD_OFFSET, 1
        LDI    TEMP_REG0, 0
        ; Load TR1.w0 <- SDFM_CFG_SD_CH_ID ;LDI      TEMP_REG1.w0, 0x210 =001100010000

        LBCO    &TEMP_REG1.w0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_CH_ID_OFFSET,  SDFM_CFG_SD_CH_ID_SZ

        LOOP    config_sd_ch_loop_end, ICSSG_NUM_SD_CH_FW   ; loop over SD channels
        AND     TEMP_REG2.b0, TEMP_REG1.b0, BF_SD_CH0_ID_MASK       ; LS byte is ID for Chi
        QBNE            SDFM_SKIP1_CH0, TEMP_REG2.b0,	SD_CH0_ID
        ; Select clock inversion
        LDI     TEMP_REG0.w0, 0;
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_CLOCK_INVERSION_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TEMP_REG0.b0, TEMP_REG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_CLOCK_SOURCE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

        ; select to ACC source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_FILTER_TYPE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

        QBBC   SDFM_SKIP1_CH2, TEMP_REG3.b0, 0
        ; configure fast detect zero count
        LBCO    &FAST_ZERO_MIN_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_FD_ZERO_MIN_REG_OFFSET, 1
        LBCO    &FAST_ZERO_MAX_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH0_FD_ZERO_MAX_REG_OFFSET, 1

        LSL     TEMP_REG0.b1, FAST_ZERO_MIN_REG, 3             ; r0.b1=120= 0111 1000
        LSL     TEMP_REG0.b2, FAST_ZERO_MAX_REG, 1 ; r0.b2=30
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41             ; clear hit flags; r0.b2 = 0101 1111 = 95
        

        JMP    SDFM_SKIP1_CH2
SDFM_SKIP1_CH0:
        QBNE            SDFM_SKIP1_CH1, TEMP_REG2.b0,	SD_CH1_ID
        ; Select clock inversion
        LDI     TEMP_REG0.w0, 0;
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_CLOCK_INVERSION_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TEMP_REG0.b0, TEMP_REG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_CLOCK_SOURCE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

        ; select to ACC source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_FILTER_TYPE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

         QBBC   SDFM_SKIP1_CH2, TEMP_REG3.b0, 1
        ; configure fast detect zero count
        LBCO    &FAST_ZERO_MIN_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_FD_ZERO_MIN_REG_OFFSET, 1
        LBCO    &FAST_ZERO_MAX_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH1_FD_ZERO_MAX_REG_OFFSET, 1

        LSL     TEMP_REG0.b1, FAST_ZERO_MIN_REG, 3             ; r0.b1=120= 0111 1000
        LSL     TEMP_REG0.b2, FAST_ZERO_MAX_REG, 1 ; r0.b2=30
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41             ; clear hit flags; r0.b2 = 0101 1111 = 95
      

        JMP    SDFM_SKIP1_CH2
SDFM_SKIP1_CH1:
        QBNE            SDFM_SKIP1_CH2, TEMP_REG2.b0,	SD_CH2_ID
        ; Select clock inversion
        LDI     TEMP_REG0.w0, 0;
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_CLOCK_INVERSION_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TEMP_REG0.b0, TEMP_REG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_CLOCK_SOURCE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

        ; select to ACC source
        LBCO    &TEMP_REG0.w2, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_FILTER_TYPE_OFFSET, 1
        LSL     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TEMP_REG0.b2, TEMP_REG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TEMP_REG0.b0, TEMP_REG0.b0, TEMP_REG0.b2

        QBBC   SDFM_SKIP1_CH2, TEMP_REG3.b0, 2
        ; configure fast detect zero count
        LBCO    &FAST_ZERO_MIN_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_FD_ZERO_MIN_REG_OFFSET, 1
        LBCO    &FAST_ZERO_MAX_REG, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_CH2_FD_ZERO_MAX_REG_OFFSET, 1

        LSL     TEMP_REG0.b1, FAST_ZERO_MIN_REG, 3             ; r0.b1=120= 0111 1000
        LSL     TEMP_REG0.b2, FAST_ZERO_MAX_REG, 1 ; r0.b2=30
        OR      TEMP_REG0.b2, TEMP_REG0.b2, 0x41             ; clear hit flags; r0.b2 = 0101 1111 = 95
      

SDFM_SKIP1_CH2:
       ;configure source clock, clock inversion & ACC source for all connected channels
        LSL     TEMP_REG2, TEMP_REG2, 3                             ; (ChID*8) bytes to Byte0 PRUn_SD_CLK_SEL_REG(ChID)
        ADD     TEMP_REG2, TEMP_REG2, 0                             ; 0 bytes to Byte0 PRUn_SD_CLK_SEL_REG0

        SBBO    &TEMP_REG0, SD_HW_BASE_PTR_REG, TEMP_REG2, 3     ; TR0.b0 -> PRUn_SD_CLK_SEL_REGi
        LSR     TEMP_REG1, TEMP_REG1, 4                             ; LS byte is ID for Ch(i+1) ; FL fix me
config_sd_ch_loop_end:

        JMP     RET_ADDR_REG



;
; Reset SD channel hardware
;
;   CT_PRU_ICSSG_LOC_DMEM: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;
reset_sd_ch_hw:
        ; Load T01.w0 <- SDFM_CFG_SD_CH_ID
        ;LDI    TEMP_REG0.w0, 0x210
        LBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_CH_ID_OFFSET,  SDFM_CFG_SD_CH_ID_SZ

        LOOP    reset_sd_ch_hw_loop_end, ICSSG_NUM_SD_CH_FW ; loop over SD channels
        AND     TEMP_REG1.b0, TEMP_REG0.b0, BF_SD_CH0_ID_MASK       ; LS byte is ID for Chi

        ; Set R30[29-26]:channel_select = channel ID.
        ; Set R30[25]:channel_en=1 (set Global Channel enable).
        LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2                       ; TR1.b0 = TR1.b0<<2 = (Ch ID)<<2
        SET     TEMP_REG1.b0.t1                                 ; R30[25] channel_enable=1
        MOV     R30.b3, TEMP_REG1.b0                            ; R30.b3 = TR1.b0
                                                            ; select SD Channel (Ch ID)
        LSR     TEMP_REG0, TEMP_REG0, 4                             ; LS byte is ID for Ch(i+1) ; FL fix me
        SET     R31.t23                                     ; R31[23] re_init=1
reset_sd_ch_hw_loop_end:

        JMP     RET_ADDR_REG


    
;
;PWM trip zone block configuration
config_pwm_trip:
       ;set trip mask       
       ;PWM0 register offset 
       LDI    TEMP_REG1, ICSSG_CFG_PWM0
       LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
       QBNE            SDFM_MASK_SKIP1_CH0,   SD_CH0_ID, 0
       OR TEMP_REG0.b1, TEMP_REG0.b1, 1
SDFM_MASK_SKIP1_CH0
       QBNE            SDFM_MASK_SKIP1_CH1,   SD_CH1_ID, 1
       OR TEMP_REG0.b1, TEMP_REG0.b1, 2
SDFM_MASK_SKIP1_CH1       
        QBNE            SDFM_MASK_SKIP1_CH2, SD_CH2_ID,	2
        OR TEMP_REG0.b1, TEMP_REG0.b1, 4
SDFM_MASK_SKIP1_CH2:           
       SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4

       JMP     RET_ADDR_REG
      

;Phase delay measurement
; Measure Phase Difference between MCLK and MDATA
; PRU mode is GPI mode (default)
;     1)Waits for rising edge of DATA using wbs instruction
;        ->GPO1 for SD_D
;     2)check status of sd clock pin when data line is high. 
;        ->GPO16 for SD_clock
;     3)if clock line is high then call falling edge macro otherwise raising edge macro
;   -> macro calcultes time between rising edge of data and upcoming nearest clock edge (rising or falling)
;   -> store 8 times calculted time into DMEM
SDFM_CLOCK_PHASE_COMPENSATION:
        ;decide mask 
        LDI32 TEMP_REG1, SDFM_11_MASK
        ; waiting zero
        wbc  R31.b0, 1
        ;waiting for rising edge of sd data
        wbs  R31.b0, 1
        ; check nereset clock edge from starting point of bit
        AND  TEMP_REG0, R31, TEMP_REG1
        ;Max value
        LDI   TEMP_REG2, 0
        QBEQ   DELAY_CAL_FOR_FALLING_EDGE, TEMP_REG0, TEMP_REG1
        LDI TEMP_REG1, 0

        LOOP    SDFM_CLOCK_PHASE_COMPENSATION_LOOP, 8
        M_SDFM_PHASE_DELAY_FOR_RAISING_EDGE
SDFM_CLOCK_PHASE_COMPENSATION_LOOP:
        JMP  END_PHASE_DELAY
DELAY_CAL_FOR_FALLING_EDGE:
        LDI TEMP_REG1, 0
        LOOP    SDFM_CLOCK_PHASE_COMPENSATION_LOOP1, 8
        M_SDFM_PHASE_DELAY_FOR_FALLING_EDGE
SDFM_CLOCK_PHASE_COMPENSATION_LOOP1:
END_PHASE_DELAY:
        ;storing phase delay (8 times) and edge status in DMEM 
        ;final result in TEMP_REG1 register
        LSR    TEMP_REG0, TEMP_REG1.w0, 3
        SUB  TEMP_REG0, TEMP_REG2,TEMP_REG0
        QBLT   SDFM_CLOCK_PHASE_COMPENSATION, TEMP_REG0, 1
        SBCO  &TEMP_REG1, CT_PRU_ICSSG_LOC_DMEM, SDFM_CFG_SD_CLOCK_PHASE_DELAY, 4
        JMP     RET_ADDR_REG

;
; Initialize SD (eCAP PWM) clock
;
; Arguments:
; CT_PRU_ICSSG_LOC_DMEM: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;
init_sd_clock:
        ; Set eCAP PWM mode
        LDI32   TEMP_REG0, (SYNCI_EN_VAL<<SYNCI_EN_SHIFT) | (SYNCO_SEL_VAL<<SYNCO_SEL_SHIFT) | (CAP_APWM_VAL<<CAP_APWM_SHIFT) | (APWMPOL_VAL<<APWMPOL_SHIFT)
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4

        ; Load TR0.w0 <-  SDFM_CFG_SD_CLK
        ; Load PWM devider for SD clock from DMEM
        LBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_LOC_DMEM,  SDFM_CFG_SD_CLK_OFFSET,  SDFM_CFG_SD_CLK_SZ

        ; Set period count
        SUB     TEMP_REG1, TEMP_REG0.b0, 1  ; TR1 = SD_PRD_CLOCKS - 1
        SBCO    &TEMP_REG1, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CAP1, 4

        ; Compute & set Duty Cycle count.
        ; Divide period count by 2, biased rounding.
        ADD     TEMP_REG1, TEMP_REG0.b0, 1  ; TR1 = SD_PRD_CLOCKS + 1
        LSR     TEMP_REG1, TEMP_REG1, 1     ; TR1 = (SD_PRD_CLOCKS+1)/2
        SBCO    &TEMP_REG1, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CAP2, 4

        LDI     TEMP_REG0, 0x0
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CNTPHS, 4 ; clear counter phase
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_TSCNT, 4  ; reset eCAP PWM Counter

        ; Enable eCAP PWM
        LBCO    &TEMP_REG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4
        SET     TEMP_REG0, TEMP_REG0, TSCNTSTP_BN   ; ICSSG_ECCTL2_ECCTL1:TSCNTSTP=1
        SBCO    &TEMP_REG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4

        JMP     RET_ADDR_REG

