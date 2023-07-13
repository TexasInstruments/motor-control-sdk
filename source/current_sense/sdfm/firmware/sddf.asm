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
        .include "sddf.h"
        .include "sddf_macros.h"


; Compile-time Host event for SDDF samples available
; R31 event interface mapping, add pru<n>_r31_vec_valid to system event number, <sysevt> + 1<<5
    .if $isdefed("SDDF_PRU_CORE")
TRIGGER_HOST_SDDF_IRQ   .set PRU_TRIGGER_HOST_SDDF_EVT + 16
    .elseif $isdefed("SDDF_RTU_CORE")
TRIGGER_HOST_SDDF_IRQ   .set RTU_TRIGGER_HOST_SDDF_EVT + 16
    .endif

;SPAD Bank for SD Ch context storage
BANK_CTXT_NC               .set BANK0
BANK_CTXT_OC               .set BANK1
;differentiator state located in BANK locations 9-17
NUM_REGS_DIFF_STATE    .set  9  ; Number of PRU registers for differentiator state
OUT_SAMP_MASK           .set 0x0FFFFFFF ; 28-bit mask applied to Integrator & Differentiator output


;registers R20 - R24
;R20: contain address of NC local output
;R21; output mask
;R22.w0 channel ID
;R22.W1 required OC Sample count for NC
;R23 DMEM base address + (0x02)
;R24 PRUx CFG base address


        ; local interleaved NC output sample buffer
OUT_SAMP_BUF:   .usect  ".outSamps", ICSSG_NUM_SD_CH_FW*4, 4
        .retain         ".outSamps"
        .retainrefs     ".outSamps"

        .def    SDDF_ENTRY  ; global entry point
        .ref    dbg_setup_pinmux

        .sect   ".text"
        .retain ".text"
        .retainrefs ".text"
SDDF_ENTRY:
        ; Clear registers R0-R30
        ZERO    &R0, 124

        ; Disable Task Manager
        .word 0x32000000

        ; Clear Task Manager status which is sticky after debug halt
        LDI     TREG0.w0, 0x0fff
        SBCO    &TREG0.w0, CT_PRU_ICSSG_TM, 0, 2
        XIN     TM_YIELD_XID, &R0.b3,1
        LDI     TREG0.w0, 0
        SBCO    &TREG0.w0, CT_PRU_ICSSG_TM, 0, 2


        ; Write C24 block index for access to FW registers
        WRITE_C24_BLK_INDEX C24_BLK_INDEX_FW_REGS_VAL

;
; Check PRU ID & set PRU ID acknowledge
;
check_pru_id:
        ; Wait for Host to update PRU ID
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CTRL, FW_REG_SDDF_CTRL_SZ ; Load TR0.b0 <- FW_REG_SDDF_CTRL
        AND     TREG0.b0, TREG0.b0, SDDF_CTRL_BF_PRU_ID_MASK                            ; Mask PRU_ID bit field in TR0.b0
        LDI     TREG1.b0, BF_PRU_ID_UNINIT<<SDDF_CTRL_BF_PRU_ID_SHIFT                   ; Load TR1.b0 <- PRU_ID_UNINIT at PRU_ID bit field location
        SUB     TREG2.b0, TREG0.b0, TREG1.b0                                            ; TR2.b0 = TR1.b0 - TR0.b0
        QBEQ    check_pru_id, TREG2.b0, 0                                               ; TR2.b0 == 0 indicates PRU_ID is uninitialized,
                                                                                        ; TR2.b0 != 0 indicates PRU_ID is initialized
        ; Set PRU ID acknowledge
        LBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_STAT, FW_REG_SDDF_STAT_SZ ; Load TR1.b0 <- FW_REG_SDDF_STAT
        AND     TREG2.b0, TREG1.b0, SDDF_STAT_BF_PRU_ID_ACK_MASK                        ; Mask PRU_ID_ACK bit field in TR1.b0
        QBEQ    check_sddf_en, TREG0.b0, TREG2.b0                                       ; TR0.b0 == TR2.b0 indicates PRU_ID and PRU_ID_ACK bit fields are equal
        LDI     TREG2.b0, SDDF_STAT_BF_PRU_ID_ACK_MASK                                  ; Load TR2.b0 <- PRU_ID_ACK mask
        NOT     TREG2.b0, TREG2.b0                                                      ; TR2.b0 = ~(PRU_ID_ACK mask)
        AND     TREG1.b0, TREG1.b0, TREG2.b0                                            ; Clear PRU_ID_ACK bit field
        OR      TREG1.b0, TREG1.b0, TREG0.b0                                            ; Insert PRU_ID into PRU_ID_ACK bit field
        SBCO    &TREG1.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_STAT, FW_REG_SDDF_STAT_SZ ; Store TR1.b0 -> FW_REG_SDDF_STAT

;
; Check SDDF global enable & set SDDF global enable acknowledge.
; If SDDF global enable not set, allow re-selection of PRU ID.
;
check_sddf_en:
        ; Check SDDF global enable
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CTRL, FW_REG_SDDF_CTRL_SZ ; Load TR0.b0 <- FW_REG_SDDF_CTRL
        QBBC    check_pru_id, TREG0.b0, SDDF_CTRL_BF_SDDF_EN_SHIFT                      ; If SDDF_EN not set, re-check PRU_ID

        ; Set SDDF global enable acknowledge
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_STAT, FW_REG_SDDF_STAT_SZ ; Load TR0.b0 <- FW_REG_SDDF_STAT
        SET     TREG0, TREG0, SDDF_CTRL_BF_SDDF_EN_SHIFT                                ; Set SDDF_EN_ACK bit
        SBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_STAT, FW_REG_SDDF_STAT_SZ ; Store FW_REG_SDDF_STAT -> TR0.b0

;
; Perform initialization
;
init_sddf:
        ; Enable XIN/XOUT shifting.
        ; Used for context & SD state save/restore in TM tasks.
        LBCO    &TREG0.b0, CT_PRU_ICSSG_CFG, ICSSG_CFG_SPPC, 1
    .if $isdefed("SDDF_PRU_CORE")
        SET     TREG0, TREG0, XFR_SHIFT_EN_BN   ; ICSSG_SPP_REG:XFR_SHIFT_EN=1
    .elseif $isdefed("SDDF_RTU_CORE")
        SET     TREG0, TREG0, RTU_XFR_SHIFT_EN   ; ICSSG_SPP_REG:RTU_XFR_SHIFT_EN=1
    .endif
        SBCO    &TREG0.b0, CT_PRU_ICSSG_CFG, ICSSG_CFG_SPPC, 1

        ; Initialize Task Manager
        JAL     RET_ADDR_REG, tm_init

        ; Enable Task Manager
        .word   0x32800000

    .if $isdefed("SDDF_PRU_CORE") ; no IEP on RTU
        ; Initialize IEP0
        JAL     RET_ADDR_REG, iep0_init
    .endif


        ; Initialize SD mode
        LDI32   TREG1, PR1_PRUn_GP_MUX_SEL_VAL<<PR1_PRUn_GP_MUX_SEL_SHIFT
        LBCO    &TREG0.b0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CTRL, FW_REG_SDDF_CTRL_SZ ; Load TR0.b0 <- FW_REG_SDDF_CTRL
        QBBS    init_pru_id1, TREG0, SDDF_CTRL_BF_PRU_ID_SHIFT                          ; Check PRU ID 0 or 1
init_pru_id0:
        SBCO    &TREG1, CT_PRU_ICSSG_CFG, ICSSG_CFG_GPCFG0, 4                           ; Initialize PRU0 SD mode
        QBA     init_sddf_cont
init_pru_id1:
        SBCO    &TREG1, CT_PRU_ICSSG_CFG, ICSSG_CFG_GPCFG1, 4                           ; Initialize PRU1 SD mode


init_sddf_cont:

        ; Reset SDDF state
        JAL     RET_ADDR_REG, reset_sddf_state

        ; Set base pointer to FW Configuration registers
        LDI     SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_IEP_CFG

        ; Set base point to SD HW registers ; (PRUx_CFG_BASE address)
        SET_SD_HW_REG_BASE_PTR SD_HW_BASE_PTR_REG

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

        ; Initialize dedicated registers:
        ;   MASK register,
        ;   Local NC output sample buffer address,
        ;   clear sample count
        LDI32   MASK_REG, OUT_SAMP_MASK
        LDI32   OUT_SAMP_BUF_REG, OUT_SAMP_BUF
        LDI     SAMP_CNT_REG, 0

    .if $isdefed("SDDF_PRU_CORE") ; no IEP on RTU
        ; Start IEP
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        SET     TREG0, TREG0, CNT_ENABLE_BN ; ICSSG_IEP_GLOBAL_CFG_REG:CNT_ENABLE=1
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
    .endif

        ; Generate OC output samples
        ;
        ; TR0.b0: ch ID
        ; TR0.b1: channel ID FW register shift amount
        ; TR0.w2: channel ID FW register

        ; Load TR0.w1 <- SDDF_CFG_SD_CH_ID
        ;LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CH_ID_OFFSET, FW_REG_SDDF_CFG_SD_CH_ID_SZ
        LBBO    &SD_CH_ID, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CH_ID_OFFSET, FW_REG_SDDF_CFG_SD_CH_ID_SZ
        ;LDI     TREG0.b1, 0 ; init Ch ID shift
        ;LDI     TREG1.w2, 0 ; init OC detect count

Waitloop:
       JMP Waitloop



ts1_oc_loop:


        ; Save/restore context
        ; restore differentiator state(R9-R17)
        xchg    BANK_CTXT_OC, &R9, 4*NUM_REGS_DIFF_STATE

        ; Clear IEP0 CMP4 events
        .if $isdefed("SDDF_PRU_CORE")
        LDI		TREG0.b0, 0x10
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_STATUS_REG, 1
        .endif

        .if $isdefed("DEBUG_CODE")
        ;Debug code  :GPIO HIGH
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        ;debug code end
        .endif

         ; update IEP0 CMP4
       .if $isdefed("SDDF_PRU_CORE")
        LDI     TREG0,0
        LBCO    &TREG0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_OC_PRD_IEP_CNT, FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_SZ
        LBCO    &TREG1, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4 ;
        MOV     CURRENT_COMP4_REG_VALUE, TREG1 ; store current cmp4 register value for check time trigger
        ADD     TREG0, TREG1, TREG0
        ;read iep counter maximum value
        LDI     TREG1, 0
        LBCO    &TREG1, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_IEP_CFG_SIM_EPWM_PRD, 2
        QBLE    UPDATE_CMP4_FOR_IEP_RESET,  TREG0, TREG1
        ;update Cmp4 with old value + next sample time value
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
        JMP     END_CMP4_UPDATE
UPDATE_CMP4_FOR_IEP_RESET:
        ;Update cmp4 according to iep reset
        SUB      TREG0, TREG0, TREG1
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4

END_CMP4_UPDATE:
       .endif

        ;select ch0, enable all channel, set SD snoop=1 & sample_counter_select=1
        LDI     R30.w2, (SD_CH0_ID<<10 | 1<<9 | 1<<6 | 1<<5)
        NOP

        ; Snoop read Ch0 sample_counter,
        ; wait for ChX sample count+1.
        AND     TREG1, R31, 0xFF ; snoop read LSB Ch0 sample_counter
wait_sample_count_incr_oc:
        AND     TREG2, R31, 0xFF ; snoop read LSB Ch0 sample_counter
        QBEQ    wait_sample_count_incr_oc, TREG2, TREG1

        ; Snoop read Ch0 ACC3
        CLR     R30.t21                 ; set SD sample_counter_select=0
        NOP
        AND     TREG0, R31, MASK_REG      ; TREG0 = Ch0 SD HW ACC3 output sample
        CLR     R30.t22                 ; set SD snoop=0

         ;select ch1, enable all channel & set SD snoop=1
        LDI     R30.w2, (SD_CH1_ID<<10 | 1<<9 | 1<<6)
        NOP

        ; Snoop read ACC3 for SD Ch1
        AND     TREG1, R31, MASK_REG    ; TREG1 = Ch1 SD HW ACC3 output sample
        CLR     R30.t22                 ; set SD snoop=0

        ;select ch2, enable all channel & set SD snoop=1
        LDI     R30.w2, (SD_CH2_ID<<10 | 1<<9 | 1<<6)
        NOP

        ; Snoop read ACC3 for SD Ch2
        AND     TREG2, R31, MASK_REG    ; TREG2 = Ch2 SD HW ACC3 output sample
        CLR     R30.t22                 ; set SD snoop=0

         ;NC sampling
         ;One NC sample after every NC_PRD_CNT number of OC sample because NC OSR = NC_PRD_CNT * OC_OSR
         ;read NC_PRD_CNT
         LDI     TREG3, 0
         LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SAMPLE_COUNT, 2
         SUB     TREG3, TREG3, 1
         QBLE    read_nc_frame, SAMP_CNT_REG, TREG3
         ADD     SAMP_CNT_REG, SAMP_CNT_REG, 1   ; increment OC sample count
         QBA     skip_nc_frame
read_nc_frame:

          .if $isdefed("DEBUG_CODE")
        ;Debug code  :GPIO HIGH
         LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
         LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        ;debug code end
        .endif

        ; restore differentiator state(R9-R17)
        xchg    BANK_CTXT_NC, &R9, 4*NUM_REGS_DIFF_STATE

         ; Execute SINC3 differentiation for Ch0
         MOV     DN0, TREG0 ; DN0 = Ch1 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0
        ; Save NC output sample to local output sample buffer
        SBBO    &CN5, OUT_SAMP_BUF_REG, 0, 4


        ; Execute SINC3 differentiation for Ch1
        MOV     DN0, TREG1 ; DN0 = Ch1 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1
        ; Save output sample to local output sample buffer
       SBBO    &CN5, OUT_SAMP_BUF_REG, 4, 4

        ; Execute SINC3 differentiation for Ch2
        MOV     DN0, TREG2 ; DN0 = Ch2 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2
        ; Save output sample to local output sample buffer
        SBBO    &CN5, OUT_SAMP_BUF_REG, 8, 4


        .if $isdefed("DEBUG_CODE")
        ;Debug code
        ; Write local interleaved output samples to Host buffer address
        LBBO    &TREG3, OUT_SAMP_BUF_REG, 0, ICSSG_NUM_SD_CH_FW*4
        SBBO    &TREG3,  SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_OUT_SAMP_BUF_OFFSET, ICSSG_NUM_SD_CH_FW*4
        ; Trigger interrupt
        LDI     R31.w0, TRIGGER_HOST_SDDF_IRQ
        .endif


         ;sample read
        LBCO  &TREG3, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME, FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME_SZ
        QBLE    sample_time_before_sample, CURRENT_COMP4_REG_VALUE, TREG3
        SUB  CURRENT_COMP4_REG_VALUE, TREG3, CURRENT_COMP4_REG_VALUE
        LBCO  &TREG3, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_NC_PRD_IEP_CNT, FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_SZ
        ;LDI  TREG3, 0x780
        LSR  TREG3, TREG3, 1
        QBGE    skip_sample_read, TREG3, CURRENT_COMP4_REG_VALUE
        QBA     trigger_intruppt
sample_time_before_sample:
        SUB  CURRENT_COMP4_REG_VALUE, CURRENT_COMP4_REG_VALUE, TREG3
        LBCO  &TREG3, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_NC_PRD_IEP_CNT, FW_REG_SDDF_CFG_OC_PRD_IEP_CNT_SZ
        ;LDI  TREG3, 0x780
         LSR  TREG3, TREG3, 1
        QBLE    skip_sample_read,  CURRENT_COMP4_REG_VALUE, TREG3
        QBA     trigger_intruppt
trigger_intruppt:

         .if $isdefed("DEBUG_CODE")
          ;GPIO HIGH
           LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
          LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
          SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
         .endif
        ; Write local interleaved output samples to Host buffer address
        LBBO    &TREG3, OUT_SAMP_BUF_REG, 0, ICSSG_NUM_SD_CH_FW*4
        SBBO    &TREG3,  SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_OUT_SAMP_BUF_OFFSET, ICSSG_NUM_SD_CH_FW*4
        ; Trigger interrupt
        LDI     R31.w0, TRIGGER_HOST_SDDF_IRQ

         .if $isdefed("DEBUG_CODE")
         ;GPIO LOW
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        .endif

skip_sample_read:



         ;restore/save contex
         ;save differentiator state(R9-R17)
         xchg    BANK_CTXT_NC, &R9, 4*NUM_REGS_DIFF_STATE

         .if $isdefed("DEBUG_CODE")
        ;GPIO LOW
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        .endif

        LDI     SAMP_CNT_REG, 0 ; reset  sample count
skip_nc_frame:

        ; Execute SINC3 differentiation for Ch0
        MOV     DN0, TREG0 ; DN0 = Ch1 SD HW ACC3 output sample
        MOV     TREG0, TREG1  ;move Ch1 data into TREG0
        MOV     TREG1, TREG2  ; Move ch2 data into TREG1
        M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0


        ;Comparator for Ch0
        MOV     TREG2, CN5

        .if $isdefed("DEBUG_CODE")
        SBBO    &CN5, OUT_SAMP_BUF_REG, 0, 4
        .endif

        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBBO    &OC_HIGH_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_HIGH_THR_CH0_OFFSET, FW_REG_SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBBO    &OC_LOW_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_LOW_THR_CH0_OFFSET, FW_REG_SDFM_CFG_OC_LOW_THR_SZ
        ;Load the zero crossing threshold value for current channel
        LBBO    &OC_ZC_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_ZC_THR_CH0_OFFSET, FW_REG_SDFM_CFG_ZC_THR_SZ

        ;Check if the comparator is enabled for current channel
        LBBO    &COMPARATOR_EN, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_SD_EN_COMP_OFFSET, FW_REG_SDFM_CFG_EN_COMP_SZ
        QBBC    comp_ch0_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH0_EN_COMP_BIT
        ;Check if the sample value is greater than the high threshold
        QBGE    over_threshold_start_ch0, OC_HIGH_THR, TREG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch0, OC_HIGH_THR, TREG2
low_threshold_ch0_check:
        ;Check if the sample value is greater than the low threshold
        QBGE    below_threshold_end_ch0, OC_LOW_THR, TREG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch0, OC_LOW_THR, TREG2
over_threshold_start_ch0:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch0_check

over_threshold_end_ch0:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch0_check

below_threshold_end_ch0:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH0_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA comp_ch0_end

below_threshold_start_ch0:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH0_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH0_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ


comp_ch0_end:


        ; Execute SINC3 differentiation for Ch1
        MOV     DN0, TREG0 ; DN0 = Ch1 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1


        ;Comparator for ch1
        MOV     TREG2, CN5

        .if $isdefed("DEBUG_CODE")
         SBBO    &CN5, OUT_SAMP_BUF_REG, 4, 4
        .endif

        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBBO    &OC_HIGH_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_HIGH_THR_CH1_OFFSET, FW_REG_SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBBO    &OC_LOW_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_LOW_THR_CH1_OFFSET, FW_REG_SDFM_CFG_OC_LOW_THR_SZ
        ;Load the zero crossing threshold value for current channel
        LBBO    &OC_ZC_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_ZC_THR_CH1_OFFSET, FW_REG_SDFM_CFG_ZC_THR_SZ

        ;Check if the comparator is enabled for current channel
        LBBO    &COMPARATOR_EN, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_SD_EN_COMP_OFFSET, FW_REG_SDFM_CFG_EN_COMP_SZ
        QBBC    comp_ch1_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH1_EN_COMP_BIT
        ;Check if the sample value is greater than the high threshold
        QBGE    over_threshold_start_ch1, OC_HIGH_THR, TREG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch1, OC_HIGH_THR, TREG2
low_threshold_ch1_check:
        ;Check if the sample value is greater than the low threshold
        QBGE    below_threshold_end_ch1, OC_LOW_THR, TREG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch1, OC_LOW_THR, TREG2
over_threshold_start_ch1:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch1_check

over_threshold_end_ch1:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch1_check

below_threshold_end_ch1:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH1_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA     comp_ch1_end

below_threshold_start_ch1:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH1_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ

comp_ch1_end:

        ; Execute SINC3 differentiation for Ch2
        MOV     DN0, TREG1 ; DN0 = Ch2 SD HW ACC3 output sample
        M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2


        ;Comparator for ch2
        MOV     TREG2, CN5

        .if $isdefed("DEBUG_CODE")
        SBBO    &CN5, OUT_SAMP_BUF_REG, 8, 4
        .endif

        ;For the current channel, compare against the High threshold, Low threshold and ZC thresholds (if enabled)
        ;Load the positive threshold value for current channel
        LBBO    &OC_HIGH_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_HIGH_THR_CH2_OFFSET, FW_REG_SDFM_CFG_OC_HIGH_THR_SZ
        ;Load the positive threshold value for current channel
        LBBO    &OC_LOW_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_LOW_THR_CH2_OFFSET, FW_REG_SDFM_CFG_OC_LOW_THR_SZ
        ;Load the zero crossing threshold value for current channel
        LBBO    &OC_ZC_THR, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_OC_ZC_THR_CH2_OFFSET, FW_REG_SDFM_CFG_ZC_THR_SZ

        ;Check if the comparator is enabled for current channel
        LBBO    &COMPARATOR_EN, SDDF_CFG_BASE_PTR_REG, FW_REG_SDFM_CFG_SD_EN_COMP_OFFSET, FW_REG_SDFM_CFG_EN_COMP_SZ
        QBBC    comp_ch2_end, COMPARATOR_EN, SDFM_CFG_BF_SD_CH2_EN_COMP_BIT
        ;Check if the sample value is greater than the high threshold
        QBGE    over_threshold_start_ch2, OC_HIGH_THR, TREG2
        ;Check if the sample value is lower than the high threshold
        QBLE    over_threshold_end_ch2, OC_HIGH_THR, TREG2
        ;Check if the sample value is greater than the low threshold
low_threshold_ch2_check:
        QBGE    below_threshold_end_ch2, OC_LOW_THR, TREG2
        ;Check if the sample value is lower than the low threshold
        QBLE    below_threshold_start_ch2, OC_LOW_THR, TREG2
over_threshold_start_ch2:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_SET_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch2_check

over_threshold_end_ch2:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA low_threshold_ch2_check

below_threshold_end_ch2:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH2_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        QBA comp_ch2_end

below_threshold_start_ch2:
        LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH2_SET_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
        LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_LOW_THR_CH2_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
comp_ch2_end:

        .if $isdefed("DEBUG_CODE")
        ;Debug code
        ; Write local interleaved output samples to Host buffer address
        LBBO    &TREG3, OUT_SAMP_BUF_REG, 0, ICSSG_NUM_SD_CH_FW*4
        SBBO    &TREG3,  SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_OUT_SAMP_BUF_OFFSET, ICSSG_NUM_SD_CH_FW*4
        ; Trigger interrupt
        LDI     R31.w0, TRIGGER_HOST_SDDF_IRQ
        .endif



         .if $isdefed("DEBUG_CODE")
        ;GPIO LOW
         LBBO    &GPIO_TGL_ADDR, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_CLR_VAL_ADDR_OFFSET, FW_REG_SDFM_CFG_GPIO_CLR_ADDR_SZ
         LBBO    &TREG3, SDDF_CFG_BASE_PTR_REG, SDFM_CFG_HIGH_THR_CH1_WRITE_VAL_OFFSET, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
         SBBO    &TREG3, GPIO_TGL_ADDR, 0, FW_REG_SDFM_CFG_GPIO_VALUE_SZ
        .endif


        ; Save/restore context
        ; save differentiator state(R9-R17)
        xchg    BANK_CTXT_OC, &R9, 4*NUM_REGS_DIFF_STATE



        XIN     TM_YIELD_XID, &R0.b3,1  ; exit task after two instructions/cycles
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
        LDI     TREG0.b0, (1b<<3|0b<<2|11b<<0)           ; enable  T1_s1
        SBCO    &TREG0.b0, CT_PRU_ICSSG_TM, 0, 1

        ;set T1_S1 address
        LDI     TREG0.w0, $CODE(ts1_oc_loop)
        SBCO    &TREG0.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_PC_S1, 2

        ; Set Task triggers
        ; set T1_S1 trigger to IEP0 CMP4 event = 20
        LDI     TREG0.w0, (COMP4_EVENT_NUMBER<<COMP_EVENT_FOUR_SIFT)
        SBCO    &TREG0.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_GEN_CFG1, 2

        JMP     RET_ADDR_REG


; Initialize IEP0.
;
iep0_init:
        ; Disable IEP0 counter
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        AND     TREG0.b0, TREG0.b0, 0xFE
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1

        ; Set IEP0 counter to zero
        LDI     TREG0, 0
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_COUNT_REG1, 4
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_COUNT_REG0, 4

        ; Clear IEP0  CMP4 events
        LDI		TREG0.b0, 0x10
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_STATUS_REG, 1

        ; Write IEP0 default increment
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1
        AND     TREG0.b0, TREG0.b0, 0x0F
        OR      TREG0.b0, TREG0.b0, IEP_DEFAULT_INC<<DEFAULT_INC_BN
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_GLOBAL_CFG_REG, 1

        ;Enable IEP0 counter on EPWM0 SYNC0 event

        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0_0x100, ICSSG_IEP_PWM_REG, 1
        SET     TREG0.b0.t0 ; IEP_PWM_REG:PWM0_RST_CNT_EN = 1, enable IEP0 counter reset on EPWM0 SYNCO event
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0_0x100, ICSSG_IEP_PWM_REG, 1


        ; Initialize Trigger sample time for OC
        ; Set IEP0 CMP4 value: IEP0_CMP4_REG1:REG0 = 0:TRIG_SAMPLE_TIME
        LDI     TREG0, 0
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG1, 4
        LBCO    &TREG0, CT_PRU_ICSSG_LOC_DMEM, FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME, FW_REG_SDDF_CFG_TRIG_SAMPLE_TIME_SZ
        LDI     TREG1, SDDF_CFG_TRIG_SAMP_TIME_BF_TRIG_SAMP_TIME_MASK
        AND     TREG0, TREG1, TREG0
        SUB     TREG0, TREG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP0
        SBCO    &TREG0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP4_REG0, 4
        ; Enable IEP0 CMP4
        LBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_CFG_REG, 1  ; TR0 <- Byte0 ICSSG_CMP_CFG_REG
        SET     TREG0.t5    ; CMP_EN[5]=1 => CMP4 enabled
        SBCO    &TREG0.b0, CT_PRU_ICSSG_IEP0, ICSSG_IEP_CMP_CFG_REG, 1  ; TR0 -> ICSSG_CMP_CFG_REG Byte0



        JMP     RET_ADDR_REG


;
; Reset SDDF state
;
; Arguments: None
;
reset_sddf_state:
        ZERO    &R9, (4*NUM_REGS_DIFF_STATE)
        XOUT    BANK_CTXT_OC, &R9, (4*NUM_REGS_DIFF_STATE);clear OC  differentiator state

        XOUT    BANK_CTXT_NC, &R9, (4*NUM_REGS_DIFF_STATE) ;clear NC  differentiator state
        JMP     RET_ADDR_REG


;
; Configure OC Over Sampling Rate for SD channels
;
; Arguments:
;   SDDF_CFG_BASE_PTR_REG: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;   SD_HW_BASE_PTR_REG: base address of SD HW configuration registers (&ICSSG_PRUn_SD_CLK_SEL_REG0)
;
config_oc_osr:

        ; Load TR1.w0 <- SDDF_CFG_SD_CH_ID = ;LDI      TREG1.w0, 0x210 =001100010000

        LBBO    &TREG1.w0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CH_ID_OFFSET, FW_REG_SDDF_CFG_SD_CH_ID_SZ

        LOOP    config_osr_loop_end, ICSSG_NUM_SD_CH_FW ; loop over SD channels
        AND     TREG2.b0, TREG1.b0, BF_SD_CH0_ID_MASK   ; LS byte is ID for Ch0
        QBNE            SDFM_SKIP0_CH0, TREG2.b0,	0
        ; Load TR0.b0 <- FW_REG_SDDF_CFG_OSR load osr from DMEM for ch0
        LBBO    &TREG0.b0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH0_OSR_OFFSET, FW_REG_SDDF_CFG_OSR_SZ
        JMP    SDFM_SKIP0_CH2
SDFM_SKIP0_CH0:
        QBNE            SDFM_SKIP0_CH1, TREG2.b0,	1
        ; Load TR0.b0 <- FW_REG_SDDF_CFG_OSR load osr from DMEM for ch1
        LBBO    &TREG0.b0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH1_OSR_OFFSET, FW_REG_SDDF_CFG_OSR_SZ
        JMP    SDFM_SKIP0_CH2
SDFM_SKIP0_CH1:
        QBNE            SDFM_SKIP0_CH2, TREG2.b0,	2
        ; Load TR0.b0 <- FW_REG_SDDF_CFG_OSR load osr from DMEM for ch2
        LBBO    &TREG0.b0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH2_OSR_OFFSET, FW_REG_SDDF_CFG_OSR_SZ
SDFM_SKIP0_CH2:

        LSL     TREG2, TREG2, 3                         ; (ChID*8) bytes to Byte0 PRUn_SD_SAMPLE_SIZE_REG(ChID)
        ADD     TREG2, TREG2, 4                         ; 4 bytes added for Byte0 PRUn_SD_SAMPLE_SIZE_REG0

        SBBO    &TREG0.b0, SD_HW_BASE_PTR_REG, TREG2, 1 ; TR0.b0 -> PRUn_SD_SAMPLE_SIZE_REG(ChID)
        LSR     TREG1, TREG1, 4                         ; LS byte is ID for Ch(i+1) ; FL fix me
config_osr_loop_end:

        JMP     RET_ADDR_REG


;
; Configure SD channels. For SD channels, initialize:
;   ACC select = ACC3
;   Clock inversion
;   Clock source: pr1_pru<n>_pru_r31_in[16] Primary Input
;
;   SDDF_CFG_BASE_PTR_REG: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;   SD_HW_BASE_PTR_REG: base address of SD HW configuration registers (&ICSSG_PRUn_SD_CLK_SEL_REG0)
;
config_sd_ch:
        ; Load TR1.w0 <- SDDF_CFG_SD_CH_ID ;LDI      TREG1.w0, 0x210 =001100010000

        LBBO    &TREG1.w0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CH_ID_OFFSET, FW_REG_SDDF_CFG_SD_CH_ID_SZ

        LOOP    config_sd_ch_loop_end, ICSSG_NUM_SD_CH_FW   ; loop over SD channels
        AND     TREG2.b0, TREG1.b0, BF_SD_CH0_ID_MASK       ; LS byte is ID for Chi
        QBNE            SDFM_SKIP1_CH0, TREG2.b0,	0
        ; Select clock inversion
        LDI     TREG0.w0, 0;
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH0_CLOCK_INVERSION_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TREG0.b0, TREG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH0_CLOCK_SOURCE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2

        ; select to ACC source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH0_FILTER_TYPE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2
        JMP    SDFM_SKIP1_CH2
SDFM_SKIP1_CH0:
        QBNE            SDFM_SKIP1_CH1, TREG2.b0,	1
        ; Select clock inversion
        LDI     TREG0.w0, 0;
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH1_CLOCK_INVERSION_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TREG0.b0, TREG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH1_CLOCK_SOURCE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2

        ; select to ACC source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH1_FILTER_TYPE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2
        JMP    SDFM_SKIP1_CH2
SDFM_SKIP1_CH1:
        QBNE            SDFM_SKIP1_CH2, TREG2.b0,	2
        ; Select clock inversion
        LDI     TREG0.w0, 0;
        LBBO    &TREG0.w1, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH2_CLOCK_INVERSION_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_INVi_SHIFT
        AND     TREG0.b0, TREG0.b2, PRUn_SD_CLK_INVi_MASK<<PRUn_SD_CLK_INVi_SHIFT   ; TR1.b2 = SD_CLK_INV

        ; select clock source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH2_CLOCK_SOURCE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_CLK_SELi_MASK<<PRUn_SD_CLK_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2

        ; select to ACC source
        LBBO    &TREG0.w2, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_CH2_FILTER_TYPE_OFFSET, 1
        LSL     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_SHIFT
        AND     TREG0.b2, TREG0.b2, PRUn_SD_ACC_SELi_MASK<<PRUn_SD_ACC_SELi_SHIFT
        OR      TREG0.b0, TREG0.b0, TREG0.b2
SDFM_SKIP1_CH2:
       ;configure source clock, clock inversion & ACC source for all connected channels
        LSL     TREG2, TREG2, 3                             ; (ChID*8) bytes to Byte0 PRUn_SD_CLK_SEL_REG(ChID)
        ADD     TREG2, TREG2, 0                             ; 0 bytes to Byte0 PRUn_SD_CLK_SEL_REG0

        SBBO    &TREG0.b0, SD_HW_BASE_PTR_REG, TREG2, 1     ; TR0.b0 -> PRUn_SD_CLK_SEL_REGi
        LSR     TREG1, TREG1, 4                             ; LS byte is ID for Ch(i+1) ; FL fix me
config_sd_ch_loop_end:

        JMP     RET_ADDR_REG


;
; Reset SD channel hardware
;
;   SDDF_CFG_BASE_PTR_REG: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;
reset_sd_ch_hw:
        ; Load T01.w0 <- SDDF_CFG_SD_CH_ID
        ;LDI    TREG0.w0, 0x210
        LBBO    &TREG0.w0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CH_ID_OFFSET, FW_REG_SDDF_CFG_SD_CH_ID_SZ

        LOOP    reset_sd_ch_hw_loop_end, ICSSG_NUM_SD_CH_FW ; loop over SD channels
        AND     TREG1.b0, TREG0.b0, BF_SD_CH0_ID_MASK       ; LS byte is ID for Chi

        ; Set R30[29-26]:channel_select = channel ID.
        ; Set R30[25]:channel_en=1 (set Global Channel enable).
        LSL     TREG1.b0, TREG1.b0, 2                       ; TR1.b0 = TR1.b0<<2 = (Ch ID)<<2
        SET     TREG1.b0.t1                                 ; R30[25] channel_enable=1
        MOV     R30.b3, TREG1.b0                            ; R30.b3 = TR1.b0
                                                            ; select SD Channel (Ch ID)
        LSR     TREG0, TREG0, 4                             ; LS byte is ID for Ch(i+1) ; FL fix me
        SET     R31.t23                                     ; R31[23] re_init=1
reset_sd_ch_hw_loop_end:

        JMP     RET_ADDR_REG


;
; Initialize SD (eCAP PWM) clock
;
; Arguments:
;   SDDF_CFG_BASE_PTR_REG: base address of SD Configuration registers (&IEP_CFG_EPWM_PRD)
;
init_sd_clock:
        ; Set eCAP PWM mode
        LDI32   TREG0, (SYNCI_EN_VAL<<SYNCI_EN_SHIFT) | (SYNCO_SEL_VAL<<SYNCO_SEL_SHIFT) | (CAP_APWM_VAL<<CAP_APWM_SHIFT) | (APWMPOL_VAL<<APWMPOL_SHIFT)
        SBCO    &TREG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4

        ; Load TR0.w0 <- FW_REG_SDDF_CFG_SD_CLK
        ; Load PWM devider for SD clock from DMEM
        LBBO    &TREG0.w0, SDDF_CFG_BASE_PTR_REG, FW_REG_SDDF_CFG_SD_CLK_OFFSET, FW_REG_SDDF_CFG_SD_CLK_SZ

        ; Set period count
        SUB     TREG1, TREG0.b0, 1  ; TR1 = SD_PRD_CLOCKS - 1
        SBCO    &TREG1, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CAP1, 4

        ; Compute & set Duty Cycle count.
        ; Divide period count by 2, biased rounding.
        ADD     TREG1, TREG0.b0, 1  ; TR1 = SD_PRD_CLOCKS + 1
        LSR     TREG1, TREG1, 1     ; TR1 = (SD_PRD_CLOCKS+1)/2
        SBCO    &TREG1, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CAP2, 4

        LDI     TREG0, 0x0
        SBCO    &TREG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_CNTPHS, 4 ; clear counter phase
        SBCO    &TREG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_TSCNT, 4  ; reset eCAP PWM Counter

        ; Enable eCAP PWM
        LBCO    &TREG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4
        SET     TREG0, TREG0, TSCNTSTP_BN   ; ICSSG_ECCTL2_ECCTL1:TSCNTSTP=1
        SBCO    &TREG0, CT_PRU_ICSSG_ECAP, ICSSG_eCAP_ECCTL1, 4

        JMP     RET_ADDR_REG
