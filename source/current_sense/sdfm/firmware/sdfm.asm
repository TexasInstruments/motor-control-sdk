; Copyright (c) 2025, Texas Instruments Incorporated
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
;       file: sdfm.asm
;
;

;********************************** includes **************************************

        .cdecls C,NOLIST
%{
        #include "icssg_sdfm.h"
%}
        .include "sdfm.h"
        .include "sdfm_macros.h"
        .include "firmware_version.h"
        .include "../../../../mcu_plus_sdk/source/pru_io/firmware/common/icss_regs.inc"
	    .include "../../../../mcu_plus_sdk/source/pru_io/firmware/common/icss_cfg_regs.inc"

;***********************************************************************************


;********************************************* defines *******************************************

; Compile-time Host event for SDFM samples available
; R31 event interface mapping, add pru<n>_r31_vec_valid to system event number, <sysevt> + 1<<5
    .if $isdefed("SDFM_PRU_CORE")
TRIGGER_HOST_SDFM_IRQ_CH0   .set PRU_TRIGGER_HOST_SDFM_EVT_CH0 + 16
TRIGGER_HOST_SDFM_IRQ_CH1   .set PRU_TRIGGER_HOST_SDFM_EVT_CH1 + 16
TRIGGER_HOST_SDFM_IRQ_CH2   .set PRU_TRIGGER_HOST_SDFM_EVT_CH2 + 16
    .elseif $isdefed("SDFM_RTU_CORE")
TRIGGER_HOST_SDFM_IRQ_CH0   .set RTU_TRIGGER_HOST_SDFM_EVT_CH0 + 16
TRIGGER_HOST_SDFM_IRQ_CH1   .set RTU_TRIGGER_HOST_SDFM_EVT_CH1 + 16
TRIGGER_HOST_SDFM_IRQ_CH2   .set RTU_TRIGGER_HOST_SDFM_EVT_CH2 + 16
    .elseif $isdefed("SDFM_TXPRU_CORE")
TRIGGER_HOST_SDFM_IRQ_CH0   .set TXPRU_TRIGGER_HOST_SDFM_EVT_CH0 + 16
TRIGGER_HOST_SDFM_IRQ_CH1   .set TXPRU_TRIGGER_HOST_SDFM_EVT_CH1 + 16
TRIGGER_HOST_SDFM_IRQ_CH2   .set TXPRU_TRIGGER_HOST_SDFM_EVT_CH2 + 16
    .endif

    .if	$isdefed("SLICE0")
	.asg	PRU0_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU0_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT_CH0_CFG1
	.endif

	.if	$isdefed("SLICE1")
	.asg	PRU1_DMEM,		PRUx_DMEM
	.asg    ICSS_CFG_PRU1_ENDAT_CH0_CFG1, ICSS_CFG_PRUx_ENDAT_CH0_CFG1
	.endif

;SPAD Bank for SD Ch context storage
BANK_CTXT_NC               .set BANK0

;differentiator state located in BANK locations 9-17
NUM_REGS_DIFF_STATE    .set  9  ; Number of PRU registers for differentiator state
OUT_SAMP_MASK           .set 0x0FFFFFFF ; 28-bit mask applied to Integrator & Differentiator output

;Required sample for stable NC sample = NC_SAMP_CNT - 1
NC_SAMP_CNT .set 4

;***************************************************************************************************

;--------------------------------------Used Registers-------------------------------------;
;registers R20 - R24
;R20: contain address of NC local output
;R21; output mask
;R22.w0 channel ID
;R24 PRUx CFG base address
;-------------------------------------------------------------------------------------------;


;;************************************ SDFM FIRMWARE ENTRY POINT *******************************   
    .def    SDFM_ENTRY  ; global entry point    
    .sect   ".text"
    .retain ".text"
    .retainrefs ".text"

;***************************
;     *ENTRY POINT*
;***************************
SDFM_ENTRY:
    ; Clear registers R0-R30
    ZERO    &R0, 124
    ;load firmware version info to PRU DMEM
    LDI32  TEMP_REG0, ICSS_FIRMWARE_RELEASE_1
    LDI32  TEMP_REG1, ICSS_FIRMWARE_RELEASE_2
    SBCO   &TEMP_REG0, PRUx_DMEM, SDFM_FIRMWARE_VERSION_OFFSET, 8

    ; Disable Task Manager
    ;.word 0x32000000
    M_PRU_TM_DISABLE    
    ; Clear Task Manager status which is sticky after debug halt
    LDI     TEMP_REG0.w0, 0x0fff
    .if $isdefed("SDFM_TXPRU_CORE")
    SBCO    &TEMP_REG0.w0, C28, 0, 2
    .else
    SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, 0, 2
    .endif
    XIN     TM_YIELD_XID, &R0.b3,1
    LDI     TEMP_REG0.w0, 0
    .if $isdefed("SDFM_TXPRU_CORE")
    SBCO    &TEMP_REG0.w0, C28, 0, 2
    .else
    SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, 0, 2
    .endif    

    ;Write C24 block index for local PRU DMEM
    ;;M_WRITE_C24_BLK_INDEX C24_BLK_INDEX_FW_REGS_VAL

    .if $isdefed("SDFM_PHASE_DELAY_CALC")
PHASE_DELAY_CAL:
    ;check phase delay measurment active
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_CFG_SD_CH0_EN_PHASE_DELAY, 1
    QBBC    SKIP_PHASE_DELAY_CAL, TEMP_REG0.b0, 0
    JAL     RET_ADDR_REG, SDFM_CLOCK_PHASE_COMPENSATION
    ;acknowledge 
    CLR     TEMP_REG0, TEMP_REG0, 0
    SBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_CFG_SD_CH0_EN_PHASE_DELAY, 1
SKIP_PHASE_DELAY_CAL:
    .endif


;
; Check SDFM global enable & set SDFM global enable acknowledge to inform R5 core.
; If SDFM global enable not set, wait for SDFM global enable from R5.
;
CHECK_SDFM_EN:
    ;check phase delay measurment active
    .if $isdefed("SDFM_PHASE_DELAY_CALC")
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_CFG_SD_CH0_EN_PHASE_DELAY, 1
    QBBS    PHASE_DELAY_CAL, TEMP_REG0.b0, 0
    .endif
    ; Check SDFM global enable
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_EN_OFFSET, SDFM_ONE_BYTE
    QBBC    CHECK_SDFM_EN, TEMP_REG0.b0, 0                      ; If SDFM_EN not set, wait to set sdfm enable    
    ; Set SDFM global enable acknowledge
    SET     TEMP_REG0, TEMP_REG0, 0                             ; Set SDFM_EN_ACK
    SBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_EN_ACK_OFFSET, SDFM_ONE_BYTE

;
; Perform initialization
;
INIT_SDFM:

    ;read connected channel mask 
    LBCO    &TEMP_REG0, PRUx_DMEM,  SDFM_CFG_SD_CH_MASK_OFFSET,  2
    LSR  TEMP_REG0, TEMP_REG0, ICSS_PRU_SD_FIRST_CH
    MOV SD_CHANNEL_MASK, TEMP_REG0.w0

    ; Reset SDFM state, FIXME add check for snoop mode
    JAL     RET_ADDR_REG, FN_RESET_SDFM_STATE

    ; Global enable SD HW,
    ; reset SD channel HW
    JAL     RET_ADDR_REG, FN_RESET_SD_CH_HW
    SET     R30.t25 ; R30[25] channel_en = 1, all channels enabled

    
    ; Initialize dedicated registers:
    ;   MASK register,
    ;   Local NC output sample buffer address,
    ;   Clear NC sample count.
    LDI32   MASK_REG, OUT_SAMP_MASK
    LDI32   OUT_SAMP_BUF_REG, SDFM_LOCAL_OUTPUT_SAMPLE_BUFFER_OFFSET
    LDI  SAMP_CNT_REG,  0
    LBCO  &EN_DOUBLE_UPDATE,  PRUx_DMEM, SDFM_CFG_EN_DOUBLE_UPDATE,  1

    ;NC trigger mode status 
    LBCO  &EN_NC_TRIGGER_MODE, PRUx_DMEM,  SDFM_CFG_EN_NC_TRIGGER_MODE, 1
    LDI  SAMP_NAME, 0

    ;Zero cross resgiter
    LDI TEMP_REG0, 0
    LDI TEMP_REG2, SDFM_CFG_ZC_THR_EN_CH0_OFFSET
    
    LDI ZERO_CROSS_EN, 0
    LDI TEMP_REG3, 0
    LDI TEMP_REG1, SD_CH0
    
    LOOP CONF_ZC_REG_LOOP, ICSS_PRU_MAX_NUM_OF_SD_CH
    QBBC SKIP_CH_FOR_ZC, SD_CHANNEL_MASK, TEMP_REG1.b0
    LBCO &TEMP_REG3, PRUx_DMEM, TEMP_REG2, 1
      
    LSL TEMP_REG3, TEMP_REG3, TEMP_REG1
    ; OR with accumulated zero cross enable bits
    OR ZERO_CROSS_EN, ZERO_CROSS_EN, TEMP_REG3
      
SKIP_CH_FOR_ZC:
      ADD TEMP_REG2, TEMP_REG2, ICSSG_SDFM_CH_MEM_OFFSET
      ADD TEMP_REG1, TEMP_REG1, 1
CONF_ZC_REG_LOOP

    ;In trigger mode select common sinc filter for all channels
    LDI TEMP_REG2, SDFM_CFG_CH0_FILTER_TYPE_OFFSET
    
    LDI TEMP_REG1, SD_CH0
LOOP_START:
    ; Check if this channel is enabled/connected
    QBBC  SKIP_CH_SINC_FILTER_TYPE, SD_CHANNEL_MASK, TEMP_REG1.b0 
    ; Process enabled channel
    LBCO  &NC_SINC_FILTER_TYPE, PRUx_DMEM, TEMP_REG2, 1
    ; Found an enabled channel, so break out of loop
    JMP   END_CH_SINC_FILTER_TYPE
SKIP_CH_SINC_FILTER_TYPE:
    ; Move to next channel
    ADD   TEMP_REG2, TEMP_REG2, ICSSG_SDFM_CH_MEM_OFFSET
    ADD   TEMP_REG1, TEMP_REG1, 1
    ; Check if we've processed all channels
    QBLT  LOOP_START, TEMP_REG1, ICSS_PRU_MAX_NUM_OF_SD_CH
END_CH_SINC_FILTER_TYPE:

    ;min number of NC continuous sample for sinc filter in trigger mode 
    LDI   NC_SAMPLE_COUNT, NC_SAMP_CNT
    QBNE    SKIP_ACC3, NC_SINC_FILTER_TYPE, 0
    SUB   NC_SAMPLE_COUNT, NC_SAMPLE_COUNT, 1
    JMP    END_NC_SAMPLE_COUNT
SKIP_ACC3:
    QBNE    SKIP_ACC2, NC_SINC_FILTER_TYPE, 1
    SUB   NC_SAMPLE_COUNT, NC_SAMPLE_COUNT, 2
    JMP  END_NC_SAMPLE_COUNT
SKIP_ACC2:
    SUB   NC_SAMPLE_COUNT, NC_SAMPLE_COUNT, 3
END_NC_SAMPLE_COUNT:

    LBCO    &TEMP_REG0.b0, PRUx_DMEM, SDFM_EN_NC_USING_SNOOP_REG_OFFSET, 1
    QBBC    SKIP_ENABLE_TM, TEMP_REG0.b0, 0                  
    ;Initialize Task Manager
    JAL     RET_ADDR_REG, FN_TM_INIT

    ;Enable Task Manager
    M_PRU_TM_ENABLE

    JAL     RET_ADDR_REG, CONFG_IEP_CMP_FOR_TRIGGER_MODE
SKIP_ENABLE_TM:
    
    ;set cmparator regsiter
    LDI TEMP_REG2, SDFM_CFG_SD_CH0_EN_COMP_OFFSET
    
    LDI TEMP_REG3, 0
    LDI COMPARATOR_EN, 0
    LDI TEMP_REG1, SD_CH0
    LOOP   CONF_COMP_REG_LOOP, ICSS_PRU_MAX_NUM_OF_SD_CH
    QBBC  SKIP_CH_FOR_COMP, SD_CHANNEL_MASK, TEMP_REG1.b0 
    LBCO  &TEMP_REG3, PRUx_DMEM, TEMP_REG2, 1

    ; Calculate bit position based on channel number
    LSL TEMP_REG3, TEMP_REG3, TEMP_REG1

    ; OR with accumulated comparator enable bits
    OR    COMPARATOR_EN, COMPARATOR_EN, TEMP_REG3
SKIP_CH_FOR_COMP:
    ADD   TEMP_REG2, TEMP_REG2, ICSSG_SDFM_CH_MEM_OFFSET
    ADD   TEMP_REG1, TEMP_REG1, 1
CONF_COMP_REG_LOOP

    ;Skip normal mode if snoop mode is enabled
    LBCO    &TEMP_REG0, PRUx_DMEM, SDFM_EN_NC_USING_SNOOP_REG_OFFSET, 1
    QBBS    SKIP_NORMAL_MODE, TEMP_REG0, 0
    QBBC    SKIP_TRIGGER_MODE_NC, EN_NC_TRIGGER_MODE, 0
    JAL     RET_ADDR_REG, CONFG_IEP_CMP_FOR_TRIGGER_MODE
    JMP      TRIGGER_MODE_START
SKIP_TRIGGER_MODE_NC:
    JMP     CONTINUOUS_MODE_START
SKIP_NORMAL_MODE:
    
    LDI TEMP_REG1, ICSS_PRU_SD_FIRST_CH
    LOOP   CHECK_COMP_REG_LOOP, ICSS_PRU_MAX_NUM_OF_SD_CH
    QBBC   NEXT_CH_FOR_COMP, COMPARATOR_EN, TEMP_REG1.b0 
    JMP    TS0_OC_LOOP
NEXT_CH_FOR_COMP:
    ADD   TEMP_REG1, TEMP_REG1, 1
CHECK_COMP_REG_LOOP

    ;waiting loop if OC is disable
WAIT_LOOP:
    JMP   WAIT_LOOP

;------------------------------------Over Current---------------------------------------------------------;
;1) Select channel & read data from shadow register
;2) Does SINC3 differentiation
;3) Check sampled data value with High & Low threshold
;4) Toggle GPIO based on comparison
;---------------------------------------------------------------------------------------------------------;
TS0_OC_LOOP:

    QBBC    COMP_CH0_END, COMPARATOR_EN, SDFM_CFG_BF_SD_CH0_EN_COMP_BIT
    ;Switch to SD HW to Ch0 & Enable channels
    LDI     TEMP_REG1.b0, SD_CH0            ;Select channel 0
    LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2      ;R30[26-29] channel select bits
    SET     TEMP_REG1.b0.t1                    ;R30[25] global channel enable bit
    MOV     R30.b3, TEMP_REG1.b0
    NOP    
    ; R31[28], check shadow_update_flag for Ch0
    QBBC    COMP_CH0_END, R31, 28

    ;R31[24], ; clear shadow update flag for ch0
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     DN0, R31, MASK_REG
    
    ;Execute Sinc3/Sinc2/Sinc1 Differentiation
    LDI     TEMP_REG3, SDFM_CFG_CH0_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG3, 1
    QBNE    SKIP_OC_ACC3_CH0, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0
    JMP  END_OC_ACC_CH0
SKIP_OC_ACC3_CH0:
    QBNE    SKIP_OC_ACC2_CH0, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0
    JMP  END_OC_ACC_CH0
SKIP_OC_ACC2_CH0:
    M_ACC1_PROCESS ACC3_DN1_CH0
END_OC_ACC_CH0:
    
    ;Comparator for Ch0
    MOV     TEMP_REG2, CN5

    ;Trip Zone based over current detection
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_HIGH_THR_CH0_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_LOW_THR_CH0_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Check if the sample value is greater than the high threshold
    QBGE    OVER_CURRENT_HIGH_THRESHOLD_CH0, OC_HIGH_THR, TEMP_REG2
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OVER_CURRENT_LOW_THRESHOLD_CH0, OC_LOW_THR, TEMP_REG2
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH0
OVER_CURRENT_HIGH_THRESHOLD_CH0:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH0
OVER_CURRENT_LOW_THRESHOLD_CH0:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
END_OVER_CURRENT_DETECTION_CH0:

    ;zero cross for ch0
    QBBC    SKIP_ZERO_CROSS_CH0, ZERO_CROSS_EN, SDFM_CFG_BF_SD_CH0_ZC_EN_BIT
    ;Load the zero cross threshold value for current channel & store in OC_HIGH_THR register 
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH0_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  4
    ;Check if this is the first time after zero crossing is enabled
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH0_ZC_START_OFFSET
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    ;If this is the first time, simply store the current value in DMEM and skip the comparison
    QBBS    ZERO_CROSS_STARTED_CH0, TEMP_REG0.b0, 0
    SET     TEMP_REG0.t0
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH0_ZC_START_OFFSET
    SBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    QBA     SKIP_ZERO_CROSS_CH0
ZERO_CROSS_STARTED_CH0:
    ;Load the previous value for zero cross comparison
    LDI    TEMP_REG3, SDFM_CFG_ZC_CH0_PREV_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 4
    ;Store the current value in DMEM for the next zero cross comparison
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    ;Check if the previous sample value is greater than zero crossing threshold
    ;If that is the case, check if the current sample value is smaller than the zero cross threshold
    QBGE    CHECK_FOR_BELOW_THRESHOLD_CH0, OC_HIGH_THR, TEMP_REG0
    ;Check if the sample value is greater than the zero crossing threshold
    QBGE    OVER_ZC_THRESHOLD_CH0, OC_HIGH_THR, TEMP_REG2
    QBA     SKIP_ZERO_CROSS_CH0
CHECK_FOR_BELOW_THRESHOLD_CH0:
    ;Check if the sample value is lower than the zero crossing threshold
    QBLE    BELOW_ZC_THRESHOLD_CH0, OC_HIGH_THR, TEMP_REG2
    QBA SKIP_ZERO_CROSS_CH0
OVER_ZC_THRESHOLD_CH0:
    ;Set the ZC trip status as high
    LDI     TEMP_REG0, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as high
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH0_SET_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
    QBA SKIP_ZERO_CROSS_CH0
BELOW_ZC_THRESHOLD_CH0:
    ;Set the ZC trip status as low
    LDI     TEMP_REG0.b0, 0
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH0_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as low
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH0_CLR_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH0_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
SKIP_ZERO_CROSS_CH0:
      
COMP_CH0_END:
    
    ;CH1
    QBBC    COMP_CH1_END, COMPARATOR_EN, SDFM_CFG_BF_SD_CH1_EN_COMP_BIT
    ; Switch to SD HW to Ch1 & Enable channels
    LDI     TEMP_REG1.b0,  SD_CH1         ;Select channel 1
    LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2    ;R30[26-29] channel select bits
    SET     TEMP_REG1.b0.t1                  ;R30[25] global channel enable bit
    MOV     R30.b3, TEMP_REG1.b0
    NOP  

    ; R31[28], check  shadow_update_flag for ch1
    QBBC    COMP_CH1_END, R31, 28

    ; R31[24], shadow_update_flag_clr for ch1
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     DN0, R31, MASK_REG

    ;Execute Sinc3/Sinc2/Sinc1 Differentiation
    LDI    TEMP_REG3, SDFM_CFG_CH1_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG3, 1
    QBNE    SKIP_OC_ACC3_CH1, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1
    JMP  END_OC_ACC_CH1
SKIP_OC_ACC3_CH1:
    QBNE    SKIP_OC_ACC2_CH1, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1
    JMP  END_OC_ACC_CH1
SKIP_OC_ACC2_CH1:
    M_ACC1_PROCESS ACC3_DN1_CH1
END_OC_ACC_CH1:
    
    ;Comparator for Ch1
    MOV     TEMP_REG2, CN5  

    ;Trip Zone based over current detection
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_HIGH_THR_CH1_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_LOW_THR_CH1_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Check if the sample value is greater than the high threshold
    QBGE    OVER_CURRENT_HIGH_THRESHOLD_CH1, OC_HIGH_THR, TEMP_REG2
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OVER_CURRENT_LOW_THRESHOLD_CH1, OC_LOW_THR, TEMP_REG2
    LDI    TEMP_REG0.b0, 0
    LDI   TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH1
OVER_CURRENT_HIGH_THRESHOLD_CH1:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Store DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH1
OVER_CURRENT_LOW_THRESHOLD_CH1:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI   TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
END_OVER_CURRENT_DETECTION_CH1:  


    ;zero cross for ch1
    QBBC    SKIP_ZERO_CROSS_CH1, ZERO_CROSS_EN, SDFM_CFG_BF_SD_CH1_ZC_EN_BIT
    ;Load the zero cross threshold value for current channel & store in OC_HIGH_THR register 
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH1_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  4
    ;Check if this is the first time after zero crossing is enabled
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH1_ZC_START_OFFSET
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    ;If this is the first time, simply store the current value in DMEM and skip the comparison
    QBBS    ZERO_CROSS_STARTED_CH1, TEMP_REG0.b0, 0
    SET     TEMP_REG0.t0
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH1_ZC_START_OFFSET
    SBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    QBA     SKIP_ZERO_CROSS_CH1
ZERO_CROSS_STARTED_CH1:
    ;Load the previous value for zero cross comparison
    LDI     TEMP_REG3, SDFM_CFG_ZC_CH1_PREV_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 4
    ;Store the current value in DMEM for the next zero cross comparison
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    ;Check if the previous sample value is greater than zero crossing threshold
    ;If that is the case, check if the current sample value is smaller than the zero cross threshold
    QBGE    CHECK_FOR_BELOW_THRESHOLD_CH1, OC_HIGH_THR, TEMP_REG0
    ;Check if the sample value is greater than the zero crossing threshold
    QBGE    OVER_ZC_THRESHOLD_CH1, OC_HIGH_THR, TEMP_REG2
    QBA     SKIP_ZERO_CROSS_CH1
CHECK_FOR_BELOW_THRESHOLD_CH1:
    ;Check if the sample value is lower than the zero crossing threshold
    QBLE    BELOW_ZC_THRESHOLD_CH1, OC_HIGH_THR, TEMP_REG2
    QBA SKIP_ZERO_CROSS_CH1
OVER_ZC_THRESHOLD_CH1:
    ;Set the ZC trip status as high
    LDI     TEMP_REG0, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as high
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH1_SET_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
    QBA SKIP_ZERO_CROSS_CH1
BELOW_ZC_THRESHOLD_CH1:
    ;Set the ZC trip status as low
    LDI     TEMP_REG0.b0, 0
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH1_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as low
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH1_CLR_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH1_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
SKIP_ZERO_CROSS_CH1:

COMP_CH1_END:

    ;CH2
    QBBC    COMP_CH2_END, COMPARATOR_EN, SDFM_CFG_BF_SD_CH2_EN_COMP_BIT
    ; Switch to SD HW to Ch2 & Enable Channels
    LDI     TEMP_REG1.w0,  SD_CH2       ;Select channel 2
    LSL     TEMP_REG1.b0, TEMP_REG1.b0, 2  ;R30[26-29] channel select bits
    SET     TEMP_REG1.b0.t1                ;R30[25] global channel enable bit
    MOV     R30.b3, TEMP_REG1.b0
    NOP    
    ; R31[28], check shadow_update_flag for Ch2
    QBBC    TS0_OC_LOOP, R31, 28

    ; R31[24], shadow_update_flag_clr for Ch2
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     DN0, R31, MASK_REG
    
    ;Execute Sinc3/Sinc2/Sinc1 Differentiation
    LDI     TEMP_REG3, SDFM_CFG_CH2_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG3, 1
    QBNE    SKIP_OC_ACC3_CH2, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2
    JMP  END_OC_ACC_CH2
SKIP_OC_ACC3_CH2:
    QBNE    SKIP_OC_ACC2_CH2, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2
    JMP  END_OC_ACC_CH2
SKIP_OC_ACC2_CH2:
    M_ACC1_PROCESS ACC3_DN1_CH2
END_OC_ACC_CH2:

    ;Comparator for Ch2
    MOV     TEMP_REG2, CN5   

    ;Trip Zone based over current detection
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_HIGH_THR_CH2_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG3, SDFM_CFG_OC_LOW_THR_CH2_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG3,  SDFM_FOUR_BYTE
    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Check if the sample value is greater than the high threshold
    QBGE    OVER_CURRENT_HIGH_THRESHOLD_CH2, OC_HIGH_THR, TEMP_REG2
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OVER_CURRENT_LOW_THRESHOLD_CH2, OC_LOW_THR, TEMP_REG2
    LDI    TEMP_REG0.b0, 0
    LDI   TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH2
OVER_CURRENT_HIGH_THRESHOLD_CH2:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG3, SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    JMP     END_OVER_CURRENT_DETECTION_CH2
OVER_CURRENT_LOW_THRESHOLD_CH2:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG3, SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
END_OVER_CURRENT_DETECTION_CH2:

    ;zero cross for ch2
    QBBC    SKIP_ZERO_CROSS_CH2, ZERO_CROSS_EN, SDFM_CFG_BF_SD_CH2_ZC_EN_BIT
    ;Load the zero cross threshold value for current channel & store in OC_HIGH_THR register 
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH2_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG3,  4
    ;Check if this is the first time after zero crossing is enabled
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH2_ZC_START_OFFSET
    LBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    ;If this is the first time, simply store the current value in DMEM and skip the comparison
    QBBS    ZERO_CROSS_STARTED_CH2, TEMP_REG0.b0, 0
    SET     TEMP_REG0.t0
    LDI     TEMP_REG3, SDFM_CFG_BF_SD_CH2_ZC_START_OFFSET
    SBCO    &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG3, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    QBA     SKIP_ZERO_CROSS_CH2
ZERO_CROSS_STARTED_CH2:
    ;Load the previous value for zero cross comparison
    LDI     TEMP_REG3, SDFM_CFG_ZC_CH2_PREV_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 4
    ;Store the current value in DMEM for the next zero cross comparison
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG3, 4
    ;Check if the previous sample value is greater than zero crossing threshold
    ;If that is the case, check if the current sample value is smaller than the zero cross threshold
    QBGE    CHECK_FOR_BELOW_THRESHOLD_CH2, OC_HIGH_THR, TEMP_REG0
    ;Check if the sample value is greater than the zero crossing threshold
    QBGE    OVER_ZC_THRESHOLD_CH2, OC_HIGH_THR, TEMP_REG2
    QBA     SKIP_ZERO_CROSS_CH2
CHECK_FOR_BELOW_THRESHOLD_CH2:
    ;Check if the sample value is lower than the zero crossing threshold
    QBLE    BELOW_ZC_THRESHOLD_CH2, OC_HIGH_THR, TEMP_REG2
    QBA SKIP_ZERO_CROSS_CH2
OVER_ZC_THRESHOLD_CH2:
    ;Set the ZC trip status as high
    LDI     TEMP_REG0, 1
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as high
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH2_SET_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
    QBA SKIP_ZERO_CROSS_CH2
BELOW_ZC_THRESHOLD_CH2:
    ;Set the ZC trip status as low
    LDI     TEMP_REG0.b0, 0
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_STATUS_CH2_OFFSET
    SBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, 1
    ;Set the associated GPIO pin as low
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH2_CLR_VAL_ADDR_OFFSET
    LBCO    &GPIO_TGL_ADDR, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    LDI     TEMP_REG3, SDFM_CFG_ZC_THR_CH2_WRITE_VAL_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG3, SDFM_FOUR_BYTE
    SBBO    &TEMP_REG0, GPIO_TGL_ADDR, 0, SDFM_FOUR_BYTE
SKIP_ZERO_CROSS_CH2:

COMP_CH2_END:

    QBA     TS0_OC_LOOP


;--------------------------------Normal Current---------------------------------------------------;
;Normal current task
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
FN_NC_LOOP_TASK:
    ; Save/restore context
    ; restore differentiator state(R1-R18) for NC
    ; save current registers values for OC
    LDI   R0.b0, 0
    MOV     R18.b0, R30.b3                  ; save T0 SD channel select
    xchg    BANK_CTXT_NC, &R1, 4*18     
  
    
    ;select ch0, enable all channel, set SD snoop=1 & sample_counter_select=1
    LDI     R30.w2, (SD_CH0<<10 | 1<<9 | 1<<6 | 1<<5)
    NOP

    ; Snoop read Ch0 sample_counter,
    ; wait for ChX sample count+1.
    AND     TEMP_REG1, R31, 0xFF ; snoop read LSB Ch0 sample_counter
WAIT_SAMPLE_COUNT_INCR:
    AND     TEMP_REG2, R31, 0xFF ; snoop read LSB Ch0 sample_counter
    QBEQ    WAIT_SAMPLE_COUNT_INCR, TEMP_REG2, TEMP_REG1

    .if $isdefed("DEBUG_CODE")   
    ;Store sample counter values
    LDI    TEMP_REG1, SDFM_DUBUG_OFFSET
    LSL    TEMP_REG3, SAMP_CNT_REG, 1
    ADD     TEMP_REG1, TEMP_REG3, TEMP_REG1
    SBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG1, 1
    .endif

    ;Snoop read Ch0 ACC3/ACC2/ACC1
    CLR     R30.t21                 ; set SD sample_counter_select=0
    NOP
    AND     DN0, R31, MASK_REG      ; DN0 = Ch0 SD HW ACC3/ACC2/ACC1 output sample
    CLR     R30.t22                 ; set SD snoop=0    

    ;select ch1, enable all channel & set SD snoop=1
    LDI     R30.w2, (SD_CH1<<10 | 1<<9 | 1<<6)
    NOP
    ; Snoop read ACC3/ACC2/ACC1 for SD Ch1
    AND     TEMP_REG1, R31, MASK_REG    ; TEMP_REG1 = Ch1 SD HW ACC3/ACC2/ACC1 output sample
    CLR     R30.t22                     ; set SD snoop=0

    ;select ch2, enable all channel & set SD snoop=1
    LDI     R30.w2, (SD_CH2<<10 | 1<<9 | 1<<6)
    NOP
    ; Snoop read ACC3/ACC2/ACC1 for SD Ch2
    AND     TEMP_REG2, R31, MASK_REG    ; TEMP_REG0 = Ch2 SD HW ACC3/ACC2/ACC1 output sample
    CLR     R30.t22                     ; set SD snoop=0 

    ;Execute SINC3/SINC2/SINC1 differentiation for Ch0 ; 0h = acc3, 1h =acc2 & 2h = acc1
    QBNE    SKIP_ACC3_CH0, NC_SINC_FILTER_TYPE, 0
    M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0
    JMP  END_ACC_CH0
SKIP_ACC3_CH0:
    QBNE    SKIP_ACC2_CH0, NC_SINC_FILTER_TYPE, 1
    M_ACC2_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0
    JMP  END_ACC_CH0
SKIP_ACC2_CH0:
    M_ACC1_PROCESS ACC3_DN1_CH0
END_ACC_CH0:

    ;Save NC output sample to local output sample buffer
    SBCO    &CN5, PRUx_DMEM, OUT_SAMP_BUF_REG, 4   

    ;Execute SINC3/SINC2/SINC1 differentiation for Ch1 ; 0h = acc3, 1h =acc2 & 2h = acc1
    MOV     DN0, TEMP_REG1 ; DN0 = Ch1 SD HW ACC3/ACC2/ACC1 output sample

    QBNE    SKIP_ACC3_CH1, NC_SINC_FILTER_TYPE, 0
    M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1
    JMP  END_ACC_CH1
SKIP_ACC3_CH1:
    QBNE    SKIP_ACC2_CH1, NC_SINC_FILTER_TYPE, 1
    M_ACC2_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1
    JMP  END_ACC_CH1
SKIP_ACC2_CH1:
    M_ACC1_PROCESS ACC3_DN1_CH1
END_ACC_CH1:

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 4
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4

    ; Execute SINC3/SINC2/SINC1 differentiation for Ch2: 0h = acc3, 1h =acc2 & 2h = acc1
    MOV     DN0, TEMP_REG2 ; DN0 = Ch2 SD HW ACC3/ACC2/ACC1 output sample

    QBNE    SKIP_ACC3_CH2, NC_SINC_FILTER_TYPE, 0
    M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2
    JMP  END_ACC_CH2
SKIP_ACC3_CH2:
    QBNE    SKIP_ACC2_CH2, NC_SINC_FILTER_TYPE, 1
    M_ACC2_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2
    JMP  END_ACC_CH2
SKIP_ACC2_CH2:
    M_ACC1_PROCESS ACC3_DN1_CH2
END_ACC_CH2:

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 8
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
    
    ;Check NC sample count
    QBLE    RESET_NC_FRAME, SAMP_CNT_REG, NC_SAMPLE_COUNT
    ; NC sample count < NC_SAMP_CNT-1
    ; Add configured IEP count for NC OSR
    ; IEP0 CMP4_reg = cmp4_reg + NC_OSR*IEP_CLOCK* SD_cycle
    LBCO    &TEMP_REG0, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_CNT_OFFSET,  4
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_REG_OFFSET,  4
    LBCO    &TEMP_REG2, PRUx_DMEM,  SDFM_CFG_IEP_CFG_SIM_EPWM_PRD_OFFSET, 4
    ;load IEP register value
    LBBO    &TEMP_REG3, TEMP_REG1, 0, 4 
    ;Add with next sample time value
    ADD     TEMP_REG3, TEMP_REG3, TEMP_REG0
    QBLE    UPDATE_CMP_FOR_IEP_RESET,  TEMP_REG3, TEMP_REG2
    JMP     END_CMP_UPDATE
UPDATE_CMP_FOR_IEP_RESET:
    ;Update cmp event according to iep reset
    SUB      TEMP_REG3, TEMP_REG3, TEMP_REG2
END_CMP_UPDATE:
    ;update cmp event fro next NC sample
    SBBO    &TEMP_REG3, TEMP_REG1, 0, 4
        
    ADD     SAMP_CNT_REG, SAMP_CNT_REG, 1   ; increment NC sample count    
    QBA     NRESET_NC_FRAME
     
RESET_NC_FRAME:
    ; Set IEP CMP$ value: IEP_CMP_REG1:REG0 = 0:TRIG_SAMPLE_TIME
    QBBS    FIRST_NC_SAMPLE, SAMP_NAME, 0
    QBBC    FIRST_NC_SAMPLE, EN_DOUBLE_UPDATE, 0 ;check double update is enable
    LBCO    &TEMP_REG0, PRUx_DMEM, FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME, 4
    SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP
    
    ;update Cmp register for next sample time
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_REG_OFFSET,  4
    SBBO    &TEMP_REG0, TEMP_REG1, 0, 4

    LDI    SAMP_NAME, 1 ;clear sample name for first sample
    QBA     END_RESET_NC_FRAME
FIRST_NC_SAMPLE:
    LBCO    &TEMP_REG0, PRUx_DMEM, FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME, 4
    SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP4

    ;update Cmp register for next sample time
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_REG_OFFSET,  4
    SBBO    &TEMP_REG0, TEMP_REG1, 0, 4  

    LDI    SAMP_NAME, 0 ; update for Second sample
END_RESET_NC_FRAME:
    LDI     SAMP_CNT_REG, 0 ; reset NC sample count
    ; Write local interleaved output samples to Host buffer address
    LBCO    &TEMP_REG3, PRUx_DMEM, OUT_SAMP_BUF_REG, ICSS_PRU_MAX_NUM_OF_SD_CH*4
    LBCO    &TEMP_REG0, PRUx_DMEM, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET,4
    SBBO    &TEMP_REG3,  TEMP_REG0,  SDFM_CFG_OUT_SAMP_BUF_OFFSET, ICSS_PRU_MAX_NUM_OF_SD_CH*4
    ;Trigger interrupt for NC sampling
    LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ_CH0

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
FN_TM_INIT:
    ; TM general purpose mode
    ; Enable  T1_S1:
    LDI     TEMP_REG0.b0, (1b<<3|0b<<2|11b<<0)           ; enable  T1_s1
    .if $isdefed("SDFM_TXPRU_CORE")
    SBCO    &TEMP_REG0.b0, C28, 0, 1
    .else
    SBCO    &TEMP_REG0.b0, CT_PRU_ICSSG_TM, 0, 1
    .endif

    ;set T1_S1 address
    LDI     TEMP_REG0.w0, $CODE(FN_NC_LOOP_TASK)

    .if $isdefed("SDFM_TXPRU_CORE")
    SBCO    &TEMP_REG0.w0, C28, TASKS_MGR_TS1_PC_S1, 2
    .else
    SBCO    &TEMP_REG0.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_PC_S1, 2
    .endif

    ; Set Task triggers
    ; set T1_S1 trigger to IEP CMP event = 20, CMP7 = 23, CMP8 = 24
    ;load cmp event number
    LDI32 TEMP_REG2, PRU_ICSS_IEP1_BASE
    LBCO  &TEMP_REG0, PRUx_DMEM, SDFM_CFG_NC_PRD_IEP_REG_OFFSET, 4
    QBGT SKIP_IEP0, TEMP_REG2, TEMP_REG0
    LDI  TEMP_REG2, TASK_IEP0_STARTING_EVT
    JMP  SKIP_IEP1
SKIP_IEP0:
    LDI  TEMP_REG2, TASK_IEP1_STARTING_EVT
SKIP_IEP1:
    LDI TEMP_REG1.w0, 0
    LBCO  &TEMP_REG1.b0, PRUx_DMEM, SDFM_CFG_SD_CMP_EVENT_NUM_OFFSET, 1
    ADD   TEMP_REG1.b0, TEMP_REG1.b0, TEMP_REG2.b0
    LSL   TEMP_REG1.w0, TEMP_REG1.b0, CMP_EVENT_BIT_SHIFT

    .if $isdefed("SDFM_TXPRU_CORE")
    SBCO    &TEMP_REG1.w0, C28, TASKS_MGR_TS1_GEN_CFG1, 2
    .else
    SBCO    &TEMP_REG1.w0, CT_PRU_ICSSG_TM, TASKS_MGR_TS1_GEN_CFG1, 2
    .endif

    JMP     RET_ADDR_REG 

;;      
;; Initialize IEP CMP event .
;;
CONFG_IEP_CMP_FOR_TRIGGER_MODE:
    ; Load IEP CMP4 register address
    LDI     TEMP_REG1, FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 4
    LDI     TEMP_REG1, SDFM_CFG_NC_PRD_IEP_REG_OFFSET
    LBCO    &TEMP_REG2, PRUx_DMEM, TEMP_REG1, 4
    SBBO    &TEMP_REG0, TEMP_REG2, 0, 4
    LDI     TEMP_REG0, 0
    SBBO    &TEMP_REG0, TEMP_REG2, 4, 4
    JMP     RET_ADDR_REG
;
;;
;; Reset Scratchpad for NC registers
;;
;; Arguments: None
;;
;
FN_RESET_SDFM_STATE:
    LDI     R0.b0, 0
    ZERO    &R1, 4*18
    XOUT    BANK_CTXT_NC, &R1, 4*18 ;clear ScratchPad registers for NC
    JMP     RET_ADDR_REG

;
; Reset SD channel hardware
;
;   SDFM_CFG_BASE_PTR_REG: base address of SD Configuration registers 
;
FN_RESET_SD_CH_HW:
    LDI    TEMP_REG0, 0
    LDI    TEMP_REG1.b0, ICSS_PRU_SD_FIRST_CH
    LOOP   RESET_SD_LOAD_SHARE_CH_HW_LOOP, ICSS_PRU_MAX_NUM_OF_SD_CH ; loop over SD channels
    ;Set R30[29-26]:channel_select = channel ID.
    LSL     TEMP_REG0.b0, TEMP_REG1.b0, 2
    SET     TEMP_REG0.b0.t1                                 ; R30[25] channel_enable=1
    MOV     R30.b3, TEMP_REG0.b0
    ADD     TEMP_REG1, TEMP_REG1, 1                     ; increment to next channel 
    SET     R31.t23                                     ; R31[23] re_init=1
RESET_SD_LOAD_SHARE_CH_HW_LOOP
    JMP     RET_ADDR_REG
  

    .if $isdefed("SDFM_PHASE_DELAY_CALC")
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
    SBCO  &TEMP_REG1, PRUx_DMEM, SDFM_CFG_SD_CH0_CLOCK_PHASE_DELAY, 4
    JMP     RET_ADDR_REG
    .endif ; SDFM_PHASE_DELAY_CALC

TRIGGER_MODE_START:  
    ; read IEP CMP status register address from dmem
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_CMP_STATUS_REG_OFFSET,  4
    LBCO    &TEMP_REG2.b0, PRUx_DMEM, SDFM_CFG_SD_CMP_EVENT_NUM_OFFSET, 1
    ; Get pending events from IEP
    LBBO	&TEMP_REG0,	TEMP_REG1,	0,	2
    ; wait till IEP CMP event get hit
	QBBC	TRIGGER_MODE_START,	TEMP_REG0,	TEMP_REG2.b0
    ; Clear IEP CMP event
    LDI    TEMP_REG3, 1
    LSL    TEMP_REG3, TEMP_REG3, TEMP_REG2.b0
    SBBO	&TEMP_REG3,	TEMP_REG1,	0,	2
    ; Clear the differentiation state registers (R9-R17) for SD channels
    ; These registers store the previous sample values for SINC3/SINC2/SINC1 differentiation
    ZERO &R9, 4*9
    ; reset SD channel HW
    JAL     RET_ADDR_REG, FN_RESET_SD_CH_HW
    

    ;Sample count initialization
    LDI     SAMP_CNT_REG, 1
    LDI     NC_SAMPLE_DONE, 0
   
CONTINUOUS_MODE_START:
    ; Read accumulator output
    QBBC CH0_SKIP, SD_CHANNEL_MASK, SD_CH0
    ;select ch0, enable all channel
    LDI     R30.w2, (SD_CH0<<10 | 1<<9 )
    NOP
CH0_WAIT:
    ;R31[28], check shadow_update_flag for Ch0
    QBBC    CH0_SKIP, R31, 28

    ;R31[24], ; clear shadow update flag for ch0
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     DN0, R31, MASK_REG

    ;Performs differentiation of accumulator's output
    
    ;Execute SINC3/SINC2/SINC1 differentiation for Ch0 ; 0h = acc3, 1h =acc2 & 2h = acc1
    LDI     TEMP_REG1, SDFM_CFG_CH0_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG1, 1
    QBNE    SKIP1_ACC3_CH0, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0, ACC3_DN5_CH0
    JMP  END1_ACC_CH0
SKIP1_ACC3_CH0:
    QBNE    SKIP1_ACC2_CH0, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH0, ACC3_DN3_CH0
    JMP  END1_ACC_CH0
SKIP1_ACC2_CH0:
    M_ACC1_PROCESS ACC3_DN1_CH0
END1_ACC_CH0:

     ;CH0 sampling done
     OR NC_SAMPLE_DONE, NC_SAMPLE_DONE, 1<<SD_CH0
    ;Check if the continuous mode is enabled 
    QBBC    SKIP_TRIGGER_MODE_CH0, EN_NC_TRIGGER_MODE, 0

    ;Save NC output sample to local output sample buffer
    MOV     TEMP_REG1, OUT_SAMP_BUF_REG 
    SBCO    &CN5, PRUx_DMEM, TEMP_REG1, 4

    JMP     CH0_SKIP
SKIP_TRIGGER_MODE_CH0:
    ;Over current detection 
    QBBC    END_OC_DETECTION_CH0, COMPARATOR_EN, SDFM_CFG_BF_SD_CH0_EN_COMP_BIT
    ;Comparator for Ch0
    MOV     TEMP_REG3, CN5
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG1, SDFM_CFG_OC_HIGH_THR_CH0_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG1, SDFM_CFG_OC_LOW_THR_CH0_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE

    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Check if the sample value is greater than the high threshold
    QBGE    OC_HIGH_THRESHOLD_CH0, OC_HIGH_THR, TEMP_REG3
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG1, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OC_LOW_THRESHOLD_CH0, OC_LOW_THR, TEMP_REG3
    ;Unset in DMEM
    LDI     TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET 
    SBCO   &TEMP_REG0.b0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH0
OC_HIGH_THRESHOLD_CH0:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4 
    ;Store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH0
OC_LOW_THRESHOLD_CH0:
      ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4 
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH0_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
END_OC_DETECTION_CH0:
 
    ;Save NC output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 0
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
    LDI     TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 4
    LDI    TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_OFFSET
    SBBO    &CN5,  TEMP_REG0,  TEMP_REG1, 4
    ;Trigger interrupt
    LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ_CH0

CH0_SKIP:
    

    ;Ch1 sampling
    QBBC CH1_SKIP, SD_CHANNEL_MASK, SD_CH1
    
    ;select ch1, enable all channel
    LDI     R30.w2, (SD_CH1<<10 | 1<<9 )
    NOP
CH1_WAIT:
    ;R31[28], check shadow_update_flag for Ch1
    ;;QBBC    CH1_WAIT, R31, 28
    QBBC CH1_SKIP, R31, 28
     

    ;R31[24], ; clear shadow update flag for ch1
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     TEMP_REG1, R31, MASK_REG

    ;Execute SINC3/SINC2/SINC1 differentiation for Ch1 ; 0h = acc3, 1h =acc2 & 2h = acc1
    MOV     DN0, TEMP_REG1 ; DN0 = Ch2 SD HW ACC3/ACC2/ACC1 output sample

    LDI     TEMP_REG1, SDFM_CFG_CH1_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG1, 1
    QBNE    SKIP1_ACC3_CH1, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1, ACC3_DN5_CH1
    JMP  END1_ACC_CH1
SKIP1_ACC3_CH1:
    QBNE    SKIP1_ACC2_CH1, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH1, ACC3_DN3_CH1
    JMP  END1_ACC_CH1
SKIP1_ACC2_CH1:
    M_ACC1_PROCESS ACC3_DN1_CH1
END1_ACC_CH1:

    OR NC_SAMPLE_DONE, NC_SAMPLE_DONE, 1<<SD_CH1
    ;Check if the continuous mode is enabled 
    QBBC    SKIP_TRIGGER_MODE_CH1, EN_NC_TRIGGER_MODE, 0

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 4
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
   
    JMP     CH1_SKIP
SKIP_TRIGGER_MODE_CH1:

    ;Over Current 
    QBBC    END_OC_DETECTION_CH1, COMPARATOR_EN, SDFM_CFG_BF_SD_CH1_EN_COMP_BIT
    ;Comparator for Ch1
    MOV     TEMP_REG3, CN5  
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG1, SDFM_CFG_OC_HIGH_THR_CH1_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG1, SDFM_CFG_OC_LOW_THR_CH1_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE

    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
   
    ;Check if the sample value is greater than the high threshold
    QBGE    OC_HIGH_THRESHOLD_CH1, OC_HIGH_THR, TEMP_REG3
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OC_LOW_THRESHOLD_CH1, OC_LOW_THR, TEMP_REG3
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH1
OC_HIGH_THRESHOLD_CH1:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Store DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH1
OC_LOW_THRESHOLD_CH1:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH1_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
END_OC_DETECTION_CH1:  

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 4
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
    ;SBCO    &CN5, OUT_SAMP_BUF_REG, 4, 4

    ; Write local interleaved output samples to Host buffer address
    LDI    TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 4
    LDI    TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_OFFSET
    ADD    TEMP_REG1, TEMP_REG1, 4
    SBBO    &CN5,  TEMP_REG0,  TEMP_REG1, 4
    ;Trigger interrupt
    LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ_CH1

CH1_SKIP:

    QBBC CH2_SKIP, SD_CHANNEL_MASK, SD_CH2
    ;select ch1, enable all channel
    LDI     R30.w2, (SD_CH2<<10 | 1<<9 )
    NOP
CH2_WAIT:
    ;R31[28], check shadow_update_flag for Ch1
    QBBC    CH2_SKIP, R31, 28
    ;R31[24], ; clear shadow update flag for ch1
    SET     R31, R31.t24
    ; Load reg R31[0-27] SD HW ACC3/ACC2/ACC1 output sample
    AND     TEMP_REG2, R31, MASK_REG

    ; Execute SINC3/SINC2/SINC1 differentiation for Ch2: 0h = acc3, 1h =acc2 & 2h = acc1
    MOV     DN0, TEMP_REG2 ; DN0 = Ch2 SD HW ACC3/ACC2/ACC1 output sample

    LDI     TEMP_REG1, SDFM_CFG_CH2_FILTER_TYPE_OFFSET
    LBCO    &TEMP_REG0.b2, PRUx_DMEM,  TEMP_REG1, 1
    QBNE    SKIP1_ACC3_CH2, TEMP_REG0.b2, 0
    M_ACC3_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2, ACC3_DN5_CH2
    JMP  END1_ACC_CH2
SKIP1_ACC3_CH2:
    QBNE    SKIP1_ACC2_CH2, TEMP_REG0.b2, 1
    M_ACC2_PROCESS ACC3_DN1_CH2, ACC3_DN3_CH2
    JMP  END1_ACC_CH2
SKIP1_ACC2_CH2:
    M_ACC1_PROCESS ACC3_DN1_CH2
END1_ACC_CH2:
    
    LDI TEMP_REG0, 1
    LSL TEMP_REG0, TEMP_REG0, SD_CH2
    OR NC_SAMPLE_DONE, NC_SAMPLE_DONE, TEMP_REG0
    ;Check if the continuous mode is enabled 
    QBBC    SKIP_TRIGGER_MODE_CH2, EN_NC_TRIGGER_MODE, 0

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 8
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
    
    JMP CH2_SKIP
SKIP_TRIGGER_MODE_CH2:
    ;Over current 
    QBBC    END_OC_DETECTION_CH2, COMPARATOR_EN, SDFM_CFG_BF_SD_CH2_EN_COMP_BIT
    ;Comparator for Ch2
    MOV     TEMP_REG3, CN5   
    
    ;Load the positive threshold value for current channel
    LDI     TEMP_REG1, SDFM_CFG_OC_HIGH_THR_CH2_OFFSET
    LBCO    &OC_HIGH_THR, PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE
    ;Load the positive threshold value for current channel
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_CH2_OFFSET
    LBCO    &OC_LOW_THR,  PRUx_DMEM,  TEMP_REG1,  SDFM_FOUR_BYTE
    ;PWM0 register offset 
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    LBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Check if the sample value is greater than the high threshold
    QBGE    OC_HIGH_THRESHOLD_CH2, OC_HIGH_THR, TEMP_REG3
    ;Unset in DMEM
    LDI    TEMP_REG0.b0, 0
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    ;Check if the sample value is lower than the low threshold
    QBLE    OC_LOW_THRESHOLD_CH2, OC_LOW_THR, TEMP_REG3
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH2
OC_HIGH_THRESHOLD_CH2:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;Store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_HIGH_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
    JMP     END_OC_DETECTION_CH2
OC_LOW_THRESHOLD_CH2:
    ;Generate PWM trip
    SET   TEMP_REG0.b2.t3
    LDI    TEMP_REG1, ICSSG_CFG_PWMx
    SBCO   &TEMP_REG0, CT_PRU_ICSSG_CFG, TEMP_REG1, 4
    ;set bit store in DMEM
    LDI    TEMP_REG0.b0, 1
    LDI    TEMP_REG1, SDFM_CFG_OC_LOW_THR_STATUS_CH2_OFFSET
    SBCO   &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 1
END_OC_DETECTION_CH2:

    ; Save output sample to local output sample buffer
    ADD  TEMP_REG0, OUT_SAMP_BUF_REG, 8
    SBCO    &CN5, PRUx_DMEM, TEMP_REG0, 4
    
    ; Write local interleaved output samples to Host buffer address
    LDI     TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET
    LBCO    &TEMP_REG0, PRUx_DMEM, TEMP_REG1, 4
    LDI    TEMP_REG1, SDFM_CFG_OUT_SAMP_BUF_OFFSET
    ADD    TEMP_REG1, TEMP_REG1, 8
    SBBO    &CN5,  TEMP_REG0,  TEMP_REG1, 4
    ;Trigger interrupt
    LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ_CH2
    QBA CONTINUOUS_MODE_START

CH2_SKIP: 

    ;Intruppt for trigger mode 
    QBBC    SKIP_TRIGGER_MODE, EN_NC_TRIGGER_MODE, 0
    AND     TEMP_REG0.w0, NC_SAMPLE_DONE, SD_CHANNEL_MASK
    QBNE    CONTINUOUS_MODE_START,  TEMP_REG0.w0, SD_CHANNEL_MASK
    LDI     NC_SAMPLE_DONE, 0
    QBLE    NC_SAMPLING_DONE, SAMP_CNT_REG, NC_SAMPLE_COUNT 
    ; Increment NC sample count
    ADD     SAMP_CNT_REG, SAMP_CNT_REG, 1   ; increment NC sample count    
    QBA     SKIP_TRIGGER_MODE
NC_SAMPLING_DONE:

    QBBS    NEXT_NC_SAMPLE, SAMP_NAME, 0
    QBBC    NEXT_NC_SAMPLE, EN_DOUBLE_UPDATE, 0 ;check double update is enable
    LBCO    &TEMP_REG0, PRUx_DMEM, FW_REG_SDFM_CFG_SECOND_TRIG_SAMPLE_TIME, 4
    SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP4
    
    ;update Cmp register for next sample time
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_REG_OFFSET,  4
    SBBO    &TEMP_REG0, TEMP_REG1, 0, 4

    LDI    SAMP_NAME, 1 ;clear sample name for first sample
    QBA     TRIGGER_HOST_INT
NEXT_NC_SAMPLE:
    LBCO    &TEMP_REG0, PRUx_DMEM, FW_REG_SDFM_CFG_FIRST_TRIG_SAMPLE_TIME, 4
    SUB     TEMP_REG0, TEMP_REG0, IEP_DEFAULT_INC ; subtract IEP default increment since IEP counts 0...CMP4
    ;update Cmp register for next sample time
    LBCO    &TEMP_REG1, PRUx_DMEM,  SDFM_CFG_NC_PRD_IEP_REG_OFFSET,  4
    SBBO    &TEMP_REG0, TEMP_REG1, 0, 4  
    LDI    SAMP_NAME, 0 ; update for Second sample
TRIGGER_HOST_INT:
    LDI     SAMP_CNT_REG, 1 ; reset NC sample count
    ; Write local interleaved output samples to Host buffer address
    MOV     TEMP_REG1,  OUT_SAMP_BUF_REG
    LDI     TEMP_REG2, SD_CH0
    LBCO    &TEMP_REG0, PRUx_DMEM, SDFM_CFG_OUT_SAMP_BUF_BASE_ADD_OFFSET, 4
    LOOP    LOAD_SAMPLE_IN_MEMORY,  ICSS_PRU_MAX_NUM_OF_SD_CH
    QBBC    SKIP_CH_FOR_MEMORY, SD_CHANNEL_MASK, TEMP_REG2
    LBCO    &TEMP_REG3, PRUx_DMEM, TEMP_REG1, 4
    SBBO    &TEMP_REG3,  TEMP_REG0,  0,  4
SKIP_CH_FOR_MEMORY:
    ADD     TEMP_REG1, TEMP_REG1, 4
    ADD     TEMP_REG2, TEMP_REG2, 1
    ADD     TEMP_REG0, TEMP_REG0, 4
LOAD_SAMPLE_IN_MEMORY
    ;Trigger interrupt for NC sampling
    LDI     R31.w0, TRIGGER_HOST_SDFM_IRQ_CH0

    QBA     TRIGGER_MODE_START
SKIP_TRIGGER_MODE:
    QBA     CONTINUOUS_MODE_START

   
