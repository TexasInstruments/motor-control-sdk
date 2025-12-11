;
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
;*   File:     endat3_interface.h                                            		*
;*                                                                                  *
;*   Brief:    Firmware constants, and structure memory map offsets     *
;************************************************************************************

;******************************************************************************
; endat3_interface Structure Memory Map Offsets
;******************************************************************************
; Manchester Decode Lookup Table (256 bytes in DMEM)

; Used for efficient Manchester decoding in firmware
ENDAT3_INTERFACE_LUT_OFFSET                     .set    0x0000

; Communication Buffers
; RX buffer starts at offset 0x0100 (256 bytes after LUT start)
ENDAT3_INTERFACE_RX_BUFFER_OFFSET               .set    0x0100

; High Priority Frame (HPF)
ENDAT3_INTERFACE_HPF_OFFSET                     .set    0x0140
ENDAT3_INTERFACE_HPF_DATA_OFFSET                .set    0x0140
ENDAT3_INTERFACE_HPF_STATUS_OFFSET              .set    0x0146
ENDAT3_INTERFACE_HPF_CRC_OFFSET                 .set    0x0147

; Low Priority Header (LPH)
ENDAT3_INTERFACE_LPH_OFFSET                     .set    0x0148
ENDAT3_INTERFACE_LPH_STATUS_OFFSET              .set    0x0148
ENDAT3_INTERFACE_LPH_NUM_LPF_OFFSET             .set    0x0149
ENDAT3_INTERFACE_LPH_RESERVED_OFFSET            .set    0x014A
ENDAT3_INTERFACE_LPH_CRC_OFFSET                 .set    0x014B

; Low Priority Frames (LPF)
ENDAT3_INTERFACE_LPF_OFFSET                     .set    0x014C
ENDAT3_INTERFACE_LPF_SIZE                       .set    8
ENDAT3_INTERFACE_LPF0_OFFSET                    .set    0x014C
ENDAT3_INTERFACE_LPF1_OFFSET                    .set    0x0154
ENDAT3_INTERFACE_LPF2_OFFSET                    .set    0x015C
ENDAT3_INTERFACE_LPF3_OFFSET                    .set    0x0164
ENDAT3_INTERFACE_LPF4_OFFSET                    .set    0x016C
ENDAT3_INTERFACE_LPF5_OFFSET                    .set    0x0174
ENDAT3_INTERFACE_LPF6_OFFSET                    .set    0x017C
ENDAT3_INTERFACE_LPF7_OFFSET                    .set    0x0184

; Status Flags
ENDAT3_INTERFACE_CONNECTED_OFFSET               .set    0x018C
ENDAT3_INTERFACE_BUSY_OFFSET                    .set    0x018D
ENDAT3_INTERFACE_COMM_CYCLE_FLAG_OFFSET         .set    0x018E
ENDAT3_INTERFACE_RESERVED1_OFFSET               .set    0x018F

; Frame Control
ENDAT3_INTERFACE_EXPECTED_TX_FRAMES_OFFSET      .set    0x0190
ENDAT3_INTERFACE_CURRENT_TX_FRAME_OFFSET        .set    0x0194

; Data Buffers
ENDAT3_INTERFACE_TX_BUFFER_OFFSET               .set    0x0198
ENDAT3_INTERFACE_BG_DATA_OFFSET                 .set    0x01B0

; Timing and Operation Codes
ENDAT3_INTERFACE_PROPAGATION_TIME_OFFSET        .set    0x01C8
ENDAT3_INTERFACE_FOREGROUND_OP_CODE_OFFSET      .set    0x01CC
ENDAT3_INTERFACE_BACKGROUND_OP_CODE_OFFSET      .set    0x01D0

; Periodic Trigger Configuration (NEW)
ENDAT3_OPMODE_CONFIG_OFFSET                     .set    0x01D4
ENDAT3_OPMODE_RESERVED2_OFFSET                  .set    0x01D5
ENDAT3_OPMODE_RESERVED3_OFFSET                  .set    0x01D6
ENDAT3_OPMODE_RESERVED4_OFFSET                  .set    0x01D7
ENDAT3_OPMODE_PERIODIC                          .set    0       ; Periodic trigger mode
ENDAT3_OPMODE_HOST                              .set    1       ; Host trigger mode

; Delay Cycle Configuration (frequency-independent timing)
; These values are calculated by R5F based on actual PRU frequency
ENDAT3_DELAY_TX_START_1_OFFSET                  .set    0x01D8
ENDAT3_DELAY_TX_START_2_OFFSET                  .set    0x01DC
ENDAT3_DELAY_TX_START_3_OFFSET                  .set    0x01E0
ENDAT3_DELAY_SAMPLING_OFFSET                    .set    0x01E4
ENDAT3_DELAY_10MS_OFFSET                        .set    0x01E8

; Trigger Control (unified for host and periodic modes)
ENDAT3_INTERFACE_START_TRIGGER_OFFSET            .set    0x01EC

; Channel Enable Mask
; Bit 0: Channel 0 enable
; Bit 1: Channel 1 enable
; Bit 2: Channel 2 enable
ENDAT3_CHANNEL_ENABLE_MASK_OFFSET                .set    0x01ED

; Total Size (reduced from 0x022E to 0x01EE, saving 64 bytes)
ENDAT3_INTERFACE_TOTAL_SIZE                     .set    0x01EE

