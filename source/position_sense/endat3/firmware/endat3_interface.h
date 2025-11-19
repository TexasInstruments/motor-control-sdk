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
; endat3_Interface Structure Memory Map Offsets
;******************************************************************************
; Communication Buffers
ENDAT3_INTERFACE_LUT_OFFSET                     .set    0x0000
ENDAT3_INTERFACE_RX_BUFFER_OFFSET               .set    0x0140
DECODED_DATA_OFFSET                             .set    0x0140

; High Priority Frame (HPF)
ENDAT3_INTERFACE_HPF_OFFSET                     .set    0x0180
ENDAT3_INTERFACE_HPF_DATA_OFFSET                .set    0x0180
ENDAT3_INTERFACE_HPF_STATUS_OFFSET              .set    0x0186
ENDAT3_INTERFACE_HPF_CRC_OFFSET                 .set    0x0187

; Low Priority Header (LPH)
ENDAT3_INTERFACE_LPH_OFFSET                     .set    0x0188
ENDAT3_INTERFACE_LPH_STATUS_OFFSET              .set    0x0188
ENDAT3_INTERFACE_LPH_NUM_LPF_OFFSET             .set    0x0189
ENDAT3_INTERFACE_LPH_RESERVED_OFFSET            .set    0x018A
ENDAT3_INTERFACE_LPH_CRC_OFFSET                 .set    0x018B

; Low Priority Frames (LPF)
ENDAT3_INTERFACE_LPF_OFFSET                     .set    0x018C
ENDAT3_INTERFACE_LPF_SIZE                       .set    8
ENDAT3_INTERFACE_LPF0_OFFSET                    .set    0x018C
ENDAT3_INTERFACE_LPF1_OFFSET                    .set    0x0194
ENDAT3_INTERFACE_LPF2_OFFSET                    .set    0x019C
ENDAT3_INTERFACE_LPF3_OFFSET                    .set    0x01A4
ENDAT3_INTERFACE_LPF4_OFFSET                    .set    0x01AC
ENDAT3_INTERFACE_LPF5_OFFSET                    .set    0x01B4
ENDAT3_INTERFACE_LPF6_OFFSET                    .set    0x01BC
ENDAT3_INTERFACE_LPF7_OFFSET                    .set    0x01C4

; Status Flags
ENDAT3_INTERFACE_CONNECTED_OFFSET               .set    0x01CC
ENDAT3_INTERFACE_BUSY_OFFSET                    .set    0x01CD
HOST_TRIGGER_STATUS_FLAG                        .set    0x01CD
ENDAT3_INTERFACE_COMM_CYCLE_FLAG_OFFSET         .set    0x01CE
ENDAT3_INTERFACE_RESERVED1_OFFSET               .set    0x01CF

; Frame Control
ENDAT3_INTERFACE_EXPECTED_TX_FRAMES_OFFSET      .set    0x01D0
TX_FRAMES_EXPECTED_OFFSET                       .set    0x01D0
ENDAT3_INTERFACE_CURRENT_TX_FRAME_OFFSET        .set    0x01D4

; Data Buffers
ENDAT3_INTERFACE_TX_BUFFER_OFFSET               .set    0x01D8
DMEM_BASE_TX_BUFFER_OFFSET                      .set    0x01D8
ENDAT3_INTERFACE_BG_DATA_OFFSET                 .set    0x01F0

; Timing and Operation Codes
ENDAT3_INTERFACE_PROPAGATION_TIME_OFFSET        .set    0x0208
ENDAT3_INTERFACE_FOREGROUND_OP_CODE_OFFSET      .set    0x020C
TX_FRAME_ID_OFFSET                              .set    0x020C
ENDAT3_INTERFACE_BACKGROUND_OP_CODE_OFFSET      .set    0x0210
BG_OPCODE_OFFSET                                .set    0x0210

; Periodic Trigger Configuration (NEW)
ENDAT3_OPMODE_CONFIG_OFFSET                     .set    0x0214
ENDAT3_OPMODE_RESERVED2_OFFSET                  .set    0x0215
ENDAT3_OPMODE_RESERVED3_OFFSET                  .set    0x0216
ENDAT3_OPMODE_RESERVED4_OFFSET                  .set    0x0217
ENDAT3_OPMODE_PERIODIC                          .set    0       ; Periodic trigger mode
ENDAT3_OPMODE_HOST                              .set    1       ; Host trigger mode

; Delay Cycle Configuration (frequency-independent timing)
; These values are calculated by R5F based on actual PRU frequency
ENDAT3_DELAY_TX_START_1_OFFSET                  .set    0x0218
ENDAT3_DELAY_TX_START_2_OFFSET                  .set    0x021C
ENDAT3_DELAY_TX_START_3_OFFSET                  .set    0x0220
ENDAT3_DELAY_SAMPLING_OFFSET                    .set    0x0224
ENDAT3_DELAY_10MS_OFFSET                        .set    0x0228

; Trigger Control (unified for host and periodic modes)
ENDAT3_INTERFACE_START_TRIGGER_OFFSET            .set    0x022C
START_TRIGGER_OFFSET                            .set    0x022C

; Total Size
ENDAT3_INTERFACE_TOTAL_SIZE                     .set    0x022D

