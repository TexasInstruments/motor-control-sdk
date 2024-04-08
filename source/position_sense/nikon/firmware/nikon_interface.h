;
; Copyright (C) 2024 Texas Instruments Incorporated
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
;*   File:     nikon_interface.h                                                    *
;*                                                                                  *
;*   Brief:    Defining Data Memory Offsets                                         *
;************************************************************************************

NIKON_HOST_TRIGGER_STATUS_OFFSET            .set    0x00            ;status of ch0 current cycle
NIKON_HOST_TRIGGER_STATUS_RTU_OFFSET        .set    0x00            ;status of ch0/RTU current cycle
NIKON_HOST_TRIGGER_STATUS_PRU_OFFSET        .set    0x01            ;status of ch1/PRU current cycle
NIKON_HOST_TRIGGER_STATUS_TXPRU_OFFSET      .set    0x02            ;status of ch2/TXPRU current cycle
NIKON_CHANNEL_CONFIG_OFFSET                 .set    0x03            ;channel config

NIKON_NUMBER_OF_ENCODERS_OFFSET             .set    0x04            ;number of encoders connected
NIKON_NUMBER_OF_ENCODERS_RTU_OFFSET         .set    0x04            ;number of encoders connected to RTU
NIKON_NUMBER_OF_ENCODERS_PRU_OFFSET         .set    0x05            ;number of encoders connected to PRU
NIKON_NUMBER_OF_ENCODERS_TXPRU_OFFSET       .set    0x06            ;number of encoders connected to TXPRU
NIKON_CRC_POS_DATA_CRC_LEN_OFFSET           .set    0x07            ;rx crc length

NIKON_RX_FRAME_SIZE_CONFIG                  .set    0x08            ;config the overall rx frame size
NIKON_RX_FRAME_SIZE_RTU_CONFIG              .set    0x08            ;config the overall rx frame size for RTU
NIKON_RX_FRAME_SIZE_PRU_CONFIG              .set    0x0A            ;config the overall rx frame size for PRU
NIKON_RX_FRAME_SIZE_TXPRU_CONFIG            .set    0x0C            ;config the overall rx frame size for TXPRU
NIKON_VALID_BIT_IDX_OFFSET                  .set    0x0E            ;offset for valid bit index
NIKON_FIFO_BIT_IDX_OFFSET                   .set    0x0F            ;offset for fifo bot index

NIKON_RX_CLK_FREQ_CONFIG_OFFSET             .set    0x10            ;rx clock frequency
NIKON_MEM_ACCESS_STATUS_OFFSET              .set    0x11            ;eeprom access status
NIKON_NUMBER_OF_RX_FRAMES_OFFSET            .set    0x12            ;number of Rx frames to be receive from encoder
NIKON_CONFIG_MT_DELAY_OFFSET                .set    0x13            ;delay between two consecutive encoders connected in bus connection

NIKON_LS_RTU_SYNC_STATUS_OFFSET             .set    0x14            ;synchronization status of RTU
NIKON_LS_PRU_SYNC_STATUS_OFFSET             .set    0x15            ;synchronization status of PRU
NIKON_LS_TXPRU_SYNC_STATUS_OFFSET           .set    0x16            ;synchronization status of TXPRU
NIKON_PRIMARY_CORE_MASK_OFFSET              .set    0x17            ;mask for primary core in load share mode.

NIKON_OPMODE_CONFIG_OFFSET                  .set    0x18            ;operration mode trigger offset
NIKON_OPMODE_RTU_CONFIG_OFFSET              .set    0x18            ;operation mode of RTU offset
NIKON_OPMODE_PRU_CONFIG_OFFSET              .set    0x19            ;operation mode of PRU offset
NIKON_OPMODE_TXPRU_CONFIG_OFFSET            .set    0x1A            ;operation mode of TxPRU offset

NIKON_COMMAND_DATA_FRAME_OFFSET             .set    0x1C            ;command data frame to be send over Tx
NIKON_COMMAND_DATA_FRAME_RTU_OFFSET         .set    0x1C            ;command data frame to be send over Tx by RTU
NIKON_COMMAND_DATA_FRAME_PRU_OFFSET         .set    0x20            ;command data frame to be send over Tx by PRU
NIKON_COMMAND_DATA_FRAME_TXPRU_OFFSET       .set    0x24            ;command data frame to be send over Tx by TXPRU



NIKON_MEMORY_DATA0_FRAME_OFFSET             .set    0x28            ;MDF0 for upper byte of data to be written in EEPROM or as ID code

NIKON_MEMORY_DATA1_FRAME_OFFSET             .set    0x2C            ;MDF1 for lower byte of data to be written in EEPROM or as  middle byte of ID code

NIKON_MEMORY_ADDR_FRAME_OFFSET              .set    0x30            ;MDF2 for address of EEPROM location or lower byte of ID code

NIKON_POSITION_DATA_ENC0_RES_OFFSET         .set    0x34            ;Base Offset for encoder 0
NIKON_INFO_FIELD_OFFSET                     .set    0x00            ;information field offset
NIKON_INFO_FIELD_CH0_OFFSET                 .set    0x00            ;information field ch0 offset
NIKON_INFO_FIELD_CH1_OFFSET                 .set    0x02            ;information field ch1 offset
NIKON_INFO_FIELD_CH2_OFFSET                 .set    0x04            ;information field ch2 offset

NIKON_DATA_FIELD0_OFFSET                    .set    0x06            ;data field 0 offset
NIKON_DATA_FIELD0_CH0_OFFSET                .set    0x00            ;data field 0 ch0 offset
NIKON_DATA_FIELD0_CH1_OFFSET                .set    0x02            ;data field 0 ch1 offset
NIKON_DATA_FIELD0_CH2_OFFSET                .set    0X04            ;data field 0 ch2 offset

NIKON_DATA_FIELD1_OFFSET                    .set    0x0C            ;data field 1 offset
NIKON_DATA_FIELD1_CH0_OFFSET                .set    0x00            ;data field 1 ch0 offset
NIKON_DATA_FIELD1_CH1_OFFSET                .set    0x02            ;data field 1 ch1 offset
NIKON_DATA_FIELD1_CH2_OFFSET                .set    0X04            ;data field 1 ch2 offset

NIKON_DATA_FIELD2_OFFSET                    .set    0x12            ;data field 2 offset
NIKON_DATA_FIELD2_CH0_OFFSET                .set    0x00            ;data field 2 ch0 offset
NIKON_DATA_FIELD2_CH1_OFFSET                .set    0x02            ;data field 2 ch1 offset
NIKON_DATA_FIELD2_CH2_OFFSET                .set    0X04            ;data field 2 ch2 offset

NIKON_POS_DATA_CRC_ERROR_COUNT              .set    0x18            ;CRC Error Count offset
NIKON_POS_DATA_CH0_CRC_ERROR_COUNT          .set    0X18            ;CRC Error Count ch0 offset
NIKON_POS_DATA_CH1_CRC_ERROR_COUNT          .set    0x1C            ;CRC Error Count ch1 offset
NIKON_POS_DATA_CH2_CRC_ERROR_COUNT          .set    0x20            ;CRC Error Count ch2 offset

NIKON_POSITION_DATA_OTF_CRC_OFFSET          .set    0x24            ;8-bit on the fly crc offset
NIKON_POSITION_DATA_OTF_CRC_CH0_OFFSET      .set    0x24            ;8-bit on the fly crc ch0 offset
NIKON_POSITION_DATA_OTF_CRC_CH1_OFFSET      .set    0x25            ;8-bit on the fly crc ch1 offset
NIKON_POSITION_DATA_OTF_CRC_CH2_OFFSET      .set    0x26            ;8-bit on the fly crc ch2 offset

NIKON_POSITION_DATA_RCV_CRC_OFFSET          .set    0x27            ;8-bit receive crc offset
NIKON_POSITION_DATA_RCV_CRC_CH0_OFFSET      .set    0x27            ;8-bit receive crc ch0 offset
NIKON_POSITION_DATA_RCV_CRC_CH1_OFFSET      .set    0x28            ;8-bit receive crc ch1 offset
NIKON_POSITION_DATA_RCV_CRC_CH2_OFFSET      .set    0x29            ;8-bit receive crc ch2 offset

NIKON_POSITION_DATA_ENC1_RES_OFFSET         .set    0x60            ;offset of raw data receive from encoder 1
NIKON_POSITION_DATA_ENC2_RES_OFFSET         .set    0x8C            ;offset of raw data receive from encoder 2


NIKON_CONFIG_DELAY_10US_OFFSET              .set    0xB8            ;10 micro second delay offest

NIKON_CONFIG_DELAY_300US_OFFSET             .set    0xBC            ;300 micro second delay offset

NIKON_CONFIG_DELAY_30MS_OFFSET              .set    0xC0            ;30 milli second delay offset

NIKON_CONFIG_ICSSG_CLK_OFFSET               .set    0xC4            ;icssg clock configuration offset
