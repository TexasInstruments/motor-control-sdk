
;
; Copyright (C) 2023 Texas Instruments Incorporated
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
;*   File:     bissc_interface.h                                                    *
;*                                                                                  *
;*   Brief:    Defining Data Memory Offsets                                         *
;************************************************************************************

BISSC_POS_CRC_LEN_CONFIG_OFFSET	                    .set		0x0     ;encoder data crc length
BISSC_RX_CLK_FREQ_CONFIG_OFFSET                     .set		0x1     ;clock freq
BISSC_CTRL_CMD_CRC_LEN_CONFIG_OFFSET	            .set		0x2	    ;register access crc length
BISSC_CHANNEL_CONFIG_OFFSET       	                .set		0x3	    ;channel config

BISSC_STATUS_CH0_CONFIG_OFFSET	                    .set		0x4	    ;Firmware init status for RTU
BISSC_STATUS_CH1_CONFIG_OFFSET                      .set        0x5     ;Firmware init status for PRU
BISSC_STATUS_CH2_CONFIG_OFFSET                      .set        0x6     ;Firmware init status for TXPRU
BISSC_MASK_FOR_PRIMARY_CORE                         .set        0x7     ;mask for core set
BISSC_CYCLE_TRIGGER_CH0_STATUS_OFFSET               .set	    0x8	    ;Status of current cycle for RTU
BISSC_CYCLE_TRIGGER_CH1_STATUS_OFFSET               .set        0x9     ;Status of current cycle for PRU
BISSC_CYCLE_TRIGGER_CH2_STATUS_OFFSET               .set        0xA     ;Status of current cycle for TXPRU
BISSC_MEAS_PROC_DELAY_OFFSET	                    .set	    0xB     ;Measure proc delay and line delay, set when there is a change in config

BISSC_OPMODE_CH0_CONFIG_OFFSET                      .set        0xC     ;operation mode of ch0 offset
BISSC_OPMODE_CH1_CONFIG_OFFSET                      .set        0xD     ;operation mode of ch1 offset
BISSC_OPMODE_CH2_CONFIG_OFFSET                      .set        0xE     ;operation mode of ch2 offset

BISSC_NUM_ENCODER_CH0_CONFIG_OFFSET                 .set        0xF     ;number of encoders connected in daisy chain
BISSC_DATA_LEN_CH0_CONFIG_OFFSET	                .set		0x10	;encoder 0 data length
BISSC_DATA_LEN_ENCODER1_CH0_CONFIG_OFFSET           .set        0x11    ;encoder 1 data length
BISSC_DATA_LEN_ENCODER2_CH0_CONFIG_OFFSET           .set        0x12    ;encoder 2 data length

BISSC_NUM_ENCODER_CH1_CONFIG_OFFSET                 .set        0x13    ;number of encoders connected in daisy chain
BISSC_DATA_LEN_CH1_CONFIG_OFFSET	                .set		0x14	;encoder 0 data length
BISSC_DATA_LEN_ENCODER1_CH1_CONFIG_OFFSET           .set        0x15    ;encoder 1 data length
BISSC_DATA_LEN_ENCODER2_CH1_CONFIG_OFFSET           .set        0x16    ;encoder 2 data length

BISSC_NUM_ENCODER_CH2_CONFIG_OFFSET                 .set        0x17    ;number of encoders connected in daisy chain
BISSC_DATA_LEN_CH2_CONFIG_OFFSET	                .set		0x18	;encoder 0 data length
BISSC_DATA_LEN_ENCODER1_CH2_CONFIG_OFFSET           .set        0x19    ;encoder 1 data length
BISSC_DATA_LEN_ENCODER2_CH2_CONFIG_OFFSET           .set        0x1A    ;encoder 2 data length

BISSC_VALID_BIT_IDX_OFFSET                          .set        0x1B    ;offset for valid bit index
BISSC_FIFO_BIT_IDX_OFFSET                           .set        0x1C    ;offset for fifo bit index
BISSC_CTRL_CMD_CH0_STATUS_OFFSET                    .set	    0x1D    ;control command complete status; 1 - pending; 0 - completed
BISSC_CTRL_CMD_CH1_STATUS_OFFSET                    .set	    0x1E    ;control command complete status; 1 - pending; 0 - completed
BISSC_CTRL_CMD_CH2_STATUS_OFFSET                    .set	    0x1F    ;control command complete status; 1 - pending; 0 - completed

BISSC_CH0_PROC_DELAY_OFFSET	                        .set	    0x20    ;Measured Processing delay for ch0
BISSC_CH1_PROC_DELAY_OFFSET	                        .set	    0x22    ;Measured Processing delay for ch1
BISSC_CH2_PROC_DELAY_OFFSET	                        .set	    0x24    ;Measured Processing delay for ch2


BISSC_CTRL_CMD_CH0_CONFIG_OFFSET                    .set		0x28	;register access commad

BISSC_CTRL_CMD_CH1_CONFIG_OFFSET                    .set		0x2C	;register access commad

BISSC_CTRL_CMD_CH2_CONFIG_OFFSET                    .set		0x30	;register access commad

BISSC_MAX_PROC_DELAY_OFFSET	                        .set	    0x34    ;Max Processing delay

BISSC_POS_DATA_RES_ENCODER0_OFFSET                  .set        0x38    ;position data results offset for encoder 0
BISSC_POS_DATA_RES_ENCODER1_OFFSET                  .set        0x60    ;position data results offset for encoder 1
BISSC_POS_DATA_RES_ENCODER2_OFFSET                  .set        0X88    ;position data results offset for encoder 2

BISSC_POS_DATA_WORD_CH0_OFFSET                      .set        0x0     ;position data results offset for ch 0
BISSC_POS_DATA_WORD_CH1_OFFSET                      .set        0x8     ;position data results offset for ch 1
BISSC_POS_DATA_WORD_CH2_OFFSET                      .set        0x10    ;position data results offset for ch 2
BISSC_POS_CRC_ERR_COUNT_CH0_OFFSET                  .set        0x18    ;position data crc error count for ch 0
BISSC_POS_CRC_ERR_COUNT_CH1_OFFSET                  .set        0x1C    ;position data crc error count for ch 1
BISSC_POS_CRC_ERR_COUNT_CH2_OFFSET                  .set        0x20    ;position data crc error count for ch 2
BISSC_POS_OTF_CRC_CH0_OFFSET                        .set        0x24    ;position data otf crc for ch 0
BISSC_POS_OTF_CRC_CH1_OFFSET                        .set        0x25    ;position data otf crc for ch 1
BISSC_POS_OTF_CRC_CH2_OFFSET                        .set        0x26    ;position data otf crc for ch 2


BISSC_POSITION_DATA_WORD0_CH0_ENCODER0_OFFSET       .set	    0X38    ;Lower word of position data from ch0 encoder 0
BISSC_POSITION_DATA_WORD1_CH0_ENCODER0_OFFSET	    .set	    0x3C    ;Upper word of position data(0 if length < 32) from ch0 encoder 0
BISSC_POSITION_DATA_WORD0_CH1_ENCODER0_OFFSET       .set	    0X40    ;Lower word of position data from ch1 encoder 0
BISSC_POSITION_DATA_WORD1_CH1_ENCODER0_OFFSET	    .set	    0x44    ;Upper word of position data(0 if length < 32) from ch1 encoder 0
BISSC_POSITION_DATA_WORD0_CH2_ENCODER0_OFFSET       .set	    0X48    ;Lower word of position data from ch2 encoder 0
BISSC_POSITION_DATA_WORD1_CH2_ENCODER0_OFFSET	    .set	    0x4C    ;Upper word of position data(0 if length < 32) from ch2 encoder 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_ENCODER0_OFFSET  .set        0x50    ;offset for position data crc errors of ch0 encoder 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_ENCODER0_OFFSET  .set        0x54    ;offset for position data crc errors of ch1 encoder 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_ENCODER0_OFFSET  .set        0x58    ;offset for position data crc errors of ch2 encoder 0
BISSC_POS_DATA_OTF_CRC_CH0_ENCODER0_OFFSET          .set		0x5C    ;6-bit calculated otf pos data CRC of ch0 encoder 0
BISSC_POS_DATA_OTF_CRC_CH1_ENCODER0_OFFSET          .set		0x5D    ;6-bit calculated otf pos data CRC of ch1 encoder 0
BISSC_POS_DATA_OTF_CRC_CH2_ENCODER0_OFFSET          .set		0x5E    ;6-bit calculated otf pos data CRC of ch2 encoder 0

BISSC_POSITION_DATA_WORD0_CH0_ENCODER1_OFFSET       .set	    0X60    ;Lower word of position data from ch0 encoder 1
BISSC_POSITION_DATA_WORD1_CH0_ENCODER1_OFFSET	    .set	    0x64    ;Upper word of position data(0 if length < 32) from ch0 encoder 1
BISSC_POSITION_DATA_WORD0_CH1_ENCODER1_OFFSET       .set	    0X68    ;Lower word of position data from ch1 encoder 1
BISSC_POSITION_DATA_WORD1_CH1_ENCODER1_OFFSET	    .set	    0x6C    ;Upper word of position data(0 if length < 32) from ch1 encoder 1
BISSC_POSITION_DATA_WORD0_CH2_ENCODER1_OFFSET       .set	    0X70    ;Lower word of position data from ch2 encoder 1
BISSC_POSITION_DATA_WORD1_CH2_ENCODER1_OFFSET	    .set	    0x74    ;Upper word of position data(0 if length < 32) from ch2 encoder 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_ENCODER1_OFFSET  .set        0x78    ;offset for position data crc errors of ch0 encoder 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_ENCODER1_OFFSET  .set        0x7C    ;offset for position data crc errors of ch1 encoder 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_ENCODER1_OFFSET  .set        0x80    ;offset for position data crc errors of ch2 encoder 1
BISSC_POS_DATA_OTF_CRC_CH0_ENCODER1_OFFSET          .set		0x84    ;6-bit calculated otf pos data CRC of ch0 encoder 1
BISSC_POS_DATA_OTF_CRC_CH1_ENCODER1_OFFSET          .set		0x85    ;6-bit calculated otf pos data CRC of ch1 encoder 1
BISSC_POS_DATA_OTF_CRC_CH2_ENCODER1_OFFSET          .set		0x86    ;6-bit calculated otf pos data CRC of ch2 encoder 1


BISSC_POSITION_DATA_WORD0_CH0_ENCODER2_OFFSET       .set	    0X88    ;Lower word of position data from ch0 encoder 2
BISSC_POSITION_DATA_WORD1_CH0_ENCODER2_OFFSET	    .set	    0x8C    ;Upper word of position data(0 if length < 32) from ch0 encoder 2
BISSC_POSITION_DATA_WORD0_CH1_ENCODER2_OFFSET       .set	    0X90    ;Lower word of position data from ch1 encoder 2
BISSC_POSITION_DATA_WORD1_CH1_ENCODER2_OFFSET	    .set	    0x94    ;Upper word of position data(0 if length < 32) from ch1 encoder 2
BISSC_POSITION_DATA_WORD0_CH2_ENCODER2_OFFSET       .set	    0X98    ;Lower word of position data from ch2 encoder 2
BISSC_POSITION_DATA_WORD1_CH2_ENCODER2_OFFSET	    .set	    0x9C    ;Upper word of position data(0 if length < 32) from ch2 encoder 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_ENCODER2_OFFSET  .set        0xA0    ;offset for position data crc errors of ch0 encoder 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_ENCODER2_OFFSET  .set        0xA4    ;offset for position data crc errors of ch1 encoder 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_ENCODER2_OFFSET  .set        0xA8    ;offset for position data crc errors of ch2 encoder 2
BISSC_POS_DATA_OTF_CRC_CH0_ENCODER2_OFFSET          .set		0xAC    ;6-bit calculated otf pos data CRC of ch0 encoder 2
BISSC_POS_DATA_OTF_CRC_CH1_ENCODER2_OFFSET          .set		0xAD    ;6-bit calculated otf pos data CRC of ch1 encoder 2
BISSC_POS_DATA_OTF_CRC_CH2_ENCODER2_OFFSET          .set		0xAE    ;6-bit calculated otf pos data CRC of ch2 encoder 2

BISSC_CH0_CTRL_BASE_OFFSET                          .set        0xB0    ;channel 0 control communication base offset

BISSC_CTRL_CMD_CRC_ERROR_COUNT_OFFSET               .set        0x00    ;offset for control command crc errors

BISSC_CTRL_CMD_RESULT_OFFSET	                    .set		0x04    ;8-bit control communication result
BISSC_CTRL_CMD_DATA_CRC_OFFSET	                    .set		0x05    ;4-bit control communication CRC received
BISSC_CTRL_CMD_OTF_CRC_OFFSET                       .set		0x06    ;4-bit otf control communication CRC calculated

BISSC_CH1_CTRL_BASE_OFFSET                          .set        0xB8    ;channel 1 control communication base offset(ch0 base offset + last defined offset)

BISSC_CH2_CTRL_BASE_OFFSET                          .set        0xC0    ;channel 2 control communication base offset(ch1 base offset + last defined offset)

BISSC_LS_EXEC_RTU_STATE                             .set        0xC8    ;RTU execution state offset
BISSC_LS_EXEC_PRU_STATE                             .set        0xC9    ;PRU execution state offset
BISSC_LS_EXEC_TXPRU_STATE                           .set        0xCA    ;TXPRU execution state offset

BISSC_REG_BACKUP                                    .set        0xD0    ;register backup for different channels

BISSC_RE_MEASURE_PROC_DELAY                         .set        0xE8    ;Flag used to indicate remeasure proc delay

BISSC_CONFIG_DELAY_40US_OFFSET                      .set        0xEC    ;40 micro second delay offest

BISSC_CONFIG_DELAY_100MS_OFFSET                     .set        0xF0    ;100 milli second delay offset

BISSC_CONFIG_ICSSG_CLK_OFFSET                       .set        0xF8    ;icssg clock configuration offset
