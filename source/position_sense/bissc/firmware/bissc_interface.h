
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

BISSC_POS_CRC_LEN_CONFIG_OFFSET	                    .set		0x0     ;slave data crc length
BISSC_RX_CLK_FREQ_CONFIG_OFFSET                     .set		0x1     ;clock freq
BISSC_CTRL_CMD_CRC_LEN_CONFIG_OFFSET	            .set		0x2	    ;register access crc length
BISSC_CHANNEL_CONFIG_OFFSET       	                .set		0x3	    ;channel config

BISSC_STATUS_CH0_CONFIG_OFFSET	                    .set		0x4	    ;Firmware init status for RTU
BISSC_STATUS_CH1_CONFIG_OFFSET                      .set        0x5     ;Firmware init status for PRU
BISSC_STATUS_CH2_CONFIG_OFFSET                    .set        0x6     ;Firmware init status for TXPRU
BISSC_MASK_FOR_PRIMARY_CORE                               .set        0x7     ;mask for core set
BISSC_CYCLE_TRIGGER_CH0_STATUS_OFFSET               .set	    0x8	    ;Status of current cycle for RTU
BISSC_CYCLE_TRIGGER_CH1_STATUS_OFFSET               .set        0x9     ;Status of current cycle for PRU
BISSC_CYCLE_TRIGGER_CH2_STATUS_OFFSET             .set        0xA     ;Status of current cycle for TXPRU
BISSC_MEAS_PROC_DELAY_OFFSET	                    .set	    0xB     ;Measure proc delay and line delay, set when there is a change in config

BISSC_NUM_SLAVE_CH0_CONFIG_OFFSET                       .set        0xC     ;number of slaves connected in daisy chain
BISSC_DATA_LEN_CH0_CONFIG_OFFSET	                    .set		0xD	    ;slave 0 data length
BISSC_DATA_LEN_SLAVE1_CH0_CONFIG_OFFSET                 .set        0xE     ;slave 1 data length
BISSC_DATA_LEN_SLAVE2_CH0_CONFIG_OFFSET                 .set        0xF     ;slave 2 data length

BISSC_NUM_SLAVE_CH1_CONFIG_OFFSET                       .set        0x10     ;number of slaves connected in daisy chain
BISSC_DATA_LEN_CH1_CONFIG_OFFSET	                    .set		0x11	    ;slave 0 data length
BISSC_DATA_LEN_SLAVE1_CH1_CONFIG_OFFSET                 .set        0x12     ;slave 1 data length
BISSC_DATA_LEN_SLAVE2_CH1_CONFIG_OFFSET                 .set        0x13     ;slave 2 data length

BISSC_NUM_SLAVE_CH2_CONFIG_OFFSET                       .set        0x14     ;number of slaves connected in daisy chain
BISSC_DATA_LEN_CH2_CONFIG_OFFSET	                    .set		0x15	    ;slave 0 data length
BISSC_DATA_LEN_SLAVE1_CH2_CONFIG_OFFSET                 .set        0x16     ;slave 1 data length
BISSC_DATA_LEN_SLAVE2_CH2_CONFIG_OFFSET                 .set        0x17     ;slave 2 data length

BISSC_VALID_BIT_IDX_OFFSET                          .set        0x18    ;offset for valid bit index
BISSC_FIFO_BIT_IDX_OFFSET                           .set        0x19    ;offset for fifo bit index  
BISSC_CTRL_CMD_CH0_STATUS_OFFSET                        .set	    0x1A    ;control command complete status; 1 - pending; 0 - completed
BISSC_CTRL_CMD_CH1_STATUS_OFFSET                        .set	    0x1B    ;control command complete status; 1 - pending; 0 - completed
BISSC_CTRL_CMD_CH2_STATUS_OFFSET                        .set	    0x1C    ;control command complete status; 1 - pending; 0 - completed
    
BISSC_CH0_PROC_DELAY_OFFSET	                        .set	    0x1E    ;Measured Processing delay for ch0
BISSC_CH1_PROC_DELAY_OFFSET	                        .set	    0x20    ;Measured Processing delay for ch1
BISSC_CH2_PROC_DELAY_OFFSET	                        .set	    0x22    ;Measured Processing delay for ch2


BISSC_CTRL_CMD_CH0_CONFIG_OFFSET                        .set		0x24	;register access commad

BISSC_CTRL_CMD_CH1_CONFIG_OFFSET                        .set		0x28	;register access commad

BISSC_CTRL_CMD_CH2_CONFIG_OFFSET                        .set		0x2C	;register access commad

BISSC_MAX_PROC_DELAY_OFFSET	                        .set	    0x30    ;Max Processing delay

BISSC_POS_DATA_RES_SLAVE0_OFFSET                    .set        0x34    ;position data results offset for slave 0
BISSC_POS_DATA_RES_SLAVE1_OFFSET                    .set        0x5C    ;position data results offset for slave 1
BISSC_POS_DATA_RES_SLAVE2_OFFSET                    .set        0X84    ;position data results offset for slave 2

BISSC_POS_DATA_WORD_CH0_OFFSET                      .set        0x0     ;position data results offset for ch 0
BISSC_POS_DATA_WORD_CH1_OFFSET                      .set        0x8     ;position data results offset for ch 1
BISSC_POS_DATA_WORD_CH2_OFFSET                      .set        0x10    ;position data results offset for ch 2
BISSC_POS_CRC_ERR_COUNT_CH0_OFFSET                  .set        0x18    ;position data crc error count for ch 0
BISSC_POS_CRC_ERR_COUNT_CH1_OFFSET                  .set        0x1C    ;position data crc error count for ch 1
BISSC_POS_CRC_ERR_COUNT_CH2_OFFSET                  .set        0x20    ;position data crc error count for ch 2
BISSC_POS_OTF_CRC_CH0_OFFSET                        .set        0x24    ;position data otf crc for ch 0
BISSC_POS_OTF_CRC_CH1_OFFSET                        .set        0x25    ;position data otf crc for ch 1
BISSC_POS_OTF_CRC_CH2_OFFSET                        .set        0x26    ;position data otf crc for ch 2


BISSC_POSITION_DATA_WORD0_CH0_SLAVE0_OFFSET         .set	    0X34    ;Lower word of position data from ch0 slave 0
BISSC_POSITION_DATA_WORD1_CH0_SLAVE0_OFFSET	        .set	    0x38    ;Upper word of position data(0 if length < 32) from ch0 slave 0
BISSC_POSITION_DATA_WORD0_CH1_SLAVE0_OFFSET         .set	    0X3C    ;Lower word of position data from ch1 slave 0
BISSC_POSITION_DATA_WORD1_CH1_SLAVE0_OFFSET	        .set	    0x40    ;Upper word of position data(0 if length < 32) from ch1 slave 0
BISSC_POSITION_DATA_WORD0_CH2_SLAVE0_OFFSET         .set	    0X44    ;Lower word of position data from ch2 slave 0
BISSC_POSITION_DATA_WORD1_CH2_SLAVE0_OFFSET	        .set	    0x48    ;Upper word of position data(0 if length < 32) from ch2 slave 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_SLAVE0_OFFSET    .set        0x4C    ;offset for position data crc errors of ch0 slave 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_SLAVE0_OFFSET    .set        0x50    ;offset for position data crc errors of ch1 slave 0
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_SLAVE0_OFFSET    .set        0x54    ;offset for position data crc errors of ch2 slave 0
BISSC_POS_DATA_OTF_CRC_CH0_SLAVE0_OFFSET            .set		0x58    ;6-bit calculated otf pos data CRC of ch0 slave 0
BISSC_POS_DATA_OTF_CRC_CH1_SLAVE0_OFFSET            .set		0x59    ;6-bit calculated otf pos data CRC of ch1 slave 0
BISSC_POS_DATA_OTF_CRC_CH2_SLAVE0_OFFSET            .set		0x5A    ;6-bit calculated otf pos data CRC of ch2 slave 0

BISSC_POSITION_DATA_WORD0_CH0_SLAVE1_OFFSET         .set	    0X5C    ;Lower word of position data from ch0 slave 1
BISSC_POSITION_DATA_WORD1_CH0_SLAVE1_OFFSET	        .set	    0x60    ;Upper word of position data(0 if length < 32) from ch0 slave 1
BISSC_POSITION_DATA_WORD0_CH1_SLAVE1_OFFSET         .set	    0X64    ;Lower word of position data from ch1 slave 1
BISSC_POSITION_DATA_WORD1_CH1_SLAVE1_OFFSET	        .set	    0x68    ;Upper word of position data(0 if length < 32) from ch1 slave 1
BISSC_POSITION_DATA_WORD0_CH2_SLAVE1_OFFSET         .set	    0X6C    ;Lower word of position data from ch2 slave 1
BISSC_POSITION_DATA_WORD1_CH2_SLAVE1_OFFSET	        .set	    0x70    ;Upper word of position data(0 if length < 32) from ch2 slave 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_SLAVE1_OFFSET    .set        0x74    ;offset for position data crc errors of ch0 slave 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_SLAVE1_OFFSET    .set        0x78    ;offset for position data crc errors of ch1 slave 1
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_SLAVE1_OFFSET    .set        0x7C    ;offset for position data crc errors of ch2 slave 1
BISSC_POS_DATA_OTF_CRC_CH0_SLAVE1_OFFSET            .set		0x80    ;6-bit calculated otf pos data CRC of ch0 slave 1
BISSC_POS_DATA_OTF_CRC_CH1_SLAVE1_OFFSET            .set		0x81    ;6-bit calculated otf pos data CRC of ch1 slave 1
BISSC_POS_DATA_OTF_CRC_CH2_SLAVE1_OFFSET            .set		0x82    ;6-bit calculated otf pos data CRC of ch2 slave 1


BISSC_POSITION_DATA_WORD0_CH0_SLAVE2_OFFSET         .set	    0X84    ;Lower word of position data from ch0 slave 2
BISSC_POSITION_DATA_WORD1_CH0_SLAVE2_OFFSET	        .set	    0x88    ;Upper word of position data(0 if length < 32) from ch0 slave 2
BISSC_POSITION_DATA_WORD0_CH1_SLAVE2_OFFSET         .set	    0X8C    ;Lower word of position data from ch1 slave 2
BISSC_POSITION_DATA_WORD1_CH1_SLAVE2_OFFSET	        .set	    0x90    ;Upper word of position data(0 if length < 32) from ch1 slave 2
BISSC_POSITION_DATA_WORD0_CH2_SLAVE2_OFFSET         .set	    0X94    ;Lower word of position data from ch2 slave 2
BISSC_POSITION_DATA_WORD1_CH2_SLAVE2_OFFSET	        .set	    0x98    ;Upper word of position data(0 if length < 32) from ch2 slave 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH0_SLAVE2_OFFSET    .set        0x9C    ;offset for position data crc errors of ch0 slave 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH1_SLAVE2_OFFSET    .set        0xA0    ;offset for position data crc errors of ch1 slave 2
BISSC_POS_DATA_CRC_ERROR_COUNT_CH2_SLAVE2_OFFSET    .set        0xA4    ;offset for position data crc errors of ch2 slave 2
BISSC_POS_DATA_OTF_CRC_CH0_SLAVE2_OFFSET            .set		0xA8    ;6-bit calculated otf pos data CRC of ch0 slave 2
BISSC_POS_DATA_OTF_CRC_CH1_SLAVE2_OFFSET            .set		0xA9    ;6-bit calculated otf pos data CRC of ch1 slave 2
BISSC_POS_DATA_OTF_CRC_CH2_SLAVE2_OFFSET            .set		0xAA    ;6-bit calculated otf pos data CRC of ch2 slave 2

BISSC_CH0_CTRL_BASE_OFFSET                          .set        0xAC    ;channel 0 control communication base offset

BISSC_CTRL_CMD_CRC_ERROR_COUNT_OFFSET               .set        0x00    ;offset for control command crc errors

BISSC_CTRL_CMD_RESULT_OFFSET	                    .set		0x04    ;8-bit control communication result
BISSC_CTRL_CMD_DATA_CRC_OFFSET	                    .set		0x05    ;4-bit control communication CRC received
BISSC_CTRL_CMD_OTF_CRC_OFFSET                       .set		0x06    ;4-bit otf control communication CRC calculated

BISSC_CH1_CTRL_BASE_OFFSET                          .set        0xB4    ;channel 1 control communication base offset(ch0 base offset + last defined offset)

BISSC_CH2_CTRL_BASE_OFFSET                          .set        0xBC    ;channel 2 control communication base offset(ch1 base offset + last defined offset)

BISSC_LS_EXEC_RTU_STATE                             .set        0xC4    ;RTU execution state offset
BISSC_LS_EXEC_PRU_STATE                             .set        0xC5    ;PRU execution state offset
BISSC_LS_EXEC_TXPRU_STATE                           .set        0xC6    ;TXPRU execution state offset

BISSC_REG_BACKUP                                    .set        0xC8    ;register backup for different channels

BISSC_RE_MEASURE_PROC_DELAY                             .set        0xE0

BISSC_CONFIG_DELAY_40US_OFFSET                      .set        0xE4    ;40 micro second delay offest

BISSC_CONFIG_DELAY_100MS_OFFSET                     .set        0xE8    ;100 milli second delay offset

BISSC_CONFIG_ICSSG_CLK_OFFSET                       .set        0xF0    ;icssg clock configuration offset
