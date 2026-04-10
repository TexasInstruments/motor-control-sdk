; Copyright (C) 2022-2025 Texas Instruments Incorporated
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


	.if	!$isdefed("__tamagawa_interface_h")
__tamagawa_interface_h	.set	1

    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
TAMAGAWA_OPMODE_CONFIG_OFFSET      .set    0x0
TAMAGAWA_CHANNEL_CONFIG_OFFSET     .set    0x1
TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET  .set    0x2 ;/*  Command trigger indication to firmware*/
TAMAGAWA_INTFC_CMD_STATUS_OFFSET   .set    0x3 ;/*  Status word for channel busy indication etc - Not used now*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
TAMAGAWA_OPMODE_CONFIG_OFFSET      .set    0x4
TAMAGAWA_CHANNEL_CONFIG_OFFSET     .set    0x5
TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET  .set    0x6 ;/*  Command trigger indication to firmware*/
TAMAGAWA_INTFC_CMD_STATUS_OFFSET   .set    0x7 ;/*  Status word for channel busy indication etc - Not used now*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
TAMAGAWA_OPMODE_CONFIG_OFFSET      .set    0x8
TAMAGAWA_CHANNEL_CONFIG_OFFSET     .set    0x9
TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET  .set    0xA ;/*  Command trigger indication to firmware*/
TAMAGAWA_INTFC_CMD_STATUS_OFFSET   .set    0xB ;/*  Status word for channel busy indication etc - Not used now*/
    .else
TAMAGAWA_OPMODE_CONFIG_OFFSET      .set    0x0
TAMAGAWA_CHANNEL_CONFIG_OFFSET     .set    0x1
TAMAGAWA_INTFC_CMD_TRIGGER_OFFSET  .set    0x2 ;/*  Command trigger indication to firmware*/
TAMAGAWA_INTFC_CMD_STATUS_OFFSET   .set    0x3 ;/*  Status word for channel busy indication etc - Not used now*/
	.endif


    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
TAMAGAWA_WORD_0_OFFSET              .set    0xC    ;/*Tx Data without start and stop bits*/
TAMAGAWA_WORD_1_OFFSET              .set    0x10    ;/*No. of Tx frames,Rx frames expected (first byte is Tx frames second is Rx frames)*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
TAMAGAWA_WORD_0_OFFSET              .set    0x14    ;/*Tx Data without start and stop bits*/
TAMAGAWA_WORD_1_OFFSET              .set    0x18    ;/*No. of Tx frames,Rx frames expected (first byte is Tx frames second is Rx frames)*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
TAMAGAWA_WORD_0_OFFSET              .set    0x1C    ;/*Tx Data without start and stop bits*/
TAMAGAWA_WORD_1_OFFSET              .set    0x20    ;/*No. of Tx frames,Rx frames expected (first byte is Tx frames second is Rx frames)*/
    .else
TAMAGAWA_WORD_0_OFFSET              .set    0xC    ;/*Tx Data without start and stop bits*/
TAMAGAWA_WORD_1_OFFSET              .set    0x10    ;/*No. of Tx frames,Rx frames expected (first byte is Tx frames second is Rx frames)*/
	.endif

TAMAGAWA_CH0_BASE_OFFSET                   .set 0x24
TAMAGAWA_CH0_POSITION_DATA_WORD0_OFFSET    .set TAMAGAWA_CH0_BASE_OFFSET + 0     ;/*Rx position data for ch0 */
TAMAGAWA_CH0_POSITION_DATA_WORD1_OFFSET    .set TAMAGAWA_CH0_BASE_OFFSET + 4   ;
TAMAGAWA_CH0_POSITION_DATA_WORD2_OFFSET    .set TAMAGAWA_CH0_BASE_OFFSET + 8   ;
TAMAGAWA_CH0_CRC_OFFSET                    .set TAMAGAWA_CH0_BASE_OFFSET + 12

TAMAGAWA_CH1_BASE_OFFSET                   .set TAMAGAWA_CH0_CRC_OFFSET + 4
TAMAGAWA_CH1_POSITION_DATA_WORD0_OFFSET    .set TAMAGAWA_CH1_BASE_OFFSET + 0     ;/*Rx position data for ch1 */
TAMAGAWA_CH1_POSITION_DATA_WORD1_OFFSET    .set TAMAGAWA_CH1_BASE_OFFSET + 4   ;
TAMAGAWA_CH1_POSITION_DATA_WORD2_OFFSET    .set TAMAGAWA_CH1_BASE_OFFSET + 8   ;
TAMAGAWA_CH1_CRC_OFFSET                    .set TAMAGAWA_CH1_BASE_OFFSET + 12

TAMAGAWA_CH2_BASE_OFFSET                    .set TAMAGAWA_CH1_CRC_OFFSET  + 4
TAMAGAWA_CH2_POSITION_DATA_WORD0_OFFSET     .set TAMAGAWA_CH2_BASE_OFFSET + 0     ;/*Rx position data for ch2 */
TAMAGAWA_CH2_POSITION_DATA_WORD1_OFFSET     .set TAMAGAWA_CH2_BASE_OFFSET + 4
TAMAGAWA_CH2_POSITION_DATA_WORD2_OFFSET     .set TAMAGAWA_CH2_BASE_OFFSET + 8
TAMAGAWA_CH2_CRC_OFFSET                     .set TAMAGAWA_CH2_BASE_OFFSET + 12


    .if $isdefed("ENABLE_MULTI_MAKE_RTU")
TAMAGAWA_EEPROM_CMD_OFFSET               .set   0x58        ;/*tamagawa eeprom command id offset*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
TAMAGAWA_EEPROM_CMD_OFFSET               .set   0x80        ;/*tamagawa eeprom command id offset*/
    .elseif $isdefed("ENABLE_MULTI_MAKE_TXPRU")
TAMAGAWA_EEPROM_CMD_OFFSET               .set   0xA8        ;/*tamagawa eeprom command id offset*/
    .else
TAMAGAWA_EEPROM_CMD_OFFSET               .set   0x58        ;/*tamagawa eeprom command id offset*/
	.endif

TAMAGAWA_EEPROM_TX_CMD_0_CH0         .set       0x78                                    ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 0*/
TAMAGAWA_EEPROM_TX_CMD_1_CH0         .set       TAMAGAWA_EEPROM_TX_CMD_0_CH0 + 4        ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 0*/
TAMAGAWA_EEPROM_TX_CMD_0_CH1         .set       0xA0                                    ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 1*/
TAMAGAWA_EEPROM_TX_CMD_1_CH1         .set       TAMAGAWA_EEPROM_TX_CMD_0_CH1 + 4        ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 1*/
TAMAGAWA_EEPROM_TX_CMD_0_CH2         .set       0xC8                                   ;/*tamagawa eeprom tx command offset for bytes 0-3 for channel 2*/
TAMAGAWA_EEPROM_TX_CMD_1_CH2         .set       TAMAGAWA_EEPROM_TX_CMD_0_CH2 + 4 ;/*tamagawa eeprom tx command offset for bytes 4-7 for channel 2*/

TAMAGAWA_LS_EXEC_RTU_STATE           .set       0xD0       ;/*tamagawa offset to sync RTU in load share mode*/
TAMAGAWA_LS_EXEC_PRU_STATE           .set       0xD1       ;/*tamagawa offset to sync PRU in load share mode*/
TAMAGAWA_LS_EXEC_TX_PRU_STATE        .set       0xD2       ;/*tamagawa offset to sync TX PRU in load share mode*/
TAMAGAWA_PRIMARY_CORE_MASK_OFFSET    .set       0xD3       ;/*tamagawa primary core mask for load share mode*/

;/* IEP periodic trigger configuration offsets - Per-channel for load share mode */
;/* Structure layout: tamagawa_periodic_trigger_cfg[3] - 8 bytes per channel */
;/* Each channel structure contains: iep_cmp_event(1), iep_cap_event(1), reserved(2), iep_capture_reg(4) */
TAMAGAWA_IEP_BASE_ADDR_OFFSET        .set 0xD4   ;/* IEP base address (4 bytes) for periodic trigger */
;/* Channel 0 trigger params at offset 0xD8 */
TAMAGAWA_CH0_IEP_CMP_EVENT_OFFSET    .set 0xD8   ;/* CH0: iep_cmp_event (1 byte) */
TAMAGAWA_CH0_IEP_CAP_EVENT_OFFSET    .set 0xD9   ;/* CH0: iep_cap_event (1 byte) */
TAMAGAWA_CH0_IEP_RESERVED_OFFSET     .set 0xDA   ;/* CH0: reserved (2 bytes) */
TAMAGAWA_CH0_IEP_CAPTURE_REG_OFFSET  .set 0xDC   ;/* CH0: iep_capture_reg (4 bytes) */
;/* Channel 1 trigger params at offset 0xE0 (0xD8 + 8) */
TAMAGAWA_CH1_IEP_CMP_EVENT_OFFSET    .set 0xE0   ;/* CH1: iep_cmp_event (1 byte) */
TAMAGAWA_CH1_IEP_CAP_EVENT_OFFSET    .set 0xE1   ;/* CH1: iep_cap_event (1 byte) */
TAMAGAWA_CH1_IEP_RESERVED_OFFSET     .set 0xE2   ;/* CH1: reserved (2 bytes) */
TAMAGAWA_CH1_IEP_CAPTURE_REG_OFFSET  .set 0xE4   ;/* CH1: iep_capture_reg (4 bytes) */
;/* Channel 2 trigger params at offset 0xE8 (0xD8 + 16) */
TAMAGAWA_CH2_IEP_CMP_EVENT_OFFSET    .set 0xE8   ;/* CH2: iep_cmp_event (1 byte) */
TAMAGAWA_CH2_IEP_CAP_EVENT_OFFSET    .set 0xE9   ;/* CH2: iep_cap_event (1 byte) */
TAMAGAWA_CH2_IEP_RESERVED_OFFSET     .set 0xEA   ;/* CH2: reserved (2 bytes) */
TAMAGAWA_CH2_IEP_CAPTURE_REG_OFFSET  .set 0xEC   ;/* CH2: iep_capture_reg (4 bytes) */
	.endif
