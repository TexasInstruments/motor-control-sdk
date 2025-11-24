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
;*   File:     endat3_icss_reg_defs.h                                               *
;*                                                                                  *
;*    Brief:  Defining R30, R31 register bits as specific status bits for different *
;*		      channels.  							    							*
;************************************************************************************
    .include "pru_io/firmware/common/icss_regs.inc"
    .include "pru_io/firmware/common/icss_cfg_regs.inc"
    .include "pru_io/firmware/common/icss_constant_defines.inc"
    .include "pru_io/firmware/common/icss_xfer_defines.inc"
;******************************************************************************
; Channel-specific Bit Flags
;******************************************************************************
; R31 flags for RX
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
RX_OVERFLOW_FLAG                .set                    29
RX_VALID_FLAG                   .set                    26
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
RX_OVERFLOW_FLAG                .set                    28
RX_VALID_FLAG                   .set                    25
    .elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
RX_OVERFLOW_FLAG                .set                    27
RX_VALID_FLAG                   .set                    24
    .else
RX_OVERFLOW_FLAG                .set                    27
RX_VALID_FLAG                   .set                    24
    .endif

; R30 flags for RX
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
RX_ENABLE                       .set                    26
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
RX_ENABLE                       .set                    25
    .elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
RX_ENABLE                       .set                    24
    .else
RX_ENABLE                       .set                    24
    .endif

; Global TX flags
TX_GLOBAL_TX_GO                 .set                    20
TX_GLOBAL_REINIT                .set                    19
TX_CHANNEL_GO                   .set                    18

; Channel-specific TX flags
    .if $isdefed("ENABLE_MULTI_MAKE_TXPRU")
TX_GLOBAL_REINIT_ACTIVE         .set                    21
TX_UNDERRUN                     .set                    17
TX_OVERRUN                      .set                    16
    .elseif $isdefed("ENABLE_MULTI_MAKE_PRU")
TX_GLOBAL_REINIT_ACTIVE         .set                    13
TX_UNDERRUN                     .set                     9
TX_OVERRUN                      .set                     8
    .elseif $isdefed("ENABLE_MULTI_MAKE_RTU")
TX_GLOBAL_REINIT_ACTIVE         .set                     5
TX_UNDERRUN                     .set                     1
TX_OVERRUN                      .set                     0
    .else
TX_GLOBAL_REINIT_ACTIVE         .set                     5
TX_UNDERRUN                     .set                     1
TX_OVERRUN                      .set                     0
    .endif