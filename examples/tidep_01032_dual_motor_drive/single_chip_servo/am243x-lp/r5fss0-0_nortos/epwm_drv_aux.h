/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EPWM_DRV_AUX_H_
#define _EPWM_DRV_AUX_H_

#include <stdint.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/epwm.h>

/* Write EPWM CMPA */
static inline void writeCmpA(
    uint32_t baseAddr,
    uint32_t cmpVal
)
{
    HW_WR_REG16((baseAddr + PWMSS_EPWM_CMPA), (uint16_t)cmpVal);
}

/* Write EPWM CMPB */
static inline void writeCmpB(
    uint32_t baseAddr,
    uint32_t cmpVal
)
{
    HW_WR_REG16((baseAddr + PWMSS_EPWM_CMPB), (uint16_t)cmpVal);
}

/* Write EPWM CMPA/CMPB */
static inline void writeCmpAB(
    uint32_t baseAddr,
    uint32_t cmpAVal,
    uint32_t cmpBVal
)
{
    HW_WR_REG16((baseAddr + PWMSS_EPWM_CMPA), (uint16_t)cmpAVal);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_CMPB), (uint16_t)cmpBVal);
}

/* Configure Output ChannelA AQ Zero  */
static inline void cfgOutChAAqZero(
    uint32_t baseAddr,
    uint32_t zeroAction
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_AQCTLA);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_AQCTLA_ZRO, zeroAction);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_AQCTLA), (uint16_t)regVal);
}

/* Configure Output ChannelA AQ CMPA Up  */
static inline void cfgOutChAAqCAU(
    uint32_t baseAddr,
    uint32_t cmpAUpAction
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_AQCTLA);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_AQCTLA_CAU, cmpAUpAction);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_AQCTLA), (uint16_t)regVal);
};

/* Configure Output ChannelA AQ CMPA Down */
static inline void cfgOutChAAqCAD(
    uint32_t baseAddr,
    uint32_t cmpADownAction
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_AQCTLA);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_AQCTLA_CAD, cmpADownAction);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_AQCTLA), (uint16_t)regVal);
}

/* Configure Output ChannelB AQ Zero  */
static inline void cfgOutChBAqZero(
    uint32_t baseAddr,
    uint32_t zeroAction
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_AQCTLB);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_AQCTLB_ZRO, zeroAction);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_AQCTLB), (uint16_t)regVal);
}

/* Configure Output ChannelA AQ CMPB Up  */
static inline void cfgOutChAAqCBU(
    uint32_t baseAddr,
    uint32_t cmpBUpAction
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_AQCTLA);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_AQCTLA_CBU, cmpBUpAction);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_AQCTLA), (uint16_t)regVal);
}

/* Write TB Period */
static inline void writeTbPrd(
    uint32_t baseAddr,
    uint32_t tbPeriodCount
)
{
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBPRD), (uint16_t)tbPeriodCount);
}

/* Write TB Phase */
static inline void writeTbPhase(
    uint32_t baseAddr,
    uint32_t tbPhsValue
)
{
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBPHS), (uint16_t)tbPhsValue);    
}

/* Write TBCTL HSPDIV & CLKDIV */
static inline void writeTbClkDiv(
    uint32_t baseAddr,
    uint32_t hspClkDiv, 
    uint32_t clkDiv
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_TBCTL);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_CLKDIV, clkDiv);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_HSPCLKDIV, hspClkDiv);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBCTL), (uint16_t)regVal);
}

/* Write TBCTL CTRMODE */
static inline void writeTbCtrMode(
    uint32_t baseAddr,
    uint32_t ctrMode
)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_TBCTL);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_CTRMODE, ctrMode);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBCTL), (uint16_t)regVal);
}

/* Configure PWM Time base counter Frequency/Period */
void tbPwmFreqCfg(
    uint32_t baseAddr,
    uint32_t tbClk,
    uint32_t pwmFreq,
    uint32_t counterDir,
    uint32_t enableShadowWrite, 
    uint32_t *pPeriodCount
);

#endif /* _EPWM_DRV_AUX_H_ */
