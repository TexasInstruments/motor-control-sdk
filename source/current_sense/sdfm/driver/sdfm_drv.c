/*
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <drivers/hw_include/tistdtypes.h>
#include <drivers/hw_include/hw_types.h>
#include <current_sense/sdfm/include/sdfm_drv.h>
#include <current_sense/sdfm/include/sdfm_api.h>
#include <current_sense/sdfm/firmware/icssg_sdfm.h>
#include <pruicss_pwm/include/pruicss_pwm.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>
#include <drivers/soc.h>
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>

/* Internal structure for managing each PRU SD */
SDFM g_sdfm[NUM_PRU] = {
    {NULL, NULL, PRU_ID_0, 0, 0, 0, 0, NULL},
    {NULL, NULL, PRU_ID_1, 0, 0, 0, 0, NULL},
};

/* Initialize SDFM instance */
sdfm_handle SDFM_init(PRUICSS_Handle pruIcssHandle, uint8_t pruId, uint8_t coreId)
{
    SDFM *p_sdfm;
    PRUICSS_Handle PruIcssXHandle = pruIcssHandle;
    
    /* Initialize PRU 0 SD */
    p_sdfm = &g_sdfm[pruId];
    p_sdfm->gPruIcssHandle = pruIcssHandle;

    /* Initialize SDFM control address */
    if(pruId == PRU_ID_0)
    {
        switch (coreId)
        {
            case PRUICSS_RTU_PRU0:
            case PRUICSS_RTU_PRU1:               
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + RTUx_DMEM_BASE_ADD;
            
                break;
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + PRUx_DMEM_BASE_ADD;
                
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + TXPRUx_DMEM_BASE_ADD;
                
                break;
            default:
                return NULL;
            break;
        }

    }
   else if (pruId == PRU_ID_1)
   {
        switch (coreId)
        {
            case PRUICSS_RTU_PRU0:
            case PRUICSS_RTU_PRU1:               
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + RTUx_DMEM_BASE_ADD;
            
                break;
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + PRUx_DMEM_BASE_ADD;
                
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                p_sdfm->pSdfmInterface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + TXPRUx_DMEM_BASE_ADD;
                
                break;
            default:
                return NULL;
            break;
        }
   }
   else
   {
        return NULL;
   }
 
    /* Set FW PRU ID */
    p_sdfm->pSdfmInterface->sdfm_ctrl.sdfm_pru_id = pruId;
    return (sdfm_handle)p_sdfm;
}

/*Configuration of iep & pwm time period */
void SDFM_configIepCount(sdfm_handle h_sdfm, uint32_t epwm_out_freq)
{
    /*; IEP0 default increment=1*/
    h_sdfm->pSdfmInterface->sdfm_cfg_iep_ptr.iep_inc_value = h_sdfm->iepInc;
    /*
     IEP0 CMP0 count to simulate EPWM (FOC loop) period:
     - IEP frequency = 300 MHz
     - IEP Default Increment = 1
     - Simulated EPWM frequency = 8e3
     CMP0 = 300e6/1/8e3 = 37500 = 0x927C
    */
    uint32_t cnt_epwm_prd = h_sdfm->iepClock/epwm_out_freq;
    h_sdfm->pSdfmInterface->sdfm_cfg_iep_ptr.cnt_epwm_prd = cnt_epwm_prd;

}

/*ecap configuration for SD clock*/
void SDFM_configEcap(sdfm_handle h_sdfm, uint8_t ecap_divider)
{
    /* SD_PRD_CLOCKS divider = 15; SDFM_CLOCK = IEP freq./SD_PRD_CLOCKS; 300/15  => 20 MHz, SD_CLK_INV==0 => No clock inversion*/
    h_sdfm->pSdfmInterface->sd_clk.sd_prd_clocks = ecap_divider;
    h_sdfm->pSdfmInterface->sd_clk.sd_clk_inv = 0;

    void *pruicssEcap = h_sdfm->pruicssEcap;
    uint32_t rgval;
    uint32_t count;
    
    /*Set eCAP APWM mode*/ 
    rgval = HW_RD_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |= (0<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCI_EN_SHIFT) | (2<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_SYNCO_SEL_SHIFT) | (1<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_CAP_APWM_SHIFT) | (0<<CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_APWMPOL_SHIFT);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

    /*Set period count*/
    count = ecap_divider - 1;
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP1, count);

    /*Compute & set Duty Cycle count.
    Divide period count by 2, biased rounding.*/
    count = count + 1;
    count = count/2;
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CAP2, count);

    /*Clear counter phase and Reset eCAP PWM Counter  */
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_CNTPHS, 0);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_TSCNT, 0);

    /* Enable eCAP APWM*/
    rgval = HW_RD_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1);
    rgval |=  (1 << CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1_TSCNTSTP_SHIFT);
    HW_WR_REG32((uint8_t *)pruicssEcap + CSL_ICSS_G_PR1_ICSS_ECAP0_ECAP_SLV_ECCTL2_ECCTL1, rgval);

}

/*sdfm Hw osr configuration */
 void SDFM_setCompFilterOverSamplingRatio(sdfm_handle h_sdfm, uint8_t ch_id, uint16_t osr)
 {
    /*Over current OSR*/
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].osr = osr - 1;
 }

/*sdfm high, low  threshold config */
void SDFM_setCompFilterThresholds(sdfm_handle h_sdfm, uint8_t ch_id, uint32_t *thresholdParms)
{
   /* SD Over current high threshold */
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.high_threshold = thresholdParms[0];
    /* SD Over current low threshold */
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.low_threshold = thresholdParms[1];

}

/*sdfm smapling time configuation */
void SDFM_setSampleTriggerTime(sdfm_handle h_sdfm, float samp_trig_time)
{   /*convert sample time into IEP count*/
    /*samp time in us */
    int32_t count = (h_sdfm->iepClock /1000000)*((float)samp_trig_time);
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.first_samp_trig_time = count;



}
/*Second normal current sampling configuration*/
void SDFM_enableDoubleSampling(sdfm_handle h_sdfm, float samp_trig_time)
{
    /*Enable double normal current sampling*/
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.en_double_nc_sampling = 1;
    /*Second sample point*/
    int32_t count = (h_sdfm->iepClock /1000000)*((float)samp_trig_time);
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.second_samp_trig_time = count;
}

/*Disable Double update*/
void SDFM_disableDoubleSampling(sdfm_handle h_sdfm)
{
    /*Enable double normal current sampling*/
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.en_double_nc_sampling = 0;
}
/* Enable the channel specified by the channel number parameter*/
void SDFM_setEnableChannel(sdfm_handle h_sdfm, uint8_t channel_number)
{
    uint32_t temp;
    temp  = 1 << channel_number;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        h_sdfm->pSdfmInterface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDFM_CFG_BF_SD_CH0_ID_SHIFT);
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].ch_id = channel_number;
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->pSdfmInterface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number<< SDFM_CFG_BF_SD_CH1_ID_SHIFT);
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].ch_id = channel_number;
    }
    else 
    {
        h_sdfm->pSdfmInterface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDFM_CFG_BF_SD_CH2_ID_SHIFT);
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].ch_id = channel_number;
    }
}
/* set SDFM channel acc source */
void SDFM_configDataFilter(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t filter)
{
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].filter_type = filter;
}

/*set  clock source for SDFM channel*/
void SDFM_selectClockSource(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t clk_source)
{
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].sdfm_clk_parms.clk_source = clk_source;
   
}
/* set clock inversion for SDFM channel*/
void SDFM_setClockInversion(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t clk_inv)
{
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch_id].sdfm_clk_parms.clk_inv = clk_inv;
}
/* Enable the comparator feature for a specified filter/channel */
void SDFM_enableComparator(sdfm_handle h_sdfm, uint8_t ch)
{
    /*It is setting bits for enable Over current: 0th bit for Over current enable & from 1st bits to 3rd bits are for ch0 to ch2.*/
    h_sdfm->pSdfmInterface->sdfm_ch_ctrl.enable_comparator |= ((1 << (ch+1))|(1));
}

/* Disable the comparator feature for a specified filter/channel */
void SDFM_disableComparator(sdfm_handle h_sdfm, uint8_t ch)
{
    h_sdfm->pSdfmInterface->sdfm_ch_ctrl.enable_comparator &= (0xFFFF ^ (1<<ch));
}
/*GPIO configuration*/
void SDFM_configComparatorGpioPins(sdfm_handle h_sdfm, uint8_t ch,uint32_t gpio_base_addr, uint32_t pin_number)
{

    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) gpio_base_addr);
    uint32_t reg_index = GPIO_GET_REG_INDEX(pin_number);
    uint32_t reg_val = GPIO_GET_BIT_MASK(pin_number);
    uint32_t clr_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].CLR_DATA;
    uint32_t set_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].SET_DATA;

    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].sdfm_gpio_params.write_val = reg_val;
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].sdfm_gpio_params.set_val_addr = set_data_addr;
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].sdfm_gpio_params.clr_val_addr = clr_data_addr;
}

/* Get current (or latest) sample for the specified channel */
uint32_t SDFM_getFilterData(sdfm_handle h_sdfm, uint8_t ch)
{
    return h_sdfm->sampleOutputInterface->sampleOutput[ch];
}

/*Configure normal current OSR for data filter*/
void SDFM_setFilterOverSamplingRatio(sdfm_handle h_sdfm, uint16_t nc_osr)
{
    
    /*IEP0 counts in normal current sampling period*/
    uint16_t count;
    uint32_t iep_freq = h_sdfm->iepClock;
    uint32_t sd_clock = h_sdfm->sdfmClock;
    count = (int)((float)nc_osr*((float)iep_freq/(float)sd_clock));
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.nc_prd_iep_cnt = count;
}
/*return firmware version */
uint32_t SDFM_getFirmwareVersion(sdfm_handle h_sdfm)
{
   return h_sdfm->pSdfmInterface->firmwareVersion >> SDFM_FW_VERSION_BIT_SHIFT;
}
/*Enable free run NC */
void SDFM_enableContinuousNormalCurrent(sdfm_handle h_sdfm)
{
    h_sdfm->pSdfmInterface->sdfm_cfg_trigger.en_continuous_mode = 1;
}
/*FD block confiuration */
void SDFM_configFastDetect(sdfm_handle h_sdfm, uint8_t ch, uint8_t *fdParms)
{
    h_sdfm->pSdfmInterface->sdfm_ch_ctrl.enFastDetect |= fdParms[0]<<ch;
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].fd_window = fdParms[1];
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].fd_zero_max = fdParms[2];
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].fd_zero_min = fdParms[3];
      
    /*Configure one max to window size + 1 and one min to 0, so they never get set*/
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].fd_one_max = (fdParms[1] + 1) * 4 + 1;
    h_sdfm->pSdfmInterface->sdfm_cfg_ptr[ch].fd_one_min = 0;

}

/*return status of PWM trip vector status bit*/
int32_t SDFM_getFastDetectErrorStatus(sdfm_handle h_sdfm, uint8_t chNum) 
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pruPwmHandle = h_sdfm->gPruPwmHandle;
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    
    /*PWM trip vector */
    retVal = PRUICSS_PWM_getPwmTripTriggerCauseVector(pruPwmHandle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    else
    {

        retVal =  retVal >> 2;
        uint32_t temp;
        temp  = 1 << chNum;
        if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
        {
            return ((retVal) & (1 << SDFM_CHANNEL0)) ? 1 : 0;
        }
        else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
        {
            return ((retVal) & (1 << SDFM_CHANNEL1))? 1 : 0;
        }
        else 
        {
            return ((retVal) & (1 << SDFM_CHANNEL2)) ? 1 : 0;
        }
        
    }
    
}

/*Clear Trip status bit*/
int32_t SDFM_clearPwmTripStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pruPwmHandle = h_sdfm->gPruPwmHandle;
    
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*clear trip status*/
    retVal = PRUICSS_PWM_generatePwmTripReset(pruPwmHandle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*clear trip reset status*/
    retVal = PRUICSS_PWM_clearPwmTripResetStatus(pruPwmHandle, pwmSet);

    return retVal;
}
/*Enable Load share mode*/
void SDFM_enableLoadShareMode(sdfm_handle h_sdfm, uint8_t sliceId)
{
    void *pruicssCfg = h_sdfm->pruicssCfg;
   
    uint32_t rgval;
    if(sliceId)
    {
       rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU1CLKDIV);
       rgval |= CSL_ICSSCFG_SDPRU1CLKDIV_PRU1_SD_SHARE_EN_MASK;
       HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU1CLKDIV, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU0CLKDIV);
        rgval |= CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_SDPRU0CLKDIV, rgval);
    }

}
/*Measure Phase delay*/
void SDFM_measureClockPhaseDelay(sdfm_handle h_sdfm, uint16_t clkEdg)
{
    /*enable phase delay measurement*/
    h_sdfm->pSdfmInterface->sdfm_ch_ctrl.en_phase_delay = 1;
    /*waiting till measurment done */
    uint8_t ack = h_sdfm->pSdfmInterface->sdfm_ch_ctrl.en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK;
    while(ack)
    {
       ack = h_sdfm->pSdfmInterface->sdfm_ch_ctrl.en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK ;
    }


   uint16_t nEdge = h_sdfm->pSdfmInterface->sdfm_ch_ctrl.clock_edge;
   float temp = h_sdfm->pSdfmInterface->sdfm_ch_ctrl.clock_phase_delay;
   /*avg*/
    temp = temp/SDFM_PHASE_DELAY_CAL_LOOP_SIZE;
   /*check data reading edge(clk polarity) & nearest edge */
   if(nEdge == clkEdg)
   {
      /*PRU cycles for half SD clock period*/
      uint32_t pruCycles = ceil(((float)h_sdfm->pruCoreClk)/(2*h_sdfm->sdfmClock));
      h_sdfm->pSdfmInterface->sdfm_ch_ctrl.clock_phase_delay = pruCycles - temp;
   }
   else
   {
      /*PRU cycles for one SD clock period*/
      uint32_t pruCycles = ceil((float)(h_sdfm->pruCoreClk/(h_sdfm->sdfmClock)));
      h_sdfm->pSdfmInterface->sdfm_ch_ctrl.clock_phase_delay = pruCycles - temp;
   }

}
float SDFM_getClockPhaseDelay(sdfm_handle h_sdfm)
{
    /*conversion from PRU cycle to ns */
    float phaseDelay =  ((float)h_sdfm->pSdfmInterface->sdfm_ch_ctrl.clock_phase_delay * 1000000000)/h_sdfm->pruCoreClk;
    return phaseDelay;
}
uint8_t SDFM_getHighThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.highThStatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.highThStatus;
    }
    else 
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.highThStatus;
    }
     
}
uint8_t SDFM_getLowThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.lowThStatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.lowThStatus;
    }
    else 
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.lowThStatus;
    }
}

int32_t SDFM_clearOverCurrentError(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_PWM_Handle pruPwmHandle = h_sdfm->gPruPwmHandle;
    if(chNum < SDFM_CHANNEL3)
    {
        pwmSet = 0;
    }
    else if (chNum > SDFM_CHANNEL2 && chNum < SDFM_CHANNEL6)
    {
        pwmSet = 1;
    }
    else if (chNum > SDFM_CHANNEL5 && chNum <= SDFM_CHANNEL8)
    {
        pwmSet = 2;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*Clear over current Error PWM trip*/
    retVal = PRUICSS_PWM_clearPwmOverCurrentErrorTrip(pruPwmHandle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }
    
    /*Clear PWM trip*/
    retVal = SDFM_clearPwmTripStatus(h_sdfm, chNum);
    return retVal;
}
void SDFM_enableZeroCrossDetection(sdfm_handle h_sdfm, uint8_t chNum, uint32_t zcThr)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
    else 
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
   
}
uint8_t SDFM_getZeroCrossThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossThstatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossThstatus; 
    }
    else 
    {
        return h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossThstatus;  
    }

}
void SDFM_disableZeroCrossDetection(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
    else 
    {
        h_sdfm->pSdfmInterface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
   
}

int32_t SDFM_enableEpwmSync(sdfm_handle h_sdfm, uint8_t epwmIns)
{
    void *pru_iep = h_sdfm->pruicssIep;
    int32_t   retVal = SystemP_FAILURE;
    
    if(pru_iep != NULL && (epwmIns == 0 || epwmIns == 3))
    {
        retVal = SystemP_SUCCESS;

        switch (epwmIns)
        {
            case 0:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM0_RST_CNT_EN, 1);
                break;
             case 3:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM3_RST_CNT_EN, 1);
                break;
        }
    }
    
    return retVal;
}

int32_t SDFM_disableEpwmSync(sdfm_handle h_sdfm, uint8_t epwmIns)
{
    void *pru_iep = h_sdfm->pruicssIep;
    int32_t   retVal = SystemP_FAILURE;
    
    if(pru_iep != NULL && (epwmIns == 0 || epwmIns == 3))
    {
        retVal = SystemP_SUCCESS;

        switch (epwmIns)
        {
            case 0:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM0_RST_CNT_EN, 0);
                break;
             case 3:
                HW_WR_FIELD32(((uint8_t *)pru_iep + CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_PWM_REG_PWM3_RST_CNT_EN, 0);
                break;
        }
    }
    
    return retVal;
}

int32_t SDFM_configIepSyncMode(sdfm_handle h_sdfm, uint32_t highPulseWidth, uint32_t periodTime, uint32_t syncStartTime)
{

    void        *pruIep = h_sdfm->pruicssIep;
    int32_t     retVal = SystemP_FAILURE;
    uint32_t    regVal ;

    if(pruIep != NULL)
    {
       
        /*Set CMP1 period - SYNC0 trigger */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0, syncStartTime);

        /*Set CMP2 period - SYNC1 trigger */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP2_REG0, syncStartTime);
               
        /*Set sync ctrl register: SYNC1 dependent, cyclic generation , SYNC0 and SYNC1 enable, SYNC enable*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_EN_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_EN_SHIFT);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_SHIFT) | (1<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_SHIFT) | (0<<CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG_SYNC1_IND_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG, regVal);

        /*Set SYNC0/1 high pulse time in iep clok cycles  */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, highPulseWidth);

        /*Set SYNC0/1 period*/
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC0_PERIOD_REG, periodTime);

        /*Set offset from cpm hit*/
        HW_WR_REG32( (uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_START_REG, 0);

        /*Enable cmp1 and cmp2 for sync start trigger generation*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
        regVal |= (1<<SDFM_IEP_CMP1_EN_SHIFT)|(1<<SDFM_IEP_CMP2_EN_SHIFT);
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, regVal);
       
        /*Set default and compensation increment to 1*/
        regVal = HW_RD_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        regVal |= (1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT)|(1<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT );
        HW_WR_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

        retVal = SystemP_SUCCESS;

    }
    
    return retVal;
}

int32_t SDFM_enableIep(sdfm_handle h_sdfm)
{
    void       *pruIep = h_sdfm->pruicssIep;
    int32_t    retVal = SystemP_FAILURE;
    uint32_t   regVal ;

    if(pruIep != NULL)
    {
        /*start iep0_timer*/
        regVal = HW_RD_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG);
        regVal |= 0x1;
        HW_WR_REG8((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, regVal);

        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t SDFM_configSync1Delay(sdfm_handle h_sdfm, uint32_t delay)
{
    void      *pruIep = h_sdfm->pruicssIep;
    int32_t   retVal = SystemP_FAILURE;

    if(pruIep != NULL)
    {
        /*Set delay between SYNC0 and SYNC1 in clock cycles */
        HW_WR_REG32((uint8_t *)pruIep + CSL_ICSS_G_PR1_IEP0_SLV_SYNC1_DELAY_REG, delay);

        retVal = SystemP_SUCCESS;
    }

    return retVal;
}



int32_t SDFM_configClockFromGPO1(sdfm_handle h_sdfm, uint8_t div0, uint8_t div1)
{
   
    uint32_t rgval;
    void *pruicssCfg = h_sdfm->pruicssCfg;
    int32_t   retVal = SystemP_SUCCESS;
    
    if( div0 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MAX || div1 >= CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MAX )
    {
        retVal = SystemP_FAILURE;
        return retVal;
    }
    /*configure divider*/
    if(h_sdfm->pruId == 1)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);

        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div0<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
        
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (div1<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
    }
    

    /*enabling shift mode */
    if(h_sdfm->pruId == 1)
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1);
        rgval |= (CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MAX<<CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_SHIFT)&(CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG1, rgval);
    }
    {
        rgval = HW_RD_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0);
        rgval |= (CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MAX<<CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_SHIFT)&(CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MASK);
        HW_WR_REG32((uint8_t *)pruicssCfg + CSL_ICSSCFG_GPCFG0, rgval);
    }

    return retVal;
    
}
/* SDFM global enable */
void SDFM_enable(sdfm_handle h_sdfm)
{
    uint8_t sdfm_en_ack;

    /*Enable SDFM */
    h_sdfm->pSdfmInterface->sdfm_ctrl.sdfm_en = 1;

    /* wait for ACK */
    do {
        sdfm_en_ack = h_sdfm->pSdfmInterface->sdfm_ctrl.sdfm_en_ack;
    } while (sdfm_en_ack != BF_SDFM_EN_ENABLE);


}


