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
    {NULL, PRU_ID_0, 0, 0, 0, 0, NULL},
    {NULL, PRU_ID_1, 0, 0, 0, 0, NULL},
};

/* Initialize SDFM instance */
sdfm_handle SDFM_init(sdfm_handle h_sdfm, uint8_t pruId, uint8_t coreId)
{
    SDFM *p_sdfm;
    PRUICSS_Handle PruIcssXHandle = h_sdfm->gPruIcssHandle;
    
    /* Initialize PRU 0 SD */
    p_sdfm = &g_sdfm[pruId];
    p_sdfm->gPruIcssHandle = h_sdfm->gPruIcssHandle;

    /* Initialize SDFM control address */
    if(pruId == PRU_ID_0)
    {
        switch (coreId)
        {
            case PRUICSS_RTU_PRU0:
            case PRUICSS_RTU_PRU1:               
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + RTUx_DMEM_BASE_ADD;
            
                break;
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + PRUx_DMEM_BASE_ADD;
                
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru0DramBase) + TXPRUx_DMEM_BASE_ADD;
                
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
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + RTUx_DMEM_BASE_ADD;
            
                break;
            case PRUICSS_PRU0:
            case PRUICSS_PRU1:
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + PRUx_DMEM_BASE_ADD;
                
                break;
            case PRUICSS_TX_PRU0:
            case PRUICSS_TX_PRU1:
                p_sdfm->p_sdfm_interface = (void *)(((PRUICSS_HwAttrs *)(PruIcssXHandle->hwAttrs))->pru1DramBase) + TXPRUx_DMEM_BASE_ADD;
                
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
    p_sdfm->p_sdfm_interface->sdfm_ctrl.sdfm_pru_id = pruId;
    return (sdfm_handle)p_sdfm;
}

/*Configuration of iep & pwm time period */
void SDFM_configIepCount(sdfm_handle h_sdfm, uint32_t epwm_out_freq)
{
    /*; IEP0 default increment=1*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_iep_ptr.iep_inc_value = h_sdfm->iepInc;
    /*
     IEP0 CMP0 count to simulate EPWM (FOC loop) period:
     - IEP frequency = 300 MHz
     - IEP Default Increment = 1
     - Simulated EPWM frequency = 8e3
     CMP0 = 300e6/1/8e3 = 37500 = 0x927C
    */
    uint32_t cnt_epwm_prd = h_sdfm->iepClock/epwm_out_freq;
    h_sdfm->p_sdfm_interface->sdfm_cfg_iep_ptr.cnt_epwm_prd = cnt_epwm_prd;

}

/*ecap configuration for SD clock*/
void SDFM_configEcap(sdfm_handle h_sdfm, uint8_t ecap_divider)
{
    /* SD_PRD_CLOCKS divider = 15; SDFM_CLOCK = IEP freq./SD_PRD_CLOCKS; 300/15  => 20 MHz, SD_CLK_INV==0 => No clock inversion*/
    h_sdfm->p_sdfm_interface->sd_clk.sd_prd_clocks = ecap_divider;
    h_sdfm->p_sdfm_interface->sd_clk.sd_clk_inv = 0;
}

/*sdfm Hw osr configuration */
 void SDFM_setCompFilterOverSamplingRatio(sdfm_handle h_sdfm, uint8_t ch_id, uint16_t osr)
 {
    /*Over current OSR*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].osr = osr;
 }

/*sdfm high, low  threshold config */
void SDFM_setCompFilterThresholds(sdfm_handle h_sdfm, uint8_t ch_id, SDFM_ThresholdParms thresholdParms)
{
   /* SD Over current high threshold = (Over current OSR)^(Over current SINC ORDER) = 14^3 = 2744*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.high_threshold = thresholdParms.high_threshold;
    /* SD Over current low threshold */
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.low_threshold = thresholdParms.low_threshold;

}

/*sdfm smapling time configuation */
void SDFM_setSampleTriggerTime(sdfm_handle h_sdfm, float samp_trig_time)
{   /*convert sample time into IEP count*/
    /*samp time in us */
    int32_t count = (h_sdfm->iepClock /1000000)*((float)samp_trig_time);
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.first_samp_trig_time = count;



}
/*Second normal current sampling configuration*/
void SDFM_enableDoubleSampling(sdfm_handle h_sdfm, float samp_trig_time)
{
    /*Enable double normal current sampling*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.en_double_nc_sampling = 1;
    /*Second sample point*/
    int32_t count = (h_sdfm->iepClock /1000000)*((float)samp_trig_time);
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.second_samp_trig_time = count;
}

/*Disable Double update*/
void SDFM_disableDoubleSampling(sdfm_handle h_sdfm)
{
    /*Enable double normal current sampling*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.en_double_nc_sampling = 0;
}
/* Enable the channel specified by the channel number parameter*/
void SDFM_setEnableChannel(sdfm_handle h_sdfm, uint8_t channel_number)
{
    uint32_t temp;
    temp  = 1 << channel_number;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDFM_CFG_BF_SD_CH0_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].ch_id = channel_number;
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number<< SDFM_CFG_BF_SD_CH1_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].ch_id = channel_number;
    }
    else 
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDFM_CFG_BF_SD_CH2_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].ch_id = channel_number;
    }
}
/* set SDFM channel acc source */
void SDFM_configDataFilter(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t filter)
{
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].filter_type = filter;
}

/*set  clock source & clock inversion for SDFM channel*/
void SDFM_selectClockSource(sdfm_handle h_sdfm, uint8_t ch_id, SDFM_ClkSourceParms clkPrams)
{
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_clk_parms.clk_source = clkPrams.clk_source;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_clk_parms.clk_inv = clkPrams.clk_inv;
}

/* Enable the comparator feature for a specified filter/channel */
void SDFM_enableComparator(sdfm_handle h_sdfm, uint8_t ch)
{
    /*It is setting bits for enable Over current: 0th bit for Over current enable & from 1st bits to 3rd bits are for ch0 to ch2.*/
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.enable_comparator |= ((1 << (ch+1))|(1));
}

/* Disable the comparator feature for a specified filter/channel */
void SDFM_disableComparator(sdfm_handle h_sdfm, uint8_t ch)
{
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.enable_comparator &= (0xFFFF ^ (1<<ch));
}
/*GPIO configuration*/
void SDFM_configComparatorGpioPins(sdfm_handle h_sdfm, uint8_t ch,uint32_t gpio_base_addr, uint32_t pin_number)
{

    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) gpio_base_addr);
    uint32_t reg_index = GPIO_GET_REG_INDEX(pin_number);
    uint32_t reg_val = GPIO_GET_BIT_MASK(pin_number);
    uint32_t clr_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].CLR_DATA;
    uint32_t set_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].SET_DATA;

    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params.write_val = reg_val;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params.set_val_addr = set_data_addr;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params.clr_val_addr = clr_data_addr;
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
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.nc_prd_iep_cnt = count;
}
/*return firmware version */
uint32_t SDFM_getFirmwareVersion(sdfm_handle h_sdfm)
{
   return h_sdfm->p_sdfm_interface->firmwareVersion >> SDFM_FW_VERSION_BIT_SHIFT;
}
/*Enable free run NC */
void SDFM_enableContinuousNormalCurrent(sdfm_handle h_sdfm)
{
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.en_continuous_mode = 1;
}
/*FD block confiuration */
void SDFM_configFastDetect(sdfm_handle h_sdfm, uint8_t ch, uint8_t *fdParms)
{
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.enFastDetect |= 1<<ch;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].fd_window = fdParms[0];
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].fd_zero_max = fdParms[1];
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].fd_zero_min = fdParms[2];
      
    /*Configure one max to window size + 1 and one min to 0, so they never get set*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].fd_one_max = (fdParms[0] + 1) * 4 + 1;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].fd_one_min = 0;

}

/*return status of PWM trip vector status bit*/
int32_t SDFM_getFastDetectErrorStatus(sdfm_handle h_sdfm, uint8_t chNum) 
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_Handle pruIcssHandle = h_sdfm->gPruIcssHandle;
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
    retVal = PRUICSS_PWM_getPwmTripTriggerCauseVector(pruIcssHandle, pwmSet);
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
    PRUICSS_Handle pruIcssHandle = h_sdfm->gPruIcssHandle;
    
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
    retVal = PRUICSS_PWM_generatePwmTripReset(pruIcssHandle, pwmSet);
    if(retVal == SystemP_FAILURE)
    {
        return retVal;
    }

    /*clear trip reset status*/
    retVal = PRUICSS_PWM_clearPwmTripResetStatus(pruIcssHandle, pwmSet);

    return retVal;
}
/*Enable Load share mode*/
void SDFM_enableLoadShareMode(sdfm_handle h_sdfm, uint8_t sliceId)
{
    void *pruss_cfg = h_sdfm->pruss_cfg;
   
    uint32_t rgval;
    if(sliceId)
    {
       rgval = HW_RD_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_SDPRU1CLKDIV);
       rgval |= CSL_ICSSCFG_SDPRU1CLKDIV_PRU1_SD_SHARE_EN_MASK;
       HW_WR_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_SDPRU1CLKDIV, rgval);
    }
    else
    {
        rgval = HW_RD_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_SDPRU0CLKDIV);
        rgval |= CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_SHARE_EN_MASK;
        HW_WR_REG32((uint8_t *)pruss_cfg + CSL_ICSSCFG_SDPRU0CLKDIV, rgval);
    }

}
/*Measure Phase delay*/
void SDFM_measureClockPhaseDelay(sdfm_handle h_sdfm, uint16_t clkEdg)
{
    /*enable phase delay measurement*/
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.en_phase_delay = 1;
    /*waiting till measurment done */
    uint8_t ack = h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK;
    while(ack)
    {
       ack = h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.en_phase_delay & SDFM_PHASE_DELAY_ACK_BIT_MASK ;
    }


   uint16_t nEdge = h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.clock_edge;
   float temp = h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.clock_phase_delay;
   /*avg*/
    temp = temp/SDFM_PHASE_DELAY_CAL_LOOP_SIZE;
   /*check data reading edge(clk polarity) & nearest edge */
   if(nEdge == clkEdg)
   {
      /*PRU cycles for half SD clock period*/
      uint32_t pruCycles = ceil(((float)h_sdfm->pruCoreClk)/(2*h_sdfm->sdfmClock));
      h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.clock_phase_delay = pruCycles - temp;
   }
   else
   {
      /*PRU cycles for one SD clock period*/
      uint32_t pruCycles = ceil((float)(h_sdfm->pruCoreClk/(h_sdfm->sdfmClock)));
      h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.clock_phase_delay = pruCycles - temp;
   }

}
float SDFM_getClockPhaseDelay(sdfm_handle h_sdfm)
{
    /*conversion from PRU cycle to ns */
    float phaseDelay =  ((float)h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.clock_phase_delay * 1000000000)/h_sdfm->pruCoreClk;
    return phaseDelay;
}
uint8_t SDFM_getHighThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.highThStatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.highThStatus;
    }
    else 
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.highThStatus;
    }
     
}
uint8_t SDFM_getLowThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.lowThStatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.lowThStatus;
    }
    else 
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.lowThStatus;
    }
}

int32_t SDFM_clearOverCurrentError(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint8_t pwmSet;
    int32_t                 retVal = SystemP_SUCCESS;
    PRUICSS_Handle pruIcssHandle = h_sdfm->gPruIcssHandle;
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
    retVal = PRUICSS_PWM_clearPwmOverCurrentErrorTrip(pruIcssHandle, pwmSet);
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
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
    else 
    {
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossEn = 1; 
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossTh = zcThr; 
    }
   
}
uint8_t SDFM_getZeroCrossThresholdStatus(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossThstatus; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossThstatus; 
    }
    else 
    {
        return h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossThstatus;  
    }

}
void SDFM_disableZeroCrossDetection(sdfm_handle h_sdfm, uint8_t chNum)
{
    uint32_t temp;
    temp  = 1 << chNum;
    if(temp & SDFM_CH_MASK_FOR_CH0_CH3_CH6)
    {
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[0].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
    else if(temp & SDFM_CH_MASK_FOR_CH1_CH4_CH7)
    {
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[1].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
    else 
    {
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[2].sdfm_threshold_parms.zeroCrossEn = 0; 
    }
   
}
/* SDFM global enable */
void SDFM_enable(sdfm_handle h_sdfm)
{
    uint8_t sdfm_en_ack;

    /*Enable SDFM */
    h_sdfm->p_sdfm_interface->sdfm_ctrl.sdfm_en = 1;

    /* wait for ACK */
    do {
        sdfm_en_ack = h_sdfm->p_sdfm_interface->sdfm_ctrl.sdfm_en_ack;
    } while (sdfm_en_ack != BF_SDFM_EN_ENABLE);


}


