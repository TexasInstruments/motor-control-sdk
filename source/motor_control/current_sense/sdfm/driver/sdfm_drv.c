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
#include <motor_control/current_sense/sdfm/include/sdfm_drv.h>
#include <motor_control/current_sense/sdfm/include/sddf_api.h>
#include <motor_control/current_sense/sdfm/firmware/icssg_sddf.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>
#include <drivers/soc.h>
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>

//*****************************************************************************
//
// Defines for the API.
//
//*****************************************************************************
//! Macro to get the low threshold
#define SDFM_GET_LOW_THRESHOLD(C)    ((uint16_t)(C))

//! Macro to get the high threshold
#define SDFM_GET_HIGH_THRESHOLD(C)   ((uint16_t)((uint32_t)(C) >> 16U))

//! Macro to get the high threshold 1 & 2 to be passed as lowThreshold
//! parameter to SDFM_setCompFilterLowThreshold().
#define SDFM_GET_LOW_THRESHOLD_BOTH(C1, C2)                                   \
                        ((((uint32_t)(SDFM_GET_LOW_THRESHOLD(C2))) << 16U) |  \
                         ((uint32_t)(SDFM_GET_LOW_THRESHOLD(C1))))

//! Macro to get the high threshold 1 & 2 to be passed as highThreshold
//! parameter to SDFM_setCompFilterHighThreshold().
#define SDFM_GET_HIGH_THRESHOLD_BOTH(C1, C2)                                  \
                        ((((uint32_t)(SDFM_GET_HIGH_THRESHOLD(C2))) << 16U) | \
                         ((uint32_t)(SDFM_GET_HIGH_THRESHOLD(C1))))

//! Macro to convert comparator over sampling ratio to acceptable bit location
#define SDFM_SET_OSR(X)    (((X) - 1) << 8U)

//! Macro to convert the data shift bit values to acceptable bit location
#define SDFM_SHIFT_VALUE(X)    ((X) << 2U)

//! Macro to combine high threshold and low threshold values
#define SDFM_THRESHOLD(H, L)    ((((uint32_t)(H)) << 16U) | (L))

//! Macro to set the FIFO level to acceptable bit location
#define SDFM_SET_FIFO_LEVEL(X)    ((X) << 7U)

//! Macro to set and enable the zero cross threshold value.
#define SDFM_SET_ZERO_CROSS_THRESH_VALUE(X)    (0x8000 | (X))

//! Macros to enable or disable filter.
#define SDFM_FILTER_DISABLE    (0x0U)
#define SDFM_FILTER_ENABLE    (0x2U)

//*****************************************************************************
//
//! Values that can be returned from SDFM_getThresholdStatus()
//
//*****************************************************************************
#define SDFM_OUTPUT_WITHIN_THRESHOLD  (0)  //!< SDFM output is within threshold
#define SDFM_OUTPUT_ABOVE_THRESHOLD  (1)  //!< SDFM output is above high threshold
#define SDFM_OUTPUT_BELOW_THRESHOLD  (2)  //!< SDFM output is below low threshold

//! Filter output is in 16 bits 2's complement format.
#define SDFM_DATA_FORMAT_16_BIT    (0)
//! Filter output is in 32 bits 2's complement format.
#define SDFM_DATA_FORMAT_32_BIT    (1)

//! Mask for Interrupt is generated if Modulator fails.
//!
#define SDFM_MODULATOR_FAILURE_INTERRUPT_MASK    ( 0 )
//!  Mask for Interrupt on Comparator low-level threshold.
//!
#define SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT_MASK  ( 1 )
//!  Mask for Interrupt on Comparator high-level threshold.
//!
#define SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT_MASK ( 2 )
//!  Mask for Interrupt on Acknowledge flag
//!
#define SDFM_DATA_FILTER_ACKNOWLEDGE_INTERRUPT_MASK ( 3 )

/* Internal structure for managing each PRU SD */
//static sdfm g_sdfm[NUM_PRU] = {
SDFM g_sdfm[NUM_PRU] = {
    {PRU_ID_0,0,0,0, NULL},
    {PRU_ID_1,0,0,0, NULL},
};
// static sdfm g_sdfm[NUM_PRU];

/* Initialize SDFM instance */
sdfm_handle SDFM_init(uint8_t pru_id)
{
    SDFM *p_sdfm;
    uint8_t ctrl;
    uint8_t stat;
    uint8_t pru_id_ack;

    if (pru_id == PRU_ID_0) {
        /* Initialize PRU 0 SD */

        p_sdfm = &g_sdfm[pru_id];

        /* Initialize SDFM control address */
        p_sdfm->p_sdfm_interface = (SDFM_Interface *)(PRU_ICSSG_DRAM0_SLV_RAM + 0x0);

        /* Set FW PRU ID */
        ctrl = p_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl;
        ctrl &= ~SDDF_CTRL_BF_PRU_ID_MASK;
        ctrl |= pru_id << SDDF_CTRL_BF_PRU_ID_SHIFT;
        p_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl = ctrl;

        /* Wait for FW PRU ID ACK */
        do {
            stat = p_sdfm->p_sdfm_interface->sdfm_ctrl.stat;
            pru_id_ack = (stat & SDDF_STAT_BF_PRU_ID_ACK_MASK) >> SDDF_STAT_BF_PRU_ID_ACK_SHIFT;
        } while (pru_id_ack != pru_id);
    }
    else if (pru_id == PRU_ID_1) {
        /* Initialize PRU 1 SD */

        p_sdfm = &g_sdfm[pru_id];

        /* Initialize SDFM control address */
        p_sdfm->p_sdfm_interface = (SDFM_Interface *)(PRU_ICSSG_DRAM1_SLV_RAM + 0x0);

        /* Set FW PRU ID */
        ctrl = p_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl;
        ctrl &= ~SDDF_CTRL_BF_PRU_ID_MASK;
        ctrl |= pru_id << SDDF_CTRL_BF_PRU_ID_SHIFT;
        p_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl = ctrl;

        /* Wait for FW PRU ID ACK */
        do {
            stat = p_sdfm->p_sdfm_interface->sdfm_ctrl.stat;
            pru_id_ack = (stat & SDDF_STAT_BF_PRU_ID_ACK_MASK) >> SDDF_STAT_BF_PRU_ID_ACK_SHIFT;
        } while (pru_id_ack != pru_id);
    }
    else {
        p_sdfm = NULL;
    }

    return (sdfm_handle)p_sdfm;
}

/*Configuration of iep & pwm time period */
void SDFM_configIepCount(sdfm_handle h_sdfm, uint32_t epwm_out_freq)
{
    /*; IEP0 default increment=1*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_iep_ptr.iep_inc_value = h_sdfm->iep_inc;
    /*
     IEP0 CMP0 count to simulate EPWM (FOC loop) period:
     - IEP frequency = 300 MHz
     - IEP Default Increment = 1
     - Simulated EPWM frequency = 8e3
     CMP0 = 300e6/1/8e3 = 37500 = 0x927C
    */
    uint32_t cnt_epwm_prd = h_sdfm->iep_clock/epwm_out_freq;
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
 void SDFM_setAccOverSamplingRatio(sdfm_handle h_sdfm, uint8_t ch_id, uint8_t osr)
 {
    /*SD Hw OSR*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].osr = osr;
 }

/*sdfm high, low  threshold config */
void SDFM_setCompFilterThresholds(sdfm_handle h_sdfm, uint8_t ch_id, SDFM_ThresholdParms thresholdParms)
{
   /* SD OC high threshold = (OC OSR)^(OC SINC ORDER) = 14^3 = 2744*/
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.high_threshold = thresholdParms.high_threshold;
    /* SD OC low threshold */
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch_id].sdfm_threshold_parms.low_threshold = thresholdParms.low_threshold;

}

/*sdfm smapling time configuation */
void SDFM_setSampleReadingTime(sdfm_handle h_sdfm, float trig_samp_time)
{   /*convert sample time into IEP count*/
    /*samp time in us */
    int32_t count = (h_sdfm->iep_clock /1000000)*((float)trig_samp_time);
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.trig_samp_time = count;
}
/* Enable the channel specified by the channel number parameter*/
void SDFM_setEnableChannel(sdfm_handle h_sdfm, uint8_t channel_number)
{

    if(channel_number == 0)
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDDF_CFG_BF_SD_CH0_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[channel_number].ch_id = channel_number;
    }
    else if(channel_number == 1)
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number<< SDDF_CFG_BF_SD_CH1_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[channel_number].ch_id = channel_number;
    }
    else
    {
        h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.sdfm_ch_id |= (channel_number << SDDF_CFG_BF_SD_CH2_ID_SHIFT);
        h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[channel_number].ch_id = channel_number;
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
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.enable_comparator |= (1 << ch);
}

/* Disable the comparator feature for a specified filter/channel */
void SDFM_disableComparator(sdfm_handle h_sdfm, uint8_t ch)
{
    h_sdfm->p_sdfm_interface->sdfm_ch_ctrl.enable_comparator &= (0xFFFF ^ (1<<ch));
}
/*GPIO configuration*/
void SDFM_configComparatorGpioPins(sdfm_handle h_sdfm, uint8_t ch,uint32_t gpio_base_addr, uint32_t pin_number, uint32_t threshold_type)
{

    volatile CSL_GpioRegs*  hGpio = (volatile CSL_GpioRegs*)((uintptr_t) gpio_base_addr);
    uint32_t reg_index = GPIO_GET_REG_INDEX(pin_number);
    uint32_t reg_val = GPIO_GET_BIT_MASK(pin_number);
    uint32_t clr_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].CLR_DATA;
    uint32_t set_data_addr = (uint32_t)&hGpio->BANK_REGISTERS[reg_index].SET_DATA;

    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params[threshold_type].write_val = reg_val;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params[threshold_type].set_val_addr = set_data_addr;
    h_sdfm->p_sdfm_interface->sdfm_cfg_ptr[ch].sdfm_gpio_params[threshold_type].clr_val_addr = clr_data_addr;
}

/* Get current (or latest) sample for the specified channel */
uint32_t SDFM_getFilterData(uint8_t ch)
{
    SDFM *p_sdfm = NULL;
    p_sdfm->p_sdfm_interface = (SDFM_Interface *)(PRU_ICSSG_DRAM0_SLV_RAM + 0x0);
    return p_sdfm->p_sdfm_interface->curr_out_samp_buf[ch];
}
/*Configure Over current OSR*/
void SDFM_setCompFilterOverSamplingRatio(sdfm_handle h_sdfm, uint16_t osr)
{
    /*; IEP0 counts in OC sampling period:
    ;   - IEP0 count =
    ;       (NC OSR * SD clock period)*(IEP0 frequency * IEP default count) =
    ;       (NC OSR * IEP0 frequency * IEP default count)/(SD clock frequency) =
    ;       64*300e6*1/20e6 = 960*/
   //IEP freq. 300MHz //SDFM clock 20MHz
    // IEP Count = (OSR *(1/SD clock))*(IEP freq.)
    uint16_t count;
    uint32_t iep_freq = h_sdfm->iep_clock;
    uint32_t sd_clock = h_sdfm->sdfm_clock;
    count = (osr*(iep_freq/sd_clock));
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.oc_prd_iep_cnt = count;

}

/*Configure normal current OSR*/
void SDFM_setFilterOverSamplingRatio(sdfm_handle h_sdfm, uint16_t nc_osr, uint16_t oc_osr)
{
    /*OC sample count for NC sampling */
    uint16_t count;
    count  = (nc_osr/oc_osr);
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.sample_count = count;

    /*IEP0 counts in NC sampling period*/
    uint32_t iep_freq = h_sdfm->iep_clock;
    uint32_t sd_clock = h_sdfm->sdfm_clock;
    count = (nc_osr*(iep_freq/sd_clock));
    h_sdfm->p_sdfm_interface->sdfm_cfg_trigger.nc_prd_iep_cnt = count;
}
/* SDFM global enable */
void SDFM_enable(sdfm_handle h_sdfm)
{
    uint8_t sdfm_ctrl;
    uint8_t sdfm_stat;
    uint8_t sdfm_en_ack;

    sdfm_ctrl = h_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl;
    sdfm_ctrl &= ~SDDF_CTRL_BF_SDDF_EN_MASK;
    sdfm_ctrl |= BF_SDDF_EN_ENABLE << SDDF_CTRL_BF_SDDF_EN_SHIFT;
    h_sdfm->p_sdfm_interface->sdfm_ctrl.ctrl = sdfm_ctrl;

    /* wait for ACK */
    do {
        sdfm_stat = h_sdfm->p_sdfm_interface->sdfm_ctrl.stat;
        sdfm_en_ack = (sdfm_stat & SDDF_STAT_BF_SDDF_EN_ACK_MASK) >> SDDF_STAT_BF_SDDF_EN_ACK_SHIFT;
    } while (sdfm_en_ack != BF_SDDF_EN_ENABLE);


}


