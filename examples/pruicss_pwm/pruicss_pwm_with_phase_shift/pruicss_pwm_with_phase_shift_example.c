/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss.h>
#include <drivers/pinmux.h>
#include <stdlib.h>
#include <board/ioexp/ioexp_tca6424.h>
#include <pru0_g0_load_bin.h>

/* Frequency of PWM output signal in Hz - 16KHZ is selected */
#define PWM_OUTPUT_FREQ                                (16000)
/* Phase shift between even and odd axis in nano seconds*/
#define PHASE_SHIFT_BETWEEN_AXES               (6944)
/* IEP increment value (3ns -> 3)*/
#define IEP_INCREMENT_VALUE                            (3)
/* Number axis controlled from each slice of ICSS*/
#define NUMBER_OF_AXES                                 (3)
/* PRUICSS PWM PARMS DMEM OFFSET*/
/*NOTE: Make sure to change PWM_PARAMS_DMEM0_OFFSET define in PRU firmware when below
configuration is shared*/
#define PRUICSS_PWM_PARAMS_DMEM_OFFSET                 (0x600)
/* PWM dead band value of A or Pos signal*/
#define PWM_DEAD_BAND_0     0
/* PWM dead band value of B or Neg signal*/
#define PWM_DEAD_BAND_3000  3000
/*state Config of PWM signals*/
#define STATE_TOGG 0
#define STATE_LOW  1
#define STATE_HIGH 2

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

static TCA6424_Config  gTCA6424_Config;

typedef struct{
    /*Compare up values of PWM signals*/
    uint32_t Pwm0CmpUpValueIncrement;
    uint32_t Pwm1CmpUpValueIncrement;
    uint32_t Pwm2CmpUpValueIncrement;
    uint32_t Pwm3CmpUpValueIncrement;
    uint32_t Pwm4CmpUpValueIncrement;
    uint32_t Pwm5CmpUpValueIncrement;
    /*Compare down values of PWM signals*/
    uint32_t Pwm0CmpDowmvalueIncrement;
    uint32_t Pwm1CmpDowmvalueIncrement;
    uint32_t Pwm2CmpDowmvalueIncrement;
    uint32_t Pwm3CmpDowmvalueIncrement;
    uint32_t Pwm4CmpDowmvalueIncrement;
    uint32_t Pwm5CmpDowmvalueIncrement;
}pruIcssPwmCmpValues;

typedef struct{
    /*Dead band values of PWM signals*/
    uint32_t pwm0DeadBand;
    uint32_t pwm1DeadBand;
    uint32_t pwm2DeadBand;
    uint32_t pwm3DeadBand;
    uint32_t pwm4DeadBand;
    uint32_t pwm5DeadBand;
    /*Duty cycle values of PWM signals*/
    float pwm0DutyCycle;
    float pwm1DutyCycle;
    float pwm2DutyCycle;
    float pwm3DutyCycle;
    float pwm4DutyCycle;
    float pwm5DutyCycle;
}pruIcssPwmDutyCycleDeadBandValues;


/**
 * @brief pruIcssPwmCmpValues
 *
 * Structure to hold the increment values of the PWM compare values of
 * each PWM axis and the scheduler initial and increment values placed in PRUICSS DMEM.
 */
typedef struct{
    /*axis 0 to 2 compare up and down values*/
    pruIcssPwmCmpValues    axisCmpValues[NUMBER_OF_AXES];
    /*Below parameter configures phase shift between axis 0 and axis 1 (axis 0 rise edge time stamp - axis 1 rise edge time stamp) in positive direction*/
    uint32_t axis0_1_increment_value;
    /*Below parameter configures phase shift between axis 1 and axis 2 (axis 1 rise edge time stamp - axis 2 rise edge time stamp) in positive direction*/
    uint32_t axis1_2_increment_value;
    /*Below parameter time gap between axis 2 rise edge and axis 1 fall edge update*/
    uint32_t axis2_0_increment_value;
    /*Scheduler initial value*/
    uint32_t SchedulerIntialValue;
    /*Stop axis_X_pwm_x_signals*/
    uint32_t stop_axis_x_pwm_y_signals;
    /*Off axis_x_pwm_x_signals*/
    /*Below configuration is used to */
    uint8_t off_axis_0_2_pwm_signals;
    /*ON axis_x_pwm_x_signals*/
    uint8_t exit_pwm_signals_from_off;
}pruIcssPwmParamsInDmem;

/*Compare values of 0 to 2 axis stored in PRUICSS DMEM used by PRU updated based on duty cycle configured*/
pruIcssPwmParamsInDmem *gPruIcssPwmParams;
/*Duty values of 0 to 2 axis*/
pruIcssPwmDutyCycleDeadBandValues gpruIcssPwmDutyCycleDeadBandValues[NUMBER_OF_AXES];


/**
 * updatePruIcssPwmParamsInDmem
 *
 * This function is used to update the compare values of the PWM signals in PRUICSS DMEM.
 *
 * The function first calculates the PWM period in IEP clock cycles.
 *
 * Then it calculates the increment and decrement values of the compare values.
 * The increment and decrement values are calculated based on the duty cycle and dead band of each PWM channel.
 *
 * The function sets the initial value of the scheduler used to update the compare values.
 *
 * The function sets the increment value of the schedular to use after configuring compare values of each axis.
 *
 * The function initializes the duty cycles and dead bands of all PWM channels.
 *
 * The function is called by the main function of the example.
 */
void updatePruIcssPwmParamsInDmem(float dutycycle)
{
    uint32_t pruIcssIepClkFrequency = CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ;
    uint32_t pruIcssPwmFrequency = PWM_OUTPUT_FREQ;
    uint32_t axesPhaseShiftValue = PHASE_SHIFT_BETWEEN_AXES;
    uint8_t iepIncrementValue = IEP_INCREMENT_VALUE;
    uint32_t pruIcssPwmPeriodby2 = (((float)(pruIcssIepClkFrequency * iepIncrementValue) / (pruIcssPwmFrequency))/2);
    uint8_t i;

    /*Below parameter configures initial value of scheduler used to update compare values of axis 0*/
    gPruIcssPwmParams->SchedulerIntialValue = pruIcssPwmPeriodby2;
    /*Below parameter is used to update scheduler (scheduler + increment value) used to update compare values of axis 1*/
    gPruIcssPwmParams->axis0_1_increment_value  = ((float)(axesPhaseShiftValue * iepIncrementValue) / (CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
    /*Below parameter is used to update scheduler (scheduler value after configuring axis 1 + increment value) used to update compare values of axis 2*/
    gPruIcssPwmParams->axis1_2_increment_value  = ((float)(axesPhaseShiftValue * iepIncrementValue) / (CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
    /*Below parameter is used to update scheduler (scheduler value after configuring axis 2 + increment value) used to update compare values of axis 0*/
    gPruIcssPwmParams->axis2_0_increment_value  = (gPruIcssPwmParams->SchedulerIntialValue) - (gPruIcssPwmParams->axis0_1_increment_value + gPruIcssPwmParams->axis1_2_increment_value );

    /* NOTE : Compare Increment and decrement values are updated once every PWM period in PRU Firmware*/
    for(i = 0; i < NUMBER_OF_AXES; i++)
    {
        /*Initialize duty cycles of all PWM channels*/

        gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle = dutycycle;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle = dutycycle;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle = dutycycle;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle = dutycycle;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle = dutycycle;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle = dutycycle;
        /*Initialize dead band of all PWM channels in nanoseconds*/
        /*Note:  below parameter used by both cmpUpIncrement and cmpDowmIncremenat, so it is used both fall edge and rise edge delay*/
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DeadBand = PWM_DEAD_BAND_0;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DeadBand = PWM_DEAD_BAND_3000;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DeadBand = PWM_DEAD_BAND_0;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DeadBand = PWM_DEAD_BAND_3000;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DeadBand = PWM_DEAD_BAND_0;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DeadBand = PWM_DEAD_BAND_3000;
        /*update all the axis PWM signals compare up values as per duty cycle and PWM period
         *compare Up increment value = (PWM period/2) - (Duty_cycle*(PWM period/2)) + DeadBand
         *compare Up value = Scheduler value +  compare up increment value (this add logic is implemented in PRU firmware)*/
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm0CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm1CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm2CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm3CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm4CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm5CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        /*update all the axis PWM signals compare down values
         *compare Down increment value = (Duty_cycle*(PWM period/2)) - DeadBand
         *compare Down value = Scheduler value + compare down increment value (this add logic is implemented in PRU firmware)*/
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm0CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm1CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm2CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm3CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm4CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmParams->axisCmpValues[i]).Pwm5CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
    }
}

#if defined(am243x_evm)

/*
 * Function: i2c_io_expander
 * Description: This function is used to configure the TCA6424 IO expander
 * to enable the GPIOs used by the PRU.
 *
 * Parameters:
 * args - not used
 *
 * Return:
 * None
 */
static void i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    TCA6424_Params      tca6424Params;

    /* Initialize the TCA6424 params */
    TCA6424_Params_init(&tca6424Params);

    /* Open the TCA6424 device */
    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    /* If the open is successful, set the IO expander pin high and
     * configure it as output
     */
    if(status == SystemP_SUCCESS)
    {
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        uint32_t ioIndex = 0x0a;
        status = TCA6424_setOutput(
                    &gTCA6424_Config,
                    ioIndex,
                    TCA6424_OUT_STATE_HIGH);

        /* Configure the pin as output */
        status += TCA6424_config(
                    &gTCA6424_Config,
                    ioIndex,
                    TCA6424_MODE_OUTPUT);
    }

    /* Close the TCA6424 device */
    TCA6424_close(&gTCA6424_Config);
}
#endif

/**
 * Change the initial state of PRGx_PWMy_Bz signals to high and PRGx_PWMy_Az signals to low.
 * Configure trip state to low.
 * @param args Unused.
 */
void configureIntialAndTripStates(void *args)
{
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0), CSL_ICSSCFG_PWM0_0_PWM0_0_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1), CSL_ICSSCFG_PWM0_1_PWM0_1_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2), CSL_ICSSCFG_PWM0_2_PWM0_2_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0), CSL_ICSSCFG_PWM1_0_PWM1_0_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1), CSL_ICSSCFG_PWM1_1_PWM1_1_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2), CSL_ICSSCFG_PWM1_2_PWM1_2_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0), CSL_ICSSCFG_PWM2_0_PWM2_0_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1), CSL_ICSSCFG_PWM2_1_PWM2_1_POS_INIT, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2), CSL_ICSSCFG_PWM2_2_PWM2_2_POS_INIT, STATE_LOW);

    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0), CSL_ICSSCFG_PWM0_0_PWM0_0_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1), CSL_ICSSCFG_PWM0_1_PWM0_1_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2), CSL_ICSSCFG_PWM0_2_PWM0_2_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0), CSL_ICSSCFG_PWM1_0_PWM1_0_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1), CSL_ICSSCFG_PWM1_1_PWM1_1_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2), CSL_ICSSCFG_PWM1_2_PWM1_2_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0), CSL_ICSSCFG_PWM2_0_PWM2_0_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1), CSL_ICSSCFG_PWM2_1_PWM2_1_NEG_INIT, STATE_HIGH);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2), CSL_ICSSCFG_PWM2_2_PWM2_2_NEG_INIT, STATE_HIGH);
    /*set trip state to low
     * NOTE : PWM signals are changed to trip by firmware, when off_axis_0_2_pwm_signals is set to 1*/

    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0), CSL_ICSSCFG_PWM0_0_PWM0_0_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1), CSL_ICSSCFG_PWM0_1_PWM0_1_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2), CSL_ICSSCFG_PWM0_2_PWM0_2_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0), CSL_ICSSCFG_PWM1_0_PWM1_0_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1), CSL_ICSSCFG_PWM1_1_PWM1_1_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2), CSL_ICSSCFG_PWM1_2_PWM1_2_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0), CSL_ICSSCFG_PWM2_0_PWM2_0_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1), CSL_ICSSCFG_PWM2_1_PWM2_1_POS_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2), CSL_ICSSCFG_PWM2_2_PWM2_2_POS_TRIP, STATE_LOW);

    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_0), CSL_ICSSCFG_PWM0_0_PWM0_0_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_1), CSL_ICSSCFG_PWM0_1_PWM0_1_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM0_2), CSL_ICSSCFG_PWM0_2_PWM0_2_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_0), CSL_ICSSCFG_PWM1_0_PWM1_0_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_1), CSL_ICSSCFG_PWM1_1_PWM1_1_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM1_2), CSL_ICSSCFG_PWM1_2_PWM1_2_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_0), CSL_ICSSCFG_PWM2_0_PWM2_0_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_1), CSL_ICSSCFG_PWM2_1_PWM2_1_NEG_TRIP, STATE_LOW);
    HW_WR_FIELD32((gPruIcss0Handle->hwAttrs->cfgRegBase + CSL_ICSSCFG_PWM2_2), CSL_ICSSCFG_PWM2_2_PWM2_2_NEG_TRIP, STATE_LOW);

}

/**
 * pru_icss_with_phase_shift_main - The main function for the PRUICSS PWM
 *                                    with phase shift example
 *
 * @param args - not used
 *
 * This function is the entry point for the PRUICSS PWM with phase shift
 * example. It performs the following steps:
 * - Opens the peripheral drivers added in sysconfig
 * - Opens the board drivers
 * - Opens the PRUICSS driver
 * - Configures the PRUICSS PWM parameters in DMEM
 * - Loads the PRU0 G0 firmware
 * - Enters an infinite loop
 * - Before exiting the function, closes the board drivers and PRUICSS driver
 */
void pru_icss_with_phase_shift_main(void *args)
{
    int32_t status;

    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Get the pointer to the PRUICSS PWM parameters in DMEM */
    gPruIcssPwmParams = (pruIcssPwmParamsInDmem *)(gPruIcss0Handle->hwAttrs->pru0DramBase + PRUICSS_PWM_PARAMS_DMEM_OFFSET);

    /* Call the board specific function to configure the IO expander */
    #if defined(am243x_evm)
    i2c_io_expander(NULL);
    #endif

    /* Initialize the PRUICSS DMEM memory */
    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    DebugP_assert(status != 0);

    /* Update the PRUICSS PWM parameters in DMEM */
    updatePruIcssPwmParamsInDmem(12.5);

    /* Now change PRGx_PWMy_Bz signal initial state to high to make it complementary to PRGx_PWMy_Az*/
    configureIntialAndTripStates(NULL);

    /*exit all pwm signals from off state*/
    gPruIcssPwmParams->exit_pwm_signals_from_off = 0x0;

    /*After firmware initialization is done and PWM signals are in off state below parameter is set to zero by firmware*/
    gPruIcssPwmParams->off_axis_0_2_pwm_signals  = 0x1;

    /* Load the PRU0 G0 firmware */
    status = PRUICSS_loadFirmware(gPruIcss0Handle, PRUICSS_PRU0, PRU0_G0_Firmware_0, sizeof(PRU0_G0_Firmware_0));
    DebugP_assert(SystemP_SUCCESS == status);

    ClockP_usleep(500);
    DebugP_log("PRUICSS PWM firmware version \t: %x.%x.%x (%s)\n\n", ((PRU0_G0_Firmware_0[0])>> 24) & 0x7F,
                ((PRU0_G0_Firmware_0[0]) >> 16) & 0xFF, (PRU0_G0_Firmware_0[0]) & 0xFFFF, (PRU0_G0_Firmware_0[0]) & (1 << 31) ? "internal" : "release");

    /*PWM SET 0 -> byte_0[0:5]
        *byte_0 -> bit 0 -> pwm0(PWM0_0_POS)
        *byte_0 -> bit 1 -> pwm1(PWM0_0_NEG)
        *byte_0 -> bit 2 -> pwm2
        *byte_0 -> bit 3 -> pwm3
        *byte_0 -> bit 4 -> pwm4
        *byte_0 -> bit 5 -> pwm5
        */
        /*PWM SET 1 -> byte_1[0:5]*/
        /*PWM SET 2 -> byte_2[0:5]*/
    gPruIcssPwmParams->stop_axis_x_pwm_y_signals = 0x00000000;

    /* Enable IEP0*/
    status = PRUICSS_controlIepCounter(gPruIcss0Handle, 0, 1);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enter an infinite loop */
    while (1)
    {
        uint32_t timeoutCount = 0;
        const uint32_t timeoutThreshold = UINT32_MAX; /* timeout threshold */
        /* Loop until the PWM signals are in off state or timeout occurs */
        while (gPruIcssPwmParams->off_axis_0_2_pwm_signals == 1)
        {
            /* Increment the timeout counter */
            timeoutCount++;

            /* Check for timeout */
            if (timeoutCount >= timeoutThreshold)
            {
                /* Fatal error - assert will halt execution in debug builds */
                DebugP_logError("Timeout waiting for PRU firmware to change PWM signals to Off state\n");
                DebugP_assert(0);
            }
        }
        /* Now PWM signals are in active state*/
        /* Update the PRUICSS PWM parameters in DMEM */
        updatePruIcssPwmParamsInDmem(12.5);
        /*exit all pwm signals from off state*/
        gPruIcssPwmParams->exit_pwm_signals_from_off = 1;
        /* Generate PWM signals for 500usecs with 12.5% duty cycle*/
        ClockP_usleep(500);
        /* Update the PRUICSS PWM parameters in DMEM */
        updatePruIcssPwmParamsInDmem(25);
        /* Generate PWM signals for 500usecs with 25% duty cycle*/
        ClockP_usleep(500);
        /* Update the PRUICSS PWM parameters in DMEM */
        updatePruIcssPwmParamsInDmem(50);
        /* Generate PWM signals for 500usecs with 50% duty cycle*/
        ClockP_usleep(500);
        /* Update the PRUICSS PWM parameters in DMEM */
        updatePruIcssPwmParamsInDmem(75);
        /* Generate PWM signals for 500usecs with 75% duty cycle*/
        ClockP_usleep(500);
        /* Fix PRGx_PWMy_Az and PRGx_PWMy_Bz to ~(initial_state), this can be altered to
         * to Fix PRGx_PWMy_Az to and PRGx_PWMy_Bz to initial_state by changing
         * FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT to 0 in PRU firmware
         * and rebuilding PRU project
         */
        /* when FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT is set to 1, generating or stopping of
         * PWM signal is decided while configuring second compare event and AXIS_X_SKIP_CMP_UP_AND_DOWN_INIT_VAL
         * can be used to generate or stop PWM signal initially
         *
         * when FIX_PRGX_PWMY_AZ_PRGX_PWMY_BZ_TO_INITIAL_STATE_COMPLIMENT is set to 0, generating or stopping of
         * PWM signal is decided while configuring first compare event and AXIS_X_SKIP_CMP_UP_AND_DOWN_INIT_VAL
         * parameter configuration will be redundant
        */
        gPruIcssPwmParams->stop_axis_x_pwm_y_signals = 0x003F3F3F;
        /* Stop for 500usec*/
        ClockP_usleep(500);
        /* Regenerate PWM signals*/
        gPruIcssPwmParams->stop_axis_x_pwm_y_signals = 0x00000000;
        /* Regenerate PWM signals for 1sec with duty of 75% for 500usec*/
        ClockP_usleep(500);
        /* Below steps are handled by PRU when off_axis_0_2_pwm_signals is set to 1
         * Configure initial state to low for PRGx_PWMy_Az and PRGx_PWMy_Bz
         * Move all PWM signals to initial state
         * */
        /*set exit all pwm signals from off state to zero and off all pwm signals*/
        gPruIcssPwmParams->exit_pwm_signals_from_off = 0;
        gPruIcssPwmParams->off_axis_0_2_pwm_signals = 0x1;
        /* off PWM signals for 500usec*/
        ClockP_usleep(500);
    }

    /* Close the board drivers */
    Board_driversClose();

    /* Close the PRUICSS driver */
    Drivers_close();
}
