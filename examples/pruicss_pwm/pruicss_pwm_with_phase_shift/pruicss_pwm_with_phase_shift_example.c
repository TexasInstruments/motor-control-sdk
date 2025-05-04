/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#define PRUICSS_PWM_PARAMS_DMEM_OFFSET                 (0x200)
/* PWM duty cycle values of all PWM channels*/
#define PWM_DUTY_CYCLE   25
/* PWM dead band value of A or Pos signal*/
#define PWM_DEAD_BAND_0     0
/* PWM dead band value of B or Neg signal*/
#define PWM_DEAD_BAND_3000  3000

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

static TCA6424_Config  gTCA6424_Config;

typedef struct{
    /*Compare up values of PWM signals*/
    uint16_t Pwm0CmpUpValueIncrement;
    uint16_t Pwm1CmpUpValueIncrement;
    uint16_t Pwm2CmpUpValueIncrement;
    uint16_t Pwm3CmpUpValueIncrement;
    uint16_t Pwm4CmpUpValueIncrement;
    uint16_t Pwm5CmpUpValueIncrement;
    /*Compare down values of PWM signals*/
    uint16_t Pwm0CmpDowmvalueIncrement;
    uint16_t Pwm1CmpDowmvalueIncrement;
    uint16_t Pwm2CmpDowmvalueIncrement;
    uint16_t Pwm3CmpDowmvalueIncrement;
    uint16_t Pwm4CmpDowmvalueIncrement;
    uint16_t Pwm5CmpDowmvalueIncrement;
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
    uint8_t pwm0DutyCycle;
    uint8_t pwm1DutyCycle;
    uint8_t pwm2DutyCycle;
    uint8_t pwm3DutyCycle;
    uint8_t pwm4DutyCycle;
    uint8_t pwm5DutyCycle;
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
    uint16_t axis0_1_increment_value;
    /*Below parameter configures phase shift between axis 1 and axis 2 (axis 1 rise edge time stamp - axis 2 rise edge time stamp) in positive direction*/
    uint16_t axis1_2_increment_value;
    /*Below parameter time gap between axis 2 rise edge and axis 1 fall edge update*/
    uint16_t axis2_0_increment_value;
    /*Scheduler initial value*/
    uint32_t SchedulerIntialValue;
}pruIcssPwmParamsInDmem;

/*Compare values of 0 to 2 axis stored in PRUICSS DMEM used by PRU updated based on duty cycle configured*/
pruIcssPwmParamsInDmem *gPruIcssPwmCmpValues;
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
void updatePruIcssPwmParamsInDmem()
{
    uint32_t pruIcssIepClkFrequency = CONFIG_PRU_ICSS0_IEP_CLK_FREQ_HZ;
    uint32_t pruIcssPwmFrequency = PWM_OUTPUT_FREQ;
    uint32_t axesPhaseShiftValue = PHASE_SHIFT_BETWEEN_AXES;
    uint8_t iepIncrementValue = IEP_INCREMENT_VALUE;
    uint32_t pruIcssPwmPeriodby2 = (((float)(pruIcssIepClkFrequency * iepIncrementValue) / (pruIcssPwmFrequency))/2);

    /*Below parameter configures initial value of scheduler used to update compare values of axis 0*/
    gPruIcssPwmCmpValues->SchedulerIntialValue = pruIcssPwmPeriodby2;
    /*Below parameter is used to update scheduler (scheduler + increment value) used to update compare values of axis 1*/
    gPruIcssPwmCmpValues->axis0_1_increment_value  = ((float)(axesPhaseShiftValue * iepIncrementValue) / (CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
    /*Below parameter is used to update scheduler (scheduler value after configuring axis 1 + increment value) used to update compare values of axis 2*/
    gPruIcssPwmCmpValues->axis1_2_increment_value  = ((float)(axesPhaseShiftValue * iepIncrementValue) / (CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
    /*Below parameter is used to update scheduler (scheduler value after configuring axis 2 + increment value) used to update compare values of axis 0*/
    gPruIcssPwmCmpValues->axis2_0_increment_value  = (gPruIcssPwmCmpValues->SchedulerIntialValue) - (gPruIcssPwmCmpValues->axis0_1_increment_value + gPruIcssPwmCmpValues->axis1_2_increment_value );

    /* NOTE : Compare Increment and decrement values are updated once every PWM period in PRU Firmware*/
    for(uint8_t i = 0; i < NUMBER_OF_AXES; i++)
    {
        /*Initialize duty cycles of all PWM channels*/

        gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle = PWM_DUTY_CYCLE;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle = PWM_DUTY_CYCLE;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle = PWM_DUTY_CYCLE;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle = PWM_DUTY_CYCLE;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle = PWM_DUTY_CYCLE;
        gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle = PWM_DUTY_CYCLE;
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
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm0CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm1CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm2CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm3CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm4CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm5CmpUpValueIncrement = pruIcssPwmPeriodby2 - (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle/100)*(pruIcssPwmPeriodby2)) + ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        /*update all the axis PWM signals compare down values
         *compare Down increment value = (Duty_cycle*(PWM period/2)) - DeadBand
         *compare Down value = Scheduler value + compare down increment value (this add logic is implemented in PRU firmware)*/
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm0CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm0DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm1CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm1DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm2CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm2DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm3CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm3DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm4CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm4DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
        (gPruIcssPwmCmpValues->axisCmpValues[i]).Pwm5CmpDowmvalueIncrement = (((float)gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DutyCycle/100)*(pruIcssPwmPeriodby2)) - ((float)(gpruIcssPwmDutyCycleDeadBandValues[i].pwm5DeadBand * iepIncrementValue)/(CONFIG_PRU_ICSS0_IEP_CLK_PERIOD_NSEC));
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
    Drivers_open();

    int status;
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Get the pointer to the PRUICSS PWM parameters in DMEM */
    gPruIcssPwmCmpValues = (pruIcssPwmParamsInDmem *)(gPruIcss0Handle->hwAttrs->pru0DramBase + PRUICSS_PWM_PARAMS_DMEM_OFFSET);

    /* Call the board specific function to configure the IO expander */
    #if defined(am243x_evm)
    i2c_io_expander(NULL);
    #endif

    /* Initialize the PRUICSS DMEM memory */
    status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
    DebugP_assert(status != 0);

    /* Update the PRUICSS PWM parameters in DMEM */
    updatePruIcssPwmParamsInDmem();

    /* Load the PRU0 G0 firmware */
    status = PRUICSS_loadFirmware(gPruIcss0Handle, PRUICSS_PRU0, PRU0_G0_Firmware_0, sizeof(PRU0_G0_Firmware_0));
    DebugP_assert(SystemP_SUCCESS == status);

    /* Enter an infinite loop */
    while (1)
    {
        /* Sleep for 1 microsecond */
        ClockP_usleep(1);
    }

    /* Close the board drivers */
    Board_driversClose();

    /* Close the PRUICSS driver */
    Drivers_close();
}
