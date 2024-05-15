/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
//
// the includes
//
#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "kernel/dpl/CycleCounterP.h"



#include "sfra_main.h"

//
// globals
//

int16_t vTimer0[4];         // Virtual Timers slaved off CPU Timer 0 (A events)
int16_t vTimer1[4];         // Virtual Timers slaved off CPU Timer 1 (B events)
int16_t vTimer2[4];         // Virtual Timers slaved off CPU Timer 2 (C events)
volatile int16_t time=0;
//
// Variable declarations for state machine
//
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

//
// State Machine function prototypes
//

//
// Alpha states
//
void A0(void);  //state A0
void B0(void);  //state B0

//
// A branch states
//
void A1(void);  //state A1

//
// B branch states
//
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

//
//--- System Related ---
//
DCL_PI gi;
DCL_DF22 gi2;

volatile float32_t gi_out;
volatile float32_t gi_out_prev;
//
// Reference variables
// current set point
//
volatile float32_t ac_cur_ref;
//
//flag to close the loop
//
volatile int32_t closeGiLoop;

//
// SFRA Related Variables
//
SFRA_F32 sfra1;

float32_t plantMagVect[SFRA_FREQ_LENGTH];
float32_t plantPhaseVect[SFRA_FREQ_LENGTH];
float32_t olMagVect[SFRA_FREQ_LENGTH];
float32_t olPhaseVect[SFRA_FREQ_LENGTH];
float32_t clMagVect[SFRA_FREQ_LENGTH];
float32_t clPhaseVect[SFRA_FREQ_LENGTH];
float32_t freqVect[SFRA_FREQ_LENGTH];
float32_t injectarray[SFRA_FREQ_LENGTH];

int32_t bcgcnt=0;
int32_t injcnt=0;
volatile int32_t var1 = 0;
volatile int32_t sfrastate;
volatile int32_t tableptrstart;
volatile int32_t tableptr;
volatile int32_t collectcnt;
volatile int32_t injectcnt;
volatile int32_t precount;
volatile int32_t count;
volatile int32_t sfra_act_state;

#define Test_Automation
#ifdef Test_Automation
#include "sfra_expected.h"
extern int olmag_exp[100], olphase_exp[100];
int cnt, temp1, temp2, error;
#endif

int main(void)
{
    //
    // This routine sets up the basic device and peripheral initialization configuration done through syscfg
    // this routine will also initialize the timers that are used in
    // the background task for this system
    //
    System_init();
    Board_init();
    Drivers_open();
    Board_driversOpen();
    // Initialize timer frequency for Task A to 100Hz, Task B to 10Hz and Task C TO 10KHz
    //
    // Tasks State-machine init
    //
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    //
    // Stop all PWM mode clock
    //
    disablePWMCLKCounting();

    setupUpDwnCountPWM(PWM_BASE, PFC_PWM_PERIOD);

    //
    // power up ADC on the device
    //
    setupADC();

    //
    //Profiling GPIO
    //
    setupProfilingGPIO();

    //
    // Initialize global variables generic to the board like ones used to
    // read current values and others
    //
    globalVariablesInit();

    //
    // setup SFRA
    //
    setupSFRA();

    //
    // Enable PWM Clocks
    //
    enablePWMCLKCounting();

    //
    // safe to setup PWM pins
    //
    setPinsAsPWM();

    //
    // ISR Mapping
    //
    setupInterrupt();

    //
    // IDLE loop. Just sit and loop forever,
    // periodically will branch into A0-A3, B0-B3, C0-C3 tasks
    // Frequency of this branching is set in setupDevice routine:
    //
    for(;;)  //infinite loop
    {
        //
        // State machine entry & exit point
        //
        (*Alpha_State_Ptr)(); // jump to an Alpha state (A0,B0,...)
        #ifdef Test_Automation      
            cnt = sfra1.freqIndex;
            if (cnt == 100){
              for(int j=0; j<100; j++){
                temp1 = olMagVect[j];
                temp2 = olPhaseVect[j];
                if( (temp1 != olmag_exp[j]) || (temp2 !=olphase_exp[j])){
                    error++;
                    DebugP_log("Error in %d element: mag_vect = %d and phase_vect = %d \r\n", j,temp1,temp2);
                }
            }
             if(error == 0){
                DebugP_log("Test passed\r\n");
                break;
            }
         }
        #endif
    }
    Board_driversClose();
    Drivers_close();
    Board_deinit();
    System_deinit();
    return 0;
}

//
// control ISR Code
//

void controlISR(void *args)
{
    controlCode();
    EPWM_clearEventTriggerInterruptFlag(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);
}


//
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//

void A0(void)
{
    time++;

    //
    // loop rate synchronizer for A-tasks
    //
    if(GET_TASK_A_TIMER_OVERFLOW_STATUS == 1)
    {
        CLEAR_TASK_A_TIMER_OVERFLOW_FLAG;   // clear flag

        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)

    }

    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    //
    // loop rate synchronizer for B-tasks
    //
    if(GET_TASK_B_TIMER_OVERFLOW_STATUS  == 1)
    {
        CLEAR_TASK_B_TIMER_OVERFLOW_FLAG;               // clear flag

        (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;      // Allow C state tasks
}

//
//  A - TASKS (executed in every 1 msec)
//
void A1(void)
{

    SFRA_F32_runBackgroundTask(&sfra1);
  //  SFRA_GUI_runSerialHostComms(&sfra1);


    bcgcnt++;
    //
    //the next time Timer0 'counter' reaches Period value go to A2
    //
    A_Task_Ptr = &A1;
}

//
//  B - TASKS (executed in every 5 msec)
//
void B1(void)
{
    //
    // the next time CpuTimer1 'counter' reaches Period value go to B2
    //
    B_Task_Ptr = &B2;
}

void B2(void)
{
    B_Task_Ptr = &B3;
}

void B3(void) //  SPARE
{
    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B1;
}

//
// setupSFRA
//
void setupSFRA(void)
{
    //
    //Resets the internal data of sfra module to zero
    //
    SFRA_F32_reset(&sfra1);

    //
    //Configures the SFRA module
    //
    SFRA_F32_config(&sfra1,
                    SFRA_ISR_FREQ,
                    SFRA_AMPLITUDE,
                    SFRA_FREQ_LENGTH,
                    SFRA_FREQ_START,
                    SFRA_FREQ_STEP_MULTIPLY,
                    plantMagVect,
                    plantPhaseVect,
                    olMagVect,
                    olPhaseVect,
                    clMagVect,
                    clPhaseVect,
                    freqVect,
                    1);

    //
    //Resets the response arrays to all zeroes
    //
    SFRA_F32_resetFreqRespArray(&sfra1);

    //
    //Initializes the frequency response array ,
    //The first element is SFRA_FREQ_START
    //The subsequent elements are freqVect[n-1]*SFRA_FREQ_STEP_MULTIPLY
    //This enables placing a fixed number of frequency points
    //between a decade of frequency.
    // The below routine can be substituted by a routine that sets
    // the frequency points arbitrarily as needed.
    //
    SFRA_F32_initFreqArrayWithLogSteps(&sfra1,
                                       SFRA_FREQ_START,
                                       SFRA_FREQ_STEP_MULTIPLY);


}

void globalVariablesInit(void)
{

    gi.Kp = GI_PI_KP;
    gi.Ki = GI_PI_KI;
    gi.Umax = GI_PI_MAX;
    gi.Umin = GI_PI_MIN;
    gi.i10 = 0;
    gi.i6 = 0;

    ac_cur_ref = 0.03;
    closeGiLoop = 1;

}


//
// End
//
