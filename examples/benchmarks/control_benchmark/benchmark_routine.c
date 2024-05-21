/*
 * Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <math.h>
#include "fwc.h"
#include "mtpa.h"
#include "vs_freq.h"
#include "math_types.h"

#define TEST_SIZE 600

int control_benchmarks(void);

void benchmark_main(void *args)
{   
    int status;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("\n BENCHMARK START - Benchmark results - Control benchmark \r\n");
    DebugP_log("\n\nFunctions\t| Avg Cycles \t| Max Cycles \t|\r\n");
    DebugP_log("----------------|---------------|-----------------------|\r\n");
    
    status = control_benchmarks();

    DebugP_log("- Ran with the latest LTS version of TI Clang Compiler, with -Os flag, obtained the average result from 600 consecutive reading of running the function with DPL CycleCountP minus the overhead, simulating a control loop scenario.\n");
    DebugP_log("- Actual result may vary depending on provided datasets and memory configuration. For R5F, it is recommended for users to map control loops to TCM for the best performance.\n");

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("BENCHMARK END \r\n");
        DebugP_log("Control Benchmark Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

int control_benchmarks()
{
    uint32_t i;
    uint32_t t1, t2, t3;
    uint32_t overhead;
    uint32_t totCnt, currCnt;
    float32_t maxCnt, avgCnt;

    float32_t    RadStep  = 0.062831853071f;
    float32_t    Rad      = 0.062831853071f;
    float32_t    Rad2     = -1.0f;

    FWC_Obj testFWC;
    MTPA_Obj testMTPA;
    VS_FREQ_Obj testVS;

    FWC_setParams(&testFWC, 2.5, 0, -1, 1);
    MTPA_computeParameters(&testMTPA, 2.0f, 1.5f, 1.0f);
    MTPA_enable(&testMTPA);
    VS_FREQ_setProfile(&testVS,-1,1,-3,3);

    maxCnt = 0;
    totCnt = 0;
    avgCnt = 0;
    for (i = 0; i < TEST_SIZE; i++) 
    {
        Rad2 = fmodf(Rad + RadStep, 1.0f);

        t1 = CycleCounterP_getCount32();
        FWC_computeCurrentAngle(&testFWC,Rad, Rad2);
        t2 = CycleCounterP_getCount32();
        t3 = CycleCounterP_getCount32();

        overhead = t3 - t2;
        currCnt = t2 - t1 - overhead;
        totCnt += currCnt;
        if (maxCnt < currCnt)
        {
            maxCnt = currCnt;
        }
    }
    avgCnt = totCnt / TEST_SIZE;
    DebugP_log("**Field weakening control**\r\n");
    DebugP_log("FWC_computerCurrentAngle() | \t%.2f\t | \t%.2f\t |\r\n", avgCnt, maxCnt);

    maxCnt = 0;
    totCnt = 0;
    avgCnt = 0;
    for (i = 0; i < TEST_SIZE; i++) 
    {
        Rad2 = fmodf(Rad + RadStep, 2.0f);

        t1 = CycleCounterP_getCount32();
        MTPA_computeCurrentReference(&testMTPA,Rad2);
        t2 = CycleCounterP_getCount32();
        t3 = CycleCounterP_getCount32();

        overhead = t3 - t2;
        currCnt = t2 - t1 - overhead;
        totCnt += currCnt;
        if (maxCnt < currCnt)
        {
            maxCnt = currCnt;
        }
    }
    avgCnt = totCnt / TEST_SIZE;
    DebugP_log("**Maximum Torque Per Ampere**\r\n");
    DebugP_log("MTPA_computeCurrentReference() | \t%.2f\t | \t%.2f\t |\r\n", avgCnt, maxCnt);

    maxCnt = 0;
    totCnt = 0;
    avgCnt = 0;
    for (i = 0; i < TEST_SIZE; i++) 
    {
        Rad2 = fmodf(Rad + RadStep, 2.0f);

        t1 = CycleCounterP_getCount32();
        MTPA_computeCurrentAngle(&testMTPA,Rad2);
        t2 = CycleCounterP_getCount32();
        t3 = CycleCounterP_getCount32();

        overhead = t3 - t2;
        currCnt = t2 - t1 - overhead;
        totCnt += currCnt;
        if (maxCnt < currCnt)
        {
            maxCnt = currCnt;
        }
    }
    avgCnt = totCnt / TEST_SIZE;
    DebugP_log("MTPA_computeCurrentAngle() | \t%.2f\t | \t%.2f\t |\r\n", avgCnt, maxCnt);


    maxCnt = 0;
    totCnt = 0;
    avgCnt = 0;
    for (i = 0; i < TEST_SIZE; i++) 
    {
        Rad2 = fmodf(Rad + RadStep, 2.0f);

        t1 = CycleCounterP_getCount32();
        VS_FREQ_run(&testVS,Rad2);
        t2 = CycleCounterP_getCount32();
        t3 = CycleCounterP_getCount32();

        overhead = t3 - t2;
        currCnt = t2 - t1 - overhead;
        totCnt += currCnt;
        if (maxCnt < currCnt)
        {
            maxCnt = currCnt;
        }
    }
    avgCnt = totCnt / TEST_SIZE;
    DebugP_log("**Stator Voltage Frequency Generator**\r\n");
    DebugP_log("VS_FREQ_run() | \t%.2f\t | \t%.2f\t |\r\n", avgCnt, maxCnt);

    return 0;
}
