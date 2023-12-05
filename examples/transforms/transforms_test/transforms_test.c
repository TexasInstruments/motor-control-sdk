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

#include "transforms_test.h"

int verify_output(int test_num,\
float32_t* alphaBetaCurrent, float32_t* dqCurrent, float32_t* alphaBetaVoltage, float32_t* abcOutput)
{
    // Compares park output with expected park output
    bool is_park_erroneous = is_different(testParams[test_num].refAlpha, dqCurrent[0]) || is_different(testParams[test_num].refBeta, dqCurrent[1]);

    // Compares svgen output with expected svgen output
    bool is_svgen_erroneous = is_different(testParams[test_num].refA, abcOutput[0]) || is_different(testParams[test_num].refB, abcOutput[1]) || is_different(testParams[test_num].refC, abcOutput[2]);

    // Compares output make sure that running park then ipark would revert all the changes
    bool is_park_ipark_erroneous = is_different(alphaBetaCurrent[0], alphaBetaVoltage[0]) || is_different(alphaBetaCurrent[1], alphaBetaVoltage[1]);

    if (is_park_erroneous || is_svgen_erroneous || is_park_ipark_erroneous)
    {
        DebugP_log("Test case %d (zero-indexed) produced error:\n",test_num);
        if(is_park_erroneous)\
            DebugP_log("Incorrect park value, the resultant is %f %f, but the expected is %f %f\n",dqCurrent[0],dqCurrent[1], testParams[test_num].refAlpha, testParams[test_num].refBeta);
        if(is_svgen_erroneous)\
            DebugP_log("Incorrect svgen value, the resultant is %f %f %f, but the expected is %f %f %f\n", abcOutput[0], abcOutput[1], abcOutput[2], testParams[test_num].refA, testParams[test_num].refB, testParams[test_num].refC);
        if(is_park_ipark_erroneous)\
            DebugP_log("Incorrect value, the original clarke value is %f %f, but the value after park-ipark is %f %f, before and after value should be the same\n", alphaBetaCurrent[0], alphaBetaCurrent[1], alphaBetaVoltage[0], alphaBetaVoltage[1]);
        return 1;
    }
    return 0;
}

void transforms_test_main()
{
    int testSize = sizeof(testParams)/sizeof(*testParams);
    int error = 0;


    for (int idx = 0; idx < testSize; idx++) 
    {
        float32_t abcOutput[3], abcInput[3] = {testParams[idx].inA,testParams[idx].inB,testParams[idx].inC};
        float32_t alphaBetaI[2], dqI[2], alphaBetaV[2];

        float32_t sincosVal[2];
        ti_arm_sincos(phasorOffset, &sincosVal[0]);

        CLARKE_run_threeInput(abcInput[0], abcInput[1], abcInput[2], &alphaBetaI[0], &alphaBetaI[1]);
        PARK_run(sincosVal[0], sincosVal[1], alphaBetaI[0], alphaBetaI[1], &dqI[0], &dqI[1]);
        IPARK_run(sincosVal[0], sincosVal[1], dqI[0], dqI[1], &alphaBetaV[0], &alphaBetaV[1]);
        
        //
        // Using common mode for SVM modulation, functions for DPWM min/max modulation are also provided 
        //
        SVGEN_runCom(1.0f, alphaBetaV[0], alphaBetaV[1], &abcOutput[0], &abcOutput[1], &abcOutput[2]);

        error += verify_output(idx, &alphaBetaI[0], &dqI[0], &alphaBetaV[0], &abcOutput[0]);
    }

    DebugP_log("Transforms test produced %d error",error);

}
