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

#include "pi_test.h"

void dcl_pi_main(void *args)
{   
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    //
    // Available PI controllers are:
    // - DCL_runPISeries
    // - DCL_runPIParallel
    // - DCL_runPISeriesTustin
    // - DCL_runPIParallelEnhanced (Requires additional parameters)
    //
    PI_runTest(pi_controller, &DCL_runPISeries);

    Board_driversClose();
    Drivers_close();
}

int PI_runTest(DCL_PI *ctrl_handle, DCL_PI_FUNC dcl_pi_func)
{   
    //
    // Define DFLOG pointers that will be used to access the data buffer
    //
    DCL_FDLOG rkBuf, ykBuf, outBuf, ctlBuf;
    DCL_resetPI(ctrl_handle);

    //
    // Initialize Log pointers to the data buffer
    //
    DCL_initLog(&rkBuf, (float32_t*)rk_buffer, DATA_LENGTH);
    DCL_initLog(&ykBuf, (float32_t*)yk_buffer, DATA_LENGTH);
    DCL_initLog(&outBuf, (float32_t*)out_buffer, DATA_LENGTH);
    DCL_initLog(&ctlBuf, (float32_t*)ctl_buffer, DATA_LENGTH);
    DCL_clearLog(&outBuf);

    int i;
    for (i = 0; i < NUM_ELEMENTS; i++)
    {   
        //
        // rk = Target referenced value
        // yk = Current feedback value
        // uk = Output control effort
        //
        float32_t rk,yk,uk;
        
        //
        // Read the input data buffers
        //
        rk = DCL_readLog(&rkBuf);
        yk = DCL_readLog(&ykBuf);

        //
        // Run the controller
        // Equivalent to uk = DCL_runPI_series(ctrl_handle, rk, yk);
        //
        uk = (*dcl_pi_func)(ctrl_handle, rk, yk); 

        //
        // Write the results to the output buffer
        //
        DCL_writeLog(&outBuf, uk);

    }

    //
    // Reset the log pointer so it starts from the beginning
    //
    DCL_resetLog(&outBuf);

    //
    // Check output against expected output with tolerance (1e-06)
    //
    int errors = 0;
    for (i = 0; i < NUM_ELEMENTS; i++)
    {
        float32_t output = DCL_readLog(&outBuf);   // out_buffer[i]
        float32_t expected = DCL_readLog(&ctlBuf); // ctl_buffer[i]
        if (!DCL_isZero(output - expected))
        {
            errors++;
            DebugP_log("FAIL at sample %d, outputs %f, should be %f\n", i, output, expected);
        }
    }

    DebugP_log("PI test produced %d error\n",errors);

    return errors;
}


