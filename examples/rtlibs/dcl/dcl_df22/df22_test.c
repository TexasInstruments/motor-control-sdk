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

#include "df22_test.h"

void dcl_df22_main(void *args)
{   
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DF22_runTest(df22_controller);

    Board_driversClose();
    Drivers_close();
}

int DF22_runTest(DCL_DF22 *ctrl_handle)
{
    //
    // Define DFLOG pointers that will be used to access the data buffer
    //
    DCL_FDLOG inBuf, outBuf, ctlBuf;
    DCL_resetDF22(ctrl_handle);

    //
    // Initialize Log pointers to the data buffer
    //
    DCL_initLog(&inBuf, (float32_t*)in_buffer, DATA_LENGTH);
    DCL_initLog(&outBuf, (float32_t*)out_buffer, DATA_LENGTH);
    DCL_initLog(&ctlBuf, (float32_t*)ctl_buffer, DATA_LENGTH);
    DCL_clearLog(&outBuf);

    int i;
    for (i = 0; i < NUM_ELEMENTS; i++)
    {
        //
        // ek = Servo error value
        // uk = Output control effort
        //
        float32_t ek,uk;

        //
        // Read the input data buffers
        //
        ek = DCL_readLog(&inBuf);
       
        //
        // Run the controller
        // Note: DCL_runDF22() does not have an internal clamp
        //       Use DCL_runDF22Clamp(ctrl_handle, ek, max, min)
        //       instead if an internal clamp is needed.
        //       DCL also provides external clamp DCL_runClamp(*data, max, min)
        //
        uk = DCL_runDF22(ctrl_handle, ek);

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
    // Check output against reference output with tolerance
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

    DebugP_log("DF22 test produced %d error\n",errors);

    return errors;
}