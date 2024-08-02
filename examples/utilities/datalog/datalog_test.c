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

#include <datalog.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */


#define Test_automation
#ifdef Test_automation
#include "dlog_expected.h"
int j =0;
int pass, error;
int logged_data;
extern int expected_data[400];
#endif

float32_t i = 0;
int32_t temp = 0;
float32_t var1;
float32_t var2;
float32_t var3;
float32_t var4;

float32_t trigger = 5.0f;

void datalog_test(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    datalogHandle = DATALOG_init(&datalog, sizeof(datalog),manual ,5, 1);
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;


   while(1){
       var1 = i;
       var2 = i*10;
       var3 = i*100;
       var4 = i*1000;

       datalogObj->iptr[0] = &var1;
       datalogObj->iptr[1] = &var2;
       datalogObj->iptr[2] = &var3;
       datalogObj->iptr[3] = &var4;

       i = i+1;
       temp++;
       if(datalogObj->trigger == manual)
        {
        if( (var1 > trigger) && (datalogObj->prev_value <= trigger))
        {
           datalogObj->flag_enableLogOneShot = TRUE;
           DebugP_log("Datalog initiated with manual trigger!!\r\n");
        }
         }
       DATALOG_update(datalogHandle);


       if(temp == 2000)
       {
           break;
       }
   }

#ifdef Test_automation
   for (j=0; j<400; j++){
       logged_data = datalogObj->datalogBuff[0][j];
       if (logged_data == expected_data[j]){
           pass++;
       }
       else
       {
           error++;
       }
   }
#endif

    DebugP_log("Datalog buffers are updated !!\r\n");
   if(error == 0){
       DebugP_log("Test passed !!\r\n");
   }
   else{
     DebugP_log("Test failed with %d errors !!\r\n",error);
   }


    Board_driversClose();
    Drivers_close();
}
