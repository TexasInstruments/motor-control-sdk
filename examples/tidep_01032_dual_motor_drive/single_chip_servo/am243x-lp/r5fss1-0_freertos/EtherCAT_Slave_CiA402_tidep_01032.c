/*!
 *  \example EtherCAT_Slave_CiA402.c
 *
 *  \brief
 *  EtherCAT<sup>&reg;</sup> Slave Example Application with CiA402 Profile.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 *  Copyright (c) 2024 KUNBUS GmbH.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer./<li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGE.
 *
 */

#define TIESC_HW	0
#define PRU_200MHZ  1

#include <stdio.h>
#include <drivers/soc.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_dpl_config.h"
#include "ti_drivers_config.h"

/* this one shall the only one to be using it ! */
#include "project.h"
#include "ecSlvCiA402.h"

#if (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
#if (defined FBTLIMPL_LINEUART) && (1==FBTLIMPL_LINEUART)
 #include "sysLib_lineUart.h"
#else
 #include <sysLib_sharedMemory.h>
#endif
#endif

/* OS */
#include <osal.h>

/* ESL */
#include <ESL_os.h>
#include <ESL_BOARD_config.h>
#include <ESL_BOARD_OS_config.h>
#include <ESL_cia402Obd.h>
#include <ESL_version.h>

/* stack */
#include <ecSlvApi.h>

/* @cppcheck_justify{misra-c2012-8.9} prefer module global over threadsafety */
/* cppcheck-suppress misra-c2012-8.9 */
static OSAL_PJumpBuf_t  farJumpBuf_cia;

static uint32_t EC_SLV_APP_CIA_remoteInit(EC_SLV_APP_CIA_Application_t *applicationInstance);


/* For the tidep_01032_dual_motor_drive reference design, ICSSG0 core clock should be configured
 * to 300 MHz and ICSSG1 core clock should be configured to 200 MHz. R5F_0_0 configures ICSSG0 first
 * and then R5F_1_0 configures ICSSG1. Because of MCUSDK-12117 bug, ICSSG1 clock configuration to
 * 200 MHz causes ICSSG0 also to run at 200 MHz which is a problem for EnDat and SDFM firmwares.
 * To avoid this issue, the clock configuration of ICSSG1 should be done separately and not from
 * SysConfig generated API. Therefore ESL_OS_init()->System_init() is not called here. Instead
 * System_init_modified() is defined and called here, which uses a different API call for
 * initializing ICSSG1 clock when compared to System_init().
 *
 * System_init_modified() function is same as System_init() except PowerClock_init() API call.
 * If SysConfig is being updated which leads to changes in the generated code for System_init(),
 * please update System_init_modified() function as well.
 */
void System_init_modified(void);
void PowerClock_init_modified(void);
void Module_clockSetFrequency_modified(void);

extern void Drivers_uartInit(void);
extern void Pinmux_init(void);
extern void GPIO_init();
extern void Module_clockEnable(void);

typedef struct {

    uint32_t moduleId;
    uint32_t clkId;
    uint32_t clkRate;

} SOC_ModuleClockFrequency;

extern SOC_ModuleClockFrequency gSocModulesClockFrequency[];

#define SOC_MODULES_END         (0xFFFFFFFFu)

void Module_clockSetFrequency_modified(void)
{
    int32_t status;
    uint32_t i = 0;
    uint32_t icssg0ClkParent, icssg1ClkParent;

    while(gSocModulesClockFrequency[i].moduleId!=SOC_MODULES_END)
    {
        /*For ICSSG1, call a different API*/

        if((gSocModulesClockFrequency[i].moduleId == TISCI_DEV_PRU_ICSSG1) &&
           (gSocModulesClockFrequency[i].clkId == TISCI_DEV_PRU_ICSSG1_CORE_CLK))
        {

            status = Sciclient_pmGetModuleClkParent(TISCI_DEV_PRU_ICSSG0,
                                                    TISCI_DEV_PRU_ICSSG0_CORE_CLK,
                                                    &icssg0ClkParent,
                                                    SystemP_WAIT_FOREVER);
            DebugP_assertNoLog(status == SystemP_SUCCESS);

            /*Use a different parent clock for ICSSG1*/
            icssg1ClkParent = icssg0ClkParent + 1;

            status = SOC_moduleSetClockFrequencyWithParent(
                        gSocModulesClockFrequency[i].moduleId,
                        gSocModulesClockFrequency[i].clkId,
                        icssg1ClkParent,
                        gSocModulesClockFrequency[i].clkRate
                        );
        }
        if((gSocModulesClockFrequency[i].moduleId == TISCI_DEV_PRU_ICSSG1) &&
           (gSocModulesClockFrequency[i].clkId == TISCI_DEV_PRU_ICSSG1_IEP_CLK))
        {

            status = Sciclient_pmGetModuleClkParent(TISCI_DEV_PRU_ICSSG0,
                                                    TISCI_DEV_PRU_ICSSG0_IEP_CLK,
                                                    &icssg0ClkParent,
                                                    SystemP_WAIT_FOREVER);
            DebugP_assertNoLog(status == SystemP_SUCCESS);

            /*Use a different parent clock for ICSSG1*/
            icssg1ClkParent = icssg0ClkParent + 1;

            status = SOC_moduleSetClockFrequencyWithParent(
                        gSocModulesClockFrequency[i].moduleId,
                        gSocModulesClockFrequency[i].clkId,
                        icssg1ClkParent,
                        gSocModulesClockFrequency[i].clkRate
                        );
        }
        else
        {
            status = SOC_moduleSetClockFrequency(
                        gSocModulesClockFrequency[i].moduleId,
                        gSocModulesClockFrequency[i].clkId,
                        gSocModulesClockFrequency[i].clkRate
                        );
        }
        DebugP_assertNoLog(status == SystemP_SUCCESS);
        i++;
    }
}

void PowerClock_init_modified(void)
{
    Module_clockEnable();
    Module_clockSetFrequency_modified();
}

void System_init_modified(void)
{

    /* This function is same as System_init() except PowerClock_init() API call.
     * If SysConfig is being updated which leads to changes in the generated
     * code for System_init(), please update this function as well.*/

    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();
    /* We should do sciclient init before we enable power and clock to the peripherals */
    /* SCICLIENT init */
    {
        int32_t retVal = SystemP_SUCCESS;

        retVal = Sciclient_init(CSL_CORE_ID_R5FSS1_0);
        DebugP_assertNoLog(SystemP_SUCCESS == retVal);
    }


    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init_modified();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    OSPI_init();
    GPIO_init();
    PRUICSS_init();
    /* PRU IEP Enable SYNC MODE */
    CSL_REG32_WR(CSL_PRU_ICSSG1_PR1_CFG_SLV_BASE + CSL_ICSSCFG_IEPCLK, 1);

    Drivers_uartInit();
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Application Loop Task
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArg_p      Task Parm.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // required variables
 *  void* pvVariable = NULL;
 *
 *  // the Call
 *  EC_SLV_APP_CIA_loopTask(pvVariable);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
static void EC_SLV_APP_CIA_loopTask(void *pArg_p)
{
    /* @cppcheck_justify{misra-c2012-11.5} generic API requires cast */
    /* cppcheck-suppress misra-c2012-11.5 */
    EC_SLV_APP_CIA_Application_t *pApplicationInstance      = (EC_SLV_APP_CIA_Application_t*)pArg_p;
    uint32_t                        error                   = EC_API_eERR_NONE;
    char *pProductName = NULL;
    uint32_t vendorId = 0;
    uint32_t productCode = 0;
    EC_API_SLV_EEsmState_t          state                   = EC_API_SLV_eESM_uninit;
    uint16_t                        alErrorCode             = 0;

    if (!pApplicationInstance)
    {
        /* @cppcheck_justify{misra-c2012-15.1} goto is used to assure single point of exit */
        /* cppcheck-suppress misra-c2012-15.1 */
        goto Exit;
    }

    EC_SLV_APP_CIA_initBoardFunctions(pApplicationInstance);
    EC_SLV_APP_CIA_registerStacklessBoardFunctions(pApplicationInstance);

    error = EC_API_SLV_stackInit(); // EtherCAT stack init
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

    EC_SLV_APP_cia402_initAxisObjects(pApplicationInstance);

    EC_SLV_APP_CIA_applicationInit(pApplicationInstance);

    EC_API_SLV_getVendorId(pApplicationInstance->ptEcSlvApi, &vendorId);
    EC_API_SLV_getProductCode(pApplicationInstance->ptEcSlvApi, &productCode);
    error = EC_API_SLV_getProductName(pApplicationInstance->ptEcSlvApi, &pProductName);
    OSAL_printf("%s - %xh / %xh\r\n", pProductName, vendorId, productCode);

    /* spit out versions */
    ESL_dumpVersions(pApplicationInstance->ptEcSlvApi);

    error = EC_SLV_APP_CiA_fetchAxisObjects(pApplicationInstance);
    if (error != EC_API_eERR_NONE)
    {
        OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

    for(;;)
    {
#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (defined GPIO_TEST_PROFILE_1) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_set(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_0_MASK/*|ESL_TESTPIN_1_MASK*/);
#endif
#endif

        EC_API_SLV_mainLoopCyclic();

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#if (defined GPIO_TEST_PROFILE_SEL) && (defined GPIO_TEST_PROFILE_1) && (GPIO_TEST_PROFILE_1 == GPIO_TEST_PROFILE_SEL)
        ESL_GPIO_testPins_clear(ESL_TESTPIN_STATE_REG_BANK, ESL_TESTPIN_0_MASK);
#endif
#endif

        EC_API_SLV_getState(pApplicationInstance->ptEcSlvApi, &state, &alErrorCode);
        if (EC_API_SLV_eESM_init < state)
        {
            OSAL_SCHED_yield();
        }
        else
        {
            /* for carve up give some air */
            OSAL_SCHED_sleep(10);
        }
    }
    // not reachable
    //EC_SLV_APP_applicationDeInit();
Exit:
    ESL_OS_taskLeave();

    return;
}

static void EC_SLV_APP_CIA_mainTask(void *pArg_p)
{
    /* @cppcheck_justify{misra-c2012-11.5} generic API requires cast */
    /* cppcheck-suppress misra-c2012-11.5 */
    EC_SLV_APP_CIA_Application_t *applicationInstance = (EC_SLV_APP_CIA_Application_t*)pArg_p;
    uint32_t                        retVal              = OSAL_ERR_NoError;

    if (!applicationInstance)
    {
        OSAL_error(__func__, __LINE__, OSAL_ERR_InvalidParm, true, 1, "Application instance missing!!!\r\n");
        goto Exit;
    }

    retVal = ESL_OS_boardInit(applicationInstance->selectedPruInstance);
    if (OSAL_ERR_NoError != retVal)
    {
        OSAL_error(__func__, __LINE__, retVal, true, 1, "OS Board init error\r\n");
    }

    OSAL_registerPrintOut(NULL, ESL_OS_printf);

    retVal = EC_SLV_APP_CIA_remoteInit(applicationInstance);

    if(!((OSAL_CONTAINER_LOCALIMPLEMENTATION == retVal) || (OSAL_ERR_NoError == retVal)))
    {
        OSAL_error(__func__, __LINE__, OSAL_ERR_InvalidParm, true, 1, "Fatal error remote API init!!!\r\n");
        /* @cppcheck_justify{misra-c2012-15.1} assure single point of exit */
        /* cppcheck-suppress misra-c2012-15.1 */
        goto Exit;
    }

    retVal = EC_API_SLV_load(&farJumpBuf_cia, NULL /* &applErrHandler*/, applicationInstance->selectedPruInstance);

    if (EC_API_eERR_NONE == retVal)
    {
        EC_API_SLV_prepareTasks(KBECSLV_PRIO_PDI, KBECSLV_PRIO_LED, KBECSLV_PRIO_SYNC0, KBECSLV_PRIO_SYNC1);

        applicationInstance->loopThreadHandle = OSAL_SCHED_startTask(EC_SLV_APP_CIA_loopTask
                                                                    ,applicationInstance
#if !(defined FBTL_REMOTE) || (0==FBTL_REMOTE)
                                                                    ,OSAL_TASK_Prio_Normal
#else
                                                                    ,OSAL_TASK_Prio_Idle
#endif
                                                                    ,(uint8_t*)EC_SLV_APP_applLoopTaskStack_g
                                                                    ,APPLLOOP_TASK_SIZE_BYTE
                                                                    ,0
                                                                    ,"Appl_LoopTask");
        if ( NULL == applicationInstance->loopThreadHandle)
        {
            OSAL_printf("Error return start Loop Task\r\n");
            OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
        }

        OSAL_SCHED_joinTask(applicationInstance->loopThreadHandle);

        /* No default tasks (run forever) here, so introduce a wait cycle */
        for (;;)
        {
            OSAL_SCHED_sleep(1000);
        }
    }
    else
    {
        for (;;)
        {

            for (;;)
            {
                OSAL_printf("long jmp restart\r\n");
                OSAL_SCHED_sleep(1000);
            }
        }
    }

    // not reachable

    //ESL_OS_boardDeinit();
    //EC_API_SLV_unLoad();

    //return 0;
Exit:
    return;
}

/*!
 *  \brief
 *  Main entry point.
 *
 *  \details
 *  Simple EtherCAT Slave example demonstrating the configuration the hardware,
 *  creation of the base slave information as well as the Object Dictionary
 *  and the Process Data configuration. Furthermore, the FoE Protocol and the
 *  EEPROM handling are covered on this example.
 *
 *  \param[in]  argc                            Number of command line arguments.
 *  \param[in]  *argv                           Pointer to an array of command line arguments.
 *
 *  \return     uint32_t of type #NAMESPACE_EError_t.
 *  \retval     0                               Success.
 *  \retval     -1                              Board initialization failed.
 *
 */
int main(int argc, char *argv[])
{
    /* @cppcheck_justify{threadsafety-threadsafety} no called reentrant */
    /* cppcheck-suppress threadsafety-threadsafety */
    static uint32_t                     selectedPruInstance     = UINT32_MAX;
    /* @cppcheck_justify{threadsafety-threadsafety} no called reentrant */
    /* cppcheck-suppress threadsafety-threadsafety */
    static EC_SLV_APP_CIA_Application_t applicationInstance     = {0};
    uint32_t                            error                   = OSAL_ERR_NoError;

    /* For the tidep_01032_dual_motor_drive reference design, ICSSG0 core clock should be configured
     * to 300 MHz and ICSSG1 core clock should be configured to 200 MHz. R5F_0_0 configures ICSSG0 first
     * and then R5F_1_0 configures ICSSG1. Because of MCUSDK-12117 bug, ICSSG1 clock configuration to
     * 200 MHz causes ICSSG0 also to run at 200 MHz which is a problem for EnDat and SDFM firmwares.
     * To avoid this issue, the clock configuration of ICSSG1 should be done separately and not from
     * SysConfig generated API. Therefore ESL_OS_init()->System_init() is not called here. Instead
     * System_init_modified() is defined and called here, which uses a different API call for
     * initializing ICSSG1 clock when compared to System_init().
     *
     * System_init_modified() function is same as System_init() except PowerClock_init() API call.
     * If SysConfig is being updated which leads to changes in the generated code for System_init(),
     * please update System_init_modified() function as well.
     */

    /* ESL_OS_init(); */
    System_init_modified();

#if !(defined FBTL_REMOTE)
    selectedPruInstance = ESL_DEFAULT_PRUICSS;
#endif

    if (1 < argc)
    {
        char *inst = argv[1];
        selectedPruInstance = strtoul(inst, NULL, 0);
    }

    /* Init osal */
    OSAL_init();

    applicationInstance.selectedPruInstance = selectedPruInstance;

    TaskP_Params_init(&applicationInstance.mainThreadParam);

    applicationInstance.mainThreadParam.name        = "mainThread";
    applicationInstance.mainThreadParam.stackSize   = MAIN_TASK_SIZE_BYTE; /* FreeRTOS wants Stacksize in Units, when called direct, using TaskP it's bytes */
    applicationInstance.mainThreadParam.stack       = (uint8_t*)EC_SLV_APP_mainTaskStack_g;
    applicationInstance.mainThreadParam.priority    = (TaskP_PRIORITY_LOWEST+1);
    applicationInstance.mainThreadParam.taskMain    = (TaskP_FxnMain)EC_SLV_APP_CIA_mainTask;
    applicationInstance.mainThreadParam.args        = (void*)&applicationInstance;

    error = TaskP_construct(&applicationInstance.mainThreadHandle
                           ,&applicationInstance.mainThreadParam);
    if (SystemP_SUCCESS != error)
    {
        OSAL_printf("Error setting create thread of %s (%ld)\r\n", applicationInstance.mainThreadParam.name, error);
        OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
    }

    OSAL_startOs();

    OSAL_error(__func__, __LINE__, OSAL_ERR_InvalidParm, true, 1, "Not reachable by design!!!\r\n");

    // not reachable
    //EC_API_SLV_unLoad();

    return error;
}

static uint32_t EC_SLV_APP_CIA_remoteInit(EC_SLV_APP_CIA_Application_t *applicationInstance)
{
    uint32_t retVal = OSAL_CONTAINER_LOCALIMPLEMENTATION;

    if (!applicationInstance)
    {
        /* @cppcheck_justify{misra-c2012-15.1} goto is used to assure single point of exit */
        /* cppcheck-suppress misra-c2012-15.1 */
        goto Exit;
    }

#if (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
#if (defined FBTLIMPL_LINEUART) && (FBTLIMPL_LINEUART==1)
    retVal = SYSLIB_createLibInstanceLine(  FBTL_LINE_UART_NAME,
                                            FBTL_MAX_ASYNC_LEN, FBTL_MAX_ASYNC_LEN,
                                            FBTL_MAX_PD_LEN, FBTL_MAX_PD_LEN
#if (defined FBTLTRACECALLS) && (FBTLTRACECALLS==1)
                                      ,true
#else
                                      ,false
#endif
                                      ,&(applicationInstance->remoteHandle)
    );
#else
    retVal = SYSLIB_createLibInstanceShm(FBTLSHARED_MEM_NAME, FBTLSHARED_MEM_SIZE, false,
                                         FBTL_MAX_ASYNC_LEN, FBTL_MAX_ASYNC_LEN, FBTL_MAX_PD_LEN, FBTL_MAX_PD_LEN
#if (defined FBTLTRACECALLS) && (FBTLTRACECALLS==1)
            ,true
#else
            ,false
#endif
            ,&(applicationInstance->remoteHandle)
    );
#endif
#else
    OSALUNREF_PARM(applicationInstance);
    retVal = OSAL_CONTAINER_LOCALIMPLEMENTATION;
#endif

    if(OSAL_CONTAINER_LOCALIMPLEMENTATION == retVal)
    {
        OSAL_printf("\r\nLocal Implementation\r\n");
    }
#if (defined FBTL_REMOTE) && (FBTL_REMOTE==1)
    else if(EC_API_eERR_NONE == retVal)
    {
        retVal = EC_API_SLV_FBTL_configuration(applicationInstance->remoteHandle);
        if(EC_API_eERR_NONE != retVal)
        {
            OSAL_printf("%s:%d Error code: 0x%08x\r\n", __func__, __LINE__, retVal);
            OSAL_error(__func__, __LINE__, OSAL_STACK_INIT_ERROR, true, 0);
            /* @cppcheck_justify{misra-c2012-15.1} goto is used to assure single point of exit */
            /* cppcheck-suppress misra-c2012-15.1 */
            goto Exit;
        }
    }
#endif
    else
    {
        /* nothing to do */
        ;
    }

    Exit:
    if((OSAL_ERR_NoError != retVal) && (OSAL_CONTAINER_LOCALIMPLEMENTATION != retVal))
    {
        OSAL_printf("%s:%d::> err ret = 0x%x\r\n", __func__, __LINE__, retVal);
    }
    return retVal;
}
