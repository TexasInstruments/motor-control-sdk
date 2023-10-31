
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=16384
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=32768
-e_vectors  /* This is the entry of the application, _vector MUST be plabed starting address 0x0 */

/* This is the size of stack when R5 is in IRQ mode
 * In NORTOS,
 * - Here interrupt nesting is disabled as of now
 * - This is the stack used by ISRs registered as type IRQ
 * In FreeRTOS,
 * - Here interrupt nesting is enabled
 * - This is stack that is used initally when a IRQ is received
 * - But then the mode is switched to SVC mode and SVC stack is used for all user ISR callbacks
 * - Hence in FreeRTOS, IRQ stack size is less and SVC stack size is more
 */
__IRQ_STACK_SIZE = 8192;
/* This is the size of stack when R5 is in IRQ mode
 * - In both NORTOS and FreeRTOS nesting is disabled for FIQ
 */
__FIQ_STACK_SIZE = 256;
__SVC_STACK_SIZE = 8192; /* This is the size of stack when R5 is in SVC mode */
__ABORT_STACK_SIZE = 256;  /* This is the size of stack when R5 is in ABORT mode */
__UNDEFINED_STACK_SIZE = 256;  /* This is the size of stack when R5 is in UNDEF mode */

SECTIONS
{
    /* This has the R5F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > R5F_VECS

    /* This has the R5F boot code until MPU is enabled,  this MUST be at a address < 0x80000000
     * i.e this cannot be placed in DDR
     */
    GROUP {
        .text.hwi: palign(8)
        .text.cache: palign(8)
        .text.mpu: palign(8)
        .text.boot: palign(8)
    } > MSRAM_0_1

    /* This is rest of code. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .text:   {} palign(8)   /* This is where code resides */
        .rodata: {} palign(8)   /* This is where const's go */
    } > MSRAM_0_1

    /* This is rest of initialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
    } > MSRAM_0_1

    /* This is rest of uninitialized data. This can be placed in DDR if DDR is available and needed */
    GROUP {
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)
		RUN_END(__BSS_END)
        .sysmem: {} palign(8)   /* This is where the malloc heap goes */
        .stack:  {} palign(8)   /* This is where the main() stack goes */
    } > MSRAM_0_1

    /* This is where the stacks for different R5F modes go */
    GROUP {
        .irqstack: {. = . + __IRQ_STACK_SIZE;} align(8)
        RUN_START(__IRQ_STACK_START)
        RUN_END(__IRQ_STACK_END)
        .fiqstack: {. = . + __FIQ_STACK_SIZE;} align(8)
        RUN_START(__FIQ_STACK_START)
        RUN_END(__FIQ_STACK_END)
        .svcstack: {. = . + __SVC_STACK_SIZE;} align(8)
        RUN_START(__SVC_STACK_START)
        RUN_END(__SVC_STACK_END)
        .abortstack: {. = . + __ABORT_STACK_SIZE;} align(8)
        RUN_START(__ABORT_STACK_START)
        RUN_END(__ABORT_STACK_END)
        .undefinedstack: {. = . + __UNDEFINED_STACK_SIZE;} align(8)
        RUN_START(__UNDEFINED_STACK_START)
        RUN_END(__UNDEFINED_STACK_END)
    } > MSRAM_0_1

    /* General purpose user shared memory, used in some examples */
    .bss.user_shared_mem (NOLOAD) 	: {} > USER_SHM_MEM
    /* this is used when Debug log's to shared memory are enabled, else this is not used */
    .bss.log_shared_mem  (NOLOAD) 	: {} > LOG_SHM_MEM
    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) 	: {} > RTOS_NORTOS_IPC_SHM_MEM

    .gSddfChSampsRaw (NOLOAD)       : {} align(4) > R5F_TCMB0_SDDF_0_1
    .gTxDataSection					: {} align(4) > R5F_TCMB0_PDO
    .gRxDataSection					: {} align(4) > OTHER_R5F_TCMB0_PDO

    // debug, capture buffers
    .gDebugBuff1    : {} align(4) > MSRAM1
    .gDebugBuff2    : {} align(4) > MSRAM2

    .gEnDatChInfo (NOLOAD)       : {} align(4) > R5F_TCMB0_ENC_0_1

    .critical_code		: {} palign(4) > R5F_TCMB0
    .critical_data		: {} palign(8) > R5F_TCMB0
    
    .gCtrlVars          : {} palign(8) > MSRAM_CTRL_VARS_0_1

    .gSharedPrivStep (NOLOAD)    : {} palign(8) > USER_SHM_MEM
    .gSharedPruHandle (NOLOAD)   : {} palign(8) > USER_SHM_MEM
    .gEtherCatCia402 (NOLOAD)	: {} palign(4) > MSRAM_NO_CACHE
}

/*
NOTE: Below memory is reserved for DMSC usage
 - During Boot till security handoff is complete
   0x701E0000 - 0x701FFFFF (128KB)
 - After "Security Handoff" is complete (i.e at run time)
   0x701FC000 - 0x701FFFFF (16KB)

 Security handoff is complete when this message is sent to the DMSC,
   TISCI_MSG_SEC_HANDOVER

 This should be sent once all cores are loaded and all application
 specific firewall calls are setup.
*/

MEMORY
{
    R5F_VECS  : ORIGIN = 0x00000000 , LENGTH = 0x00000040
    R5F_TCMA  : ORIGIN = 0x00000040 , LENGTH = 0x00007FC0
    R5F_TCMB0_SDDF_0_0 : ORIGIN = 0x078100000 , LENGTH = 0x80
    R5F_TCMB0_SDDF_0_1 : ORIGIN = 0x078100080 , LENGTH = 0x80
    R5F_TCMB0_ENC	: ORIGIN = 0x41010100 , LENGTH = 0x100
    R5F_TCMB0_ENC_0_1	: ORIGIN = 0x078100100 , LENGTH = 0x100
    R5F_TCMB0_PDO	: ORIGIN = 0x41010200 , LENGTH = 0x100
    R5F_TCMB0 : ORIGIN = 0x41010300 , LENGTH = 0x00008000 - 0x300
    /* when using multi-core application's i.e more than one R5F/M4F active, make sure
     * this memory does not overlap with other R5F's
     */
    /* MSRAM0     : ORIGIN = 0x70000000 , LENGTH = 0x40000 */
    MSRAM1     : ORIGIN = 0x70040000 , LENGTH = 0x40000
    MSRAM2     : ORIGIN = 0x70080000 , LENGTH = 0x40000
    MSRAM_0_0	  : ORIGIN = 0x70140000 , LENGTH = 0x40000
    MSRAM_0_1	  : ORIGIN = 0x70180000 , LENGTH = 0x40000 - 0x200
    MSRAM_1_0	  : ORIGIN = 0x700C0000 , LENGTH = 0x70000
    MSRAM_CTRL_VARS_0_0 : ORIGIN = 0x701BFE00 , LENGTH = 0x100
    MSRAM_CTRL_VARS_0_1 : ORIGIN = 0x701BFF00 , LENGTH = 0x100
    MSRAM_NO_CACHE : ORIGIN = 0x701C0000, LENGTH = 0x100

    /* shared memories that are used by RTOS/NORTOS cores */
    /* On R5F,
     * - make sure there is a MPU entry which maps below regions as non-cache
     */
    USER_SHM_MEM            : ORIGIN = 0x701D0000, LENGTH = 0x00004000
    LOG_SHM_MEM             : ORIGIN = 0x701D4000, LENGTH = 0x00004000
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0x701D8000, LENGTH = 0x00008000

    OTHER_R5F_TCMB0_PDO		: ORIGIN = 0x78500000, LENGTH = 0x100
}
