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
 
#ifndef _DCL_COMMON_H_
#define _DCL_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \addtogroup DCL_API_MODULE APIs for Digital Control Library
 *  @{
 *  
 *  \file       dcl_common.h
 *  \brief      Top level header that contains all DCL common strctures and functions
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//! \brief  Defines single,double precision data type.
//!
//! \note   Note: Assumes ABI to be __TI_EABI__, 
//!         does not support legacy __TI_COFF__.
#ifndef IEEE754_TYPES
#define IEEE754_TYPES
typedef float         float32_t;
typedef double        float64_t;
#endif // IEEE754_TYPES

//! \brief  Defines the scope of dcl functions
#define _DCL_CODE_ACCESS   static inline

//! \brief  Defines the scope of critical dcl functions
//!
//! \note   For users wanting to map critial functions in a specific memory region, 
//!         uncomment "__attribute__((section("dclfuncs")))" and map 
//!         the memory directive "dclfuncs" in the .cmd linker script
#define _DCL_CRIT_ACCESS   static __attribute__((always_inline)) //__attribute__((section("dclfuncs")))

//! \brief  Defines volatile for DCL strctures
//!
//! \note   Define "DCL_VOLATILE_ENABLED" to enable violatile 
//!         (either in in dcl.h or user files that includes dcl.h)
#ifdef DCL_VOLATILE_ENABLED
    #define _DCL_VOLATILE volatile
#else
    #define _DCL_VOLATILE
#endif

//! \brief  Set a software breakpoint assembly instruction
//!
#if defined (__TMS320C28XX__)   //C28 ISA
    #define DCL_setBreakPoint() asm(" ESTOP")
#elif defined (__ARM_ARCH)      //ARM ISA
    #define DCL_setBreakPoint()  __asm(" bkpt #0")
#else
    #define DCL_setBreakPoint()
    #warning "DCL currently doesn't support break point for this architecture"
#endif

//! \brief  Define enable and disable interrupt operations
//!
#if defined (__TMS320C28XX__)
    #define DCL_disableInts()   __disable_interrupts()
    #define DCL_restoreInts(v)  if (0U == (v & 0x1)) __enable_interrupts()   
    typedef uint16_t            dcl_interrupt_t;
#elif defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263X)
    #include <kernel/dpl/HwiP.h>
    #define DCL_disableInts()   HwiP_disable()
    #define DCL_restoreInts(v)  HwiP_restore(v)
    typedef uint32_t            dcl_interrupt_t;
#else 
    #define DCL_disableInts()   0
    #define DCL_restoreInts(v) 
    typedef uint32_t            dcl_interrupt_t;
    #warning "DCL currently doesn't support interrupt operations for this architecture"
#endif

#include "common/dcl_macro.h"
#include "common/dcl_css.h"
#include "common/dcl_zpk3.h"
#include "common/dcl_clamp.h"
#include "common/dcl_stability.h"

/** @} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // _DCL_COMMON_H_
