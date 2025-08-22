/*!
 *  \example project.h
 *
 *  \brief
 *  EtherCAT_SubDevice_Simple project defines example.
 *
 *  \author
 *  Texas Instruments Incorporated
 *
 *  \copyright
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

#if !(defined PROTECT_PROJECT_H)
#define PROTECT_PROJECT_H 1

/* @cppcheck_justify{misra-c2012-21.6} we need stdio for snprintf */
/* cppcheck-suppress misra-c2012-21.6 */
#include <stdio.h>

/* Brief description of purpose and functionality*/

#define EC_MAX_PD_LEN                       1024
#define EC_DEFAULT_PD_LEN                   256

#define EC_REVISION                         0x00010000
#define EC_DEVICE_SERIALNUMBER              0xFFFFFFFF //replace with serial number from uboot

#define ESC_EE_PDI_CONTROL                  0x0C80 // 0x80 = On Chip PDI Type
#define ESC_EE_PDI_CONFIG                   0x88E0 // ?? copy from TI

#define COE_SDO_INFO_SUPPORTED              0x02
#define COE_PDO_ASSIGN_SUPPORTED            0x04
#define COE_PDO_CONFIG_SUPPORTED            0x08
#define COE_PDO_UPLOAD_SUPPORTED            0x10
#define COE_SDO_COMPLETE_ACCESS_SUPPORTED   0x20

#define EC_BOOTSTRAP_MBXOUT_START           0x1000
#define EC_BOOTSTRAP_MBXOUT_DEF_LENGTH      256

#define EC_BOOTSTRAP_MBXIN_START            0x1200
#define EC_BOOTSTRAP_MBXIN_DEF_LENGTH       256

#define EC_MBXOUT_START                     0x1000
#define EC_MBXOUT_DEF_LENGTH                256
#define EC_MBXOUT_CONTROLREG                0x26
#define EC_MBXOUT_ENABLE                    1

#define EC_MBXIN_START                      EC_MBXOUT_START + EC_MBXOUT_DEF_LENGTH
#define EC_MBXIN_DEF_LENGTH                 256
#define EC_MBXIN_CONTROLREG                 0x22
#define EC_MBXIN_ENABLE                     1

#define EC_OUTPUT_START                     EC_MBXIN_START + EC_MBXIN_DEF_LENGTH
#define EC_OUTPUT_CONTROLREG                0x64
#define EC_OUTPUT_DEF_LENGTH                (EC_MAX_PD_LEN * 3) //in Bytes
#define EC_OUTPUT_ENABLE                    1

#define EC_INPUT_START                      EC_OUTPUT_START + EC_OUTPUT_DEF_LENGTH
#define EC_INPUT_CONTROLREG                 0x20
#define EC_INPUT_DEF_LENGTH                 (EC_MAX_PD_LEN * 3) //in Bytes
#define EC_INPUT_ENABLE                     1

/* pru/ecat address settings of shared ram */
#define MIN_PD_READ_ADDRESS                 0x1000      // ti register offset
#define MIN_PD_WRITE_ADDRESS                0x1000
#define MIN_MBX_WRITE_ADDRESS               0x1000
#define MIN_MBX_READ_ADDRESS                0x1000

/*#define   OS_TICKS_IN_MILLI_SEC   10 // 100us tick */
#define OS_TICKS_IN_MILLI_SEC               1 /* 1000us tick */
/* task priorities */
#ifndef BSP_OS_MAX_TASKS
#define BSP_OS_MAX_TASKS                    20
#endif

#define KBECSLV_PRIO_PDI                    OSAL_TASK_Prio_ECPDI
#define KBECSLV_PRIO_SYNC0                  OSAL_TASK_Prio_ECSync
#define KBECSLV_PRIO_SYNC1                  OSAL_TASK_Prio_ECSync
#define KBECSLV_PRIO_EOE                    OSAL_TASK_Prio_ECEoE
#define KBECSLV_PRIO_LED                    OSAL_TASK_Prio_ECLED

/* task stack sizes in bytes */
#define KBECSLV_STACKSIZE_PDI               0x0800
#define KBECSLV_STACKSIZE_LED               0x0400
#define KBECSLV_STACKSIZE_SYNC0             0x0800
#define KBECSLV_STACKSIZE_SYNC1             0x0800

#define EEPROM_MAGIC_KEY \
    /* @cppcheck_justify{misra-c2012-11.6} void cast required for signature */ \
    /* cppcheck-suppress misra-c2012-11.6 */ \
    ((void*)0xEE11AA44u)

#if (defined __cplusplus)
extern "C" {
#endif

/* extern void func(void); */

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_PROJECT_H */