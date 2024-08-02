/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/* Following is the list of the Build Level choices */
#define  OPEN_LOOP_IQ_ID        1           /* Open loop Iq, Id */
#define  CLOSED_LOOP_IQ_ID      2           /* Closed loop Iq with reference array, Id closed loop to 0.0 */
#define  CLOSED_LOOP_SPEED      3           /* Closed loop Speed */
#define  CLOSED_LOOP_POSITION   4           /* Closed loop Position */
#define  CLOSED_LOOP_CIA402     5           /* Closed loop Iq, Speed, or Position based on EtherCAT CiA402 */

#define  PID_TUNING             1           /* Enable the runtime PID tuning code */
#define  NO_TUNING              2           /* Disabe the runtime PID tuning, tuning has already occurred */

#define  DEBUG_BUFFERS_ON       1           /* Log data into the debug buffers */
#define  DEBUG_BUFFERS_OFF      2           /* No data logging */

#define  DEBUG_BUFFER_WRAP      1           /* Wrap the debug buffer continuously */
#define  DEBUG_BUFFER_SINGLE    2           /* Fill the debug buffer once to focus on startup conditions */

#define  PRECOMPUTE_CLARKE      1           /* Use the SDDF ISR to convert/scale phase currents and compute Clarke Transform */
#define  NO_PRECOMPUTE          2           /* Convert/scale and compute Clarke Transform in the EnDat ISR along with the FOC loop */

#define  BUILDLEVEL             CLOSED_LOOP_SPEED ///CLOSED_LOOP_CIA402 ///CLOSED_LOOP_SPEED ///CLOSED_LOOP_IQ_ID ///OPEN_LOOP_IQ_ID
#define  PID_TUNE_LEVEL         NO_TUNING
#define  DEBUG_LEVEL            DEBUG_BUFFERS_OFF
#define  DEBUG_WRAP_TYPE        DEBUG_BUFFER_WRAP
#define  PRECOMPUTE_LEVEL       NO_PRECOMPUTE

/* The number of ISR cycles to use the same target from the list of 8 targets */
#define  CYCLES_PER_TARGET      600
#define  TARGET_BUFF_SIZE       CYCLES_PER_TARGET * 8   /* This value needs to be less than DEBUG_BUFF_SZ (65536) if DEBUG_BUFFERS_ON */

/* Iq testing value for open loop */
#define  INIT_PWM_DIFF          0.05 ///0.1
#define  IQ_TESTING             0.3 ///0.2
#define  ID_TESTING             0.0

/* Max speed change request allowed between updates */
#define  MAX_SPD_CHANGE         0.12 ///0.12
/* Max position change request allowed between updates */
#define  MAX_POS_CHANGE         0.06        /* 500RPMs max speed movement between positions at 100KHz update rate */

#define MAX_SPD_RPM             120 ///500

#define  ISR_PRD_IN_SECONDS     0.00002     /* 50KHz @ 1x update is 20us */
#define  ISR_PRD_IN_MINUTES     (ISR_PRD_IN_SECONDS / 60.0)

/* PRU event to clear for the EnDat interrupt */
#define  ENDAT_EVENT            ( 18 )    /* 18 (EVT) for 120 (INT#) pr0_pru_mst_intr[4]_intr_req */

/* Sigma Delta definitions for SINC3 OSR64 - 0 - 2^18 */
#define  SDDF_FULL_SCALE        262144
#define  SDDF_HALF_SCALE        131072
#define  SDDF_HALF_SCALE_FLT    131072.0

/* Intial settling counts for SDDF and Electrical Angle offset calculations */
#define  SETTLING_COUNT         8192.0
#define  OFFSET_FINISHED        (uint32_t) (SETTLING_COUNT * 2.0)

#define CYCLIC_SYNC_POSITION_MODE   8 /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE   9 /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE     10/**< \brief Cyclic Synchronous Torque mode*/

#define SINGLE_AXLE_USE_M1 /* for R5F_0_0, it controls the M1 */
#define DUAL_AXIS_USE_M1_M2 /* for R5F_0_0, it controls the M1. for R5F_0_1, it controls the M2 */

///#define SINGLE_AXLE_USE_M2 /* for R5F_0_1, it controls the M2 */

#define CORE_R5F_0_0

#define SDDF_HW_EVM_ADAPTER_BP
///#define AM243X_ALV
#define AM243X_ALX

///#define USE_PURE_OPEN_LOOP
///#define USE_OPEN_LOOP_WITH_SDDF


