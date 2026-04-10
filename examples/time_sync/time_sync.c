/*
 *  Copyright (C) 2025-2026 Texas Instruments Incorporated
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


/* Main synchronization state machine
 * - Captures new timestamp from master
 * - Calculates current offset
 * - Updates moving averages
 * - Applies compensation based on state
 * - Manages transitions between sync states
 */

/* State transitions:
 * RESET -> FIRST_ADJUSTMENT_DONE : After initial sync
 * FIRST_ADJUSTMENT_DONE -> SLOW_COMPENSATION : Start drift compensation
 * SLOW_COMPENSATION -> TIMER_IN_SYNC : When offset stabilizes and rate difference converges
 * Any state -> RESET : If offset exceeds threshold
 */

#include <time_sync.h>
#include <drivers/pruicss.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include "ti_drivers_config.h"
#include <stdlib.h>


#ifndef TIME_TRANSMITTER_RECEIVER
extern TimesyncHandle timesyncHandle0;
#endif
extern TimesyncHandle timesyncHandle1;

#ifdef ENABLE_DEBUG_GPIO
uint32_t gpioBaseAddr, pinNum;
#endif

void timesync_run(TimesyncHandle handle)
{
    uint32_t iepBaseAddress = handle->iepBaseAddress;
    uint64_t expectedTimestamp;
    uint32_t timeElapsed;
    uint8_t count;
    int32_t avgCorrection;
    uint32_t index;

#ifdef ENABLE_DEBUG_LOGS
    TimesyncDebug *timesyncDebugPtr = handle->timesyncDebugPtr;
#endif

    /* disallow nesting of interrupts */
    HwiP_disable();

    /* Wait for the IEP0 latch timestamp from capture with 0 sleep*/
    handle->timeOutErrorCount += timesync_wait_iep_latch0_event(iepBaseAddress, 0);

    /* Read ICSSG0 IEP0 Latch timestamp */
    handle->currentTimestamp = timesync_read_latch_input(iepBaseAddress);

    if(handle->state == TIMESYNC_STATE_RESET)
    {
        /* FIRST SYNC EVENT after start or restart */
        /* Store the initial count into IEP timer */
        timeElapsed = (timesync_read_iep_count(iepBaseAddress) - handle->currentTimestamp) + handle->processingDelay;
        timesync_do_first_adjustment(iepBaseAddress, timeElapsed);

        handle->state = TIMESYNC_STATE_FIRST_ADJUSTMENT_DONE;

        /* configure compare 1 with sync period after first sync and enable compare events */
        HW_WR_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CMP1_REG0),(SYNC_PERIOD_IN_NS));
        HW_WR_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG), 0x1FFFE);
        HW_WR_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_CTRL_REG), 0x0003);
    }
    else if (handle->state == TIMESYNC_STATE_FIRST_ADJUSTMENT_DONE)
    {
        /* FIRST SYNC EVENT after initial adjustment */
        /* Just store current timestamp and begin slow compensation from next cycle */
        handle->state = TIMESYNC_STATE_SLOW_COMPENSATION;
    }
    else if((handle->state == TIMESYNC_STATE_SLOW_COMPENSATION) || (handle->state == TIMESYNC_STATE_TIMER_IN_SYNC))
    {
        expectedTimestamp = handle->prevTimestamp + SYNC_PERIOD_IN_NS;

        handle->currOffset = (int32_t)((int64_t)(expectedTimestamp) - (int64_t)(handle->currentTimestamp));

        /*Take running average of the offset*/
        if(handle->ltaOffsetValid)
        {
            handle->ltaOffset = (int32_t)((double)(FILTER_ALPHA_COEFF) * (double)handle->ltaOffset + (double)(1 - FILTER_ALPHA_COEFF) * (double)handle->currOffset);
        }
        else
        {
            handle->ltaOffset = handle->currOffset;
            handle->ltaOffsetValid = 1;
        }

        if(handle->driftStable)
        {
            handle->adjOffset = handle->currOffset + handle->ltaOffset + handle->initialOffset;
        }
        else
        {
            handle->adjOffset = handle->currOffset;
        }

        timesync_adjust_slow_compensation(handle, handle->adjOffset);

        if((abs(handle->currOffset) > OFFSET_THRESHOLD_FOR_RESET) && (handle->state == TIMESYNC_STATE_TIMER_IN_SYNC))
        {
            handle->resetCount++;
            timesync_reset(handle);
#ifdef ENABLE_DEBUG_LOGS
            timesync_debug_reset(timesyncDebugPtr);
#endif
            return;
        }
        /* Find clock drift */
        if(handle->prevOffsetValid[0])
        {
            handle->clockDrift += abs(handle->currOffset - handle->prevOffset[0]);
            handle->clockDrift = handle->clockDrift / 2;
        }

        /* Set prevoffset to current once drift is computed*/
        if(handle->prevOffsetValid[0])
        {
            handle->prevOffset[1] = handle->prevOffset[0];
            handle->prevOffsetValid[1] = 1;
        }

        handle->prevOffset[0] = handle->currOffset;
        handle->prevOffsetValid[0] = 1;

        /* Wait for the drift to stabilize */
        if(handle->clockDrift <= STABLE_FILTER_THRESHOLD && (handle->driftStable == 0))
        {
            handle->driftStable = 1;
            handle->initialOffset = handle->currOffset;
#ifdef ENABLE_DEBUG_LOGS
            timesyncDebugPtr->MinitialOffset = handle->initialOffset;
#endif
        }

        /* Wait for offset to become zero */
        if(abs(handle->ltaOffset) <= STABLE_FILTER_THRESHOLD && (handle->offsetStable == 0) && (handle->driftStable))
        {
            handle->offsetStable = 1;
            /*Indicate that the device is in sync now*/
            handle->state = TIMESYNC_STATE_TIMER_IN_SYNC;
        }
    }
#ifdef ENABLE_DEBUG_LOGS
        if(handle->currOffset > timesyncDebugPtr->MmaxOffset)
        {
            timesyncDebugPtr->MmaxOffset = handle->currOffset;
        }
        if(handle->currOffset < timesyncDebugPtr->MminOffset)
        {
            timesyncDebugPtr->MminOffset = handle->currOffset;
        }
        timesyncDebugPtr->McurrOffset[timesyncDebugPtr->Mindex] = (uint32_t)(handle->currOffset);
        timesyncDebugPtr->MadjOffset[timesyncDebugPtr->Mindex] = (handle->adjOffset);
        timesyncDebugPtr->MltaOffset[timesyncDebugPtr->Mindex] = (uint32_t)(handle->ltaOffset);
        timesyncDebugPtr->MdriftStable[timesyncDebugPtr->Mindex] = handle->driftStable;
        timesyncDebugPtr->MoffsetStable[timesyncDebugPtr->Mindex] = handle->offsetStable;
        timesyncDebugPtr->McurrTimestamp[timesyncDebugPtr->Mindex] = handle->currentTimestamp;
        timesyncDebugPtr->MprevTimestamp[timesyncDebugPtr->Mindex] = handle->prevTimestamp;
        timesyncDebugPtr->MdiffTimestamp[timesyncDebugPtr->Mindex] = handle->currentTimestamp - handle->prevTimestamp;
        timesyncDebugPtr->Mstate[timesyncDebugPtr->Mindex] = handle->state;

        timesyncDebugPtr->Mindex++;
        if(timesyncDebugPtr->Mindex == MSIZE)
        {
            timesyncDebugPtr->Mindex = 0;
        }
#endif

    handle->prevTimestamp = handle->currentTimestamp;

    /*Run the logic only once we are stable*/
    if(handle->offsetStable)
    {
        handle->lastSeen_good_drift_index++;

        /*If the drift is below our threshold and we have a close cluster
         * then we use this value for our averaging purpose
         */
        if((handle->clockDrift <
                TIMESYNC_OFFSET_STABLE_ALGO_THRESHOLD)
                && (handle->lastSeen_good_drift_index <
                    OFFSET_ALGO_CLUSTER_SIZE))
        {
            handle->lastSeen_good_drift_index = 0;

            if(handle->num_entries_index < OFFSET_ALGO_BIN_SIZE)
            {
                /*Store the value for averaging later*/
                index = handle->num_entries_index++;
                handle->correction[index] = handle->currOffset;
            }

            else
            {
                /*get average of all offsets*/
                avgCorrection = 0;
                for(count = 0; count < OFFSET_ALGO_BIN_SIZE; count++)
                {
                    avgCorrection += handle->correction[count];
                }

                avgCorrection /= OFFSET_ALGO_BIN_SIZE;
                /*add the new correction value to initial offset or the PPM value*/
                handle->initialOffset += avgCorrection;
                handle->num_entries_index = 0;
            }

        }
        /*If cluster is broken then we reset the counters*/
        else if((handle->lastSeen_good_drift_index >=
                 OFFSET_ALGO_CLUSTER_SIZE))
        {
            /*reset the count if we can't find a good match in our window*/
            handle->lastSeen_good_drift_index = 0;
            handle->num_entries_index = 0;
        }
    }
    HwiP_enable();
    return;
}

TimesyncHandle timesync_init(TimesyncParams *params, uint32_t iepBaseAddress)
{
    TimesyncHandle handle;

    DebugP_assert(params != NULL);

#ifdef ENABLE_DEBUG_GPIO
    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(TIMESYNC_DEBUG_BASE_ADDR);
    pinNum       = TIMESYNC_DEBUG_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, TIMESYNC_DEBUG_DIR);
#endif

    handle = (TimesyncHandle)params;
    timesync_reset(handle);
    handle->iepBaseAddress = iepBaseAddress;
    handle->resetCount = 0;

    /* Enable capture 6 in PRU ICSSGx IEP0 for LATCH_IN0*/
    timesync_enable_latch(iepBaseAddress);

    return params;
}

uint8_t timesync_wait_iep_latch0_event(uint32_t iepBaseAddress, uint32_t sleepTime)
{
    volatile uint32_t capRegValue = 0;
    uint32_t timeoutCount = 0;
    const uint32_t timeoutThreshold = LATCH_EVENT_TIMEOUT; /* Adjust this value based on your system's timing requirements */

    /* Loop until the latch condition is met or a timeout occurs */
    while (1)
    {
        /* Read the capture register */
        capRegValue = HW_RD_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAP_STATUS_REG));

        /* Check if the latch condition is met */
        if (capRegValue & (1 << 6))
        {
            break; /* Exit the loop if the latch condition is met */
        }

        if(0 != sleepTime)
        {
            ClockP_usleep(sleepTime);
        }

        /* Increment the timeout counter */
        timeoutCount++;

        /* Check for timeout */
        if (timeoutCount >= timeoutThreshold)
        {
            /* Handle timeout condition (e.g., log an error, reset the system, etc.) */
            DebugP_log("Timeout waiting for IEP0 latch timestamp\n");
            return 1;
        }
    }
    return 0;
}

void timesync_enable_latch(uint32_t iepBaseAddress)
{
    uint32_t captureCtrlRegValue = 0;

    /* Read the current value of the capture control register */
    captureCtrlRegValue = HW_RD_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG));

    /*Enable capture 6 by setting the corresponding bit */
    captureCtrlRegValue |= (1 << 6);

    /* Write the updated value back to the capture control register */
    HW_WR_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG), captureCtrlRegValue);

    /* Read the current value of the capture control register */
    captureCtrlRegValue = HW_RD_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG));

    /*Enable capture 6 by setting the corresponding bit */
    captureCtrlRegValue |= (1 << 6);

    /* Write the updated value back to the capture control register */
    HW_WR_REG32((uint32_t)(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAP_CFG_REG), captureCtrlRegValue);

    return;
}

void timesync_adjust_slow_compensation(TimesyncHandle handle, int32_t adjOffset)
{
    uint32_t iepBaseAddress = handle->iepBaseAddress;
    uint32_t compensationPeriod = 0;
    uint32_t compensationIncrementValue;
    volatile uint64_t timeElapsed = 0;
    uint32_t iepGlobalConfigValue = ((handle->iepIncrementValue << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) | 1);

#ifdef ENABLE_DEBUG_LOGS
    TimesyncDebug *timesyncDebugPtr = handle->timesyncDebugPtr;
#endif

    if(adjOffset != 0)
    {
        /* set compensation interval = (sync interval - time elapsed since last sync interval)/drift */
        timeElapsed = timesync_read_iep_count(iepBaseAddress) - handle->currentTimestamp;
        compensationPeriod = (uint32_t)((double)(SYNC_PERIOD_IN_NS - timeElapsed)/(double)(abs(adjOffset)));
    }

#ifdef ENABLE_DEBUG_LOGS
        timesyncDebugPtr->MtimeElapsedSinceSync[timesyncDebugPtr->Mindex] = timeElapsed;
        timesyncDebugPtr->McompensationPeriod[timesyncDebugPtr->Mindex] = compensationPeriod;
#endif

    /* NOTE: This adjustment assumes default IEP increment count is 5, which is corresponding to 200 MHz IEP Clock */
    if (adjOffset == 0) /* No compensation required */
    {
        /* set compensation increment = 5ns (default val) */
        compensationIncrementValue = ((handle->iepIncrementValue)<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
    }
    else if (adjOffset > 0) /* master is faster */
    {
        /* set compensation increment = 10ns */
        compensationIncrementValue = ((handle->iepIncrementValue*2)<<CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CMP_INC_SHIFT);
    }
    else /* slave is faster */
    {
        /* set compensation increment = 0ns */
        compensationIncrementValue = 0;
    }
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG,  compensationIncrementValue | iepGlobalConfigValue);
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, compensationPeriod);
    return;
}

void timesync_reset(TimesyncHandle handle)
{
    uint32_t iepBaseAddress;
    uint32_t iepGlobalConfigValue;

    DebugP_assert(handle != NULL);

    iepBaseAddress = handle->iepBaseAddress;
    iepGlobalConfigValue = ((handle->iepIncrementValue << CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT) | 1);

    /* set compensation increment = 5ns (default val) */
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG, iepGlobalConfigValue);
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_SLOW_COMPEN_REG, 0);

    handle->currentTimestamp = 0;
    handle->prevTimestamp = 0;
    handle->currOffset = 0;
    handle->prevOffset[0] = 0;
    handle->prevOffset[1] = 0;
    handle->prevOffsetValid[0] = 0;
    handle->prevOffsetValid[1] = 0;
    handle->initialOffset = 0;
    handle->driftStable = 0;
    handle->offsetStable = 0;
    handle->ltaOffset = 0;
    handle->ltaOffsetValid = 0;
    handle->adjOffset = 0;
    handle->clockDrift = 0;
    handle->state = TIMESYNC_STATE_RESET;
    handle->timeOutErrorCount = 0;
}

void timesync_debug_reset(TimesyncDebug *timesyncDebugPtr)
{
    if (timesyncDebugPtr == NULL) {
        return;
    }
    timesyncDebugPtr->Mindex = 0;
    timesyncDebugPtr->MinitialOffset = 0;
    timesyncDebugPtr->MmaxOffset = 0;
    timesyncDebugPtr->MminOffset = INT32_MAX;
}

void timesync_do_first_adjustment(uint32_t iepBaseAddress, uint64_t initialCount)
{
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0, initialCount & LOW32_MASK);
    HW_WR_REG32(iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG1, (initialCount & HIGH32_MASK) >> 32);
}

volatile uint64_t timesync_read_latch_input(uint32_t iepBaseAddress)
{
    uint32_t capr6LowOffset = iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_CAPR6_REG0;
    volatile uint64_t currentTimestamp;
    currentTimestamp = (uint64_t)(HW_RD_REG32((capr6LowOffset + 4)));
    currentTimestamp <<= 32;
    currentTimestamp += (HW_RD_REG32(capr6LowOffset));

    return currentTimestamp;
}

volatile uint64_t timesync_read_iep_count(uint32_t iepBaseAddress)
{
    uint32_t lowOffset = iepBaseAddress + CSL_ICSS_G_PR1_IEP0_SLV_COUNT_REG0;
    uint64_t currentTimestamp;
    volatile uint64_t lowByte, highByte, highByte2, lowByte2;

    highByte = (volatile uint64_t)HW_RD_REG32((lowOffset + 4));
    lowByte = (volatile uint64_t)HW_RD_REG32(lowOffset);

    highByte2 = (volatile uint64_t)HW_RD_REG32((lowOffset + 4));

    if(highByte2 != highByte)
    {
        lowByte2 = (volatile uint64_t)HW_RD_REG32(lowOffset);
        currentTimestamp = ((highByte2 << 32) | lowByte2);
    }
    else
    {
        currentTimestamp = ((highByte << 32) | lowByte);
    }

    return currentTimestamp;
}
