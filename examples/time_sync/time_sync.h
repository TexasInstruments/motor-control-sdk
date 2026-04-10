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

#include <stdint.h>

/* Enable/disable debug logs and debug GPIO */
#define ENABLE_DEBUG_LOGS
#define ENABLE_DEBUG_GPIO

/**
 * \brief State definitions for time synchronization state machine
 *
 * Defines the possible states during the synchronization process:
 * - RESET: Initial state or after sync loss
 * - FIRST_ADJUSTMENT_DONE: Initial sync completed
 * - SLOW_COMPENSATION: Normal operation with drift compensation
 * - TIMER_IN_SYNC: System is synchronized with master
 */
#define TIMESYNC_STATE_RESET                    (0U) /**< Initial/reset state */
#define TIMESYNC_STATE_FIRST_ADJUSTMENT_DONE    (1U) /**< First sync completed */
#define TIMESYNC_STATE_SLOW_COMPENSATION        (2U) /**< Applying drift compensation */
#define TIMESYNC_STATE_TIMER_IN_SYNC            (3U) /**< Stable synchronized state */

/*IMPORTANT NOTE: Please modify following macros as per system use case*/

/* TIMESYNC Configuration Macros */

/**
 * Sync period - time in nano-seconds
 */
#define SYNC_PERIOD_IN_NS (100 * 1000U) /* 100 us */

/* Filter coefficient for exponential moving average filter
 * Range: 0.0 to 1.0
 * Higher value = more weight to historical data
 * Lower value = more weight to new measurements
 */
#define FILTER_ALPHA_COEFF          (0.80)

/* Maximum allowed offset (in nanoseconds) from master before triggering reset
 * If slave's clock differs by more than this threshold,
 * timesync mechanism will reset and resynchronize
 */
#define OFFSET_THRESHOLD_FOR_RESET  (10 * 1000U) /* 10 us*/

/* Threshold to determine when drift and offset have stabilized
 * System is considered stable when variations are below this value
 */
#define STABLE_FILTER_THRESHOLD         100

/**
 * \def Default drift threshold used in offset stabilization algo
 */
#define  TIMESYNC_OFFSET_STABLE_ALGO_THRESHOLD              15

/**
 * Sync period - time in IEP cycles
 */
#define SYNC_PERIOD_IN_IEP_CYCLES (SYNC_PERIOD_IN_NS/CONFIG_PRU_ICSS1_IEP_CLK_PERIOD_NSEC)


/* Timeout value for waiting for the latch event */
#define LATCH_EVENT_TIMEOUT (UINT32_MAX)

/* Size of array used for debugging */
#define MSIZE 1000

/* Masks for extracting high and low 32 bits from a 64-bit value */
#define HIGH32_MASK 0xFFFFFFFF00000000
#define LOW32_MASK  0x00000000FFFFFFFF

/**
 * \def OFFSET_ALGO_BIN_SIZE
 *      Bin size used for stabilization algo
 *      See design doc for details
 */
#define OFFSET_ALGO_BIN_SIZE    5

/**
 * \def OFFSET_ALGO_CLUSTER_SIZE
 *      Number of entries used for clustering
 */
#define OFFSET_ALGO_CLUSTER_SIZE   3


/**
 * \brief Debug information structure for time synchronization monitoring
 *
 * Maintains historical data and statistics for analyzing synchronization behavior.
 * Uses circular buffers of size MSIZE to store temporal data.
 */
typedef struct {
    uint32_t    Mindex;                         /**< Current index in circular debug buffers */
    int32_t     MinitialOffset;                 /**< Initial offset captured when drift stabilized */
    int32_t     MmaxOffset;                     /**< Maximum offset observed since last reset */
    int32_t     MminOffset;                     /**< Minimum offset observed since last reset */
    int32_t     McurrOffset[MSIZE];             /**< Circular buffer of measured offsets */
    int32_t     MadjOffset[MSIZE];              /**< Circular buffer of applied adjustments */
    int32_t     MltaOffset[MSIZE];              /**< Circular buffer of long-term average offsets */
    uint8_t     MdriftStable[MSIZE];            /**< History of drift stability status */
    uint8_t     MoffsetStable[MSIZE];           /**< History of offset stability status */
    uint64_t    McurrTimestamp[MSIZE];          /**< Circular buffer of captured timestamps */
    uint64_t    MprevTimestamp[MSIZE];          /**< Circular buffer of previous timestamps */
    uint64_t    MdiffTimestamp[MSIZE];          /**< Circular buffer of timestamp differences */
    uint32_t    Mstate[MSIZE];                  /**< History of state machine transitions */
    uint64_t    MtimeElapsedSinceSync[MSIZE];   /**< Time elapsed since last sync event */
    uint32_t    McompensationPeriod[MSIZE];     /**< History of calculated compensation periods */
} TimesyncDebug;


/**
 * \brief Core parameters and state for time synchronization
 *
 * Contains all runtime parameters, state tracking, and filtering data needed
 * for maintaining synchronization with the master clock.
 */
typedef struct {
    volatile uint32_t    iepBaseAddress;     /**< Base address of IEP registers for this instance */
    volatile uint8_t     iepIncrementValue;  /**< IEP increment value for this instance */
    volatile uint64_t    currentTimestamp;   /**< Most recently captured timestamp from master */
    volatile uint64_t    prevTimestamp;      /**< Previously captured timestamp for interval calculation */
    volatile uint64_t    processingDelay;    /**< Processing delay in nanoseconds (includes propagation delay and CPU latency) */

    /* Offset Tracking */
    volatile int32_t     currOffset;         /**< Current measured offset from master time */
    volatile int32_t     prevOffset[2];      /**< Previous offset values for trend analysis [0]=last, [1]=second-last */
    volatile int32_t     prevOffsetValid[2]; /**< Validity flags for prevOffset values */
    volatile int32_t     initialOffset;      /**< Initial offset captured when system first stabilizes */

    /* Stability Indicators */
    volatile uint8_t     driftStable;        /**< Flag indicating clock drift has stabilized */
    volatile uint8_t     offsetStable;       /**< Flag indicating offset from master has stabilized */

    /* Long-term Average Tracking */
    volatile int32_t     ltaOffset;          /**< Long-term average offset using exponential filter */
    volatile int32_t     ltaOffsetValid;     /**< Indicates if ltaOffset contains valid data */

    /* Adjustment Parameters */
    volatile int32_t     adjOffset;          /**< Currently calculated adjustment to apply */
    volatile uint32_t    clockDrift;         /**< Measured clock drift rate between master and slave */

    /* State Machine */
    volatile uint32_t    state;              /**< Current state in synchronization state machine */
    volatile uint32_t    resetCount;         /**< Number of times sync has been reset due to large offset */
    volatile uint32_t    timeOutErrorCount;  /**< Number of times iep_latch0 has missed */

#ifdef ENABLE_DEBUG_LOGS
    TimesyncDebug *timesyncDebugPtr;/**< Pointer to debug structure when debugging enabled */
#endif

    /* Below are used by stable algorithm */

    /**< Index of the last sync cycle that had drift below threshold.
     *   Increments each sync frame and resets to 0 when it crosses OFFSET_ALGO_CLUSTER_SIZE. */
    volatile uint8_t lastSeen_good_drift_index;

    /**< Counter tracking the number of correction values currently stored in the correction array */
    volatile uint8_t num_entries_index;

    /**< Array of correction offset values with low drift that are clustered together.
     *   Averaged when OFFSET_ALGO_BIN_SIZE entries are collected. */
    volatile int32_t correction[OFFSET_ALGO_BIN_SIZE];
} TimesyncParams;

/**
 * \brief TimesyncHandle type definition
 *
 * Opaque pointer type for accessing TimesyncParams structure.
 * Used to maintain encapsulation of internal parameters.
 */
typedef TimesyncParams *TimesyncHandle;

/**
 * \brief Initializes time synchronization parameters and hardware
 *
 * \param params Pointer to TimesyncParams structure to initialize
 * \param iepBaseAddress Base address of the IEP module
 *
 * \return Handle to the initialized timesync instance
 */
TimesyncHandle timesync_init(TimesyncParams *params, uint32_t iepBaseAddress);

/**
 * \brief Main time synchronization state machine execution
 *
 *        Performs one iteration of the synchronization process:
 *        - Captures new timestamp
 *        - Calculates offset from expected time
 *        - Updates filtering and statistics
 *        - Applies compensation
 *        - Manages state transitions
 *
 * \param handle Handle to timesync instance
 */
void timesync_run(TimesyncHandle handle);

/**
 * \brief Reads the current IEP latch0 input value
 *
 * \param iepBaseAddress Base address of the IEP module
 * \return 64-bit timestamp value from latch register
 */
volatile uint64_t timesync_read_latch_input(uint32_t iepBaseAddress);

/**
 * \brief Reads the current IEP counter value
 *
 * \param iepBaseAddress Base address of the IEP module
 * \return 64-bit current counter value
 *
 * Uses a double-read technique to safely read the 64-bit counter over a 32-bit bus:
 * reads high word, then low word, then high word again. If the high word changed
 * between the two reads (indicating a carry from low to high occurred mid-read),
 * both words are re-read to obtain a consistent 64-bit value.
 */
volatile uint64_t timesync_read_iep_count(uint32_t iepBaseAddress);

/**
 * \brief Resets the time synchronization mechanism to initial state
 *
 * \param handle Handle to timesync instance
 */
void timesync_reset(TimesyncHandle handle);

/**
 * \brief Enables the latch0 capture mechanism for timestamp capture
 *
 * \param iepBaseAddress Base address of the IEP module
 */
void timesync_enable_latch(uint32_t iepBaseAddress);

/**
 * \brief Waits for an IEP latch0 event to occur
 *
 * \param iepBaseAddress Base address of the IEP module
 * \param sleepTime Time to sleep between polling attempts in microseconds (0 for busy-wait)
 *
 * \return 0 if latch event detected, 1 if timeout
 */
uint8_t timesync_wait_iep_latch0_event(uint32_t iepBaseAddress, uint32_t sleepTime);

/**
 * \brief Applies slow compensation adjustment to synchronize clocks
 *
 *        Updates IEP increment value and compensation period based on
 *        measured offset to gradually synchronize clocks.
 *
 * \param handle Handle to timesync instance
 * \param adjOffset Calculated adjustment offset to apply
 *
 */
void timesync_adjust_slow_compensation(TimesyncHandle handle, int32_t adjOffset);

/**
 * \brief Sets the IEP counter to align the local clock after the first sync event
 *
 * \param iepBaseAddress    Base address of the IEP module
 * \param initialCount      Initial counter value to set
 */
void timesync_do_first_adjustment(uint32_t iepBaseAddress, uint64_t initialCount);

/**
 * \brief Resets the time synchronization debug structure to initial state
 *
 * \param timesyncDebugPtr Pointer to the timesync debug structure
 */
void timesync_debug_reset(TimesyncDebug *timesyncDebugPtr);
