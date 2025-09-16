# PRU-ICSS TIME SYNC DESIGN {#TIME_SYNC_DESIGN}

### Time Synchronization Principle and Algorithm

The time synchronization mechanism is based on a Transmitter-Receiver architecture:
- The Transmitter maintains a reference clock and periodically transmits synchronization messages containing timestamp information.
    - Generates periodic synchronization signal
    - The transmission occurs at precise intervals (typically 100us)
- The Receiver receives these synchronization messages, compares the Transmitter timestamp with its local time, and adjusts its clock to minimize the difference.

Note : The synchronization state machine must be executed once within each sync cycle (100us) on the R5F processor

#### Synchronization States

The synchronization process operates through four distinct states:

- RESET: Initial state or entered after a synchronization error
- FIRST_ADJUSTMENT_DONE: After the first time offset correction
- SLOW_COMPENSATION: Gradual adjustment to compensate for clock drift
- TIMER_IN_SYNC: Stable synchronized state with minimal adjustments

#### Synchronization Process
- Offset Measurement:
    - The receiver captures the transmitter's timestamp and compares it with its local time
    - The difference represents the current time offset
- Filtering:
    - An exponential moving average filter is applied to the raw offset measurements
    - This reduces the impact of jitter and communication delays
- Drift Estimation:
    - The rate at which the offset changes over time indicates clock drift
    - A drift coefficient is calculated and used to predict future offset
- Clock Adjustment:
    - In the FIRST_ADJUSTMENT_DONE state, a large correction is made to quickly synchronize
    - In the SLOW_COMPENSATION state, gradual adjustments compensate for drift
    - In the TIMER_IN_SYNC state, minor corrections maintain synchronization
- Stability Monitoring:
    - The system continuously monitors the variance in offset and drift
    - When these values remain within thresholds, the system is considered stable

\imageStyle{time_sync_flow_chart.jpg, width:45%}
\image html time_sync_flow_chart.jpg "High level time sync design"

#### Synchronization Performance
- Filter Coefficient
    - Higher values (closer to 1.0) provide more stability but slower response
    - Lower values provide faster response but may be more susceptible to jitter
- Synchronization Period
    - More frequent synchronization messages improve accuracy but increase processing load
    - The default period is 100us, which balances accuracy and overhead
- Stability Thresholds
    - The STABLE_FILTER_THRESHOLD determines when the system considers synchronization stable
    - This affects the transition to the TIMER_IN_SYNC state

##### Processing delay
- Accounts for hardware and software processing latencies
- Different values for transmitter and receiver configurations
- Compensates for deterministic timing delays
- Below are the default processing delay values
    \code
        /* Macros for time synchronization processing delays which add's propagation delay and cpu latency*/
        /* Processing delays in nanoseconds */
        #define TIMESYNC_PROCESSING_DELAY_TRANSMITTER_RECEIVER  (400U)
        #define TIMESYNC_PROCESSING_DELAY_RECEIVER              (750U)
    \endcode

##### Follow below steps to calculate processing delay
- Set both delay values to zero in <sdk-install-dir/examples/time_sync/time_sync_main.c>:
    \code
        #define TIMESYNC_PROCESSING_DELAY_TRANSMITTER_RECEIVER  (0U)
        #define TIMESYNC_PROCESSING_DELAY_RECEIVER              (0U)
    \endcode
- Measure Baseline Time Difference:
    - Connect an oscilloscope to monitor the sync output signals from both devices
    - Probe PRG0_IEP0_EDC_SYNC_OUT0 on both the transmitter and receiver boards
    - Measure the time difference (delta) between the two signals
    - This delta represents the actual system delay that needs to be compensated

#### Error Handling
- Threshold Monitoring
    - If the offset exceeds OFFSET_THRESHOLD_FOR_RESET, the system reverts to RESET state
    - This handles cases where synchronization is lost or severely degraded
- Timeout Detection
    - If synchronization messages are not received within an expected timeframe, the system can detect this condition
    - The system can continue operating with the last known drift compensation
- Gradual Recovery
    - After detecting synchronization errors, the system implements a gradual recovery process