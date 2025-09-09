# PRUICSS TIME SYNC {#EXAMPLE_PRUICSS_TIME_SYNC}

[TOC]

## Introduction

The Time Synchronization System provides precise clock synchronization between multiple devices using Texas Instruments' PRU-ICSS (Programmable Real-time Unit Industrial Communication Subsystem) hardware. The system implements a trasmitter-receiver synchronization protocol where one device acts as a time trasmitter generating periodic sync signals, while other devices synchronize their local clocks to match the trasmitter's timing.

### System Configurations

#### Time Transmitter_Receiver Configuration
- Dual ICSS instances (ICSS0 and ICSS1)
- ICSS0 generates trasmitter sync signals
- ICSS1 receives and synchronizes to external sync signals
- Suitable for devices that need to both provide and consume timing references

#### Time Receiver Configuration
- Single ICSS instance (ICSS0)
- Receives and synchronizes to external sync signals only
- Optimized for receiver devices in a timing network

### Time Synchronization Principle and Algorithm

The time synchronization mechanism is based on a Transmitter-Receiver architecture:
- The Transmitter maintains a reference clock and periodically transmits synchronization messages containing timestamp information.
    - Generates periodic synchronization signal
    - The transmission occurs at precise intervals (typically 100us)
- The Receiver receives these synchronization messages, compares the Transmitter timestamp with its local time, and adjusts its clock to minimize the difference.

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

#### Synchronization Performance
- Filter Coefficient
    - Higher values (closer to 1.0) provide more stability but slower response
    - Lower values provide faster response but may be more susceptible to jitter
- Synchronization Period
    - More frequent synchronization messages improve accuracy but increase processing load
    - The default period is 1ms, which balances accuracy and overhead
- Stability Thresholds
    - The STABLE_FILTER_THRESHOLD determines when the system considers synchronization stable
    - This affects the transition to the TIMER_IN_SYNC state
- Processing Delays
    - Accounts for hardware and software processing latencies
    - Different values for transmitter and receiver configurations
    - Compensates for deterministic timing delays

#### Error Handling
- Threshold Monitoring
    - If the offset exceeds OFFSET_THRESHOLD_FOR_RESET, the system reverts to RESET state
    - This handles cases where synchronization is lost or severely degraded
- Timeout Detection
    - If synchronization messages are not received within an expected timeframe, the system can detect this condition
    - The system can continue operating with the last known drift compensation
- Gradual Recovery
    - After detecting synchronization errors, the system implements a gradual recovery process

For more details refer below:
- section 6.4.13 of Technical Reference Manual
- <sdk-install-dir/examples/time_sync/time_sync_main.c>
- <sdk-install-dir/examples/time_sync/time_sync.c>
- <sdk-install-dir/examples/time_sync/time_sync.h>

### Hardware connections
#### TMDS243EVM
<a href="https://www.ti.com/tool/TMDS64DC01EVM" target="_blank"> An IO Breakout Board </a> is required to probe the PWM outputs
- PRG0_IEP0_EDC_SYNC_OUT0 can be probed on J4.3
- PRG0_IEP0_EDC_LATCH_IN0 can be probed on J4.1
- connect PRG0_IEP0_EDC_SYNC_OUT0 of time_sync_time_transmitter_receiver(Borad1) to PRG0_IEP0_EDC_LATCH_IN0 of time_sync_time_receiver(Board2)

#### Probe Output running time_sync_time_transmitter_receiver on Board1 and time_sync_time_receiver on Board2

\imageStyle{time_sync_transmitter_and_receiver.png, width:90%}
\image html time_sync_transmitter_and_receiver.png "Delta of 20ns can be observed from DEVICE1 and DEVICE2 sync output signals"

### Supported Combinations

\cond SOC_AM243X

 Parameter            | Value
 -------------------- |-----------
 CPU + OS             | r5fss0-0 freertos
 ^                    | r5fss0-0 nortos
 Toolchain            | ti-arm-clang
 Boards               | @VAR_BOARD_NAME_LOWER
 R5F project (Board1) | examples/time_sync/time_sync_time_receiver
 R5F project (Board1) | examples/time_sync/time_sync_time_transmitter_receiver
 PRU Project folder   | source/pruicss_iep_sync_out_generation

\endcond

### Steps to Run the Example

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for R5F and PRU. After this, `main.asm`, `linker.cmd` files get copied to ccs workspace of PRU project. The `main.asm` contains code to configure compare1 and generate sync out

- Build the PRU project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Build Flow: Once you click on build in PRU project, firmware header file which is generated in release or debug folder of ccs workspace, is moved to  `<sdk-install-dir/source/pruicss_iep_sync_out_generation/firmware/device/>`

- Build the R5F project using the CCS project menu (see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html" target="_blank"> Using SDK with CCS Projects </a>).
     - Firmware header file path is included in R5F project include options by default. Instructions in Firmware header file can be written into PRU IRAM memory using PRUICSS_loadFirmware API call
     - Build Flow: Once you click on build in R5F project, SysConfig files are generated. Finally the R5F project will be generated using both the generated SysConfig and PRU project binaries.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- **When using makefiles to build**, note the required combination and build PRU project then R5F project using make command (see <a href="@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html" target="_blank"> Using SDK with Makefiles </a>)

- Launch a CCS debug session and run the executable, see <a href="@VAR_MCU_SDK_DOCS_PATH/CCS_LAUNCH_PAGE.html" target="_blank">  CCS Launch, Load and Run </a>

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.

