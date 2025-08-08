# Current Sense {#CURRENT_SENSE}

[TOC]

Current sensing is handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low-level firmware. The PRU-ICSS frees up the main Arm®-based cores in the device for other functions, such as control and data processing.

## SDFM {#SDFM}

ICSS %SDFM is a sigma delta interface for phase current measurement in high-performance motor and servo drives. During Sigma delta decimation filtering (SDDF), the PRU hardware provides hardware integrators that do the accumulation part of Sinc filtering, while the ICSS %SDFM firmware does the differentiation part.

## Features Supported
 - 3 %SDFM channels on single PRU core
 - Normal Current (NC) for data read: SINC3 filter with Over-sampling Ratio (OSR) 16 to 256
 - Over-current (OC) for comparator: free-running SINC3 filter with OSR 16 to 256
 - Event generation (Interrupt to Arm-based cores for data read from DMEM, GPIO toggle for high and low thresholds)
 - Single-level high and low threshold comparators
 - Trigger-based normal current sampling
 - Continuous normal current sampling
 - Double update: Double normal current sampling per EPWM cycle
 - %SDFM Sync with EPWM
 - Fast detect
 - PWM Trip generation for overcurrent
 - Clock phase compensation
 - Zero cross comparator

## Features Not Supported
- Multi-level threshold

## System Design Considerations

### Over Sample Ratio

- OSR Below 16 at SD clock greater than 20MHz. The normal current task takes 300ns to 400ns to complete and its execution is based on compare event and task manager. When OSR below 16 is configured for SD clock greater than 20 MHz, the normal current task will not be able to complete its processing until the next sample is ready, which will cause the normal current samples to be inaccurate.

### PWM TripZone (TZ) Block Inputs and Outputs
 - Fixed mapping between the fast detect errors and PWM TZ blocks
   - Axis 1 (Channel0 - Channel2) mapped with PWM0
   - Axis 2 (Channel3 - Channel5) mapped with PWM1
   - Axis 3 (Channel6 - Channel8) mapped with PWM2
 - PWM1 and PWM2 TZ output pins are only available on LP-AM243 in servo BP signal mode

### SDFM Data Pin Conflicts on LP-AM243 Board
- In default signal mode, all 9 SD data pins are available on LP jumpers. But in servo BP signal mode, the routing for 4 SD data pins (SD4_D, SD5_D, SD6_D and SD7_D) is changing. Out of the 4 pins, two pins are available on board jumpers (sd4_d and sd5_d) and two are not available (sd6_d and sd7_d). For more details on pinmux with LP, please see <a href="https://www.ti.com/lit/ug/spruj12e/spruj12e.pdf" target="_blank">AM243x LaunchPad Development Kit User's Guide</a>

\image html SDFM_PIN_CONFLICT.png "PIN routing for SD channels"

## Datasheet
### Fast Detect Latency
 - Fast Detect block starts comparison after the first 32 sampling clock cycles
 - Latency measured for the 20MHz sigma delta clock is 1.632us

\image html SDFM_FD_Latency.png "FD latency"

### Task duration for Normal Current at 300MHz PRU Core Clock
Normal current processing time for its different execution flows
- Task duration when only single update is enabled
  - 320ns, without R5 interrupt and samples stored in TCM memory
  - 328ns, with R5 interrupt and samples stored in TCM memory
\image html SDFM_NC_Task_time_for_single_update.png "NC Task duration for single update"
- Task duration when double update is enabled
  - 320ns, without R5 interrupt and samples stored in TCM memory
  - 336ns, with R5 interrupt and samples stored in TCM memory.
  \image html SDFM_NC_Task_time_for_double_update.png "NC Task duration for double update"
- Task duration for continuous mode is 360ns
\image html SDFM_NC_Task_time_for_continuous_mode.png "NC Task duration for continuous mode"
- Worst case Normal current task duration = 360ns + 3-4 PRU cycles time (Task switch, task exit & scratch pad switch)

## ICSS SDFM Design
\subpage SDFM_DESIGN explains the design in detail.

## Example
\ref EXAMPLES_MOTORCONTROL_SDFM

## API
\ref SDFM_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.