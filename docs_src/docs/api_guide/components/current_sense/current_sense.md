# Current Sense {#CURRENT_SENSE}

[TOC]

Current sensing is handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low-level firmware. The PRU-ICSS frees up the main Arm®-based cores in the device for other functions, such as control and data processing.

## SDFM {#SDFM}

ICSS %SDFM is a sigma delta interface for phase current measurement in high-performance motor and servo drives. During Sigma delta decimation filtering (SDDF), the PRU hardware provides hardware integrators that do the accumulation part of Sinc filtering, while the ICSS %SDFM firmware does the differentiation part.

\note This implementation using SD input/output mode of PRU-ICSS. Refer \ref PRUICSS_SD_MODE for more details.

## Features Supported
- 3 %SDFM channels on a single PRU core
- Normal Current (NC) for data read: SINC3 filter with an Over-Sampling Ratio (OSR) ranging from 8 to 256
- Over-Current (OC) for comparator: free-running SINC3 filter with an OSR ranging from 8 to 256
- Event generation:
  - Interrupt to Arm-based cores for data read from DMEM
  - GPIO toggle for zero-cross detection
  - PRU-ICSS PWM trip for high and low thresholds
- Single-level high and low threshold comparators
- Trigger-based normal current sampling
- Continuous normal current sampling
- Double update: Double normal current sampling per EPWM cycle
- %SDFM synchronization with EPWM
- PRU-ICSS PWM trip generation for fast detect
- PRU-ICSS PWM trip generation for overcurrent
- Clock phase compensation
- Zero-cross comparator
- Support for up to 9 channels using load share mode (Refer \ref PRUICSSG_LOAD_SHARE_MODE for more details)
- SINC1, SINC2, and SINC3 filters
- Supported clock sources:
  - Independent clock source for each channel
  - Shared clock source for three channels
  - Common clock source for all nine channels

## Features Not Supported
- Multi-level threshold
- Other example level limitations mentioned in the \ref SDFM_EXAMPLES_DESCRIPTION section

## ICSS PRU Resource Usage

<table>
<tr>
   <th>Configuration</th>
   <th>PRU Core</th>
   <th>Memory Usage</th>
   <th>Other Module/Peripheral Usage</th>
   <th>Description</th>
</tr>
<tr>
   <td>Single axis single PRU</td>
   <td>PRUx</td>
   <td>DMEM: from offset <code>0x00</code> to <code>0x200</code> Offset <br> IMEM: <code>4.1 KB</code></td>
   <td>
    PRU-ICSS EVENT:
     - INTC event/input number <code>21</code> (<code>pr[0/1]_pru_mst_intr[5]_intr_req</code>) is used to trigger interrupt to Arm® Cortex®-R5F for Channel <code>0</code>
     - INTC event/input number <code>22</code> (<code>pr[0/1]_pru_mst_intr[6]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>1</code>
     - INTC event/input number <code>23</code> (<code>pr[0/1]_pru_mst_intr[7]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>2</code>
    PRU-ICSS PWM:
     - PWM0 TRIP ZONE to generate a trip for overcurrent and fast detection error

    PRU-ICSS IEP:
     - IEP0 CMP4 is used to trigger sampling
    PRU-ICSS Task Manager:
     - PRU T1_S1 task is used for normal current task
   </td>
   <td>
    \note Individual channel events are used for continuous mode when snoop mode is disabled,
    for other configurations, only Channel <code>0</code> event is used.
   </td>
</tr>
<tr>
   <td rowspan="3">Multi-axis with load share across <code>3</code> PRU cores</td>
   <td>PRUx</td>
   <td rowspan="3">DMEM:
    from offset <code>0x00</code> to <code>0x200</code> for PRU core,
    from offset <code>0x200</code> to <code>0x400</code> for RTU core,
    and from offset <code>0x400</code> to <code>0x600</code> for TX PRU
    <br> PRU_IMEM: <code>4.1 KB</code>
    <br> RTU_IMEM: <code>4 KB</code>
    <br> TX_IMEM: <code>4 KB</code>
   </td>
   <td rowspan="3">
    PRU-ICSS EVENT:
     - INTC event/input number <code>21</code> (<code>pr[0/1]_pru_mst_intr[5]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>0</code>
     - INTC event/input number <code>22</code> (<code>pr[0/1]_pru_mst_intr[6]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>1</code>
     - INTC event/input number <code>23</code> (<code>pr[0/1]_pru_mst_intr[7]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>2</code>
     - INTC event/input number <code>24</code> (<code>pr[0/1]_pru_mst_intr[8]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>3</code>
     - INTC event/input number <code>25</code> (<code>pr[0/1]_pru_mst_intr[9]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>4</code>
     - INTC event/input number <code>26</code> (<code>pr[0/1]_pru_mst_intr[10]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>5</code>
     - INTC event/input number <code>27</code> (<code>pr[0/1]_pru_mst_intr[11]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>6</code>
     - INTC event/input number <code>28</code> (<code>pr[0/1]_pru_mst_intr[12]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>7</code>
     - INTC event/input number <code>29</code> (<code>pr[0/1]_pru_mst_intr[13]_intr_req</code>) is used to trigger interrupt to R5F for Channel <code>8</code>

    PRU-ICSS PWM:
     - PWM0 TRIP ZONE to generate a trip for overcurrent and fast detection error
     - PWM1 TRIP ZONE to generate a trip for overcurrent and fast detection error
     - PWM2 TRIP ZONE to generate a trip for overcurrent

    PRU-ICSS IEP:
     - IEP0 CMP4 is used to trigger sampling for PRU core channels
     - IEP0 CMP7 is used to trigger sampling for RTU core channels
     - IEP0 CMP8 is used to trigger sampling for TX PRU core channels

    PRU-ICSS Task Manager:
     - Each PRU core T1_S1 task is used for normal current task
   </td>
   <td rowspan="3">
    \note Individual channel events are used for continuous mode when snoop mode is disabled,
    for other configurations, only the first channel event is used for each core.
   </td>
</tr>
<tr>
   <td>RTU_PRUx</td>
</tr>
<tr>
   <td>TX_PRUx</td>
</tr>
## System Design Considerations

### Over Sample Ratio

- OSR Below 16 at SD clock greater than 20MHz. The normal current task takes 300ns to 400ns to complete and its execution is based on compare event and task manager. When OSR below 16 is configured for SD clock greater than 20 MHz, the normal current task will not be able to complete its processing until the next sample is ready, which will cause the normal current samples to be inaccurate.

### PRU-ICSS PWM TripZone (TZ) Block Inputs and Outputs
 - Fixed mapping between the fast detect errors and PWM TZ blocks
   - Axis 1 (Channel0 - Channel2) mapped with PWM0
   - Axis 2 (Channel3 - Channel5) mapped with PWM1
 - PWM1 and PWM2 TZ output pins are only available on LP-AM243 in servo BP signal mode

### SDFM Data Pin Conflicts on LP-AM243 Board
- In default signal mode, all 9 %SDFM data pins are available on LP jumpers. But in servo BP signal mode, the routing for 4 %SDFM data pins (SD4_D, SD5_D, SD6_D and SD7_D) is changing. Out of the 4 pins, two pins are available on board jumpers (sd4_d and sd5_d) and two are not available (sd6_d and sd7_d). For more details on pinmux with LP, please see <a href="https://www.ti.com/lit/ug/spruj12e/spruj12e.pdf" target="_blank">AM243x LaunchPad Development Kit User's Guide</a>

\image html SDFM_PIN_CONFLICT.png "PIN routing for %SDFM channels"

## Datasheet
### Fast Detect Latency
 - Fast Detect block starts comparison after the first 32 sampling clock cycles
 - Latency measured for the 20MHz sigma delta clock is 1.632us

\image html SDFM_FD_Latency.png "FD latency"

### Task duration for Normal Current at 300 MHz PRU Core Clock
Normal current processing time for its different execution flows
- Task duration when only single update is enabled
  - 320ns, without R5F interrupt and samples stored in TCM memory
  - 328ns, with R5F interrupt and samples stored in TCM memory
\image html SDFM_NC_Task_time_for_single_update.png "NC Task duration for single update"
- Task duration when double update is enabled
  - 320ns, without R5F interrupt and samples stored in TCM memory
  - 336ns, with R5F interrupt and samples stored in TCM memory.
  \image html SDFM_NC_Task_time_for_double_update.png "NC Task duration for double update"
- Task duration for continuous mode is 360ns
\image html SDFM_NC_Task_time_for_continuous_mode.png "NC Task duration for continuous mode"
- Worst case Normal current task duration = 360ns + 3-4 PRU cycles time (Task switch, task exit & scratch pad switch)

## ICSS SDFM Design
\subpage SDFM_DESIGN explains the design in detail.

## Example
\ref EXAMPLES_CURRENT_SENSE

## API
\ref SDFM_API_MODULE

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.