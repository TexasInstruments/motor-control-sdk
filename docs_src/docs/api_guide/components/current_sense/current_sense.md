# Current Sense {#CURRENT_SENSE}

[TOC]

Current sensing is handled by the Programmable Real-Time Unit Industrial Communication Subsystem (PRU-ICSS). The PRU-ICSS is a co-processor subsystem containing Programmable Real-Time (PRU) cores which implement the low-level firmware. The PRU-ICSS frees up the main Arm®-based cores in the device for other functions, such as control and data processing.

## SDFM {#SDFM}

ICSS %SDFM is a sigma delta interface for phase current measurement in high-performance motor and servo drives. During Sigma delta decimation filtering (SDDF), the PRU hardware provides hardware integrators that do the accumulation part of Sinc filtering, while the ICSS %SDFM firmware does the differentiation part.

\note This implementation using SD input/output mode of PRU-ICSS. Refer \ref PRUICSS_SD_MODE for more details.

## Features Supported

### Channel Capacity
- **Up to 9 %SDFM channels per PRU-ICSSG slice** (channels 0-8)
  - Single PRU core mode: All 9 channels handled by one PRU core
  - Load share mode: Channels distributed across three PRU cores (RTU-PRU, PRU, TX-PRU)
    - RTU-PRU: Channels 0-2
    - PRU: Channels 3-5
    - TX-PRU: Channels 6-8
  - Refer to \ref PRUICSSG_LOAD_SHARE_MODE for load share mode details

### Filtering and Sampling
- Normal Current (NC) for data read: SINC1, SINC2, or SINC3 filter with an Over-Sampling Ratio (OSR) ranging from 8 to 256
- Over-Current (OC) for comparator: free-running SINC1, SINC2, or SINC3 filter with an OSR ranging from 8 to 256
- Trigger-based normal current sampling synchronized with EPWM
- Continuous normal current sampling
- Double update: Two normal current samples per EPWM cycle

### Threshold Detection and Trip Generation
- Single-level high and low threshold comparators for over-current detection
- Fast detect for rapid over-current detection (4 to 28 bit sliding window), refer \ref FAST_DETECT for more detail
- PRU-ICSS PWM trip generation for over-current threshold violations
- PRU-ICSS PWM trip generation for fast detect errors
- Zero-cross comparator with GPIO output toggle

### Event Generation
- Interrupt to Arm-based cores for data read from DMEM
- GPIO toggle for zero-cross detection
- PRU-ICSS PWM trip for high and low thresholds

### Synchronization and Clocking
- %SDFM synchronization with EPWM
- Clock phase compensation
- Supported clock sources:
  - Independent clock source for each channel
  - Shared clock source for three channels
  - Common clock source for all nine channels
- Clock generation options: PRU-ICSSG eCAP, PRU-ICSSG IEP, PRU GPO1, or external source

## Features Not Supported
- Multi-level threshold comparators (only single-level high/low thresholds supported)
- Fast detect trip for channels 6-8 via PRU-ICSS PWM (hardware limitation; software workaround available as described in \ref OC_FD_TRIP)
- Other example-level limitations mentioned in the \ref SDFM_EXAMPLES_DESCRIPTION section

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
   <td>DMEM: from offset <code>0x00</code> to <code>0x200</code> Offset <br> IMEM: <code>3.1 KB</code></td>
   <td>
    PRU-ICSS EVENT:

    **Interrupt Mapping (Same for both Slice0 and Slice1):**
    <table>
    <tr><th>Channel</th><th>INTC Event (pr[0/1]_pru_mst_intr)</th><th>Usage</th></tr>
    <tr><td>0</td><td>21 (intr[5]_intr_req)</td><td>Continuous mode: Channel 0<br>Trigger mode: All channels (common interrupt)</td></tr>
    <tr><td>1</td><td>22 (intr[6]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>2</td><td>23 (intr[7]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>3</td><td>24 (intr[8]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>4</td><td>25 (intr[9]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>5</td><td>26 (intr[10]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>6</td><td>27 (intr[11]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>7</td><td>28 (intr[12]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>8</td><td>29 (intr[13]_intr_req)</td><td>Continuous mode only</td></tr>
    </table>

    PRU-ICSS PWM:
     - PWM0 TRIP ZONE to generate a trip for overcurrent and fast detection error

    PRU-ICSS IEP:
     - IEP0 CMP3 is the default event for triggering sampling (user-selectable via SysConfig)
     - Without EPWM synchronization, CMP0 is used for IEP counter reset in trigger mode
     - CMP1/CMP2 are used to generate clock from IEP SYNC0/SYNC1 if IEP is used as the clock generation source

    \note CMP0, CMP1, and CMP2 are reserved for specific purposes when those features are enabled. To avoid conflicts with the CMP event used for sampling trigger, it is recommended to select CMP events greater than 2 (CMP3 and above). 

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
    <br> PRU_IMEM: <code>3.1 KB</code>
    <br> RTU_IMEM: <code>3.1 KB</code>
    <br> TX_IMEM: <code>3.1 KB</code>
   </td>
   <td rowspan="3">
    PRU-ICSS EVENT (Same for both Slice0 and Slice1):

    **RTU-PRU Core - Channels 0-2:**
    <table>
    <tr><th>Channel</th><th>INTC Event (pr[0/1]_pru_mst_intr)</th><th>Usage</th></tr>
    <tr><td>0</td><td>21 (intr[5]_intr_req)</td><td>Continuous mode: Channel 0<br>Trigger mode: Channels 0-2 (common)</td></tr>
    <tr><td>1</td><td>22 (intr[6]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>2</td><td>23 (intr[7]_intr_req)</td><td>Continuous mode only</td></tr>
    </table>

    **PRU Core - Channels 3-5:**
    <table>
    <tr><th>Channel</th><th>INTC Event (pr[0/1]_pru_mst_intr)</th><th>Usage</th></tr>
    <tr><td>3</td><td>24 (intr[8]_intr_req)</td><td>Continuous mode: Channel 3<br>Trigger mode: Channels 3-5 (common)</td></tr>
    <tr><td>4</td><td>25 (intr[9]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>5</td><td>26 (intr[10]_intr_req)</td><td>Continuous mode only</td></tr>
    </table>

    **TX-PRU Core - Channels 6-8:**
    <table>
    <tr><th>Channel</th><th>INTC Event (pr[0/1]_pru_mst_intr)</th><th>Usage</th></tr>
    <tr><td>6</td><td>27 (intr[11]_intr_req)</td><td>Continuous mode: Channel 6<br>Trigger mode: Channels 6-8 (common)</td></tr>
    <tr><td>7</td><td>28 (intr[12]_intr_req)</td><td>Continuous mode only</td></tr>
    <tr><td>8</td><td>29 (intr[13]_intr_req)</td><td>Continuous mode only</td></tr>
    </table>

    PRU-ICSS PWM:
     - PWM0 TRIP ZONE to generate a trip for overcurrent and fast detection error
     - PWM1 TRIP ZONE to generate a trip for overcurrent and fast detection error
     - PWM2 TRIP ZONE to generate a trip for overcurrent

    PRU-ICSS IEP:
     - IEP0 CMP3 is the default event for PRU core channels (user-selectable via SysConfig)
     - IEP0 CMP4 is the default event for RTU core channels (user-selectable via SysConfig)
     - IEP0 CMP5 is the default event for TX PRU core channels (user-selectable via SysConfig)
     - Without EPWM synchronization, CMP0 is used for IEP counter reset in trigger mode
     - CMP1/CMP2 are used to generate clock from IEP SYNC0/SYNC1 if IEP is used as the clock generation source

    \note CMP0, CMP1, and CMP2 are reserved for specific purposes when those features are enabled. To avoid conflicts with the CMP event used for sampling trigger, it is recommended to select CMP events greater than 2 (CMP3 and above). 


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