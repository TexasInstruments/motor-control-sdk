# %SDFM Interface Design {#SDFM_DESIGN}

[TOC]

## Introduction
This design implements Sigma delta interface on TI Sitara™ AM64x/AM243x.
ICSS %SDFM is a Sigma delta filter for phase current measurement.
Only two lines are required for each channel, differential pair each for %SDFM clock & %SDFM data.
Clock is provided by external device or internal device and data comes from sigma delta modulator in form of digital bit stream.


## System Overview



## Implementation
The Sigma delta filter is implemented on TI Sitara™ Devices.
Design is split into three parts – Sigma delta hardware support in PRU, firmware running in PRU and driver running in ARM.
Application is supposed to use the ICSS %SDFM driver APIs to leverage %SDFM functionality.
SDK example uses the %SDFM hardware capability in Slice 1 of PRU-ICSSG0.


###  Specifications
<table>
<tr>
    <th>Parameter
    <th>Default Value
	<th>Details
</tr>
<tr>
    <td>Normal current OSR
    <td>64
	<td>Tested with 16, 32, 64, 128 and 256
</tr>
<tr>
    <td>Over current OSR
    <td>16
	<td>Tested with 16, 32, 64, 128 and 256
</tr>
<tr>
    <td>Sigma Delta Modulator Clock
    <td>20 MHz
	<td> Tested with 5MHz, 10MHz and 20MHz from clock from PRU-ICSSG ECAP and 5MHz clock from SoC EPWM1
</tr>
<tr>
    <td>Simulated EPWM frequency
    <td>8 KHz
	<td>Tested up to 20KHz
</tr>
<tr>
    <td>IEP frequency
    <td>300 MHz
	<td>Tested with 200MHz, 225MHz and 300MHz
</tr>
</table>

### ICSS SDFM PRU hardware

Refer section 6.4.5.2.2.3.5 Sigma Delta (SD) Decimation Filtering in Technical Reference Manual(TRM) of AM243x for details.

### ICSS SDFM Firmware Implementation

Following section describes the firmware implementation of Sigma Delta Decimation Filter on PRU-ICSS.

#### Firmware Architecture

\image html SDFM_FIRMWARE_FLOWCHART.png "Overall Block Diagram"

- Firmware first clears the PRU registers and task manager.
- Then it waits for the ARM core to set %SDFM enable bit. After the enable bit is set, it sends an acknowledgement to ARM core.
- After this, the firmware does initialization of PRU-ICSSG's %SDFM hardware interface, task manager and IEP0.
- If threshold comparator is enabled, then a free run over current loop is setup, else it sets up an infinite waiting loop. In over current loop, the firmware reads sample data from the shadow copy register and does low and high theshold compersion with sample data, and depending on the configuration it toggles the GPIO pins.
- Time triggered normal current task is configured to be triggered based on IEP CMP4 event. When the CMP4 event hits, the task manager sets the program counter to normal current task. In normal current task, firmware reads sample data from accumulator and it checks for fourth normal current sample (for SINC3 filtering). If the current normal current sample belongs to fourth normal current sample, then it stores the same in data memory DMEM as normal current row data and trigger interrupt.
- At the end of normal current firmware task, execution flow comes into infinite waiting loop or over current loop.

##### Normal Curent (NC)
This section describes normal current implementation. Its implementation is trigger based. It starts execution when the trigger point is acquired (first time CMP4 event hits) and performs four continuous samplings to bring the accumulator and differntiator registers to stable state for the configured normal current OSR.

Initially the CMP4 register is configured with the first sample trigger start time and then until the next third continuous normal current sample it is updated with the normal current OSR sampling time. At the end of the fourth normal current sample again, it is updated with the second sample start time if double update is enabled otherwise with the first sample trigger start time.

\image html SDFM_NC_FLOW_CHART.png "Normal Current"

###### Single Update

Normal current sampling is done per EPWM cycle.
\image html SDFM_single_update.PNG "Single Update"

###### Double Update

Normal current sampling is done twice in one EPWM cycle.

\image html SDFM_Double_update.PNG "Double Update"

##### Over Current (OC)/Threshold Comparator 
This section describes the over current implementation. It performs continuous sampling (free run) and when the sample value crosses the high or low threshold, the corresponding GPIO pin goes high.

\image html SDFM_OC_Flow_Chart.png "Over current"
\image html SDFM_GPIO_toggle.png "GPIOs behaviour for High and Low threshold"

#### Sync with EPWM and trigger timing
This section describes the EPWM to %SDFM synchronization and trigger timing for each EPWM cycle. At the end of the every EPWM cycle, the EPWM generates a sync out event that resets the IEP timer.
The firmware initiates normal current sampling at the sample trigger point in each EPWM cycle. It takes four consecutive samples to bring the accumulator and differentiator registers to stable state. It takes the first sample at the trigger point and the next three samples, each after ONE_SAMPLE_TIME.
Here ONE_SAMPLE_TIME is: OSR*(1/SD_CLK)  
\image html SDFM_epwm_sync_and_trigger_timing.png "Sync with EPWM and trigger timing"

#### Fast Detect and Trip generation
The Fast Detect block is used for fast over current detection, it comparatively measures the number of zeros and ones presented in a programmable sliding window of 4 to 32 bits. It starts the comparison after the first 32 sample clocks. Based on the configured zero max/min count limits, it compares zero counter with these limits. If zero counter crosses limit then it sends a error signal to respective PWM Trip zone block.
PWM TZ block receives this error signal and generates a Trip on TZ_OUT pin.

#### AM64x/AM243x EVM Pin-Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH0
    <td>MCU_SPI0_D1/B6
	<td>Ch0 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH0
    <td>MCU_SPI1_D0/C7
	<td>Ch0 low threshold output
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH1
    <td>MCU_SPI1_CS0/A7
	<td>Ch1 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH1
    <td>MCU_SPI1_CLK/D7
	<td>Ch1 low threshold output
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH2
    <td>MCU_SPI1_D1/C8
	<td>Ch2 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH2
    <td>MCU_SPI0_CLK/E6
	<td>Ch2 Low threshold output
</tr>
<tr>
    <td>SD0_D
    <td>PIN_PRG0_PRU0_GPO1
	<td>Channel0 data input
</tr>
<tr>
    <td>SD1_D
    <td>PIN_PRG0_PRU0_GPO3
	<td>Channel1 data input
</tr>
<tr>
    <td>SD2_D
    <td>PIN_PRG0_PRU0_GPO5
	<td>Channel2 data input
</tr>
<tr>
    <td>PRG0_ECAP0_IN_APWM_OUT
    <td>PIN_PRG0_PRU1_GPO15
	<td>ECAP output frequency 
</tr>
<tr>
    <td>GPIO_MTR_1_PWM_EN
    <td>GPMC0_AD15/Y20
	<td>Enable EPWM0 on 3-axis board
</tr>
<tr>
    <td>SD8_CLK
    <td>PIN_PRG0_PRU0_GPO16
	<td>Comman %SDFM clock input pin
</tr>
</table>

\cond SOC_AM243X
#### AM243x LP Pin-Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH0
    <td>PRG1_PRU0_GPO18
	<td>(J7.64)Ch0 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH0
    <td>PRG0_PRU1_GPO11
	<td>(J7.70)Ch0 low threshold output
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH1
    <td>PRG1_PRU0_GPO17
	<td>(J7.65)Ch1 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH1
    <td>PRG1_PRU0_GPO7
	<td>(J7.66)Ch1 low threshold output
</tr>
<tr>
    <td>GPIO_HIGH_TH_CH2
    <td>PRG0_PRU1_GPO1
	<td>(J7.67)Ch2 High threshold output
</tr>
<tr>
    <td>GPIO_LOW_TH_CH2
    <td>PRG0_PRU1_GPO2
	<td>(J7.68)Ch2 Low threshold output
</tr>
<tr>
    <td>SD0_D
    <td>PIN_PRG0_PRU0_GPO1
	<td>(J4.32)Channel0 data input
</tr>
<tr>
    <td>SD1_D
    <td>PIN_PRG0_PRU0_GPO3
	<td>(J2.19)Channel1 data input
</tr>
<tr>
    <td>SD2_D
    <td>PIN_PRG0_PRU0_GPO5
	<td>(J2.13)Channel2 data input
</tr>
<tr>
    <td>PRG0_ECAP0_IN_APWM_OUT
    <td>PIN_PRG0_PRU1_GPO15
	<td>(J6.59)ECAP output frequency 
</tr>
<tr>
    <td>SD8_CLK
    <td>PIN_PRG0_PRU0_GPO16
	<td>(J1.7)Comman %SDFM clock input pin
</tr>
<tr>
    <td>PWM0_TZ_OUT
    <td>PIN_PRG0_PRU0_GPO19
	<td>(J5.45)TZ output pin for Axis-1
</tr>
</table>
\endcond