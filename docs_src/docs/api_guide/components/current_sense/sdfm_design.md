# SDFM Interface Design {#SDFM_DESIGN}

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
    <th>Value
	<th>Details
</tr>
<tr>
    <td>Normal current OSR
    <td>32
	<td>Tested with 32, 64, 128 & 256
</tr>
<tr>
    <td>Over current OSR
    <td>64
	<td>Tested with 32, 64, 128 & 256
</tr>
<tr>
    <td>Sigma Delta Modulator Clock
    <td>20 MHz
	<td>
</tr>
<tr>
    <td>Simulated EPWM frequency
    <td>8 KHz
	<td>Tested up to 20KHz
</tr>
<tr>
    <td>IEP frequency
    <td>300MHz
	<td>
</tr>

</table>

### ICSS SDFM PRU hardware

Refer TRM for details

### ICSS SDFM Firmware Implementation

Following section describes the firmware implementation of Sigma delta decimation filter on PRU-ICSS.

####	Firmware Architecture
\image html SDFM_FIRMWARE_FLOWCHART.png "Overall Block Diagram"

Firmware first clear PRU registers & Task manager.
Then it waits for the R5 to set %SDFM enable bit. After the enable set then it acknowledge to R5.
After above these initial steps firmware does initialization of PRU-ICSSG's %SDFM hardware interface, task manager & IEP0 then it comes in infinite waiting loop.

When the COMP4 event hits the task manager assign OC loop to PC and Firmware starts execution of OC loop
In OC loop firmware read sample data from accumulator and it checks for NC sample if the current OC sample belongs to NC sample then it does sampling for NC.
During the NC sampling if current NC sample is closest to sample read time then it trigger R5 interrupt.
at the end of OC loop firmware exit task manager and again firmware execution flow comes into infinite waiting loop.

##### Threshold Comparator
This section describe threshold comparator implementation. When the sample value crosses the high or low thresholds, the corresponding GPIO pin goes high.
 \image html Threshold_comparator_flow.png "Threshold Comparator"



##### Sample data read jitter
Firmware trigger R5 interrupt for the NC sample closest to the sample read time in every PWM cycle.

NOTE: There is some jitter in sample read timing, Sample data can be sampled before or after the maximum half nc sample time.

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
	<td>
</tr>
<tr>
    <td>GPIO_MTR_1_PWM_EN
    <td>GPMC0_AD15/Y20
	<td>
</tr>
<tr>
    <td>SD8_CLK
    <td>PIN_PRG0_PRU0_GPO16
	<td>%SDFM clock input pin
</tr>
</table>