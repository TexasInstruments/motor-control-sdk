# BISS-C Protocol Design {#BISSC_DESIGN}

[TOC]

## Introduction

This design implements BISS-C Receiver (a.k.a subsequent electronics) using the 3 channel peripheral interface available on the TI Sitara™ processors/microcontrollers. The 3 channel peripheral interface is a digital bidirectional serial interface for position encoders, also suited for safety related applications. Only four signal lines are required, differential pair each for clock and data.
In BISS-C, clock is provided by the receiver and data is provided by the encoder. Data is transmitted in synchronism with the clock.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceivers at both ends.

## System Overview

Position feedback system consists of a position encoder attached to a motor, up to 100 meters of cable which provides power and serial communication and the receiver interface for position encoder.
In the case of Sitara™ processor/microcontroller, the receiver interface for position encoder is just one function of a connected drive controller.
The Sitara™ processor/microcontroller provides, in addition to the resources for Industrial Ethernet and motor control application, including on-chip ADCs, Delta Sigma demodulator for current measurement.
BISS-C Receiver on processor/microcontroller uses one ICSSx Slice.
Clock, data transmit, data receive and receive enable signals from PRU of ICSS are available in Sitara™ processor/microcontroller.

## Implementation

The BiSS-C receiver function is implemented on TI Sitara™ processors/microcontrollers.

Design is split into three parts:
    1. BiSS-C hardware support in PRU using three channel peripheral interface
    2. Firmware running in PRU
    3. Driver running in Arm®-based core

Application is supposed to use the BiSS-C driver APIs to leverage BiSS-C functionality.

Default SDK examples three channel peripheral interface in \if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PRU0 of PRU-ICSSM \else Slice 1 (either 1 core or 3 cores based on the configuration) of PRU-ICSSG0 \endif.

###  Specifications

<table>
<tr>
    <th>Parameter
    <th>Value
	<th>Details
</tr>
<tr>
    <td>Maximum Cable Length
    <td>100m
	<td>Supports up to 10MHz with delay compensation
</tr>
<tr>
    <td>Startup/Initialization Frequency
    <td>1 MHz
	<td>After power on or reset
</tr>
<tr>
    <td>Frequencies supported
    <td>Up to 10 MHz
	<td>Changeable at run-time
</tr>
<tr>
    <td>CRC
    <td>6/16 bits
	<td>Position-data/control-data verification
</tr>
<tr>
    <td>Receive oversample ratio
    <td>1x to 8x
	<td>Tested with 4x, 6x & 8x (Frequency specific)
</tr>
</table>

### 3 Channel Peripheral Interface PRU hardware interface

Refer to TRM for details

### BISS-C Firmware Implementation

\cond SOC_AM243X

Following section describes the firmware implementation of BISS-C receiver on PRU-ICSS. Deterministic behavior of the 32 bit RISC core running up to 333 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission.

The PRU-ICSS firmware supports the following configurations:
1. Single Channel per PRU slice
2. Multi Channel with encoders of same make per PRU slice
3. Multi Channel with encoders of different make under load share mode per PRU slice

#### Implementation for Single Channel and Multi-channel with encoder of same make
Single core of PRU-ICSSG slice is used in this configuration.

\image html biss_multichannel_same_make.png "Arm-based core, PRU, BISS-C module Integration for 'Single Channel' or 'Multi Channel with encoders of same make' configuration"

#### Implementation for Multi Channel with encoders of different make
Each of PRU, TX-PRU and RTU-PRU handle one channel in this configuration. Load share mode for three channel peripheral interface is enabled.

\image html biss_multichannel_different_make.png "PRU, BiSS-C module Integration for 'Multi Channel with encoders of different make' configuration"

\endcond

\cond SOC_AM261X

Following section describes the firmware implementation of BISS-C receiver on PRU-ICSS. Deterministic behavior of the 32 bit RISC core running up to 225 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission.

The PRU-ICSS firmware supports the following configuration:
1. Single Channel per PRU slice

#### Implementation for Single Channel
Single core of PRU-ICSS slice is used in this configuration.

\image html biss_multichannel_same_make.png "Arm-based core, PRU, BISS-C module Integration for Single Channel "

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

Following section describes the firmware implementation of BISS-C receiver on PRU-ICSS. Deterministic behavior of the 32 bit RISC core running up to 200 MHz provides resolution on sampling external signals and generating external signals. It makes use of 3 channel peripheral interface support in PRU for data transmission.

The PRU-ICSS firmware supports the following configuration:
1. Single Channel per PRU slice

#### Implementation for Single Channel
Single core of PRU-ICSS slice is used in this configuration.

\image html biss_multichannel_same_make.png "Arm-based core, PRU, BISS-C module Integration for Single Channel "

\endcond

#### Firmware Architecture

\image html bissc_overall_firmware.png "Overall Block Diagram"

Firmware first detects and estimates the processing delay of the encoder as part of the initialization. Then it checks the operation mode: host trigger mode, periodic CMP mode, or periodic CAP mode.

**Host Trigger Mode:** The firmware waits until a command has been triggered through the interface by the host application.

**Periodic CMP/CAP Mode:** The firmware monitors the configured IEP compare/capture event and sets the host trigger bit when the event occurs, automatically initiating Nikon transactions at regular intervals.

Upon detecting trigger, first it checks whether the clock frequency has changed and if yes, it re-estimates the processing delay for the new clock frequency.

Then it reads the position data and checks if a control communication is in process. It verifies the position data CRC by comparing it with the on-the-fly computation of CRC. In case of control communication mode, it backs up the CDS bit and transmits the CDM bit by overriding the clock pulse during the BISS-C cycle timeout phase. If the control communication is in progress it goes back to read the position data for the next cycle. If the control communication is completed, it updates the control command status, position data status and returns to wait for the next trigger from the interface or IEP compare/capture event.

In case the "safety mode" is enabled, firmware will be executed as explained below:

\image html bissc_safety_rx_flow.png "RX flow when Safety is enabled"

Firmware will perform configuration as usual and then before entering into RX it checks whether safety is enabled for that particular encoder or not. If enabled, firmware will perform receive and downsample and CRC computation is excluded, after all the encoders data bits are read successfully, before going into timeout firmware will perform post-processing to compute the 16 bit CRC for safety enabled encoders. Please note that post-processing will be applicable only for the encoders for which safety is enabled using control communication. Please find below an image that explains CPW and SPW as per BiSS-C safety specifications.

\image html bissc_safety_frame.png "BiSS Frame according to the BiSS Safety profile"

Above image is taken from <a href="https://biss-interface.com/download/biss-safety-concept-english/" target="_blank">BiSS Safety Concept Document</a>.

\note BiSS safety is implemented by assuming 2 encoders connected in Daisy chain, one will send CPW and another one will send SPW. CRC errors are expected while enabling Safety using control communication. Because control communication will need multiple BiSS cycles to confirm and the safety may be enabled in the encoder before control communication is completed and encoder may send 16-bit CRC in some of the BiSS cycles.

\note Firmware running on PRU-ICSS will remain HALTED if encoder is not detected and application will wait for 5 seconds and exit with error code.

##### Initialization
\image html biss_initialization.png "Initialization for All Modes"

Initialization is performed both on the Arm-based core and PRU as shown in the figure above. During the initialization, based on the clock frequency selected the PRU detects the encoder and estimates its processing delay in terms of clock cycles. The processing delay is measured 8 times and an average value is used for compensation. Note that whenever the user changes the clock frequency, the initialization routine on the PRU is executed to estimate the processing delay.

If using "Multi Channel with encoders of different make" configuration where load share mode is enabled, one of the cores among enabled cores will be set as the primary core for performing global configurations of PRU-ICSS's BISS-C interface. These global configurations include clock frequency configuration and TX global re-initialization.

There needs to be a synchronization between PRUs before changing any global configuration. For this purpose, each active PRU core sets synchronization bit before any operation needing synchronization and clears the synchronization bit when it is ready. The assigned primary core will wait for all active channel's synchronization bits to be cleared and then perform the global configuration.

##### Receive Position Data
\image html main_bissc.png "BISS-C main loop for Position Data"

Once the firmware receives the trigger from Arm-based core, it will first calculate the RX frame size and check if there is a change in the clock frequency from the previous run. If yes, it will recalculate the processing delay of the encoder for the given clock frequency.

Next, it will start the clock signal to wait for the acknowledgement bit followed by the start bit. If the program is in control mode, it will back up the CDS bit for subsequent processing. Next it will read the position data bits, error and warning bits while computing the CRC on the fly. Finally, it will read the 6-bit CRC. If the program is in control mode, it will override the clock signal to transmit the CDM bit to the encoder.

The program will then wait for the timeout period and verify CRC of the position data bits. If the program is in control mode, it will check if the control command cycle is completed. If yes, it will update the results and the status bits before returning to the starting point.

##### Control Communication
\image html bissc_read_control_communication.png "BISS-C control communication loop"

BISS-C control communication is performed over multiple cycles. Refer to the standard for more details on control communication. The firmware expects a control command as a 16-bit hex value. Once a control communication is started, the program will first transmit 14 0's as CDM bit during the timeout period at the end of the BISS-C cycle. This is then followed by a start bit to indicate to the encoder that control communication is in progress. The figure shown above explains the flow for a register read access. The register write access and control commands follow similar steps. Example commands for the register read access are given in the table below.

\image html bissc_hex_control_commands.png "BISS-C hex commands"

##### Post-processing for 16-bit Safety CRC
\image html bissc_safety_postprocessing_16_bit_crc.png "BISS-C Safety post-processing for 16-bit Safety CRC"

In case safety is enabled, only receive and downsample for the RX bits will be performed first. CRC computation will be performed during timeout in the post-processing section. Processing will be done in loop for all encoders across channels. 16-bit CRC is computed using XOR based approach with the provided polynomial from the BiSS specifications and compared with the received CRC and the error statistics are updated.

###### Periodic Trigger Modes

The BiSS-C receiver supports two types of periodic trigger modes for continuous position sampling: CMP (Compare) mode and CAP (Capture) mode as described in \ref BISSC_PERIODIC_MODES.

Following is the operation flow for periodic mode:
1. Firmware polls IEP CMP/CAP status register and clears status after event is detected
2. On event detection, firmware initiates BiSS-C transaction
3. Position data is automatically updated in shared memory
4. R5F interrupt notifies application of new data

\image html bissc_periodic_mode.png "Periodic Trigger Mode"

\cond SOC_AM243X
\note In load share mode, each channel can have independent IEP CMP/CAP event configuration.
\endcond

\attention Input cycle time should be greater than or equal to the BiSS-C cycle time by considering the position data bits, E, W, CRC and timeout.

User can stop periodic mode by switching to host trigger mode.

### 3 Channel Peripheral Interface

The physical data transmission in 3 channel peripheral interface is done using RS-485 standard. The data is transmitted as differential signals using the RS485 between the 3 channel peripheral interface receiver and the encoder.

The receiver sends the clock to the BISS-C encoder, data transmission in either direction (one at a time) occurs in synchronism with the clock. The design uses two differential signals for each of the lines (clock and data).

BISS-C receiver and the encoder are connected using the RS-485 transceiver. Data is transmitted differentially over RS-485. It has the advantages of high noise immunity and long distance transmission capabilities.

#### Pin Multiplexing {#BISSC_PIN_USAGE}

\attention \ref PRUICSS_PERIPHERAL_IF_MODE_SIGNAL_CONFIGURATION section has details on PRU pin functions in Peripheral IF mode

\note
    - k = 0,1 (PRU-ICSS Instance) for AM243x/AM261x/AM64x and k = 0 for AM263x/AM263Px
    - n = 0,1 (PRU-ICSS Slice)

<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO0 \else PRG<%k>_PRU<n>_GPO0 \endif
    <td>pru<n>_bissc0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO2 \else PRG<%k>_PRU<n>_GPO2 \endif
    <td>pru<n>_bissc0_outen
	<td>Channel 0 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI9 \else PRG<%k>_PRU<n>_GPI13/PRG<%k>_PRU<n>_GPI9 \endif
    <td>pru<n>_bissc0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO3 \else PRG<%k>_PRU<n>_GPO3 \endif
    <td>pru<n>_bissc1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO5 \else PRG<%k>_PRU<n>_GPO5 \endif
    <td>pru<n>_bissc1_outen
	<td>Channel 1 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI10 \else PRG<%k>_PRU<n>_GPI14/PRG<%k>_PRU<n>_GPI10 \endif
    <td>pru<n>_bissc1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO6 \else PRG<%k>_PRU<n>_GPO6 \endif
    <td>pru<n>_bissc2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPO8 \else PRG<%k>_PRU<n>_GPO8 \endif
    <td>pru<n>_bissc2_outen
	<td>Channel 2 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>\if (SOC_AM263X || SOC_AM263PX || SOC_AM261X) PR<%k>_PRU<n>_GPI11 \else PRG<%k>_PRU<n>_GPI11 \endif
    <td>pru<n>_bissc2_in
	<td>Channel 2 receive
</tr>
</table>

\cond SOC_AM243X
##### LP-AM243 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_bissc0_clk
	<td>PRU1 Channel 0 clock
</tr>
<tr>
    <td>GPIO Pin (PRG0_PRU1_GPO2/M2)
    <td>BISSC_CH0_OUT_EN
	<td>PRU1 Channel 0 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_bissc0_in
	<td>PRU1 Channel 0 receive when SA mux selection is enabled (ICSSG_SA_MX_REG[7] G_MUX_EN = 1)
</tr>
<tr>
    <td>PRG0_PRU1_GPO6
    <td>pru1_bissc2_clk
	<td>PRU1 Channel 2 clock
</tr>
<tr>
    <td>GPIO Pin (PRG0_PRU1_GPO8/F4)
    <td>BISSC_CH2_OUT_EN
	<td>PRU1 Channel 2 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_bissc2_in
	<td>PRU1 Channel 2 receive
</tr>
<tr>
    <td>GPIO Pin (MMC1_SDWP/C16)
    <td>ENC0_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
    <td>GPIO Pin (MMC1_SDCD/B17)
    <td>ENC2_EN
    <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>

\endcond

\cond SOC_AM261X
##### LP-AM261 + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example

<table>
<tr>
    <th>Pin name
    <th>Signal name
    <th>Function
</tr>
<tr>
    <td>PR1_PRU0_GPIO0
    <td>pru0_bissc0_clk
    <td>PRU0 Channel 0 clock
</tr>
<tr>
    <td>GPIO Pin (GPIO_83/B4)
    <td>BISSC_CH0_OUT_EN
    <td>PRU0 Channel 0 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>PR1_PRU0_GPI9
    <td>pru0_bissc0_in
    <td>PRU0 Channel 0 receive
</tr>
<tr>
    <td>PR1_PRU1_GPIO0
    <td>pru1_bissc0_clk
    <td>PRU1 Channel 0 clock
</tr>
<tr>
    <td>GPIO Pin (GPIO_73/W17)
    <td>BISSC_CH1_OUT_EN
    <td>PRU1 Channel 0 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>PR1_PRU1_GPI9
    <td>pru1_bissc0_in
    <td>PRU1 Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin (GPIO_21/B10)
    <td>ENC0_EN (PRU0)
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
<tr>
    <td>GPIO Pin (GPIO_22/A10)
    <td>ENC0_EN (PRU1)
    <td>Enable encoder voltage in Axis 2 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>

\endcond

\cond (SOC_AM263X || SOC_AM263PX)

##### @VAR_LP_BOARD_NAME + BP-AM2BLDCSERVO Booster Pack Pin Multiplexing for SDK example
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PR0_PRU0_GPIO3
    <td>pru0_bissc1_clk
	<td>PRU0 Channel 1 clock
</tr>
<tr>
    <td>GPIO Pin (PR0_PRU0_GPIO5)
    <td>BISSC_CH0_OUT_EN
    <td>PRU0 Channel 1 transmit enable (Fix this pin to low with SoC GPIO mode)
</tr>
<tr>
    <td>PR0_PRU0_GPI10
    <td>pru0_bissc1_in
	<td>PRU0 Channel 1 receive
</tr>
<tr>
    <td>GPIO Pin (SDFM0_D1/D13)
    <td>ENC1_EN
    <td>Enable encoder voltage in Axis 1 of BP (Fix this pin to high with SoC GPIO mode)
</tr>
</table>
\endcond

\note Arm is a registered trademark of Arm Limited (or its subsidiaries or affiliates) in the US and/or elsewhere.
