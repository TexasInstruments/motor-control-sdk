# BISS-C Protocol Design {#BISSC_DESIGN}

[TOC]

## Introduction

This design implements BISS-C Receiver (a.k.a subsequent electronics) using the 3 channel peripheral interface available on the TI Sitara™ AM64x/AM243x EVM. The 3 channel peripheral interface is a digital bidirectional serial interface for position encoders, also suited fo safety related applications. Only four signal lines are required, differential pair each for clock and data.
In BISS-C, clock is provided by receiver and data is provided by the encoder. Data is transmitted in synchronism with clock.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceiver at both ends.

## System Overview

Position feedback system consists of a position encoder attached to a motor, up to 100 meter of cable which provides power and serial communication and the receiver interface for position encoder.
In case of Sitara™ AM64x/AM243x processor the receiver interface for position encoder is just one function of a connected drive controller.
The AM64x/AM243x provides in addition to the resources for Industrial Ethernet and motor control application including on-chip ADCs, Delta Sigma demodulator for current measurement.
BISS-C Receiver on Sitara™AM64x/AM243x processor uses one ICSSGx Slice.
Clock, data transmit, data receive and receive enable signals from PRU1 of ICSS_G is available in AM64x/AM243x EVM.

## Implementation

The BISS-C receiver function is implemented on TI Sitara™ Devices.
Design is split into three parts – 3 channel peripheral interface support in PRU, firmware running in PRU and driver running in ARM.
Application is supposed to use the BISS-C driver APIs to leverage 3 channel peripheral interface functionality.
SDK examples used the BISS-C hardware capability in Slice 1 (either 1 core or 3 cores based on the confiuration) of PRU-ICSSG0.
Remaining PRUs in the AM64x/AM243x EVM are available for Industrial Ethernet communication and/or motor control interfaces.


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
	<td>Supports up-to 10MHz with delay compensation
</tr>
<tr>
    <td>Maximum Frequency
    <td>10 MHz
	<td>Supports up-to 100m cable.
</tr>
<tr>
    <td>Startup/Initialization Frequency
    <td>1 MHz
	<td>After power on or reset
</tr>
<tr>
    <td>Frequencies supported
    <td>Upto 10 MHz
	<td>Changeable at run-time
</tr>
<tr>
    <td>CRC
    <td>6/4 bits
	<td>Position-data/control-data verification
</tr>
<tr>
    <td>Receive oversample ratio
    <td>8 or 4
	<td>Oversample of 4 needs to be used for 10 MHz.
</tr>
</table>

### 3 Channel Peripheral Interface PRU hardware interface

Refer TRM for details

### BISS-C Firmware Implementation

Following section describes the firmware implementation of BISS-C receiver on PRU-ICSS.
Deterministic behavior of the 32 bit RISC core running upto 333MHz provides resolution on sampling external signals and generating external signals.
It makes uses of 3 channel peripheral interface support in PRU for data transmission.

There are three different variations of PRU-ICSS firmware.
1. Single Channel
2. Multi Channel with Encoders of Same Make
3. Multi Channel with Encoders of Different Make under load sharing

#### Implementation for Single Channel and Multi-channel with Encoder of Same Make
Single core of PRU-ICSSG slice is used in this configuration.

\image html biss_multichannel_same_make.png "ARM, PRU, BISS-C module Integration for 'Single Channel' or 'Multi Channel with Encoders of Same Make' configuration"

#### Implementation for Multi Channel with Encoders of Different Make
Each of PRU, TX-PRU and RTU-PRU handle one channel in this configuration Enbale load share mode in case of multi make encoders.

\image html biss_multichannel_different_make.png "PRU, BiSS-C module Integration for 'Multi Channel with Encoders of Different Make' configuration"

####	Firmware Architecture

\image html bissc_overall_firmware.png "Overall Block Diagram"

Firmware first detects and estimates the processing delay of the encoder as part of the initialization. Then it waits for the user to provide command (user after setting up the command, user sets command trigger bit), upon detecting trigger, first it checks whether the clock frequency has changed and if yes, it re-estimates the processing delay for the new clock frequency.

Then it reads the position data and check if a control communication is in process. It verifies the position data CRC by comparing it with the on-the-fly computation of CRC. In case of control communication mode, it backs up the CDS bit and transmits the CDM bit by overriding the clock pulse during the BISS-C cycle timeout phase.  If the control communication is in progress it goes back to read the position data for the next cycle. If the control communication is completed, it updates the control command status, position data status and returns to wait for the next trigger command from ARM.

\note Firmware running on PRU-ICSS will be remain HALTED if encoder is not detected and application will wait for 5 seconds and exit with error code.

#####	 Initialization
\image html biss_initialization.png "Initilization for All Modes"

Initialization is performed both on the ARM and PRU as shown in the figure above. During the initialization, based on the clock frequency selected the PRU detects the Encoder and estimates its processing delay in terms of clock cycles. The processing delay is measured 8 times and an average value is used for compensation. Note that whenever the user changes the clock frequency, the initialization routine on the PRU is executed to estimate the processing delay.

If using "Multi Channel with Encoders of Different Make" configuration where load share mode is enabled, one of the cores among enabled cores will be set as the primary core for performing global configurations of PRU-ICSSG's BISS-C interface. These global configurations include clock frequency configuration and TX global re-initialization.

There needs to be a synchronization between PRUs before changing any global configuration. For this purpose, each active PRU core sets synchronization bit before any operation needing synchronization and clears the synchronization bit when it is ready. The assigned primary core will wait for all active channel's synchronization bits to be cleared and then perform the global configuration.

#####	Receive Position Data
\image html main_bissc.png "BISS-C main loop for Position Data"

Once the firmware receives the trigger from ARM, it will first calculate the RX frame size and check if there is a change in the clock frequency from the previous run. If yes, it will recalculate the processing delay of the encoder for the given clock frequency.

Next, it will start the clock signal to wait for the acknowledgement bit followed by the start bit. If the program is in control mode, it will back up the CDS bit for subsequent processing. Next it will read the position data bits, error and warning bits while computing the CRC on the fly. Finally, it will read the 6-bit CRC. If the program is in control mode, it will overide the clock signal to transmit the CDM bit to the encoder.

The program will then wait for the timeout period and verify CRC of the position data bits. If the program is in control mode, it will check if the control command cycle is completed. If yes, it will update the results and the status bits before returning to the starting point.

#####	Control Communication
\image html bissc_read_control_communication.png "BISS-C control communication loop"

BISS-c control communication is performed over multiple cycles. Refer to the standard for more details on control communication. The firmware expects a control command as a 16-bit hex value. Once a control communication is started, the program will first transmit 14 0's as CDM bit during the timeout period at the end of the BISS-C cycle. This is then followed by a start bit to indicate to the encoder that control communication is in progress. The figure shown above explains the flow for a register read access. The register write access and control commands follows similar steps. Example commands for the register read access is given in the table below.

\image html bissc_hex_control_commands.png "BISS-C hex commands"

### 3 Channel Peripheral Interface

The physical data transmission in 3 channel peripheral interface is done using RS-485 standard. The data is transmitted as differential signals using the RS485 between the 3 channel peripheral interface Receiver and the Encoder.

The Receiver sends the clock to the BISS-C encoder, data transmission in either direction (one at a time) occurs in synchronism with the clock. The design uses two differential signals for each of the lines (clock and data).

BISS-C Receiver and the encoder is connected using the RS-485 transceiver. Data is transmitted differentially over RS-485. It has the advantages of high noise immunity and long distance transmission capabilities.

\cond SOC_AM243X
##### AM243x-LP Booster Pack Pin-Multiplexing
<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_bissc0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_bissc0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>GPIO Pin(GPIO1_22)
    <td>BISSC_CH0_OUT_EN
	<td>Channel 0 transmit disable
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_bissc0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>GPIO Pin(GPIO1_78)
    <td>ENC0_EN
    <td>Enbale 3 channel peripheral interface mode in Axis 1 of BP (C16 GPIO pin)
</tr>
</table>
\endcond
